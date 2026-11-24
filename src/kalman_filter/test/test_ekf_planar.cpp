// Tests for EkfPlanar.
//   1. Jacobians (slopes) match numerical differentiation.
//   2. Sonar geometry gives the expected ranges for the real sonar layout.
//   3. A simulated zig-zag run (start x=0.5, y=5, heading 45 deg from pool x)
//      keeps wall distance and heading accurate, and the along-wall position
//      drifts less than 15 cm.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>

#include "ekf_planar/ekf_planar.h"

namespace
{
constexpr double kDeg = M_PI / 180.0;

// Real layout: 0.30 x 0.30 m base_link, sonars 0.30 m below (ignored in the
// plane). Corner sonars point outwards at +-45 deg / +-135 deg. The
// downward sonar is not part of the planar filter.
enum SonarId { FRONT = 0, BACK, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT };

EkfPlanar::Params pool_params()
{
    EkfPlanar::Params p;
    const double h = 0.15;
    p.sonars = {
        { h, 0.0, 0.0},            // front
        {-h, 0.0, M_PI},           // back
        { h, h, 45 * kDeg},        // front-left
        { h, -h, -45 * kDeg},      // front-right
        {-h, h, 135 * kDeg},       // back-left
        {-h, -h, -135 * kDeg},     // back-right
    };
    // 15 m wide (x), 25 m long (y)
    p.walls = {
        { 1, 0, 0},      // left wall,  x = 0
        {-1, 0, -15},    // right wall, x = 15
        { 0, 1, 0},      // near end,   y = 0
        { 0, -1, -25},   // far end,    y = 25
    };
    return p;
}

EkfPlanar::State make_state(double px, double py, double psi,
                            double vx = 0, double vy = 0, double bg = 0)
{
    EkfPlanar::State x;
    x << px, py, psi, vx, vy, bg;
    return x;
}

// ------------------------------------------------------------ 1. Jacobians

TEST(EkfPlanarJacobian, MotionModel)
{
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    const double eps = 1e-6;

    for (int trial = 0; trial < 200; ++trial) {
        const EkfPlanar::State x = make_state(2 + U(rng), 10 + U(rng), 2.5 * U(rng),
                                              0.3 * U(rng), 0.3 * U(rng), 0.01 * U(rng));
        const Eigen::Vector2d a(0.5 * U(rng), 0.5 * U(rng));
        const double gyro = 0.5 * U(rng);
        const double dt = trial % 2 ? 0.01 : 0.5;  // large dt makes errors visible

        const EkfPlanar::Cov F = EkfPlanar::F_jacobian(x, a, dt);
        for (int j = 0; j < EkfPlanar::N; ++j) {
            EkfPlanar::State xp = x, xm = x;
            xp(j) += eps;
            xm(j) -= eps;
            EkfPlanar::State col = (EkfPlanar::f(xp, a, gyro, dt) - EkfPlanar::f(xm, a, gyro, dt)) / (2 * eps);
            col(EkfPlanar::PSI) = EkfPlanar::wrap_angle(
                EkfPlanar::f(xp, a, gyro, dt)(EkfPlanar::PSI) - EkfPlanar::f(xm, a, gyro, dt)(EkfPlanar::PSI)) / (2 * eps);
            for (int i = 0; i < EkfPlanar::N; ++i)
                ASSERT_NEAR(F(i, j), col(i), 1e-6) << "F(" << i << "," << j << ") trial " << trial;
        }
    }
}

// Checks H against central differences wherever the model is smooth.
// Returns how many (state, sonar) cases were checked.
int check_sonar_jacobian(const EkfPlanar& ekf, std::mt19937& rng)
{
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    const double eps = 1e-6;
    const EkfPlanar::Wall left{1, 0, 0};
    int checked = 0;

    for (int trial = 0; trial < 2000; ++trial) {
        const EkfPlanar::State x = make_state(1.2 + 0.9 * U(rng), 10 + 5 * U(rng), M_PI * U(rng),
                                              0.2 * U(rng), 0.2 * U(rng), 0.01 * U(rng));
        for (int s = 0; s < static_cast<int>(ekf.params().sonars.size()); ++s) {
            double h0;
            EkfPlanar::Row H;
            if (ekf.sonar_wall_model(x, s, left, h0, &H) != EkfPlanar::SonarResult::Used) continue;

            bool smooth = true;
            EkfPlanar::Row num;
            for (int j = 0; j < EkfPlanar::N && smooth; ++j) {
                EkfPlanar::State xp = x, xm = x;
                xp(j) += eps;
                xm(j) -= eps;
                double hp, hm;
                if (ekf.sonar_wall_model(xp, s, left, hp) != EkfPlanar::SonarResult::Used ||
                    ekf.sonar_wall_model(xm, s, left, hm) != EkfPlanar::SonarResult::Used) {
                    smooth = false;
                    break;
                }
                // skip kinks (e.g. exactly at the beam-width switch)
                if (std::abs((hp - h0) - (h0 - hm)) / eps > 1e-4) smooth = false;
                num(j) = (hp - hm) / (2 * eps);
            }
            if (!smooth) continue;
            for (int j = 0; j < EkfPlanar::N; ++j)
                EXPECT_NEAR(H(j), num(j), 1e-5) << "sonar " << s << " col " << j;
            ++checked;
        }
    }
    return checked;
}

TEST(EkfPlanarJacobian, SonarThinBeam)
{
    std::mt19937 rng(2);
    EkfPlanar ekf(pool_params());
    EXPECT_GT(check_sonar_jacobian(ekf, rng), 500);
}

TEST(EkfPlanarJacobian, SonarWideBeam)
{
    std::mt19937 rng(3);
    EkfPlanar::Params p = pool_params();
    p.beam_half_angle = 12.5 * kDeg;  // both branches of the beam model get exercised
    EkfPlanar ekf(p);
    EXPECT_GT(check_sonar_jacobian(ekf, rng), 500);
}

TEST(EkfPlanarJacobian, Sideslip)
{
    std::mt19937 rng(4);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    const double eps = 1e-6;
    for (int trial = 0; trial < 200; ++trial) {
        const EkfPlanar::State x = make_state(U(rng), U(rng), M_PI * U(rng), U(rng), U(rng), U(rng));
        EkfPlanar::Row H;
        EkfPlanar::sideslip_model(x, &H);
        for (int j = 0; j < EkfPlanar::N; ++j) {
            EkfPlanar::State xp = x, xm = x;
            xp(j) += eps;
            xm(j) -= eps;
            const double num = (EkfPlanar::sideslip_model(xp) - EkfPlanar::sideslip_model(xm)) / (2 * eps);
            ASSERT_NEAR(H(j), num, 1e-7);
        }
    }
}

// ------------------------------------------------------------ 2. Geometry

TEST(EkfPlanarGeometry, RealLayoutOnAwayLeg)
{
    // Robot 1 m from the left wall, heading 45 deg (away from the wall, along +y).
    EkfPlanar ekf(pool_params());
    const EkfPlanar::State x = make_state(1.0, 5.0, 45 * kDeg);
    double h;

    // back-left corner points straight at the wall; it sits 0.15*sqrt(2) closer
    ASSERT_EQ(ekf.sonar_model(x, BACK_LEFT, h), EkfPlanar::SonarResult::Used);
    EXPECT_NEAR(h, 1.0 - 0.15 * std::sqrt(2.0), 1e-9);

    // back sonar hits the wall at 45 deg: (1 - 0.15/sqrt2) / cos 45
    ASSERT_EQ(ekf.sonar_model(x, BACK, h), EkfPlanar::SonarResult::Used);
    EXPECT_NEAR(h, (1.0 - 0.15 / std::sqrt(2.0)) * std::sqrt(2.0), 1e-9);

    // the rest point along the pool or away from the wall: nothing within 2.5 m
    for (int s : {FRONT, FRONT_LEFT, FRONT_RIGHT, BACK_RIGHT})
        EXPECT_EQ(ekf.sonar_model(x, s, h), EkfPlanar::SonarResult::OutOfRange) << "sonar " << s;
}

TEST(EkfPlanarGeometry, LeftWallOnlyAndIncidenceLimit)
{
    EkfPlanar::Params p = pool_params();
    p.walls = {{1, 0, 0}};
    p.max_incidence = 40 * kDeg;
    EkfPlanar ekf(p);
    const EkfPlanar::State x = make_state(1.0, 5.0, 45 * kDeg);
    double h;
    EXPECT_EQ(ekf.sonar_model(x, BACK_LEFT, h), EkfPlanar::SonarResult::Used);
    EXPECT_EQ(ekf.sonar_model(x, BACK, h), EkfPlanar::SonarResult::TooOblique);   // 45 > 40 deg
    EXPECT_EQ(ekf.sonar_model(x, FRONT_LEFT, h), EkfPlanar::SonarResult::NoWall);  // parallel to wall
    EXPECT_EQ(ekf.sonar_model(x, FRONT, h), EkfPlanar::SonarResult::NoWall);       // facing away
}

TEST(EkfPlanarGeometry, ConeEdgeClipsSideWallFirst)
{
    // 1 m from the left wall, heading 45 deg. The front-left sonar points
    // straight down the pool (+y), parallel to the left wall. With a 10 deg
    // half cone, its edge clips the left wall at 1/sin(10 deg) = 5.8 m, before
    // the centre line reaches the far end at ~7.8 m. That echo is far too
    // oblique to model, so the reading must be rejected, not predicted as 7.8 m.
    EkfPlanar::Params p = pool_params();
    p.walls = {{1, 0, 0}, {-1, 0, -5}, {0, 1, 0}, {0, -1, -10}};
    p.max_range = 30.0;
    p.beam_half_angle = 10 * kDeg;
    EkfPlanar ekf(p);
    double h;
    EXPECT_NE(ekf.sonar_model(make_state(1.0, 2.0, 45 * kDeg), FRONT_LEFT, h), EkfPlanar::SonarResult::Used);

    // Far enough from the left wall, the far end is nearer: used, head on.
    // Sonar is at y = 2 + 0.15*sqrt(2), so the range is 8 - 0.15*sqrt(2).
    ASSERT_EQ(ekf.sonar_model(make_state(2.0, 2.0, 45 * kDeg), FRONT_LEFT, h), EkfPlanar::SonarResult::Used);
    EXPECT_NEAR(h, 8.0 - 0.15 * std::sqrt(2.0), 1e-9);

    // a thin beam has no cone, so the far end is used even near the side wall
    p.beam_half_angle = 0.0;
    EkfPlanar thin(p);
    EXPECT_EQ(thin.sonar_model(make_state(1.0, 2.0, 45 * kDeg), FRONT_LEFT, h), EkfPlanar::SonarResult::Used);
}

// ------------------------------------------------------------ Recovery

// The filter is 1 m wrong but confident, so it gates every true reading.
// After gate_reset_count rejections in a row it must recover and track again.
TEST(EkfPlanarRecovery, RecoversFromConfidentWrongPosition)
{
    EkfPlanar::Params p = pool_params();
    p.walls = {{1, 0, 0}};  // left wall only
    EkfPlanar ekf(p), truth_model(p);
    const EkfPlanar::State truth = make_state(1.0, 5.0, 45 * kDeg);

    EkfPlanar::Cov P0 = EkfPlanar::Cov::Identity() * 1e-4;  // 1 cm, 0.6 deg: far too sure
    ekf.init(make_state(2.0, 5.0, 45 * kDeg), P0);

    double h;
    ASSERT_EQ(truth_model.sonar_model(truth, BACK_LEFT, h), EkfPlanar::SonarResult::Used);
    for (int i = 1; i < p.gate_reset_count; ++i)
        ASSERT_EQ(ekf.update_sonar(BACK_LEFT, h), EkfPlanar::SonarResult::Gated) << "reading " << i;
    EXPECT_EQ(ekf.update_sonar(BACK_LEFT, h), EkfPlanar::SonarResult::Recovered);

    // back on track: the next readings are accepted and x is right
    for (int i = 0; i < 5; ++i) EXPECT_EQ(ekf.update_sonar(BACK_LEFT, h), EkfPlanar::SonarResult::Used);
    EXPECT_NEAR(ekf.state()(EkfPlanar::PX), 1.0, 0.02);
}

TEST(EkfPlanarRecovery, OccasionalBadEchoDoesNotTriggerRecovery)
{
    EkfPlanar::Params p = pool_params();
    p.walls = {{1, 0, 0}};
    EkfPlanar ekf(p), truth_model(p);
    const EkfPlanar::State truth = make_state(1.0, 5.0, 45 * kDeg);
    EkfPlanar::Cov P0 = EkfPlanar::Cov::Identity() * 1e-4;
    ekf.init(truth, P0);

    double h;
    ASSERT_EQ(truth_model.sonar_model(truth, BACK_LEFT, h), EkfPlanar::SonarResult::Used);
    // bad echo every other reading, many times over: the good ones reset the count
    for (int i = 0; i < 3 * p.gate_reset_count; ++i) {
        EXPECT_EQ(ekf.update_sonar(BACK_LEFT, h + 0.5), EkfPlanar::SonarResult::Gated);
        EXPECT_EQ(ekf.update_sonar(BACK_LEFT, h), EkfPlanar::SonarResult::Used);
    }
    EXPECT_NEAR(ekf.state()(EkfPlanar::PX), 1.0, 0.01);
}

// ------------------------------------------------------------ 3. Zig-zag run

struct Segment {
    bool turn;
    double amount;  // leg length [m] or turn angle [rad]
};

struct RunResult {
    double max_x_err = 0, max_psi_err = 0;
    double final_y_err = 0, final_y_err_imu_only = 0, final_bg_err = 0;
    double travelled_y = 0;
    int sonar_used = 0, sonar_gated = 0;
};

RunResult run_zigzag(const EkfPlanar::Params& p, unsigned seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);

    // Sensor errors
    const double accel_noise = 0.02, gyro_noise = 0.002, sonar_noise = 0.01;
    const double gyro_bias = 0.3 * kDeg;  // 0.3 deg/s: 18 deg/min if not learned

    // Truth starts at the requested pose
    double px = 0.5, py = 5.0, psi = 45 * kDeg, v = 0.0;

    // 6 legs of 1.6 m at 45 deg (x stays within ~0.5..1.6 m), 90 deg turns between
    std::vector<Segment> plan;
    for (int i = 0; i < 6; ++i) {
        if (i) plan.push_back({true, (i % 2 ? 90 : -90) * kDeg});
        plan.push_back({false, 1.6});
    }

    EkfPlanar ekf(p), imu_only(p), truth_model(p);
    EkfPlanar::Cov P0 = EkfPlanar::Cov::Zero();
    P0.diagonal() << 0.05 * 0.05, 0.05 * 0.05, std::pow(2 * kDeg, 2), 0.01 * 0.01, 0.01 * 0.01,
        std::pow(0.5 * kDeg, 2);
    const EkfPlanar::State x0 = make_state(px, py, psi);
    ekf.init(x0, P0);
    imu_only.init(x0, P0);

    const double dt = 0.001;  // truth at 1 kHz, IMU at 100 Hz, sonars at 20 Hz
    long step = 0;
    RunResult r;
    const double vmax = 0.1, amax = 0.05, wmax = 0.3;

    for (const Segment& seg : plan) {
        const double ramp_t = vmax / amax;
        const double dur = seg.turn ? std::abs(seg.amount) / wmax
                                    : 2 * ramp_t + (seg.amount - vmax * ramp_t) / vmax;
        for (double t = 0; t < dur; t += dt, ++step) {
            double a = 0, w = 0;
            if (seg.turn) {
                w = std::copysign(wmax, seg.amount);
            } else {
                a = t < ramp_t ? amax : (t > dur - ramp_t ? -amax : 0.0);
            }
            const double a_lat = v * w;  // centripetal, 0 here (turns happen in place)

            // truth: moves only the way it points
            v = std::max(0.0, v + a * dt);
            psi += w * dt;
            px += v * std::cos(psi) * dt;
            py += v * std::sin(psi) * dt;

            if (step % 10 == 0) {  // IMU, 100 Hz
                const Eigen::Vector2d am(a + accel_noise * N01(rng), a_lat + accel_noise * N01(rng));
                const double gm = w + gyro_bias + gyro_noise * N01(rng);
                ekf.predict(am, gm, 0.01);
                ekf.update_no_sideslip();
                imu_only.predict(am, gm, 0.01);
            }

            if (step % 50 == 0) {  // sonars, 20 Hz, generated from the true pose
                const EkfPlanar::State xt = make_state(px, py, psi);
                for (int s = 0; s < static_cast<int>(p.sonars.size()); ++s) {
                    double h;
                    if (truth_model.sonar_model(xt, s, h) != EkfPlanar::SonarResult::Used) continue;
                    const auto res = ekf.update_sonar(s, h + sonar_noise * N01(rng));
                    r.sonar_used += res == EkfPlanar::SonarResult::Used;
                    r.sonar_gated += res == EkfPlanar::SonarResult::Gated;
                }

                if (step * dt > 5.0) {  // after the filter settles
                    const auto& x = ekf.state();
                    r.max_x_err = std::max(r.max_x_err, std::abs(x(EkfPlanar::PX) - px));
                    r.max_psi_err = std::max(r.max_psi_err,
                                             std::abs(EkfPlanar::wrap_angle(x(EkfPlanar::PSI) - psi)));
                }
            }
        }
    }

    r.final_y_err = std::abs(ekf.state()(EkfPlanar::PY) - py);
    r.final_y_err_imu_only = std::abs(imu_only.state()(EkfPlanar::PY) - py);
    r.final_bg_err = std::abs(ekf.state()(EkfPlanar::BG) - gyro_bias);
    r.travelled_y = py - 5.0;
    return r;
}

void print(const char* name, const RunResult& r)
{
    std::printf("[%s] y travelled %.2f m | max x err %.1f mm | max heading err %.2f deg | "
                "final y err %.1f mm (IMU only %.1f mm) | gyro bias err %.3f deg/s | "
                "sonar used %d gated %d\n",
                name, r.travelled_y, 1000 * r.max_x_err, r.max_psi_err / kDeg, 1000 * r.final_y_err,
                1000 * r.final_y_err_imu_only, r.final_bg_err / kDeg, r.sonar_used, r.sonar_gated);
}

TEST(EkfPlanarZigzag, ThinBeam)
{
    EkfPlanar::Params p = pool_params();
    p.sonar_sigma = 0.01;
    p.accel_sigma = 0.05;
    p.gyro_sigma = 0.005;
    const RunResult r = run_zigzag(p, 42);
    print("thin beam", r);

    EXPECT_GT(r.travelled_y, 5.0);                          // it really went along the pool
    EXPECT_LT(r.max_x_err, 0.03);                           // wall distance within 3 cm
    EXPECT_LT(r.max_psi_err, 2.0 * kDeg);                   // heading within 2 deg
    EXPECT_LT(r.final_bg_err, 0.05 * kDeg);                 // gyro bias learned
    // Absolute limit only: the IMU-only drift is random and sometimes lands near
    // the truth by chance, so comparing against it fails on ~5% of seeds.
    EXPECT_LT(r.final_y_err, 0.15);                         // along-wall drift under 15 cm over ~6 m
}

TEST(EkfPlanarZigzag, WideBeam)
{
    EkfPlanar::Params p = pool_params();
    p.beam_half_angle = 12.5 * kDeg;
    p.sonar_sigma = 0.01;
    const RunResult r = run_zigzag(p, 7);
    print("12.5 deg half beam", r);

    EXPECT_LT(r.max_x_err, 0.03);
    EXPECT_LT(r.max_psi_err, 2.0 * kDeg);
    EXPECT_LT(r.final_y_err, 0.15);
}
}  // namespace
