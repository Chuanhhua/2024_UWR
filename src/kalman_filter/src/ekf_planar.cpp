#include "../include/ekf_planar.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
// Where a sonar is and which way it points, in the pool frame, plus the
// derivatives of both w.r.t. heading (the only state they depend on besides
// position, whose derivatives are trivial).
struct SonarGeom {
    double sx, sy;        // sonar position (x, y): where the sonar is in the pool
    double dsx, dsy;      // d(sx, sy)/dpsi: how sonar position changes if the heading changes
    double ux, uy;        // unit beam direction
    double dux, duy;      // d(ux, uy)/dpsi: how beam direction changes if the heading changes
};

// Computes the global position and direction of a sonar based on the robot's state 
// and the sonar's parameters. It utilizes trigonometric functions to transform the
// robot's frame of reference into the pool's coordinate system, returning a SonarGeom object.
SonarGeom sonar_geom(const EkfPlanar::State& x, const EkfPlanar::Sonar& sonar)
{
    // cosine and sine of the heading, used to rotate from the robot's frame into the pool's
    const double cos_psi = std::cos(x(EkfPlanar::PSI));
    const double sin_psi = std::sin(x(EkfPlanar::PSI));

    // beam's direction in the pool = robot heading + the sonar's angle on the robot
    const double beam_dir = x(EkfPlanar::PSI) + sonar.alpha;

    SonarGeom g;
    // sonar position (global) = robot position + mount offset rotated by heading
    g.sx = x(EkfPlanar::PX) + cos_psi * sonar.ox - sin_psi * sonar.oy;
    g.sy = x(EkfPlanar::PY) + sin_psi * sonar.ox + cos_psi * sonar.oy;
    g.dsx = -sin_psi * sonar.ox - cos_psi * sonar.oy;
    g.dsy = cos_psi * sonar.ox - sin_psi * sonar.oy;
    g.ux = std::cos(beam_dir);
    g.uy = std::sin(beam_dir);
    g.dux = -g.uy;
    g.duy = g.ux;
    return g;
}
}  // namespace

EkfPlanar::EkfPlanar(const Params& params) : ekf_params_(params)
{
    x_.setZero();
    P_ = Cov::Identity();
}

void EkfPlanar::init(const State& x0, const Cov& P0)
{
    x_ = x0;
    x_(PSI) = wrap_angle(x_(PSI));
    P_ = P0;
}

double EkfPlanar::wrap_angle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

// -----predict-----

// the motion model
EkfPlanar::State EkfPlanar::f(const State& x, const Eigen::Vector2d& a_body,
                              double gyro_z, double dt)
{
    // cosine and sine of the heading, used to rotate from the robot's frame into the pool's
    const double cos_psi = std::cos(x(PSI));
    const double sin_psi = std::sin(x(PSI));
    // body acceleration rotated into the pool frame by the current heading
    const double ax = cos_psi * a_body(0) - sin_psi * a_body(1);
    const double ay = sin_psi * a_body(0) + cos_psi * a_body(1);

    State xn = x;
    xn(PX) += x(VX) * dt + 0.5 * ax * dt * dt;  // position: s = s + v·t + ½·a·t²
    xn(PY) += x(VY) * dt + 0.5 * ay * dt * dt;  
    xn(PSI) = wrap_angle(x(PSI) + (gyro_z - x(BG)) * dt);   // heading: add bias-corrected yaw rate
    xn(VX) += ax * dt;  // velocity: v = v + a·t
    xn(VY) += ay * dt;
    // xn(BG) stays constant; Q lets it wander slowly
    return xn;
}

EkfPlanar::Cov EkfPlanar::F_jacobian(const State& x, const Eigen::Vector2d& a_body, double dt)
{
    // cosine and sine of the heading, used to rotate from the robot's frame into the pool's
    const double cos_psi = std::cos(x(PSI));
    const double sin_psi = std::sin(x(PSI));
    // body acceleration rotated into the pool frame by the current heading
    const double ax = cos_psi * a_body(0) - sin_psi * a_body(1);
    const double ay = sin_psi * a_body(0) + cos_psi * a_body(1);
    // d(ax)/dpsi = -ay, d(ay)/dpsi = ax

    // Start from the identity, "each state carries over to the next step unchanged", 
    // then add how each state also affects the others.
    Cov F = Cov::Identity(); 
    F(PX, VX) = dt;                     // an x-velocity error of 2 m/s becomes an x-position error of dt metres
    F(PX, PSI) = -0.5 * ay * dt * dt;   // a heading error tilts the rotated acceleration, which shifts position
    F(PY, VY) = dt;                     // an y-velocity error of 2 m/s becomes an y-position error of dt metres
    F(PY, PSI) = 0.5 * ax * dt * dt;    // a heading error tilts the rotated acceleration, which shifts position
    F(PSI, BG) = -dt;                   // a bias error of 1 rad/s becomes a heading error of −dt radians
    F(VX, PSI) = -ay * dt;              // a heading error shifts velocity 
    F(VY, PSI) = ax * dt;               // a heading error shifts velocity
    return F;
}

void EkfPlanar::predict(const Eigen::Vector2d& a_body, double gyro_z, double dt)
{
    if (dt <= 0.0) return;

    const Cov F = F_jacobian(x_, a_body, dt);   // slopes at the OLD state
    x_ = f(x_, a_body, gyro_z, dt);             // move the guess forward using motion model

    // Process noise: accelerometer noise held over dt (isotropic, so the
    // rotation does not matter), gyro noise on heading, random walk on bias.
    const double qa = ekf_params_.accel_sigma * ekf_params_.accel_sigma;
    const double dt2 = dt * dt;
    Cov Q = Cov::Zero();
    for (int pos : {PX, PY}) {
        const int vel = pos == PX ? VX : VY;
        Q(pos, pos) = qa * dt2 * dt2 / 4.0;
        Q(pos, vel) = Q(vel, pos) = qa * dt2 * dt / 2.0;
        Q(vel, vel) = qa * dt2;
    }
    Q(PSI, PSI) = ekf_params_.gyro_sigma * ekf_params_.gyro_sigma * dt2;
    Q(BG, BG) = ekf_params_.gyro_bias_walk * ekf_params_.gyro_bias_walk * dt;

    P_ = F * P_ * F.transpose() + Q;
}

// ----- sonar -----

// the measurement model: expected reading against one wall
EkfPlanar::SonarResult EkfPlanar::sonar_wall_model(const State& x, int sonar, const Wall& w,
                                                   double& h, Row* H) const
{
    const SonarGeom g = sonar_geom(x, ekf_params_.sonars.at(sonar));

    const double d = w.nx * g.sx + w.ny * g.sy - w.c;   // perpendicular distance from sonar to wall
    const double k = -(w.nx * g.ux + w.ny * g.uy);      // cos of the angle between the beam and the wall's normal (1 = head on)

    if (k <= 1e-9 || d < 0.0) return SonarResult::NoWall;   // facing away or beyond the wall

    const double phi = std::acos(std::min(1.0, k));                         // angle from head-on
    if (phi > ekf_params_.max_incidence) return SonarResult::TooOblique;    // beyond 50° the reading is ignored

    const double dd_dpsi = w.nx * g.dsx + w.ny * g.dsy;     // how d changes as heading changes
    const double dk_dpsi = -(w.nx * g.dux + w.ny * g.duy);  // how k changes as heading changes
    const double beta = ekf_params_.beam_half_angle;

    // Range the sonar reports. It returns the NEAREST echo inside its cone:
    //  - wall normal inside the cone (phi <= beta): the perpendicular distance d
    //  - otherwise: along the cone edge closest to the normal, d / cos(phi - beta)
    double dh_dpos;  // dh/dd, multiplies the wall normal for d/dpx, d/dpy
    double dh_dpsi;
    if (phi <= beta) {
        h = d;
        dh_dpos = 1.0;
        dh_dpsi = dd_dpsi;
    } else if (beta <= 0.0) {
        // thin beam: h = d / k (written this way to avoid acos' slope at k = 1)
        h = d / k;
        dh_dpos = 1.0 / k;
        dh_dpsi = (dd_dpsi * k - d * dk_dpsi) / (k * k);    // quotient rule
    } else {
        const double gc = std::cos(phi - beta);
        const double dphi_dpsi = -dk_dpsi / std::sqrt(1.0 - k * k);  // phi > beta > 0, so k < 1
        const double dgc_dpsi = -std::sin(phi - beta) * dphi_dpsi;
        h = d / gc;
        dh_dpos = 1.0 / gc;
        dh_dpsi = (dd_dpsi * gc - d * dgc_dpsi) / (gc * gc);
    }

    if (H) {
        H->setZero();
        (*H)(PX) = dh_dpos * w.nx;
        (*H)(PY) = dh_dpos * w.ny;
        (*H)(PSI) = dh_dpsi;
    }
    return SonarResult::Used;
}

EkfPlanar::SonarResult EkfPlanar::sonar_model(const State& x, int sonar, double& h, Row* H) const
{
    // The beam's first hit is the wall with the shortest distance along the
    // beam (the pool is convex, so this is exact).
    const SonarGeom g = sonar_geom(x, ekf_params_.sonars.at(sonar));
    const Wall* first = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const Wall& w : ekf_params_.walls) {
        const double d = w.nx * g.sx + w.ny * g.sy - w.c;
        const double k = -(w.nx * g.ux + w.ny * g.uy);
        if (k <= 1e-9 || d < 0.0) continue;
        if (d / k < best) {
            best = d / k;
            first = &w;
        }
    }
    if (!first) return SonarResult::NoWall;

    const SonarResult r = sonar_wall_model(x, sonar, *first, h, H);
    if (r != SonarResult::Used) return r;
    if (h < ekf_params_.min_range || h > ekf_params_.max_range) return SonarResult::OutOfRange;
    return SonarResult::Used;
}

EkfPlanar::SonarResult EkfPlanar::update_sonar(int sonar, double range)
{
    if (sonar < 0 || sonar >= static_cast<int>(ekf_params_.sonars.size()))
        throw std::out_of_range("EkfPlanar::update_sonar: bad sonar index");

    double h;
    Row H;
    const SonarResult r = sonar_model(x_, sonar, h, &H);
    if (r != SonarResult::Used) return r;
    // A reading outside the sonar's limits is unusable whatever we predicted.
    if (range < ekf_params_.min_range || range > ekf_params_.max_range) return SonarResult::OutOfRange;

    const double R = ekf_params_.sonar_sigma * ekf_params_.sonar_sigma;
    if (!scalar_update(range - h, H, R, ekf_params_.gate_chi2)) return SonarResult::Gated;
    return SonarResult::Used;
}

// ----- sideslip -----

double EkfPlanar::sideslip_model(const State& x, Row* H)
{
    const double c = std::cos(x(PSI));
    const double s = std::sin(x(PSI));
    // velocity component along the body's left axis
    const double v_side = -s * x(VX) + c * x(VY);
    if (H) {
        H->setZero();
        (*H)(PSI) = -c * x(VX) - s * x(VY);
        (*H)(VX) = -s;
        (*H)(VY) = c;
    }
    return v_side;
}

void EkfPlanar::update_no_sideslip()
{
    Row H;
    const double h = sideslip_model(x_, &H);
    const double R = ekf_params_.sideslip_sigma * ekf_params_.sideslip_sigma;
    // "measurement" is 0: the robot never moves sideways. No gate.
    scalar_update(0.0 - h, H, R, std::numeric_limits<double>::infinity());
}

// ---------------------------------------------------------------- update core

bool EkfPlanar::scalar_update(double y, const Row& H, double R, double gate)
{
    const double S = (H * P_ * H.transpose())(0, 0) + R;  // expected spread of y
    last_y_ = y;
    last_S_ = S;
    if (y * y / S > gate) return false;                     // surprise too large: outlier

    const State K = P_ * H.transpose() / S;                 // Kalman gain
    x_ += K * y;
    x_(PSI) = wrap_angle(x_(PSI));

    // Joseph form: same result as (I - KH)P, but stays symmetric and positive
    const Cov I_KH = Cov::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    return true;
}
