#include "ekf_planar/ekf_planar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
// Where a sonar is and which way it points, in the pool frame, plus the derivatives of both w.r.t.
// heading (the only state they depend on besides position, whose derivatives are trivial).
struct SonarGeom {
  double sx, sy;    // sonar position (x, y): where the sonar is in the pool
  double dsx, dsy;  // d(sx, sy)/dpsi: how sonar position changes if the heading changes
  double ux, uy;    // unit beam direction
  double dux, duy;  // d(ux, uy)/dpsi: how beam direction changes if the heading changes
};

// Computes the global position and direction of a sonar based on the robot's state and the sonar's
// parameters. It utilizes trigonometric functions to transform the robot's frame of reference into
// the pool's coordinate system, returning a SonarGeom object.
SonarGeom sonar_geom(const EkfPlanar::State& x, const EkfPlanar::Sonar& sonar) {
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

EkfPlanar::EkfPlanar(const Params& params)
    : ekf_params_(params), gated_in_a_row_(params.sonars.size(), 0) {
  x_.setZero();
  P_ = Cov::Identity();
}

void EkfPlanar::init(const State& x0, const Cov& P0) {
  x_ = x0;
  x_(PSI) = wrap_angle(x_(PSI));
  P_ = P0;
  std::fill(gated_in_a_row_.begin(), gated_in_a_row_.end(), 0);
}

double EkfPlanar::wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// -----predict-----

// the motion model
EkfPlanar::State EkfPlanar::f(const State& x, const Eigen::Vector2d& a_body, double gyro_z,
                              double dt) {
  // cosine and sine of the heading, used to rotate from the robot's frame into the pool's
  const double cos_psi = std::cos(x(PSI));
  const double sin_psi = std::sin(x(PSI));
  // body acceleration rotated into the pool frame by the current heading
  const double ax = cos_psi * a_body(0) - sin_psi * a_body(1);
  const double ay = sin_psi * a_body(0) + cos_psi * a_body(1);

  State xn = x;
  xn(PX) += x(VX) * dt + 0.5 * ax * dt * dt;  // position: s = s + v·t + ½·a·t²
  xn(PY) += x(VY) * dt + 0.5 * ay * dt * dt;
  xn(PSI) = wrap_angle(x(PSI) + (gyro_z - x(BG)) * dt);  // heading: add bias-corrected yaw rate
  xn(VX) += ax * dt;                                     // velocity: v = v + a·t
  xn(VY) += ay * dt;
  // xn(BG) stays constant; Q lets it wander slowly
  return xn;
}

EkfPlanar::Cov EkfPlanar::F_jacobian(const State& x, const Eigen::Vector2d& a_body, double dt) {
  // cosine and sine of the heading, used to rotate from the robot's frame into the pool's
  const double cos_psi = std::cos(x(PSI));
  const double sin_psi = std::sin(x(PSI));
  // body acceleration rotated into the pool frame by the current heading
  const double ax = cos_psi * a_body(0) - sin_psi * a_body(1);
  const double ay = sin_psi * a_body(0) + cos_psi * a_body(1);
  // d(ax)/dpsi = -ay, d(ay)/dpsi = ax

  // Start from the identity, "each state carries over to the next step unchanged", then add how
  // each state also affects the others.
  Cov F = Cov::Identity();
  F(PX, VX) = dt;  // an x-velocity error of 2 m/s becomes an x-position error of dt metres
  F(PX, PSI) =
      -0.5 * ay * dt * dt;  // a heading error tilts the rotated acceleration, which shifts position
  F(PY, VY) = dt;           // an y-velocity error of 2 m/s becomes an y-position error of dt metres
  F(PY, PSI) =
      0.5 * ax * dt * dt;  // a heading error tilts the rotated acceleration, which shifts position
  F(PSI, BG) = -dt;        // a bias error of 1 rad/s becomes a heading error of −dt radians
  F(VX, PSI) = -ay * dt;   // a heading error shifts velocity
  F(VY, PSI) = ax * dt;    // a heading error shifts velocity
  return F;
}

void EkfPlanar::predict(const Eigen::Vector2d& a_body, double gyro_z, double dt) {
  if (dt <= 0.0) return;

  const Cov F = F_jacobian(x_, a_body, dt);  // slopes at the OLD state
  x_ = f(x_, a_body, gyro_z, dt);            // move the guess forward using motion model

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

/// The measurement model: expected reading against one wall
EkfPlanar::SonarResult EkfPlanar::sonar_wall_model(const State& x, int sonar, const Wall& wall,
                                                   double& h, Row* H) const {
  const SonarGeom geom = sonar_geom(x, ekf_params_.sonars.at(sonar));

  // perpendicular distance from sonar to wall, n·q − c
  const double d = wall.nx * geom.sx + wall.ny * geom.sy - wall.c;
  // cos of the angle between beam and wall's unit normal (1 = head on),
  // based on n_1 . n_2 = cos(theta) for unit vectors
  const double cos_phi = -(wall.nx * geom.ux + wall.ny * geom.uy);

  if (cos_phi <= 1e-9 || d < 0.0) return SonarResult::NoWall;  // facing away or beyond the wall

  const double phi =
      std::acos(std::min(1.0, cos_phi));  // angle between beam and wall's unit normal
  if (phi > ekf_params_.max_incidence)
    return SonarResult::TooOblique;  // beyond threshold the reading is ignored

  const double dd_dpsi =
      wall.nx * geom.dsx + wall.ny * geom.dsy;  // how d changes as heading changes
  const double dcos_phi_dpsi =
      -(wall.nx * geom.dux + wall.ny * geom.duy);   // how cos_phi changes with heading
  const double beta = ekf_params_.beam_half_angle;  // half the sonar's cone width

  double dh_dd;    // how h (expected reading) changes with d (perpendicular distance)
  double dh_dpsi;  // how h (expected reading) changes with psi (robot heading)

  if (phi <= beta) {
    // case 1: the wall's normal is inside the cone, so the sonar reads d
    h = d;
    dh_dd = 1.0;
    dh_dpsi = dd_dpsi;
  } else if (beta <= 0.0) {
    // case 2: ideal thin beam, h = d / cos_phi
    // (written this way to avoid acos' slope at cos_phi = 1)
    h = d / cos_phi;
    dh_dd = 1.0 / cos_phi;
    dh_dpsi = (dd_dpsi * cos_phi - d * dcos_phi_dpsi) / (cos_phi * cos_phi);  // quotient rule
  } else {
    // case 3: wide beam, the ray closest to the normal is the cone edge, at angle phi − beta from
    // the normal
    const double cos_edge_angle = std::cos(phi - beta);

    const double dphi_dpsi =
        -dcos_phi_dpsi / std::sqrt(1.0 - cos_phi * cos_phi);  // phi > beta > 0, so cos_phi < 1
    const double dcos_edge_angle_dpsi = -std::sin(phi - beta) * dphi_dpsi;

    h = d / cos_edge_angle;  // Its length is d / cos(phi − beta)
    dh_dd = 1.0 / cos_edge_angle;
    dh_dpsi =
        (dd_dpsi * cos_edge_angle - d * dcos_edge_angle_dpsi) / (cos_edge_angle * cos_edge_angle);
  }

  // Fills in the Jacobian row. Velocity and gyro bias don't affect range readings
  if (H) {
    H->setZero();
    (*H)(PX) = dh_dd * wall.nx;
    (*H)(PY) = dh_dd * wall.ny;
    (*H)(PSI) = dh_dpsi;
  }
  return SonarResult::Used;
}
///  The pool has several walls and this finds the one the beam actually hits.
EkfPlanar::SonarResult EkfPlanar::sonar_model(const State& x, int sonar, double& h, Row* H) const {
  // The sonar reports the NEAREST echo anywhere in its cone, so the wall it
  // hits is the one with the shortest cone range, not centre-line range. Near
  // a wall, a beam running alongside it can clip it with the cone edge before
  // the centre line reaches the wall ahead.
  const SonarGeom geom = sonar_geom(x, ekf_params_.sonars.at(sonar));
  const double beta = std::max(0.0, ekf_params_.beam_half_angle);
  const Wall* first = nullptr;
  double best = std::numeric_limits<double>::infinity();

  for (const Wall& wall : ekf_params_.walls) {
    const double d = wall.nx * geom.sx + wall.ny * geom.sy -
                     wall.c;  // perpendicular distance from sonar to wall
    if (d < 0.0) continue;    // beyond the wall
    const double cos_phi =
        -(wall.nx * geom.ux +
          wall.ny * geom.uy);  // cos of the angle between beam and wall's unit normal
    // the ray in the cone closest to the wall's normal
    const double edge = std::max(0.0, std::acos(std::clamp(cos_phi, -1.0, 1.0)) - beta);
    if (edge >= M_PI / 2 - 1e-9) continue;  // no ray in the cone reaches this wall
    const double range = d / std::cos(edge);
    if (range < best) {
      best = range;
      first = &wall;
    }
  }
  if (!first) return SonarResult::NoWall;

  // If the nearest echo is a grazing one, sonar_wall_model rejects it
  // (TooOblique / NoWall) rather than this reading being matched to a wall
  // further away.

  const SonarResult result = sonar_wall_model(x, sonar, *first, h, H);
  if (result != SonarResult::Used) return result;
  if (h < ekf_params_.min_range || h > ekf_params_.max_range) return SonarResult::OutOfRange;
  return SonarResult::Used;
}

/// Correct the state and covariance with one sonar reading. Returns whether the reading was used.
EkfPlanar::SonarResult EkfPlanar::update_sonar(int sonar, double range) {
  if (sonar < 0 || sonar >= static_cast<int>(ekf_params_.sonars.size()))
    throw std::out_of_range("EkfPlanar::update_sonar: bad sonar index");

  double h;
  Row H;
  const SonarResult result = sonar_model(x_, sonar, h, &H);
  if (result != SonarResult::Used) return result;
  // A reading outside the sonar's limits is unusable
  if (range < ekf_params_.min_range || range > ekf_params_.max_range)
    return SonarResult::OutOfRange;

  const double R = ekf_params_.sonar_sigma * ekf_params_.sonar_sigma;  // measurement noise variance
  int& gated = gated_in_a_row_[sonar];
  if (scalar_update(range - h, H, R, ekf_params_.gate_chi2)) {
    gated = 0;
    return SonarResult::Used;
  }

  // Rejected. One or two in a row is a bad echo; a whole run of them means the
  // filter itself is off and too confident to accept the truth.
  if (ekf_params_.gate_reset_count <= 0 || ++gated < ekf_params_.gate_reset_count)
    return SonarResult::Gated;

  // Lost: admit more uncertainty, then take this reading without the gate.
  // x_ is unchanged, so h and H still hold.
  gated = 0;
  const double sp = ekf_params_.reset_pos_sigma, sh = ekf_params_.reset_psi_sigma,
               sv = ekf_params_.reset_vel_sigma;
  P_(PX, PX) += sp * sp;
  P_(PY, PY) += sp * sp;
  P_(PSI, PSI) += sh * sh;
  P_(VX, VX) += sv * sv;
  P_(VY, VY) += sv * sv;
  scalar_update(range - h, H, R, std::numeric_limits<double>::infinity());
  return SonarResult::Recovered;
}

// ----- sideslip -----

double EkfPlanar::sideslip_model(const State& x, Row* H) {
  // The robot's left axis in pool coordinates is (−sinψ, cosψ).
  const double cos_psi = std::cos(x(PSI));
  const double sin_psi = std::sin(x(PSI));

  // Taking the dot product of the global velocity with robot's left axis gives the sideways
  // velocity v_side in the body frame.
  const double v_side = -sin_psi * x(VX) + cos_psi * x(VY);

  if (H) {
    H->setZero();
    (*H)(PSI) = -cos_psi * x(VX) -
                sin_psi * x(VY);  // partial derivatives of v_side with respect to heading
    (*H)(VX) = -sin_psi;          // partial derivative of v_side with respect to vx
    (*H)(VY) = cos_psi;           // partial derivative of v_side with respect to vy
  }
  return v_side;
}

void EkfPlanar::update_no_sideslip() {
  Row H;
  const double h = sideslip_model(x_, &H);
  const double R = ekf_params_.sideslip_sigma * ekf_params_.sideslip_sigma;
  // "measurement" is 0: the robot never moves sideways. No gate.
  scalar_update(0.0 - h, H, R, std::numeric_limits<double>::infinity());
}

// ----- update core -----

bool EkfPlanar::scalar_update(double y, const Row& H, double R, double gate) {
  const double S = (H * P_ * H.transpose())(0, 0) +
                   R;  // S is the innovation variance: how much spread we expect in y (innovation)
  last_y_ = y;         // saved for diagnostics or logging
  last_S_ = S;         // saved for diagnostics or logging

  // This is a chi-square gate. y²/S measures how surprising the reading is in units of standard
  // deviations, squared. With one degree of freedom, a value above 6.63 happens only 1% of the time
  // if everything is working, so it's treated as an outlier (multipath echo, a corner) and
  // ignored.
  if (y * y / S > gate) return false;

  const State K = P_ * H.transpose() / S;  // Kalman gain
  x_ += K * y;  //Applies the correction
  x_(PSI) = wrap_angle(x_(PSI));  // keep heading in [−π, π]

  // Joseph form: same result as (I - KH)P, but stays symmetric and positive
  const Cov I_KH = Cov::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  return true;
}
