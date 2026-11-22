#pragma once

// Planar EKF for the zig-zag-with-yaw experiment.
//
// Frames
//   Pool frame (ENU): +x along the width, away from the left wall; 
//   +y down the length.
//
//   Body frame (FLU): +x forward, +y left, +z up. Heading is measured
//   counter-clockwise from pool +x.
//
// State x = [px, py, psi, vx, vy, b_g]
//   px, py  position of the body origin in the pool frame [m]
//   psi     heading [rad]
//   vx, vy  velocity in the pool frame [m/s]
//   b_g     gyro z bias [rad/s]
//
// Information sources
//   predict()             IMU: body-frame planar acceleration and yaw rate
//
//   update_sonar()        Sonar range to whichever wall the beam hits first
//
//   update_no_sideslip()  The robot only moves the way it points, so its
//                         sideways body velocity is ~0. Together with the
//                         wall ranges this makes along-wall motion (py)
//                         observable on legs that are diagonal to the wall.

#include <eigen3/Eigen/Dense>
#include <vector>

class EkfPlanar
{
public:
    static constexpr int N = 6;     // number of states
    enum Idx { PX = 0, PY, PSI, VX, VY, BG };   // names for each state-vector position; the values increment from PX=0
    using State = Eigen::Matrix<double, N, 1>;  // State is a 6x1 Eigen column vector, one value per state
    using Cov = Eigen::Matrix<double, N, N>;    // 6×6: the covariance P (and F, Q)
    using Row = Eigen::Matrix<double, 1, N>;    // 1×6: a Jacobian row H for ONE measurement
    
    struct Sonar {
        double ox, oy;   // mount position in the body frame [m] (z ignored: walls are vertical)
        double alpha;    // beam direction in the body frame [rad], 0 = forward, CCW positive
    };

    // A wall is the line n.q = c (dot product form), with unit normal n pointing INTO the pool,
    // so n.q - c is the (positive) distance of a point q inside the pool.
    //   left wall  x = 0 : {  1,  0,   0 }
    //   right wall x = W : { -1,  0,  -W }
    //   near end   y = 0 : {  0,  1,   0 }
    //   far end    y = L : {  0, -1,  -L }
    struct Wall {
        double nx, ny, c;
    };

    struct Params {
        std::vector<Sonar> sonars;
        std::vector<Wall> walls;

        double beam_half_angle = 0.0;       // [rad] half the cone width; 0 = ideal thin beam
        double max_incidence = 0.873;       // [rad] reject beams more oblique than this (50 deg)
        double min_range = 0.1;             // [m]
        double max_range = 2.5;             // [m]
        double sonar_sigma = 0.02;          // [m] range noise std
        double gate_chi2 = 6.63;            // reject if innovation^2 / S exceeds this (99%, 1 DOF)

        double accel_sigma = 0.05;          // [m/s^2] accelerometer noise (also covers small biases)
        double gyro_sigma = 0.005;          // [rad/s] gyro noise
        double gyro_bias_walk = 1e-4;       // [rad/s/sqrt(s)] how fast the gyro bias may wander

        double sideslip_sigma = 0.01;       // Non-holonomic constraint: std of sideways body velocity [m/s]
    };

    enum class SonarResult { Used, NoWall, TooOblique, OutOfRange, Gated };

    explicit EkfPlanar(const Params& params);

    void init(const State& x0, const Cov& P0);

    // Predict the next state and covariance given the IMU readings and time step.
    // a_body: gravity-free planar acceleration in the body frame [m/s^2]
    // gyro_z: measured yaw rate [rad/s]
    void predict(const Eigen::Vector2d& a_body, double gyro_z, double dt);

    SonarResult update_sonar(int sonar, double range);  // correct with one sonar reading
    void update_no_sideslip();                          // apply "no sideways motion"

    const State& state() const { return x_; }           // get the current estimate
    const Cov& cov() const { return P_; }               // get the current uncertainty
    const Params& params() const { return ekf_params_; }    // get the parameters

    // Last innovation (measurement minus prediction) and its variance S,
    // for logging and tuning.
    double last_innovation() const { return last_y_; }
    double last_innovation_var() const { return last_S_; }

    // --- Models. Public so the unit tests can check the Jacobians. ---

    // Motion model x_k+1 = f(x_k, u)
    static State f(const State& x, const Eigen::Vector2d& a_body, double gyro_z, double dt);
    // Motion model Jacobian F = df/dx.
    static Cov F_jacobian(const State& x, const Eigen::Vector2d& a_body, double dt);

    // Expected range of `sonar` against one wall, and H = dh/dx.
    // Returns TooOblique / NoWall when the beam cannot produce a usable echo
    // from that wall; range limits are NOT applied here.
    SonarResult sonar_wall_model(const State& x, int sonar, const Wall& wall,
                                 double& h,                 // ← output: h(x), the expected reading
                                 Row* H = nullptr) const;   // ← optional output: H = ∂h/∂x (1×6)

    // The full measurement model the filter uses: 
    // picks the wall the beam hits first, calls sonar_wall_model on it, then applies the range limits.
    SonarResult sonar_model(const State& x, int sonar, double& h, Row* H = nullptr) const;

    // Sideways body velocity, and its Jacobian.
    static double sideslip_model(const State& x, Row* H = nullptr);

    static double wrap_angle(double a);

private:
    // Scalar EKF update with innovation y, Jacobian row H, noise variance R.
    // Returns false (and changes nothing) if the innovation fails the gate.
    bool scalar_update(double y, const Row& H, double R, double gate);

    Params ekf_params_;
    State x_;
    Cov P_;
    double last_y_ = 0.0, last_S_ = 0.0;
};