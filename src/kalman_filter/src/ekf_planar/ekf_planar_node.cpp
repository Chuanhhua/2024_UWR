// ROS 2 wrapper around EkfPlanar.
//
// Inputs
//   imu_topic   (sensor_msgs/Imu)              predict + no-sideslip update
//   range_topics (sensor_msgs/Range, one per sonar), or if that is empty
//   sonar_topic (sonar_msgs/ThreeSonarDepth)   one range update per configured sonar
//
// Outputs
//   ~/odom        (nav_msgs/Odometry)          pose + velocity in `frame_id`
//   ~/sonar_diag  (std_msgs/Float64MultiArray) one message per sonar update, for tuning:
//                 [sonar, range, predicted, innovation, S, innovation^2/S, used]
//                 (predicted, innovation and S are from before the update)
//   TF frame_id -> child_frame_id              (if publish_tf)
//
// Services
//   ~/reset       (std_srvs/Trigger)           back to the start pose (init_*), as at launch
//
// Frames
//   The EKF works in the pool frame (origin at the left wall / near end corner,
//   +x across the width, +y down the length). Outputs are shifted by
//   (pool_origin_x, pool_origin_y) into `frame_id`, which must have the same
//   axes as the pool frame (true for the Gazebo world).
//
// Assumes the IMU axes are aligned with base_link (true in the sim).

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sonar_msgs/msg/three_sonar_depth.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ekf_planar/ekf_planar.h"

namespace
{
  using SonarResult = EkfPlanar::SonarResult;

  // distance [mm] and confidence of field 1, 2 or 3 of a ThreeSonarDepth
  std::pair<double, double> sonar_field(const sonar_msgs::msg::ThreeSonarDepth &m, int field)
  {
    switch (field)
    {
    case 1:
      return {m.distance_1, m.confidence_1};
    case 2:
      return {m.distance_2, m.confidence_2};
    case 3:
      return {m.distance_3, m.confidence_3};
    }
    throw std::invalid_argument("sonar field must be 1, 2 or 3");
  }
} // namespace

class EkfPlanarNode : public rclcpp::Node
{
public:
  EkfPlanarNode() : Node("ekf_planar"), ekf_(load_params())
  {
    // ----- initial state (pool frame) -----
    EkfPlanar::State x0;
    x0 << declare_parameter("init_x", 7.5), declare_parameter("init_y", 12.5),
        declare_parameter("init_psi", M_PI / 2), 0.0, 0.0, 0.0;
    const double sp = declare_parameter("init_pos_sigma", 0.1);
    const double sh = declare_parameter("init_psi_sigma", 0.05);
    const double sv = declare_parameter("init_vel_sigma", 0.02);
    const double sb = declare_parameter("init_bias_sigma", 0.01);
    EkfPlanar::Cov P0 = EkfPlanar::Cov::Zero();
    P0.diagonal() << sp * sp, sp * sp, sh * sh, sv * sv, sv * sv, sb * sb;
    ekf_.init(x0, P0);
    x0_ = x0; // kept for ~/reset
    P0_ = P0;

    // ----- output -----
    frame_id_ = declare_parameter("frame_id", std::string("world"));
    child_frame_id_ = declare_parameter("child_frame_id", std::string("base_link_ekf"));
    // pool frame -> frame_id offset; 0 = frame_id is the pool frame (the Gazebo world)
    origin_x_ = declare_parameter("pool_origin_x", 0.0);
    origin_y_ = declare_parameter("pool_origin_y", 0.0);
    out_z_ = declare_parameter("output_z", 0.0);
    publish_tf_ = declare_parameter("publish_tf", true);

    // ----- input handling -----
    min_confidence_ = declare_parameter("min_confidence", 50.0);
    tilt_compensate_ = declare_parameter("tilt_compensate", true);
    max_imu_gap_ = declare_parameter("max_imu_gap", 0.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/sonar_diag", 10);
    if (publish_tf_)
    {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        declare_parameter("imu_topic", std::string("/imu/data")), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu &m)
        { on_imu(m); });
    // Sonar input: one sensor_msgs/Range topic per sonar if range_topics is
    // set, otherwise the three fields of a ThreeSonarDepth.
    const auto range_topics = declare_parameter("range_topics", std::vector<std::string>{});
    const auto sonar_topic = declare_parameter("sonar_topic", std::string("/sbs"));
    if (!range_topics.empty())
    {
      if (range_topics.size() != ekf_.params().sonars.size())
      {
        throw std::invalid_argument("range_topics needs one topic per sonar");
      }
      for (size_t i = 0; i < range_topics.size(); ++i)
      {
        range_subs_.push_back(create_subscription<sensor_msgs::msg::Range>(
            range_topics[i], rclcpp::SensorDataQoS(),
            [this, i](const sensor_msgs::msg::Range &m)
            { on_range(i, m); }));
      }
    }
    else
    {
      if (sonar_fields_.size() != ekf_.params().sonars.size())
      {
        throw std::invalid_argument("sonar_fields needs one entry per sonar");
      }
      sonar_sub_ = create_subscription<sonar_msgs::msg::ThreeSonarDepth>(
          sonar_topic, 10, [this](const sonar_msgs::msg::ThreeSonarDepth &m)
          { on_sonar(m); });
    }

    // ~/reset (std_srvs/Trigger): back to the start pose and uncertainty, as at
    // launch, e.g. to start a new run without restarting the node
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
                ekf_.init(x0_, P0_);
                have_imu_ = false;  // the next IMU sample starts the prediction afresh
                counts_.fill(0);
                low_confidence_ = 0;
                no_echo_ = 0;
                res->success = true;
                res->message = "EKF reset to its start pose";
                RCLCPP_INFO(get_logger(), "reset to pool (%.2f, %.2f), heading %.1f deg",
                            x0_(EkfPlanar::PX), x0_(EkfPlanar::PY), x0_(EkfPlanar::PSI) * 180 / M_PI); });

    // periodic health summary: how the sonar updates went
    stats_timer_ = create_wall_timer(std::chrono::seconds(5), [this]
                                     { log_stats(); });

    RCLCPP_INFO(get_logger(), "EKF started at pool (%.2f, %.2f), heading %.1f deg, %zu sonars",
                x0(EkfPlanar::PX), x0(EkfPlanar::PY), x0(EkfPlanar::PSI) * 180 / M_PI,
                ekf_.params().sonars.size());
  }

private:
  // Reads the filter parameters. Called from the initialiser list, before
  // anything else in the constructor.
  EkfPlanar::Params load_params()
  {
    EkfPlanar::Params p;

    // Sonars, one entry per EKF sonar. Defaults: the sim's surge (forward)
    // and sway (left) sonars, both at the body origin in the plane.
    const auto ox = declare_parameter("sonar_ox", std::vector<double>{0.0, 0.0});
    const auto oy = declare_parameter("sonar_oy", std::vector<double>{0.0, 0.0});
    const auto alpha = declare_parameter("sonar_alpha", std::vector<double>{0.0, M_PI / 2});
    // which ThreeSonarDepth field (1, 2, 3) each sonar reads from; only used
    // without range_topics, but its length must still match when given
    const auto fields = declare_parameter("sonar_fields", std::vector<int64_t>{1, 2});
    const size_t n = ox.size();
    if (oy.size() != n || alpha.size() != n)
    {
      throw std::invalid_argument("sonar_ox, sonar_oy and sonar_alpha must have the same length");
    }
    for (size_t i = 0; i < n; ++i)
    {
      p.sonars.push_back({ox[i], oy[i], alpha[i]});
    }
    for (const int64_t f : fields)
    {
      if (f < 1 || f > 3)
      {
        throw std::invalid_argument("sonar_fields entries must be 1, 2 or 3");
      }
      sonar_fields_.push_back(static_cast<int>(f));
    }

    // Rectangular pool: width along x, length along y. Defaults are the real
    // (and simulated) 15 x 25 m pool; only override for a different pool.
    const double W = declare_parameter("pool_width", 15.0);
    const double L = declare_parameter("pool_length", 25.0);
    p.walls = {{1, 0, 0}, {-1, 0, -W}, {0, 1, 0}, {0, -1, -L}};

    p.beam_half_angle = declare_parameter("beam_half_angle", p.beam_half_angle);
    p.max_incidence = declare_parameter("max_incidence", p.max_incidence);
    p.min_range = declare_parameter("min_range", p.min_range);
    p.max_range = declare_parameter("max_range", p.max_range);
    p.sonar_sigma = declare_parameter("sonar_sigma", p.sonar_sigma);
    p.gate_chi2 = declare_parameter("gate_chi2", p.gate_chi2);
    p.gate_reset_count = static_cast<int>(declare_parameter("gate_reset_count", int64_t{p.gate_reset_count}));
    p.gate_reset_spread = declare_parameter("gate_reset_spread", p.gate_reset_spread);
    p.reset_pos_sigma = declare_parameter("reset_pos_sigma", p.reset_pos_sigma);
    p.reset_psi_sigma = declare_parameter("reset_psi_sigma", p.reset_psi_sigma);
    p.reset_vel_sigma = declare_parameter("reset_vel_sigma", p.reset_vel_sigma);
    p.accel_sigma = declare_parameter("accel_sigma", p.accel_sigma);
    p.gyro_sigma = declare_parameter("gyro_sigma", p.gyro_sigma);
    p.gyro_bias_walk = declare_parameter("gyro_bias_walk", p.gyro_bias_walk);
    p.sideslip_sigma = declare_parameter("sideslip_sigma", p.sideslip_sigma);
    return p;
  }

  void on_imu(const sensor_msgs::msg::Imu &m)
  {
    const rclcpp::Time t(m.header.stamp);
    if (!have_imu_)
    {
      have_imu_ = true;
      last_imu_ = t;
      return;
    }
    const double dt = (t - last_imu_).seconds();
    last_imu_ = t;
    if (dt <= 0.0 || dt > max_imu_gap_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "skipping IMU sample, dt = %.3f s", dt);
      return;
    }

    // The accelerometer measures specific force (gravity included). Undo
    // roll and pitch so gravity lands on z only, leaving planar acceleration
    // in a level frame that turns with the robot.
    tf2::Vector3 f(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
    if (tilt_compensate_ && m.orientation_covariance[0] >= 0.0)
    { // -1 = no orientation
      const tf2::Quaternion q(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      tf2::Matrix3x3 level;
      level.setRPY(roll, pitch, 0.0);
      f = level * f;
    }

    ekf_.predict(Eigen::Vector2d(f.x(), f.y()), m.angular_velocity.z, dt);
    ekf_.update_no_sideslip();
    publish(t);
  }

  // ThreeSonarDepth: distances in mm, with a confidence per sonar
  void on_sonar(const sonar_msgs::msg::ThreeSonarDepth &m)
  {
    for (size_t i = 0; i < sonar_fields_.size(); ++i)
    {
      const auto [mm, conf] = sonar_field(m, sonar_fields_[i]);
      if (conf < min_confidence_ || mm <= 0.0)
      {
        ++low_confidence_;
        continue;
      }
      apply_sonar(i, mm / 1000.0);
    }
  }

  // Range: metres; +inf (or anything non-finite) means no echo
  void on_range(size_t i, const sensor_msgs::msg::Range &m)
  {
    if (!std::isfinite(m.range))
    {
      ++no_echo_;
      return;
    }
    apply_sonar(i, m.range);
  }

  void apply_sonar(size_t i, double range)
  {
    if (!have_imu_)
    {
      return; // wait until predict has a time base
    }

    const SonarResult r = ekf_.update_sonar(static_cast<int>(i), range);
    ++counts_[static_cast<int>(r)];

    if (r == SonarResult::Recovered)
    {
      RCLCPP_WARN(get_logger(), "sonar %zu gated %d times in a row, consistently: widened uncertainty to recover", i,
                  ekf_.params().gate_reset_count);
    }

    // the innovation is only computed when the update actually ran
    if (r == SonarResult::Used || r == SonarResult::Gated || r == SonarResult::Recovered)
    {
      const double y = ekf_.last_innovation(), S = ekf_.last_innovation_var();
      std_msgs::msg::Float64MultiArray d;
      d.data = {static_cast<double>(i), range, range - y, y, S, y * y / S, r == SonarResult::Gated ? 0.0 : 1.0};
      diag_pub_->publish(d);
    }
  }

  void publish(const rclcpp::Time &stamp)
  {
    const auto &x = ekf_.state();
    const auto &P = ekf_.cov();
    const double psi = x(EkfPlanar::PSI);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, psi);

    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = frame_id_;
    o.child_frame_id = child_frame_id_;
    o.pose.pose.position.x = x(EkfPlanar::PX) + origin_x_;
    o.pose.pose.position.y = x(EkfPlanar::PY) + origin_y_;
    o.pose.pose.position.z = out_z_;
    o.pose.pose.orientation.x = q.x();
    o.pose.pose.orientation.y = q.y();
    o.pose.pose.orientation.z = q.z();
    o.pose.pose.orientation.w = q.w();

    // pose covariance is 6x6 over (x, y, z, roll, pitch, yaw); fill the planar part
    const int pose_idx[3] = {0, 1, 5};
    const int ekf_idx[3] = {EkfPlanar::PX, EkfPlanar::PY, EkfPlanar::PSI};
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
      {
        o.pose.covariance[pose_idx[r] * 6 + pose_idx[c]] = P(ekf_idx[r], ekf_idx[c]);
      }
    }

    // Odometry twist is in the child (body) frame: rotate the pool velocity by -psi
    const double c = std::cos(psi), s = std::sin(psi);
    Eigen::Matrix2d Rt;
    Rt << c, s, -s, c;
    const Eigen::Vector2d vb = Rt * Eigen::Vector2d(x(EkfPlanar::VX), x(EkfPlanar::VY));
    const Eigen::Matrix2d Pv = Rt * P.block<2, 2>(EkfPlanar::VX, EkfPlanar::VX) * Rt.transpose();
    o.twist.twist.linear.x = vb(0);
    o.twist.twist.linear.y = vb(1);
    o.twist.covariance[0] = Pv(0, 0);
    o.twist.covariance[1] = o.twist.covariance[6] = Pv(0, 1);
    o.twist.covariance[7] = Pv(1, 1);
    odom_pub_->publish(o);

    if (tf_)
    {
      geometry_msgs::msg::TransformStamped tr;
      tr.header = o.header;
      tr.child_frame_id = child_frame_id_;
      tr.transform.translation.x = o.pose.pose.position.x;
      tr.transform.translation.y = o.pose.pose.position.y;
      tr.transform.translation.z = o.pose.pose.position.z;
      tr.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(tr);
    }
  }

  void log_stats()
  {
    const auto &x = ekf_.state();
    RCLCPP_INFO(get_logger(),
                "pool (%.2f, %.2f) psi %.1f deg bias %.3f deg/s | sonar used %d gated %d recovered %d "
                "no-wall %d oblique %d out-of-range %d low-conf %d no-echo %d",
                x(EkfPlanar::PX), x(EkfPlanar::PY), x(EkfPlanar::PSI) * 180 / M_PI,
                x(EkfPlanar::BG) * 180 / M_PI, counts_[static_cast<int>(SonarResult::Used)],
                counts_[static_cast<int>(SonarResult::Gated)], counts_[static_cast<int>(SonarResult::Recovered)],
                counts_[static_cast<int>(SonarResult::NoWall)],
                counts_[static_cast<int>(SonarResult::TooOblique)],
                counts_[static_cast<int>(SonarResult::OutOfRange)], low_confidence_, no_echo_);
    counts_.fill(0);
    low_confidence_ = 0;
    no_echo_ = 0;
  }

  std::vector<int> sonar_fields_; // filled by load_params(), so declared before ekf_
  EkfPlanar ekf_;

  std::string frame_id_, child_frame_id_;
  double origin_x_ = 0, origin_y_ = 0, out_z_ = 0;
  bool publish_tf_ = true, tilt_compensate_ = true;
  double min_confidence_ = 50, max_imu_gap_ = 0.5;

  bool have_imu_ = false;
  rclcpp::Time last_imu_;

  std::array<int, 6> counts_{}; // indexed by SonarResult
  int low_confidence_ = 0, no_echo_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sonar_msgs::msg::ThreeSonarDepth>::SharedPtr sonar_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  EkfPlanar::State x0_; // start state and covariance, for ~/reset
  EkfPlanar::Cov P0_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfPlanarNode>());
  rclcpp::shutdown();
  return 0;
}
