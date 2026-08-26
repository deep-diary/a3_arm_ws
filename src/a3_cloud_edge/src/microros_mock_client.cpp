#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{

double duration_to_sec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

}  // namespace

class MicrorosMockClient : public rclcpp::Node
{
public:
  MicrorosMockClient()
  : Node("microros_mock_client")
  {
    declare_parameter<std::string>("trajectory_topic", "/joint_group_effort_controller/joint_trajectory");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<double>("joint_state_rate_hz", 50.0);
    declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint"});

    const auto trajectory_topic = get_parameter("trajectory_topic").as_string();
    const auto joint_states_topic = get_parameter("joint_states_topic").as_string();
    const auto rate_hz = get_parameter("joint_state_rate_hz").as_double();
    joint_names_ = get_parameter("joint_names").as_string_array();

    positions_.assign(joint_names_.size(), 0.0);
    velocities_.assign(joint_names_.size(), 0.0);

    traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic, rclcpp::QoS(10),
      std::bind(&MicrorosMockClient::on_trajectory, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);

    const auto period_ms = static_cast<int64_t>(1000.0 / std::max(rate_hz, 1.0));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&MicrorosMockClient::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "microros_mock_client ready (XRCE via RMW_IMPLEMENTATION=rmw_microxrcedds). "
      "sub=%s pub=%s @ %.1f Hz",
      trajectory_topic.c_str(), joint_states_topic.c_str(), rate_hz);
  }

private:
  void on_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty JointTrajectory, ignoring");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    active_traj_ = *msg;
    traj_start_ = now();
    has_traj_ = true;

    RCLCPP_INFO(
      get_logger(), "Trajectory received: %zu points, %zu joints",
      active_traj_.points.size(), active_traj_.joint_names.size());
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_traj_) {
      sample_trajectory(now());
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position = positions_;
    js.velocity = velocities_;
    joint_state_pub_->publish(js);
  }

  void sample_trajectory(const rclcpp::Time & t)
  {
    const double elapsed = (t - traj_start_).seconds();
    const auto & points = active_traj_.points;

    if (elapsed >= duration_to_sec(points.back().time_from_start)) {
      apply_point(points.back());
      has_traj_ = false;
      return;
    }

    size_t idx = 0;
    while (idx + 1 < points.size() &&
      elapsed > duration_to_sec(points[idx + 1].time_from_start))
    {
      ++idx;
    }

    if (idx + 1 >= points.size()) {
      apply_point(points.back());
      return;
    }

    const auto & p0 = points[idx];
    const auto & p1 = points[idx + 1];
    const double t0 = duration_to_sec(p0.time_from_start);
    const double t1 = duration_to_sec(p1.time_from_start);
    const double dt = t1 - t0;
    const double u = (dt > 1e-12) ? (elapsed - t0) / dt : 0.0;
    interpolate_fields(p0, p1, u, dt);
  }

  void apply_point(const trajectory_msgs::msg::JointTrajectoryPoint & pt)
  {
    interpolate_fields(pt, pt, 0.0, 1.0);
  }

  static double at_or(const std::vector<double> & v, size_t i, double fb = 0.0)
  {
    return i < v.size() ? v[i] : fb;
  }

  void interpolate_fields(
    const trajectory_msgs::msg::JointTrajectoryPoint & p0,
    const trajectory_msgs::msg::JointTrajectoryPoint & p1,
    double u,
    double dt)
  {
    const bool has_v = !p0.velocities.empty() && !p1.velocities.empty();
    const bool has_a = !p0.accelerations.empty() && !p1.accelerations.empty();
    const char * method = "linear";
    if (has_v && has_a) {
      method = "quintic";
    } else if (has_v) {
      method = "cubic";
    }

    const auto & names = active_traj_.joint_names;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      size_t src = i;
      if (!names.empty()) {
        const auto it = std::find(names.begin(), names.end(), joint_names_[i]);
        if (it == names.end()) {
          continue;
        }
        src = static_cast<size_t>(std::distance(names.begin(), it));
      }
      const double qp0 = at_or(p0.positions, src);
      const double qp1 = at_or(p1.positions, src, qp0);
      const double v0 = at_or(p0.velocities, src);
      const double v1 = at_or(p1.velocities, src);
      const double a0 = at_or(p0.accelerations, src);
      const double a1 = at_or(p1.accelerations, src);
      double p = qp0;
      double v = 0.0;
      const double uu = std::clamp(u, 0.0, 1.0);
      if (std::string(method) == "linear" || dt <= 1e-12) {
        p = qp0 + uu * (qp1 - qp0);
        v = (dt > 1e-12) ? (qp1 - qp0) / dt : 0.0;
      } else if (std::string(method) == "cubic") {
        const double u2 = uu * uu;
        const double u3 = u2 * uu;
        const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
        const double h10 = u3 - 2.0 * u2 + uu;
        const double h01 = -2.0 * u3 + 3.0 * u2;
        const double h11 = u3 - u2;
        p = h00 * qp0 + h10 * (dt * v0) + h01 * qp1 + h11 * (dt * v1);
        const double dh00 = 6.0 * u2 - 6.0 * uu;
        const double dh10 = 3.0 * u2 - 4.0 * uu + 1.0;
        const double dh01 = -6.0 * u2 + 6.0 * uu;
        const double dh11 = 3.0 * u2 - 2.0 * uu;
        v = (dh00 * qp0 + dh10 * (dt * v0) + dh01 * qp1 + dh11 * (dt * v1)) / dt;
      } else {
        const double u2 = uu * uu;
        const double u3 = u2 * uu;
        const double u4 = u3 * uu;
        const double u5 = u4 * uu;
        const double h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
        const double h1 = uu - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
        const double h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
        const double h3 = 0.5 * u3 - u4 + 0.5 * u5;
        const double h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;
        const double h5 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
        p = h0 * qp0 + h1 * (dt * v0) + h2 * (dt * dt * a0) +
          h3 * (dt * dt * a1) + h4 * (dt * v1) + h5 * qp1;
        const double dh0 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
        const double dh1 = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
        const double dh2 = uu - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
        const double dh3 = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;
        const double dh4 = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
        const double dh5 = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
        v = (dh0 * qp0 + dh1 * (dt * v0) + dh2 * (dt * dt * a0) +
          dh3 * (dt * dt * a1) + dh4 * (dt * v1) + dh5 * qp1) / dt;
      }
      positions_[i] = p;
      velocities_[i] = v;
    }
  }

  std::mutex mutex_;
  bool has_traj_{false};
  trajectory_msgs::msg::JointTrajectory active_traj_;
  rclcpp::Time traj_start_;
  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MicrorosMockClient>());
  rclcpp::shutdown();
  return 0;
}
