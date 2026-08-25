#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{

double duration_to_sec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

}  // namespace

class GravityCompensationNode : public rclcpp::Node
{
public:
  GravityCompensationNode()
  : Node("gravity_compensation_node")
  {
    declare_parameter<std::string>("input_topic", "/a3/planned_joint_trajectory");
    declare_parameter<std::string>("output_topic", "/joint_group_effort_controller/joint_trajectory");
    declare_parameter<std::string>("urdf_package", "a3_description");
    declare_parameter<std::string>("urdf_relpath", "urdf/el_a3.urdf");
    declare_parameter<double>("gravity_ff_scale", 1.0);
    declare_parameter<int>("max_points", 32);

    const auto pkg = get_parameter("urdf_package").as_string();
    const auto rel = get_parameter("urdf_relpath").as_string();
    const auto urdf_path = ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;

    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    q_ = pinocchio::neutral(model_);

    const auto in_topic = get_parameter("input_topic").as_string();
    const auto out_topic = get_parameter("output_topic").as_string();
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(out_topic, 10);
    sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      in_topic, 10,
      std::bind(&GravityCompensationNode::on_traj, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "gravity_compensation_node: %s -> %s  urdf=%s  nq=%d  scale=%.3f  max_points=%ld",
      in_topic.c_str(), out_topic.c_str(), urdf_path.c_str(), model_.nq,
      get_parameter("gravity_ff_scale").as_double(),
      static_cast<long>(get_parameter("max_points").as_int()));
  }

private:
  int q_index_for_joint(const std::string & name) const
  {
    if (!model_.existJointName(name)) {
      return -1;
    }
    const pinocchio::JointIndex jid = model_.getJointId(name);
    if (jid < 1 || jid >= static_cast<pinocchio::JointIndex>(model_.njoints)) {
      return -1;
    }
    const int idx = model_.idx_qs[jid];
    const int nq_j = model_.nqs[jid];
    if (nq_j != 1 || idx < 0 || idx >= model_.nq) {
      return -1;
    }
    return idx;
  }

  std::vector<double> gravity_for_positions(
    const std::vector<std::string> & names, const std::vector<double> & positions)
  {
    q_ = pinocchio::neutral(model_);
    for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
      const int idx = q_index_for_joint(names[i]);
      if (idx >= 0) {
        q_[idx] = positions[i];
      }
    }
    pinocchio::computeGeneralizedGravity(model_, data_, q_);
    const double scale = get_parameter("gravity_ff_scale").as_double();
    std::vector<double> effort(names.size(), 0.0);
    for (size_t i = 0; i < names.size(); ++i) {
      const int idx = q_index_for_joint(names[i]);
      if (idx >= 0 && idx < data_.g.size()) {
        effort[i] = scale * data_.g[idx];
      }
    }
    return effort;
  }

  trajectory_msgs::msg::JointTrajectory downsample(
    const trajectory_msgs::msg::JointTrajectory & in) const
  {
    const int max_pts = std::max(2, static_cast<int>(get_parameter("max_points").as_int()));
    trajectory_msgs::msg::JointTrajectory out;
    out.header = in.header;
    out.joint_names = in.joint_names;
    const size_t n = in.points.size();
    if (n == 0) {
      return out;
    }
    if (static_cast<int>(n) <= max_pts) {
      out.points = in.points;
      return out;
    }
    out.points.reserve(static_cast<size_t>(max_pts));
    for (int k = 0; k < max_pts; ++k) {
      const size_t src = (k == max_pts - 1)
        ? (n - 1)
        : static_cast<size_t>(std::llround(static_cast<double>(k) * (n - 1) / (max_pts - 1)));
      out.points.push_back(in.points[src]);
    }
    return out;
  }

  void on_traj(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(get_logger(), "Empty trajectory, ignoring");
      return;
    }

    auto out = downsample(*msg);
    for (auto & pt : out.points) {
      pt.accelerations.clear();
      pt.effort = gravity_for_positions(out.joint_names, pt.positions);
    }

    double mag = 0.0;
    if (!out.points.empty() && out.points.front().effort.size() >= 3) {
      mag = std::hypot(out.points.front().effort[1], out.points.front().effort[2]);
    }
    RCLCPP_INFO(
      get_logger(),
      "Filled effort on %zu points (from %zu); |G L2/L3|~%.4f Nm  duration=%.3fs",
      out.points.size(), msg->points.size(), mag,
      duration_to_sec(out.points.back().time_from_start));
    pub_->publish(out);
  }

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompensationNode>());
  rclcpp::shutdown();
  return 0;
}
