#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

namespace a3_hardware_interface
{

class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("urdf_path", "");
    auto_declare<std::string>("controller_manager_name", "controller_manager");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration()
  const override
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
      conf.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration()
  const override
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
      conf.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      conf.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return conf;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "parameter 'joints' is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    std::string urdf = get_node()->get_parameter("robot_description").as_string();
    if (urdf.empty()) {
      const std::string urdf_path =
        get_node()->get_parameter("urdf_path").as_string();
      if (!urdf_path.empty()) {
        urdf = ReadFile(urdf_path);
      }
    }
    if (urdf.empty()) {
      urdf = FetchRobotDescription();
    }

    if (urdf.empty()) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "no robot_description (param/urdf_path/controller_manager) available");
      return controller_interface::CallbackReturn::ERROR;
    }

    try {
      pinocchio::urdf::buildModelFromXML(urdf, model_);
      data_ = pinocchio::Data(model_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "pinocchio model build failed: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    q_.resize(model_.nq);
    q_.setZero();
    v_zero_ = Eigen::VectorXd::Zero(model_.nv);
    a_zero_ = Eigen::VectorXd::Zero(model_.nv);

    q_index_.resize(joint_names_.size());
    v_index_.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!model_.existJointName(joint_names_[i])) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "pinocchio model has no joint '%s'",
          joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      const pinocchio::JointIndex jid = model_.getJointId(joint_names_[i]);
      q_index_[i] = model_.joints[jid].idx_q();
      v_index_[i] = model_.joints[jid].idx_v();
    }

    gravity_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "~/gravity_torque", 10);

    RCLCPP_INFO(
      get_node()->get_logger(),
      "configured: %zu joints, pinocchio nq=%d (free-drive gravity compensation)",
      joint_names_.size(), model_.nq);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "gravity-comp free drive ACTIVATED: arm follows the operator, effort=gravity");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    for (auto & cmd : command_interfaces_) {
      cmd.set_value(0.0);
    }
    RCLCPP_INFO(get_node()->get_logger(), "free drive DEACTIVATED");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    q_.setZero();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      q_[q_index_[i]] =
        LookupState(joint_names_[i], hardware_interface::HW_IF_POSITION);
    }

    Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q_, v_zero_, a_zero_);

    // command_interfaces_ order is not guaranteed to match joint_names_
    // (ResourceManager returns handles in its own claim order): map by name.
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      LookupCommand(joint_names_[i]).set_value(tau[v_index_[i]]);
    }

    if (++pub_divider_ >= 10) {
      pub_divider_ = 0;
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = time;
      msg.name = joint_names_;
      msg.position.resize(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        msg.position[i] = q_[q_index_[i]];
      }
      msg.effort.resize(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        msg.effort[i] = tau[v_index_[i]];
      }
      gravity_pub_->publish(msg);
    }
    return controller_interface::return_type::OK;
  }

private:
  static std::string ReadFile(const std::string & path)
  {
    std::ifstream f(path);
    if (!f.is_open()) {
      return "";
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  std::string FetchRobotDescription()
  {
    const std::string cm_name =
      get_node()->get_parameter("controller_manager_name").as_string();
    const std::string ns = get_node()->get_namespace();
    const std::string remote =
      (ns == "/" ? std::string("/") : ns + "/") + cm_name;
    // The controller node is already owned by the controller_manager executor,
    // so SyncParametersClient's internal executor cannot spin it: fetch through
    // a standalone temporary node instead.
    auto temp_node = std::make_shared<rclcpp::Node>("gravity_comp_urdf_fetch");
    auto client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, remote);
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_node()->get_logger(),
        "FetchRobotDescription: parameter service %s unavailable", remote.c_str());
      return "";
    }
    try {
      auto params = client->get_parameters({"robot_description"});
      if (!params.empty() && params[0].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        return params[0].as_string();
      }
      RCLCPP_WARN(get_node()->get_logger(),
        "FetchRobotDescription: robot_description not set on %s", remote.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_node()->get_logger(),
        "FetchRobotDescription failed: %s", e.what());
    }
    return "";
  }

  double LookupState(const std::string & joint, const std::string & iface)
  {
    // In Humble's controller API get_name() is the full "joint/interface" name.
    const std::string full = joint + "/" + iface;
    for (const auto & s : state_interfaces_) {
      if (s.get_name() == full) {
        return s.get_value();
      }
    }
    RCLCPP_ERROR(
      get_node()->get_logger(), "state interface '%s' not found", full.c_str());
    return 0.0;
  }

  hardware_interface::LoanedCommandInterface & LookupCommand(const std::string & joint)
  {
    for (auto & c : command_interfaces_) {
      if (c.get_prefix_name() == joint) {
        return c;
      }
    }
    throw std::runtime_error("effort command interface for joint " + joint + " not found");
  }

  std::vector<std::string> joint_names_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_zero_;
  Eigen::VectorXd a_zero_;
  std::vector<Eigen::Index> q_index_;
  std::vector<Eigen::Index> v_index_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_pub_;
  int pub_divider_{0};
};

}  // namespace a3_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  a3_hardware_interface::GravityCompensationController,
  controller_interface::ControllerInterface)
