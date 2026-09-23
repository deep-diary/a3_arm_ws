// F91 motor maintenance node: SetZero (0x06) / SaveParam (0x16) for the
// product ros2_control stack. The SystemInterface plugin cannot host
// services, so maintenance runs as a standalone tool with an exclusive CAN
// socket, interlocked against active controllers. Modeled on the EDULITE_A3
// SDK (per-motor commands, 50 ms spacing).

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rclcpp/rclcpp.hpp"

#include "a3_hardware_interface/protocol_codec.hpp"
#include "a3_hardware_interface/socketcan_transport.hpp"
#include "a3_msgs/srv/motor_id_command.hpp"

namespace a3_hardware_interface
{

using namespace std::chrono_literals;
using controller_manager_msgs::srv::ListControllers;
using a3_msgs::srv::MotorIdCommand;

static constexpr uint8_t kBroadcastId = 0xFF;

class MotorMaintenanceNode : public rclcpp::Node
{
public:
  MotorMaintenanceNode()
  : rclcpp::Node("motor_maintenance")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can1");
    bus_ = (can_interface_ == "can0") ? CanBus::CAN0 : CanBus::CAN1;
    const auto ids = declare_parameter<std::vector<int64_t>>(
      "motor_ids", {1, 2, 3, 4, 5, 6, 7});
    motor_ids_.assign(ids.begin(), ids.end());
    inter_command_delay_ms_ = declare_parameter<int>(
      "inter_command_delay_ms", 50);
    enforce_interlock_ = declare_parameter<bool>(
      "enforce_controller_interlock", true);
    guarded_controllers_ = declare_parameter<std::vector<std::string>>(
      "guarded_controllers",
      {"arm_controller", "gripper_controller", "zero_torque_controller"});

    // Interlock probing runs from inside a service callback, so the client
    // lives on a separate node with its own executor — the node already
    // associated with the outer MultiThreadedExecutor cannot be spun
    // recursively in Humble.
    probe_node_ = std::make_shared<rclcpp::Node>(
      "motor_maintenance_cm_probe");
    cm_cli_ = probe_node_->create_client<ListControllers>(
      "/controller_manager/list_controllers");
    probe_exec_.add_node(probe_node_);

    set_zero_srv_ = create_service<MotorIdCommand>(
      "/a3/maintenance/set_zero",
      [this](
        const std::shared_ptr<MotorIdCommand::Request> req,
        std::shared_ptr<MotorIdCommand::Response> resp) {
        this->handle(
          req, resp,
          [this](uint8_t id) {
            return this->transport_.Send(
              ProtocolCodec::BuildSetZeroFrame(bus_, id), nullptr);
          },
          "set_zero");
      });
    save_param_srv_ = create_service<MotorIdCommand>(
      "/a3/maintenance/save_parameters",
      [this](
        const std::shared_ptr<MotorIdCommand::Request> req,
        std::shared_ptr<MotorIdCommand::Response> resp) {
        this->handle(
          req, resp,
          [this](uint8_t id) {
            return this->transport_.Send(
              ProtocolCodec::BuildSaveParamFrame(bus_, id), nullptr);
          },
          "save_parameters");
      });

    RCLCPP_INFO(
      get_logger(),
      "motor_maintenance ready on %s, %zu motor(s), interlock=%s",
      can_interface_.c_str(), motor_ids_.size(),
      enforce_interlock_ ? "on" : "off");
  }

private:
  using FrameSender = std::function<bool(uint8_t)>;

  void handle(
    const std::shared_ptr<MotorIdCommand::Request> req,
    const std::shared_ptr<MotorIdCommand::Response> resp,
    FrameSender send, const char * op_name)
  {
    std::vector<uint8_t> targets;
    if (req->motor_id == kBroadcastId) {
      targets = motor_ids_;
    } else if (
      std::find(motor_ids_.begin(), motor_ids_.end(), req->motor_id) !=
      motor_ids_.end())
    {
      targets.push_back(static_cast<uint8_t>(req->motor_id));
    } else {
      resp->success = false;
      resp->message = "motor_id out of configured range (1..7, 255=all)";
      RCLCPP_WARN(get_logger(), "%s rejected: motor_id=%u", op_name,
        req->motor_id);
      return;
    }

    if (enforce_interlock_) {
      const auto active = active_guarded_controllers();
      if (!active.empty()) {
        resp->success = false;
        resp->message =
          "refused: controllers active — stop the stack first: " +
          join(active, ",");
        RCLCPP_WARN(get_logger(), "%s refused, active: %s", op_name,
          join(active, ",").c_str());
        return;
      }
    }

    std::string error;
    if (!ensure_open(&error)) {
      resp->success = false;
      resp->message = "cannot open " + can_interface_ + ": " + error;
      return;
    }

    size_t sent = 0;
    for (size_t i = 0; i < targets.size(); ++i) {
      if (!send(targets[i])) {
        transport_.Close();
        resp->success = false;
        resp->message = "CAN send failed for motor " +
          std::to_string(targets[i]);
        return;
      }
      ++sent;
      if (i + 1 < targets.size()) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(inter_command_delay_ms_));
      }
    }

    resp->success = true;
    resp->message = op_name;
    resp->message += " sent to " + std::to_string(sent) + " motor(s)";
    RCLCPP_INFO(get_logger(), "%s ok, %zu frame(s)", op_name, sent);
  }

  std::vector<std::string> active_guarded_controllers()
  {
    std::vector<std::string> out;
    if (!cm_cli_->wait_for_service(300ms)) {
      return out;  // stack down — maintenance allowed
    }
    auto future = cm_cli_->async_send_request(
      std::make_shared<ListControllers::Request>());
    if (probe_exec_.spin_until_future_complete(future, 2s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(
        get_logger(), "list_controllers timed out; treating stack as down");
      return out;
    }
    const auto result = future.get();
    for (const auto & c : result->controller) {
      if (c.state == "active" &&
        std::find(
          guarded_controllers_.begin(), guarded_controllers_.end(), c.name) !=
        guarded_controllers_.end())
      {
        out.push_back(c.name);
      }
    }
    return out;
  }

  bool ensure_open(std::string * error)
  {
    if (transport_.IsOpen()) {
      return true;
    }
    return transport_.Open(can_interface_, error);
  }

  static std::string join(
    const std::vector<std::string> & items, const std::string & sep)
  {
    std::string out;
    for (size_t i = 0; i < items.size(); ++i) {
      if (i) {
        out += sep;
      }
      out += items[i];
    }
    return out;
  }

  std::string can_interface_;
  CanBus bus_{CanBus::CAN1};
  std::vector<uint8_t> motor_ids_;
  int inter_command_delay_ms_{50};
  bool enforce_interlock_{true};
  std::vector<std::string> guarded_controllers_;
  SocketcanTransport transport_;
  rclcpp::Node::SharedPtr probe_node_;
  rclcpp::executors::SingleThreadedExecutor probe_exec_;
  rclcpp::Client<ListControllers>::SharedPtr cm_cli_;
  rclcpp::Service<MotorIdCommand>::SharedPtr set_zero_srv_;
  rclcpp::Service<MotorIdCommand>::SharedPtr save_param_srv_;
};

}  // namespace a3_hardware_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node =
    std::make_shared<a3_hardware_interface::MotorMaintenanceNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
