// F95: on-demand, fully read-only self-test service for the A3 arm stack.
// Uses the standard self_test::TestRunner (diagnostic_msgs/srv/SelfTest).
// All state is cached by subscriptions/timers so task callbacks never block.

#include <chrono>
#include <deque>
#include <memory>
#include <climits>
#include <set>
#include <unistd.h>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "self_test/test_runner.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using controller_manager_msgs::srv::ListControllers;
using diagnostic_updater::DiagnosticStatusWrapper;

class A3SelfTestNode : public rclcpp::Node
{
public:
  A3SelfTestNode()
  : rclcpp::Node("a3_self_test")
  {
    min_js_rate_ = declare_parameter<double>("min_joint_states_rate", 40.0);
    expected_joints_ = declare_parameter<std::vector<std::string>>(
      "expected_joints",
      {"L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint",
        "L7_joint"});
    expected_controllers_ = declare_parameter<std::vector<std::string>>(
      "expected_controllers", {"arm_controller", "gripper_controller"});
    controllers_timeout_ =
      declare_parameter<double>("controllers_timeout", 5.0);

    char host_buf[HOST_NAME_MAX + 1] = {0};
    gethostname(host_buf, sizeof(host_buf) - 1);
    hostname_ = host_buf[0] != '\0' ? host_buf : "a3-arm";

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        const double now = now_seconds();
        js_stamps_.push_back(now);
        prune(js_stamps_, now, 1.0);
        for (const auto & name : msg->name) {
          seen_joints_.insert(name);
        }
      });

    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        const double now = now_seconds();
        for (const auto & status : msg->status) {
          if (status.level >= 2) {
            fault_stamps_.push_back({now, status.name, status.message});
          }
        }
        prune_faults(now);
      });

    cm_client_ = create_client<ListControllers>("/controller_manager/list_controllers");

    refresh_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() { on_timer(); });

    runner_ = std::make_shared<self_test::TestRunner>(
      get_node_base_interface(), get_node_services_interface(),
      get_node_logging_interface());
    runner_->setID(hostname_);
    runner_->add(
      "Connection", [this](DiagnosticStatusWrapper & s) { check_connection(s); });
    runner_->add(
      "Controllers",
      [this](DiagnosticStatusWrapper & s) { check_controllers(s); });
    runner_->add(
      "Faults", [this](DiagnosticStatusWrapper & s) { check_faults(s); });

    RCLCPP_INFO(
      get_logger(),
      "F95 a3_self_test ready: %s/self_test (min_joint_states_rate=%.1f)",
      get_name(), min_js_rate_);
  }

private:
  struct FaultEvent
  {
    double time;
    std::string name;
    std::string message;
  };

  double now_seconds() const
  {
    return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  }

  static void prune(std::deque<double> & stamps, double now, double window)
  {
    while (!stamps.empty() && stamps.front() < now - window) {
      stamps.pop_front();
    }
  }

  void prune_faults(double now)
  {
    while (!fault_stamps_.empty() && fault_stamps_.front().time < now - 2.0) {
      fault_stamps_.pop_front();
    }
  }

  void on_timer()
  {
    const double now = now_seconds();
    prune(js_stamps_, now, 1.0);
    prune_faults(now);
    // Names that have not been seen for a while cannot be removed by time
    // (messages carry no identity timestamps); stale names only make checks
    // more permissive toward present joints, which is harmless.

    if (cm_request_pending_ || !cm_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<ListControllers::Request>();
    cm_request_pending_ = true;
    cm_client_->async_send_request(
      request,
      [this](rclcpp::Client<ListControllers>::SharedFuture future) {
        cm_request_pending_ = false;
        try {
          cm_response_ = future.get();
          cm_response_time_ = now_seconds();
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "list_controllers failed: %s", e.what());
        }
      });
  }

  void check_connection(DiagnosticStatusWrapper & s)
  {
    const double now = now_seconds();
    prune(js_stamps_, now, 1.0);
    const double rate = static_cast<double>(js_stamps_.size());
    s.add("joint_states_rate_hz", rate);

    std::vector<std::string> missing;
    for (const auto & expected : expected_joints_) {
      if (seen_joints_.find(expected) == seen_joints_.end()) {
        missing.push_back(expected);
      }
    }

    if (rate + 1e-6 < min_js_rate_ || !missing.empty()) {
      s.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        rate + 1e-6 < min_js_rate_
          ? "joint_states rate below minimum"
          : "missing expected joints");
    } else {
      s.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "joint_state_broadcaster feed healthy");
    }
    if (!missing.empty()) {
      std::string list;
      for (const auto & name : missing) {
        list += name + ",";
      }
      list.pop_back();
      s.add("missing_joints", list);
    }
  }

  void check_controllers(DiagnosticStatusWrapper & s)
  {
    const double now = now_seconds();
    if (!cm_response_ || now - cm_response_time_ > controllers_timeout_) {
      s.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "controller_manager not responding");
      return;
    }

    std::set<std::string> present;
    for (const auto & c : cm_response_->controller) {
      present.insert(c.name);
      s.add("controller:" + c.name, c.state);
    }

    bool ok = true;
    std::string message;

    const bool jsb_active = present.count("joint_state_broadcaster") > 0 &&
      controller_state("joint_state_broadcaster") == "active";
    if (!jsb_active) {
      ok = false;
      message += "joint_state_broadcaster not active; ";
    }

    for (const auto & expected : expected_controllers_) {
      const auto it = present.find(expected);
      if (it == present.end()) {
        ok = false;
        message += expected + " missing; ";
        continue;
      }
      const std::string state = controller_state(expected);
      // Humble controller lifecycle: inactive(已 configure 未 activate)/active；
      // "configured" 仅旧版 controller_manager 出现，一并接受。
      if (state != "inactive" && state != "configured" && state != "active") {
        ok = false;
        message += expected + " state=" + state + "; ";
      }
    }

    if (ok) {
      s.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "expected controllers loaded (inactive/configured or active)");
    } else {
      if (!message.empty() && message.back() == ' ') {
        message.pop_back();
        message.pop_back();
      }
      s.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    }
  }

  std::string controller_state(const std::string & name) const
  {
    for (const auto & c : cm_response_->controller) {
      if (c.name == name) {
        return c.state;
      }
    }
    return "";
  }

  void check_faults(DiagnosticStatusWrapper & s)
  {
    const double now = now_seconds();
    prune_faults(now);
    s.add("fault_events_last_2s", static_cast<int>(fault_stamps_.size()));
    if (fault_stamps_.empty()) {
      s.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "no ERROR-level diagnostics in last 2s (absence of diagnostics is "
        "not a failure)");
      return;
    }
    std::string message = fault_stamps_.back().name + ": " +
      fault_stamps_.back().message;
    s.add("latest_fault", message);
    s.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
  }

  double min_js_rate_;
  std::vector<std::string> expected_joints_;
  std::vector<std::string> expected_controllers_;
  double controllers_timeout_;
  std::string hostname_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diag_sub_;
  rclcpp::Client<ListControllers>::SharedPtr cm_client_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  std::shared_ptr<self_test::TestRunner> runner_;

  std::deque<double> js_stamps_;
  std::set<std::string> seen_joints_;
  std::deque<FaultEvent> fault_stamps_;
  ListControllers::Response::SharedPtr cm_response_;
  double cm_response_time_ = -100.0;
  bool cm_request_pending_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A3SelfTestNode>());
  rclcpp::shutdown();
  return 0;
}
