#include "a3_hardware_interface/socketcan_transport.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/bool.hpp>

#include "a3_hardware_interface/protocol_codec.hpp"

namespace a3_hardware_interface
{

namespace
{
constexpr double kDefaultKp = 80.0;
constexpr double kDefaultKd = 2.0;
constexpr double kDefaultEffortKd = 2.0;
constexpr double kDefaultFeedbackTimeoutS = 0.2;

double ParseDouble(const std::string & value, double fallback)
{
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return fallback;
  }
}

std::string GetParam(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}
}  // namespace

struct JointMapping
{
  std::string name;
  uint8_t motor_id{0};
  double direction{1.0};
  double position_offset{0.0};
  double torque_max{6.0};
  double speed_max{50.0};

  double hw_pos{0.0};
  double hw_vel{0.0};
  double hw_eff{0.0};
  double cmd_pos{0.0};
  double cmd_vel{0.0};
  double cmd_eff{0.0};

  bool has_feedback{false};
  bool stale{false};
  std::chrono::steady_clock::time_point last_fb_time{};
};

class A3MITHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    const auto & hp = info.hardware_parameters;
    can_interface_ = GetParam(hp, "can_interface", "can1");
    kp_ = ParseDouble(GetParam(hp, "kp", ""), kDefaultKp);
    kd_ = ParseDouble(GetParam(hp, "kd", ""), kDefaultKd);
    effort_kd_ = ParseDouble(GetParam(hp, "effort_kd", ""), kDefaultEffortKd);
    effort_kd_ = std::clamp(effort_kd_, 0.0, 5.0);
    feedback_timeout_s_ = ParseDouble(
      GetParam(hp, "feedback_timeout_s", ""), kDefaultFeedbackTimeoutS);
    feedback_timeout_s_ = std::clamp(feedback_timeout_s_, 0.02, 5.0);
    bus_ = (can_interface_ == "can0") ? CanBus::CAN0 : CanBus::CAN1;

    joints_.clear();
    for (const auto & joint : info.joints) {
      JointMapping j;
      j.name = joint.name;
      j.motor_id = static_cast<uint8_t>(
        std::stoi(GetParam(joint.parameters, "motor_id", "0")));
      j.direction = ParseDouble(GetParam(joint.parameters, "direction", "1.0"), 1.0);
      j.position_offset =
        ParseDouble(GetParam(joint.parameters, "position_offset", "0.0"), 0.0);

      // Default ranges by motor model: RS00 (id 1-3) ±14 Nm / ±33 rad/s,
      // EL05 (id 4-7) ±6 Nm / ±50 rad/s (LL-024).
      const bool is_rs00 = j.motor_id >= 1 && j.motor_id <= 3;
      j.torque_max = is_rs00 ? 14.0 : 6.0;
      j.speed_max = is_rs00 ? 33.0 : 50.0;
      j.torque_max = ParseDouble(
        GetParam(joint.parameters, "torque_max", ""), j.torque_max);
      j.speed_max = ParseDouble(
        GetParam(joint.parameters, "speed_max", ""), j.speed_max);

      // Seed state from URDF initial_value (L2 0.785 / L3 -0.785 = home).
      for (const auto & si : joint.state_interfaces) {
        if (si.name == "position") {
          j.hw_pos = ParseDouble(si.initial_value, 0.0);
        }
      }
      j.cmd_pos = j.hw_pos;
      joints_.push_back(std::move(j));
    }

    for (const auto & j : joints_) {
      if (j.motor_id == 0 || j.motor_id > 7) {
        RCLCPP_FATAL(
          rclcpp::get_logger(kLoggerName),
          "joint %s has invalid motor_id %u", j.name.c_str(), j.motor_id);
        return CallbackReturn::ERROR;
      }
      torque_max_by_motor_[j.motor_id] = j.torque_max;
      speed_max_by_motor_[j.motor_id] = j.speed_max;
    }

    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "configured: interface=%s kp=%.1f kd=%.2f fb_timeout=%.2fs joints=%zu",
      can_interface_.c_str(), kp_, kd_, feedback_timeout_s_, joints_.size());
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(joints_.size() * 3);
    for (auto & j : joints_) {
      interfaces.emplace_back(j.name, "position", &j.hw_pos);
      interfaces.emplace_back(j.name, "velocity", &j.hw_vel);
      interfaces.emplace_back(j.name, "effort", &j.hw_eff);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(joints_.size() * 3);
    for (auto & j : joints_) {
      interfaces.emplace_back(j.name, "position", &j.cmd_pos);
      interfaces.emplace_back(j.name, "velocity", &j.cmd_vel);
      interfaces.emplace_back(j.name, "effort", &j.cmd_eff);
    }
    return interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &,
    const std::vector<std::string> &) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    const auto contains_effort = [](const std::vector<std::string> & ifaces) {
      for (const auto & i : ifaces) {
        if (i.find("/effort") != std::string::npos) {
          return true;
        }
      }
      return false;
    };

    if (contains_effort(start_interfaces) && !effort_mode_.load()) {
      effort_mode_.store(true);
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "command mode -> EFFORT (gravity-comp free drive): kp=0 kd=%.2f",
        effort_kd_);
    }
    if (contains_effort(stop_interfaces) && effort_mode_.load()) {
      effort_mode_.store(false);
      // Re-anchor position commands at measured pose so activating a
      // position controller after free-drive cannot snap the arm back.
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (auto & j : joints_) {
        j.cmd_pos = j.hw_pos;
        j.cmd_vel = 0.0;
        j.cmd_eff = 0.0;
      }
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "command mode -> POSITION: kp=%.1f kd=%.2f, commands re-anchored",
        kp_, kd_);
    }
    return hardware_interface::return_type::OK;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::string error;
    if (!transport_.Open(can_interface_, &error)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLoggerName), "open %s failed: %s",
        can_interface_.c_str(), error.c_str());
      return CallbackReturn::ERROR;
    }

    health_node_ = std::make_shared<rclcpp::Node>("a3_hardware_health");
    rclcpp::QoS latched_qos{
      rclcpp::KeepLast(1)};
    latched_qos.reliable();
    latched_qos.transient_local();
    diag_pub_ = health_node_->create_publisher<
      diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", latched_qos);
    stale_pub_ = health_node_->create_publisher<std_msgs::msg::Bool>(
      "/a3/hardware/feedback_stale", latched_qos);
    health_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    health_exec_->add_node(health_node_);

    rx_run_.store(true);
    rx_thread_ = std::thread(&A3MITHardwareInterface::RxLoop, this);

    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName), "CAN %s open, RX thread running",
      can_interface_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // F51/F81 power sequence: reset ALL first (MIT reset leaves the motor in
    // disabled/coast), then prove every motor answers before enabling ANY.
    // ros2_control aborts controller_manager when on_activate returns ERROR,
    // so a dark motor must be discovered while all motors are still safely
    // reset — never after six have been enabled (LL-083).
    for (const auto & j : joints_) {
      transport_.Send(ProtocolCodec::BuildResetFrame(bus_, j.motor_id), nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      size_t seen = 0;
      {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        for (const auto & j : joints_) {
          if (j.has_feedback) {
            ++seen;
          }
        }
      }
      if (seen == joints_.size()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    size_t responding = 0;
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (const auto & j : joints_) {
        if (j.has_feedback) {
          ++responding;
        }
      }
    }
    if (responding != joints_.size()) {
      active_.store(false);
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "activate aborted: only %zu/%zu motors answered after reset; "
        "no enable frames sent, all motors left disabled",
        responding, joints_.size());
      return CallbackReturn::ERROR;
    }

    for (const auto & j : joints_) {
      transport_.Send(ProtocolCodec::BuildEnableFrame(bus_, j.motor_id), nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    size_t anchored = 0;
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (auto & j : joints_) {
        j.cmd_pos = j.hw_pos;
        j.cmd_vel = 0.0;
        j.cmd_eff = 0.0;
        ++anchored;
      }
    }

    active_.store(true);
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName), "activated, re-anchored %zu/%zu motors",
      anchored, joints_.size());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_.store(false);
    fb_stale_.store(false);

    // Zero-gain refresh at the last measured positions, then reset (MIT stop).
    for (int cycle = 0; cycle < 3; ++cycle) {
      std::vector<double> positions(joints_.size());
      {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        for (size_t i = 0; i < joints_.size(); ++i) {
          positions[i] = joints_[i].hw_pos;
        }
      }
      for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        const double motor_pos = j.direction * positions[i] + j.position_offset;
        const auto frame = ProtocolCodec::BuildMitControlFrame(
          bus_, j.motor_id, motor_pos, 0.0, 0.0, 0.0, 0.0,
          j.torque_max, j.speed_max);
        transport_.Send(frame, nullptr);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    for (const auto & j : joints_) {
      transport_.Send(ProtocolCodec::BuildResetFrame(bus_, j.motor_id), nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "deactivated, motors reset");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    active_.store(false);
    rx_run_.store(false);
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    transport_.Close();
    stale_pub_.reset();
    diag_pub_.reset();
    health_exec_.reset();
    health_node_.reset();
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_.load()) {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (auto & j : joints_) {
        j.stale = false;
      }
      fb_stale_.store(false);
      return hardware_interface::return_type::OK;
    }

    // F81: staleness must NEVER return ERROR — System::read() calls error() on
    // ERROR, which forces the component straight to unconfigured, silencing
    // write() (LL-083). Latch internally; write() runs the freeze-hold.
    const auto now = std::chrono::steady_clock::now();
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (auto & j : joints_) {
        j.stale = false;
        if (!j.has_feedback) {
          continue;
        }
        const double age_s =
          std::chrono::duration<double>(now - j.last_fb_time).count();
        if (age_s > feedback_timeout_s_) {
          stale = true;
          j.stale = true;
          RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger(kLoggerName), clock_, 1000,
            "feedback stale: motor=%u (%s) age=%.3fs timeout=%.2fs",
            j.motor_id, j.name.c_str(), age_s, feedback_timeout_s_);
        }
      }
    }
    fb_stale_.store(stale);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_.load()) {
      return hardware_interface::return_type::OK;
    }

    if (fb_stale_.load()) {
      // F81: discard controller commands and hold the last known pose so a
      // blind joint cannot keep advancing; other joints freeze with it.
      std::vector<double> motor_positions(joints_.size());
      {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        for (size_t i = 0; i < joints_.size(); ++i) {
          motor_positions[i] =
            joints_[i].direction * joints_[i].hw_pos + joints_[i].position_offset;
        }
      }
      for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        const auto frame = ProtocolCodec::BuildMitControlFrame(
          bus_, j.motor_id, motor_positions[i], 0.0, kp_, kd_, 0.0,
          j.torque_max, j.speed_max);
        std::string error;
        if (!transport_.Send(frame, &error)) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger(kLoggerName), clock_, 1000, "%s", error.c_str());
        }
      }
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger(kLoggerName), clock_, 1000,
        "protective freeze-hold active: feedback stale on >=1 motor");
      return hardware_interface::return_type::OK;
    }

    if (effort_mode_.load()) {
      // MIT torque mode: kp=0 removes the position spring; kd keeps joint
      // damping; torque_ff is the joint-space command mapped to the motor
      // axis (motor τ = joint τ / direction; directions are ±1). Position
      // field is the current measured motor angle (no position target).
      std::vector<double> motor_positions(joints_.size());
      {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        for (size_t i = 0; i < joints_.size(); ++i) {
          motor_positions[i] =
            joints_[i].direction * joints_[i].hw_pos + joints_[i].position_offset;
        }
      }
      for (size_t i = 0; i < joints_.size(); ++i) {
        const auto & j = joints_[i];
        const double motor_torque =
          std::clamp(j.cmd_eff, -j.torque_max, j.torque_max) * j.direction;
        const auto frame = ProtocolCodec::BuildMitControlFrame(
          bus_, j.motor_id, motor_positions[i], 0.0, 0.0, effort_kd_,
          motor_torque, j.torque_max, j.speed_max);
        std::string error;
        if (!transport_.Send(frame, &error)) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger(kLoggerName), clock_, 1000, "%s", error.c_str());
        }
      }
      return hardware_interface::return_type::OK;
    }

    for (const auto & j : joints_) {
      // MIT position mode: velocity/torque FF stay 0 — JTC position commands
      // already carry a smooth spline; kp/kd close the loop on the motor.
      const double motor_pos = j.direction * j.cmd_pos + j.position_offset;
      const auto frame = ProtocolCodec::BuildMitControlFrame(
        bus_, j.motor_id, motor_pos, 0.0, kp_, kd_, 0.0,
        j.torque_max, j.speed_max);
      std::string error;
      if (!transport_.Send(frame, &error)) {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName), clock_, 1000, "%s", error.c_str());
      }
    }
    return hardware_interface::return_type::OK;
  }

private:
  static constexpr const char * kLoggerName = "a3_mit_hardware_interface";

  void PublishHealth()
  {
    const auto now = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = health_node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "a3_hardware:feedback_watchdog";
    status.hardware_id = "a3";
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (const auto & j : joints_) {
        const double age = j.has_feedback
          ? std::chrono::duration<double>(now - j.last_fb_time).count()
          : -1.0;
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "motor" + std::to_string(j.motor_id) + "_age_s";
        kv.value = std::to_string(age);
        status.values.push_back(kv);
        if (j.stale) {
          stale = true;
        }
      }
    }
    status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                         : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = stale
      ? "feedback stale on >=1 motor; whole-arm freeze-hold engaged"
      : "all feedback channels healthy";
    arr.status.push_back(status);
    diag_pub_->publish(arr);
    std_msgs::msg::Bool b;
    b.data = stale;
    stale_pub_->publish(b);
  }

  void RxLoop()
  {
    auto last_health = std::chrono::steady_clock::now();
    while (rx_run_.load()) {
      CanFrameMessage frame;
      if (transport_.Receive(&frame)) {
        const uint8_t motor_id = static_cast<uint8_t>((frame.can_id >> 8) & 0xFF);
        if (motor_id != 0 && motor_id <= 7) {
          auto fb = ProtocolCodec::DecodeFeedback(
            frame, torque_max_by_motor_[motor_id],
            speed_max_by_motor_[motor_id]);
          if (fb) {
            const auto rx_now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(fb_mutex_);
            for (auto & j : joints_) {
              if (j.motor_id != fb->motor_id) {
                continue;
              }
              // motor_pos = direction*joint_pos + offset → invert.
              // Effort is NOT re-signed (motor torque axis, LL conventions).
              j.hw_pos = (fb->current_angle - j.position_offset) / j.direction;
              j.hw_vel = fb->current_speed / j.direction;
              j.hw_eff = fb->current_torque;
              j.has_feedback = true;
              j.last_fb_time = rx_now;
            }
          }
        }
      }
      health_exec_->spin_some(std::chrono::nanoseconds(0));
      const auto t_now = std::chrono::steady_clock::now();
      if (t_now - last_health >= std::chrono::milliseconds(100)) {
        last_health = t_now;
        PublishHealth();
      }
    }
  }

  rclcpp::Clock clock_;

  std::string can_interface_{"can1"};
  CanBus bus_{CanBus::CAN1};
  double kp_{kDefaultKp};
  double kd_{kDefaultKd};
  double effort_kd_{kDefaultEffortKd};
  double feedback_timeout_s_{kDefaultFeedbackTimeoutS};
  std::atomic_bool effort_mode_{false};
  std::atomic_bool fb_stale_{false};

  std::vector<JointMapping> joints_;
  std::array<double, 8> torque_max_by_motor_{};
  std::array<double, 8> speed_max_by_motor_{};

  SocketcanTransport transport_;
  std::thread rx_thread_;
  std::atomic_bool rx_run_{false};
  std::atomic_bool active_{false};
  std::mutex fb_mutex_;

  std::shared_ptr<rclcpp::Node> health_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> health_exec_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stale_pub_;
};

}  // namespace a3_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  a3_hardware_interface::A3MITHardwareInterface,
  hardware_interface::SystemInterface)
