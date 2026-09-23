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
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>

#include <a3_can_bridge/msg/motor_state.hpp>
#include <a3_can_bridge/msg/motor_states.hpp>
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
constexpr double kDefaultKdMin = 0.001;
constexpr double kDefaultKdMax = 0.15;
constexpr double kDefaultKdVelocityRef = 1.0;
constexpr double kDefaultKdSmoothingAlpha = 0.15;
constexpr double kDefaultFeedbackTimeoutS = 0.2;
constexpr double kDefaultStartupKd = 4.0;
constexpr int kDefaultSoftStartCycles = 10;
constexpr double kDefaultTempWarnC = 90.0;
constexpr double kDefaultTempProtectC = 95.0;
constexpr double kDefaultMotorStatesRateHz = 50.0;
constexpr bool kDefaultMotorCanTimeoutEnabled = true;
constexpr double kDefaultMotorCanTimeoutS = 0.2;
// 0x7028 is uint32 with ~50 us per count (20000 ≈ 1 s)
constexpr double kCanTimeoutCountsPerSec = 20000.0;

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

bool ParseBool(const std::string & value, bool fallback)
{
  if (value.empty()) {
    return fallback;
  }
  std::string s;
  for (char c : value) {
    s.push_back(static_cast<char>(std::tolower(c)));
  }
  if (s == "true" || s == "1" || s == "yes" || s == "on") {
    return true;
  }
  if (s == "false" || s == "0" || s == "no" || s == "off") {
    return false;
  }
  return fallback;
}

int ParseInt(const std::string & value, int fallback)
{
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stoi(value);
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

std::string ExpandHome(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  if (const char * home = std::getenv("HOME"); home != nullptr) {
    return std::string(home) + path.substr(1);
  }
  return path;
}

// F49 key -> URDF link carrying the fitted mass/CoM; must stay identical to
// gravity_compensation_controller.cpp so FF uses the calibrated model.
const std::unordered_map<std::string, std::string> kF49JointToLink = {
  {"L2", "l2_l3_urdf_asm"},
  {"L3", "l3_lnik_urdf_asm"},
  {"L4", "l4_l5_urdf_asm"},
  {"L5", "part_9"},
  {"L6", "l5_l6_urdf_asm"},
};

std::array<uint8_t, 4> TimeoutCountsRaw(uint32_t counts)
{
  return {
    static_cast<uint8_t>(counts & 0xFF),
    static_cast<uint8_t>((counts >> 8) & 0xFF),
    static_cast<uint8_t>((counts >> 16) & 0xFF),
    static_cast<uint8_t>((counts >> 24) & 0xFF)};
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
  double hw_temp{0.0};
  uint8_t hw_mode{0};
  uint8_t hw_fault{0};
  // Firmware fault words are latched conditions: once a fresh frame reports a
  // nonzero word, keep health ERROR even after feedback traffic stops (MIT
  // firmware only emits feedback after command frames, e.g. after emergency
  // deactivate). Cleared by the clear-fault choreography in on_activate.
  bool fault_latched{false};
  // F85 per-joint adaptive-Kd overrides (<0 = use global) and EMA state.
  double kd_min_override{-1.0};
  double kd_max_override{-1.0};
  double adaptive_kd{kDefaultKdMax};
  double cmd_pos{0.0};
  double cmd_vel{0.0};
  double cmd_eff{0.0};
  // Per-joint command mode: true when an effort-interface controller claims
  // this joint (e.g. GripperActionController on L7), false for position JTC.
  bool effort_mode{false};

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
    // F85: fixed fallback is zero_torque_kd (xacro), with the legacy
    // effort_kd name as a secondary key.
    effort_kd_ = ParseDouble(
      GetParam(hp, "zero_torque_kd", GetParam(hp, "effort_kd", "")),
      kDefaultEffortKd);
    effort_kd_ = std::clamp(effort_kd_, 0.0, 5.0);
    adaptive_kd_enabled_ = ParseBool(GetParam(hp, "adaptive_kd_enabled", ""), false);
    kd_min_ = ParseDouble(GetParam(hp, "zero_torque_kd_min", ""), kDefaultKdMin);
    kd_max_ = ParseDouble(GetParam(hp, "zero_torque_kd_max", ""), kDefaultKdMax);
    kd_min_ = std::clamp(kd_min_, 0.0, 5.0);
    kd_max_ = std::clamp(kd_max_, 0.0, 5.0);
    if (kd_min_ > kd_max_) {
      std::swap(kd_min_, kd_max_);
    }
    kd_velocity_ref_ = ParseDouble(
      GetParam(hp, "kd_velocity_ref", ""), kDefaultKdVelocityRef);
    kd_velocity_ref_ = std::max(kd_velocity_ref_, 1e-3);
    kd_smoothing_alpha_ = ParseDouble(
      GetParam(hp, "kd_smoothing_alpha", ""), kDefaultKdSmoothingAlpha);
    kd_smoothing_alpha_ = std::clamp(kd_smoothing_alpha_, 1e-3, 1.0);
    motor_can_timeout_enabled_ =
      ParseBool(GetParam(hp, "motor_can_timeout_enabled", ""), kDefaultMotorCanTimeoutEnabled);
    motor_can_timeout_s_ = ParseDouble(
      GetParam(hp, "motor_can_timeout_s", ""), kDefaultMotorCanTimeoutS);
    motor_can_timeout_s_ = std::clamp(motor_can_timeout_s_, 0.05, 10.0);
    feedback_timeout_s_ = ParseDouble(
      GetParam(hp, "feedback_timeout_s", ""), kDefaultFeedbackTimeoutS);
    feedback_timeout_s_ = std::clamp(feedback_timeout_s_, 0.02, 5.0);
    startup_kd_ = ParseDouble(GetParam(hp, "startup_kd", ""), kDefaultStartupKd);
    startup_kd_ = std::clamp(startup_kd_, 0.0, 5.0);
    soft_start_cycles_ =
      ParseInt(GetParam(hp, "soft_start_cycles", ""), kDefaultSoftStartCycles);
    soft_start_cycles_ = std::clamp(soft_start_cycles_, 0, 200);
    temp_warn_c_ = ParseDouble(GetParam(hp, "temp_warn_c", ""), kDefaultTempWarnC);
    temp_warn_c_ = std::clamp(temp_warn_c_, 40.0, 150.0);
    temp_protect_c_ =
      ParseDouble(GetParam(hp, "temp_protect_c", ""), kDefaultTempProtectC);
    temp_protect_c_ = std::clamp(temp_protect_c_, temp_warn_c_, 150.0);
    motor_states_topic_ = GetParam(hp, "motor_states_topic", "/a3/motor/states");
    motor_states_rate_hz_ = ParseDouble(
      GetParam(hp, "motor_states_rate_hz", ""), kDefaultMotorStatesRateHz);
    motor_states_rate_hz_ = std::clamp(motor_states_rate_hz_, 1.0, 200.0);
    // F108: position-mode gravity feedforward (EDULITE gravity_feedforward_ratio).
    gravity_ff_ratio_ = ParseDouble(
      GetParam(hp, "gravity_feedforward_ratio", ""), 1.0);
    gravity_ff_ratio_ = std::clamp(gravity_ff_ratio_, 0.0, 1.0);
    use_pinocchio_gravity_ =
      ParseBool(GetParam(hp, "use_pinocchio_gravity", ""), true);
    use_calibrated_inertia_ =
      ParseBool(GetParam(hp, "use_calibrated_inertia", ""), true);
    inertia_params_file_ = GetParam(hp, "inertia_config_path",
      GetParam(hp, "inertia_params_file", ""));
    gravity_scales_file_ =
      GetParam(hp, "gravity_scales_file", ExpandHome("~/.a3/gravity_scales.yaml"));
    robot_description_ = GetParam(hp, "robot_description", "");
    urdf_path_ = GetParam(hp, "urdf_path", "");
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

      // F85 per-joint adaptive-Kd min/max overrides (declared on L4-L7).
      j.kd_min_override = ParseDouble(
        GetParam(joint.parameters, "zero_torque_kd_min", ""), -1.0);
      j.kd_max_override = ParseDouble(
        GetParam(joint.parameters, "zero_torque_kd_max", ""), -1.0);
      j.adaptive_kd = (j.kd_max_override > 0.0)
        ? j.kd_max_override : kd_max_;

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
    interfaces.reserve(joints_.size() * 4);
    for (auto & j : joints_) {
      interfaces.emplace_back(j.name, "position", &j.hw_pos);
      interfaces.emplace_back(j.name, "velocity", &j.hw_vel);
      interfaces.emplace_back(j.name, "effort", &j.hw_eff);
      interfaces.emplace_back(j.name, "temperature", &j.hw_temp);
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
    const auto split = [](const std::string & iface) {
      const auto slash = iface.find('/');
      return std::make_pair(
        iface.substr(0, slash),
        slash != std::string::npos ? iface.substr(slash + 1) : std::string());
    };
    const auto find_joint = [this](const std::string & name) -> JointMapping * {
      for (auto & j : joints_) {
        if (j.name == name) {
          return &j;
        }
      }
      return nullptr;
    };

    // Modes are per-joint: zero_torque claims effort on all 7, but
    // GripperActionController claims effort only on L7 while the arm JTC
    // keeps position on L1-L6.
    std::vector<std::string> started;
    for (const auto & iface : start_interfaces) {
      const auto [joint_name, iface_name] = split(iface);
      JointMapping * j = find_joint(joint_name);
      if (j == nullptr || iface_name != "effort" || j->effort_mode) {
        continue;
      }
      j->effort_mode = true;
      // Seed EMA state at the per-joint max so damping cannot collapse on
      // entry (matches el_a3_hardware adaptive_kd_values_ init).
      std::lock_guard<std::mutex> lock(fb_mutex_);
      j->adaptive_kd = (j->kd_max_override > 0.0) ? j->kd_max_override : kd_max_;
      started.push_back(joint_name);
    }
    if (!started.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "command mode -> EFFORT on %zu joint(s): kp=0 %s", started.size(),
        adaptive_kd_enabled_
          ? "velocity-adaptive Kd (Lorentzian + EMA)"
          : ("fixed kd=" + std::to_string(effort_kd_)).c_str());
    }

    std::vector<std::string> stopped;
    for (const auto & iface : stop_interfaces) {
      const auto [joint_name, iface_name] = split(iface);
      JointMapping * j = find_joint(joint_name);
      if (j == nullptr || iface_name != "effort" || !j->effort_mode) {
        continue;
      }
      j->effort_mode = false;
      // Re-anchor position commands at measured pose so activating a
      // position controller after free-drive cannot snap the joint back.
      std::lock_guard<std::mutex> lock(fb_mutex_);
      j->cmd_pos = j->hw_pos;
      j->cmd_vel = 0.0;
      j->cmd_eff = 0.0;
      stopped.push_back(joint_name);
    }
    if (!stopped.empty()) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "command mode -> POSITION on %zu joint(s): kp=%.1f kd=%.2f, commands re-anchored",
        stopped.size(), kp_, kd_);
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
    rclcpp::QoS states_qos{rclcpp::KeepLast(10)};
    states_qos.best_effort();
    motor_states_pub_ = health_node_->create_publisher<
      a3_can_bridge::msg::MotorStates>(motor_states_topic_, states_qos);
    health_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    health_exec_->add_node(health_node_);

    // F108: ratio tunable online (EDULITE semantics) — ramp the feedforward
    // up during commissioning without restarting controller_manager.
    health_node_->declare_parameter(
      "gravity_feedforward_ratio", gravity_ff_ratio_);
    health_node_->declare_parameter(
      "use_pinocchio_gravity", use_pinocchio_gravity_);
    // Must retain the returned handle: rclcpp stores it as a weak_ptr, so a
    // dropped handle makes the parameter service accept updates while the
    // callback (and these member writes) never runs. LL-126.
    on_set_parameters_handle_ = health_node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "gravity_feedforward_ratio" &&
            p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            gravity_ff_ratio_ = std::clamp(p.as_double(), 0.0, 1.0);
          } else if (p.get_name() == "use_pinocchio_gravity" &&
            p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
          {
            use_pinocchio_gravity_ = p.as_bool();
          }
        }
        return result;
      });

    motor_states_timer_ = health_node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / motor_states_rate_hz_)),
      [this] { PublishMotorStates(); });

    rx_run_.store(true);
    rx_thread_ = std::thread(&A3MITHardwareInterface::RxLoop, this);

    InitGravityModel();

    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName), "CAN %s open, RX thread running",
      can_interface_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // F51/F81 power sequence: reset ALL first (MIT reset leaves the motor in
    // disabled/coast), then prove every motor answers before enabling ANY.
    // A dark motor must be discovered while all motors are still safely
    // reset — never after six have been enabled (LL-083). Hardware starts
    // INACTIVE (hardware_components_initial_state, LL-086), so this runs
    // exactly when the operator enables, never at controller_manager boot.
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (auto & j : joints_) {
        j.fault_latched = false;
      }
    }
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
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "activate aborted: only %zu/%zu motors answered after reset; "
        "no enable frames sent, all motors left disabled",
        responding, joints_.size());
      return CallbackReturn::ERROR;
    }

    // F83 vendor-standard choreography (EDULITE_A3 EnableArm), per motor:
    // clear latched faults (Type 4, data[0]=1) → integer RUN_MODE write
    // (Type 18, 0x7005=0 MOTION_CONTROL; float encoding corrupts the uint8)
    // → F87 arm the firmware torque limit (0x700B float Nm per joint)
    // → F86 arm the motor-side CAN timeout (0x7028 uint32 counts; 0 disarms)
    // → enable (Type 3); 30 ms settling as in the vendor SDK.
    const uint32_t can_timeout_counts = motor_can_timeout_enabled_
      ? static_cast<uint32_t>(
          std::llround(motor_can_timeout_s_ * kCanTimeoutCountsPerSec))
      : 0u;
    for (const auto & j : joints_) {
      transport_.Send(ProtocolCodec::BuildClearFaultFrame(bus_, j.motor_id), nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      transport_.Send(
        ProtocolCodec::BuildSetParamU8Frame(
          bus_, j.motor_id, ProtocolCodec::kParamRunMode, 0),
        nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      // F87: 0x700B is float32 Nm (protocol manual); re-armed every activate
      // because the write is volatile.
      transport_.Send(
        ProtocolCodec::BuildSetParamFrame(
          bus_, j.motor_id, ProtocolCodec::kParamLimitTorque, j.torque_max),
        nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      transport_.Send(
        ProtocolCodec::BuildSetParamRawFrame(
          bus_, j.motor_id, ProtocolCodec::kParamCanTimeout,
          TimeoutCountsRaw(can_timeout_counts)),
        nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      transport_.Send(ProtocolCodec::BuildEnableFrame(bus_, j.motor_id), nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

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

    // MIT firmware only emits feedback after a command frame; by now motor 1
    // last answered ~0.6 s ago (its own enable step). Without fresh feedback
    // the first read() would fire the stale freeze-hold instead of soft-start.
    // Send the first soft-start damping round here and wait for the replies,
    // so the RT write loop picks up with feedback proven on every motor.
    if (soft_start_cycles_ > 0) {
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
        transport_.Send(
          ProtocolCodec::BuildMitControlFrame(
            bus_, j.motor_id, motor_positions[i], 0.0, 0.0, startup_kd_, 0.0,
            j.torque_max, j.speed_max),
          nullptr);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      soft_start_remaining_ = soft_start_cycles_ - 1;
    }

    active_.store(true);
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "vendor enable choreography complete, re-anchored %zu/%zu motors, "
      "soft-start %d cycles (kp=0 kd=%.1f)",
      anchored, joints_.size(), soft_start_cycles_, startup_kd_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_.store(false);
    fb_stale_.store(false);
    soft_start_remaining_ = 0;

    // F86: disarm the motor-side timeout before anything else, so no motor
    // can self-reset mid-shutdown while frames briefly pause.
    for (const auto & j : joints_) {
      transport_.Send(
        ProtocolCodec::BuildSetParamRawFrame(
          bus_, j.motor_id, ProtocolCodec::kParamCanTimeout,
          TimeoutCountsRaw(0u)),
        nullptr);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

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
    motor_states_timer_.reset();
    motor_states_pub_.reset();
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

    if (soft_start_remaining_ > 0) {
      // F83: pure-damping take-up at the measured pose — no position spring —
      // then hand off to normal stiffness. Our hardening; vendor startup_kd
      // is declared but never applied in EDULITE_A3.
      --soft_start_remaining_;
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
          bus_, j.motor_id, motor_positions[i], 0.0, 0.0, startup_kd_, 0.0,
          j.torque_max, j.speed_max);
        std::string error;
        if (!transport_.Send(frame, &error)) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger(kLoggerName), clock_, 1000, "%s", error.c_str());
        }
      }
      return hardware_interface::return_type::OK;
    }

    // Per-joint modes: effort-claimed joints (zero_torque, or GripperAction
    // on L7) get kp=0 + torque_ff; the rest get position frames.
    const bool gravity_ff_active = gravity_ff_ready_ && use_pinocchio_gravity_ &&
      gravity_ff_ratio_ > 0.0;
    std::vector<double> motor_positions(joints_.size());
    Eigen::VectorXd gravity_tau;
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      for (size_t i = 0; i < joints_.size(); ++i) {
        motor_positions[i] =
          joints_[i].direction * joints_[i].hw_pos + joints_[i].position_offset;
        if (joints_[i].effort_mode && adaptive_kd_enabled_) {
          // F85 adaptive damping (Lorentzian + EMA).
          const double kd_min = (joints_[i].kd_min_override > 0.0)
            ? joints_[i].kd_min_override : kd_min_;
          const double kd_max = (joints_[i].kd_max_override > 0.0)
            ? joints_[i].kd_max_override : kd_max_;
          const double ratio = std::abs(joints_[i].hw_vel) / kd_velocity_ref_;
          const double kd_raw = kd_min + (kd_max - kd_min) / (1.0 + ratio * ratio);
          joints_[i].adaptive_kd =
            std::clamp(kd_smoothing_alpha_ * kd_raw +
              (1.0 - kd_smoothing_alpha_) * joints_[i].adaptive_kd,
              0.0, 5.0);
        }
      }
      if (gravity_ff_active) {
        q_.setZero();
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (q_index_[i] >= 0) {
            q_[q_index_[i]] = joints_[i].hw_pos;
          }
        }
        gravity_tau = pinocchio::rnea(model_, data_, q_, v_zero_, a_zero_);
      }
    }

    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto & j = joints_[i];
      CanFrameMessage frame;
      if (j.effort_mode) {
        // MIT torque mode: kp=0 removes the position spring; kd keeps joint
        // damping; torque_ff is the joint-space command mapped to the motor
        // axis (motor τ = joint τ / direction; directions are ±1). Position
        // field is the current measured motor angle (no position target).
        const double kd = adaptive_kd_enabled_ ? j.adaptive_kd : effort_kd_;
        const double motor_torque =
          std::clamp(j.cmd_eff, -j.torque_max, j.torque_max) * j.direction;
        frame = ProtocolCodec::BuildMitControlFrame(
          bus_, j.motor_id, motor_positions[i], 0.0, 0.0, kd,
          motor_torque, j.torque_max, j.speed_max);
      } else {
        // MIT position mode: kp/kd close the loop on the motor; F108 adds the
        // static gravity torque as t_ff so joints do not sag under their own
        // weight (EDULITE gravity_feedforward_ratio).
        double t_ff = 0.0;
        if (gravity_ff_active && v_index_[i] >= 0) {
          t_ff = std::clamp(
              gravity_ff_ratio_ * ff_scale_[i] * gravity_tau[v_index_[i]],
              -j.torque_max, j.torque_max) * j.direction;
        }
        frame = ProtocolCodec::BuildMitControlFrame(
          bus_, j.motor_id, j.direction * j.cmd_pos + j.position_offset,
          0.0, kp_, kd_, t_ff, j.torque_max, j.speed_max);
      }
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

  // F108: builds the gravity model once at configure. Failure never aborts
  // hardware bring-up — the arm simply runs without feedforward.
  void InitGravityModel()
  {
    gravity_ff_ready_ = false;
    if (!use_pinocchio_gravity_) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "F108 gravity feedforward disabled: use_pinocchio_gravity=false");
      return;
    }

    std::string urdf = robot_description_;
    if (urdf.empty() && !urdf_path_.empty()) {
      urdf = ReadFile(urdf_path_);
    }
    if (urdf.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "F108: no robot_description available (hardware param or urdf_path); "
        "running without gravity feedforward");
      return;
    }

    try {
      pinocchio::urdf::buildModelFromXML(urdf, model_);
      data_ = pinocchio::Data(model_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "F108: pinocchio model build failed: %s; no gravity feedforward",
        e.what());
      return;
    }

    ApplyCalibratedInertia();

    q_.resize(model_.nq);
    q_.setZero();
    v_zero_ = Eigen::VectorXd::Zero(model_.nv);
    a_zero_ = Eigen::VectorXd::Zero(model_.nv);

    q_index_.assign(joints_.size(), -1);
    v_index_.assign(joints_.size(), -1);
    size_t mapped = 0;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!model_.existJointName(joints_[i].name)) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "F108: pinocchio model has no joint '%s'", joints_[i].name.c_str());
        continue;
      }
      const pinocchio::JointIndex jid = model_.getJointId(joints_[i].name);
      q_index_[i] = model_.joints[jid].idx_q();
      v_index_[i] = model_.joints[jid].idx_v();
      ++mapped;
    }

    ff_scale_ = LoadGravityScales();
    gravity_ff_ready_ = mapped > 0;
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "F108 gravity feedforward ready: ratio=%.2f, %zu/%zu joints mapped, "
      "F49 calibrated inertia %s",
      gravity_ff_ratio_, mapped, joints_.size(),
      use_calibrated_inertia_ ? "enabled" : "disabled");
  }

  void ApplyCalibratedInertia()
  {
    if (!use_calibrated_inertia_) {
      return;
    }
    std::string path = ExpandHome(inertia_params_file_);
    if (path.empty()) {
      try {
        path = ament_index_cpp::get_package_share_directory("a3_description") +
          "/config/inertia_params.yaml";
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLoggerName),
          "F108: cannot resolve a3_description share for F49 file: %s",
          e.what());
        return;
      }
    }

    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "F108: cannot load inertia params %s: %s; nominal URDF inertia",
        path.c_str(), e.what());
      return;
    }
    if (!root["use_calibrated_params"].as<bool>(false)) {
      return;
    }
    const YAML::Node params = root["inertia_params"];
    if (!params) {
      return;
    }

    size_t applied = 0;
    for (auto it = params.begin(); it != params.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      const auto link_it = kF49JointToLink.find(key);
      if (link_it == kF49JointToLink.end()) {
        continue;
      }
      const std::string & link = link_it->second;
      if (!model_.existBodyName(link)) {
        continue;
      }
      const pinocchio::FrameIndex fid = model_.getBodyId(link);
      const pinocchio::JointIndex jid = model_.frames[fid].parentJoint;
      if (jid <= 0 || jid >= static_cast<pinocchio::JointIndex>(
        model_.inertias.size()))
      {
        continue;
      }
      const YAML::Node entry = it->second;
      const double mass = entry["mass"].as<double>();
      const YAML::Node c = entry["com"];
      const Eigen::Vector3d com(
        c[0].as<double>(), c[1].as<double>(), c[2].as<double>());
      const auto & Y = model_.inertias[jid];
      model_.inertias[jid] = pinocchio::Inertia(mass, com, Y.inertia());
      ++applied;
    }
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "F108: applied F49 calibrated inertia to %zu links", applied);
  }

  // Per-joint F89 tau_scale in joints_ order; L1-L6 default 1.0, L7 always 0
  // (gripper gets no gravity FF, matching zero_torque's L1-L6 joint set).
  std::vector<double> LoadGravityScales()
  {
    std::vector<double> scales(joints_.size(), 0.0);
    for (size_t i = 0; i < joints_.size() && i < 6; ++i) {
      scales[i] = 1.0;
    }

    const std::string path = ExpandHome(gravity_scales_file_);
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception &) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "F108: gravity scales file %s not readable; tau_scale=1.0 for L1-L6",
        path.c_str());
      return scales;
    }
    YAML::Node node =
      root["zero_torque_controller"]["ros__parameters"]["tau_scale"];
    if (!node) {
      node = root["tau_scale"];
    }
    if (!node || !node.IsSequence()) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "F108: no tau_scale sequence in %s; using 1.0 for L1-L6",
        path.c_str());
      return scales;
    }
    const size_t n = std::min(node.size(), size_t(6));
    for (size_t i = 0; i < n; ++i) {
      scales[i] = node[i].as<double>();
    }
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "F108: loaded F89 tau_scale for %zu arm joints from %s", n, path.c_str());
    return scales;
  }

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

  void PublishHealth()
  {
    const auto now = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = health_node_->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "a3_hardware:feedback_watchdog";
    status.hardware_id = "a3";
    diagnostic_msgs::msg::DiagnosticStatus health;
    health.name = "a3_hardware:motor_health";
    health.hardware_id = "a3";
    bool stale = false;
    bool has_fault = false;
    bool overheat = false;
    bool overwarn = false;
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

        diagnostic_msgs::msg::KeyValue tkv;
        tkv.key = "motor" + std::to_string(j.motor_id) + "_temp_c";
        tkv.value = std::to_string(j.hw_temp);
        health.values.push_back(tkv);
        diagnostic_msgs::msg::KeyValue mkv;
        mkv.key = "motor" + std::to_string(j.motor_id) + "_mode";
        mkv.value = std::to_string(static_cast<unsigned>(j.hw_mode));
        health.values.push_back(mkv);
        diagnostic_msgs::msg::KeyValue fkv;
        fkv.key = "motor" + std::to_string(j.motor_id) + "_fault";
        fkv.value = std::to_string(static_cast<unsigned>(j.hw_fault));
        health.values.push_back(fkv);

        // Fault words are firmware-latched conditions; once seen they stay
        // ERROR until on_activate's clear-fault, even after traffic stops.
        if (j.fault_latched) {
          has_fault = true;
        }
        // Temperature is a live measurement — only evaluate FRESH frames, a
        // stale value from a silent motor must not read as cooled (LL-011).
        const bool fresh = j.has_feedback && age >= 0.0 &&
          age <= feedback_timeout_s_;
        if (fresh) {
          if (j.hw_temp >= temp_protect_c_) {
            overheat = true;
          } else if (j.hw_temp >= temp_warn_c_) {
            overwarn = true;
          }
        }
      }
    }
    status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                         : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = stale
      ? "feedback stale on >=1 motor; whole-arm freeze-hold engaged"
      : "all feedback channels healthy";
    arr.status.push_back(status);

    if (has_fault || overheat) {
      health.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      health.message = has_fault
        ? "motor fault word nonzero on >=1 motor; emergency reset"
        : "motor temperature >= protect threshold; safe park + cooling";
    } else if (overwarn) {
      health.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      health.message = "motor temperature >= warn threshold";
    } else {
      health.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      health.message = "all motors within thermal/fault limits";
    }
    arr.status.push_back(health);
    diag_pub_->publish(arr);
    std_msgs::msg::Bool b;
    b.data = stale;
    stale_pub_->publish(b);
  }

  void PublishMotorStates()
  {
    // F84: under hardware:=can the a3_can_bridge stack does not run, so the
    // F44/F43 thermal/fault FSM gating (subscription /a3/motor/states) would
    // be silently dead. Republish decoded plugin state in the bridge message
    // contract so the existing FSM logic works unchanged.
    a3_can_bridge::msg::MotorStates msg;
    msg.header.stamp = health_node_->now();
    const auto now = std::chrono::steady_clock::now();
    const bool enabled = active_.load();
    {
      std::lock_guard<std::mutex> lock(fb_mutex_);
      msg.states.reserve(joints_.size());
      for (const auto & j : joints_) {
        a3_can_bridge::msg::MotorState s;
        s.motor_id = j.motor_id;
        s.position_rad = j.hw_pos;
        s.speed_rad_s = j.hw_vel;
        s.torque_nm = j.hw_eff;
        s.temperature_c = static_cast<float>(j.hw_temp);
        s.mode_status = j.hw_mode;
        s.fault_mask = j.hw_fault;
        s.has_feedback = j.has_feedback;
        s.fresh = j.has_feedback &&
          std::chrono::duration<double>(now - j.last_fb_time).count() <=
            feedback_timeout_s_;
        s.enabled = enabled;
        msg.states.push_back(s);
      }
    }
    motor_states_pub_->publish(msg);
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
              j.hw_temp = fb->current_temp;
              j.hw_mode = fb->mode_status;
              j.hw_fault = fb->fault_code;
              if (fb->fault_code != 0) {
                j.fault_latched = true;
              }
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
  bool adaptive_kd_enabled_{false};
  double kd_min_{kDefaultKdMin};
  double kd_max_{kDefaultKdMax};
  double kd_velocity_ref_{kDefaultKdVelocityRef};
  double kd_smoothing_alpha_{kDefaultKdSmoothingAlpha};
  bool motor_can_timeout_enabled_{kDefaultMotorCanTimeoutEnabled};
  double motor_can_timeout_s_{kDefaultMotorCanTimeoutS};
  double feedback_timeout_s_{kDefaultFeedbackTimeoutS};
  double startup_kd_{kDefaultStartupKd};
  double temp_warn_c_{kDefaultTempWarnC};
  double temp_protect_c_{kDefaultTempProtectC};
  std::string motor_states_topic_{"/a3/motor/states"};
  double motor_states_rate_hz_{kDefaultMotorStatesRateHz};
  int soft_start_cycles_{kDefaultSoftStartCycles};
  int soft_start_remaining_{0};
  std::atomic_bool fb_stale_{false};

  // F108 position-mode gravity feedforward state.
  double gravity_ff_ratio_{1.0};
  bool use_pinocchio_gravity_{true};
  bool use_calibrated_inertia_{true};
  std::string inertia_params_file_;
  std::string gravity_scales_file_;
  std::string robot_description_;
  std::string urdf_path_;
  bool gravity_ff_ready_{false};
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_zero_;
  Eigen::VectorXd a_zero_;
  std::vector<Eigen::Index> q_index_;
  std::vector<Eigen::Index> v_index_;
  std::vector<double> ff_scale_;

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
  rclcpp::Publisher<a3_can_bridge::msg::MotorStates>::SharedPtr motor_states_pub_;
  rclcpp::TimerBase::SharedPtr motor_states_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_handle_;
};

}  // namespace a3_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  a3_hardware_interface::A3MITHardwareInterface,
  hardware_interface::SystemInterface)
