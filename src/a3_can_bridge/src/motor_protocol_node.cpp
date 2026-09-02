#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "a3_can_bridge/arm_mapper.hpp"
#include "a3_can_bridge/frame_codec.hpp"
#include "a3_can_bridge/protocol_codec.hpp"
#include "a3_can_bridge/trajectory_interpolator.hpp"
#include "a3_can_bridge/srv/motor_command.hpp"
#include "a3_can_bridge/srv/set_can_id.hpp"
#include "a3_can_bridge/srv/set_motor_param.hpp"
#include "a3_can_bridge/srv/motor_scan.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using trajectory_msgs::msg::JointTrajectory;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using std_msgs::msg::UInt8MultiArray;
using std_srvs::srv::Trigger;
using a3_can_bridge::srv::MotorCommand;
using a3_can_bridge::srv::SetCanId;
using a3_can_bridge::srv::SetMotorParam;
using a3_can_bridge::srv::MotorScan;

namespace a3_can_bridge
{

static constexpr size_t kNumArmJoints = DogMapper::kTemporaryIndexMap.size();

/// Unused for arm (kept so expand_joint_offsets_from_mirror=false path compiles if referenced).
static std::vector<double> BuildMirrorJointOffsetsRad(double hip, double thigh, double calf_mag)
{
  (void)hip;
  (void)thigh;
  (void)calf_mag;
  return std::vector<double>(kNumArmJoints, 0.0);
}

enum class TuneScope
{
  ALL = 0,
  FRONT = 1,
  REAR = 2
};

static std::string ToLower(std::string v)
{
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return v;
}

static std::vector<std::string> SplitTokens(const std::string & text)
{
  std::istringstream iss(text);
  std::vector<std::string> out;
  std::string token;
  while (iss >> token) {
    out.push_back(token);
  }
  return out;
}

static bool ParseBool(const std::string & text)
{
  const std::string v = ToLower(text);
  return (v == "1" || v == "true" || v == "yes" || v == "on");
}

static TuneScope ParseScope(const std::string & text)
{
  const std::string v = ToLower(text);
  if (v == "front" || v == "can0") {
    return TuneScope::FRONT;
  }
  if (v == "rear" || v == "can1") {
    return TuneScope::REAR;
  }
  return TuneScope::ALL;
}

static std::optional<MotorRoute> GetRouteByMotorId(uint8_t motor_id)
{
  for (const auto & route : DogMapper::kTemporaryIndexMap) {
    if (route.motor_id == motor_id) {
      return route;
    }
  }
  return std::nullopt;
}

class MotorProtocolNode : public rclcpp::Node
{
public:
  MotorProtocolNode()
  : Node("motor_protocol_node")
  {
    kp_ = this->declare_parameter<double>("kp", 30.0);
    kd_ = this->declare_parameter<double>("kd", 1.5);
    default_velocity_ = this->declare_parameter<double>("default_velocity", 0.0);
    default_tau_ff_ = this->declare_parameter<double>("default_tau_ff", 0.0);
    runtime_tune_topic_ = this->declare_parameter<std::string>("runtime_tune_topic", "/mit_gains_cmd");
    publish_feedback_joint_states_ = this->declare_parameter<bool>("publish_feedback_joint_states", true);
    feedback_joint_states_topic_ = this->declare_parameter<std::string>(
      "feedback_joint_states_topic", "/joint_states_feedback");
    feedback_joint_states_timer_hz_ = this->declare_parameter<double>("feedback_joint_states_timer_hz", 50.0);
    publish_feedback_velocity_effort_ = this->declare_parameter<bool>("publish_feedback_velocity_effort", true);
    feedback_joint_fill_nan_ = this->declare_parameter<bool>("feedback_joint_fill_nan", true);
    publish_motor_diagnostics_ = this->declare_parameter<bool>("publish_motor_diagnostics", false);
    motor_diagnostics_topic_ = this->declare_parameter<std::string>("motor_diagnostics_topic", "/diagnostics");
    enable_joint_tau_ff_ = this->declare_parameter<bool>("enable_joint_tau_ff", false);
    gravity_ff_scale_ = this->declare_parameter<double>("gravity_ff_scale", 1.0);
    tau_ff_nominal_nm_ = this->declare_parameter<std::vector<double>>(
      "tau_ff_nominal_nm", std::vector<double>(kNumArmJoints, 0.0));
    // Pinocchio gravity feedforward (EDULITE-aligned): URDF-frame τ_g * joint_signs → MIT tau
    enable_gravity_compensation_ = this->declare_parameter<bool>(
      "enable_gravity_compensation", false);
    gravity_apply_joint_signs_ = this->declare_parameter<bool>(
      "gravity_apply_joint_signs", true);
    gravity_compensation_topic_ = this->declare_parameter<std::string>(
      "gravity_compensation_topic", "/a3/gravity_torque");
    gravity_fresh_timeout_s_ = this->declare_parameter<double>(
      "gravity_fresh_timeout_s", 0.5);
    gravity_joint_scale_ = this->declare_parameter<std::vector<double>>(
      "gravity_joint_scale", std::vector<double>(kNumArmJoints, 1.0));
    use_joint_cmd_limits_ = this->declare_parameter<bool>("use_joint_cmd_limits", true);
    joint_cmd_min_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_cmd_min_rad", std::vector<double>(kNumArmJoints, -3.14));
    joint_cmd_max_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_cmd_max_rad", std::vector<double>(kNumArmJoints, 3.14));
    use_motor_domain_limits_ = this->declare_parameter<bool>("use_motor_domain_limits", true);
    derive_motor_limits_from_joint_cmd_ = this->declare_parameter<bool>(
      "derive_motor_limits_from_joint_cmd", true);
    joint_mit_min_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_mit_min_rad", std::vector<double>(kNumArmJoints, ProtocolCodec::kPMin));
    joint_mit_max_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_mit_max_rad", std::vector<double>(kNumArmJoints, ProtocolCodec::kPMax));
    enable_startup_smoothing_ = this->declare_parameter<bool>("enable_startup_smoothing", true);
    startup_smoothing_duration_s_ = this->declare_parameter<double>("startup_smoothing_duration_s", 1.5);
    command_max_velocity_rad_s_ = this->declare_parameter<double>("command_max_velocity_rad_s", 6.0);
    limit_velocity_only_during_smoothing_ = this->declare_parameter<bool>(
      "limit_velocity_only_during_smoothing", true);
    enable_mode_rising_smoothing_ = this->declare_parameter<bool>("enable_mode_rising_smoothing", true);
    motor_enabled_mode_status_ = this->declare_parameter<int>("motor_enabled_mode_status", 2);
    enable_feedback_step_limit_ = this->declare_parameter<bool>("enable_feedback_step_limit", true);
    feedback_step_limit_rad_ = this->declare_parameter<double>("feedback_step_limit_rad", 0.20);
    feedback_fresh_timeout_s_ = this->declare_parameter<double>("feedback_fresh_timeout_s", 0.30);
    enable_rx_decode_log_ = this->declare_parameter<bool>("enable_rx_decode_log", true);
    prefer_joint_name_mapping_ = this->declare_parameter<bool>("prefer_joint_name_mapping", true);
    enable_power_sequence_gate_ = this->declare_parameter<bool>("enable_power_sequence_gate", true);
    power_sequence_gate_topic_ = this->declare_parameter<std::string>("power_sequence_gate_topic", "/power_sequence/gate_open");
    tx_enable_can0_ = this->declare_parameter<bool>("tx_enable_can0", true);
    tx_enable_can1_ = this->declare_parameter<bool>("tx_enable_can1", true);

    const std::string arm_bus_str = this->declare_parameter<std::string>("arm_bus", "can1");
    CanBus parsed_bus = CanBus::CAN1;
    if (ArmMapper::ParseArmBus(arm_bus_str, &parsed_bus)) {
      ArmMapper::SetArmBus(parsed_bus);
      RCLCPP_INFO(this->get_logger(), "arm_bus=%s", arm_bus_str.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "invalid arm_bus='%s' (expect can0/can1); keeping default can1",
        arm_bus_str.c_str());
    }
    enable_min_tx_refresh_ = this->declare_parameter<bool>("enable_min_tx_refresh", true);
    min_tx_refresh_interval_s_ = this->declare_parameter<double>("min_tx_refresh_interval_s", 0.02);
    enable_max_tx_rate_limit_ = this->declare_parameter<bool>("enable_max_tx_rate_limit", true);
    max_tx_rate_per_motor_hz_ = this->declare_parameter<double>("max_tx_rate_per_motor_hz", 60.0);
    joint_signs_ = this->declare_parameter<std::vector<double>>(
      "joint_signs", std::vector<double>(kNumArmJoints, 1.0));

    joint_offsets_rad_ = this->declare_parameter<std::vector<double>>(
      "joint_offsets_rad", std::vector<double>(kNumArmJoints, 0.0));

    const double off_hip = this->declare_parameter<double>("joint_offset_hip_rad", 0.0);
    const double off_thigh = this->declare_parameter<double>("joint_offset_thigh_rad", 0.0);
    const double off_calf = this->declare_parameter<double>("joint_offset_calf_rad", 0.0);

    const bool expand_mirror = this->declare_parameter<bool>(
      "expand_joint_offsets_from_mirror", false);
    if (expand_mirror) {
      joint_offsets_rad_ = BuildMirrorJointOffsetsRad(off_hip, off_thigh, off_calf);
      RCLCPP_INFO(
        this->get_logger(),
        "expand_joint_offsets_from_mirror: hip=%.5f thigh=%.5f calf_mag=%.5f (overrides joint_offsets_rad)",
        off_hip, off_thigh, off_calf);
    }

    if (joint_signs_.size() != kNumArmJoints || joint_offsets_rad_.size() != kNumArmJoints) {
      RCLCPP_WARN(
        this->get_logger(),
        "joint_signs/joint_offsets_rad size mismatch, reset to kNumArmJoints defaults.");
      joint_signs_ = std::vector<double>(kNumArmJoints, 1.0);
      joint_offsets_rad_ = std::vector<double>(kNumArmJoints, 0.0);
    }
    if (tau_ff_nominal_nm_.size() != kNumArmJoints) {
      RCLCPP_WARN(this->get_logger(), "tau_ff_nominal_nm size != %zu, padding/truncating.", kNumArmJoints);
      tau_ff_nominal_nm_.resize(kNumArmJoints, 0.0);
    }
    if (gravity_joint_scale_.size() != kNumArmJoints) {
      RCLCPP_WARN(this->get_logger(), "gravity_joint_scale size mismatch, reset to 1.0");
      gravity_joint_scale_ = std::vector<double>(kNumArmJoints, 1.0);
    }
    gravity_tau_urdf_.fill(0.0);
    has_gravity_sample_ = false;
    last_gravity_stamp_ns_ = 0;

    publish_mit_mapped_ = this->declare_parameter<bool>("publish_mit_mapped_positions", true);
    mit_mapped_topic_ = this->declare_parameter<std::string>(
      "mit_mapped_positions_topic", "/mit_motor_position_rad");

    enable_trajectory_interpolation_ = this->declare_parameter<bool>(
      "enable_trajectory_interpolation", true);
    trajectory_interp_rate_hz_ = this->declare_parameter<double>(
      "trajectory_interp_rate_hz", 200.0);
    traj_interp_method_ = ParseTrajInterpMethod(
      this->declare_parameter<std::string>("trajectory_interpolation_method", "auto"));
    zero_torque_kp_ = this->declare_parameter<double>("zero_torque_kp", 0.0);
    zero_torque_kd_ = this->declare_parameter<double>("zero_torque_kd", 1.0);
    control_mode_topic_ = this->declare_parameter<std::string>(
      "control_mode_topic", "/a3/control_mode");
    zero_torque_active_ = false;

    traj_sub_ = this->create_subscription<JointTrajectory>(
      "/joint_group_effort_controller/joint_trajectory", rclcpp::SensorDataQoS(),
      std::bind(&MotorProtocolNode::OnTrajectory, this, std::placeholders::_1));
    rx_sub_ = this->create_subscription<UInt8MultiArray>(
      "/can_rx_frames", rclcpp::SensorDataQoS(),
      std::bind(&MotorProtocolNode::OnRxFrame, this, std::placeholders::_1));
    tune_sub_ = this->create_subscription<String>(
      runtime_tune_topic_, 10,
      std::bind(&MotorProtocolNode::OnTuneCommand, this, std::placeholders::_1));
    if (enable_power_sequence_gate_) {
      gate_sub_ = this->create_subscription<Bool>(
        power_sequence_gate_topic_, 10,
        std::bind(&MotorProtocolNode::OnPowerGate, this, std::placeholders::_1));
    }

    gravity_sub_ = this->create_subscription<JointState>(
      gravity_compensation_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MotorProtocolNode::OnGravityTorque, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(),
      "Gravity FF: enable=%d topic=%s scale=%.3f (URDF τ_g × joint_signs → MIT; EDULITE-aligned)",
      enable_gravity_compensation_ ? 1 : 0,
      gravity_compensation_topic_.c_str(),
      gravity_ff_scale_);

    control_mode_pub_ = this->create_publisher<String>(control_mode_topic_, 10);
    control_mode_sub_ = this->create_subscription<String>(
      control_mode_topic_, 10,
      [this](const String::SharedPtr msg) {
        if (!msg) {
          return;
        }
        last_control_mode_ = msg->data;
      });
    zero_torque_start_srv_ = this->create_service<Trigger>(
      "/a3/zero_torque/start",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp) {
        if (last_control_mode_ == "TRAJ_RUNNING" || last_control_mode_ == "SERVO") {
          resp->success = false;
          resp->message = "rejected: mode=" + last_control_mode_;
          return;
        }
        if (!zero_torque_active_) {
          saved_kp_can0_ = runtime_kp_can0_;
          saved_kp_can1_ = runtime_kp_can1_;
          saved_kd_can0_ = runtime_kd_can0_;
          saved_kd_can1_ = runtime_kd_can1_;
        }
        runtime_kp_can0_ = Clamp(zero_torque_kp_, ProtocolCodec::kKpMin, ProtocolCodec::kKpMax);
        runtime_kp_can1_ = runtime_kp_can0_;
        runtime_kd_can0_ = Clamp(zero_torque_kd_, ProtocolCodec::kKdMin, ProtocolCodec::kKdMax);
        runtime_kd_can1_ = runtime_kd_can0_;
        zero_torque_active_ = true;
        enable_gravity_compensation_ = true;
        PublishControlMode("ZERO_TORQUE");
        resp->success = true;
        resp->message = "ZERO_TORQUE on (soft kp + gravity FF)";
      });
    zero_torque_stop_srv_ = this->create_service<Trigger>(
      "/a3/zero_torque/stop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp) {
        if (zero_torque_active_) {
          runtime_kp_can0_ = saved_kp_can0_;
          runtime_kp_can1_ = saved_kp_can1_;
          runtime_kd_can0_ = saved_kd_can0_;
          runtime_kd_can1_ = saved_kd_can1_;
          zero_torque_active_ = false;
          PublishControlMode("IDLE");
        }
        resp->success = true;
        resp->message = "ZERO_TORQUE off";
      });

    // EL05 电机协议命令服务（F17）：enable/reset/set_zero/get_device_id/request_version 共用 MotorCommand
    auto make_cmd_srv = [this](const std::string & name, uint8_t command) {
        return this->create_service<MotorCommand>(
          name,
          [this, command](const std::shared_ptr<MotorCommand::Request> req,
                          std::shared_ptr<MotorCommand::Response> resp) {
            HandleMotorCommandService(req, resp, command);
          });
      };
    enable_srv_ = make_cmd_srv("/a3/motor/enable", 1);
    reset_srv_ = make_cmd_srv("/a3/motor/reset", 2);
    set_zero_srv_ = make_cmd_srv("/a3/motor/set_zero", 3);
    get_device_id_srv_ = make_cmd_srv("/a3/motor/get_device_id", 0);
    request_version_srv_ = make_cmd_srv("/a3/motor/request_version", 4);

    set_can_id_srv_ = this->create_service<SetCanId>(
      "/a3/motor/set_can_id",
      [this](const std::shared_ptr<SetCanId::Request> req,
             std::shared_ptr<SetCanId::Response> resp) {
        HandleSetCanIdService(req, resp);
      });

    set_param_srv_ = this->create_service<SetMotorParam>(
      "/a3/motor/set_param",
      [this](const std::shared_ptr<SetMotorParam::Request> req,
             std::shared_ptr<SetMotorParam::Response> resp) {
        HandleSetParamService(req, resp);
      });

    scan_srv_ = this->create_service<MotorScan>(
      "/a3/motor/scan",
      [this](const std::shared_ptr<MotorScan::Request> req,
             std::shared_ptr<MotorScan::Response> resp) {
        HandleScanService(req, resp);
      });

    if (enable_trajectory_interpolation_ && trajectory_interp_rate_hz_ > 1e-3) {
      const int64_t period_ns = static_cast<int64_t>(1e9 / trajectory_interp_rate_hz_);
      traj_interp_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(period_ns),
        std::bind(&MotorProtocolNode::OnTrajectoryInterpTimer, this));
      RCLCPP_INFO(
        this->get_logger(),
        "Trajectory time interpolation enabled @ %.1f Hz", trajectory_interp_rate_hz_);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Trajectory time interpolation DISABLED (legacy first-point mode)");
    }

    const int can_tx_qos_depth = this->declare_parameter<int>("can_tx_frames_qos_depth", 4000);
    rclcpp::QoS qos_can_tx(static_cast<size_t>(std::max(1, can_tx_qos_depth)));
    qos_can_tx.reliable();
    tx_pub_ = this->create_publisher<UInt8MultiArray>("/can_tx_frames", qos_can_tx);
    feedback_pub_ = this->create_publisher<String>("/motor_feedback", 50);
    device_id_pub_ = this->create_publisher<String>("/a3/motor/device_id", 10);
    version_pub_ = this->create_publisher<String>("/a3/motor/version", 10);
    if (publish_feedback_joint_states_) {
      feedback_joint_states_pub_ = this->create_publisher<JointState>(
        feedback_joint_states_topic_, rclcpp::SensorDataQoS());
    }
    if (publish_motor_diagnostics_) {
      diagnostics_pub_ = this->create_publisher<DiagnosticArray>(motor_diagnostics_topic_, 10);
    }
    if (
      (publish_feedback_joint_states_ || publish_motor_diagnostics_) &&
      feedback_joint_states_timer_hz_ > 1e-3)
    {
      const int64_t period_ns = static_cast<int64_t>(1e9 / feedback_joint_states_timer_hz_);
      feedback_js_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(period_ns),
        std::bind(&MotorProtocolNode::OnFeedbackJointStatesTimer, this));
    }

    if (publish_mit_mapped_) {
      mapped_positions_pub_ = this->create_publisher<JointState>(mit_mapped_topic_, 10);
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing MIT-mapped angles (θ_mit=sign*θ_champ+offset) on '%s' (same joint order as trajectory)",
        mit_mapped_topic_.c_str());
    }
    tx_refresh_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&MotorProtocolNode::OnTxRefreshTimer, this));
    tx_stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MotorProtocolNode::LogTxWindowStats, this));

    RCLCPP_WARN(
      this->get_logger(),
      "A3 arm mapping enabled: 7 joints L1..L7 -> motor IDs [1,2,3,4,5,6,7], all on %s.",
      (ArmMapper::ArmBus() == CanBus::CAN0) ? "can0" : "can1");
    RCLCPP_INFO(
      this->get_logger(),
      "Runtime MIT tuning topic: %s (example: 'scope=all kp=25 kd=1.8 tau=0.3' or 'scope=rear tau=0.6' or 'reset=1')",
      runtime_tune_topic_.c_str());
    if (enable_power_sequence_gate_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Power sequence gate enabled: waiting gate_open on topic '%s' before forwarding trajectory",
        power_sequence_gate_topic_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Power sequence gate disabled, trajectory forwarding always enabled");
    }
    ResetRuntimeTuning();
    last_commanded_mit_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    last_feedback_champ_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    last_feedback_joint_vel_rad_s_.fill(std::numeric_limits<double>::quiet_NaN());
    last_feedback_effort_nm_.fill(std::numeric_limits<double>::quiet_NaN());
    last_feedback_mode_status_.fill(-1);
    last_feedback_temp_c_.fill(std::numeric_limits<double>::quiet_NaN());
    last_feedback_fault_mask_.fill(0);
    last_sent_champ_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    boot_feedback_champ_rad_.fill(std::numeric_limits<double>::quiet_NaN());
    boot_feedback_captured_.fill(false);
    has_last_sent_.fill(false);
    has_mode_status_.fill(false);
    last_mode_status_.fill(-1);
    last_feedback_stamp_ns_.fill(0);
    first_command_time_by_joint_ns_.fill(0);
    last_command_time_by_joint_ns_.fill(0);
    last_tx_pub_stamp_ns_.fill(0);
    latest_input_champ_rad_.fill(0.0);
    has_latest_input_.fill(false);

    if (joint_cmd_min_rad_.size() != kNumArmJoints || joint_cmd_max_rad_.size() != kNumArmJoints) {
      RCLCPP_WARN(this->get_logger(), "joint_cmd_min_rad/max size mismatch, using default [-3.14, 3.14]x12");
      joint_cmd_min_rad_ = std::vector<double>(kNumArmJoints, -3.14);
      joint_cmd_max_rad_ = std::vector<double>(kNumArmJoints, 3.14);
    }
    if (joint_mit_min_rad_.size() != kNumArmJoints || joint_mit_max_rad_.size() != kNumArmJoints) {
      RCLCPP_WARN(this->get_logger(), "joint_mit_min_rad/max size mismatch, using protocol default range x12");
      joint_mit_min_rad_ = std::vector<double>(kNumArmJoints, ProtocolCodec::kPMin);
      joint_mit_max_rad_ = std::vector<double>(kNumArmJoints, ProtocolCodec::kPMax);
    }
    RebuildMotorLimitTable();

    on_set_params_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return this->OnSetParameters(parameters);
      });
  }

private:
  rcl_interfaces::msg::SetParametersResult OnSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult out;
    out.successful = true;
    out.reason = "ok";
    for (const rclcpp::Parameter & p : parameters) {
      const std::string & name = p.get_name();
      if (name == "enable_joint_tau_ff") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          out.successful = false;
          out.reason = "enable_joint_tau_ff must be bool";
          return out;
        }
        enable_joint_tau_ff_ = p.as_bool();
        RCLCPP_WARN(
          this->get_logger(), "Runtime param: enable_joint_tau_ff=%d", enable_joint_tau_ff_ ? 1 : 0);
      } else if (name == "enable_gravity_compensation") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          out.successful = false;
          out.reason = "enable_gravity_compensation must be bool";
          return out;
        }
        enable_gravity_compensation_ = p.as_bool();
        RCLCPP_WARN(
          this->get_logger(), "Runtime param: enable_gravity_compensation=%d",
          enable_gravity_compensation_ ? 1 : 0);
      } else if (name == "gravity_apply_joint_signs") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          out.successful = false;
          out.reason = "gravity_apply_joint_signs must be bool";
          return out;
        }
        gravity_apply_joint_signs_ = p.as_bool();
        RCLCPP_WARN(
          this->get_logger(), "Runtime param: gravity_apply_joint_signs=%d",
          gravity_apply_joint_signs_ ? 1 : 0);
      } else if (name == "gravity_ff_scale") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          out.successful = false;
          out.reason = "gravity_ff_scale must be double";
          return out;
        }
        gravity_ff_scale_ = p.as_double();
        RCLCPP_WARN(this->get_logger(), "Runtime param: gravity_ff_scale=%.6f", gravity_ff_scale_);
      } else if (name == "tau_ff_nominal_nm") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          out.successful = false;
          out.reason = "tau_ff_nominal_nm must be double[]";
          return out;
        }
        const auto v = p.as_double_array();
        if (v.size() != kNumArmJoints) {
          out.successful = false;
          out.reason = "tau_ff_nominal_nm must have 7 elements";
          return out;
        }
        tau_ff_nominal_nm_.assign(v.begin(), v.end());
        RCLCPP_WARN(this->get_logger(), "Runtime param: tau_ff_nominal_nm updated (7 doubles)");
      } else if (name == "gravity_joint_scale") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          out.successful = false;
          out.reason = "gravity_joint_scale must be double[]";
          return out;
        }
        const auto v = p.as_double_array();
        if (v.size() != kNumArmJoints) {
          out.successful = false;
          out.reason = "gravity_joint_scale must have 7 elements";
          return out;
        }
        gravity_joint_scale_.assign(v.begin(), v.end());
        RCLCPP_WARN(this->get_logger(), "Runtime param: gravity_joint_scale updated");
      }
    }
    return out;
  }

  void OnGravityTorque(const JointState::SharedPtr msg)
  {
    if (msg->effort.empty()) {
      return;
    }
    // Prefer named mapping (L1_joint..); fall back to index order.
    std::array<double, kNumArmJoints> tau{};
    tau.fill(0.0);
    bool any = false;
    if (!msg->name.empty()) {
      for (size_t i = 0; i < msg->name.size() && i < msg->effort.size(); ++i) {
        const auto & name = msg->name[i];
        for (size_t j = 0; j < DogMapper::kChampJointNames.size(); ++j) {
          if (name == DogMapper::kChampJointNames[j]) {
            tau[j] = msg->effort[i];
            any = true;
            break;
          }
        }
      }
    }
    if (!any) {
      const size_t n = std::min(msg->effort.size(), kNumArmJoints);
      for (size_t i = 0; i < n; ++i) {
        tau[i] = msg->effort[i];
      }
      any = n > 0;
    }
    if (!any) {
      return;
    }
    gravity_tau_urdf_ = tau;
    has_gravity_sample_ = true;
    last_gravity_stamp_ns_ = this->now().nanoseconds();
  }

  void OnTrajectory(const JointTrajectory::SharedPtr msg)
  {
    ++traj_cb_count_window_;
    if (msg->points.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore trajectory without points");
      return;
    }

    if (enable_power_sequence_gate_ && !power_gate_open_) {
      ++skip_power_gate_window_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore trajectory: power gate closed");
      return;
    }
    if (zero_torque_active_ || last_control_mode_ == "SERVO") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore trajectory: mode zero_torque/SERVO");
      return;
    }

    if (enable_trajectory_interpolation_) {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      active_traj_ = *msg;
      traj_start_ = this->now();
      has_active_traj_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Trajectory buffered for interpolation: %zu points, %zu joints",
        active_traj_.points.size(), active_traj_.joint_names.size());
      return;
    }

    // Legacy: first-point only
    ApplyPositionTargets(msg->points.front().positions, msg->joint_names);
  }

  void OnTrajectoryInterpTimer()
  {
    if (!enable_trajectory_interpolation_) {
      return;
    }

    JointTrajectory traj_copy;
    rclcpp::Time start;
    bool has = false;
    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      if (!has_active_traj_) {
        return;
      }
      traj_copy = active_traj_;
      start = traj_start_;
      has = true;
    }
    if (!has) {
      return;
    }

    if (enable_power_sequence_gate_ && !power_gate_open_) {
      return;
    }

    const double elapsed = (this->now() - start).seconds();
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> effort;
    bool finished = false;
    if (!SampleJointTrajectory(
        traj_copy, elapsed, positions, velocities, effort, finished, traj_interp_method_))
    {
      return;
    }

    ApplyPositionTargets(positions, traj_copy.joint_names);

    if (finished) {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      has_active_traj_ = false;
    }
  }

  void ApplyPositionTargets(
    const std::vector<double> & positions,
    const std::vector<std::string> & joint_names)
  {
    traj_joint_count_window_ += std::min(positions.size(), DogMapper::kTemporaryIndexMap.size());
    uint32_t front_count = 0;
    uint32_t rear_count = 0;
    size_t total_sent = 0;

    std::array<double, DogMapper::kTemporaryIndexMap.size()> mapped_rad{};
    mapped_rad.fill(std::numeric_limits<double>::quiet_NaN());

    const bool use_names = prefer_joint_name_mapping_ && !joint_names.empty();

    for (size_t i = 0; i < DogMapper::kTemporaryIndexMap.size(); ++i) {
      const auto route = DogMapper::GetRouteByTrajectoryIndex(i);
      if (!route.has_value()) {
        continue;
      }

      double target = 0.0;
      bool got = false;
      if (use_names) {
        const char * jn = DogMapper::kChampJointNames[i];
        for (size_t j = 0; j < joint_names.size(); ++j) {
          if (joint_names[j] == jn && j < positions.size()) {
            target = positions[j];
            got = true;
            break;
          }
        }
      }
      if (!got) {
        if (i < positions.size()) {
          target = positions[i];
          got = true;
        } else if (has_latest_input_[i]) {
          target = latest_input_champ_rad_[i];
          got = true;
        }
      }
      if (!got) {
        continue;
      }

      latest_input_champ_rad_[i] = target;
      has_latest_input_[i] = true;

      if (enable_power_sequence_gate_ && !power_gate_open_) {
        ++skip_power_gate_window_;
        continue;
      }
      SendMitFrame(route.value(), target, front_count, rear_count, &mapped_rad);
      ++total_sent;
    }

    if (publish_mit_mapped_ && mapped_positions_pub_) {
      JointState js;
      js.header.stamp = this->now();
      js.name.reserve(DogMapper::kChampJointNames.size());
      for (const char * jn : DogMapper::kChampJointNames) {
        js.name.emplace_back(jn);
      }
      js.position.assign(mapped_rad.begin(), mapped_rad.end());
      mapped_positions_pub_->publish(js);
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MIT publish: total=%zu front(can0)=%u rear(can1)=%u kp=%.2f kd=%.2f v=%.2f tau=%.2f",
      total_sent, front_count, rear_count, runtime_kp_can0_, runtime_kd_can0_, default_velocity_, runtime_tau_can0_);
  }

  void SendMitFrame(
    const MotorRoute & route, double champ_position_raw,
    uint32_t & front_count, uint32_t & rear_count,
    std::array<double, DogMapper::kTemporaryIndexMap.size()> * mapped_out = nullptr)
  {
    const size_t idx = std::min(route.trajectory_index, static_cast<size_t>(ArmMapper::kTemporaryIndexMap.size() - 1));
    const int64_t now_ns = this->now().nanoseconds();
    if (
      enable_max_tx_rate_limit_ &&
      max_tx_rate_per_motor_hz_ > 1e-6 &&
      idx < last_tx_pub_stamp_ns_.size() &&
      last_tx_pub_stamp_ns_[idx] > 0)
    {
      const int64_t min_interval_ns = static_cast<int64_t>(1e9 / max_tx_rate_per_motor_hz_);
      if ((now_ns - last_tx_pub_stamp_ns_[idx]) < min_interval_ns) {
        ++skip_max_rate_limit_window_;
        return;
      }
    }
    const double champ_limited = ClampJointCommand(idx, champ_position_raw);
    const double champ_smoothed = SmoothJointCommand(idx, champ_limited);
    const double mapped_position_raw = joint_signs_[idx] * champ_smoothed + joint_offsets_rad_[idx];
    const double mapped_position = ClampMotorCommand(idx, mapped_position_raw);
    if (mapped_out != nullptr) {
      (*mapped_out)[idx] = mapped_position;
    }

    const bool is_front = (ArmMapper::ArmBus() == CanBus::CAN0);
    if ((is_front && !tx_enable_can0_) || (!is_front && !tx_enable_can1_)) {
      ++skip_bus_disabled_window_;
      return;
    }
    const double use_kp = is_front ? runtime_kp_can0_ : runtime_kp_can1_;
    const double use_kd = is_front ? runtime_kd_can0_ : runtime_kd_can1_;
    const double use_tau = ComputeMitTorqueFf(idx, is_front);

    const auto frame = ProtocolCodec::BuildMitControlFrame(
      ArmMapper::ArmBus(),
      route.motor_id,
      static_cast<float>(mapped_position),
      static_cast<float>(default_velocity_),
      static_cast<float>(use_kp),
      static_cast<float>(use_kd),
      static_cast<float>(use_tau));

    auto packed = FrameCodec::Pack(frame);
    tx_pub_->publish(packed);
    ++tx_traj_total_window_;
    if (ArmMapper::ArmBus() == CanBus::CAN0) {
      ++front_count;
      ++tx_traj_can0_window_;
    } else {
      ++rear_count;
      ++tx_traj_can1_window_;
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "MIT TX joint=%s motor=%u raw=%.4f lim=%.4f smooth=%.4f %s",
      route.joint_hint,
      route.motor_id,
      champ_position_raw,
      champ_limited,
      champ_smoothed,
      ProtocolCodec::FrameSummary(frame).c_str());

    if (route.motor_id < last_commanded_mit_rad_.size()) {
      last_commanded_mit_rad_[route.motor_id] = mapped_position;
    }
    if (idx < last_tx_pub_stamp_ns_.size()) {
      last_tx_pub_stamp_ns_[idx] = now_ns;
    }
  }

  void OnTxRefreshTimer()
  {
    if (!enable_min_tx_refresh_ || min_tx_refresh_interval_s_ <= 1e-6) {
      return;
    }
    if (enable_power_sequence_gate_ && !power_gate_open_) {
      return;
    }
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t refresh_ns = static_cast<int64_t>(min_tx_refresh_interval_s_ * 1e9);
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      const size_t idx = std::min(route.trajectory_index, static_cast<size_t>(ArmMapper::kTemporaryIndexMap.size() - 1));
      if (idx >= last_tx_pub_stamp_ns_.size()) {
        continue;
      }
      if (last_tx_pub_stamp_ns_[idx] > 0 && (now_ns - last_tx_pub_stamp_ns_[idx]) < refresh_ns) {
        continue;
      }
      const bool is_front = (ArmMapper::ArmBus() == CanBus::CAN0);
      if ((is_front && !tx_enable_can0_) || (!is_front && !tx_enable_can1_)) {
        continue;
      }
      const double mapped_position =
        route.motor_id < last_commanded_mit_rad_.size() ? last_commanded_mit_rad_[route.motor_id] :
        std::numeric_limits<double>::quiet_NaN();
      if (!std::isfinite(mapped_position)) {
        continue;
      }
      const double use_kp = is_front ? runtime_kp_can0_ : runtime_kp_can1_;
      const double use_kd = is_front ? runtime_kd_can0_ : runtime_kd_can1_;
      const double use_tau = ComputeMitTorqueFf(idx, is_front);
      const auto frame = ProtocolCodec::BuildMitControlFrame(
        ArmMapper::ArmBus(),
        route.motor_id,
        static_cast<float>(mapped_position),
        static_cast<float>(default_velocity_),
        static_cast<float>(use_kp),
        static_cast<float>(use_kd),
        static_cast<float>(use_tau));
      auto packed = FrameCodec::Pack(frame);
      tx_pub_->publish(packed);
      ++tx_refresh_total_window_;
      if (is_front) {
        ++tx_refresh_can0_window_;
      } else {
        ++tx_refresh_can1_window_;
      }
      last_tx_pub_stamp_ns_[idx] = now_ns;
    }
  }

  void LogTxWindowStats()
  {
    const size_t tx_total = tx_traj_total_window_ + tx_refresh_total_window_;
    RCLCPP_INFO(
      this->get_logger(),
      "MP TX window(5s): traj_cb=%zu traj_joint_targets=%zu tx_total=%zu tx_traj=%zu(can0=%zu can1=%zu) tx_refresh=%zu(can0=%zu can1=%zu) skip_max_rate=%zu skip_bus_disabled=%zu",
      traj_cb_count_window_,
      traj_joint_count_window_,
      tx_total,
      tx_traj_total_window_,
      tx_traj_can0_window_,
      tx_traj_can1_window_,
      tx_refresh_total_window_,
      tx_refresh_can0_window_,
      tx_refresh_can1_window_,
      skip_max_rate_limit_window_,
      skip_bus_disabled_window_);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MP gate window(5s): gate_open=%d skip_gate=%zu",
      power_gate_open_ ? 1 : 0,
      skip_power_gate_window_);

    traj_cb_count_window_ = 0;
    traj_joint_count_window_ = 0;
    tx_traj_total_window_ = 0;
    tx_traj_can0_window_ = 0;
    tx_traj_can1_window_ = 0;
    tx_refresh_total_window_ = 0;
    tx_refresh_can0_window_ = 0;
    tx_refresh_can1_window_ = 0;
    skip_max_rate_limit_window_ = 0;
    skip_bus_disabled_window_ = 0;
    skip_power_gate_window_ = 0;
  }

  void OnPowerGate(const Bool::SharedPtr msg)
  {
    const bool next = msg->data;
    if (next == power_gate_open_) {
      return;
    }
    power_gate_open_ = next;
    RCLCPP_WARN(this->get_logger(), "Power sequence gate changed: gate_open=%d", power_gate_open_ ? 1 : 0);
    if (!power_gate_open_) {
      last_tx_pub_stamp_ns_.fill(0);
      if (zero_torque_active_) {
        runtime_kp_can0_ = saved_kp_can0_;
        runtime_kp_can1_ = saved_kp_can1_;
        runtime_kd_can0_ = saved_kd_can0_;
        runtime_kd_can1_ = saved_kd_can1_;
        zero_torque_active_ = false;
        PublishControlMode("IDLE");
      }
    }
  }

  void PublishControlMode(const std::string & mode)
  {
    last_control_mode_ = mode;
    String out;
    out.data = mode;
    if (control_mode_pub_) {
      control_mode_pub_->publish(out);
    }
  }

  void PublishFrame(const CanFrameMessage & frame)
  {
    tx_pub_->publish(FrameCodec::Pack(frame));
  }

  static CanBus BusForMotorId(uint8_t motor_id)
  {
    (void)motor_id;
    return ArmMapper::ArmBus();
  }

  static void FormatMcuUidHex(uint64_t uid, char * out, size_t out_len)
  {
    if (out == nullptr || out_len < 17) {
      return;
    }
    static const char kHex[] = "0123456789ABCDEF";
    for (int i = 0; i < 8; ++i) {
      const uint8_t b = static_cast<uint8_t>(uid >> (56 - i * 8));
      out[i * 2] = kHex[b >> 4];
      out[i * 2 + 1] = kHex[b & 0x0F];
    }
    out[16] = '\0';
  }

  void HandleMotorCommandService(
    const std::shared_ptr<MotorCommand::Request> & req,
    const std::shared_ptr<MotorCommand::Response> & resp,
    uint8_t command)
  {
    if (req->motor_id > 127) {
      resp->success = false;
      resp->message = "motor_id must be 0..127";
      return;
    }
    std::vector<uint8_t> ids;
    if (req->motor_id == 0) {
      ids.reserve(DogMapper::kTemporaryIndexMap.size());
      for (const auto & route : DogMapper::kTemporaryIndexMap) {
        ids.push_back(route.motor_id);
      }
    } else {
      ids.push_back(req->motor_id);
    }

    size_t sent = 0;
    for (const uint8_t mid : ids) {
      const CanBus bus = BusForMotorId(mid);
      CanFrameMessage frame;
      switch (command) {
        case 0:
          frame = ProtocolCodec::BuildGetDeviceIdProbeFrame(bus, mid);
          break;
        case 1:
          frame = ProtocolCodec::BuildEnableFrame(bus, mid);
          break;
        case 2:
          frame = ProtocolCodec::BuildResetFrame(bus, mid);
          break;
        case 3:
          frame = ProtocolCodec::BuildSetZeroFrame(bus, mid);
          break;
        case 4:
          frame = ProtocolCodec::BuildRequestVersionFrame(bus, mid);
          break;
        default:
          continue;
      }
      PublishFrame(frame);
      ++sent;
    }

    if (sent == 0) {
      resp->success = false;
      resp->message = "unknown command";
      return;
    }
    resp->success = true;
    resp->message = "ok (" + std::to_string(sent) + " frame(s))";
  }

  void HandleSetCanIdService(
    const std::shared_ptr<SetCanId::Request> & req,
    const std::shared_ptr<SetCanId::Response> & resp)
  {
    if (req->current_id == 0 || req->current_id > 127 || req->new_id == 0 || req->new_id > 127) {
      resp->success = false;
      resp->message = "current_id/new_id must be 1..127";
      return;
    }
    PublishFrame(ProtocolCodec::BuildSetCanIdFrame(
      BusForMotorId(req->current_id), req->current_id, req->new_id));
    resp->success = true;
    resp->message = "ok";
  }

  void HandleSetParamService(
    const std::shared_ptr<SetMotorParam::Request> & req,
    const std::shared_ptr<SetMotorParam::Response> & resp)
  {
    if (req->motor_id == 0 || req->motor_id > 127) {
      resp->success = false;
      resp->message = "motor_id must be 1..127";
      return;
    }
    PublishFrame(ProtocolCodec::BuildSetParamFrame(
      BusForMotorId(req->motor_id), req->motor_id, req->param_id, req->value));
    resp->success = true;
    resp->message = "ok";
  }

  void HandleScanService(
    const std::shared_ptr<MotorScan::Request> & req,
    const std::shared_ptr<MotorScan::Response> & resp)
  {
    uint8_t id_min = req->id_min == 0 ? 1 : req->id_min;
    uint8_t id_max = req->id_max == 0 ? 127 : req->id_max;
    if (id_max > 127) {
      id_max = 127;
    }
    if (id_min > id_max) {
      resp->success = false;
      resp->message = "id_min > id_max";
      return;
    }
    const CanBus bus = (req->bus == 1) ? CanBus::CAN1 : CanBus::CAN0;
    uint32_t sent = 0;
    for (uint32_t mid = id_min; mid <= id_max; ++mid) {
      PublishFrame(ProtocolCodec::BuildGetDeviceIdProbeFrame(bus, static_cast<uint8_t>(mid)));
      ++sent;
    }
    resp->success = true;
    resp->message = "sent " + std::to_string(sent) + " probes [" +
      std::to_string(static_cast<int>(id_min)) + ".." +
      std::to_string(static_cast<int>(id_max)) + "] on " +
      (bus == CanBus::CAN0 ? "can0" : "can1");
  }

  void PublishDeviceId(const DeviceIdResponse & rsp)
  {
    char uid[17];
    FormatMcuUidHex(rsp.mcu_uid, uid, sizeof(uid));
    std::ostringstream oss;
    oss << "motor=" << static_cast<int>(rsp.motor_id) << " uid=" << uid;
    String out;
    out.data = oss.str();
    if (device_id_pub_) {
      device_id_pub_->publish(out);
    }
    RCLCPP_INFO(this->get_logger(), "Device ID response: %s", out.data.c_str());
  }

  void PublishSoftwareVersion(const CanFrameMessage & frame)
  {
    const uint8_t motor_id = static_cast<uint8_t>((frame.can_id >> 8) & 0xFF);
    const std::string version = ProtocolCodec::DecodeVersionString(frame);
    std::ostringstream oss;
    oss << "motor=" << static_cast<int>(motor_id) << " version=" << version;
    String out;
    out.data = oss.str();
    if (version_pub_) {
      version_pub_->publish(out);
    }
    RCLCPP_INFO(this->get_logger(), "Software version response: %s", out.data.c_str());
  }

  void OnRxFrame(const UInt8MultiArray::SharedPtr msg)
  {
    const auto frame = FrameCodec::Unpack(*msg);
    if (!frame.has_value()) {
      return;
    }
    const CanFrameMessage & can = frame.value();

    if (const auto rsp = ProtocolCodec::DecodeDeviceIdResponse(can); rsp.has_value()) {
      PublishDeviceId(rsp.value());
      return;
    }
    if (ProtocolCodec::IsSoftwareVersionResponse(can)) {
      PublishSoftwareVersion(can);
      return;
    }

    const auto feedback = ProtocolCodec::DecodeFeedback(can);
    if (!feedback.has_value()) {
      return;
    }

    const double expected_mit = feedback->motor_id < last_commanded_mit_rad_.size() ?
      last_commanded_mit_rad_[feedback->motor_id] :
      std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(expected_mit)) {
      const double abs_err = std::fabs(expected_mit - feedback->current_angle);
      ++error_sample_count_;
      error_abs_sum_ += abs_err;
      error_abs_max_ = std::max(error_abs_max_, abs_err);
    }

    if (const auto route = GetRouteByMotorId(feedback->motor_id); route.has_value()) {
      const size_t idx = std::min(route->trajectory_index, static_cast<size_t>(ArmMapper::kTemporaryIndexMap.size() - 1));
      const double sign = std::fabs(joint_signs_[idx]) < 1e-6 ? 1.0 : joint_signs_[idx];
      const double champ_feedback = (feedback->current_angle - joint_offsets_rad_[idx]) / sign;
      const rclcpp::Time now = this->now();
      last_feedback_champ_rad_[idx] = champ_feedback;
      // CHAMP 关节角速度：θ_champ = (θ_mit - offset)/sign ⇒ ω_champ ≈ ω_mit/sign（直连模型）
      last_feedback_joint_vel_rad_s_[idx] = static_cast<double>(feedback->current_speed) / sign;
      // effort：与 EL05 / MIT 协议同号的电机报告力矩 (Nm)；与 tau_ff_nominal_nm 同一约定，不经 joint_signs 二次取反
      last_feedback_effort_nm_[idx] = static_cast<double>(feedback->current_torque);
      last_feedback_mode_status_[idx] = static_cast<int>(feedback->mode_status);
      last_feedback_temp_c_[idx] = static_cast<double>(feedback->current_temp);
      {
        uint32_t mask = 0;
        if (feedback->error_status) {
          mask |= 1u << 0;
        }
        if (feedback->hall_error) {
          mask |= 1u << 1;
        }
        if (feedback->magnet_error) {
          mask |= 1u << 2;
        }
        if (feedback->temp_error) {
          mask |= 1u << 3;
        }
        if (feedback->current_error) {
          mask |= 1u << 4;
        }
        if (feedback->voltage_error) {
          mask |= 1u << 5;
        }
        last_feedback_fault_mask_[idx] = mask;
      }
      last_feedback_stamp_ns_[idx] = now.nanoseconds();
      if (!boot_feedback_captured_[idx]) {
        boot_feedback_champ_rad_[idx] = champ_feedback;
        boot_feedback_captured_[idx] = true;
      }

      const int mode_curr = static_cast<int>(feedback->mode_status);
      const uint8_t mid = feedback->motor_id;
      if (enable_mode_rising_smoothing_) {
        const bool prev_known = has_mode_status_[mid];
        const int mode_prev = last_mode_status_[mid];
        const bool rising_to_enabled = IsEnabledMode(mode_curr) && (!prev_known || !IsEnabledMode(mode_prev));
        if (rising_to_enabled) {
          ResetSmoothingForJoint(idx, champ_feedback, now);
          RCLCPP_WARN(
            this->get_logger(),
            "Mode rising edge detected: motor=%u mode %d->%d, reset smoothing anchor to %.4f rad",
            static_cast<unsigned>(mid), mode_prev, mode_curr, champ_feedback);
        }
      }
      has_mode_status_[mid] = true;
      last_mode_status_[mid] = mode_curr;
    }

    if (enable_rx_decode_log_) {
      std::ostringstream oss;
      oss << "bus=" << feedback->source_bus
          << " motor=" << static_cast<int>(feedback->motor_id)
          << " mode=" << static_cast<int>(feedback->mode_status)
          << " angle=" << feedback->current_angle
          << " speed=" << feedback->current_speed
          << " torque=" << feedback->current_torque
          << " temp=" << feedback->current_temp
          << " err=" << (feedback->error_status ? "1" : "0");
      if (std::isfinite(expected_mit)) {
        oss << " cmd_angle=" << expected_mit
            << " abs_err=" << std::fabs(expected_mit - feedback->current_angle);
      }

      String out;
      out.data = oss.str();
      feedback_pub_->publish(out);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", out.data.c_str());
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "MIT tracking abs_err(rad): mean=%.4f max=%.4f samples=%zu | can0(kp=%.2f,kd=%.2f,tau=%.2f) can1(kp=%.2f,kd=%.2f,tau=%.2f)",
        error_sample_count_ > 0 ? (error_abs_sum_ / static_cast<double>(error_sample_count_)) : 0.0,
        error_abs_max_,
        error_sample_count_,
        runtime_kp_can0_, runtime_kd_can0_, runtime_tau_can0_,
        runtime_kp_can1_, runtime_kd_can1_, runtime_tau_can1_);
    }
  }

  void OnFeedbackJointStatesTimer()
  {
    if (publish_feedback_joint_states_ && feedback_joint_states_pub_) {
      PublishFeedbackJointStates();
    }
    if (publish_motor_diagnostics_ && diagnostics_pub_) {
      PublishMotorDiagnostics();
    }
  }

  void PublishMotorDiagnostics()
  {
    if (!diagnostics_pub_) {
      return;
    }
    DiagnosticArray da;
    da.header.stamp = this->now();
    for (size_t i = 0; i < DogMapper::kChampJointNames.size(); ++i) {
      DiagnosticStatus st;
      st.name = std::string("a3_can_bridge:motor_feedback:") + DogMapper::kChampJointNames[i];
      st.hardware_id = std::to_string(static_cast<int>(DogMapper::kTemporaryIndexMap[i].motor_id));
      st.level = DiagnosticStatus::OK;
      st.message = "feedback";

      auto push_kv = [&](const std::string & key, const std::string & val) {
        KeyValue kv;
        kv.key = key;
        kv.value = val;
        st.values.push_back(kv);
      };

      if (last_feedback_mode_status_[i] >= 0) {
        push_kv("mode_status", std::to_string(last_feedback_mode_status_[i]));
      } else {
        push_kv("mode_status", "unknown");
      }
      if (std::isfinite(last_feedback_temp_c_[i])) {
        push_kv("temp_c", std::to_string(last_feedback_temp_c_[i]));
      } else {
        push_kv("temp_c", "nan");
      }

      const uint32_t m = last_feedback_fault_mask_[i];
      push_kv("fault_mask", std::to_string(m));
      push_kv("fault_error_status", (m & (1u << 0)) ? "1" : "0");
      push_kv("fault_hall", (m & (1u << 1)) ? "1" : "0");
      push_kv("fault_magnet", (m & (1u << 2)) ? "1" : "0");
      push_kv("fault_temp", (m & (1u << 3)) ? "1" : "0");
      push_kv("fault_current", (m & (1u << 4)) ? "1" : "0");
      push_kv("fault_voltage", (m & (1u << 5)) ? "1" : "0");

      if (m != 0u) {
        st.level = DiagnosticStatus::ERROR;
        st.message = "motor fault bits set";
      }

      da.status.push_back(st);
    }
    diagnostics_pub_->publish(da);
  }

  void PublishFeedbackJointStates()
  {
    if (!feedback_joint_states_pub_) {
      return;
    }
    JointState js;
    js.header.stamp = this->now();
    const size_t n = DogMapper::kChampJointNames.size();
    js.name.reserve(n);
    js.position.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      js.name.emplace_back(DogMapper::kChampJointNames[i]);
      const double pos = std::isfinite(last_feedback_champ_rad_[i]) ? last_feedback_champ_rad_[i] : 0.0;
      js.position.emplace_back(pos);
    }
    if (publish_feedback_velocity_effort_) {
      const double nan_vel = std::numeric_limits<double>::quiet_NaN();
      const double nan_eff = std::numeric_limits<double>::quiet_NaN();
      js.velocity.assign(n, 0.0);
      js.effort.assign(n, 0.0);
      for (size_t i = 0; i < n; ++i) {
        const bool v_ok = std::isfinite(last_feedback_joint_vel_rad_s_[i]);
        const bool e_ok = std::isfinite(last_feedback_effort_nm_[i]);
        if (feedback_joint_fill_nan_) {
          js.velocity[i] = v_ok ? last_feedback_joint_vel_rad_s_[i] : nan_vel;
          js.effort[i] = e_ok ? last_feedback_effort_nm_[i] : nan_eff;
        } else {
          js.velocity[i] = v_ok ? last_feedback_joint_vel_rad_s_[i] : 0.0;
          js.effort[i] = e_ok ? last_feedback_effort_nm_[i] : 0.0;
        }
      }
    }
    feedback_joint_states_pub_->publish(js);
  }

  double ComputeMitTorqueFf(size_t idx, bool is_front)
  {
    const double bus_tau = is_front ? runtime_tau_can0_ : runtime_tau_can1_;
    double tau = bus_tau;

    // Legacy static nominal (Nm, already in MIT/motor frame). Independent of live gravity.
    if (enable_joint_tau_ff_) {
      const double nominal = idx < tau_ff_nominal_nm_.size() ? tau_ff_nominal_nm_[idx] : 0.0;
      tau += gravity_ff_scale_ * nominal;
    }

    // Live Pinocchio gravity (EDULITE): τ_mit = τ_g_urdf * joint_signs (apply once).
    // /a3/gravity_torque.effort is URDF-frame when gravity_torque_node joint_direction=1.
    if (enable_gravity_compensation_ && has_gravity_sample_ && idx < kNumArmJoints) {
      const int64_t age_ns = this->now().nanoseconds() - last_gravity_stamp_ns_;
      const double age_s = static_cast<double>(age_ns) * 1e-9;
      if (age_s <= gravity_fresh_timeout_s_) {
        const double sign =
          gravity_apply_joint_signs_ && idx < joint_signs_.size() ? joint_signs_[idx] : 1.0;
        const double jscale =
          idx < gravity_joint_scale_.size() ? gravity_joint_scale_[idx] : 1.0;
        tau += gravity_ff_scale_ * jscale * sign * gravity_tau_urdf_[idx];
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Gravity FF skipped: sample age %.3fs > timeout %.3fs",
          age_s, gravity_fresh_timeout_s_);
      }
    }

    return Clamp(tau, ProtocolCodec::kTMin, ProtocolCodec::kTMax);
  }

  bool IsEnabledMode(int mode_status) const
  {
    return mode_status == motor_enabled_mode_status_;
  }

  void ResetSmoothingForJoint(size_t idx, double current_champ_feedback, const rclcpp::Time & now)
  {
    if (idx >= boot_feedback_champ_rad_.size()) {
      return;
    }
    boot_feedback_champ_rad_[idx] = current_champ_feedback;
    boot_feedback_captured_[idx] = true;
    has_last_sent_[idx] = false;
    last_sent_champ_rad_[idx] = current_champ_feedback;
    first_command_time_by_joint_ns_[idx] = now.nanoseconds();
    last_command_time_by_joint_ns_[idx] = now.nanoseconds();
  }

  void ResetRuntimeTuning()
  {
    runtime_kp_can0_ = Clamp(kp_, ProtocolCodec::kKpMin, ProtocolCodec::kKpMax);
    runtime_kp_can1_ = runtime_kp_can0_;
    runtime_kd_can0_ = Clamp(kd_, ProtocolCodec::kKdMin, ProtocolCodec::kKdMax);
    runtime_kd_can1_ = runtime_kd_can0_;
    runtime_tau_can0_ = Clamp(default_tau_ff_, ProtocolCodec::kTMin, ProtocolCodec::kTMax);
    runtime_tau_can1_ = runtime_tau_can0_;
  }

  static double Clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  double ClampJointCommand(size_t idx, double champ_position)
  {
    if (!use_joint_cmd_limits_ || idx >= joint_cmd_min_rad_.size() || idx >= joint_cmd_max_rad_.size()) {
      return champ_position;
    }
    const double clamped = Clamp(champ_position, joint_cmd_min_rad_[idx], joint_cmd_max_rad_[idx]);
    if (std::fabs(clamped - champ_position) > 1e-9) {
      ++joint_cmd_clamp_count_total_;
    } else {
      ++joint_cmd_no_clamp_count_total_;
    }
    return clamped;
  }

  double SmoothJointCommand(size_t idx, double champ_position)
  {
    if (idx >= last_sent_champ_rad_.size()) {
      return champ_position;
    }
    const rclcpp::Time now = this->now();
    const int64_t now_ns = now.nanoseconds();
    if (first_command_time_by_joint_ns_[idx] <= 0) {
      first_command_time_by_joint_ns_[idx] = now_ns;
      last_command_time_by_joint_ns_[idx] = now_ns;
    }

    double target = champ_position;
    bool smoothing_active = false;
    if (
      enable_startup_smoothing_ &&
      startup_smoothing_duration_s_ > 1e-3 &&
      idx < boot_feedback_captured_.size() &&
      boot_feedback_captured_[idx] &&
      std::isfinite(boot_feedback_champ_rad_[idx]))
    {
      const double elapsed = static_cast<double>(now_ns - first_command_time_by_joint_ns_[idx]) * 1e-9;
      const double alpha = Clamp(elapsed / startup_smoothing_duration_s_, 0.0, 1.0);
      target = boot_feedback_champ_rad_[idx] + alpha * (champ_position - boot_feedback_champ_rad_[idx]);
      smoothing_active = (alpha < 0.999);
    }

    if (
      enable_feedback_step_limit_ &&
      feedback_step_limit_rad_ > 1e-6 &&
      idx < last_feedback_champ_rad_.size() &&
      std::isfinite(last_feedback_champ_rad_[idx]))
    {
      const int64_t stamp_ns = last_feedback_stamp_ns_[idx];
      const int64_t fresh_ns = static_cast<int64_t>(feedback_fresh_timeout_s_ * 1e9);
      if (stamp_ns > 0 && (now_ns - stamp_ns) <= fresh_ns) {
        const double delta_fb = target - last_feedback_champ_rad_[idx];
        if (std::fabs(delta_fb) > feedback_step_limit_rad_) {
          target = last_feedback_champ_rad_[idx] + std::copysign(feedback_step_limit_rad_, delta_fb);
        }
      }
    }

    const bool apply_velocity_limit = !limit_velocity_only_during_smoothing_ || smoothing_active;
    const double dt = std::max(
      static_cast<double>(now_ns - last_command_time_by_joint_ns_[idx]) * 1e-9,
      1e-3);
    const double max_step = apply_velocity_limit ?
      (std::max(0.0, command_max_velocity_rad_s_) * dt) :
      std::numeric_limits<double>::infinity();
    if (!has_last_sent_[idx]) {
      if (idx < boot_feedback_captured_.size() && boot_feedback_captured_[idx] && std::isfinite(boot_feedback_champ_rad_[idx])) {
        last_sent_champ_rad_[idx] = boot_feedback_champ_rad_[idx];
      } else {
        last_sent_champ_rad_[idx] = target;
      }
      has_last_sent_[idx] = true;
    }
    const double prev = last_sent_champ_rad_[idx];
    const double delta = Clamp(target - prev, -max_step, max_step);
    const double out = prev + delta;
    last_sent_champ_rad_[idx] = out;
    last_command_time_by_joint_ns_[idx] = now_ns;
    return out;
  }

  void RebuildMotorLimitTable()
  {
    const size_t n = std::min<size_t>(
      kNumArmJoints, runtime_joint_mit_min_rad_.size());
    if (!derive_motor_limits_from_joint_cmd_) {
      for (size_t i = 0; i < n; ++i) {
        const double lo = std::min(joint_mit_min_rad_[i], joint_mit_max_rad_[i]);
        const double hi = std::max(joint_mit_min_rad_[i], joint_mit_max_rad_[i]);
        runtime_joint_mit_min_rad_[i] = lo;
        runtime_joint_mit_max_rad_[i] = hi;
      }
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      const double a = joint_signs_[i] * joint_cmd_min_rad_[i] + joint_offsets_rad_[i];
      const double b = joint_signs_[i] * joint_cmd_max_rad_[i] + joint_offsets_rad_[i];
      runtime_joint_mit_min_rad_[i] = std::min(a, b);
      runtime_joint_mit_max_rad_[i] = std::max(a, b);
    }
  }

  double ClampMotorCommand(size_t idx, double mit_position)
  {
    if (!use_motor_domain_limits_ || idx >= runtime_joint_mit_min_rad_.size()) {
      return mit_position;
    }
    const double clamped = Clamp(mit_position, runtime_joint_mit_min_rad_[idx], runtime_joint_mit_max_rad_[idx]);
    if (std::fabs(clamped - mit_position) > 1e-9) {
      ++motor_cmd_clamp_count_total_;
    } else {
      ++motor_cmd_no_clamp_count_total_;
    }
    return clamped;
  }

  static bool ParseKV(const std::string & token, std::string & key, std::string & value)
  {
    const auto pos = token.find('=');
    if (pos == std::string::npos || pos == 0 || pos + 1 >= token.size()) {
      return false;
    }
    key = ToLower(token.substr(0, pos));
    value = token.substr(pos + 1);
    return true;
  }

  void OnTuneCommand(const String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    auto scope = TuneScope::ALL;
    bool do_reset = false;
    bool has_kp = false;
    bool has_kd = false;
    bool has_tau = false;
    double new_kp = 0.0;
    double new_kd = 0.0;
    double new_tau = 0.0;

    const auto tokens = SplitTokens(msg->data);
    for (const auto & token : tokens) {
      std::string key;
      std::string value;
      if (!ParseKV(token, key, value)) {
        continue;
      }
      if (key == "scope") {
        scope = ParseScope(value);
        continue;
      }
      if (key == "reset") {
        do_reset = ParseBool(value);
        continue;
      }
      try {
        const double parsed = std::stod(value);
        if (key == "kp") {
          new_kp = Clamp(parsed, ProtocolCodec::kKpMin, ProtocolCodec::kKpMax);
          has_kp = true;
        } else if (key == "kd") {
          new_kd = Clamp(parsed, ProtocolCodec::kKdMin, ProtocolCodec::kKdMax);
          has_kd = true;
        } else if (key == "tau" || key == "tau_ff") {
          new_tau = Clamp(parsed, ProtocolCodec::kTMin, ProtocolCodec::kTMax);
          has_tau = true;
        }
      } catch (const std::exception &) {
        RCLCPP_WARN(this->get_logger(), "Ignore invalid tune token: %s", token.c_str());
      }
    }

    if (do_reset) {
      ResetRuntimeTuning();
      RCLCPP_WARN(this->get_logger(), "MIT tune reset to defaults from parameters.");
    }

    auto apply_scoped = [&](double & front, double & rear, double v) {
      if (scope == TuneScope::ALL) {
        front = v;
        rear = v;
      } else if (scope == TuneScope::FRONT) {
        front = v;
      } else {
        rear = v;
      }
    };

    if (has_kp) {
      apply_scoped(runtime_kp_can0_, runtime_kp_can1_, new_kp);
    }
    if (has_kd) {
      apply_scoped(runtime_kd_can0_, runtime_kd_can1_, new_kd);
    }
    if (has_tau) {
      apply_scoped(runtime_tau_can0_, runtime_tau_can1_, new_tau);
    }

    RCLCPP_WARN(
      this->get_logger(),
      "MIT tune applied: raw='%s' | can0(kp=%.2f,kd=%.2f,tau=%.2f) can1(kp=%.2f,kd=%.2f,tau=%.2f)",
      msg->data.c_str(),
      runtime_kp_can0_, runtime_kd_can0_, runtime_tau_can0_,
      runtime_kp_can1_, runtime_kd_can1_, runtime_tau_can1_);
  }

  double kp_{30.0};
  double kd_{1.5};
  double default_velocity_{0.0};
  double default_tau_ff_{0.0};
  std::string runtime_tune_topic_;
  bool publish_feedback_joint_states_{true};
  std::string feedback_joint_states_topic_;
  double feedback_joint_states_timer_hz_{50.0};
  bool publish_feedback_velocity_effort_{true};
  bool feedback_joint_fill_nan_{true};
  bool publish_motor_diagnostics_{false};
  std::string motor_diagnostics_topic_;
  bool enable_joint_tau_ff_{false};
  bool enable_gravity_compensation_{false};
  bool gravity_apply_joint_signs_{true};
  double gravity_ff_scale_{1.0};
  double gravity_fresh_timeout_s_{0.5};
  std::string gravity_compensation_topic_;
  std::vector<double> tau_ff_nominal_nm_;
  std::vector<double> gravity_joint_scale_;
  std::array<double, kNumArmJoints> gravity_tau_urdf_{};
  bool has_gravity_sample_{false};
  int64_t last_gravity_stamp_ns_{0};
  bool use_joint_cmd_limits_{true};
  std::vector<double> joint_cmd_min_rad_;
  std::vector<double> joint_cmd_max_rad_;
  bool use_motor_domain_limits_{true};
  bool derive_motor_limits_from_joint_cmd_{true};
  std::vector<double> joint_mit_min_rad_;
  std::vector<double> joint_mit_max_rad_;
  std::array<double, DogMapper::kTemporaryIndexMap.size()> runtime_joint_mit_min_rad_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> runtime_joint_mit_max_rad_{};
  bool enable_startup_smoothing_{true};
  double startup_smoothing_duration_s_{1.5};
  double command_max_velocity_rad_s_{6.0};
  bool limit_velocity_only_during_smoothing_{true};
  bool enable_mode_rising_smoothing_{true};
  int motor_enabled_mode_status_{2};
  bool enable_feedback_step_limit_{true};
  double feedback_step_limit_rad_{0.20};
  double feedback_fresh_timeout_s_{0.30};
  bool enable_rx_decode_log_{true};
  bool prefer_joint_name_mapping_{true};
  bool enable_power_sequence_gate_{true};
  std::string power_sequence_gate_topic_;
  bool power_gate_open_{false};
  bool tx_enable_can0_{true};
  bool tx_enable_can1_{true};
  bool enable_min_tx_refresh_{true};
  double min_tx_refresh_interval_s_{0.02};
  bool enable_max_tx_rate_limit_{true};
  double max_tx_rate_per_motor_hz_{60.0};
  std::vector<double> joint_signs_;
  std::vector<double> joint_offsets_rad_;
  bool publish_mit_mapped_{false};
  std::string mit_mapped_topic_;
  rclcpp::Publisher<JointState>::SharedPtr mapped_positions_pub_;
  std::array<double, 256> last_commanded_mit_rad_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> last_feedback_champ_rad_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> last_feedback_joint_vel_rad_s_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> last_feedback_effort_nm_{};
  std::array<int, 12> last_feedback_mode_status_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> last_feedback_temp_c_{};
  std::array<uint32_t, 12> last_feedback_fault_mask_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> last_sent_champ_rad_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> latest_input_champ_rad_{};
  std::array<double, DogMapper::kTemporaryIndexMap.size()> boot_feedback_champ_rad_{};
  std::array<bool, 12> has_latest_input_{};
  std::array<bool, 12> boot_feedback_captured_{};
  std::array<bool, 12> has_last_sent_{};
  std::array<int, 256> last_mode_status_{};
  std::array<bool, 256> has_mode_status_{};
  std::array<int64_t, 12> last_feedback_stamp_ns_{};
  std::array<int64_t, 12> first_command_time_by_joint_ns_{};
  std::array<int64_t, 12> last_command_time_by_joint_ns_{};
  std::array<int64_t, 12> last_tx_pub_stamp_ns_{};
  size_t error_sample_count_{0};
  double error_abs_sum_{0.0};
  double error_abs_max_{0.0};
  double runtime_kp_can0_{30.0};
  double runtime_kd_can0_{1.5};
  double runtime_tau_can0_{0.0};
  double runtime_kp_can1_{30.0};
  double runtime_kd_can1_{1.5};
  double runtime_tau_can1_{0.0};
  size_t joint_cmd_clamp_count_total_{0};
  size_t joint_cmd_no_clamp_count_total_{0};
  size_t motor_cmd_clamp_count_total_{0};
  size_t motor_cmd_no_clamp_count_total_{0};
  size_t traj_cb_count_window_{0};
  size_t traj_joint_count_window_{0};
  size_t tx_traj_total_window_{0};
  size_t tx_traj_can0_window_{0};
  size_t tx_traj_can1_window_{0};
  size_t tx_refresh_total_window_{0};
  size_t tx_refresh_can0_window_{0};
  size_t tx_refresh_can1_window_{0};
  size_t skip_max_rate_limit_window_{0};
  size_t skip_bus_disabled_window_{0};
  size_t skip_power_gate_window_{0};

  bool enable_trajectory_interpolation_{true};
  double trajectory_interp_rate_hz_{200.0};
  TrajInterpMethod traj_interp_method_{TrajInterpMethod::Auto};
  bool zero_torque_active_{false};
  double zero_torque_kp_{0.0};
  double zero_torque_kd_{1.0};
  double saved_kp_can0_{30.0};
  double saved_kp_can1_{30.0};
  double saved_kd_can0_{1.5};
  double saved_kd_can1_{1.5};
  std::string control_mode_topic_;
  std::string last_control_mode_{"IDLE"};
  std::mutex traj_mutex_;
  JointTrajectory active_traj_;
  rclcpp::Time traj_start_{0, 0, RCL_ROS_TIME};
  bool has_active_traj_{false};

  rclcpp::Subscription<JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<String>::SharedPtr tune_sub_;
  rclcpp::Subscription<Bool>::SharedPtr gate_sub_;
  rclcpp::Subscription<JointState>::SharedPtr gravity_sub_;
  rclcpp::Subscription<String>::SharedPtr control_mode_sub_;
  rclcpp::Publisher<String>::SharedPtr control_mode_pub_;
  rclcpp::Service<Trigger>::SharedPtr zero_torque_start_srv_;
  rclcpp::Service<Trigger>::SharedPtr zero_torque_stop_srv_;
  rclcpp::Service<MotorCommand>::SharedPtr enable_srv_;
  rclcpp::Service<MotorCommand>::SharedPtr reset_srv_;
  rclcpp::Service<MotorCommand>::SharedPtr set_zero_srv_;
  rclcpp::Service<MotorCommand>::SharedPtr get_device_id_srv_;
  rclcpp::Service<MotorCommand>::SharedPtr request_version_srv_;
  rclcpp::Service<SetCanId>::SharedPtr set_can_id_srv_;
  rclcpp::Service<SetMotorParam>::SharedPtr set_param_srv_;
  rclcpp::Service<MotorScan>::SharedPtr scan_srv_;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<String>::SharedPtr device_id_pub_;
  rclcpp::Publisher<String>::SharedPtr version_pub_;
  rclcpp::Publisher<JointState>::SharedPtr feedback_joint_states_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;
  rclcpp::TimerBase::SharedPtr feedback_js_timer_;
  rclcpp::TimerBase::SharedPtr tx_refresh_timer_;
  rclcpp::TimerBase::SharedPtr tx_stats_timer_;
  rclcpp::TimerBase::SharedPtr traj_interp_timer_;
};

}  // namespace a3_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a3_can_bridge::MotorProtocolNode>());
  rclcpp::shutdown();
  return 0;
}
