#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <cctype>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "a3_can_bridge/arm_mapper.hpp"
#include "a3_can_bridge/frame_codec.hpp"
#include "a3_can_bridge/protocol_codec.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using sensor_msgs::msg::Joy;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using std_msgs::msg::UInt8MultiArray;

namespace a3_can_bridge
{
namespace
{
uint16_t HzToEpScanTime(double hz)
{
  if (hz <= 0.0) {
    hz = 5.0;
  }
  const double period_ms = 1000.0 / hz;
  int n = static_cast<int>(std::lround((period_ms - 10.0) / 5.0 + 1.0));
  n = std::max(1, std::min(n, 65535));
  return static_cast<uint16_t>(n);
}
}  // namespace

enum class SequenceState
{
  Idle = 0,
  Precheck = 1,
  EnableInit = 2,
  SoftStand = 3,
  Running = 4,
  SoftProne = 5,
  ProneHold = 6,
  Disable = 7,
  Fault = 8,
};

class PowerSequenceNode : public rclcpp::Node
{
public:
  PowerSequenceNode()
  : Node("power_sequence_node")
  {
    gate_topic_ = this->declare_parameter<std::string>("gate_topic", "/power_sequence/gate_open");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/power_sequence/state");
    tx_topic_ = this->declare_parameter<std::string>("tx_topic", "/can_tx_frames");
    body_pose_topic_ = this->declare_parameter<std::string>("body_pose_topic", "body_pose");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    command_topic_ = this->declare_parameter<std::string>("command_topic", "/power_sequence/command");

    timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 20);
    precheck_duration_s_ = this->declare_parameter<double>("precheck_duration_s", 0.20);
    init_hold_duration_s_ = this->declare_parameter<double>("init_hold_duration_s", 0.35);
    stand_duration_s_ = this->declare_parameter<double>("stand_duration_s", 2.50);
    prone_duration_s_ = this->declare_parameter<double>("prone_duration_s", 2.50);

    stand_z_ = this->declare_parameter<double>("stand_z", 0.0);
    prone_z_ = this->declare_parameter<double>("prone_z", -0.35);
    start_longpress_s_ = this->declare_parameter<double>("start_longpress_s", 1.0);
    shutdown_longpress_s_ = this->declare_parameter<double>("shutdown_longpress_s", 1.0);
    prone_longpress_s_ = this->declare_parameter<double>("prone_longpress_s", 0.8);

    button_l1_ = this->declare_parameter<int>("button_l1", 4);
    button_r1_ = this->declare_parameter<int>("button_r1", 5);
    button_share_ = this->declare_parameter<int>("button_share", 8);
    button_circle_ = this->declare_parameter<int>("button_circle", 1);
    button_square_ = this->declare_parameter<int>("button_square", 3);

    motor_master_id_ = this->declare_parameter<int>("motor_master_id", 253);

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

    init_kp_ = this->declare_parameter<double>("init_kp", 20.0);
    init_kd_ = this->declare_parameter<double>("init_kd", 1.5);
    init_velocity_ = this->declare_parameter<double>("init_velocity", 0.0);
    init_tau_ = this->declare_parameter<double>("init_tau", 0.0);
    send_set_zero_in_startup_ = this->declare_parameter<bool>("send_set_zero_in_startup", false);
    send_enable_in_startup_ = this->declare_parameter<bool>("send_enable_in_startup", true);
    send_mit_zero_in_startup_ = this->declare_parameter<bool>("send_mit_zero_in_startup", true);
    enable_active_report_in_startup_ = this->declare_parameter<bool>("enable_active_report_in_startup", true);
    active_report_hz_ = this->declare_parameter<double>("active_report_hz", 10.0);
    active_report_master_id_ = this->declare_parameter<int>("active_report_master_id", -1);
    enable_active_report_at_launch_ = this->declare_parameter<bool>("enable_active_report_at_launch", true);
    active_report_boot_delay_s_ = this->declare_parameter<double>("active_report_boot_delay_s", 0.25);

    enable_remote_set_zero_ = this->declare_parameter<bool>("enable_remote_set_zero", true);
    button_option_ = this->declare_parameter<int>("button_option", 9);
    set_zero_longpress_s_ = this->declare_parameter<double>("set_zero_longpress_s", 2.0);
    set_zero_event_topic_ = this->declare_parameter<std::string>("set_zero_event_topic", "");
    set_zero_repeat_count_ = this->declare_parameter<int>("set_zero_repeat_count", 3);
    set_zero_repeat_interval_ms_ = this->declare_parameter<int>("set_zero_repeat_interval_ms", 40);

    track_body_pose_height_ = this->declare_parameter<bool>("track_body_pose_height", true);
    body_pose_stale_s_ = this->declare_parameter<double>("body_pose_stale_s", 2.0);
    track_z_clamp_min_ = this->declare_parameter<double>("track_z_clamp_min", -0.5);
    track_z_clamp_max_ = this->declare_parameter<double>("track_z_clamp_max", 0.5);

    const int can_tx_depth = this->declare_parameter<int>("can_tx_frames_qos_depth", 4000);
    rclcpp::QoS tx_qos(static_cast<size_t>(std::max(1, can_tx_depth)));
    tx_qos.reliable();
    tx_pub_ = this->create_publisher<UInt8MultiArray>(tx_topic_, tx_qos);
    pose_pub_ = this->create_publisher<Pose>(body_pose_topic_, 10);
    cmd_vel_pub_ = this->create_publisher<Twist>(cmd_vel_topic_, 10);
    gate_pub_ = this->create_publisher<Bool>(gate_topic_, rclcpp::QoS(1).reliable().transient_local());
    // 与 status_led 等 transient_local 订阅对齐：晚启动节点仍可收到最后一帧状态（否则易卡在 Idle 灯语）。
    state_pub_ = this->create_publisher<String>(state_topic_, rclcpp::QoS(1).reliable().transient_local());
    if (!set_zero_event_topic_.empty()) {
      event_pub_ = this->create_publisher<String>(set_zero_event_topic_, 10);
    }
    joy_sub_ = this->create_subscription<Joy>(joy_topic_, 10, std::bind(&PowerSequenceNode::OnJoy, this, std::placeholders::_1));
    command_sub_ = this->create_subscription<String>(
      command_topic_, 10, std::bind(&PowerSequenceNode::OnCommand, this, std::placeholders::_1));
    if (track_body_pose_height_) {
      body_pose_sub_ = this->create_subscription<Pose>(
        body_pose_topic_, rclcpp::QoS(10),
        std::bind(&PowerSequenceNode::OnBodyPose, this, std::placeholders::_1));
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(5, timer_period_ms_)),
      std::bind(&PowerSequenceNode::OnTimer, this));

    if (enable_active_report_in_startup_ && enable_active_report_at_launch_) {
      const int boot_ms = std::max(0, static_cast<int>(std::lround(active_report_boot_delay_s_ * 1000.0)));
      boot_report_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(boot_ms),
        std::bind(&PowerSequenceNode::OnBootActiveReportOnce, this));
    }

    PublishGate();
    PublishState();
    RCLCPP_INFO(this->get_logger(), "Power sequence node started: state=Idle gate=closed");
  }

private:
  static constexpr uint8_t kCmdEnable = 0x03;
  static constexpr uint8_t kCmdReset = 0x04;
  static constexpr uint8_t kCmdSetZero = 0x06;
  static constexpr uint8_t kCmdSetParam = 18;
  /// 手册通信类型 24，CAN ID 高 5 位为 0x18
  static constexpr uint8_t kCommActiveReport = 24;
  static constexpr uint16_t kParamEpScanTime = 0x7026;

  void OnBootActiveReportOnce()
  {
    if (boot_active_report_done_) {
      return;
    }
    boot_active_report_done_ = true;
    if (boot_report_timer_) {
      boot_report_timer_->cancel();
    }
    const uint16_t scan = HzToEpScanTime(active_report_hz_);
    SendEpScanTimeAll(scan);
    SendActiveReportAll(true);
    RCLCPP_INFO(
      this->get_logger(),
      "active report at launch (Idle): EPScan_time=%u (~%.2f Hz), COMM 0x18 ON — before start, for RViz /joint_states_feedback",
      static_cast<unsigned>(scan), active_report_hz_);
  }

  void OnJoy(const Joy::SharedPtr msg)
  {
    latest_joy_ = *msg;
    has_joy_ = true;
  }

  static std::string ToLower(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    return value;
  }

  void OnCommand(const String::SharedPtr msg)
  {
    const std::string command = ToLower(msg->data);
    if (command == "start") {
      if (state_ == SequenceState::Idle) {
        use_stand_resume_for_softstand_ = false;
        EnterState(SequenceState::Precheck);
      } else if (state_ == SequenceState::ProneHold) {
        EnterState(SequenceState::SoftStand);
      } else if (state_ == SequenceState::SoftProne && !soft_prone_then_disable_) {
        // 趴下动画未结束就发 start：原先会被忽略，导致“要发两次才站起来”
        pending_start_after_prone_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "command=start latched during SoftProne(prone path): will enter SoftStand when prone motion completes");
      } else {
        RCLCPP_WARN(this->get_logger(), "ignore command=start in state=%s", StateName(state_));
      }
      return;
    }
    if (command == "prone") {
      if (state_ == SequenceState::Running) {
        BeginSoftProne(false);
      } else {
        RCLCPP_WARN(this->get_logger(), "ignore command=prone in state=%s", StateName(state_));
      }
      return;
    }
    if (command == "shutdown") {
      if (state_ == SequenceState::ProneHold) {
        EnterState(SequenceState::Disable);
        return;
      }
      if (
        state_ == SequenceState::Idle || state_ == SequenceState::Precheck ||
        state_ == SequenceState::EnableInit || state_ == SequenceState::SoftStand ||
        state_ == SequenceState::Running || state_ == SequenceState::SoftProne)
      {
        BeginSoftProne(true);
        return;
      }
      RCLCPP_WARN(this->get_logger(), "ignore command=shutdown in state=%s", StateName(state_));
      return;
    }
    if (command == "set_zero") {
      (void)TryRemoteSetZero("command");
      return;
    }
    RCLCPP_WARN(
      this->get_logger(), "unknown command=%s (supported: start/prone/shutdown/set_zero)", command.c_str());
  }

  bool ButtonOn(int idx) const
  {
    if (!has_joy_ || idx < 0 || static_cast<size_t>(idx) >= latest_joy_.buttons.size()) {
      return false;
    }
    return latest_joy_.buttons[static_cast<size_t>(idx)] != 0;
  }

  void OnTimer()
  {
    const double dt = static_cast<double>(std::max(5, timer_period_ms_)) / 1000.0;

    // RQ-024：PS4 Options（可配置）长按 → 12 路机械 set_zero；仅 TryRemoteSetZero 内允许 Idle+门禁关
    const bool option_btn =
      (button_option_ >= 0) && enable_remote_set_zero_ && ButtonOn(button_option_);
    option_hold_s_ = option_btn ? (option_hold_s_ + dt) : 0.0;
    if (!option_btn) {
      option_action_done_this_hold_ = false;
    }
    if (option_btn && option_hold_s_ >= set_zero_longpress_s_ && !option_action_done_this_hold_) {
      (void)TryRemoteSetZero("joy");
      option_action_done_this_hold_ = true;
    }

    const bool l1 = ButtonOn(button_l1_);
    const bool r1 = ButtonOn(button_r1_);
    const bool share = ButtonOn(button_share_);
    const bool circle = ButtonOn(button_circle_);
    const bool square = (button_square_ >= 0) && ButtonOn(button_square_);

    // □ 长按与 L1+R1（且非 Share）共用 start_longpress_s，等价话题 start
    const bool start_combo = (l1 && r1 && !share) || square;
    const bool shutdown_combo = l1 && r1 && share;
    const bool prone_combo = circle;

    start_hold_s_ = start_combo ? (start_hold_s_ + dt) : 0.0;
    shutdown_hold_s_ = shutdown_combo ? (shutdown_hold_s_ + dt) : 0.0;
    prone_hold_s_ = prone_combo ? (prone_hold_s_ + dt) : 0.0;

    if (!start_combo) {
      start_latch_ = false;
    }
    if (!shutdown_combo) {
      shutdown_latch_ = false;
    }
    if (!prone_combo) {
      prone_latch_ = false;
    }

    if (state_ == SequenceState::Idle) {
      if (!start_latch_ && start_hold_s_ >= start_longpress_s_) {
        start_latch_ = true;
        use_stand_resume_for_softstand_ = false;
        EnterState(SequenceState::Precheck);
        return;
      }
      if (!shutdown_latch_ && shutdown_hold_s_ >= shutdown_longpress_s_) {
        shutdown_latch_ = true;
        BeginSoftProne(true);
        return;
      }
      return;
    }

    if (state_ == SequenceState::Running) {
      if (!shutdown_latch_ && shutdown_hold_s_ >= shutdown_longpress_s_) {
        shutdown_latch_ = true;
        BeginSoftProne(true);
      } else if (!prone_latch_ && prone_hold_s_ >= prone_longpress_s_) {
        prone_latch_ = true;
        BeginSoftProne(false);
      }
      return;
    }

    if (state_ == SequenceState::ProneHold) {
      if (!shutdown_latch_ && shutdown_hold_s_ >= shutdown_longpress_s_) {
        shutdown_latch_ = true;
        EnterState(SequenceState::Disable);
      } else if (!start_latch_ && start_hold_s_ >= start_longpress_s_) {
        start_latch_ = true;
        EnterState(SequenceState::SoftStand);
      }
      TickState(dt);
      return;
    }

    if (
      state_ == SequenceState::SoftStand || state_ == SequenceState::EnableInit ||
      state_ == SequenceState::Precheck)
    {
      if (!shutdown_latch_ && shutdown_hold_s_ >= shutdown_longpress_s_) {
        shutdown_latch_ = true;
        BeginSoftProne(true);
        return;
      }
    }

    if (state_ == SequenceState::SoftProne && !soft_prone_then_disable_) {
      if (!start_latch_ && start_hold_s_ >= start_longpress_s_) {
        start_latch_ = true;
        pending_start_after_prone_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "joy start latched during SoftProne(prone path): will enter SoftStand when prone motion completes");
      }
    }

    TickState(dt);
  }

  void TickState(double dt)
  {
    elapsed_in_state_s_ += dt;
    switch (state_) {
      case SequenceState::Precheck:
        if (elapsed_in_state_s_ >= precheck_duration_s_) {
          EnterState(SequenceState::EnableInit);
        }
        break;
      case SequenceState::EnableInit:
        if (!init_sent_) {
          DoEnableInitFrames();
          init_sent_ = true;
        }
        PublishHoldPose(prone_z_);
        if (elapsed_in_state_s_ >= init_hold_duration_s_) {
          EnterState(SequenceState::SoftStand);
        }
        break;
      case SequenceState::SoftStand: {
          SetGateOpen(true);
          const double alpha = stand_duration_s_ > 1e-6 ? std::min(1.0, elapsed_in_state_s_ / stand_duration_s_) : 1.0;
          const double z_end = use_stand_resume_for_softstand_ ? stand_resume_z_ : stand_z_;
          const double z = prone_z_ + alpha * (z_end - prone_z_);
          PublishHoldPose(z);
          if (alpha >= 0.999) {
            EnterState(SequenceState::Running);
          }
        }
        break;
      case SequenceState::SoftProne: {
          SetGateOpen(true);
          const double alpha = prone_duration_s_ > 1e-6 ? std::min(1.0, elapsed_in_state_s_ / prone_duration_s_) : 1.0;
          const double z = soft_prone_z0_ + alpha * (prone_z_ - soft_prone_z0_);
          PublishHoldPose(z);
          if (alpha >= 0.999) {
            if (soft_prone_then_disable_) {
              EnterState(SequenceState::Disable);
            } else if (pending_start_after_prone_) {
              pending_start_after_prone_ = false;
              EnterState(SequenceState::SoftStand);
            } else {
              EnterState(SequenceState::ProneHold);
            }
          }
        }
        break;
      case SequenceState::ProneHold:
        SetGateOpen(true);
        PublishHoldPose(prone_z_);
        break;
      case SequenceState::Disable:
        if (!disable_sent_) {
          // 不下发「上报 OFF」：shutdown 后 RViz/观测仍依赖 0x18 刷新关节态；需要关上报请用手动脚本 reset
          SendCmdAll(kCmdReset);
          disable_sent_ = true;
        }
        SetGateOpen(false);
        PublishHoldPose(prone_z_);
        if (elapsed_in_state_s_ >= 0.25) {
          EnterState(SequenceState::Idle);
        }
        break;
      case SequenceState::Fault:
      case SequenceState::Idle:
      case SequenceState::Running:
      default:
        break;
    }
  }

  void BeginSoftProne(bool then_disable)
  {
    pending_start_after_prone_ = false;
    soft_prone_then_disable_ = then_disable;
    const double raw = CurrentHeightCmdZ();
    soft_prone_z0_ = ClampTrackedZ0(raw);
    if (!then_disable) {
      stand_resume_z_ = soft_prone_z0_;
      use_stand_resume_for_softstand_ = true;
    } else {
      use_stand_resume_for_softstand_ = false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "SoftProne anchor z0=%.4f (tracked, then_disable=%d)", soft_prone_z0_, then_disable ? 1 : 0);
    EnterState(SequenceState::SoftProne);
  }

  void OnBodyPose(const Pose::SharedPtr msg)
  {
    if (!std::isfinite(msg->position.z)) {
      return;
    }
    last_body_pose_cmd_z_ = msg->position.z;
    last_body_pose_receive_time_ = this->now();
    has_body_pose_sample_ = true;
  }

  double CurrentHeightCmdZ()
  {
    if (!track_body_pose_height_) {
      return stand_z_;
    }
    if (has_body_pose_sample_) {
      const double age = (this->now() - last_body_pose_receive_time_).seconds();
      if (age <= body_pose_stale_s_) {
        return last_body_pose_cmd_z_;
      }
    }
    if (has_last_published_cmd_z_) {
      return last_published_cmd_z_;
    }
    return stand_z_;
  }

  double ClampTrackedZ0(double raw) const
  {
    return std::clamp(raw, track_z_clamp_min_, track_z_clamp_max_);
  }

  void EnterState(SequenceState next)
  {
    state_ = next;
    elapsed_in_state_s_ = 0.0;
    init_sent_ = false;
    disable_sent_ = false;
    PublishState();
    RCLCPP_WARN(this->get_logger(), "power_sequence state -> %s", StateName(state_));
  }

  const char * StateName(SequenceState s) const
  {
    switch (s) {
      case SequenceState::Idle: return "Idle";
      case SequenceState::Precheck: return "Precheck";
      case SequenceState::EnableInit: return "EnableInit";
      case SequenceState::SoftStand: return "SoftStand";
      case SequenceState::Running: return "Running";
      case SequenceState::SoftProne: return "SoftProne";
      case SequenceState::ProneHold: return "ProneHold";
      case SequenceState::Disable: return "Disable";
      case SequenceState::Fault: return "Fault";
      default: return "Unknown";
    }
  }

  void PublishState()
  {
    String msg;
    msg.data = StateName(state_);
    state_pub_->publish(msg);
  }

  void PublishGate()
  {
    Bool msg;
    msg.data = gate_open_;
    gate_pub_->publish(msg);
  }

  /** 仅在实际跳变时发布，避免 TickState 每周期 PublishGate 触发下游（如 status_led）狂刷 WS2812。 */
  void SetGateOpen(bool open)
  {
    if (gate_open_ == open) {
      return;
    }
    gate_open_ = open;
    PublishGate();
  }

  void PublishHoldPose(double z)
  {
    last_published_cmd_z_ = z;
    has_last_published_cmd_z_ = true;

    Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose_pub_->publish(pose);

    Twist tw;
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    tw.linear.z = 0.0;
    tw.angular.x = 0.0;
    tw.angular.y = 0.0;
    tw.angular.z = 0.0;
    cmd_vel_pub_->publish(tw);
  }

  void DoEnableInitFrames()
  {
    if (send_set_zero_in_startup_) {
      SendSetZeroAll();
    }
    if (enable_active_report_in_startup_ && send_enable_in_startup_) {
      const uint16_t scan = HzToEpScanTime(active_report_hz_);
      SendEpScanTimeAll(scan);
      RCLCPP_INFO(
        this->get_logger(),
        "active report: EPScan_time=%u (~%.2f Hz requested) before enable",
        static_cast<unsigned>(scan), active_report_hz_);
    }
    if (send_enable_in_startup_) {
      SendCmdAll(kCmdEnable);
    }
    if (send_mit_zero_in_startup_) {
      SendMitZeroAll();
    }
    if (enable_active_report_in_startup_ && send_enable_in_startup_) {
      SendActiveReportAll(true);
      RCLCPP_INFO(this->get_logger(), "active report: COMM 0x18 ON (all motors)");
    }
  }

  int ReportMasterForCanId() const
  {
    return active_report_master_id_ >= 0 ? active_report_master_id_ : motor_master_id_;
  }

  void SendEpScanTimeAll(uint16_t scan_time)
  {
    const uint8_t p_lo = static_cast<uint8_t>(kParamEpScanTime & 0xFF);
    const uint8_t p_hi = static_cast<uint8_t>((kParamEpScanTime >> 8) & 0xFF);
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      CanFrameMessage frame = BuildCmdFrame(ArmMapper::ArmBus(), route.motor_id, kCmdSetParam, motor_master_id_);
      frame.data[0] = p_lo;
      frame.data[1] = p_hi;
      frame.data[2] = 0;
      frame.data[3] = 0;
      frame.data[4] = static_cast<uint8_t>(scan_time & 0xFF);
      frame.data[5] = static_cast<uint8_t>((scan_time >> 8) & 0xFF);
      frame.data[6] = 0;
      frame.data[7] = 0;
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  void SendActiveReportAll(bool on)
  {
    static constexpr std::array<uint8_t, 6> kReportPrefix{0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    const int rmaster = ReportMasterForCanId();
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      CanFrameMessage frame = BuildCmdFrame(ArmMapper::ArmBus(), route.motor_id, kCommActiveReport, rmaster);
      for (size_t i = 0; i < kReportPrefix.size(); ++i) {
        frame.data[i] = kReportPrefix[i];
      }
      frame.data[6] = on ? 0x01 : 0x00;
      frame.data[7] = 0x00;
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  void PublishSetZeroEvent(const std::string & payload)
  {
    if (!event_pub_) {
      return;
    }
    String msg;
    msg.data = payload;
    event_pub_->publish(msg);
  }

  bool TryRemoteSetZero(const char * trigger)
  {
    if (!enable_remote_set_zero_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "reject set_zero (%s): enable_remote_set_zero=false", trigger);
      return false;
    }
    if (state_ != SequenceState::Idle || gate_open_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "reject set_zero (%s): require state=Idle and gate_open=false (now state=%s gate_open=%d)",
        trigger, StateName(state_), gate_open_ ? 1 : 0);
      std::ostringstream oss;
      oss << "set_zero_rejected:" << StateName(state_) << ":gate=" << (gate_open_ ? 1 : 0) << ":" << trigger;
      PublishSetZeroEvent(oss.str());
      return false;
    }
    const int rounds = std::clamp(set_zero_repeat_count_, 1, 10);
    const int gap_ms = std::max(0, set_zero_repeat_interval_ms_);
    for (int r = 0; r < rounds; ++r) {
      SendSetZeroAll();
      if (r + 1 < rounds && gap_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
      }
    }
    RCLCPP_WARN(
      this->get_logger(),
      "set_zero: sent CMD 0x06 to all 12 motors %d round(s), gap_ms=%d (trigger=%s). Mechanical zero redefined — verify URDF/joint_offsets vs EL05 layer.",
      rounds, gap_ms, trigger);
    PublishSetZeroEvent(std::string("set_zero_ok:") + trigger);
    return true;
  }

  void SendSetZeroAll()
  {
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      CanFrameMessage frame = BuildCmdFrame(ArmMapper::ArmBus(), route.motor_id, kCmdSetZero, motor_master_id_);
      frame.data[0] = 0x01;
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  void SendCmdAll(uint8_t cmd)
  {
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      tx_pub_->publish(FrameCodec::Pack(BuildCmdFrame(ArmMapper::ArmBus(), route.motor_id, cmd, motor_master_id_)));
    }
  }

  void SendMitZeroAll()
  {
    for (const auto & route : DogMapper::kTemporaryIndexMap) {
      auto frame = ProtocolCodec::BuildMitControlFrame(
        ArmMapper::ArmBus(),
        route.motor_id,
        0.0f,
        static_cast<float>(init_velocity_),
        static_cast<float>(init_kp_),
        static_cast<float>(init_kd_),
        static_cast<float>(init_tau_));
      tx_pub_->publish(FrameCodec::Pack(frame));
    }
  }

  CanFrameMessage BuildCmdFrame(CanBus bus, uint8_t motor_id, uint8_t cmd, int master_id) const
  {
    CanFrameMessage frame;
    frame.bus = bus;
    frame.is_extended = true;
    frame.dlc = 8;
    frame.can_id = ((static_cast<uint32_t>(cmd) << 24) |
      (static_cast<uint32_t>(master_id & 0xFF) << 8) |
      static_cast<uint32_t>(motor_id));
    frame.data.fill(0);
    return frame;
  }

  std::string gate_topic_;
  std::string state_topic_;
  std::string tx_topic_;
  std::string body_pose_topic_;
  std::string cmd_vel_topic_;
  std::string joy_topic_;
  std::string command_topic_;

  int timer_period_ms_{20};
  double precheck_duration_s_{0.2};
  double init_hold_duration_s_{0.35};
  double stand_duration_s_{2.5};
  double prone_duration_s_{2.5};
  double stand_z_{0.0};
  double prone_z_{-0.35};
  double start_longpress_s_{1.0};
  double shutdown_longpress_s_{1.0};
  double prone_longpress_s_{0.8};

  int button_l1_{4};
  int button_r1_{5};
  int button_share_{8};
  int button_circle_{1};
  int button_square_{3};

  int motor_master_id_{253};
  double init_kp_{20.0};
  double init_kd_{1.5};
  double init_velocity_{0.0};
  double init_tau_{0.0};
  bool send_set_zero_in_startup_{false};
  bool send_enable_in_startup_{true};
  bool send_mit_zero_in_startup_{true};
  bool enable_active_report_in_startup_{true};
  double active_report_hz_{10.0};
  int active_report_master_id_{-1};
  bool enable_active_report_at_launch_{true};
  double active_report_boot_delay_s_{0.25};

  bool enable_remote_set_zero_{true};
  int button_option_{9};
  double set_zero_longpress_s_{2.0};
  std::string set_zero_event_topic_;
  int set_zero_repeat_count_{3};
  int set_zero_repeat_interval_ms_{40};
  double option_hold_s_{0.0};
  bool option_action_done_this_hold_{false};

  bool track_body_pose_height_{true};
  double body_pose_stale_s_{2.0};
  double track_z_clamp_min_{-0.5};
  double track_z_clamp_max_{0.5};
  double last_body_pose_cmd_z_{0.0};
  rclcpp::Time last_body_pose_receive_time_;
  bool has_body_pose_sample_{false};
  double last_published_cmd_z_{0.0};
  bool has_last_published_cmd_z_{false};
  double soft_prone_z0_{0.0};
  double stand_resume_z_{0.0};
  bool use_stand_resume_for_softstand_{false};

  bool has_joy_{false};
  Joy latest_joy_;
  bool soft_prone_then_disable_{true};
  bool pending_start_after_prone_{false};
  SequenceState state_{SequenceState::Idle};
  double elapsed_in_state_s_{0.0};
  bool gate_open_{false};
  bool init_sent_{false};
  bool disable_sent_{false};

  double start_hold_s_{0.0};
  double shutdown_hold_s_{0.0};
  double prone_hold_s_{0.0};
  bool start_latch_{false};
  bool shutdown_latch_{false};
  bool prone_latch_{false};

  rclcpp::Publisher<UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<Bool>::SharedPtr gate_pub_;
  rclcpp::Publisher<String>::SharedPtr state_pub_;
  rclcpp::Publisher<String>::SharedPtr event_pub_;
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<String>::SharedPtr command_sub_;
  rclcpp::Subscription<Pose>::SharedPtr body_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr boot_report_timer_;
  bool boot_active_report_done_{false};
};

}  // namespace a3_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a3_can_bridge::PowerSequenceNode>());
  rclcpp::shutdown();
  return 0;
}

