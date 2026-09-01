#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

#include "a3_can_bridge/motor_model.hpp"

namespace a3_can_bridge
{

class ProtocolCodec
{
public:
  static constexpr uint8_t kMotorCmdGetDeviceId = 0x00;  // 通信类型 0：获取设备 ID（应答 bit0-7=0xFE）
  static constexpr uint8_t kMotorCmdControl = 0x01;      // MIT 运控
  static constexpr uint8_t kMotorCmdFeedback = 0x02;     // 电机反馈（应答）
  static constexpr uint8_t kMotorCmdEnable = 0x03;       // 使能
  static constexpr uint8_t kMotorCmdReset = 0x04;        // 停止（data[1]=0xC0 触发返回软件版本）
  static constexpr uint8_t kMotorCmdSetZero = 0x06;      // 设置零点（data[0]=1）
  static constexpr uint8_t kMotorCmdSetCanId = 0x07;     // 设置电机 CAN_ID（立即生效）
  static constexpr uint8_t kMotorCmdSetParam = 0x12;     // 设置参数（18）
  static constexpr uint8_t kMotorCmdVersion = 0x17;      // 获取软件版本号（23）
  /// 手册通信类型 24：主动周期上报，29bit ID 高 5 位为 0x18，8B 数据域与 0x02 反馈相同
  static constexpr uint8_t kMotorCmdActiveReport = 0x18;
  static constexpr uint8_t kMasterIdDefault = 0xFD;      // 非运控帧主机号（bit8-15）
  static constexpr uint8_t kGetIdRspMarker = 0xFE;       // 获取设备 ID 应答帧 bit0-7 标记

  // 参数索引（PARAM_*，写参数命令 data[0..1] 小端）
  static constexpr uint16_t kParamSinEnable = 0x7001;
  static constexpr uint16_t kParamSinFreq = 0x7002;
  static constexpr uint16_t kParamSinAmp = 0x7003;
  static constexpr uint16_t kParamRunMode = 0x7005;
  static constexpr uint16_t kParamIqRef = 0x7006;
  static constexpr uint16_t kParamSpdRef = 0x700A;
  static constexpr uint16_t kParamLimitTorque = 0x700B;
  static constexpr uint16_t kParamLocRef = 0x7016;
  static constexpr uint16_t kParamLimitSpd = 0x7017;
  static constexpr uint16_t kParamLimitCur = 0x7018;
  static constexpr uint16_t kParamLocKp = 0x701E;
  static constexpr uint16_t kParamSpdKp = 0x701F;
  static constexpr uint16_t kParamSpdKi = 0x7020;
  static constexpr uint16_t kParamEpScanTime = 0x7026;

  static constexpr float kPMin = -12.57f;
  static constexpr float kPMax = 12.57f;
  static constexpr float kVMin = -50.0f;
  static constexpr float kVMax = 50.0f;
  static constexpr float kKpMin = 0.0f;
  static constexpr float kKpMax = 500.0f;
  static constexpr float kKdMin = 0.0f;
  static constexpr float kKdMax = 5.0f;
  static constexpr float kTMin = -6.0f;
  static constexpr float kTMax = 6.0f;

  static CanFrameMessage BuildMitControlFrame(
    CanBus bus, uint8_t motor_id, float position, float velocity,
    float kp, float kd, float torque_ff)
  {
    CanFrameMessage out;
    out.bus = bus;
    out.is_extended = true;
    out.dlc = 8;

    const uint16_t torque_u16 = FloatToUint16(torque_ff, kTMin, kTMax, 16);
    out.can_id = BuildMitControlCanId(motor_id, torque_u16);

    const uint16_t pos_u16 = FloatToUint16(position, kPMin, kPMax, 16);
    const uint16_t vel_u16 = FloatToUint16(velocity, kVMin, kVMax, 16);
    const uint16_t kp_u16 = FloatToUint16(kp, kKpMin, kKpMax, 16);
    const uint16_t kd_u16 = FloatToUint16(kd, kKdMin, kKdMax, 16);

    out.data[0] = static_cast<uint8_t>((pos_u16 >> 8) & 0xFF);
    out.data[1] = static_cast<uint8_t>(pos_u16 & 0xFF);
    out.data[2] = static_cast<uint8_t>((vel_u16 >> 8) & 0xFF);
    out.data[3] = static_cast<uint8_t>(vel_u16 & 0xFF);
    out.data[4] = static_cast<uint8_t>((kp_u16 >> 8) & 0xFF);
    out.data[5] = static_cast<uint8_t>(kp_u16 & 0xFF);
    out.data[6] = static_cast<uint8_t>((kd_u16 >> 8) & 0xFF);
    out.data[7] = static_cast<uint8_t>(kd_u16 & 0xFF);
    return out;
  }

  static std::optional<MotorFeedback> DecodeFeedback(const CanFrameMessage & frame)
  {
    const uint8_t cmd_type = static_cast<uint8_t>((frame.can_id >> 24) & 0x1F);
    if (cmd_type != kMotorCmdFeedback && cmd_type != kMotorCmdActiveReport) {
      return std::nullopt;
    }

    MotorFeedback fb;
    fb.cmd_type = cmd_type;
    fb.master_id = static_cast<uint8_t>(frame.can_id & 0xFF);
    fb.motor_id = static_cast<uint8_t>((frame.can_id >> 8) & 0xFF);
    fb.error_status = static_cast<uint8_t>(((frame.can_id >> 16) & 0x3F) > 0 ? 1 : 0);
    fb.hall_error = static_cast<bool>((frame.can_id >> 20) & 0x01);
    fb.magnet_error = static_cast<bool>((frame.can_id >> 19) & 0x01);
    fb.temp_error = static_cast<bool>((frame.can_id >> 18) & 0x01);
    fb.current_error = static_cast<bool>((frame.can_id >> 17) & 0x01);
    fb.voltage_error = static_cast<bool>((frame.can_id >> 16) & 0x01);
    fb.mode_status = static_cast<uint8_t>((frame.can_id >> 22) & 0x03);
    fb.current_angle = UintToFloat(U16Be(frame.data[0], frame.data[1]), kPMin, kPMax, 16);
    fb.current_speed = UintToFloat(U16Be(frame.data[2], frame.data[3]), kVMin, kVMax, 16);
    fb.current_torque = UintToFloat(U16Be(frame.data[4], frame.data[5]), kTMin, kTMax, 16);
    fb.current_temp = static_cast<float>(U16Be(frame.data[6], frame.data[7])) / 10.0f;
    fb.source_bus = (frame.bus == CanBus::CAN0) ? "can0" : "can1";
    return fb;
  }

  // ---- 非运控命令组帧（统一走 FrameCodec::Pack + /can_tx_frames） ----

  static CanFrameMessage BuildEnableFrame(CanBus bus, uint8_t motor_id)
  {
    return BuildCommandFrame(bus, motor_id, kMotorCmdEnable);
  }

  static CanFrameMessage BuildResetFrame(CanBus bus, uint8_t motor_id)
  {
    auto out = BuildCommandFrame(bus, motor_id, kMotorCmdReset);
    out.data[1] = 0xC0;  // 触发返回软件版本号
    return out;
  }

  static CanFrameMessage BuildSetZeroFrame(CanBus bus, uint8_t motor_id)
  {
    auto out = BuildCommandFrame(bus, motor_id, kMotorCmdSetZero);
    out.data[0] = 0x01;
    return out;
  }

  static CanFrameMessage BuildSetCanIdFrame(CanBus bus, uint8_t current_id, uint8_t new_id)
  {
    CanFrameMessage out;
    out.bus = bus;
    out.is_extended = true;
    out.dlc = 8;
    // 通信类型 7：bit28-24=0x07；bit23-16=新 CAN_ID；bit15-8=主站 ID；bit7-0=当前 CAN_ID
    out.can_id = (static_cast<uint32_t>(kMotorCmdSetCanId) << 24) |
                 (static_cast<uint32_t>(new_id) << 16) |
                 (static_cast<uint32_t>(kMasterIdDefault) << 8) |
                 static_cast<uint32_t>(current_id);
    return out;
  }

  static CanFrameMessage BuildSetParamFrame(CanBus bus, uint8_t motor_id, uint16_t param_id, float value)
  {
    auto out = BuildCommandFrame(bus, motor_id, kMotorCmdSetParam);
    out.data[0] = static_cast<uint8_t>(param_id & 0xFF);
    out.data[1] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
    const uint32_t bits = FloatBits(value);
    out.data[4] = static_cast<uint8_t>(bits & 0xFF);
    out.data[5] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    out.data[6] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    out.data[7] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    return out;
  }

  static CanFrameMessage BuildSetParamRawFrame(
    CanBus bus, uint8_t motor_id, uint16_t param_id, const std::array<uint8_t, 4> & raw)
  {
    auto out = BuildCommandFrame(bus, motor_id, kMotorCmdSetParam);
    out.data[0] = static_cast<uint8_t>(param_id & 0xFF);
    out.data[1] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
    out.data[4] = raw[0];
    out.data[5] = raw[1];
    out.data[6] = raw[2];
    out.data[7] = raw[3];
    return out;
  }

  static CanFrameMessage BuildActiveReportFrame(CanBus bus, uint8_t motor_id, bool enable)
  {
    auto out = BuildCommandFrame(bus, motor_id, kMotorCmdActiveReport);
    out.data[0] = 0x01;
    out.data[1] = 0x02;
    out.data[2] = 0x03;
    out.data[3] = 0x04;
    out.data[4] = 0x05;
    out.data[5] = 0x06;
    out.data[6] = enable ? 0x01 : 0x00;
    out.data[7] = 0x00;
    return out;
  }

  static CanFrameMessage BuildGetDeviceIdProbeFrame(CanBus bus, uint8_t motor_id)
  {
    return BuildCommandFrame(bus, motor_id, kMotorCmdGetDeviceId);
  }

  static CanFrameMessage BuildRequestVersionFrame(CanBus bus, uint8_t motor_id)
  {
    // 通信类型 4 + data 00 C4：读取软件版本（应答为类型 2/24，data 00 C4 56 + 版本号）
    auto out = BuildCommandFrame(bus, motor_id, kMotorCmdReset);
    out.data[0] = 0x00;
    out.data[1] = 0xC4;
    return out;
  }

  // ---- EPScan 周期换算：period_ms = 10 + (n-1)*5 ----

  static uint8_t PeriodMsToEpScanN(uint32_t period_ms)
  {
    if (period_ms < 10) {
      period_ms = 10;
    }
    return static_cast<uint8_t>((period_ms - 10) / 5 + 1);
  }

  static uint32_t EpScanNToPeriodMs(uint8_t n)
  {
    if (n < 1) {
      n = 1;
    }
    return 10 + static_cast<uint32_t>(n - 1) * 5;
  }

  // ---- RX 解析扩展 ----

  static std::optional<DeviceIdResponse> DecodeDeviceIdResponse(const CanFrameMessage & frame)
  {
    const uint8_t cmd_type = static_cast<uint8_t>((frame.can_id >> 24) & 0x1F);
    if (cmd_type != kMotorCmdGetDeviceId) {
      return std::nullopt;
    }
    if (static_cast<uint8_t>(frame.can_id & 0xFF) != kGetIdRspMarker) {
      return std::nullopt;
    }
    DeviceIdResponse rsp;
    rsp.motor_id = static_cast<uint8_t>((frame.can_id >> 8) & 0xFF);
    rsp.mcu_uid = 0;
    for (int i = 0; i < 8; ++i) {
      rsp.mcu_uid = (rsp.mcu_uid << 8) | frame.data[i];
    }
    return rsp;
  }

  static bool IsSoftwareVersionResponse(const CanFrameMessage & frame)
  {
    const uint8_t cmd_type = static_cast<uint8_t>((frame.can_id >> 24) & 0x1F);
    return (cmd_type == kMotorCmdFeedback || cmd_type == kMotorCmdActiveReport) &&
           frame.data[0] == 0x00 && frame.data[1] == 0xC4 && frame.data[2] == 0x56;
  }

  static std::string DecodeVersionString(const CanFrameMessage & frame)
  {
    if (frame.data[0] == 0x00 && frame.data[1] == 0xC4 && frame.data[2] == 0x56) {
      char buf[16];
      std::snprintf(
        buf, sizeof(buf), "%u.%u.%u.%u",
        static_cast<unsigned>(frame.data[3]), static_cast<unsigned>(frame.data[4]),
        static_cast<unsigned>(frame.data[5]), static_cast<unsigned>(frame.data[6]));
      return std::string(buf);
    }
    std::string s;
    s.reserve(8);
    for (int i = 0; i < 8; ++i) {
      const char c = static_cast<char>(frame.data[i]);
      if (c == '\0') {
        break;
      }
      s.push_back(c);
    }
    return s;
  }

  static std::string FrameSummary(const CanFrameMessage & frame)
  {
    std::ostringstream oss;
    oss << ((frame.bus == CanBus::CAN0) ? "can0" : "can1")
        << " id=0x" << std::hex << frame.can_id << std::dec
        << " dlc=" << static_cast<int>(frame.dlc) << " data=[";
    for (size_t i = 0; i < frame.data.size(); ++i) {
      if (i) {
        oss << " ";
      }
      oss << static_cast<int>(frame.data[i]);
    }
    oss << "]";
    return oss.str();
  }

  /// Human-readable single line: bus, std/ext, CAN id + DLC + payload (hex).
  /// Example: TX can1 EXT id=0x017FFF0D len=08 75 4B 7F FF 0F 5C 4C CC
  static std::string FrameLineHex(bool is_tx, const CanFrameMessage & frame)
  {
    std::ostringstream oss;
    oss << (is_tx ? "TX " : "RX ")
        << ((frame.bus == CanBus::CAN0) ? "can0 " : "can1 ")
        << (frame.is_extended ? "EXT " : "STD ")
        << "id=0x";

    oss << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
        << static_cast<unsigned long>(frame.can_id & 0xFFFFFFFFu)
        << std::dec << std::setfill(' ')
        << " len=" << std::setw(2) << std::setfill('0')
        << static_cast<unsigned>(frame.dlc);

    const unsigned n = std::min<unsigned>(frame.dlc, 8u);
    for (unsigned i = 0; i < n; ++i) {
      oss << ' ' << std::setw(2) << std::setfill('0') << std::hex << std::uppercase
          << static_cast<unsigned>(frame.data[i]);
    }
    oss << std::dec << std::setfill(' ');
    return oss.str();
  }

private:
  /// 非运控命令 29 位 ID：cmd(bit24-28) | master_id(bit8-15) | motor_id(bit0-7)
  static CanFrameMessage BuildCommandFrame(CanBus bus, uint8_t motor_id, uint8_t cmd)
  {
    CanFrameMessage out;
    out.bus = bus;
    out.is_extended = true;
    out.dlc = 8;
    out.can_id = (static_cast<uint32_t>(cmd) << 24) |
                 (static_cast<uint32_t>(kMasterIdDefault) << 8) |
                 static_cast<uint32_t>(motor_id);
    return out;
  }

  static uint32_t FloatBits(float value)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
  }

  static uint16_t FloatToUint16(float x, float x_min, float x_max, int bits)
  {
    const float clamped = std::max(x_min, std::min(x, x_max));
    const float span = x_max - x_min;
    const float norm = (clamped - x_min) / span;
    const uint32_t max_u = static_cast<uint32_t>((1u << bits) - 1u);
    return static_cast<uint16_t>(norm * static_cast<float>(max_u));
  }

  static float UintToFloat(uint16_t x, float x_min, float x_max, int bits)
  {
    const float span = x_max - x_min;
    const float max_u = static_cast<float>((1u << bits) - 1u);
    return static_cast<float>(x) * span / max_u + x_min;
  }

  static uint16_t U16Be(uint8_t high, uint8_t low)
  {
    return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
  }

  static uint32_t BuildMitControlCanId(uint8_t motor_id, uint16_t torque_u16)
  {
    uint32_t id = 0;
    id |= (static_cast<uint32_t>(kMotorCmdControl) << 24);
    id |= (static_cast<uint32_t>(torque_u16) << 8);
    id |= static_cast<uint32_t>(motor_id);
    return id & 0x1FFFFFFFu;
  }
};

}  // namespace a3_can_bridge
