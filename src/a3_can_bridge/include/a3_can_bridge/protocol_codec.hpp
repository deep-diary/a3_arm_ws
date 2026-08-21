#pragma once

#include <algorithm>
#include <cstdint>
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
  static constexpr uint8_t kMotorCmdControl = 0x01;   // MIT 运控
  static constexpr uint8_t kMotorCmdFeedback = 0x02;  // 电机反馈（应答）
  /// 手册通信类型 24：主动周期上报，29bit ID 高 5 位为 0x18，8B 数据域与 0x02 反馈相同
  static constexpr uint8_t kMotorCmdActiveReport = 0x18;

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
