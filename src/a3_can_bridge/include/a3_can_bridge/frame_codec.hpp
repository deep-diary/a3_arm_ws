#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "a3_can_bridge/motor_model.hpp"

namespace a3_can_bridge
{

class FrameCodec
{
public:
  // 固定 15 字节:
  // [0]=bus(0/1), [1]=is_extended(0/1), [2]=dlc,
  // [3..6]=can_id(大端), [7..14]=data[8]
  static constexpr size_t kPackedSize = 15;

  static std_msgs::msg::UInt8MultiArray Pack(const CanFrameMessage & frame)
  {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(kPackedSize, 0);
    msg.data[0] = static_cast<uint8_t>(frame.bus);
    msg.data[1] = frame.is_extended ? 1 : 0;
    msg.data[2] = frame.dlc;
    msg.data[3] = static_cast<uint8_t>((frame.can_id >> 24) & 0xFF);
    msg.data[4] = static_cast<uint8_t>((frame.can_id >> 16) & 0xFF);
    msg.data[5] = static_cast<uint8_t>((frame.can_id >> 8) & 0xFF);
    msg.data[6] = static_cast<uint8_t>(frame.can_id & 0xFF);
    for (size_t i = 0; i < frame.data.size(); ++i) {
      msg.data[7 + i] = frame.data[i];
    }
    return msg;
  }

  static std::optional<CanFrameMessage> Unpack(const std_msgs::msg::UInt8MultiArray & msg)
  {
    if (msg.data.size() != kPackedSize) {
      return std::nullopt;
    }
    CanFrameMessage frame;
    frame.bus = (msg.data[0] == 0) ? CanBus::CAN0 : CanBus::CAN1;
    frame.is_extended = msg.data[1] != 0;
    frame.dlc = static_cast<uint8_t>(msg.data[2] & 0x0F);
    frame.can_id = (static_cast<uint32_t>(msg.data[3]) << 24) |
      (static_cast<uint32_t>(msg.data[4]) << 16) |
      (static_cast<uint32_t>(msg.data[5]) << 8) |
      static_cast<uint32_t>(msg.data[6]);
    for (size_t i = 0; i < frame.data.size(); ++i) {
      frame.data[i] = msg.data[7 + i];
    }
    return frame;
  }
};

}  // namespace a3_can_bridge
