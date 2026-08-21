#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace a3_can_bridge
{

enum class CanBus : uint8_t
{
  CAN0 = 0,
  CAN1 = 1
};

struct CanFrameMessage
{
  CanBus bus{CanBus::CAN0};
  bool is_extended{true};
  uint32_t can_id{0};
  uint8_t dlc{8};
  std::array<uint8_t, 8> data{};
};

struct MotorFeedback
{
  uint8_t master_id{0};
  uint8_t motor_id{0};
  uint8_t cmd_type{0};
  uint8_t mode_status{0};
  bool error_status{false};
  bool hall_error{false};
  bool magnet_error{false};
  bool temp_error{false};
  bool current_error{false};
  bool voltage_error{false};
  float current_angle{0.0f};
  float current_speed{0.0f};
  float current_torque{0.0f};
  float current_temp{0.0f};
  std::string source_bus{"can0"};
};

}  // namespace a3_can_bridge
