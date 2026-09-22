#pragma once

// Vendored from a3_can_bridge (real-hardware-verified) — namespace swap only.
// Keep in sync with src/a3_can_bridge/include/a3_can_bridge/motor_model.hpp.

#include <array>
#include <cstdint>
#include <string>

namespace a3_hardware_interface
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
  uint8_t fault_code{0};
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

/// 通信类型 0 应答：获取设备 ID（bit0-7=0xFE，bit8-15=motor_id，data 8 字节大端 MCU UID）
struct DeviceIdResponse
{
  uint8_t motor_id{0};
  uint64_t mcu_uid{0};
};

/// 软件版本应答（类型 2/24，data 00 C4 56 + 四段版本号）
struct SoftwareVersion
{
  uint8_t motor_id{0};
  std::string version;
};

/// 读参数应答（类型 17，F47）：data[0..1]=参数索引小端，data[4]=u8 值 / data[4..7]=float32 小端
struct GetParamResponse
{
  uint8_t motor_id{0};
  uint16_t param_id{0};
  uint8_t value_u8{0};
  float value_f32{0.0f};
};

}  // namespace a3_hardware_interface
