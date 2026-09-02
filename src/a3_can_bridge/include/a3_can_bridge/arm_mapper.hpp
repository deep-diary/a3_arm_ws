#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "a3_can_bridge/motor_model.hpp"

namespace a3_can_bridge
{

struct MotorRoute
{
  size_t trajectory_index{0};
  uint8_t motor_id{0};
  const char * joint_hint{""};
};

/// EL-A3: L1..L6 arm + L7 gripper, single SocketCAN, motor IDs 1..7.
/// The bus is a runtime-wide single value (ArmBus), not per-route.
class ArmMapper
{
public:
  static constexpr std::array<const char *, 7> kChampJointNames{{
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
  }};

  static constexpr std::array<MotorRoute, 7> kTemporaryIndexMap{{
    {0, 1, "L1"},
    {1, 2, "L2"},
    {2, 3, "L3"},
    {3, 4, "L4"},
    {4, 5, "L5"},
    {5, 6, "L6"},
    {6, 7, "L7_gripper"},
  }};

  static void SetArmBus(CanBus bus) { kArmBus = bus; }

  static CanBus ArmBus() { return kArmBus; }

  /// "can0"/"0"/"CAN0" -> CAN0；"can1"/"1"/"CAN1" -> CAN1；其它返回 false。
  static bool ParseArmBus(const std::string & s, CanBus * out)
  {
    if (out == nullptr) {
      return false;
    }
    if (s == "can0" || s == "0" || s == "CAN0" || s == "can0 ") {
      *out = CanBus::CAN0;
      return true;
    }
    if (s == "can1" || s == "1" || s == "CAN1" || s == "can1 ") {
      *out = CanBus::CAN1;
      return true;
    }
    return false;
  }

  static std::optional<MotorRoute> GetRouteByTrajectoryIndex(size_t idx)
  {
    if (idx >= kTemporaryIndexMap.size()) {
      return std::nullopt;
    }
    return kTemporaryIndexMap[idx];
  }

  static std::optional<MotorRoute> GetRouteByJointName(const std::string & joint_name)
  {
    for (size_t i = 0; i < kChampJointNames.size(); ++i) {
      if (joint_name == kChampJointNames[i]) {
        return GetRouteByTrajectoryIndex(i);
      }
    }
    return std::nullopt;
  }

private:
  static inline CanBus kArmBus{CanBus::CAN1};
};

using DogMapper = ArmMapper;

}  // namespace a3_can_bridge
