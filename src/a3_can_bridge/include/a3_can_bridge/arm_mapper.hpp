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
  CanBus bus{CanBus::CAN0};
  const char * joint_hint{""};
};

/// EL-A3: L1..L6 arm + L7 gripper, single SocketCAN (can0), motor IDs 1..7.
class ArmMapper
{
public:
  static constexpr std::array<const char *, 7> kChampJointNames{{
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
  }};

  static constexpr std::array<MotorRoute, 7> kTemporaryIndexMap{{
    {0, 1, CanBus::CAN0, "L1"},
    {1, 2, CanBus::CAN0, "L2"},
    {2, 3, CanBus::CAN0, "L3"},
    {3, 4, CanBus::CAN0, "L4"},
    {4, 5, CanBus::CAN0, "L5"},
    {5, 6, CanBus::CAN0, "L6"},
    {6, 7, CanBus::CAN0, "L7_gripper"},
  }};

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
};

using DogMapper = ArmMapper;

}  // namespace a3_can_bridge
