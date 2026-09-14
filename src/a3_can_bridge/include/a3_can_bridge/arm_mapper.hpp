#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "a3_can_bridge/motor_model.hpp"

namespace a3_can_bridge
{

struct MotorRoute
{
  size_t trajectory_index{0};
  uint8_t motor_id{0};
  std::string joint_hint;
};

/// EL-A3: L1..L6 arm + L7 gripper, single SocketCAN, motor IDs 1..7.
/// The bus is a runtime-wide single value (ArmBus), not per-route.
///
/// F52（缺电机降级档）：关节/电机清单**运行期可配**——默认是下面编译期写死的 7J 表
/// （L1..L7 → 电机 1..7），启动时可由 `motor_map.yaml` 的 `joint_names` +
/// `motor_ids_by_index` 覆盖成任意档位（例如事故后只剩 L1–L5 的 5J 档）。
/// 全栈的 `/joint_states`/`MotorStates`/tx_stats 名字表、广播 id 展开、
/// F51 使能新鲜度门禁范围、逐关节参数长度校验都以本清单为准。
class ArmMapper
{
public:
  /// 编译期上限（= 满配 7 关节）；运行期档位不得超出。
  static constexpr size_t kMaxArmJoints = 7;

  /// 编译期默认档位：7J（L1..L7）。也是 Configure() 缺省与 ResetToDefault() 的目标。
  static inline const std::array<const char *, kMaxArmJoints> kChampJointNames{{
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
  }};

  static inline const std::array<MotorRoute, kMaxArmJoints> kTemporaryIndexMap{{
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

  // ---------------------------------------------------------------- F52 运行期档位

  /// 配置当前档位：joint_names[i] ↔ motor_ids[i]（下标即轨迹关节位）。
  /// 长度不一致 / 为空 / 超出 kMaxArmJoints 时**拒绝并保持原档位**，返回 false。
  static bool Configure(
    const std::vector<std::string> & joint_names,
    const std::vector<uint8_t> & motor_ids)
  {
    if (joint_names.empty() || joint_names.size() != motor_ids.size() ||
      joint_names.size() > kMaxArmJoints)
    {
      return false;
    }
    std::vector<MotorRoute> routes;
    routes.reserve(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      std::string hint = joint_names[i];
      const std::string suffix = "_joint";
      if (hint.size() > suffix.size() &&
        hint.compare(hint.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        hint = hint.substr(0, hint.size() - suffix.size());
      }
      routes.push_back(MotorRoute{i, motor_ids[i], hint});
    }
    kJointNames = joint_names;
    kRoutes = std::move(routes);
    return true;
  }

  /// 回到编译期默认档位（7J）。
  static void ResetToDefault()
  {
    std::vector<std::string> names(
      kChampJointNames.begin(), kChampJointNames.end());
    std::vector<uint8_t> ids;
    ids.reserve(kTemporaryIndexMap.size());
    for (const auto & r : kTemporaryIndexMap) {
      ids.push_back(r.motor_id);
    }
    Configure(names, ids);
  }

  /// 当前档位关节数（默认 7；5J 档为 5）。
  static size_t NumJoints()
  {
    EnsureInitialized();
    return kJointNames.size();
  }

  /// 当前档位关节名（顺序 = 轨迹下标顺序）。
  static const std::vector<std::string> & JointNames()
  {
    EnsureInitialized();
    return kJointNames;
  }

  /// 当前档位路由表（trajectory_index / motor_id / joint_hint）。
  static const std::vector<MotorRoute> & Routes()
  {
    EnsureInitialized();
    return kRoutes;
  }

  static std::optional<MotorRoute> GetRouteByTrajectoryIndex(size_t idx)
  {
    EnsureInitialized();
    if (idx >= kRoutes.size()) {
      return std::nullopt;
    }
    return kRoutes[idx];
  }

  static std::optional<MotorRoute> GetRouteByJointName(const std::string & joint_name)
  {
    EnsureInitialized();
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      if (joint_name == kJointNames[i]) {
        return GetRouteByTrajectoryIndex(i);
      }
    }
    return std::nullopt;
  }

  static std::optional<MotorRoute> GetRouteByMotorId(uint8_t motor_id)
  {
    EnsureInitialized();
    for (const auto & route : kRoutes) {
      if (route.motor_id == motor_id) {
        return route;
      }
    }
    return std::nullopt;
  }

  /// 该电机 id 是否属于当前档位（用于「缺电机」判定与广播展开）。
  static bool HasMotor(uint8_t motor_id)
  {
    return GetRouteByMotorId(motor_id).has_value();
  }

private:
  static void EnsureInitialized()
  {
    if (kJointNames.empty()) {
      ResetToDefault();
    }
  }

  static inline CanBus kArmBus{CanBus::CAN1};
  static inline std::vector<std::string> kJointNames{};
  static inline std::vector<MotorRoute> kRoutes{};
};

using DogMapper = ArmMapper;

}  // namespace a3_can_bridge
