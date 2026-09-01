# LL-003 — trotbot→arm 移植遗留：限位表越界把目标 clamp 成 0、脚本 CRLF 报 pipefail

> **日期：** 2026-09-01
> **产品线：** Edge
> **环境：** RK3588（LubanCat-4-V1）+ Ubuntu 22.04 + ROS 2 Humble，SocketCAN 1 Mbps

## 现象

RK3588 真机 CAN 联调时，硬件链路本身正常（`can0↔can1` 双向回环通、电机 ID=1 手动 `cansend` 能正常运动），但一旦走 `motor_protocol_node` 发轨迹，电机就不动：

- 总线抓到的 MIT 帧恒为 `017FFF01  [8]  7F FF 7F FF 28 F5 66 66`，即位置 `p` 恒为 `0x7FFF = 0.0 rad`（`kp=80`、`kd=2.0` 是对的）。
- `motor_protocol_node` 日志 / `/motor_feedback` 里出现 `cmd_angle=1.63042e-322`（denormal 垃圾值，不是正常的 `0.5`）。
- 电机反馈 `angle` 被拉回 0，而不是跟随目标。

## 根因

`a3_can_bridge` 从 trotbot（12 电机四足）继承时，`motor_protocol_node.cpp` 的 `RebuildMotorLimitTable()` 循环上限仍硬编码为 12：

```cpp
for (size_t i = 0; i < 12; ++i) {   // A3 arm 只有 7 关节
  const double a = joint_signs_[i] * joint_cmd_min_rad_[i] + joint_offsets_rad_[i];
  ...
  runtime_joint_mit_min_rad_[i] = std::min(a, b);   // size=7，i=7..11 越界写
  runtime_joint_mit_max_rad_[i] = std::max(a, b);
}
```

而 `joint_signs_`、`joint_cmd_min_rad_`、`joint_mit_*_rad_`、`runtime_joint_mit_*_rad_` 都是 7 元素（`kNumArmJoints = DogMapper::kTemporaryIndexMap.size() = 7`）。`i=7..11` 越界读写污染了限位表，随后 `ClampMotorCommand` 把合法目标（0.3/0.5）错误 clamp 成 0。手动 `cansend` 不经过 `motor_protocol_node`，所以不受影响。

## 正确做法 / 规避

1. 循环上限改为关节数，不要用魔数：

```cpp
const size_t n = std::min<size_t>(kNumArmJoints, runtime_joint_mit_min_rad_.size());
for (size_t i = 0; i < n; ++i) { ... }
```

2. 排查同类问题：`grep -nE 'i < 12|< 12;' src/a3_can_bridge/src/*.cpp`，arm 场景一律以 `kNumArmJoints` / `kTemporaryIndexMap.size()` 为准。
3. 真机联调若发现「MIT 帧 p 恒为 0 / `cmd_angle` 是 denormal 垃圾值」，优先怀疑限位表越界，而不是总线或电机。

## 相关坑（同一移植批次）

`src/a3_can_bridge/scripts/el05_motor_cansend.sh`、`a3_motor_cansend.sh` 是 **CRLF 换行**，直接执行报：

```
set: pipefail: 无效的选项名
```

规避：临时 `sed -i 's/\r$//'` 转 LF 再跑，或干脆手动 `cansend`。新写 `*.sh` 后务必 `bash -n` 抽查换行。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`RebuildMotorLimitTable`）
- `src/a3_can_bridge/scripts/el05_motor_cansend.sh`
- `src/a3_can_bridge/scripts/a3_motor_cansend.sh`
