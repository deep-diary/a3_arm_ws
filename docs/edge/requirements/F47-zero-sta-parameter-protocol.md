# F47 — 0x7029 zero_sta 参数读写协议（断电多圈窗口选择）


- **说明：** 真机 7 关节臂实测：断电后手动转动关节再上电，多圈计数在默认 zero_sta=0（0~2π 重建）下会环绕——负向转 ~22° 的 L6 上电读 +5.8994 rad（=+338°，+2π 环绕），用户担心的「-10° 变 350°」真实发生；环绕读数超关节限位时严禁使能（kp×误差会瞬间猛拉）。固件参数 0x7029 `zero_sta`（uint8，1 字节，默认 0）选择上电位置重建窗口：0=0~2π，1=-π~π。置 1 后 ±180° 内的断电转动可被正确记录（L2/L3 行程超 π，超 ±180° 的大转动仍会环绕）。本需求为 a3_can_bridge 新增三个协议能力（均为 EL05 通信类型，单电机或广播）：
  1. **`/a3/motor/get_param`**（新 srv `GetMotorParam`）：通信类型 17（0x11）读参数——发请求后收集应答直至超时（默认 0.4 s），`motor_id=0` 时广播到全臂映射电机并聚合；应答 data[4] 为 u8 值、data[4..7] 为 float32 小端，两数组同时返回（uint8 参数看 `values_u8`，float 参数看 `values_f32`）。
  2. **`/a3/motor/set_param_u8`**（新 srv `SetMotorParamU8`）：通信类型 18（0x12）的 uint8 形式（值写 data[4]），`motor_id=0` 广播；float 参数继续用既有 `/a3/motor/set_param`。
  3. **`/a3/motor/save_param`**（MotorCommand `command=5`）：通信类型 22（0x16）保存参数到 flash，0=广播。
  读操作不受 power-sequence gate 互锁（与 get_device_id/request_version 同语义）；set/save 为写操作，gate 打开时拒绝。
- **验收标准：**
  1. 上电后 `get_param`（motor_id=0, param_id=0x7029）读到 7 关节 `values_u8` 全为 0（出厂默认）
  2. `set_param_u8` 广播置 1 → `save_param` 广播 → 断电重启后 `get_param` 读回全为 1（flash 持久）
  3. zero_sta=1 后断电手动转动各关节 ±（<180°）再上电：probe 读数与转动方向/幅度一致，负向转动不再 +2π 环绕（L6 类场景回归）
  4. 读参数无应答时返回 `success=false` + `"no response (timeout)"`；忙时拒绝（"get_param busy"）
- **关联：** F32（扫描收集 DeferResponse 模式，复用同一实现路径）；[QUICKSTART.md](QUICKSTART.md)（真机断电记忆验证流程）；[lessons_learned/](../lessons_learned/)（断电多圈环绕 LL 条目）
- **状态：** `implemented`（2026-09-13 真机 7 关节臂验证通过：2/4/5/6/7 读回=1 且跨断电保留；断电负转后 L5/L6/L7/L1/L3 读数全部连续——L5=-0.8361、L6=-1.6068、L1=-0.6055、L3=-0.5200 不再环绕（L3 旧固件行为验证通过，无需升级）；L1/L3 旧固件 0.0.3.4 读回恒 0 但写+保存生效。save 帧数据域须 `01 02 03 04 05 06 07 08`，全零不触发保存，见 LL-019）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
