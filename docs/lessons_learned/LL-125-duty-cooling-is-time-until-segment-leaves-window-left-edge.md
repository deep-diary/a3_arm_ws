# LL-125 — 滚动窗口占空比门的冷却时间：等最旧段末端越过「窗口左沿」，不是越过 now

> **日期：** 2026-09-24  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) Ubuntu 22.04 + ROS 2 Humble

## 现象

F107 占空比门在预算耗尽时返回冷却秒数，初版对完全落在窗口内的旧运动段算出 `wait 0s`——预算明明是满的，却立刻拒绝新运动且提示不用等，用户无法操作。

## 根因

冷却公式写成了 `(start + duration) - now`（等段结束越过当前时刻）。对于已经开始、甚至已经结束但仍在窗口内的段，该值 ≤ 0。正确语义：该段占用的预算要等到**段的末端越过滚动窗口的左沿 `now - window`** 才会释放。

## 正确做法 / 规避

```python
# 拒绝时遍历窗口内的段，累计需要腾出的时长
cooling = (start + dur) - (now - window)   # 段末端越过窗口左沿所需等待
```

配套的窗口求和也必须是「重叠感知」的：段只有与 `[now-window, now]` 重叠的部分计入，

```python
used += min(dur, start + dur - (now - window))
```

验收口径：窗口 20 s / 比例 0.2（预算 4 s），每段 0.3 s 的 jog 第 14 次被拒，消息 `4s motion in window, wait 9s`；把窗口缩小到 1 s 后旧记录全部出窗，立即恢复可运动。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_duty_check` / `_duty_used_s`）
- `scripts/a3_test/f107_payload_duty_acceptance.py`（P4）
