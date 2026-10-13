#!/usr/bin/env python3
"""5 模式 TCP 轨迹对比——离线报告（采集 JSON → FK → 3 张 SVG + MD）。

读 traj_modes_tcp_capture.py 的 JSON（各模式实测 /joint_states 关节轨迹），用 Pinocchio
FK 到末端 end_effector，得 TCP 位置/速度/加速度（中心差分），出：
  * 位置图：TCP 在 y-z 平面（x≈const）的轨迹叠加，标 5 个目标点；
  * 速度图：TCP 速率 |v| vs 时间；
  * 加速度图：TCP 加速度 |a| vs 时间。
手写 SVG（LL-152：不依赖 matplotlib）。

用法（先 source a3_shell_env.sh）：
  python3 scripts/a3_test/traj_modes_tcp_report.py \
      --run /tmp/traj_modes_tcp.json --out-dir docs/dev
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts"))
JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint"]
from gen_stream_modes import MODES  # noqa: E402

COLORS = {
    "ptp": "#7f7f7f", "ptp_blend": "#8c564b",
    "lin_exact": "#1f77b4", "lin_blend": "#ff7f0e", "circ": "#17becf",
    "jnt_g64_quintic": "#2ca02c", "jnt_g64_trapezoid": "#d62728",
    "jnt_spline_bell": "#9467bd", "jnt_spline_approx": "#e377c2",
}
LABELS = {
    "ptp": "ptp 关节逐腿", "ptp_blend": "ptp_blend 关节Seq+blend",
    "lin_exact": "lin_exact 笛卡尔LIN停", "lin_blend": "lin_blend 笛卡尔LIN圆角",
    "circ": "circ 笛卡尔圆弧",
    "jnt_g64_quintic": "jnt_g64_quintic G64+quintic",
    "jnt_g64_trapezoid": "jnt_g64_trapezoid G64+梯形",
    "jnt_spline_bell": "jnt_spline_bell 插值样条+钟形",
    "jnt_spline_approx": "jnt_spline_approx 逼近样条",
}
SHORT = {
    "ptp": "ptp", "ptp_blend": "ptpbl", "lin_exact": "linE", "lin_blend": "linB",
    "circ": "circ", "jnt_g64_quintic": "g64q", "jnt_g64_trapezoid": "g64t",
    "jnt_spline_bell": "bell", "jnt_spline_approx": "fit",
}
# F167 报告补充列：过首末点 / 过中间点 / 应用场合（语义依据 arm_controller.TRAJ_MODES）
MODE_META = {
    "ptp": ("✅ 段末精确停", "✅ 逐腿逐点停", "点对点定位 / 示教逐段回放；需停稳、精确到点"),
    "ptp_blend": ("✅ 段末精确停", "❌ 切角（可逐点指定精确）", "关节快速过渡、多路点连续运动（不停车）"),
    "lin_exact": ("✅ 直线到点+停", "✅ 逐点精确", "需笛卡尔直线且逐点停：精密接触 / 涂胶 / 焊接示教"),
    "lin_blend": ("✅ 段末精确停", "❌ 切角（可逐点指定精确）", "需笛卡尔直线的连续轨迹：喷涂 / 巡边，拐角圆滑"),
    "jnt_g64_quintic": ("✅ 端点精确", "❌ 切角（可逐点指定精确）", "关节平滑连续路径（G64）+ 加-巡-减；产线节拍"),
    "jnt_g64_trapezoid": ("✅ 端点精确", "❌ 切角（可逐点指定精确）", "同 G64 但梯形速度，最省时；节拍优先"),
    "jnt_spline_bell": ("✅ 端点精确", "✅ 全部精确过点", "要求精确过全部路点且 C2 平滑（抖小）"),
    "circ": ("✅ 弧到点+停", "✅ 过辅助点（弧过 3 点）", "圆弧路径：绕行 / 倒角 / 球面轨迹"),
    "jnt_spline_approx": ("✅ 端点精确", "❌ 仅端点（中间逼近不过点）", "给一批点求最平滑逼近：点不必严格过、最抗过冲"),
}


def _legend(parts, modes, H, L, per_row=5, dx=104, y0=None):
    """多行图例（9 模式）。"""
    y = H - 34 if y0 is None else y0
    for i, m in enumerate(modes):
        row, col = divmod(i, per_row)
        lx = L + col * dx
        ly = y + row * 20
        parts.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+16}" y2="{ly}" '
                     f'stroke="{COLORS[m]}" stroke-width="3"/>')
        parts.append(f'<text x="{lx+20}" y="{ly+4}" font-size="10" fill="#333">{SHORT[m]}</text>')


def load_pin():
    import pinocchio as pin
    from ament_index_python.packages import get_package_share_directory
    urdf = os.path.join(get_package_share_directory("a3_description"), "urdf", "el_a3.urdf")
    model = pin.buildModelFromUrdf(urdf)
    q_idx = [int(model.idx_qs[model.getJointId(jn)]) for jn in JOINTS]
    fid = model.getFrameId("end_effector")
    return pin, model, q_idx, fid


def fk_tcp(pin, model, q_idx, fid, q6) -> np.ndarray:
    q = np.zeros(model.nq)
    for v, qi in zip(q6, q_idx):
        q[qi] = float(v)
    d = model.createData()
    pin.forwardKinematics(model, d, q)
    pin.updateFramePlacement(model, d, fid)
    return np.array(d.oMf[fid].translation, dtype=float)


def process_mode(pin, model, q_idx, fid, samples: List[List[float]], n_total: int = 2):
    """samples: [[t,q0..q6,seg],...] → (t, xyz, speed, accel)。
    F166：模式段 = 除末尾 ptp 回起点段外的所有段（seg < n_total）。"""
    rows = [s for s in samples if int(s[8]) < n_total]
    if len(rows) < 5:
        rows = samples  # 段号异常则退回全部
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows], dtype=float)
    t = t - t[0]
    # 去重/单调（同 stamp 采样）
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    t = t[keep]
    q = np.array([[r[1 + i] for i in range(6)] for r, k in zip(rows, keep) if k], dtype=float)
    # 去掉尾部静止样本（回放返回后的收尾采样，避免虚增时长）
    if len(q) > 3:
        dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
        last_move = int(np.max(np.where(dq > 1e-5)[0])) + 1 if np.any(dq > 1e-5) else len(q)
        t, q = t[:last_move + 1], q[:last_move + 1]
    xyz = np.array([fk_tcp(pin, model, q_idx, fid, row) for row in q])
    if len(t) < 5:
        return t, xyz, np.zeros(len(t)), np.zeros(len(t))
    # 0.05s 均匀重采样，中心差分求速度/加速度
    tu = np.arange(t[0], t[-1], 0.02)
    xu = np.column_stack([np.interp(tu, t, xyz[:, k]) for k in range(3)])
    v = np.gradient(xu, tu, axis=0)
    a = np.gradient(v, tu, axis=0)
    speed = np.linalg.norm(v, axis=1)
    accel = np.linalg.norm(a, axis=1)
    return tu, xu, speed, accel


# ------------------------------------------------------------------ SVG

def _axes(parts, left, top, right, bottom, xlo, xhi, ylo, yhi, xlab, ylab, title):
    parts.append(f'<rect x="{left}" y="{top}" width="{right-left}" height="{bottom-top}" '
                 f'fill="none" stroke="#bbb"/>')
    parts.append(f'<text x="{(left+right)/2:.0f}" y="{top-8}" font-size="13" '
                 f'text-anchor="middle" fill="#222">{title}</text>')
    parts.append(f'<text x="{(left+right)/2:.0f}" y="{bottom+30}" font-size="11" '
                 f'text-anchor="middle" fill="#555">{xlab}</text>')
    parts.append(f'<text x="{left-8}" y="{(top+bottom)/2:.0f}" font-size="11" '
                 f'fill="#555" transform="rotate(-90 {left-8} {(top+bottom)/2:.0f})" '
                 f'text-anchor="middle">{ylab}</text>')
    for f in (0.0, 0.25, 0.5, 0.75, 1.0):
        gx = left + f * (right - left)
        gy = bottom - f * (bottom - top)
        parts.append(f'<line x1="{gx:.1f}" y1="{top}" x2="{gx:.1f}" y2="{bottom}" stroke="#eee"/>')
        parts.append(f'<line x1="{left}" y1="{gy:.1f}" x2="{right}" y2="{gy:.1f}" stroke="#eee"/>')
        parts.append(f'<text x="{gx:.1f}" y="{bottom+14}" font-size="9" fill="#888" '
                     f'text-anchor="middle">{xlo+f*(xhi-xlo):.2f}</text>')
        parts.append(f'<text x="{left-4}" y="{gy+3:.1f}" font-size="9" fill="#888" '
                     f'text-anchor="end">{ylo+f*(yhi-ylo):.3f}</text>')


def svg_time_series(series: Dict[str, Tuple[np.ndarray, np.ndarray]], xlab, ylab, title,
                    out_svg: str, height: float = 520) -> None:
    W, H = 560, int(height)
    L, T, R, B = 70, 30, W - 20, H - 72
    allt = np.concatenate([t for t, _ in series.values()]) if series else np.array([0, 1])
    ally = np.concatenate([y for _, y in series.values()]) if series else np.array([0, 1])
    xlo, xhi = 0.0, float(allt.max())
    ylo, yhi = 0.0, float(ally.max() * 1.1) if ally.max() > 0 else 1.0

    def sx(x): return L + (x - xlo) / (xhi - xlo or 1) * (R - L)
    def sy(y): return B - (y - ylo) / (yhi - ylo or 1) * (B - T)

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace"><rect width="{W}" height="{H}" '
             f'fill="white"/>']
    _axes(parts, L, T, R, B, xlo, xhi, ylo, yhi, xlab, ylab, title)
    for mode, (t, y) in series.items():
        if len(t) < 2:
            continue
        poly = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(t, y))
        parts.append(f'<polyline points="{poly}" fill="none" stroke="{COLORS[mode]}" '
                     f'stroke-width="2"/>')
    _legend(parts, list(series), H, L)
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def svg_planar_path(paths: Dict[str, np.ndarray], targets: List[Tuple[float, float]],
                    out_svg: str, height: float = 520) -> None:
    """位置图：y-z 平面轨迹（x≈const 平面，等比例）。"""
    W, H = 560, int(height)
    L, T, R, B = 70, 30, W - 20, H - 72
    ally = np.concatenate([p[:, 1] for p in paths.values()])
    allz = np.concatenate([p[:, 2] for p in paths.values()])
    pad = 0.005
    y0, y1 = float(ally.min() - pad), float(ally.max() + pad)
    z0, z1 = float(allz.min() - pad), float(allz.max() + pad)
    # F166 等比例（isotropic）：y/z 同 scale，避免 z 量程小被拉满放大 mm 级偏差
    uy = (R - L) / max(y1 - y0, 1e-9)
    uz = (B - T) / max(z1 - z0, 1e-9)
    u = min(uy, uz)
    yc, zc = (y0 + y1) / 2, (z0 + z1) / 2
    ylo, yhi = yc - (R - L) / (2 * u), yc + (R - L) / (2 * u)
    zlo, zhi = zc - (B - T) / (2 * u), zc + (B - T) / (2 * u)

    def sx(y): return L + (y - ylo) / (yhi - ylo or 1) * (R - L)
    def sz(z): return B - (z - zlo) / (zhi - zlo or 1) * (B - T)

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace"><rect width="{W}" height="{H}" '
             f'fill="white"/>']
    _axes(parts, L, T, R, B, ylo, yhi, zlo, zhi, "TCP y (m)", "TCP z (m)",
          "TCP 轨迹（y-z 平面，x≈-0.20）")
    for mode, xyz in paths.items():
        poly = " ".join(f"{sx(p[1]):.1f},{sz(p[2]):.1f}" for p in xyz)
        parts.append(f'<polyline points="{poly}" fill="none" stroke="{COLORS[mode]}" '
                     f'stroke-width="2" opacity="0.9"/>')
    for i, (y, z) in enumerate(targets):
        parts.append(f'<circle cx="{sx(y):.1f}" cy="{sz(z):.1f}" r="4" fill="#222"/>')
        parts.append(f'<text x="{sx(y)+6:.1f}" y="{sz(z)-6:.1f}" font-size="10" fill="#222">'
                     f'P{i}</text>')
    _legend(parts, list(paths), H, L)
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


def main() -> None:
    ap = argparse.ArgumentParser(description="5 模式 TCP 轨迹对比报告")
    ap.add_argument("--run", default="/tmp/traj_modes_tcp.json")
    ap.add_argument("--waypoints", default=os.path.join(WS, "trajectories", "waypoints", "latest.yaml"))
    ap.add_argument("--out-dir", default=os.path.join(WS, "docs", "dev"))
    args = ap.parse_args()

    with open(args.run, "r", encoding="utf-8") as f:
        data = json.load(f)
    with open(args.waypoints, "r", encoding="utf-8") as f:
        wp = yaml.safe_load(f)
    targets = [(w["pose"]["position"]["y"], w["pose"]["position"]["z"]) for w in wp["waypoints"]]

    pin, model, q_idx, fid = load_pin()
    P = np.array(targets, dtype=float)  # 折线 P0..P4（y,z）

    def dev_from_polyline(yz: np.ndarray) -> float:
        def seg_dist(pt, a, b):
            ab = b - a
            t = np.clip(np.dot(pt - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
            return np.linalg.norm(pt - (a + t * ab))
        mx = 0.0
        for p in yz[:, [1, 2]]:
            mx = max(mx, min(seg_dist(p, P[i], P[i + 1]) for i in range(len(P) - 1)))
        return mx

    paths: Dict[str, np.ndarray] = {}
    speed_series: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    accel_series: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    rows = []
    for mode in MODES:
        rec = data["modes"].get(mode)
        if not rec or not rec.get("samples"):
            print(f"[warn] {mode} 无数据")
            continue
        t, xyz, speed, accel = process_mode(
            pin, model, q_idx, fid, rec["samples"], int(rec.get("n_total", 2)))
        paths[mode] = xyz
        speed_series[mode] = (t, speed)
        accel_series[mode] = (t, accel)
        seglen = float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))
        rows.append({"mode": mode, "dur": float(t[-1]) if len(t) else 0.0,
                     "vmax": float(speed.max()) if len(speed) else 0.0,
                     "amax": float(accel.max()) if len(accel) else 0.0,
                     "path_len": seglen, "dev_mm": dev_from_polyline(xyz) * 1000.0})

    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    pos_svg = os.path.join(out_dir, "traj_modes_tcp_position.svg")
    spd_svg = os.path.join(out_dir, "traj_modes_tcp_speed.svg")
    acc_svg = os.path.join(out_dir, "traj_modes_tcp_accel.svg")
    svg_planar_path(paths, targets, pos_svg)
    svg_time_series(speed_series, "时间 t (s)", "TCP 速率 |v| (m/s)", "TCP 速度对比", spd_svg)
    svg_time_series(accel_series, "时间 t (s)", "TCP 加速度 |a| (m/s²)", "TCP 加速度对比", acc_svg)

    lines = ["# 9 模式 TCP 轨迹对比（口风琴 5 点，实采）", "",
             "- 采集：仿真栈逐模式回放，录制 /joint_states，Pinocchio FK 到 `end_effector`。",
             "- 位置图：TCP 在 y-z 平面（x≈-0.20）轨迹，**等比例**；速度/加速度：TCP |v|/|a| vs 时间。",
             "- 「距直线折线」= 各 TCP 点到 P0..P4 折线的最短垂距最大值（量化是否走直线）。",
             "", "## 位置对比", "", f"![位置]({os.path.basename(pos_svg)})", "",
             "## 速度对比", "", f"![速度]({os.path.basename(spd_svg)})", "",
             "## 加速度对比", "", f"![加速度]({os.path.basename(acc_svg)})", "",
             "| 模式 | 时长(s) | TCP峰值速率(m/s) | TCP峰值加速度(m/s²) | TCP路径长(m) | 距直线折线(mm) | 过首末点 | 过中间点 | 应用场合 |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        ep, vp, sc = MODE_META.get(r["mode"], ("?", "?", "?"))
        lines.append(f"| {r['mode']} | {r['dur']:.2f} | {r['vmax']:.3f} | "
                     f"{r['amax']:.2f} | {r['path_len']:.4f} | {r['dev_mm']:.1f} | "
                     f"{ep} | {vp} | {sc} |")
    lines.append("")
    md = os.path.join(out_dir, "traj_modes_tcp_report.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"报告 -> {md}")
    print(f"图 -> {pos_svg}\n     {spd_svg}\n     {acc_svg}")
    for r in rows:
        print(f"  {r['mode']:20s} dur={r['dur']:6.2f}s vmax={r['vmax']:.3f} "
              f"amax={r['amax']:5.2f} len={r['path_len']:.4f} dev={r['dev_mm']:.1f}mm")


if __name__ == "__main__":
    main()
