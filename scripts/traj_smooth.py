#!/usr/bin/env python3
"""F137 录制轨迹几何去噪 + 平滑重规划（先离线仿真验证）。

方法（工业对标：B 样条最小二乘光顺 + 重定时）：
  1) 加载录制轨迹 YAML（arm_controller `_dump_recording` 格式）
  2) 逐关节五次 B 样条平滑（scipy.interpolate.UnivariateSpline，k=5），
     时间参数化 u=(t-t0)/T；平滑因子 s=N·ε²（ε ≈ RMS 残差目标）
  3) quintic 时间剖线（起止零速/零加速度）重采样
  4) 复用 traj_smoothness_calibration.compute_metrics 算三列对比：
       原始 / F137 平滑 / 标准 quintic（两点直线 + 同 quintic 时间，仅作诊断下限）

输出：stdout 摘要 + `docs/dev/F137_TRAJ_SMOOTHING_REPORT.md` + 每轨迹 CSV。

示例：
  python3 scripts/traj_smooth.py --dir ~/.a3/trajectories --latest 3 --eps 0.01
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy.interpolate import UnivariateSpline

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_smoothness_calibration import (  # noqa: E402
    build_standard_trajectory,
    compute_metrics,
    load_recorded_trajectory,
)

LABELS = ("raw", "smooth", "standard")
METRIC_KEYS = ("v_rms", "a_rms", "j_rms", "a_peak", "j_peak")


def _quintic_s(tau: float) -> float:
    """0→0、1→1，起止一/二阶导数均为 0 的五次时间剖线。"""
    return 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5


def fit_smoothing_splines(
    raw: np.ndarray, times: Sequence[float], eps: float
) -> Tuple[List[UnivariateSpline], np.ndarray]:
    """逐关节五次 B 样条平滑（时间参数化）。

    raw: (N, D)。返回 (splines, u_all)；splines[j](u) -> 关节 j 位置，u∈[0,1]。
    """
    t = np.asarray(times, dtype=float)
    u_all = (t - t[0]) / (t[-1] - t[0]) if t[-1] > t[0] else np.linspace(0.0, 1.0, len(t))
    n = raw.shape[0]
    s = n * eps * eps  # 平滑因子：RMS 残差目标 ≈ ε
    splines = [
        UnivariateSpline(u_all, raw[:, j], k=5, s=s)
        for j in range(raw.shape[1])
    ]
    return splines, u_all


def smooth_path(
    raw: np.ndarray,
    times: Sequence[float],
    eps: float,
    resample_n: int = 400,
) -> Tuple[List[float], List[List[float]]]:
    """B 样条平滑 + quintic 时间重定时。

    raw: (N, D)。返回 (times_smooth, positions_smooth)。
    """
    splines, _u = fit_smoothing_splines(raw, times, eps)
    duration = float(times[-1] - times[0]) if len(times) > 1 else 1.0
    tau = np.linspace(0.0, 1.0, resample_n)
    u_time = np.array([_quintic_s(x) for x in tau])
    q_smooth = np.column_stack([spl(u_time) for spl in splines])
    return (tau * duration).tolist(), q_smooth.tolist()


def time_matched_deviation(
    raw: np.ndarray, splines: List[UnivariateSpline], u_all: np.ndarray
) -> Tuple[float, float]:
    """平滑样条在原始时刻与原始点的偏离：返回 (RMS, max)。"""
    q_at = np.column_stack([spl(u_all) for spl in splines])
    err = q_at - raw
    rms = float(np.sqrt(np.mean(err * err)))
    mx = float(np.abs(err).max())
    return rms, mx


def write_smoothed_yaml(path: str, eps: float, out_yaml: str, resample_n: int = 400) -> int:
    """把单条录制轨迹去噪后另存为可回放的 YAML（不覆盖原文件）。

    输出为 arm_controller `_dump_recording` 同款格式：positions 为 B 样条去噪后
    值，时间轴用**均匀重采样**（uniform dt）——干净的时间戳让 F68 Ruckig 的差分
    播种稳定，避免原始录制抖动时间戳导致 Ruckig 失败回退 TOTG。返回点数。
    """
    joint_names, times, positions = load_recorded_trajectory(path)
    raw = np.asarray(positions, dtype=float)
    splines, _u = fit_smoothing_splines(raw, times, eps)
    duration = float(times[-1] - times[0]) if len(times) > 1 else 1.0
    u_uniform = np.linspace(0.0, 1.0, resample_n)
    q_s = np.column_stack([spl(u_uniform) for spl in splines])
    t_uniform = times[0] + u_uniform * duration
    data = {
        "joint_names": list(joint_names),
        "points": [
            {
                "positions": [float(v) for v in q_s[i]],
                "time_from_start_sec": float(t_uniform[i]),
            }
            for i in range(resample_n)
        ],
    }
    out_yaml = os.path.expanduser(out_yaml)
    with open(out_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f)
    return resample_n


def process_one(path: str, eps: float, resample_n: int = 400) -> Dict[str, object]:
    """对单条录制轨迹做三列对比，返回结果字典。"""
    joint_names, times, positions = load_recorded_trajectory(path)
    raw = np.asarray(positions, dtype=float)
    n_raw = raw.shape[0]
    duration = float(times[-1] - times[0]) if len(times) > 1 else 1.0

    # 1) 原始
    raw_metrics = compute_metrics(times, [list(p) for p in raw])

    # 2) F137 平滑
    splines, u_all = fit_smoothing_splines(raw, times, eps)
    tau = np.linspace(0.0, 1.0, resample_n)
    u_time = np.array([_quintic_s(x) for x in tau])
    q_smooth = np.column_stack([spl(u_time) for spl in splines])
    t_smooth = (tau * duration).tolist()
    smooth_metrics = compute_metrics(t_smooth, [list(p) for p in q_smooth])

    # 3) 标准 quintic（首末点直线 + 同 quintic 时间，仅诊断下限）
    q0 = raw[0].tolist()
    q1 = raw[-1].tolist()
    std_t, std_q = build_standard_trajectory(q0, q1, duration, resample_n)
    std_metrics = compute_metrics(std_t, std_q)

    dev_rms, dev_max = time_matched_deviation(raw, splines, u_all)

    return {
        "path": path,
        "basename": os.path.basename(path),
        "joint_names": joint_names,
        "n_raw": n_raw,
        "duration": duration,
        "eps": eps,
        "dev_rms": dev_rms,
        "dev_max": dev_max,
        "metrics": {
            "raw": raw_metrics,
            "smooth": smooth_metrics,
            "standard": std_metrics,
        },
    }


def eps_sweep(
    path: str, eps_list: Sequence[float], resample_n: int = 400
) -> List[Dict[str, float]]:
    """单条轨迹的 ε 扫描：平滑因子 vs j_rms vs 几何偏离（Pareto 曲线）。"""
    joint_names, times, positions = load_recorded_trajectory(path)
    raw = np.asarray(positions, dtype=float)
    duration = float(times[-1] - times[0]) if len(times) > 1 else 1.0
    out = []
    for eps in eps_list:
        splines, u_all = fit_smoothing_splines(raw, times, eps)
        tau = np.linspace(0.0, 1.0, resample_n)
        u_time = np.array([_quintic_s(x) for x in tau])
        q_smooth = np.column_stack([spl(u_time) for spl in splines])
        t_smooth = (tau * duration).tolist()
        m = compute_metrics(t_smooth, [list(p) for p in q_smooth])
        dev_rms, dev_max = time_matched_deviation(raw, splines, u_all)
        out.append({
            "eps": eps, "j_rms": m["j_rms"], "a_rms": m["a_rms"],
            "dev_rms": dev_rms, "dev_max": dev_max,
        })
    return out


def _fmt(v: float) -> str:
    if v >= 1e3:
        return f"{v:.3g}"
    if v >= 100:
        return f"{v:.1f}"
    return f"{v:.4f}"


def _downsample(x: np.ndarray, n: int) -> np.ndarray:
    if len(x) <= n:
        return x
    idx = np.linspace(0, len(x) - 1, n).astype(int)
    return x[idx]


def write_overlay_svg(result: Dict[str, object], out_svg: str) -> None:
    """画前两个大幅关节（L2/L7）原始 vs 平滑位置曲线 SVG，直观展示去噪。"""
    jn, times, positions = load_recorded_trajectory(result["path"])
    raw = np.asarray(positions, dtype=float)
    t = np.asarray(times, dtype=float)
    eps = result["eps"]
    splines, u_all = fit_smoothing_splines(raw, times, eps)
    tau = np.linspace(0.0, 1.0, 600)
    u_time = np.array([_quintic_s(x) for x in tau])
    t_s = tau * (t[-1] - t[0])

    # 选运动幅度最大的两个关节
    rng = raw.max(0) - raw.min(0)
    order = np.argsort(rng)[::-1]
    picks = [int(i) for i in order[:2]]

    W, H, ML, MR, MT, MB = 800, 420, 70, 20, 30, 40
    panel_h = (H - MT - MB) / 2
    colors = {"raw": "#e4572e", "smooth": "#1e6bb8"}

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" font-family="monospace">']
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')

    for pi, j in enumerate(picks):
        top = MT + pi * panel_h
        ymin = float(raw[:, j].min()); ymax = float(raw[:, j].max())
        span = (ymax - ymin) or 1e-6
        def sx(v):
            return ML + (v - t[0]) / (t[-1] - t[0]) * (W - ML - MR)
        def sy(v):
            return top + panel_h - (v - ymin) / span * (panel_h - 10)

        # 网格与轴标签
        parts.append(f'<text x="{ML}" y="{top+12}" font-size="12" fill="#333">'
                     f'{jn[j]} (range {span:.3f} rad)</text>')
        # 原始（降采样）
        ti = _downsample(t, 400); vi = _downsample(raw[:, j], 400)
        pts = " ".join(f"{sx(ti[k]):.1f},{sy(vi[k]):.1f}" for k in range(len(ti)))
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{colors["raw"]}" '
                     f'stroke-width="1" opacity="0.6"/>')
        # 平滑
        vs = splines[j](u_time)
        pts2 = " ".join(f"{sx(t_s[k]):.1f},{sy(vs[k]):.1f}" for k in range(len(t_s)))
        parts.append(f'<polyline points="{pts2}" fill="none" stroke="{colors["smooth"]}" '
                     f'stroke-width="2"/>')

    # 图例
    parts.append(f'<line x1="{ML}" y1="{H-18}" x2="{ML+24}" y2="{H-18}" '
                 f'stroke="{colors["raw"]}" stroke-width="2"/>')
    parts.append(f'<text x="{ML+30}" y="{H-13}" font-size="12">raw 原始录制</text>')
    parts.append(f'<line x1="{ML+140}" y1="{H-18}" x2="{ML+164}" y2="{H-18}" '
                 f'stroke="{colors["smooth"]}" stroke-width="2"/>')
    parts.append(f'<text x="{ML+170}" y="{H-13}" font-size="12">F137 平滑 (ε={eps:.3f})</text>')
    parts.append("</svg>")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))



def discover_latest(directory: str, latest: int) -> List[str]:
    """按 mtime 取最近 N 条 teach_*.yaml（排除 *.preretime.yaml）。"""
    files = []
    for name in os.listdir(directory):
        if not name.startswith("teach_") or not name.endswith(".yaml"):
            continue
        if ".preretime." in name:
            continue
        full = os.path.join(directory, name)
        files.append((os.path.getmtime(full), full))
    files.sort(reverse=True)
    return [f for _, f in files[:latest]]


def write_report(
    results: List[Dict[str, object]],
    sweep: Dict[str, List[Dict[str, float]]],
    out_path: str,
    overlay_svg: Optional[str] = None,
) -> None:
    """生成 markdown 对比报告。"""
    lines: List[str] = []
    lines.append("# F137 录制轨迹几何去噪 + 平滑重规划 —— 仿真对比报告")
    lines.append("")
    lines.append(f"- 生成时间：{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("- 方法：逐关节五次 B 样条平滑（`UnivariateSpline k=5`，时间参数化，平滑因子 `s=N·ε²`）→ quintic 时间重定时")
    lines.append("- 指标：`v_rms / a_rms / j_rms / a_peak / j_peak`（逐关节有限差分，跨关节聚合）")
    lines.append("- 三列：`raw`=原始录制（原始时间戳）、`smooth`=F137 平滑（quintic 时间）、`standard`=首末点直线 + 同 quintic 时间（仅诊断下限，非任务路径）")
    lines.append("")
    if overlay_svg:
        lines.append(f"![轨迹去噪示意]({overlay_svg})")
        lines.append("")
    lines.append("## 汇总（j_rms 为主指标，越小越平滑）")
    lines.append("")
    lines.append("| 轨迹 | n 原始 | ε (rad) | j_rms raw | j_rms smooth | j_rms standard | 降幅 | 偏离 RMS/max (rad) |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for r in results:
        m = r["metrics"]
        jr = m["raw"]["j_rms"]
        js = m["smooth"]["j_rms"]
        jst = m["standard"]["j_rms"]
        ratio = jr / js if js > 1e-12 else float("inf")
        lines.append(
            f"| {r['basename']} | {r['n_raw']} | {r['eps']:.4f} "
            f"| {_fmt(jr)} | {_fmt(js)} | {_fmt(jst)} | ×{ratio:.1f} "
            f"| {r['dev_rms']:.4f} / {r['dev_max']:.4f} |"
        )
    lines.append("")
    lines.append("## ε 扫描（平滑强度 vs 几何偏离的 Pareto 曲线）")
    lines.append("")
    for r in results:
        sw = sweep.get(r["basename"], [])
        if not sw:
            continue
        lines.append(f"### {r['basename']}")
        lines.append("")
        lines.append("| ε (rad) | j_rms smooth | a_rms smooth | 偏离 RMS (rad) | 偏离 max (rad) |")
        lines.append("|---|---|---|---|---|")
        for s in sw:
            lines.append(
                f"| {s['eps']:.4f} | {_fmt(s['j_rms'])} | {_fmt(s['a_rms'])} "
                f"| {s['dev_rms']:.4f} | {s['dev_max']:.4f} |"
            )
        lines.append("")
    lines.append("## 分轨迹明细")
    lines.append("")
    for r in results:
        lines.append(f"### {r['basename']}")
        lines.append("")
        lines.append(
            f"- 关节：{', '.join(r['joint_names'])}；点数 {r['n_raw']}；"
            f"时长 {r['duration']:.2f}s；ε={r['eps']:.4f} rad"
        )
        lines.append(
            f"- 几何保持：平滑样条在原始时刻的偏离 RMS={r['dev_rms']:.4f} rad、max={r['dev_max']:.4f} rad"
        )
        lines.append("")
        lines.append("| 指标 | raw | smooth | standard |")
        lines.append("|---|---|---|---|")
        for key in METRIC_KEYS:
            lines.append(
                f"| {key} | {_fmt(r['metrics']['raw'][key])} "
                f"| {_fmt(r['metrics']['smooth'][key])} "
                f"| {_fmt(r['metrics']['standard'][key])} |"
            )
        lines.append("")
    lines.append("## 结论")
    lines.append("")
    lines.append("`smooth` 相对 `raw` 的 j_rms/a_rms 应下降 ≥ 2 个数量级（去几何量化噪声），")
    lines.append("同时几何偏离受 ε 控制（RMS≈ε、max≈数倍 ε）。ε 越大越平滑但越偏离原始路径——")
    lines.append("据 ε 扫描曲线在「平滑度」与「保形」之间选工作点。`standard` 仅作该时长/首末点下的平滑下限。")
    lines.append("")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="F137 录制轨迹几何去噪 + 平滑重规划")
    ap.add_argument("--dir", default=os.path.expanduser("~/.a3/trajectories"),
                    help="录制轨迹目录")
    ap.add_argument("--latest", type=int, default=3, help="处理最近 N 条 teach_*.yaml")
    ap.add_argument("--traj", action="append", default=None,
                    help="显式指定单条轨迹（可重复）")
    ap.add_argument("--eps", type=float, default=0.01, help="平滑因子 ε（≈RMS 残差目标，rad）")
    ap.add_argument("--resample", type=int, default=400, help="平滑后重采样点数")
    ap.add_argument("--out", default=None,
                    help="报告输出路径（默认 docs/dev/F137_TRAJ_SMOOTHING_REPORT.md）")
    ap.add_argument("--write", default=None,
                    help="把单条轨迹去噪后另存为可回放 YAML（需配合单个 --traj）")
    args = ap.parse_args(argv)

    if args.traj:
        paths = [os.path.expanduser(p) for p in args.traj]
    else:
        paths = discover_latest(os.path.expanduser(args.dir), args.latest)
    if not paths:
        print("未找到录制轨迹", file=sys.stderr)
        return 2

    if args.write:
        if len(paths) != 1:
            print("--write 需配合单个 --traj（不能对多条）", file=sys.stderr)
            return 2
        n = write_smoothed_yaml(paths[0], args.eps, args.write, args.resample)
        print(f"去噪轨迹已写入 {os.path.expanduser(args.write)} ({n} pts, ε={args.eps:.4f})")
        return 0

    results = [process_one(p, args.eps, args.resample) for p in paths]
    sweep = {
        r["basename"]: eps_sweep(r["path"], [0.005, 0.01, 0.02, 0.04], args.resample)
        for r in results
    }

    print(f"{'trajectory':>32} | {'n_raw':>6} | "
          f"{'j_rms_raw':>12} {'j_rms_smooth':>12} {'j_rms_std':>12} | {'dev_max':>8}")
    for r in results:
        m = r["metrics"]
        print(f"{r['basename']:>32} | {r['n_raw']:>6} | "
              f"{_fmt(m['raw']['j_rms']):>12} {_fmt(m['smooth']['j_rms']):>12} "
              f"{_fmt(m['standard']['j_rms']):>12} | {r['dev_max']:>8.4f}")

    out = args.out
    if out is None:
        repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(repo, "docs", "dev", "F137_TRAJ_SMOOTHING_REPORT.md")

    overlay_svg = None
    if results:
        svg_name = "F137_traj_overlay_" + os.path.splitext(results[0]["basename"])[0] + ".svg"
        overlay_svg = os.path.join(os.path.dirname(out), svg_name)
        write_overlay_svg(results[0], overlay_svg)
        overlay_svg = svg_name  # 报告内用相对路径

    write_report(results, sweep, out, overlay_svg)
    print(f"\n报告 -> {out}")

    csv_dir = os.path.dirname(out)
    for r in results:
        csv_path = os.path.join(
            csv_dir, f"F137_metrics_{os.path.splitext(r['basename'])[0]}.csv")
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("method," + ",".join(METRIC_KEYS) + "\n")
            for label in LABELS:
                m = r["metrics"][label]
                f.write(label + "," + ",".join(f"{m[k]:.6g}" for k in METRIC_KEYS) + "\n")
    print(f"CSV -> {csv_dir}/F137_metrics_<name>.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
