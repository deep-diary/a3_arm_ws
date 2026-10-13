# F167.1 A/B：软件模式时间律 software vs totg（仿真实采）

同一组软件模式、同一 5 点路径，仅 `goto_via_time_backend` 不同；
几何路径（路径长）应一致，时间律（时长 / j_rms）允许差异。

| 模式 | backend | 时长(s) | j_rms(rad/s³) | a_rms(rad/s²) | TCP路径长(m) | 几何相对差 |
|---|---|---|---|---|---|---|
| jnt_g64_quintic | software | 5.50 | 6.1 | 0.6 | 0.3444 | - |
| jnt_g64_quintic | totg | 3.28 | 16.2 | 1.4 | 0.3438 | 0.17% ✓ |
| jnt_g64_trapezoid | software | 4.98 | 8.2 | 0.7 | 0.3445 | - |
| jnt_g64_trapezoid | totg | 3.24 | 16.3 | 1.4 | 0.3430 | 0.44% ✓ |
| jnt_spline_bell | software | 11.14 | 2.5 | 0.3 | 0.3456 | - |
| jnt_spline_bell | totg | 5.06 | 10.5 | 1.0 | 0.3477 | 0.62% ✓ |
| jnt_spline_approx | software | 4.34 | 1.4 | 0.3 | 0.3429 | - |
| jnt_spline_approx | totg | 2.96 | 11.7 | 0.9 | 0.3440 | 0.33% ✓ |

- 几何一致判定（路径长相对差 ≤ 5%）：PASS ✓
