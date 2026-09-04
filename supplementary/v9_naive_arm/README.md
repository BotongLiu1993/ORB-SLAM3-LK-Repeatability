# v9_naive_arm — v9 naive-LK 对照臂原始数据（归档 2026-08-20，2026-08-29 补 KITTI 05）

> 用途：v10 正文把 v9 的 naive LK 实现降级为消融臂（§3.2 / Table 4 中
> "naive" 行、Intro 三个失败模式）。本目录保存这些数字的原始出处，保证
> 投稿时可复现。所有文件从 D 盘复制而来，仅做归档，未修改内容。

## 内容与来源

| 子目录 | 内容 | D 盘来源 |
|---|---|---|
| `supplementary_v9/` | 旧版 S0–S4（S0_kitti00.csv、S1_per_run_ATE.csv、S2_adoption.csv、S3_failure_excerpts.txt、S3_frame_events.csv、S4_lk.patch、S4_adaptive_gate.patch、README.txt） | `D:\0 科研学习\SLAM\paper\supplementary\` |
| `kitti00_evo/` | KITTI 00 v9 批次（n=3，τ=0.40）：baseline/LK 各 3 轮 evo 汇总 + 轨迹 + 逐帧耗时 + LK 统计 | `D:\0 科研学习\SLAM\result\evo_results\kitti00_*` |
| `kitti05_evo/` | KITTI 05 v9 批次（τ=0.40）：baseline/LK 的 evo 汇总 + 轨迹 + 耗时 + LK 统计（r1 的原始 evo 未在 D 盘存档，见下方缺口 2） | `D:\0 科研学习\SLAM\result\evo_results\kitti05_*` |
| `tum_tau_ablation/` | fr3_office / fr3_sitting 的 τ=0.10–1.00 门阈值消融：逐档 + 逐轮 evo / lkstats / times / timing 文本（192 个） | `D:\0 科研学习\SLAM\result\evo_results\tum_experiments\ablation\` |

## 关键口径（与正文一致）

- ATE 一律 evo SE(3) Umeyama（不修尺度）；TUM 用 `evo_ape tum -a`，KITTI 用 `evo_ape kitti -a`。
- KITTI 00 三跑批次（n=3 per method，τ=0.40）：基线 rmse 1.1979/1.1622/1.6262 m，LK rmse
  1.2156/2.1062/1.2916 m（均值 +15.7%，ns；统一置换 p 值见 `../eval/pvalues_v10.py` 与 S1）。
- KITTI 05 τ 消融（正文口径）：per-run ATE 见 `supplementary_v9/S1_per_run_ATE.csv`
  （kitti05_ablation 组）：基线均值 0.8948 m，τ=0.10 均值 0.9447 m（+5.6%），
  τ=1.00 均值 1.1658 m（+30.3%）。
- τ 消融（fr3_office/fr3_sitting）：τ=0.10→1.00 的逐档逐轮 evo 原始输出在
  `tum_tau_ablation/`。

## ⚠️ 已知缺口（引用前必须处理）

1. **KITTI 00 τ=1.00（+37.3%）已从正文删除（2026-08-20 作者确认）**。正文改引
   KITTI 05 τ 消融（+5.6% τ=0.10 → +30.3% τ=1.00），per-run ATE 以 S1 csv
   （kitti05_ablation 组）为准。
2. **KITTI 05 的 r1 原始 evo 文件未在 D 盘存档**：`kitti05_evo/` 只有 baseline/LK
   的 r2、r3 及聚合（无 _rN 后缀的 run=0）evo 汇总；全部三轮 per-run ATE 见 S1。
   另外 `kitti05_lk_r2/r3_stats.txt` 与 `*_rot_stats.txt` 部分为空文件（日志未解析出
   对应统计块），保留原样以便审稿人核验。
3. 完整轨迹/逐帧耗时的超大文件（部分 `*_traj.txt`、`*_kf_traj.txt`、`*_cam_traj.txt`）
   已复制常见批次；其余全部保留在 D 盘上述来源路径。
