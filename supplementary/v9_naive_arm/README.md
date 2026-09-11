# v9_naive_arm - raw data of the v9 naive-LK ablation arm (archived 2026-08-20; KITTI 05 added 2026-08-29)

> Purpose: the v10 manuscript demotes the v9 naive-LK implementation to an ablation arm
> (the "naive" rows of Section 3.2 / Table 4, and the three failure modes in the Introduction).
> The original files are archived here unmodified so that every number can be reproduced.

## Contents and sources

| Subfolder | Contents | Original location (D: drive) |
|---|---|---|
| `supplementary_v9/` | Legacy S0-S4 (S0_kitti00.csv, S1_per_run_ATE.csv, S2_adoption.csv, S3_failure_excerpts.txt, S3_frame_events.csv, S4_lk.patch, S4_adaptive_gate.patch, README.txt) | `D:\SLAM\paper\supplementary\` |
| `kitti00_evo/` | KITTI 00, v9 batch (n=3, tau=0.40): baseline/LK evo summaries for 3 runs + trajectories + per-frame timings + LK statistics | `D:\SLAM\result\evo_results\kitti00_*` |
| `kitti05_evo/` | KITTI 05, v9 batch (tau=0.40): baseline/LK evo summaries + trajectories + timings + LK statistics (the raw r1 evo output was not archived on the D: drive, see gap 2 below) | `D:\SLAM\result\evo_results\kitti05_*` |
| `tum_tau_ablation/` | tau=0.10-1.00 gate-threshold sweep on fr3_office / fr3_sitting: per-threshold and per-run evo / lkstats / times / timing text files (192 files) | `D:\SLAM\result\evo_results\tum_experiments\ablation\` |

## Definitions used in the manuscript

- ATE is always evo SE(3) Umeyama (no scale correction): `evo_ape tum -a` for TUM, `evo_ape kitti -a` for KITTI.
- KITTI 00, three-run batch (n=3 per method, tau=0.40): baseline rmse 1.1979/1.1622/1.6262 m, LK rmse
  1.2156/2.1062/1.2916 m (mean +15.7%, ns; unified permutation p-values in `../eval/pvalues_v10.py` and S1).
- KITTI 05 tau sweep (manuscript definition): per-run ATE in `supplementary_v9/S1_per_run_ATE.csv`
  (group kitti05_ablation): baseline mean 0.8948 m, tau=0.10 mean 0.9447 m (+5.6%),
  tau=1.00 mean 1.1658 m (+30.3%).
- tau sweep (fr3_office/fr3_sitting): the raw per-threshold, per-run evo output for tau=0.10 to 1.00 is in
  `tum_tau_ablation/`.

## Known gaps (resolve before citing)

1. **The KITTI 00 tau=1.00 figure (+37.3%) was removed from the manuscript (confirmed by the author on 2026-08-20).** The manuscript instead cites
   the KITTI 05 tau sweep (+5.6% at tau=0.10 to +30.3% at tau=1.00), with per-run ATE taken from the S1 csv
   (group kitti05_ablation).
2. **The raw r1 evo output for KITTI 05 was not archived on the D: drive**: `kitti05_evo/` contains only the baseline/LK
   r2, r3 and aggregate (no _rN suffix, run=0) evo summaries; all three per-run ATE values are in S1.
   In addition, `kitti05_lk_r2/r3_stats.txt` and `*_rot_stats.txt` are partly empty (the log did not yield the
   corresponding statistics block); they are kept as-is for reviewer inspection.
3. Very large full-trajectory / per-frame timing files (some `*_traj.txt`, `*_kf_traj.txt`, `*_cam_traj.txt`)
   were copied only for the most common batches; all remaining files stay on the D: drive paths listed above.
