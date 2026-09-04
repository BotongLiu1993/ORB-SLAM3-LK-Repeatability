# S4 — Reproduction guide for the final v10 (LK-v2) batch

This file describes how every number in the v10.2 manuscript was produced,
and where the code and data live. All paths under `v10_work/` refer to the
submission archive `archive_v10.2_delivery/` (a copy of the scripts used in
this study).

## 1. Code state and patch

- Base: ORB-SLAM3 commit snapshot (`ORB_SLAM3_wsl_snapshot/`, git HEAD).
- Method patch: `S5_lk_v2.patch` (this directory) — the final
  competing-hypothesis LK motion prior applied to `src/Tracking.cc` /
  `include/Tracking.h`, plus System/example-program hooks for printing LK
  decision statistics and per-frame timing.
- Reference implementation for review: `Tracking_v2.cc` / `Tracking_v2.h`
  (v10_work; identical to the patched files).
- Defaults: `LK_MM_MATCH_TH = 200` (nmatches < 200 triggers LK), `LK_MAX_POINTS = 400`,
  `LK_FB_TH = 1.5 px`, `LK_NCC_TH = 0.75`, `LK_PNP_REPROJ = 3.0 px`.

## 2. Build & run (WSL, Ubuntu 20.04, 12 cores)

1. `bash deploy_v2.sh <WORKDIR>` — backs up current code, copies the v2
   patch, restores clean System/example programs, disables the viewer, and
   builds `rgbd_tum`, `stereo_euroc`, `stereo_kitti`.
2. `bash run_v2_tum.sh` — TUM RGB-D, baseline vs v2, n = 3 (fr1_room) / n = 12
   (fr1_desk, fr1_desk2, fr1_360, fr1_floor); writes trajectories, evo
   outputs, per-frame times to `results/v2_tum/`.
3. `bash run_v2_euroc.sh V101 V102 V103` — EuRoC stereo-only, n = 3;
   `results/v2_euroc/`.
4. `bash run_v2_kitti.sh` — KITTI 00/02/05/07/09/10, n = 3;
   `results/v2_kitti/`.
5. `bash run_v2_mmth_ablation.sh` — LK_MM_MATCH_TH sweep on fr1_desk (n = 2,
   TH = 40–250); `results/v2_mmth_ablation/` (Table 4), archived to
   `result/evo_results/tum_experiments/v2_mmth_ablation/` and verified against
   Table 4 on 2026-08-20.

Dataset installation/verification: `install_datasets.sh` (TUM RGB-D,
EuRoC ASL, KITTI gray odometry), `check_euroc_align.py` (frame counts and
GT-path sanity), plus the frame/path-length verification reported in S0.

## 3. Evaluation (local, `myenv` Python)

- `pvalues_v10.py` — exact two-sided paired permutation tests for all TUM and
  KITTI comparisons (statistic |mean(v-b)|, 2^n sign flips, 1e-12 tolerance).
- `eval_euroc_archival.py` — EuRoC ATE (SE(3) Umeyama, no scale) against the
  official left-camera GT `evaluation/Ground_truth/EuRoC_left_cam/{seq}_GT.txt`,
  direct timestamp matching (tol 0.03 s, no time-offset correction), plus
  permutation p-values and scale ratios. Outputs evo-style per-run files and
  per-frame error CSVs (see `archive_euroc_v10.2/`).
- `make_fig2_v10.py` — Figure 2 (TUM ablation/timing).
- `diag_fr1_desk2.py` — fr1_desk2 mechanism diagnosis (Figure 3).

## 4. Data files

- `S0_dataset_overview.csv` — sequences, sensors, motion classes, n, data
  batch, path lengths/durations. Path lengths are computed from the archived
  ground truth where available (`GT` source) and from the baseline r1
  estimated trajectory otherwise (`baseline est.`); they follow the
  "accumulated Euclidean distance over all rows" convention and may differ by
  a few percent from official benchmark pages (downsampling conventions).
- `S1_per_run_ATE.csv` + `S1_pooled_summary.csv` — every run's ATE RMSE
  (TUM 9 seqs, KITTI 6 seqs, EuRoC 3 seqs; 180 runs) and pooled statistics.
- `S2_lk_decision_stats.csv` — LK trigger/adoption/margin/FB/NCC/fallback.
- `S3_failure_events.csv` — degraded runs, relocalization counts, and the v9
  naive-LK failure modes used as motivation.

## 5. Reproducibility notes

- ORB-SLAM3 is non-deterministic; all reported comparisons use n = 3–12 runs
  and exact permutation p-values. Single-run numbers are not meaningful.
- EuRoC estimate timestamps (camera clock) are matched directly to the GT
  timestamps (same time base); do NOT add a fixed offset (an earlier
  evaluation bug added +1.04 s and inflated ATE 5–11x; values 0.45/0.93 m
  are obsolete).
- The naive-LK arm (Section 3.2) uses the v9 patch `S4_lk.patch` data as a
  contrast baseline; only the v2 arm corresponds to the final method.
