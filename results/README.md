# Results (sample expected outputs)

This folder contains the three per-platform result summaries produced by the
final LK-v2 batch (gate `LK_MM_MATCH_TH = 200`):

- `results_v2_tum_summary.md`  - TUM RGB-D (fr1_desk/fr1_desk2 n = 12, etc.)
- `results_v2_euroc_summary.md` - EuRoC stereo-only, official GT re-evaluation
- `results_v2_kitti_summary.md` - KITTI odometry (6 sequences, n = 3)

These files document the exact numbers reported in the paper and serve as
reference outputs when re-running the experiments.

Complete per-run data (trajectories, per-frame timings, evo outputs, LK
decision statistics) are archived under `../supplementary/`,
`../eval/archive_euroc_v10.2/`, and `../supplementary/v9_naive_arm/`; see the
top-level README section 7 for the mapping.
