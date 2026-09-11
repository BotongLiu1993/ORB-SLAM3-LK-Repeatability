# EuRoC stereo-only re-evaluation (corrected evaluation, 2026-08-19)

> **Correction (important)**: the earlier `re_eval_euroc.sh` used `state_groundtruth_estimate0/data.csv` (body frame, EKF estimate) and forced a fixed time offset (V101 +1.04 s, etc.), which misaligned the estimated trajectory against the ground-truth frames by about 21 frames and inflated ATE by 5-11x (the values V101 0.45 m / V103 0.93 m are wrong).
> **Corrected protocol**: ground truth is the official ORB-SLAM3 repository file `evaluation/Ground_truth/EuRoC_left_cam/{seq}_GT.txt` (left-camera trajectory, 20 Hz); estimated timestamps (camera clock) are matched by direct time association (camera and vicon share a clock domain, and the later ground-truth start is a recording-origin difference rather than a clock offset), SE(3) Umeyama (no scale correction), ATE RMSE in m.
> Archival scripts: `v10_work/eval_euroc_corrected.py` and `v10_work/eval_euroc_archival.py` (with permutation p-values); per-run evo-style outputs, per-frame errors and the summary are in `v10_work/archive_euroc_v10.2/` (evo_outputs/ + summary_per_run.csv + summary_pooled.txt).
> p-value definition: paired difference mean |mean(v-b)|, 2^n enumeration, 1e-12 tolerance; after fixing a floating-point artefact at the boundary on 2026-08-19, V102 gives p=1.000 (previously 0.750).

## Per-run and pooled results (n=3)

| Sequence | Method | r1 | r2 | r3 | mean+-std | delta mean | p | scale ratio est/GT |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|---|
| V101 | Baseline | 0.0399 | 0.0360 | 0.0554 | 0.0438 ± 0.0104 | — | — | 1.015–1.024 |
| V101 | +LK-v2 | 0.0344 | 0.0374 | 0.0357 | 0.0359 ± 0.0015 | −18.1% | 0.500 | 1.018–1.029 |
| V102 | Baseline | 0.0379 | 0.0422 | 0.0353 | 0.0385 ± 0.0036 | — | — | 1.019–1.022 |
| V102 | +LK-v2 | 0.0314 | 0.0308 | 0.0583 | 0.0401 ± 0.0155 | +4.3% | 1.000 | 1.020–1.024 |
| V103 | Baseline | 0.2146 | 0.1174 | 0.1624 | 0.1648 ± 0.0490 | — | — | 1.068–1.158 |
| V103 | +LK-v2 | 0.0525 | 0.0664 | 0.1618 | 0.0936 ± 0.0598 | −43.2% | 0.250 | 1.055–1.064 |

## Conclusions (basis for the manuscript wording)
- After the correction the absolute magnitudes agree with the literature: V101/V102 ~ 0.03-0.06 m; V103 baseline 0.12-0.21 m (difficult sequence).
- Direction: V101 -18.1%, V103 -43.2% (v2 lower), V102 +4.3% (v2 r3=0.0583, a single-run outlier); with n=3 none of the exact permutation tests is significant (p=0.500/1.000/0.250), so the manuscript states "numerically better or comparable, statistically neutral".
- **V103 scale drift**: the baseline estimated path length / GT = 1.07-1.16, while v2 converges to 1.055-1.064; this is consistent with LK continuously correcting frames where the motion model is weak (the most valuable directional evidence for v2 on EuRoC).
- Data provenance: the EuRoC_TimeStamps files have 2912/1710/2149 lines (unchanged from the snapshotted git HEAD); the ground-truth path lengths 58.6/75.9/79.0 m match the official values; EuRoC.yaml is the official stereo configuration (stereo calibration fx=458.654, baseline 0.110 m).
- Timing (unaffected by the evaluation fix and still valid): baseline median 5.4-6.6 ms, v2 +0.9-2.0 ms per frame (+10 to +42%, largest on V103); 3.5-6 ms per trigger; LK triggers on 24-41% of frames, forward-backward error <= 0.02 px, zero fallbacks (see euroc_timing_mechanism.md).
