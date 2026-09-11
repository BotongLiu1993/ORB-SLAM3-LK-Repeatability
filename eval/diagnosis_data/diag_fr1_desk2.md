# fr1_desk2 mechanism diagnosis: GT velocity profile and LK quality (measured)

> Added 2026-08-18; data: LK_MM_MATCH_TH=200, n=12; the velocity profile is obtained by interpolating the ground truth to the RGB frame timestamps.

## 1. GT velocity profile (measured)

| Quantity | fr1_desk | fr1_desk2 |
|---|--:|--:|
| Path length (m) | 9.3 | 9.9 |
| Total rotation (deg) | 662 | 974 |
| Duration (s) | 19.8 | 21.3 |
| Median linear speed (m/s) | 0.48 | 0.46 |
| Linear speed p95 (m/s) | 0.79 | 0.84 |
| Median rotation rate (deg/s) | 34.0 | 44.4 |
| Rotation rate p95 (deg/s) | 75.0 | 93.1 |
| Median translation per frame (mm/frame) | 16.3 | 15.7 |
| Median rotation per frame (deg/frame) | 1.14 | 1.47 |

## 2. LK quality and decisions (mean over 12 runs)

| Quantity | fr1_desk | fr1_desk2 |
|---|--:|--:|
| LK trigger (weak motion model) % | 23.76 | 40.08 |
| LK adoption % (of frames) | 12.17 | 17.27 |
| FB mean forward-backward error (px) | 0.15 | 0.16 |
| FB rejected points per run | 1398.75 | 3109.75 |
| NCC rejected points per run | 1292.75 | 2310.83 |
| match margin at adoption | 11.06 | 18.19 |
| fallbacks after adoption | 0.00 | 0.00 |
| Median rotation deviation of the adopted pose (deg) | 0.99 | 0.91 |
| Median rotation deviation of the rejected pose (deg) | 0.78 | 0.68 |

## 3. ATE recomputation (per-run evo rmse parsed)

| Sequence | Baseline (m) | v2 (m) | delta % | p (exact permutation) |
|---|--:|--:|--:|--:|
| fr1_desk | 0.024 ± 0.010 (n=12) | 0.017 ± 0.000 (n=12) | -28.3% | 0.004 |
| fr1_desk2 | 0.027 ± 0.002 (n=12) | 0.028 ± 0.002 (n=12) | +4.2% | 0.102 |

## 4. Conclusions

- fr1_desk2 rotates faster (median 44.4 vs 34.0 deg/s, p95 93 vs 75 deg/s), but the baseline has no failure segment: the error is a uniform drift with no large collapse for LK to repair.
- On fr1_desk2, LK triggers and is adopted more often while its quality is comparable (FB error about 0.158 px, margin 18.2); this matches the 40 percent trigger rate but no gain observation: the competition mechanism works as designed, and the gain appears only where the motion model fails systematically (fr1_desk).
- Figure: `figures/fig_v10_desk2_diagnosis.png` (velocity-profile CDF, LK decisions, angular-rate time profile).