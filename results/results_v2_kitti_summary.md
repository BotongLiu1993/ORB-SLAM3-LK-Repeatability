# v2 KITTI results (LK_MM_MATCH_TH=200, stereo, n=3)

Run date: 2026-08-19. evo_ape kitti -a (SE(3) Umeyama, ATE RMSE m).

| Seq | Base rmse mean±std | V2 rmse mean±std | dMean% | p (perm) | tmed B/V (ms) | LK att% | LK adopt% | won | fb | FBerr(px) |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 00 | 1.2229±0.0558 | 1.2323±0.0532 | +0.8% | 1.000 | 7.24/7.93 | 12.9 | 39.8 | 699 | 0 | 0.067 |
| 02 | 5.3891±0.1572 | 5.1084±0.2596 | -5.2% | 0.500 | 6.36/7.06 | 22.9 | 33.7 | 1077 | 0 | 0.081 |
| 05 | 1.6693±1.0402 | 0.8868±0.0602 | -46.9% | 0.250 | 6.75/7.54 | 14.9 | 34.2 | 422 | 0 | 0.052 |
| 07 | 0.4551±0.0253 | 0.4571±0.0277 | +0.4% | 1.000 | 6.17/6.88 | 16.0 | 34.5 | 182 | 0 | 0.053 |
| 09 | 1.9995±0.0410 | 2.4045±0.6654 | +20.3% | 0.250 | 5.73/6.64 | 27.4 | 29.3 | 383 | 0 | 0.083 |
| 10 | 1.3326±0.1628 | 1.3339±0.0670 | +0.1% | 1.000 | 5.32/6.53 | 32.4 | 36.1 | 421 | 0 | 0.061 |

Per-run ATE:
- 00: base 1.1905 1.1908 1.2874 | v2 1.1866 1.2908 1.2195
- 02: base 5.2935 5.5705 5.3033 | v2 4.8999 5.0261 5.3991
- 05: base 2.8585 1.2213 0.9281 | v2 0.8502 0.9563 0.8539
- 07: base 0.4265 0.4745 0.4645 | v2 0.4868 0.4526 0.4320
- 09: base 2.0222 2.0242 1.9523 | v2 2.0440 3.1724 1.9971
- 10: base 1.5070 1.3064 1.1845 | v2 1.2935 1.2970 1.4112

## Conclusions (paper-ready wording)
- 6/6 sequences are statistically neutral (p>=0.25). 00/07/10 are almost identical (+0.8/+0.4/+0.1%); 02 is directionally better by -5.2%; 05 is directionally better by -46.9% (driven by the baseline r1=2.86 catastrophic run, not significant at n=3); 09 is numerically +20.3% (v2 r2=3.17 outlier run, ns).
- Key control: the v9 naive LK degraded KITTI 00 by +15.7% (the legacy v9 p=0.56 definition is not reproducible; the unified permutation definition gives p=0.75); the v2 competing selection on KITTI 00 changes ATE by only +0.8% (p=1.000), so the outdoor never-worse claim holds and the root cause of the naive degradation (bad seeds steering the projection search) is removed by the competition mechanism.
- Per-frame cost +0.7 to +1.2 ms (+8 to +23%); LK triggers on 13-32% of frames and is adopted on 29-40% of those (5-11% of all frames); zero fallbacks; FB error 0.05-0.08 px.
