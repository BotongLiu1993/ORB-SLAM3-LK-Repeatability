# v2 TUM RGB-D results (LK_MM_MATCH_TH=200)

Run date: 2026-08-18/19 (fr1_desk & fr1_desk2 n=12; fr1_room n=3; fr1_360/fr1_floor n=12 completed on 2026-08-19).

> p-value definition (unified on 2026-08-19): exact two-sided paired permutation test, statistic = |mean(v-b)|, 2^n enumeration (`pvalues_v10.py`). The earlier values fr2_desk 0.201 / fr3_sitting 0.301 / fr3_office 0.596 used a different definition and are superseded.

| Sequence | n | Base ATE mean | V2 ATE mean | dMean% | p (two-sided perm) | V2 med | Base med | LK att% | LK adopt% | Bad B/V | Time med B/V (ms) |
|---|:--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| fr1_desk | 12 | 0.0238 | 0.0170 | -28.3% | 0.004 | 0.0169 | 0.0189 | 23.8% | 51.2% | 2 / 0 | 9.38 / 10.12 |
| fr1_desk2 | 12 | 0.0271 | 0.0283 | +4.2% | 0.102 | 0.0284 | 0.0270 | 40.1% | 43.1% | 1 / 1 | 9.53 / 11.22 |
| fr1_room | 3 | 0.0666 | 0.0720 | +8.0% | 0.500 | 0.0728 | 0.0669 | 26.3% | 34.6% | 0 / 0 | 6.97 / 8.63 |
| fr1_360 | 12 | 0.2098 | 0.1981 | -5.6% | 0.533 | 0.1951 | 0.2214 | 37.2% | 30.5% | 0 / 0 | 5.78 / 7.43 |
| fr1_floor | 12 | 0.4017 | 0.2609 | -35.1% | 0.169 | 0.1145 | 0.3753 | 55.2% | 17.9% | 7 / 3 | 4.98 / 6.58 |
| fr2_xyz | 3 | 0.0039 | 0.0040 | +0.4% | 1.000 | 0.0040 | 0.0040 | 0.5% | 22.2% | 0 / 0 | 7.13 / 7.27 |
| fr2_desk | 3 | 0.0181 | 0.0189 | +4.5% | 0.250 | 0.0192 | 0.0176 | 12.1% | 22.4% | 0 / 0 | 9.02 / 9.32 |
| fr3_sitting | 3 | 0.0093 | 0.0091 | -2.2% | 0.250 | 0.0092 | 0.0094 | 19.6% | 28.7% | 0 / 0 | 6.20 / 6.86 |
| fr3_office | 3 | 0.0105 | 0.0127 | +20.5% | 1.000 | 0.0106 | 0.0109 | 0.7% | 38.5% | 0 / 0 | 9.89 / 9.88 |

Bad B/V: number of degraded runs (threshold ATE > 0.3 m for fr1_floor; otherwise clearly degraded runs). Per-run ATE:
- fr1_floor (n=12): base 0.0950 0.5072 0.9038 0.4341 0.0995 0.5068 0.1051 0.0941 0.3165 0.7262 0.9170 0.1155 | v2 0.0631 0.0752 0.1134 0.7601 0.0954 0.5601 0.1874 0.0932 0.1156 0.0932 0.7377 0.2365
- fr1_360 (n=12): base 0.2213 0.2252 0.2214 0.2305 0.1827 0.1653 0.2744 0.1771 0.1762 0.1733 0.2335 0.2370 | v2 0.2041 0.2095 0.1585 0.2219 0.3083 0.1387 0.1754 0.1900 0.2389 0.2002 0.1495 0.1824
- fr1_room (n=3): base 0.0669 0.0723 0.0608 | v2 0.0800 0.0631 0.0728

## Conclusions (paper-ready wording)
- fr1_desk (n=12): the only statistically significant positive result, -28.3% (p=0.004); the 2 degraded baseline runs are removed, CoV 46% -> 2%.
- fr1_floor (n=12): directional improvement but **not significant** - mean -35.1% (p=0.169), median -69.5% (0.375 -> 0.115), degraded runs (>0.3 m) 7/12 -> 3/12 and catastrophic runs (>0.5 m) 5/12 -> 3/12; however v2 still has 3 degraded runs (r4=0.760 / r6=0.560 / r11=0.738, with r6/r11 degraded in the baseline too) and 4/12 runs are worse than the baseline (r4 0.434 -> 0.760), so the manuscript can only claim a directional improvement with fewer failures, not a second positive result.
- fr1_360 (n=12): -5.6% (p=0.533), neutral; the "fast-rotation positive-result candidate" claim was withdrawn.
- fr1_room (n=3): +8.0% (p=0.50), neutral.
- Control group: statistically neutral throughout, no degradation.
- The never-worse claim is restricted to: no statistically significant degradation on the control/smooth sequences, and no degraded runs on fr1_desk; it is not guaranteed per run on fr1_floor.
- LK trigger rate: 24-55% on fast-motion sequences (55% on fr1_floor), <1-20% on the control group; adoption rate 18-51% of triggering frames.
- Median per-frame cost (pooled n=12): +24 to +32% on the fast-motion group (+1.6 to +1.7 ms absolute); +0.2 to +1 ms on the control group. Reported as measured; the "<5%" wording is not used.
