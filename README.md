# Verified Competing-Hypothesis Motion Prediction with Lucas-Kanade Optical Flow for Robust Tracking in ORB-SLAM3: A Repeated-Run Study

Repository for the PeerJ Computer Science submission (manuscript #146737):

> **Verified competing-hypothesis motion prediction with Lucas-Kanade optical flow for robust tracking in ORB-SLAM3: a repeated-run study** (2026, in review).

This package contains the code patch, experiment scripts, per-run data, and
statistical evaluation for the paper. It accompanies the manuscript submitted
to PeerJ and fulfils the journal's code/data availability and reproducibility
requirements.

---

## 1. Description

ORB-SLAM3 tracks most frames with a constant-velocity motion model. That
assumption fails under fast and aggressive motion, and because the pipeline is
randomized, repeated runs of the same sequence can give qualitatively different
outcomes. This study:

1. **Shows the naive fix does not work** (v9 "naive" arm, kept for ablation):
   replacing the motion model with unverified Lucas-Kanade (LK) optical flow is
   statistically neutral on smooth sequences, degrades on unverified outdoor
   flow, and can silently diverge.
2. **Converts the negative results into a verified method** (LK-v2, this
   repository): a *competing-hypothesis motion prior* that
   - computes an LK pose hypothesis from forward-backward- and NCC-verified
     optical flow **only when the constant-velocity model is weak**
     (`nmatches < LK_MM_MATCH_TH`, default 200);
   - makes LK compete with the motion-model seed by **projection-matching
     inlier count** and adopts the winner (guaranteeing no degradation at the
     seed level);
   - **falls back** to the motion-model seed when the LK-seeded optimization
     collapses (sudden inlier drop).
3. **Evaluates under a repeated-run protocol** (n = 3-12 runs per
   sequence/method, exact two-sided permutation tests) on TUM RGB-D, EuRoC MAV,
   and KITTI odometry.

### Headline results (see README section 7 and the paper)

| Setting | Result |
|---|---|
| TUM fr1_desk (n = 12) | ATE 0.0238 -> 0.0170 m, **-28.3%, p = 0.004**; degraded runs 2/12 -> 0/12; run-to-run CoV 46% -> 2% |
| TUM fr1_floor (n = 12) | -35.1% (p = 0.169, directional); degraded runs 7/12 -> 3/12 |
| EuRoC stereo-only (n = 3) | V101 -18.1%, V103 -43.2% (directional, ns); V103 path-length scale drift 7-16% -> 5-6% |
| KITTI odometry (n = 3, 6 sequences) | all statistically neutral; naive outdoor degradation removed (KITTI 00: +0.8%, p = 1.000) |
| Per-frame cost | median +0.9-2.0 ms overall; 3.5-6 ms on LK-triggering frames |

## 2. Repository layout

| Folder | Contents |
|---|---|
| `patch/` | `S5_lk_v2.patch` (final method patch) and the reference implementation `Tracking_v2.cc` / `Tracking_v2.h` |
| `build/` | `deploy_v2.sh` (applies LK-v2 to an ORB-SLAM3 tree and builds the example binaries) |
| `run/` | experiment scripts: `run_v2_tum.sh`, `run_v2_kitti.sh`, `run_v2_euroc.sh`, `run_v2_mmth_ablation.sh`, resume/re-eval helpers |
| `eval/` | statistics and evaluation: `pvalues_v10.py`, `eval_euroc_archival.py`, `eval_euroc_corrected.py`, figure/diagnosis scripts, `archive_euroc_v10.2/` (per-run EuRoC evo outputs), `diagnosis_data/` (fr1_desk2 GT subset for Figure 3) |
| `datasets/` | `install_datasets.sh` (convenience extractor for the author's WSL layout) and `README.md` (official download links, expected layout) |
| `supplementary/` | S0 dataset overview, S1 per-run ATE (+pooled summary), S2 LK decision statistics, S3 failure events, S4 reproduction guide, S5 patch, and `v9_naive_arm/` (raw v9 naive-LK data for the Section 3.2 ablation arm) |
| `figures/` | the six figures of the paper (PNG + PDF) |
| `results/` | the three per-platform result summaries (sample expected outputs) |
| `docker/` | optional, **unvalidated** `Dockerfile.example` (not used for any number in the paper) |

The full ORB-SLAM3 source tree and the benchmark datasets are **not** bundled
(see section 3); the patch and scripts are applied to a standard ORB-SLAM3
clone.

## 3. Dataset information

Three public benchmarks are used. None are included in this repository because
of size and license terms; download them from the official sources below.

| Benchmark | Sequences used (n per method) | Official source |
|---|---|---|
| TUM RGB-D | `fr1_desk` (12), `fr1_desk2` (12), `fr1_360` (12), `fr1_floor` (12), `fr1_room` (3), `fr2_xyz` (3), `fr2_desk` (3), `fr3_long_office_household` (3), `fr3_sitting_xyz` (3) | https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download |
| EuRoC MAV (ASL format) | `V1_01_easy`, `V1_02_medium`, `V1_03_difficult` (3) | https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets |
| KITTI odometry | `00`, `02`, `05`, `07`, `09`, `10` (3) | https://www.cvlibs.net/datasets/kitti/eval_odometry.php |

Ground truth: TUM and KITTI use their official ground-truth files; EuRoC uses
the official left-camera ground truth shipped with ORB-SLAM3
(`evaluation/Ground_truth/EuRoC_left_cam/`), with direct timestamp matching
(no time-offset correction).

Expected layout inside the ORB-SLAM3 tree (used by the run scripts):

```
$ORB_SLAM3_ROOT/
  datasets/
    rgbd_dataset_freiburg1_desk/          # TUM: extracted sequence folders
    ...
    euroc/V1_01_easy/mav0/                # EuRoC: ASL zip extracted per sequence
    euroc/V1_02_medium/mav0/
    euroc/V1_03_difficult/mav0/
    kitti/dataset/sequences/00/           # KITTI: gray/colour + poses
    kitti/dataset/sequences/02/
    ...
    kitti/dataset/poses/
```

`datasets/install_datasets.sh` is a convenience extractor written for the
author's WSL setup (`DATASET_SRC` env var overrides the source folder). For
reviewers, simply download and extract the sequences to the layout above.

## 4. Requirements (environment)

The paper's numbers were produced on: **Ubuntu 20.04 (WSL2), 12-core x86-64,
12 GB RAM, viewer disabled, runs executed sequentially**.

- **OS**: Ubuntu 20.04/22.04 (native or WSL2).
- **Toolchain**: GCC >= 9, CMake >= 3.14, Make.
- **Libraries** (ORB-SLAM3 dependencies): OpenCV (>= 4.2, with contrib), Eigen3,
  Pangolin, Boost, GLog, GFlags; ORB-SLAM3's `Thirdparty/` (g2o, DBoW2,
  Sophus) is built automatically by ORB-SLAM3's own `build.sh`.
- **Python 3.8+** (evaluation scripts): `numpy`, `scipy`, `pandas`,
  `matplotlib`; `evo` (`pip install evo --upgrade --no-binary evo`) for
  `evo_ape`.
- A git clone of the official **ORB-SLAM3** repository (the deploy script uses
  `git show HEAD` to snapshot the baseline files).

## 5. Usage instructions

Let `$REPO` be this repository and `$ORB_SLAM3_ROOT` the ORB-SLAM3 tree
(defaults to `$HOME/ORB_SLAM3`; override with the `ORB_SLAM3_ROOT` env var).

**Step 1 - Build ORB-SLAM3 (once).**
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$HOME/ORB_SLAM3"
cd "$HOME/ORB_SLAM3"
chmod +x build.sh && ./build.sh          # builds Thirdparty + ORB_SLAM3
```

**Step 2 - Deploy LK-v2 and build the example binaries.**
```bash
bash "$REPO/build/deploy_v2.sh" "$REPO/patch"
```
This snapshots the baseline files, copies `Tracking_v2.cc/.h` into the tree,
adds the LK statistics hook, disables the viewer (headless), and builds
`rgbd_tum` and `stereo_kitti`. It is idempotent and backs up prior states
(`*.bak_tum_v2`, `code_backups_tum/*.baseline`).

**Step 3 - Download the datasets** (section 3) into
`$ORB_SLAM3_ROOT/datasets/` as shown above.

**Step 4 - Run the repeated-run experiments** (all commands from
`$ORB_SLAM3_ROOT`):

```bash
# TUM RGB-D (all 9 sequences; ~hours). Phases: all | fast | control |
#   sel <seq...>; n per sequence as in section 3.
bash "$REPO/run/run_v2_tum.sh" all

# KITTI stereo (6 sequences, n = 3)
bash "$REPO/run/run_v2_kitti.sh"

# EuRoC stereo-only (V101 V102 V103, n = 3)
bash "$REPO/run/run_v2_euroc.sh" V101 V102 V103

# Gate-threshold sweep for Table 4 (TH = 40..250, n = 2)
bash "$REPO/run/run_v2_mmth_ablation.sh"
```

Each script writes per-run trajectories, per-frame timings, evo outputs, and
LK decision statistics under `results/v2_*` inside the ORB-SLAM3 tree.

**Step 5 - Statistics and figures** (paths are overridable with the env vars
listed below):

```bash
# Exact two-sided paired permutation tests for TUM/KITTI
python "$REPO/eval/pvalues_v10.py"

# EuRoC archival evaluation (official GT, direct timestamp matching)
python "$REPO/eval/eval_euroc_archival.py"

# Figure 2 (TUM fast-motion paired results) - auto-locates results/v2_tum
python "$REPO/eval/make_fig2_v10.py"

# fr1_desk2 mechanism diagnosis (Figure 3)
bash "$REPO/eval/diag_fr1_desk2.sh"
```

Environment variables (all optional; defaults are the author's original paths):

| Variable | Used by |
|---|---|
| `ORB_SLAM3_ROOT` | all shell scripts (default `$HOME/ORB_SLAM3`) |
| `LK_MM_MATCH_TH` | run scripts (default `200`; the value used in the paper) |
| `TUM_RESULTS_ROOT`, `KITTI_RESULTS_ROOT` | `pvalues_v10.py` |
| `EUROC_RESULTS_ROOT`, `EUROC_GT_ROOT` | `eval_euroc_archival.py`, `eval_euroc_corrected.py` |
| `TUM_V2_ROOT`, `GT_DESK_PATH` | `diag_fr1_desk2.py` |
| `DATASET_SRC` | `datasets/install_datasets.sh` |

**Quick check** (no ORB-SLAM3 tree required):

```bash
bash "$REPO/run_demo.sh"
```

## 6. Methodology (reproducibility notes)

- **ATE metric**: ATE RMSE from `evo_ape` with SE(3) Umeyama alignment,
  translation part only (`evo_ape tum -a` for TUM, `evo_ape kitti -a` for
  KITTI). No scale correction is applied.
- **Repeated runs**: each (sequence, method) pair is run n = 3 times;
  failure-prone `fr1` sequences (`fr1_desk`, `fr1_desk2`, `fr1_360`,
  `fr1_floor`) are run n = 12 to characterise run-to-run variability. Runs are
  sequential on the same workstation, viewer disabled.
- **Statistics**: exact two-sided paired permutation tests, statistic
  |mean(v - b)| over 2^n sign flips (1e-12 tolerance);
  `eval/pvalues_v10.py` (TUM/KITTI) and `eval/eval_euroc_archival.py` (EuRoC).
  Statistical neutrality (ns) is reported for p >= 0.05.
- **Naive arm (v9)**: the naive LK motion prior (Section 3.2 ablation) raw
  data are archived in `supplementary/v9_naive_arm/` (legacy S0-S4, KITTI 00/05
  evo summaries, TUM tau-sweep). The manuscript reports KITTI 00 (tau = 0.40)
  +15.7% (n = 3, ns) and the KITTI 05 tau sweep +5.6% (tau = 0.10) to +30.3%
  (tau = 1.00).
- **Baseline vs method**: the baseline is the unmodified ORB-SLAM3 code in the
  same tree (restored from `code_backups_tum/*.baseline` by the run scripts).

## 7. Results (summary)

Raw per-run data supporting every table are archived in this repository:

| Data | Location |
|---|---|
| S0 dataset overview, S1 per-run ATE (+pooled), S2 LK decision stats, S3 failure events | `supplementary/S0..S3` |
| S4 reproduction guide, S5 patch | `supplementary/S4`, `supplementary/S5_lk_v2.patch`, `patch/S5_lk_v2.patch` |
| EuRoC per-run evo outputs + per-frame errors | `eval/archive_euroc_v10.2/` |
| v9 naive-LK ablation arm (Section 3.2) | `supplementary/v9_naive_arm/` |
| Per-platform summaries (sample expected outputs) | `results/` |

Headline numbers are listed in section 1; full tables are in the manuscript.

## 8. Citation

If you use this code or data in your research, please cite the paper:

```bibtex
@article{lk2026verified,
  author  = {L., Botong},   % TODO: final author list
  title   = {Verified competing-hypothesis motion prediction with Lucas--Kanade optical flow for robust tracking in ORB-SLAM3: a repeated-run study},
  journal = {PeerJ Computer Science},
  year    = {2026}
}
```

## 9. License

This repository is licensed under the **GNU General Public License v3.0** (see
`LICENSE`), consistent with the GPLv3 license of ORB-SLAM3, which this code
modifies. The benchmark datasets are not redistributed here; if you download
them, respect their original licenses (see the official pages in section 3).

## 10. Contact

Please open a GitHub issue for questions, bug reports, or reproduction
difficulties.
