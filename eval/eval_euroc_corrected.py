# -*- coding: utf-8 -*-
"""
EuRoC corrected evaluation (v2 data, official GT, direct timestamp matching).
Archival script: computes ATE (SE(3) Umeyama, no scale) + permutation p-values.
Usage: python eval_euroc_corrected.py
"""
import io, os, itertools
import numpy as np
from scipy.spatial.transform import Rotation as R

BASE = os.environ.get("EUROC_RESULTS_ROOT", r"D:\0 科研学习\SLAM\result\evo_results\v2_euroc")
GT = os.environ.get("EUROC_GT_ROOT", r"D:\0 科研学习\SLAM\ORB_SLAM3_wsl_snapshot\evaluation\Ground_truth\EuRoC_left_cam")
SEQS = ["V101", "V102", "V103"]
RUNS = 3

def load_gt_csv(p):
    ts, pos = [], []
    with io.open(p, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            c = line.split(",")
            ts.append(float(c[0]) / 1e9)
            pos.append([float(c[1]), float(c[2]), float(c[3])])
    ts = np.array(ts); pos = np.array(pos)
    order = np.argsort(ts); return ts[order], pos[order]

def load_traj(p):
    ts, pos = [], []
    with io.open(p, encoding="utf-8") as f:
        for line in f:
            c = line.split()
            if len(c) < 4:
                continue
            ts.append(float(c[0]) / 1e9)
            pos.append([float(c[1]), float(c[2]), float(c[3])])
    ts = np.array(ts); pos = np.array(pos)
    order = np.argsort(ts); return ts[order], pos[order]

def path_len(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))

def ate_rmse(t_est, p_est, t_gt, p_gt, tol=0.03):
    idx = np.clip(np.searchsorted(t_gt, t_est), 0, len(t_gt) - 1)
    v = np.abs(t_gt[idx] - t_est) <= tol
    if v.sum() < 10:
        return np.nan, 0
    e, g = p_est[v], p_gt[idx][v]
    ce, cg = e.mean(0), g.mean(0)
    rot = R.align_vectors(g - cg, e - ce)[0]
    err = g - cg - rot.apply(e - ce)
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))), int(v.sum())

results = {}
for seq in SEQS:
    gt_path = os.path.join(GT, f"{seq}_GT.txt")
    t_gt, p_gt = load_gt_csv(gt_path)
    gt_len = path_len(p_gt)
    for meth in ["baseline", "v2"]:
        for r in range(1, RUNS + 1):
            p = os.path.join(BASE, meth, f"euroc{seq}_r{r}_traj.txt")
            t_e, p_e = load_traj(p)
            rmse, n = ate_rmse(t_e, p_e, t_gt, p_gt)
            results[(seq, meth, r)] = (rmse, n, path_len(p_e))

print(f"{'seq':5} {'meth':8} {'r':>2} {'ATE':>8} {'n':>6} {'est/GT scale':>12}")
for seq in SEQS:
    gt_len = path_len(load_gt_csv(os.path.join(GT, f"{seq}_GT.txt"))[1])
    for meth in ["baseline", "v2"]:
        for r in range(1, RUNS + 1):
            rmse, n, elen = results[(seq, meth, r)]
            print(f"{seq:5} {meth:8} {r:>2} {rmse:8.4f} {n:>6} {elen/gt_len:12.3f}")

print("\n--- pooled stats + permutation p (two-sided, mean-diff statistic) ---")
for seq in SEQS:
    b = [results[(seq, "baseline", r)][0] for r in range(1, RUNS + 1)]
    v = [results[(seq, "v2", r)][0] for r in range(1, RUNS + 1)]
    d_obs = abs(np.mean([v[i] - b[i] for i in range(len(b))]))
    signs = list(itertools.product([-1, 1], repeat=len(b)))
    cnt = 0; tot = 0
    for s in signs:
        d = np.mean([s[i] * (v[i] - b[i]) for i in range(len(b))])
        tot += 1
        if abs(d) >= abs(d_obs):
            cnt += 1
    p = cnt / tot
    print(f"{seq}: base={np.mean(b):.4f} v2={np.mean(v):.4f} d={d_obs*100:+.2f}% p={p:.3f} (n={len(b)})")
