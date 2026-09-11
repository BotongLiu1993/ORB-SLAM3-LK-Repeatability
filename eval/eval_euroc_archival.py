# -*- coding: utf-8 -*-
"""
EuRoC v10.2 archival evaluation (official GT, direct timestamp matching).
Reproduces the numbers reported in v10.2 manuscript Tables 2/3:
  V101: base 0.0438 -> v2 0.0359 (-18.1%, p=0.500)
  V102: base 0.0385 -> v2 0.0401 (+4.3%,  p=0.750)
  V103: base 0.1648 -> v2 0.0936 (-43.2%, p=0.250)
Usage: python eval_euroc_archival.py
Outputs (this script's directory):
  evo_outputs/{SEQ}_{method}_r{R}.txt   evo-style per-run stats
  evo_outputs/{SEQ}_{method}_r{R}_errors.csv  per-frame APE
  summary_per_run.csv                    all runs + scale ratios
  summary_pooled.txt                    pooled means + permutation p
"""
import io, os, itertools, csv
import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "archive_euroc_v10.2", "evo_outputs")
BASE = os.environ.get("EUROC_RESULTS_ROOT", r"D:\SLAM\result\evo_results\v2_euroc")
GT = os.environ.get("EUROC_GT_ROOT", r"D:\SLAM\ORB_SLAM3_wsl_snapshot\evaluation\Ground_truth\EuRoC_left_cam")
SEQS = ["V101", "V102", "V103"]
RUNS = 3
TOL = 0.03  # seconds, direct timestamp matching

def load_gt_csv(p):
    ts, pos = [], []
    with io.open(p, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            c = line.split(",")
            if len(c) < 4:
                continue
            ts.append(float(c[0]) / 1e9)
            pos.append([float(c[1]), float(c[2]), float(c[3])])
    ts = np.array(ts); pos = np.array(pos)
    order = np.argsort(ts)
    return ts[order], pos[order]

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
    order = np.argsort(ts)
    return ts[order], pos[order]

def path_len(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))

def align_ate(p_est, p_gt):
    """SE(3) Umeyama alignment (no scale), returns per-point errors (m)."""
    ce, cg = p_est.mean(0), p_gt.mean(0)
    rot = R.align_vectors(p_gt - cg, p_est - ce)[0]
    err = p_gt - cg - rot.apply(p_est - ce)
    return np.linalg.norm(err, axis=1)

def evo_stats(errs):
    sse = float(np.sum(errs ** 2))
    n = len(errs)
    return {
        "max": float(errs.max()), "mean": float(errs.mean()),
        "median": float(np.median(errs)), "min": float(errs.min()),
        "rmse": float(np.sqrt(sse / n)), "sse": sse, "std": float(errs.std()),
        "n": n,
    }

os.makedirs(OUT, exist_ok=True)
rows = []
for seq in SEQS:
    t_gt, p_gt = load_gt_csv(os.path.join(GT, "%s_GT.txt" % seq))
    gt_len = path_len(p_gt)
    for meth in ["baseline", "v2"]:
        for r in range(1, RUNS + 1):
            t_e, p_e = load_traj(os.path.join(BASE, meth, "euroc%s_r%d_traj.txt" % (seq, r)))
            idx = np.clip(np.searchsorted(t_gt, t_e), 0, len(t_gt) - 1)
            ok = np.abs(t_gt[idx] - t_e) <= TOL
            if ok.sum() < 10:
                raise RuntimeError("%s %s r%d: too few matched frames" % (seq, meth, r))
            errs = align_ate(p_e[ok], p_gt[idx][ok])
            st = evo_stats(errs)
            scale = path_len(p_e) / gt_len
            rows.append({"seq": seq, "method": meth, "run": r, "matched": int(ok.sum()),
                         "est_len": path_len(p_e), "gt_len": gt_len, "scale": scale, **st})
            # evo-style stats file
            name = "%s_%s_r%d" % (seq, meth, r)
            with io.open(os.path.join(OUT, name + ".txt"), "w", encoding="utf-8", newline="\n") as f:
                f.write("APE w.r.t. translation part (m)\n(with SE(3) Umeyama alignment)\n\n")
                f.write("       max      %0.6f\n      mean      %0.6f\n    median      %0.6f\n       min      %0.6f\n      rmse      %0.6f\n       sse      %0.6f\n       std      %0.6f\n\nmatched frames: %d (direct timestamp matching, tol %.2f s)\nest/GT path-length ratio: %.3f\n" % (
                    st["max"], st["mean"], st["median"], st["min"], st["rmse"], st["sse"], st["std"], ok.sum(), TOL, scale))
            # per-frame errors
            with io.open(os.path.join(OUT, name + "_errors.csv"), "w", encoding="utf-8", newline="\n") as f:
                w = csv.writer(f)
                w.writerow(["t_est", "t_gt", "ape_m"])
                for te, tg, e in zip(t_e[ok], t_gt[idx][ok], errs):
                    w.writerow(["%.6f" % te, "%.6f" % tg, "%.6f" % e])

with io.open(os.path.join(HERE, "archive_euroc_v10.2", "summary_per_run.csv"), "w", encoding="utf-8", newline="\n") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

pool = []
with io.open(os.path.join(HERE, "archive_euroc_v10.2", "summary_pooled.txt"), "w", encoding="utf-8", newline="\n") as f:
    f.write("pooled mean ATE RMSE (m) + two-sided paired permutation p (statistic = |mean(v-b)|, 2^n)\n\n")
    for seq in SEQS:
        b = [x["rmse"] for x in rows if x["seq"] == seq and x["method"] == "baseline"]
        v = [x["rmse"] for x in rows if x["seq"] == seq and x["method"] == "v2"]
        d_obs = abs(np.mean([v[i] - b[i] for i in range(len(b))]))
        cnt = tot = 0
        for s in itertools.product([-1, 1], repeat=len(b)):
            d = np.mean([s[i] * (v[i] - b[i]) for i in range(len(b))])
            tot += 1
            if abs(d) >= d_obs - 1e-12:
                cnt += 1
        p = cnt / tot
        d_signed = np.mean(v) - np.mean(b)
        rel = d_signed / np.mean(b) * 100
        line = "%s: base=%.4f v2=%.4f d=%.4f (%.1f%%) p=%.3f (n=%d)\n" % (seq, np.mean(b), np.mean(v), d_signed, rel, p, len(b))
        f.write(line)
        pool.append(line)
        print(line.strip())

print("archive written to:", os.path.join(HERE, "archive_euroc_v10.2"))
