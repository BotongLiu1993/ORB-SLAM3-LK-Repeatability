# -*- coding: utf-8 -*-
"""
pvalues_v10.py - exact two-sided paired permutation tests (single source of truth for every p-value in the paper)
Statistic: absolute paired mean difference |mean(v_i - b_i)|; all 2^n sign combinations enumerated exactly.
TUM/KITTI read rmse from the evo result files; for EuRoC see eval_euroc_corrected.py.
"""
import io, os, itertools
import numpy as np

TUM = os.environ.get("TUM_RESULTS_ROOT", r"D:\SLAM\result\evo_results\tum_experiments\v2_tum")
KITTI = os.environ.get("KITTI_RESULTS_ROOT", r"D:\SLAM\result\evo_results\v2_kitti")

TUM_SEQS = {
    "fr1_desk": 12, "fr1_desk2": 12, "fr1_room": 3, "fr1_360": 12, "fr1_floor": 12,
    "fr2_xyz": 3, "fr2_desk": 3, "fr3_sitting": 3, "fr3_office": 3,
}
KITTI_SEQS = {s: 3 for s in ["00", "02", "05", "07", "09", "10"]}

def read_rmse(evo_txt):
    with io.open(evo_txt, encoding="utf-8") as f:
        for line in f:
            if line.strip().startswith("rmse"):
                return float(line.split()[1])
    return None

def perm_p(b, v):
    d = np.array([v[i] - b[i] for i in range(len(b))])
    d_obs = abs(d.mean())
    n = len(d)
    cnt = 0
    for s in itertools.product([-1.0, 1.0], repeat=n):
        if abs(np.mean(np.array(s) * d)) >= d_obs - 1e-12:
            cnt += 1
    return cnt / 2 ** n

def collect(base_dir, seq, n, prefix=""):
    b, v = [], []
    for r in range(1, n + 1):
        rb = read_rmse(os.path.join(base_dir, "baseline", f"{prefix}{seq}_r{r}_evo.txt"))
        rv = read_rmse(os.path.join(base_dir, "v2", f"{prefix}{seq}_r{r}_evo.txt"))
        if rb is None or rv is None:
            return None
        b.append(rb); v.append(rv)
    return b, v

def run(tag, base_dir, seqs, prefix=""):
    print(f"=== {tag} ===")
    for seq, n in seqs.items():
        res = collect(base_dir, seq, n, prefix)
        if res is None:
            print(f"{seq}: missing files"); continue
        b, v = res
        p = perm_p(b, v)
        d = (np.mean(v) - np.mean(b)) / np.mean(b) * 100
        print(f"{seq:11s} n={n:2d} base={np.mean(b):.4f} v2={np.mean(v):.4f} d={d:+6.1f}% p={p:.4f}")

run("TUM v2 (evo_ape tum -a)", TUM, TUM_SEQS)
run("KITTI v2 (evo_ape kitti -a)", KITTI, KITTI_SEQS, prefix="kitti")

print("\n=== fr1_desk detail ===")
b, v = collect(TUM, "fr1_desk", 12)
print("baseline:", [f"{x:.4f}" for x in b])
print("v2      :", [f"{x:.4f}" for x in v])
print("p =", perm_p(b, v))
