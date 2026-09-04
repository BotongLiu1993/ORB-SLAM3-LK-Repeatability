#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_fig2_v10.py - Figure 2 (v10): fr1_desk & fr1_desk2 per-run ATE (n=12),
baseline (ORB-SLAM3) vs LK-v2 (verified competing motion prior).

- 自动 pip installing缺失包 (matplotlib / numpy)
- 自动定位数据: D:\\0 科研学习\\SLAM\\result\\evo_results\\tum_experiments\\v2_tum
  (找不到时回退到本地 results/v2_tum)
- 双面板图: fr1/desk (左) + fr1/desk2 (右), 箱线 + 散点 + 配对连线
- 统计: 两尾配对置换检验 (20000 次, 固定随机种子)

用法 (Windows 本地, miniconda):
    python make_fig2_v10.py
输出: fig2_v10_tum.png (保存到脚本所在目录的 figures/)
"""
import os
import re
import sys
import random
import shutil
import subprocess
import statistics


def ensure(pkg):
    try:
        __import__(pkg)
        return True
    except ImportError:
        print("[pip] installing %s ..." % pkg, flush=True)
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", pkg])
        return False


ensure("numpy")
ensure("matplotlib")

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT_CANDIDATES = [
    r"D:\0 科研学习\SLAM\result\evo_results\tum_experiments\v2_tum",
    os.path.join(os.getcwd(), "results", "v2_tum"),
    r"C:\Users\Administrator\Documents\Codex\2026-08-12\wo\v10_work\results\v2_tum",
]
RUNS = 12
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
OUT = os.path.join(OUT_DIR, "fig2_v10_tum.png")


def find_root():
    for cand in ROOT_CANDIDATES:
        if os.path.isdir(os.path.join(cand, "v2")):
            return cand
    print("[ERROR] not found v2_tum 结果目录, check:")
    for cand in ROOT_CANDIDATES:
        print("   ", cand)
    sys.exit(1)


def read_rmse(evotxt):
    try:
        with open(evotxt, encoding="utf-8", errors="ignore") as f:
            for line in f:
                p = line.split()
                if len(p) == 2 and p[0] == "rmse":
                    return float(p[1])
    except OSError:
        pass
    return None


def load_seq(root, seq, tag):
    out = []
    for r in range(1, RUNS + 1):
        v = read_rmse(os.path.join(root, tag, "%s_r%d_evo.txt" % (seq, r)))
        if v is not None:
            out.append(v)
    return out


def perm_paired(a, b, n_perm=20000, seed=7):
    """Two-sided paired permutation test on the mean difference.

    Exact enumeration when n <= 16 (2**n sign combinations, deterministic),
    otherwise Monte Carlo with a fixed seed.
    """
    n = len(a)
    diff = statistics.mean(a) - statistics.mean(b)
    d = [a[i] - b[i] for i in range(n)]
    if n <= 16:
        cnt = 0
        for mask in range(1 << n):
            s = 0.0
            for i in range(n):
                s += d[i] if (mask >> i) & 1 else -d[i]
            if abs(s / n) >= abs(diff):
                cnt += 1
        return cnt / (1 << n)
    rng = random.Random(seed)
    cnt = 0
    for _ in range(n_perm):
        s = sum((1 if rng.random() < 0.5 else -1) * d[i] for i in range(n)) / n
        if abs(s) >= abs(diff):
            cnt += 1
    return (cnt + 1) / (n_perm + 1)


def main():
    root = find_root()
    print("data root:", root)
    print("data root:", root, file=sys.stderr)

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.8), sharey=True)
    for ax, seq, title in zip(
        axes,
        ("fr1_desk", "fr1_desk2"),
        ("fr1/desk (fast motion)", "fr1/desk2 (fast motion)"),
    ):
        base = load_seq(root, seq, "baseline")
        v2 = load_seq(root, seq, "v2")
        if len(base) != RUNS or len(v2) != RUNS:
            print("[warn] %s incomplete runs: baseline=%d, v2=%d" % (seq, len(base), len(v2)))

        p = perm_paired(base, v2)
        dmean = (statistics.mean(v2) - statistics.mean(base)) / statistics.mean(base) * 100.0
        sbase = statistics.stdev(base) if len(base) > 1 else 0.0
        sv2 = statistics.stdev(v2) if len(v2) > 1 else 0.0

        print("")
        print("== %s (n=%d) ==" % (seq, len(base)))
        print("  baseline: mean %.4f  std %.4f  range [%.4f, %.4f]" % (
            statistics.mean(base), sbase, min(base), max(base)))
        print("  v2:       mean %.4f  std %.4f  range [%.4f, %.4f]" % (
            statistics.mean(v2), sv2, min(v2), max(v2)))
        print("  dMean %+.1f%%   p(two-sided perm) = %.4f" % (dmean, p))

        data = [base, v2]
        bp = ax.boxplot(data, positions=[0, 1], widths=0.44, patch_artist=True,
                        showfliers=False, medianprops=dict(color="k", lw=1.4),
                        boxprops=dict(linewidth=1.1), whiskerprops=dict(linewidth=1.1),
                        capprops=dict(linewidth=1.1))
        for patch, col in zip(bp["boxes"], ["#9ecae1", "#a1d99b"]):
            patch.set_facecolor(col)
            patch.set_alpha(0.85)

        rng = np.random.default_rng(1)
        for xi, vals, col in [(0, base, "#3182bd"), (1, v2, "#31a354")]:
            jit = rng.normal(0, 0.05, len(vals))
            ax.scatter(np.full(len(vals), xi) + jit, vals, s=26, color=col,
                       edgecolor="white", linewidth=0.6, zorder=3)
        for a, b in zip(base, v2):
            ax.plot([0, 1], [a, b], color="0.75", lw=0.5, zorder=1, alpha=0.7)

        ax.set_xticks([0, 1])
        ax.set_xticklabels(["Baseline", "LK-v2"], fontsize=11)
        ax.set_title(title, fontsize=12)
        if ax is axes[0]:
            ax.set_ylabel("ATE RMSE (m)", fontsize=11)
        ax.grid(axis="y", ls="--", alpha=0.4)
        ax.set_axisbelow(True)

        star = "***" if p < 0.001 else ("**" if p < 0.01 else ("*" if p < 0.05 else "ns"))
        ax.text(0.5, 0.97, "%+.1f%%  (p = %.3f, %s)" % (dmean, p, star),
                transform=ax.transAxes, ha="center", va="top", fontsize=10.5,
                color="#d62728" if p < 0.05 else "0.25")

    fig.suptitle("Per-run ATE (n = 12): baseline vs LK-v2 (verified competing motion prior)",
                 fontsize=12.5, y=0.99)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs(OUT_DIR, exist_ok=True)
    fig.savefig(OUT, dpi=300, bbox_inches="tight")
    print("")
    print("[ok] saved:", OUT)


if __name__ == "__main__":
    main()