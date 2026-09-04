#!/usr/bin/env bash
# =============================================================================
# diag_fr1_desk2.sh - fr1_desk vs fr1_desk2 速度剖面 + LK 决策统计对照
# 用法 (WSL, ~/ORB_SLAM3 下):
#   bash "$W/diag_fr1_desk2.sh"
# 输出:
#   - 平移速度 / 旋转速率 的 median / p95 / max (来自 groundtruth)
#   - LK 触发/采纳/margin/FB 汇总 (来自 results/v2_tum/v2 或 /mnt/d 已复制结果)
# =============================================================================
set -uo pipefail
ORB="${ORB_SLAM3_ROOT:-$HOME/ORB_SLAM3}"
cd "$ORB" || exit 1

python3 - "$ORB" <<'PYEOF'
import sys, os, math, re, statistics
orb = sys.argv[1]

def quat_rot_rate(q1, q2, dt):
    # 单位四元数差 -> 旋转角 (rad) -> deg/s
    w1,x1,y1,z1 = q1; w2,x2,y2,z2 = q2
    w = w1*w2 + x1*x2 + y1*y2 + z1*z2
    w = max(-1.0, min(1.0, w))
    return math.degrees(2.0*math.acos(abs(w)))/dt if dt > 0 else 0.0

def profile(datadir):
    gt = os.path.join(datadir, "groundtruth.txt")
    if not os.path.isfile(gt):
        print(f"  [warn] 缺 {gt}")
        return None
    rows = []
    with open(gt) as f:
        for line in f:
            p = line.split()
            if len(p) < 8: continue
            t = float(p[0])
            xyz = tuple(float(v) for v in p[1:4])
            q = tuple(float(v) for v in p[4:8])
            rows.append((t, xyz, q))
    if len(rows) < 2: return None
    vs, rs = [], []
    for i in range(1, len(rows)):
        dt = rows[i][0] - rows[i-1][0]
        if dt <= 0: continue
        dx = math.sqrt(sum((a-b)**2 for a,b in zip(rows[i][1], rows[i-1][1])))
        vs.append(dx/dt)
        rs.append(quat_rot_rate(rows[i-1][2], rows[i][2], dt))
    return vs, rs

def stats(x):
    x = sorted(x)
    n = len(x)
    def q(p):
        k = (n-1)*p; lo = int(k); hi = min(lo+1, n-1)
        return x[lo] + (x[hi]-x[lo])*(k-lo)
    return q(0.5), q(0.95), x[-1]

seqs = [("fr1_desk",  "datasets/rgbd_dataset_freiburg1_desk"),
        ("fr1_desk2", "datasets/rgbd_dataset_freiburg1_desk2")]
print("== 速度剖面 (groundtruth) ==")
print(f"{'seq':<12}{'frames':>8}{'dur_s':>8}{'v_med':>9}{'v_p95':>9}{'v_max':>9}{'rot_med':>10}{'rot_p95':>10}{'rot_max':>10}")
for name, d in seqs:
    pr = profile(os.path.join(orb, d))
    if not pr: continue
    vs, rs = pr
    vm, vp, vx = stats(vs); rm, rp, rx = stats(rs)
    ts = []
    with open(os.path.join(orb, d, "groundtruth.txt")) as f:
        for l in f:
            if len(l.split()) >= 8: ts.append(float(l.split()[0]))
    dur = ts[-1]-ts[0] if len(ts) >= 2 else 0.0
    print(f"{name:<12}{len(vs):>8}{dur:>8.1f}{vm:>9.2f}{vp:>9.2f}{vx:>9.2f}{rm:>10.1f}{rp:>10.1f}{rx:>10.1f}")

print()
print("== LK 决策统计 (v2 侧, 12 轮均值) ==")
resdirs = [os.path.join(orb, "results/v2_tum/v2"),
           "/mnt/d/0 科研学习/SLAM/result/evo_results/tum_experiments/v2_tum/v2"]
resdir = next((d for d in resdirs if os.path.isdir(d)), None)
if not resdir:
    print("  [warn] 未找到 v2_tum 结果目录, 跳过")
    sys.exit(0)
for name in ("fr1_desk", "fr1_desk2"):
    att, adp, margin, fb, frames = [], [], [], [], []
    for r in range(1, 13):
        p = os.path.join(resdir, f"{name}_r{r}_lkstats.txt")
        if not os.path.isfile(p): continue
        t = open(p).read()
        def g(pat):
            m = re.search(pat, t)
            return float(m.group(1)) if m else float('nan')
        att.append(g(r"LK attempts:\s+(\d+)"))
        adp.append(g(r"LK pose used:\s+(\d+)"))
        margin.append(g(r"LK match margin:\s+mean\s+([\d.]+)"))
        fb.append(g(r"LK mean FB error:\s+([\d.]+)"))
        frames.append(g(r"Total frames:\s+(\d+)"))
    n = len(att)
    if n == 0: continue
    def m(x): return statistics.mean(x) if x else float('nan')
    f = m(frames)
    print(f"{name:<12} att% {100*m(att)/f:6.1f}  adopt(帧%) {100*m(adp)/f:6.1f}  margin {m(margin):6.1f}  FB {m(fb):.3f} px  (n={n})")
PYEOF