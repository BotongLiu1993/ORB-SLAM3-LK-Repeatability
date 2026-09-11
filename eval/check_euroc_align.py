# -*- coding: utf-8 -*-
import sys, os, zipfile, re
import numpy as np
res_dir = sys.argv[1]; zips_dir = sys.argv[2]
SEQS = {"V101":"V1_01_easy.zip","V102":"V1_02_medium.zip","V103":"V1_03_difficult.zip"}
def load_gt(seq):
    with zipfile.ZipFile(os.path.join(zips_dir, SEQS[seq])) as z:
        data = z.read("mav0/state_groundtruth_estimate0/data.csv").decode("utf-8")
    ts=[]; pos=[]
    for line in data.splitlines():
        line=line.strip()
        if not line or line.startswith("#"): continue
        p=line.split(",")
        ts.append(float(p[0])/1e9); pos.append([float(p[1]),float(p[2]),float(p[3])])
    o=np.argsort(ts); ts=np.array(ts)[o]; pos=np.array(pos)[o]
    return ts,pos
for seq,zf in SEQS.items():
    gt_ts,gt_pos = load_gt(seq)
    print("==",seq,"GT rows:",len(gt_ts))
    for meth in ("baseline","v2"):
        for r in (1,2,3):
            tp=os.path.join(res_dir,meth,"euroc%s_r%d_traj.txt"%(seq,r))
            est=[]
            for line in open(tp):
                p=line.split()
                est.append([float(p[0])/1e9, float(p[1]),float(p[2]),float(p[3])])
            est=np.array(est)
            off=gt_ts[0]-est[0,0]
            st=est[:,0]+off
            idx=np.searchsorted(gt_ts,st)
            idx=np.clip(idx,0,len(gt_ts)-1)
            near=np.minimum(np.abs(gt_ts[idx]-st), np.abs(gt_ts[np.maximum(idx-1,0)]-st))
            if meth=="baseline" and r==1:
                # Umeyama scale check on matched positions
                def umeyama(src,dst):
                    mu_s=src.mean(0); mu_d=dst.mean(0); sc=src-mu_s; dc=dst-mu_d
                    cov=dc.T@sc/len(src)
                    U,S,Vt=np.linalg.svd(cov); d=np.sign(np.linalg.det(U@Vt)); R=U@np.diag([1,1,d])@Vt
                    s=np.trace(np.diag([1,1,d])@np.diag(S))/(sc**2).sum()*len(src)
                    return s,R,mu_d-s*R@mu_s
                g=np.array([gt_pos[max(i-1,0)] for i in idx])
                s,R,t=umeyama(est[:,1:],g)
                print("   %s r1: nearest-GT distance median=%.2f ms max=%.2f ms | Umeyama scale=%.4f | frames=%d"%(meth,np.median(near)*1e3,near.max()*1e3,s,len(est)))
            elif meth=="baseline" and r==2:
                pass
    # LK stats from v2 r1
    for r in (1,2,3):
        lp=os.path.join(res_dir,"v2","euroc%s_r%d_lkstats.txt"%(seq,r))
        if os.path.isfile(lp):
            txt=open(lp,encoding="utf-8",errors="replace").read()
            m1=re.search(r"LK attempts:\s+(\d+) \(([\d.]+)%",txt)
            m2=re.search(r"LK pose used:\s+(\d+) \(([\d.]+)%",txt)
            m3=re.search(r"LK match margin:\s+mean ([\d.]+) med (\d+)",txt)
            m4=re.search(r"LK mean FB error:\s+([\d.]+)",txt)
            print("   %s lkstats r%d: attempts=%s%% used=%s%% margin_mean=%s FB=%.4f"%(seq,r,m1.group(2) if m1 else "?",m2.group(2) if m2 else "?",m3.group(1) if m3 else "?",float(m4.group(1)) if m4 else -1))