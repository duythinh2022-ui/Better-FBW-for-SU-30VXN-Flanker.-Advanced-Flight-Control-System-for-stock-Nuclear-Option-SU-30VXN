import sys; sys.path.insert(0,"/home/claude/su30/src/tools")
from sweep import *
import numpy as np
# usage: hold.py lims iass [ROLL] [Name=value ...]
lims=[int(x) for x in sys.argv[1].split(",")]; iass=[int(x) for x in sys.argv[2].split(",")]; extra=sys.argv[3:]
roll=("ROLL" in extra); push=("PUSH" in extra); extra=[e for e in extra if e not in("ROLL","PUSH")]
tag0="_".join(e.replace("=","") for e in extra)[:40]
jobs=[]
for L in lims:
    for ias in iass:
        st="-1" if push else "1"
        a=[f"ias={ias}","alt=3000","thr=1","T=9",f"pitch=0:0,0.3:{st},9:{st}",f"AoAPos={L}",f"AoANeg=-{L}","NoTvcAoACapPos=65","NoTvcThrCapPos=65","NoTvcAoACapNeg=45","NoTvcThrCapNeg=45"]+extra
        if roll: a.append("roll=0:0,4:0,4.2:1,7:1,7.2:-1,9:-1")
        jobs.append((f"{'P' if push else 'L'}{L}_v{ias}{'_r' if roll else ''}_{tag0}",a))
with cf.ThreadPoolExecutor(2) as ex: res=list(ex.map(lambda j: run(*j), jobs))
print(f"{'case':28s} {'aPk':>6} {'err':>5} {'rms':>5} {'aRng':>11} {'bMax':>5} {'|p|':>5} {'canR':>5} {'elR':>5} {'canMax':>6} {'elMax':>6} {'iasEnd':>6} {'nz':>5}")
for tag,f in res:
    d=pd.read_csv(f); sg=-1 if push else 1
    w=d[d.t>=3.0]
    lim=np.minimum(w.authCapEff, abs(int(tag.split('_')[0][1:]))) if not push else -np.minimum(-w.authCapNegEff if 'authCapNegEff' in w else 99, abs(int(tag.split('_')[0][1:])))
    if push: lim=-np.full(len(w),abs(int(tag.split('_')[0][1:])))
    e=w.alpha-lim
    dt=np.diff(w.t.values)
    cr=np.sqrt(((np.diff(w.canard_L_now.values)/dt)**2).mean()); er=np.sqrt(((np.diff(w.elevator_L_now.values)/dt)**2).mean())
    print(f"{tag:28s} {(d.alpha.max() if not push else d.alpha.min()):6.1f} {e.mean():5.2f} {np.sqrt((e**2).mean()):5.2f} {w.alpha.min():5.1f}..{w.alpha.max():5.1f} {d.beta.abs().max():5.1f} {w.p.abs().mean():5.0f} {cr:5.0f} {er:5.0f} {d.canard_L_now.abs().max():6.1f} {d.elevator_L_now.abs().max():6.1f} {d.ias.iloc[-1]*3.6:6.0f} {(d.nz.max() if not push else d.nz.min()):5.1f}")
