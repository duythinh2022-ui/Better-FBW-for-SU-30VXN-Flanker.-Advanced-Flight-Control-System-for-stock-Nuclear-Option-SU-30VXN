import sys; sys.path.insert(0,"/home/claude/su30/src/tools")
from sweep import *
import numpy as np
# rollchat.py tag Name=value ... : loaded rolls at 35/40/45 x 250/450, steady-roll surface activity
extra=sys.argv[2:]; tag=sys.argv[1]
jobs=[]
for L in (35,40,45):
    for ias in (250,450):
        a=[f"ias={ias}","alt=3000","thr=1","T=7.5","pitch=0:0,0.3:1,9:1",f"AoAPos={L}","NoTvcAoACapPos=65","NoTvcThrCapPos=65","roll=0:0,4:0,4.2:1,9:1"]+extra
        jobs.append((f"rc_{tag}_L{L}_v{ias}",a))
with cf.ThreadPoolExecutor(2) as ex: res=list(ex.map(lambda j: run(*j), jobs))
tot=0
for t,f in res:
    d=pd.read_csv(f); w=d[(d.t>5.0)&(d.t<7.4)]; dt=np.diff(w.t.values)
    act={}
    for s in ["canard_L","canard_R","elevator_L","elevator_R","flaperon_L","rollervon_L","rudder_L"]:
        r=np.diff(w[s+"_now"].values)/dt; act[s]=np.sqrt((r**2).mean())
    pV=(w.p*np.cos(np.radians(w.alpha))+w.r*np.sin(np.radians(w.alpha))).mean()
    tot+=act["canard_L"]+act["canard_R"]+act["elevator_L"]
    print(f"{t:32s} can {act['canard_L']:4.0f}/{act['canard_R']:4.0f} el {act['elevator_L']:4.0f} fp {act['flaperon_L']:4.0f} rv {act['rollervon_L']:4.0f} ru {act['rudder_L']:4.0f}  pV {pV:5.1f} a {w.alpha.mean():5.1f}±{w.alpha.std():.2f} bMax {d[d.t>4].beta.abs().max():4.1f}")
print("TOTAL pitch-surface activity", round(tot))
