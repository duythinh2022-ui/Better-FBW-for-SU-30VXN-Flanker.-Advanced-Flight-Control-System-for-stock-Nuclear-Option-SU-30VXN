import sys; sys.path.insert(0,"/home/claude/su30/src/tools")
from sweep import *
lims=[int(x) for x in sys.argv[1].split(",")]; iass=[int(x) for x in sys.argv[2].split(",")]; extra=sys.argv[3:]
roll = any(e.startswith("ROLL") for e in extra); extra=[e for e in extra if not e.startswith("ROLL")]
jobs=[]
for L in lims:
    for ias in iass:
        a=[f"ias={ias}","alt=3000","thr=1","T=9","pitch=0:0,0.3:1,9:1",f"AoAPos={L}","NoTvcAoACapPos=65","NoTvcThrCapPos=65"]+extra
        if roll: a.append("roll=0:0,4:0,4.2:1,9:1")
        jobs.append((f"L{L}_v{ias}{'_r' if roll else ''}",a))
with cf.ThreadPoolExecutor(2) as ex:
    res=list(ex.map(lambda j: run(*j), jobs))
print(f"{'case':14s} {'aMean':>6} {'aMin':>6} {'aMax':>6} {'aPk':>6} {'bMax':>5} {'|p|':>5} {'cap':>5} {'cnL':>6} {'elL':>6} {'can':>5} {'el':>5} {'ias':>5} {'nz':>5}")
for tag,f in res:
    s=summ(f,6,9) if not roll else summ(f,5,9)
    if s: print(f"{tag:14s} {s['aMean']:6.1f} {s['aMin']:6.1f} {s['aMax']:6.1f} {s['aPk']:6.1f} {s['bMax']:5.1f} {s['pMean']:5.0f} {s['cap']:5.1f} {s['cnL']:6.1f} {s['elL']:6.1f} {s['can']:5.1f} {s['el']:5.1f} {s['ias']*3.6:5.0f} {s['nz']:5.1f}")
