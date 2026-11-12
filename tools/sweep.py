import subprocess, sys, os, itertools, pandas as pd, concurrent.futures as cf
B=os.environ.get("BSRC","/home/claude/su30/src"); OUT="/home/claude/su30/out/sw"; os.makedirs(OUT,exist_ok=True)
def run(tag, args):
    f=f"{OUT}/{tag}.csv"
    subprocess.run(["bash",f"{B}/bench.sh","fly"]+args+[f"fcslog={f}"],capture_output=True,cwd=B)
    return tag,f
def summ(f,t0,t1):
    d=pd.read_csv(f); w=d[(d.t>=t0)&(d.t<=t1)]
    if len(w)==0: return None
    can=max(d.canard_L_now.abs().max(),d.canard_R_now.abs().max()); el=max(d.elevator_L_now.abs().max(),d.elevator_R_now.abs().max())
    return dict(aMean=w.alpha.mean(),aMin=w.alpha.min(),aMax=w.alpha.max(),aPk=d.alpha.max(),aMinAll=d.alpha.min(),bMax=d.beta.abs().max(),pMean=w.p.abs().mean(),cap=w.authCapEff.min() if 'authCapEff' in w else -1,
        cnL=w.canard_L_now.mean(),elL=w.elevator_L_now.mean(),can=can,el=el,ias=w.ias.mean(),nz=d.nz.max(),q=w.q.mean())
if __name__=="__main__":
    pass
