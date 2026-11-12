import sys, pandas as pd
d=pd.read_csv(sys.argv[1]); step=float(sys.argv[2]) if len(sys.argv)>2 else 0.5
cols=["t","ias","alpha","beta","nz","p","q","r","phi","tw"]
S=["canard_L","canard_R","elevator_L","elevator_R","flaperon_L","flaperon_R","rollervon_L","rollervon_R","rudder_L","rudder_R"]
ab={"canard_L":"cnL","canard_R":"cnR","elevator_L":"elL","elevator_R":"elR","flaperon_L":"fpL","flaperon_R":"fpR","rollervon_L":"rvL","rollervon_R":"rvR","rudder_L":"ruL","rudder_R":"ruR"}
hdr=" ".join(f"{c:>6}" for c in cols)+" | "+" ".join(f"{ab[s]:>5}" for s in S)
print(hdr); nxt=0
for _,r in d.iterrows():
    if r.t+1e-6<nxt: continue
    nxt=r.t+step
    print(" ".join(f"{r[c]:6.1f}" if c!="t" else f"{r[c]:6.2f}" for c in cols)+" | "+" ".join(f"{r[s+'_now']:5.1f}" for s in S))
print("max|beta| %.1f  alpha %.1f..%.1f  nz %.2f..%.2f  |p|max %.0f"%(d.beta.abs().max(),d.alpha.min(),d.alpha.max(),d.nz.min(),d.nz.max(),d.p.abs().max()))
