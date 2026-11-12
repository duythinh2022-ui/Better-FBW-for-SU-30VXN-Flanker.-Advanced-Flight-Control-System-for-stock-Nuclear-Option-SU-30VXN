// native/verify/cplcheck.cs (Su-30 0.5.0): the coupled-cap solver (CplPoint) on the game's Mono. A hand case (expected 60 / 40),
// 2,001 random problems, and the one-side solve against the two-side solve. Build with csc against the game's managed folder
// like jetcheck.cs; run: host.exe <mono dll> <managed> <etc> -O=-float32 cplcheck.exe <path to Su30_VXN_FCS.dll>
using System; using System.Reflection;
static class C { static int Main(string[] a) {
  Assembly asm = Assembly.LoadFrom(a[0]); Type t = asm.GetType("Su30_VXN.FCS.CplPoint", true);
  BindingFlags BF = BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic;
  int bad = 0; var rnd = new Random(7); var sw = System.Diagnostics.Stopwatch.StartNew(); int n = 0; int one = 0, badOne = 0;
  for (int k = 0; k < 2001; k++) {
    object cp = Activator.CreateInstance(t, true); t.GetMethod("Reset", BF).Invoke(cp, null);
    MethodInfo add = t.GetMethod("Add", BF), end = t.GetMethod("EndEff", BF), res = t.GetMethod("Reserve", BF);
    float bR, bQ, bY, sinA, cosA; 
    if (k == 0) { foreach (float r in new float[]{-50,0,50}) add.Invoke(cp, new object[]{r,0f,0f,0f}); end.Invoke(cp,null);
      foreach (float q in new float[]{-30,0,30}) add.Invoke(cp, new object[]{0f,q,0f,0f}); end.Invoke(cp,null);
      foreach (float y in new float[]{-20,0,20}) add.Invoke(cp, new object[]{0f,0f,y,0f}); end.Invoke(cp,null);
      bR=10; bQ=5; bY=3; sinA=0; cosA=1; }
    else { int ne = 3 + rnd.Next(7); for (int e = 0; e < ne; e++) { int np = 2 + rnd.Next(8); float sr=(float)(rnd.NextDouble()*80-40), sq=(float)(rnd.NextDouble()*60-30), sy=(float)(rnd.NextDouble()*50-25), o=(float)(rnd.NextDouble()*20-10);
        for (int p = 0; p < np; p++) { float u = -1f + 2f*p/(np-1); add.Invoke(cp, new object[]{o+sr*u+ (float)(rnd.NextDouble()*4-2), sq*u*(1f-0.3f*u), sy*u, u}); } end.Invoke(cp,null); }
      double al = (rnd.NextDouble()*110-40)*Math.PI/180; sinA=(float)Math.Sin(al); cosA=(float)Math.Cos(al); bR=(float)(rnd.NextDouble()*60-30); bQ=(float)(rnd.NextDouble()*30-15); bY=(float)(rnd.NextDouble()*30-15); }
    object[] args = new object[]{sinA,cosA,bR,bQ,bY,1,0f,0f,0}; bool ok = (bool)res.Invoke(cp, args); n++;
    float rp=(float)args[6], rn=(float)args[7];
    if (k == 0) { Console.WriteLine("[cpl] hand case: ok " + ok + " rp " + rp + " rn " + rn + " (expected 60 and 40)"); if (!ok || Math.Abs(rp-60)>1e-3 || Math.Abs(rn-40)>1e-3) bad++; }
    else if (ok && (float.IsNaN(rp) || float.IsNaN(rn) || rp + rn < -1e-3f)) { bad++; Console.WriteLine("[cpl] odd result k " + k + " rp " + rp + " rn " + rn); }
    { object[] ap = new object[]{sinA,cosA,bR,bQ,bY,1,0f,0f,1}; bool okp = (bool)res.Invoke(cp, ap); object[] an = new object[]{sinA,cosA,bR,bQ,bY,1,0f,0f,-1}; bool okn = (bool)res.Invoke(cp, an); one++;
      float rp1=(float)ap[6], rn1=(float)an[7];
      if (okp != ok || okn != ok || (ok && (Math.Abs(rp1-rp) > 1e-3f*(1f+Math.Abs(rp)) || Math.Abs(rn1-rn) > 1e-3f*(1f+Math.Abs(rn))))) { badOne++; if (badOne < 6) Console.WriteLine("[cpl] one-side differs k " + k + " ok " + ok + "/" + okp + "/" + okn + " rp " + rp + "/" + rp1 + " rn " + rn + "/" + rn1); } }
  }
  sw.Stop(); Console.WriteLine("[cpl] " + n + " problems, " + bad + " bad, " + (sw.Elapsed.TotalMilliseconds*1000/n).ToString("F0") + " us each (reflection included)");
  Console.WriteLine("[cpl] one-side solves against the two-side solve: " + one + " problems, " + badOne + " differ"); bad += badOne;
  Console.WriteLine("[result] " + (bad == 0 ? "PASS" : "FAIL")); return bad; } }
