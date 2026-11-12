// Su-30 0.5.5: on the game's Mono - JIT-compile the flap path (FcsController.LandingFlapJob / GameFlapStep, Hooks.SurfacePostfix,
// FlapSystem.Update) so every field and type it touches (ControlSurfaceFields.gearState / pitchRange / currentPitch, LandingGear.GearState)
// resolves against the game's Assembly-CSharp; print the GearState values the job's flap branch tests. Nothing is invoked.
using System; using System.Reflection; using System.Runtime.CompilerServices;
static class FlapCheck {
  const BindingFlags BF = BindingFlags.Instance|BindingFlags.Static|BindingFlags.Public|BindingFlags.NonPublic;
  static int Main(string[] a) {
    var asm = Assembly.LoadFrom(a[0]); int bad = 0;
    var gs = Type.GetType("LandingGear+GearState, Assembly-CSharp", true);
    foreach (var n in Enum.GetNames(gs)) Console.WriteLine("[flap] GearState." + n + " = " + Convert.ToInt32(Enum.Parse(gs, n)));
    var csf = Type.GetType("NuclearOption.Jobs.ControlSurfaceFields, Assembly-CSharp", true);
    foreach (var n in new[]{"gearState","pitchRange","currentPitch","servoSpeed","flap"}) { var f = csf.GetField(n, BF); Console.WriteLine("[flap] ControlSurfaceFields." + n + ": " + (f == null ? "MISSING" : f.FieldType.Name)); if (f == null) bad++; }
    foreach (var tm in new[]{"Su30_VXN.FCS.FcsController:LandingFlapJob","Su30_VXN.FCS.FcsController:GameFlapStep","Su30_VXN.FCS.Hooks:SurfacePostfix","Su30_VXN.FCS.FlapSystem:Update"}) {
      var p = tm.Split(':'); try { var t = asm.GetType(p[0], true); int k = 0;
        foreach (var m in t.GetMethods(BF)) if (m.Name == p[1]) { RuntimeHelpers.PrepareMethod(m.MethodHandle); k++; }
        Console.WriteLine("[flap] JIT " + tm + ": " + (k > 0 ? "ok x" + k : "NOT FOUND")); if (k == 0) bad++;
      } catch (Exception e) { bad++; Console.WriteLine("[flap] JIT " + tm + " FAILED: " + (e.InnerException ?? e).GetType().Name + " " + (e.InnerException ?? e).Message); }
    }
    Console.WriteLine("[result] " + (bad == 0 ? "PASS" : "FAIL " + bad)); return bad;
  }
}
