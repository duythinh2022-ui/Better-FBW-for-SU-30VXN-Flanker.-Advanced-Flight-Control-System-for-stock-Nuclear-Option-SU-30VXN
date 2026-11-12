// Su-30 0.5.3: on the game's Mono - Acc's FieldRefAccess set (incl. the 4 new LandingGear fields) and the Harmony patch of
// LandingGear.FixedUpdate with Hooks.GearPrefix / GearPostfix (struct __state). Nothing is invoked.
using System; using System.Reflection; using System.Runtime.CompilerServices;
static class GearCheck {
  static int Main(string[] a) {
    var asm = Assembly.LoadFrom(a[0]); int bad = 0;
    try { var acc = asm.GetType("Su30_VXN.FCS.Acc", true); RuntimeHelpers.RunClassConstructor(acc.TypeHandle);
      foreach (var n in new[]{"LG_braked","LG_dbf","LG_groundSpeed","LG_brakeStrength","LG_inputs","LG_steering"}) Console.WriteLine("[gear] Acc." + n + ": " + (acc.GetField(n, BindingFlags.Public|BindingFlags.Static).GetValue(null) != null ? "bound" : "NULL"));
    } catch (Exception e) { bad++; Console.WriteLine("[gear] Acc init FAILED: " + (e.InnerException ?? e).Message); }
    try {
      var hooks = asm.GetType("Su30_VXN.FCS.Hooks", true);
      var lg = Type.GetType("LandingGear, Assembly-CSharp", true);
      var h = new HarmonyLib.Harmony("su30.gearcheck");
      var orig = HarmonyLib.AccessTools.Method(lg, "FixedUpdate");
      h.Patch(orig, new HarmonyLib.HarmonyMethod(hooks.GetMethod("GearPrefix")), new HarmonyLib.HarmonyMethod(hooks.GetMethod("GearPostfix")));
      var info = HarmonyLib.Harmony.GetPatchInfo(orig);
      Console.WriteLine("[gear] LandingGear.FixedUpdate patched: prefixes " + info.Prefixes.Count + ", postfixes " + info.Postfixes.Count + ", __state " + hooks.GetMethod("GearPrefix").GetParameters()[1].ParameterType.GetElementType().Name);
    } catch (Exception e) { bad++; Console.WriteLine("[gear] patch FAILED: " + (e.InnerException ?? e).ToString().Split('\n')[0]); }
    Console.WriteLine("[gear result] " + (bad == 0 ? "PASS" : "FAIL")); return bad;
  }
}
