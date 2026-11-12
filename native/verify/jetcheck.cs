// jetcheck.exe <plugin dll> <namespace> : on the game's own Mono (host.exe, -O=-float32 as Unity runs it), with the plugin's native
// DLL beside it: the float mode, FcsNative.Load (LoadLibraryW by full path, ABI, structure sizes, the grid kernels' self-test against
// the C# twin as this runtime compiles it), and the state lines the plugin logs at start-up. NativeAllocator 2 (the default).
using System;
using System.IO;
using System.Reflection;
static class JetCheck
{
	const BindingFlags BF = BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic;
	static int Main(string[] args)
	{
		string plugin = args[0], ns = args[1];
		Assembly asm = Assembly.LoadFrom(plugin);
		Type tPlugin = asm.GetType(ns + ".Plugin", true), tNat = asm.GetType(ns + ".FcsNative", true);
		float fa = float.Parse("16777216"), fb = float.Parse("1"); float fs = fa + fb; float fd = fs - fa;
		string fake = Path.Combine(Path.GetTempPath(), "fcs_jetcheck_game"); Directory.CreateDirectory(fake);
		MethodInfo sep = typeof(BepInEx.Paths).GetMethod("SetExecutablePath", BF);
		object[] sa = new object[sep.GetParameters().Length]; sa[0] = Path.Combine(fake, "Game.exe");
		for (int i = 1; i < sa.Length; i++) sa[i] = sep.GetParameters()[i].HasDefaultValue ? sep.GetParameters()[i].DefaultValue : null;
		sep.Invoke(null, sa);
		// Plugin.Log for the plugin's warnings
		FieldInfo fLog = tPlugin.GetField("Log", BF) ?? tPlugin.GetField("log", BF);
		PropertyInfo pLog = tPlugin.GetProperty("Log", BF);
		BepInEx.Logging.ManualLogSource ls = BepInEx.Logging.Logger.CreateLogSource("check");
		BepInEx.Logging.Logger.Listeners.Add(new ConsoleListener());
		if (fLog != null) fLog.SetValue(null, ls); else if (pLog != null && pLog.CanWrite) pLog.SetValue(null, ls, null);
		Console.WriteLine("[check] Plugin.NativeAllocator = " + tPlugin.GetProperty("NativeAllocator", BF).GetValue(null, null) + ", check = " + tPlugin.GetProperty("NativeAllocatorCheck", BF).GetValue(null, null)
			+ ", NativeSweep = " + tPlugin.GetProperty("NativeSweep", BF).GetValue(null, null) + ", NativeTables = " + tPlugin.GetProperty("NativeTables", BF).GetValue(null, null) + ", SweepGridCache = " + tPlugin.GetProperty("SweepGridCache", BF).GetValue(null, null));
		Console.WriteLine("[check] native allocator core " + tNat.GetMethod("Describe", BF).Invoke(null, null) + "; " + tNat.GetMethod("DescribeKernels", BF).Invoke(null, null) + ".");
		Console.WriteLine("[check] loaded from: " + tNat.GetField("LoadedFrom", BF).GetValue(null) + " | state: " + tNat.GetField("Why", BF).GetValue(null) + " | kernels: " + tNat.GetField("KernelWhy", BF).GetValue(null));
		// the self-test again, timed (its 6 rounds x 31 points x both modes, library and twin)
		MethodInfo st = tNat.GetMethod("SelfTest", BF);
		System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew(); string r = null; int k = 0;
		for (; k < 200; k++) { r = (string)st.Invoke(null, null); if (r != null) break; }
		sw.Stop();
		Console.WriteLine("[check] self-test x" + k + ": " + (r ?? "passed every time") + " (" + (sw.Elapsed.TotalMilliseconds * 1000.0 / Math.Max(1, k)).ToString("F0") + " us per run)");
		// the coupled-cap solver (Ternion 0.3.0; every FCS core from 11 Nov) (its self-test ran inside Load) and its timing on this runtime; where the core came from
		Console.WriteLine("[check] coupled-cap solver: " + tNat.GetField("CplWhy", BF).GetValue(null) + " | unpacked from the plugin DLL: " + (tNat.GetField("Unpacked", BF) != null ? tNat.GetField("Unpacked", BF).GetValue(null) : (object)"(not reported by this jet; see the load line)"));
		MethodInfo cst = tNat.GetMethod("CplSelfTest", BF); string cr = null; int ck = 0;
		for (; ck < 50; ck++) { cr = (string)cst.Invoke(null, null); if (cr != null) break; }
		Console.WriteLine("[check] coupled-cap self-test x" + ck + ": " + (cr ?? "passed every time"));
		Console.WriteLine("[check] " + tNat.GetMethod("CplBench", BF).Invoke(null, new object[] { 2000 }));
		bool ok = cr == null && ((string)tNat.GetField("CplWhy", BF).GetValue(null)) == "ready" && (bool)tNat.GetProperty("Ready", BF).GetValue(null, null) && ((string)tNat.GetField("KernelWhy", BF).GetValue(null)) == "ready" && r == null;
		Console.WriteLine("[result] " + (ok ? "PASS" : "FAIL"));
		return ok ? 0 : 1;
	}
	sealed class ConsoleListener : BepInEx.Logging.ILogListener
	{
		public void LogEvent(object sender, BepInEx.Logging.LogEventArgs e) { Console.WriteLine("[log " + e.Level + "] " + e.Data); }
		public void Dispose() { }
	}
}
