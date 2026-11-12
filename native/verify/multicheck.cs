// multicheck.exe <dll> <ns> [<dll> <ns> ...]: every FCS plugin in one Mono process, as in the game: each loads its own native DLL;
// then each one's solve state and kernel self-test, after all are loaded (a library bound to the wrong module would fail its ABI check
// or its self-test, or report another jet's path)
using System;
using System.IO;
using System.Reflection;
static class MultiCheck
{
	const BindingFlags BF = BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic;
	static int Main(string[] args)
	{
		string fake = Path.Combine(Path.GetTempPath(), "fcs_multicheck_game"); Directory.CreateDirectory(fake);
		MethodInfo sep = typeof(BepInEx.Paths).GetMethod("SetExecutablePath", BF);
		object[] sa = new object[sep.GetParameters().Length]; sa[0] = Path.Combine(fake, "Game.exe");
		for (int i = 1; i < sa.Length; i++) sa[i] = sep.GetParameters()[i].HasDefaultValue ? sep.GetParameters()[i].DefaultValue : null;
		sep.Invoke(null, sa);
		int n = args.Length / 2; Type[] nat = new Type[n]; string[] ns = new string[n];
		for (int i = 0; i < n; i++)
		{
			Assembly a = Assembly.LoadFrom(args[2 * i]); ns[i] = args[2 * i + 1];
			nat[i] = a.GetType(ns[i] + ".FcsNative", true);
			Console.WriteLine("[load] " + ns[i] + ": " + nat[i].GetMethod("Describe", BF).Invoke(null, null).ToString().Split(';')[0]);
		}
		bool ok = true;
		for (int i = 0; i < n; i++)
		{
			string st = (string)nat[i].GetMethod("SelfTest", BF).Invoke(null, null);
			string from = (string)nat[i].GetField("LoadedFrom", BF).GetValue(null);
			bool ready = (bool)nat[i].GetProperty("Ready", BF).GetValue(null, null);
			Console.WriteLine("[after all] " + ns[i] + ": ready " + ready + ", " + Path.GetFileName(from) + ", self-test " + (st ?? "passed") + ", kernels " + nat[i].GetField("KernelWhy", BF).GetValue(null));
			ok &= ready && st == null;
		}
		Console.WriteLine("[result] " + (ok ? "PASS" : "FAIL"));
		return ok ? 0 : 1;
	}
}
