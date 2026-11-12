using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// F-22E 3.0.4 tick timer: the main-thread wall time of FcsController.Tick + VmcApply per physics tick, per jet. Every 10 s
	// (real time) one log line for the player's F-22E and one for all AI Su-30s together. Drives nothing.
	internal static class TickStats
	{
		private sealed class Acc
		{
			public readonly float[] s = new float[2048];
			public int n;
			public double sum;
			public float max;
			public bool ai;
			public int joins0, waits0;
			public double waitSum0;
			public double reb, all, kick, row;
			public double law, asym, asymMom, slip, sweep, env, mass, flaps, gnd, vmc;
			public int inline0;
			// 4.1.3: the tick in 13 consecutive segments (us sums over ticks that ran to their end), heap growth over ticks without a
			// collection, ticks with a collection, the switches at the window's start and whether they changed in it, slow-tick lines
			public readonly double[] seg = new double[13];
			public int segN;
			public double memSum;
			public int memN;
			public int gcTicks;
			public double gcTickUs;
			public int mask0 = -1;
			public bool mixed;
			public int slowLines;
		}

		// F-22E 4.1.3: the segments' names (FcsController.mk checkpoints), in tick order
		internal static readonly string[] SegNames = new string[13] { "join + last row", "mass props", "air data + gear", "flaps", "surface prep", "tables", "pre-law (INDI, authority, weights, envelope)", "asym step", "mode + stick", "ground steering", "laws + allocator", "apply", "csv capture" };

		private static string DescribeMask(int m)
		{
			string d = m.ToString("D9");
			return "Telemetry " + d[0] + ", NativeAllocator " + d[1] + ", NativeSweep " + d[2] + ", NativeTables " + d[3] + ", SweepGridCache " + d[4] + ", ParallelTables " + d[5] + ", ParallelSweep " + d[6] + ", ParallelAllocator " + d[7] + ", EstimatorThreaded " + d[8];
		}

		// F-22E 4.1.3: the managed heap and the collection count before a tick (Hooks)
		public static void MemStart(FcsController c)
		{
			if (!Plugin.TickTimerLog)
			{
				return;
			}
			c.gcT0 = GC.CollectionCount(0);
			c.memT0 = GC.GetTotalMemory(false);
		}

		private static readonly Dictionary<FcsController, Acc> accs = new Dictionary<FcsController, Acc>();

		private static readonly List<FcsController> dead = new List<FcsController>();

		private static float[] sortBuf = new float[2048];

		private static float lastLog = -1f;

		public static long Start()
		{
			return Stopwatch.GetTimestamp();
		}

		public static void Stop(FcsController c, long t0, long vmcTicks = 0L)
		{
			float us = (float)((double)(Stopwatch.GetTimestamp() - t0) * 1000000.0 / (double)Stopwatch.Frequency);
			c.NoteTickTime(us, vmcTicks);
			if (!Plugin.TickTimerLog)
			{
				return;
			}
			// 4.1.3: heap growth and collections over this tick (the heap is read in 4 KB blocks, so a single tick's figure is coarse;
			// the window's sum over ticks without a collection is the allocation rate)
			try   // 4.1.3: diagnostics never take the FCS down (an exception here would reach the hook's Registry.Fail)
			{
			long mem1 = GC.GetTotalMemory(false);
			int gc1 = GC.CollectionCount(0);
			c.memDelta = mem1 - c.memT0;
			c.gcDuring = gc1 - c.gcT0;
			if (c.mkDone)
			{
				for (int i = 0; i < 13; i++)
				{
					c.segUs[i] = FcsController.TicksToUs(c.mk[i + 1] - c.mk[i]);
				}
			}
			else
			{
				Array.Clear(c.segUs, 0, 13);
			}
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("Su-30 FCS tick segments: " + ex.Message);
			}
			if (!accs.TryGetValue(c, out Acc a))
			{
				a = new Acc();
				accs[c] = a;
			}
			a.ai = c.isAI;
			if (a.n < a.s.Length)
			{
				a.s[a.n] = us;
			}
			a.n++;
			a.sum += us;
			a.reb += FcsController.TicksToUs(c.phRebuild);
			a.all += FcsController.TicksToUs(c.phAlloc);
			a.kick += FcsController.TicksToUs(c.phKick);
			a.row += FcsController.TicksToUs(c.phRow);
			a.law += FcsController.TicksToUs(c.phLaws);
			a.asym += FcsController.TicksToUs(c.phAsym);
			a.asymMom += FcsController.TicksToUs(c.phAsymMom);
			a.slip += FcsController.TicksToUs(c.phSlip);
			a.sweep += FcsController.TicksToUs(c.phSweep);
			a.env += FcsController.TicksToUs(c.phEnv);
			a.mass += FcsController.TicksToUs(c.phMass);
			a.flaps += FcsController.TicksToUs(c.phFlaps);
			a.gnd += FcsController.TicksToUs(c.phGround);
			a.vmc += FcsController.TicksToUs(c.phVmc);
			if (us > a.max)
			{
				a.max = us;
			}
			try
			{
			if (c.mkDone)
			{
				a.segN++;
				for (int i = 0; i < 13; i++)
				{
					a.seg[i] += c.segUs[i];
				}
			}
			if (c.gcDuring > 0)
			{
				a.gcTicks++;
				a.gcTickUs += us;
			}
			else
			{
				a.memSum += c.memDelta;
				a.memN++;
			}
			int msk = FcsController.SwitchMask();
			if (a.mask0 < 0)
			{
				a.mask0 = msk;
			}
			else if (msk != a.mask0)
			{
				a.mixed = true;
			}
			float slow = Plugin.SlowTickLogUs;
			if (!c.isAI && slow > 0f && us > slow && a.slowLines < 10)
			{
				a.slowLines++;
				SlowTick(c, us, vmcTicks);
			}
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("Su-30 FCS tick segment stats: " + ex.Message);
			}
			float now = Time.realtimeSinceStartup;
			if (lastLog < 0f)
			{
				lastLog = now;
			}
			if (now - lastLog >= 10f)
			{
				Log(now - lastLog);
				lastLog = now;
			}
		}

		// F-22E 4.1.3: one slow tick's breakdown
		private static void SlowTick(FcsController c, float us, long vmcTicks)
		{
			try
			{
				System.Text.StringBuilder sb = new System.Text.StringBuilder(400);
				for (int i = 0; i < 13; i++)
				{
					sb.Append((i == 0) ? "" : ", ").Append(SegNames[i]).Append(' ').Append(c.segUs[i].ToString("F0", System.Globalization.CultureInfo.InvariantCulture));
				}
				Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
					"Su-30 FCS slow tick: {0:F0} us at flight time {1:F2} s ({2}) | {3}, VmcApply {4:F0} | garbage collection during it: {5} | heap growth {6} bytes | of it: tables {7:F0}, allocator {8:F0}, asym sweep {9:F0}, estimator wait {10:F0}",
					us, c.FlightTime, c.mkDone ? "complete" : "returned early", sb.ToString(), FcsController.TicksToUs(vmcTicks), (c.gcDuring > 0) ? "yes" : "no", c.memDelta,
					FcsController.TicksToUs(c.phRebuild), FcsController.TicksToUs(c.phAlloc), FcsController.TicksToUs(c.phSweep), c.EstLastWaitUs));
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("Su-30 FCS slow-tick log: " + ex.Message);
			}
		}

		private static void Log(float span)
		{
			try
			{
				int aiJets = 0, aiTicks = 0;
				double aiSum = 0.0, aiMeanSum = 0.0;
				float aiMax = 0f;
				int aiWaits = 0, aiJoins = 0;
				dead.Clear();
				foreach (KeyValuePair<FcsController, Acc> kv in accs)
				{
					Acc a = kv.Value;
					if (a.n == 0)
					{
						dead.Add(kv.Key);
						continue;
					}
					EstimatorJob j = kv.Key.estJob;
					int dj = j.joins - a.joins0, dw = j.waits - a.waits0;
					double dws = j.waitUsSum - a.waitSum0;
					int di = j.inline - a.inline0;
					a.inline0 = j.inline;
					a.joins0 = j.joins;
					a.waits0 = j.waits;
					a.waitSum0 = j.waitUsSum;
					if (!a.ai)
					{
						int m = Math.Min(a.n, a.s.Length);
						Array.Copy(a.s, sortBuf, m);
						Array.Sort(sortBuf, 0, m);
						float p95 = sortBuf[Math.Min(m - 1, (int)(0.95 * (double)m))];
						float p50 = sortBuf[m / 2];
						Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
							"Su-30 FCS timing (player, {0:F0} s, {1} ticks): mean {2:F0} us, median {3:F0}, p95 {4:F0}, max {5:F0} | per tick: tables {10:F0}, allocator {11:F0}, estimator kick {12:F0}, csv {13:F0}, rest {14:F0} us | threads: estimator {6}, tables {15}, allocator {16}; estimator joins {8}: {7} waited (mean {9:F0} us), {17} run inline",
							span, a.n, a.sum / (double)a.n, p50, p95, a.max, Plugin.EstimatorThreaded ? "on" : "off", dw, dj, (dw > 0) ? (dws / (double)dw) : 0.0,
							a.reb / a.n, a.all / a.n, a.kick / a.n, a.row / a.n, (a.sum - a.reb - a.all - a.kick - a.row) / a.n,
							Plugin.ParallelTables ? "on" : "off", Plugin.ParallelAllocator ? "on" : "off", di));
						double nn = a.n;
						// 4.1.3: the whole tick in consecutive segments, the heap, the switches
						if (a.segN > 0)
						{
							System.Text.StringBuilder sb = new System.Text.StringBuilder(512);
							double segSum = 0.0;
							for (int i = 0; i < 13; i++)
							{
								double v = a.seg[i] / (double)a.segN;
								segSum += v;
								sb.Append((i == 0) ? "" : ", ").Append(SegNames[i]).Append(' ').Append(v.ToString("F0", System.Globalization.CultureInfo.InvariantCulture));
							}
							Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
								"Su-30 FCS timing, whole tick in segments (us per tick, {0} ticks): {1} | sum {2:F0}, VmcApply {3:F0}, call + timer overhead {4:F0} | heap growth {5:F0} bytes per tick over {6} ticks without a collection; {7} ticks with a collection (mean {8:F0} us) | switches: {9}{10}",
								a.segN, sb.ToString(), segSum, a.vmc / (double)a.n, (a.sum - a.vmc) / (double)a.n - segSum,
								(a.memN > 0) ? (a.memSum / (double)a.memN) : 0.0, a.memN, a.gcTicks, (a.gcTicks > 0) ? (a.gcTickUs / (double)a.gcTicks) : 0.0,
								DescribeMask(a.mask0), a.mixed ? " (CHANGED during this window: mixed)" : ""));
						}
						Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
							"Su-30 FCS timing, rest split (us per tick): laws {0:F0}, asym step {1:F0} (of it: asym moment {2:F0}, slip probe {3:F0}, asym sweep {4:F0}), envelope {5:F0}, mass props {6:F0}, flaps {7:F0}, ground steering {8:F0}, VmcApply {9:F0}, still unaccounted {10:F0}",
							a.law / nn, a.asym / nn, a.asymMom / nn, a.slip / nn, a.sweep / nn, a.env / nn, a.mass / nn, a.flaps / nn, a.gnd / nn, a.vmc / nn,
							(a.sum - a.reb - a.all - a.kick - a.row - a.law - a.asym - a.env - a.mass - a.flaps - a.gnd - a.vmc) / nn));
					}
					else
					{
						aiJets++;
						aiTicks += a.n;
						aiSum += a.sum;
						aiMeanSum += a.sum / (double)a.n;
						if (a.max > aiMax)
						{
							aiMax = a.max;
						}
						aiWaits += dw;
						aiJoins += dj;
					}
					a.n = 0;
					a.sum = 0.0;
					a.reb = a.all = a.kick = a.row = 0.0;
					a.law = a.asym = a.asymMom = a.slip = a.sweep = a.env = a.mass = a.flaps = a.gnd = a.vmc = 0.0;
					a.max = 0f;
					Array.Clear(a.seg, 0, 13);
					a.segN = 0;
					a.memSum = 0.0;
					a.memN = 0;
					a.gcTicks = 0;
					a.gcTickUs = 0.0;
					a.mask0 = -1;
					a.mixed = false;
					a.slowLines = 0;
				}
				if (aiJets > 0)
				{
					Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
						"Su-30 FCS timing (AI, {0} jets, {1:F0} s): mean {2:F0} us per jet-tick, sum of the jets' means {3:F0} us per physics tick, max {4:F0} | estimator joins waited {5} of {6}",
						aiJets, span, aiSum / (double)Math.Max(1, aiTicks), aiMeanSum, aiMax, aiWaits, aiJoins));
				}
				foreach (FcsController c in dead)
				{
					accs.Remove(c);
				}
				// F-22E 4.0.0: the native allocator's state and its counts since the game started
				Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
					"Su-30 FCS native allocator: {0} | since start: {1} native solves; {2} solved both ways: {3} identical, {4} within {7} deg, {5} equal-cost alternatives, {6} different; {8} left to the C# solve | {9}: {10} kernel calls, {11} left to C#",
					FcsNative.Describe(),
					System.Threading.Interlocked.Read(ref FcsNative.Solves), System.Threading.Interlocked.Read(ref FcsNative.Checked),
					System.Threading.Interlocked.Read(ref FcsNative.Identical), System.Threading.Interlocked.Read(ref FcsNative.Close),
					System.Threading.Interlocked.Read(ref FcsNative.Tie), System.Threading.Interlocked.Read(ref FcsNative.Mismatches),
					FcsNative.CloseDeg, System.Threading.Interlocked.Read(ref FcsNative.Fallbacks),
					FcsNative.DescribeKernels(), System.Threading.Interlocked.Read(ref FcsNative.GridCalls), System.Threading.Interlocked.Read(ref FcsNative.GridFallbacks)));
				HookTimer.Log();
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("Su-30 FCS timing log: " + ex.Message);
			}
		}
	}

	// F-22E 3.1.3: the airframe force hooks' main-thread time (DragRelief, CamberForce, DownwashForce: MonoBehaviour.FixedUpdate on
	// each F-22E, outside the FCS tick and its timer), reported with the TickTimerLog lines. Drives nothing.
	internal static class HookTimer
	{
		private static readonly long[] pl = new long[3];

		private static readonly long[] all = new long[3];

		private static int steps;

		private static float lastFixed = -1f;

		public static void Note(int k, bool player, long ticks)
		{
			float ft = Time.fixedTime;
			if (ft != lastFixed)
			{
				lastFixed = ft;
				steps++;
			}
			all[k] += ticks;
			if (player)
			{
				pl[k] += ticks;
			}
		}

		public static void Log()
		{
			if (steps == 0)
			{
				return;
			}
			double n = steps;
			Plugin.Log.LogInfo(string.Format(System.Globalization.CultureInfo.InvariantCulture,
				"Su-30 force hooks (us per physics tick, outside the FCS tick, {0} ticks): player jet: drag relief {1:F0}, camber {2:F0}, downwash {3:F0} | all Su-30s: drag relief {4:F0}, camber {5:F0}, downwash {6:F0}",
				steps, FcsController.TicksToUs(pl[0]) / n, FcsController.TicksToUs(pl[1]) / n, FcsController.TicksToUs(pl[2]) / n,
				FcsController.TicksToUs(all[0]) / n, FcsController.TicksToUs(all[1]) / n, FcsController.TicksToUs(all[2]) / n));
			for (int i = 0; i < 3; i++)
			{
				pl[i] = 0L;
				all[i] = 0L;
			}
			steps = 0;
		}
	}
}
