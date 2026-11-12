using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;

namespace Su30_VXN.FCS
{
	// Su-30 0.4.0 (NativeAllocator, ported from the F-22E FCS 4.0.0 - 4.1.3): the allocator's solve in native code
	// (native/fcs_core in the source, Rust), Su30_FcsCore.dll, carried inside this plugin's DLL (0.4.1: one DLL per jet; see Extract). Its own DLL name and export prefix (su30_fcs_),
	// so the six FCS mods in one game never bind to each other's library. What follows is the F-22E's description; it holds here.
	// This jet's additions: the velocity-frame lateral residual (slipFrame, slipSin, slipCos in AllocIo), the Newton candidate for the yaw nozzles (EffN.kind 3 = TvcYawEffector) and, from the table kernel, each grid point's normal force (SurfaceEffector.FN, GridIn.outFN).
	//
	// What is native: Allocator.Solve (SolveInner, the reach pass, the two-horizon pick, SolveCore, the paired search, the linear
	// solve, Effector.MAt), ported line for line. It uses nothing but + - * /, comparisons, conversions and square roots, which
	// IEEE 754 defines to the last bit, so the library returns exactly what the C# solve returns wherever C# float arithmetic is
	// strict 32-bit: .NET (the bench), Mono with its float32 option on.
	//
	// What it cannot match: Unity starts the game's Mono with -O=-float32 (checked in UnityPlayer.dll 2022.3.62f2 and on the game's
	// own mono-2.0-bdwgc.dll). In that mode every C# float expression is evaluated in double and rounded to float only when it is
	// stored to a field or array, passed to or returned from a call that is not inlined, or kept across a branch. The C# solve in
	// the game therefore differs from the same C# on the bench, and from this library, in the last digits of most solves, and the
	// allocator's discrete picks turn some of those into a different (mostly equal-cost) answer for that tick. No outside library
	// reproduces the JIT's rounding points, so in the game the native solve is not the same arithmetic as the C# one.
	//
	// Hence the switch (Dev: NativeAllocator): 1 = use the library only where it is bit-identical (StrictFloat below), 2 = always,
	// 0 = never. The C# allocator is unchanged and stays the reference and the fallback (library missing, a failed check).
	// NativeAllocatorCheck solves picked solves both ways, keeps the C# result on them, and counts how the two compare.
	//
	// F-22E 4.1.0: the same library also holds the grid kernels (su30_fcs_grid): a control surface's moment at every deflection of
	// its grid in one airflow. Two users: the table rebuild (SurfaceEffector.EvalCompute's loop of Eval over the grid) and the
	// asymmetric-load sweep's capacity loop (FcsController.AsymSweepPoint). The rotations, which need sin / cos, are worked out in
	// C# (SurfaceEffector.GridGeom); the kernels do the flow's part: rotate, one atan2, the airfoil table, the force and its moment.
	// Same rule as the solve: bit-identical where C# float arithmetic is strict (the bench's suite checks it), the bench's arithmetic
	// instead of Mono's in the game. Switches (Dev): NativeSweep, NativeTables, under NativeAllocator. A self-test at load compares
	// the kernels with a C# twin on made-up data; a failure leaves the kernels off and the solve on.
	internal static unsafe class FcsNative
	{
		public const string Lib = "Su30_FcsCore";

		// the library's su30_fcs_abi(): bumped with any change to the structures below
		public const int Abi = 5005006;

		public const int MaxEff = 16;

		public const int MaxN = 32;

		// one effector as the solve reads it (native/fcs_core/src/lib.rs: EffN), 640 bytes
		[StructLayout(LayoutKind.Sequential)]
		public struct EffN
		{
			public int active;
			public int kind;        // 1 = TvcEffector, 2 = SurfaceEffector of kind Stabilator, 3 = TvcYawEffector, 0 = any other
			public int partner;     // index in the list, -1 = none
			public int rollGroup;
			public int pitchGroup;
			public int twoHorizon;
			public int n;
			public int nodeOn;
			public int nodeK;
			public float min;
			public float max;
			public float now;
			public float cmd;
			public float rate;
			public float reachTau;
			public float mu;
			public float lam;
			public float lo;
			public float hi;
			public float pairSign;
			public float symCost;
			public float symTarget;
			public float antiCost;
			public float anchor;
			public float band;
			public float anchorK;
			public float regN;
			public float step;
			public float nodeC;
			public float nodeDx;
			public float nodeDy;
			public float nodeDz;
			public fixed float grid[MaxN];
			public fixed float m[MaxN * 3];
		}

		// the allocator's configuration, the target and what the solve leaves (lib.rs: AllocIo)
		[StructLayout(LayoutKind.Sequential)]
		public struct AllocIo
		{
			public double lmFactor;
			public int n;
			public int gnIterations;
			public int pairedTrimSearch;
			public int fightStabSplitOnly;
			public int reachPass;
			public int twoHorizon;
			public int twoHorizonPath;
			public int coordNewton;
			public float axX;
			public float axY;
			public float axZ;
			public float regScale;
			public float fightK;
			public fixed float fightW[4];
			public float pitchFightK;
			public float pitchFightSign;
			public fixed float inv[9];
			public float fightTargetMin;
			public float pairHorizon;
			public float tX;
			public float tY;
			public float tZ;
			public float lastCost;          // in / out
			public float resX;              // in / out: resDbg
			public float resY;
			public float resZ;
			public int lastNeededPick;      // in / out
			public int reachMoves;          // out: this solve's increments
			public int twoHorizonBound;
			public int coordNewtonMoves;
			public int pairedMoves;
			public int coreSolves;
			public int slipFrame;           // Allocator.slipFrame (the velocity-frame lateral residual)
			public float slipSin;
			public float slipCos;
			public int reserved;
		}

		// one surface, one airflow, every grid deflection (lib.rs: GridIn), 248 bytes; the arrays are pinned for the call
		[StructLayout(LayoutKind.Sequential)]
		public struct GridIn
		{
			public UnityEngine.Quaternion* gInv;
			public UnityEngine.Vector3* gRight;
			public UnityEngine.Vector3* gArm;
			public UnityEngine.Quaternion* ghInv;
			public UnityEngine.Vector3* ghRight;
			public byte* ghZero;
			public float* cl;
			public float* cd;
			public float* hcl;
			public float* hcd;
			public UnityEngine.Vector3* outM;     // mode 0: the n moments
			public float* outFN;                  // mode 0: the n normal forces (SurfaceEffector.FN), null = not wanted
			public int n;
			public int host;                      // hostK > 0
			public int hvSmall;                   // hv.sqrMagnitude < 1E-06f
			public int reserved;
			public UnityEngine.Vector3 v;         // the surface's airflow
			public UnityEngine.Vector3 vhat;      // airflow / speed
			public float num;                     // 0.5 rho v^2 S eff
			public UnityEngine.Vector3 hv;        // the host section's airflow
			public UnityEngine.Vector3 hvn;       // its Vector3.normalized
			public float hq;                      // its 0.5 rho v^2 S eff
			public UnityEngine.Vector3 hostArm;
			public UnityEngine.Vector3 hostBase;
			public UnityEngine.Quaternion rootInv;
			public fixed float ia[4];             // mode 1: IaInv[0,0], [0,2], [2,0], [2,2]
			public fixed float o[6];              // mode 1: pitch min, max, roll accel min, max, yaw accel min, max
		}

		// Su-30 0.5.6 (from the Ternion 0.3.0): one call of the coupled asymmetric-load check's LP (lib.rs / cpl.rs: CplIo), 136 bytes; the arrays are the
		// CplPoint's own, pinned for the call. op: 0 = Reserve, 1 = LateralFeasible, 2 = PitchHeld
		[StructLayout(LayoutKind.Sequential)]
		public struct CplIo
		{
			public float* r;
			public float* q;
			public float* y;
			public int* effStart;
			public int* effOf;
			public double* t;
			public double* t1;
			public int* basis;
			public int* basis1;
			public double* a0;
			public double* a1;
			public double* cc;
			public int nCol;
			public int nEff;
			public int op;
			public int sgn;       // Reserve: sgnA; PitchHeld: dir
			public float sinA;
			public float cosA;
			public float bR;
			public float bQ;
			public float bY;
			public float rp;      // out
			public float rn;      // out
			public int ok;        // out
			public int iters;     // out
			public int side;      // Reserve: 0 both sides, +1 / -1 that side only, 2 phase 1 only
		}

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_cpl(CplIo* io);

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_cpl_size();

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_grid(GridIn* g, int mode);

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_grid_size();

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_abi();

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_sizes();

		[DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
		private static extern int su30_fcs_solve(AllocIo* io, EffN* effs);

		[DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
		private static extern IntPtr LoadLibraryW(string path);

		private static readonly object gate = new object();

		private static int state;   // 0 not tried, 1 ready, -1 not usable this session

		public static string Why = "not loaded yet";

		public static string LoadedFrom = "";

		// counters (diagnostics; the tick timer's log line and the bench read them). Of the Checked solves: Identical = every
		// output bit equal; Close = commands within CloseDeg; Tie = further apart with the costs within TieRel of each other;
		// Mismatches = the rest (and, with strict float arithmetic, anything not Identical: there it is a fault).
		public static long Solves, Checked, Identical, Close, Tie, Mismatches, Fallbacks;

		// 4.1.0: the grid kernels. kernels: 0 not tested, 1 usable, -1 off for this session (KernelWhy says why)
		private static int kernels;

		public static string KernelWhy = "not tested yet";

		public static long GridCalls, GridFallbacks;

		// Su-30 0.5.6 (from the Ternion 0.3.0): the coupled check's LP. cpl: 0 not tested, 1 usable, -1 off for this session (CplWhy says why).
		// CplChecked / CplIdentical / CplMismatches: with NativeAllocatorCheck 2 every call runs both ways (the C# result is used)
		private static int cpl;

		public static string CplWhy = "not tested yet";

		public static long CplCalls, CplFallbacks, CplChecked, CplIdentical, CplMismatches;

		public static bool CplOn => Plugin.NativeCoupledCap && Wanted && Ready && Volatile.Read(ref cpl) == 1;

		// one LP call; false = nothing in io may be used (the caller runs its C# method)
		public static bool Cpl(CplIo* io)
		{
			int r = su30_fcs_cpl(io);
			if (r == 0)
			{
				Interlocked.Increment(ref CplCalls);
				return true;
			}
			Interlocked.Increment(ref CplFallbacks);
			DisableCpl("su30_fcs_cpl returned " + r);
			return false;
		}

		public static void CplCheck(bool same)
		{
			Interlocked.Increment(ref CplChecked);
			if (same)
			{
				Interlocked.Increment(ref CplIdentical);
			}
			else
			{
				Interlocked.Increment(ref CplMismatches);
			}
		}

		public static void DisableCpl(string why)
		{
			bool first;
			lock (gate)
			{
				first = cpl != -1;
				CplWhy = why;
				Volatile.Write(ref cpl, -1);
			}
			if (first)
			{
				try
				{
					Plugin.Log.LogWarning("Su-30 FCS: native coupled-cap solver switched off for this session, the C# solver takes over (" + why + ").");
				}
				catch (Exception)
				{
				}
			}
		}

		// the LP's self-test: made-up surfaces (9 of 9 points, roll / pitch / yaw accelerations of a plausible size) and loads through
		// the library and through CplPoint's own C# methods. As with the kernels the comparison has a tolerance where the runtime
		// carries floats in double; with strict float arithmetic every bit must match.
		private static string CplSelfTest()
		{
			rng = 0x51C0FFEEu;
			CplPoint a = new CplPoint();
			CplPoint b = new CplPoint();
			for (int k = 0; k < 24; k++)
			{
				a.Reset();
				b.Reset();
				int nE = 4 + (k % 7);
				for (int e = 0; e < nE; e++)
				{
					float r0 = Rnd() * 120f, q0 = Rnd() * 90f, y0 = Rnd() * 40f, cr = Rnd() * 30f, cq = Rnd() * 30f, cy = Rnd() * 10f;
					for (int i = 0; i < 9; i++)
					{
						float u = (float)i / 8f * 2f - 1f;
						float rr = r0 * u + cr * u * u, qq = q0 * u + cq * u * u * u, yy = y0 * u + cy * (1f - u * u);
						a.Add(rr, qq, yy);
						b.Add(rr, qq, yy);
					}
					a.EndEff();
					b.EndEff();
				}
				float aD = Rnd() * 55f * 0.0174532924f;
				float sinA = (float)Math.Sin(aD), cosA = (float)Math.Cos(aD);
				float lr = Rnd() * ((k % 5 == 4) ? 900f : 60f), lq = Rnd() * 40f, ly = Rnd() * 15f;
				int sg = (aD >= 0f) ? 1 : -1;
				bool okN = a.ReserveNative(sinA, cosA, lr, lq, ly, sg, out float rpN, out float rnN, out bool fellN);
				bool okC = b.ReserveCs(sinA, cosA, lr, lq, ly, sg, out float rpC, out float rnC);
				if (fellN)
				{
					return "Reserve call failed in test " + k;
				}
				if (okN != okC || (okC && !CplNear(rpN, rpC)) || (okC && !CplNear(rnN, rnC)))
				{
					return "Reserve differs in test " + k + " (" + okN + " " + rpN + " " + rnN + " / " + okC + " " + rpC + " " + rnC + ")";
				}
				for (int sd = -1; sd <= 2; sd++)
				{
					if (sd == 0)
					{
						continue;
					}
					bool okNs = a.ReserveNative(sinA, cosA, lr, lq, ly, sg, out float rpNs, out float rnNs, out bool fellS, sd);
					bool okCs = b.ReserveCs(sinA, cosA, lr, lq, ly, sg, out float rpCs, out float rnCs, sd);
					if (fellS || okNs != okCs || (okCs && !CplNear(rpNs, rpCs)) || (okCs && !CplNear(rnNs, rnCs)))
					{
						return "Reserve(side " + sd + ") differs in test " + k + " (" + okNs + " " + rpNs + " " + rnNs + " / " + okCs + " " + rpCs + " " + rnCs + ")";
					}
				}
				bool lfN = a.LateralFeasibleNative(sinA, cosA, lr, ly, out fellN);
				bool lfC = b.LateralFeasibleCs(sinA, cosA, lr, ly);
				if (fellN || lfN != lfC)
				{
					return "LateralFeasible differs in test " + k;
				}
				float phN = a.PitchHeldNative(-sg, lr, lq, ly, out fellN);
				float phC = b.PitchHeldCs(-sg, lr, lq, ly);
				if (fellN || float.IsNaN(phN) != float.IsNaN(phC) || (!float.IsNaN(phC) && !CplNear(phN, phC)))
				{
					return "PitchHeld differs in test " + k + " (" + phN + " / " + phC + ")";
				}
			}
			return null;
		}

		// timing of the coupled check's solver, library against C#, on a made-up sweep point of the jet's size (10 surfaces of 9 points;
		// per round what CplSolve does: three Reserve calls and one PitchHeld). For the bench and the game-runtime check.
		public static string CplBench(int rounds)
		{
			rng = 0x0BADC0DEu;
			CplPoint a = new CplPoint();
			a.Reset();
			for (int e = 0; e < 10; e++)
			{
				float r0 = Rnd() * 120f, q0 = Rnd() * 90f, y0 = Rnd() * 40f, cr = Rnd() * 30f, cq = Rnd() * 30f, cy = Rnd() * 10f;
				for (int i = 0; i < 9; i++)
				{
					float u = (float)i / 8f * 2f - 1f;
					a.Add(r0 * u + cr * u * u, q0 * u + cq * u * u * u, y0 * u + cy * (1f - u * u));
				}
				a.EndEff();
			}
			float sinA = (float)Math.Sin(0.5), cosA = (float)Math.Cos(0.5);
			double[] us = new double[2];
			float sink = 0f;
			for (int pass = 0; pass < 2; pass++)
			{
				for (int warm = 0; warm < 2; warm++)
				{
					System.Diagnostics.Stopwatch sw = System.Diagnostics.Stopwatch.StartNew();
					for (int k = 0; k < rounds; k++)
					{
						float lr = 30f + 0.01f * (k % 50), lq = 12f, ly = 4f;
						for (int v = 0; v < 3; v++)
						{
							float rp, rn;
							if (pass == 0)
							{
								a.ReserveNative(sinA, cosA, lr + 3f * v, lq, ly - v, 1, out rp, out rn, out bool _);
							}
							else
							{
								a.ReserveCs(sinA, cosA, lr + 3f * v, lq, ly - v, 1, out rp, out rn);
							}
							sink += rp + rn;
						}
						sink += (pass == 0) ? a.PitchHeldNative(-1, lr, lq, ly, out bool _) : a.PitchHeldCs(-1, lr, lq, ly);
					}
					sw.Stop();
					us[pass] = sw.Elapsed.TotalMilliseconds * 1000.0 / rounds;
				}
			}
			return "coupled-cap solve per sweep point (3 Reserve + 1 PitchHeld, 10 surfaces x 9 points): native " + us[0].ToString("F1") + " us, C# " + us[1].ToString("F1") + " us (x" + (us[1] / Math.Max(1e-9, us[0])).ToString("F1") + ")" + (float.IsNaN(sink) ? " [nan]" : "");
		}

		private static bool CplNear(float x, float y)
		{
			// a side the call did not solve is NaN in both (the one-side Reserve); compared as equal on every runtime
			if (float.IsNaN(x) || float.IsNaN(y))
			{
				return float.IsNaN(x) && float.IsNaN(y);
			}
			if (StrictFloat)
			{
				return (float.IsNaN(x) && float.IsNaN(y)) || BitConverter.ToInt32(BitConverter.GetBytes(x), 0) == BitConverter.ToInt32(BitConverter.GetBytes(y), 0);
			}
			return Math.Abs(x - y) <= 0.01f + 1e-4f * Math.Max(Math.Abs(x), Math.Abs(y));
		}


		// the kernels in use for the asymmetric-load sweep / the table rebuild (the solve's switch, the library and the self-test first)
		public static bool SweepOn => Plugin.NativeSweep && Wanted && Ready && Volatile.Read(ref kernels) == 1;

		public static bool TablesOn => Plugin.NativeTables && Wanted && Ready && Volatile.Read(ref kernels) == 1;

		// one kernel call; false = nothing in g may be used (the caller runs its C# loop). A NaN airfoil index (-7) is the flow's
		// doing, not the library's: the C# loop then throws exactly as before, and the kernels stay on.
		public static bool Grid(GridIn* g, int mode)
		{
			int r = su30_fcs_grid(g, mode);
			if (r == 0)
			{
				Interlocked.Increment(ref GridCalls);
				return true;
			}
			Interlocked.Increment(ref GridFallbacks);
			if (r != -7)
			{
				DisableKernels("su30_fcs_grid returned " + r);
			}
			return false;
		}

		public static void DisableKernels(string why)
		{
			bool first;
			lock (gate)
			{
				first = kernels != -1;
				KernelWhy = why;
				Volatile.Write(ref kernels, -1);
			}
			if (first)
			{
				try
				{
					Plugin.Log.LogWarning("Su-30 FCS: native grid kernels switched off for this session, the C# loops take over (" + why + ").");
				}
				catch (Exception)
				{
				}
			}
		}

		public const float CloseDeg = 0.001f;

		public const float TieRel = 0.001f;

		// Whether this runtime does C# float arithmetic in strict 32-bit (then the library and the C# solve agree to the bit).
		// (16777216f + 1f) - 16777216f is 0 when the sum is rounded to float and 1 when it is carried in double.
		private static float probeA = 16777216f, probeB = 1f;

		public static readonly bool StrictFloat = ProbeStrict();

		private static bool ProbeStrict()
		{
			float s = probeA + probeB;
			float d = s - probeA;
			return d == 0f;
		}

		// what Plugin.NativeAllocator asks for on this runtime
		public static bool Wanted
		{
			get
			{
				int m = Plugin.NativeAllocator;
				return m >= 2 || (m == 1 && StrictFloat);
			}
		}

		public static bool Ready
		{
			get
			{
				int s = Volatile.Read(ref state);
				if (s == 0)
				{
					Load();
					s = Volatile.Read(ref state);
				}
				return s == 1;
			}
		}

		public static bool Failed => Volatile.Read(ref state) < 0;

		public static void Load()
		{
			lock (gate)
			{
				if (state != 0)
				{
					return;
				}
				try
				{
					string dir = null;
					try
					{
						dir = Path.GetDirectoryName(typeof(FcsNative).Assembly.Location);
					}
					catch (Exception)
					{
					}
					if (Environment.OSVersion.Platform == PlatformID.Win32NT)
					{
						// Windows looks for a DllImport's library beside the game's exe, not beside the plugin: load it by its full
						// path first, and the imports above then find the loaded module by name.
						// 0.4.1: the library is carried inside this plugin's DLL (resource "Su30_FcsCore.dll") and written once to
						// BepInEx/cache/Su30_FcsCore/<first 16 hex of its SHA-256>/Su30_FcsCore.dll (the file name stays Su30_FcsCore.dll, which is what
						// the imports bind to). A loose Su30_FcsCore.dll beside the plugin (older installs) is used only if that fails.
						string p = Extract(dir, out string ewhy);
						string loose = string.IsNullOrEmpty(dir) ? null : Path.Combine(dir, Lib + ".dll");
						if (p == null && loose != null && File.Exists(loose))
						{
							p = loose;
						}
						if (p == null)
						{
							Why = "the embedded " + Lib + ".dll could not be written out (" + ewhy + ") and none is beside the plugin";
							Volatile.Write(ref state, -1);
							return;
						}
						if (LoadLibraryW(p) == IntPtr.Zero)
						{
							Why = "Windows could not load " + p + " (error " + Marshal.GetLastWin32Error() + ")";
							Volatile.Write(ref state, -1);
							return;
						}
						LoadedFrom = p;
					}
					int abi = su30_fcs_abi();
					if (abi != Abi)
					{
						Why = "library version " + abi + ", this plugin needs " + Abi;
						Volatile.Write(ref state, -1);
						return;
					}
					int want = (sizeof(EffN) << 16) | sizeof(AllocIo);
					int got = su30_fcs_sizes();
					if (got != want)
					{
						Why = "structure sizes differ (library " + (got >> 16) + " / " + (got & 0xFFFF) + ", plugin " + sizeof(EffN) + " / " + sizeof(AllocIo) + ")";
						Volatile.Write(ref state, -1);
						return;
					}
					Why = "ready";
					// 4.1.0: the grid kernels' layout and a self-test; failing either leaves the solve usable. The solve is published as
					// ready only after it (port fix: a table thread arriving during the self-test saw the solve ready and the kernels
					// "not tested yet", and ran that tick's table in C#)
					try
					{
						int gs = su30_fcs_grid_size();
						if (gs != sizeof(GridIn))
						{
							KernelWhy = "GridIn is " + gs + " bytes in the library, " + sizeof(GridIn) + " in the plugin";
							Volatile.Write(ref kernels, -1);
						}
						else
						{
							string bad = SelfTest();
							KernelWhy = bad ?? "ready";
							Volatile.Write(ref kernels, bad == null ? 1 : -1);
						}
					}
					catch (Exception e)
					{
						KernelWhy = e.GetType().Name + ": " + e.Message;
						Volatile.Write(ref kernels, -1);
					}
					if (Volatile.Read(ref kernels) != 1)
					{
						try
						{
							Plugin.Log.LogWarning("Su-30 FCS: native grid kernels not usable, the C# loops run (" + KernelWhy + "). The native solve is not affected.");
						}
						catch (Exception)
						{
						}
					}
					// Su-30 0.5.6 (from the Ternion 0.3.0): the coupled check's LP, its layout and self-test; failing either leaves the rest usable
					try
					{
						int cs = su30_fcs_cpl_size();
						if (cs != sizeof(CplIo))
						{
							CplWhy = "CplIo is " + cs + " bytes in the library, " + sizeof(CplIo) + " in the plugin";
							Volatile.Write(ref cpl, -1);
						}
						else
						{
							string badC = CplSelfTest();
							CplWhy = badC ?? "ready";
							Volatile.Write(ref cpl, badC == null ? 1 : -1);
						}
					}
					catch (Exception e)
					{
						CplWhy = e.GetType().Name + ": " + e.Message;
						Volatile.Write(ref cpl, -1);
					}
					if (Volatile.Read(ref cpl) != 1)
					{
						try
						{
							Plugin.Log.LogWarning("Su-30 FCS: native coupled-cap solver not usable, the C# solver runs (" + CplWhy + "). The native solve and kernels are not affected.");
						}
						catch (Exception)
						{
						}
					}
					Volatile.Write(ref state, 1);
				}
				catch (Exception e)
				{
					Why = e.GetType().Name + ": " + e.Message;
					Volatile.Write(ref state, -1);
				}
			}
		}

		// 0.4.1: the embedded library, written out once per version (its SHA-256 names the folder) and checked byte for byte before
		// use. Another game instance holding the file open is fine: it is the same bytes. null = could not (why says what).
		private static string Extract(string dir, out string why)
		{
			why = null;
			try
			{
				byte[] b;
				using (Stream s = typeof(FcsNative).Assembly.GetManifestResourceStream(Lib + ".dll"))
				{
					if (s == null)
					{
						why = "this plugin DLL carries no " + Lib + ".dll";
						return null;
					}
					b = new byte[s.Length];
					int o = 0;
					while (o < b.Length)
					{
						int r = s.Read(b, o, b.Length - o);
						if (r <= 0)
						{
							break;
						}
						o += r;
					}
					if (o != b.Length)
					{
						why = "the embedded " + Lib + ".dll could not be read";
						return null;
					}
				}
				string h;
				using (System.Security.Cryptography.SHA256 sha = System.Security.Cryptography.SHA256.Create())
				{
					byte[] d = sha.ComputeHash(b);
					System.Text.StringBuilder sb = new System.Text.StringBuilder(16);
					for (int i = 0; i < 8; i++)
					{
						sb.Append(d[i].ToString("x2"));
					}
					h = sb.ToString();
				}
				string folder = Path.Combine(Path.Combine(CacheRoot(dir), Lib), h);
				string p = Path.Combine(folder, Lib + ".dll");
				if (SameBytes(p, b))
				{
					return p;
				}
				Directory.CreateDirectory(folder);
				string tmp = p + "." + Guid.NewGuid().ToString("N") + ".tmp";
				File.WriteAllBytes(tmp, b);
				try
				{
					if (File.Exists(p))
					{
						File.Delete(p);
					}
					File.Move(tmp, p);
				}
				catch (Exception)
				{
					try
					{
						File.Delete(tmp);
					}
					catch (Exception)
					{
					}
				}
				if (SameBytes(p, b))
				{
					return p;
				}
				why = "could not write " + p;
				return null;
			}
			catch (Exception e)
			{
				why = e.GetType().Name + ": " + e.Message;
				return null;
			}
		}

		// BepInEx/cache above the plugin's folder; the user's temp folder if the plugin is not under a BepInEx folder
		private static string CacheRoot(string dir)
		{
			try
			{
				for (DirectoryInfo d = string.IsNullOrEmpty(dir) ? null : new DirectoryInfo(dir); d != null; d = d.Parent)
				{
					if (string.Equals(d.Name, "BepInEx", StringComparison.OrdinalIgnoreCase))
					{
						return Path.Combine(d.FullName, "cache");
					}
				}
			}
			catch (Exception)
			{
			}
			return Path.Combine(Path.GetTempPath(), "NO_FCS_native");
		}

		private static bool SameBytes(string p, byte[] b)
		{
			try
			{
				if (!File.Exists(p) || new FileInfo(p).Length != b.Length)
				{
					return false;
				}
				byte[] f = File.ReadAllBytes(p);
				for (int i = 0; i < b.Length; i++)
				{
					if (f[i] != b[i])
					{
						return false;
					}
				}
				return true;
			}
			catch (Exception)
			{
				return false;
			}
		}

		// one solve; 0 = the buffers hold the answer, anything else = nothing in them may be used
		public static int Solve(AllocIo* io, EffN* effs)
		{
			return su30_fcs_solve(io, effs);
		}

		// the native solve is not to be used again this session (a failed check, an error return)
		public static void Disable(string why)
		{
			bool first;
			lock (gate)
			{
				first = state != -1;
				Why = why;
				Volatile.Write(ref state, -1);
			}
			if (first)
			{
				try
				{
					Plugin.Log.LogWarning("Su-30 FCS: native allocator switched off for this session, the built-in C# solve takes over (" + why + ").");
				}
				catch (Exception)
				{
				}
			}
		}

		// ---- 4.1.0: the kernels' self-test. Made-up geometry, flows and airfoil tables through the library (both modes) and through
		// GridTwin below, the same maths in C# on the same arrays. The two differ in the last digits where the runtime carries floats
		// in double, so the comparison has a tolerance: it is a test of the call (layout, pointers, the right function), not of bits.
		private static uint rng;

		private static float Rnd()
		{
			rng = rng * 1664525u + 1013904223u;
			return (float)(rng >> 8) / 16777216f * 2f - 1f;
		}

		private static UnityEngine.Quaternion RndQ()
		{
			float x = Rnd(), y = Rnd(), z = Rnd(), w = Rnd();
			float m = (float)Math.Sqrt(x * x + y * y + z * z + w * w);
			if (m < 0.01f)
			{
				return new UnityEngine.Quaternion(0f, 0f, 0f, 1f);
			}
			return new UnityEngine.Quaternion(x / m, y / m, z / m, w / m);
		}

		private static UnityEngine.Vector3 RndV(float scale)
		{
			return new UnityEngine.Vector3(Rnd() * scale, Rnd() * scale, Rnd() * scale);
		}

		// AirfoilTable.Read
		private static float TwinRead(float[] t, float idx)
		{
			if (idx <= 0f)
			{
				return t[0];
			}
			if (idx >= 127f)
			{
				return t[127];
			}
			int num = (int)idx;
			float num2 = idx - (float)num;
			return t[num] + (t[num + 1] - t[num]) * num2;
		}

		// SurfaceEffector.EvalFlowGrid on plain arrays (= lib.rs grid_point)
		private static UnityEngine.Vector3 GridTwin(int l, UnityEngine.Quaternion[] gInv, UnityEngine.Vector3[] gRight, UnityEngine.Vector3[] gArm,
			UnityEngine.Quaternion[] ghInv, UnityEngine.Vector3[] ghRight, bool[] ghZero, float[] cl, float[] cd, float[] hcl, float[] hcd, bool host,
			UnityEngine.Vector3 v, UnityEngine.Vector3 vhat, float num, UnityEngine.Vector3 hv, UnityEngine.Vector3 hvn, bool hvSmall, float hq,
			UnityEngine.Vector3 hostArm, UnityEngine.Vector3 hostBase, UnityEngine.Quaternion rootInv, out float fn)
		{
			fn = 0f;
			UnityEngine.Vector3 vector = gInv[l] * v;
			float idx = UnityEngine.Mathf.Atan2(vector.y, vector.z) * 20.37185f + 64f;
			float CL = TwinRead(cl, idx), CD = TwinRead(cd, idx);
			UnityEngine.Vector3 vector2 = UnityEngine.Vector3.Cross(v, gRight[l]);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return UnityEngine.Vector3.zero;
			}
			UnityEngine.Vector3 rhs2 = -vector2 / magnitude * (CL * num) - vhat * (CD * num);
			UnityEngine.Vector3 mF = UnityEngine.Vector3.Cross(gArm[l], rhs2);
			UnityEngine.Vector3 fTot = rhs2;
			if (host)
			{
				UnityEngine.Vector3 dH = UnityEngine.Vector3.zero;
				if (!ghZero[l])
				{
					UnityEngine.Vector3 vl = ghInv[l] * hv;
					float hidx = UnityEngine.Mathf.Atan2(vl.y, vl.z) * 20.37185f + 64f;
					float hl = TwinRead(hcl, hidx), hd = TwinRead(hcd, hidx);
					UnityEngine.Vector3 n = UnityEngine.Vector3.Cross(hv, ghRight[l]);
					float m = n.magnitude;
					UnityEngine.Vector3 f = (m < 1E-06f || hvSmall) ? UnityEngine.Vector3.zero : (-n / m * (hl * hq) - hvn * (hd * hq));
					dH = f - hostBase;
				}
				mF += UnityEngine.Vector3.Cross(hostArm, dH);
				fTot += dH;
			}
			fn = (rootInv * fTot).y;
			UnityEngine.Vector3 w = rootInv * mF;
			return new UnityEngine.Vector3(0f - w.z, 0f - w.x, w.y);
		}

		private static bool Near(float a, float b)
		{
			float d = Math.Abs(a - b);
			return d <= 0.002f * (1f + Math.Abs(a) + Math.Abs(b));   // NaN fails
		}

		// null = the kernels agree with the C# twin; otherwise what differed
		internal static string SelfTest()
		{
			rng = 0x5030400Au;
			const int n = 31;
			for (int round = 0; round < 6; round++)
			{
				bool host = round % 2 == 1;
				UnityEngine.Quaternion[] gInv = new UnityEngine.Quaternion[n], ghInv = new UnityEngine.Quaternion[n];
				UnityEngine.Vector3[] gRight = new UnityEngine.Vector3[n], gArm = new UnityEngine.Vector3[n], ghRight = new UnityEngine.Vector3[n], outM = new UnityEngine.Vector3[n];
				float[] outFN = new float[n];
				bool[] ghZero = new bool[n];
				float[] cl = new float[128], cd = new float[128], hcl = new float[128], hcd = new float[128];
				for (int i = 0; i < 128; i++)
				{
					float a = (float)(i - 64) * 0.04908734f;
					cl[i] = 1.1f * (float)Math.Sin(2.0 * a) + 0.1f * Rnd();
					cd[i] = 0.02f + 1.2f * (float)(1.0 - Math.Cos(2.0 * a)) * 0.5f + 0.01f * Rnd();
					hcl[i] = 0.9f * (float)Math.Sin(2.0 * a) + 0.1f * Rnd();
					hcd[i] = 0.03f + 1.1f * (float)(1.0 - Math.Cos(2.0 * a)) * 0.5f + 0.01f * Rnd();
				}
				for (int l = 0; l < n; l++)
				{
					UnityEngine.Quaternion q = RndQ();
					gInv[l] = new UnityEngine.Quaternion(0f - q.x, 0f - q.y, 0f - q.z, q.w);
					gRight[l] = q * UnityEngine.Vector3.right;
					gArm[l] = RndV(6f);
					UnityEngine.Quaternion r = RndQ();
					ghInv[l] = new UnityEngine.Quaternion(0f - r.x, 0f - r.y, 0f - r.z, r.w);
					ghRight[l] = r * UnityEngine.Vector3.right;
					ghZero[l] = l == n / 2;
				}
				UnityEngine.Vector3 v = RndV(120f) + new UnityEngine.Vector3(0f, 0f, 60f);
				float sp = v.magnitude;
				UnityEngine.Vector3 vhat = v / sp;
				float num = 0.5f * 1.1f * sp * sp * (4f + Rnd());
				UnityEngine.Vector3 hv = round >= 4 ? v : (RndV(120f) + new UnityEngine.Vector3(0f, 0f, 50f));
				UnityEngine.Vector3 hvn = hv.normalized;
				bool hvSmall = hv.sqrMagnitude < 1E-06f;
				float hq = 0.5f * 1.1f * hv.sqrMagnitude * (6f + Rnd());
				UnityEngine.Vector3 hostArm = RndV(5f), hostBase = RndV(2000f);
				UnityEngine.Quaternion rootInv = RndQ();
				float i00 = 2e-5f * (1f + 0.3f * Rnd()), i02 = 2e-6f * Rnd(), i20 = 2e-6f * Rnd(), i22 = 6e-6f * (1f + 0.3f * Rnd());
				GridIn g = default(GridIn);
				float[] o = new float[6];
				bool ok0, ok1;
				fixed (UnityEngine.Quaternion* pgInv = gInv, pghInv = ghInv)
				fixed (UnityEngine.Vector3* pgRight = gRight, pgArm = gArm, pghRight = ghRight, pOut = outM)
				fixed (bool* pZero = ghZero)
				fixed (float* pcl = cl, pcd = cd, phcl = hcl, phcd = hcd, pFN = outFN)
				{
					g.gInv = pgInv;
					g.gRight = pgRight;
					g.gArm = pgArm;
					g.ghInv = pghInv;
					g.ghRight = pghRight;
					g.ghZero = (byte*)pZero;
					g.cl = pcl;
					g.cd = pcd;
					g.hcl = phcl;
					g.hcd = phcd;
					g.outM = pOut;
					g.outFN = pFN;
					g.n = n;
					g.host = host ? 1 : 0;
					g.hvSmall = hvSmall ? 1 : 0;
					g.v = v;
					g.vhat = vhat;
					g.num = num;
					g.hv = hv;
					g.hvn = hvn;
					g.hq = hq;
					g.hostArm = hostArm;
					g.hostBase = hostBase;
					g.rootInv = rootInv;
					g.ia[0] = i00;
					g.ia[1] = i02;
					g.ia[2] = i20;
					g.ia[3] = i22;
					ok0 = su30_fcs_grid(&g, 0) == 0;
					ok1 = su30_fcs_grid(&g, 1) == 0;
					for (int k = 0; k < 6; k++)
					{
						o[k] = g.o[k];
					}
				}
				if (!ok0 || !ok1)
				{
					return "self-test: the library returned an error (round " + round + ")";
				}
				float pMin = float.MaxValue, pMax = float.MinValue, rMin = float.MaxValue, rMax = float.MinValue, yMin = float.MaxValue, yMax = float.MinValue;
				for (int l = 0; l < n; l++)
				{
					UnityEngine.Vector3 m = GridTwin(l, gInv, gRight, gArm, ghInv, ghRight, ghZero, cl, cd, hcl, hcd, host, v, vhat, num, hv, hvn, hvSmall, hq, hostArm, hostBase, rootInv, out float fnT);
					if (!Near(fnT, outFN[l]))
					{
						return string.Format(System.Globalization.CultureInfo.InvariantCulture, "self-test: normal force {0} of round {1} is {2:G9} in the library, {3:G9} in C#", l, round, outFN[l], fnT);
					}
					if (!Near(m.x, outM[l].x) || !Near(m.y, outM[l].y) || !Near(m.z, outM[l].z))
					{
						return string.Format(System.Globalization.CultureInfo.InvariantCulture, "self-test: moment {0} of round {1} is ({2:G9}, {3:G9}, {4:G9}) in the library, ({5:G9}, {6:G9}, {7:G9}) in C#", l, round, outM[l].x, outM[l].y, outM[l].z, m.x, m.y, m.z);
					}
					pMin = UnityEngine.Mathf.Min(pMin, m.y);
					pMax = UnityEngine.Mathf.Max(pMax, m.y);
					float b = (i00 * m.x + i02 * m.z) * 57.29578f;
					float b2 = (i20 * m.x + i22 * m.z) * 57.29578f;
					rMin = UnityEngine.Mathf.Min(rMin, b);
					rMax = UnityEngine.Mathf.Max(rMax, b);
					yMin = UnityEngine.Mathf.Min(yMin, b2);
					yMax = UnityEngine.Mathf.Max(yMax, b2);
				}
				if (!Near(pMin, o[0]) || !Near(pMax, o[1]) || !Near(rMin, o[2]) || !Near(rMax, o[3]) || !Near(yMin, o[4]) || !Near(yMax, o[5]))
				{
					return string.Format(System.Globalization.CultureInfo.InvariantCulture, "self-test: the ranges of round {0} are {1:G9} {2:G9} {3:G9} {4:G9} {5:G9} {6:G9} in the library, {7:G9} {8:G9} {9:G9} {10:G9} {11:G9} {12:G9} in C#", round, o[0], o[1], o[2], o[3], o[4], o[5], pMin, pMax, rMin, rMax, yMin, yMax);
				}
			}
			return null;
		}

		// the grid kernels' state in words (after Describe has loaded the library, if it is wanted)
		public static string DescribeKernels()
		{
			if (!Wanted || Volatile.Read(ref state) != 1)
			{
				return "grid kernels not used";
			}
			if (Volatile.Read(ref kernels) != 1)
			{
				return "grid kernels off (" + KernelWhy + ")";
			}
			return "grid kernels: sweep " + (Plugin.NativeSweep ? "native" : "C# (NativeSweep off)") + ", tables " + (Plugin.NativeTables ? "native" : "C# (NativeTables off)")
				+ "; coupled-cap solver " + ((Volatile.Read(ref cpl) != 1) ? ("off (" + CplWhy + ")") : (Plugin.NativeCoupledCap ? "native" : "C# (NativeCoupledCap off)"));
		}

		// the state in words, without loading anything that is not going to be used
		public static string Describe()
		{
			string fm = StrictFloat ? "this runtime's float arithmetic is strict 32-bit: the native solve is bit-identical to the C# one"
				: "this runtime carries C# float arithmetic in double (Unity's Mono setting): the native solve is not bit-identical to the C# one here";
			int m = Plugin.NativeAllocator;
			if (!Wanted)
			{
				return "not used (NativeAllocator = " + m + (m == 1 ? ", used only where it is bit-identical" : "") + "); " + fm;
			}
			return (Ready ? ("in use (NativeAllocator = " + m + "), loaded" + (LoadedFrom.Length > 0 ? (" from " + LoadedFrom) : "")) : ("wanted (NativeAllocator = " + m + ") but not usable, the C# solve runs: " + Why)) + "; " + fm;
		}
	}
}
