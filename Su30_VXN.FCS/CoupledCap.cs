using System;

namespace Su30_VXN.FCS
{
	// Su-30 (CoupledCap, from the Ternion 0.2.2): the asymmetric-load AoA cap judged on the coupled problem.
	// Su-30 0.5.6: the solve also in the native core (Ternion 0.3.0's NativeCoupledCap), plus the one-side Reserve (Su-30 0.5.0).
	//
	// Before (0.2.1 and every other jet): at each sweep AoA the load's roll and yaw were divided by the sums of each surface's
	// roll and yaw half-ranges, taken one axis at a time, at zero sideslip and with no pitch trim; the pitch margin was the stabs
	// alone at full nose-down. So a surface pinned holding pitch still counted its full roll travel, the roll surfaces' own yaw
	// (adverse at high AoA) and the body yaw a velocity-vector roll needs at 45 deg were not in it, and the nose-down margin
	// ignored that the same surfaces were busy holding the load.
	//
	// Now, at each sweep AoA, every surface is a set of points (its grid, CplPts of them) with its full 3-axis acceleration at that
	// AoA, and the question is asked of the attainable set those points span (convex combinations per surface: the standard
	// attainable-moment-set relaxation; exact at the vertices, so the stall peaks are kept):
	//   A(side): with the load in, pitch held (no nose-up acceleration at positive AoA, no nose-down at negative) and no sideslip
	//            acceleration (sin a p' - cos a r' = 0), the most velocity-vector roll acceleration (cos a p' + sin a r') toward
	//            that side. The reserve is the weaker side against the mean of the two (the clean jet's figure).
	//   B:       with the load's roll and yaw held (p' = r' = 0), the most nose-down (positive AoA) / nose-up (negative AoA) pitch
	//            acceleration. This replaces the stabs-only nose-down margin for the asymmetric pitch cap.
	//
	// Each is a small LP with one convexity row per surface and two axis rows. The simplex starts from a basis that already holds
	// one point per surface (the best one for the objective), so only the two axis rows need artificials; both sides of A share
	// that phase 1.
	internal sealed unsafe class CplPoint
	{
		internal const int MaxCol = 420;
		internal const int MaxEff = 18;
		internal const int IterCap = 400;

		public readonly float[] R = new float[MaxCol];
		public readonly float[] Q = new float[MaxCol];
		public readonly float[] Y = new float[MaxCol];
		public readonly float[] G = new float[MaxCol];   // debug: the point's deflection (deg)
		public readonly double[] solP = new double[MaxCol + 1], solN = new double[MaxCol + 1];   // debug: last Reserve's optimal mixes
		public static bool KeepSolution;
		// Ternion 0.3.0: this sweep point's scratch for one surface's point moments (FcsController.AsymSweepPoint; one CplPoint per point)
		public readonly UnityEngine.Vector3[] scratchM = new UnityEngine.Vector3[64];
		// Su-30 0.5.6: the second surface of a pair the LP takes as one effector (the canards)
		public readonly UnityEngine.Vector3[] scratchM2 = new UnityEngine.Vector3[64];
		public readonly int[] effStart = new int[MaxEff + 1];
		private readonly int[] effOf = new int[MaxCol + 1];
		public int nCol;
		public int nEff;
		public bool overflow;

		// LP workspace: rows 0..nEff-1 convexity, nEff and nEff+1 the axis rows, nEff+2 the objective
		private const int W0 = MaxCol + 1 + 2 + 1;
		private readonly double[] T = new double[(MaxEff + 3) * W0];
		private readonly double[] T1 = new double[(MaxEff + 3) * W0];
		private readonly int[] basis = new int[MaxEff + 2];
		private readonly int[] basis1 = new int[MaxEff + 2];
		private readonly double[] a0 = new double[MaxCol + 1];
		private readonly double[] a1 = new double[MaxCol + 1];
		private readonly double[] cc = new double[MaxCol + 1];
		private int n;
		private int m;
		private int W;

		public int Iters;

		public void Reset()
		{
			nCol = 0;
			nEff = 0;
			effStart[0] = 0;
			overflow = false;
		}

		public void Add(float r, float q, float y, float gDbg = 0f)
		{
			if (nCol >= MaxCol)
			{
				overflow = true;
				return;
			}
			R[nCol] = r;
			Q[nCol] = q;
			Y[nCol] = y;
			G[nCol] = gDbg;
			nCol++;
		}

		// close the current effector (its points are those added since the last close)
		public void EndEff()
		{
			if (nEff >= MaxEff)
			{
				overflow = true;
				return;
			}
			if (nCol > effStart[nEff])
			{
				for (int j = effStart[nEff]; j < nCol; j++)
				{
					effOf[j] = nEff;
				}
				nEff++;
				effStart[nEff] = nCol;
			}
		}

		// A: the most velocity-vector roll acceleration toward each side (rp: +, rn: -), pitch held (sgnA = +1: q' <= 0; -1: q' >= 0),
		// no sideslip acceleration. bR/bQ/bY: the airframe's (load's) own accelerations. false: no such point (pitch or lateral).
		// side (Su-30 0.5.0, everywhere from the native port): 0 both sides; +1 / -1 that side only (the other figure stays NaN);
		// 2 phase 1 only (the feasibility both sides share; both figures NaN, the bool = feasible)
		public bool Reserve(float sinA, float cosA, float bR, float bQ, float bY, int sgnA, out float rp, out float rn, int side = 0)
		{
			// Ternion 0.3.0 (NativeCoupledCap): the same solve in the native core (native/fcs_core/src/cpl.rs); the C# method below is
			// the reference and the fall-back. With NativeAllocatorCheck 2 both run and the C# result is used.
			if (!KeepSolution && FcsNative.CplOn)
			{
				bool okN = ReserveNative(sinA, cosA, bR, bQ, bY, sgnA, out float rpN, out float rnN, out bool fell, side);
				if (!fell)
				{
					if (Plugin.NativeAllocatorCheck < 2)
					{
						rp = rpN;
						rn = rnN;
						return okN;
					}
					bool okC = ReserveCs(sinA, cosA, bR, bQ, bY, sgnA, out rp, out rn, side);
					FcsNative.CplCheck(okN == okC && Same(rpN, rp) && Same(rnN, rn));
					return okC;
				}
			}
			return ReserveCs(sinA, cosA, bR, bQ, bY, sgnA, out rp, out rn, side);
		}

		private static bool Same(float x, float y)
		{
			// a failed solve leaves NaN (the callers test for it, never read its bits): C#'s float.NaN and the library's differ in the sign bit
			return (float.IsNaN(x) && float.IsNaN(y)) || BitConverter.ToInt32(BitConverter.GetBytes(x), 0) == BitConverter.ToInt32(BitConverter.GetBytes(y), 0);
		}

		private unsafe bool Native(int op, int sgn, float sinA, float cosA, float bR, float bQ, float bY, out FcsNative.CplIo io, int side = 0)
		{
			io = default(FcsNative.CplIo);
			fixed (float* pR = R, pQ = Q, pY = Y)
			fixed (int* pS = effStart, pO = effOf, pB = basis, pB1 = basis1)
			fixed (double* pT = T, pT1 = T1, pA0 = a0, pA1 = a1, pC = cc)
			{
				io.r = pR;
				io.q = pQ;
				io.y = pY;
				io.effStart = pS;
				io.effOf = pO;
				io.t = pT;
				io.t1 = pT1;
				io.basis = pB;
				io.basis1 = pB1;
				io.a0 = pA0;
				io.a1 = pA1;
				io.cc = pC;
				io.nCol = nCol;
				io.nEff = nEff;
				io.op = op;
				io.sgn = sgn;
				io.sinA = sinA;
				io.cosA = cosA;
				io.bR = bR;
				io.bQ = bQ;
				io.bY = bY;
				io.side = side;
				fixed (FcsNative.CplIo* pio = &io)
				{
					if (!FcsNative.Cpl(pio))
					{
						return false;
					}
				}
			}
			Iters += io.iters;
			return true;
		}

		// fell: the library's answer may not be used (the caller runs the C# method)
		internal bool ReserveNative(float sinA, float cosA, float bR, float bQ, float bY, int sgnA, out float rp, out float rn, out bool fell, int side = 0)
		{
			fell = !Native(0, sgnA, sinA, cosA, bR, bQ, bY, out FcsNative.CplIo io, side);
			rp = io.rp;
			rn = io.rn;
			return !fell && io.ok != 0;
		}

		internal bool LateralFeasibleNative(float sinA, float cosA, float bR, float bY, out bool fell)
		{
			fell = !Native(1, 0, sinA, cosA, bR, 0f, bY, out FcsNative.CplIo io);
			return !fell && io.ok != 0;
		}

		internal float PitchHeldNative(int dir, float bR, float bQ, float bY, out bool fell)
		{
			fell = !Native(2, dir, 0f, 0f, bR, bQ, bY, out FcsNative.CplIo io);
			return fell ? float.NaN : io.rp;
		}

		internal bool ReserveCs(float sinA, float cosA, float bR, float bQ, float bY, int sgnA, out float rp, out float rn, int side = 0)
		{
			rp = (rn = float.NaN);
			// columns: the points, then the pitch slack (its own column, outside every surface)
			n = nCol + 1;
			for (int j = 0; j < nCol; j++)
			{
				a0[j] = Q[j];
				a1[j] = sinA * R[j] - cosA * Y[j];
				cc[j] = -(cosA * R[j] + sinA * Y[j]);   // side +: minimise -(velocity-vector roll)
			}
			a0[nCol] = (sgnA >= 0) ? 1.0 : -1.0;
			a1[nCol] = 0.0;
			cc[nCol] = 0.0;
			double b0 = -bQ;
			double b1 = -(sinA * bR - cosA * bY);
			double lin = cosA * bR + sinA * bY;
			if (!Phase1(b0, b1))
			{
				return false;
			}
			if (side == 2)
			{
				return true;
			}
			if (side != 0)
			{
				if (!Phase2((side > 0) ? 1.0 : -1.0, out double objS))
				{
					return false;
				}
				if (side > 0)
				{
					if (KeepSolution) Sol(solP);
					rp = (float)(-objS + lin);
				}
				else
				{
					if (KeepSolution) Sol(solN);
					rn = (float)(-objS - lin);
				}
				return true;
			}
			Array.Copy(T, T1, (m + 1) * W);
			Array.Copy(basis, basis1, m);
			if (!Phase2(1.0, out double objP))
			{
				return false;
			}
			if (KeepSolution) Sol(solP);
			Array.Copy(T1, T, (m + 1) * W);
			Array.Copy(basis1, basis, m);
			if (!Phase2(-1.0, out double objN))
			{
				return false;
			}
			if (KeepSolution) Sol(solN);
			rp = (float)(-objP + lin);
			rn = (float)(-objN - lin);
			return true;
		}

		// the lateral problem alone (no pitch row): can the load's sideslip acceleration be cancelled at all?
		public bool LateralFeasible(float sinA, float cosA, float bR, float bY)
		{
			if (!KeepSolution && FcsNative.CplOn)
			{
				bool okN = LateralFeasibleNative(sinA, cosA, bR, bY, out bool fell);
				if (!fell)
				{
					if (Plugin.NativeAllocatorCheck < 2)
					{
						return okN;
					}
					bool okC = LateralFeasibleCs(sinA, cosA, bR, bY);
					FcsNative.CplCheck(okN == okC);
					return okC;
				}
			}
			return LateralFeasibleCs(sinA, cosA, bR, bY);
		}

		internal bool LateralFeasibleCs(float sinA, float cosA, float bR, float bY)
		{
			n = nCol + 1;
			for (int j = 0; j < nCol; j++)
			{
				a0[j] = 0.0;
				a1[j] = sinA * R[j] - cosA * Y[j];
				cc[j] = 0.0;
			}
			a0[nCol] = 1.0;   // the first row is free (0 = slack)
			a1[nCol] = 0.0;
			cc[nCol] = 0.0;
			return Phase1(0.0, -(sinA * bR - cosA * bY));
		}

		// B: with the load's roll and yaw held (p' = r' = 0), the most nose-down (dir = -1) / nose-up (dir = +1) pitch acceleration
		// (deg/s^2, nose-up +). NaN if the load cannot be held.
		public float PitchHeld(int dir, float bR, float bQ, float bY)
		{
			if (!KeepSolution && FcsNative.CplOn)
			{
				float qN = PitchHeldNative(dir, bR, bQ, bY, out bool fell);
				if (!fell)
				{
					if (Plugin.NativeAllocatorCheck < 2)
					{
						return qN;
					}
					float qC = PitchHeldCs(dir, bR, bQ, bY);
					FcsNative.CplCheck(Same(qN, qC));
					return qC;
				}
			}
			return PitchHeldCs(dir, bR, bQ, bY);
		}

		internal float PitchHeldCs(int dir, float bR, float bQ, float bY)
		{
			n = nCol;
			for (int j = 0; j < nCol; j++)
			{
				a0[j] = R[j];
				a1[j] = Y[j];
				cc[j] = (dir < 0) ? Q[j] : (0f - Q[j]);
			}
			if (!Phase1(-bR, -bY) || !Phase2(1.0, out double obj))
			{
				return float.NaN;
			}
			return (float)(((dir < 0) ? obj : (0.0 - obj)) + bQ);
		}

		// builds the tableau from the basis {one point per surface (the best for cc), artificials on the two axis rows} and drives
		// the artificials out. Columns n.. n+1 are the artificials, n+2 the right-hand side.
		private bool Phase1(double b0, double b1)
		{
			m = nEff + 2;
			W = n + 3;
			int rhs = W - 1;
			int ra = nEff;
			int rb = nEff + 1;
			Array.Clear(T, 0, (m + 1) * W);
			double s0 = b0;
			double s1 = b1;
			for (int e = 0; e < nEff; e++)
			{
				int best = effStart[e];
				for (int j = effStart[e] + 1; j < effStart[e + 1]; j++)
				{
					if (cc[j] < cc[best])
					{
						best = j;
					}
				}
				basis[e] = best;
				int ro = e * W;
				for (int j = effStart[e]; j < effStart[e + 1]; j++)
				{
					T[ro + j] = 1.0;
				}
				T[ro + rhs] = 1.0;
				s0 -= a0[best];
				s1 -= a1[best];
			}
			double g0 = (s0 < 0.0) ? -1.0 : 1.0;
			double g1 = (s1 < 0.0) ? -1.0 : 1.0;
			int r0o = ra * W;
			int r1o = rb * W;
			for (int j = 0; j < n; j++)
			{
				int e = (j < nCol) ? effOf[j] : -1;
				double sub0 = (e >= 0) ? a0[basis[e]] : 0.0;
				double sub1 = (e >= 0) ? a1[basis[e]] : 0.0;
				T[r0o + j] = g0 * (a0[j] - sub0);
				T[r1o + j] = g1 * (a1[j] - sub1);
			}
			T[r0o + n] = 1.0;
			T[r1o + n + 1] = 1.0;
			T[r0o + rhs] = g0 * s0;
			T[r1o + rhs] = g1 * s1;
			basis[ra] = n;
			basis[rb] = n + 1;
			double bScale = 1.0 + Math.Abs(b0) + Math.Abs(b1);
			if (Math.Abs(s0) + Math.Abs(s1) <= 1e-9 * bScale)
			{
				// already feasible at the starting basis
				return DropArtificials();
			}
			int zo = m * W;
			for (int j = 0; j < n; j++)
			{
				T[zo + j] = -(T[r0o + j] + T[r1o + j]);
			}
			T[zo + n] = 0.0;
			T[zo + n + 1] = 0.0;
			T[zo + rhs] = -(T[r0o + rhs] + T[r1o + rhs]);
			if (Run(n + 2) == 2)
			{
				return false;
			}
			if (-T[zo + rhs] > 1e-7 * bScale)
			{
				return false;
			}
			return DropArtificials();
		}

		private void Sol(double[] x)
		{
			Array.Clear(x, 0, x.Length);
			int rhs = W - 1;
			for (int i = 0; i < m; i++)
			{
				if (basis[i] < n) x[basis[i]] = T[i * W + rhs];
			}
		}

		private bool DropArtificials()
		{
			for (int i = 0; i < m; i++)
			{
				if (basis[i] < n)
				{
					continue;
				}
				int ro = i * W;
				for (int j = 0; j < n; j++)
				{
					if (Math.Abs(T[ro + j]) > 1e-9)
					{
						Pivot(i, j);
						break;
					}
				}
			}
			return true;
		}

		// minimise sgn * cc . x from the present (feasible) basis
		private bool Phase2(double sgn, out double obj)
		{
			obj = 0.0;
			int zo = m * W;
			int rhs = W - 1;
			for (int j = 0; j < W; j++)
			{
				T[zo + j] = (j < n) ? (sgn * cc[j]) : 0.0;
			}
			for (int i = 0; i < m; i++)
			{
				int bj = basis[i];
				double cb = (bj < n) ? (sgn * cc[bj]) : 0.0;
				if (cb != 0.0)
				{
					int ro = i * W;
					for (int j = 0; j < W; j++)
					{
						T[zo + j] -= cb * T[ro + j];
					}
				}
			}
			if (Run(n) == 2)
			{
				return false;
			}
			obj = -T[zo + rhs];
			return true;
		}

		private int Run(int nAllowed)
		{
			int zo = m * W;
			int rhs = W - 1;
			int it = 0;
			while (true)
			{
				if (++it > IterCap)
				{
					Iters += it;
					return 2;
				}
				bool bland = it > 80;
				int e = -1;
				double best = -1e-9;
				for (int j = 0; j < nAllowed; j++)
				{
					double rc = T[zo + j];
					if (rc < best)
					{
						e = j;
						if (bland)
						{
							break;
						}
						best = rc;
					}
				}
				if (e < 0)
				{
					Iters += it;
					return 0;
				}
				int r = -1;
				double ratio = double.MaxValue;
				for (int i = 0; i < m; i++)
				{
					double a = T[i * W + e];
					if (a > 1e-10)
					{
						double q = T[i * W + rhs] / a;
						if (q < ratio - 1e-12 || (q < ratio + 1e-12 && r >= 0 && basis[i] < basis[r]))
						{
							ratio = q;
							r = i;
						}
					}
				}
				if (r < 0)
				{
					Iters += it;
					return 2;
				}
				Pivot(r, e);
			}
		}

		private void Pivot(int r, int e)
		{
			int ro = r * W;
			double pv = T[ro + e];
			for (int j = 0; j < W; j++)
			{
				T[ro + j] /= pv;
			}
			for (int i = 0; i <= m; i++)
			{
				if (i == r)
				{
					continue;
				}
				int io = i * W;
				double f = T[io + e];
				if (f == 0.0)
				{
					continue;
				}
				for (int j = 0; j < W; j++)
				{
					T[io + j] -= f * T[ro + j];
				}
			}
			basis[r] = e;
		}
	}
}
