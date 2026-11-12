using System;

namespace Su30_VXN.FCS
{
	internal static class AsymCap
	{
		// Ifrit 0.4.6: the scan for the cap starts at the AoA of least lateral usage, not at 0. On this jet the usage is
		// highest at LOW AoA (no rudders: the canted tails' and the splits' yaw grows with AoA, while the thrust's yaw about
		// an offset CoM does not), so a scan from 0 met the threshold at the first sample and returned the floor - the 65 <-> 12
		// snap of the 23 Oct 22:48 / 23:34 logs, a cap that pushed the jet into the AoA where the load is hardest to hold.
		// Lowering the AoA only helps above the least-usage AoA, so the cap is the first crossing above it. The least-usage
		// AoA is the lowest AoA whose usage is within StartBand of the side's minimum, so a curve that is lowest (or nearly)
		// at 0 - every F-22E-style curve, rising with AoA - starts at 0 exactly as before, hump rule included. If the usage
		// is over the threshold everywhere, the cap is that AoA (not lower than the floor).
		public static float StartBand = 0.15f;

		// Ifrit 0.4.8: the cap never sits inside the flat stretch of the usage curve. From the start AoA (above) the usage is
		// followed upward while it stays within FlatBand of the side's minimum; the cap is not set below the end of that
		// stretch. Cutting the AoA inside it buys less than FlatBand of usage, so it is not worth the AoA (24 Oct 22:17 log,
		// 66.8 s and 72.5 s: usage 1.05-1.2 from 40 to 65 deg at 43 m/s, the whole curve just over AsymCapDemand 1.08, and
		// the 0.4.6 'over everywhere' branch dropped the cap 65 -> 40 for a usage change of ~0.1; with the curve back under
		// 1.08 at 65 it returned to 65). A curve that actually rises - with AoA past the flat stretch, or a hump - is capped
		// where it crosses, as before.
		public static float FlatBand = 0.15f;

		// debug (positive side, last call): start AoA, end of the flat stretch, side minimum, usage at the limit
		public static float DbgStart, DbgFlat, DbgUMin, DbgULim;

		// Su-30 0.5.0 (CplTopEdge): the cap from the coupled curve. On this jet that curve is not monotonic: with a lateral CG offset
		// the weaker side's share of the roll range is lowest at 10-30 deg AoA (a hump), recovers to 40-50 deg (a plateau), then
		// collapses past 50-55 (the cliff). Find's first crossing from zero put the cap at 12 deg the moment the hump touched the
		// threshold (bench, 16 cm, nozzles locked, afterburner, 65 m/s: 51 -> 12 deg in one sweep, the nose pushed from 50 deg to 0
		// through the hump, to an AoA with no more roll toward the weak side than at 45 deg). A cap can only take the jet down, so:
		//   - the threshold is uMax, or the lowest demand anywhere between the floor and the limit plus FlatBand if that is higher
		//     (where the whole usable range is over uMax, cutting AoA buys nothing until the curve rises FlatBand above its best);
		//   - the cap is the lower edge of the stretch that is over that threshold all the way up to the limit (scanned down from
		//     the limit): a hump below a stretch that is under the threshold does not set it;
		//   - a plateau that is itself over the threshold, above a hump, is kept (the cap goes to the cliff above it, at the
		//     threshold + FlatBand) until it is more than FlatBand over; then the cap is the strict lower edge, under the hump, and it
		//     stays there until the plateau is back within half of FlatBand ('low', one latch per side: the one place the cap moves
		//     in a step, and it does so once).
		// A curve that rises with AoA (every curve of the Compass and the Ternion) gives the same cap as Find.
		public static float FindTop(float[] alphas, float[] u, int n, int side, float limit, float floor, float uMax, ref bool low)
		{
			float lim = Math.Abs(limit);
			if (n < 2)
			{
				low = false;
				return limit;
			}
			float flo = Math.Min(Math.Abs(floor), lim);
			float uRef = Math.Min(Interp(alphas, u, n, side * flo), Interp(alphas, u, n, side * lim));
			for (int i = 0; i < n; i++)
			{
				float ai = side * alphas[i];
				if (ai > flo && ai < lim)
				{
					uRef = Math.Min(uRef, u[i]);
				}
			}
			float thr = Math.Max(uMax, uRef + FlatBand);
			float capA = TopEdge(alphas, u, n, side, lim, flo, thr, flo);
			// the lowest point of the over-threshold stretch above capA, and whether a higher point (a hump) lies under it
			int kMin = -1;
			float uLow = float.MaxValue;
			bool hump = false;
			float uBelowMax = float.MinValue;
			for (int k = 0; k < n; k++)
			{
				int i = (side > 0) ? k : (n - 1 - k);
				float ai = side * alphas[i];
				if (ai <= capA || ai >= lim)
				{
					continue;
				}
				if (u[i] <= uLow)
				{
					uLow = u[i];
					kMin = i;
					hump = uBelowMax > u[i] + 0.02f;
				}
				uBelowMax = Math.Max(uBelowMax, u[i]);
			}
			if (kMin < 0 || !hump)
			{
				low = false;
				return side * capA;
			}
			low = low ? !(uLow < thr + 0.5f * FlatBand) : (uLow > thr + FlatBand);
			if (low)
			{
				return side * capA;
			}
			return side * TopEdge(alphas, u, n, side, lim, flo, Math.Max(thr + FlatBand, uLow), side * alphas[kMin]);
		}

		// the lower edge of the stretch over thr that reaches up to lim, scanned down from lim, not below aStop
		private static float TopEdge(float[] alphas, float[] u, int n, int side, float lim, float flo, float thr, float aStop)
		{
			float aHi = lim;
			float uHi = Interp(alphas, u, n, side * lim);
			if (uHi <= thr)
			{
				return lim;
			}
			for (int k = 0; k < n; k++)
			{
				int i = (side > 0) ? (n - 1 - k) : k;
				float ai = side * alphas[i];
				if (ai >= lim)
				{
					continue;
				}
				if (ai < aStop)
				{
					break;
				}
				if (u[i] <= thr)
				{
					return Math.Max(flo, Cross(ai, u[i], aHi, uHi, thr));
				}
				aHi = ai;
				uHi = u[i];
			}
			float uF = Interp(alphas, u, n, side * aStop);
			return (uF <= thr) ? Math.Max(flo, Cross(aStop, uF, aHi, uHi, thr)) : Math.Max(flo, aStop);
		}

		public static float Find(float[] alphas, float[] u, int n, int side, float limit, float floor, float uMax)
		{
			float r = FindCrossing(alphas, u, n, side, limit, floor, uMax);
			if (n < 2)
			{
				return r;
			}
			float lim0 = Math.Abs(limit);
			float aStart = StartAoA(alphas, u, n, side, lim0);
			float uMin = SideMin(alphas, u, n, side, lim0);
			float aFlat = FlatEnd(alphas, u, n, side, lim0, aStart, uMin + FlatBand);
			if (side > 0)
			{
				DbgStart = aStart;
				DbgFlat = aFlat;
				DbgUMin = uMin;
				DbgULim = Interp(alphas, u, n, lim0);
			}
			return side * Math.Min(lim0, Math.Max(Math.Abs(r), aFlat));
		}

		private static float SideMin(float[] alphas, float[] u, int n, int side, float lim)
		{
			float uMin = Interp(alphas, u, n, 0f);
			for (int i = 0; i < n; i++)
			{
				float ai = side * alphas[i];
				if (ai > 0f && ai <= lim)
				{
					uMin = Math.Min(uMin, u[i]);
				}
			}
			return Math.Min(uMin, Interp(alphas, u, n, side * lim));
		}

		// the |AoA| where the usage, followed up from aStart, first rises above uTop (lim if it never does)
		private static float FlatEnd(float[] alphas, float[] u, int n, int side, float lim, float aStart, float uTop)
		{
			float aPrev = aStart;
			float uPrev = Interp(alphas, u, n, side * aStart);
			if (uPrev > uTop)
			{
				return aStart;
			}
			for (int k = 0; k < n; k++)
			{
				int i = (side > 0) ? k : (n - 1 - k);
				float ai = side * alphas[i];
				if (ai <= aStart)
				{
					continue;
				}
				if (ai > lim)
				{
					ai = lim;
				}
				float ui = Interp(alphas, u, n, side * ai);
				if (ui > uTop)
				{
					return Cross(aPrev, uPrev, ai, ui, uTop);
				}
				aPrev = ai;
				uPrev = ui;
				if (ai >= lim)
				{
					break;
				}
			}
			return lim;
		}

		private static float FindCrossing(float[] alphas, float[] u, int n, int side, float limit, float floor, float uMax)
		{
			if (n < 2)
			{
				return limit;
			}
			float lim0 = Math.Abs(limit);
			float flo0 = Math.Min(Math.Abs(floor), lim0);
			float aStart = StartAoA(alphas, u, n, side, lim0);
			if (aStart > 0f)
			{
				float us = Interp(alphas, u, n, side * aStart);
				if (us > uMax)
				{
					return side * Math.Max(flo0, aStart);
				}
				float aPrev = aStart;
				float uPrev = us;
				if (side > 0)
				{
					for (int i = 0; i < n; i++)
					{
						float ai = alphas[i];
						if (ai <= aStart)
						{
							continue;
						}
						if (ai > lim0)
						{
							ai = lim0;
						}
						float ui = (ai == alphas[i]) ? u[i] : Interp(alphas, u, n, ai);
						if (ui > uMax)
						{
							return Math.Max(flo0, Cross(aPrev, uPrev, ai, ui, uMax));
						}
						aPrev = ai;
						uPrev = ui;
						if (ai >= lim0)
						{
							break;
						}
					}
					return lim0;
				}
				for (int i = n - 1; i >= 0; i--)
				{
					float ai = 0f - alphas[i];
					if (ai <= aStart)
					{
						continue;
					}
					if (ai > lim0)
					{
						ai = lim0;
					}
					float ui = (ai == 0f - alphas[i]) ? u[i] : Interp(alphas, u, n, 0f - ai);
					if (ui > uMax)
					{
						return 0f - Math.Max(flo0, Cross(aPrev, uPrev, ai, ui, uMax));
					}
					aPrev = ai;
					uPrev = ui;
					if (ai >= lim0)
					{
						break;
					}
				}
				return 0f - lim0;
			}
			float num = Math.Abs(limit);
			float val = Math.Min(Math.Abs(floor), num);
			float a = 0f;
			float u2 = Interp(alphas, u, n, 0f);
			if (side > 0)
			{
				for (int i = 0; i < n; i++)
				{
					float num2 = alphas[i];
					if (!(num2 <= 0f))
					{
						if (num2 > num)
						{
							num2 = num;
						}
						float num3 = ((num2 == alphas[i]) ? u[i] : Interp(alphas, u, n, num2));
						if (num3 > uMax)
						{
							return Math.Max(val, Cross(a, u2, num2, num3, uMax));
						}
						a = num2;
						u2 = num3;
						if (num2 >= num)
						{
							break;
						}
					}
				}
				return num;
			}
			for (int num4 = n - 1; num4 >= 0; num4--)
			{
				float num5 = 0f - alphas[num4];
				if (!(num5 <= 0f))
				{
					if (num5 > num)
					{
						num5 = num;
					}
					float num6 = ((num5 == 0f - alphas[num4]) ? u[num4] : Interp(alphas, u, n, 0f - num5));
					if (num6 > uMax)
					{
						return 0f - Math.Max(val, Cross(a, u2, num5, num6, uMax));
					}
					a = num5;
					u2 = num6;
					if (num5 >= num)
					{
						break;
					}
				}
			}
			return 0f - num;
		}

		// the lowest |AoA| on this side (0 .. lim) whose usage is within StartBand of the side's minimum
		public static float StartAoA(float[] alphas, float[] u, int n, int side, float lim)
		{
			float uMin = Interp(alphas, u, n, 0f);
			for (int i = 0; i < n; i++)
			{
				float ai = side * alphas[i];
				if (ai > 0f && ai <= lim)
				{
					uMin = Math.Min(uMin, u[i]);
				}
			}
			uMin = Math.Min(uMin, Interp(alphas, u, n, side * lim));
			if (Interp(alphas, u, n, 0f) <= uMin + StartBand)
			{
				return 0f;
			}
			if (side > 0)
			{
				for (int i = 0; i < n; i++)
				{
					if (alphas[i] > 0f && alphas[i] <= lim && u[i] <= uMin + StartBand)
					{
						return alphas[i];
					}
				}
			}
			else
			{
				for (int i = n - 1; i >= 0; i--)
				{
					if (alphas[i] < 0f && -alphas[i] <= lim && u[i] <= uMin + StartBand)
					{
						return -alphas[i];
					}
				}
			}
			return lim;
		}

		private static float Cross(float a0, float u0, float a1, float u1, float thr)
		{
			if (u0 >= thr)
			{
				return a0;
			}
			float val = (thr - u0) / Math.Max(1E-06f, u1 - u0);
			return a0 + (a1 - a0) * Math.Min(1f, Math.Max(0f, val));
		}

		public static float Interp(float[] alphas, float[] u, int n, float a)
		{
			if (n == 0)
			{
				return 0f;
			}
			if (a <= alphas[0])
			{
				return u[0];
			}
			if (a >= alphas[n - 1])
			{
				return u[n - 1];
			}
			for (int i = 1; i < n; i++)
			{
				if (a <= alphas[i])
				{
					float num = (a - alphas[i - 1]) / (alphas[i] - alphas[i - 1]);
					return u[i - 1] + (u[i] - u[i - 1]) * num;
				}
			}
			return u[n - 1];
		}

		public static float Slew(float eff, float target, int side, float downRate, float upRate, float dt)
		{
			float num = (((target - eff) * (float)side > 0f) ? upRate : downRate) * dt;
			if (Math.Abs(target - eff) <= num)
			{
				return target;
			}
			return eff + (float)Math.Sign(target - eff) * num;
		}
	}
}
