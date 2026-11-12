using System;
using System.Collections.Generic;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// 2.0.0: staggered stall of the horizontal control surfaces.
	// Aryx gives every horizontal surface the same airfoil (F22E_Wing, lift peak at +/-35 deg local AoA). Each
	// family (stabilators, inner flaperons Flap_*, outer flaperons Aileron_*) gets its own copy of that curve with the
	// peak moved to its own stall AoA. Shape-preserving remap, symmetric on the negative side:
	//   |x| <= P:       CL'(x) = A * CL(x * P0 / P)                  (A = (P/P0)^LiftSlope)
	//   P < |x| <= 90:  CL'(x) = A(x) * CL(P0 + (|x|-P)(90-P0)/(90-P)) (A fades back to 1 at 90)
	//   |x| > 90:       unchanged (reversed flow)
	// LiftSlope 1 keeps the small-angle lift slope (the peak CL scales with the stall AoA, like a real wing whose
	// stall moves); 0 keeps the peak CL (the slope scales instead). Drag follows the same angle map (drag rise moves
	// with the stall), without the amplitude factor.
	// Shared by the plugin and the bench: the game's aero job reads the remapped charts (StallStaggerJobs), the FCS's
	// effector and body models read the same 128-point tables through TableFor.
	internal static class StallStagger
	{
		public enum Fam
		{
			None = -1,
			Stab = 0,
			Inner = 1,
			Outer = 2,
			BodyFore = 3,
			BodyAft = 4,
			// Compass 0.1.1: the wing-root strakes (wingroot1_L/R) with a slender-strake vortex-lift curve (Strake.cs)
			Strake = 5,
			// Compass 0.1.3: the all-moving tail slabs (elevator_L/R) and the fixed hstab_L/R with a low-aspect-ratio swept-slab curve
			Slab = 6,
			// Su-30 0.1.5 (HighAoADragShape): the part's own foil with its drag scaled down past the stall (wing, canard, body foils)
			Drag = 7,
			DragCanard = 8,
			// Su-30 0.2.5 (HighAoAPitchBump): the body-foil sections ahead of / behind the CG with a lift change at 53-70 deg AoA
			DragFore = 9,
			DragAft = 10
		}

		public const int ChartN = 128;

		// Su-30 0.1.4 (StabHighAoAScale): local-AoA window of the stab fade
		internal static float StabHiA0 = 35f;

		internal static float StabHiA1 = 60f;

		public const float ChartStep = 0.04908734f;

		// family of a part whose chart is replaced: a staggered control surface (with StallStagger) or a 2.0.8 body
		// section (with BodyLiftShaping), else None
		// Su-30 0.1.5 (HighAoADragShape): which foils get the drag cut - the Su-30VXN's own wing, canard and body foils. The stabs
		// fly the F-22E's stab curve (RaptorStabFoil) and the fins sit edge-on (no drag to speak of), so both keep theirs.
		public static bool IsDragShapeFoil(string afName)
		{
			return Plugin.HighAoADragShape && Plugin.HighAoADragShapeDepth > 0.001f && !string.IsNullOrEmpty(afName) && (afName == "Su-30_wing" || afName == "Su-30_canards" || afName == "Su-30_body");
		}

		// the drag factor at a local AoA (deg, magnitude; mirrored past 90). Target: the jet's total drag halfway between the Ifrit's
		// and the Revoker's drag per kg in Tom's logs (7 / 8 Nov). Two curves, solved together on the bench (stab and fin drag held)
		// so the static pitch moment at each AoA is unchanged: the canards' drag sits 3.4 m ahead of the CG and is nose-up at high
		// AoA (+29 deg/s^2 at 50 deg, 300 km/h), the wing's mostly behind it and nose-down (-14), so one factor on both left the jet
		// 5-8 deg/s^2 more nose-down at 45-90 deg; the canards are cut less (to ~0.73) and the wing and body more (to ~0.48).
		private static readonly float[] DragA = new float[20] { 0f, 17.5f, 20f, 22.5f, 25f, 27.5f, 30f, 32.5f, 35f, 37.5f, 45f, 50f, 55f, 60f, 65f, 70f, 75f, 80f, 85f, 90f };

		private static readonly float[] DragK = new float[20] { 1f, 1f, 0.954f, 0.907f, 0.807f, 0.71f, 0.628f, 0.548f, 0.52f, 0.493f, 0.483f, 0.476f, 0.515f, 0.567f, 0.598f, 0.635f, 0.673f, 0.699f, 0.718f, 0.735f };

		private static readonly float[] DragKCanard = new float[20] { 1f, 1f, 0.973f, 0.949f, 0.897f, 0.847f, 0.803f, 0.761f, 0.746f, 0.732f, 0.73f, 0.725f, 0.744f, 0.771f, 0.751f, 0.739f, 0.766f, 0.784f, 0.798f, 0.811f };

		public static float DragFactor(float aDeg, bool canard = false)
		{
			float[] K = canard ? DragKCanard : DragK;
			float x = Mathf.Abs(aDeg);
			if (x > 90f)
			{
				x = Mathf.Max(0f, 180f - x);
			}
			float k = K[K.Length - 1];
			for (int i = 1; i < DragA.Length; i++)
			{
				if (x <= DragA[i])
				{
					float t = (x - DragA[i - 1]) / Mathf.Max(0.001f, DragA[i] - DragA[i - 1]);
					k = Mathf.Lerp(K[i - 1], K[i], t);
					break;
				}
			}
			return 1f - Plugin.HighAoADragShapeDepth * (1f - k);
		}

		public static Fam FamilyOf(string name, string afName)
		{
			Fam f = FamilyOf(name);
			if (f == Fam.None && afName == "Su-30_body" && PitchBump.On)
			{
				Fam b = PitchBump.SideOf(name);
				if (b != Fam.None)
				{
					return b;
				}
			}
			if (f == Fam.None && IsDragShapeFoil(afName))
			{
				return (afName == "Su-30_canards") ? Fam.DragCanard : Fam.Drag;
			}
			return f;
		}

		public static Fam FamilyOf(string name)
		{
			Fam f = SurfaceFamilyOf(name);
			// Su-30 0.1.0: the Raptor stab curve is on by its own key, independent of the trailing-edge stagger (off on the Su-30)
			if (f == Fam.Stab && Plugin.RaptorStabFoil)
			{
				return Fam.Stab;
			}
			if (f == Fam.Stab && Plugin.StabSlabFoil)
			{
				return Fam.Slab;
			}
			if (f != Fam.None)
			{
				return Plugin.StallStagger ? f : Fam.None;
			}
			if (Plugin.StrakeVortexLift && Strake.IsStrake(name))
			{
				return Fam.Strake;
			}
			return Plugin.BodyLiftShaping ? BodyLift.FamilyOf(name) : Fam.None;
		}

		public static Fam SurfaceFamilyOf(string name)
		{
			if (string.IsNullOrEmpty(name))
			{
				return Fam.None;
			}
			name = Names.Canon(name);   // Su-30 0.1.0
			// Revoker 0.2.20: the FS-12's trailing-edge elevon parts (lower case): aileron_L/R = outboard, flap_L/R = inboard.
			// The canards keep their own foil (no family).
			// Ternion 0.1.0: the FS-3's two trailing-edge pairs, as the Revoker's elevons: rollervon_L/R = outboard, flaperon_L/R = inboard
			if (name.StartsWith("rollervon_") && !name.Contains("visible"))
			{
				return Fam.Outer;
			}
			if (name.StartsWith("flaperon_") && !name.Contains("visible"))
			{
				return Fam.Inner;
			}
			if (name.StartsWith("aileron_") && !name.Contains("visible"))
			{
				return Fam.Outer;
			}
			if (name.StartsWith("flap_") && !name.Contains("visible"))
			{
				return Fam.Inner;
			}
			if (name.Contains("Elevator"))
			{
				return Fam.Stab;
			}
			// Compass 0.1.0: the T/A-30's tail is elevator_L/R (the moving slabs, 1.8 m2 each) behind the fixed hstab_L/R (0.63 m2):
			// both sit in the wing's wake, so both get the stab downwash
			// Ternion 0.1.0: the FS-3's elevator_L/R keep the game's own foil (Multirole1_wing) and fly in the free stream (no
			// downwash, Tom), so they are not in the stab family
			if (name.StartsWith("hstab_") && !name.Contains("visible") && !name.Contains("liftNormal"))
			{
				return Fam.Stab;
			}
			// Ternion 0.1.1 (ElevatorStallAoA > 0): the aft elevators stall earlier on a knee-preserving curve (no downwash still)
			if ((Plugin.ElevatorStallAoA > 0f || Plugin.RaptorStabFoil) && name.StartsWith("elevator_") && !name.Contains("visible") && !name.Contains("liftNormal"))
			{
				return Fam.Stab;
			}
			if (name.Contains("Aileron"))
			{
				return Fam.Outer;
			}
			if (name.Contains("Flap") && !name.Contains("Servo") && !name.Contains("Drive"))
			{
				return Fam.Inner;
			}
			return Fam.None;
		}

		public static bool Enabled => Plugin.StallStagger || Plugin.BodyLiftShaping || Plugin.StrakeVortexLift || Plugin.StabSlabFoil || Plugin.RaptorStabFoil || Plugin.HighAoADragShape || PitchBump.On;

		public static float PeakFor(Fam f)
		{
			switch (f)
			{
			case Fam.Stab:
				return (Plugin.ElevatorStallAoA > 0f) ? Plugin.ElevatorStallAoA : Plugin.StabStallAoA;
			case Fam.Slab:
				return Plugin.SlabStallAoA;
			case Fam.Inner:
				return Plugin.InnerFlaperonStallAoA;
			case Fam.Outer:
				return Plugin.OuterFlaperonStallAoA;
			default:
				return 0f;
			}
		}

		// cache key: one remapped chart per (source airfoil, peak, slope mode)
		public static string Key(object af, Fam f)
		{
			if (f == Fam.BodyFore || f == Fam.BodyAft)
			{
				return (af == null ? 0 : af.GetHashCode()) + "|" + (int)f + "|" + BodyLift.KeyPart(f);
			}
			if (f == Fam.Strake)
			{
				return "strake|" + Strake.KeyPart();
			}
			if (f == Fam.Stab && Plugin.RaptorStabFoil)
			{
				return "raptorstab|" + PeakFor(f).ToString("F2") + "|" + Plugin.StabHighAoAScale.ToString("F3") + "|" + StabHiA0.ToString("F0") + "|" + StabHiA1.ToString("F0");
			}
			if (f == Fam.Drag || f == Fam.DragCanard)
			{
				return (af == null ? 0 : af.GetHashCode()) + "|drag2|" + (int)f + "|" + Plugin.HighAoADragShapeDepth.ToString("F3");
			}
			if (f == Fam.DragFore || f == Fam.DragAft)
			{
				return (af == null ? 0 : af.GetHashCode()) + "|bump|" + (int)f + "|" + (Plugin.HighAoADragShape ? Plugin.HighAoADragShapeDepth : 0f).ToString("F3") + "|" + PitchBump.KeyPart();
			}
			if (f == Fam.Slab)
			{
				return (af == null ? 0 : af.GetHashCode()) + "|slab|" + Plugin.SlabStallAoA.ToString("F2") + "|" + Plugin.SlabLiftSlope.ToString("F2");
			}
			return (af == null ? 0 : af.GetHashCode()) + "|" + (int)f + "|" + PeakFor(f).ToString("F2") + "|knee" + ((f == Fam.Stab) ? ("|" + Plugin.ElevatorStallDragShift + "|" + Plugin.ElevatorPlateauCL.ToString("F3")) : "");
		}

		// peak AoA (deg, magnitude) of a lift curve on one side (sign +1 / -1), searched 2..89 deg
		public static float FindPeakDeg(Func<float, float> clOfRad, int sign)
		{
			float best = 0f, bestA = 35f;
			for (float a = 2f; a <= 89f; a += 0.25f)
			{
				float c = sign * clOfRad(sign * a * Mathf.Deg2Rad);
				if (c > best)
				{
					best = c;
					bestA = a;
				}
			}
			return bestA;
		}

		// Revoker 0.2.22: the knee of a lift curve on one side (deg, magnitude): the first local minimum of the lift slope after its
		// steepest point, below the peak, if that minimum is under 25 % of the steepest slope (Fighter1_wing: the 17.19 deg keyframe,
		// slope 0.15 against 3.8 /rad, recovering to ~0.4 above it); else where the slope first falls under 15 % of it; 0 if neither
		// (no knee: the whole curve below the peak is compressed).
		public static float FindKneeDeg(Func<float, float> clOfRad, int sign, float peakDeg)
		{
			const float h = 0.25f;
			int n = Mathf.FloorToInt((peakDeg - 0.5f) / h);
			if (n < 4)
			{
				return 0f;
			}
			float[] sl = new float[n];
			for (int i = 0; i < n; i++)
			{
				float a = 0.5f + i * h;
				sl[i] = sign * (clOfRad(sign * (a + h) * Mathf.Deg2Rad) - clOfRad(sign * (a - h) * Mathf.Deg2Rad)) / (2f * h);
			}
			int iMax = 0;
			for (int i = 1; i < n; i++)
			{
				if (sl[i] > sl[iMax])
				{
					iMax = i;
				}
			}
			float sMax = sl[iMax];
			if (sMax <= 0f)
			{
				return 0f;
			}
			for (int i = iMax + 1; i < n - 1; i++)
			{
				if (sl[i] <= sl[i - 1] && sl[i] < sl[i + 1] && sl[i] < 0.25f * sMax)
				{
					return 0.5f + i * h;
				}
			}
			for (int i = iMax + 1; i < n; i++)
			{
				if (sl[i] < 0.15f * sMax)
				{
					return 0.5f + i * h;
				}
			}
			return 0f;
		}

		// Revoker 0.2.22: knee-preserving angle map for one side. Below the source curve's knee the curve is untouched; the plateau
		// from the knee to the source peak P0 is compressed (or stretched) linearly onto knee..P; past P the post-peak side runs
		// from P0 to 90 as before. Lift amplitude 1 everywhere: the peak CL is kept and nothing below the knee changes. 0.2.20-0.2.21
		// compressed the whole curve (and with it the 17 deg knee, to 12-14 deg): roll authority -6..-30 % at 10 deg and ~-23 %
		// at 15 deg for the inboard pair, and a lift loss above the knee.
		public static void MapKnee(float x, float P, float P0, float knee, out float m)
		{
			P = Mathf.Clamp(P, 5f, 80f);
			float k = Mathf.Min(knee, Mathf.Min(P, P0) - 2f);
			if (k < 0f)
			{
				k = 0f;
			}
			if (x <= k)
			{
				m = x;
			}
			else if (x <= P)
			{
				m = k + (x - k) * (P0 - k) / Mathf.Max(0.01f, P - k);
			}
			else if (x <= 90f)
			{
				float t = (x - P) / (90f - P);
				m = P0 + t * (90f - P0);
			}
			else
			{
				m = x;
			}
		}

		// angle map for one side: x = |AoA| (deg), returns the source |AoA| (deg) and the lift amplitude
		public static void Map(float x, float P, float P0, float slope, out float m, out float amp)
		{
			P = Mathf.Clamp(P, 5f, 80f);
			float aP = Mathf.Pow(P / P0, Mathf.Clamp01(slope));
			if (x <= P)
			{
				m = x * P0 / P;
				amp = aP;
			}
			else if (x <= 90f)
			{
				float t = (x - P) / (90f - P);
				m = P0 + t * (90f - P0);
				// Revoker 0.2.20: the amplitude holds aP past the peak and blends back to 1 only over 80-90 deg. A linear fade from
				// the peak (the F-22E's form, harmless at LiftSlope 0 where aP = 1) outran the flat-topped wing foil's post-peak
				// lift drop at LiftSlope 1 and moved the peak: inboard 32 -> 39.4 deg on Fighter1_wing.
				float tf = Mathf.Clamp01((x - 80f) / 10f);
				tf = tf * tf * (3f - 2f * tf);
				amp = aP + (1f - aP) * tf;
			}
			else
			{
				m = x;
				amp = 1f;
			}
		}

		// fills 128-point lift and drag charts (the game's sampling: index i <-> (i - 64) * ChartStep rad)
		public static void BuildCharts(Func<float, float> clOfRad, Func<float, float> cdOfRad, float P, float slope, float[] cl, float[] cd, bool stab = false)
		{
			float p0Pos = FindPeakDeg(clOfRad, 1);
			float p0Neg = FindPeakDeg(clOfRad, -1);
			float kPos = FindKneeDeg(clOfRad, 1, p0Pos);
			float kNeg = FindKneeDeg(clOfRad, -1, p0Neg);
			for (int i = 0; i < ChartN; i++)
			{
				float a = (i - 64) * ChartStep;
				float xd = Mathf.Abs(a) * Mathf.Rad2Deg;
				int s = (a < 0f) ? -1 : 1;
				MapKnee(xd, P, (s > 0) ? p0Pos : p0Neg, (s > 0) ? kPos : kNeg, out float m);
				float amp = 1f;
				float src = s * m * Mathf.Deg2Rad;
				if (xd > 90f)
				{
					src = a;
				}
				cl[i] = amp * clOfRad(src);
				cd[i] = cdOfRad(src);
				if (stab && xd <= 90f)
				{
					// Ternion 0.1.4: the elevators' drag stays on the source angle (ElevatorStallDragShift false), so moving the lift peak to
					// 30 deg does not pull the drag rise in with it (0.1.2-0.1.3: CD 0.60 at 30 deg local, normal force +8-25 % at 25-60 deg);
					// ElevatorPlateauCL scales the lift above the knee (dev test, 1 = off)
					if (!Plugin.ElevatorStallDragShift)
					{
						cd[i] = cdOfRad(a);
					}
					float kS = (s > 0) ? kPos : kNeg;
					if (Plugin.ElevatorPlateauCL != 1f && xd > kS)
					{
						float w = Mathf.Clamp01((xd - kS) / 5f) * (1f - Mathf.Clamp01((xd - 70f) / 20f));
						cl[i] *= 1f + (Plugin.ElevatorPlateauCL - 1f) * w;
					}
				}
			}
		}

		// Compass 0.1.3 (StabSlabFoil): the T/A-30's slabs fly the game's Trainer_stabilator foil, a copy of the straight wing's
		// Trainer_wing (lift slope 5.0 /rad, CL 1.0 at 17 deg, 0.8 at 23). A low-aspect-ratio swept slab (AR ~3, ~40 deg sweep) has a
		// lower slope and stalls much later at a higher CLmax: ~3.5 /rad, CLmax ~1.2 at ~30 deg. With the wing foil the slab is
		// at its CLmax by 17 deg local AoA, so at 15-25 deg body AoA (the tail ~4-5 deg less in the downwash) the elevators had no
		// nose-down left past neutral: both went to the stop for nothing, and pushing from the 35 deg limit got only what moving
		// back from the trim to neutral gives. Whole-curve map (StallStagger.Map, not the knee map): |x| <= P reads the source at
		// x * P0 / P scaled by (P / P0)^slope, past P the post-stall side runs P0..90 over P..90. Symmetric. Drag follows the angle map.
		public static void BuildSlabCharts(Func<float, float> clOfRad, Func<float, float> cdOfRad, float P, float slope, float[] cl, float[] cd)
		{
			float p0Pos = FindPeakDeg(clOfRad, 1);
			float p0Neg = FindPeakDeg(clOfRad, -1);
			for (int i = 0; i < ChartN; i++)
			{
				float a = (i - 64) * ChartStep;
				float xd = Mathf.Abs(a) * Mathf.Rad2Deg;
				int sg = (a < 0f) ? -1 : 1;
				Map(xd, P, (sg > 0) ? p0Pos : p0Neg, slope, out float m, out float amp);
				float src = sg * m * Mathf.Deg2Rad;
				if (xd > 90f)
				{
					src = a;
					amp = 1f;
				}
				cl[i] = amp * clOfRad(src);
				cd[i] = cdOfRad(src);
			}
		}

		// the chart for a family: the stall remap for a control surface, the lift multiplier for a body section
		public static void BuildFor(Fam f, Func<float, float> clOfRad, Func<float, float> cdOfRad, float[] cl, float[] cd)
		{
			if (f == Fam.BodyFore || f == Fam.BodyAft)
			{
				BodyLift.BuildCharts(f, clOfRad, cdOfRad, cl, cd);
				return;
			}
			if (f == Fam.Strake)
			{
				Strake.BuildCharts(cl, cd);
				return;
			}
			if (f == Fam.Drag || f == Fam.DragCanard)
			{
				for (int i = 0; i < ChartN; i++)
				{
					float a = (i - 64) * ChartStep;
					cl[i] = clOfRad(a);
					cd[i] = cdOfRad(a) * DragFactor(a * Mathf.Rad2Deg, f == Fam.DragCanard);
				}
				return;
			}
			if (f == Fam.DragFore || f == Fam.DragAft)
			{
				for (int i = 0; i < ChartN; i++)
				{
					float a = (i - 64) * ChartStep;
					cl[i] = clOfRad(a) + PitchBump.DeltaCL(f, a * Mathf.Rad2Deg);
					cd[i] = cdOfRad(a) * (Plugin.HighAoADragShape ? DragFactor(a * Mathf.Rad2Deg) : 1f);
				}
				return;
			}
			if (f == Fam.Slab)
			{
				BuildSlabCharts(clOfRad, cdOfRad, Plugin.SlabStallAoA, Plugin.SlabLiftSlope, cl, cd);
				return;
			}
			// Su-30 0.1.0 (RaptorStabFoil): the stabs fly the F-22E FCS's stab curve - Aryx's F22E_Wing foil (not the part's own) on the
			// F-22E's whole-curve map (StallStagger.BuildCharts there = Map here) with the peak at ElevatorStallAoA (25, the F-22E's
			// StabilatorStallAoA) and lift slope mode 0 (peak CL kept, the F-22E's StallStaggerLiftSlope); drag follows the angle map
			if (f == Fam.Stab && Plugin.RaptorStabFoil)
			{
				BuildSlabCharts(RaptorFoil.Cl, RaptorFoil.Cd, PeakFor(f), 0f, cl, cd);
				// Su-30 0.1.4 (StabHighAoAScale, Tom: pitch-neutral at mid to high AoA): the stab's lift and drag past StabHiA0 local AoA
				// fade to StabHighAoAScale of the curve by StabHiA1 (and mirrored in reversed flow). Up there a deflection hardly moves
				// the stab's normal force (it is mostly drag), so this takes away the stab's nose-down at high AoA, not its control.
				float sH = Plugin.StabHighAoAScale;
				if (sH < 0.999f)
				{
					for (int i = 0; i < ChartN; i++)
					{
						float xd = Mathf.Abs((i - 64) * ChartStep) * Mathf.Rad2Deg;
						if (xd > 90f)
						{
							xd = 180f - xd;
						}
						float t = Mathf.Clamp01((xd - StabHiA0) / Mathf.Max(1f, StabHiA1 - StabHiA0));
						t = t * t * (3f - 2f * t);
						float k = Mathf.Lerp(1f, sH, t);
						cl[i] *= k;
						cd[i] *= k;
					}
				}
				return;
			}
			BuildCharts(clOfRad, cdOfRad, PeakFor(f), Plugin.StallStaggerLiftSlope, cl, cd, f == Fam.Stab);
		}

		private static readonly Dictionary<string, AirfoilTable> tables = new Dictionary<string, AirfoilTable>();

		// the FCS's table for an AeroPart: the remapped one for a staggered control surface, else the airfoil's own
		public static AirfoilTable TableFor(AeroPart part, Airfoil af)
		{
			Fam f = (part != null && Enabled && af != null && af.liftCoef != null && af.dragCoef != null) ? FamilyOf(part.gameObject.name, af.name) : Fam.None;
			return TableForFamily(af, f);
		}

		// Revoker 0.2.20: a family's remapped table for any airfoil. Used for the camber increment a trailing-edge elevon puts on
		// the wing section ahead of it: the increment is that elevon's circulation, so it stalls (and reverses) with the elevon,
		// while the section's own clean lift keeps its own curve.
		public static AirfoilTable TableForFamily(Airfoil af, Fam f)
		{
			if (f == Fam.None || af == null || af.liftCoef == null || af.dragCoef == null)
			{
				return new AirfoilTable(af);
			}
			string k = Key(af, f);
			if (!tables.TryGetValue(k, out AirfoilTable t))
			{
				float[] cl = new float[ChartN];
				float[] cd = new float[ChartN];
				BuildFor(f, r => af.liftCoef.Evaluate(r), r => af.dragCoef.Evaluate(r), cl, cd);
				t = new AirfoilTable(cl, cd);
				tables[k] = t;
			}
			return t;
		}
	}
}
