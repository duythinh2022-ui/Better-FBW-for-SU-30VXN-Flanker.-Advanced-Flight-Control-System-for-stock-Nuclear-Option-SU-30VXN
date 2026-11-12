using UnityEngine;

namespace Su30_VXN.FCS
{
	// Su-30 0.2.5 (HighAoAPitchBump; Tom 10 Nov 15:52: "change the lift curve so there will be more pitch up enough that it covers
	// the 55-65 while not affecting anything else").
	// Why: on the calibrated bench (eqerr on the 15:44 log, 72.6-74.3 s: bench vs game pitch acceleration within ~5 deg/s^2) the jet's
	// best nose-up pitch acceleration (nozzles +15, the best stab / canard pose, q = 0, T/W 1.3) falls from +66 deg/s^2 at 55 deg AoA
	// to +33 at 57.5 and stays at +31-42 to 70 (450 km/h; the drop grows with dynamic pressure: 10 at 250 km/h, 58 at 600). The stabs at
	// +35 (the best pose to 55 deg) stall past 55, and the best pose moves to -30, which the stabs reach only through the worst-pitch
	// middle of their travel (stabs 0, canards -25: -31 deg/s^2 at 57.5). In the game: full aft + roll from 550 km/h, pitch rate
	// 70 -> 0 deg/s at 56-58 deg for ~1.3 s while the left stab went +35 -> -25.
	// What: the Su-30_body foil sections ahead of the CG (Front_cover, Cheek_L/R, Mid_cover) get extra lift at 53-70 deg AoA and the
	// ones behind it (intake / engine covers, engines, tail cone) the same total lift less, so the change is a pure pitching couple:
	// no net lift, no drag change (the drag charts are untouched). The lift change is K x w(a) / cos(a): w ramps in 53 -> 57.5 deg, is
	// 1 from 57.5 to 65 and ramps out 65 -> 70 deg (smoothstep); 1 / cos keeps the couple's nose-up even across the band (a lift
	// force's moment arm goes with cos a). Positive AoA only: negative AoA and reversed flow keep the stock curve.
	internal static class PitchBump
	{
		internal static float A0 = 53f, A1 = 57.5f, A2 = 65f, A3 = 70f;

		public static bool On => Plugin.HighAoAPitchBump > 0.0001f;

		// the body sections each side of the CG (the bench's per-part dump, 60 deg / 450 km/h: fore x 4.65-8.21 m, aft x -0.58..-6.7 m)
		public static StallStagger.Fam SideOf(string name)
		{
			if (string.IsNullOrEmpty(name))
			{
				return StallStagger.Fam.None;
			}
			string n = name.Replace("(Clone)", "").Trim();
			switch (n)
			{
			case "Front_cover":
			case "Cheek_L":
			case "Cheek_R":
			case "Mid_cover":
				return StallStagger.Fam.DragFore;
			case "Left_intake and engine cover":
			case "Right_intake and engine cover":
			case "L-engine":
			case "R-engine":
			case "Su_30_tail":
				return StallStagger.Fam.DragAft;
			default:
				return StallStagger.Fam.None;
			}
		}

		// lifting areas each side (m^2) as the game flies them: the cheeks carry LerxEffectiveness, the aft parts AftBodyLift
		public static float ForeArea => 0.5f + 2f * 0.7f * Plugin.LerxEffectiveness + 2.1f;

		public static float AftArea => (2f * 3f + 2f * 1f + 0.5f) * Mathf.Max(0.05f, Plugin.AftBodyLift);

		public static float W(float aDeg)
		{
			if (aDeg <= A0 || aDeg >= A3)
			{
				return 0f;
			}
			float up = Mathf.Clamp01((aDeg - A0) / (A1 - A0));
			float dn = Mathf.Clamp01((aDeg - A2) / (A3 - A2));
			up = up * up * (3f - 2f * up);
			dn = dn * dn * (3f - 2f * dn);
			return up * (1f - dn);
		}

		// aDeg is the chart's angle. The body sections' charts run the other way from the jet's AoA: a positive (nose-up) AoA is a
		// negative chart angle, where the stock body foil's lift coefficient is negative (bench per-part dump at +60 deg AoA: cl -0.50,
		// the lift up). So the jet's +53..+70 deg is chart -53..-70, and more lift there is a more negative coefficient.
		public static float DeltaCL(StallStagger.Fam f, float aDeg)
		{
			if (aDeg >= 0f)
			{
				return 0f;
			}
			float x = -aDeg;
			float w = W(x);
			if (w <= 0f)
			{
				return 0f;
			}
			float d = Plugin.HighAoAPitchBump * w / Mathf.Max(0.2f, Mathf.Cos(x * Mathf.Deg2Rad));
			return f == StallStagger.Fam.DragFore ? -d : d * ForeArea / AftArea;
		}

		public static string KeyPart()
		{
			return Plugin.HighAoAPitchBump.ToString("F3") + "|" + A0.ToString("F1") + "|" + A1.ToString("F1") + "|" + A2.ToString("F1") + "|" + A3.ToString("F1") + "|" + ForeArea.ToString("F2") + "|" + AftArea.ToString("F2");
		}
	}
}
