using UnityEngine;

namespace Su30_VXN.FCS
{
	// Compass 0.1.1: slender-strake vortex lift for the T/A-30's wing-root strakes (wingroot1_L/R, the jet's LERX).
	// In the game the strakes fly the wing's own foil (Trainer_wing: lift-curve slope 5.0 /rad, stall at 17 deg, CL 0.8-1.0 past it), so
	// they add a flat ~8 deg/s^2 of nose-up above the stall and nothing more. A real strake / LEX is a slender, sharp-edged surface whose
	// lift is mostly leading-edge vortex lift (Polhamus): a low attached-flow slope and a nonlinear vortex term that keeps building to
	// 35-40 deg before the vortex breaks down - that is what lets the F/A-18's or the F-35's all-moving tail work past its own travel.
	// Normal force, Polhamus leading-edge suction analogy:
	//   CN = Kp sin a cos a + Kv sin^2 a,   Kp = pi AR / 2 (AR ~ 1 strake: 1.57),   Kv = pi (sharp leading edge)
	// faded to a flat plate (CN = 1.2 sin a) from VortexBreakdownStart to VortexBreakdownEnd; reversed flow (|a| > 90) is a flat plate.
	// CL = CN cos a, CD = CD0 + CN sin a. Odd in a for lift, even for drag. Chart layout as StallStagger (128 points, -pi..pi).
	// The strake area is the part's wingArea x LerxEffectiveness (Registry); this curve replaces only the strakes' chart.
	internal static class Strake
	{
		public const float Kp = 1.5708f;

		public const float Kv = 3.1416f;

		public const float CD0 = 0.01f;

		public const float FlatPlateCN = 1.2f;

		// Compass 0.1.2: the breakdown window and the post-breakdown plate are settings (StrakeBreakdownStart / End / StrakeResidualCN)
		public static float VortexBreakdownStart => Plugin.StrakeBreakdownStart;

		public static float VortexBreakdownEnd => Mathf.Max(Plugin.StrakeBreakdownStart + 1f, Plugin.StrakeBreakdownEnd);

		public static float ResidualCN => Plugin.StrakeResidualCN;

		public static bool IsStrake(string name)
		{
			if (string.IsNullOrEmpty(name) || name.Contains("visible") || name.Contains("liftNormal"))
			{
				return false;
			}
			// Su-30 0.1.4: the Su-30VXN's LERX (Cheek_L/R -> shoulder_L/R)
			return name.StartsWith("wingroot1_") || Names.Canon(name).StartsWith("shoulder_");
		}

		public static string KeyPart()
		{
			return Kp.ToString("F3") + "|" + Kv.ToString("F3") + "|" + VortexBreakdownStart.ToString("F1") + "|" + VortexBreakdownEnd.ToString("F1") + "|" + ResidualCN.ToString("F2");
		}

		// normal-force coefficient for |a| in [0, pi]
		public static float CN(float aAbs)
		{
			if (aAbs > Mathf.PI * 0.5f)
			{
				return ResidualCN * Mathf.Sin(aAbs);
			}
			float s = Mathf.Sin(aAbs);
			float c = Mathf.Cos(aAbs);
			float vortex = Kp * s * c + Kv * s * s;
			float flat = ResidualCN * s;
			float d = aAbs * Mathf.Rad2Deg;
			if (d <= VortexBreakdownStart)
			{
				return vortex;
			}
			float t = Mathf.Clamp01((d - VortexBreakdownStart) / (VortexBreakdownEnd - VortexBreakdownStart));
			t = t * t * (3f - 2f * t);
			return Mathf.Lerp(vortex, flat, t);
		}

		public static void Coeffs(float a, out float cl, out float cd)
		{
			float aAbs = Mathf.Abs(a);
			float cn = CN(aAbs);
			cl = Mathf.Sign(a) * cn * Mathf.Cos(aAbs);
			cd = CD0 + cn * Mathf.Sin(aAbs);
		}

		public static void BuildCharts(float[] cl, float[] cd)
		{
			for (int i = 0; i < StallStagger.ChartN; i++)
			{
				float a = (i - 64) * StallStagger.ChartStep;
				Coeffs(a, out cl[i], out cd[i]);
			}
		}
	}
}
