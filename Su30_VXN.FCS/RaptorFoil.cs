using UnityEngine;

namespace Su30_VXN.FCS
{
	// Su-30 0.1.0 (RaptorStabFoil): the F-22E FCS's stabilator lift curve on the Su-30's all-moving stabs (Left/Right_stab_struct).
	// Source: Aryx's F22E_Wing airfoil (the F-22E's stab foil; lift peak 1.10 at 35 deg local, 2.67 /rad at 0), copied key for key from
	// the F-22E bench extract (airframe_cal.json). StallStagger remaps it exactly as the F-22E FCS does for its stabs (StallStagger
	// 2.0.0 whole-curve map, peak moved to StabilatorStallAoA 25 deg, StallStaggerLiftSlope 0 = peak CL kept, StabilatorCLScale 1), so the
	// Su-30's stabs fly the Raptor's stab curve (CL 1.10 at 25 deg local, 3.74 /rad at 0) instead of the Su-30_elevators foil.
	// The F-22E's stab effectiveness (StabilatorEffectiveness 1.2, a wing-area multiplier) is ElevatorEffectiveness here.
	internal static class RaptorFoil
	{
		private static Keyframe K(float t, float v, float i, float o)
		{
			return new Keyframe(t, v, i, o);
		}

		public static readonly AnimationCurve Lift = new AnimationCurve(
			K(-3.14159274f, -0f, 1.33690119f, 1.33690119f),
			K(-2.61799383f, 0.699999988f, 0.381971627f, 0.381971627f),
			K(-2.18166161f, 0.449999988f, -0.654808998f, -0.654808998f),
			K(-1.57079637f, -0f, -1.28506243f, -1.28506243f),
			K(-1.13446403f, -0.800000012f, -1.13159156f, -1.13159156f),
			K(-0.785398185f, -0.949999988f, -0.644577503f, -0.644577503f),
			K(-0.610865235f, -1.10000002f, 8.94069672e-08f, 8.94069672e-08f),
			K(-0.436332315f, -0.949999988f, 1.28915524f, 1.28915524f),
			K(-0.261799395f, -0.649999976f, 2.00535226f, 2.00535226f),
			K(-0.130899698f, -0.349999994f, 2.48281693f, 2.48281693f),
			K(0f, 0f, 2.67380285f, 2.67380285f),
			K(0.130899698f, 0.349999994f, 2.48281693f, 2.48281693f),
			K(0.261799395f, 0.649999976f, 2.00535226f, 2.00535226f),
			K(0.436332315f, 0.949999988f, 1.28915524f, 1.28915524f),
			K(0.610865235f, 1.10000002f, 8.94069672e-08f, 8.94069672e-08f),
			K(0.785398185f, 0.949999988f, -0.644577503f, -0.644577503f),
			K(1.13446403f, 0.800000012f, -1.13159156f, -1.13159156f),
			K(1.57079637f, 0f, -1.28506243f, -1.28506243f),
			K(2.18166161f, -0.449999988f, -0.654808998f, -0.654808998f),
			K(2.61799383f, -0.699999988f, 0.381971627f, 0.381971627f),
			K(3.14159274f, 0f, 1.33690119f, 1.33690119f));

		public static readonly AnimationCurve Drag = new AnimationCurve(
			K(-3.14159274f, 0.100000001f, 0.572957695f, 0.572957695f),
			K(-2.61799383f, 0.400000006f, 0.85943675f, 0.85943675f),
			K(-2.18166161f, 0.899999976f, 0.532032371f, 0.532032371f),
			K(-1.57079637f, 0.850000024f, 0.0736659914f, 0.0736659914f),
			K(-1.13446403f, 0.949999988f, -0.243507117f, -0.243507117f),
			K(-0.785398185f, 0.699999988f, -1.07429576f, -1.07429576f),
			K(-0.610865235f, 0.449999988f, -1.40374649f, -1.40374649f),
			K(-0.436332315f, 0.209999993f, -1.05997193f, -1.05997193f),
			K(-0.261799395f, 0.0799999982f, -0.601605654f, -0.601605654f),
			K(-0.130899698f, 0.0199999996f, -0.305577487f, -0.305577487f),
			K(0f, 0f, 0f, 0f),
			K(0.130899698f, 0.0199999996f, 0.305577487f, 0.305577487f),
			K(0.261799395f, 0.0799999982f, 0.601605654f, 0.601605654f),
			K(0.436332315f, 0.209999993f, 1.05997193f, 1.05997193f),
			K(0.610865235f, 0.449999988f, 1.40374649f, 1.40374649f),
			K(0.785398185f, 0.699999988f, 1.07429576f, 1.07429576f),
			K(1.13446403f, 0.949999988f, 0.243507117f, 0.243507117f),
			K(1.57079637f, 0.850000024f, -0.0736659914f, -0.0736659914f),
			K(2.18166161f, 0.899999976f, -0.532032371f, -0.532032371f),
			K(2.61799383f, 0.400000006f, -0.85943675f, -0.85943675f),
			K(3.14159274f, 0.100000001f, -0.572957695f, -0.572957695f));

		public static float Cl(float rad)
		{
			return Lift.Evaluate(Wrap(rad));
		}

		public static float Cd(float rad)
		{
			return Drag.Evaluate(Wrap(rad));
		}

		// the game's curves loop over +-pi (pre / post infinity 2)
		private static float Wrap(float r)
		{
			while (r > Mathf.PI)
			{
				r -= 2f * Mathf.PI;
			}
			while (r < 0f - Mathf.PI)
			{
				r += 2f * Mathf.PI;
			}
			return r;
		}
	}
}
