using UnityEngine;

namespace Su30_VXN.FCS
{
	// 2.2.0: flap-camber coupling. Nuclear Option flies each control surface as its own plate: a deflected rudder changes
	// only its own lift, and the fixed fin ahead of it does not see the camber. On a real flapped section the deflection
	// cambers the whole fin + rudder (or wing + flaperon) section, so the fixed part carries a share of the extra lift
	// (thin-airfoil flap effectiveness ~0.75 for a half-chord rudder, ~0.55 for the ~0.35-chord flaperons). Modelled as
	// the host part's airfoil force at its local AoA plus K x deflection (about the control surface's hinge axis), minus
	// its force at its local AoA - zero with the surface centred, so the trimmed centre of lift does not move.
	// Shared by the game-side CamberForce, the FCS surface models and the bench.
	internal static class Camber
	{
		public static string HostOf(string surfaceName)
		{
			string n = Names.Canon(surfaceName);
			// Su-30 0.1.0: the inner flaperons LFlap_struct / RFlap_struct (flaperon_L/R, 5.0 m2) sit behind the inner wing panels
			// Left_wing_inner / Right_wing_inner (wing_inner_L/R, 5.4 m2); the rudders LRudder_struct / RRudder_struct (rudder_L/R, 1.5 m2)
			// behind the fins Left/Right_vertical_stabiliser (vstab_L/R, 2.0 m2). Host names are canonical (Names.Canon) too.
			if (n.StartsWith("flaperon_"))
			{
				return "wing_inner_" + n.Substring(9);
			}
			if (n.StartsWith("rudder_"))
			{
				return "vstab_" + n.Substring(7);
			}
			// Revoker 0.1.0 / Compass 0.1.0: aileron_L/R sits right behind wingtip_L/R (Compass: aileron 3.0 m2 behind a 3.37 m2 tip)
			// Ternion 0.1.0: the FS-3's trailing edge: flaperon_L/R (inboard, 3.71 m2) behind wing_2b_L/R (4.24 m2), rollervon_L/R
			// (outboard, 3.30 m2) behind wing_3b_L/R (2.81 m2); rudder_L/R (5.29 m2) behind VStab_L/R (10.58 m2)
			if (n.StartsWith("flaperon_"))
			{
				return "wing_2b_" + n.Substring(9);
			}
			if (n.StartsWith("rollervon_"))
			{
				return "wing_3b_" + n.Substring(10);
			}
			if (n.StartsWith("rudder_"))
			{
				return "VStab_" + n.Substring(7);
			}
			if (n.StartsWith("aileron_"))
			{
				return "wingtip_" + n.Substring(8);
			}
			if (n.StartsWith("Rudder_"))
			{
				return "Vstab_" + n.Substring(7);
			}
			// Compass 0.1.0: rudder_L/R hang on the canted fins vstab2_L/R (rudder 1.35 m2 behind a 1.26 m2 fin)
			if (n.StartsWith("rudder_"))
			{
				return "vstab2_" + n.Substring(7);
			}
			if (n.StartsWith("Flap_"))
			{
				return "WingRear_" + n.Substring(5);
			}
			if (n.StartsWith("Aileron_"))
			{
				return "Wingmid_" + n.Substring(8);
			}
			return null;
		}

		public static float K(SurfaceKind kind)
		{
			switch (kind)
			{
			case SurfaceKind.Rudder:
				return Plugin.FinCamberK;
			case SurfaceKind.Flaperon:
				return Plugin.FlapCamberK;
			case SurfaceKind.Aileron:
				return Plugin.AileronCamberK;
			default:
				return 0f;
			}
		}

		public static float KForName(string surfaceName)
		{
			string n = Names.Canon(surfaceName);
			if (n.StartsWith("aileron_") || n.StartsWith("rollervon_"))
			{
				return Plugin.AileronCamberK;
			}
			if (n.StartsWith("flaperon_"))
			{
				return Plugin.FlapCamberK;
			}
			if (n.StartsWith("Rudder_") || n.StartsWith("rudder_"))
			{
				return Plugin.FinCamberK;
			}
			if (n.StartsWith("Flap_") || n.StartsWith("Aileron_"))
			{
				return Plugin.WingCamberK;
			}
			return 0f;
		}

		// lift + airfoil drag of a part with lift-normal rotation rot in flow v (qS = 0.5 rho v^2 S eff)
		public static Vector3 Foil(Vector3 v, Quaternion rot, AirfoilTable t, float qS)
		{
			Vector3 vl = Quaternion.Inverse(rot) * v;
			t.Get(Mathf.Atan2(vl.y, vl.z), out var cl, out var cd);
			Vector3 n = Vector3.Cross(v, rot * Vector3.right);
			float m = n.magnitude;
			if (m < 1E-06f || v.sqrMagnitude < 1E-06f)
			{
				return Vector3.zero;
			}
			return -n / m * (cl * qS) - v.normalized * (cd * qS);
		}

		// F-22E 4.0.1: Foil with the rotation's terms (Quaternion.Inverse(rot), rot * Vector3.right) and the flow's terms
		// (v.normalized, v.sqrMagnitude < 1E-06f) worked out by the caller, for callers that evaluate many rotations in one flow
		// and the same rotations in several flows (SurfaceEffector.EvalFlowGrid). Same expressions in the same order as Foil.
		public static Vector3 FoilPre(Vector3 v, Quaternion rotInv, Vector3 rotRight, AirfoilTable t, float qS, Vector3 vN, bool vSmall)
		{
			Vector3 vl = rotInv * v;
			t.Get(Mathf.Atan2(vl.y, vl.z), out var cl, out var cd);
			Vector3 n = Vector3.Cross(v, rotRight);
			float m = n.magnitude;
			if (m < 1E-06f || vSmall)
			{
				return Vector3.zero;
			}
			return -n / m * (cl * qS) - vN * (cd * qS);
		}

		// host force increment for a control-surface deflection delta (deg) about the world hinge axis
		public static Vector3 Delta(Vector3 v, Quaternion hostRot, Vector3 hingeAxis, float k, float delta, AirfoilTable t, float qS)
		{
			if (k <= 0f || Mathf.Abs(delta) < 0.01f)
			{
				return Vector3.zero;
			}
			Quaternion r1 = Quaternion.AngleAxis(k * delta, hingeAxis) * hostRot;
			return Foil(v, r1, t, qS) - Foil(v, hostRot, t, qS);
		}

		// F-22E 3.1.3: Delta with its undeflected half (Foil(v, hostRot, t, qS)) passed in, for callers that evaluate several
		// deflections in one flow; the caller has applied Delta's zero test (k <= 0 or |delta| < 0.01)
		public static Vector3 DeltaFrom(Vector3 baseF, Vector3 v, Quaternion hostRot, Vector3 hingeAxis, float k, float delta, AirfoilTable t, float qS)
		{
			Quaternion r1 = Quaternion.AngleAxis(k * delta, hingeAxis) * hostRot;
			return Foil(v, r1, t, qS) - baseF;
		}

		// a control surface's current deflection about its hinge (deg), from the visible mesh vs its resting rotation
		public static float DeflectionOf(Transform visible, Quaternion resting)
		{
			Quaternion d = Quaternion.Inverse(resting) * visible.localRotation;
			float a = 2f * Mathf.Atan2(d.x, d.w) * 57.29578f;
			if (a > 180f)
			{
				a -= 360f;
			}
			else if (a < -180f)
			{
				a += 360f;
			}
			return a;
		}
	}
}
