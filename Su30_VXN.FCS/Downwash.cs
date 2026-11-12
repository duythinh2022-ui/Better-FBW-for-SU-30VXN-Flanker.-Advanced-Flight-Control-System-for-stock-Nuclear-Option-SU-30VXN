using System.Collections.Generic;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// 2.0.8: wing/LERX downwash at the stabilators. Nuclear Option gives every AeroPart the free-stream flow, so a stab
	// at 0 deflection sees the body AoA 1:1 and is stalled at any AoA above its peak. Real tails sit in the wing's
	// downwash: the flow reaching them is turned down by eps = k * alpha (k = d eps / d alpha, 0.3-0.5 on fighters).
	// Apply turns a part's airflow velocity about the aircraft's pitch axis so its AoA in the aircraft's symmetry plane
	// drops from alpha to alpha - eps; sideslip and speed are kept. eps = k * alpha up to FadeStart (45), then fades
	// smoothly to 0 at FadeEnd (60): above that the stabs see the free stream as in 2.0.7. Bench: kept to 90 deg it cost
	// a fifth of the post-stall yaw rate at 65 deg AoA (falling leaf), because the differential stab then works on the
	// shoulder of its lift curve instead of deep in stall.
	// Shared by the game-side correction force (DownwashForce), the FCS's stab models and the bench.
	internal static class Downwash
	{
		// Su-30 0.1.4: settings (DownwashFadeStart / End); 45 / 60 = the F-22E's
		public static float FadeStart => Plugin.DownwashFadeStart;

		public static float FadeEnd => Mathf.Max(Plugin.DownwashFadeStart + 1f, Plugin.DownwashFadeEnd);

		// Compass 0.1.0: lift-driven downwash. eps follows the wing's circulation, eps = k x CL_wing(alpha) / CLalpha_wing(0),
		// instead of k x alpha: the same below the wing's stall (17 deg on Trainer_wing), and past it eps tracks the wing's
		// post-stall lift (CL 0.8-1.0 from 23 to 45 deg) instead of growing with the AoA. Faded out from 60 to 90 deg, where the
		// wake no longer crosses the tail plane. WingTable is set at spawn (Registry) and by the bench; null = the linear model.
		public static AirfoilTable WingTable;

		private static float wingSlope = -1f;

		public const float LiftFadeStart = 60f;

		public const float LiftFadeEnd = 90f;

		public static void SetWing(AirfoilTable t)
		{
			WingTable = t;
			wingSlope = -1f;
			if (t != null)
			{
				// small-angle lift slope (per rad) from a +-2 deg central difference
				t.Get(2f * Mathf.Deg2Rad, out var c1, out var _);
				t.Get(-2f * Mathf.Deg2Rad, out var c0, out var _);
				wingSlope = (c1 - c0) / (4f * Mathf.Deg2Rad);
			}
		}

		public static float Eps(float alphaDeg)
		{
			float k = Plugin.StabDownwash;
			if (k <= 0f)
			{
				return 0f;
			}
			if (Plugin.DownwashLiftDriven && WingTable != null && wingSlope > 0.1f)
			{
				float ax = Mathf.Abs(alphaDeg);
				if (ax >= LiftFadeEnd)
				{
					return 0f;
				}
				WingTable.Get(alphaDeg * Mathf.Deg2Rad, out var cl, out var _);
				float e = k * cl / wingSlope * Mathf.Rad2Deg;
				if (ax > LiftFadeStart)
				{
					float tt = (ax - LiftFadeStart) / (LiftFadeEnd - LiftFadeStart);
					e *= 1f - tt * tt * (3f - 2f * tt);
				}
				return e;
			}
			float x = Mathf.Abs(alphaDeg);
			if (x >= FadeEnd)
			{
				return 0f;
			}
			float w = 1f;
			if (x > FadeStart)
			{
				float t = (x - FadeStart) / (FadeEnd - FadeStart);
				w = 1f - t * t * (3f - 2f * t);
			}
			return k * alphaDeg * w;
		}

		private static readonly Dictionary<int, bool> stabCache = new Dictionary<int, bool>();

		// a stabilator AeroPart (cached: gameObject.name allocates)
		public static bool IsStab(AeroPart ap)
		{
			if (ap == null || Plugin.StabDownwash <= 0f)
			{
				return false;
			}
			int id = ap.GetInstanceID();
			if (!stabCache.TryGetValue(id, out bool r))
			{
				r = StallStagger.SurfaceFamilyOf(ap.gameObject.name) == StallStagger.Fam.Stab;
				stabCache[id] = r;
			}
			return r;
		}

		public static bool IsStabName(string name)
		{
			return Plugin.StabDownwash > 0f && StallStagger.SurfaceFamilyOf(name) == StallStagger.Fam.Stab;
		}

		// eps (deg) for a reference airflow (the aircraft's own air velocity, world): the wing's AoA in the symmetry plane
		public static float EpsFor(Vector3 vRef, Quaternion rootRot)
		{
			if (Plugin.StabDownwash <= 0f)
			{
				return 0f;
			}
			Vector3 b = Quaternion.Inverse(rootRot) * vRef;
			if (b.y * b.y + b.z * b.z < 1f)
			{
				return 0f;
			}
			return Eps(Mathf.Atan2(0f - b.y, b.z) * 57.29578f);
		}

		// turns a part's airflow v (world) by eps (deg) about the aircraft's pitch axis: its AoA in the symmetry plane
		// drops by eps, sideslip and speed are kept. The stab's own pitch-rate AoA is kept (eps comes from the wing's
		// AoA, not the stab's), so the tail's pitch damping is not cut.
		public static Vector3 Turn(Vector3 v, Quaternion rootRot, float eps)
		{
			if (eps == 0f)
			{
				return v;
			}
			Vector3 b = Quaternion.Inverse(rootRot) * v;
			float r = Mathf.Sqrt(b.y * b.y + b.z * b.z);
			if (r < 1f)
			{
				return v;
			}
			float a2 = Mathf.Atan2(0f - b.y, b.z) - eps * (Mathf.PI / 180f);
			b.y = (0f - Mathf.Sin(a2)) * r;
			b.z = Mathf.Cos(a2) * r;
			return rootRot * b;
		}

		// the flow v turned by the downwash of a reference flow (for a flow that is its own reference, vRef = v)
		public static Vector3 Apply(Vector3 v, Quaternion rootRot, Vector3 vRef)
		{
			return Turn(v, rootRot, EpsFor(vRef, rootRot));
		}
	}
}
