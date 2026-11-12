using System;
using System.Collections.Generic;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// Su-30 0.1.5 (ThrustLineAtCg, Tom 9 Nov: "fix the CG above the thrust"). The Su-30VXN's nozzles sit 0.49 m below its CG, so
	// thrust pitches the jet nose-up about the main wheels on the runway (~128 kN m at full AB against ~160 kN m of weight
	// moment) and changes the pitch trim with throttle in the air. None of Tom's other jets has it (F-22E and Revoker within
	// ~0.1 m, the Ifrit's thrust line 0.65 m above its CG). JetNozzle.Thrust applies each nozzle's force at its thrust
	// transform; for this jet the point is moved, along the jet's vertical axis only, to the CG's height. Fore-aft and
	// side-to-side stay, so the nozzles' vectoring arm and the engines' yaw moment are unchanged; the thrust line simply runs
	// through the CG height instead of 0.49 m below it. The FCS's own nozzle models (TvcEffector / TvcYawEffector) use the
	// same arm through Arm().
	internal static class ThrustLine
	{
		private static readonly Dictionary<Transform, FcsController> map = new Dictionary<Transform, FcsController>();

		// set once the JetNozzle.Thrust transpiler has replaced the force point (0 = another plugin got there first: inactive)
		internal static int Patched;

		internal static bool Active => Patched > 0 && Plugin.ThrustLineAtCg;

		internal static void Register(Transform t, FcsController c)
		{
			if (t != null && c != null)
			{
				map[t] = c;
			}
		}

		// the hook: the point JetNozzle.Thrust applies the force at
		public static Vector3 Point(Transform t)
		{
			Vector3 p = t.position;
			try
			{
				if (Plugin.ThrustLineAtCg && map.TryGetValue(t, out var c) && c != null && c.ac != null && c.HasCg)
				{
					Vector3 up = ((Component)(object)c.ac).transform.up;
					return p - up * Vector3.Dot(p - c.CgWorld, up);
				}
			}
			catch (Exception)
			{
			}
			return p;
		}

		// the thrust arm (application point - CG) the FCS's models use; rootInv = inverse of the jet's root rotation
		internal static Vector3 Arm(Vector3 pos, Vector3 cg, Quaternion rootInv)
		{
			Vector3 r = pos - cg;
			if (!Active)
			{
				return r;
			}
			Vector3 up = Quaternion.Inverse(rootInv) * Vector3.up;
			return r - up * Vector3.Dot(r, up);
		}
	}
}
