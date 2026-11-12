using System.Collections.Generic;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// Su-30 0.1.0 (CantedTvc): the AL-31FP's real thrust vectoring. Each nozzle swivels about ONE axis, +-max (15) deg, in a plane
	// tilted cant (32) deg outboard from vertical: a V seen from behind (Flight International, "Russians reveal multi-axis
	// nozzles": "realigned to operate at a 32 deg outward incline from the vertical plane"; "single-axis movement").
	//
	// A nozzle deflected by d (deg, + = jet up = thrust tilted down = nose-up moment) on side s (+1 left, -1 right) points its
	// thrust along (s sin d sin c, -sin d cos c, cos d) in the nozzle mount's frame. The game sets the vectoring transform's
	// localEulerAngles = (x, y, 0) (Turbojet.FixedUpdate), which turns forward into (cos x sin y, -sin x, cos x cos y), so
	//   x = asin(sin d cos c),   y = atan2(s sin d sin c, cos d)          (exact, no small-angle approximation)
	// With d on both nozzles the side parts cancel: pure pitch, 12.8 deg of it at 15. With d on one and -d on the other the pitch
	// parts cancel and the side parts add: both nozzles push the tail the same way (yaw) while their vertical parts make a roll
	// in the same direction (proverse). There is no independent yaw.
	//
	// The FCS states its command as a symmetric part and a differential part, d_i = sym + s_i x diff (diff > 0: left nozzle up,
	// right down = roll left and yaw left). Each nozzle slews toward its own d_i at TvcSlewRate (65 deg/s), and the result is
	// written to the transform by the Turbojet transpiler (Hooks.SetNozzleEuler) in place of the game's own angles. The FCS's TVC
	// effectors (TvcEffector, TvcDiffEffector) build their moment tables from the same geometry.
	internal sealed class CantedTvc
	{
		public readonly List<Turbojet> engines;

		public readonly float[] side;

		public readonly float[] dNow;

		public readonly float max;

		private readonly float[] lastStamp;

		public float cant;

		private readonly Dictionary<int, int> index = new Dictionary<int, int>();

		public CantedTvc(List<Turbojet> engines, Transform root, float max)
		{
			this.engines = engines;
			this.max = Mathf.Max(0.5f, max);
			cant = Plugin.TvcCantAngle;
			side = new float[engines.Count];
			dNow = new float[engines.Count];
			lastStamp = new float[engines.Count];
			for (int k = 0; k < lastStamp.Length; k++)
			{
				lastStamp[k] = float.NegativeInfinity;
			}
			Quaternion rootInv = (root != null) ? Quaternion.Inverse(root.rotation) : Quaternion.identity;
			for (int i = 0; i < engines.Count; i++)
			{
				Turbojet e = engines[i];
				if (e == null)
				{
					continue;
				}
				index[e.GetInstanceID()] = i;
				Transform[] vt = Acc.TF_vecT(e);
				Transform t = (vt != null && vt.Length > 0 && vt[0] != null) ? vt[0] : e.transform;
				float x = (root != null) ? (rootInv * (t.position - root.position)).x : t.localPosition.x;
				side[i] = (x < 0f) ? 1f : -1f;
			}
		}

		public bool Valid
		{
			get
			{
				if (engines.Count < 2)
				{
					return false;
				}
				bool l = false, r = false;
				for (int i = 0; i < side.Length; i++)
				{
					if (side[i] > 0f)
					{
						l = true;
					}
					else
					{
						r = true;
					}
				}
				return l && r;
			}
		}

		public int IndexOf(Turbojet e)
		{
			return (e != null && index.TryGetValue(e.GetInstanceID(), out int i)) ? i : -1;
		}

		// the nozzle transform's local Euler angles for deflection d (deg) on side s
		public static Vector3 Euler(float d, float s, float cantDeg)
		{
			float dr = d * Mathf.Deg2Rad;
			float c = cantDeg * Mathf.Deg2Rad;
			float sd = Mathf.Sin(dr);
			float x = Mathf.Asin(Mathf.Clamp(sd * Mathf.Cos(c), -1f, 1f)) * Mathf.Rad2Deg;
			float y = Mathf.Atan2(s * sd * Mathf.Sin(c), Mathf.Cos(dr)) * Mathf.Rad2Deg;
			return new Vector3(x, y, 0f);
		}

		public Vector3 EulerOf(int i, float d)
		{
			return Euler(Mathf.Clamp(d, 0f - max, max), side[i], cant);
		}

		// the symmetric and differential parts of the nozzles' present deflections
		public float SymNow
		{
			get
			{
				float s = 0f;
				int n = 0;
				for (int i = 0; i < dNow.Length; i++)
				{
					if (engines[i] != null)
					{
						s += dNow[i];
						n++;
					}
				}
				return (n > 0) ? (s / n) : 0f;
			}
		}

		public float DiffNow
		{
			get
			{
				float s = 0f;
				int n = 0;
				for (int i = 0; i < dNow.Length; i++)
				{
					if (engines[i] != null)
					{
						s += side[i] * dNow[i];
						n++;
					}
				}
				return (n > 0) ? (s / n) : 0f;
			}
		}

		// one nozzle: slew toward sym + side x diff at the slew rate, return its transform angles
		public Vector3 Step(int i, float sym, float diff, float dt, float stamp)
		{
			// one slew per physics tick per nozzle, however many vectoring transforms ask
			if (stamp == lastStamp[i])
			{
				return EulerOf(i, dNow[i]);
			}
			lastStamp[i] = stamp;
			float tgt = Mathf.Clamp(sym + side[i] * diff, 0f - max, max);
			float step = Plugin.TvcSlew * Mathf.Max(0f, dt);
			dNow[i] = Mathf.MoveTowards(dNow[i], tgt, step);
			return EulerOf(i, dNow[i]);
		}
	}
}
