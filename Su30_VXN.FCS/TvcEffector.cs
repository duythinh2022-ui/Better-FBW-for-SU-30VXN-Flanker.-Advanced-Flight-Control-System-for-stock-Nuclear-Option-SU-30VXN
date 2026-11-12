using System.Collections.Generic;
using UnityEngine;

namespace Su30_VXN.FCS
{
	internal sealed class TvcEffector : Effector
	{
		public readonly List<Turbojet> engines = new List<Turbojet>();

		public float totalThrust;

		// Compass 0.1.11: thrust of engines without vectoring, and whether Mnow holds their (fixed-nozzle) moment
		public float fixedThrust;

		public bool fixedNozzles;

		internal static bool FixedNozzleMoment = true;

		public float speedFade = 1f;

		public static float CutoffFadeBand = 0.05f;

		public float NozzleLimit => max;

		// Revoker 0.1.0: the yaw angle is its own effector (TvcYawEffector); the pitch grid is then taken at yaw 0
		public bool yawSeparate;

		public TvcEffector(IEnumerable<Turbojet> fans)
		{
			name = "TVC";
			engines.AddRange(fans);
			float num = 10f;
			foreach (Turbojet engine in engines)
			{
				num = Mathf.Max(0.1f, Mathf.Abs(Acc.TF_vec(engine).x));
			}
			min = 0f - num;
			max = num;
			lo = min;
			hi = max;
			regNorm = 10f;
			mu = 5E-05f;
			lam = 2E-05f;
			AllocGrid(11);
			for (int i = 0; i < N; i++)
			{
				grid[i] = min + (float)i * (max - min) / (float)(N - 1);
			}
		}

		// Su-30 0.1.0: the real canted single-axis nozzles (CantedTvc); null = the game's pitch-axis model below
		public CantedTvc canted;

		// Su-30 0.5.0 (CplTvc): each engine's own part of M (its whole thrust moment about the CoM with its nozzle at the grid angle),
		// and whether it is vectoring: the asymmetric-load check's LP takes each nozzle as one effector
		public Vector3[][] Mn;

		public bool[] MnOn;

		// Su-30 0.1.0 (CantedTvc): the symmetric part of the canted nozzles' deflection. The table is built with both nozzles at the grid
		// value (no differential, as the stock-axis model below builds its pitch table); Mnow is the moment at the nozzles' actual
		// deflections, so the differential effector reports only its difference to this model (as on the Ifrit).
		private void EvaluateCanted(ref EvalContext ctx)
		{
			bool flag = false;
			if (Mn == null || Mn.Length != engines.Count || (Mn.Length > 0 && Mn[0].Length != N))
			{
				Mn = new Vector3[engines.Count][];
				MnOn = new bool[engines.Count];
				for (int e0 = 0; e0 < Mn.Length; e0++)
				{
					Mn[e0] = new Vector3[N];
				}
			}
			for (int e0 = 0; e0 < Mn.Length; e0++)
			{
				MnOn[e0] = false;
				for (int k0 = 0; k0 < N; k0++)
				{
					Mn[e0][k0] = Vector3.zero;
				}
			}
			for (int ei = 0; ei < engines.Count; ei++)
			{
				Turbojet engine = engines[ei];
				if (engine == null || !Acc.TF_operable(engine))
				{
					continue;
				}
				Transform[] vt = Acc.TF_vecT(engine);
				if (vt == null || vt.Length == 0 || vt[0] == null)
				{
					continue;
				}
				Aircraft aircraft = Acc.TF_aircraft(engine);
				JetNozzle[] nz = Acc.TF_nozzles(engine);
				if (nz == null)
				{
					continue;
				}
				bool gated = (Object)(object)aircraft != null && aircraft.speed > Acc.TF_vecMaxSpeed(engine);
				if ((Object)(object)aircraft != null && CutoffFadeBand > 0f)
				{
					float vmax = Acc.TF_vecMaxSpeed(engine);
					speedFade = Mathf.Min(speedFade, Mathf.Clamp01((vmax - aircraft.speed) / Mathf.Max(1f, CutoffFadeBand * vmax)));
				}
				Transform t0 = vt[0];
				Quaternion parentRot = (t0.parent != null) ? t0.parent.rotation : Quaternion.identity;
				foreach (JetNozzle jn in nz)
				{
					Transform tt = (jn != null) ? Acc.JN_thrustT(jn) : null;
					if (tt == null)
					{
						continue;
					}
					float th = Acc.JN_totalThrust(jn);
					if (th <= 1f)
					{
						continue;
					}
					totalThrust += th;
					Quaternion rot = t0.rotation;
					Quaternion rel = Quaternion.Inverse(rot) * tt.rotation;
					Vector3 off = Quaternion.Inverse(rot) * (tt.position - t0.position);
					for (int k = 0; k <= N; k++)
					{
						Vector3 e = (k < N) ? canted.EulerOf(ei, Mathf.Clamp(grid[k], min * speedFade, max * speedFade)) : canted.EulerOf(ei, canted.dNow[ei]);
						Quaternion q3 = parentRot * Quaternion.Euler(e.x, e.y, 0f);
						Vector3 dir = q3 * rel * Vector3.forward;
						Vector3 pos = t0.position + q3 * off;
						Vector3 m = Effector.ToAero(ctx.rootInv, Vector3.Cross(ThrustLine.Arm(pos, ctx.cg, ctx.rootInv), dir * th));
						if (k < N)
						{
							if (!gated)
							{
								M[k] += m;
								Mn[ei][k] += m;
								MnOn[ei] = true;
							}
						}
						else
						{
							Mnow += m;
						}
					}
					if (!gated)
					{
						flag = true;
					}
				}
			}
			now = canted.SymNow;
			rate = Plugin.TvcSlew;
			active = flag;
			lo = min * speedFade;
			hi = max * speedFade;
			if (!active)
			{
				cmd = 0f;
				lo = 0f;
				hi = 0f;
			}
		}

		public override void Evaluate(ref EvalContext ctx)
		{
			if (canted != null && !Plugin.DisableTVC)
			{
				active = false;
				for (int i = 0; i < N; i++)
				{
					M[i] = Vector3.zero;
				}
				Mnow = Vector3.zero;
				totalThrust = 0f;
				speedFade = 1f;
				fixedNozzles = false;
				fixedThrust = 0f;
				EvaluateCanted(ref ctx);
				return;
			}
			active = false;
			for (int i = 0; i < N; i++)
			{
				M[i] = Vector3.zero;
			}
			Mnow = Vector3.zero;
			totalThrust = 0f;
			bool flag = false;
			float num = 0f;
			int num2 = 0;
			speedFade = 1f;
			if (Plugin.DisableTVC)
			{
				// 2.0.8 debug: nozzles locked at 0, not an effector. 2.0.12: the thrust is still summed, so the FCS's T/W
				// (envelope thrust term, pitch-rate limit, low-thrust caps) sees the engines as they are (2.0.8-2.0.11 read 0)
				foreach (Turbojet engine0 in engines)
				{
					if (engine0 == null || !Acc.TF_operable(engine0))
					{
						continue;
					}
					JetNozzle[] nz0 = Acc.TF_nozzles(engine0);
					if (nz0 == null)
					{
						continue;
					}
					foreach (JetNozzle jn0 in nz0)
					{
						if (jn0 != null)
						{
							float th0 = Mathf.Max(0f, Acc.JN_totalThrust(jn0));
							totalThrust += th0;
							Transform tt0 = Acc.JN_thrustT(jn0);
							if (tt0 != null && th0 > 1f)
							{
								// thrust moment as it is (nozzles at 0): the asymmetric-load model's engine-out yaw
								Mnow += Effector.ToAero(ctx.rootInv, Vector3.Cross(ThrustLine.Arm(tt0.position, ctx.cg, ctx.rootInv), tt0.forward * th0));
							}
						}
					}
				}
				now = 0f;
				lo = 0f;
				hi = 0f;
				cmd = 0f;
				return;
			}
			foreach (Turbojet engine in engines)
			{
				if (engine == null || !Acc.TF_operable(engine))
				{
					continue;
				}
				Transform[] array = Acc.TF_vecT(engine);
				if (array == null || array.Length == 0)
				{
					continue;
				}
				Aircraft aircraft = Acc.TF_aircraft(engine);
				if ((Object)(object)aircraft != null && aircraft.speed > Acc.TF_vecMaxSpeed(engine))
				{
					continue;
				}
				if ((Object)(object)aircraft != null && CutoffFadeBand > 0f)
				{
					float vmax = Acc.TF_vecMaxSpeed(engine);
					speedFade = Mathf.Min(speedFade, Mathf.Clamp01((vmax - aircraft.speed) / Mathf.Max(1f, CutoffFadeBand * vmax)));
				}
				Transform transform = array[0];
				if (transform == null)
				{
					continue;
				}
				Quaternion quaternion = ((transform.parent != null) ? transform.parent.rotation : Quaternion.identity);
				Vector3 vector = Acc.TF_nozzleAngles(engine);
				float y = vector.y;
				num += vector.x;
				num2++;
				JetNozzle[] array2 = Acc.TF_nozzles(engine);
				if (array2 == null)
				{
					continue;
				}
				JetNozzle[] array3 = array2;
				foreach (JetNozzle jetNozzle in array3)
				{
					Transform transform2 = ((jetNozzle != null) ? Acc.JN_thrustT(jetNozzle) : null);
					if (transform2 == null)
					{
						continue;
					}
					float num3 = Acc.JN_totalThrust(jetNozzle);
					if (num3 <= 1f)
					{
						continue;
					}
					totalThrust += num3;
					Quaternion rotation = transform.rotation;
					Quaternion quaternion2 = Quaternion.Inverse(rotation) * transform2.rotation;
					Vector3 vector2 = Quaternion.Inverse(rotation) * (transform2.position - transform.position);
					for (int k = 0; k <= N; k++)
					{
						float value = ((k < N) ? Mathf.Clamp(grid[k], min * speedFade, max * speedFade) : vector.x);
						Quaternion quaternion3 = quaternion * Quaternion.Euler(Mathf.Clamp(value, -20f, 20f), (yawSeparate && k < N) ? 0f : y, 0f);
						Vector3 vector3 = quaternion3 * quaternion2 * Vector3.forward;
						Vector3 vector4 = transform.position + quaternion3 * vector2;
						Vector3 vector5 = Effector.ToAero(ctx.rootInv, Vector3.Cross(ThrustLine.Arm(vector4, ctx.cg, ctx.rootInv), vector3 * num3));
						if (k < N)
						{
							M[k] += vector5;
						}
						else
						{
							Mnow += vector5;
						}
					}
					flag = true;
				}
			}
			// Compass 0.1.11: engines with no vectoring transform (the T/A-30's) never reached the sum above, so the FCS saw no thrust
			// moment at all: with one engine out the live engine's yaw was missing from the asymmetric-load model and the VMC
			// protection's yaw usage read 0. Their thrust moment is taken as it is here (fixed nozzles), as in the DisableTVC case.
			// totalThrust (the FCS's T/W) is left as it was: fixedThrust carries the sum.
			fixedNozzles = false;
			fixedThrust = 0f;
			if (!flag && FixedNozzleMoment)
			{
				foreach (Turbojet engF in engines)
				{
					if (engF == null || !Acc.TF_operable(engF))
					{
						continue;
					}
					JetNozzle[] nzF = Acc.TF_nozzles(engF);
					if (nzF == null)
					{
						continue;
					}
					foreach (JetNozzle jF in nzF)
					{
						if (jF == null)
						{
							continue;
						}
						float thF = Mathf.Max(0f, Acc.JN_totalThrust(jF));
						Transform tF = Acc.JN_thrustT(jF);
						if (tF != null && thF > 1f)
						{
							fixedThrust += thF;
							Mnow += Effector.ToAero(ctx.rootInv, Vector3.Cross(ThrustLine.Arm(tF.position, ctx.cg, ctx.rootInv), tF.forward * thF));
							fixedNozzles = true;
						}
					}
				}
			}
			now = ((num2 > 0) ? (num / (float)num2) : 0f);
			rate = Plugin.TvcSlew;
			active = flag;
			lo = min * speedFade;
			hi = max * speedFade;
			if (!active)
			{
				cmd = 0f;
			}
		}
	}
}
