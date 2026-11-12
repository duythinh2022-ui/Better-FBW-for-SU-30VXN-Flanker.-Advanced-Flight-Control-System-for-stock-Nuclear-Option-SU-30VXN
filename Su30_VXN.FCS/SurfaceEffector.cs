using NuclearOption.Jobs;
using UnityEngine;

namespace Su30_VXN.FCS
{
	internal sealed class SurfaceEffector : Effector
	{
		public readonly ControlSurface cs;

		public readonly AeroPart part;

		public readonly SurfaceKind kind;

		public readonly AirfoilTable foil;

		public readonly float stallAoA;

		// 0.4.3: the foil's lift-slope knee (see AirfoilTable.KneeAoADeg), cached per slope fraction
		private float kneeFrac = -1f;

		private float kneeAoA;

		public float KneeAoA(float frac)
		{
			if (frac != kneeFrac)
			{
				kneeFrac = frac;
				kneeAoA = (foil != null) ? foil.KneeAoADeg(frac) : stallAoA;
			}
			return kneeAoA;
		}

		public float origPitchRange;

		public float origRollRange;

		public float origYawRange;

		public bool overriding;

		public float localAoANow;

		public float localAoACmd;

		public bool linearPitch;

		public bool noPitch;

		public float pitchFrac = 1f;

		public float branchCool;

		private float[] rawY;

		public float honestGate;

		private float[] ySave;

		private bool honestOn;

		// 1.8.40 roll-pitch pass: swap in the surface's true pitch curve (flaperons: their pitch moment, otherwise held
		// at the current value; stabilators: the raw curve, past the lift peak included, instead of the monotone envelope).
		public void HonestPitchOn()
		{
			float[] src = noPitch ? pitchRaw : (linearPitch ? rawY : null);
			if (!active || src == null || src.Length != N || N < 2)
			{
				honestOn = false;
				return;
			}
			if (ySave == null || ySave.Length != N)
			{
				ySave = new float[N];
			}
			for (int k = 0; k < N; k++)
			{
				ySave[k] = M[k].y;
				M[k].y = src[k];
			}
			honestOn = true;
		}

		public float HonestPitchAt(float c)
		{
			float[] src = noPitch ? pitchRaw : (linearPitch ? rawY : null);
			if (src == null || src.Length != N || N < 2)
			{
				return MAt(c).y;
			}
			float f = (c - min) / base.Step;
			if (f <= 0f)
			{
				return src[0];
			}
			if (f >= (float)(N - 1))
			{
				return src[N - 1];
			}
			int i = (int)f;
			return src[i] + (src[i + 1] - src[i]) * (f - (float)i);
		}

		public void HonestPitchOff()
		{
			if (!honestOn)
			{
				return;
			}
			for (int k = 0; k < N; k++)
			{
				M[k].y = ySave[k];
			}
			honestOn = false;
		}

		public bool branchOn;

		public float branchFail;

		public float pitchSlope;

		public float localAoA0;

		public float[] DbgRaw;

		public float[] pitchRaw;

		public bool revFlow;

		public float teUp;

		internal static float RevFlowOn = 120f;

		internal static float RevFlowOff = 110f;

		internal static bool RevFlowEnabled = true;

		private Quaternion visRot;

		private Quaternion lRel;

		private float deltaVis;

		private Vector3 com;

		private Vector3 col;

		private Vector3 v;

		private Vector3 vhat;

		private float qS;

		private float v2;

		// 2.2.0 flap-camber coupling: the fixed part ahead of this surface (fin / wing section) and its airfoil
		public AeroPart host;

		public AirfoilTable hostFoil;

		private Quaternion hostRot;

		private Vector3 hostCom;

		private Vector3 hostCol;

		private Vector3 hostV;

		private float hostQS;

		private float hostK;

		private Vector3 hingeAxis;

		// F-22E 3.1.3: the host's undeflected camber force (Camber.Foil(hostV, hostRot, hostFoil, hostQS)), the same for every table
		// point of a rebuild; worked out once per rebuild (on whichever thread runs EvalCompute) instead of once per point. Cleared by
		// EvalGather whenever the host's state is read again.
		private Vector3 hostBase;

		private bool hostBaseOk;

		public bool Valid
		{
			get
			{
				if (cs != null && part != null && Acc.CS_visible(cs) != null && Acc.AP_liftNormal(part) != null && part.rb != null && Acc.JobCreated(cs))
				{
					return !part.IsDetached();
				}
				return false;
			}
		}

		// Revoker 0.2.1: TE-up sign from the geometry, for the runway (teUp is learned from the flow only above 10 m/s): +1 when a
		// positive deflection about the hinge raises the trailing edge (aft of the hinge) in the airframe's up direction
		public float GeoTeUp(Transform root)
		{
			if (teUp != 0f)
			{
				return teUp;
			}
			GameObject vis = (cs != null) ? Acc.CS_visible(cs) : null;
			if (vis == null || root == null || !Acc.JobCreated(cs))
			{
				return 0f;
			}
			Transform t = vis.transform;
			Quaternion r0 = ((t.parent != null) ? t.parent.rotation : Quaternion.identity) * Acc.Job(cs).restingRotation;
			return GeoSign(r0 * Vector3.right, root);
		}

		public static float GeoSign(Vector3 hingeAxis, Transform root)
		{
			float s = Vector3.Dot(Vector3.Cross(hingeAxis, -root.forward), root.up);
			return (Mathf.Abs(s) < 0.1f) ? 0f : Mathf.Sign(s);
		}

		public SurfaceEffector(ControlSurface cs, AeroPart part, SurfaceKind kind, AirfoilTable foil)
		{
			this.cs = cs;
			this.part = part;
			this.kind = kind;
			this.foil = foil;
			name = Names.Canon(cs.gameObject.name);   // Su-30 0.1.0: canonical role names (Names.cs)
			// Ternion 0.1.0: the FS-3 has two pitch-surface pairs: the canards (canard_L/R, Revoker-style: their own travel,
			// attached-flow window, rollout brake) and the aft elevators (elevator_L/R, F-22E-style stabilators)
			isCanard = kind == SurfaceKind.Stabilator && name.ToLowerInvariant().Contains("canard");
			origPitchRange = Acc.CS_pitchRange(cs);
			origRollRange = Acc.CS_rollRange(cs);
			origYawRange = Acc.CS_yawRange(cs);
			float num = Mathf.Abs(origPitchRange) + Mathf.Abs(origRollRange) + Mathf.Abs(origYawRange);
			if (num < 1f)
			{
				num = 20f;
			}
			regNorm = ((RegOrigSurf >= 2 || (RegOrigSurf == 1 && kind == SurfaceKind.Stabilator)) ? num : 0f);
			if (kind == SurfaceKind.Rudder && Plugin.RudderRange > 0f)
			{
				num = Plugin.RudderRange;
			}
			else if ((kind == SurfaceKind.Aileron || kind == SurfaceKind.Flaperon) && Plugin.FlaperonRange > 0f)
			{
				num = Plugin.FlaperonRange;
			}
			else if (kind == SurfaceKind.Stabilator && isCanard && (Plugin.CanardRangeLEDown > 0f || Plugin.CanardRangeLEUp > 0f))
			{
				// canard: trailing edge up = leading edge down
				rangeUp = ((Plugin.CanardRangeLEDown > 0f) ? Plugin.CanardRangeLEDown : num);
				rangeDown = ((Plugin.CanardRangeLEUp > 0f) ? Plugin.CanardRangeLEUp : num);
				asymRange = Mathf.Abs(rangeUp - rangeDown) > 0.01f;
				num = (asymRange ? Mathf.Min(rangeUp, rangeDown) : rangeUp);
				gridSpan = Mathf.Max(rangeUp, rangeDown);
			}
			else if (kind == SurfaceKind.Stabilator && !isCanard && (Plugin.StabRangeTEUp > 0f || Plugin.StabRangeTEDown > 0f))
			{
				rangeUp = ((Plugin.StabRangeTEUp > 0f) ? Plugin.StabRangeTEUp : num);
				rangeDown = ((Plugin.StabRangeTEDown > 0f) ? Plugin.StabRangeTEDown : num);
				asymRange = Mathf.Abs(rangeUp - rangeDown) > 0.01f;
				num = (asymRange ? Mathf.Min(rangeUp, rangeDown) : rangeUp);
				gridSpan = Mathf.Max(rangeUp, rangeDown);
			}
			min = 0f - num;
			max = num;
			lo = min;
			hi = max;
			stallAoA = foil.StallAoADeg();
			int num2 = Mathf.Clamp(Mathf.RoundToInt(Mathf.Max(num, gridSpan)) + 1, 9, 31);
			AllocGrid(num2);
			for (int i = 0; i < num2; i++)
			{
				grid[i] = min + (float)i * (max - min) / (float)(num2 - 1);
			}
		}

		// F-22E 3.1.0: Evaluate = EvalGather (main thread: reads the game's objects into this effector's fields) + EvalCompute (pure
		// maths on those fields: the table over the grid, the live moment, the post-processing; safe on a worker, writes only this
		// effector). The table rebuild runs EvalCompute for the surfaces in parallel (FcsController, ParallelTables); same code, same order.
		public override void Evaluate(ref EvalContext ctx)
		{
			if (EvalGather(ref ctx))
			{
				EvalCompute(ref ctx);
			}
		}

		internal bool EvalGather(ref EvalContext ctx)
		{
			nodeOn = false;
			active = false;
			gFresh = false;   // 4.1.0 port: GridGeom's arrays belong to the state gathered before this
			if (!Valid)
			{
				return false;
			}
			ref ControlSurfaceFields reference = ref Acc.Job(cs);
			if (reference.IsDetached)
			{
				return false;
			}
			Transform transform = Acc.CS_visible(cs).transform;
			Transform transform2 = Acc.AP_liftNormal(part);
			visRot = transform.rotation;
			lRel = Quaternion.Inverse(visRot) * transform2.rotation;
			Quaternion quaternion = Quaternion.Inverse(reference.restingRotation) * transform.localRotation;
			deltaVis = 2f * Mathf.Atan2(quaternion.x, quaternion.w) * 57.29578f;
			if (deltaVis > 180f)
			{
				deltaVis -= 360f;
			}
			else if (deltaVis < -180f)
			{
				deltaVis += 360f;
			}
			now = reference.currentPitch + reference.currentRoll + reference.currentYaw;
			rate = Mathf.Max(1f, reference.servoSpeed);
			com = part.rb.worldCenterOfMass;
			col = Acc.AP_centerOfLift(part);
			v = part.rb.velocity - ctx.wind;
			if (kind == SurfaceKind.Stabilator && !isCanard)
			{
				// 2.0.8: the stabs fly in the wing's downwash (same turn of the flow as the game-side DownwashForce)
				v = Downwash.Apply(v, Quaternion.Inverse(ctx.rootInv), ctx.vAir);
			}
			v2 = v.sqrMagnitude;
			if (v2 < 1f)
			{
				v2 = 0f;
				vhat = Vector3.forward;
			}
			else
			{
				vhat = v / Mathf.Sqrt(v2);
			}
			qS = 0.5f * ctx.rho * v2 * Acc.AP_wingArea(part) * Acc.AP_wingEff(part);
			hostK = 0f;
			hostBaseOk = false;
			if (host != null && hostFoil != null && host.rb != null && !host.IsDetached())
			{
				Transform hlt = Acc.AP_liftNormal(host);
				hostK = Camber.K(kind);
				if (hlt != null && hostK > 0f)
				{
					hostRot = hlt.rotation;
					hostCom = host.rb.worldCenterOfMass;
					hostCol = Acc.AP_centerOfLift(host);
					hostV = host.rb.velocity - ctx.wind;
					hostQS = 0.5f * ctx.rho * hostV.sqrMagnitude * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
					hingeAxis = visRot * Vector3.right;
				}
				else
				{
					hostK = 0f;
				}
			}
			if (teUp == 0f && v2 > 100f)
			{
				teUp = ((LocalAoAAt(5f) < LocalAoAAt(-5f)) ? 1f : (-1f));
				if (asymRange)
				{
					ApplyAsymRange(teUp);
				}
			}
			return true;
		}

		internal void EvalCompute(ref EvalContext ctx)
		{
			// F-22E 4.1.0 port (NativeTables): the table over the grid in the native core - the rotations here (GridGeom), the flow's
			// part of Eval there, the normal force (FN) with it. Eval's own early exit (no airflow: v2 = 0) and a generic airfoil stay
			// on the C# loop.
			if (!(FcsNative.TablesOn && v2 > 0f && NativeGridOk && GridMomentsNative(ref ctx)))
			{
				for (int i = 0; i < N; i++)
				{
					M[i] = Eval(grid[i], ref ctx, out var _);
					FN[i] = lastFN;
				}
			}
			Mnow = Eval(now, ref ctx, out localAoANow);
			if (noPitch)
			{
				if (pitchRaw == null || pitchRaw.Length != N)
				{
					pitchRaw = new float[N];
				}
				for (int j = 0; j < N; j++)
				{
					pitchRaw[j] = M[j].y;
					M[j].y = Mnow.y;
				}
			}
			if (linearPitch)
			{
				float num = 0f - Mathf.Sign((origPitchRange == 0f) ? (-1f) : origPitchRange);
				float num2 = Mathf.Abs(LocalAoAAt(0f));
				if (!revFlow && num2 > RevFlowOn)
				{
					revFlow = true;
				}
				else if (revFlow && num2 < RevFlowOff)
				{
					revFlow = false;
				}
				if (revFlow && RevFlowEnabled)
				{
					num = 0f - num;
				}
				if (DbgRaw != null && DbgRaw.Length >= N)
				{
					for (int k = 0; k < N; k++)
					{
						DbgRaw[k] = M[k].y;
					}
				}
				if (rawY == null || rawY.Length != N)
				{
					rawY = new float[N];
				}
				for (int k2 = 0; k2 < N; k2++)
				{
					rawY[k2] = M[k2].y;
				}
				float num3 = float.MaxValue;
				float num4 = float.MinValue;
				for (int l = 0; l < N; l++)
				{
					num3 = Mathf.Min(num3, M[l].y);
					num4 = Mathf.Max(num4, M[l].y);
				}
				if (num > 0f)
				{
					for (int m = 1; m < N; m++)
					{
						M[m].y = Mathf.Max(M[m].y, M[m - 1].y);
					}
				}
				else
				{
					for (int num5 = N - 2; num5 >= 0; num5--)
					{
						M[num5].y = Mathf.Max(M[num5].y, M[num5 + 1].y);
					}
				}
				float num6 = Mnow.y - MAt(now).y;
				for (int n = 0; n < N; n++)
				{
					M[n].y += num6;
				}
				float num7 = float.MaxValue;
				float num8 = float.MinValue;
				for (int num9 = 0; num9 < N; num9++)
				{
					num7 = Mathf.Min(num7, M[num9].y);
					num8 = Mathf.Max(num8, M[num9].y);
				}
				float num10 = num4 - num3;
				pitchFrac = ((num10 > 0.001f) ? Mathf.Clamp01((num8 - num7 - 0.25f * num10) / (0.75f * num10)) : 1f);
				localAoA0 = LocalAoAAt(0f);
				if (honestGate > 0.001f)
				{
					bool flag = localAoA0 >= 0f;
					float hg = Mathf.Clamp01(honestGate);
					for (int k3 = 0; k3 < N; k3++)
					{
						float hv = (flag ? Mathf.Max(M[k3].y, rawY[k3]) : Mathf.Min(M[k3].y, rawY[k3]));
						M[k3].y += (hv - M[k3].y) * hg;
					}
				}
			}
			// Revoker 0.8.2 (AnchorLateral): the roll and yaw columns pass through the live moment at the surface's position (Effector.SetLiveNode)
			nodeOn = false;
			if (AnchorLateral)
			{
				SetLiveNode(now, Mnow);
			}
			anchorDbg = nodeOn ? nodeD : Vector3.zero;
			active = true;
		}

		internal static bool AnchorLateral = true;

		public Vector3 anchorDbg;

		public Vector3 Eval(float delta, ref EvalContext ctx, out float localAoADeg)
		{
			lastFN = 0f;
			Quaternion quaternion = visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel;
			Vector3 vector = Quaternion.Inverse(quaternion) * v;
			float num = Mathf.Atan2(vector.y, vector.z);
			localAoADeg = (0f - num) * 57.29578f;
			if (v2 <= 0f)
			{
				return Vector3.zero;
			}
			foil.Get(num, out var CL, out var CD);
			Vector3 rhs = quaternion * Vector3.right;
			Vector3 vector2 = Vector3.Cross(v, rhs);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			vector2 /= magnitude;
			Vector3 rhs2 = -vector2 * (CL * qS) - vhat * (CD * qS);
			Vector3 vector3 = com + quaternion * col;
			Vector3 mTot = Vector3.Cross(vector3 - ctx.cg, rhs2);
			Vector3 fTot = rhs2;
			if (hostK > 0f)
			{
				// 3.1.3: Camber.Delta with its undeflected half from the per-rebuild cache (same zero test, same expressions)
				Vector3 dH = Vector3.zero;
				if (!(hostK <= 0f) && !(Mathf.Abs(delta) < 0.01f))
				{
					if (!hostBaseOk)
					{
						hostBase = Camber.Foil(hostV, hostRot, hostFoil, hostQS);
						hostBaseOk = true;
					}
					dH = Camber.DeltaFrom(hostBase, hostV, hostRot, hingeAxis, hostK, delta, hostFoil, hostQS);
				}
				mTot += Vector3.Cross(hostCom + hostRot * hostCol - ctx.cg, dH);
				fTot += dH;
			}
			lastFN = (ctx.rootInv * fTot).y;
			return Effector.ToAero(ctx.rootInv, mTot);
		}

		public Vector3 EvalFlow(float delta, Vector3 vFlow, float rho, ref EvalContext ctx)
		{
			float sqrMagnitude = vFlow.sqrMagnitude;
			if (sqrMagnitude < 1f)
			{
				return Vector3.zero;
			}
			if (kind == SurfaceKind.Stabilator)
			{
				vFlow = Downwash.Apply(vFlow, Quaternion.Inverse(ctx.rootInv), vFlow);
			}
			Quaternion quaternion = visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel;
			Vector3 vector = Quaternion.Inverse(quaternion) * vFlow;
			foil.Get(Mathf.Atan2(vector.y, vector.z), out var CL, out var CD);
			Vector3 rhs = quaternion * Vector3.right;
			Vector3 vector2 = Vector3.Cross(vFlow, rhs);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			float num = 0.5f * rho * sqrMagnitude * Acc.AP_wingArea(part) * Acc.AP_wingEff(part);
			Vector3 rhs2 = -vector2 / magnitude * (CL * num) - vFlow / Mathf.Sqrt(sqrMagnitude) * (CD * num);
			Vector3 mF = Vector3.Cross(com + quaternion * col - ctx.cg, rhs2);
			if (hostK > 0f)
			{
				float hq = 0.5f * rho * sqrMagnitude * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
				Vector3 dH = Camber.Delta(vFlow, hostRot, hingeAxis, hostK, delta, hostFoil, hq);
				mF += Vector3.Cross(hostCom + hostRot * hostCol - ctx.cg, dH);
			}
			return Effector.ToAero(ctx.rootInv, mF);
		}

		// F-22E 3.1.3: EvalFlow for several deflections in the same flow (the asymmetric-load sweep evaluates a flaperon twice and a
		// rudder up to four times per AoA point): what depends only on the flow - its magnitude, the stab's downwash turn, the
		// dynamic pressures, the host's arm and its undeflected camber force - is worked out once (PrepFlow), the deflection's part
		// per call (EvalFlowAt). Same expressions in the same order as EvalFlow, so the results are bit-identical.
		internal struct FlowPrep
		{
			public bool zero;
			public Vector3 v;
			public float sqr;
			public float num;
			public Vector3 vhat;
			public float hq;
			public Vector3 hostArm;
			public Vector3 hostBase;
			public bool hostBaseOk;
			// F-22E 4.0.1 port (EvalFlowGrid): Camber.Foil's flow-only terms (v.normalized, its small-flow test), once per surface and point
			public Vector3 hostVN;
			public bool hostVSmall;
			public bool hostNOk;
		}

		public void PrepFlow(Vector3 vFlow, float rho, ref EvalContext ctx, out FlowPrep p)
		{
			p = default(FlowPrep);
			float sqrMagnitude = vFlow.sqrMagnitude;
			if (sqrMagnitude < 1f)
			{
				p.zero = true;
				return;
			}
			if (kind == SurfaceKind.Stabilator)
			{
				vFlow = Downwash.Apply(vFlow, Quaternion.Inverse(ctx.rootInv), vFlow);
			}
			p.v = vFlow;
			p.sqr = sqrMagnitude;
			p.num = 0.5f * rho * sqrMagnitude * Acc.AP_wingArea(part) * Acc.AP_wingEff(part);
			p.vhat = vFlow / Mathf.Sqrt(sqrMagnitude);
			if (hostK > 0f)
			{
				p.hq = 0.5f * rho * sqrMagnitude * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
				p.hostArm = hostCom + hostRot * hostCol - ctx.cg;
			}
		}

		public Vector3 EvalFlowAt(float delta, ref FlowPrep p, ref EvalContext ctx)
		{
			if (p.zero)
			{
				return Vector3.zero;
			}
			Vector3 vFlow = p.v;
			Quaternion quaternion = visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel;
			Vector3 vector = Quaternion.Inverse(quaternion) * vFlow;
			foil.Get(Mathf.Atan2(vector.y, vector.z), out var CL, out var CD);
			Vector3 rhs = quaternion * Vector3.right;
			Vector3 vector2 = Vector3.Cross(vFlow, rhs);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			float num = p.num;
			Vector3 rhs2 = -vector2 / magnitude * (CL * num) - p.vhat * (CD * num);
			Vector3 mF = Vector3.Cross(com + quaternion * col - ctx.cg, rhs2);
			if (hostK > 0f)
			{
				Vector3 dH = Vector3.zero;
				if (!(hostK <= 0f) && !(Mathf.Abs(delta) < 0.01f))
				{
					if (!p.hostBaseOk)
					{
						p.hostBase = Camber.Foil(vFlow, hostRot, hostFoil, p.hq);
						p.hostBaseOk = true;
					}
					dH = Camber.DeltaFrom(p.hostBase, vFlow, hostRot, hingeAxis, hostK, delta, hostFoil, p.hq);
				}
				mF += Vector3.Cross(p.hostArm, dH);
			}
			return Effector.ToAero(ctx.rootInv, mF);
		}

		// Su-30 0.1.8 (CanardPitchPeakWindow): the deflections of this surface's own raw (not monotone) pitch-moment peaks, the most
		// nose-up and the most nose-down, over its whole travel. Among grid points within 3 % of the span of a peak the one nearest
		// zero deflection is taken, so a flat top does not send the surface to the end of its travel.
		public bool RawPitchPeaks(out float cUp, out float cDn)
		{
			cUp = max;
			cDn = min;
			if (!active || rawY == null || rawY.Length != N || N < 3)
			{
				return false;
			}
			float yMax = float.MinValue, yMin = float.MaxValue;
			for (int i = 0; i < N; i++)
			{
				yMax = Mathf.Max(yMax, rawY[i]);
				yMin = Mathf.Min(yMin, rawY[i]);
			}
			float tol = 0.03f * (yMax - yMin);
			if (yMax - yMin < 1f)
			{
				return false;
			}
			float bU = float.MaxValue, bD = float.MaxValue;
			for (int j = 0; j < N; j++)
			{
				float g = grid[j];
				if (rawY[j] >= yMax - tol && Mathf.Abs(g) < bU)
				{
					bU = Mathf.Abs(g);
					cUp = g;
				}
				if (rawY[j] <= yMin + tol && Mathf.Abs(g) < bD)
				{
					bD = Mathf.Abs(g);
					cDn = g;
				}
			}
			return true;
		}

		// Su-30 0.1.9: the deflections whose raw pitch moment is within frac of the range below the most nose-up one. Past the
		// stall that set can be both ends of the travel (stabs at 60-65 deg AoA: -30 and +35 good, 0 worst), so it is split into
		// its runs; with a lateral demand the run whose roll / yaw moment best matches it (roll and yaw together) is taken,
		// else the run nearest nearC.
		public bool RawPitchGoodZone(float frac, float rollDem, float yawDem, float nearC, out float lo, out float hi)
		{
			lo = min;
			hi = max;
			if (!active || rawY == null || rawY.Length != N || N < 3 || M == null || M.Length < N)
			{
				return false;
			}
			float yMax = float.MinValue, yMin = float.MaxValue;
			for (int i = 0; i < N; i++)
			{
				yMax = Mathf.Max(yMax, rawY[i]);
				yMin = Mathf.Min(yMin, rawY[i]);
			}
			if (yMax - yMin < 1f)
			{
				return false;
			}
			float thr = yMax - frac * (yMax - yMin);
			float best = float.MinValue;
			bool found = false;
			int j = 0;
			while (j < N)
			{
				if (rawY[j] < thr)
				{
					j++;
					continue;
				}
				int a = j;
				float mx = 0f;
				while (j < N && rawY[j] >= thr)
				{
					mx += M[j].x * rollDem + M[j].z * yawDem;
					j++;
				}
				int b = j - 1;
				float score;
				if (Mathf.Abs(rollDem) + Mathf.Abs(yawDem) > 0f)
				{
					score = mx / (float)(b - a + 1);
				}
				else
				{
					score = 0f - Mathf.Max(0f, Mathf.Max(grid[a] - nearC, nearC - grid[b]));
				}
				if (!found || score > best)
				{
					best = score;
					lo = grid[a];
					hi = grid[b];
					found = true;
				}
			}
			return found;
		}

		// Su-30 0.4.0, port of F-22E 4.0.1 (SweepGridCache): the asymmetric-load sweep's capacity loop evaluates every grid deflection of a surface in each
		// AoA point's flow (26-31 deflections per surface, three points per sweep tick). What depends on the deflection alone - the
		// surface's rotation at that deflection (its inverse, its span axis, its moment arm) and the host section's cambered rotation
		// (inverse, span axis) - is the same for every point of the tick: GridGeom works it out once per sweep tick, EvalFlowGrid then
		// does the flow's part of EvalFlow for grid point l. Same expressions in the same order as EvalFlow / Camber.Delta.
		private Quaternion[] gInv;

		private Vector3[] gRight;

		private Vector3[] gArm;

		private Quaternion[] gHInv;

		private Vector3[] gHRight;

		private bool[] gHZero;

		// 4.1.0: GridGeom's arrays are those of the state EvalGather last read, for this cg (EvalGather clears it). The table rebuild's
		// native path works the geometry out every tick; the sweep, later in the same tick, then uses it as it is.
		private bool gFresh;

		private Vector3 gCg;

		// this jet: the deflections the arrays were built for (the table's grid, or the sweep's own list: AsymStabTravel), so the
		// sweep reuses the table's geometry only when it evaluates the same deflections
		private float[] gDefl;

		internal bool GridGeomFreshFor(ref EvalContext ctx, float[] defl)
		{
			if (!(gFresh && gInv != null && gInv.Length == N && gCg.x == ctx.cg.x && gCg.y == ctx.cg.y && gCg.z == ctx.cg.z && gDefl != null && defl != null))
			{
				return false;
			}
			for (int l = 0; l < N; l++)
			{
				if (gDefl[l] != defl[l])
				{
					return false;
				}
			}
			return true;
		}

		// the grid kernels need tabulated airfoils (a generic one is sin / cos formulas: C# only)
		internal bool NativeGridOk => N > 0 && foil != null && !foil.Generic && (hostK <= 0f || (hostFoil != null && !hostFoil.Generic));

		// the per-call part of FcsNative.GridIn that does not depend on the flow; the caller has pinned the arrays
		private unsafe bool GridCall(ref FcsNative.GridIn g, int mode, Vector3[] outM, float[] outFN)
		{
			float[] cl = foil.ClTable, cd = foil.CdTable;
			bool host = hostK > 0f;
			float[] hcl = host ? hostFoil.ClTable : cl, hcd = host ? hostFoil.CdTable : cd;
			fixed (Quaternion* pgInv = gInv, pghInv = gHInv)
			fixed (Vector3* pgRight = gRight, pgArm = gArm, pghRight = gHRight, pOut = outM)
			fixed (bool* pZero = gHZero)
			fixed (float* pcl = cl, pcd = cd, phcl = hcl, phcd = hcd, pFN = outFN)
			{
				g.gInv = pgInv;
				g.gRight = pgRight;
				g.gArm = pgArm;
				g.ghInv = pghInv;
				g.ghRight = pghRight;
				g.ghZero = (byte*)pZero;
				g.cl = pcl;
				g.cd = pcd;
				g.hcl = phcl;
				g.hcd = phcd;
				g.outM = pOut;
				g.outFN = pFN;   // this jet: Eval's normal force per grid point (FN), mode 0
				g.n = N;
				g.host = host ? 1 : 0;
				fixed (FcsNative.GridIn* pg = &g)
				{
					return FcsNative.Grid(pg, mode);
				}
			}
		}

		// EvalCompute's loop M[i] = Eval(grid[i]) in the native core. false = M is untouched or not to be used: run the C# loop.
		private unsafe bool GridMomentsNative(ref EvalContext ctx)
		{
			if (M == null || M.Length < N || FN == null || FN.Length < N)
			{
				return false;
			}
			GridGeom(ref ctx, grid);
			FcsNative.GridIn g = default(FcsNative.GridIn);
			g.v = v;
			g.vhat = vhat;
			g.num = qS;
			if (hostK > 0f)
			{
				// Eval's host terms: the undeflected force once per rebuild (as Eval caches it), Camber.Foil's flow-only terms
				if (!hostBaseOk)
				{
					hostBase = Camber.Foil(hostV, hostRot, hostFoil, hostQS);
					hostBaseOk = true;
				}
				g.hv = hostV;
				g.hvn = hostV.normalized;
				g.hvSmall = (hostV.sqrMagnitude < 1E-06f) ? 1 : 0;
				g.hq = hostQS;
				g.hostArm = hostCom + hostRot * hostCol - ctx.cg;
				g.hostBase = hostBase;
			}
			g.rootInv = ctx.rootInv;
			return GridCall(ref g, 0, M, FN);
		}

		// the sweep's capacity loop for this surface in the flow p was prepared for (FcsController.AsymSweepPoint): the min / max of
		// pitch moment, roll and yaw acceleration over the grid. false = nothing written: run the C# loop.
		internal unsafe bool GridRangeNative(ref FlowPrep p, ref EvalContext ctx, float i00, float i02, float i20, float i22,
			ref float pMin, ref float pMax, ref float rMin, ref float rMax, ref float yMin, ref float yMax)
		{
			if (p.zero || !NativeGridOk || gInv == null || gInv.Length != N)
			{
				return false;
			}
			FcsNative.GridIn g = default(FcsNative.GridIn);
			g.v = p.v;
			g.vhat = p.vhat;
			g.num = p.num;
			if (hostK > 0f)
			{
				// EvalFlowGrid's host terms, worked out before the loop instead of at its first deflected point (same values)
				if (!p.hostBaseOk)
				{
					p.hostBase = Camber.Foil(p.v, hostRot, hostFoil, p.hq);
					p.hostBaseOk = true;
				}
				if (!p.hostNOk)
				{
					p.hostVSmall = p.v.sqrMagnitude < 1E-06f;
					p.hostVN = p.v.normalized;
					p.hostNOk = true;
				}
				g.hv = p.v;
				g.hvn = p.hostVN;
				g.hvSmall = p.hostVSmall ? 1 : 0;
				g.hq = p.hq;
				g.hostArm = p.hostArm;
				g.hostBase = p.hostBase;
			}
			g.rootInv = ctx.rootInv;
			g.ia[0] = i00;
			g.ia[1] = i02;
			g.ia[2] = i20;
			g.ia[3] = i22;
			if (!GridCall(ref g, 1, gArm, null))
			{
				return false;
			}
			pMin = g.o[0];
			pMax = g.o[1];
			rMin = g.o[2];
			rMax = g.o[3];
			yMin = g.o[4];
			yMax = g.o[5];
			return true;
		}

		internal void GridGeom(ref EvalContext ctx, float[] defl)
		{
			if (gDefl == null || gDefl.Length != N)
			{
				gDefl = new float[N];
			}
			if (gInv == null || gInv.Length != N)
			{
				gInv = new Quaternion[N];
				gRight = new Vector3[N];
				gArm = new Vector3[N];
				gHInv = new Quaternion[N];
				gHRight = new Vector3[N];
				gHZero = new bool[N];
			}
			bool hasHost = hostK > 0f;
			for (int l = 0; l < N; l++)
			{
				float delta = defl[l];
				gDefl[l] = delta;
				Quaternion quaternion = visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel;
				gInv[l] = Quaternion.Inverse(quaternion);
				gRight[l] = quaternion * Vector3.right;
				gArm[l] = com + quaternion * col - ctx.cg;
				if (hasHost)
				{
					// Camber.Delta's zero test, and its cambered host rotation r1
					bool z = hostK <= 0f || Mathf.Abs(delta) < 0.01f;
					gHZero[l] = z;
					if (!z)
					{
						Quaternion r1 = Quaternion.AngleAxis(hostK * delta, hingeAxis) * hostRot;
						gHInv[l] = Quaternion.Inverse(r1);
						gHRight[l] = r1 * Vector3.right;
					}
				}
			}
			gCg = ctx.cg;
			gFresh = true;
		}

		// = EvalFlowAt(the l-th deflection GridGeom was built for, p), on GridGeom's values of this tick
		internal Vector3 EvalFlowGrid(int l, ref FlowPrep p, ref EvalContext ctx)
		{
			if (p.zero)
			{
				return Vector3.zero;
			}
			Vector3 vFlow = p.v;
			Vector3 vector = gInv[l] * vFlow;
			foil.Get(Mathf.Atan2(vector.y, vector.z), out var CL, out var CD);
			Vector3 vector2 = Vector3.Cross(vFlow, gRight[l]);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			float num = p.num;
			Vector3 rhs2 = -vector2 / magnitude * (CL * num) - p.vhat * (CD * num);
			Vector3 mF = Vector3.Cross(gArm[l], rhs2);
			if (hostK > 0f)
			{
				Vector3 dH = Vector3.zero;
				if (!gHZero[l])
				{
					if (!p.hostBaseOk)
					{
						p.hostBase = Camber.Foil(vFlow, hostRot, hostFoil, p.hq);
						p.hostBaseOk = true;
					}
					if (!p.hostNOk)
					{
						p.hostVSmall = vFlow.sqrMagnitude < 1E-06f;
						p.hostVN = vFlow.normalized;
						p.hostNOk = true;
					}
					dH = Camber.FoilPre(vFlow, gHInv[l], gHRight[l], hostFoil, p.hq, p.hostVN, p.hostVSmall) - p.hostBase;
				}
				mF += Vector3.Cross(p.hostArm, dH);
			}
			return Effector.ToAero(ctx.rootInv, mF);
		}

		public float PitchRawAt(float c)
		{
			if (pitchRaw == null || N < 2)
			{
				return 0f;
			}
			float num = (c - min) / base.Step;
			if (num <= 0f)
			{
				return pitchRaw[0];
			}
			if (num >= (float)(N - 1))
			{
				return pitchRaw[N - 1];
			}
			int num2 = (int)num;
			float num3 = num - (float)num2;
			return pitchRaw[num2] + (pitchRaw[num2 + 1] - pitchRaw[num2]) * num3;
		}

		public static int RegOrigSurf = 0;

		public float rangeUp;

		// Ternion 0.1.0: a canard (the forward pitch pair); false for the aft elevators
		public bool isCanard;

		public float rangeDown;

		public bool asymRange;

		private float gridSpan;

		public float Span => Mathf.Max(max, 0f - min);

		public float LimitFor(float c)
		{
			return (c >= 0f) ? max : (0f - min);
		}

		// Revoker 0.2.18: the asymmetric travel is applied from the geometry sign (GeoTeUp) when the surface has not seen 10 m/s
		// of flow yet (teUp is learned from the flow only), so on the runway at a standstill the canards already have their
		// +70 / -45 instead of +-45. GeoTeUp is the same definition as the flow-learned teUp (+1: a positive deflection raises the
		// trailing edge), and the 0.2.8 rollout brake already relies on it.
		public void EnsureAsymRange(Transform root)
		{
			if (!asymRange)
			{
				return;
			}
			float g = GeoTeUp(root);
			if (g != 0f)
			{
				ApplyAsymRange(g);
			}
		}

		private void ApplyAsymRange(float sgn)
		{
			if (sgn > 0f)
			{
				max = rangeUp;
				min = 0f - rangeDown;
			}
			else
			{
				max = rangeDown;
				min = 0f - rangeUp;
			}
			for (int i = 0; i < N; i++)
			{
				grid[i] = min + (float)i * (max - min) / (float)(N - 1);
			}
			asymRange = false;
			Plugin.Log.LogInfo($"Su-30 FCS: {name} travel {min:0}/+{max:0} deg (trailing edge up is {(sgn > 0f ? "+" : "-")}).");
		}

		// ---- Su-30 0.5.6 (from the Ternion 0.3.0) (CplGridCache): the coupled asymmetric-load check's own points (FcsController.CplIdx's subset of the grid,
		// CplPts of them per surface). Their geometry once per sweep tick (CplGeom, as GridGeom), then per sweep point the flow's
		// part of EvalFlowAt for point k (EvalFlowCpl = EvalFlowGrid on these arrays), the point's local AoA from the same rotation
		// (LocalAoACpl = LocalAoAFlow) and, with the native kernels, every point's moment in one call (CplMomentsNative: the table
		// kernel, mode 0, in the sweep point's flow). 0.2.2 built the rotation for each point at every sweep point, and again for the
		// attached-side test.
		private Quaternion[] cInv;

		private Vector3[] cRight;

		private Vector3[] cArm;

		private Quaternion[] cHInv;

		private Vector3[] cHRight;

		private bool[] cHZero;

		internal int cN;

		internal void CplGeom(ref EvalContext ctx, int n)
		{
			if (cInv == null || cInv.Length != n)
			{
				cInv = new Quaternion[n];
				cRight = new Vector3[n];
				cArm = new Vector3[n];
				cHInv = new Quaternion[n];
				cHRight = new Vector3[n];
				cHZero = new bool[n];
			}
			cN = n;
			bool hasHost = hostK > 0f;
			for (int l = 0; l < n; l++)
			{
				float delta = grid[FcsController.CplIdx(l, N)];
				Quaternion quaternion = visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel;
				cInv[l] = Quaternion.Inverse(quaternion);
				cRight[l] = quaternion * Vector3.right;
				cArm[l] = com + quaternion * col - ctx.cg;
				if (hasHost)
				{
					// Camber.Delta's zero test, and its cambered host rotation r1
					bool z = hostK <= 0f || Mathf.Abs(delta) < 0.01f;
					cHZero[l] = z;
					if (!z)
					{
						Quaternion r1 = Quaternion.AngleAxis(hostK * delta, hingeAxis) * hostRot;
						cHInv[l] = Quaternion.Inverse(r1);
						cHRight[l] = r1 * Vector3.right;
					}
				}
			}
		}

		// = LocalAoAFlow(the k-th coupled-check deflection, p), on CplGeom's rotation
		internal float LocalAoACpl(int k, ref FlowPrep p)
		{
			if (p.zero)
			{
				return 0f;
			}
			Vector3 vector = cInv[k] * p.v;
			return (0f - Mathf.Atan2(vector.y, vector.z)) * 57.29578f;
		}

		// = EvalFlowAt(the l-th coupled-check deflection, p), on CplGeom's values of this tick
		internal Vector3 EvalFlowCpl(int l, ref FlowPrep p, ref EvalContext ctx)
		{
			if (p.zero)
			{
				return Vector3.zero;
			}
			Vector3 vFlow = p.v;
			Vector3 vector = cInv[l] * vFlow;
			foil.Get(Mathf.Atan2(vector.y, vector.z), out var CL, out var CD);
			Vector3 vector2 = Vector3.Cross(vFlow, cRight[l]);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			float num = p.num;
			Vector3 rhs2 = -vector2 / magnitude * (CL * num) - p.vhat * (CD * num);
			Vector3 mF = Vector3.Cross(cArm[l], rhs2);
			if (hostK > 0f)
			{
				Vector3 dH = Vector3.zero;
				if (!cHZero[l])
				{
					if (!p.hostBaseOk)
					{
						p.hostBase = Camber.Foil(vFlow, hostRot, hostFoil, p.hq);
						p.hostBaseOk = true;
					}
					if (!p.hostNOk)
					{
						p.hostVSmall = vFlow.sqrMagnitude < 1E-06f;
						p.hostVN = vFlow.normalized;
						p.hostNOk = true;
					}
					dH = Camber.FoilPre(vFlow, cHInv[l], cHRight[l], hostFoil, p.hq, p.hostVN, p.hostVSmall) - p.hostBase;
				}
				mF += Vector3.Cross(p.hostArm, dH);
			}
			return Effector.ToAero(ctx.rootInv, mF);
		}

		private unsafe bool CplGridCall(ref FcsNative.GridIn g, Vector3[] outM)
		{
			float[] cl = foil.ClTable, cd = foil.CdTable;
			bool host = hostK > 0f;
			float[] hcl = host ? hostFoil.ClTable : cl, hcd = host ? hostFoil.CdTable : cd;
			fixed (Quaternion* pgInv = cInv, pghInv = cHInv)
			fixed (Vector3* pgRight = cRight, pgArm = cArm, pghRight = cHRight, pOut = outM)
			fixed (bool* pZero = cHZero)
			fixed (float* pcl = cl, pcd = cd, phcl = hcl, phcd = hcd)
			{
				g.gInv = pgInv;
				g.gRight = pgRight;
				g.gArm = pgArm;
				g.ghInv = pghInv;
				g.ghRight = pghRight;
				g.ghZero = (byte*)pZero;
				g.cl = pcl;
				g.cd = pcd;
				g.hcl = phcl;
				g.hcd = phcd;
				g.outM = pOut;
				g.outFN = null;
				g.n = cN;
				g.host = host ? 1 : 0;
				fixed (FcsNative.GridIn* pg = &g)
				{
					return FcsNative.Grid(pg, 0);
				}
			}
		}

		// every coupled-check point's moment in the flow p was prepared for, in the native core (the table kernel on CplGeom's
		// arrays). false = outM is not to be used: run EvalFlowCpl.
		internal unsafe bool CplMomentsNative(ref FlowPrep p, ref EvalContext ctx, Vector3[] outM)
		{
			if (p.zero || !NativeGridOk || cInv == null || cN < 1 || cInv.Length != cN || outM == null || outM.Length < cN)
			{
				return false;
			}
			FcsNative.GridIn g = default(FcsNative.GridIn);
			g.v = p.v;
			g.vhat = p.vhat;
			g.num = p.num;
			if (hostK > 0f)
			{
				// EvalFlowGrid's host terms, worked out before the loop instead of at its first deflected point (same values)
				if (!p.hostBaseOk)
				{
					p.hostBase = Camber.Foil(p.v, hostRot, hostFoil, p.hq);
					p.hostBaseOk = true;
				}
				if (!p.hostNOk)
				{
					p.hostVSmall = p.v.sqrMagnitude < 1E-06f;
					p.hostVN = p.v.normalized;
					p.hostNOk = true;
				}
				g.hv = p.v;
				g.hvn = p.hostVN;
				g.hvSmall = p.hostVSmall ? 1 : 0;
				g.hq = p.hq;
				g.hostArm = p.hostArm;
				g.hostBase = p.hostBase;
			}
			g.rootInv = ctx.rootInv;
			return CplGridCall(ref g, outM);
		}

		// Su-30 0.5.0 (from the Ternion 0.2.2): the surface's local AoA at deflection delta in the sweep's flow (FlowPrep)
		public float LocalAoAFlow(float delta, ref FlowPrep p)
		{
			if (p.zero)
			{
				return 0f;
			}
			Vector3 vector = Quaternion.Inverse(visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel) * p.v;
			return (0f - Mathf.Atan2(vector.y, vector.z)) * 57.29578f;
		}

		public float LocalAoAAt(float delta)
		{
			Vector3 vector = Quaternion.Inverse(visRot * Quaternion.AngleAxis(delta - deltaVis, Vector3.right) * lRel) * v;
			return (0f - Mathf.Atan2(vector.y, vector.z)) * 57.29578f;
		}
	}
}
