using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Su30_VXN.FCS
{
	internal sealed class FcsController
	{
		// StickCancelsLimit (ported from F-22E FCS 2.44.0, Tom 4 Nov 20:26): "if i hold stick all the way back at max aoa, and the jet still can
		// do positive pitch rate, then nothing happens; as soon as the only pitch rate that can hold that aoa is 0, that is the point the stick
		// can cancel the protection". At the AoA limit the nose-down part of the limit's own command (the capture bound below the limit,
		// protection above it) scales with 1 - stick / 0.9, as protection's push always did; a positive holding rate is never scaled. Before,
		// below limit + 0.5 the aft-stick floor held the command at 0 and above it 1.9.5's 'pilot command' (the stick clamped by the capture
		// bound) brought the bound's nose-down back through full aft: slowing with the flight path falling, the command cycled 0 <-> ~-12
		// deg/s at ~2 Hz (F-22E 20:07 log). 1.9.5's rule keeps its purpose: a stick asking for more nose-down than protection still gets it.
		internal static bool StickCancelsLimit = true;

		// Su-30 0.1.11 (GyroBudget, Tom 9 Nov 11:28: inverted corkscrews stop, then tumble): at low IAS the nose follows the falling flight
		// path at up to 45-60 deg/s while the jet yaws at 30-40 deg/s, and the gyroscopic roll (Iyy - Izz) / Ixx x q x r (+30 deg/s^2 at
		// 82 km/h, 11:08 log 201 s) is more than the nozzles' roll authority (14-33). The FCS answered with the nozzle differential, and on
		// the canted nozzles that same differential yaws the jet as hard as it rolls it, the way the yaw is already going: r -30 -> -56,
		// sideslip 52 deg. The pitch-rate command is held to what leaves gyro roll within this share of the roll authority. Faded in from 170 to 130 km/h
		// (above that the surfaces carry enough roll; a 0.25 share reaching up to 250 km/h cost the falling leaf 13 deg of AoA on the bench). Never limits a nose-down or protection command.
		internal static float GyroBudgetQFloor = 8f;
		internal static float GyroBudgetAoA0 = 35f;
		internal static float GyroBudgetAoA1 = 45f;
		internal static float GyroBudgetIasHi = 170f;
		internal static float GyroBudgetIasLo = 130f;
		public float gyroQLimDbg;
		private bool capAtLimU, capAtLimD;
		// Compass 0.1.2 (Tom 5 Nov 17:03: "no zero-rate hold rule for bypass protection on non tvc jet"): the stick may cancel the AoA
		// limit (StickCancelsLimit above, and protection's push scaled by aft stick) only on a jet with pitch thrust vectoring. Without
		// it nothing holds the nose once the elevators are out of travel, so on the T/A-30 full aft is a request for the limit, not past
		// it: protection's push and the capture bound's nose-down apply in full whatever the stick (forward stick asking for more
		// nose-down still gets it).
		private bool StickBypass => StickCancelsLimit && tvc != null && tvc.max > 0.5f && !Plugin.DisableTVC;

		// Su-30 0.1.11 (LowSpeedTvcRoll): in that mode's low-speed band the AoA law keeps its limit, so the stick stops cancelling it
		private bool StickBypassNow => StickBypass && !(Mode == FcsMode.AoAG && lowSpeedAxisW > 0.5f);

		private float StickScale(float stick)
		{
			if (!StickBypass)
			{
				return 1f;
			}
			return Mathf.Lerp(1f - Mathf.Clamp01(stick / 0.9f), 1f, (Mode == FcsMode.AoAG) ? lowSpeedAxisW : 0f);
		}

		public readonly Aircraft ac;

		public readonly ControlInputs inputs;

		public readonly List<SurfaceEffector> surfaces = new List<SurfaceEffector>();

		public readonly TvcEffector tvc;

		public readonly FlapSystem flaps;

		private readonly List<Effector> effs = new List<Effector>();

		private readonly Allocator alloc = new Allocator();

		private readonly List<LandingGear> gears = new List<LandingGear>();

		private readonly List<Rigidbody> bodies = new List<Rigidbody>();

		private readonly List<AeroPart> aeroParts = new List<AeroPart>();

		private readonly Dictionary<AeroPart, AirfoilTable> foils = new Dictionary<AeroPart, AirfoilTable>();

		public bool Engaged;

		// Su-30 0.1.0
		public CantedTvc canted;

		public SurfaceEffector canL;

		public SurfaceEffector canR;

		internal static float CanardSplitCost = 100f;

		// set by the game plugin (Registry): switches the Su-30's stock CanardSurface AoA schedule off on the canards while the FCS flies
		// (true) and back on when it stops (false). Not set on the bench (no CanardSurface there).
		public static System.Action<FcsController, bool> CanardHook;

		private bool canardOwned;

		// Su-30 0.1.8: while owned with no schedule script found yet, the hook is called again every ~2 s (up to 60 times)
		public static System.Func<FcsController, int> CanardCountHook;

		private int canardRetries;

		private float canardRetryT;

		public void OwnCanards(bool own)
		{
			if (Plugin.StockCanardSchedule)
			{
				own = false;
			}
			if (own && canardOwned && CanardHook != null && CanardCountHook != null && canardRetries < 60)
			{
				canardRetryT += UnityEngine.Time.fixedDeltaTime;
				if (canardRetryT >= 2f)
				{
					canardRetryT = 0f;
					int nFound = 0;
					try
					{
						nFound = CanardCountHook(this);
						if (nFound == 0)
						{
							canardRetries++;
							CanardHook(this, true);
						}
						else
						{
							canardRetries = 60;
						}
					}
					catch
					{
						canardRetries = 60;
					}
				}
				return;
			}
			if (own == canardOwned || CanardHook == null)
			{
				return;
			}
			canardOwned = own;
			try
			{
				CanardHook(this, own);
			}
			catch
			{
			}
		}

		// Su-30 0.1.0 (CantedTvc): one nozzle's transform angles this physics tick (Hooks.SetNozzleEuler in game, the plant on the bench):
		// the nozzle slews toward sym + side x diff, sym from the pitch nozzle command the game would have turned into its pitch angle
		// (-tvcPitchInput x vec.x x gain.x: the air law's TVC command, the ground law's, or the brake's), diff from the canted differential
		// effector. Past the engine's thrustVectoringMaxAirspeed (or with DisableTVC) the target is 0.
		public Vector3 NozzleEuler(Turbojet eng, Vector3 stock, float dt, float stamp)
		{
			if (canted == null)
			{
				return stock;
			}
			int i = canted.IndexOf(eng);
			if (i < 0)
			{
				return stock;
			}
			float sym = 0f;
			float diff = 0f;
			if (!Plugin.DisableTVC && Acc.TF_operable(eng) && !((UnityEngine.Object)(object)ac != null && ac.speed > Acc.TF_vecMaxSpeed(eng)))
			{
				float vx = Mathf.Max(0.1f, Mathf.Abs(Acc.TF_vec(eng).x));
				float gx = Acc.TF_vecGain(eng).x;
				if (Mathf.Abs(gx) < 0.001f)
				{
					gx = 1f;
				}
				sym = Mathf.Clamp((0f - tvcPitchInput) * vx * gx, 0f - canted.max, canted.max);
				diff = (tvcd != null && tvcd.active && Mode != FcsMode.Ground) ? tvcd.cmdOut : 0f;
				// Su-30 0.2.3 (GroundTvcRollYaw, Tom 9 Nov 23:16: "the nozzles only react to pitch input on ground, i want roll and yaw
				// input effect the nozzles as well"): on the ground the differential follows the roll stick and the rudder pedals
				// directly, like the pitch stick does the symmetric part: left stick or left pedal = left nozzle jet up, right nozzle
				// jet down (diff > 0, rolls and yaws the jet left), each nozzle clamped to its own travel.
				if (Mode == FcsMode.Ground && Plugin.GroundTvcRollYaw)
				{
					diff = Mathf.Clamp(0f - sr - sy, -1f, 1f) * canted.max;
				}
				// Su-30 0.1.15 (Tom 9 Nov 20:25): in the LowSpeedTvcRoll band the nozzles' differential follows the roll stick
				// directly, in body axes: left stick = left nozzle jet up / right nozzle jet down (diff > 0, rolls and yaws the jet
				// left), whatever the AoA, the attitude or the flow direction. 0.1.11-0.1.14 flew it through the rate loops, which
				// flipped it with the reversed-flow roll sense (20:21 log, 122.5-123.3 s, AoA 130-143: full left stick, nozzles
				// right) and when the roll or yaw ran past its command. With Stability Assist on the pitch part keeps what it needs
				// for the AoA limit (the differential takes what is left of the travel); in MPO the roll takes it all.
				if (lowSpeedAxisW > 0f && Mode != FcsMode.Ground && tvcd != null && tvcd.active)
				{
					float room = (Mode == FcsMode.AoAG) ? Mathf.Max(0f, canted.max - Mathf.Abs(sym)) : canted.max;
					// Su-30 0.3.0 (AxisPedalRoll, Tom 10 Nov 16:46: "make pedals also do tvc vector roll"): the pedals drive the
					// differential the same way as the roll stick (left pedal = left nozzle jet up), stick and pedal summed
					float dStick = Mathf.Clamp(0f - sr - (AxisPedalRoll ? sy : 0f), -1f, 1f) * room;
					// Su-30 0.3.0 (MpoAxisReleaseStop, Tom: "when i release control, a lot of the time the fcs refuses to stop it"):
					// with both released the direct-stick differential was 0, so the nozzles sat neutral and only the airframe damped the
					// yaw (16:16 log 728.6 s: r -68 deg/s for 2 s, differential 0-3 deg while the allocator asked for 15). In the
					// released stop the allocator's differential (yaw brake, roll weight cut) takes over.
					diff = Mathf.Lerp(diff, dStick, lowSpeedAxisW * (1f - axRelDbg) * (1f - lsSlipBodyW));
				}
			}
			return canted.Step(i, sym, diff, dt, stamp);
		}

		public bool Failed;

		public FcsMode Mode;

		private float mass = 20000f;

		private Vector3 cg;

		internal Vector3 CgWorld => cg;

		internal bool HasCg => cgValid;

		private bool cgValid;

		private bool thrustLineReg;

		private readonly float[,] Ia = new float[3, 3];

		private readonly float[,] IaInv = new float[3, 3];

		private float massTimer = 99f;

		private float nAlphaTimer = 99f;

		private Vector3 omega;

		private Vector3 omegaPrev;

		private Lpf3 omegaDotF;

		private Lpf3 mNowF;

		private Lpf3 accF;

		// F-22E 2.30.0 port: air data at the CG. vCg = the mass-weighted velocity of every body (the same weighting as cg); the AoA,
		// sideslip, TAS / IAS and the flight-path rates (omegaV, vDot) are read from it instead of the root rigidbody's velocity.
		// nz stays the root body's. The root sits ahead of the CG on these jets (Ifrit ~0.62 m, Revoker ~0.95 m), so a yaw / pitch
		// rate put a false sideslip / AoA of rate x arm / V on every read.
		// Revoker 0.5.1: off by default. Bench (CG-to-CG comparison, the root sits 0.93 m BEHIND the CG on this jet): 2 of 20
		// regression cases worse by more than the ~0.5 deg noise, both 65 deg AoA releases (peak sideslip 1.7 -> 2.8 and 1.1 -> 1.8
		// deg), and 5 of 32 damage-suite cases. Logged both ways (alphaRoot/betaRoot, alphaCg/betaCg) for the flight to decide.
		internal static bool AirDataAtCg = false;

		internal static bool FlightPathAtCg = false;

		public float alphaRoot, betaRoot, alphaCgDbg, betaCgDbg;

		private Vector3 vCg;

		private Vector3 vCgPrev;

		private Lpf3 accCgF;

		// F-22E 3.0.0 capacity estimator, stock-jet port (every effector grouped, sideslip-axis read): telemetry only
		internal readonly CapacityEstimator capEst = new CapacityEstimator();

		// Su-30 0.2.0 (from F-22E 3.1.0): the table rebuild's surfaces for the fork-join (ParallelTables)
		private readonly List<SurfaceEffector> parSurf = new List<SurfaceEffector>();

		// F-22E 3.1.0 per-phase timer (Stopwatch ticks, this tick): table rebuild, allocator solves (main allocator), estimator kick, CSV capture
		internal long phRebuild, phAlloc, phKick, phRow;

		// F-22E 3.1.1: the rest split further - laws, the asym step (its moment, slip probe and sweep inside it), the envelope, mass
		// properties, flaps, ground steering, VmcApply (timed by the hook)
		internal long phLaws, phAsym, phAsymMom, phSlip, phSweep, phEnv, phMass, phFlaps, phGround, phVmc;

		// Su-30 0.4.0 (port of F-22E 4.1.3) (diagnostic): checkpoints through Tick (Stopwatch ticks), so its whole time is accounted for in 13 consecutive
		// segments (TickStats.SegNames); mkDone = this tick ran to its end. Drives nothing.
		internal readonly long[] mk = new long[14];

		internal bool mkDone;

		// Su-30 0.4.0 (port of F-22E 4.1.3): the segments of the last finished tick (us), the managed heap's growth over it (bytes, Hooks) and whether a
		// garbage collection ran during it (the CSV row of that tick, FinishRow)
		internal readonly float[] segUs = new float[13];

		internal long memDelta;

		internal int gcDuring;

		internal long memT0;

		internal int gcT0;

		internal float FlightTime => time;

		// Su-30 0.4.0 (port of F-22E 4.1.3): the performance switches as digits: Telemetry, NativeAllocator, NativeSweep, NativeTables, SweepGridCache,
		// ParallelTables, ParallelSweep, ParallelAllocator, EstimatorThreaded
		internal static int SwitchMask()
		{
			int m = Plugin.TelemetryEnabled ? 1 : 0;
			m = m * 10 + Plugin.NativeAllocator;
			m = m * 10 + (Plugin.NativeSweep ? 1 : 0);
			m = m * 10 + (Plugin.NativeTables ? 1 : 0);
			m = m * 10 + (Plugin.SweepGridCache ? 1 : 0);
			m = m * 10 + (Plugin.ParallelTables ? 1 : 0);
			m = m * 10 + (Plugin.ParallelSweep ? 1 : 0);
			m = m * 10 + (Plugin.ParallelAllocator ? 1 : 0);
			m = m * 10 + (Plugin.EstimatorThreaded ? 1 : 0);
			return m;
		}


		internal float EstLastWaitUs => (estJobF != null) ? estJobF.lastWaitUs : 0f;


		// F-22E 3.0.4: the estimator's probes on a worker thread (EstimatorJob); kicked where the estimator ran, joined at the top of
		// the next Tick
		private EstimatorJob estJobF;

		internal EstimatorJob estJob => estJobF ?? (estJobF = new EstimatorJob(capEst));

		// collect the estimator now (bench: before reading capEst after a Tick)
		internal void EstJoin()
		{
			estJobF?.Join();
		}

		// F-22E 3.0.4: the CSV row of tick N is captured at the end of tick N and finished at the top of tick N+1, once the estimator
		// job of tick N is joined (its columns) and tick N's time is known (fcsUs)
		private TelemetryRow pendingRow;

		// F-22E 3.0.4 tick timer (Hooks: Tick + VmcApply, wall time, us)
		internal float lastTickUs;

		internal void NoteTickTime(float us, long vmcTicks = 0L)
		{
			phVmc = vmcTicks;
			lastTickUs = us;
			phAlloc = alloc.solveTicks - allocTicks0;
		}

		private long allocTicks0;

		internal static float TicksToUs(long t)
		{
			return (float)((double)t * 1000000.0 / (double)System.Diagnostics.Stopwatch.Frequency);
		}

		private Vector3 velPrev;

		private bool havePrev;

		private Lpf alphaDotF;

		private Lpf aUpF;

		private Lpf aDnF;

		private Lpf qTdotF;

		private float alphaPrev;

		public float alphaU;

		public bool protHold;

		private float protHoldSide;

		public float alpha;

		public float beta;

		public float tas;

		public float ias;

		public float nz;

		public float omegaV;

		public float eLy;

		public float phi;

		public float theta;

		public float alphaDot;

		public float nAlpha = 0.1f;

		public float tw;

		public float nMaxAero = 9f;

		// 1.9.0: wing lift-to-weight ratio at the lift peak, thrust not counted
		public float nLiftMax = 9f;

		// 1.9.0 command shaping (AoA/G pre-filter)
		public float alphaCmdRaw;

		public float nCmdRaw = 1f;

		private float aRefV;

		private float nRefV;

		private bool shapeInit;

		// 1.9.0 AoA -> pitch-rate hand-over
		public float rateBlendAoA = 36f;

		// 1.9.7 negative side (magnitude), which side the rate command / floating hold belongs to, wing negative-lift capacity
		public float rateBlendAoANeg = 36f;

		private int hiSide = 1;

		private int floatSide = 1;

		public float nLiftMin = 9f;

		private bool rbInit;

		public bool hiRate;

		public float rateW;

		public float zoneX;

		private bool hiAnchor;

		public bool floatOn;

		public float floatS;

		internal static float FloatKnee = 2f;

		private float hiAS;

		private float hiAQ;

		// 1.9.0 low-thrust (nose-down authority) AoA cap
		public float authCap = 999f;

		public float authCapEff = 999f;

		// 2.0.9: negative AoA limit while the nozzles do not cover pitch (DisableTVC or low thrust), and the test-switch state
		public float authCapNegEff = -999f;

		private bool noTvcOn;

		// Ternion 0.1.1: no vectorable nozzle on this jet (see the constructor)
		internal bool noVecJet;

		private string testSwitchKey;

		private bool authInit;

		internal static float AuthCapLead = 1f;

		internal static float ReleaseTauG = 0.3f;

		internal static bool ReleaseShapeAoA = true;

		internal static bool MpoNoVecFullRate = true;

		internal static float AuthCapTauUp = 1f;

		internal static float AuthCapTauDown = 0.3f;

		internal static float AuthCapRise = 4f;

		internal static float AuthCapRiseAtCap = 1f;

		internal static float AuthCapThrustStep = 1f;   // deg/s^2 of nozzle nose-down authority gained between sweeps

		internal static float AuthCapFastHold = 1f;     // s the fast rise stays open after the last such gain

		private float authTvPrev;

		private float authFastT;

		public float alphaAtNMax = 30f;

		public float gAvail = 9f;

		public float gInt;

		public float vDot;

		public float alphaDotHoldG;

		private Lpf vDotF;

		public float aUp = 60f;

		public float aDn = 60f;

		internal static float RollLoadLead = 0.2f;

		// 2.1.10: allocator yaw weight above ~40 deg AoA while sideslip builds (YawAxisPriority below 20 deg, blended between).
		// The sideslip rate is p sin(AoA) - r cos(AoA): at 45 deg a body-roll error counts as much as a yaw-rate error, but
		// the allocator weighted yaw 8 : roll 0.35, so it let the stabs' yaw (their roll side-effect uncancelled, flaperons at
		// their stops) run the body roll 30 deg/s past its command (13 Oct log, 45 deg limit: sideslip 20-28 deg in rolls
		// and reversals at 70-100 m/s). With |sideslip + 0.3 s lead| over 2 deg the yaw weight comes down toward this value
		// (full at 8 deg); <= 0 = 2.1.9 (flat YawAxisPriority).
		internal static float YawPrioHi = 2f;

		// sideslip gate for YawPrioHi: 0 below YawHiSlip0 of |sideslip + lead|, 1 at YawHiSlip0 + YawHiSlipSpan (0 span = always 1)
		internal static float YawHiSlip0 = 2f;

		internal static float YawHiSlipSpan = 6f;

		internal static float YawHiSlipLead = 0.3f;

		internal static float YawHiSat0 = 0.6f;

		internal static float YawHiSatSpan = 0f;


		private float YawHiSlipGate()
		{
			if (YawHiSlipSpan <= 0f)
			{
				return 1f;
			}
			float bl = beta + slipRateDbg * YawHiSlipLead;
			float g = Mathf.Clamp01((Mathf.Abs(bl) - YawHiSlip0) / YawHiSlipSpan);
			if (YawHiSatSpan > 0f)
			{
				g *= 1f - Mathf.Clamp01((yawSat - YawHiSat0) / YawHiSatSpan);
			}
			return g;
		}

		internal static float RollStickRate = 10f;

		internal static float PreLag = 0.01f;

		internal static bool LegacyLag;

		internal static float RollJerkServo = 1f;

		internal static float RollJerkTau = 0.18f;

		internal static bool BrakeUseAll = true;

		internal static float RollEndJerk = 0.35f;



		// 2.1.0 sideslip-rate lead for the lateral guard (see LateralLaw)
		internal static float SlipLeadT = 0.8f;

		internal static float SlipRateTau = 0.05f;

		internal static float GuardCutTau = 0.08f;

		internal static float SlipGovStart = 4f;

		internal static float SlipGovSpan = 10f;

		public float slipGovDbg = 1f;

		private float latLagEff = 0.47f;

		private float latLagYawNom = 0.47f;

		public float slipLeadDbg;

		public static bool GLimitAccelLead = true;

		public float rateCmdG;

		private float stickLagF;

		public bool capOn;

		private float capSp;

		private float capQ;

		private float capSpPrev;

		private float capSteadyT;

		internal static float CapStickRate = 0.15f;

		internal static float CapSteadyTime = 0.15f;

		internal static float CapAoABand = 0.5f;

		internal static float CapAlphaDot = 2f;

		internal static float CapStickBand = 0.02f;

		private bool leadInit;

		public float stickLeadDbg;

		public bool latRevDbg;

		private float latRevSign;

		internal static float NegGateA0 = 0f;

		internal static bool LowCapLatFix = true;

		internal static bool AsymSharedLat = true;

		public float sharedLatDbg = 1f;

		internal static float LoadRevYawK = 0.4f;

		public float loadRevDbg;

		internal static float LowCapCoupBand = 4f;

		internal static bool CoupGateAtLimit = true;

		internal static float CoupLimBand = 4f;

		internal static float CoupGateRelease = 0.6f;

		internal static float CoupGateLoaded = 0f;

		private float coupGateHeld;

		private bool LowThrustCapOn => authFixedOn || authCapEff < Plugin.AoAPos - 0.5f;

		internal static float NegGateSpan = 3f;

		public float negGateDbg;

		internal static float YawAnchorK = 0.5f;

		internal static float YawAnchorT = 0.25f;

		internal static float YawAnchorFloor = 10f;

		internal static bool YawAnchorGrav = true;

		public float yawAnchorDbg = 999f;

		private Lpf bdkF;

		// Ifrit 0.3.5: sideslip predictor history (commanded rS less gravity, washed out) and its frame times
		private readonly float[] slipHistV = new float[256];

		private readonly float[] slipHistDt = new float[256];

		private int slipHistN;

		private int slipHistHead;

		private float slipHistLp;

		public float slipPredDbg;

		public float slipHoldDbg = 1f;

		private void SlipHistPush(float x, float dt)
		{
			float tw = Plugin.SlipPredictWashout;
			slipHistLp = (tw > 0f) ? (slipHistLp + (x - slipHistLp) * (dt / (tw + dt))) : 0f;
			slipHistHead = (slipHistHead + 1) % slipHistV.Length;
			slipHistV[slipHistHead] = x - slipHistLp;
			slipHistDt[slipHistHead] = dt;
			if (slipHistN < slipHistV.Length)
			{
				slipHistN++;
			}
		}

		private float SlipHistSum(float window)
		{
			float sum = 0f;
			float tt = 0f;
			int idx = slipHistHead;
			for (int k = 0; k < slipHistN && tt < window; k++)
			{
				float d = Mathf.Min(slipHistDt[idx], window - tt);
				sum += slipHistV[idx] * d;
				tt += slipHistDt[idx];
				idx = (idx - 1 + slipHistV.Length) % slipHistV.Length;
			}
			return sum;
		}

		private void SlipHistClear()
		{
			slipHistN = 0;
			slipHistLp = 0f;
		}

		public float slipRateDbg;

		// 2.23.0: stays 0.5. The sweep cap is the kinematic demand (a coordinated rotation pS needs pS cos AoA of body yaw per unit
		// AoA, see LateralLaw) against half the yaw capacity; it is the binder of every released push-over from 65 deg once the AoA
		// falls through ~55 deg (3 Nov 11:19 log, 47.5-48.1 s: 15-21 deg/s of AoA rate allowed against 20-23 of flight-path rotation
		// = a pitch command of +4..+9). 1 removes that and doubles the bench's bottom sideslip (clean released push 600: 9.2 -> 21.3;
		// 0.65: 13.3): the larger pitch demand takes the stabs as they return to pitch and the lateral axes lose the coordination.
		internal static float SweepYawK = 0.7f;   // 2.27.0: 0.5 -> 0.7. Since 2.26.0 the kinematic cap acts on the HELD push only (released: scaled out, the follow pace paces). Tom (14:28): "for the held push-over the pitch-down is too slow, the sideslip is being kept too tight" - 14:23 log held pushes 135-201 kt: sideslip <= 2 deg of the 4 deg margin, AoA rate 15-20 deg/s against +12..+23 of flight-path rotation. Bench (its held pushes do not reach this cap): one-fin held 560 5.7 -> 9.3, push+roll 300 2.7 -> 3.2, pull-then-pushover 350 4.2 -> 4.8, push-then-pullthrough 450 7.5 -> 8.3; the 2.23.0 bisect at 1.0 doubled the released push's sideslip when this cap still paced it.
		// 2.24.0: a cap on the sweep toward zero AoA may hold the nose, never turn it back: the pitch-rate command it leaves is bounded
		// at zero (the AoA still falls at the flight-path rate). Where the deficit is real (11:19 log, 223-244 kt with 70-80 deg/s on,
		// sideslip 5-6 deg and climbing) the honest AoA-rate cap of 14-19 deg/s sat under +20..+24 of flight-path rotation = a nose-UP
		// command of +5..+18 with the stick full forward - Tom's hitch, whatever the pace behind it.
		internal static bool SweepNoReverse = true;
		internal static float SweepNoReverseStick = 0.2f;   // lateral stick fraction over which 'released' fades to 'held' (the floor and the delivered term are released-push devices; the held push keeps 2.10.0's accepted pace)
		// 2.25.0: the floor on the held push too (Tom, 3 Nov 13:17: "sometimes it steps, sometimes it goes all the way to the hard stop then
		// goes again, for both the released push-over and the held push-over" - 13:12 log, 122.0 s held push: qCmd -25 -> -13 -> 0 -> +7 ->
		// -9 -> -36, body pitch rate +4 with the stick full forward). A positive pitch command with the stick forward is a hitch whatever
		// the rotation behind it; what the held push loses in the bottom sideslip the shed cap's pace has to buy back.
		internal static bool SweepNoReverseHeld = true;
		// 2.34.0 (the push governor): the push side of the reference-capacity governor replaces the 2.5.8-2.27.0 push-over stack (the kinematic
		// sweep cap and its delivered term, the shed cap and its follow pace, the crossing lead, the cap filters and slews, the released
		// follow latch, its roll target and its yaw brake). Tom: "it limits the lateral rate when only the yaw is the bottleneck and the
		// pitch rate when both yaw and roll are". A sweep toward zero AoA under a velocity roll pS drives the stability-axis yaw rate
		// rS = r cos a - p sin a at pS x AoA-rate; the yaw (through cos a) and the roll (through sin a) turn it back. The yaw sheds at
		// most capY: the velocity roll may come down no faster than (pS cos a AoA-rate - capY) / sin a (and must GROW when that is
		// positive: the roll follows the yaw) - the lateral limit. Where the roll cannot grow that fast either (capX), the AoA rate is
		// limited, and over the whole sweep the AoA may come no lower than atan(yaw left / body-roll ceiling) (released: 1.5 x the body
		// roll at the push; held: the stick's target), closed on at PushGovDecel. Capacities: one 0.2 s filter, TradeCapShare of them for
		// the pitch, all of the yaw for the lateral edge. Bench record: claude/Pushover_governor_2.34_dev_findings.md
		// 2.35.3: the stack it replaced (and the PushGov switch back to it) is deleted; it had not run since 2.34.0.
		internal static float PushGovCapTau = 0.2f;   // s: the governor's one capacity filter (the pace is one curve, not the estimator's tick-to-tick read - 2.25.0)
		internal static float PushGovTau = 0.5f;      // s: the pace closes on the yaw-limited AoA over this time (when PushGovDecel = 0)
		internal static bool PushGovNoStab = true;   // 2.34.1: plan the push on the yaw capacity without the stabs (see LateralLaw)
		// Su-30 0.5.1 (Tom 11 Nov 17:26: "I release roll and then the roll rate drifts a lot", 17:22 log, 16 cm of lateral CG offset, nozzles
		// locked): the no-stab read is for a push the pitch law COMMANDS - there the stabilators go to pitch. With the pitch stick centred the
		// pitch command is zero body rate, the AoA comes down at the flight path's own 12-15 deg/s and the stabs are free; the allocator does
		// use them for yaw (6-16 deg of differential in the stop). Planned without them, the yaw toward the load's side read 0-12 deg/s^2,
		// the sweep alone took it (44 deg/s of roll x 15 deg/s of AoA rate = 10), and the released roll was not allowed to brake: 109.0 /
		// 117.8 s, 36-44 deg/s held for 2.3-2.6 s after the stick was centred, 71-90 deg of bank; the other way 1.1 s.
		// With the roll stick released the discount is now weighted by the nose-down body rate the pitch law commands (0 at none, full
		// from PushGovNsQ deg/s); with the roll stick held it stays whole (bench, clean, push-then-pull-through + roll at 450 km/h: without
		// that the peak sideslip went 12.9 -> 16.1 deg).
		// PushGovNsByPitchCmd = false: 0.5.0.
		internal static bool PushGovNsByPitchCmd = true;
		// Su-30 0.5.1: with the roll stick released the governor may slow the roll's brake but not make the roll grow, and the brake keeps
		// PushGovRelBrakeMin deg/s^2. Where the yaw toward the braking side reads zero (the load takes it) the band's lower edge L is
		// positive and the roll was re-aimed up to PushGovRollK x the body roll at the sweep's start, 15 deg/s of body roll at least: both
		// sticks centred at the 50 deg limit, 104 kt, 16 cm of offset, nozzles locked, the roll went 17 -> 19..26 deg/s by itself and ran
		// 46-83 deg of bank (bench); 17:22 log 155.4 s: 35.9 -> 39.0 deg/s in the 0.5 s after the release. Tom, on the released stick
		// under a push: "the roll stops, a little slower is fine". PushGovRelNoGrow = false: 0.5.0.
		internal static bool PushGovRelNoGrow = true;
		internal static float PushGovRelBrakeMin = 10f;
		internal static float PushGovNsQ = 10f;
		internal static float PushGovSlipK = 0.5f;    // x the PushoverSideslip allowance the pace may leave to the bottom
		// Su-30 0.2.4: the governor (unplanned sweeps toward zero AoA, pull-throughs included) keeps the 0.2.3 allowance (8); 15 there let a
		// pull-through with roll at 450 km/h reach 18 deg of sideslip (bench reg, 12 before). The 12 / 15 allowances are the push plan's.
		internal static float PushGovSlipMax = 8f;
		internal static float PushGovDecel = 40f;     // deg/s^2: the pace closes on a_min at this deceleration (sqrt(2 D gap)); 0 = over PushGovTau
		internal static float PushGovRollK = 1.5f;    // the body roll may rise to this x the body roll it had when the sweep began
		// 2.35.0 (PushPlan, Tom 22:19: "velocity vector roll bleeds while the pitch is smoothly slowed down for the push over"). The push
		// is PLANNED once, when it starts, not re-paced every tick from the capacity read (2.34.0/2.34.1: a pace that follows a read
		// swinging +-30 % in 0.3 s, sitting at the flight path's own 15-20 deg/s of rotation, hesitated). At the push: the yaw on board
		// (less what PushPlanSlip deg of sideslip absorbs at the bottom) over the yaw the push will shed - from the logs, 77 push-overs
		// 14:45-22:06, the mean shed from the push to 10 deg AoA was 28.6e-4 x IAS^2 (r 0.81 with IAS; the capacity reads 0.53-0.64)
		// - gives the push time T; the body pitch rate is set to bring the AoA to 10 deg in T (less the flight path's rotation), ramped
		// in at PushPlanJerk and then held; it may rise (never fall) at PushPlanRise when the yaw goes faster than planned. With the
		// lateral stick released the velocity-vector roll bleeds to zero over the same T (not the stop law's brake). One number to tune
		// the trade: PushPlanK (higher = faster push, more sideslip at the bottom).
		internal static bool PushPlan = true;
		// Compass 0.1.8: off (Revoker 0.7.0's request, inherited). Tom 5 Nov 20:51: "held roll and pitch, then move the stick to the other side
		// while holding roll, the roll stops when I get to low AoA then starts again when AoA builds up" (20:25 log, 29.5-30.7 s: plan on
		// from 20 deg, roll target 0 to 0 deg AoA, p 52 -> -3, back to 110 at -1 deg). The Compass's rudders hold the push's yaw at 35 deg:
		// bench, full aft + roll then full / 0.6 push at 250-430 km/h, held: roll kept 25-34 deg/s (stop: -2..-17, i.e. reversing),
		// sideslip 1.4-3.8 deg (stop 1.6-4.8). The held stick now gets PushPlanHeld's paced roll.
		internal static bool PushPlanStopsRoll = false;
		internal static bool RudderReach = true;   // Revoker 0.7.0: the allocator's reach pass on the rudder (see the reachTau loop)   // Revoker 0.7.0: the plan bleeds a held roll as well as a released one
		// Revoker 0.6.0: shed = PushPlanK0 + PushPlanK x IAS^PushPlanExp (deg/s^2, IAS m/s). The F-22E's shed is all aero (K0 0, K 0.028 from
		// 93 push-overs). This jet's yaw at the top of a push is the yaw nozzle (thrust, no fade with speed) and the elevons on the sideslip
		// axis: the 4 Nov 23:03 / 23:13 logs' 4 push-overs shed about the same at every IAS, so the fit is a constant: K0 40, K 0 (residual
		// 3.9 deg/s^2 rms). Provisional: 4 samples, flown on 0.5.1. Refit as more push-overs are flown.
		// Su-30 0.2.4: refit from the Su-30's own push-overs (the Revoker's constant 40 had never been refit for this jet). From the
		// 9 Nov 23:37 - 10 Nov 14:55 logs, the planned pushes' mean yaw shed from the push to 10 deg AoA: with the nozzles working
		// (19 pushes) 0.0224 x IAS^1.5 (K0 0.2, rms 8.9 deg/s^2: 11 at 60 m/s, 16 at 80, 23 at 100); with TVC off (15 pushes) about 4-6
		// at any IAS, no trend (the fins alone; 2 x 2 + 2 x 1.5 m^2). The plan against 40 asked for 60 deg/s pushes the yaw could not
		// follow: 12-29 deg of sideslip at the bottom (14:55 log). With TVC off the shed sits under the 6 deg/s^2 clamp, so those
		// pushes plan slow and run on the PushOverMinRate floor.
		// Su-30 0.3.0 (Tom 10 Nov 16:46: "push overs with tvc on didnt use all of the allowed sideslip budget so the push overs where kinda
		// slow"): K 0.0224 -> 0.028 (x1.25). The 0.2.4 fit was the mean shed of pushes the plan itself had paced, so it read the plan's own
		// slowness back. Bench replays of the 23 TVC-on planned pushes of the 14:55 and 16:16 logs, PushoverSideslip 15: median time to
		// 10 deg AoA 1.85 -> 1.72 s, mean 2.33 -> 2.13 (Tom flew 2.17 / 2.54 with the file's 8); median peak sideslip after the bottom 7.0 ->
		// 8.3 deg, worst 35.5 -> 19.5, 3 -> 5 of 23 past 15. x1.5 and x2 were no faster at the median and put 7-9 past 15. Known: the bench
		// held push-over at 560 km/h reaches 18.6 deg (9.8 at 0.0224) - the yaw sheds slower than the IAS^1.5 law says at 100+ m/s.
		internal static float PushPlanK0 = 0f;
		internal static float PushPlanK = 0.028f;
		internal static float PushPlanExp = 1.5f;
		internal static float PushPlanK0NoTvc = 6f;
		internal static float PushPlanKNoTvc = 0f;
		// Revoker 0.6.0: the plan's sideslip allowance at the bottom is the push-over allowance Tom set for the stock jets (PushoverSideslip, 8;
		// the F-22E plan uses 4). -1 = follow it.
		internal static float PushPlanSlip = -1f;
		internal static float PushPlanA0 = 20f;       // deg: the push is planned only from above this AoA
		internal static float PushPlanQMin = 8f;      // deg/s: the nose always visibly moves
		internal static float PushPlanJerk = 120f;    // deg/s^2: the planned rate is ramped in at this
		internal static float PushPlanRise = 15f;     // deg/s^2: how fast it may rise when the yaw is ahead of the plan
		internal static float PushPlanTMin = 0.6f;
		internal static float PushPlanR0 = 15f;       // deg/s of body yaw on board at the push for it to be planned
		internal static float PushPlanWvK = 1f;       // the flight path's rotation at the push counted against the planned AoA rate
		internal static float PushPlanRelease = 60f;  // deg/s^2: the bound ramps out below 10 deg AoA
		internal static bool PushPlanOwnsPitch = true;    // 2.36.0: under a plan the sweep caps (the 60 deg/s AoA-rate ceiling) stand aside; with PushPlanClipKey the allocator's pitch priority and the stabs' window read the plan's bound (2.35.1's try without ClipKey: 4.8 -> 11.2)
		internal static bool PushPlanWinOff = true;
		internal static bool PushPlanClipKey = true;      // 2.36.0: pitchNeedDn / sweepHoldWin read the plan's bound (see the pitch law)
		// 2.40.0 (PushPlanHeld; 2.36.0's held-stick rule): a planned push owns the held lateral stick too, in place of the push governor's
		// lateral band. The band paced the roll toward the stick's target by L = (pS cos a a' - capY) / sin a with capY the measured yaw
		// capacity read, which swings +-30 % in 0.3 s (the 2.34.1 noise). Under a plan the same rule reads the plan's own yaw budget D,
		// fixed at the push's start - D = max(capPlan, (Y0 - Yend) / T): the coordinated yaw of the rotation, Y = pS sin a, drained to its
		// end (the held roll's coordinated yaw at 10 deg AoA) over the planned push time - and this tick's commanded AoA rate a'.
		// Released stick: unchanged (the plan's linear bleed). 2.36.0 flown 09:03: "held push over is smooth".
		internal static bool PushPlanHeld = true;
		// 2.41.0 (PushPlanReachTau): through a planned push the allocator places the nozzles and wing surfaces against where each stab will
		// be within this many seconds (its servo's reach toward its command), not against the command (Allocator.ReachPass). 11:01 log
		// (2.40.0), 34.1 s: the window sent the left stab -25 -> +35 deg (0.45 s, through its lift peak); the allocator, seeing the stab
		// already at +35 (nose-up), drove the nozzles -4 -> -9 deg nose-down while the stab gave its peak nose-down on the way: -34 deg/s
		// flown against -26.8 asked. Hybrid replays, 59 released pushes: overshoot 6.8 -> 2.8 deg/s median, the mid-push step 3.9 -> 1.3,
		// the body-roll rise 9.3 -> 4.8; sideslip 5.0 -> 7.4 median (the stab's transit roll pulse had been holding it down). 0 = off.
		internal static float PushPlanReachTau = 0.1f;
		// 2.44.0 (StickCancelsLimit, Tom 4 Nov 20:26): "if i hold stick all the way back at max aoa, and the jet still can do positive pitch
		// rate, then nothing happens; as soon as the only pitch rate that can hold that aoa is 0, that is the point the stick can cancel the
		// protection". At the AoA limit the nose-down part of the limit's own command (the capture bound below the limit, protection
		// above it) scales with 1 - stick / 0.9, as protection's push always did; a positive holding rate is never scaled. Before, two
		// rules disagreed at the limit: below limit + 0.5 the aft-stick floor held the command at 0, above it 1.9.5's 'pilot command'
		// (the stick clamped by the capture bound) brought the bound's nose-down back through full aft. Slowing at 22-50 IAS with the
		// flight path falling 1-8 deg/s (20:07 log, 143-154 s, 173-182 s) the command cycled 0 <-> -11..-13 deg/s at ~2 Hz, AoA
		// 64.5-66.6, nozzles -13 <-> +16 deg. Now full aft holds zero rate there and the AoA rises with the falling flight path (bench,
		// those segments: 112 / 85 deg at their ends); release and protection pushes. 1.9.5's rule keeps its purpose: a stick asking for
		// more nose-down than protection (forward stick above the limit) still gets it.

		internal static float SweepPs0 = 10f;


		internal static float SweepFloor = 6f;





		internal static bool SweepPosPullFree = true;



		internal static float SweepPosK = 0.25f;

		internal static float SlipGovPs0 = 10f;





		internal static float PullRollTradeFinLoss = 0.7f;









		public float decelDbg;

		private float decelF;

		// Ifrit 0.4.0: split-deceleron yaw tier gate (0..1), engine out / damaged yaw axis only
		private float splitTierF;

		public float splitTierDbg;

		public float tierDemDbg, tierReachDbg, tierSplitDbg;

		internal static float SplitTierRise = 3f;

		internal static float SplitTierFall = 1.5f;

		internal static float SplitYawClose = 0.6f;

		internal static float SplitYawSatOpen = 0.05f;

		internal static float SplitYawSatSpan = 0.6f;

		private float decelYieldF;

		internal static float DecelYieldFall = 1f;

		internal static float DecelFightRelax = 1f;

		internal static float DecelStabRelax = 0f;




		internal static float DecelRise = 3f;

		internal static float DecelFall = 1.5f;

		private float pullGovF = 1f;

		// 2.2.24 adaptive feed-forward in the rolling-pull governor
		private float pullFFBoost = 1f;

		public float pullServoDbg;

		private float pullServoEx;






		public float pullServoWDbg;







		internal static float RelLatRollPrio = 3f;

		public float relLatPrioDbg = 1f;






		public float pullXsDbg;

		public float pullLeadDbg;





		internal static float PullFFBoostIdle = 2f;




		// 2.2.8 governor state
		internal static float PullGovIdleRise = 4f;

		internal static float PullTradeFloor = 30f;

		// 2.4.17 roll-stop sideslip governor (see LateralLaw)
		internal static float StopSlipYawUse = 1f;



		internal static float YawSlewLeadK = 1f;

		public float yawLeadDbg;

		private float rCmdRateF;



		private bool rCmdLeadInit;

		// 2.5.0 reference governor on the commanded roll rate (see LateralLaw). The stop / reversal sideslip model fitted
		// on the 30 Oct flight logs (200 stops and reversals, 3-40 deg AoA): peak sideslip after the stop = StopSlipBase +
		// |pS| sin(AoA) cos(AoA) x (StopSlipTau0 + StopSlipTau1 x rudder load above StopSlipKnee), with the rudder load the share
		// of the rudders' yaw on the roll's side (StopSlipHoldTau filter).
		internal static float StopSlipBase = 2.0f;

		internal static float StopSlipTau0 = 0.054f;

		internal static float StopSlipTau1 = 0.22f;

		internal static float StopSlipKnee = 0.7f;

		// 2.5.2 yaw-reserve governor: rudder load (share of their yaw on the roll's side) above which the roll reference is
		// trimmed, and below which it is released; the release rises at StopSlipReserveRiseK x the fall rate
		internal static float StopSlipLoad = 0.8f;


		internal static float StopSlipReserveRiseK = 0.5f;


		// 2.21.0: the AoA-rise cap (StopSlipPitchK / Start / Lead / Close, StopSlipPinAB0 / 1) is deleted - see LateralLaw. The
		// floor stays as the trade's pitch-share floor (deg/s of AoA rate)
		internal static float StopSlipPitchFloor = 6f;

		private float paceRDotLast;

		// 2.6.4: the trade's split. PullRollTrade is the share of a rolling pull's yaw shortfall the roll gives up (the roll limit
		// goes that far from pSmax toward the trade's target); the pitch gives up the rest - the AoA-rate ask is scaled so that the
		// pair's sweep demand (pS cos AoA x AoA rate) fits the share: with the roll at R_r of pSmax and the demand needing R of it,
		// the AoA rate is held to R / R_r of the ask. 1 = all roll (2.5.1-2.6.3: the remainder was simply uncovered - at 170-210 kt
		// the roll sat at the 30-35 deg/s floor with the pitch at full rate, Tom: "the slower, the more pitch and less roll; not the
		// way the old governor divided pitch and roll with the 0.7 trade"; 22:17 log); 0 = all pitch; your 0.7 = 70 / 30.
		private float pullTradeAsk;
		internal float tradeAlphaRate = -1f;

		// 2.8.0: the trade from the measured capacity (CapacityEstimator) against the roll the stick asks, not the roll limit
		// (15:02 log, 210.3 / 215.0 / 224.1 s: stick released, roll braking to zero, yaw idle, and the pitch still held to 30 deg/s
		// for ~0.35 s while the slewed roll limit climbed back - the re-pull hitch). 2.10.0: the 2.7.x arithmetic is deleted.

		// share of the measured yaw capacity a rolling sweep may spend before the trade cuts in
		internal static float TradeCapShare = 0.7f;

		// the AoA-rate cap the measured trade asks of the pitch this tick (-1: none), and the roll the stick asks before any cap
		internal float tradeAlphaNew = -1f;

		internal float pSAskStick;
		public float tradeAlphaDbg;


		internal static float StopSlipPitchFree = 60f;

		private float stopSlipAbsF;

		internal float yawSatAlphaRate = -1f;

		// 2.5.5: the lateral governors (margin cap, yaw reserve, pace, pitch caps, trade, no-chase) act in the AoA/G and rate laws
		// and, with the gear down, in the air (no wheel on the ground; GearDownSlipProtect as for the 2.4.13 limiters). 10:10 log:
		// 51 s gear-down airborne, 20.9 deg of sideslip, every governor column idle.
		private bool LatGovMode => Mode == FcsMode.AoAG || Mode == FcsMode.Rate || (Mode == FcsMode.GearDownRate && Plugin.GearDownSlipProtect && !gsAnyContact);

		private float yawSatCapQ = 120f;

		private float yawSatCapQLo = -120f;

		internal float yawSatAlphaRateDn = -1f;

		private bool yawSatCapOn;

		public float yawSatPitchDbg;
		private float sweepClipDbg;   // 2.36.0: the push's clip (ask - command) by the caps or the plan (see PushPlanClipKey)
		private bool sweepClipOn;

		// margin schedule: RollStopSideslip up to StopSlipAoALo, RollStopSideslip50 at 50, RollStopSideslip65 at 65, off past StopSlipAoAEnd
		internal static float StopSlipAoALo = 10f;

		internal static float StopSlipAoAEnd = 70f;

		internal static float StopSlipLimMargin = 5f;

		// the limit comes down at StopSlipFallK x (the rudders' braking yaw acceleration / sin AoA), within StopSlipFallMin..Max
		// deg/s per s, so a falling limit is a roll the yaw can coordinate, never a stop of its own
		internal static float StopSlipFallK = 0.35f;

		internal static float StopSlipFallMin = 20f;


		// while the trajectory is still accelerating into the roll (faster than this, deg/s^2) the limit does not come down:
		// the rudders pin in every fast entry and unpin as the yaw catches up

		// yaw-saturation hold (see LateralLaw)
		internal static float StopSlipHoldTau = 0.08f;

		public float stopSlipUDbg;

		public float stopSlipPredDbg;

		public float stopSlipMarginDbg;

		private float rCmdCoordPrev;


		public float noChaseDbg;

		// no-chase and the yaw slew lead fade out over this AoA band (the one-axis high-AoA law owns the yaw there)
		internal static float StopSlipA0 = 36f;

		internal static float StopSlipA1 = 45f;


		internal static float StopSlipRiseK = 1.5f;


		public float stopSlipLimDbg = 999f;

		public float stopSlipABDbg;

		// 2.5.3 yaw-reach (pace) governor, RollGovernorReplaces >= 2 (see LateralLaw, after the roll trajectory step). The lateral
		// pair's body-yaw demand (pS sin AoA + the sideslip correction's cos AoA share) may not get further from the measured yaw
		// rate than the yaw axis can close within PaceHorizon: the yaw acceleration the jet is showing now (PaceAccelTau filter)
		// times the horizon, plus what the rudders can add within it from where they are (their tables, slew-limited, at
		// PaceRudderUse), plus PaceFloor deg/s for the loop's normal tracking error. The roll reference is clamped to that window.
		internal static float PaceHorizon = 0.15f;

		internal static float PaceFloor = 4f;

		internal static float PaceRudderUse = 1f;

		internal static float PaceAccelTau = 0.08f;

		// below this |sin AoA| the roll's yaw share is too small for the window to mean anything
		internal static float PaceSinMin = 0.05f;

		internal static float PaceRollUse = 1f;

		// Revoker 0.3.0: with the yaw and roll windows disjoint, the window of the axis that can move less toward the target
		// stands (true); false = the F-22E 2.6.6 rule (the yaw window stands)
		internal static bool PaceWeakAxis = true;

		// Revoker 0.3.0: the roll window's reach from the roll effectors' tables and slews (true); false = F-22E 2.6.6 (aRollPos x horizon)
		internal static bool PaceRollSlew = true;

		// Revoker 0.3.0: the roll effectors' slew in use opens the pace window (true; telemetry rollGate)
		internal static bool PaceRollGate = true;

		private float[] rollEffPrev;

		private bool rollEffInit;

		private float rollLoadF;

		public float rollLoadDbg;

		// open-loop: share of the rudders' available yaw the trajectory's acceleration / braking may ask for (level 3)
		internal static float PaceYawUse = 0.85f;

		private float paceAccelCap = -1f;

		// 1 = the open-loop cap / the rudder-usage gate fade out over the rudder fade (StopSlipA0..A1); 0 = they stand at every AoA


		// 1 = the open-loop cap hands over to the one-axis law as the rudders fade (rudderAuth); 0 = 2.5.5 (cap at every AoA)

		// false = keep the margin cap and the yaw-reserve governor under the pace governor (2.5.6 and before), for A/B
		// 2.5.15 sideslip-tolerance probe: the airframe's own aero (every AeroPart at the game's per-part law, the surfaces at
		// their present deflection) evaluated at the present AoA and a sweep of hypothetical sideslip, every SlipProbePeriod.
		// Two bounds come out: the sideslip where the restoring yaw moment stops growing (directional stiffness gone - the
		// departure boundary) and the sideslip where the rudders' or the roll surfaces' authority has fallen to SlipAuthFrac of
		// its zero-sideslip value. The smaller, x SlipTolUse, caps the margin. Nothing entered: the tables are the craft's.
		internal static float SlipProbePeriod = 0.5f;
		internal static float SlipProbeStep = 4f;
		internal static float SlipProbeMax = 20f;
		internal static float SlipTolUse = 0.7f;
		internal static float SlipAuthFrac = 0.5f;
		internal static float SlipAuthUse = 0.5f;
		private float slipSignR;
		internal static float SlipTolTau = 0.5f;
		private float slipProbeTimer;
		private float slipTolF = 25f;
		public float slipTolDbg = 25f;
		public float slipDepDbg = 25f;
		public float slipAuthDbg = 25f;
		public float slipStiffDbg;

		// 2.31.2 (telemetry only, drives nothing): the passive airframe, evaluated per part as the game does (AirframeMomentAt).
		// psA*: the airframe's own roll / yaw acceleration now (surfaces where they are, no thrust; deg/s^2, x roll right, z yaw right)
		// psD*: how that changes if the body rates go from (p, r) to this tick's (pCmd, rCmd) - the help (or opposition) the
		//       measured term does not contain yet; cePass*: psD as capacity toward each side, and along the velocity-roll axis
		// psL* / psN*: roll / yaw derivatives at the present state, per deg of sideslip (b) and per deg/s of p and r (0.1 s cadence)
		internal static bool PassiveProbeOn = true;
		internal static float PassiveDerivPeriod = 0.1f;
		public float psAX, psAZ, psDX, psDZ, psLb, psNb, psLp, psNp, psLr, psNr;
		public float cePassXP, cePassXN, cePassZP, cePassZN, cePassSP, cePassSN;
		private float psDerivTimer;
		private List<(ControlSurface cs, AeroPart host, AirfoilTable table)> psCamber;
		private readonly Dictionary<AeroPart, float> psCdMin = new Dictionary<AeroPart, float>();
		// 2.5.8 rate mode (RollPaceRateMode): the lever is the commanded roll rate, not the trajectory's acceleration or braking.
		// The roll-rate target is capped at the rate whose stick reversal, run at the tuned acceleration / braking, the yaw can
		// follow within the sideslip margin: the reversal swings the yaw demand by 2 pS sin AoA; the rudders first cross from
		// where they are (slew) and then pull at their full yaw on the new side; the yaw-rate error that accumulates meanwhile,
		// x cos AoA, is sideslip. PaceRateK scales the predicted sideslip (1 = kinematic, no credit for the airframe's own
		// directional stiffness); PaceRateFloor is the least rate the cap leaves (deg/s).
		internal static float PaceRateK = 1f;
		internal static float PaceRateFloor = 10f;

		// 2.6.9: the rate cap's blend weighted by rudderAuth (its model's validity); false = 2.6.8 (blend at every AoA)


		// 2.6.9: the rate cap models the ramped reversal the law flies (false = the 2.5.8 step model)
		internal static bool PaceRateRamp = true;

		// bench: scale on the cap's rudder-crossing dead time (1 = 2.6.9)
		internal static float RateCapTauK = 1f;

		// 2.6.9: the cap's dead time is the travel to the deflection the reversal needs, not to the far stop
		internal static bool RateCapNeedTravel = true;

		// 2.6.11: the pace window reads the whole airframe's yaw - its usage for the gate, every effector's travel for the reach - at every AoA (false = rudders)

		internal static bool PaceMarginReach = true;

		// bench: false = no open-loop acceleration cap at any blend (the trajectory always runs at its tuned acceleration / braking)
		internal static bool PaceAccelCapOn = true;

		// 2.7.0 measured pace window (PaceMeasured). One governor on the commanded roll rate, from measured state and the
		// allocator's own output - no effector tables beyond the deflections the allocator has actually commanded, no surface
		// list, no rate cap, no blend, no open-loop acceleration cap. Each tick, for the yaw axis and the roll axis:
		//   what the axis will have done by the end of PaceHorizon = its measured acceleration now (everything acting on the
		//   airframe, passive and active) plus the increment still in flight toward the effectors' commands, as far as their
		//   servos get within the horizon (the tables at the commanded deflection: the allocator's decision, not a hypothesis);
		//   the sideslip margin still unused, spent at that acceleration (sqrt(2 a (margin - |beta|) / cos AoA) of yaw-rate lead);
		//   PaceFloor for the loop's ordinary tracking error.
		// The roll target may not ask the axis for more than that - but only while the axis measurably cannot keep up: the
		// allocator reports the target unmet at the commands it gave (allocUnmet), or the commanded increment is further away
		// than the servos reach within the horizon (slew). In a coordinated roll, a corkscrew, the falling leaf, nothing binds
		// (no steady-state tax). Where an axis is out - rudders crossing their travel in a reversal, the stabs busy with the
		// pull - the target is held to what the axis delivers and the trajectory still runs at its tuned acceleration / braking
		// toward it (Tom's rule: the commanded rate is the only lever).

		// deg/s^2 of unmet-plus-undelivered acceleration at which the gate starts (PaceSatDead) and is fully on (PaceSatDead + PaceSatSpan)
		internal static float PaceSatDead = 5f;

		// 2.7.2: the unmet acceleration, as a share of the axis's authority, at which it counts as fully saturated
		internal static float PaceSatFrac = 0.25f;

		internal static float PaceSatSpan = 40f;

		// filter on the allocator's unmet report (s)
		internal static float PaceSatTau = 0.05f;

		// share of the axis's demonstrated acceleration (a decaying peak of |omega-dot|, PaceSeenTau) credited where nothing is
		// yet committed: the margin lead and the release pace after a hold
		internal static float PaceSeenUse = 0.5f;

		internal static float PaceSeenTau = 3f;

		// 2.7.1: lead allowed over the axis as a time at its capacity (s) - the sideslip rate accepted while the roll waits for it
		internal static float PaceLeadT = -1f;   // 2.17.0: < 0 = 1 / KYaw (0.4 s at the default gain); 0.1 = the 2.7.1 value

		private float paceRatePrev;

		private bool paceRateInit;

		// 2.7.0: the trajectory's acceleration law reads the stick's intent (a reversal is a reversal even while the window
		// holds the target on the near side), so a governed reversal brakes at the reversal's tuned acceleration, never the stop's
		internal static bool PaceRevIntent = true;

		public Vector3 allocUnmet;

		private Lpf3 allocUnmetF;

		public float aYawSeen;

		private float aRollSeen;

		public float paceGateZDbg;

		public float paceGateXDbg;

		public float paceDzHDbg;

		public float paceDzAllDbg;

		public float paceDxHDbg;

		public float paceDxAllDbg;

		public float paceRHDbg;

		public float pacePHDbg;

		public float paceTgtRaw;

		public float paceSeenZDbg;

		private float paceBetaG;

		public float paceMLeftDbg;

		public float paceLoDbg;

		public float paceHiDbg;

		public float paceClampDbg;

		// 2.20.0: the window's no-fin stop (the roll target held at the stopped-body-roll rotation r sin AoA; the yaw loop asked for its brake)
		// Revoker 0.6.0: off on this jet. The F-22E device assumes that with the fins gone there is no yaw axis left but the stabs; here
		// yawHealth reads the rudder and the fins, while the yaw nozzle and the elevons (the yaw axis at high AoA: elevon pair +-68 deg/s^2,
		// rudder +-12 at 350 km/h / 70 deg) are untouched. Bench damage suite, rudder or fins+rudder detached: with it on, 4 held / released
		// push-overs departed (sideslip 41-52 deg, AoA to -80..-93 after the release at -50) and the leaf stops made 13-17 deg against 0.5.1's
		// 2-3; off, every damaged case is back to 0.5.1's (worst +0.8 deg), clean cases identical. Inert on the Ifrit's bench either way.
		internal static bool PaceStopNoFin = false;

		private float paceStopNF;

		public float paceStopNFDbg;

		public float paceCapZPDbg;

		public float paceCapZNDbg;

		public float paceCapXPDbg;

		public float paceCapXNDbg;

		public float paceHdZPDbg;

		public float paceHdZNDbg;

		// 2.7.2: passive directional stiffness credited to the reversal cap, as a share of the restoring yaw at the margin sideslip
		internal static float PaceStiffUse = 1f;

		// 2.7.1's target rate limit at the capacity (Tom, 10:19: felt as slowed deceleration - off)


		internal static float PaceGateOn = 0.5f;

		internal static float PaceGateOff = 0.25f;

		private bool paceLatchZ;

		private bool paceLatchX;

		public float paceNEffDbg;

		public float paceDdDbg;


		private float revBlendF;

		internal static float PaceMarginK = 1f;

		public float paceMarginLeadDbg;

		// 2.6.9: the pace window's gate reads the whole airframe's yaw usage past the rudder fade (false = rudder load only)

		private float yawUseAllF;

		private float rateBlendEffDbg;
		// 2.5.12: the yaw the cap credits is derated by the allocator's own unmet-yaw report (yawSat, 0..1 over 10-50 deg/s^2
		// unmet, 0.25 s filter): by PaceSatDerateHi of it past the rudder fade (the stabs are the yaw axis there and the tables
		// overstate them 3-4x: 13:31 log, 55 s - a 64 deg/s roll allowed at 65 deg AoA, the release left 60 deg/s of yaw the
		// axis shed at 25 deg/s^2, 11 deg as the AoA fell through 30), PaceSatDerateLo below it. "Not too much" (Tom).
		internal static float PaceSatDerateHi = 0.5f;
		internal static float PaceSatDerateLo = 0.25f;




		// 2.6.6: 1 - cos AoA at which the shed cap stands at full strength (0.5 = 60 deg); the pace eases below it as holding buys less
		internal static float SweepShedCos = 0.05f;   // 2.8.0: 0.5 -> 0.05, the cap stands at every AoA (Tom, 2 Nov: smooth over fast in the push-over)

		// weight of the allocator's unmet yaw (yawSat) in the window's gate

		// the window's gate releases over this (s); the rudders read zero usage for a few ticks as they swing through center
		internal static float PaceGateTau = 0.3f;

		private float paceGateF;

		private float paceGateXF;
		// 2.6.1: the window's release is paced by the yaw. The window holds the target while the rudders are loaded (PaceLoad0..1 of
		// their yaw, 0.08 s filter) and let it go in one tick when they unloaded, into the full trade cap at the blended acceleration
		// (300-720 deg/s^2 at blend 0.41): the yaw demand jumped, the rudders reloaded, the window held again - a 1.9 Hz hold /
		// release cycle, +-5 deg/s on the target and +-5 deg on the rudders, in every corkscrew entry at 20-50 deg AoA once the
		// 2.5.14 trade share gave the target room above the window (bench; 14:55 / 15:10 logs: pSTarget chatter 0.7 -> 3.0-3.7
		// deg/s from 2.5.11). The hold may still come on in one tick (that is the protection); the climb back out of it may not
		// ask the yaw for more than PaceYawUse of what the rudders can add toward the move's side (their tables, from where they
		// are): roll-target rise x sin AoA = yaw acceleration. The trajectory's own acceleration and braking are untouched; this
		// is the commanded rate, climbing out of a hold at the yaw's pace instead of in a step. PaceReleasePace 0 = 2.6.0.
		// Only a hold with the roll already going the target's way is paced out (paceWinSame): a stick reversal's hold (the
		// window keeps the new target at zero while the roll brakes through) is released as before, so the reversal still runs at
		// the tuned acceleration (pacing it too took 450 km/h / 12-19 deg AoA reversals from 0.3 to 0.7-1.0 s on the bench).
		private bool paceWinHeld;
		private bool paceWinSame;
		private float paceWinPrev;
		public float paceRelDbg;
		internal static float PaceReleasePace = 1f;

		// the window applies as the rudders' usage (either side) runs from PaceLoad0 to PaceLoad1 of their yaw
		internal static float PaceLoad0 = 0.5f;

		internal static float PaceLoad1 = 0.75f;

		private Lpf paceRDotF;

		private Lpf pacePDotF;

		private float paceOmegaZPrev;

		private float paceOmegaXPrev;

		private bool paceInit;

		public float paceLeadDbg;

		public float paceHoldDbg;
		public float dbgWinLo, dbgWinHi, dbgWinLoR, dbgWinHiR, dbgReachPos, dbgReachNeg, dbgYawCorr, dbgWSat, dbgRateCap, dbgWin, dbgRollPos, dbgRDot, dbgPDot;

		internal static float PullTradeFFK = 0.3f;
		// 2.5.14: the share of the yaw authority an AoA sweep under a roll may spend, of the part the rudders are not using
		internal static float PullTradeFreeK = 0.35f;

		internal static float PullTradeFFIas0 = 93f;

		internal static float PullTradeFFIasSpan = 25f;

		internal static float PullTradeFall = 300f;

		internal static float PullTradeRise = 120f;

		internal static float PullTradeCutA = 15f;








		private bool pullGovTouched;


		internal static float GovCapTau = 0.3f;



		private float pullTradePs = 999f;

		private float pullTradeTgt = 999f;

		public bool pullTradeHoldDbg;

		internal static float PullTradeRiseHold = 0f;

		public float pullTradeDbg;

		public float slipGovPullDbg = 1f;






		internal static float ToeLatRelease = 0.5f;

		private float toeLatGate;

		public float toeTgtF;


		public float coordJerkDbg;

		internal static float AsymRollMax = 120f;

		internal static float AsymRollU0 = 0.1f;

		internal static bool AsymRollHardOnly = true;

		internal static float EndJerkHi = 2f;

		internal static float CoupGateA0 = 12f;

		// 2.9.0 SweepHoldPitchPrio: the nose-down counterpart of OvershootPitchPriority. While the AoA-sweep cap holds the pitch-rate
		// command above the stick's push (a push-over under a rotation the yaw cannot shed in time) and the nose drops faster than
		// that command by SweepHoldErr0..+SweepHoldErrSpan deg/s, the pitch hold owns the stabilators (allocator pitch weight up,
		// the released-roll priority off, the honest stab model, the nozzles free), as the run-away case does. 15:59 log (2.8.1),
		// 46.3 / 87.9 s: push-over from 65 deg AoA at 225 kt with 72-79 deg/s of rotation; the cap asked -12 then +9 deg/s, the
		// nose went down at 31 deg/s with the right stab at its +35 peak and the left at -8 (both on the roll stop, 240-280 deg/s^2
		// of roll unmet at weight 1.3 against pitch's 7.5), AoA 40 -> 20 in 0.4 s against a nose-up command, 19.5 / 21.2 deg of
		// sideslip at 25 -> 5 deg AoA from the 50 deg/s of yaw still on board. Tom: slow the pitch, not the rotation, at high AoA.
		internal static float SweepHoldErr0 = 2f;

		internal static float SweepHoldErrSpan = 8f;

		// 2.9.2 SweepHoldStallWindow: during the capped sweep toward zero AoA, once the stabilators have pitch range back
		// (stabPitchFrac > 0), they are held inside their attached window (local AoA <= lift peak + RpMainStallMargin), the
		// 1.8.40 StallWindow. 16:37 log (2.9.1), 53.7 s: the right stab was parked at -30 (45 deg of local AoA) by the roll
		// stop through 65 -> 50 deg while pitch was TVC-only; when the hold came (pitch weight 8, rate-bounded) the allocator
		// had no nose-up gradient on a stab that deep past its stall - the honest model reads TE-up as un-stalling it (more
		// tail lift, nose-down) - so it drove it further TE-down (-25) and put all the nose-up on the nozzles (+20, the stop);
		// the nose went 37 -> 22 deg at -24 deg/s against +5..+10 commanded until the stab had been walked through (0.5 s).
		// Inside the window the stabs have gradient the moment the hold needs them; the roll stop keeps what they give there.
		internal static float CoupHeldGate = 1f;

		internal static float CoupHeldDy0 = 0.04f;

		internal static float CoupHeldDySpan = 0.08f;

		internal static float AsymPitchPrio = 1f;

		internal static float AsymPitchPrioLead = 3f;

		// Compass 0.1.5: 0 -> 0.1 (asymmetric load, AoA at the cap and the pitch short: the stabs' split for roll is priced, pitch
		// first). Bench, right outer wing gone, full aft + full left roll at 280 km/h: with the capture braking fixed (AsymCapBrake)
		// the stabs went to the roll at the 12 deg cap, the AoA ran to 39 and the jet departed; 0.1 holds 12.4.
		internal static float AsymStabAntiCost = 0.1f;

		internal static float AsymPitchCapLead = 2f;

		internal static float AsymPitchCapFloor = 10f;

		public float asymCapPitch = 999f;


		internal static float EndJerkLp0 = 1.5f;

		internal static float EndJerkLp1 = 2.5f;

		public float lpDbg;

		internal static float AIRollDirectAccel = 3000f;

		public bool aiGunTrack;

		public float aiRollFade = 1f;

		private float rollSlew = 100f;

		public static bool DebugYaw;

		public float dbgWVe;

		public float dbgYawRud;

		public float dbgYawWing;

		public float dbgYawStab;

		public float dbgYawTvc;

		public float dbgTargetZ;

		public float dbgMfZ;

		public float dbgTgtY;

		public float dbgMfY;

		public float aRoll = 300f;

		public float aYaw = 60f;

		private Lpf aRollF;

		private Lpf aYawF;

		private Lpf aRollWF;

		public float aRollWing = 200f;

		public float pS;

		private RateTraj pTraj;

		// Ifrit 0.2.0: MPO separate-axes yaw-rate trajectory and the state of the last frame (for the hand-over)
		private RateTraj mpoRTraj;

		private RateTraj mpoPTraj;

		private float mpoCoord;

		private bool mpoSepPrev;

		public bool mpoSepDbg;

		private RateTraj qTraj;

		private Lpf rSF;

		public float rudderAuth = 1f;

		// 1.8.35 sideslip rudder: signed yaw-command travel allowed beyond the normal rudder fade (deg), and the pedal
		// sideslip command it corrects toward (from the previous lateral-law pass)
		public float rudSideDbg;

		private float betaPedalCmd;

		private float rudSideF;

		// 1.8.36 ground steering
		public bool gsActive;

		public float gsNwsDeg;

		public float gsRCmd;

		public float gsRud;

		public bool gsHold;

		public float gsLineY;

		public float gsChiRef;

		private Vector3 gsLineP;

		// the captured line's point in the game's global coordinates: NO shifts its Unity world origin as the jet travels, so a
		// Unity world position stored at the capture jumps (27 Oct F-22E log: 242 m and 58 m in one tick)
		private GlobalPosition gsLinePG;

		// GroundStockMatch: the jet's own ControlsFilter (stock fly-by-wire parameters), the stock yaw sent to the gear and
		// rudders, and the sideslip rate for the drift limit
		public ControlsFilter stockCf;

		private float gsStockYaw;

		public bool gsStockPhase;

		private float gsBetaPrev = float.NaN;

		private float gsBetaDot;

		private float gsInt;

		// the raw pedal (inp.yaw), as LandingGear reads it: no stick smoothing or rate limit
		public float gsPedRaw;

		private float gsRudInt;

		private float gsBrakeT;

		private float gsLead = -1f;

		public float gsChiAir;

		private float gsSinceContact = 99f;

		public float gsAirT;

		public float aInt;

		// 2.0.14: AoA limit capture. The capture bound used to stop at zero once the AoA was past the limit (the
		// flight-path rate only: nothing brought it back) and protection only began at limit + 0.5 with zero onset, so
		// the jet sat 0.3-0.6 deg over the limit in rolls. The bound now continues through the limit (CapKp per deg,
		// nose-down past it) and an integrator (CapKi) trims the measured residual while the pilot asks for more than
		// the limit. Protection is never less nose-down than this bound, so there is no step at limit + 0.5.
		public float capIUp;

		public float capIDn;

		public float capBoundU;

		public float capBoundD;

		// nose-down (U) / nose-up (D) AoA rate the capture asks for past the flight-path rate, for protection's onset
		public float capPushU;

		public float capPushD;

		internal static float CapKi = 3f;

		internal static float CapIMax = 4f;

		internal static float CapIAlphaDot = 3f;

		internal static float CapIBand = 0.6f;

		internal static float CapIHoldT = 0.3f;

		public float capHoldTU;

		public float capHoldTD;

		public float yawSat;

		private Lpf yawSatF;

		private Lpf guardF;

		public float stabPitchFrac = 1f;

		public float tvcAnchor;

		public float tvcBudgetGate;

		private float tvcBudgetLat;

		private bool tvcAnchorInit;

		private float pCmdPrev;

		private float rCmdPrev;

		private Lpf pDotF;

		private Lpf rDotF;

		public float pFull = 300f;

		public float pStockRef;

		public float pMaxDbg;

		// Ifrit 0.3.0: roll acceleration available from the differential TVC (deg/s^2), and the body-roll budget the sideslip
		// correction may use (deg/s)
		public float aRollTvc;

		private Lpf aRollTvcF;

		public float pSlipMax;

		public float slipGainDbg;

		public float rMaxDbg;

		public float rollCtrlHalf = 100000f;

		public float guardDbg = 1f;

		public float pSmaxDbg;

		public float pSTgtDbg;

		public float latAccDbg;

		public float latJerkDbg;

		public float rSDbg;

		public bool revFlow;

		private bool revFlowLogged;

		public float sp;

		public float sr;

		public float sy;

		private float spF;

		private float srF;

		private float syF;

		public float qB;

		public float qRaw;

		public float qCmd;

		public float qT;

		public float qTprev;

		public float betaG;

		public float alphaCmd;

		public float nCmd;

		public bool unload;

		public bool prot;

		public float qMaxDbg;

		public float qMinDbg;

		public float gUnderDbg;

		public float pCmd;

		public float rCmd;

		public float pDotFF;

		public float rDotFF;

		public Vector3 omegaDotDes;

		public Vector3 mTarget;

		// 2.0.8: DisableTVC (debug) locks the nozzles at 0 whatever the FCS computes
		private float tvcPitchRaw;

		public readonly List<SplitEffector> splits = new List<SplitEffector>();

		// Ifrit 0.2.2: the splits are still coming in after an airbrake release
		private bool splitRetract;

		// Ifrit 0.2.2: the split effectors' coordinates this frame (true = retraction from full split, brake out); the surface
		// hook feeds the job the matching throttle so the two never disagree
		public bool splitBrakeMode;

		public TvcDiffEffector tvcd;

		// Revoker 0.1.0: the nozzle's yaw axis
		public TvcYawEffector tvcy;

		// Revoker 0.3.0 (F-22E 2.6.6 reference governor, ported): the yaw axis the lateral governor counts on - the surfaces whose
		// job is yaw (the rudder) and the yaw nozzle, each with its own yaw table, stops and slew. The F-22E's governor reads
		// "the rudders" everywhere (the tables, the crossing time, the load); here that set is these effectors. The rudder's
		// share fades with the fin (rudderAuth; 75-85 deg on this jet); the nozzle's yaw is thrust and holds at any AoA and
		// IAS (speedFade above the vectoring speed, off under 3 m). The allocator's aYaw is not used as the capacity: it sums
		// every effector's yaw table (canards, elevons, splits included) and read 1.5-2.2x what the rudder and nozzle at their
		// stops delivered in the 30-31 Oct logs (calibrated plant: 58-91 deg/s^2 from the two, against aYaw 101-168; the plant's
		// total yaw acceleration matched the logs to 2-9 deg/s^2 RMS, so the two are the yaw there).
		private struct YawEff
		{
			public Effector e;
			public float lo;
			public float hi;
			public float auth;
			public bool rudder;
		}

		private readonly List<YawEff> yawEffs = new List<YawEff>(4);

		public float yawCapDbg = 60f;

		// Revoker 0.4.0: 1 = the probe's directional stiffness x margin / 2 is credited to the yaw capacity (see yawCapDbg); 0 = kinematic (F-22E)
		internal static float PaceStiffCredit = 1f;

		private float stiffCredit;

		// Revoker 0.4.0: the airframe's yaw damping from the probe (SlipMomentAt with a yaw rate) credited to the yaw a reversal has to shed:
		// a rotation r is braked by |Nr| r on its own, half of that on average over the swing to zero. The rate cap and the shed
		// cap (the two that brake the yaw) take it; the pull and sweep caps (a growing demand, which the damping opposes) do not.
		// 1 = credited; 0 = the F-22E's effectors-only model. SlipDampProbeRate: the yaw rate the probe perturbs by (rad/s).
		internal static float PaceDampCredit = 1f;

		internal static float SlipDampProbeRate = 0.35f;

		private float slipDampF;

		public float slipDampDbg;

		private float dampCredit;
		
		// Revoker 0.4.0: the rate cap from a forward model of the reversal instead of the kinematic yaw-swing formula. Each tick, for a
		// candidate velocity-vector roll rate pS, the governor reverses the jet in its head: the roll trajectory swings the demand
		// from +pS to -pS at its own reversal acceleration; the rate loops (KRoll / KYaw, the sideslip hold at the loop's own
		// gain) ask the effectors for roll and yaw; a small least-squares allocation over EVERY allocator effector (each from where
		// it is, at its own slew, within its allocator window, its tables at the present flow, pitch held) delivers what it can;
		// the body rates integrate with the airframe's sideslip stiffness in yaw and roll and its yaw damping (all three from the
		// probe's tables); the sideslip follows the kinematics beta' = p sin a - r cos a. The largest pS whose reversal keeps the
		// peak sideslip inside the margin is the cap. No effector set to pick by hand (the ailerons' yaw at 65 deg AoA counts as
		// the rudder's at 10), and where the roll axis is as slow as the yaw the model says so instead of charging the yaw's lag.
		// false = the kinematic formula (F-22E 2.5.8).
		// Revoker 0.4.0: 1 = the allocator weighs the lateral residual in the velocity frame (Allocator.slipFrame); 0 = body frame (2.6.6)
		internal static bool SlipFrameAlloc = true;

		// Revoker 0.8.0: off - the steady-rate cap is the F-22E 3.0.0 reversal model, as on the Raptor (Tom, 5 Nov 01:21: "I thought the gov is
		// airframe agnostic outside of the model"). RevSim, fed the estimator, sat at its 300 deg/s ceiling (01:23 log 54.1 s: 200 deg/s reversal
		// with the pull 8 -> 27 deg AoA, RevSim predicted 1.3-5 deg of sideslip, the jet made 14.5). Bench with the 3.0.0 cap at the jet's
		// RollPaceRateBlend 0.4: every regression case 0.3-3 deg less, sweep reversal worst 8.5 -> 6.2, stop 5.4 -> 3.9, corkscrews -0.7..-3.3
		// (one +0.6), damage suite better or equal; the price is rate: sweep mean roll 113 -> 106, corkscrew velocity roll 10-50 deg/s lower,
		// a clean leaf stop 1.73 -> 1.98 s. Replay of the 54.1 s reversal: 11.1 -> 6.7 deg.
		internal static bool RevSim = false;
		
		// Revoker 0.4.1: the yaw-reach window's reach is computed over EVERY allocator effector, not the hand-picked yaw set
		// (CollectYawEffs). The window holds the commanded roll to what the measured yaw can follow within PaceHorizon, so
		// its reach is a statement about the airframe's yaw - and the hand-picked set leaves out the trailing edge, which is
		// the yaw axis at moderate-to-high AoA on this jet (per-surface on the calibrated plant at 150 km/h / 65 deg AoA the
		// aileron pair makes +-12.5 deg/s^2 of yaw against the tails' +-2.2). With the old set the reach read 5-6 deg/s in
		// flight at 15-45 deg AoA and the window held the roll 12-14 deg/s under the stick while the sideslip sat at 2-5 deg
		// of an 8 deg margin (1 Nov Ifrit 0.7.0 log). Each effector from where it is, at its own slew, inside its allocator
		// window, its tables at the present flow, the same PaceRudderUse derate and one-side cap as before. The crossing
		// time, the probe's authority and the yaw slew lead keep the dedicated set. false = the pre-Revoker 0.4.1 reach.
		internal static bool PaceReachAll = true;
		
		public float paceReachDbg;
		
		// yaw moment (deg/s^2) every allocator effector can still add toward each side from where it is within the horizon,
		// in the same two forms the window needs: the steady one-side yaw (nAv) and the horizon-integrated reach (g)
		private void ReachAll(out float gPosA, out float gNegA, out float nAvPosA, out float nAvNegA)
		{
			gPosA = 0f;
			gNegA = 0f;
			nAvPosA = 0f;
			nAvNegA = 0f;
			foreach (Effector eR in effs)
			{
				if (eR == null || !eR.active || eR.M == null || eR.N < 2 || eR.rate <= 0f || eR.hi - eR.lo < 0.5f)
				{
					continue;
				}
				float nowR = Mathf.Clamp(eR.now, eR.lo, eR.hi);
				float zNowR = AccYaw(eR.MAt(nowR));
				float zHiR = AccYaw(eR.MAt(eR.hi));
				float zLoR = AccYaw(eR.MAt(eR.lo));
				float nSideR = 0.5f * Mathf.Abs(zHiR - zLoR);
				nAvPosA += Mathf.Min(nSideR, Mathf.Max(0f, Mathf.Max(zHiR, zLoR) - zNowR));
				nAvNegA += Mathf.Min(nSideR, Mathf.Max(0f, zNowR - Mathf.Min(zHiR, zLoR)));
				for (int dR = -1; dR <= 1; dR += 2)
				{
					float tgtR = ((zHiR >= zLoR) == (dR > 0)) ? eR.hi : eR.lo;
					float ddR = Mathf.Abs(tgtR - nowR);
					float tsR = Mathf.Min(PaceHorizon, ddR / Mathf.Max(1f, eR.rate));
					float dhR = nowR + Mathf.Sign(tgtR - nowR) * Mathf.Max(1f, eR.rate) * tsR;
					float aSR = Mathf.Max(0f, (float)dR * (AccYaw(eR.MAt(dhR)) - zNowR));
					float gR = 0.5f * aSR * tsR + aSR * (PaceHorizon - tsR);
					if (dR > 0)
					{
						gPosA += gR;
					}
					else
					{
						gNegA += gR;
					}
				}
			}
		}
		
		internal static float RevSimDt = 0.02f;
		
		internal static float RevSimTmax = 3f;
		
		internal static float RevSimMax = 300f;
		
		public float revSimBetaDbg;
		
		public float revSimTDbg;
		
		public float revSimAtDbg;
		
		private struct RevEff
		{
			public float pHi;
			public float qHi;
			public float rHi;
			public float pLo;
			public float qLo;
			public float rLo;
			public float tHi;
			public float tLo;
		}
		
		private readonly List<RevEff> revEffs = new List<RevEff>(16);
		
		private float[] revX = new float[16];
		
		// the probe's signed sideslip slopes (deg/s^2 per deg of sideslip, in the tables' own sign): yaw and roll
		public float slipYawSlopeDbg;
		
		public float slipRollSlopeDbg;
		
		private float slipYawSlopeF;
		
		private float slipRollSlopeF;
		
		internal static bool RevSimDump = false;

		private void CollectRevEffs()
		{
			revEffs.Clear();
			foreach (Effector e in effs)
			{
				if (e == null || !e.active || e.M == null || e.N < 2 || e.rate <= 0f || e.hi - e.lo < 0.5f)
				{
					continue;
				}
				float now = Mathf.Clamp(e.now, e.lo, e.hi);
				Vector3 m0 = e.MAt(now);
				Vector3 mH = e.MAt(e.hi) - m0;
				Vector3 mL = e.MAt(e.lo) - m0;
				RevEff x = default(RevEff);
				x.pHi = AccRoll(mH);
				x.qHi = IaInv[1, 1] * mH.y * 57.29578f;
				x.rHi = AccYaw(mH);
				x.pLo = AccRoll(mL);
				x.qLo = IaInv[1, 1] * mL.y * 57.29578f;
				x.rLo = AccYaw(mL);
				x.tHi = Mathf.Max(0.01f, (e.hi - now) / Mathf.Max(1f, e.rate));
				x.tLo = Mathf.Max(0.01f, (now - e.lo) / Mathf.Max(1f, e.rate));
				revEffs.Add(x);
				if (RevSimDump)
				{
					System.Console.WriteLine($"   reveff {e.name} now {now:F1} [{e.lo:F1},{e.hi:F1}] rate {e.rate:F0} hi p{x.pHi:F1} q{x.qHi:F1} r{x.rHi:F1} t{x.tHi:F2} | lo p{x.pLo:F1} q{x.qLo:F1} r{x.rLo:F1} t{x.tLo:F2}");
				}
			}
			if (revX.Length < revEffs.Count)
			{
				revX = new float[revEffs.Count + 8];
			}
		}
		
		// peak |sideslip| (deg) of a full reversal from velocity-vector roll pS at the trajectory's reversal acceleration aT
		private float RevSimBeta(float pS, float aT, float jT, float c, float sn, float kY, float kL, float nR, float g, out float tRev)
		{
			int n = revEffs.Count;
			float dt = Mathf.Max(0.005f, RevSimDt);
			float p = pS * c;
			float r = pS * sn;
			float r0 = r;
			float beta = 0f;
			float wd = pS;
			float aRamp = 0f;
			float peak = 0f;
			tRev = RevSimTmax;
			for (int i = 0; i < n; i++)
			{
				revX[i] = 0f;
			}
			float kRoll = Plugin.KRoll;
			float kYaw = Plugin.KYaw;
			for (float t = 0f; t < RevSimTmax; t += dt)
			{
				bool ramp = wd > 0f - pS;
				// Revoker 0.4.1: the trajectory ramps its acceleration at the roll jerk limit and never reaches aT on a short swing -
				// stepping straight to aT made the modelled reversal 3-4x more violent than the one the law actually flies
				// (bench, Ifrit at 150 km/h / 35 deg AoA: model 127 deg/s^2 against a realized 27-35), and the cap came out
				// 14-29 deg/s where the real reversal at 42 made 1.9 deg of sideslip against an 8 deg margin.
				aRamp = Mathf.Min(aT, aRamp + jT * dt);
				wd = Mathf.Max(0f - pS, wd - aRamp * dt);
				// the law: rate commands (the sideslip hold rotates about the velocity normal: -g beta sin a of roll, +g beta cos a of yaw)
				float pd = wd * c - g * beta * sn;
				float rd = wd * sn + g * beta * c;
				float ap = kRoll * (pd - p) + (ramp ? (0f - aRamp * c) : 0f);
				float ar = kYaw * (rd - r) + (ramp ? (0f - aRamp * sn) : 0f);
				// allocation: least squares over roll, pitch (held at zero) and yaw, each effector within [-1, 1] of its two moves,
				// three Gauss-Seidel sweeps, then its slew
				float sp = 0f;
				float sq = 0f;
				float sr = 0f;
				for (int i = 0; i < n; i++)
				{
					RevEff e = revEffs[i];
					float xi = revX[i];
					if (xi >= 0f)
					{
						sp += xi * e.pHi;
						sq += xi * e.qHi;
						sr += xi * e.rHi;
					}
					else
					{
						sp -= xi * e.pLo;
						sq -= xi * e.qLo;
						sr -= xi * e.rLo;
					}
				}
				for (int sweep = 0; sweep < 3; sweep++)
				{
					for (int i = 0; i < n; i++)
					{
						RevEff e = revEffs[i];
						float xi = revX[i];
						// residual without this effector
						float ep;
						float eq;
						float er;
						if (xi >= 0f)
						{
							ep = ap - (sp - xi * e.pHi);
							eq = 0f - (sq - xi * e.qHi);
							er = ar - (sr - xi * e.rHi);
						}
						else
						{
							ep = ap - (sp + xi * e.pLo);
							eq = 0f - (sq + xi * e.qLo);
							er = ar - (sr + xi * e.rLo);
						}
						float nH = e.pHi * e.pHi + e.qHi * e.qHi + e.rHi * e.rHi;
						float nL = e.pLo * e.pLo + e.qLo * e.qLo + e.rLo * e.rLo;
						float xH = (nH > 1E-06f) ? Mathf.Clamp01((ep * e.pHi + eq * e.qHi + er * e.rHi) / nH) : 0f;
						float xL = (nL > 1E-06f) ? Mathf.Clamp01((ep * e.pLo + eq * e.qLo + er * e.rLo) / nL) : 0f;
						float cH = (ep - xH * e.pHi) * (ep - xH * e.pHi) + (eq - xH * e.qHi) * (eq - xH * e.qHi) + (er - xH * e.rHi) * (er - xH * e.rHi);
						float cL = (ep - xL * e.pLo) * (ep - xL * e.pLo) + (eq - xL * e.qLo) * (eq - xL * e.qLo) + (er - xL * e.rLo) * (er - xL * e.rLo);
						float xNew = (cH <= cL) ? xH : (0f - xL);
						// slew: from where it is, at its own rate
						float lim = (xNew > xi) ? (dt / ((xi >= 0f) ? e.tHi : e.tLo)) : (dt / ((xi > 0f) ? e.tHi : e.tLo));
						xNew = Mathf.Clamp(xNew, xi - lim, xi + lim);
						if (xNew >= 0f)
						{
							sp += xNew * e.pHi - ((xi >= 0f) ? (xi * e.pHi) : (0f - xi * e.pLo));
							sq += xNew * e.qHi - ((xi >= 0f) ? (xi * e.qHi) : (0f - xi * e.qLo));
							sr += xNew * e.rHi - ((xi >= 0f) ? (xi * e.rHi) : (0f - xi * e.rLo));
						}
						else
						{
							sp += (0f - xNew * e.pLo) - ((xi >= 0f) ? (xi * e.pHi) : (0f - xi * e.pLo));
							sq += (0f - xNew * e.qLo) - ((xi >= 0f) ? (xi * e.qHi) : (0f - xi * e.qLo));
							sr += (0f - xNew * e.rLo) - ((xi >= 0f) ? (xi * e.rHi) : (0f - xi * e.rLo));
						}
						revX[i] = xNew;
					}
				}
				// the airframe: effectors + sideslip stiffness (yaw and roll) + yaw damping on the yaw-rate change
				float accP = sp + kL * beta;
				float accR = sr + kY * beta + nR * (r - r0);
				p += accP * dt;
				r += accR * dt;
				beta += (p * sn - r * c) * dt;
				if (RevSimDump && pS == RevSimMax && ((int)(t / dt + 0.5f)) % 10 == 0)
				{
					System.Console.WriteLine($"   revsim t{t:F2} wd{wd:F0} p{p:F1} r{r:F1} beta{beta:F2} ap{ap:F0} ar{ar:F0} sp{sp:F0} sq{sq:F0} sr{sr:F0} x[{string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Take(revX, n), v => v.ToString("F2")))}]");
				}
				float ab = Mathf.Abs(beta);
				if (ab > peak)
				{
					peak = ab;
				}
				if (peak > 60f)
				{
					break;
				}
				if (p * c + r * sn <= -0.9f * pS)
				{
					tRev = t;
					break;
				}
			}
			return peak;
		}
		
		// the largest velocity-vector roll rate whose full reversal stays inside the sideslip margin (bisection over the forward model)
		private float RevSimCap(float margin, float c, float sn, float aPace)
		{
			CollectRevEffs();
			if (revEffs.Count == 0)
			{
				return PaceRateFloor;
			}
			float cA = Mathf.Max(Mathf.Abs(c), 0.05f);
			// the trajectory's reversal acceleration, as the roll law sets it (RevFloorK x roll authority, in velocity-roll terms)
			float aT = Mathf.Max(20f, ((ReversalFloor && RevConst) ? (RevFloorK * aRoll) : (Plugin.RollBrakeUse * (BrakeUseAll ? Mathf.Min(aRollPos, aRollNeg) : aRollWing))) / cA);
			// Revoker 0.6.0 (as Ifrit 0.7.2): the reversal the governor flies is paced by the yaw the effectors can add (the estimator's,
			// PaceYawUse of it over sin AoA); simulating an unpaced full-stick reversal and capping on that double-counts the protection
			if (aPace > 0f)
			{
				aT = Mathf.Min(aT, aPace);
			}
			revSimAtDbg = aT;
			// Revoker 0.4.1: the trajectory's own roll-jerk limit (the a2 of the lateral law), in velocity-roll terms
			float jT = Mathf.Max(1f, Mathf.Min(aT / Mathf.Max(0.01f, RollJerkTau), RollJerkServo * aRoll * rollSlew / cA));
			float kY = slipYawSlopeF;
			float kL = slipRollSlopeF;
			float nR = (PaceDampCredit > 0f) ? Mathf.Min(0f, slipDampF) : 0f;
			float g = Mathf.Max(0f, slipGainDbg);
			float lo = 0f;
			float hi = RevSimMax;
			float bHi = RevSimBeta(hi, aT, jT, c, sn, kY, kL, nR, g, out float tH);
			if (bHi * Mathf.Max(0.1f, PaceRateK) <= margin)
			{
				revSimBetaDbg = bHi;
				revSimTDbg = tH;
				return hi;
			}
			float bLo = 0f;
			float tLo = 0f;
			for (int it = 0; it < 8; it++)
			{
				float mid = 0.5f * (lo + hi);
				float b = RevSimBeta(mid, aT, jT, c, sn, kY, kL, nR, g, out float tM);
				if (b * Mathf.Max(0.1f, PaceRateK) <= margin)
				{
					lo = mid;
					bLo = b;
					tLo = tM;
				}
				else
				{
					hi = mid;
				}
			}
			revSimBetaDbg = bLo;
			revSimTDbg = tLo;
			return Mathf.Max(PaceRateFloor, lo);
		}

		// Revoker 0.4.0: the sweep cap fades in with the forecast sideslip over the margin (true); false = F-22E 2.6.6 (at every sideslip)
		internal static bool SweepMarginGate = true;

		private void CollectYawEffs()
		{
			yawEffs.Clear();
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Rudder && se.active && se.M != null && se.N >= 2 && se.rate > 0f)
				{
					yawEffs.Add(new YawEff { e = se, lo = se.min, hi = se.max, auth = Mathf.Max(0.05f, rudderAuth), rudder = true });
				}
			}
			if (tvcy != null && tvcy.active && tvcy.M != null && tvcy.N >= 2 && tvcy.hi - tvcy.lo > 0.5f && tvcy.rate > 0f)
			{
				yawEffs.Add(new YawEff { e = tvcy, lo = tvcy.lo, hi = tvcy.hi, auth = 1f, rudder = false });
			}
			// Su-30 0.1.0 (CantedTvc): the canted differential is the jet's yaw nozzle (its side forces add; its roll comes with it)
			if (canted != null && tvcd != null && tvcd.active && tvcd.M != null && tvcd.N >= 2 && tvcd.hi - tvcd.lo > 0.5f && tvcd.rate > 0f)
			{
				yawEffs.Add(new YawEff { e = tvcd, lo = tvcd.lo, hi = tvcd.hi, auth = 1f, rudder = false });
			}
		}

		// the yaw moment (aero frame z) a yaw effector can reach each way within its usable travel
		private static void YawRange(YawEff y, out float zHi, out float zLo)
		{
			zHi = float.MinValue;
			zLo = float.MaxValue;
			for (int i = 0; i < y.e.N; i++)
			{
				float g = y.e.grid[i];
				if (g < y.lo - 0.001f || g > y.hi + 0.001f)
				{
					continue;
				}
				zHi = Mathf.Max(zHi, y.e.M[i].z);
				zLo = Mathf.Min(zLo, y.e.M[i].z);
			}
			float zA = y.e.MAt(y.lo).z;
			float zB = y.e.MAt(y.hi).z;
			zHi = Mathf.Max(zHi, Mathf.Max(zA, zB));
			zLo = Mathf.Min(zLo, Mathf.Min(zA, zB));
		}

		// Revoker 0.2.0: the inboard trailing-edge flaps as flaperons
		public readonly List<HingeFlapEffector> hinges = new List<HingeFlapEffector>();

		internal static float HingeSymCostK = 1f;

		// Compass 0.1.10 (FlapPitchLock, Tom 5 Nov): the inner flaps (flap_L/R) take no part in pitch. Their common deflection is
		// held at its schedule (the high-lift droop, the airbrake set, the ground-brake set) and only their differential is free,
		// for roll and drag yaw. The allocator prices the common part FlapPitchLockCostK x harder, so it plans the pitch around
		// them (stabs), the command is then projected onto the schedule exactly, and their pitch range leaves aUp / aDn.
		// false = 0.1.9 (the pair's common deflection a pitch effector at ElevonSymCost).
		internal static bool FlapPitchLock = true;

		internal static float FlapPitchLockCostK = 200f;

		internal static bool FlapPitchLockAuth = false;

		// with a stabilator shot off the lock lets go: the flaps' common part is then part of the jet's last nose-down (bench, left
		// outer wing + right stab, 1 g -> full aft -> roll reversal at 470 km/h: locked, AoA 43 deg and departs; free, 7.5 deg)
		internal static bool FlapPitchLockStabRelease = true;

		private bool flapLockOn = true;

		private float HingeSymK => flapLockOn ? Mathf.Max(1f, FlapPitchLockCostK) : 1f;

		private void FlapLockStep()
		{
			bool stabLost = false;
			if (FlapPitchLockStabRelease)
			{
				foreach (SurfaceEffector sL in surfaces)
				{
					if (sL.kind == SurfaceKind.Stabilator && !sL.active)
					{
						stabLost = true;
						break;
					}
				}
			}
			flapLockOn = FlapPitchLock && !stabLost;
			foreach (HingeFlapEffector hF in hinges)
			{
				hF.pitchFlat = flapLockOn && hF.mate != null && hF.mate.active && hF.active;
			}
		}

		// the droop gives way to the differential: with the common part held, a drooped pair runs out of differential on the
		// down-going flap's stop (droop 12, range 20: 8 deg left), which 0.1.9 found by raising the common part. Here the droop
		// comes off (as FlapRollBias does for the stick) far enough that the differential the allocator asks for has
		// FlapYieldMargin deg to spare: out at FlapRollRetractRate, back at FlapRollRestoreRate. Lateral demand only.
		public float flapDiffYield;

		internal static float FlapYieldMargin = 3f;

		private static float FlapRoom(Effector a, Effector b, float c, float sgn)
		{
			return (sgn >= 0f) ? Mathf.Min(a.max - c, c - b.min) : Mathf.Min(c - a.min, b.max - c);
		}

		// projects each inner-flap pair's command onto its scheduled common deflection, keeping as much of the differential as fits
		private void FlapPitchLockProject(float dt)
		{
			if (!flapLockOn)
			{
				flapDiffYield = 0f;
				return;
			}
			float yTgt = 0f;
			foreach (HingeFlapEffector hP in hinges)
			{
				Effector m = hP.partner;
				if (m == null || !hP.active || !m.active)
				{
					continue;
				}
				float c = hP.symTarget;
				float hd = 0.5f * (hP.cmd - m.cmd);
				// the droop part of c at no yield (c0) and the yield that gives |hd| + margin of room
				float keep = 1f - Mathf.Max(flapRollBias, flapDiffYield);
				float cD = (hP.teUp != 0f) ? ((0f - hP.teUp) * Plugin.FlapHighLiftDroop * flaps.hlPos) : 0f;
				float cRest = c - cD * keep;
				// the margin grows in with the differential, so a pair with none keeps its full droop (on its stop)
				float need = Mathf.Abs(hd) + FlapYieldMargin * Mathf.Clamp01(Mathf.Abs(hd) / Mathf.Max(0.1f, FlapYieldMargin));
				if (Mathf.Abs(cD) > 0.5f)
				{
					float y = 1f;
					for (int k = 0; k <= 20; k++)
					{
						float yk = k * 0.05f;
						if (FlapRoom(hP, m, cRest + cD * (1f - yk), hd) >= need)
						{
							y = yk;
							break;
						}
					}
					yTgt = Mathf.Max(yTgt, y);
				}
				float hdLo = Mathf.Max(hP.min - c, c - m.max);
				float hdHi = Mathf.Min(hP.max - c, c - m.min);
				hd = (hdLo <= hdHi) ? Mathf.Clamp(hd, hdLo, hdHi) : 0f;
				hP.cmd = Mathf.Clamp(c + hd, hP.min, hP.max);
				m.cmd = Mathf.Clamp(c - hd, m.min, m.max);
			}
			flapDiffYield = Mathf.MoveTowards(flapDiffYield, yTgt, ((yTgt > flapDiffYield) ? FlapRollRetractRate : FlapRollRestoreRate) * dt);
		}

		internal static int HingeRollGroup = 3;

		public float tvcYawInput;

		public float tvcRollInput;

		public float tvcPitchInput
		{
			get
			{
				return Plugin.DisableTVC ? 0f : tvcPitchRaw;
			}
			set
			{
				tvcPitchRaw = value;
			}
		}

		private int groundFrames;

		public bool rotating;

		internal static float RotLeadT = 0.1f;

		internal static float RotLeadMax = 3f;

		public bool mainsOnGround;

		// 2.0.6 ground rate law state
		public bool groundRateOn;

		public bool noseContactLaw;

		private float gNeutralT;

		private float gContactAir;

		private float rotMomLim;

		private bool rotMomOn;

		public float groundBrake;

		public float airBrake;

		// 2.2.16 (AirbrakeAoAFade): the in-flight brake's share of the drag and its opening target
		private readonly List<Airbrake> airbrakes = new List<Airbrake>();

		private static readonly System.Reflection.FieldInfo fABDrag = HarmonyLib.AccessTools.Field(typeof(Airbrake), "dragAmount");

		private static readonly System.Reflection.FieldInfo fABPart = HarmonyLib.AccessTools.Field(typeof(Airbrake), "part");

		private readonly List<Rigidbody> abRb = new List<Rigidbody>();

		private readonly List<float> abDrag = new List<float>();

		private readonly List<UnitPart> abPart = new List<UnitPart>();

		public float airbrakeShare = 1f;

		public float airbrakeFade = 1f;

		public float airbrakeOpenNow;

		// the opening the brake is driven to: 0 when not commanded, else 1 or less with AirbrakeAoAFade
		public float AirbrakeOpenTarget => AirbrakeOut ? (Plugin.AirbrakeAoAFade ? airbrakeFade : 1f) : 0f;

		public bool onGround;

		private float[] brakeBias;

		// Revoker 0.2.8: TrailingEdgeBrakeConfig, only with ThrottleIndependentAirbrake on (off: the airbrake is fully stock)
		public bool TeBrakeOn => Plugin.TeBrakeConfig && Plugin.ThrottleIndependentAirbrake;

		public float teBrakeAir;

		public float teBrakeAirF;

		public float teBrakeLandF;

		public bool derotGate;

		public bool teBrakeMov;

		internal static float DerotationRate = 4f;

		internal static float TeBrakeAirRate = 0f;

		// Revoker 0.2.9: the outboard elevons' trailing-edge-up share of the in-flight set is solved each frame so that the set's
		// normal force, including the canards' re-trim of its pitch moment, stays at zero (TeBrakeLiftNeutral); at most TeBrakeOutMax
		internal static bool TeBrakeLiftNeutral = true;

		internal static float TeBrakeOutMax = 30f;

		public float teBrakeOutDbg;

		public float teBrakeFnDbg;

		// Revoker 0.2.9 (TeBrakeLiftNeutral): 0.2.8 fixed the outboard share at 20 deg (inboard 25 TE down) and moved the set at
		// 0.5 /s, because faster it bumped the g: the two sets' lift does not rise in proportion (bench, 900 km/h, set in 0.4 s:
		// nz 1.09 -> 1.25 at 0.2 s, 0.84 at 0.5 s, still 0.94 after 1 s with the AoA working back up at the held attitude).
		// Here, after the effectors are evaluated, the outboard common deflection is the one whose normal force cancels the
		// inboard set's plus the canards' normal force from re-trimming both sets' pitch moment (canard normal force per unit
		// of its pitch moment, from its own model at its present deflection). Scanned over 0..TeBrakeOutMax in 0.5 deg steps.
		internal static float TeBrakeSymK = 50f;

		// Revoker 0.2.10 (TrailingEdgeBrakeAoAFade): the in-flight set closes with AoA as the F-22E's speedbrake does. The F-22E
		// opens it by drag share (AirbrakeAoAFade, 2.2.16: full at 40 % of the total drag, stowed at 15 %), which on the clean
		// F-22E is full to ~15 deg AoA, half at ~25 and stowed past ~35. The same share rule does not carry over: the Revoker's
		// elevon set is a smaller share of a cleaner airframe (bench, model drag, 400 km/h: 0.41 at 6 deg, 0.23 at 9, 0.13-0.17
		// flat from 12 to 36 deg), so it would close by 12 deg and barely reopen (at most 9 % open up to 36 deg). The F-22E's resulting AoA schedule is used
		// instead: full to TeFadeAoA0, closing linearly to stowed at TeFadeAoA1 (|AoA|), at most 2 /s as the F-22E's filter.
		internal static float TeFadeAoA0 = 15f;

		internal static float TeFadeAoA1 = 35f;

		public float teFade = 1f;

		private void TeBrakeFadeStep(float dt)
		{
			float tgt = 1f - Mathf.InverseLerp(TeFadeAoA0, Mathf.Max(TeFadeAoA0 + 1f, TeFadeAoA1), Mathf.Abs(alpha));
			teFade = Mathf.MoveTowards(teFade, tgt, 2f * dt);
		}

		private void TeBrakeLiftBalance()
		{
			teBrakeOutDbg = 0f;
			teBrakeFnDbg = 0f;
			// the pairs' common deflection held to the set this much harder while it is in (1 = the normal ElevonSymCost)
			float kS = 1f + (Mathf.Max(1f, TeBrakeSymK) - 1f) * Mathf.Max(teBrakeAirF, teBrakeLandF);
			foreach (SurfaceEffector sK in surfaces)
			{
				// Ternion 0.1.0: the inboard trailing edge (flaperon_L/R) is a control surface too
				if (sK.partner != null && (sK.kind == SurfaceKind.Aileron || sK.kind == SurfaceKind.Flaperon))
				{
					sK.symCost = Plugin.ElevonSymCost * kS;
				}
			}
			foreach (HingeFlapEffector hK in hinges)
			{
				if (hK.partner != null)
				{
					hK.symCost = Plugin.ElevonSymCost * HingeSymCostK * HingeSymK * kS;
				}
			}
			if (!TeBrakeLiftNeutral || teBrakeAirF <= 0.001f || Plugin.AirBrakeElevonUp <= 0f)
			{
				return;
			}
			Transform root = ((Component)(object)ac).transform;
			SurfaceEffector ob = null;
			foreach (SurfaceEffector sB in surfaces)
			{
				if (sB.partner != null && sB.kind == SurfaceKind.Aileron && sB.active && sB.GeoTeUp(root) != 0f)
				{
					ob = sB;
					break;
				}
			}
			if (ob == null)
			{
				return;
			}
			float tuO = ob.GeoTeUp(root);
			float dF = 0f;
			float dM = 0f;
			foreach (HingeFlapEffector hB in hinges)
			{
				float tuH = hB.GeoTeUp(root);
				if (hB.active && tuH != 0f)
				{
					float dI = 0f - tuH * Plugin.AirBrakeInboardDown * teBrakeAirF;
					dF += hB.FNAt(dI) - hB.FNAt(0f);
					dM += hB.MAt(dI).y - hB.MAt(0f).y;
				}
			}
			// Ternion 0.1.0: the inboard set on the FS-3 is flaperon_L/R (control surfaces, not HLD hinges)
			foreach (SurfaceEffector sI in surfaces)
			{
				float tuI = sI.GeoTeUp(root);
				if (sI.kind == SurfaceKind.Flaperon && sI.partner != null && sI.active && tuI != 0f)
				{
					float dI = 0f - tuI * Plugin.AirBrakeInboardDown * teBrakeAirF;
					float wI = (sI.partner is SurfaceEffector sIP && sIP.active) ? 2f : 1f;
					dF += wI * (sI.FNAt(dI) - sI.FNAt(0f));
					dM += wI * (sI.MAt(dI).y - sI.MAt(0f).y);
				}
			}
			// canards: normal force per unit pitch moment (Ternion 0.1.0: the canards only - they carry the steady pitch)
			bool anyCan = surfaces.Exists((SurfaceEffector o) => o.kind == SurfaceKind.Stabilator && o.isCanard && o.active);
			float kC = 0f;
			int nC = 0;
			foreach (SurfaceEffector sC in surfaces)
			{
				if (sC.kind == SurfaceKind.Stabilator && sC.active && (sC.isCanard || !anyCan))
				{
					float a0 = Mathf.Clamp(sC.now - 1f, sC.min, sC.max - 2f);
					float dMc = sC.MAt(a0 + 2f).y - sC.MAt(a0).y;
					if (Mathf.Abs(dMc) > 1f)
					{
						kC += (sC.FNAt(a0 + 2f) - sC.FNAt(a0)) / dMc;
						nC++;
					}
				}
			}
			kC = (nC > 0) ? (kC / (float)nC) : 0f;
			int nO = (ob.partner is SurfaceEffector obP && obP.active) ? 2 : 1;
			float bestU = 0f;
			float bestE = float.MaxValue;
			float uMax = Mathf.Min(TeBrakeOutMax, ob.max, 0f - ob.min);
			for (float u = 0f; u <= uMax + 0.001f; u += 0.5f)
			{
				float dO = tuO * u;
				float fO = (float)nO * (ob.FNAt(dO) - ob.FNAt(0f));
				float mO = (float)nO * (ob.MAt(dO).y - ob.MAt(0f).y);
				float e = Mathf.Abs(dF + fO - kC * (dM + mO));
				if (e < bestE)
				{
					bestE = e;
					bestU = u;
				}
			}
			teBrakeOutDbg = bestU;
			teBrakeFnDbg = bestE;
			foreach (SurfaceEffector sB in surfaces)
			{
				float tuB = sB.GeoTeUp(root);
				if (sB.partner != null && sB.kind == SurfaceKind.Aileron && tuB != 0f)
				{
					sB.symTarget += tuB * bestU;
				}
			}
		}

		private float gbLogTimer;

		private float noseDownTime;

		private float noseUpTime;

		private float airTime;

		private LandingGear noseGearCached;

		private bool noseGearSearched;

		private FcsMode lastLoggedMode;

		private Telemetry telemetry;

		private float time;

		private static readonly float[] envAlphas = new float[25]
		{
			-1f, 0f, 1f, 5f, 10f, 15f, 20f, 25f, 30f, 35f,
			40f, 45f, 50f, 55f, 60f, 65f, -5f, -10f, -15f, -20f,
			-25f, -30f, -35f, -40f, -45f
		};

		private const int EnvNegStart = 16; // 1.9.7: negative-lift points for the negative hand-over threshold

		private readonly float[] envLift = new float[25];

		private static readonly float[] asymAlphas = BuildAsymAlphas();

		private readonly float[] asymU = new float[22];

		private readonly float[] asymAr = new float[22];

		private readonly float[] asymAy = new float[22];

		private readonly float[] asymCr = new float[22];

		private readonly float[] asymCy = new float[22];

		private readonly float[] asymNetDn = new float[22];

		// Compass 0.1.7 (AsymPitchBias): the modelled bare-airframe pitch (deg/s^2, nose-up +) at each sweep AoA
		private readonly float[] asymAirP = new float[22];

		private readonly float[] asymStabLo = new float[22];

		// 1.9.0: nose-down authority incl. the flaperons at their present deflection (asymNetDn leaves their lift out)
		private readonly float[] authNetDn = new float[22];

		private readonly float[] asymNetUp = new float[22];

		public float aBrakeTgt;

		internal static int CaptureMode = 1;

		internal static bool RollCapture = true;

		internal static float CapLagT = 0.3f;

		// share of the nose-down (nose-up at negative AoA) pitch authority that the nozzles carry
		private float CapTvcShare()
		{
			if (tvc == null || !tvc.active || tvc.M == null || tvc.N < 2 || Ia[1, 1] <= 1f)
			{
				return 0f;
			}
			float lo = float.MaxValue;
			float hi = float.MinValue;
			for (int k = 0; k < tvc.N; k++)
			{
				lo = Mathf.Min(lo, tvc.M[k].y);
				hi = Mathf.Max(hi, tvc.M[k].y);
			}
			float half = 0.5f * (hi - lo) * IaInv[1, 1] * 57.29578f;
			return Mathf.Clamp01(half / Mathf.Max(20f, (alpha >= 0f) ? aDn : aUp));
		}


		internal static float RollCaptureCoupK = 1f;

		// Su-30 0.1.15 (CaptureMeasured): how long the pitch effectors take to deliver the swing that aDn / aUp count on, from
		// where each one is now to the end of its travel that gives the most nose-down (tauDn) / nose-up (tauUp), at its own
		// slew rate. Each one ramps its share in linearly over its swing time, which costs the same stopping distance as a
		// pure delay of half that time, so the result is the moment-weighted mean of the half swing times.
		private float capTauDnDbg;
		private float pitchTrajTauDbg;
		private float gCapBF, gCapTDbg, gCapBDbg, gCapEDbg;
		private bool gCapBInit;
		// Su-30 0.2.0 (SoftAoALimit): the AoA law has handed the jet to the MPO law (forced past the limit); inside the limit since (s)
		private bool softMpo;

		// 0.3.10: off - 0.3.7 misread Tom's 00:45 report; a released stick past the limit holds (no recovery), as he asked at 01:01.
		internal static bool SoftMpoStick = false;

		internal static float SoftMpoMinTas = 5f;

		internal static bool SoftPitchHold = true;

		internal static float SoftPitchMargin = 0.5f;

		private bool softPitch;

		private float softPitchInside;

		public float softPitchDbg;

		internal static bool SoftMpoFromMpo = true;

		private bool softMpoFromMpo;

		private bool pilotMpoPrev;

		internal static float SoftMpoStickIn = 0.5f;

		internal static float SoftMpoStickOut = 0.3f;
		private float softMpoInside;
		// Su-30 0.2.0 (StopIgnoresSlipOutsideMargin): weight of the full-rate stop, and the stop's predicted sideslip (deg)
		private float stopHardW;
		private bool stopHardOn;
		private float stopHardPredDbg;
		internal static float PitchTrajEndK = 1f;
		internal static float GCapRollLeadK = 1f;
		internal static float StopHardRelease = 5f;
		// Su-30 0.2.4: the pitch wall's canard / stab peak pin runs only up to 2 deg short of the AoA limit in force (see the wall)
		internal static bool WallLiveLimit = true;
		private float capTauUpDbg;
		private float capBrkDnDbg;
		private float capBrkUpDbg;

		private float qDotMeasDeg;

		// Su-30 0.1.15: the pitch swing the effectors can still deliver from where they are, with what the jet is doing in roll
		// and yaw kept: a pair of surfaces only moves its common part, so it stops when the first of the two reaches its stop
		// (a stab held at -30 for roll gives nothing more, its mate only what keeps the difference); the nozzles' common part
		// only up to max - |differential|. Returns the nose-down / nose-up swing (deg/s^2) and each one's delivery delay.
		private void PitchReach(out float swDn, out float tauDn, out float swUp, out float tauUp)
		{
			float wDn = 0f, sDn = 0f, wUp = 0f, sUp = 0f;
			float diffAbs = (tvcd != null && tvcd.active) ? Mathf.Abs(tvcd.now) : 0f;
			foreach (Effector e in effs)
			{
				if (!e.active || e.M == null || e.N < 2 || e.rate < 0.5f || e == tvcd)
				{
					continue;
				}
				for (int dirSel = 0; dirSel < 2; dirSel++)
				{
					int iT = 0;
					for (int i = 1; i < e.N; i++)
					{
						if (dirSel == 0 ? (e.M[i].y < e.M[iT].y) : (e.M[i].y > e.M[iT].y)) iT = i;
					}
					float xT = e.min + iT * e.Step;
					if (e == tvc && diffAbs > 0f)
					{
						float lim = Mathf.Max(0f, Mathf.Max(Mathf.Abs(e.min), Mathf.Abs(e.max)) - diffAbs);
						xT = Mathf.Clamp(xT, -lim, lim);
					}
					float r = Mathf.Abs(xT - e.now);
					float dir = Mathf.Sign(xT - e.now);
					Effector pe = e.partner;
					if (pe != null && pe.active && pe.M != null && pe.N >= 2)
					{
						int jT = 0;
						for (int i = 1; i < pe.N; i++)
						{
							if (dirSel == 0 ? (pe.M[i].y < pe.M[jT].y) : (pe.M[i].y > pe.M[jT].y)) jT = i;
						}
						r = Mathf.Min(r, Mathf.Abs(pe.min + jT * pe.Step - pe.now));
					}
					float xR = e.now + dir * r;
					float dM = (e.MAt(xR).y - e.MAt(e.now).y) * IaInv[1, 1] * 57.29578f;
					float tSw = r / e.rate;
					if (dirSel == 0 && dM < 0f)
					{
						wDn += -dM;
						sDn += -dM * 0.5f * tSw;
					}
					else if (dirSel == 1 && dM > 0f)
					{
						wUp += dM;
						sUp += dM * 0.5f * tSw;
					}
				}
			}
			swDn = wDn;
			swUp = wUp;
			tauDn = ((wDn > 1E-06f) ? (sDn / wDn) : 0f) + IndiTau;
			tauUp = ((wUp > 1E-06f) ? (sUp / wUp) : 0f) + IndiTau;
		}

		internal static float PitchLeadT = 0.25f;

		internal static float StabsPitchOnlyCost = 1e6f;

		internal static float HiAoAStabRollCost = 0f;

		// Revoker 0.2.7: only a live rudder keeps the canards out of the first yaw tier
		internal static bool RudderTierLiveOnly = true;

		// Revoker 0.2.7 (FinLossTvcHold)
		internal static float FinLossTvcSpeed = 1000f;

		// Revoker 0.2.7 (CanardAttachedWindow)
		internal static float CanWinA0 = 30f;

		internal static bool CanWinLatOnly = true;

		internal static bool CanWinNegOnly = true;

		// Revoker 0.2.7: canard travel the asymmetric-load sweep counts (deg)
		internal static float AsymStabTravel = 40f;

		private float hiRollWF;

		internal static float CanWinRate = 120f;

		public float canWinDbg;

		internal static float TvcNeutralQdot = 20f;

		internal static float TvcNeutralTau = 1f;

		internal static float TvcNeutralBand = 1f;

		private float tvcNeutralW;

		// Revoker 0.1.0: yaw-TVC neutral hold state
		private float tvcYawNeutralW;

		private float srHeldY;

		private float syHeldY;

		public float tvcNeutralDbg;

		public float fightGateDbg;

		// 0.4.3: the 0.4.1 roll-fight exemption with an engine down on thrust (not only out); false (see the fight gate)
		internal static bool FightExemptAsym = false;

		// bench/debug: local AoA now of the k-th stabilator (order of the surfaces list)
		public float StabLocalAoA(int k)
		{
			int n = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Stabilator)
				{
					if (n == k)
					{
						return se.localAoANow;
					}
					n++;
				}
			}
			return 0f;
		}

		// Ifrit 0.3.0: roll-TVC neutral hold weight
		public float tvcRollNeutralW;

		internal static float HiAoAStabRollA0 = 22f;

		internal static float HiAoAStabRollSpan = 10f;

		public float pitchLeadDbg;



		private float asymTimer = 99f;

		private bool asymValid;

		public float asymCapPos = 999f;

		public float asymCapNeg = -999f;

		public float asymCapPosEff = 999f;

		public float asymCapNegEff = -999f;

		public float asymDy;

		public float asymArNow;

		public float asymAyNow;

		public float asymUNow;

		public float asymCrNow;

		public float asymCyNow;

		public Vector3 asymMNow;

		private Lpf3 asymMF;

		public Vector3 asymFF;

		public float asymFFr;

		public float asymFFy;

		public float asymFFGate;

		public float rollPitchFF;

		private float tvcPin;

		private bool tvcPinned;

		private readonly float[] pinLo = new float[16];

		private readonly float[] pinHi = new float[16];

		private readonly float[] symSave = new float[16];

		public float dMeasR;

		// Compass 0.1.7 (AsymPitchBias): nose-up the asymmetric-load pitch cap's model leaves out (deg/s^2, >= 0). The cap's
		// nose-down estimate is the swept bare airframe plus the stabs at full nose-down; it leaves out the other surfaces,
		// which on the T/A-30 carry the lost wing's roll and pitch nose-up as they do it, and the bare-airframe model reads
		// nose-down of the jet. Bench, right outer wing + left fin + left stab gone, full aft at 330 km/h: the ailerons /
		// flaps / rudder pitched +7..+24 deg/s^2, the airframe 7-10 more nose-up than modelled; the cap read 22-24 deg/s^2 of
		// nose-down margin at 12 deg where the jet had none (the stab on its lift peak), so it sat at 12.4 and the jet
		// departed from 12-13 deg (in game, 5 Nov 19:18 log, 138 s, the cap was 12 and its pitch part read 25-29). Measured:
		// (I wdot + w x Iw - the stabs' moment) / Iyy less the modelled airframe at the present AoA, 0.5 s low-pass, then held
		// at its peak and let down at AsymPitchBiasDecay. Only the positive (nose-up) side counts.
		internal static bool AsymPitchBias = true;

		// Compass 0.1.8 (AsymDamageGate): the asymmetric-load pitch protections (pitch cap and bias, pitch priority, stab roll price,
		// held coupling gate) were gated on the lateral CG offset alone (|asymDy| over 4-12 cm). asymDy is measured from the
		// surviving left/right surface pairs; with the Compass's outer wing, left stab and left fin shot off the only pair left was
		// the rudders, the left one hanging off the fuselage hinge, and asymDy swung -0.28 <-> 0.00 m every 15-20 s (5 Nov 20:25 log).
		// Each swing to 0 dropped the pitch cap 7 -> 12 deg, and both departures came in those windows (193 s, 217-223 s). Now a
		// control surface lost with the asymmetric-load cap in force opens the same gates.
		internal static bool AsymDamageGate = true;

		private float dmgGateHold;

		private float AsymGate()
		{
			float dyG = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan);
			return (AsymDamageGate && dmgGateHold > 0f) ? 1f : dyG;
		}

		private void DamageGateStep(float dt)
		{
			int nAll = 0;
			int nOn = 0;
			foreach (SurfaceEffector sG2 in surfaces)
			{
				nAll++;
				if (sG2.active)
				{
					nOn++;
				}
			}
			bool on = nOn < nAll && asymValid && asymCapPosEff < Plugin.AoAPos - 0.5f && (tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC);
			dmgGateHold = on ? 1f : Mathf.Max(0f, dmgGateHold - dt);
		}

		internal static float AsymPitchBiasTau = 0.5f;

		// with a stab gone, the bias also takes in the stabs' live nose-down end against the swept one at this AoA: the lone
		// slab's table reads 8-12 deg/s^2 less nose-down than the sweep at 5-8 deg (bench). Not with both stabs: there it
		// lowered the cap to 7 with the outer wing alone and cost 4 cases of 77 (bench matrix, 0.1.7 notes).
		internal static bool AsymPitchBiasStab = true;

		internal static float AsymPitchBiasDecay = 2f;

		// Compass 0.1.7: with no pitch TVC the asymmetric-load pitch cap may come down to 7 deg (AsymPitchCapFloor, 10, is the
		// F-22E's). Bench, right outer wing + left stab gone (with and without the left fin), full aft at 280 km/h: the slab
		// reaches its lift peak at 9-10 deg body AoA there, and a cap held at 10 departed on a roll; at 7 (~1.3 g at 280 km/h,
		// ~4 g at 470) the 20 damage cases depart once (roll spam with the fin intact) against 4-5 times at 10.
		internal static float AsymPitchCapFloorNoTvc = 7f;

		public float asymPitchBias;

		// Compass 0.1.8 (AsymPitchBiasNeg): the same at negative AoA, for a negative pitch cap (AsymPitchCapNeg). 5 Nov 20:25 log,
		// 243.5 s: full push + roll with the same damage, nose-up ran out at -7 deg (aUp at its floor, the lone stab at its
		// nose-up end) and the AoA went to -24 with full aft stick; the negative cap was the lateral one at -8.
		internal static bool AsymPitchBiasNeg = true;

		internal static float AsymPitchCapFloorNegNoTvc = 4f;

		public float asymPitchBiasNeg;

		private Lpf asymPitchBiasNegF;

		public float asymCapPitchNeg = -999f;

		private readonly float[] asymStabHi = new float[22];

		private Lpf asymPitchBiasF;

		public float dMeasY;

		private Lpf dMeasRF;

		private Lpf dTrimRF;

		public float dTrimR;

		public float aRollPos = 300f;

		public float aRollNeg = 300f;

		internal static bool RollTrimHeadroom = true;

		private Lpf dMeasYF;

		private float asymLogTimer = 99f;

		private float asymLoggedCap = 999f;

		private float asymLoggedCapN = -999f;

		private readonly HashSet<AeroPart> surfaceParts = new HashSet<AeroPart>();

		private int asymK = -1;

		private bool asymCycleDone;

		private bool asymFirstLogged;

		internal static int AsymPointsPerStep = 3;

		private Vector3 asymMT;

		// Compass 0.1.11: the roll and yaw of fixed-nozzle thrust for the asymmetric-load model, with an engine out or down on thrust
		// only. Its pitch is left out (the pitch caps' model is as before), and with both engines matched it is left out too: counted,
		// the thrust about a store-shifted CG lowered the asymmetric-store AoA cap by up to 4 deg (bench, 435 kg on wing2_R, 250 km/h:
		// 17.2 -> 13.4), a change to a feature not touched here.
		private Vector3 FixedThrustLat()
		{
			if (tvc == null || !tvc.fixedNozzles)
			{
				return Vector3.zero;
			}
			if (engOut || engAsym)
			{
				return new Vector3(tvc.Mnow.x, 0f, tvc.Mnow.z);
			}
			// faded in on the lateral CG offset, so a symmetric jet gets exactly zero (Compass 0.4.0: on always, the tiny thrust moments
			// re-ordered the allocator's equal-cost picks in every clean case)
			float gT = AsymThrustCgLoad ? Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan) : 0f;
			return (gT > 0f) ? (gT * new Vector3(tvc.Mnow.x, 0f, tvc.Mnow.z)) : Vector3.zero;
		}

		// Su-30 0.5.0 (AsymThrustCgLoad, from the Ternion 0.2.2): the thrust's roll and yaw about the real CG are part of the asymmetric
		// load with both engines running too. On this jet that was already so while the nozzles vector or are locked (asymMT); this
		// covers the one state left, nozzles not vectoring (past their cut-off airspeed), where only an engine failure counted.
		internal static bool AsymThrustCgLoad = true;

		// Su-30 0.5.0 (from the Ternion 0.2.2): the nose-down margin of the asymmetric-load pitch cap scales with dynamic pressure below
		// AsymPitchMarginIasRef (the airframe's pitch-up and the surfaces' nose-down both scale with q, so a fixed deg/s^2 margin is a
		// speed limit in disguise: the Compass's 11 Nov 13:29 log, the cap 27-31 deg at 65-70 m/s and 7 deg under 62 m/s), never under
		// AsymPitchMarginMin; the nose-down is judged at the predicted dynamic pressure (qPredK) like the lateral side.
		internal static float AsymPitchMarginIasRef = 130f;
		internal static float AsymPitchMarginMin = 6f;
		internal static bool AsymPitchAtPredQ = true;
		public float asymPitchMargDbg;

		private float AsymPitchMarginEff(out float kqP)
		{
			kqP = AsymPitchAtPredQ ? Mathf.Clamp(qPredK, 0.3f, 1f) : 1f;
			if (AsymPitchMarginIasRef <= 0f)
			{
				return Plugin.AsymPitchMargin;
			}
			float iasP = asymSweepIas * Mathf.Sqrt(kqP);
			float sc = Mathf.Clamp01(iasP * iasP / (AsymPitchMarginIasRef * AsymPitchMarginIasRef));
			return Mathf.Max(Mathf.Min(AsymPitchMarginMin, Plugin.AsymPitchMargin), Plugin.AsymPitchMargin * sc);
		}

		// bench/debug: the sweep's thrust moment as yaw accel (deg/s^2), and whether the pitch / differential TVC effectors are active
		public float AsymMTYaw => AccYaw(asymMT);

		public int TvcActiveDbg => (tvc != null && tvc.active ? 1 : 0) + (tvcd != null && tvcd.active ? 2 : 0);

		private float[] asymDsym;

		public float Lp = -100000f;

		private const float KCapture = 3f;

		internal static float YawBoostFixed = -1f;

		internal static bool RudderRevBack = true;

		internal static float YawFFAoA0 = 15f;

		internal static float YawFFAoA1 = 28f;

		internal static bool ReversalFloor = true;

		internal static float RevFloorK = 1f;

		internal static bool RevConst = true;

		internal static bool RevCarry = true;

		// Su-30 0.3.8 (F-22E 4.1.1): one acceleration through a reversal, the load's limit included (false = before)
		internal static bool RevOneAccel = true;

		// Su-30 0.3.8 (F-22E 4.1.2): a reversal the load itself brakes builds its acceleration over the rudders' travel (false = 4.1.1 shape)
		internal static bool RevLoadRamp = true;

		private float revA;

		private float revTarget;

		internal static float WashK = -1f;

		internal static float WashTau = -1f;

		internal static float WashUtilHi = 0.75f;

		internal static bool GUnderAboveOnly = true;

		private float washApplied;

		internal static float MLeadTau = 0.04f;

		internal static float IndiTau = 0.04f;

		// 1.9.0 hybrid INDI (pitch): the incremental law takes the airframe's own moment from filtered measurements, so
		// it lags the true one by the filter (IndiTau). At high dynamic pressure the non-control-surface moment moves fast
		// with AoA and pitch rate (the fuselage/LERX nose-up grows with AoA), and that lag let the pitch rate overshoot its
		// command by ~2 deg/s for 0.3 s at 1300 km/h - ~1.4 g at that speed, the G-command overshoot/undershoot cycle.
		// The lag is predicted from the airframe model: dM = dM/dAoA x (AoA - AoA filtered) + dM/dq x (q - q filtered),
		// slopes of the non-surface parts' moment evaluated every 0.1 s at the present flow.
		public float mAlphaO;

		public float mQO;

		public float hybridDM;

		private float dHatPrevY;

		private bool dHatInit;

		internal float dDotYF;

		private float distLeadGate;

		internal float distLeadAcc;

		internal static float DistLeadTau = 0.06f;

		internal static float DistLeadMax = 15f;

		internal static bool DistLeadNoDamp = true;

		internal static float DistLeadTw0 = 0.2f;

		internal static float DistLeadTw1 = 0.4f;

		internal static float NewtonMinTw = 0.25f;

		internal static bool NewtonReleasedOnly = true;

		private Lpf alphaIndiF;

		private Lpf qIndiF;

		private float mPrevY;

		private bool mLeadInit;

		private float mDotYF;

		public float mLeadGate;

		public float mLeadAgree = 1f;

		internal static bool PairedTrim = true;

		public float latReserve;

		public bool isAI;

		public float aiGLimit = 9.5f;

		public float aiCornerSpeed = 140f;

		private bool telemetryOpened;

		private float aiLogT;

		private float aiLogAMax;

		private float aiLogAMin;

		private float aiLogGMax;

		private float aiLogGMin;

		private float aiLogIasMin = 9999f;

		private float aiLogIasMax;

		private float aiLogProtT;

		private int aiLogN;

		private float aiLogP2;

		private int aiLogRev;

		private int aiLogRevSign;

		private float aiLogGunT;

		private float aiLogFadeT;

		private void AILogStep(float dt)
		{
			if (aiLogN == 0)
			{
				aiLogAMax = (aiLogAMin = alpha);
				aiLogGMax = (aiLogGMin = nz);
			}
			aiLogN++;
			aiLogT += dt;
			aiLogAMax = Mathf.Max(aiLogAMax, alpha);
			aiLogAMin = Mathf.Min(aiLogAMin, alpha);
			aiLogGMax = Mathf.Max(aiLogGMax, nz);
			aiLogGMin = Mathf.Min(aiLogGMin, nz);
			aiLogIasMin = Mathf.Min(aiLogIasMin, ias * 3.6f);
			aiLogIasMax = Mathf.Max(aiLogIasMax, ias * 3.6f);
			if (prot)
			{
				aiLogProtT += dt;
			}
			float pDeg = omega.x * 57.29578f;
			aiLogP2 += pDeg * pDeg * dt;
			int num = ((pDeg > 15f) ? 1 : ((pDeg < -15f) ? (-1) : 0));
			if (num != 0)
			{
				if (aiLogRevSign != 0 && num != aiLogRevSign)
				{
					aiLogRev++;
				}
				aiLogRevSign = num;
			}
			if (aiGunTrack)
			{
				aiLogGunT += dt;
				if (aiRollFade < 0.99f)
				{
					aiLogFadeT += dt;
				}
			}
			if (aiLogT >= 20f)
			{
				Plugin.Log.LogInfo($"Su-30 FCS AI jet #{((UnityEngine.Object)(object)ac).GetInstanceID()} last {aiLogT:0}s: mode {Mode}, IAS {aiLogIasMin:0}-{aiLogIasMax:0} km/h, AoA {aiLogAMin:0.0}..{aiLogAMax:0.0} deg, g {aiLogGMin:0.0}..{aiLogGMax:0.0}, AoA protection active {aiLogProtT:0.0}s, roll rate rms {Mathf.Sqrt(aiLogP2 / Mathf.Max(aiLogT, 0.01f)):0} deg/s with {aiLogRev} left/right reversals (>15 deg/s), gun-tracking a moving target {aiLogGunT:0.0}s (roll faded {aiLogFadeT:0.0}s).");
				aiLogT = 0f;
				aiLogN = 0;
				aiLogProtT = 0f;
				aiLogP2 = 0f;
				aiLogRev = 0;
				aiLogGunT = 0f;
				aiLogFadeT = 0f;
				aiLogIasMin = 9999f;
				aiLogIasMax = 0f;
			}
		}

		public int stabBranch;

		public float pitchNeed;

		// 2.9.0: the two halves of pitchNeed - the nose-up run-away (1.8.23) and the capped nose-down sweep running ahead of its cap
		public float pitchNeedUp;

		public float pitchNeedDn;

		public bool sweepHoldWinDbg;

		// Revoker 0.2.5: 20 (F-22E / Ifrit: 5). Without TVC the modelled net nose-down at the 45 deg cap is under 10 deg/s^2,
		// so the floor set the velocity-vector roll at 25 deg/s at every IAS (in-game 28 Oct 19:56, DisableTVC). Bench, TVC
		// off, loaded rolls at the cap 220-600 km/h: 23-24 -> 32-46 deg/s mean, AoA overshoot 0.3 deg, sideslip 4.5-7.3 deg,
		// push recovery unchanged (1.56 -> 1.66 s at 300 km/h). 30 gave little more (32-52) for more sideslip.
		internal static float CoupRollFloor = 20f;

		// Compass 0.1.3: roll-coupling limit without pitch TVC (see the coupling block)
		internal static bool CoupNoTvcAlways = true;

		internal static float CoupNoTvcA0 = 3f;

		internal static float CoupNoTvcSpan = 4f;

		internal static float CoupRollFloorNoTvc = 12f;

		// bench knob: caps the high-lift devices' deployment (1 = stock schedule)
		internal static float HldMaxPos = 1f;

		// Compass 0.1.3: share of the estimated pitch authority the pitch-rate trajectory may use (F-22E / Ifrit / Revoker: 0.7).
		// The T/A-30's slabs are its only pitch effector and at the AoA limits the trajectory, not the slabs, set the reversal pace:
		// pushing from +35 deg the elevators sat at a constant ~10 deg local AoA while the target rate ramped at 30 deg/s^2 (5 Nov
		// 17:41 log, 164-166 s: aDn 41-53, achieved 27-30). Bench with the slab tail: 20 deg of AoA off the +35 limit in 1.2 s at
		// 300 km/h (0.1.2: 1.5), 20 deg off the -30 limit in 0.9 s (1.1); no overshoot at the captures.
		internal static float PitchTrajAuthFrac = 1.0f;

		// Compass 0.1.5: roll-biased flaps (see the hinge droop in the flap update)
		public float flapRollBias;

		// Su-30 0.1.4 (FlaperonRollPriority): the gear-driven landing flap on the flaperons (the game's ControlSurfaceJob_Math: a
		// flap surface slews to -pitchRange at its servo rate while the gear is extending / extended) yields to roll, as the
		// F-22E's flaperon droop does. In the air its range comes down with flapRollBias (lateral stick or a running roll command,
		// 10 % -> 60 % of full: out over ~0.3 s, back over ~0.7 s), slewed at the flap's own servo rate; on the wheels it stays.
		// The flap sits above the flaperon's roll hinge, so the flaperon models see the change by themselves.
		private readonly Dictionary<ControlSurface, float> landFlapRng = new Dictionary<ControlSurface, float>();

		internal float landFlapDbg = -1f;

		internal void LandingFlapJob(ControlSurface cs, ref NuclearOption.Jobs.ControlSurfaceFields job)
		{
			// Su-30 0.5.5 (FlaperonFlapSchedule): the flap flies the F-22E flap schedule, gear up or down. The game's job slews the flap
			// toward -pitchRange at its servo rate while gearState is LockedExtended / Extending and clamps it to [-pitchRange, 0], so
			// with gearState held at LockedExtended and pitchRange = this tick's step the flap lands exactly on the step, out or in.
			if (Plugin.FlaperonFlapSchedule)
			{
				float fCur = Mathf.Max(0f, 0f - job.currentPitch);
				float fNext = GameFlapStep(cs, fCur, job.servoSpeed, Time.fixedDeltaTime);
				job.gearState = LandingGear.GearState.LockedExtended;
				job.pitchRange = fNext;
				landFlapDbg = fCur;
				return;
			}
			float full = Mathf.Abs(Acc.CS_pitchRange(cs));
			float tgt = full * (1f - Plugin.FlaperonRollPriority * Mathf.Clamp01(flapRollBias));
			if (!landFlapRng.TryGetValue(cs, out var cur))
			{
				cur = full;
			}
			cur = Mathf.MoveTowards(cur, tgt, Mathf.Max(1f, job.servoSpeed) * Time.fixedDeltaTime);
			landFlapRng[cs] = cur;
			job.pitchRange = cur;
			landFlapDbg = 0f - job.currentPitch;
		}

		// Su-30 0.5.5: one step of the flap (deg, trailing edge down) toward the schedule: the F-22E flap schedule's droop (flapBias:
		// AoA, load, the high-lift speed schedule, the flaperon's stall cap), up to FlaperonFlapMax, less what roll takes
		// (FlaperonRollPriority x flapRollBias), at the flap's servo rate. Records the present droop for the stall cap.
		internal float GameFlapStep(object key, float cur, float servo, float dt)
		{
			float tgt = Mathf.Clamp(flaps.flapBias, 0f, Plugin.FlaperonFlapMax) * (1f - Plugin.FlaperonRollPriority * Mathf.Clamp01(flapRollBias));
			float next = Mathf.MoveTowards(cur, tgt, Mathf.Max(1f, servo) * dt);
			gameFlapNow[key] = cur;
			landFlapDbg = cur;
			float mx = 0f;
			foreach (float v in gameFlapNow.Values)
			{
				mx = Mathf.Max(mx, v);
			}
			flaps.gameFlapDroop = mx;
			return next;
		}

		private readonly Dictionary<object, float> gameFlapNow = new Dictionary<object, float>();

		internal static void LandingFlapRestore(ControlSurface cs, ref NuclearOption.Jobs.ControlSurfaceFields job)
		{
			float full = Mathf.Abs(Acc.CS_pitchRange(cs));
			if (Mathf.Abs(job.pitchRange - full) > 1E-04f)
			{
				job.pitchRange = Mathf.MoveTowards(job.pitchRange, full, Mathf.Max(1f, job.servoSpeed) * Time.fixedDeltaTime);
			}
		}

		internal static float FlapRollDem0 = 0.1f;

		internal static float FlapRollDem1 = 0.6f;

		internal static float FlapRollRetractRate = 3f;

		internal static float FlapRollRestoreRate = 1.5f;

		// 2.6.9: the coupling roll limit from the pitch axis's measured nose-down headroom (false = the table cap + gates)

		// bench: split the blend's two effects (target mix with the rate cap / relaxation of the acceleration cap)
		internal static float PaceBlendTargetK = 1f;

		internal static float PaceBlendAccelK = 1f;

		public float coupHeadDbg;

		public float rcAOppDbg;

		public float rcTcDbg;

		public float rcSinHDbg;

		public float coupRollLim;

		private bool branchGateOn;

		internal static float BranchFailTime = 0.25f;

		internal static bool BranchFreeTvc = true;


		internal static bool BranchScaleToPeak = true;

		internal static bool BranchFullTravel = true;

		internal static float BranchDeepMargin = 5f;

		internal static float BranchCoolTime = 0.25f; // 2.0.1: 1 s -> 0.25 s (a stab could park deep in stall during the wait)

		internal static float BranchCoolTimeStop = 0.25f;

		internal static float LatTrimGain = 100f;

		internal static bool LatReservePitchFrac = true;
		public float latFloorDbg;
		internal static bool ReleaseCoordinate = true;   // 2.18.0: released roll under a capped nose-down sweep, fins gone: target = r / sin AoA
		internal static bool SweepHoldFade = true;       // 2.18.0: the nose waits above the rudder fade while excess yaw remains, fins gone
		internal static float SweepHoldRate = 12f;       // 2.19.0: velocity-roll rotation (deg/s) above which the nose waits, fins gone
		internal static float SweepHoldRateBlend = 6f;   // deg/s of blend above it
		internal static float SweepHoldStabFrac = 0.25f; // the hold stands while stabPitchFrac is below this (stabs off pitch)
		internal static bool PaceHoldPacedByRudder = true;   // 2.19.0: the window paces where the rudders are not the yaw axis (rudW < 1)
		public float sweepHoldDbg;
		public float pSStickRaw;
		internal static bool PaceNoRetreat = true;   // 2.18.0: the latched window never moves the target back toward the rotation
		internal static float PaceHoldCapFloor = 5f;   // deg/s^2


		public float capSweepDbg, capShedDbg;   // 2.23.0: the two AoA-rate caps on the nose-down sweep this tick (deg/s; 60 = open) - which one binds
		public float releaseBrakeTdDbg;   // 2.35.3: the device is deleted; the CSV column is kept (always 0)
		public float sweepDelF;   // 2.24.0: the delivered wind-yaw deceleration the sweep cap credited (deg/s^2, low-passed)
		public float stickRelW;   // 2.24.0: lateral stick released (1) .. held (0), over SweepNoReverseStick of stick travel
		public float relFollowDbg;   // 2.35.3: the device is deleted; the CSV column is kept (always 0)
		// 2.34.0 (PushGov): the lateral limit this tick (deg/s^2 of velocity-roll change allowed: < 0 the most the rotation may come down,
		// > 0 what it must grow; NaN = not in force), the rotation side, the roll capacity read (3-tick, TradeCapTau)
		public float pushGovL = float.NaN, pushGovU, pushGovA;

		private bool pushPlanStepped;

		internal static bool GyroRollTrade = true;

		internal static float GyroRollShare = 1f;

		public float gyroRollW, gyroQNeed, gyroBudgetA, gyroRollLimDbg;
		private float pushGovSide, pushGovCapXF, pushGovCapYF, pushGovP0, pushGovGrowF;
		private bool pushGovXInit, pushGovActive;
		public float pushGovCeil, pushGovP0Dbg, pushGovAMin;
		private int reachLogPrev;
		public bool pushPlanOn; public float pushPlanQ, pushPlanQNow, pushPlanBleed, pushPlanT, pushPlanSide, pushPlanD, pushHeldB;

		public float aYawShedDbg;   // 2.22.0: the shed the cap paced to (deg/s^2)
		internal static bool LatLimitYawFloor = true;   // 2.16.0: the lateral limit's coordination floor (see num11)
		internal static float LatFloorSinMin = 0.26f;    // sin 15 deg

		public float aUpPitch;

		// Revoker 0.2.12: see PitchLaw (lateral reserve). false = 0.2.11.
		internal static bool LatReserveTvcFull = true;

		public float aDnPitch;


		internal static bool LeadAgreeGate = true;

		public float pitchPrio = 1f;

		public bool stabRateBound;

		private float tvcTrimRef;

		private float tvcSlow;

		private float[] cmdSave = new float[64];

		public bool SplitDeploy
		{
			get
			{
				return (Plugin.AirbrakeSplit || !Plugin.ThrottleIndependentAirbrake) && AirbrakeOut;
			}
		}

		// Su-30 0.1.2 (RateCapRudderShare)
		internal static bool RateCapRudderShare = true;

		internal static float RateCapShareFull = 0.5f;


		public float rudShareDbg = 1f;

		// Su-30 0.1.1 (CantedVvRollWeight)
		public float cantVvWDbg = 1f;

		private float canPitchFirstW;

		private float vvPitchLagF;

		public float leafBoostDbg = 1f;

		public float aYawTvc;

		private Lpf aYawTvcF;

		public float yawThrustShareDbg;

		internal static bool YawThrustShare = true;

		internal static float LeafBoostA0 = 55f;

		internal static float LeafBoostIasLo = 115f;

		internal static float LeafBoostIasHi = 250f;

		internal static bool VvWeightPitchGate = true;

		public float canPitchFirstDbg;

		public float surfFirstDbg;

		public float gcReachDbg = 1f;

		public float canPeakLo, canPeakHi;

		private float canPeakHold;


		internal static float CantVvA0 = 25f;

		internal static float CantVvSpan = 20f;

		private LandingGear NoseGear()
		{
			if (noseGearSearched)
			{
				return noseGearCached;
			}
			noseGearSearched = true;
			LandingGear landingGear = null;
			float num = float.MinValue;
			float num2 = float.MinValue;
			int num3 = 0;
			foreach (LandingGear gear in gears)
			{
				if (!(gear == null))
				{
					num3++;
					float num4 = Vector3.Dot(gear.transform.position - ((ac.rb != null) ? ac.rb.worldCenterOfMass : ((Component)(object)ac).transform.position), ((Component)(object)ac).transform.forward);
					if (num4 > num)
					{
						num2 = num;
						num = num4;
						landingGear = gear;
					}
					else if (num4 > num2)
					{
						num2 = num4;
					}
				}
			}
			noseGearCached = ((num3 >= 2 && num - num2 > 1.5f) ? landingGear : null);
			Plugin.Log.LogInfo((noseGearCached != null) ? $"Su-30 FCS: nose gear '{noseGearCached.name}' ({num - num2:F1} m ahead of the next leg), {num3} gear legs." : $"Su-30 FCS: no distinct nose gear among {num3} gear legs; rotation law disabled.");
			return noseGearCached;
		}

		public FcsController(Aircraft aircraft)
		{
			ac = aircraft;
			inputs = aircraft.GetInputs();
			AircraftParameters aircraftParameters = aircraft.GetAircraftParameters();
			foreach (ControlSurface item in Acc.Collect<ControlSurface>(aircraft))
			{
				// Su-30 0.1.0: the gear-driven landing flaps (flap = 1: Flaperon_*_LandingBias, 15 deg trailing edge down while the gear is
				// extending / extended) stay with the game, and the outer flaps (every range 0: they never move) stay fixed (Tom); neither is
				// an FCS effector. The landing flap sits above the flaperon's roll hinge, so the flaperon models see its deflection.
				if (Acc.CS_flap(item) || Mathf.Abs(Acc.CS_pitchRange(item)) + Mathf.Abs(Acc.CS_rollRange(item)) + Mathf.Abs(Acc.CS_yawRange(item)) < 0.01f)
				{
					continue;
				}
				AeroPart aeroPart = Acc.CS_attached(item) as AeroPart;
				if (!(aeroPart == null))
				{
					SurfaceKind kind = Classify(item);
					surfaces.Add(new SurfaceEffector(item, aeroPart, kind, FoilFor(aeroPart, aircraftParameters)));
				}
			}
			if (surfaces.Count < 5)
			{
				throw new InvalidOperationException("Su-30 FCS: expected 5 control surfaces, found " + surfaces.Count);
			}
			// 2.2.0 flap-camber coupling: each rudder / flaperon / aileron knows the fixed part ahead of it
			Dictionary<string, AeroPart> partsByName = new Dictionary<string, AeroPart>();
			foreach (AeroPart apH in Acc.Collect<AeroPart>(aircraft))
			{
				partsByName[Names.Canon(apH.gameObject.name)] = apH;
			}
			foreach (SurfaceEffector seH in surfaces)
			{
				string hn = (seH.cs != null) ? Camber.HostOf(seH.cs.gameObject.name) : null;
				if (hn != null && partsByName.TryGetValue(hn, out var hp))
				{
					seH.host = hp;
					seH.hostFoil = FoilFor(hp, aircraftParameters);
					// Revoker 0.2.20: the camber increment on the host follows the elevon's own (staggered) curve
					StallStagger.Fam fH = StallStagger.Enabled ? StallStagger.FamilyOf(seH.cs.gameObject.name) : StallStagger.Fam.None;
					if (fH != StallStagger.Fam.None)
					{
						seH.hostFoil = StallStagger.TableForFamily(AirfoilOf(hp, aircraftParameters), fH);
					}
				}
			}
			foreach (SurfaceEffector surface in surfaces)
			{
				if (surface.kind == SurfaceKind.Stabilator)
				{
					surface.mu = 0.0002f;
					surface.linearPitch = true;
					// Ternion 0.1.0 (PitchSurfaceMoveCost): a cost on moving the canards / elevators from their last command
					surface.lam = Plugin.PitchSurfaceMoveCost;
				}
				else
				{
					// 2.0.6: with RudderPitchAid the rudders keep their true pitch moment in the allocator's model (toe-in /
					// toe-out of the canted fins), so a symmetric rudder split is available as a pitch effector at a cost
					surface.noPitch = !(surface.kind == SurfaceKind.Rudder && Plugin.RudderPitchAid);
					// Revoker 0.1.0 (ElevonPitch): the delta's trailing-edge flaperons with their true pitch moment
					if (surface.kind == SurfaceKind.Aileron && Plugin.ElevonPitch)
					{
						surface.noPitch = false;
					}
				}
				surface.rollGroup = ((surface.kind == SurfaceKind.Stabilator) ? 1 : ((surface.kind == SurfaceKind.Aileron) ? 2 : ((surface.kind == SurfaceKind.Flaperon) ? 3 : 0)));
				surface.pitchGroup = ((surface.kind == SurfaceKind.Stabilator) ? 1 : 0);
				effs.Add(surface);
			}
			// 2.0.0: the roll-time nozzle budget and the roll-pitch pass fade out by body AoA at points chosen for Aryx's stab
			// foil (peak ~34 deg local). They stand for "the roll's down-going stab is reaching the flat top of its lift
			// curve", so they move with the stab's own modelled peak (StallStagger): 25 deg stabs -> ~8.5 deg lower.
			float stabPeak = float.MaxValue;
			foreach (SurfaceEffector sp0 in surfaces)
			{
				if (sp0.kind == SurfaceKind.Stabilator)
				{
					stabPeak = Mathf.Min(stabPeak, sp0.stallAoA);
				}
			}
			stabStallShift = (StabPeakShift && stabPeak < 90f) ? Mathf.Max(0f, StabPeakRef - stabPeak) : 0f;
			foreach (SurfaceEffector l in surfaces)
			{
				if (!l.name.EndsWith("_L"))
				{
					continue;
				}
				SurfaceEffector surfaceEffector = surfaces.Find((SurfaceEffector o) => o.kind == l.kind && o.name == l.name.Substring(0, l.name.Length - 2) + "_R");
				if (surfaceEffector == null)
				{
					continue;
				}
				bool num = l.kind == SurfaceKind.Rudder;
				float num2 = (num ? Mathf.Sign(l.origYawRange) : Mathf.Sign(l.origRollRange));
				float num3 = (num ? Mathf.Sign(surfaceEffector.origYawRange) : Mathf.Sign(surfaceEffector.origRollRange));
				if (num2 != 0f && num3 != 0f && num2 != num3)
				{
					l.partner = surfaceEffector;
					l.pairSign = num2;
					surfaceEffector.pairSign = num3;
					if (l.kind == SurfaceKind.Stabilator)
					{
						// Ternion 0.1.0: the canards carry the steady pitch (StabilatorTrimCost, the Revoker's); the aft elevators
						// are the second pitch pair (ElevatorTrimCost), so the two pairs do not trade deflection against each other
						l.symCost = l.isCanard ? Plugin.StabTrimCost : Plugin.ElevatorTrimCost;
						l.antiCost = 0.01f;
					}
					else
					{
						l.symCost = (l.kind == SurfaceKind.Rudder && Plugin.RudderPitchAid) ? Plugin.RudderToeCost : ((l.kind == SurfaceKind.Aileron) ? Plugin.ElevonSymCost : 2f);
						l.antiCost = 0f;
					}
				}
			}
			// Su-30 0.1.0 (CanardSymmetricOnly): the canards' stock ranges carry no roll sign (pitch 10, roll 0 on both), so the pairing above
			// leaves them unpaired; they are paired here as a pitch-only pair: the common (pitch) part at the canards' trim cost, the
			// differential at a prohibitive cost, so the allocator never plans a split (and the command is averaged before it is sent)
			canL = surfaces.Find((SurfaceEffector o) => o.isCanard && o.name.EndsWith("_L"));
			canR = surfaces.Find((SurfaceEffector o) => o.isCanard && o.name.EndsWith("_R"));
			if (canL != null && canR != null)
			{
				if (Plugin.CanardSymmetric)
				{
					canL.partner = canR;
					canL.pairSign = 1f;
					canR.pairSign = -1f;
					canL.symCost = Plugin.StabTrimCost;
					canL.antiCost = CanardSplitCost;
					canL.rollGroup = 0;
					canR.rollGroup = 0;
				}
			}
			tvc = new TvcEffector(Acc.Collect<Turbojet>(aircraft));
			effs.Insert(0, tvc);
			// Su-30 0.1.0 (CantedTvc): the AL-31FP's single-axis nozzles canted 32 deg outboard; both TVC effectors build their tables from it
			// and Hooks.SetNozzleEuler drives the nozzle transforms
			if (Plugin.CantedTvc)
			{
				CantedTvc ct = new CantedTvc(tvc.engines, ((Component)(object)aircraft).transform, (Plugin.TvcRange > 0f) ? Plugin.TvcRange : tvc.max);
				if (ct.Valid)
				{
					canted = ct;
					tvc.canted = ct;
					tvc.min = 0f - ct.max;
					tvc.max = ct.max;
					tvc.lo = tvc.min;
					tvc.hi = tvc.max;
					for (int gi = 0; gi < tvc.N; gi++)
					{
						tvc.grid[gi] = tvc.min + (float)gi * (tvc.max - tvc.min) / (float)(tvc.N - 1);
					}
				}
			}
			// Ternion 0.1.1: a jet with no vectorable nozzle at all (the FS-3) - the no-TVC caps exist for TVC jets whose nozzles
			// stop covering pitch at low thrust; here they are not a thrust state, the airframe never had nozzle pitch
			noVecJet = true;
			foreach (Turbojet eV in tvc.engines)
			{
				Transform[] vt = (eV != null) ? Acc.TF_vecT(eV) : null;
				if (vt != null && vt.Length > 0 && Mathf.Abs(Acc.TF_vec(eV).x) > 0.5f)
				{
					noVecJet = false;
				}
			}
			tvc.pitchGroup = 2;
			// Ifrit: split-flap decelerons and differential (roll) TVC as allocator effectors
			if (Plugin.IfritSplitYaw)
			{
				foreach (SurfaceEffector sf in surfaces)
				{
					if (Acc.CS_maxSplit(sf.cs) > 0f && sf.part != null)
					{
						SplitEffector spe = new SplitEffector(sf.cs, sf.part);
						spe.host = sf;
						effs.Add(spe);
						splits.Add(spe);
					}
				}
			}
			if (Plugin.IfritTvcRoll && tvc.engines.Count >= 2)
			{
				tvcd = new TvcDiffEffector(tvc);
				if (canted != null)
				{
					tvcd.UseCanted(canted);
				}
				if (tvcd.max > 0.5f)
				{
					effs.Add(tvcd);
				}
				else
				{
					tvcd = null;
				}
			}
			// Revoker 0.1.0: yaw TVC (the single nozzle's yaw axis) as an allocator effector
			if (Plugin.YawTvc)
			{
				float vyMax = 0f;
				foreach (Turbojet ey in tvc.engines)
				{
					if (ey != null)
					{
						vyMax = Mathf.Max(vyMax, Mathf.Abs(Acc.TF_vec(ey).y));
					}
				}
				if (vyMax > 0.5f)
				{
					tvc.yawSeparate = true;
					tvcy = new TvcYawEffector(tvc);
					effs.Add(tvcy);
				}
			}
			foreach (AeroPart apF in Acc.Collect<AeroPart>(aircraft))
			{
				string fn = Names.Canon(apF.gameObject.name).ToLowerInvariant();
				if (fn.StartsWith("vstab") || fn == "rudder")
				{
					finParts.Add(apF);
					finAreaTotal += Mathf.Max(0f, Acc.AP_wingArea(apF));
				}
			}
			gears.AddRange(Acc.Collect<LandingGear>(aircraft));
			try
			{
				foreach (Airbrake ab in Acc.Collect<Airbrake>(aircraft))
				{
					float dA = (fABDrag != null) ? Convert.ToSingle(fABDrag.GetValue(ab)) : 0f;
					Component pc = (fABPart != null) ? (fABPart.GetValue(ab) as Component) : null;
					Rigidbody rbA = (pc is UnitPart up) ? up.rb : null;
					if (rbA != null && dA > 0f)
					{
						airbrakes.Add(ab);
						abRb.Add(rbA);
						abDrag.Add(dA);
						abPart.Add(pc as UnitPart);
					}
				}
			}
			catch (Exception exAB)
			{
				Plugin.Log.LogWarning("Su-30 FCS airbrake drag lookup: " + exAB.Message);
			}
			foreach (UnitPart item2 in aircraft.partLookup)
			{
				if (item2 is AeroPart aeroPart2)
				{
					aeroParts.Add(aeroPart2);
					if (!foils.ContainsKey(aeroPart2))
					{
						foils[aeroPart2] = FoilFor(aeroPart2, aircraftParameters);
					}
				}
			}
			RebuildBodies();
			flaps = new FlapSystem(aircraft, surfaces);
			// Revoker 0.2.0 (FlapFlaperons): flap_L/R, driven by the HLD only in the game, become allocator effectors (pitch and
			// lateral) with wing2_X as their camber host; the FCS's FlapSystem then leaves their two transforms alone
			if (Plugin.FlapFlaperons)
			{
				foreach (AeroPart apF in aeroParts)
				{
					string fn = apF.gameObject.name;
					if (!(fn == "flap_L" || fn == "flap_R"))
					{
						continue;
					}
					Transform visF = Acc.AP_liftNormal(apF);
					if (visF == null || visF == apF.transform)
					{
						continue;
					}
					// Compass 0.1.0: the flap sits behind wing1_X (the stock HLD turns wing1_X_liftNormal 10 deg per 20 deg of flap)
					AeroPart hostF = aeroParts.Find((AeroPart o) => o.gameObject.name == "wing1_" + fn.Substring(5));
					Transform hostLN = (hostF != null) ? Acc.AP_liftNormal(hostF) : null;
					if (hostLN == hostF?.transform)
					{
						hostLN = null;
					}
					// Revoker 0.2.20: with ElevonStallStagger the camber share on wing2 uses the inboard elevon's curve (wing2's airfoil, remapped)
					AirfoilTable hostTbl = (hostF != null && foils.TryGetValue(hostF, out var tH)) ? tH : null;
					StallStagger.Fam fF = StallStagger.Enabled ? StallStagger.FamilyOf(apF.gameObject.name) : StallStagger.Fam.None;
					if (hostF != null && fF != StallStagger.Fam.None)
					{
						hostTbl = StallStagger.TableForFamily(AirfoilOf(hostF, aircraftParameters), fF);
					}
					HingeFlapEffector hf = new HingeFlapEffector(apF, visF, hostF, hostLN, foils.TryGetValue(apF, out var tF) ? tF : FoilFor(apF, aircraftParameters), hostTbl, Plugin.FlapHingeRange, flaps.RestOf(visF), (hostLN != null) ? flaps.RestOf(hostLN) : null);
					if (hostTbl != null && fF != StallStagger.Fam.None)
					{
						CamberForce.AddHinge(ac, hf, hostTbl);
					}
					hf.servo = (Plugin.SurfaceServoSpeed > 0f) ? Plugin.SurfaceServoSpeed : 70f;
					hf.rollGroup = HingeRollGroup;
					hinges.Add(hf);
					effs.Add(hf);
					flaps.Own(visF);
					if (hostLN != null)
					{
						flaps.Own(hostLN);
					}
				}
				HingeFlapEffector hL = hinges.Find((HingeFlapEffector h) => h.left);
				HingeFlapEffector hR = hinges.Find((HingeFlapEffector h) => !h.left);
				if (hL != null && hR != null)
				{
					hL.partner = hR;
					hL.mate = hR;
					hR.mate = hL;
					hL.pairSign = -1f;
					hR.pairSign = 1f;
					hL.symCost = Plugin.ElevonSymCost * HingeSymCostK * HingeSymK;
					hL.antiCost = 0f;
				}
			}
			Plugin.Log.LogInfo(string.Format("Su-30 FCS attached: {0} surfaces [{1}], ", surfaces.Count, string.Join(", ", surfaces.ConvertAll((SurfaceEffector s) => s.name + ":" + s.kind.ToString() + ((s.min == -s.max) ? (" ±" + s.max.ToString("0")) : (" " + s.min.ToString("0") + "/+" + s.max.ToString("0")))))) + $"TVC engines {tvc.engines.Count} (±{tvc.max:0} deg pitch, roll {(tvcd != null ? "±" + tvcd.max.ToString("0") : "off")}, yaw {(tvcy != null ? "±" + tvcy.max.ToString("0") : "off")}), fin area {finAreaTotal:F1} m², splits {splits.Count}, inboard elevons {hinges.Count} (±{(hinges.Count > 0 ? hinges[0].max : 0f):0}), {bodies.Count} rigidbodies, {flaps.Describe()}.");
		}

		private static SurfaceKind Classify(ControlSurface cs)
		{
			string name = Names.Canon(cs.gameObject.name);   // Su-30 0.1.0
			// Ifrit: the stock surfaces are named elevator_L / aileron_L / flap_L; match case-insensitively
			string lower = name.ToLowerInvariant();
			name = lower.Contains("elevator") ? "Elevator" : lower.Contains("rudder") ? "Rudder" : lower.Contains("flap") ? "Flap" : (lower.Contains("aileron") || lower.Contains("rollervon")) ? "Aileron" : name;
			if (Mathf.Abs(Acc.CS_pitchRange(cs)) > 0.01f || name.Contains("Elevator") || name.Contains("Stab"))
			{
				return SurfaceKind.Stabilator;
			}
			if (name.Contains("Rudder"))
			{
				return SurfaceKind.Rudder;
			}
			if (name.Contains("Flap"))
			{
				return SurfaceKind.Flaperon;
			}
			if (name.Contains("Aileron"))
			{
				return SurfaceKind.Aileron;
			}
			if (Mathf.Abs(Acc.CS_yawRange(cs)) > Mathf.Abs(Acc.CS_rollRange(cs)))
			{
				return SurfaceKind.Rudder;
			}
			return SurfaceKind.Other;
		}

		private static Airfoil AirfoilOf(AeroPart part, AircraftParameters prm)
		{
			int k = Acc.AP_airfoil(part);
			return (k >= 0 && prm != null && prm.airfoils != null && k < prm.airfoils.Length) ? prm.airfoils[k] : null;
		}

		private static AirfoilTable FoilFor(AeroPart part, AircraftParameters prm)
		{
			Airfoil af = null;
			int num = Acc.AP_airfoil(part);
			if (num >= 0 && prm != null && prm.airfoils != null && num < prm.airfoils.Length)
			{
				af = prm.airfoils[num];
			}
			return StallStagger.TableFor(part, af);
		}

		private void RebuildBodies()
		{
			bodies.Clear();
			foreach (UnitPart item in ac.partLookup)
			{
				if (!(item == null) && !(item.rb == null) && !item.IsDetached() && !bodies.Contains(item.rb))
				{
					bodies.Add(item.rb);
				}
			}
			if (bodies.Count == 0 && ac.rb != null)
			{
				bodies.Add(ac.rb);
			}
		}

		public void Dispose()
		{
			try
			{
				estJobF?.Join();
				FlushPendingRow();
			}
			catch (Exception)
			{
			}
			telemetry?.Close();
			telemetry = null;
		}

		private void UpdateMassProps(bool full, Quaternion rootInv)
		{
			float num = 0f;
			Vector3 zero = Vector3.zero;
			Vector3 vSum = Vector3.zero;
			for (int num2 = bodies.Count - 1; num2 >= 0; num2--)
			{
				Rigidbody rigidbody = bodies[num2];
				if (rigidbody == null)
				{
					bodies.RemoveAt(num2);
				}
				else
				{
					num += rigidbody.mass;
					zero += rigidbody.mass * rigidbody.worldCenterOfMass;
					vSum += rigidbody.mass * rigidbody.velocity;
				}
			}
			if (num < 1f)
			{
				return;
			}
			mass = num;
			cg = zero / num;
			vCg = vSum / num;
			cgValid = true;
			if (!thrustLineReg && tvc != null && tvc.engines != null)
			{
				// Su-30 0.1.5 (ThrustLineAtCg): this jet's nozzles get their force point moved to the CG's height
				foreach (Turbojet eT in tvc.engines)
				{
					JetNozzle[] nzT = (eT != null) ? Acc.TF_nozzles(eT) : null;
					if (nzT == null)
					{
						continue;
					}
					foreach (JetNozzle jT in nzT)
					{
						Transform tT = (jT != null) ? Acc.JN_thrustT(jT) : null;
						if (tT != null)
						{
							ThrustLine.Register(tT, this);
							thrustLineReg = true;
						}
					}
				}
			}
			if (!full)
			{
				return;
			}
			double[,] array = new double[3, 3];
			foreach (Rigidbody body in bodies)
			{
				Matrix4x4 matrix4x = Matrix4x4.Rotate(body.rotation * body.inertiaTensorRotation);
				Vector3 inertiaTensor = body.inertiaTensor;
				Vector3 vector = body.worldCenterOfMass - cg;
				float sqrMagnitude = vector.sqrMagnitude;
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						double num3 = matrix4x[i, 0] * inertiaTensor.x * matrix4x[j, 0] + matrix4x[i, 1] * inertiaTensor.y * matrix4x[j, 1] + matrix4x[i, 2] * inertiaTensor.z * matrix4x[j, 2];
						num3 += (double)(body.mass * (((i == j) ? sqrMagnitude : 0f) - vector[i] * vector[j]));
						array[i, j] += num3;
					}
				}
			}
			Matrix4x4 matrix4x2 = Matrix4x4.Rotate(rootInv);
			double[,] array2 = new double[3, 3];
			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					double num4 = 0.0;
					for (int m = 0; m < 3; m++)
					{
						for (int n = 0; n < 3; n++)
						{
							num4 += (double)matrix4x2[k, m] * array[m, n] * (double)matrix4x2[l, n];
						}
					}
					array2[k, l] = num4;
				}
			}
			int[] array3 = new int[3] { 2, 0, 1 };
			float[] array4 = new float[3] { -1f, -1f, 1f };
			for (int num5 = 0; num5 < 3; num5++)
			{
				for (int num6 = 0; num6 < 3; num6++)
				{
					Ia[num5, num6] = (float)((double)(array4[num5] * array4[num6]) * array2[array3[num5], array3[num6]]);
				}
			}
			Invert3(Ia, IaInv);
			alloc.SetInverseInertia(IaInv);
		}

		private static void Invert3(float[,] a, float[,] o)
		{
			float num = a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0]) + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
			if (Mathf.Abs(num) < 1E-06f)
			{
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						o[i, j] = ((i == j) ? (1f / Mathf.Max(1f, a[i, i])) : 0f);
					}
				}
				return;
			}
			float num2 = 1f / num;
			o[0, 0] = (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) * num2;
			o[0, 1] = (a[0, 2] * a[2, 1] - a[0, 1] * a[2, 2]) * num2;
			o[0, 2] = (a[0, 1] * a[1, 2] - a[0, 2] * a[1, 1]) * num2;
			o[1, 0] = (a[1, 2] * a[2, 0] - a[1, 0] * a[2, 2]) * num2;
			o[1, 1] = (a[0, 0] * a[2, 2] - a[0, 2] * a[2, 0]) * num2;
			o[1, 2] = (a[0, 2] * a[1, 0] - a[0, 0] * a[1, 2]) * num2;
			o[2, 0] = (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]) * num2;
			o[2, 1] = (a[0, 1] * a[2, 0] - a[0, 0] * a[2, 1]) * num2;
			o[2, 2] = (a[0, 0] * a[1, 1] - a[0, 1] * a[1, 0]) * num2;
		}

		private Vector3 MulI(Vector3 v)
		{
			return new Vector3(Ia[0, 0] * v.x + Ia[0, 1] * v.y + Ia[0, 2] * v.z, Ia[1, 0] * v.x + Ia[1, 1] * v.y + Ia[1, 2] * v.z, Ia[2, 0] * v.x + Ia[2, 1] * v.y + Ia[2, 2] * v.z);
		}

		private static float[] BuildAsymAlphas()
		{
			float[] array = new float[22];
			for (int i = 0; i < 22; i++)
			{
				array[i] = -40f + 5f * (float)i;
			}
			return array;
		}

		// 1.8.35: past the normal rudder fade (RudderFadeStartAoA-EndAoA) the canted rudders' "inverse roll" is what
		// moves sideslip: beta-dot = p sin(a) - r cos(a), and a yaw-right rudder command rolls the jet left, so both terms
		// push beta the same way. Bench sweep: from ~55 deg AoA their sideslip effect is >3x their velocity-vector roll
		// (6x at 75), while the flaperons' and stabs' sideslip authority collapses near 60 and reverses past ~62.
		// So above the fade the rudders get a limited range that can only REDUCE the sideslip error: deflection only in
		// the beta-reducing direction (taken from the live surface model, so it follows the sign flip at negative AoA),
		// scaled in by the error (0 below 0.5 deg, full at 3 deg), and capped so the body yaw acceleration the rudders make
		// stays within RudderSideslipYawCap. Their yaw per degree falls with AoA (about 0 by 80 deg), so the travel opens
		// up as AoA rises. Uses the previous tick's surface model and pedal-sideslip command.
		private void SideslipRudder(float dt)
		{
			rudSideDbg = 0f;
			float cap = Plugin.RudderSideslipYawCap;
			float gate = (1f - rudderAuth) * ((Mathf.Abs(alpha) < 100f) ? 1f : 0f);
			SurfaceEffector rl = null;
			SurfaceEffector rr = null;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Rudder)
				{
					if (se.name.EndsWith("_L"))
					{
						rl = se;
					}
					else if (se.name.EndsWith("_R"))
					{
						rr = se;
					}
				}
			}
			if (cap <= 0f || gate <= 0.001f || !Engaged || !havePrev || Mode == FcsMode.Ground || Mode == FcsMode.Off || rl == null || rr == null || !rl.active || !rr.active || rl.N < 2 || rr.N < 2)
			{
				rudSideF = Mathf.MoveTowards(rudSideF, 0f, 60f * dt);
				return;
			}
			float sL = Mathf.Sign(rl.origYawRange);
			float sR = Mathf.Sign(rr.origYawRange);
			const float probe = 10f;
			Vector3 m = rl.MAt(sL * probe) + rr.MAt(sR * probe) - rl.MAt(0f) - rr.MAt(0f);
			float pDot = AccRoll(m) / probe;
			float rDot = AccYaw(m) / probe;
			float ar = alpha * (MathF.PI / 180f);
			float bdd = pDot * Mathf.Sin(ar) - rDot * Mathf.Cos(ar);
			float err = beta - betaPedalCmd;
			float y = 0f;
			if (Mathf.Abs(bdd) > 1E-4f)
			{
				float span = Mathf.Min(Mathf.Max(rl.max, 0f - rl.min), Mathf.Max(rr.max, 0f - rr.min));
				float lim = Mathf.Min(span, cap / Mathf.Max(Mathf.Abs(rDot), 1E-4f));
				float use = Mathf.Clamp01((Mathf.Abs(err) - 0.5f) / 2.5f);
				y = (0f - Mathf.Sign(err) * Mathf.Sign(bdd)) * lim * use * gate;
			}
			// servo-rate-limited so the allowed range cannot jump across zero in one frame when the error changes sign
			rudSideF = Mathf.MoveTowards(rudSideF, y, Mathf.Max(1f, rl.rate) * dt);
			rudSideDbg = rudSideF;
			float yLo = Mathf.Min(0f, rudSideF);
			float yHi = Mathf.Max(0f, rudSideF);
			ApplyYawInterval(rl, sL, yLo, yHi);
			ApplyYawInterval(rr, sR, yLo, yHi);
		}

		private static void ApplyYawInterval(SurfaceEffector se, float sgn, float yLo, float yHi)
		{
			float a = sgn * yLo;
			float b = sgn * yHi;
			float lo = Mathf.Min(a, b);
			float hi = Mathf.Max(a, b);
			se.lo = Mathf.Clamp(Mathf.Min(se.lo, lo), se.min, se.max);
			se.hi = Mathf.Clamp(Mathf.Max(se.hi, hi), se.min, se.max);
		}

		// 1.8.36 fly-by-wire ground steering. Outer loop: pedal -> yaw-rate command (GroundYawRateMax, limited to
		// GroundLateralAccel / V); pedal released -> yaw braking (zero-rate command) until |r| < 0.7 deg/s, then capture the
		// ground track (or the heading when nearly stopped) and the line through the CG, and hold that line: desired track =
		// captured track minus atan(cross-track / max(4 s x V, 25 m)) (<= 5 deg), desired heading = desired track (the heading
		// is lined up with the track: a touchdown crab or a weathervaning crosswind is steered out, the mains' side force
		// removes the drift). Inner loop: nose-wheel angle = kinematic feed-forward atan(r L / V) + yaw-rate P+I, limited to
		// what the tyre can use at that speed (NO's tyre saturates at 1/(0.2 + 0.01 v) deg of slip); in the ground roll the
		// rudders run the same yaw-rate error. Below 1.5 m/s the pedal steers the wheel directly.
		private void GroundSteer(float dt, bool mainsWow, bool noseWowWeight)
		{
			// contact rather than weight: in the ground roll the F-22E's nose leg carries only ~15-20 kN (about 0.02 m of
			// its 0.7 m travel), below WeightOnWheel(0.05), yet the tyre still steers with that load
			bool noseWow = false;
			bool anyContact = mainsWow;
			foreach (LandingGear g in gears)
			{
				if (g != null && g.WeightOnWheel(0.002f))
				{
					anyContact = true;
					if (g == NoseGear())
					{
						noseWow = true;
					}
				}
			}
			gsAnyContact = anyContact;
			// 0.5.3: the brake effector is armed tick by tick (GsBrakeAlloc); the light-nose hold flag for the log
			gsBrkOnPrev = gsBrkOn;
			gsBrkOn = false;
			gsLnDbg = false;
			gsBrkDbgL = -1f;
			gsBrkDbgR = -1f;
			// human pilots only: AI taxi logic is written for the stock pedal-to-wheel-angle steering
			bool on = Plugin.NoseWheelFbw && !isAI && Engaged && Mode != FcsMode.Off && anyContact;
			if (!anyContact && ac.rb != null)
			{
				// airborne: remember the ground track (0.5 s filter) so a touchdown captures the approach track (the runway),
				// not the track the mains have already dragged toward a crabbed heading
				Vector3 va = ac.rb.velocity;
				if (va.x * va.x + va.z * va.z > 400f)
				{
					float ca = Mathf.Atan2(va.x, va.z) * 57.29578f;
					gsChiAir = ((gsAirT > 0f) ? (gsChiAir + Mathf.DeltaAngle(gsChiAir, ca) * (dt / (0.5f + dt))) : ca);
					gsAirT += dt;
				}
				else
				{
					gsAirT = 0f;
				}
			}
			if (!on)
			{
				gsActive = false;
				gsStockPhase = false;
				gsBetaPrev = float.NaN;
				gsRPrev = float.NaN;
				gsRdotF = 0f;
				gsFyF = 0f;
				gsIndiOn = false;
				gsLnHold = false;
				gsInt = 0f;
				gsSinceContact += dt;
				if (gsSinceContact < 1.5f && Plugin.NoseWheelFbw && !isAI)
				{
					// a bounce: keep the captured line and track
					gsNwsDeg = 0f;
					gsRud = 0f;
					gsMainsContact = false;
					return;
				}
				gsHold = false;
				gsRudInt = 0f;
				gsBrakeT = 0f;
				gsRCmdMains = 0f;
				gsSurfV = Vector3.zero;
				gsSurfR = 0f;
				gsNwsDeg = 0f;
				gsRud = 0f;
				gsMainsContact = false;
				return;
			}
			LandingGear nose = NoseGear();
			float steerLock = ((nose != null) ? Mathf.Max(1f, Mathf.Abs(Acc.LG_steerLock(nose))) : 45f);
			if (gsLead < 0f)
			{
				gsLead = 5.9f;
				if (nose != null)
				{
					float best = float.MinValue;
					foreach (LandingGear g in gears)
					{
						if (g != null && g != nose)
						{
							best = Mathf.Max(best, Vector3.Dot(g.transform.position - nose.transform.position, -((Component)(object)ac).transform.forward));
						}
					}
					if (best > 1f && best < 30f)
					{
						gsLead = best;
					}
				}
			}
			Rigidbody rb = ac.rb;
			Vector3 v = ((rb != null) ? rb.velocity : Vector3.zero);
			// 0.5.4: everything on the ground relative to the surface under the wheels, as the game's tyres see it (a moving
			// carrier deck: its rigidbody's point velocity plus an AnimatedPhysicsSurface's velocity). With world velocity the
			// line hold captured the ship's track and steered the jet back toward the ship's heading on every release, and the
			// drift limit (on the aerodynamic sideslip) read the wind over the deck as a skid.
			GsSurface(nose);
			v -= gsSurfV;
			Vector3 vh = new Vector3(v.x, 0f, v.z);
			float V = vh.magnitude;
			Vector3 fwd = ((Component)(object)ac).transform.forward;
			float psi = Mathf.Atan2(fwd.x, fwd.z) * 57.29578f;
			float chi = ((V > 0.5f) ? (Mathf.Atan2(vh.x, vh.z) * 57.29578f) : psi);
			float r = ((rb != null) ? (Vector3.Dot(rb.angularVelocity, Vector3.up) * 57.29578f) : 0f);
			// slip over the surface (+ = moving right of the nose), for the drift limit
			gsSlip = ((V > 1f) ? Vector3.SignedAngle(Vector3.ProjectOnPlane(fwd, Vector3.up), vh, Vector3.up) : 0f);
			Vector3 pos = ((rb != null) ? rb.worldCenterOfMass : ((Component)(object)ac).transform.position);
			float rLim = Mathf.Min(Plugin.GroundYawRateMax, Plugin.GroundLateralAccel / Mathf.Max(V, 0.5f) * 57.29578f);
			float ped = sy;
			float rc;
			// GroundStockMatch (0.5.2): stock steering, a drift limit, and the straight-line correction. While the pedal is in,
			// and after release until the yaw has stopped, the nose leg, the rudders and the main legs' differential brake get
			// the stock fly-by-wire's yaw - clamp(yawTightness x remap x (pedal - yaw rate) + yawWeathervaning x sideslip) with
			// the jet's own ControlsFilter parameters, raw pedal below its minSpeed - exactly as stock steers. On top of it the
			// drift limit lets the sideslip reach GroundDriftAllow and counters what goes past it (sideslip led by its rate,
			// faded in over GroundDriftFadeSpeed .. +10 m/s, so tight taxi turns are untouched). Once the yaw has stopped
			// (|r| < 0.7 deg/s, or 2 s after release) the FCS holds the straight line along the track until the pedal moves;
			// the touchdown capture holds the approach track the same way.
			bool sm = Plugin.GroundStockMatch;
			if (sm)
			{
				ped = gsPedRaw;
			}
			if (float.IsNaN(gsBetaPrev) || dt <= 0f)
			{
				gsBetaDot = 0f;
			}
			else
			{
				gsBetaDot += (Mathf.DeltaAngle(gsBetaPrev, gsSlip) / dt - gsBetaDot) * Mathf.Clamp01(dt / 0.08f);
			}
			gsBetaPrev = gsSlip;
			// 0.5.4 INDI: measured yaw acceleration and the nose tyre's lateral force now, through the same low-pass so
			// the increment is taken between synchronous values
			float aIndi = (dt > 0f) ? (dt / (Mathf.Max(0.005f, Plugin.GroundIndiTau) + dt)) : 0f;
			gsRdotF += (((float.IsNaN(gsRPrev) || dt <= 0f) ? 0f : ((r - gsRPrev) / dt)) - gsRdotF) * aIndi;
			gsRPrev = r;
			bool indiOk = GsNoseTyre(nose, noseWow, out float fyNow, out float alphaC, out float kTyre, out float muN, out float yawPerN);
			gsFyF += (fyNow - gsFyF) * aIndi;
			bool indi = Plugin.GroundIndi && indiOk && V >= 1.5f;
			if (indi)
			{
				gsYawPerNLast = yawPerN;
				gsLnT = 0f;
			}
			// 0.5.3 light nose: the nose tyre can't be used by INDI (no load, or off the ground) during a release stop or the hold
			// that INDI was flying - keep the wheel at INDI's last slip from the contact velocity
			// (at most GsLnMaxT s: the logged events last ~0.55 s; a deliberate nose-up - the take-off rotation - is left to 0.5.2)
			bool lnHold = Plugin.GroundIndiLightNoseHold && Plugin.GroundIndi && !indi && gsLnHold && V >= 1.5f && Mathf.Abs(ped) <= 0.05f && gsLnT < GsLnMaxT;
			if (lnHold)
			{
				gsLnT += dt;
			}
			if (Plugin.GroundIndiLightNoseHold && nose != null)
			{
				gsWheelDbg = Acc.LG_steerAngle(nose);
			}
			if (!indi && !lnHold)
			{
				gsLnHold = false;
			}
			gsIndiOn = false;
			gsSinceContact = 0f;
			if (!gsActive && !gsHold && gsAirT > 0.3f && V > 20f && Mathf.Abs(ped) <= 0.05f)
			{
				// touchdown: hold the approach track through the touchdown point straight away (drift / crab correction)
				gsHold = true;
				gsChiRef = ((gsSurfV.sqrMagnitude > 1f) ? chi : gsChiAir);
				gsLineY = 0f;
				gsLineP = pos;
				gsLinePG = ac.GlobalPosition();
				gsRCmd = r;
			}
			if (anyContact)
			{
				gsAirT = 0f;
			}
			if (Mathf.Abs(ped) > 0.05f || V < 1.5f)
			{
				rc = (sm ? r : (ped * rLim));
				gsHold = false;
				gsBrakeT = 0f;
			}
			else
			{
				rc = 0f;
				if (!gsHold)
				{
					gsBrakeT += dt;
					if (Mathf.Abs(r - gsSurfR) < 0.7f || gsBrakeT > 2f)
					{
						gsHold = true;
						gsChiRef = ((V > 3f) ? chi : psi);
						gsLineY = 0f;
						gsLineP = pos;
						gsLinePG = ac.GlobalPosition();
					}
				}
				if (gsHold)
				{
					gsChiRef += gsSurfR * dt;
					float cr = gsChiRef * (MathF.PI / 180f);
					Vector3 right = new Vector3(Mathf.Cos(cr), 0f, 0f - Mathf.Sin(cr));
					// cross-track from the surface-relative velocity (2.4.12-2.4.13: from the global position, which on a moving
					// deck is the ship's own travel)
					gsLineY += Vector3.Dot(vh, right) * dt;
					float chiDes = gsChiRef - Mathf.Clamp(Mathf.Atan2(gsLineY * Mathf.Clamp01(Plugin.GroundLineHold), Mathf.Max(4f * V, 25f)) * 57.29578f, -5f, 5f);
					float e = Mathf.DeltaAngle(psi, chiDes);
					// corrections are not limited by the pedal's comfort limit (GroundLateralAccel), only by GsHoldRate
					float rHold = Mathf.Min(GsHoldRate, Plugin.GroundYawRateMax);
					rc = Mathf.Clamp(GsKpsi * e, 0f - rHold, rHold) + gsSurfR;
				}
			}
			// yaw-acceleration limit on the command so the nose tyre is not asked for more than it can grip at the entry
			// stock-matched: the release braking asks for zero rate at once (no yaw-acceleration limit); the line hold and the
			// touchdown capture keep the limit
			float rCmdPrev = gsRCmd;
			gsRCmd = ((sm && !gsHold) ? r : Mathf.MoveTowards(gsRCmd, rc, GsYawAccel * dt));
			float rCmdDot = ((sm && !gsHold) || dt <= 0f) ? 0f : ((gsRCmd - rCmdPrev) / dt);
			// 0.5.4: the yaw-rate command for the nose-up roll on the mains (gear-down law, below). 2.4.12-2.4.13 handed it
			// gsRCmd, which in the stock phase is the measured yaw rate, so the rate error was zero and the pedal did nothing
			// (27 Oct 12:58 log, 381-386 s: 71 m/s on the mains, theta 3.8, full left / right pedal, yaw rate 1.9 throughout).
			// Pedal: the capped rate of 1.8.36-2.4.11 (GroundLateralAccel / V); released: stop the yaw; holding: the hold's.
			gsRCmdMains = ((sm && !gsHold) ? Mathf.MoveTowards(gsRCmdMains, ((Mathf.Abs(ped) > 0.05f) ? (ped * rLim) : 0f) + gsSurfR, GsYawAccel * dt) : gsRCmd);
			rc = gsRCmd;
			float err = rc - r;
			if (sm && (Mathf.Abs(ped) > 0.05f || V < 1.5f || !gsHold))
			{
				// stock steering (pedal in, and after release until the line is captured) with the drift limit on top
				float ys = GsStockYaw(ped, V);
				gsStockYaw = ys;
				gsStockPhase = true;
				gsInt = 0f;
				gsRudInt = 0f;
				gsNwsDeg = (noseWow ? (ys * steerLock) : 0f);
				if (indi && Mathf.Abs(ped) <= 0.05f)
				{
					// 0.5.4 release braking: the nose wheel stops the yaw by INDI (rudders and differential brake stay stock)
					// 0.5.3: with GroundIndiBrake the outside main's brake takes what lies beyond the nose tyre's grip
					float nuRel = (0f - Plugin.GroundIndiGain) * (r - gsSurfR);
					float nuNose = GsBrakeAlloc(nuRel, noseUsable: true, muN, yawPerN, V, dt, nose, ys, aIndi);
					gsNwsDeg = GsIndiWheel(nuRel, alphaC, kTyre, muN, yawPerN, steerLock, ped, V, nuNose);
				}
				else if (lnHold)
				{
					GsBrakeAlloc((0f - Plugin.GroundIndiGain) * (r - gsSurfR), noseUsable: false, 0f, 0f, V, dt, nose, ys, aIndi);
					gsNwsDeg = GsLightNoseWheel(nose, steerLock, ped, V);
				}
				else if (Mathf.Abs(ped) > 0.05f && V >= 1.5f)
				{
					GsPedalBrake(ped, nose, ys);
				}
				gsRud = ys;
				gsMainsContact = mainsContactNow(nose);
				gsActive = true;
				return;
			}
			gsStockPhase = false;
			if (indi)
			{
				// 0.5.4: INDI on the nose wheel - wanted yaw acceleration = GroundIndiGain x rate error + the command's slope
				gsInt = 0f;
				float nuHold = Plugin.GroundIndiGain * err + rCmdDot;
				float nuNose = GsBrakeAlloc(nuHold, noseUsable: true, muN, yawPerN, V, dt, nose, float.NaN, aIndi);
				gsNwsDeg = GsIndiWheel(nuHold, alphaC, kTyre, muN, yawPerN, steerLock, ped, V, nuNose);
			}
			else if (lnHold)
			{
				gsInt = 0f;
				GsBrakeAlloc(Plugin.GroundIndiGain * err + rCmdDot, noseUsable: false, 0f, 0f, V, dt, nose, float.NaN, aIndi);
				gsNwsDeg = GsLightNoseWheel(nose, steerLock, ped, V);
			}
			else if (noseWow && V >= 1.5f)
			{
				float ff = Mathf.Atan(rc * (MathF.PI / 180f) * gsLead / Mathf.Max(V, 1f)) * 57.29578f;
				float kp = Mathf.Clamp(GsKp0 / Mathf.Max(V, 3f), GsKpMin, GsKpMax);
				float lim = Mathf.Min(steerLock, GsLimK / Mathf.Max(V, 1f));
				// NO's tyre: lateral force = clamp(slip x (0.2 + 0.01 v), +-1) x muN, so it saturates at 1 / (0.2 + 0.01 v) deg of
				// slip. Feedback beyond that only drives the wheel into a skid and winds the integrator up.
				float sat = GsSatMargin / (0.2f + 0.01f * V);
				// stock-matched release braking: proportional only (bench: the integrator swung the yaw back 1-2.5 deg/s)
				gsInt = ((sm && !gsHold) ? 0f : Mathf.Clamp(gsInt + GsKi * kp * err * dt, 0f - sat, sat));
				float fb = Mathf.Clamp(kp * err + gsInt, 0f - sat, sat);
				float blend = Mathf.InverseLerp(1.5f, 3f, V);
				gsNwsDeg = Mathf.Lerp(ped * steerLock, Mathf.Clamp(ff + fb, 0f - lim, lim), blend);
			}
			else if (noseWow)
			{
				gsInt = 0f;
				gsNwsDeg = ped * steerLock;
			}
			else
			{
				gsInt = 0f;
				gsNwsDeg = 0f;
			}
			float rq = Mathf.InverseLerp(10f, 25f, V);
			gsRudInt = ((rq > 0f && !(sm && !gsHold)) ? Mathf.Clamp(gsRudInt + GsKri * err * dt * rq, -0.5f, 0.5f) : 0f);
			// below taxi speed the rudders have no authority: they show the pedal, as stock
			// stock-matched, pedal released: the rudders on the (released) pedal with the yaw-rate loop trimming on top
			gsRud = (sm ? Mathf.Clamp(ped + rq * (GsKr * err + gsRudInt), -1f, 1f) : Mathf.Lerp(ped, Mathf.Clamp(GsKr * err + gsRudInt, -1f, 1f), rq));
			gsMainsContact = mainsContactNow(nose);
			gsActive = true;
		}

		public bool gsMainsContact;

		// 0.5.4 INDI nose wheel state: filtered yaw acceleration (deg/s^2) and nose-tyre lateral force (N), the yaw-rate
		// command for the nose-up-on-the-mains phase, debug values
		private float gsRPrev = float.NaN;

		private float gsRdotF;

		private float gsFyF;

		public float gsRCmdMains;

		private bool gsIndiOn;

		private float gsNuDbg;

		private float gsWheelDbg;

		// any wheel touching the ground this tick (set by GroundSteer, which runs every tick)
		public bool gsAnyContact;

		// 0.5.4 INDI nose wheel. The game's nose tyre (LandingGear.FixedUpdate): lateral force = clamp(clamp(slip, +-10) x
		// response x (0.2 + 0.01 |wheelSpeed|), +-1) x mu N, perpendicular to the contact velocity, slip = angle from the wheel to
		// the contact velocity. Its yaw moment about the CG divided by the yaw inertia is the wheel's yaw effectiveness; all
		// of it is read from the game each tick (leg load, friction, the FCS's own CG and inertia), so loads, fuel and CG
		// move it with them. The virtual control is that force: wanted = force now (filtered) + (wanted yaw acceleration -
		// measured, filtered) / effectiveness, so the mains, the rudders, the brakes, gusts and model errors all show up in the
		// measured yaw acceleration rather than having to be modelled.
		private bool GsNoseTyre(LandingGear nose, bool noseWow, out float fy, out float alphaC, out float k, out float muN, out float yawPerN)
		{
			fy = 0f;
			alphaC = 0f;
			k = 0.2f;
			muN = 0f;
			yawPerN = 0f;
			Rigidbody body = ac.rb;
			if (nose == null || body == null || !noseWow)
			{
				return false;
			}
			Transform tr = ((Component)(object)ac).transform;
			Vector3 up = tr.up;
			Vector3 p = nose.transform.position;
			RaycastHit hit = Acc.LG_hit(nose);
			// the contact point when it is current (a floating-origin shift leaves last frame's hit point behind)
			if (hit.collider != null && (hit.point - p).sqrMagnitude < 9f)
			{
				p = hit.point;
			}
			Vector3 vc = body.GetPointVelocity(p) - SurfaceVelAt(hit, p, out float _);
			Vector3 vp = Vector3.ProjectOnPlane(vc, up);
			float sp = vp.magnitude;
			if (sp < 0.5f)
			{
				return false;
			}
			// + = the contact moving right of the nose
			alphaC = Vector3.SignedAngle(Vector3.ProjectOnPlane(tr.forward, up), vp, up);
			float delta = Acc.LG_steerAngle(nose);
			k = Mathf.Max(0.05f, Acc.LG_response(nose)) * (0.2f + 0.01f * Mathf.Abs(Acc.LG_wheelSpeed(nose)));
			muN = Mathf.Max(0f, Acc.LG_compForce(nose) + Acc.LG_dampForce(nose)) * Acc.LG_friction(nose);
			// + = force to the right of the contact velocity
			fy = Mathf.Clamp(Mathf.Clamp(delta - alphaC, -10f, 10f) * k, -1f, 1f) * muN;
			Vector3 right = Vector3.Cross(up, vp / sp);
			float iz = Ia[2, 2];
			yawPerN = ((iz > 1f) ? (Vector3.Dot(Vector3.Cross(p - cg, right), up) / iz * 57.29578f) : 0f);
			gsWheelDbg = delta;
			return muN > 1f && yawPerN > 1E-06f;
		}

		// nose-wheel angle for a wanted yaw acceleration nu (deg/s^2): the INDI force increment, turned into a slip angle through the
		// tyre's slope and kept inside GsSatMargin x its saturation slip (more slip only skids and scrubs), plus the contact
		// velocity's angle. Below 1.5-3 m/s it blends to the pedal, as before.
		// 0.5.3: nuNose = the nose tyre's share of the wanted yaw acceleration (absolute, deg/s^2) when the main brakes are in the
		// loop (GsBrakeAlloc); NaN = the nose alone, the 0.5.4 increment
		private float GsIndiWheel(float nu, float alphaC, float k, float muN, float yawPerN, float steerLock, float ped, float V, float nuNose = float.NaN)
		{
			float fDes = float.IsNaN(nuNose) ? (gsFyF + (nu - gsRdotF) / yawPerN) : (nuNose / yawPerN);
			float sMax = GsSatMargin / k;
			float slip = Mathf.Clamp(fDes / (muN * k), 0f - sMax, sMax);
			float d = Mathf.Clamp(alphaC + slip, 0f - steerLock, steerLock);
			gsNuDbg = nu;
			gsIndiOn = true;
			gsLnSlip = slip;
			gsLnHold = true;
			return Mathf.Lerp(ped * steerLock, d, Mathf.InverseLerp(1.5f, 3f, V));
		}

		// 0.5.3 light-nose hold: the nose tyre has no usable load (the leg extending - damping force negative - or the wheel off the
		// ground) in a stop or hold INDI was flying. The game slews the wheel toward its command even off the ground, so the 0.5.2
		// command (the stock law's angle, i.e. full opposite lock, then 0) ran the wheel away from the contact velocity and it met
		// the ground again ~25 deg out (18:55 log, 25.90-26.42 s: 23.7 -> 0.4 deg). Here it keeps INDI's last slip from the
		// contact velocity, so on re-contact the tyre picks up where it left off; INDI resumes from the filtered force and yaw
		// acceleration, which followed the lost load all along.
		private float GsLightNoseWheel(LandingGear nose, float steerLock, float ped, float V)
		{
			float d = gsNwsDeg;
			if (GsNoseAlpha(nose, out float alphaC))
			{
				d = Mathf.Clamp(alphaC + gsLnSlip, 0f - steerLock, steerLock);
			}
			gsLnDbg = true;
			return Mathf.Lerp(ped * steerLock, d, Mathf.InverseLerp(1.5f, 3f, V));
		}

		// the nose contact velocity's angle from the nose (deg, + = moving right of the nose), contact or not
		private bool GsNoseAlpha(LandingGear nose, out float alphaC)
		{
			alphaC = 0f;
			Rigidbody body = ac.rb;
			if (nose == null || body == null)
			{
				return false;
			}
			Transform tr = ((Component)(object)ac).transform;
			Vector3 up = tr.up;
			Vector3 p = nose.transform.position;
			RaycastHit hit = Acc.LG_hit(nose);
			if (hit.collider != null && (hit.point - p).sqrMagnitude < 9f)
			{
				p = hit.point;
			}
			Vector3 vp = Vector3.ProjectOnPlane(body.GetPointVelocity(p) - gsSurfV, up);
			if (vp.sqrMagnitude < 0.25f)
			{
				return false;
			}
			alphaC = Vector3.SignedAngle(Vector3.ProjectOnPlane(tr.forward, up), vp, up);
			return true;
		}

		// 0.5.3 main brakes as the INDI's second yaw effector. The game (LandingGear.FixedUpdate) brakes a leg with
		// brakeStrength = Lerp(brakeStrength, b, 5 dt), b = (braked ? brake : 0) + Clamp01(yaw x 0.3 x differentialBrakeFactor) for
		// 3-30 m/s ground speed, as a force b x mu N along -contact velocity; the leg's total tyre force is clamped to mu N. Each
		// braked main's yaw effectiveness per unit brake is read from the game every tick (its load, friction, contact velocity,
		// the CG and yaw inertia). Allocation: the wanted total effector yaw acceleration next tick (the nose's and the brakes'
		// filtered values now + the INDI increment) goes to the nose tyre up to its grip; only what lies beyond it goes to the
		// brakes on the side that yaws the right way (never below what the game already applies there: your brake + the stock
		// differential term); the nose then takes the rest, crediting each brake's lagged value next tick, so a brake that is
		// releasing is covered by the nose instead of being held on. Authority GroundIndiBrakeMax, faded from
		// GroundIndiBrakeFullSpeed to GroundIndiBrakeZeroSpeed. Returns the nose's share (deg/s^2), NaN when the brakes are not
		// in the loop (the nose then flies the 0.5.4 increment). stockYaw: the yaw the main legs read this tick in the stock phase
		// (StockBrakeYaw), NaN = their own pedal input.
		private float GsBrakeAlloc(float nu, bool noseUsable, float muN, float yawPerN, float V, float dt, LandingGear nose, float stockYaw, float aIndi)
		{
			if (!Plugin.GroundIndiBrake || !Plugin.GroundIndi)
			{
				return float.NaN;
			}
			float vF = Plugin.GroundIndiBrakeFullSpeed;
			float vZ = Plugin.GroundIndiBrakeZeroSpeed;
			float fade = (vZ > vF + 0.01f) ? Mathf.Clamp01((vZ - V) / (vZ - vF)) : ((V <= vZ) ? 1f : 0f);
			float bMax = Plugin.GroundIndiBrakeMax * fade;
			Rigidbody body = ac.rb;
			float iz = Ia[2, 2];
			if (bMax <= 0f || body == null || iz <= 1f)
			{
				return float.NaN;
			}
			int n = gears.Count;
			if (gsBrkF == null || gsBrkF.Length != n)
			{
				gsBrkF = new float[n];
				gsBrkE = new float[n];
				gsBrkBase = new float[n];
				gsBrkTgt = new float[n];
				gsBrkOnPrev = false;
			}
			Transform tr = ((Component)(object)ac).transform;
			Vector3 up = tr.up;
			Vector3 rightAx = tr.right;
			bool init = !gsBrkOnPrev;
			float mb = 0f;
			float ePos = 0f;
			float eNeg = 0f;
			int legs = 0;
			for (int i = 0; i < n; i++)
			{
				gsBrkE[i] = 0f;
				gsBrkTgt[i] = -1f;
				LandingGear g = gears[i];
				if (g == null || g == nose || !Acc.LG_braked(g) || !g.WeightOnWheel(0.002f))
				{
					continue;
				}
				Vector3 p = g.transform.position;
				RaycastHit hit = Acc.LG_hit(g);
				if (hit.collider != null && (hit.point - p).sqrMagnitude < 9f)
				{
					p = hit.point;
				}
				Vector3 vp = Vector3.ProjectOnPlane(body.GetPointVelocity(p) - SurfaceVelAt(hit, p, out float _), up);
				float sp = vp.magnitude;
				float muNm = Mathf.Max(0f, Acc.LG_compForce(g) + Acc.LG_dampForce(g)) * Acc.LG_friction(g);
				if (sp < 0.5f || muNm <= 1f)
				{
					continue;
				}
				float e = Vector3.Dot(Vector3.Cross(p - cg, -vp / sp), up) / iz * 57.29578f * muNm;
				ControlInputs ci = Acc.LG_inputs(g);
				float yawIn = float.IsNaN(stockYaw) ? ((ci != null) ? ci.yaw : 0f) : stockYaw;
				float dbf = Acc.LG_dbf(g);
				float gsp = Acc.LG_groundSpeed(g);
				float stock = (dbf != 0f && gsp > 3f && gsp < 30f) ? Mathf.Clamp01(yawIn * 0.3f * dbf) : 0f;
				gsBrkBase[i] = ((ci != null) ? ci.brake : 0f) + stock;
				float bs = Acc.LG_brakeStrength(g);
				gsBrkF[i] = init ? bs : (gsBrkF[i] + (bs - gsBrkF[i]) * aIndi);
				gsBrkE[i] = e;
				mb += gsBrkF[i] * e;
				if (e > 0f)
				{
					ePos += e;
				}
				else
				{
					eNeg += e;
				}
				legs++;
			}
			if (legs == 0)
			{
				return float.NaN;
			}
			// wanted total effector yaw acceleration next tick; the nose tyre's part of the filtered state uses its last
			// effectiveness, so a nose that has just gone light is counted as it fades (as the measured yaw acceleration sees it)
			float mTot = gsFyF * (noseUsable ? yawPerN : gsYawPerNLast) + mb + (nu - gsRdotF);
			float nCap = noseUsable ? (muN * yawPerN) : 0f;
			float mbT = mTot - Mathf.Clamp(mTot, 0f - nCap, nCap);
			float eSide = (mbT > 0f) ? ePos : eNeg;
			float cmd = (eSide != 0f && mbT * eSide > 0f) ? Mathf.Min(bMax, mbT / eSide) : 0f;
			float lerpK = Mathf.Clamp01(5f * dt);
			float mbP = 0f;
			for (int i = 0; i < n; i++)
			{
				float e = gsBrkE[i];
				if (e == 0f)
				{
					continue;
				}
				float tgt = Mathf.Max(gsBrkBase[i], (e * mbT > 0f) ? cmd : 0f);
				gsBrkTgt[i] = tgt;
				mbP += (gsBrkF[i] + (tgt - gsBrkF[i]) * lerpK) * e;
				float side = Vector3.Dot(gears[i].transform.position - cg, rightAx);
				if (side < 0f)
				{
					gsBrkDbgL = tgt;
				}
				else
				{
					gsBrkDbgR = tgt;
				}
			}
			gsBrkOn = true;
			return noseUsable ? (mTot - mbP) : float.NaN;
		}

		// 0.5.4 pedal brake (drift help): while the pedal is in, the main whose brake yaws the jet the pedal's way gets
		// |pedal| x GroundPedalBrake (never below what the game applies there: your brake + the stock differential term). A
		// braked tyre also gives up side grip (the game clamps each tyre's total force to mu N), so the mains break loose.
		private void GsPedalBrake(float ped, LandingGear nose, float stockYaw)
		{
			float k = Plugin.GroundPedalBrake;
			Rigidbody body = ac.rb;
			if (k <= 0f || body == null)
			{
				return;
			}
			int n = gears.Count;
			if (gsBrkF == null || gsBrkF.Length != n)
			{
				gsBrkF = new float[n];
				gsBrkE = new float[n];
				gsBrkBase = new float[n];
				gsBrkTgt = new float[n];
			}
			Transform tr = ((Component)(object)ac).transform;
			Vector3 up = tr.up;
			Vector3 rightAx = tr.right;
			bool any = false;
			for (int i = 0; i < n; i++)
			{
				gsBrkTgt[i] = -1f;
				LandingGear g = gears[i];
				if (g == null || g == nose || !Acc.LG_braked(g) || !g.WeightOnWheel(0.002f))
				{
					continue;
				}
				Vector3 p = g.transform.position;
				RaycastHit hit = Acc.LG_hit(g);
				if (hit.collider != null && (hit.point - p).sqrMagnitude < 9f)
				{
					p = hit.point;
				}
				Vector3 vp = Vector3.ProjectOnPlane(body.GetPointVelocity(p) - SurfaceVelAt(hit, p, out float _), up);
				float sp = vp.magnitude;
				if (sp < 0.5f)
				{
					continue;
				}
				// sign of the yaw this leg's brake gives (+ = nose right)
				float e = Vector3.Dot(Vector3.Cross(p - cg, -vp / sp), up);
				ControlInputs ci = Acc.LG_inputs(g);
				float dbf = Acc.LG_dbf(g);
				float gsp = Acc.LG_groundSpeed(g);
				float stock = (dbf != 0f && gsp > 3f && gsp < 30f) ? Mathf.Clamp01(stockYaw * 0.3f * dbf) : 0f;
				float bse = ((ci != null) ? ci.brake : 0f) + stock;
				float tgt = (e * ped > 0f) ? Mathf.Max(bse, Mathf.Abs(ped) * k) : bse;
				gsBrkTgt[i] = tgt;
				if (Vector3.Dot(g.transform.position - cg, rightAx) < 0f)
				{
					gsBrkDbgL = tgt;
				}
				else
				{
					gsBrkDbgR = tgt;
				}
				any = true;
			}
			gsBrkOn = any;
		}

		// the brake GsBrakeAlloc wants on this leg this tick (total, as the game sums it), for Hooks.GearPrefix
		public bool MainBrakeTarget(LandingGear g, out float target)
		{
			target = 0f;
			if (!gsActive || !gsBrkOn || gsBrkTgt == null)
			{
				return false;
			}
			int i = gears.IndexOf(g);
			if (i < 0 || i >= gsBrkTgt.Length || gsBrkTgt[i] < 0f)
			{
				return false;
			}
			target = gsBrkTgt[i];
			return true;
		}

		private bool gsBrkOn;

		private bool gsBrkOnPrev;

		private float[] gsBrkF;

		private float[] gsBrkE;

		private float[] gsBrkBase;

		private float[] gsBrkTgt;

		private float gsBrkDbgL = -1f;

		private float gsBrkDbgR = -1f;

		private bool gsLnHold;

		private float gsLnSlip;

		private bool gsLnDbg;

		private float gsYawPerNLast;

		private float gsLnT;

		internal static float GsLnMaxT = 1f;

		// 0.5.4: the velocity (and yaw rate, deg/s) of the surface the wheels stand on - a moving deck's rigidbody plus an
		// AnimatedPhysicsSurface, the same two the game's LandingGear takes off the contact velocity. Zero on fixed ground.
		public Vector3 gsSurfV;

		public float gsSurfR;

		private float gsSlip;

		private Collider gsSurfCol;

		private AnimatedPhysicsSurface gsSurfAps;

		private Vector3 SurfaceVelAt(RaycastHit hit, Vector3 p, out float yawRate)
		{
			yawRate = 0f;
			Collider col = hit.collider;
			if (col == null)
			{
				return Vector3.zero;
			}
			Vector3 vs = Vector3.zero;
			Rigidbody srb = col.attachedRigidbody;
			if (srb != null)
			{
				vs += srb.GetPointVelocity(p);
				yawRate = Vector3.Dot(srb.angularVelocity, Vector3.up) * 57.29578f;
			}
			if (col != gsSurfCol)
			{
				gsSurfCol = col;
				gsSurfAps = col.gameObject.GetComponent<AnimatedPhysicsSurface>();
			}
			if (gsSurfAps != null)
			{
				vs += gsSurfAps.GetVelocity();
			}
			return vs;
		}

		// the surface under the wheels this tick: a main leg in contact first, else the nose; kept through a bounce
		private void GsSurface(LandingGear nose)
		{
			LandingGear pick = null;
			foreach (LandingGear g in gears)
			{
				if (g != null && g.WeightOnWheel(0.002f) && (pick == null || pick == nose))
				{
					pick = g;
				}
			}
			if (pick == null)
			{
				return;
			}
			RaycastHit hit = Acc.LG_hit(pick);
			Vector3 p = pick.transform.position;
			if (hit.collider != null && (hit.point - p).sqrMagnitude < 16f)
			{
				p = hit.point;
			}
			gsSurfV = SurfaceVelAt(hit, p, out gsSurfR);
		}

		private bool mainsContactNow(LandingGear nose)
		{
			foreach (LandingGear g in gears)
			{
				if (g != null && g != nose && g.WeightOnWheel(0.002f))
				{
					return true;
				}
			}
			return false;
		}

		private static System.Reflection.FieldInfo fCfFbw, fCfMinSpeed, fCfMinAlt, fFbwEnabled, fFbwYawT, fFbwYawWv, fFbwCorner;

		private static bool fCfFailed;

		// the stock fly-by-wire's yaw (FlyByWire.Filter, read from the game's IL) for pedal ped, plus the drift limit
		private float GsStockYaw(float ped, float V)
		{
			float y = ped;
			try
			{
				if (stockCf != null && !fCfFailed)
				{
					if (fCfFbw == null)
					{
						Type t = typeof(ControlsFilter);
						fCfFbw = HarmonyLib.AccessTools.Field(t, "flyByWire");
						fCfMinSpeed = HarmonyLib.AccessTools.Field(t, "minSpeed");
						fCfMinAlt = HarmonyLib.AccessTools.Field(t, "minAlt");
						Type ft = fCfFbw.FieldType;
						fFbwEnabled = HarmonyLib.AccessTools.Field(ft, "Enabled");
						fFbwYawT = HarmonyLib.AccessTools.Field(ft, "yawTightness");
						fFbwYawWv = HarmonyLib.AccessTools.Field(ft, "yawWeathervaning");
						fFbwCorner = HarmonyLib.AccessTools.Field(ft, "cornerSpeed");
					}
					float spd = ac.speed;
					bool below = spd < (float)fCfMinSpeed.GetValue(stockCf) && ac.radarAlt < (float)fCfMinAlt.GetValue(stockCf);
					object fbw = fCfFbw.GetValue(stockCf);
					if (!below && fbw != null && (bool)fFbwEnabled.GetValue(fbw))
					{
						float T = (float)fFbwYawT.GetValue(fbw);
						float wv = (float)fFbwYawWv.GetValue(fbw);
						float corner = Mathf.Max(1f, (float)fFbwCorner.GetValue(fbw));
						float x = spd * spd * ac.airDensity / (corner * corner * 1.225f);
						float remap = 1f / Mathf.Max(x, 1f);
						Rigidbody body = ac.rb;
						float wy = (body != null) ? body.transform.InverseTransformDirection(body.angularVelocity).y : 0f;
						float k = T * remap * (wy - ped);
						if (wv > 0f && body != null)
						{
							Transform tr = ((Component)(object)ac).transform;
							k -= wv * Vector3.SignedAngle(Vector3.ProjectOnPlane(tr.forward, tr.up), Vector3.ProjectOnPlane(body.velocity, tr.up), tr.up);
						}
						y = Mathf.Clamp(0f - k, -1f, 1f);
					}
				}
			}
			catch (Exception e)
			{
				fCfFailed = true;
				Plugin.Log.LogWarning("ground steering: stock fly-by-wire parameters not readable, using the raw pedal (" + e.Message + ")");
				y = ped;
			}
			// drift limit: sideslip (led by its rate) past GroundDriftAllow is countered; beta > 0 = velocity right of the nose,
			// so a nose-right skid (beta < 0) is met with left yaw
			float fade = Mathf.InverseLerp(Plugin.GroundDriftFadeSpeed, Plugin.GroundDriftFadeSpeed + 10f, V);
			if (fade > 0f && Plugin.GroundDriftGain > 0f)
			{
				float bp = gsSlip + Plugin.GroundDriftLead * gsBetaDot;
				float allow = Mathf.Max(0f, Plugin.GroundDriftAllow);
				float over = bp - Mathf.Clamp(bp, 0f - allow, allow);
				y = Mathf.Clamp(y + fade * Plugin.GroundDriftGain * over, -1f, 1f);
			}
			return y;
		}

		// the main legs' differential brake reads the stock fly-by-wire's yaw in stock, not the pedal
		public bool StockBrakeYaw(out float yaw)
		{
			yaw = gsStockYaw;
			return gsActive && gsStockPhase;
		}

		public bool NoseSteerOverride(out float yaw)
		{
			LandingGear nose = NoseGear();
			float steerLock = ((nose != null) ? Mathf.Max(1f, Mathf.Abs(Acc.LG_steerLock(nose))) : 45f);
			yaw = Mathf.Clamp(gsNwsDeg / steerLock, -1f, 1f);
			return gsActive;
		}

		internal static float GsKpsi = 1f;

		internal static float GsHoldRate = 4f;

		internal static float GsYawAccel = 20f;

		internal static float GsSatMargin = 1.2f;

		internal static float GsKp0 = 15f;

		internal static float GsKpMin = 0.15f;

		internal static float GsKpMax = 3f;

		internal static float GsKi = 1f;

		internal static float GsLimK = 300f;

		internal static float GsKr = 0.1f;

		internal static float GsKphi = 3f;

		internal static float GsPhiFull = 10f;

		internal static float GsKri = 0.1f;

		private static float RudderAuthAt(float a)
		{
			float num = 1f - Mathf.Clamp01((Mathf.Abs(a) - Plugin.RudderFadeStart) / Mathf.Max(1f, Plugin.RudderFadeEnd - Plugin.RudderFadeStart));
			if (RudderRevBack)
			{
				num = Mathf.Max(num, Mathf.InverseLerp(120f, 140f, Mathf.Abs(a)));
			}
			return num;
		}

		private float AccRoll(Vector3 m)
		{
			return (IaInv[0, 0] * m.x + IaInv[0, 2] * m.z) * 57.29578f;
		}

		private float AccYaw(Vector3 m)
		{
			return (IaInv[2, 0] * m.x + IaInv[2, 2] * m.z) * 57.29578f;
		}

		private void AsymStep(float dt, float rho, Quaternion rootRot, ref EvalContext ctx, Vector3 wdF, Vector3 mF)
		{
			DecelStep(dt);
			DamageGateStep(dt);
			if (havePrev)
			{
				Vector3 m = MulI(wdF) + Vector3.Cross(omega, MulI(omega)) - mF;
				// 2.0.12: on the wheels the runway takes whatever roll moment the surfaces make, so the 'unexplained' roll
				// is the gear's reaction, not an asymmetry: the estimates bleed to zero there (10:37 log: 0 -> 25 deg/s^2
				// on the take-off roll, carried into the climb)
				float rollIn = wheelsTouch ? 0f : AccRoll(m);
				dMeasR = dMeasRF.Step(rollIn, 0.3f, dt);
				if (Mathf.Abs(omega.x * 57.29578f) < 20f)
				{
					dTrimR = dTrimRF.Step(wheelsTouch ? 0f : (AccRoll(m) - Lp * omega.x * IaInv[0, 0] * 57.29578f), 0.5f, dt);
				}
				dMeasY = dMeasYF.Step(AccYaw(m), 0.3f, dt);
				bool negSide = alpha < 0f;
				if (AsymPitchBias && asymValid && !wheelsTouch && tas > 30f && Mathf.Abs(alpha) < 30f && alpha != 0f && (alpha > 0f || AsymPitchBiasNeg))
				{
					float stabNow = 0f;
					float stabLo = 0f;
					float stabHi = 0f;
					int nStab = 0;
					int nStabAll = 0;
					foreach (SurfaceEffector sB in surfaces)
					{
						if (sB.kind == SurfaceKind.Stabilator)
						{
							nStabAll++;
						}
						if (sB.active && sB.kind == SurfaceKind.Stabilator)
						{
							nStab++;
							stabNow += sB.Mnow.y;
							if (sB.M != null && sB.N > 1)
							{
								float lo = float.MaxValue;
								float hi = float.MinValue;
								for (int iB = 0; iB < sB.N; iB++)
								{
									lo = Mathf.Min(lo, sB.M[iB].y);
									hi = Mathf.Max(hi, sB.M[iB].y);
								}
								stabLo += lo;
								stabHi += hi;
							}
						}
					}
					float bRaw = (m.y + mF.y - stabNow) * IaInv[1, 1] * 57.29578f - AsymCap.Interp(asymAlphas, asymAirP, asymAlphas.Length, alpha);
					if (!negSide)
					{
						if (AsymPitchBiasStab && nStab < nStabAll)
						{
							// the stabs' live nose-down end against the swept one at this AoA
							bRaw += stabLo * IaInv[1, 1] * 57.29578f - AsymCap.Interp(asymAlphas, asymStabLo, asymAlphas.Length, alpha);
						}
						float bF = asymPitchBiasF.Step(Mathf.Max(0f, bRaw), AsymPitchBiasTau, dt);
						asymPitchBias = Mathf.Max(bF, asymPitchBias - AsymPitchBiasDecay * dt);
						asymPitchBiasNeg = Mathf.Max(0f, asymPitchBiasNeg - AsymPitchBiasDecay * dt);
					}
					else
					{
						// Compass 0.1.8: the mirror at negative AoA - nose-down the model leaves out, and the stabs' live nose-up end
						float bN = 0f - bRaw;
						if (AsymPitchBiasStab && nStab < nStabAll)
						{
							bN += AsymCap.Interp(asymAlphas, asymStabHi, asymAlphas.Length, alpha) - stabHi * IaInv[1, 1] * 57.29578f;
						}
						float bFN = asymPitchBiasNegF.Step(Mathf.Max(0f, bN), AsymPitchBiasTau, dt);
						asymPitchBiasNeg = Mathf.Max(bFN, asymPitchBiasNeg - AsymPitchBiasDecay * dt);
						asymPitchBias = Mathf.Max(0f, asymPitchBias - AsymPitchBiasDecay * dt);
					}
				}
				else
				{
					asymPitchBias = Mathf.Max(0f, asymPitchBias - AsymPitchBiasDecay * dt);
					asymPitchBiasNeg = Mathf.Max(0f, asymPitchBiasNeg - AsymPitchBiasDecay * dt);
				}
			}
			if (!Plugin.AsymLimiter)
			{
				asymCapPos = (asymCapPosEff = 999f);
				asymCapNeg = (asymCapNegEff = -999f);
				asymCapFilt = 999f;
				asymCapFiltNeg = -999f;
				asymValid = false;
				asymFF = Vector3.zero;
				return;
			}
			cplClock += dt;
			AsymPrepSym(rootRot);
			FlapSymStep(dt);
			// 0.4.1: entering the engine-out / damaged-engine state, the cap from the last sweep was judged with the thrust
			// term in (and below AsymMinIAS there is no next sweep to replace it): it is replaced by the fixed cap at once
			// and a fresh sweep (thrust term out) is started, so a damage at high AoA and low speed gets the fixed cap and
			// not a stale 12 deg.
			bool fixedNow = (engOut || engAsym) && EngineOutFixedCap;
			if (fixedNow && !engFixedPrev && asymValid)
			{
				float capF = engOut ? Plugin.EngineOutAoACap : Plugin.EngineDamagedAoACap;
				asymCapPos = capF;
				asymCapNeg = 0f - capF;
				PushCapSample(asymCapPos, asymCapNeg, reset: true);
				asymK = -1;
				asymTimer = 1f;
			}
			// leaving it because the thrust match took the split away: the limit is lifted at once to the aero-only cap
			// (the sweep's arrays were made without the thrust term) instead of relaxing up over 1.5 s - the "hitch" at 29 deg
			// on the way to the full AoA (23 Oct 00:00 / 00:06 logs: capEff 30 -> 55 in 1.5 s after each engagement)
			if (!fixedNow && engFixedPrev && asymValid && matchFull)
			{
				asymCapPos = CplKneeOn ? CplKnee(1) : CplTop ? AsymCap.FindTop(asymAlphas, cplU, asymAlphas.Length, 1, Plugin.AoAPos, Mathf.Min(Plugin.AsymCapFloor, Plugin.AoAPos), AsymCapUMax, ref cplLowPos) : AsymCap.Find(asymAlphas, (cplActive && CplLateral) ? cplU : asymU, asymAlphas.Length, 1, Plugin.AoAPos, Mathf.Min(Plugin.AsymCapFloor, Plugin.AoAPos), AsymCapUMax);
				asymCapNeg = CplKneeOn ? CplKnee(-1) : CplTop ? AsymCap.FindTop(asymAlphas, cplU, asymAlphas.Length, -1, Plugin.AoANeg, 0f - Mathf.Min(Plugin.AsymCapFloorNeg, 0f - Plugin.AoANeg), AsymCapUMax, ref cplLowNeg) : AsymCap.Find(asymAlphas, (cplActive && CplLateral) ? cplU : asymU, asymAlphas.Length, -1, Plugin.AoANeg, 0f - Mathf.Min(Plugin.AsymCapFloorNeg, 0f - Plugin.AoANeg), AsymCapUMax);
				PushCapSample(asymCapPos, asymCapNeg, reset: true);
				asymCapPosEff = asymCapPos;
				asymCapNegEff = asymCapNeg;
				tKHoldT = 1f;
			}
			if (!fixedNow)
			{
				tKHoldT = Mathf.Max(0f, tKHoldT - dt);
			}
			engFixedPrev = fixedNow;
			long tPh0 = System.Diagnostics.Stopwatch.GetTimestamp();
			asymMNow = AsymMomentAt(alpha, rho, rootRot, ref ctx);
			long tPh1 = System.Diagnostics.Stopwatch.GetTimestamp();
			phAsymMom += tPh1 - tPh0;
			SlipProbeStep_(dt, rho, rootRot, ref ctx);
			phSlip += System.Diagnostics.Stopwatch.GetTimestamp() - tPh1;
			Vector3 vector = asymMF.Step(asymMNow, Mathf.Max(0.05f, Plugin.AsymFFWashout), dt);
			float num = Mathf.Max(0.01f, Plugin.AsymDeadband);
			float num2 = Mathf.Clamp01(Mathf.Max(Mathf.Abs(AccRoll(asymMNow)) / num, Mathf.Abs(AccYaw(asymMNow)) / (0.25f * num)) - 1f);
			float num3 = 1f - Mathf.Clamp01((Mathf.Abs(beta) - Plugin.AsymFFMaxBeta) / 12f);
			float num4 = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.AsymFFMaxAoA) / 15f);
			asymFFGate = num2 * num3 * num4;
			asymFF = (0f - Mathf.Clamp01(Plugin.AsymFeedForward)) * asymFFGate * new Vector3(asymMNow.x - vector.x, 0f, asymMNow.z - vector.z);
			asymFFr = AccRoll(asymFF);
			asymFFy = AccYaw(asymFF);
			asymTimer += dt;
			if (asymK < 0 && asymTimer > 0.2f && ias > Plugin.AsymMinIas)
			{
				asymTimer = 0f;
				long tPh2 = System.Diagnostics.Stopwatch.GetTimestamp();
				AsymSweepPrep(rootRot);
				phSweep += System.Diagnostics.Stopwatch.GetTimestamp() - tPh2;
				asymK = 0;
			}
			if (asymK >= 0)
			{
				int num5 = Mathf.Min(asymAlphas.Length, asymK + ((cplActive && cplLoaded) ? Mathf.Min(AsymPointsPerStep, CplPointsPerStep) : AsymPointsPerStep));
				long tPh3 = System.Diagnostics.Stopwatch.GetTimestamp();
				AsymSweepPoints(asymK, num5, rho, rootRot, ref ctx);
				phSweep += System.Diagnostics.Stopwatch.GetTimestamp() - tPh3;
				asymK = ((num5 < asymAlphas.Length) ? num5 : (-1));
			}
			if (asymK < 0 && asymCycleDone)
			{
				asymCycleDone = false;
				if (cplActive)
				{
					CplDemand();
					CplFill();
					if (CplDbgT >= 0f && cplClock >= CplDbgT && cplLoaded)
					{
						// bench diagnostic: the whole curve of one sweep
						CplDbgT = -1f;
						System.Text.StringBuilder sb = new System.Text.StringBuilder();
						sb.AppendFormat(CultureInfo.InvariantCulture, "CPLDBG t {0:F2} ias {1:F1} alpha {2:F1} dy {3:F3} kq {4:F2}\n", cplClock, ias, alpha, asymDy, qPredK);
						for (int iD = 0; iD < asymAlphas.Length; iD++)
						{
							sb.AppendFormat(CultureInfo.InvariantCulture, "  a {0,4:F0}  cplU {1,6:F2}  rp {2,7:F1}  rn {3,7:F1}  heldDn {4,7:F1} stabDn {5,7:F1} | one-axis U {6,6:F2}  Ar {7,7:F1} Ay {8,7:F1} Cr {9,6:F1} Cy {10,6:F1}\n", asymAlphas[iD], cplU[iD], cplRp[iD], cplRn[iD], cplDn[iD], asymNetDn[iD], asymU[iD], asymAr[iD], asymAy[iD], asymCr[iD], asymCy[iD]);
						}
						Console.Write(sb.ToString());
					}
				}
				asymCapPos = CplKneeOn ? CplKnee(1) : CplTop ? AsymCap.FindTop(asymAlphas, cplU, asymAlphas.Length, 1, Plugin.AoAPos, Mathf.Min(Plugin.AsymCapFloor, Plugin.AoAPos), AsymCapUMax, ref cplLowPos) : AsymCap.Find(asymAlphas, (cplActive && CplLateral) ? cplU : asymU, asymAlphas.Length, 1, Plugin.AoAPos, Mathf.Min(Plugin.AsymCapFloor, Plugin.AoAPos), AsymCapUMax);
				asymCapNeg = CplKneeOn ? CplKnee(-1) : CplTop ? AsymCap.FindTop(asymAlphas, cplU, asymAlphas.Length, -1, Plugin.AoANeg, 0f - Mathf.Min(Plugin.AsymCapFloorNeg, 0f - Plugin.AoANeg), AsymCapUMax, ref cplLowNeg) : AsymCap.Find(asymAlphas, (cplActive && CplLateral) ? cplU : asymU, asymAlphas.Length, -1, Plugin.AoANeg, 0f - Mathf.Min(Plugin.AsymCapFloorNeg, 0f - Plugin.AoANeg), AsymCapUMax);
				if (engOut && EngineOutFixedCap)
				{
					asymCapPos = Mathf.Min(asymCapPos, Plugin.EngineOutAoACap);
					asymCapNeg = Mathf.Max(asymCapNeg, 0f - Plugin.EngineOutAoACap);
				}
				else if (engAsym && EngineOutFixedCap)
				{
					// Ifrit 0.4.1: a weak (damaged, running) engine gets its own fixed cap, not the dynamic thrust cap
					asymCapPos = Mathf.Min(asymCapPos, Plugin.EngineDamagedAoACap);
					asymCapNeg = Mathf.Max(asymCapNeg, 0f - Plugin.EngineDamagedAoACap);
				}
				asymCapPitch = 999f;
				float capGate = Mathf.Max(Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan), (AsymDamageGate && dmgGateHold > 0f) ? 1f : 0f);
				if (Plugin.AsymPitchMargin > 0f && capGate > 0.5f && (asymCapPos < Plugin.AoAPos - 0.5f || (cplActive && cplLoaded)) && Ia[1, 1] > 1f)
				{
					float tvDn = 0f;
					if (tvc.active && tvc.M != null)
					{
						float tvLo = float.MaxValue;
						for (int tk = 0; tk < tvc.N; tk++)
						{
							tvLo = Mathf.Min(tvLo, tvc.M[tk].y);
						}
						if (tvLo < float.MaxValue)
						{
							tvDn = tvLo * IaInv[1, 1] * 57.29578f;
						}
					}
					float margP = AsymPitchMarginEff(out float kqP);
					asymPitchMargDbg = margP;
					float prevA = 0f;
					float prevM = float.NaN;
					for (int ia = 0; ia < asymAlphas.Length; ia++)
					{
						float aA = asymAlphas[ia];
						if (aA < 0f)
						{
							continue;
						}
						float mDn = kqP * (0f - ((cplActive ? cplDn[ia] : asymNetDn[ia]) + ((cplActive && cplDnTv[ia]) ? 0f : tvDn))) - ((AsymPitchBias && (tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC)) ? asymPitchBias : 0f);
						if (mDn < margP)
						{
							float aP = (float.IsNaN(prevM) || prevM <= mDn) ? aA : Mathf.Lerp(prevA, aA, (prevM - margP) / (prevM - mDn));
							asymCapPitch = Mathf.Max((AsymPitchBias && (tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC)) ? AsymPitchCapFloorNoTvc : AsymPitchCapFloor, aP - AsymPitchCapLead);
							break;
						}
						prevA = aA;
						prevM = mDn;
					}
					asymCapPos = Mathf.Min(asymCapPos, asymCapPitch);
				}
				asymCapPitchNeg = -999f;
				if (AsymPitchBiasNeg && Plugin.AsymPitchMargin > 0f && capGate > 0.5f && asymCapNeg > Plugin.AoANeg + 0.5f && Ia[1, 1] > 1f && (tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC))
				{
					float margN = AsymPitchMarginEff(out float kqN);
					float prevAN = 0f;
					float prevMN = float.NaN;
					for (int ia = asymAlphas.Length - 1; ia >= 0; ia--)
					{
						float aA = asymAlphas[ia];
						if (aA > 0f)
						{
							continue;
						}
						float mUp = kqN * (cplActive ? cplUp[ia] : asymNetUp[ia]) - asymPitchBiasNeg;
						if (mUp < margN)
						{
							float aP = (float.IsNaN(prevMN) || prevMN <= mUp) ? aA : Mathf.Lerp(prevAN, aA, (prevMN - margN) / (prevMN - mUp));
							asymCapPitchNeg = Mathf.Min(0f - AsymPitchCapFloorNegNoTvc, aP + AsymPitchCapLead);
							break;
						}
						prevAN = aA;
						prevMN = mUp;
					}
					asymCapNeg = Mathf.Max(asymCapNeg, asymCapPitchNeg);
				}
				AuthCapFromSweep();
				PushCapSample(asymCapPos, asymCapNeg, !asymValid);
				if (!asymValid)
				{
					asymCapPosEff = asymCapPos;
					asymCapNegEff = asymCapNeg;
					asymValid = true;
				}
				if (!asymFirstLogged)
				{
					asymFirstLogged = true;
					Plugin.Log.LogInfo($"Su-30 FCS: asymmetric-load check at {ias:F0} m/s IAS - lateral CG offset {asymDy * 100f:F1} cm, lateral usage at 30/45/60 deg AoA " + $"{AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 30f):F2}/{AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 45f):F2}/{AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 60f):F2} " + $"(cap engages above 1.00); roll/yaw authority there {AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 30f):F0}/{AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 30f):F0}, " + $"{AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 45f):F0}/{AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 45f):F0}, " + $"{AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 60f):F0}/{AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 60f):F0} deg/s^2; " + $"load accel at 45 deg {AsymCap.Interp(asymAlphas, asymAr, asymAlphas.Length, 45f):F1}/{AsymCap.Interp(asymAlphas, asymAy, asymAlphas.Length, 45f):F1} deg/s^2 (deadband {Plugin.AsymDeadband:F1}); AoA cap {asymCapPos:F1}/{asymCapNeg:F1} deg.");
				}
			}
			if (asymValid)
			{
				CapFilterStep(dt);
				float upRate = Plugin.AsymRelax * (1f + Mathf.Max(0f, Plugin.AsymRelaxBoost) * (1f - Mathf.Clamp01(AsymCap.Interp(asymAlphas, (cplActive && CplLateral) ? cplU : asymU, asymAlphas.Length, asymCapPosEff))));
				float tightR = Plugin.AsymTighten * ((engOut || engAsym) ? Mathf.Max(1f, EngOutTightenK) : 1f);
				// Compass 0.1.9 (AsymCapGapTighten): a cap that has to come down a long way (a part shot off) comes down faster, up to AsymCapGapMax x
				// the tighten rate, in proportion to the gap over AsymCapGapRef (3) deg. At 10 deg/s a 65 -> 12 deg drop took 5 s; 5 Nov 21:09
				// F-22E log, right stab shot off at 260.6 s with the fins gone: full aft 2 s later reached 25 deg AoA under a cap still at 45,
				// and the lone stab's load rolled the jet to 78 deg of bank against full opposite roll.
				float gapP = asymCapPosEff - asymCapFilt;
				float gapN = asymCapFiltNeg - asymCapNegEff;
				float kGP = AsymCapGapTighten ? Mathf.Clamp(gapP / Mathf.Max(1f, AsymCapGapRef), 1f, AsymCapGapMax) : 1f;
				float kGN = AsymCapGapTighten ? Mathf.Clamp(gapN / Mathf.Max(1f, AsymCapGapRef), 1f, AsymCapGapMax) : 1f;
				asymCapPosEff = AsymCap.Slew(asymCapPosEff, asymCapFilt, 1, tightR * kGP, upRate, dt);
				asymCapNegEff = AsymCap.Slew(asymCapNegEff, asymCapFiltNeg, -1, tightR * kGN, upRate, dt);
				asymUNow = AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, alpha);
				asymArNow = AsymCap.Interp(asymAlphas, asymAr, asymAlphas.Length, alpha);
				asymAyNow = AsymCap.Interp(asymAlphas, asymAy, asymAlphas.Length, alpha);
				asymCrNow = AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, alpha);
				asymCyNow = AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, alpha);
				if (cplActive)
				{
					cplUNow = AsymCap.Interp(asymAlphas, cplU, asymAlphas.Length, alpha);
					cplRpNow = AsymCap.Interp(asymAlphas, cplRp, asymAlphas.Length, alpha);
					cplRnNow = AsymCap.Interp(asymAlphas, cplRn, asymAlphas.Length, alpha);
					cplDnNow = AsymCap.Interp(asymAlphas, cplDn, asymAlphas.Length, alpha);
				}
				FuelTrimStep(dt, rootRot);
				asymLogTimer += dt;
				bool flag = asymCapPos < Plugin.AoAPos - 0.1f || asymCapNeg > Plugin.AoANeg + 0.1f;
				bool flag2 = asymLoggedCap < Plugin.AoAPos - 0.1f || asymLoggedCapN > Plugin.AoANeg + 0.1f;
				if (asymLogTimer > 2f && (flag != flag2 || (flag && (Mathf.Abs(asymCapPos - asymLoggedCap) > 3f || Mathf.Abs(asymCapNeg - asymLoggedCapN) > 3f))))
				{
					asymLogTimer = 0f;
					asymLoggedCap = asymCapPos;
					asymLoggedCapN = asymCapNeg;
					Plugin.Log.LogInfo(flag ? $"Su-30 FCS: asymmetric load - lateral CG offset {asymDy * 100f:F1} cm, AoA cap {asymCapPos:F1}/{asymCapNeg:F1} deg at {ias:F0} m/s IAS ({((cplActive && cplLoaded) ? "coupled" : "one-axis")} lateral demand at 30 deg: {AsymCap.Interp(asymAlphas, (cplActive && CplLateral) ? cplU : asymU, asymAlphas.Length, 30f):F2}, at 60: {AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 60f):F2}; load accel at 45 deg {AsymCap.Interp(asymAlphas, asymAr, asymAlphas.Length, 45f):F1} roll / {AsymCap.Interp(asymAlphas, asymAy, asymAlphas.Length, 45f):F1} yaw, authority there {AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 45f):F0} / {AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 45f):F0} deg/s^2)" : $"Su-30 FCS: asymmetric-load AoA cap released (lateral CG offset {asymDy * 100f:F1} cm)");
				}
			}
		}

		// ---- 1.8.45: IAS-decay prediction and asymmetric-cap filter ----
		public float iasDotF;

		public float qPredK = 1f;

		public float latPredK = 1f;

		public float asymCapFilt = 999f;

		public float asymCapFiltNeg = -999f;

		private float iasPrevD = -1f;

		private readonly float[] capHist = new float[3];

		private readonly float[] capHistN = new float[3];

		private int capHistI;

		private float capMed = 999f;

		private float capMedNeg = -999f;

		internal static float BrakeRefIas => Plugin.YawBrakeRefKmh / 3.6f;   // 1.9.2: 270 km/h (was 280): the reference manoeuvre's IAS

		// 1.9.2: below YawMarginIasHi the yaw braking margin grows smoothly (smoothstep) to YawMarginLow x at YawMarginIasLo
		internal static float YawMarginIasHi => Plugin.YawMarginStartKmh;

		internal static float YawMarginIasLo => Plugin.YawMarginFullKmh;

		internal static float YawMarginLow => Plugin.YawMarginLowIas;

		internal static float BrakeSoftMin = 0.12f;      // soft-min width, fraction of the limits' mean

		public float yawMarginDbg = 1f;

		public float yawMidDbg = 1f;

		internal static float RollStopTau = 0.5f;

		internal static float ProtOnsetGain = 8f;

		internal static float ProtFlagBand = 1.5f;

		private float rollStopAF = -1f;

		// C1-smooth minimum: equals min(a, b) once they differ by more than k, at most k/4 below it where they cross
		private static float SoftMin(float a, float b, float k)
		{
			if (k <= 1e-4f)
			{
				return Mathf.Min(a, b);
			}
			float h = Mathf.Clamp01(0.5f + 0.5f * (b - a) / k);
			return Mathf.Lerp(b, a, h) - k * h * (1f - h);
		}

		internal static float DecelTau = 0.25f;          // s, IAS-rate filter

		internal static float DecelLook => Plugin.DecelLookahead;   // s, how far ahead the decay is projected

		internal static float Decel0 => Plugin.DecelStart;          // m/s^2: below this, no anticipation

		internal static float Decel1 => Plugin.DecelFull;           // m/s^2: full weight from here (quadratic in between)

		internal static float DecelKMin = 0.35f;

		internal static float AsymResMaxFrac = 0.8f;

		// 1.8.45: demand at the cap (was 1.0). In-game, with the light mixed load (6.5 cm lateral CG offset, 60-110 m/s)
		// the demand rises ~0.025 per degree above the cap, so 1.08 is ~3 deg more AoA there; heavier loads rise faster.
		internal static float AsymCapUMax => Plugin.AsymCapDemand;

		internal static float AsymCapTauUp => Plugin.AsymCapSmoothing;

		internal static float AsymCapTauDown = 0.3f;

		internal static bool LatBrakeFloor => Plugin.HighAoABrakeFloor;

		internal static float LatBrakeA0 = 25f;

		internal static float LatBrakeA1 = 35f;

		internal static float LatBrakeMinRate = 8f;

		private void DecelStep(float dt)
		{
			if (dt <= 0f)
			{
				return;
			}
			if (iasPrevD < 0f || !Engaged)
			{
				iasPrevD = ias;
				iasDotF = 0f;
				qPredK = 1f;
				return;
			}
			float d = (ias - iasPrevD) / dt;
			iasPrevD = ias;
			iasDotF += (Mathf.Clamp(d, -80f, 80f) - iasDotF) * Mathf.Clamp01(dt / Mathf.Max(0.02f, DecelTau));
			float dec = Mathf.Max(0f, 0f - iasDotF);
			float w = Mathf.Clamp01((dec - Decel0) / Mathf.Max(0.1f, Decel1 - Decel0));
			float iasP = Mathf.Max(ias - dec * w * w * DecelLook, 0.1f * ias);
			float r = (ias > 1f) ? (iasP / ias) : 1f;
			qPredK = Mathf.Clamp(r * r, DecelKMin, 1f);
		}

		private void PushCapSample(float pos, float neg, bool reset)
		{
			if (reset)
			{
				for (int i = 0; i < 3; i++)
				{
					capHist[i] = pos;
					capHistN[i] = neg;
				}
				capMed = (asymCapFilt = pos);
				capMedNeg = (asymCapFiltNeg = neg);
				return;
			}
			capHist[capHistI] = pos;
			capHistN[capHistI] = neg;
			capHistI = (capHistI + 1) % 3;
			capMed = Med3(capHist[0], capHist[1], capHist[2]);
			capMedNeg = Med3(capHistN[0], capHistN[1], capHistN[2]);
		}

		private static float Med3(float a, float b, float c)
		{
			return Mathf.Max(Mathf.Min(a, b), Mathf.Min(Mathf.Max(a, b), c));
		}

		internal static bool AsymCapGapTighten = true;

		internal static float AsymCapGapRef = 3f;

		internal static float AsymCapGapMax = 4f;

		private void CapFilterStep(float dt)
		{
			// tightening passes quickly, relaxing slowly: noise can't lift the cap, a real drop is followed in ~0.3 s
			float kU = Mathf.Clamp01(dt / Mathf.Max(0.05f, AsymCapTauUp));
			float kD = Mathf.Clamp01(dt / Mathf.Max(0.05f, AsymCapTauDown));
			asymCapFilt += (capMed - asymCapFilt) * ((capMed < asymCapFilt) ? kD : kU);
			asymCapFiltNeg += (capMedNeg - asymCapFiltNeg) * ((capMedNeg > asymCapFiltNeg) ? kD : kU);
		}

		private void AsymPrepSym(Quaternion rootRot)
		{
			Vector3 rhs = rootRot * Vector3.right;
			Vector3 zero = Vector3.zero;
			int num = 0;
			foreach (SurfaceEffector surface in surfaces)
			{
				if (surface.partner != null && surface.active && surface.partner.active && surface.partner is SurfaceEffector surfaceEffector && !(surface.part.rb == null) && !(surfaceEffector.part.rb == null))
				{
					zero += 0.5f * (surface.part.rb.worldCenterOfMass + surfaceEffector.part.rb.worldCenterOfMass);
					num++;
				}
			}
			// Revoker 0.2.1: with no intact pair left the reference is the airframe's centreline (the root transform; the
			// Revoker's parts are mirrored about it); 0.2.0 reported 0
			asymDy = ((num > 0) ? Vector3.Dot(cg - zero / num, rhs) : Vector3.Dot(cg - ((Component)(object)ac).transform.position, rhs));
			float[] array = asymDsym;
			if (array == null || array.Length != surfaces.Count)
			{
				array = (asymDsym = new float[surfaces.Count]);
			}
			for (int i = 0; i < surfaces.Count; i++)
			{
				array[i] = 0f;
			}
			for (int j = 0; j < surfaces.Count; j++)
			{
				SurfaceEffector surfaceEffector2 = surfaces[j];
				if (surfaceEffector2.partner != null)
				{
					int num2 = ((surfaceEffector2.partner is SurfaceEffector item) ? surfaces.IndexOf(item) : (-1));
					if (num2 >= 0)
					{
						float num3 = surfaceEffector2.pairSign * surfaceEffector2.now;
						float num4 = surfaceEffector2.partner.pairSign * surfaceEffector2.partner.now;
						float num5 = 0.5f * (num3 + num4);
						array[j] = surfaceEffector2.pairSign * (num3 - num5);
						array[num2] = surfaceEffector2.partner.pairSign * (num4 - num5);
					}
				}
			}

			if (surfaceParts.Count != 0)
			{
				return;
			}
			foreach (SurfaceEffector surface2 in surfaces)
			{
				if (surface2.part != null)
				{
					surfaceParts.Add(surface2.part);
				}
			}
			// Revoker 0.2.0: the inboard elevons are modelled by their effector too
			foreach (HingeFlapEffector hP in hinges)
			{
				if (hP.part != null)
				{
					surfaceParts.Add(hP.part);
				}
			}
		}

		// Revoker 0.2.0: a part's lift-normal rotation for the airframe models: an inboard elevon's camber host without the
		// camber the elevon's effector already models
		private Quaternion AirframeRot(AeroPart ap, Transform ln)
		{
			for (int i = 0; i < hinges.Count; i++)
			{
				if (hinges[i].host == ap && hinges[i].hostLN == ln)
				{
					return hinges[i].HostRot0;
				}
			}
			return ln.rotation;
		}

		// Revoker 0.2.4: gravity's sideslip feed-forward fade (TAS, m/s) and cap (deg/s); F-22E / Ifrit: 40, 20, none
		// Revoker 0.2.5: 0.6 (see the authority sweep)
		internal static float ElevonAuthShare = 0.6f;

		// 0.2.4 defaults: down to 18 -> 28 m/s TAS, capped at 30 deg/s (bench corkscrews: sideslip 7.4-9.0 -> 4.0-5.7 deg)
		internal static float GravFFTas0 = 18f;

		internal static float GravFFSpan = 10f;

		internal static float GravFFMax = 30f;

		internal static float LimTau = 0.4f;

		private float pLimF = -1f;

		private float rLimF;

		internal static float CorkYawOmega = 0f;

		internal static float CorkYawAccel = 120f;

		internal static float CorkShapeA0 = 30f;

		private float corkR;

		private float corkRd;

		public float corkWDbg;

		internal static float CorkAoA0 = 35f;

		internal static float CorkTolClose = 8f;

		public float corkTolNow;

		private float RudderHoldGate()
		{
			float use = -1f;
			foreach (SurfaceEffector rS in surfaces)
			{
				if (rS.kind == SurfaceKind.Rudder && rS.active)
				{
					float lim = (rS.cmdOut >= 0f) ? Mathf.Max(1f, rS.hi) : Mathf.Max(1f, 0f - rS.lo);
					use = Mathf.Max(use, Mathf.Abs(rS.cmdOut) / lim);
				}
			}
			return (use < 0f) ? 0f : (1f - Mathf.Clamp01((use - 0.55f) / 0.3f));
		}

		// 2.5.15: the whole airframe's aero moment (aero frame: x roll, z yaw) at a hypothetical AoA + sideslip, the surfaces at
		// their present deflection. Same per-part law as AsymMomentAt (the game's AeroJob_Math per AeroPart, the FCS's foil tables).
		private Vector3 SlipMomentAt(float aDeg, float bDeg, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			return SlipMomentAt(aDeg, bDeg, rho, rootRot, ref ctx, 0f);
		}

		// Revoker 0.4.0: the same with a body yaw rate rYaw (rad/s, aero z): every part and surface sees its own velocity, v + omega x r,
		// so the difference to rYaw = 0 is the airframe's yaw damping at this flow - what a nose-high fuselage does to a rotation by
		// itself (the rate cap's reversal model had only the effectors braking it). rYaw = 0 is the 2.5.15 function exactly.
		private Vector3 SlipMomentAt(float aDeg, float bDeg, float rho, Quaternion rootRot, ref EvalContext ctx, float rYaw)
		{
			float num = Mathf.Max(tas, 1f);
			float fa = aDeg * (MathF.PI / 180f);
			float fb = bDeg * (MathF.PI / 180f);
			Vector3 vector = rootRot * new Vector3(Mathf.Sin(fb) * num, (0f - Mathf.Sin(fa)) * Mathf.Cos(fb) * num, Mathf.Cos(fa) * Mathf.Cos(fb) * num);
			Vector3 omegaW = (rYaw != 0f) ? (rootRot * new Vector3(0f, rYaw, 0f)) : Vector3.zero;
			Vector3 zero = Vector3.zero;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached())
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				if (!(transform == null))
				{
					Quaternion rotation = transform.rotation;
					Vector3 lhs = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					Vector3 vP = (rYaw != 0f) ? (vector + Vector3.Cross(omegaW, lhs)) : vector;
					float vPm = Mathf.Max(vP.magnitude, 1f);
					Vector3 rhs = -(vP / vPm) * (0.5f * (0.5f * rho * vPm * vPm) * Acc.AP_dragArea(aeroPart));
					if (!surfaceParts.Contains(aeroPart) && foils.TryGetValue(aeroPart, out var value) && Acc.AP_wingArea(aeroPart) > 0f)
					{
						rhs += PartForce(vP, rotation, rotation * Vector3.right, value, 0.5f * rho * Acc.AP_wingArea(aeroPart) * Acc.AP_wingEff(aeroPart));
					}
					zero += Vector3.Cross(lhs, rhs);
				}
			}
			Vector3 vector3 = Quaternion.Inverse(rootRot) * zero;
			Vector3 result = new Vector3(0f - vector3.z, 0f - vector3.x, vector3.y);
			for (int i = 0; i < surfaces.Count; i++)
			{
				if (surfaces[i].active)
				{
					Vector3 vS = vector;
					if (rYaw != 0f && surfaces[i].part != null && surfaces[i].part.rb != null)
					{
						vS = vector + Vector3.Cross(omegaW, surfaces[i].part.rb.worldCenterOfMass - cg);
					}
					result += surfaces[i].EvalFlow(surfaces[i].now, vS, rho, ref ctx);
				}
			}
			return result;
		}

		// 2.31.2: the whole airframe's aero moment (aero frame: x roll, z yaw) at a hypothetical CG air velocity vAir and body rotation
		// w (world, rad/s), every aero part evaluated as the game's AeroJob_Math does it (the bench plant's Forces): the flow at the part
		// vAir + w x arm, lift and drag from the part's table, the parasitic drag, the transonic factor, the stabs' downwash on the root
		// body's flow, the per-part force clamp (30 x mass x |v|), and the flap-camber increment as CamberForce applies it. Control
		// surfaces sit where their transforms are now; no thrust. SlipMomentAt (2.5.15) differs: uniform flow, surfaces through their
		// effector model, no clamp - it over-reads the sideslip stiffness 2-15x above 40 deg AoA (claude/Passive_lateral_stiffness_finding.md).
		private Vector3 AirframeMomentAt(Vector3 vAir, Vector3 w, float rho, Quaternion rootRot)
		{
			Vector3 vRootRef = vAir + Vector3.Cross(w, ac.rb.worldCenterOfMass - cg);
			Vector3 m = Vector3.zero;
			foreach (AeroPart ap in aeroParts)
			{
				if (ap == null || ap.rb == null || ap.IsDetached())
				{
					continue;
				}
				Transform lt = Acc.AP_liftNormal(ap);
				if (lt == null)
				{
					continue;
				}
				Vector3 com = ap.rb.worldCenterOfMass;
				Vector3 v = vAir + Vector3.Cross(w, com - cg);
				float v2 = v.sqrMagnitude;
				if (v2 < 1f)
				{
					continue;
				}
				float vmag = Mathf.Sqrt(v2);
				Quaternion lr = lt.rotation;
				Vector3 vf = Downwash.IsStabName(ap.name) ? Downwash.Apply(v, rootRot, vRootRef) : v;
				Vector3 v6 = Quaternion.Inverse(lr) * vf;
				float a = Mathf.Atan2(v6.y, v6.z);
				float cl, cd;
				if (foils.TryGetValue(ap, out var foil) && foil != null)
				{
					foil.Get(a, out cl, out cd);
				}
				else
				{
					cl = 1.8f * Mathf.Sin(5f * a);
					cd = 1.5f * (1f - Mathf.Cos(2f * a)) + 0.02f;
				}
				float area = Acc.AP_wingArea(ap), eff = Acc.AP_wingEff(ap), dA = Acc.AP_dragArea(ap);
				float n8 = (dA + area * 0.1f) * (1f - eff);
				float n9 = 0.5f * rho * v2 * 0.5f * (dA + n8);
				float n10 = cd * rho * v2 * 0.5f * area * eff;
				float tr = DragReliefMath.Transonic(vmag / DragReliefMath.SpeedOfSound(com.y));
				Vector3 vfHat = vf.normalized, vHat = v / vmag;
				Vector3 f = (-vfHat * n10 - vHat * n9) * tr;
				// the buffed profile's drag relief (DragRelief hook): part of the foil drag above its minimum, by part AoA and Mach
				if (Plugin.DragReliefAoA > 0f || Plugin.DragReliefAoAHigh > 0f || Plugin.DragReliefTransonic > 0f)
				{
					if (!psCdMin.TryGetValue(ap, out float cdMin))
					{
						cdMin = float.MaxValue;
						if (foil != null)
						{
							for (float aa = -0.35f; aa <= 0.35f; aa += 0.005f)
							{
								foil.Get(aa, out _, out var cdv);
								cdMin = Mathf.Min(cdMin, cdv);
							}
						}
						if (cdMin == float.MaxValue)
						{
							cdMin = 0f;
						}
						psCdMin[ap] = cdMin;
					}
					float dRel = DragReliefMath.Relief(n9, n10, Mathf.Min(cdMin, cd) * rho * v2 * 0.5f * area * eff, vmag / DragReliefMath.SpeedOfSound(com.y), a * 57.29578f);
					if (dRel > 0f)
					{
						f += vHat * dRel;
					}
				}
				Vector3 nrm = Vector3.Cross(vf, lr * Vector3.right);
				float nm = nrm.magnitude;
				if (nm > 0.0001f)
				{
					f += -nrm / nm * (cl * rho * v2 * 0.5f * area * eff);
				}
				f = Vector3.ClampMagnitude(f, vmag * 0.5f * ap.mass * 60f);
				m += Vector3.Cross(com + lr * Acc.AP_centerOfLift(ap) - cg, f);
			}
			// flap-camber coupling, as CamberForce: the host's force increment for its surface's present deflection, unclamped
			if (psCamber == null)
			{
				psCamber = new List<(ControlSurface, AeroPart, AirfoilTable)>();
				AircraftParameters prm = ac.GetAircraftParameters();
				Dictionary<string, AeroPart> byName = new Dictionary<string, AeroPart>();
				foreach (AeroPart ap in aeroParts)
				{
					if (ap != null)
					{
						byName[Names.Canon(ap.gameObject.name)] = ap;
					}
				}
				foreach (ControlSurface cs in Acc.Collect<ControlSurface>(ac))
				{
					string hn = Camber.HostOf(cs.gameObject.name);
					if (hn == null || !byName.TryGetValue(hn, out var host))
					{
						continue;
					}
					int idx = Acc.AP_airfoil(host);
					Airfoil af = (idx >= 0 && prm != null && prm.airfoils != null && idx < prm.airfoils.Length) ? prm.airfoils[idx] : null;
					if (af == null || af.liftCoef == null || af.dragCoef == null || Acc.CS_visible(cs) == null)
					{
						continue;
					}
					psCamber.Add((cs, host, StallStagger.TableFor(host, af)));
				}
			}
			foreach (var (cs, host, table) in psCamber)
			{
				if (cs == null || host == null || host.rb == null || host.IsDetached() || !Acc.JobCreated(cs))
				{
					continue;
				}
				ref NuclearOption.Jobs.ControlSurfaceFields job = ref Acc.Job(cs);
				if (job.IsDetached)
				{
					continue;
				}
				float k = Camber.KForName(cs.gameObject.name);
				Transform lt = Acc.AP_liftNormal(host);
				if (k <= 0f || lt == null)
				{
					continue;
				}
				Transform vis = Acc.CS_visible(cs).transform;
				float delta = Camber.DeflectionOf(vis, job.restingRotation);
				Vector3 com = host.rb.worldCenterOfMass;
				Vector3 v = vAir + Vector3.Cross(w, com - cg);
				float v2 = v.sqrMagnitude;
				if (v2 < 25f)
				{
					continue;
				}
				float qS = 0.5f * rho * v2 * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
				Vector3 dF = Camber.Delta(v, lt.rotation, vis.rotation * Vector3.right, k, delta, table, qS);
				m += Vector3.Cross(com + lt.rotation * Acc.AP_centerOfLift(host) - cg, dF);
			}
			Vector3 mb = Quaternion.Inverse(rootRot) * m;
			return new Vector3(0f - mb.z, 0f - mb.x, mb.y);
		}

		// 2.31.2: the passive airframe telemetry (see the psA / psD / cePass fields). Body rates in the aero frame (x roll, z yaw),
		// world rotation w = rootRot * (-qa.y, qa.z, -qa.x) for aero rates qa = (p, q, r) - the inverse of the omega mapping in Tick.
		private void PassiveStep(float dt, float rho, Quaternion rootRot)
		{
			if (!PassiveProbeOn || isAI || tas < 30f)
			{
				return;
			}
			Vector3 vAir = vCgAir;
			Vector3 W(float pa, float qa, float ra)
			{
				return rootRot * new Vector3(0f - qa, ra, 0f - pa);
			}
			float pNow = omega.x, qNow = omega.y, rNow = omega.z;   // rad/s
			Vector3 m0 = AirframeMomentAt(vAir, W(pNow, qNow, rNow), rho, rootRot);
			psAX = AccRoll(m0);
			psAZ = AccYaw(m0);
			Vector3 m1 = AirframeMomentAt(vAir, W(pCmd * (MathF.PI / 180f), qNow, rCmd * (MathF.PI / 180f)), rho, rootRot);
			psDX = AccRoll(m1) - psAX;
			psDZ = AccYaw(m1) - psAZ;
			cePassXP = Mathf.Max(0f, psDX);
			cePassXN = Mathf.Max(0f, 0f - psDX);
			cePassZP = Mathf.Max(0f, psDZ);
			cePassZN = Mathf.Max(0f, 0f - psDZ);
			float ar = alpha * (MathF.PI / 180f);
			float dS = psDX * Mathf.Cos(ar) + psDZ * Mathf.Sin(ar);   // along the velocity-roll axis (body roll : yaw = cos a : sin a)
			cePassSP = Mathf.Max(0f, dS);
			cePassSN = Mathf.Max(0f, 0f - dS);
			psDerivTimer += dt;
			if (psDerivTimer < PassiveDerivPeriod)
			{
				return;
			}
			psDerivTimer = 0f;
			// sideslip: the CG flow at (alpha, beta +- db), same TAS (the parametrization of SlipMomentAt and the bench), the rates held
			const float db = 2f, dw = 5f;
			Vector3 VAt(float bDeg)
			{
				float fa = alpha * (MathF.PI / 180f), fb = bDeg * (MathF.PI / 180f);
				return rootRot * new Vector3(Mathf.Sin(fb), (0f - Mathf.Sin(fa)) * Mathf.Cos(fb), Mathf.Cos(fa) * Mathf.Cos(fb)) * tas;
			}
			Vector3 wNow = W(pNow, qNow, rNow);
			Vector3 mbP = AirframeMomentAt(VAt(beta + db), wNow, rho, rootRot);
			Vector3 mbN = AirframeMomentAt(VAt(beta - db), wNow, rho, rootRot);
			psLb = (AccRoll(mbP) - AccRoll(mbN)) / (2f * db);
			psNb = (AccYaw(mbP) - AccYaw(mbN)) / (2f * db);
			float dwr = dw * (MathF.PI / 180f);
			Vector3 mpP = AirframeMomentAt(vAir, W(pNow + dwr, qNow, rNow), rho, rootRot);
			Vector3 mpN = AirframeMomentAt(vAir, W(pNow - dwr, qNow, rNow), rho, rootRot);
			psLp = (AccRoll(mpP) - AccRoll(mpN)) / (2f * dw);
			psNp = (AccYaw(mpP) - AccYaw(mpN)) / (2f * dw);
			Vector3 mrP = AirframeMomentAt(vAir, W(pNow, qNow, rNow + dwr), rho, rootRot);
			Vector3 mrN = AirframeMomentAt(vAir, W(pNow, qNow, rNow - dwr), rho, rootRot);
			psLr = (AccRoll(mrP) - AccRoll(mrN)) / (2f * dw);
			psNr = (AccYaw(mrP) - AccYaw(mrN)) / (2f * dw);
		}

		// the rudders' one-side yaw and the other surfaces' roll authority at a hypothetical flow (deg/s^2)
		private void SlipAuthorityAt(float aDeg, float bDeg, float rho, Quaternion rootRot, ref EvalContext ctx, out float yawA, out float rollA)
		{
			float num = Mathf.Max(tas, 1f);
			float fa = aDeg * (MathF.PI / 180f);
			float fb = bDeg * (MathF.PI / 180f);
			Vector3 vector = rootRot * new Vector3(Mathf.Sin(fb) * num, (0f - Mathf.Sin(fa)) * Mathf.Cos(fb) * num, Mathf.Cos(fa) * Mathf.Cos(fb) * num);
			yawA = 0f;
			rollA = 0f;
			for (int i = 0; i < surfaces.Count; i++)
			{
				SurfaceEffector se = surfaces[i];
				if (!se.active)
				{
					continue;
				}
				se.PrepFlow(vector, rho, ref ctx, out SurfaceEffector.FlowPrep fp);   // F-22E 3.1.3: = EvalFlow twice in one flow
				Vector3 mMax = se.EvalFlowAt(se.max, ref fp, ref ctx);
				Vector3 mMin = se.EvalFlowAt(se.min, ref fp, ref ctx);
				if (se.kind == SurfaceKind.Rudder)
				{
					yawA += 0.5f * Mathf.Abs(mMax.z - mMin.z) * IaInv[2, 2] * 57.29578f * Mathf.Max(0.05f, rudderAuth);
				}
				else
				{
					rollA += 0.5f * Mathf.Abs(mMax.x - mMin.x) * IaInv[0, 0] * 57.29578f;
				}
			}
			// Revoker 0.3.0: the yaw nozzle is part of the yaw axis at every sideslip (thrust; its table does not move with the flow)
			if (tvcy != null && tvcy.active && tvcy.M != null && tvcy.N >= 2 && tvcy.hi - tvcy.lo > 0.5f)
			{
				float zA = tvcy.MAt(tvcy.lo).z;
				float zB = tvcy.MAt(tvcy.hi).z;
				yawA += 0.5f * Mathf.Abs(zB - zA) * IaInv[2, 2] * 57.29578f;
			}
			// Su-30 0.1.1: the canted nozzle differential is yaw AND roll at every sideslip (thrust; its table does not move with the
			// flow). 0.1.0 counted only the surfaces here, so past ~60 deg AoA, where the stabs' roll falls off with sideslip and the
			// nozzles are most of the lateral authority, the probe read the authority as halved by 4 deg of sideslip and cut the roll
			// governor's sideslip margin from 6 to 2.8 deg (Tom's 7 Nov 23:24 log: 70 deg AoA, 160-200 km/h, velocity roll held to
			// 22-25 deg/s with the nozzles' differential at ~1 deg).
			if (SlipAuthTvc && canted != null && tvcd != null && tvcd.active && tvcd.M != null && tvcd.N >= 2 && tvcd.hi - tvcd.lo > 0.5f)
			{
				Vector3 dA = tvcd.MAt(tvcd.lo);
				Vector3 dB = tvcd.MAt(tvcd.hi);
				yawA += 0.5f * Mathf.Abs(dB.z - dA.z) * IaInv[2, 2] * 57.29578f;
				rollA += 0.5f * Mathf.Abs(dB.x - dA.x) * IaInv[0, 0] * 57.29578f;
			}
		}

		// Su-30 0.1.1: the canted nozzle differential counted in the sideslip-authority probe (false = 0.1.0, surfaces only)
		internal static bool SlipAuthTvc = true;

		private void SlipProbeStep_(float dt, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			if (!Plugin.RollStopSlipProbe || isAI)
			{
				slipTolF = 25f;
				slipTolDbg = 25f;
				return;
			}
			slipProbeTimer += dt;
			if (slipProbeTimer >= SlipProbePeriod && tas > 30f)
			{
				slipProbeTimer = 0f;
				// the sign of a restoring yaw moment, latched once at low AoA where any conventional airframe is directionally
				// stable (the probe's sweep gives it: the yaw moment grows with the sideslip one way or the other)
				if (slipSignR == 0f && Mathf.Abs(alpha) < 10f && tas > 60f)
				{
					float n0 = AccYaw(SlipMomentAt(alpha, 0f, rho, rootRot, ref ctx));
					float nP = AccYaw(SlipMomentAt(alpha, 8f, rho, rootRot, ref ctx));
					float nN = AccYaw(SlipMomentAt(alpha, -8f, rho, rootRot, ref ctx));
					float dP = nP - n0;
					float dN = n0 - nN;
					slipSignR = ((dP + dN) >= 0f) ? 1f : (-1f);
				}
				if (slipSignR == 0f)
				{
					slipTolDbg = 25f;
				}
				else
				{
					float dep = SlipProbeMax + 5f;
					float auth = SlipProbeMax + 5f;
					float stiff = 0f;
					float n0 = AccYaw(SlipMomentAt(alpha, 0f, rho, rootRot, ref ctx));
					SlipAuthorityAt(alpha, 0f, rho, rootRot, ref ctx, out float y0, out float r0);
					for (int sgn = -1; sgn <= 1; sgn += 2)
					{
						for (float b = SlipProbeStep; b <= SlipProbeMax + 0.01f; b += SlipProbeStep)
						{
							float bb = sgn * b;
							// restoring yaw at this sideslip (deg/s^2; positive = toward zero sideslip)
							float rest = (AccYaw(SlipMomentAt(alpha, bb, rho, rootRot, ref ctx)) - n0) * slipSignR * sgn;
							if (b <= SlipProbeStep + 0.01f)
							{
								stiff += 0.5f * rest / SlipProbeStep;
							}
							SlipAuthorityAt(alpha, bb, rho, rootRot, ref ctx, out float yB, out float rB);
							// departure: the airframe pushes the sideslip open harder than SlipAuthUse of the yaw axis can pull back - the
							// rudders' yaw at that sideslip, or past the rudder fade the allocator's yaw authority (the stabs'): 2.5.16,
							// the 14:55 log read a 4 deg bound at -20..-60 deg AoA where the rudders' tables are faded to nothing
							// and any destabilizing moment at all tripped the test (the margin never moved, the episodes were short)
							// Revoker 0.3.0: the yaw axis at this sideslip is the rudder (faded) + the nozzle, as SlipAuthorityAt sums them;
							// no allocator-wide credit (see yawEffs)
							float yHold = yB;
							if (rest < 0f - SlipAuthUse * yHold && b < dep)
							{
								dep = b;
							}
							// authority: the rudders' yaw or the surfaces' roll down to SlipAuthFrac of their zero-sideslip value
							if ((yB < SlipAuthFrac * y0 || rB < SlipAuthFrac * r0) && b < auth)
							{
								auth = b;
							}
							if (dep <= b && auth <= b)
							{
								break;
							}
						}
					}
					slipDepDbg = dep;
					slipAuthDbg = auth;
					slipStiffDbg = stiff;
					slipTolDbg = Mathf.Min(dep, auth);
					// Revoker 0.4.0: the airframe's yaw damping at the present AoA and sideslip, deg/s^2 per deg/s of body yaw rate (negative = damping)
					float nR0 = AccYaw(SlipMomentAt(alpha, beta, rho, rootRot, ref ctx, 0f));
					float nR1 = AccYaw(SlipMomentAt(alpha, beta, rho, rootRot, ref ctx, SlipDampProbeRate));
					slipDampDbg = (nR1 - nR0) / (SlipDampProbeRate * 57.29578f);
					// Revoker 0.4.0: the signed sideslip slopes for the reversal model (tables' own sign; the kinematics decide what restores)
					Vector3 mP = SlipMomentAt(alpha, SlipProbeStep, rho, rootRot, ref ctx);
					Vector3 mN = SlipMomentAt(alpha, 0f - SlipProbeStep, rho, rootRot, ref ctx);
					slipYawSlopeDbg = (AccYaw(mP) - AccYaw(mN)) / (2f * SlipProbeStep);
					slipRollSlopeDbg = (AccRoll(mP) - AccRoll(mN)) / (2f * SlipProbeStep);
				}
			}
			slipTolF += (slipTolDbg - slipTolF) * (dt / (SlipTolTau + dt));
			slipDampF += (slipDampDbg - slipDampF) * (dt / (SlipTolTau + dt));
			slipYawSlopeF += (slipYawSlopeDbg - slipYawSlopeF) * (dt / (SlipTolTau + dt));
			slipRollSlopeF += (slipRollSlopeDbg - slipRollSlopeF) * (dt / (SlipTolTau + dt));
		}

		private void AsymSweepPrep(Quaternion rootRot)
		{
			asymSweepIas = Mathf.Max(ias, 1f);
			AsymPrepSym(rootRot);
			// Su-30 0.5.0: the coupled check's state, latched for the whole sweep. The LP runs only with a load to judge (lateral CG
			// offset past CoupHeldDy0 or a damage gate); without one the cap reads the old sweep, as before. It runs in both TVC
			// states: with the nozzles vectoring they are two of its effectors (CplTvc), each on its own canted axis, read here on the
			// main thread; locked (DisableTVC) or not vectoring, the thrust stays in the load as before.
			cplActive = CoupledCap;
			cplLoaded = Mathf.Abs(asymDy) > CoupHeldDy0 || (AsymDamageGate && dmgGateHold > 0f);
			cplPitchGate = Plugin.AsymPitchMargin > 0f && (Mathf.Abs(asymDy) - CoupHeldDy0 > 0.5f * CoupHeldDySpan || (AsymDamageGate && dmgGateHold > 0f));
			// Su-30 0.5.2: the clean jet's limit on the sweep's grid (the reference point of CplDemand), each side
			{
				int nRef = asymAlphas.Length;
				cplRefAp = Mathf.Clamp(Mathf.Min(Plugin.AoAPos, authCapEff), 5f, asymAlphas[nRef - 1]);
				cplRefAn = Mathf.Clamp(Mathf.Max(Plugin.AoANeg, authCapNegEff), asymAlphas[0], -5f);
				int i0p = -1, i0n = -1;
				for (int kR = 0; kR < nRef - 1; kR++)
				{
					if (asymAlphas[kR] <= cplRefAp && cplRefAp <= asymAlphas[kR + 1])
					{
						i0p = kR;
					}
					if (i0n < 0 && asymAlphas[kR] <= cplRefAn && cplRefAn <= asymAlphas[kR + 1])
					{
						i0n = kR;
					}
				}
				cplRefI0p = i0p;
				cplRefI1p = (i0p < 0) ? -1 : (i0p + 1);
				cplRefI0n = i0n;
				cplRefI1n = (i0n < 0) ? -1 : (i0n + 1);
			}
			cplTvOn = false;
			if (cplActive && cplLoaded && CplTvc && tvc != null && tvc.active && tvc.canted != null && !Plugin.DisableTVC && tvc.Mn != null && tvc.N > 1)
			{
				int nE = Mathf.Min(tvc.Mn.Length, cplTvM.Length);
				cplTvE = nE;
				cplTvN = tvc.N;
				if (cplTvG == null || cplTvG.Length != tvc.N)
				{
					cplTvG = new float[tvc.N];
				}
				Array.Copy(tvc.grid, cplTvG, tvc.N);
				for (int e = 0; e < nE; e++)
				{
					if (cplTvM[e] == null || cplTvM[e].Length != tvc.N)
					{
						cplTvM[e] = new Vector3[tvc.N];
					}
					Array.Copy(tvc.Mn[e], cplTvM[e], tvc.N);
					cplTvHas[e] = tvc.MnOn[e];
					cplTvNow[e] = tvc.canted.dNow[e];
					cplTvOn |= tvc.MnOn[e];
				}
			}
			asymMT = ((tvc.active || Plugin.DisableTVC) ? (((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : FixedThrustLat());
		}

		private Vector3 AsymMomentAt(float aDeg, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			float[] array = asymDsym;
			if (array == null)
			{
				return Vector3.zero;
			}
			float num = Mathf.Max(tas, 1f);
			float f = aDeg * (MathF.PI / 180f);
			Vector3 vector = rootRot * new Vector3(0f, (0f - Mathf.Sin(f)) * num, Mathf.Cos(f) * num);
			Vector3 vector2 = vector / num;
			float num2 = 0.5f * rho * num * num;
			Vector3 zero = Vector3.zero;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached())
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				if (!(transform == null))
				{
					Quaternion rotation = AirframeRot(aeroPart, transform);
					Vector3 lhs = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					Vector3 rhs = -vector2 * (0.5f * num2 * Acc.AP_dragArea(aeroPart));
					if (!surfaceParts.Contains(aeroPart) && foils.TryGetValue(aeroPart, out var value) && Acc.AP_wingArea(aeroPart) > 0f)
					{
						rhs += PartForce(vector, rotation, rotation * Vector3.right, value, 0.5f * rho * Acc.AP_wingArea(aeroPart) * Acc.AP_wingEff(aeroPart));
					}
					zero += Vector3.Cross(lhs, rhs);
				}
			}
			Vector3 vector3 = Quaternion.Inverse(rootRot) * zero;
			Vector3 result = new Vector3(0f - vector3.z, 0f - vector3.x, vector3.y) + (tvc.active ? (((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : FixedThrustLat());
			for (int i = 0; i < surfaces.Count; i++)
			{
				if (surfaces[i].active)
				{
					result += surfaces[i].EvalFlow(array[i], vector, rho, ref ctx);
				}
			}
			foreach (HingeFlapEffector hA in hinges)
			{
				if (hA.active)
				{
					result += hA.EvalFlow(hA.SymNow, vector, rho, ref ctx);
				}
			}
			return result;
		}

		// Su-30 0.2.0 (from F-22E 3.1.2, ParallelSweep): the asymmetric-load sweep. The parts' Unity state (lift-normal rotation, centre of
		// mass and lift, areas, foil) is read once per sweep tick on the main thread (SwGather) - the same values every point of the tick
		// read before, since nothing moves the parts inside a tick - and the tick's AoA points are computed in parallel (IndexFork), each
		// writing only its own index of the sweep arrays, joined before anything reads them. Same expressions in the same order per point.
		// The hinged flaps' EvalFlow reads their host's rigidbody, so with any hinged flap active the points run on this thread.
		private void AsymSweepPoints(int k0, int k1, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			float[] array = asymDsym;
			if (array == null)
			{
				return;
			}
			float num = Mathf.Max(tas, 1f);
			Vector3 vector = asymMT;
			SwGather(rho);
			int n = k1 - k0;
			SwGridGeom(k0, k1, ref ctx);
			swNative = FcsNative.SweepOn;   // F-22E 4.1.0 port: one answer for all of this tick's points
			bool hingeOn = false;
			foreach (HingeFlapEffector hA in hinges)
			{
				hingeOn |= hA.active;
			}
			if (Plugin.ParallelSweep && n > 1 && !hingeOn)
			{
				if (swWork == null)
				{
					swWork = new SweepWork(this);
				}
				swWork.Set(k0, rho, rootRot, ctx, array, num, vector);
				IndexFork.Finish(IndexFork.Start(swWork, n));
			}
			else
			{
				for (int i = k0; i < k1; i++)
				{
					AsymSweepPoint(i, rho, rootRot, ref ctx, array, num, vector);
				}
			}
			if (k1 >= asymAlphas.Length)
			{
				asymCycleDone = true;
			}
		}

		// Su-30 0.4.0, port of F-22E 4.0.1 (SweepGridCache): the sweep's capacity loop evaluates each surface's whole deflection grid in every AoA point's
		// flow. The deflections' geometry does not depend on the point, so it is worked out here once per sweep tick
		// (SurfaceEffector.GridGeom, the surfaces in parallel) for the surfaces whose capacity loop at least one of the tick's points
		// runs; the points then use SurfaceEffector.EvalFlowGrid. swGrid[j] false = that surface's loop calls EvalFlow as before.
		private bool[] swGrid;

		// 4.1.0 (NativeSweep): this tick's capacity loops run in the native core (FcsNative.Grid, mode 1) where swGrid[j] is set
		private bool swNative;

		private readonly List<SurfaceEffector> swGeomList = new List<SurfaceEffector>();

		private readonly List<float[]> swGeomDefl = new List<float[]>();

		private GeomWork swGeomWork;

		// this jet: per surface, the deflections the capacity loop evaluates (the grid, or for a stabilator the AsymStabTravel list)
		private float[][] swDefl;

		private void SwGridGeom(int k0, int k1, ref EvalContext ctx)
		{
			if (swGrid == null || swGrid.Length != surfaces.Count)
			{
				swGrid = new bool[surfaces.Count];
				swDefl = new float[surfaces.Count][];
			}
			swGeomList.Clear();
			swGeomDefl.Clear();
			bool on = Plugin.SweepGridCache;
			for (int j = 0; j < surfaces.Count; j++)
			{
				swGrid[j] = false;
				SurfaceEffector se = surfaces[j];
				if (!on || !se.active || se.N <= 0 || se.grid == null)
				{
					continue;
				}
				// the capacity loop's own condition (AsymSweepPoint: num6 != 0 || num7 != 0), for any point of this tick
				bool need = false;
				switch (se.kind)
				{
				case SurfaceKind.Flaperon:
				case SurfaceKind.Aileron:
					need = true;
					break;
				case SurfaceKind.Stabilator:
					for (int i = k0; i < k1 && !need; i++)
					{
						need = Mathf.Lerp(Plugin.StabRollShare, 1f, Mathf.InverseLerp(30f, 45f, Mathf.Abs(asymAlphas[i]))) != 0f;
					}
					break;
				case SurfaceKind.Rudder:
					for (int i = k0; i < k1 && !need; i++)
					{
						need = RudderAuthAt(asymAlphas[i]) != 0f;
					}
					break;
				}
				if (need)
				{
					swGrid[j] = true;
					float[] dl = SwDeflections(se, j);
					// 4.1.0: with the native table rebuild the geometry of this tick is already there (SurfaceEffector.EvalCompute),
					// if the sweep evaluates the same deflections
					if (!se.GridGeomFreshFor(ref ctx, dl))
					{
						swGeomList.Add(se);
						swGeomDefl.Add(dl);
					}
				}
			}
			// Su-30 0.5.6 (CplGridCache, from the Ternion 0.3.0): the coupled check's points, for every surface it takes points from, if any
			// of this tick's sweep points solves it
			if (swCpl == null || swCpl.Length != surfaces.Count)
			{
				swCpl = new bool[surfaces.Count];
			}
			swCplList.Clear();
			bool cplAny = false;
			if (on && CplGridCache && cplActive && cplLoaded)
			{
				for (int i = k0; i < k1 && !cplAny; i++)
				{
					cplAny = CplHere(i);
				}
			}
			for (int j = 0; j < surfaces.Count; j++)
			{
				SurfaceEffector se = surfaces[j];
				swCpl[j] = cplAny && se.active && se.N > 0 && se.grid != null;
				if (swCpl[j])
				{
					swCplList.Add(se);
				}
			}
			int m = swGeomList.Count;
			int mc = swCplList.Count;
			if (m + mc == 0)
			{
				return;
			}
			if (Plugin.ParallelSweep && m + mc > 1)
			{
				if (swGeomWork == null)
				{
					swGeomWork = new GeomWork(swGeomList, swGeomDefl, swCplList);
				}
				swGeomWork.ctx = ctx;
				IndexFork.Finish(IndexFork.Start(swGeomWork, m + mc));
			}
			else
			{
				for (int i = 0; i < m; i++)
				{
					swGeomList[i].GridGeom(ref ctx, swGeomDefl[i]);
				}
				for (int i = 0; i < mc; i++)
				{
					swCplList[i].CplGeom(ref ctx, CplStride(swCplList[i].N));
				}
			}
		}

		private sealed class GeomWork : IIndexWork
		{
			private readonly List<SurfaceEffector> list;

			private readonly List<float[]> defl;

			public EvalContext ctx;

			private readonly List<SurfaceEffector> cpl;

			public GeomWork(List<SurfaceEffector> list, List<float[]> defl, List<SurfaceEffector> cpl)
			{
				this.list = list;
				this.defl = defl;
				this.cpl = cpl;
			}

			public void Do(int i)
			{
				EvalContext cx = ctx;
				if (i < list.Count)
				{
					list[i].GridGeom(ref cx, defl[i]);
				}
				else
				{
					SurfaceEffector se = cpl[i - list.Count];
					se.CplGeom(ref cx, CplStride(se.N));
				}
			}
		}

		// the deflections AsymSweepPoint's capacity loop evaluates for surface j, in its order (same expression as the loop's gS)
		private float[] SwDeflections(SurfaceEffector se, int j)
		{
			if (!(se.kind == SurfaceKind.Stabilator && AsymStabTravel > 0f && se.N > 1))
			{
				return se.grid;
			}
			float[] d = swDefl[j];
			if (d == null || d.Length != se.N)
			{
				d = (swDefl[j] = new float[se.N]);
			}
			for (int l = 0; l < se.N; l++)
			{
				d[l] = Mathf.Lerp(0f - AsymStabTravel, AsymStabTravel, (float)l / (float)(se.N - 1));
			}
			return d;
		}

		private sealed class SweepWork : IIndexWork
		{
			private readonly FcsController c;
			private int k0;
			private float rho, num;
			private Quaternion rootRot;
			private EvalContext ctx;
			private float[] array;
			private Vector3 vector;

			public SweepWork(FcsController c)
			{
				this.c = c;
			}

			public void Set(int k0, float rho, Quaternion rootRot, EvalContext ctx, float[] array, float num, Vector3 vector)
			{
				this.k0 = k0;
				this.rho = rho;
				this.rootRot = rootRot;
				this.ctx = ctx;
				this.array = array;
				this.num = num;
				this.vector = vector;
			}

			public void Do(int i)
			{
				EvalContext cx = ctx;
				c.AsymSweepPoint(k0 + i, rho, rootRot, ref cx, array, num, vector);
			}
		}

		private SweepWork swWork;

		// the sweep's per-part snapshot (SwGather)
		private int swN;
		private Quaternion[] swRot = new Quaternion[0];
		private Vector3[] swLhs = new Vector3[0];
		private float[] swDrag = new float[0];
		private AirfoilTable[] swFoil = new AirfoilTable[0];
		private float[] swQ = new float[0];

		private void SwGather(float rho)
		{
			int cap = aeroParts.Count;
			if (swRot.Length < cap)
			{
				swRot = new Quaternion[cap];
				swLhs = new Vector3[cap];
				swDrag = new float[cap];
				swFoil = new AirfoilTable[cap];
				swQ = new float[cap];
			}
			swN = 0;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached())
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				if (!(transform == null))
				{
					Quaternion rotation = AirframeRot(aeroPart, transform);
					int k = swN++;
					swRot[k] = rotation;
					swLhs[k] = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					swDrag[k] = Acc.AP_dragArea(aeroPart);
					if (!surfaceParts.Contains(aeroPart) && foils.TryGetValue(aeroPart, out var value) && Acc.AP_wingArea(aeroPart) > 0f)
					{
						swFoil[k] = value;
						swQ[k] = 0.5f * rho * Acc.AP_wingArea(aeroPart) * Acc.AP_wingEff(aeroPart);
					}
					else
					{
						swFoil[k] = null;
						swQ[k] = 0f;
					}
				}
			}
		}

		// one AoA point of the sweep: pure maths on the snapshot, the surfaces' gathered state (EvalFlow) and the controller's settings;
		// writes index i of the sweep arrays only
		private void AsymSweepPoint(int i, float rho, Quaternion rootRot, ref EvalContext ctx, float[] array, float num, Vector3 vector)
		{
			float f = asymAlphas[i] * (MathF.PI / 180f);
			Vector3 vector2 = rootRot * new Vector3(0f, (0f - Mathf.Sin(f)) * num, Mathf.Cos(f) * num);
			Vector3 vector3 = vector2 / num;
			float num2 = 0.5f * rho * num * num;
			Vector3 zero = Vector3.zero;
			// the parts' state from SwGather (read once per sweep tick on the main thread), same expressions, same order
			for (int k = 0; k < swN; k++)
			{
				Vector3 rhs = -vector3 * (0.5f * num2 * swDrag[k]);
				if (swFoil[k] != null)
				{
					rhs += PartForce(vector2, swRot[k], swRot[k] * Vector3.right, swFoil[k], swQ[k]);
				}
				zero += Vector3.Cross(swLhs[k], rhs);
			}
			Vector3 vector4 = Quaternion.Inverse(rootRot) * zero;
			Vector3 m = new Vector3(0f - vector4.z, 0f - vector4.x, vector4.y) + vector;
			// Su-30 0.5.0 (CoupledCap, from the Ternion 0.2.2): the airframe's own moment (the load) before any surface; the surfaces
			// become the LP's points
			Vector3 mAir = m;
			CplPoint cp = null;
			bool cplHere = CplHere(i);
			if (cplActive && cplLoaded && cplHere)
			{
				cp = cplPts[i] ?? (cplPts[i] = new CplPoint());
				cp.Reset();
			}
			// (CplBetaDesign) the airframe's moment change at +/- CplBetaDesign of sideslip, same parts: each side's reserve is solved
			// with the sideslip that rolls against it (CplSolve)
			Vector3 dMbP = Vector3.zero, dMbN = Vector3.zero;
			if (cp != null && CplBetaDesign > 0f)
			{
				float bD = CplBetaDesign * (MathF.PI / 180f);
				for (int sB = 0; sB < 2; sB++)
				{
					float sb = (sB == 0) ? 1f : -1f;
					Vector3 vB = rootRot * new Vector3(sb * Mathf.Sin(bD) * num, (0f - Mathf.Sin(f)) * Mathf.Cos(bD) * num, Mathf.Cos(f) * Mathf.Cos(bD) * num);
					Vector3 vBh = vB / num;
					Vector3 zB = Vector3.zero;
					for (int k = 0; k < swN; k++)
					{
						Vector3 rhsB = -vBh * (0.5f * num2 * swDrag[k]);
						if (swFoil[k] != null)
						{
							rhsB += PartForce(vB, swRot[k], swRot[k] * Vector3.right, swFoil[k], swQ[k]);
						}
						zB += Vector3.Cross(swLhs[k], rhsB);
					}
					Vector3 zBl = Quaternion.Inverse(rootRot) * (zB - zero);
					Vector3 dM = new Vector3(0f - zBl.z, 0f - zBl.x, zBl.y);
					if (sB == 0) dMbP = dM; else dMbN = dM;
				}
			}
			// (CplTvc) the nozzles as effectors of the LP, one per engine: each swivels about its own canted axis, so one nozzle's
			// deflection makes pitch, roll and yaw together, and its table carries that engine's whole thrust moment about the CoM
			// at each angle (the thrust x lateral-offset yaw included). The thrust at the nozzles' present angle ('vector') then
			// leaves the load. Not with an engine out or weak (the thrust is left out of the cap there: EngineOutFixedCap).
			bool thrOut = ((engOut || engAsym) && EngineOutFixedCap) || tKHoldT > 0f;
			bool cplTv = cp != null && cplTvOn && !thrOut;
			if (cplTv)
			{
				for (int eT = 0; eT < cplTvE; eT++)
				{
					if (!cplTvHas[eT])
					{
						continue;
					}
					if (CplDebug) { Vector3 mNw = TvNozzleAt(eT, cplTvNow[eT]); cplDbgNow[cp.nEff] = new Vector3(AccRoll(mNw), mNw.y * IaInv[1, 1] * 57.29578f, AccYaw(mNw)); cplDbgName[cp.nEff] = "nozzle " + eT; cplDbgDefl[cp.nEff] = cplTvNow[eT]; }
					for (int kT = 0; kT < CplStride(cplTvN); kT++)
					{
						int lT = CplIdx(kT, cplTvN);
						Vector3 mT = cplTvM[eT][lT];
						cp.Add(AccRoll(mT), mT.y * IaInv[1, 1] * 57.29578f, AccYaw(mT), cplTvG[lT]);
					}
					cp.EndEff();
				}
			}
			float pAero = m.y - vector.y;
			float pStabMin = 0f;
			float pStabMax = 0f;
			float pFlapNow = 0f;
			float pRud0 = 0f;
			float pRudP = 0f;
			float pRudM = 0f;
			float toe = 0f;
			float num3 = 0f;
			float num4 = 0f;
			float t = Mathf.InverseLerp(30f, 45f, Mathf.Abs(asymAlphas[i]));
			float num5 = RudderAuthAt(asymAlphas[i]);
			// Revoker 0.2.0: the inboard elevon pair at its symmetric deflection (the roll it makes for the pilot is no load)
			float pHingeAuth = 0f;
			foreach (HingeFlapEffector hA in hinges)
			{
				if (hA.active)
				{
					Vector3 mH = hA.EvalFlow(hA.SymNow, vector2, rho, ref ctx);
					m += mH;
					// Revoker 0.2.5: and its nose-down travel for the authority cap (ElevonAuthShare of it)
					float pLoH = mH.y;
					for (int lH = 0; lH < hA.N; lH++)
					{
						pLoH = Mathf.Min(pLoH, hA.EvalFlow(hA.grid[lH], vector2, rho, ref ctx).y);
					}
					pHingeAuth += Mathf.Lerp(mH.y, pLoH, Plugin.ElevonPitch ? ElevonAuthShare : 0f);
					if (cp != null)
					{
						if (CplDebug) { Vector3 mNw = hA.EvalFlow(hA.now, vector2, rho, ref ctx); cplDbgNow[cp.nEff] = new Vector3(AccRoll(mNw), mNw.y * IaInv[1, 1] * 57.29578f, AccYaw(mNw)); cplDbgName[cp.nEff] = "hinge " + hA.name; cplDbgDefl[cp.nEff] = hA.now; }
						for (int kH = 0; kH < CplStride(hA.N); kH++)
						{
							int lH = CplIdx(kH, hA.N);
							Vector3 mHl = hA.EvalFlow(hA.grid[lH], vector2, rho, ref ctx);
							cp.Add(AccRoll(mHl), mHl.y * IaInv[1, 1] * 57.29578f, AccYaw(mHl), hA.grid[lH]);
						}
						cp.EndEff();
					}
				}
			}
			for (int j = 0; j < surfaces.Count; j++)
			{
				SurfaceEffector surfaceEffector = surfaces[j];
				if (!surfaceEffector.active)
				{
					continue;
				}
				m += surfaceEffector.EvalFlow(array[j], vector2, rho, ref ctx);
				// the canards move together only (CanardSymmetricOnly): the pair is one effector of the LP, added at the left one
				bool canPair = cp != null && Plugin.CanardSymmetric && surfaceEffector.isCanard && canL != null && canR != null && canL.active && canR.active && canL.N == canR.N;
				if (cp != null && !(canPair && surfaceEffector == canR))
				{
					SurfaceEffector seO = canPair ? canR : null;
					surfaceEffector.PrepFlow(vector2, rho, ref ctx, out SurfaceEffector.FlowPrep fpC);
					SurfaceEffector.FlowPrep fpO = default(SurfaceEffector.FlowPrep);
					if (seO != null)
					{
						seO.PrepFlow(vector2, rho, ref ctx, out fpO);
					}
					// (CplAttachedOnly, CplStabPeakRule) a stab's or canard's points as the allocator would fly them: its own rule
					// (ClampStabToPeak) holds them to a local AoA of the stall angle + StabPeakMargin, opening by 8 deg per degree of body
					// AoA past 40. On this jet StabPeakMargin is 90 (the clamp is off), so every point counts (11 Nov bench, 16 cm,
					// 52 deg: the allocator holds the left elevator at 9.7 deg, stalled, where the attached side starts at 26). With a
					// clamp set, only the points inside it; if none is, the least stalled one.
					float attLim = CplStabPeakRule ? (surfaceEffector.stallAoA + Plugin.StabPeakMargin + Mathf.Max(0f, Mathf.Abs(asymAlphas[i]) - 40f) * 8f) : surfaceEffector.stallAoA;
					bool attOnly = CplAttachedOnly && surfaceEffector.kind == SurfaceKind.Stabilator && !(CplStabPeakRule && (!StabPeakClamp || attLim > 89f));
					// Su-30 0.5.6 (CplGridCache): this tick's geometry for the points (SwGridGeom) and, native, all their moments at once
					// (both canards of the pair: each on its own geometry, summed as before)
					int jO = (seO != null) ? surfaces.IndexOf(seO) : -1;
					bool cGeo = swCpl != null && swCpl[j] && surfaceEffector.cN == CplStride(surfaceEffector.N)
						&& (seO == null || (jO >= 0 && swCpl[jO] && seO.cN == CplStride(seO.N) && seO.N == surfaceEffector.N));
					Vector3[] cM = (cGeo && swNative && surfaceEffector.CplMomentsNative(ref fpC, ref ctx, cp.scratchM)) ? cp.scratchM : null;
					Vector3[] cMO = (cM != null && seO != null) ? (seO.CplMomentsNative(ref fpO, ref ctx, cp.scratchM2) ? cp.scratchM2 : null) : null;
					if (seO != null && cMO == null)
					{
						cM = null;
					}
					int kLeast = -1;
					float locLeast = float.MaxValue;
					if (attOnly)
					{
						for (int kA = 0; kA < CplStride(surfaceEffector.N); kA++)
						{
							float la = Mathf.Abs((cGeo ? surfaceEffector.LocalAoACpl(kA, ref fpC) : surfaceEffector.LocalAoAFlow(surfaceEffector.grid[CplIdx(kA, surfaceEffector.N)], ref fpC)));
							if (la < locLeast)
							{
								locLeast = la;
								kLeast = kA;
							}
						}
					}
					if (CplDebug) { Vector3 mNw = surfaceEffector.EvalFlowAt(surfaceEffector.now, ref fpC, ref ctx); if (seO != null) mNw += seO.EvalFlowAt(seO.now, ref fpO, ref ctx); cplDbgNow[cp.nEff] = new Vector3(AccRoll(mNw), mNw.y * IaInv[1, 1] * 57.29578f, AccYaw(mNw)); cplDbgName[cp.nEff] = (seO != null) ? "canard pair" : surfaceEffector.name; cplDbgDefl[cp.nEff] = surfaceEffector.now; }
					for (int kC = 0; kC < CplStride(surfaceEffector.N); kC++)
					{
						float gC = surfaceEffector.grid[CplIdx(kC, surfaceEffector.N)];
						if (attOnly && kC != kLeast && Mathf.Abs((cGeo ? surfaceEffector.LocalAoACpl(kC, ref fpC) : surfaceEffector.LocalAoAFlow(gC, ref fpC))) > attLim)
						{
							continue;
						}
						Vector3 mC = (cM != null) ? cM[kC] : (cGeo ? surfaceEffector.EvalFlowCpl(kC, ref fpC, ref ctx) : surfaceEffector.EvalFlowAt(gC, ref fpC, ref ctx));
						if (seO != null)
						{
							mC += (cMO != null) ? cMO[kC] : (cGeo ? seO.EvalFlowCpl(kC, ref fpO, ref ctx) : seO.EvalFlowAt(gC, ref fpO, ref ctx));
						}
						cp.Add(AccRoll(mC), mC.y * IaInv[1, 1] * 57.29578f, AccYaw(mC), gC);
					}
					cp.EndEff();
				}
				if (surfaceEffector.kind == SurfaceKind.Flaperon || surfaceEffector.kind == SurfaceKind.Aileron)
				{
					float pRef = surfaceEffector.EvalFlow((flapSymRef != null && j < flapSymRef.Length) ? flapSymRef[j] : 0f, vector2, rho, ref ctx).y;
					// Revoker 0.2.5 (ElevonAuthShare): the delta's elevons are pitch effectors (ElevonPitch), so their nose-down
					// travel counts toward the authority the low-thrust / no-TVC cap judges by, at ElevonAuthShare of it (the
					// rest stays for roll at the cap); the F-22E counted its flaperons only at their present trim
					if (surfaceEffector.kind == SurfaceKind.Aileron && Plugin.ElevonPitch && ElevonAuthShare > 0f)
					{
						float pLo = pRef;
						for (int lE = 0; lE < surfaceEffector.N; lE++)
						{
							pLo = Mathf.Min(pLo, surfaceEffector.EvalFlow(surfaceEffector.grid[lE], vector2, rho, ref ctx).y);
						}
						pRef = Mathf.Lerp(pRef, pLo, ElevonAuthShare);
					}
					pFlapNow += pRef;
				}
				if (surfaceEffector.kind == SurfaceKind.Rudder && toe > 0f)
				{
					pRud0 += surfaceEffector.EvalFlow(0f, vector2, rho, ref ctx).y;
					pRudP += surfaceEffector.EvalFlow(toe, vector2, rho, ref ctx).y;
					pRudM += surfaceEffector.EvalFlow(0f - toe, vector2, rho, ref ctx).y;
				}
				float num6;
				float num7;
				switch (surfaceEffector.kind)
				{
				case SurfaceKind.Flaperon:
				case SurfaceKind.Aileron:
					num6 = 1f;
					num7 = 1f;
					break;
				case SurfaceKind.Stabilator:
					num6 = (num7 = Mathf.Lerp(Plugin.StabRollShare, 1f, t));
					break;
				case SurfaceKind.Rudder:
					num6 = 0f;
					num7 = num5;
					break;
				default:
					num6 = (num7 = 0f);
					break;
				}
				if (num6 != 0f || num7 != 0f)
				{
					float num8 = float.MaxValue;
					float num9 = float.MinValue;
					float num10 = float.MaxValue;
					float num11 = float.MinValue;
					float pMinS = float.MaxValue;
					float pMaxS = float.MinValue;
					// F-22E 4.0.1 port (SweepGridCache): the deflection's geometry from this tick's GridGeom (built for the same gS list),
					// the flow's terms from fp (= EvalFlowAt). F-22E 4.1.0 port (NativeSweep): the whole loop in the native core; it leaves
					// the six values untouched when it declines
					bool gridGeom = swGrid != null && j < swGrid.Length && swGrid[j];
					SurfaceEffector.FlowPrep fp = default(SurfaceEffector.FlowPrep);
					if (gridGeom)
					{
						surfaceEffector.PrepFlow(vector2, rho, ref ctx, out fp);
					}
					if (!(gridGeom && swNative && surfaceEffector.GridRangeNative(ref fp, ref ctx, IaInv[0, 0], IaInv[0, 2], IaInv[2, 0], IaInv[2, 2], ref pMinS, ref pMaxS, ref num8, ref num9, ref num10, ref num11)))
					for (int l = 0; l < surfaceEffector.N; l++)
					{
						// Revoker 0.2.7: the asymmetric-load cap judges the canards on their stock 40 deg; the 5 deg added in 0.2.7
						// stay a reserve it does not spend. Counted, they raised the cap with a canard shot off from 30-47 to 49-64
						// deg (bench, canard_L + wingtip_R + aileron_R + flap_L gone, 350 km/h reversal), where the stop-side roll
						// authority was down to 11-13 deg/s^2 (0.2.6: 33-65) and the roll error doubled.
						// Sampled on the stock grid too (31 points over +-40), not the +-45 grid with points over 40 skipped: the cap is
						// sensitive to where the sweep samples, and the coarser grid alone raised it 25-38 -> 51-53 deg in the full
						// pull at 290-300 km/h with that damage.
						float gS = (surfaceEffector.kind == SurfaceKind.Stabilator && AsymStabTravel > 0f && surfaceEffector.N > 1) ? Mathf.Lerp(0f - AsymStabTravel, AsymStabTravel, (float)l / (float)(surfaceEffector.N - 1)) : surfaceEffector.grid[l];
						Vector3 m2 = gridGeom ? surfaceEffector.EvalFlowGrid(l, ref fp, ref ctx) : surfaceEffector.EvalFlow(gS, vector2, rho, ref ctx);
						pMinS = Mathf.Min(pMinS, m2.y);
						pMaxS = Mathf.Max(pMaxS, m2.y);
						float b = AccRoll(m2);
						float b2 = AccYaw(m2);
						num8 = Mathf.Min(num8, b);
						num9 = Mathf.Max(num9, b);
						num10 = Mathf.Min(num10, b2);
						num11 = Mathf.Max(num11, b2);
					}
					num3 += num6 * 0.5f * (num9 - num8);
					num4 += num7 * 0.5f * (num11 - num10);
					if (surfaceEffector.kind == SurfaceKind.Stabilator)
					{
						pStabMin += pMinS;
						pStabMax += pMaxS;
					}
				}
			}
			asymNetDn[i] = (pAero + pStabMin) * IaInv[1, 1] * 57.29578f;
			asymAirP[i] = pAero * IaInv[1, 1] * 57.29578f;
			asymStabLo[i] = pStabMin * IaInv[1, 1] * 57.29578f;
			asymStabHi[i] = pStabMax * IaInv[1, 1] * 57.29578f;
			// 2.0.9: the symmetric rudder toe (2.0.7 pitch aid, up to RudderToeMax each way) counts toward the nose-down
			// authority the low-thrust cap judges by (the rudders' pitch was left out before, neutral or toed)
			float pRud = (toe > 0f) ? Mathf.Min(pRud0, Mathf.Min(pRudP, pRudM)) : 0f;
			authNetDn[i] = (pAero + pStabMin + pFlapNow + pRud + pHingeAuth) * IaInv[1, 1] * 57.29578f;
			asymNetUp[i] = (pAero + pStabMax) * IaInv[1, 1] * 57.29578f;
			asymAr[i] = AccRoll(m);
			asymAy[i] = AccYaw(m);
			asymCr[i] = num3;
			asymCy[i] = num4;
			// 1.8.45: the demand is judged at the dynamic pressure predicted from a rapid IAS decay. Load and authority
			// both scale with q and the absolute reserves (AsymRollReserveAccel / AsymYawReserveAccel) do not, so where
			// those bind, and through the fixed deadband everywhere, a fast deceleration brings the cap down ahead of
			// the authority loss. The share of the authority the reserve may claim is AsymResMaxFrac (0.8, as before).
			float kq = qPredK;
			float crK = kq * num3;
			float cyK = kq * num4;
			float num12 = Mathf.Clamp01(Plugin.AsymReserve);
			float tauRes = (Plugin.AsymRollTau > 0.01f) ? (Mathf.Max(0f, pStockRef) / Plugin.AsymRollTau) : 0f;
			float num13 = Mathf.Min(Mathf.Max(num12 * crK, Mathf.Max(tauRes, Plugin.AsymRollReserveAccel)), AsymResMaxFrac * crK);
			float num14 = Mathf.Min(Mathf.Max(num12 * cyK, Plugin.AsymYawReserveAccel), AsymResMaxFrac * cyK);
			float num15 = Mathf.Max(0f, Plugin.AsymDeadband);
			float num16 = 0.25f * num15;
			// 2.0.12: the thrust part of the load (one engine out: tvc.Mnow) does not fall with q the way the aero load
			// and the authority do, so only the aero part is scaled by the predicted q. 2.0.11 scaled both: the engine-out
			// cap was judged at today's IAS however fast the pull was bleeding it (10:37 log: AoA 64.7 deg at 101 m/s before
			// the cap came down, 3 s after the pull).
			float arT = AsymThrustSplit ? AccRoll(vector) : 0f;
			float ayT = AsymThrustSplit ? AccYaw(vector) : 0f;
			// 2.0.13: with an engine out the thrust part is left out of the cap altogether (the cap is the fixed
			// EngineOutAoACap plus whatever an aerodynamic asymmetry needs): the thrust yaw is the throttle's business
			// (VMC protection), and judging it here made the cap follow speed and throttle, 12 <-> 65 deg in seconds
			float tK = (((engOut || engAsym) && EngineOutFixedCap) || tKHoldT > 0f) ? 0f : 1f;
			float arL = kq * (asymAr[i] - arT) + tK * arT;
			float ayL = kq * (asymAy[i] - ayT) + tK * ayT;
			asymU[i] = Mathf.Max(0f, Mathf.Abs(arL) - num15) / Mathf.Max(crK - num13, 1f) + Mathf.Max(0f, Mathf.Abs(ayL) - num16) / Mathf.Max(cyK - num14, 1f);
			if (cp != null)
			{
				Vector3 mLd = cplTv ? (mAir - vector) : (mAir - (1f - tK) * vector);
				cplDnTv[i] = cplTv;
				CplSolve(i, cp, mLd, kq, num12, tauRes, dMbP, dMbN);
				if (CplDebug && cplClock >= CplDbgT2 && Mathf.Abs(asymAlphas[i] - alpha) <= 2.5f) { CplDbgT2 = float.MaxValue; CplDump(i, cp, mLd); }
			}
			else if (cplActive)
			{
				cplU[i] = asymU[i];
				cplRp[i] = (cplRn[i] = 0f);
				cplDn[i] = asymNetDn[i];
				cplUp[i] = asymNetUp[i];
				cplDnTv[i] = false;
			}
		}

		// one nozzle's moment at deflection d from the sweep's copy of its table (bench diagnostic)
		private Vector3 TvNozzleAt(int e, float d)
		{
			if (cplTvN < 2)
			{
				return Vector3.zero;
			}
			float x = Mathf.Clamp((d - cplTvG[0]) / (cplTvG[cplTvN - 1] - cplTvG[0]) * (float)(cplTvN - 1), 0f, (float)(cplTvN - 1) - 0.001f);
			int k = (int)x;
			return Vector3.Lerp(cplTvM[e][k], cplTvM[e][k + 1], x - (float)k);
		}

		// Su-30 0.5.0 (CoupledCap, from the Ternion 0.2.2 / 0.2.3; see CoupledCap.cs). Every piece has a bench switch; CoupledCap = false,
		// AsymThrustCgLoad = false and AsymPitchMarginIasRef = 0 together are 0.4.1.
		internal static bool CoupledCap = true;
		internal static bool CplLateral = true;          // the coupled demand drives the lateral cap (false: only the held-load pitch figure)
		internal static float CplDeadband = 2f;          // deg/s^2 of flight-path roll shortfall ignored (fuel imbalance, rounding)
		internal static int CplPts = 9;                  // grid points per surface the LP takes (ends included, evenly spaced)
		internal static int CplPointsPerStep = 2;        // sweep points per tick while the LP runs (3 without it)
		internal static float CplBetaDesign = 3f;
		internal static bool CplAttachedOnly = true;
		// the share of the flight-path roll range the reserve may claim (the one-axis check: AsymResMaxFrac 0.8): the dev setting
		// AsymCoupledReserveMax
		internal static float CplResMaxFrac => Plugin.AsymCoupledReserveMax;
		private bool cplActive;

		// Su-30 0.5.6 (CplGridCache, from the Ternion 0.3.0): the coupled check's surface points on geometry worked out once per sweep
		// tick (SurfaceEffector.CplGeom) and, with the native kernels, their moments in one call per surface and sweep point. Off =
		// the previous release's per-point evaluation.
		internal static bool CplGridCache = true;

		private bool[] swCpl;

		private readonly List<SurfaceEffector> swCplList = new List<SurfaceEffector>();

		// the LP only where the cap can use it: positive AoA up to 5 deg past the limit, the negative side down to 5 deg past its limit
		private bool CplHere(int i)
		{
			return (asymAlphas[i] >= 0f) ? (asymAlphas[i] <= Plugin.AoAPos + 5f) : (asymAlphas[i] >= Plugin.AoANeg - 5f);
		}
		private bool cplLoaded;
		private bool cplPitchGate;
		// (CplTopEdge) while the coupled curve is in use the cap is the lower edge of the curve's top over-threshold stretch
		// (AsymCap.FindTop), not its first crossing from zero AoA
		internal static bool CplTopEdge = true;
		// Su-30 0.5.2 (CplRefAuth; Tom 11 Nov 18:26: "Dont make it a gate or switch. Make it estimate the remaining useful authority to
		// limit aoa"). The lateral demand of the coupled check was relative: how far the weaker side's flight-path roll had fallen under
		// the mean of the two sides. On this jet the two sides stay within about 20 % of each other up to the cliff past 50 deg (17:22
		// log, 197 kt, 49 deg: 62 / 79 deg/s^2, demand 0.44 against 1.08), so the limit rode to the cliff, where both sides are small:
		// 23 / 34 deg/s^2 at 129 kt with 46 deg/s of roll allowed. (The Ternion, same load and speed: 115 / 182 at its 33 deg limit.)
		// The demand is now an estimate of what is left: the weaker side's reserve with the load held (A, what the coupled check already
		// solves) against what the same airframe has without the load's roll and yaw at the limit the clean jet flies (Aref: the same
		// effectors, the same pitch load, the same design sideslip, solved at the two sweep points around min(AoA limit, authority cap),
		// each side's end of the grid at most):  U(AoA) = Aref / (A(AoA) + CplDeadband).  The limit is where the jet with the load has
		// the roll the clean jet has at its limit. No load: U(limit) < 1, no cap, by construction - nothing switches it. The clean
		// reference of the last finished sweep is used while the next one runs. CplRefAuth = false: the relative demand (0.5.1).
		internal static bool CplRefAuth = true;
		// Su-30 0.5.2 (CplKnee; Tom 11 Nov 18:56: "the roll to the heavy side is very slow, i'd rather have my aoa a bit lower to get more roll.
		// But dont make it pessimistic"). With the load on, the weak side's remaining roll W(AoA) on this jet is flat from about 20 to 47 deg
		// and falls off past 50 (16 cm, 122 kt: 33 / 30 / 32 / 27 deg/s^2 at 20 / 30 / 45 / 50; 20 cm, 117 kt: 26 at 47, 20 at 50), so any
		// limit found by a crossing jumps between the bottom and the top of that flat stretch (bench, 20 cm: 48 -> 20 -> 47 deg in a second).
		// The limit is now the AoA where the usable lateral capacity  C(a) = a^AsymAoAWeight x roll(a)  is largest, with roll(a) the roll the
		// law would allow there: the law's roll limit, held to what the weak side's reserve stops in the clean jet's stopping time at its
		// own limit (law roll / clean reserve there) and in AsymStopTime, faded in with the load as the coupled check is. No load: roll(a) is
		// the law's roll at every AoA and the maximum is the clean limit. A load that eats the reserve: the maximum moves to the knee before
		// the reserve falls off (about 45-47 deg here). It cannot drop to the bottom of the flat stretch: 20 deg would need 2.3x the reserve.
		internal static bool CplKneeOn0 = true;
		public float cplLawRoll;
		private bool CplKneeOn => CplKneeOn0 && CplRefAuth && cplActive && CplLateral && cplLoaded;

		private float CplKnee(int side)
		{
			int n = asymAlphas.Length;
			float lim = (side > 0) ? Mathf.Min(Plugin.AoAPos, Mathf.Max(5f, authCapEff)) : (0f - Mathf.Max(0f - Plugin.AoANeg, 0f - Mathf.Min(-5f, authCapNegEff)));
			float noCap = (side > 0) ? Plugin.AoAPos : Plugin.AoANeg;
			float lAbs = Mathf.Abs(lim);
			float flo = Mathf.Min((side > 0) ? Plugin.AsymCapFloor : Plugin.AsymCapFloorNeg, lAbs);
			float wC = (side > 0) ? cplRefPos : cplRefNeg;
			float pLaw = Mathf.Max(CplStopFloor, cplLawRoll);
			if (wC <= 0f)
			{
				return noCap;
			}
			int iRef = (side > 0) ? cplRefI0p : cplRefI0n;
			float kqR = (iRef >= 0) ? Mathf.Max(0.01f, cplKqA[iRef]) : 1f;
			float tC = pLaw / Mathf.Max(1f, kqR * wC);
			float tA = (Plugin.AsymStopTime > 0f) ? Plugin.AsymStopTime : 999f;
			float tS = Mathf.Min(tC, tA);
			float dyG = Mathf.Max(Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan)), (AsymDamageGate && dmgGateHold > 0f) ? 1f : 0f);
			float wE = Plugin.AsymAoAWeight;
			float bestJ = -1f, bestA = lAbs, wEnd = -1f, aEnd = 0f;
			int nC = 0;
			float[] ja = cplKa, jj = cplKj;
			for (int k = 0; k < n; k++)
			{
				int i = (side > 0) ? k : (n - 1 - k);
				float ai = side * asymAlphas[i];
				if (ai < flo - 0.01f || ai > lAbs + 0.01f)
				{
					continue;
				}
				float u = cplU[i];
				if (u < 0f)
				{
					continue;   // no pitch-held roll here: the nose-down / authority caps own this point
				}
				float w = (u < 8.5f && cplRp[i] > -900f && cplRn[i] > -900f) ? Mathf.Max(0f, cplKqA[i] * Mathf.Min(cplRp[i], cplRn[i])) : 0f;
				float roll = Mathf.Lerp(pLaw, Mathf.Min(pLaw, Mathf.Max(CplStopFloor, tS * w)), dyG);
				float j = Mathf.Pow(Mathf.Max(0.1f, ai), wE) * roll;
				ja[nC] = ai;
				jj[nC] = j;
				nC++;
				wEnd = w;
				aEnd = ai;
			}
			if (nC == 0)
			{
				return noCap;
			}
			// the limit itself: the reserve there taken from the last point the sweep has (the grid ends at +65 / -40)
			if (aEnd < lAbs - 0.01f && wEnd >= 0f)
			{
				float rollL = Mathf.Lerp(pLaw, Mathf.Min(pLaw, Mathf.Max(CplStopFloor, tS * wEnd)), dyG);
				ja[nC] = lAbs;
				jj[nC] = Mathf.Pow(lAbs, wE) * rollL;
				nC++;
			}
			int kB = 0;
			for (int k = 1; k < nC; k++)
			{
				if (jj[k] > jj[kB])
				{
					kB = k;
				}
			}
			bestA = ja[kB];
			if (kB > 0 && kB < nC - 1)
			{
				// the peak between the neighbours (parabola through three points): a smooth limit, not one stepping 5 deg at a time
				float x0 = ja[kB - 1], x1 = ja[kB], x2 = ja[kB + 1];
				float y0 = jj[kB - 1], y1 = jj[kB], y2 = jj[kB + 1];
				float den = (x0 - x1) * (x0 - x2) * (x1 - x2);
				if (Mathf.Abs(den) > 1E-06f)
				{
					float a2 = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / den;
					float b2 = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / den;
					if (a2 < 0f)
					{
						bestA = Mathf.Clamp(-b2 / (2f * a2), x0, x2);
					}
				}
			}
			cplKneeJ = jj[kB];
			if (bestA >= lAbs - 0.25f)
			{
				return noCap;
			}
			return side * Mathf.Max(flo, bestA);
		}
		private readonly float[] cplKa = new float[24];
		private readonly float[] cplKj = new float[24];
		public float cplKneeJ;
		private readonly float[] cplClean = new float[22];
		private readonly float[] cplKqA = new float[22];
		private int cplRefI0p = -1, cplRefI1p = -1, cplRefI0n = -1, cplRefI1n = -1;
		private float cplRefAp, cplRefAn;
		public float cplRefPos, cplRefNeg;

		private float CplRefAt(float a, int i0, int i1)
		{
			if (i0 < 0 || i1 < 0)
			{
				return float.NaN;
			}
			float c0 = cplClean[i0], c1 = cplClean[i1];
			bool ok0 = !float.IsNaN(c0) && c0 > 0f, ok1 = !float.IsNaN(c1) && c1 > 0f;
			if (ok0 && ok1)
			{
				float w = Mathf.Clamp01((a - asymAlphas[i0]) / Mathf.Max(0.01f, asymAlphas[i1] - asymAlphas[i0]));
				return Mathf.Lerp(c0, c1, w);
			}
			// one end of the bracket has no pitch-held roll without the load either: the end nearer the reference AoA that has one
			return ok0 ? c0 : (ok1 ? c1 : float.NaN);
		}

		// at the end of a sweep: the clean reference of this sweep, and every point's demand against it
		private void CplDemand()
		{
			if (!CplRefAuth)
			{
				return;
			}
			float rP = CplRefAt(cplRefAp, cplRefI0p, cplRefI1p);
			float rN = CplRefAt(cplRefAn, cplRefI0n, cplRefI1n);
			if (!float.IsNaN(rP))
			{
				cplRefPos = rP;
			}
			if (!float.IsNaN(rN))
			{
				cplRefNeg = rN;
			}
			for (int i = 0; i < asymAlphas.Length; i++)
			{
				float rf = (asymAlphas[i] >= 0f) ? cplRefPos : cplRefNeg;
				if (rf > 0f && cplRp[i] > -900f && cplRn[i] > -900f && cplU[i] >= 0f && cplU[i] < 8.5f)
				{
					float kq = Mathf.Max(0.01f, cplKqA[i]);
					cplU[i] = kq * rf / Mathf.Max(kq * Mathf.Min(cplRp[i], cplRn[i]) + CplDeadband, 1f);
				}
			}
		}
		// (CplStabPeakRule) the LP's stab and canard points follow the allocator's own attached-side rule (ClampStabToPeak)
		internal static bool CplStabPeakRule = true;
		private bool CplTop => cplActive && CplLateral && cplLoaded && CplTopEdge;
		private bool cplLowPos, cplLowNeg;
		// (CplTvc) the vectoring nozzles as effectors of the LP; the sweep's copy of their tables (AsymSweepPrep, main thread)
		internal static bool CplTvc = true;
		private bool cplTvOn;
		private int cplTvE, cplTvN;
		private float[] cplTvG;
		private readonly Vector3[][] cplTvM = new Vector3[4][];
		private readonly bool[] cplTvHas = new bool[4];
		private readonly float[] cplTvNow = new float[4];
		private readonly bool[] cplDnTv = new bool[22];
		private readonly CplPoint[] cplPts = new CplPoint[22];
		private readonly float[] cplU = new float[22];
		private readonly float[] cplRp = new float[22];
		private readonly float[] cplRn = new float[22];
		private readonly float[] cplDn = new float[22];
		private readonly float[] cplUp = new float[22];
		public float cplUNow, cplRpNow, cplRnNow, cplDnNow;
		// Su-30 0.5.1: the stoppable roll rate each way (see LateralLaw); its floor (deg/s) and its filter (s)
		public float cplStopP = 999f, cplStopN = 999f;
		private float cplStopPF, cplStopNF;
		internal static float CplStopFloor = 10f;
		internal static float CplStopTau = 0.3f;
		private float cplClock;
		internal static float CplDbgT = -1f;             // bench: print one sweep's curve at this time (s)
		// bench diagnostic (run with ParallelSweep=false CplBetaDesign=0): the LP's mix for each side against the real deflections
		internal static bool CplDebug = false;
		internal static float CplDbgT2 = 0f;
		private readonly Vector3[] cplDbgNow = new Vector3[CplPoint.MaxEff];
		private readonly string[] cplDbgName = new string[CplPoint.MaxEff];
		private readonly float[] cplDbgDefl = new float[CplPoint.MaxEff];

		private void CplDump(int i, CplPoint cp, Vector3 mLoad)
		{
			var sb = new System.Text.StringBuilder();
			sb.AppendFormat(CultureInfo.InvariantCulture, "CPLMIX t {0:F2} aPt {1:F0} aNow {2:F1} ias {3:F1} load R {4:F1} Q {5:F1} Y {6:F1} | rp {7:F1} rn {8:F1} u {9:F2} heldDn {10:F1}\n", cplClock, asymAlphas[i], alpha, ias, AccRoll(mLoad), mLoad.y * IaInv[1, 1] * 57.29578f, AccYaw(mLoad), cplRp[i], cplRn[i], cplU[i], cplDn[i]);
			for (int e = 0; e < cp.nEff; e++)
			{
				double gP = 0, gN = 0; float rN = 0, qN = 0, yN = 0, rP = 0, qP = 0, yP = 0;
				for (int j = cp.effStart[e]; j < cp.effStart[e + 1]; j++)
				{
					gP += cp.solP[j] * cp.G[j]; gN += cp.solN[j] * cp.G[j];
					rN += (float)cp.solN[j] * cp.R[j]; qN += (float)cp.solN[j] * cp.Q[j]; yN += (float)cp.solN[j] * cp.Y[j];
					rP += (float)cp.solP[j] * cp.R[j]; qP += (float)cp.solP[j] * cp.Q[j]; yP += (float)cp.solP[j] * cp.Y[j];
				}
				Vector3 nw = cplDbgNow[e];
				sb.AppendFormat(CultureInfo.InvariantCulture, "   {0,-16} now {1,6:F1} (R {2,6:F1} Q {3,6:F1} Y {4,6:F1}) | LP-neg {5,6:F1} (R {6,6:F1} Q {7,6:F1} Y {8,6:F1}) | LP-pos {9,6:F1} (R {10,6:F1} Q {11,6:F1} Y {12,6:F1}) | pts {13}\n", cplDbgName[e] ?? "?", cplDbgDefl[e], nw.x, nw.y, nw.z, gN, rN, qN, yN, gP, rP, qP, yP, cp.effStart[e + 1] - cp.effStart[e]);
			}
			foreach (Effector se in effs)
			{
				sb.AppendFormat(CultureInfo.InvariantCulture, "   BOUNDS {0,-12} now {1,6:F1} cmd {2,6:F1} lo {3,6:F1} hi {4,6:F1}\n", se.name, se.now, se.cmd, se.lo, se.hi);
			}
			Console.Write(sb.ToString());
		}

		private static int CplStride(int n)
		{
			return Mathf.Min(n, Mathf.Max(2, CplPts));
		}

		internal static int CplIdx(int k, int n)
		{
			int m = CplStride(n);
			return (m >= n) ? k : Mathf.RoundToInt((float)k * (float)(n - 1) / (float)(m - 1));
		}

		// a point where the pitch itself cannot be held (no reserve figure) takes the demand of the next point toward zero AoA, so the
		// curve does not read as falling there
		private void CplFill()
		{
			int i0 = Array.IndexOf(asymAlphas, 0f);
			if (i0 < 0)
			{
				return;
			}
			if (cplU[i0] < 0f)
			{
				cplU[i0] = 0f;
			}
			for (int k = i0 + 1; k < asymAlphas.Length; k++)
			{
				if (cplU[k] < 0f)
				{
					cplU[k] = cplU[k - 1];
				}
			}
			for (int k = i0 - 1; k >= 0; k--)
			{
				if (cplU[k] < 0f)
				{
					cplU[k] = cplU[k + 1];
				}
			}
		}

		// point i's flight-path roll reserve each way with the load in, pitch held and no sideslip acceleration, the demand from it, and
		// the held-load pitch figure (the most nose-down / nose-up with the load's roll and yaw held)
		private void CplSolve(int i, CplPoint cp, Vector3 mLoad, float kq, float resShare, float tauRes, Vector3 dMbP, Vector3 dMbN)
		{
			CplPoint.KeepSolution = CplDebug;
			float aDeg = asymAlphas[i];
			float aR = aDeg * (MathF.PI / 180f);
			float sinA = Mathf.Sin(aR);
			float cosA = Mathf.Cos(aR);
			float bR = AccRoll(mLoad);
			float bQ = mLoad.y * IaInv[1, 1] * 57.29578f;
			float bY = AccYaw(mLoad);
			int sgn = (aDeg >= 0f) ? 1 : -1;
			if (cp.overflow || cp.nEff == 0)
			{
				cplU[i] = asymU[i];
				cplRp[i] = (cplRn[i] = 0f);
				cplDn[i] = asymNetDn[i];
				cplUp[i] = asymNetUp[i];
				cplDnTv[i] = false;
				return;
			}
			bool okA;
			float rp, rn;
			Vector3 dAgP = Vector3.zero, dAgN = Vector3.zero;
			if (CplBetaDesign > 0f)
			{
				// each side with the sideslip that rolls against it, and only that side of each (Su-30 0.5.0: the Compass solved the
				// zero-sideslip case too and both sides of all three, 3 phase 1 + 6 phase 2 where 2 + 2 give the same two figures)
				float vvP = cosA * AccRoll(dMbP) + sinA * AccYaw(dMbP);
				float vvN = cosA * AccRoll(dMbN) + sinA * AccYaw(dMbN);
				dAgP = (vvP <= vvN) ? dMbP : dMbN;
				dAgN = (vvP <= vvN) ? dMbN : dMbP;
				bool okP = cp.Reserve(sinA, cosA, bR + AccRoll(dAgP), bQ + dAgP.y * IaInv[1, 1] * 57.29578f, bY + AccYaw(dAgP), sgn, out rp, out float _, 1);
				bool okN = cp.Reserve(sinA, cosA, bR + AccRoll(dAgN), bQ + dAgN.y * IaInv[1, 1] * 57.29578f, bY + AccYaw(dAgN), sgn, out float _, out rn, -1);
				okA = okP && okN;
			}
			else
			{
				okA = cp.Reserve(sinA, cosA, bR, bQ, bY, sgn, out rp, out rn);
			}
			cplRp[i] = okA ? rp : -999f;
			cplRn[i] = okA ? rn : -999f;
			cplKqA[i] = kq;
			if (CplRefAuth && (i == cplRefI0p || i == cplRefI1p || i == cplRefI0n || i == cplRefI1n))
			{
				// Su-30 0.5.2: the same point without the load's roll and yaw (each side with its adverse design sideslip); the mean of the
				// two sides (they differ only by the effectors' arms about the offset CG)
				float cA = float.NaN;
				if (CplBetaDesign > 0f)
				{
					bool c1 = cp.Reserve(sinA, cosA, AccRoll(dAgP), bQ + dAgP.y * IaInv[1, 1] * 57.29578f, AccYaw(dAgP), sgn, out float cRp, out float _, 1);
					bool c2 = cp.Reserve(sinA, cosA, AccRoll(dAgN), bQ + dAgN.y * IaInv[1, 1] * 57.29578f, AccYaw(dAgN), sgn, out float _, out float cRn, -1);
					if (c1 && c2)
					{
						cA = 0.5f * (cRp + cRn);
					}
				}
				else if (cp.Reserve(sinA, cosA, 0f, bQ, 0f, sgn, out float cRp0, out float cRn0))
				{
					cA = 0.5f * (cRp0 + cRn0);
				}
				cplClean[i] = cA;
			}
			if (!okA)
			{
				// no pitch-held, sideslip-free roll either way: if the lateral problem itself is feasible, it is the pitch that cannot
				// be held here, which the nose-down / authority caps own; if not, the load cannot be held at all
				cplU[i] = cp.LateralFeasible(sinA, cosA, bR, bY) ? -1f : 9f;   // -1: filled from the point below (CplFill)
			}
			else
			{
				float rc = 0.5f * (rp + rn);
				float rMin = Mathf.Min(rp, rn);
				float rcK = kq * Mathf.Max(rc, 0f);
				float req = Mathf.Min(Mathf.Max(Mathf.Clamp01(resShare) * rcK, Mathf.Max(tauRes, Plugin.AsymRollReserveAccel)), CplResMaxFrac * rcK);
				cplU[i] = Mathf.Max(0f, kq * (rc - rMin) - CplDeadband) / Mathf.Max(rcK - req, 1f);
				float rfA = (aDeg >= 0f) ? cplRefPos : cplRefNeg;
				if (CplRefAuth && rfA > 0f)
				{
					cplU[i] = kq * rfA / Mathf.Max(kq * rMin + CplDeadband, 1f);
				}
			}
			if (!cplPitchGate)
			{
				cplDn[i] = asymNetDn[i];
				cplUp[i] = asymNetUp[i];
				cplDnTv[i] = false;
			}
			else if (sgn > 0)
			{
				float qd = cp.PitchHeld(-1, bR, bQ, bY);
				cplDn[i] = float.IsNaN(qd) ? 999f : qd;
				cplUp[i] = asymNetUp[i];
			}
			else
			{
				float qu = cp.PitchHeld(1, bR, bQ, bY);
				cplUp[i] = float.IsNaN(qu) ? -999f : qu;
				cplDn[i] = asymNetDn[i];
			}
		}

		private float NonSurfacePitch(Vector3 wind, float rho, Quaternion rootInv, Vector3 rootRight, float dAlphaDeg, float qRad)
		{
			Quaternion rot = Quaternion.AngleAxis(dAlphaDeg, rootRight);
			Vector3 w = -rootRight * qRad;
			Vector3 m = Vector3.zero;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached() || surfaceParts.Contains(aeroPart))
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				float area = Acc.AP_wingArea(aeroPart);
				if (transform == null || area <= 0f || !foils.TryGetValue(aeroPart, out var foil))
				{
					continue;
				}
				Quaternion rotation = AirframeRot(aeroPart, transform);
				Vector3 arm = aeroPart.rb.worldCenterOfMass - cg;
				Vector3 r = arm + rotation * Acc.AP_centerOfLift(aeroPart);
				// flow at the part: the CG's air velocity turned by dAlpha, plus the pitch rotation qRad
				Vector3 v = rot * (vCgAir) + Vector3.Cross(w, arm);
				m += Vector3.Cross(r, PartForce(v, rotation, rotation * Vector3.right, foil, 0.5f * rho * area * Acc.AP_wingEff(aeroPart)));
			}
			return 0f - (rootInv * m).x;
		}

		private Vector3 vCgAir;

		// Su-30 0.2.0 (from F-22E 3.1.3, ParallelEnvelope): the four NonSurfacePitch evaluations of the hybrid slopes on the parts' state
		// read once (HsGather, main thread); HsPoint(k) = NonSurfacePitch with the k-th arguments, same expressions in the same part order.
		private void HsGather(Vector3 wind, float rho, Vector3 rootRight)
		{
			if (surfaceParts.Count == 0)
			{
				foreach (SurfaceEffector se in surfaces)
				{
					if (se.part != null)
					{
						surfaceParts.Add(se.part);
					}
				}
			}
			hsRootInv = Quaternion.Inverse(((Component)(object)ac).transform.rotation);
			vCgAir = ((AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : ac.rb.velocity) - wind;
			float q0 = omega.y;
			hsDA[0] = 1f;
			hsQR[0] = q0;
			hsDA[1] = -1f;
			hsQR[1] = q0;
			hsDA[2] = 0f;
			hsQR[2] = q0 + 0.05f;
			hsDA[3] = 0f;
			hsQR[3] = q0 - 0.05f;
			hsRootRight = rootRight;
			int cap = aeroParts.Count;
			if (hsArm.Length < cap)
			{
				hsArm = new Vector3[cap];
				hsR = new Vector3[cap];
				hsRot = new Quaternion[cap];
				hsFoil = new AirfoilTable[cap];
				hsQS = new float[cap];
			}
			hsN = 0;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached() || surfaceParts.Contains(aeroPart))
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				float area = Acc.AP_wingArea(aeroPart);
				if (transform == null || area <= 0f || !foils.TryGetValue(aeroPart, out var foil))
				{
					continue;
				}
				Quaternion rotation = AirframeRot(aeroPart, transform);
				Vector3 arm = aeroPart.rb.worldCenterOfMass - cg;
				int k = hsN++;
				hsRot[k] = rotation;
				hsArm[k] = arm;
				hsR[k] = arm + rotation * Acc.AP_centerOfLift(aeroPart);
				hsFoil[k] = foil;
				hsQS[k] = 0.5f * rho * area * Acc.AP_wingEff(aeroPart);
			}
		}

		private void HsPoint(int c)
		{
			Quaternion rot = Quaternion.AngleAxis(hsDA[c], hsRootRight);
			Vector3 w = -hsRootRight * hsQR[c];
			Vector3 m = Vector3.zero;
			for (int k = 0; k < hsN; k++)
			{
				Quaternion rotation = hsRot[k];
				// flow at the part: the CG's air velocity turned by dAlpha, plus the pitch rotation qRad
				Vector3 v = rot * (vCgAir) + Vector3.Cross(w, hsArm[k]);
				m += Vector3.Cross(hsR[k], PartForce(v, rotation, rotation * Vector3.right, hsFoil[k], hsQS[k]));
			}
			hsOut[c] = 0f - (hsRootInv * m).x;
		}

		private void HsFinish()
		{
			mAlphaO = 0.5f * (hsOut[0] - hsOut[1]);
			mQO = (hsOut[2] - hsOut[3]) / (0.1f * 57.29578f);
		}

		private Quaternion hsRootInv;
		private Vector3 hsRootRight;
		private readonly float[] hsDA = new float[4];
		private readonly float[] hsQR = new float[4];
		private readonly float[] hsOut = new float[4];
		private int hsN;
		private Vector3[] hsArm = new Vector3[0];
		private Vector3[] hsR = new Vector3[0];
		private Quaternion[] hsRot = new Quaternion[0];
		private AirfoilTable[] hsFoil = new AirfoilTable[0];
		private float[] hsQS = new float[0];

		// the envelope's per-part snapshot (ComputeEnvelope) and per-point rotations / downwash
		private int enN;
		private Vector3[] enVel = new Vector3[0];
		private Quaternion[] enRot = new Quaternion[0];
		private Vector3[] enRight = new Vector3[0];
		private AirfoilTable[] enFoil = new AirfoilTable[0];
		private float[] enQS = new float[0];
		private bool[] enStab = new bool[0];
		private readonly Quaternion[] enQ = new Quaternion[envAlphas.Length];
		private readonly float[] enEps = new float[envAlphas.Length];
		private Quaternion enRoot;
		private Vector3 enEL;

		// one envelope point: envLift[j] summed over the parts in the same order as the part-major loop it replaces
		private void EnvPoint(int j)
		{
			Quaternion quaternion2 = enQ[j];
			for (int p = 0; p < enN; p++)
			{
				Vector3 vj = quaternion2 * enVel[p];
				if (enStab[p])
				{
					vj = Downwash.Turn(vj, enRoot, enEps[j]);   // = Downwash.Apply(vj, root, quaternion2 * (v_ac - wind)): eps depends on j only
				}
				envLift[j] += PartLift(vj, enRot[p], enRight[p], enFoil[p], enQS[p], enEL);
			}
		}

		private sealed class EnvWork : IIndexWork
		{
			private readonly FcsController c;

			public EnvWork(FcsController c)
			{
				this.c = c;
			}

			public void Do(int i)
			{
				if (i < envAlphas.Length)
				{
					c.EnvPoint(i);
				}
				else
				{
					c.HsPoint(i - envAlphas.Length);
				}
			}
		}

		private EnvWork envWork;

		private void ComputeEnvelope(Vector3 wind, float rho, Vector3 rootRight, Vector3 eL)
		{
			for (int i = 0; i < envLift.Length; i++)
			{
				envLift[i] = 0f;
			}
			Quaternion quaternion = Quaternion.Inverse(((Component)(object)ac).transform.rotation);
			Vector3 lhs = -((Component)(object)ac).transform.forward * 0.2f;
			float num = 0f;
			float num2 = 0f;
			float num3 = mass * 9.81f;
			Vector3 vCgHat = (((AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : ac.rb.velocity) - wind).normalized;
			float dragAf = 0f;
			// Su-30 0.2.0 (F-22E 3.1.3): the 25 points' per-part inputs are kept (enVel ... enStab) and the points computed after the loop (EnvPoint)
			int cap = aeroParts.Count;
			if (enVel.Length < cap)
			{
				enVel = new Vector3[cap];
				enRot = new Quaternion[cap];
				enRight = new Vector3[cap];
				enFoil = new AirfoilTable[cap];
				enQS = new float[cap];
				enStab = new bool[cap];
			}
			enN = 0;
			bool anyStab = false;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null)
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				float num4 = Acc.AP_wingArea(aeroPart);
				if (transform == null || aeroPart.IsDetached() || num4 <= 0f || !foils.TryGetValue(aeroPart, out var value))
				{
					continue;
				}
				Vector3 vector = aeroPart.rb.velocity - wind;
				Vector3 vRaw = vector;
				float sqrMagnitude = vector.sqrMagnitude;
				bool dwPart = Downwash.IsStab(aeroPart);
				if (dwPart)
				{
					vector = Downwash.Apply(vector, ((Component)(object)ac).transform.rotation, ac.rb.velocity - wind);
				}
				if (!(sqrMagnitude < 25f))
				{
					float qS = 0.5f * rho * sqrMagnitude * num4 * Acc.AP_wingEff(aeroPart);
					Quaternion rotation = AirframeRot(aeroPart, transform);
					Vector3 right = rotation * Vector3.right;
					Vector3 lhs2 = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					Vector3 v = vector + Vector3.Cross(lhs, aeroPart.rb.worldCenterOfMass - cg);
					num += 0f - (quaternion * Vector3.Cross(lhs2, PartForce(vector, rotation, right, value, 0.5f * rho * num4 * Acc.AP_wingEff(aeroPart)))).z;
					num2 += 0f - (quaternion * Vector3.Cross(lhs2, PartForce(v, rotation, right, value, 0.5f * rho * num4 * Acc.AP_wingEff(aeroPart)))).z;
					// 2.2.16: airframe drag along the flight path (airfoil + parasitic, AeroJob_Math terms), for the brake's share
					float effD = Acc.AP_wingEff(aeroPart);
					float dArea = Acc.AP_dragArea(aeroPart);
					dragAf += 0f - Vector3.Dot(PartForce(vector, rotation, right, value, 0.5f * rho * num4 * effD), vCgHat) + 0.25f * rho * sqrMagnitude * (dArea + (dArea + num4 * 0.1f) * (1f - effD));
					int p = enN++;
					enVel[p] = vRaw;
					enRot[p] = rotation;
					enRight[p] = right;
					enFoil[p] = value;
					enQS[p] = qS;
					enStab[p] = dwPart;
					anyStab |= dwPart;
				}
			}
			enRoot = ((Component)(object)ac).transform.rotation;
			enEL = eL;
			Vector3 vAc = ac.rb.velocity - wind;
			for (int j = 0; j < envAlphas.Length; j++)
			{
				enQ[j] = Quaternion.AngleAxis((j < 3) ? envAlphas[j] : (envAlphas[j] - alpha), rootRight);
				enEps[j] = anyStab ? Downwash.EpsFor(enQ[j] * vAc, enRoot) : 0f;
			}
			HsGather(wind, rho, rootRight);
			if (Plugin.ParallelEnvelope)
			{
				if (envWork == null)
				{
					envWork = new EnvWork(this);
				}
				IndexFork.Finish(IndexFork.Start(envWork, envAlphas.Length + 4));
			}
			else
			{
				for (int j = 0; j < envAlphas.Length; j++)
				{
					EnvPoint(j);
				}
				for (int c = 0; c < 4; c++)
				{
					HsPoint(c);
				}
			}
			HsFinish();
			// 2.2.16 (AirbrakeAoAFade): the brake's drag (game Airbrake.FixedUpdate: dragAmount x rho x v^2 x opening, on each
			// carrying part) against the airframe's own. Full open while the brake is AirbrakeFadeShareFull of the total drag or
			// more, stowed at AirbrakeFadeShareZero or less. Clean jet, bench: share 81 % at 0 deg AoA, 56 % at 10, 42 % at 15,
			// 32 % at 20, 19 % at 30, 12 % at 40, 9 % at 50.
			float dragAb = 0f;
			for (int iA = 0; iA < abRb.Count; iA++)
			{
				if (abRb[iA] != null && !(abPart[iA] != null && abPart[iA].IsDetached()))
				{
					dragAb += abDrag[iA] * rho * (abRb[iA].velocity - wind).sqrMagnitude;
				}
			}
			airbrakeShare = (dragAb > 0f) ? (dragAb / (dragAb + Mathf.Max(0f, dragAf))) : 1f;
			float abTgt = Mathf.Clamp01((airbrakeShare - Plugin.AirbrakeFadeShareZero) / Mathf.Max(0.01f, Plugin.AirbrakeFadeShareFull - Plugin.AirbrakeFadeShareZero));
			airbrakeFade = Mathf.MoveTowards(airbrakeFade, abTgt, 0.2f);
			nAlpha = Mathf.Clamp((envLift[2] - envLift[0]) * 0.5f / num3, 0.005f, 12f);
			Lp = Mathf.Min((num2 - num) / 0.2f, -1f);
			float num5 = float.MinValue;
			float num6 = 30f;
			for (int k = 3; k < EnvNegStart; k++)
			{
				float num7 = envLift[k] / num3 + tw * Mathf.Sin(envAlphas[k] * (MathF.PI / 180f));
				if (num7 > num5)
				{
					num5 = num7;
					num6 = envAlphas[k];
				}
			}
			nMaxAero = Mathf.Max(1f, num5);
			float nl = 0f;
			for (int kl = 3; kl < EnvNegStart; kl++)
			{
				nl = Mathf.Max(nl, envLift[kl] / num3);
			}
			nLiftMax = nl;
			float nlN = 0f;
			for (int kn = EnvNegStart; kn < envAlphas.Length; kn++)
			{
				nlN = Mathf.Max(nlN, (0f - envLift[kn]) / num3);
			}
			nLiftMin = nlN;
			alphaAtNMax = num6;
		}

		private static Vector3 PartForce(Vector3 v, Quaternion lr, Vector3 right, AirfoilTable t, float qSr)
		{
			float sqrMagnitude = v.sqrMagnitude;
			if (sqrMagnitude < 1f)
			{
				return Vector3.zero;
			}
			Vector3 vector = Quaternion.Inverse(lr) * v;
			t.Get(Mathf.Atan2(vector.y, vector.z), out var CL, out var CD);
			Vector3 vector2 = Vector3.Cross(v, right);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			float num = qSr * sqrMagnitude;
			return -vector2 / magnitude * (CL * num) - v / Mathf.Sqrt(sqrMagnitude) * (CD * num);
		}

		private float StockRollRef()
		{
			float num = 0.01868f * tas;
			float b = tas * tas * Mathf.Max(0.01f, ac.airDensity) / 24010f;
			float num2 = 1f / Mathf.Max(1f, b);
			float num3 = Mathf.Min(num, num * (1f + 4.5f * num2) / (1f + 1.1f * num2 * num)) * 57.29578f;
			float num4 = Mathf.Lerp(Plugin.RollGainLowIAS, Plugin.RollGainHighIAS, Mathf.InverseLerp(120f, 170f, ias));
			return num3 * num4;
		}

		private float NPath(float a)
		{
			float W = mass * 9.81f;
			if (a <= 5f)
			{
				return Nk(3) + nAlpha * (a - 5f);
			}
			if (a >= 65f)
			{
				return Nk(15);
			}
			float num = (a - 5f) / 5f;
			int num2 = Mathf.Clamp((int)num, 0, 11);
			float num3 = num - (float)num2;
			return Nk(3 + num2) + (Nk(4 + num2) - Nk(3 + num2)) * num3;
			float Nk(int k)
			{
				return envLift[k] / W + tw * Mathf.Sin(envAlphas[k] * (MathF.PI / 180f));
			}
		}

		private static float PartLift(Vector3 v, Quaternion lr, Vector3 right, AirfoilTable t, float qS, Vector3 eL)
		{
			Vector3 vector = Quaternion.Inverse(lr) * v;
			float a = Mathf.Atan2(vector.y, vector.z);
			t.Get(a, out var CL, out var _);
			Vector3 vector2 = Vector3.Cross(v, right);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return 0f;
			}
			return Vector3.Dot(-vector2 / magnitude * (CL * qS), eL);
		}

		// Ifrit 0.2.1: the pilot's throttle as it came into the FCS this frame (the stock airbrake rule needs it)
		public float rawThrottle = 1f;

		// Ifrit 0.2.1: whether the in-flight speedbrake is commanded out now (brake held, or with
		// ThrottleIndependentAirbrake off, throttle at zero too), in the air and on the ground.
		public bool AirbrakeOut
		{
			get
			{
				// Ifrit 0.2.1: the split decelerons are the airbrake in the air and on the ground (the stock Ifrit's behaviour)
				return airBrake > 0.01f;
			}
		}

		public void Tick(ControlInputs inp, bool flightAssist)
		{
			mkDone = false;   // F-22E 4.1.3 port: the tick's segments
			mk[0] = System.Diagnostics.Stopwatch.GetTimestamp();
			// Su-30 0.2.0 (from F-22E 3.0.4): collect last tick's estimator job before anything reads capEst, then finish last tick's CSV row
			estJobF?.Join();
			FlushPendingRow();
			mk[1] = System.Diagnostics.Stopwatch.GetTimestamp();
			// F-22E 3.1.0: per-phase timer for this tick, and the main allocator's switches
			phRebuild = phKick = phRow = 0L;
			phLaws = phAsym = phAsymMom = phSlip = phSweep = phEnv = phMass = phFlaps = phGround = 0L;
			allocTicks0 = alloc.solveTicks;
			alloc.timeSolves = true;
			alloc.parallelReach = Plugin.ParallelAllocator;
			OwnCanards(true);   // Su-30 0.1.0
			rawThrottle = (inp != null) ? inp.throttle : 1f;
			float num = Mathf.Max(0.001f, Time.fixedDeltaTime);
			time += num;
			Rigidbody rb = ac.rb;
			Transform transform = ((Component)(object)ac).transform;
			if (rb == null || transform == null)
			{
				return;
			}
			Quaternion rotation = transform.rotation;
			Quaternion quaternion = Quaternion.Inverse(rotation);
			massTimer += num;
			bool flag = massTimer > 0.5f;
			long tMass0 = System.Diagnostics.Stopwatch.GetTimestamp();
			if (flag)
			{
				massTimer = 0f;
				RebuildBodies();
			}
			UpdateMassProps(flag || !havePrev, quaternion);
			phMass += System.Diagnostics.Stopwatch.GetTimestamp() - tMass0;
			mk[2] = System.Diagnostics.Stopwatch.GetTimestamp();
			Vector3 vector = Acc.AC_wind(ac);
			Vector3 vRef = (AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : rb.velocity;
			Vector3 vector2 = vRef - vector;
			tas = vector2.magnitude;
			float num2 = Mathf.Max(0.01f, ac.airDensity);
			ias = tas * Mathf.Sqrt(num2 / 1.225f);
			Vector3 vector3 = quaternion * vector2;
			alpha = ((tas > 1f) ? (Mathf.Atan2(0f - vector3.y, vector3.z) * 57.29578f) : 0f);
			beta = ((tas > 1f) ? (Mathf.Asin(Mathf.Clamp(vector3.x / tas, -1f, 1f)) * 57.29578f) : 0f);
			{
				Vector3 vRootB = quaternion * (rb.velocity - vector);
				float tasR = vRootB.magnitude;
				alphaRoot = (tasR > 1f) ? (Mathf.Atan2(0f - vRootB.y, vRootB.z) * 57.29578f) : 0f;
				betaRoot = (tasR > 1f) ? (Mathf.Asin(Mathf.Clamp(vRootB.x / tasR, -1f, 1f)) * 57.29578f) : 0f;
				Vector3 vCgB = quaternion * (((vCg.sqrMagnitude > 0f) ? vCg : rb.velocity) - vector);
				float tasC = vCgB.magnitude;
				alphaCgDbg = (tasC > 1f) ? (Mathf.Atan2(0f - vCgB.y, vCgB.z) * 57.29578f) : 0f;
				betaCgDbg = (tasC > 1f) ? (Mathf.Asin(Mathf.Clamp(vCgB.x / tasC, -1f, 1f)) * 57.29578f) : 0f;
			}
			Vector3 vector4 = quaternion * rb.angularVelocity;
			omega = new Vector3(0f - vector4.z, 0f - vector4.x, vector4.y);
			Vector3 x = (havePrev ? ((rb.velocity - velPrev) / num) : Vector3.zero);
			Vector3 vector5 = accF.Step(x, 0.05f, num);
			Vector3 xCg = (havePrev ? ((vRef - vCgPrev) / num) : Vector3.zero);
			Vector3 accCg = accCgF.Step(xCg, 0.05f, num);
			if (!FlightPathAtCg)
			{
				accCg = vector5;
			}
			Vector3 lhs = vector5 - Physics.gravity;
			nz = Vector3.Dot(lhs, transform.up) / 9.81f;
			nxDbg = Vector3.Dot(lhs, transform.forward) / 9.81f;   // 0.4.6 telemetry: body-axis specific force along the nose (g)
			Vector3 vector6 = ((tas > 1f) ? Vector3.Cross(vector2, transform.right).normalized : transform.up);
			eLy = vector6.y;
			float vc = Mathf.Max(tas, 30f);
			omegaV = Vector3.Dot(accCg, vector6) / Mathf.Max(tas, 5f) * 57.29578f;
			vDot = vDotF.Step((tas > 1f) ? Vector3.Dot(accCg, vector2 / tas) : 0f, 0.2f, num);
			theta = Mathf.Asin(Mathf.Clamp(transform.forward.y, -1f, 1f)) * 57.29578f;
			phi = Mathf.Atan2(0f - transform.right.y, transform.up.y) * 57.29578f;
			float x2 = (havePrev ? (Mathf.DeltaAngle(alphaPrev, alpha) / num) : 0f);
			alphaDot = alphaDotF.Step(x2, 0.05f, num);
			alphaPrev = alpha;
			alphaU = ((Mathf.Abs(alpha) < 90f || !havePrev) ? alpha : Mathf.Clamp(alphaU + Mathf.DeltaAngle(alphaU, alpha), -300f, 300f));
			bool flag2 = false;
			foreach (LandingGear gear in gears)
			{
				if (gear != null && gear.WeightOnWheel(0.05f))
				{
					flag2 = true;
					break;
				}
			}
			long tFl0 = System.Diagnostics.Stopwatch.GetTimestamp();
			mk[3] = tFl0;
			flaps.gameFlapSchedule = Plugin.FlaperonFlapSchedule;
			flaps.Update(num, this, flag2 && tas < 110f, ac.speed, Mathf.Clamp(inp.roll, -1f, 1f), groundBrake);
			phFlaps += System.Diagnostics.Stopwatch.GetTimestamp() - tFl0;
			mk[4] = System.Diagnostics.Stopwatch.GetTimestamp();
			// Ifrit 0.3.0 (IfritFlaperons): the decelerons (inner) and ailerons (outer) droop on the F-22E flap schedule (AoA,
			// load and the high-lift speed schedule, stall-capped on each surface's own section flow). Each left/right pair's
			// common deflection is held at the droop through its pair cost (symCost, as for rudder toe); the differential
			// stays free for roll. TE-down is -teUp in the effector's deflection sign.
			foreach (SurfaceEffector sF in surfaces)
			{
				if (sF.partner != null && (sF.kind == SurfaceKind.Flaperon || sF.kind == SurfaceKind.Aileron))
				{
					float droop = (sF.kind == SurfaceKind.Flaperon) ? flaps.innerDroop : flaps.outerDroop;
					sF.symTarget = (sF.teUp != 0f) ? (0f - sF.teUp * droop) : 0f;
				}
			}
			// Revoker 0.2.0: the inboard elevons are drawn toward FlapHighLiftDroop x the HLD position (default 0: a canard-delta
			// has no flap schedule); no manoeuvre droop
			// Compass 0.1.5 (FlapRollBias): roll first, as the F-22E's flaperons. In the air the flaps' droop comes off with the lateral
			// demand (stick or the roll command still running), so a rolling flap pair has its full +-FlapHingeRange differential: at
			// the 20 deg droop the down-going flap sat on its stop and the pair rolled on the up-going one only (5 Nov 18:12 log,
			// full-stick rolls: flap differential median 3 deg against 13 for the ailerons, common droop 12-16). Out fast
			// (FlapRollRetractRate), back slowly (FlapRollRestoreRate); on the wheels the droop stays.
			float latDem = Mathf.Max(Mathf.Abs(Mathf.Clamp(inp.roll, -1f, 1f)), Mathf.Clamp01(Mathf.Abs(pTraj.x) / Mathf.Max(30f, pMaxDbg)));
			float rbTgt = (Plugin.FlapRollBias > 0f && !flag2 && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? Mathf.Clamp01(Plugin.FlapRollBias * Mathf.Clamp01((latDem - FlapRollDem0) / Mathf.Max(0.05f, FlapRollDem1 - FlapRollDem0))) : 0f;
			flapRollBias = Mathf.MoveTowards(flapRollBias, rbTgt, ((rbTgt > flapRollBias) ? FlapRollRetractRate : FlapRollRestoreRate) * num);
			foreach (HingeFlapEffector hS in hinges)
			{
				hS.symTarget = (hS.teUp != 0f) ? ((0f - hS.teUp) * Plugin.FlapHighLiftDroop * flaps.hlPos * (1f - Mathf.Max(flapRollBias, flapDiffYield))) : 0f;
			}
			// Revoker 0.2.8 (TrailingEdgeBrakeConfig): in the air, with the airbrake out, the elevon pairs' common deflections are
			// drawn to the F-22E's speedbrake set: inboard trailing edge down (its flaperons), outboard up (its ailerons). Their
			// lift and pitch largely cancel (the canards trim what is left); the differentials stay free for roll. Scaled by the
			// airbrake's AoA fade, off on the wheels (the rollout aero-brake takes over there).
			teBrakeAir = (TeBrakeOn && !flag2 && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (airBrake * (Plugin.TeBrakeAoAFade ? teFade : 1f)) : 0f;
			// Revoker 0.2.9: as fast as the actuators move it (the inboard elevons' full set at their servo rate; 0.2.8: 0.5 /s)
			float teRate = (TeBrakeAirRate > 0f) ? TeBrakeAirRate : (((hinges.Count > 0) ? Mathf.Max(1f, hinges[0].servo) : ((Plugin.SurfaceServoSpeed > 0f) ? Plugin.SurfaceServoSpeed : 70f)) / Mathf.Max(1f, Mathf.Max(Plugin.AirBrakeInboardDown, TeBrakeOutMax)));
			teBrakeAirF = Mathf.MoveTowards(teBrakeAirF, teBrakeAir, teRate * num);
			// the derotation after a braked touchdown (derotGate, last frame's): the elevons go to the rollout's trailing-edge-up
			// set already, held through the pair costs, and the rate law flies the nose down with the canards
			teBrakeLandF = Mathf.MoveTowards(teBrakeLandF, (derotGate && Mode == FcsMode.GearDownRate) ? 1f : 0f, 1.5f * num);
			if (teBrakeAirF > 0.001f || teBrakeLandF > 0.001f)
			{
				Transform rootB = ((Component)(object)ac).transform;
				foreach (SurfaceEffector sB in surfaces)
				{
					float tuB = sB.GeoTeUp(rootB);
					if (sB.partner != null && sB.kind == SurfaceKind.Aileron && tuB != 0f)
					{
						sB.symTarget += tuB * (((TeBrakeLiftNeutral && Plugin.AirBrakeElevonUp > 0f) ? 0f : (Plugin.AirBrakeElevonUp * teBrakeAirF)) + Plugin.GroundBrakeFlap * teBrakeLandF);
					}
					// Ternion 0.1.0: the FS-3's inboard trailing edge (flaperon_L/R) is a control surface, not an HLD hinge: it takes
					// the inboard share of the set (trailing edge down in the air, up in the rollout)
					if (sB.partner != null && sB.kind == SurfaceKind.Flaperon && tuB != 0f)
					{
						sB.symTarget += tuB * (Plugin.GroundBrakeInboard * teBrakeLandF - Plugin.AirBrakeInboardDown * teBrakeAirF);
					}
				}
				foreach (HingeFlapEffector hB in hinges)
				{
					float tuH = hB.GeoTeUp(rootB);
					if (tuH != 0f)
					{
						hB.symTarget += tuH * (Plugin.GroundBrakeInboard * teBrakeLandF - Plugin.AirBrakeInboardDown * teBrakeAirF);
					}
				}
			}
			rudderAuth = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.RudderFadeStart) / Mathf.Max(1f, Plugin.RudderFadeEnd - Plugin.RudderFadeStart));
			if (RudderRevBack)
			{
				rudderAuth = Mathf.Max(rudderAuth, Mathf.InverseLerp(120f, 140f, Mathf.Abs(alpha)));
			}
			foreach (SurfaceEffector surface in surfaces)
			{
				float num3 = ((surface.kind == SurfaceKind.Rudder) ? rudderAuth : 1f);
				surface.lo = surface.min * num3;
				surface.hi = surface.max * num3;
				if (StabPeakClamp && surface.kind == SurfaceKind.Stabilator)
				{
					ClampStabToPeak(surface);
				}
			}
			SideslipRudder(num);
			EvalContext ctx = new EvalContext
			{
				cg = cg,
				rootInv = quaternion,
				rho = num2,
				wind = vector,
				vAir = ac.rb.velocity - vector
			};
			// Ifrit 0.2.2: split effectors in retraction coordinates while the brake is out; the pair cost keeps the brake
			// symmetric (common retraction costs SplitBrakeHoldCost) and leaves one-sided retraction free for yaw
			splitBrakeMode = SplitDeploy;
			SplitEffector spL = null;
			SplitEffector spR = null;
			foreach (SplitEffector sm in splits)
			{
				sm.SetBrakeMode(splitBrakeMode);
				sm.partner = null;
				if (sm.name.EndsWith("_L"))
				{
					spL = sm;
				}
				else
				{
					spR = sm;
				}
			}
			if (splitBrakeMode && spL != null && spR != null)
			{
				spL.partner = spR;
				spL.pairSign = 1f;
				spR.pairSign = 1f;
				spL.symCost = 0f;
				spL.symTarget = 0f;
				spL.antiCost = Plugin.SplitBrakeHoldCost;
			}
			Vector3 zero = Vector3.zero;
			RollPitchGateStep(num);
			long tReb0 = System.Diagnostics.Stopwatch.GetTimestamp();
			mk[5] = tReb0;
			if (Plugin.ParallelTables)
			{
				// Su-30 0.2.0 (from F-22E 3.1.0, ParallelTables): the surfaces' game reads here (EvalGather), their table maths on the pool
				// and this thread (EvalCompute), the other effectors' Evaluate here meanwhile (they read the engines' and parts' transforms
				// throughout); then the same sum, in the same order, as below
				parSurf.Clear();
				foreach (Effector eff in effs)
				{
					if (eff is SurfaceEffector seH)
					{
						seH.honestGate = Mathf.Max(Plugin.OvershootPitchPriority ? pitchNeed : 0f, HonestRollGate * rpLat);
						if (seH.EvalGather(ref ctx))
						{
							parSurf.Add(seH);
						}
					}
				}
				TableFork.Run tRun = TableFork.Start(parSurf, ctx);
				foreach (Effector eff in effs)
				{
					if (!(eff is SurfaceEffector))
					{
						eff.Evaluate(ref ctx);
					}
				}
				TableFork.Finish(tRun);
				foreach (Effector eff in effs)
				{
					if (eff.active)
					{
						zero += eff.Mnow;
						if (!Engaged || !havePrev)
						{
							eff.cmd = eff.now;
							eff.cmdOut = eff.now;
							eff.cmdLag = eff.now;
							eff.outTraj.Reset(eff.now);
						}
					}
				}
			}
			else
			{
				foreach (Effector eff in effs)
				{
					if (eff is SurfaceEffector seH)
					{
						seH.honestGate = Mathf.Max(Plugin.OvershootPitchPriority ? pitchNeed : 0f, HonestRollGate * rpLat);
					}
					eff.Evaluate(ref ctx);
					if (eff.active)
					{
						zero += eff.Mnow;
						if (!Engaged || !havePrev)
						{
							eff.cmd = eff.now;
							eff.cmdOut = eff.now;
							eff.cmdLag = eff.now;
							eff.outTraj.Reset(eff.now);
						}
					}
				}
			}
			phRebuild += System.Diagnostics.Stopwatch.GetTimestamp() - tReb0;
			mk[6] = System.Diagnostics.Stopwatch.GetTimestamp();
			TeBrakeFadeStep(num);
			FlapLockStep();
			TeBrakeLiftBalance();
			// Ifrit 0.2.2: airbrake. The game holds the splits at full travel while the brake is out (SurfacePostfix feeds the job
			// zero throttle); 0.2.1 modelled them as still under the allocator's control, so each frame it 'retracted' splits
			// that never moved to meet part of the pitch target. The INDI increment went into that phantom, the stabs made only
			// the rest, and the brake's nose-down drag moment (flaps 2.5 m aft and 0.15 m below the CG, ~q) left a steady
			// pitch-rate error instead of being integrated out (bench, neutral stick, brake held: q -0.9 / -1.6 / -2.6 deg/s at
			// 400 / 700 / 1000 km/h; pedal with the brake out did nothing). The split effectors are now in retraction
			// coordinates while the brake is out (see SplitEffector), and each split is bounded to where its servo gets it
			// next frame while the brake is out and while the splits come back in after it, so deploy and release are fed
			// forward to the stabs and nozzles.
			foreach (SplitEffector sb in splits)
			{
				if (sb.active && (SplitDeploy || splitRetract))
				{
					sb.lo = Mathf.Max(sb.min, sb.now - sb.rate * num);
					sb.hi = Mathf.Min(sb.max, sb.now + sb.rate * num);
					sb.cmd = Mathf.Clamp(sb.cmd, sb.lo, sb.hi);
				}
			}
			// Ifrit 0.3.1 (DeceleronSplitBudget): the split is bounded to the travel the deceleron's deflection leaves
			// (|deflection| + split <= travel, deflection first); SurfacePostfix applies the same cap to the job.
			foreach (SplitEffector sbB in splits)
			{
				if (!sbB.active)
				{
					continue;
				}
				float sAllow = sbB.BudgetSplit();
				if (sbB.brakeMode)
				{
					sbB.lo = Mathf.Min(Mathf.Max(sbB.lo, sbB.max - sAllow), sbB.hi);
				}
				else
				{
					sbB.hi = Mathf.Max(Mathf.Min(sbB.hi, sAllow), sbB.lo);
				}
				sbB.cmd = Mathf.Clamp(sbB.cmd, sbB.lo, sbB.hi);
			}
			if (SplitDeploy)
			{
				splitRetract = true;
			}
			else if (splitRetract)
			{
				bool anyOut = false;
				foreach (SplitEffector sb2 in splits)
				{
					if (sb2.active && sb2.now > 0.5f)
					{
						anyOut = true;
					}
				}
				splitRetract = anyOut;
			}
			Vector3 x3 = (havePrev ? ((omega - omegaPrev) / num) : Vector3.zero);
			Vector3 vector7 = omegaDotF.Step(x3, IndiTau, num);
			Vector3 vector8 = mNowF.Step(zero, IndiTau, num);
			float aIndiF = alphaIndiF.Step(alpha, IndiTau, num);
			float qIndiFv = qIndiF.Step(omega.y * 57.29578f, IndiTau, num);
			// faded out from 25 to 40 deg AoA: the lag matters at high dynamic pressure, and the stalled-foil slopes up there cost roll (bench, full pull + roll at 350 km/h, 64 deg: bank at 1 s 60 -> 48 deg without the fade)
			float hybLo = 1f - Mathf.InverseLerp(25f, 40f, Mathf.Abs(alpha));
			hybridDM = hybLo * ((Mathf.Abs(alpha) < 80f && tas > 30f) ? (mAlphaO * Mathf.Clamp(alpha - aIndiF, -5f, 5f) + mQO * Mathf.Clamp(omega.y * 57.29578f - qIndiFv, -20f, 20f)) : 0f);
			// Ifrit 0.4.9 (F-22E 2.4.7/2.4.8) disturbance-rate lead; the bench figures below are the F-22E's. INDI's pitch target is I(wdot_des - d), d = the filtered measured pitch acceleration
			// less the effectors' modelled share (the airframe's own moment, gravity-driven AoA change, coupling, model error:
			// everything the effectors are not doing). INDI uses d as it was IndiTau ago and the nozzles need ~0.1 s more to
			// deliver, so while d falls steadily the effectors trail it by d' x that lag and the rate hold sags. After a
			// corkscrew release the fuselage loses ~15 deg/s^2 of nose-up in half a second as the AoA drops 57 -> 48 deg
			// (bench: d' about -30 deg/s^3; 24 Oct 22:26 log, 118.3 s: q -3.7). Most of that dip was the allocator leaving the
			// nozzles short (Allocator, CoordNewton); this covers the lag that remains (bench, corkscrew release at 1 km:
			// q min -1.38 -> -0.68). The lead takes d forward by d' x HybridIndiHighAoALead. Model-free on purpose:
			// the stalled-foil slopes the 1.9.0 hybrid lead uses have the wrong sign up here (the fuselage's moment falls as
			// the AoA falls through 57-41 deg). Above the 25-40 deg fade only, with the pitch stick released (zero-rate hold
			// or AoA-limit ride with the stick centred); with the stick in, the previous law (a held pull + roll at 64 deg lost
			// roll rate to a high-AoA lead: bench, 350 km/h, bank at 1 s 74 -> 58 deg).
			float dHatY = vector7.y - IaInv[1, 1] * vector8.y;
			float dDotRaw = (dHatInit && num > 1E-05f) ? ((dHatY - dHatPrevY) / num) : 0f;
			dHatPrevY = dHatY;
			dHatInit = havePrev;
			// the airframe's pitch damping is in d too; its rate (mQ x the pitch acceleration) is the rate loop's own doing, not
			// a disturbance, and leading it fought the capture (bench: q 1.4 -> -0.5 deg/s at the moment the hold took over)
			dDotRaw -= DistLeadNoDamp ? (IaInv[1, 1] * mQO * vector7.y * 57.29578f) : 0f;
			dDotYF += (Mathf.Clamp(dDotRaw, -10f, 10f) - dDotYF) * (num / (DistLeadTau + num));
			// its horizon is the nozzles' response: with their thrust near idle the stabs carry the pitch and the lead only
			// shuffled them (bench, idle release at 380 km/h: q min -0.9 -> -2.0), so it fades out with T/W 0.4 -> 0.2
			float hiGate = (1f - hybLo) * (1f - Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f)) * ((tas > 30f && Mathf.Abs(alpha) < 80f) ? 1f : 0f) * Mathf.InverseLerp(DistLeadTw0, DistLeadTw1, tw);
			distLeadGate = Mathf.MoveTowards(distLeadGate, hiGate, num / 0.15f);
			distLeadAcc = distLeadGate * Mathf.Clamp(dDotYF * Plugin.HybridIndiHighAoALead, -DistLeadMax * (MathF.PI / 180f), DistLeadMax * (MathF.PI / 180f));
			omegaPrev = omega;
			velPrev = rb.velocity;
			vCgPrev = (AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : rb.velocity;
			havePrev = true;
			float num4 = 0f;
			float num5 = 0f;
			Vector3 zero2 = Vector3.zero;
			Vector3 zero3 = Vector3.zero;
			float zNowAll = 0f;
			foreach (Effector eff2 in effs)
			{
				if (eff2.active)
				{
					if (eff2.M != null && eff2.N >= 2)
					{
						zNowAll += eff2.MAt(eff2.now).z;
					}
					Vector3 vector9 = new Vector3(float.MinValue, float.MinValue, float.MinValue);
					Vector3 vector10 = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
					for (int i = 0; i < eff2.N; i++)
					{
						vector9 = Vector3.Max(vector9, eff2.M[i]);
						vector10 = Vector3.Min(vector10, eff2.M[i]);
					}
					// Compass 0.1.10 (FlapPitchLock): the inner flaps add no pitch margin either way (where they are is all they give)
					if (flapLockOn && FlapPitchLockAuth && eff2 is HingeFlapEffector hfe && hfe.mate != null && hfe.mate.active)
					{
						float yNow = eff2.MAt(eff2.now).y;
						vector9.y = yNow;
						vector10.y = yNow;
					}
					zero2 += vector9;
					zero3 += vector10;
				}
			}
			num4 = zero2.y;
			num5 = zero3.y;
			float num6 = IaInv[1, 1];
			aUp = Mathf.Clamp(aUpF.Step((vector7.y + (num4 - vector8.y) * num6) * 57.29578f, 0.15f, num), 5f, 1500f);
			aDn = Mathf.Clamp(aDnF.Step((0f - (vector7.y + (num5 - vector8.y) * num6)) * 57.29578f, 0.15f, num), 5f, 1500f);
			qDotMeasDeg = vector7.y * 57.29578f;   // Su-30 0.1.15 (CaptureMeasured)
			float num7 = 0f;
			float num8 = 0f;
			rollSlew = 100f;
			foreach (SurfaceEffector surface2 in surfaces)
			{
				if (!surface2.active)
				{
					continue;
				}
				float num9 = ((surface2.kind == SurfaceKind.Flaperon || surface2.kind == SurfaceKind.Aileron) ? 1f : ((surface2.kind == SurfaceKind.Stabilator) ? Mathf.Lerp(Plugin.StabRollShare, 0.25f, Mathf.InverseLerp(160f, 240f, ias)) : 0f));
				if (num9 != 0f)
				{
					float num10 = float.MinValue;
					float num11 = float.MaxValue;
					for (int j = 0; j < surface2.N; j++)
					{
						num10 = Mathf.Max(num10, surface2.M[j].x);
						num11 = Mathf.Min(num11, surface2.M[j].x);
					}
					num7 += num9 * 0.5f * (num10 - num11);
					if (surface2.kind != SurfaceKind.Stabilator)
					{
						num8 += 0.5f * (num10 - num11);
					}
					if (surface2.kind != SurfaceKind.Stabilator && surface2.rate > 0f)
					{
						rollSlew = Mathf.Min(rollSlew, surface2.rate / Mathf.Max(1f, 0.5f * (surface2.max - surface2.min)));
					}
				}
			}
			aRoll = Mathf.Clamp(aRollF.Step(num7 * IaInv[0, 0] * 57.29578f, 0.2f, num), 10f, 5000f);
			// Ifrit 0.3.0: roll acceleration the differential TVC can add (half its roll range), scaled by the nozzle travel the
			// pitch TVC leaves free (both share the same +-15 deg). Thrust-based, so it holds at any IAS: at 80 km/h and 65 deg
			// AoA it is ~58 deg/s^2 against ~5-9 from the wing surfaces. Used only for the sideslip correction (SlipTvcAssist).
			float tvcRollHalf = 0f;
			if (tvcd != null && tvcd.active && tvcd.N > 1)
			{
				float txMax = float.MinValue;
				float txMin = float.MaxValue;
				for (int iT = 0; iT < tvcd.N; iT++)
				{
					txMax = Mathf.Max(txMax, tvcd.M[iT].x);
					txMin = Mathf.Min(txMin, tvcd.M[iT].x);
				}
				float tvFree = (tvc != null && tvc.max > 0.1f) ? Mathf.Clamp01(1f - Mathf.Abs(tvc.now) / tvc.max) : 1f;
				tvcRollHalf = 0.5f * (txMax - txMin) * tvFree;
			}
			aRollTvc = Mathf.Clamp(aRollTvcF.Step(tvcRollHalf * IaInv[0, 0] * 57.29578f, 0.2f, num), 0f, 5000f);
			rollCtrlHalf = aRoll * (MathF.PI / 180f) / Mathf.Max(IaInv[0, 0], 1E-09f);
			aRollWing = Mathf.Clamp(aRollWF.Step(num8 * IaInv[0, 0] * 57.29578f, 0.2f, num), 10f, 5000f);
			aYaw = Mathf.Clamp(aYawF.Step(0.5f * (zero2.z - zero3.z) * IaInv[2, 2] * 57.29578f, 0.2f, num), 5f, 3000f);
			// Su-30 0.1.6 (YawThrustShare): the part of aYaw the thrust carries - the canted nozzles' differential (and a yaw nozzle,
			// if any): half their yaw range, from the same tables as aYaw. Thrust-borne yaw does not fade as the IAS falls, so the
			// braking floor's IAS-decay anticipation (qPredK) applies only to the rest (fins, stabs, flaperons).
			float yawTvcHalf = 0f;
			foreach (Effector eY in new Effector[2] { tvcd, tvcy })
			{
				if (eY == null || !eY.active || eY.N < 2)
				{
					continue;
				}
				float zMx = float.MinValue;
				float zMn = float.MaxValue;
				for (int iY = 0; iY < eY.N; iY++)
				{
					zMx = Mathf.Max(zMx, eY.M[iY].z);
					zMn = Mathf.Min(zMn, eY.M[iY].z);
				}
				yawTvcHalf += 0.5f * (zMx - zMn);
			}
			aYawTvc = Mathf.Clamp(aYawTvcF.Step(yawTvcHalf * IaInv[2, 2] * 57.29578f, 0.2f, num), 0f, 3000f);
			yawThrustShareDbg = (YawThrustShare && aYaw > 1f) ? Mathf.Clamp01(aYawTvc / aYaw) : 0f;
			// 2.6.9: how much of the airframe's yaw range all the effectors together are using now (0 = centred, 1 = at the stops):
			// the same effectors and tables as aYaw, read from where they are - whatever carries the yaw at this AoA
			float yawUseRaw = Mathf.Abs(zNowAll - 0.5f * (zero2.z + zero3.z)) / Mathf.Max(1E-06f, 0.5f * (zero2.z - zero3.z));
			yawUseAllF += (Mathf.Clamp01(yawUseRaw) - yawUseAllF) * (num / (0.05f + num));
			float t = Mathf.Clamp01((Mathf.Abs(alpha) - 20f) / 20f);
			float num12 = ((!flightAssist) ? Plugin.MpoGPos : Plugin.GPos);
			float num13 = ((!flightAssist) ? Plugin.MpoGNeg : Plugin.GNeg);
			float t2 = Mathf.Clamp01(Mathf.Max((nz - 0.85f * num12) / (0.15f * num12), (0.85f * num13 - nz) / (0.15f * (0f - num13))));
			stabPitchFrac = 1f;
			foreach (SurfaceEffector surface3 in surfaces)
			{
				if (surface3.kind == SurfaceKind.Stabilator && surface3.active)
				{
					stabPitchFrac = Mathf.Min(stabPitchFrac, surface3.pitchFrac);
				}
			}
			float t3 = 1f - stabPitchFrac;
			pitchNeed = 0f;
			pitchNeedUp = 0f;
			pitchNeedDn = 0f;
			if (Plugin.OvershootPitchPriority && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				float num47 = Mathf.Sign(alpha) * (omega.y * 57.29578f - qT);
				float pnA0 = 15f;
				float pnRamp = 10f;
				if (Plugin.AsymRollCoupling && AsymPitchPrio > 0f && alpha > 0f && asymCapPosEff < 60f)
				{
					float pnG = AsymPitchPrio * AsymGate();
					pnA0 = Mathf.Lerp(15f, Mathf.Min(15f, asymCapPosEff - AsymPitchPrioLead), pnG);
					pnRamp = Mathf.Lerp(10f, 4f, pnG);
				}
				pitchNeed = Mathf.Clamp01((num47 - 2f) / 8f) * Mathf.Clamp01((Mathf.Abs(alpha) - pnA0) / pnRamp);
				if (prot)
				{
					pitchNeed = 1f;
				}
				pitchNeedUp = pitchNeed;
				// 2.9.0: the capped sweep toward zero AoA (last tick's cap state; the cap runs later in the tick). Only while the cap
				// clipped the command on the toward-zero side (positive AoA: the stick asked for more nose-down than the cap gave,
				// yawSatPitchDbg < 0; negative AoA: more nose-up, > 0) and the nose is moving toward zero faster than the capped
				// command. Not into the coupling roll gate (gateC reads pitchNeedUp): the rotation is not cut, the stabs are taken
				// back for the hold. (Bench: gated on the raw sign it also fired on a push to -20 deg AoA under a roll, where the cap
				// clipped the sweep that grows |AoA| and the pitch merely lagged it: pitch weight 8 -> 16 for 0.2 s, 1.3 deg more
				// sideslip in the pull-through after.)
				pitchNeedDn = 0f;
				if (sweepClipOn && Mathf.Sign(alpha) * sweepClipDbg < -1f)
				{
					float dnOver = Mathf.Sign(alpha) * (qT - omega.y * 57.29578f);
					pitchNeedDn = Mathf.Clamp01((dnOver - SweepHoldErr0) / Mathf.Max(0.5f, SweepHoldErrSpan)) * Mathf.Clamp01((Mathf.Abs(alpha) - 15f) / 10f);
				}
				pitchNeed = Mathf.Max(pitchNeed, pitchNeedDn);
				t3 *= 1f - pitchNeed;
			}
			// Revoker 0.2.7 (HighAoARollWeight): with the nozzles not covering pitch (TVC off / low thrust) the surfaces carry
			// every axis and the 0.35 high-AoA roll weight let pitch and yaw take the roll away under saturation; with the nozzles
			// working the F-22E's 0.35 stays (bench: 1 there loosened the damaged-jet reversal, roll error x3). Faded at 1/s.
			hiRollWF = Mathf.MoveTowards(hiRollWF, noTvcOn ? 1f : 0f, num);
			// Revoker 0.4.0: the allocator's lateral residual in the velocity frame (see Allocator.slipFrame). The AoA is the measured
			// one; on the ground, with the stick in the ground modes and in MPO (separate axes) the body frame stands.
			alloc.slipFrame = SlipFrameAlloc && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off;
			alloc.slipSin = Mathf.Sin(alpha * (MathF.PI / 180f));
			alloc.slipCos = Mathf.Cos(alpha * (MathF.PI / 180f));
			alloc.axisWeight = new Vector3(Mathf.Lerp(alloc.slipFrame ? 1f : Mathf.Lerp(1f, Mathf.Lerp(0.35f, Plugin.HighAoARollWeight, hiRollWF), t), 1.5f, t3), Mathf.Lerp(Mathf.Lerp(8f, 15f, t2), 1f, t3), Mathf.Lerp(Mathf.Lerp(Plugin.YawPriority, (YawPrioHi > 0f && Plugin.SlipYawWeightOn) ? Mathf.Lerp(Mathf.Max(2f, Plugin.YawPriority), Mathf.Min(Plugin.YawPriority, YawPrioHi), YawHiSlipGate()) : Mathf.Max(2f, Plugin.YawPriority), t), Mathf.Max(2.5f, Plugin.YawPriority), t3));
			// Su-30 0.1.1 (CantedVvRollWeight): at high AoA the canted nozzles' differential is above all a velocity-roll effector
			// (its rotation axis is ~39 deg from body x, so ~85 % of what it makes lies along the velocity roll at 60-70 deg AoA), and
			// it shares each nozzle's single axis with pitch. With the velocity-roll weight at 1.5 against pitch 1, and the nozzles
			// stronger in pitch than in roll per degree, the allocator kept both nozzles at full nose-up while a roll was asked
			// (Tom's 7 Nov 23:24 log, 67.8-68.9 s: 300-330 km/h, 60 deg AoA, full aft + full roll, nozzles 15 / 15, differential 0,
			// roll 15-30 and yaw 17-45 deg/s^2 short, velocity roll 18-22 deg/s). The velocity-roll weight rises to
			// HighAoAVvRollWeight from 25 to 45 deg AoA while the nozzles have thrust to vector and the roll stick is in; the roll
			// governor still sets the rate.
			cantVvWDbg = 1f;
			{
				float qLag = (qT - omega.y * 57.29578f) * Mathf.Sign(qT);
				vvPitchLagF += (qLag - vvPitchLagF) * Mathf.Clamp01(num / 0.05f);
			}
			if (canted != null && tvcd != null && tvcd.active && alloc.slipFrame && !isAI && Plugin.HighAoAVvRollWeight > 1f)
			{
				float hiA = Mathf.Clamp01((Mathf.Abs(alpha) - CantVvA0) / Mathf.Max(1f, CantVvSpan)) * Mathf.Clamp01(tw / Mathf.Max(0.05f, TvcLowThrustTw));
				// only while a roll is asked (stick or coupled pedal): in a released stop the 0.1.0 weights brake the roll at the
				// pace the yaw can follow (bench, 65 deg release at 120 km/h: sideslip 6.7 -> 8.5 deg with the weight held through it)
				hiA *= Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.05f) / 0.25f);
				// Su-30 0.1.4 (VvWeightPitchGate): not while the pitch is behind its own target. The weight hands the nozzles to the
				// roll; in a rolling pull through 25-35 deg AoA it took them off the pitch the trade had already left it (Tom's
				// 8 Nov 00:32 log, 209.8-210.3 s: nozzles at full differential, q 37 -> 25 deg/s against a target rising 22 -> 42:
				// the "hitch at mid-way AoA"). The pitch-rate error (low-passed 0.05 s) opening from 2 to 8 deg/s on the lagging
				// side takes the weight back to 1; the roll governor's pitch / roll split (PullRollTrade) is what decides the share.
				if (VvWeightPitchGate)
				{
					hiA *= 1f - Mathf.Clamp01((vvPitchLagF - 2f) / 6f);
				}
				cantVvWDbg = Mathf.Lerp(1f, Plugin.HighAoAVvRollWeight, hiA);
				alloc.axisWeight = new Vector3(alloc.axisWeight.x * cantVvWDbg, alloc.axisWeight.y, alloc.axisWeight.z);
			}
			// Su-30 0.3.0 (MpoAxisReleaseStop): in the released stop of a nozzle-axis rotation the allocator works in body axes with the roll
			// weight cut (AxRelRollWeight) and the yaw weight raised (AxRelYawWeight): the nozzles brake the yaw and the roll they make is not
			// fought. In the velocity frame at 100-120 deg AoA a body yaw rate is mostly velocity-axis roll while the canted differential is
			// mostly velocity-normal, so the residual asked the nozzles for almost nothing (bench replay, 16:16 log 728.6 s: differential
			// 0-3 deg for 2 s with r -68 deg/s and the yaw command already at zero).
			// Su-30 0.3.2 (LowSpeedSlipTvc): velocity-normal (sideslip) weight up and velocity-axis weight up with it, in the slip frame
			if (lsSlipBodyW > 0f)
			{
				alloc.axisWeight = new Vector3(Mathf.Lerp(alloc.axisWeight.x, Mathf.Max(alloc.axisWeight.x, LsSlipVvWeight), lsSlipBodyW), alloc.axisWeight.y, Mathf.Lerp(alloc.axisWeight.z, Mathf.Max(alloc.axisWeight.z, LsSlipYawWeight), lsSlipBodyW));
			}
			if (axRelDbg > 0f)
			{
				alloc.slipFrame = false;
				// (bringing the roll weight back as the yaw falls under 30 deg/s was tried: the yaw stop took 1-2 s longer in two of six replays)
				alloc.axisWeight = new Vector3(Mathf.Lerp(alloc.axisWeight.x, AxRelRollWeight, axRelDbg), alloc.axisWeight.y, Mathf.Lerp(alloc.axisWeight.z, Mathf.Max(alloc.axisWeight.z, AxRelYawWeight), axRelDbg));
			}
			alloc.PairedTrimSearch = PairedTrim && Plugin.StabTvcPairedSearch;
			// 2.0.6 rudder pitch aid: the symmetric rudder split is cheap only while no roll or yaw is asked for (stick,
			// pedal, or a roll still running); with lateral input the rudders go back to yaw, as the stabs do.
			if (!Plugin.RudderPitchAid)
			{
				foreach (SurfaceEffector seT0 in surfaces)
				{
					if (seT0.kind == SurfaceKind.Rudder && seT0.partner != null)
					{
						seT0.symCost = 2f;
						seT0.symTarget = 0f;
						seT0.noPitch = true;
					}
				}
			}
			else
			{
				float latNow = Mathf.Max(Mathf.Max(Mathf.Clamp01((Mathf.Abs(sr) - 0.03f) / 0.12f), Mathf.Clamp01((Mathf.Abs(sy) - 0.03f) / 0.12f)), Mathf.Clamp01((Mathf.Abs(pS) - 5f) / 20f));
				toeLatGate = Mathf.Max(latNow, toeLatGate - num / ToeLatRelease);
				float toeCost = Mathf.Lerp(Plugin.RudderToeCost, 2f, toeLatGate);
				// 2.0.7 toe schedule: the rudders split with the pitch demand (the pitch loop's desired pitch acceleration as a
				// share of the authority in that direction, x RudderToeGain), up to RudderToeMax each way, in every airborne
				// mode and at any AoA (a symmetric split makes no net roll, so the rudder fade does not apply to it); the
				// allocator pulls the pair toward it (and still uses more split when the stabs and nozzles run short). Zero
				// with lateral input. Pair value +d (both rudders at +d) is nose-up across the whole AoA range in the surface
				// model; nose-down authority is about 40 % of nose-up.
				float nuY = omegaDotDes.y * 57.29578f;
				float share = (nuY >= 0f) ? (nuY / Mathf.Max(20f, aUp)) : (nuY / Mathf.Max(20f, aDn));
				float toeT = (Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (Plugin.RudderToeMax * Mathf.Clamp(share * Plugin.RudderToeGain, -1f, 1f) * (1f - toeLatGate)) : 0f;
				toeTgtF = Mathf.MoveTowards(toeTgtF, toeT, 100f * num);
				foreach (SurfaceEffector seT in surfaces)
				{
					if (seT.kind == SurfaceKind.Rudder && seT.partner != null)
					{
						seT.symCost = toeCost;
						seT.symTarget = toeTgtF;
						// in a roll the allocator does not see the rudders' pitch at all (2.0.5 model), so it has no reason to
						// split them; takes effect from the next tick's surface model
						seT.noPitch = toeLatGate > 0.5f;
					}
					if (seT.kind == SurfaceKind.Rudder && Mathf.Abs(toeTgtF) > 0.01f)
					{
						// open the window to the toe value past the rudder fade too (both rudders the same value = no net roll)
						seT.lo = Mathf.Clamp(Mathf.Min(seT.lo, toeTgtF), seT.min, seT.max);
						seT.hi = Mathf.Clamp(Mathf.Max(seT.hi, toeTgtF), seT.min, seT.max);
					}
				}
			}
			// active while the pedals are in (and FightPedalRelease s after): rolls without pedal allocate as in 1.8.40
			float pedAct = Mathf.Clamp01((Mathf.Abs(sy) - 0.03f) / 0.15f);
			fightPed = Mathf.Max(pedAct, fightPed - num / Mathf.Max(0.05f, FightPedalRelease));
			float rollIn = Mathf.Clamp01((Mathf.Abs(sr) - 0.05f) / 0.2f);
			fightRollHold = Mathf.Max(rollIn, fightRollHold - num / Mathf.Max(0.05f, FightRollRelease));
			// 2.2.16: RollFightReleased (default on) = FightMode 2: the roll-fight cost also acts while the lateral stick is
			// centred (and FightRollRelease s after). Without it, a crossed set (one stab split against the flaperons, ailerons
			// and rudders at their stops) could hold with the stick centred: 19 Oct 18:05 / 18:32 logs, TVC off.
			int fightMode = (FightMode == 0 && Plugin.RollFightReleased) ? 2 : FightMode;
			// 2.2.16b: with the stick released the cost waits for the pitch to settle (pitch-rate command steady, pitch-rate
			// error under 2-6 deg/s): on a release from a rolling pull the stabs carry the nose-down and their split against
			// the wing is the price of that; pricing it there moved the second stab nose-down too (bench 600 km/h, release
			// at 30 deg AoA: pitch-rate dip -4 -> -8 deg/s). The locked sets formed and held with the pitch settled.
			// 2.2.21: not with a fin lost or the deceleron split working: the surfaces are crossed on purpose there (wing for
			// yaw, stabs for the roll it makes), and pricing it froze the stabs for pitch (19 Oct 22:05 log, fins gone)
			// Ifrit 0.4.1: nor with an engine out or down on thrust: the thrust yaw is held by the tails' differential with
			// the wing taking the roll it makes. Priced, the allocator sat on the cliff of the sign-dependent cost (the wing
			// may cancel the tails' roll only once it over-cancels it) and flipped between two corner sets: tails -10/+21
			// with the inboard flaperon on its stop, or the left tail on its stop at 47 deg local AoA (16 kN of drag) with the
			// nozzle at 15 deg for the pitch (22 Oct 22:15 log, right engine destroyed, right banks at 90-110 m/s).
			// The cost comes back as the yaw saturates (yawSat 0.05 -> 0.35): on a hard pull with the tails spent it keeps the
			// crossed flaperon sets and the splits to what the yaw needs (bench, 300 km/h mil pull to 22 deg: 27 km/h slower
			// after 14 s with the cost simply off).
			// 0.4.3: with an engine OUT only. With an engine down on thrust but running (a third to a half of the split), the
			// cost being off is what let the crossed set hold: 23 Oct 00:51 log, 112.5-122.4 s, stabs +45/+27 (the left on
			// its stop at 41-47 deg of local AoA) against the nozzle at -15 and the crossed wing for 10 s with the stick
			// centred, fightGate 0 - and it collapsed within 0.3 s of the gate coming back at 122.3 s (yawSat 0.5). Of the
			// two corner sets the 0.4.1 exemption was made for, the stab on its stop is now excluded by the settled stall
			// window (0.4.3, StabParkEscape). Bench, the log's push and release at 55 m/s: with the exemption the wing stays
			// crossed (ailerons -30/+26) for 10 s; without it the wing is symmetric within 3 s.
			float relFin = (yawHealth > 0.99f && decelF <= 0.01f) ? 1f : 0f;
			if (engOut || (engAsym && FightExemptAsym))
			{
				relFin *= Mathf.Clamp01((yawSat - 0.05f) / 0.3f);
			}
			float relQuiet = (Plugin.RollFightReleased && FightMode == 0) ? relFin * ((RelQuietHold ? (1f - pfQHold) : 1f) * (1f - Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - RelQuietErr0) / Mathf.Max(0.1f, RelQuietErrSpan)))) : 1f;
			float fightGate = (fightMode == 1) ? 1f : ((fightMode == 2) ? Mathf.Max(fightPed, (1f - fightRollHold) * relQuiet) : fightPed);
			fightGateDbg = fightGate;
			alloc.fightK = ((Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (Mathf.Max(0f, Plugin.RollFightCost) * fightGate) : 0f);
			// 1.8.42: stab-vs-nozzle pitch fight, while a roll is commanded (and RpRelease after)
			// not while the pitch-rate command is moving (captures, releases: the nozzles are the fast pitch effector there),
			// in AoA protection or on an AoA run-away
			pfQHold = Mathf.Max(Mathf.Clamp01(Mathf.Abs(qTraj.v) / Mathf.Max(1f, PitchFightQdot)), pfQHold - num / Mathf.Max(0.05f, PitchFightQHold));
			// and not while the nose is coming up faster than commanded: the nozzles have to be free to stop it
			float pfErr = Mathf.Sign(alpha) * (omega.y * 57.29578f - qT);
			if (PitchFightAbsErr) pfErr = Mathf.Abs(pfErr);
			float pfGate = rpLat * (1f - pfQHold) * (prot ? 0f : (1f - pitchNeed)) * (1f - Mathf.Clamp01((pfErr - PitchFightErr0) / Mathf.Max(0.5f, PitchFightErrSpan)));
			alloc.pitchFightSign = ((alpha >= 0f) ? 1f : (-1f));
			alloc.pitchFightK = ((Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (Mathf.Max(0f, Plugin.RollPitchFightCost) * pfGate) : 0f);
			float num14 = 1f + Mathf.Max(0f, Plugin.AsymAxisBoost) * Mathf.Clamp01((asymUNow - 0.5f) / 0.5f);
			if (num14 > 1.001f)
			{
				alloc.axisWeight = new Vector3(alloc.axisWeight.x * num14, alloc.axisWeight.y, alloc.axisWeight.z * num14);
			}
			foreach (SurfaceEffector surface4 in surfaces)
			{
				if (surface4.kind == SurfaceKind.Stabilator && surface4.partner != null)
				{
					surface4.antiCost = Mathf.Lerp(0.0002f, Mathf.Lerp(Plugin.StabRollCost, 0.01f, Mathf.InverseLerp(160f, 240f, ias)), stabPitchFrac);
					surface4.antiCost = Mathf.Lerp(surface4.antiCost, Mathf.Max(surface4.antiCost, AsymStabAntiCost), pitchNeed * AsymGate());
					// 2.15.0: the lateral reserve prices the stabs' pitch use only where they have pitch use (stabPitchFrac). Past their
					// pitch stall (65 deg AoA, stabPitchFrac 0) the symmetric movement the allocator needs for yaw and roll is not pitch
					// trim, and x 71 on it held the stabs still for 0.5 s at a no-fin reversal while the yaw loop was 60 deg/s^2 short
					// (19:54 log, 164.8 s: symCost 0 -> 0.28 at the stick, stabs parked until the roll target fell away).
					// 2.15.0: the lateral reserve keeps symmetric stab travel for anti-symmetric use. Where the estimator counts the
					// stabs' SYMMETRIC motion as the yaw capacity (rudW -> 0: fins gone or rudders faded) that reserve starves the
					// yaw it is meant to serve (19:54 log, 164.8 s: stabs parked 0.5 s on symCost 0.28), so it goes with the pitch
					// range the stabs still have. With the rudders present it is unchanged.
					float rudWsym = Mathf.Clamp01(rudderAuth) * Mathf.Clamp01(yawHealth / 0.5f);
					surface4.symCost = (surface4.isCanard ? Plugin.StabTrimCost : Plugin.ElevatorTrimCost) * (1f + LatTrimGain * latReserve * (LatReservePitchFrac ? Mathf.Lerp(stabPitchFrac, 1f, rudWsym) : 1f));   // Ternion 0.1.0: per pitch pair
					// 2.2.14 test: extra cost on the stab split at high AoA (roll moved to the rudders / flaperons / sideslip)
					if (HiAoAStabRollCost > 0f)
					{
						surface4.antiCost = Mathf.Max(surface4.antiCost, HiAoAStabRollCost * Mathf.Clamp01((Mathf.Abs(alpha) - HiAoAStabRollA0) / Mathf.Max(1f, HiAoAStabRollSpan)));
					}
					// 2.2.5 test switch: stabs pitch only (any antisymmetric deflection made prohibitively expensive)
					if (Plugin.StabsPitchOnly && !isAI)
					{
						surface4.antiCost = StabsPitchOnlyCost;
					}
				}
			}
			// 2.2.29 (RelLatRollPrio): with the lateral stick and pedals released, the roll axis is weighted up by up to this
			// factor as the body roll runs ahead of its command (2 -> 10 deg/s). Stopping a high-AoA rotation brakes the yaw
			// with the rudders, whose inverse roll then rolled the jet toward the yaw with the ailerons already at their stops
			// (20 Oct 10:35 log, falling-leaf stops at 198 / 285 s: body roll 14 -> 34 deg/s against a 19 command)
			if (RelLatRollPrio > 1f)
			{
				float relLat = 1f - Mathf.Max(Mathf.Clamp01((Mathf.Abs(sr) - 0.03f) / 0.12f), Mathf.Clamp01((Mathf.Abs(sy) - 0.03f) / 0.12f));
				float pErr = Mathf.Abs(omega.x * 57.29578f - pCmd);
				// 2.9.0: not while the capped sweep's pitch hold has the stabs (pitchNeedDn)
				float rlp = Mathf.Lerp(1f, RelLatRollPrio, relLat * Mathf.Clamp01((pErr - 2f) / 8f) * (1f - pitchNeedDn));
				relLatPrioDbg = rlp;
				if (rlp > 1.001f)
				{
					alloc.axisWeight = new Vector3(alloc.axisWeight.x * rlp, alloc.axisWeight.y, alloc.axisWeight.z);
				}
			}
			float relStick = 1f - Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f);
			float relErr = Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - 1f) / 8f);
			pitchPrio = Mathf.Lerp(1f, Mathf.Max(1f, Plugin.ReleasedPitchPriority), stabPitchFrac * relStick * relErr);
			if (pitchPrio > 1.001f)
			{
				alloc.axisWeight = new Vector3(alloc.axisWeight.x, alloc.axisWeight.y * pitchPrio, alloc.axisWeight.z);
			}
			// Revoker 0.2.8: not while the TrailingEdgeBrakeConfig set moves in or out: the elevons' common deflection needs the
			// canards to re-trim with it, and bounded to their own (near-zero) rate they held both still for 0.7 s mid-way and
			// let go in one step (bench, 400 km/h: elevons 11 -> 18 deg in 0.2 s, nz 1.07 -> 0.81)
			bool teBrakeMoving = Mathf.Abs(teBrakeAir - teBrakeAirF) > 0.001f || (teBrakeLandF > 0.001f && teBrakeLandF < 0.999f);
			teBrakeMov = teBrakeMoving;
			// 2.4.6 telemetry: the allocator's axis weights after the priority terms (roll, pitch, yaw)
			axWDbg = alloc.axisWeight;
			// 2.9.1: also while the capped sweep's pitch hold has the stabs (pitchNeedDn). 16:29 log (2.9.0), 75.0 s: the hold took
			// the weights (pitch 8, roll 0.5) and the allocator still left the left stab at -6 while it sent the right from -30 to
			// +35 - in command space one stab's travel covered the pitch increment, so the other stayed on the roll; the servo
			// took 0.9 s over the 65 deg and the nose dropped 40 -> 20 deg AoA meanwhile, then the same stab swung back +35 -> -30
			// for the push once the hold let go: the "hitch" at 17 deg AoA (q -2 for 0.3 s, then -47). Bounded to what the servos
			// reach in 0.1 s, the increment is spread over both stabs and the nozzles and no stab swings stop to stop.
			// 2.41.0 (PushPlanReach): through a planned push the allocator balances the nozzles and wings against where the stabs
			// will be within PushPlanReachTau s, not where they are sent (see Allocator.ReachPass)
			// 2.43.0: the stab rate bound (ReleasedStickStabRateBound, 1.8.14 / 2.9.1) is replaced by the allocator's reach model, in the same
			// places (pitch stick released, or the capped sweep's pitch hold, scaled by the stabs' pitch share) and under a planned push.
			// The bound clipped each stab command to 0.1 s of servo travel. The reach model keeps the command and places the other surfaces
			// against where the stab will be in PushPlanReachTau (Allocator.ReachPass), then compares that plan with the stab moved only
			// as far as it gets in that time, over both horizons and the stab's whole transit (Allocator.TwoHorizon): the bound is the
			// second plan, chosen when the first one's transit costs more than its destination saves. Not under a plan (2.41.0 unchanged).
			// The two-horizon pick matters: with the reach pass alone a roll entry at -43 deg AoA (23:37 log, 50.6 s) sent the right stab to
			// the far stop the wrong way through its range, the roll lagged 0.4 s (pinned command dip 2.8 -> 5.8 deg/s, sideslip 5.7 -> 7.4).
			float relG = Mathf.Max(relStick, pitchNeedDn) * stabPitchFrac;
			// Revoker: not while the TrailingEdgeBrakeConfig set moves in or out (see teBrakeMoving: the 0.2.8 exception the stab rate bound had)
			bool reachHere = ((PushPlan && pushPlanOn) || relG > 0.01f) && !teBrakeMoving;
			stabRateBound = false;
			foreach (SurfaceEffector seR in surfaces)
			{
				seR.reachTau = (seR.kind == SurfaceKind.Stabilator && reachHere && PushPlanReachTau > 0f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? PushPlanReachTau : 0f;
				// Revoker 0.7.0 (RudderReach): the rudder too, always (in flight). It crosses its +-25 deg in ~0.35 s and the solve placed the
				// yaw nozzle against where the rudder was SENT: 00:44 log, falling-leaf reversals at 73.0 / 76.9 s (69 deg AoA): the rudder sent
				// -2 -> -25 at the yaw crossing, the nozzle backed off -15 -> -2 as if the rudder were there, the rudder then flipped -25 <-> +25
				// every few ticks until the nozzle came back for the new side (11-13 deg of nozzle given up, 0.9-1.1 s to the roll's zero).
				// Placed against where the rudder will be in PushPlanReachTau (0.1 s), the nozzle keeps braking. Bench suite: no case worse by
				// more than 0.4 deg, roll-then-pull-rev 400 7.6 -> 4.6, sweep reversal worst 9.5 -> 8.5. The leaf dither itself is not on the
				// bench (its fin sidewash differs): flight-validated.
				if (RudderReach && seR.kind == SurfaceKind.Rudder && PushPlanReachTau > 0f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
				{
					seR.reachTau = PushPlanReachTau;
				}
				seR.twoHorizon = !(PushPlan && pushPlanOn);
			}
			// 1.8.40: while rolling at low/moderate AoA, the stabs are not driven past their lift peak in any pass: beyond
			// it the down-going stab's lift falls off and its nose-down turns into pitch-up exactly when it rolls hardest.
			// 2.9.2: also during the capped sweep toward zero AoA with the stabs' pitch range back (see SweepHoldStallWindow)
			sweepHoldWinDbg = !isAI && sweepClipOn && Mathf.Sign(alpha) * sweepClipDbg < -1f && stabPitchFrac > 0.05f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off;
			if (sweepHoldWinDbg)
			{
				foreach (SurfaceEffector sw in surfaces)
				{
					if (sw.kind == SurfaceKind.Stabilator && sw.active)
					{
						StallWindow(sw, sw.stallAoA + RpMainStallMargin);
					}
				}
			}
			// 2.2.15: with the lateral stick and pedals centred and the AoA under StabParkAoA, a stab past its lift peak is walked
			// back to the attached side. On the stall plateau its modelled moment hardly changes with deflection, so the
			// allocator had no gradient to bring it back and held every other surface against it: 19 Oct 18:05 log (TVC off),
			// 212-227 s, left stab at -30 (local AoA 31-35 deg against a 25 deg peak), flaperons and rudders at their stops
			// against it, for 15 s with the stick centred and the AoA falling 45 -> 0.
			if (Plugin.StabParkEscape && !isAI && Mathf.Abs(sr) < 0.05f && Mathf.Abs(sy) < 0.05f && Mathf.Abs(alpha) < StabParkAoA && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				foreach (SurfaceEffector sw in surfaces)
				{
					if (sw.kind == SurfaceKind.Stabilator && sw.active)
					{
						StallWindow(sw, sw.stallAoA + RpMainStallMargin);
					}
				}
			}
			// 0.4.3 (the F-22E's 2.3.0/2.3.1 settled stall window, ported, on top of the 2.2.15 one - which stays: it keeps a
			// stab from running past the 45 deg peak into falling lift, and the engine-out trims lean on it (bench, engine-out
			// pull at 250 km/h without it: 9.9 deg of sideslip and the VMC cutting the throttle, against 2.9 and none).
			// The 2.2.15 window only acts below 30 deg AoA and allows the peak itself. On this jet the stabilator foil has no
			// stall before 45 deg: its lift keeps
			// rising from 20 to 45 deg of local AoA at a fifth of the linear slope and less (0.17 /rad at 40), so a stab parked
			// out there is "attached" to the 2.2.15 window and the allocator has next to no gradient to bring it back, while it
			// makes its maximum lift: 23 Oct 00:51 log, 112.5-122.4 s, after a full push through zero AoA at 58 m/s with an
			// engine down on thrust - left stab on its stop at +45 (local AoA -41 to -47) for 10 s with the stick centred, the
			// right at +27, the nozzle at -15 against them, the wing crossed, 3-7 deg of sideslip at 70-130 m/s, released all at
			// once at 128 m/s. Bench, same push at 55 m/s: right stab at -37.5/-40.5 (local 47-49) for 10 s.
			// Now: with the lateral stick and pedals centred and the lateral rates settled (the stop finished), the stabilators
			// and rudders may not rest past the smaller of (peak - SettledStallMargin) and the foil's lift-slope knee (where
			// the slope has fallen to SettledSlopeFrac of its steepest, 26 deg on this foil at 0.15) of local AoA, at any AoA
			// where such an attached range exists (none: the window is left open); a surface outside is walked in at
			// SettledWalkRate. The gate latches on below SettledRateOn deg/s of body roll and yaw and off above SettledRateOff,
			// or with lateral input. Telemetry: settledWin.
			// Not while the thrust yaw is loading the fins (|asymAy| / asymCy above SettledYawUse): this jet has no rudders -
			// its yaw is the canted stabilators (10 deg of yaw per 35 of pitch), the nozzles and the deceleron split - and with
			// an engine out a stab run out to 47-63 deg of local AoA is its drag rudder. Bench, engine-out pull at 250 km/h to
			// 22 deg: windowed, the yaw went to the deceleron split, saturated (7.6 deg of sideslip) and the VMC cut the
			// throttle to 0.5 for 3 s; the thrust yaw there is 1.2 x the fins' authority, against 0.29 in the 00:51 log's set.
			// (the usage filtered 0.5 s, with 0.1 of hysteresis: the raw value dips through 0.5 in the engine-out pulls as the
			// AoA moves, and the window flickering on there cost 1-2 deg of sideslip)
			float yawUse = (asymValid && asymCyNow > 1f) ? (Mathf.Abs(asymAyNow) / asymCyNow) : 0f;
			settledUseF += (yawUse - settledUseF) * Mathf.Clamp01(num / 0.5f);
			if (settledUseF > Plugin.SettledYawUse)
			{
				settledUseHi = true;
			}
			else if (settledUseF < Plugin.SettledYawUse - 0.1f)
			{
				settledUseHi = false;
			}
			if (Plugin.StabParkEscape && !isAI && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				float latRate = Mathf.Max(Mathf.Abs(omega.x), Mathf.Abs(omega.z)) * 57.29578f;
				bool latIn = Mathf.Abs(sr) >= 0.05f || Mathf.Abs(sy) >= 0.05f;
				if (latIn || latRate > Plugin.SettledRateOff || settledUseHi)
				{
					settledWin = false;
				}
				else if (latRate < Plugin.SettledRateOn)
				{
					settledWin = true;
				}
				if (settledWin)
				{
					foreach (SurfaceEffector sw in surfaces)
					{
						// Ternion 0.1.0: canards only (ElevatorSettledWindow off): walking the aft elevators to their lift knee
						// (18 deg local) at high AoA took the tail off its lift peak - nose-up the canards then had to cancel
						// Su-30 0.1.0: not the canards when they are a pitch-only pair (CanardSymmetricOnly): the window keeps a canard's roll /
						// yaw gradient, which a pitch-only pair has no use for, and against the pitch allocation it swung the pair 34 <-> 65 deg
						// at the AoA limit at low speed (bench, 55 deg hold at 30-40 m/s)
						if (((sw.kind == SurfaceKind.Stabilator && ((sw.isCanard && !Plugin.CanardSymmetric) || (!sw.isCanard && Plugin.ElevatorSettledWindow))) || (SettledWindowRudders && sw.kind == SurfaceKind.Rudder)) && sw.active)
						{
							float lim = Mathf.Min(sw.stallAoA - Plugin.SettledStallMargin, sw.KneeAoA(Plugin.SettledSlopeFrac));
							SettledWindow(sw, Mathf.Max(5f, lim), Plugin.SettledWalkRate, num);
						}
					}
				}
			}
			else
			{
				settledWin = false;
			}
			// Revoker 0.2.7 (CanardAttachedWindow): at high AoA either sign, the canards are walked into the deflections where their
			// own flow still has lift slope (|local AoA| up to the foil's knee + CanardWindowMargin), whenever such deflections exist in
			// their travel. Left alone, past the knee the allocator sees no roll / yaw gradient on them and parks them stalled
			// (28 Oct 21:18 log, full push + roll at -50 deg: both canards at -1 deg, local AoA -48..-50, for 20 s).
			// Revoker 0.2.7: with a canard shot off, the one left goes back to its stock 40 deg. The extra 5 deg let it trim the jet
			// to 51-53 deg against the asymmetric-load cap (bench: canard_L + wingtip_R + aileron_R + flap_L gone, full pull at
			// 290-300 km/h; 0.2.6: 25-38 deg), with the roll error up from 1.4-1.7 to 7.7-8.6 deg/s.
			if (AsymStabTravel > 0f)
			{
				// Ternion 0.1.0: per pitch pair (a canard lost limits the other canard, an elevator lost the other elevator)
				bool canGone = false, elevGone = false;
				foreach (SurfaceEffector sg in surfaces)
				{
					if (sg.kind == SurfaceKind.Stabilator && !sg.Valid)
					{
						if (sg.isCanard) canGone = true; else elevGone = true;
					}
				}
				if (canGone || elevGone)
				{
					foreach (SurfaceEffector sg in surfaces)
					{
						if (sg.kind == SurfaceKind.Stabilator && sg.active && (sg.isCanard ? canGone : elevGone))
						{
							sg.lo = Mathf.Max(sg.lo, 0f - AsymStabTravel);
							sg.hi = Mathf.Min(sg.hi, AsymStabTravel);
							if (sg.lo > sg.hi)
							{
								sg.lo = (sg.hi = Mathf.Clamp(sg.cmd, 0f - AsymStabTravel, AsymStabTravel));
							}
						}
					}
				}
			}
			canWinDbg = 0f;
			if (Plugin.CanardAttachedWindow && !Plugin.CanardSymmetric && !settledWin && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off && (alpha < 0f - CanWinA0 || (alpha > CanWinA0 && (noTvcOn || !CanWinNegOnly))))
			{
				bool latInC = Mathf.Abs(sr) >= 0.05f || Mathf.Abs(sy) >= 0.05f || Mathf.Max(Mathf.Abs(omega.x), Mathf.Abs(omega.z)) * 57.29578f > Plugin.SettledRateOff;
				if (latInC || !CanWinLatOnly)
				{
					foreach (SurfaceEffector sw in surfaces)
					{
						// Ternion 0.1.0: the canards only, never the aft elevators
						if (sw.kind == SurfaceKind.Stabilator && sw.isCanard && sw.active)
						{
							SettledWindow(sw, sw.KneeAoA(Plugin.SettledSlopeFrac) + Plugin.CanardWindowMargin, CanWinRate, num);
							canWinDbg = 1f;
						}
					}
				}
			}
			// Su-30 0.1.9 (PitchWallSurfacePeak; 0.1.8 CanardPitchPeakWindow, canards only and above 300 km/h). In the pitch wall the
			// canards and stabs are put at their own raw nose-up peak in this tick's flow. Past their stall the allocator sees their
			// pitch as a monotone envelope that is flat, so it had nothing to choose by: the stabs went to ~0 deg at 60-68 deg AoA,
			// the worst place for them (9 Nov 09:59 log, 196.4-197.6 s: both stabs at 0.3 deg, pitch rate 10-15 deg/s against a
			// 30-44 target; bench, 450 km/h static: stabs at +35 or -30 give 33-50 deg/s^2 more nose-up than at 0 there), and the
			// canards went to -45 (local AoA 110-117) where -15..-37 gives 14-43 deg/s^2 more. The canards (symmetric only) are
			// pinned to their peak; each stab is held within WallStabRollBand of its own, so the pair can still split for roll.
			// Only in the wall: 40 deg AoA to 2 short of the limit, above 150 km/h, the pitch rate over 10 deg/s short of its
			// target (held 0.5 s). Elsewhere they keep their whole travel: past their peak they still carry the roll / yaw of their
			// uneven flow (held at every AoA, 0.1.8's canard window cost 13 % of the low-AoA roll rate on the bench).
			// With roll or yaw stick in, the stabs are not held at their peak (bench, full aft + roll from 350 km/h: peak sideslip
			// 8.5 -> 16.8 deg) but kept out of the worst-pitch middle of their travel (WallLateralGoodFrac): past the stall the good
			// set is often both ends (60-65 deg AoA: -30 and +35), so one stab can sit at each end and the pair still rolls (9 Nov
			// 10:25 log, 197.0-198.1 s: full aft + full left roll at 65 deg AoA, 400 km/h, the right stab sat at +2..+11 deg, the
			// pitch rate went from +22 to -9 deg/s against a 36-53 target and the AoA fell to 45).
			canPeakLo = canPeakHi = 0f;
			float qShortP = (alpha >= 0f) ? (qT - omega.y * 57.29578f) : 0f;
			// Su-30 0.2.4 (WallLiveLimit): "2 short of the limit" is the limit in force now - the no-TVC / low-thrust cap and the asymmetric-load
			// cap included - not AoALimitPositive. With TVC off (cap 46-50) the wall ran on to 68 deg, so the canards were pinned at their
			// nose-up peak (-34 deg, local AoA 60-83) right through the approach to the cap and held there 0.5 s more; they then had ~100 deg
			// to swing for the stop (14:55 log, TVC off: 245.5 s 50 -> 81 deg, 297.5 s 48 -> 64.7, 353.2 s 46.4 -> 72.4). The pin is also let
			// go at once when the pitch rate reaches its target (the 0.5 s hold is only for a target that stays short).
			float wallTop = Plugin.AoAPos;
			if (WallLiveLimit && Mode != FcsMode.MPO)
			{
				wallTop = Mathf.Min(wallTop, Mathf.Min(asymCapPosEff, authCapEff));
			}
			bool inWall = alpha > 40f && alpha < wallTop - 2f && qShortP > 10f && ias * 3.6f > 150f;
			canPeakHold = inWall ? 0.5f : ((WallLiveLimit && (qShortP <= 0f || alpha >= wallTop - 2f)) ? 0f : Mathf.Max(0f, canPeakHold - num));
			bool wallLat = Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) > 0.1f;
			if (Plugin.PitchWallSurfacePeak && canPeakHold > 0f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				if ((!wallLat || Plugin.WallPinLateralCanard) && Plugin.CanardSymmetric && canL != null && canR != null && canL.active && canR.active && canL.RawPitchPeaks(out float cuL, out float cdL) && canR.RawPitchPeaks(out float cuR, out float cdR))
				{
					float cUp = 0.5f * (cuL + cuR);
					foreach (SurfaceEffector sp in new SurfaceEffector[2] { canL, canR })
					{
						PinToward(sp, cUp, cUp, num);
					}
					canPeakLo = cUp;
				}
				foreach (SurfaceEffector se2 in surfaces)
				{
					if (se2.kind != SurfaceKind.Stabilator || se2.isCanard || !se2.active)
					{
						continue;
					}
					if (wallLat)
					{
						// with roll / yaw stick: each stab is kept in the good-pitch run of its travel whose roll / yaw moment best
						// matches the last lateral demand, so the pair still splits for roll but not through the worst-pitch middle
						if (Plugin.WallLateralGoodFrac > 0f && se2.RawPitchGoodZone(Plugin.WallLateralGoodFrac, wallRollDem, wallYawDem, se2.now, out float gLo, out float gHi))
						{
							PinToward(se2, gLo, gHi, num);
							canPeakHi = 0.5f * (gLo + gHi);
						}
					}
					else if (se2.RawPitchPeaks(out float euP, out float edP))
					{
						PinToward(se2, euP - WallStabRollBand, euP + WallStabRollBand, num);
						canPeakHi = euP;
					}
				}
			}
			settledWinDbg = settledWin;
			nAlphaTimer += num;
			if (nAlphaTimer > 0.1f)
			{
				nAlphaTimer = 0f;
				long tEnv0 = System.Diagnostics.Stopwatch.GetTimestamp();
				ComputeEnvelope(vector, num2, transform.right, vector6);   // Su-30 0.2.0 (F-22E 3.1.3): with the hybrid slopes inside
				phEnv += System.Diagnostics.Stopwatch.GetTimestamp() - tEnv0;
			}
			long tAsym0 = System.Diagnostics.Stopwatch.GetTimestamp();
			mk[7] = tAsym0;
			AsymStep(num, num2, rotation, ref ctx, vector7, vector8);
			phAsym += System.Diagnostics.Stopwatch.GetTimestamp() - tAsym0;
			mk[8] = System.Diagnostics.Stopwatch.GetTimestamp();
			tw = tvc.totalThrust / (mass * 9.81f);
			float num15 = 0.5f * num2 * tas * tas;
			float num16 = 13781.25f;
			float num17 = Mathf.Clamp01(num15 / num16);
			num17 *= num17;
			float t4 = Mathf.Clamp01(Plugin.LowQRegRelief) * Mathf.InverseLerp(40f, 60f, ias);
			alloc.regScale = Mathf.Lerp(1f, Mathf.Max(0.02f, num17), t4);
			if (!revFlow && Mathf.Abs(alpha) > 100f)
			{
				revFlow = true;
			}
			else if (revFlow && Mathf.Abs(alpha) < 80f)
			{
				revFlow = false;
			}
			if (revFlow != revFlowLogged)
			{
				revFlowLogged = revFlow;
				Plugin.Log.LogInfo(revFlow ? "Su-30 FCS: reversed flow (flying backwards) - roll sense flipped to body axes." : "Su-30 FCS: forward flow.");
			}
			bool flag3 = false;
			bool flag4 = false;
			bool flag5 = false;
			LandingGear landingGear = NoseGear();
			foreach (LandingGear gear2 in gears)
			{
				if (!(gear2 == null) && gear2.WeightOnWheel(0.05f))
				{
					flag3 = true;
					if (gear2 == landingGear)
					{
						flag5 = true;
					}
					else
					{
						flag4 = true;
					}
				}
			}
			groundFrames = (flag3 ? Mathf.Min(groundFrames + 1, 1000) : 0);
			onGround = flag3;
			airTime = (flag3 ? 0f : (airTime + num));
			bool flag6 = flag3 || airTime < 0.2f;
			noseUpTime = ((landingGear != null && flag4 && !flag5 && tas > Plugin.RotationMinSpeed) ? (noseUpTime + num) : 0f);
			if (noseUpTime > 0.15f)
			{
				rotating = true;
				noseDownTime = 0f;
			}
			else if (rotating)
			{
				noseDownTime = (flag5 ? (noseDownTime + num) : 0f);
				if (!flag3 || noseDownTime > 0.3f || tas < Plugin.RotationMinSpeed - 8f)
				{
					rotating = false;
				}
			}
			mainsOnGround = flag4;
			// 2.0.6: pitch-rate command in every airborne mode on request (MPO stays MPO: it is rate command already)
			FcsMode airMode = (!flightAssist) ? FcsMode.MPO : ((!ac.gearDeployed) ? (isAI ? FcsMode.AIRate : (Plugin.RateCommandOnly ? FcsMode.Rate : FcsMode.AoAG)) : FcsMode.GearDownRate);
			// Su-30 0.2.0 (SoftAoALimit, Tom 9 Nov 21:28: "+70/-65 is a hard cap, but with those weird corkscrew maneuvers that forced the
			// jet out of the AoA limit, no protection, treat it as MPO"). The capture holds the limit within 0.5-1.5 deg in every pull of
			// the 20:59-21:11 logs; the excursions past that (corkscrews with the yaw saturated, a tumble) went 7-14 deg over, and the
			// protection then pitched the nose back down at 48 deg/s with the rotation still going (21:04 log, 175 s: 81.5 -> 44 deg in
			// 1.5 s, 23 deg of sideslip). Past the limit by more than SoftAoALimitMargin the AoA law hands the jet to the MPO law; it takes
			// it back once the AoA has been inside the limit for 0.25 s.
			// Su-30 0.3.10 (Tom 11 Nov 09:34): no airspeed cut-out once the hand-over holds (it used to need TAS > 15 m/s every tick:
			// 09:31 log 122.4 s and 00:32 log 628.0 s, the hold dropped at 14-15 m/s and the AoA law pushed the nose down at 44-48 deg/s);
			// a hand-over starts down to SoftMpoMinTas.
			if (airMode == FcsMode.AoAG && Plugin.SoftAoALimit && !isAI && (softMpo || softPitch || tas > SoftMpoMinTas))
			{
				float limSP = Plugin.AoAPos;
				float limSN = Mathf.Abs(Plugin.AoANeg);
				float mS = Plugin.SoftAoALimitMargin;
				// Su-30 0.3.7 (SoftMpoStick, Tom 11 Nov 00:45: "at the end my aoa was at 100 something the fcs is supposed to hold it there
				// even without mpo"): the soft hand-over is for the pilot forcing the jet past the limit, so it needs the stick pulling that
				// way (past SoftMpoStickIn) and ends when the stick eases under SoftMpoStickOut - the AoA law's protection then pushes back
				// to the limit. 00:32 log 625.8-628.0 s: full aft + full left forced 78 deg, the stick was released at 626.5 and the MPO law
				// held zero pitch rate with no push-back - AoA 87 -> 112 hands off, handed back only when TAS fell under 15 m/s (peak 124).
				float sideS = (alpha >= 0f) ? 1f : -1f;
				float spS = sp * sideS;
				// Su-30 0.3.8 (SoftMpoFromMpo, Tom 11 Nov 01:04: "if i mpo the jet and then turn it back on, no recovery, i want that
				// behavior"): Stability Assist switched back on past the limit (last tick the pilot's MPO) lands in the soft hand-over and
				// stays there until the AoA is back inside - the stick's release does not end it. 00:59 log 42.15 s: MPO to 121 deg,
				// assist on, the AoA law pushed at 48 deg/s.
				bool pastLim = alpha > limSP + mS || alpha < 0f - (limSN + mS);
				if (!softMpo && SoftMpoFromMpo && pilotMpoPrev && pastLim)
				{
					softMpo = true;
					softMpoFromMpo = true;
					softMpoInside = 0f;
					Plugin.Log.LogInfo(string.Format(CultureInfo.InvariantCulture, "Su-30 FCS soft AoA limit: assist back on at AoA {0:F1}, MPO law until back inside", alpha));
				}
				if (!softMpo)
				{
					if (Mode == FcsMode.AoAG && pastLim && (!SoftMpoStick || spS > SoftMpoStickIn))
					{
						softMpo = true;
						softMpoFromMpo = false;
						softMpoInside = 0f;
						Plugin.Log.LogInfo(string.Format(CultureInfo.InvariantCulture, "Su-30 FCS soft AoA limit: AoA {0:F1} past the limit, MPO law until back inside", alpha));
					}
				}
				else
				{
					softMpoInside = (alpha < limSP && alpha > 0f - limSN) ? (softMpoInside + num) : 0f;
					if (softMpoInside > 0.25f)
					{
						softMpo = false;
					}
					else if (SoftMpoStick && !softMpoFromMpo && spS < SoftMpoStickOut)
					{
						softMpo = false;
						softMpoInside = 0f;
						Plugin.Log.LogInfo(string.Format(CultureInfo.InvariantCulture, "Su-30 FCS soft AoA limit: stick released at AoA {0:F1}, back to the AoA law", alpha));
					}
				}
				// Su-30 0.3.10 (SoftPitchHold, Tom 11 Nov 09:34: "get rid of the 3degs after overshooting aoa limit since that cause the
				// jet to nose down"; his pick: pitch only, the AoA law keeps its lateral law): past the limit by SoftPitchMargin (where the
				// protection's push starts) and until the AoA has been back inside for 0.25 s, the AoA law does not pitch the nose down by
				// itself: a released stick holds zero pitch rate, aft stick may not pull further past, forward stick still pitches in. The
				// MPO hand-over past SoftAoALimitMargin (3 deg) is unchanged. 09:31 log 108.8 s: full aft let the AoA creep to 77.7, the
				// stick was released inside the 3 deg band and the protection pushed at 22 deg/s to 69.
				if (SoftPitchHold && !softMpo)
				{
					bool pastP = alpha > limSP + SoftPitchMargin || alpha < 0f - (limSN + SoftPitchMargin);
					if (!softPitch && pastP)
					{
						softPitch = true;
						softPitchInside = 0f;
					}
					else if (softPitch)
					{
						softPitchInside = (alpha < limSP && alpha > 0f - limSN) ? (softPitchInside + num) : 0f;
						if (softPitchInside > 0.25f)
						{
							softPitch = false;
						}
					}
				}
				else
				{
					softPitch = false;
				}
				if (softMpo)
				{
					airMode = FcsMode.MPO;
				}
			}
			else
			{
				softMpo = false;
				softMpoInside = 0f;
				softPitch = false;
			}
			pilotMpoPrev = !flightAssist;
			FcsMode fcsMode;
			if (Plugin.GroundRateLaw && landingGear != null)
			{
				// 2.0.6 ground pitch: the gear-down rate law flies the pitch axis on the runway as soon as the pilot pulls
				// above RotationMinSpeed, and keeps it through the rotation and the lift-off. 2.0.5 flew an open-loop stick
				// map until the nose wheel had been off for 0.15 s, then an 8 deg/s rotation law, and dropped back to the
				// open-loop map for 0.2 s every time the mains' weight-on-wheel flickered as the wing took the weight: full
				// stab and nozzle deflection for those frames, the spike at lift-off. Contact here is 'touching' (0.2 % of
				// the leg travel) rather than 'carrying weight' (5 %), so a lightly loaded leg still counts.
				bool noseC = landingGear.WeightOnWheel(0.002f);
				bool mainC = false;
				foreach (LandingGear gC in gears)
				{
					if (gC != null && gC != landingGear && gC.WeightOnWheel(0.002f))
					{
						mainC = true;
					}
				}
				bool anyC = noseC || mainC;
				wheelsTouch = anyC;
				gContactAir = anyC ? 0f : (gContactAir + num);
				// Revoker 0.2.8 (TrailingEdgeBrakeConfig), touchdown with the brake held and the stick not pulled back:
				// - mains only: the gear-down rate law stays on and lowers the nose at DerotationRate (derotGate, pitch law),
				//   with the elevons already trailing edge up (lift dump), so the canards go leading edge down to fly it;
				// - nose wheel down too: straight to the ground mode and its full rollout aero-brake. Before, the rate law held
				//   the attitude until the stick had been neutral for 0.3 s with all wheels down.
				// Forcing the open-loop ground mode at the mains contact instead let the nose fall on its own (bench, 260 km/h,
				// 10 deg touchdown: -13.6 deg/s at the nose-wheel contact, 210 kN on the nose leg against 13 kN static).
				derotGate = TeBrakeOn && mainC && !noseC && airBrake > 0.05f && sp <= 0.08f && tas >= Plugin.RotationMinSpeed;
				bool brakeLand = TeBrakeOn && mainC && noseC && airBrake > 0.05f && sp <= 0.08f;
				if (anyC && tas >= Plugin.RotationMinSpeed && brakeLand)
				{
					groundRateOn = false;
				}
				else if (anyC && tas >= Plugin.RotationMinSpeed)
				{
					if (!groundRateOn)
					{
						if (!noseC || sp > 0.08f)
						{
							groundRateOn = true;
							gNeutralT = 0f;
						}
					}
					else
					{
						gNeutralT = (noseC && mainC && sp < 0.03f) ? (gNeutralT + num) : 0f;
						if (gNeutralT > 0.3f)
						{
							groundRateOn = false;
						}
					}
				}
				else if (anyC && tas < Plugin.RotationMinSpeed - 8f)
				{
					groundRateOn = false;
				}
				else if (!anyC && gContactAir > 0.5f)
				{
					groundRateOn = false;
				}
				noseContactLaw = noseC && groundRateOn;
				if (anyC)
				{
					fcsMode = groundRateOn ? FcsMode.GearDownRate : FcsMode.Ground;
				}
				else
				{
					// a bounce in the open-loop ground mode stays there for 0.2 s, as before, unless the pilot pulls
					fcsMode = (Mode == FcsMode.Ground && gContactAir < 0.2f && sp <= 0.08f && tas < 110f) ? FcsMode.Ground : (groundRateOn ? FcsMode.GearDownRate : airMode);
				}
			}
			else
			{
				noseContactLaw = false;
				derotGate = false;
				fcsMode = ((rotating && flag4) ? FcsMode.GearDownRate : ((flag6 && (tas < 110f || groundFrames > 10)) ? FcsMode.Ground : airMode));
			}
			if (fcsMode != Mode)
			{
				if (Mode == FcsMode.Ground || Mode == FcsMode.Off)
				{
					qT = omega.y * 57.29578f;
					qTprev = qT;
					qTraj.Reset(qT);
					pTraj.Reset(0f);
					mpoRTraj.Reset(0f);
					mpoPTraj.Reset(0f);
					pS = 0f;
					aInt = 0f;
					capIUp = (capIDn = 0f);
					gInt = 0f;
				}
				Mode = fcsMode;
			}
			if (Mode != lastLoggedMode)
			{
				Plugin.Log.LogInfo("Su-30 FCS mode: " + Mode);
				lastLoggedMode = Mode;
			}
			gbLogTimer += num;
			if (Plugin.GroundBrakeLog && flag3 && gbLogTimer > 1f)
			{
				gbLogTimer = 0f;
				LandingGear landingGear2 = NoseGear();
				Plugin.Log.LogInfo(string.Format("Su-30 FCS ground brake: mode {0}, mainsWow {1}, noseWow {2} (nose gear {3}), ", Mode, flag4, flag5, (landingGear2 != null) ? landingGear2.name : "none") + $"tas {tas:F1} (min {Plugin.GroundBrakeMinSpeed:F0}), brake {inp.brake:F2}, throttle {inp.throttle:F2} (< {Plugin.GroundBrakeThrottle:F2}), " + string.Format("rotating {0}, blend {1:F2}, airbrake {2:F2} (split {3})", rotating, groundBrake, airBrake, SplitDeploy ? "out" : "stowed"));
			}
			float num18 = num / (Plugin.StickSmoothing + num);
			spF += (Mathf.Clamp(0f - inp.pitch, -1f, 1f) - spF) * num18;
			srF += (Mathf.Clamp(inp.roll, -1f, 1f) - srF) * num18;
			syF += (Mathf.Clamp(inp.yaw, -1f, 1f) - syF) * num18;
			sp = Mathf.MoveTowards(sp, spF, 8f * num);
			sr = Mathf.MoveTowards(sr, srF, RollStickRate * num);
			sy = Mathf.MoveTowards(sy, syF, 8f * num);
			// Ifrit 0.2.1: the brake command: the brake key, or with ThrottleIndependentAirbrake off zero throttle instead (stock rule).
			// In the air it is the speedbrake (AirbrakeOut), on the main wheels the rollout aero-brake below.
			// F-22E 2.4.15 / Ifrit 0.5.5 / Revoker 0.2.6: with ThrottleIndependentAirbrake off the brake key no longer opens it: the stock
			// rule only (Airbrake.Update and the split surfaces open at throttle == 0; the stock Brake action is the wheel brake alone).
			// Before, the brake key opened it as well, so a controller with Brake and Countermeasures on one button popped the
			// airbrake with every flare in the air.
			bool abCmd = Plugin.ThrottleIndependentAirbrake ? (Mathf.Clamp01(inp.brake) > 0.05f) : (rawThrottle <= 0f);
			airBrake = Mathf.MoveTowards(airBrake, abCmd ? 1f : 0f, 2f * num);
			gsPedRaw = Mathf.Clamp(inp.yaw, -1f, 1f);
			long tGs0 = System.Diagnostics.Stopwatch.GetTimestamp();
			mk[9] = tGs0;
			GroundSteer(num, flag4, flag5);
			phGround += System.Diagnostics.Stopwatch.GetTimestamp() - tGs0;
			mk[10] = System.Diagnostics.Stopwatch.GetTimestamp();
			Vector3 vector11;
			tvc.anchorK = 0f;
			if (Mode == FcsMode.Ground)
			{
				float num19 = 0f - sp;
				// Revoker 0.2.18: canard travel known at a standstill (see SurfaceEffector.EnsureAsymRange)
				foreach (SurfaceEffector seA in surfaces)
				{
					if (seA.kind == SurfaceKind.Stabilator && seA.active)
					{
						seA.EnsureAsymRange(((Component)(object)ac).transform);
					}
				}
				float num20 = sr;
				if (gsActive && gsMainsContact)
				{
					num20 = Mathf.Lerp(Mathf.Clamp((0f - phi) / GsPhiFull, -1f, 1f), sr, Mathf.Clamp01(Mathf.Abs(sr) / 0.3f));
				}
				float num21 = (gsActive ? gsRud : sy);
				float target = 0f;
				bool flag7 = flag4;
				bool flag8 = inp.brake > 0.05f || (!Plugin.ThrottleIndependentAirbrake && rawThrottle <= 0f) || (Plugin.GroundBrakeThrottle >= 0f && inp.throttle < Plugin.GroundBrakeThrottle);
				if (TeBrakeOn && flag7 && flag8 && tas >= Plugin.GroundBrakeMinSpeed)
				{
					target = Plugin.GroundAeroBrake * airBrake;
				}
				groundBrake = Mathf.MoveTowards(groundBrake, target, 1.5f * num);
				if (brakeBias == null || brakeBias.Length != surfaces.Count)
				{
					brakeBias = new float[surfaces.Count];
				}
				for (int k = 0; k < surfaces.Count; k++)
				{
					brakeBias[k] = 0f;
				}
				if (groundBrake > 0.01f)
				{
					for (int l = 0; l < surfaces.Count; l++)
					{
						SurfaceEffector surfaceEffector = surfaces[l];
						if (surfaceEffector.active && surfaceEffector.kind != SurfaceKind.Rudder)
						{
							// Revoker 0.2.8: geometry sign as a fallback (teUp is learned from the flow only above 10 m/s). Canards:
							// trailing edge up = leading edge down, full GroundBrakeCanard (the nose-down sense on a canard).
							float teUp = surfaceEffector.GeoTeUp(((Component)(object)ac).transform);
							if (teUp != 0f)
							{
								// Ternion 0.1.2: canards leading edge down (GroundBrakeCanard), the aft elevators trailing edge up
								// (GroundBrakeElevator, the F-22E's stabilators); flaperon_L/R take the inboard share, rollervon_L/R the outboard one
								float gbAmt = (surfaceEffector.kind == SurfaceKind.Stabilator) ? (surfaceEffector.isCanard ? Plugin.GroundBrakeStab : Plugin.GroundBrakeElevator) : ((surfaceEffector.kind == SurfaceKind.Flaperon) ? Plugin.GroundBrakeInboard : Plugin.GroundBrakeFlap);
								brakeBias[l] = groundBrake * teUp * gbAmt;
							}
						}
					}
					for (int m = 0; m < surfaces.Count; m++)
					{
						SurfaceEffector surfaceEffector2 = surfaces[m];
						if (surfaceEffector2.kind == SurfaceKind.Rudder && surfaceEffector2.partner is SurfaceEffector surfaceEffector3 && surfaceEffector2.active && surfaceEffector3.active)
						{
							int num22 = surfaces.IndexOf(surfaceEffector3);
							if (num22 >= 0)
							{
								float num23 = groundBrake * Plugin.GroundBrakeRudder;
								brakeBias[m] = surfaceEffector2.pairSign * num23;
								brakeBias[num22] = (0f - surfaceEffector3.pairSign) * num23;
							}
						}
					}
				}
				for (int n = 0; n < surfaces.Count; n++)
				{
					SurfaceEffector surfaceEffector4 = surfaces[n];
					// 2.0.7: on the runway the rudders split with the pitch stick too (pull = the nose-up split), as for rotation
					float gToe = (Plugin.RudderPitchAid && surfaceEffector4.kind == SurfaceKind.Rudder) ? ((0f - num19) * Plugin.RudderToeMax) : 0f;
					// Ifrit 0.3.6 (DeceleronHldFold): on the runway the decelerons carry the HLD's flap share as droop (TE-down = -teUp)
					float gDroop = (surfaceEffector4.kind == SurfaceKind.Flaperon && flaps != null && Plugin.DeceleronHldFold) ? ((0f - surfaceEffector4.teUp) * flaps.decHl) : 0f;
					// Revoker 0.2.1 (GroundElevonPitch): on the runway the trailing-edge elevons follow the pitch stick too (pull = TE up),
					// as fractions of their own travel; with roll on top the up-going side saturates first
					float gElev = (surfaceEffector4.kind == SurfaceKind.Aileron && Plugin.ElevonPitch) ? ((0f - num19) * surfaceEffector4.GeoTeUp(((Component)(object)ac).transform) * Plugin.GroundElevonPitch * Mathf.Abs(surfaceEffector4.origRollRange)) : 0f;
					// Revoker 0.2.18: on the runway full pitch stick takes the canards to their full travel that way (push: leading edge
					// down 70, pull: leading edge up 45), not stick x the stock pitch range (~35 deg): with the +70 travel a full push
					// reached only half of it (30 Oct logs: stick 0.8 -> 27.7 deg), and only push + brake together reached the stop.
					float gPitch = num19 * surfaceEffector4.origPitchRange;
					if (surfaceEffector4.kind == SurfaceKind.Stabilator && Mathf.Abs(gPitch) > 0.0001f)
					{
						gPitch = Mathf.Abs(num19) * Mathf.Sign(gPitch) * surfaceEffector4.LimitFor(gPitch);
					}
					surfaceEffector4.cmd = Mathf.Clamp(gPitch + num20 * surfaceEffector4.origRollRange + num21 * surfaceEffector4.origYawRange + brakeBias[n] + gToe + gDroop + gElev, surfaceEffector4.min, surfaceEffector4.max);
				}
				foreach (HingeFlapEffector hG in hinges)
				{
					float gElevH = FlapPitchLock ? 0f : ((0f - num19) * hG.GeoTeUp(((Component)(object)ac).transform) * Plugin.GroundElevonPitch * hG.max);
					// Revoker 0.2.8 (TrailingEdgeBrakeConfig): the inboard elevons go trailing edge up in the rollout aero-brake too
					float gBrkH = groundBrake * hG.GeoTeUp(((Component)(object)ac).transform) * Plugin.GroundBrakeInboard;
					hG.cmd = Mathf.Clamp(num20 * hG.pairSign * hG.max + hG.symTarget + gElevH + gBrkH, hG.min, hG.max);
				}
				tvc.cmd = Mathf.Clamp((0f - num19) * tvc.max + groundBrake * Plugin.GroundBrakeTvc, tvc.min, tvc.max);
				// Ifrit 0.2.2: no allocator on the ground, so with the brake out the splits are held at zero retraction (full
				// brake) here; a command left over from the air would otherwise hold the split in on the runway
				if (splitBrakeMode)
				{
					foreach (SplitEffector sg in splits)
					{
						sg.cmd = 0f;
					}
				}
				qCmd = (qT = omega.y * 57.29578f);
				qTprev = qT;
				qTraj.Reset(qT);
				pTraj.Reset(0f);
				mpoRTraj.Reset(0f);
				mpoPTraj.Reset(0f);
				pCmd = omega.x * 57.29578f;
				rCmd = omega.z * 57.29578f;
				omegaDotDes = Vector3.zero;
				unload = (prot = false);
				pS = 0f;
				gInt = 0f;
				aInt = 0f;
				capIUp = (capIDn = 0f);
				vector11 = vector8;
				tvcPitchInput = Mathf.Clamp(num19, -1f, 1f);
			}
			else
			{
				long tLaw0 = System.Diagnostics.Stopwatch.GetTimestamp();
				PitchLaw(num, vc);
				LateralLaw(num);
				phLaws += System.Diagnostics.Stopwatch.GetTimestamp() - tLaw0;
				if (gsActive && gsMainsContact)
				{
					// rolling on the mains with the nose up (rotation / landing flare-out): the ground yaw-rate command replaces
					// the airborne sideslip law, so a crosswind cannot weathervane the jet off the line
					rCmd = gsRCmdMains;
					rDotFF = 0f;
					// on the mains the roll axis is a bank hold in the air law; a crabbed touchdown's tyre side force (2 m below
					// the CG) tips the jet onto one main and the neutral stick would then hold that bank. Level the wings instead,
					// fading out with roll stick.
					pCmd = Mathf.Lerp(Mathf.Clamp((0f - GsKphi) * phi, -30f, 30f), pCmd, Mathf.Clamp01(Mathf.Abs(sr) / 0.3f));
				}
				qTprev = qT;
				Vector3 vector12 = new Vector3(pCmd, qT, rCmd) * (MathF.PI / 180f) - omega;
				// 2.2.5 pitch lead with lateral stick in a pull: the stabs go to the roll (latReserve) and the nozzles, slewing
				// ~70 deg/s, carry the pitch. The rate loop then reads the pitch-rate error ~0.3 s late, drives the nozzles
				// stop to stop and the pitch rate swings 20-40 deg/s about its command (14 Oct 18:36 log, 666-671 s: full aft
				// from a 525 km/h dive, full roll at 37 deg AoA, q 61 against 48, then 10 against 46, AoA 66.6 at the 65 limit).
				// The measured pitch rate is taken PitchLeadT x nozzle share ahead along the measured pitch acceleration.
				pitchLeadDbg = 0f;
				// pulls only (positive pitch stick, positive AoA): the push + roll corkscrews are left as they were
				float leadT = PitchLeadT * Plugin.RollPitchLead / 0.25f;
				if (leadT > 0f && Plugin.LateralStabReserve > 0.01f && latReserve > 0.001f && sp > 0f && alpha > 0f)
				{
					float gL = Mathf.Clamp01(latReserve / Plugin.LateralStabReserve) * CapTvcShare();
					pitchLeadDbg = leadT * gL;
					// (lead on q itself; taking the trajectory's own acceleration out doubled the bench hitch, 6 -> 12 deg/s)
					vector12.y -= pitchLeadDbg * vector7.y;
				}
				// 2.4.18 yaw slew lead: the rudders reach the other side of their travel no faster than their servo allows (35 deg
				// at 100 deg/s), so the yaw loop reads the yaw-rate error late by the time they need to swing back through neutral,
				// keeps accelerating the yaw past its command and then cannot brake it in time. 30 Oct 22:36 log, 51.6-52.4 s:
				// rudders at +35, yaw rate 42 against a command of 28 -> 7, sideslip -12.4 on the reversal that followed (the
				// 2.4.17 roll limit held the roll to 30-51 deg/s and did not prevent it). The measured yaw rate is taken ahead
				// along the measured yaw acceleration by that time (antisymmetric rudder deflection / servo rate), as the 2.2.5
				// pitch lead does for the nozzles. Rudders near neutral = no lead, so a yaw axis that is keeping up is unchanged.
				yawLeadDbg = 0f;
				if (Plugin.YawSlewLead && YawSlewLeadK > 0f && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.Rate))
				{
					// Revoker 0.3.0: the time each yaw effector (rudder, yaw nozzle) needs to swing back through neutral, weighted by
					// its one-side yaw (the yaw the loop will have to wait for is each one's share)
					CollectYawEffs();
					float tLeadSum = 0f;
					float wLeadSum = 0f;
					foreach (YawEff yL in yawEffs)
					{
						YawRange(yL, out float zH, out float zL);
						float wY = 0.5f * Mathf.Abs(zH - zL) * yL.auth;
						tLeadSum += wY * Mathf.Abs(yL.e.now) / Mathf.Max(1f, yL.e.rate);
						wLeadSum += wY;
					}
					if (wLeadSum > 1E-06f)
					{
						float gYL = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - StopSlipA0) / Mathf.Max(1f, StopSlipA1 - StopSlipA0));
						float tauY = YawSlewLeadK * gYL * tLeadSum / wLeadSum;
						// only where the yaw is carried onto or past its command within that time (the error closing by at least
						// half of itself); a yaw rate still building toward a far command is left alone, so roll entries keep their
						// yaw build-up
						float travel = tauY * (vector7.z - rCmdRateF * (MathF.PI / 180f));
						float kY = (Mathf.Abs(vector12.z) > 1E-04f) ? (travel / vector12.z) : ((Mathf.Abs(travel) > 1E-04f) ? 9f : 0f);
						float gO = Mathf.Clamp01((kY - 0.5f) / 0.5f);
						gO = gO * gO * (3f - 2f * gO);
						yawLeadDbg = tauY * gO;
						vector12.z -= yawLeadDbg * vector7.z;
					}
				}
				omegaDotDes = new Vector3(Plugin.KRoll * vector12.x + pDotFF * (MathF.PI / 180f), Plugin.KPitch * vector12.y + 0.5f * qTraj.v * (MathF.PI / 180f), Plugin.KYaw * vector12.z + rDotFF * (MathF.PI / 180f));
				float mDotY = (mLeadInit ? ((vector8.y - mPrevY) / num) : 0f);
				mPrevY = vector8.y;
				mLeadInit = true;
				mDotYF = mDotYF + (mDotY - mDotYF) * (num / (MLeadTau + num));
				vector11 = vector8 + MulI(omegaDotDes - vector7);
				vector11.y -= Plugin.HybridIndiGain * hybridDM;
				wallRollDem = vector11.x;
				wallYawDem = vector11.z;
				vector11.y -= MulI(new Vector3(0f, distLeadAcc, 0f)).y;
				if (Plugin.ReleasedMomentLead > 0f)
				{
					float leadGate = (1f - Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f)) * (1f - Mathf.Clamp01(Mathf.Abs(qTraj.v) / 20f)) * ((Mathf.Abs(qT) < 1f && !prot && !unload) ? 1f : 0f) * stabPitchFrac;
					mLeadGate = Mathf.MoveTowards(mLeadGate, leadGate, num / 0.2f);
					float leadM = Plugin.ReleasedMomentLead * mLeadGate * mDotYF * Mathf.Clamp01(1f - Plugin.HybridIndiGain); // 1.9.0: the hybrid INDI term covers this lag
					float leadErr = vector12.y * 57.29578f;
					mLeadAgree = ((LeadAgreeGate && leadM * leadErr < 0f) ? (1f - Mathf.Clamp01((Mathf.Abs(leadErr) - 1f) / 2f)) : 1f);
					vector11.y += leadM * mLeadAgree;
				}
				Vector3 vector13 = new Vector3(pCmd, qT, rCmd) * (MathF.PI / 180f);
				vector11 += Plugin.CouplingFF * (Vector3.Cross(vector13, MulI(vector13)) - Vector3.Cross(omega, MulI(omega)));
				vector11 += asymFF;
				// 2.0.12: on the wheels the incremental (INDI) roll target is an integrator with nothing to stop it: the gear
				// holds the wings, the measured roll acceleration stays 0, and any roll moment the surfaces make (the mode
				// switch at the rotation speed left one) is kept for the whole roll. Take-off logs: stabs split +7 / -6 deg
				// from 46 m/s to lift-off. While a wheel touches, the surfaces' roll moment is the proportional roll law
				// alone (I x the wanted roll acceleration), with the airframe's own roll moment from the model.
				if (GroundRollP && wheelsTouch)
				{
					float restX = vector8.x;
					foreach (Effector eX in effs)
					{
						if (eX.active)
						{
							restX -= eX.MAt(eX.now).x;
						}
					}
					vector11.x = restX + MulI(new Vector3(omegaDotDes.x, 0f, 0f)).x;
				}
				// 2.0.12: with a rudder gone or an engine out the yaw target can sit far beyond what the jet has (10:29 log,
				// fin shot off: -50..-106 deg/s^2 asked, -12 available). Chasing it, the allocator bought small yaw gains with
				// the stabs and flaperons at any roll / pitch cost (one stab swapping +35 <-> -30 at 0.6 Hz, roll +18 <-> -68
				// with the stick held, pitch rate +-40). Here the yaw target is limited to what the rudders reach with every
				// other surface where it is now, so the rest of the allocation is not traded away for yaw it cannot get.
				decelDbg = 0f;
				yawClampStabDbg = 0f;
				alloc.fightW[1] = (alloc.fightW[2] = (alloc.fightW[3] = 1f));
				if (YawTargetClamp && (yawHealth < 0.999f || engOut || engAsym))
				{
					bool hasRudders = false;
					foreach (SurfaceEffector sR in surfaces)
					{
						// Revoker 0.2.7: a live rudder only. With the fin shot off the rudder's effector stays in the list
						// (inactive), so the canards never joined the first yaw tier, and once the nozzle's yaw was gone too (TVC
						// off, or past its 250 m/s cut-off) the yaw target was clamped to what nothing reached: 28 Oct 21:22 log,
						// fins and wings gone, 247 m/s, the nozzles cut out at 211.0 s and the yaw diverged (r -0.8 -> -128 deg/s
						// in 2 s, sideslip to 85 deg) with both canards held at +1..2 deg.
						if (sR.kind == SurfaceKind.Rudder && (sR.active || !RudderTierLiveOnly))
						{
							hasRudders = true;
							break;
						}
					}
					float restZ = vector8.z;
					float rudLo = 0f;
					float rudHi = 0f;
					float decelLo = 0f;
					float decelHi = 0f;
					float splitLo = 0f;
					float splitHi = 0f;
					foreach (Effector eY in effs)
					{
						if (!eY.active)
						{
							continue;
						}
						SurfaceEffector sY2 = eY as SurfaceEffector;
						float zNow = eY.MAt(eY.now).z;
						restZ -= zNow;
						if ((eY is SurfaceEffector sF && (sF.kind == SurfaceKind.Aileron || sF.kind == SurfaceKind.Flaperon)) || eY is HingeFlapEffector)
						{
							float fl = float.MaxValue;
							float fh = float.MinValue;
							for (int iF = 0; iF < eY.N; iF++)
							{
								fl = Mathf.Min(fl, eY.M[iF].z);
								fh = Mathf.Max(fh, eY.M[iF].z);
							}
							decelLo += Mathf.Min(0f, fl - zNow);
							decelHi += Mathf.Max(0f, fh - zNow);
						}
						// Ifrit: no rudders. The yaw sources are the canted tails and the split decelerons, so the clamp is to
						// what they reach (with the rest where it is now); without this the yaw target froze at the moment
						// the engine quit (engine-out pull at 300 km/h: sideslip 36-65 deg, splits idle).
						// Ifrit 0.4.0 (SplitYawTier): the splits are a tier of their own, after the tails (see below).
						if (!hasRudders && eY is SplitEffector && Plugin.SplitYawTier)
						{
							float zl = float.MaxValue;
							float zh = float.MinValue;
							for (int iY = 0; iY < eY.N; iY++)
							{
								zl = Mathf.Min(zl, eY.M[iY].z);
								zh = Mathf.Max(zh, eY.M[iY].z);
							}
							splitLo += Mathf.Min(0f, zl);
							splitHi += Mathf.Max(0f, zh);
						}
						// Revoker 0.1.0: the nozzle's yaw axis is in the first tier with the rudder (least drag: it costs only the
						// thrust's cosine), so the split flaperons come in only past what rudder + yaw TVC reach
						else if ((eY is SurfaceEffector sY && (sY.kind == SurfaceKind.Rudder || (!hasRudders && sY.kind == SurfaceKind.Stabilator))) || (!hasRudders && eY is SplitEffector) || eY is TvcYawEffector || (sY2 != null && sY2.kind == SurfaceKind.Stabilator && Plugin.CanardYawTier))
						{
							float zl = float.MaxValue;
							float zh = float.MinValue;
							for (int iY = 0; iY < eY.N; iY++)
							{
								zl = Mathf.Min(zl, eY.M[iY].z);
								zh = Mathf.Max(zh, eY.M[iY].z);
							}
							rudLo += zl;
							rudHi += zh;
						}
						else if (YawClampStabReach && eY is SurfaceEffector sS && sS.kind == SurfaceKind.Stabilator && stabPitchFrac < YawClampStabFree)
						{
							// 2.20.0: where the stabs are off the pitch (nozzles carry it, stabPitchFrac -> 0: ~60 deg and up) their
							// differential IS the yaw axis without fins - the reach includes their yaw range, scaled by how free
							// they are. 00:25 log (2.19.0, no fins): the clamp held the yaw target at the flaperons' drag-yaw (+8..+20
							// deg/s^2) against a ~100 demand for 4 s per right-going reversal, stabs idle at -7/+2 with 22-27 deg/s^2
							// of yaw in their tables; the left-going ones were quick only because the roll loop's stab differential
							// happened to yaw the right way. On the pitch (2.0.12's one-fin mid-AoA case) the reach is unchanged.
							float zl = float.MaxValue;
							float zh = float.MinValue;
							for (int iS = 0; iS < eY.N; iS++)
							{
								zl = Mathf.Min(zl, eY.M[iS].z);
								zh = Mathf.Max(zh, eY.M[iS].z);
							}
							float wFree = Mathf.Clamp01((YawClampStabFree - stabPitchFrac) / YawClampStabFree);
							rudLo += zNow + wFree * Mathf.Min(0f, zl - zNow);
							rudHi += zNow + wFree * Mathf.Max(0f, zh - zNow);
							yawClampStabDbg = wFree;
						}
						else
						{
							rudLo += zNow;
							rudHi += zNow;
						}
					}
					// Ifrit 0.4.0 (SplitYawTier, dev 3. Lateral-directional): least drag first. With an engine out (or a damaged
					// yaw axis) the canted tails carry the yaw as long as they can, the split decelerons take the rest, the
					// flaperon crossing (decelF) and the VMC throttle limit come after. 22 Oct 00:35 log, right engine alone at
					// AB, 140 m/s, 3 deg AoA: the left split sat at 15-24 deg for 30 s while the tails held +-2 deg of difference
					// (the split was the allocator's cheapest pure yaw); at the 30 deg cap, 50-85 m/s, the inner/outer flaperons
					// crossed at +-30 with the splits shut (the deflection had taken the split's travel) and the live engine was
					// cut to 30-40 %.
					if (!hasRudders && Plugin.SplitYawTier)
					{
						// the tails' yaw comes with roll (they are canted), which the wing must take back: that is the
						// roll-fight cost's "stabs against the wing" case, so the stab group's fight weight is relaxed by
						// StabYawRelax here. Without it the allocator leaves the yaw to sideslip rather than pay the fight
						// (bench, right engine alone at 500 km/h: 2.5 deg of sideslip held with the tails at +-0.1 deg).
						alloc.fightW[1] = 1f - Mathf.Clamp01(Plugin.StabYawRelax);
						// The split gate opens on measured yaw saturation (the yaw target out of reach with the splits as bounded
						// now, after the tails and the flaperon crossing the allocator finds on its own) and leaks shut slowly
						// (SplitTierFall) once the yaw is met, so it settles on the least split that keeps the yaw met. Bench, yaw
						// per kN of added drag at 250 km/h, 20 deg AoA: tails +-10 deg 4.8 deg/s^2 per kN, inner/outer crossing
						// 1.1-1.5, one split at 30 deg 0.4 (17.7 kN for 7.8 deg/s^2), the VMC throttle cut 0.17 (thrust given up).
						float stYawNow = 0f;
						foreach (SurfaceEffector sS in surfaces)
						{
							if (sS.kind == SurfaceKind.Stabilator && sS.active)
							{
								stYawNow += sS.MAt(sS.now).z;
							}
						}
						float sTgt = Mathf.Clamp01((yawSat - SplitYawSatOpen) / SplitYawSatSpan);
						splitTierF = Mathf.MoveTowards(splitTierF, sTgt, ((sTgt > splitTierF) ? SplitTierRise : SplitTierFall) * num);
						rudLo += splitTierF * splitLo;
						rudHi += splitTierF * splitHi;
						float demSign = Mathf.Sign(stYawNow);
						foreach (SplitEffector sT in splits)
						{
							if (!sT.active || sT.brakeMode)
							{
								continue;
							}
							// gate on the split's travel; with the gate open the split has priority over the host deceleron's
							// deflection for their shared travel (the reverse of the normal DeceleronSplitBudget rule), so the
							// allocator can always open it: the crossing had taken the deceleron's travel first and left the split
							// nothing (22 Oct 00:35 log, 30 deg cap)
							float gateHi = Mathf.Min(sT.max, Mathf.Max(sT.now - sT.rate * num, splitTierF * sT.max));
							sT.hi = Mathf.Max(sT.lo, Mathf.Min(Mathf.Max(sT.hi, gateHi), gateHi));
							sT.cmd = Mathf.Clamp(sT.cmd, sT.lo, sT.hi);
							float sReachSide = (demSign > 0f) ? sT.M[sT.N - 1].z : (0f - sT.M[sT.N - 1].z);
							if (sT.host != null && sT.host.active && splitTierF > 0f && sReachSide > 0f)
							{
								float travel = Mathf.Max(sT.host.max, 0f - sT.host.min);
								float keep = Mathf.Max(0f, travel - Mathf.Max(sT.cmdOut, sT.now));
								sT.host.hi = Mathf.Max(sT.host.lo, Mathf.Min(sT.host.hi, keep));
								sT.host.lo = Mathf.Min(sT.host.hi, Mathf.Max(sT.host.lo, 0f - keep));
								sT.host.cmd = Mathf.Clamp(sT.host.cmd, sT.host.lo, sT.host.hi);
							}
						}
						tierDemDbg = yawSat; tierReachDbg = stYawNow * IaInv[2, 2] * 57.29578f; tierSplitDbg = (splitHi - splitLo) * IaInv[2, 2] * 57.29578f;
					}
					else
					{
						splitTierF = 0f;
					}
					splitTierDbg = splitTierF;
					float zMargin = YawTargetMargin * (rudHi - rudLo);
					float zLo = restZ + rudLo - zMargin;
					float zHi = restZ + rudHi + zMargin;
					// 2.2.7 split flaperons as decelerons: when the yaw asked for is past what the rudders reach (both fins
					// gone, one fin gone, or the live engine's thrust more than the rudders hold), the reach is extended by
					// SplitFlaperonYawUse x the drag yaw the four flaperons have left. The allocator then buys it by opening the
					// flaperons on one wing against each other (inner and outer pair cancelling each other's roll, their drag
					// yawing the jet), with the roll-fight cost relaxed for the inner/outer split in proportion. Only the part
					// of the demand the rudders cannot cover; the VMC throttle limit (which acts on the remaining yaw
					// saturation) comes after it.
					// 2.2.8: lateral stick takes the flaperons back (the split's reach shrinks by SplitFlaperonRollYield x |lateral
					// stick|, held for 1 s after the stick comes back), the roll-fight cost is relaxed for the wing groups only
					// (the stabs still may not roll against the wing), and both follow a rate-limited gate (up 3/s, down 1.5/s)
					// instead of the frame's own saturation. 19 Oct log, both fins gone: the split held all four flaperons at
					// +-25 through rolls, the gate flicked 0.3 <-> 1 frame to frame and the fight cost with it; roll rate
					// swung 5 <-> 40 deg/s against a 35 command, stab difference -46..+3, nozzles chasing the pitch.
					zLoDbg = zLo; zHiDbg = zHi; zTgtDbg = vector11.z;
					float dTgt = 0f;
					decelYieldF = Mathf.Max(Mathf.Clamp01(Mathf.Abs(sr)), Mathf.MoveTowards(decelYieldF, 0f, DecelYieldFall * num));
					if (Plugin.SplitFlaperonYaw && (vector11.z < zLo || vector11.z > zHi))
					{
						float k = Plugin.SplitFlaperonYawUse * (1f - Plugin.SplitFlaperonRollYield * decelYieldF);
						float span = Mathf.Max(1E-06f, k * ((vector11.z < zLo) ? (0f - decelLo) : decelHi));
						float over = (vector11.z < zLo) ? (zLo - vector11.z) : (vector11.z - zHi);
						dTgt = Mathf.Clamp01(over / span);
					}
					decelF = Mathf.MoveTowards(decelF, dTgt, ((dTgt > decelF) ? DecelRise : DecelFall) * num);
					if (Plugin.SplitFlaperonYaw && decelF > 0f)
					{
						float k = Plugin.SplitFlaperonYawUse * (1f - Plugin.SplitFlaperonRollYield * decelYieldF);
						zLo += k * decelF * decelLo;
						zHi += k * decelF * decelHi;
						alloc.fightW[2] = (alloc.fightW[3] = 1f - DecelFightRelax * decelF);
						// 2.2.19 (SplitFlaperonStabRoll): the stabs may take up the wing's roll too, so the flaperons can be set for drag
						// yaw without having to cancel their own roll inside each wing
						alloc.fightW[1] = 1f - Mathf.Clamp01(DecelStabRelax) * decelF;
					}
					decelDbg = decelF;
					// 2.2.22: what the clamp takes away still counts as yaw saturation (the VMC throttle limit watches it). With
					// both fins gone the clamped target was always "reached", yawSat stayed 0 and the limiter never cut the
					// live engine (bench, one engine out + fins gone: departures at 250-600 km/h with the throttle untouched).
					yawClampExcess = Mathf.Abs(vector11.z - Mathf.Clamp(vector11.z, zLo, zHi));
					vector11.z = Mathf.Clamp(vector11.z, zLo, zHi);
				}
				else
				{
					decelF = 0f;
					splitTierF = 0f;
					splitTierDbg = 0f;
				}
				// 2.0.6: with the nose wheel on the runway the measured pitch acceleration stays ~0 whatever the surfaces do,
				// so the incremental (INDI) pitch target would integrate to full nose-up deflection before the nose moves and
				// then have to be unwound at servo rate after it lifts (the over-rotation). While the nose wheel touches, the
				// nose-up pitch moment target may only grow at GroundRotationRamp (deg/s^2 per s), starting from what the
				// surfaces make now; the nose comes up when that moment unloads the leg, with little left over.
				if (noseContactLaw)
				{
					float rampM = Plugin.GroundRotationRamp * (MathF.PI / 180f) / Mathf.Max(1E-06f, IaInv[1, 1]) * num;
					if (!rotMomOn)
					{
						rotMomLim = vector8.y;
						rotMomOn = true;
					}
					rotMomLim = Mathf.Min(rotMomLim + rampM, Mathf.Max(rotMomLim, vector11.y));
					vector11.y = Mathf.Min(vector11.y, rotMomLim);
				}
				else
				{
					rotMomOn = false;
				}
				tvc.lam = Mathf.Lerp(2E-05f, Mathf.Max(2E-05f, Plugin.TvcMoveCost), stabPitchFrac);
				// 2.0.3: with little thrust the nozzles make almost no moment per degree, so the allocator swung them across
				// their travel chasing pitch errors they could not fix (idle at the AoA limit: +-10-13 deg sd). Their movement
				// costs more as the thrust they vector falls.
				tvc.lam *= 1f + TvcLowThrustLam * (1f - Mathf.Clamp01(tw / TvcLowThrustTw));
				// (with the nozzles' thrust near idle their moment per degree is small and the grid's coarse steps were the
				// only thing holding them still; the Newton step had them chase the hold: bench, idle release at 450 km/h,
				// q held +1 deg/s for 2 s)
				// Ifrit 0.4.9: with the pitch stick released only, where the dip is, so a held pull runs 0.4.8's allocation. (Held,
				// it flipped single bench runs between the two stab roll-split branches of the full-pull + roll approach to 65 deg;
				// over a 350-560 km/h sweep the slow branch came up 10 times against 0.4.8's 11, so not a trend either way.)
				Allocator.CoordNewton = (Plugin.AllocNozzleNewton && tw >= NewtonMinTw && (!NewtonReleasedOnly || Mathf.Abs(sp) < 0.18f)) ? 1 : 0;
				TvcBudgetStep(num, vector11);
				// 2.2.14 (TvcNeutralHold, Dev): while the pitch rate is held (pitch-rate command steady, no pitch error, no roll
				// budget in force), the nozzles are drawn back to neutral and the stabs take the steady pitch moment; the
				// pull toward neutral drops at once when the pitch-rate command moves or a pitch error opens, so the nozzles
				// still take the changes. It returns over TvcNeutralTau.
				tvcNeutralDbg = 0f;
				// Revoker 0.2.8: not while the TrailingEdgeBrakeConfig set moves (teBrakeMov): held at neutral the nozzle froze the
				// whole pitch allocation through the retraction (bench, 900 km/h: canards, elevons and nozzle still for 1.6 s with
				// the pitch target unmet, nz down to 0.33); the hold comes back once the set has settled
				if (Plugin.TvcNeutralHold > 0f && tvc.anchorK <= 0f && !prot && !isAI && Mode == FcsMode.AoAG && !teBrakeMov)
				{
					float steadyW = (1f - Mathf.Clamp01(Mathf.Abs(qTraj.v) / TvcNeutralQdot)) * (1f - Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - 1f) / 4f)) * stabPitchFrac;
					tvcNeutralW = (steadyW < tvcNeutralW) ? steadyW : Mathf.MoveTowards(tvcNeutralW, steadyW, num / Mathf.Max(0.05f, TvcNeutralTau));
					if (tvcNeutralW > 0.001f)
					{
						tvc.anchor = 0f;
						tvc.band = TvcNeutralBand;
						tvc.anchorK = Plugin.TvcNeutralHold * tvcNeutralW;
						tvcNeutralDbg = tvcNeutralW;
					}
				}
				else
				{
					tvcNeutralW = 0f;
				}
				// Su-30 0.1.4 (HighAoACanardPitchFirst, Tom 8 Nov: "make the canards take more pitch job away from the tvc so it can
				// focus more on velocity vector roll"). Each canted nozzle has one axis, so what the pair spends in common (pitch) is
				// travel the differential (velocity roll) no longer has. While a roll is asked past 25 deg AoA (the same gate as
				// HighAoAVvRollWeight) the common nozzle deflection is drawn toward neutral, so the canards and stabs carry the pitch
				// first; a pitch error that opens (1 -> 5 deg/s off the pitch-rate target) lets go of it at once, so the nozzles still
				// take what the surfaces cannot. Not in AoA protection.
				canPitchFirstDbg = 0f;
				if (canted != null && tvcd != null && tvcd.active && Plugin.HighAoACanardPitchFirst > 0f && (tvc.anchorK <= 0f || tvc.band > TvcNeutralBand) && !prot && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.GearDownRate))
				{
					float cpA = Mathf.Clamp01((Mathf.Abs(alpha) - CantVvA0) / Mathf.Max(1f, CantVvSpan)) * Mathf.Clamp01(tw / Mathf.Max(0.05f, TvcLowThrustTw));
					cpA *= Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.05f) / 0.25f);
					cpA *= 1f - Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - 1f) / 4f);
					canPitchFirstW = (cpA < canPitchFirstW) ? cpA : Mathf.MoveTowards(canPitchFirstW, cpA, num / Mathf.Max(0.05f, TvcNeutralTau));
					if (canPitchFirstW > 0.001f)
					{
						// (over the roll-time budget's band, which is wide open here: past TvcBudgetAoA it no longer holds the nozzles)
						tvc.anchor = 0f;
						tvc.band = TvcNeutralBand;
						tvc.anchorK = Mathf.Max(tvc.anchorK, Plugin.HighAoACanardPitchFirst * canPitchFirstW);
						canPitchFirstDbg = canPitchFirstW;
					}
				}
				else
				{
					canPitchFirstW = 0f;
				}
				// Su-30 0.1.8 (SurfacesFirstPitch, Tom 9 Nov: "i want the stabs and canards to do more in starting a pitch rate at low
				// to mid aoa when they still have useful pitch authority so the tvc can do more lateral stuffs"). The nozzles' common
				// (pitch) deflection is drawn toward neutral at every pitch rate and pitch error up to SurfacesFirstMaxAoA (fading over
				// the next 10 deg), so the stabs and canards take the pitch and the nozzles only what those cannot. A cost, not a clamp.
				surfFirstDbg = 0f;
				if (Plugin.SurfacesFirstPitch > 0f && tvc != null && tvc.active && (tvc.anchorK <= 0f || tvc.band > TvcNeutralBand) && !prot && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.GearDownRate))
				{
					float sfW = (1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.SurfacesFirstMaxAoA) / 10f)) * Mathf.Clamp01(tw / Mathf.Max(0.05f, TvcLowThrustTw)) * Mathf.Clamp01((ias * 3.6f - Plugin.SurfacesFirstIasLo) / Mathf.Max(1f, Plugin.SurfacesFirstIasHi - Plugin.SurfacesFirstIasLo));
					if (sfW > 0.001f)
					{
						tvc.anchor = 0f;
						tvc.band = TvcNeutralBand;
						tvc.anchorK = Mathf.Max(tvc.anchorK, Plugin.SurfacesFirstPitch * sfW);
						surfFirstDbg = sfW;
					}
				}
				// Ifrit 0.3.0 (TvcRollNeutralHold): the same for the differential (roll) TVC. With the lateral stick and pedals
				// centred, the roll on its command and the sideslip under 2 deg, the nozzles are drawn back to symmetric and the
				// wing surfaces carry the steady roll trim; any roll command, roll error or sideslip lets go of them at once. Seen
				// after lift-off (11:51 log): with the ground law off, the nozzles took the roll trim for a few seconds.
				if (tvcd != null)
				{
					tvcd.anchorK = 0f;
					if (Plugin.TvcRollNeutralHold && Plugin.TvcRollNeutralWeight > 0f && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.GearDownRate))
					{
						float rollSteady = (1f - Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.02f) / 0.08f)) * (1f - Mathf.Clamp01((Mathf.Abs(omega.x * 57.29578f - pCmd) - 2f) / 6f)) * (1f - Mathf.Clamp01((Mathf.Abs(beta) - 2f) / 2f));
						tvcRollNeutralW = (rollSteady < tvcRollNeutralW) ? rollSteady : Mathf.MoveTowards(tvcRollNeutralW, rollSteady, num / Mathf.Max(0.05f, TvcNeutralTau));
						if (tvcRollNeutralW > 0.001f)
						{
							tvcd.anchor = 0f;
							tvcd.band = TvcNeutralBand;
							tvcd.anchorK = Plugin.TvcRollNeutralWeight * tvcRollNeutralW;
						}
					}
					else
					{
						tvcRollNeutralW = 0f;
					}
				}
				// Revoker 0.1.0 (YawTvcNeutralHold): the same for the nozzle's yaw axis. With the lateral stick and pedals steady,
				// the roll on its command and the sideslip under 2 deg (a held turn), the nozzle is drawn back to neutral in yaw
				// and the rudder and canards carry the steady yaw; any lateral command, roll error or sideslip lets go of it at once.
				if (tvcy != null)
				{
					tvcy.anchorK = 0f;
					if (Plugin.YawTvcNeutralHold > 0f && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.GearDownRate))
					{
						float yawSteady = (1f - Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr - srHeldY), Mathf.Abs(sy - syHeldY)) - 0.02f) / 0.08f)) * (1f - Mathf.Clamp01((Mathf.Abs(omega.x * 57.29578f - pCmd) - 2f) / 6f)) * (1f - Mathf.Clamp01((Mathf.Abs(beta) - 2f) / 2f)) * (1f - Mathf.Clamp01((Mathf.Max(Mathf.Abs(pCmd), Mathf.Abs(omega.x * 57.29578f)) - 5f) / 10f));
						// Revoker 0.2.1: only as far as the rudder has travel to take the steady yaw. With a steady yaw load beyond the
						// rudder (in-game log 28 Oct 18:57, right wingtip and a missile gone, ~-10 deg/s^2 at 60-100 m/s) the hold pulled
						// the nozzle to neutral, the rudder sat on its stop, the sideslip grew past 2 deg and let go, the nozzle took the
						// load back and the hold came in again: a 2.3 s cycle, sideslip 1-4.4 deg, rudder +-25. Fades out from 55 to 85 %
						// of the rudder's travel in the direction it is deflected; none with no rudder left.
						yawSteady *= RudderHoldGate();
						srHeldY = Mathf.MoveTowards(srHeldY, sr, num / 0.5f);
						syHeldY = Mathf.MoveTowards(syHeldY, sy, num / 0.5f);
						tvcYawNeutralW = (yawSteady < tvcYawNeutralW) ? yawSteady : Mathf.MoveTowards(tvcYawNeutralW, yawSteady, num / Mathf.Max(0.05f, TvcNeutralTau));
						if (tvcYawNeutralW > 0.001f)
						{
							tvcy.anchor = 0f;
							tvcy.band = TvcNeutralBand;
							tvcy.anchorK = Plugin.YawTvcNeutralHold * tvcYawNeutralW;
						}
					}
					else
					{
						tvcYawNeutralW = 0f;
					}
				}
				float num24 = Mathf.Clamp01(Mathf.Abs(pCmd) / 40f) * stabPitchFrac * (1f - Mathf.InverseLerp(12f, 20f, Mathf.Abs(alpha))) * (1f - Mathf.Clamp01(Mathf.Abs(sp) / 0.15f)) * (1f - Mathf.Clamp01(Mathf.Abs(qT) / 8f));
				float num25 = ((WashK >= 0f) ? WashK : Plugin.RollNozzleHold);
				if (num24 < 0.05f)
				{
					tvcTrimRef += (tvc.cmdOut - tvcTrimRef) * (num / (0.3f + num));
				}
				tvc.lam = Mathf.Max(tvc.lam, 0.05f * Mathf.Clamp01(num25) * num24 * Mathf.InverseLerp(125f, 150f, ias));
				if (num25 > 0f && num24 > 0.02f)
				{
					for (int num26 = 0; num26 < effs.Count; num26++)
					{
						cmdSave[num26] = effs[num26].cmd;
					}
					alloc.Solve(effs, vector11);
					float b = ((WashTau >= 0f) ? WashTau : Plugin.RollNozzleWashout);
					tvcSlow += (tvc.cmd - tvcTrimRef - tvcSlow) * (num / (Mathf.Max(0.02f, b) + num));
					float num27 = 0f;
					foreach (SurfaceEffector surface5 in surfaces)
					{
						if (surface5.kind == SurfaceKind.Stabilator && surface5.active)
						{
							num27 = Mathf.Max(num27, Mathf.Abs(surface5.cmd) / Mathf.Max(1f, surface5.LimitFor(surface5.cmd)));
						}
					}
					float num28 = Mathf.Clamp01((WashUtilHi - num27) / 0.2f);
					float washTarget = Mathf.Clamp01(num25) * num24 * num28 * tvcSlow;
					washApplied = ((Plugin.RollNozzleWashoutSlew > 0f) ? Mathf.MoveTowards(washApplied, washTarget, Plugin.RollNozzleWashoutSlew * num) : washTarget);
					float hi = Mathf.Clamp(tvc.cmd - washApplied, tvc.lo, tvc.hi);
					for (int num29 = 0; num29 < effs.Count; num29++)
					{
						effs[num29].cmd = cmdSave[num29];
					}
					float lo = tvc.lo;
					float hi2 = tvc.hi;
					tvc.lo = (tvc.hi = hi);
					alloc.Solve(effs, vector11);
					tvc.lo = lo;
					tvc.hi = hi2;
					tvcPin = hi;
					tvcPinned = true;
				}
				else
				{
					tvcSlow += (0f - tvcSlow) * (num / (0.2f + num));
					washApplied = ((Plugin.RollNozzleWashoutSlew > 0f) ? Mathf.MoveTowards(washApplied, 0f, Plugin.RollNozzleWashoutSlew * num) : 0f);
					tvcPinned = false;
					alloc.Solve(effs, vector11);
				}
				stabBranch = 0;
				bool branchGate = Plugin.StabBranchSwitch && Plugin.StabBranchMaxAoA > 0f;
				if (branchGate)
				{
					float aAbs = Mathf.Abs(alpha);
					branchGateOn = branchGateOn ? (aAbs <= Plugin.StabBranchMaxAoA + 2f) : (aAbs <= Plugin.StabBranchMaxAoA);
					branchGate = branchGateOn;
				}
				foreach (SurfaceEffector sb in surfaces)
				{
					if (sb.kind != SurfaceKind.Stabilator)
					{
						continue;
					}
					sb.branchCool = Mathf.Max(0f, sb.branchCool - num);
					if (Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.1f)
					{
						sb.branchCool = Mathf.Min(sb.branchCool, BranchCoolTimeStop);
					}
					if (!branchGate || !sb.active)
					{
						sb.branchOn = false;
						continue;
					}
					if (!sb.branchOn && sb.branchCool > 0f)
					{
						continue;
					}
					// 2.0.1: "deep past the peak" and the target are scaled to the stab's own peak (10 and 20 deg were set
					// for Aryx's 33.75 deg foil), and the attached-side target is searched over the stab's whole travel, not
					// this tick's rate-limited window: from -30 deg that window never reached the attached side, so the
					// switch never fired.
					// (with lateral input only: without it the 2.0.0 behaviour is kept)
					bool latIn = Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) >= 0.1f;
					float peakK = (BranchScaleToPeak && latIn) ? Mathf.Clamp(sb.stallAoA / StabPeakRef, 0.3f, 1.5f) : 1f;
					if (Mathf.Abs(sb.LocalAoAAt(sb.cmd)) < sb.stallAoA + (latIn ? BranchDeepMargin : 10f) * peakK)
					{
						continue;
					}
					float bestD = sb.cmd;
					float bestE = float.MaxValue;
					float gLo = (BranchFullTravel && latIn) ? sb.min : sb.lo;
					float gHi = (BranchFullTravel && latIn) ? sb.max : sb.hi;
					for (int gi = 0; gi < sb.N; gi++)
					{
						float gd = sb.grid[gi];
						if (gd < gLo - 0.0001f || gd > gHi + 0.0001f)
						{
							continue;
						}
						float la = Mathf.Abs(sb.LocalAoAAt(gd));
						if (la >= sb.stallAoA)
						{
							continue;
						}
						float e = Mathf.Abs(la - Plugin.StabBranchTarget * peakK);
						if (e < bestE)
						{
							bestE = e;
							bestD = gd;
						}
					}
					if (bestE == float.MaxValue)
					{
						sb.branchOn = false;
						continue;
					}
					float y0 = 0f;
					for (int ci = 0; ci < effs.Count; ci++)
					{
						cmdSave[ci] = effs[ci].cmd;
						if (effs[ci].active)
						{
							y0 += effs[ci].MAt(effs[ci].cmd).y;
						}
					}
					float slo = sb.lo;
					float shi = sb.hi;
					// the stab is walked there through this tick's reachable window, so the allocator plans the nozzles and
					// the other surfaces around where it actually is on the way across its lift peak
					float pinD = (BranchFullTravel && latIn) ? Mathf.Clamp(bestD, sb.lo, sb.hi) : bestD;
					sb.lo = (sb.hi = (sb.cmd = pinD));
					// 2.0.1: the nozzles are free in the branch solve. With the roll-time nozzle budget on, they could not take the
					// stab's nose-down, the pitch check failed every time and the stab stayed parked deep in stall until the AoA
					// was low (in game: right stab at -30 from 29 to 3 deg AoA with full roll, then a roll-rate jump).
					float akSave = tvc.anchorK;
					if (BranchFreeTvc)
					{
						tvc.anchorK = 0f;
					}
					alloc.Solve(effs, vector11);
					tvc.anchorK = akSave;
					sb.lo = slo;
					sb.hi = shi;
					float y1 = 0f;
					for (int ci2 = 0; ci2 < effs.Count; ci2++)
					{
						if (effs[ci2].active)
						{
							y1 += effs[ci2].MAt(effs[ci2].cmd).y;
						}
					}
					float kP = IaInv[1, 1] * 57.29578f;
					bool pitchOk = Mathf.Abs(y1 - vector11.y) * kP <= Mathf.Abs(y0 - vector11.y) * kP + Plugin.StabBranchPitchTol;
					// 2.0.4: with lateral input, nozzles at the end of their travel no longer block the switch when it does not
					// make the modelled pitch error worse: at idle they sit pinned holding against the parked stab, so the test
					// failed every time and the stab stayed in stall (in game, 156-161 s: right stab -30, nozzles +20, AoA 36 -> 0).
					bool tvcRoom = tvc == null || !tvc.active || Mathf.Abs(tvc.cmd) <= 0.8f * Mathf.Max(1f, tvc.max) || (latIn && BranchFreeTvc && Mathf.Abs(y1 - vector11.y) <= Mathf.Abs(y0 - vector11.y));
					if (sb.branchOn)
					{
						sb.branchFail = (pitchOk ? 0f : (sb.branchFail + num));
						if (sb.branchFail > BranchFailTime)
						{
							for (int ci3 = 0; ci3 < effs.Count; ci3++)
							{
								effs[ci3].cmd = cmdSave[ci3];
							}
							sb.branchOn = false;
							sb.branchCool = ((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.1f) ? BranchCoolTimeStop : BranchCoolTime);
						}
						else
						{
							stabBranch++;
						}
					}
					else if (pitchOk && tvcRoom)
					{
						sb.branchOn = true;
						sb.branchFail = 0f;
						stabBranch++;
					}
					else
					{
						for (int ci4 = 0; ci4 < effs.Count; ci4++)
						{
							effs[ci4].cmd = cmdSave[ci4];
						}
						sb.branchCool = ((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.1f) ? BranchCoolTimeStop : BranchCoolTime);
					}
				}
				rollPitchFF = 0f;
				float num30 = Plugin.RollPitchFF * (1f - Mathf.Clamp01((Mathf.Abs(alpha) - (Plugin.RollPitchFFAoA - 15f)) / 15f));
				if (num30 > 0.01f)
				{
					foreach (SurfaceEffector surface6 in surfaces)
					{
						if (surface6.active && surface6.noPitch)
						{
							float c = Mathf.MoveTowards(surface6.now, surface6.cmd, Mathf.Max(1f, surface6.rate) * num);
							rollPitchFF += surface6.PitchRawAt(c) - surface6.PitchRawAt(surface6.now);
						}
					}
					rollPitchFF *= num30;
					float num31 = Plugin.RollPitchFFMax / Mathf.Max(IaInv[1, 1] * 57.29578f, 1E-06f);
					rollPitchFF = Mathf.Clamp(rollPitchFF, 0f - num31, num31);
					if (Mathf.Abs(rollPitchFF) * IaInv[1, 1] * 57.29578f > 0.5f)
					{
						float lam = tvc.lam;
						int num32 = 0;
						if (Plugin.RollPitchFFStabCost > 1.01f)
						{
							tvc.lam = 2E-05f;
							foreach (SurfaceEffector surface7 in surfaces)
							{
								if (surface7.kind == SurfaceKind.Stabilator && surface7.partner != null && num32 < symSave.Length)
								{
									symSave[num32] = surface7.symCost;
									surface7.symCost *= Plugin.RollPitchFFStabCost;
									num32++;
								}
							}
						}
						float lo2 = tvc.lo;
						float hi3 = tvc.hi;
						if (tvcPinned)
						{
							tvc.lo = (tvc.hi = tvcPin);
						}
						int num33 = 0;
						foreach (SurfaceEffector surface8 in surfaces)
						{
							if (surface8.active && surface8.noPitch && num33 < pinLo.Length)
							{
								pinLo[num33] = surface8.lo;
								pinHi[num33] = surface8.hi;
								surface8.lo = (surface8.hi = Mathf.Clamp(surface8.cmd, surface8.lo, surface8.hi));
								num33++;
							}
						}
						alloc.Solve(effs, vector11 - new Vector3(0f, rollPitchFF, 0f));
						num33 = 0;
						foreach (SurfaceEffector surface9 in surfaces)
						{
							if (surface9.active && surface9.noPitch && num33 < pinLo.Length)
							{
								surface9.lo = pinLo[num33];
								surface9.hi = pinHi[num33];
								num33++;
							}
						}
						tvc.lo = lo2;
						tvc.hi = hi3;
						tvc.lam = lam;
						if (Plugin.RollPitchFFStabCost > 1.01f)
						{
							num32 = 0;
							foreach (SurfaceEffector surface10 in surfaces)
							{
								if (surface10.kind == SurfaceKind.Stabilator && surface10.partner != null && num32 < symSave.Length)
								{
									surface10.symCost = symSave[num32];
									num32++;
								}
							}
						}
					}
				}
				RollPitchPass(vector11, num);
				Vector3 zero4 = Vector3.zero;
				dbgYawRud = (dbgYawWing = (dbgYawStab = (dbgYawTvc = 0f)));
				foreach (Effector eff3 in effs)
				{
					if (!eff3.active)
					{
						continue;
					}
					Vector3 vector14 = eff3.MAt(eff3.cmd);
					zero4 += vector14;
					if (DebugYaw)
					{
						float num34 = vector14.z * IaInv[2, 2] * 57.29578f;
						if (eff3.name.Contains("Rudder"))
						{
							dbgYawRud += num34;
						}
						else if (eff3.name.Contains("Flap") || eff3.name.Contains("Aileron"))
						{
							dbgYawWing += num34;
						}
						else if (eff3.name.Contains("Elevator"))
						{
							dbgYawStab += num34;
						}
						else
						{
							dbgYawTvc += num34;
						}
					}
				}
				dbgTargetZ = vector11.z * IaInv[2, 2] * 57.29578f;
				dbgMfZ = vector8.z * IaInv[2, 2] * 57.29578f;
				dbgTgtY = vector11.y * IaInv[1, 1] * 57.29578f;
				dbgMfY = vector8.y * IaInv[1, 1] * 57.29578f;
				float num35 = Mathf.Max((vector11.z - zero4.z) * Mathf.Sign(vector11.z - vector8.z), YawSatCountClamp ? yawClampExcess : 0f);
				yawClampExcess = 0f;
				yawSat = yawSatF.Step(Mathf.Clamp01((num35 * IaInv[2, 2] * 57.29578f - 10f) / 40f), 0.25f, num);
				// 2.7.0: the allocator's own unmet report per axis, signed with the increment it was asked for (deg/s^2): what the
				// effectors at their commands fall short of the target by, toward the side the demand went. Read by the measured
				// pace window as its saturation gate (with the slew still in flight), no surface list, no filter beyond PaceSatTau.
				Vector3 unmetRaw = new Vector3(Mathf.Max(0f, (vector11.x - zero4.x) * Mathf.Sign(vector11.x - vector8.x)) * IaInv[0, 0] * 57.29578f * Mathf.Sign(vector11.x - vector8.x), Mathf.Max(0f, (vector11.y - zero4.y) * Mathf.Sign(vector11.y - vector8.y)) * IaInv[1, 1] * 57.29578f * Mathf.Sign(vector11.y - vector8.y), Mathf.Max(0f, num35) * IaInv[2, 2] * 57.29578f * Mathf.Sign(vector11.z - vector8.z));
				allocUnmet = allocUnmetF.Step(unmetRaw, PaceSatTau, num);
				// the stock-jet capacity estimator (every effector grouped, sideslip-axis read), from this tick's commands (restores them).
				// It drives the governor from this version on, so it runs every tick, logging or not. The last argument is the F-22E's
				// rudder-fade weight, kept for the telemetry; the estimator's own rudW comes from its probes.
				// Revoker 0.8.2: what the allocation left unmade of the INDI target (deg/s^2), before the estimator's probes re-solve
				allocResMain = alloc.Accel(alloc.resDbg);
				// Su-30 0.2.0 (from F-22E 3.0.4): on a worker thread when EstimatorThreaded (the same Step on copies; collected at the top of
				// the next Tick, before anything reads capEst - every reader is in PitchLaw / LateralLaw / the CSV row)
				long tKick0 = System.Diagnostics.Stopwatch.GetTimestamp();
				estJob.Kick(Plugin.EstimatorThreaded, effs, alloc, vector7, IaInv, zero4, num, unmetRaw, alpha, Mathf.Clamp01(rudderAuth) * Mathf.Clamp01(yawHealth / 0.5f));
				phKick += System.Diagnostics.Stopwatch.GetTimestamp() - tKick0;
				float num36 = 10f;
				float num37 = 1f;
				if (tvc.engines.Count > 0 && tvc.engines[0] != null)
				{
					num36 = Mathf.Max(0.1f, Mathf.Abs(Acc.TF_vec(tvc.engines[0]).x));
					float x4 = Acc.TF_vecGain(tvc.engines[0]).x;
					num37 = ((Mathf.Abs(x4) > 0.001f) ? x4 : 1f);
				}
				tvcPitchInput = Mathf.Clamp((0f - tvc.cmdOut) / (num36 * num37), -1f, 1f);
			}
			mTarget = vector11;
			mk[11] = System.Diagnostics.Stopwatch.GetTimestamp();
			if (Mode != FcsMode.Ground)
			{
				FlapPitchLockProject(num);
			}
			else
			{
				flapDiffYield = 0f;
			}
			// Su-30 0.1.0 (CanardSymmetricOnly): the canards move together only
			if (Plugin.CanardSymmetric && canL != null && canR != null)
			{
				float cm = 0.5f * (canL.cmd + canR.cmd);
				canL.cmd = cm;
				canR.cmd = cm;
			}
			float actuatorDamping = Plugin.ActuatorDamping;
			foreach (Effector eff4 in effs)
			{
				if (Mode == FcsMode.Ground || actuatorDamping <= 0.0001f)
				{
					eff4.cmdOut = eff4.cmd;
					eff4.cmdLag = eff4.cmd;
					eff4.outTraj.Reset(eff4.cmd);
				}
				else if (LegacyLag)
				{
					eff4.cmdOut += (eff4.cmd - eff4.cmdOut) * (num / (actuatorDamping + num));
					eff4.outTraj.Reset(eff4.cmdOut);
				}
				else
				{
					float num38 = Mathf.Max(1f, eff4.rate);
					eff4.cmdLag += (eff4.cmd - eff4.cmdLag) * (num / (PreLag + num));
					eff4.outTraj.Step(eff4.cmdLag, num38, num38 / actuatorDamping, num);
					eff4.cmdOut = eff4.outTraj.x;
				}
			}
			tvcRollInput = (tvcd != null && tvcd.active && Mode != FcsMode.Ground) ? tvcd.RollInput(tvcd.cmdOut) : 0f;
			tvcYawInput = (tvcy != null && tvcy.active && Mode != FcsMode.Ground) ? tvcy.YawInput(tvcy.cmdOut) : 0f;
			// Su-30 0.2.3 (GroundTvcRollYaw): stock-vectoring jets (no canted model) get the roll stick and pedals on the ground too
			if (Mode == FcsMode.Ground && Plugin.GroundTvcRollYaw && canted == null)
			{
				tvcRollInput = Mathf.Clamp(sr, -1f, 1f);
				tvcYawInput = Mathf.Clamp(sy, -1f, 1f);
			}
			foreach (HingeFlapEffector hD in hinges)
			{
				if (hD.Valid)
				{
					hD.Drive(hD.active ? hD.cmdOut : hD.symTarget, num);
				}
			}
			if (Mode == FcsMode.Ground)
			{
				float num39 = 10f;
				float num40 = 1f;
				if (tvc.engines.Count > 0 && tvc.engines[0] != null)
				{
					num39 = Mathf.Max(0.1f, Mathf.Abs(Acc.TF_vec(tvc.engines[0]).x));
					float x5 = Acc.TF_vecGain(tvc.engines[0]).x;
					num40 = ((Mathf.Abs(x5) > 0.001f) ? x5 : 1f);
				}
				float num41 = groundBrake * Plugin.GroundBrakeTvc / (num39 * num40);
				tvcPitchInput = Mathf.Clamp(0f - sp - num41, -1f, 1f);
			}
			foreach (SurfaceEffector surface11 in surfaces)
			{
				if (surface11.active)
				{
					surface11.localAoACmd = surface11.LocalAoAAt(surface11.cmd);
				}
			}
			Engaged = true;
			mk[12] = System.Diagnostics.Stopwatch.GetTimestamp();
			if (isAI && Plugin.AIFlightLog)
			{
				AILogStep(num);
			}
			if (!telemetryOpened && Plugin.TelemetryEnabled && !isAI)
			{
				telemetryOpened = true;
				telemetry = new Telemetry(this);
			}
			// F-22E 4.1.3 port: the Telemetry setting takes effect at once (rows stop and resume; the file stays open)
			if (telemetry != null && Plugin.TelemetryEnabled)
			{
				long tRow0 = System.Diagnostics.Stopwatch.GetTimestamp();
				pendingRow = CaptureRow(time, sp, sr, sy, num2);   // Su-30 0.2.0 (F-22E 3.0.4): finished and written at the top of the next Tick
				phRow += System.Diagnostics.Stopwatch.GetTimestamp() - tRow0;
			}
			mk[13] = System.Diagnostics.Stopwatch.GetTimestamp();
			mkDone = true;
		}

		// F-22E 3.0.4: finish the pending CSV row (the estimator job it waits for is joined first) and hand it to the writer thread
		private void FlushPendingRow()
		{
			if (pendingRow == null)
			{
				return;
			}
			estJobF?.Join();
			TelemetryRow r = pendingRow;
			pendingRow = null;
			FinishRow(r);
			telemetry?.Enqueue(r);
		}

		// Ifrit 0.3.1 (FullStickPitchRate): the pitch-rate cap rises from MaxPitchRate to FullStickPitchRate over the last 20 %
		// of aft stick, so full aft pulls faster while every stick position up to 0.8 (and the push side) maps as before
		public float PitchRateCap
		{
			get
			{
				float m = Plugin.MaxPitchRate;
				if (Plugin.FullStickPitchRate > m && sp > 0f && Mode != FcsMode.MPO)
				{
					m = Mathf.Lerp(m, Plugin.FullStickPitchRate, Mathf.InverseLerp(0.8f, 1f, sp));
				}
				return m;
			}
		}

		private float RateBudget()
		{
			float cap = PitchRateCap;
			float num = Mathf.Clamp(5f + 0.15f * ias + 40f * tw, 5f, cap);
			// Ternion 0.1.0 (FlightPathRateFloor): the 40 x T/W term is the nozzles' pitch authority, 0 on a jet without thrust
			// vectoring. Without it the high-AoA budget fell to 5 + 0.15 x IAS (15 deg/s at 250 km/h), below the flight path's own
			// rotation at the AoA limit (21 deg/s at 2.6 g there), so full aft could never hold the limit at low speed: the AoA
			// bled 3-7 deg below it. The budget now never drops below the measured flight-path rate (omegaV) + 3 deg/s: holding an
			// AoA needs q = the path's rate. The AoA protection and the capture braking still stop the nose at the limit.
			if (Plugin.FlightPathRateFloor && Mode != FcsMode.MPO)
			{
				num = Mathf.Min(cap, Mathf.Max(num, Mathf.Abs(omegaV) + 3f));
			}
			// Ternion 0.1.8 (MpoNoVecFullRate): in MPO the 40 x T/W term above is the only thing that kept the budget near the cap
			// on the vectoring jets; on the Ternion (no nozzle pitch, T/W term 0) the MPO pitch-rate command fell to 5 + 0.15 x IAS
			// at high AoA (24 deg/s at 450 km/h, 14 deg/s at 215 km/h) and the jet followed it to the letter: full aft from 450 km/h
			// stopped at 58 deg (Tom, 7 Nov 13:29 log, 420 and 462 s; "mpo supposed to have no aoa limit"). MPO gets the whole cap.
			if (MpoNoVecFullRate && Mode == FcsMode.MPO && noVecJet)
			{
				num = cap;
			}
			float b = Mathf.Clamp((ias - 22f) * 1.1f, 0f, cap);
			float t = Mathf.Clamp01((Mathf.Abs(alpha) - (Plugin.CoupleAoA - 6f)) / 12f);
			return Mathf.Clamp(Mathf.Lerp(Mathf.Max(num, b), num, t), 6f, cap);
		}

		private float AIStickRate()
		{
			float num = Mathf.Max(1f, aiCornerSpeed);
			float num2 = tas * tas * Mathf.Max(0.01f, ac.airDensity) / (num * num * 1.225f);
			float num3 = aiGLimit * 9.81f / Mathf.Max(tas, 0.75f * num) * 57.29578f;
			if (num2 < 1f)
			{
				num3 *= Mathf.Clamp(num2, 0.3f, 1f);
			}
			return num3;
		}

		// 1.8.40: roll-pitch priority. While a roll is commanded (and for RpRelease s after), the flaperons' pitch moment
		// is in the allocator's model instead of being fed forward: the roll's pitch side-effect is then balanced in the
		// same solve that picks the roll deflections, with pitch weighted ~8-15x roll, so the solver takes it off the roll
		// (less flaperon/stab differential) before it pushes the stabs past their lift peak or the nozzles off their trim.
		// The flaperon pair's collective costs RpSymCost x more meanwhile, so they are not used as pitch surfaces.
		internal static float RpEnable = 1f;

		internal static float RpSymCost = 50f;

		internal static float RpRelease = 0.5f;

		internal static float RpAoA0 = 18f;

		internal static float RpAoA1 = 26f;

		private float rpLat;

		public float rpGate;

		internal static float RpPitchW = 10f;

		internal static float RpPitchTol = 4f;

		internal static float RpYawW = 10f;

		internal static float RpErrBand = 0f;

		internal static float RpHpTau = 0.4f;

		internal static float RpHpK = 0f;

		private float rpTvcLp;

		private bool rpLpInit;

		internal static float RpErr0 = 1.5f;

		internal static float RpBandMin = 0.5f;

		public float rpBand = -1f;

		public float rpPitchErr;

		// After the normal allocation: if the nozzles were sent more than RollTvcBudget off their pre-roll trim while a
		// roll is commanded, solve again with the nozzles held inside that band, every surface's true pitch curve
		// (flaperon pitch moment in, stab lift past its peak honest) and pitch weighted RpPitchW x more. The roll is what
		// gives. Accepted only if its modelled pitch error stays within RpPitchTol deg/s^2 of the normal solution's;
		// otherwise the band is widened (x3, x8) and finally the normal solution kept.
		private void RollPitchPass(Vector3 target, float dt)
		{
			rpBand = -1f;
			rpPitchErr = 0f;
			if (tvc == null || !tvc.active)
			{
				return;
			}
			// the band is centred on the pre-roll trim plus the fast (high-passed) part of what the normal solution wants
			// from the nozzles: they keep doing the quick pitch corrections the surfaces cannot follow (reversals, actuator
			// lag) and hand the steady part of the roll's pitch moment to the stabs within RpHpTau.
			float dtR = Mathf.Max(0.001f, dt);
			float tvcMain = tvc.cmd;
			if (!rpLpInit || rpGate <= 0.01f)
			{
				rpTvcLp = tvcMain;
				rpLpInit = true;
			}
			else
			{
				rpTvcLp += (tvcMain - rpTvcLp) * (dtR / (Mathf.Max(0.01f, RpHpTau) + dtR));
			}
			if (rpGate <= 0.01f)
			{
				return;
			}
			float center = tvcAnchor + RpHpK * (tvcMain - rpTvcLp);
			// the band opens with the pitch-rate error: the nozzles stay the fast pitch effector for transients the
			// surfaces cannot follow (roll reversals: rate limits and actuator lag), only the steady roll trim is held off them
			float qErrNow = Mathf.Abs(omega.y * 57.29578f - qT);
			float band0 = Mathf.Max(RpBandMin, Plugin.RollTvcBudget) + (1f - rpGate) * (1f - rpGate) * Mathf.Max(tvc.max, 0f - tvc.min) * RpGateSpan + RpErrBand * Mathf.Max(0f, qErrNow - RpErr0);
			// with afterburner-class thrust the nozzles are the stronger and faster pitch effector: holding them off the
			// trim moves the stabs' reserve for reversal transients into the trim (bench: asym-load AB roll spam overshot
			// the AoA cap by 3-4 deg more). The band widens by RpTwBand between T/W 0.7 and 1.1.
			band0 += RpTwBand * Mathf.Clamp01((tw - TvcBudgetTw0) / Mathf.Max(0.05f, TvcBudgetTwFade));
			// roll stop / reversal (stick against the roll rate): the roll's pitch moment is about to swing the other way
			// faster than the nozzles slew, so they are let go for it
			float pNow = omega.x * 57.29578f;
			float rev = Mathf.Clamp01(-Mathf.Sign(pNow) * sr * 2f) * Mathf.Clamp01((Mathf.Abs(pNow) - 20f) / 40f);
			band0 += RpRevBand * rev;
			rpRevNow = Mathf.Max(rev, rpRevNow - dtR / Mathf.Max(0.05f, RpRevHold));
			// AoA protection / overshoot past the AoA (or asymmetric-load) limit: the nozzles are free again
			band0 += Mathf.Max(prot ? 1f : 0f, pitchNeed) * Mathf.Max(tvc.max, 0f - tvc.min) * 2f;
			float dev = tvc.cmd - center;
			if (Mathf.Abs(dev) <= band0 + 0.05f)
			{
				return;
			}
			int n = effs.Count;
			if (cmdSave.Length < n)
			{
				cmdSave = new float[n * 2];
			}
			for (int i = 0; i < n; i++)
			{
				cmdSave[i] = effs[i].cmd;
			}
			float kP = IaInv[1, 1] * 57.29578f;
			int ns = 0;
			int nl = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				// stabs stay on the attached side of their lift peak in this pass: past it, their pitch moment turns with the
				// roll (the pitch-up this pass exists to avoid) and lags the plan by a stall's worth of travel
				if (RpStallLimit && se.kind == SurfaceKind.Stabilator && se.active && nl < pinLo.Length)
				{
					pinLo[nl] = se.lo;
					pinHi[nl] = se.hi;
					nl++;
					StallWindow(se, se.stallAoA + RpStallMargin);
					// and keep a reserve short of the travel stops for the pitch transients to come (a roll reversal
					// swings the roll's pitch moment the other way faster than the nozzles slew)
					float rlo = se.min + RpStabReserve;
					float rhi = se.max - RpStabReserve;
					if (se.lo < rlo)
					{
						se.lo = Mathf.Min(rlo, se.hi);
					}
					if (se.hi > rhi)
					{
						se.hi = Mathf.Max(rhi, se.lo);
					}
				}
				se.HonestPitchOn();
				if (se.noPitch && se.partner != null && se.kind != SurfaceKind.Rudder && ns < symSave.Length)
				{
					symSave[ns++] = se.symCost;
					se.symCost *= 1f + RpSymCost;
				}
			}
			Vector3 w0 = alloc.axisWeight;
			alloc.axisWeight = new Vector3(w0.x, w0.y * RpPitchW, w0.z * RpYawW);
			float lo0 = tvc.lo;
			float hi0 = tvc.hi;
			// reference: the same solve with the nozzles free - the pitch error the surfaces cannot avoid anyway
			alloc.Solve(effs, target);
			float eA = Mathf.Abs(PitchSum() - target.y) * kP;
			bool ok = false;
			float e = 0f;
			for (int k = 0; k < 3 && !ok; k++)
			{
				float band = band0 * ((k == 0) ? 1f : ((k == 1) ? 3f : 8f));
				for (int i = 0; i < n; i++)
				{
					effs[i].cmd = cmdSave[i];
				}
				float lo = Mathf.Max(lo0, center - band);
				float hi = Mathf.Min(hi0, center + band);
				if (lo > hi)
				{
					lo = (hi = ((center - band > hi0) ? hi0 : lo0));
				}
				tvc.lo = lo;
				tvc.hi = hi;
				alloc.Solve(effs, target);
				e = Mathf.Abs(PitchSum() - target.y) * kP;
				if (e <= eA + RpPitchTol + RpRevTol * rpRevNow)
				{
					ok = true;
					rpBand = band;
				}
			}
			tvc.lo = lo0;
			tvc.hi = hi0;
			alloc.axisWeight = w0;
			ns = 0;
			nl = 0;
			foreach (SurfaceEffector se2 in surfaces)
			{
				if (RpStallLimit && se2.kind == SurfaceKind.Stabilator && se2.active && nl < pinLo.Length)
				{
					se2.lo = pinLo[nl];
					se2.hi = pinHi[nl];
					nl++;
				}
				se2.HonestPitchOff();
				if (se2.noPitch && se2.partner != null && se2.kind != SurfaceKind.Rudder && ns < symSave.Length)
				{
					se2.symCost = symSave[ns++];
				}
			}
			rpPitchErr = e;
			if (!ok)
			{
				for (int i = 0; i < n; i++)
				{
					effs[i].cmd = cmdSave[i];
				}
			}
		}

		internal static bool RpStallLimit = true;

		internal static float RpStallMargin = 0f;

		internal static float RpStabReserve = 0f;

		internal static float RpTwBand = 0f;

		// 1.8.42: on roll stops/reversals the nozzles are no longer let go (RpRevBand 12 -> 0); the pass is instead accepted
		// with up to RpRevTol deg/s^2 more modelled pitch error than free nozzles would leave, so the stabs and roll rate
		// take the reversal's pitch swing and the nozzles stay near trim (2026-09-28 21:18 log: -20 deg and +1.7..3.3 deg
		// over the 12 deg cap on reversals at 600 km/h).
		internal static float RpRevBand = 12f;

		internal static float RpRevTol = 0f;

		internal static float RpRevHold = 0.6f;

		internal static float HonestRollGate = 0f;

		private float rpRevNow;

		internal static float RpGateSpan = 1f;



		internal static float RpMainStallMargin = 3f;


		internal static float StabParkAoA = 30f;

		// 0.4.3 settled stall window (F-22E 2.3.0): like StallWindow, but a surface outside the attached range is brought in at
		// 'rate' deg/s: the window's near edge starts at the surface's own command and moves toward the range, so the
		// allocator re-solves around a small step each frame instead of a 30 deg jump
		internal static float WallStabRollBand = 8f;
		private float wallRollDem;
		private float wallYawDem;

		// Su-30 0.1.9: narrow a surface's allowed window to [pLo, pHi], walking toward it at the servo rate when out of reach
		private static void PinToward(SurfaceEffector sp, float pLo, float pHi, float dt)
		{
			float wLo = Mathf.Max(sp.min, sp.now - sp.rate * dt);
			float wHi = Mathf.Min(sp.max, sp.now + sp.rate * dt);
			float nLo = Mathf.Max(sp.lo, pLo);
			float nHi = Mathf.Min(sp.hi, pHi);
			if (nLo > nHi)
			{
				float tgtP = (sp.now < pLo) ? pLo : pHi;
				nLo = nHi = Mathf.Clamp(tgtP, wLo, wHi);
			}
			sp.lo = nLo;
			sp.hi = nHi;
			sp.cmd = Mathf.Clamp(sp.cmd, sp.lo, sp.hi);
		}

		private static void SettledWindow(SurfaceEffector se, float lim, float rate, float dt)
		{
			float a = float.MaxValue;
			float b = float.MinValue;
			for (int i = 0; i < se.N; i++)
			{
				float g = se.grid[i];
				if (Mathf.Abs(se.LocalAoAAt(g)) <= lim)
				{
					a = Mathf.Min(a, g);
					b = Mathf.Max(b, g);
				}
			}
			if (a > b)
			{
				return;
			}
			float h = 0.5f * se.Step;
			float lo = Mathf.Max(se.lo, a - h);
			float hi = Mathf.Min(se.hi, b + h);
			if (lo > hi)
			{
				return;
			}
			float c = se.cmd;
			float step = Mathf.Max(0.01f, rate * dt);
			if (c < lo)
			{
				lo = Mathf.Min(lo, c + step);
				hi = Mathf.Max(hi, lo);
			}
			else if (c > hi)
			{
				hi = Mathf.Max(hi, c - step);
				lo = Mathf.Min(lo, hi);
			}
			se.lo = Mathf.Max(se.lo, lo);
			se.hi = Mathf.Min(se.hi, hi);
			if (se.lo > se.hi)
			{
				se.lo = (se.hi = Mathf.Clamp(c, se.min, se.max));
			}
		}

		internal static bool SettledWindowRudders = true;

		private bool settledWin;

		public Vector3 allocResMain;

		public bool settledWinDbg;

		// 0.4.6 telemetry: the thrust moment's yaw about the CoM, which dMeasY leaves out (the nozzles are an effector, so their
		// whole moment, the CoM-offset part included, is subtracted there); asymLoadY = dMeasY + asymThrY is the measured load
		// yaw comparable with asymMy (the model's, thrust included). Plus the mass and the body-axis specific force.
		public float nxDbg;

		private float settledUseF;

		private bool settledUseHi;
		internal static float CoordStopErr0 = 3f;





		// 2.3.3 rate-capture effector lag: servo/actuator lag added to the unwind time (s)
		internal static float RateCaptureLagServo = 0.06f;

		public float rateCapLagDbg;
		public Vector3 axWDbg;   // 0.6.0: the allocator axis weights this tick (telemetry)
		public float sweepLimDbg, stopSlipHoldDbg;   // 0.6.0: retired with the 2.6.6 sweep cap and the AoA-rise cap (F-22E 3.0.0 replaced them); kept as columns, always 0

		private static void StallWindow(SurfaceEffector se, float lim)
		{
			float a = float.MaxValue;
			float b = float.MinValue;
			// 2.2.18: the attached window is searched over the surface's whole travel, not only this frame's reach. Searched
			// within the reach, a stab more than one frame's travel past its stall found no attached point and was left
			// alone, so it stayed stalled until the AoA brought it back (19 Oct 20:11 log, 31.5-34.3 s: left stab at -30,
			// local AoA 47 -> 38, for 2.8 s after the release while the jet kept rolling at 3-17 deg/s).
			for (int i = 0; i < se.N; i++)
			{
				float g = se.grid[i];
				if (Mathf.Abs(se.LocalAoAAt(g)) <= lim)
				{
					a = Mathf.Min(a, g);
					b = Mathf.Max(b, g);
				}
			}
			if (a > b)
			{
				return;
			}
			float h = 0.5f * se.Step;
			float lo = Mathf.Max(se.lo, a - h);
			float hi = Mathf.Min(se.hi, b + h);
			if (lo > hi)
			{
				// the attached window is out of this frame's reach: go toward it as far as the surface can
				lo = (hi = ((b + h < se.lo) ? se.lo : se.hi));
			}
			se.lo = lo;
			se.hi = hi;
		}

		// 2.14.0 telemetry: the stabilators' allocator windows and pair costs (index 0 = first stab found, 1 = second), for the no-fin stop diagnosis
		private float StabWin(int which, bool lo)
		{
			int k = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind != SurfaceKind.Stabilator) continue;
				if (k == which) return lo ? se.lo : se.hi;
				k++;
			}
			return 0f;
		}

		private float StabCost(bool anti)
		{
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Stabilator) return anti ? se.antiCost : se.symCost;
			}
			return 0f;
		}

		private float PitchSum()
		{
			float y = 0f;
			foreach (Effector ef in effs)
			{
				if (ef.active)
				{
					y += ef.MAt(ef.cmd).y;
				}
			}
			return y;
		}

		private void RollPitchGateStep(float dt)
		{
			float g = (Plugin.RollPitchPriority && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? RpEnable : 0f;
			float latNow = Mathf.Max(Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.02f) / 0.2f), Mathf.Clamp01(Mathf.Abs(pCmd) / 30f));
			rpLat = Mathf.Max(latNow, rpLat - dt / Mathf.Max(0.05f, RpRelease));
			g *= rpLat * (1f - Mathf.Clamp01((Mathf.Abs(alpha) + stabStallShift - RpAoA0) / Mathf.Max(1f, RpAoA1 - RpAoA0)));
			rpGate = Mathf.Clamp01(g);
		}

		// 1.8.32: during roll/yaw the nozzles keep within RollTvcBudget of where they were before the lateral input
		// (the pitch trim they held for the pilot's own pitch command); the pitch moment the roll itself makes is
		// taken by the stabilators beyond that. A cost in the allocator rather than a clamp: when the stabs cannot
		// supply it (on a stop, stalled) the nozzles still go past the budget, as far as the pitch error needs.
		// The band opens to full travel between 14 and 20 deg AoA, between T/W 0.7 and 1.1, in AoA protection / run-away
		// and while the pitch-rate command is moving (AoA/G capture braking).
		private void TvcBudgetStep(float dt, Vector3 target)
		{
			tvc.anchorK = 0f;
			float budget = Plugin.RollTvcBudget;
			if (!tvcAnchorInit)
			{
				tvcAnchor = tvc.cmd;
				tvcAnchorInit = true;
			}
			float latNow = Mathf.Max(Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.02f) / 0.2f), Mathf.Clamp01(Mathf.Abs(pCmd) / 30f));
			tvcBudgetLat = Mathf.Max(latNow, tvcBudgetLat - dt / Mathf.Max(0.05f, TvcBudgetRelease));
			// Where the stabs can take the pitch (g = 1) the band is RollTvcBudget; it opens toward the full nozzle travel
			// as they lose that ability or the nozzles need the room: AoA from 14 to 20 deg (the stab going trailing-edge
			// down for the roll is then at ~30 deg local AoA, where the F22E_Wing foil's lift curve flattens, and nose-down from it costs roll rate:
			// 13-22 % in 250 km/h roll spam on the bench), AoA protection / run-away, and while the pitch-rate command is
			// moving (AoA/G capture braking keeps full authority).
			float g = 1f;
			g *= 1f - Mathf.Clamp01((Mathf.Abs(alpha) + stabStallShift - TvcBudgetAoA) / Mathf.Max(1f, TvcBudgetAoAFade));
			g *= prot ? 0f : (1f - pitchNeed);
			g *= 1f - Mathf.Clamp01(Mathf.Abs(qTraj.v) / TvcBudgetQdot);
			g *= stabPitchFrac;
			// With afterburner-class thrust the nozzles' roll trim is only a few degrees anyway (1.8.31 bench: -2..-7 mean)
			// and they are the fast pitch effector; holding them there moved that fast work onto the stabs, which rang
			// with 3 frames of control delay (450 km/h AB roll spam: q sd 2.1 -> 3.8 deg/s). Full budget up to T/W 0.7,
			// open by 1.1.
			// Revoker 0.1.0 (CanardPitchPriority): the canards are the faster pitch effector (100 deg/s vs the nozzle's 70)
			// and sit far ahead of the CG, so the budget holds at every thrust
			if (!Plugin.CanardPitchPriority)
			{
				g *= 1f - Mathf.Clamp01((tw - TvcBudgetTw0) / Mathf.Max(0.05f, TvcBudgetTwFade));
			}
			float active = tvcBudgetLat;
			if (budget <= 0f || Plugin.RollTvcBudgetCost <= 0f)
			{
				active = 0f;
			}
			tvcBudgetGate = active * g;
			// the anchor (pre-roll nozzle trim for the pilot's own pitch command) follows the nozzles only while no lateral input is held
			float follow = 1f - Mathf.Clamp01(active / 0.3f);
			tvcAnchor += (tvc.cmd - tvcAnchor) * follow * (dt / (TvcAnchorTau + dt));
			// 2.0.0: with lateral input held, the anchor no longer freezes at the trim from before the roll. It follows the
			// nozzle position the same allocation gives for this tick's pitch demand with no roll or yaw demand: the trim for
			// the pilot's own (possibly changing) pitch command, so only the roll's pitch moment is held off the nozzles. A
			// frozen anchor made the stabs carry every change of pitch trim during a held roll (slow ease from high to low
			// AoA): with the 25 deg stab peak they sat on it for nose-down, the roll stab parked deep in stall, the nozzles
			// pushed against them, and roll rate spiked when the stabs came free (bench, 400 km/h: 141 vs 186 deg/s mean).
			if (AnchorPitchOnly && active > 0.001f && budget > 0f && Plugin.RollTvcBudgetCost > 0f)
			{
				int n = effs.Count;
				if (cmdSave.Length < n)
				{
					cmdSave = new float[n * 2];
				}
				for (int i = 0; i < n; i++)
				{
					cmdSave[i] = effs[i].cmd;
				}
				// the stabs are kept on the attached side of their peak here: a stab parked past it (and the nose-up the
				// nozzles then hold against it) is not the pilot's trim
				int nl = 0;
				foreach (SurfaceEffector se in surfaces)
				{
					if (se.kind == SurfaceKind.Stabilator && se.active && nl < pinLo.Length)
					{
						pinLo[nl] = se.lo;
						pinHi[nl] = se.hi;
						nl++;
						float g0 = se.cmd;
						StallWindow(se, se.stallAoA);
						se.cmd = Mathf.Clamp(g0, se.lo, se.hi);
					}
				}
				alloc.Solve(effs, new Vector3(0f, target.y, 0f));
				nl = 0;
				foreach (SurfaceEffector se in surfaces)
				{
					if (se.kind == SurfaceKind.Stabilator && se.active && nl < pinLo.Length)
					{
						se.lo = pinLo[nl];
						se.hi = pinHi[nl];
						nl++;
					}
				}
				float trimNow = tvc.cmd;
				for (int i = 0; i < n; i++)
				{
					effs[i].cmd = cmdSave[i];
				}
				tvcAnchor += (trimNow - tvcAnchor) * (1f - follow) * (dt / (TvcAnchorTau + dt));
			}
			if (active <= 0.001f)
			{
				return;
			}
			float span = Mathf.Max(tvc.max, 0f - tvc.min);
			// Revoker 0.1.0 (CanardPitchPriority): the band is about neutral, not the pitch-only trim. That trim is the
			// nozzle angle of a pitch-only solve, where the nozzle is the cheapest pitch effector; held there, the nozzle went
			// on carrying the roll's pitch (bench, 550 km/h 0.5-stick roll: anchor drifting to -14, nozzle pinned at -15).
			// With it at neutral the canards take the roll's pitch moment and the pilot's trim; the band still opens where
			// they cannot (their stall band, AoA protection, capture braking).
			tvc.anchor = Plugin.CanardPitchPriority ? 0f : tvcAnchor;
			tvc.band = Mathf.Lerp(span * 2f, budget, active * g);
			tvc.anchorK = Plugin.RollTvcBudgetCost;
		}

		internal static float TvcBudgetRelease = 0.5f;

		// Revoker 0.1.0: 28 / 10 (F-22E 14 / 6). The canard foil's lift keeps rising to 45 deg local AoA (CL 0.9 at 17, 1.0 at
		// 45), so the canard going TE-down for a roll still has nose-down to give well past the F-22E stab's 14-20 deg band.
		// Bench, 0.5-stick rolls at 350-450 km/h (AoA 13-25): nozzle -15 -> -7 deg, pitch-rate error <= 3.8 deg/s.
		internal static float TvcBudgetAoA = 28f;

		internal static bool StabPeakShift = true;

		internal static float StabPeakRef = 33.75f;

		public float stabStallShift;

		internal static float TvcBudgetAoAFade = 10f;

		internal static float TvcBudgetQdot = 25f;

		internal static float TvcBudgetTw0 = 0.7f;

		internal static float TvcBudgetTwFade = 0.4f;

		internal static float TvcAnchorTau = 0.25f;

		internal static bool AnchorPitchOnly = true;

		internal static bool AuthRateLimit = true;

		internal static float TvcLowThrustLam = 0f;

		internal static float TvcLowThrustTw = 0.3f;

		internal static float AuthRateStopAngle = 15f;

		internal static float AuthRateLook = 5f;

		internal static float AuthRateMin = 6f;

		internal static float AuthBrakeFloor = 4f;

		public float authRateUp = 999f;

		public float authRateDn = 999f;

		internal static bool AuthCapFixed = true;

		internal static float AuthCapFixedAoA = 36f;


		private bool authFixedOn;

		// 1.9.0: highest AoA at which the nose-down authority available now (stabs full nose-down, flaperons where they
		// are, nozzles at the present thrust - none above the vectoring cut-off speed) still leaves LowThrustCapMargin.
		internal static bool AuthCapPredict = false;

		private void AuthCapFromSweep()
		{
			if (!Plugin.LowThrustAoACap || Ia[1, 1] <= 1f)
			{
				authCap = 999f;
				return;
			}
			float tvDn = 0f;
			if (tvc.active && tvc.M != null)
			{
				float tvLo = float.MaxValue;
				for (int tk = 0; tk < tvc.N; tk++)
				{
					tvLo = Mathf.Min(tvLo, tvc.M[tk].y);
				}
				if (tvLo < float.MaxValue)
				{
					tvDn = Mathf.Min(0f, tvLo * IaInv[1, 1] * 57.29578f);
				}
			}
			// 1.9.5: nozzle nose-down authority growing (throttle up / spool-up): the cap may follow at once
			if (0f - tvDn > authTvPrev + AuthCapThrustStep)
			{
				authFastT = AuthCapFastHold;
			}
			authTvPrev = 0f - tvDn;
			float margin = Plugin.AuthCapMargin;
			float cap = 999f;
			float prevA = 0f;
			float prevM = float.NaN;
			for (int ia = 0; ia < asymAlphas.Length; ia++)
			{
				float aA = asymAlphas[ia];
				if (aA < 0f)
				{
					continue;
				}
				// Ternion 0.1.2 (AuthCapPredict): the airframe's nose-down authority at the IAS the deceleration will bring (qPredK, the
				// same prediction the lateral brakes use), so the cap comes down ahead of a bleeding pull instead of behind it
				float mDn = 0f - (authNetDn[ia] * (AuthCapPredict ? qPredK : 1f) + tvDn);
				if (mDn < margin)
				{
					float aP = (float.IsNaN(prevM) || prevM <= mDn) ? aA : Mathf.Lerp(prevA, aA, (prevM - margin) / (prevM - mDn));
					cap = Mathf.Max(Plugin.AuthCapFloor, aP - AuthCapLead);
					break;
				}
				prevA = aA;
				prevM = mDn;
			}
			authCap = cap;
		}

		// 2.0.3: nose-down pitch acceleration (deg/s^2) the nozzles give at the present thrust below which they count as
		// not covering the pitch authority (fixed AuthCapFixedAoA limit). 2.0.4: 12 -> 20 (~T/W 0.2; idle is ~9): between T/W 0.13
		// and 0.2 a pull from high IAS overshot the swept cap by 13 deg as it came down with the speed (bench, in game 102-135 s).
		internal static float AuthFixedTvcMin = 20f;

		// 1.9.0: the flaperons' symmetric deflection for the low-thrust cap, followed only while no roll/yaw is commanded:
		// during a roll one flaperon runs into its stop and the pair's mean shifts, which is not a trim the cap should judge by
		private float[] flapSymRef;

		private bool flapSymInit;

		private void FlapSymStep(float dt)
		{
			float[] anti = asymDsym;
			if (anti == null || anti.Length != surfaces.Count)
			{
				return;
			}
			if (flapSymRef == null || flapSymRef.Length != surfaces.Count)
			{
				flapSymRef = new float[surfaces.Count];
				flapSymInit = false;
			}
			bool quiet = Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.05f && Mathf.Abs(omega.x * 57.29578f) < 15f;
			float k = Mathf.Clamp01(dt / 0.3f);
			for (int i = 0; i < surfaces.Count; i++)
			{
				SurfaceEffector se = surfaces[i];
				float sym = se.now - anti[i];
				if (!flapSymInit)
				{
					flapSymRef[i] = sym;
				}
				else if (quiet)
				{
					flapSymRef[i] += (sym - flapSymRef[i]) * k;
				}
			}
			flapSymInit = true;
		}

		// 2.0.10: below the high-AoA regime a stab is not commanded past its lift peak (local AoA within the peak +
		// StabPeakMargin, either sign). Past the peak the lift curve is flat or falling, so further travel buys nothing,
		// and the allocator could not get back: in Tom's 10:00 flight a left roll at ~5 deg AoA drove the right stab to
		// -30 (local 32, peak 25) and it stayed there for 10 s, the rolling-back direction looking worse until it had
		// crossed the peak, with every other surface saturated against it. At 36 deg AoA (stab at its peak by design)
		// the same search swung the stabs +-30 every 1.5 s. The margin opens up from 40 to 50 deg body AoA, so the deep-
		// stall use above that (falling leaf, post-stall roll) is unchanged.
		internal static bool StabPeakClamp = true;


		private void ClampStabToPeak(SurfaceEffector se)
		{
			float x = Mathf.Abs(alpha);
			float lim = se.stallAoA + Plugin.StabPeakMargin + Mathf.Max(0f, x - 40f) * 8f;
			if (lim > 89f || !se.Valid)
			{
				return;
			}
			float l0 = se.LocalAoAAt(0f);
			float slope = (se.LocalAoAAt(10f) - l0) / 10f;
			if (float.IsNaN(slope) || Mathf.Abs(slope) < 0.5f)
			{
				return;
			}
			float d1 = (lim - l0) / slope;
			float d2 = (0f - lim - l0) / slope;
			float dLo = Mathf.Min(d1, d2);
			float dHi = Mathf.Max(d1, d2);
			float lo = Mathf.Max(se.lo, dLo);
			float hi = Mathf.Min(se.hi, dHi);
			if (lo < hi - 1f)
			{
				se.lo = lo;
				se.hi = hi;
			}
		}

		// 2.2.2: the positive no-TVC cap. With DisableTVC the stabs alone hold 45 deg with the thrust up (bench, 160-750 km/h,
		// rolls included); at part throttle, rolling as the speed bled through ~200-250 km/h, 45 departed where 36 held. So
		// NoTvcAoACapPositive (36) up to T/W NoTvcThrustCapTW0, NoTvcThrustAoACapPositive (45) from NoTvcThrustCapTW1.
		internal static float NoTvcThrustCapTW0 = 0.8f;

		// Ternion 0.1.1: the fixed no-TVC AoA caps off on a jet with no vectorable nozzle (bench switch)
		internal static bool NoVecUncapped = true;

		internal static float NoTvcThrustCapTW1 = 1f;

		private float NoTvcCapPosNow()
		{
			if (!Plugin.DisableTVC)
			{
				return Plugin.NoTvcAoACapPos;
			}
			float k = Mathf.Clamp01((tw - NoTvcThrustCapTW0) / Mathf.Max(0.05f, NoTvcThrustCapTW1 - NoTvcThrustCapTW0));
			return Mathf.Lerp(Plugin.NoTvcAoACapPos, Mathf.Max(Plugin.NoTvcAoACapPos, Plugin.NoTvcThrCapPos), k);
		}

		private void AuthCapStep(float dt)
		{
			// 2.0.9: after a change of the airframe test switches the cap follows its new target as fast as it comes down
			// (2.0.8 let it creep up from 36 at the at-cap rise rate, which looked like the old limit was still on)
			string key = Plugin.DisableTVC + "|" + Plugin.AoAFull65 + "|" + Plugin.AoACap36 + "|" + Plugin.NoTvcAoACapPos.ToString("F1") + "|" + Plugin.NoTvcAoACapNeg.ToString("F1") + "|" + Plugin.NoTvcThrCapPos.ToString("F1") + "|" + Plugin.NoTvcThrCapNeg.ToString("F1");
			if (testSwitchKey != key)
			{
				if (testSwitchKey != null)
				{
					// a looser cap is followed as fast as a tighter one (the 1.9.5 thrust-step path)
					authFastT = AuthCapFastHold;
				}
				testSwitchKey = key;
			}
			if (!Plugin.LowThrustAoACap)
			{
				authCapEff = 999f;
				authCapNegEff = -999f;
				authInit = false;
				return;
			}
			// 2.0.9: nozzles not covering pitch: low thrust (T/W under LowThrustTW, released 0.03 above) or DisableTVC
			noTvcOn = Plugin.DisableTVC || (noTvcOn ? (tw < Plugin.LowThrustTW + 0.03f) : (tw < Plugin.LowThrustTW));
			// Ternion 0.1.1: on a jet with no nozzle pitch at all the fixed no-TVC caps are not applied; the swept authority cap
			// (authCap: where the canards and elevators still have nose-down / nose-up authority) and AoALimitPositive / Negative rule
			bool negCapOn = noTvcOn && !Plugin.AoAFull65 && !(noVecJet && NoVecUncapped);
			// 2.2.2: with DisableTVC the negative cap is NoTvcThrustAoACapNegative (45) at every thrust
			float tgtNeg = negCapOn ? Mathf.Max(Plugin.AoANeg, 0f - (Plugin.DisableTVC ? Plugin.NoTvcThrCapNeg : Plugin.NoTvcAoACapNeg)) : -999f;
			if (!authInit || tgtNeg <= authCapNegEff)
			{
				// first step, or released / loosened: straight to the new limit
				authCapNegEff = tgtNeg;
			}
			else
			{
				// tightening toward 0: from the limit that applied so far, as fast as the positive cap comes down
				float from = Mathf.Max(authCapNegEff, Plugin.AoANeg);
				authCapNegEff = from + (tgtNeg - from) * Mathf.Clamp01(dt / AuthCapTauDown);
			}
			float tgt = Mathf.Min(authCap, Plugin.AoAPos);
			// 2.2.19 (FinLossAoACap): with the fins gone the positive AoA is capped where the sideslip stays bounded in a held
			// roll (bench, both fins detached: 35-45 deg holds, 25-30 departs with full roll held, 50-65 lets the sideslip
			// grow to 15-22 deg with the nozzles on). Phased in from one fin lost (yawHealth 0.5) to both (0).
			if (Plugin.FinLossAoACap > 0f && yawHealth < 0.5f)
			{
				float kF = Mathf.Clamp01((0.5f - yawHealth) / 0.5f);
				tgt = Mathf.Min(tgt, Mathf.Lerp(Plugin.AoAPos, Plugin.FinLossAoACap, kF));
			}
			// Revoker 0.2.7 (FinLossNoTvcAoACap): with the fin damaged and no nozzle yaw (TVC off or below the low-thrust T/W), the
			// yaw left is the canards' and the elevons' drag, a few deg/s^2 at high AoA, against a fuselage that is directionally
			// unstable there. 28 Oct 20:31 log (TVC off, fin gone at 685 s): at the 45 deg cap, 50-60 m/s, the sideslip ran
			// -1 -> -75 deg in 10 s with every surface on its stop, then the jet flipped. Bench, fin gone, TVC off, held pull +
			// roll: 45-50 deg departs at 250-300 km/h, 40 departs with the game-like command delay, 35 and below hold (sideslip
			// <= 3.3 deg). Phased in from no damage (yawHealth 1) to the fin gone (0).
			if (Plugin.FinLossNoTvcAoACap > 0f && yawHealth < 0.999f && (noTvcOn || tvcy == null || !tvcy.active))
			{
				float kF2 = Mathf.Clamp01((1f - yawHealth) / 0.5f);
				tgt = Mathf.Min(tgt, Mathf.Lerp(Plugin.AoAPos, Plugin.FinLossNoTvcAoACap, kF2));
			}
			// 2.0.3: when the nozzles can no longer cover the nose-down need (a cap exists at all), the limit is a fixed
			// AuthCapFixedAoA instead of a value sliding with IAS, as long as the authority sweep allows it there;
			// below that the swept cap still rules. Engages when a cap appears below 62 deg, releases above 64.5.
			if (AuthCapFixed)
			{
				// nozzles short of nose-down authority (decided by the thrust you have, so a pull at idle from high IAS is
				// limited from the start instead of the cap coming down onto the jet as the speed bleeds), or a swept cap
				// 2.0.5: judged by thrust-to-weight (Tom: floor T/W 0.3), release 0.03 above
				bool tvShort = authFixedOn ? (tw < Plugin.LowThrustTW + 0.03f) : (tw < Plugin.LowThrustTW);
				// 2.0.8: with the nozzles locked (DisableTVC) they never cover pitch
				tvShort |= Plugin.DisableTVC;
				authFixedOn = tvShort || (authFixedOn ? (authCap < Plugin.AoAPos - 0.5f) : (authCap < Plugin.AoAPos - 3f));
				// Ternion 0.1.1: with no nozzle pitch at all, T/W (vectorable) is always 0, so this fixed NoTvcAoACapPositive (45) capped
				// every limit above it, MPO included (Tom, 7 Nov: AoALimitPositive 50 flew to 45 and no more). The swept cap alone applies.
				if (noVecJet && NoVecUncapped)
				{
					authFixedOn = false;
				}
				// 2.0.8: AoALimit Full65 (debug) turns the fixed 36 deg rule off; the swept authority cap still applies
				if (Plugin.AoAFull65)
				{
					authFixedOn = false;
				}
				if (authFixedOn)
				{
					// 2.0.9: the fixed value is the debug NoTvcAoACapPositive (36 by default)
					tgt = Mathf.Min(tgt, NoTvcCapPosNow());
				}
			}
			if (!authInit)
			{
				authCapEff = tgt;
				authInit = true;
				return;
			}
			if (tgt < authCapEff)
			{
				authCapEff += (tgt - authCapEff) * Mathf.Clamp01(dt / AuthCapTauDown);
				return;
			}
			// 1.9.1: rising, the cap is also rate-limited, and nearly frozen while the jet sits at or above it. In 1.9.0 the
			// exponential rise (up to ~10 deg/s just after a drop) let the capture re-accelerate the nose into the still-
			// rising cap and the recovery push it back: a 1 Hz, 2-18 deg/s pitch-rate cycle at 55-60 deg AoA, half thrust.
			authFastT = Mathf.Max(0f, authFastT - dt);
			if (authFastT > 0f)
			{
				// 1.9.5: the rise comes from more thrust (the nozzles' nose-down authority grew at the last sweeps), not from
				// the airframe - follow it as fast as a drop. 1.9.1-1.9.4 held it to 1 deg/s while the jet sat at the cap, so
				// idle -> max AB with the stick held took ~40 s to give back the full AoA.
				authCapEff += (tgt - authCapEff) * Mathf.Clamp01(dt / AuthCapTauDown);
				return;
			}
			float riseMax = ((alpha > authCapEff - 1f) ? AuthCapRiseAtCap : AuthCapRise) * dt;
			authCapEff += Mathf.Min((tgt - authCapEff) * Mathf.Clamp01(dt / AuthCapTauUp), riseMax);
		}

		private static float SmoothMin(float a, float b, float k)
		{
			float h = Mathf.Clamp01(0.5f + 0.5f * (b - a) / Mathf.Max(0.01f, k));
			return Mathf.Lerp(b, a, h) - k * h * (1f - h);
		}

		private static float Smooth01(float x)
		{
			x = Mathf.Clamp01(x);
			return x * x * (3f - 2f * x);
		}

		// 1.9.0: AoA/G command pre-filter. The reference moves toward the stick's command at a speed that grows with the
		// step (small steps gently, large ones as before), its rate building up over an onset time that also shrinks with
		// the step. Released stick: the reference sits on the jet's present AoA/G, so the next step is measured from there.
		private static float ShapeStep(ref float x, ref float v, ref float mag, float target, float span, float kSmall, float kLarge, float onsetSmall, float dt)
		{
			float d = target - x;
			// the step's size is held (decaying at one span per second) so a large step keeps its speed all the way in
			// instead of slowing down on its last few degrees / tenths of a g as if it were a small one
			mag = Mathf.Max(Mathf.Abs(d), mag - span * dt);
			float r = Mathf.Clamp01(mag / span);
			float s = r * r;
			float k = Mathf.Lerp(kSmall, kLarge, s);
			float ton = Mathf.Lerp(onsetSmall, 0.03f, s);
			float vDes = k * d;
			if (vDes * v < 0f)
			{
				v = 0f;
			}
			float aLim = Mathf.Max(Mathf.Abs(vDes) / Mathf.Max(0.01f, ton), k * k * Mathf.Abs(d));
			v = Mathf.MoveTowards(v, vDes, aLim * dt);
			float xn = x + v * dt;
			if ((target - xn) * d <= 0f)
			{
				xn = target;
				v = 0f;
			}
			x = xn;
			return x;
		}

		private void ShapeCommands(float dt, bool held)
		{
			if (!Plugin.CmdShaping)
			{
				alphaCmd = alphaCmdRaw;
				nCmd = nCmdRaw;
				shapeInit = false;
				return;
			}
			if (!held || !shapeInit)
			{
				aRefX = alpha;
				nRefX = nz;
				aRefV = 0f;
				nRefV = 0f;
				aRefMag = 0f;
				nRefMag = 0f;
				shapeInit = true;
			}
			if (held)
			{
				ShapeStep(ref aRefX, ref aRefV, ref aRefMag, alphaCmdRaw, Plugin.ShapeAoASpan, Plugin.ShapeAoAGainSmall, Plugin.ShapeAoAGainLarge, Plugin.ShapeOnsetSmall, dt);
				ShapeStep(ref nRefX, ref nRefV, ref nRefMag, nCmdRaw, Plugin.ShapeGSpan, Plugin.ShapeGGainSmall, Plugin.ShapeGGainLarge, Plugin.ShapeOnsetSmall, dt);
				alphaCmd = aRefX;
				nCmd = nRefX;
			}
			else
			{
				alphaCmd = alphaCmdRaw;
				nCmd = nCmdRaw;
			}
		}

		private float aRefX;

		private float nRefX = 1f;

		private float aRefMag;

		private float nRefMag;

		private void PitchLaw(float dt, float Vc)
		{
			qB = RateBudget();
			AuthCapStep(dt);
			latReserve = Mathf.Clamp01(Plugin.LateralStabReserve) * Mathf.Clamp01(Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy))) * Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f);
			float aUpP = aUp;
			float aDnP = aDn;
			if (latReserve > 0.001f)
			{
				float tvUp = 0f;
				float tvDn = 0f;
				if (tvc.active && tvc.M != null && tvc.N > 1)
				{
					float tvMax = float.MinValue;
					float tvMin = float.MaxValue;
					for (int tvI = 0; tvI < tvc.N; tvI++)
					{
						tvMax = Mathf.Max(tvMax, tvc.M[tvI].y);
						tvMin = Mathf.Min(tvMin, tvc.M[tvI].y);
					}
					// Revoker 0.2.12 (LatReserveTvcFull): the nozzles' share is their whole travel either side of mid-travel, not
					// what is left from where they are now. Measured from the current nozzle angle, the share shrank as the
					// nozzles moved to brake the pitch (none left at the stop), so the reserve took 70 % of everything and the
					// authority the capture planned with fell 120 -> 42 deg/s^2 mid-stop. The capture then cut the pitch command
					// (48 -> 21 deg/s), the nozzles came off the stop, the authority came back and the command rose again: the
					// AoA stepped in to the negative limit in two or three surges (29 Oct 11:40 log, 90-93 s and 120-123 s, full push +
					// full roll at 150 kt, capture from -34 deg against the -55 limit).
					float tvMid = 0.5f * (tvMax + tvMin);
					if (LatReserveTvcFull)
					{
						tvUp = Mathf.Max(0f, (tvMax - tvMid) * IaInv[1, 1] * 57.29578f);
						tvDn = Mathf.Max(0f, (tvMid - tvMin) * IaInv[1, 1] * 57.29578f);
					}
					else
					{
						tvUp = Mathf.Max(0f, (tvMax - tvc.Mnow.y) * IaInv[1, 1] * 57.29578f);
						tvDn = Mathf.Max(0f, (tvc.Mnow.y - tvMin) * IaInv[1, 1] * 57.29578f);
					}
				}
				aUpP = Mathf.Max(20f, aUp - latReserve * Mathf.Max(0f, aUp - tvUp));
				aDnP = Mathf.Max(20f, aDn - latReserve * Mathf.Max(0f, aDn - tvDn));
			}
			aUpPitch = aUpP;
			aDnPitch = aDnP;
			bool flag = Mode != FcsMode.MPO;
			float num = (flag ? Plugin.AoAPos : 999f);
			float num2 = (flag ? Plugin.AoANeg : (-999f));
			float num3 = (flag ? Mathf.Min(Mathf.Min(num, asymCapPosEff), authCapEff) : num);
			float num4 = (flag ? Mathf.Max(Mathf.Max(num2, asymCapNegEff), authCapNegEff) : num2);
			float num5 = ((Mode == FcsMode.MPO) ? Plugin.MpoGPos : Plugin.GPos);
			float num6 = ((Mode == FcsMode.MPO) ? Plugin.MpoGNeg : Plugin.GNeg);
			float num7 = 9.81f / Vc * 57.29578f;
			float num8 = (dbgWVe = omega.y * 57.29578f - alphaDot);
			alphaDotHoldG = Mathf.Clamp(2f * Mathf.Abs(nz) * Mathf.Max(0f, 0f - vDot) / (Vc * Mathf.Max(nAlpha, 0.02f)), 0f, 15f);
			unload = false;
			prot = false;
			float num9 = 1f;
			float num10 = 0f;
			alphaCmd = 0f;
			nCmd = 1f;
			betaG = 0f;
			if (Mode == FcsMode.AoAG)
			{
				// 1.9.0: hand-over AoA (lower when the wings can barely lift the weight), the stick breakpoint that keeps the
				// AoA scale below it, and how far into the rate-command travel the stick is
				float aTgtRb = Plugin.RateBlendAoA;
				if (nLiftMax < Plugin.RateBlendLiftG)
				{
					aTgtRb = Mathf.Lerp(Mathf.Min(Plugin.RateBlendAoAMin, Plugin.RateBlendAoA), Plugin.RateBlendAoA, Mathf.Clamp01((nLiftMax - 1f) / (Plugin.RateBlendLiftG - 1f)));
				}
				// 1.9.7: the negative side's threshold (magnitude) from the wings' negative-lift capacity
				float aTgtRbN = Plugin.RateBlendAoANeg;
				if (nLiftMin < Plugin.RateBlendLiftG)
				{
					aTgtRbN = Mathf.Lerp(Mathf.Min(Plugin.RateBlendAoAMinNeg, Plugin.RateBlendAoANeg), Plugin.RateBlendAoANeg, Mathf.Clamp01((nLiftMin - 1f) / (Plugin.RateBlendLiftG - 1f)));
				}
				rateBlendAoA = (rbInit ? (rateBlendAoA + (aTgtRb - rateBlendAoA) * Mathf.Clamp01(dt / 0.5f)) : aTgtRb);
				rateBlendAoANeg = (rbInit ? (rateBlendAoANeg + (aTgtRbN - rateBlendAoANeg) * Mathf.Clamp01(dt / 0.5f)) : aTgtRbN);
				rbInit = true;
				// 1.9.1 stick map (fixed AoA per stick: RateBlendAoA at RateBlendStick, 72 deg per full stick by default).
				// The threshold's stick point moves with the threshold (24 deg -> 0.333). Up to it: AoA command on that line.
				// Past it: AoA command on from the threshold to the envelope limit at full stick (full stick = the limit, so
				// the jet goes through the threshold at the rate budget); once the jet crosses the threshold with the stick
				// there, pitch-rate command. With the envelope limit (asymmetric / low-thrust cap) at or below the
				// threshold the whole travel maps onto the limit as before; blended over the 4 deg above it.
				// 1.9.7: the same on the negative side, mirrored. Everything below runs in side space (s = +1 pull, -1 push):
				// stick s*sp, AoA s*alpha, limit s*limit, threshold as a magnitude; the outputs are mapped back with s.
				bool blendP = Plugin.HighAoARateBlend;
				bool blendN = Plugin.HighAoARateBlendNeg;
				float aTP = rateBlendAoA;
				float aTN = rateBlendAoANeg;
				float limP = num3;
				float limN = 0f - num4;
				float gAP = Plugin.RateBlendAoA / Plugin.RateBlendStick;
				StickCurveSetup();
				float gAN = Plugin.RateBlendAoANeg / Plugin.RateBlendStick;
				float zwP = (blendP ? Mathf.Clamp01((limP - aTP) / 4f) : 0f);
				float zwN = (blendN ? Mathf.Clamp01((limN - aTN) / 4f) : 0f);
				float aBP = (blendP ? Mathf.Min(aTP, limP) : limP); // 1.9.5: with the blend off full stick is the AoA limit again (1.9.1-1.9.4 stopped at the threshold)
				float aBN = (blendN ? Mathf.Min(aTN, limN) : limN);
				float sBP = Mathf.Lerp(1f, Mathf.Clamp(StickCurveInv(aTP), 0.05f, 0.95f), zwP);
				float sBN = Mathf.Lerp(1f, Mathf.Clamp(aTN / gAN, 0.05f, 0.95f), zwN);
				// floating hold point: the jet came back down to the threshold out of rate command with the stick past the
				// threshold's point - that stick position is the threshold now; easing below it holds the (live) threshold
				// with a soft knee until the stick meets the threshold's own point on the line, where it is caught
				if (floatOn)
				{
					float fsp = (float)floatSide * sp;
					bool fzone = ((floatSide > 0) ? zwP : zwN) > 0.001f;
					if (!fzone || fsp <= ((floatSide > 0) ? sBP : sBN) || Mathf.Abs(sp) < 0.03f || fsp > 0.98f)
					{
						floatOn = false;
					}
				}
				int sm = ((sp >= 0f) ? 1 : (-1));
				float spS = (float)sm * sp;
				float aT = ((sm > 0) ? aTP : aTN);
				float zw = ((sm > 0) ? zwP : zwN);
				bool zone = zw > 0.001f;
				float aB = ((sm > 0) ? aBP : aBN);
				float sB = ((sm > 0) ? sBP : sBN);
				float limS = ((sm > 0) ? limP : limN);
				float sH = ((floatOn && floatSide == sm) ? Mathf.Max(floatS, sB) : sB);
				float aCmdS;
				if (spS <= sH || sH > 0.999f)
				{
					// 2.0.12: the pull side follows the stick curve (StickFor24degAoA / StickFor36degAoA), scaled so the
					// breakpoint still lands on aB; on the default points it is the 1.9.1 straight line
					aCmdS = ((sm > 0) ? (StickCurve(Mathf.Min(spS, 1f)) * aB / Mathf.Max(StickCurve(Mathf.Max(sB, 0.01f)), 0.01f)) : (aB * Mathf.Min(spS, 1f) / Mathf.Max(sB, 0.01f)));
					if (floatOn && floatSide == sm)
					{
						aCmdS = SmoothMin(aCmdS, aT, FloatKnee);
					}
					aCmdS = Mathf.Min(aCmdS, limS);
				}
				else
				{
					aCmdS = aB + (spS - sH) / (1f - sH) * Mathf.Max(0f, limS - aB);
				}
				alphaCmdRaw = (float)sm * aCmdS;
				nCmdRaw = ((sp >= 0f) ? (1f + sp * (num5 - 1f)) : (1f + sp * (1f - num6)));
				zoneX = ((zone && spS > sH) ? Mathf.Clamp01((spS - sH) / Mathf.Max(0.02f, 1f - sH)) : 0f);
				// 2.1.0 G + pitch-rate law (PitchCommandLaw = GAndPitchRate): no AoA command at all. The AoA branch below is
				// replaced by a pitch-rate command; the G branch and the betaG blend are unchanged. The AoA envelope (limits,
				// low-thrust / asymmetric-load caps, AoA protection) still bounds the rate further down as a limit only.
				bool gRate = Plugin.GRateLaw;
				if (gRate)
				{
					alphaCmdRaw = alpha;
					zoneX = 0f;
					hiRate = false;
					floatOn = false;
				}
				num9 = Mathf.Clamp01((Mathf.Abs(sp) - 0.01f) / 0.06f);
				ShapeCommands(dt, num9 > 0.5f);
				float a = 1f + qB * (MathF.PI / 180f) * Vc / 9.81f;
				gAvail = Mathf.Min(a, nMaxAero);
				betaG = Mathf.Clamp01((gAvail - num5) / 3f + 0.5f);
				if (gRate)
				{
				}
				else if (!hiRate)
				{
					if (zone && zoneX > 0f && (float)sm * alpha >= aT && betaG < 0.5f && num9 > 0.5f)
					{
						hiRate = true;
						hiSide = sm;
						hiAnchor = true;
						hiAS = spS;
						hiAQ = Mathf.Clamp((float)sm * qTraj.x, 0f, qB);
						floatOn = false;
					}
				}
				else
				{
					// 1.9.5: only a pilot ease latches it - not a full pull (full stick always means all the way), and not the
					// jet being pushed down through the threshold by an envelope cap coming down (throttle to idle)
					float hs = hiSide;
					float aTh = ((hiSide > 0) ? aTP : aTN);
					float sBh = ((hiSide > 0) ? sBP : sBN);
					bool zoneh = ((hiSide > 0) ? zwP : zwN) > 0.001f;
					bool capPush = ((hiSide > 0) ? (num3 < num - 0.05f && alphaU > num3 - 1f) : (num4 > num2 + 0.05f && alphaU < num4 + 1f));
					if (hs * alpha < aTh && hs * sp > sBh && hs * sp < 0.95f && !capPush && !floatOn && zoneh)
					{
						floatOn = true;
						floatS = hs * sp;
						floatSide = hiSide;
					}
					if (!zoneh || hs * alpha < aTh - Plugin.RateBlendBand || betaG > 0.6f)
					{
						hiRate = false;
					}
				}
				// 2.1.8 AoAThenRateHold: AoA command until the stick is steady and the AoA has settled on the command, then hold
				// the pitch rate the jet has at that moment (the AoA is left to drift with IAS); a stick movement past
				// CapStickBand returns to AoA command (shaping restarts from the present AoA), a release ends it too.
				bool hyb = !gRate && Plugin.AoAThenRateLaw;
				float stickRate = Mathf.Abs(sp - capSpPrev) / Mathf.Max(dt, 1E-4f);
				capSpPrev = sp;
				capSteadyT = (stickRate < CapStickRate) ? (capSteadyT + dt) : 0f;
				if (!hyb || hiRate || num9 < 0.5f || betaG >= 0.5f)
				{
					capOn = false;
				}
				else if (capOn)
				{
					if (Mathf.Abs(sp - capSp) > CapStickBand)
					{
						capOn = false;
						shapeInit = false;
					}
				}
				else if (capSteadyT >= CapSteadyTime && Mathf.Abs(alphaCmd - alpha) < CapAoABand && Mathf.Abs(alphaDot) < CapAlphaDot && Mathf.Abs(sp) > 0.03f)
				{
					capOn = true;
					capSp = sp;
					capQ = omega.y * 57.29578f;
				}
				bool flag2 = !gRate && !hiRate && !capOn && sp * alpha >= 0f && Mathf.Abs(alpha) > 3f && Mathf.Abs(alphaCmd) < Mathf.Abs(alpha) - 1f;
				bool flag3 = sp * (nz - 1f) >= 0f && Mathf.Abs(nCmd - 1f) < Mathf.Abs(nz - 1f) - 0.3f;
				unload = ((betaG < 0.5f) ? flag2 : flag3);
				if (nz > num5 || nz < num6 || alpha > num || alpha < num2 || num9 < 0.5f)
				{
					unload = false;
				}
				num10 = ((betaG < 0.5f) ? Mathf.Sign(alpha) : Mathf.Sign(nz - 1f));
				float num11 = alphaCmd - alpha;
				float num12 = Mathf.Sign(num11) * Mathf.Min(Plugin.KpAoA * Mathf.Abs(num11), Mathf.Sqrt(240f * Mathf.Abs(num11)));
				if (!gRate && !capOn && betaG < 0.5f && num9 > 0.5f && !flag2 && !hiRate && Mathf.Abs(num11) < 4f && Mathf.Abs(qT) < 0.95f * qB)
				{
					aInt = Mathf.Clamp(aInt + Plugin.KiAoA * num11 * dt, -10f, 10f);
				}
				else
				{
					aInt = Mathf.MoveTowards(aInt, 0f, 20f * dt);
				}
				float a2 = num8 + num12 + aInt;
				if (flag2)
				{
					float num13 = num8 + (NPath(alphaCmd) - NPath(alpha)) * num7;
					float num14 = Mathf.Max(0f, num10 * alphaDot);
					a2 = num13 + (0.15f + 0.45f * (1f - Mathf.Clamp01(Mathf.Abs(num11) / 10f))) * num11 - num10 * Plugin.UnloadRiseGain * num14;
				}
				rateW = 0f;
				if (hiRate)
				{
					// past the threshold: pitch-rate command. The map runs through (0, 0), the entry point (stick, rate at
					// the crossing) and (full stick, budget); a release drops the entry point (zero-rate hold, and the next
					// pull maps 0..full stick straight onto 0..budget).
					// a release or a full pull drops the entry point: from then on 0..full stick = 0..budget
					// 1.9.7: in side space (hiSide), so a push past the negative threshold is the same nose-down rate command
					float hs = hiSide;
					float spH = hs * sp;
					if (Mathf.Abs(sp) < 0.03f || spH > 0.98f)
					{
						hiAnchor = false;
					}
					float qMap;
					if (spH <= 0f)
					{
						qMap = spH * qB;
					}
					else if (hiAnchor && hiAS > 0.05f)
					{
						qMap = ((spH <= hiAS || hiAS > 0.999f) ? (hiAQ * spH / hiAS) : Mathf.Lerp(hiAQ, Mathf.Max(hiAQ, qB), (spH - hiAS) / (1f - hiAS)));
					}
					else
					{
						qMap = spH * qB;
					}
					qMap = Mathf.Min(qMap, qB);
					float aTh = ((hiSide > 0) ? aTP : aTN);
					rateW = Smooth01((hs * alpha - (aTh - Plugin.RateBlendBand)) / Plugin.RateBlendBand);
					a2 = Mathf.Lerp(a2, hs * qMap, rateW);
				}
				if (capOn)
				{
					a2 = capQ;
				}
				if (gRate)
				{
					// stick -> body pitch rate, 0..full stick = 0..the pitch-rate budget. Pull side keeps the pilot's stick
					// shape (section 5 curve, normalised to full stick), push side linear as on the AoA law. Released stick =
					// zero-rate hold (num9), as before.
					float spAbs = Mathf.Min(Mathf.Abs(sp), 1f);
					float frac = (sp >= 0f) ? (StickCurve(spAbs) / Mathf.Max(1f, StickCurve(1f))) : spAbs;
					rateCmdG = Mathf.Sign(sp) * Mathf.Clamp01(frac) * qB;
					// 2.1.7 stick lead (StickLeadGain, default 0 = off): a washed-out term on stick movement, so a quick small
					// input moves the nose a little at once (as an AoA step would) and then settles into the plain rate
					// response. Extra nose movement ~ gain x stick change x budget x tau; slow, smooth stick motion barely
					// triggers it; a held stick gets none.
					float sfr = Mathf.Sign(sp) * Mathf.Clamp01(frac);
					if (!leadInit)
					{
						stickLagF = sfr;
						leadInit = true;
					}
					float tauL = Mathf.Max(0.02f, Plugin.StickLeadTau);
					stickLagF += (sfr - stickLagF) * Mathf.Clamp01(dt / tauL);
					stickLeadDbg = Plugin.StickLeadGain * (sfr - stickLagF) * qB;
					a2 = rateCmdG + stickLeadDbg;
				}
				float num15 = Mathf.Max(nAlpha, 0.1f);
				float b;
				if (flag3)
				{
					float num16 = Mathf.Max(0f, num10 * alphaDot);
					b = (nCmd - eLy) * num7 + 0.3f * Plugin.KpG * (nCmd - nz) / num15 - num10 * Plugin.UnloadRiseGain * num16;
				}
				else
				{
					b = (nCmd - eLy) * num7 + Plugin.KpG * (nCmd - nz) / num15 + gInt + Mathf.Sign(nCmd - 1f) * alphaDotHoldG;
				}
				float num17 = Mathf.Lerp(a2, b, betaG);
				if (betaG > 0.5f && num9 > 0.5f && !unload && Mathf.Abs(qT) < 0.95f * qB && Mathf.Abs(nCmd - nz) < 1.5f)
				{
					gInt = Mathf.Clamp(gInt + Plugin.KiG * (nCmd - nz) / Mathf.Max(nAlpha, 0.1f) * dt, -10f, 10f);
				}
				else
				{
					gInt = Mathf.MoveTowards(gInt, 0f, 10f * dt);
				}
				qRaw = num17 * num9;
			}
			else
			{
				qRaw = sp * (isAI ? Mathf.Min(qB, AIStickRate()) : qB);
				hiRate = false;
				floatOn = false;
				shapeInit = false;
				leadInit = false;
				stickLeadDbg = 0f;
				rateW = 0f;
				zoneX = 0f;
				if (rotating && mainsOnGround && !Plugin.GroundRateLaw)
				{
					qRaw = sp * Mathf.Min(qB, Plugin.RotationRate);
				}
				gInt = 0f;
				aInt = 0f;
				capIUp = (capIDn = 0f);
			}
			float num18 = qB;
			float num19 = 0f - qB;
			// 2.0.3: the pitch-rate budget is IAS/T-W based and knows nothing about what can stop the rate. With little
			// nozzle thrust and the stabs near the end of their authority (idle, high AoA) it allowed rates the surfaces
			// could not stop or regulate. The AoA rate (body pitch rate less the flight-path rate) is now also held to what
			// the braking authority available a little further on can stop within AuthRateStopAngle: nose-down authority
			// (stabs full nose-down, flaperons, nozzles at the present thrust) AuthRateLook deg above the present AoA for
			// a nose-up rate, nose-up authority below it for a nose-down rate. Measured state, no schedule. Only while the
			// low-thrust AoA cap is in force (the nozzles cannot cover the nose-down need); otherwise the old budget.
			authRateUp = authRateDn = 999f;
			if (AuthRateLimit && flag && asymValid && tvc != null && tvc.M != null && tvc.N > 1 && (authFixedOn || authCapEff < Plugin.AoAPos - 0.5f))
			{
				float tvLo = float.MaxValue;
				float tvHi = float.MinValue;
				for (int tvJ = 0; tvJ < tvc.N; tvJ++)
				{
					tvLo = Mathf.Min(tvLo, tvc.M[tvJ].y);
					tvHi = Mathf.Max(tvHi, tvc.M[tvJ].y);
				}
				float tvLoA = (tvc.active ? (tvLo * IaInv[1, 1] * 57.29578f) : 0f);
				float tvHiA = (tvc.active ? (tvHi * IaInv[1, 1] * 57.29578f) : 0f);
				float brDnA = 0f - (AsymCap.Interp(asymAlphas, asymNetDn, asymAlphas.Length, alpha + AuthRateLook) + tvLoA);
				float brUpA = AsymCap.Interp(asymAlphas, asymNetUp, asymAlphas.Length, alpha - AuthRateLook) + tvHiA;
				authRateUp = Mathf.Max(AuthRateMin, Mathf.Sqrt(2f * Mathf.Max(0f, brDnA) * AuthRateStopAngle));
				authRateDn = Mathf.Max(AuthRateMin, Mathf.Sqrt(2f * Mathf.Max(0f, brUpA) * AuthRateStopAngle));
				if (authRateUp < num18 - num8)
				{
					num18 = Mathf.Max(num8 + authRateUp, Mathf.Min(num18, AuthRateMin));
				}
				if (authRateDn < num8 - num19)
				{
					num19 = Mathf.Min(num8 - authRateDn, Mathf.Max(num19, 0f - AuthRateMin));
				}
			}
			capAtLimU = capAtLimD = false;
			capPushU = (capPushD = 0f);
			if (!flag)
			{
				capIUp = (capIDn = 0f);
			}
			if (flag)
			{
				float num20 = num3;
				float num21 = num4;
				if (Mode == FcsMode.AoAG && !Plugin.GRateLaw && !capOn && !unload && num9 > 0.5f && betaG < 0.5f && !hiRate)
				{
					if (alphaCmdRaw > 5f)
					{
						num20 = Mathf.Min(num3, alphaCmdRaw + 0.2f);
					}
					if (alphaCmdRaw < -5f)
					{
						num21 = Mathf.Max(num4, alphaCmdRaw - 0.2f);
					}
				}
				float eCapU = (num20 - alpha) * 0.9f - Mathf.Max(0f, alphaDot) * 0.15f;
				float eCapD = (alpha - num21) * 0.9f - Mathf.Max(0f, 0f - alphaDot) * 0.15f;
				float num22 = Mathf.Max(0f, eCapU);
				float num23 = Mathf.Max(0f, eCapD);
				// 2.0.14: at the envelope limit itself (not the 0.2 deg shelf above an ordinary AoA command) the bound goes
				// on through the limit with the slope it has at zero (3 per unit = 2.7 deg/s per deg), and the integrator
				// trims what the flight-path-rate estimate misses. Only while the pilot asks for more than the limit.
				bool capAtU = num20 >= num3 - 0.01f && Mode == FcsMode.AoAG && num9 > 0.5f;
				bool capAtD = num21 <= num4 + 0.01f && Mode == FcsMode.AoAG && num9 > 0.5f;
				capAtLimU = capAtU && alpha > num20 - 2f;
				capAtLimD = capAtD && alpha < num21 + 2f;
				float capNegU = (capAtU && eCapU < 0f) ? (3f * eCapU) : 0f;
				float capNegD = (capAtD && eCapD < 0f) ? (3f * eCapD) : 0f;
				aBrakeTgt = aDn;
				if (CaptureMode == 1 && Plugin.AoACaptureThrustBrake && asymValid && tvc.M != null && tvc.N > 1)
				{
					float tvLo = float.MaxValue;
					float tvHi = float.MinValue;
					for (int tvJ = 0; tvJ < tvc.N; tvJ++)
					{
						tvLo = Mathf.Min(tvLo, tvc.M[tvJ].y);
						tvHi = Mathf.Max(tvHi, tvc.M[tvJ].y);
					}
					float tvLoA = (tvc.active ? (tvLo * IaInv[1, 1] * 57.29578f) : 0f);
					float tvHiA = (tvc.active ? (tvHi * IaInv[1, 1] * 57.29578f) : 0f);
					float brDn = 0f - (AsymCap.Interp(asymAlphas, asymNetDn, asymAlphas.Length, num20) + tvLoA);
					float brUp = AsymCap.Interp(asymAlphas, asymNetUp, asymAlphas.Length, num21) + tvHiA;
					// 2.0.3: floor 20 -> AuthBrakeFloor: with little authority the capture planned with braking it did not have
					aBrakeTgt = Mathf.Max(AuthBrakeFloor, Mathf.Min(aDn, brDn));
					float aBrakeUp = Mathf.Max(AuthBrakeFloor, Mathf.Min(aUp, brUp));
					// Compass 0.1.5 (AsymCapBrake): with the asymmetric-load AoA cap in force and no pitch TVC, the capture plans the stop at
					// the cap with AsymCapBrakeK of the modelled nose-down there and does not lean toward the authority it has now. 5 Nov
					// 19:04 log (right outer wing gone, cap 12): a full pull from -8 deg reached 30 deg/s of pitch rate on a plan that
					// counted on ~75 deg/s^2 of braking (160 available at -8, blend 0.3); past ~8 deg the stabs were also holding the lost
					// wing's roll (its moment flips sign through zero AoA and reached ~600 deg/s^2 at 12), the AoA ran to 47 and the jet
					// departed (sideslip 50).
					bool asymCapIn = AsymCapBrake && asymCapPosEff < Plugin.AoAPos - 0.5f && (tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC);
					if (asymCapIn)
					{
						// Compass 0.1.7: less the nose-up the cap's model leaves out (AsymPitchBias)
						aBrakeTgt = Mathf.Max(AuthBrakeFloor, Mathf.Min(aDn, brDn - (AsymPitchBias ? asymPitchBias : 0f)) * AsymCapBrakeK);
					}
					else
					{
						aBrakeTgt += Mathf.Clamp01(Plugin.AoACaptureBrakeBlend) * Mathf.Max(0f, aDn - aBrakeTgt);
					}
					aBrakeUp += Mathf.Clamp01(Plugin.AoACaptureBrakeBlend) * Mathf.Max(0f, aUp - aBrakeUp);
					// 2.1.10 rolling capture: in a roll the capture planned with braking the jet did not have - the stabs are
					// partly spent on the roll (aDnPitch) and the roll's inertial pitch-up, (Iyaw - Iroll) / Ipitch x p r, works
					// against the stop. At the 45 deg limit the AoA overshot 3-6 deg, protection cut the rate to zero, the AoA fell
					// 5-10 deg below and the cycle repeated every ~1.2 s (bench 520 km/h entry; 13 Oct log 92-95 s). While a
					// velocity-vector roll runs, the braking is taken as what is left for pitch less the coupling, and the
					// capture's near-limit gain comes down with it.
					float kU = 3f;
					float kD = 3f;
					if (RollCapture && Plugin.CoupGateAtLimitOn && Ia[1, 1] > 1f)
					{
						float saR = Mathf.Sin(alpha * (MathF.PI / 180f));
						float caR = Mathf.Cos(alpha * (MathF.PI / 180f));
						float pSr = Mathf.Abs((omega.x * caR + omega.z * saR) * 57.29578f);
						float lg = Mathf.Clamp01((pSr - 10f) / 20f);
						if (lg > 0f)
						{
							float coupA = (Ia[2, 2] - Ia[0, 0]) / Ia[1, 1] * omega.x * omega.z * 57.29578f;
							float bU = Mathf.Max(AuthBrakeFloor, Mathf.Min(aBrakeTgt, aDnPitch) - RollCaptureCoupK * Mathf.Max(0f, coupA));
							float bD = Mathf.Max(AuthBrakeFloor, Mathf.Min(aBrakeUp, aUpPitch) - RollCaptureCoupK * Mathf.Max(0f, 0f - coupA));
							bU = Mathf.Lerp(aBrakeTgt, Mathf.Min(aBrakeTgt, bU), lg);
							bD = Mathf.Lerp(aBrakeUp, Mathf.Min(aBrakeUp, bD), lg);
							kU = 3f * Mathf.Sqrt(bU / Mathf.Max(1f, aBrakeTgt));
							kD = 3f * Mathf.Sqrt(bD / Mathf.Max(1f, aBrakeUp));
							aBrakeTgt = bU;
							aBrakeUp = bD;
						}
					}
					// 2.2.1: the stop also takes the pitch effectors' lag (CapLagT, scaled by the nozzles' share of the authority:
					// they slew ~70 deg/s across their travel and carry most of it at low IAS). Stopping distance q tau + q^2 / 2a
					// instead of q^2 / 2a: a re-pull at ~200 km/h from 38 deg came in at 30 deg/s against a bound that assumed the
					// jet could stop at once, and overshot the 45 deg limit by 1.5 deg (13 Oct 10:46 log, 107 s; bench 46.4).
					float sqU = Mathf.Sqrt(1.4f * aBrakeTgt * num22);
					float sqD = Mathf.Sqrt(1.4f * aBrakeUp * num23);
					if (CapLagT > 0f && Plugin.CapLagOn)
					{
						float tauC = CapLagT * CapTvcShare();
						if (tauC > 0.001f)
						{
							float aU = 0.7f * Mathf.Max(1f, aBrakeTgt);
							float aD = 0.7f * Mathf.Max(1f, aBrakeUp);
							sqU = aU * (Mathf.Sqrt(tauC * tauC + 2f * num22 / aU) - tauC);
							sqD = aD * (Mathf.Sqrt(tauC * tauC + 2f * num23 / aD) - tauC);
						}
					}
					capBoundU = num8 + Mathf.Min(sqU, kU * num22);
					capBoundD = num8 - Mathf.Min(sqD, kD * num23);
					// Su-30 0.1.15 (CaptureMeasured, Tom 9 Nov 20:19: "measure how much pitch decel and accel the jet has so it could
					// stop on the aoa limit right on time without being too slow or too fast then overshoots"). Replaces the planned
					// braking above (the modelled stop at the limit blended toward aDn, the roll-coupling cut, the 0.7 margin, the
					// 3/s near-limit gain cap and the AoA-rate lead) with what the jet measures it has:
					// - braking now = the measured pitch acceleration plus the swing the effectors can still deliver from where they
					//   are with what the jet is doing in roll and yaw kept (PitchReach: a surface pair moves only its common part, the
					//   nozzles' common part only up to max - |differential|) - the airframe's own moment, the roll's inertial
					//   coupling and the thrust are all in it as they are now;
					// - braking at the limit = that, plus how much the airframe's own moment changes between here and the limit
					//   (the live sweep, surfaces at their extreme); the stop uses the mean of the two;
					// - the delay before it arrives = each effector's swing time at its own slew, plus the command path (the pitch
					//   trajectory's jerk ramp and the rate loop's tracking lag);
					// - closing rate = the most that can still be stopped on the limit: a (sqrt(tau^2 + 2 e / a) - tau). Near the
					//   limit that is e / tau, so the last degrees close at the rate the effectors can follow, not a set gain.
					// Bench (calibrated, 70 limit): full-aft pulls 150-550 km/h reach 69.8 deg 0.3-0.8 s sooner, rolling pulls
					// 0.05-0.8 s sooner, peaks 70.0-70.5 as before. Without the roll-kept swing and the command-path lag the first try
					// overshot rolling pulls to 73-75 deg: the tables promised ~165 deg/s^2 and the jet delivered ~100.
					// 9 Nov 20:00 log, 54.6-55.6 s (pull from 441 km/h): the old plan braked on 60-118 deg/s^2 against 160-200 the jet
					// had, the command fell 60 -> 18 deg/s at 49-61 deg, the jet over-braked to q -1 at 60 deg and took 0.5 s to
					// get going again; in the rolling pulls of that log and the 18:24 one the last 4 deg took 1-1.5 s at k 1.1-2.4/s.
					if (Plugin.CaptureMeasured)
					{
						PitchReach(out float swDn, out float tDn, out float swUp, out float tUp);
						float bDnNow = Mathf.Max(AuthBrakeFloor, swDn - qDotMeasDeg);
						float bUpNow = Mathf.Max(AuthBrakeFloor, swUp + qDotMeasDeg);
						float bDn = bDnNow;
						float bUp = bUpNow;
						if (asymValid)
						{
							int nA = asymAlphas.Length;
							float dDnLim = AsymCap.Interp(asymAlphas, asymNetDn, nA, num20) - AsymCap.Interp(asymAlphas, asymNetDn, nA, alpha);
							float dUpLim = AsymCap.Interp(asymAlphas, asymNetUp, nA, num21) - AsymCap.Interp(asymAlphas, asymNetUp, nA, alpha);
							bDn = 0.5f * (bDn + Mathf.Max(AuthBrakeFloor, bDnNow - dDnLim));
							bUp = 0.5f * (bUp + Mathf.Max(AuthBrakeFloor, bUpNow + dUpLim));
						}
						// the command itself reaches the effectors late too: the pitch trajectory's jerk ramp (half of its 0.15 s) and the
						// rate loop's tracking lag (the share of the trajectory's deceleration not fed forward, over PitchRateGain)
						float tLoop = 0.075f + 0.5f / Mathf.Max(0.5f, Plugin.KPitch);
						tDn += tLoop;
						tUp += tLoop;
						float eMU = Mathf.Max(0f, num20 - alpha);
						float eMD = Mathf.Max(0f, alpha - num21);
						float vU = bDn * (Mathf.Sqrt(tDn * tDn + 2f * eMU / bDn) - tDn);
						float vD = bUp * (Mathf.Sqrt(tUp * tUp + 2f * eMD / bUp) - tUp);
						capBoundU = num8 + vU;
						capBoundD = num8 - vD;
						capTauDnDbg = tDn;
						capTauUpDbg = tUp;
						capBrkDnDbg = bDn;
						capBrkUpDbg = bUp;
					}
				}
				else
				{
					capBoundU = num8 + Mathf.Min(Mathf.Sqrt(2f * aDn * num22), 3f * num22);
					capBoundD = num8 - Mathf.Min(Mathf.Sqrt(2f * aUp * num23), 3f * num23);
				}
				capBoundU += capNegU;
				capBoundD -= capNegD;
				// integrators: while the pilot's command is held back by this bound near the limit. Past the limit they
				// learn whenever the AoA rate is small. Short of it only after CapIHoldT of continuous hold within CapIBand:
				// the proportional capture converges from below, and learning on that approach banked error that carried
				// the jet over the limit on arrival. They keep their value through transients and bleed off otherwise.
				bool bindU = capAtU && Mathf.Abs(num20 - alpha) < 2f && qRaw >= capBoundU + capIUp - 0.5f;
				bool bindD = capAtD && Mathf.Abs(alpha - num21) < 2f && qRaw <= capBoundD + capIDn + 0.5f;
				bool capSettled = Mathf.Abs(alphaDot) < CapIAlphaDot;
				capHoldTU = (bindU && Mathf.Abs(num20 - alpha) < CapIBand) ? (capHoldTU + dt) : 0f;
				capHoldTD = (bindD && Mathf.Abs(alpha - num21) < CapIBand) ? (capHoldTD + dt) : 0f;
				if (bindU)
				{
					float eI = num20 - alpha;
					if (capSettled && capHoldTU > 0f && (eI < 0f || capHoldTU > CapIHoldT))
					{
						capIUp = Mathf.Clamp(capIUp + CapKi * eI * dt, 0f - CapIMax, CapIMax);
					}
				}
				else
				{
					capIUp = Mathf.MoveTowards(capIUp, 0f, 8f * dt);
				}
				if (bindD)
				{
					float eI = num21 - alpha;
					if (capSettled && capHoldTD > 0f && (eI > 0f || capHoldTD > CapIHoldT))
					{
						capIDn = Mathf.Clamp(capIDn + CapKi * eI * dt, 0f - CapIMax, CapIMax);
					}
				}
				else
				{
					capIDn = Mathf.MoveTowards(capIDn, 0f, 8f * dt);
				}
				capBoundU += capIUp;
				capBoundD += capIDn;
				capPushU = Mathf.Max(0f, num8 - capBoundU);
				capPushD = Mathf.Max(0f, capBoundD - num8);
				num18 = Mathf.Min(num18, capBoundU);
				num19 = Mathf.Max(num19, capBoundD);
			}
			if (tas > 25f)
			{
				float num24 = Mathf.Max(nAlpha, 0.02f);
				float num25 = ((nz > 1f) ? alphaDotHoldG : 0f);
				float num26 = ((nz < 0f) ? alphaDotHoldG : 0f);
				float num27 = 0.15f + RollLoadLead * Mathf.Clamp01(Mathf.Abs(omega.x * 57.29578f) / 150f);
				float num28 = (num5 - nz) / num24 - Mathf.Max(0f, alphaDot - num25) * num27;
				float num29 = (nz - num6) / num24 - Mathf.Max(0f, 0f - alphaDot - num26) * num27;
				float num30 = ((num28 > 0f) ? Mathf.Min(Mathf.Sqrt(2f * aDn * num28), 3f * num28) : (3f * num28));
				float num31 = ((num29 > 0f) ? Mathf.Min(Mathf.Sqrt(2f * aUp * num29), 3f * num29) : (3f * num29));
				// Ifrit 0.3.1 (GLimitAccelLead): the hold terms above cover a jet slowing down (the AoA has to keep rising to
				// hold the load); speeding up, the load grows at a fixed AoA, so the AoA has to come back toward zero by
				// 2 |n| vDot / (V n_alpha) just to stay at the limit. Without it a push held in an accelerating dive sat
				// 0.3-0.4 g past GLimitNegative (20 Oct 14:03 log 438-446 s: -3.8 g, -4.4 g peak; bench full push from
				// 360 km/h: -3.87 g steady).
				float accHold = GLimitAccelLead ? Mathf.Clamp(2f * Mathf.Abs(nz) * Mathf.Max(0f, vDot) / (Vc * num24), 0f, 15f) : 0f;
				// Su-30 0.1.9 (GCaptureReachable): the load-limit capture brakes the pitch rate only as far as the limit can be
				// reached. It predicts the AoA left to the limit on the linear lift slope (nAlpha); where the airframe's own
				// maximum load at this airspeed (nMaxAero, its lift peak) is below the limit, the jet runs out of lift first
				// and the AoA limit takes over, but the capture still braked as if 9 g were 8-15 deg away: in pulls from low
				// AoA at 340-600 km/h the pitch-rate command came back 8-24 deg/s for ~0.3 s and the pitch rate dipped 9-21
				// deg/s (9 Nov 09:59 log, 9 of 21 pulls; bench 480 km/h: command 58 -> 37 -> 60, nMaxAero 6-8). Faded out
				// from the limit + 0.5 g down to the limit - 0.5 g of reachable load.
				// Not with roll / yaw stick in (faded back over 0.1-0.4 of stick): in a rolling pull the faster AoA rise outran the
				// yaw that coordinates the roll (bench, full aft + full roll from 480 km/h: peak sideslip 11.4 -> 16.0 deg).
				float gcW = Plugin.GCaptureReachable ? Mathf.Clamp01((nMaxAero - num5 + 0.5f) / 1f) : 1f;
				if (!Plugin.GCaptureReachableRolling)
				{
					gcW = Mathf.Lerp(gcW, 1f, Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.1f) / 0.3f));
				}
				gcReachDbg = gcW;
				// Su-30 0.2.0 (GCaptureMeasured, Tom 9 Nov 21:28: rolling pulls from above ~300 kt hitch in roll or pitch): the load-limit
				// capture on the same measured stopping law as the AoA capture (0.1.15): the AoA left to the load limit on the lift slope,
				// closed no faster than the measured nose-down braking can stop after the effectors' swing and the command path's lag,
				// a (sqrt(tau^2 + 2 e / a) - tau). The old bound subtracted an AoA-rate lead (0.15 s, up to 0.35 s rolling) from the AoA
				// left and closed the rest at 3 /s; the pitch rate's own overshoot fed that lead, so the bound fell, the jet slowed, the
				// lead shrank and the bound rose again: full aft + full roll at 580-645 km/h (9 Nov 21:04 log, 152.8 / 161.6 / 332.3 /
				// 390.6 s; 21:11 log 165.3 / 337.5 s) ran the pitch-rate command 50 -> 20 -> 55 deg/s and the pitch rate 46 -> 23 -> 52
				// inside 0.6 s, with the roll trimmed 111 -> 76 deg/s over the same dip. Straight pulls (the reachable-load fade) are
				// unchanged; past the limit the push-back is the old 3 /s.
				if (Plugin.GCaptureMeasured && nz >= num5)
				{
					num30 = 3f * (num5 - nz) / num24;   // past the limit: the old push-back, without the AoA-rate lead
				}
				else if (Plugin.GCaptureMeasured)
				{
					PitchReach(out float gSwDn, out float gTDn, out float gSwUp, out float gTUp);
					// the braking measured now, low-passed over 0.1 s (the measured pitch acceleration steps with every effector reversal; one
					// tick of it took the bound 41 -> 22 deg/s on the bench)
					float gBNow = Mathf.Max(AuthBrakeFloor, gSwDn - qDotMeasDeg);
					gCapBF = gCapBInit ? (gCapBF + (gBNow - gCapBF) * Mathf.Clamp01(dt / 0.1f)) : gBNow;
					gCapBInit = true;
					float gB = Mathf.Max(AuthBrakeFloor, gCapBF);
					// plus the old capture's roll lead (RollLoadLead, up to 0.2 s at 150 deg/s): in a fast roll the inertial pitch-up arrives with the load
					float gT = gTDn + 0.075f + 0.5f / Mathf.Max(0.5f, Plugin.KPitch) + GCapRollLeadK * RollLoadLead * Mathf.Clamp01(Mathf.Abs(omega.x * 57.29578f) / 150f);
					float eG = Mathf.Max(0f, (num5 - nz) / num24);
					num30 = gB * (Mathf.Sqrt(gT * gT + 2f * eG / gB) - gT);
					gCapTDbg = gT;
					gCapBDbg = gB;
					gCapEDbg = eG;
				}
				if (gcW > 0.001f)
				{
					num18 = Mathf.Min(num18, num8 + num25 + num30 / gcW - ((nz > 1f) ? accHold : 0f));
				}
				num19 = Mathf.Max(num19, num8 - num26 - num31 + ((nz < 0f) ? accHold : 0f));
				// 1.9.0: only while the load is being taken off (the stick eased, or the shaped G command still coming down to
				// it). In a steady hold its 0.15 g dead zone pinned the load 0.15-0.2 g above the command once the hybrid
				// pitch loop removed the oscillation that used to hide it.
				bool gUnderGate = unload || nCmdRaw < nCmd - 0.05f || !Plugin.CmdShaping;
				if (Mode == FcsMode.AoAG && betaG > 0.5f && Plugin.GUndershootCapture > 0f && gUnderGate)
				{
					float num32 = (nz - nCmd - 0.15f) / num24;
					float num33 = ((num32 > 0f) ? Mathf.Min(Mathf.Sqrt(2f * aUp * num32), 3f * num32) : (3f * num32));
					if (num32 > 0f || !GUnderAboveOnly)
					{
						num19 = Mathf.Max(num19, num8 - num33 / Mathf.Clamp01(Plugin.GUndershootCapture));
					}
					gUnderDbg = num33;
				}
				else
				{
					gUnderDbg = 0f;
				}
			}
			if (protHold && (!flag || Mode != FcsMode.AoAG || Mathf.Abs(sp) < 0.7f || (protHoldSide > 0f && alphaU < num - 5f) || (protHoldSide < 0f && alphaU > num2 + 5f)))
			{
				protHold = false;
			}
			if (flag && Mode == FcsMode.AoAG && !protHold && lowSpeedAxisW <= 0.5f)
			{
				if (alphaU > num + 5f && sp > 0.85f)
				{
					protHold = true;
					protHoldSide = 1f;
				}
				else if (alphaU < num2 - 5f && sp < -0.85f)
				{
					protHold = true;
					protHoldSide = -1f;
				}
			}
			// Su-30 0.1.11 (LowSpeedTvcRoll): the AoA limit stays in force in that band, so full aft does not latch the zero-rate hold
			if (protHold && Mode == FcsMode.AoAG && lowSpeedAxisW > 0.5f)
			{
				protHold = false;
			}
			float num34;
			// 1.9.5: the pilot's own command within the normal bounds - the AoA protection may not hold it back from a
			// command that is already beyond protection's own (more nose-down above the limit, more nose-up below the
			// negative one). Before, protection set both bounds to its command, and from 1.9.3 (soft onset) that command is
			// ~the flight-path rate just past the limit: full forward stick at 65.5 deg still got +8..+12 deg/s nose-up.
			float qPilot = ((!(num19 > num18)) ? Mathf.Clamp(qRaw, num19, num18) : ((alpha >= 0f) ? num18 : num19));
			if (protHold)
			{
				num34 = 0f;
				num18 = (num19 = num34);
				prot = true;
			}
			else if (flag && alphaU > num + 0.5f)
			{
				// 1.9.3: continuous onset. The protection branch starts at alpha > limit + 0.5, where the capture branch commands
				// the flight-path rate; it used to subtract sqrt(2 a e) + 4 there at once (~14 deg/s), a step that pitched the
				// jet 3-4.5 deg below the limit in max-AoA turns (the 'pitch hitch'). Now 0 at the threshold, linear
				// (ProtOnsetGain /s per deg) until it meets the braking curve, and the +4 ramps in over 2 deg.
				float eP = alphaU - num - 0.5f;
				float num35 = Mathf.Min(Plugin.ProtRate, Mathf.Min(ProtOnsetGain * eP, Mathf.Sqrt(2f * Mathf.Max(aUp, 40f) * (alphaU - num))) + 4f * Mathf.Clamp01(eP / 2f));
				num35 = Mathf.Min(Plugin.ProtRate, Mathf.Max(num35, capPushU)); // 2.0.14: never weaker than the capture just below
				float num36 = Mathf.Max(num8 - num35, 0f - Plugin.ProtRate);
				float num37 = StickScale(sp);
				num34 = ((num36 < 0f) ? (num36 * num37) : num36);
				if (!StickBypassNow || qRaw < num34) num34 = Mathf.Min(num34, qPilot);
				num18 = (num19 = num34);
				// 1.9.3: the protection state (pitch priority in the allocator, fight costs off, ...) only past ProtFlagBand,
				// so a jet holding the limit in a turn does not toggle it every few frames
				prot = eP > ProtFlagBand;
			}
			else if (flag && alphaU < num2 - 0.5f)
			{
				float eN = num2 - alphaU - 0.5f;
				float num38 = Mathf.Min(Plugin.ProtRate, Mathf.Min(ProtOnsetGain * eN, Mathf.Sqrt(2f * Mathf.Max(aDn, 40f) * (num2 - alphaU))) + 4f * Mathf.Clamp01(eN / 2f));
				num38 = Mathf.Min(Plugin.ProtRate, Mathf.Max(num38, capPushD)); // 2.0.14
				float num39 = Mathf.Min(num8 + num38, Plugin.ProtRate);
				float num40 = StickScale(0f - sp);
				num34 = ((num39 > 0f) ? (num39 * num40) : num39);
				if (!StickBypassNow || qRaw > num34) num34 = Mathf.Max(num34, qPilot);
				num18 = (num19 = num34);
				prot = eN > ProtFlagBand;
			}
			else
			{
				num34 = ((!(num19 > num18)) ? Mathf.Clamp(qRaw, num19, num18) : ((alpha >= 0f) ? num18 : num19));
				if (unload)
				{
					if (num10 > 0f)
					{
						num34 = Mathf.Max(num34, 0f);
					}
					else if (num10 < 0f)
					{
						num34 = Mathf.Min(num34, 0f);
					}
				}
				if (Mode == FcsMode.AoAG)
				{
					// 2.44.0 (StickCancelsLimit): at the AoA limit the limit's own nose-down (the capture bound) scales with 1 - stick / 0.9
					float flU = capAtLimU ? Mathf.Min(0f, num18) * StickScale(sp) : 0f;
					float flD = capAtLimD ? Mathf.Max(0f, num19) * StickScale(0f - sp) : 0f;
					if (sp > 0.02f && alpha > -2f)
					{
						num34 = Mathf.Max(num34, (nz > num5) ? ((0f - Plugin.ProtRate) * (1f - sp)) : flU);
					}
					else if (sp < -0.02f && alpha < 2f)
					{
						num34 = Mathf.Min(num34, (nz < num6) ? (Plugin.ProtRate * (1f + sp)) : flD);
					}
				}
			}
			if (flag && !prot)
			{
				if (num3 < num - 0.05f && alphaU > num3)
				{
					float num41 = alphaU - num3;
					float num42 = Mathf.Min(Plugin.AsymRecoveryRate, Mathf.Min(3f * num41, Mathf.Sqrt(2f * Mathf.Max(aUp, 40f) * num41)));
					num34 = Mathf.Min(num34, Mathf.Max(num8 - num42, 0f - Plugin.ProtRate));
				}
				else if (num4 > num2 + 0.05f && alphaU < num4)
				{
					float num43 = num4 - alphaU;
					float num44 = Mathf.Min(Plugin.AsymRecoveryRate, Mathf.Min(3f * num43, Mathf.Sqrt(2f * Mathf.Max(aDn, 40f) * num43)));
					num34 = Mathf.Max(num34, Mathf.Min(num8 + num44, Plugin.ProtRate));
				}
			}
			// 2.1.8 AoA-sweep yaw limit. A velocity-vector roll pS at AoA a is coordinated when the stability-axis yaw rate
			// rS = -p sin a + r cos a is zero; the kinematic sideslip rate is -rS. Sweeping the AoA under a roll changes rS by
			// -pS * alphaDot, and only the yaw (r, through cos a) and roll (p, through sin a) accelerations can turn the rate
			// vector back. 12 Oct 23:00 log, 232-243 s: a 30 deg/s roll at 65 deg AoA, 57 m/s, left r = +25..29 deg/s; the full
			// push right after swept the AoA 65 -> -43 at 48 deg/s, the yaw (12-15 deg/s^2 modelled) could not reverse r, the
			// sideslip ran to -70 deg and the reversed dihedral at negative AoA drove the roll up to 43 deg/s against the
			// command. The AoA rate is held to what the lateral axes can follow; a sweep that brings rS back to zero is free.
			// Measured state only; off with no roll (|pS| under SweepPs0), beyond the AoA limits and for AI.
			pullGovTouched = false;
			pullTradeTgt = 999f;
			tradeAlphaNew = -1f;
			pullTradeHoldDbg = false;
			float qAskPre = num34;
			// 0.4.2: whether the pitch command is asking for a sweep at all. The FF roll trade's ask is the command's distance
			// from the measured flight-path rate (qAskPre - wVe); with the stick in the detent the command is *exactly* zero
			// (qRaw = sp x qB behind the num9 detent gate), so that distance becomes the measured rate itself and the trade
			// pays for a sweep nobody asked for. Rolling at 250 km/h through zero AoA with the pitch stick centred, the jet's
			// own settling sweeps the AoA 12 -> -2 deg at 2-7 deg/s, and the trade read that as a 2-7 deg/s ask: the roll-rate
			// limit swung 30 <-> 160 deg/s as it wandered - the roll "randomly speeding up and down" at low IAS. The Raptor
			// hides it: its rudders keep the capacity high enough that the same cut only moves the limit 106-127.
			// The pilot's stick (the law's own detent gate) and any bound that actually bound the command - protection, the
			// g / AoA-rate clamps, the unload - all count; a centred stick with nothing binding does not. A steady rolling
			// pull still trades (the stick is out of the detent), and the ask there is 0 anyway once the g has settled.
			// |sp| > 0.04 is the law's own detent (num9 = Clamp01((|sp| - 0.01) / 0.06) crosses 0.5 there), written on sp
			// directly so it reads the same under the rate law, where num9 stays 1
			bool qSweepAsk = Mathf.Abs(sp) > 0.04f || prot || protHold || Mathf.Abs(num34 - qRaw) > 0.05f;
			// 2.4.16: also with PitchRateCommandOnly (FcsMode.Rate). The sweep limit, the rolling-pull sideslip governor and
			// PullRollTrade only ever act on the pitch-rate command (num34) and the measured state, so they apply the same way
			// under the rate law; gated to AoAG they were simply off there (28-29 Oct logs, mode 6: sweepLim 0, slipGovPull 1,
			// pullTrade 0 in every rolling pull; full aft + full roll ran to 17-23 deg of sideslip at 150-340 kt)
			// 2.2.11 PullRollTrade, feed-forward part: with the trade on, the roll limit is also set from the AoA rate the
			// stick asks for, before the governor has to react: the roll whose coordinating yaw the yaw axis can build while
			// the AoA rises at that rate (the governor's own feed-forward, solved for pS). At high IAS the roll got to
			// 140-157 deg/s inside 0.4 s while the AoA rose at 50-60 deg/s; the governor caught it on the sideslip and
			// unloaded, and pitch and roll hitched together (19 Oct 10:29 log, 254 s at 295 kt: sideslip 14.7 deg, q 61 -> 5
			// -> 40, pS 138 -> 56 -> 129; 467 s at 345 kt the same).
			// 2.5.2 roll first, then pitch: the lateral governor's AoA-rise cap (yaw saturated with the roll already held; see
			// LateralLaw) holds the nose-up pitch-rate command to the AoA-hold rate plus that rise, with the 2.2.9 release lag
			// 2.5.3: both ways. The yaw the pair needs is |pS sin AoA|; any AoA sweep under a roll changes it at pS cos AoA x AoA
			// rate, nose-up or nose-down, at positive or negative AoA. A sweep that grows the demand is paid from the yaw left
			// on the roll's side (yawSatAlphaRate); one that shrinks it from the rudders' other side (yawSatAlphaRateDn). This is
			// the 2.1.8 sweep yaw limit (off from RollGovernorReplaces 2) from the same rule as the pull cap, so a full push +
			// full roll from positive AoA no longer sweeps the yaw demand through zero faster than the rudders can swing (bench,
			// 300 km/h: the roll outran the yaw at the zero crossing and the sideslip diverged).
			yawSatPitchDbg = 0f;
			float num34Ask = num34;
			bool planOwns = PushPlan && pushPlanOn && PushPlanOwnsPitch;
			if (yawSatAlphaRate > 0f && flag && !prot && !isAI && LatGovMode && !planOwns)
			{
				// nose-up pitch grows |AoA| at positive AoA and shrinks it at negative AoA
				float upRate = (alpha >= 0f) ? yawSatAlphaRate : yawSatAlphaRateDn;
				float dnRate = (alpha >= 0f) ? yawSatAlphaRateDn : yawSatAlphaRate;
				// 2.24.0 (SweepNoReverse), 2.31.1 written into the rates: a sweep toward zero AoA may be paced down to the flight-path
				// rotation, never below it (the nose is held, never turned back: the pitch-rate command the pace leaves is never on the
				// far side of zero). Held pushes too (SweepNoReverseHeld, 2.25.0); not under the no-fin nose-hold (2.19.0: holding the
				// AoA on a turning flight path IS a positive body rate; with the floor the no-fin held pushes departed on the bench).
				if (SweepNoReverse)
				{
					float wNr = (SweepNoReverseHeld ? 1f : stickRelW) * (1f - sweepHoldDbg);
					upRate = Mathf.Lerp(upRate, Mathf.Max(upRate, 0f - num8), wNr);
					dnRate = Mathf.Lerp(dnRate, Mathf.Max(dnRate, num8), wNr);
				}
				float capY = num8 + upRate;
				float capLo = num8 - dnRate;
				if (num34 > capY)
				{
					yawSatPitchDbg = num34 - capY;
					num34 = capY;
				}
				else if (num34 < capLo)
				{
					yawSatPitchDbg = num34 - capLo;
					num34 = capLo;
				}
				yawSatCapQ = capY;
				yawSatCapQLo = capLo;
				yawSatCapOn = true;
			}
			else if (planOwns)
			{
				// 2.36.0: the plan owns the nose-down pitch; the caps re-arm from open when it ends (no stale bound at the bottom)
				yawSatCapQ = 120f;
				yawSatCapQLo = -120f;
				yawSatCapOn = false;
			}
			else if (yawSatCapOn && !(PushPlan && pushPlanOn))
			{
				// let go over GovCapTau, not in one frame
				yawSatCapQ = Mathf.Lerp(yawSatCapQ, 120f, Mathf.Clamp01(dt / Mathf.Max(0.02f, GovCapTau)));
				yawSatCapQLo = Mathf.Lerp(yawSatCapQLo, -120f, Mathf.Clamp01(dt / Mathf.Max(0.02f, GovCapTau)));
				num34 = Mathf.Clamp(num34, yawSatCapQLo, yawSatCapQ);
				if (yawSatCapQ > 100f)
				{
					yawSatCapOn = false;
				}
			}
			// 2.35.0 (PushPlan): the planned push - the nose-down body rate is bounded at the plan's, one curve
			if (PushPlan && pushPlanOn && !prot && !isAI)
			{
				if (pushPlanSide > 0f) num34 = Mathf.Max(num34, 0f - pushPlanQNow);
				else num34 = Mathf.Min(num34, pushPlanQNow);
			}
			// 2.36.0 (PushPlanClipKey): "the sweep toward zero is held back" - the allocator's pitch priority (pitchNeedDn) and the stabs'
			// attached window (sweepHoldWin) read it next tick - is whatever bounds the push: the sweep caps, or the plan when it is on.
			// Until 2.35.3 only the caps' clip was read, so under a plan those two ran only because the 60 deg/s ceiling also clipped
			// the stick (a full push always does) - the ceiling could not stand aside without taking them with it (2.35.1: sideslip
			// median 4.8 -> 11.2 with PushPlanOwnsPitch).
			sweepClipDbg = yawSatPitchDbg;
			sweepClipOn = yawSatCapOn;
			if (PushPlanClipKey && PushPlan && pushPlanOn && !prot && !isAI)
			{
				sweepClipDbg = num34Ask - num34;
				sweepClipOn = true;
			}
			// 2.5.1 (RollGovernorReplaces >= 1): the trade is the rolling-pull protection on its own - the AoA-rise hold above is off, so
			// the trade no longer waits for it: no IAS fade (the hold's own reaction covered low IAS before), and its slow release
			// while the stick still asks (PullTradeRiseHold) is kept here
			// 2.6.0: level 3 only - the trade is the rolling-pull protection on its own, at every IAS and either AoA sign
			// 2.5.5: both ways and at either AoA sign. A nose-down sweep (push-over under a roll) or a pull from negative AoA moves
			// the yaw the roll needs just as a pull does, and the roll is what is traded first; the AoA-rate caps then only have the
			// roll that is left to contend with. 10:10 log, 268 / 315 s: full push + full roll from +9 deg at 104 / 68 m/s - no trade
			// (pullTrade 139-154 against pSmax 119), the sweep cap took 15-50 deg/s off the pitch-rate command instead, and the pace
			// window cut the roll a second later anyway.
			if (Plugin.PullRollTrade > 0f && PullTradeFFK > 0f && SweepPosPullFree && flag && !prot && !isAI && tas > 25f && LatGovMode)
			{
				float aAskF = qSweepAsk ? Mathf.Abs(qAskPre - num8) : 0f;
				pullTradeHoldDbg = aAskF > 5f;
				pullTradeAsk = aAskF;
				if (aAskF > 5f)
				{
					float caF = Mathf.Abs(Mathf.Cos(alpha * (MathF.PI / 180f)));
					// faded in with IAS (PullTradeFFIas0 -> +PullTradeFFIasSpan m/s, 180 -> 230 kt; 2.2.12, was 230 -> 280: 13 deg of sideslip at 240 kt in the 10:53 log): below that the governor's own reaction keeps up
					// with the roll build (your 0.7 / 9 setting held at 250 kt and below)
					float tgtF = 999f;
					// 2.8.0: the yaw the pair needs grows at pS cos AoA x AoA rate. Against the roll the stick asks (pSAskStick, before
					// any cap) and the measured capacity on the side the sweep needs (CapacityEstimator, last tick), TradeCapShare of it:
					// no shortfall, no trade - the pitch runs free whatever the roll limit is doing; a shortfall is split PullRollTrade to
					// the roll (its target drops until its share is paid) and the rest to the AoA rate. Nothing here reads the roll limit,
					// so a released stick (pSAskStick 0) asks nothing of the pitch.
					float kDem = caF * aAskF * (MathF.PI / 180f);
					float growM = ((qAskPre - num8) * Mathf.Sign(alpha) >= 0f) ? 1f : -1f;
					float rollRef = (Mathf.Abs(pTraj.x) > 2f) ? pTraj.x : paceTgtRaw;
					float sideM = growM * Mathf.Sign(rollRef * Mathf.Sin(alpha * (MathF.PI / 180f)));
					// 2.21.0: the low-passed read (TradeCapTau) - the trade
					// is the one accounting of the rolling pull's pitch share and must not dip with the rudders' servo transit (see CapacityEstimator)
					float capM = (sideM >= 0f) ? capEst.capTrZeP : capEst.capTrZeN;
					float capS = Mathf.Max(0f, TradeCapShare * capM);
					float demandM = pSAskStick * kDem;
					if (demandM > capS && kDem > 1E-04f)
					{
						float shortM = demandM - capS;
						float trM = (yawHealth >= 0.99f) ? Plugin.PullRollTrade : Mathf.Min(Plugin.PullRollTrade, PullRollTradeFinLoss);
						float pSNew = Mathf.Max(PullTradeFloor, pSAskStick - trM * shortM / kDem);
						float leftM = pSNew * kDem - capS;
						float aNew = (leftM > 0f && pSNew > 1f) ? (aAskF - leftM / (pSNew * caF * (MathF.PI / 180f))) : aAskF;
						tgtF = pSNew;
						tradeAlphaNew = Mathf.Max(StopSlipPitchFloor, aNew);
					}
					else
					{
						tgtF = 999f;
					}
					if (tgtF < pullTradeTgt)
					{
						float pSF = Mathf.Abs((omega.x * caF + omega.z * Mathf.Sin(alpha * (MathF.PI / 180f))) * 57.29578f);
						// it stops the roll building past the target, it does not cut a roll already established (cutting a
						// 70 deg/s roll at 35 deg AoA swung the sideslip 4 -> -10 deg on the bench); the governor's own trade does that
						// 2.2.25: below PullTradeCutA (fading out over the next 10 deg) it does cut an established roll: a pull
						// into a fast roll from low AoA otherwise ran the roll on until the governor caught the sideslip, then cut
						// the AoA rise and the roll together (20 Oct 01:41 log, 230-260 kt, rolling 175-210 deg/s: pitch rate
						// 35 -> 22-31 -> 48 deg/s, sideslip 6-8 deg). At low AoA a velocity-vector roll is nearly a body roll, so
						// slowing it does not swing the sideslip the way it did at 35 deg AoA.
						float cutW = (PullTradeCutA > 0f) ? Mathf.Clamp01((PullTradeCutA + 10f - Mathf.Abs(alpha)) / 10f) : 0f;
						tgtF = Mathf.Max(tgtF, Mathf.Lerp(Mathf.Min(pSF, pullTradePs), tgtF, cutW));
						if (pullTradePs > Mathf.Max(pSF, tgtF))
						{
							pullTradePs = Mathf.Max(pSF, tgtF);
						}
						pullTradeTgt = tgtF;
					}
				}
			}
			// 2.2.8: the gate recovers whenever it is not being updated (no roll, negative AoA, at a limit). It used to hold
			// its last value until the next rolling pull, which then started governed (19 Oct log, 30-36 s: 0.10 held through
			// 5 s of wings-level flight; the next rolling pull at 36.3 s was held to 4 deg/s of AoA rise at 1 deg of sideslip).
			if (!pullGovTouched)
			{
				pullGovF = Mathf.MoveTowards(pullGovF, 1f, PullGovIdleRise * dt);
				pullFFBoost = Mathf.MoveTowards(pullFFBoost, 1f, PullFFBoostIdle * dt);
				pullServoEx = 0f;
				slipGovPullDbg = 1f;
				// 2.2.20: when the governor stops acting (roll under SlipGovPs0, AoA through zero) its cap is let go over
				// GovCapTau instead of at once, so an unload does not snap back to the stick's 48 deg/s in one frame
			}
			pullTradePs = Mathf.MoveTowards(pullTradePs, pullTradeTgt, ((pullTradeTgt < pullTradePs) ? PullTradeFall : ((pullTradeHoldDbg && PullTradeRiseHold > 0f) ? PullTradeRiseHold : PullTradeRise)) * dt);
			if (pullTradeTgt >= 999f && pullTradePs > 400f)
			{
				pullTradePs = 999f;
			}
			qMaxDbg = num18;
			qMinDbg = num19;
			// 2.0.5: the final clamp used AoARecoveryRate (48) only, so MaxPitchRate above it did nothing
			float qAbsMax = Mathf.Max(Plugin.ProtRate, PitchRateCap);
			qCmd = Mathf.Clamp(num34, 0f - qAbsMax, qAbsMax);
			// Su-30 0.2.0 (SoftAoALimit): handed to the MPO law past the limit, the stick may not pull the jet further past it: away from
			// the limit's side the body rate is held at the flight path's turn or zero, whichever lets the nose go less far (no protection
			// push-back: a released stick holds zero pitch rate, forward stick pitches back in). Replayed, Tom's 21:11 log 279-285 s (full
			// aft + full roll held at 82 deg) went to 125 deg AoA and 53 deg of sideslip with the plain MPO rate command.
			if (softPitch && Mode == FcsMode.AoAG)
			{
				float fwd = Mathf.Min(0f, sp) * Plugin.ProtRate;
				if (alpha > 0f)
				{
					qCmd = Mathf.Clamp(qCmd, Mathf.Min(fwd, Mathf.Max(0f, num8)), Mathf.Max(0f, num8));
				}
				else
				{
					float aftN = Mathf.Max(0f, sp) * Plugin.ProtRate;
					qCmd = Mathf.Clamp(qCmd, Mathf.Min(0f, num8), Mathf.Max(aftN, Mathf.Min(0f, num8)));
				}
			}
			softPitchDbg = softPitch ? 1f : 0f;

			if (softMpo && Mode == FcsMode.MPO)
			{
				if (alpha > 0f)
				{
					qCmd = Mathf.Min(qCmd, Mathf.Max(0f, num8));
				}
				else
				{
					qCmd = Mathf.Max(qCmd, Mathf.Min(0f, num8));
				}
			}
			gyroQLimDbg = 0f;
			gyroRollW = 0f;
			if (Plugin.GyroBudgetFrac > 0f && canted != null && !prot && Mathf.Abs(alpha) > GyroBudgetAoA0 && qCmd * Mathf.Sign(alpha) > 0f)
			{
				float kG = Mathf.Abs((Ia[1, 1] - Ia[2, 2]) / Mathf.Max(1f, Ia[0, 0])) * (MathF.PI / 180f);
				float rAbs = Mathf.Abs(omega.z) * 57.29578f;
				float qLim = Mathf.Max(GyroBudgetQFloor, Plugin.GyroBudgetFrac * (aRollTvc + aRoll) / Mathf.Max(0.001f, kG * rAbs));
				float wG = Mathf.Clamp01((Mathf.Abs(alpha) - GyroBudgetAoA0) / Mathf.Max(1f, GyroBudgetAoA1 - GyroBudgetAoA0));
				wG = wG * wG * (3f - 2f * wG);
				float wI = Mathf.Clamp01((GyroBudgetIasHi - ias * 3.6f) / Mathf.Max(1f, GyroBudgetIasHi - GyroBudgetIasLo));
				wG *= wI * wI * (3f - 2f * wI);
				// Su-30 0.3.5 (GyroRollTrade): the budget also goes to the roll (lateral law): the velocity roll is held to what leaves this
				// pitch rate inside the budget, so the yaw comes down and the cap below lets the pitch through
				gyroRollW = wG;
				gyroQNeed = Mathf.Abs(qCmd);
				gyroBudgetA = Plugin.GyroBudgetFrac * (aRollTvc + aRoll) / Mathf.Max(0.001f, kG);
				float lim = Mathf.Lerp(Mathf.Abs(qCmd), Mathf.Min(Mathf.Abs(qCmd), qLim), wG);
				qCmd = Mathf.Sign(qCmd) * lim;
				gyroQLimDbg = qLim;
			}
			if (Mode == FcsMode.AoAG && !prot && Plugin.PitchAntiWindup > 0f && num9 > 0.5f)
			{
				float aw = Plugin.PitchAntiWindup * (qCmd - qRaw) * dt;
				if (aw * aInt < 0f)
				{
					aInt = ((aInt > 0f) ? Mathf.Max(0f, aInt + (1f - betaG) * aw) : Mathf.Min(0f, aInt + (1f - betaG) * aw));
				}
				if (aw * gInt < 0f)
				{
					gInt = ((gInt > 0f) ? Mathf.Max(0f, gInt + betaG * aw) : Mathf.Min(0f, gInt + betaG * aw));
				}
			}
			// Revoker 0.2.8: braked touchdown on the mains, nose still up: lower it at DerotationRate (see derotGate)
			if (derotGate && Mode == FcsMode.GearDownRate)
			{
				qCmd = Mathf.Min(qCmd, 0f - DerotationRate);
			}
			float num45 = qCmd - qTraj.x;
			bool pitchOnset = num45 * qTraj.x >= 0f;
			float a3 = Mathf.Max(PitchTrajAuthFrac * ((num45 > 0f) ? (pitchOnset ? aUpP : aUp) : (pitchOnset ? aDnP : aDn)), 30f);
			a3 = Mathf.Min(a3, Mathf.Max(30f, Plugin.PitchOnsetGps / Mathf.Max(0.05f, nAlpha)));
			// 1.9.0: stick released in the G regime: the pitch rate is taken off over ~ReleaseTauG instead of in one step, so
			// the load comes off at a steadier rate at high dynamic pressure (the zero-rate hold itself is unchanged)
			if (Mode == FcsMode.AoAG && Plugin.CmdShaping && !prot && betaG > 0.5f && Mathf.Abs(sp) < 0.07f && num45 * qTraj.x < 0f)
			{
				a3 = Mathf.Min(a3, Mathf.Max(3f, Mathf.Abs(qTraj.x) / ReleaseTauG));
			}
			// Ternion 0.1.3: the same release shaping in the AoA regime (a corkscrew or a high-AoA pull let go). The canards
			// stop a 25-30 deg/s pitch rate by going to their 60 deg LE-down stop and must slew back 40 deg at 100 deg/s, so a
			// step stop overshot nose-down (bench: -3 deg/s at 450 km/h, Tom saw -4 to -5)
			else if (ReleaseShapeAoA && Plugin.ReleaseTauAoA > 0.01f && Mode == FcsMode.AoAG && Plugin.CmdShaping && !prot && Mathf.Abs(sp) < 0.07f && num45 * qTraj.x < 0f)
			{
				a3 = Mathf.Min(a3, Mathf.Max(3f, Mathf.Abs(qTraj.x) / Plugin.ReleaseTauAoA));
			}
			float j = (prot ? (a3 / 0.1f) : (a3 / 0.15f));
			if (unload)
			{
				a3 = ((num10 * alphaDot > 0.5f) ? Mathf.Min(a3, Mathf.Max(60f, 2f * Plugin.UnloadDecel)) : Mathf.Min(a3, Plugin.UnloadDecel));
				j = a3 / 0.3f;
			}
			// Su-30 0.2.0 (PitchTrajMeasuredEnd, Tom 9 Nov 21:28: push-overs not as smooth as the Raptor's and the Ifrit's): a move away
			// from zero rate (a push or a pull toward the stick's rate) eases into its target over the effectors' measured swing time
			// (PitchReach: moment-weighted half-swing at each one's slew, plus the INDI lag) instead of 0.15 s. The jet answers the
			// trajectory's acceleration that late, so a 0.15 s ease-in left it still accelerating at the target: full pushes from 63-70
			// deg asked for 48 deg/s and flew 57-63 (21:04 log), the canards having to swing back 40 deg and the stabs 35 to stop it.
			// Moves back toward zero (captures, releases) keep their own shaping.
			pitchTrajTauDbg = 0f;
			float jEnd = j;
			if (Plugin.PitchTrajMeasuredEnd && !prot && !unload && pitchOnset && Mathf.Abs(num45) > 0.5f && (Mode == FcsMode.AoAG || Mode == FcsMode.MPO || Mode == FcsMode.Rate))
			{
				PitchReach(out float tjSwDn, out float tjTDn, out float tjSwUp, out float tjTUp);
				float tE = ((num45 > 0f) ? tjTDn : tjTUp) * PitchTrajEndK;
				pitchTrajTauDbg = tE;
				jEnd = Mathf.Min(j, a3 / Mathf.Max(0.15f, tE));
			}
			qTraj.Step(qCmd, a3, j, jEnd, dt);
			// 2.0.6: nose wheel still on the runway in the rate law: the jet cannot pitch yet, so the rate trajectory is
			// kept just ahead of the measured rate (its acceleration still asks for the rotation). When the nose comes up
			// the command starts from the jet's own rate, as it would in the air, instead of a step to the full command.
			if (noseContactLaw && qTraj.x > 0f)
			{
				float qMeas = omega.y * 57.29578f;
				qTraj.x = Mathf.Min(qTraj.x, Mathf.Max(qMeas, 0f) + Mathf.Min(Mathf.Max(qTraj.v, 0f) * RotLeadT, RotLeadMax));
			}
			qT = qTraj.x;
		}

		internal static float LowIasYawBoostAt(float iasNow)
		{
			if (YawBoostFixed >= 1f)
			{
				return YawBoostFixed;
			}
			float num = Mathf.Max(1f, Plugin.LowIasYawBoost) - 1f;
			float lowIasYawBoostFullIas = Plugin.LowIasYawBoostFullIas;
			float num2 = ((iasNow <= lowIasYawBoostFullIas) ? 1f : ((float)Math.Exp((0f - (iasNow - lowIasYawBoostFullIas)) / Mathf.Max(1f, Plugin.LowIasYawBoostFalloff))));
			num2 *= Mathf.Clamp01((120f - iasNow) / 10f);
			return 1f + num * num2;
		}

		internal static float FightPedalRelease = 1f;

		internal static int FightMode = 0;

		internal static bool RelQuietHold = true;

		internal static float RelQuietErr0 = 2f;

		internal static float RelQuietErrSpan = 4f;


		internal static float PitchFightQdot = 25f;

		internal static float PitchFightQHold = 0.6f;

		private float pfQHold;

		public static float PitchFightErr0 = 3f;

		public static bool PitchFightAbsErr = false;

		public static float PitchFightErrSpan = 5f;

		internal static float FightRollRelease = 0.3f;

		private float fightRollHold;

		private float fightPed;

		// 1.8.41 pitch-reserve roll governor. Rolling at a held or restricted AoA makes pitch-up: the canted/stalled
		// down-going stab and flaperon lose lift for roll, and the inertial coupling (Izz-Ixx)/Iyy * p * r grows with the
		// roll rate squared. Above the roll-pitch-priority band (18-26 deg AoA) only the nozzles are left to hold it, and with
		// a store they sat on the -20 deg stop for whole rolls (2026-09-28 log: 28 deg cap, full aft + roll, nozzles pinned
		// 4 s). The governor watches the nozzles' remaining nose-down travel (nose-up at negative AoA) at the live thrust and,
		// while a roll is commanded, lowers the roll-rate limit and entry acceleration when less than RollPitchReserve of
		// their nose-down authority is left; it gives it back as the margin returns. Measured state only - no AoA/speed
		// schedule. 0 = off (1.8.40).
		internal static float RollGovDown = 1.5f;

		internal static float RollGovUp = 0.6f;

		internal static float RollGovFloor = 0.35f;

		internal static float RollGovMinRate = 30f;

		internal static float RollGovAccelFloor = 0.5f;

		internal static float RollGovAoA0 = 3f;

		// Compass 0.1.5: the roll governor on the stabs' nose-down margin when there is no pitch TVC
		internal static bool StabRollGovernor = true;

		internal static float StabRollGovAoA0 = 3f;

		// Compass 0.1.5: AoA capture braking under the asymmetric-load cap without pitch TVC
		internal static bool AsymCapBrake = true;

		internal static float AsymCapBrakeK = 0.5f;

		public float rollGov = 1f;

		public float rollGovMargin;

		private Lpf rollGovMF;

		private Lpf govMarginRateF;

		private float rollGovMarginPrev = -1f;

		public float rollGovPredDbg = 1f;

		internal static bool GovPredictive = true;

		internal static float GovLookT = 0.3f;

		internal static float GovNeedHi = 0.2f;

		internal static float GovDownP = 0.8f;

		internal static float GovUpP = 1.0f;

		private void RollGovStep(float dt, float pLim, float rollSide)
		{
			float res = Plugin.RollPitchReserve;
			// Compass 0.1.5 (StabRollGovernor): without pitch TVC the governor reads the stabs' nose-down margin instead of the
			// nozzles'. 5 Nov 18:52 log: right outer wing gone (asym-load AoA cap 12 deg), full aft + full left roll at 90 deg/s and
			// 12-14 deg AoA: the roll's inertial pitch-up took the last of the stabs' nose-down (aDn 5 deg/s^2, both slabs at their
			// peak), the AoA ran 12 -> 24 deg, the lost wing's lift asymmetry grew 64 -> 470 deg/s^2 of roll and the jet departed
			// (sideslip 47 deg). The coupling limit there reads modelled authority and an ideal velocity-vector roll; this reads
			// where the stabs actually are.
			bool stabGov = StabRollGovernor && (tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC);
			if (res <= 0f || (!stabGov && (tvc == null || !tvc.active || tvc.N < 2 || tvc.M == null)) || !Engaged || Mode == FcsMode.Ground || Mode == FcsMode.Off || revFlow)
			{
				rollGov = Mathf.MoveTowards(rollGov, 1f, 2f * dt);
				rollGovMargin = 1f;
				slatRelief = Mathf.MoveTowards(slatRelief, 0f, SlatReliefOut * dt);
				return;
			}
			float yLo = float.MaxValue;
			float yHi = float.MinValue;
			float yCmd = 0f;
			float yMid = 0f;
			if (stabGov)
			{
				yLo = 0f;
				yHi = 0f;
				int nS = 0;
				foreach (SurfaceEffector sG in surfaces)
				{
					if (sG.kind != SurfaceKind.Stabilator || !sG.active || sG.M == null || sG.N < 2)
					{
						continue;
					}
					float lo = float.MaxValue;
					float hi = float.MinValue;
					for (int k = 0; k < sG.N; k++)
					{
						lo = Mathf.Min(lo, sG.M[k].y);
						hi = Mathf.Max(hi, sG.M[k].y);
					}
					yLo += lo;
					yHi += hi;
					yCmd += sG.MAt(sG.cmd).y;
					yMid += sG.MAt(0f).y;
					nS++;
				}
				if (nS == 0)
				{
					rollGov = Mathf.MoveTowards(rollGov, 1f, 2f * dt);
					rollGovMargin = 1f;
					return;
				}
			}
			else
			{
				for (int k = 0; k < tvc.N; k++)
				{
					yLo = Mathf.Min(yLo, tvc.M[k].y);
					yHi = Mathf.Max(yHi, tvc.M[k].y);
				}
				yCmd = tvc.MAt(tvc.cmd).y;
				yMid = tvc.MAt(0f).y;
			}
			// margin toward the side the roll's pitch moment pushes against: nose-down at positive AoA
			float half = (alpha >= 0f) ? (yMid - yLo) : (yHi - yMid);
			float marg = (alpha >= 0f) ? (yCmd - yLo) : (yHi - yCmd);
			float frac = (half > 1E-06f) ? Mathf.Clamp01(marg / half) : 1f;
			// 2.1.3: at the low-thrust AoA cap the nozzles carry little of the nose-down authority, so "nozzles pinned" no
			// longer means "no pitch reserve" (the stabs and the airframe hold it). The margin counts only by the nozzles'
			// share of the nose-down authority there; elsewhere unchanged. Without it the governor sat at its floor in every
			// idle-cap roll and jumped back whenever a pitch transient unpinned the nozzles (12 Oct log, 657 / 713 s).
			if (!stabGov && LowCapLatFix && Plugin.LowCapRollFix && LowThrustCapOn && alpha > 0f)
			{
				float tvShare = Mathf.Clamp01(half * IaInv[1, 1] * 57.29578f / Mathf.Max(20f, aDn));
				float nearCap = Mathf.Clamp01((alpha - (authCapEff - LowCapCoupBand)) / Mathf.Max(0.5f, LowCapCoupBand));
				frac = Mathf.Lerp(frac, Mathf.Lerp(1f, frac, tvShare), nearCap);
			}
			frac = rollGovMF.Step(frac, 0.06f, dt);
			// 2.1.5 predictive governor: the margin's rate of change projected GovLookT ahead. The old governor cut only once
			// the nozzles were already pinned (margin < 15 %) and then recovered at 0.6/s: a cut-and-restore of the roll target
			// (12 Oct 18:50 log: 120 -> 42 -> 85 deg/s at 96 s, 120 -> 74 -> 86 at 202 s, clean jet 347 s). Now it eases off
			// early and in proportion while the margin is heading for the stop, and comes back only when the margin is there and
			// no longer falling.
			float fracRate = govMarginRateF.Step(rollGovMarginPrev >= 0f ? (frac - rollGovMarginPrev) / Mathf.Max(dt, 1E-4f) : 0f, 0.1f, dt);
			rollGovMarginPrev = frac;
			float fracPred = Mathf.Clamp01(frac + Mathf.Min(0f, fracRate) * (Plugin.LoadedRollSteadyOn ? 0f : GovLookT));
			rollGovPredDbg = fracPred;
			float aoaGate = Mathf.Clamp01((Mathf.Abs(alpha) - (stabGov ? StabRollGovAoA0 : RollGovAoA0)) / 3f);
			float latNow = Mathf.Max(Mathf.Clamp01((Mathf.Abs(sr) - 0.05f) / 0.2f), Mathf.Clamp01((Mathf.Abs(pTraj.x) - 10f) / 20f));
			// 2.1.9 slat roll relief. The slats (Aryx's high-lift devices) are, in this game, only wing area on WingRear_L/R
			// whose centre of lift sits 1.34 m ahead of the CG, and the flap schedule has them fully out from 20 deg AoA. Out,
			// they push the nose up: retracting them gives 8-11 deg/s^2 nose-down at 300 km/h, 14-20 at 400, 27-39 at 550
			// (20-40 deg AoA), for ~7 % of the lift. While a roll is running at positive AoA and the nozzles' nose-down margin
			// heads toward the governor's threshold, the slats come in before the governor has to cut the roll; they go back
			// out once the margin is back or the roll is over. Measured state only (same margin the governor reads).
			SlatReliefStep(dt, fracPred, frac, aoaGate * latNow);
			if (latNow * aoaGate < 0.01f)
			{
				rollGov = Mathf.MoveTowards(rollGov, 1f, 2f * dt);
				return;
			}
			float need = Mathf.Clamp01(res);
			if (GovPredictive && Plugin.RollGovPredictiveOn)
			{
				float needHi = Mathf.Max(need + 0.05f, GovNeedHi);
				if (fracPred < needHi)
				{
					float deficitP = (needHi - fracPred) / needHi;
					rollGov = Mathf.Max(RollGovFloor, rollGov - GovDownP * deficitP * aoaGate * latNow * dt);
				}
				else if (frac >= need && fracRate > -0.05f)
				{
					rollGov = Mathf.Min(1f, rollGov + GovUpP * Mathf.Clamp01((frac - need) / Mathf.Max(0.05f, need)) * dt);
				}
				return;
			}
			if (frac < need)
			{
				float deficit = (need - frac) / Mathf.Max(0.05f, need);
				rollGov = Mathf.Max(RollGovFloor, rollGov - RollGovDown * deficit * aoaGate * latNow * dt);
			}
			else
			{
				rollGov = Mathf.Min(1f, rollGov + RollGovUp * Mathf.Clamp01((frac - need) / Mathf.Max(0.05f, need)) * dt);
			}
		}

		public float slatRelief;

		internal static float SlatReliefStart = 0.5f;

		internal static float SlatReliefFull = 0.2f;

		internal static float SlatReliefIn = 4f;

		internal static float SlatReliefOut = 1f;

		internal static float SlatReliefAoA0 = 12f;

		internal static float SlatReliefAoASpan = 6f;

		internal static float SlatReliefAoAHi = 35f;

		internal static float SlatReliefAoAHiSpan = 10f;

		private void SlatReliefStep(float dt, float fracPred, float frac, float rollGate)
		{
			if (!Plugin.SlatRollReliefOn || alpha <= 0f || isAI)
			{
				slatRelief = Mathf.MoveTowards(slatRelief, 0f, SlatReliefOut * dt);
				return;
			}
			float aG = Mathf.Clamp01((alpha - SlatReliefAoA0) / Mathf.Max(1f, SlatReliefAoASpan));
			aG *= 1f - Mathf.Clamp01((alpha - SlatReliefAoAHi) / Mathf.Max(1f, SlatReliefAoAHiSpan));
			float g = aG * Mathf.Clamp01(rollGate);
			// demand from the predicted margin: 0 at SlatReliefStart, 1 at SlatReliefFull (the governor's own threshold)
			float want = g * Mathf.Clamp01((SlatReliefStart - fracPred) / Mathf.Max(0.05f, SlatReliefStart - SlatReliefFull));
			if (want > slatRelief)
			{
				slatRelief = Mathf.MoveTowards(slatRelief, want, SlatReliefIn * dt);
			}
			else if (g < 0.01f || frac > SlatReliefStart)
			{
				// roll over, or the margin is back above the start: out again (no chasing the margin it creates while rolling)
				slatRelief = Mathf.MoveTowards(slatRelief, want, SlatReliefOut * dt);
			}
		}

		internal static bool DamageLatLimit = true;
		internal static float FinLossPedalSideslip = 5f;
		internal static float FinLossPedalIas = 600f;

		internal static bool YawTargetClamp = true;

		internal static bool GroundRollP = true;

		public bool wheelsTouch;

		internal static bool AsymThrustSplit = true;

		internal static bool EngineOutFixedCap = true;

		// 2.0.12: with an engine out the asymmetric-load cap comes down this many times faster (AsymCapTightenRate)
		internal static float EngOutTightenK = 4f;

		internal static float YawTargetMargin = 1f;

		// 2.20.0: the yaw-target clamp's reach includes the stabilators' yaw range while they are off the pitch
		// (stabPitchFrac below YawClampStabFree, scaled to 0 at it)
		internal static bool YawClampStabReach = true;

		internal static float YawClampStabFree = 0.25f;

		public float yawClampStabDbg;

		public bool engOut;

		// Ifrit 0.4.1: one engine running weak (damaged, not destroyed: under EngineAsymEnter of the strongest one's thrust,
		// released above EngineAsymExit; the ratio is filtered over 1 s so spool-up transients do not trip it)
		public bool engAsym;

		// 0.4.1: weaker / stronger engine's thrust at full throttle, predicted from their damage state (1 = even)
		public float engAbRatio = 1f;

		// one engine can still light its afterburner and another cannot (predicted from the damage state)
		public bool engAbMismatch;

		// 0.4.1 thrust match: the parasitic loss the FCS is holding on the stronger engine(s), N (0 = none), for the telemetry
		public float engMatchLoss;

		private bool matchApplied;

		private struct EngModel
		{
			public float rMin;

			public float rMax;

			public float ceil;

			public float remapX;

			public float remapY;

			public float t0;

			public float ab;

			public float abStart;

			public float abEnd;
		}

		private readonly EngModel[] engModel = new EngModel[4];

		private readonly float[] engUnmatched = new float[4];

		// predicted thrust of engine i at throttle input x (the game's Turbojet model, see the engine scan)
		private float EngThrustAt(int i, float x)
		{
			ref EngModel em = ref engModel[i];
			float tr = (em.remapY > em.remapX + 0.001f) ? Mathf.Clamp01((x - em.remapX) / (em.remapY - em.remapX)) : Mathf.Clamp01(x);
			float rpm = Mathf.Lerp(em.rMin, em.ceil, tr);
			float dry = (em.rMax > em.rMin) ? Mathf.Clamp01((rpm - 0.85f * em.rMin) / (em.rMax - em.rMin)) : tr;
			float th = em.t0 * dry;
			if (em.ab > 0f && em.rMax > 0f && rpm / em.rMax > 0.85f && x > em.abStart)
			{
				th += em.ab * Mathf.Clamp01((x - em.abStart) / Mathf.Max(0.01f, em.abEnd - em.abStart));
			}
			return th;
		}

		// weaker / stronger predicted thrust at throttle input x (1 when both are at idle or there is one engine)
		private float EngRatioAt(int n, float x)
		{
			float lo = float.MaxValue;
			float hi = 0f;
			for (int i = 0; i < n; i++)
			{
				float th = EngThrustAt(i, x);
				lo = Mathf.Min(lo, th);
				hi = Mathf.Max(hi, th);
			}
			return (hi > 1000f) ? Mathf.Clamp01(lo / hi) : 1f;
		}

		// the throttle input where the afterburner starts (1 without one)
		public float engAbStart = 1f;

		// the afterburner gate is holding the throttle at the afterburner's start
		public bool abGateOn;

		private bool engFixedPrev;

		// 0.4.1 / 2.4.1 thrust match on prediction: the yaw the thrust split WOULD make without the match (deg/s^2, from the
		// engines' RPMs and the game's thrust law) against the usable yaw authority at this AoA and speed; the match comes
		// in above MatchYawOn and lets go under MatchYawOff, so a split the fins cannot hold is never allowed to form
		public float splitYawPred;

		public float splitUsage;

		internal static float MatchYawOn = 0.8f;

		internal static float MatchYawOff = 0.5f;

		private bool matchYawOn;

		// 0.4.1b / 2.4.1b: the match is proportional. On the way up to the cap it comes in ahead of the AoA (the AoA
		// predicted MatchLead s ahead from its rate, over the MatchBand deg under the cap), so the healthy engine is already
		// down to the damaged one's thrust when the cap is reached and the cap lifts without a step; stick released with
		// the AoA under the cap, it eases back out (unless the fins could not hold the split: splitUsage / the VMC latch).
		public float matchF;

		public bool matchFull;

		internal static float MatchBand = 4f;

		internal static float MatchLead = 0.3f;

		// the match is complete this long before the AoA would reach the cap at its present rate
		internal static float MatchLeadTime = 0.8f;

		internal static float MatchRiseRate = 4f;

		internal static float MatchFallRate = 1.5f;

		// the VMC protection with a weak (running) engine: the match, latched until the sideslip is calm, instead of the throttle
		private bool vmcMatch;

		private float vmcMatchCalmT;

		// after leaving the fixed-cap state the thrust term stays out of the sweep for this long (the first sweep after the
		// match engages still sees the split settling: 23 Oct 00:00 log, cap 12 for one cycle)
		private float tKHoldT;

		// IAS at the last asymmetric-load sweep: the stored yaw authority scales with q from there
		private float asymSweepIas = 1f;

		public float engRatioF = 1f;

		internal static float EngineAsymEnter = 0.85f;

		internal static float EngineAsymExit = 0.92f;

		private int engTotal = -1;

		internal static float DamageLatFloor = 0.35f;

		internal static float DamageLatFloorNoTvc = 0.15f;

		public float yawHealth = 1f;

		public float zLoDbg, zHiDbg, zTgtDbg;

		private int rudTotal = -1;

		// 2.0.12: share of the rudders still on the jet (a rudder goes with its fin)
		private void YawHealthStep()
		{
			if (rudTotal < 0)
			{
				rudTotal = 0;
				foreach (SurfaceEffector s0 in surfaces)
				{
					if (s0.kind == SurfaceKind.Rudder)
					{
						rudTotal++;
					}
				}
			}
			if (engTotal < 0)
			{
				engTotal = tvc.engines.Count;
			}
			// an engine counts as out when destroyed / inoperable or when it makes under 15 % of the strongest one's
			// thrust while that one makes over 20 kN (flame-out, damage)
			int engOk = 0;
			float tMax = 0f;
			float tMin = float.MaxValue;
			// 0.4.1: each engine's thrust-vs-throttle from its damage state (Turbojet.FixedUpdate): the RPM it can reach is
			// minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5); at a throttle input x the target RPM is
			// Lerp(minRPM, that ceiling, (x - remap.x) / (remap.y - remap.x)); dry thrust = (RPM - 0.85 minRPM) / (maxRPM - minRPM)
			// of the maximum; the afterburner runs only above 85 % of maxRPM (condition > 0.55 on a 1000-3000 RPM engine) and
			// fades in from its start throttle to its end. From that: the weaker / stronger ratio at full throttle
			// (engAbRatio) and whether one can light its afterburner while another cannot (engAbMismatch).
			int nE = 0;
			engAbStart = 1f;
			foreach (Turbojet eg in tvc.engines)
			{
				float tE = 0f;
				if (eg != null && Acc.TF_operable(eg))
				{
					engOk++;
					if (nE < engModel.Length)
					{
						ref EngModel em = ref engModel[nE];
						em.rMin = Acc.TF_minRPM(eg);
						em.rMax = Acc.TF_maxRPM(eg);
						em.ceil = em.rMin + (em.rMax - em.rMin) * (0.5f * Mathf.Clamp01(Acc.TF_condition(eg)) + 0.5f);
						Vector2 rm = Acc.TF_remap(eg);
						em.remapX = rm.x;
						em.remapY = rm.y;
						em.t0 = Mathf.Max(0f, Acc.TF_staticThrust(eg));
						em.ab = 0f;
						em.abStart = 1f;
						em.abEnd = 1f;
						nE++;
					}
					JetNozzle[] nzE = Acc.TF_nozzles(eg);
					if (nzE != null)
					{
						foreach (JetNozzle jE in nzE)
						{
							if (jE != null)
							{
								tE += Mathf.Max(0f, Acc.JN_totalThrust(jE));
								if (nE > 0 && nE <= engModel.Length)
								{
									ref EngModel em2 = ref engModel[nE - 1];
									em2.ab += Mathf.Max(0f, Acc.JN_abMax(jE));
									em2.abStart = Mathf.Min(em2.abStart, Acc.JN_abStart(jE));
									em2.abEnd = Mathf.Min(em2.abEnd, Acc.JN_abEnd(jE));
								}
								engAbStart = Mathf.Min(engAbStart, Acc.JN_abStart(jE));
							}
						}
					}
				}
				tMax = Mathf.Max(tMax, tE);
				tMin = Mathf.Min(tMin, tE);
			}
			engAbRatio = 1f;
			engAbMismatch = false;
			if (engTotal > 1 && nE >= 2)
			{
				engAbRatio = EngRatioAt(nE, 1f);
				bool anyAb = false;
				bool allAb = true;
				for (int ie = 0; ie < nE; ie++)
				{
					ref EngModel emc = ref engModel[ie];
					bool ok = emc.ab > 0f && emc.rMax > 0f && emc.ceil / emc.rMax > 0.85f;
					anyAb |= ok;
					allAb &= ok;
				}
				engAbMismatch = anyAb && !allAb;
			}
			// the yaw the thrust split would make right now without the match: each engine's dry thrust from its RPM, plus
			// the afterburner it would have at the pilot's throttle (rawThrottle, before the match holds it under the
			// afterburner's start), less the weakest one's, on its nozzles' arms.
			// 0.4.3: the afterburner was left out ("equal on both when both have it, and the throttle is held under it
			// when only one has") - but the throttle is only held while the match is in. 23 Oct 00:51 log, 62-70 s: vertical
			// at 35-42 m/s, the match released as the AoA fell under the cap, the healthy engine lit its afterburner and
			// the split was 7.1 deg/s^2 against 6-7 of yaw authority; the prediction without the 37 kN of afterburner
			// said 3.8, splitUsage peaked at 0.80 (the match latches above 0.8), and the nose yawed 0.3 -> 9.5 deg/s in 8 s
			// with the stick centred. With it the prediction is 1.5 and the match holds.
			splitYawPred = 0f;
			splitUsage = 0f;
			if (engTotal > 1 && engOk == engTotal && nE >= 2)
			{
				float tgt = float.MaxValue;
				int k = 0;
				foreach (Turbojet eg in tvc.engines)
				{
					if (eg != null && Acc.TF_operable(eg) && k < nE)
					{
						ref EngModel emk = ref engModel[k];
						float ratio = Mathf.Clamp01(eg.GetThrustRatio());   // Ternion 0.1.0: the Turbojet's own thrust ratio (the Turbojet rpm rule does not apply)
						engUnmatched[k] = emk.t0 * ratio;
						if (emk.ab > 0f && emk.rMax > 0f && emk.ceil / emk.rMax > 0.85f && rawThrottle > emk.abStart)
						{
							engUnmatched[k] += emk.ab * Mathf.Clamp01((rawThrottle - emk.abStart) / Mathf.Max(0.01f, emk.abEnd - emk.abStart));
						}
						tgt = Mathf.Min(tgt, engUnmatched[k]);
						k++;
					}
				}
				Vector3 mW = Vector3.zero;
				k = 0;
				foreach (Turbojet eg in tvc.engines)
				{
					if (eg != null && Acc.TF_operable(eg) && k < nE)
					{
						float extra = engUnmatched[k] - tgt;
						JetNozzle[] nzS = Acc.TF_nozzles(eg);
						if (extra > 1f && nzS != null && nzS.Length > 0)
						{
							float per = extra / nzS.Length;
							foreach (JetNozzle jS in nzS)
							{
								Transform tS = (jS != null) ? Acc.JN_thrustT(jS) : null;
								if (tS != null)
								{
									mW += Vector3.Cross(tS.position - cg, tS.forward * per);
								}
							}
						}
						k++;
					}
				}
				Quaternion rInv = Quaternion.Inverse(((Component)(object)ac).transform.rotation);
				Vector3 mA = rInv * mW;
				splitYawPred = Mathf.Abs(AccYaw(new Vector3(0f - mA.z, 0f - mA.x, mA.y)));
				float qScale = Mathf.Clamp((ias * ias) / (asymSweepIas * asymSweepIas), 0.05f, 2f);
				float cyNow = asymValid ? (asymCyNow * qScale) : 0f;
				splitUsage = (cyNow > 1f) ? (splitYawPred / Mathf.Max(1f, AsymResMaxFrac * cyNow)) : ((splitYawPred > 1f) ? 9f : 0f);
			}
			engOut = engTotal > 1 && engOk > 0 && (engOk < engTotal || (tMax > 20000f && tMin < 0.15f * tMax));
			float ratioNow = (engTotal > 1 && engOk == engTotal && tMax > 20000f) ? Mathf.Clamp01(tMin / tMax) : 1f;
			engRatioF += (ratioNow - engRatioF) * Mathf.Clamp01(Time.fixedDeltaTime / 1f);
			// the asymmetric state is the MEASURED thrust split (filtered 1 s, with hysteresis), and only while the split is
			// allowed: with the thrust-match limiter holding the throttle (AoA above the damaged-engine cap) the thrust is
			// symmetric and there is nothing to cap for - the pilot keeps the full AoA until he brings the AoA down himself
			engAsym = !engOut && engTotal > 1 && engOk == engTotal && tMax > 20000f && !matchFull && (engAsym ? (engRatioF < EngineAsymExit) : (engRatioF < EngineAsymEnter));
			if (rudTotal <= 0)
			{
				yawHealth = 1f;
				return;
			}
			int act = 0;
			int any = 0;
			foreach (SurfaceEffector s1 in surfaces)
			{
				if (s1.active)
				{
					any++;
					if (s1.kind == SurfaceKind.Rudder)
					{
						act++;
					}
				}
			}
			yawHealth = (any == 0) ? 1f : ((float)act / (float)rudTotal);
			// Revoker 0.1.0: the fin can go without the rudder (vstab2, the upper fin, carries no surface): the share of the
			// vertical-tail area still attached counts too
			if (finAreaTotal > 0.1f)
			{
				float aF = 0f;
				foreach (AeroPart pF in finParts)
				{
					if (pF != null && !pF.IsDetached())
					{
						aF += Mathf.Max(0f, Acc.AP_wingArea(pF));
					}
				}
				yawHealth = Mathf.Min(yawHealth, Mathf.Clamp01(aF / finAreaTotal));
			}
		}

		// Revoker 0.1.0: vertical tail parts (fins and rudder) and their total area at spawn
		private readonly List<AeroPart> finParts = new List<AeroPart>();

		private float finAreaTotal;

		// ---- 2.0.12 pitch stick curve: monotone cubic (Fritsch-Carlson) through (0,0), (s24,24), (s36,36), straight on
		// past 36 with the end slope -----------------------------------------------------------------------------------
		private readonly float[] scX = new float[3];

		private readonly float[] scY = new float[3] { 0f, 24f, 36f };

		private readonly float[] scM = new float[3];

		private float scK24 = -1f;

		private float scK36 = -1f;

		private void StickCurveSetup()
		{
			float a = Plugin.StickAoA24;
			float b = Plugin.StickAoA36;
			if (a == scK24 && b == scK36)
			{
				return;
			}
			scK24 = a;
			scK36 = b;
			scX[0] = 0f;
			scX[1] = a;
			scX[2] = b;
			float d0 = (scY[1] - scY[0]) / (scX[1] - scX[0]);
			float d1 = (scY[2] - scY[1]) / (scX[2] - scX[1]);
			// interior tangent: weighted harmonic mean (monotone, no overshoot); ends: one-sided, kept inside 3x the secant
			scM[1] = (d0 * d1 > 0f) ? (3f * (scX[2] - scX[0]) / ((2f * scX[2] - scX[1] - scX[0]) / d0 + (scX[2] + scX[1] - 2f * scX[0]) / d1)) : 0f;
			scM[0] = Mathf.Clamp(((2f * (scX[1] - scX[0]) + (scX[2] - scX[1])) * d0 - (scX[1] - scX[0]) * d1) / (scX[2] - scX[0]), 0.2f * d0, 3f * d0);
			scM[2] = Mathf.Clamp(((2f * (scX[2] - scX[1]) + (scX[1] - scX[0])) * d1 - (scX[2] - scX[1]) * d0) / (scX[2] - scX[0]), 0.2f * d1, 3f * d1);
		}

		internal float StickCurve(float s)
		{
			if (s <= 0f)
			{
				return 0f;
			}
			if (s >= scX[2])
			{
				return scY[2] + scM[2] * (s - scX[2]);
			}
			int k = (s < scX[1]) ? 0 : 1;
			float h = scX[k + 1] - scX[k];
			float t = (s - scX[k]) / h;
			float t2 = t * t;
			float t3 = t2 * t;
			return (2f * t3 - 3f * t2 + 1f) * scY[k] + (t3 - 2f * t2 + t) * h * scM[k] + (-2f * t3 + 3f * t2) * scY[k + 1] + (t3 - t2) * h * scM[k + 1];
		}

		internal float StickCurveInv(float aoa)
		{
			if (aoa <= 0f)
			{
				return 0f;
			}
			if (aoa >= scY[2])
			{
				return scX[2] + (aoa - scY[2]) / Mathf.Max(1f, scM[2]);
			}
			float lo = 0f;
			float hi = scX[2];
			for (int i = 0; i < 30; i++)
			{
				float mid = 0.5f * (lo + hi);
				if (StickCurve(mid) < aoa)
				{
					lo = mid;
				}
				else
				{
					hi = mid;
				}
			}
			return 0.5f * (lo + hi);
		}

		// ---- 2.0.12 VMC protection: last resort only -------------------------------------------------------------------
		// With an engine out the live engine's thrust yaws the jet; the rudders hold it down to the speed where their
		// authority (falling with q) equals that yaw (afterburner: ~64 m/s IAS on the bench airframe). Below it nothing the
		// FCS has can hold the heading and the jet yaws off into a departure (10:37 log, 150-158 s: sideslip -1 -> -67 deg
		// at 12 deg AoA, 47-58 m/s, every surface on its stop). The throttle is NOT touched while the FCS still holds the
		// yaw: only once the yaw is saturated AND the sideslip is running away (past VmcBeta and growing, or past
		// 2 x VmcBeta) is the throttle input pulled back, at VmcCutRate per s, until the sideslip stops growing; it is given
		// back at VmcReleaseRate per s once the sideslip is under VmcBeta / 2 and the yaw has room again. The pilot's
		// throttle is never raised, only capped.
		internal static bool VmcProtect = true;

		internal static float VmcBeta = 5f;

		internal static float VmcQuiet = 1f;

		private float vmcQuietT;

		internal static float VmcCutRate = 0.8f;

		internal static float VmcReleaseRate = 0.2f;

		public float vmcLim = 1f;

		internal static bool VmcPredict = true;

		internal static float VmcPredUseMax = 0.7f;

		internal static float VmcPredHyst = 0.1f;

		internal static float VmcPredCutRate = 1f;

		internal static float VmcPredReleaseRate = 0.25f;

		public float vmcPredLim = 1f;

		public float vmcPredUse;

		private float ThrustYawNow()
		{
			if (tvc == null)
			{
				return 0f;
			}
			return tvc.active ? AccYaw(((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : AccYaw(FixedThrustLat());
		}

		private float yawClampExcess;

		internal static bool YawSatCountClamp = true;

		public bool vmcOn;

		private float vmcBetaPrev;

		private float vmcSatF;

		// Revoker 0.2.7 (FinLossTvcHold): with the fin gone the yaw nozzle is the yaw axis, and the game stops vectoring above
		// the engine's thrustVectoringMaxAirspeed (250 m/s; Turbojet.FixedUpdate zeroes the nozzle angles past it). With the fin
		// shot off nothing held the yaw there: 28 Oct 21:22 log, fin and wings gone, full afterburner to 247 m/s, the nozzles cut
		// out at 211.0 s and the jet yawed round (r -128 deg/s, sideslip 85 deg in 2 s). While yawHealth is under 0.5 (the FCS
		// flying, TVC enabled) the cut-off is raised to FinLossTvcSpeed on this jet's engines; put back once the fin is whole
		// (repair / respawn) or the FCS lets go. Holding the throttle down below the cut-off was tried instead: the nozzle's
		// yaw authority is the thrust, and the bench departed at 230 m/s with the throttle cut (fin + wings gone).
		private readonly Dictionary<Turbojet, float> vecMaxOrig = new Dictionary<Turbojet, float>();

		public bool tvcHoldOn;

		private void TvcCutoffStep()
		{
			if (tvc == null)
			{
				return;
			}
			bool hold = Plugin.FinLossTvcHold && Engaged && !Failed && !Plugin.DisableTVC && tvcy != null && yawHealth < 0.5f;
			tvcHoldOn = hold;
			foreach (Turbojet e in tvc.engines)
			{
				if (e == null)
				{
					continue;
				}
				ref float vm = ref Acc.TF_vecMaxSpeed(e);
				if (!vecMaxOrig.TryGetValue(e, out float o))
				{
					o = vm;
					vecMaxOrig[e] = o;
				}
				vm = hold ? Mathf.Max(o, FinLossTvcSpeed) : o;
			}
		}

		public void VmcApply(ControlInputs inp, float dt)
		{
			TvcCutoffStep();
			if (!VmcProtect || !Engaged || Failed || inp == null || Mode == FcsMode.Ground || wheelsTouch)
			{
				vmcLim = 1f;
				vmcPredLim = 1f;
				vmcPredUse = 0f;
				vmcOn = false;
				vmcBetaPrev = beta;
				abGateOn = false;
				matchYawOn = false;
				vmcMatch = false;
				matchF = 0f;
				matchFull = false;
				ThrustMatchClear();
				return;
			}
			float bDot = (beta - vmcBetaPrev) / Mathf.Max(dt, 0.001f);
			vmcBetaPrev = beta;
			// saturation: the yaw target out of reach (yawSat) or the asymmetric-load model's yaw usage past 1 at this AoA
			// yaw usage including the dead engine's thrust (the cap no longer counts it)
			float uY = asymValid ? (Mathf.Abs(asymAyNow) / Mathf.Max(asymCyNow, 1f)) : 0f;
			float satNow = Mathf.Max(yawSat, Mathf.Clamp01((uY - 0.9f) / 0.3f));
			vmcSatF += (satNow - vmcSatF) * Mathf.Clamp01(dt / 0.3f);
			// 2.0.13: sideslip from the pilot's own roll / pedal is not the engine's doing. The 13:07 log cut the throttle at
			// 80-87 m/s (290-315 km/h) on full-stick rolls toward the dead engine (sideslip -10..-16 from the roll, with the
			// rudders busy holding the thrust): those don't count. The lateral input (and the roll rate it makes) must have
			// been quiet for VmcQuiet s, and the sideslip must be on the side the live engine's thrust pushes it.
			bool latQuiet = Mathf.Abs(sr) < 0.15f && Mathf.Abs(sy) < 0.15f && Mathf.Abs(omega.x * 57.29578f) < 20f;
			vmcQuietT = latQuiet ? (vmcQuietT + dt) : 0f;
			bool thrustSide = !asymValid || asymAyNow * beta < 0f;
			bool growing = Mathf.Abs(beta) > VmcBeta && beta * bDot > 0f;
			bool runaway = (engOut || engAsym) && vmcSatF > 0.5f && thrustSide && (vmcQuietT > VmcQuiet || Mathf.Abs(beta) > 3f * VmcBeta) && (growing || Mathf.Abs(beta) > 2f * VmcBeta);
			// with a weak (running) engine the run-away is answered by the thrust match, not by cutting both engines
			// (23 Oct 00:06 F-22E log: both throttled to T/W 0.13 for 5 s at 40-60 m/s): the match holds until the
			// sideslip has been calm for 2 s
			bool weakEng = engAbRatio < EngineAsymEnter && !engOut;
			if (runaway && weakEng && Plugin.EngineAbAoAGate && EngineOutFixedCap)
			{
				vmcMatch = true;
				vmcMatchCalmT = 0f;
				runaway = false;
			}
			if (vmcMatch)
			{
				bool calmM = Mathf.Abs(beta) < 0.5f * VmcBeta && vmcSatF < 0.3f;
				vmcMatchCalmT = calmM ? (vmcMatchCalmT + dt) : 0f;
				if (vmcMatchCalmT > 2f || !weakEng)
				{
					vmcMatch = false;
				}
			}
			if (runaway)
			{
				if (!vmcOn)
				{
					vmcLim = Mathf.Min(vmcLim, inp.throttle);
				}
				vmcOn = true;
				vmcLim = Mathf.Max(0f, vmcLim - VmcCutRate * dt);
			}
			else
			{
				bool calm = Mathf.Abs(beta) < 0.5f * VmcBeta && vmcSatF < 0.3f;
				if (calm || !(engOut || engAsym))
				{
					vmcLim = Mathf.Min(1f, vmcLim + VmcReleaseRate * dt);
				}
				if (vmcLim >= 0.999f)
				{
					vmcOn = false;
				}
			}
			// Compass 0.1.11 (VmcPredict): the throttle comes back BEFORE the yaw runs out. With an engine out, the live engine's
			// yaw (fixed while the throttle is) against the yaw authority there (falling with q, judged at the predicted q
			// of a decelerating climb): past VmcPredUse of it the throttle limit comes down at up to VmcPredCutRate per s,
			// starting from the pilot's throttle, and goes back at VmcPredReleaseRate once the usage is VmcPredHyst under it.
			// The runaway cut above stays as the last resort. 5 Nov 22:18 log, 202-220 s: one engine out, full aft in a
			// 67 deg climb, 75 -> 36 m/s at full throttle; the rudders ran out at 44 m/s (sideslip -2 -> -43 deg, spin) and the
			// runaway cut began only at 218.8 s, after the departure. (The live engine's yaw had read 0 on this jet until
			// 0.1.11: the T/A-30's engines have no vectoring transform, so the FCS saw none of their thrust moment.)
			bool predOn = VmcPredict && (engOut || engAsym) && asymValid;
			if (predOn)
			{
				float qS = Mathf.Clamp((ias * ias) / (asymSweepIas * asymSweepIas), 0.05f, 2f) * Mathf.Clamp(qPredK, 0.3f, 1f);
				float cyQ = asymCyNow * qS;
				float aT = Mathf.Abs(ThrustYawNow());
				vmcPredUse = aT / Mathf.Max(cyQ, 1f);
				if (vmcPredUse > VmcPredUseMax)
				{
					if (vmcPredLim >= 0.999f)
					{
						vmcPredLim = Mathf.Min(vmcPredLim, inp.throttle);
					}
					vmcPredLim = Mathf.Max(0f, vmcPredLim - VmcPredCutRate * Mathf.Clamp01((vmcPredUse - VmcPredUseMax) / 0.15f) * dt);
				}
				else if (vmcPredUse < VmcPredUseMax - VmcPredHyst)
				{
					vmcPredLim = Mathf.Min(1f, vmcPredLim + VmcPredReleaseRate * dt);
				}
			}
			else
			{
				vmcPredUse = 0f;
				vmcPredLim = Mathf.Min(1f, vmcPredLim + VmcPredReleaseRate * dt);
			}
			if (vmcLim < 0.999f)
			{
				inp.throttle = Mathf.Min(inp.throttle, vmcLim);
			}
			if (vmcPredLim < 0.999f)
			{
				inp.throttle = Mathf.Min(inp.throttle, vmcPredLim);
			}
			// 0.4.1 thrust match: with one engine down on thrust (predicted full-throttle ratio under EngineAsymEnter) and the
			// AoA above the damaged-engine cap, the healthy engine is held down to the damaged one's thrust at whatever
			// throttle the pilot sets: the game derates a damaged engine's whole throttle-to-RPM map (target RPM =
			// Lerp(minRPM, minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5), throttle)), so the two only ever match at
			// idle by throttle alone; the healthy engine's parasiticThrustLoss (thrust = (maxThrust - loss) x RPM ratio,
			// unused on this jet) is set each tick from the measured RPMs so its dry thrust equals the weak one's. If the weak
			// engine cannot light its afterburner the throttle input is also held at the afterburner's start. No thrust
			// split, so no AoA cap for it: the pilot keeps the full AoA, and the thrust comes back once he brings the AoA
			// down to the cap himself (on above cap + 2, off under cap + 0.5: riding the limiter counts as inside). With
			// an engine OUT there is nothing to match: above EngineOutAoACap the throttle is held at military power. The
			// VMC protection (measured sideslip run-away) stays the last resort.
			bool weak = engAbRatio < EngineAsymEnter;
			bool gateNeed = Plugin.EngineAbAoAGate && EngineOutFixedCap && (engOut || weak);
			float tgtF = 0f;
			if (gateNeed && engOut)
			{
				// an engine out: nothing to match; above EngineOutAoACap the throttle is held at the afterburner's start
				float capO = Plugin.EngineOutAoACap;
				float aAbsO = Mathf.Abs(alpha);
				abGateOn = abGateOn ? (aAbsO > capO + 0.5f) : (aAbsO > capO + 2f);
				matchYawOn = false;
				vmcMatch = false;
			}
			else if (gateNeed)
			{
				float capA = Plugin.EngineDamagedAoACap;
				float aAbs = Mathf.Abs(alpha);
				float spDir = sp * ((alpha >= 0f) ? 1f : (-1f));
				// 0.4.2 / 2.4.2: the match is timed, not placed. The AoA limiter starts braking for the cap well before it, and
				// a match that only completes as the AoA arrives at the cap leaves the pilot feeling the brake until it does
				// (23 Oct 00:39 log: full match 1.4-1.9 s after it began at 230-360 kt, where the AoA came up at 18 deg/s,
				// against 0.23 s at 95-170 kt and 35 deg/s). Now: with the stick pulling toward the cap, the match goes to 1
				// once the AoA is predicted to reach the cap within MatchLeadTime s at its present rate (or is already within
				// MatchBand of it), so it is complete, at MatchRiseRate, before the limiter would have begun to brake,
				// whatever the speed; above the cap it is 1 whatever the stick; stick relaxed under the cap it is 0 (thrust
				// comes back).
				float aRate = alphaDot * ((alpha >= 0f) ? 1f : (-1f));
				float tCap = (capA - aAbs) / Mathf.Max(aRate, 0.5f);
				float ramp = 0f;
				if (aAbs > capA + 2f)
				{
					ramp = 1f;
				}
				else if (spDir > 0.15f && (aAbs > capA - MatchBand || (aRate > 0.5f && tCap < MatchLeadTime)))
				{
					ramp = 1f;
				}
				else if (spDir >= 0.05f && aAbs > capA - MatchBand)
				{
					ramp = Mathf.Clamp01((aAbs - (capA - MatchBand)) / MatchBand);
				}
				// and on prediction: the split's yaw against what the fins have at this AoA and speed (see the engine scan)
				matchYawOn = matchYawOn ? (splitUsage > MatchYawOff) : (splitUsage > MatchYawOn);
				tgtF = Mathf.Max(ramp, (matchYawOn || vmcMatch) ? 1f : 0f);
				abGateOn = tgtF > 0.001f || matchF > 0.001f;
			}
			else
			{
				abGateOn = false;
				matchYawOn = false;
				vmcMatch = false;
			}
			if (abGateOn && engOut)
			{
				inp.throttle = Mathf.Min(inp.throttle, engAbStart);
				matchF = 0f;
				matchFull = false;
				ThrustMatchClear();
			}
			else if (gateNeed && !engOut)
			{
				matchF = Mathf.MoveTowards(matchF, tgtF, ((tgtF > matchF) ? MatchRiseRate : MatchFallRate) * dt);
				matchFull = matchF >= 0.995f;
				if (matchF > 0.001f)
				{
					if (engAbMismatch)
					{
						// the afterburner the weak engine cannot light fades out with the match
						inp.throttle = Mathf.Min(inp.throttle, Mathf.Lerp(1f, engAbStart, matchF));
					}
					ThrustMatchApply(matchF);
				}
				else
				{
					ThrustMatchClear();
				}
			}
			else
			{
				matchF = 0f;
				matchFull = false;
				ThrustMatchClear();
			}
		}

		// hold every stronger engine's dry thrust down to the weakest one's, from the measured RPMs (see VmcApply)
		private void ThrustMatchApply(float frac)
		{
			float target = float.MaxValue;
			foreach (Turbojet eg in tvc.engines)
			{
				if (eg != null && Acc.TF_operable(eg))
				{
					float rMin = Acc.TF_minRPM(eg);
					float rMax = Acc.TF_maxRPM(eg);
					float ratio = Mathf.Clamp01(eg.GetThrustRatio());   // Ternion 0.1.0: Turbojet thrust ratio
					target = Mathf.Min(target, Mathf.Max(0f, Acc.TF_staticThrust(eg)) * ratio);
				}
			}
			if (target == float.MaxValue)
			{
				ThrustMatchClear();
				return;
			}
			float lossMax = 0f;
			foreach (Turbojet eg in tvc.engines)
			{
				if (eg == null)
				{
					continue;
				}
				float loss = 0f;
				if (Acc.TF_operable(eg))
				{
					float rMin = Acc.TF_minRPM(eg);
					float rMax = Acc.TF_maxRPM(eg);
					float ratio = Mathf.Clamp01(eg.GetThrustRatio());   // Ternion 0.1.0: Turbojet thrust ratio
					float t0 = Mathf.Max(0f, Acc.TF_staticThrust(eg));
					if (ratio > 0.001f)
					{
						loss = Mathf.Clamp(t0 - target / ratio, 0f, t0) * Mathf.Clamp01(frac);
					}
				}
				Acc.TF_parasitic(eg) = loss;
				lossMax = Mathf.Max(lossMax, loss);
			}
			engMatchLoss = lossMax;
			matchApplied = true;
		}

		public void ThrustMatchClear()
		{
			if (!matchApplied)
			{
				return;
			}
			matchApplied = false;
			engMatchLoss = 0f;
			if (tvc == null)
			{
				return;
			}
			foreach (Turbojet eg in tvc.engines)
			{
				if (eg != null)
				{
					Acc.TF_parasitic(eg) = 0f;
				}
			}
		}

		// ---- 2.0.12 fuel trim: lateral CG moved with fuel when yaw or roll runs short -----------------------------------
		// Moving the CG sideways by d (m) changes the jet's moments by -d x (thrust + lift) about the new CG: toward the
		// live engine it shortens that engine's lever arm (less engine-out yaw: at full afterburner ~3 deg/s^2 per 10 cm,
		// of ~18), and with damage it can also offset an aerodynamic yaw or roll asymmetry. It costs roll: lift through a
		// CG 10 cm off the centreline rolls the jet ~15 deg/s^2 per g. So nothing moves until the yaw or roll usage at the
		// present AoA (load / authority, the asymmetric-load model's numbers) passes FuelTrimStart; then the offset is
		// the one that best evens out the roll and yaw usage (minimax), with the roll side judged at 1.5 g or more so a
		// pull does not run the roll out, at most FuelTrimMaxOffset either side. When both usages are back under
		// FuelTrimStop for 2 s the normal balancing brings the CG back to the centreline.
		internal static float FuelTrimStart = 0.75f;

		internal static float FuelTrimStop = 0.5f;

		internal static float FuelTrimMaxOffset = 0.3f;

		public bool fuelTrimOn;

		public float fuelTrimLat;

		public float fuelTrimDelta;

		private float fuelTrimTimer;

		private float fuelTrimCalm;

		private void FuelTrimStep(float dt, Quaternion rootRot)
		{
			if (!Plugin.FuelTrim || !Plugin.FuelBalanceOn || !asymValid || wheelsTouch || Mode == FcsMode.Ground || (UnityEngine.Object)(object)ac == null)
			{
				fuelTrimOn = false;
				return;
			}
			float cr = Mathf.Max(asymCrNow, 1f);
			float cy = Mathf.Max(asymCyNow, 1f);
			float ur = Mathf.Abs(asymArNow) / cr;
			float uy = Mathf.Abs(asymAyNow) / cy;
			float uMax = Mathf.Max(ur, uy);
			fuelTrimCalm = (uMax < FuelTrimStop) ? (fuelTrimCalm + dt) : 0f;
			if (fuelTrimOn && fuelTrimCalm > 2f)
			{
				fuelTrimOn = false;
			}
			fuelTrimTimer -= dt;
			if (fuelTrimTimer > 0f || (!fuelTrimOn && uMax < FuelTrimStart))
			{
				return;
			}
			fuelTrimTimer = 0.5f;
			Transform tr = ((Component)(object)ac).transform;
			float m = Mathf.Max(1000f, mass);
			Vector3 fT = tr.forward * (tw * m * 9.81f);
			Vector3 fL = tr.up * (Mathf.Max(nz, 1.5f) * m * 9.81f);
			Quaternion inv = Quaternion.Inverse(rootRot);
			Vector3 w = inv * (-Vector3.Cross(tr.right, fT + fL));
			Vector3 dM = new Vector3(0f - w.z, 0f - w.x, w.y);
			float gR = AccRoll(dM);
			float gY = AccYaw(dM);
			float best = 0f;
			float bestU = uMax;
			for (int i = -30; i <= 30; i++)
			{
				float d = FuelTrimMaxOffset * i / 30f;
				float u = Mathf.Max(Mathf.Abs(asymArNow + gR * d) / cr, Mathf.Abs(asymAyNow + gY * d) / cy);
				if (u < bestU - 1e-4f)
				{
					bestU = u;
					best = d;
				}
			}
			if (!fuelTrimOn && uMax - bestU < 0.1f)
			{
				return;
			}
			float latNow = FuelBalance.LateralCg(ac);
			fuelTrimDelta = best;
			fuelTrimLat = latNow + best;
			fuelTrimOn = true;
		}

		public float revSenseDbg = 1f;

		// Ifrit 0.3.4: roll stick sense vs AoA for the velocity-vector roll: +1 up to RevFlowSenseStart (|AoA|), -1 from
		// RevFlowSenseEnd, smoothstep between
		private static float RevFlowSense(float a)
		{
			float a0 = Plugin.RevFlowSenseStart;
			float a1 = Mathf.Max(a0 + 1f, Plugin.RevFlowSenseEnd);
			float x = Mathf.Clamp01((Mathf.Abs(a) - a0) / (a1 - a0));
			x = x * x * (3f - 2f * x);
			return 1f - 2f * x;
		}

		// 2.35.0 (PushPlan, see the statics): plan the push once at its start; hold; let it rise only
		private void PushPlanStep(float dt, float cA, float sA, float num11Stick)
		{
			// (fins damaged: the plan's IAS-based shed does not hold - the 2.19.0 nose hold and the governor pace the push instead)
			if (!PushPlan || isAI || tas < 40f || yawHealth < 0.99f)
			{
				pushPlanOn = false;
				return;
			}
			float aAbs = Mathf.Abs(alpha);
			float sgA = (alpha >= 0f) ? 1f : -1f;
			bool pushing = sgA * sp < -0.3f;
			float rDeg = Mathf.Abs(omega.z * 57.29578f);
			// Su-30 0.2.4: the shed and the sideslip allowance by whether the nozzles work (TVC on: PushoverSideslip 15; off: 12)
			bool nozOut = NozzlesOut;
			float capPlan = Mathf.Clamp((nozOut ? PushPlanK0NoTvc : PushPlanK0) + (nozOut ? PushPlanKNoTvc : PushPlanK) * Mathf.Pow(Mathf.Max(1f, ias), PushPlanExp), 6f, 80f);
			float rLeft = Mathf.Sqrt(2f * capPlan * Mathf.Max(0f, (PushPlanSlip < 0f) ? PushSlipNow() : PushPlanSlip));
			// Su-30 0.3.0 (PushPlanHeldYawEnd): with the roll stick held, the yaw the held roll keeps at the plan's end (its coordinated yaw
			// at 10 deg AoA, pTEnd sin 10) is not yaw to shed - it stays as the rotation goes on. 0.2.5 counted all of the body yaw, so held
			// push-overs with TVC planned on 60-65 deg/s of yaw and ran on the 23 deg/s floor with 2-8 deg of sideslip against the 15
			// allowed (16:16 log, 230.4 s: r -64.5, the held roll's coordinated yaw at the bottom ~12; 2.07 s to 10 deg, peak sideslip 4.3)
			float pTEndP = (!PushPlanHeldYawEnd || stickRelW > 0.5f || num11Stick * pTraj.x <= 0f) ? 0f : Mathf.Min(Mathf.Abs(num11Stick), Mathf.Abs(pTraj.x));
			rDeg = Mathf.Max(0f, rDeg - pTEndP * 0.17364818f);
			float wv = Mathf.Max(0f, sgA * omegaV);
			if (!pushPlanOn)
			{
				if (pushing && aAbs > PushPlanA0 && rDeg > PushPlanR0)   // a push-over OF a rotation (push + roll entered together is the trade's)
				{
					pushPlanOn = true;
					pushPlanSide = sgA;
					float T0 = Mathf.Max(PushPlanTMin, Mathf.Max(0f, rDeg - rLeft) / capPlan);
					pushPlanT = T0;
					pushPlanQ = Mathf.Clamp(Mathf.Max(0f, aAbs - 10f) / T0 - PushPlanWvK * wv, PushPlanQMin, 60f);
					// 2.40.0: the 2.35.1 clamp under the sweep's AoA-rate ceiling is gone. It kept the plan 15 deg/s under the 60 deg/s ceiling
					// so the ceiling would not cut it mid-push; since 2.36.0 the ceiling stands aside under a plan (PushPlanOwnsPitch), and the
					// clamp only limited speed - the plan's own yaw model sets it now. Hybrid replays, the 20 pushes it bound: 10 deg AoA reached
					// 0.05-0.23 s sooner, sideslip +0..+2.6 deg (worst push unchanged, 8.3), the re-attach overshoot mostly smaller (23:10 70.4 s
					// 24.5 -> 3.5 deg/s: the plan now matches the jet's own pitch-down).
					pushPlanQNow = Mathf.Max(0f, -sgA * qCmd);
					pushPlanBleed = Mathf.Abs(pTraj.x) / T0;
					float pTEnd = (stickRelW > 0.5f || num11Stick * pTraj.x <= 0f) ? 0f : Mathf.Min(Mathf.Abs(num11Stick), Mathf.Abs(pTraj.x));
					pushPlanD = Mathf.Max(capPlan, (Mathf.Abs(pTraj.x) * Mathf.Abs(sA) - pTEnd * 0.17364818f) / T0);
				}
				return;
			}
			// end: the stick no longer pushes or the AoA changed side; below 10 deg the bound ramps out at PushPlanRelease (no kick at the
			// bottom) and lets go once it is above anything the stick can ask
			// 2.40.0 (2.36-dev's PushPlanEndAtBottom, now always): only the bottom's ramp-out ends it - a plan whose rate is the stick's full 60 deg/s mid-push is
			// still the plan (the yaw allows the full rate); ending there handed the push to the governor's caps mid-sweep (-60 -> -51)
			if (!pushing || sgA != pushPlanSide || (pushPlanQNow >= 60f && aAbs < 10f))
			{
				pushPlanOn = false;
				return;
			}
			if (aAbs < 10f)
			{
				pushPlanQ = 60f;
				pushPlanQNow = Mathf.MoveTowards(pushPlanQNow, 60f, PushPlanRelease * dt);
				return;
			}
			// rise only: what the yaw left now would allow, approached at PushPlanRise
			float Tn = Mathf.Max(0.2f, Mathf.Max(0f, rDeg - rLeft) / capPlan);
			float qNeed = Mathf.Clamp(Mathf.Max(0f, aAbs - 10f) / Tn - PushPlanWvK * wv, PushPlanQMin, 60f);
			if (qNeed > pushPlanQ) pushPlanQ = Mathf.Min(qNeed, pushPlanQ + PushPlanRise * dt);
			// Su-30 0.2.4 (F-22E 3.1.4/3.1.5, Tom 10 Nov 14:55: "set the pitch rate floor of 23 like raptor, and forget sideslip in those
			// cases"): the plan is computed as before; the rate the pitch law is held to is max(plan, PushOverMinRate), no sideslip guard
			pushPlanQNow = Mathf.MoveTowards(pushPlanQNow, Mathf.Max(pushPlanQ, Plugin.PushOverMinRate), PushPlanJerk * dt);
		}

		// Su-30 0.2.4: the nozzles do not cover pitch / yaw (TVC off, or thrust under LowThrustFixedAoATW)
		// Su-30 0.3.0 (AxisPedalRoll / MpoAxisReleaseStop / LowSpeedSlipTvc)
		internal static bool AxisPedalRoll = true;

		internal static float AxRelRollWeight = 0.05f;

		internal static float AxRelYawWeight = 8f;


		public float axRelDbg, lsSlipDbg, axRelYawLeft, lsSlipBodyW;

		// Su-30 0.3.1: 300 - at 8 the allocator traded the yaw away for the roll target (r -45 -> -58 deg/s, AoA to 111 on the 855 s replay)
		// Su-30 0.3.2 (LowSpeedSlipTvc, see LateralLaw)
		// Su-30 0.3.3: the pace window's TAS floor, 40 -> 20 m/s. The window (the clean jet's only check of the roll target against what the
		// yaw can follow) was off below a hard 40 m/s, where Tom's low-speed corkscrews (25-40 m/s) live: in the 16:16 log's 858 s replay
		// every lateral protection read idle while the sideslip built 8 -> 44 deg. Bench, 8 low-speed sideslip events x 6 replay offsets,
		// LowSpeedSlipTvc off: 0 of 48 runs more than 3 deg worse, 124 s peak 47.6 -> 41.0 deg; with LowSpeedSlipTvc on (0.3.2 default):
		// mean peak change -2.4 -> -3.2 deg, runs > 3 deg worse 3 -> 2. Held-stick velocity-vector roll within 1.5 deg/s on every event;
		// the regression suite identical.
		internal static float PaceTasMin = 20f;

		internal static bool MpoRelPastLimit = true;

		internal static bool TvcRollLim = false;   // 0.3.3 bench: helps post-stall corkscrews, departs the 858 s type (see the comment at pFull)

		internal static float TvcRollLimShare = 1f;

		public float tvcRollLimDbg;

		internal static float LsSlipYawWeight = 40f;

		internal static float LsSlipVvWeight = 30f;

		internal static float LsSlipStiffStart = 0.8f;

		internal static float LsSlipStiffFull = 0.4f;

		internal static bool LsGate = true;

		internal static float LsGateT = 0.15f;

		internal static float LsGateFade = 6f;

		internal static bool LsSmooth = true;

		internal static float LsOnsetSpan = 6f;

		internal static float LsCmdTau = 0.25f;

		internal static float LsWeightTau = 0.4f;

		internal static float LsCmdTauUp = 0.25f;

		internal static float LsWeightTauUp = 0.4f;

		internal static float LsGateFadeSmooth = 2.5f;

		internal static float LsRearmMargin = 2f;

		public float lsDRsF, lsBodyWF;

		public float lsCorrT, lsM0, lsProgDbg, lsGateW = 1f;

		public bool lsGateOff;

		private float lsSlipStiffF = 1f;

		// Su-30 0.3.0: the held roll's coordinated yaw at the plan's end is left out of the yaw the push plan sheds
		// Su-30 0.3.0: off. Tried with K 0.028 on the 23 log replays: no faster at the median than K alone (1.65 vs 1.72 s) but 3 pushes
		// past 20 deg of sideslip after the bottom (worst 36.9) against none (worst 19.5), and the bench pull-through 5.1 -> 8.7 deg
		internal static bool PushPlanHeldYawEnd = false;

		private bool NozzlesOut => Plugin.DisableTVC || noTvcOn || tvc == null || !tvc.active;

		// Su-30 0.2.4: the push-over sideslip allowance in force (PushoverSideslip with the nozzles, PushoverSideslipNoTvc without)
		private float PushSlipNow()
		{
			return NozzlesOut ? Plugin.PushoverSideslipNoTvc : Plugin.PushoverSideslip;
		}

		// Su-30 0.1.0 (MpoTvcAxisRoll)
		public float mpoAxisW;

		public float lowSpeedAxisW;

		private float lsStiffF = 1f;

		public float mpoAxisDeg;

		// the axis (rad from body x toward body z, in the roll / yaw plane; (-90, 90] deg) the canted nozzle differential turns the jet about:
		// the angular acceleration I^-1 x dM of a small differential, dM from the differential effector's own moment table
		private bool TvcAxis(out float th)
		{
			th = 0f;
			if (tvcd == null || !tvcd.active || tvcd.N < 3)
			{
				return false;
			}
			float dd = 0.25f * tvcd.max;
			Vector3 dM = tvcd.MAt(dd) - tvcd.MAt(0f - dd);
			float aR = IaInv[0, 0] * dM.x + IaInv[0, 1] * dM.y + IaInv[0, 2] * dM.z;
			float aY = IaInv[2, 0] * dM.x + IaInv[2, 1] * dM.y + IaInv[2, 2] * dM.z;
			if (aR * aR + aY * aY < 1E-10f)
			{
				return false;
			}
			if (aR < 0f)
			{
				aR = 0f - aR;
				aY = 0f - aY;
			}
			th = Mathf.Atan2(aY, aR);
			return true;
		}

		// Su-30 0.3.8 (F-22E 4.1.2): 1 where the load takes no roll authority from the stick's side (the load pulls that way), fading to 0 as the
		// trim it needs takes a tenth of the roll authority there (aRollGo: the stick-side authority after the load's trim)
		private float LoadSideWeight(float aRollGo)
		{
			return Mathf.Clamp01(1f - (aRoll - aRollGo) / Mathf.Max(1f, 0.1f * aRoll));
		}

		// Su-30 0.3.8 (F-22E 4.1.2): how much of a reversal's build-up the rudders' travel paces (0..1): with an asymmetric load (the 2.1.6 gate), toward
		// the side the load pulls, and where the yaw left after the load is what bounds the reversal (yawTop under the law's own reversal
		// acceleration; at low AoA the roll asks little yaw, yawTop is above it and nothing is paced)
		private float LoadRevPace(float aRollGo, float cosA, float yawTop)
		{
			if (!RevLoadRamp || !AsymSharedLat || !Plugin.AsymSharedLatOn || !asymValid || LoadRevYawK <= 0f)
			{
				return 0f;
			}
			float dyG = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan));
			if (dyG <= 0f || RudderCrossTime() <= 0f)
			{
				return 0f;
			}
			float revLaw = RevFloorK * aRoll / Mathf.Max(Mathf.Abs(cosA), 0.05f);
			return dyG * LoadSideWeight(aRollGo) * Mathf.Clamp01(2f * (1f - yawTop / Mathf.Max(1f, revLaw)));
		}

		// Su-30 0.3.8 (F-22E 4.1.2): the time the rudders need from where they are to the stop on the other side of center (0: no rudder left)
		private float RudderCrossTime()
		{
			float sum = 0f;
			int n = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind != SurfaceKind.Rudder || !se.active)
				{
					continue;
				}
				sum += (0.5f * (se.max - se.min) + Mathf.Abs(se.now - 0.5f * (se.max + se.min))) / Mathf.Max(1f, se.rate);
				n++;
			}
			return (n > 0) ? (sum / n) : 0f;
		}

		private void LateralLaw(float dt)
		{
			// Su-30 0.1.11 (LowSpeedTvcRoll, Tom 9 Nov 11:47): the low-speed band, from the IAS alone (the airframe's sideslip stiffness goes with
			// dynamic pressure: 11:08 log above 50 deg AoA, ~2.9 deg/s^2 per deg at 300-400 km/h, 0.6 at 150, 0.2 at 110)
			{
				float lsW = 0f;
				// Su-30 0.1.16 (Tom 20:49: "tvc roll always works for mpo regardless if it is on or off. So allow velocity vector roll when
				// the airframe still have enough sideslip stiffness"): in MPO the stiffness gate below decides on its own (MpoTvcAxisRoll),
				// whatever LowSpeedTvcRoll says; LowSpeedTvcRoll now switches it for the AoA law only
				bool lsMpo = Mode == FcsMode.MPO && Plugin.MpoTvcAxisRoll && Plugin.RollStopSlipProbe && slipSignR != 0f;
				if (((Plugin.LowSpeedTvcRoll && Mode == FcsMode.AoAG) || lsMpo) && !isAI && canted != null && tvcd != null && tvcd.active)
				{
					// the probe's restoring-yaw sign is latched the first time the jet flies below 10 deg AoA; until then its stiffness
					// reads 0, so the band falls back to the IAS keys
					if (Plugin.RollStopSlipProbe && slipSignR != 0f)
					{
						// Su-30 0.1.16 (Tom 9 Nov 20:47: "i want tvc roll only works when the sideslip stiffness is like a brick"): the
						// band follows the airframe's directional stiffness the FCS reads (slipStiff, deg/s^2 of restoring yaw per deg
						// of sideslip, from the 2 Hz probe) instead of IAS. In from LowSpeedTvcRollStiffStart, full at
						// LowSpeedTvcRollStiffFull, and only past 25-35 deg of total flow angle off the nose, where the canted nozzles are
						// what rolls the jet. 20:00-20:21 logs at 60-80 deg AoA: 0.43 at 120-150 km/h, 0.30 at 90-120, 0.69 at 150-200, 1.1 at
						// 200-260 (the leaf); flying backwards (100-180 deg) 0.2 at 120-200 km/h.
						// filtered over ~1 s: the probe reads each 0.5 s at the AoA and sideslip of that moment and wanders 0.25-0.55 in a
						// slow post-stall roll at 120-140 km/h (bench replay of the 20:21 log, 84-92 s)
						lsStiffF += (Mathf.Max(0f, slipStiffDbg) - lsStiffF) * Mathf.Clamp01(Time.fixedDeltaTime / 1f);
						float kS = lsStiffF;
						float wK = Mathf.Clamp01((Plugin.LowSpeedTvcRollStiffStart - kS) / Mathf.Max(0.01f, Plugin.LowSpeedTvcRollStiffStart - Plugin.LowSpeedTvcRollStiffFull));
						wK = wK * wK * (3f - 2f * wK);
						// the total flow angle off the nose (AoA and sideslip together), so a tumble through low AoA with 40 deg of sideslip
						// counts as post-stall too while slow flight near the nose does not
						float aTot = Mathf.Acos(Mathf.Clamp(Mathf.Cos(alpha * (MathF.PI / 180f)) * Mathf.Cos(beta * (MathF.PI / 180f)), -1f, 1f)) * 57.29578f;
						float wA = Mathf.Clamp01((aTot - 25f) / 10f);
						wA = wA * wA * (3f - 2f * wA);
						// flying backwards (AoA past 85-100 deg either way) the nozzles always roll the jet, as the stick says, in body
						// axes: a velocity-vector roll there turns the body the opposite way to the stick (the velocity points behind),
						// and the probe's restoring sign, latched in forward flight, means little (20:21 log: 0.6-1.0 at 100-155 deg)
						float wRev = Mathf.Clamp01((Mathf.Abs(alpha) - 85f) / 15f);
						wRev = wRev * wRev * (3f - 2f * wRev);
						lsW = Mathf.Max(wK * wA, wRev);
					}
					else
					{
						lsW = Mathf.Clamp01((Plugin.LowSpeedTvcRollStart - ias * 3.6f) / Mathf.Max(1f, Plugin.LowSpeedTvcRollStart - Plugin.LowSpeedTvcRollFull));
						lsW = lsW * lsW * (3f - 2f * lsW);
					}
				}
				// the probe steps every 0.5 s: the band moves at most 2 per second (full in or out in 0.5 s)
				lowSpeedAxisW = Mathf.MoveTowards(lowSpeedAxisW, lsW, 2f * Time.fixedDeltaTime);
			}
			float f = alpha * (MathF.PI / 180f);
			float num = Mathf.Cos(f);
			float num2 = Mathf.Sin(f);
			// Su-30 0.3.3 (TvcRollLim, Tom 10 Nov 18:29: why does the allocator not just use the nozzles' roll + yaw and the surfaces' yaw to fix
			// the sideslip): aRoll counts the flaperons, ailerons and the stabs' roll share only, so every roll-rate limit built on it (pFull, the
			// stop-authority limit, the high-AoA braking floor) ignored the canted nozzles' roll (25-31 deg/s^2 against 10 at 25-45 m/s): the
			// roll command was capped near 14 deg/s before the allocator saw it while coordination needed ~30. TvcRollLimShare x aRollTvc is
			// now part of the roll authority those limits use (the nozzles' yaw is already in aYaw and the yaw-rate limits).
			// OFF: bench, 16:16 log, 8 low-speed sideslip events x 6 replay offsets: 124 s peak sideslip 47.6 -> 41.1 (AoA max 129 -> 109),
			// 950 s 45 -> 37, 613 s 9.0 -> 7.7, but 858 s 34.1 -> 40.6 with 4 of 6 runs departing (AoA 115): the roll command now reaches the
			// ~-29 deg/s coordination asks for, the nozzles give it - and the left yaw that comes with it (r -56 -> -58.5), so the velocity-vector
			// roll grows and -pS tan(beta) pumps the AoA at 37-54 deg/s. pSlipMax tracks pMax there (the sideslip roll governor is not limiting).
			tvcRollLimDbg = (TvcRollLim && canted != null && tvcd != null && tvcd.active && !Plugin.DisableTVC) ? TvcRollLimShare * aRollTvc : 0f;
			pFull = (aRoll + tvcRollLimDbg) * (MathF.PI / 180f) / Mathf.Max(IaInv[0, 0], 1E-09f) / Mathf.Max(1f, 0f - Lp) * 57.29578f;
			pStockRef = StockRollRef();
			float trimR = ((RollTrimHeadroom && Plugin.AsymRollHeadroom) ? (Mathf.Sign(dTrimR) * Mathf.Min(Mathf.Max(0f, Mathf.Abs(dTrimR) - Mathf.Max(0f, Plugin.AsymDeadband)), 0.8f * aRoll)) : 0f);
			aRollPos = aRoll - Mathf.Max(0f, 0f - trimR);
			aRollNeg = aRoll - Mathf.Max(0f, trimR);
			bool mpoSep = Mode == FcsMode.MPO && Plugin.MpoSeparateAxes && !isAI;
			mpoSepDbg = mpoSep;
			// Su-30 0.3.0 (AxisPedalRoll, Tom 10 Nov 16:46: "make pedals also do tvc vector roll"): in the nozzle-axis rotation the pedals add
			// to the stick's rotation at full weight, whatever the AoA (below CoupleAoA + 14 deg they were faded, and with the sideslip hold
			// faded out by the axis blend they did nothing there)
			float axPedW = AxisPedalRoll ? Mathf.Max(mpoAxisW, lowSpeedAxisW) : 0f;
			float rollSide = mpoSep ? sr : (sr + Mathf.Max(Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.CoupleAoA) / 14f), axPedW) * sy);
			if (Mathf.Abs(rollSide) < 0.02f)
			{
				rollSide = pTraj.x;
			}
			float aRollGo = ((rollSide >= 0f) ? aRollPos : aRollNeg);
			float aRollStop = ((rollSide >= 0f) ? aRollNeg : aRollPos);
			YawHealthStep();
			float num3 = Mathf.Min(Plugin.MaxRollRate, Mathf.Min(pStockRef, Plugin.RollAuthorityUse * pFull * (aRollGo / aRoll)));
			if (Plugin.RollStopFactor > 0.01f)
			{
				num3 = Mathf.Min(num3, Plugin.RollStopFactor * (Mathf.Min(aRollGo, aRollStop) + tvcRollLimDbg));
			}
			float num4 = ((Mode == FcsMode.MPO) ? Plugin.MpoGPos : Plugin.GPos);
			float num5 = ((Mode == FcsMode.MPO) ? Plugin.MpoGNeg : Plugin.GNeg);
			float value = 1f;
			float t = Mathf.Clamp01((ias - 250f) / 150f);
			float num6 = Mathf.Lerp(6f, 3f, t);
			if (nz > num6)
			{
				value = Mathf.Lerp(1f, Mathf.Lerp(0.5f, 0.35f, t), (nz - num6) / Mathf.Max(0.5f, num4 - num6));
			}
			else if (nz < -1.5f)
			{
				value = Mathf.Lerp(1f, 0.6f, (-1.5f - nz) / Mathf.Max(0.5f, -1.5f - num5));
			}
			num3 *= Mathf.Clamp(value, 0.4f, 1f);
			num3 = (pMaxDbg = Mathf.Max(num3, 30f));
			float num7 = LowIasYawBoostAt(ias);
			// Su-30 0.1.4 (FallingLeafYawBoost, Tom 8 Nov: "at least 65 deg/s during a falling leaf", steady at 130-200 km/h;
			// default 1 = off). Past 55 deg AoA (fully by 65) the yaw-rate limit, and its braking floor, are x this: from 115 km/h
			// (fully by 145) to 250 km/h (gone by 300). Below 115 km/h the rotation is held as before (a held roll released there
			// departs on the bench at any rate). Off by default: on the bench the rotation it adds is rotation the nozzles cannot
			// stop - a 70 deg release at 120-180 km/h went from 10 to 15-18 deg of sideslip at 1.12 and to 38 at 1.55.
			float leafA = Mathf.Clamp01((Mathf.Abs(alpha) - LeafBoostA0) / 10f);
			float iasKL = ias * 3.6f;
			float leafI = Mathf.Clamp01((iasKL - LeafBoostIasLo) / 30f) * Mathf.Clamp01((LeafBoostIasHi + 50f - iasKL) / 50f);
			leafI = leafI * leafI * (3f - 2f * leafI);
			leafBoostDbg = Mathf.Lerp(1f, Mathf.Max(1f, Plugin.FallingLeafYawBoost), leafA * leafI);
			float num8 = (rMaxDbg = Mathf.Lerp(Plugin.MaxYawRate, Mathf.Clamp(leafBoostDbg * num7 * Plugin.YawStopFactor * aYaw, 20f * num7, Plugin.MaxYawRate), Mathf.Clamp01((Mathf.Abs(alpha) - 15f) / 10f)));
			// 1.8.45 braking floor: at high AoA the roll and yaw rate limits may not exceed what the stop-side authority,
			// at the IAS predicted from a rapid deceleration, brakes in the stop time of the reference manoeuvre (full
			// pedal + full lateral stick at 65 deg AoA, 280 km/h: RollStopFactor for roll, YawStopFactor x yaw boost there
			// for yaw). Replaces the fixed 30 deg/s roll / 20 x boost yaw floors there, which outran the authority below ~65 m/s.
			float gB = Mathf.Clamp01((Mathf.Abs(alpha) - LatBrakeA0) / Mathf.Max(1f, LatBrakeA1 - LatBrakeA0));
			latPredK = qPredK;
			if (LatBrakeFloor && gB > 0f)
			{
				// 1.9.3: yaw floor = the yaw braking at 270 km/h; from 210 to 150 km/h the allowed stop time grows smoothly
				// (smoothstep) by YawMarginLow (1.5 x: 50 % more yaw rate for the same authority, slower braking), flat below.
				// The roll stop time gets the same factor: at 65 deg AoA a velocity-vector turn needs body roll = cos(AoA) x its
				// rate, and a roll limit left at 1.0 s capped the turn (and so the yaw rate) at ~160 km/h.
				// The floor limits join the normal limits with a C1 soft minimum (no corner near 270 km/h).
				// Roll: the stop-side authority is the modelled asymmetric-load trim off the full authority, low-passed. The
				// measured trim (dTrimR) it used carried the turn's own dihedral / yaw-rate roll moment, which swings with
				// sideslip and yaw rate: in a max-AoA stick turn it moved 20 <-> 29 deg/s^2 at 1.4 Hz, the roll ceiling
				// 9 <-> 18 deg/s with it, the velocity-vector ceiling 20 <-> 42, and yaw rate 27 <-> 40 (a limit cycle).
				float iasKmh = ias * 3.6f;
				float sm = Mathf.Clamp01((YawMarginIasHi - iasKmh) / Mathf.Max(1f, YawMarginIasHi - YawMarginIasLo));
				sm = sm * sm * (3f - 2f * sm);
				float yawMargin = Mathf.Lerp(1f, Mathf.Max(1f, YawMarginLow), sm);
				yawMarginDbg = yawMargin;
				float tY = Plugin.YawStopFactor * LowIasYawBoostAt(BrakeRefIas) * yawMargin;
				float asymTrim = asymValid ? Mathf.Max(0f, Mathf.Abs(asymArNow) - Mathf.Max(0f, Plugin.AsymDeadband)) : 0f;
				float stopA = Mathf.Max(0.2f * aRoll, aRoll - Mathf.Min(asymTrim, 0.8f * aRoll)) + tvcRollLimDbg;
				rollStopAF = (rollStopAF < 0f) ? stopA : (rollStopAF + (stopA - rollStopAF) * Mathf.Clamp01(dt / Mathf.Max(0.02f, RollStopTau)));
				float pB = Mathf.Max(LatBrakeMinRate, Mathf.Max(0.01f, Plugin.RollStopFactor) * yawMargin * rollStopAF * qPredK);
				// Su-30 0.1.6 (YawThrustShare): the IAS-decay anticipation scales only the aero share of the yaw authority. On the
				// F-22E the yaw was its rudders, which do lose authority as the jet slows; on the Su-30 at 55-75 deg AoA most of it
				// is the canted nozzles, which do not. 8 Nov logs, 215-290 km/h, full roll stick: deceleration 22 m/s^2, qPredK 0.47,
				// yaw-rate limit 39 deg/s on 70 deg/s^2 of yaw authority (Ifrit: 63 on 70, not decelerating).
				float qYaw = yawThrustShareDbg + (1f - yawThrustShareDbg) * qPredK;
				float rB = Mathf.Max(LatBrakeMinRate, leafBoostDbg * tY * aYaw * qYaw);
				num3 = Mathf.Lerp(num3, SoftMin(num3, pB, BrakeSoftMin * 0.5f * (num3 + pB)), gB);
				// 1.9.4: mid-IAS band (290-330 km/h) with less yaw stop time (x YawMidFactor), smoothstep ramps outside it
				float mLo = Mathf.Clamp01((iasKmh - (Plugin.YawMidLoKmh - Plugin.YawMidRampKmh)) / Mathf.Max(1f, Plugin.YawMidRampKmh));
				float mHi = Mathf.Clamp01(((Plugin.YawMidHiKmh + Plugin.YawMidRampKmh) - iasKmh) / Mathf.Max(1f, Plugin.YawMidRampKmh));
				mLo = mLo * mLo * (3f - 2f * mLo);
				mHi = mHi * mHi * (3f - 2f * mHi);
				float yawMid = Mathf.Lerp(1f, Plugin.YawMidFactor, Mathf.Min(mLo, mHi));
				yawMidDbg = yawMid;
				num8 = Mathf.Lerp(num8, yawMid * SoftMin(num8, rB, BrakeSoftMin * 0.5f * (num8 + rB)), gB);
				pMaxDbg = num3;
				rMaxDbg = num8;
			}
			// Ifrit 0.3.1 (HighAoALateralBrakeMargin): the roll and yaw rate limits x this factor at high AoA, ramped in
			// smoothly (smoothstep) from HighAoALateralMarginStart to HighAoALateralMarginFull. < 1 = less allowed stop
			// time: lower roll / yaw rate for the same authority, harder braking; > 1 the other way.
			{
				float kL = Plugin.HighAoALatMargin;
				if (Mathf.Abs(kL - 1f) > 0.0001f)
				{
					float a0L = Plugin.HighAoALatMarginStart;
					float wL = Mathf.Clamp01((Mathf.Abs(alpha) - a0L) / Mathf.Max(0.5f, Plugin.HighAoALatMarginFull - a0L));
					wL = wL * wL * (3f - 2f * wL);
					float fL = Mathf.Lerp(1f, kL, wL);
					num3 *= fL;
					num8 *= fL;
					pMaxDbg = num3;
					rMaxDbg = num8;
				}
			}
			// 2.0.12: rudder lost (fin shot off): the roll and yaw rate limits shrink with the yaw authority left. A body roll
			// at AoA turns AoA into sideslip at p sin(AoA) unless the rudders make the matching yaw rate; the clean limits
			// (250 deg/s) asked one rudder for twice what it has (10:29 log, 276 s: -265 deg/s, sideslip -29, departure).
			// 2.0.13: and with the yaw already partly spent holding an asymmetry (engine out: the live engine's yaw), by the
			// share of the yaw authority left over at this AoA
			float yawLeft = 1f;
			if (DamageLatLimit && asymValid && asymCyNow > 1f)
			{
				yawLeft = Mathf.Clamp01(1f - Mathf.Abs(asymAyNow) / asymCyNow);
				// 2.1.4: past the rudder fade the stabs are the yaw source as well as a roll source, and the deflection that
				// trims the load's roll is the same deflection the yaw needs: the roll trim also comes out of the yaw left
				// (in proportion to the rudder fade). Only the load beyond the deadbands counts, so a clean jet is unchanged.
				// 12 Oct 18:29 log: at the asymmetric cap (~50 deg AoA) the limits still allowed 25-33 deg/s of roll and
				// 20-30 deg/s of yaw rate; centring the stick, the stabs could not brake both against the load and the
				// sideslip ran to 23 deg (74 s, 209 s).
				if (AsymSharedLat && Plugin.AsymSharedLatOn && asymCrNow > 1f)
				{
					float dbR = Mathf.Max(0f, Plugin.AsymDeadband);
					float rollUse = Mathf.Max(0f, Mathf.Abs(asymArNow) - dbR) / asymCrNow;
					float yawUse = Mathf.Max(0f, Mathf.Abs(asymAyNow) - 0.25f * dbR) / asymCyNow;
					yawLeft = Mathf.Clamp01(1f - yawUse - (1f - rudderAuth) * rollUse);
				}
			}
			sharedLatDbg = yawLeft;
			// 2.12.0: the fin-health factor fades with the rudders (rudderAuth, 50 -> 60 deg AoA): past the fade the fins are not the
			// yaw axis, so losing them takes nothing away there - the stabs, flaperons and the airframe's own balance carry the
			// velocity-vector roll, and the measured capacity says so (18:32 log, both fins gone: capZ 38 at 50-65 deg against 39 with
			// fins; 29-33 against 125-147 at 15-40 deg). Un-faded, rMax x 0.35 capped the corkscrew at 24 deg/s of yaw = 28 deg/s of
			// velocity roll at 65 deg with the margin untouched (sideslip 2-6 deg) - Tom: "isn't that when the rudders can't really
			// do any yaw, so why is it capped". Below the fade the open-loop factor stays until the rate cap's reversal probe
			// carries the roll demand (18:08 log, 541 s: capZ 99 -> 5-23 once a no-fin reversal loaded the flaperons: 17.6 deg).
			// The asymmetric-load share (yawLeft) is already rudderAuth-aware and is not faded.
			float finH = Mathf.Lerp(1f, yawHealth, Mathf.Clamp01(rudderAuth));
			if (DamageLatLimit && Mathf.Min(finH, yawLeft) < 0.999f)
			{
				// Revoker 0.1.0: with the vertical tail damaged and the nozzle unable to yaw only drag yaw is left (see DamageLatFloorNoTvc)
				float floorK = (tvcy != null && !tvcy.active && yawHealth < 0.5f) ? Mathf.Min(DamageLatFloor, DamageLatFloorNoTvc) : DamageLatFloor;
				float hK = Mathf.Max(floorK, Mathf.Min(finH, yawLeft));
				num3 = Mathf.Max(30f * hK, num3 * hK);
				num8 *= hK;
				pMaxDbg = num3;
				rMaxDbg = num8;
			}
			// Revoker 0.2.4 (HighAoALimitTau): above 30-45 deg AoA the roll / yaw rate limits are low-passed. They come from
			// the stop-side authority estimates, which flicker with the deflections (bench corkscrew: body-roll limit 25.6 <->
			// 29.8 deg/s every few frames; in game 13 <-> 28 deg/s below 40 m/s), and the velocity-vector roll command and the
			// yaw rate followed every step. The real changes (IAS decay) take seconds.
			if (LimTau > 0f)
			{
				if (pLimF < 0f)
				{
					pLimF = num3;
					rLimF = num8;
				}
				float kLim = Mathf.Clamp01(dt / LimTau);
				pLimF += (num3 - pLimF) * kLim;
				rLimF += (num8 - rLimF) * kLim;
				float wLim = Mathf.InverseLerp(30f, 45f, Mathf.Abs(alpha));
				num3 = Mathf.Lerp(num3, pLimF, wLim);
				num8 = Mathf.Lerp(num8, rLimF, wLim);
				pMaxDbg = num3;
				rMaxDbg = num8;
			}
			float num9 = Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.CoupleAoA) / 14f);
			float num10 = Mathf.Min(num3 / Mathf.Max(Mathf.Abs(num), 0.05f), num8 / Mathf.Max(Mathf.Abs(num2), 0.05f));
			// Ifrit 0.3.4: the reversed-flow roll sense blends smoothly with the AoA (RevFlowSenseStart -> End: +1 -> -1 through
			// 0) instead of flipping at 100 deg (released at 80). Past 90 deg the velocity-vector roll is mostly body yaw
			// (sin AoA) and only a little body roll (cos AoA), so the flip there reversed the whole rotation: 20 Oct 23:01 log,
			// MPO, full right stick held from -115 to -60 deg AoA, yawing right at 10-20 deg/s; at -80 deg the target went
			// -23 -> +29 deg/s (353.7 s, 394.4 s, the same at -79 deg) and the jet had to reverse its yaw. Now the rotation
			// stays the same through +-90 deg and the sense turns over near 180 deg, where the velocity-vector roll is nearly
			// all body roll and the flip keeps the stick rolling the jet the same way from the seat. In the blend band the
			// stick's roll command passes through zero rather than jumping.
			float revS = Plugin.RevFlowRollSense ? RevFlowSense(alpha) : 1f;
			revSenseDbg = revS;
			float num11raw = Mathf.Clamp(revS * (sr + Mathf.Max(num9, axPedW) * sy), -1f, 1f);
			// 2.16.0: a rate LIMIT bounds the rotation the stick may build; it may not cut a rotation the jet already has faster than
			// the yaw can shed it. With the stick on the rotation's side the limit is floored at the velocity roll the body yaw
			// implies (r / sin AoA: the coordinated roll for the yaw the jet has), which decays as the yaw brakes - the roll follows the
			// yaw down instead of braking away from it. 20:19 log (both fins gone), held push-over from 62 deg: as the AoA crossed the
			// rudder fade the fin-health factor took pSmax 61 -> 16, the trajectory braked at the tuned rate, p -28 -> -13 against a
			// yaw of -47 that needed -42 to stay coordinated, sideslip +20 deg/s, departure (and the bench twin, dmgsuite 'pushover
			// HELD', the same chain). The stick scales the target as always (easing off is the pilot's deceleration; a release is the
			// window's job); sin AoA floored at LatFloorSinMin so a low-AoA turn's yaw is not read as a rotation.
			latFloorDbg = 0f;
			pSStickRaw = Mathf.Abs(num11raw) * num10;   // 2.18.0: the stick's own target before the floor (the shed cap's credit)
			stickRelW = Mathf.Clamp01(1f - Mathf.Abs(num11raw) / Mathf.Max(0.02f, SweepNoReverseStick));   // 2.24.0: 1 = lateral stick released
			float finKL = Mathf.Clamp01(yawHealth / 0.5f);
			if (LatLimitYawFloor)
			{
				float sinF = ((num2 >= 0f) ? 1f : -1f) * Mathf.Max(Mathf.Abs(num2), LatFloorSinMin);
				float pSyaw = omega.z * 57.29578f / sinF;
				if (Mathf.Abs(num11raw) > 0.05f)
				{
					if (num11raw * pSyaw > 0f)
					{
						float fl = Mathf.Abs(pSyaw);
						if (fl > num10)
						{
							latFloorDbg = fl - num10;
							num10 = fl;
						}
					}
				}
				else if (ReleaseCoordinate && finKL < 1f && yawSatAlphaRateDn >= 0f && alphaDot * alpha < 0f && Mathf.Abs(pSyaw) > 5f)
				{
					// 2.18.0: stick released under a capped nose-down sweep with the fins gone: the roll stays coordinated with the yaw
					// the jet has (target = r / sin AoA, decaying as the yaw sheds) instead of going to zero while 20-30 deg/s of yaw is
					// still on (23:49 log, 122 s: roll target 9 -> 0 in 1.5 s, r 29 -> 22, sideslip -1 -> -53 deg). With the fins the
					// rudders shed the yaw in tenths and the stop stands as it is (finK 1: off).
					float fl = Mathf.Abs(pSyaw) * (1f - finKL);
					num10 = Mathf.Max(num10, fl);
					num11raw = ((pSyaw >= 0f) ? 1f : -1f) * Mathf.Min(1f, fl / Mathf.Max(1f, num10));
					latFloorDbg = fl;
				}
			}
			float num11 = num11raw * num10;
			pSAskStick = Mathf.Abs(num11);
			coupRollLim = 0f;
			if (Plugin.CouplingRollLimit > 0f && Ia[1, 1] > 1f && asymValid && tvc.M != null && tvc.N > 1)
			{
				float kIc = (Ia[2, 2] - Ia[0, 0]) / Ia[1, 1];
				float s2c = Mathf.Abs(Mathf.Sin(2f * f));
				{
					float gateC = Mathf.Max(pitchNeedUp, Mathf.Clamp01((Mathf.Sign(alpha) * alphaDot - 5f) / 15f)) * Mathf.Clamp01((Mathf.Abs(alpha) - CoupGateA0) / 10f);
					// 2.1.3: at the low-thrust AoA cap the gate stays on within LowCapCoupBand of the cap instead of waiting for the
					// AoA to rise: with the nozzles pinned and weak, the inertial pitch-up of the roll took the jet 2 deg past the cap,
					// pitch priority took the stabs back and the roll was cut and restored in a cycle.
					if (LowCapLatFix && Plugin.LowCapRollFix && LowThrustCapOn && alpha > 0f)
					{
						gateC = Mathf.Max(gateC, Mathf.Clamp01((alpha - (authCapEff - LowCapCoupBand)) / Mathf.Max(0.5f, LowCapCoupBand)));
					}
					gateC = Mathf.Max(gateC, (Plugin.AsymRollCoupling ? CoupHeldGate : 0f) * Mathf.Clamp01((Mathf.Abs(alpha) - 4f) / 4f) * AsymGate());
					// 2.1.10: at any AoA limit (not only the low-thrust cap) the gate stays on within CoupLimBand of the limit, and it
					// comes off over CoupGateRelease s instead of at once. It used to follow pitchNeed, which flips 0 <-> 1 as the pitch
					// error comes and goes: the roll limit switched off and on every ~0.6 s, the roll rate (and its inertial pitch-up)
					// jumped 60 <-> 110 deg/s, the nozzles went stop to stop and the pitch rate bounced (13 Oct logs, 45 deg limit,
					// 89-97 s: q 0..62 deg/s, AoA 39..52).
					if (CoupGateAtLimit && Plugin.CoupGateAtLimitOn)
					{
						float limU = Mathf.Min(Plugin.AoAPos, Mathf.Min(authCapEff, asymCapPosEff));
						float limD = Mathf.Max(Plugin.AoANeg, authCapNegEff);
						float nearLim = (alpha >= 0f) ? Mathf.Clamp01((alpha - (limU - CoupLimBand)) / Mathf.Max(0.5f, CoupLimBand)) : Mathf.Clamp01(((limD + CoupLimBand) - alpha) / Mathf.Max(0.5f, CoupLimBand));
						// only while the pilot holds the jet at the limit (stick toward it)
						nearLim *= Mathf.Clamp01(((alpha >= 0f) ? sp : (0f - sp)) / 0.5f);
						gateC = Mathf.Max(gateC, nearLim);
						// 2.2.2 (LoadedRollSteady, off by default): in a loaded roll (stick toward the AoA side, lateral held, AoA past
						// the gate start) the coupling roll limit stays on instead of following pitchNeed, and the roll governor judges
						// the nozzles' margin as it is, not projected 0.3 s ahead (their slew alone made the projection hit zero). 13 Oct
						// 11:08 log, full aft + full roll from 690 km/h: both switched every ~0.45 s, roll target 98 <-> 172 deg/s,
						// nozzles stop to stop, pitch rate 15..50, AoA 41..48 at the limit.
						float gLoaded = Plugin.LoadedRollSteadyOn ? 1f : CoupGateLoaded;
						if (gLoaded > 0f)
						{
							float loadedC = Mathf.Clamp01(((alpha >= 0f) ? sp : (0f - sp)) / 0.5f) * Mathf.Clamp01((Mathf.Abs(sr) - 0.05f) / 0.2f) * Mathf.Clamp01((Mathf.Abs(alpha) - CoupGateA0) / 10f);
							gateC = Mathf.Max(gateC, gLoaded * loadedC);
						}
						coupGateHeld = Mathf.Max(gateC, coupGateHeld - dt / Mathf.Max(0.05f, CoupGateRelease));
						gateC = coupGateHeld;
					}
					// Compass 0.1.3 (CoupNoTvcAlways): without pitch TVC the limit is on at any AoA past CoupNoTvcA0, whatever the stick.
					// The gate above waits for 12-22 deg AoA, a building AoA or a stick held at the limit; on the T/A-30 the stabs are the only
					// nose-down effector and at 60-75 m/s with the flaps down they are used up by the trim alone around 15-20 deg. A full
					// velocity-vector roll there (95-100 deg/s) put both elevators at the nose-down stop for 1.5 s and the inertial pitch-up
					// took the jet from 10 to 46 deg AoA with the stick centred (5 Nov 17:41 log, 84.5-87.4 s). The limit itself reads the
					// live nose-down authority at the present AoA and dynamic pressure, so where there is authority to spare it does not bind.
					bool noPitchTvc = tvc == null || tvc.max <= 0.5f || Plugin.DisableTVC;
					if (CoupNoTvcAlways && noPitchTvc)
					{
						gateC = Mathf.Max(gateC, Mathf.Clamp01((Mathf.Abs(alpha) - CoupNoTvcA0) / Mathf.Max(0.5f, CoupNoTvcSpan)));
					}
					if (gateC > 0.001f && s2c > 0.05f && kIc > 0.01f)
					{
						float tvLoC = float.MaxValue;
						float tvHiC = float.MinValue;
						for (int tvK = 0; tvK < tvc.N; tvK++)
						{
							tvLoC = Mathf.Min(tvLoC, tvc.M[tvK].y);
							tvHiC = Mathf.Max(tvHiC, tvc.M[tvK].y);
						}
						float netC = ((alpha >= 0f) ? (0f - (AsymCap.Interp(asymAlphas, asymNetDn, asymAlphas.Length, alpha) + (tvc.active ? (tvLoC * IaInv[1, 1] * 57.29578f) : 0f))) : (AsymCap.Interp(asymAlphas, asymNetUp, asymAlphas.Length, alpha) + (tvc.active ? (tvHiC * IaInv[1, 1] * 57.29578f) : 0f)));
						// Compass 0.1.7: with the asymmetric-load cap in force, less the nose-up the swept model leaves out (AsymPitchBias)
						if (AsymPitchBias && noPitchTvc && alpha >= 0f && asymCapPosEff < Plugin.AoAPos - 0.5f)
						{
							netC -= asymPitchBias;
						}
						float aAvC = Mathf.Max((CoupNoTvcAlways && noPitchTvc) ? CoupRollFloorNoTvc : CoupRollFloor, Plugin.CouplingRollLimit * netC) * (MathF.PI / 180f);
						float pLimC = Mathf.Sqrt(2f * aAvC / (kIc * s2c)) * 57.29578f;
						coupRollLim = pLimC;
						num10 = Mathf.Lerp(num10, Mathf.Min(num10, pLimC), gateC);
						num11 = Mathf.Clamp(num11, 0f - num10, num10);
					}
				}
			}
			if (AsymRollMax > 0f && (!AsymRollHardOnly || aRollGo < aRollStop))
			{
				float asymG = Mathf.Clamp01((asymUNow - AsymRollU0) / 0.3f);
				if (asymG > 0f)
				{
					num10 = Mathf.Min(num10, Mathf.Lerp(num10, AsymRollMax, asymG));
					num11 = Mathf.Clamp(num11, 0f - num10, num10);
				}
			}
			// Su-30 0.5.1 (AsymStopTime; Tom 11 Nov 17:26, 0.5.0 flown with 16 cm of lateral CG offset, nozzles locked: "the jet still tends
			// to tips to one side. I release roll and then the roll rate drifts a lot"). The coupled check places the AoA limit where the
			// weaker side still has its share of the flight-path roll; it says nothing of how much roll that share can stop. At the 50 deg
			// limit under 90 m/s the reserve toward the side that brakes a roll INTO the load's side is 10-29 deg/s^2 (cplRn / cplRp) and the
			// law let that roll reach 32-50 deg/s: 17:22 log 155.4 s, 36 deg/s on at the release, 74 deg of bank in the next 2.7 s and still
			// rolling; bench, 103 kt, 32 deg/s held at 50 deg: not stopped in 3.5 s (reserve 10-17, measured brake 9.3 deg/s^2). The roll
			// the stick may ask toward each side is now what the reserve on its braking side stops in AsymStopTime s (the reserve at the
			// present AoA from the coupled sweep; CplStopFloor at least), faded in with the load as the coupled check is. 0 s: 0.5.0.
			cplStopP = (cplStopN = 999f);
			cplLawRoll = num10;
			if ((Plugin.AsymStopTime > 0f || CplRefAuth) && cplActive && cplLoaded && asymValid)
			{
				float dyGs = Mathf.Max(Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan)), (AsymDamageGate && dmgGateHold > 0f) ? 1f : 0f);
				// Su-30 0.5.2: the roll toward each side is the law's roll limit x (the reserve that brakes it / the clean jet's reserve at its own
				// limit, at most 1): a roll the jet with the load stops as fast as the clean jet stops the law's roll at its limit. No load, or an
				// AoA where the reserve is at least the clean limit's: the law's roll, untouched. AsymStopTime > 0 adds its fixed-time bound.
				float refS = (alpha >= 0f) ? cplRefPos : cplRefNeg;
				float limP = 999f, limN = 999f;
				if (CplRefAuth && refS > 0f)
				{
					limP = num10 * Mathf.Clamp01(Mathf.Max(0f, cplRnNow) / refS);   // a roll to + is braked by the reserve toward -
					limN = num10 * Mathf.Clamp01(Mathf.Max(0f, cplRpNow) / refS);
				}
				if (Plugin.AsymStopTime > 0f)
				{
					limP = Mathf.Min(limP, Plugin.AsymStopTime * Mathf.Max(0f, cplRnNow));
					limN = Mathf.Min(limN, Plugin.AsymStopTime * Mathf.Max(0f, cplRpNow));
				}
				limP = Mathf.Max(CplStopFloor, limP);
				limN = Mathf.Max(CplStopFloor, limN);
				if (cplStopPF <= 0f)
				{
					cplStopPF = limP;
					cplStopNF = limN;
				}
				float kSt = dt / (Mathf.Max(0f, CplStopTau) + dt);
				cplStopPF += (limP - cplStopPF) * kSt;
				cplStopNF += (limN - cplStopNF) * kSt;
				cplStopP = Mathf.Lerp(num10, Mathf.Min(num10, cplStopPF), dyGs);
				cplStopN = Mathf.Lerp(num10, Mathf.Min(num10, cplStopNF), dyGs);
				num11 = Mathf.Clamp(num11, 0f - cplStopN, cplStopP);
			}
			else
			{
				cplStopPF = (cplStopNF = 0f);
			}
			RollGovStep(dt, num10, rollSide);
			if (rollGov < 0.999f)
			{
				num10 = Mathf.Min(num10, Mathf.Max(num10 * Mathf.Lerp(rollGov, 1f, lowSpeedAxisW), RollGovMinRate));   // Su-30 0.1.11: the sideslip governor lets go in the LowSpeedTvcRoll band
				num11 = Mathf.Clamp(num11, 0f - num10, num10);
			}
			pullTradeDbg = 0f;
			if (pullTradePs < 998f && Plugin.PullRollTrade > 0f)
			{
				// start from the roll in force so the limit acts at once, not after it has slewed down from 999
				if (pullTradePs > num10)
				{
					pullTradePs = num10;
				}
				// 2.2.26: with a fin lost the trade stays at most PullRollTradeFinLoss (bench, both fins gone, full aft + full
				// roll at 243 kt: the full trade let the AoA reach the 40 deg cap and the sideslip went 16 -> 23 deg)
				float trK = (yawHealth >= 0.99f) ? Plugin.PullRollTrade : Mathf.Min(Plugin.PullRollTrade, PullRollTradeFinLoss);
				float trLim = Mathf.Min(num10, Mathf.Max(PullTradeFloor, pullTradePs));
				pullTradeDbg = trLim;
				// 2.6.4: the pitch covers what the roll's share leaves (see pullTradeAsk)
				tradeAlphaRate = -1f;
				// 2.7.4: in measured mode the pitch's share is the measured AoA-rise cap below (capacity less the roll's build); this
				// second cap waited for the roll to slew down before letting the pull through (11:30 log, 112.7 / 167.8 s: qCmd 24 -> 12
				// -> 44 on a re-pull with the roll still at 60-190 deg/s - the hitch)
				// (2.7.4 tried it off in measured mode: 13-17 deg on pull + roll reversals on the bench, AoA overshoots; it stays)
				// 2.8.0: the pitch share is what the measured trade computed from the asked roll, not the limit's slew
				tradeAlphaRate = (pullTradeHoldDbg && tradeAlphaNew > 0f) ? tradeAlphaNew : -1f;
				num10 = Mathf.Min(num10, trLim);
				num11 = Mathf.Clamp(num11, 0f - num10, num10);
			}
			else
			{
				tradeAlphaRate = -1f;
			}
			// 2.5.0 reference governor on the commanded roll rate (replaces the 2.4.17 cap and the 2.4.18 rudder-load gate).
			// A velocity-vector roll at AoA carries a body yaw rate of pS sin(AoA); when the roll stops or reverses the wing kills
			// the body roll in a few tenths of a second while the yaw rate is braked more slowly (rudders slewing across their
			// travel, then their yaw authority), and the yaw left over turns into sideslip at r cos(AoA). Fitted on the 30 Oct
			// flight logs (200 stops and reversals, 3-40 deg AoA, 150-420 kt), the sideslip excursion of a stop or reversal is
			//   |pS| sin(AoA) cos(AoA) x (StopSlipTau0 + StopSlipTau1 x max(0, (L - StopSlipKnee) / (1 - StopSlipKnee)))
			// with L the rudders' load for the roll's yaw (their yaw moment toward the roll's yaw side over what they have on
			// that side, StopSlipHoldTau filter). It is linear in the roll rate and did not depend on IAS or on the rudders'
			// authority in the logs (the 2.4.17 bench model, quadratic in the yaw rate over the rudder authority, held low-IAS
			// rolls that stop cleanly in game to its 30 deg/s floor). Stops with the rudders at their stop before the release
			// (14:35 log 267.7 / 271.2 / 273.8 s, 10:28 log 61.8 / 139.3 / 277.1 s: 12-16 deg) are the knee term; stops from
			// rolls settled with rudder margin were benign at 124-152 deg/s (14:35 log 85.0 / 152.9 s: 1.1 deg).
			// The governor caps the roll-rate command (not the trajectory's acceleration or braking) to the rate whose
			// predicted excursion is the margin schedule: RollStopSideslip up to StopSlipAoALo deg AoA, RollStopSideslip50 at 50,
			// RollStopSideslip65 at 65, off past StopSlipAoAEnd; RollStopSlipGain scales the prediction (1 = the median fit).
			// Reference-governor dynamics: the cap comes down at StopSlipFallK x aB / sin(AoA) (and rises at StopSlipRiseK times that) - the
			// roll deceleration the rudders' braking yaw aB could coordinate - so a falling cap is a roll the yaw can follow, not
			// a stop of its own (2.4.18's gate, engaging over 0.3 s mid-roll, braked the roll with the rudders pinned and made the
			// sideslip it was meant to prevent: 23:27 log, 96-98 s). While the trajectory is still accelerating into the roll the
			// cap does not come down: the rudders pin in every fast entry and unpin as the yaw catches up.
			stopSlipABDbg = 0f;
			stopSlipMarginDbg = 0f;
			// 2.5.3: no AoA gate here (was |sin AoA| > 0.05): the AoA-sweep cap below must hold through zero AoA - with the gate
			// the cap let go for 0.15 s as a push + roll crossed zero and the pitch rate stepped from -24 to -60 deg/s there
			// (bench, 300 km/h). The margin cap itself is unbounded near zero AoA through sc's floor.
			// Su-30 0.3.5 (Tom 10 Nov 22:44: "at the end, stick was to the left, the jet yawed right"): the push governor's band (pushGovL / U /
			// A, pushGovSide) and the push plan's latch (pushPlanOn) are refreshed only inside the governor block below, which stops running
			// past StopSlipAoAEnd + 5 (the limit + margin + 5 deg) - but both were applied every tick from whatever they last held. 22:34 log,
			// 251.7-276 s: a right corkscrew handed over to MPO past the limit, the band froze at side +1, L 1.5, and for 24 s (released,
			// then full left stick from 255.6 s) the governor turned the stick's -31 deg/s target into +4..+46 deg/s: the jet rolled and
			// yawed right against the stick (bench: stick -31 -> target +3.8..+16, the band identical every tick from 252.2 s). Now cleared
			// each tick; the plan's latch drops on any tick its step did not run.
			pushGovL = float.NaN;
			pushGovSide = 0f;
			pushPlanStepped = false;
			if (!isAI && LatGovMode)
			{
				float aAbs = Mathf.Abs(alpha);
				// Su-30 0.1.3: the governor's fade-out (StopSlipAoAEnd, 70, for jets limited at <= 65) sits StopSlipLimMargin past this
				// side's AoA limit when that is higher. With Tom's 75 limit the governor was off at the limit itself (margin 0, rate cap
				// 0 from 75.0 deg): 8 Nov 00:19 log, full aft + full roll at 75 deg / 130 -> 90 km/h, sideslip to 16 deg, then a yaw
				// departure at 73 deg / 72 km/h (sideslip -24 -> +33, yaw rate 69 deg/s). With 70 it began fading on any overshoot.
				float limS = (alpha >= 0f) ? Plugin.AoAPos : Mathf.Abs(Plugin.AoANeg);
				float gEnd = Mathf.Max(StopSlipAoAEnd, limS + StopSlipLimMargin);
				float gS = 1f - Mathf.Clamp01((aAbs - gEnd) / 5f);
				float sgnR = (Mathf.Abs(num11) > 2f) ? Mathf.Sign(num11) : ((Mathf.Abs(pTraj.x) > 2f) ? Mathf.Sign(pTraj.x) : 0f);
				// yaw moment sign that brakes the roll's yaw (the roll's yaw rate is pS sin(AoA))
				float yB = 0f - sgnR * Mathf.Sign(alpha);
				float nPos = 0f;
				float nNeg = 0f;
				// Revoker 0.3.0: the rudder and the yaw nozzle (yawEffs), each within its usable travel, the rudder faded with the fin
				CollectYawEffs();
				float zRudNow = 0f;
				foreach (YawEff yS in yawEffs)
				{
					YawRange(yS, out float zHi, out float zLo);
					nPos += Mathf.Max(0f, zHi) * yS.auth;
					nNeg += Mathf.Max(0f, 0f - zLo) * yS.auth;
					zRudNow += yS.e.MAt(yS.e.now).z * yS.auth;
				}
				// Revoker 0.3.1 (from Ifrit 0.6.0): the fin's own restoring yaw is part of the yaw axis - the probe's directional
				// stiffness k (slipStiff, deg/s^2 per deg, from the airframe's tables at the present flow) over an excursion to the
				// margin m adds k m / 2 (the reversal model with the weathervane in it peaks at m for dR^2 = (2 a m + k m^2) / c).
				// Nothing entered: the probe's number and the margin. ~10 % of the rudder + nozzle here.
				stiffCredit = (PaceStiffCredit > 0f && Plugin.RollStopSlipProbe) ? (PaceStiffCredit * Mathf.Max(0f, slipStiffDbg) * 0.5f * Mathf.Max(1f, stopSlipMarginDbg)) : 0f;
				yawCapDbg = 0.5f * (nPos + nNeg) * IaInv[2, 2] * 57.29578f + stiffCredit;
				dampCredit = (PaceDampCredit > 0f && Plugin.RollStopSlipProbe) ? (PaceDampCredit * Mathf.Max(0f, 0f - slipDampF) * 0.5f * Mathf.Abs(omega.z) * 57.29578f) : 0f;
				float nB = (yB > 0f) ? nPos : ((yB < 0f) ? nNeg : Mathf.Min(nPos, nNeg));
				// the yaw effectors' load for the roll's yaw: their yaw moment now, toward the roll's yaw side, over what they have there
				float sY = 0f - yB;
				float nDrive = (sY > 0f) ? nPos : nNeg;
				float uR = (sY != 0f && nDrive > 1E-06f) ? Mathf.Clamp01(sY * zRudNow / nDrive) : 0f;
				stopSlipUDbg = uR;
				// 2.5.2: the rudders' usage whichever side they are on - after a reversal they are still pinned the old way, and
				// the yaw axis is just as saturated for the new roll until they have crossed
				float nAbs = (zRudNow >= 0f) ? nPos : nNeg;
				float uAbs = (nAbs > 1E-06f) ? Mathf.Clamp01(Mathf.Abs(zRudNow) / nAbs) : 0f;
				stopSlipAbsF += (uAbs - stopSlipAbsF) * (dt / (StopSlipHoldTau + dt));
				float aB = Mathf.Max(1f, StopSlipYawUse * nB * IaInv[2, 2] * 57.29578f + stiffCredit);
				stopSlipABDbg = aB;
				if (gS > 0f)
				{
					// margin schedule
					float bLo = Plugin.RollStopSideslip;
					// 2.5.14: one margin at every AoA (RollStopSideslip); the 50 / 65 deg values are no longer read - a schedule by AoA
					// was the last one in the lateral law, and the fins' stall bound runs the other way and is not what carries the
					// lateral authority above 35 deg (14:05 analysis). The margin is a handling number; the yaw side is from the tables.
					float bM = bLo;
					if (Plugin.RollStopSlipProbe)
					{
						bM = Mathf.Max(1f, Mathf.Min(bM, SlipTolUse * slipTolF));
					}
					stopSlipMarginDbg = bM;
					float sc = Mathf.Max(0.02f, Mathf.Abs(num2 * num));
					// 2.21.0: the 2.5.3 AoA-rise cap ('roll first, then pitch': StopSlipPitchK x (capacity - the roll's own build x sin AoA)
					// / (pS cos AoA), gated by the rudders' yaw and the sideslip lead) is deleted. It spent the same yaw capacity the trade
					// had already divided (TradeCapShare, PullRollTrade), a second time and with the roll trajectory's acceleration taken
					// off it first, so a pull with the roll stick coming in 0.3 s later went 60 -> 24 -> 44 deg/s of pitch command where
					// the trade's own share read 26-38 (3 Nov 09:53 log, 39.9-40.5 s, 214 kt: rudders to the stop, sideslip 0.1-1.5 deg)
					// - the pitch hitch. The trade is the one accounting of the sweep's yaw demand on the sweep that grows |AoA|; the
					// roll's own build is a transient the sideslip margin pays for, not a steady demand (counted as one it is this cap
					// again: 141 deg/s of roll asked, 40 deg/s^2 of build at 12 deg AoA, 45 of capacity -> 10 deg/s of pitch).
					yawSatAlphaRate = -1f;
					yawSatAlphaRateDn = -1f;
					// 2.5.4: sized from the roll the jet has, not only the reference - when the window drops the reference the body roll
					// is still 70-120 deg/s for half a second and the sweep under it still needs the yaw (09:44 log, 324 / 328 s: the cap
					// let go and the push / pull ran at 50-60 deg/s of AoA rate over a 100 deg/s roll, 9-11 deg); and whenever a roll
					// is in progress, stick held or released (280 s: released mid-roll, cap off, 13.5 deg)
					float pSNow = Mathf.Abs(omega.x * num + omega.z * num2) * 57.29578f;
					float pSCap = Mathf.Max(Mathf.Abs(pTraj.x), pSNow);
					pushGovL = float.NaN;
					PushPlanStep(dt, num, num2, num11);
					pushPlanStepped = true;
					if (Plugin.RollGovernorPitch && pSCap > 5f)
					{
						// 2.34.0 (PushGov, see the static): the sweep that grows |AoA| is the trade's (either AoA sign, held stick; a released
						// stick asks nothing of it); the sweep toward zero is the governor's push side.
						float capPullG = StopSlipPitchFree;
						if (tradeAlphaRate > 0f) capPullG = Mathf.Min(capPullG, tradeAlphaRate);
						tradeAlphaDbg = (tradeAlphaRate > 0f) ? tradeAlphaRate : 0f;
						float sG = Mathf.Abs(num2), cG = Mathf.Abs(num);
						float pSSignedG = omega.x * num + omega.z * num2;
						float sideG = (Mathf.Abs(pTraj.x) > 2f) ? Mathf.Sign(pTraj.x) : Mathf.Sign(pSSignedG);
						pushGovSide = sideG;
						// the yaw that coordinates the rotation is pS sin a: its sign is the side the yaw sheds FROM
						float rSideG = sideG * Mathf.Sign(num2);
						// 2.34.1 (PushGovNoStab): the yaw the push will have is the yaw WITHOUT the stabilators - the push itself takes them for pitch
						// (21:52 log, 149.0 / 189.1 s: stabPitchFrac 0 -> 0.9 by 53 deg, the full read said 30-37 deg/s^2 of shed, 16-20 was delivered
						// at 50-30 deg; the no-stab read said 14-28). With the fins gone the stabs ARE the yaw (2.22.0's blend): by the fin health.
						float capYFull = (rSideG > 0f) ? capEst.capZeN : capEst.capZeP;
						float capYNs = (rSideG > 0f) ? capEst.capZeNsN : capEst.capZeNsP;
						float wNsG = PushGovNsByPitchCmd ? Mathf.Max(1f - stickRelW, Mathf.Clamp01((0f - Mathf.Sign(alpha)) * qCmd / Mathf.Max(1f, PushGovNsQ))) : 1f;
						float capYRaw = PushGovNoStab ? Mathf.Lerp(capYFull, Mathf.Min(capYFull, capYNs), Mathf.Clamp01(yawHealth / 0.5f) * wNsG) : capYFull;
						// the body roll pS cos a grows on the rotation's side
						// (the full-travel read: the discounted one follows the roll loop's own last command - braking, it read 30; growing, 94)
						float capXRaw = (sideG > 0f) ? capEst.capXPraw : capEst.capXNraw;
						float kG = dt / (Mathf.Max(0f, PushGovCapTau) + dt);
						if (!pushGovXInit) { pushGovCapXF = capXRaw; pushGovCapYF = capYRaw; pushGovXInit = true; }
						pushGovCapXF += (capXRaw - pushGovCapXF) * kG;
						pushGovCapYF += (capYRaw - pushGovCapYF) * kG;
						float capYG = Mathf.Max(0f, TradeCapShare * pushGovCapYF);
						float capXG = Mathf.Max(0f, TradeCapShare * pushGovCapXF);
						float aCmdG = Mathf.Max(0f, (0f - Mathf.Sign(alpha)) * (qCmd - omegaV));
						bool actG = aCmdG > 2f && !isAI;
						// the body roll when the sweep began (frozen while it runs)
						if (!actG) pushGovP0 = Mathf.Abs(omega.x) * 57.29578f;
						pushGovActive = actG;
						// how far the velocity roll may grow: held stick, the stick's own target (never faster than the pilot asks); released,
						// PushGovRollK x the body roll it had when the sweep began (Tom: "the roll stops, a little slower is fine" - never faster)
						float growRawG = Mathf.Max(5f, Mathf.Lerp(Mathf.Abs(num11), PushGovRollK * Mathf.Max(10f, pushGovP0) / Mathf.Max(0.2f, cG), stickRelW));
						if (!actG || pushGovGrowF <= 0f) pushGovGrowF = growRawG;
						pushGovGrowF += (growRawG - pushGovGrowF) * kG;   // (the same 0.2 s: the stick's target steps as the law's limits move)
						float growCapG = pushGovGrowF;
						float pCeilG = growCapG * cG;   // as a body roll
						pushGovCeil = growCapG; pushGovP0Dbg = pushGovP0;
						// the roll's growth is credited only while the velocity roll is under that cap
						float capXAvail = capXG * Mathf.Clamp01((growCapG - Mathf.Abs(pTraj.x)) / Mathf.Max(1f, 0.25f * growCapG));
						// (2) over the sweep: coordinated with the body roll at its ceiling the yaw is pCeil tan a, so while the yaw is r (less what
						// PushoverSideslip absorbs at the bottom) the AoA may come no lower than a_min = atan(r_ex / pCeil); a_min falls as the yaw
						// sheds (at capY: d a_min / dt = capY pCeil / (pCeil^2 + r_ex^2)). The pace closes on a_min over PushGovTau.
						float rOnG = Mathf.Max(0f, rSideG * omega.z * 57.29578f);
						float rLeftG = Mathf.Sqrt(2f * capYG * Mathf.Max(0f, PushGovSlipK * Mathf.Min(PushSlipNow(), PushGovSlipMax)));
						float rExG = Mathf.Max(0f, rOnG - rLeftG);
						float aMinG = Mathf.Atan(rExG / Mathf.Max(5f, pCeilG)) * 57.29578f;
						float gapG = Mathf.Max(0f, Mathf.Abs(alpha) - aMinG);
						float capTermG = ((PushGovDecel > 0f) ? Mathf.Sqrt(2f * PushGovDecel * gapG) : (gapG / Mathf.Max(0.05f, PushGovTau))) + 57.29578f * capYG * pCeilG / Mathf.Max(1f, pCeilG * pCeilG + rExG * rExG);
						pushGovAMin = aMinG;
						// (3) the shed time: the yaw over the allowance has to be gone when the AoA is at zero - |AoA-rate| <= |a| capY / r_ex. When
						// the yaw sheds as planned, a and r_ex fall together and this is ONE constant rate through the push (rising at the bottom as
						// r_ex reaches the allowance); r_ex weighted by (1 - cos a) against SweepShedCos (2.6.6: near zero a hold buys nothing)
						float wShG = Mathf.Clamp01((1f - cG) / Mathf.Max(0.05f, SweepShedCos));
						float rExW = rExG * wShG;
						float capShedG = (rExW < 0.5f) ? StopSlipPitchFree : Mathf.Abs(alpha) * capYG / rExW;
						// The roll's ACCELERATION is never what binds (100-500 deg/s^2 against the yaw's 20-50): what binds is how far it may grow, which
						// is (2). (1), the one-tick two-axis rate (capY cos a + capX sin a) / pS, was a bench switch (PushGovInst, off; deleted 2.44.1).
						float capPaceG = Mathf.Min(capShedG, capTermG);
						float capDnG = Mathf.Max(SweepFloor, capPaceG);
						// 2.19.0 nose-hold without fins (unchanged)
						sweepHoldDbg = 0f;
						float finKcG = Mathf.Clamp01(yawHealth / 0.5f);
						if (SweepHoldFade && finKcG < 1f)
						{
							float rotNowG = Mathf.Abs(pSSignedG) * 57.29578f;
							float wRG = Mathf.Clamp01((rotNowG - SweepHoldRate) / Mathf.Max(1f, SweepHoldRateBlend));
							float wSG = Mathf.Clamp01((SweepHoldStabFrac - stabPitchFrac) / Mathf.Max(0.05f, SweepHoldStabFrac));
							float wHG = (1f - finKcG) * wRG * wSG;
							capDnG = Mathf.Lerp(capDnG, 0f, wHG);
							sweepHoldDbg = wHG;
						}
						// the sweep that grows |AoA| from negative AoA (a push through zero under a held roll): the trade, and the yaw the rotation
						// needs on the far side may build no faster than its capacity there allows, SweepYawK x capY(grow) cos a / pS (the 2.1.8
						// rule for that sweep, unchanged in kind; the roll's share is the trade's)
						if (alpha < 0f)
						{
							float capYGrow = (rSideG > 0f) ? capEst.capTrZeP : capEst.capTrZeN;
							capPullG = Mathf.Min(capPullG, Mathf.Max(SweepFloor, 57.29578f * SweepYawK * Mathf.Max(0f, capYGrow) * cG / Mathf.Max(5f, pSCap)));
						}
						yawSatAlphaRate = capPullG;
						yawSatAlphaRateDn = Mathf.Min(StopSlipPitchFree, (PushPlan && pushPlanOn) ? StopSlipPitchFree : capDnG);
						// lateral: with the commanded AoA rate toward zero (this tick's pitch command less the flight path's rotation) the
						// velocity roll may come down no faster than L, and must grow when L > 0 (applied to the roll trajectory below)
						pushGovA = aCmdG;
						if (actG)
						{
							pushGovL = (Mathf.Abs(pTraj.x) * cG * aCmdG * (MathF.PI / 180f) - Mathf.Max(0f, pushGovCapYF)) / Mathf.Max(0.05f, sG);
							// the roll side: the body roll pS cos a grows by pS sin a x AoA-rate at constant pS - the velocity roll may grow by no
							// more than (capX available - pS sin a AoA-rate) / cos a (negative at the ceiling: it must come down)
							pushGovU = Mathf.Lerp(1E+06f, (capXAvail - Mathf.Abs(pTraj.x) * sG * aCmdG * (MathF.PI / 180f)) / Mathf.Max(0.2f, cG), stickRelW);
						}
						capSweepDbg = Mathf.Min(StopSlipPitchFree, capDnG);
						capShedDbg = float.IsNaN(pushGovL) ? 0f : Mathf.Clamp(pushGovL, -999f, 999f);
						aYawShedDbg = capYG;
						sweepDelF = capXAvail;   // (telemetry 'capSm' under PushGov: the roll growth credited)
					}
					else
					{
						tradeAlphaDbg = 0f;
						sweepDelF = 0f;
						capSweepDbg = 0f;
						capShedDbg = 0f;
						aYawShedDbg = 0f;
						pushGovXInit = false;
						pushGovActive = false;
					}
					num11 = Mathf.Clamp(num11, 0f - num10, num10);
				}
			}
			pSmaxDbg = num10;
			float num12 = (0f - (1f - num9)) * sy * Plugin.PedalBeta;
			// Revoker 0.2.7 (FinLossPedalSideslip): with the fin gone the pedal asks for at most this much sideslip. 0.2.6's
			// yaw clamp counted the dead rudder, which held the pedal to ~1 deg; with the yaw tiers fixed (RudderTierLiveOnly)
			// the full 15 deg went through, and the finless airframe is unstable in yaw: bench, fins gone, AB, 1 s of full
			// pedal at 1100 km/h: sideslip 13 deg with the nozzle saturated against it, then the jet swapped ends.
			if (FinLossPedalSideslip > 0f && yawHealth < 0.999f)
			{
				float pedLim = Mathf.Lerp(Plugin.PedalBeta, Mathf.Min(Plugin.PedalBeta, FinLossPedalSideslip * Mathf.Min(1f, FinLossPedalIas * FinLossPedalIas / Mathf.Max(1f, 12.96f * ias * ias))), Mathf.Clamp01((1f - yawHealth) / 0.5f));
				num12 = Mathf.Clamp(num12, 0f - pedLim, pedLim);
			}
			betaPedalCmd = num12;
			// 2.1.0 sideslip-rate lead: the guard also looks at where the sideslip is heading, from the measured body rates
			// (kinematic sideslip rate p sin(AoA) - r cos(AoA) plus gravity's g cos(theta) sin(phi) / V). A stability-axis roll
			// needs a yaw rate of p sin(AoA); when the yaw axis cannot build it (low IAS, an AoA sweep under a roll, fins near
			// stall, the roll's own adverse yaw) the sideslip rate shows it at once, while the sideslip itself builds a second
			// later (10 Oct log: -7..-10 deg/s of sideslip rate at 2..5 deg of sideslip, departing to 30..50 deg).
			float gTermB = ((tas > 40f) ? (9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - 40f) / 20f)) : 0f);
			float bdk = (omega.x * num2 - omega.z * num) * 57.29578f + gTermB;
			bdk = bdkF.Step(bdk, SlipRateTau, dt);
			slipRateDbg = bdk;
			float bErrNow = beta - num12;
			// Revoker 0.2.14/0.2.15: the lag the lateral loops answer within, for the sideslip-hold gain (SlipGainFromLoops) and
			// the governor's look-ahead below. Yaw path: 1/KYaw + the actuator lag; roll path: 1/KRoll + the actuator lag; a path
			// that would have to deliver the hold's share beyond the spare authority of its axis counts the time that share takes
			// at the spare authority instead (capped at 1 s); cos^2 / sin^2 weighted by the kinematic split of the correction.
			{
				float lagAct = Plugin.ActuatorLagT;
				latLagYawNom = 1f / Mathf.Max(0.1f, Plugin.KYaw) + lagAct;
				float lagYaw = latLagYawNom;
				float lagRoll = 1f / Mathf.Max(0.1f, Plugin.KRoll) + lagAct;
				float rSprev = rSF.init ? Mathf.Abs(rSF.y) : 0f;
				float accTraj = Mathf.Abs(pTraj.v);
				float spareYaw = Mathf.Max(5f, aYaw - accTraj * Mathf.Abs(num2));
				float spareRoll = Mathf.Max(5f, Mathf.Min(aRollGo, aRollStop) - accTraj * Mathf.Abs(num));
				lagYaw = Mathf.Min(1f, Mathf.Max(lagYaw, rSprev * Mathf.Abs(num) / spareYaw));
				lagRoll = Mathf.Min(1f, Mathf.Max(lagRoll, rSprev * Mathf.Abs(num2) / spareRoll));
				float c2 = num * num;
				float s2 = num2 * num2;
				latLagEff = (c2 * lagYaw + s2 * lagRoll) / Mathf.Max(0.01f, c2 + s2);
			}
			// Revoker 0.2.15 (SlipGovLeadFromLoops): the governor looks ahead as far as the loops take to answer (latLagEff), not a
			// fixed 0.8 s. 30 Oct 23:45 log (0.2.14), push + roll at -42 deg AoA: the sideslip crossing zero at +11 deg/s on its
			// way back from -3.3 (the hold's own recovery, which peaked at +2.4) read as +9 deg 0.8 s out and cut the roll target
			// 79 -> 46 deg/s; the roll braked at -95 deg/s^2, the yaw command fell 7 deg/s with the yaw nozzle and rudder on their
			// stops, the yaw rate ran 6 deg/s past it and the nozzle went stop to stop; 0.35 s later the release re-built the
			// roll 46 -> 85 and the yaw ran the same way again. Over the yaw loop's own 0.33-0.47 s that crossing projects to
			// 3.6 deg: no cut. The first cut (sideslip -3.3 with -10 deg/s of rate, the yaw axis behind the sweep) goes from
			// 0.38 to ~0.85 of the roll: the hold is left to take out what the F-22E takes out there (-6.8 deg in one move).
			float leadT = (Plugin.SlipGainFromLoops && Plugin.SlipGovLeadFromLoops) ? Mathf.Min(SlipLeadT, latLagEff) : SlipLeadT;
			slipLeadDbg = leadT;
			float bErrLead = bErrNow + bdk * leadT;
			float num14 = Mathf.Clamp01((Mathf.Abs(alpha) - 8f) / 10f);
			// Ifrit 0.3.8: one low-TAS / reversed-flow factor for everything that works off the sideslip or coordinates the
			// roll with yaw (0.3.7's hold fade, moved up): 1 at speed, 0 below SlipHoldMinTAS or past 120 deg |AoA|.
			float latRevK = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - 100f) / 20f);
			latRevK = latRevK * latRevK * (3f - 2f * latRevK);
			float latHoldK = Mathf.Clamp01((tas - Plugin.SlipHoldTasMin) / Mathf.Max(1f, Plugin.SlipHoldTasSpan)) * latRevK;
			// Ifrit 0.3.9: the roll limiters (sideslip guard / governor, yaw anchor, roll-acceleration coordination cap) use
			// their own, wider band (Plugin.SlipLimTasMin -> +SlipLimTasSpan, smoothstep): 20 Oct 23:59 log, 0.3.8, full
			// stick at 13-48 m/s: roll built to 29 deg/s, then as TAS passed 18-35 m/s the guard came back with 15-16 deg of
			// sideslip the 5-10 deg/s^2 yaw axis could not take out yet (target 33 -> 15) and the yaw anchor held the roll
			// on its 10 deg/s floor from 30 m/s: ~11 deg/s for 3+ s with full stick up to 48 m/s.
			float latLimK = Mathf.Clamp01((tas - Plugin.SlipLimTasMin) / Mathf.Max(1f, Plugin.SlipLimTasSpan));
			latLimK = latLimK * latLimK * (3f - 2f * latLimK) * latRevK;
			// 2.11.0: the 2.0.15 sideslip guard (SideslipGuardStart: the stick's roll scaled down past 4 deg of sideslip error) and the
			// yaw-saturation guard (YawSaturationGuard x yawSat) are deleted. 2 Nov, 2.9.x-2.10.0 logs: the sideslip term read 1.0 in
			// 93-96 % of roll-asked ticks (Tom flew it at 40 = off from 16:37 with the 22-34 deg AoA rolls unchanged: |pS| 70 -> 71,
			// sideslip p90 5.7 -> 6.4 deg), and the yaw-saturation term only ever cut (x 0.66-0.78) where the pace window was already
			// holding the target (paceHold > 2 in 67-100 % of those ticks: reversals and stops with the roll near zero). The window is
			// the saturation guard; the asymmetric-load sideslip lead (slipGov) below still scales x.
			float x = 1f;
			// the asymmetric-load sideslip lead: backs the
			// velocity-vector roll off only while the predicted sideslip error is large AND still growing, so a sideslip that is
			// already recovering never holds the roll back
			slipGovDbg = 1f;
			// 2.1.1: neither the sideslip lead nor the yaw anchor acts in a roll reversal (pilot target against the running
			// roll): the yaw rate the old roll left behind sat on the wrong side of the anchor window and the reversal's own
			// transient sideslip read as a departure, so the new roll was held near zero for ~0.7 s and then released at once
			// (10 Oct 22:50 log, 196.9 / 293.9 / 748.8 s). Reversals use the 2.0.15 braking / reversal limits again. (No AoA
			// fade: in a push + roll the AoA sweeps through any fade band exactly while the roll builds.)
			// latched from the reversal until the new roll is half-way to its target (as RevCarry does), so the sideslip the
			// reversal leaves behind does not hold the new roll back right after the zero crossing
			float tgtPre = num11;
			if (tgtPre * pTraj.x < 0f && Mathf.Abs(pTraj.x) > 2f && Mathf.Abs(tgtPre) > 5f)
			{
				latRevSign = Mathf.Sign(tgtPre);
			}
			else if (latRevSign != 0f && (tgtPre * latRevSign < 5f || pTraj.x * latRevSign >= 0.5f * Mathf.Abs(tgtPre)))
			{
				latRevSign = 0f;
			}
			bool latRev = latRevSign != 0f;
			latRevDbg = latRev;
			// 2.1.2: negative AoA only. Both limiters were for the push + lateral departure; at positive AoA the lateral law
			// is 2.0.15's again (2.1.0 / 2.1.1 changed the positive-AoA roll feel). Full at AoA <= NegGateA0 - NegGateSpan,
			// off at AoA >= NegGateA0.
			// 2.6.0: the 2.1.0-2.1.2 push + roll limiters (sideslip-rate lead, yaw anchor) act only for an asymmetric load past the
			// rudder fade (the damage case, below); the clean jet's push + roll is the governor's
			float negGate = 0f;
			// 2.1.4: at positive AoA with an asymmetric load past the rudder fade the same two limiters act: the flaperons are
			// spent trimming the load's roll, the stabs are the only yaw source and their yaw carries roll that nothing can
			// cancel, so the roll outruns the yaw it needs (12 Oct 18:29 log: sideslip 10-24 deg at ~50 deg AoA). Clean jets
			// (lateral CG offset under CoupHeldDy0) keep the 2.0.15 law.
			if (AsymSharedLat && Plugin.AsymSharedLatOn && alpha > 0f && asymValid)
			{
				float asymG = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan)) * (1f - rudderAuth);
				negGate = Mathf.Max(negGate, asymG);
			}
			negGateDbg = negGate;
			// 0.5.3: the push + roll limiters (sideslip lead, yaw anchor) also act gear-down in the air - 2.1.0 left them off with
			// the gear down, and a full roll then full push at idle, gear down, 165 kt went to -34 deg AoA and -55 deg sideslip
			// (27 Oct 12:12 log, 136-138 s). On the runway (any wheel touching) the gear-down law is unchanged.
			if (SlipLeadT > 0f && !isAI && !latRev && negGate > 0f && Mode != FcsMode.Ground && (Mode != FcsMode.GearDownRate || (Plugin.GearDownSlipProtect && !gsAnyContact)))
			{
				float growing = (bdk * bErrNow > 0f || Mathf.Abs(bErrNow) < 1f) ? 1f : 0f;
				float sg = Mathf.Clamp01(1f - (Mathf.Abs(bErrLead) - SlipGovStart) / Mathf.Max(1f, SlipGovSpan));
				sg = Mathf.Lerp(1f, sg, growing * negGate);
				slipGovDbg = sg;
				x = Mathf.Min(x, sg);
			}
			// the fast cut only when the sideslip lead is what cuts (the 2.0.15 guard terms keep their 0.25 s)
			// Ifrit 0.3.8: the sideslip guard / governor fade out with latHoldK. 20 Oct 23:42 log, MPO off, 5-18 deg AoA, 11-16
			// m/s, full right stick (338.6-339.5 s): the sideslip nothing could coordinate there built to -17 deg and the guard
			// cut the roll target 19 -> 13 -> 5 -> 2 deg/s (guard 0.7 -> 0.0): roll stick dead.
			x = Mathf.Lerp(1f, x, latLimK);
			x = guardF.Step(x, (guardF.init && x < guardF.y && slipGovDbg < 0.999f) ? GuardCutTau : 0.25f, dt);
			num11 *= x;
			// 2.1.0 yaw-anchored roll: the velocity-vector roll needs a body yaw rate of pS sin(AoA). It may only ask for a yaw
			// rate the yaw axis can reach from the yaw rate it has now within YawAnchorT (at YawAnchorK of the modelled yaw
			// authority). In a coordinated roll the yaw rate already matches and this never binds; when the AoA sweeps under a
			// held roll (full push + full lateral) or the yaw axis stalls, the roll shrinks with what the yaw can follow instead
			// of turning the AoA into sideslip.
			yawAnchorDbg = 999f;
			if (YawAnchorK > 0f && !isAI && !latRev && negGate > 0f && Mode != FcsMode.Ground && (Mode != FcsMode.GearDownRate || (Plugin.GearDownSlipProtect && !gsAnyContact)) && Mathf.Abs(num2) > 0.05f)
			{
				// 2.2.10: the yaw rate the turn itself needs (gravity's g cos(theta) sin(phi) / V, the same term the
				// sideslip-rate lead uses) is taken out first. Without it, banked past ~60 deg at low IAS the turn's own yaw
				// rate (+4.5 deg/s at 55 m/s) looked like yaw going the wrong way for a right roll at -5 deg AoA, and the
				// roll was held at the 10 deg/s floor for 5 s with full stick (19 Oct 10:00 log, 205.9-210.7 s).
				// Only near zero AoA (full above -10, gone below -20 deg), where the turn's yaw is large next to pS sin(AoA), and
				// only with the AoA steady (gone above 15 deg/s of AoA rate): in pushes with roll it let the sideslip grow
				// (bench, -50 deg at 200 km/h: 3.2 -> 7.5 deg).
				float rNow = omega.z * 57.29578f - (YawAnchorGrav ? (gTermB / Mathf.Max(0.3f, num) * Mathf.Clamp01((alpha + 20f) / 10f) * Mathf.Clamp01(1f - (Mathf.Abs(alphaDot) - 5f) / 10f)) : 0f);
				float reach = YawAnchorK * aYaw * YawAnchorT;
				float lo = (rNow - reach) / num2;
				float hi = (rNow + reach) / num2;
				if (lo > hi)
				{
					float tmp = lo;
					lo = hi;
					hi = tmp;
				}
				// keep zero inside the window (stopping the roll is always allowed) and never below the floor rate
				lo = Mathf.Min(lo, 0f - YawAnchorFloor);
				hi = Mathf.Max(hi, YawAnchorFloor);
				yawAnchorDbg = (num11 >= 0f) ? hi : lo;
				num11 = Mathf.Lerp(num11, Mathf.Clamp(num11, lo, hi), negGate * latLimK);
			}
			// 2.5.3 yaw-reach (pace) governor (RollGovernorReplaces >= 2): one rule for the pace of every lateral move - the
			// pair's body-yaw demand, pS sin(AoA) plus the sideslip correction's rS cos(AoA), may not lead the measured yaw
			// rate by more than the yaw axis can close within PaceHorizon. The reach is what the jet's yaw is doing now
			// (filtered yaw acceleration x horizon, so a yaw that is already braking opens the window ahead of itself) plus what
			// the rudders can add within the horizon from where they are (their moment tables, slewed from the present
			// deflection, at PaceRudderUse), plus PaceFloor for the loop's normal tracking error. The roll-rate target is held
			// inside that window and the trajectory runs toward it at its tuned acceleration / braking, so where the yaw keeps
			// up nothing changes and where it does not the roll reference moves at the yaw's own pace. In a coordinated roll
			// the yaw already matches and the window never binds; in a stop or reversal the roll reference comes down as the
			// yaw actually brakes instead of at the wing's rate (31 Oct 01:40 log, level 3: 720 deg/s^2 brakes at 22 deg AoA
			// asked the yaw for 270 deg/s^2 against 85-100 from pinned rudders - 10.5 / 14.2 deg at 195.4 / 198.8 s); with
			// the stick released at 65 deg AoA it does CoordinatedStop's job (214 s: a 62 deg/s rotation told to stop in
			// 0.3 s, 15.5 deg for 5 s); at negative AoA it is the 2.1.0 yaw anchor with the reach from the tables and the
			// measured yaw instead of a constant. It never asks for a roll against the one in progress: when the yaw the jet
			// has only fits a roll the other way (the AoA swept through zero under a roll), the target is zero - stop, not
			// reverse. Replaces the trajectory's yaw-coordination caps (RollEntryCoord, BrakeYawCap, RevYawCap, CoordJerk,
			// CoordinatedStop) at level 3 and the push + roll limiters at level 2.
			// Su-30 0.2.0 (StopIgnoresSlipOutsideMargin, Tom 9 Nov 21:28: "if stopping requires sideslip outside of the margin, then just
			// don't care about sideslip, stop the thing as fast as possible"). The stop window below holds the roll at the rotation the yaw
			// can carry so the sideslip stays inside RollStopSideslip; once the sideslip is past it the hold no longer protects anything and
			// only slows the stop. 21:04 log, 175 s: corkscrew released at 77 deg with 23 deg of sideslip and the yaw saturated, the roll
			// target held at -42 deg/s (stick at zero) for 0.8 s, the velocity roll 55 -> 40 deg/s in 2.5 s. Here: lateral stick and
			// pedals released and the sideslip (less the pedal's own) past the margin -> the window and the sideslip hold stand aside and
			// both rates brake at full, until the rotation is down to StopHardRelease deg/s or the stick moves.
			{
				float bMh = Mathf.Max(1f, (stopSlipMarginDbg > 0f) ? stopSlipMarginDbg : Plugin.RollStopSideslip);
				float bOwnH = Mathf.Abs(beta - betaPedalCmd);
				bool relH = stickRelW > 0.9f && Mathf.Abs(sy) < 0.05f;
				float pSH = Mathf.Abs(pTraj.x);
				float rotH = Mathf.Max(pSH, Mathf.Abs(omega.z) * 57.29578f);
				stopHardPredDbg = bOwnH - bMh;
				if (Plugin.StopIgnoresSlip && !isAI && LatGovMode && tas > 25f && relH)
				{
					if (!stopHardOn && bOwnH > bMh && rotH > StopHardRelease * 2f)
					{
						stopHardOn = true;
					}
					else if (stopHardOn && rotH < StopHardRelease)
					{
						stopHardOn = false;
					}
				}
				else
				{
					stopHardOn = false;
				}
				stopHardW = Mathf.MoveTowards(stopHardW, stopHardOn ? 1f : 0f, dt / 0.1f);
			}
			paceLeadDbg = 0f;
			paceHoldDbg = 0f;
			rateBlendEffDbg = 0f;
			paceAccelCap = -1f;
			paceTgtRaw = num11;
			paceGateZDbg = 0f;
			paceGateXDbg = 0f;
			// 2.35.3: with the lateral stick released in a planned push-over the plan's bleed owns the velocity roll (target 0 at the bleed
			// rate, set below); the stop window's hold and pacing were computed and then overwritten every tick, so they stand down
			// Revoker 0.7.0 (PushPlanStopsRoll, Tom 5 Nov 01:10: "slow down the pitch and stop the roll during that pitchdown / AoA transient"): under a
			// planned push the velocity roll bleeds to zero over the plan's time with the lateral stick held too, not only released. The yaw that
			// coordinates a held roll is what the push has to shed before zero AoA (r = pS sin AoA); held, the roll kept making it.
			// 00:59 log, 73.1 s: a -60 deg corkscrew with the stick moved to a positive one, roll held at 70 deg/s, 63 deg/s of yaw on
			// the yaw shedding at 16 deg/s^2 - 2.2 s from -40 deg to zero and 13 deg of sideslip; 00:23 log 232 / 257 s, 17.6 / 24.6 deg.
			// The roll comes back to the stick when the plan ends (below 10 deg AoA).
			if (!pushPlanStepped)
			{
				pushPlanOn = false;
			}
			bool pushBleed = PushPlan && pushPlanOn && (stickRelW > 0.5f || PushPlanStopsRoll) && Mathf.Abs(pTraj.x) > 0.5f;
			bool pushHeld = PushPlanHeld && PushPlan && pushPlanOn && !pushBleed && Mathf.Abs(pTraj.x) > 0.5f;
			bool winDown = pushBleed && PushPlanWinOff;
			if (winDown)
			{
				paceStopNF = 0f;   // (the no-fin stop; inert here: the plan needs both fins)
			}
			if (!winDown && !isAI && LatGovMode && tas > PaceTasMin && Mathf.Max(Mathf.Abs(num2), Mathf.Abs(Mathf.Sin((alpha + alphaDot * PaceHorizon) * (MathF.PI / 180f)))) > PaceSinMin)
			{
				float rM = omega.z * 57.29578f;
				float rDotRaw = paceInit ? ((omega.z - paceOmegaZPrev) / Mathf.Max(dt, 1E-04f) * 57.29578f) : 0f;
				float rDot = paceRDotF.Step(rDotRaw, PaceAccelTau, dt);
				paceRDotLast = rDot;
				float gPos = 0f;
				float gNeg = 0f;
				float nAvPos = 0f;
				float nAvNeg = 0f;
				float iz = IaInv[2, 2] * 57.29578f;
				// Revoker 0.3.0: the rudder and the yaw nozzle (yawEffs), each from where it is at its own slew, faded by its auth
				CollectYawEffs();
				foreach (YawEff yP in yawEffs)
				{
					Effector seP = yP.e;
					float zNow = seP.MAt(seP.now).z * yP.auth;
					float zAtMax = seP.MAt(yP.hi).z * yP.auth;
					float zAtMin = seP.MAt(yP.lo).z * yP.auth;
					float rate = Mathf.Max(1f, seP.rate);
					// yaw the effector can still add toward each side from where it is, no more than one side's worth
					float nSide = 0.5f * Mathf.Abs(zAtMax - zAtMin);
					nAvPos += Mathf.Min(nSide, Mathf.Max(0f, Mathf.Max(zAtMax, zAtMin) - zNow));
					nAvNeg += Mathf.Min(nSide, Mathf.Max(0f, zNow - Mathf.Min(zAtMax, zAtMin)));
					for (int dirP = -1; dirP <= 1; dirP += 2)
					{
						float tgt = ((zAtMax >= zAtMin) == (dirP > 0)) ? yP.hi : yP.lo;
						float dd = Mathf.Abs(tgt - seP.now);
						float ts = Mathf.Min(PaceHorizon, dd / rate);
						float dh = seP.now + Mathf.Sign(tgt - seP.now) * rate * ts;
						float aS = Mathf.Max(0f, dirP * (seP.MAt(dh).z * yP.auth - zNow)) * iz;
						// the yaw acceleration ramps with the slew to ts, then holds to the horizon
						float g = 0.5f * aS * ts + aS * (PaceHorizon - ts);
						if (dirP > 0)
						{
							gPos += g;
						}
						else
						{
							gNeg += g;
						}
					}
				}
				// Revoker 0.4.1: the reach over every allocator effector (PaceReachAll)
				if (PaceReachAll)
				{
					ReachAll(out float gPA, out float gNA, out float nPA, out float nNA);
					gPos = gPA;
					gNeg = gNA;
					nAvPos = nPA / Mathf.Max(1E-06f, iz);
					nAvNeg = nNA / Mathf.Max(1E-06f, iz);
				}
				float reachPos = Mathf.Max(0f, rDot * PaceHorizon + PaceRudderUse * gPos) + PaceFloor;
				float reachNeg = Mathf.Max(0f, 0f - rDot * PaceHorizon + PaceRudderUse * gNeg) + PaceFloor;
				// 2.6.11 (PaceMarginReach): the sideslip margin still unused is reach too. A yaw-rate lead L that the yaw closes at a
				// builds L^2 cos(AoA) / (2 a) of sideslip, so the lead whose sideslip stays inside margin - |beta| is
				// sqrt(2 a (margin - |beta|) / cos AoA), a = PaceYawUse of what the rudders can add toward that side (gPos / gNeg
				// over the horizon, as a rate) - the same yaw the window already credits. Measured sideslip, so the window holds
				// only as the margin is actually being used (02 Nov 00:54 log, 340 kt corkscrew entry: roll held 70 -> 17 deg/s at
				// 45-55 deg AoA with the sideslip at 0.8-2.2 of a 4 deg margin).
				paceMarginLeadDbg = 0f;
				if (PaceMarginReach)
				{
					float mLeft = Mathf.Max(0f, Mathf.Max(1f, stopSlipMarginDbg) - Mathf.Abs(beta - betaPedalCmd));
					float cAm = Mathf.Max(0.2f, Mathf.Abs(num));
					float aP = PaceYawUse * Mathf.Max(1f, gPos / Mathf.Max(0.02f, PaceHorizon));
					float aN = PaceYawUse * Mathf.Max(1f, gNeg / Mathf.Max(0.02f, PaceHorizon));
					float lP = PaceMarginK * Mathf.Sqrt(2f * aP * mLeft / cAm);
					float lN = PaceMarginK * Mathf.Sqrt(2f * aN * mLeft / cAm);
					paceMarginLeadDbg = Mathf.Max(lP, lN);
					reachPos += lP;
					reachNeg += lN;
				}
				// 2.7.0 (PaceMeasured): the reach from what the axes are doing and what the allocator has already committed
				float pMm = omega.x * 57.29578f;
				float pDotRawM = paceInit ? ((omega.x - paceOmegaXPrev) / Mathf.Max(dt, 1E-04f) * 57.29578f) : 0f;
				float pDotM = pacePDotF.Step(pDotRawM, PaceAccelTau, dt);
				float dzH = 0f;
				float dzAll = 0f;
				float dxH = 0f;
				float dxAll = 0f;
				float gateZ = 0f;
				float gateX = 0f;
				float rHm = rM;
				float pHm = pMm;
				float ix = IaInv[0, 0] * 57.29578f;
				float hdZP = 0f;
				float hdZN = 0f;
				float hdXP = 0f;
				float hdXN = 0f;
				paceNEffDbg = 0f;
				paceDdDbg = 0f;
				foreach (Effector seM in effs)
				{
					if (seM == null || !seM.active || seM.M == null || seM.N < 2)
					{
						continue;
					}
					// the increment in flight: from where the effector is to where the allocator sent it, and how much of
					// that its servo covers within the horizon
					float dd = seM.cmd - seM.now;
					paceNEffDbg += 1f;
					paceDdDbg += Mathf.Abs(dd);
					float rateM = Mathf.Max(1f, seM.rate);
					float ts = Mathf.Min(PaceHorizon, Mathf.Abs(dd) / rateM);
					float dh = seM.now + Mathf.Sign(dd) * rateM * ts;
					Vector3 mNow = seM.MAt(seM.now);
					Vector3 mH = seM.MAt(dh);
					Vector3 mT = seM.MAt(seM.cmd);
					dzH += (mH.z - mNow.z) * iz;
					dzAll += (mT.z - mNow.z) * iz;
					dxH += (mH.x - mNow.x) * ix;
					dxAll += (mT.x - mNow.x) * ix;
					// 2.7.1: yaw and roll still available toward each side from the commanded deflections (the open capacity)
					Vector3 mMax = seM.MAt(seM.max);
					Vector3 mMin = seM.MAt(seM.min);
					hdZP += Mathf.Max(0f, Mathf.Max(mMax.z, mMin.z) - mT.z) * iz;
					hdZN += Mathf.Max(0f, mT.z - Mathf.Min(mMax.z, mMin.z)) * iz;
					hdXP += Mathf.Max(0f, Mathf.Max(mMax.x, mMin.x) - mT.x) * ix;
					hdXN += Mathf.Max(0f, mT.x - Mathf.Min(mMax.x, mMin.x)) * ix;
				}
				// the axis's demonstrated acceleration (a decaying peak), credited where nothing is committed yet
				aYawSeen = Mathf.Max(Mathf.Abs(rDot), Mathf.Max(PaceFloor / PaceHorizon, aYawSeen - aYawSeen * dt / PaceSeenTau));
				aRollSeen = Mathf.Max(Mathf.Abs(pDotM), Mathf.Max(PaceFloor / PaceHorizon, aRollSeen - aRollSeen * dt / PaceSeenTau));
				rHm = rM + (rDot + 0.5f * dzH) * PaceHorizon;
				pHm = pMm + (pDotM + 0.5f * dxH) * PaceHorizon;
				// gravity's share of the sideslip: the jet falling sideways under the sideslip loop's own correction (its time
				// constant 1 / KBeta). Tom: the one acceptable failure - excluded from the margin, never paid for with roll.
				float gBm = ((tas > 40f) ? (9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - 40f) / 20f)) : 0f);
				paceBetaG += (gBm - Mathf.Max(0.2f, Plugin.KBeta) * paceBetaG) * dt;
				paceBetaG = Mathf.Clamp(paceBetaG, -20f, 20f);
				// gravity is charged only for sideslip that exists and has its sign (the loop may be correcting it already)
				float bErrM = beta - betaPedalCmd;
				float bGEff = (bErrM * paceBetaG > 0f) ? Mathf.Sign(bErrM) * Mathf.Min(Mathf.Abs(bErrM), Mathf.Abs(paceBetaG)) : 0f;
				float betaOwn = bErrM - bGEff;
				float mLeftM = Mathf.Max(0f, Mathf.Max(1f, stopSlipMarginDbg) - Mathf.Abs(betaOwn));
				paceMLeftDbg = mLeftM;
				float cAmM = Mathf.Max(0.2f, Mathf.Abs(num));
				// the yaw's capacity toward each side: no more than the allocator is delivering of the loop's demand (demand less
				// its unmet report), no more than the axis is measurably doing plus the committed increment arriving within the
				// horizon; at least a share of what it has shown it can do (so a move can start)
				float desZ = omegaDotDes.z * 57.29578f;
				float capZP = Mathf.Min(Mathf.Max(0f, desZ - Mathf.Max(0f, allocUnmet.z)), Mathf.Max(0f, rDot + dzH));
				float capZN = Mathf.Min(Mathf.Max(0f, 0f - desZ - Mathf.Max(0f, 0f - allocUnmet.z)), Mathf.Max(0f, 0f - (rDot + dzH)));
				capZP = Mathf.Max(capZP, PaceSeenUse * aYawSeen);
				capZN = Mathf.Max(capZN, PaceSeenUse * aYawSeen);
				// 2.7.1: while the allocator reports nothing unmet toward a side, the capacity there is the authority still available
				// (so the loop is let ask for it); it falls to what is being delivered as the allocator reports it cannot. 2.7.0 used
				// only the delivered / demonstrated figure, and the loop's demand is proportional to the lead the window allows, so
				// the yaw was never asked for more than it had shown: a reversal at 413 kt sat at zero roll for 0.3 s with the rudders
				// at 14 of 35 deg and unmet 0, and one at 369 kt rolled back toward the old side (Tom, 09:59 log).
				// 2.7.2: saturation relative to the axis's authority (an unmet of 15 deg/s^2 is nothing at 400 kt and most of the yaw at
				// 200 km/h, where aYaw is 23)
				float satScZ = Mathf.Max(PaceSatDead, PaceSatFrac * aYaw);
				float satZP = Mathf.Clamp01(Mathf.Max(0f, allocUnmet.z) / satScZ);
				float satZN = Mathf.Clamp01(Mathf.Max(0f, 0f - allocUnmet.z) / satScZ);
				// the open capacity: what the axis is doing plus the committed increment arriving within the horizon, plus the
				// authority still available beyond the commanded deflections (zero with the effectors sent to their stops: then the
				// move is paced by what arrives, the slew case of the 154 s / 339 kt reversal)
				float openZP = Mathf.Max(0f, rDot + dzH) + PaceYawUse * hdZP;
				float openZN = Mathf.Max(0f, 0f - (rDot + dzH)) + PaceYawUse * hdZN;
				capZP = Mathf.Lerp(Mathf.Max(capZP, openZP), capZP, satZP);
				capZN = Mathf.Lerp(Mathf.Max(capZN, openZN), capZN, satZN);
				// 2.7.4: never above the allocator's own authority less its saturation derate (the 2.5.12 ceiling: past the rudder
				// fade the tables overstate the stabs' yaw 3-4x, and the committed increment is read from those tables). 11:30 log,
				// 116 s: a push-over from 64 deg AoA read 143-162 of capacity with 26-129 unmet and made 22 deg of sideslip.
				float capCeilZ = Mathf.Max(5f, aYaw) * (1f - Mathf.Lerp(PaceSatDerateLo, PaceSatDerateHi, 1f - rudderAuth) * Mathf.Clamp01(yawSat));
				capZP = Mathf.Min(capZP, capCeilZ);
				capZN = Mathf.Min(capZN, capCeilZ);
				// the lead that capacity closes inside the margin left (L^2 cos AoA / 2a = margin), plus the loop's ordinary
				// tracking error, which shrinks with the margin so that at the margin the roll waits for the yaw
				float flM = PaceFloor * Mathf.Clamp01(mLeftM / 2f);
				float lPm = PaceMarginK * Mathf.Sqrt(2f * capZP * mLeftM / cAmM);
				float lNm = PaceMarginK * Mathf.Sqrt(2f * capZN * mLeftM / cAmM);
				// never less than what the capacity closes within the horizon: the loop keeps driving the axis at its capacity
				// with the margin spent, instead of pinning the target to the axis (which asks it for nothing and stalls the move)
				// 2.7.1: the floor is what the yaw loop needs as error to ask for that capacity (its gain KYaw; a held target has no
				// feed-forward), so at the margin the lead x gain = the delivered capacity - the loop keeps asking exactly what the
				// axis gives. 2.7.0's capacity x 0.15 s asked for a fraction of it.
				// 2.7.1: the lead is the margin's (plus PaceLeadT of the capacity, the sideslip rate accepted while the roll waits); the
				// yaw is driven by the loop's feed-forward instead - the target moves toward the demand at the capacity (below), so the
				// loop asks exactly what the axis gives with no lead at all
				// 2.17.0: the lead floor is the error the yaw loop needs to ask for the whole capacity, capacity / KYaw (the 2.7.1 comment
				// above; the code had 0.1 s of capacity against a 2.5 /s loop, a quarter of it, and 2.7.2 switched off the feed-forward that
				// covered the rest). With the margin spent and the yaw weak (no fins, 65 deg) the target was pinned 1 deg/s ahead of the
				// rotation and the yaw was asked for ~5 deg/s^2 of its 20 (23:30 log, 110 s reversal: 2.5 s at +19 against a -30 demand).
				float leadTw = (PaceLeadT < 0f) ? (1f / Mathf.Max(0.5f, Plugin.KYaw)) : PaceLeadT;
				reachPos = Mathf.Max(lPm, capZP * leadTw) + flM;
				reachNeg = Mathf.Max(lNm, capZN * leadTw) + flM;
				paceMarginLeadDbg = Mathf.Max(lPm, lNm);
				paceCapZPDbg = capZP;
				paceCapZNDbg = capZN;
				// the roll axis the same way (its shortfall is sideslip at sin AoA)
				float desX = omegaDotDes.x * 57.29578f;
				float capXP = Mathf.Min(Mathf.Max(0f, desX - Mathf.Max(0f, allocUnmet.x)), Mathf.Max(0f, pDotM + dxH));
				float capXN = Mathf.Min(Mathf.Max(0f, 0f - desX - Mathf.Max(0f, 0f - allocUnmet.x)), Mathf.Max(0f, 0f - (pDotM + dxH)));
				float satScX = Mathf.Max(PaceSatDead, PaceSatFrac * aRoll);
				float satXP = Mathf.Clamp01(Mathf.Max(0f, allocUnmet.x) / satScX);
				float satXN = Mathf.Clamp01(Mathf.Max(0f, 0f - allocUnmet.x) / satScX);
				float openXP = Mathf.Max(0f, pDotM + dxH) + PaceRollUse * hdXP;
				float openXN = Mathf.Max(0f, 0f - (pDotM + dxH)) + PaceRollUse * hdXN;
				paceCapXPDbg = Mathf.Lerp(Mathf.Max(capXP, PaceSeenUse * aRollSeen, openXP), Mathf.Max(capXP, PaceSeenUse * aRollSeen), satXP);
				paceCapXNDbg = Mathf.Lerp(Mathf.Max(capXN, PaceSeenUse * aRollSeen, openXN), Mathf.Max(capXN, PaceSeenUse * aRollSeen), satXN);
				paceCapXPDbg = Mathf.Min(paceCapXPDbg, Mathf.Max(5f, aRollPos));
				paceCapXNDbg = Mathf.Min(paceCapXNDbg, Mathf.Max(5f, aRollNeg));
				// 2.7.2 (Tom, 10:19): the window does not hold the target in measured mode - the commanded steady rate is the only
				// lever (the cap below), and a reversal from it runs at the tuned acceleration / braking whatever the yaw does.
				// The window binds only while the allocator reports the axis cannot deliver what the loop asks (airframe-wide, any
				// effector): a latch on the unmet report, on past PaceSatDead + PaceSatSpan x PaceGateOn, off below x PaceGateOff -
				// never a blend toward the raw target (2.7.0's partial gate leaked it).
				float satZm = Mathf.Max(satZP, satZN);
				if (satZm > PaceGateOn)
				{
					paceLatchZ = true;
				}
				else if (satZm < PaceGateOff)
				{
					paceLatchZ = false;
				}
				gateZ = paceLatchZ ? 1f : 0f;
				float satXm = Mathf.Max(satXP, satXN);
				if (satXm > PaceGateOn)
				{
					paceLatchX = true;
				}
				else if (satXm < PaceGateOff)
				{
					paceLatchX = false;
				}
				gateX = paceLatchX ? 1f : 0f;
				paceHdZPDbg = hdZP;
				paceHdZNDbg = hdZN;
				paceDzHDbg = dzH;
				paceDzAllDbg = dzAll;
				paceDxHDbg = dxH;
				paceDxAllDbg = dxAll;
				paceRHDbg = rHm;
				pacePHDbg = pHm;
				paceSeenZDbg = aYawSeen;
				paceReachDbg = reachPos;
				// the sideslip correction's share of the body yaw (last tick's rS, velocity-vector yaw)
				float yawCorr = rSDbg * num;
				// the demand is judged at the AoA the jet will have at the end of the horizon (an AoA sweep under the roll moves
				// the yaw the roll needs at pS cos AoA x AoA rate; a push + roll through zero AoA reverses it)
				float aH = (alpha + alphaDot * PaceHorizon) * (MathF.PI / 180f);
				float sinH = Mathf.Sin(aH);
				// 2.5.4: inside +-PaceSinMin of zero AoA the roll's share of the yaw demand (pS sin AoA) is too small for any roll to
				// settle a yaw error - the yaw loop owns it - so the window is off there (2.5.3 divided by a floored sin and turned a
				// leftover yaw rate at the zero crossing into roll demands of 400-570 deg/s, then a flip to zero: 09:44 log, 318 / 324
				// / 328 s)
				bool winOn = Mathf.Abs(sinH) >= PaceSinMin;
				float sinD = winOn ? sinH : (PaceSinMin * ((sinH >= 0f) ? 1f : -1f));
				float lo = (rM - reachNeg - yawCorr) / sinD;
				float hi = (rM + reachPos - yawCorr) / sinD;
				if (lo > hi)
				{
					float tmpP = lo;
					lo = hi;
					hi = tmpP;
				}
				// the roll axis the same way: the pair's body-roll demand (pS cos AoA - rS sin AoA) may not lead the measured roll
				// rate by more than the roll axis can close within the horizon (its acceleration now, plus PaceRollUse of the
				// roll authority on that side). Past ~40 deg AoA the roll axis is the weak one: the yaw delivers its share and the
				// roll does not, and that too is sideslip (bench, full aft + roll at 400 km/h, 42 deg AoA: p 1 against 30 asked,
				// r 22, sideslip -9).
				float pM = pMm;
				float pDot = pDotM;
				float cosH = Mathf.Cos(aH);
				float loR = float.MinValue;
				float hiR = float.MaxValue;
				if (Mathf.Abs(cosH) > PaceSinMin)
				{
					float rollPos = Mathf.Max(0f, pDot * PaceHorizon + PaceRollUse * aRollPos * PaceHorizon) + PaceFloor;
					float rollNeg = Mathf.Max(0f, 0f - pDot * PaceHorizon + PaceRollUse * aRollNeg * PaceHorizon) + PaceFloor;
					// 2.7.0: measured; a roll shortfall is sideslip at sin AoA, so the margin buys sqrt(2 a (margin - |beta|) / sin AoA) of roll-rate lead
					float sAmR = Mathf.Max(0.05f, Mathf.Abs(num2));
					float flR = PaceFloor * Mathf.Clamp01(paceMLeftDbg / 2f);
					rollPos = Mathf.Max(PaceMarginK * Mathf.Sqrt(2f * paceCapXPDbg * paceMLeftDbg / sAmR), paceCapXPDbg * PaceLeadT) + flR;
					rollNeg = Mathf.Max(PaceMarginK * Mathf.Sqrt(2f * paceCapXNDbg * paceMLeftDbg / sAmR), paceCapXNDbg * PaceLeadT) + flR;
					float rollCorr = 0f - rSDbg * num2;
					loR = (pM - rollNeg - rollCorr) / cosH;
					hiR = (pM + rollPos - rollCorr) / cosH;
					dbgWinLoR = loR; dbgWinHiR = hiR; dbgRollPos = rollPos;
					if (loR > hiR)
					{
						float tmpR = loR;
						loR = hiR;
						hiR = tmpR;
					}
					// intersect; when the two axes disagree the yaw window stands (the sideslip is what is protected)
					if (loR <= hi && hiR >= lo)
					{
						lo = Mathf.Max(lo, loR);
						hi = Mathf.Min(hi, hiR);
					}
					else
					{
						// 2.7.0: the windows do not overlap - one axis is ahead of the other. The laggard is the one whose implied velocity
						// roll is further from the stick's demand; its window stands, so the other axis is brought back to it (a yaw that
						// cannot follow holds the roll down to it; a roll that cannot follow holds the yaw back to it)
						float pSrM = (rM - yawCorr) / sinD;
						float pSpM = (pM - rollCorr) / cosH;
						if (Mathf.Abs(num11 - pSpM) > Mathf.Abs(num11 - pSrM))
						{
							lo = loR;
							hi = hiR;
						}
					}
				}
				// never a roll against the one in progress (or against the one asked for, from rest): stop instead
				float sideP = (Mathf.Abs(pTraj.x) > 2f) ? Mathf.Sign(pTraj.x) : Mathf.Sign(num11);
				if (sideP > 0f && hi < 0f)
				{
					lo = 0f;
					hi = 0f;
				}
				else if (sideP < 0f && lo > 0f)
				{
					lo = 0f;
					hi = 0f;
				}
				// 2.7.0: the gate is the measured shortfall (allocator unmet + undelivered slew) per axis, nothing from the rudders' load
				// (2.12.1: the 2.5.x rudder-usage gate and its airframe-usage variant, overwritten here since 2.7.0, are deleted)
				float wSat = gateZ;
				paceGateF = Mathf.Max(wSat, paceGateF - dt / Mathf.Max(0.02f, PaceGateTau));
				wSat = winOn ? paceGateF : 0f;
				paceGateXF = Mathf.Max(gateX, paceGateXF - dt / Mathf.Max(0.02f, PaceGateTau));
				paceGateZDbg = wSat;
				paceGateXDbg = paceGateXF;
				float tgtRaw = num11;
				paceTgtRaw = tgtRaw;
				// 2.5.11: the two laws are blended by RollPaceRateBlend (0 = 2.5.7 pacing: window + acceleration cap; 1 = 2.5.8 rate
				// cap; between = both, in proportion). RollPaceRateMode false = blend 0.
				// 2.6.9: the rate cap below is a model of the RUDDERS - their crossing time to the far stop, their tables - so it holds
				// only where the rudders are the yaw axis. Past the rudder fade (rudderAuth -> 0, 50-60 deg AoA here) it read the
				// faded rudders' crossing against the stabs' authority and sat on PaceRateFloor (65 deg / 300 km/h: cap 10 against an
				// airframe limit of 36), and at 0.4 that held the steady falling-leaf command to ~70 % of the limit while the
				// window alone kept the reversal sideslip the same (5.7 vs 5.5 deg). So the blend counts where its model is valid:
				// your RollPaceRateBlend at full where the rudders have their authority, fading to the window alone as they fade.
				// 2.7.2: in measured mode the cap counts at every AoA (its capacity is the whole airframe's), at Tom's blend
				// 2.7.2: in measured mode the cap's capacity is the whole airframe's; its share is Tom's blend, fading with the rudders
				// (past their fade the yaw is the velocity roll and the one-axis law's own braking owns the reversal: no falling-leaf tax)
				// Su-30 0.1.2 (RateCapRudderShare): the rate cap is a model of the RUDDERS (their crossing time, their tables), so it
				// counts as far as the rudders ARE the yaw axis. The F-22E / Ifrit have that from their 50-60 deg rudder fade; this jet
				// keeps its fins to 75-85 deg (the Revoker's centreline rule), so at 70 deg and 160 km/h the cap still read the rudders'
				// ~3 deg/s^2 against the nozzles' ~23 and held the velocity roll flat at 23-26 deg/s from 110 to 400 km/h (Tom's 7 Nov
				// logs; Ifrit 25 -> 67, F-22E 20 -> 40 deg/s over the same IAS). Its share is now the rudders' part of the yaw axis's
				// one-side capacity (rudders + nozzles, each over its usable travel), full from RateCapShareFull up.
				rudShareDbg = 1f;
				if (RateCapRudderShare && Plugin.RollRateCapFloor < 0.999f && canted != null)
				{
					float zRud = 0f;
					float zOth = 0f;
					foreach (YawEff yS2 in yawEffs)
					{
						YawRange(yS2, out float zH2, out float zL2);
						float aE2 = 0.5f * (Mathf.Max(0f, zH2) + Mathf.Max(0f, 0f - zL2)) * yS2.auth;
						if (yS2.rudder)
						{
							zRud += aE2;
						}
						else
						{
							zOth += aE2;
						}
					}
					rudShareDbg = (zRud + zOth > 1E-06f) ? (zRud / (zRud + zOth)) : 1f;
				}
				float rateBlend = Plugin.RollPaceRateBlend * Mathf.Clamp01(rudderAuth) * Mathf.Lerp(Plugin.RollRateCapFloor, 1f, Mathf.Clamp01(rudShareDbg / Mathf.Max(0.05f, RateCapShareFull)));
				rateBlendEffDbg = rateBlend;
				float num11Rate = num11;
				float pSRateCap = 9999f;
				if (rateBlend > 0f)
				{
					// 2.5.8 rate mode: no window, no acceleration cap. The target is capped at the roll rate whose full stick
					// reversal at the tuned acceleration / braking the yaw can follow within the sideslip margin (your rule: slow
					// the commanded rate, not the move). Yaw-rate swing of the reversal: dR = 2 pS |sin AoA|. The rudders cross
					// from where they are to the other stop (tCross, slew), then pull dR down at their one-side yaw aOpp:
					// sideslip ~ cos AoA x (dR tCross / 2 + dR^2 / (2 aOpp)) x PaceRateK. Solve for dR at the margin, cap pS.
					float aOpp = 0f;
					float tCross = 0f;
					// Revoker 0.3.0: the rudder (faded with the fin) and the yaw nozzle, each crossing from where it is at its own
					// slew; the crossing time the reversal waits for is the yaw-weighted mean of theirs (the two rudders of the F-22E
					// were one time). No allocator-wide credit past the rudder fade: the nozzle is the yaw axis there (see yawEffs).
					float tCrossW = 0f;
					foreach (YawEff yP in yawEffs)
					{
						Effector seP = yP.e;
						float aE = 0.5f * Mathf.Abs(seP.MAt(yP.hi).z - seP.MAt(yP.lo).z) * iz * yP.auth;
						aOpp += aE;
						// the crossing: from where the effector is now to the stop on the far side of center (the reversal side)
						float far = (Mathf.Abs(seP.now - yP.hi) > Mathf.Abs(seP.now - yP.lo)) ? yP.hi : yP.lo;
						tCrossW += aE * Mathf.Abs(far - seP.now) / Mathf.Max(1f, seP.rate);
					}
					tCross = (aOpp > 1E-06f) ? (tCrossW / aOpp) : 0f;
					// Revoker 0.4.0: the fin's restoring yaw over the excursion (see stiffCredit)
					aOpp += stiffCredit + dampCredit;
					// 2.5.12: less what the allocator reports it cannot deliver
					aOpp *= 1f - Mathf.Lerp(PaceSatDerateLo, PaceSatDerateHi, 1f - rudderAuth) * Mathf.Clamp01(yawSat);
					float bMrg = Mathf.Max(1f, stopSlipMarginDbg);
					// 2.7.2 (Tom, 10:19): the capacity a reversal can call on is the whole airframe's, not the rudders': every effector's
					// yaw still available toward the reversal's side from where the allocator has it (rudders, stabs, the Ifrit's roll TVC
					// - whatever the allocator can turn to yaw), less what it reports unmet, plus the airframe's own directional
					// stiffness at the margin sideslip (the passive share: the airframe balancing laterally at this AoA). The dead time
					// is the yaw-weighted travel of those effectors to the deflection the reversal needs.
					float sideRev = 0f - Mathf.Sign(num11 * sinH);
					float passive = PaceStiffUse * Mathf.Max(0f, slipStiffDbg) * bMrg;
					// 2.8.1: the reversal's capacity is the estimator's figure toward the reversal side (measured yaw acceleration now, the
					// committed increment arriving within the horizon, and what the allocator itself would still give - 1.2-1.4x the realised
					// yaw in the 12:31 flight where the table headroom read 1.8x), plus the passive stiffness at the margin
					aOpp = Mathf.Max(5f, ((sideRev >= 0f) ? capEst.capZeP : capEst.capZeN) + passive);
					tCross = 0f;
					float wT = 0f;
					foreach (Effector seA in effs)
					{
						if (seA == null || !seA.active || seA.M == null || seA.N < 2)
						{
							continue;
						}
						Vector3 mA = seA.MAt(seA.max);
						Vector3 mB = seA.MAt(seA.min);
						float yA = Mathf.Abs(mA.z - mB.z) * iz;
						if (yA < 5f)
						{
							continue;
						}
						float farA = ((mA.z >= mB.z) == (sideRev > 0f)) ? seA.max : seA.min;
						tCross += yA * Mathf.Abs(farA - seA.cmd) / Mathf.Max(1f, seA.rate);
						wT += yA;
					}
					tCross = (wT > 1f) ? (tCross / wT) : 0f;
					rcAOppDbg = aOpp;
					float cH = Mathf.Max(0.2f, Mathf.Abs(cosH));
						// 2.5.10: RollPaceRateSlip (Dev key) scales the margin the cap aims at - the roll rate / sideslip compromise in flight
						float cQ = 2f * bMrg * Plugin.RollPaceRateSlip / Mathf.Max(1f, aOpp * cH * Mathf.Max(0.1f, PaceRateK));
					float dR = aOpp * (0f - 0.5f * tCross + Mathf.Sqrt(0.25f * tCross * tCross + cQ));
					// 2.6.9 (PaceRateRamp): the reversal the cap models is the one the law will fly, not a step. The demanded yaw
					// rate swings at aT |sin AoA|, aT = the trajectory's reversal acceleration as the law sets it (RevFloorK x roll
					// authority, or the brake law) bounded by the governor's own open-loop cap for the reversing side - the num17
					// the trajectory gets. The yaw answers after tCross / 2 at aOpp. Sideslip ~ the area between the two:
					//   E(dR) = dR tau + dR^2 / 2 (1/aOpp - 1/aD),  tau = tCross / 2
					// which is the 2.5.8 step model exactly as aD -> infinity. Where the yaw out-accelerates the demand it catches
					// the ramp and tracks it: E saturates at aD tm tau / 2 and, inside the margin, the cap does not bind. Measured
					// state and the law's own numbers only. (Bench, blend 0.4: full-aft reversal 320 km/h 6.6 -> 3.6 deg, worst of
					// the regression set 6.6 -> 4.3; Ifrit 0.7.2 found the same 4x pessimism in the step model.)
					if (PaceRateRamp)
					{
						float cA = Mathf.Max(Mathf.Abs(num), 0.05f);
						float aLaw = (ReversalFloor && RevConst) ? (RevFloorK * aRoll / cA) : (Plugin.RollBrakeUse * (BrakeUseAll ? aRollGo : aRollWing) / cA);
						// 2.7.2: in measured mode the trajectory runs at the law's own acceleration (no open-loop cap)
						float aT = Mathf.Clamp(aLaw, 20f, Plugin.MaxRollAccel);
						float aD = aT * Mathf.Max(Mathf.Abs(sinH), PaceSinMin);
						float tau = 0.5f * tCross * RateCapTauK;
						// 2.6.9 (RateCapNeedTravel): the rudders do not have to reach the far stop before the yaw turns - only the
						// deflection that makes the yaw acceleration the reversal asks for (aD of their one-side aOpp), from where they
						// sit, through center. At high dynamic pressure that is a few degrees (corkscrew entry at 700 km/h: rudders at
						// +-8..13 deg, the far-stop crossing put the cap at 25-30 deg/s where reversals from 208 made 3.9 deg); at
						// moderate q it is most of the stroke and nothing changes.
						if (RateCapNeedTravel)
						{
							// the effectors need only the deflection that makes aD of their aOpp, through center: about (0.5 + 0.5 need) of the stroke
							float needM = Mathf.Clamp01(aD / Mathf.Max(1f, aOpp));
							tau = 0.5f * tCross * Mathf.Lerp(0.5f, 1f, needM) * RateCapTauK;
						}
						else if (RateCapNeedTravel)
						{
							float need = Mathf.Clamp01(aD / Mathf.Max(1f, aOpp));
							float tN = 0f;
							foreach (SurfaceEffector seN in surfaces)
							{
								if (seN.kind != SurfaceKind.Rudder || !seN.active || seN.M == null || seN.N < 2)
								{
									continue;
								}
								float cN = 0.5f * (seN.max + seN.min);
								float hN = 0.5f * (seN.max - seN.min);
								float farN = (Mathf.Abs(seN.now - seN.max) > Mathf.Abs(seN.now - seN.min)) ? seN.max : seN.min;
								float dN = Mathf.Min(Mathf.Abs(farN - seN.now), Mathf.Abs(seN.now - cN) + need * hN);
								tN = Mathf.Max(tN, dN / Mathf.Max(1f, seN.rate));
							}
							tau = 0.5f * tN * RateCapTauK;
						}
						float qM = bMrg * Plugin.RollPaceRateSlip / Mathf.Max(0.01f, cH * Mathf.Max(0.1f, PaceRateK));
						float kQ = 0.5f * (1f / Mathf.Max(1f, aOpp) - 1f / Mathf.Max(1f, aD));
						float disc = tau * tau + 4f * kQ * qM;
						dR = (disc > 0f && tau + Mathf.Sqrt(disc) > 1E-04f) ? (2f * qM / (tau + Mathf.Sqrt(disc))) : 1E+06f;
					}
					pSRateCap = Mathf.Max(PaceRateFloor, dR / (2f * Mathf.Max(Mathf.Abs(sinH), PaceSinMin)));
					rcAOppDbg = aOpp;
					rcTcDbg = tCross;
					rcSinHDbg = sinH;
					if (RevSim)
					{
						// Revoker 0.6.0: the forward model, its reversal paced at the yaw the estimator says the effectors can add (as the Ifrit's)
						float aPaceRev = Mathf.Max(20f, PaceYawUse * aOpp / Mathf.Max(Mathf.Abs(sinH), PaceSinMin));
						pSRateCap = RevSimCap(bMrg * Plugin.RollPaceRateSlip, cosH, sinH, aPaceRev);
					}
					num11Rate = Mathf.Clamp(num11, 0f - pSRateCap, pSRateCap);
				}
				// the window may hold the roll in progress while the yaw comes down (that is the paced brake / reversal) but never
				// asks for more roll than the stick or the roll in progress, and never for a roll against the one in progress
				float clamped;
				// 2.19.0: where the rudders are the yaw axis (rudW -> 1) the band holds the roll at the yaw's rotation; where they are not
				// (faded past ~55 deg, or gone) the yaw effectors ARE the roll effectors (the stabs' differential, the flaperons) and a
				// roll loop holding the roll against them starves the yaw brake: 3 Nov 00:09 log, both fins gone, 82.4 s reversal at 65
				// deg - target held at 0 for 3.8 s, one stab at -21 and the other at 0, 8 deg/s^2 of yaw against a measured 24 when the
				// roll went with the yaw (23:49 log, 62 s). There the target is paced toward the demand at the yaw's capacity instead.
				float rudWwin = Mathf.Clamp01(rudderAuth) * Mathf.Clamp01(yawHealth / 0.5f);
				float clampedBand = Mathf.Clamp(num11, lo, hi);
				if (PaceHoldPacedByRudder && rudWwin < 0.999f)
				{
					// 2.17.0: while the yaw is reported saturated (the latch), the target is PACED toward the demand at the yaw's measured
					// capacity (the 2.7.1 limiter, on the estimator's fin-aware figure), not clamped to the band. The band's lead scales
					// with the sideslip margin LEFT, so with the margin spent it pinned the target to the rotation the jet had; the yaw loop
					// then had no error to brake with and nothing moved until the unmet report happened to fall under the latch's release
					// (23:30 log, both fins gone, 65 deg: reversal at 110 s held at +19 deg/s for 2.5 s against a -30 demand, zero crossing
					// 4.0 s after the stick; held push-over at 157 s cut the roll target -47 -> -10..-23 while coordination needed -30..-54,
					// sideslip +9 deg/s, departure). A move toward the axes (less lead) is free; a move away is paced at capacity / sin AoA.
					float pSrNow = (rM - yawCorr) / sinD;
					float from = paceRateInit ? paceRatePrev : pTraj.x;
					float dirT = Mathf.Sign(num11 - from);
					float capZd = Mathf.Max(PaceHoldCapFloor, (dirT * sinD > 0f) ? capEst.capZeP : capEst.capZeN);
					float rateT = capZd / Mathf.Max(Mathf.Abs(sinD), PaceSinMin);
					bool away = (num11 - pSrNow) * dirT > 0f;
					float lim = from + dirT * rateT * dt;
					float clampedPaced = (away && (num11 - lim) * dirT > 0f) ? lim : num11;
					clamped = Mathf.Lerp(clampedPaced, clampedBand, rudWwin);
				}
				else
				{
					clamped = clampedBand;
				}
				// 2.18.0: the window may HOLD the target where it is; it may never move it back toward the rotation (away from the stick's
				// demand). 23:49 log (both fins gone, 65 deg), 59.8 s reversal: the first 0.3 s of braking took the body roll 6 -> 1 deg/s
				// while the yaw (30 deg/s, slow to shed) still implied a 33 deg/s rotation; the band set the target at rotation - lead = 25
				// (p = 10.6 coordinated), the roll loop drove +X the OLD way for 1.2 s and took the stabs and flaperons from the yaw brake
				// (tgtX +47..+58 against a -30 demand); once the target came down the yaw braked at 24 deg/s^2. Tom rejected the same
				// behaviour in 2.7.0 ('keeps rolling the old way for a moment then reverses'). The target only ever moves toward the demand.
				if (PaceNoRetreat && paceRateInit && Mathf.Abs(num11 - paceRatePrev) > 0.01f)
				{
					float dirD = Mathf.Sign(num11 - paceRatePrev);
					if ((clamped - paceRatePrev) * dirD < 0f)
					{
						clamped = paceRatePrev;
					}
				}
				// 2.20.0 (PaceStopNoFin): without the rudders the yaw sheds at the stabs' pace (20-25 deg/s^2 at 65 deg) whatever the
				// roll does; a velocity-roll target of 0 against a yaw still carrying the rotation asks the body roll for -r tan AoA
				// (anti-coordinated), and the roll loop's try at it is the reversal's sideslip (bench leaf rev 370 no fins: p +11 -> -10
				// while r went 24 -> 7, sideslip 23 deg). Here the target may not go below the rotation the jet has with the body roll
				// stopped (r sin AoA, falling with the yaw): p -> 0, the nose yaws through, the sideslip grows at r cos AoA only.
				// The yaw loop is asked for its brake capacity directly (below, on rCmd), since this target alone gives it ~0.18 r.
				// Scaled by the FIN loss, not the rudder fade: with the fins on above the fade the sideslip the old stop builds is what
				// weathercocks the yaw round (bench 65 deg fullaft reversal 500: 1.6 s to zero; held to r sin AoA it took 3.1 s) - the
				// fins are the yaw brake there, and the clean jet keeps its reversal.
				paceStopNF = 0f;
				float finKw = Mathf.Clamp01(yawHealth / 0.5f);
				float stopNF = (PaceStopNoFin && finKw < 0.999f && sideP != 0f) ? Mathf.Max(0f, sideP * rM * sinD) : 0f;
				if (sideP != 0f && clamped * sideP < stopNF)
				{
					clamped = Mathf.Lerp(sideP * stopNF, 0f, finKw);
					paceStopNF = (stopNF > 0f) ? sideP : 0f;
				}
				else
				{
					float limP = Mathf.Max(Mathf.Abs(num11), Mathf.Abs(pTraj.x));
					clamped = Mathf.Clamp(clamped, 0f - limP, limP);
				}
				paceLoDbg = lo;
				paceHiDbg = hi;
				paceClampDbg = clamped;
				wSat *= 1f - stopHardW;   // Su-30 0.2.0: the full-rate stop (above)
				float num11Win = Mathf.Lerp(num11, clamped, wSat);
				dbgWinLo = lo; dbgWinHi = hi; dbgReachPos = reachPos; dbgReachNeg = reachNeg; dbgYawCorr = yawCorr; dbgWSat = wSat; dbgRateCap = pSRateCap; dbgWin = clamped; dbgRDot = rDot; dbgPDot = pDot;
				// the paced target continues from the target actually flown (the raw one while the latch is off)
				paceRatePrev = num11Win;
				paceRateInit = true;
				// 2.6.1: climbing back out of a hold at the yaw's pace (see paceWinHeld)
				paceRelDbg = 0f;
				if (PaceReleasePace > 0f)
				{
					float dirRel = Mathf.Sign(num11Win * sinH);
					float relAcc = PaceReleasePace * Mathf.Max(PaceFloor / PaceHorizon, PaceYawUse * Mathf.Max(dirRel * (rDot + dzH), PaceSeenUse * aYawSeen)) / Mathf.Max(Mathf.Abs(sinD), PaceSinMin);
					if (num11Win * paceWinPrev < 0f)
					{
						paceWinSame = false;
					}
					if (!paceWinHeld)
					{
						paceWinSame = Mathf.Abs(pTraj.x) > 2f && pTraj.x * num11Win > 0f;
					}
					if (paceWinHeld && paceWinSame && num11Win * paceWinPrev >= 0f && Mathf.Abs(num11Win) > Mathf.Abs(paceWinPrev))
					{
						// the measured yaw already supports a roll of (r - correction) / sin AoA: up to that the climb is free (no yaw
						// demand leads the yaw there); the pacing applies to the part above it
						float pSup = (rM - yawCorr) / sinD;
						float lim = Mathf.Max(Mathf.Abs(paceWinPrev) + relAcc * dt, (pSup * num11Win > 0f) ? Mathf.Abs(pSup) : 0f);
						if (Mathf.Abs(num11Win) > lim)
						{
							paceRelDbg = Mathf.Abs(num11Win) - lim;
							num11Win = Mathf.Sign(num11Win) * lim;
						}
					}
					paceWinHeld = Mathf.Abs(num11Win) < Mathf.Abs(tgtRaw) - 1f;
					paceWinPrev = num11Win;
				}
				else
				{
					paceWinHeld = false;
					paceWinSame = false;
					paceWinPrev = num11Win;
				}
				// blend: the window's hold and the rate cap's trim of the target, in proportion
				// 2.6.12 (RateCapOnReversal): the cap's share of the target applies only while a reversal is under way - the stick
				// asking for the opposite direction to the roll in progress. It predicts what a reversal from the present rate would
				// cost, and with no reversal it was trimming the steady roll for one that never came (02 Nov 00:54 log, 340 kt
				// corkscrew entry: roll 70 -> 17 deg/s at 45-55 deg AoA, sideslip 0.8-2.2 of a 4 deg margin; bench, the window
				// holds 0 there and the whole hold is the cap's 41 %)
				float revW = 1f;
				num11 = Mathf.Lerp(num11Win, num11Rate, rateBlend * PaceBlendTargetK * revW * (1f - stopHardW));
				paceLeadDbg = (rateBlend >= 0.5f) ? pSRateCap : ((sideP >= 0f) ? reachPos : reachNeg);
				paceHoldDbg = tgtRaw - num11;
				// 2.10.0: the 2.5.x open-loop acceleration cap (rudder tables) went with the table governor; the trajectory runs at the law's own acceleration
				paceAccelCap = -1f;
			}
			else
			{
				paceRDotF.init = false;
				pacePDotF.init = false;
				paceRateInit = false;
				paceGateF = 0f;
				paceWinHeld = false;
				paceWinSame = false;
				paceWinPrev = 0f;
				paceRelDbg = 0f;
			}
			paceOmegaZPrev = omega.z;
			paceOmegaXPrev = omega.x;
			paceInit = true;
			guardDbg = x;
			// 2.34.0 (PushGov): the lateral limit of the push side. Where the law would hold or bring the rotation down (stop, release,
			// reversal, a stick target under the rotation) and the yaw cannot shed the sweep's demand, the trajectory comes down no faster
			// than -L; where L > 0 it grows at L (the roll follows the yaw down)
			// rate band [L, U] for the velocity-roll reference; outside it the trajectory is re-aimed (target) and its acceleration set to the
			// band's edge. L > U (the pitch ran ahead of both axes this tick - the cap catches it next tick): the roll's edge, U.
			bool pushGovOn = !float.IsNaN(pushGovL) && pushGovSide != 0f && !pushBleed && !pushHeld;
			if (pushBleed) num11 = 0f;
			float pushGovRate = 0f;
			if (pushGovOn)
			{
				bool lawBrakes = pushGovSide * (num11 - pTraj.x) < 0f;
				float num11Law = num11;
				float lo = pushGovL, hi = Mathf.Max(pushGovL, pushGovU);
				if (pushGovL > pushGovU) lo = hi = pushGovU;
				if (PushGovRelNoGrow && stickRelW > 0.5f)
				{
					lo = Mathf.Min(lo, 0f - Mathf.Max(0.5f, PushGovRelBrakeMin));
				}
				// the law's own rate sign: braking (fast), or holding / growing (toward its target)
				if (lawBrakes && lo > -1E+05f)
				{
					// braking: as fast as the law wants, but no faster than -lo; if lo > 0 the reference grows at lo instead
					pushGovRate = lo;   // applied as min(num17, |lo|) below when lo < 0
					if (lo > 0f) num11 = pushGovSide * Mathf.Max(Mathf.Max(Mathf.Abs(pTraj.x), Mathf.Min(Mathf.Abs(pTraj.x) + 30f, pushGovCeil)), pushGovSide * num11);
					else if (hi < 0f) { }   // the law's brake is at least as fast as needed? (num17 >= |hi| normally)
				}
				else if (hi < 0f)
				{
					// holding or growing where the roll's ceiling says it must come down: brake at |hi|
					pushGovRate = hi;
					num11 = pushGovSide * Mathf.Max(0f, Mathf.Abs(pTraj.x) - 30f);
				}
				else if (lo > 0f)
				{
					// holding where the yaw says it must grow: grow at lo (no faster than hi)
					pushGovRate = Mathf.Min(lo, hi);
					num11 = pushGovSide * Mathf.Max(Mathf.Max(Mathf.Abs(pTraj.x), Mathf.Min(Mathf.Abs(pTraj.x) + 30f, pushGovCeil)), pushGovSide * num11);
				}
				else
				{
					pushGovOn = false;   // the law's own move is inside the band
				}
				// faded in over 2..8 deg/s of commanded AoA rate toward zero (an AoA held at a limit flickers around zero: no band there)
				float wGt = Mathf.Clamp01((pushGovA - 2f) / 6f);
				num11 = Mathf.Lerp(num11Law, num11, wGt);
			}
			// Su-30 0.3.5 (GyroRollTrade, Tom 10 Nov 22:44: "for corkscrew, aoa limit is often undershot"): GyroBudget (0.1.11) keeps the
			// gyroscopic roll (Iyy - Izz)/Ixx q r inside its share of the roll authority by cutting the pitch rate; over the top of a slow
			// corkscrew (72-100 km/h) the flight path bends at 30-48 deg/s and the pitch, held to 16-17, let the AoA fall 75 -> 38 (22:34
			// log, 184.5-186.5 s), and the 40 deg/s of yaw on the collapsing AoA made 38 deg of sideslip. The same budget is now also
			// spent on the roll: the velocity roll target is held to (budget / pitch rate wanted) / sin AoA, the yaw it implies, so the
			// yaw comes down and the pitch cap (which reads the measured yaw) opens - the corkscrew slows instead of the AoA collapsing.
			gyroRollLimDbg = 0f;
			if (GyroRollTrade && gyroRollW > 0f && Mode == FcsMode.AoAG && !isAI)
			{
				float rAllowG = gyroBudgetA / Mathf.Max(GyroBudgetQFloor, gyroQNeed);
				float pLimG = rAllowG / Mathf.Max(0.5f, Mathf.Abs(num2));
				gyroRollLimDbg = pLimG;
				if (Mathf.Abs(num11) > pLimG)
				{
					num11 = Mathf.Lerp(num11, Mathf.Sign(num11) * pLimG, gyroRollW * GyroRollShare);
				}
			}
			// Su-30 0.2.0 (StopIgnoresSlipOutsideMargin): the full-rate stop's target is zero, whatever the sweep / yaw-shed bands would hold
			if (stopHardW > 0f)
			{
				num11 = Mathf.Lerp(num11, 0f, stopHardW);
			}
			pSTgtDbg = num11;
			float num15 = Mathf.Abs(pTraj.x * num);
			float max = Mathf.Lerp(Plugin.RollAccelUseLowIas, Plugin.RollAccelUse, Mathf.InverseLerp(60f, 110f, ias));
			float a = Mathf.Clamp(0.85f - num15 / Mathf.Max(30f, pFull * (aRollGo / aRoll)), 0.1f, max) * aRollGo / Mathf.Max(Mathf.Abs(num), 0.05f);
			float num16 = 0.6f * aYaw / Mathf.Max(Mathf.Abs(num2), 0.05f);
			// 2.6.0: the trajectory's yaw-coordination caps (RollEntryCoord, BrakeYawCap, RevYawCap, CoordJerk) are gone; the pace
			// governor's acceleration cap (paceAccelCap) is the only pacing of the trajectory
			float num17 = (latAccDbg = a);
			// 1.8.41: the roll entry's acceleration yields with the governor too (the entry is where the surfaces deflect most)
			num17 *= Mathf.Lerp(RollGovAccelFloor, 1f, rollGov);
			loadRevDbg = 0f;
			// Su-30 0.3.8 (F-22E 4.1.2): the roll entry's acceleration as the law has it now (stick-side authority, the governor's share), and how much of a
			// loaded reversal's build-up is paced by the rudders' travel (0 = none: clean jet, the previous build)
			float entryA = num17;
			float revRampW = 0f;
			if (pTraj.x * (num11 - pTraj.x) < 0f)
			{
				float num18 = aRoll / Mathf.Max(30f, pFull);
				num17 = (Plugin.RollBrakeUse * (BrakeUseAll ? aRollGo : aRollWing) + num18 * num15) / Mathf.Max(Mathf.Abs(num), 0.05f);
				// 2.7.0 (PaceRevIntent): a reversal the stick asked for is a reversal, even while the measured window holds the target on
				// the near side - it brakes at the reversal's tuned acceleration, not the stop's
				bool revIntent = PaceRevIntent && paceTgtRaw * pTraj.x < 0f && Mathf.Abs(paceTgtRaw) > 10f;
				bool revNow = ((num11 * pTraj.x < 0f && Mathf.Abs(num11) > 10f) || revIntent) && ReversalFloor;
				float revFree = 0f;
				if (revNow)
				{
					float num19 = RevFloorK * aRoll / Mathf.Max(Mathf.Abs(num), 0.05f);
					num17 = (revFree = (RevConst ? num19 : Mathf.Max(num17, num19)));
				}
				// 2.1.6: with an asymmetric load, a roll brake / reversal may only accelerate as fast as the yaw left after the load
				// can follow (the roll's yaw rate p sin AoA has to reverse with it; with the rudders already deflected for the load
				// they slew across their whole travel). Clean jets unchanged. Su-30 0.3.8: moved inside the brake branch, before revA is stored.
				if (AsymSharedLat && Plugin.AsymSharedLatOn && asymValid && LoadRevYawK > 0f)
				{
					float dyG = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan));
					if (dyG > 0f)
					{
						float revCap = LoadRevYawK * aYaw * Mathf.Max(0.2f, sharedLatDbg) / Mathf.Max(Mathf.Abs(num2), 0.05f);
						// Su-30 0.3.8 (F-22E 4.1.2, RevLoadRamp): a reversal toward the side the load pulls. There the load and the roll damping brake
						// the old roll by themselves (the roll surfaces sit near neutral); held to this limit the trajectory waited at it and
						// built the new roll's acceleration after the zero crossing ("stop the roll, then spike"). What the 40 % stands for is
						// the rudders' travel from where the load and the old roll hold them to the other side, so on that side the reversal's
						// acceleration builds over that travel time (the jerk below) to one figure - the entry's, within the trajectory's own
						// yaw figure of the yaw left (60 %) - and keeps it through the crossing up to the new rate (the carry below). Against
						// the load the limit stays as it is. Stops are unchanged. RevLoadRamp = false: the 4.1.1 shape.
						if (RevLoadRamp && revNow && RudderCrossTime() > 0f)
						{
							float yawTop = num16 * Mathf.Max(0.2f, sharedLatDbg);
							revRampW = LoadRevPace(aRollGo, num, yawTop);
							revCap = Mathf.Lerp(revCap, Mathf.Max(revCap, Mathf.Min(entryA, yawTop)), LoadSideWeight(aRollGo));
						}
						loadRevDbg = dyG;
						num17 = Mathf.Lerp(num17, Mathf.Min(num17, revCap), dyG);
					}
				}
				if (revNow)
				{
					// Su-30 0.3.8 (F-22E 4.1.1, RevOneAccel): the reversal's acceleration is the one its brake flies, the load's limit included, and
					// the new roll starts there (one acceleration through the reversal, capped by what the yaw axis can follow).
					// RevOneAccel = false: the previous build (revA stored before the load's limit).
					revA = RevOneAccel ? num17 : revFree;
					revTarget = revIntent ? paceTgtRaw : num11;
				}
			}
			else if (revA > 0f && RevCarry)
			{
				if (num11 * revTarget <= 0f || Mathf.Abs(num11) < 10f)
				{
					revA = 0f;
				}
				else
				{
					// the new roll starts at the reversal's acceleration and blends to the entry's by half its target rate; Su-30 0.3.8 (F-22E 4.1.1)
					// from either side. Su-30 0.3.8 (F-22E 4.1.2): a reversal the load brakes (pace > 0) keeps its one acceleration up to the new rate
					// and its build-up keeps the rudders' pace
					float yawTopC = num16 * Mathf.Max(0.2f, sharedLatDbg);
					float pace = RevOneAccel ? LoadRevPace(aRollGo, num, yawTopC) : 0f;
					float num20 = 1f - Mathf.Clamp01(Mathf.Abs(pTraj.x) / Mathf.Max(10f, Mathf.Lerp(0.5f, 1f, pace) * Mathf.Abs(num11)));
					float carried = Mathf.Lerp(num17, revA, num20);
					num17 = RevOneAccel ? carried : Mathf.Max(num17, carried);
					if (pace > 0f)
					{
						num17 = Mathf.Lerp(num17, Mathf.Min(num17, Mathf.Min(entryA, yawTopC)), pace);
					}
					revRampW = pace;
					if (num20 <= 0f)
					{
						revA = 0f;
					}
				}
			}
			if (paceAccelCap > 0f)
			{
				num17 = Mathf.Min(num17, paceAccelCap);
			}
			num17 = Mathf.Clamp(num17, 20f, Plugin.MaxRollAccel);
			if (pushGovOn)
			{
				// faded in over 2..8 deg/s of commanded AoA rate (as the target above)
				float wG = Mathf.Clamp01((pushGovA - 2f) / 6f);
				num17 = Mathf.Lerp(num17, Mathf.Min(num17, Mathf.Abs(pushGovRate)), wG);
			}
			pushHeldB = 0f;
			if (pushBleed)
			{
				num17 = Mathf.Max(0.5f, pushPlanBleed);
			}
			else if (pushHeld && pTraj.x * (num11 - pTraj.x) < 0f)
			{
				// held stick, the roll moving toward a lower target (the law's own: the stick, the pace window): no faster than the plan's
				// yaw budget leaves after the AoA's share, and no faster than the law's own brake
				float aDotP = Mathf.Max(0f, (0f - Mathf.Sign(alpha)) * (qCmd - omegaV));
				pushHeldB = Mathf.Max(0f, pushPlanD - Mathf.Abs(pTraj.x) * Mathf.Abs(num) * aDotP * (MathF.PI / 180f)) / Mathf.Max(0.05f, Mathf.Abs(num2));
				num17 = Mathf.Max(0.5f, Mathf.Min(num17, pushHeldB));
			}
			float a2 = Mathf.Min(num17 / RollJerkTau, RollJerkServo * aRoll * rollSlew / Mathf.Max(Mathf.Abs(num), 0.05f));
			// Su-30 0.3.8 (F-22E 4.1.2): the jerk as the law has it; the run-out onto the target keeps it where a loaded reversal's build-up is paced below
			float a2Free = a2;
			if (revRampW > 0f)
			{
				// the acceleration of a reversal the load brakes builds over the time the rudders need from where they are to the other
				// side's stop (measured positions, their servo rate), not over RollJerkTau
				float tRud = RudderCrossTime();
				if (tRud > RollJerkTau) a2 = Mathf.Min(a2, num17 / Mathf.Lerp(RollJerkTau, tRud, revRampW));
			}
			if (isAI && Plugin.AIDirectRoll)
			{
				float aDir = Mathf.Min(AIRollDirectAccel, 1.3f * num16);
				num17 = Mathf.Max(num17, aDir);
				a2 = Mathf.Max(a2, aDir / 0.05f);
			}
			// 2.0.6 coordinated roll jerk: the stability-axis roll needs a yaw rate of p sin(a); the rudders reach full travel
			// no faster than their servo allows (35 deg at 100 deg/s), so the yaw acceleration can only build at about
			// aYaw x (rudder rate / half travel). The roll's jerk is held to what keeps that yaw build-up within reach
			// (CoordJerkK of it), below the rudder fade only (rudderAuth), so roll entries and reversals at moderate AoA no
			// longer outrun the yaw axis and leave sideslip behind.
			float jFloor = 100f;
			coordJerkDbg = a2;
			latAccDbg = num17;
			latJerkDbg = Mathf.Max(a2, jFloor);
			float endJ = RollEndJerk * Mathf.Lerp(1f, EndJerkHi, Mathf.InverseLerp(EndJerkLp0, EndJerkLp1, aRoll / Mathf.Max(30f, pFull)));
			lpDbg = aRoll / Mathf.Max(30f, pFull);
			// Ifrit 0.2.0, Stability Assist off: lateral stick = body roll rate, pedals = body yaw rate, two independent axes.
			// No sideslip hold, no pedal-to-roll coupling, no automatic coordination: the jet rolls about its velocity vector
			// only when the pilot puts in roll and yaw together. The rate limits, the damage / asymmetric-load limits on
			// them and the allocator's VMC / asymmetry trim are unchanged.
			if (mpoSep)
			{
				// Revoker 0.2.7 (Tom, 28 Oct): Stability Assist off = the stick rolls about the velocity vector at every AoA and
				// the pedals are pure body yaw rate at every AoA. 0.2.6 (the Ifrit 0.2.0 law) made the stick body roll and only
				// the part of it shared with a same-direction pedal a velocity-vector roll; below the coupling AoA the yaw axis
				// then kept the sideslip inside +-PedalSideslip (and washed it out), which coordinated a stick-only roll there
				// anyway (velocity-vector roll below ~36 deg, body roll above), and clamped the pedal's sideslip the same way.
				// Now: stick = the coupled law's stability-axis roll (its roll limits and acceleration), with the sideslip hold
				// and gravity term while the stick is in and the pedals are centred; pedals = body yaw rate on top, no sideslip
				// bound, no washout, no hold (the hold fades out with pedal so it never fights it).
				float cRaw = Mathf.Abs(sr);
				float cSgn = Mathf.Sign(sr);
				if (!mpoSepPrev)
				{
					pTraj.Reset(pCmdPrev * num + rCmdPrev * num2);
					mpoPTraj.Reset(0f);
					mpoRTraj.Reset(0f);
					mpoCoord = 1f;
				}
				mpoSepPrev = true;
				mpoCoord = Mathf.MoveTowards(mpoCoord, cRaw * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.3f)), 4f * dt);
				// velocity-vector roll from the stick
				float pSc = sr * num10;
				pTraj.Step(pSc, num17, Mathf.Max(a2, jFloor), Mathf.Max(a2 * Mathf.Min(endJ, 1f), jFloor), dt);
				float gTermM = (tas > 40f) ? (9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - 40f) / 20f)) : 0f;
				// Ifrit 0.3.7: the sideslip hold fades out at very low TAS / in reversed flow as in the coupled law
				float holdM = Mathf.Clamp01((tas - Plugin.SlipHoldTasMin) / Mathf.Max(1f, Plugin.SlipHoldTasSpan));
				float revM = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - 100f) / 20f);
				holdM *= revM * revM * (3f - 2f * revM);
				slipHoldDbg = holdM;
				float rSc = mpoCoord * rSF.Step(Mathf.Clamp(Plugin.KBeta * holdM * beta, 0f - num8, num8) + gTermM, 0.12f, dt);
				rSDbg = rSc;
				// no body roll of its own any more: the body-roll trajectory only winds down what a switch left in it
				float aB = Mathf.Clamp(Plugin.RollBrakeUse * aRollStop, 20f, Plugin.MaxRollAccel);
				float jB = Mathf.Max(aB / RollJerkTau, jFloor);
				mpoPTraj.Step(0f, aB, jB, jB, dt);
				// pure body yaw from the pedals (no sideslip limit on them). With the pedals centred and below the coupling AoA the
				// sideslip still washes out slowly (MpoSideslipWashout, like the airframe's own weathervane, which a held zero yaw
				// rate would otherwise block: bench, MPO roll and release, sideslip frozen at 57 deg).
				float rBodyT = sy * num8;
				float wB = (1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.CoupleAoA) / 14f)) * (1f - mpoCoord) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.05f));
				if (wB > 0f)
				{
					rBodyT = Mathf.Lerp(rBodyT, Mathf.Clamp(Plugin.MpoSlipWashout * beta + gTermM, 0f - num8, num8), wB);
				}
				float aR = Mathf.Max(10f, 0.7f * aYaw);
				mpoRTraj.Step(rBodyT, aR, Mathf.Max(aR / 0.15f, 60f), dt);
				pS = pTraj.x;
				float pM = pTraj.x * num - rSc * num2 + mpoPTraj.x;
				float rM = pTraj.x * num2 + rSc * num + mpoRTraj.x;
				float scM = Mathf.Max(1f, Mathf.Max(Mathf.Abs(pM) / num3, Mathf.Abs(rM) / num8));
				pCmd = pM / scM;
				rCmd = rM / scM;
				pDotFF = (pTraj.v * num + mpoPTraj.v) / scM;
				rDotFF = (pTraj.v * num2 + mpoRTraj.v) / scM;
				if (Mathf.Abs(omega.z * 57.29578f) > num8 && rDotFF * omega.z > 0f)
				{
					rDotFF = 0f;
				}
				pCmdPrev = pCmd;
				rCmdPrev = rCmd;
				SlipHistClear();
				return;
			}
			if (mpoSepPrev)
			{
				// back to the coupled law: the stability-axis roll rate the body rates make now
				pTraj.Reset(pCmdPrev * num + rCmdPrev * num2);
				mpoSepPrev = false;
			}
			pTraj.Step(num11, num17, Mathf.Max(a2, jFloor), Mathf.Max(Mathf.Max(a2, a2Free) * Mathf.Min(endJ, 1f), jFloor), dt);
			pS = pTraj.x;
			// Revoker 0.2.4 (CorkscrewYawShaping): with roll stick held above 30-45 deg AoA (pedals centred) the velocity-
			// vector roll is mostly body yaw (sin AoA), and its yaw part, pS sin(AoA), carried every step of the limits (the
			// braking floor's IAS prediction, the authority estimates) and every AoA change straight into the yaw rate. It is
			// passed through a critically damped second-order filter (CorkYawOmega, rad/s; acceleration cap CorkYawAccel) and
			// the roll part is derived from the same filtered value, so roll and yaw stay coordinated about the velocity vector.
			float wCork = (CorkYawOmega > 0f && Mode == FcsMode.AoAG && !isAI) ? (Mathf.Clamp01((Mathf.Abs(sr) - 0.3f) / 0.4f) * Mathf.InverseLerp(CorkShapeA0, CorkShapeA0 + 15f, alpha) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f))) : 0f;
			float rTgtC = pS * num2;
			if (wCork <= 0.001f)
			{
				corkR = rTgtC;
				corkRd = 0f;
			}
			else
			{
				float accC = CorkYawOmega * CorkYawOmega * (rTgtC - corkR) - 2f * CorkYawOmega * corkRd;
				accC = Mathf.Clamp(accC, 0f - CorkYawAccel, CorkYawAccel);
				corkRd += accC * dt;
				corkR += corkRd * dt;
				float rLimC = num10 * Mathf.Abs(num2);
				if (Mathf.Abs(corkR) > rLimC)
				{
					corkR = Mathf.Sign(corkR) * rLimC;
					corkRd = 0f;
				}
				pS = Mathf.Lerp(pS, corkR / Mathf.Max(0.5f, num2), wCork);
			}
			corkWDbg = wCork;
			float num21 = 0f;
			if (tas > GravFFTas0)
			{
				num21 = 9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - GravFFTas0) / Mathf.Max(1f, GravFFSpan));
				num21 = Mathf.Clamp(num21, 0f - GravFFMax, GravFFMax);
			}
			float num22 = ((Mode == FcsMode.GearDownRate) ? 1f : Plugin.KBeta);
			// Ifrit 0.3.0 (SlipTvcAssist): above 25 deg AoA the sideslip gain ramps to SlipGainHighAoA by 45 deg. There the
			// correction is mostly body roll and the nozzles supply it at any IAS. Bench, 65 deg corkscrew with a 1 s full-pedal
			// kick, 140-150 km/h: back under 3 deg sideslip in 4.3-4.7 s at KBeta 1.3, 0.5-0.8 s at 4.
			// Revoker 0.2.1 (SlipYawTvcAssist): the single nozzle has no roll authority (aRollTvc 0), so the high-AoA gain never
			// ramped and the sideslip loop ran at KBeta at 70 deg AoA; the yaw nozzle carries the yaw part of the correction at any
			// IAS below the TVC cutoff, so it opens the ramp too
			bool yawTvcSlip = Plugin.SlipYawTvcAssist && tvcy != null && tvcy.active;
			if (Mode != FcsMode.GearDownRate && Plugin.SlipGainFromLoops)
			{
				// Revoker 0.2.14 (SlipGainFromLoops): the sideslip-hold gain is what the rate loops carrying the correction can
				// follow. rS is flown as body yaw rS cos(a) through the yaw-rate loop (KYaw: 2.5/s = 0.4 s) and as body roll
				// -rS sin(a) through the roll-rate loop (KRoll: 6/s = 0.17 s), each plus the actuator lag. The sideslip loop
				// closes through them as gain/s x lag (beta-dot = -rS), so its phase margin is set by gain x lag: SideslipGain
				// (1.3) is the gain the yaw loop supports (1.3 x 0.47 s), and where the correction rides the faster roll loop
				// the gain rises with the shorter lag, cos^2 / sin^2 weighted by the kinematic split. A path that would have
				// to deliver its share beyond the spare authority of its axis (the roll axis braking or saturated for the
				// velocity-vector roll itself) counts the time that share takes at the spare authority instead of 1/K, capped
				// at 1 s. 30 Oct 22:48 log, full push + full roll, -8 -> -55 deg AoA, 78 -> 55 kt: the fixed SlipGainHighAoA
				// ramp (4, opened on this jet by the yaw nozzle in 0.2.1) ran the loop through the 0.4 s yaw loop at ~15 deg of
				// phase margin: sideslip -3.4 <-> +3.6 deg at ~1 Hz, yaw rate +-12 deg/s about its command, rudder stop to
				// stop, the roll governor cutting the roll twice (the hitch). The F-22E, at 1.3 there, recovers -6.8 deg in one
				// move. Here: 0 deg AoA 1.3, -50 deg ~1.85, 70 deg ~2.3.
				float gLoops = Plugin.KBeta * latLagYawNom / Mathf.Max(0.02f, latLagEff);
				// pedals centred only: with pedal in, the pedal sideslip command is tracked at KBeta as before
				num22 = Mathf.Lerp(num22, gLoops, 1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f));
			}
			else if (Mode != FcsMode.GearDownRate && Plugin.SlipTvcAssist && (aRollTvc > 0.5f || yawTvcSlip))
			{
				// pedals centred only: with pedal in, the pedal sideslip command is tracked at KBeta as before
				num22 = Mathf.Lerp(num22, Mathf.Max(num22, Plugin.SlipGainHighAoA), Mathf.InverseLerp(Plugin.SlipGainAoAStart, Plugin.SlipGainAoAStart + 20f, Mathf.Abs(alpha)) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f)));
			}
			slipGainDbg = num22;
			// Ifrit 0.3.0 (SlipTvcAssist): the sideslip correction rS is flown as body roll -rS sin(a) and body yaw rS cos(a), so
			// each part is bounded on its own axis: the yaw part by the yaw-rate limit, the roll part by a stop-side roll budget
			// that counts the differential TVC. 0.2.2 bounded rS by the yaw-rate limit alone; at 65 deg AoA and 70-120 km/h that
			// is 11-14 deg/s from the aero yaw authority, the correction sat on it and the sideslip reached -15 deg (11:51 log,
			// 145-152 s) while the nozzles, the strongest roll effector there, had travel to spare. Same as 0.2.2 at low AoA.
			float slipLim = num8;
			pSlipMax = num3;
			if (Plugin.SlipTvcAssist && aRollTvc > 0.5f)
			{
				float aStopSide = Mathf.Min(aRollPos, aRollNeg);
				pSlipMax = Mathf.Max(num3, Mathf.Min(Plugin.MaxRollRate, Plugin.RollStopFactor * (aStopSide + aRollTvc)));
				slipLim = Mathf.Max(num8, Mathf.Min(num8 / Mathf.Max(num, 0.25f), pSlipMax / Mathf.Max(Mathf.Abs(num2), 0.25f)));
			}
			// Ifrit 0.3.5: sideslip predictor (dev SlipPredictTime, s; SlipPredictWashout, s). The sideslip correction acts
			// through the inner roll/yaw rate loops, which in game follow pCmd/rCmd ~0.3 s late (20 Oct logs: pCmd -> p
			// 0.19-0.32 s, rCmd -> r 0.29-0.37 s, gain ~1; the bench's are ~0.1 s, which is why the bench never showed it).
			// beta-dot = -rS, so the loop is gain / s with that delay: at the high-AoA gain of 4 it rings at omega ~ gain
			// (0.72 Hz) and with 0.33 s of delay it sits on the stability edge. 20 Oct 23:10 log (0.3.3), full thrust, roll
			// stick and pedals centred, 45-55 deg AoA, 54-75 m/s: sideslip peaks 2.9 -> 3.7 -> 4.4 deg every 1.4 s
			// (growing), decelerons +-30 deg; also 157-161 s (+-2.4 deg) and the 22:37 log at idle. The 0.3.3 sideslip-rate
			// lead could not cover 0.3 s (delay-loop model, gain 4, delay 0.3: decay per cycle 0.89 -> 0.78 with the lead;
			// it matched the logged 1.4 s period and growth) and is removed. The predictor is a Smith predictor: the rS this
			// law has already commanded over the last SlipPredictTime has not shown up in the sideslip yet, so the correction
			// works on beta - integral(commanded rS) over that window, the sideslip those commands will leave. The commanded
			// rS is the one actually sent (after the limit scaling), less the gravity term (which only cancels gravity's own
			// sideslip rate); it is washed out over SlipPredictWashout so a held correction (a yaw trim against a load) leaves
			// no extra steady sideslip. Model, gain 4, predictor at 0.3 s: decay per cycle <= 0.23 for a true delay of
			// 0.2-0.45 s and inner-loop gain 0.8-1.2 (no predictor: 0.21-2.99), settles in 0.8-5 s. Gear-down rate law and
			// the MPO separate-axes law unaffected; faded in from 20 to 40 m/s TAS. 0 = no predictor (0.3.2).
			float predT = (Mode == FcsMode.GearDownRate) ? 0f : Plugin.SlipPredictTime;
			// Ifrit 0.4.0: the roll-stick gate is a dev key (SlipPredictRollGate, default 0 = none). With the gate the predictor
			// was off in every held corkscrew, and that is exactly where the 22 Oct 00:30 log rang (+-12 deg, +-50 deg/s).
			float rollGate = Plugin.SlipPredictRollGate;
			float predGate = Mathf.Clamp01((tas - 20f) / 20f) * ((rollGate > 0f) ? (1f - Mathf.Clamp01(Mathf.Abs(sr) / rollGate)) : 1f);
			// faded in by the loop's gain x delay (its phase cost, rad at crossover): off up to 0.4 (KBeta 1.3), full from 1.0
			// (gain 3.3+). Below that the loop has its margin and the predictor only slowed the recovery (bench, rolling pull
			// at 600 km/h, 13 deg AoA, gain 2.4: -4.5 deg back to 0 in 2 s instead of 1 s).
			float predW = (predT > 0f) ? Mathf.Clamp01((num22 * predT - 0.4f) / 0.6f) : 0f;
			float slipPred = (predW > 0f) ? (predW * SlipHistSum(predT)) : 0f;
			slipPredDbg = slipPred;
			// Ifrit 0.3.7: the sideslip hold fades out at very low TAS (Plugin.SlipHoldTasMin -> +SlipHoldTasSpan) and in true
			// reversed flow (|AoA| 100 -> 120 deg); the pedal sideslip command stays, as a plain yaw-rate command. 20 Oct 23:42
			// log, MPO: tail slide at -172 deg AoA, 20-33 m/s, sticks centred: the hold (high-AoA gain 4, it ramps on |AoA|)
			// asked 25-36 deg/s of yaw against a 7 deg/s limit (yaw authority ~5 deg/s^2) and bang-banged on the unstable
			// backwards weathervane, sideslip +6.6 -> -9.8 -> +10 deg on a ~5 s cycle. 391-400 s, 8-17 m/s, full lateral stick:
			// sideslip 52 deg, the hold saturated at 27.9 deg/s, and the joint roll/yaw limit scaling (num26) took the roll
			// command from -5 to ~0 deg/s: roll stick dead. At those speeds the sideslip is not a usable signal.
			float holdK = latHoldK;
			slipHoldDbg = holdK;
			// Revoker 0.2.2 (CorkscrewSlipTol): with roll stick held at high AoA and the pedals centred, the sideslip hold ignores
			// sideslip inside a tolerance band, so the corkscrew keeps rotating instead of the hold taking the yaw back as soon as
			// gravity (g/V ~19 deg/s at 30 m/s) pushes the sideslip out. In-game 28 Oct 19:05 log, 70 deg AoA, 30-50 m/s, full
			// roll: sideslip 5-11 deg, rS 16-38 deg/s, body yaw command -58 -> -12 deg/s, velocity-vector roll 63 -> 13 deg/s.
			// The band opens at once and closes at CorkTolClose deg/s after the stick is released or the AoA drops; the
			// rolling-pull sideslip governor (PullSideslipMargin) still bounds the sideslip.
			float corkT = (Mode == FcsMode.AoAG && !isAI) ? (Plugin.CorkscrewSlipTol * Mathf.Clamp01((Mathf.Abs(sr) - 0.2f) / 0.5f) * Mathf.InverseLerp(CorkAoA0, CorkAoA0 + 15f, Mathf.Abs(alpha)) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f))) : 0f;
			corkTolNow = (corkT >= corkTolNow) ? corkT : Mathf.MoveTowards(corkTolNow, corkT, CorkTolClose * dt);
			float bHold = beta - slipPred;
			if (corkTolNow > 0f)
			{
				bHold = Mathf.Sign(bHold) * Mathf.Max(0f, Mathf.Abs(bHold) - corkTolNow);
			}
			float num23 = (rSDbg = rSF.Step(Mathf.Clamp(num22 * (holdK * bHold - num12), 0f - slipLim, slipLim) + num21, 0.12f, dt));
			// Su-30 0.2.0 (StopIgnoresSlipOutsideMargin): the sideslip hold stands aside in the full-rate stop. It rolls the jet into the
			// sideslip to take it out, and that roll is what keeps the rotation going: bench, roll released at 70 deg once the sideslip had
			// passed the margin, 200 / 250 km/h: stop 3.05 / 3.27 s with it, 2.12 / 2.27 s without (sideslip 13.6 / 15.1 -> 20.6 / 25 deg)
			num23 *= 1f - stopHardW;
			// Su-30 0.1.0 (MpoTvcAxisRoll): in MPO past the AoA limit the stick's roll turns about the axis the canted nozzle differential
			// turns the jet about (the direction of the angular acceleration it makes, inertia included, measured from its moment table every
			// tick), blended in over MpoTvcAxisBlend deg past the limit; the sideslip hold fades out with it (that rotation is not about
			// the velocity vector, so it builds sideslip by design). Below the limit, and in every other mode: velocity-vector roll.
			float cAx = num;
			float sAx = num2;
			mpoAxisW = 0f;
			if (((Mode == FcsMode.MPO && Plugin.MpoTvcAxisRoll) || lowSpeedAxisW > 0f) && !isAI && canted != null && tvcd != null && tvcd.active)
			{
				float w = 0f;
				// Su-30 0.1.16: with the stiffness probe running, MPO's nozzle-axis roll comes from the stiffness gate (lowSpeedAxisW) and no
				// longer from the AoA limit: above the limit with the airframe still stiff (fast), velocity-vector roll
				if (Mode == FcsMode.MPO && Plugin.MpoTvcAxisRoll && !(Plugin.RollStopSlipProbe && slipSignR != 0f))
				{
					float lim = (alpha >= 0f) ? Plugin.AoAPos : (0f - Plugin.AoANeg);
					float over = Mathf.Abs(alpha) - Mathf.Abs(lim);
					w = Mathf.Clamp01(over / Plugin.MpoTvcAxisBlend);
					w = w * w * (3f - 2f * w);
				}
				// Su-30 0.1.11 (LowSpeedTvcRoll): too slow for the airframe to hold sideslip: the same blend, from the IAS, in the AoA law and in MPO
				w = Mathf.Max(w, lowSpeedAxisW);
				if (w > 0f && TvcAxis(out float thAx))
				{
					float ang = Mathf.Lerp(f, thAx, w);
					cAx = Mathf.Cos(ang);
					sAx = Mathf.Sin(ang);
					num23 *= 1f - w;
					mpoAxisW = w;
					mpoAxisDeg = thAx * 57.29578f;
				}
			}
			float num24 = pS * cAx - num23 * num2;
			float num25 = pS * sAx + num23 * num;
			float pLimSc = Mathf.Max(num3, Mathf.Min(pSlipMax, Mathf.Abs(num23 * num2)));
			// Su-30 0.1.11 (LowSpeedTvcRoll): about the nozzle axis the jet turns on the nozzles' own coupled roll and yaw, so the body roll
			// may be as fast as the yaw (the surfaces' roll-stop limit is ~13 deg/s here and would scale the whole rotation down)
			if (lowSpeedAxisW > 0f)
			{
				pLimSc = Mathf.Max(pLimSc, Mathf.Lerp(pLimSc, num8, lowSpeedAxisW));
			}
			float num26 = Mathf.Max(1f, Mathf.Max(Mathf.Abs(num24) / pLimSc, Mathf.Abs(num25) / num8));
			pCmd = num24 / num26;
			rCmd = num25 / num26;
			// Su-30 0.3.0 (MpoAxisReleaseStop, Tom 10 Nov 16:46: "when i release control, a lot of the time the fcs refuses to stop it, since the
			// rate were mostly yaw in those events, so can u just make it to stop faster and care less about induced uncommanded roll"). About the
			// nozzle axis the commands stay on that axis (r = pS sin th), so a release asked the roll loop to keep the axis's roll going while
			// the yaw loop braked the yaw, and the one canted nozzle differential did both: 16:16 log, MPO at 105-140 deg AoA, 90-170 km/h,
			// released with r 57-73 deg/s and p -2..+9: the commands wound down at the trajectory's 20 deg/s^2 floor (pCmd 36 -> 29, rCmd
			// 22 -> 16 in 0.5 s) and the yaw took 3-5 s to fall under 10 deg/s. Now, released (stick and pedals under 0.05) in the axis
			// rotation: the yaw command is zero at once (the loop brakes the yaw with all it has) and the roll command is the jet's own roll
			// while the yaw is over 30 deg/s, faded to zero by 10 deg/s, so the nozzles' roll is not fought until the yaw is down.
			// Su-30 0.3.4 (Tom 10 Nov 20:06: "the yaw stop at past max aoa is not fully working. Sometimes the yaw still keep going. I want it to
			// stop even if it cause some roll"): the stop also applies whenever the MPO law flies past the AoA limit (the soft hand-over
			// included), not only in the nozzle-axis band. 19:37 log, seven releases past the limit with 50-58 deg/s of yaw: five in the
			// soft hand-over with mpoAxisW 0-0.2, where the yaw command stayed at 36-50 deg/s (at 85-88 deg AoA the released velocity roll is
			// body yaw, and it wound down at the 4 deg/s^2 the rudder-blanked yaw estimate allowed): three never stopped before the stick came
			// back (135, 540, 924 s), two took 2.3 s. Weight: full from the limit, faded in over the 3 deg below it.
			axRelDbg = 0f;
			float relPastW = 0f;
			if (Mode == FcsMode.MPO && MpoRelPastLimit)
			{
				float limRel = (alpha >= 0f) ? Mathf.Min(Plugin.AoAPos, Mathf.Min(asymCapPosEff, authCapEff)) : Mathf.Min(Mathf.Abs(Plugin.AoANeg), Mathf.Abs(authCapNegEff));
				relPastW = Mathf.Clamp01((Mathf.Abs(alpha) - (limRel - 3f)) / 3f);
			}
			float kRelIn = Mathf.Max(mpoAxisW, relPastW);
			if (Plugin.MpoAxisReleaseStop && kRelIn > 0f && !isAI && Mathf.Abs(sr) < 0.05f && Mathf.Abs(sy) < 0.05f)
			{
				float kRel = kRelIn;
				float rNow = omega.z * 57.29578f;
				float pNow = omega.x * 57.29578f;
				float yawLeftR = Mathf.Clamp01((Mathf.Abs(rNow) - 10f) / 20f);
				rCmd = Mathf.Lerp(rCmd, 0f, kRel);
				pCmd = Mathf.Lerp(pCmd, yawLeftR * pNow, kRel);
				axRelYawLeft = yawLeftR;
				// keep the trajectory on the rotation that is left, so a new stick input starts from it
				pTraj.Reset(Mathf.Lerp(pTraj.x, pCmd * cAx + rCmd * sAx, kRel));
				axRelDbg = kRel;
			}
			// Su-30 0.3.2 (LowSpeedSlipTvc; Tom 10 Nov 16:46: in very low-speed corkscrews a lot of sideslip builds, let the TVC correct some of
			// it; 17:58: rather than a plain switch, the FCS should see when it does not have the authority and then not try). When the airframe
			// is too slow to hold sideslip itself (the probe's directional stiffness under LsSlipStiffStart 0.8, full at 0.4, 0.3 s filter;
			// without the probe IAS under LowSpeedTvcRollStartKmh, full at ...FullKmh) and the sideslip is past LowSpeedSlipTvcStart:
			// - the command gets stability-axis yaw dRs = sgn(beta) x gain x (|beta| - start) (beta_dot ~ -r_s, r_s = r cos a - p sin a),
			//   added along (-sin a, cos a) - no velocity-vector roll is added;
			// - the allocator's velocity-normal weight goes to LsSlipYawWeight (40) and its velocity-axis weight to LsSlipVvWeight (30): the
			//   sideslip is chased only with effector combinations that do not buy it with velocity-vector roll (the canted nozzles' cheap
			//   side effect at 45-75 deg AoA, which pumps the AoA through -pS tan(beta): 858 s replay, AoA 77 -> 90-120);
			// - authority check: m = sgn(beta) r_s is the correction's progress (m > 0 takes sideslip out). If after LsGateT (0.15 s) m is
			//   still negative and has not risen since the correction came in, the effectors cannot turn it; the correction fades out
			//   (LsGateFade 6 /s) and stays out until the sideslip is back under the start.
			// AoA law, forward flow, faded out from 75 to 85 deg AoA. Bench (16:16 log, 8 low-speed sideslip events x 6 replay start offsets):
			// peak sideslip -2.4 deg on average, no event worse on average, 3 of 48 runs more than 3 deg worse, none more than 3.4 deg of AoA
			// over the uncorrected run. Without the check the same correction departed in 3 of 48 (AoA to 121).
			lsSlipDbg = 0f;
			lsSlipBodyW = 0f;
			// Su-30 0.3.6 (LsSmooth, Tom 10 Nov 23:22: "maybe the correction is better but it is not a smooth ride when that happen"): the
			// correction is computed as a target every tick (zero when it is not wanted) and both outputs follow it through their own lags,
			// so it fades in and out whatever switches it: the stability-yaw command through LsCmdTau (0.25 s), the allocator's raised
			// weights through LsWeightTau (0.4 s - the weights step re-shuffled every effector at once); the onset is a smoothstep over
			// LsOnsetSpan (6 deg past the start, was a ramp over 3); the authority cut fades over ~0.4 s (smoothstep, was 0.17 s linear)
			// and re-arms only LsRearmMargin (2 deg) under the start. 0.3.5 bench (48 runs): with it on the roll command's jitter was +70 %.
			float lsRearm = Plugin.LowSpeedSlipTvcStart - (LsSmooth ? LsRearmMargin : 0f);
			if (Mathf.Abs(beta) < lsRearm)
			{
				lsGateOff = false;
				lsCorrT = 0f;
			}
			float dRsT = 0f;
			float wCT = 0f;
			if (Plugin.LowSpeedSlipTvc && Mode == FcsMode.AoAG && !isAI && canted != null && tvcd != null && tvcd.active && Mathf.Abs(alpha) < 85f && !noTvcOn)
			{
				float wS;
				if (Plugin.RollStopSlipProbe && slipSignR != 0f)
				{
					lsSlipStiffF += (Mathf.Max(0f, slipStiffDbg) - lsSlipStiffF) * Mathf.Clamp01(dt / 0.3f);
					wS = Mathf.Clamp01((LsSlipStiffStart - lsSlipStiffF) / Mathf.Max(0.01f, LsSlipStiffStart - LsSlipStiffFull));
				}
				else
				{
					wS = Mathf.Clamp01((Plugin.LowSpeedTvcRollStart - ias * 3.6f) / Mathf.Max(1f, Plugin.LowSpeedTvcRollStart - Plugin.LowSpeedTvcRollFull));
				}
				wS = wS * wS * (3f - 2f * wS);
				wS *= 1f - Mathf.Clamp01((Mathf.Abs(alpha) - 75f) / 10f);
				float bEx = Mathf.Max(0f, Mathf.Abs(beta) - Plugin.LowSpeedSlipTvcStart);
				if (wS > 0f && bEx > 0f)
				{
					float mNow = Mathf.Sign(beta) * (omega.z * Mathf.Cos(f) - omega.x * Mathf.Sin(f)) * 57.29578f;
					if (lsCorrT <= 0f)
					{
						lsM0 = mNow;
					}
					lsCorrT += dt;
					lsProgDbg = mNow - lsM0;
					if (LsGate && lsCorrT > LsGateT && mNow < 0f && mNow - lsM0 < 0f)
					{
						lsGateOff = true;
					}
					float onset = LsSmooth ? Mathf.Clamp01(bEx / Mathf.Max(0.5f, LsOnsetSpan)) : Mathf.Clamp01(bEx / 3f);
					if (LsSmooth)
					{
						onset = onset * onset * (3f - 2f * onset);
					}
					wCT = wS * onset;
					dRsT = Mathf.Sign(beta) * Mathf.Min(Plugin.LowSpeedSlipTvcGain * bEx, Plugin.LowSpeedSlipTvcMax);
				}
			}
			lsGateW = Mathf.MoveTowards(lsGateW, lsGateOff ? 0f : 1f, (LsSmooth ? LsGateFadeSmooth : LsGateFade) * dt);
			float gateS = LsSmooth ? (lsGateW * lsGateW * (3f - 2f * lsGateW)) : lsGateW;
			wCT *= gateS;
			float dRsTgt = dRsT * wCT;
			if (LsSmooth)
			{
				float tauC = (Mathf.Abs(dRsTgt) > Mathf.Abs(lsDRsF)) ? LsCmdTauUp : LsCmdTau;
				float tauW = (wCT > lsBodyWF) ? LsWeightTauUp : LsWeightTau;
				lsDRsF += (dRsTgt - lsDRsF) * Mathf.Clamp01(dt / Mathf.Max(dt, tauC));
				lsBodyWF += (wCT - lsBodyWF) * Mathf.Clamp01(dt / Mathf.Max(dt, tauW));
			}
			else
			{
				lsDRsF = dRsTgt;
				lsBodyWF = wCT;
			}
			if (Mathf.Abs(lsDRsF) > 0.01f || lsBodyWF > 0.001f)
			{
				pCmd += -lsDRsF * Mathf.Sin(f);
				rCmd += lsDRsF * Mathf.Cos(f);
				lsSlipDbg = lsDRsF;
				lsSlipBodyW = lsBodyWF;
			}
			// 2.20.0 (PaceStopNoFin): in the no-fin stop the yaw loop is asked for its brake capacity (the error that makes it, capacity /
			// KYaw, from the measured yaw), where the stopped-roll target alone gives it a fraction of that
			paceStopNFDbg = paceStopNF;
			if (paceStopNF != 0f)
			{
				float capB = Mathf.Max(PaceHoldCapFloor, (paceStopNF > 0f) ? capEst.capZeN : capEst.capZeP);
				float rNF = omega.z * 57.29578f - paceStopNF * capB / Mathf.Max(0.5f, Plugin.KYaw);
				float finKnf = Mathf.Clamp01(yawHealth / 0.5f);
				if ((rNF - rCmd) * paceStopNF < 0f)
				{
					rCmd = Mathf.Lerp(rNF, rCmd, finKnf);
				}
			}
			// 2.4.17 no-chase on a roll brake: while the trajectory brakes the roll (stop, reversal, or a limit coming down), the
			// yaw loop is not asked for more yaw rate than the jet already has on the same side. In a fast roll at AoA the yaw
			// trails its coordinated command (the rudders at their stop: 30 Oct 14:35 log, 267.7 s, r -27 against -36); left to
			// chase it, the yaw kept accelerating for 0.3 s after the stick came back (bench: -33 -> -43 deg/s), and all of that
			// had to be braked again by rudders slewing across their travel. Same AoA band as the roll-stop governor.
			float rCmdCoord = rCmd;
			noChaseDbg = 0f;
			// 2.4.18: only while the coordinated yaw command itself is shrinking. Under a rolling pull the trajectory brakes the
			// roll (the pull governor / trade) while the rising AoA raises the yaw the roll needs (pS sin AoA); holding the yaw there
			// built sideslip (bench, full roll then full aft at 400 km/h: 2.9 -> 4.9 deg in 2.4.17)
			// rate of the yaw the roll itself needs, |pS| sin|AoA|, from the trajectory's own acceleration and the AoA rate
			// (or the AoA not rising fast: at a steady AoA the braking trajectory alone sets it, from its first tick)
			// 2.5.15: retired with the pace governor at level 3 (bench at blend 0.41 / margin 4: stops 5.2 -> 4.1 worst, reversals
			// 8.8 -> 9.3, regressions mostly down 0.2-4 deg, one up 1.6; a wash - one device fewer)
			pDotFF = pTraj.v * cAx / num26;
			SlipHistPush((num23 / num26 - num21) * predGate, dt);
			float num27 = aRoll / Mathf.Max(30f, pFull);
			pCmd += Plugin.RollLead * Mathf.Max(0f, num27 - 2.5f) * pDotFF;
			float t2 = Mathf.Max(Mathf.InverseLerp(200f, 300f, ias), Mathf.InverseLerp(YawFFAoA0, YawFFAoA1, Mathf.Abs(alpha)));
			// 2.4.17: the yaw feed-forward follows the coordinated command, not the no-chase hold (its release would step it)
			rDotFF = Mathf.Lerp(Plugin.YawRateFF, 1f, t2) * rDotF.Step((rCmdCoord - rCmdCoordPrev) / dt, 0.1f, dt);
			rCmdCoordPrev = rCmdCoord;
			pCmdPrev = pCmd;
			// 2.4.18: the rate of the yaw-rate command the loop actually follows (after the no-chase hold), for the yaw slew lead
			rCmdRateF += ((rCmdLeadInit ? ((rCmd - rCmdPrev) / Mathf.Max(dt, 1E-04f)) : 0f) - rCmdRateF) * (dt / (0.05f + dt));
			rCmdLeadInit = true;
			rCmdPrev = rCmd;
			if (Mathf.Abs(omega.z * 57.29578f) > num8 && rDotFF * omega.z > 0f)
			{
				rDotFF = 0f;
			}
		}

		public void WriteHeader(TextWriter w)
		{
			w.Write("t,mode,ias,tas,rho,alpha,beta,nz,nAlpha,tw,p,q,r,sp,sr,sy,qB,qRaw,qCmd,qT,pCmd,rCmd,aUp,aDn,betaG,alphaCmd,nCmd,unload,prot,omegaV,phi,theta,tvcCmd,tvcNow,nMaxAero,alphaAtNMax,gAvail,gInt,aInt,flapBias,flapTarget,hlPos,relief,rudderAuth,yawSat,aRoll,aYaw,pMax,pStockRef,pFull,rMax,guard,pSmax,pSTarget,latAcc,rS,stabPitchFrac,gndBrake,revFlow,asymDy,asymAr,asymAy,asymU,asymCap,asymCapEff,asymCapNeg,dMeasR,dMeasY,asymCr,asymCy,asymFFr,asymFFy,asymMr,asymMy,asymFFGate,rollPitchFF,gUnder,pitchPrio,stabRateBound,momentLead,rollTrim,aRollPos,aRollNeg,latReserve,aUpPitch,aDnPitch,aBrakeTgt,stabBranch,tvcAnchor,tvcBudget,rudSide,gsRCmd,gsNws,gsRud,gsHold,gsLineY,asymCapPitch,pitchNeed,coupRollLim,rpGate,rpBand,rollGov,rollGovMargin,iasDot,qPredK,asymCapRaw,asymCapFilt,alphaCmdRaw,nCmdRaw,rateBlendAoA,hiRate,rateW,zoneX,authCap,authCapEff,nLiftMax,hybridDMacc,floatOn,floatS,rateBlendAoANeg,nLiftMin,hiSide,authCapNeg,vmcLim,engOut,fuelTrimCm,yawHealth,capIUp,capIDn,capPushU,slipRate,slipGov,yawAnchor,gRateLaw,rateCmdG,negGate,sharedLat,rollGovPred,loadRev,stickLead,capOn,capQ,sweepLim,slatRelief,hlPosNow,slipAlloc,coupGate,pitchLead,slipGovPull,decel,pullTrade,abShare,abTgt,abOpen,fightK,pullFFBoost,tvcdCmd,tvcdNow,splitL,splitR,aRollTvc,pSlipMax,slipGain,droopIn,droopOut,revSense,slipPred,slipHold,splitTier,tvcHold,qTrajV,fightGate,abGate,engAbRatio,engMatchLoss,splitUsage,settledWin,asymThrY,asymLoadY,massKg,nx,capStart,capFlat,capUMin,capULim,gsWheel,gsIndi,gsRdot,gsNu,gsFyK,tvcyCmd,tvcyNow,tvcyHold,canL,canR,flapHL,flapHR,latLag,slipLead,stopSlipLim,stopSlipAB,stopSlipU,stopSlipPred,stopSlipMargin,stopSlipHold,yawSatPitch,yawLead,paceLead,paceHold,slipTol,slipDep,slipAuth,slipStiff,paceRel,tradeAlpha,slipDamp,dampCredit,yawCap,rollGate,rateCap,revSimBeta,revSimT,paceReach" + CapacityEstimator.Header() + ",alphaRoot,betaRoot,airRefCg,alphaCg,betaCg,nuR,nuY,anchR,anchY,unR,unY,gUnmetX,gUnmetZ,paceGateZ,paceGateX,paceMLeft,paceCapZP,paceCapZN,paceCapXP,paceCapXN,capSwp,capShd,aYawShed,ppOn,ppQ,ppReach,rateCapLag,axWr,axWp,axWy,asymPitchBias,asymPitchBiasNeg,asymCapPitchNeg,dmgGate,flapYield,flapSymTgt,vmcPred,vmcPredUse,mpoAxisW,mpoAxisDeg,nozL,nozR,cantVvW,rudShare,rateBlendEff,landFlap,canPitchFirst,leafBoost,aYawTvc,yawThrustShare,surfFirst,canPeakLo,canPeakHi,gcReach,gyroQLim,capTauDn,capTauUp,capBrkDn,capBrkUp,trajTau,softMpo,softPitch,stopHard,stopHardPred,axRel,lsSlip,gCapT,gCapB,gCapE,fcsUs,estWaitUs,estThr,tRebUs,tAllocUs,tKickUs,tRowUs,tLawUs,tAsymUs,tAsymMomUs,tSlipUs,tSweepUs,tEnvUs,tMassUs,tFlapUs,tGndUs,tVmcUs,sgJoin,sgMass,sgAir,sgFlap,sgSurf,sgTab,sgPre,sgAsym,sgMode,sgGnd,sgLaw,sgApply,sgRow,heapB,gcRan,swMask,cplU,cplRp,cplRn,cplDn,pitchMarg,stopRateP,stopRateN,cplRef,gsBrkL,gsBrkR,gsBsL,gsBsR,gsLn");
			foreach (SurfaceEffector surface in surfaces)
			{
				w.Write("," + surface.name + "_cmd," + surface.name + "_now," + surface.name + "_aoaNow," + surface.name + "_aoaCmd");
			}
			w.WriteLine();
		}

		// Su-30 0.2.0 (from F-22E 3.0.4): WriteRow split in two. CaptureRow runs at the end of tick N on the main thread and boxes every
		// value, in the same order and with the same format strings as WriteRow did; the estimator's columns and the reach count (which
		// includes the probes' re-solves) are slots that FinishRow fills at the top of tick N+1, once the estimator job of tick N is
		// joined. The text is made by TelemetryRow.WriteTo on the writer thread: string.Format(InvariantCulture, fmt, args) per part.
		private TelemetryRow CaptureRow(float t, float spv, float srv, float syv, float rho)
		{
			TelemetryRow r = new TelemetryRow();
			r.Add("{0:F3},{1},{2:F1},{3:F1},{4:F3},{5:F2},{6:F2},{7:F2},{8:F3},{9:F2},{10:F1},{11:F1},{12:F1},{13:F2},{14:F2},{15:F2},{16:F1},{17:F1},{18:F1},{19:F1},{20:F1},{21:F1},{22:F0},{23:F0},{24:F2},{25:F1},{26:F2},{27},{28},{29:F1},{30:F1},{31:F1},{32:F2},{33:F2}", t, (int)Mode, ias, tas, rho, alpha, beta, nz, nAlpha, tw, omega.x * 57.29578f, omega.y * 57.29578f, omega.z * 57.29578f, spv, srv, syv, qB, qRaw, qCmd, qT, pCmd, rCmd, aUp, aDn, betaG, alphaCmd, nCmd, unload ? 1 : 0, prot ? 1 : 0, omegaV, phi, theta, tvc.cmd, tvc.now);
			r.Add(",{0:F2},{1:F0},{2:F2},{3:F2},{4:F2},{5:F1},{6:F1},{7:F2},{8:F1},{9:F2},{10:F2},{11:F0},{12:F0}", nMaxAero, alphaAtNMax, gAvail, gInt, aInt, flaps.flapBias, flaps.flapTarget, flaps.hlPos, flaps.relief, rudderAuth, yawSat, aRoll, aYaw);
			r.Add(",{0:F0},{1:F0},{2:F0},{3:F0},{4:F2},{5:F0},{6:F0},{7:F0},{8:F1},{9:F2}", pMaxDbg, pStockRef, pFull, rMaxDbg, guardDbg, pSmaxDbg, pSTgtDbg, latAccDbg, rSDbg, stabPitchFrac);
			r.Add(",{0:F2},{1}", groundBrake, revFlow ? 1 : 0);
			r.Add(",{0:F3},{1:F1},{2:F1},{3:F2},{4:F1},{5:F1},{6:F1},{7:F1},{8:F1}", asymDy, asymArNow, asymAyNow, asymUNow, Mathf.Min(asymCapPos, 999f), Mathf.Min(asymCapPosEff, 999f), Mathf.Max(asymCapNegEff, -999f), dMeasR, dMeasY);
			r.Add(",{0:F0},{1:F0},{2:F1},{3:F1},{4:F1},{5:F1}", asymCrNow, asymCyNow, asymFFr, asymFFy, AccRoll(asymMNow), AccYaw(asymMNow));
			r.Add(",{0:F2}", asymFFGate);
			r.Add(",{0:F1},{1:F1}", rollPitchFF * IaInv[1, 1] * 57.29578f, gUnderDbg);
			r.Add(",{0:F1},{1},{2:F2},{3:F0},{4:F0},{5:F0},{6:F2},{7:F0},{8:F0},{9:F0},{10}", pitchPrio, stabRateBound ? 1 : 0, mLeadGate * mLeadAgree, dTrimR, aRollPos, aRollNeg, latReserve, aUpPitch, aDnPitch, aBrakeTgt, stabBranch);
			r.Add(",{0:F2},{1:F2},{2:F1}", tvcAnchor, tvcBudgetGate, rudSideDbg);
			r.Add(",{0:F2},{1:F2},{2:F2},{3},{4:F2},{5:F1},{6:F2},{7:F0}", gsRCmd, gsNwsDeg, gsRud, gsHold ? 1 : 0, gsLineY, Mathf.Min(asymCapPitch, 999f), pitchNeed, coupRollLim);
			r.Add(",{0:F2},{1:F1},{2:F2},{3:F2}", rpGate, rpBand, rollGov, rollGovMargin);
			r.Add(",{0:F2},{1:F3},{2:F1},{3:F1}", iasDotF, qPredK, Mathf.Clamp(asymCapPos, -99f, 99f), Mathf.Clamp(asymCapFilt, -99f, 99f));
			r.Add(",{0:F1},{1:F2},{2:F1},{3},{4:F2},{5:F2},{6:F1},{7:F1},{8:F2},{9:F1}", alphaCmdRaw, nCmdRaw, rateBlendAoA, hiRate ? 1 : 0, rateW, zoneX, Mathf.Min(authCap, 99f), Mathf.Min(authCapEff, 99f), nLiftMax, hybridDM * IaInv[1, 1] * 57.29578f);
			r.Add(",{0},{1:F2}", floatOn ? 1 : 0, floatS);
			r.Add(",{0:F1},{1:F2},{2},{3:F1}", rateBlendAoANeg, nLiftMin, hiRate ? hiSide : 0, Mathf.Max(authCapNegEff, -99f));
			r.Add(",{0:F2},{1},{2:F1},{3:F2}", vmcLim, engOut ? 1 : (engAsym ? 2 : 0), fuelTrimOn ? (fuelTrimDelta * 100f) : 0f, yawHealth);
			r.Add(",{0:F2},{1:F2},{2:F2}", capIUp, capIDn, capPushU);
			r.Add(",{0:F1},{1:F2},{2:F1},{3},{4:F1}", slipRateDbg, slipGovDbg, Mathf.Clamp(yawAnchorDbg, -999f, 999f), Plugin.GRateLaw ? 1 : 0, rateCmdG);
			r.Add(",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5},{6:F1},{7:F1},{8:F2},{9:F2},{10:F2},{11:F2}", negGateDbg, sharedLatDbg, rollGovPredDbg, loadRevDbg, stickLeadDbg, capOn ? 1 : 0, capQ, sweepLimDbg, slatRelief, flaps != null ? flaps.hlPos : 0f, (YawPrioHi > 0f && Plugin.SlipYawWeightOn) ? YawHiSlipGate() * Mathf.Clamp01((Mathf.Abs(alpha) - 20f) / 20f) : 0f, coupGateHeld);
			r.Add(",{0:F3},{1:F2},{2:F2},{3:F0},{4:F3},{5:F2},{6:F2},{7:F2},{8:F2}", pitchLeadDbg, slipGovPullDbg, decelDbg, pullTradeDbg, airbrakeShare, AirbrakeOpenTarget, airbrakeOpenNow, alloc.fightK, pullFFBoost);
			// Ifrit 0.3.0: roll TVC, the split decelerons (physical deg), the sideslip assist and the flaperon droop
			float spLog = 0f;
			float spRog = 0f;
			foreach (SplitEffector sLg in splits)
			{
				float ph = sLg.Phys(sLg.now);
				if (sLg.name.EndsWith("_L"))
				{
					spLog = ph;
				}
				else
				{
					spRog = ph;
				}
			}
			r.Add(",{0:F2},{1:F2},{2:F1},{3:F1},{4:F0},{5:F0},{6:F2},{7:F1},{8:F1}", (tvcd != null) ? tvcd.cmdOut : 0f, (tvcd != null) ? tvcd.now : 0f, spLog, spRog, aRollTvc, pSlipMax, slipGainDbg, (flaps != null) ? flaps.innerDroop : 0f, (flaps != null) ? flaps.outerDroop : 0f);
			r.Add(",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F1},{6:F2},{7:F2},{8:F2},{9:F2},{10:F2},{11}", revSenseDbg, slipPredDbg, slipHoldDbg, splitTierDbg, tvcNeutralDbg, qTraj.v, fightGateDbg, abGateOn ? ((engOut || matchF <= 0f) ? 1f : matchF) : 0f, engAbRatio, engMatchLoss * 0.001f, splitUsage, settledWinDbg ? 1 : 0);
			float thrY = (tvc != null && tvc.active) ? AccYaw(((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : ((tvc != null) ? AccYaw(FixedThrustLat()) : 0f);
			r.Add(",{0:F2},{1:F2},{2:F1},{3:F3}", thrY, dMeasY + thrY, mass, nxDbg);
			r.Add(",{0:F1},{1:F1},{2:F2},{3:F2}", AsymCap.DbgStart, AsymCap.DbgFlat, AsymCap.DbgUMin, AsymCap.DbgULim);
			r.Add(",{0:F2},{1},{2:F1},{3:F1},{4:F2}", gsWheelDbg, gsIndiOn ? 1 : 0, gsRdotF, gsNuDbg, gsFyF * 0.001f);
			// Revoker 0.1.0: yaw nozzle command / angle / neutral-hold weight, and the two canards' deflection
			r.Add(",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2}", (tvcy != null) ? tvcy.cmdOut : 0f, (tvcy != null) ? tvcy.now : 0f, tvcYawNeutralW, (surfaces.Count > 0) ? surfaces[0].now : 0f, (surfaces.Count > 1) ? surfaces[1].now : 0f);
			r.Add(",{0:F2},{1:F2}", (hinges.Count > 0) ? hinges.Find((HingeFlapEffector h) => h.left)?.hingeNow ?? 0f : 0f, (hinges.Count > 0) ? hinges.Find((HingeFlapEffector h) => !h.left)?.hingeNow ?? 0f : 0f);
			// Revoker 0.2.15: the lateral loops' answer lag (s) and the governor's look-ahead (s)
			r.Add(",{0:F3},{1:F3}", latLagEff, slipLeadDbg);
			r.Add(",{0:F0},{1:F0},{2:F2},{3:F2},{4:F1},{5:F2},{6:F1},{7:F3}", Mathf.Min(stopSlipLimDbg, 999f), stopSlipABDbg, stopSlipUDbg, stopSlipPredDbg, stopSlipMarginDbg, stopSlipHoldDbg, yawSatPitchDbg, yawLeadDbg);
			r.Add(",{0:F1},{1:F1},{2:F1},{3:F1},{4:F1},{5:F2},{6:F2},{7:F1}", paceLeadDbg, paceHoldDbg, slipTolDbg, slipDepDbg, slipAuthDbg, slipStiffDbg, paceRelDbg, tradeAlphaDbg);
			r.Add(",{0:F2},{1:F0}", slipDampDbg, dampCredit);
			r.Add(",{0:F0},{1:F2}", yawCapDbg, rollLoadDbg);
			r.Add(",{0:F0},{1:F2},{2:F2},{3:F1}", Mathf.Min(dbgRateCap, 9999f), revSimBetaDbg, revSimTDbg, paceReachDbg);
			r.ceA = r.Slot();   // the estimator's columns (FinishRow)
			r.Add(",{0:F2},{1:F2},{2},{3:F2},{4:F2}", alphaRoot, betaRoot, AirDataAtCg ? 1 : 0, alphaCgDbg, betaCgDbg);
			// Revoker 0.8.2: what the roll / yaw rate loops ask for (deg/s^2; ~0 with the stick centred once the jet holds its command),
			// how far the tables' roll / yaw columns were off their live moment this tick before anchoring (deg/s^2, summed), and what
			// the allocation left unmade of its target (deg/s^2, the main solve)
			{
				Vector3 anchSum = Vector3.zero;
				foreach (SurfaceEffector sA in surfaces)
				{
					if (sA.active)
					{
						anchSum += sA.anchorDbg;
					}
				}
				foreach (HingeFlapEffector hA in hinges)
				{
					if (hA.active)
					{
						anchSum += hA.anchorDbg;
					}
				}
				r.Add(",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2}", omegaDotDes.x * 57.29578f, omegaDotDes.z * 57.29578f, anchSum.x * IaInv[0, 0] * 57.29578f, anchSum.z * IaInv[2, 2] * 57.29578f, -allocResMain.x, -allocResMain.z);
			}
			// 0.6.0: the governor port's columns (F-22E 3.0.0): the allocator's unmet (filtered), the pace window's gates, margin left and
			// capacities, the push governor's caps, the push plan, the reach-pass re-solves, the rate cap's servo lag, the axis weights
			r.Add(",{0:F1},{1:F1},{2:F2},{3:F2},{4:F1},{5:F0},{6:F0},{7:F0},{8:F0}", allocUnmet.x, allocUnmet.z, paceGateZDbg, paceGateXDbg, paceMLeftDbg, paceCapZPDbg, paceCapZNDbg, paceCapXPDbg, paceCapXNDbg);
			// the reach count is filled in FinishRow (its total includes the worker's probe re-solves)
			r.reachSlot = r.Slot();
			r.reachArgs = new object[] { capSweepDbg, Mathf.Clamp(capShedDbg, -999f, 999f), aYawShedDbg, pushPlanOn ? 1 : 0, pushPlanOn ? pushPlanQNow : 0f };
			r.reachMain = alloc.reachMoves;
			r.Add(",{0:F2},{1:F2},{2:F2},{3:F2}", rateCapLagDbg, axWDbg.x, axWDbg.y, axWDbg.z);
			r.Add(",{0:F1},{1:F1},{2:F1},{3:F0},{4:F2},{5:F1},{6:F2},{7:F2}", asymPitchBias, asymPitchBiasNeg, Mathf.Max(asymCapPitchNeg, -999f), dmgGateHold, flapDiffYield, (hinges.Count > 0) ? hinges[0].symTarget : 0f, vmcPredLim, vmcPredUse);
			// Su-30 0.1.0: MPO roll-axis blend toward the TVC axis (0-1) and that axis (deg from body x toward yaw); each canted nozzle's deflection
			{
				float nL = 0f, nR = 0f;
				if (canted != null)
				{
					for (int iN = 0; iN < canted.dNow.Length; iN++)
					{
						if (canted.side[iN] > 0f) nL = canted.dNow[iN]; else nR = canted.dNow[iN];
					}
				}
				r.Add(",{0:F2},{1:F1},{2:F2},{3:F2}", mpoAxisW, mpoAxisDeg, nL, nR);
			}
			r.Add(",{0:F2}", cantVvWDbg);
			r.Add(",{0:F2},{1:F2},{2:F1},{3:F2},{4:F2},{5:F1},{6:F2}", rudShareDbg, rateBlendEffDbg, landFlapDbg, canPitchFirstDbg, leafBoostDbg, aYawTvc, yawThrustShareDbg);
			r.Add(",{0:F2},{1:F1},{2:F1},{3:F2},{4:F1}", surfFirstDbg, canPeakLo, canPeakHi, gcReachDbg, gyroQLimDbg);
			r.Add(",{0:F3},{1:F3},{2:F1},{3:F1}", capTauDnDbg, capTauUpDbg, capBrkDnDbg, capBrkUpDbg);
			r.Add(",{0:F3},{1},{6:F0},{2:F2},{3:F1},{4:F2},{5:F1}", pitchTrajTauDbg, softMpo ? 1 : 0, stopHardW, stopHardPredDbg, axRelDbg, lsSlipDbg, softPitchDbg);
			r.Add(",{0:F3},{1:F1},{2:F2}", gCapTDbg, gCapBDbg, gCapEDbg);
			r.timeSlot = r.Slot();   // F-22E 3.0.4 / 3.1.0: fcsUs, estWaitUs, estThr, tRebUs..tRowUs
			r.timeSlot2 = r.Slot();   // F-22E 3.1.1: tLawUs..tVmcUs
			r.timeSlot3 = r.Slot();   // F-22E 4.1.3 port: sgJoin..sgRow, heapB, gcRan, swMask
			bool cplRow = cplActive && cplLoaded;
			r.Add(",{0:F2},{1:F1},{2:F1},{3:F1},{4:F1}", cplRow ? cplUNow : 0f, cplRow ? cplRpNow : 0f, cplRow ? cplRnNow : 0f, cplRow ? Mathf.Clamp(cplDnNow, -999f, 999f) : 0f, asymPitchMargDbg);
			r.Add(",{0:F0},{1:F0}", Mathf.Min(cplStopP, 999f), Mathf.Min(cplStopN, 999f));
			r.Add(",{0:F1}", cplRow ? ((alpha >= 0f) ? cplRefPos : cplRefNeg) : 0f);
			// 0.5.3: the brake each main is asked for by the yaw effector (-1 = not in the loop), the brake it applies (lagged), and the
			// light-nose hold
			float bsL = 0f;
			float bsR = 0f;
			if (gsAnyContact)
			{
				Vector3 rightAx = ((Component)(object)ac).transform.right;
				foreach (LandingGear g in gears)
				{
					if (g != null && !Acc.LG_steering(g) && Acc.LG_braked(g))
					{
						if (Vector3.Dot(g.transform.position - cg, rightAx) < 0f)
						{
							bsL = Acc.LG_brakeStrength(g);
						}
						else
						{
							bsR = Acc.LG_brakeStrength(g);
						}
					}
				}
			}
			r.Add(",{0:F2},{1:F2},{2:F2},{3:F2},{4}", gsBrkDbgL, gsBrkDbgR, bsL, bsR, gsLnDbg ? 1 : 0);
			r.threaded = estJobF != null && estJobF.Pending;
			foreach (SurfaceEffector surface in surfaces)
			{
				r.Add(",{0:F2},{1:F2},{2:F1},{3:F1}", surface.cmd, surface.now, surface.localAoANow, surface.localAoACmd);
			}
			return r;
		}

		// the CSV row of the tick just run, written at once (the estimator joined first): the bench's synchronous log, and the text the
		// writer thread would write for that tick
		public void WriteRow(TextWriter w, float t, float spv, float srv, float syv, float rho)
		{
			estJobF?.Join();
			TelemetryRow r = CaptureRow(t, spv, srv, syv, rho);
			FinishRow(r);
			r.WriteTo(w);
		}

		private void FinishRow(TelemetryRow r)
		{
			CultureInfo ci = CultureInfo.InvariantCulture;
			using (StringWriter sw = new StringWriter(ci))
			{
				capEst.Row(sw, ci);
				r.Fill(r.ceA, "{0}", sw.ToString());
			}
			// the main allocator's count at capture (tick N's real solves; in the synchronous path also its probes) + the worker's probes
			int reachTot = r.reachMain + ((estJobF != null) ? estJobF.WorkerReachMoves : 0);
			object[] ra = r.reachArgs;
			r.Fill(r.reachSlot, ",{0:F1},{1:F1},{2:F1},{3},{4:F1},{5}", ra[0], ra[1], ra[2], ra[3], ra[4], reachTot - reachLogPrev);
			reachLogPrev = reachTot;
			r.Fill(r.timeSlot, ",{0:F0},{1:F0},{2},{3:F0},{4:F0},{5:F0},{6:F0}", lastTickUs, r.threaded ? estJobF.lastWaitUs : 0f, r.threaded ? 1 : 0, TicksToUs(phRebuild), TicksToUs(phAlloc), TicksToUs(phKick), TicksToUs(phRow));
			r.Fill(r.timeSlot2, ",{0:F0},{1:F0},{2:F0},{3:F0},{4:F0},{5:F0},{6:F0},{7:F0},{8:F0},{9:F0}", TicksToUs(phLaws), TicksToUs(phAsym), TicksToUs(phAsymMom), TicksToUs(phSlip), TicksToUs(phSweep), TicksToUs(phEnv), TicksToUs(phMass), TicksToUs(phFlaps), TicksToUs(phGround), TicksToUs(phVmc));
			float[] g = segUs;
			r.Fill(r.timeSlot3, ",{0:F0},{1:F0},{2:F0},{3:F0},{4:F0},{5:F0},{6:F0},{7:F0},{8:F0},{9:F0},{10:F0},{11:F0},{12:F0},{13},{14},{15}", g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7], g[8], g[9], g[10], g[11], g[12], memDelta, gcDuring, SwitchMask());
		}
	}
}
