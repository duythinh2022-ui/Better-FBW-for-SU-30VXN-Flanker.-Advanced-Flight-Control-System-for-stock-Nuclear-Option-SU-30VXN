using System.Reflection;
﻿using System;
using System.Collections.Generic;
using BepInEx.Configuration;
using BepInEx.Logging;
using HarmonyLib;
using NuclearOption.Jobs;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// 1.8.43: one DLL for both builds. PerformanceProfile picks stock or buffed performance at runtime.
	public enum PerformanceProfile
	{
		Buffed,
		Stock
	}

	// Ifrit 0.3.1: lateral law with Stability Assist off (MPO)
	public enum MpoRollMode
	{
		VelocityVectorRoll,
		SeparateRollYaw
	}

	// 2.1.0: gear-up pitch command law
	public enum PitchCommandLaw
	{
		GAndPitchRate,
		GAoAAndPitchRate,
		AoAThenRateHold
	}

	// 2.0.8: debug AoA-limit test switch
	public enum AoALimitTest
	{
		Normal,
		Full65,
		Cap36
	}

	internal static class Plugin
	{
		internal static ManualLogSource Log;

		private static ConfigEntry<bool> cEnabled;

		private static ConfigEntry<PerformanceProfile> cProfile;

		private static ConfigEntry<bool> cHostAllowFcs;

		private static ConfigEntry<bool> cHostLock;


		private static ConfigEntry<bool> cTelemetry;

		private static ConfigEntry<bool> cAIFcs;

		private static ConfigEntry<bool> cAILog;

		private static ConfigEntry<bool> cAIDirectRoll;

		private static ConfigEntry<float> cStabUp;

		private static ConfigEntry<float> cStabDn;


		private static ConfigEntry<float> cFlapRange;

		private static ConfigEntry<float> cTvcRange;

		private static ConfigEntry<float> cTvcSpeed;

		private static ConfigEntry<bool> cAIGunRollFade;

		private static ConfigEntry<bool> cAsymOn;

		private static ConfigEntry<bool> cAsymRollHead;

		private static ConfigEntry<bool> cRevRoll;



		private static ConfigEntry<float> cAoAPos;

		private static ConfigEntry<float> cAoANeg;

		private static ConfigEntry<int> cAoANegRev;

		private static ConfigEntry<bool> cIfritSplit;

		private static ConfigEntry<float> cFlapCamber;

		private static ConfigEntry<float> cDownwash;

		private static ConfigEntry<bool> cDownwashLift;

		private static ConfigEntry<bool> cStrakeVortex;

		private static ConfigEntry<float> cStrakeBd0, cStrakeBd1, cStrakeRes;

		private static ConfigEntry<bool> cSlab;

		private static ConfigEntry<float> cSlabStall, cSlabSlope;

		private static ConfigEntry<int> cCompassRev, cCompassDevRev;

		private static ConfigEntry<float> cLerxEff;

		private static ConfigEntry<float> cFinCamber;

		private static ConfigEntry<float> cCgShift;

		private static ConfigEntry<float> cAftBodyLift;

		private static ConfigEntry<bool> cCapMeas;
		private static ConfigEntry<bool> cGCapMeas, cSoftLim, cStopHard, cTrajMeas;
		private static ConfigEntry<float> cSoftLimMargin;
		// Su-30 0.2.0: the F-22E 3.0.4-3.1.3 multithreading and timers
		private static ConfigEntry<bool> cEstThreaded, cTickTimer, cParTables, cParAlloc, cParSweep, cParEnv, cCgRef;
		public static bool Su30CgReference => cCgRef == null || cCgRef.Value;
		public static bool EstimatorThreaded => cEstThreaded == null || cEstThreaded.Value;
		public static bool TickTimerLog => cTickTimer == null || cTickTimer.Value;
		public static bool ParallelTables => cParTables == null || cParTables.Value;
		public static bool ParallelAllocator => cParAlloc == null || cParAlloc.Value;
		public static bool ParallelSweep => cParSweep == null || cParSweep.Value;

		// 0.4.0 (the F-22E 4.0.0 - 4.1.3 performance work): the native core, the sweep's geometry cache, the slow-tick log
		public static int NativeAllocator => (cNativeAlloc == null) ? 2 : cNativeAlloc.Value;

		public static int NativeAllocatorCheck => (cNativeAllocCheck == null) ? 0 : cNativeAllocCheck.Value;

		public static bool NativeSweep => cNativeSweep == null || cNativeSweep.Value;

		public static bool NativeCoupledCap => cNativeCpl == null || cNativeCpl.Value;

		public static bool NativeTables => cNativeTables == null || cNativeTables.Value;

		public static bool SweepGridCache => cSweepGrid == null || cSweepGrid.Value;

		public static float SlowTickLogUs => (cSlowTick == null) ? 1500f : cSlowTick.Value;

		private static ConfigEntry<int> cNativeAlloc;

		private static ConfigEntry<bool> cSweepGrid;

		private static ConfigEntry<bool> cNativeSweep;

		private static ConfigEntry<bool> cNativeCpl;

		private static ConfigEntry<bool> cNativeTables;

		private static ConfigEntry<int> cNativeAllocCheck;

		private static ConfigEntry<float> cSlowTick;
		public static bool ParallelEnvelope => cParEnv == null || cParEnv.Value;

		private static ConfigEntry<float> cAilCamber;

		private static ConfigEntry<MpoRollMode> cMpoSep;

		private static ConfigEntry<int> cIfritRev;

		// Revoker 0.1.0
		private static ConfigEntry<bool> cYawTvc;

		private static ConfigEntry<float> cTvcYawMu;

		private static ConfigEntry<bool> cElevonPitch;

		private static ConfigEntry<bool> cCanardYawTier;

		private static ConfigEntry<float> cGroundElevonPitch;

		private static ConfigEntry<bool> cSlipYawTvc;

		private static ConfigEntry<float> cCorkTol;

		// Revoker 0.2.7
		private static ConfigEntry<float> cHiAoARollW;

		private static ConfigEntry<bool> cCanWin;

		private static ConfigEntry<float> cCanWinMargin;

		private static ConfigEntry<bool> cFinTvcHold;

		private static ConfigEntry<float> cFinNoTvcCap;

		private static ConfigEntry<bool> cVisHinge;

		private static ConfigEntry<float> cYawTvcNeutral;

		private static ConfigEntry<bool> cFlapRelief;

		private static ConfigEntry<bool> cCanardPitch;

		private static ConfigEntry<int> cRevokerRev;

		private static ConfigEntry<bool> cFlapFlaperons;

		private static ConfigEntry<float> cFlapHinge;

		private static ConfigEntry<float> cFlapHiLift;

		private static ConfigEntry<float> cFlapRollBias;

		private static ConfigEntry<float> cFlaperonRollPrio;

		private static ConfigEntry<float> cCanPitchFirst;
		private static ConfigEntry<float> cSurfFirst;
		private static ConfigEntry<float> cSurfFirstAoA;
		private static ConfigEntry<float> cSurfFirstIasLo;
		private static ConfigEntry<float> cSurfFirstIasHi;
		private static ConfigEntry<bool> cStockCanSched;
		private static ConfigEntry<bool> cWallPeak;
		private static ConfigEntry<bool> cGCapReach;
		private static ConfigEntry<float> cWallLatBand;
		private static ConfigEntry<bool> cWallLatCan;
		private static ConfigEntry<bool> cGCapRoll;

		private static ConfigEntry<float> cLeafBoost;

		private static ConfigEntry<bool> cThrustLineAtCg;

		private static ConfigEntry<bool> cDragShape;

		private static ConfigEntry<float> cDragShapeDepth;

		private static ConfigEntry<float> cPitchBump;

		private static ConfigEntry<float> cGyroBudget;

		private static ConfigEntry<bool> cLowSpeedTvc;

		private static ConfigEntry<float> cLowSpeedStart;

		private static ConfigEntry<float> cLowSpeedFull;

		private static ConfigEntry<float> cElevonSymCost;

		public static float FlapCamberK => (cFlapCamber != null) ? Mathf.Clamp(cFlapCamber.Value, 0f, 1f) : 0.62f;

		public static float AileronCamberK => (cAilCamber != null) ? Mathf.Clamp(cAilCamber.Value, 0f, 1f) : 0.66f;

		public static bool MpoSeparateAxes => cMpoSep != null && cMpoSep.Value == MpoRollMode.SeparateRollYaw;

		private static ConfigEntry<float> cMpoWash;

		public static float MpoSlipWashout => (cMpoWash != null) ? Mathf.Max(0f, cMpoWash.Value) : 0.5f;

		private static ConfigEntry<bool> cIfritTvcRoll;

		private static ConfigEntry<float> cSplitMu;

		private static ConfigEntry<bool> cSlipTvc;

		private static ConfigEntry<bool> cIfritFlaperons;

		private static ConfigEntry<float> cServoSpeed;

		// Ternion 0.1.0: canard travel (the FS-3's forward pitch pair) and rudder travel
		private static ConfigEntry<float> cCanLEDown;

		private static ConfigEntry<float> cCanLEUp;

		private static ConfigEntry<float> cRudRange;

		private static ConfigEntry<float> cElevTrim;

		private static ConfigEntry<bool> cElevSettled;

		private static ConfigEntry<bool> cPathFloor;

		private static ConfigEntry<float> cPitchMove;

		private static ConfigEntry<float> cElevStall;

		private static ConfigEntry<bool> cRaptorStab;

		private static ConfigEntry<bool> cCantedTvc;

		private static ConfigEntry<float> cTvcCant;

		private static ConfigEntry<bool> cMpoTvcAxis;

		private static ConfigEntry<float> cMpoTvcAxisBlend;

		private static ConfigEntry<bool> cCanSym;

		private static ConfigEntry<float> cElevEff;

		private static ConfigEntry<float> cCanEff;

		private static ConfigEntry<float> cGbElevator;

		private static ConfigEntry<float> cSplitRange;

		private static ConfigEntry<bool> cTvcRollNeutral;

		private static ConfigEntry<float> cTvcRollNeutralW;

		private static ConfigEntry<float> cSlipGainHi;

		private static ConfigEntry<bool> cSlipGainLoops;

		private static ConfigEntry<float> cActLag;

		private static ConfigEntry<bool> cStagger;

		private static ConfigEntry<float> cStallInner;

		private static ConfigEntry<float> cStallOuter;

		private static ConfigEntry<float> cStallSlope;

		private static ConfigEntry<bool> cSlipGovLead;

		private static ConfigEntry<bool> cDecHldFold;

		private static ConfigEntry<float> cSlipHoldTas0;

		private static ConfigEntry<float> cSlipLimTas0;

		private static ConfigEntry<float> cSlipLimTasSpan;

		private static ConfigEntry<float> cSlipHoldTasSpan;

		private static ConfigEntry<float> cSlipPredT;

		private static ConfigEntry<float> cSlipPredRollGate;

		private static ConfigEntry<bool> cSplitYawTier;

		private static ConfigEntry<float> cStabYawRelax;


		private static ConfigEntry<float> cSlipPredW;

		private static ConfigEntry<float> cRevS0;

		private static ConfigEntry<float> cRevS1;

		private static ConfigEntry<float> cSlipGainA0;

		private static ConfigEntry<float> cFullStickQ;

		private static ConfigEntry<float> cSplitBrakeHold;

		private static ConfigEntry<float> cTvcDiffMu;

		public static bool IfritSplitYaw => cIfritSplit != null && cIfritSplit.Value;

		public static bool IfritTvcRoll => cIfritTvcRoll == null || cIfritTvcRoll.Value;

		public static bool SlipTvcAssist => cSlipTvc != null && cSlipTvc.Value;

		public static bool IfritFlaperons => cIfritFlaperons == null || cIfritFlaperons.Value;

		public static float SurfaceServoSpeed => (cServoSpeed != null) ? Mathf.Clamp(cServoSpeed.Value, 0f, 300f) : 100f;

		// split limit: SplitRange, or 0 = the deceleron's own travel (FlaperonRange; the stock 25 when that is 0 too)
		public static float SplitRange => (cSplitRange != null && cSplitRange.Value > 0f) ? Mathf.Clamp(cSplitRange.Value, 1f, 45f) : ((FlaperonRange > 0f) ? FlaperonRange : 0f);

		public static bool TvcRollNeutralHold => cTvcRollNeutral != null && cTvcRollNeutral.Value;

		public static float TvcRollNeutralWeight => (cTvcRollNeutralW != null) ? Mathf.Max(0f, cTvcRollNeutralW.Value) : 30f;

		public static float FullStickPitchRate => (cFullStickQ != null) ? Mathf.Clamp(cFullStickQ.Value, 0f, 90f) : 60f;

		public static float SlipGainAoAStart => (cSlipGainA0 != null) ? Mathf.Clamp(cSlipGainA0.Value, 0f, 60f) : 5f;

		public static float RevFlowSenseStart => (cRevS0 != null) ? Mathf.Clamp(cRevS0.Value, 90f, 178f) : 120f;

		public static float RevFlowSenseEnd => (cRevS1 != null) ? Mathf.Clamp(cRevS1.Value, 91f, 180f) : 150f;

		public static float SlipLimTasMin => (cSlipLimTas0 != null) ? Mathf.Clamp(cSlipLimTas0.Value, 0f, 100f) : 25f;

		public static float SlipLimTasSpan => (cSlipLimTasSpan != null) ? Mathf.Clamp(cSlipLimTasSpan.Value, 1f, 100f) : 50f;

		public static float SlipHoldTasMin => (cSlipHoldTas0 != null) ? Mathf.Clamp(cSlipHoldTas0.Value, 0f, 60f) : 15f;

		public static float SlipHoldTasSpan => (cSlipHoldTasSpan != null) ? Mathf.Clamp(cSlipHoldTasSpan.Value, 1f, 60f) : 20f;

		public static bool DeceleronHldFold => cDecHldFold == null || cDecHldFold.Value;

		public static float SlipPredictTime => (cSlipPredT != null) ? Mathf.Clamp(cSlipPredT.Value, 0f, 0.8f) : 0.3f;

		public static bool SplitYawTier => cSplitYawTier == null || cSplitYawTier.Value;


		public static float StabYawRelax => (cStabYawRelax != null) ? Mathf.Clamp01(cStabYawRelax.Value) : 1f;

		public static float SlipPredictRollGate => (cSlipPredRollGate != null) ? Mathf.Clamp(cSlipPredRollGate.Value, 0f, 1f) : 0f;

		public static float SlipPredictWashout => (cSlipPredW != null) ? Mathf.Clamp(cSlipPredW.Value, 0f, 10f) : 1.5f;

		public static float SlipGainHighAoA => (cSlipGainHi != null) ? Mathf.Max(0f, cSlipGainHi.Value) : 0f;

		public static bool SlipGainFromLoops => cSlipGainLoops == null || cSlipGainLoops.Value;

		public static bool SlipGovLeadFromLoops => cSlipGovLead == null || cSlipGovLead.Value;

		public static float ActuatorLagT => (cActLag != null) ? Mathf.Clamp(cActLag.Value, 0f, 0.5f) : 0.07f;

		public static float SplitMu => (cSplitMu != null) ? Mathf.Max(1E-06f, cSplitMu.Value) : 1E-03f;

		public static float SplitBrakeHoldCost => (cSplitBrakeHold != null) ? Mathf.Max(0f, cSplitBrakeHold.Value) : 0.5f;

		public static bool YawTvc => cYawTvc == null || cYawTvc.Value;

		public static float TvcYawMu => (cTvcYawMu != null) ? Mathf.Max(1E-06f, cTvcYawMu.Value) : 0.0005f;

		public static bool ElevonPitch => cElevonPitch == null || cElevonPitch.Value;

		public static bool FlapFlaperons => cFlapFlaperons == null || cFlapFlaperons.Value;

		public static float FlapHingeRange => (cFlapHinge != null) ? Mathf.Clamp(cFlapHinge.Value, 5f, 35f) : 25f;

		public static bool ThrustLineAtCg => cThrustLineAtCg == null || cThrustLineAtCg.Value;

		public static bool HighAoADragShape => cDragShape == null || cDragShape.Value;

		public static bool LowSpeedTvcRoll => cLowSpeedTvc != null && cLowSpeedTvc.Value;

		// Su-30 0.3.0 (LowSpeedSlipTvc): at very low speed the canted nozzles take out sideslip past a threshold, accepting the roll / yaw they make
		public static bool LowSpeedSlipTvc => cLsSlipTvc != null && cLsSlipTvc.Value;

		public static float LowSpeedSlipTvcStart => cLsSlipTvcStart == null ? 8f : Mathf.Clamp(cLsSlipTvcStart.Value, 0f, 45f);

		public static float LowSpeedSlipTvcGain => cLsSlipTvcGain == null ? 2f : Mathf.Clamp(cLsSlipTvcGain.Value, 0f, 5f);

		public static float LowSpeedSlipTvcMax => cLsSlipTvcMax == null ? 40f : Mathf.Clamp(cLsSlipTvcMax.Value, 0f, 120f);

		// Su-30 0.3.0 (MpoAxisReleaseStop): nozzle-axis rotation, lateral stick and pedals released: stop the yaw at once, let the roll go
		public static bool MpoAxisReleaseStop => cMpoAxRel == null || cMpoAxRel.Value;

		public static float LowSpeedTvcRollStart => (cLowSpeedStart != null) ? Mathf.Clamp(cLowSpeedStart.Value, 40f, 400f) : 150f;

		public static float LowSpeedTvcRollFull => (cLowSpeedFull != null) ? Mathf.Clamp(cLowSpeedFull.Value, 20f, 390f) : 120f;

		private static ConfigEntry<float> cLowSpeedKStart;

		private static ConfigEntry<float> cLowSpeedKFull;

		public static float LowSpeedTvcRollStiffStart => (cLowSpeedKStart != null) ? Mathf.Clamp(cLowSpeedKStart.Value, 0.02f, 5f) : 0.4f;

		public static float LowSpeedTvcRollStiffFull => (cLowSpeedKFull != null) ? Mathf.Clamp(cLowSpeedKFull.Value, 0f, 4.9f) : 0.2f;

		public static float GyroBudgetFrac => (cGyroBudget != null) ? Mathf.Clamp(cGyroBudget.Value, 0f, 2f) : 0.25f;

		// Su-30 0.2.5 (PitchBump.cs)
		public static float HighAoAPitchBump => (cPitchBump != null) ? Mathf.Clamp(cPitchBump.Value, 0f, 3f) : 0.75f;

		public static float HighAoADragShapeDepth => (cDragShapeDepth != null) ? Mathf.Clamp(cDragShapeDepth.Value, 0f, 1.5f) : 1f;

		public static float FallingLeafYawBoost => (cLeafBoost != null) ? Mathf.Clamp(cLeafBoost.Value, 1f, 3f) : 1.2f;

		public static float HighAoACanardPitchFirst => (cCanPitchFirst != null) ? Mathf.Max(0f, cCanPitchFirst.Value) : 0f;
		public static float SurfacesFirstPitch => (cSurfFirst != null) ? Mathf.Max(0f, cSurfFirst.Value) : 30f;
		public static float SurfacesFirstIasLo => (cSurfFirstIasLo != null) ? Mathf.Max(0f, cSurfFirstIasLo.Value) : 350f;
		public static float SurfacesFirstIasHi => (cSurfFirstIasHi != null) ? Mathf.Max(0f, cSurfFirstIasHi.Value) : 450f;
		public static float SurfacesFirstMaxAoA => (cSurfFirstAoA != null) ? Mathf.Clamp(cSurfFirstAoA.Value, 0f, 90f) : 35f;
		public static bool GCaptureReachable => cGCapReach == null || cGCapReach.Value;
		public static bool PitchWallSurfacePeak => cWallPeak == null || cWallPeak.Value;
		public static float WallLateralGoodFrac => (cWallLatBand == null) ? 0.45f : cWallLatBand.Value;
		public static bool WallPinLateralCanard => cWallLatCan == null || cWallLatCan.Value;
		public static bool GCaptureReachableRolling => cGCapRoll != null && cGCapRoll.Value;
		public static bool StockCanardSchedule => cStockCanSched != null && cStockCanSched.Value;

		public static float FlaperonRollPriority => (cFlaperonRollPrio != null) ? Mathf.Clamp01(cFlaperonRollPrio.Value) : 1f;

		public static float FlapRollBias => (cFlapRollBias != null) ? Mathf.Clamp01(cFlapRollBias.Value) : 1f;

		public static float FlapHighLiftDroop => (cFlapHiLift != null) ? Mathf.Clamp(cFlapHiLift.Value, 0f, 35f) : 0f;

		public static bool CanardPitchPriority => cCanardPitch == null || cCanardPitch.Value;

		public static bool FlaperonLoadRelief => cFlapRelief != null && cFlapRelief.Value;

		public static float YawTvcNeutralHold => (cYawTvcNeutral != null) ? Mathf.Max(0f, cYawTvcNeutral.Value) : 300f;

		public static bool VisualHingeFix => cVisHinge == null || cVisHinge.Value;

		public static float CorkscrewSlipTol => (cCorkTol != null) ? Mathf.Clamp(cCorkTol.Value, 0f, 15f) : 5f;

		public static float HighAoARollWeight => (cHiAoARollW != null) ? Mathf.Clamp(cHiAoARollW.Value, 0.1f, 5f) : 1f;

		public static bool CanardAttachedWindow => cCanWin == null || cCanWin.Value;

		public static float CanardWindowMargin => (cCanWinMargin != null) ? Mathf.Clamp(cCanWinMargin.Value, 0f, 30f) : 12f;

		public static bool FinLossTvcHold => cFinTvcHold == null || cFinTvcHold.Value;

		public static float FinLossNoTvcAoACap => (cFinNoTvcCap != null) ? Mathf.Clamp(cFinNoTvcCap.Value, 0f, 70f) : 30f;

		public static bool SlipYawTvcAssist => cSlipYawTvc == null || cSlipYawTvc.Value;

		public static float GroundElevonPitch => (cGroundElevonPitch != null) ? Mathf.Clamp01(cGroundElevonPitch.Value) : 1f;

		public static bool CanardYawTier => cCanardYawTier != null && cCanardYawTier.Value;

		public static float ElevonSymCost => (cElevonSymCost != null) ? Mathf.Max(0f, cElevonSymCost.Value) : 2f;

		public static float TvcDiffMu => (cTvcDiffMu != null) ? Mathf.Max(1E-06f, cTvcDiffMu.Value) : 0.003f;

		private static ConfigEntry<float> cGPos;

		private static ConfigEntry<float> cGNeg;

		private static ConfigEntry<float> cMpoGPos;

		private static ConfigEntry<float> cMpoGNeg;

		private static ConfigEntry<float> cMaxPitch;

		private static ConfigEntry<float> cProt;

		private static ConfigEntry<float> cMaxRoll;

		private static ConfigEntry<float> cMaxYaw;

		private static ConfigEntry<float> cCouple;

		private static ConfigEntry<float> cPedalBeta;

		private static ConfigEntry<float> cKBeta;

		private static ConfigEntry<float> cKRoll;

		private static ConfigEntry<float> cKPitch;

		private static ConfigEntry<float> cKYaw;

		private static ConfigEntry<float> cKpAoA;

		private static ConfigEntry<bool> cPairTrim;

		private static ConfigEntry<float> cLatRes;

		private static ConfigEntry<bool> cBranch;

		private static ConfigEntry<bool> cOvPrio;

		private static ConfigEntry<float> cCoupRoll;

		private static ConfigEntry<float> cBranchMaxAoA;

		private static ConfigEntry<float> cBranchTarget;

		private static ConfigEntry<float> cBranchTol;

		private static ConfigEntry<float> cAntiWind;

		private static ConfigEntry<bool> cThrBrake;

		private static ConfigEntry<float> cBrakeBlend;

		private static ConfigEntry<float> cKiAoA;

		private static ConfigEntry<float> cKpG;

		private static ConfigEntry<float> cKiG;



		private static ConfigEntry<float> cActuator;



		private static ConfigEntry<float> cUnloadDecel;

		private static ConfigEntry<float> cUnloadRise;

		private static ConfigEntry<float> cRollLo;

		private static ConfigEntry<float> cRollHi;

		private static ConfigEntry<float> cRollAuth;

		private static ConfigEntry<float> cRollBrake;

		private static ConfigEntry<float> cStabTrim;





		private static ConfigEntry<float> cStabPeakMargin;




		private static ConfigEntry<bool> cTvcOff;

		private static ConfigEntry<AoALimitTest> cAoATest;

		private static ConfigEntry<float> cNoTvcCapPos;

		private static ConfigEntry<float> cNoTvcCapNeg;

		private static ConfigEntry<float> cNoTvcThrCapPos;

		private static ConfigEntry<float> cNoTvcThrCapNeg;

		private static ConfigEntry<float> cLowThrustTW;

		private static ConfigEntry<bool> cRateOnly;

		private static ConfigEntry<PitchCommandLaw> cPitchLaw;

		private static ConfigEntry<bool> cLowCapRoll;

		private static ConfigEntry<bool> cAsymShared;

		private static ConfigEntry<bool> cGovPred;

		private static ConfigEntry<float> cPullRollTrade;

		// Revoker FCS 0.3.0 (F-22E 2.6.0): the lateral reference governor's knobs (player config section 6) and the 2.4.18 yaw slew lead (Dev)
		private static ConfigEntry<float> cRollStopSlip;

		private static ConfigEntry<int> cGovRev;
		private static ConfigEntry<float> cPushSlip;

		private static ConfigEntry<bool> cLsSlipTvc;

		private static ConfigEntry<float> cLsSlipTvcStart;

		private static ConfigEntry<float> cLsSlipTvcGain;

		private static ConfigEntry<float> cLsSlipTvcMax;

		private static ConfigEntry<bool> cMpoAxRel;

		private static ConfigEntry<float> cPushSlipNoTvc;

		private static ConfigEntry<float> cPushMinRate;


		private static ConfigEntry<bool> cYawSlewLead;

		private static ConfigEntry<bool> cRollGovPitch;

		private static ConfigEntry<bool> cRollStopSlipProbe;

		private static ConfigEntry<float> cRollPaceRateSlip;

		private static ConfigEntry<float> cRollPaceRateBlend;

		private static ConfigEntry<float> cSplitFlapRollYield;

		private static ConfigEntry<bool> cSplitFlapYaw;

		private static ConfigEntry<float> cSplitFlapYawUse;

		private static ConfigEntry<float> cRollPitchLead;

		private static ConfigEntry<bool> cStabsPitchOnly;

		private static ConfigEntry<bool> cSlatRelief;

		private static ConfigEntry<bool> cCoupGateLim;

		private static ConfigEntry<bool> cLoadedRollSteady;

		private static ConfigEntry<bool> cSlipYawW;


		private static ConfigEntry<bool> cCapLag;


		private static ConfigEntry<float> cLeadK;

		private static ConfigEntry<float> cLeadTau;

		private static ConfigEntry<bool> cFuelBal;

		private static ConfigEntry<bool> cFuelAttached;

		private static ConfigEntry<bool> cFuelBalLog;

		private static ConfigEntry<float> cFuelRate;

		private static ConfigEntry<float> cFuelTolLat;

		private static ConfigEntry<float> cFuelTolLong;

		private static ConfigEntry<bool> cFuelCgHold;

		private static ConfigEntry<bool> cGndTvcRY;

		private static ConfigEntry<float> cFuelCgHoldRef;





		private static ConfigEntry<bool> cGroundRate;

		private static ConfigEntry<float> cGroundRotRamp;






		private static ConfigEntry<float> cTvcNeutral;

		private static ConfigEntry<bool> cStabPark;

		private static ConfigEntry<float> cSettledMargin;

		private static ConfigEntry<float> cSettledOn;

		private static ConfigEntry<float> cSettledOff;

		private static ConfigEntry<float> cSettledWalk;

		private static ConfigEntry<float> cSettledSlope;

		private static ConfigEntry<float> cSettledYawUse;


		private static ConfigEntry<bool> cRollFightRel;


		private static ConfigEntry<int> cAirframeRev;


		private static ConfigEntry<float> cCoupFF;

		private static ConfigEntry<float> cRelPitchPrio;

		private static ConfigEntry<float> cWashSlew;

		private static ConfigEntry<float> cMomLead;

		private static ConfigEntry<float> cOnset;

		private static ConfigEntry<float> cStabRoll;

		private static ConfigEntry<float> cStabRollShare;

		private static ConfigEntry<float> cStickSmooth;

		private static ConfigEntry<float> cActDamp;

		private static ConfigEntry<float> cYawStop;

		private static ConfigEntry<float> cOuterFlap;

		private static ConfigEntry<float> cRollAcc;

		private static ConfigEntry<float> cThrust;

		private static ConfigEntry<float> cDrag;

		private static ConfigEntry<float> cDrAoA;

		private static ConfigEntry<float> cStockDrAoA;

		private static ConfigEntry<float> cStockDrFade0;

		private static ConfigEntry<float> cStockDrFade1;



		private static ConfigEntry<float> cDrTrans;


		private static ConfigEntry<float> cYawPri;

		private static ConfigEntry<float> cYawFF;

		private static ConfigEntry<float> cRollLead;

		private static ConfigEntry<float> cFlareScaleStock;

		private static ConfigEntry<float> cFlareScaleBuffed;

		private static ConfigEntry<float> cEwCapStock;

		private static ConfigEntry<float> cEwCapBuffed;

		private static ConfigEntry<float> cEwRateStock;

		private static ConfigEntry<float> cEwRateBuffed;

		private static ConfigEntry<float> cLowYaw;

		private static ConfigEntry<float> cLowYawIas;

		private static ConfigEntry<float> cLowYawFall;

		private static ConfigEntry<float> cTvcMove;

		private static ConfigEntry<float> cRollTvc;

		private static ConfigEntry<float> cRollWash;

		private static ConfigEntry<float> cRollTvcBudget;

		private static ConfigEntry<float> cRollTvcBudgetCost;


		private static ConfigEntry<bool> cNwsFbw;

		private static ConfigEntry<bool> cGndStock;

		private static ConfigEntry<bool> cGearDownSlip;

		private static ConfigEntry<bool> cGndIndi;

		private static ConfigEntry<float> cGndIndiK;

		private static ConfigEntry<float> cGndIndiTau;

		private static ConfigEntry<bool> cGndLightNose;

		private static ConfigEntry<bool> cFlapSched;

		private static ConfigEntry<float> cFlapSchedMax;

		private static ConfigEntry<float> cGndPedalBrake;

		private static ConfigEntry<bool> cGndIndiBrake;

		private static ConfigEntry<float> cGndIndiBrakeMax;

		private static ConfigEntry<float> cGndIndiBrakeFull;

		private static ConfigEntry<float> cGndIndiBrakeZero;

		private static ConfigEntry<float> cGndDriftAllow;

		private static ConfigEntry<float> cGndDriftGain;

		private static ConfigEntry<float> cGndDriftLead;

		private static ConfigEntry<float> cGndDriftFade;

		private static ConfigEntry<float> cGndYawRate;

		private static ConfigEntry<float> cGndLatAcc;

		private static ConfigEntry<float> cGndLine;



		private static ConfigEntry<float> cAsymPitchMargin;

		private static ConfigEntry<bool> cAsymRollCoupling;

		private static ConfigEntry<bool> cRollPitchPriority;

		private static ConfigEntry<float> cRollFightCost;


		private static ConfigEntry<float> cPitchFight;

		private static ConfigEntry<float> cRollPitchReserve;



		private static ConfigEntry<float> cGunAmmoStock;

		private static ConfigEntry<float> cGunAmmoBuffed;

		private static ConfigEntry<float> cFuelUse;

		private static ConfigEntry<float> cRotRate;

		private static ConfigEntry<float> cAsymReserve;
		private static ConfigEntry<float> cAsymCplResMax;
		private static ConfigEntry<float> cAsymStopTime;
		private static ConfigEntry<float> cAsymAoAWeight;

		private static ConfigEntry<float> cAsymFloor;

		private static ConfigEntry<float> cAsymFloorNeg;

		private static ConfigEntry<float> cAsymTighten;

		private static ConfigEntry<float> cAsymRelax;

		private static ConfigEntry<float> cAsymRec;

		private static ConfigEntry<float> cAsymMinIas;

		private static ConfigEntry<float> cAsymRollAcc;

		private static ConfigEntry<float> cAsymYawAcc;

		private static ConfigEntry<float> cAsymBoost;

		private static ConfigEntry<float> cAsymDead;

		private static ConfigEntry<float> cAsymFF;

		private static ConfigEntry<float> cAsymFFTau;

		private static ConfigEntry<float> cAsymRollTau;

		private static ConfigEntry<float> cAsymRelaxBoost;

		private static ConfigEntry<float> cRollPitchFF;

		private static ConfigEntry<float> cRollPitchFFAoA;

		private static ConfigEntry<float> cAsymFFBeta;

		private static ConfigEntry<float> cAsymFFAoA;



		private static ConfigEntry<float> cLowQReg;

		private static ConfigEntry<float> cRollStop;

		private static ConfigEntry<float> cRollAccUse;

		private static ConfigEntry<float> cRollAccUseLow;






		private static ConfigEntry<float> cHlMaxAoA;

		private static ConfigEntry<float> cHlHighAoA;


		private static ConfigEntry<float> cRotMinSpeed;

		// Su-30 0.1.4 (DownwashFadeStart / End)
		private static ConfigEntry<float> cDwFade0;

		private static ConfigEntry<float> cDwFade1;

		public static float DownwashFadeStart => (cDwFade0 != null) ? Mathf.Clamp(cDwFade0.Value, 20f, 85f) : 45f;

		public static float DownwashFadeEnd => (cDwFade1 != null) ? Mathf.Clamp(cDwFade1.Value, 25f, 90f) : 60f;

		// Su-30 0.1.4 (StabHighAoAScale)
		private static ConfigEntry<float> cStabHiScale;

		public static float StabHighAoAScale => (cStabHiScale != null) ? Mathf.Clamp(cStabHiScale.Value, 0f, 1f) : 1f;

		// Su-30 0.1.2 (RollRateCapRudderShare)
		private static ConfigEntry<float> cRateCapFloor;

		public static float RollRateCapFloor => (cRateCapFloor != null) ? Mathf.Clamp01(cRateCapFloor.Value) : 0f;

		// Su-30 0.1.1 (CantedVvRollWeight)
		private static ConfigEntry<float> cHiAoAVvW;

		public static float HighAoAVvRollWeight => (cHiAoAVvW != null) ? Mathf.Clamp(cHiAoAVvW.Value, 1f, 20f) : 4f;

		private static ConfigEntry<float> cSlatStart;

		private static ConfigEntry<float> cSlatFull;


		private static ConfigEntry<float> cGUnder;

		private static ConfigEntry<float> cFFMax;

		private static ConfigEntry<float> cFFStabCost;

		private static ConfigEntry<bool> cGndBrakeSplit;

		private static ConfigEntry<bool> cThrIndepBrake;

		public static bool ThrottleIndependentAirbrake => cThrIndepBrake == null || cThrIndepBrake.Value;




		public static bool Enabled => cEnabled.Value && MpPolicy.FcsAllowed;

		public static bool TelemetryEnabled => cTelemetry.Value;

		public static bool AIFlightControl => cAIFcs.Value;

		public static bool AIFlightLog => cAILog.Value;

		public static bool AIDirectRoll => cAIDirectRoll.Value;

		public static float StabRangeTEUp => Mathf.Clamp(cStabUp.Value, 0f, 80f);

		public static float StabRangeTEDown => Mathf.Clamp(cStabDn.Value, 0f, 80f);

		public static float RudderRange => (cRudRange != null) ? Mathf.Clamp(cRudRange.Value, 0f, 45f) : 35f;

		public static float CanardRangeLEDown => (cCanLEDown != null) ? Mathf.Clamp(cCanLEDown.Value, 0f, 80f) : 60f;

		public static float ElevatorTrimCost => (cElevTrim != null) ? Mathf.Clamp(cElevTrim.Value, 0f, 1f) : 0.004f;

		public static float ElevatorEffectiveness => (cElevEff != null) ? Mathf.Clamp(cElevEff.Value, 0.5f, 3f) : 1.2f;

		public static float CanardEffectiveness => (cCanEff != null) ? Mathf.Clamp(cCanEff.Value, 0.5f, 3f) : 1.1f;

		public static float GroundBrakeElevator => (cGbElevator != null) ? Mathf.Clamp(cGbElevator.Value, 0f, 80f) : 35f;


		// Su-30 0.1.0
		public static bool RaptorStabFoil => cRaptorStab != null && cRaptorStab.Value;
		public static bool CantedTvc => cCantedTvc == null || cCantedTvc.Value;
		public static float TvcCantAngle => (cTvcCant != null) ? Mathf.Clamp(cTvcCant.Value, 0f, 60f) : 32f;
		public static bool MpoTvcAxisRoll => cMpoTvcAxis == null || cMpoTvcAxis.Value;
		public static float MpoTvcAxisBlend => (cMpoTvcAxisBlend != null) ? Mathf.Clamp(cMpoTvcAxisBlend.Value, 0.5f, 30f) : 5f;
		public static bool CanardSymmetric => cCanSym == null || cCanSym.Value;
		public static float ElevatorStallAoA => (cElevStall != null) ? ((cElevStall.Value > 0f) ? Mathf.Clamp(cElevStall.Value, 15f, 80f) : 0f) : 0f;

		public static float PitchSurfaceMoveCost => (cPitchMove != null) ? Mathf.Clamp(cPitchMove.Value, 0f, 1f) : 0f;

		public static bool FlightPathRateFloor => cPathFloor == null || cPathFloor.Value;

		public static bool ElevatorSettledWindow => cElevSettled != null && cElevSettled.Value;

		public static float CanardRangeLEUp => (cCanLEUp != null) ? Mathf.Clamp(cCanLEUp.Value, 0f, 80f) : 45f;

		public static float FlaperonRange => Mathf.Clamp(cFlapRange.Value, 0f, 40f);

		public static float TvcRange => Mathf.Clamp(cTvcRange.Value, 0f, 20f);

		public static float TvcSlew => (cTvcSpeed.Value > 0f) ? Mathf.Clamp(cTvcSpeed.Value, 10f, 300f) : (70f * ActuatorScale);

		public static bool AIGunRollFade => cAIGunRollFade.Value;

		// 2.0.8: the debug AoALimit test switch overrides the positive limit (Full65 = 65, Cap36 = 36 at most)
		public static float AoAPos => AoAFull65 ? 65f : (AoACap36 ? Mathf.Min(cAoAPos.Value, NoTvcAoACapPos) : cAoAPos.Value);

		// 2.0.9: AoA caps while the nozzles do not cover pitch (DisableTVC or T/W under LowThrustFixedAoATW), debug-adjustable
		public static float NoTvcAoACapPos => (cNoTvcCapPos != null) ? Mathf.Clamp(cNoTvcCapPos.Value, 10f, 90f) : 50f;

		// Su-30 0.2.4: the magnitudes were clamped to 45 (F-22E heritage), so a -50 set in the file still flew -45 (Tom 10 Nov 14:55). Now 10-90;
		// the AoA limit itself still applies on top.
		public static float NoTvcAoACapNeg => (cNoTvcCapNeg != null) ? Mathf.Clamp(Mathf.Abs(cNoTvcCapNeg.Value), 10f, 90f) : 50f;

		// 2.2.2: with DisableTVC, the caps when the thrust is up (positive: from T/W NoTvcThrustCapTW0, full at NoTvcThrustCapTW1)
		public static float NoTvcThrCapPos => (cNoTvcThrCapPos != null) ? Mathf.Clamp(cNoTvcThrCapPos.Value, 10f, 90f) : 50f;

		public static float NoTvcThrCapNeg => (cNoTvcThrCapNeg != null) ? Mathf.Clamp(Mathf.Abs(cNoTvcThrCapNeg.Value), 10f, 90f) : 50f;

		public static bool DisableTVC => cTvcOff != null && cTvcOff.Value;

		public static bool AoAFull65 => cAoATest != null && cAoATest.Value == AoALimitTest.Full65;

		public static bool AoACap36 => cAoATest != null && cAoATest.Value == AoALimitTest.Cap36;

		public static float StabDownwash => (cDownwash != null) ? Mathf.Clamp(cDownwash.Value, 0f, 0.7f) : 0f;

		public static bool DownwashLiftDriven => cDownwashLift == null || cDownwashLift.Value;

		public static bool StrakeVortexLift => cStrakeVortex == null || cStrakeVortex.Value;

		public static bool StabSlabFoil => cSlab != null && cSlab.Value;

		public static float SlabStallAoA => (cSlabStall != null) ? Mathf.Clamp(cSlabStall.Value, 17f, 45f) : 30f;

		public static float SlabLiftSlope => (cSlabSlope != null) ? Mathf.Clamp01(cSlabSlope.Value) : 0.35f;

		public static float StrakeBreakdownStart => (cStrakeBd0 != null) ? Mathf.Clamp(cStrakeBd0.Value, 10f, 80f) : 38f;

		public static float StrakeBreakdownEnd => (cStrakeBd1 != null) ? Mathf.Clamp(cStrakeBd1.Value, 11f, 89f) : 60f;

		public static float StrakeResidualCN => (cStrakeRes != null) ? Mathf.Clamp(cStrakeRes.Value, 0.3f, 2f) : 1.2f;

		public static float StabPeakMargin => (cStabPeakMargin != null) ? Mathf.Clamp(cStabPeakMargin.Value, 0f, 90f) : 90f;

		public static bool BodyLiftShaping => (false);

		public static float BodyLiftFore => Mathf.Clamp((0.9f), 0.3f, 2f);

		public static float BodyLiftAft => Mathf.Clamp((1.8f), 0.3f, 3f);

		public static float AoANeg => AoACap36 ? Mathf.Max(0f - Mathf.Abs(cAoANeg.Value), 0f - NoTvcAoACapNeg) : (0f - Mathf.Abs(cAoANeg.Value));

		public static float GPos => cGPos.Value;

		public static float GNeg => 0f - Mathf.Abs(cGNeg.Value);

		public static float MpoGPos => cMpoGPos.Value;

		public static float MpoGNeg => 0f - Mathf.Abs(cMpoGNeg.Value);

		public static float MaxPitchRate => cMaxPitch.Value;

		public static float RotationRate => cRotRate.Value;

		public static float ProtRate => cProt.Value;

		public static float MaxRollRate => cMaxRoll.Value;

		public static float MaxYawRate => cMaxYaw.Value;

		public static float RollGainLowIAS => cRollLo.Value;

		public static float RollGainHighIAS => cRollHi.Value;

		public static float RollAuthorityUse => cRollAuth.Value;

		public static float RollBrakeUse => cRollBrake.Value;

		public static float StabTrimCost => cStabTrim.Value;

		public static float LerxEffectiveness => (cLerxEff != null) ? Mathf.Clamp(cLerxEff.Value, 0.5f, 6f) : 1.13f;

		public static bool StallStagger => cStagger == null || cStagger.Value;

		public static float LowThrustTW => cLowThrustTW.Value;

		public static bool RateCommandOnly => cRateOnly.Value;

		public static float StickLeadGain => cLeadK == null ? 0f : Mathf.Clamp(cLeadK.Value, 0f, 3f);

		public static float StickLeadTau => cLeadTau == null ? 0.15f : Mathf.Clamp(cLeadTau.Value, 0.03f, 0.5f);

		public static bool RollGovPredictiveOn => cGovPred == null || cGovPred.Value;

		public static bool SplitFlaperonYaw => cSplitFlapYaw == null || cSplitFlapYaw.Value;

		public static float SplitFlaperonYawUse => cSplitFlapYawUse == null ? 1f : Mathf.Clamp(cSplitFlapYawUse.Value, 0f, 1f);

		public static float SplitFlaperonRollYield => cSplitFlapRollYield == null ? 0f : Mathf.Clamp(cSplitFlapRollYield.Value, 0f, 1f);

		public static float PullRollTrade => cPullRollTrade == null ? 0.7f : Mathf.Clamp(cPullRollTrade.Value, 0f, 1f);

		public static float RollStopSideslip => cRollStopSlip == null ? 10f : Mathf.Clamp(cRollStopSlip.Value, 1f, 15f);

		public static float PushoverSideslip => cPushSlip == null ? 15f : (cPushSlip.Value < 0f ? RollStopSideslip : Mathf.Clamp(cPushSlip.Value, 0f, 25f));

		// Su-30 0.2.4: the push-over sideslip allowance while the nozzles do not cover pitch / yaw (TVC off or thrust under LowThrustFixedAoATW)
		public static float PushoverSideslipNoTvc => cPushSlipNoTvc == null ? 12f : (cPushSlipNoTvc.Value < 0f ? PushoverSideslip : Mathf.Clamp(cPushSlipNoTvc.Value, 0f, 25f));

		// Su-30 0.2.4 (F-22E 3.1.5): the push-over minimum rate (deg/s): a planned push is never held slower than this, whatever its sideslip
		public static float PushOverMinRate => cPushMinRate == null ? 23f : Mathf.Clamp(cPushMinRate.Value, 0f, 60f);

		public static bool RollGovernorPitch => cRollGovPitch == null || cRollGovPitch.Value;

		public static bool RollStopSlipProbe => cRollStopSlipProbe == null || cRollStopSlipProbe.Value;

		public static float RollPaceRateBlend => (cRollPaceRateBlend == null) ? 1f : Mathf.Clamp01(cRollPaceRateBlend.Value);

		public static float RollPaceRateSlip => (cRollPaceRateSlip == null) ? 1f : Mathf.Clamp(cRollPaceRateSlip.Value, 0.5f, 3f);

		public static bool YawSlewLead => cYawSlewLead == null || cYawSlewLead.Value;

		public static bool StabsPitchOnly => cStabsPitchOnly != null && cStabsPitchOnly.Value;

		public static float RollPitchLead => cRollPitchLead == null ? 0.25f : Mathf.Clamp(cRollPitchLead.Value, 0f, 0.6f);

		public static bool SlatRollReliefOn => cSlatRelief != null && cSlatRelief.Value;

		public static bool CoupGateAtLimitOn => cCoupGateLim == null || cCoupGateLim.Value;

		public static bool LoadedRollSteadyOn => cLoadedRollSteady == null || cLoadedRollSteady.Value;

		public static bool SlipYawWeightOn => cSlipYawW == null || cSlipYawW.Value;

		public static bool CapLagOn => cCapLag == null || cCapLag.Value;

		public static float FinCamberK => (cFinCamber != null) ? Mathf.Clamp(cFinCamber.Value, 0f, 1f) : 0.54f;

		public static float WingCamberK => false ? 0f : Mathf.Clamp((0f), 0f, 1f);

		public static bool AsymSharedLatOn => cAsymShared == null || cAsymShared.Value;

		public static bool LowCapRollFix => cLowCapRoll == null || cLowCapRoll.Value;

		public static bool GRateLaw => cPitchLaw == null || cPitchLaw.Value == PitchCommandLaw.GAndPitchRate;

		public static bool AoAThenRateLaw => cPitchLaw != null && cPitchLaw.Value == PitchCommandLaw.AoAThenRateHold;

		public static bool FuelBalanceOn => cFuelBal.Value;

		public static bool FuelAttachedOnly => cFuelAttached.Value;

		public static bool FuelBalanceLog => cFuelBalLog.Value;

		public static float FuelTransferRate => Mathf.Max(0f, cFuelRate.Value);

		public static bool FuelTrim => cFuelTrim.Value;

		public static float FuelTrimRate => Mathf.Max(0f, cFuelTrimRate.Value);

		public static float FuelBalanceTolLat => Mathf.Max(0.001f, cFuelTolLat.Value);

		public static float FuelBalanceTolLong => Mathf.Max(0.001f, cFuelTolLong.Value);

		public static bool FuelCgHold => cFuelCgHold == null || cFuelCgHold.Value;

		public static bool GroundTvcRollYaw => cGndTvcRY == null || cGndTvcRY.Value;

		public static float FuelCgHoldRef => (cFuelCgHoldRef != null) ? Mathf.Clamp01(cFuelCgHoldRef.Value) : 0.7f;

		public static bool RudderPitchAid => (false);

		public static float RudderToeCost => Mathf.Max(0.001f, (0.05f));

		public static float RudderToeMax => Mathf.Clamp((20f), 0f, 35f);

		public static float RudderToeGain => Mathf.Max(0f, (2f));

		public static bool GroundRateLaw => cGroundRate.Value;

		public static float GroundRotationRamp => Mathf.Max(1f, cGroundRotRamp.Value);

		public static float StabStallAoA => (25f);

		public static float InnerFlaperonStallAoA => (cStallInner != null) ? Mathf.Clamp(cStallInner.Value, 20f, 80f) : 26f;

		public static float OuterFlaperonStallAoA => (cStallOuter != null) ? Mathf.Clamp(cStallOuter.Value, 10f, 80f) : 45f;

		public static float StallStaggerLiftSlope => 0f;

		public static bool RollFightReleased => cRollFightRel == null || cRollFightRel.Value;

		// Ifrit: the F-22E's speedbrake fade (2.2.16) works on Aryx's Airbrake components; the Ifrit has none
		public static bool AirbrakeAoAFade => false;

		public static bool TeBrakeAoAFade => cTeBrakeFade == null || cTeBrakeFade.Value;

		public static float AirbrakeFadeShareFull => 0.4f;

		public static float AirbrakeFadeShareZero => 0.15f;

		public static float FinLossAoACap => false ? 40f : Mathf.Clamp((0f), 0f, 65f);

		public static bool StabParkEscape => cStabPark == null || cStabPark.Value;

		public static float SettledStallMargin => (cSettledMargin == null) ? 6f : Mathf.Clamp(cSettledMargin.Value, 0f, 20f);

		public static float SettledRateOn => (cSettledOn == null) ? 10f : Mathf.Clamp(cSettledOn.Value, 1f, 60f);

		public static float SettledRateOff => (cSettledOff == null) ? 20f : Mathf.Max(SettledRateOn + 1f, cSettledOff.Value);

		public static float SettledWalkRate => (cSettledWalk == null) ? 30f : Mathf.Clamp(cSettledWalk.Value, 1f, 100f);

		public static float SettledSlopeFrac => (cSettledSlope == null) ? 0.15f : Mathf.Clamp(cSettledSlope.Value, 0f, 1f);

		public static float SettledYawUse => (cSettledYawUse == null) ? 0.4f : Mathf.Clamp(cSettledYawUse.Value, 0f, 5f);

		public static float TvcNeutralHold => cTvcNeutral == null ? 1000f : Mathf.Max(0f, cTvcNeutral.Value);

		public static float StabCLScale => false ? 1f : Mathf.Clamp((1f), 0.5f, 2f);

		public static float StabLiftSlope => false ? -1f : Mathf.Clamp((-1f), -1f, 1f);

		public static float CgShiftAft => (cCgShift != null) ? Mathf.Clamp(cCgShift.Value, 0f, 0.6f) : 0.25f;

		// Su-30 0.1.12: lift-area multiplier on the aft body parts (intake / engine covers, engine bodies, tail cone)
		public static bool CaptureMeasured => cCapMeas == null || cCapMeas.Value;
		public static bool GCaptureMeasured => cGCapMeas == null || cGCapMeas.Value;
		public static bool SoftAoALimit => cSoftLim == null || cSoftLim.Value;
		public static float SoftAoALimitMargin => (cSoftLimMargin != null) ? Mathf.Clamp(cSoftLimMargin.Value, 0.5f, 30f) : 3f;
		public static bool StopIgnoresSlip => cStopHard == null || cStopHard.Value;
		public static bool PitchTrajMeasuredEnd => cTrajMeas == null || cTrajMeas.Value;

		public static float AftBodyLift => (cAftBodyLift != null) ? Mathf.Clamp(cAftBodyLift.Value, 0f, 2f) : 0.5f;

		public static float CouplingFF => cCoupFF.Value;

		public static float ReleasedPitchPriority => cRelPitchPrio.Value;

		public static float RollNozzleWashoutSlew => Mathf.Max(0f, cWashSlew.Value);

		public static float ReleasedMomentLead => Mathf.Clamp(cMomLead.Value, 0f, 0.15f);

		public static float PitchOnsetGps => cOnset.Value;

		public static float StabRollCost => cStabRoll.Value;

		public static float StickSmoothing => cStickSmooth.Value;

		public static float ActuatorDamping => cActDamp.Value;

		public static float YawStopFactor => cYawStop.Value;

		public static float LowIasYawBoost => cLowYaw.Value;

		public static float LowIasYawBoostFullIas => cLowYawIas.Value;

		public static float LowIasYawBoostFalloff => cLowYawFall.Value;

		public static float TvcMoveCost => cTvcMove.Value;

		public static float RollNozzleHold => cRollTvc.Value;

		public static float RollNozzleWashout => cRollWash.Value;

		public static float RollTvcBudget => cRollTvcBudget.Value;

		public static float RollTvcBudgetCost => cRollTvcBudgetCost.Value;

		public static float RudderSideslipYawCap => (2f);

		public static bool NoseWheelFbw => cNwsFbw.Value;

		public static bool GroundStockMatch => cGndStock == null || cGndStock.Value;

		public static bool GearDownSlipProtect => cGearDownSlip == null || cGearDownSlip.Value;

		public static bool GroundIndi => cGndIndi == null || cGndIndi.Value;

		public static float GroundIndiGain => (cGndIndiK == null) ? 15f : Mathf.Max(0f, cGndIndiK.Value);

		public static float GroundIndiTau => (cGndIndiTau == null) ? 0.04f : cGndIndiTau.Value;

		public static float GroundPedalBrake => (cGndPedalBrake == null) ? 0.5f : Mathf.Clamp01(cGndPedalBrake.Value);

		public static bool FlaperonFlapSchedule => cFlapSched == null || cFlapSched.Value;

		public static float FlaperonFlapMax => (cFlapSchedMax == null) ? 23f : Mathf.Clamp(cFlapSchedMax.Value, 0f, 30f);

		public static bool GroundIndiLightNoseHold => cGndLightNose == null || cGndLightNose.Value;

		public static bool GroundIndiBrake => cGndIndiBrake == null || cGndIndiBrake.Value;

		public static float GroundIndiBrakeMax => (cGndIndiBrakeMax == null) ? 1f : Mathf.Clamp01(cGndIndiBrakeMax.Value);

		public static float GroundIndiBrakeFullSpeed => (cGndIndiBrakeFull == null) ? 7.7f : Mathf.Max(0f, cGndIndiBrakeFull.Value);

		public static float GroundIndiBrakeZeroSpeed => (cGndIndiBrakeZero == null) ? 12.9f : Mathf.Max(0f, cGndIndiBrakeZero.Value);

		public static float GroundDriftAllow => (cGndDriftAllow == null) ? 8f : cGndDriftAllow.Value;

		public static float GroundDriftGain => (cGndDriftGain == null) ? 0.2f : cGndDriftGain.Value;

		public static float GroundDriftLead => (cGndDriftLead == null) ? 0.25f : cGndDriftLead.Value;

		public static float GroundDriftFadeSpeed => (cGndDriftFade == null) ? 15f : cGndDriftFade.Value;

		public static float GroundYawRateMax => cGndYawRate.Value;

		public static float GroundLateralAccel => cGndLatAcc.Value;

		public static float GroundLineHold => cGndLine.Value;

		public static bool NozzleAnimation => (false);

		public static bool NozzleAnimationAI => (false);

		public static float AsymPitchMargin => cAsymPitchMargin.Value;

		public static bool AsymRollCoupling => cAsymRollCoupling.Value;

		public static bool RollPitchPriority => cRollPitchPriority.Value;

		public static float RollFightCost => cRollFightCost.Value;

		public static float NozzleSeamShade => (0.07f);

		public static float RollPitchFightCost => cPitchFight.Value;

		public static float RollPitchReserve => cRollPitchReserve.Value;

		public static bool NozzleIdleOpen => (true);

		public static float NozzleAreaRate => (1.2f);

		public static float YawPriority => cYawPri.Value;

		public static float RollLead => cRollLead.Value;

		public static float YawRateFF => cYawFF.Value;

		public static float OuterFlapShare => cOuterFlap.Value;

		public static float StabRollShare => cStabRollShare.Value;

		public static float CoupleAoA => cCouple.Value;

		public static float PedalBeta => cPedalBeta.Value;

		public static float KBeta => cKBeta.Value;

		public static float KRoll => cKRoll.Value;

		public static float KPitch => cKPitch.Value;

		public static float KYaw => cKYaw.Value;

		public static float KpAoA => cKpAoA.Value;

		public static bool StabTvcPairedSearch => cPairTrim.Value;

		public static float LateralStabReserve => cLatRes.Value;

		public static bool StabBranchSwitch => cBranch.Value;

		public static bool OvershootPitchPriority => cOvPrio.Value;

		public static float CouplingRollLimit => cCoupRoll.Value;

		public static float StabBranchMaxAoA => cBranchMaxAoA.Value;

		public static float StabBranchTarget => cBranchTarget.Value;

		public static float StabBranchPitchTol => cBranchTol.Value;

		public static float PitchAntiWindup => Mathf.Max(0f, cAntiWind.Value);

		public static bool AoACaptureThrustBrake => cThrBrake.Value;

		public static float AoACaptureBrakeBlend => cBrakeBlend.Value;

		public static float KiAoA => cKiAoA.Value;

		// Revoker 0.1.0: the single centreline rudder keeps its yaw authority to ~75 deg AoA (bench: rudder 25 deg = -40
		// deg/s^2 of stability-axis yaw at 500 km/h from 0 to 75 deg; reversed by 80), unlike the F-22E's canted fins (50-60)
		public static float RudderFadeStart => (75f);

		public static float RudderFadeEnd => (85f);

		public static float UnloadDecel => cUnloadDecel.Value;

		public static float UnloadRiseGain => cUnloadRise.Value;

		public static float MaxRollAccel => cRollAcc.Value;

		private static ConfigEntry<float> cDecelLook;

		private static ConfigEntry<float> cDecelStart;

		private static ConfigEntry<float> cDecelFull;

		private static ConfigEntry<float> cAsymCapDemand;

		private static ConfigEntry<float> cAsymCapSmooth;

		private static ConfigEntry<bool> cHiAoABrake;
		private static ConfigEntry<bool> cCmdShape;

		private static ConfigEntry<float> cShapeASpan;

		private static ConfigEntry<float> cShapeAKSmall;

		private static ConfigEntry<float> cShapeAKLarge;

		private static ConfigEntry<float> cShapeOnset;

		private static ConfigEntry<float> cShapeGSpan;

		private static ConfigEntry<float> cShapeGKSmall;

		private static ConfigEntry<float> cShapeGKLarge;

		private static ConfigEntry<bool> cRateBlend;

		private static ConfigEntry<float> cRateBlendAoA;

		private static ConfigEntry<float> cRateBlendStick;

		private static ConfigEntry<bool> cFuelTrim;

		private static ConfigEntry<float> cFuelTrimRate;

		private static ConfigEntry<bool> cTeBrake;

		private static ConfigEntry<bool> cTeBrakeFade;

		private static ConfigEntry<float> cGbElev, cGbInb, cGbCan, cAbElevUp, cAbInbDn;

		private static ConfigEntry<float> cEngOutCap;

		private static ConfigEntry<float> cEngDmgCap;

		private static ConfigEntry<float> cThrResp;

		private static ConfigEntry<bool> cEngAbGate;

		private static ConfigEntry<int> cStickRev;

		private static ConfigEntry<float> cStick24;

		private static ConfigEntry<float> cStick36;

		private static ConfigEntry<float> cRateBlendAoAMin;

		private static ConfigEntry<float> cRateBlendLiftG;

		private static ConfigEntry<float> cRateBlendBand;

		private static ConfigEntry<bool> cRateBlendNeg;

		private static ConfigEntry<float> cRateBlendAoANeg;

		private static ConfigEntry<float> cRateBlendAoAMinNeg;

		private static ConfigEntry<bool> cAuthCap;

		private static ConfigEntry<float> cAuthCapMargin;

		private static ConfigEntry<float> cAuthCapFloor;

		private static ConfigEntry<float> cHybridIndi;

		private static ConfigEntry<float> cHybridHiLead;

		private static ConfigEntry<bool> cAllocNozzleNewton;


		public static float DecelLookahead => Mathf.Max(0f, cDecelLook.Value);

		public static float DecelStart => cDecelStart.Value;

		public static float DecelFull => Mathf.Max(cDecelStart.Value + 0.5f, cDecelFull.Value);

		public static float AsymCapDemand => Mathf.Clamp(cAsymCapDemand.Value, 0.5f, 2f);

		public static float AsymCapSmoothing => Mathf.Max(0.05f, cAsymCapSmooth.Value);

		public static bool HighAoABrakeFloor => cHiAoABrake.Value;

		public static bool CmdShaping => cCmdShape.Value;

		public static float ShapeAoASpan => Mathf.Max(1f, cShapeASpan.Value);

		public static float ShapeAoAGainSmall => Mathf.Max(0.3f, cShapeAKSmall.Value);

		public static float ShapeAoAGainLarge => Mathf.Max(0.3f, cShapeAKLarge.Value);

		public static float ShapeOnsetSmall => Mathf.Clamp(cShapeOnset.Value, 0f, 1f);

		public static float ShapeGSpan => Mathf.Max(0.2f, cShapeGSpan.Value);

		public static float ShapeGGainSmall => Mathf.Max(0.3f, cShapeGKSmall.Value);

		public static float ShapeGGainLarge => Mathf.Max(0.3f, cShapeGKLarge.Value);

		public static bool HighAoARateBlend => cRateBlend.Value;

		public static float RateBlendAoA => Mathf.Clamp(cRateBlendAoA.Value, 10f, 64f);

		public static float RateBlendStick => Mathf.Clamp(cRateBlendStick.Value, 0.2f, 0.9f);

		// 2.0.12 stick curve: stick position of 24 and 36 deg AoA (defaults on the 72 deg per stick line). 24 within
		// 0.333 +-0.25, 36 within 0.5 +-0.2, and 36 at least 0.1 of stick past 24 (36 is pushed out if they come closer).
		public static float EngineOutAoACap => Mathf.Clamp(cEngOutCap.Value, 10f, 65f);

		public static float EngineDamagedAoACap => (cEngDmgCap != null) ? Mathf.Clamp(cEngDmgCap.Value, 10f, 65f) : 40f;

		public static float ThrottleResponseScale => (cThrResp != null) ? Mathf.Clamp(cThrResp.Value, 0.25f, 4f) : 1f;

		public static bool EngineAbAoAGate => cEngAbGate == null || cEngAbGate.Value;

		public static float StickAoA24 => Mathf.Clamp(cStick24.Value, 0.0833f, 0.7f);

		public static float StickAoA36 => Mathf.Clamp(Mathf.Max(Mathf.Clamp(cStick36.Value, 0.3f, 0.8f), StickAoA24 + 0.1f), 0.3f, 0.95f);

		public static float RateBlendAoAMin => Mathf.Clamp(cRateBlendAoAMin.Value, 10f, 64f);

		public static float RateBlendLiftG => Mathf.Max(1.1f, cRateBlendLiftG.Value);

		public static float RateBlendBand => Mathf.Clamp(cRateBlendBand.Value, 0.5f, 15f);

		public static bool HighAoARateBlendNeg => cRateBlendNeg.Value;

		public static float RateBlendAoANeg => Mathf.Clamp(Mathf.Abs(cRateBlendAoANeg.Value), 10f, 64f);

		public static float RateBlendAoAMinNeg => Mathf.Clamp(Mathf.Abs(cRateBlendAoAMinNeg.Value), 10f, 64f);

		public static bool LowThrustAoACap => cAuthCap.Value;

		public static float AuthCapMargin => cAuthCapMargin.Value;

		private static ConfigEntry<float> cRelTauAoA;

		private static ConfigEntry<bool> cElevDragShift;

		private static ConfigEntry<float> cElevPlateau;

		public static bool ElevatorStallDragShift => cElevDragShift != null && cElevDragShift.Value;

		public static float ElevatorPlateauCL => (cElevPlateau != null) ? Mathf.Clamp(cElevPlateau.Value, 0.5f, 1.5f) : 1f;

		public static float ReleaseTauAoA => cRelTauAoA.Value;

		public static float AuthCapFloor => Mathf.Clamp(cAuthCapFloor.Value, 5f, 65f);

		public static float HybridIndiGain => Mathf.Clamp(cHybridIndi.Value, 0f, 1.5f);

		public static float HybridIndiHighAoALead => (cHybridHiLead != null) ? Mathf.Clamp(cHybridHiLead.Value, 0f, 0.3f) : 0.15f;

		public static bool AllocNozzleNewton => cAllocNozzleNewton == null || cAllocNozzleNewton.Value;


		private static ConfigEntry<float> cYawRefKmh;

		private static ConfigEntry<float> cYawMargStart;

		private static ConfigEntry<float> cYawMargFull;

		private static ConfigEntry<float> cYawMargLow;

		public static float YawBrakeRefKmh => Mathf.Max(50f, cYawRefKmh.Value);

		public static float YawMarginStartKmh => cYawMargStart.Value;

		public static float YawMarginFullKmh => Mathf.Min(cYawMargFull.Value, cYawMargStart.Value - 1f);

		public static float YawMarginLowIas => Mathf.Max(1f, cYawMargLow.Value);

		private static ConfigEntry<float> cYawMidF;

		private static ConfigEntry<float> cYawMidLo;

		private static ConfigEntry<float> cYawMidHi;

		private static ConfigEntry<float> cYawMidRamp;

		public static float YawMidFactor => Mathf.Clamp(cYawMidF.Value, 0.2f, 2f);

		public static float YawMidLoKmh => cYawMidLo.Value;

		public static float YawMidHiKmh => Mathf.Max(cYawMidHi.Value, cYawMidLo.Value);

		public static float YawMidRampKmh => Mathf.Max(1f, cYawMidRamp.Value);

		// Ifrit 0.4.0 multiplayer: the Local* values are this machine's settings; the plain ones are what applies here, which
		// as a client is the host's policy (MpPolicy): the host's variant when it locks it, stock otherwise.
		public static bool HostAllowFcs => cHostAllowFcs == null || cHostAllowFcs.Value;

		public static bool HostLockVariant => cHostLock != null && cHostLock.Value;

		public static bool LocalStockPerformance => cProfile != null && cProfile.Value == PerformanceProfile.Stock;

		public static float LocalThrustScale => LocalStockPerformance ? 1f : cThrust.Value;

		public static float LocalFlareCountScale => LocalStockPerformance ? cFlareScaleStock.Value : cFlareScaleBuffed.Value;

		public static float LocalEWCapacityScale => LocalStockPerformance ? cEwCapStock.Value : cEwCapBuffed.Value;

		public static float LocalEWRechargeScale => LocalStockPerformance ? cEwRateStock.Value : cEwRateBuffed.Value;

		public static float LocalGunAmmoScale => LocalStockPerformance ? cGunAmmoStock.Value : cGunAmmoBuffed.Value;

		public static float LocalFuelUseScale => LocalStockPerformance ? 1f : cFuelUse.Value;

		public static float LocalDragScale => LocalStockPerformance ? 1f : cDrag.Value;

		public static float LocalDragReliefAoAHigh => LocalStockPerformance ? 0f : Mathf.Clamp(cDrAoA.Value, 0f, 0.5f);

		public static float LocalDragReliefTransonic => LocalStockPerformance ? 0f : Mathf.Clamp(cDrTrans.Value, 0f, 0.5f);

		public static bool StockPerformance => MpPolicy.StockForced || (MpPolicy.VariantLocked ? MpPolicy.Host.stock : LocalStockPerformance);

		public static float ThrustScale => MpPolicy.VariantLocked ? MpPolicy.Host.thrust : (MpPolicy.StockForced ? 1f : LocalThrustScale);

		public static float FlareCountScale => MpPolicy.VariantLocked ? MpPolicy.Host.flare : (MpPolicy.StockForced ? 1f : LocalFlareCountScale);

		public static float EWCapacityScale => MpPolicy.VariantLocked ? MpPolicy.Host.ewCap : (MpPolicy.StockForced ? 1f : LocalEWCapacityScale);

		public static float EWRechargeScale => MpPolicy.VariantLocked ? MpPolicy.Host.ewRate : (MpPolicy.StockForced ? 1f : LocalEWRechargeScale);

		public static float GunAmmoScale => MpPolicy.VariantLocked ? MpPolicy.Host.gun : (MpPolicy.StockForced ? 1f : LocalGunAmmoScale);

		public static float FuelUseScale => MpPolicy.VariantLocked ? MpPolicy.Host.fuel : (MpPolicy.StockForced ? 1f : LocalFuelUseScale);

		public static float DragScale => MpPolicy.VariantLocked ? MpPolicy.Host.drag : (MpPolicy.StockForced ? 1f : LocalDragScale);

		// 2.2.3 (rev 2): AoA drag relief as a curve over each part's local AoA. Up to DragReliefFadeStart it is DragReliefAoA
		// (LowAoADragRelief, both profiles); across the fade band it goes over to DragReliefAoAHigh: none in Stock (post-stall
		// drag is Aryx's), the old flat AoADragRelief in Buffed.
		public static float DragReliefAoA => MpPolicy.FcsAllowed ? Mathf.Clamp(cStockDrAoA.Value, 0f, 0.5f) : 0f;

		public static float DragReliefAoAHigh => MpPolicy.VariantLocked ? MpPolicy.Host.drAoAHigh : (MpPolicy.StockForced ? 0f : LocalDragReliefAoAHigh);

		public static float DragReliefFadeStart => Mathf.Clamp(cStockDrFade0.Value, 5f, 60f);

		public static float DragReliefFadeEnd => Mathf.Clamp(Mathf.Max(cStockDrFade1.Value, cStockDrFade0.Value + 1f), 6f, 70f);

		public static float HighAoALatMargin => (cLatMargin != null) ? Mathf.Clamp(cLatMargin.Value, 0.5f, 1.5f) : 0.86f;

		public static float HighAoALatMarginStart => (cLatMargin0 != null) ? Mathf.Clamp(cLatMargin0.Value, 0f, 89f) : 25f;

		public static float HighAoALatMarginFull => (cLatMargin1 != null) ? Mathf.Clamp(cLatMargin1.Value, HighAoALatMarginStart + 0.5f, 90f) : 30f;

		private static ConfigEntry<float> cLatMargin, cLatMargin0, cLatMargin1;

		public static bool DeceleronSplitBudget => cSplitBudget == null || cSplitBudget.Value;

		private static ConfigEntry<bool> cSplitBudget;

		public static float AoADragBump => (cDragBump != null) ? Mathf.Clamp(cDragBump.Value, 0f, 1f) : 0.3f;

		public static float AoADragBumpStart => (cDragBump0 != null) ? Mathf.Clamp(cDragBump0.Value, 0f, 89f) : 18f;

		public static float AoADragBumpPeak => (cDragBump1 != null) ? Mathf.Clamp(cDragBump1.Value, AoADragBumpStart, 89f) : 24f;

		public static float AoADragBumpEnd => (cDragBump2 != null) ? Mathf.Clamp(cDragBump2.Value, AoADragBumpPeak + 1f, 90f) : 35f;

		private static ConfigEntry<float> cDragBump, cDragBump0, cDragBump1, cDragBump2;

		public static float DragReliefTransonic => MpPolicy.VariantLocked ? MpPolicy.Host.drTrans : (MpPolicy.StockForced ? 0f : LocalDragReliefTransonic);


		public static float KpG => cKpG.Value;

		public static float KiG => cKiG.Value;

		public static float FlaperonEffectiveness => (1f);

		public static float StabilatorEffectiveness => (1f);

		public static float ActuatorScale => cActuator.Value;

		public static bool AsymLimiter => cAsymOn.Value;

		public static bool AsymRollHeadroom => cAsymRollHead.Value;

		public static float AsymReserve => cAsymReserve.Value;
		public static float AsymCoupledReserveMax => Mathf.Clamp(cAsymCplResMax.Value, 0.2f, 0.9f);
		public static float AsymStopTime => Mathf.Clamp(cAsymStopTime.Value, 0f, 5f);
		public static float AsymAoAWeight => Mathf.Clamp(cAsymAoAWeight.Value, 0.25f, 4f);

		public static float AsymCapFloor => Mathf.Abs(cAsymFloor.Value);

		public static float AsymCapFloorNeg => Mathf.Abs(cAsymFloorNeg.Value);

		public static float AsymTighten => Mathf.Max(0.1f, cAsymTighten.Value);

		public static float AsymRelax => Mathf.Max(0.1f, cAsymRelax.Value);

		public static float AsymRecoveryRate => Mathf.Max(1f, cAsymRec.Value);

		public static float AsymMinIas => cAsymMinIas.Value;

		public static float AsymRollReserveAccel => Mathf.Max(0f, cAsymRollAcc.Value);

		public static float AsymYawReserveAccel => Mathf.Max(0f, cAsymYawAcc.Value);

		public static float AsymAxisBoost => cAsymBoost.Value;

		public static float AsymDeadband => cAsymDead.Value;

		public static float AsymFeedForward => cAsymFF.Value;

		public static float AsymFFWashout => cAsymFFTau.Value;

		public static float AsymFFMaxBeta => cAsymFFBeta.Value;

		public static float AsymFFMaxAoA => cAsymFFAoA.Value;



		public static float LowQRegRelief => cLowQReg.Value;

		public static float RollStopFactor => cRollStop.Value;

		public static float RollAccelUse => Mathf.Clamp(cRollAccUse.Value, 0.1f, 0.95f);

		public static float RollAccelUseLowIas => Mathf.Clamp(cRollAccUseLow.Value, 0.1f, 0.95f);

		// Revoker 0.2.8 (TrailingEdgeBrakeConfig): the F-22E's brake surface configurations, in the air and on the runway.
		// Off with ThrottleIndependentAirbrake off (the airbrake is then fully stock).
		public static bool TeBrakeConfig => cTeBrake == null || cTeBrake.Value;

		public static float GroundAeroBrake => Mathf.Clamp01((1f));

		public static float GroundBrakeMinSpeed => (0f);

		public static float GroundBrakeFlap => (cGbElev != null) ? Mathf.Clamp(cGbElev.Value, 0f, 30f) : 30f;

		public static float GroundBrakeInboard => (cGbInb != null) ? Mathf.Clamp(cGbInb.Value, 0f, 25f) : 25f;

		public static float GroundBrakeStab => (cGbCan != null) ? Mathf.Clamp(cGbCan.Value, 0f, 80f) : 60f;

		public static float GroundBrakeRudder => (25f);

		public static float HighLiftMaxAoA => cHlMaxAoA.Value;

		public static float SlatAoAStart => cSlatStart.Value;

		public static float SlatAoAFull => cSlatFull.Value;

		public static float HighLiftHighAoA => Mathf.Clamp01(cHlHighAoA.Value);

		// Su-30 0.1.0: the F-22E FCS's GroundAeroBrakeTVC (10 deg in the aft-stick sense: the nozzles sit up with the stabs in the rollout brake)
		public static float GroundBrakeTvc => (10f);

		public static float AirBrakeElevonUp => (cAbElevUp != null) ? Mathf.Clamp(cAbElevUp.Value, 0f, 30f) : 20f;

		public static float AirBrakeInboardDown => (cAbInbDn != null) ? Mathf.Clamp(cAbInbDn.Value, 0f, 25f) : 25f;

		public static float GroundBrakeThrottle => (-1f);

		public static bool GroundBrakeLog => (false);

		public static bool AirbrakeSplit => cGndBrakeSplit.Value;

		public static string VentDoors => ("Bypass");

		public static float RotationMinSpeed => Mathf.Max(20f, cRotMinSpeed.Value);

		public static bool RevFlowRollSense => cRevRoll.Value;

		public static float AsymRollTau => cAsymRollTau.Value;

		public static float AsymRelaxBoost => cAsymRelaxBoost.Value;

		public static float RollPitchFF => cRollPitchFF.Value;

		public static float RollPitchFFAoA => cRollPitchFFAoA.Value;

		public static float GUndershootCapture => cGUnder.Value;

		public static float RollPitchFFMax => Mathf.Max(1f, cFFMax.Value);

		public static float RollPitchFFStabCost => cFFStabCost.Value;

		public static void Init(ConfigFile cfg, ConfigFile dev, bool devUnlocked, bool devFresh, ManualLogSource log)
		{
			Log = log;
			// Su-30 0.1.4: read before any Bind (a Bind can save the file, which rewrites the default lines)
			string rssOldDef = FileDefaultLine(cfg.ConfigFilePath, "RollStopSideslip");
			// Compass 0.1.1 / 0.1.2: one-time moves of changed defaults. The player file carries CompassConfigRevision, the dev file
			// CompassDevRevision; a file without its key was written by 0.1.0 (player) / 0.1.0-0.1.1 (dev). Values set by hand stay.
			bool cmpPlayerExisted = false;   // Ternion 0.1.0: no Compass-era files to migrate
			bool cmpPlayerHasRev = FileHasKey(cfg.ConfigFilePath, "CompassConfigRevision");
			// Su-30 0.1.9: a player file written by 0.1.0-0.1.8 (no Su30ConfigRevision line yet)
			bool suMainOld = FileHasKey(cfg.ConfigFilePath, "ThrottleIndependentAirbrake") && !FileHasKey(cfg.ConfigFilePath, "Su30ConfigRevision");
			bool cmpDevOld = false;   // Ternion 0.1.0: no Compass-era files to migrate
			bool cmpDevHadRev = devUnlocked && !devFresh && !cmpDevOld;
			cProfile = cfg.Bind("1. General", "PerformanceProfile", PerformanceProfile.Stock, "Stock (default) = the game's Su-30VXN thrust, drag and fuel burn. Buffed = thrust x1.15 (afterburner included), fuel burn x0.65 at every throttle setting (35 % less fuel per throttle setting) and parasitic drag x0.90 (10 % less). Countermeasures and gun ammo come from the matching Countermeasures section below. The flight control is the same in both. Switch before spawning: thrust, drag, countermeasures and ammo are set when a jet spawns and stay until it respawns; fuel burn follows the setting live.");
			cThrIndepBrake = cfg.Bind("1. General", "ThrottleIndependentAirbrake", defaultValue: true, "Su-30 0.1.9: true (Tom 9 Nov: the airbrake independent of the throttle, as on his other FCS jets; a 0.1.0-0.1.8 file holding false is moved once). Su-30 0.1.0: false by default (Tom: no special airbrake logic, the Su-30's own dorsal airbrake as the game has it). false = fully stock: the game opens the airbrake at zero throttle and the brake key is the wheel brake. What opens the Su-30's airbrake. true = the brake key / axis only, in the air and on the ground; idle or zero throttle never opens it. false = the stock rule only: throttle at zero opens it; the brake key is the wheel brake only (it no longer opens it, so Brake and Countermeasures can share a controller button). Only while the FCS flies the jet.");
			cTeBrake = cfg.Bind("1. General", "TrailingEdgeBrakeConfig", defaultValue: false, "Su-30 0.1.0: false by default (Tom: no special airbrake logic). The brake also sets the control surfaces as the F-22E does, on the runway: with the main wheels down and the brake held (stick not pulled back, from touchdown on) the flaperons go fully trailing edge up to dump the wing lift, the stabs trailing edge up (as the Raptor's), the canards leading edge down to their stop, the nozzles up and the rudders toed in for drag. In the air the brake is the dorsal airbrake alone (the Su-30 has one flaperon pair, so there is no crow set to cancel its lift). Needs ThrottleIndependentAirbrake = true (with it false the airbrake is fully stock). false = the dedicated airbrake only. Takes effect at once.");
			cEnabled = cfg.Bind("1. General", "Enabled", defaultValue: true, "Replace the stock fly-by-wire with the Su-30VXN FCS (player aircraft only).");
			cHostAllowFcs = cfg.Bind("8. Multiplayer (host)", "HostAllowFcs", defaultValue: true, "Read on the HOST (or dedicated server). true = players who run this plugin may fly their Su-30s with its control laws on this server. false = every client flies the stock fly-by-wire. A client whose host does not run this plugin always flies stock (fly-by-wire, thrust, drag, fuel burn, countermeasures and ammo), whatever its own settings. The host's own jet is bound by this switch too. While this or HostLockVariant is on, the server is flagged as modded (server browser / NOMM 'Modded' tag).");
			// Ifrit 0.4.7: HostLockVariant on by default. A player file written by an older build still says "# Default value:
			// false" above the key; if it also still holds false (the old default), it is moved to true once. The file is then
			// saved, which rewrites the default line, so a later false set by the player stays.
			bool hostLockOldFile = HostLockOldDefault(cfg.ConfigFilePath);
			cHostLock = cfg.Bind("8. Multiplayer (host)", "HostLockVariant", defaultValue: true, "Read on the HOST. On by default. true = every client gets the host's variant: the host's PerformanceProfile with its thrust and fuel-burn scaling and the host's countermeasure and gun-ammo capacities from the matching Countermeasures section. false = every client flies the Stock variant with stock countermeasures and ammo (their own profile and capacity settings are ignored while connected); the host flies Stock on its own server as well. Changes are sent to the clients at once and apply at the next spawn.");
			if (hostLockOldFile)
			{
				if (!cHostLock.Value)
				{
					cHostLock.Value = true;
				}
				cfg.Save();
				Log.LogInfo("Ifrit 0.4.7 defaults: HostLockVariant on.");
			}
			cRateOnly = cfg.Bind("1. General", "PitchRateCommandOnly", defaultValue: false, "2.0.6: true = the pitch stick is a pitch-rate command in every mode (gear up or down, on the runway, Stability Assist on or off), like the gear-down law. With Stability Assist on, the AoA and G limits and AoA protection still apply; with it off (MPO) the MPO limits apply as before. false = AoA/G command with the gear up (default). Takes effect at once.");
			cPitchLaw = cfg.Bind("1. General", "PitchCommandLaw", PitchCommandLaw.GAndPitchRate, "2.1.0: the gear-up pitch law (Stability Assist on). GAndPitchRate (default) = the stick commands pitch rate where full G is out of reach (low IAS) and G where it is in reach, blended as before; no AoA command - the AoA limits, the low-thrust / asymmetric-load caps and AoA protection still apply as limits. GAoAAndPitchRate = the 1.9-2.0 law: AoA command at low IAS (stick curve in section 5), G command at high IAS, pitch-rate command past the hand-over AoA. AoAThenRateHold (2.1.8) = as GAoAAndPitchRate, but once the stick is steady and the AoA has settled on its command, the pitch rate the jet has then is held instead of the AoA (the AoA drifts with speed); moving the stick goes back to AoA command. PitchRateCommandOnly = true overrides all of them. Takes effect at once.");
			cFuelBal = dev.Bind("6. Fuel", "FuelBalancing", defaultValue: true, "2.0.6: move fuel between the healthy tanks to hold the centre of mass where the clean jet has it (centreline; stock fore/aft position without stores), against store asymmetry, a part shot off or a leaking tank. Only between attached, non-leaking, non-burning tanks; mass is conserved. All Ifrits simulated on this machine.");
			cEngOutCap = dev.Bind("Asymmetric load", "EngineOutAoACap", 22f, "Ifrit 0.4.0: 22 (was 30; the Ifrit's engines sit 1.19 m off the centreline, one at AB is 24 deg/s^2 of yaw against 10-24 from the tails at 200-300 km/h and 20 deg AoA; bench one-engine pulls from 300 km/h: 18-25 hold without the throttle cut, 30 bleeds to 50-85 m/s and departs). 2.0.13: with an engine out the AoA limit is this, both signs, steady (plus whatever an aerodynamic asymmetry from damage needs on top). The dead engine's thrust yaw is left to the VMC protection (throttle, last resort) instead of the AoA cap, which made the limit follow speed and throttle (12 <-> 65 deg within seconds in 2.0.12).");
			cEngDmgCap = dev.Bind("Asymmetric load", "EngineDamagedAoACap", 40f, "Ifrit 0.4.2: 40 (first 0.4.1 / 2.4.1 cut: 30, the engine-out cap; a running engine's split is a third to a half of a dead one's, and below the speed where the fins could not hold it the thrust match now takes over on prediction, so the cap only says where the full split is traded for matched thrust). Ifrit 0.4.1: with one engine running weak (damaged, not destroyed: under 85 % of the other's thrust, released above 92 %, 1 s filter) the AoA limit is this fixed value, both signs, instead of the dynamic thrust cap (which fell to 12 deg within 5 s in the 22 Oct 22:08 log and then followed IAS). The dead-engine case keeps EngineOutAoACap. The game caps a damaged engine at 50-100 % of its RPM range by its condition (no afterburner below condition ~0.6), so the weak engine makes 35-75 % of the good one's thrust at AB. Bench, full pulls from 300-400 km/h with the left engine at condition 0 / 0.5: 26-30 hold with 5-8 deg peak sideslip, 36 saturates the yaw once the speed is gone (throttle cut a third of the time, one departure), 45 departs.");
			cThrResp = dev.Bind("5. Airframe", "ThrottleResponseScale", 1f, "Ifrit 0.4.1: throttle response (Ifrit 0.4.5: back to the game's own, 1) for every variant (Stock included): the engines' spool rate (RPM per second toward the throttle's target) and the afterburner's light-up / fade speed are multiplied by this at the airframe setup. 1 = the mod's own values.");
			cEngAbGate = dev.Bind("Asymmetric load", "EngineAbAoAGate", true, "Ifrit 0.4.1: thrust-match limiter. With one engine down on thrust (predicted from the engines' damage state: an engine's reachable RPM is minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5), the dry thrust follows the RPM and the afterburner needs 85 % of maxRPM, i.e. condition above 0.55) and the AoA above EngineDamagedAoACap, the throttle input is held where the two engines still make the same thrust, so there is no thrust split and no AoA cap for it: you keep the full AoA, and the thrust comes back once you bring the AoA down to the cap yourself (on above cap + 2, off under cap + 0.5). Below the cap the throttle is free; the thrust split then brings the fixed EngineDamagedAoACap. With an engine OUT there is nothing to match: above EngineOutAoACap the throttle is held at military power (the afterburner's start). The VMC throttle protection (measured sideslip run-away) stays as the last resort. false = 0.4.0 (dynamic cap from the thrust split, nose pushed down).");
			cFuelTrim = dev.Bind("6. Fuel", "FuelTrim", defaultValue: true, "2.0.12: when the yaw or roll authority runs short (an engine out, a fin or wing section shot off), the balancing moves the CG sideways with fuel instead of holding it on the centreline: toward the live engine to shorten its thrust lever arm, or against an aerodynamic asymmetry. Only once the yaw or roll usage passes 0.75, only as far as evens out the roll and yaw usage (a sideways CG costs roll: ~15 deg/s^2 per g per 10 cm), at most 30 cm; back to the centreline once both are under 0.5 for 2 s. false = centreline always (2.0.6-2.0.11).");
			cFuelTrimRate = dev.Bind("6. Fuel", "FuelTrimRate", 240f, "2.0.12: fuel moved per second (kg/s, in total) while the fuel trim works (normal balancing: FuelTransferRate). Ifrit 0.4.1: 240 (was 120).");
			cFuelRate = dev.Bind("6. Fuel", "FuelTransferRate", 80f, "2.0.6: total fuel the balancing moves per second (kg/s). Ifrit 0.4.1: 80 (was 40).");
			cFuelTolLat = dev.Bind("6. Fuel", "FuelBalanceLateralTolerance", 0.01f, "2.0.6: lateral CG error (m) the balancing weighs as one unit; it leaves the fuel alone inside half of it.");
			cFuelTolLong = dev.Bind("6. Fuel", "FuelBalanceLongitudinalTolerance", 0.03f, "2.0.6: fore/aft CG error (m) the balancing weighs as one unit; it leaves the fuel alone inside half of it.");
			cGndTvcRY = dev.Bind("5. Airframe", "GroundTvcRollYaw", defaultValue: true, "Su-30 0.2.3 (Tom 9 Nov 23:16: on the ground the nozzles followed only the pitch stick). true = on the ground the roll stick and rudder pedals move the nozzles too, directly like the pitch stick: left stick or left pedal = left nozzle jet up, right nozzle jet down (the canted nozzles' differential, which rolls and yaws the jet left); stick and pedals add, each nozzle clamped to its own +-15 deg. Takes effect at once. false = pitch only on the ground (0.1.0-0.2.2).");
			cFuelCgHold = dev.Bind("6. Fuel", "FuelCgHold", defaultValue: true, "Su-30 0.2.1 (Tom 9 Nov 22:45: the CG should not move as the fuel runs out). With the stock spread (every tank at the same fraction) the Su-30's CG walks about 30 cm aft from full tanks to empty, since its fuel sits 1.3 m ahead of the dry CG. true = the fuel balancing holds the CG fore/aft where the clean jet has it at FuelCgHoldRef of its fuel, whatever is left, as long as the forward tanks can (down to about 10 % fuel), and the jet keeps one balance through the flight. Needs FuelBalancing. false = the stock spread's CG at the present fuel (it moves with fuel).");
			cFuelCgHoldRef = dev.Bind("6. Fuel", "FuelCgHoldRef", 0.7f, "Su-30 0.2.1: the fuel fraction whose CG (clean jet, stock spread) FuelCgHold holds. 0.7 = the bench's calibration point, where CgShiftAft and the airframe keys were tuned. 1 = full-tank CG (about 7 cm further forward on the Su-30).");
			cFuelAttached = dev.Bind("6. Fuel", "FuelFromAttachedTanksOnly", defaultValue: true, "2.0.6: engines draw fuel only from tanks still attached, and the fuel gauge / fuel checks count only those (against the full original capacity); a tank on a part that has been shot off is not refuelled. false = stock (a shot-off tank's fuel keeps feeding the engines and showing on the gauge).");
			cFuelBalLog = dev.Bind("0. Debug", "FuelBalanceLog", defaultValue: false, "2.0.6: log the fuel balancing's plan (CG error and the fuel it moves) when it starts and every 10 s while it works.");
			cFlareScaleStock = cfg.Bind("3. Countermeasures - Stock profile", "FlareCountScale", 1f, "Flare capacity multiplier (the load at spawn and what rearming refills to). Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cGunAmmoStock = cfg.Bind("3. Countermeasures - Stock profile", "GunAmmoScale", 1f, "Gun ammunition multiplier: magazine capacity, so the load at spawn and what rearming refills to. Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cEwCapStock = cfg.Bind("3. Countermeasures - Stock profile", "EWCapacityScale", 1f, "Electrical storage multiplier for the EW/jammer power supply (more jamming time per charge). Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cEwRateStock = cfg.Bind("3. Countermeasures - Stock profile", "EWRechargeScale", 1f, "Recharge-rate multiplier for the EW/jammer power supply (engine-driven charging). Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cFlareScaleBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "FlareCountScale", 10f, "Flare capacity multiplier (the load at spawn and what rearming refills to). Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cGunAmmoBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "GunAmmoScale", 2f, "Gun ammunition multiplier: magazine capacity, so the load at spawn and what rearming refills to. Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cEwCapBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "EWCapacityScale", 1.5f, "Electrical storage multiplier for the EW/jammer power supply (more jamming time per charge). Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cEwRateBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "EWRechargeScale", 1.2f, "Recharge-rate multiplier for the EW/jammer power supply (engine-driven charging). Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cTelemetry = dev.Bind("0. Debug", "Telemetry", defaultValue: false, "Write a 50 Hz CSV of the FCS state to BepInEx/Ifrit_FCS_<time>.csv. 0.4.0: switching it off in flight stops the rows (the file stays open) and switching it on resumes them, so its cost can be timed in flight.");
			cAIFcs = cfg.Bind("2. AI", "AIFlightControl", defaultValue: false, "Off by default (every AI Su-30 in a mission would otherwise fly this FCS). true = AI-flown Su-30s simulated on this machine (no human pilot) get the full FCS with a pitch-rate command law (the AI's stick is a pitch-rate command; AoA/G protection, coordination and the allocator as for a player). Takes effect at the next spawn. Experimental.");
			cAILog = dev.Bind("0. Debug", "AIFlightLog", defaultValue: false, "Every 20 s, log one summary line per AI F-22E the FCS is flying (mode, IAS, AoA and g range, time in AoA protection), so AI jets can be checked from BepInEx/LogOutput.log after any mission. Each F-22E also logs once which control path it takes when it first reaches the flight-control hook.");
			cAIDirectRoll = cfg.Bind("2. AI", "AIDirectRoll", defaultValue: true, "AI Su-30s only: the AI's roll stick drives the velocity-vector roll rate almost directly (up to the jet's roll authority, still capped by the roll limits).");
			cAIGunRollFade = cfg.Bind("2. AI", "AIGunRollFade", defaultValue: true, "AI Su-30s only: while the AI points its nose (guns) at a moving target, its roll stick is faded out when the aim error is small, so it does not wobble the gun solution.");
			cAoAPos = dev.Bind("2. Envelope", "AoALimitPositive", 75f, "Su-30 0.3.0: 75 (Tom 10 Nov 16:46: \"+75 aoa limit is very usable now, so set it as default\"; a file holding 70 moves to 75 once). Su-30 0.2.0: 70 (Tom 9 Nov 21:28 after flying 75: \"75 is flyable but a little too sketchy, yaw rate is slow, and the jet struggles to follow inverted corkscrews. I think 70 is nice\"; a file holding 75 moves to 70 once). Su-30 0.1.10: 70 (Tom 9 Nov 11:18, from his own file: migrate his settings). Su-30 0.1.7: 75 (Tom 9 Nov: \"max aoa at 75 if possible\"). Bench on 0.1.6, 75 against 70: full-aft pulls from 150-250 km/h hold 75.0-75.1; the regression set, the stop / reversal sweep and the corkscrews equal or better on sideslip (65 deg roll releases 7.0 / 5.9 -> 3.7 / 2.8); falling leaf 42-43 -> 39 deg/s at ~12 km/h less IAS; a full-aft rolling pull from ~200 km/h goes past the limit to ~82 (72 at the 70 limit) through StickCancelsLimit, with the same ~17 deg of sideslip. Su-30 0.1.0: 70 (bench, the highest usable: full-aft pulls hold 70 within 0.1-0.2 deg rms from 200, 300 and 450 km/h; loaded rolls at the limit (full aft + full roll and reversal) stay within 69.3-70.3 deg, sideslip 4.7-7.9 deg; push-overs from 70 come down at the 48 deg/s cap with a smooth rate curve. 75 holds from 300 km/h up but a loaded roll entered at 200 km/h dips to 70.6 with 20 deg of sideslip; 80 does not hold below ~20 m/s and rolls there reach 31 deg of sideslip). At idle (no thrust for the nozzles) the low-thrust cap holds 45. Ternion lineage: Ternion 0.1.5: 55 (Tom). 0.1.3: 50 (Tom: 50 is a good limit). Bench with LowThrustCapMargin 4: full-aft pulls hold 50 down to ~150-175 km/h; 55 holds to ~165-185 km/h; 60 reaches 58-60 but nothing holds it below ~180 km/h. Ternion 0.1.0: 40 (bench, the highest usable): full-aft pulls from 200-550 km/h hold 40.0 (+-0.04-0.13 deg rms) with the canards at 28-33 of 60 and the elevators at their 35 TE-up stop only below ~250 km/h; loaded rolls at the limit (full aft + full roll and reversal, 250-450 km/h) stay within 39.4-40.8 at 27-29 deg/s velocity-vector roll, sideslip <= 1.6. At 45 the loaded rolls swing 39.6-46.0 and the elevators pin and chatter below 350 km/h; at 50-55 the canards reach their 60 stop and the limit is not held. Below ~180 km/h the authority cap (LowThrustAoACap) takes it lower. AoA limit (deg) in AoA/G command and gear-down rate command.");
			cAoANeg = dev.Bind("2. Envelope", "AoALimitNegative", 65f, "Su-30 0.2.0: 65 (Tom 9 Nov 21:28: \"-65 is very flyable, not much issues found, so set it at default\"; a file holding 45 moves to 65 once). Su-30 0.1.0: 45 (bench: full pushes hold -45 at 200-300 km/h within 0.2-0.3 deg rms with the stabs short of their 30 deg stop; at -50 the stabs sit on the stop and the AoA hangs 0.6-0.7 deg short). Ternion lineage: Ternion 0.1.0: 40 (bench: pushes from a 60 deg climb at 220-300 km/h hold -40.1, push + full roll -40.5..-41.0 at 21-24 deg/s, sideslip <= 1.1; the elevators sit on their 30 TE-down stop, the canards keep 20+ deg of margin in a plain push and reach their 45 stop in push-rolls). Negative AoA limit magnitude (deg).");
			cStagger = dev.Bind("5. Airframe", "ElevonStallStagger", false, "Su-30 0.1.0: off. The Su-30's flaperons keep the stock Su-30_wing curve (Tom asked camber coupling only); its outer flaps are fixed. Ternion lineage: Ternion 0.1.0: on (Tom: the Revoker's staggered, knee-preserving stall on the FS-3's flaperon_L/R (inboard, 26) and rollervon_L/R (outboard, 45)). Compass 0.1.0: off (Revoker-only: staggered stall of the FS-12's two trailing-edge elevon pairs). The T/A-30's ailerons and flaps keep the stock Trainer_wing curve.");
			cStallInner = dev.Bind("5. Airframe", "InnerElevonReversalAoA", 26f, "Revoker 0.2.22: 26 (0.2.21: 37, 0.2.20: 32; migrated once). Inboard elevon (flap_L/R) lift-peak local AoA (deg, 20-80) with ElevonStallStagger. 0.2.22 moves only the plateau above the wing foil's 17.2 deg knee (the curve below it and the 1.10 peak CL are untouched). Model, outboard at 45, both pairs + camber, measured arms: 26 maximises the worst-case velocity-vector roll authority over 10-55 deg (0.26 small / 0.18 full deflection, vs 0.03 / 0.01 unstaggered); the inboard pair reverses at ~27.7 (33.2 at full deflection) and is flown reversed from there. Next spawn.");
			cStallOuter = dev.Bind("5. Airframe", "OuterElevonReversalAoA", 45f, "Revoker 0.2.20: outboard elevon (aileron_L/R) lift-peak local AoA (deg) with ElevonStallStagger. 45 = the wing foil's own (44.6). Next spawn.");
			cFlapCamber = dev.Bind("5. Airframe", "FlapCamberCoupling", 0.63f, "Su-30 0.1.0: 0.63 = the thin-airfoil host share for LFlap_struct / RFlap_struct (5.0 m2) behind Left/Right_wing_inner (5.4 m2): chord fraction 5.0 / 10.4 = 0.48, flap effectiveness tau 0.81, (0.81 x 10.4 - 5.0) / 5.4 = 0.63. The gear-driven 15 deg landing flap on the same part cambers the panel the same way. Ternion lineage: Ternion 0.1.0: 0.62 = the Revoker's thin-airfoil host share on the FS-3's geometry: flaperon_L/R (3.71 m2, chord fraction 0.47, tau 0.80) behind wing_2b (4.24 m2): (0.80 x 7.95 - 3.71) / 4.24. Compass 0.1.0: each flap (flap_L/R, an allocator effector with FlapFlaperons) also cambers the wing1 section ahead of it: wing1's lift normal turns by K x the flap deflection. Thin-airfoil flap effectiveness of the 0.25-chord flap (1.8 of 7.2 m2) is 0.55; host share (0.55 x 7.2 - 1.8) / 5.4 = 0.40 (the stock high-lift device uses 0.5: wing1 -10 deg at flap -20). 0 = off.");
			cAilCamber = dev.Bind("5. Airframe", "AileronCamberCoupling", 0f, "Su-30 0.1.0: 0 (no aileron pair on the Su-30; the outer flaps do not move). Ternion lineage: Ternion 0.1.0: 0.66 = the same rule for rollervon_L/R (3.30 m2, chord fraction 0.54, tau 0.84) behind wing_3b (2.81 m2): (0.84 x 6.11 - 3.30) / 2.81. Compass 0.1.0: each aileron (aileron_L/R) also cambers the wingtip section ahead of it: the wingtip's lift normal turns by K x the deflection. Thin-airfoil flap effectiveness of the 0.47-chord aileron (3.0 of 6.37 m2) is 0.76; host share (0.76 x 6.37 - 3.0) / 3.37 = 0.55. Zero with the ailerons centred. 0 = off.");
			cDownwash = dev.Bind("5. Airframe", "StabDownwash", 0.6f, "Su-30 0.1.0: 0.6 = the F-22E FCS's stab downwash (Tom: the Raptor's downwash modelling): the flow at the stabs is turned down by eps = 0.6 x AoA, faded out from 45 to 60 deg AoA, free stream above. The stabs' stock airflowChanneling (0.6 rad, a fixed 34 deg pull of their inflow toward the fuselage axis) is set to 0 while the FCS flies, so this model replaces it. Applied in the game's aero as a correction force on the stabs and in the FCS's stab models. 0 = off. Respawn after changing. Ternion lineage: Ternion 0.1.0: 0 (Tom: no stab downwash). The FS-3's tail flies in the free stream, as in the stock game. Compass 0.1.0: wing downwash at the tail (elevator_L/R and the fixed hstab_L/R), d eps / d alpha below the wing's stall. Nuclear Option gives every part the free-stream flow, so a tail at 0 deflection sees the body AoA 1:1; a real tail sits in the wing's downwash. 0.45: the T/A-30's low-aspect straight wing with the tail 5 m behind it, close to the wing plane (thin-wing estimate 2 CLa / (pi A) ~ 0.4-0.5). Applied in the game's aero as a correction force on the tail parts and in the FCS's elevator models. 0 = off (the stock jet). Respawn after changing.");
			cDownwashLift = dev.Bind("5. Airframe", "StabDownwashLiftDriven", false, "Su-30 0.1.0: false = the F-22E's linear model (eps = k x AoA, fade 45-60), not the Compass's lift-driven one. Ternion lineage: Compass 0.1.0: true = the downwash follows the wing's lift, eps = StabDownwash x CL_wing(alpha) / CLalpha_wing (Trainer_wing: the same as StabDownwash x alpha up to the 17 deg stall, then it tracks the wing's post-stall CL 0.8-1.0 to 45 deg, faded out 60-90 deg). false = the F-22E's linear model (StabDownwash x alpha, faded out 45-60 deg).");
			cFinCamber = dev.Bind("5. Airframe", "FinCamberCoupling", 0.60f, "Su-30 0.1.0: 0.60 = the same rule for LRudder_struct / RRudder_struct (1.5 m2) behind Left/Right_vertical_stabiliser (2.0 m2): chord fraction 0.43, tau 0.77, (0.77 x 3.5 - 1.5) / 2.0 = 0.60. Ternion lineage: Ternion 0.1.0: 0.54 = the same rule for rudder_L/R (5.29 m2, chord fraction 0.33, tau 0.69) behind VStab_L/R (10.58 m2): (0.69 x 15.87 - 5.29) / 10.58. Compass 0.1.0: each rudder also cambers the fixed fin ahead of it (rudder_L/R on vstab2_L/R): the fin's lift normal turns by K x the rudder deflection. Thin-airfoil flap effectiveness of the 0.52-chord rudder (1.35 of 2.61 m2) is 0.78; host share (0.78 x 2.61 - 1.35) / 1.26 = 0.55. Zero with the rudders centred. 0 = off.");
			cStrakeVortex = dev.Bind("5. Airframe", "StrakeVortexLift", false, "Ternion 0.1.0: off (the T/A-30's wingroot1 strakes; the FS-3 has none). Compass 0.1.1: the wing-root strakes (wingroot1_L/R, the T/A-30's LERX) fly a slender-strake vortex-lift curve (Polhamus: CN = 1.57 sin a cos a + pi sin^2 a, vortex breakdown 38-60 deg, then a flat plate) instead of the wing's foil. Low lift slope at low AoA (1.6 /rad against the foil's 5.0), lift building to CL ~1.55 at 38-40 deg: like an F/A-18 LEX it trims the jet at high AoA without making it less stable at low AoA. Area from LerxEffectiveness. false = the stock foil. Respawn after changing.");
			cStrakeBd0 = dev.Bind("5. Airframe", "StrakeBreakdownStart", 30f, "Compass 0.1.2: 30 (0.1.1: 38). AoA (deg) where the strake vortex starts to break down (StrakeVortexLift): from here to StrakeBreakdownEnd the vortex lift fades to a flat plate of StrakeResidualCN. With 30 / 50 the strake lift peaks at CL ~1.32 near 32 deg and falls past the 35 deg limit (1.09 at 40), so the strakes trim the jet up to the limit without pitching it up past it (38 / 60 peaked at 40: bigger strakes overshot to 40-49 deg in loaded rolls). Respawn after changing.");
			cStrakeBd1 = dev.Bind("5. Airframe", "StrakeBreakdownEnd", 50f, "Compass 0.1.2: 50 (0.1.1: 60). AoA (deg) where the strake vortex has fully broken down. Respawn after changing.");
			cStrakeRes = dev.Bind("5. Airframe", "StrakeResidualCN", 1.2f, "Compass 0.1.2: normal-force coefficient of the broken-down strake (flat plate x sin a). Respawn after changing.");
			cSlab = dev.Bind("5. Airframe", "StabSlabFoil", false, "Ternion 0.1.0: off (the T/A-30's slab tail curve; the FS-3's elevators keep the game's foil). Compass 0.1.3: on by default. The tail slabs (elevator_L/R) and the fixed hstab_L/R fly a low-aspect-ratio swept-slab lift curve instead of the game's Trainer_stabilator foil (a copy of the straight wing's: 5.0 /rad, CL 1.0 at 17 deg). The wing foil is at its CLmax by 17 deg local AoA, so from ~15 deg body AoA the elevators have no nose-down left past neutral. Slab: stall at SlabStallAoA, peak CL (SlabStallAoA / 17.2)^SlabLiftSlope. Bench (300 km/h, with LerxEffectiveness 3.4): full nose-down at 35 deg AoA 44 -> 57 deg/s^2, full nose-up at -30 deg 42 -> 57; static margin at 5 deg +0.13 -> +0.05 m; the elevator command no longer chatters at the stop. false = the stock foil. Respawn after changing.");
			cSlabStall = dev.Bind("5. Airframe", "SlabStallAoA", 30f, "Compass 0.1.3: local AoA (deg) of the slab curve's lift peak (StabSlabFoil). Respawn after changing.");
			cSlabSlope = dev.Bind("5. Airframe", "SlabLiftSlope", 0.35f, "Compass 0.1.3: 0 = the slab keeps the foil's CL 1.0 peak and its lift slope drops with the later stall (5.0 x 17.2 / SlabStallAoA); 1 = it keeps the 5.0 /rad slope and the peak CL rises with the stall angle. 0.35 at 30 deg: 3.5 /rad, peak CL 1.22, a typical low-aspect-ratio swept tail. Respawn after changing.");
			cLerxEff = dev.Bind("5. Airframe", "LerxEffectiveness", 1.5f, "Su-30 0.1.12: 1.5 (Tom 9 Nov 16:51, with CgShiftAft 0.30 and AftBodyLift 0.5: bench, nozzle pitch trim in the 75 deg falling leaf 5.2 -> 3.4 deg). Su-30 0.1.3: 1.15 (Tom), the LERX = Cheek_L/R (area x this). Su-30 0.1.0: 1 (no LERX multiplier on the Su-30). Ternion lineage: Ternion 0.1.2: 1.13 = the shoulder LERX (shoulder_L/R) lift multiplier that keeps the static margin (aerodynamic centre) where 0.1.1 had it with ElevatorEffectiveness 1.2 and CanardEffectiveness 1.1 (bench, 300 / 500 km/h, -5..10 deg: -0.286 / -0.289 m vs -0.286 / -0.287; 1.0 would be -0.204 / -0.206, the tail moving it 8 cm aft). Its high-lift device's areas scale with it. Respawn after changing. Ternion 0.1.0: 1 (T/A-30 only). Compass 0.1.3: 3.4 (0.1.2: 3, 0.1.1: 2), with the slab tail (StabSlabFoil), whose lower lift slope gives less nose-up at full trailing-edge-up: 3.4 puts the elevator trim at 35 deg AoA back at 26-28 deg of the 30 (bench, 200-450 km/h). Area multiplier on the wing-root strakes (wingroot1_L/R, 0.45 m2 each, 2.2 m ahead of the CG) - the T/A-30's LERX (3 = 1.35 m2 each, ~7 % of the wing). With StrakeVortexLift their extra lift comes mostly at 20-35 deg (vortex lift), so it trims the jet up there without making it much less stable at low AoA. Bench, 30 / 50 breakdown: x3 with CgShiftAft 0.15 holds 35 deg in pulls and loaded rolls (peak 35.1-35.3); x4 overshot to 40.7 in loaded rolls and departed to -54 in a push. 1 = stock area. Respawn after changing.");
			cDwFade0 = dev.Bind("5. Airframe", "DownwashFadeStart", 45f, "Su-30 0.1.4: body AoA (deg) where the linear tail downwash (StabDownwash) starts to fade out; 45 = the F-22E's. Respawn after changing.");
			cDwFade1 = dev.Bind("5. Airframe", "DownwashFadeEnd", 60f, "Su-30 0.1.4: body AoA (deg) where the tail downwash has faded out; 60 = the F-22E's. Respawn after changing.");
			cStabHiScale = dev.Bind("5. Airframe", "StabHighAoAScale", 1f, "Su-30 0.1.4 (Tom: pitch-neutral at mid to high AoA). The stabs' lift and drag past 35 deg local AoA fade to this fraction by 60 (Raptor stab curve only). 1 = the curve as is. Respawn after changing.");
			cCgShift = dev.Bind("5. Airframe", "CgShiftAft", 0.10f, "Su-30 0.2.2: 0.10 (Tom 9 Nov 23:05; with CanardEffectiveness 1.6: bench static pitch within ~3.4 deg/s^2 of 0.25 / 1.3 at 0-75 deg AoA, nose-wheel share 12.2 %). Su-30 0.2.1: 0.15 (Tom 9 Nov 22:45: the jet tipped onto its tail in a slow taxi turn; CanardEffectiveness 1.5 keeps the in-flight balance: bench static pitch within ~2 deg/s^2 of 0.25 / 1.3 at 0-75 deg AoA, nose-wheel share 9.6 -> 11.3 %). Su-30 0.1.14: 0.25 (Tom 9 Nov 19:43: 0.30 tipped the jet onto its tail briefly at spawn; CanardEffectiveness 1.3 keeps the same in-flight balance). Su-30 0.1.12: 0.30 (Tom 9 Nov 16:51; bench ground roll at full AB, stick neutral: no self-rotation at low fuel up to 0.35, starts at 0.40). Su-30 0.1.10: 0.25 (Tom 9 Nov 11:18: \"with cg aft 0.25 it is much better\"; the pitch wall at 50-65 deg AoA, where the game logs put the airframe's nose-down above all the pitch authority from ~350-400 km/h). Su-30 0.1.2: 0, now works on the Su-30 (mass from Front_cover to the L/R-engine cans; bench static margin at low AoA 0.41 m stock -> 0.21 / 0.11 / 0.01 at 0.2 / 0.3 / 0.4, and at 70 deg / 190 km/h the airframe's nose-down with the best surface setting -17 -> -9 / -5 / -1 deg/s^2, i.e. nozzle trim ~6 -> ~1.5 deg at 0.3; it did not speed up the high-AoA roll on the bench). Respawn after changing. Ternion 0.1.0: 0 (T/A-30 only; the FS-3's balance is stock). Compass 0.1.2: 0.15 (0.1.1: 0.20, 0.1.0: 0.27). Moves the empty jet's centre of mass aft by this many metres (mass moved from cockpit_F to tail, both rigidbodies updated) for relaxed static stability: static margin at low AoA 0.45 m stock, 0.28 with the downwash, 0.13 here with the 3x vortex strakes. The strakes take the high-AoA trim, so the CG shift can stay small and the airframe still soaks up a low-speed roll's inertia coupling (p x r pitch-up) without pinning the elevators. 0 = stock. Respawn after changing.");
			cCapMeas = dev.Bind("2. Envelope", "CaptureMeasured", true, "Su-30 0.1.15 (Tom 9 Nov 20:19): the approach to the AoA limit plans its stop on the pitch braking the jet measures it has (aDn: the filtered measured pitch acceleration plus the effectors' remaining swing), corrected for how the airframe's own moment changes up to the limit, with each effector's swing time at its own slew as the delay. No margin factor and no fixed near-limit gain: the last degrees close at the rate the effectors can follow. false = the 0.1.14 capture (modelled braking blended toward aDn, x0.7, 3/s gain cap, roll-coupling cut).");
			cCgRef = dev.Bind("5. Airframe", "Su30CgReference", true, "Su-30 0.2.0 (Tom 9 Nov 22:14: the Su-30 1.0.1 airframe mod moved the CG; the FCS should not be affected). On the 1.0.1 airframe (told by its 500 kg nose cone) the eight parts it re-massed (600 kg moved forward: about 0.19 m of CG and more pitch / yaw inertia) get their 1.0.0 masses back, so the FCS flies the 1.0.0 mass layout and CgShiftAft works from it as before. 1.0.0: no effect. false = the installed airframe's own masses. Takes effect on respawn.");
			cEstThreaded = dev.Bind("0. Debug", "EstimatorThreaded", true, "Su-30 0.2.0 (from F-22E 3.0.4): the capacity estimator's allocator probes run on a worker thread instead of the game's main thread. The flying is identical either way: the control laws read the previous tick's estimate in both cases, and the worker solves on its own copies of the effectors. false = single thread. Takes effect at once.");
			cTickTimer = dev.Bind("0. Debug", "TickTimerLog", true, "Su-30 0.2.0 (from F-22E 3.0.4 / 3.1.1 / 3.1.3): every 10 s, LogOutput.log gets the FCS's main-thread time per physics tick (mean / median / p95 / max in microseconds, player and AI Su-30s separately), its split by phase, how often the main thread waited for the estimator's worker, and the airframe force hooks' time. Drives nothing.");
			cParTables = dev.Bind("0. Debug", "ParallelTables", true, "Su-30 0.2.0 (from F-22E 3.1.0): the surfaces' moment tables (rebuilt every physics tick) are computed on several cores: the game's objects are read on the main thread, the table maths runs on the thread pool and this thread together. Identical tables either way. false = one thread. Takes effect at once.");
			cParAlloc = dev.Bind("0. Debug", "ParallelAllocator", true, "Su-30 0.2.0 (from F-22E 3.1.0): when the allocator's two-horizon pick is active, its reach-plan solve runs on a second core while the main thread does the destination solve. Same pick, same commands either way. false = one thread. Takes effect at once.");
			cParSweep = dev.Bind("0. Debug", "ParallelSweep", true, "Su-30 0.2.0 (from F-22E 3.1.2): the asymmetric-load sweep reads the parts' state once per tick and computes its AoA points on several cores. Identical results either way. false = one thread. Takes effect at once.");
			cSlowTick = dev.Bind("0. Debug", "SlowTickLogUs", 1500f, "0.4.0 (diagnostic, from the F-22E 4.1.3): log any of the player's FCS ticks slower than this (us) on its own line, with the tick's 13 segments, whether a garbage collection ran during it and the managed heap's growth over it (at most 10 lines per 10 s). 0 = off. Drives nothing. Takes effect at once.");
			cNativeAlloc = dev.Bind("0. Debug", "NativeAllocator", 2, new ConfigDescription("0.4.0: the FCS's native core, Su30_FcsCore.dll, carried inside this plugin's DLL (0.4.1) (Rust, ported from the F-22E FCS 4.1.0): the control allocator's solves (the main ones each tick and the capacity estimator's probes), and with NativeSweep / NativeTables the asymmetric-load sweep's inner loop and the surface tables' maths. 2 = use it in the game: several times faster, the same laws, the bench's float arithmetic instead of Mono's (Unity runs Mono with floats carried in double, so the C# code differs from any compiled code in the last digits there, and the allocator's picks turn some of those into a different, mostly equal-cost answer for a tick). 1 = use it only where it returns exactly what the C# code returns, bit for bit (the bench; in the game that means C#). 0 = never: the C# code, as 0.3.10. With the DLL missing or failing its checks the C# code runs. Takes effect at once; the tick timer's log line reports what is in use.", new AcceptableValueList<int>(0, 1, 2)));
			cNativeAllocCheck = dev.Bind("0. Debug", "NativeAllocatorCheck", 0, new ConfigDescription("0.4.0: when the native allocator is in use, how it is compared with the C# one while flying. 0 = not at all: every solve is the native one. 1 = each allocator's first 64 solves and every 256th after them are solved both ways, and the C# result is the one used on those. 2 = every solve both ways, the C# result used on all of them (a comparison flight: the jet flies C# and pays for both). The tick timer's log line reports the counts: identical, close (commands within 0.001 deg), an equal-cost alternative (costs within 0.1 %), different. Where float arithmetic is strict, anything but identical is a fault: one warning in BepInEx/LogOutput.log and the native solve is switched off until the game restarts. Takes effect at once.", new AcceptableValueList<int>(0, 1, 2)));
			cSweepGrid = dev.Bind("0. Debug", "SweepGridCache", defaultValue: true, "0.4.0 (F-22E 4.0.1): the asymmetric-load sweep evaluates every grid deflection of every surface in each AoA point's airflow. On: what depends on the deflection alone (the surface's rotation at that deflection, its span axis and moment arm, and the same for the section it cambers) is worked out once per sweep tick instead of once per point. Same maths, same order; byte-identical on the bench. Off = the 0.3.10 loop. Takes effect at once.");
			cNativeSweep = dev.Bind("0. Debug", "NativeSweep", defaultValue: true, "0.4.0 (F-22E 4.1.0): with the native core in use (NativeAllocator) and SweepGridCache on, the asymmetric-load sweep's capacity loop runs in Su30_FcsCore.dll: the rotations are still worked out here, the flow's part (one atan2, the airfoil table, the force and its moment, the min / max kept) there. Byte-identical on the bench. Off = the C# loop. Takes effect at once.");
			cNativeCpl = dev.Bind("0. Debug", "NativeCoupledCap", defaultValue: true, "0.5.6 (from the Ternion 0.3.0): with the native core in use (NativeAllocator), the asymmetric-load check's coupled solves (the LP that finds the roll left each way with pitch held) run in the native core. Byte-identical on the bench. Off = the C# solver. Takes effect at once.");
			cNativeTables = dev.Bind("0. Debug", "NativeTables", defaultValue: true, "0.4.0 (F-22E 4.1.0): with the native core in use (NativeAllocator), the surfaces' moment tables (rebuilt every physics tick) get their per-deflection maths from Su30_FcsCore.dll: rotations here, the flow's part there; the live point and the table's post-processing stay here. Byte-identical on the bench. Off = the C# loop. Takes effect at once.");
			cParEnv = dev.Bind("0. Debug", "ParallelEnvelope", true, "Su-30 0.2.0 (from F-22E 3.1.3): the lift envelope (25 AoA points over every airframe part) and the hybrid pitch slopes, refreshed every 0.1 s, read the parts' state once and compute their points on several cores. Identical results either way. false = one thread. Takes effect at once.");
			cGCapMeas = dev.Bind("2. Envelope", "GCaptureMeasured", true, "Su-30 0.2.0 (Tom 9 Nov 21:28: rolling pulls from above ~300 kt hitch in roll or pitch): the approach to the load limit plans its stop on the same measured pitch braking and effector swing time as CaptureMeasured, from the AoA left to the limit on the lift slope. The old capture took an AoA-rate lead off that AoA and closed the rest at 3 /s; in full aft + full roll from 580-645 km/h the pitch rate's own overshoot drove the lead, and the pitch-rate command went 50 -> 20 -> 55 deg/s inside 0.6 s. false = the old capture. Takes effect at once.");
			cSoftLim = dev.Bind("2. Envelope", "SoftAoALimit", true, "Su-30 0.2.0 (Tom 9 Nov 21:28: \"+70/-65 is a hard cap, but when a corkscrew forces the jet out of the AoA limit, no protection, treat it as MPO\"). The AoA limits hold as before in normal flying. When the AoA is forced past a limit by more than SoftAoALimitMargin (a corkscrew, a tumble; the capture's own overshoot is under 1.5 deg), the stability-assist law hands the jet to the MPO law (pitch-rate command, no AoA limit, released stick = hold the attitude rate at zero; MPO's lateral law) until the AoA is back inside the limit for 0.25 s. The flight-assist toggle is not touched. false = the protection pitches the nose back to the limit (48 deg/s). Takes effect at once.");
			cSoftLimMargin = dev.Bind("2. Envelope", "SoftAoALimitMargin", 3f, "Su-30 0.2.0: how far past AoALimitPositive / Negative (deg) the AoA has to be forced before SoftAoALimit hands over to the MPO law. The logs' normal limit overshoot is 0.5-1.5 deg; the forced ones 7-14. Takes effect at once.");
			cStopHard = dev.Bind("3. Lateral-directional", "StopIgnoresSlipOutsideMargin", true, "Su-30 0.2.0 (Tom 9 Nov 21:28: \"if stopping requires sideslip outside of the margin, just don't care about sideslip, stop the thing as fast as possible\"). With the roll / yaw stick released, when the sideslip is already past RollStopSideslip or the stop the jet is starting would take it there (the yaw rate still to shed, at the yaw's measured capacity: L^2 cos AoA / 2a), the roll is no longer held for the yaw to catch up and the sideslip hold stands aside: both rates brake at full. It ends when the rotation has stopped or the stick moves. false = the paced stop (sideslip first). Takes effect at once.");
			cLsSlipTvcStart = dev.Bind("3. Lateral-directional", "LowSpeedSlipTvcStart", 8f, "Su-30 0.3.0 (LowSpeedSlipTvc): sideslip (deg) past which the nozzles start taking it out at very low speed. 0-45. Takes effect at once.");
			cLsSlipTvcGain = dev.Bind("3. Lateral-directional", "LowSpeedSlipTvcGain", 2f, "Su-30 0.3.2: 2 (was 1; a file holding 1 moves once). Su-30 0.3.0 (LowSpeedSlipTvc): stability-axis yaw rate asked (deg/s) per deg of sideslip past LowSpeedSlipTvcStart. 0-5. Takes effect at once.");
			cLsSlipTvcMax = dev.Bind("3. Lateral-directional", "LowSpeedSlipTvcMax", 40f, "Su-30 0.3.0 (LowSpeedSlipTvc): the most stability-axis yaw rate (deg/s) the correction asks. 0-120. Takes effect at once.");
			cMpoAxRel = dev.Bind("3. Lateral-directional", "MpoAxisReleaseStop", true, "Su-30 0.3.0 (Tom 10 Nov 16:46: in MPO with the nozzle-axis roll, after a release the FCS often would not stop the rotation, which was mostly yaw; stop it faster and care less about the roll it induces). true = in the nozzle-axis rotation (MPO past the AoA limit, or the low-speed band) with the lateral stick and pedals released, the yaw-rate command goes to zero at once (full braking) and the roll command follows the jet's own roll until the yaw is down to 10-30 deg/s, then brakes it too. false = 0.2.5 (the rotation about the nozzle axis winds down at the roll trajectory's braking rate). Takes effect at once.");
			cPushMinRate = dev.Bind("4. Gains", "PushOverMinRate", 23f, "Su-30 0.2.4 (F-22E 3.1.4/3.1.5; Tom 10 Nov 14:55: \"set the pitch rate floor of 23 like raptor, and forget sideslip in those cases\"): a planned push-over (a push from above 20 deg AoA with 15+ deg/s of yaw on board) is held to max(plan rate, this) deg/s, with no sideslip guard. 0 = the plan alone (0.2.3). Takes effect at once.");
			cTrajMeas = dev.Bind("4. Gains", "PitchTrajMeasuredEnd", true, "Su-30 0.2.0 (Tom 9 Nov 21:28: push-overs not as smooth as the Raptor's and the Ifrit's). The pitch-rate trajectory eases into its target over the pitch effectors' measured swing time (the moment-weighted half-swing at each one's own slew, plus the rate loop's lag) instead of a fixed 0.15 s, so the jet's own lag no longer overshoots the commanded rate. 9 Nov 21:04 log, full pushes from 63-70 deg: 48 deg/s asked, 57-63 flown (Ifrit 52, F-22E 48-60). false = the fixed 0.15 s. Takes effect at once.");
			cAftBodyLift = dev.Bind("5. Airframe", "AftBodyLift", 0.5f, "Su-30 0.1.12: 0.5 (Tom 9 Nov 16:51). Lift-area multiplier on the aft body parts behind the CG: the intake / engine covers (3 m2 each, ~1.5 m aft), the engine bodies (1 m2 each, ~5.7 m aft) and the tail cone (0.5 m2). Bench at 350 km/h, stabs and nozzles at 0: nose-down at 75 deg AoA -57 -> -47 deg/s^2 (with CgShiftAft 0.30), stability at 5-10 deg -1.75 -> -1.6 deg/s^2 per deg, normal force at 75 deg -5 %. 1 = stock. Respawn after changing.");
			cLatMargin = cfg.Bind("1. General", "HighAoALateralBrakeMargin", 0.86f, new ConfigDescription("High-AoA lateral braking margin. Above HighAoALateralMarginFull (30 deg AoA, dev cfg) the roll- and yaw-rate limits are multiplied by this, blended in smoothly from HighAoALateralMarginStart (25). 0.86 = 14 % less allowed stop time: 14 % less roll and yaw rate for the same authority, the rotation stops sooner. 1 = 0.3.0. Takes effect at once.", new AcceptableValueRange<float>(0.5f, 1.5f)));
			cLsSlipTvc = cfg.Bind("1. General", "LowSpeedSlipTvc", defaultValue: false, "Su-30 0.3.5: off by default (Tom 10 Nov 22:44: rough, prefers it off). Su-30 0.3.2 (Tom 10 Nov: let the TVC take out some of the sideslip low-speed corkscrews build, and stop trying when it lacks the authority). true = when the airframe is too slow to hold sideslip itself (measured directional stiffness under 0.8, full at 0.4) and the sideslip passes LowSpeedSlipTvcStart (dev, 8 deg), the FCS asks for stability-axis yaw toward zero sideslip (LowSpeedSlipTvcGain, dev, 2 deg/s per deg past the start) without adding velocity-vector roll, and the allocator chases it only with effector combinations that do not swing the velocity-vector roll (that pumps the AoA at large sideslip). If 0.15 s after it comes in the stability-axis yaw is not turning toward less sideslip, the FCS judges it lacks the authority, fades the correction out and leaves it out until the sideslip is back under the start. AoA law, forward flow (fades out at 75-85 deg AoA). Bench (16:16 log, 8 low-speed sideslip events x 6 replay offsets): peak sideslip 2.4 deg lower on average, no event worse on average, 3 of 48 runs more than 3 deg worse. false = off (0.2.5 behaviour)");
			cLowSpeedTvc = cfg.Bind("1. General", "LowSpeedTvcRoll", defaultValue: false, "Su-30 0.1.16: now gated by the airframe's sideslip stiffness (LowSpeedTvcRollStiffStart / Full), not IAS, and for Stability Assist on only: MPO always uses the stiffness gate. Su-30 0.1.11 (Tom 9 Nov): true = when the jet is too slow for the airframe to hold sideslip, the FCS stops trying. The sideslip hold and the sideslip roll governor fade out and the roll stick turns the jet about the axis the canted nozzles turn it about (roll and yaw together), between LowSpeedTvcRollStartKmh and LowSpeedTvcRollFullKmh. With Stability Assist on the AoA limit stays in force, even at full aft stick. In MPO there is no AoA limit. false = velocity-vector roll at every speed (0.1.10).");
			cLowSpeedStart = cfg.Bind("1. General", "LowSpeedTvcRollStartKmh", 150f, "Su-30 0.1.11: IAS (km/h) where LowSpeedTvcRoll starts to blend in. The airframe's sideslip stiffness above 50 deg AoA, 11:08 log: about 2.9 deg/s^2 per deg at 300-400 km/h, 0.6 at 150, 0.2 at 110.");
			cLowSpeedFull = cfg.Bind("1. General", "LowSpeedTvcRollFullKmh", 120f, "Su-30 0.1.11: IAS (km/h) at and below which LowSpeedTvcRoll is fully in. Keep it below LowSpeedTvcRollStartKmh.");
			cLowSpeedKStart = cfg.Bind("1. General", "LowSpeedTvcRollStiffStart", 0.4f, "Su-30 0.1.16 (Tom: TVC roll only when the sideslip stiffness is like a brick): the airframe's directional stiffness (deg/s^2 of restoring yaw per deg of sideslip, the FCS's slipStiff) below which the nozzle-axis roll starts to blend in (Stability Assist on: only with LowSpeedTvcRoll; MPO: always, unless the dev MpoTvcAxisRoll is off). Filtered over ~1 s. Only past 25-35 deg of total flow angle off the nose (AoA and sideslip together). For scale, 60-80 deg AoA in your logs: 0.43 at 120-150 km/h, 0.30 at 90-120, 0.69 at 150-200, 1.1 at 200-260 (the falling leaf).");
			cLowSpeedKFull = cfg.Bind("1. General", "LowSpeedTvcRollStiffFull", 0.2f, "Su-30 0.1.16: stiffness at and below which LowSpeedTvcRoll is fully in. Keep it below LowSpeedTvcRollStiffStart. (LowSpeedTvcRollStartKmh / FullKmh are used only if the dev RollStopSlipProbe is off.)");
			cMpoSep = cfg.Bind("1. General", "MpoRollMode", MpoRollMode.VelocityVectorRoll, "VelocityVectorRoll by default. The lateral law with Stability Assist off (MPO). VelocityVectorRoll = as with it on: the stick rolls about the velocity vector, the sideslip is held and the pedals work as they do with it on. SeparateRollYaw = the stick rolls about the velocity vector at every AoA (sideslip held while the stick is in and the pedals centred) and the pedals command pure body yaw rate at every AoA.");
			cMpoWash = dev.Bind("3. Lateral-directional", "MpoSideslipWashout", 0.5f, "Revoker 0.2.7: pedals centred only (the pedals themselves are pure body yaw, with no sideslip limit). Ifrit 0.2.0, separate axes: with the pedals centred and below the coupling AoA, the yaw axis lets sideslip wash out at this rate (deg/s of yaw rate per deg of sideslip), like the airframe's own weathervane. Sideslip is kept inside +-PedalSideslip there. 0 = the yaw rate is held exactly (sideslip stays where the roll left it).");
			cIfritRev = dev.Bind("2. Envelope", "IfritConfigRevision", 8, "Internal: config revision (inherited from the Ifrit FCS). Leave it alone.");
			cIfritSplit = dev.Bind("3. Lateral-directional", "SplitFlapYaw", defaultValue: false, "Ternion 0.1.3: false. The FS-3's split surfaces have no split yaw (yawSplitFactor 0 on all of them), so as effectors they promised a yaw the game never produced; with this off, the FCS leaves every split surface at its stock limit and closed while it flies. Ifrit: the split-flap decelerons are allocator effectors (drag yaw; the game splits a flap from its job's yaw input). The stock zero-throttle airbrake split is suppressed while the FCS flies. false = the flaps never split while the FCS is engaged. Applied when the jet spawns.");
			cIfritTvcRoll = dev.Bind("3. Lateral-directional", "RollTvc", defaultValue: true, "Su-30 0.1.0: on = the canted differential (CantedTvc): one nozzle up, the other down gives roll and yaw together (proverse, same direction). Ternion lineage: Ifrit: differential (roll) nozzle deflection about the pitch angle, the Ifrit's stock roll vectoring (+-15 deg), as an allocator effector. false = pitch-only TVC like the F-22E law. Applied when the jet spawns.");
			cSplitMu = dev.Bind("4. Gains", "SplitFlapCost", 1E-03f, "Ifrit: allocator cost of split-flap use (drag). Bench: roll/yaw results unchanged from 1e-4 to 5e-3.");
			cSlipTvc = dev.Bind("3. Lateral-directional", "SlipTvcAssist", false, "Su-30 0.1.10: false (Tom's setting). Ifrit 0.3.0: the sideslip correction may use body roll up to what the wing surfaces plus the differential TVC can brake, instead of being held to the aero yaw-rate limit. At high AoA the correction is mostly body roll (x sin AoA), and the nozzles roll the jet at any IAS. false = 0.2.2.");
			cIfritFlaperons = dev.Bind("5. Airframe", "IfritFlaperons", false, "Ternion 0.1.0: off (Revoker-style trailing edges: no flap schedule droop; the canards and elevators carry a held turn's pitch). Ifrit 0.3.0: the decelerons (inner) and ailerons (outer) work as flaperons on the F-22E flap schedule: up to 23 deg (inner) and OuterFlaperonDroop x that (outer) TE-down with AoA, load and speed (the high-lift schedule), held off the stall on each surface's own flow, out of the way of full roll stick, faded out from 20 to 30 deg AoA. false = 0.2.2 (no droop).");
			cTvcRollNeutral = dev.Bind("4. Gains", "TvcRollNeutralHold", false, "Ifrit 0.3.0, default OFF: on the bench any weight here makes the released-stick pitch hold sluggish (q -0.3 deg/s bleeding off over ~7 s in level flight at 400 km/h, the stabs held still), an interaction with the roll-fight cost. TvcNeutralHold for the differential (roll) nozzles. With the lateral stick and pedals centred, the roll on its command and the sideslip under 2 deg, the nozzles go back to symmetric and the wing surfaces hold the roll trim; any roll input, roll error or sideslip frees them at once. false = 0.2.2.");
			cTvcRollNeutralW = dev.Bind("4. Gains", "TvcRollNeutralWeight", 30f, "Ifrit 0.3.0: cost weight of TvcRollNeutralHold.");
			cSlipGainHi = dev.Bind("3. Lateral-directional", "SlipGainHighAoA", 0f, "Su-30 0.1.10: 0 (Tom's setting). Ifrit 0.3.0 (SlipTvcAssist): the sideslip gain (KBeta) ramps to this from 25 to 45 deg AoA, where the correction is mostly body roll from the nozzles. Bench, 65 deg corkscrew, 1 s full-pedal kick: back under 3 deg in 4.3-4.7 s at 1.3, 0.5-0.8 s at 4. Below 25 deg KBeta applies as before.");
			cSlipGainLoops = dev.Bind("3. Lateral-directional", "SlipGainFromLoops", true, "Revoker 0.2.14: the sideslip-hold gain is what the rate loops carrying the correction can follow, instead of the fixed SlipGainHighAoA ramp. The correction rS is flown as body yaw rS cos(AoA) through the yaw-rate loop (YawRateGain, 2.5/s = 0.4 s) and as body roll -rS sin(AoA) through the roll-rate loop (RollRateGain, 6/s = 0.17 s), each plus the actuator lag (ActuatorLagT). The sideslip loop closes through them as gain/s x lag, so its phase margin is gain x lag: the gain is SideslipGain x (yaw-loop lag / the cos^2-sin^2 weighted lag of the two paths), and a path that would have to deliver its share beyond the spare authority of its axis counts the time that takes instead. 0 deg AoA: exactly SideslipGain (1.3); -50: ~1.85; 70: ~2.3. The 30 Oct 22:48 log flew the Revoker's push + roll at the fixed 4 through the 0.4 s yaw loop: ~15 deg of phase margin, sideslip -3.4 <-> +3.6 at 1 Hz, yaw rate +-12 deg/s about its command, rudder stop to stop, the roll governor cutting twice (the hitch); the F-22E, at 1.3 there, recovers -6.8 deg in one move. false = the SlipGainHighAoA ramp (0.2.13).");
			cActLag = dev.Bind("3. Lateral-directional", "ActuatorLagT", 0.07f, "Revoker 0.2.14: command-to-surface lag (s) the SlipGainFromLoops gain counts on top of each rate loop's 1/gain (30 Oct 22:48 log: rudder and canard command lead their position by 0.07 s, the yaw nozzle by 0.02).");
			cSlipGovLead = dev.Bind("3. Lateral-directional", "SlipGovLeadFromLoops", true, "Revoker 0.2.15: the push + roll sideslip governor looks ahead as far as the lateral loops take to answer (the SlipGainFromLoops lag, 0.33-0.47 s) instead of a fixed 0.8 s. 30 Oct 23:45 log (0.2.14), push + roll at -42 deg AoA: the sideslip crossing zero at +11 deg/s on its way back from -3.3 deg (the hold's own recovery, peak +2.4) read as +9 deg 0.8 s out and cut the roll target 79 -> 46 deg/s; the roll braked, the yaw command fell with the yaw nozzle and rudder on their stops, the yaw rate overran by 6 deg/s and the nozzle went stop to stop, then the release 0.35 s later re-built the roll 46 -> 85 and it went round again. Over the loops' own lag that crossing projects to 3.6 deg: no cut. false = 0.8 s (0.2.14). Needs SlipGainFromLoops.");
			cSlipLimTas0 = dev.Bind("3. Lateral-directional", "RollSlackMinTAS", 25f, "Ifrit 0.3.9: below this true airspeed (m/s) the roll is not held back for sideslip or yaw coordination (sideslip guard / governor, yaw-anchored roll limit, roll-acceleration coordination cap); they come back in over RollSlackTASSpan (smoothstep) and are off past 120 deg |AoA|. Higher = more roll slack at low speed, more sideslip.");
			cSlipLimTasSpan = dev.Bind("3. Lateral-directional", "RollSlackTASSpan", 50f, "Ifrit 0.3.9: m/s over which the roll limiters come back in above RollSlackMinTAS (default: full from 75 m/s TAS, 270 km/h). Bench, full stick from near zero airspeed accelerating to 250 km/h: 25/50 holds 21-44 deg/s where 20/30 sat at 11 deg/s; corkscrew from 150 km/h and slow rolls unchanged.");
			cSlipHoldTas0 = dev.Bind("3. Lateral-directional", "SlipHoldMinTAS", 15f, "Ifrit 0.3.7: the sideslip hold is off below this true airspeed (m/s) and fades in over SlipHoldTASSpan; it also fades out from 100 to 120 deg |AoA| (flying backwards). Pedals keep commanding yaw there. In tail slides and near zero airspeed the hold asked for yaw it had no authority for: the jet swayed left/right and the roll stick was scaled to nothing. 0 = 0.3.6 at speed (reversed-flow fade still applies).");
			cSlipHoldTasSpan = dev.Bind("3. Lateral-directional", "SlipHoldTASSpan", 20f, "Ifrit 0.3.7: m/s over which the sideslip hold fades in above SlipHoldMinTAS (default: full from 35 m/s TAS).");
			cDecHldFold = dev.Bind("5. Airframe", "DeceleronHldFold", true, "Ifrit 0.3.6: the stock high-lift device also rotates each deceleron's inner hinge 20 deg TE-down (the hinge carrying the deceleron's lift and both split halves), on top of the FCS deflection and the split: a split half reached 50 deg on the runway with the airbrake out (80 worst case). With this on, that hinge stays put and the decelerons fly the same 20 deg x deploy as droop instead, so everything stays inside the +-30 deg deceleron travel and the split budget. The airbrake has priority over that droop (it retracts while the splits are out). false = 0.3.5.");
			cSlipPredT = dev.Bind("3. Lateral-directional", "SlipPredictTime", 0.3f, "Ifrit 0.3.5: sideslip predictor window (s), the inner roll/yaw rate loops' lag in game (~0.3 s in the 20 Oct logs). The sideslip correction works on the sideslip minus the yaw (stability-axis) rate it has already commanded over this window, which has not reached the sideslip yet. Fixes the 0.7 Hz sideslip / roll oscillation at the high-AoA sideslip gain (20 Oct 22:37 and 23:10 logs). 0 = off (0.3.2). Replaces 0.3.3's SlipRateLead.");
			cSlipPredRollGate = dev.Bind("3. Lateral-directional", "SlipPredictRollGate", 0f, "Ifrit 0.4.0: roll-stick fraction over which the sideslip predictor fades OUT (0.3.5-0.3.9: 0.2, i.e. the predictor was off whenever roll stick was in). 0 = the predictor runs with roll stick in too. 22 Oct 00:30 log, 65 deg AoA corkscrew at 55 m/s with full roll stick held: without the predictor the high-AoA sideslip loop (gain 4) rang against the ~0.34 s inner-loop lag, sideslip +-12 deg and body roll +-50 deg/s on a 1.6 s cycle, every roll surface and the nozzles bang-bang on their stops. A delay-loop model with that lag and 60-100 deg/s^2 of roll authority reproduces the +-10-12 deg cycle without the predictor and settles 12 -> -1.5 -> 0.5 deg in 1.6 s with it.");
			cSplitYawTier = dev.Bind("3. Lateral-directional", "SplitYawTier", true, "Ifrit 0.4.0: least drag first with an engine out or a damaged yaw axis. The canted tails carry the yaw as long as they can; the inner/outer flaperon crossing comes next (the allocator finds it), the split decelerons open only for the yaw still out of reach (a gate on measured yaw saturation that leaks shut once the yaw is met, the split then having priority over its deceleron's deflection for their shared travel), the VMC throttle limit last. Bench, yaw per kN of added drag at 250 km/h and 20 deg AoA: tails +-10 deg 4.8 deg/s^2, crossing 1.1-1.5, one split at 30 deg 0.4 (17.7 kN of drag), the throttle cut 0.17, false = 0.3.9 (tails and splits one tier, the split free: with the right engine alone at 140 m/s the left split sat at 15-24 deg while the tails held +-2 deg; at the 30 deg cap the flaperons crossed with the splits shut and the engine cut to 30-40 %). Both engines running: no change.");
			cStabYawRelax = dev.Bind("3. Lateral-directional", "StabYawRelax", 1f, "Ifrit 0.4.0 (SplitYawTier): with an engine out or a damaged yaw axis, how far (0-1) the roll-fight cost is relaxed for the canted tails, so their differential may buy yaw with the wing taking the roll back. 0 = the 1.8.41 fight cost in full (the allocator then prefers sideslip or the splits to a tail difference).");
			cSlipPredW = dev.Bind("3. Lateral-directional", "SlipPredictWashout", 1.5f, "Ifrit 0.3.5: washout (s) on the predictor's command history, so a held correction (e.g. a yaw trim against a load) leaves no extra steady sideslip. 0 = no washout.");
			cRevS0 = dev.Bind("3. Lateral-directional", "ReversedFlowSenseStart", 120f, "Ifrit 0.3.4 (ReversedFlowRollSense): |AoA| (deg) where the roll stick sense starts to turn over for reversed flow. It blends smoothly to fully flipped at ReversedFlowSenseEnd, passing through zero. 0.3.2 flipped at once at 100 deg (released at 80), next to 90 deg, where the velocity-vector roll is nearly all body yaw, so the flip reversed the rotation mid-manoeuvre.");
			cRevS1 = dev.Bind("3. Lateral-directional", "ReversedFlowSenseEnd", 150f, "Ifrit 0.3.4: |AoA| (deg) where the roll stick sense is fully flipped for reversed flow (see ReversedFlowSenseStart).");
			cSlipGainA0 = dev.Bind("3. Lateral-directional", "SlipGainAoAStart", 5f, "Ifrit 0.3.1: AoA (deg) where the SlipGainHighAoA ramp starts (0.3.0: 25); it reaches SlipGainHighAoA 20 deg higher. Pedals centred only. Bench, corkscrew entry at 475 km/h: from 25 the sideslip built at 15-20 deg AoA and the rolling-pull governor held the AoA there (the 20 deg hitch).");
			cFullStickQ = dev.Bind("2. Envelope", "FullStickPitchRate", 60f, "Ifrit 0.3.1: pitch-rate limit (deg/s) at full aft stick; the limit rises from MaxPitchRate (48) over the last 20 % of aft stick, so up to 0.8 stick and the push side map as before. MPO unchanged. 0 or <= MaxPitchRate = off.");
			cSplitBrakeHold = dev.Bind("7. Airbrake", "SplitBrakeHoldCost", 0.5f, "Ifrit 0.3.0: 0.5 (0.2.2: 0.05). With the airbrake out, the allocator cost of bringing both splits in together (the brake symmetric part). With the 30 deg split and 100 deg/s servos, 0.05 let the allocator trade split for pitch and stall there (bench, 700 km/h at 3 km: splits held at 27, pitch rate drifting to -0.6 deg/s, no retraction on release). Higher keeps the full brake through pulls; one-sided retraction for yaw is free, as on the stock Ifrit.");
			cTvcDiffMu = dev.Bind("4. Gains", "RollTvcCost", 0.003f, "Ifrit 0.3.1: 0.003 (was 5e-5). Allocator cost of differential (roll) nozzle deflection. At 5e-5 the roll TVC was nearly free, so on a pitch release the allocator could run the decelerons/ailerons unevenly and cancel their roll with the nozzles (one nozzle back to neutral before the other, 20 Oct 14:03 log; bench 330-420 km/h: up to 12 deg nozzle split, pitch-rate overshoot -3 deg/s). At 0.003: under 1 deg, overshoot -0.2; corkscrew and rolling-pull sideslip within 0.1-0.3 deg of before.");
			cYawTvc = dev.Bind("3. Lateral-directional", "YawTvc", false, "Su-30 0.1.0: off. The real nozzles have one axis each (CantedTvc); yaw comes only from the canted differential, so there is no separate yaw nozzle effector. Ternion lineage: Revoker 0.1.0: the nozzle's stock yaw axis (+-15 deg, controlInputs.yaw x thrustVectoring.y) is an allocator effector, in the first yaw tier with the rudder. The game zeroes it under 3 m radar altitude and above 250 m/s (with pitch). false = pitch-only TVC (F-22E principle).");
			cTvcYawMu = dev.Bind("4. Gains", "YawTvcCost", 0.0005f, "Revoker 0.1.0: allocator cost of yaw nozzle deflection.");
			cFlapFlaperons = dev.Bind("5. Airframe", "FlapFlaperons", false, "Ternion 0.1.0: off (the T/A-30 / FS-12 HLD flaps; the FS-3's flaperons are real control surfaces). Revoker 0.2.0: the inboard trailing-edge surfaces (flap_L/R; in the game moved only by the high-lift device, 25 deg TE-down below 90-150 m/s) are inboard elevons: allocator effectors for pitch and lateral (the inboard pair, roll group 3, so they can also cross the ailerons for drag yaw as split elevons), driven at SurfaceServoSpeed. The FCS drives the hinge and turns the wing2 section's lift normal by FlapCamberCoupling x the deflection. No flap schedule (FlapHighLiftDroop 0). false = HLD flaps as stock.");
			cFlapHinge = dev.Bind("5. Airframe", "FlapHingeRange", 20f, "Compass 0.1.0: 20 (the stock high-lift device's 20 deg). Travel each way of the flaps (flap_L/R) as allocator effectors (deg).");
			cFlapHiLift = dev.Bind("5. Airframe", "FlapHighLiftDroop", 20f, "Compass 0.1.0: 20. TE-down droop (deg) the flaps are drawn toward at full high-lift-device deployment (the game's low-speed flap schedule: out below 80 m/s, in above 120), so the T/A-30 keeps its stock landing flaps; the allocator adds roll / pitch on top. 0 = no droop.");
			cGyroBudget = dev.Bind("4. Gains", "GyroBudget", 0.25f, "Su-30 0.1.11 (Tom 9 Nov 11:28: inverted corkscrews stop, then tumble). Past 35-45 deg AoA the pitch-rate command is held to what keeps the gyroscopic roll (Iyy - Izz) / Ixx x q x r within this share of the roll authority (nozzles + surfaces). At low IAS the gyro roll outgrew the nozzles, and the canted nozzles' differential spent against it yawed the jet the wrong way until it tumbled. Fades in from 170 to 130 km/h IAS; never limits a nose-down or protection command. 0 = off (0.1.10).");
			cDragShape = dev.Bind("5. Airframe", "HighAoADragShape", true, "Su-30 0.1.5 (Tom: the jet loses airspeed comically fast as AoA rises; drag curve halfway between the Ifrit's and the Revoker's). The Su-30VXN's own foils lose lift fast past ~30 deg while their drag climbs to flat-plate levels: drag per kg in Tom's logs was 1.5-2x the Ifrit's and Revoker's from 30 deg up (60-70 deg: 79 against 48 and 54). On: the drag curves of the wing (wing panels, flaperons, outer flaps), body (fuselage, LERX) and canard foils are scaled against each part's own AoA, x1 up to 17.5 deg (same by AoA magnitude on the negative side): wing and body ~0.63 at 30, ~0.48 at 40-50, 0.64 at 70, 0.74 at 90; canards ~0.80 at 30, ~0.73 at 40-70, 0.81 at 90. Solved on the bench so the jet's total drag lands halfway between the Ifrit and Revoker logs and the static pitch moment at each AoA stays as it was (the canards' drag is nose-up, the wing's nose-down), with the stabs (already on the F-22E's stab curve) and the fins unchanged. Lift is not touched. Respawn after changing.");
			cPitchBump = dev.Bind("5. Airframe", "HighAoAPitchBump", 0.75f, "Su-30 0.2.5 (Tom 10 Nov 15:52: more pitch-up at 55-65 deg AoA, nothing else changed). Lift-curve change on the body sections: ahead of the CG (Front_cover, Cheek_L/R, Mid_cover) lift + K x w / cos(AoA), behind it (intake / engine covers, engines, tail cone) the same total lift less, so a pure nose-up couple with no net lift or drag change. w ramps in 53 -> 57.5 deg, holds to 65, ramps out to 0 at 70; positive AoA only. 0.75 = +29-31 deg/s^2 at 57.5-62.5 deg and 450 km/h (+13 at 300, +42-46 at 550, scaling with dynamic pressure); the best nose-up pitch acceleration then stays at +62-71 deg/s^2 from 55 to 65 deg at 450-600 km/h instead of dropping to +13-36, filling the drop in the best nose-up pitch acceleration past 55 deg where the stabs stall at +35. 0 = off (0.2.4). Respawn after changing.");
			cDragShapeDepth = dev.Bind("5. Airframe", "HighAoADragShapeDepth", 1f, "Su-30 0.1.5: how much of the HighAoADragShape cut is applied: 1 = the full cut (halfway between the Ifrit and Revoker), 0 = none (0.1.4), 0.5 = half of it. 0-1.5. Respawn after changing.");
			cThrustLineAtCg = dev.Bind("5. Airframe", "ThrustLineAtCg", true, "Su-30 0.1.5 (Tom: fix the CG above the thrust). The Su-30VXN's nozzles sit 0.49 m below its CG, so thrust pitched it nose-up on the runway (nearly unloading the nose wheel at full AB) and changed the pitch trim with throttle. On: each nozzle's force is applied at the CG's height (moved along the jet's vertical axis only; the vectoring arm and engine-out yaw are unchanged), as on the F-22E and Revoker, whose thrust lines are within ~0.1 m of their CG. The take-off nozzle feed-forward (NoseThrustFeedForward) then has nothing to cancel and stays at 0. Off = 0.1.4. Takes effect at once.");
			cLeafBoost = dev.Bind("3. Lateral-directional", "FallingLeafYawBoost", 1.2f, "Su-30 0.2.0: 1.2 (Tom 9 Nov 21:28: \"I like falling leaf boost at 1.2, so default there\"; a file holding 1 moves to 1.2 once). Su-30 0.1.4 (Tom: at least 65 deg/s in a falling leaf), default 1 = off. Past 55 deg AoA (fully by 65) the yaw-rate limit (YawStopFactor x LowIasYawBoost x yaw authority) and its braking floor are x this, from 115 km/h IAS (fully by 145) to 250 km/h (gone by 300). The rate it allows is the rate the canted nozzles can no longer stop: bench, full aft + full roll at the 75 deg limit (steady ~160 km/h) / roll released at 70 deg (~120-180 km/h), peak sideslip: 1 = 38-39 deg/s / 10 deg; 1.12 = 43 / 15-18; 1.25 = 49 / 21-25; 1.55 = 64 / 38 (a departure). Takes effect at once.");
			cSurfFirst = dev.Bind("4. Gains", "SurfacesFirstPitch", 30f, "Su-30 0.1.8 (Tom 9 Nov: stabs and canards do more of starting a pitch rate at low to mid AoA, so the TVC can do the lateral). The nozzles' common (pitch) deflection costs this much (as TvcNeutralHold) at every pitch rate and pitch error up to SurfacesFirstMaxAoA, so the stabs and canards take the pitch first and the nozzles only what they cannot; not in AoA protection. Bench (full-aft pulls and corkscrews from 450 / 600 km/h): the nozzles' mean common deflection at 5-45 deg AoA 1.7-6.2 -> -0.1-1.1 deg, time to 55 deg unchanged (100: a half-stick pull's pitch error 0.2 -> 1.0 deg/s). 0 = off (0.1.7). Takes effect at once.");
			cSurfFirstAoA = dev.Bind("4. Gains", "SurfacesFirstMaxAoA", 35f, "Su-30 0.1.8: AoA (deg) up to which SurfacesFirstPitch holds in full; it fades out over the next 10 deg (past it HighAoACanardPitchFirst applies with a roll asked). Bench: 45 slowed the pitch through 45-55 deg in high-speed corkscrews.");
			cSurfFirstIasLo = dev.Bind("4. Gains", "SurfacesFirstIasLow", 350f, "Su-30 0.1.8: IAS (km/h) below which SurfacesFirstPitch is off (the stabs and canards have less pitch authority there and the nozzles are needed); it fades in up to SurfacesFirstIasHigh. Bench: from 300 the full-aft corkscrews entered at ~300 km/h had more sideslip (6.9 -> 13 deg in one case).");
			cSurfFirstIasHi = dev.Bind("4. Gains", "SurfacesFirstIasHigh", 450f, "Su-30 0.1.8: IAS (km/h) from which SurfacesFirstPitch holds in full.");
			cGCapReach = dev.Bind("4. Gains", "GCaptureReachable", true, "Su-30 0.1.9 (Tom 9 Nov: pull-up hesitant). The load-limit capture brakes the pitch rate only as far as the load limit can actually be reached at this airspeed: where the airframe's maximum load (its lift peak) is under the G limit it fades out (from limit + 0.5 g to limit - 0.5 g), and the AoA limit takes over. Before, in pulls from low AoA at 340-600 km/h the pitch-rate command came back 8-24 deg/s for ~0.3 s and the pitch rate dipped 9-21 deg/s (9 Nov 09:59 log, 9 of 21 pulls). false = 0.1.8. Takes effect at once.");
			cGCapRoll = dev.Bind("4. Gains", "GCaptureReachableRolling", false, "Su-30 0.1.9. false: GCaptureReachable fades back out with roll or yaw stick in (0.1 to 0.4 of stick), because in a rolling pull the faster AoA rise outran the yaw that coordinates the roll (bench, full aft + full roll from 480 km/h: peak sideslip 11.4 -> 16.0 deg). true = it acts in rolling pulls too. Takes effect at once.");
			cWallPeak = dev.Bind("4. Gains", "PitchWallSurfacePeak", true, "Su-30 0.1.9 (0.1.8 CanardPitchPeakWindow: canards only, above 300 km/h). In the pitch wall (40 deg AoA to 2 short of the limit, above 150 km/h, pitch rate over 10 deg/s short of its target; held 0.5 s) the canards and stabs are put at their own most nose-up deflection in the flow they see (the stabs within 8 deg of it). With roll or yaw stick in, each stab is instead kept where its nose-up moment is near its best (WallLateralGoodFrac), so the pair can still split for roll but not through the worst-pitch middle (9 Nov 10:25 log: full aft + full roll at 65 deg AoA, 400 km/h, the AoA fell to 45). Past their stall the allocator's pitch envelope is flat and left the stabs at ~0 deg at 60-68 deg AoA (9 Nov 09:59 log), the worst place for them (bench, 450 km/h: +35 or -30 gives 33-50 deg/s^2 more nose-up), and the canards at -45 (local AoA 110-117; -15..-37 gives 14-43 deg/s^2 more). false = 0.1.7. Takes effect at once.");
			cWallLatBand = dev.Bind("4. Gains", "WallLateralGoodFrac", 0.45f, "Su-30 0.1.9. PitchWallSurfacePeak with roll or yaw stick in: each stab is kept where its own nose-up moment is within this fraction (of its whole range over the travel) of its best. Past the stall that is both ends of the travel, so the stabs can still split for roll, one at each end, but not through the middle where they push the nose down hardest (9 Nov 10:25 log, 197.0-198.1 s: full aft, full left roll at 65 deg AoA, 400 km/h; the right stab sat at +2..+11 and the AoA fell to 45). 0 = not held. Takes effect at once.");
			cWallLatCan = dev.Bind("4. Gains", "WallPinLateralCanard", true, "Su-30 0.1.9. PitchWallSurfacePeak with roll or yaw stick in: the canards are still put at their most nose-up deflection. Takes effect at once.");
			cStockCanSched = dev.Bind("5. Airframe", "StockCanardSchedule", false, "Su-30 0.1.8: true = leave the mod's CanardSurface AoA schedule running on the canards (the hinge turns leading edge down 1:1 with AoA up to 30 deg, under the FCS's own deflection), as it did by mistake in 0.1.0-0.1.7. false = switched off while the FCS flies (the design since 0.1.0; the bench flies it this way). Respawn after changing.");
			cCanPitchFirst = dev.Bind("4. Gains", "HighAoACanardPitchFirst", 0f, "Su-30 0.1.9: default 1000 -> 0 (off). It let go of the nozzles whenever the pitch rate was more than 1-5 deg/s off its target and took them back below that, so in the falling leaf it cycled at ~1.3 Hz: the nozzles' common deflection 1 <-> 15 deg, pitch rate 6 <-> 16 deg/s, the stabs, flaperons and rudders swinging 10-25 deg with it, and the AoA held at 71-72 instead of 75 (9 Nov 09:59 log, 206-222 and 263-283 s). Off on the bench: the same mean nozzle deflection in the leaf (6.1 vs 6.4 deg), its swings 5.2 -> 1.0 deg rms, the AoA held at 75. A 0.1.4-0.1.8 dev file still holding 1000 is moved once. Su-30 0.1.4 (Tom: canards take more of the pitch job from the TVC so it can do the velocity roll). Each canted nozzle has one axis, so pitch and roll share its +-15 deg. While a roll is asked past 25 deg AoA (fully by 45) the nozzles' common (pitch) deflection is drawn toward neutral and the canards and stabs carry the pitch first; a pitch-rate error over 1-5 deg/s lets go at once, so the nozzles still take what the surfaces cannot. Cost weight (as TvcNeutralHold); 0 = off (0.1.3). Takes effect at once.");
			cFlaperonRollPrio = dev.Bind("5. Airframe", "FlaperonRollPriority", 1f, "Su-30 0.1.4: roll first on the flaperons, as the F-22E's. The game lowers the flaperons' landing flap 15 deg trailing edge down with the gear; in the air it now comes up with lateral stick or a running roll command (the FlapRollBias schedule: from 10 % of full, all of it by 60 %; out over ~0.3 s, back over ~0.7 s), at the flap's servo rate, so a gear-down roll has the flaperons' full +-25 differential on unstalled flow instead of the down-going side sitting 40 deg trailing edge down. On the wheels the flap stays down. 1 = all of the flap, 0 = 0.1.3 (flap held in rolls). Takes effect at once.");
			cFlapRollBias = dev.Bind("5. Airframe", "FlapRollBias", 1f, "Compass 0.1.5: roll-biased flaps, as the F-22E's flaperons. In the air the flaps' high-lift droop (FlapHighLiftDroop) comes off with lateral stick or a running roll command (from 10 % of full, all of it by 60 %; out over ~0.3 s, back over ~0.7 s), so the pair rolls with its full +-FlapHingeRange differential instead of on the up-going flap alone. 1 = all of the droop, 0 = 0.1.4 (droop held in rolls). Takes effect at once.");
			cCanardPitch = dev.Bind("4. Gains", "CanardPitchPriority", true, "Revoker 0.1.0: during rolls the roll-time nozzle budget (RollTvcBudget) keeps the nozzle within its band of NEUTRAL (F-22E: of the pitch-only trim) at every thrust (F-22E: open from T/W 0.7 to 1.1), so the canards cancel the roll's nose-up / nose-down and carry the pitch trim; the nozzle still goes past the band where the canards cannot (their stall band, AoA protection, AoA/G capture braking).");
			cFlapRelief = dev.Bind("5. Airframe", "FlaperonLoadRelief", false, "Revoker 0.1.0: off (F-22E/Ifrit: on). The flap schedule's high-g term that turns the trailing-edge flaperons up to 9.6 deg TE-up from 3 to 9 g below 16-24 deg AoA. Off, the trailing edges stay at neutral in a held turn and the canards carry the pitch.");
			cYawTvcNeutral = dev.Bind("4. Gains", "YawTvcNeutralHold", 300f, "Revoker 0.1.0: while the lateral stick and pedals are steady, the roll on its command and the sideslip under 2 deg (a held turn), the nozzle's yaw axis is drawn back to neutral over ~1 s and the rudder and canards carry the steady yaw; the nozzle still takes every change. Cost weight; 0 = off. The pitch axis has the same (TvcNeutralHold, canards carrying the steady pitch).");
			cVisHinge = dev.Bind("5. Airframe", "VisualHingeFix", true, "Revoker 0.2.3: draw the trailing-edge surfaces turning about their modelled hinge lines (render only; aero unchanged). Off: the game's own visuals.");
			cCorkTol = dev.Bind("3. Lateral-directional", "CorkscrewSlipTol", 5f, "Su-30 0.1.10: 5 (Tom's setting). Revoker 0.2.2: with roll stick held above ~35-50 deg AoA (pedals centred), sideslip (deg) the sideslip hold lets build before it takes the yaw back; the corkscrew keeps rotating. 0 = 0.2.1.");
			cHiAoARollW = dev.Bind("3. Lateral-directional", "HighAoARollWeight", 1f, "Revoker 0.2.7: 1 (F-22E/Ifrit 0.35). The allocator's roll-axis weight from 40 deg AoA (blended in from 20) while the nozzles do not cover pitch (TVC off or the low-thrust T/W); with them covering it stays 0.35. With the roll target out of reach the pitch (weight 8) and yaw (YawPriority) axes took everything at 0.35 and the roll went the wrong way for moments, which with the game's surface lag set up a ~0.6 Hz cycle of every surface stop to stop (28 Oct 20:31 log, TVC off at the 50 deg cap). Bench, TVC off, 50 deg loaded rolls with a game-like 4-frame command delay: surface slew peaks 35-70 -> 10-45 deg/s, sideslip 6.2-6.9 -> 2.9-4.6 deg, roll 24-27 -> 29-33 deg/s. Takes effect at once.");
			cCanWin = dev.Bind("3. Lateral-directional", "CanardAttachedWindow", true, "Ternion 0.1.0: on, canards only (the Revoker's, same Fighter1_canards foil); the aft elevators are never walked by it. Compass 0.1.1: off (Revoker: on). Above 30 deg AoA with lateral input the window walked the stabilators to keep their local AoA near the foil's knee - right for the FS-12's canards, wrong for the T/A-30's elevators, whose foil keeps CL 0.8-1.0 past its stall: it held them nose-up while AoA protection needed nose-down.");
			cCanWinMargin = dev.Bind("3. Lateral-directional", "CanardWindowMargin", 12f, "Revoker 0.2.7: local AoA (deg) past the canard foil's lift-slope knee (15.8 deg) that CanardAttachedWindow allows. Bench: 6 cost up to 5 deg of AoA in 50 deg roll reversals, 12 held it within 2-4.");
			cFinTvcHold = dev.Bind("3. Lateral-directional", "FinLossTvcHold", true, "Revoker 0.2.7: with the fin gone (yawHealth under 0.5) the engines keep vectoring above their 250 m/s cut-off (raised to 1000 m/s on this jet, put back when the fin is whole or the FCS lets go). The yaw nozzle is the only yaw axis then; at the cut-off the game zeroes the nozzles and the jet yawed round (28 Oct 21:22 log, fin + wings gone, 247 m/s). false = the game's cut-off always. Takes effect at once.");
			cFinNoTvcCap = dev.Bind("2. Envelope", "FinLossNoTvcAoACap", 30f, "Revoker 0.2.7: positive AoA limit (deg) with the fin gone and no nozzle yaw (TVC off, or the thrust under the low-thrust T/W), phased in from no fin damage to the fin gone. Bench, fin gone, TVC off, held pull + roll: 45-50 departs, 40 departs with a game-like command delay, 35 and below hold. 0 = off. Takes effect at once.");
			cSlipYawTvc = dev.Bind("3. Lateral-directional", "SlipYawTvcAssist", true, "Revoker 0.2.1: with the yaw nozzle active, the sideslip gain ramps to SlipGainHighAoA at high AoA (Ifrit: only with roll TVC, which the Revoker has none of).");
			cGroundElevonPitch = dev.Bind("5. Airframe", "GroundElevonPitch", 1f, "Revoker 0.2.1: on the ground, the share of each trailing-edge elevon's travel the pitch stick moves (0-1; full aft = TE up). In the air the allocator uses them for pitch (ElevonPitch).");
			cCanardYawTier = dev.Bind("3. Lateral-directional", "CanardYawTier", false, "Revoker 0.1.0, default off: with the fin or rudder damaged, count the canards' yaw (differential drag) in the first yaw tier with the rudder and the yaw TVC. Off: the canards are still full pitch / roll / yaw effectors in the allocator, but their yaw is not counted as free reach (one canard at 40 deg makes ~45 deg/s^2 of yaw at a pitch and roll cost the allocator will not pay, so counting it kept the split-flaperon tier shut: bench, fins gone at 700 km/h, a pedal kick departed with nothing moving for 1.3 s).");
			cElevonPitch = dev.Bind("5. Airframe", "ElevonPitch", false, "Compass 0.1.0: off. true = the ailerons are pitch effectors as well as roll (symmetric deflection). The T/A-30 has a conventional tail, so the elevators carry the pitch.");
			cElevonSymCost = dev.Bind("4. Gains", "ElevonSymCost", 2f, "Revoker 0.1.0: allocator cost of the symmetric (pitch) part of the flaperon pair (F-22E/Ifrit: 2).");
			cAoANegRev = dev.Bind("2. Envelope", "AoALimitNegativeRevision", 1, "Internal: the AoALimitNegative default revision this file was last updated to. Leave it alone.");
			if (cAoANegRev.Value < 1)
			{
				// 2.2.4: default 45 -> 50. Runs once: a file still holding the old default 45 is moved; any other value is the
				// player's own and is kept. Afterwards the file is at revision 1 and 45 set later stays.
				if (Mathf.Abs(Mathf.Abs(cAoANeg.Value) - 45f) < 0.001f)
				{
					cAoANeg.Value = 50f;
					Log.LogInfo("Su-30 FCS 2.2.4: AoALimitNegative moved from the old default 45 to the new 50 (once).");
				}
				cAoANegRev.Value = 1;
				dev.Save();
			}
			cGPos = dev.Bind("2. Envelope", "GLimitPositive", 9f, "Compass 0.1.0: 9 (the T/A-30's rated aircraftGLimit; stock FBW 8). Positive load limit (g), normal law.");
			cGNeg = dev.Bind("2. Envelope", "GLimitNegative", 3.5f, "Negative load limit magnitude (g), normal law.");
			cMpoGPos = dev.Bind("2. Envelope", "MPOGLimitPositive", 12.5f, "Positive load limit (g) with Stability Assist OFF (MPO).");
			cMpoGNeg = dev.Bind("2. Envelope", "MPOGLimitNegative", 3.5f, "Negative load limit magnitude (g) in MPO.");
			cMaxPitch = dev.Bind("2. Envelope", "MaxPitchRate", 48f, "Pitch-rate budget ceiling (deg/s).");
			cProt = dev.Bind("2. Envelope", "AoARecoveryRate", 48f, "AoA-protection recovery rate (deg/s), independent of thrust and weight.");
			cMaxRoll = dev.Bind("3. Lateral-directional", "MaxRollRate", 250f, "Absolute body roll-rate ceiling (deg/s). Below ~250 m/s IAS the original-aircraft schedule and RollAuthorityUse set the rate; above it this ceiling holds the rate flat.");
			cMaxYaw = dev.Bind("3. Lateral-directional", "MaxYawRate", 90f, "Body yaw-rate limit (deg/s). Revoker 0.2.2: 90 (was 75); bench, 70 deg loaded rolls 68-70 -> 75-80 deg/s velocity-vector roll at the same sideslip, corkscrews body yaw 64 -> 70 deg/s.");
			cRollLo = dev.Bind("3. Lateral-directional", "RollRateGainLowIAS", 1.2f, "Ternion 0.1.0: 1.2 (the F-22E / Revoker value; the T/A-30's 2.0 was for its slow ailerons). Compass 0.1.4: 2.0 (0.1.0-0.1.3: 1.2, the F-22E's). The 'original aircraft' reference is the F-22E's stock roll schedule, inherited; on the T/A-30 it and RollAuthorityUse 1 held a full-stick roll to ~105 deg/s at 300 km/h with the ailerons at 6-7 deg of their 30 (5 Nov 18:12 log). Bench at 2.0 / 1.8 / RollAuthorityUse 2: 1 g full roll 107 -> 180 deg/s at 200 km/h, 142 -> 247 at 300, the 250 ceiling from ~330 km/h, ailerons 15-24 deg, sideslip <= 2.5 deg. Roll-rate ceiling vs the ORIGINAL aircraft's roll rate below ~120 m/s IAS (blends to the high-IAS gain by 170 m/s).");
			cRollHi = dev.Bind("3. Lateral-directional", "RollRateGainHighIAS", 1.1f, "Ternion 0.1.0: 1.1 (F-22E / Revoker). Compass 0.1.4: 1.8 (0.1.0-0.1.3: 1.1). Roll-rate ceiling vs the original aircraft's roll rate above ~170 m/s IAS.");
			cRollBrake = dev.Bind("3. Lateral-directional", "RollBrakeUse", 0.8f, "Fraction of the full roll authority (wings and stabilators) used to stop a roll, on top of roll damping. Before 1.8.26 this was a fraction of the wing-only authority.");
			cRollAuth = dev.Bind("3. Lateral-directional", "RollAuthorityUse", 1f, "Ternion 0.1.0: 1 (F-22E / Revoker; the T/A-30's 2 corrected its own under-read). Compass 0.1.4: 2 (0.1.0-0.1.3: 1). The FCS's steady-roll estimate (roll authority / modelled roll damping) under-reads the T/A-30 by ~2x: at 1 it capped the roll where the ailerons held it with 6-15 deg of their 30. Steady roll never needs more than this fraction of the available roll control moment (roll rate is given up instead of saturating the surfaces).");
			cCouple = dev.Bind("3. Lateral-directional", "CoupledAxisAoA", 36f, "Above this |AoA| the pedals blend into the stick's velocity-vector roll (full coupling 14 deg higher).");
			cPedalBeta = dev.Bind("3. Lateral-directional", "PedalSideslip", 15f, "Sideslip commanded by full pedal below the coupling AoA (deg).");
			cRollAcc = dev.Bind("3. Lateral-directional", "MaxRollAccel", 720f, "Ceiling on the shaped velocity-vector roll acceleration (deg/s^2); below it the measured authority sets the rate.");
			cKBeta = dev.Bind("4. Gains", "SideslipGain", 1.3f, "Stability-axis yaw rate per degree of sideslip error (1/s).");
			cKRoll = dev.Bind("4. Gains", "RollRateGain", 6f, "Roll-rate loop bandwidth (1/s).");
			cKPitch = dev.Bind("4. Gains", "PitchRateGain", 5f, "Pitch-rate loop bandwidth (1/s).");
			cKYaw = dev.Bind("4. Gains", "YawRateGain", 2.5f, "Yaw-rate loop bandwidth (1/s).");
			cBrakeBlend = dev.Bind("4. Gains", "AoACaptureBrakeBlend", 0.3f, "How far the capture braking plan leans from the authority at the target AoA (0) toward what is available right now (1). Lower = earlier braking, less overshoot, slower AoA build; higher = faster AoA build, more overshoot below afterburner. 0.3 is the bench balance.");
			cThrBrake = dev.Bind("4. Gains", "AoACaptureThrustBrake", defaultValue: true, "AoA capture plans its braking with the nose-down authority it can count on at the TARGET AoA: the nozzles' full authority at the live thrust (so less below afterburner and much less at part throttle) plus the stabs' authority at the target AoA from the surface model (almost none past ~45 deg, where they stall). The approach follows sqrt(2 x 0.7 x that x distance), the same deceleration the pitch trajectory can deliver. false = pre-1.8.20.");
			cAntiWind = dev.Bind("4. Gains", "AoAGAntiWindup", 4f, "Back-calculation anti-windup for the AoA and G integrators (1/s): whenever a limiter (AoA/G capture braking, AoA limit, pitch-rate budget) clips the pitch-rate command below what the AoA/G loop asks for, the integrator is pulled back toward the command at this rate. Without it the integrator winds up during every capture and then holds AoA about 0.2 deg above the command until it unwinds (several seconds). 0 = pre-1.8.20.");
			cLatRes = dev.Bind("4. Gains", "LateralStabReserve", 0f, "Ternion 0.1.2: 0 (F-22E / Revoker 0.7). The reserve exists so the nozzles take the pitch while the stabs stay for roll/yaw; the FS-3 has no nozzle pitch, so at 0.7 a corkscrew entry built pitch on 30 % of the canard / elevator authority (bench, full aft + full roll at 450 km/h: 30 deg AoA at 2.47 s against 0.93 s in a plain pull; at 0: 1.17 s, sideslip 2.1 -> 2.3 deg). While you hold pitch stick AND lateral stick (roll or pedal) together, the pitch law builds pitch rate with the nozzles' authority plus only (1 - this x |lateral stick|) of the stabs' remaining authority, and the allocator's cost for symmetric (pitch) stab deflection rises with it, so the nozzles take the pitch and the stabs stay for roll/yaw. Braking toward the commanded AoA/G, the AoA/G limits and protection always use full authority. No lateral stick, or pitch stick released = unchanged. 0 = pre-1.8.19.");
			cPairTrim = dev.Bind("4. Gains", "StabTvcPairedSearch", defaultValue: true, "The allocator also tries moving each stabilator with the nozzles taking up its pitch change, and keeps the pair move when it lowers the total error. Without it, a stab parked on a stalled, flat part of its curve for pitch cannot be moved to where it makes roll, because moving it alone breaks pitch and moving the nozzles alone breaks it the other way. Lets the nozzles carry the pitch trim whenever they have room, freeing the stabs for roll/yaw. false = pre-1.8.18.");
			cBranch = dev.Bind("4. Gains", "StabBranchSwitch", defaultValue: true, "When a stabilator has been driven deep past its stall (local AoA more than 10 deg past the peak) during roll/yaw at moderate AoA, move it to the attached-flow side of its lift peak instead (trailing edge up, where the lift curve is steep), hold it there, and re-allocate everything else around it, the nozzles taking up the pitch. Undone when the modelled pitch error would grow by more than StabBranchPitchTol. false = pre-1.8.21.");
			cOvPrio = dev.Bind("4. Gains", "OvershootPitchPriority", defaultValue: true, "While the nose is pitching up faster than the pitch law commands above ~15-25 deg AoA (an AoA run-away), or AoA protection is active: (1) the allocator keeps pitch priority instead of handing stalled stabs to roll/yaw, and (2) it models the nose-down a stab really loses when it is driven deep past its stall for roll/yaw (the monotone stab pitch model hides that loss). false = pre-1.8.23.");
			cLeadK = dev.Bind("4. Gains", "StickLeadGain", 0f, "2.1.7: G + pitch-rate law only. Extra pitch-rate command on stick movement, washed out over StickLeadTau: a quick small input moves the nose a little at once, like an AoA step, then the plain rate response takes over; a held stick gets none. 0 = off (default). Around 1 = noticeable crispness; much above 1.5 small inputs can feel twitchy. Takes effect at once.");
			cLeadTau = dev.Bind("4. Gains", "StickLeadTau", 0.15f, "2.1.7: wash-out time (s) of StickLeadGain. Longer = the extra nose movement lasts longer and is larger for the same gain.");
			cGovPred = dev.Bind("3. Lateral-directional", "PredictiveRollGovernor", true, "2.1.5: the roll governor (1.8.41, keeps nozzle margin for pitch during rolls) looks at where the nozzle margin is heading 0.4 s ahead and eases the roll off early and in proportion, instead of cutting it once the nozzles are already pinned and restoring it a second later (the roll hitch). It comes back only when the margin is there and no longer falling. false = 2.1.4. Takes effect at once.");
			cYawSlewLead = dev.Bind("3. Lateral-directional", "YawSlewLead", true, "F-22E 2.4.18 (ported in Revoker FCS 0.3.0): the yaw-rate loop looks ahead along the measured yaw acceleration by the time the rudders need to swing back through neutral (their deflection / servo rate), so it starts braking the yaw before the yaw rate reaches its command instead of overshooting it (F-22E 22:36 log, 52 s: yaw rate 42 against 28, sideslip -12 on the reversal). No effect with the rudders near neutral; fades out from 36 to 45 deg AoA. false = off. Takes effect at once.");
			cSplitFlapYaw = dev.Bind("3. Lateral-directional", "SplitFlaperonYaw", true, "2.2.7: decelerons. When the yaw needed is past what the rudders can give (both fins gone, one fin gone, or one engine out with the other's thrust more than the rudders hold), the inner and outer flaperons on one wing open against each other (their rolls cancel, their drag yaws the jet), up to SplitFlaperonYawUse of the drag yaw they have. Only the part the rudders cannot cover; the engine-out VMC throttle limit acts after it. false = 2.2.6 (yaw target limited to the rudders). Takes effect at once.");
			cSplitFlapYawUse = dev.Bind("3. Lateral-directional", "SplitFlaperonYawUse", 1f, "2.2.19: default 1 (was 0.7). Bench, both fins gone, full roll held at 25-40 deg AoA: worst sideslip mean 12.4 -> 10.8 deg, roll tracking unchanged. 2.2.7: share (0-1) of the flaperons' remaining drag yaw the deceleron split may ask for. Higher = more yaw, more roll and pitch disturbance from the split. Takes effect at once.");
			cSplitFlapRollYield = dev.Bind("3. Lateral-directional", "SplitFlaperonRollYield", 0f, "2.2.8: how much of the deceleron split gives way to lateral stick (the split's reach shrinks by this x |lateral stick|, held 1 s after the stick returns). 0 = split kept through rolls (as 2.2.7; least sideslip). 1 = no split while rolling (roll tracks more evenly, but with both fins gone the sideslip in rolls runs much higher, e.g. 15 -> 27 deg replaying your 19 Oct 338-362 s inputs on the bench). Takes effect at once.");
			cRollPitchLead = dev.Bind("4. Gains", "RollPitchLead", 0.25f, "2.2.5: pull + lateral stick (the stabs handed to the roll, the nozzles carrying the pitch). The pitch-rate loop reads the measured pitch rate this many seconds ahead along the measured pitch acceleration, scaled by the nozzles' share of the pitch authority and by how much of the stab reserve the lateral stick has taken, so the slow nozzles stop driving stop to stop and the pitch rate stops swinging about its command (the hitch with an AoA overshoot in a full pull with roll added mid-pull, worse at high IAS). Pulls only. 0 = off (2.2.3). Bench: 0.15 / 0.25 / 0.35 / 0.5 all helped; 0.25 had the least AoA overshoot. Takes effect at once.");
			cSlatRelief = dev.Bind("3. Lateral-directional", "SlatRollRelief", false, "2.1.9 (off by default, experimental): during a roll at 12-45 deg AoA, when the nozzles' nose-down margin is heading for the roll governor's threshold, the slats retract (they are wing area ahead of the CG, so out they push the nose up) to give the pitch axis nose-down moment before the governor has to slow the roll. Costs ~7 % of the lift while they are in (bench: 0.05-0.4 g less in the roll); they come back out when the margin returns or the roll ends. Bench gain is modest: peak roll rate +4..+28 deg/s in 20-30 deg AoA rolls with thrust, no change to the first second of the roll. false = 2.1.8. Takes effect at once.");
			cCoupGateLim = dev.Bind("3. Lateral-directional", "CouplingLimitAtAoALimit", true, "2.1.10: rolls at an AoA limit. (1) While the stick holds the jet within 4 deg of an AoA limit, the roll-rate limit that keeps the roll's inertial pitch-up within the nose-down authority stays on and eases off over 0.6 s; it used to follow the pitch error on and off every ~0.6 s, so the roll rate and the pitch rate bounced. (2) While a velocity-vector roll runs, the AoA-limit capture plans its braking with what is left for pitch less the roll's inertial pitch-up, so it no longer overshoots the limit, hits protection and drops back several degrees in a cycle. In fast rolls the AoA can sit 1-3 deg under the limit. false = 2.1.9. Takes effect at once.");
			cLoadedRollSteady = dev.Bind("3. Lateral-directional", "LoadedRollSteady", true, "2.2.3: on by default (2.2.2: off). full aft + roll. The roll-rate limit that keeps the roll's inertial pitch-up within the nose-down authority stays on for the whole loaded roll (not only within 4 deg of the AoA limit), and the roll governor reads the nozzles' margin as it is instead of projecting it 0.3 s ahead. Both switched on and off every ~0.45 s in the 13 Oct 11:08 log (690 km/h pull + roll: roll target 98 <-> 172 deg/s, nozzles stop to stop, AoA 41..48). Bench: the pitch-rate and nozzle oscillation in the roll about halve; bank after 4 s 0-11 % lower. Only with CouplingLimitAtAoALimit on. Takes effect at once.");
			cSlipYawW = dev.Bind("3. Lateral-directional", "HighAoASideslipAllocation", true, "2.1.10: above ~20 deg AoA, while sideslip is building (from 2 deg, full by 8, counting where it is heading 0.3 s ahead), the control allocator stops trading roll accuracy for yaw (yaw weight 8 -> 2 at 40+ deg AoA): at high AoA a body roll that runs past its command makes sideslip as fast as a yaw-rate error does. No change while the sideslip is small. false = 2.1.9. Takes effect at once.");
			cCapLag = dev.Bind("2. Envelope", "AoACaptureEffectorLag", true, "2.2.1: the AoA-limit capture plans its stop with the pitch effectors' lag (0.3 s x the nozzles' share of the nose-down authority - they slew ~70 deg/s and carry most of it at low IAS), not as if the jet could start braking at once. A re-pull from ~38 deg at ~200 km/h overshot a 45 deg limit by 1.5 deg (3.5 with a roll entry). Where the nozzles carry the authority the AoA can stop 1-2 deg short of the limit and creep the rest. false = 2.2.0. Takes effect at once.");
			cAsymShared = dev.Bind("Asymmetric load", "AsymSharedStabLateral", true, "2.1.4: with an asymmetric load, past the rudder fade the roll and yaw rate limits also give up the stab deflection that trims the load's roll (the stabs are both the roll and the yaw source there), not only the yaw part of the load; past the rudder fade the yaw anchor and sideslip lead also act at positive AoA; and (2.1.6) a roll brake or reversal accelerates no faster than the yaw left after the load can follow. Only the load beyond the deadbands / a lateral CG offset over 4 cm counts; a clean jet is unchanged. false = 2.1.3. Takes effect at once.");
			cLowCapRoll = dev.Bind("3. Lateral-directional", "LowThrustCapRollFix", true, "2.1.3: rolls at the low-thrust AoA cap (36 deg at idle / low T-W). Within 4 deg of the cap the roll-coupling limit stays on instead of waiting for the AoA to rise, and the roll governor judges the nozzle margin only by the nozzles' share of the nose-down authority (pinned weak nozzles no longer read as no pitch reserve). Aimed at the hitches where the roll was cut and restored as the AoA overshot the cap. Only acts at the low-thrust cap. false = 2.1.2. Takes effect at once.");
			cCoupRoll = dev.Bind("4. Gains", "CouplingRollLimit", 0.5f, "While AoA is building past 20-30 deg (or running away), cap the velocity-vector roll rate so the inertial pitch-up it causes, (Izz-Ixx)/Iyy * p * r, stays within this fraction of the nose-down authority the jet has at that AoA (aero + stabs + nozzles at live thrust). This is what let full-aft + full-roll pulls overshoot to 70-77 deg below afterburner. Held-AoA rolls are not affected. 0 = off (pre-1.8.23).");
			cBranchMaxAoA = dev.Bind("4. Gains", "StabBranchMaxAoA", 42f, "Body AoA (deg) above which StabBranchSwitch is not used. Above ~44 deg the stalled side of the stab is the better one for roll/yaw.");
			cBranchTarget = dev.Bind("4. Gains", "StabBranchTarget", 20f, "Local AoA (deg) StabBranchSwitch moves the stab to, on the attached side of its lift peak.");
			cBranchTol = dev.Bind("4. Gains", "StabBranchPitchTol", 5f, "Largest growth in modelled pitch-acceleration error (deg/s^2) StabBranchSwitch accepts. Protects AoA holding when the nozzles cannot take up the stab's pitch.");
			cKpAoA = dev.Bind("4. Gains", "AoAGain", 5f, "AoA command: deg/s of pitch rate per deg of AoA error.");
			cKiAoA = dev.Bind("4. Gains", "AoAIntegral", 3f, "AoA command integral gain (1/s) near the target.");
			cStick24 = cfg.Bind("5. Pitch stick curve", "StickFor24degAoA", 0.6f, "2.0.12 (default 0.6 since 2.0.15; 0.45 in 2.0.13-2.0.14): pull-stick position (0..1) that commands 24 deg AoA. 0.333 = the standard 72 deg per full stick line. Allowed 0.083-0.7: lower = more AoA early in the pull (sharper), higher = less (softer, finer control at low AoA). The curve through 0, this point and the 36 deg point is smooth (monotone cubic), so the gain changes gradually instead of at a corner. Takes effect at once.");
			cStick36 = cfg.Bind("5. Pitch stick curve", "StickFor36degAoA", 0.8f, "2.0.12 (default 0.8 since 2.0.15; 0.65 in 2.0.13-2.0.14): pull-stick position (0..1) that commands 36 deg AoA (on the standard line: 0.5). Allowed 0.3-0.8, and always at least 0.1 of stick past StickFor24degAoA (if set closer it is moved out to 24's point + 0.1, so 36 can never come at or before 24). Past 36 the curve continues to the AoA limit at full stick as before (the high-AoA rate hand-over is unchanged). Applies to the pull side; the push side keeps the 72 deg per stick line. Takes effect at once.");
			cStickRev = cfg.Bind("5. Pitch stick curve", "StickCurveRevision", 2, "Internal: the stick-curve default revision this file was last updated to. Leave it alone.");
			// Revoker FCS 0.3.0 (F-22E 2.6.0): the lateral reference governor's knobs live in the player config; PullRollTrade moves here from the Dev file (its value carried over once)
			cPullRollTrade = cfg.Bind("6. Lateral governor", "PullRollTrade", 0.7f, "2.2.8 (2.2.23: default 0 -> 0.7; 2.2.26: 1; 2.2.27: back to 0.7): in a rolling pull the yaw cannot keep up with, how much of the shortfall is paid by slowing the roll and how much by slowing the AoA rise (the sideslip a rising AoA makes goes as roll rate x AoA rate). 0 = roll kept, AoA rise limited. 1 = roll slowed (to no less than 30 deg/s) until the asked AoA rise fits; in between, the roll limit goes that share of the way and the pitch covers the rest (F-22E 2.6.4/2.6.5). The roll comes back once the AoA rise is no longer limited. Takes effect at once.");
			cRollStopSlip = cfg.Bind("6. Lateral governor", "RollStopSideslip", 10f, "Su-30 0.1.4: default 6 -> 10 (Tom 8 Nov: the sideslip is too tight at mid to high AoA; a file still holding the old 6 under the old default moves once). Revoker FCS 0.7.0: default 8 -> 6 (Tom, 5 Nov 01:10: roll reversals at low to mid AoA made 7-9 deg, the margin spent in every one; 2 deg less, at some roll rate there). Revoker FCS 0.4.0: default 8 (the F-22E flies 4). From your 30-31 Oct logs the old lateral stack ran this jet at 7-8 deg (95th percentile) of sideslip above 45 deg AoA and 2.6-3.3 at 15-45, and the rates you were used to came with that; the governor at 4 kept the sideslip to 2-4 and took the yaw rate in the falling leaf from 77 to 54 and the moderate-AoA roll from 64 to 46 deg/s (Revoker 0.3.0 flight). 8 is where this jet was; lower = cleaner stops, slower rolls. F-22E 2.4.17 (2.5.0: 3 -> 4): sideslip (deg) a roll stop or reversal is allowed to make; the lateral governor caps the commanded roll rate (never the roll acceleration or braking) to the rate whose full stick reversal the yaw can follow within it. The one margin at every AoA (no schedule); the airframe probe (RollStopSlipProbe) can only lower it. A handling (sway) number: lower = slower rolls at moderate AoA, cleaner stops. Allowed 1-15. Takes effect at once.");
			if (rssOldDef != null && rssOldDef.StartsWith("6") && Mathf.Abs(cRollStopSlip.Value - 6f) < 0.001f)
			{
				cRollStopSlip.Value = 10f;
				cfg.Save();
				Log.LogInfo("Su-30 FCS 0.1.4 default moved once: RollStopSideslip 10.");
			}
			cGovRev = cfg.Bind("6. Lateral governor", "GovernorRevision", 3, "Internal: the lateral-governor default revision this file was last updated to. Leave it alone.");
			cRollGovPitch = cfg.Bind("6. Lateral governor", "RollGovernorPitch", true, "F-22E 2.5.2, rebuilt 2.5.3-2.6.6: roll first, then pitch, both ways. With the roll already held to its cap, the AoA rate the pitch law may command under the roll is held to what the yaw can pay for: an AoA sweep changes the yaw the pair needs at roll rate x cos AoA x AoA rate. Nose-up at positive AoA: 0.8 x the rudders' braking yaw not yet in use on the roll's side, faded in with the rudders' braking yaw and only as the forecast sideslip heads past 75 % of the margin - a rolling pull that pins the rudders but holds its sideslip keeps its pitch rate. Every other sweep (nose-down anywhere, nose-up from negative AoA): 0.5 x the yaw authority plus half the roll's, from the same rule, at every IAS; a sweep toward zero AoA under a body yaw rate is paced so the yaw that will have nothing to coordinate at zero AoA is shed first (weighted by 1 - cos AoA, so the nose is let through near zero). Never under 6 deg/s of AoA rate; released over 0.3 s. false = the trade alone. Takes effect at once.");
			cRollPaceRateSlip = cfg.Bind("6. Lateral governor", "RollPaceRateSlip", 1f, new ConfigDescription("F-22E 2.5.10: how much of the sideslip margin the roll-rate cap spends - the roll rate / sideslip compromise. 1 = the cap aims a full stick reversal at the margin as set. Above 1 the cap aims past the margin and lets the roll run faster (F-22E bench, margin 15: worst / mean sideslip and mean roll rate 5.8 / 3.9 at 52 deg/s for 1.0, 8.3 / 4.8 at 59 for 1.25, 10.7 / 6.3 at 67 for 1.67, 13.0 / 7.6 at 74 for 2.0, against 10.9 / 8.0 at 83 with no cap); the game's worst held-roll reversal reads about 1.4x the bench's. Below 1 the cap is stricter than the margin. Allowed 0.5-3. Takes effect at once.", new AcceptableValueRange<float>(0.5f, 3f)));
			cRollPaceRateBlend = cfg.Bind("6. Lateral governor", "RollPaceRateBlend", 0.4f, new ConfigDescription("Su-30 0.1.9: default 0.65 -> 0.40 (Tom 9 Nov); a 0.1.0-0.1.8 file still holding 0.65 is moved once, a value set by hand is kept. Revoker FCS 0.8.1: default 0.4 -> 0.65 (Tom, 5 Nov 01:40: low to mid AoA roll sideslip somewhere between 0.5.1 and 0.8.0). F-22E 2.5.11: how the pace governor splits its work between its two laws. 0 = pacing only (the yaw-reach window holds the roll target while the yaw catches up and the trajectory's acceleration / braking is capped at the yaw's authority: reversals are paced, the margin does not matter). 1 = rate cap only (the target is capped at the rate whose stick reversal lands on the sideslip margin x RollPaceRateSlip; acceleration and braking stay as tuned: reversals are crisp, the rate is lower). Between: the proportional mix of the two targets, and the acceleration cap gives way to the tuned acceleration in the same proportion. The F-22E flies at ~0.4. Allowed 0-1. Takes effect at once.", new AcceptableValueRange<float>(0f, 1f)));
			cRollStopSlipProbe = cfg.Bind("6. Lateral governor", "RollStopSlipProbe", true, "F-22E 2.5.15/2.5.16: the airframe's own sideslip tolerance, measured from its aero tables in flight - no number entered. Twice a second the FCS evaluates the whole airframe (every aero part at the game's per-part law, the surfaces where they are) at the present AoA and a sweep of hypothetical sideslip to 20 deg each way, and finds the sideslip where the restoring yaw moment stops growing (directional stiffness gone - the departure boundary) and where the yaw effectors' or the roll surfaces' authority has fallen to half of its zero-sideslip value. The smaller, x 0.7, caps RollStopSideslip. On an airframe with nothing inside 20 deg the cap never binds and RollStopSideslip rules; on one with a soft fin it binds where the fin gives out. Telemetry: slipTol / slipDep / slipAuth / slipStiff. false = RollStopSideslip alone. Takes effect at once.");
			cPushSlip = cfg.Bind("6. Lateral governor", "PushoverSideslip", 15f, "Su-30 0.2.4: 15 with TVC (Tom 10 Nov 14:55: \"with tvc on is 15\"; a file still holding 8 moves to 15 once); PushoverSideslipNoTvc applies with the nozzles off. Revoker 0.6.0 (the F-22E 3.0.0 push governor and push plan): 8, set explicitly (Tom, 4 Nov: the stock jets' RollStopSideslip, not the F-22E's 4); -1 = follow RollStopSideslip. F-22E 2.9.4: sideslip (deg) a push-over from high AoA may make at the zero crossing from the yaw rate still on board. The sweep is paced so the yaw above what that allowance can absorb (r = sqrt(2 x measured shed x allowance)) is shed before the AoA reaches zero; the rest is left to the rudders at low AoA. Higher = faster nose-down with a high yaw rate on, more sideslip at the bottom. -1 = RollStopSideslip. 0-25. Takes effect at once.");
			cPushSlipNoTvc = cfg.Bind("6. Lateral governor", "PushoverSideslipNoTvc", 12f, "Su-30 0.2.4 (Tom 10 Nov 14:55: 12 without TVC): PushoverSideslip while the nozzles do not cover pitch and yaw (DisableTVC, or T/W under LowThrustFixedAoATW). -1 = PushoverSideslip. 0-25. Takes effect at once.");
			if (cStickRev.Value < 1)
			{
				// 2.0.13: new defaults 0.45 / 0.65; a file still holding 2.0.12's 0.333 / 0.5 is moved once
				if (Mathf.Abs(cStick24.Value - 0.3333f) < 0.001f)
				{
					cStick24.Value = 0.45f;
				}
				if (Mathf.Abs(cStick36.Value - 0.5f) < 0.001f)
				{
					cStick36.Value = 0.65f;
				}
				cStickRev.Value = 1;
				cfg.Save();
			}
			if (cStickRev.Value < 2)
			{
				// 2.0.15: new defaults 0.6 / 0.8. Runs once, on the first start of 2.0.15 or later: a file still holding
				// the 2.0.13-2.0.14 defaults (both 0.45 and 0.65) is moved; any other pair is the player's own and is kept.
				// From then on the file is at revision 2 and is never touched again, so 0.45 / 0.65 set later stays.
				if (Mathf.Abs(cStick24.Value - 0.45f) < 0.001f && Mathf.Abs(cStick36.Value - 0.65f) < 0.001f)
				{
					cStick24.Value = 0.6f;
					cStick36.Value = 0.8f;
					Log.LogInfo("Su-30 FCS 2.0.15: pitch stick curve moved from the old defaults 0.45 / 0.65 to the new 0.6 / 0.8 (once).");
				}
				cStickRev.Value = 2;
				cfg.Save();
			}
			cStickSmooth = cfg.Bind("1. General", "StickSmoothing", 0.06f, "Time constant (s) of the first-order smoothing on pitch/roll/pedal inputs. 0 = off.");
			cActDamp = dev.Bind("5. Airframe", "ActuatorDamping", 0.06f, "Actuator smoothing (s): time for a servo command to accelerate to full servo speed (and brake into its target on a matching curve). Servo speed is unchanged and a steady slew runs at full speed. 0 = off.");
			cYawStop = dev.Bind("3. Lateral-directional", "YawStopFactor", 0.7f, "Above 15-25 deg AoA the body yaw rate is capped at this x measured yaw authority (so the yaw can be stopped again). Higher = more responsive moderate-AoA rolls, more sideslip on stops.");
			cRollLead = dev.Bind("3. Lateral-directional", "RollLagCompensation", 0.008f, "Roll command lead per unit of roll damping (s per 1/s above 2.5/s): cancels the roll loop's tracking lag at high dynamic pressure so roll stops end when the command does, without a slow tail. 0 = off.");
			cYawPri = dev.Bind("4. Gains", "YawAxisPriority", 8f, "Allocator weight of the yaw axis relative to roll below ~20 deg AoA. Higher = rudders keep the roll coordinated even when their roll side-effect costs some roll acceleration.");
			cYawFF = dev.Bind("4. Gains", "YawRateFeedForward", 0.5f, "Fraction of the yaw-rate command's rate of change fed forward to the yaw loop at low AoA below 200 m/s IAS (blends to 1.0 by 300 m/s IAS, and by 28 deg AoA, where the body yaw is the velocity-vector roll). Lower = calmer rudders at low speed, slightly more sideslip.");
			cLowYaw = dev.Bind("3. Lateral-directional", "LowIasYawBoost", 1.7f, "Above 15-25 deg AoA, widens the yaw-rate cap (YawStopFactor x yaw authority, and its 20 deg/s floor) by this factor at very low IAS (falling leaf). It fades out along a curve above LowIasYawBoostFullIAS (see LowIasYawBoostFalloff). 1 = off.");
			cLowYawIas = dev.Bind("3. Lateral-directional", "LowIasYawBoostFullIAS", 70f, "IAS (m/s) up to which the full LowIasYawBoost applies.");
			cLowYawFall = dev.Bind("3. Lateral-directional", "LowIasYawBoostFalloff", 20f, "Above LowIasYawBoostFullIAS the extra margin decays exponentially with this IAS scale (m/s): 10 = ~50 % of it left at +7 m/s, ~10 % at +23 m/s, gone by 120 m/s.");
			cTvcMove = dev.Bind("4. Gains", "TvcMoveCost", 2E-05f, "Allocator cost of moving the TVC nozzles. 2e-5 = nozzles take pitch changes first; ~0.02 = stabs do the quick pitch changes and the nozzles slowly take over the sustained (trim) share. Post-stall the nozzles are always fast.");
			cTvcNeutral = dev.Bind("4. Gains", "TvcNeutralHold", 1000f, "2.2.14: while the pitch rate is held steady (no change of pitch-rate command, no pitch error, no roll in progress) the nozzles are drawn back to neutral over ~1 s and the stabs carry the steady pitch moment; the nozzles still take every change. Cost weight: 1000 (default); 0 = off (2.2.13). Takes effect at once.");
			cStabPark = dev.Bind("4. Gains", "StabParkEscape", true, "0.4.3 (the F-22E's 2.3.0/2.3.1 settled stall window): with the lateral stick and pedals centred and the lateral rates settled (under SettledRateOn deg/s, off again above SettledRateOff or with lateral input), the stabilators and rudders may not rest past the smaller of (lift peak - SettledStallMargin) and the foil's lift-slope knee (SettledSlopeFrac) of local AoA, at any AoA where such a range exists. This jet's stabilator foil has no stall before 45 deg (its lift keeps rising from 20 to 45 deg at a fifth of the linear slope, the drag four times), so a stab parked out there had next to no gradient for the allocator and made its maximum lift: 23 Oct 00:51 log, left stab on its stop for 10 s after a push through zero AoA at 58 m/s, the nozzle and the crossed wing against it. 0.4.2 and earlier only acted below 30 deg AoA and allowed the peak itself. false = off. Takes effect at once.");
			cSettledMargin = dev.Bind("4. Gains", "SettledStallMargin", 6f, "0.4.3: how far below the lift peak (deg of local AoA) the settled stall window keeps the stabilators and rudders (see StabParkEscape); the foil's slope knee applies where it is lower. Larger = further onto the attached slope, less trim deflection available from them. Takes effect at once.");
			cSettledSlope = dev.Bind("4. Gains", "SettledSlopeFrac", 0.15f, "0.4.3: the settled stall window's limit is also where the foil's lift slope has fallen to this fraction of its steepest (the knee of a plateau foil: 26 deg on the stabilator foil at 0.15, where the slope is 0.66 /rad against 4.4 at zero; 0.25 = 17 deg, 0.1 = 31 deg). 0 = the peak - SettledStallMargin only. Takes effect at once.");
			cSettledOn = dev.Bind("4. Gains", "SettledRateOn", 10f, "0.4.3: the settled stall window (StabParkEscape) comes on when the body roll and yaw rates are both under this (deg/s). Takes effect at once.");
			cSettledYawUse = dev.Bind("4. Gains", "SettledYawUse", 0.4f, "0.4.3: the settled stall window is held off while the thrust yaw (an engine out or down on thrust) uses more than this fraction of the fins' yaw authority at the present AoA (|asymAy| / asymCy, filtered 0.5 s, back on under this - 0.1): this jet has no rudders, and with an engine out a stabilator run out past its lift knee is its drag rudder (bench, engine-out pull at 250 km/h: windowed, the yaw saturated and the VMC cut the throttle). The 00:51 log's parked set had 0.29; the engine-out trims run 0.5-1.3. Takes effect at once.");
			cSettledOff = dev.Bind("4. Gains", "SettledRateOff", 20f, "0.4.3: the settled stall window goes off again above this body roll / yaw rate (deg/s), or with lateral input. Takes effect at once.");
			cSettledWalk = dev.Bind("4. Gains", "SettledWalkRate", 30f, "0.4.3: deg/s at which the settled window walks a surface resting past it back in (the window's edge moves, the allocator re-solves around each step). Takes effect at once.");
			cRollFightRel = dev.Bind("4. Gains", "RollFightReleased", true, "2.2.16: the roll-fight cost (RollFightCost) also acts while the lateral stick is centred, not only with the pedals in. With the stick released, a crossed set of surfaces - one stab split against the flaperons, ailerons and rudders, all near their stops, their roll cancelling - could hold for 10-15 s (19 Oct 18:05 / 18:32 logs, TVC off). false = 2.2.15. Takes effect at once.");
			cGndBrakeSplit = dev.Bind("7. Airbrake", "AirbrakeSplitInFlight", defaultValue: false, "Ternion 0.1.3: false (Tom: the stock airbrake was doing stupid stuff). The stock FS-3 brake splits all eight trailing-edge surfaces (elevators, flaperons, rollervons and rudders), and the FCS had widened every split to 25 deg: the elevators and rudders clamshelled and the split ate the flaperons' and rudders' travel. false = the in-flight brake is the F-22E trailing-edge set alone (TrailingEdgeBrakeConfig). In the AIR, the brake control deploys the airframe's split surfaces (the stock speedbrake mechanism). On the ground they stay stowed and the airbrake is the rollout surface configuration alone. The stock rule deployed them whenever the throttle was at idle, airborne as well as on the ground; the FCS drives them now, so idle throttle deploys nothing. false = split surfaces never deploy.");
			cAbElevUp = dev.Bind("7. Airbrake", "AirbrakeOutboardElevonUp", 0f, "Su-30 0.1.0: 0 (no outboard pair). Ternion lineage: Revoker 0.2.8 (TrailingEdgeBrakeConfig): outboard elevons trailing edge up with the airbrake out in the air, as the F-22E's ailerons. 0.2.9: the deflection is solved every frame so that the set adds no lift (with the canards' re-trim of its pitch counted): 14-19 deg on the bench at 400-1200 km/h. This key only switches the outboard share on (> 0) or off (0); 0.2.8 used it as a fixed deflection. The differential stays free for roll.");
			cTeBrakeFade = dev.Bind("7. Airbrake", "TrailingEdgeBrakeAoAFade", true, "Revoker 0.2.10 (TrailingEdgeBrakeConfig): the in-flight elevon brake set closes with AoA as the F-22E's speedbrake does (its AirbrakeAoAFade: full to ~15 deg AoA, half at ~25, stowed past ~35 on the clean jet): full to 15 deg |AoA|, closing linearly to stowed at 35, reopening as the AoA comes back. false = 0.2.9 (always the full set with the brake held). Takes effect at once.");
			cAbInbDn = dev.Bind("7. Airbrake", "AirbrakeInboardElevonDown", 0f, "Su-30 0.1.0: 0. With only one flaperon pair (no outboard pair to cancel its lift) the in-air brake is the dorsal airbrake alone. Ternion lineage: Ternion 0.1.0: 25 (of 25). Compass 0.1.0: 20 (of 20). Flap trailing-edge-down deflection with the airbrake out in the air (TrailingEdgeBrakeConfig).");
			cGbElev = dev.Bind("7. Airbrake", "GroundBrakeOutboardElevon", 0f, "Su-30 0.1.0: 0 (no outboard pair; the outer flaps do not move). Ternion lineage: Ternion 0.1.0: 25 (of 25), rollervons. Compass 0.1.0: aileron trailing-edge-up deflection (deg, of 30) in the rollout aero-brake.");
			cGbInb = dev.Bind("7. Airbrake", "GroundBrakeInboardElevon", 25f, "Ternion 0.1.0: 25 (of 25), flaperons. Compass 0.1.0: 20 (of 20). Flap trailing-edge-up deflection in the rollout aero-brake.");
			cGbCan = dev.Bind("7. Airbrake", "GroundBrakeCanard", 65f, "Su-30 0.1.0: 65 = the full canard leading-edge-down travel on the runway (brake alone or full push alone). Ternion lineage: Ternion 0.1.0: 60 = the full canard leading-edge-down travel (Revoker rule: on the runway the brake alone or full push alone takes the canards to the stop). Canards only; the aft elevators stay on the pitch stick. Compass 0.1.0: 0. Elevator trailing-edge-up deflection (deg) in the rollout aero-brake. On the T/A-30's aft tail trailing edge up is a nose-UP moment that unloads the nose wheel (full travel at 60 m/s is about the nose wheel's load), so the elevators stay on the pitch stick. (Revoker: the canards' leading edge down.)");
			cRotMinSpeed = dev.Bind("Ground", "RotationMinSpeed", 45f, "The take-off/landing rotation law (closed-loop pitch with the mains on the ground) only arms above this TAS (m/s). It used to arm at 15 m/s, an ordinary taxi speed, where it flew closed-loop pitch against the ground constraint and threw the surfaces about. It disarms 8 m/s below this.");
			cRateCapFloor = dev.Bind("3. Lateral-directional", "HighAoARollRateCapFloor", 0f, "Su-30 0.1.4: default 0.7 -> 0 (Tom). Su-30 0.1.2 (Tom 8 Nov: high-AoA yaw/roll authority worse than the Raptor and Ifrit FCS). The roll governor's rate cap (RollPaceRateBlend) is a model of the rudders - their crossing time and tables - so it now counts in proportion to how much of the yaw axis the rudders are (their share of the one-side yaw capacity, rudders + canted nozzles, full from 50 %), down to this floor. The F-22E / Ifrit drop that cap entirely past their 50-60 deg rudder fade; the Su-30 keeps its fins to 75-85 deg, so at 70 deg and 160 km/h the cap counted in full on the rudders' ~3 deg/s^2 against the nozzles' ~23 and held the velocity roll flat at 23-26 deg/s from 110 to 400 km/h (7 Nov logs). It is the roll rate / sideslip trade at low speed and high AoA. Bench, 250 km/h corkscrew (full aft + full roll): 1 = 0.1.1, 32 deg/s, sideslip 4.4 deg; 0.7 = 36 / 5.3; 0.5 = 39 / 7.8; 0 = 47 / 14 (the yaw cannot keep up). Pull 0.8: 36 / 3.3, 42 / 4.7, 45 / 6.4, 55 / 14.1.");
			cHiAoAVvW = dev.Bind("3. Lateral-directional", "HighAoAVvRollWeight", 4f, "Su-30 0.1.1 (Tom 7 Nov: the TVC only seems to do pitch; they are canted for velocity roll). The allocator's velocity-roll weight from 45 deg AoA (blended in from 25), while the canted nozzles have thrust to vector and the roll stick (or coupled pedal) is in (it was 1.5, against 1 for pitch); a released stop keeps the old weights. The nozzles' differential is mostly a velocity-roll effector up there and shares each nozzle's single axis with pitch; at the old weight both nozzles stayed at full nose-up through a rolling pull (log 23:24, 67.8-68.9 s). The roll governor still sets the roll rate; this decides who gets the nozzles when both axes want them. 1 = 0.1.0.");
			cGroundRate = dev.Bind("Ground", "GroundRateLaw", defaultValue: true, "2.0.6: on the runway above RotationMinSpeed, pulling the stick hands pitch to the same rate law as gear-down flight, held through the rotation and lift-off (the rate command, its limits and stick scaling are the airborne ones). Stick neutral or forward with all three wheels down for 0.3 s goes back to the open-loop ground mode (steering, aero-brake). false = 2.0.5 behaviour (open-loop stick until the nose is up, then RotationRate).");
			cGroundRotRamp = dev.Bind("Ground", "GroundRotationRamp", 600f, "Revoker 0.2.9: 600 (F-22E 60). At 60 the nose stayed on the wheel for 0.7-0.85 s after a full pull at 200-320 km/h on the bench (in game, 29 Oct 00:25 log at 283 km/h: two full pulls of 0.45 s each did not lift it); at 600 it lifts in 0.16-0.20 s, with the same or smaller pitch-rate overshoot after lift-off. 2.0.6: while the nose wheel still touches in the ground rate law, the nose-up pitch moment the FCS asks for may grow by at most this much per second (in deg/s^2 of pitch acceleration). Stops the surfaces winding to full deflection against the nose leg and over-rotating when it lifts.");
			cNwsFbw = dev.Bind("Ground", "NoseWheelFbw", true, "1.8.36: fly-by-wire ground steering. Pedal commands a yaw rate (GroundYawRateMax at taxi speed, limited to GroundLateralAccel of turn at speed); pedal released = yaw braking to zero rate, then the jet holds a straight line along the track it had (a crab/drift at touchdown is steered out, the heading lined up with the track). The nose wheel is steered by the FCS (kinematic feed-forward + yaw-rate feedback) and, in the ground roll, the rudders join the yaw-rate loop. Below 1.5 m/s the pedal steers the wheel directly (pivoting). false = stock: pedal sets the nose-wheel angle, rudders follow the pedal.");
			cGndYawRate = dev.Bind("Ground", "GroundYawRateMax", 15f, "Full-pedal yaw rate on the ground (deg/s) at taxi speed.");
			cGndLatAcc = dev.Bind("Ground", "GroundLateralAccel", 2.5f, "Full pedal never asks for more than this lateral acceleration (m/s^2) in a ground turn: the yaw-rate command is min(GroundYawRateMax, this / ground speed). 2.5 = full pedal gives 2 deg/s at 260 km/h.");
			cGndLine = dev.Bind("Ground", "GroundLineHold", 1f, "Pedal released: how strongly the jet steers back onto the straight line it was on (1 = full, 0 = hold the track direction only and accept the sideways offset a gust or touchdown drift left).");
			cGndStock = dev.Bind("Ground", "GroundStockMatch", true, "0.5.2: with NoseWheelFbw on, stock steering with a drift limit and the straight-line correction. While the pedal is in (and after release until the yaw has stopped) the nose wheel, the rudders and the differential brake get the stock fly-by-wire's yaw, computed with the jet's own stock parameters (yaw-rate command with yaw damping; raw pedal below the stock minimum speed), so the turn is stock; GroundDrift* limit the sideslip on top. Once the yaw has stopped the FCS holds the straight line along the track until the pedal moves; at touchdown it holds the approach track. GroundYawRateMax and GroundLateralAccel are not used. false = the capped yaw-rate law of 1.8.36-0.4.9.");
			cGearDownSlip = dev.Bind("Ground", "GearDownSlipProtect", true, "0.5.3: the negative-AoA push + roll limiters (sideslip lead and yaw-anchored roll, 2.1.0) also act with the gear down in the air (not with a wheel on the ground). false = off with the gear down, as before.");
			cGndIndi = dev.Bind("Ground", "GroundIndi", true, "0.5.4: INDI on the nose wheel after the pedal is released - the release braking (stop the yaw) and the straight-line / touchdown hold. The wheel effectiveness comes from the game each tick (nose-leg load, tyre friction and slope, CG and yaw inertia); the measured yaw acceleration covers the rest. While the pedal is in the steering stays stock. false = 2.4.13 (stock law on release, PI line hold).");
			cGndIndiK = dev.Bind("Ground", "GroundIndiGain", 15f, "0.5.4: INDI nose wheel: wanted yaw acceleration per deg/s of yaw-rate error (1/s).");
			cGndIndiTau = dev.Bind("Ground", "GroundIndiTau", 0.04f, "0.5.4: INDI nose wheel: low-pass time constant (s) on the measured yaw acceleration and the tyre-force estimate.");
			cGndPedalBrake = dev.Bind("Ground", "GroundPedalBrake", 0.5f, "Su-30 0.5.4: while the pedal is in, the main on the pedal's side is braked by |pedal| x this (stock: 0.3, and only 3-30 m/s; this works at any ground speed). The jet pivots harder and the mains break loose, so it drifts. Bench, 3 s full pedal from 15 kt at idle: peak sideslip 7.1 deg (stock) -> 12.8 (0.5) -> 20.6 (1.0); speed after 3 s 13.2 -> 9.2 -> 5.9 kt. Never below your own brake. 0 = stock.");
			cFlapSched = dev.Bind("5. Airframe", "FlaperonFlapSchedule", true, "Su-30 0.5.5: the flaperons' flap (the game's landing-flap hinge under each flaperon) follows the F-22E's flap schedule instead of the landing gear: trailing edge down with AoA, load and the high-lift speed schedule (full at 100 m/s and below, out by 130 m/s; up to 8 deg of manoeuvre droop at speed), held 5 deg off the flaperon's stall with 12 deg x lateral stick of headroom, faded out from 20 to 30 deg AoA, off with the rollout aero-brake. The flaperons keep their full roll travel on top, and the flap still comes up for roll (FlaperonRollPriority). Gear up or down makes no difference. false = 0.5.4 (game: 15 deg with the gear down, 0 with it up).");
			cFlapSchedMax = dev.Bind("5. Airframe", "FlaperonFlapMax", 23f, "Su-30 0.5.5: most flap (deg, trailing edge down) the schedule may set; 23 = the F-22E's schedule limit, 15 = the game's landing flap.");
			cGndLightNose = dev.Bind("Ground", "GroundIndiLightNoseHold", true, "Su-30 0.5.3: when the nose leg goes light during a release stop or the straight-line hold (the jet rocks on its suspension when a hard turn is released; 11 Nov 18:55 log: 3 ticks with no usable load, then 0.5 s off the ground), the nose wheel keeps the angle INDI had relative to the contact velocity instead of dropping to the stock law's angle (full opposite lock) or 0. The game slews the wheel toward its command even off the ground, so 0.5.2 let it run to ~0 deg and met the ground again at ~28 deg of slip. false = 0.5.2.");
			cGndIndiBrake = dev.Bind("Ground", "GroundIndiBrake", true, "Su-30 0.5.3: the main-wheel brakes as a second yaw effector for the INDI after the pedal is released (release stop and straight-line hold). The nose wheel goes first; the brake on the outside main takes only the yaw acceleration beyond the nose tyre's grip, read from each main's load, friction and contact velocity every tick. Your own brake is never reduced and the stock differential brake stays underneath. Each main's brake is written directly, so it does not depend on the stock 3-30 m/s window. Costs speed: bench, release from a full-pedal turn at 10 kt: stop 1.50 -> 1.27 s, 6.4 kt lost instead of 1.7. false = 0.5.2.");
			cGndIndiBrakeMax = dev.Bind("Ground", "GroundIndiBrakeMax", 1f, "Su-30 0.5.3: most brake (0-1) the yaw effector may put on one main. Lower = less speed lost per stop, slower stop. Bench, 10 kt: 0.4 = stop 1.45 s / 2.7 kt lost, 0.6 = 1.38 s / 4.1 kt, 1.0 = 1.27 s / 6.4 kt (0.5.2: 1.50 s / 1.7 kt).");
			cGndIndiBrakeFull = dev.Bind("Ground", "GroundIndiBrakeFullSpeed", 7.7f, "Su-30 0.5.3: full brake authority at and below this ground speed (m/s; 7.7 = 15 kt).");
			cGndIndiBrakeZero = dev.Bind("Ground", "GroundIndiBrakeZeroSpeed", 12.9f, "Su-30 0.5.3: brake authority fades to zero at this ground speed (m/s; 12.9 = 25 kt). Above it the ground steering is 0.5.2.");
			cGndDriftAllow = dev.Bind("Ground", "GroundDriftAllow", 8f, "0.5.2: sideslip (deg) the jet may drift to on the ground before the steering counters it (stock steering does not limit it). 8 = about half of the Ifrit stock-pedal drift above 50 kt (27 Oct logs: 17-25 deg peaks, 42 in a slalom). 0 = hold the nose on the track; large = no limit.");
			cGndDriftGain = dev.Bind("Ground", "GroundDriftGain", 0.2f, "0.5.2: counter-yaw per degree of sideslip past GroundDriftAllow (1 = full pedal; 0.2 = full counter-yaw 5 deg past the allowance). 0 = drift limit off.");
			cGndDriftLead = dev.Bind("Ground", "GroundDriftLead", 0.25f, "0.5.2: the drift limit acts on the sideslip predicted this far ahead (s) from its rate, so a fast skid is caught before it overshoots.");
			cGndDriftFade = dev.Bind("Ground", "GroundDriftFadeSpeed", 15f, "0.5.2: the drift limit fades in from this ground speed (m/s) to 10 m/s above it (default 15-25 m/s = 29-49 kt), so tight taxi turns keep their natural sideslip.");
			cPitchFight = dev.Bind("4. Gains", "RollPitchFightCost", 1f, "1.8.42: while you roll, a stabilator pitching the nose up (trailing edge up) while the nozzles pitch it down to cancel costs the allocator this x its pitch acceleration squared (rad/s^2), so it rolls with less stab trailing-edge up and the nozzles keep their nose-down travel instead of the roll rate. Near the stabs' roll reversal (the trailing-edge-down stab stalled) one stab went fully trailing-edge up with the other near neutral and the nozzles pinned nose-down. Only that pattern counts (stabs nose-down with nozzles nose-up, the roll-pitch-priority trim, is left alone); off while the pitch-rate command is moving, while the nose comes up faster than commanded, in AoA protection and on AoA run-away. Bench vs 1.8.41: reversals at 19 deg AoA / 300 km/h nozzle min -20 -> -13, AoA over 1.2 -> 0.5 deg; roll stop at 200 km/h -14 -> -10; 2.5 t store at 600 km/h -20 -> -12, over 2.9 -> 0.9; at 26 deg a stab past 35 deg local AoA 32 % -> 0 % of the roll. Costs: reversal at 13 deg / 600 km/h over 3.9 -> 8.3 deg, 10 deg / 300 km/h 1.1 -> 2.7; bank at 1 s at 19-22 deg AoA 42 -> 36 and 28 -> 20 deg. 0 = off (1.8.41).");
			cRotRate = dev.Bind("2. Envelope", "RotationRate", 8f, "Only with GroundRateLaw = false: takeoff / landing rotation (main wheels on the ground, nose wheel up): full stick commands this pitch rate (deg/s) and neutral stick holds the nose attitude.");
			cRollTvc = dev.Bind("4. Gains", "RollNozzleHold", 0f, "1.8.31: default 0 (the handover kept both stabs trailing-edge down for whole roll cycles and bounced the nozzles in roll spam; with ±20 deg the nozzles now take the roll trim). Low-AoA rolls with no pitch command: the stabs take over the roll surfaces' pitch-moment change so the TVC stays near its pre-roll trim. Below ~150 m/s IAS the nozzles still catch the fast part and hand the sustained part to the stabs as far as the stabs have travel to spare; above it the nozzles simply hold still. 1 = full, 0 = off (nozzles trim the roll alone, 1.7.0 behaviour).");
			cAsymPitchMargin = dev.Bind("4. Gains", "AsymPitchMargin", 20f, "1.8.37: with an asymmetric load (lateral CG offset above ~8 cm) the asymmetric-load AoA cap is also held below the AoA where the jet runs out of nose-down authority - full nose-down stabs plus the nozzles at the thrust you have - minus this margin (deg/s^2), less 2 deg. On Aryx's airframe that point is only ~18-19 deg AoA at idle (the stabs stall trailing-edge down), so at idle the old cap (up to ~21 deg) sat right on the edge and any overshoot pitched up into a departure. Throttle up and the nozzles raise it again. Never below 10 deg. 0 = off (1.8.36).");
			cAsymRollCoupling = dev.Bind("4. Gains", "AsymRollCoupling", true, "1.8.37: with an asymmetric load, the roll-coupling limit (CouplingRollLimit) also works at a held AoA, not just while AoA is building past 12-22 deg, and pitch gets priority over roll as soon as AoA overshoots the asymmetric-load cap. Rolling toward the heavy wing at 12-20 deg AoA and idle used to reach 100-160 deg/s and then pitch up past the cap it could not stop in. Clean jets are unaffected. false = 1.8.36.");
			cRollPitchPriority = dev.Bind("4. Gains", "RollPitchPriority", true, "1.8.40: while you roll (below ~18-26 deg AoA), the nozzles stay within RollTvcBudget of their pre-roll trim and the pitch moment the roll makes is taken by the stabilators, with roll rate given up where they cannot. The allocation is re-solved with the flaperons' true pitch moment and the stabs kept on the attached side of their lift peak (past it the down-going stab turns nose-down into pitch-up), pitch and yaw weighted 10x over roll. Accepted only if the modelled pitch error stays within 4 deg/s^2 of what free nozzles would give; otherwise the band widens (x3, x8) or the normal allocation is kept. The nozzles are let go in AoA protection / overshoot past the AoA or asymmetric-load limit and briefly on roll stops and reversals. Bench cost: up to ~20 % roll rate at 15-20 deg AoA, ~35 % in a 1 g roll at 250 km/h (19 deg AoA); none above ~350 km/h at 1 g. RollTvcBudget sets the band: raise it to trade nozzle travel back for roll rate. false = 1.8.39.");
			cRollFightCost = dev.Bind("4. Gains", "RollFightCost", 5f, "1.8.41: while the pedals are in (and 1 s after), the allocator may not split the horizontal surfaces against each other: a group (stabilators, outboard flaperons, inboard flaperons) whose roll opposes the net roll of the others costs this x its roll acceleration squared. With full pedal the rudders reach their stops and the allocator used to buy a few deg/s^2 of yaw from the flaperons' drag by rolling them one way and the stabs the other (roll cancelled) - the large, reversing stab/flaperon deflections on pedal inputs, and all of the yaw once the vertical tails are shot off. Rolls without pedal are allocated exactly as in 1.8.40. Bench, full pedal at 340 km/h: stab-vs-flaperon roll fight mean 24 -> 4 deg/s^2; tails shot off: 79 -> 9. 0 = off (1.8.40).");
			cRollPitchReserve = dev.Bind("4. Gains", "RollPitchReserve", 0.15f, "1.8.41: while you roll, the roll-rate limit and roll-entry acceleration yield when the TVC nozzles have less than this fraction of their nose-down travel (nose-up at negative AoA) left at the live thrust, and come back as the margin returns. Above the RollPitchPriority band (18-26 deg AoA) the stabs lose nose-down authority when they roll (the down-going stab stalls) and the nozzles carry the roll's pitch-up alone: they sat on the -20 deg stop for whole rolls at 22-32 deg AoA, with or without a store. Bench, full roll at 26 deg AoA, AB, 450 km/h: nozzles on the stop 85 % -> 26 % of the roll, bank at 2 s 110 -> 87 deg; with an 800 kg store at the 22 deg cap: 35 % -> 18 %, 87 -> 77 deg. No change below ~18 deg AoA. Higher = more nozzle margin, less roll rate. 0 = off (1.8.40).");
			cRollTvcBudget = dev.Bind("4. Gains", "RollTvcBudget", 1.5f, "1.8.34: 1.5 (1.8.33: 2, 1.8.32: 4). While you roll or yaw, the TVC nozzles stay within this many degrees of where they were before the lateral input (the pitch trim for your own pitch command), and the stabilators take the pitch moment the roll itself makes (mostly the flaperons' lift change). It is an allocator cost, not a clamp: if the stabs cannot supply it (on a stop), the nozzles still go further. The limit opens up to full nozzle travel between 14 and 20 deg AoA (the down-going stab is then near its stall and nose-down from it costs roll rate), between T/W 0.7 and 1.1 (with afterburner the roll trim is only a few degrees and the nozzles stay the fast pitch effector), in AoA protection / run-away and while the pitch-rate command is moving. Cost on the bench: 3-10 % roll rate in 300-350 km/h part-throttle roll spam, about 1 % bank at 1 s in single rolls. 0 = off (1.8.31: the nozzles take the whole roll trim, down to -20 deg at part throttle).");
			cRollTvcBudgetCost = dev.Bind("4. Gains", "RollTvcBudgetCost", 8f, "1.8.34: 8 (1.8.33: 6, 1.8.32: 2). Allocator cost per (degree / 10)^2 of nozzle travel beyond RollTvcBudget. Higher = a harder budget.");
			cRollWash = dev.Bind("4. Gains", "RollNozzleWashout", 0.4f, "Time constant (s) separating 'fast' (nozzles) from 'sustained' (stabs) in RollNozzleHold. Lower = the stabs take over sooner.");
			cSlatStart = dev.Bind("5. Airframe", "SlatAoAStart", 2f, "AoA (deg) at which the slats start coming down. The high-lift device has no aero model beyond the wing area it adds, and the stock AoA term is a step rather than a schedule, so the FCS drives it off AoA directly.");
			cSlatFull = dev.Bind("5. Airframe", "SlatAoAFull", 20f, "AoA (deg) at which the slats are fully down. They then stay down to HighLiftMaxAoA.");
			cHlMaxAoA = dev.Bind("5. Airframe", "HighLiftMaxAoA", 70f, "The slats (high-lift device) stay deployed up to this |AoA|, fading over the 10 deg below it. They used to be tied to the flaperon's trailing-edge-down schedule and so retracted by 30 deg AoA. The device's mechanical TE-down swing is subtracted by the flap drive, so the net flaperon bias is unchanged: only the slats and the wing area they add stay out.");
			cHlHighAoA = dev.Bind("5. Airframe", "HighLiftHighAoAShare", 1f, "How much of the slat deployment is kept at high AoA (0..1). 0 = pre-1.8.0 behaviour (slats retract with the flap schedule).");
			cOuterFlap = dev.Bind("5. Airframe", "OuterFlaperonDroop", 0f, "Revoker 0.1.0: 0 (F-22E/Ifrit 0.6). How far the trailing-edge flaperons droop TE-down with the flap schedule (fraction of the inboard flap angle, stall-capped, roll-biased). 0 = the trailing edges stay at neutral except for roll, and the canards carry a held turn's pitch (with the nozzle drawn to neutral by TvcNeutralHold).");
			cStabRoll = dev.Bind("4. Gains", "StabilatorRollCost", 0.01f, "Ternion 0.1.6: 0.01 (was 0.0005; Tom: control surfaces stuck). At 0.0005 a canard split balanced by an opposite elevator split cost almost nothing, so after a manoeuvre the pair kept 5-20 deg of opposing differential with the stick centred (and ~0.3-0.6 deg of sideslip) for many seconds; bench replay of the 7 Nov 01:45 log: gone within 0.5 s at 0.01. Allocator cost of differential (roll) stabilator use at attached flow. Lower = stabs do more of the roll.");
			cStabRollShare = dev.Bind("4. Gains", "StabilatorRollShare", 0.55f, "Share of the differential-stab roll moment counted as usable roll authority (sets roll rate ceiling / acceleration).");
			cOnset = dev.Bind("4. Gains", "PitchOnsetLimit", 150f, "High-speed pitch pacing: pitch-rate command acceleration <= this / lift slope (g per deg). Lower = gentler at high dynamic pressure.");
			cRollPitchFF = dev.Bind("4. Gains", "RollPitchFeedForward", 1f, "Corrects the pitch target by the pitch moment the roll surfaces are about to make (their pitch side-effect is not in the allocator's model), so the stabilators and nozzles move with a roll instead of chasing it afterwards. Mostly visible at low IAS, where the nozzles are the fast pitch effector and the chase shows as nozzle twitch in rolls. 0 = off (pre-1.7.12 behaviour).");
			cLowQReg = dev.Bind("4. Gains", "LowQAllocatorRelief", 1f, "Scales the allocator's regularisation (neutral preference, move cost, pair costs) with dynamic pressure squared, capped at the high-speed value. The cost's error term grows with q^2 and the regularisation did not, so below ~150 m/s the surfaces were held near neutral and near their last position: weak roll and sluggish reversals at low IAS. 0 = pre-1.8.0 behaviour.");
			cRollStop = dev.Bind("3. Lateral-directional", "RollStopFactor", 1f, "Roll-rate ceiling is also limited to this many seconds of the measured roll acceleration (the roll equivalent of YawStopFactor). Since roll acceleration is moment / inertia, this is what makes the roll-rate limit follow a heavy load, especially one far from the CG. 0 = off.");
			cRollAccUse = dev.Bind("3. Lateral-directional", "RollAccelUse", 0.5f, "Share of the roll authority planned for acceleration at and above ~110 m/s IAS (the rest is reserve for regulation against roll damping).");
			cRollAccUseLow = dev.Bind("3. Lateral-directional", "RollAccelUseLowIAS", 0.8f, "Same at and below 60 m/s IAS, blending to RollAccelUse by 110. Roll damping is small down there and the surfaces are far from saturation, so more of the authority can go into the manoeuvre - this is what makes a low-speed reversal crisp.");
			cFFMax = dev.Bind("4. Gains", "RollPitchFeedForwardMax", 30f, "Ceiling on the roll->pitch feed-forward, in deg/s^2 of pitch acceleration. It is an anticipation rather than a measurement, and at high dynamic pressure the roll surfaces' pitch moment is large enough that an error in it moves real g.");
			cFFStabCost = dev.Bind("4. Gains", "RollPitchFeedForwardStabCost", 1f, "During the feed-forward pass, symmetric (pitch) stabilator deflection costs this much more, so the correction lands on the nozzles instead of showing up as a large mean stabilator deflection under the roll's differential. 1 = no preference.");
			cRollPitchFFAoA = dev.Bind("4. Gains", "RollPitchFeedForwardMaxAoA", 35f, "RollPitchFeedForward fades out over the 15 deg below this |AoA| and is off above it: post-stall the stabilators have no pitch authority to redistribute and the pass would only compete for the surfaces holding the jet in yaw.");
			cCoupFF = dev.Bind("4. Gains", "InertialCouplingFF", 0f, "Feed-forward of the gyroscopic roll/pitch/yaw coupling (w x Iw) between commanded and present rates. 0 = off (default since 1.8.14: evaluated at the commanded rates it arrives as a nose-down step the instant you roll, before the jet has the rates; the INDI loop already measures the real coupling).");
			cRelPitchPrio = dev.Bind("4. Gains", "ReleasedStickPitchPriority", 30f, "Stick released (zero-rate hold): when the pitch rate starts to leave the hold, the allocator weights pitch this many times harder against roll/yaw for the stabilators, so a roll cannot pull them into a nose-down shove. Scales in with the pitch-rate error (1 to 9 deg/s) and out with stick (fully off from 18% stick) and with stabilator stall. 1 = off (pre-1.8.14).");
			cWashSlew = dev.Bind("4. Gains", "RollNozzleWashoutSlew", 5f, "RollNozzleHold hands the nozzles' pitch trim to the stabs during rolls. While a stab is on its stop that handover is blocked and builds up; this caps how fast it is then applied (deg of nozzle per second), so it cannot all land in one frame the moment the stabs come off the stop - which is when they are braking the roll. 0 = uncapped (pre-1.8.15).");
			cMomLead = dev.Bind("4. Gains", "ReleasedStickMomentLead", 0.035f, "1.8.31: 0.035 (was 0.07; with any control delay 0.07 rang for 1-2 s after a pitch stop). Stick released, zero-rate hold established: the INDI pitch loop leads its measured effector moment by this many seconds of its own rate of change, covering the servo/filter lag when the trim moment is ramping (AoA bleeding off after a hard pull). Fades in over 0.2 s; off with stick, while the pitch-rate command is moving, in AoA protection and with stab stall. 0 = off (pre-1.8.16). Above ~0.1 s it starts to wobble.");
			cStabTrim = dev.Bind("4. Gains", "StabilatorTrimCost", 0.0002f, "Revoker 0.1.0: 0.0002 (F-22E/Ifrit 0.004): the canards carry the steady pitch (held turns, rolls) and the nozzle the transients. Allocator cost of symmetric (pitch) stabilator deflection relative to the nozzles. Higher = steady trim / sustained pitch rate carried by TVC, stabs kept for transients and whatever TVC cannot hold.");
			cUnloadDecel = dev.Bind("4. Gains", "UnloadDecel", 20f, "Easing the stick: pitch-rate deceleration (deg/s^2) toward the rate the aircraft will sustain at the new AoA/G. Lower = smoother.");
			cUnloadRise = dev.Bind("4. Gains", "UnloadRiseGain", 1.5f, "Easing the stick: if AoA still rises during the decel, the pitch-rate target drops by this x AoA rate (to zero at most).");
			cGUnder = dev.Bind("4. Gains", "GUndershootCapture", 1f, "Easing out of a hard pull at high dynamic pressure, the pitch rate is bounded so the load is captured at the commanded G instead of continuing past it into negative g - the same kinematic capture the load limiter uses, applied from above the target. Lower = a softer bound (0.5 allows twice the nose-down rate); 0 = off (pre-1.8.10 behaviour). Only in the G-command regime.");
			cKpG = dev.Bind("4. Gains", "GGain", 2.5f, "G command bandwidth (1/s): load error is converted to AoA error through the live lift slope.");
			cKiG = dev.Bind("4. Gains", "GIntegral", 1f, "G command integral gain (1/s^2), removes steady load error.");
			cActuator = dev.Bind("5. Airframe", "ActuatorSpeedScale", 1f, "Servo speed multiplier for every control surface and the TVC nozzles (Aryx's servos run at 70 deg/s; 1.428571 = 100 deg/s).");
			cStabUp = dev.Bind("5. Airframe", "StabilatorRangeTEUp", 35f, "Ternion 0.1.0: 35, the F-22E FCS's stabilator travel (Tom: the Raptor FCS's ranges; stock 30 = pitch 20 + roll 10). Aft elevator (elevator_L/R) travel, trailing edge up (nose up), deg up to 80; 0 = the stock travel. The canards have their own keys (CanardRangeLEDown / LEUp).");
			cCanLEDown = dev.Bind("5. Airframe", "CanardRangeLEDown", 65f, "Su-30 0.1.0: 65 (Tom). Canard (LCanard_struct / RCanard_struct) travel leading edge down = trailing edge up (deg, up to 80). The stock CanardSurface AoA schedule (canard hinge turned 1:1 with AoA, +-30) is switched off while the FCS flies; the FCS owns the canard. 0 = stock. Ternion lineage: Ternion 0.1.0: 60, the Revoker FCS's canard travel (Tom; stock 48 = pitch 45 + roll 3). Canard (canard_L/R) travel, leading edge down = trailing edge up (deg, up to 80; 0 = stock).");
			cCanLEUp = dev.Bind("5. Airframe", "CanardRangeLEUp", 45f, "Su-30 0.1.0: 45 (Tom). Canard travel leading edge up = trailing edge down (deg). Ternion lineage: Ternion 0.1.0: 45, the Revoker FCS's canard travel (Tom; stock 48). Canard travel, leading edge up = trailing edge down (deg, up to 80; 0 = stock).");
			cElevTrim = dev.Bind("4. Gains", "ElevatorTrimCost", 0.004f, "Ternion 0.1.0: allocator cost of the aft elevators' symmetric (pitch) deflection (the canards: StabilatorTrimCost 0.0002). The canards carry the steady pitch and the elevators add to it when the canards cannot; with equal costs the two pairs traded deflection against each other (opposed canard / elevator swings, 35-40 deg/s rms at the AoA limit on the bench).");
			cElevStall = dev.Bind("5. Airframe", "ElevatorStallAoA", 25f, "Su-30 0.1.0: 25 = the F-22E FCS's StabilatorStallAoA: lift-peak local AoA (deg) of the stabs (Left/Right_stab_struct) on the Raptor stab curve (RaptorStabFoil). 0 = the Su-30's own Su-30_elevators foil (peak 1.38 at 25). Ternion lineage: Ternion 0.1.2: 30 (Tom; 0.1.1: 0). Lift-peak local AoA (deg) of the aft elevators (elevator_L/R) on the knee-preserving staggered curve (the flaperons' method: unchanged below the 17 deg knee, peak CL kept). 0 = the game's Multirole1_wing curve (peak 45). Respawn after changing.");
			cRaptorStab = dev.Bind("5. Airframe", "RaptorStabFoil", true, "Su-30 0.1.0: on (Tom: the stabs get the Raptor's lift curve). The stabs (Left/Right_stab_struct) fly the F-22E FCS's stab curve: Aryx's F22E_Wing foil remapped to its lift peak at ElevatorStallAoA (25 deg local) with the peak CL kept (1.10; 3.74 /rad at 0), x ElevatorEffectiveness 1.2 on area, in the game's aero and in the FCS's models. false = the Su-30's own Su-30_elevators foil. Respawn after changing.");
			cCantedTvc = dev.Bind("5. Airframe", "CantedTvc", true, "Su-30 0.1.0: on (Tom: TVC accurate to the real jet). Each AL-31FP nozzle swivels on one axis, +-TvcPitchRange (15) deg, in a plane tilted TvcCantAngle (32) deg outboard from vertical (a V seen from behind; Flight International: 32 deg outward incline from the vertical plane). Both nozzles together: pitch only (12.8 deg of pitch at 15). Opposite: the pitch parts cancel and the side parts add: yaw with roll in the same direction (proverse). No independent yaw. Each nozzle slews at TvcSlewRate. The FCS sets the nozzle transforms directly (Turbojet transpiler); off = the game's stock pitch / roll / yaw nozzle law.");
			cTvcCant = dev.Bind("5. Airframe", "TvcCantAngle", 32f, "Su-30 0.1.0: 32 = the AL-31FP / Su-30MKI nozzle swivel planes' outward tilt from vertical (deg).");
			cMpoTvcAxis = dev.Bind("3. Lateral-directional", "MpoTvcAxisRoll", true, "Su-30 0.1.0: on (Tom: past the AoA limit, MPO rolls about whatever axis the canted TVC gives). In MPO the stick is a velocity-vector roll (no separate roll / yaw); once the AoA is past AoALimitPositive / Negative the roll axis blends (over MpoTvcAxisBlend deg) from the velocity vector to the axis the canted nozzle differential turns the jet about: the direction of the angular acceleration it makes (inertia included), measured every tick. The sideslip hold fades out with it there. false = velocity-vector roll at every AoA.");
			cMpoTvcAxisBlend = dev.Bind("3. Lateral-directional", "MpoTvcAxisBlend", 5f, "Su-30 0.1.0: AoA band (deg) past the AoA limit over which the MPO roll axis blends from the velocity vector to the TVC axis.");
			cCanSym = dev.Bind("5. Airframe", "CanardSymmetricOnly", true, "Su-30 0.1.0: on (Tom: the canards only move together). The canards are a pitch pair only: no differential for roll or yaw.");
			cElevEff = dev.Bind("5. Airframe", "ElevatorEffectiveness", 1.2f, "Su-30 0.1.0: 1.2 = the F-22E FCS's StabilatorEffectiveness (stab wing-area multiplier), part of the Raptor stab curve: 1.10 peak CL x 1.2 = 1.32, close to the stock Su-30_elevators' 1.38. Respawn after changing. Ternion lineage: Ternion 0.1.2: 1.2 (Tom). Lift multiplier of the aft elevators (elevator_L/R wing area). Respawn after changing.");
			cCanEff = dev.Bind("5. Airframe", "CanardEffectiveness", 1.6f, "Su-30 0.2.2: 1.6 (with CgShiftAft 0.10, Tom 9 Nov 23:05). Su-30 0.2.1: 1.5 (with CgShiftAft 0.15, Tom 9 Nov 22:45). Su-30 0.1.14: 1.3 (Tom 9 Nov 19:49: CG back to 0.25 for the ground, canards 1.3 keep 0.1.13's pitch balance; bench static pitch within ~1 deg/s^2 of CG 0.30 at 0-75 deg AoA). Su-30 0.1.3: 1.2 (Tom). Su-30 0.1.0: 1 (stock canard area). Ternion lineage: Ternion 0.1.2: 1.1 (Tom). Lift multiplier of the canards (canard_L/R wing area). Respawn after changing.");
			cGbElevator = dev.Bind("7. Airbrake", "GroundBrakeElevator", 35f, "Ternion 0.1.2: 35 = the elevators' full trailing-edge-up travel in the rollout aero-brake, as the F-22E's stabilators (Tom: the Raptor's ground airbrake). 0 = the elevators stay on the pitch stick (0.1.0-0.1.1).");
			cPitchMove = dev.Bind("4. Gains", "PitchSurfaceMoveCost", 0f, "Ternion 0.1.0: allocator cost on moving the canards and elevators away from their last command (as TvcMoveCost for the nozzles). 0 = off.");
			cPathFloor = dev.Bind("4. Gains", "FlightPathRateFloor", true, "Ternion 0.1.0: the pitch-rate budget never drops below the measured flight-path rate + 3 deg/s (outside MPO). The budget's thrust term is the nozzles' authority, 0 on the FS-3 (no TVC); without the floor full aft could not hold the AoA limit at low speed (the commanded rate fell below the rate the path itself turns at). false = the inherited budget.");
			cElevSettled = dev.Bind("3. Lateral-directional", "ElevatorSettledWindow", false, "Ternion 0.1.0: off. The settled window (StabParkEscape) walks the pitch surfaces to their lift knee with the stick lateral-quiet; on the FS-3 it applies to the canards only. true = the aft elevators too.");
			cRudRange = dev.Bind("5. Airframe", "RudderRange", 35f, "Ternion 0.1.0: 35, the F-22E FCS's rudder travel (Tom; stock 30). Rudder travel each way (deg, up to 45; 0 = stock).");
			cStabDn = dev.Bind("5. Airframe", "StabilatorRangeTEDown", 30f, "Ternion 0.1.0: 30, the F-22E FCS's stabilator travel (stock 30). Aft elevator travel, trailing edge down (nose down), deg up to 80; 0 = the stock travel.");
			cFlapRange = dev.Bind("5. Airframe", "FlaperonRange", 25f, "Ternion 0.1.0: 25, the F-22E FCS's flaperon travel (Tom: the Raptor FCS's ranges). Travel each way of both trailing-edge pairs: flaperon_L/R (inboard, stock 25) and rollervon_L/R (outboard, stock 40) (deg). 0 = stock.");
			cServoSpeed = dev.Bind("5. Airframe", "SurfaceServoSpeed", 100f, "Ifrit 0.3.0: servo speed of every control surface (deg/s), the F-22E FCS's 100. Stock: stabilators 60, ailerons 45, decelerons 70. 0 = the stock speeds x ActuatorSpeedScale. TVC: TvcSlewRate.");
			cSplitRange = dev.Bind("5. Airframe", "SplitRange", 0f, "Ifrit 0.3.0: deceleron split limit (deg), for the airbrake and for split yaw. 0 = the decelerons' own travel (FlaperonRange, 30); the stock limit is 25. Split drag is split x splitDrag x rho x v^2, so 30 is 20 % more than 25.");
			cTvcRange = dev.Bind("5. Airframe", "TvcPitchRange", 15f, "Su-30 0.1.0: 15 = the AL-31FP's +-15 deg (CantedTvc: travel of each nozzle along its canted axis). Ternion lineage: Pitch thrust-vectoring travel each way (deg, game maximum 20). 0 = Aryx's original (10).");
			cTvcSpeed = dev.Bind("5. Airframe", "TvcSlewRate", 65f, "Su-30 0.1.0: 65 (Tom; everything else at 100). Slew rate (deg/s) of each nozzle along its own canted axis (CantedTvc). Ternion lineage: TVC nozzle slew rate (deg/s). 0 = follow ActuatorSpeedScale like the control surfaces (70 x scale).");
			cStabPeakMargin = dev.Bind("5. Airframe", "StabPeakMargin", 90f, "2.0.11: default 90 (off, 2.0.8 behaviour). 2.0.10: below 40 deg body AoA a stab is not commanded more than this many degrees past its lift peak (local AoA, either sign); the limit opens up from 40 to 50 deg AoA (deep-stall use above, e.g. falling leaf, unchanged). Stops the stabs swinging through deep stall in rolls at 36 deg and parking past the peak after a roll (10:00 flight). Bench, full roll at 36 deg with TVC off: stab rate 40 -> 16 deg/s rms, roll rate 27-33 -> 19-25 deg/s; with TVC on the roll rate is within 5 %. 90 = off.");
			cTvcOff = dev.Bind("0.1 Debug - Airframe test", "DisableTVC", defaultValue: false, "2.0.8 test switch: nozzles locked at 0 deg pitch at every throttle, and the FCS flies without them (pitch from the stabs, flaperons and rudders only). With AoALimit Normal the low-thrust caps (NoTvcAoACapPositive / Negative, +-36) then apply (the nozzles never cover pitch). All Ifrits on this machine. Takes effect at once.");
			cAoATest = dev.Bind("0.1 Debug - Airframe test", "AoALimit", AoALimitTest.Normal, "2.0.8 test switch for the AoA limits. Normal = AoALimitPositive / Negative (+65 / -45), capped at NoTvcAoACapPositive / Negative when the nozzles do not cover pitch (low thrust or DisableTVC). Full65 = +65 / -45 and those caps off (the swept authority cap, which stops the FCS pulling past where the stabs can still push the nose down, stays on; LowThrustAoACap = false removes that too). Cap36 = the NoTvcAoACap values in every condition. Takes effect at once.");
			cStabsPitchOnly = dev.Bind("0.1 Debug - Airframe test", "StabsPitchOnly", false, "2.2.5 test switch: the stabilators move symmetrically only (pitch), with no roll or yaw job, so roll and yaw come from the flaperons and rudders alone. For checking what the flaperons can do. The lateral rate limits still count the stabs' authority, so at high AoA the FCS will ask for more roll/yaw than the jet then has (expect slower rolls and more sideslip there). Off for AI. Takes effect at once.");
			cNoTvcCapPos = dev.Bind("0.1 Debug - Airframe test", "NoTvcAoACapPositive", 50f, "Su-30 0.2.4: default 50 (Tom 10 Nov; a file holding 45 moves to 50 once). F-22E 2.2.3: default 45 (2.0.9-2.2.2: 36). Positive AoA cap (deg) while the nozzles do not cover pitch (T/W under LowThrustFixedAoATW, or DisableTVC). Also the positive limit with AoALimit = Cap36. Takes effect at once.");
			cNoTvcCapNeg = dev.Bind("0.1 Debug - Airframe test", "NoTvcAoACapNegative", 50f, "Su-30 0.3.0: default 50 (Tom 10 Nov 16:46: low thrust and no TVC +-50; a file holding 45 moves to 50 once). Su-30 0.2.4: values above 45 now apply (0.2.3 and before clamped them to 45). F-22E 2.2.3: default 45 (2.0.9-2.2.2: 36). Negative AoA cap magnitude (deg) under the same conditions (2.0.8 had none: -45 always). Also the negative limit with AoALimit = Cap36. Takes effect at once.");
			cNoTvcThrCapPos = dev.Bind("0.1 Debug - Airframe test", "NoTvcThrustAoACapPositive", 50f, "Revoker 0.2.5: 50 (canards + elevons; bench, TVC off, full push from the cap on afterburner recovers below 20 deg in 2.1-2.3 s at 180-300 km/h; a fixed 50 at idle did not recover, so NoTvcAoACapPositive stays 45; 55+ did not recover at 220 km/h). F-22E: 2.2.2: with DisableTVC, the positive AoA cap (deg) once the thrust is up: NoTvcAoACapPositive (36) up to T/W 0.8, this value from T/W 1.0 (afterburner), linear between. Bench: with afterburner the stabs alone hold 45 deg at 160-750 km/h, rolls and reversals included (sideslip 6 deg or less); at part or idle throttle, rolling as the speed bleeds through ~200-250 km/h, 45 departed to 48-62 deg in 5 of 20 cases where 36 held. Takes effect at once.");
			cNoTvcThrCapNeg = dev.Bind("0.1 Debug - Airframe test", "NoTvcThrustAoACapNegative", 50f, "Su-30 0.3.0: default 50 (Tom 10 Nov 16:46; a file holding 45 moves to 50 once). Su-30 0.2.4: values above 45 now apply (0.2.3 and before clamped them to 45). F-22E 2.2.2: with DisableTVC, the negative AoA cap magnitude (deg) at every thrust (NoTvcAoACapNegative still applies at low thrust with the nozzles working). Bench: -45 held at 200-450 km/h, idle to afterburner, with a roll, sideslip 5.4 deg or less. Takes effect at once.");
			cAirframeRev = dev.Bind("5. Airframe", "AirframeRevision", 13, "Internal: airframe-default revision (inherited from the Ifrit FCS). Leave it alone.");
			cFuelUse = dev.Bind("5. Airframe", "FuelConsumptionScale", 0.65f, "Su-30 0.1.13: 0.65 (Tom 9 Nov 17:02: 35 % less fuel burn). Compass 0.1.0: 0.75. Fuel burn multiplier at every throttle setting (0.75 = 25 % less fuel used per throttle setting). Buffed PerformanceProfile only.");
			cThrust = dev.Bind("5. Airframe", "ThrustScale", 1.15f, "Su-30 0.2.4: 1.15 (Tom 10 Nov 15:22: 15 % instead of 12; a file holding 1.12 moves to 1.15 once). Su-30 0.1.13: 1.12 (Tom 9 Nov 17:02: 12 % more thrust; dry and afterburner). Compass 0.1.0: 1.08. Engine thrust multiplier (staticThrust; the Compass has no afterburner). Buffed PerformanceProfile only. Fuel flow per power setting is unchanged by this; see FuelConsumptionScale.");
			cDrag = dev.Bind("5. Airframe", "ParasiticDragScale", 0.90f, "Ternion 0.1.0: 0.90 (Tom: 10 % less parasitic drag, as the Revoker). Compass 0.1.0: 1 (stock drag in both profiles). Multiplier on every part's parasitic drag area. Buffed PerformanceProfile only.");
			cDrAoA = dev.Bind("5. Airframe", "AoADragRelief", 0f, "Revoker 0.1.0: 0 (off). Buffed profile only: post-stall AoA drag relief (F-22E 2.2.3 feature for Aryx's airfoils). The Revoker keeps its stock drag polar.");
			cDrTrans = dev.Bind("5. Airframe", "TransonicDragRelief", 0f, "Revoker 0.1.0: 0 (off). Buffed profile only: Mach drag relief (F-22E 1.8.38 feature). The Revoker keeps its stock transonic drag.");
			cStockDrAoA = dev.Bind("5. Airframe", "LowAoADragRelief", 0f, "Revoker 0.1.0: 0 (off). Both profiles: fraction of each part's AoA-dependent drag removed at low AoA (F-22E 2.2.3, a fix for Aryx's airfoil drag). The Revoker keeps its stock drag polar.");
			cStockDrFade0 = dev.Bind("5. Airframe", "LowAoADragReliefFadeStart", 25f, "2.2.3: local AoA (deg) up to which LowAoADragRelief applies in full (both profiles).");
			cLatMargin0 = dev.Bind("3. Lateral-directional", "HighAoALateralMarginStart", 25f, "Ifrit 0.3.1: AoA (deg) where HighAoALateralBrakeMargin (main cfg) starts to blend in.");
			cLatMargin1 = dev.Bind("3. Lateral-directional", "HighAoALateralMarginFull", 30f, "Ifrit 0.3.1: AoA (deg) from which HighAoALateralBrakeMargin applies in full (smoothstep in between).");
			cSplitBudget = dev.Bind("5. Airframe", "DeceleronSplitBudget", true, "Ifrit 0.3.1: the deceleron's deflection and its split share one travel: the game turns the two halves to deflection +- split, so a 30 deg deflection with a 30 deg split put one half at 60. Now |deflection| + split <= the deceleron travel (FlaperonRange); the deflection comes first and the split (airbrake and split yaw) gets what is left. false = 0.3.0 (they add).");
			cDragBump = dev.Bind("5. Airframe", "AoADragBump", 0f, "Revoker 0.1.0: 0 (off). Ifrit 0.3.1 feature: extra drag over a band of body AoA.");
			cDragBump0 = dev.Bind("5. Airframe", "AoADragBumpStart", 18f, "Ifrit 0.3.1: body |AoA| (deg) where the AoADragBump starts rising.");
			cDragBump1 = dev.Bind("5. Airframe", "AoADragBumpPeak", 24f, "Ifrit 0.3.1: body |AoA| (deg) of the full AoADragBump.");
			cDragBump2 = dev.Bind("5. Airframe", "AoADragBumpEnd", 35f, "Ifrit 0.3.1: body |AoA| (deg) by which the AoADragBump is gone.");
			cStockDrFade1 = dev.Bind("5. Airframe", "LowAoADragReliefFadeEnd", 30f, "2.2.3: local AoA (deg) by which the relief has gone over to its post-stall value: none in Stock, AoADragRelief in Buffed.");
			cAsymOn = dev.Bind("Asymmetric load", "AsymLoadAoALimit", defaultValue: true, "Cap the AoA when an asymmetric load (e.g. one heavy store on one wing) would use more of the roll/yaw control authority than AsymLateralReserve leaves free. The cap is the stick's full-travel AoA (the stick is remapped onto it). A load within the margin is simply trimmed out by the control loop (no cap). Not applied in MPO.");
			cAsymRollHead = dev.Bind("Asymmetric load", "AsymRollHeadroom", defaultValue: true, "The roll-rate and roll-acceleration limits count only the roll authority left in each direction after the measured asymmetric trim (asymmetric store, wing damage). The trim is learned from the measured non-control roll moment minus roll damping while the jet is not rolling (under 20 deg/s) and held during rolls; it only ever lowers the limit on the loaded side. With the left wing shot off it stops a full right roll at the AoA cap from saturating the stabs and rudders, building sideslip and departing. No effect on a symmetric jet. false = pre-1.8.17.");
			cAsymAoAWeight = dev.Bind("Asymmetric load", "AsymAoAWeight", 1f, "0.5.2: with an asymmetric load the AoA limit is where AoA^AsymAoAWeight x the roll the jet can still use (the law's roll, held to what the weaker side can stop) is largest. Higher = keep more AoA and accept less roll into the heavy side; lower = give up more AoA for roll. No load: no effect.");
			cAsymStopTime = dev.Bind("Asymmetric load", "AsymStopTime", 0.8f, "0.5.1: with a lateral CG offset (or a damage asymmetry) the flight-path roll rate the stick may ask toward each side is held to what the roll reserve on its braking side (from the coupled check, at the present AoA) can stop in this many seconds, 10 deg/s at least. Lower = slower rolls into the heavy side at high AoA and low speed, and a roll that stops sooner once the stick is centred. 0 = off (0.5.0).");
			cAsymCplResMax = dev.Bind("Asymmetric load", "AsymCoupledReserveMax", 0.8f, "0.5.0: with a lateral CG offset (or a damage asymmetry) the AoA cap comes from the coupled check: at each AoA, the most flight-path roll each way with the load held, pitch held and no sideslip acceleration, every surface and (while vectoring) each nozzle counted. This is the largest share of that roll range the reserve for the stick may claim; the cap sits where the weaker side is down to about this share of the mean of the two sides. Higher = lower cap, more roll left on it.");
			cAsymReserve = dev.Bind("Asymmetric load", "AsymLateralReserve", 0.35f, "Share of the roll/yaw control authority kept free for the pilot's roll and pedal commands (0..1). Used together with AsymRollReserveAccel / AsymYawReserveAccel: whichever asks for more sets the cap. Higher = lower cap, more roll/yaw left.");
			cAsymDead = dev.Bind("Asymmetric load", "AsymDeadbandAccel", 5f, "Asymmetric roll acceleration (deg/s^2) ignored before any cap is considered - the airframe is never perfectly symmetric even clean (off-centre gun and ammunition, engines never exactly matched). The yaw deadband is a quarter of this. A balanced loadout stays well inside it; one 300 kg store is far outside it.");
			cAsymRollAcc = dev.Bind("Asymmetric load", "AsymRollReserveAccel", 80f, "Roll acceleration (deg/s^2) that must stay available to the pilot after the asymmetric load is trimmed out, whatever AsymLateralReserve works out to. Post-stall a share of the remaining authority is almost nothing, so this absolute floor is what sets the cap up high and at low speed. Higher = lower cap.");
			cAsymYawAcc = dev.Bind("Asymmetric load", "AsymYawReserveAccel", 20f, "Same for yaw acceleration (deg/s^2).");
			cAsymFF = dev.Bind("Asymmetric load", "AsymFeedForward", 0f, "DEFAULT 0 (off) since 1.8.17. Hands the CHANGE in the modelled asymmetric moment straight to the allocator, washed out over AsymFeedForwardWashout seconds. Inside the INDI loop an added moment is not a one-off kick: the loop measures the acceleration it causes and settles with a steady roll-rate error of about FF/(I x KRoll) for as long as the term lasts, and while the load keeps changing (pulling, levelling off) it lasts the whole time. With a wing shot off it gave a 20-35 deg/s roll drift in pulls and level-offs and helped depart the jet. The loop already measures the asymmetric moment. 1 = pre-1.8.17.");
			cAsymFFTau = dev.Bind("Asymmetric load", "AsymFeedForwardWashout", 1f, "Time constant (s) over which the feed-forward above hands the load back to the control loop.");
			cAsymFFBeta = dev.Bind("Asymmetric load", "AsymFeedForwardMaxSideslip", 12f, "Su-30 0.1.10: 12 (Tom's setting). AsymFeedForward fades out over the 12 deg of sideslip above this and is off beyond it: the asymmetric moment it is built from is modelled at zero sideslip, so at large sideslip (falling leaf) it is not a measurement of anything.");
			cRevRoll = dev.Bind("3. Lateral-directional", "ReversedFlowRollSense", defaultValue: true, "Flying backwards: the velocity-vector roll maps to body roll through cos(AoA), which flips sign past 90 deg, so the stick would roll the jet the other way from the pilot's seat. This flips the roll command with it, blended smoothly from ReversedFlowSenseStart to ReversedFlowSenseEnd (0.3.4; 0.3.2 flipped at once at 100 deg AoA, released at 80). Pitch and pedal are unaffected - the control loop already handles reversed flow on the surfaces themselves.");
			cAsymFFAoA = dev.Bind("Asymmetric load", "AsymFeedForwardMaxAoA", 40f, "AsymFeedForward fades out over the 15 deg of AoA above this and is off beyond it. Post-stall the lateral authority it would spend is what the manoeuvre needs.");
			cAsymRollTau = dev.Bind("Asymmetric load", "AsymRollTau", 0.5f, "The roll reserve also has to cover the roll rate the law allows at this speed, reached in this many seconds. At high IAS that is what sets the reserve (250 deg/s in 0.5 s = 500 deg/s^2), not AsymRollReserveAccel. 0 = off (fixed reserve only).");
			cAsymRelaxBoost = dev.Bind("Asymmetric load", "AsymCapRelaxBoost", 4f, "The cap relaxes faster the further the demand is below the limit: rate = AsymCapRelaxRate x (1 + this x margin). With the load gone the cap comes back at (1 + this) x the base rate. 0 = fixed rate.");
			cAsymBoost = dev.Bind("Asymmetric load", "AsymAxisPriorityBoost", 0f, "With more than half the lateral authority taken by an asymmetric load, the allocator's roll and yaw weight is raised by up to this factor (1 = up to double) so the surfaces hold the jet straight before they fine-tune pitch. DEFAULT 0 (off) since 1.7.11: it starves the pitch axis, which showed up as the nozzles running nose-down in rolling pulls.");
			cAsymFloor = dev.Bind("Asymmetric load", "AsymMinAoACap", 12f, "The asymmetric-load cap never goes below this positive AoA (deg).");
			cAsymFloorNeg = dev.Bind("Asymmetric load", "AsymMinAoACapNegative", 8f, "The asymmetric-load cap never goes closer to 0 than this on the negative side (deg, magnitude).");
			cAsymTighten = dev.Bind("Asymmetric load", "AsymCapTightenRate", 10f, "How fast the cap comes down when the asymmetry grows (deg/s).");
			cAsymRelax = dev.Bind("Asymmetric load", "AsymCapRelaxRate", 8f, "How fast the cap goes back up when the asymmetry shrinks (store released, fuel, speed) (deg/s).");
			cAsymRec = dev.Bind("Asymmetric load", "AsymCapRecoveryRate", 15f, "Above the cap (it just came down), AoA is brought back to it at up to this rate (deg/s). Not cancelled by full stick.");
			cAsymMinIas = dev.Bind("Asymmetric load", "AsymMinIAS", 35f, "Below this IAS (m/s) the cap is held at its last value instead of being recomputed (no aero authority to judge by).");
			cAsymCapDemand = dev.Bind("Asymmetric load", "AsymCapDemand", 1.08f, "1.8.45: lateral demand (load share of the authority left after the reserve) at which the asymmetric-load AoA cap sits (1.8.44 and before: 1.0). With a light mixed load (6.5 cm lateral CG offset) the demand rises ~0.025 per degree above the cap, so 1.08 lets ~3 deg more AoA; heavier loads, whose demand rises faster with AoA, gain less.");
			cAsymCapSmooth = dev.Bind("Asymmetric load", "AsymCapSmoothing", 1f, "1.8.45: time constant (s) with which the asymmetric-load cap may rise. Each 0.2 s sweep's cap goes through a median of the last three and then this low-pass; a lower cap is followed with 0.3 s. The raw cap moved 1-8 deg every sweep (4.4 Hz) with a 6.5 cm lateral offset at 35-40 deg AoA, which is what made full-aft stick twitch.");
			cDecelLook = dev.Bind("Asymmetric load", "DecelLookahead", 1f, "1.8.45: seconds of IAS decay projected ahead (filtered IAS rate, rapid decelerations only, see DecelStart/DecelFull). The asymmetric-load cap is judged, and the high-AoA roll/yaw rate limits are set, at the dynamic pressure of that predicted IAS, so they come down before the surfaces lose the authority. 0 = off.");
			cDecelStart = dev.Bind("Asymmetric load", "DecelStart", 6f, "1.8.45: IAS deceleration (m/s^2) below which nothing is anticipated. Between this and DecelFull the deceleration is weighted by the square of where it sits, so ordinary turning bleed (and most of a J-turn's tail) predicts almost nothing and a collapsing corkscrew predicts nearly all of it.");
			cDecelFull = dev.Bind("Asymmetric load", "DecelFull", 20f, "1.8.45: IAS deceleration (m/s^2) from which it is anticipated in full.");
			cHiAoABrake = dev.Bind("3. Lateral-directional", "HighAoABrakeFloor", true, "1.8.45: from 25 to 35 deg AoA up, the roll and yaw rate limits may not exceed what the stop-side authority (at the IAS predicted by DecelLookahead) brakes in the stop time of the reference manoeuvre, full pedal + full lateral stick at 65 deg AoA and 280 km/h (RollStopFactor for roll, YawStopFactor x the yaw boost at 280 km/h for yaw). Replaces there the fixed 30 deg/s roll and 20 x boost yaw floors, which outran the authority below ~65 m/s IAS. Costs yaw rate in slow post-stall yaw: bench, 65 deg AoA at 200 km/h 34 -> 23 deg/s, 300 km/h 37 -> 32. false = 1.8.44.");
			cCmdShape = dev.Bind("4. Gains", "CommandShaping", true, "1.9.0: AoA/G command pre-filter. The AoA and G commands are shaped before the loop: a small request (a few degrees or a fraction of a g from where the jet is) builds up gently, a large one runs as before. The shaping speed (1/s) grows with the step from ShapingAoAGainSmall to ShapingAoAGainLarge over ShapingAoASpan (G: ShapingG*), and small steps also ramp their onset over ShapingOnset. The same idea as the self-adjusting pre-filter gain of the T-50 control law (60 % gain below 1 g of error, 100 % above 3 g). false = 1.8.45.");
			cShapeASpan = dev.Bind("4. Gains", "ShapingAoASpan", 15f, "1.9.0: AoA step (deg) from which the AoA command is no longer slowed.");
			cShapeAKSmall = dev.Bind("4. Gains", "ShapingAoAGainSmall", 1.5f, "1.9.0: shaping speed (1/s) of the AoA command for a very small step. Lower = gentler fine tracking.");
			cShapeAKLarge = dev.Bind("4. Gains", "ShapingAoAGainLarge", 10f, "1.9.0: shaping speed (1/s) of the AoA command at and above ShapingAoASpan (fast enough to be transparent).");
			cShapeOnset = dev.Bind("4. Gains", "ShapingOnset", 0.25f, "1.9.0: onset time (s) of a very small AoA/G step (the command's rate builds up over this); fades to 0.03 s at the span.");
			cShapeGSpan = dev.Bind("4. Gains", "ShapingGSpan", 3f, "1.9.0: load step (g) from which the G command is no longer slowed.");
			cShapeGKSmall = dev.Bind("4. Gains", "ShapingGGainSmall", 1.5f, "1.9.0: shaping speed (1/s) of the G command for a very small step (onset and unload).");
			cShapeGKLarge = dev.Bind("4. Gains", "ShapingGGainLarge", 10f, "1.9.0: shaping speed (1/s) of the G command at and above ShapingGSpan.");
			cRateBlend = dev.Bind("2. Envelope", "HighAoARateBlend", true, "1.9.1: AoA command hands over to pitch-rate command past RateBlendAoA. The stick scale is fixed: RateBlendAoA at RateBlendStick (36 deg at 0.5 = 72 deg per full stick); the threshold's stick point moves with the threshold (24 deg -> 0.333). Up to that point the stick is AoA command; past it, still AoA command, on to the AoA limit at full stick (full stick goes through the threshold at the rate budget). Holding the threshold's point holds the threshold AoA. When the jet crosses the threshold with the stick past its point, the stick becomes a pitch-rate command starting from the pitch rate at the crossing, up to the budget at full stick; a full pull or a release switches it to 0..full stick = 0..budget, and a release holds zero rate. Easing below the point stays rate command until the jet is back at the threshold. If the jet comes back to the threshold with the stick still past the point, that stick position is the threshold: easing from it holds the live threshold (soft knee) until the stick meets the threshold's own point on the line. false = AoA command to the limit (1.8.45).");
			cRateBlendStick = dev.Bind("2. Envelope", "RateBlendStick", 0.5f, "1.9.1: stick position of RateBlendAoA; sets the AoA per stick below the threshold (36 / 0.5 = 72 deg per full stick). It stays fixed when the threshold moves.");
			cRateBlendAoA = dev.Bind("2. Envelope", "RateBlendAoA", 36f, "1.9.0: AoA (deg) where the AoA command hands over to pitch-rate command when the wings still lift at least RateBlendLiftG times the weight.");
			cRateBlendAoAMin = dev.Bind("2. Envelope", "RateBlendAoAMin", 25f, "1.9.0: the handover AoA when the wings can lift only the weight (1 g, thrust not counted); in between it is interpolated by the lift-to-weight ratio (dynamic pressure x wing lift / mass). Slower = lower threshold.");
			cRateBlendLiftG = dev.Bind("2. Envelope", "RateBlendLiftG", 2f, "1.9.0: wing lift-to-weight ratio (g, thrust not counted) from which the full RateBlendAoA applies. ~330 km/h clean at combat weight.");
			cRateBlendBand = dev.Bind("2. Envelope", "RateBlendBand", 4f, "1.9.0: AoA band (deg) below the threshold over which rate command and AoA command are blended once the rate command is engaged (pitch rate stays continuous across it).");
			cRateBlendNeg = dev.Bind("2. Envelope", "HighAoARateBlendNegative", true, "1.9.7: the same AoA -> pitch-rate hand-over on the negative side, mirrored: RateBlendAoANegative at -RateBlendStick (-36 deg at -0.5 = 72 deg per full push, the same scale as the positive side), full push = the negative AoA limit, rate command (nose-down) past the threshold, floating hold point. RateBlendStick, RateBlendLiftG and RateBlendBand are shared. false = linear AoA command to the negative limit (1.9.6).");
			cRateBlendAoANeg = dev.Bind("2. Envelope", "RateBlendAoANegative", 36f, "1.9.7: negative hand-over AoA magnitude (deg) while the wings can still push at least RateBlendLiftG times the weight nose-down (negative lift).");
			cRateBlendAoAMinNeg = dev.Bind("2. Envelope", "RateBlendAoAMinNegative", 25f, "1.9.7: negative hand-over AoA magnitude when the wings' negative lift is only the weight; interpolated by the negative lift-to-weight ratio in between.");
			cAuthCap = dev.Bind("2. Envelope", "LowThrustAoACap", true, "1.9.0: the AoA limit is held below the AoA where the nose-down authority you have now runs out: stabilators full nose-down, flaperons at their trim, plus the nozzles at the present thrust (none above the thrust-vectoring cut-off, 450 m/s true airspeed - the high-altitude supersonic case where the nozzles freeze). Bench, 350 km/h: zero thrust 23 deg, ~0.1 T/W 26 deg, 30 % throttle 34 deg, full afterburner no cap; a full pull then full push recovers in every case (1.8.45 departed to 73-90 deg at the first two). The stick is remapped onto the capped envelope like the asymmetric-load cap. false = 1.8.45.");
			cElevDragShift = dev.Bind("5. Airframe", "ElevatorStallDragShift", false, "Ternion 0.1.4: false (Tom: curve C). With ElevatorStallAoA moving the elevators' lift peak to 30 deg, the drag stays on the airfoil's own angle. true = 0.1.2-0.1.3: the drag rise moved in with the peak (CD 0.60 at 30 deg local instead of 0.22), which made the elevator's force normal to the surface 8-25 % higher at 25-60 deg local than stock, the opposite of an earlier stall. Bench: rolls and 50 deg pulls within noise of 0.1.3. Respawn to apply.");
			cElevPlateau = dev.Bind("5. Airframe", "ElevatorPlateauCL", 1f, "Ternion 0.1.4 (dev test): scales the elevators' lift above the 17 deg knee (blends in over 17-22, out over 70-90 deg local). 1 = off. Bench: 1.2 (peak 45) gave 2-4 % more velocity-vector roll at 20-30 deg AoA and a quicker push recovery, slower AoA build from fast entries; 0.8 the reverse. Respawn to apply.");
			cRelTauAoA = dev.Bind("2. Envelope", "HighAoAReleaseTau", 0.3f, "Ternion 0.1.3: when the stick is let go at high AoA (a corkscrew or a pull above the G regime), the pitch rate is taken off over about this time (s) instead of as fast as the canards allow. The canards stopped a 20-30 deg/s pitch rate by going to their 60 deg LE-down stop and then had to slew back 40 deg, so the nose kept going down (Tom: -4 to -5 deg/s; 7 Nov 00:33 log: -3.6 / -5.2 / -4.9). Bench, corkscrew release at 40-50 deg: -2.2 to -3.1 deg/s -> -0.3 to -0.6. 0 = off (0.1.2). Try 0.4-0.5 if a dip is still there.");
			cAuthCapMargin = dev.Bind("2. Envelope", "LowThrustCapMargin", 4f, "Ternion 0.1.3: 4 (was 8). With 8 the authority cap pulled a 55-60 limit down to 44-50 below ~210 km/h and the jet undershot it; with 4 it stays at the limit to ~165 km/h and a full push still brings the AoA under 15 in 1.9-2.6 s on the bench. 1.9.0: nose-down pitch acceleration (deg/s^2) that must be left at the cap. Higher = lower cap.");
			cAuthCapFloor = dev.Bind("2. Envelope", "LowThrustCapFloor", 20f, "1.9.0: the low-thrust AoA cap never goes below this (deg).");
			cLowThrustTW = dev.Bind("2. Envelope", "LowThrustFixedAoATW", 0.3f, "2.0.5: below this thrust-to-weight the nozzles are treated as not covering pitch and the AoA limit is fixed at NoTvcAoACapPositive / Negative (+-36 by default; 2.0.8 and before: +36 only, released 0.03 above). The swept LowThrustAoACap still applies under it.");
			cHybridIndi = dev.Bind("4. Gains", "HybridIndi", 1f, "1.9.0: pitch-loop lag compensation. The incremental (INDI) pitch loop estimates the airframe's own moment from filtered sensors; at high dynamic pressure that moment moves fast with AoA and pitch rate and the filter lag made the pitch rate overshoot (bench, 2 g command at 1300 km/h: 2.8 g peak then 1.8 g, a slow G oscillation). The lag is now predicted from the airframe model (moment slope with AoA and pitch rate x the filter lag). 0 = off (1.8.45).");
			cHybridHiLead = dev.Bind("4. Gains", "HybridIndiHighAoALead", 0.15f, "0.4.9 (from F-22E 2.4.7/2.4.8): disturbance-rate lead, seconds. Above 25-40 deg AoA with the pitch stick released, the pitch loop takes the airframe's own pitch acceleration (what the effectors are not doing) forward by its rate x this, so a steady fall in it as the AoA unwinds after a corkscrew release is met when it arrives instead of ~0.1 s late. Pitch damping is left out. Fades out with T/W 0.4 -> 0.2 (the horizon is the nozzles' response). 0 = off (0.4.8).");
			cAllocNozzleNewton = dev.Bind("4. Gains", "AllocNozzleNewton", true, "0.4.9 (from F-22E 2.4.7/2.4.8): with the pitch stick released and T/W at least 0.25, the allocator's last pass also tries a local Newton step for the nozzles, not only their grid points (every 4 deg). Without it the nozzles could sit between two grid points with a pitch miss while the joint steps were rejected (stabs stalled on their stops, large yaw miss): the post-corkscrew nose dip. false = 0.4.8.");
			cYawRefKmh = dev.Bind("3. Lateral-directional", "YawBrakeRefIAS", 270f, "1.9.2: IAS (km/h) of the reference manoeuvre for the high-AoA yaw braking floor (HighAoABrakeFloor): full pedal + full lateral stick at max AoA. The yaw-rate limit may not exceed what the yaw authority brakes in YawStopFactor x the low-IAS yaw boost at this IAS (1.8.45: 280).");
			cYawMargStart = dev.Bind("3. Lateral-directional", "YawStopTimeStartIAS", 210f, "1.9.3: below this IAS (km/h) the allowed yaw stop time grows smoothly (smoothstep) to YawStopTimeLowIAS x at YawStopTimeFullIAS; flat above and below.");
			cYawMargFull = dev.Bind("3. Lateral-directional", "YawStopTimeFullIAS", 150f, "1.9.3: IAS (km/h) at and below which the full YawStopTimeLowIAS applies.");
			cYawMargLow = dev.Bind("3. Lateral-directional", "YawStopTimeLowIAS", 1.5f, "1.9.3: allowed yaw stop time at low IAS relative to the reference: 1.5 = 50 % more yaw rate for the same yaw authority (slower braking). 1 = flat. (1.9.2's YawBrakeMargin* keys worked the other way and are no longer read.)");
			cYawMidF = dev.Bind("3. Lateral-directional", "YawStopTimeMidFactor", 1f, "Revoker 0.2.4: 1 (no mid-IAS band; the F-22E rudder rule cut the corkscrew entry yaw limit 77 -> 54 deg/s from 330 to 290 km/h). F-22E: 2.0.12: 0.75 (1.9.4-2.0.11: 0.8). high-AoA yaw stop time (so yaw-rate limit) factor between YawStopTimeMidLoIAS and YawStopTimeMidHiIAS: 0.8 = 20 % less yaw rate for the same authority, faster braking. Smoothstep ramps of YawStopTimeMidRamp km/h outside the band. 1 = off.");
			cYawMidLo = dev.Bind("3. Lateral-directional", "YawStopTimeMidLoIAS", 280f, "2.0.12: 280 (1.9.4: 290). lower edge (km/h) of the mid-IAS yaw band (full factor from here up).");
			cYawMidHi = dev.Bind("3. Lateral-directional", "YawStopTimeMidHiIAS", 380f, "2.0.12: 380 (1.9.4: 330). upper edge (km/h) of the mid-IAS yaw band (full factor up to here).");
			cYawMidRamp = dev.Bind("3. Lateral-directional", "YawStopTimeMidRamp", 30f, "1.9.4: width (km/h) of the smooth ramps outside the mid-IAS yaw band (250-280 and 380-410 by default).");
			DevUnlocked = devUnlocked;
			// Revoker 0.2.0: one-time move of the 0.1.0 defaults that changed (AoALimitPositive 65 -> 70, ElevonPitch false ->
			// true). A dev file written by 0.1.0 has no RevokerConfigRevision line; values set by hand to anything else stay.
			bool revOld = false;   // Ternion 0.1.0: no Revoker-era files to migrate
			cRevokerRev = dev.Bind("2. Envelope", "RevokerConfigRevision", 15, "Internal: the Revoker config revision this file was last updated to. Leave it alone.");
			if (revOld)
			{
				cRevokerRev.Value = 0;
			}
			if (cRevokerRev.Value < 1)
			{
				if (Mathf.Abs(cAoAPos.Value - 65f) < 0.001f)
				{
					cAoAPos.Value = 70f;
				}
				if (!cElevonPitch.Value)
				{
					cElevonPitch.Value = true;
				}
				cRevokerRev.Value = 1;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.0 defaults: AoALimitPositive 70, ElevonPitch on (moved from the 0.1.0 defaults once).");
			}
			// Revoker 0.2.2: MaxYawRate 75 -> 90 once (a 0.2.0 / 0.2.1 dev file holds the old default)
			if (cRevokerRev.Value < 2)
			{
				if (Mathf.Abs(cMaxYaw.Value - 75f) < 0.001f)
				{
					cMaxYaw.Value = 90f;
				}
				cRevokerRev.Value = 2;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.2 defaults: MaxYawRate 90 (moved from 75 once).");
			}
			// Revoker 0.2.4: YawStopTimeMidFactor 0.75 -> 1 once
			if (cRevokerRev.Value < 3)
			{
				if (Mathf.Abs(cYawMidF.Value - 0.75f) < 0.001f)
				{
					cYawMidF.Value = 1f;
				}
				cRevokerRev.Value = 3;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.4 defaults: YawStopTimeMidFactor 1 (moved from 0.75 once).");
			}
			// Revoker 0.2.5: NoTvcThrustAoACapPositive 45 -> 50 once
			if (cRevokerRev.Value < 4)
			{
				if (Mathf.Abs(cNoTvcThrCapPos.Value - 45f) < 0.001f)
				{
					cNoTvcThrCapPos.Value = 50f;
				}
				cRevokerRev.Value = 4;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.5 defaults: NoTvcThrustAoACapPositive 50 (moved from 45 once).");
			}
			// Revoker 0.2.7: canard travel 40 -> 45 (StabilatorRangeTEUp/Down 0 -> 45) and AoALimitNegative 50 -> 55, once
			if (cRevokerRev.Value < 5)
			{
				if (cStabUp.Value < 0.001f && cStabDn.Value < 0.001f)
				{
					cStabUp.Value = 45f;
					cStabDn.Value = 45f;
				}
				if (Mathf.Abs(cAoANeg.Value - 50f) < 0.001f)
				{
					cAoANeg.Value = 55f;
				}
				cRevokerRev.Value = 5;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.7 defaults: canard travel 45 (StabilatorRangeTEUp/Down, from 0 = stock 40) and AoALimitNegative 55 (from 50), once.");
			}
			// Revoker 0.2.9: GroundRotationRamp 60 -> 600, once (only if it was exactly 60)
			if (cRevokerRev.Value < 6)
			{
				if (Mathf.Abs(cGroundRotRamp.Value - 60f) < 0.001f)
				{
					cGroundRotRamp.Value = 600f;
				}
				cRevokerRev.Value = 6;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.9 defaults: GroundRotationRamp 600 (from 60), once.");
			}
			// Revoker 0.2.13: PullSideslipMargin 15 -> 9, once (only if it was exactly 15)
			if (cRevokerRev.Value < 7)
			{
				// Revoker FCS 0.3.0: PullSideslipMargin is gone (the reference governor replaced the rolling-pull governor); the revision still advances
				cRevokerRev.Value = 7;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.13 defaults: (PullSideslipMargin retired in 0.3.0).");
			}
			// Revoker 0.2.14: canard travel trailing edge down (leading edge up) 45 -> 80, once (only if it was exactly 45)
			if (cRevokerRev.Value < 8)
			{
				if (Mathf.Abs(cStabDn.Value - 45f) < 0.001f)
				{
					cStabDn.Value = 80f;
				}
				cRevokerRev.Value = 8;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.14 defaults: StabilatorRangeTEDown 80 (from 45), once.");
			}
			// Revoker 0.2.16: 0.2.14 extended the wrong side. Leading edge DOWN (trailing edge up, the + side) gets the 80, leading
			// edge up goes back to 45, once (only from the exact 0.2.14/0.2.15 pair 45 / 80; anything else set by hand is kept)
			if (cRevokerRev.Value < 9)
			{
				if (Mathf.Abs(cStabUp.Value - 45f) < 0.001f && Mathf.Abs(cStabDn.Value - 80f) < 0.001f)
				{
					cStabUp.Value = 80f;
					cStabDn.Value = 45f;
				}
				cRevokerRev.Value = 9;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.16 defaults: canard travel leading edge down 80 / leading edge up 45 (StabilatorRangeTEUp 80, StabilatorRangeTEDown 45; was 45 / 80), once.");
			}
			// Revoker 0.2.17: canard travel leading edge down 80 -> 70, once, leading edge up kept at 45 (from the exact 0.2.16 pair 80 / 45,
			// or 70 / 35 from a withdrawn first 0.2.17 build; anything else set by hand is kept)
			if (cRevokerRev.Value < 11)
			{
				bool from216 = Mathf.Abs(cStabUp.Value - 80f) < 0.001f && Mathf.Abs(cStabDn.Value - 45f) < 0.001f;
				bool fromWithdrawn = Mathf.Abs(cStabUp.Value - 70f) < 0.001f && Mathf.Abs(cStabDn.Value - 35f) < 0.001f;
				if (from216 || fromWithdrawn)
				{
					cStabUp.Value = 70f;
					cStabDn.Value = 45f;
				}
				cRevokerRev.Value = 11;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.17 defaults: canard travel leading edge down 70 / leading edge up 45 (StabilatorRangeTEUp 70, StabilatorRangeTEDown 45), once.");
			}
			// Revoker 0.2.18: GroundBrakeCanard 45 -> 80 (= the full leading-edge-down travel), once (only if it was exactly 45)
			if (cRevokerRev.Value < 12)
			{
				if (Mathf.Abs(cGbCan.Value - 45f) < 0.001f)
				{
					cGbCan.Value = 80f;
				}
				cRevokerRev.Value = 12;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.18 defaults: GroundBrakeCanard 80 (full leading-edge-down travel; from 45), once.");
			}
			// Revoker 0.2.19: canard leading-edge-down travel 70 -> 60, once (only if it was exactly 70)
			if (cRevokerRev.Value < 13)
			{
				if (Mathf.Abs(cStabUp.Value - 70f) < 0.001f)
				{
					cStabUp.Value = 60f;
				}
				cRevokerRev.Value = 13;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.19 defaults: canard travel leading edge down 60 (StabilatorRangeTEUp; from 70), once.");
			}
			// Revoker 0.2.21: inboard elevon reversal AoA 32 -> 37, once (only if it was exactly 32)
			if (cRevokerRev.Value < 14)
			{
				if (Mathf.Abs(cStallInner.Value - 32f) < 0.001f)
				{
					cStallInner.Value = 37f;
				}
				cRevokerRev.Value = 14;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.21 defaults: InnerElevonReversalAoA 37 (from 32), once.");
			}
			// Revoker 0.2.22: knee-preserving stagger; inboard elevon reversal AoA 37 -> 26, once (only if it was exactly 37)
			if (cRevokerRev.Value < 15)
			{
				if (Mathf.Abs(cStallInner.Value - 37f) < 0.001f)
				{
					cStallInner.Value = 26f;
				}
				cRevokerRev.Value = 15;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.22 defaults: InnerElevonReversalAoA 26 (from 37; knee-preserving stagger), once.");
			}
			cCompassRev = cfg.Bind("1. General", "CompassConfigRevision", 2, "Internal: config revision (inherited from the Compass FCS) this file was last updated to. Leave it alone.");
			int cmpPlayerRev = (cmpPlayerExisted && !cmpPlayerHasRev) ? 0 : cCompassRev.Value;
			if (cmpPlayerRev < 1 && cMpoSep.Value == MpoRollMode.SeparateRollYaw)
			{
				cMpoSep.Value = MpoRollMode.VelocityVectorRoll;
				Log.LogInfo("Su-30 FCS 0.1.1 defaults: MpoRollMode VelocityVectorRoll (from SeparateRollYaw), once.");
			}
			if (cCompassRev.Value != 2)
			{
				cCompassRev.Value = 2;
				cfg.Save();
			}
			cCompassDevRev = dev.Bind("2. Envelope", "CompassDevRevision", 4, "Internal: the Compass dev-config revision this file was last updated to. Leave it alone.");
			int cmpDevRev = cmpDevOld ? 0 : (cmpDevHadRev ? cCompassDevRev.Value : 4);
			if (cmpDevRev < 2)
			{
				string moved = "";
				// 0.1.0 -> 0.1.1
				if (cStabUp.Value < 0.001f) { cStabUp.Value = 30f; moved += " StabilatorRangeTEUp 30"; }
				if (cStabDn.Value < 0.001f) { cStabDn.Value = 30f; moved += " StabilatorRangeTEDown 30"; }
				if (Mathf.Abs(cAoAPos.Value - 30f) < 0.001f) { cAoAPos.Value = 35f; moved += " AoALimitPositive 35"; }
				if (cmpDevRev < 1 && cCanWin.Value) { cCanWin.Value = false; moved += " CanardAttachedWindow off"; }
				// 0.1.0 / 0.1.1 -> 0.1.2
				if (Mathf.Abs(cCgShift.Value - 0.27f) < 0.0001f || Mathf.Abs(cCgShift.Value - 0.2f) < 0.0001f) { cCgShift.Value = 0.15f; moved += " CgShiftAft 0.15"; }
				if (Mathf.Abs(cLerxEff.Value - 1f) < 0.0001f || Mathf.Abs(cLerxEff.Value - 2f) < 0.0001f) { cLerxEff.Value = 3f; moved += " LerxEffectiveness 3"; }
				cCompassDevRev.Value = 2;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.2 defaults moved once:" + (moved.Length > 0 ? moved : " (none held the old values)") + ".");
			}
			// Compass 0.1.3: LerxEffectiveness 3 -> 3.4 with the slab tail (StabSlabFoil is a new key and takes its default)
			if (cmpDevRev < 3)
			{
				string moved3 = "";
				if (Mathf.Abs(cLerxEff.Value - 3f) < 0.0001f || Mathf.Abs(cLerxEff.Value - 2f) < 0.0001f || Mathf.Abs(cLerxEff.Value - 1f) < 0.0001f) { cLerxEff.Value = 3.4f; moved3 += " LerxEffectiveness 3.4"; }
				cCompassDevRev.Value = 3;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.3 defaults moved once:" + (moved3.Length > 0 ? moved3 : " (none held the old values)") + ".");
			}
			// Compass 0.1.4: low-AoA roll-rate ceiling (RollRateGainLowIAS 1.2 -> 2.0, RollRateGainHighIAS 1.1 -> 1.8, RollAuthorityUse 1 -> 2)
			if (cmpDevRev < 4)
			{
				string moved4 = "";
				if (Mathf.Abs(cRollLo.Value - 1.2f) < 0.0001f) { cRollLo.Value = 2.0f; moved4 += " RollRateGainLowIAS 2.0"; }
				if (Mathf.Abs(cRollHi.Value - 1.1f) < 0.0001f) { cRollHi.Value = 1.8f; moved4 += " RollRateGainHighIAS 1.8"; }
				if (Mathf.Abs(cRollAuth.Value - 1f) < 0.0001f) { cRollAuth.Value = 2f; moved4 += " RollAuthorityUse 2"; }
				cCompassDevRev.Value = 4;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.4 defaults moved once:" + (moved4.Length > 0 ? moved4 : " (none held the old values)") + ".");
			}
			// Ternion 0.1.2: one-time move of the dev defaults that changed since 0.1.0 / 0.1.1 (a dev file without TernionDevRevision
			// was written by them); a value set by hand to anything else stays
			bool tnDevOld = devUnlocked && !devFresh && !FileHasKey(dev.ConfigFilePath, "TernionDevRevision");
			ConfigEntry<int> cTernDevRev = dev.Bind("2. Envelope", "TernionDevRevision", 4, "Internal: the Ternion dev-config revision this file was last updated to. Leave it alone.");
			if (tnDevOld)
			{
				string movedT = "";
				if (cElevStall.Value < 0.001f) { cElevStall.Value = 30f; movedT += " ElevatorStallAoA 30"; }
				if (Mathf.Abs(cLerxEff.Value - 1f) < 0.0001f) { cLerxEff.Value = 1.13f; movedT += " LerxEffectiveness 1.13"; }
				if (Mathf.Abs(cLatRes.Value - 0.7f) < 0.0001f) { cLatRes.Value = 0f; movedT += " LateralStabReserve 0"; }
				cTernDevRev.Value = 1;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.2 defaults moved once:" + (movedT.Length > 0 ? movedT : " (none held the old values)") + ".");
			}
			// Su-30 0.1.3 (Tom 8 Nov): LERX x1.15 and canards x1.2; a dev file without Su30DevRevision was written by 0.1.0-0.1.2. Only a key
			// still at the old default (1) moves.
			bool suDevOld = devUnlocked && !devFresh && !FileHasKey(dev.ConfigFilePath, "Su30DevRevision");
			ConfigEntry<int> cSuDevRev = dev.Bind("2. Envelope", "Su30DevRevision", 14, "Internal: the Su-30 dev-config revision this file was last updated to. Leave it alone.");
			if (suDevOld)
			{
				string movedS = "";
				if (Mathf.Abs(cLerxEff.Value - 1f) < 0.0001f) { cLerxEff.Value = 1.15f; movedS += " LerxEffectiveness 1.15"; }
				if (Mathf.Abs(cCanEff.Value - 1f) < 0.0001f) { cCanEff.Value = 1.2f; movedS += " CanardEffectiveness 1.2"; }
				cSuDevRev.Value = 1;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.3 defaults moved once:" + (movedS.Length > 0 ? movedS : " (none held the old values)") + ".");
			}
			// Su-30 0.1.4 (Tom 8 Nov): HighAoARollRateCapFloor 0.7 -> 0, only if the file still holds 0.7
			if (devUnlocked && !devFresh && cSuDevRev.Value < 2)
			{
				string movedS4 = "";
				if (Mathf.Abs(cRateCapFloor.Value - 0.7f) < 0.0001f) { cRateCapFloor.Value = 0f; movedS4 += " HighAoARollRateCapFloor 0"; }
				cSuDevRev.Value = 2;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.4 defaults moved once:" + (movedS4.Length > 0 ? movedS4 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 2)
			{
				cSuDevRev.Value = 2;
			}
			// Su-30 0.1.7 (Tom 9 Nov): AoALimitPositive 70 -> 75, only if the file still holds 70
			if (devUnlocked && !devFresh && cSuDevRev.Value < 3)
			{
				string movedS7 = "";
				if (Mathf.Abs(cAoAPos.Value - 70f) < 0.001f) { cAoAPos.Value = 75f; movedS7 += " AoALimitPositive 75"; }
				cSuDevRev.Value = 3;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.7 defaults moved once:" + (movedS7.Length > 0 ? movedS7 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 3)
			{
				cSuDevRev.Value = 3;
			}
			// Su-30 0.1.9 (Tom 9 Nov: the falling leaf rough in all three axes): HighAoACanardPitchFirst 1000 -> 0, only if the file still holds 1000
			if (devUnlocked && !devFresh && cSuDevRev.Value < 4)
			{
				string movedS9 = "";
				if (Mathf.Abs(cCanPitchFirst.Value - 1000f) < 0.001f) { cCanPitchFirst.Value = 0f; movedS9 += " HighAoACanardPitchFirst 0"; }
				cSuDevRev.Value = 4;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.9 defaults moved once:" + (movedS9.Length > 0 ? movedS9 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 4)
			{
				cSuDevRev.Value = 4;
			}
			// Su-30 0.1.10 (Tom 9 Nov 11:18: "migrate settings"): his dev values become the defaults; only a key still holding the old
			// default moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 5)
			{
				string movedS10 = "";
				if (Mathf.Abs(cCgShift.Value) < 0.0001f) { cCgShift.Value = 0.25f; movedS10 += " CgShiftAft 0.25"; }
				if (Mathf.Abs(cAoAPos.Value - 75f) < 0.001f) { cAoAPos.Value = 70f; movedS10 += " AoALimitPositive 70"; }
				if (cSlipTvc.Value) { cSlipTvc.Value = false; movedS10 += " SlipTvcAssist false"; }
				if (Mathf.Abs(cSlipGainHi.Value - 4f) < 0.0001f) { cSlipGainHi.Value = 0f; movedS10 += " SlipGainHighAoA 0"; }
				if (Mathf.Abs(cCorkTol.Value) < 0.0001f) { cCorkTol.Value = 5f; movedS10 += " CorkscrewSlipTol 5"; }
				if (Mathf.Abs(cAsymFFBeta.Value - 6f) < 0.0001f) { cAsymFFBeta.Value = 12f; movedS10 += " AsymFeedForwardMaxSideslip 12"; }
				cSuDevRev.Value = 5;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.10 defaults moved once:" + (movedS10.Length > 0 ? movedS10 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 5)
			{
				cSuDevRev.Value = 5;
			}
			// Su-30 0.1.12 (Tom 9 Nov 16:51: "build this one"): CgShiftAft 0.25 -> 0.30, LerxEffectiveness 1.15 -> 1.5 (AftBodyLift is a new
			// key and takes its default 0.5); only a key still holding the old default moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 6)
			{
				string movedS12 = "";
				if (Mathf.Abs(cCgShift.Value - 0.25f) < 0.0001f) { cCgShift.Value = 0.3f; movedS12 += " CgShiftAft 0.30"; }
				if (Mathf.Abs(cLerxEff.Value - 1.15f) < 0.0001f) { cLerxEff.Value = 1.5f; movedS12 += " LerxEffectiveness 1.5"; }
				cSuDevRev.Value = 6;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.12 defaults moved once:" + (movedS12.Length > 0 ? movedS12 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 6)
			{
				cSuDevRev.Value = 6;
			}
			// Su-30 0.1.13 (Tom 9 Nov 17:02): Buffed thrust x1.08 -> x1.12, fuel burn x0.75 -> x0.65; only a key still holding the old default moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 7)
			{
				string movedS13 = "";
				if (Mathf.Abs(cThrust.Value - 1.08f) < 0.0001f) { cThrust.Value = 1.12f; movedS13 += " ThrustScale 1.12"; }
				if (Mathf.Abs(cFuelUse.Value - 0.75f) < 0.0001f) { cFuelUse.Value = 0.65f; movedS13 += " FuelConsumptionScale 0.65"; }
				cSuDevRev.Value = 7;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.13 defaults moved once:" + (movedS13.Length > 0 ? movedS13 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 7)
			{
				cSuDevRev.Value = 7;
			}
			// Su-30 0.1.14 (Tom 9 Nov 19:49): CgShiftAft 0.30 -> 0.25, CanardEffectiveness 1.2 -> 1.3; only a key still holding the old default moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 8)
			{
				string movedS14 = "";
				if (Mathf.Abs(cCgShift.Value - 0.3f) < 0.0001f) { cCgShift.Value = 0.25f; movedS14 += " CgShiftAft 0.25"; }
				if (Mathf.Abs(cCanEff.Value - 1.2f) < 0.0001f) { cCanEff.Value = 1.3f; movedS14 += " CanardEffectiveness 1.3"; }
				cSuDevRev.Value = 8;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.14 defaults moved once:" + (movedS14.Length > 0 ? movedS14 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 8)
			{
				cSuDevRev.Value = 8;
			}
			// Su-30 0.2.0 (Tom 9 Nov 21:28): AoALimitPositive 75 -> 70, AoALimitNegative 45 -> 65, FallingLeafYawBoost 1 -> 1.2; only a key
			// still holding the old value moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 9)
			{
				string movedS17 = "";
				if (Mathf.Abs(cAoAPos.Value - 75f) < 0.001f) { cAoAPos.Value = 70f; movedS17 += " AoALimitPositive 70"; }
				if (Mathf.Abs(Mathf.Abs(cAoANeg.Value) - 45f) < 0.001f) { cAoANeg.Value = 65f; movedS17 += " AoALimitNegative 65"; }
				if (Mathf.Abs(cLeafBoost.Value - 1f) < 0.0001f) { cLeafBoost.Value = 1.2f; movedS17 += " FallingLeafYawBoost 1.2"; }
				cSuDevRev.Value = 9;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.2.0 defaults moved once:" + (movedS17.Length > 0 ? movedS17 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 9)
			{
				cSuDevRev.Value = 9;
			}
			// Su-30 0.2.1 (Tom 9 Nov 22:45): CgShiftAft 0.25 -> 0.15 (taxi tip-back), CanardEffectiveness 1.3 -> 1.5 (same in-flight balance);
			// only a key still holding the old value moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 10)
			{
				string movedS21 = "";
				if (Mathf.Abs(cCgShift.Value - 0.25f) < 0.0001f) { cCgShift.Value = 0.15f; movedS21 += " CgShiftAft 0.15"; }
				if (Mathf.Abs(cCanEff.Value - 1.3f) < 0.0001f) { cCanEff.Value = 1.5f; movedS21 += " CanardEffectiveness 1.5"; }
				cSuDevRev.Value = 10;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.2.1 defaults moved once:" + (movedS21.Length > 0 ? movedS21 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 10)
			{
				cSuDevRev.Value = 10;
			}
			// Su-30 0.2.2 (Tom 9 Nov 23:05): CgShiftAft -> 0.10, CanardEffectiveness -> 1.6 (more nose-wheel load, same in-flight balance);
			// a key still holding the 0.2.0 or 0.2.1 value moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 11)
			{
				string movedS22 = "";
				if (Mathf.Abs(cCgShift.Value - 0.15f) < 0.0001f || Mathf.Abs(cCgShift.Value - 0.25f) < 0.0001f) { cCgShift.Value = 0.10f; movedS22 += " CgShiftAft 0.10"; }
				if (Mathf.Abs(cCanEff.Value - 1.5f) < 0.0001f || Mathf.Abs(cCanEff.Value - 1.3f) < 0.0001f) { cCanEff.Value = 1.6f; movedS22 += " CanardEffectiveness 1.6"; }
				cSuDevRev.Value = 11;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.2.2 defaults moved once:" + (movedS22.Length > 0 ? movedS22 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 11)
			{
				cSuDevRev.Value = 11;
			}
			// Su-30 0.2.4 (Tom 10 Nov 14:55 / 15:22): NoTvcAoACapPositive 45 -> 50, ThrustScale 1.12 -> 1.15; only a key still holding the old default moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 12)
			{
				string movedS24 = "";
				if (Mathf.Abs(cNoTvcCapPos.Value - 45f) < 0.001f) { cNoTvcCapPos.Value = 50f; movedS24 += " NoTvcAoACapPositive 50"; }
				if (Mathf.Abs(cThrust.Value - 1.12f) < 0.0001f) { cThrust.Value = 1.15f; movedS24 += " ThrustScale 1.15"; }
				cSuDevRev.Value = 12;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.2.4 defaults moved once:" + (movedS24.Length > 0 ? movedS24 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 12)
			{
				cSuDevRev.Value = 12;
			}
			// Su-30 0.3.0 (Tom 10 Nov 16:46): AoALimitPositive 70 -> 75, NoTvcAoACapNegative / NoTvcThrustAoACapNegative 45 -> 50; only a key still
			// holding the old default moves
			if (devUnlocked && !devFresh && cSuDevRev.Value < 13)
			{
				string movedS30 = "";
				if (Mathf.Abs(cAoAPos.Value - 70f) < 0.001f) { cAoAPos.Value = 75f; movedS30 += " AoALimitPositive 75"; }
				if (Mathf.Abs(Mathf.Abs(cNoTvcCapNeg.Value) - 45f) < 0.001f) { cNoTvcCapNeg.Value = 50f; movedS30 += " NoTvcAoACapNegative 50"; }
				if (Mathf.Abs(Mathf.Abs(cNoTvcThrCapNeg.Value) - 45f) < 0.001f) { cNoTvcThrCapNeg.Value = 50f; movedS30 += " NoTvcThrustAoACapNegative 50"; }
				cSuDevRev.Value = 13;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.3.0 defaults moved once:" + (movedS30.Length > 0 ? movedS30 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 13)
			{
				cSuDevRev.Value = 13;
			}
			// Su-30 0.3.2: LowSpeedSlipTvcGain 1 -> 2
			if (devUnlocked && !devFresh && cSuDevRev.Value < 14)
			{
				string movedS32 = "";
				if (Mathf.Abs(cLsSlipTvcGain.Value - 1f) < 0.001f) { cLsSlipTvcGain.Value = 2f; movedS32 += " LowSpeedSlipTvcGain 2"; }
				cSuDevRev.Value = 14;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.3.2 defaults moved once:" + (movedS32.Length > 0 ? movedS32 : " (none held the old values)") + ".");
			}
			else if (cSuDevRev.Value < 14)
			{
				cSuDevRev.Value = 14;
			}
			// Ternion 0.1.3: stock split airbrake and split yaw off, AoA limit 40 -> 50, authority-cap margin 8 -> 4 (only a key still
			// holding the old default moves)
			if (devUnlocked && !devFresh && cTernDevRev.Value < 2)
			{
				string movedT3 = "";
				if (cIfritSplit.Value) { cIfritSplit.Value = false; movedT3 += " SplitFlapYaw false"; }
				if (cGndBrakeSplit.Value) { cGndBrakeSplit.Value = false; movedT3 += " AirbrakeSplitInFlight false"; }
				if (Mathf.Abs(cAoAPos.Value - 40f) < 0.001f) { cAoAPos.Value = 50f; movedT3 += " AoALimitPositive 50"; }
				if (Mathf.Abs(cAuthCapMargin.Value - 8f) < 0.001f) { cAuthCapMargin.Value = 4f; movedT3 += " LowThrustCapMargin 4"; }
				cTernDevRev.Value = 2;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.3 defaults moved once:" + (movedT3.Length > 0 ? movedT3 : " (none held the old values)") + ".");
			}
			else if (cTernDevRev.Value < 2)
			{
				cTernDevRev.Value = 2;
			}
			// Ternion 0.1.5: AoALimitPositive 50 -> 55 (Tom), only if the file still holds 50 (or the older 40)
			if (devUnlocked && !devFresh && cTernDevRev.Value < 3)
			{
				string movedT5 = "";
				if (Mathf.Abs(cAoAPos.Value - 50f) < 0.001f || Mathf.Abs(cAoAPos.Value - 40f) < 0.001f) { cAoAPos.Value = 55f; movedT5 += " AoALimitPositive 55"; }
				cTernDevRev.Value = 3;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.5 defaults moved once:" + (movedT5.Length > 0 ? movedT5 : " (none held the old values)") + ".");
			}
			else if (cTernDevRev.Value < 3)
			{
				cTernDevRev.Value = 3;
			}
			// Ternion 0.1.6: StabilatorRollCost 0.0005 -> 0.01, only if the file still holds 0.0005
			if (devUnlocked && !devFresh && cTernDevRev.Value < 4)
			{
				string movedT6 = "";
				if (Mathf.Abs(cStabRoll.Value - 0.0005f) < 0.00001f) { cStabRoll.Value = 0.01f; movedT6 += " StabilatorRollCost 0.01"; }
				cTernDevRev.Value = 4;
				dev.Save();
				Log.LogInfo("Su-30 FCS 0.1.6 defaults moved once:" + (movedT6.Length > 0 ? movedT6 : " (none held the old values)") + ".");
			}
			else if (cTernDevRev.Value < 4)
			{
				cTernDevRev.Value = 4;
			}
			// Su-30 0.3.4: the revision is bound BEFORE MigrateOldConfig. MigrateOldConfig clears every key of the player file not bound yet and
			// saves; the revision used to be bound after it, so each launch dropped the line, re-created it at the newest default and skipped
			// every "moved once" step: the 0.2.4 / 0.3.0 PushoverSideslip 8 -> 15 and the 0.3.2 LowSpeedSlipTvc on never reached Tom's file
			// (19:37 log: revision 4, PushoverSideslip 8, LowSpeedSlipTvc false, no "moved once" line). It was the only key bound after it.
			ConfigEntry<int> cSuMainRev = cfg.Bind("1. General", "Su30ConfigRevision", 5, "Internal: the Su-30 player-config revision this file was last updated to. Leave it alone.");
			MigrateOldConfig(cfg, dev, devUnlocked && devFresh);
			MigrateGovernorKeys(cfg, devUnlocked ? dev : null);
			// Revoker FCS 0.4.0: RollStopSideslip default 4 -> 8; a player file written by 0.3.0 still holding exactly 4 is moved once, a value set by hand is kept
			if (cGovRev.Value < 1)
			{
				if (Mathf.Abs(cRollStopSlip.Value - 4f) < 0.001f)
				{
					cRollStopSlip.Value = 8f;
				}
				cGovRev.Value = 1;
				cfg.Save();
				Log.LogInfo($"Revoker FCS 0.4.0 defaults: RollStopSideslip {cRollStopSlip.Value} (4 -> 8 unless set by hand).");
			}
			// Revoker FCS 0.7.0: RollStopSideslip default 8 -> 6; a file still holding exactly 8 is moved once, a value set by hand is kept
			if (cGovRev.Value < 2)
			{
				if (Mathf.Abs(cRollStopSlip.Value - 8f) < 0.001f)
				{
					cRollStopSlip.Value = 6f;
				}
				cGovRev.Value = 2;
				cfg.Save();
				Log.LogInfo($"Revoker FCS 0.7.0 defaults: RollStopSideslip {cRollStopSlip.Value} (8 -> 6 unless set by hand).");
			}
			// Revoker FCS 0.8.1: RollPaceRateBlend default 0.4 -> 0.65; a file still holding exactly 0.4 is moved once, a value set by hand is kept
			if (cGovRev.Value < 3)
			{
				if (Mathf.Abs(cRollPaceRateBlend.Value - 0.4f) < 0.0001f)
				{
					cRollPaceRateBlend.Value = 0.65f;
				}
				cGovRev.Value = 3;
				cfg.Save();
				Log.LogInfo($"Revoker FCS 0.8.1 defaults: RollPaceRateBlend {cRollPaceRateBlend.Value} (0.4 -> 0.65 unless set by hand).");
			}
			// Su-30 0.1.9 (Tom 9 Nov): the airbrake independent of the throttle, and RollPaceRateBlend 0.65 -> 0.40, once per player file
			if (suMainOld)
			{
				string movedM = "";
				if (!cThrIndepBrake.Value) { cThrIndepBrake.Value = true; movedM += " ThrottleIndependentAirbrake true"; }
				if (Mathf.Abs(cRollPaceRateBlend.Value - 0.65f) < 0.0001f) { cRollPaceRateBlend.Value = 0.4f; movedM += " RollPaceRateBlend 0.4"; }
				cSuMainRev.Value = 1;
				cfg.Save();
				Log.LogInfo("Su-30 FCS 0.1.9 player defaults moved once:" + (movedM.Length > 0 ? movedM : " (none held the old values)") + ".");
			}
			// Su-30 0.2.4 (Tom 10 Nov 14:55): PushoverSideslip 8 -> 15 (with TVC; PushoverSideslipNoTvc 12 is new), once per player file
			if (cSuMainRev.Value < 2)
			{
				if (Mathf.Abs(cPushSlip.Value - 8f) < 0.001f)
				{
					cPushSlip.Value = 15f;
					Log.LogInfo("Su-30 FCS 0.2.4 player defaults moved once: PushoverSideslip 15 (from 8).");
				}
				cSuMainRev.Value = 2;
				cfg.Save();
			}
			// Su-30 0.3.0: the 15:44-16:16 flights still read PushoverSideslip 8 from the player file with Su30ConfigRevision 2, so the 0.2.4 move
			// did not hold (Tom 16:46: "the push overs with tvc on didnt use all of the allowed sideslip budget"); a file holding 8 moves to 15 again
			if (cSuMainRev.Value < 3)
			{
				if (Mathf.Abs(cPushSlip.Value - 8f) < 0.001f)
				{
					cPushSlip.Value = 15f;
					Log.LogInfo("Su-30 FCS 0.3.0 player defaults moved once: PushoverSideslip 15 (from 8).");
				}
				cSuMainRev.Value = 3;
				cfg.Save();
			}
			// Su-30 0.3.2 turned LowSpeedSlipTvc on here once; 0.3.5 leaves it to the player (default off again)
			if (cSuMainRev.Value < 4)
			{
				cSuMainRev.Value = 4;
				cfg.Save();
			}
			// Su-30 0.3.4: the move that never landed (see the revision's Bind): PushoverSideslip 8 -> 15, once (0.3.5: no longer turns LowSpeedSlipTvc on)
			if (cSuMainRev.Value < 5)
			{
				string movedM5 = "";
				if (Mathf.Abs(cPushSlip.Value - 8f) < 0.001f) { cPushSlip.Value = 15f; movedM5 += " PushoverSideslip 15"; }
				cSuMainRev.Value = 5;
				cfg.Save();
				Log.LogInfo("Su-30 FCS 0.3.4 player defaults moved once:" + (movedM5.Length > 0 ? movedM5 : " (none held the old values)") + ".");
			}
			// Ifrit 0.2.0: 0.1.0 ran the F-22E 2.0.3 airframe migration and wrote 1.2 into the (unused) effectiveness keys; reset
			// them once. The Ifrit airframe is never effectiveness-scaled.
			// Ifrit 0.2.1: the rollout aero-brake is off; the split decelerons are the airbrake on the ground too
			if (cIfritRev.Value < 2)
			{
				if (cIfritRev.Value >= 1)
				{
					cIfritRev.Value = 2;
					dev.Save();
				}
			}
			// Ifrit 0.2.2: the Buffed profile's parasitic drag 0.885 -> 0.95 and transonic relief 0.2 -> 0.1; a file still on the
			// old defaults is moved once (a value set by hand is left alone)
			if (cIfritRev.Value < 3)
			{
				if (Mathf.Abs(cDrag.Value - 0.885f) < 0.0005f)
				{
					cDrag.Value = 0.95f;
				}
				if (Mathf.Abs(cDrTrans.Value - 0.2f) < 0.0005f)
				{
					cDrTrans.Value = 0.1f;
				}
				if (cIfritRev.Value >= 1)
				{
					cIfritRev.Value = 3;
					dev.Save();
				}
			}
			// Ifrit 0.3.0: FlaperonRange stock (0) -> 30 (ailerons and decelerons +-30); the servo speed and split limit are new keys
			if (cIfritRev.Value < 4)
			{
				if (cFlapRange.Value <= 0f)
				{
					cFlapRange.Value = 30f;
				}
				if (Mathf.Abs(cSplitBrakeHold.Value - 0.05f) < 0.0001f)
				{
					cSplitBrakeHold.Value = 0.5f;
				}
				if (cIfritRev.Value >= 1)
				{
					cIfritRev.Value = 4;
					dev.Save();
				}
			}
			// Ifrit 0.3.1: roll TVC cost 5e-5 -> 0.003 (uneven nozzles on pitch release), rolling-pull sideslip margin 9 -> 15 and
			// the high-AoA sideslip gain ramp from 5 deg (was 25): the corkscrew-entry AoA hitch near 20 deg
			if (cIfritRev.Value < 5)
			{
				if (cIfritRev.Value >= 1)
				{
					if (Mathf.Abs(cTvcDiffMu.Value - 5E-05f) < 1E-07f)
					{
						cTvcDiffMu.Value = 0.003f;
					}
					// Revoker FCS 0.3.0: PullSideslipMargin is gone (the reference governor replaced the rolling-pull governor); the revision still advances
					if (Mathf.Abs(cSlipGainA0.Value - 25f) < 0.001f)
					{
						cSlipGainA0.Value = 5f;
					}
					cIfritRev.Value = 5;
					dev.Save();
				}
			}
			// Ifrit 0.3.7: MPO lateral law default VelocityVectorRoll -> SeparateRollYaw; a file still holding the old default
			// is moved once (a SeparateRollYaw / later choice is never touched)
			if (cIfritRev.Value < 6)
			{
				if (cIfritRev.Value >= 1)
				{
					if (cMpoSep.Value == MpoRollMode.VelocityVectorRoll)
					{
						cMpoSep.Value = MpoRollMode.SeparateRollYaw;
						cfg.Save();
					}
					cIfritRev.Value = 6;
					dev.Save();
				}
			}
			// Ifrit 0.4.0: the engine-out AoA cap 30 -> 22 (bench, one engine at AB: at 30 the pull bleeds to 50-85 m/s where the
			// tails, splits and crossing are all spent and the live engine gets cut; 22 holds without the throttle cut from
			// 300 km/h). A file still holding 30 is moved once.
			if (cIfritRev.Value < 7)
			{
				if (cIfritRev.Value >= 1)
				{
					if (Mathf.Abs(cEngOutCap.Value - 30f) < 0.01f)
					{
						cEngOutCap.Value = 22f;
					}
					cIfritRev.Value = 7;
					dev.Save();
				}
			}
			// Ifrit 0.4.1: the buffed variant's fuel burn 0.75 -> 0.65 (35 % less). A file still holding 0.75 is moved once.
			if (cIfritRev.Value < 8)
			{
				if (cIfritRev.Value >= 1)
				{
					if (Mathf.Abs(cFuelUse.Value - 0.75f) < 0.001f)
					{
						cFuelUse.Value = 0.65f;
					}
					cIfritRev.Value = 8;
					dev.Save();
				}
			}
			if (cIfritRev.Value < 1)
			{
				if (cAirframeRev.Value < 3)
				{
					cAirframeRev.Value = 3;
				}
				cIfritRev.Value = 8;
				dev.Save();
			}
			// 2.0.12: the high-AoA mid-IAS yaw band is 280-380 km/h at 0.75 (25 % less yaw braking margin); a file still on
			// 1.9.4's 290-330 / 0.8 is moved once
			if (cAirframeRev.Value < 6)
			{
				if (Mathf.Abs(cYawMidF.Value - 0.8f) < 0.001f)
				{
					cYawMidF.Value = 0.75f;
				}
				if (Mathf.Abs(cYawMidLo.Value - 290f) < 0.01f)
				{
					cYawMidLo.Value = 280f;
				}
				if (Mathf.Abs(cYawMidHi.Value - 330f) < 0.01f)
				{
					cYawMidHi.Value = 380f;
				}
			}
			if (cAirframeRev.Value < 6)
			{
				cAirframeRev.Value = 6;
			}
			// 2.2.23: rolling-pull governor defaults 6 / 0 -> 9 / 0.7; a file still on the old pair is moved once, anything
			// set by hand is kept
			if (cAirframeRev.Value < 7)
			{
				// Revoker FCS 0.3.0: PullSideslipMargin is gone (the reference governor replaced the rolling-pull governor); the revision still advances
				if (Mathf.Abs(cPullRollTrade.Value) < 0.0001f)
				{
					cPullRollTrade.Value = 0.7f;
				}
				Log.LogInfo($"Revoker FCS 0.3.0 (F-22E 2.2.23) defaults: PullRollTrade {cPullRollTrade.Value} (PullSideslipMargin retired).");
				cAirframeRev.Value = 7;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// 2.2.26 (revision 8) moved PullRollTrade 0.7 -> 1; 2.2.27 undoes it
			// 2.2.27: PullRollTrade back to 0.7. A file that 2.2.26 moved to 1 (revision 8 with the value 1) goes back once;
			// a file never touched by 2.2.26 keeps its value
			if (cAirframeRev.Value < 9)
			{
				if (cAirframeRev.Value == 8 && Mathf.Abs(cPullRollTrade.Value - 1f) < 0.001f)
				{
					cPullRollTrade.Value = 0.7f;
				}
				Log.LogInfo($"Revoker FCS 0.3.0 (F-22E 2.2.27) defaults: PullRollTrade {cPullRollTrade.Value}.");
				cAirframeRev.Value = 9;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// 2.2.29: ReversedFlowRollSense on (a file holding false is set to true once)
			if (cAirframeRev.Value < 10)
			{
				cRevRoll.Value = true;
				Log.LogInfo("Revoker FCS 0.3.0 (F-22E 2.2.29) defaults: ReversedFlowRollSense true.");
				cAirframeRev.Value = 10;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.4.1: fuel moved twice as fast, balancing 40 -> 80 kg/s and trim 120 -> 240; files still on the old values are moved once
			if (cAirframeRev.Value < 11)
			{
				if (Mathf.Abs(cFuelRate.Value - 40f) < 0.01f)
				{
					cFuelRate.Value = 80f;
				}
				if (Mathf.Abs(cFuelTrimRate.Value - 120f) < 0.01f)
				{
					cFuelTrimRate.Value = 240f;
				}
				Log.LogInfo("Ifrit 0.4.1 defaults: FuelTransferRate 80, FuelTrimRate 240.");
				cAirframeRev.Value = 11;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.4.2: the damaged-engine cap 30 -> 40; files still on 30 are moved once
			if (cAirframeRev.Value < 12)
			{
				if (Mathf.Abs(cEngDmgCap.Value - 30f) < 0.01f)
				{
					cEngDmgCap.Value = 40f;
				}
				Log.LogInfo("Ifrit 0.4.2 defaults: EngineDamagedAoACap 40.");
				cAirframeRev.Value = 12;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.4.5: throttle response back to the game's own (1.3 -> 1); files still on 1.3 are moved once
			if (cAirframeRev.Value < 13)
			{
				if (Mathf.Abs(cThrResp.Value - 1.3f) < 0.001f)
				{
					cThrResp.Value = 1f;
				}
				Log.LogInfo("Ifrit 0.4.5 defaults: ThrottleResponseScale 1.");
				cAirframeRev.Value = 13;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.3.0: keys this build no longer reads (the F-22E-only settings, keys that moved section) are dropped from
			// both files instead of being carried as orphans
			DropOrphans(cfg, true);
			DropOrphans(dev, devUnlocked);
		}

		// true when the player file exists and was written with HostLockVariant's old default (false)
		// Revoker 0.2.0: whether a config file already has a key line
		private static bool FileHasKey(string path, string key)
		{
			try
			{
				if (string.IsNullOrEmpty(path) || !System.IO.File.Exists(path))
				{
					return false;
				}
				foreach (string l in System.IO.File.ReadAllLines(path))
				{
					if (l.TrimStart().StartsWith(key + " ") || l.TrimStart().StartsWith(key + "="))
					{
						return true;
					}
				}
			}
			catch (System.Exception)
			{
			}
			return false;
		}

		// Su-30 0.1.4: the "# Default value:" line BepInEx wrote above a key (null if none). A player file that still carries an
		// old default there was written by an older build; it is rewritten on the next save, so a move keyed on it happens once.
		private static string FileDefaultLine(string path, string key)
		{
			try
			{
				if (string.IsNullOrEmpty(path) || !System.IO.File.Exists(path))
				{
					return null;
				}
				string[] lines = System.IO.File.ReadAllLines(path);
				for (int i = 0; i < lines.Length; i++)
				{
					string t = lines[i].TrimStart();
					if (t.StartsWith(key + " =") || t.StartsWith(key + "="))
					{
						for (int k = i - 1; k >= 0 && k >= i - 6; k--)
						{
							string l = lines[k].Trim();
							if (l.StartsWith("# Default value:"))
							{
								return l.Substring("# Default value:".Length).Trim();
							}
						}
						return null;
					}
				}
			}
			catch (System.Exception)
			{
			}
			return null;
		}

		private static bool HostLockOldDefault(string path)
		{
			try
			{
				if (string.IsNullOrEmpty(path) || !System.IO.File.Exists(path))
				{
					return false;
				}
				string[] lines = System.IO.File.ReadAllLines(path);
				for (int i = 0; i < lines.Length; i++)
				{
					if (lines[i].TrimStart().StartsWith("HostLockVariant"))
					{
						for (int k = i - 1; k >= 0 && k >= i - 4; k--)
						{
							string l = lines[k].Trim();
							if (l.StartsWith("# Default value:"))
							{
								return l.EndsWith("false", System.StringComparison.OrdinalIgnoreCase);
							}
						}
						return false;
					}
				}
			}
			catch (System.Exception)
			{
			}
			return false;
		}

		public static bool DevUnlocked { get; private set; }

		// Revoker FCS 0.3.0 (F-22E 2.6.0): PullRollTrade moved from the Dev file to the player file (section "6. Lateral governor"); the
		// retired rolling-pull devices' keys (AoASweepYawLimit, AoASweepYawLimitPositive, PullSideslipMargin, PullFFBoostMax,
		// PullGovSmooth) are gone. Once, the Dev file's leftover value of the moved key is carried into the player file; the
		// leftovers are then dropped with the other orphans.
		private static readonly string[] GovMovedKeys = new string[] { "PullRollTrade", "RollStopSideslip", "RollGovernorPitch", "RollPaceRateSlip", "RollPaceRateBlend", "RollStopSlipProbe" };
		// 2.44.1: Dev keys of devices that are gone, dropped from the Dev file once (no value carried): section / key
		private static readonly string[][] RetiredDevKeys = new string[][] { new string[] { "4. Gains", "ReleasedStickStabRateBound" } };
		private static void MigrateGovernorKeys(ConfigFile cfg, ConfigFile dev)
		{
			if (dev == null)
			{
				return;
			}
			try
			{
				var prop = typeof(ConfigFile).GetProperty("OrphanedEntries", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
				if (!(prop?.GetValue(dev) is Dictionary<ConfigDefinition, string> orph) || orph.Count == 0)
				{
					return;
				}
				int n = 0;
				int dropped = 0;
				int retired = 0;
				foreach (KeyValuePair<ConfigDefinition, string> o in new List<KeyValuePair<ConfigDefinition, string>>(orph))
				{
					bool isRetired = false;
					foreach (string[] rk in RetiredDevKeys)
					{
						if (o.Key.Section == rk[0] && o.Key.Key == rk[1]) { isRetired = true; break; }
					}
					if (isRetired)
					{
						orph.Remove(o.Key);
						retired++;
						continue;
					}
					if (o.Key.Section != "3. Lateral-directional")
					{
						continue;
					}
					if (Array.IndexOf(GovMovedKeys, o.Key.Key) >= 0)
					{
						foreach (ConfigDefinition def in cfg.Keys)
						{
							if (def.Section == "6. Lateral governor" && def.Key == o.Key.Key)
							{
								try
								{
									ConfigEntryBase e = cfg[def];
									e.BoxedValue = TomlTypeConverter.ConvertToValue(o.Value, e.SettingType);
									n++;
								}
								catch
								{
								}
							}
						}
					}
					orph.Remove(o.Key);
					dropped++;
				}
				if (retired > 0 && dropped == 0)
				{
					dev.Save();
				}
				if (retired > 0)
				{
					Log.LogInfo($"F-22E FCS: {retired} retired device key(s) dropped from the dev file.");
				}
				if (dropped > 0)
				{
					dev.Save();
					cfg.Save();
					Log.LogInfo($"Revoker FCS 0.3.0: lateral governor keys moved to the player config ({n} values carried over, {dropped} old lateral keys dropped from the dev file).");
				}
			}
			catch (Exception ex)
			{
				Log.LogWarning("Revoker FCS 0.3.0 governor key migration: " + ex.Message);
			}
		}

		private static void DropOrphans(ConfigFile f, bool save)
		{
			try
			{
				var prop = typeof(ConfigFile).GetProperty("OrphanedEntries", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
				if (f == null || !(prop?.GetValue(f) is Dictionary<ConfigDefinition, string> orph) || orph.Count == 0)
				{
					return;
				}
				int n = orph.Count;
				orph.Clear();
				if (save)
				{
					f.Save();
				}
				Log.LogInfo($"Su-30 FCS: dropped {n} setting(s) this build no longer uses from {System.IO.Path.GetFileName(f.ConfigFilePath)}.");
			}
			catch (Exception ex)
			{
				Log.LogWarning("Su-30 FCS: could not tidy " + ((f != null) ? System.IO.Path.GetFileName(f.ConfigFilePath) : "config") + ": " + ex.Message);
			}
		}

		// 1.8.44: the config was split into the player file (this plugin's) and the dev/debug file. Keys left in the player
		// file from older builds are orphans: the ones that moved section inside the player file (AI toggles) and, when the
		// dev file is being created now, the dev settings are carried over by key name once; then the orphans are dropped.
		private static void MigrateOldConfig(ConfigFile cfg, ConfigFile dev, bool toDev)
		{
			try
			{
				var prop = typeof(ConfigFile).GetProperty("OrphanedEntries", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
				if (!(prop?.GetValue(cfg) is Dictionary<ConfigDefinition, string> orph) || orph.Count == 0)
				{
					return;
				}
				var byKey = new Dictionary<string, string>();
				foreach (KeyValuePair<ConfigDefinition, string> o in orph)
				{
					if (o.Key.Section.StartsWith("Countermeasures (all") || !byKey.ContainsKey(o.Key.Key))
					{
						byKey[o.Key.Key] = o.Value;
					}
				}
				int n = 0;
				void Carry(ConfigFile f, bool skipStockCm)
				{
					foreach (ConfigDefinition def in new List<ConfigDefinition>(f.Keys))
					{
						if (skipStockCm && def.Section.StartsWith("3. Countermeasures"))
						{
							continue;
						}
						if (!byKey.TryGetValue(def.Key, out string v))
						{
							continue;
						}
						ConfigEntryBase e = f[def];
						try
						{
							e.BoxedValue = TomlTypeConverter.ConvertToValue(v, e.SettingType);
							n++;
						}
						catch
						{
						}
					}
				}
				Carry(cfg, skipStockCm: true);
				if (toDev)
				{
					Carry(dev, skipStockCm: false);
					dev.Save();
				}
				orph.Clear();
				cfg.Save();
				Log.LogInfo($"Su-30 FCS: old config keys migrated ({n} values carried over{(toDev ? ", dev file created" : "")}, leftovers removed).");
			}
			catch (Exception ex)
			{
				Log.LogWarning("Su-30 FCS config migration: " + ex.Message);
			}
		}

		public static void ApplyPatches(Harmony h)
		{
			MpPolicy.Install(h, (string what, Action a) => TryPatch(h, what, a));
			TryPatch(h, "ControlsFilter.Filter", delegate
			{
				h.Patch(AccessTools.Method(typeof(ControlsFilter), "Filter"), new HarmonyMethod(typeof(Hooks), "FilterPrefix"));
			});
			TryPatch(h, "AutopilotPlane.AutoAim", delegate
			{
				h.Patch(AccessTools.Method(typeof(AutopilotPlane), "AutoAim", new Type[9]
				{
					typeof(GlobalPosition),
					typeof(bool),
					typeof(bool),
					typeof(bool),
					typeof(float),
					typeof(float),
					typeof(bool),
					typeof(float),
					typeof(Vector3)
				}), new HarmonyMethod(typeof(Hooks), "AutoAimPrefix"), new HarmonyMethod(typeof(Hooks), "AutoAimPostfix"));
			});
			TryPatch(h, "ControlSurface.UpdateJobFields", delegate
			{
				h.Patch(AccessTools.Method(typeof(ControlSurface), "UpdateJobFields"), null, new HarmonyMethod(typeof(Hooks), "SurfacePostfix"));
			});
			TryPatch(h, "PowerSupply.ModifyCapacitance", delegate
			{
				h.Patch(AccessTools.Method(typeof(PowerSupply), "ModifyCapacitance"), new HarmonyMethod(typeof(Countermeasures), "ModifyCapacitancePrefix"));
			});
			TryPatch(h, "Aircraft.UseFuel", delegate
			{
				MethodInfo useFuel = AccessTools.Method(typeof(Aircraft), "UseFuel", new Type[1] { typeof(float) });
				h.Patch(useFuel, new HarmonyMethod(typeof(FuelBalance), "UseFuelScalePrefix") { priority = Priority.First });
				h.Patch(useFuel, new HarmonyMethod(typeof(FuelBalance), "UseFuelPrefix") { priority = Priority.Last });
			});
			TryPatch(h, "Aircraft.GetFuelLevel", delegate
			{
				h.Patch(AccessTools.Method(typeof(Aircraft), "GetFuelLevel"), null, new HarmonyMethod(typeof(FuelBalance), "GetFuelLevelPostfix"));
			});
			TryPatch(h, "Aircraft.GetFuelQuantity", delegate
			{
				h.Patch(AccessTools.Method(typeof(Aircraft), "GetFuelQuantity"), null, new HarmonyMethod(typeof(FuelBalance), "GetFuelQuantityPostfix"));
			});
			TryPatch(h, "FuelTank.Refuel", delegate
			{
				h.Patch(AccessTools.Method(typeof(FuelTank), "Refuel"), new HarmonyMethod(typeof(FuelBalance), "RefuelPrefix"));
			});
			TryPatch(h, "Hardpoint.ModifyMass", delegate
			{
				h.Patch(AccessTools.Method(typeof(Hardpoint), "ModifyMass"), null, new HarmonyMethod(typeof(FuelBalance), "HardpointMassPostfix"));
			});
			TryPatch(h, "AeroJobSettings.Schedule", delegate
			{
				h.Patch(AccessTools.Method(typeof(AeroJobSettings), "Schedule"), new HarmonyMethod(typeof(StallStaggerJobs), "SchedulePrefix"));
			});
			TryPatch(h, "HighLiftDevice.FixedUpdate", delegate
			{
				h.Patch(AccessTools.Method(typeof(HighLiftDevice), "FixedUpdate"), new HarmonyMethod(typeof(Hooks), "FlapDevicePrefix"));
			});
			TryPatch(h, "LandingGear.FixedUpdate", delegate
			{
				h.Patch(AccessTools.Method(typeof(LandingGear), "FixedUpdate"), new HarmonyMethod(typeof(Hooks), "GearPrefix"), new HarmonyMethod(typeof(Hooks), "GearPostfix"));
			});
			TryPatch(h, "Turbojet.FixedUpdate", delegate
			{
				// Su-30 0.1.0: the Su-30's nozzles are Turbojet nozzles: the transpiler routes the pitch / roll / yaw nozzle reads to this FCS
				// and the nozzle transform write to the canted-nozzle model (CantedTvc). It runs after the KR-67 Ifrit and FS-12 Revoker FCS
				// transpilers on the same method and chains onto their calls (any other jet keeps their value or the stock one).
				HarmonyMethod tr = new HarmonyMethod(typeof(Hooks), "TurbojetTranspiler");
				tr.priority = Priority.Last;
				tr.after = new string[2] { Hooks.IfritHarmonyId, "tom.revoker_fs12.fcs" };
				h.Patch(AccessTools.Method(typeof(Turbojet), "FixedUpdate"), new HarmonyMethod(typeof(Hooks), "TurbojetPrefix"), null, tr);
				FcsController.CanardHook = Registry.SetCanardScripts;
				FcsController.CanardCountHook = Registry.CanardScriptCount;
			});
			// Su-30 0.1.5 (ThrustLineAtCg): the nozzles' force point at the CG's height
			TryPatch(h, "JetNozzle.Thrust", delegate
			{
				h.Patch(AccessTools.Method(typeof(JetNozzle), "Thrust"), null, null, new HarmonyMethod(typeof(Hooks), "ThrustTranspiler"));
			});
			// Revoker 0.1.0 / Compass 0.1.0: the dedicated airbrake (game Airbrake component, opened by the game at zero throttle)
			// is commanded by the FCS's brake logic instead (F-22E 2.2.6 / 2.2.16 hooks)
			TryPatch(h, "Airbrake.Update", delegate
			{
				h.Patch(AccessTools.Method(typeof(Airbrake), "Update"), new HarmonyMethod(typeof(Hooks), "AirbrakeUpdatePrefix"), new HarmonyMethod(typeof(Hooks), "AirbrakeUpdatePostfix"));
			});
			Log.LogInfo($"Su-30VXN FCS {FcsStandalonePlugin.Version} ({(StockPerformance ? "stock" : "buffed")} performance{(DevUnlocked ? ", dev/debug settings unlocked" : "")}) ready (AoA +{AoAPos}/{AoANeg}, G +{GPos}/{GNeg}, MPO +{MpoGPos}/{MpoGNeg}, " + string.Format("q<= {0}, p<= {1}, r<= {2}; asym-load AoA cap {3}; camber K aileron {4:F2} / fin {5:F2} / flap {6:F2}, tail downwash {7:F2} {8}, CG shift aft {9:F2} m, LERX x{10:F2} ({14}), tail {15}, actuators {11:0} deg/s, thrust x{12}, fuel burn x{13:F2}).", MaxPitchRate, MaxRollRate, MaxYawRate, AsymLimiter ? ("on, reserve " + AsymReserve.ToString("0.00")) : "off", AileronCamberK, FinCamberK, FlapCamberK, StabDownwash, DownwashLiftDriven ? "lift-driven" : "linear", CgShiftAft, LerxEffectiveness, SurfaceServoSpeed, ThrustScale, FuelUseScale, StrakeVortexLift ? "vortex" : "foil", RaptorStabFoil ? ("Raptor stab curve, stall " + ElevatorStallAoA.ToString("0") + " deg") : (StabSlabFoil ? ("slab curve, stall " + SlabStallAoA.ToString("0") + " deg") : "stock foil")));
		}

		private static void TryPatch(Harmony h, string what, Action a)
		{
			try
			{
				a();
				Log.LogInfo("Installed hook: " + what);
			}
			catch (Exception ex)
			{
				Log.LogError("Failed to install hook " + what + ": " + ex);
			}
		}
	}
}
