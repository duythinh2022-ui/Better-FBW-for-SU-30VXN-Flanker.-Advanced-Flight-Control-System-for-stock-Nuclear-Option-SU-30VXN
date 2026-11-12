using System;
using System.Collections.Generic;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// F-22E 3.0.4 (EstimatorThreaded): the capacity estimator's allocator probes run on a worker thread.
	//
	// Why the result is the same: every consumer of the estimator (PitchLaw ~7067, LateralLaw ~8621-9619) runs at Tick ~4737,
	// BEFORE the estimator's call at ~5336, so the laws already read the PREVIOUS tick's estimate. The job is kicked where the
	// estimator used to run (tick N) and collected at the very top of tick N+1, before anything reads it: the laws see exactly the
	// numbers they saw before.
	//
	// What the worker owns: the estimator's probes call Allocator.Solve and write effector commands (restored afterwards). Run against
	// the live effectors that would race tick N+1's own allocation, so the worker solves on its own copies (EffectorMirror: one clone
	// per effector, same concrete type and fields, own tables, partners re-pointed; its own Allocator with the main one's
	// configuration), refreshed by value at the kick. The estimator object itself is written only by whichever thread runs Step,
	// never by two at once.
	//
	// 3.1.0: on the shared Claimable - if no pool thread has started the job when tick N+1 needs it, the main thread runs it there
	// (the 3.0.4 cost for that tick, never a stall); the mirror is shared code with the allocator's reach-plan job.
	//
	// The one side effect of Step on the MAIN allocator in the synchronous path was its reachMoves counter (read by the telemetry's
	// reach column); that column adds the mirror allocator's reachMoves back in (FcsController.FinishRow), so the CSV is unchanged.
	internal sealed class EstimatorJob : Claimable
	{
		private readonly CapacityEstimator est;

		private readonly EffectorMirror mirror = new EffectorMirror();

		private readonly float[,] wIa = new float[3, 3];

		// Step's arguments, by value
		private Vector3 aWdot, aTarget, aUnmet;
		private float aDt, aAlpha, aRudW;

		private bool pending;

		// telemetry / timer: the last join's wait (us), and totals
		public float lastWaitUs;
		public bool lastThreaded;
		public int joins, waits, inline;
		public double waitUsSum, waitUsMax;

		public EstimatorJob(CapacityEstimator est)
		{
			this.est = est;
		}

		// the probe re-solves the worker's allocator made (the telemetry's reach column adds them to the main allocator's)
		public int WorkerReachMoves => mirror.alloc.reachMoves;

		// Where FcsController called capEst.Step: same arguments.
		public void Kick(bool threaded, List<Effector> effs, Allocator alloc, Vector3 wdot, float[,] IaInv, Vector3 target, float dt, Vector3 unmet, float alphaDeg, float rudW)
		{
			Join();
			lastThreaded = threaded && CapacityEstimator.CapEstOn;
			if (!lastThreaded)
			{
				// the 3.0.3 call, on the live effectors and allocator (its probe solves are counted as the estimator's time, not the allocator's)
				bool ts = alloc.timeSolves;
				alloc.timeSolves = false;
				try
				{
					est.Step(effs, alloc, wdot, IaInv, target, dt, unmet, alphaDeg, rudW);
				}
				finally
				{
					alloc.timeSolves = ts;
				}
				return;
			}
			mirror.Sync(effs, alloc);
			for (int r = 0; r < 3; r++)
			{
				for (int q = 0; q < 3; q++)
				{
					wIa[r, q] = IaInv[r, q];
				}
			}
			aWdot = wdot;
			aTarget = target;
			aUnmet = unmet;
			aDt = dt;
			aAlpha = alphaDeg;
			aRudW = rudW;
			pending = true;
			lastWaitUs = 0f;
			Queue();
		}

		protected override void Work()
		{
			est.Step(mirror.effs, mirror.alloc, aWdot, wIa, aTarget, aDt, aUnmet, aAlpha, aRudW);
		}

		// Collect the job (no-op if none is out). Returns after the estimator's fields hold the job's result.
		public void Join()
		{
			if (!pending)
			{
				return;   // lastWaitUs keeps the last job's wait (read by the CSV row of that job's tick)
			}
			pending = false;
			float w = JoinCore(out bool ranInline);
			if (w < 0f)
			{
				w = 0f;
			}
			joins++;
			if (ranInline)
			{
				inline++;
			}
			else if (w > 0f)
			{
				waits++;
			}
			lastWaitUs = w;
			waitUsSum += w;
			if (w > waitUsMax)
			{
				waitUsMax = w;
			}
		}

		public bool Pending => pending;

		// bench: the worker's copies, for the clone-completeness check
		internal List<Effector> WorkerEffs => mirror.effs;

		internal Allocator WorkerAlloc => mirror.alloc;

		internal void SyncForCheck(List<Effector> effs, Allocator alloc)
		{
			mirror.Sync(effs, alloc);
		}
	}
}
