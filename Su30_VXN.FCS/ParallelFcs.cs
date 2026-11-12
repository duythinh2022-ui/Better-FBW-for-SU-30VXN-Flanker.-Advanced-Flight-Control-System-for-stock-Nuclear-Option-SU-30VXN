using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace Su30_VXN.FCS
{
	// F-22E 3.1.0: shared pieces of the FCS's multithreading (estimator, table rebuild, allocator reach plan).
	//
	// Rules that keep several FCS mods (F-22E, Ifrit, Revoker, Compass) and several jets well-behaved together:
	// - everything runs on the one process-wide .NET thread pool (no private threads, nothing per mod to oversubscribe);
	// - a jet's fan-out is capped (FcsPool.Helpers: a quarter of the logical cores, 1..3);
	// - the main thread always takes part, and a job no pool thread has started yet is run inline by the main thread when it is
	//   needed (Claimable): a pool kept busy by other jets or other mods can delay the speed-up, never stall a tick;
	// - worker code touches only its own copies and the thread-safe maths (UnityEngine.Mathf / Vector3 / Quaternion: Quaternion's
	//   native AngleAxis / Inverse / EulerToQuaternion are IsThreadSafe in UnityEngine.CoreModule); no Unity objects off the main thread.
	internal static class FcsPool
	{
		public static readonly int Helpers = Mathf.Clamp(Environment.ProcessorCount / 4, 1, 3);

		public static void Queue(WaitCallback cb, object state)
		{
			ThreadPool.QueueUserWorkItem(cb, state);
		}
	}

	// A one-shot job run either by a pool thread or, if none has started it yet, inline by the thread that needs the result.
	internal abstract class Claimable
	{
		private int state;   // 0 idle, 1 queued, 2 running, 3 done

		private readonly ManualResetEventSlim done = new ManualResetEventSlim(true);

		private Exception err;

		private static readonly WaitCallback Cb = s => ((Claimable)s).TryRun();

		public bool Busy => Volatile.Read(ref state) == 1 || Volatile.Read(ref state) == 2;

		public bool Outstanding => Volatile.Read(ref state) == 1 || Volatile.Read(ref state) == 2;

		protected abstract void Work();

		protected void Queue()
		{
			err = null;
			done.Reset();
			Volatile.Write(ref state, 1);
			FcsPool.Queue(Cb, this);
		}

		private void TryRun()
		{
			if (Interlocked.CompareExchange(ref state, 2, 1) != 1)
			{
				return;   // already claimed inline, or abandoned
			}
			RunNow();
		}

		private void RunNow()
		{
			try
			{
				Work();
			}
			catch (Exception e)
			{
				err = e;
			}
			finally
			{
				Volatile.Write(ref state, 3);
				done.Set();
			}
		}

		// Wait for the result (running it here if no pool thread has started it). Returns the microseconds spent waiting on a pool
		// thread (0 if it was finished, or run inline), -1 if there was no job. Re-throws a worker exception.
		protected float JoinCore(out bool ranInline)
		{
			ranInline = false;
			int s = Volatile.Read(ref state);
			if (s == 0)
			{
				return -1f;
			}
			float w = 0f;
			if (s == 1 && Interlocked.CompareExchange(ref state, 2, 1) == 1)
			{
				ranInline = true;
				RunNow();
			}
			else if (!done.IsSet)
			{
				long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
				done.Wait();
				w = (float)((double)(System.Diagnostics.Stopwatch.GetTimestamp() - t0) * 1000000.0 / (double)System.Diagnostics.Stopwatch.Frequency);
			}
			Volatile.Write(ref state, 0);
			if (err != null)
			{
				Exception e = err;
				err = null;
				throw new InvalidOperationException("Su-30 FCS worker: " + e.Message, e);
			}
			return w;
		}

		// Give the result up: a job not started yet will not run; one running finishes on its own (Busy until then).
		protected void Abandon()
		{
			if (Interlocked.CompareExchange(ref state, 3, 1) == 1)
			{
				done.Set();
			}
			if (Volatile.Read(ref state) == 3)
			{
				Volatile.Write(ref state, 0);
			}
		}

		public bool Idle => Volatile.Read(ref state) == 0 || Volatile.Read(ref state) == 3;
	}

	// Copies of the effectors and an allocator for a worker: same concrete types and fields, own tables, partners re-pointed.
	internal sealed class EffectorMirror
	{
		public readonly List<Effector> effs = new List<Effector>();

		private Effector[] src = new Effector[0];

		public readonly Allocator alloc = new Allocator();

		public void Sync(List<Effector> s, Allocator a)
		{
			int n = s.Count;
			bool same = src.Length == n;
			for (int i = 0; same && i < n; i++)
			{
				if (!ReferenceEquals(src[i], s[i]))
				{
					same = false;
				}
			}
			if (!same)
			{
				src = s.ToArray();
				effs.Clear();
				for (int i = 0; i < n; i++)
				{
					effs.Add(s[i].ProbeClone());
				}
			}
			for (int i = 0; i < n; i++)
			{
				Effector e = s[i];
				Effector c = effs[i];
				c.ProbeSyncFrom(e);
				Effector p = e.partner;
				if (p != null)
				{
					int k = Array.IndexOf(src, p);
					c.partner = (k >= 0) ? effs[k] : p;   // a partner outside the list is not in the allocator's active set either way
				}
				else
				{
					c.partner = null;
				}
			}
			alloc.CopyConfigFrom(a);
		}
	}

	// The table rebuild's fork-join: each surface's EvalCompute (pure maths on what EvalGather copied on the main thread) on up to
	// FcsPool.Helpers pool threads plus the caller. One Run object per rebuild, so a pool thread that starts late can only ever
	// find its own run's items (all claimed) and leave.
	internal static class TableFork
	{
		internal sealed class Run
		{
			public SurfaceEffector[] items;
			public int count;
			public int next;
			public int done;
			public EvalContext ctx;
			public readonly ManualResetEventSlim ev = new ManualResetEventSlim(false);
			public Exception err;
		}

		private static readonly WaitCallback Cb = s => Help((Run)s);

		public static Run Start(List<SurfaceEffector> list, EvalContext ctx)
		{
			Run r = new Run
			{
				items = list.ToArray(),
				count = list.Count,
				ctx = ctx
			};
			int h = Math.Min(r.count - 1, FcsPool.Helpers);
			for (int i = 0; i < h; i++)
			{
				FcsPool.Queue(Cb, r);
			}
			return r;
		}

		private static void Help(Run r)
		{
			while (true)
			{
				int i = Interlocked.Increment(ref r.next) - 1;
				if (i >= r.count)
				{
					return;
				}
				try
				{
					EvalContext c = r.ctx;
					r.items[i].EvalCompute(ref c);
				}
				catch (Exception e)
				{
					r.err = e;
				}
				if (Interlocked.Increment(ref r.done) == r.count)
				{
					r.ev.Set();
				}
			}
		}

		// the caller works through what is left, then waits for items a pool thread is still on
		public static void Finish(Run r)
		{
			if (r.count > 0)
			{
				Help(r);
				if (Volatile.Read(ref r.done) < r.count)
				{
					r.ev.Wait();
				}
			}
			if (r.err != null)
			{
				throw new InvalidOperationException("Su-30 FCS table rebuild (worker): " + r.err.Message, r.err);
			}
		}
	}

	// F-22E 3.1.2: the same fork-join for any set of independent items (the asymmetric-load sweep's AoA points)
	internal interface IIndexWork
	{
		void Do(int i);
	}

	internal static class IndexFork
	{
		internal sealed class Run
		{
			public IIndexWork work;
			public int count;
			public int next;
			public int done;
			public readonly ManualResetEventSlim ev = new ManualResetEventSlim(false);
			public Exception err;
		}

		private static readonly WaitCallback Cb = s => Help((Run)s);

		public static Run Start(IIndexWork work, int count)
		{
			Run r = new Run
			{
				work = work,
				count = count
			};
			int h = Math.Min(count - 1, FcsPool.Helpers);
			for (int i = 0; i < h; i++)
			{
				FcsPool.Queue(Cb, r);
			}
			return r;
		}

		private static void Help(Run r)
		{
			while (true)
			{
				int i = Interlocked.Increment(ref r.next) - 1;
				if (i >= r.count)
				{
					return;
				}
				try
				{
					r.work.Do(i);
				}
				catch (Exception e)
				{
					r.err = e;
				}
				if (Interlocked.Increment(ref r.done) == r.count)
				{
					r.ev.Set();
				}
			}
		}

		public static void Finish(Run r)
		{
			if (r.count > 0)
			{
				Help(r);
				if (Volatile.Read(ref r.done) < r.count)
				{
					r.ev.Wait();
				}
			}
			if (r.err != null)
			{
				throw new InvalidOperationException("Su-30 FCS parallel points (worker): " + r.err.Message, r.err);
			}
		}
	}
}
