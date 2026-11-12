using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using BepInEx;

namespace Su30_VXN.FCS
{
	// F-22E 3.0.4: the rows are formatted and written on a background thread. The main thread only boxes the values
	// (FcsController.CaptureRow) and queues the finished row; the text is the same, written in the same order. The file is flushed
	// about once a second, as before.
	internal sealed class Telemetry
	{
		private StreamWriter w;

		private readonly Queue<TelemetryRow> q = new Queue<TelemetryRow>();

		private readonly object qLock = new object();

		private readonly AutoResetEvent signal = new AutoResetEvent(false);

		private Thread thread;

		private volatile bool stop;

		public Telemetry(FcsController c)
		{
			try
			{
				string text = Path.Combine(Paths.BepInExRootPath, "Su30_FCS_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");
				w = new StreamWriter(text, append: false);
				c.WriteHeader(w);
				Plugin.Log.LogInfo("Su-30 FCS telemetry: " + text);
				thread = new Thread(Run);
				thread.IsBackground = true;
				thread.Name = "Su30 FCS telemetry";
				thread.Start();
			}
			catch (Exception ex)
			{
				Plugin.Log.LogError("Telemetry disabled: " + ex.Message);
				w = null;
			}
		}

		public void Enqueue(TelemetryRow r)
		{
			if (w == null || stop)
			{
				return;
			}
			lock (qLock)
			{
				q.Enqueue(r);
			}
			signal.Set();
		}

		private void Run()
		{
			Stopwatch flushSw = Stopwatch.StartNew();
			try
			{
				while (true)
				{
					signal.WaitOne(250);
					Drain();
					if (flushSw.ElapsedMilliseconds > 1000)
					{
						w.Flush();
						flushSw.Reset();
						flushSw.Start();
					}
					if (stop)
					{
						Drain();
						return;
					}
				}
			}
			catch (Exception ex)
			{
				try
				{
					Plugin.Log.LogError("Su-30 FCS telemetry writer stopped: " + ex.Message);
				}
				catch
				{
				}
			}
		}

		private void Drain()
		{
			while (true)
			{
				TelemetryRow r;
				lock (qLock)
				{
					if (q.Count == 0)
					{
						return;
					}
					r = q.Dequeue();
				}
				r.WriteTo(w);
			}
		}

		public void Close()
		{
			stop = true;
			signal.Set();
			try
			{
				if (thread != null && !thread.Join(2000))
				{
					Plugin.Log.LogWarning("Su-30 FCS telemetry: writer did not finish in 2 s");
				}
			}
			catch
			{
			}
			try
			{
				w?.Flush();
				w?.Dispose();
			}
			catch
			{
			}
			w = null;
		}
	}
}
