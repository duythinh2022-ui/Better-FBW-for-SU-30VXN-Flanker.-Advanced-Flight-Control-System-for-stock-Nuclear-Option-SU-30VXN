using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Su30_VXN.FCS
{
	// F-22E 3.0.4: one CSV row as its parts - each a format string and its boxed values, captured on the main thread in 3.0.3's
	// WriteRow order. Slots are parts filled later (FcsController.FinishRow). WriteTo formats and writes them in order on whichever
	// thread calls it: string.Format(InvariantCulture, fmt, args) per part, then a line end - the text 3.0.3 wrote.
	internal sealed class TelemetryRow
	{
		private readonly List<string> fmts = new List<string>(48);

		private readonly List<object[]> args = new List<object[]>(48);

		// slots (FinishRow)
		public int ceA, ceB, ceC, ceD, ceE, ceF, ceG, reachSlot, timeSlot, timeSlot2, timeSlot3;

		public int reachMain;

		// Su-30 0.2.0: the values captured with the reach count's part (its format string has the count as its last argument)
		public object[] reachArgs;

		public bool threaded;

		public void Add(string fmt, params object[] a)
		{
			fmts.Add(fmt);
			args.Add(a);
		}

		public int Slot()
		{
			fmts.Add(null);
			args.Add(null);
			return fmts.Count - 1;
		}

		public void Fill(int slot, string fmt, params object[] a)
		{
			fmts[slot] = fmt;
			args[slot] = a;
		}

		public void WriteTo(TextWriter w)
		{
			CultureInfo ci = CultureInfo.InvariantCulture;
			for (int i = 0; i < fmts.Count; i++)
			{
				if (fmts[i] != null)
				{
					w.Write(string.Format(ci, fmts[i], args[i]));
				}
			}
			w.WriteLine();
		}
	}
}
