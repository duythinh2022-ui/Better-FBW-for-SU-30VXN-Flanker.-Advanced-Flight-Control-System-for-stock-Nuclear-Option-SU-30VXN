using BepInEx;

namespace Su30_VXN.FCS.Dev
{
	// Dev/debug unlock for the Su-30VXN FCS. When it is loaded, the FCS binds every setting other than the player ones to
	// BepInEx/config/Su30_VXN.FCS.Dev.cfg and shows them in the ConfigurationManager (F1). It does nothing else.
	[BepInPlugin("Su30_VXN.FCS.Dev", "Su-30VXN FCS - Dev/Debug settings", "0.3.10")]
	public class FcsDevPlugin : BaseUnityPlugin
	{
		private void Awake()
		{
			base.Logger.LogInfo("Su-30 FCS dev/debug settings unlocked.");
		}
	}
}
