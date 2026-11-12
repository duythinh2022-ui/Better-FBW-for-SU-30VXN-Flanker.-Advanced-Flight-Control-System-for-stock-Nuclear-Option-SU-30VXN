using System.Collections.Generic;

namespace Su30_VXN.FCS
{
	// Su-30 0.1.0: the Su-30VXN blueprint names its parts with an L/R prefix (LCanard_struct, Left_stab_struct, ...). The FCS laws
	// inherited from the Ternion / Compass / Revoker / F-22E lineage key their surface roles, pairing (_L / _R suffix), camber hosts,
	// stall families and downwash on lower-case suffixed names. Canon maps a Su-30 GameObject name to that scheme; any other name is
	// returned unchanged, so the game objects themselves are never renamed (other mods, the damage tool and replays see the stock names).
	internal static class Names
	{
		private static readonly Dictionary<string, string> map = new Dictionary<string, string>
		{
			{ "LCanard_struct", "canard_L" },
			{ "RCanard_struct", "canard_R" },
			{ "Left_stab_struct", "elevator_L" },
			{ "Right_stab_struct", "elevator_R" },
			{ "LFlap_struct", "flaperon_L" },
			{ "RFlap_struct", "flaperon_R" },
			{ "LOuterFlap_struct", "outerflap_L" },
			{ "ROuterFlap_struct", "outerflap_R" },
			{ "LRudder_struct", "rudder_L" },
			{ "RRudder_struct", "rudder_R" },
			{ "Left_vertical_stabiliser", "vstab_L" },
			{ "Right_vertical_stabiliser", "vstab_R" },
			{ "Left_wing_inner", "wing_inner_L" },
			{ "Right_wing_inner", "wing_inner_R" },
			{ "Left_wing_middle", "wing_mid_L" },
			{ "Right_wing_mid", "wing_mid_R" },
			{ "Left_wing_outer", "wing_outer_L" },
			{ "Right_wing_outer", "wing_outer_R" },
			// Su-30 0.1.3: the LERX. Cheek_L/R (0.7 m2 each, Su-30_body foil, beside the cockpit ahead of the wing root; Cheek_R carries
			// the right canard, as the real jet's canards sit on its LERX) take the lineage's shoulder-LERX role for LerxEffectiveness
			{ "Cheek_L", "shoulder_L" },
			{ "Cheek_R", "shoulder_R" }
		};

		// Su-30 0.1.12: the aft body lift parts (AftBodyLift)
		public static bool IsAftBody(string goName)
		{
			return goName == "L-engine" || goName == "R-engine" || goName == "Su_30_tail" || (goName != null && goName.Contains("intake and engine cover"));
		}

		public static string Canon(string goName)
		{
			if (string.IsNullOrEmpty(goName))
			{
				return goName;
			}
			return map.TryGetValue(goName, out var c) ? c : goName;
		}
	}
}
