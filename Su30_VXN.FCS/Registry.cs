using System;
using System.Collections.Generic;
using System.Reflection;
using HarmonyLib;
using UnityEngine;

namespace Su30_VXN.FCS
{
	internal static class Registry
	{
		// Su-30 0.2.0 (Su30CgReference): kyleisred's Su-30VXN 1.0.1 against 1.0.0, dry part masses (kg), read from the two SolarSu30.dll
		// bundles. The total is unchanged (18 213 kg); 600 kg moved forward.
		internal static readonly Dictionary<string, float> Su30Delta101 = new Dictionary<string, float>
		{
			{ "nose_cone_thingy", 200f },
			{ "Front_cover", 200f },
			{ "Mid_cover", 100f },
			{ "Su-30", -200f },
			{ "Right_intake and engine cover", -100f },
			{ "Left_intake and engine cover", -100f },
			{ "Su-30L", -50f },
			{ "Su-30R", -50f }
		};

		private static readonly Dictionary<int, bool> isF22 = new Dictionary<int, bool>();

		private static readonly HashSet<int> setupDone = new HashSet<int>();

		private static readonly Dictionary<int, FcsController> controllers = new Dictionary<int, FcsController>();

		private static readonly Dictionary<int, SurfaceEffector> surfaceMap = new Dictionary<int, SurfaceEffector>();

		private static readonly Dictionary<int, FcsController> surfaceOwner = new Dictionary<int, FcsController>();

		private static readonly Dictionary<int, FcsController> anySurfaceOwner = new Dictionary<int, FcsController>();

		private static readonly Dictionary<ControlInputs, FcsController> byInputs = new Dictionary<ControlInputs, FcsController>();

		private static readonly HashSet<int> failed = new HashSet<int>();

		private static readonly Dictionary<int, int> attachRetry = new Dictionary<int, int>();

		private static readonly HashSet<int> tvcFixed = new HashSet<int>();

		private static readonly HashSet<int> servoScaled = new HashSet<int>();

		private static readonly HashSet<int> areaScaled = new HashSet<int>();

		private static readonly Dictionary<int, float> setupRetry = new Dictionary<int, float>();

		private static readonly HashSet<int> dragScaled = new HashSet<int>();

		private static readonly HashSet<int> cgShifted = new HashSet<int>();

		// Su-30 0.2.0: jets whose 1.0.1 part masses were set back to 1.0.0 (Su30CgReference)
		private static readonly HashSet<int> massRef = new HashSet<int>();

		private static readonly Dictionary<int, FcsController> deviceOwner = new Dictionary<int, FcsController>();

		private static readonly FieldInfo fAfterburners = AccessTools.Field(typeof(JetNozzle), "afterburners");

		public static FcsController Find(Aircraft a)
		{
			if (!((UnityEngine.Object)(object)a != null) || !controllers.TryGetValue(((UnityEngine.Object)(object)a).GetInstanceID(), out var value))
			{
				return null;
			}
			return value;
		}

		public static FcsController FindAnySurface(ControlSurface cs)
		{
			if (!(cs != null) || !anySurfaceOwner.TryGetValue(cs.GetInstanceID(), out var value))
			{
				return null;
			}
			return value;
		}

		public static bool IsF22(Aircraft a)
		{
			if ((UnityEngine.Object)(object)a == null)
			{
				return false;
			}
			int instanceID = ((UnityEngine.Object)(object)a).GetInstanceID();
			if (isF22.TryGetValue(instanceID, out var value))
			{
				return value;
			}
			// Compass 0.1.0: the T/A-30 Compass's prefab is "trainer" (AircraftParameters TrainerParameters, aircraftName "T/A-30 Compass");
			// exact name or a clone of it, so VTOLTrainer1 / TailHook_Trainer never match
			string goName = ((Component)(object)a).gameObject.name;
			// Ternion 0.1.0: the FS-3 Ternion is a Blueprinter blueprint (FS-3.Ternion_1.0.0.nobp); its prefab is "P_Trisurface1"
			// (AircraftParameters aircraftName "FS-3 Ternion"). Exact name, a clone, or by the aircraft name.
			// Su-30 0.1.0: kyleisred's Su-30VXN Gandiva (Blueprinter, SolarSu30.dll): prefab "Su-30", AircraftParameters Su30Parameters,
			// aircraftName "Su-30VXN Gandiva". Matched by the aircraft name (other Su-30 mods with a different name are left alone).
			value = false;
			if (!value)
			{
				try
				{
					AircraftParameters aircraftParameters = a.GetAircraftParameters();
					value = aircraftParameters != null && aircraftParameters.aircraftName != null && (aircraftParameters.aircraftName.Contains("Su-30VXN") || aircraftParameters.aircraftName.Contains("Gandiva"));
				}
				catch
				{
					value = false;
				}
			}
			isF22[instanceID] = value;
			return value;
		}

		private static void ScaleAfterburners(JetNozzle n, float k)
		{
			if (n == null || fAfterburners == null || !(fAfterburners.GetValue(n) is Array array))
			{
				return;
			}
			foreach (object item in array)
			{
				if (item != null)
				{
					Traverse traverse = Traverse.Create(item).Field("thrust");
					traverse.SetValue(traverse.GetValue<float>() * k);
				}
			}
		}

		public static bool TryDevice(Component d, out FcsController c)
		{
			return deviceOwner.TryGetValue(d.GetInstanceID(), out c);
		}

		public static void EnsureSetup(Aircraft a)
		{
			if ((UnityEngine.Object)(object)a == null)
			{
				return;
			}
			Countermeasures.Ensure(a);
			int instanceID = ((UnityEngine.Object)(object)a).GetInstanceID();
			if (setupDone.Contains(instanceID))
			{
				return;
			}
			// 0.4.0 multiplayer: the airframe and engine changes are part of the FCS package; a client the host does not
			// allow keeps the stock jet (checked again every 0.5 s in case the policy arrives later)
			if (!MpPolicy.FcsAllowed)
			{
				return;
			}
			float time = Time.time;
			if (setupRetry.TryGetValue(instanceID, out var value) && time < value)
			{
				return;
			}
			setupRetry[instanceID] = time + 0.5f;
			try
			{
				int num = 0;
				int num2 = 0;
				int num3 = 0;
				int num4 = 0;
				// T/A-30 Compass: stock airframe plus the Compass 0.1.0 airframe changes - thrust / fuel profile, LERX (wingroot1)
				// multiplier, CG shift aft (relaxed stability), camber coupling (CamberForce) and the tail downwash (DownwashForce).
				// No thrust vectoring on this jet.
				foreach (Turbojet item in Acc.Collect<Turbojet>(a))
				{
					num++;
					if (!tvcFixed.Add(item.GetInstanceID()))
					{
						continue;
					}
					Vector3 v = Acc.TF_vec(item);
					if (Plugin.TvcRange > 0f)
					{
						v.x = Plugin.TvcRange;
					}
					if (!Plugin.YawTvc)
					{
						v.y = 0f;
					}
					Acc.TF_vec(item) = v;
					// Ifrit 0.4.1: throttle response (every variant): the engine's spool rate and the afterburner's light-up speed
					if (Mathf.Abs(Plugin.ThrottleResponseScale - 1f) > 0.0001f)
					{
						Acc.TF_spoolRate(item) *= Plugin.ThrottleResponseScale;
						JetNozzle[] arrS = Acc.TF_nozzles(item);
						if (arrS != null)
						{
							for (int si = 0; si < arrS.Length; si++)
							{
								Acc.JN_scaleAbSmoothing(arrS[si], Plugin.ThrottleResponseScale);
							}
						}
					}
					if (Mathf.Abs(Plugin.ThrustScale - 1f) > 0.0001f)
					{
						Acc.TF_staticThrust(item) *= Plugin.ThrustScale;
						JetNozzle[] array = Acc.TF_nozzles(item);
						if (array != null)
						{
							for (int i = 0; i < array.Length; i++)
							{
								ScaleAfterburners(array[i], Plugin.ThrustScale);
							}
						}
					}
				}
				foreach (UnitPart item2 in a.partLookup)
				{
					if (item2 is AeroPart aeroPart && Mathf.Abs(Plugin.DragScale - 1f) > 0.0001f && dragScaled.Add(aeroPart.GetInstanceID()))
					{
						Acc.AP_dragArea(aeroPart) *= Plugin.DragScale;
						num4++;
					}
				}
				// Ternion 0.1.2: elevator / canard effectiveness and the shoulder LERX (shoulder_L/R) multiplier that keeps the aerodynamic
				// centre where it was. The shoulders carry a high-lift device whose area the FCS's flap system writes every tick, so
				// its retracted / deployed areas are scaled too (the device's own increment kept).
				int aftBodyScaled = 0;
				foreach (UnitPart itemT in a.partLookup)
				{
					if (!(itemT is AeroPart apT))
					{
						continue;
					}
					string nT = Names.Canon(itemT.gameObject.name);
					float kT = nT.StartsWith("elevator_") ? Plugin.ElevatorEffectiveness : (nT.StartsWith("canard_") ? Plugin.CanardEffectiveness : (nT.StartsWith("shoulder_") ? Plugin.LerxEffectiveness : 1f));
					// Su-30 0.1.12: aft body lift (intake / engine covers, engine bodies, tail cone)
					if (Names.IsAftBody(itemT.gameObject.name))
					{
						kT = Plugin.AftBodyLift;
						if (Mathf.Abs(kT - 1f) >= 0.0001f && !nT.Contains("visible") && !areaScaled.Contains(apT.GetInstanceID()))
						{
							aftBodyScaled++;
						}
					}
					if (Mathf.Abs(kT - 1f) < 0.0001f || nT.Contains("visible") || !areaScaled.Add(apT.GetInstanceID()))
					{
						continue;
					}
					float a0T = Acc.AP_wingArea(apT);
					Acc.AP_wingArea(apT) = a0T * kT;
					foreach (HighLiftDevice hT in Acc.Collect<HighLiftDevice>(a))
					{
						if (hT != null && Acc.HL_part(hT) == apT)
						{
							float rT = Acc.HL_areaRet(hT);
							Acc.HL_areaDep(hT) += (kT - 1f) * rT;
							Acc.HL_areaRet(hT) = rT * kT;
						}
					}
				}
				if (aftBodyScaled > 0)
				{
					Plugin.Log.LogInfo($"Su-30 FCS aft body lift x{Plugin.AftBodyLift:F2} on {aftBodyScaled} part(s) (intake / engine covers, engines, tail cone).");
				}
				// Compass 0.1.0: LERX (wingroot1_L/R) lift multiplier
				foreach (UnitPart itemL in a.partLookup)
				{
					if (itemL is AeroPart apL && itemL.gameObject.name.StartsWith("wingroot1_") && Mathf.Abs(Plugin.LerxEffectiveness - 1f) > 0.0001f && areaScaled.Add(apL.GetInstanceID()))
					{
						Acc.AP_wingArea(apL) *= Plugin.LerxEffectiveness;
					}
				}
				// Compass 0.1.0 (F-22E 2.0.x CG shift): relaxed static stability by moving mass from the forward cockpit section to the
				// tail. Both parts' rigidbodies get the new mass; the fuel balancing's clean-jet reference reads the part masses, so it
				// holds the shifted CG instead of undoing it.
				// Su-30 0.2.0 (Su30CgReference, Tom 9 Nov 22:14: "the guy changed the CG as well, so make the fcs mod don't get affected by the
				// cg change"): kyleisred's Su-30 1.0.1 redistributes 600 kg of the 18.2 t dry mass forward (nose cone +200, Front_cover +200,
				// Mid_cover +100; root body -200, intakes -100 each, Su-30L / R -50 each), about 0.19 m of CG forward of 1.0.0, and more pitch
				// and yaw inertia. On 1.0.1 those eight parts get their 1.0.0 masses back (Su30Delta101 taken off each, through the game's own
				// UnitPart.ModifyMass, which also carries the change to the part's rigidbody), so the FCS flies the 1.0.0 mass layout: same
				// CG, same inertia. CgShiftAft below then works from 1.0.0's layout as before.
				// The part masses in game carry their tanks' fuel and their stores, so the version is told by a part that holds neither (the
				// nose cone: 300 kg on 1.0.0, 500 on 1.0.1). Once per jet (massRef).
				float cgRefFwd = 0f;
				if (Plugin.Su30CgReference && !massRef.Contains(instanceID))
				{
					massRef.Add(instanceID);
					Transform rootR = ((Component)(object)a).transform;
					float totR = 0f;
					bool is101 = false;
					foreach (UnitPart itemR in a.partLookup)
					{
						if (itemR == null)
						{
							continue;
						}
						totR += itemR.mass;
						if (itemR.gameObject.name == "nose_cone_thingy" && Mathf.Abs(itemR.mass - 500f) < 1f)
						{
							is101 = true;
						}
					}
					if (is101 && totR > 1000f)
					{
						float momR = 0f;
						int nR = 0;
						float dmTot = 0f;
						foreach (UnitPart itemR in a.partLookup)
						{
							if (itemR != null && Su30Delta101.TryGetValue(itemR.gameObject.name, out float dm) && itemR.mass - dm > 1f)
							{
								momR += dm * Vector3.Dot(itemR.transform.position - rootR.position, rootR.forward);
								itemR.ModifyMass(0f - dm);
								dmTot += dm;
								nR++;
							}
						}
						Plugin.Log.LogInfo($"Su-30 FCS mass reference: Su-30 1.0.1 airframe; {nR} of 8 re-massed parts set back to their 1.0.0 masses (CG {momR / totR:F3} m aft, net {0f - dmTot:+0;-0;0} kg), so the FCS flies the 1.0.0 mass layout");
					}
				}
				if ((Plugin.CgShiftAft + cgRefFwd) > 0.0001f && !cgShifted.Contains(instanceID))
				{
					UnitPart fore = null;
					UnitPart back = null;
					UnitPart back2 = null;
					float tot = 0f;
					foreach (UnitPart itemC in a.partLookup)
					{
						if (itemC == null)
						{
							continue;
						}
						tot += itemC.mass;
						string nC = itemC.gameObject.name;
						// Su-30 0.1.2: the Su-30VXN's forward fuselage (Front_cover, 2 t at +5.3 m) to its two engine cans (750 kg each at
						// -8.6 m); its own tail cone is 120 kg, too light to take the mass
						if (nC == "cockpit_F" || nC == "Front_cover")
						{
							fore = itemC;
						}
						else if (nC == "tail" || nC == "L-engine")
						{
							back = itemC;
						}
						else if (nC == "R-engine")
						{
							back2 = itemC;
						}
					}
					if (fore != null && back != null && tot > 1000f)
					{
						Transform rootT = ((Component)(object)a).transform;
						Vector3 backPos = (back2 != null) ? (0.5f * (back.transform.position + back2.transform.position)) : back.transform.position;
						float dz = Vector3.Dot(fore.transform.position - backPos, rootT.forward);
						float mv = Mathf.Min((Plugin.CgShiftAft + cgRefFwd) * tot / Mathf.Max(1f, dz), 0.6f * fore.mass);
						fore.mass -= mv;
						if (back2 != null)
						{
							back.mass += 0.5f * mv;
							back2.mass += 0.5f * mv;
							if (back2.rb != null && back2.rb != a.rb)
							{
								back2.rb.mass = back2.mass;
							}
						}
						else
						{
							back.mass += mv;
						}
						if (fore.rb != null && fore.rb != a.rb)
						{
							fore.rb.mass = fore.mass;
						}
						if (back.rb != null && back.rb != a.rb)
						{
							back.rb.mass = back.mass;
						}
						cgShifted.Add(instanceID);
						Plugin.Log.LogInfo($"Su-30 FCS CG shift: {mv:F0} kg moved {fore.gameObject.name} -> {back.gameObject.name} ({dz:F2} m) = {mv * dz / tot:F3} m aft of {tot:F0} kg");
					}
				}
				foreach (ControlSurface item5 in Acc.Collect<ControlSurface>(a))
				{
					num2++;
					// Ifrit 0.3.0: SurfaceServoSpeed sets every control surface's servo to one speed (stock: stabs 60, ailerons 45,
					// decelerons 70 deg/s); 0 = the old ActuatorSpeedScale multiplier on the stock speeds
					if (Plugin.SurfaceServoSpeed > 0f)
					{
						Acc.CS_servo(item5) = Plugin.SurfaceServoSpeed;
					}
					else if (Mathf.Abs(Plugin.ActuatorScale - 1f) > 0.0001f && servoScaled.Add(item5.GetInstanceID()))
					{
						Acc.CS_servo(item5) *= Plugin.ActuatorScale;
					}
					// Ifrit 0.3.0: deceleron split limit (game: the job copies maxSplit once when it is created; see Hooks)
					// Ternion 0.1.3: only when the FCS uses the splits at all; otherwise every surface keeps its stock split limit
					if (Acc.CS_maxSplit(item5) > 0f && Plugin.SplitRange > 0f && (Plugin.IfritSplitYaw || Plugin.AirbrakeSplit))
					{
						Acc.CS_maxSplit(item5) = Plugin.SplitRange;
					}
				}
				bool flag = num2 >= 5 && num >= 1;
				if (flag)
				{
					setupDone.Add(instanceID);
					setupRetry.Remove(instanceID);
					try
					{
						DragRelief.Attach(a);
					}
					catch (Exception exD)
					{
						Plugin.Log.LogWarning("Su-30 FCS drag relief: " + exD.Message);
					}
					try
					{
						int nch = StallStaggerJobs.ZeroChanneling(a);
						Plugin.Log.LogInfo($"Su-30 FCS: {nch} stab part(s) with the stock airflow channeling set to 0 (Raptor downwash model, k {Plugin.StabDownwash:F2}).");
					}
					catch (Exception exCh)
					{
						Plugin.Log.LogWarning("Su-30 FCS stab channeling: " + exCh.Message);
					}
					try
					{
						int ns = StallStaggerJobs.Register(a);
						if (ns > 0)
						{
							Plugin.Log.LogInfo($"Su-30 FCS aero charts on {((UnityEngine.Object)(object)a).name}: {ns} parts get their own curves (strake vortex lift {(Plugin.StrakeVortexLift ? "on" : "off")}, LERX area x{Plugin.LerxEffectiveness:F2}).");
						}
					}
					catch (Exception exS)
					{
						Plugin.Log.LogWarning("Su-30 FCS staggered elevon stall: " + exS.Message);
					}
					try
					{
						int ncb = CamberForce.Attach(a);
						Plugin.Log.LogInfo($"Su-30 FCS camber coupling: {ncb} surfaces paired with their wing section (flap K {Plugin.FlapCamberK:F2}, aileron K {Plugin.AileronCamberK:F2}).");
					}
					catch (Exception exC)
					{
						Plugin.Log.LogWarning("Su-30 FCS camber coupling: " + exC.Message);
					}
					try
					{
						// Compass 0.1.0: the tail downwash, lift-driven from the wing's own airfoil (wing1_L's chart)
						AircraftParameters prmW = a.GetAircraftParameters();
						foreach (UnitPart itemW in a.partLookup)
						{
							if (itemW is AeroPart apW && itemW.gameObject.name == "wing1_L")
							{
								int iw = Acc.AP_airfoil(apW);
								if (prmW != null && prmW.airfoils != null && iw >= 0 && iw < prmW.airfoils.Length)
								{
									Downwash.SetWing(new AirfoilTable(prmW.airfoils[iw]));
								}
								break;
							}
						}
						int nd = DownwashForce.Attach(a);
						Plugin.Log.LogInfo($"Su-30 FCS tail downwash on {((UnityEngine.Object)(object)a).name}: {nd} tail parts, k {Plugin.StabDownwash:F2} ({(Plugin.DownwashLiftDriven && Downwash.WingTable != null ? "lift-driven" : "linear")}; eps {Downwash.Eps(10f):F1} deg at 10 deg AoA, {Downwash.Eps(20f):F1} at 20, {Downwash.Eps(40f):F1} at 40).");
					}
					catch (Exception exDw)
					{
						Plugin.Log.LogWarning("Su-30 FCS tail downwash: " + exDw.Message);
					}
					try
					{
						int nvh = VisualHinge.Attach(a);
						if (nvh >= 0)
						{
							Plugin.Log.LogInfo($"Su-30 FCS: {nvh} trailing-edge surfaces drawn about their modelled hinge lines (render only).");
						}
					}
					catch (Exception exV)
					{
						Plugin.Log.LogWarning("Su-30 FCS hinge visuals: " + exV.Message);
					}
				}
				Plugin.Log.LogInfo(string.Format("Su-30 FCS airframe setup on {0}: {1} engine(s) (pitch + yaw TVC {8}, thrust x{2}), {3} surfaces (servo x{4}), {5} parts drag x{6}{7}.", ((UnityEngine.Object)(object)a).name, num, Plugin.ThrustScale, num2, Plugin.ActuatorScale, num4, Plugin.DragScale, flag ? "" : " - incomplete, will retry", Plugin.YawTvc ? "kept" : "pitch only"));
			}
			catch (Exception ex)
			{
				Plugin.Log.LogError("Su-30 FCS airframe setup failed: " + ex);
			}
		}

		private static float nextPrune;

		private static readonly Dictionary<int, int> hookState = new Dictionary<int, int>();

		public static void NoteHook(Aircraft a, bool ai)
		{
			try
			{
				int num = (a.LocalSim ? (ai ? (Plugin.AIFlightControl ? 1 : 2) : 0) : 3);
				int instanceID = ((UnityEngine.Object)(object)a).GetInstanceID();
				if (hookState.TryGetValue(instanceID, out var value) && value == num)
				{
					return;
				}
				hookState[instanceID] = num;
				string text = num switch
				{
					0 => "human pilot on this machine -> full FCS (AoA/G command)", 
					1 => "AI pilot on this machine -> FCS with AI pitch-rate command", 
					2 => "AI pilot on this machine -> stock fly-by-wire (AIFlightControl = false)", 
					_ => "simulated on another machine -> left to that machine", 
				};
				Plugin.Log.LogInfo($"Su-30 FCS: jet #{instanceID} reached the flight-control hook: {text}.");
			}
			catch
			{
			}
		}

		private static readonly FieldInfo fFbw = AccessTools.Field(typeof(ControlsFilter), "flyByWire");

		private static readonly FieldInfo fFbwG = ((fFbw != null) ? AccessTools.Field(fFbw.FieldType, "gLimitPositive") : null);

		private static readonly FieldInfo fFbwCorner = ((fFbw != null) ? AccessTools.Field(fFbw.FieldType, "cornerSpeed") : null);

		public static void ReadStockRateGain(ControlsFilter f, FcsController c)
		{
			try
			{
				object obj = ((f != null && fFbw != null) ? fFbw.GetValue(f) : null);
				if (obj != null)
				{
					if (fFbwG != null)
					{
						c.aiGLimit = Mathf.Clamp((float)fFbwG.GetValue(obj), 3f, 15f);
					}
					if (fFbwCorner != null)
					{
						c.aiCornerSpeed = Mathf.Clamp((float)fFbwCorner.GetValue(obj), 50f, 400f);
					}
				}
				Plugin.Log.LogInfo($"Su-30 FCS flying an AI Ifrit: pitch-rate command, stock stick gain ({c.aiGLimit:0.#} g, corner {c.aiCornerSpeed:0} m/s), FCS AoA/G limits.");
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("Su-30 FCS: could not read the stock fly-by-wire gain for an AI jet, using 9.5 g / 140 m/s: " + ex.Message);
			}
		}

		private static void Prune()
		{
			float time = Time.time;
			if (time < nextPrune && time >= nextPrune - 10f)
			{
				return;
			}
			nextPrune = time + 5f;
			List<int> list = null;
			foreach (KeyValuePair<int, FcsController> controller in controllers)
			{
				if ((UnityEngine.Object)(object)controller.Value.ac == null)
				{
					(list ?? (list = new List<int>())).Add(controller.Key);
				}
			}
			if (list == null)
			{
				return;
			}
			foreach (int item in list)
			{
				FcsController c = controllers[item];
				controllers.Remove(item);
				try
				{
					c.Dispose();
				}
				catch
				{
				}
				foreach (KeyValuePair<int, FcsController> item3 in new List<KeyValuePair<int, FcsController>>(surfaceOwner))
				{
					if (item3.Value == c)
					{
						surfaceOwner.Remove(item3.Key);
						surfaceMap.Remove(item3.Key);
					}
				}
				RemoveFrom(anySurfaceOwner, (FcsController o) => o == c);
				RemoveFrom(deviceOwner, (FcsController o) => o == c);
				List<ControlInputs> list2 = new List<ControlInputs>();
				foreach (KeyValuePair<ControlInputs, FcsController> byInput in byInputs)
				{
					if (byInput.Value == c)
					{
						list2.Add(byInput.Key);
					}
				}
				foreach (ControlInputs item2 in list2)
				{
					byInputs.Remove(item2);
				}
			}
		}

		private static void RemoveFrom<T>(Dictionary<int, T> d, Func<T, bool> match)
		{
			List<int> list = new List<int>();
			foreach (KeyValuePair<int, T> item in d)
			{
				if (match(item.Value))
				{
					list.Add(item.Key);
				}
			}
			foreach (int item2 in list)
			{
				d.Remove(item2);
			}
		}

		public static FcsController GetOrCreate(Aircraft a)
		{
			Prune();
			int instanceID = ((UnityEngine.Object)(object)a).GetInstanceID();
			if (controllers.TryGetValue(instanceID, out var value))
			{
				return value;
			}
			if (failed.Contains(instanceID))
			{
				return null;
			}
			try
			{
				EnsureSetup(a);
				value = new FcsController(a);
				controllers[instanceID] = value;
				foreach (SurfaceEffector surface in value.surfaces)
				{
					surfaceMap[surface.cs.GetInstanceID()] = surface;
					surfaceOwner[surface.cs.GetInstanceID()] = value;
				}
				List<ControlSurface> list = Acc.Collect<ControlSurface>(a);
				List<string> list2 = new List<string>();
				foreach (ControlSurface item in list)
				{
					if (!(item == null))
					{
						anySurfaceOwner[item.GetInstanceID()] = value;
						float num = Acc.CS_maxSplit(item);
						float num2 = Acc.CS_brakeRange(item);
						bool flag = surfaceMap.ContainsKey(item.GetInstanceID());
						if (num > 0f || Mathf.Abs(num2) > 0.01f)
						{
							list2.Add(string.Format("{0}{1}: maxSplit {2:F1}, brakeRange {3:F1}", item.gameObject.name.Replace("Aryx_KingRaptor_", ""), flag ? "" : " [not an FCS effector]", num, num2));
						}
					}
				}
				Plugin.Log.LogInfo((list2.Count > 0) ? string.Format("Su-30 FCS airbrake inventory ({0} control surfaces): {1} - the FCS drives these instead of the stock brake/idle-throttle rules.", list.Count, string.Join("; ", list2.ToArray())) : $"Su-30 FCS airbrake inventory ({list.Count} control surfaces): none of them split or respond to the brake input, so whatever deploys is not a ControlSurface.");
				foreach (int ownedId in value.flaps.ownedIds)
				{
					deviceOwner[ownedId] = value;
				}
				if (value.inputs != null)
				{
					byInputs[value.inputs] = value;
				}
				return value;
			}
			catch (InvalidOperationException ex)
			{
				if (!attachRetry.TryGetValue(instanceID, out var value2))
				{
					value2 = 0;
				}
				value2 = (attachRetry[instanceID] = value2 + 1);
				if (value2 == 1 || value2 % 250 == 0)
				{
					Plugin.Log.LogWarning("Su-30 FCS waiting to attach (attempt " + value2 + "): " + ex.Message);
				}
				if (value2 > 1500)
				{
					failed.Add(instanceID);
					Plugin.Log.LogError("Su-30 FCS gave up attaching; stock flight controls stay active: " + ex.Message);
				}
				return null;
			}
			catch (Exception ex2)
			{
				failed.Add(instanceID);
				Plugin.Log.LogError("Su-30 FCS could not attach; stock flight controls stay active for this aircraft: " + ex2);
				return null;
			}
		}

		// Su-30 0.1.0: the stock CanardSurface script (Aryx's, in the Su-30 mod's own DLL) turns each canard hinge 1:1 with AoA (+-30);
		// on the leading-edge flaps it only animates the mesh. While the FCS flies the canards it is switched off on the canards (hinge
		// back to its rest angle) and the FCS drives them through their control surface (+65 / -45); when the FCS stops it is on again.
		private static readonly Dictionary<int, List<MonoBehaviour>> canardScripts = new Dictionary<int, List<MonoBehaviour>>();

		private static readonly HashSet<int> canardMissLogged = new HashSet<int>();

		private static readonly Dictionary<int, int> canardFound = new Dictionary<int, int>();

		private static Type canardType;

		private static bool canardTypeSearched;

		internal static int CanardScriptCount(FcsController c)
		{
			Aircraft a = c?.ac;
			if ((UnityEngine.Object)(object)a == null)
			{
				return 0;
			}
			return canardFound.TryGetValue(((UnityEngine.Object)(object)a).GetInstanceID(), out int n) ? n : 0;
		}

		// Su-30 0.1.8: 0.1.0-0.1.7 searched only under the Aircraft's own transform and never found the script: Blueprinter splits
		// the jet into separate physics bodies (34 aero sections), and the canard struct carrying CanardSurface is not under the
		// Aircraft. Every Su-30 log so far shows the schedule running alongside the FCS: the canard's flow angle at zero
		// deflection was max(0, AoA - 30) instead of the AoA (8-9 Nov). Now the scene's CanardSurface scripts are searched too,
		// matched to this aircraft by their own 'aircraft' field.
		private static List<MonoBehaviour> FindCanardScripts(Aircraft a)
		{
			var list = new List<MonoBehaviour>();
			var seen = new HashSet<MonoBehaviour>();
			void Consider(MonoBehaviour mb, bool underAircraft)
			{
				if (mb == null || !seen.Add(mb) || mb.GetType().Name != "CanardSurface")
				{
					return;
				}
				Type t = mb.GetType();
				Aircraft owner = AccessTools.Field(t, "aircraft")?.GetValue(mb) as Aircraft;
				if ((UnityEngine.Object)(object)owner != null ? (owner != a) : !underAircraft)
				{
					return;
				}
				Transform st = AccessTools.Field(t, "surface")?.GetValue(mb) as Transform;
				if (st != null && st.name.ToLowerInvariant().Contains("canard"))
				{
					list.Add(mb);
				}
			}
			foreach (MonoBehaviour mb in ((Component)(object)a).GetComponentsInChildren<MonoBehaviour>(true))
			{
				Consider(mb, true);
			}
			if (list.Count == 0)
			{
				if (!canardTypeSearched)
				{
					canardTypeSearched = true;
					foreach (Assembly asm in AppDomain.CurrentDomain.GetAssemblies())
					{
						Type[] types;
						try
						{
							types = asm.GetTypes();
						}
						catch (ReflectionTypeLoadException rtle)
						{
							types = rtle.Types;
						}
						catch
						{
							continue;
						}
						foreach (Type ty in types)
						{
							if (ty != null && ty.Name == "CanardSurface" && typeof(MonoBehaviour).IsAssignableFrom(ty))
							{
								canardType = ty;
								break;
							}
						}
						if (canardType != null)
						{
							break;
						}
					}
				}
				if (canardType != null)
				{
					foreach (UnityEngine.Object o in UnityEngine.Object.FindObjectsOfType(canardType))
					{
						Consider(o as MonoBehaviour, false);
					}
				}
			}
			return list;
		}

		internal static void SetCanardScripts(FcsController c, bool own)
		{
			Aircraft a = c?.ac;
			if ((UnityEngine.Object)(object)a == null)
			{
				return;
			}
			int id = ((UnityEngine.Object)(object)a).GetInstanceID();
			if (!canardScripts.TryGetValue(id, out var list))
			{
				list = FindCanardScripts(a);
				// Su-30 0.1.8: an empty search is not cached (retried by the controller), and logged once per aircraft
				if (list.Count > 0)
				{
					canardScripts[id] = list;
					Plugin.Log.LogInfo($"Su-30 FCS: {list.Count} canard AoA-schedule script(s) found (CanardSurface); switched {(own ? "off" : "on")}.");
				}
				else if (canardMissLogged.Add(id))
				{
					Plugin.Log.LogWarning("Su-30 FCS: no canard AoA-schedule script (CanardSurface) found yet; retrying while the FCS flies.");
				}
			}
			canardFound[id] = list.Count;
			foreach (MonoBehaviour mb in list)
			{
				if (mb == null)
				{
					continue;
				}
				mb.enabled = !own;
				if (own)
				{
					Type t = mb.GetType();
					Transform st = AccessTools.Field(t, "surface")?.GetValue(mb) as Transform;
					FieldInfo fr = AccessTools.Field(t, "restingRotation");
					FieldInfo fa = AccessTools.Field(t, "currentAngle");
					if (st != null && fr != null)
					{
						st.localRotation = (Quaternion)fr.GetValue(mb);
					}
					fa?.SetValue(mb, 0f);
				}
			}
		}

		public static void Fail(FcsController c, Exception e)
		{
			Plugin.Log.LogError("Su-30 FCS fault, reverting this aircraft to stock flight controls: " + e);
			c.Failed = true;
			c.Engaged = false;
			c.OwnCanards(false);
			if ((UnityEngine.Object)(object)c.ac != null)
			{
				failed.Add(((UnityEngine.Object)(object)c.ac).GetInstanceID());
			}
			Remove(c);
		}

		public static void Disengage(Aircraft a)
		{
			if (!((UnityEngine.Object)(object)a == null) && controllers.TryGetValue(((UnityEngine.Object)(object)a).GetInstanceID(), out var value))
			{
				value.Engaged = false;
				value.OwnCanards(false);
				value.ThrustMatchClear();
			}
		}

		public static void Remove(FcsController c)
		{
			c.OwnCanards(false);
			c.ThrustMatchClear();
			c.Dispose();
			if ((UnityEngine.Object)(object)c.ac != null)
			{
				controllers.Remove(((UnityEngine.Object)(object)c.ac).GetInstanceID());
			}
			if (c.inputs != null)
			{
				byInputs.Remove(c.inputs);
			}
			foreach (int ownedId in c.flaps.ownedIds)
			{
				deviceOwner.Remove(ownedId);
			}
		}

		public static bool TrySurface(ControlSurface cs, out SurfaceEffector s, out FcsController c)
		{
			int instanceID = cs.GetInstanceID();
			s = null;
			c = null;
			if (surfaceMap.TryGetValue(instanceID, out s))
			{
				return surfaceOwner.TryGetValue(instanceID, out c);
			}
			return false;
		}

		public static void ForgetSurface(ControlSurface cs)
		{
			int instanceID = cs.GetInstanceID();
			surfaceMap.Remove(instanceID);
			surfaceOwner.Remove(instanceID);
		}

		public static bool TryInputs(ControlInputs ci, out FcsController c)
		{
			return byInputs.TryGetValue(ci, out c);
		}
	}
}
