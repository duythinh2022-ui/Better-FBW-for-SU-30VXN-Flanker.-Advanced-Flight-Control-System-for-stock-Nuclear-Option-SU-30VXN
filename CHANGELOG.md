# Changelog

## Su-30VXN FCS 0.5.6 (12 Nov 2026)
No law change (bench identical to 0.5.5). The coupled asymmetric-load check's LP now runs in the native core (su30_fcs_cpl) and its surface points come from the per-tick grid in one kernel call: the loaded sweep costs about 37 % less on the bench. See Su30_FCS_0.5.6_notes.md.

## Su-30VXN FCS 0.5.5 (11 Nov 2026)
Tom: "the landing gears state effects flaps logic, can u get rid of that, and make the flaperons work like on the raptor". Not flown.
- FlaperonFlapSchedule (on) / FlaperonFlapMax (23): Hooks.SurfacePostfix -> LandingFlapJob writes the flap job's gearState =
  LockedExtended and pitchRange = GameFlapStep (the F-22E flapBias schedule x roll priority, 40 deg/s), so the game's own slew + clamp
  put the landing-flap hinge (parent of the flaperon) on the FCS's droop at any gear state. FlapSystem: the flaperon's DroopCap stall
  cap (present droop removed, 12 x |stick roll| headroom) on the game flap. Ground = weight on wheels, as on the F-22E. Off = 0.5.4.
- Bench: Plant.GameFlaps (LandingBias hinges; the game's gear rule or GameFlapStep); stubs GearState in the game's order,
  ControlSurfaceFields.gearState. 1 g droop 20 / 14 / 10 / 2 deg at 119 / 140 / 162 / 216 kt; 0.5-1.2 deg less AoA for the same g;
  ~0 in full rolls, roll rate unchanged; take-off rotation identical. Off: reg / cork / sweep 72 / 72 identical to 0.5.4.
  On vs 0.5.4: sweep reversal worst beta 15.7 -> 13.9; reg mean peak beta 8.47 -> 7.96 (worse in 8 / 20); cork worst 18.2 -> 22.0.
- Game's Mono under Wine (native/verify/flapcheck.cs): flap path JIT-compiles against Assembly-CSharp; gear patch applied.
  Notes: Su30_FCS_0.5.5_notes.md.

## Su-30VXN FCS 0.5.4 (11 Nov 2026)
Tom (21:06-21:12): "cannot drift on runway"; no gates or timed logic, "why not make the diff brakes help in drifting". Not flown.
- GsPedalBrake / GroundPedalBrake (0.5): pedal in -> the main whose brake yaws the jet the pedal's way gets |pedal| x 0.5 (never below
  brake + stock term; no speed window). Bench 3 s full pedal: peak sideslip 8 kt 12.9 -> 20.0, 15 kt 7.1 -> 12.8, 40 kt 8.4 -> 13.8 deg;
  hold / 0.6 s release / re-apply at 8 kt: 12.7 -> 20.3 deg. Speed bleeds while the pedal is in.
- Fast-roll latch left as it was: "only a pull hands over" lifted the jet off by itself on the take-off bench (stick centred, no
  pedal: airborne at 13.4 s / 6.9 s, pitch 6.9 deg; current latch: pitch max 0.1 deg).
- Logs: 21:04 above 87 kt, stick centred: mode 2 in 34-86 % of the ticks, nose wheel at 0-3 deg (others 0 %); 21:02 forward stick
  drifted 20-27 deg. Suites vs 0.5.2: 87 / 87 identical. Notes: Su30_FCS_0.5.4_notes.md.

## Su-30VXN FCS 0.5.3 (11 Nov 2026)
Tom: "the turn stop is very slow" → ground-brake handoff → "do the first one" (option 1, Su-30 first). Pedal phase unchanged (stock).
Not flown.
- IL (LandingGear.FixedUpdate): b = (braked ? brake : 0) [+ parking] + Clamp01(yaw x 0.3 x dbf) for 3-30 m/s, summed;
  brakeStrength = Lerp(.., b, 5 dt) (0.2 s lag); brake force x muN along -velocity, total clamped to muN. The steering slew runs off
  the ground too (the 18:55 log's frozen gsWheel was the debug value; the real wheel ran 23.7 -> 0.4 deg, after 3 ticks of -60).
- GroundIndiLightNoseHold: nose tyre unusable in a stop / hold INDI was flying -> wheel = contact-velocity angle + INDI's last slip,
  at most 1 s (GsLnMaxT).
- GroundIndiBrake (+ GroundIndiBrakeMax 1.0, GroundIndiBrakeFullSpeed 7.7, GroundIndiBrakeZeroSpeed 12.9 m/s): GsBrakeAlloc - the
  wanted total effector yaw acceleration goes to the nose tyre up to muN, the excess to the brakes on the right side; the nose takes
  the rest crediting each brake's lagged value next tick. Never below brake + stock term. Hooks.GearPrefix writes the leg's brake
  (struct __state GearSwap).
- Log columns gsBrkL, gsBrkR, gsBsL, gsBsR, gsLn. Bench mode gyaw (the FCS's C# on the IL gear model), groundsim/brakesim.py,
  native/verify/gearcheck.cs.
- Bench (gyaw, 23.9 t, 21 % on the nose): release stops 7-17 kt 20 % shorter (10 kt 1.67 -> 1.33 s, heading 43.5 -> 35.5 deg),
  5-7 kt more speed lost; light nose as logged 2.10 -> 1.70 s; hold kicks inside the nose's grip unchanged. Suites vs 0.5.2 with the
  switches off and on: 87 / 87 identical. Game's Mono under Wine: self-test 200 / 200, accessors bound, gear patch applied.
  Notes: Su30_FCS_0.5.3_notes.md.

## Su-30VXN FCS 0.5.2 (11 Nov 2026)
Tom: "Dont make it a gate or switch. Make it estimate the remaining useful authority to limit aoa" (18:26); after 0.5.1 (18:53 log, 20 cm,
locked): "the roll to the heavy side is very slow, i'd rather have my aoa a bit lower to get more roll. But dont make it pessimistic". Not flown.
Log: heavy-side roll at the 50 deg limit 10-16 deg/s at 100-125 kt (0.8 s x a 13-20 deg/s^2 reserve); negative limit on its 8 deg floor.
- CplKnee: with a load, the AoA limit is where AoA^AsymAoAWeight x the usable roll is largest (usable roll = the law's roll, held to what
  the weak side's reserve stops in the clean jet's stopping time at its limit and in AsymStopTime; faded in with the load). No load: the
  clean limit. On this airframe the weak side's reserve is flat 20-47 deg and falls past 50; the maximum sits at the knee, about 45 deg
  (a crossing search there flipped 48 -> 20 -> 47 deg). CplRefAuth: the clean reference solved at the clean limit's sweep points.
- Roll toward each side: law roll x weak reserve / clean reserve, and AsymStopTime (0.8 s) x weak reserve, 10 deg/s at least.
- Dev setting AsymAoAWeight (1.0). Log: new column cplRef.
- Bench: limit 45.2 / 44.8 / 44.0 deg at 16 / 20 / 31 cm locked (0.5.1: 51-52), TVC on 45-49 (0.5.1: 52 -> 75 below ~100 kt), negative
  -65 (0.5.1: down to -9); heavy-side roll x1.3-1.45 in replays of the 18:53 rolls; pitch-held stops 0.9-1.8 s (clean 1.4-1.8).
  Loaded reversals: AoA above the (lower) limit mean 1.0 -> 2.4, worst 5.0 -> 9.3, peak AoA mean 61.9 -> 49.4. Suites vs 0.5.1: identical
  (87 / 87, 4 / 4). Sweep while loaded 329 -> 399 us. Notes: Su30_FCS_0.5.2_notes.md.

## Su-30VXN FCS 0.5.1 (11 Nov 2026)
Tom (17:26, 0.5.0 flown, 17:22 log, nozzles locked, 16 cm): "now the cap is at 50 aoa without tvc instead of swinging like before. And
the jet still tends to tips to one side. I release roll and then the roll rate drifts a lot". Base 0.5.0. Not flown.
Log: limit steady 50.0-52.8 deg; rolls into the heavy side ran 67-90 deg of bank after the stick was centred (36-46 deg/s on, 29-43 still
on 1 s later; 109.0 / 117.8 / 125.3 / 155.4 s); the other way 1.1 s.
- AsymStopTime (dev setting, 0.8 s): with a load the flight-path roll the stick may ask toward each side is the coupled check's reserve
  on its braking side at the present AoA x AsymStopTime, 10 deg/s at least (at 50 deg / 131 kt that reserve was 29 deg/s^2 against
  32-50 deg/s of roll allowed). 0 = 0.5.0.
- Push governor, roll stick released: the no-stab discount of its yaw read is weighted by the commanded nose-down body rate (pitch stick
  centred = the stabs are free; read without them the roll's brake was held to 0-8 deg/s^2), and the roll may not be made to grow
  (155.4 s: 35.9 -> 39.0 deg/s after the release); the brake keeps 10 deg/s^2. Both act on the clean jet too.
- Bench: replays of the four slow stops, bank after the release 47-117 -> 27-44 deg; heavy side at 104 kt, pitch held: not stopped in
  3.5 s -> 13 deg in 1.05 s (roll 32 -> 17 deg/s). Roll on the limit with the load: locked 53 -> 39 deg/s (243 kt), 48 -> 31 (189 kt);
  TVC on 77 -> 65, 71 -> 55. Three switches off = 0.5.0 (87 / 87 files, 4 / 4 tables); clean tables with the defaults: 5 of 105 rows
  changed (released rolls at 65 deg: peak sideslip 13.4 -> 12.0, 16.2 -> 14.6). Bench plant against the log: yaw acceleration rms 1.3-3.0.
- Not fixed: both sticks centred at the 50 deg no-TVC limit still runs 47-75 deg (0.5.0: 118-195; the clean jet 69-89 in both builds).
- Log: new columns stopRateP, stopRateN. Bench: bench/an/stopmat.sh|py. Notes: Su30_FCS_0.5.1_notes.md.

## Su-30VXN FCS 0.5.0 (11 Nov 2026)
Tom: "For flanker with asymmetric load, when i roll aoa limit rises"; "flanker asymmetric load is not fixed." Base 0.4.1. Not flown.
Logs (16.4-19.5 cm of lateral CG offset, 25.5 t): raw lateral limit 12-75 deg, 42 and 68 jumps over 10 deg in 0.5 s, 4 and 7 forced
nose-downs with full aft stick (pitch rate to -50 deg/s, AoA up to 17.9 deg over the limit). Cause: the one-axis check froze the surfaces
at their present symmetric deflection, so its answer moved with the stick and the AoA.
- Coupled lateral check (CoupledCap.cs, from the Compass 0.4.0 / Ternion 0.2.3): per sweep AoA a linear programme over every effector's
  moment points gives the flight-path roll left each way with the load, pitch and sideslip held. With TVC on and both engines running
  each canted nozzle is its own effector (TvcEffector.Mn) and the thrust at the present angle leaves the load; canard pair = one
  effector; stabs / canards offered the allocator's own range (ClampStabToPeak rule); rudders full range.
- AsymCap.FindTop: the limit is the lower edge of the over-the-line stretch that reaches the top of the envelope, with a latch; the
  first-crossing rule fell 51 -> 12 deg on this jet's hump at 10-30 deg AoA.
- Nose-down limit reads the same check (PitchHeld); margin scales with dynamic pressure under 130 m/s IAS (floor 6 deg/s^2).
- AsymThrustCgLoad (Ternion 0.2.2): thrust about the offset CG is part of the load when the nozzles are fixed.
- Dev setting AsymCoupledReserveMax (0.8). One side solved per sideslip case; 2 sweep points per tick while loaded.
- Bench (22.8 t, 800 kg = 16 cm, 1600 kg = 31 cm, above 45 m/s): limit 51.3-52.7 deg (16 cm, TVC on and locked), 46-51 deg (31 cm),
  0 jumps (0.4.1: 12-75 deg, 3-14 jumps per roll case, AoA 11-20 deg over); TVC on it opens to 75 deg as IAS falls 75 -> 50 m/s.
  Loaded reversals (88): AoA over the limit mean max 31.3 -> 1.1 deg, worst 52.5 -> 4.6. Suites vs 0.4.1: 87 / 87, 4 / 4 identical.
  Sweep cost while loaded about 100 -> 400-490 us per tick. Damage asymmetry not benchable (no Su-30 damage data).
- Log: new columns cplU, cplRp, cplRn, cplDn, pitchMarg. Bench: engout= now finds this jet's engines; bench/an/asymmat.sh|py.
- Notes: Su30_FCS_0.5.0_notes.md.

## Su-30VXN FCS 0.4.1 (11 Nov 2026)
The native core (Su30_FcsCore.dll) ships inside the plugin DLL: one DLL to install. No control-law change. Notes: Su30_FCS_0.4.1_notes.md.

## Su-30VXN FCS 0.4.0 (11 Nov 2026)
The allocator's solve, the surface tables' maths and the asymmetric-load sweep's inner loop run in a native library (Rust, ported from
the F-22E FCS). Same flying, less CPU time per tick. Notes: Su30_FCS_0.4.0_notes.md.

## Su-30VXN FCS 0.3.10 (11 Nov 2026)
Tom (09:34, 09:31 log): "get rid of the 3degs after overshooting aoa limit since that cause the jet to nose down". Bench-only.
- SoftPitchHold (internal, on; his pick of two): past the AoA limit by 0.5 deg (where the protection push starts) and until the AoA has
  been back inside for 0.25 s, the AoA law does not pitch the nose down by itself - released stick = zero pitch rate, aft stick may not pull
  further past (rate capped at the flight path's), forward stick still pitches in (down to -48 deg/s). The AoA law keeps its lateral law;
  the full MPO hand-over stays at +3 deg (SoftAoALimitMargin). A 0 deg MPO margin was benched and not taken: every normal max-AoA pull
  overshoots 0.5-1.5 deg, so corkscrews flew the MPO law 12-71 % of the time, peak AoA up to 79.6 (75.6), worst sideslip 6.8 -> 15.8.
- Soft hand-over: no airspeed cut-out once it holds (needed TAS > 15 m/s every tick: 09:31 log 122.4 s and 00:32 log 628.0 s it dropped
  at 14-15 m/s and the AoA law pushed at 44-48 deg/s); it starts down to 5 m/s.
- SoftMpoStick (0.3.7) off: it misread Tom's 00:45 report ("hold it there" = hold the AoA, no recovery); a released stick past the limit
  holds, as he described at 01:01.
- Bench from the game's state: 09:31 108.7 s (released at 77.6 inside the old 3 deg band) holds 77.0-77.6 in the AoA law (0.3.9: push to
  56.6; game: -22 deg/s to 68.9); 122.2 s (held at 112, TAS falling) keeps holding, no push, AoA back to 78 by itself (0.3.9: -48 deg/s to
  60). 00:32 626.3 s released at 84 deg, 20 m/s, banked 141-170: zero rate holds, the flight path falls through, AoA 90 -> -49 and 75 deg
  sideslip; the AoA law at 13 m/s does not recover it either (G-rate command 0 with the stick released, unchanged). Suite identical to
  0.3.9. 48 low-speed replays vs the pre-0.3.7 soft limit: mean peak sideslip +0.1, peak AoA up where the push used to act (78.3 s 77.2 ->
  83.0, 950 s 81.7 -> 87.8, 1046.9 s 90.7 -> 97.9).
- Log: new column softPitch.

## Su-30VXN FCS 0.3.9 (11 Nov 2026)
Tom (01:04): "if i mpo the jet and then turn it back on, no recovery i want that behavior". Base 0.3.8 (0.3.7 + loaded roll-reversal port). Bench-only.
- SoftMpoFromMpo (internal, on): Stability Assist switched back on past the AoA limit (+3 deg margin) after the pilot's MPO lands in the
  soft hand-over (MPO law, no protection push) and stays there until the AoA has been back inside for 0.25 s - releasing the stick does not
  end it (that release rule, 0.3.7, stays for the hand-over the stick forced). Forward stick still pitches back in; aft stick may not pull
  further past (0.2.0). 00:59 log 42.15 s (MPO to 121 deg, assist on, stick released): the AoA law pushed at 45-48 deg/s, under 75 at
  43.0 s, peak sideslip 12; now zero rate, the AoA drifts back with the flight path, under 75 at 44.7 s, hand-back at 44.95 s, peak
  sideslip 34. 16:16 log 54.1 s: under 75 at 55.9 (54.8 with the push), sideslip 9.1 (2.2); 813.3 s (forward stick): unchanged.
  00:32 log 122 s (assist on at 99.8 with full forward stick): soft now, the stick flies it down (36.7 deg at 123.7 s). 0.3.7's stick-release
  case (00:32 626.3 s) unchanged: peak 88.4. Regression suite identical.

## Su-30VXN FCS 0.3.8 (11 Nov 2026)
With an asymmetric load, a roll reversal toward the heavy side builds one roll acceleration from the stick flip, paced by the rudders' travel, and keeps it through the zero crossing; against the load the new roll starts at its brake's acceleration (F-22E 4.1.1 + 4.1.2). Bench switches RevOneAccel / RevLoadRamp. Benched, not flown; see Su30_FCS_0.3.8_notes.md.

## Su-30VXN FCS 0.3.7 (11 Nov 2026)
Tom (00:45, 00:32 log): "at the end my aoa was at 100 something the fcs is supposed to hold it there even without mpo". Bench-only.
- SoftMpoStick (internal, on): the soft AoA-limit hand-over to the MPO law (0.2.0) now needs the stick pulling toward that limit (past
  0.5) and ends as soon as it eases under 0.3; the AoA law's protection then pushes back to the limit. 00:32 log 625.8-628.0 s: full aft
  + full left at 22 m/s let the AoA rise with the falling flight path (StickCancelsLimit: full aft at the limit holds zero rate), soft MPO
  took over at 78 deg, the stick was released at 626.5 and the MPO law held zero pitch rate with no push-back: 87 -> 112 deg hands off,
  handed back only when TAS fell under 15 m/s, peak 124 deg, 52 deg of sideslip. Bench from the game's state at 626.3 s: released, the
  AoA law takes it the same tick and pushes at 48 deg/s - peak 88 deg (90 with the old law on the bench), back under 75 by 627.7 s,
  sideslip peak 43 (75). 122 s (full forward stick at 99 deg, which used to enter soft MPO): stays in the AoA law, same recovery, less
  undershoot (46 vs 38 deg). 48 low-speed replays: 0 worse; 1046.9 / 1049.8 s (stick eased in the hand-over) peak AoA 90.7 -> 85.3 and
  104 -> 99.7, mean sideslip 10.3 -> 7.8 and 8.3 -> 5.1. Regression suite identical.
- Bench: replayed MPO rows of a log fly the AoA law when the row was the soft hand-over (softMpo = 1), so the FCS decides it.

## Su-30VXN FCS 0.3.6 (10 Nov 2026)
Tom (23:22): "maybe the correction is better but it is not a smooth ride when that happen". Bench-only.
- LsSmooth (internal, on): LowSpeedSlipTvc now fades in and out instead of switching. The correction is worked out as a target every tick
  (zero when it is not wanted) and the outputs follow it: the stability-yaw command through a 0.25 s lag (LsCmdTau), the allocator's
  raised weights (velocity-roll 30, yaw 40) through a 0.4 s lag (LsWeightTau) - a weight step re-shuffled every effector at once; onset
  is a smoothstep over 6 deg past the start (was a ramp over 3); the "can't correct, stop trying" cut fades over ~0.4 s with an S-curve
  (was 0.17 s linear) and re-arms 2 deg under the start. In 0.3.5 that cut dropped the command from ~16 deg/s to 0 in one tick.
  48 low-speed replays (16:16 log), around the corrections: biggest per-tick step of the correction 10.5 -> 1.0 deg/s; roll-command
  jitter 1.12 -> 0.68 (off: 0.64); roll-acceleration jitter 2.59 -> 2.45. The price: mean peak sideslip vs off -1.8 -> -0.9 deg
  (6 better / 6 worse by >3 deg, was 10 / 3); the big one kept (950 s: off 41.8, 0.3.5 31.4, now 31.1). Faster fade-in (0.1 / 0.2 s)
  tried: no sideslip back (-0.7), jitter up. Still off by default.

## Su-30VXN FCS 0.3.5 (10 Nov 2026)
Tom (22:44, 22:34 log): in corkscrews the AoA limit is often undershot; the sideslip correction is rough, prefers it off; at the end the
stick was left and the jet yawed right. Bench-only.
- Fix (push governor stale state): the push governor's band (pushGovL / U / A, pushGovSide) and the push plan's latch were refreshed only
  inside the governor block, which stops running past StopSlipAoAEnd + 5 deg, but applied every tick from their last values. 22:34 log
  251.7-276 s: a right corkscrew handed over to MPO, the band froze at side +1 and for 24 s turned the stick's -31 deg/s target into
  +4..+46 (rolled and yawed right with full left stick). Cleared each tick now; the plan's latch drops when its step did not run. Bench
  replay: the left stick gives -33..-37 deg/s and the jet yaws left. Regression suite identical.
- GyroRollTrade (internal, on): GyroBudget's (Iyy - Izz)/Ixx q r budget is also spent on the roll - the velocity-roll target is held to
  (budget / wanted pitch rate) / sin AoA. Corkscrew replays (22:34 log): peak sideslip 37.8 -> 24.2 (177 s), 30.3 -> 22.6 (205 s), others
  unchanged; the AoA dips over the top (75 -> 38 at 72 km/h) stay - there the flight path bends at 40-48 deg/s and the yaw cannot be shed
  with the rudders blanked, so the pitch to hold 75 deg is not available either way. 48 low-speed runs: 0 worse, 13 better, mean peak
  sideslip -3.9 deg. Suite: 65 deg AoA roll releases at 300 km/h 25.6 -> 13.5 / 23.3 -> 15.8 deg of sideslip, leaf cases slightly better.
- LowSpeedSlipTvc default off again (Tom: rough); the 0.3.2 / 0.3.4 player migrations no longer turn it on.

## Su-30VXN FCS 0.3.4 (10 Nov 2026)
Tom (20:06, 19:37 log): the yaw stop past the max AoA is not fully working, sometimes the yaw keeps going - stop it even if it causes
some roll. Bench-only.
- MpoRelPastLimit (internal, on): the MPO release stop (yaw command to zero, body-axis allocation, roll weight cut, nozzle differential
  from the allocator) now applies whenever the MPO law flies past the AoA limit - the soft hand-over included - not only with the
  nozzle-axis weight. 19:37 log: seven releases past the limit with 50-58 deg/s of yaw, five in the soft hand-over with mpoAxisW 0-0.2;
  the yaw command stayed at 36-50 deg/s (the released velocity roll at 85-88 deg AoA is body yaw, wound down at the ~4 deg/s^2 the
  rudder-blanked yaw estimate allowed): three never stopped before the stick came back (135, 540, 924 s). Replays: 924 s yaw under
  10 deg/s in 1.3 s (never), 605 s 2.0 -> 1.25 s, 481 s 1.8 -> 1.4 s (and AoA 178 -> 94); |r| 1.5 s after release 30 -> 6 (924 s),
  15 -> 5 (605 s), 18 -> 8 (481 s), 28 -> 24 (540 s, nozzles at full differential - authority-limited). The roll it costs: 924 s peak
  sideslip 31 -> 42 deg. The 16:16 releases (already in the nozzle-axis band) and the regression suite unchanged.
- Config bug: Su30ConfigRevision was bound after MigrateOldConfig, which clears every player key not yet bound and saves - each launch
  dropped the revision, re-created it at the newest default and skipped every player "moved once" step. Tom's file: revision 4,
  PushoverSideslip 8, LowSpeedSlipTvc false (off for the whole 19:37 flight: 0 active ticks). Now bound before it; revision 5 moves
  PushoverSideslip 8 -> 15 and LowSpeedSlipTvc on once. The dev file was not affected (its revision is bound before its migrations,
  orphans dropped after all binds).

## Su-30VXN FCS 0.3.3 (10 Nov 2026)
Tom (18:29 / 18:31 / 18:37): why does the allocator not see that the nozzles roll and yaw together and the surfaces yaw, and fix the
sideslip itself? Then: count the nozzles' roll in the roll limits; then: find why the sideslip roll governor does not act. Bench-only.
- The allocator does see it: in the 858 s corkscrew every yaw effector, nozzles included, was saturated (yawSat 1.0) braking -46 deg/s
  of yaw. It works per tick on moments; the sideslip comes from the AoA falling 73 -> 43 deg under that yaw (beta_dot ~ p sin a -
  r cos a) and the fix needs more left roll with less left yaw - the one combination the canted nozzles cannot make.
- TvcRollLim (internal, OFF): aRoll counts the surfaces only, so pFull / the roll-stop limit / the braking floor ignored the nozzles'
  25-31 deg/s^2 (pMax ~14 where coordination needed ~30). Counted: 124 s 47.6 -> 41.1, 950 s 45 -> 37, but 858 s departs in 4 of 6
  runs (AoA 115): the roll the nozzles give comes with left yaw. Left off.
- PaceTasMin 40 -> 20 m/s: the pace window (the clean jet's check of the roll target against what the yaw can follow) had a hard
  40 m/s TAS floor; the low-speed corkscrews live at 25-40 m/s, where every lateral protection read idle. The 2.11.0 sideslip guard is
  gone and slipGov / the yaw anchor act only with an asymmetric load, so nothing else checks there. 8 events x 6 offsets: alone 0/48
  runs worse, 124 s 47.6 -> 41.0; with LowSpeedSlipTvc (default) mean peak change -2.4 -> -3.2, runs > 3 deg worse 3 -> 2, none departs.
  Held-stick velocity-vector roll within 1.5 deg/s; regression suite identical.
- Why the AoA falls with full aft stick at 25-35 m/s: GyroBudget (0.1.11) holds the pitch rate at 12-20 deg/s so the gyroscopic roll
  (Iyy - Izz)/Ixx q r stays within 25 % of the roll authority while yawing at 46 deg/s. Loosening it is far worse (0.5: 17/48 runs
  worse, AoA to 140-179; off: tumbles to 180). Unchanged: in that state the jet cannot pitch up, brake the yaw or coordinate with roll.

## Su-30VXN FCS 0.3.2 (10 Nov 2026)
Tom (17:58): instead of a plain switch, the FCS should see that it lacks the authority to correct the sideslip - and that trying would
make it worse - and then not try. Bench-only.
- LowSpeedSlipTvc rebuilt and now ON by default (player Su30ConfigRevision 4 turns it on once; dev Su30DevRevision 14 moves
  LowSpeedSlipTvcGain 1 -> 2 once):
  - command: stability-axis yaw toward zero sideslip, dRs = sgn(beta) x gain x (|beta| - start), along (-sin a, cos a) - the axis the
    sideslip actually answers to (beta_dot ~ -r_s); no velocity-vector roll added. The 0.3.1 body roll answered through p sin a only
    and came with the nozzles' yaw;
  - allocator: velocity-normal weight 40, velocity-axis weight 30 (internal LsSlipYawWeight / LsSlipVvWeight): the sideslip is chased only
    with effector combinations that do not swing the velocity-vector roll. Raising the yaw weight alone let the canted nozzles buy a
    little stability-axis yaw with a lot of velocity-vector roll, which pumps the AoA at large sideslip (-pS tan beta): 858 s replay,
    AoA 77 -> 90-121;
  - authority check (internal LsGate / LsGateT 0.15 s / LsGateFade 6 /s): progress m = sgn(beta) r_s; if 0.15 s after the correction
    comes in m is still negative and has not risen, the effectors are not turning it - the correction fades out and stays out until the
    sideslip is back under the start. A predicted-authority version (available sideslip acceleration vs the growth to stop) never fired
    where the correction hurt and fired where it helped; dropped.
  - Bench, 16:16 log, 8 low-speed sideslip events x 6 replay start offsets (the events are chaotic; single replays misled): peak
    sideslip 2.4 deg lower on average, every event equal or better on average (950 s 45 -> 32, 1050 s 27 -> 24, 124 s 47.6 -> 45.8,
    858 s 34.1 -> 33.2), 3 of 48 runs more than 3 deg worse (worst +5.9), none more than 3.4 deg of AoA over the uncorrected run.
    Without the check: 3 of 48 departed (AoA to 121). Regression suite: equal except the slow cases, all better (leaf stop 296 now
    stops in 3.8 s, leaf sideslip 19.3 -> 17.9 / 18.4 -> 17.1, 250 km/h rolling-pull corkscrews less sideslip).

## Su-30VXN FCS 0.3.1 (10 Nov 2026)
Tom (17:47): LowSpeedSlipTvc - make the TVC do the body roll and the control surfaces counter the yaw. Bench-only.
- LowSpeedSlipTvc (still off by default) rebuilt: the roll command blends toward p = (r cos a - sgn(beta) gain (|beta| - start)) / sin a
  (body roll that coordinates the present yaw and bleeds the sideslip; |p| <= LowSpeedSlipTvcMax), no added yaw; the allocator flies it
  in body axes with the yaw weight at 300 (LsSlipYawWeight, internal) so the nozzles roll and the surfaces hold off the nozzle yaw. Own
  stiffness band 0.8 -> 0.4 with a 0.3 s filter (was the AoA law's 0.4 -> 0.2, 1 s: came in with 21 deg of sideslip already built).
- Bench, 16:16 log: 855-870 s corkscrew peak sideslip 40.9 -> 25.6 deg, AoA within 75.7; but of seven other low-speed sideslip events
  one improves slightly, three are unchanged and three get worse (950 s departs to 85 deg of sideslip, 1047 s 24 -> 60). Why: full
  nozzle differential gives ~29 deg/s^2 of roll with ~23.5 of yaw; below ~45 m/s the surfaces have ~7-8 deg/s^2 of yaw, so the yaw-free
  roll is capped near 9 deg/s^2 and the allocator ends up braking the yaw and pausing (even reversing) the roll. At 55-65 deg AoA that
  helps (the r cos a term feeds the sideslip); near 75 the roll term dominates and stopping the roll builds sideslip faster.
- Off, the jet flies exactly as 0.3.0 (bench replay bit-identical).

## Su-30VXN FCS 0.3.0 (10 Nov 2026)
Tom (16:46, 16:16 log): low-thrust / no-TVC AoA limit +-50; TVC-on push-overs did not use the sideslip allowance and were slow; in MPO
the pedals should do the nozzle-axis (TVC vector) roll too, and after releasing the controls the FCS often would not stop a mostly-yaw
rotation - stop it faster, care less about the roll it makes; at very low speed corkscrews build a lot of sideslip - let the TVC take
some out, with a toggle; +75 AoA limit as the default. Bench-only results below (nothing here has been flown yet).
- AoALimitPositive default 70 -> 75; NoTvcAoACapNegative / NoTvcThrustAoACapNegative 45 -> 50 (positive already 50). Su30DevRevision 13
  moves 70 -> 75 and 45 -> 50 once. (With AoACap36 the TVC-on negative limit follows the no-TVC negative cap: -45 -> -50.)
- Player Su30ConfigRevision 3: PushoverSideslip 8 -> 15 once more - the 16:16 log's player file still held 8 after 0.2.4's move.
- Push plan yaw shed (TVC on) 0.0224 -> 0.028 x IAS^1.5. The 0.2.4 fit read the plan's own pacing back. 23 TVC-on planned pushes of the
  14:55 / 16:16 logs replayed at PushoverSideslip 15: median time to 10 deg AoA 1.85 -> 1.72 s (2.17 as flown with 8), median peak
  sideslip after the bottom 7.0 -> 8.3 deg, worst 35.5 -> 19.5. Known: the bench held push-over at 560 km/h reaches 18.6 deg (9.8 before).
  PushPlanHeldYawEnd (internal) was tried and left off (no faster, worse outliers).
- MpoAxisReleaseStop (dev, 3. Lateral-directional, true): in MPO's nozzle-axis rotation, with stick and pedals released, the yaw command
  goes to zero at once and the allocator works in body axes with the roll weight cut (0.05) and the yaw weight raised (8). Cause found on
  the bench: past 85 deg AoA the nozzle differential followed the roll stick directly, so on release it went to 0 and only the airframe
  damped the yaw; the released stop now hands the differential to the allocator. Replays of six releases in the 16:16 log (105-165 deg
  AoA): yaw under 10 deg/s 0.6-3.5 s sooner in five (e.g. 741 s 5.0+ -> 1.5 s, 1048 s 3.7 -> 2.0, 728 s 4.5 -> 2.9), the sixth unchanged;
  peak sideslip down in four; the induced roll up (peak 10-16 -> 13-23 deg/s), left to decay on its own. Bringing the roll weight back as
  the yaw dropped was tried: 1-2 s slower in two cases.
- AxisPedalRoll (internal, true): in the nozzle-axis band the pedals drive the differential like the roll stick (left pedal = left nozzle
  jet up; stick + pedal summed) and add to the rotation command at full weight at any AoA. Bench MPO at 112-138 deg AoA, full pedal: the
  nozzles move to 7.5-12 deg (0 before) and the rotation roughly doubles (p 18 / r 29 deg/s vs 10 / 14).
- LowSpeedSlipTvc (player, 1. General, default OFF - experimental) with dev LowSpeedSlipTvcStart 8 / Gain 1 / Max 40: at very low
  directional stiffness the nozzles turn the jet about their axis to take sideslip past the start out. On the bench (16:16 log 855-870 s)
  it did not help: peak sideslip 41 -> 43-44 deg and the AoA ran to 88-91, every gain and sign tried.
- Bench: scripts/kr.sh / ksum.py (TVC-on push replays, run from the folder above src); the stub's player values come from tomsu30.cfg (PushoverSideslip there is 8).

## Su-30VXN FCS 0.2.5 (10 Nov 2026)
Tom (15:45-15:52, 15:44 log): AoA hit the wall again (full aft + late roll from 550 km/h, pitch rate 70 -> 0 at 56-58 deg for ~1.3 s);
change the lift curve so there is enough pitch-up to cover 55-65 deg without affecting anything else.
- HighAoAPitchBump (dev, 5. Airframe, 0.75; new file PitchBump.cs): the Su-30_body sections ahead of the CG (Front_cover, Cheek_L/R,
  Mid_cover) get +K w/cos(a) lift, the ones behind it (intake / engine covers, engines, tail cone) the same total lift less - a pure
  nose-up couple, no net lift or drag (bench: |dnz| and |dax| < 0.0001 over the whole pose grid). w: 53 -> 57.5 deg in, flat to 65,
  out to 0 at 70 (smoothstep); positive AoA only. Bench best nose-up pitch acceleration (nozzles +15, best stab / canard pose, q 0,
  T/W 1.3): +29-31 deg/s^2 at 57.5-62.5 deg / 450 km/h (+13 at 300, +42-46 at 550), unchanged below 52.5 and at 70; the drop past
  55 deg (66 -> 33 at 450, 71 -> 13 at 600) is filled to +62-71 from 55 to 65 deg. Best nose-down unchanged within 1 deg/s^2.
- Bench: PitchBump statics settable by Name=value; rsync copies PitchBump.cs; FAMDBG=1 prints each part's chart family.

## Su-30VXN FCS 0.2.4 (10 Nov 2026)
Tom (14:55, 15:22; 14:55 log, TVC on and off): positive low-thrust / no-TVC AoA cap 50 by default; a -50 negative cap still flew -45;
one big AoA-limit overshoot with TVC off; push-overs make a lot of sideslip - allow 12 deg without TVC, 15 with, and a 23 deg/s floor
like the Raptor's (sideslip ignored there); Buffed thrust 15 % instead of 12.
- NoTvcAoACap* / NoTvcThrustAoACap* magnitudes were clamped to 45 (F-22E heritage); now 10-90. NoTvcAoACapPositive default 45 -> 50.
- WallLiveLimit (internal, true): the pitch wall's canard / stab peak pin ends 2 deg short of the AoA limit in force (no-TVC, low-thrust and
  asymmetric caps included), not AoALimitPositive; the pin is let go at once when the pitch rate reaches its target. With TVC off the pin
  held the canards at their nose-up peak (-34 deg, local AoA 60-83) through the approach to the 46-50 cap (log: 245.5 s to 81 deg,
  297.5 s to 64.7, 353.2 s to 72.4). Bench replay of 353 s: 60.5 -> 52.9 deg peak against a 50 cap.
- Push plan shed refit to the Su-30's own logs (was the Revoker's constant 40 deg/s^2): TVC on 0.0224 x IAS^1.5, TVC off 6.
- PushoverSideslip (player) 8 -> 15 (moved once when the file holds 8), new PushoverSideslipNoTvc 12; the push governor keeps 8.
- PushOverMinRate (dev, 4. Gains, 23): planned pushes are held to max(plan, 23 deg/s), no sideslip guard (F-22E 3.1.4/3.1.5).
- ThrustScale (Buffed) 1.12 -> 1.15. Su30DevRevision 12 moves NoTvcAoACapPositive 45 -> 50 and ThrustScale 1.12 -> 1.15 once.

## Su-30VXN FCS 0.2.3 (9 Nov 2026)
Tom (23:16): on the ground the nozzles only follow the pitch stick; roll and yaw inputs should move them too.
- GroundTvcRollYaw (5. Airframe, true): in the Ground mode the canted nozzles' differential follows roll stick + pedals directly
  (left stick / left pedal = left nozzle jet up, right jet down, the same sense as the LowSpeedTvcRoll band), added to the pitch stick's
  symmetric part, each nozzle clamped to +-15 deg. Stock-vectoring jets get the roll / yaw inputs too. Flight unchanged (bench output
  identical to 0.2.2).
- Bench: takeoff mode takes groll= / gyaw= and prints the nozzles' final symmetric / differential deflection.

## Su-30VXN FCS 0.2.2 (9 Nov 2026)
Tom (23:03-23:05): best CG / lift setup for taxi without losing the stability margin; "do it".
- CgShiftAft 0.15 -> 0.10, CanardEffectiveness 1.5 -> 1.6 (Su30DevRevision 11 moves a file holding the 0.2.0 or 0.2.1 values once).
  Bench static pitch vs 0.2.0 (0.25 / 1.3): 5-10 deg slope -1.67 vs -1.56 deg/s^2 per deg, 20-45 deg within 3.4, 60-75 within 1.1.
  Static nose-wheel share 9.6 % (0.2.0) -> 12.2 %. AftBodyLift 0.5 and LerxEffectiveness 1.5 unchanged (they act above ~45 deg).
- Bench: the Su-30 setup took CgShiftAft fixed at 0.25 instead of the plugin default; it now follows the default.

## Su-30VXN FCS 0.2.1 (9 Nov 2026)
Tom (22:45): the jet is tail-heavy and tips onto its tail while taxiing (22:41 log: slow full-pedal turn at 19 km/h, nose to 11 deg);
proposed CG 0.15 / aft 0.25 / LERX 1.6 / canards 1.5 and asked whether it keeps the stability margin; and whether fuel burn moves the CG
(if so, change it).
- CgShiftAft 0.25 -> 0.15, CanardEffectiveness 1.3 -> 1.5 (Su30DevRevision 10 moves a file still holding 0.25 / 1.3 once). Bench static
  pitch (350 km/h, surfaces 0): 5-10 deg slope -1.56 -> -1.63 deg/s^2 per deg, 20-45 deg within 2.4 deg/s^2, 60-75 within 0.8. Static
  nose-wheel share 9.6 -> 11.3 %. Tom's set (aft 0.25, LERX 1.6) matches at low AoA but has 4-7 deg/s^2 less nose-down at 60-75 deg.
- FuelCgHold / FuelCgHoldRef (6. Fuel, true / 0.7): the fuel balancing holds the fore/aft CG at the clean jet's 70 %-fuel point (the
  bench calibration point) whatever fuel is left, down to about 10 % fuel. Stock spread: the CG walks ~30 cm aft from full to empty.

## Su-30VXN FCS 0.2.0 (9 Nov 2026)
Tom (21:28): +70 / -65 and FallingLeafYawBoost 1.2 as defaults; corkscrew stop shouldn't wait on sideslip once it is past the margin;
smoother push-overs; soft AoA limit (forced past it = MPO); hitch in rolling pulls above ~300 kt. (21:48-21:51): port the F-22E 3.2.1
threading and perf work, bump to 0.2. (22:14): work with kyleisred's Su-30VXN 1.0.1 and don't let its CG change affect the FCS.
- Defaults: AoALimitNegative 45 -> 65, FallingLeafYawBoost 1 -> 1.2; Su30DevRevision 9 moves a file still holding 75 / 45 / 1 once
  (AoALimitPositive 75 -> 70).
- GCaptureMeasured (2. Envelope, true): the load-limit capture plans its stop on the measured pitch braking (as CaptureMeasured) instead of
  an AoA-rate lead closed at 3 /s. Bench: pitch-command dip in full aft + full roll at 520-650 km/h 11-20 -> 6-8 deg/s.
- SoftAoALimit / SoftAoALimitMargin (2. Envelope, true / 3 deg): forced more than 3 deg past a limit, the jet is flown by the MPO law until
  back inside for 0.25 s; a nose-up clamp stops full aft stick pulling it further past the limit.
- StopIgnoresSlipOutsideMargin (3. Lateral-directional, true): stick released with the sideslip past RollStopSideslip (or about to be),
  roll and yaw both brake at full; the roll isn't held for the yaw and the sideslip hold stands aside.
- PitchTrajMeasuredEnd (4. Gains, true): the pitch-rate trajectory eases into its target over the effectors' measured swing time
  (at least the old 0.15 s).
- Threading / perf from F-22E 3.0.4-3.2.1: EstimatorThreaded, ParallelTables, ParallelAllocator, ParallelSweep, ParallelEnvelope,
  TickTimerLog (0. Debug, all true). Bench: identical output, main-thread tick ~2.05 -> ~0.79 ms.
- Su30CgReference (5. Airframe, true): on the Su-30VXN 1.0.1 airframe (500 kg nose cone) the eight parts it re-massed get their 1.0.0
  masses back (UnitPart.ModifyMass), so the FCS flies the 1.0.0 mass layout (same CG and inertia). 1.0.0: no effect.

## Su-30VXN FCS 0.1.16 (9 Nov 2026)
Tom (20:47-20:50): TVC roll only when the sideslip stiffness is like a brick; in MPO always (regardless of the setting), velocity-vector
roll while the airframe still has enough stiffness; TVC roll should forget about sideslip.
- Nozzle-axis roll band gated by the probe's slipStiff (filtered ~1 s) instead of IAS: main keys LowSpeedTvcRollStiffStart 0.4 /
  LowSpeedTvcRollStiffFull 0.2 deg/s^2 per deg, only past 25-35 deg of total flow angle off the nose (AoA and sideslip together), and always
  when flying backwards (|AoA| 85-100 deg and up). Falls back to the IAS keys until the probe's sign has latched (first flight below 10 deg AoA)
  or with RollStopSlipProbe off. The band moves at most 2/s.
- MPO: the same gate decides regardless of LowSpeedTvcRoll (MpoTvcAxisRoll); the old past-the-AoA-limit trigger is used only without the probe.
  LowSpeedTvcRoll now switches the band for the AoA law only.
- Bench: Plant.CreateFcs latches the probe sign from BENCH_SLIPSIGN; replays latch it (+1).

## Su-30VXN FCS 0.1.15 (9 Nov 2026)
Tom (20:19): "measure how much pitch decel and accel the jet has so it could stop on the aoa limit right on time without being too
slow or too fast then overshoots" (no gain change). (20:25): in the low-speed TVC roll mode, roll as the stick says, even flying
backwards.
- CaptureMeasured (dev, 2. Envelope, true): the AoA-limit approach plans its stop on the measured pitch acceleration plus the
  effectors' reachable swing with the present roll/yaw use kept (PitchReach), corrected to the limit by the live sweep, with each
  effector's swing time and the command-path lag as the delay. Replaces the modelled-braking blend, x0.7, 3/s cap and roll-coupling cut.
  Telemetry: capTauDn, capTauUp, capBrkDn, capBrkUp.
- LowSpeedTvcRoll band: the nozzles' differential follows the roll stick directly in body axes (left stick = left nozzle jet up,
  right jet down), never flipped by the reversed-flow roll sense; with Stability Assist the pitch part keeps its share for the AoA limit.
- Bench: replay flies MPO on the log's MPO rows (Plant.BenchMpo public).

## Su-30VXN FCS 0.1.14 (9 Nov 2026)
Tom (19:43): CG 0.30 tipped the jet up briefly at spawn; keep the in-flight margin with CG 0.25. (19:49) "Build it then".
- CgShiftAft 0.30 -> 0.25, CanardEffectiveness 1.2 -> 1.3 (Su30DevRevision 8 moves a file still holding the old value, once).
  Bench static pitch (350 km/h, surfaces and nozzles 0): within ~1 deg/s^2 of 0.1.13 at 0-75 deg AoA (5-10 deg slope -1.56 vs -1.53).

## Su-30VXN FCS 0.1.13 (9 Nov 2026)
Tom (17:02): Buffed variant 12 % more thrust instead of 8, 35 % less fuel burn instead of 25.
- ThrustScale 1.08 -> 1.12 (dry and afterburner), FuelConsumptionScale 0.75 -> 0.65 (Su30DevRevision 7 moves a file still holding
  the old value, once). Buffed profile only; Stock unchanged. Multiplayer: the host's values, as before.

## Su-30VXN FCS 0.1.12 (9 Nov 2026)
Tom (16:51): "build this one" - CgShiftAft 0.30, aft body lift 0.5, LERX 1.5, canards unchanged.
- New dev key AftBodyLift (5. Airframe, 0.5): lift-area multiplier on the aft body parts (Left/Right_intake and engine cover, L-/R-engine,
  Su_30_tail). LogOutput: "aft body lift x0.50 on N part(s)".
- Defaults: CgShiftAft 0.25 -> 0.30, LerxEffectiveness 1.15 -> 1.5 (Su30DevRevision 6 moves a file still holding the old value, once).
- Bench: the Su-30 branch now flies LerxEffectiveness from the config default (the shared Ifrit defaults had held it at 1.0 since 0.1.3,
  against the game's 1.15); AftBodyLift is the game key (Names.IsAftBody), CgAftShift 0.30.

## Su-30VXN FCS 0.1.11 (9 Nov 2026)
Tom (11:28): AoA past the limit, and inverted corkscrews stop with no airspeed, then swing into a tumble. (11:47): a main-config mode that
forgets sideslip when too slow and turns about the canted nozzles' axis; keep the AoA limit; same in MPO without it.
- GyroBudget (dev, 0.25): below 170-130 km/h past 35-45 deg AoA the pitch-rate command is held so the gyroscopic roll (Iyy-Izz)/Ixx q r stays
  within this share of the roll authority. Telemetry column gyroQLim.
- LowSpeedTvcRoll (main, off), LowSpeedTvcRollStartKmh 150, LowSpeedTvcRollFullKmh 120: sideslip hold and roll governor fade out, roll stick
  turns about the nozzle axis, in the AoA law (AoA limit stays in force at full aft: no stick cancel, no zero-rate hold) and in MPO.
- Bench: CgAftShift 0.25 default, surfaces / nozzles posed, camber hosts found, flap centre-of-lift and thrust-line calibration (CALIBRATION_su30.md).

## Su-30VXN FCS 0.1.10 (9 Nov 2026)
Tom (11:18): "with cg aft 0.25 it is much better. migrate settings and the closed loop nose down while taxiing is no longer needed,
delete that".
- His dev values are the new defaults (Su30DevRevision 5 moves a file still holding the old default, once): CgShiftAft 0 -> 0.25,
  AoALimitPositive 75 -> 70, SlipTvcAssist true -> false, SlipGainHighAoA 4 -> 0, CorkscrewSlipTol 0 -> 5,
  AsymFeedForwardMaxSideslip 6 -> 12. Game-log fit (9 Nov, 52-66 deg AoA): above ~350-400 km/h at 55-62 deg the airframe's
  nose-down was larger than all the pitch authority; the CG shift adds nose-up in proportion to the wing's normal force.
- Removed NoseWheelHold (0.1.1, dev keys NoseWheelHold / NoseHoldLoad): the nose-leg load integral, the attitude term and the
  0.1.4 thrust feed-forward (already off with ThrustLineAtCg). Telemetry columns noseHold, noseLoad, noseThetaG, noseFf gone.
  Bench ground roll from 0 / 100 / 200 km/h at full throttle, stick neutral: attitude within 0.1 deg until the nose unloads
  at ~350 km/h.

## Su-30VXN FCS 0.1.9 (9 Nov 2026) - test build, flown by Tom before the bench is recalibrated
Tom: pitch-up hits a wall at 50-65 deg AoA; the falling leaf struggles to hold 75 and is rough in all 3 axes; canards nose-up
at high AoA (right or wrong?); airbrake still tied to the throttle; RollPaceRateBlend 0.40 by default.
- HighAoACanardPitchFirst 1000 -> 0 (dev; Su30DevRevision 4 moves a file holding 1000 once). It acted as a relay on the pitch
  error and cycled the leaf at ~1.3 Hz (09:59 log). Bench leaf: holds 75, nozzle swing 5.2 -> 1.0 deg rms.
- PitchWallSurfacePeak (dev, on; replaces 0.1.8's CanardPitchPeakWindow): in the pitch wall the canards go to their raw pitch
  peak and the stabs stay within 8 deg of theirs. With roll / yaw stick in, each stab is kept where its raw nose-up is within
  WallLateralGoodFrac (0.45) of its best, in the run whose roll + yaw moment matches the lateral demand (10:25 log, 197-198 s:
  the right stab sat at +2..+11 deg and the AoA fell 65 -> 45). Bench, full aft + roll at 60-65 deg from 520 km/h: AoA dip
  4-7 -> 0.1-0.5 deg, bank after 3 s ~10 % less.
- GCaptureReachable (dev, on): the G-limit capture only brakes as far as the limit is reachable (nMaxAero); not with lateral
  stick (GCaptureReachableRolling false). Bench 450 km/h pull: pitch-command dip 10.7 -> 0 deg/s.
- ThrottleIndependentAirbrake default true; RollPaceRateBlend default 0.65 -> 0.40. A 0.1.0-0.1.8 player file is moved once
  (Su30ConfigRevision); a hand-set RollPaceRateBlend is kept.
- Known on the bench (not fixed): full aft + roll from 350 km/h corkscrew peak sideslip 8.5 -> 11.4 deg; new AoA dips in two
  full-aft rolling pulls (350 / 600 km/h, up to 7 deg). The bench is wrong in size and sometimes sign at 50-70 deg AoA (see
  Su30_bench_handoff.md).
- Telemetry: gcReach.

## Su-30VXN FCS 0.1.8 (9 Nov 2026)
Tom: the TVC still spends a lot of deflection holding pitch; a corkscrew from high speed stops in pitch and roll at ~55 deg AoA,
then gets going again; the stabs and canards should start the pitch rate at low to mid AoA so the TVC can do the lateral.
- The mod's CanardSurface AoA schedule (hinge leading edge down 1:1 with AoA to 30 deg) had never been switched off: the search
  looked only under the Aircraft, and Blueprinter's split physics bodies put the canard struct elsewhere (every LogOutput: "0 canard
  AoA-schedule script(s) found"; every Su-30 log: canard flow angle at zero deflection max(0, AoA - 30)). The scene's CanardSurface
  scripts are now searched, matched by their aircraft field, and retried while the FCS flies. StockCanardSchedule (dev, false) keeps
  the stock schedule. The 0.1.5 drag-shape pitch balance was solved on a bench without the schedule, so it did not hold in game.
  Bench: Plant models the schedule with CANSCHED=1.
- SurfacesFirstPitch (dev, 30; SurfacesFirstMaxAoA 35, SurfacesFirstIasLow / High 350 / 450 km/h): the nozzles' common deflection
  costs this much at every pitch rate and error, so the stabs and canards carry the pitch first. Bench, full-aft pulls / corkscrews
  from 450-600 km/h: nozzle common at 5-45 deg AoA 1.7-6.2 -> -0.1-1.1 deg; velocity roll in the wall from 600 km/h 36 -> 50 deg/s.
- CanardPitchPeakWindow (dev, on): in the pitch wall (40 deg AoA to limit - 5, above 300 km/h, pitch rate > 10 deg/s short) the
  canards stay between their own raw pitch peaks instead of parking at -45 in deep stall (bench, 450 km/h: -20..-37 is 14-43 deg/s^2
  more nose-up at 55-70 deg). Removes the pause in the 550 km/h full-aft corkscrew (velocity roll 43 -> 53 deg/s mean).
- Tried, not kept: canards planned pitch-only (cost ~8 % low-AoA roll; the 'nose-down' canards in the log are mostly the allocator
  trimming the rolling stab split); a 5 deg nozzle reserve for roll in the wall (slower pitch, no roll gain).
- Regression set: roll-then-pull-rev 400 sideslip 16.6 -> 8.6; pull-then-roll 320 / 400 7.1 / 8.2 -> 4.4 / 5.4; pull-then-pushover
  + roll 350 7.0 -> 9.1 (worse). Sweep: stop worst 6.7 -> 8.2, reversal worst 13.4 -> 10.8.
- Telemetry: surfFirst, canPeakLo, canPeakHi.

## Su-30VXN FCS 0.1.7 (9 Nov 2026)
Tom: bench a pitch-neutral airframe at high AoA so the nozzles stop holding pitch and both can brake yaw; max AoA 75 if possible.
- AoALimitPositive 70 -> 75 (a dev file still holding 70 moves once; Su30DevRevision 3). Bench, 75 against 70: full-aft pulls
  from 150-250 km/h hold 75.0-75.1; regression set equal or better (65 deg roll releases 7.0 / 5.9 -> 3.7 / 2.8 deg of sideslip);
  stop / reversal sweep unchanged; corkscrews mostly lower peak sideslip (full aft 250 / 350: 14.6 / 12.8 -> 9.6 / 7.5; pull 0.8 at
  250: 13.3 -> 16.5) with velocity roll 3-6 deg/s slower; falling leaf 42-43 -> 39 deg/s at ~12 km/h less IAS; reversals at the limit
  end ~25-30 km/h slower. A full-aft rolling pull entered ~200 km/h passes the limit to ~82 deg (72 at the 70 limit) through
  StickCancelsLimit, with the same ~17 deg of sideslip.
- Pitch-neutral airframe NOT adopted. Candidates (more canard and LERX lift, LERX vortex models) took the nozzles' pitch trim at the
  75 deg hold from ~10 deg to 1-5 deg and let a stop reach the full +-15 differential, but the falling leaf only went 39 -> 41-42
  deg/s (the nozzles are ~40 % of the yaw authority at leaf speed and the yaw-rate limit already counts the travel left), the leaf
  lost 6-20 km/h, full-aft rolling pulls from 150-200 km/h overshot to 83-92 deg AoA, and hold-then-roll releases at low IAS had
  about twice the sideslip (9 / 17 -> 23 / 35 deg).

## Su-30VXN FCS 0.1.6 (9 Nov 2026)
Tom: high-AoA yaw / velocity roll worse than the Ifrit (no yaw TVC) and no better than the F-22E (pitch TVC only).
- YawThrustShare: the high-AoA yaw braking floor's IAS-decay anticipation (qPredK) now scales only the aero share of the yaw
  authority; the canted nozzles' share (half the differential's yaw range, aYawTvc) is counted in full, as thrust-borne yaw does not
  fade as the jet slows. 8 Nov logs (0.1.4), 215-290 km/h at 55-75 deg AoA, full roll stick: deceleration 22 m/s^2, qPredK 0.47,
  yaw-rate limit 39 deg/s on 70 deg/s^2 of yaw authority (Ifrit 63 on 70, not decelerating); velocity roll 34 deg/s (Ifrit 67).
  The FCS's tables put the nozzles at only 12-23 % of the yaw authority at 55-70 deg / 200-300 km/h (15-17 of 73-98 deg/s^2), so
  the effect is small: bench yaw-rate limit in 350 / 450 km/h rolling pulls 60 -> 63 / 69 -> 70 deg/s; corkscrew velocity roll
  0-6 deg/s faster. Most of the 8 Nov limit cut came from the 0.1.4 drag (qPredK 0.47 / 0.36 there); with 0.1.5's drag the same
  bench pulls read 0.8-0.9.
- Telemetry: aYawTvc, yawThrustShare.

## Su-30VXN FCS 0.1.5 (9 Nov 2026)
Tom: the airframe still likes to pitch up taxiing and taking off (the Raptor, Ifrit and Revoker don't); it loses airspeed comically
fast as AoA rises - make its drag curve halfway between the Ifrit's and the Revoker's; fix the CG above the thrust.
- ThrustLineAtCg (dev, on): JetNozzle.Thrust's force point moved, along the jet's vertical axis only, to the CG's height (the
  Su-30VXN's nozzles sit 0.49 m below it; F-22E / Revoker within ~0.1 m, Ifrit 0.65 m above). The FCS's nozzle models use the same
  arm. NoseThrustFeedForward then stays at 0. Bench take-off, full AB, stick neutral: nose-wheel load 26-32 kN (static ~30) with the
  nozzles at 0 and no hold action (0.1.4: nozzles at -7 deg; 0.1.3: nose off at 0.5 s).
- HighAoADragShape (dev, on; HighAoADragShapeDepth 1): the drag curves of the Su-30's own wing, body and canard foils scaled against
  each part's AoA, x1 to 17.5 deg; wing/body ~0.48 and canards ~0.73 at 40-50 deg, 0.64 / 0.74 at 70. Target: total drag per kg halfway
  between the Ifrit's and Revoker's (7-8 Nov logs; the Su-30 was 1.5-2x them from 30 deg up). The two curves are solved together so
  the static pitch moment at every AoA is unchanged (the canards' drag is nose-up, the wing's nose-down; one factor on all left the
  jet 5-8 deg/s^2 more nose-down at 45-90 deg). Stabs (F-22E stab curve) and fins unchanged; lift unchanged.
  Bench: CD*S x0.97 / 0.72 / 0.59 / 0.64 / 0.69 / 0.77 at 20 / 30 / 45 / 60 / 70 / 90 deg; pitch within 0.2 deg/s^2.
- Bench cmsweep prints CD*S and CL*S; the dump has per-part drag and lift.

## Su-30VXN FCS 0.1.4 (8 Nov 2026)
Tom: at least 65 deg/s in a falling leaf; a pitch-neutral airframe toward mid / high AoA so the TVC doesn't spend itself on pitch;
canards to take more of the pitch from the TVC; HighAoARollRateCapFloor 0; more sideslip at mid to high AoA; a hitch at mid-way AoA
in rolling pulls; the nose pitching up, bouncy and floaty on the take-off roll; flaperons roll first like the Raptor's.
- Rolling-pull hitch: TradeCapTau 0.05 -> 0.45 s (the roll governor's trade reads the yaw capacity through this low-pass). The capacity
  dips while the effector carrying it crosses its travel; on this jet that is the canted nozzle pair (30 deg at 65 deg/s = 0.46 s), not
  the F-22E's 0.05 s rudder transit. At 0.05 the trade followed each dip: the pitch-rate target fell 43 -> 23 deg/s through 15-25 deg
  AoA and came back to 60 (bench, 350 km/h full aft + full roll; 8 Nov 00:32 log 209.6-210.4 s: 31 -> 22 -> 42). Bench at 0.45: the q dip
  through 15-30 deg is 5-7 deg/s (16 before) at 280 / 350 km/h, the lag behind the target 4-7 deg/s (13-17). Cost: +1-3 deg of peak sideslip in some rolling pulls.
- VvWeightPitchGate: HighAoAVvRollWeight steps back to 1 while the pitch lags its target by 2 -> 8 deg/s (the log showed the nozzles at
  full differential with q falling behind a rising target). The bench did not reproduce that case; no effect there.
- HighAoACanardPitchFirst (dev, 1000): with roll stick in past 25 deg AoA the nozzles' common deflection is drawn toward neutral (over
  the roll-time budget's open band), so the canards and stabs carry pitch first; a pitch error over 1-5 deg/s lets go.
- FallingLeafYawBoost (dev, default 1 = off): x the yaw-rate limit past 55 deg AoA, 115-250 km/h. Bench: the falling-leaf rate it adds
  is rotation the nozzles cannot stop. 1 / 1.12 / 1.25 / 1.55: 39 / 43 / 49 / 64 deg/s steady (350 km/h entry) at the 75 limit (~160 km/h), roll
  released at 70 deg: 10 / 15-18 / 21-25 / 38 deg of sideslip.
- HighAoARollRateCapFloor 0.7 -> 0 (Tom; dev file moved once if still 0.7; Su30DevRevision 2). RollStopSideslip 6 -> 10 (player key,
  moved once from a file still at the old default 6).
- Take-off: NoseThrustFeedForward. The thrust line runs 0.49 m below the CG, so the run's acceleration pitches the jet up about its
  mains; the nozzles take the angle that cancels it on the ground (asin(height / (arm x cos 32)), throttle-independent, ~5.5-7 deg),
  open loop; the load hold trims the rest with NoseHoldKi 0.8 -> 0.4 (the 8 Nov logs: nose load cycling 0 <-> 1.3 at ~1 Hz, then the
  nose floating off at 46-51 m/s and the gear-down rate law taking over with the nose 2-3 deg up). Bench: nose off at 0.5 s and held at
  ~50 % load -> never off, 25-36 kN (static 30) early in the run.
- FlaperonRollPriority (dev, 1): the game's gear-driven 15 deg landing flap on the flaperons comes up in the air with lateral stick or a
  running roll command (the FlapRollBias schedule), at the flap's servo rate; on the wheels it stays. Not benched (no landing flap in
  the bench).
- Pitch-neutral airframe: benched, not made default. At 70-75 deg the 0.1.3 airframe already holds with the nozzles at ~0-1.5 deg; the
  nozzles' pitch use in the logs is at 15-55 deg in pulls. The neutral set (CanardEffectiveness 1.7, LerxEffectiveness 3, StrakeVortexLift,
  breakdown 50/89, residual 2, DownwashFadeEnd 90) cost speed (IAS at the end of a 350 km/h rolling pull 148 -> 131 km/h) and roll
  rate, and made the falling leaf slower (37 -> 34 deg/s); canards 1.45 alone was mixed.
- Telemetry: landFlap, canPitchFirst, leafBoost, noseFf.

## Su-30VXN FCS 0.1.3 (8 Nov 2026)
Tom: LERX 15 % and canards 20 % more effective; the jet went past the AoA limit to ~110 deg in the latest log.
- LerxEffectiveness 1.15 and CanardEffectiveness 1.2 (dev defaults, moved once in an existing dev file still at 1; Su30DevRevision).
  The LERX is Cheek_L/R (0.7 m2 each, beside the cockpit, carries the canards); it now maps to the lineage's shoulder-LERX role.
  Bench: low-AoA static margin 0.41 -> 0.31 m; at 70 deg / 190 km/h the best-surface nose-down 17 -> 12.6 deg/s^2.
- The roll governor's sideslip-margin model fades out StopSlipLimMargin (5) past the AoA limit instead of from a fixed 70 deg. With
  AoALimitPositive 75 it was off at the limit itself (8 Nov 00:19 log: margin 0 at 75 deg, sideslip 16 deg, then a yaw departure at
  73 deg / 72 km/h that turned into the 114 deg excursion with the stick held full aft). Bench, 75 limit, 250 km/h corkscrew:
  33 deg/s / 10.9 deg sideslip -> 37 / 7.8. No change at the 70 limit.

## Su-30VXN FCS 0.1.2 (8 Nov 2026)
Tom: high-AoA yaw/roll authority worse than the Raptor and Ifrit FCS; is the jet too stable; the TVC spends a lot holding the AoA.
- HighAoARollRateCapFloor (dev, 0.7): the roll governor's rate cap (a model of the rudders) now counts in proportion to the rudders'
  share of the yaw capacity (rudders + canted nozzles, full from 50 %), down to this floor. The Su-30 inherited the Revoker's 75-85 deg
  rudder fade (F-22E / Ifrit: 50-60, which drops the cap past 60 deg), so at 70 deg the cap counted in full on the rudders' ~3 deg/s^2
  at 160 km/h against the nozzles' ~23: velocity roll flat at 23-26 deg/s from 110 to 400 km/h in the 7 Nov logs (Ifrit 25 -> 67,
  F-22E 20 -> 40). 0 drops it like the F-22E / Ifrit: the bench corkscrew at 250 km/h then reached 47 deg/s with 14 deg of sideslip,
  so 0.7 (36 deg/s, 5.3 deg); 1 = 0.1.1.
- CgShiftAft now works on the Su-30 (Front_cover -> L/R-engine); default stays 0.
- Telemetry: rudShare, rateBlendEff.

## Su-30VXN FCS 0.1.1 (8 Nov 2026)
From Tom's first flights (7 Nov 23:23 / 23:24 logs): the velocity roll and yaw at high AoA are slow, the nozzles seem to do only
pitch, and the tail drops onto the runway on the take-off roll with no pitch input.
- Sideslip probe counts the canted nozzles: the roll governor's sideslip-tolerance probe (RollStopSlipProbe) summed only the surfaces'
  yaw and roll. Past ~60 deg AoA, where the stabs' roll falls off with sideslip and the nozzles carry most of the lateral authority, it
  read the authority as halved by 4 deg of sideslip and cut the roll governor's margin from RollStopSideslip 6 to 2.8 deg. The nozzle
  differential (thrust, independent of sideslip) is now in both sums (SlipAuthTvc).
- HighAoAVvRollWeight (dev, 4): the allocator's velocity-roll weight rises from 1.5 to x4 from 25 to 45 deg AoA, while the nozzles have
  thrust and the roll stick (or coupled pedal) is in. The canted differential is mostly a velocity-roll effector up there and shares
  each nozzle's one axis with pitch; at the old weights both nozzles stayed at full nose-up through a rolling pull (23:24 log,
  67.8-68.9 s: 15 / 15, differential 0, roll and yaw 15-45 deg/s^2 short). A released stop keeps the 0.1.0 weights.
- NoseWheelHold (dev, on; NoseHoldLoad 0.5): in the open-loop ground mode, stick at or forward of neutral and the mains down, the nose
  leg's spring force is held at no less than half its static load (measured at a standstill, else from the CG and gear positions)
  through the stabs, canards and nozzles, with an attitude term for a nose that has come up. The thrust line is 0.5 m below the CG and
  the CG only 0.74 m ahead of the mains, so full afterburner nearly unloads the nose wheel (23:23 log: nose up at 80 km/h with the
  stick centred, tail strike at 13.5 deg). Pulling the stick lets go of it at once.
- Ready log reads "Raptor stab curve" when RaptorStabFoil is on (it said "stock foil").
- Telemetry: new columns noseHold, noseLoad, noseThetaG, cantVvW.

## Su-30VXN FCS 0.1.0 (7 Nov 2026)
New mod (`Su30_VXN.FCS`), forked from FS-3 Ternion FCS 0.1.8 (the Compass / Revoker / Ifrit / F-22E lineage, without the F-22E's
multithreading). Kyleisred's Su-30VXN Gandiva (Blueprinter, SolarSu30.dll; aircraftName "Su-30VXN Gandiva"). Turbofan -> Turbojet.
- Stabs: the F-22E FCS's stab curve (RaptorStabFoil: F22E_Wing remapped to a 25 deg peak, peak CL kept, x1.2 area) and its linear
  downwash (StabDownwash 0.6, fade 45-60); the stabs' stock airflowChanneling (0.6 rad) is held at 0 in the aero job.
- Camber coupling: flaperons 0.63 on the inner wing panels, rudders 0.60 on the fins. No trailing-edge stall stagger.
- Travel: stabs +35 / -30, canards +65 LE down / -45 LE up and symmetric only (CanardSymmetricOnly: paired as a pitch-only pair with a
  prohibitive split cost and the command averaged; the canard settled / attached windows are off for them), flaperons +-25, rudders
  +-35, every surface 100 deg/s. The stock CanardSurface AoA schedule is switched off on the canards while the FCS flies.
- TVC: the AL-31FP's single-axis nozzles canted 32 deg outboard (CantedTvc): +-15 deg each at 65 deg/s, symmetric = pitch only,
  differential = proverse roll + yaw; Turbojet transpiler writes the nozzle transforms. The canted differential counts as a yaw source.
- MPO: velocity-vector roll (no separate axes); past the AoA limit the roll axis blends over MpoTvcAxisBlend (5) deg to the axis the
  canted differential turns the jet about (~39 deg from body x on the bench), with the sideslip hold faded out there.
- AoA limits +70 / -45 (bench); idle-thrust cap 45. Gear-driven landing flaps left to the game; outer flaps fixed (Tom).
- Airbrake: stock dedicated airbrake (ThrottleIndependentAirbrake and TrailingEdgeBrakeConfig false by default, Tom).
- Buffed: thrust x1.08, fuel x0.75, parasitic drag x0.90, flares x10, gun x2, EW x1.5 / x1.2 (as the Ternion).
- Telemetry: Su30_FCS_<time>.csv, new columns mpoAxisW, mpoAxisDeg, nozL, nozR.


## FS-3 Ternion FCS 0.1.2 (7 Nov 2026)
Elevators stall at 30 deg local (ElevatorStallAoA) and x1.2 effective (ElevatorEffectiveness); canards x1.1 (CanardEffectiveness);
shoulder LERX x1.13 (LerxEffectiveness, with its HLD areas) so the static margin stays at 0.1.1's -0.29 m. Rollout aero-brake adds
the elevators fully trailing edge up (GroundBrakeElevator 35, the F-22E's stabs). LateralStabReserve 0.7 -> 0: corkscrew entries
built pitch on 30 % of the canard/elevator authority (no nozzle pitch on this jet); now within ~0.1-0.3 s of a plain pull.
One-time move of the three changed dev defaults (TernionDevRevision 1).

## FS-3 Ternion FCS 0.1.1 (7 Nov 2026)
AoA limits above 45 now fly: the inherited fixed no-TVC caps (NoTvcAoACapPositive / Negative 45) were always on (vectorable T/W
is 0 on this jet) and capped every limit above 45, MPO included. On a jet with no vectorable nozzle (noVecJet) they are off;
AoALimitPositive / Negative and the swept authority cap rule. Defaults unchanged (40 / -40). New dev key ElevatorStallAoA
(0 = stock curve) for the earlier-elevator-stall test; bench: no high-AoA roll gain (roll is yaw-capacity-limited).

## FS-3 Ternion FCS 0.1.0 (7 Nov 2026)
New mod (`Ternion_FS3.FCS`), forked from T/A-30 Compass FCS 0.1.11. FS-3 Ternion (Blueprinter, prefab P_Trisurface1). Canards and
aft elevators are both Stabilator-kind pairs (`isCanard` flag): canard travel 60 LE down / 45 LE up (Revoker), elevators +35/-30,
rudders +-35, flaperon_L/R and rollervon_L/R +-25 (F-22E), all surfaces 100 deg/s. Revoker staggered trailing-edge stall (inboard
flaperons 26, outboard rollervons 45, knee-preserving) with camber coupling (0.62 / 0.66) and rudder camber 0.54. No downwash.
Canard attached-flow window, settled window and rollout brake on the canards only; per-pair trim costs (ElevatorTrimCost);
FlightPathRateFloor; trailing-edge brake set and its lift balance handle control-surface flaperons. Turbojet -> Turbofan.
AoA limits +40 / -40. Buffed: thrust x1.08, fuel x0.75, parasitic drag x0.90, flares x10, gun x2, EW x1.5 / x1.2.


## T/A-30 Compass FCS 0.1.11 (5 Nov 2026)
VMC protection fixed for this jet. The T/A-30's engines have no vectoring transform and the FCS skipped their thrust entirely, so
with an engine out the live engine's yaw read 0 and only the last-resort cut (after the sideslip ran away) could act. Their thrust
moment is now taken as it is (fixed nozzles; asymmetric-load model roll/yaw only, engine out only), and a predictive limit
(VmcPredict) brings the throttle back once the live engine's yaw passes 70 % of the yaw authority at the predicted q. Bench, engine
out, full-aft climbs from 220-300 km/h: departures at 25-31 m/s -> yaw held to the top of the climb; 0.25 pull at 180/200 km/h:
sideslip 53/35 -> 0.6/1.9 deg. Both engines running: identical. Note: the FCS's T/W (tw) reads 0 on this jet for the same reason;
not changed (it moves several laws).

## T/A-30 Compass FCS 0.1.10 (5 Nov 2026)
The inner flaps no longer move with the pitch stick, in the air or taxiing (FlapPitchLock): their common deflection is held to the
schedule (landing droop, airbrake / rollout sets), only the differential is free for roll, and the droop comes off when the
differential needs the travel. The ground law's pitch-stick throw on them is gone. With a tail slab shot off the lock lets go (the
flaps are then part of the last nose-down). Clean jet: rolls identical; 5-10 % less pitch rate at a push onset and pulling out of a
push. Damage matrix: same 1 departure; outer-wing cases 14.4 -> 12.4 deg peak AoA.

## T/A-30 Compass FCS 0.1.9 (5 Nov 2026)
The asymmetric-load cap comes down faster when it has far to go (AsymCapGapTighten: tighten rate x gap / 3 deg, 1-4x), so a pull
right after a part is shot off no longer outruns it. Bench, outer wing shot off 0.9 s before full aft: 6 of 8 depart -> 0 of 8;
damage matrix, clean jet and stores identical. 5 Nov 21:16 log: the 0.1.8 gate held the 7 deg cap; the 180 s departure was the
speed bleeding to 45 m/s at idle (minimum speed with this damage ~100 m/s).

## T/A-30 Compass FCS 0.1.8 (5 Nov 2026)
The asymmetric-load pitch protections (pitch cap and bias, pitch priority, stab roll price, held coupling gate) also open when a
control surface is lost with the cap in force (AsymDamageGate); the lateral CG offset they were gated on swung -0.28 <-> 0.00 m
with a hanging rudder and dropped the pitch cap 7 -> 12 deg (5 Nov 20:25 log, departures at 193 and 217-223 s). Negative-AoA
mirror of the bias and a negative pitch cap (AsymPitchBiasNeg, floor -4). PushPlanStopsRoll off: a held roll is no longer
stopped through a push-over (Tom 20:51). Bench, 133 damaged cases with the CG-offset gate shut as in game: 27 -> 2 departures;
clean jet identical.

## T/A-30 Compass FCS 0.1.7 (5 Nov 2026)
Under the asymmetric-load AoA cap without pitch TVC, the nose-up the cap's model leaves out is measured in flight (AsymPitchBias:
measured pitch less the stabs' share less the modelled airframe, peak-held; with a stab gone also the stabs' live nose-down end vs the
sweep) and taken off the pitch cap's nose-down margin, the capture braking at the cap and the roll-coupling limit; the pitch cap may
come down to 7 deg (AsymPitchCapFloorNoTvc; 10 before). 5 Nov 19:18 log: right outer wing, left fin and left stab gone, the cap's
model read 25-29 deg of room with every pitch surface out of nose-down at 12, AoA 12 -> 46. Bench, 133 damaged cases: 23 -> 1
departures; clean jet identical.

## T/A-30 Compass FCS 0.1.6 (5 Nov 2026)
Under the asymmetric-load AoA cap without pitch TVC: the AoA capture plans its stop with half the modelled nose-down at the cap and
no lean toward the authority available now (AsymCapBrake); the stabs' roll split is priced when the AoA reaches the cap and pitch is
short (AsymStabAntiCost 0 -> 0.1). 5 Nov 19:04 log: right outer wing gone, push to -8 then full aft (no roll) came through zero AoA at
30 deg/s, the stabs went to holding the lost wing's roll, AoA 12 -> 47, departure. Bench, outer wing detached: 0 of 20 depart
(0.1.5: 1, plus a 19 deg overshoot).

## T/A-30 Compass FCS 0.1.5 (5 Nov 2026)
Roll-biased flaps (FlapRollBias, dev, 1): in the air the flaps' high-lift droop comes off with lateral demand so the pair rolls with
its full +-20 differential (F-22E flaperon behaviour). Roll governor on the stabs' nose-down margin without pitch TVC
(StabRollGovernor): the 5 Nov 18:52 log departed after the right outer wing was lost (full aft + roll at 90 deg/s, the roll's
inertial pitch-up took the last nose-down, AoA 12 -> 24 -> 47). Bench, outer wing detached: no departure (0.1.4: 2 of 6).

## T/A-30 Compass FCS 0.1.4 (5 Nov 2026)
Low-AoA roll-rate ceiling: RollRateGainLowIAS 1.2 -> 2.0, RollRateGainHighIAS 1.1 -> 1.8, RollAuthorityUse 1 -> 2 (dev file moved once,
CompassDevRevision 4). The inherited F-22E roll reference and the FCS's steady-roll estimate held full-stick rolls to ~105 deg/s at
300 km/h with the ailerons at 6-7 deg (5 Nov 18:12 log). Bench 1 g: 142 -> 247 deg/s at 300 km/h.

## T/A-30 Compass FCS 0.1.3 (5 Nov 2026)
Tail slabs (elevator_L/R, hstab_L/R) fly a swept-slab curve (StabSlabFoil, new StallStagger family Slab: stall 30 deg, peak CL 1.22,
3.5 /rad) instead of the wing's 17 deg foil; LerxEffectiveness 3 -> 3.4 (dev file moved once, CompassDevRevision 3). Without pitch TVC
the roll-coupling limit is on from 3 deg AoA whatever the stick (CoupNoTvcAlways, floor 12 deg/s^2). Pitch-rate trajectory uses 1.0x the
authority estimate (PitchTrajAuthFrac; 0.7 before). From the 5 Nov 17:41 log: both elevators at the nose-down stop in a 100 deg/s roll at
70 m/s with flaps out, AoA 10 -> 46; slow reversals at +35 / -30.

## T/A-30 Compass FCS 0.1.2 (5 Nov 2026)
Strake vortex breakdown 38-60 -> 30-50 (settings StrakeBreakdownStart / End / StrakeResidualCN): strake lift peaks ~32 deg and falls past the
35 limit; LerxEffectiveness 2 -> 3, CgShiftAft 0.20 -> 0.15. AoA protection: the stick cancels the limit only on a jet with pitch TVC
(FcsController.StickBypass; never on the Compass). Config moves: CompassDevRevision in the dev file (0.1.1's check read the wrong file).

## T/A-30 Compass FCS 0.1.1 (5 Nov 2026)
Elevators +-30; AoA +35/-30; strakes (wingroot1) get a Polhamus vortex-lift curve (StrakeVortexLift, new Strake.cs / StallStagger family)
at 2x area; CgShiftAft 0.27 -> 0.20 (low-speed roll inertia coupling no longer pins both elevators nose-down); CanardAttachedWindow off;
MPO VelocityVectorRoll by default; one-time move of 0.1.0's defaults (CompassConfigRevision). Bench: Defaults() restores CgShiftAft /
LerxEffectiveness (0.1.0's closed-loop AoA sweeps had run without the CG shift); chart remap on the stock-jet path; cparts tool.

## T/A-30 Compass FCS 0.1.0 (5 Nov 2026)
New mod: the Revoker 0.8.2 laws on the stock T/A-30 Compass. AoA +-30, G +9/-3.5; surfaces 100 deg/s; lift-driven tail downwash 0.45;
camber coupling aileron 0.55 / rudder 0.55 / flap 0.40; flaps as allocator effectors with the stock 20 deg landing droop; relaxed
stability by CG 0.27 m aft; Buffed: thrust x1.08, fuel x0.75, flares x10, gun x2, EW x1.5 / x1.2. No Turbojet transpiler.

---
(Revoker history, inherited)
# Changelog

## 0.4.0
- **The allocator weighs the lateral residual in the velocity frame** (`SlipFrameAlloc`): the error about the velocity normal (dp sin a - dr cos a) is the sideslip acceleration itself, the error about the velocity vector is stability-axis roll. The roll weight becomes the velocity-axis weight and the yaw weight the velocity-normal one; the body-frame high-AoA roll fade is bypassed while it is on. Identical below ~10 deg AoA, swapped by 70. Bench: full-aft reversal at 320 km/h 4.1 -> 1.8 deg of sideslip, the 65 deg stick-release cases 5.4/5.7 -> 1.8/1.8, steady rolls below 45 deg unchanged.
- **The lateral governor's rate cap is a forward model of the reversal** (`RevSim`): trajectory, rate loops, a least-squares allocation over every effector, the probe's stiffness and damping, beta-dot kinematics, bisected for the largest rate that stays inside the margin. Replaces the kinematic formula over a hand-picked yaw-effector set (which left out the trailing edge - the dominant yaw axis above ~45 deg on this jet). 22 us/tick.
- **`RollStopSideslip` 4 -> 8**, once (`GovernorRevision`; a hand-set value is kept).
- Yaw-damping credit from the probe (`slipDamp`, `dampCredit`), `PaceStiffCredit`, `SweepMarginGate` (were to be 0.3.1, never released).
- Bench: falling leaf at 70 deg AoA 82 deg/s at 2.5 deg of sideslip (0.2.22: 71 at 11.3; 0.3.0: 41 at 3.0). Corkscrews 62-71 (0.2.22: 72-81, 0.3.0: 34-51).
- Bench-only. Not flown.

## 0.2.22
- **Knee-preserving elevon stall stagger.** The stagger now moves only the plateau above the wing foil's 17.2° knee; below it the curve is unchanged and the peak CL (1.10) is kept. 0.2.20–0.2.21 compressed the whole curve, which dragged the knee down to 12–14° and cost the inboard pair roll authority at 10–15° plus some lift.
- **Inboard reversal 37 → 26°** (`InnerElevonReversalAoA`, migrated once from exactly 37). Outboard stays 45°. Model (both pairs + camber, measured arms): worst-case velocity-vector roll authority over 10–55° AoA 0.26 / 0.18 (small / full deflection) vs 0.15 / 0.07 in 0.2.21 and 0.03 / 0.01 unstaggered; unchanged at 5–15°.
- `ElevonStaggerLiftSlope` retired (the peak CL is always kept). Takes effect on the next spawn.
- Compiled only. Not flown.

## 0.2.21
- **Inboard elevon reversal 32 → 37°** (`InnerElevonReversalAoA` 37; a dev file holding exactly 32 is moved once). Outboard stays 45. Reversal: inboard ±37.2 (±36.0 at full deflection), outboard ±44.8 (±40.7 at full). Takes effect on the next spawn.
- Compiled only. Not flown.

## 0.2.20
- **Staggered elevon stall:** the inboard elevons now reverse for velocity-vector roll at ±32° AoA and the outboard at ±45°, instead of both at ±45° together (`ElevonStallStagger`, `InnerElevonReversalAoA` 32, `OuterElevonReversalAoA` 45, `ElevonStaggerLiftSlope` 1). Each pair flies its own copy of the wing foil with the peak moved (game aero and FCS models alike); the inboard pair's camber share on wing2 is now a force on the inboard curve so it reverses with the elevon. Combined elevon roll authority at 45° AoA goes from 0.016 to 0.059 (×3.7, small-deflection model); the low point anywhere 25–50° goes from 0.016 to 0.059. Takes effect on the next spawn.
- Compiled only. Not flown.

## 0.2.19
- **Canard travel +60/−45°:** leading edge down 70 → 60° (`StabilatorRangeTEUp` 60; a dev file holding exactly 70 is moved once). On the runway full push and the rollout brake each go to the 60° stop.
- Compiled only. Not flown.

## 0.2.18
- **Runway canards reach their stops:** full push takes them to their full leading-edge-down travel (70°), full pull to full leading-edge-up (45°), instead of stick × the stock pitch range (~35°). The rollout aero-brake alone also takes them all the way down (`GroundBrakeCanard` 45 → 80, capped by the 70° stop; migrated once). Before, only push + brake together reached the stop.
- The canards' +70/−45 travel now applies from a standstill (geometry sign), not only after 10 m/s of airflow.
- Compiled only. Not flown.

## 0.2.17
- **Canard travel +70/−45°:** leading edge down 80 → 70° (`StabilatorRangeTEUp` 70; a dev file holding 0.2.16's 80 / 45 is moved once). Leading edge up stays 45°.
- Compiled only. Not flown.

## 0.2.16
- **Canard travel the right way round:** +80/−45 — leading edge **down** to 80°, leading edge up back to 45° (`StabilatorRangeTEUp` 80, `StabilatorRangeTEDown` 45; dev files holding 0.2.14/0.2.15's 45 / 80 are swapped once). 0.2.14 had extended the leading-edge-up side. Sign checked from flight data: at +45..70° AoA a +40° canard sits at ~+8° local AoA, at −45..−56° a −40° canard at ~−14°.
- Compiled only. Not flown.

## 0.2.15
- **Push + roll sideslip governor:** looks ahead as far as the lateral loops take to answer (0.33–0.47 s, the same lag model as the sideslip-hold gain) instead of a fixed 0.8 s (`SlipGovLeadFromLoops`, on). In the 30 Oct 23:45 log (0.2.14) the sideslip crossing zero at +11 deg/s on its way back from −3.3° read as +9° 0.8 s out and cut the roll target 79 → 46 deg/s with the yaw nozzle and rudder on their stops; the release 0.35 s later re-built it 46 → 85 and the yaw axis went round again. New telemetry columns `latLag`, `slipLead`.
- Compiled only. Not benched, not flown.

## 0.2.14
- **Push + roll (negative corkscrew) entry:** the sideslip-hold gain now comes from what the rate loops carrying the correction can follow (`SlipGainFromLoops`, on): 1.3 at 0° AoA, ~1.85 at −50°, ~2.3 at 70°, instead of the fixed ramp to 4. In the 30 Oct 22:48 log the fixed 4 through the 0.4 s yaw loop ran the sideslip at ~1 Hz (−3.4 ↔ +3.6°), the yaw rate ±12 deg/s about its command and the roll governor cut the roll twice — the hitch. The ramp is still there under `SlipGainFromLoops = false`. New dev key `ActuatorLagT` (0.07 s).
- **Canards:** travel −80/+45° (leading edge up to 80°; `StabilatorRangeTEDown` 45 → 80, migrated once) — the wrong side, corrected in 0.2.16.
- Compiled only. Not benched, not flown.

## 0.2.10
- In-flight brake surfaces close with AoA the way the F-22E's speedbrake does. They are fully out to 15° |AoA|, close linearly until they are stowed at 35°, and reopen as the AoA comes back. Dev key `TrailingEdgeBrakeAoAFade`, on by default.

## 0.2.9
- **In-flight brake surfaces:** move as fast as the actuators allow. The outboard elevon share is solved every frame for zero added lift, with the canards' re-trim counted, so deploying and retracting doesn't upset pitch, roll or yaw. Bench, 400–1200 km/h: nz 0.97–1.11 while deploying, pitch rate under 0.2 deg/s, sideslip 0°.
- **Take-off:** the nose wheel now lifts 0.14–0.2 s after a pull instead of 0.7–0.85 s (`GroundRotationRamp` 60 → 600, migrated once). A short pull no longer fails to rotate the jet.

## 0.2.8
- **`TrailingEdgeBrakeConfig`** (player config, on by default): F-22E-style brake surface sets.
  - **In the air:** inboard elevons down, outboard up.
  - **On the runway:** every trailing edge up, canards leading edge fully down.
  - **Braked touchdown:** the nose is lowered at 4 deg/s, then the full rollout set comes in.
- Needs `ThrottleIndependentAirbrake` on.

## 0.2.7
- **No-TVC rolls at the AoA cap:** smoother. Roll weight 1, and the canards are kept in their attached-flow range.
- **Canards and negative AoA:** canard travel ±45°, negative AoA limit 55°.
- **Fin lost:** 30° AoA limit without nozzle yaw. With TVC, the nozzle keeps vectoring above the game's 250 m/s cut-off. The pedal's sideslip is limited.
- **MPO:** the stick is velocity-vector roll and the pedals are pure body yaw, at every AoA.

## 0.2.6
- **ThrottleIndependentAirbrake off:** the airbrake is fully stock, so the brake key no longer opens it. This fixes Brake and Countermeasures sharing one controller button.

## 0.2.5
- **FcsApi v4:** HUD and visualizer auto-detection.
- **No TVC:** faster roll rate; AoA cap 50° on afterburner.

## 0.2.4
- **Corkscrew:** smoother yaw-rate curve.
- **Aileron visuals:** drawn about the measured hinge line.

## 0.2.3
- Aileron hinge-line visuals (render only).

## 0.2.2
- `MaxYawRate` 90.
- Yaw-nozzle hold gated on rudder headroom.

## 0.2.1
- Ground elevon pitch.
- Yaw-TVC sideslip gain ramp.

## 0.2.0
- AoA limit 70°.
- Inboard elevons as hinge effectors.
- `ElevonPitch` on.

## 0.1.0
- First release. A fork of KR-67 Ifrit FCS 0.5.4, retargeted to the FS-12 Revoker.
