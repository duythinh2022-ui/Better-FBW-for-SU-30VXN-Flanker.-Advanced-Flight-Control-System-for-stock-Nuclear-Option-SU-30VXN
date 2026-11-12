# Better-FBW-for-SU-30VXN-Flanker.-Advanced-Flight-Control-System-for-stock-Nuclear-Option-SU-30VXN
Flight Controls overhaul for kyleisred's Su-30VXN Gandiva with a modern FBW system, for carefree handling even for PSM, damaged airframe departure tolerance, asymmetric weapons load compensation.

AI disclosure: Coding for this mod is entirely using AI, since idk how to write code lol, mainly Claude Opus 5/5.5 (feel free to call this AI slop if you like 🙂). Feel free to make modifications or new mods based on this mod *NOTE: this mod is my side fun project and WIP, issues and problems are expected.

Another FCS mod with all of the goodies from my other FCS mods (F-22E, Ifrit, Revoker, Compass, Ternion), and the first one with canted 3D TVC. INDI (Incremental Non-linear Dynamic Inversion) with a control-surface and TVC allocation solver on top, carefree handling, easy post stall maneuvering, asymmetric weight compensation, envelope protection, and departure tolerance for damaged airframe.

Current version: **0.5.6** (see `CHANGELOG.md`).

## What it does
- **Pitch:** G command at high airspeed blending to AoA / pitch-rate command at low airspeed. AoA limit +75 / -65 deg, soft limit: if a corkscrew forces the jet past it, the FCS holds pitch rate where you are instead of shoving the nose down.
- **Roll / yaw:** velocity-vector roll. Roll and yaw blend into one axis as AoA rises, so a full roll at the AoA limit (falling leaf) rotates about the flight path instead of slipping.
- **TVC like the real jet:** AL-31FP single-axis nozzles, canted 32 deg outboard, ±15 deg. The allocator uses them for pitch, roll and yaw together, so the jet keeps lateral authority past the stall.
- **Stability Assist off (MPO):** no AoA limit, velocity-vector roll by default (`MpoRollMode`); past the limit it rolls about whatever axis the canted nozzles give. `LowSpeedTvcRoll` (off by default) adds nozzle roll when the airframe's sideslip stiffness is gone.
- **Canards and LERX** take pitch work off the nozzles, so the TVC is free for roll. The flaperons are roll first, flaps second (as on my F-22E).
- **Asymmetric loads:** the AoA limit comes from a coupled check (an LP solved each sweep: how much flight-path roll is left each way with the load held, pitch held and no sideslip). The heavier the load, the lower the limit. No switches, it just runs out of authority where the jet does.
- **Engine out:** no AoA cap for it; the live engine's thrust is matched to the dead one's, so the jet doesn't yaw off.
- **Damage:** missing or damaged surfaces and engines are taken out of the allocator; the FCS limits what it can't cover.
- **On the ground:** INDI nose-wheel steering, and the main brakes help yaw (differential braking).
- **Airframe tweaks:** CG moved aft to make the jet close to neutral in pitch in flight without tipping back on the ground, LERX 1.5x, canards 1.6x, Flanker-style drag eased a bit. The CG doesn't move as fuel burns.
- **Native core:** the allocator, the grid kernels and the asymmetric-load LP run in a small Rust library carried inside the plugin DLL (written to `BepInEx\cache\Su30_FcsCore\` on first load). If it can't load (Windows 7/8, antivirus, read-only folder) the FCS uses its C# code instead, just slower; `LogOutput.log` says which.

## Variants
`PerformanceProfile` in the main config:
- **Stock:** the Su-30VXN's own thrust, drag and fuel burn (with the airframe tweaks above).
- **Buffed:** thrust x1.15 (afterburner too), fuel burn x0.65, parasitic drag x0.90, 10x flares, 2x gun ammo, 1.5x EW capacity, 1.2x EW recharge.

The flight control is the same in both.

## Multiplayer
`HostAllowFcs` and `HostLockVariant` are on by default: the host decides whether clients may fly the FCS and forces everyone to the host's variant.

## Install
1. Install kyleisred's Su-30VXN Gandiva mod (Blueprinter addon) and BepInEx 5.
2. Put `Su30_VXN_FCS.dll` in `BepInEx\plugins\` (any subfolder). Both mods have to load.
3. Optional: `Su30_VXN_FCS_Dev.dll` unlocks the dev / tuning config (`Su30_VXN.FCS.Dev.cfg`). Not needed to fly.

Windows 10/11 (or Proton) for the native core; anything older falls back to C#.

## Source layout
- `Su30_VXN.FCS/` – the plugin (C#).
- `DevUnlock/` – the dev-settings unlock plugin.
- `native/` – the Rust core (`fcs_core/`), the prebuilt `Su30_FcsCore.dll` that `build.sh` embeds, its rules and the Mono check tools (`native/verify/`). See `native/README.md`.
- `tools/` – sweep / hold scripts for the bench.

The bench (rigid-body sim of the airframe with the game's aero model) is not in this repo: it needs the airframe data extracted from kyleisred's mod, which isn't mine to publish.

## Build
Needs .NET SDK 8 (for its C# compiler) and the game folder with BepInEx installed (the build references the game's `Managed\` DLLs and BepInEx core). From Linux or WSL:
```
NO_GAME="/mnt/c/Program Files (x86)/Steam/steamapps/common/Nuclear Option" bash build.sh Su30_VXN.FCS Su30_VXN_FCS.dll
```
- Name the output exactly `Su30_VXN_FCS.dll`: the assembly name comes from the file name.
- The build is deterministic: from this tree it reproduces the released 0.5.6 DLL byte for byte (sha256 `cd0dfbcb0a8b6cc8...`).
- Rebuilding the native core: `bash native/build.sh` (Rust 1.97, `x86_64-pc-windows-gnu`, mingw-w64), then copy the DLL to `native/prebuilt/`. Its code bytes match the prebuilt one; only the link timestamps in the PE header differ.

## Other FCS mods
- [F-22E FCS mod](https://github.com/duythinh2022-ui/Advanced-flight-controls-addon-for-Aryx-s-F-22E-mod-for-Nuclear-Option.)
- [Ifrit FCS mod](https://github.com/duythinh2022-ui/Better-FBW-for-KR-67-Ifrit.-Advanced-Flight-Control-System-for-stock-Nuclear-Option-Ifrit)
- [Revoker FCS mod](https://github.com/duythinh2022-ui/Better-FBW-for-FS-12-Revoker.-Advanced-Flight-Control-System-for-stock-Nuclear-Option-Revoker)
- [Ternion FCS mod](https://github.com/duythinh2022-ui/Better-FBW-for-FS-3-Ternion.-Advanced-Flight-Control-System-for-Ternion)
