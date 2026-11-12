# Native core (Su-30 FCS 0.4.0)

`fcs_core/` is Rust: `Su30_FcsCore.dll` for the game, `libSu30_FcsCore.so` for the bench. A port of the F-22E Strike Raptor FCS 4.1.0 core
(`native/fcs_core` in the F-22E source) to this jet's own C#. It holds

- `Allocator.cs`'s solve, plugin side `Allocator.NativeSolve` (`SolveDispatch`);
- the grid kernels (`su30_fcs_grid`): one surface's moment at every deflection of its grid in one airflow. Mode 0 fills the
  surface's table (`SurfaceEffector.EvalCompute`'s loop of `Eval`), mode 1 returns the min / max the asymmetric-load sweep keeps
  (`FcsController.AsymSweepPoint`'s capacity loop). Plugin side `SurfaceEffector.GridMomentsNative` / `GridRangeNative`; the
  rotations (sin / cos) stay in C# (`SurfaceEffector.GridGeom`).

This jet's differences from the F-22E's core: the velocity-frame lateral residual in `Allocator.E` (`slipFrame`, `slipSin`, `slipCos` in `AllocIo`); the coordinate pass's Newton candidate also for the yaw nozzles (`EffN.kind` 3 = `TvcYawEffector`); the table kernel also returns each grid point's normal force (`SurfaceEffector.FN`, `GridIn.outFN`); the sweep evaluates the stabilators at their `AsymStabTravel` deflections, so its geometry is built for that list (`FcsController.SwDeflections`) and reused from the table only when the deflections match. (Same allocator and surface code as the Revoker; its sweep calls `EvalFlow` per deflection, so the cached path prepares the flow with `PrepFlow`, EvalFlow's own expressions.)

The C# code is the reference; the library must return the same bits wherever C# float arithmetic is strict 32-bit.
The wrapper is `Su30_VXN.FCS/FcsNative.cs`. At load it checks the library's ABI number and structure sizes and runs a self-test of the
kernels against a C# twin (`FcsNative.GridTwin`); a failure switches the kernels off and leaves the solve on; a missing or failing
library leaves everything on the C# code.

**Own names.** Every FCS mod loads in the same game process, so this one has its own DLL name (`Su30_FcsCore.dll`), its own
`DllImport` name and its own export prefix (`su30_fcs_`). The plugin loads the DLL by its full path (`LoadLibraryW`) before the
first call. Checked on the game's Mono with all five FCS plugins loaded at once: each bound its own library.

## Rules for changing it (as the F-22E's)
- Any change to `Allocator.cs` (`SolveInner`, `ReachFrom`, `SolveCore`, `PairedSearch`, `SolveLinear`, the cost terms, `E`) or to
  `Effector.MAt` has to be made in `fcs_core/src/lib.rs` as well, expression for expression: same operand types, same order,
  same float / double conversions. No transcendental functions, no reordering, no fused multiply-add.
- A new field the solve reads goes into `EffN` or `AllocIo` on both sides (`lib.rs` and `FcsNative.cs`), and `ABI` / `Abi` is bumped.
- Any change to `SurfaceEffector.Eval`, `EvalFlow` / `EvalFlowAt`, `EvalFlowGrid`, `Camber.Foil` / `FoilPre` / `DeltaFrom`,
  `AirfoilTable.Get` / `Read`, `Effector.ToAero`, or to the sweep's capacity loop has to be made in `grid_point` / `grid_run`
  (`lib.rs`) and in `FcsNative.GridTwin` as well. The kernels may call `atan2` (written as C# does) and nothing else from a maths
  library.
- Then run the checks below. Zero differences on the bench is the pass mark.

## Build
```
bash native/build.sh          # the game's DLL  -> fcs_core/target/x86_64-pc-windows-gnu/release/Su30_FcsCore.dll
bash native/build.sh linux    # the bench's .so  (bench.sh build does this and copies it into bench/bin)
```
Built with Rust 1.97.1 (Ubuntu's `rustc-1.97`), target `x86_64-pc-windows-gnu`, mingw-w64 gcc as the linker. The Windows target's
prebuilt standard library could not be downloaded in the build environment, so `build.sh` compiles it from source
(`-Zbuild-std`); see the comment at its top. No crates are downloaded. The F-22E's core was built with Rust 1.99.1: the code
needs nothing version-specific (IEEE float arithmetic, no maths library but `atan2`).

## Check on the bench (strict float arithmetic: must be bit-identical)
```
bash bench.sh suite nat                                          # default: solve + both kernels native
bash bench.sh suite chk NativeAllocatorCheck=2                   # every solve both ways, compared bit for bit
bash bench.sh suite cs NativeAllocator=0                         # all C#
bash bench.sh suite koff NativeSweep=false NativeTables=false    # solve native, kernels in C#
bash bench.sh suite nogrid SweepGridCache=false                  # the sweep without the geometry cache
python3 native/verify/csvcmp.py bench/out base <tag>             # every per-case CSV (timing columns aside) and table vs base
```
`FCS_NATIVE_STATS=<file>` makes each bench run append its counts (`native/verify/nsum.py` sums them).

## Check on the game's own Mono runtime (native/verify)
`host.c` (from the F-22E) runs a managed exe on the game's `mono-2.0-bdwgc.dll` with Unity's `-O=-float32`, under Wine.
`jetcheck.cs` loads this plugin there with `Su30_FcsCore.dll` beside it: the library loaded by full path, ABI and structure sizes, the
kernels' self-test against the C# twin as the game's Mono compiles it (repeated 200 times). `multicheck.cs` does the same with
several FCS plugins in one process.
