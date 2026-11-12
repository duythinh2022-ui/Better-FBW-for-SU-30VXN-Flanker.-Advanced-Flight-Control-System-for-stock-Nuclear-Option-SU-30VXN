#!/bin/bash
# Builds the native core (Su30_FcsCore): the allocator's solve and the grid kernels, ported from the F-22E FCS 4.1.0 core.
#   build.sh            -> fcs_core/target/x86_64-pc-windows-gnu/release/Su30_FcsCore.dll   (the game; copy it to prebuilt/, build.sh embeds it in the plugin DLL)
#   build.sh linux      -> fcs_core/target/release/libSu30_FcsCore.so                       (the bench; bench.sh build copies it)
# Built with Rust 1.97.1 (Ubuntu's rustc-1.97 / rust-1.97-src): the Windows standard library is compiled from source
# (-Zbuild-std, RUSTC_BOOTSTRAP), because the Windows target's prebuilt std could not be downloaded here. Ubuntu's source
# package has std's windows-link dependency pruned; RUST_SRC points at a copy of library/ with it put back (std/Cargo.toml:
# [target.'cfg(any(windows, target_os = "cygwin"))'.dependencies.windows-link] path = "../windows_link"). mingw-w64 gcc links.
# With a rustup toolchain that has the x86_64-pc-windows-gnu target installed, plain `cargo build --release --target ...` works.
cd "$(dirname "$0")/fcs_core"
# Rust 1.97 (the released core was built with Ubuntu's rustc-1.97); RUSTBIN = its bin folder if it is not on PATH
[ -n "$RUSTBIN" ] && export PATH=$RUSTBIN:$PATH; [ -d /usr/lib/rust-1.97/bin ] && [ -z "$RUSTBIN" ] && export PATH=/usr/lib/rust-1.97/bin:$PATH
# RUST_SRC = the standard library source (rustup: `rustup component add rust-src`; found from the sysroot if unset)
RUST_SRC=${RUST_SRC:-$(rustc --print sysroot)/lib/rustlib/src/rust/library}
if [ "$1" = "linux" ]; then cargo build --release --offline
else RUSTC_BOOTSTRAP=1 __CARGO_TESTS_ONLY_SRC_ROOT=$RUST_SRC \
  CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER=${MINGW_GCC:-x86_64-w64-mingw32-gcc} \
  cargo build --release --target x86_64-pc-windows-gnu -Zbuild-std=std,panic_unwind -Zbuild-std-features=; fi
