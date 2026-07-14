# Contributing to Gravity

## Toolchain

Gravity is built only with Zig `0.16.0`, as locked independently by
`.zigversion` and `build.zig.zon`. Use the hermetic Zig distribution selected
by your version manager; do not substitute a system compiler, libc, libm, or
an unpinned Zig nightly for core builds.

## Commands

Run these commands from the repository root (the build graph resolves paths
from `build.zig`, so invoking Zig from another working directory is supported):

- `zig fmt --check .`
- `zig build test`
- `zig build test-all-modes`
- `zig build determinism`
- `zig build abi-test`
- `zig build fuzz`
- `zig build benchmark`
- `zig build -Dtarget=wasm32-freestanding`
- `zig build -Dtarget=aarch64-linux-gnu`
- `zig build demo`

`zig build demo` is the only foundation command that probes Node.js and pnpm.
It builds the core WebAssembly artifact and checks those frontend prerequisites;
the React/Vite application is added in Task 26. `zig build demo-run` performs
the same prerequisite checks and prints the explicit hand-off message until
that application exists.

Keep generated artifacts under `zig-out` or Zig cache directories. Do not
commit generated build output, and do not add runtime floating-point math or
platform-dependent core dependencies.
