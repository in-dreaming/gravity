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
- `zig build release-check -j1`

`zig build demo` and release packaging are the only commands that require
Node.js and pnpm. `zig build demo-run` is the sole supported local Demo launch
entry. Release qualification and the platform matrix are documented under
`docs/release` and `.github/workflows/ci.yml`.

Keep generated artifacts under `zig-out` or Zig cache directories. Do not
commit generated build output, and do not add runtime floating-point math or
platform-dependent core dependencies.
