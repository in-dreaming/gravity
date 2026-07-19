# Gravity

Gravity 1.0 is a deterministic 3D rigid-body physics engine written in Zig
0.16.0. It uses Q32.32 fixed-point state, supports rollback and deterministic
native parallelism, and publishes one stable C ABI for C, C++, C#, native host
languages, and WebAssembly. A 2D world is the same 3D pipeline with planar DOF
locks.

## Product surface

- Static, dynamic, and kinematic bodies; Sphere, Box, Capsule, ConvexHull,
  Compound, immutable TriangleMesh, and HeightField shapes.
- Complete discrete shape-pair coverage, convex CCD, ray/shape/AABB/point
  queries, sensors and deterministic contact events.
- Distance, Ball-Socket, Hinge, Slider, Fixed, and Cone-Twist joints with the
  applicable limit, motor, spring, and damping controls.
- Canonical snapshots, 120-Tick rollback, replay/diff tooling, layered BLAKE3
  hashes, deterministic sleep, and 1/2/4/8-worker native execution.
- Static/shared native libraries, freestanding WASM, asset baker and replay
  tools, plus a local 15-case React/Three.js Demo.

## Build

Use exactly Zig 0.16.0. Clone with submodules, then run:

```text
zig build
zig build test-all-modes -j1
zig build abi-test -j1
zig build demo-run -j1
```

The default install writes `gravity.h`, native static/shared libraries, and
`gravity.wasm` below `zig-out`. `zig build release-check -j1` creates and
verifies deterministic source, six native target, WASM, and Demo packages with
an SPDX SBOM and SHA-256 manifest under `zig-out/release`.

The stable integration boundary is [`include/gravity.h`](include/gravity.h).
See the [C ABI guide](docs/integration/c-abi.md), [Web Demo guide](docs/integration/web-demo.md),
[determinism contract](docs/integration/determinism.md), and [product limits](docs/release/limits.md).

## Version boundaries

Package 1.0.0 uses C ABI v1, protocol v1, snapshot format v1, and asset format
v2. ABI compatibility does not imply simulation compatibility across protocol
majors. Golden hashes are changed only with an explicit protocol review.

Gravity and the pinned executor dependency are MIT licensed. Security reports
follow [SECURITY.md](SECURITY.md).
