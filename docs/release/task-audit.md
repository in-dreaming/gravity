# Tasks 00-27 product audit

This audit binds every required predecessor to production code and an executable
qualification gate. Task documents remain the authoritative acceptance source;
the table prevents a passing umbrella step from hiding an omitted subsystem.

| Task | Production evidence | Task 28 gate |
|---:|---|---|
| 00 | build graph, pinned Zig, CI, package boundaries | `fmt`, native matrix, release source consumer |
| 01 | `math/fp.zig`, `math/wide.zig`, envelope validator | FP unit/fuzz/golden in three modes and WASI |
| 02 | vector, quaternion, matrix and transform math | geometry suite in three modes, ARM64 and WASI |
| 03 | caller memory, IDs, config and radix ordering | core/codec suites and bounded fuzz in three modes |
| 04 | canonical codec and BLAKE3 domains | codec, metadata and cross-target golden suites |
| 05 | asset source/parser/store and baker | asset suites, parser corpus/fuzz, `security-gate` |
| 06 | baked hull/mesh/heightfield/compound BVHs | baked-geometry suites in three modes/WASI/ARM64 |
| 07 | runtime shapes, mass, filtering and Compound | runtime-shapes suites in three modes/WASI/ARM64 |
| 08 | canonical 3D SAP | broadphase suites and worker scheduling matrix |
| 09 | analytic primitive collision | analytic suites in three modes/WASI/ARM64 |
| 10 | GJK/EPA/clipping | GJK suites and geometry fuzz in three modes |
| 11 | mesh/HeightField/Compound narrow phase | mesh-collision suites including dynamic mesh-mesh |
| 12 | contact cache, material and ordered events | contact-cache suites in three modes/WASI/ARM64 |
| 13 | bodies, canonical commands and integration | dynamics suites and pipeline long run |
| 14 | islands, DOF locks and constraint rows | constraints suites plus 2D Demo qualification |
| 15 | 3D contact solver | contact-solver suites in three modes/WASI/ARM64 |
| 16 | all six joints and controls | joint unit/scenario suites plus Demo gallery/ragdoll |
| 17 | ray, shape cast and overlaps | query suites and formal-WASM query case |
| 18 | deterministic island sleep/wake | sleeping suites and Demo sleep/wake behavior test |
| 19 | frozen convex-caster CCD boundary | CCD suites and thin-wall/moving-mesh Demo case |
| 20 | transactional complete World pipeline | pipeline suites and native/WASI one-million Tick gates |
| 21 | snapshot, 120-Tick rollback, replay and diff | snapshot/replay suites with 100k random rollback |
| 22 | stable C ABI and multi-target artifacts | ABI three-mode, C11/C++17/C#/WASM consumers, six targets |
| 22A | executor-only pinned Spindle adoption | adapter/lifecycle/fault suites, import/license audit |
| 23 | deterministic parallel production kernels | serial and Spindle 1/2/4/8 hashes, TSAN, worker switching |
| 24 | fixed corpus optimization and budgets | `performance-ci`, zero Tick allocations, scaling reports |
| 25 | bounded fuzz, hostile ABI, security and SBOM | `fuzz-all-modes`, `security-gate`, SPDX/package audit |
| 26 | isolated WASM/TypeScript wrapper/build | ABI smoke, memory growth/lifecycle and isolation consumer |
| 27 | React/Three renderer and 15 classic cases | 15 reset hashes, rollback, DOM, lifecycle and screenshot |

The `qualification-native` build step depends explicitly on every focused
three-mode subsystem gate, rather than relying on the older convenience
`test-all-modes` subset. CI executes it natively on Windows, Linux and macOS on
x86-64 and ARM64. Dedicated WASM, Demo and reproducible-release jobs complete
the Task 28 matrix.
