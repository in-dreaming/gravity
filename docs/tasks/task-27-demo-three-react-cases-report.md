# Task 27 qualification report

Date: 2026-07-20

## Implemented

- React 19.2.7 control surface and Three.js 0.185.1 renderer with camera,
  lighting, shadows, OrbitControls, resize handling, query debug primitives,
  render-only interpolation, and complete geometry/material disposal.
- Simulation controller outside React with exact pause, single-step, reset,
  fixed-Tick accumulation, canonical decimal-to-Q32.32 conversion, command
  history, full snapshots, late-input replay, and fresh-authority verification.
- All 15 required cases using only official C ABI WASM calls, including six
  joint kinds, baked hull/mesh/height/compound assets, CCD, shape cast, planar
  DOF locks, sleep/wake, rollback, dual-World determinism, and stress reporting.
- Diagnostics for Tick/hash, body/awake/collider/contact/joint/pair/event counts,
  deterministic phase visits, step duration, rollback state, query results,
  worker count, WASM pages, and optional native worker report input.
- Fixed Chromium/software-renderer Playwright qualification for ABI behavior,
  all case hashes and reset parity, exact stepping, interpolation isolation,
  sleep/wake, rollback, lifecycle high-water mark, DOM/accessibility, and a
  perceptual screenshot baseline.
- `zig build demo-run` remains the sole documented local launch entry. Core
  module/package consumers remain independent of Node, pnpm, React, and Three.

## Qualification hashes

| Case | Tick | Hash |
|---|---:|---|
| Sphere / Box Stacks | 4 | `5ede974e91341090464b89f92347c2cd` |
| Friction & Restitution | 4 | `21bb6174ef8fc090fe0186ad1575acd2` |
| Newton Cradle | 4 | `de5e65cedb4928e5b68968231563bad7` |
| Six Joint Gallery | 4 | `5ee81fecac78ac45387cbe8618acabde` |
| 3D Ragdoll | 4 | `70dff34395111e11f6af91ad4d2c6fef` |
| Hull & Compound Collapse | 4 | `e6ca20bb55d98fae6b4ed042b97f308e` |
| Dynamic Mesh-Mesh | 4 | `4859f9b6e51c3143203a27dea96592be` |
| HeightField Terrain | 4 | `9b01aac6f537bca84fb23c66472b6d9e` |
| CCD Thin Wall & Moving Mesh | 4 | `7de0f5cd05411dc6d0d90c3987f2965c` |
| Ray / Shape Cast / Overlap | 4 | `1483074bce75f17737c1d2615e06df05` |
| 2D Planar DOF | 4 | `1f4c8353daba9b65a7c7ed2d870aeb55` |
| Sleep / Wake | 36 | `d1a7e33d11edf57fd8826c83abe787f7` |
| Snapshot / Rollback | 8 | `87d8d73dfdcb6b7b87ba61dcc48eafbd` |
| Dual-World Determinism | 4 | `e624782f796d619da67d1e50a0df2c5b` |
| Stress / Worker Scaling | 4 | `c258557689232338d5f025fbc4b2d1c5` |

## Dependency corrections

Task 27 exposed missing formal ABI seams that made several required cases
impossible without forbidden JavaScript/static substitutes. The ABI gained
additive joint create/destroy, per-body CCD policy, shape cast, structured World
statistics, and pipeline-fault diagnostics. Full snapshots and hashes include
joint, sleep, and CCD state. The original 96-byte `WorldDesc` prefix remains
accepted and the legacy ABI hash is unchanged.

Qualification then exposed two implementation bugs. CCD enable storage was
incorrectly sized and indexed as body slots even though the production pipeline
indexes collider slots; it now follows collider capacity and body policy is
applied to all live colliders owned by that body. `awake_body_count` also counted
static bodies; it now reports awake dynamic bodies only. Focused ABI regressions
cover the corrected paths in all optimization modes.

## Validation

Final validation used detached worktree commit `8ac3849` and exact Spindle
revision `6756fb2feecfa354a7ae42bca3af5d9bd66c7558`.

| Command | Result |
|---|---|
| cold `zig build demo -j1 --summary all` | passed, 16/16 steps; frozen install, four baked assets, 34 exports, zero imports, strict TypeScript, production Vite build |
| incremental `zig build demo -j1 --summary all` | passed, 16/16 steps; dependencies unchanged and Zig artifacts cached |
| `zig build demo-test -j1 --summary all` | passed, 17/17 steps and 5/5 Chromium tests |
| `zig build demo-isolation -j1 --summary all` without Node/pnpm on `PATH` | passed, 2/2 steps |
| `zig build abi-test -j1 --summary all` | passed, 18/18 steps and 2/2 tests, including C11/C++17/C# consumers |
| `zig build test-abi-all-modes -j1 --summary all` | passed, 11/11 steps and 6/6 Debug/ReleaseSafe/ReleaseFast tests |
| `zig fmt --check build.zig src tests tools` | passed |
| `git diff --check` | passed |

## Completion

- All 15 cases execute through the official WASM ABI and reproduce their
  qualification hashes after reset.
- Late-input replay equals a fresh authority World; renderer interpolation does
  not change the World hash; repeated case cycles stabilize at the same WASM
  high-water mark.
- Screenshot comparison uses a fixed 1440x900 Chromium software-rendering
  configuration with a 4% perceptual pixel threshold. It is a UI regression
  check, not a cross-GPU physics golden.
- Unfinished items: none.
