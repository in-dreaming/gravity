# Task 26 qualification report

Date: 2026-07-20

## Implemented

- Independent Node 22.22.2/pnpm 11.7.0 project with frozen lockfile, strict TypeScript, Vite production output, Playwright, and incremental dependency stamp.
- Official `wasm32-freestanding` C ABI artifact with the exact 28-symbol baseline, exported memory, zero imports, and no Spindle/thread/executor/host-dispatch build path.
- Generated TypeScript ABI constants/layouts checked against the frozen schema, C header size assertions, Zig size/offset assertions, and export baseline.
- Reusable aligned linear-memory arena; buffer-view refresh after `memory.grow`; AssetStore/World lifecycle ownership; raw body/collider/command/event/query/snapshot/hash APIs.
- `demo`, `demo-test`, `demo-isolation`, and `demo-run` Zig build steps. Playwright starts the application through `zig build demo-run`.
- External-style Zig consumer fixture proves the published core module builds without Node/pnpm and the package manifest excludes Demo files.

## Frozen invariants

- TypeScript never implements physics math and contains no explicit `any` escape.
- WASM always dispatches Gravity jobs serially. A non-null host dispatcher is rejected as unsupported; the stable symbol remains ABI-compatible.
- Insufficient output capacity is handled by required-count probes and caller-owned buffers.
- Disposed World/AssetStore memory returns to the arena. After one warm allocation, 32 repeated create/dispose cycles do not grow linear memory.
- Snapshot load after forced `memory.grow` reproduces hash `4336297d3f06a9c557e75aea2a839853`.

## Validation

Validation used a detached clean worktree at commit `a4cb18c` with the exact
Spindle revision `6756fb2feecfa354a7ae42bca3af5d9bd66c7558`.

| Command | Result |
|---|---|
| cold `zig build demo -j1 --summary all` | passed, 9/9 steps; frozen pnpm install and production Vite build |
| incremental `zig build demo -j1 --summary all` | passed, 9/9 steps; reported `demo dependencies unchanged` and skipped install |
| `zig build demo-test -j1 --summary all` | passed, 10/10 steps and 1/1 Chromium test |
| `zig build demo-isolation -j1 --summary all` without Node/pnpm on `PATH` | passed, 2/2 steps |
| ordinary `zig build -j1 --summary all` without Node/pnpm on `PATH` | passed, 11/11 steps |
| `zig build abi-wasm-smoke -j1 --summary all` | passed, 5/5 steps; zero imports and exact export baseline |
| `zig build test-abi-all-modes -j1 --summary all` | passed, 11/11 steps and 3/3 tests |
| `zig build abi-test -j1 --summary all` | passed, 18/18 steps and 1/1 native ABI smoke test |
| `zig fmt --check build.zig src tests tools` | passed |
| `git diff --check` | passed |

An additional repository-wide `zig build test-all-modes -j1 --summary all`
attempt reached the 1,804-second command limit while compiling the pre-existing
`tests/unit/geometry_test.zig` ReleaseFast artifact. It emitted no test failure
and is not a Task 26 gate; the affected ABI surface completed its explicit
Debug, ReleaseSafe, and ReleaseFast matrix above.

## Dependency correction

Task 26 exposed a real isolation defect in the previous ABI build: the WASM
artifact unconditionally imported the native host dispatcher module. The ABI
module now selects a serial WASM jobs implementation at build time, while the
native dispatcher contract and stable C ABI symbol remain unchanged. Regression
checks reject Spindle, host-dispatcher, pthread, atomic-wait, or other imported
WASM dependencies.

## Completion

- Golden replay hash: `3abdf5be432885c4b137c5367272516f`.
- ABI initial-state hash: `4336297d3f06a9c557e75aea2a839853`.
- ABI baseline change: none; the exact 28-symbol stable export set is preserved.
- Unfinished items: none.
