# Task 26 qualification report

Date: 2026-07-19

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

Final command results are recorded after clean-worktree cold build and the complete Task 26 regression matrix. Unfinished items remain none only when every Task 26 gate below is green.
