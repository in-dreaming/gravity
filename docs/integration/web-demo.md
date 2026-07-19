# Web Demo

Task 27 provides the local React and Three.js demonstration in `demo/web`. It
runs every physics case through the official `wasm32-freestanding` C ABI; the
renderer only reads body transforms and never feeds floating-point state back
into a World.

## Start locally

Use Node 22.22.2 and pnpm 11.7.0, then run the sole supported launch entry from
the repository root:

```text
zig build demo-run
```

Open `http://127.0.0.1:5173`. The build generates and loads the ConvexHull,
TriangleMesh, HeightField, and Compound `.grav` assets, compiles the official
C ABI WASM, verifies its exact exports and zero imports, checks the generated
TypeScript layout, and builds the frontend before Vite starts.

Use the case selector to switch among all 15 qualification scenes. Simulation
starts paused. `Run`, `Step`, and `Reset` operate on exact ticks; impulse text is
converted from a canonical decimal to Q32.32 using ties-to-even rounding. The
diagnostics panel reports the canonical Tick hash, body/awake/contact/joint/pair
counts, deterministic phase visits, WASM memory pages, query results, rollback
status, and the second World hash where applicable. The stress case reports the
serial WASM worker and accepts a native scaling report file for comparison.

## Case and rollback invariants

Every case owns fixed assets, configuration, command seed, qualification Tick,
and expected hash. Reset reconstructs its World and must reproduce that hash.
The rollback case retains full snapshots plus canonical command history; a late
input restores the target Tick and replays both the displayed World and a fresh
authority World to the same final hash. The determinism case advances two
independent Worlds with identical command batches and compares them every Tick.

Rendering uses previous/current body states only. Three.js interpolation,
OrbitControls, resize handling, lights, shadows, and debug primitives cannot
mutate simulation state. Switching cases disposes both Worlds and all scene
geometry/material resources; the reusable WASM arena reaches a stable high-water
mark after the largest case.

## Build and test

```text
zig build demo
zig build demo-test
zig build demo-isolation
```

`demo` writes the production site to `zig-out/demo`. `demo-test` uses Chromium
with a fixed viewport and software renderer. It checks the ABI smoke contract,
all 15 case hashes and resets, exact pause/single-step behavior, sleep/wake,
late-input rollback, dual-World parity, DOM/accessibility, stable WASM memory,
and a perceptual screenshot baseline. The screenshot is a UI regression check,
not a cross-GPU physics golden; canonical World hashes remain the physics
qualification oracle.

`demo-isolation` builds an external-style Zig core consumer without frontend
dependencies. The package manifest excludes `demo`, so normal core builds and
Zig module consumers do not require Node, pnpm, React, or Three.js.

## Wrapper boundary

`demo/web/src/wasm/gravity.ts` exposes raw `bigint` fixed-point values,
fixed-width IDs, and RAII-style `AssetStore` and `World` objects. It refreshes
all views after `memory.grow` and owns no collision, integration, interpolation,
or fallback physics. Joint creation, CCD policy, queries including shape cast,
snapshots, hashes, statistics, and structured pipeline faults all call
`gravity_v1_*` exports.

`docs/formats/c-abi-v1.schema.json` is the TypeScript layout source.
`pnpm abi:generate` updates checked-in constants; normal builds run the check
mode and fail on schema, header, generated layout, or exact WASM export drift.
