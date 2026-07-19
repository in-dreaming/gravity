# Web Demo WASM integration

Task 26 provides the isolated `demo/web` pnpm project and the official TypeScript wrapper for the `wasm32-freestanding` C ABI. The core package manifest excludes `demo`; normal `zig build` and `gravity` Zig module consumers do not require Node or pnpm.

## Toolchain and build

Use the exact versions declared by `demo/web/package.json`: Node 22.22.2 and pnpm 11.7.0.

```text
zig build demo
zig build demo-test
zig build demo-run
```

`demo` compiles the official C ABI WASM, verifies the exact export set and zero imports, checks the generated ABI layout, installs with `pnpm install --frozen-lockfile` only when the package, lockfile, Node version, or pnpm version stamp changes, runs strict TypeScript, and writes the production site to `zig-out/demo`.

`demo-test` launches `demo-run` through Playwright and verifies the native/Zig reference hash, `memory.grow` view refresh, command/body/event/query batches, snapshot round trip, and stable linear-memory page count across repeated World and AssetStore create/dispose cycles.

## Wrapper boundary

`src/wasm/gravity.ts` exposes raw `bigint` fixed-point values, fixed-width IDs, raw vectors/quaternions/transforms, and RAII-style `AssetStore` and `World` objects. It owns a reusable aligned linear-memory arena and refreshes every `DataView`/`Uint8Array` after a memory buffer change. Call `World.dispose()` before `AssetStore.dispose()`.

The wrapper contains no collision, integration, interpolation, or fixed-point approximation. All steps, queries, events, body states, snapshots, and hashes call `gravity_v1_*` exports. WASM is serial-only: the stable dispatcher symbol remains for ABI parity, clearing it is accepted, and supplying a host callback returns `GRAVITY_ERROR_UNSUPPORTED`.

`docs/formats/c-abi-v1.schema.json` is the TypeScript layout source. `pnpm abi:generate` updates the checked-in generated constants; normal builds use `pnpm abi:check` and fail on schema, header, generated layout, or exact WASM export drift.
