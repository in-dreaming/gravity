# Task 28 product qualification report

Date: 2026-07-20

Status: local qualification complete; product-ready declaration pending the
six-platform native hosted-runner matrix.

## Implemented

- A six-target native CI matrix for Windows, Linux, and macOS on x86-64 and
  ARM64, plus dedicated WASM, Demo, reproducible-release, and aggregation jobs.
- A complete `qualification-native` gate covering the focused Task 00-25
  suites in Debug, ReleaseSafe, and ReleaseFast; 1/2/4/8-worker equivalence and
  scaling; one-million-Tick execution; 100k rollback probes; fuzz, security,
  ABI, performance, and deterministic-state gates.
- A qualification audit which rejects unfinished main-path markers, missing
  Task 00-27 records, missing product documentation, version/SBOM drift, and an
  incomplete native CI target matrix.
- Versioned 1.0.0 source, Windows/Linux/macOS x86-64/ARM64 static and shared,
  freestanding WASM, and Demo packages. The nine deterministic tar files use
  normalized USTAR metadata, a frozen order, a manifest, and SHA-256 checksums.
- Product documentation for the Zig module, C ABI, integration, deterministic
  operation, formats, security, limits, contribution workflow, release audit,
  and release checklist.

## Corrections exposed by qualification

- Release libraries are stripped of nonsemantic compiler metadata. Static
  archives are rebuilt with deterministic member names because Zig otherwise
  embeds the selected cache directory in the archive string table. Windows PDB
  files are excluded from stripped runtime packages because their build-session
  GUID is nonsemantic and nonreproducible; the DLL and static library remain.
- The million-Tick test now reads its opt-in switch from
  `std.testing.environ`. Constructing a target-global environment directly was
  valid for Windows/WASI but failed while cross-compiling the Linux ARM64 test.
- Playwright starts Vite directly after the outer Demo build has installed and
  checked all WASM/assets. The previous nested `zig build demo-run` could wait
  indefinitely on the outer Zig build/cache lock. Qualification also forbids
  reusing an unknown service already listening on the Demo port.

## Local validation

All commands below used Zig 0.16.0 and the exact Spindle revision
`6756fb2feecfa354a7ae42bca3af5d9bd66c7558`.

| Command | Result |
|---|---|
| `zig build qualification-native -j1 --summary all` | passed in 1245 s; 376/376 steps and 724/724 tests; all modes, long-run, rollback, fuzz, ABI, security, performance, determinism, and worker gates |
| `zig build wasm-validate -j1 --summary all` | passed in 139.7 s; 71/71 steps under Wasmtime |
| `zig build test-pipeline-long-run-wasm -j1 --summary all` | passed after the portable-environment correction; 5/5 steps and 13/13 tests, including the million-Tick hash |
| `zig build abi-wasm-smoke -j1 --summary all` | passed; 5/5 steps and Node execution of the freestanding ABI |
| `zig build arm-validate -j1 --prefix zig-out-arm --summary all` | passed after the portable-environment correction; 68/68 Linux ARM64 cross-build steps |
| WSL `qemu-aarch64` over every `zig-out-arm/bin/gravity-*-aarch64` | passed; all 22 ARM64 golden, feature, rollback, pipeline, and benchmark binaries executed |
| `zig build demo-test -j1 --summary all` | passed after removing the nested-build deadlock; 17/17 steps and 5/5 Playwright tests over all 15 cases |
| `zig build demo-isolation -j1 --summary all` | passed; 2/2 steps |
| `node tools/qualification_audit.mjs` | passed; Tasks 00-27, 18 product documents, source scan, versions, SBOM, and six CI targets verified |
| `zig fmt --check build.zig src tests tools` and `git diff --check` | passed |

The native performance gate reported `budget_pass: true` for both frozen
scenes. The Medium scene measured 52.8331 ms Tick p50, 63.9417 ms p95,
0.6862 ms snapshot p95, and 479.1351 ms eight-Tick rollback p95. Tick-time
allocation remained zero. The worker gate produced the same deterministic
result for serial and 1/2/4/8 workers; it recorded 1.385x at 8 workers for the
Medium scene and 1.232x for Stress on this host.

## Release and consumer validation

- Two detached clean builds used separate install prefixes, local caches, and
  global caches. Each `release-check` passed 58/58 steps and independently
  verified all nine package hashes.
- `node tools/release.mjs --compare <first> <second>` verified both manifests
  and then compared every tar byte; all nine packages were byte-identical.
- The source tar was extracted outside the repository and passed
  `zig build demo-isolation -j1 --summary all` (2/2).
- The Windows x86-64 binary tar was extracted outside the repository. The C11
  consumer compiled using only the package's `include/gravity.h` and
  `lib/gravity_static.lib`, then ran successfully with exit code zero.

## Current pin requalification

The July 20 requalification used Spindle
`45aab8adf5f89500f6196b383265a5f9826312c2` in an isolated clean worktree.

| Command | Result |
|---|---|
| `zig build spindle-check-all-modes -j1 --summary all` | passed; 17/17 steps and 33/33 tests |
| `zig build security-gate -j1 --summary all` | passed; 124/124 steps and 80/80 tests |
| `zig build qualification-native -j1 --summary all` | passed in 1183.4 s; 376/376 steps and 724/724 tests |
| `zig build performance-gate -j1 --summary all` | passed; 20/20 steps, all six 64-sample scene budgets true, zero Tick allocations |
| `zig build release-check -j1 --summary all` | passed; 58/58 steps and all nine deterministic packages/checksums verified |
| `zig build product-qualification -j1 --summary all` | passed in 1546.4 s; native, WASM, million-Tick WASM, ABI WASM, Demo, and reproducible release gates |

Qualification exposed and fixed an ordering defect in the combined local gate:
the C# ABI consumer formerly wrote tracked `bin/obj` outputs before the release
cleanliness check. Its .NET artifacts now live under `zig-out/dotnet-artifacts`;
`abi-csharp-smoke` passed 5/5 steps and left the tracked worktree clean before
the successful combined qualification.

## Platform evidence boundary

| Target | Evidence | Status |
|---|---|---|
| Windows x86-64 | native host `qualification-native`, Demo, ABI consumers, release consumer | passed |
| Linux ARM64 | complete musl cross-build and QEMU execution of 22 binaries | passed as emulated ARM execution; native hosted runner pending |
| WASM/WASI | Wasmtime feature/long-run suite and Node freestanding ABI | passed |
| Windows ARM64 | release cross-build and configured `windows-11-arm` job | native hosted runner pending |
| Linux x86-64 | release cross-build and configured `ubuntu-24.04` job | native hosted runner pending |
| macOS x86-64 | release cross-build and configured `macos-15-intel` job | native hosted runner pending |
| macOS ARM64 | release cross-build and configured `macos-15` job | native hosted runner pending |

The hosted labels follow the current GitHub-hosted runner table:
<https://docs.github.com/en/actions/reference/runners/github-hosted-runners>.
Cross-compilation, QEMU execution, and workflow configuration are not recorded
as substitutes for the required native hosted runs.

## Completion boundary

No source, package, local runtime, documentation, or release-assembly item is
known incomplete. The Task 28 `product-ready` decision remains open until the
checked-in workflow runs successfully on all six native jobs, the WASM job, the
Demo job, and the two-clean-build release job. No product-ready tag or release
may be created from this report alone.

