# Gravity 1.0 release checklist

- [ ] Clean checkout and recursive submodules at the pinned Spindle revision.
- [ ] Zig 0.16.0, Node 22.22.2, and pnpm 11.7.0 exact versions.
- [ ] TODO/mock/stub/skip audit has no unexplained production hit.
- [ ] Debug, ReleaseSafe, and ReleaseFast subsystem and ABI suites pass.
- [ ] Native Windows/Linux/macOS x86-64/ARM64 qualification jobs pass.
- [ ] Serial and 1/2/4/8-worker Tick hashes match; TSAN passes on Linux.
- [ ] WASI suite, one-million-Tick run, and freestanding ABI smoke pass.
- [ ] 100k rollback, complete shape/joint/query/CCD corpus, fuzz and security gates pass.
- [ ] Frozen performance budgets and zero Tick allocations pass.
- [ ] Demo cold build and all 15 case/reset/rollback/screenshot tests pass.
- [ ] C11, C++17, C#, WASM, and source-package consumers pass without repository sources.
- [ ] `zig build release-check -j1` emits nine packages, SPDX SBOM, manifest, and SHA256SUMS.
- [ ] Two independent clean release builds produce byte-identical packages.
- [ ] API, ABI, formats, integration, determinism, limits, security, license, and SBOM docs reviewed.
- [ ] Task 00-27 audit has no unfinished required item; Task 28 report records exact evidence.
