# Task 25 fuzz qualification

`zig build fuzz` is the fast deterministic seed gate. `zig build fuzz-all-modes -j1` replays every committed property and minimized corpus in Debug, ReleaseSafe, and ReleaseFast. `zig build security-gate -j1` additionally runs the C/C++/C# ABI consumers, ABI mutation/transaction corpus, Spindle lifecycle corpus, executor-only build-graph audit, and Linux ThreadSanitizer target.

Coverage groups:

- asset JSON/TLV, snapshot, replay, command batches, decimal, C ABI pointer/length/alignment/command/query;
- fixed-point/core IDs, geometry/BVH, analytic triangle symmetry, GJK/EPA and mesh-mesh candidate ordering;
- create/destroy/joint/snapshot/100k rollback tests from the state suites plus worker/backend switching from the Task 21/23 corpus;
- Spindle submit, completion, queue release, reset, cancel, callback failure, backpressure, shutdown, stale handle, and one-million reuse sequence.

The coverage-guided parser entry uses Zig 0.16 `std.testing.fuzz`. Its empty generated seed is complemented by the committed deterministic corpus below. The Task 25 acceptance budget is 10 million generated inputs on Linux; release qualification raises this to one CPU-hour for each supported native target, followed by deterministic three-mode replay. CI freezes the explicit loop counts in source (currently more than 60,000 parser/decimal probes, 20,000 triangle pairs, 125 GJK/EPA offsets, 1,000 ABI descriptor mutations, up to 1,024 snapshot mutations, 10,000 World state transitions, and 2,000 Spindle sequences per mode). Timeouts, OOM, and `internal` results are failures.

Zig 0.16 does not implement `--fuzz` on Windows. Run the coverage-guided target under Linux/WSL with cache directories on the Linux filesystem (an NTFS-hosted Zig cache cannot atomically rename every entry):

```sh
zig build fuzz-instrumented --fuzz=10M -j1 --summary all \
  --cache-dir /tmp/gravity-local-cache \
  --global-cache-dir /tmp/gravity-zig-cache
```

Windows remains a required native target for `fuzz-all-modes`, ABI consumers, and the build-graph audit. `jobs-tsan` executes on Linux and cross-compiles on other hosts.

Use `zig build fuzz-minimize -- <asset|asset_tlv|snapshot|replay|commands> <input> <output>` to remove chunks while preserving the exact error class. Commit the minimized file under `tests/fuzz/corpus/` and add an embedded regression assertion; never overwrite a golden automatically.
