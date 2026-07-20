# Task 24 performance report

## Frozen contract

The corpus is defined in `tools/task24_corpus.zig`; changing a scene or budget is a reviewed benchmark-protocol change. Every scene uses the product defaults for CCD, sleeping, two substeps, ten velocity iterations, and four position iterations. No pair, contact, joint, shape, or iteration is disabled for a result.

The strict reference runner is an Intel Core i9-14900K (24 cores/32 logical processors), Windows 11 23H2 build 22631, Zig 0.16.0, ReleaseFast, all logical processors enabled, fixed high-performance power mode, Windows High process priority, process affinity fixed to the reference machine's P-core logical processors 0-15, no competing workload, 8 warmups, and 64 measured samples. Fixed-runner mode sets its own Windows process priority and affinity before setup so worker threads inherit the controlled scheduling class and cannot migrate between heterogeneous P/E cores. Strict runs use `zig build performance-gate`. Shared CI uses `zig build performance-ci`: it validates the same schema at normal priority and unrestricted affinity, and rejects a result only beyond a 2x noise band. Local numbers below are qualification evidence, not a replacement for the controlled runner.

## Corpus and budgets

| Scene | Bodies | Contacts | Joints | Tick P95/P99 budget | Snapshot P95 | 8-tick rollback P95 | Workspace |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Small | 32 | 48 | 16 | 4 / 6 ms | 2 ms | 16.667 ms | 64 MiB |
| Medium | 2,500 | 5,000 | 512 | 100 / 120 ms | 12 ms | 750 ms | 512 MiB |
| Stress | 8,192 | 28,672 | 8,192 | 450 / 550 ms | 40 ms | 8 s | 2 GiB |
| MeshHeavy | 257 | 1,186 | 1 | 300 / 350 ms | 8 ms | 2.5 s | 256 MiB |
| JointHeavy | 2,048 | 245 | 8,192 | 600 / 700 ms | 20 ms | 7 s | 768 MiB |
| CCD | 1,024 | 512 | 1 | 2.5 / 3 s | 16 ms | 20 s | 384 MiB |

## Local qualification

The July 20 strict run used the frozen reference CPU/OS, High priority, P-core affinity, high-performance power mode, no competing workload, 8 warmups, and 64 measured samples for every scene. `zig build performance-gate -j1 --summary all` completed 20/20 steps and every result below reported `budget_pass: true`. The checked-in JSONL preserves the exact observations.

| Scene, 8 workers | P50 | P95 | P99 | Snapshot bytes / P95 | 8-tick rollback P95 | Workspace |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Small | 0.895 ms | 0.950 ms | 1.122 ms | 18,724 / 0.008 ms | 7.888 ms | 3.08 MB |
| Medium | 48.180 ms | 52.506 ms | 55.921 ms | 1,166,216 / 0.932 ms | 467.669 ms | 124.74 MB |
| Stress | 232.073 ms | 256.462 ms | 293.430 ms | 6,152,612 / 4.734 ms | 2.398 s | 568.74 MB |
| MeshHeavy | 132.522 ms | 142.016 ms | 145.923 ms | 101,439 / 0.045 ms | 1.182 s | 29.75 MB |
| JointHeavy | 395.478 ms | 544.980 ms | 558.652 ms | 3,738,020 / 2.954 ms | 4.062 s | 133.40 MB |
| CCD | 1.073 s | 1.384 s | 1.576 s | 402,858 / 0.245 ms | 12.430 s | 59.79 MB |

All six runs reported zero Tick allocations and reproduced the serial canonical hash.

## Worker scaling

`performance-scaling` runs Medium and Stress with 1/2/4/8 Spindle workers. Speedup and efficiency below use local P50 because the diagnostic run has two samples; fixed-runner acceptance is the absolute 8-worker P95 budget above. The explicit throughput target is at least 1.8x Medium and 2.0x Stress speedup at 8 workers.

| Scene | Workers | P50 | Speedup | Efficiency |
| --- | ---: | ---: | ---: | ---: |
| Medium | 1 / 2 / 4 / 8 | 117.038 / 80.699 / 61.216 / 58.978 ms | 1.00 / 1.45 / 1.91 / 1.98x | 100 / 72.5 / 47.8 / 24.8% |
| Stress | 1 / 2 / 4 / 8 | 800.244 / 549.237 / 399.573 / 360.033 ms | 1.00 / 1.46 / 2.00 / 2.22x | 100 / 72.9 / 50.1 / 27.8% |

Scaling flattens because broadphase, state hash, stable merge, and portions of contact preparation remain serial; the executor does not hide those costs.

## Executor overhead and WASM

For 4,096 caller-owned tasks, serial-inline submit cost was 32 ns/task, FixedPool diagnostic 479 ns/task, and Spindle work-stealing 406 ns/task. Spindle shutdown was 0.338 ms; all eight workers were active with 681-1,270 tasks per worker. FixedPool remains a diagnostic comparator, not a production backend.

Wasmtime 46.0.1 executed the real 48-body/72-contact product pipeline for 120 ticks at 5.836 ms average per tick, below the 16.667 ms WASM budget. `wasm-validate` also runs the WASI golden and subsystem suites.

Raw evidence: `docs/performance/task24/local-qualification.jsonl`. The flamegraph-compatible folded profile is `docs/performance/task24/medium-8w.folded`.
