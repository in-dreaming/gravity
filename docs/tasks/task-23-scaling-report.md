# Task 23 deterministic job scaling report

## Reproduction

```powershell
zig build job-scaling
```

The harness creates one canonical World snapshot per scene, warms every
backend for 12 ticks, restores the snapshot, and measures 120 complete C ABI
ticks. The serial result is the oracle. Spindle 1/2/4/8 workers must end with
the exact same `gravity_v1_world_hash`; a mismatch fails the run instead of
publishing timing data. Tick timing includes the C ABI rollback snapshot,
physics phases, stable merges, and dispatcher barriers.

## Reference run

- Revision: `959e0b3` plus the report-only cleanup that removed progress text.
- Date: 2026-07-19.
- CPU: Intel Core i9-14900K, 24 cores / 32 logical processors.
- OS: Windows 11 Pro 10.0.22631.
- Zig: 0.16.0, `ReleaseFast`, baseline CPU target.
- Power mode and CPU affinity were not pinned, so these values establish
  functional scaling evidence rather than Task 24 regression thresholds.

| Scene | Dynamic bodies | Backend workers | ns/tick | Speedup vs serial |
|---|---:|---:|---:|---:|
| Medium | 2,048 | serial | 4,541,045 | 1.000x |
| Medium | 2,048 | 1 | 4,545,816 | 0.999x |
| Medium | 2,048 | 2 | 3,509,871 | 1.294x |
| Medium | 2,048 | 4 | 3,485,363 | 1.303x |
| Medium | 2,048 | 8 | 3,700,075 | 1.227x |
| Stress | 16,384 | serial | 119,178,949 | 1.000x |
| Stress | 16,384 | 1 | 150,978,820 | 0.789x |
| Stress | 16,384 | 2 | 131,428,921 | 0.907x |
| Stress | 16,384 | 4 | 114,508,522 | 1.041x |
| Stress | 16,384 | 8 | 113,442,990 | 1.051x |

A second clean-worktree run on the same machine retained Medium acceleration
but exposed the expected unpinned-system variance at Stress scale:

| Scene | serial ns/tick | 2 workers | 4 workers | 8 workers |
|---|---:|---:|---:|---:|
| Medium | 4,517,272 | 1.256x | 1.208x | 1.197x |
| Stress | 142,694,069 | 0.911x | 0.941x | 0.970x |

## Interpretation and boundary

The repeated runs demonstrate real multi-worker acceleration without changing
the state hash: Medium reaches 1.208x–1.303x. One-worker overhead is visible
and is not hidden by substituting another executor. Stress is honestly at the
noise/parity boundary (0.970x–1.051x at eight workers), because each public C
ABI tick first serializes the full rollback snapshot; at 16,384 bodies that
serial boundary dominates the parallel integration ranges. Task 23 therefore
establishes working parallel acceleration and reports the remaining serial
limit rather than claiming monotonic scaling.

These two allocation-free scenes isolate Task 23 range ownership and executor
cost. They are not the frozen Task 24 product benchmark corpus: contacts,
joints, MeshHeavy, CCD, percentile sampling, affinity, power mode, and CI noise
bands remain owned by Task 24.
