# ADR 0024: deterministic performance layout and scheduling

Status: accepted for Task 24.

## Context

Phase and detail observers showed three dominant avoidable costs: solver islands repeatedly scanned all contacts and rows; island construction repeatedly traversed the same graph; and stable constraint ordering used quadratic insertion sort. Contact PGS also recomputed world inertia, lever arms, bases, and effective masses inside every iteration. JointHeavy spent more than three seconds building rows before sorting was corrected, while the old Medium contact PGS was roughly 668 ms.

## Decision

- Keep canonical public order and hashes unchanged. Partition contacts and joint rows into caller-owned stable per-island index arrays, then solve indexed ranges.
- Build islands with caller-buffered union-find and retain canonical body/member ordering. A regression test compares it with the former BFS oracle.
- Replace quadratic stable insertion sort with a caller-buffered stable block merge sort. Do not use an unstable library sort.
- Prepare contact basis, body indices, world inertia, lever arms, friction, and effective mass once per substep; PGS consumes the cache.
- Skip wake-graph traversal only after proving every dynamic body is already awake and validating every edge/request.
- Preflight persistent contact-cache output using exact live merge counts rather than the old worst-case double capacity.
- Keep Spindle work-stealing as the production parallel adapter. Serial is the canonical oracle and FixedPool is a diagnostic baseline only.
- Do not enable SIMD: cross-target bit identity has not been proven. Do not change protocol because no operation/result order exposed by the deterministic contract changed.

All workspaces are initialized before Tick; the benchmark freezes its allocator limit and proves zero Tick allocations.

## Consequences

Medium local Spindle P50 is about 59 ms and Stress about 360 ms at eight workers. JointHeavy local P50 fell to about 408 ms after the stable merge sort. Remaining profile concentration is expected in CCD casts, contact preparation, joint row construction, PGS, and serial merge/hash phases. Future work may partition those phases only if stable publication order and golden hashes remain unchanged.

Machine-readable phase/profile and executor evidence uses `gravity.performance.v1`, `gravity.profile.v1`, `gravity.profile-detail.v1`, `gravity.executor-overhead.v1`, and `gravity.wasm-performance.v1`.
