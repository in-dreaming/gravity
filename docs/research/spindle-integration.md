# Spindle Integration Decision

Gravity pins `in-dreaming/spindle` as `third_party/spindle` at a reviewed commit.
Spindle is MIT licensed, targets Zig 0.16.0, and supplies the native executor
primitives needed by Gravity's deterministic job system.

Reviewed dependency record:

- gitlink commit: `2a1f5e074fcc4c63a61e4b7437c288a2db9bb24b`;
- license: MIT, preserved verbatim at `third_party/spindle/LICENSE`;
- imported entry: `third_party/spindle/src/executor.zig` only;
- qualification gate: `zig build spindle-check-all-modes`.

## Decision

Use Spindle as the native execution substrate behind a Gravity-owned batch
dispatcher. Gravity imports only Spindle's public `src/executor.zig` /
`spindle_executor` entry point. The aggregate Runtime, parallel algorithms,
Local Task Graph, ECS, Resource Graph, Workflow, SQLite, archive, I/O, and
observability surfaces stay outside Gravity's dependency graph.

Gravity owns:

- ordered physics work lists, logical job/range ownership, staging, and stable merge;
- GRAVSNAP, GRAVREPL, rollback storage, state hashes, and field diff;
- capacity preflight, fault publication, Tick commit, C ABI batch descriptors,
  and reentrancy rules;
- the serial oracle and all cross-worker determinism assertions.

Spindle owns:

- native worker lifecycle, queues, barriers/help-until behavior, and shutdown;
- type-erased executor dispatch and intrusive Task lifecycle;
- work-stealing production execution;
- FixedPool timing baselines and DeterministicExecutor schedule reproduction for
  diagnostics only.

## Allocation, lifetime, and determinism audit

`WorkStealingExecutor` allocates queues and worker state at initialization; its
submitted `Task` is intrusive and caller-owned. Gravity initializes it outside
World ticks and preallocates one Task/context/completion/fault record per
logical job slot. A Task is reusable only after completion,
`waitQueueReleased`, and `Task.reset`; completed state alone is not a safe reuse
barrier.

`DeterministicExecutor` grows ArrayLists while recording submissions and is only
a schedule recorder/reproducer, not the physics determinism oracle.
`parallel.forRange` allocates Task storage per invocation and
`LocalTaskGraph.start` allocates runtime nodes/state/counters, so neither is part
of Gravity runtime or tooling dependencies. The serial Gravity dispatcher is
the golden oracle.

Task completion order is unspecified. Callbacks write only staging owned by
their logical job/input range; actual worker ID cannot select output layout,
capacity, overflow behavior, or merge order. Callbacks must not append, allocate
IDs, update shared impulses, or emit simulation events in completion order.

Spindle scheduling decisions do not make unordered physics mutations
deterministic. Gravity preassigns output ownership and commits only after a
successful barrier using canonical keys.

## Build integration

Gravity creates a module directly from
`third_party/spindle/src/executor.zig`. This public narrow entry has no
`build_options` dependency and cannot expose Runtime or upper-level models.
`zig build spindle-check` validates both the executor behavior and absence of
aggregate/parallel/task-graph declarations.
`zig build spindle-check-all-modes` repeats the surface in Debug, ReleaseSafe,
and ReleaseFast.

The submodule commit may only advance after an explicit dependency review of
license, Zig version, executor API, Task reset/queue-release semantics, shutdown,
allocation sites, and Gravity's determinism matrix. Gravity pins an exact commit,
never a branch tip.

## Refactoring map

Task 21 snapshot, rollback, replay, hashes, and diff remain Gravity code. Replay
may use the dispatcher to execute a Tick, but it never serializes Spindle task
pointers, queue order, worker IDs, or scheduler logs.

Task 20 phase code is split at ordered batch boundaries. Numerical kernels keep
their serial operation order. Fixed-size outputs use logical input slots;
variable-size outputs use count → canonical prefix sum → fill or fixed input
slots → canonical compact. The serial backend remains the golden oracle.

The production `stepWithAnalyticSolver` path dispatches prevalidate, commit,
integrate, broadphase, narrowphase, islands, solve, CCD, sleep, events, and hash
as synchronous Gravity batches. Task 22A deliberately assigns one logical
range to each existing phase so adoption cannot change numerical order; Task 23
may subdivide only after it supplies the documented staging and merge proof.

Task 22 exposes synchronous
`dispatch_batch(user, job_count, run_job, batch_context)`. It executes every
logical index exactly once and returns only after the barrier. Descriptors are
borrowed for the call and cannot escape. Task 22A maps this contract to serial,
Spindle, and host backends; Spindle types never cross the C ABI.

The ABI World allocates its host exact-once accounting slab from caller memory.
The native adapter receives a caller-owned intrusive Task/context/fault slab at
host/World construction and performs no Tick allocation. Reuse is always
completion wait → queue-release wait → reset → generation/context update.

Task 22A qualifies the adapter, preallocated Task slab, lifetime, capacity, and
atomic publication rules. Task 23 parallelizes physics phases through that
contract. Task 24 profiles the result, and Task 25 records Spindle in the SBOM
and fuzzes the Gravity/Spindle boundary rather than duplicating all upstream
executor fuzzing.
