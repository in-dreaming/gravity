# Gravity C ABI v1

`include/gravity.h` is the only stable public ABI. All exported names start with
`gravity_v1_`; Zig declarations and C/C++ static assertions freeze the scalar
layouts. The ABI, protocol, snapshot, and asset format versions are independent.

## Ownership and memory

- `GravityAssetStore` and `GravityWorld` are opaque and live in aligned memory
  supplied by the caller. Query the exact required byte count and alignment
  before initialization.
- Asset bytes are validated completely and copied into the store. A store may
  be shared read-only by multiple worlds and must outlive them.
- Commands, query inputs, output buffers, snapshot buffers, dispatch callbacks,
  and callback contexts are borrowed only for the duration of one call.
- A Tick performs no general heap allocation. Capacity exhaustion is explicit;
  no API truncates or grows storage implicitly.

## Extension and failure rules

Extensible descriptors begin with `struct_size` and `reserved`. Callers set
`struct_size` to `sizeof(the struct)` and every reserved field to zero. New ABI
minor revisions may append fields; v1 readers accept a larger `struct_size` but
never read beyond the v1 prefix. Math values, IDs, hashes, and fixed query values
have frozen layouts and are not extensible descriptors.

`GravityWorldDesc` retains its original 96-byte v1 prefix. Its appended
capability tail opts a World into caller-buffered joint, sleep, CCD, and
diagnostics state and supplies joint capacity; a 96-byte descriptor preserves
the original behavior and reference hash. The extension is additive: existing
function signatures and field offsets are unchanged.

Every function returns a stable `GravityResult`. Recoverable failures never
panic or unwind across the ABI. Buffer APIs always publish the required count;
if the supplied capacity is too small, no partial output is published.

## Batch dispatch

`GravityDispatchBatchFn` is synchronous. It must call `run_job(batch_context,
job_index)` exactly once for every index in `[0, job_count)` and return only after
all calls finish. It must not retain any pointer, call a World API reentrantly,
or return success after a worker failure. Gravity validates missing, duplicate,
out-of-range, failed, and reentrant execution before publishing Tick state.

Task 22A may attach Spindle behind this contract. Spindle types, queues, tasks,
allocators, and worker identities never cross the C ABI.

## Snapshot and query semantics

Snapshot load is a two-pass transaction bound to the world's protocol,
configuration, and asset-set hash. Extended Worlds include joint impulses,
sleep state, and per-body CCD policy. Invalid input leaves the destination World
unchanged. Ray, point, AABB, convex overlap, and convex shape-cast queries use
the production query implementation and return canonical hit order.
Asset-backed traversal scratch is part of caller-provided World memory.

Joint creation supports Distance, Ball-Socket, Hinge, Slider, Fixed, and
Cone-Twist constraints with optional limit, motor, spring, and cone/twist
fields. `gravity_v1_world_stats` publishes derived counts and deterministic
phase visits; it is diagnostic output and does not participate in state hashes.

The machine-readable ABI baseline is `tests/abi/abi-baseline-v1.json`.
