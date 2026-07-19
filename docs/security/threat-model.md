# Gravity threat model and resource limits

## Trust boundaries

Untrusted data enters through canonical asset JSON, baked asset TLV, GRAVSNAP, GRAVREPL, decimal strings, C ABI descriptors/commands/queries, and host dispatch callbacks. Attackers may control every byte and length, repeat calls, use stale IDs, request maximum capacities, provide null/misaligned/overlapping buffers inside a controlled address space, or return malformed scheduler behavior. They cannot make an arbitrary invalid virtual address safely dereferenceable in C; callers must still provide an address range valid for the declared length.

Security objectives are: no panic/OOB/UAF/leak or unbounded recursion/allocation; bounded work before rejection; no partial World publication on malformed input or callback failure; canonical results independent of platform, mode, worker order, or address; and stable public error codes rather than `internal` for hostile wire data.

## Frozen limits

| Entry | Hard limit and allocation rule | Failure state |
| --- | --- | --- |
| source JSON | baker reads at most 16 MiB; JSON allocation is proportional to that caller-bounded file | no asset store mutation |
| baked TLV section | each payload at most 16 MiB; counts validated before slice construction | no asset publication |
| snapshot/replay section | each payload at most 16 MiB; entry/command/output arrays are caller bounded | World unchanged |
| decimal | at most 36 significant digits are useful; overflow saturates with the first deterministic MathFault | no parser allocation |
| C ABI blob/snapshot | `u64` length must fit `usize`; null is valid only for zero length; addition/multiplication checked | stable argument/capacity/corrupt result |
| C ABI output | required count is published before writes; insufficient capacity writes no element | World unchanged |
| geometry algorithms | setup.md vertex/face/cell limits; all GJK/EPA/BVH/shape-cast loops and workspaces are fixed | explicit capacity/non-convergence error |
| World sequence | configured capacities and 120-tick default rollback window; no implicit growth | command/Tick transaction rejected or deterministically Faulted |
| Spindle adapter | at most caller slab length and `submission_capacity`; synchronous barrier before reuse | no serial fallback or partial Tick publication |

## Abuse cases and controls

- Length bombs and integer wrap are checked before pointer slicing or allocation.
- Required TLV ordering, duplicate sections, trailing bytes, invalid enums/bools, and corrupt contact ordering map to `corrupt_input` at the C ABI.
- Snapshot load validates the whole envelope before committing and mutation fuzz verifies the pre-call hash on rejection.
- Query/event/body output never uses the caller buffer as scratch, so overlap with unrelated caller data cannot influence simulation.
- Callback duplication, omission, reentry, cancellation, backpressure, shutdown, and task-generation reuse are bounded and return a stable callback failure.
- Runtime asset topology is immutable; resource bombs are rejected by baker/config envelopes.

Residual risks are CPU exhaustion within an intentionally configured maximum world, malicious hosts passing unmapped pointers, and denial of service outside Gravity's synchronous call boundary. These are deployment controls, not silent engine fallbacks.
