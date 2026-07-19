# Determinism and rollback contract

Gravity guarantees bit-identical canonical state when protocol version,
configuration, baked asset set, initial snapshot, and ordered Tick commands are
identical. The guarantee covers Windows, Linux, macOS, x86-64, ARM64,
freestanding/WASI WebAssembly, Debug/ReleaseSafe/ReleaseFast, and native
1/2/4/8-worker execution. WASM is serial but produces the same canonical state.

Commands are ordered by phase priority, issuer, sequence, and discriminant.
Pairs, manifolds, points, islands, rows, events, and query hits each have a
complete canonical key. Workers own logical ranges and staging slots; physical
worker identity and completion order never select output layout. Publication
occurs only after all jobs succeed.

Snapshots contain every future-relevant logical column and exclude derived
SAP, traversal, island, row, event, query, and profile buffers. Load validates
the complete input before copying, then uniquely rebuilds derived state.
Rollback may resume with a different worker count. Replay qualification compares
each Tick's complete and layered hashes, not an epsilon or final transform.

The protocol, C ABI, snapshot, and asset versions are independent. A change to
math, tolerances, ordering, algorithms, or iteration semantics requires a
protocol review and an explicit golden update. Build metadata and performance
timing are excluded from canonical simulation state.
