# Gravity asset format v1

Task 05 freezes the asset-set boundary. Individual complex geometry records use
[geometry-asset-tlv-v1.md](geometry-asset-tlv-v1.md); this document defines the
container and source rules around them.

An asset file is a complete little-endian geometry TLV stream. Its content hash
is BLAKE3 over `gravity/asset/v1\0 || asset bytes`. Asset identity is this full
32-byte hash, while `source_id` is a unique, strictly increasing diagnostic key
inside an asset set. The loader rejects duplicates and never repairs ordering.

The manifest payload is `u32 count`, then `count` entries sorted by source ID:
`u64 source_id, u8 hash[32]`. Its hash is BLAKE3 over
`gravity/asset-set/v1\0 || manifest payload`. No filesystem name, pointer or
source JSON bytes participates in either hash.

Source JSON is UTF-8 and canonical by schema: object keys are ASCII snake_case;
`source_id`, counts, indices and material IDs are JSON integers; all physical
reals are decimal strings (no exponent, NaN, infinity, whitespace or float JSON
tokens). Asset kinds are `Sphere`, `Box`, `Capsule`, `ConvexHull`,
`TriangleMesh`, `HeightField`, `Compound`, and `Material`. The primitive kinds
are schema-only runtime descriptors; complex kinds are always emitted as the
frozen geometry TLV and validated before publication. A compound references a
full content hash and is resolved only within the same verified asset set.

`gravity-bake validate` validates one emitted TLV and prints its canonical hash.
`gravity-bake manifest` accepts source-ID sorted emitted TLVs, validates all of
them transactionally, and writes the manifest. Geometry construction itself is
Task 06's responsibility; this CLI deliberately does not invent topology.
