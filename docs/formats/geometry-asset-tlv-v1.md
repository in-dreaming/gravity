# Geometry asset TLV v1

This document freezes the binary boundary between the offline geometry baker
(Task 06) and `GravityAssetStore` (Task 05).  It is part of asset format v1.
All integers are little-endian; no native struct layout, padding, or alignment
is serialized.

## Envelope

Each geometry asset is one Task 04 TLV stream: `u16 format_version = 1`,
`u16 section_count`, then strictly increasing `u16 tag, u32 byte_length,
payload` records.  Payloads are packed byte streams with no alignment padding.
Unknown optional records are ignorable; every tag below is required and has bit
15 set.  Empty arrays are encoded as their `u32 count` only.

The BLAKE3 content hash is `BLAKE3("gravity/asset/v1\\0" || complete_tlv)`.
It includes the version, record count, record headers, every payload byte and
any future optional record; it does not include a manifest entry, filesystem
name, source JSON, or an enclosing asset-set hash.  References are the full
32-byte content hash, never an address or a process-local ID.

## Required common record

`0x8001 GEOMETRY_HEADER` (16 bytes): `u8 kind`, `u8 schema_version=1`,
`u16 flags`, `u64 source_id`, `u32 reserved=0`.  `source_id` is a stable
canonical JSON integer and is diagnostic only; identity and deduplication use
the content hash.  Kinds are `1 ConvexHull`, `2 TriangleMesh`, `3 HeightField`,
and `4 Compound`.  Reserved bits and fields must be zero and rejected when
nonzero.

## Geometry records

`0x8002 POSITIONS`: `u32 count`, then `count` triples of `i64 x,y,z` Q32.32.
Vertex order is the baker's canonical order.

`0x8003 TRIANGLES`: `u32 count`, then `count` triples of `u32` position
indices.  A triangle's primitive ID is its zero-based canonical array index.

`0x8004 BVH_NODES`: `u32 count`, then nodes in preorder.  A node is 64 bytes:
six `i64` bounds (`min xyz,max xyz`), `u32 first`, `u16 count`, `u8 axis`,
`u8 flags`, `u32 reserved`.  A leaf has `flags=1`, `count in 1..4`, `first`
into `BVH_PRIMITIVES`, and `axis=0`.  An internal node has `flags=0`,
`count=0`, `first` equal to its left child node index, and `axis in 0..2`; its
right child is `first+1`.  Internal nodes are emitted with adjacent children.

`0x8005 BVH_PRIMITIVES`: `u32 count`, then `u32 primitive_id` values.  The
array has exactly the triangle count and each canonical primitive occurs once.

`0x8006 HEIGHT_SAMPLES`: HeightField only: `u32 width,u32 height`, then
`width*height` i64 Q32.32 samples in row-major `(z,x)` order.

`0x8007 HEIGHT_CELLS`: HeightField only: `u32 count`, then one `u8 flags` and
one `u32 material_id` per cell in row-major order. Bit 0 is the hole flag;
all remaining bits are zero.

`0x8008 COMPOUND_CHILDREN`: Compound only: `u32 count`, then in canonical
child order: `u32 child_ordinal`, 32-byte child content hash, `i64 position[3]`,
and `i64 quaternion[4]`.  The quaternion is normalized and canonical under
Task 02.  `child_ordinal` starts at zero and is contiguous.  A child reference
must resolve in the same verified asset set; the loader verifies no cycle,
depth <= 8, and direct count <= 256 before publishing the store.

`0x8009 HULL_FACES`: ConvexHull only: `u32 count`, followed by `count`
records of `u32 first_half_edge,u32 half_edge_count`.  Faces and their edge
rings are in canonical lexicographic order `(lowest vertex, normal, ring)`;
each face has at least three edges.

`0x800a HALF_EDGES`: ConvexHull only: `u32 count`, followed by `count`
records of `u32 origin,u32 twin,u32 next,u32 face`.  Every edge has exactly
one reciprocal twin; `next` closes exactly the face ring named by `face`.
This representation is intentionally index-only: face planes are recomputed
from the immutable positions with Task 02 wide arithmetic.

`0x800b MASS_PROPERTIES`: ConvexHull and TriangleMesh only: 80 bytes,
ten `i64` Q32.32 values in order `volume, center.x, center.y, center.z,
inertia.xx, inertia.yy, inertia.zz, inertia.xy, inertia.xz, inertia.yz`.
`volume` is positive.  Inertia is about `center`, symmetric, and is computed
by the baker with wide accumulation.  TriangleMesh may omit this record only
when it is not a closed oriented manifold; its runtime user must then supply
a separately validated override.

`0x800c HEIGHT_TILE_TREE`: HeightField only: `u32 tile_width,u32
tile_height,u32 node_count`, then `node_count` 64-byte BVH_NODES.  Tiles are
row-major `(tile_z,tile_x)` and are no larger than 64x64 samples.  Leaves use
the same flags and child convention as `BVH_NODES`; their primitive range is
the canonical non-hole cell triangle range.

ConvexHull requires positions, triangles, faces, half-edges, and mass
properties. TriangleMesh requires positions, triangles, BVH nodes and
primitives, and includes mass properties iff it is a closed oriented
manifold. HeightField requires samples, cells, and its tile tree. Compound
requires children and BVH nodes; its primitive IDs are child ordinals. A kind
may not contain records prohibited by its schema, and all required records
must occur exactly once.

The writer serializes tags in numeric order.  Canonical ordering is performed
before serialization: source IDs/child ordinals order source objects, vertex
and triangle normalization is Task 06's responsibility, and BVH ties use
`(cost, axis, split, primitiveId)`.  The loader never repairs or reorders data.
