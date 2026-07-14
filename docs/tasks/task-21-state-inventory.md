# Task 21 State Inventory

This inventory is the single serialization checklist for Task 21.  A field is
either a logical snapshot field or explicitly derived after load; no third
category is allowed.

## GRAVSNAP sections

Sections are canonical ascending `u16` TLVs under the existing codec framing.
The outer header additionally carries the `GRAVSNAP` magic, snapshot format,
protocol version, encoded simulation configuration and asset-set hash.  All
sections below are required for this protocol version.

| id | section | logical fields |
| --- | --- | --- |
| 0x8001 | pipeline | tick, fault tick/phase/object/code/detail/math fault |
| 0x8002 | bodies | settings; every body slot's type, transform, velocities, inverse mass/inertia, force, torque, DOF locks, generation, alive/retired and kinematic target |
| 0x8003 | colliders | every collider slot's owner, local transform, shape, material, filter, sensor/enabled flags, revision, generation, alive/retired |
| 0x8004 | contacts | ordered contact-cache patches and warm impulses |
| 0x8005 | joints | every joint slot, frames, limits, motor/spring/cone settings, limit state and all accumulated impulses plus generation/alive/retired |
| 0x8006 | sleep | awake, counter and wake reason for every body slot |
| 0x8007 | ccd | per-body CCD enabled state |

`SAP` endpoints, proxy bounds, BVH traversal stacks, island memberships,
constraint rows, solver temporary impulses and phase diagnostics are derived
scratch and are not serialized.

## Canonical tagged values

Every enum is written as its frozen `u8` discriminant after range validation;
booleans use the codec's exact `0`/`1` representation.  A collider shape is
written as `ShapeKind` followed by exactly one payload: Sphere=`radius`,
Box=`half_extents`, Capsule=`radius,half_height`, and each immutable asset
shape=`source_id,AssetId,revision`.  Material is `friction,restitution`.
Transforms always write position followed by canonical quaternion.  Decoding
must reject an invalid discriminant, noncanonical quaternion, illegal dynamic
surface shape, or any stale/non-live ID reference before commit.

## Load contract

1. The first pass checks outer metadata, section order/completeness, exact
   capacities, enum/bool ranges, ID references, shape legality, canonical
   quaternions and contact ordering without mutating any destination state.
2. The second pass decodes into caller-supplied staging columns, verifies the
   same cross-section references, then copies every persistent column as one
   commit.  A failure leaves the complete destination unchanged.
3. Load clears `pipeline.in_step`, restores the recorded fault verbatim, and
   leaves only documented derived scratch for the next fixed pipeline rebuild.

## GRAVREPL records

Replay stores a validated initial `GRAVSNAP`, then canonical command batches
and the expected full/section hashes per tick.  A mismatch is located by binary
search over recorded hashes and reported as the first section, ID and field
whose canonical encoding differs.
