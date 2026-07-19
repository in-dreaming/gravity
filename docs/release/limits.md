# Product limits and unsupported behavior

## Defaults

Worlds default to 60 Hz, two substeps, 10 velocity and four position iterations,
8,192 bodies, 16,384 colliders and commands, 8,192 joints, 131,072 broad pairs,
32,768 contact patches, 131,072 contact points/events, and a 120-Tick rollback
window. Capacity, tolerance, and iteration values participate in the config
hash. Exhaustion is an explicit deterministic failure; storage never truncates
or grows implicitly during a Tick.

The supported runtime envelope is documented in `docs/tasks/setup.md`: positions
within +/-1,000,000 m, dynamic sizes 0.001-100,000 m, linear speed up to
100,000 m/s, and angular speed up to 1,000 rad/s. The fixed-point envelope
validator remains authoritative for compound expressions, mass, inertia, and
impulse inputs.

## Geometry and CCD

TriangleMesh topology and HeightField samples are immutable at runtime.
Dynamic meshes must be baked as closed, consistently oriented manifold meshes
or provide a validated mass override. Convex shapes and convex Compound leaves
may cast continuously against every supported target. TriangleMesh is not a CCD
caster and continuous mesh-mesh CCD is not supported; dynamic mesh-mesh remains
fully supported in the discrete pipeline.

## Platform and compatibility

Native release targets are Windows, Linux, and macOS on x86-64 and ARM64.
WASM is `wasm32-freestanding` and single-worker. The core has no libc/libm
dependency. GPU solving, fluids, cloth, soft bodies, runtime deformable/fracture
geometry, and network transport are outside Gravity 1.0.

Canonical simulation compatibility is guaranteed only within protocol v1 with
identical inputs. C ABI v1 can evolve through size-tagged additive descriptor
tails, but old callers do not gain new opted-in World capabilities implicitly.
