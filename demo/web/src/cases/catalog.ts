import { ABI } from "../wasm/abi.generated";
import type { AssetStore, BodyInput, ColliderInput, CommandInput, GravityId, JointInput, QuatRaw, TransformRaw, Vec3Raw, World, WorldOptions } from "../wasm/gravity";
import { IDENTITY, ONE, ZERO, decimalToRaw, integerRaw, transform, vec } from "../physics/fixed";
import type { CaseRuntime, DemoCase, Visual, VisualKind } from "./types";

const ALL_FEATURES = ABI.enums.worldFeature.joints | ABI.enums.worldFeature.sleep | ABI.enums.worldFeature.ccd | ABI.enums.worldFeature.diagnostics;
const FILTER = { category: 0xffff_ffff, mask: 0xffff_ffff, group: 0 } as const;
const INERTIA = { xx: ONE, yy: ONE, zz: ONE, xy: 0n, xz: 0n, yz: 0n } as const;
const FRAME = { anchor: ZERO, axis: { x: ONE, y: 0n, z: 0n }, secondary: { x: 0n, y: ONE, z: 0n } } as const;
const SLOPE_QUAT: QuatRaw = { x: 0n, y: 0n, z: -560_604_047n, w: 4_258_220_831n };

type Build = { world: World; visuals: Visual[] };

function options(bodyCapacity = 96, colliderCapacity = 128, contactCapacity = 256, jointCapacity = 64): WorldOptions {
  return { bodyCapacity, colliderCapacity, commandCapacity: 256, contactCapacity, gravity: { x: 0n, y: decimalToRaw("-9.81"), z: 0n }, linearDamping: decimalToRaw("0.02"), angularDamping: decimalToRaw("0.02"), maxLinearSpeed: integerRaw(1000), maxAngularSpeed: integerRaw(100), substeps: 2, tickHz: 60, featureFlags: ALL_FEATURES, jointCapacity };
}

function body(world: World, bodyType: number, position: Vec3Raw, dofLocks = 0, orientation: QuatRaw = IDENTITY): GravityId {
  const dynamic = bodyType === ABI.enums.bodyType.dynamic;
  const input: BodyInput = { bodyType, dofLocks, transform: transform(position, orientation), inverseMass: dynamic ? ONE : 0n, inverseInertia: dynamic ? INERTIA : { xx: 0n, yy: 0n, zz: 0n, xy: 0n, xz: 0n, yz: 0n } };
  return world.createBody(input);
}

function collider(world: World, owner: GravityId, kind: number, dimensions: Vec3Raw, friction = "0.6", restitution = "0.1", assetSourceId = 0n): GravityId {
  const input: ColliderInput = { body: owner, shapeKind: kind, flags: 0, local: transform(), dimensions, assetSourceId, friction: decimalToRaw(friction), restitution: decimalToRaw(restitution), category: 1, mask: 0xffff_ffff, group: 0, revision: 1 };
  return world.createCollider(input);
}

function visual(owner: GravityId, kind: VisualKind, size: readonly [number, number, number], color: number, wireframe = false): Visual {
  return { body: owner, kind, size, color, wireframe };
}

function addGround(build: Build, width = 12): GravityId {
  const owner = body(build.world, ABI.enums.bodyType.static, vec(0, -1, 0));
  collider(build.world, owner, ABI.enums.shapeKind.box, vec(width, 1, width), "0.8", "0");
  build.visuals.push(visual(owner, "box", [width * 2, 2, width * 2], 0x2d3d4f));
  return owner;
}

function addPrimitive(build: Build, kind: "sphere" | "box" | "capsule", position: Vec3Raw, color: number, restitution = "0.1", dofLocks = 0): GravityId {
  const owner = body(build.world, ABI.enums.bodyType.dynamic, position, dofLocks);
  if (kind === "sphere") {
    collider(build.world, owner, ABI.enums.shapeKind.sphere, { x: decimalToRaw("0.5"), y: 0n, z: 0n }, "0.6", restitution);
    build.visuals.push(visual(owner, "sphere", [1, 1, 1], color));
  } else if (kind === "box") {
    collider(build.world, owner, ABI.enums.shapeKind.box, { x: decimalToRaw("0.5"), y: decimalToRaw("0.5"), z: decimalToRaw("0.5") }, "0.6", restitution);
    build.visuals.push(visual(owner, "box", [1, 1, 1], color));
  } else {
    collider(build.world, owner, ABI.enums.shapeKind.capsule, { x: decimalToRaw("0.35"), y: decimalToRaw("0.65"), z: 0n }, "0.6", restitution);
    build.visuals.push(visual(owner, "capsule", [0.7, 2, 0.7], color));
  }
  return owner;
}

function joint(world: World, kind: number, bodyA: GravityId, bodyB: GravityId, flags = 0, overrides: Partial<JointInput> = {}): GravityId {
  const base: JointInput = { kind, flags, bodyA, bodyB, frameA: FRAME, frameB: FRAME, reference: 0n, swingReference: 0n, referenceOrientation: IDENTITY, limitMin: decimalToRaw("-0.5"), limitMax: decimalToRaw("0.5"), motorTargetVelocity: decimalToRaw("0.5"), motorMaxForce: integerRaw(20), springFrequency: integerRaw(2), springDampingRatio: decimalToRaw("0.7"), coneSwingMax: decimalToRaw("0.75"), coneTwistMin: decimalToRaw("-0.5"), coneTwistMax: decimalToRaw("0.5") };
  return world.createJoint({ ...base, ...overrides });
}

function runtime(world: World, visuals: Visual[], startupCommands: CommandInput[] = [], focus: readonly [number, number, number] = [0, 3, 0]): CaseRuntime {
  return { world, visuals, startupCommands, focus };
}

function standard(store: AssetStore, capacities?: readonly [number, number, number, number]): Build {
  const selected = capacities === undefined ? options() : options(...capacities);
  return { world: store.createWorld(selected), visuals: [] };
}

const cases: DemoCase[] = [
  {
    id: "stack-pyramid", title: "Sphere / Box Stacks", category: "Contacts", description: "Alternating stacks and a compact pyramid exercise stable broadphase and manifolds.", seed: "stack-v1", expectedTick: 4, expectedHash: "5ede974e91341090464b89f92347c2cd",
    build(store) { const b = standard(store); addGround(b); for (let y = 0; y < 5; y += 1) for (let x = 0; x < 5 - y; x += 1) addPrimitive(b, (x + y) % 2 === 0 ? "box" : "sphere", vec(x * 2 - (4 - y), y * 2 + 2, 0), 0x4cc9f0); for (let y = 0; y < 5; y += 1) addPrimitive(b, y % 2 === 0 ? "sphere" : "box", vec(-7, y * 2 + 2, 0), 0xffb85c); return runtime(b.world, b.visuals); }
  },
  {
    id: "material-ramp", title: "Friction & Restitution", category: "Contacts", description: "A rotated friction ramp and restitution array use fixed raw material inputs.", seed: "materials-v1", expectedTick: 4, expectedHash: "21bb6174ef8fc090fe0186ad1575acd2",
    build(store) { const b = standard(store); addGround(b); const ramp = body(b.world, ABI.enums.bodyType.static, vec(-3, 2, 0), 0, SLOPE_QUAT); collider(b.world, ramp, ABI.enums.shapeKind.box, { x: integerRaw(4), y: decimalToRaw("0.25"), z: integerRaw(2) }, "0.3", "0"); b.visuals.push(visual(ramp, "box", [8, 0.5, 4], 0x445b75)); for (let i = 0; i < 6; i += 1) addPrimitive(b, "sphere", vec(2 + i, 4, 0), 0xf06c9b, `0.${i + 2}`); return runtime(b.world, b.visuals); }
  },
  {
    id: "newton-cradle", title: "Newton Cradle", category: "Constraints", description: "Five suspended spheres use production distance constraints and an initial impulse.", seed: "cradle-v1", expectedTick: 4, expectedHash: "de5e65cedb4928e5b68968231563bad7",
    build(store) { const b = standard(store); const anchor = body(b.world, ABI.enums.bodyType.static, vec(0, 5, 0)); b.visuals.push(visual(anchor, "box", [7, 0.2, 0.2], 0x536b83)); const balls: GravityId[] = []; for (let i = 0; i < 5; i += 1) { const ball = addPrimitive(b, "sphere", vec(i - 2, 1, 0), 0xd9e7ef, "0.95"); balls.push(ball); joint(b.world, ABI.enums.jointKind.distance, anchor, ball, ABI.enums.jointFlag.reference, { reference: integerRaw(4) }); } const first = balls[0]; const commands: CommandInput[] = first === undefined ? [] : [{ type: ABI.enums.commandType.velocity, phasePriority: 0, issuer: 3, sequence: 0, body: first, first: vec(4, 0, 0), second: ZERO, transform: transform(), dofLocks: 0 }]; return runtime(b.world, b.visuals, commands); }
  },
  {
    id: "joint-gallery", title: "Six Joint Gallery", category: "Constraints", description: "Distance, Ball-Socket, Hinge, Slider, Fixed and Cone-Twist with limits, motor and spring.", seed: "joints-v1", expectedTick: 4, expectedHash: "5ee81fecac78ac45387cbe8618acabde",
    build(store) { const b = standard(store); addGround(b); const kinds = [ABI.enums.jointKind.distance, ABI.enums.jointKind.ballSocket, ABI.enums.jointKind.hinge, ABI.enums.jointKind.slider, ABI.enums.jointKind.fixed, ABI.enums.jointKind.coneTwist] as const; for (const [index, kind] of kinds.entries()) { const fixed = body(b.world, ABI.enums.bodyType.static, vec(index * 3 - 8, 5, 0)); const moving = addPrimitive(b, index % 2 === 0 ? "box" : "capsule", vec(index * 3 - 8, 3, 0), 0x65d6ad); const flags = kind === ABI.enums.jointKind.coneTwist ? ABI.enums.jointFlag.coneTwist | ABI.enums.jointFlag.spring : ABI.enums.jointFlag.limit | ABI.enums.jointFlag.motor | ABI.enums.jointFlag.spring; joint(b.world, kind, fixed, moving, flags); } return runtime(b.world, b.visuals); }
  },
  {
    id: "ragdoll", title: "3D Ragdoll", category: "Constraints", description: "A seven-body articulated figure combines ball, hinge and cone-twist joints.", seed: "ragdoll-v1", expectedTick: 4, expectedHash: "70dff34395111e11f6af91ad4d2c6fef",
    build(store) { const b = standard(store); addGround(b); const torso = addPrimitive(b, "box", vec(0, 5, 0), 0xe76f51); const head = addPrimitive(b, "sphere", vec(0, 7, 0), 0xf2c49b); const leftArm = addPrimitive(b, "capsule", vec(-2, 5, 0), 0x5cc8ff); const rightArm = addPrimitive(b, "capsule", vec(2, 5, 0), 0x5cc8ff); const leftLeg = addPrimitive(b, "capsule", vec(-1, 3, 0), 0x6574cd); const rightLeg = addPrimitive(b, "capsule", vec(1, 3, 0), 0x6574cd); joint(b.world, ABI.enums.jointKind.coneTwist, torso, head, ABI.enums.jointFlag.coneTwist); joint(b.world, ABI.enums.jointKind.ballSocket, torso, leftArm); joint(b.world, ABI.enums.jointKind.ballSocket, torso, rightArm); joint(b.world, ABI.enums.jointKind.hinge, torso, leftLeg, ABI.enums.jointFlag.limit); joint(b.world, ABI.enums.jointKind.hinge, torso, rightLeg, ABI.enums.jointFlag.limit); return runtime(b.world, b.visuals); }
  },
  {
    id: "hull-compound", title: "Hull & Compound Collapse", category: "Geometry", description: "Baked ConvexHull and Compound assets collide through the immutable AssetStore path.", seed: "assets-collapse-v1", expectedTick: 4, expectedHash: "e6ca20bb55d98fae6b4ed042b97f308e",
    build(store) { const b = standard(store); addGround(b); for (let i = 0; i < 8; i += 1) { const owner = body(b.world, ABI.enums.bodyType.dynamic, vec((i % 4) * 6 - 9, 3 + (i >> 2) * 5, 0)); const isCompound = i % 2 === 1; collider(b.world, owner, isCompound ? ABI.enums.shapeKind.compound : ABI.enums.shapeKind.convexHull, ZERO, "0.6", "0.1", BigInt(isCompound ? 1004 : 1001)); b.visuals.push(visual(owner, isCompound ? "compound" : "hull", isCompound ? [4, 2, 2] : [2, 2, 2], isCompound ? 0xf09f55 : 0x8c7cf0)); } return runtime(b.world, b.visuals); }
  },
  {
    id: "dynamic-mesh", title: "Dynamic Mesh–Mesh", category: "Geometry", description: "Two closed baked TriangleMesh rigid bodies take the discrete mesh–mesh path.", seed: "mesh-mesh-v1", expectedTick: 4, expectedHash: "4859f9b6e51c3143203a27dea96592be",
    build(store) { const b = standard(store); addGround(b); const commands: CommandInput[] = []; for (let i = 0; i < 2; i += 1) { const owner = body(b.world, ABI.enums.bodyType.dynamic, vec(i === 0 ? -2 : 2, 3, 0)); collider(b.world, owner, ABI.enums.shapeKind.triangleMesh, ZERO, "0.5", "0.2", 1002n); b.visuals.push(visual(owner, "mesh", [2, 2, 2], i === 0 ? 0x48cae4 : 0xff6b82, true)); commands.push({ type: ABI.enums.commandType.velocity, phasePriority: 0, issuer: 7, sequence: i, body: owner, first: vec(i === 0 ? 2 : -2, 0, 0), second: ZERO, transform: transform(), dofLocks: 0 }); } return runtime(b.world, b.visuals, commands); }
  },
  {
    id: "height-field", title: "HeightField Terrain", category: "Geometry", description: "A baked tiled HeightField with a hole receives mixed falling primitives.", seed: "height-v1", expectedTick: 4, expectedHash: "9b01aac6f537bca84fb23c66472b6d9e",
    build(store) { const b = standard(store); const terrain = body(b.world, ABI.enums.bodyType.static, vec(-2, 0, -2)); collider(b.world, terrain, ABI.enums.shapeKind.heightField, ZERO, "0.8", "0", 1003n); b.visuals.push(visual(terrain, "height", [6, 1, 6], 0x47735b, true)); for (let i = 0; i < 9; i += 1) addPrimitive(b, i % 3 === 0 ? "box" : "sphere", vec((i % 3) * 3 - 3, 8 + ((i / 3) | 0) * 2, (i % 2) * 3 - 2), 0xe5c55d); return runtime(b.world, b.visuals); }
  },
  {
    id: "ccd", title: "CCD Thin Wall & Moving Mesh", category: "Continuous", description: "CCD-enabled convex casters target a thin wall and a kinematic baked mesh.", seed: "ccd-v1", expectedTick: 4, expectedHash: "7de0f5cd05411dc6d0d90c3987f2965c",
    build(store) { const b = standard(store); addGround(b); const wall = body(b.world, ABI.enums.bodyType.static, vec(4, 2, 0)); collider(b.world, wall, ABI.enums.shapeKind.box, { x: decimalToRaw("0.1"), y: integerRaw(3), z: integerRaw(3) }, "0.5", "0"); b.visuals.push(visual(wall, "box", [0.2, 6, 6], 0xff5f67)); const target = body(b.world, ABI.enums.bodyType.kinematic, vec(0, 2, 3)); collider(b.world, target, ABI.enums.shapeKind.triangleMesh, ZERO, "0.5", "0", 1002n); b.visuals.push(visual(target, "mesh", [2, 2, 2], 0x8c7cf0, true)); const fast = addPrimitive(b, "sphere", vec(-8, 2, 0), 0xffd166); b.world.setBodyCcd(fast, true); const commands: CommandInput[] = [{ type: ABI.enums.commandType.velocity, phasePriority: 0, issuer: 9, sequence: 0, body: fast, first: vec(120, 0, 0), second: ZERO, transform: transform(), dofLocks: 0 }, { type: ABI.enums.commandType.kinematicTarget, phasePriority: 0, issuer: 9, sequence: 1, body: target, first: ZERO, second: ZERO, transform: transform(vec(0, 2, -3)), dofLocks: 0 }]; return runtime(b.world, b.visuals, commands); }
  },
  {
    id: "queries", title: "Ray / Shape Cast / Overlap", category: "Queries", description: "Canonical query batches are rendered from formal ABI hit results.", seed: "queries-v1", expectedTick: 4, expectedHash: "1483074bce75f17737c1d2615e06df05",
    build(store) { const b = standard(store); addGround(b); for (let i = 0; i < 5; i += 1) addPrimitive(b, i % 2 === 0 ? "sphere" : "box", vec(i * 2 - 4, 1, 0), 0x5ad7ff); const queryShape: ColliderInput = { body: 0n, shapeKind: ABI.enums.shapeKind.sphere, flags: 0, local: transform(), dimensions: { x: decimalToRaw("0.5"), y: 0n, z: 0n }, assetSourceId: 0n, friction: 0n, restitution: 0n, category: 1, mask: 0xffff_ffff, group: 0, revision: 1 }; const ray = b.world.queryRay(vec(-8, 1, 0), vec(1, 0, 0), integerRaw(16), FILTER, ABI.enums.queryMode.all); const cast = b.world.queryShapeCast(queryShape, transform(vec(-8, 1, 0)), vec(16, 0, 0), FILTER, ABI.enums.queryMode.all); const overlap = b.world.queryAabb(vec(-2, 0, -1), vec(2, 3, 1), FILTER, ABI.enums.queryMode.all); const value = runtime(b.world, b.visuals); value.queryDebug = { lines: [{ from: [-8, 1, 0], to: [8, 1, 0], color: 0x52e0ff }, { from: [-8, 1, 0], to: [8, 1, 0], color: 0xffd166 }], points: [...ray, ...cast].map(hit => ({ at: [Number(hit.point.x) / 4_294_967_296, Number(hit.point.y) / 4_294_967_296, Number(hit.point.z) / 4_294_967_296] as const, color: 0xff5f67 })), hits: [...ray, ...cast, ...overlap] }; return value; }
  },
  {
    id: "planar-2d", title: "2D Planar DOF", category: "Modes", description: "The 2D view is the same 3D pipeline with Z translation and X/Y rotation locked.", seed: "planar-v1", expectedTick: 4, expectedHash: "1f4c8353daba9b65a7c7ed2d870aeb55",
    build(store) { const b = standard(store); addGround(b); for (let i = 0; i < 12; i += 1) addPrimitive(b, i % 2 === 0 ? "box" : "sphere", vec((i % 4) * 2 - 3, ((i / 4) | 0) * 2 + 1, 0), 0x7ce38b, "0.2", 28); return runtime(b.world, b.visuals, [], [0, 3, 0]); }
  },
  {
    id: "sleep-wake", title: "Sleep / Wake", category: "State", description: "Bodies settle into deterministic sleep and a canonical impulse command wakes the island.", seed: "sleep-v1", expectedTick: 36, expectedHash: "d1a7e33d11edf57fd8826c83abe787f7",
    build(store) { const b: Build = { world: store.createWorld({ ...options(), gravity: ZERO }), visuals: [] }; addGround(b); const sleeper = addPrimitive(b, "box", vec(0, 2, 0), 0x8ee3c8); const value = runtime(b.world, b.visuals); value.startupCommands.push({ type: ABI.enums.commandType.velocity, phasePriority: 0, issuer: 12, sequence: 0, body: sleeper, first: ZERO, second: ZERO, transform: transform(), dofLocks: 0 }); return value; }
  },
  {
    id: "rollback", title: "Snapshot / Rollback", category: "State", description: "A late input rewinds a canonical snapshot, replays commands and exposes the authoritative hash.", seed: "rollback-v1", expectedTick: 8, expectedHash: "87d8d73dfdcb6b7b87ba61dcc48eafbd",
    build(store) { const b = standard(store); addGround(b); addPrimitive(b, "sphere", vec(0, 4, 0), 0xf8b55f); return runtime(b.world, b.visuals); }
  },
  {
    id: "determinism", title: "Dual-World Determinism", category: "State", description: "Two independent Worlds consume identical input and compare their hash every Tick.", seed: "dual-v1", expectedTick: 4, expectedHash: "e624782f796d619da67d1e50a0df2c5b",
    build(store) { const first = standard(store); const second = standard(store); addGround(first); addGround(second); for (let i = 0; i < 8; i += 1) { const at = vec((i % 4) * 2 - 3, 3 + (i >> 2) * 3, 0); addPrimitive(first, i % 2 === 0 ? "sphere" : "box", at, 0x60d9fa); addPrimitive(second, i % 2 === 0 ? "sphere" : "box", at, 0x60d9fa); } const value = runtime(first.world, first.visuals); value.mirror = second.world; return value; }
  },
  {
    id: "stress", title: "Stress / Worker Scaling", category: "Performance", description: "A dense WASM workload reports single-worker timing; native scaling reports can be loaded beside it.", seed: "stress-v1", expectedTick: 4, expectedHash: "c258557689232338d5f025fbc4b2d1c5",
    build(store) { const b = standard(store, [256, 320, 1024, 32]); addGround(b, 20); for (let i = 0; i < 180; i += 1) addPrimitive(b, i % 3 === 0 ? "box" : "sphere", vec((i % 12) * 2 - 11, 1 + ((i / 12) | 0) * 2, (((i / 4) % 6) | 0) * 2 - 5), 0x4fc3f7); return runtime(b.world, b.visuals, [], [0, 8, 0]); }
  }
];

export const CASES: readonly DemoCase[] = cases;

export function caseById(id: string): DemoCase {
  const found = CASES.find(value => value.id === id);
  if (found === undefined) throw new Error(`unknown case ${id}`);
  return found;
}
