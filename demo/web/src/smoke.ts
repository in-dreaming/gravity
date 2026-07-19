import { ABI } from "./wasm/abi.generated";
import { Gravity, type AssetStore, type WorldOptions } from "./wasm/gravity";

const zero = { x: 0n, y: 0n, z: 0n } as const;
const identity = { x: 0n, y: 0n, z: 0n, w: 1n << 32n } as const;
const transform = { position: zero, orientation: identity } as const;
const worldOptions: WorldOptions = {
  bodyCapacity: 4,
  colliderCapacity: 4,
  commandCapacity: 4,
  contactCapacity: 4,
  gravity: zero,
  linearDamping: 0n,
  angularDamping: 0n,
  maxLinearSpeed: 0x7fff_ffff_ffff_ffffn,
  maxAngularSpeed: 0x7fff_ffff_ffff_ffffn,
  substeps: 2,
  tickHz: 60,
  featureFlags: 0,
  jointCapacity: 0
};

function createBody(store: AssetStore): { world: ReturnType<AssetStore["createWorld"]>; body: bigint } {
  const world = store.createWorld(worldOptions);
  const body = world.createBody({
    bodyType: ABI.enums.bodyType.dynamic,
    dofLocks: 0,
    transform,
    inverseMass: 1n << 32n,
    inverseInertia: { xx: 1n << 32n, yy: 1n << 32n, zz: 1n << 32n, xy: 0n, xz: 0n, yz: 0n }
  });
  return { world, body };
}

export type SmokeReport = Readonly<{ hash: string; pages: number; bodies: number; tick: string }>;

export function runAbiSmoke(gravity: Gravity): SmokeReport {
  const store = gravity.createAssetStore([]);
  const { world, body } = createBody(store);
  const hash = world.hash();
  if (hash !== ABI.referenceHash) throw new Error(`C/Zig/WASM hash mismatch: ${hash}`);

  gravity.growMemory(1);
  if (world.hash() !== hash) throw new Error("hash changed after memory.grow");
  const snapshot = world.snapshot();
  world.loadSnapshot(snapshot);
  if (world.hash() !== hash) throw new Error("snapshot round trip changed hash");
  world.step([]);
  if (world.hash() !== ABI.replayHash) throw new Error("C/Zig/WASM replay hash mismatch");
  world.loadSnapshot(snapshot);
  world.step([{ type: ABI.enums.commandType.velocity, phasePriority: 0, issuer: 0, sequence: 0, body, first: zero, second: zero, transform, dofLocks: 0 }]);
  if (world.hash() !== ABI.replayHash) throw new Error("zero-velocity command changed replay hash");
  const bodies = world.bodyStates();
  if (bodies.length !== 1 || bodies[0]?.id !== body) throw new Error("body state batch mismatch");
  if (world.events().length !== 0) throw new Error("unexpected initial events");
  const filter = { category: 0xffff_ffff, mask: 0xffff_ffff, group: 0 } as const;
  if (world.queryPoint(zero, filter, ABI.enums.queryMode.all).length !== 0) throw new Error("unexpected point hit");
  if (world.queryRay(zero, { x: 1n << 32n, y: 0n, z: 0n }, 1n << 32n, filter, ABI.enums.queryMode.all).length !== 0) throw new Error("unexpected ray hit");
  if (world.queryAabb(zero, zero, filter, ABI.enums.queryMode.all).length !== 0) throw new Error("unexpected AABB hit");
  if (world.queryShape({ body, shapeKind: ABI.enums.shapeKind.sphere, flags: 0, local: transform, dimensions: { x: 1n << 32n, y: 0n, z: 0n }, assetSourceId: 0n, friction: 0n, restitution: 0n, category: 0xffff_ffff, mask: 0xffff_ffff, group: 0, revision: 1 }, transform, filter, ABI.enums.queryMode.all).length !== 0) throw new Error("unexpected shape hit");

  let lifetimeGuarded = false;
  try { store.dispose(); } catch { lifetimeGuarded = true; }
  if (!lifetimeGuarded) throw new Error("AssetStore disposed while a World was live");

  const warm = createBody(store).world;
  warm.dispose();
  const stablePages = gravity.memoryPages;
  for (let i = 0; i < 32; i += 1) {
    const disposable = createBody(store).world;
    disposable.dispose();
  }
  if (gravity.memoryPages !== stablePages) throw new Error("world create/dispose grew linear memory");

  const tick = world.tick();
  world.dispose();
  store.dispose();

  const warmStore = gravity.createAssetStore([]);
  warmStore.dispose();
  const storePages = gravity.memoryPages;
  for (let i = 0; i < 32; i += 1) {
    const disposableStore = gravity.createAssetStore([]);
    disposableStore.dispose();
  }
  if (gravity.memoryPages !== storePages) throw new Error("asset store create/dispose grew linear memory");
  return { hash, pages: gravity.memoryPages, bodies: bodies.length, tick: tick.toString() };
}
