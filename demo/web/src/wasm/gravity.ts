import { ABI } from "./abi.generated";

export type FpRaw = bigint;
export type GravityId = bigint;
export type Vec3Raw = Readonly<{ x: FpRaw; y: FpRaw; z: FpRaw }>;
export type QuatRaw = Readonly<{ x: FpRaw; y: FpRaw; z: FpRaw; w: FpRaw }>;
export type TransformRaw = Readonly<{ position: Vec3Raw; orientation: QuatRaw }>;
export type QueryFilter = Readonly<{ category: number; mask: number; group: number }>;

type Block = { ptr: number; size: number };

type WasmExports = {
  memory: WebAssembly.Memory;
  gravity_v1_abi_version(): number;
  gravity_v1_asset_store_memory_required(desc: number, size: number, alignment: number): number;
  gravity_v1_asset_store_init(memory: number, memorySize: bigint, desc: number, output: number): number;
  gravity_v1_asset_store_deinit(store: number): number;
  gravity_v1_world_memory_required(desc: number, size: number, alignment: number): number;
  gravity_v1_world_init(memory: number, memorySize: bigint, desc: number, output: number): number;
  gravity_v1_world_deinit(world: number): number;
  gravity_v1_world_tick(world: number, output: number): number;
  gravity_v1_world_last_fault(world: number, output: number): number;
  gravity_v1_world_hash(world: number, output: number): number;
  gravity_v1_world_step(world: number, commands: number, count: number): number;
  gravity_v1_world_create_body(world: number, desc: number, output: number): number;
  gravity_v1_world_destroy_body(world: number, id: bigint): number;
  gravity_v1_world_body_states(world: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_create_collider(world: number, desc: number, output: number): number;
  gravity_v1_world_destroy_collider(world: number, id: bigint): number;
  gravity_v1_world_create_joint(world: number, desc: number, output: number): number;
  gravity_v1_world_destroy_joint(world: number, id: bigint): number;
  gravity_v1_world_set_body_ccd(world: number, id: bigint, enabled: number): number;
  gravity_v1_world_stats(world: number, output: number): number;
  gravity_v1_world_events(world: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_query_ray(world: number, query: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_query_point(world: number, query: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_query_aabb(world: number, query: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_query_shape(world: number, query: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_query_shape_cast(world: number, query: number, output: number, capacity: number, required: number): number;
  gravity_v1_world_snapshot_size(world: number, output: number): number;
  gravity_v1_world_snapshot_save(world: number, output: number, capacity: bigint, required: number): number;
  gravity_v1_world_snapshot_load(world: number, input: number, length: bigint): number;
};

const PAGE_SIZE = 65_536;
const MAX_SAFE_BIGINT = BigInt(Number.MAX_SAFE_INTEGER);

function alignUp(value: number, alignment: number): number {
  if (alignment <= 0 || (alignment & (alignment - 1)) !== 0) throw new Error(`invalid alignment ${alignment}`);
  return Math.ceil(value / alignment) * alignment;
}

function checkedNumber(value: bigint, label: string): number {
  if (value < 0n || value > MAX_SAFE_BIGINT) throw new Error(`${label} exceeds JavaScript's exact integer range`);
  return Number(value);
}

class LinearArena {
  private cursor: number;
  private freeBlocks: Block[] = [];
  private buffer: ArrayBuffer;
  private dataView: DataView;
  private byteView: Uint8Array;

  constructor(readonly memory: WebAssembly.Memory) {
    this.cursor = memory.buffer.byteLength;
    this.buffer = memory.buffer;
    this.dataView = new DataView(this.buffer);
    this.byteView = new Uint8Array(this.buffer);
  }

  get pages(): number {
    return this.memory.buffer.byteLength / PAGE_SIZE;
  }

  view(): DataView {
    this.refresh();
    return this.dataView;
  }

  bytes(): Uint8Array {
    this.refresh();
    return this.byteView;
  }

  alloc(requestedSize: number, alignment = 8): Block {
    if (!Number.isSafeInteger(requestedSize) || requestedSize < 0) throw new Error(`invalid allocation size ${requestedSize}`);
    const size = Math.max(requestedSize, 1);
    for (let i = 0; i < this.freeBlocks.length; i += 1) {
      const candidate = this.freeBlocks[i];
      if (candidate === undefined) continue;
      const ptr = alignUp(candidate.ptr, alignment);
      const end = ptr + size;
      if (end > candidate.ptr + candidate.size) continue;
      this.freeBlocks.splice(i, 1);
      if (ptr > candidate.ptr) this.freeBlocks.push({ ptr: candidate.ptr, size: ptr - candidate.ptr });
      if (end < candidate.ptr + candidate.size) this.freeBlocks.push({ ptr: end, size: candidate.ptr + candidate.size - end });
      this.normalizeFreeList();
      this.bytes().fill(0, ptr, end);
      return { ptr, size };
    }

    const ptr = alignUp(this.cursor, alignment);
    if (ptr > this.cursor) this.freeBlocks.push({ ptr: this.cursor, size: ptr - this.cursor });
    const end = ptr + size;
    this.ensure(end);
    this.cursor = end;
    this.bytes().fill(0, ptr, end);
    return { ptr, size };
  }

  free(block: Block): void {
    this.freeBlocks.push(block);
    this.normalizeFreeList();
  }

  private ensure(end: number): void {
    const missing = end - this.memory.buffer.byteLength;
    if (missing > 0) this.memory.grow(Math.ceil(missing / PAGE_SIZE));
    this.refresh();
  }

  private refresh(): void {
    if (this.buffer !== this.memory.buffer) {
      this.buffer = this.memory.buffer;
      this.dataView = new DataView(this.buffer);
      this.byteView = new Uint8Array(this.buffer);
    }
  }

  private normalizeFreeList(): void {
    this.freeBlocks.sort((a, b) => a.ptr - b.ptr);
    const merged: Block[] = [];
    for (const block of this.freeBlocks) {
      const previous = merged.at(-1);
      if (previous !== undefined && previous.ptr + previous.size === block.ptr) previous.size += block.size;
      else merged.push({ ...block });
    }
    this.freeBlocks = merged;
  }
}

function writeVec(view: DataView, at: number, value: Vec3Raw): void {
  view.setBigInt64(at, value.x, true);
  view.setBigInt64(at + 8, value.y, true);
  view.setBigInt64(at + 16, value.z, true);
}

function writeQuat(view: DataView, at: number, value: QuatRaw): void {
  view.setBigInt64(at, value.x, true);
  view.setBigInt64(at + 8, value.y, true);
  view.setBigInt64(at + 16, value.z, true);
  view.setBigInt64(at + 24, value.w, true);
}

function writeTransform(view: DataView, at: number, value: TransformRaw): void {
  writeVec(view, at, value.position);
  writeQuat(view, at + 24, value.orientation);
}

function writeFilter(view: DataView, at: number, value: QueryFilter): void {
  view.setUint32(at, value.category, true);
  view.setUint32(at + 4, value.mask, true);
  view.setInt32(at + 8, value.group, true);
}

function readVec(view: DataView, at: number): Vec3Raw {
  return { x: view.getBigInt64(at, true), y: view.getBigInt64(at + 8, true), z: view.getBigInt64(at + 16, true) };
}

function readQuat(view: DataView, at: number): QuatRaw {
  return { x: view.getBigInt64(at, true), y: view.getBigInt64(at + 8, true), z: view.getBigInt64(at + 16, true), w: view.getBigInt64(at + 24, true) };
}

function writeCollider(view: DataView, at: number, value: ColliderInput): void {
  view.setUint32(at, ABI.layouts.colliderDesc.size, true);
  view.setBigUint64(at + 8, value.body, true);
  view.setUint32(at + 16, value.shapeKind, true);
  view.setUint32(at + 20, value.flags, true);
  writeTransform(view, at + ABI.layouts.colliderDesc.local, value.local);
  writeVec(view, at + ABI.layouts.colliderDesc.dimensions, value.dimensions);
  view.setBigUint64(at + 104, value.assetSourceId, true);
  view.setBigInt64(at + 112, value.friction, true);
  view.setBigInt64(at + 120, value.restitution, true);
  view.setUint32(at + 128, value.category, true);
  view.setUint32(at + 132, value.mask, true);
  view.setInt32(at + 136, value.group, true);
  view.setUint32(at + 140, value.revision, true);
}

function writeJointFrame(view: DataView, at: number, value: JointFrameInput): void {
  writeVec(view, at, value.anchor);
  writeVec(view, at + 24, value.axis);
  writeVec(view, at + 48, value.secondary);
}

function writeJoint(view: DataView, at: number, value: JointInput): void {
  view.setUint32(at, ABI.layouts.jointDesc.size, true);
  view.setUint32(at + 8, value.kind, true);
  view.setUint32(at + 12, value.flags, true);
  view.setBigUint64(at + ABI.layouts.jointDesc.bodyA, value.bodyA, true);
  view.setBigUint64(at + ABI.layouts.jointDesc.bodyB, value.bodyB, true);
  writeJointFrame(view, at + ABI.layouts.jointDesc.frameA, value.frameA);
  writeJointFrame(view, at + ABI.layouts.jointDesc.frameB, value.frameB);
  view.setBigInt64(at + ABI.layouts.jointDesc.reference, value.reference, true);
  view.setBigInt64(at + 184, value.swingReference, true);
  writeQuat(view, at + ABI.layouts.jointDesc.referenceOrientation, value.referenceOrientation);
  for (const [offset, raw] of [[224, value.limitMin], [232, value.limitMax], [240, value.motorTargetVelocity], [248, value.motorMaxForce], [256, value.springFrequency], [264, value.springDampingRatio], [272, value.coneSwingMax], [280, value.coneTwistMin], [288, value.coneTwistMax]] as const) view.setBigInt64(at + offset, raw, true);
}

const resultNames = Object.fromEntries(Object.entries(ABI.results).map(([name, code]) => [code, name]));

export class GravityAbiError extends Error {
  constructor(readonly code: number, readonly operation: string) {
    super(`${operation}: ${resultNames[code] ?? `result ${code}`}`);
  }
}

export type WorldOptions = Readonly<{
  bodyCapacity: number;
  colliderCapacity: number;
  commandCapacity: number;
  contactCapacity: number;
  gravity: Vec3Raw;
  linearDamping: FpRaw;
  angularDamping: FpRaw;
  maxLinearSpeed: FpRaw;
  maxAngularSpeed: FpRaw;
  substeps: number;
  tickHz: number;
  featureFlags: number;
  jointCapacity: number;
}>;

export type BodyInput = Readonly<{
  bodyType: number;
  dofLocks: number;
  transform: TransformRaw;
  inverseMass: FpRaw;
  inverseInertia: Readonly<{ xx: FpRaw; yy: FpRaw; zz: FpRaw; xy: FpRaw; xz: FpRaw; yz: FpRaw }>;
}>;

export type ColliderInput = Readonly<{
  body: GravityId;
  shapeKind: number;
  flags: number;
  local: TransformRaw;
  dimensions: Vec3Raw;
  assetSourceId: bigint;
  friction: FpRaw;
  restitution: FpRaw;
  category: number;
  mask: number;
  group: number;
  revision: number;
}>;

export type JointFrameInput = Readonly<{ anchor: Vec3Raw; axis: Vec3Raw; secondary: Vec3Raw }>;
export type JointInput = Readonly<{
  kind: number;
  flags: number;
  bodyA: GravityId;
  bodyB: GravityId;
  frameA: JointFrameInput;
  frameB: JointFrameInput;
  reference: FpRaw;
  swingReference: FpRaw;
  referenceOrientation: QuatRaw;
  limitMin: FpRaw;
  limitMax: FpRaw;
  motorTargetVelocity: FpRaw;
  motorMaxForce: FpRaw;
  springFrequency: FpRaw;
  springDampingRatio: FpRaw;
  coneSwingMax: FpRaw;
  coneTwistMin: FpRaw;
  coneTwistMax: FpRaw;
}>;

export type CommandInput = Readonly<{
  type: number;
  phasePriority: number;
  issuer: number;
  sequence: number;
  body: GravityId;
  first: Vec3Raw;
  second: Vec3Raw;
  transform: TransformRaw;
  dofLocks: number;
}>;

export type BodyState = Readonly<{ id: GravityId; bodyType: number; dofLocks: number; transform: TransformRaw; linearVelocity: Vec3Raw; angularVelocity: Vec3Raw }>;
export type GravityEvent = Readonly<{ type: number; colliderA: GravityId; colliderB: GravityId; featureA: bigint; featureB: bigint }>;
export type QueryHit = Readonly<{ collider: GravityId; fraction: FpRaw; point: Vec3Raw; normal: Vec3Raw; primitive: number }>;
export type WorldStats = Readonly<{ bodyCount: number; colliderCount: number; jointCount: number; awakeBodyCount: number; contactCount: number; broadPairCount: number; eventCount: number; workerCount: number; phaseVisits: readonly number[] }>;
export type WorldFault = Readonly<{ active: boolean; phase: number; code: number; detail: number; mathFault: number; tick: bigint; object: bigint | null }>;

export class Gravity {
  readonly abiVersion: number;
  private readonly arena: LinearArena;

  private constructor(readonly exports: WasmExports) {
    this.arena = new LinearArena(exports.memory);
    this.abiVersion = exports.gravity_v1_abi_version();
    if (this.abiVersion !== ABI.abiVersion) throw new Error(`expected ABI ${ABI.abiVersion}, got ${this.abiVersion}`);
  }

  static async load(source: string | URL | Uint8Array): Promise<Gravity> {
    const bytes = source instanceof Uint8Array ? source : new Uint8Array(await (await fetch(source)).arrayBuffer());
    const owned = new Uint8Array(bytes.byteLength);
    owned.set(bytes);
    const module = await WebAssembly.compile(owned.buffer);
    const instance = await WebAssembly.instantiate(module, {});
    return new Gravity(instance.exports as unknown as WasmExports);
  }

  get memoryPages(): number {
    return this.arena.pages;
  }

  createAssetStore(assets: readonly Uint8Array[]): AssetStore {
    return AssetStore.create(this, assets);
  }

  growMemory(pages: number): void {
    this.exports.memory.grow(pages);
    this.arena.view();
  }

  check(result: number, operation: string, allowed: readonly number[] = []): void {
    if (result !== ABI.results.ok && !allowed.includes(result)) throw new GravityAbiError(result, operation);
  }

  allocate(size: number, alignment = 8): Block { return this.arena.alloc(size, alignment); }
  release(block: Block): void { this.arena.free(block); }
  view(): DataView { return this.arena.view(); }
  bytes(): Uint8Array { return this.arena.bytes(); }
}

export class AssetStore {
  private disposed = false;
  private activeWorlds = 0;

  private constructor(readonly gravity: Gravity, readonly pointer: number, private readonly owned: Block[]) {}

  static create(gravity: Gravity, assets: readonly Uint8Array[]): AssetStore {
    const owned: Block[] = [];
    const blobs = gravity.allocate(assets.length * ABI.layouts.assetBlob.size, 8);
    owned.push(blobs);
    for (const [index, asset] of assets.entries()) {
      const data = gravity.allocate(asset.byteLength, 1);
      owned.push(data);
      gravity.bytes().set(asset, data.ptr);
      const at = blobs.ptr + index * ABI.layouts.assetBlob.size;
      gravity.view().setUint32(at + ABI.layouts.assetBlob.data, data.ptr, true);
      gravity.view().setBigUint64(at + ABI.layouts.assetBlob.length, BigInt(asset.byteLength), true);
    }
    const desc = gravity.allocate(ABI.layouts.assetStoreDesc.size, 4);
    const sizeOut = gravity.allocate(8, 8);
    const alignOut = gravity.allocate(4, 4);
    gravity.view().setUint32(desc.ptr, ABI.layouts.assetStoreDesc.size, true);
    gravity.view().setUint32(desc.ptr + ABI.layouts.assetStoreDesc.assets, assets.length === 0 ? 0 : blobs.ptr, true);
    gravity.view().setUint32(desc.ptr + ABI.layouts.assetStoreDesc.assetCount, assets.length, true);
    try {
      gravity.check(gravity.exports.gravity_v1_asset_store_memory_required(desc.ptr, sizeOut.ptr, alignOut.ptr), "asset_store_memory_required");
      const size = checkedNumber(gravity.view().getBigUint64(sizeOut.ptr, true), "asset store size");
      const alignment = gravity.view().getUint32(alignOut.ptr, true);
      const memory = gravity.allocate(size, alignment);
      const output = gravity.allocate(4, 4);
      try {
        gravity.check(gravity.exports.gravity_v1_asset_store_init(memory.ptr, BigInt(size), desc.ptr, output.ptr), "asset_store_init");
        owned.push(memory);
        return new AssetStore(gravity, gravity.view().getUint32(output.ptr, true), owned);
      } catch (error: unknown) {
        gravity.release(memory);
        throw error;
      } finally {
        gravity.release(output);
      }
    } catch (error: unknown) {
      for (const block of owned.reverse()) gravity.release(block);
      throw error;
    } finally {
      gravity.release(alignOut);
      gravity.release(sizeOut);
      gravity.release(desc);
    }
  }

  createWorld(options: WorldOptions): World {
    if (this.disposed) throw new Error("asset store is disposed");
    return World.create(this, options);
  }

  dispose(): void {
    if (this.disposed) return;
    if (this.activeWorlds !== 0) throw new Error("dispose every World before its AssetStore");
    this.gravity.check(this.gravity.exports.gravity_v1_asset_store_deinit(this.pointer), "asset_store_deinit");
    for (const block of this.owned.reverse()) this.gravity.release(block);
    this.disposed = true;
  }

  retainWorld(): void { this.activeWorlds += 1; }
  releaseWorld(): void { this.activeWorlds -= 1; }
}

type QueryFunction = (world: number, query: number, output: number, capacity: number, required: number) => number;

export class World {
  private disposed = false;

  private constructor(readonly store: AssetStore, readonly pointer: number, private readonly memory: Block) {}

  static create(store: AssetStore, options: WorldOptions): World {
    const gravity = store.gravity;
    const desc = gravity.allocate(ABI.layouts.worldDesc.size, 8);
    const sizeOut = gravity.allocate(8, 8);
    const alignOut = gravity.allocate(4, 4);
    const view = gravity.view();
    view.setUint32(desc.ptr, ABI.layouts.worldDesc.size, true);
    view.setUint32(desc.ptr + 8, options.bodyCapacity, true);
    view.setUint32(desc.ptr + 12, options.colliderCapacity, true);
    view.setUint32(desc.ptr + 16, options.commandCapacity, true);
    view.setUint32(desc.ptr + 20, options.contactCapacity, true);
    writeVec(view, desc.ptr + ABI.layouts.worldDesc.gravity, options.gravity);
    view.setBigInt64(desc.ptr + 48, options.linearDamping, true);
    view.setBigInt64(desc.ptr + 56, options.angularDamping, true);
    view.setBigInt64(desc.ptr + 64, options.maxLinearSpeed, true);
    view.setBigInt64(desc.ptr + 72, options.maxAngularSpeed, true);
    view.setUint32(desc.ptr + 80, options.substeps, true);
    view.setUint32(desc.ptr + 84, options.tickHz, true);
    view.setUint32(desc.ptr + ABI.layouts.worldDesc.assets, store.pointer, true);
    view.setUint32(desc.ptr + ABI.layouts.worldDesc.featureFlags, options.featureFlags, true);
    view.setUint32(desc.ptr + ABI.layouts.worldDesc.jointCapacity, options.jointCapacity, true);
    try {
      gravity.check(gravity.exports.gravity_v1_world_memory_required(desc.ptr, sizeOut.ptr, alignOut.ptr), "world_memory_required");
      const size = checkedNumber(gravity.view().getBigUint64(sizeOut.ptr, true), "world size");
      const memory = gravity.allocate(size, gravity.view().getUint32(alignOut.ptr, true));
      const output = gravity.allocate(4, 4);
      try {
        gravity.check(gravity.exports.gravity_v1_world_init(memory.ptr, BigInt(size), desc.ptr, output.ptr), "world_init");
        store.retainWorld();
        return new World(store, gravity.view().getUint32(output.ptr, true), memory);
      } catch (error: unknown) {
        gravity.release(memory);
        throw error;
      } finally {
        gravity.release(output);
      }
    } finally {
      gravity.release(alignOut);
      gravity.release(sizeOut);
      gravity.release(desc);
    }
  }

  get gravity(): Gravity { return this.store.gravity; }

  createBody(input: BodyInput): GravityId {
    this.ensureLive();
    const desc = this.gravity.allocate(ABI.layouts.bodyDesc.size, 8);
    const output = this.gravity.allocate(8, 8);
    const view = this.gravity.view();
    view.setUint32(desc.ptr, ABI.layouts.bodyDesc.size, true);
    view.setUint32(desc.ptr + 8, input.bodyType, true);
    view.setUint32(desc.ptr + 12, input.dofLocks, true);
    writeTransform(view, desc.ptr + ABI.layouts.bodyDesc.transform, input.transform);
    view.setBigInt64(desc.ptr + ABI.layouts.bodyDesc.inverseMass, input.inverseMass, true);
    for (const [offset, value] of [[80, input.inverseInertia.xx], [88, input.inverseInertia.yy], [96, input.inverseInertia.zz], [104, input.inverseInertia.xy], [112, input.inverseInertia.xz], [120, input.inverseInertia.yz]] as const) view.setBigInt64(desc.ptr + offset, value, true);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_create_body(this.pointer, desc.ptr, output.ptr), "world_create_body");
      return this.gravity.view().getBigUint64(output.ptr, true);
    } finally {
      this.gravity.release(output);
      this.gravity.release(desc);
    }
  }

  destroyBody(id: GravityId): void {
    this.ensureLive();
    this.gravity.check(this.gravity.exports.gravity_v1_world_destroy_body(this.pointer, id), "world_destroy_body");
  }

  createCollider(input: ColliderInput): GravityId {
    this.ensureLive();
    const desc = this.gravity.allocate(ABI.layouts.colliderDesc.size, 8);
    const output = this.gravity.allocate(8, 8);
    writeCollider(this.gravity.view(), desc.ptr, input);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_create_collider(this.pointer, desc.ptr, output.ptr), "world_create_collider");
      return this.gravity.view().getBigUint64(output.ptr, true);
    } finally {
      this.gravity.release(output);
      this.gravity.release(desc);
    }
  }

  destroyCollider(id: GravityId): void {
    this.ensureLive();
    this.gravity.check(this.gravity.exports.gravity_v1_world_destroy_collider(this.pointer, id), "world_destroy_collider");
  }

  createJoint(input: JointInput): GravityId {
    this.ensureLive();
    const desc = this.gravity.allocate(ABI.layouts.jointDesc.size, 8);
    const output = this.gravity.allocate(8, 8);
    writeJoint(this.gravity.view(), desc.ptr, input);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_create_joint(this.pointer, desc.ptr, output.ptr), "world_create_joint");
      return this.gravity.view().getBigUint64(output.ptr, true);
    } finally {
      this.gravity.release(output);
      this.gravity.release(desc);
    }
  }

  destroyJoint(id: GravityId): void {
    this.ensureLive();
    this.gravity.check(this.gravity.exports.gravity_v1_world_destroy_joint(this.pointer, id), "world_destroy_joint");
  }

  setBodyCcd(id: GravityId, enabled: boolean): void {
    this.ensureLive();
    this.gravity.check(this.gravity.exports.gravity_v1_world_set_body_ccd(this.pointer, id, enabled ? 1 : 0), "world_set_body_ccd");
  }

  stats(): WorldStats {
    this.ensureLive();
    const output = this.gravity.allocate(ABI.layouts.worldStats.size, 4);
    const view = this.gravity.view();
    view.setUint32(output.ptr, ABI.layouts.worldStats.size, true);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_stats(this.pointer, output.ptr), "world_stats");
      return {
        bodyCount: view.getUint32(output.ptr + 8, true), colliderCount: view.getUint32(output.ptr + 12, true), jointCount: view.getUint32(output.ptr + 16, true), awakeBodyCount: view.getUint32(output.ptr + 20, true),
        contactCount: view.getUint32(output.ptr + 24, true), broadPairCount: view.getUint32(output.ptr + 28, true), eventCount: view.getUint32(output.ptr + 32, true), workerCount: view.getUint32(output.ptr + 36, true),
        phaseVisits: Array.from({ length: 11 }, (_, index) => view.getUint32(output.ptr + ABI.layouts.worldStats.phaseVisits + index * 4, true))
      };
    } finally { this.gravity.release(output); }
  }

  lastFault(): WorldFault {
    this.ensureLive();
    const output = this.gravity.allocate(ABI.layouts.worldFault.size, 8);
    const view = this.gravity.view();
    view.setUint32(output.ptr, ABI.layouts.worldFault.size, true);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_last_fault(this.pointer, output.ptr), "world_last_fault");
      const hasObject = view.getUint32(output.ptr + ABI.layouts.worldFault.hasObject, true) !== 0;
      return { active: view.getUint32(output.ptr + ABI.layouts.worldFault.active, true) !== 0, phase: view.getUint32(output.ptr + ABI.layouts.worldFault.phase, true), code: view.getUint32(output.ptr + ABI.layouts.worldFault.code, true), detail: view.getUint32(output.ptr + ABI.layouts.worldFault.detail, true), mathFault: view.getUint32(output.ptr + ABI.layouts.worldFault.mathFault, true), tick: view.getBigUint64(output.ptr + ABI.layouts.worldFault.tick, true), object: hasObject ? view.getBigUint64(output.ptr + ABI.layouts.worldFault.object, true) : null };
    } finally { this.gravity.release(output); }
  }

  step(commands: readonly CommandInput[]): void {
    this.ensureLive();
    const block = this.gravity.allocate(commands.length * ABI.layouts.command.size, 8);
    const view = this.gravity.view();
    for (const [index, command] of commands.entries()) {
      const at = block.ptr + index * ABI.layouts.command.size;
      view.setUint32(at, ABI.layouts.command.size, true);
      view.setUint32(at + 8, command.type, true);
      view.setUint32(at + 12, command.phasePriority, true);
      view.setUint32(at + 16, command.issuer, true);
      view.setUint32(at + 20, command.sequence, true);
      view.setBigUint64(at + ABI.layouts.command.body, command.body, true);
      writeVec(view, at + ABI.layouts.command.first, command.first);
      writeVec(view, at + ABI.layouts.command.second, command.second);
      writeTransform(view, at + ABI.layouts.command.transform, command.transform);
      view.setUint32(at + 136, command.dofLocks, true);
    }
    try {
      const result = this.gravity.exports.gravity_v1_world_step(this.pointer, commands.length === 0 ? 0 : block.ptr, commands.length);
      if (result !== ABI.results.ok) {
        const fault = this.lastFault();
        if (fault.active) throw new Error(`world_step: result ${result}, phase ${fault.phase}, code ${fault.code}, detail ${fault.detail}, math ${fault.mathFault}, tick ${fault.tick}`);
        this.gravity.check(result, "world_step");
      }
    } finally {
      this.gravity.release(block);
    }
  }

  tick(): bigint {
    this.ensureLive();
    const output = this.gravity.allocate(8, 8);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_tick(this.pointer, output.ptr), "world_tick");
      return this.gravity.view().getBigUint64(output.ptr, true);
    } finally { this.gravity.release(output); }
  }

  hash(): string {
    this.ensureLive();
    const output = this.gravity.allocate(16, 1);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_hash(this.pointer, output.ptr), "world_hash");
      return Array.from(this.gravity.bytes().subarray(output.ptr, output.ptr + 16), value => value.toString(16).padStart(2, "0")).join("");
    } finally { this.gravity.release(output); }
  }

  snapshot(): Uint8Array {
    this.ensureLive();
    const sizeOut = this.gravity.allocate(8, 8);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_snapshot_size(this.pointer, sizeOut.ptr), "world_snapshot_size");
      const size = checkedNumber(this.gravity.view().getBigUint64(sizeOut.ptr, true), "snapshot size");
      const data = this.gravity.allocate(size, 1);
      const required = this.gravity.allocate(8, 8);
      try {
        this.gravity.check(this.gravity.exports.gravity_v1_world_snapshot_save(this.pointer, data.ptr, BigInt(size), required.ptr), "world_snapshot_save");
        return this.gravity.bytes().slice(data.ptr, data.ptr + size);
      } finally {
        this.gravity.release(required);
        this.gravity.release(data);
      }
    } finally { this.gravity.release(sizeOut); }
  }

  loadSnapshot(snapshot: Uint8Array): void {
    this.ensureLive();
    const input = this.gravity.allocate(snapshot.byteLength, 1);
    this.gravity.bytes().set(snapshot, input.ptr);
    try {
      this.gravity.check(this.gravity.exports.gravity_v1_world_snapshot_load(this.pointer, input.ptr, BigInt(snapshot.byteLength)), "world_snapshot_load");
    } finally { this.gravity.release(input); }
  }

  bodyStates(): BodyState[] {
    this.ensureLive();
    return this.readBatch(ABI.layouts.bodyState.size, this.gravity.exports.gravity_v1_world_body_states.bind(this.gravity.exports), (view, at) => ({
      id: view.getBigUint64(at + ABI.layouts.bodyState.id, true),
      bodyType: view.getUint32(at + 16, true),
      dofLocks: view.getUint32(at + 20, true),
      transform: { position: readVec(view, at + ABI.layouts.bodyState.transform), orientation: readQuat(view, at + ABI.layouts.bodyState.transform + 24) },
      linearVelocity: readVec(view, at + ABI.layouts.bodyState.linearVelocity),
      angularVelocity: readVec(view, at + ABI.layouts.bodyState.angularVelocity)
    }), "world_body_states");
  }

  events(): GravityEvent[] {
    this.ensureLive();
    return this.readBatch(ABI.layouts.event.size, this.gravity.exports.gravity_v1_world_events.bind(this.gravity.exports), (view, at) => ({
      type: view.getUint32(at + 8, true), colliderA: view.getBigUint64(at + 16, true), colliderB: view.getBigUint64(at + 24, true), featureA: view.getBigUint64(at + 32, true), featureB: view.getBigUint64(at + 40, true)
    }), "world_events");
  }

  queryPoint(point: Vec3Raw, filter: QueryFilter, mode: number): QueryHit[] {
    this.ensureLive();
    return this.query(ABI.layouts.pointQuery.size, (view, at) => { writeVec(view, at + ABI.layouts.pointQuery.point, point); writeFilter(view, at + ABI.layouts.pointQuery.filter, filter); view.setUint32(at + 48, mode, true); }, this.gravity.exports.gravity_v1_world_query_point.bind(this.gravity.exports), "query_point");
  }

  queryRay(origin: Vec3Raw, direction: Vec3Raw, maxFraction: FpRaw, filter: QueryFilter, mode: number): QueryHit[] {
    this.ensureLive();
    return this.query(ABI.layouts.rayQuery.size, (view, at) => { writeVec(view, at + ABI.layouts.rayQuery.origin, origin); writeVec(view, at + ABI.layouts.rayQuery.direction, direction); view.setBigInt64(at + 56, maxFraction, true); writeFilter(view, at + ABI.layouts.rayQuery.filter, filter); view.setUint32(at + 80, mode, true); }, this.gravity.exports.gravity_v1_world_query_ray.bind(this.gravity.exports), "query_ray");
  }

  queryAabb(min: Vec3Raw, max: Vec3Raw, filter: QueryFilter, mode: number): QueryHit[] {
    this.ensureLive();
    return this.query(ABI.layouts.aabbQuery.size, (view, at) => { writeVec(view, at + ABI.layouts.aabbQuery.min, min); writeVec(view, at + ABI.layouts.aabbQuery.max, max); writeFilter(view, at + ABI.layouts.aabbQuery.filter, filter); view.setUint32(at + 72, mode, true); }, this.gravity.exports.gravity_v1_world_query_aabb.bind(this.gravity.exports), "query_aabb");
  }

  queryShape(shape: ColliderInput, transform: TransformRaw, filter: QueryFilter, mode: number): QueryHit[] {
    this.ensureLive();
    return this.query(ABI.layouts.shapeQuery.size, (view, at) => { writeCollider(view, at + ABI.layouts.shapeQuery.shape, shape); writeTransform(view, at + ABI.layouts.shapeQuery.transform, transform); writeFilter(view, at + ABI.layouts.shapeQuery.filter, filter); view.setUint32(at + 224, mode, true); }, this.gravity.exports.gravity_v1_world_query_shape.bind(this.gravity.exports), "query_shape");
  }

  queryShapeCast(shape: ColliderInput, start: TransformRaw, delta: Vec3Raw, filter: QueryFilter, mode: number): QueryHit[] {
    this.ensureLive();
    return this.query(ABI.layouts.shapeCastQuery.size, (view, at) => { writeCollider(view, at + ABI.layouts.shapeCastQuery.shape, shape); writeTransform(view, at + ABI.layouts.shapeCastQuery.start, start); writeVec(view, at + ABI.layouts.shapeCastQuery.delta, delta); writeFilter(view, at + ABI.layouts.shapeCastQuery.filter, filter); view.setUint32(at + 248, mode, true); }, this.gravity.exports.gravity_v1_world_query_shape_cast.bind(this.gravity.exports), "query_shape_cast");
  }

  dispose(): void {
    if (this.disposed) return;
    this.gravity.check(this.gravity.exports.gravity_v1_world_deinit(this.pointer), "world_deinit");
    this.gravity.release(this.memory);
    this.store.releaseWorld();
    this.disposed = true;
  }

  private ensureLive(): void {
    if (this.disposed) throw new Error("world is disposed");
  }

  private readBatch<T>(stride: number, call: (world: number, output: number, capacity: number, required: number) => number, decode: (view: DataView, at: number) => T, operation: string): T[] {
    const required = this.gravity.allocate(4, 4);
    try {
      this.gravity.check(call(this.pointer, 0, 0, required.ptr), operation, [ABI.results.bufferTooSmall]);
      const count = this.gravity.view().getUint32(required.ptr, true);
      if (count === 0) return [];
      const output = this.gravity.allocate(count * stride, 8);
      try {
        this.gravity.check(call(this.pointer, output.ptr, count, required.ptr), operation);
        const view = this.gravity.view();
        return Array.from({ length: count }, (_, index) => decode(view, output.ptr + index * stride));
      } finally { this.gravity.release(output); }
    } finally { this.gravity.release(required); }
  }

  private query(size: number, encode: (view: DataView, at: number) => void, call: QueryFunction, operation: string): QueryHit[] {
    const query = this.gravity.allocate(size, 8);
    this.gravity.view().setUint32(query.ptr, size, true);
    encode(this.gravity.view(), query.ptr);
    const required = this.gravity.allocate(4, 4);
    try {
      this.gravity.check(call(this.pointer, query.ptr, 0, 0, required.ptr), operation, [ABI.results.bufferTooSmall]);
      const count = this.gravity.view().getUint32(required.ptr, true);
      if (count === 0) return [];
      const output = this.gravity.allocate(count * ABI.layouts.queryHit.size, 8);
      try {
        this.gravity.check(call(this.pointer, query.ptr, output.ptr, count, required.ptr), operation);
        const view = this.gravity.view();
        return Array.from({ length: count }, (_, index) => {
          const at = output.ptr + index * ABI.layouts.queryHit.size;
          return { collider: view.getBigUint64(at + ABI.layouts.queryHit.collider, true), fraction: view.getBigInt64(at + ABI.layouts.queryHit.fraction, true), point: readVec(view, at + ABI.layouts.queryHit.point), normal: readVec(view, at + ABI.layouts.queryHit.normal), primitive: view.getUint32(at + 72, true) };
        });
      } finally { this.gravity.release(output); }
    } finally {
      this.gravity.release(required);
      this.gravity.release(query);
    }
  }
}
