import fs from "node:fs";

const bytes = fs.readFileSync(process.argv[2]);
const { instance } = await WebAssembly.instantiate(bytes, {});
const api = instance.exports;
if (api.gravity_v1_abi_version() !== 1) throw new Error("ABI version mismatch");

const page = 65536;
const oldPages = api.memory.buffer.byteLength / page;
api.memory.grow(4);
const base = oldPages * page;
let view = new DataView(api.memory.buffer);
const u32 = (at, value) => view.setUint32(at, value, true);
const i64 = (at, value) => view.setBigInt64(at, BigInt(value), true);
const readU64 = at => view.getBigUint64(at, true);
const ok = (result, where) => { if (result !== 0) throw new Error(`${where}: ${result}`); };

const assetDesc = base;
const assetSizeOut = base + 32;
const assetAlignOut = base + 40;
u32(assetDesc, 20); u32(assetDesc + 4, 0); u32(assetDesc + 8, 0); u32(assetDesc + 12, 0); u32(assetDesc + 16, 0);
ok(api.gravity_v1_asset_store_memory_required(assetDesc, assetSizeOut, assetAlignOut), "asset memory");
const assetSize = Number(readU64(assetSizeOut));
const assetMemory = base + 64;
const assetOut = assetMemory + assetSize + 16;
ok(api.gravity_v1_asset_store_init(assetMemory, BigInt(assetSize), assetDesc, assetOut), "asset init");
const assets = view.getUint32(assetOut, true);

const worldDesc = assetOut + 16;
u32(worldDesc, 96); u32(worldDesc + 4, 0);
u32(worldDesc + 8, 4); u32(worldDesc + 12, 4); u32(worldDesc + 16, 4); u32(worldDesc + 20, 4);
for (let at = 24; at < 48; at += 8) i64(worldDesc + at, 0);
i64(worldDesc + 48, 0); i64(worldDesc + 56, 0);
i64(worldDesc + 64, 0x7fffffffffffffffn); i64(worldDesc + 72, 0x7fffffffffffffffn);
u32(worldDesc + 80, 2); u32(worldDesc + 84, 60); u32(worldDesc + 88, assets);
const worldSizeOut = worldDesc + 104; const worldAlignOut = worldDesc + 112;
ok(api.gravity_v1_world_memory_required(worldDesc, worldSizeOut, worldAlignOut), "world memory");
const worldSize = Number(readU64(worldSizeOut));
let worldMemory = (worldDesc + 128 + 7) & ~7;
const needed = worldMemory + worldSize + 1024;
if (needed > api.memory.buffer.byteLength) {
  api.memory.grow(Math.ceil((needed - api.memory.buffer.byteLength) / page));
  view = new DataView(api.memory.buffer);
}
const worldOut = worldMemory + worldSize + 8;
ok(api.gravity_v1_world_init(worldMemory, BigInt(worldSize), worldDesc, worldOut), "world init");
const world = view.getUint32(worldOut, true);

const bodyDesc = worldOut + 16;
u32(bodyDesc, 128); u32(bodyDesc + 4, 0); u32(bodyDesc + 8, 1); u32(bodyDesc + 12, 0);
for (let at = 16; at < 72; at += 8) i64(bodyDesc + at, 0);
i64(bodyDesc + 64, 1n << 32n); i64(bodyDesc + 72, 1n << 32n);
i64(bodyDesc + 80, 1n << 32n); i64(bodyDesc + 88, 1n << 32n);
i64(bodyDesc + 96, 1n << 32n); i64(bodyDesc + 104, 0); i64(bodyDesc + 112, 0); i64(bodyDesc + 120, 0);
const bodyOut = bodyDesc + 136;
ok(api.gravity_v1_world_create_body(world, bodyDesc, bodyOut), "body create");
const hashA = bodyOut + 16; const hashB = hashA + 16;
ok(api.gravity_v1_world_hash(world, hashA), "hash A");
const snapshotSizeOut = hashB + 16;
ok(api.gravity_v1_world_snapshot_size(world, snapshotSizeOut), "snapshot size");
const snapshotSize = Number(readU64(snapshotSizeOut));
const snapshotMemory = snapshotSizeOut + 16; const snapshotRequired = snapshotMemory + snapshotSize + 8;
ok(api.gravity_v1_world_snapshot_save(world, snapshotMemory, BigInt(snapshotSize), snapshotRequired), "snapshot save");
ok(api.gravity_v1_world_snapshot_load(world, snapshotMemory, BigInt(snapshotSize)), "snapshot load");
ok(api.gravity_v1_world_hash(world, hashB), "hash B");
const a = Buffer.from(api.memory.buffer.slice(hashA, hashA + 16));
const b = Buffer.from(api.memory.buffer.slice(hashB, hashB + 16));
if (!a.equals(b)) throw new Error("WASM snapshot hash mismatch");
if (a.toString("hex") !== "4336297d3f06a9c557e75aea2a839853") throw new Error("WASM/Zig reference hash mismatch");
ok(api.gravity_v1_world_deinit(world), "world deinit");
ok(api.gravity_v1_asset_store_deinit(assets), "asset deinit");
console.log(a.toString("hex"));
