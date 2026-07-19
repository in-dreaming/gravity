import { readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import process from "node:process";

const repo = resolve(import.meta.dirname, "../../..");
const schemaPath = resolve(repo, "docs/formats/c-abi-v1.schema.json");
const outputPath = resolve(repo, "demo/web/src/wasm/abi.generated.ts");
const headerPath = resolve(repo, "include/gravity.h");
const baselinePath = resolve(repo, "tests/abi/abi-baseline-v1.json");
const schema = JSON.parse(readFileSync(schemaPath, "utf8"));
const baseline = JSON.parse(readFileSync(baselinePath, "utf8"));

if (schema.abiVersion !== baseline.abi_version || schema.referenceHash !== baseline.reference_hash || schema.replayHash !== baseline.replay_hash) {
  throw new Error("ABI schema/baseline version or reference hash drift");
}

const macroNames = {
  assetBlob: "GRAVITY_V1_WASM_SIZE_ASSET_BLOB",
  assetStoreDesc: "GRAVITY_V1_WASM_SIZE_ASSET_STORE_DESC",
  worldDesc: "GRAVITY_V1_WASM_SIZE_WORLD_DESC",
  bodyDesc: "GRAVITY_V1_SIZE_BODY_DESC",
  bodyState: "GRAVITY_V1_SIZE_BODY_STATE",
  colliderDesc: "GRAVITY_V1_SIZE_COLLIDER_DESC",
  jointFrame: "GRAVITY_V1_SIZE_JOINT_FRAME",
  jointDesc: "GRAVITY_V1_SIZE_JOINT_DESC",
  command: "GRAVITY_V1_SIZE_COMMAND",
  event: "GRAVITY_V1_SIZE_EVENT",
  filter: "GRAVITY_V1_SIZE_QUERY_FILTER",
  rayQuery: "GRAVITY_V1_SIZE_RAY_QUERY",
  pointQuery: "GRAVITY_V1_SIZE_POINT_QUERY",
  aabbQuery: "GRAVITY_V1_SIZE_AABB_QUERY",
  shapeQuery: "GRAVITY_V1_SIZE_SHAPE_QUERY",
  shapeCastQuery: "GRAVITY_V1_SIZE_SHAPE_CAST_QUERY",
  queryHit: "GRAVITY_V1_SIZE_QUERY_HIT",
  worldStats: "GRAVITY_V1_SIZE_WORLD_STATS",
  worldFault: "GRAVITY_V1_SIZE_WORLD_FAULT"
};
const header = readFileSync(headerPath, "utf8");
for (const [layout, macro] of Object.entries(macroNames)) {
  const size = schema.layouts[layout].size;
  if (!header.includes(`#define ${macro} ${size}u`)) {
    throw new Error(`gravity.h is missing ${macro}=${size}`);
  }
}

const generated = `// Generated from docs/formats/c-abi-v1.schema.json. Do not edit.\n` +
  `export const ABI = ${JSON.stringify(schema, null, 2)} as const;\n` +
  `export const REQUIRED_EXPORTS = ${JSON.stringify(baseline.exports, null, 2)} as const;\n`;

if (process.argv[2] === "--write") {
  writeFileSync(outputPath, generated);
} else if (process.argv[2] === "--check") {
  if (readFileSync(outputPath, "utf8").replaceAll("\r\n", "\n") !== generated) {
    throw new Error("abi.generated.ts is stale; run pnpm abi:generate");
  }
} else {
  throw new Error("expected --write or --check");
}
