// Generated from docs/formats/c-abi-v1.schema.json. Do not edit.
export const ABI = {
  "abiVersion": 1,
  "pointerSize": 4,
  "referenceHash": "4336297d3f06a9c557e75aea2a839853",
  "replayHash": "3abdf5be432885c4b137c5367272516f",
  "results": {
    "ok": 0,
    "invalidArgument": 1,
    "badStruct": 2,
    "misaligned": 3,
    "insufficientMemory": 4,
    "capacity": 5,
    "invalidId": 6,
    "invalidState": 7,
    "corruptInput": 8,
    "callback": 9,
    "reentrant": 10,
    "bufferTooSmall": 11,
    "unsupported": 12,
    "internal": 13
  },
  "enums": {
    "bodyType": {
      "static": 0,
      "dynamic": 1,
      "kinematic": 2
    },
    "shapeKind": {
      "sphere": 0,
      "box": 1,
      "capsule": 2,
      "convexHull": 3,
      "compound": 4,
      "triangleMesh": 5,
      "heightField": 6
    },
    "jointKind": {
      "distance": 0,
      "ballSocket": 1,
      "hinge": 2,
      "slider": 3,
      "fixed": 4,
      "coneTwist": 5
    },
    "worldFeature": {
      "joints": 1,
      "sleep": 2,
      "ccd": 4,
      "diagnostics": 8
    },
    "jointFlag": {
      "reference": 1,
      "swingReference": 2,
      "referenceOrientation": 4,
      "limit": 8,
      "motor": 16,
      "spring": 32,
      "coneTwist": 64
    },
    "queryMode": {
      "any": 0,
      "closest": 1,
      "all": 2
    },
    "commandType": {
      "force": 0,
      "torque": 1,
      "impulseAtPoint": 2,
      "velocity": 3,
      "kinematicTarget": 4,
      "dofLocks": 5
    }
  },
  "layouts": {
    "assetBlob": {
      "size": 16,
      "data": 0,
      "length": 8
    },
    "assetStoreDesc": {
      "size": 20,
      "assets": 8,
      "assetCount": 12
    },
    "worldDesc": {
      "size": 104,
      "gravity": 24,
      "assets": 88,
      "featureFlags": 92,
      "jointCapacity": 96
    },
    "bodyDesc": {
      "size": 128,
      "transform": 16,
      "inverseMass": 72
    },
    "bodyState": {
      "size": 128,
      "id": 8,
      "transform": 24,
      "linearVelocity": 80,
      "angularVelocity": 104
    },
    "colliderDesc": {
      "size": 144,
      "body": 8,
      "local": 24,
      "dimensions": 80
    },
    "jointFrame": {
      "size": 72,
      "anchor": 0,
      "axis": 24,
      "secondary": 48
    },
    "jointDesc": {
      "size": 296,
      "bodyA": 16,
      "bodyB": 24,
      "frameA": 32,
      "frameB": 104,
      "reference": 176,
      "referenceOrientation": 192
    },
    "command": {
      "size": 144,
      "body": 24,
      "first": 32,
      "second": 56,
      "transform": 80
    },
    "event": {
      "size": 48,
      "colliderA": 16,
      "colliderB": 24
    },
    "filter": {
      "size": 16
    },
    "rayQuery": {
      "size": 88,
      "origin": 8,
      "direction": 32,
      "filter": 64
    },
    "pointQuery": {
      "size": 56,
      "point": 8,
      "filter": 32
    },
    "aabbQuery": {
      "size": 80,
      "min": 8,
      "max": 32,
      "filter": 56
    },
    "shapeQuery": {
      "size": 232,
      "shape": 8,
      "transform": 152,
      "filter": 208
    },
    "shapeCastQuery": {
      "size": 256,
      "shape": 8,
      "start": 152,
      "delta": 208,
      "filter": 232
    },
    "queryHit": {
      "size": 80,
      "collider": 8,
      "fraction": 16,
      "point": 24,
      "normal": 48
    },
    "worldStats": {
      "size": 84,
      "bodyCount": 8,
      "phaseVisits": 40
    },
    "worldFault": {
      "size": 48,
      "active": 8,
      "phase": 12,
      "code": 16,
      "detail": 20,
      "mathFault": 24,
      "hasObject": 28,
      "tick": 32,
      "object": 40
    }
  }
} as const;
export const REQUIRED_EXPORTS = [
  "gravity_v1_abi_version",
  "gravity_v1_build_info",
  "gravity_v1_result_string",
  "gravity_v1_asset_store_memory_required",
  "gravity_v1_asset_store_init",
  "gravity_v1_asset_store_deinit",
  "gravity_v1_asset_store_hash",
  "gravity_v1_world_memory_required",
  "gravity_v1_world_init",
  "gravity_v1_world_deinit",
  "gravity_v1_world_set_dispatcher",
  "gravity_v1_world_tick",
  "gravity_v1_world_last_error",
  "gravity_v1_world_last_fault",
  "gravity_v1_world_hash",
  "gravity_v1_world_step",
  "gravity_v1_world_create_body",
  "gravity_v1_world_destroy_body",
  "gravity_v1_world_body_states",
  "gravity_v1_world_create_collider",
  "gravity_v1_world_destroy_collider",
  "gravity_v1_world_create_joint",
  "gravity_v1_world_destroy_joint",
  "gravity_v1_world_set_body_ccd",
  "gravity_v1_world_stats",
  "gravity_v1_world_events",
  "gravity_v1_world_query_ray",
  "gravity_v1_world_query_point",
  "gravity_v1_world_query_aabb",
  "gravity_v1_world_query_shape",
  "gravity_v1_world_query_shape_cast",
  "gravity_v1_world_snapshot_size",
  "gravity_v1_world_snapshot_save",
  "gravity_v1_world_snapshot_load"
] as const;
