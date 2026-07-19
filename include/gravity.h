#ifndef GRAVITY_H
#define GRAVITY_H

#include <stdint.h>

#if defined(_WIN32) && defined(GRAVITY_SHARED)
#  if defined(GRAVITY_BUILD)
#    define GRAVITY_API __declspec(dllexport)
#  else
#    define GRAVITY_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define GRAVITY_API __attribute__((visibility("default")))
#else
#  define GRAVITY_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define GRAVITY_V1_ABI_VERSION 1u
#define GRAVITY_V1_PROTOCOL_VERSION 1u
#define GRAVITY_V1_SNAPSHOT_FORMAT_VERSION 1u
#define GRAVITY_V1_ASSET_FORMAT_VERSION 2u

/* Frozen layout constants consumed by the generated wasm32 TypeScript ABI. */
#define GRAVITY_V1_WASM_SIZE_ASSET_BLOB 16u
#define GRAVITY_V1_WASM_SIZE_ASSET_STORE_DESC 20u
#define GRAVITY_V1_WASM_SIZE_WORLD_DESC 104u
#define GRAVITY_V1_SIZE_BODY_DESC 128u
#define GRAVITY_V1_SIZE_BODY_STATE 128u
#define GRAVITY_V1_SIZE_COLLIDER_DESC 144u
#define GRAVITY_V1_SIZE_JOINT_FRAME 72u
#define GRAVITY_V1_SIZE_JOINT_DESC 296u
#define GRAVITY_V1_SIZE_COMMAND 144u
#define GRAVITY_V1_SIZE_EVENT 48u
#define GRAVITY_V1_SIZE_QUERY_FILTER 16u
#define GRAVITY_V1_SIZE_RAY_QUERY 88u
#define GRAVITY_V1_SIZE_POINT_QUERY 56u
#define GRAVITY_V1_SIZE_AABB_QUERY 80u
#define GRAVITY_V1_SIZE_SHAPE_QUERY 232u
#define GRAVITY_V1_SIZE_SHAPE_CAST_QUERY 256u
#define GRAVITY_V1_SIZE_QUERY_HIT 80u
#define GRAVITY_V1_SIZE_WORLD_STATS 84u

typedef int64_t GravityFpRaw;
typedef uint64_t GravityId;
typedef uint32_t GravityResult;

enum {
    GRAVITY_OK = 0u,
    GRAVITY_ERROR_INVALID_ARGUMENT = 1u,
    GRAVITY_ERROR_BAD_STRUCT = 2u,
    GRAVITY_ERROR_MISALIGNED = 3u,
    GRAVITY_ERROR_INSUFFICIENT_MEMORY = 4u,
    GRAVITY_ERROR_CAPACITY = 5u,
    GRAVITY_ERROR_INVALID_ID = 6u,
    GRAVITY_ERROR_INVALID_STATE = 7u,
    GRAVITY_ERROR_CORRUPT_INPUT = 8u,
    GRAVITY_ERROR_CALLBACK = 9u,
    GRAVITY_ERROR_REENTRANT = 10u,
    GRAVITY_ERROR_BUFFER_TOO_SMALL = 11u,
    GRAVITY_ERROR_UNSUPPORTED = 12u,
    GRAVITY_ERROR_INTERNAL = 13u
};

enum { GRAVITY_BODY_STATIC = 0u, GRAVITY_BODY_DYNAMIC = 1u, GRAVITY_BODY_KINEMATIC = 2u };
enum { GRAVITY_SHAPE_SPHERE = 0u, GRAVITY_SHAPE_BOX = 1u, GRAVITY_SHAPE_CAPSULE = 2u,
       GRAVITY_SHAPE_CONVEX_HULL = 3u, GRAVITY_SHAPE_COMPOUND = 4u,
       GRAVITY_SHAPE_TRIANGLE_MESH = 5u, GRAVITY_SHAPE_HEIGHT_FIELD = 6u };
enum { GRAVITY_QUERY_ANY = 0u, GRAVITY_QUERY_CLOSEST = 1u, GRAVITY_QUERY_ALL = 2u };
enum { GRAVITY_JOINT_DISTANCE = 0u, GRAVITY_JOINT_BALL_SOCKET = 1u,
       GRAVITY_JOINT_HINGE = 2u, GRAVITY_JOINT_SLIDER = 3u,
       GRAVITY_JOINT_FIXED = 4u, GRAVITY_JOINT_CONE_TWIST = 5u };
enum { GRAVITY_WORLD_FEATURE_JOINTS = 1u << 0,
       GRAVITY_WORLD_FEATURE_SLEEP = 1u << 1,
       GRAVITY_WORLD_FEATURE_CCD = 1u << 2,
       GRAVITY_WORLD_FEATURE_DIAGNOSTICS = 1u << 3 };
enum { GRAVITY_JOINT_HAS_REFERENCE = 1u << 0,
       GRAVITY_JOINT_HAS_SWING_REFERENCE = 1u << 1,
       GRAVITY_JOINT_HAS_REFERENCE_ORIENTATION = 1u << 2,
       GRAVITY_JOINT_ENABLE_LIMIT = 1u << 3,
       GRAVITY_JOINT_ENABLE_MOTOR = 1u << 4,
       GRAVITY_JOINT_ENABLE_SPRING = 1u << 5,
       GRAVITY_JOINT_ENABLE_CONE_TWIST = 1u << 6 };
enum { GRAVITY_COMMAND_FORCE = 0u, GRAVITY_COMMAND_TORQUE = 1u,
       GRAVITY_COMMAND_IMPULSE_AT_POINT = 2u, GRAVITY_COMMAND_VELOCITY = 3u,
       GRAVITY_COMMAND_KINEMATIC_TARGET = 4u, GRAVITY_COMMAND_DOF_LOCKS = 5u };

typedef struct GravityAssetStore GravityAssetStore;
typedef struct GravityWorld GravityWorld;

typedef struct GravityVec3 { GravityFpRaw x, y, z; } GravityVec3;
typedef struct GravityQuat { GravityFpRaw x, y, z, w; } GravityQuat;
typedef struct GravityTransform { GravityVec3 position; GravityQuat orientation; } GravityTransform;
typedef struct GravityHash128 { uint8_t bytes[16]; } GravityHash128;
typedef struct GravityHash256 { uint8_t bytes[32]; } GravityHash256;

typedef struct GravityBuildInfo {
    uint32_t struct_size, reserved;
    uint32_t abi_version, protocol_version, snapshot_format_version, asset_format_version;
    const uint8_t *commit; uint32_t commit_length;
    const uint8_t *zig_version; uint32_t zig_version_length;
} GravityBuildInfo;

typedef struct GravityAssetBlob { const uint8_t *data; uint64_t length; } GravityAssetBlob;
typedef struct GravityAssetStoreDesc {
    uint32_t struct_size, reserved;
    const GravityAssetBlob *assets; uint32_t asset_count, reserved1;
} GravityAssetStoreDesc;

typedef struct GravityWorldDesc {
    uint32_t struct_size, reserved;
    uint32_t body_capacity, collider_capacity, command_capacity, contact_capacity;
    GravityVec3 gravity;
    GravityFpRaw linear_damping, angular_damping, max_linear_speed, max_angular_speed;
    uint32_t substeps, tick_hz;
    const GravityAssetStore *assets;
    /* Appended v1 capability tail. A 96-byte struct retains legacy behavior. */
    uint32_t feature_flags, joint_capacity;
} GravityWorldDesc;

typedef struct GravityBodyDesc {
    uint32_t struct_size, reserved;
    uint32_t body_type, dof_locks;
    GravityTransform transform;
    GravityFpRaw inverse_mass;
    GravityFpRaw inverse_inertia_xx, inverse_inertia_yy, inverse_inertia_zz;
    GravityFpRaw inverse_inertia_xy, inverse_inertia_xz, inverse_inertia_yz;
} GravityBodyDesc;

typedef struct GravityBodyState {
    uint32_t struct_size, reserved;
    GravityId id; uint32_t body_type, dof_locks;
    GravityTransform transform;
    GravityVec3 linear_velocity, angular_velocity;
} GravityBodyState;

typedef struct GravityColliderDesc {
    uint32_t struct_size, reserved;
    GravityId body; uint32_t shape_kind, flags;
    GravityTransform local;
    GravityVec3 dimensions;
    uint64_t asset_source_id;
    GravityFpRaw friction, restitution;
    uint32_t category, mask; int32_t group; uint32_t revision;
} GravityColliderDesc;

typedef struct GravityJointFrame { GravityVec3 anchor, axis, secondary; } GravityJointFrame;
typedef struct GravityJointDesc {
    uint32_t struct_size, reserved;
    uint32_t kind, flags;
    GravityId body_a, body_b;
    GravityJointFrame frame_a, frame_b;
    GravityFpRaw reference, swing_reference;
    GravityQuat reference_orientation;
    GravityFpRaw limit_min, limit_max;
    GravityFpRaw motor_target_velocity, motor_max_force;
    GravityFpRaw spring_frequency, spring_damping_ratio;
    GravityFpRaw cone_swing_max, cone_twist_min, cone_twist_max;
} GravityJointDesc;

typedef struct GravityCommand {
    uint32_t struct_size, reserved;
    uint32_t type, phase_priority, issuer, sequence;
    GravityId body;
    GravityVec3 first, second;
    GravityTransform transform;
    uint32_t dof_locks, reserved1;
} GravityCommand;

typedef struct GravityEvent {
    uint32_t struct_size, reserved;
    uint32_t type, reserved1;
    GravityId collider_a, collider_b;
    uint64_t feature_a, feature_b;
} GravityEvent;

typedef struct GravityQueryFilter { uint32_t category, mask; int32_t group; uint32_t reserved; } GravityQueryFilter;
typedef struct GravityRayQuery {
    uint32_t struct_size, reserved;
    GravityVec3 origin, direction; GravityFpRaw max_fraction;
    GravityQueryFilter filter; uint32_t mode, reserved1;
} GravityRayQuery;
typedef struct GravityPointQuery {
    uint32_t struct_size, reserved; GravityVec3 point;
    GravityQueryFilter filter; uint32_t mode, reserved1;
} GravityPointQuery;
typedef struct GravityAabbQuery {
    uint32_t struct_size, reserved; GravityVec3 min, max;
    GravityQueryFilter filter; uint32_t mode, reserved1;
} GravityAabbQuery;
typedef struct GravityShapeQuery {
    uint32_t struct_size, reserved; GravityColliderDesc shape; GravityTransform transform;
    GravityQueryFilter filter; uint32_t mode, reserved1;
} GravityShapeQuery;
typedef struct GravityShapeCastQuery {
    uint32_t struct_size, reserved; GravityColliderDesc shape; GravityTransform start;
    GravityVec3 delta; GravityQueryFilter filter; uint32_t mode, reserved1;
} GravityShapeCastQuery;
typedef struct GravityQueryHit {
    uint32_t struct_size, reserved; GravityId collider; GravityFpRaw fraction;
    GravityVec3 point, normal; uint32_t primitive, reserved1;
} GravityQueryHit;
typedef struct GravityWorldStats {
    uint32_t struct_size, reserved;
    uint32_t body_count, collider_count, joint_count, awake_body_count;
    uint32_t contact_count, broad_pair_count, event_count, worker_count;
    uint32_t phase_visits[11];
} GravityWorldStats;

typedef GravityResult (*GravityRunJobFn)(void *batch_context, uint32_t job_index);
typedef GravityResult (*GravityDispatchBatchFn)(void *user, uint32_t job_count,
                                                GravityRunJobFn run_job, void *batch_context);
typedef struct GravityDispatcher {
    uint32_t struct_size, reserved; void *user; GravityDispatchBatchFn dispatch_batch;
} GravityDispatcher;

GRAVITY_API uint32_t gravity_v1_abi_version(void);
GRAVITY_API GravityResult gravity_v1_build_info(GravityBuildInfo *out_info);
GRAVITY_API const char *gravity_v1_result_string(GravityResult result);

GRAVITY_API GravityResult gravity_v1_asset_store_memory_required(const GravityAssetStoreDesc *desc, uint64_t *out_size, uint32_t *out_alignment);
GRAVITY_API GravityResult gravity_v1_asset_store_init(void *memory, uint64_t memory_size, const GravityAssetStoreDesc *desc, GravityAssetStore **out_store);
GRAVITY_API GravityResult gravity_v1_asset_store_deinit(GravityAssetStore *store);
GRAVITY_API GravityResult gravity_v1_asset_store_hash(const GravityAssetStore *store, GravityHash256 *out_hash);

GRAVITY_API GravityResult gravity_v1_world_memory_required(const GravityWorldDesc *desc, uint64_t *out_size, uint32_t *out_alignment);
GRAVITY_API GravityResult gravity_v1_world_init(void *memory, uint64_t memory_size, const GravityWorldDesc *desc, GravityWorld **out_world);
GRAVITY_API GravityResult gravity_v1_world_deinit(GravityWorld *world);
GRAVITY_API GravityResult gravity_v1_world_set_dispatcher(GravityWorld *world, const GravityDispatcher *dispatcher);
GRAVITY_API GravityResult gravity_v1_world_tick(const GravityWorld *world, uint64_t *out_tick);
GRAVITY_API GravityResult gravity_v1_world_last_error(const GravityWorld *world, GravityResult *out_error);
GRAVITY_API GravityResult gravity_v1_world_hash(const GravityWorld *world, GravityHash128 *out_hash);
GRAVITY_API GravityResult gravity_v1_world_step(GravityWorld *world, const GravityCommand *commands, uint32_t command_count);

GRAVITY_API GravityResult gravity_v1_world_create_body(GravityWorld *world, const GravityBodyDesc *desc, GravityId *out_id);
GRAVITY_API GravityResult gravity_v1_world_destroy_body(GravityWorld *world, GravityId id);
GRAVITY_API GravityResult gravity_v1_world_body_states(const GravityWorld *world, GravityBodyState *output, uint32_t capacity, uint32_t *out_required);
GRAVITY_API GravityResult gravity_v1_world_create_collider(GravityWorld *world, const GravityColliderDesc *desc, GravityId *out_id);
GRAVITY_API GravityResult gravity_v1_world_destroy_collider(GravityWorld *world, GravityId id);
GRAVITY_API GravityResult gravity_v1_world_create_joint(GravityWorld *world, const GravityJointDesc *desc, GravityId *out_id);
GRAVITY_API GravityResult gravity_v1_world_destroy_joint(GravityWorld *world, GravityId id);
GRAVITY_API GravityResult gravity_v1_world_set_body_ccd(GravityWorld *world, GravityId id, uint32_t enabled);
GRAVITY_API GravityResult gravity_v1_world_stats(const GravityWorld *world, GravityWorldStats *out_stats);
GRAVITY_API GravityResult gravity_v1_world_events(const GravityWorld *world, GravityEvent *output, uint32_t capacity, uint32_t *out_required);

GRAVITY_API GravityResult gravity_v1_world_query_ray(GravityWorld *world, const GravityRayQuery *query, GravityQueryHit *output, uint32_t capacity, uint32_t *out_required);
GRAVITY_API GravityResult gravity_v1_world_query_point(GravityWorld *world, const GravityPointQuery *query, GravityQueryHit *output, uint32_t capacity, uint32_t *out_required);
GRAVITY_API GravityResult gravity_v1_world_query_aabb(GravityWorld *world, const GravityAabbQuery *query, GravityQueryHit *output, uint32_t capacity, uint32_t *out_required);
GRAVITY_API GravityResult gravity_v1_world_query_shape(GravityWorld *world, const GravityShapeQuery *query, GravityQueryHit *output, uint32_t capacity, uint32_t *out_required);
GRAVITY_API GravityResult gravity_v1_world_query_shape_cast(GravityWorld *world, const GravityShapeCastQuery *query, GravityQueryHit *output, uint32_t capacity, uint32_t *out_required);

GRAVITY_API GravityResult gravity_v1_world_snapshot_size(GravityWorld *world, uint64_t *out_size);
GRAVITY_API GravityResult gravity_v1_world_snapshot_save(GravityWorld *world, uint8_t *output, uint64_t capacity, uint64_t *out_required);
GRAVITY_API GravityResult gravity_v1_world_snapshot_load(GravityWorld *world, const uint8_t *input, uint64_t length);

#ifdef __cplusplus
}
#endif

#if defined(__cplusplus)
static_assert(sizeof(GravityVec3) == 24, "GravityVec3 layout");
static_assert(sizeof(GravityQuat) == 32, "GravityQuat layout");
static_assert(sizeof(GravityHash128) == 16, "GravityHash128 layout");
static_assert(sizeof(GravityBodyDesc) == GRAVITY_V1_SIZE_BODY_DESC, "GravityBodyDesc layout");
static_assert(sizeof(GravityBodyState) == GRAVITY_V1_SIZE_BODY_STATE, "GravityBodyState layout");
static_assert(sizeof(GravityColliderDesc) == GRAVITY_V1_SIZE_COLLIDER_DESC, "GravityColliderDesc layout");
static_assert(sizeof(GravityJointFrame) == GRAVITY_V1_SIZE_JOINT_FRAME, "GravityJointFrame layout");
static_assert(sizeof(GravityJointDesc) == GRAVITY_V1_SIZE_JOINT_DESC, "GravityJointDesc layout");
static_assert(sizeof(GravityCommand) == GRAVITY_V1_SIZE_COMMAND, "GravityCommand layout");
static_assert(sizeof(GravityEvent) == GRAVITY_V1_SIZE_EVENT, "GravityEvent layout");
static_assert(sizeof(GravityQueryFilter) == GRAVITY_V1_SIZE_QUERY_FILTER, "GravityQueryFilter layout");
static_assert(sizeof(GravityRayQuery) == GRAVITY_V1_SIZE_RAY_QUERY, "GravityRayQuery layout");
static_assert(sizeof(GravityPointQuery) == GRAVITY_V1_SIZE_POINT_QUERY, "GravityPointQuery layout");
static_assert(sizeof(GravityAabbQuery) == GRAVITY_V1_SIZE_AABB_QUERY, "GravityAabbQuery layout");
static_assert(sizeof(GravityShapeQuery) == GRAVITY_V1_SIZE_SHAPE_QUERY, "GravityShapeQuery layout");
static_assert(sizeof(GravityShapeCastQuery) == GRAVITY_V1_SIZE_SHAPE_CAST_QUERY, "GravityShapeCastQuery layout");
static_assert(sizeof(GravityQueryHit) == GRAVITY_V1_SIZE_QUERY_HIT, "GravityQueryHit layout");
static_assert(sizeof(GravityWorldStats) == GRAVITY_V1_SIZE_WORLD_STATS, "GravityWorldStats layout");
#else
_Static_assert(sizeof(GravityVec3) == 24, "GravityVec3 layout");
_Static_assert(sizeof(GravityQuat) == 32, "GravityQuat layout");
_Static_assert(sizeof(GravityHash128) == 16, "GravityHash128 layout");
_Static_assert(sizeof(GravityBodyDesc) == GRAVITY_V1_SIZE_BODY_DESC, "GravityBodyDesc layout");
_Static_assert(sizeof(GravityBodyState) == GRAVITY_V1_SIZE_BODY_STATE, "GravityBodyState layout");
_Static_assert(sizeof(GravityColliderDesc) == GRAVITY_V1_SIZE_COLLIDER_DESC, "GravityColliderDesc layout");
_Static_assert(sizeof(GravityJointFrame) == GRAVITY_V1_SIZE_JOINT_FRAME, "GravityJointFrame layout");
_Static_assert(sizeof(GravityJointDesc) == GRAVITY_V1_SIZE_JOINT_DESC, "GravityJointDesc layout");
_Static_assert(sizeof(GravityCommand) == GRAVITY_V1_SIZE_COMMAND, "GravityCommand layout");
_Static_assert(sizeof(GravityEvent) == GRAVITY_V1_SIZE_EVENT, "GravityEvent layout");
_Static_assert(sizeof(GravityQueryFilter) == GRAVITY_V1_SIZE_QUERY_FILTER, "GravityQueryFilter layout");
_Static_assert(sizeof(GravityRayQuery) == GRAVITY_V1_SIZE_RAY_QUERY, "GravityRayQuery layout");
_Static_assert(sizeof(GravityPointQuery) == GRAVITY_V1_SIZE_POINT_QUERY, "GravityPointQuery layout");
_Static_assert(sizeof(GravityAabbQuery) == GRAVITY_V1_SIZE_AABB_QUERY, "GravityAabbQuery layout");
_Static_assert(sizeof(GravityShapeQuery) == GRAVITY_V1_SIZE_SHAPE_QUERY, "GravityShapeQuery layout");
_Static_assert(sizeof(GravityShapeCastQuery) == GRAVITY_V1_SIZE_SHAPE_CAST_QUERY, "GravityShapeCastQuery layout");
_Static_assert(sizeof(GravityQueryHit) == GRAVITY_V1_SIZE_QUERY_HIT, "GravityQueryHit layout");
_Static_assert(sizeof(GravityWorldStats) == GRAVITY_V1_SIZE_WORLD_STATS, "GravityWorldStats layout");
#endif

#endif
