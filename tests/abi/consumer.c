#include "gravity.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static int check(GravityResult result) { return result == GRAVITY_OK; }

int main(void) {
    GravityBuildInfo info = { sizeof(info), 0 };
    if (!check(gravity_v1_build_info(&info)) || info.abi_version != 1) return 1;

    GravityAssetStoreDesc asset_desc = { sizeof(asset_desc), 0, NULL, 0, 0 };
    uint64_t asset_size = 0; uint32_t asset_align = 0;
    if (!check(gravity_v1_asset_store_memory_required(&asset_desc, &asset_size, &asset_align))) return 2;
    void *asset_memory = malloc((size_t)asset_size);
    GravityAssetStore *assets = NULL;
    if (!asset_memory || !check(gravity_v1_asset_store_init(asset_memory, asset_size, &asset_desc, &assets))) return 3;

    const GravityFpRaw one = INT64_C(1) << 32;
    GravityWorldDesc world_desc = {0};
    world_desc.struct_size = sizeof(world_desc); world_desc.body_capacity = 4;
    world_desc.collider_capacity = 4; world_desc.command_capacity = 4; world_desc.contact_capacity = 4;
    world_desc.max_linear_speed = INT64_MAX; world_desc.max_angular_speed = INT64_MAX;
    world_desc.substeps = 2; world_desc.tick_hz = 60; world_desc.assets = assets;
    uint64_t world_size = 0; uint32_t world_align = 0;
    if (!check(gravity_v1_world_memory_required(&world_desc, &world_size, &world_align))) return 4;
    void *world_memory = malloc((size_t)world_size); GravityWorld *world = NULL;
    if (!world_memory || !check(gravity_v1_world_init(world_memory, world_size, &world_desc, &world))) return 5;

    GravityBodyDesc body_desc = {0}; body_desc.struct_size = sizeof(body_desc); body_desc.body_type = GRAVITY_BODY_DYNAMIC;
    body_desc.transform.orientation.w = one; body_desc.inverse_mass = one;
    body_desc.inverse_inertia_xx = one; body_desc.inverse_inertia_yy = one; body_desc.inverse_inertia_zz = one;
    GravityId body = 0; if (!check(gravity_v1_world_create_body(world, &body_desc, &body))) return 6;
    GravityColliderDesc collider_desc = {0}; collider_desc.struct_size = sizeof(collider_desc); collider_desc.body = body;
    collider_desc.shape_kind = GRAVITY_SHAPE_SPHERE; collider_desc.local.orientation.w = one;
    collider_desc.dimensions.x = one; collider_desc.friction = one; collider_desc.category = 1;
    collider_desc.mask = UINT32_MAX; collider_desc.revision = 1;
    GravityId collider = 0; if (!check(gravity_v1_world_create_collider(world, &collider_desc, &collider))) return 7;

    GravityCommand command = {0}; command.struct_size = sizeof(command); command.type = GRAVITY_COMMAND_VELOCITY;
    command.issuer = 1; command.sequence = 1; command.body = body; command.first.x = one;
    command.transform.orientation.w = one;
    if (!check(gravity_v1_world_step(world, &command, 1))) return 8;
    GravityHash128 before, after; if (!check(gravity_v1_world_hash(world, &before))) return 9;
    uint64_t snapshot_size = 0; if (!check(gravity_v1_world_snapshot_size(world, &snapshot_size))) return 10;
    uint8_t *snapshot = (uint8_t *)malloc((size_t)snapshot_size); uint64_t required = 0;
    if (!snapshot || !check(gravity_v1_world_snapshot_save(world, snapshot, snapshot_size, &required))) return 11;
    command.sequence = 2; if (!check(gravity_v1_world_step(world, &command, 1))) return 12;
    if (!check(gravity_v1_world_snapshot_load(world, snapshot, snapshot_size))) return 13;
    if (!check(gravity_v1_world_hash(world, &after)) || memcmp(&before, &after, sizeof(before)) != 0) return 14;

    GravityPointQuery point = {0}; point.struct_size = sizeof(point); point.filter.category = 1;
    point.filter.mask = UINT32_MAX; point.mode = GRAVITY_QUERY_ALL;
    GravityQueryHit hit; uint32_t hit_count = 0;
    if (!check(gravity_v1_world_query_point(world, &point, &hit, 1, &hit_count)) || hit_count != 1 || hit.collider != collider) return 15;

    free(snapshot); if (!check(gravity_v1_world_deinit(world))) return 16; free(world_memory);
    if (!check(gravity_v1_asset_store_deinit(assets))) return 17; free(asset_memory);
    return 0;
}
