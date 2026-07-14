# Gravity Product-Ready 任务索引

所有 Agent 必须先读 [setup.md](setup.md)，再读且只依赖自己的任务文档。

| 编号 | 任务 | 依赖 |
|---:|---|---|
| 00 | [工程、工具链与构建](task-00-foundation.md) | 无 |
| 01 | [Q32.32 与宽位数学](task-01-fixed-point-wide-math.md) | 00 |
| 02 | [Vec3、Quaternion、Matrix 与 Transform](task-02-vector-quaternion-matrix.md) | 01 |
| 03 | [内存、ID、配置与稳定排序](task-03-memory-ids-config-radix.md) | 01 |
| 04 | [Canonical Codec 与 BLAKE3](task-04-canonical-codec-hash.md) | 03 |
| 05 | [资产格式与烘焙器](task-05-asset-format-baker.md) | 02,03,04 |
| 06 | [ConvexHull、Mesh、HeightField 与 BVH 烘焙](task-06-geometry-bvh-baking.md) | 02,05 |
| 07 | [运行时形状、质量属性与过滤](task-07-runtime-shapes-mass-filter.md) | 02,03,05,06 |
| 08 | [3D SAP 广相](task-08-broadphase-3d-sap.md) | 07 |
| 09 | [解析碰撞快速路径](task-09-analytic-collision.md) | 07 |
| 10 | [GJK、EPA 与凸体流形](task-10-gjk-epa-convex-manifold.md) | 06,07,09 |
| 11 | [Mesh、HeightField 碰撞](task-11-mesh-heightfield-collision.md) | 06,07,10 |
| 12 | [接触缓存与事件](task-12-contact-cache-events.md) | 04,10,11 |
| 13 | [刚体、命令与 3D 积分](task-13-bodies-commands-integration.md) | 02,03,07 |
| 14 | [岛、DOF 锁与约束行](task-14-islands-dof-constraint-rows.md) | 03,12,13 |
| 15 | [3D 接触求解器](task-15-contact-solver-3d.md) | 12,14 |
| 16 | [全部关节](task-16-joints.md) | 14,15 |
| 17 | [3D 查询](task-17-queries.md) | 08,09,10,11 |
| 18 | [确定性睡眠](task-18-sleeping.md) | 14,15,16 |
| 19 | [CCD](task-19-ccd.md) | 08,10,11,17 |
| 20 | [完整 World Pipeline](task-20-world-pipeline.md) | 08,12-19 |
| 21 | [Snapshot、Rollback、Replay 与 Diff](task-21-snapshot-rollback-replay-diff.md) | 04,12,16,20 |
| 22 | [C ABI 与多平台产物](task-22-c-abi-packaging.md) | 05,17,20,21 |
| 23 | [确定性 Job System](task-23-deterministic-job-system.md) | 20,21,22 |
| 24 | [性能优化与基准](task-24-optimization-benchmarks.md) | 20,21,23 |
| 25 | [Fuzz、安全与鲁棒性](task-25-fuzz-security-hardening.md) | 05,21,23 |
| 26 | [Demo WASM、TypeScript 与统一构建](task-26-demo-wasm-typescript-build.md) | 22,23 |
| 27 | [Three.js、React 与经典 Cases](task-27-demo-three-react-cases.md) | 26 |
| 28 | [Product-Ready 全面验收](task-28-product-qualification.md) | 22-25,27 |
| 29 | [可选调研：布料](task-29-research-cloth.md) | 无 |
| 30 | [可选调研：软体](task-30-research-soft-body.md) | 无 |
| 31 | [可选调研：可变形 Mesh 与断裂](task-31-research-deformable-mesh-fracture.md) | 无 |

Tasks 00～28 是不可删减的 product-ready 主链。Tasks 29～31 是独立可选调研任务，只输出研究和架构建议，不以空实现污染主链。
