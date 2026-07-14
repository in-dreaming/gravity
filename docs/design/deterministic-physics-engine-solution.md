# Gravity 一致性物理引擎方案

> 本文是方案摘要。实现工作的唯一冻结上下文和验收口径见 [`../tasks/setup.md`](../tasks/setup.md)，任务入口见 [`../tasks/README.md`](../tasks/README.md)。若本文与 `setup.md` 冲突，以 `setup.md` 为准并立即修正文档。

## 1. 方案结论

Gravity 使用 Zig 0.16.0 实现 product-ready 的 3D 确定性刚体物理引擎。相同协议、配置、烘焙资产、初始快照与逐 Tick 命令，在 Windows/Linux/macOS、x86-64/ARM64、native/WASM 以及 1/2/4/8 worker 下必须得到逐位相同的状态、事件、查询结果与哈希。

核心采用 Q32.32 `i64` 定点数、canonical quaternion、固定 Tick、3D SAP、烘焙 BVH、解析窄相 + GJK/EPA + clipping、最多四点 3D 流形、Sequential Impulse/PGS、固定调度和稳定归并。快照、rollback、replay、field diff、C ABI、确定性并行、fuzz、安全、性能、资产工具和本地 Web Demo 都是同一主链的必需交付物。

2D 不是第二套引擎：它是在 3D body 上锁定 Z 平移和 X/Y 旋转的 preset，复用所有碰撞、约束、快照、并行和 ABI 路径。

## 2. 冻结范围

支持 static、dynamic、kinematic body；Sphere、Box、Capsule、ConvexHull、Compound、TriangleMesh、HeightField；完整 3×3 惯量张量；sensor、filter、contact modification；ray/shape cast 与 point/shape/AABB overlap；sleep、CCD；Distance、Ball-Socket、Hinge、Slider、Fixed、Cone-Twist 以及适用的 limit、motor、spring/damping。

TriangleMesh 可作为 static/kinematic/dynamic rigid body，但顶点和拓扑运行时不可变；支持 convex–mesh 和离散 mesh–mesh。HeightField 高度数据不可变，仅支持整体 static/kinematic 变换。复杂几何由 baker 生成版本化、带内容哈希的只读资产。

CCD caster 仅为凸形状或全部子形状均凸的 Compound；目标可以是所有形状，包括 dynamic TriangleMesh。TriangleMesh 不作 caster，不承诺连续 mesh–mesh CCD。

当前产品不实现布料、软体、运行时可变/可破坏 Mesh、粒子、流体或 GPU solver，但现有模块边界不得阻止这些能力以后通过版本化扩展加入。对应研究任务为 Tasks 29–31。

## 3. 交付架构

```text
Application / C, C++, C#, JS, Zig
                │
     stable C ABI + generated gravity.h
                │
 commands ──> World fixed-tick pipeline ──> events / queries / hashes
                │
    ┌───────────┼──────────────┬───────────────┐
    │           │              │               │
 asset store  collision     constraints    state protocol
 baker/BVH    SAP+narrow     contacts/joints snapshot/replay
    │           │              │               │
    └───────────┴──── deterministic job system ┘
                │
          Q32.32 math kernel
```

Zig 模块按 `math`、`core`、`asset`、`collision`、`dynamics`、`query`、`state`、`job`、`abi`、`tools` 分层。底层不能反向依赖上层；稳定 ABI 不暴露 Zig layout、allocator、slice、error union 或第三方类型。world-owned mutable state 与 immutable `GravityAssetStore` 分离。

## 4. 产品完成条件

项目不存在“先交一个缺并行/缺 3D 的版本再补齐”的完成口径。`gravity_v1_` 是 C ABI major 的符号前缀，只用于兼容性管理，不是开发阶段名称。

Tasks 00–28 全部通过才能发布。任何 required shape pair、joint、query、CCD 边界、rollback、worker 数、目标平台或 ABI consumer 未测试，均视为未完成；mock、stub、TODO、跳过测试、临时浮点实现和放宽 golden 都不能通过验收。

Web Demo 采用 Zig→WASM + TypeScript + Three.js + React，展示主要能力。它位于隔离目录，只通过公共 ABI 使用引擎；普通 `zig build` 和其他仓库的 Zig dependency graph 不解析 Node/pnpm/Three.js/React。`zig build demo` 构建，`zig build demo-run` 只启动本地服务。

## 5. 执行入口

- 决策、协议和全局门禁：[`../tasks/setup.md`](../tasks/setup.md)
- 任务 DAG：[`../tasks/README.md`](../tasks/README.md)
- 技术架构展开：[`deterministic-physics-engine.md`](deterministic-physics-engine.md)
- 基础调研：[`../research/1.md`](../research/1.md)
