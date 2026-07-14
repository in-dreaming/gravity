# Gravity Product-Ready 实现统一上下文

> 每个 Agent 开始前必须完整阅读本文与自己的 `task-XX-*.md`。这两份文档必须包含完成任务所需的全部上下文；不得假设 Agent 阅读过历史对话。发现冲突、缺失或无法满足的约束时必须停止并与负责人讨论，不能自行改变设计。

## 1. 最终目标

使用 Zig 0.16.0 实现一个 product-ready、跨平台逐位确定、可回滚、支持确定性并行的 3D 刚体物理引擎，并通过稳定 C ABI 对外发布。

2D 不建立独立物理管线，而是 3D 世界中的平面自由度约束：锁定 Z 平移和 X/Y 旋转。所有 2D case 复用同一碰撞、约束、快照、并行和 ABI 实现。

给定相同的 protocol version、配置、烘焙资产、初始快照和逐 Tick 命令，Windows/Linux/macOS、x86-64/ARM64 和 `wasm32-freestanding` 必须产生逐位相同的：

- 完整逻辑状态与 canonical snapshot；
- 每 Tick 分层状态哈希；
- 有序事件与查询结果；
- save/load 后续结果；
- 任意回滚点恢复并重演的结果；
- 1/2/4/8 worker 与单线程的结果。

整个主任务链完成后才能宣称 product-ready。没有“先做占位 v1、以后补齐”的完成口径。

## 2. 冻结的架构决策

| 领域 | 决策 |
|---|---|
| 语言 | Zig 0.16.0 精确锁版；核心不依赖 libc/libm |
| 公共接口 | C ABI 是唯一稳定公共 ABI；导出前缀 `gravity_v1_` |
| 核心维度 | 3D；2D 由 6-DOF 锁定实现 |
| 标量 | 单个 `i64` Q32.32；`i128`/更宽 Zig integer 仅作中间值 |
| 旋转 | canonical unit quaternion，Q32.32 分量 |
| 时间 | 60 Hz 固定 Tick，默认 2 子步；调用方不传 delta time |
| 广相 | 3D Sweep and Prune：X 主轴，Y/Z 区间复核，每子步规范化重建 |
| 静态加速 | 烘焙 BVH：TriangleMesh、HeightField、Compound |
| 凸体窄相 | 解析快速路径 + GJK distance/intersection + EPA + face clipping |
| 网格窄相 | BVH pair traversal + convex-triangle + triangle-triangle；离散 mesh-mesh |
| 接触流形 | 3D contact patch，每 patch 最多 4 点，稳定 feature ID |
| 求解器 | Sequential Impulse/PGS、warm start、二维切向摩擦锥、split impulse |
| CCD | 凸形状/凸 Compound 可作 caster；目标可为全部形状；TriangleMesh 不作 caster；无连续 mesh-mesh CCD |
| 并行 | product-ready 必需；单线程是 golden oracle；固定分区与稳定归并 |
| 快照 | 保存未来相关逻辑状态；派生 SAP/BVH traversal/island/row 重建 |
| 资源 | 复杂几何离线烘焙到只读 `GravityAssetStore`；运行时拓扑不可变 |
| Demo | 隔离的 Zig→WASM + TypeScript + Three.js + React；Zig Build 统一驱动 |
| 后续增强 | 布料、软体、可变形/可破坏 Mesh 仅做可选调研任务；当前架构预留扩展点 |

## 3. 当前大版本功能范围

### 3.1 Body 与形状

- body：static、dynamic、kinematic；
- Sphere、Box、Capsule、ConvexHull；
- Compound，可包含任意当前只读 shape；
- TriangleMesh：支持 static/kinematic/dynamic rigid body；顶点与拓扑运行时不可变；
- HeightField：支持 static/kinematic 整体变换；高度运行时不可变；
- Dynamic TriangleMesh 支持 convex–mesh 与离散 mesh–mesh；
- 仅闭合、无自交、定向一致且通过烘焙验证的 mesh 可自动计算动态质量属性；其他 mesh 必须提供经验证的质量、质心与惯量张量 override。

### 3.2 碰撞、查询和动力学

- 全部 shape pair 的离散碰撞；
- layer/mask/group、sensor、contact modification 的确定数据接口；
- ray cast、convex shape cast、point/shape/AABB overlap；
- gravity、force、torque、impulse、阻尼、质量和完整 3×3 惯量张量；
- 线速度、角速度和 quaternion 半隐式积分；
- sleeping/waking；
- CCD 采用第 2 节冻结边界；
- Distance、Ball-Socket、Hinge、Slider、Fixed、Cone-Twist；
- 所有关节支持适用的 limit、motor、spring/damping；
- 线性/角向 DOF lock，2D preset 只是 lock 组合。

### 3.3 状态与产品能力

- canonical codec、BLAKE3、snapshot、rollback ring、replay、field diff；
- 确定性 job system 和 1/2/4/8 worker；
- C ABI、静态库、动态库、WASM；
- asset baker、CLI、API 文档、集成例程、fuzz、安全和性能门禁；
- 本地 Web Demo 展示全部主要能力。

### 3.4 不属于当前主链

- 布料、软体、粒子/流体；
- 运行时修改 TriangleMesh/HeightField 顶点或拓扑；
- fracture、切割、可变形 mesh；
- 连续 mesh–mesh CCD；
- GPU 求解；
- 网络传输协议本身；
- 跨 protocol major 的模拟结果兼容。

这些方向分别通过 Tasks 29～31 调研，形成可实施方案，但不进入 product-ready 主链验收。

## 4. 产品默认容量与数值 envelope

这些是 product-ready 默认配置，可在创建 World 时降低或提高；所有容量、容差与迭代字段进入 config hash。超容量必须确定失败，不允许截断或隐式扩容。

| 参数 | 默认值 |
|---|---:|
| Tick | 60 Hz |
| 子步 | 2 |
| 速度/位置迭代 | 10 / 4 |
| Body | 8,192 |
| Collider | 16,384 |
| Joint | 8,192 |
| Command/Tick | 16,384 |
| Broad pair | 131,072 |
| Contact patch | 32,768 |
| Contact point | 131,072 |
| Sensor overlap | 32,768 |
| Event/Tick | 131,072 |
| Rollback window | 120 Tick |
| ConvexHull 顶点/面 | 256 / 512 |
| Compound 直接 child/深度 | 256 / 8 |
| 单 Mesh 顶点/三角形 | 16,777,215 / 16,777,215 |
| HeightField 单轴采样 | 65,535 |
| 运行时坐标建议范围 | ±1,000,000 m |
| 动态物体建议尺寸 | 0.001～100,000 m |
| 建议最大线速度 | 100,000 m/s |
| 建议最大角速度 | 1,000 rad/s |

质量、惯量和冲量不能只靠单个范围表判断。Task 01 必须建立表达式级位宽预算和 envelope validator；任何可能超过 Q32.32 输出范围的 dot/cross/inertia 中间计算保留宽位比较，不能过早窄化。

## 5. 数值语义

### 5.1 `Fp`

`Fp.raw: i64`，实际值为 $raw\times2^{-32}$。

- add/sub/neg/abs：显式 overflow 检测，按数学符号饱和；
- mul/div：`i128` 中间值，round-to-nearest ties-to-even；
- dot/cross/matrix accumulation：使用 `i128` 或任务指定更宽整数，最终统一舍入；
- 正数除零→`Fp.max`，负数除零→`Fp.min`，`0/0`→0；
- 负 sqrt→0；零 normalize→零值与 invalid；全部设置 MathFault；
- fallible 数学接收 `*MathStatus`，记录稳定执行顺序中第一个 fault；无全局/TLS fault；
- MathFault 进入 World、snapshot 与 hash；
- runtime 禁止 float→Fp；输入使用 raw、整数、ratio 或 canonical decimal string；
- 禁止平台 libm、fast-math 和依赖 build mode 的 overflow 语义。

### 5.2 3D 数学

- `Vec3`、`Mat3`、`SymmetricMat3`、`Quat`、`Transform3`、`Aabb3`；
- 角速度单位 rad/s；`pi/tau` 为固化 raw 常量；
- orientation 是 unit quaternion；每次积分后用固定整数 sqrt 归一化；
- quaternion canonical sign：`w>0`；若 `w==0`，依次要求首个非零 `x/y/z` 为正；因此 `q` 与 `-q` 永远只有一个存储形式；
- quaternion 乘法顺序、body/world frame 约定必须固定；
- world inverse inertia：$R I_{local}^{-1} R^T$；
- 2D preset 锁定 translation Z 与 rotation X/Y，不引入 Vec2 状态。

## 6. 默认容差与迭代上限

```text
linear_slop                 = 0.005 m
angular_slop                = 0.5 degree 的固化 rad raw
convex_skin                 = 0.01 m
aabb_margin                 = 0.05 m
max_position_correction     = 0.2 m/substep
max_angular_correction      = 8 degrees/substep 的固化 rad raw
restitution_threshold       = 1.0 m/s
warmstart_normal_cos_min    = cos(30 degrees) 的固化 raw
sleep_linear_threshold      = 0.03 m/s
sleep_angular_threshold     = 0.03 rad/s
sleep_ticks                 = 30
gjk_iterations              = 32
epa_iterations              = 64
epa_max_faces               = 256
shape_cast_iterations       = 32
ccd_max_toi_per_substep     = 8
mesh_bvh_leaf_triangles     = 4
```

所有常量使用 raw 或构建期确定生成，进入 protocol/config hash。算法达到上限未收敛必须明确 fault，不能返回最后近似并声称成功。

## 7. 接触和求解规范

- normal 始终从 Collider A 指向 B；
- ManifoldKey 包含 ordered ColliderId、child path、primitive/triangle IDs；
- 3D contact patch 最多 4 点；删减顺序固定为最深点→最大面积覆盖→最大距离覆盖→feature key；
- mesh 邻接与 welded normal 用于消除内部边 ghost contact；
- 每点 normal impulse 非负；两维 tangent impulse 投影到半径 `mu*lambda_n` 的圆盘；
- friction=`sqrt(mu_a*mu_b)`，restitution=`max(e_a,e_b)`；
- warm start 只有 feature 与 normal 阈值都匹配时继承；
- split impulse 使用独立 pseudo velocity，不向真实速度注入穿透修正能量；
- solver row 全序：joint rows 后 contact rows；每轮完全相同，不按时间或收敛提前退出。

## 8. 稳定顺序

影响状态的所有集合必须有完整全序：

```text
ID             (index,generation)
Command        (phase_priority,issuer,sequence,discriminant)
Broad pair     (minColliderId,maxColliderId)
BVH traversal  (node bounds key,node id)
Manifold       (collider A/B,child path A/B,primitive A/B)
Contact point  (ManifoldKey,feature A,feature B)
Island         (minimum dynamic BodyId)
Constraint row (kind,minBodyId,maxBodyId,ownerId,rowIndex)
Event          (type,collider/body/joint IDs,feature IDs)
Query hit      (fraction,ColliderId,child path,primitive,feature)
```

禁止地址、哈希表迭代、线程完成顺序、原子 append index、不稳定 sort 或对象池偶然 free-list 顺序进入模拟。

## 9. 资源与加速结构

- source asset 是 canonical JSON，所有实数为十进制字符串；
- baker 输出 little-endian TLV binary、BLAKE3 content hash、asset-set manifest；
- ConvexHull 离线构建并验证闭合凸多面体、half-edge 邻接、面 winding；
- TriangleMesh 离线焊接/邻接/法线/BVH；动态质量要求 watertight manifold 或显式 override；
- HeightField 离线分 tile、min/max tree、hole/material；
- Compound 离线 child tree/BVH；
- `GravityAssetStore` 使用调用方内存初始化，可被多个 World 只读共享，必须比 World 生命周期长；
- runtime 不修改 asset；future deformable shape 通过新的 shape provider/revision/cache invalidation 扩展，不污染当前 immutable fast path。

## 10. World 状态边界

必须快照：Tick/config/protocol/asset hash、slot occupancy/generation、body/collider/joint、contact cache/impulse、sleep、DOF lock、MathFault、决定未来 ID/顺序的计数器。

不快照且必须唯一重建：SAP endpoint/pair、BVH traversal stack、island、临时 constraint、query/event/profile buffer。

load 使用两遍流程：第一遍完整验证且不修改 World；第二遍只执行已验证拷贝，然后重建派生状态。下一 Tick 必须等于连续路径。

## 11. C ABI

- opaque `GravityAssetStore*`、`GravityWorld*`；
- caller-provided aligned memory；Tick 内零通用堆分配；
- `GravityFpRaw=int64_t`；Vec3/Quat/Mat3 为定宽 raw struct；ID=`uint64_t(generation<<32|index)`；
- 不使用 C bool、long、size_t、bitfield、enum 字段、柔性数组；result=`uint32_t + constants`；
- Zig `extern struct` 与 C `_Static_assert/static_assert` 双侧 layout 校验；
- command/event/query 批量跨 ABI；buffer 不足返回 required count；
- job system 是 C ABI 的正式能力；宿主可提供 enqueue/wait callbacks，回调禁止重入 World；
- WASM 使用同名导出和 linear memory，Demo TypeScript wrapper 不实现物理数学；
- panic/unwind 不跨 ABI；所有可恢复错误映射稳定错误码。

## 12. 确定性并行

- 单线程实现永远保留为 oracle；
- 主线程先生成有序 work list；worker 写固定 index/range/thread-local buffer；
- AABB、narrow pair、mesh primitive pair、独立 island、只读 query 可并行；
- merge 只按稳定 key；不按完成顺序；
- 不并行 ID 分配、命令提交和共享 impulse 累加；
- 1/2/4/8 worker 与随机/逆序 scheduler 必须逐 Tick hash 等于单线程；
- native 必须支持多 worker；WASM product build必须支持单 worker且结果相同，WASM threads 不作为当前发布门禁。

## 13. Demo 隔离与构建

```text
demo/web/
  package.json
  pnpm-lock.yaml
  vite.config.ts
  tsconfig.json
  src/
    wasm/          C ABI TypeScript wrapper
    physics/       scene bridge, no physics math
    renderer/      Three.js
    ui/            React control panel
    cases/         classic demonstrations
    diagnostics/   hash/perf/rollback panels
```

- core/root Zig package不得导入 `demo/`；其他仓库 `zig fetch`/模块引用只得到核心；
- Demo 依赖 Node.js LTS、pnpm lockfile、TypeScript、Vite、Three.js、React；
- `zig build demo`：构建 WASM + `pnpm install --frozen-lockfile` + Vite production build到 `zig-out/demo`；
- `zig build demo-run`：确保 WASM 与前端已构建后启动本地 Vite server；
- 只要求本地运行，不做托管部署；
- Demo case 必须调用正式 C ABI/WASM，禁止 JS 侧复制物理、mock 或专用后门。

## 14. 目标目录

```text
build.zig / build.zig.zon / .zigversion
include/gravity.h
src/
  math/ core/ geometry/ assets/ collision/ dynamics/ query/ state/ jobs/ abi/
tools/
  bake.zig replay.zig state_diff.zig benchmark.zig
demo/web/
tests/
  unit/ geometry/ scenarios/ determinism/ abi/ fuzz/ golden/
docs/
  api/ formats/ integration/ research/
```

核心依赖方向：`math → core/geometry → assets/collision → dynamics/query → state/jobs → abi`。Tools 与 Demo 不能被 core 依赖。为后续软体预留的是接口边界、shape kind 扩展、constraint provider 与 cache revision，不是当前空实现。

## 15. 哈希与版本

统一使用 Zig 0.16.0 `std.crypto.hash.Blake3`。Hash128 是完整 digest 前 16 字节。首段必须是 domain tag：

```text
gravity/config/v1
gravity/asset/v1
gravity/asset-set/v1
gravity/state/v1
gravity/snapshot/v1
gravity/replay/v1
```

C ABI version、protocol version、snapshot format、asset format 分开。任何影响模拟结果的数学、容差、排序、算法或迭代变化必须提升 protocol version 并显式更新 golden；测试不得自动覆盖 golden。

## 16. 代码质量与禁止事项

必须：

- `zig fmt --check`；
- 公开 API 有单位、范围、所有权、错误语义文档；
- 热路径零通用堆分配；
- 所有迭代有固定上限；
- capacity/overflow/corrupt input 有事务或 Faulted 语义；
- production implementation 同时有 unit/property/scenario/determinism tests；
- 数据结构单一来源，模块边界符合依赖方向；
- 安全解析所有 asset/snapshot/replay/C ABI 输入；
- 保留第三方许可证和锁定依赖。

禁止：

- mock、stub、TODO、空函数、固定返回成功；
- 以测试 skip、减少 shape pair、关闭功能、减少迭代通过验收；
- runtime float、platform libm、system RNG、wall clock 影响模拟；
- 将“近似相同”作为确定性通过；
- silent fallback、截断、隐式扩容、错误后继续返回成功；
- 为 Demo 增加 core 特权接口；
- 未经确认更改本文冻结决策。

## 17. 任务完成定义

任务只有同时满足以下条件才能完成：

1. 全部生产功能真实实现，无 mock/stub/TODO/临时路径；
2. 依赖任务全部通过且公共契约未破坏；
3. 任务规定的 unit/property/scenario/determinism/ABI/fuzz/benchmark 全部通过；
4. Debug、ReleaseSafe、ReleaseFast 结果一致；
5. 要求的 native/WASM/worker 矩阵通过；
6. 无未解释 skip，无降低断言/阈值/覆盖；
7. 失败和容量边界已验证；
8. 文档、格式、版本、golden 同步；
9. Agent 报告变更、命令和结果；
10. 未完成项必须为“无”，否则状态保持未完成。

## 18. 主任务依赖 DAG

```text
00 foundation
└─01 fixed-point-wide-math
  ├─02 vector-quaternion-matrix
  └─03 memory-ids-config-radix
      └─04 canonical-codec-hash

02+03+04 ─05 asset-format-baker
02+05    ─06 convex-hull-mesh-heightfield-bvh
02+03+05+06 ─07 runtime-shapes-mass-filter
07       ─08 broadphase-3d-sap
07       ─09 analytic-collision
06+07+09 ─10 gjk-epa-convex-manifold
06+07+10 ─11 mesh-heightfield-collision
04+10+11 ─12 contact-cache-events
02+03+07 ─13 bodies-commands-integration
03+12+13 ─14 islands-dof-constraint-rows
12+14    ─15 contact-solver-3d
14+15    ─16 joints
08+09+10+11 ─17 queries
14+15+16 ─18 sleeping
08+10+11+17 ─19 ccd
08+12+13+14+15+16+17+18+19 ─20 world-pipeline
04+12+16+20 ─21 snapshot-rollback-replay-diff
05+17+20+21 ─22 c-abi-packaging
20+21+22 ─23 deterministic-job-system
20+21+23 ─24 optimization-benchmarks
05+21+23 ─25 fuzz-security-hardening
22+23 ─26 demo-wasm-typescript-build
26 ─27 demo-three-react-cases
22+23+24+25+27 ─28 product-qualification

可选调研，不阻塞主链：
29 cloth research
30 soft-body research
31 deformable-mesh/fracture research
```

Agent 只能在全部依赖完成后开始。仓库状态与依赖声明不符时必须停止报告。

## 19. 跨任务规则

- 可修复直接阻塞的依赖 bug，但必须添加回归测试并单独报告；
- 不得擅自重构其他任务公共接口；
- 发现范围、性能、数值、ABI 或算法矛盾，提交证据、选项、影响和建议，等待确认；
- 不得以“先这样、以后改”推进；
- 可选调研只输出证据与方案，不创建主干空接口或伪实现。

## 20. Agent 交付格式

开始：

```text
已阅读 setup.md 与 task-XX。
依赖状态：
计划修改与测试：
冲突/问题：无；若有则停止。
```

完成：

```text
实现摘要：
变更文件：
关键不变量：
验证命令与结果：
golden/benchmark/ABI 变化：
未完成项：无
非任务范围后续：
```
