# Gravity 3D 一致性物理引擎架构

> 状态：已按产品决策校准。实现细节与验收的规范来源是 [`../tasks/setup.md`](../tasks/setup.md) 和对应任务文档；本文说明模块关系和端到端原理。

## 1. 架构目标与不变量

Gravity 是 Zig 0.16.0 编写的 3D 刚体引擎。确定性是协议属性，不是某种构建选项。任何影响未来 Tick 的值都必须满足：表示唯一、运算唯一、遍历顺序唯一、并行归并唯一、保存/恢复唯一。

一次模拟由 `(protocol_version, config, asset_hashes, snapshot, ordered_commands)` 唯一决定。系统时间、地址、hash table 遍历、线程完成顺序、平台 libm、未初始化内存、编译器 fast-math 和不定迭代不得进入语义路径。

3D 是唯一核心。所谓 2D body 只是设置线性 DOF mask 为 XY、角向 DOF mask 为 Z；形状和碰撞仍处于 3D 世界，没有分叉的 2D solver 或 snapshot schema。

## 2. 数值内核

语义标量是单个 `i64` Q32.32，原始值 `raw` 表示 `raw / 2^32`。乘除、点积、叉积、行列式、惯量变换和归一化使用 `i128` 或按任务规范实现的更宽整数中间值，随后以固定的 round-to-nearest、ties-to-even 规则回到 Q32.32。溢出、除零和非法归一化必须返回确定错误，不能按 build mode 改变行为。

`Vec3`、`Quat`、`Mat3`、`Transform` 只由该标量组成。姿态为 canonical unit quaternion：归一化算法、零长度处理和符号半球规则固定，因此 `q` 与 `-q` 不会产生两种序列化。核心不调用 libc/libm；需要的平方根、倒数平方根和三角函数由整数算法、固定迭代或版本化查表实现。

世界范围、最大速度/角速度、shape extent、质量、惯量、冲量和 solver 校正都属于协议配置并有静态范围证明。计算不能以“ReleaseFast 恰好不溢出”作为正确性依据。

## 3. 状态与所有权

World 使用容量受控的 SoA pool 和 generation handle。ID 的分配、回收和迭代按规范键排序，禁止地址成为 ID。运行时不在 step 热路径做无界分配；容量耗尽返回确定错误且不产生半提交状态。

状态分为三类：

- 未来相关逻辑状态：body、joint、contact warm impulse、sleep、command sequence、allocator generation 等，进入 canonical snapshot；
- immutable asset：凸包、mesh、heightfield、compound 和烘焙 BVH，由内容哈希引用；
- 可重建派生状态：SAP endpoints、BVH traversal scratch、island、constraint rows、job batches，load 后按稳定顺序重建。

TriangleMesh 作为刚体时几何仍是只读资产。闭合、定向一致、无自交且 baker 验证通过的 mesh 可使用烘焙质量属性；否则 dynamic body 必须提供通过正定性和范围校验的质量、COM、惯量 override。

## 4. 固定 Tick 管线

每 Tick 以固定 60 Hz 和默认两个子步执行。调用方不能传任意 delta time。

```text
validate/sort commands
        ↓
apply mutations and wake decisions
        ↓
integrate forces + predict transforms
        ↓
rebuild 3D SAP / produce canonical pairs
        ↓
narrowphase + manifold reduction
        ↓
contact cache + ordered events
        ↓
build islands / DOF rows / joint rows
        ↓
warm start + fixed-count PGS iterations
        ↓
integrate/correct transforms + sleep
        ↓
CCD TOI subpipeline where required
        ↓
canonical hash / snapshot bookkeeping
```

外部 mutation 只能通过带 Tick、sequence 和稳定目标 ID 的命令进入。回调不能直接修改 world；contact modification 接收确定输入并产出受校验的确定数据。

## 5. 碰撞架构

### 5.1 广相与静态加速

动态/运动 proxy 每子步以 ColliderId 顺序建立 3D SAP。X 为主轴，Y/Z 复核；endpoint key、min/max 并列、pair canonicalization 与 radix sort 都有固定规则。全量规范重建优先保证 oracle 正确性，后续优化只能在 hash 与顺序完全不变的前提下替换内部实现。

TriangleMesh、HeightField 和 Compound 使用离线 baker 生成的确定 BVH。节点布局、split tie-break、triangle order 和量化规则被资产版本锁定。runtime traversal 使用显式有界栈和稳定子节点顺序。

### 5.2 窄相与流形

Sphere/Box/Capsule 的常见 pair 使用解析路径。通用 convex pair 使用 GJK distance/intersection、EPA penetration 和 reference/incident face clipping；最大迭代、重复 support、零方向和并列 support 均有唯一终止规则。

convex–mesh 先遍历 BVH，再执行 convex–triangle 并归并 patch；mesh–mesh 离散碰撞执行稳定 BVH pair traversal 与 triangle–triangle。HeightField cell/triangle 化顺序固定。Compound pair 展开为稳定 child path，最终 feature ID 包含资产、child、face/edge/vertex 身份。

每个接触 patch 最多四点，使用固定几何准则和 tie-break 约简。上一 Tick 通过 feature ID 匹配 warm impulse。begin/persist/end 和 sensor 事件按事件种类、body/collider/feature 键排序后发布。

## 6. 动力学与约束

body 保存质心位置、canonical quaternion、线/角速度、逆质量和局部逆惯量。世界逆惯量由姿态确定性变换得到。力/力矩在子步积分，阻尼使用固定有理形式，姿态积分后 canonicalize。

所有接触、DOF lock 和 joint 被编译为稳定 ConstraintRow。island 通过 ID 有序 graph traversal 构造；row key 固定。PGS 使用固定迭代数、warm start、法向非穿透、二维切向摩擦锥和 split impulse。Distance、Ball-Socket、Hinge、Slider、Fixed、Cone-Twist 的 limit、motor、spring/damping 都通过相同 row 基础设施实现。

sleep 是逻辑状态：阈值、连续 Tick 计数、island 共同入睡和所有 wake 原因均序列化。自动阈值不能依赖 wall-clock 或 worker 调度。

## 7. CCD 与查询

凸 shape 和全部子形状均凸的 Compound 可作为 CCD caster；所有 shape 可作 target。通过 swept AABB 生成候选，以 conservative advancement/shape cast 求 TOI，按 `(toi, pair_key, feature)` 选择。TOI 量化、最大迭代、最大 impact 次数和失败策略固定。TriangleMesh 不作 caster；连续 mesh–mesh 不在范围内。

ray cast、convex shape cast、point/shape/AABB overlap 使用与模拟相同的 shape support 和 BVH，结果不得按发现顺序暴露，而按 fraction/distance、body/collider/feature 规范排序并受容量限制。

## 8. 确定性并行

单线程实现是 golden oracle，但并行不是后续可选项。工作从规范排序的输入按固定范围或固定 coloring 划分；worker 只写私有输出或无冲突 owner 分区；归并按 partition index 和稳定键执行。不得使用竞争 append、原子累加或 work stealing 完成顺序决定模拟结果。

必须证明 1/2/4/8 worker 的每 Tick section hash、事件和查询与 oracle 相同。WASM 本地 Demo 可只启用单 worker，但其语义 hash 必须与 native 一致。

## 9. 状态协议与诊断

canonical codec 明确 little-endian、字段顺序、长度、optional、版本、错误和无效 bit pattern。snapshot 带 protocol/config/asset hashes；load 使用 validate-then-commit，失败不能改变 world。rollback ring 保存受预算约束的 checkpoint，恢复后重放有序 command stream。

BLAKE3 对规范化 section 分层哈希，使 body、contact、joint、allocator、sleep 等首次分歧可定位。field diff 能报告第一个不同 ID/字段/raw value。Golden corpus 只能通过协议变更评审更新。

## 10. C ABI 与发布

C ABI 是唯一稳定公共 ABI。`gravity.h` 只使用定宽整数、不透明 handle、显式 size/alignment、版本化 descriptor 和 caller-owned buffer；固定点跨 ABI 传 raw `int64_t`。所有函数返回稳定错误码，不允许 Zig panic、error union、slice、allocator 或 layout 穿过边界。

导出名的 `gravity_v1_` 仅表示 C ABI major 1，不表示“先发布不完整 v1”。同 major 追加能力必须遵守 size-tagged struct 和兼容规则；协议版本、asset format、snapshot schema 与 C ABI version 分开管理。

构建发布 static/shared library、generated header、WASM、baker/CLI、文档、examples、SBOM 和 checksum。C11、C++17 与至少一种非 C 绑定 consumer 进入 ABI CI。

## 11. Demo 隔离

`demo/` 使用公共 WASM C ABI，由 TypeScript 封装 linear memory 和批量接口，Three.js 只渲染，React 只管理控制面板。它不能链接 Zig 内部模块或在 JavaScript 中重复物理计算。

根 `build.zig` 提供显式 `demo` 和 `demo-run` step；默认库构建、测试和作为 Zig dependency 被引用时，不解析 demo package、不要求 Node/pnpm，也不把前端依赖传播给 consumer。Demo 只要求本地运行，不包含部署任务。

## 12. 扩展边界

布料、软体和可变/可破坏 Mesh 不属于当前 product-ready 范围。为了以后扩展，当前实现保留版本化 shape dispatch、独立 asset/runtime geometry 所有权、通用 constraint row、phase graph、事件 namespace、snapshot section 和 size-tagged ABI descriptor；但禁止提前加入没有实现语义的 enum、handle 或函数。

后续技术选择必须由 Tasks 29–31 调研冻结后另行拆分，不能以 mock、临时全量重建或非确定浮点库接入主链。

## 13. 验收与任务映射

Tasks 00–28 构成不可删减的产品链：基础/数学/状态、资产与全部 shape、碰撞、刚体/关节/query/sleep/CCD、world pipeline、rollback/C ABI、确定性并行、优化、fuzz/安全、Demo 和最终产品验收。

最终矩阵覆盖目标 OS/CPU、Debug/ReleaseSafe/ReleaseFast、native/WASM、1/2/4/8 worker、全部离散 shape pair、全部 joint/query、CCD 边界、长跑、随机 rollback、ABI consumers、恶意资产和性能预算。任一必需项未通过即未完成。

完整依赖关系、默认容量、量化常量、错误策略和逐任务验证命令以 [`../tasks/setup.md`](../tasks/setup.md) 与 [`../tasks/README.md`](../tasks/README.md) 为准。
