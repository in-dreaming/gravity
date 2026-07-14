# Task 31（可选）：可变 Mesh、切割与断裂调研

## 目的

研究运行时顶点/拓扑变化、切割和断裂对 Gravity 的影响，形成不会破坏现有只读 TriangleMesh、确定性和稳定 ABI 的后续演进方案。该任务不改变当前“运行时 mesh 拓扑不可变”的产品约束。

## 依赖

只依赖 `setup.md`。可引用 Tasks 29/30 的最终调研结论，但不得把它们视为前置完成条件；所有引用内容必须在本文概括到足以独立决策。

## 交付物

- `docs/research/future/deformable-mesh-fracture.md`；
- 仅顶点形变、局部拓扑编辑、预切 fracture graph、运行时任意 fracture 四种路线的决策矩阵；
- asset/instance/revision/handle 所有权和确定 mutation command 设计；
- dynamic BVH refit、局部 rebuild、全量 deterministic rebuild 的策略与阈值；
- 碰撞缓存失效、质量/惯量重算、island、CCD、快照和 rollback 设计；
- 与 cloth/soft body 的职责边界及共享 deformable geometry 基础层建议；
- C ABI 和版本迁移策略、资源/安全上限；
- 分阶段但每阶段可完整验收的后续任务 DAG；
- 至少 5 个论文、标准或成熟引擎官方资料来源。

## 详细实现架构

研究必须将当前 `GravityAssetStore` 的不可变资产与未来 world-owned mutable geometry 明确分开。每次 mutation 需要 canonical command、单调 revision、稳定生成 ID、资源预算检查和原子提交语义；同 Tick 多个切割/断裂请求必须定义排序、冲突处理与失败回滚。

几何层必须比较 half-edge、winged-edge、indexed triangle soup 与 fracture graph，定义顶点/边/面 ID 生命周期、洞和非流形处理。动态 BVH 方案需要规定确定 refit/rebuild 顺序、SAH 等价项、并列规则、内存分配和 worker 归并；禁止按 wall-clock 或平台浮点误差触发 rebuild。

物理层必须说明几何 revision 如何使 broadphase proxy、contact cache、feature ID、mass properties、sleep、CCD sweep 和查询结果失效或迁移。断裂产生多个 rigid body 时，需定义质量/动量守恒、COM 变换、速度继承、handle/event 顺序和关节归属。

状态层必须比较快照完整 mesh、mutation log + checkpoint、copy-on-write page 三种模式，并给出 rollback 时间/空间上界。网络输入必须只传规范 mutation command 或经内容哈希标识的预烘焙 fracture 选择，不能依赖客户端本地非确定几何运算。

安全章节必须覆盖恶意细碎化、几何爆炸、退化三角、重复切割、深 BVH、整数溢出和内存耗尽；所有上限与错误必须是协议状态的一部分。

## 实施步骤

1. 分别研究 vertex-only deformation、预切断裂和任意拓扑编辑，列出算法与工程证据。
2. 结合当前资产/shape/body/contact/snapshot/C ABI 边界建立影响矩阵。
3. 选择推荐的首次增强范围，并明确拒绝同时支持的能力和之后的兼容演进点。
4. 定义 mutable geometry store、revision、ID、mutation transaction 和确定性 job 阶段。
5. 写出 BVH 更新、接触缓存迁移、质量重算与 body split 的规范伪代码和 tie-break。
6. 建立固定点几何谓词策略；评估需要精确整数谓词或离线预切的位置。
7. 估算 100k triangle、连续形变和高碎片场景的 CPU/内存/snapshot 上限。
8. 形成可独立实施的任务 DAG、测试 corpus、fuzz 属性和性能门禁。

## 验证

- 当前 immutable TriangleMesh 路径保持零额外运行时依赖和零行为变化；
- 任意 mutation 对 topology、BVH、contact、mass、snapshot 和事件的传播无遗漏；
- 所有阈值、排序、ID、失败和资源上限确定且可编码；
- 方案解释跨平台和 1/2/4/8 worker 的逐位一致性如何验证；
- 后续测试至少包括重复 replay、随机 rollback、切割冲突、body split 动量、退化/恶意输入和 BVH 最坏情况；
- 文档明确研究结论，不用“实现时决定”、mock 或临时全量重建冒充最终架构。

## 完成判定

形成可评审冻结的演进路线，首次增强范围、长期兼容点、核心数据结构、确定性 mutation、BVH、物理传播、rollback、ABI、安全和任务拆分均闭合。破坏现有只读资产模型或仅能演示视觉碎裂的方案不算完成。
