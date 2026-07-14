# Task 30（可选）：确定性软体系统调研

## 目的

为后续三维体积软体能力确定技术路线，使其可与现有 3D 刚体、碰撞、确定性并行和 rollback 架构组合。该任务不实现软体，也不向当前稳定 C ABI 添加空壳接口。

## 依赖

只依赖 `setup.md`。必须遵守当前 product-ready 主链边界；软体研究失败不得影响刚体发布。

## 交付物

- `docs/research/future/soft-body.md`；
- tetrahedral FEM（显式/隐式）、corotational FEM、XPBD volume/shape matching 的决策矩阵；
- 推荐的体积资产、表面映射、实例状态、材料和求解器架构；
- soft–rigid、soft–mesh、soft–soft 碰撞与耦合设计；
- Q32.32 范围/精度预算、反转单元和退化单元处理规则；
- 确定性并行、snapshot/rollback、事件/查询、C ABI 与资产版本影响；
- 不少于 10 个后续实现任务草案及验收门禁；
- 至少 5 个原始论文或官方技术资料的证据表。

## 详细实现架构

资产模型必须描述四面体节点/单元、表面三角映射、材料分区、静止形态预计算和 baker 验证。运行时模型必须区分位置、速度、逆质量、形变/旋转、约束或应力状态，以及只读拓扑；当前研究不把 fracture 作为软体完成条件。

算法比较必须量化大形变、体积保持、近不可压材料、单元反转、迭代收敛、能量行为和固定步长性能。推荐方案必须说明矩阵/约束的稀疏结构、预计算、每子步阶段、warm state、停止条件和确定迭代次数；不能依赖平台 libm、非确定线性代数库或容差驱动的不定循环。

碰撞必须定义表面 primitive 到现有 shape/BVH 的适配、厚度、稳定 feature、penetration correction 和双向冲量/力反馈。soft–soft 自碰撞必须给出候选结构和邻接过滤，即使建议延后实现也要明确依赖和产品边界。

并行必须分析 element/constraint coloring、固定块和归并次序；快照必须说明哪些预计算属于 asset，哪些历史量影响未来，哪些 BVH/颜色/island 可重建。固定点分析必须覆盖 deformation gradient、determinant、polar decomposition 或其替代、compliance、应力和冲量的位宽。

## 实施步骤

1. 基于论文和成熟实现的公开设计，列出候选方法的方程与实际工程约束。
2. 用 Gravity 的确定性、固定点、60 Hz/2 子步、跨 worker、rollback 要求进行决策。
3. 设计 `.gravity-asset` 的未来 tetra volume section 与 baker 校验，但不修改当前格式。
4. 定义 runtime SoA、表面代理、求解批次、刚体耦合和事件的阶段图。
5. 推导固定点范围、最坏内存、复杂度和 1k/10k/100k tetra 预算。
6. 规定单元反转、退化、溢出和不收敛的确定错误或降级行为。
7. 设计兼容的 ABI 扩展方向和完整后续任务 DAG。

## 验证

- 候选比较包含公式、证据、适用边界和拒绝理由；
- 推荐方案不把关键数学、碰撞或并行策略留到实现阶段决定；
- 固定点推导覆盖所有高动态范围运算，并给出离线高精度 oracle 计划；
- 明确 soft body 与 cloth、deformable/fracture 的共享模块和禁止耦合，避免重复系统；
- 后续验收至少覆盖悬臂梁、自由落体、体积保持、大形变、碰撞堆叠、rollback 和跨 worker hash；
- 不使用临时浮点 runtime、第三方不确定求解器或跳过退化输入。

## 完成判定

架构评审能够基于文档冻结一个可实现软体方案，且数据、算法、耦合、固定点、并行、状态、ABI 和测试均闭合。仅有算法综述或 Demo 级路线不算完成。
