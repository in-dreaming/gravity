# Task 14：岛、DOF 锁与约束行

## 目的

实现确定性岛划分、统一6D约束行和线性/角向DOF lock，包括2D preset。

## 依赖

Tasks 03、12、13。

## 交付物

- ordered graph/island BFS；
- 6D Jacobian row、effective mass、limits/bias/impulse；
- translation/rotation axis lock约束；
- 2D preset；
- graph oracle、row解析与重建测试。

## 详细实现架构

节点是awake dynamic body；contact/joint/DOF edge有序。static/kinematic参与row但不通过自身合并dynamic islands。IslandId=min BodyId。ConstraintRow包含JA linear/angular、JB、effective mass、bias、lower/upper、accumulated impulse和完整row key。

DOF lock在body创建/命令边界定义world-space锁轴：2D锁Z translation及X/Y rotation。锁行与joint/contact一同PGS，不能每Tick直接把坐标清零。合法active row的K=0设置InvalidConstraint fault。

## 实施步骤

1. 构建有序邻接/BFS。
2. 定义6D row和wide effective mass。
3. 实现DOF锁row与2D preset。
4. 实现row全序和fixed buffers。
5. 随机graph与解析row测试。

## 验证

- connected component等于oracle；
- static bridge不错误合岛；
- 输入顺序不影响island/row；
- 2D case保持Z和倾斜角自由度约束；
- row K/bounds正确，零K fault；
- 多次重建bytes相同。

## 完成判定

岛、6D row和DOF锁真实进入求解系统。直接清transform冒充2D约束不算完成。

