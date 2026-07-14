# Task 20：完整 World Pipeline

## 目的

组装所有3D功能为原子、固定phase、零Tick分配的正式World.step。

## 依赖

Tasks 08、12、13、14、15、16、17、18、19。

## 交付物

- World memory layout/init/deinit；
- 完整Tick/substep phase machine；
- command→integrate→collision→solve→CCD→sleep→event→hash；
- TickResult/fault/diagnostics/profile counters；
- 综合3D/2D/mesh/joint/CCD长稳场景。

## 详细实现架构

Phase严格为：Tick/command预验证→commit/wake→每子步force/velocity→SAP→narrow/cache→island/rows→warm→10 velocity→position/quaternion→4 split→CCD→Tick末sleep→clear force→events→bounds/fault→hash。

预验证错误World不变。进入模拟后的首个runtime fault停止并置`Faulted{tick,phase,object,error}`，Tick不递增、不发布普通事件；只允许hash、诊断snapshot和load旧snapshot。不得隐式checkpoint回退。

## 实施步骤

1. 计算全部persistent/scratch regions。
2. 实现phase enum/trace与不可重入。
3. 接线全部模块、capacity/fault/event/hash。
4. 建立Sphere/Box/Hull/Mesh/HeightField/Compound综合cases。
5. 建立2D DOF case与全部joint/CCD/sleep组合。
6. 跑1M Tick随机合法命令长稳。

## 验证

- phase trace有golden且不可配置换序；
- Tick零通用分配；
- 非法预验证不变，runtime fault语义准确；
- 1M Tick重复/mode/native/WASM hash相同；
- 所有shape/joint通过正式World.step到达，无测试旁路；
- 2D case完全复用3D状态。

## 完成判定

整个3D runtime闭环。手工拼模块、功能开关绕过必需能力或只比transform不算完成。

