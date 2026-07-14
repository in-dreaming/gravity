# Task 13：刚体、命令与 3D 积分

## 目的

实现3D Body/Collider状态、命令事务、完整惯量、force/torque/impulse和quaternion积分。

## 依赖

Tasks 02、03、07。

## 交付物

- Body/Collider SoA与World regions；
- create/destroy、force、torque、impulse-at-point、velocity、kinematic target、DOF lock命令；
- 命令全序、全量预验证和事务提交；
- gravity、阻尼、速度clamp、position/quaternion积分；
- free-flight、gyroscopic、kinematic和ID生命周期测试。

## 详细实现架构

Body保存position、canonical quaternion、linear/angular velocity、inverse mass、local inverse inertia、force/torque、type、DOF lock。world inverse inertia每子步计算。角速度world-space rad/s；包含$-I^{-1}(ω×Iω)$陀螺项并固定运算顺序。

命令key按setup。全部validate成功后才commit；validation错误World不变。Kinematic target生成一Tick速度并在Tick末精确snap到目标raw。v1无teleport；创建初始transform是唯一直接设置。

## 实施步骤

1. 定义SoA/layout和BodyDesc校验。
2. 实现质量/惯量与DOF lock应用。
3. 实现command sort、冲突矩阵、事务。
4. 实现force/torque/impulse和3D积分。
5. 实现kinematic target与destroy cascade hook。
6. 添加解析自由体/旋转/命令测试。

## 验证

- static不动、kinematic不受力、dynamic符合golden；
- 偏心impulse产生正确线/角速度；
- 非球形惯量自由旋转与reference一致；
- quaternion始终canonical；
- 命令乱序经key规范后相同；非法命令整批不变；
- 2D lock约束状态正确提供给Task14。

## 完成判定

3D积分和命令完整。标量惯量、Euler angle、float dt或部分commit不算完成。

