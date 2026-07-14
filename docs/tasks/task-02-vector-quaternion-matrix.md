# Task 02：Vec3、Quaternion、Matrix 与 Transform

## 目的

实现 3D 几何和动力学唯一数学层，以及可复现三角表和 canonical quaternion。

## 依赖

Task 01。

## 交付物

- Vec3、Mat3、SymmetricMat3、Quat、Transform3、Plane、Ray、Aabb3；
- dot/cross/normalize、矩阵逆/乘、quaternion multiply/rotate/integrate；
- canonical quaternion sign和确定归一化；
- 96 轮 Q32.224 CORDIC 生成的 sin/cos 表、pi/tau raw；
- 代数、等变、golden和奇异边界测试。

## 详细实现架构

dot/cross/matrix 使用 Task 01 wide API。Quat约定 `(x,y,z,w)`，主动旋转 local→world，组合顺序明确为 `q_world = q_parent * q_local`。角速度是 world-space rad/s；半隐式积分 `q += 0.5*dt*[ω,0]*q` 后归一化与 canonical sign。

Mat3 inverse 对 determinant 太小返回 invalid/fault；惯量使用 SymmetricMat3 避免非对称漂移。AABB 相切算 overlap。CORDIC generator禁止 host libm并入库表/hash。

## 实施步骤

1. 实现 Vec3/wide dot/cross/normalize。
2. 实现矩阵、对称惯量和安全 inverse。
3. 实现 quaternion、canonical sign、rotate/integrate。
4. 实现 Transform3/Aabb3/Plane/Ray。
5. 生成并验证 trig 常量/表。
6. 添加旋转组合、逆变换和惯量旋转性质测试。

## 验证

- q/-q 输入规范为同一 raw quaternion；
- 单位 quaternion 长稳积分保持规定误差；
- rotate/inverseRotate 与 matrix 表示一致；
- 奇异矩阵、零 normalize 确定 fault；
- AABB/swept AABB 包含真实边界；
- 三模式/native/WASM逐位一致。

## 完成判定

3D 数学可直接支撑碰撞和动力学，无平台 trig、临时 Vec2 管线或未规范 quaternion。

