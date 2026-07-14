# Task 03：内存、ID、配置与稳定排序

## 目的

实现固定内存 World、稳定 generation ID、容量/配置验证和确定 radix sort。

## 依赖

Task 01。

## 交付物

- arena/region layout、slot pool、bitset、fixed vector；
- Body/Collider/Joint/Asset IDs；
- SimulationConfig/CapacityConfig；
- 32/64/128/复合 key 稳定 radix sort；
- memory requirements、canary、随机模型测试。

## 详细实现架构

调用方提供一块对齐内存；init 用 checked size/alignment 计算固定 regions。Tick 内不分配。slot总取最低空闲 index，删除递增 generation；generation溢出永久 retire。所有容器容量失败不修改已发布状态。

Config包含 setup 全部容量、容差、迭代、envelope与feature flags，并提供 canonical field visitor。worker count不进入模拟 config hash，因为结果必须相同。

## 实施步骤

1. 实现 checked layout/arena/fixed containers。
2. 实现 ID encode/decode、slot分配/删除/retire。
3. 实现稳定 radix，多 pass顺序写测试。
4. 定义完整 config默认值、范围和cross-field validation。
5. 添加地址扰动、容量、少一字节、错对齐测试。

## 验证

- 操作序列相同则 ID相同；stale/double free拒绝；
- capacity满时状态不变；
- size算术无溢出/越界；
- duplicate key稳定且tie规则正确；
- 默认 config raw有golden；
- worker count变化不改变 config/state hash。

## 完成判定

核心内存/ID/排序全部生产化。通用动态容器、地址排序、隐式扩容或未检查布局不算完成。

