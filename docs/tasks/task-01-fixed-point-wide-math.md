# Task 01：Q32.32 与宽位数学

## 目的

实现全工程唯一的 `Fp`、MathStatus、宽位累加与确定舍入，建立 3D 计算的表达式级范围证据。

## 依赖

Task 00。

## 交付物

- `math/fp.zig`、`math/wide.zig`、MathFault/MathStatus；
- add/sub/mul/div/sqrt/reciprocal/ratio/canonical decimal；
- `WideScalar`/宽位 dot accumulation 的受控窄化 API；
- product envelope validator 与位宽分析文档；
- ≥10,000 golden vectors、property tests、native/WASM benchmark。

## 详细实现架构

`Fp.raw:i64`。所有 fallible 运算接收 `*MathStatus`，记录第一个 fault。乘除使用 i128；`sqrt` 对 `raw<<32` 执行固定轮 restoring integer sqrt并 ties-to-even。宽位 API保留未缩放积/和，只有显式 `narrow` 能舍入为 Fp；禁止上层直接 cast。

decimal parser只接受 ASCII `-?[0-9]+(\.[0-9]+)?`，拒绝指数/locale/NaN/Inf。运行时无 float 构造。位宽分析必须覆盖 position、distance²、cross、inertia、angular impulse、GJK/EPA determinant，不允许只分析单次乘法。

## 实施步骤

1. 定义 Fp/MathStatus/fault 优先级和饱和规则。
2. 实现统一商余 ties-to-even helper。
3. 实现算术、sqrt、parser、format、wide accumulate/narrow。
4. 建立配置 envelope validator，在 World init 拒绝必然越界组合。
5. 用独立大整数/有理数工具生成 golden。
6. 跑三模式、x86/ARM/WASM性能和一致性。

## 验证

- i64/i128 全边界、正负 ties、除零、负 sqrt、饱和全部覆盖；
- 宽位 dot/cross 不过早舍入；
- parse/format canonical round-trip；
- 所有平台/mode golden hash 相同；
- envelope 外配置准确拒绝；
- 测试不使用 runtime float oracle。

## 完成判定

全部数值语义、范围和错误路径可证明。两个 int 拼接、wrapping、平台 float 或未审计窄化均不算完成。

