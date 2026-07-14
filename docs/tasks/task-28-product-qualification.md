# Task 28：Product-Ready 全面验收

## 目的

完成全平台、全功能、并行、性能、安全、ABI、Demo、文档和可复现发布门禁。

## 依赖

Tasks 22、23、24、25、27完成；并审计Tasks 00～27全部记录。

## 交付物

- Windows/Linux/macOS、x86-64/ARM64、WASM矩阵；
- Debug/ReleaseSafe/ReleaseFast、1/2/4/8 worker矩阵；
- 全shape pair/joint/query/CCD/rollback golden corpus；
- 性能、安全、ABI、许可证/SBOM报告；
- API、C ABI、formats、integration、determinism、limits文档；
- source/static/shared/WASM/demo本地包和checksums；
- product-ready release checklist。

## 详细实现架构

每平台运行同replay并输出逐Tick section hash，汇总仅字节比较。Golden只能通过protocol变更评审更新。两个干净环境构建，剥离明确非语义metadata后artifact逐字节相同。

支持矩阵必须覆盖全部离散shape pair，包括dynamic mesh–mesh；CCD按冻结边界；所有关节；2D preset；随机rollback；worker切换。WASM只要求single worker但hash等于native。

## 实施步骤

1. 搜索并阻断TODO/mock/stub/skip和未覆盖pair。
2. 跑全平台/mode/worker/golden/long-run。
3. 跑1M Tick、100k随机rollback、fuzz与ABI consumers。
4. 验证性能/内存/rollback预算。
5. 验证Demo 15类case与隔离。
6. 完成文档/例程/限制和可复现构建。
7. 生成release artifacts、SBOM、checksum并执行清单。

## 验证

- 全目标逐Tickhash完全相同；
- 所有shape/joint/query/CCD能力有非跳过测试；
- 1/2/4/8 worker等于single；
- 安全/fuzz/ABI/perf门禁全过；
- Demo本地冷构建运行；
- 无未完成主链项；
- 发布包在无源码consumer环境运行。

## 完成判定

所有门禁通过才product-ready。局部平台成功、known issue接受必需项、放宽golden或隐藏失败均不算完成。
