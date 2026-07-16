# Task 25：Fuzz、安全与鲁棒性

## 目的

对所有不可信输入和复杂几何/状态序列做系统fuzz与安全加固，消除panic、越界、资源炸弹和未定义状态。

## 依赖

Tasks 05、21、23。

## 交付物

- asset/snapshot/replay/decimal/C ABI command/query fuzz targets；
- geometry/GJK/EPA/BVH/mesh-mesh/state-machine fuzz；
- corpus minimizer与回归保存；
- pointer/length/alignment/overflow安全审计；
- SBOM、许可证、威胁模型和安全响应文档。
- Spindle submodule commit/license/SBOM 条目，以及 Gravity adapter 的 Task
  submit/reset/queue-release/cancel/backpressure/shutdown/错误回调有界sequence
  fuzz；不把全面fuzz Spindle内部实现重复归入Gravity产品职责。

## 详细实现架构

Parser fuzz不允许按输入大小无界分配/递归。Geometry fuzz生成合法与退化shape，检查invariant、A/B swap、brute oracle和跨mode hash。Sequence fuzz随机create/destroy/joint/rollback/worker切换。C ABI fuzz在可控地址空间组合null、misaligned、overlap buffer和长度溢出。

失败自动输出最小replay/asset并进入tests/fuzz/corpus。不得把crash标记为“无效输入正常”。

## 实施步骤

1. 写威胁模型和每入口resource limits。
2. 建立各fuzz harness与sanitizer可用构建。
3. 持续运行固定CPU小时并收敛coverage。
4. 修复、最小化、添加回归。
5. 审计依赖/许可证/SBOM。

Spindle ECS、Workflow、SQLite 与 archive 必须在构建图审计中保持禁用；
Gravity只允许`spindle_executor`窄入口，fuzz不得依赖Spindle aggregate Runtime、
parallel、Local Task Graph或可选persistence dependency。发现Spindle缺陷应在
Spindle添加最小回归并由Gravity固定对应adapter corpus。

## 验证

- 每target达到冻结时长/coverage且无未处理crash；
- malformed输入World不变或确定Faulted；
- 无OOB/UAF/leak/infinite loop/资源炸弹；
- fuzz corpus在三模式重放一致；
- 所有已修问题有最小回归。

## 完成判定

安全门禁全部通过。只跑parser happy path、忽略timeout/OOM或不保存corpus不算完成。
