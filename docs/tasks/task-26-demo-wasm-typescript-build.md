# Task 26：Demo WASM、TypeScript 封装与统一构建

## 目的

建立完全隔离的Web Demo基础：正式Zig C ABI编译为WASM、类型安全TypeScript wrapper，并由Zig Build统一构建和本地运行。

## 依赖

Tasks 22、23。

## 交付物

- `demo/web`独立pnpm项目、lockfile、Vite/TS配置；
- `wasm32-freestanding`正式engine artifact；
- TypeScript C ABI wrapper、linear-memory arena、command/event/query/snapshot批处理；
- `zig build demo`与`zig build demo-run`；
- isolation、ABI parity、memory lifecycle和headless browser tests。

## 详细实现架构

Core/root/package manifest不引用demo。`zig build demo`显式执行：构建WASM→验证exports→`pnpm install --frozen-lockfile`→Vite production build到`zig-out/demo`。`demo-run`启动本地Vite server。

Wrapper只转换raw Fp/Vec3/Quat/ID、管理WASM memory、批量命令/结果；不得实现碰撞、积分或数值近似。使用生成的TS ABI常量/布局文件，其来源是同一ABI schema，CI与gravity.h/Zig layout三方校验。WASM默认single worker。

## 实施步骤

1. 建立独立package/lock/Vite/TS strict工程。
2. 接入Zig build依赖与artifact路径，不写generated到core。
3. 实现WASM加载、AssetStore/World RAII wrapper。
4. 实现step/query/snapshot/hash/event批量API。
5. 实现错误映射、内存增长/释放与dispose。
6. 加Playwright或等效headless smoke和isolation tests。

## 验证

- 普通`zig build`、其他仓库Zig引用不需要Node/pnpm且不包含demo；
- `zig build demo`冷构建成功且lock严格；
- `zig build demo-run`本地可启动；
- TS strict无any逃逸，wrapper无physics math；
- C/Zig/WASM同replay hash一致；
- 重复create/dispose无memory增长；
- exports/layout变化使CI失败。

## 完成判定

WASM与TS封装是真实正式ABI且隔离。复制engine逻辑到JS、mock WASM或普通core引用拉入demo不算完成。

