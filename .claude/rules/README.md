# .claude/rules/ — 规则触发表

> 这些规则文件通过 manifest `autoInjection.instructionsFiles` 注册，
> 在兼容 CLI (OpenCode 等) 中自动注入到系统提示。

| 文件 | 触发场景 | 内容 |
|------|----------|------|
| `embedded-safety.md` | 编辑 `.c`/`.h` 文件时 | ISR/volatile/数值安全/外设安全/极性 SSOT/内存约束 |
| `c-style.md` | 编辑 `.c`/`.h` 文件时 | 编码/类型/命名/路径/条件编译/inline 风格 |

## 维护

- 新增规则 → 在 README 新增一行 + 确认 manifest 的 `instructionsFiles` 已包含
- 规则修改 → 同步检查 `代码编写与变更规范.md` 是否冲突
- 删除规则 → 同时从 README 和 manifest 移除
