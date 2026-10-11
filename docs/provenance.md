# 来源与整理范围

作者在 2026-10-11 确认：这两套工程中的机械模型与最初 LQR 推导由自己搭建，并选择 MIT 许可证。模型元数据中的作者名称为 Gary，许可证使用同一名称。

本次发布的来源快照：

| 发布目录 | 原工程保存版 |
|---|---|
| `lqr/` | `MyProc/Total1450g` 的整机 1.45 kg LQR 模型、参数与初始化 |
| `lmpc_roll/` | `MyProc/LMPC_Roll` 的 LMPC + 横滚 PD 模型、参数、初始化和测试 |

原始保存文件的 SHA-256 见 [source_snapshot.json](source_snapshot.json)。它记录整理前的源文件哈希，不是发布包所有文件的最终校验清单。

本次整理新增统一入口、环境检查、LQR 自动验证、JSON 摘要、文档、许可证和 Git 忽略规则；LMPC 的运行入口改为按所在目录定位。机械模型、控制参数和控制律保持保存版配置。

原目录中的 `.slx.autosave` 未并入发布；缓存、生成文件、历史备份、旧响应曲线和实机诊断记录均不分发。若需要采用未保存模型，请在原工程中先明确保存版本，再做一次完整验证。

MathWorks 的 MATLAB、Simulink、Simscape、Simscape Multibody、Control System Toolbox 和 Model Predictive Control Toolbox 是外部依赖，不由本仓库 MIT 许可证授权。本包不复制或分发这些产品的源码、库或安装文件。
