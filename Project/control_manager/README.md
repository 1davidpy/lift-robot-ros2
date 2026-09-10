# luxi_control_manager — 控制阶段管理

管理 disabled/homing/aligning/teleop/hold/returning/fault；输出 /luxi/command，通过 /luxi/control 服务启停。反馈和输入超时禁止自动继续。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
