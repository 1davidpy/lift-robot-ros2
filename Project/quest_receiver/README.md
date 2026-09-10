# luxi_quest_receiver — Quest UDP 输入

输出 /luxi/quest；默认 UDP 5006，可限制 sender_ip。只发布新接收报文，拒绝畸形/重复数据。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
