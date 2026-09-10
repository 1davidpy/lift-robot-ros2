# luxi_interfaces — ROS 2 消息与服务定义

msg/ 和 srv/ 为统一接口；双臂状态、命令按 right[8],left[8]，时间字段区分 ROS 和主机单调时钟。修改接口后需全量重建下游。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
