# luxi_bringup — 整机启动和集成验证

launch/teleop.launch.xml 启动正式节点。arms.json 保留旧接口，arms-kh.json 使用鲲弘稳定通道名。integration_probe 合成 Quest UDP 并验证 mock 控制、录制及回传。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
