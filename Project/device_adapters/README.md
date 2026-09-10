# luxi_device_adapters — 硬件 SDK 边界

openarm_adapter 负责 SocketCAN 电机协议和实际反馈；hikrobot_camera 负责 MV3D RGB-D SDK。kh_can_setup 负责鲲弘四路识别和配置，见 ../../device/KH-UCANFD-B2B-X4/README.md。mock 为默认；test_camera 使用假 SDK，test_kh_can 检查身份和映射，socketcan_probe 提供只读/虚拟总线验证。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
