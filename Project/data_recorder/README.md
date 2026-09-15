# luxi_data_recorder — 录制和局域网回传

data_recorder 写 rosbag2、samples.jsonl 和 manifest；HTTP 控制台迁至 `../luxi_web_ctrl`；episode_pull 执行断点续传与 SHA256 校验。test_storage 检查清单和文件哈希；完整链路由 bringup 测试。

Quest 短按 A 开始录制或成功保存；录制中按 B 打开失败菜单并请求双臂返回，左摇杆上下选择原因、A 确认。Y 专用于底盘启用/停用，长按也不会取消录制。取消请使用网页“取消本次并保留数据”或 `/luxi/episode` 的 `cancel` 命令。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
