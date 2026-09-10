# 设备依赖

`repositories.lock` 记录仓库与确切 commit；`patches/` 记录必要差异。`external/` 是校验哈希的归档展开，不含 .git，不作为子模块提交。`sdk/` 用于用户后续提供的许可 SDK，不提交厂商二进制。

OpenArm CAN 的唯一补丁让 write_can[_fd]_frame 失败向调用者抛错，以避免录制伪成功 applied。MuJoCo MJCF 使用与旧项目相同的 2.2.0 模型版本。

[KH-UCANFD-B2B-X4 适配、驱动构建和四路配置](KH-UCANFD-B2B-X4/README.md)。使用 `arms-kh.json` 接入；底盘/升降通道保留关闭。

[R680 全向底盘驱动、Quest/PS2 映射与 CAN 配置](R680/README.md)。CH3 使用经典 CAN 1 Mbps；与双臂 FD 配置分开。
