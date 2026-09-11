# USB 掉线与网页恢复记录

2026-09-11 11:48:29，内核报告 `xHCI host controller not responding, assume dead` 和 `HC died; cleaning up`。随后 USB 集线器、KCAN 四路卡与 RGB 相机一起断开，`lsusb` 只剩两个根集线器，can1/can2/luxi_base 消失。网页 HTTP 与 ROS 服务仍运行，显示失效反馈是对实际状态的反映。

故障前 KCAN 驱动持续触发 `sleeping function called from invalid context`：`kcan_usb_write()` 在 netdev 发送及 URB 完成的原子上下文里用 `GFP_KERNEL` 提交 USB URB。已改为 `GFP_ATOMIC`，补丁位于 `device/patches/kcan-usb-atomic-submit.patch`，`tool/build_kh_driver.sh` 会应用该修复。此错误与 USB 主控故障同时出现，但尚不能据此确定主控故障的唯一原因。

## 已执行的恢复

1. 停止 OpenArm、相机和底盘进程，撤销已失效的设备句柄。
2. 对 `xhci-hcd.2.auto` 执行驱动 unbind/bind，USB 设备重新枚举，无需重启整机。
3. 编译、重新加载修复后的 kcan 模块，srcversion 为 `8839ADE76350538A851DF0C`。
4. 配置 can1/can2，并将 can3 恢复为 luxi_base，重新启动各采集进程。
5. 底盘原来是独立启动进程，现在通过临时 systemd 单元 `luxi-base-recovery.service` 恢复；该单元不是新增的开机自启配置，完整机器人启动仍由原有 bringup 负责。

初次恢复时双臂、底盘、头部 RGB 均正常，但恢复右腕采集后，11:53:02 主控再次停止响应，此时已无 CAN 驱动休眠错误。仅头部采集的第二轮测试也出现 USB 断连及 error -71，因此不能将问题仅归于右腕或认定 CAN 补丁彻底修复了主控故障。

最终隔离措施：暂停头部与右腕 RGB 服务，并取消两路开机自启，网页以 `no_cameras:=true` 明确显示采集暂停；双臂 16/16、底盘反馈已恢复，均未使能。相机节点新增 `require_usb` 参数，实际启动脚本启用它，缺失指定 USB 序列号时不进入 SDK 打开流程。Quest 没有 UDP 数据，录制/完整控制节点未启动，网页如实显示未连接。

## 验证与证据

网页测试只读取反馈与预览，不发运动指令。第一次全设备连续测试失败（60 个样本只有 1 个全部有效），记录保存在 `log/hardware-recovery/first-test-failed.json`。隔离 RGB 后的持续观察记录为 `log/hardware-recovery-test.json`，截图为 `build/web-hardware-recovered.png`。原始主控故障日志为 `log/hardware-recovery/controller-failure.log`。

```bash
lsusb
ip -details link show type can
systemctl status luxi-openarm-monitor luxi-base-recovery luxi-rgb-head_rgb
sudo journalctl -k --since '5 min ago' --no-pager
```

USB 主控重绑会使同一主控上的所有 USB 设备掉线，恢复前应先停止相关设备进程。不要在机器人运动时直接运行主控恢复命令。若再次出现设备重新枚举或 `error -71`，继续检查 USB 数据线、集线器供电及主控日志；一分钟正常反馈不等于长期稳定性已验证。

## 恢复 RGB 前提

先稳定相机数据线、USB 3 端口和集线器供电，一次只接一台相机验证。恢复时重新运行 `install_rgb_camera.sh` 启用对应服务，并将 `run_web.sh` 中的 `no_cameras` 改为 false 后重启网页。当前保留 SDK，不需重复下载安装。

最终网页验证通过：暂停 RGB 后，连续 60/60 个一秒样本均为双臂 16/16、底盘反馈有效、所有电机控制未使能，无浏览器 JavaScript 错误。该观察窗口未出现新的 CAN 原子上下文错误或 xHCI 主控死亡日志。RGB 尚未恢复，整机长时间稳定性未完成验证。

## 重新接线后复测（2026-09-11 12:00）

恢复头部采集并在网页显示真实图像，原计划观察 180 秒，实际在 31.88 秒检测到 RGB 失效，提前判定失败。31 个样本中，双臂与底盘均保持有效，未使能运动。12:01:04 内核报告相机设备地址无响应、error -71 和 USB disconnect；本次没有 HC died。右腕在 12:00:20 曾枚举，12:00:37 消失，其采集服务始终未启动。

头部服务保留本次运行与 USB 存在检查，等待重连；未恢复 RGB 开机自启。网页重新开启相机状态显示，并准确提示缺失序列号。需确认复测期间是否有人手动插拔；否则继续检查相机 USB 数据线、接口和集线器供电。日志：`log/rgb-retest-180s.json`、`log/rgb-retest-kernel.log`；截图：`build/hardware-usb-retest-failed.png`。

保存网页截图时头部相机已重新枚举并自动恢复 640×360 图像（帧号 404）。自动重连有效，但由于测试期间发生断流，不能判定稳定性通过。
