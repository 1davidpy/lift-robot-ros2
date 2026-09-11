# Web 控制台实现与测试记录

本次将原 C++ `data_server` 迁入 `Project/luxi_web_ctrl`，提供独立网页包、局域网 HTTP 网关和开机服务。原始参考为 `openarm_teleop` 数据采集 UI 的设备检查、三相机预览、使能/暂停和 episode 操作流程。

详细用法见 [Web 控制台 README](../Project/luxi_web_ctrl/README.md)。

## 页面预览

以下截图来自本机 Chromium 自动化测试，截图位于忽略的 build 目录；模拟图像不是实物图像。

![桌面控制台（mock）](../build/web-dashboard-desktop.png)

[手机布局截图](../build/web-dashboard-mobile.png) · [真实模式未连接设备截图](../build/web-dashboard-real-disconnected.png)

## 验证结果（2026-09-10）

| 验证 | 结果 |
|---|---|
| C++ 构建 | 11 个包成功；新网关和 C++ HTTP 测试成功编译 |
| 原有测试及新测试汇总 | 30 tests，0 errors，0 failures，0 skipped |
| 新网关测试 | 静态页面/JS/CSS、CSP、无令牌拒绝、令牌有效、错误参数 400、服务缺失 503、Quest/双臂/R680 新鲜反馈和断流失效、图像过期 503 |
| 原有整链路 | Quest UDP → 映射 → C++ IK → mock 双臂/底盘/三相机 → rosbag/sample → manifest → HTTP 续传及 SHA-256 |
| 桌面浏览器 | 1440 宽页面、三路模拟图像、只读与令牌解锁、底盘停车、预览开关、断网隐藏旧图及禁用按钮，无页面 JS 异常 |
| 手机浏览器 | 390 宽响应式布局，无横向溢出 |
| 板端局域网地址 | 从板端浏览器通过 192.168.3.179:8000 读取 REAL 页面；缺失设备显示未连接、相机关闭、控制按钮不可用；不等同于已在另一台电脑完成验收 |
| 快捷启动隔离 | 常驻 REAL 网页运行时，`--mock` 自动使用域 76 / HTTP 18000 / UDP 15006，确认模拟反馈未进入域 0 的 REAL 页面 |
| systemd | 服务定义校验通过；网页 active，网页及机器人服务 enabled；本次未启动真实机器人服务 |

日志：`log/build-web.log`、`log/build-web-tests.log`、`log/test-web-full.log`、`log/test-web-recheck.log`、`log/web-browser-launch.log`。

首轮新 HTTP 测试的功能断言全部通过，但清理 `ros2 run` 包装进程超时。已将独立服务和测试脚本改为直接执行 C++ 二进制；复测网关测试在约 5 秒内完成，汇总 30 项测试全部通过。首次完整日志保留失败过程，最终结果见 `test-web-recheck.log`。

## 本机部署状态

- 网页服务已启动并设置开机自启，当前地址 `http://192.168.3.179:8000`。DHCP 可能变更 IP，以 `ip -4 addr` 为准。
- `luxi-robot.service` 已设置下次开机启动，初始不使能、不录制；本次没有启动它。
- 当前 KH USB 设备未检测到；未进行真实双臂/R680 运动和实际 Quest 接入测试。
- 海康真实 SDK 尚待验证，实机开机配置默认关闭相机，不会伪造 RGB。
- 控制令牌为 `log/web-control.token`，0600 权限，未提交 Git；浏览器中输入后才授权操作。
- 保留原数据格式和回传工具，不增加训练功能；升降平台仍待接入。

## 2026-09-11：Quest 随网页启动修复

原问题：网页独立运行，但 Quest 接收节点放在整机 launch 中；CAN 未初始化或机器人服务未启动时，UDP 5006 无人监听。

修改为 `luxi-web.service` 自动拉起独立 `luxi-quest.service`，两者使用域 0。Quest 监听 `0.0.0.0:5006`，不读取 CAN、不控制电机，异常退出自动重启；网页重启/停止会同步作用于 Quest 服务。整机服务传 `quest_enabled:=false` 复用接收节点，CAN 准备不再拒绝常驻 Quest 进程。mock 快捷启动仍隔离在域 76、UDP 15006。网页追踪文案按原协议修正为 0=正常、1=过期、2=无效。

已安装并启动该服务；无需手动运行 Quest 接收命令。配套 APK 填板端 IP 和 UDP 5006、持续发送即可由网页显示接收状态；不新增其他 APK 的握手或 Air Link 协议。

验证：

- 相关包编译成功，新网关与录制回传回归通过，测试汇总仍为 30 tests / 0 failures。
- 无硬件驱动运行时，用合成协议报文验证 UDP → ROS → 网页，显示 fresh=true、约 95 Hz；停止发送后 fresh=false。
- 网页重启后 Quest PID 更新，UDP 5006 自动恢复监听。
- 隔离 mock launch 传 `quest_enabled:=false` 时未新增 Quest 接收进程。
- 模拟快捷启动预检通过；实机预检只报告 KH USB 未检测到，不再被正常占用的 UDP 5006 阻断。

日志：`log/build-quest-autostart.log`、`log/test-quest-autostart.log`、`log/quest-autostart-validation.log`、`log/quest-shared-launch.log`。上述 95 Hz 来自合成测试数据，不代表真实头显已经连接；测试后重启接收节点，清除了测试时间戳状态。未进行实机运动。
