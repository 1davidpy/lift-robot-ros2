# OpenArm 网页状态接入

当前接线：can1（CH1）为左臂，can2（CH2）为右臂。两路使用 CAN FD，仲裁速率 1 Mbps，数据速率 5 Mbps。默认配置为 `Project/robot_bringup/config/arms.json`。

SDK 位于 `device/openarm/openarm_can`，模型位于 `device/openarm/openarm_mujoco`；构建与依赖获取已统一到此目录。

## 启动状态采集

完成构建后执行：

```bash
./Project/luxi_web_ctrl/scripts/install_openarm_monitor.sh
```

该脚本安装并启动开机自启服务 `luxi-openarm-monitor.service`，只配置 can1/can2，不改变底盘接口。节点设置 `monitor_only:=true`，发送 SDK 状态查询，忽略驱动使能/禁用请求，不发送运动目标。与完整 `luxi-robot.service` 互斥，避免重复占用 CAN。完整机器人服务使用重命名后的 `luxi_left/luxi_right` 和 `arms-kh.json`；恢复独立采集前须保证接口名为 can1/can2。

状态链路：OpenArm CAN → openarm_adapter → `/luxi/arms/state` → web_ctrl → `/api/state` → 网页。内部数组固定为右臂 0–7、左臂 8–15，各臂第八项为夹爪；网页按左臂、右臂展示。显示位置、速度、力矩、MOS/转子温度以及每个关节是否有新鲜反馈。关节反馈超时阈值为 150 ms；采集节点在线不代表全部电机在线。

## 排查

```bash
systemctl status luxi-openarm-monitor.service
journalctl -u luxi-openarm-monitor.service -n 40 --no-pager
ip -details -statistics link show can1
ip -details -statistics link show can2
curl -s http://127.0.0.1:8000/api/state
```

采集节点离线：检查服务和 CAN 接口名称。节点在线但 0/8：检查供电、接线、波特率和电机反馈 ID（0x11–0x18）。部分在线：按关节表检查缺失电机。`状态采集 · 未使能` 是正常监控状态，查看状态无需 Quest 或控制令牌。

实机验证：双臂 16/16 关节均收到新鲜反馈，独立采集服务已启用开机启动。网页回归脚本：`python3 tool/test_openarm_web.py`，使用浏览器 HTTP 状态替身验证左右顺序、超时与断线，不发送机器人命令。

验证结果：HTTP 网关回归、浏览器左右臂与反馈失效回归、桌面/手机布局检查通过；独立 ROS 测试域中连续发送使能请求，301 帧模拟状态均保持 `monitor_only` 且 `enabled=false`。
