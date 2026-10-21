# OpenArm 上电抬臂（网页手动触发）

网页：`http://192.168.0.245:8000`，设备总览下方 **OpenArm 上电抬臂**。

1. 确认 OpenArm 采集开启，左右各 8/8 关节反馈在线。
2. 点击 **抬臂到启动姿态**。不需要 Quest、底盘、升降或录制程序。
3. 双臂及夹爪从当前反馈位置以最高 0.15 rad/s 移动，到位后持续保持转矩。
4. **停止抬臂并保持** 可中止轨迹；再次点击抬臂才继续。
5. 如需解除保持，请先支撑机械臂，再点击 **撤销抬臂使能（移除转矩）**。

开机、采集服务启动、页面刷新不会自动抬臂。当前实现为手动验证阶段，未加入无人触发的开机运动。

目标来自 2026-10-21 用户截图，单位 rad：

| 机械臂 | J1 | J2 | J3 | J4 | J5 | J6 | J7 | 夹爪 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 左 | -0.064 | 0.000 | 0.133 | 1.409 | 0.090 | 0.162 | 0.648 | 0.000 |
| 右 | 0.000 | -0.003 | 0.011 | 1.583 | -0.001 | -0.068 | -0.598 | 0.000 |

配置位置：`Project/robot_bringup/config/arms-kh.json` 的 `startup_pose`。ROS 消息始终为右 8 + 左 8 顺序。原 `home`、零点、关节限位与遥操作配置不变。

接口：`POST /api/arm_startup`，JSON `{"command":"start"}`、`{"command":"stop"}` 或 `{"command":"disable"}`；ROS 服务 `/luxi/arms/startup`，状态 `/luxi/arms/startup_status`，网页 `/api/state` 的 `arm_startup` 字段。

到位需全部 16 轴位置误差 ≤ 0.02 rad、反馈速度 < 0.08 rad/s，并稳定 0.5 s。轨迹使用单调时钟，单次步长按最多 50 ms 限制；45 s 未到位或跟随误差 > 0.35 rad 时停止并保持。丢反馈时停止轨迹，反馈恢复也不自动继续。只读切换到 SDK 后重新核验读数；相对切换前变化 > 0.05 rad 不使能。

实机初测发现原比例位置控制在负载下有约 0.10 rad 的静态偏差。现在在名义轨迹完成后使用受限积分补偿：补偿系数 0.5/s，幅度限制 ±0.2 rad，变化率限制 0.03 rad/s，最终发送目标仍遵守关节限位与 0.15 rad/s 速度上限。夹爪不参与积分补偿。停止和再次启动保留已有支撑目标及补偿，避免因把目标直接改成测量值而卸掉支撑转矩。状态中的 `static_compensation_rad` 可用于检查各轴补偿量。

保持期间关闭/重启 OpenArm 服务会执行驱动原有退出失能流程、移除转矩。先由现场人员支撑机械臂。网页关闭不会终止板端已提交的有限轨迹，也不会移除保持转矩。

启动轨迹执行中拒绝遥操作接管；到位或停止后可以由原有显式使能接管，原系统的 Quest/底盘/升降门禁仍生效。独立抬臂不会发布底盘或升降指令。

模拟回归（ROS domain 108，仅模拟电机）：

```bash
source /opt/ros/humble/setup.bash
source build/install/setup.bash
python3 Project/device_adapters/test/test_openarm_startup.py "$PWD"
```

修改前源文件备份：`work/startup-pose-backup-20261021/before.tgz`。

2026-10-21 gripper coordinate calibration: current physical positions are now logical zero, using persistent software offsets right=-0.3732738231479363 rad and left=-0.36717021438925773 rad. Encoder firmware zero was not changed. Gripper home/open/closed/limits/startup values were translated together, preserving all raw physical targets. J1-J7 configurations are unchanged. See Project/robot_bringup/config/gripper_zero_calibration.json. The internal restore_hold command restores captured holding targets without a startup trajectory after an explicitly supported maintenance restart.
