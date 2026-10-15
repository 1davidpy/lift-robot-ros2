# 升降握键持续运动修复（2026-09-24）

状态：经用户明确批准，X5 编译通过，9 项虚拟 CAN 回归通过，新版已加载。
实机服务保持 `allow_motion=false`、`enabled=false`；尚未进行本版本实机运动验收。

## 原因与修改

原适配器每次请求 1 mm，收到两个到位锁闸样本便立即请求下一段。
v21 固件的 `Stationary()` 额外要求距 `StopMotion()` 至少 1200 ms。
因此下一段可能被返回 BUSY（result=1），适配器把它当成失败而撤销许可。
现场网页保留的停止原因为 `lift request rejected, result=1`。
该完整实机尝试未被此前限时日志覆盖；随后使用真实旧二进制和虚拟 CAN
复现了首个 1 mm 到位后的 BUSY 自动退出，新二进制同一测试通过。

- 左中指按住下降，右中指按住上升。每次按住只发送一个有限绝对目标，
  采用设备 INFO 返回上下限的内缩边界；默认内缩 5 mm。
- 现有配置 -97..+197 mm 对应目标 -92 / +192 mm。它们是停止边界，
  松手会提前关闭本次 CAN 会话，停止当前运动，不要求一直走到边界。
- 按住期间持续心跳，不再重复发送 1 mm MOVE。内部速度分区仍由固件执行，
  分区交接可能停顿；不是速度伺服或无停顿插补。
- 松手、双键同按、正常到达边界后保留许可；到达边界后不重复启动。
  换向须先完全释放两键。新按键可等待上一动作停稳后启动；释放会取消等待。
- 连续新鲜、有效、锁闸静止反馈至少 1200 ms 后才建立下一运动会话。
  实际发 MOVE/HOME 前仍要求严格新鲜位置；5 秒未停稳则失败。
- B、Quest/反馈失联、无效位置、保护/驱动错误仍会停机并撤销许可。
  BUSY 和不确定 ACK 不被静默忽略，也不自动重发新序号的运动请求。
- HOME 仍返回原保存中心 0 mm，只增加同样的停稳等待。未写中心、限位、
  SPEED、BOOT_HOME 等配置，没有重新刷固件。

旧启动脚本的 `step_um` 参数仅兼容接收，不再决定运动距离；状态增加
`control_mode=hold_to_limit`。动作日志中的 MOVE 目标现在是本次握键方向的
内缩边界，实际高度仍取有效的 10 Hz 遥测；不能把目标直接当成实际轨迹。

## 当前另一个阻塞项

只读 CAN 和 ST-LINK RAM 快照显示：v21、位置有效位为 0、最近结果为
SENSOR_ERROR=3；速度0、锁闸输出、驱动 fault=0、通信错误0、保护锁定0。
当前光耦 raw/stable=2，稳定冲突计数0，不能据此还原较早的瞬时传感器事件。
此时遥测 `current_um=0` 是无效位置的占位值，不是已证明回到中心。

已保存只读诊断在 X5
`/home/sunrise/lift_diagnostics_20260924/held-grip-diagnostics.bin.json`。
没有重置、RECOVER、SET_CENTER 或通过禁用保护来清除该状态。现场运动测试前
须先查清位置无效原因并恢复可信反馈。

## 验证结果

- `cmake --build build/packages/luxi_device_adapters --target lift_adapter --parallel 2`
  成功；仅重建此次修改的升降适配器。
- 原二进制回归在保持握键时失败：`lift request rejected, result=1`；
  模拟设备记录 MOVE 返回 BUSY。新版 9 项回归全部通过，用时 34.378 秒。
- 增加了“等待停稳期间松手不延迟启动”和“统一模式 HOME 到保存中心并锁闸”
  的回归覆盖。脚本语法和启动脚本语法检查通过。
- 原子替换已测试的二进制并以独立升降、禁止运动方式加载；原二进制备份为
  `/home/sunrise/lift_diagnostics_20260924/lift_adapter_before_hold_fix`。
- 15.13 秒实机被动监听收到四页分别 149/148/148/149 帧，约 10 Hz；启动时
  CAN 下线/上线一次来自既有接口准备脚本。该窗口内没有 0x601 请求帧。
- 随后只读 INFO 0..9 核对协议1、固件21、stage2、-97..+197 mm、3000 rpm、
  profile_flags=15、saved_mask=4。没有 HELLO、MOVE、HOME 或标定写入。
- 当前实机仍为位置无效、SENSOR_ERROR=3，驱动故障0、速度0、锁闸输出。
  Quest 当前0 Hz。没有重新启用动作，双臂/底盘/主数采服务仍停止。
- 临时虚拟 CAN `vlift_test` 已删除。当前运动禁止是 `/run/systemd/system/`
  的升降独立测试覆盖配置及 `run_lift_only.sh`；持久 `lift.env` 也保持 false。

日志位于 X5 `/home/sunrise/lift_diagnostics_20260924/`：
`hold-baseline-test.log`、`hold-build.log`、`hold-regression-test.log`、
`hold-motion-disabled-load.json`、`hold-readonly-info.json`。

已加载二进制 SHA256：
`b77b707e7d2d3a596207a91eb503d021e965e58dec242783c0495825cc7a30f5`。

## 回归脚本

`Project/device_adapters/test/test_lift_hold.py` 启动真实适配器进程和 Quest 回调，
以隔离 ROS domain 99、localhost 和虚拟 CAN 模拟固件有限目标及 1200 ms BUSY。
脚本拒绝非 vcan 接口，不创建接口，不连接实体升降板。

覆盖持续握键不在首个 1 mm 自动退出、释放停止保留许可、快速重新按键等待
停稳、边界不重复启动、双键/换向互锁、B、Quest 断流和无效位置停机。

复现测试时，须先建立专用 vcan 接口；生产 CAN 接口不可代入。使用已编译二进制：

```bash
source /opt/ros/humble/setup.bash
source build/install/setup.bash
python3 Project/device_adapters/test/test_lift_hold.py \
  build/install/luxi_device_adapters/lib/luxi_device_adapters/lift_adapter \
  --interface vlift_test
```

离线回归通过不能代替有人监护的短按、持续按住、松手停止及端部验收。
