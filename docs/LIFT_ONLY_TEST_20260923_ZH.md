# X5 升降单独联调记录（2026-09-23）

## 当前结论

编译完成，CH4 CAN 链路、只读 INFO、HELLO/心跳/CLOSE 通过。首次 Quest 中指操作未产生可见位移，不能判定运动验收通过。双臂、底盘和统一控制程序均保持停止。

X5：192.168.3.179；网页：http://192.168.3.179:8000；Quest UDP：5006。

## 已完成

- 构建 luxi_core、luxi_control_manager、luxi_device_adapters、luxi_data_recorder、luxi_web_ctrl。反馈年龄修正后再次增量构建 control_manager 成功。
- 升降节点使用 CH4 / luxi_lift，经典 CAN 500 kbit/s，节点1；CAN 错误计数为0。
- INFO 实测协议1、固件20、stage2、范围−97..+197 mm、巡航3000 rpm、profile_flags=15、saved_mask=4。
- 无运动租期测试：HELLO 成功、29条心跳、CLOSE成功；位置始终100486 µm，故障0。
- 静止反馈回放：640个10 ms周期，运行看门狗误拒绝0；严格测量有效性仍拒绝298个周期。详见 LIFT_FRESHNESS_ZH.md。
- 三个相机约30 FPS；Quest 在用户开启后收到约31–32 Hz。

## 首次实际输入：未通过运动验收

用户报告“Enabled, but no motion”。完整事件记录为 `luxi-lift-only-motion-watch-20260923.jsonl`，时间使用 X5 单调时钟。

1. 网页启用后读取10页 INFO 成功，进入 idle。
2. 左中指触发 HELLO 成功，随后发送 MOVE 99486 µm；起点100486 µm，即请求下降1 mm。
3. t=0：MOVE；t=3.1 ms：ACK FE（排队），没有最终成功 ACK。
4. 心跳发送/应答持续正常，CAN完整遥测约每100 ms一组。
5. t≈198/298/398/498/598/698/798 ms：设备反馈年龄210/310/410/510/610/710/810 ms。不是 CAN 遥测停止，也不是把排队 ACK 当作到位。
6. t≈489.5 ms：主机因反馈年龄超过500 ms发送 CLOSE，撤销许可。
7. t≈493.3 ms：原 MOVE 最终结果0x86 LINK_LOST；未重新发送 MOVE。
8. t≈1198 ms：固件保护位出现；t≈1298 ms：target_status=0xE6、result=11、guard=0x11。驱动 fault 始终0，位置始终100486 µm。

保护锁发生在主机取消之后，不能仅由后来的 PROFILE_VIOLATION 推断最初的取消原因。

## 待定位

源码 `StartMotion()` 在同一伺服任务中依次进行200 ms等待、多次RS485写寄存器、再200 ms等待，随后才调用 ReadMonitors；这一段与起步反馈更新中断相符，但目前没有每笔RS485时序证据，不能确定是否还有重试。

详细 GUARD.reason 仅在串口输出；CAN只给出通用保护结果和方向锁。X5 USB清单有ST-LINK，没有CH340串口。已请求用户连接CH340/debug USB，保持CAN和电源不变，先只读保存GUARD和STATUS。

不自动清锁、复位、重新SET3、修改端部/速度或放宽500 ms运行看门狗。当前BOOT_HOME=1，复位可能自动回0，且会丢失本次GUARD记录。

## 单独服务模式

`luxi-lift.service` 使用 `/run/systemd/system/luxi-lift.service.d/90-lift-only-test.conf` 临时覆盖，启动 `/tmp/luxi-lift-only-run-20260923.sh`，`shared_enable=false`。本次X/Y不控制升降，独立使能使用网页，左右中指对应下/上，松开或两侧同时按下取消本次CAN运动，B撤销升降许可。

故障后将临时启动器改为 `allow_motion=false`，只保留反馈观察。统一X停放流程尚未做实机验收；恢复统一模式前需处理双臂反馈缺失并显式清理这个临时覆盖。

## 用户随后复位

用户确认在故障后复位/重新上电。设备uptime重新计时，保护锁已清除；本次故障的板端GUARD触发记录因此无法再读回，主机CAN/ROS原始记录已保存。不要把清锁当成修复。

复位后最新连续8次反馈：当前位置39912 µm，速度−1..0 rpm，flags=0x33（位置/目标有效、锁闸输出、无运动位），fault=0，guard=0，target_status=0xE2；主机allow_motion=false且enabled=false。这不构成HOME到位验收。原BOOT_HOME=1允许设备独立于X5在启动后回中。

用户无法识别调试接口。仓库原理图区分两个Type-C：USB1属于USB_D/MCU原生USB；USB2属于USART/CH340N。需要识别实物USB2或提供板上接口照片，不能用ST-LINK USB替代串口。
