# 2026-09-24 整机现场重试：X 与 Quest 延迟

本轮在用户确认温度正常、路径清空、急停可用后进行。未刷写固件，未修改运动超时/保护门槛，未由代理发送运动指令。

## 现场经过

1. 首次 X 请求在升降运动许可尚为 false 时抵达，被统一使能前提拒绝；这是加载与操作者按键的时间重叠。
2. 通过 `/run/systemd/system/luxi-lift.service.d/90-supervised-retry.conf` 启用原有 `run_lift_shared_test.sh` 的 `allow_motion=true/shared_enable=true`；仅重启升降主机适配器，未重置 CH4、未重启双臂驱动或整机控制器。持久配置 `lift.env` 仍为 false，临时覆盖随主机重启消失。可用不等于已使能，仍需操作者长按 X。
3. 第二次 X 使能成功，依次 preparing → homing → aligning → active。实读固件 v22、范围 −97..+197 mm。其后因 Quest 年龄 203.816 ms 超过统一控制的 200 ms 门槛，进入 stopped/hold。
4. 操作者短按 X 后确认进入 returning_arms，升降原位停止；归位期间再次因 Quest 年龄 207.712 ms 中止，未移除双臂转矩。另一次使能因中指/摇杆非中立输入被拒绝。完整归位失能仍未验收。
5. 操作者确认停止发生时仍佩戴头显、发送器在前台。该轮统计时没有升降 MOVE/HOME 请求，不能把此轮停机归因为升降运动期间的 CAN 错误，也未完成升降握键运动验收。

本轮开始时升降已处于 +60.387 mm（前次只读检查为 0 mm），目标状态为本地按键取消；主机权限此前关闭。本轮无主机升降位移命令。最后观测升降 +60.387 mm、约 0 rpm、抱闸输出锁定；双臂保持转矩、底盘与升降输入许可已撤销。

## Quest 报文对比

只抓取 X5 wlan0 上目标端口 5006 的 UDP 报文；抓包没有内核丢包。

| 条件 | 时长 | 报文数 | >100 ms 间隙 | >200 ms 间隙 | 最大间隙 |
| --- | --- | --- | --- | --- | --- |
| 原 Wi-Fi power_save=on | 25 s | 1787 | 31 | 1 | 205.201 ms |
| 临时 power_save=off | 25 s | 1787 | 35 | 1 | 207.550 ms |

关闭 X5 省电没有改善本次现象，已恢复原 on，未更改 NetworkManager 持久配置。

随后 20 秒完整报文采集得到 1415 帧：Quest `t` 中位间隔 13.7 ms、最大 16.6 ms；相邻报文的主机抵达间隔最高 220.122 ms。在该 220 ms 间隙两侧，发送器时间仅相差 13.7 ms，表明报文在时间戳生成后、抵达 X5 网络层前发生排队/成批交付。不能仅凭主机抓包进一步区分 Quest 发包线程/系统、路由器或 X5 无线驱动。

X5 连接 5 GHz（5180 MHz），观测信号 −40 dBm。平均约 72 Hz 并不排除超过 200 ms 的瞬时断流。下一步建议同一路由器以太网接入 X5，保留 Wi-Fi 管理通道，读取有线地址后调整 Quest 目标，比较相同时间戳指标；正在等待用户是否可以接线。

## 证据与后续

远端：`/home/sunrise/lift_input_diagnostics_20260924/shared_retry_after_reboot/`。
关键文件：`trace.jsonl`、`retry-summary.json`、`quest-packet-summary.json`、`quest-packet-summary-powersave-off.json`、`sender-vs-host-timing.json`、`quest-timing.pcap`。

`luxi-shared-retry-watch.service` 是最多约 15 分钟的只读观察服务。退出不改变机器人许可，不替代控制程序；其脚本不包含任何运动命令。后续先消除/定位数据抵达间隙，再验证升降小动作、释放停止和 X 归位失能，保留真实 CAN 故障检查。
