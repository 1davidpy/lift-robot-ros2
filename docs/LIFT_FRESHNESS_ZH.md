# 升降反馈年龄回放（2026-09-23）

CH4 实测连续约10 Hz，64个连续间隔约99.8–100.2 ms；原始电机反馈年龄113–272 ms。
250 ms 原门限在640个10 ms控制周期中拒绝298次，导致没有CAN丢帧也取消许可。
主因是设备反馈扫描年龄与10 Hz发布周期叠加；本轮不是靠放宽Quest或CAN丢组门限处理。

修正区分：
- `Sample.valid()` 保留250 ms：新位置标签、新目标规划、最终到位判定仍严格使用它。
- `Sample.healthy()` 使用500 ms电机反馈看门狗，同时仍限制首片接收年龄300 ms，
  并要求位置有效、无驱动故障/保护/bus-off；与供应方Python客户端运行期500 ms规则一致。
- 等待新鲜样本时不发送新MOVE/HOME；已受理有限目标由板端保护执行，持续会话心跳。
- HOME到位仍要求两个ACK后的250 ms内有效样本；此后锁闸/原点保持监视使用看门狗，
  不将延续状态重复计为新的训练测量。原始记录valid标志不会因看门狗门限扩大而变真。

回放修正后：640个周期、0次看门狗误拒绝，仍有298次严格测量无效。
回放同时断言CAN组超过300 ms、反馈年龄超过500 ms、驱动故障或保护必须拒绝。

```bash
c++ -std=c++17 -O0 -I Project/device_adapters/include \
  Project/device_adapters/test/lift_freshness_replay.cpp -o /tmp/lift-freshness-replay
/tmp/lift-freshness-replay Project/device_adapters/test/lift_freshness_trace.tsv
```

本回放验证软件年龄处理，不能替代实机运动/停止验收。示例为当日静止采集的匿名
时间数据，运行单位为同一主机单调纳秒，第三列为设备反馈年龄毫秒。
