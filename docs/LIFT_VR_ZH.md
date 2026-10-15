> 2026-09-24：握键修复已编译、9 项虚拟回归通过，并以禁止运动方式加载。
> 位置无效/SENSOR_ERROR 仍待解决；当前状态详见 [LIFT_HOLD_FIX_20260924_ZH.md](LIFT_HOLD_FIX_20260924_ZH.md)。下文为最初接入说明。

> 后续更新：管理模式现已在源码中接入 X 统一使能/顺序停放，详见
> [SHARED_X_CONTROL_ZH.md](SHARED_X_CONTROL_ZH.md)。下文独立网页使能、X/Y 和 B 映射
> 属于此前独立模式；统一模式以新文档为准。新版本尚未编译/加载。

# Quest 升降接入（2026-09-23）

本次交付是源码修改，尚未编译、安装运行或进行 CAN/运动验收。用户已确认升降
连接 USB-CAN CH4，对应 X5 的 `luxi_lift`。新服务单独启动，默认无运动许可。
现有双臂、底盘、相机服务和板端标定保持不变。

## 映射

| 操作 | 行为 |
| --- | --- |
| 网页“启用 Quest 升降控制” | 两握键释放、Quest/升降反馈有效且平台锁闸停稳时，读取配置并开启许可；不主动运动 |
| 左中指握键 | 按住下降；一次有限目标指向当前允许下限的内缩边界，松手提前停止 |
| 右中指握键 | 按住上升；一次有限目标指向当前允许上限的内缩边界，松手提前停止 |
| 松手 / 两中指同时按下 | CLOSE 撤销本次 CAN 持有的运动，保留许可；不会恢复旧目标 |
| 换向 | 必须两握键松开、停止确认后再按另一侧；不直接反向 |
| B | 停止本次 CAN 运动并撤销升降许可，须网页重新启用 |
| 网页“停用升降控制” | 关闭自身 CAN 会话并撤销许可，不全局打断本地按键或上电回中 |
| 网页“升降全局停止” | 显式 STOP，会请求停止包括本地控制在内的运动；ACK 不等于已停稳 |
| X / Y / 食指扳机 | 保留原来的双臂 / 底盘 / 夹爪映射 |

握键按下阈值 0.65，释放阈值 0.35。固定姿态 STALE=1 可继续使用新鲜按键；
INVALID=2、Quest 超过 200 ms 未更新、失败菜单、CAN/设备异常会撤销许可。
连接恢复后必须重新启用并释放两握键，不会重放动作。

固件是有限绝对位置目标接口，不支持连续速度/高频 MOVE。握键按住期间不再
重复请求 1 mm 小段；使用设备已有速度分区，固件内部区段交接仍可能停顿。
新动作开始前观察至少 1200 ms 停稳反馈。未发送 SPEED；独立握键模式没有写入
SET_CENTER、HOME、SPEED、LIMIT、VERIFY、BOOT_HOME 或任何 Flash 配置。

## 协议与数据

- 经典 11 位 CAN，500 kbit/s，节点 1；命令 0x601，ACK 0x581，遥测
  0x181/281/381/481。独立 SocketCAN 所有者锁，不共享 R680 的过滤器或 1M 波特率。
- 按同一 sample sequence 组齐四页，不补旧页；保留各页主机单调时间、内核接收
  墙钟、设备 uptime、反馈年龄、全部 flags/guard 和原始报文。重复组去除，缺组计数，
  冲突页/设备时间倒退撤销许可。支持正常 16 位序号及 32 位 uptime 回绕。
- 完整组首片年龄最多 300 ms，电机反馈年龄加组帧/等待时间最多 250 ms；这不是
  硬件同步。遥测真实频率约 10 Hz，不将相同快照当成新测量。
- 新随机 nonce 建立会话；每 100 ms 独立新序号心跳，300 ms ACK 超时撤销许可；
  MCU 租期仍为 500 ms。关闭失败时停止续租，不承诺机械 500 ms 内停稳。
- 普通请求同一 8 字节最多发送 3 次，重试只在首发后 400 ms 内；FE 仅排队，
  最终 ACK=0 才进入等待到位。超时不换序号重放 MOVE，不自动恢复运动。
- 使用 INFO 0..9 读协议、诊断版本、上下限、配对、速度及保存标志；协议须 v1、
  诊断版本至少 v20、stage2。运行目标与设备允许边界保持默认 5 mm 内缩；超界
  拒绝继续，不静默裁剪。内缩不代替机械端部验收，+197 mm 仍是文档中的估计值。
- 到位须最终 ACK 后两个新完整样本：目标一致、误差≤10 µm、转速≤2 rpm、
  坐标/目标有效、状态3、未运动且锁闸输出、无故障/保护、会话有效。

ROS 接口：`/luxi/lift/control`（start/disable/stop）、`/luxi/lift/state`、
`/luxi/lift/status`、`/luxi/lift/events`、`/luxi/lift/can_rx`。节点单线程通信，
不阻塞等待 ACK 或占用 Quest 接收器。`events` 包含配置、请求、ACK 和取消原因。

录制器源码已追加以上四个消息话题到原始 ROS bag。已有 LeRobot v2.1 转换器
仍导出 19 维双臂/底盘，不会自动增加升降一维；本次没有改变训练数据 schema。
后续增加升降训练标签时须定义有限目标动作语义及 10 Hz 数据有效性，不能直接
把电机 rpm 写成直线关节速度。

## 构建和安装（本次未执行）

在 X5 仓库内，明确安排编译后执行：

```bash
cd ~/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2
bash tool/build.sh --packages-select luxi_device_adapters luxi_data_recorder luxi_web_ctrl
bash Project/luxi_web_ctrl/scripts/install_lift.sh
```

安装脚本只新增 `luxi-lift.service` 并更新原有服务管理白名单，不启动或开机启用。
在录制结束、双臂/底盘禁用后，重启网页及控制录制程序以加载新版本：

```bash
sudo systemctl restart luxi-web.service
sudo systemctl restart luxi-program.service
sudo systemctl start luxi-lift.service
```

最后一条仅启动升降反馈/VR 服务并配置 CH4，需网页启用后才响应握键。只读阶段
可先在 `Project/robot_bringup/config/lift.env` 设置 `LIFT_ALLOW_MOTION=false`。
升降准备脚本只调整已命名的 CH4，不调用会重置所有通道的 `kh_can_setup --apply`。
若还没有 `luxi_lift`，先解决适配器枚举，不猜 canN 通道。

配置：`lift.env` 的节点、边界内缩。默认端口 luxi_lift；改变物理
通道时须同时修改专用准备脚本，并确认不会覆盖双臂或底盘。

## 待执行验收

按用户“不自动编译/验证”的要求，本次没有执行构建、测试、CAN 请求或实机运动。
需要先完成只读遥测/INFO、握手心跳，再由现场人员验证短按小行程、持续按住、松手、
双键、B、方向切换、Quest 失联及 CAN 断线停止。尚不能称为可直接运行的实机验收结果。
原板 BOOT_HOME=1，设备上电可能独立回中；此驱动不改变该策略。

固件对照源码：[lifting_platform_servo](https://gitee.com/fxx360724/stm32-f103-c8-t6_-chip/tree/master/Projects/lifting_platform_servo)，
读取提交 `6a44e1bcf83f3105d4105a1f05cc129bcb80f27c`。手册与主机代码均位于该项目；
本适配器只实现运行期所需子集，不暴露标定命令。
