# 统一使能（可选）

管理程序传入 `shared_enable=true`：长按 X 准备全部设备，双臂完成对齐后开放底盘/升降；短按 X 先停车并保持双臂，升降 HOME 到保存 0 mm，再返回已确认有支撑的双臂 home 并撤销转矩。B 和 `disable/halt` 只停止保持，不回位、不移除转矩。

UART 升降的位置有效位采用连续 5 组完整反馈确认；1–4 次无效允许既有遥操继续，有效反馈清零计数。确认无效后，仅暂停升降，双臂和底盘继续；升降恢复有效、停稳并收到松开握键后重新读取 INFO/READY，后续新按握键才产生运动。控制器独立验证隔离状态的反馈年龄、故障和运动禁止位；通信中断、电机故障、B 停止等仍作用于整机。状态 `lift_position_paused` 表示正在隔离位置异常；参数 `isolate_lift_position_invalid=false` 可撤销控制器侧隔离。

实现要求新鲜反馈和带 epoch 的协调心跳；错误后须新操作，不自动重放。`supported_home_confirmed` 默认 false，管理启动依据用户确认设为 true。完整顺序与交付限制见 `docs/SHARED_X_CONTROL_ZH.md`。本轮未编译/测试。

以下为 `shared_enable=false` 的原独立模式说明。

# luxi_control_manager — 控制阶段管理

管理 disabled/homing/aligning/teleop/hold/returning/fault；输出 /luxi/command，通过 /luxi/control 服务启停。反馈和输入超时禁止自动继续。

控制周期为 10 ms；aligning 仍要求对应食指扳机大于 0.92，并限制为 0.2 rad/s。普通 STALE (1) 沿用旧版兼容行为。INVALID (2) 立即冻结当前指令；在既有 150 ms 窗口内恢复跟踪且收到恢复后的 IK 结果才继续。持续跟踪丢失、Quest 数据超时或 IK 超时进入锁存 hold，不自动恢复。`tracking_gap` 隔离测试覆盖短暂中断、恢复后等待新 IK、持续跟踪无效和丢包。

Quest 数据或 IK 结果超过 150 ms 时先冻结目标，`input_hold_timeout`（秒，默认 `0.5`，允许 `[0.15, 1.0]`）决定何时锁存 hold。短暂网络/IK 中断可以在该窗口内恢复，但必须等到新鲜数据；网络恢复还须等待对应新 IK，不能继续使用中断前缓存目标。该参数不放宽电机反馈检查或 INVALID 跟踪的 150 ms 上限。实机网络抓包出现 161–212 ms 的间隔，因此把短暂冻结和锁存故障分开处理。

`/luxi/control_status` 增加 `reason`、`input_paused`、`quest_age_ms`、`intent_age_ms` 和 `input_hold_timeout`，区分 `tracking_gap`、`quest_gap`、`ik_gap`、`waiting_for_recovered_ik`、`tracking_invalid`、`quest_timeout`、`ik_timeout` 和反馈故障；hold 会保留原因。

aligning 阶段收到 `intent.phase=ik_unconverged` 时保持当前 home 指令，状态原因是 `waiting_for_ik_solution`。只有 IK 手端解收敛后才接受扳机控制的对齐运动；仅有数值有限、关节限位内的解不足以允许对齐。`tracking_gap` 中的对齐测试验证未收敛时目标保持不动，收敛后才开始受速度限制的对齐。

`home_tolerance` 是启动参数，单位为弧度，默认 `0.05`，允许范围为 `(0, 0.1]`。它用于 homing 和 returning 阶段的非夹爪关节到位判定；不会改变目标位姿、电机增益、速度限制、反馈有效性检查或 30 秒超时。带负载的实机可能存在稳态位置误差，应先确认无障碍物并对比下发目标与实测反馈，再选择容差。`/luxi/control_status` 发布实际使用的容差。

隔离测试 `homing_tolerance` 覆盖实测约 `0.09 rad` 偏差、回位后禁用、默认容差保持不变、超出容差仍阻止到位以及无效参数拒绝；测试不会连接真实执行器。

`home_joint_speed_limit`（rad/s，默认 `0`）单独限制 homing 和 returning 的非夹爪关节速度；`0` 使用配置文件限速，正值取本参数与配置限速的较小值。它不能提高配置限速，也不改变 aligning 的 `0.2 rad/s` 上限。参数必须有限且非负。状态同时发布该参数和 `joint_speed_limits`，便于确认进程实际加载的速度。托管程序使用 `arms-teleop.json`：跟随 `1.0 rad/s`、夹爪 `0.5 rad/s`、home/return `0.2 rad/s`。`phase_speeds` 隔离测试覆盖 0.5 rad 跟随阶跃、慢速归位/对齐/返回及返回后禁用。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
