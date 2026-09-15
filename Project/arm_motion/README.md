# luxi_arm_motion — 双臂逆运动学

输入 /luxi/targets 和实际 /luxi/arms/state，输出 /luxi/intent。MuJoCo C API 计算雅可比，Eigen 解约束 QP；test_ik 验证收敛与限位。

IK 每 33 ms 更新一次。独立运行时，第一次成功求解用实测关节位置初始化，后续从上一帧 IK 指令继续积分；每帧重新使用实测位置会在负载造成跟踪误差时丢失进度。目标和全部关节反馈仍须有效且新鲜，关节/任务速度与位置限制保持生效。`motion_reference` 隔离测试覆盖固定反馈下的求解进度、关节限位和反馈超时后停止输出，不连接真实执行器。

接收到 `/luxi/control_status` 后采用会话状态管理：每次进入 homing 时从配置的 home 重置 IK，只在 homing/aligning/teleop 时继续求解，disabled/hold/returning/fault 不累计控制器姿态。没有控制状态发布者时保留独立 IK 工具的求解行为。这样避免把上一会话或放下头显时积累的错误 IK 分支带入下一次对齐。

`/luxi/intent.phase` 只有在两侧 IK 手端误差均不超过 0.02 m / 0.25 rad 时标记为 `teleop`，否则标记为 `ik_unconverged`。控制管理器在 aligning 阶段等待该数学解收敛，再允许机械臂对齐。该误差衡量 IK 解与目标的差异，不代替实测关节反馈。`ik_session` 使用实机捕获的关节限位陷阱状态，覆盖未收敛标记、homing 重置和禁用后停止积分。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
