# luxi_arm_motion — 双臂逆运动学

输入 /luxi/targets 和实际 /luxi/arms/state，输出 /luxi/intent。MuJoCo C API 计算雅可比，Eigen 解约束 QP；test_ik 验证收敛与限位。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
