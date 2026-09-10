# luxi_core — 公共 C++ 配置与变换

include/luxi/core.hpp 实现报文校验、坐标映射、滤波和关节限幅。test_core 检查非法输入、时间与参考坐标。

构建：在仓库根目录运行 `bash tool/build.sh`。验证：`bash tool/test.sh`。产物统一在 `build/`，日志在 `log/`。

完整接口和限制见 [迁移与接口说明](../../docs/CPP迁移与接口说明.md)，运行步骤见 [仓库 README](../../README.md)。
