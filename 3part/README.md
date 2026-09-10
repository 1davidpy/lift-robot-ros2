# 通用依赖

`artifacts.lock` 是实际下载 URL + SHA-256；`repositories.lock` 是源码上游引用。MuJoCo 固定 3.12.0 AArch64 官方发行包，仅链接原生 C API。系统依赖见根 README；最终测试记录附本机版本。

从 `tool/fetch_dependencies.sh` 获取依赖，不使用 git submodule。编译结果由源码、补丁、锁定清单与构建日志追踪，不把产物存成嵌套 Git 仓库。
