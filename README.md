# LUXI Lift Robot / ROS 2 C++

RealSense 原生及 RGB 对齐深度图、ROS 2 话题和局域网 HTTP 测距接口见 [深度输出说明](docs/DEPTH_OUTPUT_ZH.md)。

接收方获取关节、底盘、升降、RGB、深度及调用控制服务见 [接收端数据与控制接入手册](docs/RECEIVER_INTEGRATION_ZH.md)。本次设备三路对齐相机及网关已部署，地址为 `http://192.168.0.245:18030`。RGB 与对齐深度支持 `/api/cameras/{id}/rgbd` 单次成对读取，接收示例为 `Project/luxi_web_ctrl/scripts/receive_rgbd.py`；重新部署时须先停止同一相机的旧 RGB 服务。

当前实现 **Quest 3 → 双 OpenArm / R680 三轮全向底盘 → 三路 RGB → 本地录制 → 局域网回传**。底盘为可选模块，默认 `base_enabled:=false`。
业务节点、IK、硬件适配、录制、HTTP 服务和接收工具使用 C++17；启动用 XML，构建/依赖工具用 Shell。没有 Python 业务节点、训练器或模型推理服务。

上电、Quest 操作、双臂/底盘使能与一键启动见 [上电启动与 Quest 操作指南](docs/上电启动与Quest操作指南.md)。

局域网网页、设备状态与开机自启见 [Web 控制台](Project/luxi_web_ctrl/README.md)。

## 目录

- `Project/`：自研 ROS 2 包、配置及 C++ 测试。
- `device/`：厂商仓库锁定清单、必要补丁、忽略的外部 SDK 目录。
- `3part/`：固定 URL/版本/SHA-256 的依赖清单。
- `demo/`：外部参考仓库链接和确切提交。原有 lunar_-slam 不参与构建。
- `tool/`：依赖获取、构建、测试、启动与 SDK 安装工具。
- `docs/`：用户给定框架及本次实现、接口、验证说明。
- `build/`：编译、安装、下载缓存及软件集成测试产生的数据。
- `log/`：构建、测试、ROS 节点日志。实际采集默认写忽略的 `data/`。

核心源码直接由当前 Git 仓库管理；外部目录不作为 gitlink/submodule 提交。依赖地址与确切版本可追踪，编译产物用构建脚本复现，不提交二进制。

## 构建与软件测试

目标环境：Ubuntu 22.04 / ARM64 / ROS 2 Humble。第一次准备系统依赖：

```bash
sudo apt-get install python3-colcon-common-extensions libeigen3-dev nlohmann-json3-dev   libopencv-dev libcpp-httplib-dev libssl-dev libsqlite3-dev   ros-humble-rosbag2-cpp ros-humble-rosbag2-storage-default-plugins   ros-humble-ament-cmake-gtest ros-humble-launch-xml
bash tool/fetch_dependencies.sh
bash tool/build.sh
bash tool/test.sh
```

`colcon` 的 Python 是 ROS 构建工具，不是业务运行代码。MuJoCo 3.12.0 使用原生 C API；OpenArm CAN 1.3.4 从固定源码编译。所有 install 文件位于 `build/install`。

`tool/test.sh` 包含真实 ROS 2 进程间集成测试，但执行器和相机均使用显式 mock；测试域为 73，端口为 15006/18000，不配置 CAN、不使能实体电机。

## 运行 mock 系统

```bash
bash tool/run.sh mock:=true minimum_free_gib:=0.2
```

浏览器打开 `http://127.0.0.1:8000`。Quest 应用发向当前机器 UDP 5006；没有真实 Quest 报文时不会使能，即使驱动是 mock。另可运行 `bash tool/integration.sh` 自动执行合成 Quest 的完整软件测试。

## 实机接入

先确认 `Project/robot_bringup/config/arms.json` 与实际机械臂一致。默认保留旧部署 right=can1、left=can2 及同一零点、增益、关节限位；没有按外观重新分配物理 CAN。不要与原 Dora 控制器同时运行。

使用新提供的鲲弘四路 USB CAN 时，按 [KH 模块适配说明](device/KH-UCANFD-B2B-X4/README.md) 构建驱动和映射通道，改用 `arms-kh.json`（CH1 左臂、CH2 右臂）；不要直接沿用旧 can1/can2 映射。

```bash
# 仅示例，必须已确认 CAN 配置；本脚本不配置或复位 CAN。
bash tool/run.sh mock:=false allow_hardware:=true no_cameras:=true require_cameras:=false
```

启动时只读取设备，不自动使能。Quest 长按 X 或网页使能会触发起始运动；暂停先返回起始姿态，随后失能。追踪 INVALID、真实 UDP 超时进入保持，重连不自动继续，需明确暂停/重新使能。这与旧版无自动追踪门禁有差异，见迁移说明。

海康需要与现有 ctypes ABI 对应的 **Linux AArch64 MV3D RGB-D SDK**：

```bash
bash tool/install_hikrobot_sdk.sh /path/to/extracted/sdk/lib
export LD_LIBRARY_PATH="$PWD/device/sdk/hikrobot:${LD_LIBRARY_PATH:-}"
bash tool/run.sh mock:=false allow_hardware:=true   "sdk_library:=$PWD/device/sdk/hikrobot/libMv3dRgbd.so"
```

通用 MVS SDK 不能替代 `MV3D_RGBD_*`。当前库包装保留设备时间戳与帧号；设备时间未标定到主机时钟时明确报告 `host_receive_device_unmapped`，不宣称曝光同步。配置中的头部视角默认引用原顶部相机序列号，实际搬装后需要标定。

## 录制与局域网接收

网页提供开始、成功、失败、取消；取消也提交并保留原始数据，只是不标为接受。

```bash
# LAN 只读输出；ROS 数据同样可由同域 ROS 2 客户端订阅。
bash tool/run.sh http_host:=0.0.0.0
# 远程网页运动/录制操作无需密码；仍需设备就绪和显式使能，网页按钮不是硬件急停。
```

接收机装好同版本工具后：

```bash
source build/install/setup.bash
ros2 run luxi_data_recorder episode_pull http://ROBOT_IP:8000 episode-编号 /path/to/datasets
```

工具从 `.partial-episode-*` 续传，逐文件验证 SHA-256，全部成功才提交最终目录。不会删除板端数据。已完成的相同数据重复接收是幂等的。

本阶段数据是已有双臂真实 **16 维**（右 8 + 左 8），不填充尚未接入的底盘/Lift 假数据。以后增加反馈后再版本化扩展 21 维接口，不包含训练。

详见 [迁移与接口说明](docs/CPP迁移与接口说明.md)、[验证记录](docs/编译与测试记录.md)。

R680 接入与左右摇杆分配见 [R680 驱动说明](device/R680/README.md)。启用模拟底盘：`./tool/run.sh mock:=true base_enabled:=true`。底盘数据写独立 ROS 话题，不改变原双臂 16 维样本含义。

网页现支持各设备采集开关、重启重连，以及控制/录制程序独立启动。登录方式、开机自启和安装说明见 [网页设备管理](docs/网页设备管理.md)。开启采集或程序不会自动使能电机。
