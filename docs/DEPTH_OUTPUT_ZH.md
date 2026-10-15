# RealSense 深度对外输出

当前设备配置为 RealSense D455f/D435i。新增 `realsense_depth` 是独立 C++ 深度节点，按序列号只打开深度传感器；保留现有 V4L2 RGB 采集节点。海康节点本次未扩展深度解析。

输出为原生深度相机坐标下的光轴 Z 距离，单位为米，**未对齐 RGB**，不能把 RGB 像素坐标直接用于深度查询。光学坐标轴为右、下、前。未增加点云、RGB/深度同步或机器人基座外参。

SDK 从每帧读取真实 `depth_units`，将 Z16 转为 `32FC1`，原始 0 值转为 `NaN`。不使用伪彩色或 JPEG 表示测距数据，不缩放深度分辨率。默认采集 640×480、15 Hz；三个相机需各自运行一个节点，并现场验证 USB 带宽和供电稳定性。

## 构建

Ubuntu 22.04 ARM64，已有 ROS 2 Humble 和仓库原有依赖，还需 `libusb-1.0-0-dev`。

```bash
bash tool/build_depth_sdk.sh --prebuilt
bash tool/build.sh --packages-select luxi_device_adapters luxi_web_ctrl
```

`--prebuilt` 使用官方 Ubuntu 22.04 ARM64 v2.56.5 运行库和开发包，校验 SHA-256 后解压到忽略的 `build/depth-sdk/install`，不安装系统包、不修改内核。需设备已有兼容运行库（包括 libstdc++6 ≥ 12），脚本用 `ldd` 检查缺失依赖。预下载包也可传入：

```bash
bash tool/build_depth_sdk.sh --prebuilt /path/librealsense2-runtime.deb /path/librealsense2-dev.deb
```

也可从源码构建同版本 SDK（首次在板端编译较慢），默认或指定预下载源码包：

```bash
bash tool/build_depth_sdk.sh /path/librealsense-v2.56.5.tar.gz /path/json-v3.11.3.tar.gz
```

源码方式固定 JSON v3.11.3，两份源码包均校验 SHA-256，使用原生 V4L2 后端。

没有 SDK 时仍可编译并测试 mock；真实采集会明确报告缺少 SDK，不能输出模拟值冒充实测。

SDK 放在独立后端动态库中，在 ROS 初始化后以局部符号加载，并关闭 SDK 自身的网络 DDS 功能。主节点不直接链接 SDK，避免官方预编译包内置的 Fast DDS 与 ROS Humble 发生符号/ABI 冲突。

## 运行

已有部署的 RGB、网页、机械臂和底盘服务不由这些脚本重启。确认现有程序没有录制后，启动一个头部深度进程：

```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 CAMERA_SERIAL=261043062235 \
 bash Project/luxi_web_ctrl/scripts/run_depth_camera.sh head_rgb
```

其余配置：左腕 `left_wrist_rgb` / `254843065015`，右腕 `right_wrist_rgb` / `261443062039`。先核对设备序列号；同一序列号不可被多个深度进程同时打开。`DEPTH_WIDTH`、`DEPTH_HEIGHT`、`DEPTH_FPS` 只接受设备实际支持的流配置。

`serial` 同时接受 SDK 主序列号或 ASIC/USB 序列号。当前 RGB 配置使用 ASIC/USB 序列号，与 SDK 主序列号不同；后端明确匹配两种标识，多设备匹配时拒绝采集，避免左右相机串线。

可选安装独立 systemd 服务，安装本身不启动、不启用开机运行：

```bash
bash Project/luxi_web_ctrl/scripts/install_depth_camera.sh head_rgb 261043062235
sudo systemctl start luxi-depth-head_rgb.service
# 验证稳定后按需设置开机启动：
sudo systemctl enable luxi-depth-head_rgb.service
```

网页需要加载此次构建的新 `web_ctrl` 才有 HTTP 路由；结束录制并撤销运动许可后安排重启 `luxi-web.service`。该服务与 Quest 有联动，重启前必须确认现场状态。

## HTTP（局域网设备首选）

新网页服务沿用现有监听地址/端口，例如 `http://192.168.0.245:8000`。`id` 取 `head_rgb`、`left_wrist_rgb`、`right_wrist_rgb`。

| 请求 | 返回 |
| --- | --- |
| `GET /api/cameras/{id}/depth` | 行优先、小端 float32 原始数组，单位米；`application/octet-stream` |
| `GET /api/cameras/{id}/depth/info` | JSON：尺寸、时间、帧号、有效性约定、是否 mock、内参 |
| `GET /api/cameras/{id}/depth/point?x=320&y=240` | JSON：指定**深度图**像素的 `depth_m`，无效点 `valid=false, depth_m=null` |
| `GET /api/state` | 原相机字段下新增 `depth.fresh`、尺寸、帧号、单位和 URL |

缺少帧、断线或超过 1 秒未更新均返回 HTTP 503，不回传旧图。坐标缺失、重复、越界或非整数返回 400。每次响应使用同一帧快照；不同 HTTP 请求可能属于不同帧，应对比 `sequence`/`X-Frame-Id`。

原始响应头包含 `X-Width`、`X-Height`、`X-Depth-Encoding: 32FC1`、`X-Depth-Unit: metre`、`X-Byte-Order: little-endian`、`X-Mock`、`X-Frame-Id`、`X-Optical-Frame`、ROS 时间戳和接收时间。

内参只在 `CameraInfo` 与深度帧的时间戳、坐标系和尺寸匹配时暴露。`calibration_available=false` 时不要进行三维反投影。设备若报告 ROS 无法直接表示的畸变模型，不伪装为 `plumb_bob`。`device_timestamp_ns` 是 SDK 原时间域数值转换成纳秒，应按 `timestamp_quality` 后缀区分 Hardware Clock、System Time 或 Global Time，不能无条件当作 Unix 时间或硬件同步依据；ROS 图像 stamp 使用主机接收时刻。`receive_mono_ns` 仅可在板端单调时钟域内比较。相机间不保证同步。

其他设备的 Python 示例（依赖 requests/numpy）：

```python
import requests
import numpy as np

url = 'http://192.168.0.245:8000/api/cameras/head_rgb/depth'
r = requests.get(url, timeout=3)
r.raise_for_status()
width, height = int(r.headers['X-Width']), int(r.headers['X-Height'])
depth_m = np.frombuffer(r.content, dtype='<f4').reshape(height, width)
print('mock:', r.headers['X-Mock'], 'Z(m):', depth_m[240, 320])
valid = np.isfinite(depth_m) & (depth_m > 0)
```

```bash
curl --fail 'http://192.168.0.245:8000/api/cameras/head_rgb/depth/info'
curl --fail 'http://192.168.0.245:8000/api/cameras/head_rgb/depth/point?x=320&y=240'
```

## ROS 2

| 话题 | 类型 / 含义 |
| --- | --- |
| `/camera/{id}/depth/image_raw` | `sensor_msgs/msg/Image`，32FC1，米，NaN 无效 |
| `/camera/{id}/depth/camera_info` | `sensor_msgs/msg/CameraInfo`，与图像相同 stamp/frame_id |
| `/luxi/cameras/{id}/depth` | 原有 `CameraFrame` 类型的内部网关传输，encoding=32FC1 |
| `/luxi/cameras/{id}/depth/status` | JSON 驱动状态、mock 标记、错误原因 |

图像和内参使用 SensorDataQoS（best effort / volatile）。当前板端服务设置 `ROS_LOCALHOST_ONLY=1`；局域网调用可直接使用 HTTP。需要跨设备 ROS 时，发布端和订阅端均须使用一致的 `ROS_DOMAIN_ID` 并配置 `ROS_LOCALHOST_ONLY=0` 及 DDS 网络。只改变订阅端无法访问板端话题。

本次没有把深度写入既有 episode/训练数据格式；录制 schema 保持原定义。

## 验证

`depth_output` 测试运行真正的深度可执行程序（mock=true），检查单位换算、NaN、标准图像、内参匹配、帧号及退出。`web_gateway` 测试检查原始字节、内参、测距、非法坐标、断流过期、恢复和损坏帧拒绝；测试使用隔离 ROS 域，不连接执行器。

SDK API 依据：[RealSense 官方 API How To](https://github.com/realsenseai/librealsense/wiki/API-How-To)，[固定版本 SDK 源码](https://github.com/realsenseai/librealsense/tree/v2.56.5)。

### 2026-10-15 实机验证

在本仓库完成两个包的构建；隔离域测试通过标准 ROS 图像输出、单位换算、NaN、内参配对和 HTTP 接口。修复官方 SDK 内置 DDS 与 Humble 的符号冲突后，实测以独立后端加载。

头部、左腕、右腕三台相机**分别**采集 640×480、15 Hz 原生深度，都取得真实非零深度像素和可用内参；未进行三路深度同时长期运行或测距精度验收。USB/ASIC 与 SDK 主序列号实测对应如下：

| 相机 ID | USB/ASIC 序列号 | SDK 主序列号 |
| --- | --- | --- |
| head_rgb | 261043062235 | 260922304036 |
| left_wrist_rgb | 254843065015 | 244622073011 |
| right_wrist_rgb | 261443062039 | 262422075883 |

在独立 ROS 域 85 / 临时 HTTP 18021 端口，从 Windows 设备通过局域网验证了内参、像素深度和整幅 float32 二进制响应（640×480×4 = 1,228,800 字节）。真实无效像素查询返回 `valid=false, depth_m=null`。测试进程已结束，现有控制程序和三路 RGB 继续运行。当前实际控制台来自另一份工作目录，端口为 8001，本次没有切换该控制台或安装开机服务；应用新接口需使用本仓库的新构建。
