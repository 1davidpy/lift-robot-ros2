# RealSense RGB 与对齐深度对外输出

更新日期 2026 年 10 月 15 日。当前 SDK 为 librealsense 2.56.5，不修改 SDK 上游源码；仓库适配层通过 `rs2::align(RS2_STREAM_COLOR)` 完成深度到 RGB 的对齐。海康节点未扩展此功能。

## 输出约定

`realsense_depth` 默认 `align_to_color=true`，一个进程同时拥有同一物理相机的 RGB 与深度传感器，SDK 获取 frameset 后执行对齐。RGB 为 JPEG，对齐深度为与该 RGB 相同尺寸、同一像素网格的 `32FC1` 小端浮点数组，单位米。SDK 原始 Z16 的 0 转为 NaN；遮挡、超出共同视野和无法测距处仍可能没有有效深度。

同组 RGB、深度和内参共享 ROS 主机接收时间戳与 `{id}_color_optical` 图像坐标标识，并携带会话内递增的 `sequence`。`pair_id` 为机器人单调时钟生成的进程会话标识加序号，作为不透明字符串使用；节点重启后重新生成，接收端按相机 ID 与 pair_id 去重。三台相机之间不保证同步。

**像素对齐不等于硬件时间同步。** 配对来自 SDK frameset，输出 `pairing=sdk_frameset`、`hardware_synchronized=false`，分别保留原始深度/RGB 设备帧号、设备时间戳和时钟域。两路原始时间域一致时才能计算其时间差；运动场景仍需评估时间偏差。ROS stamp 是主机收到并处理该组数据的时刻，不是相机曝光时间。

深度数值沿用 SDK align 的语义：把源深度相机测得的 Z 重采样到 RGB 像素网格，标记 `source_depth_z_resampled_to_color`，不是欧氏距离。该网格用于 RGB 像素测距。若需要严格的 RGB 光学坐标三维点、机器人基座坐标或测量精度保证，应使用原始深度、原始内参、深度到 RGB 外参进行三维变换并现场验证；当前对外接口没有完整提供这条原始标定链。

`align_to_color=false` 保留旧的原生深度模式，只打开深度传感器，可与独立 V4L2 RGB 共存；该模式没有成对 RGBD 输出。无论分辨率是否相同，都不能将其 RGB 坐标直接当作深度坐标。

## 构建

Ubuntu 22.04 ARM64、ROS 2 Humble，使用仓库原有 OpenCV 等依赖及 `libusb-1.0-0-dev`：

```bash
bash tool/build_depth_sdk.sh --prebuilt
bash tool/build.sh --packages-select luxi_interfaces luxi_device_adapters luxi_web_ctrl
```

新增 `luxi_interfaces/msg/RGBDFrame`，因此必须重建接口包。跨机 ROS 接收端也要更新该包并重新 source；仅使用 HTTP 的设备不需要 ROS 或相机 SDK。

`--prebuilt` 下载官方 Ubuntu 22.04 ARM64 v2.56.5 包，校验 SHA-256 后解压到忽略的 `build/depth-sdk/install`，不安装系统包或修改内核。也可传入预下载包，或使用脚本原有源码构建方式。SDK 在独立动态后端中加载，关闭其自带 DDS，避免与 ROS Humble 的 DDS 符号冲突。后端入口已升为 v3，主程序与后端必须一起构建和更新。

## 启动与相机所有权

默认两路都请求 640×480、15 Hz。`DEPTH_WIDTH`、`DEPTH_HEIGHT`、`DEPTH_FPS` 在对齐模式下同时用于 RGB 与原生深度；必须是两路共同支持的 profile。`JPEG_QUALITY` 默认 90。节点不自动切换到其他相机、不硬件 reset、不在失败时伪造数据。

多相机适配层使用同用户的进程锁串行执行 SDK 枚举与初始化，在取得第一组数据后释放锁，后续采集仍并行。锁文件为 `/tmp/luxi-realsense-startup-UID.lock`，不应在相机进程运行时删除。等待初始化超过 15 秒会明确报错并由节点按原有周期重试。首次部署应逐台启动并确认出帧，再验证同时采集。

| 相机 ID | ASIC 或 USB 序列号 | SDK 主序列号 |
| --- | --- | --- |
| head_rgb | 261043062235 | 260922304036 |
| left_wrist_rgb | 254843065015 | 244622073011 |
| right_wrist_rgb | 261443062039 | 262422075883 |

两种序列号均可匹配，存在歧义时拒绝启动。每台相机只运行一个对齐节点；**启动前结束录制，停止该相机原来的 V4L2 RGB 服务及旧深度进程**。否则相机 RGB 设备被占用，SDK 采集失败；不要重复启动整机控制栈。

本次设备已切换到三路对齐常驻服务及 18030 网关，状态见文末。以下命令用于重新部署或手动调试；已有对应对齐服务运行时，不要重复启动。手动运行头部对齐采集前还需停止 `luxi-depth-head_rgb.service`：

```bash
sudo systemctl stop luxi-rgb-head_rgb.service
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 CAMERA_SERIAL=261043062235 \
  ALIGN_TO_COLOR=true bash Project/luxi_web_ctrl/scripts/run_depth_camera.sh head_rgb
```

节点继续发布原有 RGB 内部话题和 JPEG 话题，采集画面切换为新的 640×480、15 Hz；依赖旧 640×360、30 Hz 的应用需要适配。结束手动节点后，可恢复旧服务：

```bash
sudo systemctl start luxi-rgb-head_rgb.service
```

原生深度模式不占用 RGB：

```bash
CAMERA_SERIAL=261043062235 ALIGN_TO_COLOR=false \
  bash Project/luxi_web_ctrl/scripts/run_depth_camera.sh head_rgb
```

可安装独立 systemd 服务。安装本身不启动、不启用开机运行：

```bash
ALIGN_TO_COLOR=true bash Project/luxi_web_ctrl/scripts/install_depth_camera.sh head_rgb 261043062235
sudo systemctl start luxi-depth-head_rgb.service
```

对齐服务与 `luxi-rgb-head_rgb.service` 和旧整机 `luxi-robot.service` 互斥：启动时会停止对应旧 RGB 服务，反向启动旧 RGB 服务也会停止对齐服务。旧的原生深度 unit 需要重新运行安装脚本才获得该约定。准备回退时先停止对齐服务，再启动旧 RGB 服务。确认画面、尺寸和持续稳定性后，由提供方安排开机启用；相互冲突的两种相机服务不要同时启用。

安装脚本同时更新白名单设备管理助手。某相机的对齐 unit 明确配置 `ALIGN_TO_COLOR=true` 且处于启用或活动状态时，`/api/devices` 及该相机的启停操作改为管理对齐服务；原生深度 unit 不接管 RGB 管理。回退时禁用对齐 unit，再启用原 RGB unit，设备入口恢复原目标。

需要常驻时，逐台安装完成后停用旧 RGB 开机项，启用对齐服务，再安装独立网关。例如头部：

```bash
sudo systemctl disable luxi-rgb-head_rgb.service
sudo systemctl enable --now luxi-depth-head_rgb.service
bash Project/luxi_web_ctrl/scripts/install_rgbd_gateway.sh 18030
sudo systemctl enable --now luxi-rgbd-gateway.service
```

左腕和右腕采用对应 ID。网关 installer 只安装、不自动启动；网关 service 只启动 HTTP 节点，不启动执行器。默认使用 ROS 域 0、本地 DDS、HTTP 对外监听 18030，文件与接收队列目录为 `data/interface_gateway`。原控制台可以继续运行，两者读取相同机器人状态。

需要恢复旧 RGB 部署时，结束录制后逐台执行以下操作，将 ID 替换为对应相机。旧 RGB 不再提供成对深度，独立网关的 rgbd 接口会在 1 秒后返回 503：

```bash
sudo systemctl disable --now luxi-depth-head_rgb.service
sudo systemctl enable --now luxi-rgb-head_rgb.service
# 如不再需要独立网关：
sudo systemctl disable --now luxi-rgbd-gateway.service
```

## HTTP 成对读取

本次已部署的新版网关地址为 `http://192.168.0.245:18030`，与三路相机节点使用 ROS 域 0。须使用本仓库重新构建的 `web_ctrl`；其他工作目录的旧网关不会自动更新。重新部署步骤见 [接收端手册](RECEIVER_INTEGRATION_ZH.md) 第 12 节。

| 路径 | 返回 |
| --- | --- |
| `GET /api/cameras/{id}/rgbd` | 同一不可变快照中的 metadata、RGB JPEG、对齐深度二进制 |
| `GET /api/cameras/{id}/rgbd/info` | 最新成对数据的 JSON 元数据与可用内参，不带像素 |
| `GET /api/cameras/{id}` | 兼容旧 RGB JPEG 快照，不保证与另一次深度请求配对 |
| `GET /api/cameras/{id}/depth` | 当前模式的深度数组；响应头标明是否对齐 |
| `GET /api/cameras/{id}/depth/info` | 当前深度信息，`aligned_to_color` 区分模式 |
| `GET /api/cameras/{id}/depth/point?x=320&y=240` | 当前深度图像素值；仅对齐模式可使用对应 RGB 像素 |
| `GET /api/state` | `cameras[id].rgbd.fresh`、pair_id、URL，以及 depth 的对齐标志 |

成对接口没有数据、仅运行原生深度节点或超过 1 秒未更新时返回 503；旧网关无此接口时返回 404。不会将独立采集的 RGB 和深度临时拼成一组。`/rgbd/info` 和下一次 `/rgbd` 请求可能属于不同组，**解码应使用 `/rgbd` 响应内的 metadata**。

响应类型为 `multipart/mixed; boundary=...`，依次包含三部分，二进制内容未经 base64 编码：

| Content-Disposition 的 name | Content-Type | 内容 |
| --- | --- | --- |
| metadata | application/json | 本组元数据 |
| color | image/jpeg | JPEG，解码尺寸等于 metadata 的 width、height |
| depth | application/octet-stream | 行优先、小端 float32，字节数 width×height×4 |

响应头有 `X-Pair-Id`、`X-Aligned-To-Color: true`、`Cache-Control: no-store`。客户端使用标准 MIME 解析器，不要用文本换行或固定字符串拆分二进制。

元数据 `schema=luxi_rgbd_v1`，主要字段为：

| 字段 | 含义 |
| --- | --- |
| camera_id、pair_id、sequence | 相机、配对标识、会话内组序号；后两者为字符串 |
| aligned_to_color | 固定 true；成对接口只接收对齐消息 |
| frame_id、width、height、stamp | 公共图像网格、尺寸和 ROS 主机接收时间戳 |
| receive_mono_ns、age_ms、mock | 机器人单调时钟、响应时年龄、是否模拟 |
| pairing、hardware_synchronized | sdk_frameset、false |
| color | encoding=jpeg、bytes、设备帧号、设备时间戳及 timestamp_quality |
| depth | encoding=32FC1、unit=metre、byte_order=little-endian、step、bytes、NaN 约定、数值语义、设备帧号与时间戳 |
| calibration_available、intrinsics | 对齐网格 CameraInfo 的 K、D、R、P 与畸变模型；仅可用时有 intrinsics |
| ros_projection_compatible | 是否可按 ROS 的 plumb_bob 模型投影；与是否已有真实标定参数分开表示 |

64 位设备帧号和时间数值使用十进制字符串传输，避免 JavaScript 精度损失。所有畸变系数严格为 0 时，SDK 模型可等价表达为 ROS `plumb_bob`。非零的 Brown Conrady 仍用 plumb_bob，其他 SDK 模型保留为 `realsense_N`，N 为 librealsense 的 rs2_distortion 枚举值，例如 `realsense_2` 表示 Inverse Brown Conrady。此时 calibration_available 可为 true，但 ros_projection_compatible=false，不能把 D 直接代入普通 ROS/OpenCV Brown 投影公式；需要使用对应 SDK 模型。这不影响 RGB 像素直接读取同组深度值。光学轴为右、下、前；frame_id 描述图像网格，不代表已经提供到机器人基座的 TF。

兼容深度接口新增 `X-Aligned-To-Color: true/false`，其余 `X-Width`、`X-Height`、`X-Depth-Encoding`、单位、字节序、帧号和时间戳头保留。独立请求仍可能取得不同帧，要求像素对应时使用成对接口。点查询坐标缺失、重复、越界或非整数返回 400，无效测距返回 `valid=false, depth_m=null`。

## 接收示例

仓库提供可执行的接收脚本，依赖 numpy 和 OpenCV，仅发起读取请求：

```bash
python3 Project/luxi_web_ctrl/scripts/receive_rgbd.py \
  http://192.168.0.245:18030 --camera head_rgb --save
```

输出配对元数据，保存 `rgb.jpg` 与 `depth_m.npy`。后者保留 float32 米和 NaN，不能用 JPEG 保存测距。程序读取响应自身的 metadata，校验三部分、尺寸、编码和 pair_id 后返回 `(metadata, color_bgr, depth_m)`。检测 RGB 中 `(x,y)` 的目标后，使用同组 `depth_m[y,x]`；先检查 `np.isfinite(z) and z > 0`，必要时在目标区域统计有效深度。

```python
import sys
import numpy as np
sys.path.insert(0, 'Project/luxi_web_ctrl/scripts')
from receive_rgbd import receive_rgbd

metadata, color_bgr, depth_m = receive_rgbd('http://192.168.0.245:18030', 'head_rgb')
x, y = color_bgr.shape[1] // 2, color_bgr.shape[0] // 2
z = depth_m[y, x]
print(metadata['pair_id'], float(z) if np.isfinite(z) and z > 0 else None)
```

持续读取时设置合适频率、按 camera_id/pair_id 去重并处理网络异常。单路默认深度像素约 18.4 MB/s，三路约 55.3 MB/s，另有 JPEG、DDS 和 HTTP 开销；对齐与 JPEG 编码也占 CPU，三路持续性能必须现场验证。

## ROS 2

| 话题 | 类型与约定 |
| --- | --- |
| `/luxi/cameras/{id}/rgbd` | `luxi_interfaces/msg/RGBDFrame`，整组消息，推荐用于配对处理 |
| `/camera/{id}/depth/image_raw` | `sensor_msgs/msg/Image`，当前模式深度，32FC1 米 |
| `/camera/{id}/depth/camera_info` | `sensor_msgs/msg/CameraInfo`，与深度相同 stamp/frame_id |
| `/camera/{id}/color/camera_info` | 对齐模式下发布的同组网格内参 |
| `/camera/{id}/image/compressed` | 对齐模式下的 RGB JPEG，SensorDataQoS |
| `/luxi/cameras/{id}` | 兼容原来的 RGB CameraFrame，可靠 QoS |
| `/luxi/cameras/{id}/depth` | 深度 CameraFrame，SensorDataQoS |
| `/luxi/cameras/{id}/depth/status` | 模式、mock、采集阶段和错误原因的 JSON |

新增消息包含 header、camera_id、pair_id、sequence、两路 SDK 原始帧号、color/depth CameraFrame 与 camera_info。内部两幅图像 sequence 使用公共组序号，原始设备帧号在外层单独表达。整组消息、深度和内参使用 SensorDataQoS，即 best effort/volatile；不支持该 QoS 的可靠订阅者可能无法匹配。ROS 图像仍须按各自 step 解码。

默认板端 `ROS_LOCALHOST_ONLY=1`，局域网设备直接使用 HTTP。跨机 ROS 需双方匹配接口包与 ROS_DOMAIN_ID、发布和接收进程均配置 `ROS_LOCALHOST_ONLY=0` 并允许 DDS 通信。不要仅修改接收端。

未新增点云、相机到基座外参或深度 episode 录制 schema。对齐输出不会自动出现在原有训练数据文件中。

## 验证

`depth_output` 保留原生深度模式的单位、NaN、ROS 内参配对和退出测试。`rgbd_output` 使用隔离 ROS 域、真实可执行程序的 mock 源和真实网关，检查整组 ROS 消息、标准图像、MIME 二进制解码、无效值、过期 503、损坏消息拒绝及节点重启后配对标识变化。测试不启动任何执行器。

本次对齐模式实机测试结果如下。SDK 依据：[同版本官方对齐示例](https://github.com/realsenseai/librealsense/blob/v2.56.5/examples/align-advanced/rs-align-advanced.cpp)。

### 当前实施状态

设备端已完成新版接口包、相机节点、SDK 后端及 HTTP 网关的构建。`depth_output`、`rgbd_output`、`web_gateway` 三项测试分别通过，检查了原生深度兼容性、整组 ROS 消息、HTTP MIME 解码、NaN、过期、损坏消息拒绝和恢复。本地也已通过接收脚本、XML 与文档示例语法检查。

串行 SDK 初始化改动已编译并完成实测。三路相机按顺序确认出帧后，同时采集，每路 HTTP 接收检查持续约 15 秒；头部、左腕、右腕分别取得 30、32、33 组不同配对数据，并通过停止后 503 检查。相机配置均为 640×480、15 Hz；此处组数取决于测试端轮询速度，不代表验证了全帧率网络吞吐。日志为设备端 `log/rgbd-hardware-serial-final.log`。

Windows 接收端通过实际局域网地址读取三路常驻服务，每路检查 3 组不同 pair_id，共 9 组，全部成功解码 RGB JPEG 与 1,228,800 字节 float32 深度。验证尺寸、米单位、NaN、有效像素、真实内参和配对元数据；响应元数据中的帧龄为 29～79 ms。头部的非零 Inverse Brown Conrady 内参以 `realsense_2` 保留，两个腕部的零畸变按等价 `plumb_bob` 表达。腕部当前场景的有效深度像素占比较低（约 1%～2%），接收方必须处理 NaN，实际目标距离与测距有效范围需现场确认。

`luxi-depth-head_rgb.service`、`luxi-depth-left_wrist_rgb.service`、`luxi-depth-right_wrist_rgb.service` 和 `luxi-rgbd-gateway.service` 均已启用开机启动并处于 running，部署检查时重启次数均为 0。三个旧 `luxi-rgb-*.service` 已停止并禁用开机启动，设备管理入口已映射到对应对齐服务。部署日志为 `log/rgbd-deployment.log`，包含 `DEPLOY_PASS`。未通过设备重启验证开机过程。

本次未执行运动控制命令。以上验证覆盖实际三路并行采集和局域网接收，不等于长期稳定性、硬件曝光同步、相机外参标定或现场测距精度验收。
