# LUXI 机器人接收端数据与控制接入手册

版本 1.1　更新日期 2026 年 10 月 15 日

面向接收方开发人员。本机指运行 LUXI ROS 2 程序的机器人设备，接收端指局域网内的计算机、工控机或其他设备。接收端可通过 HTTP 获取关节、底盘、升降、RGB 和深度数据，并调用现有控制服务；需要连续 ROS 数据时，再接入 ROS 2 话题。

**本次交付地址为 `http://192.168.0.245:18030`。** 本手册对应 `/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2` 的代码及新增 RGB 与对齐深度功能。三路统一采集节点和独立网关已部署运行并启用开机启动，已完成三路同时采集及 Windows 局域网接收测试。其他目录的旧网关不会自动更新，接收方应使用此地址，并在控制前核对当前模式与反馈。

## 1 接入选择与能力范围

| 接收方需求 | 接口选择 | 当前能力与条件 |
| --- | --- | --- |
| 读取关节与设备状态 | `GET /api/state` | HTTP JSON 快照，无需安装 ROS |
| 读取 RGB | `GET /api/cameras/{id}` | JPEG 快照，也可订阅 ROS 压缩图像 |
| 读取深度与内参 | `GET /api/cameras/{id}/depth` 和 `/depth/info` | 新版网关与对应深度节点须已运行 |
| 成对接收 RGB 与对齐深度 | `GET /api/cameras/{id}/rgbd` | 一次请求返回 metadata、JPEG 和对齐深度，按 pair_id 识别同组数据 |
| 查询深度像素距离 | `GET /api/cameras/{id}/depth/point` | 返回当前模式深度像素值；对齐模式可使用对应 RGB 像素 |
| 连续订阅反馈 | ROS 2 话题 | 机器人端和接收端须配置跨机 DDS 通信 |
| 使能与停放 | HTTP `/api/control` 或 ROS `/luxi/control` | 受当前控制模式、反馈、Quest 和状态机门禁约束 |
| 底盘键盘式移动 | HTTP `/api/keyboard` | 仅独立模式；只支持有限平移，不提供该接口下的转向 |
| 底盘连续速度与升降绝对目标 | 机器人侧外部控制话题 | 驱动有本地租期协议；需启用专用模式并增加接收端到机器人侧的桥接 |
| 双臂任意关节轨迹或末端位姿控制 | 需要扩展统一控制入口 | 当前没有公开的 HTTP 关节轨迹接口或标准轨迹 action |
| 获取已完成采集文件 | episode 清单与文件下载接口 | 与实时图像接收分开；深度尚未加入既有录制 schema |

本版本没有自动汇总的整机 `JointState`、轮关节编码器接口或 `/cmd_vel` 控制订阅。双臂 16 维、升降位置和底盘三轴速度分别表达，不能直接拼成一个具有统一关节语义的向量。

## 2 网络与首次联通

### 接入参数

| 参数 | 约定 |
| --- | --- |
| 机器人 IP | 本次设备为 `192.168.0.245`，部署时确认实际地址 |
| HTTP 地址 | 本次 `BASE_URL=http://192.168.0.245:18030`；设备地址变更时由提供方更新 |
| 端口 18030 | 已部署的独立网关 `luxi-rgbd-gateway.service`，与相机使用 ROS 域 0；第 12 节用于重新部署 |
| HTTP 访问控制 | 当前代码无需密码或令牌；局域网可达的客户端可提交控制请求 |
| HTTPS | 当前网关为 HTTP；需要加密或鉴权时另行部署网关或网络访问控制 |
| ROS 版本 | 机器人端 Ubuntu 22.04 ARM64，ROS 2 Humble；接收端 ROS 接入使用匹配版本和消息定义 |
| Quest UDP | 默认 5006，是配套 Quest 输入协议端口，不是通用机器人控制 API |

只用 HTTP 的接收端不需要 SSH 登录、相机 SDK 或 ROS。下列命令在接收端执行：

```bash
export BASE_URL=http://192.168.0.245:18030
curl --fail --max-time 3 "$BASE_URL/api/state"
curl --fail --max-time 3 "$BASE_URL/api/cameras/head_rgb" -o head.jpg
curl --fail --max-time 3 "$BASE_URL/api/cameras/head_rgb/depth/info"
```

如果状态接口可用而深度路径返回 404，先检查是否接入旧网关。新版路径返回 503 则表示尚无新鲜深度帧，需检查节点、序列号、ROS 域和驱动状态。

### 接收顺序

1. 获取 `BASE_URL`、运行版本、相机 ID、控制模式和需要使用的 ROS 域。
2. 仅读取 `/api/state`，检查各子系统有效性、使能和故障状态。
3. 分别接收一路 RGB、一路深度，验证尺寸、编码、单位和过期处理。
4. 有连续订阅需求时再配置 ROS 2，并检查实际话题 QoS。
5. 控制联调先核对第 8 节命令语义与模式，服务成功后继续观察反馈，确认实际结果。

## 3 状态快照与有效性

### 状态入口

`GET /api/state` 返回设备状态对象，主要顶层字段如下。未收到消息的对象可能为 `null`，接收端必须处理缺失值。

| 字段 | 内容 |
| --- | --- |
| `arms` | 16 路机械臂与夹爪反馈、有效位、使能及驱动阶段 |
| `control` | 控制器阶段、原因、输入是否暂停、模式与控制参数 |
| `base` 和 `base_status` | 底盘测量反馈与驱动许可、输入源、故障原因 |
| `lift` 和 `lift_status` | 升降测量反馈与驱动会话、目标和就绪状态 |
| `cameras` | 各 RGB 流状态，以及新版的 `depth`、`rgbd` 子对象 |
| `quest` | 新鲜度、接收频率、左右追踪状态、摇杆和握键 |
| `recorder` | 录制状态、episode、样本数、质量检查等 |
| `system` | 运行模式、数据目录、空间、服务是否就绪 |
| `receiver` | episode 接收队列的状态 |
| `server_mono_ns` | 机器人当前单调时钟，十进制字符串，单位 ns |

`system.control_authorized=true` 仅表示 HTTP 控制接口无需登录，不表示电机已经使能，也不表示某个动作会被接受。`system.services.control/base/lift/episode` 表示相应 ROS 服务是否发现，不能替代设备健康检查。

### 数据有效性规则

| 数据 | 接收方判断重点 |
| --- | --- |
| 双臂 | `arms.fresh` 与每一路 `valid[i]` 同时检查；全局 fresh 不代表所有关节有效 |
| 底盘 | 使用 `base.valid` 判断反馈；`base_status.fresh` 只说明驱动状态消息新鲜 |
| 升降 | 使用 `lift.valid` 判断测量；`lift_status.feedback_healthy`、`ready_for_command` 是控制门禁，含义不同 |
| RGB | `/api/state` 的相机 `fresh`，图像端点在帧超过 1 秒后返回 503 |
| 深度 | `cameras[id].depth.fresh`；图像、内参、像素接口在帧超过 1 秒后返回 503 |
| 控制器 | `control.fresh`、`phase`、`system_phase`、`reason` 与设备反馈结合判断 |

当前网关的双臂全局新鲜度阈值为 150 ms，控制状态为 500 ms，底盘测量有效性还要求接收年龄不超过 250 ms。升降同时检查接收年龄及反馈的有效年龄，不能仅因为位置数值存在就认为可控制。

无效数据可以保留用于诊断，但应在界面标灰、记录无效标记，禁止当作本次有效观测继续驱动动作。数据恢复不等于恢复运动许可。

### 时间与顺序

- `receive_mono_ns`、`source_mono_ns` 等是**机器人本机**单调时钟，不能与接收端的 `time.monotonic_ns()` 直接相减。
- HTTP 状态中可用同一响应的 `server_mono_ns - receive_mono_ns` 估计机器人内部接收年龄；它不包括网络传输耗时。
- 接收端另记自己的接收时刻、请求耗时和本地断流计时。需要跨机采集对齐时，单独建立时钟同步与时间域约定。
- JSON 中的 64 位时间通常用字符串传输。Python 可转为 `int`，JavaScript 使用 `BigInt`，避免浮点整数精度损失。
- `sequence` 是单个流进程内的序号，重启后可能复位。不要永久拒绝小于历史最大值的所有帧；连接重建和进程重启时建立新流会话。

## 4 机械臂和其他关节反馈

### 双臂 16 路顺序

HTTP `arms` 与 ROS `/luxi/arms/state` 使用同一固定顺序。下面的名称是接收端显示标签，不是另行发布的 URDF joint 名称。

| 数组下标 | 显示标签 | 含义 |
| --- | --- | --- |
| 0 至 6 | right_joint_1 至 right_joint_7 | 右臂 7 个关节 |
| 7 | right_gripper | 右夹爪电机 |
| 8 至 14 | left_joint_1 至 left_joint_7 | 左臂 7 个关节 |
| 15 | left_gripper | 左夹爪电机 |

| 字段 | ROS 定义 | 含义与单位 |
| --- | --- | --- |
| `position` | `float64[16]` | 关节或夹爪电机角度，rad；已扣除配置 offset |
| `velocity` | `float64[16]` | 电机角速度，rad/s |
| `effort` | `float64[16]` | OpenArm SDK 的电机 torque 反馈，Nm 语义；不是末端六维力传感器测量 |
| `mos_temperature` | `int32[16]` | 驱动 MOS 温度，摄氏度 |
| `rotor_temperature` | `int32[16]` | 转子温度，摄氏度 |
| `age_ns` | `int64[16]` | 各路实际反馈年龄；未收到时可为 `INT64_MAX` |
| `valid` | `bool[16]` | 每一路反馈是否有效 |
| `enabled` | `bool` | 驱动使能状态；为 true 仍可能处于保持或故障处理阶段 |
| `phase` | `string` | 驱动阶段或原因，例如 disabled、保持等 |
| `receive_mono_ns` | `int64` | 机器人发布该反馈时使用的单调时钟 |
| `header` | `std_msgs/Header` | ROS 时间戳等；HTTP `arms` 不完整镜像该对象 |

夹爪角度不是 0 至 1 的开合比例，也不是米。当前 `arms-teleop.json` 中右夹爪开位约为 `-1.42789 rad`，左夹爪开位约为 `+1.532796 rad`，闭位均为 0；实际使用应读取部署配置，不能给左右夹爪统一套用同号映射。

当前活动控制程序曾使用 `left_only` 模式，数组仍保持 16 路。接收端如看到 `control.arm_mode` 或 `active_joint_indices`，应结合这些字段与 `valid[i]` 选取实际关节；未接入侧保留的数值不能充当真实反馈。新网关保留控制状态的扩展字段，但不保证所有版本都提供模式字段。

### 升降反馈

HTTP `lift` 对应 `/luxi/lift/state` 的 JSON，schema 为 `lift_state_v1`。

| 字段 | 含义 |
| --- | --- |
| `current_um` 和 `position_m` | 当前升降坐标，分别为微米和米；`position_m = current_um × 1e-6` |
| `target_um` | 设备当前目标坐标，微米；不是到位确认 |
| `speed_rpm` | 电机转速，rpm；未定义丝杆换算前不能当作 m/s |
| `sequence`、`uptime_ms` | 完整遥测样本序号与设备运行时间 |
| `valid`、`feedback_age_ms`、`effective_age_ms` | 测量有效性与反馈年龄 |
| `flags`、`fault`、`guard`、`target_status` | 设备协议原始状态与故障字段 |
| `timestamp_quality`、`receive_mono_ns`、`ros_time_ns` | 时间质量及接收时间 |

`lift_status` 额外提供 `phase`、`reason`、`enabled`、`stationary`、`session_present`、`feedback_healthy`、`ready_for_command`、边界和外部任务状态。当前部署使用 UART，配置主机边界为 `-180000` 至 `180000 µm`，默认内缩 `5000 µm`；可下发目标范围还需与设备返回的边界求交，不能只依据这组静态值。

### 底盘反馈

HTTP `base` 对应 `/luxi/base/state` 的 JSON，schema 为 `r680_v1`。

| 字段 | 含义与单位 |
| --- | --- |
| `velocity[3]` | 底盘 `vx, vy, wz`，前两项 m/s，第三项 rad/s |
| `acceleration[3]` | 加速度 `ax, ay, az`，m/s² |
| `gyro[3]` | 角速度 `wx, wy, wz`，rad/s |
| `voltage` | 电池电压，V |
| `flag_stop` | 底盘反馈停止标记，不等于上层控制器的全部状态 |
| `valid`、`mock` | 本次测量有效性与模拟标记 |
| `frame_mono_ns` | 组成完整测量的三帧 CAN 接收时间 |

也发布 `/base/velocity`、`/base/imu`、`/base/battery` 标准消息，坐标系标记为 `base_link`。IMU 不提供有效姿态估计，`orientation_covariance[0] = -1`；电池未测量项可能为 NaN。当前没有各轮角度、轮速编码器或累计里程计消息，不可把整机 `velocity` 当成三个轮关节。

## 5 RGB 摄像头接收

### 相机标识

| `id` | 视角标签 | USB 或 ASIC 序列号 | RealSense SDK 主序列号 |
| --- | --- | --- | --- |
| `head_rgb` | 头部 | 261043062235 | 260922304036 |
| `left_wrist_rgb` | 左腕 | 254843065015 | 244622073011 |
| `right_wrist_rgb` | 右腕 | 261443062039 | 262422075883 |

软件标签与物理安装位置仍需现场核对，接收方不要靠 `/dev/videoN` 的编号识别相机。

### HTTP 与编码

`GET /api/cameras/{id}` 返回 `Content-Type: image/jpeg`。原 V4L2 RGB 配置为 640×360、30 Hz；新版 SDK 对齐节点默认同时输出 640×480、15 Hz 的 RGB 和深度。尺寸和驱动帧率应从实际状态读取。HTTP 每次获取一张最新快照，不是 RTSP、MJPEG 或 WebSocket 视频流；接收端自行按需要轮询，并用帧号去重。要求 RGB 与深度来自同组时使用第 6 节的成对接口。

响应头包括 `X-Frame-Id`、`X-Receive-Mono-Ns`、`X-Device-Timestamp`、`X-Timestamp-Quality`。当前 V4L2 RGB 的设备时间戳为 0，时间质量通常为 `v4l2_host_monotonic`；不要将通用 RGB 的原始设备时间字段与深度明确标注的纳秒字段混为一谈。

ROS 图像入口为 `/camera/{id}/image/compressed`，类型 `sensor_msgs/msg/CompressedImage`，format 为 jpeg。内部 `/luxi/cameras/{id}` 使用自定义 `CameraFrame`，data 也是 JPEG 字节。

## 6 深度画面和相机内参

### 整幅深度

新版节点默认 `align_to_color=true`，统一采集同一相机的 RGB 和深度并将深度对齐到 RGB 像素网格。保留 `align_to_color=false` 的原生深度模式。启动对齐节点前必须停止同一相机的旧 RGB 服务，避免设备占用和重复发布；详见第 12 节。

`GET /api/cameras/{id}/depth` 返回 `application/octet-stream`：

| 项目 | 约定 |
| --- | --- |
| 编码 | `32FC1`，每像素一个 IEEE 754 float32 |
| 单位 | 米 |
| 字节序 | little-endian，小端 |
| 排列 | 行优先，宽度方向连续；HTTP 行步长为 `width × 4` |
| 无效像素 | NaN；原始相机 0 值转换为 NaN |
| 默认采集 | 640×480、15 Hz；由机器人端配置和设备支持的 profile 决定 |
| 单帧长度 | 默认 `640 × 480 × 4 = 1,228,800` 字节 |
| 坐标 | 光学轴右、下、前；对齐模式为 RGB 图像网格，原生模式为深度图像网格 |
| RGB 对齐 | 通过 `aligned_to_color` 和响应头区分，默认对齐模式为 true |

深度不使用 JPEG 或伪彩色编码。接收端可将有效深度另行着色显示，但应保留原始 float32 数组，不能从显示颜色反推距离。非 NaN 的远距离值仍需按应用量程过滤；本接口不代替现场测距精度与量程验收。

| 原始响应头 | 含义 |
| --- | --- |
| `X-Width`、`X-Height` | 当前帧尺寸 |
| `X-Depth-Encoding`、`X-Depth-Unit`、`X-Byte-Order` | `32FC1`、`metre`、`little-endian` |
| `X-Mock` | 是否模拟数据，真实采集应为 false |
| `X-Frame-Id` | 当前帧序号 |
| `X-Optical-Frame` | 对齐模式如 `head_rgb_color_optical`；原生模式为 `head_rgb_depth_optical` |
| `X-Aligned-To-Color` | true 或 false，不能仅根据相同尺寸判断对齐 |
| `X-Stamp-Sec`、`X-Stamp-Nanosec` | 当前帧 ROS 时间戳，主机接收时刻 |
| `X-Device-Timestamp-Ns`、`X-Timestamp-Quality` | SDK 时间值转为 ns 及对应时间域 |
| `X-Receive-Mono-Ns`、`X-Age-Ms` | 机器人接收单调时间及服务端帧年龄 |

先从本次原始响应头读取尺寸，再解码本次 body。不要先请求 info 再假定稍后获取的原始图像必然为同一帧。

### 内参与像素距离

`GET /api/cameras/{id}/depth/info` 返回 JSON：尺寸、encoding、unit、sequence、frame_id、时间、mock、aligned_to_color，以及 `calibration_available`。内参与图像 stamp、frame_id、尺寸匹配时，才提供 `intrinsics`：

```json
{
  "camera_id": "head_rgb",
  "encoding": "32FC1",
  "unit": "metre",
  "width": 640,
  "height": 480,
  "mock": false,
  "aligned_to_color": false,
  "calibration_available": true,
  "intrinsics": {
    "distortion_model": "plumb_bob",
    "K": [388.9873, 0, 318.5822, 0, 388.9873, 239.5494, 0, 0, 1],
    "D": [0, 0, 0, 0, 0]
  }
}
```

以上为原生深度模式的字段示例，实际响应还包含 R、P 和时间等字段。对齐模式返回 `aligned_to_color=true` 及对齐网格的内参。每台相机使用自己的实时内参，不能复制示例 K 或将原生深度 K 套用到 RGB。

零畸变可以等价表达为 ROS plumb_bob；非零的 SDK 特殊模型以 `realsense_N` 保存，N 为 rs2_distortion 枚举值，`realsense_2` 为 Inverse Brown Conrady。成对元数据的 `ros_projection_compatible=false` 表示不能使用普通 ROS/OpenCV Brown 投影公式，并不表示标定参数缺失。像素对应读取深度不受影响，三维计算应使用实际模型。

`GET /api/cameras/{id}/depth/point?x=320&y=240` 返回同类元数据，加上 `x`、`y`、`valid` 和 `depth_m`。无效像素返回 `valid=false, depth_m=null`；缺坐标、重复坐标、非整数或越界返回 400。x 和 y 是**深度图**像素，原点在左上角。

原生深度模式下，当 `calibration_available=true` 且畸变为零时，可用 `fx=K[0]`、`fy=K[4]`、`cx=K[2]`、`cy=K[5]` 反投影：`X=(u-cx)Z/fx`，`Y=(v-cy)Z/fy`。非零畸变需先按对应模型校正。对齐模式沿用 SDK align 将源深度 Z 重采样到 RGB 像素的数值语义，并非重新计算严格的 RGB 光轴 Z；精确三维点应由原始深度反投影再用外参变换。本接口不提供完整原始标定链或机器人基座外参。

SDK 时间域通过 `timestamp_quality` 区分 Hardware Clock、System Time、Global Time，不能无条件视为 Unix 时间或多相机硬件同步依据。RGB 与深度、不同相机之间均不保证曝光同步。

ROS 深度图为 `/camera/{id}/depth/image_raw`，类型 `sensor_msgs/msg/Image`；内参为 `/camera/{id}/depth/camera_info`，类型 `sensor_msgs/msg/CameraInfo`。两者 stamp 与 frame_id 配对。内部网关传输为 `/luxi/cameras/{id}/depth`，自定义 `CameraFrame.encoding=32FC1`。

### 成对获取 RGB 与对齐深度

`GET /api/cameras/{id}/rgbd` 返回同一 SDK frameset 产生的不可变快照，响应为 `multipart/mixed`，包含 metadata JSON、color JPEG、depth 小端 float32 三部分。三部分的 `Content-Disposition` name 分别为 `metadata`、`color`、`depth`，像素不做 base64 编码。响应头 `X-Pair-Id` 与 metadata.pair_id 相同。

metadata 包含 `schema=luxi_rgbd_v1`、camera_id、pair_id、sequence、width、height、frame_id、stamp、receive_mono_ns、age_ms、mock、`aligned_to_color=true`、`pairing=sdk_frameset`、`hardware_synchronized=false`，以及 color/depth 的编码、字节数、设备帧号、设备时间戳和时钟域；有效标定在 intrinsics 中。组序号和 64 位设备时间、帧号均为十进制字符串。depth 另有 `step=width*4`、`unit=metre`、`byte_order=little-endian`、`invalid_value=NaN`、`value_semantics=source_depth_z_resampled_to_color`。

RGB JPEG 解码后尺寸与深度一致，可用同组 RGB 的 `(x,y)` 读取 `depth_m[y,x]`。遮挡、边界和无法测距处仍有 NaN。相机 ID 与 pair_id 共同识别一组，节点重启会换会话标识；不应跨相机比较组序号。SDK frameset 配对不是硬件曝光同步，运动场景应评估两路原始时间偏差。

`GET /api/cameras/{id}/rgbd/info` 只返回最新元数据，适合诊断；解码像素须使用同次 `/rgbd` 响应内的 metadata。无对齐节点、只运行原生深度或整组数据超过 1 秒未更新时返回 503，旧网关返回 404。不将旧 RGB 进程和独立深度节点的帧临时拼接。`/api/state` 的 `cameras[id].rgbd` 提供 fresh、pair_id 和 url。

成对接口的完整 MIME 字段与部署方式见 [RGB 与对齐深度输出](DEPTH_OUTPUT_ZH.md)。接收端不应分别请求旧 RGB 和深度接口后假定它们已配对。

## 7 ROS 2 连续数据接入

### 跨机配置

当前板端服务多使用 `ROS_LOCALHOST_ONLY=1`，接收端单独修改环境变量无法跨机订阅。由提供方将选定发布节点以 `ROS_LOCALHOST_ONLY=0` 启动，并确认 DDS 网络可达；接收端使用相同的 `ROS_DOMAIN_ID`。Humble 的环境变量与 QoS 行为见 [ROS 官方环境配置源码](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.rst) 和 [QoS 说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。

接收端使用 Ubuntu 22.04 / ROS 2 Humble 时，只需编译同版本自定义接口包，不要复制机器人上的 ARM64 二进制到 x86 计算机：

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/luxi_rx_ws/src
scp -r sunrise@192.168.0.245:/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2/Project/robot_interfaces \
  ~/luxi_rx_ws/src/
cd ~/luxi_rx_ws
colcon build --packages-select luxi_interfaces
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic list
ros2 topic info /camera/head_rgb/depth/image_raw --verbose
ros2 topic echo /luxi/arms/state --once
```

域号 0 是示例，应与提供方实际部署一致。DDS 通常需要局域网组播或专门的发现配置；不要把 ROS 发现理解为单个固定 TCP 端口。跨网段、VPN、容器和多网卡环境需单独验证发现及图像负载传输。

### 话题目录与 QoS

R 表示 reliable，BE 表示 best effort，V 表示 volatile，TL 表示 transient local。队列深度与频率是当前源码设置或标称周期，实际接收频率以现场测量为准。默认 reliable 订阅者不能接收 BE 发布者，图像接收应使用 SensorDataQoS。

| 话题 | 类型 | 当前发布配置或含义 |
| --- | --- | --- |
| `/luxi/arms/state` | `luxi_interfaces/msg/ArmState` | R V，深度 5；驱动标称 10 ms 周期 |
| `/luxi/control_status` | `std_msgs/msg/String` | JSON，R TL，深度 1 |
| `/luxi/base/state` | `std_msgs/msg/String` | JSON，R V，完整底盘反馈 |
| `/luxi/base/status` | `std_msgs/msg/String` | JSON，R TL，驱动状态 |
| `/base/velocity` | `geometry_msgs/msg/TwistStamped` | R V，整机速度反馈 |
| `/base/imu` | `sensor_msgs/msg/Imu` | R V，无有效姿态估计 |
| `/base/battery` | `sensor_msgs/msg/BatteryState` | R V，主要提供电压 |
| `/luxi/lift/state` | `std_msgs/msg/String` | JSON，R V，完整升降测量 |
| `/luxi/lift/status` | `std_msgs/msg/String` | JSON，R TL，升降驱动状态 |
| `/camera/{id}/image/compressed` | `sensor_msgs/msg/CompressedImage` | SensorDataQoS，BE V，JPEG |
| `/camera/{id}/depth/image_raw` | `sensor_msgs/msg/Image` | SensorDataQoS，BE V，32FC1 米 |
| `/camera/{id}/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | SensorDataQoS，BE V，匹配深度图 |
| `/luxi/cameras/{id}` | `luxi_interfaces/msg/CameraFrame` | R V，旧 RGB 深度 4，对齐节点深度 1 |
| `/luxi/cameras/{id}/rgbd` | `luxi_interfaces/msg/RGBDFrame` | SensorDataQoS，BE V，深度 1，RGB 与对齐深度及内参整组传输 |
| `/camera/{id}/color/camera_info` | `sensor_msgs/msg/CameraInfo` | SensorDataQoS，BE V，对齐节点提供的图像网格内参 |
| `/luxi/cameras/{id}/depth` | `luxi_interfaces/msg/CameraFrame` | SensorDataQoS，BE V，深度内部传输 |
| `/luxi/cameras/{id}/depth/status` | `std_msgs/msg/String` | JSON，R TL，深度驱动状态与错误 |
| `/luxi/recorder/status` | `std_msgs/msg/String` | JSON，R TL，录制状态 |

JSON 状态话题须先读取 `msg.data` 再解析 JSON，不是直接把 `std_msgs/String` 当成设备状态结构体。

`CameraFrame` 的完整字段为：`header`、`camera_id`、`sequence`、`device_timestamp`、`receive_mono_ns`、`timestamp_quality`、`width`、`height`、`encoding`、`data`。RGB 的 `encoding=jpeg`；新增深度为 `32FC1`，采用小端 float32 米。不能对两者都执行 JPEG 解码。

新增 `RGBDFrame` 包含公共 header、camera_id、pair_id、sequence、depth_device_sequence、color_device_sequence、color/depth CameraFrame 与 camera_info。内部两帧使用公共组序号、stamp 和 color_optical 图像网格，原始设备时间分别保留。跨机 ROS 接收端需要更新接口包；配对处理优先订阅整组消息，无需在多个独立话题之间自行猜测对应关系。

### 实测值与控制目标

`/luxi/arms/state` 才是实际关节反馈。`/luxi/targets` 是映射器生成的末端目标，`/luxi/intent` 是 IK 解，`/luxi/command` 是控制管理器输出，`/luxi/applied` 是驱动处理后的关节指令。后四者都不能冒充机械臂实际位置，也不等于机器人基座下已标定的实测末端位姿。

## 8 现有控制服务

### 请求与响应

HTTP 控制统一使用 `POST`、`Content-Type: application/json`：

```json
{"command":"start","detail":""}
```

`detail` 必须是字符串，即使里面承载 JSON 数组也要先序列化成字符串。通用控制端点的请求体上限为 16 KiB。成功响应为 `{"success":true,"message":"..."}`；状态机命令返回成功表示请求被接受，不能作为使能已完成、运动已到位或机械已停止的最终证明。

| HTTP 端点 | ROS 服务 | 类型 |
| --- | --- | --- |
| `/api/control` | `/luxi/control` | `luxi_interfaces/srv/Command` |
| `/api/base` | `/luxi/base/control` | 同上 |
| `/api/keyboard` | `/luxi/base/control` | 同上，额外限制命令集合 |
| `/api/lift` | `/luxi/lift/control` | 同上 |
| `/api/episode` | `/luxi/episode` | 同上 |

服务定义如下：

```text
string command
string detail
---
bool success
string message
```

控制调用必须串行处理明确的状态转换。HTTP 网关等待服务响应最长约 10 秒，客户端读取超时可设为 12 秒。遇到超时或断线，动作结果可能未知，先读取状态再决定下一步，不自动重放运动命令。

### 统一模式与独立模式

先检查 `control.shared_enable`、`base_status.shared_enable`、`lift_status.shared_enable` 以及各驱动的 `external_control`。统一模式通过 `/api/control` 协调设备，底盘与升降的独立使能会被拒绝。外部控制模式又与统一模式互斥，不能通过重复发送 start 强行切换模式。

| 目标与命令 | `shared_enable=true` | `shared_enable=false` |
| --- | --- | --- |
| `/api/control` 的 `start` | 准备底盘和升降，再启动双臂归位、对齐，最后开放整机许可；依赖有效 Quest 等门禁 | 启动双臂 homing、aligning、teleop 流程，仍依赖有效 Quest、IK 和反馈 |
| `/api/control` 的 `halt` | 停止并保持；不回位，也不移除双臂转矩 | 当前独立分支不支持该命令 |
| `/api/control` 的 `disable` | 与 halt 同走保持路径；不能据此认定双臂已失能 | 发出双臂失能命令，语义与统一模式不同 |
| `/api/control` 的 `stop` | 与 park 同走顺序停放路径 | 双臂返回配置姿态后失能 |
| `/api/control` 的 `park` | 停底盘与升降，确认停稳后执行双臂返回序列，再撤销双臂转矩 | 当前独立分支不支持该命令 |

**当前代码的统一停放保持升降在停止高度，不发送升降 HOME 或 MOVE。** 双臂返回采用标称 50 Hz、100 步的定时序列，完成该序列不等于每个关节已实测到达目标；最终还需检查反馈和 enabled。不要依据旧版“先升降归零再回臂”的说明控制当前版本。

统一 start 要求设备反馈健康、无失败菜单、Quest 近期输入充分且追踪有效、摇杆回中、握键释放等；HTTP 发 start 不能替代缺失的 Quest 输入。收到 `enable_rejected...` 等原因时，处理对应条件并由操作者明确重新请求，不进入无限重试。

以下是**手动调用模板**，不是接收程序启动后自动执行的步骤：

```bash
# 仅在已确认使用统一模式时请求保持
curl --max-time 12 -H 'Content-Type: application/json' \
  -d '{"command":"halt","detail":"receiver requested hold"}' "$BASE_URL/api/control"

# 显式请求停放，该操作包含机械臂运动
curl --max-time 12 -H 'Content-Type: application/json' \
  -d '{"command":"park","detail":""}' "$BASE_URL/api/control"
```

软件 halt、stop、park 和 disable 的含义各不相同，都不能当成独立硬件急停。接收端界面应显示实际请求名称、控制模式及返回原因，避免用一个含糊的“停止”按钮混合所有语义。

### 底盘控制

独立 Quest 模式下，`/api/base` 支持 `start`、`stop`、`disable`。start 是开放许可，速度仍来自 Quest，不会因为发送任意 detail 就变成速度控制。

独立键盘模式通过 `/api/keyboard` 调用：

| command | detail | 含义 |
| --- | --- | --- |
| `keyboard_start` | 空字符串 | 请求进入键盘许可，初始速度为零 |
| `keyboard` | 例如字符串 `"[0.2,0.0]"` | 两个有限数值，各在 -1 至 1，按驱动规则映射平移速度 |
| `stop` | 空字符串 | 停车并撤销许可 |

键盘平移向量按 `0.05 / max(1, hypot(x,y))` 缩放，最大平移速度为 0.05 m/s，角速度固定为零。输入需要在 200 ms 租期内持续更新，建议约 20 Hz；中断、无效输入或反馈故障会停车。统一模式或 external 模式下不能使用这套键盘使能入口。

### 升降控制

`/api/lift` 提供 `start`、`disable`、`stop`。独立 start 读取配置并开放 Quest 握键控制许可，本身不是任意位置目标接口。disable 取消当前驱动控制，stop 还发送设备全局 STOP；响应成功仍须结合 `stationary`、会话和位置反馈确认。

HTTP 目前没有 `move_to`、`target_m` 或 `set_height` 接口。外部绝对升降目标使用第 9 节的机器人侧专用模式。`speed_rpm` 是反馈，不是本接口支持的线速度命令。

### 录制与设备管理

`/api/episode` 支持 `start`、`success`、`fail`、`cancel`。start 的 detail 为任务说明，结束命令的 detail 可承载说明或失败原因。cancel 也保留原始数据，不能解释为删除本次数据。

`GET /api/devices` 查询服务管理状态。`POST /api/devices/{id}` 使用 `{"action":"start"}`、stop 或 restart，支持 arms、base、lift、quest、program 及三个 RGB id。它是程序生命周期管理，不是运动使能；录制中或设备使能时可能返回 409。新增深度 systemd 服务未加入该 HTTP 管理白名单，需机器人端按深度说明管理。

## 9 连续外部运动控制与实现边界

### 双臂控制需要机器人侧仲裁入口

当前双臂控制管理器接收内部 IK 指令并依赖 Quest 状态，不能将外部模型的 16 路输出直接 POST 给网关。当前也未提供 `FollowJointTrajectory` action、通用关节目标 HTTP 路由或公开的末端目标 action。

需要接收端自主控制双臂时，应新增机器人侧桥接与控制模式，负责接收端会话、独占控制权、指令序号与租期、机器人本地时间戳、关节限位与速度限制、反馈检查，以及停止和重新使能流程。它应接入控制管理层，处理与现有映射器、IK 和 Quest 的互斥关系。`/luxi/command` 和 `/luxi/driver_lifecycle` 是内部驱动链路，不应作为接收方绕过仲裁直接发布的控制入口。

### 底盘与升降已有本地外部协议

这两类驱动已有 `std_msgs/msg/String` JSON 话题，但当前活动部署的 `external_control=false`。切换模式需要机器人端配置和重启对应驱动，不是通过 HTTP 运行时开关完成。驱动模式切换与新的远端桥接不属于本次文档生成时自动执行的操作。

| 接口 | 启用条件 | 支持的 op |
| --- | --- | --- |
| `/luxi/base/external_command` | `external_control=true`，`shared_enable=false`，非只读模式 | arm、velocity、stop |
| `/luxi/lift/external_command` | `external_control=true`，`shared_enable=false`，UART，硬件与运动许可已配置 | arm、heartbeat、move、stop |

通用字段如下：

```json
{
  "owner": "new-owner-uuid",
  "sequence": 1,
  "source_mono_ns": "ROBOT_LOCAL_MONOTONIC_NS",
  "op": "arm"
}
```

这里的 `source_mono_ns` 必须由**运行在机器人本机的桥接程序**写入真实十进制 ns 字符串，上例占位符不能直接发送。远端计算机的 monotonic 时间通常与机器人不同，不能直接填入。该协议属于机器人侧驱动接口，不是已经实现的跨机 HTTP API。

租期约定：owner 非空且最长 64 字符；sequence 是正的无符号整数，后续严格递增；源时间不得晚于机器人当前时间，接收时年龄不得超过 150 ms，也不能倒退；运动租期为 200 ms。停止、错误或租期失效后，必须使用新的 owner 明确 arm，晚到的数据不能重新开放许可。桥接侧还应检查远端连接和包序号，不能持续刷新已经失联接收端的旧目标。

**底盘外部模式。** arm 要求新鲜、停稳的底盘反馈等条件；velocity 增加 `"velocity":[vx,vy,wz]`。调试上限为平移向量模长 0.10 m/s、角速度绝对值 0.20 rad/s，设备配置可能进一步限速。底盘没有单独的 heartbeat op，保持租期应持续发送新的 velocity，包括零速度；发送未知 op 会撤销许可。可观察 `base_status.external_control`、`external_owner`、enabled、reason 和实际速度反馈。

**升降外部模式。** arm 后先等待配置与就绪；heartbeat 保持租期；move 增加正整数 `task_id` 和整数 `target_um`。task_id 在会话内递增，目标必须在设备与主机边界交集的内缩范围内，驱动拒绝越界，不静默截断。一个有限目标运行期间持续 heartbeat，不重复高频发送 MOVE。通过 `lift_status.external_task_id`、`external_result`、`external_target_um`、phase 和位置反馈判断任务；只有 `external_result=reached` 等最终条件满足才认定完成。当前驱动任务总等待有 180 秒上限，故障后需新会话。

下列仅是本地桥接协议的负载示意，公共字段须按上述规则补齐：

```json
{"op":"velocity","velocity":[0.02,0.0,0.0]}
```

```json
{"op":"move","task_id":1,"target_um":0}
```

### 接收端实现要求

| 接收端模块 | 实现要求 |
| --- | --- |
| 状态缓存 | 按设备和流分开保存，保留 valid、时间域、序号与断流标记 |
| 控制会话 | 一次只认定一个明确控制源，主动启用，断线后不自动恢复 |
| 指令发送 | 超时结果不明时先查状态；租期命令不排队重放旧目标 |
| 机器人侧桥接 | 为远端输入建立本地租期和时间戳，按模式对接驱动或控制管理器 |
| 完成判定 | 请求应答、驱动接受、运动执行、实测到位分别表示 |
| 控制日志 | 记录客户端序号、请求、响应、状态原因与实际反馈，保留时序关系 |

## 10 接收端 Python 示例

### HTTP 接收状态与两类图像

需要成对图像时，复制仓库 `Project/luxi_web_ctrl/scripts/receive_rgbd.py`，安装 numpy 与 OpenCV 后执行 `python3 receive_rgbd.py http://192.168.0.245:18030 --camera head_rgb --save`，一次取回 RGB、对齐深度和配对元数据。脚本可在 Windows 使用，且不需要 requests、ROS 或相机 SDK。程序入口 `receive_rgbd(base_url, camera)` 返回 metadata、BGR 图和深度米数组；按 `np.isfinite(depth_m) & (depth_m > 0)` 筛选有效值。

以下旧接口示例用于分别诊断状态、RGB 和深度，不能替代成对读取。安装 `requests`、`numpy`、`opencv-python` 后，保存为 `receiver_http.py`。它默认只读，不发送控制请求；独立快照不能假定来自同组。Shell 命令按 bash 编写，Windows 可用 `curl.exe` 避免 PowerShell 的 curl 别名差异。

```python
import argparse
import cv2
import numpy as np
import requests


def receive(base_url, camera):
    base_url = base_url.rstrip('/')
    with requests.Session() as session:
        response = session.get(base_url + '/api/state', timeout=(2, 3))
        response.raise_for_status()
        state = response.json()
        arms = state.get('arms') or {}
        print('control:', state.get('control'))
        if arms.get('fresh'):
            for index, valid in enumerate(arms.get('valid', [])):
                if valid:
                    print('joint', index, 'rad', arms['position'][index])
        print('base:', state.get('base'))
        print('lift:', state.get('lift'))

        prefix = base_url + '/api/cameras/' + camera
        rgb_response = session.get(prefix, timeout=(2, 3))
        if rgb_response.status_code == 503:
            print('RGB unavailable or stale')
        else:
            rgb_response.raise_for_status()
            rgb = cv2.imdecode(np.frombuffer(rgb_response.content, np.uint8),
                               cv2.IMREAD_COLOR)
            if rgb is None:
                raise ValueError('Invalid JPEG')
            print('RGB BGR shape:', rgb.shape)

        depth_response = session.get(prefix + '/depth', timeout=(2, 3))
        if depth_response.status_code in (404, 503):
            print('Depth unavailable, stale, or gateway version is old:',
                  depth_response.status_code)
            return
        depth_response.raise_for_status()
        headers = depth_response.headers
        if (headers.get('X-Depth-Encoding') != '32FC1'
                or headers.get('X-Depth-Unit') != 'metre'
                or headers.get('X-Byte-Order') != 'little-endian'):
            raise ValueError('Unsupported depth format')
        width, height = int(headers['X-Width']), int(headers['X-Height'])
        if width <= 0 or height <= 0 or len(depth_response.content) != width * height * 4:
            raise ValueError('Depth payload size mismatch')
        depth_m = np.frombuffer(depth_response.content, dtype='<f4').reshape(height, width)
        valid = np.isfinite(depth_m) & (depth_m > 0)
        center = depth_m[height // 2, width // 2]
        print('Depth shape:', depth_m.shape, 'mock:', headers.get('X-Mock'))
        print('Depth valid fraction:', float(valid.mean()))
        print('Center Z metres:', float(center) if np.isfinite(center) and center > 0 else None)
        print('Depth frame:', headers.get('X-Frame-Id'), headers.get('X-Optical-Frame'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('base_url')
    parser.add_argument('--camera', default='head_rgb',
                        choices=['head_rgb', 'left_wrist_rgb', 'right_wrist_rgb'])
    args = parser.parse_args()
    receive(args.base_url, args.camera)
```

```bash
python receiver_http.py http://192.168.0.245:18030 --camera head_rgb
```

需要持续接收时，在上层按设定周期调用或改成常驻会话，丢弃重复帧并处理请求异常。默认深度单路若以 15 帧每秒获取，纯像素负载约为 18.4 MB/s；三路约 55.3 MB/s，尚未包含 RGB、HTTP 和 DDS 开销。根据网络和处理能力选择相机及读取频率。

### ROS 订阅整组成对图像

接收端编译新版接口包后，可直接订阅整组消息，避免独立话题匹配。下例只读取图像，不发送控制命令；生产程序还应按 camera_id/pair_id 去重，并用接收端单调时钟监测断流。

```python
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from luxi_interfaces.msg import RGBDFrame


def receive_pair(pair):
    if not (pair.header == pair.color.header == pair.depth.header == pair.camera_info.header):
        return
    w, h = pair.depth.width, pair.depth.height
    if (pair.color.encoding != 'jpeg' or pair.depth.encoding != '32FC1'
            or w != pair.color.width or h != pair.color.height
            or w == 0 or h == 0 or len(pair.depth.data) != w * h * 4):
        return
    color = cv2.imdecode(np.frombuffer(bytes(pair.color.data), np.uint8), cv2.IMREAD_COLOR)
    if color is None or color.shape[:2] != (h, w):
        return
    depth = np.frombuffer(bytes(pair.depth.data), dtype='<f4').reshape(h, w)
    valid = np.isfinite(depth) & (depth > 0)
    print(pair.camera_id, pair.pair_id, color.shape, float(valid.mean()))


if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('luxi_rgbd_receiver')
    subscription = node.create_subscription(RGBDFrame, '/luxi/cameras/head_rgb/rgbd',
                                             receive_pair, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

### ROS 订阅关节与深度

运行前 source ROS 和接收端接口工作空间。本例订阅实际关节、深度和内参，不发送运动命令；深度回调按消息的 step 处理行步长，而不是假设所有 ROS 图像都紧密排列。

```python
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from luxi_interfaces.msg import ArmState
from sensor_msgs.msg import Image, CameraInfo


class Receiver(Node):
    def __init__(self):
        super().__init__('luxi_receiver_example')
        self.subscriptions_ = [
            self.create_subscription(ArmState, '/luxi/arms/state', self.arms, 5),
            self.create_subscription(Image, '/camera/head_rgb/depth/image_raw',
                                     self.depth, qos_profile_sensor_data),
            self.create_subscription(CameraInfo, '/camera/head_rgb/depth/camera_info',
                                     self.info, qos_profile_sensor_data),
        ]
        self.last_depth_stamp = None
        self.last_info_stamp = None
        self.latest_info = None
        self.latest_arms = None
        self.latest_depth = None
        self.latest_depth_valid = None
        self.last_arm_receive = None
        self.last_depth_receive = None

    @staticmethod
    def stamp(message):
        return (message.header.stamp.sec, message.header.stamp.nanosec,
                message.header.frame_id)

    def arms(self, message):
        # 只使用 valid 为真的通道；应用层另外监测本地接收超时。
        measured = {i: q for i, q in enumerate(message.position) if message.valid[i]}
        self.latest_arms = measured
        self.last_arm_receive = time.monotonic()

    def info(self, message):
        self.last_info_stamp = self.stamp(message)
        self.latest_info = message

    def depth(self, message):
        if (message.encoding != '32FC1' or message.width == 0 or message.height == 0
                or message.step < message.width * 4
                or len(message.data) != message.step * message.height):
            return
        dtype = '>f4' if message.is_bigendian else '<f4'
        view = np.ndarray(shape=(message.height, message.width), dtype=dtype,
                          buffer=bytes(message.data), strides=(message.step, 4))
        depth_m = view.copy()
        valid = np.isfinite(depth_m) & (depth_m > 0)
        self.latest_depth = depth_m
        self.latest_depth_valid = valid
        self.last_depth_receive = time.monotonic()
        self.last_depth_stamp = self.stamp(message)
        # 反投影时必须使用相同 stamp/frame_id 和尺寸的 CameraInfo。
        # 两个话题到达顺序不同，生产程序应按 stamp 建立小型匹配缓存。


if __name__ == '__main__':
    rclpy.init()
    node = Receiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

### 控制客户端函数

下列函数只在调用者明确指定命令时提交一次请求，不包含自动重试。控制模式与门禁按第 8 节处理。函数返回成功后仍需轮询 `/api/state` 确认目标阶段或实测结果。

```python
import requests


def send_command(base_url, endpoint, command, detail=''):
    if endpoint not in ('control', 'base', 'keyboard', 'lift', 'episode'):
        raise ValueError('Unknown control endpoint')
    if not isinstance(detail, str):
        raise TypeError('detail must be a string')
    response = requests.post(base_url.rstrip('/') + '/api/' + endpoint,
                             json={'command': command, 'detail': detail},
                             timeout=(2, 12))
    response.raise_for_status()
    result = response.json()
    if not result.get('success', False):
        raise RuntimeError(result.get('message', 'Command rejected'))
    return result
```

## 11 文件接收与错误处理

### 已完成 episode

| HTTP 路径 | 含义 |
| --- | --- |
| `/api/episodes` | 已提交 episode ID 列表 |
| `/api/episodes/{id}/manifest` | 该 episode 的 manifest |
| `/api/files/{id}/{path}` | 清单所指文件，支持 Range 续传 |
| `/api/receiver` | 接收队列状态；POST 为既有接收工具使用的队列协议 |

使用已有 `episode_pull` 或 `episode_receive` 完成续传和 SHA-256 校验，按仓库接收工具说明部署。不要在自制接收器中猜测队列租期协议或仅按文件名认为传输完成。实时深度输出与 episode 录制不同，当前深度未自动写入既有训练数据格式。

### HTTP 状态码

| 状态码 | 典型含义 | 接收端处理 |
| --- | --- | --- |
| 200 | 数据返回或服务接受命令 | 继续检查业务字段与实际设备状态 |
| 202 | 设备管理请求已接受 | 观察服务状态，不能立即认定完成 |
| 400 | JSON、字段、坐标或命令格式不符合要求 | 修正请求，不反复发送同一无效负载 |
| 404 | 路径、相机 ID、文件不存在或网关版本不匹配 | 检查 BASE_URL 与版本 |
| 409 | 控制被门禁拒绝、运行状态冲突或队列冲突 | 读取 message 与状态原因 |
| 413 | 请求体超过网关限制 | 缩小负载，不通过控制请求上传大文件 |
| 503 | 服务不可用、驱动无数据或图像过期 | 标记当前数据无效，检查对应节点 |
| 504 或网络超时 | 控制响应未在限定时间返回 | 结果可能未知，查询状态后再处理 |
| 500 | 网关内部请求失败 | 保存请求与响应供排查 |

部分错误响应为空或为纯文本，不能假定所有非 200 响应都可直接 `response.json()`。浏览器跨 Origin 调用还可能受同源策略影响，普通 Python/C++ HTTP 客户端不受浏览器 CORS 机制限制；当前未提供通用 CORS 配置入口。

### 常见问题

| 现象 | 核查重点 |
| --- | --- |
| 能看到数值，但某些关节不动或无效 | 看每路 valid、age_ns、active_joint_indices；检查是否 left_only |
| ROS 能发现话题但没有图像 | 查看 QoS 是否错误地要求 reliable，检查发布端 local-only 与网络负载 |
| 深度 404 | 是否连到另一工作目录启动的旧控制台 |
| 深度 503 | 新版驱动是否运行、相机 serial 是否匹配、是否同 ROS 域、是否断流 |
| RGB 正常但 SDK 找不到深度相机 | SDK 主 serial 与 ASIC/USB serial 不同；本版本兼容两种标识 |
| 深度点返回 null | 该深度像素无有效测量；不要用 0 补成真实距离 |
| 控制服务存在但 start 被拒绝 | 当前模式、Quest、反馈、回中、握键、失败菜单或设备健康不满足门禁 |
| 升降位置存在但 valid=false | 检查反馈年龄、flags、guard、fault；状态连接不等于测量或运动就绪 |
| 发了 `/cmd_vel` 没有效果 | 当前底盘没有这个控制订阅；按第 8 或第 9 节选择实际接口 |
| 命令返回 true 但设备仍在过渡 | 区分请求接受与执行完成，继续检查 phase、reason 和实测反馈 |

## 12 机器人端提供接口的准备工作

本节由机器人端人员执行，接收端无需相机 SDK。此处是部署命令模板，不表示这些服务已经启动，也不要求接收方自行重启正在运行的控制程序。

### 构建与深度节点

```bash
cd /home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2
bash tool/build_depth_sdk.sh --prebuilt
bash tool/build.sh --packages-select luxi_interfaces luxi_device_adapters luxi_web_ctrl
```

确认物理相机与序列号、结束录制后，先停止该相机旧 RGB 服务和旧深度进程，再在独立终端启动统一对齐节点。先验证一路，再根据 CPU、USB、供电和网络负载决定是否同时启用三路：

```bash
sudo systemctl stop luxi-rgb-head_rgb.service
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 CAMERA_SERIAL=261043062235 ALIGN_TO_COLOR=true \
  bash Project/luxi_web_ctrl/scripts/run_depth_camera.sh head_rgb
```

左腕与右腕分别使用表中的 id 和 ASIC/USB serial。默认分辨率与频率可通过 `DEPTH_WIDTH`、`DEPTH_HEIGHT`、`DEPTH_FPS` 配置，对齐模式同时用于 RGB 和深度，需为两路共同支持的 profile。默认两路 640×480、15 Hz，JPEG_QUALITY 默认 90。`ALIGN_TO_COLOR=false` 保留只开深度的旧模式，不输出 rgbd。对齐节点停止后可用 `sudo systemctl start luxi-rgb-head_rgb.service` 恢复原 RGB 服务。独立 systemd 安装、开机项互斥及完整回退步骤见 [深度输出说明](DEPTH_OUTPUT_ZH.md)。对齐 service 启用后，设备管理白名单助手将该相机的启停操作映射到对齐 service，不再只操作旧 RGB 进程。

### 单独启动新版网关

可保留既有控制台，使用空闲端口启动本仓库的新网关。网关与现有机器人节点使用相同 ROS 域，HTTP 对外监听：

```bash
source /opt/ros/humble/setup.bash
source build/install/setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 \
  build/install/luxi_web_ctrl/lib/luxi_web_ctrl/web_ctrl --ros-args \
  -r __node:=receiver_gateway \
  -p host:=0.0.0.0 -p port:=18030 -p mock:=false -p base_enabled:=true \
  -p directory:=data/interface_gateway
```

这个命令只启动新网关，不启动第二套机械臂、底盘、Quest 或录制程序，也不自动使能电机。该目录只用于新网关自己的队列与文件访问；若需通过它查看现有 episode，应由提供方确认并配置实际采集目录，不可把示例目录当成现有数据位置。

HTTP 即使对外监听，ROS 仍可保持 local-only。只有接收方需要跨机 ROS 时，才统一安排选定发布节点与接收节点的 DDS 网络配置。不要为获取 HTTP 深度而重复启动整机控制栈。

需要网关常驻时可运行 `bash Project/luxi_web_ctrl/scripts/install_rgbd_gateway.sh 18030`，再执行 `sudo systemctl enable --now luxi-rgbd-gateway.service`。该独立网关使用相同参数和单独数据目录，不重启既有控制台或启动运动节点。

## 13 交付确认与实现待办

提供方交付时应明确以下信息，接收方据此完成接入，不依赖默认值猜测：

- 实际 IP、BASE_URL、网关版本、三个相机 ID 与物理位置。
- 哪些状态是真实硬件、哪些是 mock，哪些关节在当前运行模式中有效。
- 采用 HTTP 还是 ROS，若用 ROS 则给出域号、网络策略及匹配接口包。
- 深度节点是否常驻，分辨率、频率及是否经过多路同时运行验证。
- 控制的 shared、独立或 external 模式，谁持有控制权，以及正常停止和停放的约定。
- 是否要求自主双臂轨迹控制；若要求，需先实现并验收机器人侧统一桥接接口。

截至本版本，关节与设备反馈、RGB、深度及现有控制服务的接口已列明。`depth_output`、`rgbd_output`、`web_gateway` 测试均通过。三台物理相机在 640×480、15 Hz 配置下完成同时采集、成对 RGB/深度解码、内参和停止后 503 检查；Windows 接收端通过 18030 网关成功解码每路 3 组、共 9 组成对数据。三路对齐相机服务与网关已常驻运行并启用开机启动，旧 RGB 服务已停止并禁用开机启动。测试记录与限制见 [深度输出说明](DEPTH_OUTPUT_ZH.md) 的“当前实施状态”。此次未执行运动控制命令，未通过设备重启验证开机过程；短时结果不等于长期稳定性、相机外参标定或整机自主运动控制验收。

### 后续功能的边界

| 需求 | 已有基础 | 需要补充的实现 |
| --- | --- | --- |
| 接收端直接下发双臂轨迹 | 关节反馈、IK 与现有控制管理器 | 明确目标 schema、控制权仲裁、租期、限幅与状态确认的统一桥接 |
| 接收端发底盘速度和升降目标 | 两类驱动的本地 external 协议 | 跨机接入层、机器人本地时间戳转换、模式管理和故障撤销 |
| RGB 检测框查询对应深度 | 成对 RGB、对齐深度和图像网格内参已提供 | 接收端检测与有效深度统计、实际场景测距精度验证 |
| 多传感器同步数据集 | 各自时间戳、原始状态与帧 | 时钟标定、采样配对规则、质量标记及新的录制 schema |
| 基座坐标系点云或末端位姿 | 深度数据、关节角与模型 | 相机外参、坐标变换链和相应测量验证 |

### 接口实现位置

以下路径均相对于机器人仓库根目录，接口行为以对应部署版本为准：

| 路径 | 内容 |
| --- | --- |
| `Project/robot_interfaces/msg/ArmState.msg` | 16 路实际关节反馈定义 |
| `Project/robot_interfaces/msg/CameraFrame.msg` | 内部 RGB 和深度帧容器 |
| `Project/robot_interfaces/msg/RGBDFrame.msg` | RGB 与对齐深度、网格内参及原始设备帧号的整组消息 |
| `Project/luxi_web_ctrl/scripts/receive_rgbd.py` | HTTP 成对图像接收与 MIME 解码示例 |
| `Project/robot_interfaces/srv/Command.srv` | 通用控制服务定义 |
| `Project/luxi_web_ctrl/src/web_ctrl.cpp` | HTTP 路由、状态聚合与有效性 |
| `Project/control_manager/src/control_manager.cpp` | 双臂与整机控制状态机 |
| `Project/device_adapters/src/r680_adapter.cpp` | 底盘状态、独立及 external 控制 |
| `Project/device_adapters/src/lift_adapter.cpp` | 升降状态、握键与 external 任务 |
| `Project/device_adapters/include/luxi/external_control.hpp` | 本地外部指令租期与序号检查 |
| `Project/device_adapters/src/realsense_camera.cpp` | RGB 采集与 JPEG 发布 |
| `Project/device_adapters/src/realsense_depth.cpp` | 深度单位转换及 ROS 输出 |
| `Project/device_adapters/src/realsense_depth_backend.cpp` | SDK 隔离加载后端与相机识别 |
| `Project/robot_bringup/config/arms-teleop.json` | 关节顺序、夹爪映射、限位和速度配置 |
| `Project/robot_bringup/config/lift.env` | 升降传输与主机边界配置 |
