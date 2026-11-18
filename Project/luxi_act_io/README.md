# ACT HTTP / UDP 接入服务

独立包 `luxi_act_io` 提供两个 C++ ROS 2 节点，复用已有传感器，不打开相机、不操作 CAN、不使能电机。

| 服务 | 地址 | 用途 |
| --- | --- | --- |
| `luxi-act-rgb-http.service` | TCP 18040 | 三路 RGB JPEG、元数据及含关节状态的观测快照 |
| `luxi-act-action-udp.service` | UDP 18041 | 接收、重组、校验 ACT 预测动作块，发布独立 ROS 输入话题 |

机器人当前 IP：`192.168.2.199`。旧网关 18030 与旧关节控制 UDP 18034 不变。

## 执行边界

**本包是数据接入服务，不是动作执行器。** UDP 的 `accepted` 仅表示完整动作块通过传输校验并交给 ROS 发布器，不表示订阅者收到、执行器接受或电机到位。响应始终携带 `mode=ingress`、`execution=not_confirmed`。

输入被发布为 `/luxi/act/received_chunk`，类型 `std_msgs/msg/String`，内容为下述版本化 JSON。该话题使用 reliable、volatile、depth=1，不向后加入的订阅者重放旧动作。本包不发布 `/luxi/command`，不调用控制或使能服务。SDK 的 openarm_adapter 已加入会话绑定的 ACT 执行器（act_adapter.inc），实施控制权、限位/限速、反馈及观测门禁、夹爪控制和停止保持。只有显式 act_prepare → 归位到位 → act_begin 的同一会话能运动；其他会话动作被忽略。ACT 使用已补偿目标，不重复叠加旧位置补偿。

现有 ACT 标签是 `post_compensation_position_target`：右臂 7 关节、右夹爪、左臂 7 关节、左夹爪，共 16 维，绝对角度 rad。不得重复补偿。本包只检查有限值和协议结构，不宣称关节目标符合当前机械限位，也不负责底盘或升降控制。

## HTTP

```bash
curl http://192.168.2.199:18040/healthz
curl http://192.168.2.199:18040/v1/cameras
curl --fail http://192.168.2.199:18040/v1/cameras/head_rgb/rgb -o head.jpg
curl --fail http://192.168.2.199:18040/v1/cameras/left_wrist_rgb/rgb -o left.jpg
curl --fail http://192.168.2.199:18040/v1/cameras/right_wrist_rgb/rgb -o right.jpg
```

- `/healthz` 说明进程存活，不表示传感器正常。
- `/v1/cameras` 返回三路可用性、序号、原始尺寸、时间戳和新鲜度。
- `/v1/cameras/{id}/rgb` 返回原始 JPEG 字节。响应头含 `X-Frame-Sequence`、`X-Receive-Mono-Ns`、`X-Width`、`X-Height`。
- `/v1/observation` 返回 `multipart/form-data`：一个 `metadata` JSON 和 `head_rgb`、`left_wrist_rgb`、`right_wrist_rgb` 三个 JPEG 部分。三个缓存指针与状态在同一临界区取得，响应期间保持不变；JSON 描述的正是响应中的 JPEG。
- 图像缺失/过期、组间偏差超限、16 维状态缺失/失效时返回 HTTP 503，不以旧图像冒充实时观测。
- 默认图像有效期 200 ms、三图最大接收时间偏差 100 ms、状态有效期 150 ms，可用 ROS 参数修改。

快照是最新缓存的组合，**不是硬件同步曝光，也不提供训练级插值对齐**。本包原样保留分辨率、压缩与视野，不静默缩放。当前 ACT 与真机纯 RGB 服务均为 640×360，原生 30 Hz，无裁剪或缩放。100 ms 的接口默认偏差上限不是训练数据 40 ms 门限的替代。

`observation_mono_ns` 是快照中最旧图像/状态的机器人单调时钟时间，十进制字符串。UDP 请求应回传此值，不得用电脑本机时钟或发送时间替代。其他时间戳也属于机器人时钟域。

## UDP v1

采用每步 112 字节的小包，避免整个 60×16 动作块形成大型 UDP 数据报。网络字节序（大端），每个动作值为 IEEE754 float32。

| 偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0 | 8 | 固定 magic `LACTv1\0\0` |
| 8 | 16 | 非零 session UUID 原始字节 |
| 24 | 8 | uint64 chunk_id，从 1 开始，同一会话递增 |
| 32 | 8 | uint64 observation_mono_ns，来自 HTTP 原始观测 |
| 40 | 2 | uint16 step_index，从 0 开始 |
| 42 | 2 | uint16 step_count，1～60 |
| 44 | 4 | uint32 step_interval_us，当前固定 33333（约 30 Hz） |
| 48 | 64 | 16 个 float32 关节位置，rad |

Python 格式：`struct.pack('!8s16sQQHHI16f', ...)`。同一动作块中会话、ID、观测时间、步数和时间间隔必须相同。

- 接收端绑定源 IP、源 UDP 端口与 session；发送端在一个会话中保持同一个 socket。
- 允许同一块内乱序和内容相同的重传，不重复发布完整动作块。
- 新 chunk_id 替代尚未完成的旧块；旧 ID、矛盾重传、NaN/Inf、错误长度和字段被拒绝。
- 500 ms 内未收齐的块不发布；需要新 chunk_id 重试，不得更新旧块的观测时间来续命。
- 默认原始观测有效期 2000 ms，仅用于接入层测试；不是实际运动的有效期承诺。SDK 执行器实际要求观测晚于任务启动、严格递增且不超过 750 ms，任务心跳为 1 秒；首块等待 1 秒，收到块后的动作断流超时为 3 秒。
- 最多 32 个源会话，每个最多保留一个待完成块和一个已完成块。会话闲置 10 s 后回收。
- 每包返回 JSON ACK：`partial`、`accepted`、`duplicate` 或 `rejected`，带匹配的 session/chunk/step 标识。接收方确认不会消除网络丢包；客户端必须检查 ACK 和拒绝原因。

示例完整块响应：

```json
{"schema":"luxi_act_ack_v1","status":"accepted","received_steps":60,
 "mode":"ingress","execution":"not_confirmed","topic":"/luxi/act/received_chunk"}
```

ROS JSON schema `luxi_act_action_chunk_v1` 包含 `session_id`、`chunk_id`、`observation_mono_ns`、`received_mono_ns`、`step_interval_us`、`positions[N][16]`、`units`、`joint_order`、`semantics`、`source_ip`、`mode` 和 `execution`。64 位 ID/时间使用字符串。

`/luxi/act/ingress_status` 每秒发布一次计数（完整接收块、拒绝包、会话数）。不提供运动状态，因为本服务没有运动执行能力。

接口默认监听局域网所有 IPv4 地址，没有鉴权或加密；session UUID 用于去重，不是认证。UDP 可配置 `allow_ip` 限制源 IP。部署仅用于受信任的机器人网络。

## 电脑端调用

`scripts/act_io_client.py` 只依赖 Python 标准库：

```bash
python3 scripts/act_io_client.py --robot 192.168.2.199 --save-dir /tmp/act-observation
# 传输测试：发送一条当前反馈值，仍只进入接收话题，不执行电机动作。
python3 scripts/act_io_client.py --robot 192.168.2.199 --transport-test
```

程序调用：

```python
from act_io_client import get_observation, ActionSender
meta, jpeg_bytes = get_observation("http://192.168.2.199:18040")
# actions = ... 使用这一份观测做 ACT 推理并反归一化，形状 N×16 ...
sender = ActionSender("192.168.2.199")
try:
    ack = sender.send_chunk(actions, meta["observation_mono_ns"])
finally:
    sender.close()
```

持续推理应复用 `ActionSender`，不要为每个动作块创建新会话。客户端先发送整个块的小数据包，按 ACK 选择性重传缺失包，并在 400 ms 内要求完整块确认；不会逐步等待一次网络往返。吞吐量仍需在实际网络下测量。

## 构建、测试、服务管理

从 SDK 根目录执行，新增包不修改现有硬件包：

```bash
source /opt/ros/humble/setup.bash
source build/install/setup.bash
bash tool/build.sh --packages-select luxi_act_io
source build/install/setup.bash
ctest --test-dir build/packages/luxi_act_io --output-on-failure
bash Project/luxi_act_io/scripts/install_services.sh
```

安装脚本创建并启用两个 systemd 服务；只启动本包，不重启相机、控制管理器或电机驱动。ROS 2 保持 domain 0、localhost-only；HTTP/UDP 负责跨机接入。集成测试使用独立 domain 87、回环随机端口和模拟传感器。

```bash
systemctl status luxi-act-rgb-http luxi-act-action-udp
journalctl -u luxi-act-rgb-http -u luxi-act-action-udp -n 60 --no-pager
# 仅停止/取消这两个新增服务：
sudo systemctl disable --now luxi-act-rgb-http luxi-act-action-udp
```

## 抓取任务执行

电脑模型目录执行 `bash run_live_act.sh --execute --duration 30`。
先将双臂和夹爪归位到用户指定的 16 维初始角并保持支撑，再将升降置于 -64.543 mm；每次任务只归位一次。
归位速度 0.15 rad/s，ACT 执行上限 0.2 rad/s，60 步全部执行，30 Hz。
HTTP 18030 负责现有 SDK 控制接口、心跳和执行状态查询；HTTP 18040 取观测，UDP 18041 传动作。
Ctrl+C 保持双臂最后支撑并停止本任务拥有的升降；不自动回零、不自动关闭力矩。
不带 `--execute` 只做推理/传输。两种模式都不控制底盘移动。

驱动扩展位于 `Project/device_adapters/src/act_adapter.inc`，由 openarm_adapter.cpp 引入。
重编译完整执行链路还需构建 luxi_device_adapters 的 openarm_adapter，并在电机未使能时更新、重启 luxi-openarm-monitor。
状态在 HTTP `/api/state` 的 `arm_startup.act`，包含 session_id、stage、reason、chunk_id、target_step。
UDP ACK 不宣称执行成功；电脑端另外等待同会话的驱动 chunk_id 确认。
模拟硬件执行测试：`Project/device_adapters/test/test_act_execution.py`；全链路模拟夹具：`test/execution_fixture.py`。
已完成真机归位、升降及 ACT 10 秒/30 块执行验证；实际抓取成功率未验证。详情见电脑模型目录的推理说明.txt。

升降检查直接复用 `luxi::lift::Sample`：`stationary()` 要求无运动标志、速度绝对值 ≤2 rpm；运行检查用 `healthy()`（总年龄 ≤500 ms、源数据 ≤300 ms，保留故障/位置/传输检查）。归位开始及 ACT 开始用 `valid()` 的 250 ms 测量门限。电脑启动遇到暂时无效但健康的测量最多等 5 秒，不在等待期间运动。停止原因区分控制权、升降超时/运动/高度/故障。ACT 不订阅或检查底盘反馈，也不发送底盘命令。
