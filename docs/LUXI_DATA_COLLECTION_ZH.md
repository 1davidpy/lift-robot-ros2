# LUXI 数据采集简明指南

更新：2026-09-29。适用：X5 + Quest + 双臂 + R680 底盘 + UART 升降平台 + 三台 RealSense；当前采用 X 统一使能与停止。

**数据先暂存在 X5 内存中，传输完成前不要关机或重启。先启动接收电脑，再开始录制。**

## 1. 启动机器人端

登录 X5：`ssh sunrise@192.168.3.179`，执行：

```bash
cd ~/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2

# 首次设置一次：允许 UART 升降参与控制，重启后保留；实际使能仍由 X 发起。
sed -i 's/^LIFT_ALLOW_MOTION=.*/LIFT_ALLOW_MOTION=true/' \
  Project/robot_bringup/config/lift.env

sudo systemctl start \
  luxi-web.service luxi-quest.service \
  luxi-openarm-monitor.service luxi-base.service luxi-lift.service \
  luxi-rgb-head_rgb.service \
  luxi-rgb-left_wrist_rgb.service \
  luxi-rgb-right_wrist_rgb.service \
  luxi-program.service
```

- 控制网页：**http://192.168.3.179:8000**；Quest 发送目标：**192.168.3.179:5006**。
- 命令启动软件，不主动发起机械臂使能或升降运动；已运行的服务保持运行。升降固件若保存了 `BOOT_HOME=1`，上电/复位仍可能自行回中。
- 升降使用 USB UART，115200、8N1，固定 USB 设备路径由 `lift.env` 配置；不再使用 CH4 CAN 控制。不要同时打开串口助手或另一套遥操作程序。
- 采集前确认：三路图像更新、双臂全部 16 个关节反馈有效、底盘反馈正常、升降位置有效且无故障/超时、Quest 在线且左右控制器均非 `INVALID`。统一使能需要这些设备均就绪。

正式采集前确认相机实际安装位置与标签一致：

| 位置 | 型号 | 序列号 |
| --- | --- | --- |
| 头部 | D455f | 261043062235 |
| 左腕 | D435i | 254843065015 |
| 右腕 | D435i | 261443062039 |

## 2. 启动接收与自动转换

### 当前 robot 工作站：推荐日常流程

本机已配置自动接收、编号归档和 LeRobot v2.1 转换服务。在**接收电脑**执行（不用 sudo）：

```bash
systemctl --user start \
  luxi-dataset-autosave.service \
  luxi-dataset-autoconvert.service

# 查看保存和转换进度；Ctrl+C 只退出日志查看，不停止后台服务。
journalctl --user \
  -u luxi-dataset-autosave.service \
  -u luxi-dataset-autoconvert.service -f
```

这两个服务已配置自动启动；**不要再同时运行 `receive.sh` 或第二个接收器**。

| 用途 | 固定目录 / 文件 |
| --- | --- |
| 原始编号数据 | `/home/robot/datasets/luxi/episodes/` |
| 原始数据总表 / 最近保存 | `/home/robot/datasets/luxi/index.csv`、`last_saved.json` |
| LeRobot v2.1 数据 | `/home/robot/datasets/lerobot-v21/` |
| 转换总表 / 最近完成 | `/home/robot/datasets/lerobot-v21/index.csv`、`last_converted.json` |

目录示例：`episode_000001_20260929_162242/`，格式为六位流水号加录制开始日期、时间（北京时间）。编号按归档顺序递增，重启后接续。原始目录与转换目录同名。

### 换另一台局域网电脑接收

**打开网页不代表数据会保存到那台电脑，必须在目标电脑运行接收器。**

先在当前工作站停止后台流水线：

```bash
systemctl --user stop \
  luxi-dataset-autosave.service \
  luxi-dataset-autoconvert.service
```

新电脑按第 6 节安装，再执行：

```bash
~/luxi-receiver/build/receiver/episode_receive \
  http://192.168.3.179:8000 \
  "$HOME/datasets/luxi" \
  --release-source
```

第二个参数可改成其他硬盘路径。保持终端运行，并在网页确认接收端在线。手动接收保存为 `目标目录/episode-*`；**此安装只包含原始数据接收，不自动提供当前工作站的编号和 LeRobot 转换流水线**。

同一时刻只允许一个接收端；手动接收器用 `Ctrl+C` 停止，异常退出后占用约 20 秒过期。已经保存到旧电脑的数据不会自动搬家。需要恢复当前工作站接收时，先停止其他接收器，再启动上面的两个用户服务。

## 3. 完成一次采集

1. 确认机械臂、底盘、升降运动路径无障碍，升降温度正常，物理停止控制可触及。戴好 Quest，保持发送程序在前台；摇杆归中、中指握键释放。
2. **长按 X 约 1.5 秒后松开**，启动统一使能流程。双臂先回启动姿态；进入 `aligning` 后保持手柄稳定，完全按下两个食指扳机，等待进入 `teleop` 且底盘、升降均已就绪。
3. **按 A**，或点击网页“开始录制”。确认状态为录制中、样本数增长后开始示范。食指扳机控制夹爪；摇杆控制底盘；中指握键控制升降。
4. **再按 A**，或点击“成功并保存”，等待录制器回到 `idle`。保存完成不会自动停用机器人。
5. 放下夹持物，摇杆归中、握键释放，然后**短按 X**。系统先停止底盘和升降并确认停稳，再执行双臂约 2 秒的回位指令序列，同时打开夹爪，随后撤销双臂扭矩。**升降保持当前高度，不回中；不等待机械臂反馈实际到达 home。**调度延迟可能延长指令序列，设备故障或 B 会中断回位并保留扭矩。
6. 确认双臂已禁用、底盘和升降已停止。接收端出现 `SAVED #xxxxxx` 表示本机编号归档完成；手动接收器以 `Verified ... (RAM copy released)` 为准。确认该条原始数据已落盘后，才可关闭 X5。
7. 需要训练格式时，继续等待 `CONVERTED #xxxxxx` 或转换索引状态为 `complete`。仅原始传输完成不等于转换完成。

**当前 X 停止采用旧版按时间撤销扭矩的行为，不能保证实际到位；操作者须确认双臂在当前升降高度下可安全失去扭矩。X 回位会产生运动，不能代替物理急停。**

按键与操作：

| 按键 / 输入 | 当前功能 |
| --- | --- |
| X 长按约 1.5 秒 | 从禁用/停止状态启动双臂、底盘、升降的统一使能流程 |
| X 短按 | 停底盘和升降；双臂执行旧版定时回位、打开夹爪后撤销扭矩。停止但仍保持扭矩时，短按可重新请求该流程 |
| Y | 当前统一模式中保留，不再切换底盘使能 |
| 左摇杆 | 底盘前后、左右平移；归中停止 |
| 右摇杆左右 | 底盘转向；归中停止 |
| 左中指握键 | 按住下降，松开停止 |
| 右中指握键 | 按住上升，松开停止；两侧握键同时按下也停止。正常松开不撤销控制权限 |
| 左右食指扳机 | `aligning` 时用于双臂对齐；`teleop` 时分别控制夹爪 |
| A | 开始录制 / 成功结束；失败菜单中确认失败原因 |
| B | 停止整机遥操作：底盘、升降停止，双臂保持扭矩；不自动回位、不撤销双臂扭矩。录制中还打开失败菜单，左摇杆上下选原因、A 确认保存失败记录 |

B 是停止键，不是使能开关，也不是物理急停。不需移动底盘或升降时，保持摇杆归中、握键释放。

## 4. 原始数据与保存完成判据

- X5 临时队列：`/run/luxi_dataset/episodes/episode-*`。
- 一条录制结束后才开始传输；逐文件校验大小、SHA-256 并持久落盘，接收确认后才释放对应的 X5 RAM 副本。
- 本机自动流水线先下载到 `/home/robot/datasets/luxi/.incoming/`，再编号归档至 `episodes/`。未完成传输保留 `.partial-*`，网络恢复后续传。
- `Verified` 表示原始传输完整；`SAVED` 表示本机编号归档完成；两者均不代表示范质量合格。

每个原始 episode 包含：

| 文件 | 内容 |
| --- | --- |
| `samples.jsonl` | 双臂 16 维状态、速度、实际下发动作、各自时间戳与图像索引；右臂含夹爪 8 维在前，左臂含夹爪 8 维在后 |
| `bag/` | ROS 原始消息、实际三路 RGB 图像；底盘和升降作为独立话题保存 |
| `manifest.json` | 状态、质量标记 QC、样本数、话题计数、文件校验信息 |
| `session.json`、`robot_config.json`、`camera_config.json` | 采集配置与元信息 |
| `local_record.json` | 本机自动归档新增的编号、原 episode ID 等信息 |

底盘记录 `/luxi/base/state`、`applied`、`can_rx`、`status`，包含实测速度和速度命令。升降记录 `/luxi/lift/state`、`events`、`uart_rx`、`status`，包含当前位置、目标高度、电机转速 `speed_rpm`、故障/有效性、接收时间及设备时间。UART 模式下没有新的升降 CAN 帧。升降反馈标称约 10 Hz；转速 rpm 不是升降直线速度。

原始 `samples.jsonl` 的 16 维向量不直接包含底盘或升降，需结合 bag 中对应话题。保存或搬运时保留整个目录；迁移本机编号流水线时同时保留索引数据库和配置。

三路图像为 640×360、约 30 FPS 的 RGB，不含深度。**RAM 缓冲共 2 GiB，预留 0.25 GiB；单条录制必须容得下，录制中不会边录边传。未传出的数据遇断电/重启会丢失。**

## 5. LeRobot v2.1 与时间对齐

当前工作站自动流程：

`录制结束 → 原始下载校验/落盘 → 释放 X5 RAM → 编号归档 → 自动转换`

- 每个编号目录转换为一个独立的单条 LeRobot v2.1 数据集，内部 episode 索引从 0 开始；不是不断追加到同一个训练集。
- 输出包含 Parquet、三路 RGB MP4、元数据和统计信息。当前 `observation.state` / `action` 为 **19 维：双臂含夹爪 16 维 + 底盘 vx/vy/wz 3 维**。底盘部分分别是实测速度和命令速度，不是位置坐标。
- **当前转换器尚未将升降加入 LeRobot state/action，也未为升降生成对齐列。升降原始信息保存在原始 bag，不能把当前 19 维输出当作包含升降的完整训练格式。**
- 以 X5 主机单调时钟构造 30 FPS 时间网格，选择每个时刻之前最近的双臂样本、图像和底盘反馈/命令，保留来源时间戳。不是相机硬件同步，不使用未来数据或插值冒充新测量；相邻导出帧可能复用源样本。
- 当前最大样本年龄限制：双臂 150 ms、图像 200 ms、底盘状态/动作 100 ms。缺失、过期或时间异常会使转换报错；该阈值是筛选上限，不代表实际对齐误差总是这么大。
- 仅自动转换 `accepted` 且 `qc=[]` 的录制。失败/取消数据仍保存原始目录；转换失败也不删除原始数据。
- 完成判据：索引状态 `complete` / 日志 `CONVERTED`，以及正式输出目录中的 `conversion_manifest.json`（`conversion_complete=true`）。`.partial-*` 或仅存在 `meta/info.json` 不算完成。

查看最近数据及转换状态：

```bash
cat /home/robot/datasets/luxi/last_saved.json
cat /home/robot/datasets/lerobot-v21/last_converted.json
cat /home/robot/datasets/lerobot-v21/index.csv
```

比较记录编号，避免把上一条转换结果当作最新一条。转换日志在 `/home/robot/datasets/lerobot-v21/.logs/`。示范质量、相机标签和训练适用性仍需人工检查。

## 6. 新接收电脑首次安装（仅原始数据接收）

在目标 Ubuntu/Debian 电脑执行；Windows 可使用 WSL。该接收器无需 ROS 或相机 SDK。

```bash
mkdir -p ~/luxi-receiver/tool ~/luxi-receiver/Project
X5_SRC=sunrise@192.168.3.179:/home/sunrise/Desktop/LUXI_Lift_Robot/luxi_lift_ros2/lift_robot_ros2

scp -r "$X5_SRC/tool/receiver" ~/luxi-receiver/tool/
scp -r "$X5_SRC/Project/data_recorder" ~/luxi-receiver/Project/

sudo apt update
sudo apt install g++ pkg-config libcpp-httplib-dev libssl-dev nlohmann-json3-dev
cd ~/luxi-receiver
bash tool/receiver/build.sh
```

安装后执行第 2 节新电脑接收命令。WSL 保存到 Windows 磁盘时，将目录改为 `/mnt/c/...` 等实际路径。自动编号与 LeRobot 转换需另行部署，不能仅靠安装这个接收器获得。
