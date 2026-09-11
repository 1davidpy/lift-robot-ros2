# LUXI 局域网控制台

参考原 `openarm_teleop/nodes/dora-openarm-data-collection-ui/.../templates/root.html` 的设备检查、相机预览和 episode 采集流程，实现独立 C++ ROS 2 HTTP 网关。浏览器使用原生 HTML/CSS/JavaScript，无 CDN、Node 构建或 Python 后端。原 `data_server` 已迁至本包的 `web_ctrl`，录制器与文件格式保持兼容。

## 页面与操作

- Quest：有效报文速率、左右追踪状态。收到有效遥操作报文才算连接，不用 ping 或头显 Wi-Fi 状态代替。
- 双臂：实际反馈是否新鲜、16 路反馈有效性、归位/对齐/遥操作状态；使能、暂停返回。
- R680：反馈连接、平移速度、电压、FlagStop、使能状态；启用和停车。运动仍由 Quest 左握持键及双摇杆控制，网页不持续发送速度。
- 三路 RGB：头部、左腕、右腕预览，尺寸、帧号、接收年龄与时间戳质量。过期画面隐藏，可单独暂停预览。
- 录制：任务说明、开始、成功/失败保存、取消并保留、样本数、质量检查、数据目录和磁盘空间。
- 数据清单：最近 30 个已提交 episode 的 manifest 链接；文件继续通过原 HTTP 下载接口及 `episode_pull` 回传。

```mermaid
flowchart LR
 B[局域网浏览器] -->|HTTP 8000 / 令牌| W[luxi_web_ctrl / C++]
 Q[Quest 应用] -->|UDP 5006| R[quest_receiver]
 R -->|ROS 2 状态| W
 D[双臂 / R680 / 三相机] -->|状态与图像| W
 W -->|ROS 2 服务| C[控制管理 / 底盘驱动 / 录制器]
 C --> F[本地 episode]
 F -->|文件 / manifest| W
```

按钮不绕过控制器门禁，成功与否以服务返回为准。启用双臂会触发归位；“暂停 / 返回”包含运动，不是硬件急停。服务失联后页面禁用操作并隐藏旧图像；HTTP 超时不自动重试运动指令。

## 编译与启动

在仓库根目录：

```bash
./tool/build.sh
# 模拟全系统；与开机服务使用不同 ROS 域和 HTTP 端口
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ./tool/run.sh mock:=true base_enabled:=true \
  http_host:=0.0.0.0 http_port:=18000 quest_port:=15006 \
  control_token:=replace-with-your-test-token minimum_free_gib:=0.2 \
  "directory:=$PWD/data/mock"
```

访问 `http://板端IP:18000`，输入此命令设置的令牌。模拟图像和反馈始终标为 MOCK。测试 Quest 应用应向 UDP 15006 发送；真实开机部署使用 UDP 5006。不要将两个模式接到相同 ROS 域。

## 开机自动运行

普通用户在仓库根目录执行一次：

```bash
./Project/luxi_web_ctrl/scripts/install_service.sh --hardware
```

安装 systemd 系统服务，无需桌面登录：

| 服务 | 行为 |
|---|---|
| `luxi-web.service` | 立即启动，之后每次开机启动；监听 `0.0.0.0:8000`。网页即使没有 CAN/Quest 也可打开 |
| `luxi-quest.service` | 随网页启动，独立监听 `0.0.0.0:5006/UDP`；不依赖 CAN，异常退出自动重启；网页停止/重启时同步停止/重启 |
| `luxi-robot.service` | 设置为下次开机启动，本次安装不启动硬件；先检查/配置 KH CAN，再启动真实双臂、R680 和录制器，复用常驻 Quest 接收节点，初始未使能、未录制 |

当前实机开机配置关闭三相机，因为正式海康 SDK 尚待验证。网页会显示“已关闭”，不会用假图代替；升降平台尚未接入。KH 缺失时机器人服务失败，网页继续运行并显示无反馈。接好后可手动 `sudo systemctl start luxi-robot.service`，不要同时运行手动的机器人实例。

只需要网页与 Quest 接收自启，可用 `--web-only`；此选项不修改已有硬件服务启用状态。明确关闭硬件自启用 `sudo systemctl disable luxi-robot.service`。

局域网浏览器访问：

```text
http://板端局域网IPv4:8000
```

查 IP：`ip -4 addr`。令牌在板端本地读取（不要将真实令牌提交 Git）：

```bash
cat log/web-control.token
```

令牌文件随机生成、权限 0600，不进入 Git，不在启动日志中输出。局域网默认只读；输入令牌即可使能/停车/录制。浏览器刷新后需重新输入。网页无互联网资源依赖；仅用于受信任局域网，HTTP 未加密。若网络防火墙启用，需放通受信任网段 TCP 8000 和 Quest 的 UDP 5006。

```bash
systemctl status luxi-web.service luxi-quest.service luxi-robot.service
journalctl -u luxi-web.service -u luxi-quest.service -u luxi-robot.service -n 80 --no-pager
```

服务统一 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=1`：ROS 在板端进程间通信，浏览器和 Quest 仍走局域网 HTTP/UDP。手动 `tool/start_robot.sh --hardware` 会复用已运行的网页，不重复占用端口。`tool/start_robot.sh --mock` 在开机网页已运行时自动改用 ROS 域 76 / HTTP 18000 / UDP 15006。原 `tool/run.sh` 默认附带网页，如服务已启动须传 `web_enabled:=false quest_enabled:=false` 并使用同一 ROS 域。

停止之前先结束录制、底盘停车、双臂返回并确认 disabled，再执行：

```bash
sudo systemctl stop luxi-robot.service
sudo systemctl stop luxi-web.service
```

卸载自启：`sudo systemctl disable luxi-web.service luxi-robot.service`。不要在录制中直接关机。

## 接入相机

完成海康 SDK/序列号验证后，用 `sudo systemctl edit luxi-robot.service` 覆盖 `ExecStart`（先用空 `ExecStart=` 清除旧值），保留原命令其余参数并改为 `no_cameras:=false require_cameras:=true sdk_library:=绝对SDK路径`，必要时配置 `LD_LIBRARY_PATH`。同时修改 `scripts/run_web.sh` 的 `no_cameras:=false`，使页面配置与机器人一致；重启两项服务前先安全结束遥操作。不要只改网页配置而未真正启动相机。

## 代码与接口

| 位置 | 用途 |
|---|---|
| `src/web_ctrl.cpp` | C++ ROS 2 订阅/服务调用、状态新鲜度、权限校验、静态资源、文件回传 |
| `web/index.html` / `style.css` / `app.js` | 响应式中文操作台，桌面和手机布局 |
| `scripts/` | systemd 安装、独立网页启动、可选硬件 CAN 准备 |
| `test/http_probe.cpp` | C++ HTTP/ROS 集成验证：鉴权、异常请求、设备过期和图像过期 |

保留 `/api/state`、`/api/cameras/{camera_id}`、`/api/episodes`、`/api/episodes/{id}/manifest`、`/api/files/{id}/{path}` 和 `POST /api/{control,base,episode}`。控制请求使用 `Authorization: Bearer ...`，JSON 字段 `command` 与 `detail`。新增 state 字段 `quest`、`cameras`、`system` 和各节点 `fresh`。

网页以单次请求完成后再轮询的方式刷新，状态约 2 Hz、图像最多约 3 Hz，避免慢网络请求堆积；录制器按其原采集频率独立运行。图像预览是 JPEG 快照，不宣称硬件曝光同步。最终仍只录制并回传，不训练。

## Quest 填写 IP 后如何接入

配套 Quest 遥操作 APK 与板端处于可互通局域网，目标 IP 填板端地址（当前 `192.168.3.179`），UDP 端口为 `5006`。应用保持发送时，网页显示接收频率和左右追踪状态；不需要连接 CAN 或使能机器人。这个协议是单向 UDP JSON，不提供 TCP/WebSocket/自动发现握手；普通网页和 Air Link 不能替代配套 APK。

网页服务启动即拉起接收服务，开机无需另开终端。检查命令：

```bash
systemctl is-active luxi-web.service luxi-quest.service
ss -lun 'sport = :5006'
sudo journalctl -u luxi-quest.service -n 30 --no-pager
```

常驻接收运行时，不再手动运行第二个 `quest_receiver`。独立 ROS launch 默认 `quest_enabled:=true`，复用常驻节点时需设为 false。域 76 的 mock 快捷启动仍自行监听 15006，不占用实际 Quest 的 5006。
