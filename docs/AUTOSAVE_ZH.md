# 数采数据自动保存到本机

配置日期：2026-09-18。机器人端：`http://192.168.3.179:8000`；接收电脑：当前 `robot` 工作站。

## 日常使用

本机开机并联网后，后台接收服务自动运行。X5 上按 A 或网页“开始录制”，
结束时再按 A 或“成功并保存”。每条录制完成后自动下载到本机，校验完整并
落盘后释放对应的 X5 RAM 副本，然后编号归档。无需每次手动运行接收命令。

**固定保存目录：`/home/robot/datasets/luxi/episodes/`**

目录命名示例：

```text
episode_000001_20260918_162242/
episode_000002_20260918_163000/
```

格式为 `episode_六位流水号_录制开始日期_时间`，时间使用北京时间。流水号从
本流水线开始接收的数据起算，按归档顺序递增；服务重启后继续，不因重试重新
编号。历史数据目录仍保留原位置。失败/取消录制也保留，状态在索引中标明。

每个编号目录包含完整原始数据：`bag/`、`samples.jsonl`、`manifest.json`、
机器人/相机/会话配置，以及新增的 `local_record.json`。双臂、三路图像及
录制中存在的底盘话题一起保存。原始 manifest 和 episode ID 不修改。

| 文件 | 用途 |
| --- | --- |
| `/home/robot/datasets/luxi/index.csv` | 总表：编号、目录、原 ID、录制时间、状态、任务、样本数、大小、各相机和底盘消息数 |
| `/home/robot/datasets/luxi/last_saved.json` | 最近一次完成本地归档的记录 |
| `/home/robot/datasets/luxi/index.sqlite3` | 持久编号数据库，请勿删除或手工修改 |
| `/home/robot/datasets/luxi/.incoming/` | 接收暂存及中断恢复区，请勿在接收期间移动/删除 |

日志出现 `SAVED #000001 ...` 表示该条已完成编号归档。网页上的接收目标可能
显示 `.incoming`，这是传输暂存目录；最终编号数据在 `episodes/`。

## 服务管理

```bash
# 启动 / 暂停
systemctl --user start luxi-dataset-autosave.service
systemctl --user stop luxi-dataset-autosave.service

# 查看运行日志（按需手动执行）
journalctl --user -u luxi-dataset-autosave.service -f

# 取消 / 恢复自动启动
systemctl --user disable --now luxi-dataset-autosave.service
systemctl --user enable --now luxi-dataset-autosave.service
```

设置文件：`/home/robot/luxi-dataset-receiver/autosave.json`。X5 IP 变化时修改
`board_url`，然后 `systemctl --user restart luxi-dataset-autosave.service`。
编号数据库和数据目录应整套保留/备份；不要只迁移其中一项。

如需换另一台电脑接收，先停止本机自动服务，再启动另一台电脑的接收器。
同一时刻只有一个接收端拥有队列。已保存的数据不会自动跟随接收电脑迁移。

## 断线与数据保护

- 接收服务异常退出后约 5 秒重启；X5 尚未上线时持续重试。
- C++ 接收器负责断点续传、每个文件的大小/SHA-256 校验、落盘和远端释放确认。
- 只有下载校验并持久保存后才释放 X5 内存。编号归档在同一磁盘上原子移动。
- 归档前再次核对本地文件；中断恢复沿用已有编号。遇到同 ID 内容冲突时保留
  数据并报错，不覆盖已有记录。
- X5 重启或掉电会丢失仍未传出的 RAM 数据。采集时本机必须开机且网络可达；
  关机前等记录完成接收和归档。单条录制仍须容纳在 X5 的 RAM 缓冲中。
- 保存的是完整原始数据；LeRobot 转换另行执行。现有转换器已支持编号目录，
  `--source` 可直接传入 `episodes/episode_000001_...`。

本次配置未运行额外测试、构建或事后验证。后台运行时的文件校验属于正常
保存流程，用于决定何时可以释放 X5 副本。
