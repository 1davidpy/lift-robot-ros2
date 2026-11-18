# 遥操手眼标定录包

准备日期：2026-11-03。专用录包与普通 A 键 episode 相互独立；本流程不要求按 A 开始普通采集。

开始：`sudo systemctl start luxi-handeye-record.service`

停止并完成写盘：`sudo systemctl stop luxi-handeye-record.service`

查看状态：`systemctl status luxi-handeye-record.service`

查看日志：`journalctl -u luxi-handeye-record.service -n 30 --no-pager`

当前目录：`cat data/handeye_sessions/latest_session.txt`

服务不设为开机自动启动，不自动使能，不发送任何运动命令。每次启动生成独立 session；使用 flock 避免重复录制。录包写入板端持久磁盘，正式目录以 handeye_ 开头；preflight_ 为独立校验包。每个 SQLite 分卷最大约 1 GiB，剩余空间少于 2 GiB 时尝试正常停止并写盘；开始前至少需要 5 GiB 可用空间。它不受普通 episode 自动传输清理流程管理。

内容：三路 JPEG CameraFrame、三路真实 color CameraInfo、双臂实测状态与控制指令、Quest、底盘状态、升降 state/status/events/UART、故障及存在时的 TF。此配置不录制深度，避免无关带宽负担。相机内参以 bag 和 snapshot/*_camera_info.json 为准；snapshot/source_config 和 installed_config 是原始配置副本，可能含旧相机 profile。

目录：bag/ 保存完整原始时间线；lift_timeline.csv 保存逐条升降实测值、有效性和段号；height_segments/ 保存逐段 JSON；height_segments.json 是总索引。稳定高度段如 0001_h+173.2mm；升降过程中标记为 transition，反馈异常标记在 CSV。各段索引引用同一原始 bag，未将原始消息物理复制到每个高度目录。bag 内可能有多个 1 GiB 分卷，与高度分段是不同概念。

高度稳定判据为有效反馈窗口至少约 0.8 秒、窗口位置变化不超过 0.5 mm、速度不超过 2 rpm；稳定高度锚点变化超过 1 mm 后建立新段。它属于自动索引，后续标定仍需筛选清晰图像、有效关节反馈和实际静止区间。不得把升降到位自动等同于双臂已静止。

时间：bag 使用 ROS 接收时间；原始消息保留主机单调时间与相机设备时间。按图像时间匹配实测关节和升降反馈，不按命令时间匹配；不直接相减不同时间域。TF 当前没有发布者，因此需要根据快照模型及实际安装关系计算运动学；跨高度或底盘运动的数据不能直接当成固定机械臂基座下同一固定标定板。

操作：固定底盘和棋盘格；打开 Quest 发送应用，确认状态后按既有 X 长按流程进入遥操。每个高度采集多种旋转方向，每个姿态停稳至少约 2 秒；左右腕相机必须各自清晰看到完整棋盘。改变升降后停稳，再继续采集下一组。若某只腕相机看不到目标，先调整可见性，录包不能补救视野缺失。

标定板为 Q12-150-10，11×8 内角点，每格 10 mm（0.010 m），用户已提供规格，保存在 tool/handeye_target.json。之前的 8×6 检测属于局部匹配，不用于正式标定。左右腕序列号与物理位置仍需确认。停止后检查 recording_result.json 和 bag metadata，再传输归档；不能只看服务 active 判断数据完整。
