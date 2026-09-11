# RGB 网页接入

ARM64 MV3D RGB-D SDK 已下载并安装到 `device/HIK-EB435I/sdk-1.4`，版本为 1.4.0.3（250213）。下载来源、校验和及复现脚本见 [设备说明](../device/HIK-EB435I/README.md)。不再需要手动提供 SDK。

## 启动

```bash
./device/HIK-EB435I/download_sdk.sh
./Project/luxi_web_ctrl/scripts/install_rgb_camera.sh head_rgb 00DA8573794
./Project/luxi_web_ctrl/scripts/install_rgb_camera.sh right_wrist_rgb 00DA8573792
```

后续压力观察复现了 USB 主控制器故障，头部与右腕服务现已暂停并取消开机自启，详见 [USB 掉线与网页恢复记录](USB掉线与网页恢复.md)。左腕预留序列号为 `00DB1265540`，尚未在本次枚举中确认，不自动启动该路。相机服务通过 `device/HIK-EB435I/sdk-env.sh` 加载 SDK 和配套 MVCC 依赖，不修改系统全局库路径。

网页启动已打开 RGB 功能。链路：SDK → `hikrobot_camera` → `/luxi/cameras/<camera_id>` → `/api/cameras/<camera_id>` JPEG → 浏览器预览。错误状态通过 `/luxi/cameras/<camera_id>/status` 进入网页。超过 1 秒未更新的图像会隐藏，取帧失败后每 3 秒尝试重新打开。

```bash
systemctl status luxi-rgb-head_rgb.service luxi-rgb-right_wrist_rgb.service
sudo journalctl -u luxi-rgb-head_rgb.service -n 30 --no-pager
lsusb -d 2bdf:0001
lsusb -t
```

## 验证范围

已完成 ARM64 库架构、依赖解析及 ABI 校验，SDK 初始化、枚举、打开设备与读取参数成功。曾识别头部 `00DA8573794` 与右腕 `00DA8573792`。头部相机已返回真实 640×360 RGB 图像并在网页显示，帧序号持续更新。右腕曾收到图像，但随后 USB 断连，当前显示过期；内核曾报 `error -71`，SDK 报 `0x80060030`（USB 写入失败），需要保持接线和供电稳定。

隔离测试：相机库 3 项单元测试、假 SDK 恢复取帧与网页 JPEG 显示、预览开关及过期隐藏。运行（先加载 ROS 和 build/install 环境）：

```bash
python3 tool/test_rgb_web.py
```

该测试使用独立 ROS 域 88、端口 18022，不向实机网页发布模拟数据。
