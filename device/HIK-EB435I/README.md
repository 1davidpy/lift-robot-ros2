# HIK-EB435I ARM64 SDK

运行版本：`SDK_Mv3dRgbd_Linux_V1.4.0.3_250213`，运行目录 `sdk-1.4`。SDK 包、头文件、文档及许可证保存在本目录；二进制和压缩包已加入 gitignore。`source.json` 记录公开下载来源及本次下载的 SHA-256。

## 获取与运行

```bash
./device/HIK-EB435I/download_sdk.sh
source device/HIK-EB435I/sdk-env.sh
./Project/luxi_web_ctrl/scripts/install_rgb_camera.sh head_rgb 00DA8573794
./Project/luxi_web_ctrl/scripts/install_rgb_camera.sh right_wrist_rgb 00DA8573792
```

下载脚本从供应商公开资料目录获取 EB435i 完整包，提取 Linux SDK，并检查文件哈希及 AArch64 ELF 架构。`sdk-env.sh` 设置相机服务专用库路径和 SDK 环境变量，不修改全局 `/etc/profile` 或系统 MVS 安装。补齐了 MVCC 库的版本符号链接。

普通用户访问权限及防止相机自动休眠：

```bash
sudo install -m 644 device/HIK-EB435I/99-luxi-hikrobot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bdf --attr-match=idProduct=0001
```

网页为 `http://192.168.3.179:8000`。只有收到新鲜 RGB 数据才显示图像。

## 独立诊断

执行独立探测前停止相应相机服务，避免同时打开同一设备。探测默认只枚举、打开、读取参数；追加 `stream` 才配置 RGB 模式并读取五帧。

```bash
source device/HIK-EB435I/sdk-env.sh
c++ device/HIK-EB435I/probe.cpp -I "$HIK_SDK_ROOT/Include" \
 -L "$HIK_SDK_ROOT/Libs/aarch64" -lMv3dRgbd \
 -Wl,-rpath-link,"$HIK_SDK_ROOT/MVCC/Libs/aarch64" \
 -Wl,-rpath-link,"$HIK_SDK_ROOT/Libs/aarch64" -o build/hik-sdk-probe
build/hik-sdk-probe 00DA8573794 stream
```

SDK 版本查询、初始化、设备枚举和打开曾全部成功；识别到 `00DA8573794`（头部）和 `00DA8573792`（右腕）。接口结构已与此 SDK 头文件校验：Image=56、Frame=584、Param=288 字节。

取帧排查：`0x80060030` 表示 SDK 写 USB 失败；内核 `error -71`、USB disconnect 及持续重新枚举发生在 USB 层，应检查数据线、接口与供电。SDK 缺失与物理连接问题是不同故障。

## 实机验证结果

头部 RGB 网页预览通过：640×360，连续 10 秒均为新鲜帧，帧序号从 982 增至 1198（约 24 fps）。截图 `build/web-rgb-live.png`。右腕曾收到图像，随后 USB 断开；相机服务保持自动重连。OpenArm 16/16 反馈正常。相机单元测试和隔离网页图像回归通过。

## 当前运行限制

后续网页连续测试发现 USB 主控制器两次停止响应；仅头部采集时也出现断连。为保持 CAN 硬件在线，已暂停两路 RGB 服务并取消其开机自启。SDK 适配保留，待相机连接与供电稳定后逐台复测。详见 `docs/USB掉线与网页恢复.md`。
