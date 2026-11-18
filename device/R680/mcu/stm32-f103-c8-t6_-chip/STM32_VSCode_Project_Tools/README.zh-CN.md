# STM32 VS Code 工程工具包

这个目录是可随仓库迁移的 VS Code 扩展工具包。它给资源管理器中 `Projects/<工程>` 目录的右键菜单增加：

- `STM32: 编译 Debug + Release`
- `STM32: 选择固件并烧录`

扩展会读取仓库根目录的 `CMakePresets.json`，因此不把工程名、构建目录或 ST-Link 序列号写死在插件中。

## 目录内容

```text
STM32_VSCode_Project_Tools/
├── extension/                 # VS Code 扩展完整源码
├── package/                   # 可直接安装的 VSIX
│   └── archive/               # 历史安装包，仅供回退
├── docs/                      # 编译、烧录和排错说明
├── install.ps1                # 安装到指定 VS Code Profile
├── uninstall.ps1              # 从指定 Profile 卸载
├── package-extension.ps1      # 根据源码重新生成 VSIX
└── verify.ps1                 # 本地自检
```

当前推荐安装包是 `package/stm32-project-tools-0.1.2.vsix`。`archive` 中的版本不是默认安装对象。

## 新电脑安装

先安装以下外部软件：

1. VS Code 1.85 或更高版本。
2. STM32CubeCLT、STM32CubeIDE，或能够提供 CMake、Ninja、GNU Arm Embedded Toolchain 和 STM32CubeProgrammer 的 STM32Cube 工具包。
3. ST-Link USB 驱动。

安装扩展本身不需要 Node.js。打开 PowerShell，在仓库根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_VSCode_Project_Tools\install.ps1 -ProfileName STM32
```

如果使用 VS Code 默认 Profile：

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_VSCode_Project_Tools\install.ps1 -ProfileName ''
```

安装后在 VS Code 中执行“开发人员: 重新加载窗口”。也可以在扩展视图右上角选择“从 VSIX 安装…”，然后选择：

```text
STM32_VSCode_Project_Tools/package/stm32-project-tools-0.1.2.vsix
```

注意：VS Code 扩展按 Profile 分别安装。当前窗口若使用 `STM32` Profile，就必须安装到 `STM32` Profile；安装到默认 Profile 不会自动出现在另一个 Profile 中。

## 使用方法

1. 用 VS Code 打开本仓库根目录，而不是只打开某个工程子目录。
2. 在资源管理器中展开 `Projects`。
3. 右键 `Projects` 的直接子目录，例如 `Projects/excavator_conctrler`。
4. 选择“STM32: 编译 Debug + Release”，扩展会依次配置并编译两个预设。
5. 选择“STM32: 选择固件并烧录”，再选择 Debug/Release 固件和要使用的 ST-Link。
6. 确认烧录信息后，扩展通过 STM32CubeProgrammer 执行写入、校验和复位。

右键菜单只对文件夹显示。若不想使用鼠标，也可以从命令面板运行同名命令，扩展会让你选择工程。

## 工程必须满足的约定

- 仓库根目录存在 `CMakePresets.json`。
- 工程是 `Projects/` 的直接子目录，并包含 `CMakeLists.txt`。
- configure preset 的 `PROJECT_BOARD` 值与工程目录名一致。
- 工程具有可见的 Debug 和 Release build preset。
- 编译后在对应 preset 的 `binaryDir` 下生成 `.hex` 文件。

可在 VS Code 设置中修改：

- `stm32ProjectTools.projectsDirectory`
- `stm32ProjectTools.boardCacheVariable`
- `stm32ProjectTools.cmakePath`
- `stm32ProjectTools.programmerPath`
- `stm32ProjectTools.interface`

## 没有出现右键菜单

按以下顺序检查：

1. 确认右键的是 `Projects/<工程>` 文件夹，不是空白区域、文件或更深层目录。
2. 确认当前 VS Code 窗口打开的是仓库根目录。
3. 确认扩展安装到了当前 Profile：

   ```powershell
   code --profile STM32 --list-extensions --show-versions | Select-String 'luxi-embedded.stm32-project-tools'
   ```

4. 执行“开发人员: 重新加载窗口”。
5. 查看“输出 -> STM32 Project Tools”中的诊断信息。

## 源码开发、重新打包和自检

只有修改扩展源码或重新生成 VSIX 时才需要 Node.js/npm：

自检和打包脚本优先使用 PATH 中的 Node.js/npm；未找到时自动查找 `CUBE_BUNDLE_PATH` 和 `%LOCALAPPDATA%/stm32cube/bundles` 下的 Node bundle，无需手动修改系统 PATH。打包仍需要 npm 中的 `@vscode/vsce`（首次使用需要联网下载）。

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_VSCode_Project_Tools\verify.ps1
powershell -ExecutionPolicy Bypass -File .\STM32_VSCode_Project_Tools\package-extension.ps1
```

重新打包会根据 `extension/package.json` 的版本号覆盖 `package/` 中对应的 VSIX。修改扩展功能后应先更新版本号，再运行自检和重新打包脚本。

卸载命令：

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_VSCode_Project_Tools\uninstall.ps1 -ProfileName STM32
```

## Git 与迁移注意事项

- 本目录中的扩展源码、脚本、文档和 VSIX 都随 Git 管理，新电脑克隆仓库后仍然存在。
- VS Code 的“已安装状态”保存在电脑本地，不在 Git 中；换电脑后仍需运行一次 `install.ps1`。
- `build/` 已被仓库 `.gitignore` 忽略。Debug/Release 固件和中间文件需要在新电脑重新编译，不随仓库提交。
- CMake、编译器、STM32CubeProgrammer、驱动和 ST-Link 本身属于外部依赖，不会被这个目录打包。
- 多个 ST-Link 同时连接时必须核对序列号；烧录前确认工程、固件类型和目标芯片，避免写错设备。

更完整的手工编译和烧录方法见 [docs/BUILD_FLASH_GUIDE.zh-CN.md](docs/BUILD_FLASH_GUIDE.zh-CN.md)。

版本 0.1.2：支持从仓库的上一级目录右键选择工程，自动定位 CMakePresets.json。调试请打开仓库根目录或上一级的 STM32.code-workspace，使 VS Code 加载仓库的 tasks.json 和 launch.json。
