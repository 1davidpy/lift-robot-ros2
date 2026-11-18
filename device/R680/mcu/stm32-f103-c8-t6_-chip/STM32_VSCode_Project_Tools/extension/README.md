# STM32 Project Tools

This VS Code extension adds two commands to the Explorer context menu for directories directly below `Projects/`:

- `STM32: 编译 Debug + Release`
- `STM32: 选择固件并烧录`

## Workspace convention

The extension discovers projects rather than hard-coding their names. A compatible workspace has:

1. `CMakePresets.json` in the workspace root.
2. Project directories below `Projects/` containing `CMakeLists.txt`.
3. Visible configure presets whose `PROJECT_BOARD` cache variable matches the project directory name.
4. Debug and Release build presets associated with those configure presets.
5. A successful build that creates an Intel HEX file below the preset's `binaryDir`.

The project directory and cache variable names can be changed with `stm32ProjectTools.projectsDirectory` and `stm32ProjectTools.boardCacheVariable`.

## Tool discovery

The extension searches `CUBE_BUNDLE_PATH`, standard STM32Cube bundle locations on Windows and Linux, and then `PATH`. Explicit paths can be configured with:

- `stm32ProjectTools.cmakePath`
- `stm32ProjectTools.programmerPath`

## Development

From the parent `STM32_VSCode_Project_Tools` directory, run:

```powershell
.\verify.ps1
.\package-extension.ps1
.\install.ps1 -ProfileName STM32
```

Reload the VS Code window after installation. The commands are also available from the Command Palette, where the extension asks which project to use.

Version 0.1.2: detects the repository when its parent folder is open. For debugging, open the repository root or STM32.code-workspace so VS Code loads tasks.json and launch.json.
