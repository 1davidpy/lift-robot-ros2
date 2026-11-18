'use strict';

const childProcess = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');
const vscode = require('vscode');

const {
  findRepositoryRoot,
  discoverProjectPresets,
  expandBinaryDirectory,
  expandEnvironment,
  parseStLinkSerialNumbers
} = require('./core');

const output = vscode.window.createOutputChannel('STM32 Project Tools');
let operationRunning = false;

function configuration(workspaceFolder) {
  return vscode.workspace.getConfiguration('stm32ProjectTools', workspaceFolder.uri);
}

function samePath(left, right) {
  const normalize = (value) => path.resolve(value).replace(/[\\/]+$/, '').toLowerCase();
  return normalize(left) === normalize(right);
}

async function chooseProject(workspaceFolder) {
  const config = configuration(workspaceFolder);
  const projectsDirectory = config.get('projectsDirectory', 'Projects');
  const projectsRoot = path.resolve(workspaceFolder.uri.fsPath, projectsDirectory);
  let entries;

  try {
    entries = await fs.promises.readdir(projectsRoot, { withFileTypes: true });
  } catch (error) {
    throw new Error(`Cannot read projects directory: ${projectsRoot}`);
  }

  const projects = entries
    .filter((entry) => entry.isDirectory())
    .map((entry) => path.join(projectsRoot, entry.name))
    .filter((projectPath) => fs.existsSync(path.join(projectPath, 'CMakeLists.txt')));

  const selected = await vscode.window.showQuickPick(
    projects.map((projectPath) => ({
      label: path.basename(projectPath),
      description: path.relative(workspaceFolder.uri.fsPath, projectPath),
      projectPath
    })),
    {
      title: '选择 STM32 工程',
      placeHolder: '选择需要编译或烧录的工程'
    }
  );

  return selected ? vscode.Uri.file(selected.projectPath) : undefined;
}

async function resolveProjectContext(resourceUri) {
  let workspaceFolder = resourceUri
    ? vscode.workspace.getWorkspaceFolder(resourceUri)
    : vscode.workspace.workspaceFolders?.[0];

  if (!workspaceFolder) {
    throw new Error('请先打开一个 STM32 工作区。');
  }

  let workspacePath = resourceUri
    ? findRepositoryRoot(resourceUri.fsPath, workspaceFolder.uri.fsPath)
    : findRepositoryRoot(workspaceFolder.uri.fsPath, workspaceFolder.uri.fsPath);
  if (!workspacePath) {
    const entries = await fs.promises.readdir(workspaceFolder.uri.fsPath, { withFileTypes: true });
    const roots = entries.filter((entry) => entry.isDirectory())
      .map((entry) => path.join(workspaceFolder.uri.fsPath, entry.name))
      .filter((directory) => fs.existsSync(path.join(directory, 'CMakePresets.json')));
    if (roots.length === 1) {
      workspacePath = roots[0];
    } else if (roots.length > 1) {
      workspacePath = await vscode.window.showQuickPick(roots, { title: '选择 STM32 仓库' });
      if (!workspacePath) { return undefined; }
    } else {
      throw new Error('当前目录及其直属子目录中没有 CMakePresets.json。请打开 STM32 仓库。');
    }
  }
  const repositoryFolder = { ...workspaceFolder, uri: vscode.Uri.file(workspacePath) };
  const config = configuration(repositoryFolder);
  const projectsRoot = path.resolve(workspacePath, config.get('projectsDirectory', 'Projects'));
  const selectionIsProject = resourceUri
    && samePath(path.dirname(resourceUri.fsPath), projectsRoot)
    && fs.existsSync(path.join(resourceUri.fsPath, 'CMakeLists.txt'));
  const projectUri = selectionIsProject ? resourceUri : await chooseProject(repositoryFolder);
  if (!projectUri) {
    return undefined;
  }

  const projectPath = projectUri.fsPath;

  if (!samePath(path.dirname(projectPath), projectsRoot)
      || !fs.existsSync(path.join(projectPath, 'CMakeLists.txt'))) {
    throw new Error(`请选择 ${projectsRoot} 下包含 CMakeLists.txt 的直属工程目录。`);
  }

  const presetsPath = path.join(workspacePath, 'CMakePresets.json');
  if (!fs.existsSync(presetsPath)) {
    throw new Error(`CMakePresets.json not found: ${presetsPath}`);
  }

  let presetDocument;
  try {
    presetDocument = JSON.parse(await fs.promises.readFile(presetsPath, 'utf8'));
  } catch (error) {
    throw new Error(`Cannot parse ${presetsPath}: ${error.message}`);
  }

  const projectName = path.basename(projectPath);
  const boardCacheVariable = config.get('boardCacheVariable', 'PROJECT_BOARD');
  const presets = discoverProjectPresets(presetDocument, projectName, boardCacheVariable);
  if (presets.length === 0) {
    throw new Error(
      `没有找到 ${boardCacheVariable}=${projectName} 的可见 CMake configure preset。`
    );
  }

  return {
    workspaceFolder,
    workspacePath,
    projectPath,
    projectName,
    presets,
    config
  };
}

function compareBundleVersions(left, right) {
  const numbers = (name) => (name.match(/\d+/g) || []).map(Number);
  const leftNumbers = numbers(left.name);
  const rightNumbers = numbers(right.name);
  const length = Math.max(leftNumbers.length, rightNumbers.length);

  for (let index = 0; index < length; index += 1) {
    const difference = (rightNumbers[index] || 0) - (leftNumbers[index] || 0);
    if (difference !== 0) {
      return difference;
    }
  }
  return right.name.localeCompare(left.name);
}

function expandConfiguredPath(value) {
  if (!value) {
    return '';
  }
  let expanded = expandEnvironment(value);
  if (expanded === '~') {
    expanded = os.homedir();
  } else if (expanded.startsWith(`~${path.sep}`) || expanded.startsWith('~/')) {
    expanded = path.join(os.homedir(), expanded.slice(2));
  }
  return path.resolve(expanded);
}

function bundleRoots() {
  const roots = [];
  if (process.env.CUBE_BUNDLE_PATH) {
    roots.push(process.env.CUBE_BUNDLE_PATH);
  }
  if (process.env.LOCALAPPDATA) {
    roots.push(path.join(process.env.LOCALAPPDATA, 'stm32cube', 'bundles'));
  }
  roots.push(path.join(os.homedir(), '.local', 'share', 'stm32cube', 'bundles'));
  return [...new Set(roots.map((item) => path.resolve(item)))];
}

function executableFromPath(executableNames) {
  for (const directory of (process.env.PATH || '').split(path.delimiter)) {
    for (const executableName of executableNames) {
      const candidate = path.join(directory.replace(/^"|"$/g, ''), executableName);
      if (fs.existsSync(candidate)) {
        return candidate;
      }
    }
  }
  return undefined;
}

function findBundleExecutable(bundleName, executableNames) {
  for (const root of bundleRoots()) {
    const bundleDirectory = path.join(root, bundleName);
    if (!fs.existsSync(bundleDirectory)) {
      continue;
    }

    const versions = fs.readdirSync(bundleDirectory, { withFileTypes: true })
      .filter((entry) => entry.isDirectory())
      .sort(compareBundleVersions);

    for (const version of versions) {
      for (const executableName of executableNames) {
        const candidate = path.join(bundleDirectory, version.name, 'bin', executableName);
        if (fs.existsSync(candidate)) {
          return candidate;
        }
      }
    }
  }
  return executableFromPath(executableNames);
}

function requireTool(config, settingName, bundleName, executableNames) {
  const configured = expandConfiguredPath(config.get(settingName, ''));
  if (configured) {
    if (!fs.existsSync(configured)) {
      throw new Error(`${settingName} 指向的文件不存在：${configured}`);
    }
    return configured;
  }

  const detected = findBundleExecutable(bundleName, executableNames);
  if (!detected) {
    throw new Error(
      `找不到 ${executableNames[0]}。请安装 STM32Cube bundle，或配置 stm32ProjectTools.${settingName}。`
    );
  }
  return detected;
}

function commandLine(executable, args) {
  const quote = (value) => (/\s/.test(value) ? `"${value}"` : value);
  return [executable, ...args].map(quote).join(' ');
}

function runExecutable(executable, args, cwd, options = {}) {
  const { echoOutput = true } = options;
  output.appendLine(`\n> ${commandLine(executable, args)}`);

  return new Promise((resolve, reject) => {
    const child = childProcess.spawn(executable, args, {
      cwd,
      windowsHide: true,
      env: process.env
    });
    let combinedOutput = '';

    const collect = (data) => {
      const text = data.toString();
      combinedOutput += text;
      if (echoOutput) {
        output.append(text);
      }
    };

    child.stdout.on('data', collect);
    child.stderr.on('data', collect);
    child.on('error', reject);
    child.on('close', (code) => {
      if (code === 0) {
        resolve(combinedOutput);
      } else {
        const error = new Error(`命令执行失败，退出码 ${code}。`);
        error.commandOutput = combinedOutput;
        reject(error);
      }
    });
  });
}

async function buildPreset(context, preset, cmakeExecutable) {
  output.appendLine(`\n========== ${context.projectName}: ${preset.buildType} ==========`);
  await runExecutable(
    cmakeExecutable,
    ['--preset', preset.name],
    context.workspacePath
  );

  if (preset.buildPresetName) {
    await runExecutable(
      cmakeExecutable,
      ['--build', '--preset', preset.buildPresetName, '--parallel'],
      context.workspacePath
    );
  } else {
    const binaryDirectory = expandBinaryDirectory(
      preset.configurePreset.binaryDir,
      context.workspacePath,
      preset.name
    );
    await runExecutable(
      cmakeExecutable,
      ['--build', binaryDirectory, '--parallel'],
      context.workspacePath
    );
  }
}

async function buildProject(context) {
  const cmakeExecutable = requireTool(
    context.config,
    'cmakePath',
    'cmake',
    process.platform === 'win32' ? ['cmake.exe'] : ['cmake']
  );
  const variants = context.presets.filter(
    (preset) => preset.buildType === 'Debug' || preset.buildType === 'Release'
  );
  const presets = variants.length > 0 ? variants : context.presets;

  output.clear();
  output.show(true);
  await vscode.window.withProgress(
    {
      location: vscode.ProgressLocation.Notification,
      title: `STM32 编译：${context.projectName}`,
      cancellable: false
    },
    async (progress) => {
      for (let index = 0; index < presets.length; index += 1) {
        const preset = presets[index];
        progress.report({
          message: `${preset.buildType} (${index + 1}/${presets.length})`,
          increment: 100 / presets.length
        });
        await buildPreset(context, preset, cmakeExecutable);
      }
    }
  );

  vscode.window.showInformationMessage(
    `${context.projectName} 编译完成：${presets.map((item) => item.buildType).join(' + ')}`
  );
}

async function findFilesRecursively(root, extension, depth = 4) {
  if (!fs.existsSync(root)) {
    return [];
  }

  const result = [];
  async function visit(directory, remainingDepth) {
    if (remainingDepth < 0) {
      return;
    }
    const entries = await fs.promises.readdir(directory, { withFileTypes: true });
    for (const entry of entries) {
      const entryPath = path.join(directory, entry.name);
      if (entry.isFile() && path.extname(entry.name).toLowerCase() === extension) {
        result.push(entryPath);
      } else if (entry.isDirectory()
                 && entry.name !== 'CMakeFiles'
                 && entry.name !== '_deps') {
        await visit(entryPath, remainingDepth - 1);
      }
    }
  }

  await visit(root, depth);
  return result;
}

function binaryDirectoryFor(context, preset) {
  return expandBinaryDirectory(
    preset.configurePreset.binaryDir,
    context.workspacePath,
    preset.name
  );
}

async function firmwareFilesFor(context, preset) {
  const files = await findFilesRecursively(binaryDirectoryFor(context, preset), '.hex');
  const projectTargetExpression = /set\s*\(\s*TARGET_NAME\s+([^\s\)]+)/i;
  const projectCmake = await fs.promises.readFile(
    path.join(context.projectPath, 'CMakeLists.txt'),
    'utf8'
  );
  const targetName = projectTargetExpression.exec(projectCmake)?.[1];

  return files.sort((left, right) => {
    const leftPreferred = targetName && path.basename(left, '.hex') === targetName ? 1 : 0;
    const rightPreferred = targetName && path.basename(right, '.hex') === targetName ? 1 : 0;
    return rightPreferred - leftPreferred;
  });
}

async function chooseFirmwareVariant(context) {
  const items = [];
  for (const preset of context.presets) {
    const files = await firmwareFilesFor(context, preset);
    items.push({
      label: `${preset.buildType} — ${preset.name}`,
      description: files.length > 0 ? `${files.length} 个 HEX 固件` : '尚未编译',
      preset,
      files
    });
  }

  return vscode.window.showQuickPick(items, {
    title: `选择 ${context.projectName} 固件`,
    placeHolder: '选择 Debug 或 Release 固件'
  });
}

async function ensureFirmware(context, selected) {
  let files = selected.files;
  if (files.length === 0) {
    const action = await vscode.window.showInformationMessage(
      `${selected.preset.name} 尚无 HEX 固件，是否立即编译？`,
      { modal: true },
      '编译'
    );
    if (action !== '编译') {
      return undefined;
    }

    const cmakeExecutable = requireTool(
      context.config,
      'cmakePath',
      'cmake',
      process.platform === 'win32' ? ['cmake.exe'] : ['cmake']
    );
    output.show(true);
    await vscode.window.withProgress(
      {
        location: vscode.ProgressLocation.Notification,
        title: `编译 ${selected.preset.name}`,
        cancellable: false
      },
      () => buildPreset(context, selected.preset, cmakeExecutable)
    );
    files = await firmwareFilesFor(context, selected.preset);
  }

  if (files.length === 0) {
    throw new Error(`编译目录中未找到 HEX 固件：${binaryDirectoryFor(context, selected.preset)}`);
  }
  if (files.length === 1) {
    return files[0];
  }

  const picked = await vscode.window.showQuickPick(
    files.map((filePath) => ({
      label: path.basename(filePath),
      description: path.relative(context.workspacePath, filePath),
      filePath
    })),
    {
      title: '选择需要烧录的 HEX 文件',
      placeHolder: '该预设生成了多个固件'
    }
  );
  return picked?.filePath;
}

async function selectStLink(context, programmerExecutable) {
  const probeOutput = await runExecutable(
    programmerExecutable,
    ['-l', 'probe'],
    context.workspacePath,
    { echoOutput: false }
  );
  const serialNumbers = parseStLinkSerialNumbers(probeOutput);
  if (serialNumbers.length === 0) {
    output.appendLine(probeOutput);
    output.show(true);
    throw new Error('未检测到 ST-Link。请检查目标板供电、USB、SWDIO、SWCLK 和 GND。');
  }

  const picked = await vscode.window.showQuickPick(
    serialNumbers.map((serialNumber, index) => ({
      label: serialNumber,
      description: `ST-Link Probe ${index}`,
      serialNumber,
      index
    })),
    {
      title: '选择连接的 ST-Link',
      placeHolder: '固件将烧录到所选探针连接的目标板'
    }
  );
  return picked;
}

async function flashProject(context) {
  const selected = await chooseFirmwareVariant(context);
  if (!selected) {
    return;
  }

  const firmwarePath = await ensureFirmware(context, selected);
  if (!firmwarePath) {
    return;
  }

  const programmerExecutable = requireTool(
    context.config,
    'programmerPath',
    'programmer',
    process.platform === 'win32'
      ? ['STM32_Programmer_CLI.exe']
      : ['STM32_Programmer_CLI']
  );
  const probe = await selectStLink(context, programmerExecutable);
  if (!probe) {
    return;
  }

  const confirmation = await vscode.window.showWarningMessage(
    `即将用 ST-Link Probe ${probe.index}（${probe.serialNumber}）烧录 ${path.basename(firmwarePath)}。烧录会覆盖目标 MCU 的现有固件，请确认执行机构处于安全状态。`,
    { modal: true },
    '烧录'
  );
  if (confirmation !== '烧录') {
    return;
  }

  const probeInterface = context.config.get('interface', 'SWD');
  output.show(true);
  await vscode.window.withProgress(
    {
      location: vscode.ProgressLocation.Notification,
      title: `烧录 ${path.basename(firmwarePath)}`,
      cancellable: false
    },
    () => runExecutable(
      programmerExecutable,
      [
        '-c',
        `port=${probeInterface}`,
        `index=${probe.index}`,
        '-w',
        firmwarePath,
        '-v',
        '-rst'
      ],
      context.workspacePath
    )
  );

  vscode.window.showInformationMessage(
    `烧录、校验并复位成功：${path.basename(firmwarePath)} → ${probe.serialNumber}`
  );
}

async function runCommand(resourceUri, action) {
  if (operationRunning) {
    vscode.window.showWarningMessage('已有 STM32 编译或烧录任务正在运行。');
    return;
  }

  operationRunning = true;
  try {
    const context = await resolveProjectContext(resourceUri);
    if (context) {
      await action(context);
    }
  } catch (error) {
    output.appendLine(`\nERROR: ${error.stack || error.message}`);
    output.show(true);
    vscode.window.showErrorMessage(`STM32 Project Tools：${error.message}`);
  } finally {
    operationRunning = false;
  }
}

function activate(extensionContext) {
  extensionContext.subscriptions.push(
    output,
    vscode.commands.registerCommand(
      'stm32ProjectTools.buildProject',
      (resourceUri) => runCommand(resourceUri, buildProject)
    ),
    vscode.commands.registerCommand(
      'stm32ProjectTools.flashProject',
      (resourceUri) => runCommand(resourceUri, flashProject)
    )
  );
}

function deactivate() {}

module.exports = { activate, deactivate };
