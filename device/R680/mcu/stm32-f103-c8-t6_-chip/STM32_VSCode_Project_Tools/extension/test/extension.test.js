'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { EventEmitter } = require('events');
const root = path.resolve(__dirname, '../../..');
const commands = new Map();
const calls = [];
const errors = [];
let folder = { uri: { fsPath: path.dirname(root) } };
const vscode = {
  Uri: { file: (fsPath) => ({ fsPath }) },
  ProgressLocation: { Notification: 1 },
  workspace: {
    get workspaceFolders() { return [folder]; },
    getWorkspaceFolder: () => folder,
    getConfiguration: () => ({ get: (name, fallback) => name === 'cmakePath' ? process.execPath : fallback })
  },
  commands: { registerCommand: (name, callback) => commands.set(name, callback) },
  window: {
    createOutputChannel: () => ({ appendLine() {}, append() {}, clear() {}, show() {} }),
    showQuickPick: async (items) => items.find((item) => item.label === 'lifting_platform_servo') || items[0],
    withProgress: async (_, action) => action({ report() {} }),
    showInformationMessage() {},
    showWarningMessage() {},
    showErrorMessage: (message) => errors.push(message)
  }
};
const childProcess = {
  spawn: (executable, args, options) => {
    calls.push({ args, cwd: options.cwd });
    const child = new EventEmitter();
    child.stdout = new EventEmitter();
    child.stderr = new EventEmitter();
    process.nextTick(() => child.emit('close', 0));
    return child;
  }
};
const exported = { exports: {} };
vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../extension.js'), 'utf8'), {
  require: (name) => name === 'vscode' ? vscode : name === 'child_process' ? childProcess
    : name === './core' ? require('../core') : require(name),
  module: exported, process
});
exported.exports.activate({ subscriptions: [] });

(async () => {
  const build = commands.get('stm32ProjectTools.buildProject');
  const project = vscode.Uri.file(path.join(root, 'Projects/lifting_platform_servo'));
  for (const resource of [project, vscode.Uri.file(root), undefined]) {
    calls.length = 0;
    await build(resource);
    assert.deepStrictEqual(errors, []);
    assert.strictEqual(calls.length, 4);
    assert.ok(calls.every((call) => call.cwd === root));
    assert.strictEqual(calls[0].args[1], 'lifting-platform-servo-debug');
    assert.strictEqual(calls[2].args[1], 'lifting-platform-servo-release');
  }
  folder = { uri: { fsPath: root } };
  calls.length = 0;
  await build(project);
  assert.deepStrictEqual(errors, []);
  assert.strictEqual(calls.length, 4);
  console.log('STM32 extension command tests passed (parent workspace and repository root).');
})().catch((error) => { console.error(error); process.exitCode = 1; });
