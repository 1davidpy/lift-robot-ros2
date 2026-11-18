'use strict';

const assert = require('assert');
const fs = require('fs');
const path = require('path');

const {
  findRepositoryRoot,
  discoverProjectPresets,
  expandBinaryDirectory,
  parseStLinkSerialNumbers
} = require('../core');

const workspaceRoot = path.resolve(__dirname, '..', '..', '..');
const presets = JSON.parse(
  fs.readFileSync(path.join(workspaceRoot, 'CMakePresets.json'), 'utf8')
);

assert.strictEqual(findRepositoryRoot(path.join(workspaceRoot, 'Projects', 'F103C8T6_Chip'),
  path.dirname(workspaceRoot)), workspaceRoot);
assert.strictEqual(findRepositoryRoot(workspaceRoot, workspaceRoot), workspaceRoot);
assert.strictEqual(findRepositoryRoot(path.dirname(workspaceRoot), workspaceRoot), undefined);

const expectedProjects = [
  'F103C8T6_Chip',
  'excavator_conctrler',
  'excavator_controlled',
  'uart_test',
  'luxi_atlas_pwm'
];

for (const projectName of expectedProjects) {
  const projectPresets = discoverProjectPresets(presets, projectName);
  assert.deepStrictEqual(
    projectPresets.map((preset) => preset.buildType),
    ['Debug', 'Release'],
    `${projectName} should have Debug and Release presets`
  );
  assert.ok(
    projectPresets.every((preset) => preset.buildPresetName === preset.name),
    `${projectName} should have matching build presets`
  );
}

const debugPreset = discoverProjectPresets(presets, 'excavator_controlled')[0];
assert.strictEqual(
  expandBinaryDirectory(
    debugPreset.configurePreset.binaryDir,
    workspaceRoot,
    debugPreset.name
  ),
  path.join(workspaceRoot, 'build', 'excavator-controlled-debug')
);

assert.deepStrictEqual(
  parseStLinkSerialNumbers(`
===== STLink Interface =====
ST-LINK SN  : E1007200D0D2139393740544
ST-LINK FW  : V2J45S7
ST-LINK SN  : Y
ST-LINK SN  : 00112233445566778899AABB
`),
  ['E1007200D0D2139393740544', 'Y', '00112233445566778899AABB']
);

assert.deepStrictEqual(
  parseStLinkSerialNumbers('No ST-Link detected!'),
  []
);

console.log('STM32 Project Tools core tests passed.');
