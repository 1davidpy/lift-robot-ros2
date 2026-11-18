'use strict';

const path = require('path');
const fs = require('fs');

function findRepositoryRoot(resourcePath, workspacePath) {
  const boundary = path.resolve(workspacePath);
  let candidate = path.resolve(resourcePath);
  while (true) {
    const relative = path.relative(boundary, candidate);
    if (relative === '..' || relative.startsWith(`..${path.sep}`) || path.isAbsolute(relative)) {
      return undefined;
    }
    if (fs.existsSync(path.join(candidate, 'CMakePresets.json'))) {
      return candidate;
    }
    if (candidate === boundary) { return undefined; }
    candidate = path.dirname(candidate);
  }
}

function mergePreset(base, current) {
  return {
    ...base,
    ...current,
    cacheVariables: {
      ...(base.cacheVariables || {}),
      ...(current.cacheVariables || {})
    },
    environment: {
      ...(base.environment || {}),
      ...(current.environment || {})
    }
  };
}

function resolvePresetList(presets) {
  const byName = new Map((presets || []).map((preset) => [preset.name, preset]));
  const resolved = new Map();

  function resolve(name, stack = []) {
    if (resolved.has(name)) {
      return resolved.get(name);
    }

    const preset = byName.get(name);
    if (!preset) {
      throw new Error(`Preset '${name}' does not exist.`);
    }
    if (stack.includes(name)) {
      throw new Error(`Circular preset inheritance: ${[...stack, name].join(' -> ')}`);
    }

    let result = {};
    const parents = Array.isArray(preset.inherits)
      ? preset.inherits
      : (preset.inherits ? [preset.inherits] : []);

    for (const parent of parents) {
      result = mergePreset(result, resolve(parent, [...stack, name]));
    }

    result = mergePreset(result, preset);
    // CMake uses hidden presets as inheritance bases. A visible child must not
    // become hidden merely because its parent is hidden.
    result.hidden = preset.hidden === true;
    resolved.set(name, result);
    return result;
  }

  for (const name of byName.keys()) {
    resolve(name);
  }

  return resolved;
}

function cacheVariableValue(value) {
  if (value && typeof value === 'object' && Object.prototype.hasOwnProperty.call(value, 'value')) {
    return String(value.value);
  }
  return value === undefined || value === null ? undefined : String(value);
}

function buildTypeForPreset(preset) {
  const configured = cacheVariableValue(preset.cacheVariables?.CMAKE_BUILD_TYPE);
  if (configured) {
    return configured;
  }
  if (/debug/i.test(preset.name)) {
    return 'Debug';
  }
  if (/release/i.test(preset.name)) {
    return 'Release';
  }
  return 'Other';
}

function discoverProjectPresets(document, projectName, boardCacheVariable = 'PROJECT_BOARD') {
  const configurePresets = resolvePresetList(document.configurePresets || []);
  const buildPresets = resolvePresetList(document.buildPresets || []);
  const result = [];

  for (const preset of configurePresets.values()) {
    if (preset.hidden) {
      continue;
    }

    const board = cacheVariableValue(preset.cacheVariables?.[boardCacheVariable]);
    if (board !== projectName) {
      continue;
    }

    const matchingBuildPresets = [...buildPresets.values()].filter(
      (buildPreset) => !buildPreset.hidden && buildPreset.configurePreset === preset.name
    );
    const buildPreset = matchingBuildPresets.find((item) => item.name === preset.name)
      || matchingBuildPresets[0];

    result.push({
      name: preset.name,
      configurePreset: preset,
      buildPresetName: buildPreset?.name,
      buildType: buildTypeForPreset(preset)
    });
  }

  const order = { Debug: 0, Release: 1, Other: 2 };
  return result.sort((left, right) => {
    const typeOrder = (order[left.buildType] ?? 2) - (order[right.buildType] ?? 2);
    return typeOrder || left.name.localeCompare(right.name);
  });
}

function expandEnvironment(text, environment = process.env) {
  return String(text)
    .replace(/\$\{env:([^}]+)\}/g, (_, name) => environment[name] || '')
    .replace(/\$(?:p?env)\{([^}]+)\}/g, (_, name) => environment[name] || '');
}

function expandBinaryDirectory(binaryDir, sourceDir, presetName, environment = process.env) {
  const sourceParentDir = path.dirname(sourceDir);
  const sourceDirName = path.basename(sourceDir);
  const expanded = expandEnvironment(binaryDir || '${sourceDir}/build', environment)
    .replace(/\$\{sourceDir\}/g, sourceDir)
    .replace(/\$\{sourceParentDir\}/g, sourceParentDir)
    .replace(/\$\{sourceDirName\}/g, sourceDirName)
    .replace(/\$\{presetName\}/g, presetName);

  return path.resolve(sourceDir, expanded);
}

function parseStLinkSerialNumbers(output) {
  const serialNumbers = new Set();
  const expressions = [
    /^\s*ST-LINK\s+SN\s*:\s*(\S+)\s*$/gim,
    /^\s*Serial\s+Number\s*:\s*(\S+)\s*$/gim
  ];

  for (const expression of expressions) {
    let match;
    while ((match = expression.exec(output)) !== null) {
      serialNumbers.add(match[1].toUpperCase());
    }
  }

  return [...serialNumbers];
}

module.exports = {
  findRepositoryRoot,
  cacheVariableValue,
  discoverProjectPresets,
  expandBinaryDirectory,
  expandEnvironment,
  parseStLinkSerialNumbers,
  resolvePresetList
};
