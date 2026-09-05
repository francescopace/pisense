// SPDX-License-Identifier: GPL-3.0-only
// Commercial licensing available under separate agreement; see LICENSING.md.
import { readFileSync, readdirSync } from 'node:fs';
import { Linter } from 'eslint';
import js from '@eslint/js';
import globals from 'globals';

// The site loads classic scripts that share top-level bindings across files.
// Derive those bindings from source so undefined-name checks still catch typos.
const sourceDirectory = new URL('../../docs/web/assets/js/', import.meta.url);
const sharedGlobals = {};
for (const filename of readdirSync(sourceDirectory).filter((name) => name.endsWith('.js'))) {
  const linter = new Linter();
  const source = readFileSync(new URL(filename, sourceDirectory), 'utf8');
  linter.verify(source, { languageOptions: { sourceType: 'script' }, rules: {} });
  const scope = linter.getSourceCode()?.scopeManager.globalScope;
  for (const variable of scope?.variables || []) {
    if (variable.defs.length) {
      sharedGlobals[variable.name] = variable.defs.every((definition) => definition.kind === 'const')
        ? 'readonly' : 'writable';
    }
  }
}

export default [
  { ignores: ['**/node_modules/**', 'docs/web/assets/vendor/**'] },
  {
    ...js.configs.recommended,
    files: ['docs/web/assets/js/**/*.js'],
    languageOptions: { sourceType: 'script', globals: { ...globals.browser, ...sharedGlobals } },
    rules: {
      ...js.configs.recommended.rules,
      'no-unused-vars': ['error', { vars: 'local' }],
      'no-redeclare': ['error', { builtinGlobals: false }],
    },
  },
  {
    ...js.configs.recommended,
    files: ['docs/web/headless-entry.js'],
    languageOptions: { sourceType: 'module', globals: globals.browser },
  },
  {
    ...js.configs.recommended,
    files: ['docs/web/*.mjs', '.github/quality/*.mjs'],
    languageOptions: { globals: globals.node },
  },
];
