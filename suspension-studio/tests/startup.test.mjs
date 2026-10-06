import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

const startupSource = readFileSync(new URL('../src/startup.js', import.meta.url), 'utf8');
const indexSource = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
const helpMarkup = indexSource.match(/<div id="startup-help" hidden>([\s\S]*?)<\/div>/)?.[1];

function element(id) {
  let text = '';
  const attributes = new Map(id === 'studio-workspace' ? [['inert', '']] : []);
  return {
    hidden: ['startup-help', 'startup-details'].includes(id),
    get textContent() { return text; },
    set textContent(value) { text = String(value); },
    set innerHTML(_value) { throw new Error('Startup messages must use textContent'); },
    setAttribute(name, value) { attributes.set(name, String(value)); },
    removeAttribute(name) { attributes.delete(name); },
    getAttribute(name) { return attributes.get(name) ?? null; },
    hasAttribute(name) { return attributes.has(name); }
  };
}

async function runStartup({protocol = 'http:', missing = [], forbidCapabilityChecks = false} = {}) {
  const ids = ['startup-panel', 'startup-title', 'startup-message', 'startup-help',
    'startup-details', 'startup-error', 'studio-workspace', 'scene'];
  const elements = new Map(ids.map(id => [id, element(id)]));
  const dataset = {studioState: 'loading'};
  const accesses = {imports: 0, storage: 0, canvas: 0, capabilities: 0};
  elements.get('startup-help').textContent = helpMarkup?.replace(/<[^>]+>/g, '');
  elements.get('scene').getContext = kind => {
    accesses.canvas++;
    assert.equal(kind, '2d');
    if (forbidCapabilityChecks) throw new Error('file: must stop before Canvas checks');
    return missing.includes('Canvas 2D') ? null : {};
  };
  const sandbox = {
    location: {protocol},
    document: {
      documentElement: {dataset},
      getElementById(id) {
        assert.ok(elements.has(id), `Unexpected startup DOM element: ${id}`);
        return elements.get(id);
      }
    }
  };
  const capabilities = {
    structuredClone: missing.includes('structuredClone') ? undefined : structuredClone,
    Object: {hasOwn: missing.includes('Object.hasOwn') ? undefined : Object.hasOwn},
    ResizeObserver: missing.includes('ResizeObserver') ? undefined : function ResizeObserver() {},
    HTMLDialogElement: missing.includes('dialog.showModal')
      ? undefined : {prototype: {showModal() {}}}
  };
  for (const [name, value] of Object.entries(capabilities)) {
    Object.defineProperty(sandbox, name, {get() {
      accesses.capabilities++;
      if (forbidCapabilityChecks) throw new Error('file: must stop before capability checks');
      return value;
    }});
  }
  for (const name of ['localStorage', 'sessionStorage']) {
    Object.defineProperty(sandbox, name, {get() {
      accesses.storage++;
      throw new Error('Startup failure handling must not access storage');
    }});
  }
  const script = new vm.Script(startupSource, {
    filename: 'startup.js',
    importModuleDynamically() {
      accesses.imports++;
      throw new Error('An early startup failure must not import app.js');
    }
  });
  script.runInContext(vm.createContext(sandbox), {timeout: 1000});
  // An accidentally reached import can reject asynchronously. Check final state
  // after those callbacks, without modifying the source's native import syntax.
  await new Promise(resolve => setImmediate(resolve));
  return {elements, dataset, accesses};
}

function assertFailureState({elements, dataset, accesses}) {
  assert.equal(dataset.studioState, 'error');
  assert.equal(elements.get('startup-panel').hidden, false);
  assert.equal(elements.get('startup-panel').getAttribute('role'), 'alert');
  assert.equal(elements.get('startup-help').hidden, false);
  assert.equal(elements.get('studio-workspace').hasAttribute('inert'), true);
  assert.equal(accesses.imports, 0);
  assert.equal(accesses.storage, 0);
}

test('file: opening explains HTTP setup before imports, browser API checks, or storage access', async () => {
  const result = await runStartup({protocol: 'file:', forbidCapabilityChecks: true});
  assertFailureState(result);
  assert.match(indexSource, /id="studio-workspace" inert/);
  assert.match(result.elements.get('startup-title').textContent, /web address/);
  assert.match(result.elements.get('startup-message').textContent, /JavaScript modules/);
  assert.match(result.elements.get('startup-help').textContent, /python3 -m http\.server 8766 --bind 127\.0\.0\.1/);
  assert.match(helpMarkup, /href="http:\/\/127\.0\.0\.1:8766\/suspension-studio\/"/);
  assert.equal(result.elements.get('startup-details').hidden, true);
  assert.equal(result.elements.get('startup-error').textContent, '');
  assert.equal(result.accesses.capabilities, 0);
  assert.equal(result.accesses.canvas, 0);
});

test('missing ResizeObserver leaves workspace inert and names the unavailable API', async () => {
  const result = await runStartup({missing: ['ResizeObserver']});
  assertFailureState(result);
  assert.match(result.elements.get('startup-title').textContent, /cannot start/);
  assert.match(result.elements.get('startup-message').textContent, /updated browser/);
  assert.equal(result.elements.get('startup-details').hidden, false);
  assert.equal(result.elements.get('startup-error').textContent,
    'Required browser features unavailable: ResizeObserver');
  assert.equal(result.accesses.canvas, 1);
});

test('all unavailable browser APIs are reported safely as text without loading modules', async () => {
  const missing = ['structuredClone', 'Object.hasOwn', 'ResizeObserver', 'dialog.showModal', 'Canvas 2D'];
  const result = await runStartup({missing});
  assertFailureState(result);
  assert.equal(result.elements.get('startup-details').hidden, false);
  assert.equal(result.elements.get('startup-error').textContent,
    `Required browser features unavailable: ${missing.join(', ')}`);
});
