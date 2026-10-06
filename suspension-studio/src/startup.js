// Classic script: this can explain file:// startup failures before loading modules.
(() => {
  const element = id => document.getElementById(id);
  const panel = element('startup-panel');

  function fail(title, message, detail = '') {
    element('startup-title').textContent = title;
    element('startup-message').textContent = message;
    element('startup-help').hidden = false;
    element('startup-details').hidden = !detail;
    element('startup-error').textContent = detail;
    panel.setAttribute('role', 'alert');
    document.documentElement.dataset.studioState = 'error';
  }

  if (location.protocol === 'file:') {
    fail('Open Suspension Studio through a web address',
      'This local file cannot load the editor’s JavaScript modules. Open the hosted app, or serve this folder over HTTP using the steps below.');
    return;
  }

  const missing = [];
  if (typeof structuredClone !== 'function') missing.push('structuredClone');
  if (typeof Object.hasOwn !== 'function') missing.push('Object.hasOwn');
  if (typeof ResizeObserver !== 'function') missing.push('ResizeObserver');
  if (typeof HTMLDialogElement === 'undefined' || typeof HTMLDialogElement.prototype.showModal !== 'function') missing.push('dialog.showModal');
  if (!element('scene').getContext('2d')) missing.push('Canvas 2D');
  if (missing.length) {
    fail('This browser cannot start the workspace',
      'Use an updated browser with JavaScript and Canvas 2D enabled.',
      `Required browser features unavailable: ${missing.join(', ')}`);
    return;
  }

  // Resolution means the module graph loaded and app.js finished initializing.
  import('./app.js?v=0.2.8').then(() => {
    element('studio-workspace').removeAttribute('inert');
    panel.hidden = true;
    document.documentElement.dataset.studioState = 'ready';
  }).catch(error => {
    fail('The workspace could not start',
      'The editor or viewer failed to load. Refresh once; if the problem continues, review the technical details below.',
      error instanceof Error ? `${error.name}: ${error.message}` : String(error));
  });
})();
