import { calculateAckermann, engineHealth, CalculationError } from './api-client.js';

const $ = id => document.getElementById(id);
const inputIds = ['wheelbase', 'front-track', 'inside-limit'];
let result = null;
let dimensions = null;
let activeRequest = null;
let requestVersion = 0;
let viewer = null;
let chartReady = false;
let referenceLoaded = false;
let chartRevision = 0;
const formatAngle = value => `${value > 0 ? '+' : ''}${value.toFixed(3)}°`;

function notice(message, isError = false) {
  $('workbench-notice').textContent = message;
  $('workbench-notice').classList.toggle('error', isError);
  $('workbench-notice').hidden = !message;
}

function engineStatus(message, ready = false) {
  $('engine-state').textContent = message;
  $('engine-state').dataset.ready = String(ready);
}

function disablePreview(message) {
  viewer = null;
  $('geometry-scene').querySelectorAll('canvas').forEach(canvas => { canvas.hidden = true; });
  $('scene-empty').hidden = false;
  $('scene-empty').textContent = message;
  $('reset-view').disabled = true;
  $('scene-note').textContent = 'The optional 3D preview requires WebGL. All calculated headings remain available in the readouts.';
}

function emptyChart(message) {
  chartRevision += 1;
  if (chartReady && window.Plotly) window.Plotly.purge($('ackermann-chart'));
  chartReady = false;
  const placeholder = document.createElement('div');
  placeholder.className = 'engineering-empty';
  const text = document.createElement('span');
  text.textContent = message;
  placeholder.append(text);
  $('ackermann-chart').replaceChildren(placeholder);
}

function clearResults() {
  result = null;
  dimensions = null;
  for (const id of ['inside-value', 'outside-value', 'left-value', 'right-value', 'selected-angle']) $(id).textContent = '—';
  $('turn-direction').textContent = 'Awaiting calculation';
  $('sweep-summary').textContent = 'No calculated sweep';
  $('sample-slider').disabled = true;
  $('sample-slider').min = '0';
  $('sample-slider').max = '0';
  $('sample-slider').value = '0';
  $('center-sample').disabled = true;
  $('reset-view').disabled = true;
  $('negative-limit').textContent = 'Left turn';
  $('positive-limit').textContent = 'Right turn';
  $('scene-empty').hidden = false;
  try { viewer?.clear(); }
  catch { disablePreview('3D preview unavailable. Calculated numbers and chart remain available. Refresh to retry the preview.'); }
  emptyChart('The chart appears after a successful calculation.');
}

function invalidate() {
  requestVersion += 1;
  activeRequest?.abort();
  activeRequest = null;
  $('calculate').disabled = false;
  $('calculate').textContent = 'Calculate reference →';
  clearResults();
}

function readInputs() {
  const numbers = inputIds.map(id => {
    const input = $(id);
    const value = input.value.trim();
    const number = Number(value);
    if (!value || !Number.isFinite(number) || number <= 0) {
      input.focus();
      throw new CalculationError('INPUT_REQUIRED', 'Enter finite, positive values for wheelbase, front track and the inside-angle sweep limit.');
    }
    return number;
  });
  if (numbers[2] >= 89) {
    $('inside-limit').focus();
    throw new CalculationError('ANGLE_RANGE', 'The inside-angle sweep limit must be greater than 0° and below 89°.');
  }
  if (numbers[0] < 1e-6 || numbers[1] < 1e-6 || numbers[0] > 1e7 || numbers[1] > 1e7) {
    $(numbers[0] < 1e-6 || numbers[0] > 1e7 ? 'wheelbase' : 'front-track').focus();
    throw new CalculationError('DIMENSION_RANGE', 'Wheelbase and front track must be between 0.000001 and 10,000,000 mm. These are numerical input bounds, not physical recommendations.');
  }
  return { wheelbase: numbers[0], track: numbers[1], limit: numbers[2] };
}

// Sample construction only. Every displayed result is supplied by Python;
// there is no browser implementation of the Ackermann calculation here.
function inputSweep(limit) {
  const intervals = Math.max(1, Math.ceil(limit / 0.25));
  return Array.from({ length: intervals * 2 + 1 }, (_, index) => index === intervals ? 0 : (index - intervals) * limit / intervals);
}

function selectedSample() { return result?.samples[Number($('sample-slider').value)] || null; }

function showSample() {
  const sample = selectedSample();
  if (!sample) return;
  $('inside-value').textContent = formatAngle(sample.inside_angle_deg);
  $('outside-value').textContent = formatAngle(sample.outside_angle_deg);
  $('left-value').textContent = formatAngle(sample.left_heading_deg);
  $('right-value').textContent = formatAngle(sample.right_heading_deg);
  $('selected-angle').textContent = formatAngle(sample.inside_angle_deg);
  $('sample-slider').setAttribute('aria-valuetext', `${sample.inside_angle_deg.toFixed(3)} degrees, ${sample.turn_direction === 'straight' ? 'straight ahead' : `${sample.turn_direction} turn`}`);
  $('turn-direction').textContent = sample.turn_direction === 'straight' ? 'Straight ahead' : `${sample.turn_direction === 'right' ? 'Right' : 'Left'} turn`;
  try { viewer?.update(dimensions, sample); }
  catch { disablePreview('3D preview unavailable: the renderer could not display this geometry. Calculated numbers and chart remain available.'); }
  if (chartReady && window.Plotly) {
    window.Plotly.restyle($('ackermann-chart'), { x: [[sample.inside_angle_deg]], y: [[sample.outside_angle_deg]] }, [1]).catch(() => {});
  }
}

async function drawChart() {
  if (!result) return;
  if (!window.Plotly) {
    emptyChart('Chart unavailable: the bundled plotting library did not load. Calculated numbers remain available.');
    return;
  }
  const revision = ++chartRevision;
  const style = getComputedStyle(document.body);
  const dark = document.documentElement.dataset.theme === 'dark';
  const color = style.getPropertyValue('--bench-curve').trim();
  const text = dark ? '#d7d3df' : '#5c5662';
  const grid = dark ? '#333b4a' : '#efedf1';
  const sample = selectedSample();
  const traces = [
    { x: result.samples.map(row => row.inside_angle_deg), y: result.samples.map(row => row.outside_angle_deg), type: 'scatter', mode: 'lines', name: 'Ideal outside angle', line: { color, width: 2.5 }, hovertemplate: 'Inside %{x:.3f}°<br>Outside %{y:.3f}°<extra></extra>' },
    { x: [sample.inside_angle_deg], y: [sample.outside_angle_deg], type: 'scatter', mode: 'markers', name: 'Selected angle', marker: { color, size: 9, line: { color: dark ? '#1c2432' : '#fff', width: 2 } }, hovertemplate: 'Selected<br>Inside %{x:.3f}°<br>Outside %{y:.3f}°<extra></extra>' },
  ];
  try {
    if (!chartReady) $('ackermann-chart').replaceChildren();
    await window.Plotly.react($('ackermann-chart'), traces, {
      paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
      font: { family: '-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif', color: text, size: 10 },
      margin: { l: 58, r: 18, t: 12, b: 75 },
      xaxis: { title: { text: 'Signed inside angle (deg)', standoff: 10 }, gridcolor: grid, zerolinecolor: dark ? '#687082' : '#ccc5ce', fixedrange: false },
      yaxis: { title: { text: 'Signed outside angle (deg)', standoff: 8 }, gridcolor: grid, zerolinecolor: dark ? '#687082' : '#ccc5ce', fixedrange: false },
      legend: { orientation: 'h', y: -0.3, x: 0, font: { size: 10 } },
      hovermode: 'closest', autosize: true,
    }, { responsive: true, displayModeBar: false, scrollZoom: false });
    if (revision === chartRevision && result) chartReady = true;
  } catch {
    if (revision === chartRevision) emptyChart('Chart unavailable: the plotting library could not render this view. Calculated numbers remain available.');
  }
}

async function calculate(event) {
  event?.preventDefault();
  invalidate();
  let inputs;
  try { inputs = readInputs(); }
  catch (error) { notice(`${error.code}: ${error.message}`, true); return; }
  const version = requestVersion;
  activeRequest = new AbortController();
  $('calculate').disabled = true;
  $('calculate').textContent = 'Calculating…';
  notice('Calculating both turn directions with the shared engine…');
  try {
    const response = await calculateAckermann({ wheelbase_mm: inputs.wheelbase, front_track_mm: inputs.track, inside_angles_deg: inputSweep(inputs.limit) }, { signal: activeRequest.signal });
    if (version !== requestVersion) return;
    result = response;
    dimensions = inputs;
    $('sample-slider').max = String(result.samples.length - 1);
    $('sample-slider').value = String(result.samples.length - 1);
    $('sample-slider').disabled = false;
    $('center-sample').disabled = false;
    $('reset-view').disabled = !viewer;
    $('negative-limit').textContent = `Left · −${inputs.limit}°`;
    $('positive-limit').textContent = `Right · +${inputs.limit}°`;
    $('sweep-summary').textContent = `${result.samples.length} engine-calculated samples · both directions · spacing ≤0.25°`;
    $('scene-empty').hidden = Boolean(viewer);
    engineStatus('Calculation engine ready', true);
    notice(referenceLoaded ? 'Reference example calculated. These nominal dimensions are a numerical benchmark, not measured vehicle geometry.' : 'Reference calculated. The chart and schematic use the same returned samples.');
    viewer?.reset();
    showSample();
    await drawChart();
  } catch (error) {
    if (version !== requestVersion) return;
    clearResults();
    engineStatus('Calculation unavailable');
    notice(`${error.code || 'CALCULATION_FAILED'}: ${error.message}`, true);
  } finally {
    if (version === requestVersion) {
      activeRequest = null;
      $('calculate').disabled = false;
      $('calculate').textContent = 'Calculate reference →';
    }
  }
}

class NominalViewer {
  constructor(THREE) {
    this.T = THREE;
    this.host = $('geometry-scene');
    this.scene = new THREE.Scene();
    this.group = new THREE.Group();
    this.scene.add(this.group);
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 1000000000);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0, 0);
    this.host.append(this.renderer.domElement);
    this.renderer.domElement.setAttribute('aria-hidden', 'true');
    this.yaw = 0.48; this.pitch = 0.76; this.zoom = 1;
    this.observer = new ResizeObserver(() => this.render());
    this.observer.observe(this.host);
    const canvas = this.renderer.domElement;
    canvas.addEventListener('pointerdown', event => { this.drag = [event.clientX, event.clientY]; canvas.setPointerCapture(event.pointerId); });
    canvas.addEventListener('pointermove', event => {
      if (!this.drag || !this.dimensions) return;
      this.yaw -= (event.clientX - this.drag[0]) * 0.008;
      this.pitch = Math.max(0.15, Math.min(1.5, this.pitch + (event.clientY - this.drag[1]) * 0.008));
      this.drag = [event.clientX, event.clientY]; this.render();
    });
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) canvas.addEventListener(name, () => { this.drag = null; });
    canvas.addEventListener('wheel', event => {
      if (!this.dimensions) return;
      event.preventDefault(); this.zoom = Math.max(0.45, Math.min(3, this.zoom * Math.exp(-event.deltaY * 0.001))); this.render();
    }, { passive: false });
    canvas.addEventListener('webglcontextlost', event => {
      event.preventDefault();
      disablePreview('3D preview unavailable: the graphics context was lost. Calculated numbers and chart remain available. Refresh to retry the preview.');
    });
  }

  clear() {
    for (const object of [...this.group.children]) {
      this.group.remove(object);
      object.geometry?.dispose();
      if (Array.isArray(object.material)) object.material.forEach(material => material.dispose());
      else object.material?.dispose();
    }
    this.dimensions = null;
    this.renderer.clear();
  }

  // Proper rotation (determinant +1): SAE (X forward, Y right, Z down)
  // maps to Three.js display (Y, -Z, -X). Keep model points in SAE mm;
  // this transform belongs only at the rendering boundary.
  point([x, y, z]) { return new this.T.Vector3(y, -z, -x); }

  line(points, color, dashed = false) {
    const geometry = new this.T.BufferGeometry().setFromPoints(points.map(point => this.point(point)));
    const material = dashed ? new this.T.LineDashedMaterial({ color, dashSize: this.scale * 0.025, gapSize: this.scale * 0.018 }) : new this.T.LineBasicMaterial({ color });
    const line = new this.T.Line(geometry, material);
    if (dashed) line.computeLineDistances();
    this.group.add(line);
  }

  update(dimensions, sample) {
    this.clear();
    this.dimensions = dimensions;
    this.scale = Math.max(dimensions.wheelbase, dimensions.track);
    const { wheelbase: L, track: T } = dimensions;
    const style = getComputedStyle(document.body);
    const muted = document.documentElement.dataset.theme === 'dark' ? '#636f80' : '#c2bbc6';
    this.line([[0, -T * 0.64, 0], [0, T * 0.64, 0]], muted);
    this.line([[-L, -T * 0.64, 0], [-L, T * 0.64, 0]], muted, true);
    this.line([[this.scale * 0.12, 0, 0], [-L - this.scale * 0.12, 0, 0]], muted, true);
    // Wheel glyph size is illustrative, not an input to the calculation.
    const length = this.scale * 0.13, width = length * 0.25;
    for (const side of [-1, 1]) {
      const h = (side < 0 ? sample.left_heading_deg : sample.right_heading_deg) * Math.PI / 180;
      const at = (longitudinal, lateral) => [longitudinal * Math.cos(h) - lateral * Math.sin(h), side * T / 2 + longitudinal * Math.sin(h) + lateral * Math.cos(h), 0];
      const color = style.getPropertyValue(side < 0 ? '--bench-left' : '--bench-right').trim();
      this.line([at(-length/2,-width/2), at(length/2,-width/2), at(length/2,width/2), at(-length/2,width/2), at(-length/2,-width/2)], color);
      this.line([at(-length * 0.7, 0), at(length * 0.85, 0)], color);
      this.line([at(length * 0.6, -width * 0.45), at(length * 0.85, 0), at(length * 0.6, width * 0.45)], color);
    }
    this.render();
  }

  reset() { this.yaw = 0.48; this.pitch = 0.76; this.zoom = 1; this.render(); }

  render() {
    if (!this.dimensions) return;
    const width = Math.max(this.host.clientWidth, 1), height = Math.max(this.host.clientHeight, 1);
    const { wheelbase: L, track: T } = this.dimensions;
    const aspect = width / height;
    const extent = Math.max(L * 0.7, T * 0.8 / aspect) / this.zoom;
    this.camera.left = -extent * aspect; this.camera.right = extent * aspect;
    this.camera.top = extent; this.camera.bottom = -extent;
    this.camera.far = this.scale * 12;
    this.camera.near = Math.max(this.scale * 0.00001, Number.MIN_VALUE);
    const distance = this.scale * 3;
    this.camera.position.set(distance * Math.sin(this.yaw) * Math.cos(this.pitch), distance * Math.sin(this.pitch), L / 2 + distance * Math.cos(this.yaw) * Math.cos(this.pitch));
    this.camera.lookAt(0, 0, L / 2);
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
    this.renderer.render(this.scene, this.camera);
  }
}

async function prepareViewer() {
  try {
    const THREE = await import('../vendor/three/three.module.js');
    viewer = new NominalViewer(THREE);
    if (result) { $('scene-empty').hidden = true; $('reset-view').disabled = false; showSample(); }
  } catch {
    disablePreview('3D preview unavailable: this browser could not load the bundled renderer or start WebGL. Calculated numbers and chart remain available.');
  }
}

for (const id of inputIds) $(id).addEventListener('input', () => {
  invalidate();
  referenceLoaded = false;
  $('input-origin').textContent = 'User-entered dimensions. Calculate to refresh the reference.';
  notice('Inputs changed. Calculate the reference to display updated results.');
});
$('benchmark-form').addEventListener('submit', calculate);
$('load-reference').addEventListener('click', () => {
  invalidate();
  $('wheelbase').value = '1600'; $('front-track').value = '1200'; $('inside-limit').value = '30';
  referenceLoaded = true;
  $('input-origin').textContent = 'Reference example loaded · not measured vehicle geometry.';
  notice('Reference example loaded. Choose Calculate reference to run the shared engine.');
});
$('sample-slider').addEventListener('input', showSample);
$('center-sample').addEventListener('click', () => {
  if (!result) return;
  $('sample-slider').value = String(result.samples.findIndex(sample => sample.inside_angle_deg === 0));
  showSample();
});
$('reset-view').addEventListener('click', () => viewer?.reset());
new MutationObserver(() => {
  if (result) { showSample(); drawChart(); }
}).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
window.addEventListener('load', () => { if (result && !chartReady) drawChart(); });

$('calculate').disabled = false;
$('load-reference').disabled = false;
notice('Enter dimensions or load the reference example. Unknown inputs remain blank.');
prepareViewer();
engineHealth().then(() => {
  if (!activeRequest) engineStatus('Calculation engine ready', true);
}).catch(error => {
  if (!activeRequest && !result) { engineStatus('Calculation engine unavailable'); notice(`${error.code}: ${error.message}`, true); }
});
