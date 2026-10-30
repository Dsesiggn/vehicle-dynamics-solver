import test from 'node:test';
import assert from 'node:assert/strict';
import { renderKinematicsChart, exportKinematicsCsv, formatKinematicsValue } from '../src/kinematics-charts.js';

const series = [{ id: 'fl', label: 'Front left', color: '#4e8bad' }, { id: 'fr', label: 'Front right', color: 'var(--kin-front-right, #cb8651)', dash: [6, 3] }];
const render = (rows, extra = {}) => renderKinematicsChart({ id: 'camber', title: 'Camber', unit: 'deg', xLabel: 'Heave (mm)', rows, series, ...extra });
const pathFor = (html, id) => html.match(new RegExp(`<path class="kinematics-chart-trace" data-series-id="${id}" d="([^"]+)"`))?.[1];

test('each absent or nonfinite value breaks only its own trace instead of bridging failed samples', () => {
  const html = render([
    { input: -2, values: { fl: 0.1, fr: 1 } },
    { input: -1, values: { fl: null, fr: 2 } },
    { input: 0, values: { fl: 0.3, fr: 3 } },
    { input: 1, values: { fl: NaN, fr: 4 } },
    { input: 2, values: { fl: 0.5, fr: 5 } },
  ]);
  assert.equal((pathFor(html, 'fl').match(/M/g) ?? []).length, 3);
  assert.equal((pathFor(html, 'fl').match(/L/g) ?? []).length, 0);
  assert.equal((pathFor(html, 'fr').match(/M/g) ?? []).length, 1);
  assert.equal((pathFor(html, 'fr').match(/L/g) ?? []).length, 4);
  assert.doesNotMatch(html, /(?:cx|cy|x1|y1)="(?:NaN|Infinity)/);
  assert.match(html, /Missing values are gaps/);
});

test('an invalid input coordinate breaks all traces and is never coerced to zero', () => {
  const html = render([{ input: -1, values: { fl: 1 } }, { input: null, values: { fl: 2 } }, { input: 1, values: { fl: 3 } }], { selectedIndex: 1 });
  assert.equal((pathFor(html, 'fl').match(/M/g) ?? []).length, 2);
  assert.doesNotMatch(html, /data-sample-index="1"/);
  assert.match(html, /No sample selected/);
  assert.match(html, /Front left<\/span><strong>Unavailable/);
});

test('constant domains and a single solved sample produce finite coordinates and visible points', () => {
  for (const value of [0, 0.125, -0.125, 1e-8, 1e8, Number.MIN_VALUE, Number.MAX_VALUE]) {
    const html = render([{ input: value, values: { fl: value } }]);
    assert.match(html, /class="kinematics-chart-point selected"/);
    assert.doesNotMatch(html, /(?:NaN|Infinity)/);
    const [x, y] = pathFor(html, 'fl').slice(1).split(',').map(Number);
    assert.ok(x >= 77 && x <= 618, `x=${x}`);
    assert.ok(y >= 20 && y <= 243, `y=${y}`);
    assert.match(html, /class="kinematics-chart-cursor"/);
  }
});

test('linear input spacing is used rather than uniform row spacing', () => {
  const html = render([{ input: 0, values: { fl: 1 } }, { input: 1, values: { fl: 1 } }, { input: 4, values: { fl: 1 } }]);
  const xs = [...pathFor(html, 'fl').matchAll(/[ML]([\d.]+),/g)].map(match => Number(match[1]));
  assert.ok(Math.abs((xs[2] - xs[1]) / (xs[1] - xs[0]) - 3) < 0.0001);
});

test('selected sample exposes precise decimal values, unit labels and hover values', () => {
  const html = render([{ input: -0.25, values: { fl: 0.0123456, fr: -0.0000123 } }, { input: 0.5, values: { fl: null, fr: 0 } }], { selectedIndex: 0 });
  assert.match(html, /Heave \(mm\): -0.25/);
  assert.match(html, /0.0123456 \(deg\)/);
  assert.match(html, /Front left; Heave \(mm\): -0.25; Camber \(deg\): 0.0123456/);
  assert.match(html, /stroke-dasharray="6 3"/);
  assert.match(html, /aria-labelledby="kin-chart-/);
  assert.match(html, /viewBox="0 0 640 300"/);
  assert.equal(formatKinematicsValue(null), 'Unavailable');
  assert.equal(formatKinematicsValue('0'), 'Unavailable');
  assert.equal(formatKinematicsValue(-0), '0');
  assert.equal(formatKinematicsValue(0.30000000000000004), '0.3');
});

test('empty and wholly unavailable datasets show no fabricated zero series or selected values', () => {
  for (const rows of [[], [{ input: 0, values: { fl: null, fr: Infinity } }], [{ input: 0, values: {} }]]) {
    const html = render(rows);
    assert.match(html, /No available results/);
    assert.doesNotMatch(html, /class="kinematics-chart-trace"|class="kinematics-chart-point/);
    assert.match(html, /Front left<\/span><strong>Unavailable/);
  }
  const html = render([{ input: 0, values: { fl: 0, fr: null } }]);
  assert.match(html, /Front left<\/span><strong>0 \(deg\)/);
  assert.match(html, /Front right<\/span><strong>Unavailable/);
});

test('HTML labels and IDs are escaped and paint/dash arguments cannot inject markup or CSS URLs', () => {
  const bad = '"><script>alert(1)</script>&';
  const html = renderKinematicsChart({ id: bad, title: bad, unit: bad, xLabel: bad, rows: [{ input: 0, values: { [bad]: 1 } }], series: [{ id: bad, label: bad, color: 'url(https://example.com/x)', dash: '1" onload="evil' }] });
  assert.doesNotMatch(html, /<script>|url\(|onload=/);
  assert.match(html, /&quot;&gt;&lt;script&gt;/);
  assert.match(html, /stroke="var\(--accent, #70364f\)"/);
  assert.doesNotMatch(html, /stroke-dasharray="1/);
});

test('chart rendering is pure and does not sort or mutate input data', () => {
  const rows = Object.freeze([Object.freeze({ input: 2, values: Object.freeze({ fl: 1 }) }), Object.freeze({ input: -1, values: Object.freeze({ fl: 2 }) })]);
  const before = JSON.stringify(rows);
  assert.equal(render(rows), render(rows));
  assert.equal(JSON.stringify(rows), before);
});

test('CSV retains all sample rows, exact finite numeric precision and blanks for unknown values', () => {
  const csv = exportKinematicsCsv({ mode: 'heave', inputLabel: 'Heave', inputUnit: 'mm', channels: [{ id: 'fl', label: 'Front left camber', unit: 'deg' }, { id: 'mr', label: 'Wheel / damper', unit: 'mm/mm' }], rows: [
    { input: -25.4, values: { fl: 0.123456789012345, mr: null } },
    { input: 0, values: { fl: 0, mr: Infinity } },
    { input: 25.4, values: {} },
  ] });
  assert.match(csv, /# Mode,heave\r\n# Input unit,mm\r\n/);
  assert.match(csv, /# Channel IDs,input,fl,mr\r\n/);
  assert.match(csv, /Heave \(mm\),Front left camber \(deg\),Wheel \/ damper \(mm\/mm\)\r\n/);
  assert.ok(csv.endsWith('-25.4,0.123456789012345,\r\n0,0,\r\n25.4,,\r\n'));
  assert.doesNotMatch(csv, /Infinity|NaN|null|undefined/);
});

test('CSV quotes commas, quotes and newlines, keeps units/mode metadata, and protects spreadsheet text formulas', () => {
  const csv = exportKinematicsCsv({ mode: 'pitch,"test"', inputUnit: 'deg', metadata: { design: 'One\n"Two", Three', note: '=HYPERLINK("bad")' }, channels: [{ id: 'q', label: 'Camber, left', unit: 'deg' }], rows: [{ input: -1, values: { q: -0.5 } }] });
  assert.match(csv, /# Mode,"pitch,""test"""\r\n/);
  assert.match(csv, /# design,"One\n""Two"", Three"\r\n/);
  assert.match(csv, /# note,"'=HYPERLINK\(""bad""\)"\r\n/);
  assert.match(csv, /"Camber, left \(deg\)"\r\n/);
  assert.ok(csv.endsWith('-1,-0.5\r\n'));
});
