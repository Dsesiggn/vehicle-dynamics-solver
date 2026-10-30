/** Pure presentation helpers. No interpolation, derivatives or solver calculations. */
const finite = value => typeof value === 'number' && Number.isFinite(value);
const escape = value => String(value ?? '').replace(/[&<>"']/g, char => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[char]));
const FALLBACK_COLOR = 'var(--accent, #70364f)';

export function formatKinematicsValue(value) {
  if (!finite(value)) return 'Unavailable';
  if (value === 0) return '0';
  return Number(value.toPrecision(6)).toString();
}
// Axis labels are compact; selected values/tooltips retain six significant
// digits and CSV retains the unrounded numerical response.
const tickLabel=value=>value===0?'0':Math.abs(value)<.01||Math.abs(value)>=1e4?value.toExponential(2):Number(value.toPrecision(4)).toString();

function color(value) {
  const text = String(value ?? '').trim();
  const simple = /^(?:#[\da-f]{3}|#[\da-f]{4}|#[\da-f]{6}|#[\da-f]{8}|[a-z]+)$/i;
  const variable = /^var\(--[a-z][\w-]*(?:,\s*(?:#[\da-f]{3}|#[\da-f]{6}|[a-z]+))?\)$/i;
  return simple.test(text) || variable.test(text) ? text : FALLBACK_COLOR;
}

function dash(value) {
  if (value === undefined || value === null || value === '') return '';
  const values = Array.isArray(value) ? value : typeof value === 'string' ? value.trim().split(/[\s,]+/).map(Number) : [];
  return values.length && values.length <= 12 && values.every(n => finite(n) && n >= 0 && n <= 100) && values.some(n => n > 0)
    ? ` stroke-dasharray="${values.join(' ')}"` : '';
}

// Pad finite domains for constant traces too. Scaled arithmetic prevents an
// overflowing max-min from turning otherwise finite plot coordinates into NaN.
function domain(values) {
  if (!values.length) return [-1, 1];
  const lo = Math.min(...values), hi = Math.max(...values);
  const scale = Math.max(Math.abs(lo), Math.abs(hi)) || 1;
  const pad = lo === hi ? 0.08 : (hi / scale - lo / scale) * 0.06;
  const bounded = value => Math.max(-Number.MAX_VALUE, Math.min(Number.MAX_VALUE, value));
  let min = bounded(lo - scale * pad), max = bounded(hi + scale * pad);
  if (min === max) { min = lo - Number.MIN_VALUE; max = hi + Number.MIN_VALUE; }
  return [min, max];
}

function ratio(value, [min, max]) {
  const scale = Math.max(Math.abs(min), Math.abs(max)) || 1;
  return (value / scale - min / scale) / (max / scale - min / scale);
}
const ticks = ([min, max]) => Array.from({ length: 5 }, (_, i) => min * (1 - i / 4) + max * (i / 4));
const coord = value => Number(value.toFixed(3));
const unitLabel = (label, unit) => unit ? `${label} (${unit})` : label;

/**
 * Render a self-contained responsive SVG chart and accessible selected values.
 * rows: [{input: number, values: {[seriesId]: number|null}}]
 * series: [{id, label, color, dash?: '6 3'|[6,3]}]
 * selectedIndex defaults to the last row. Invalid supplied indices select none.
 * Every absent/nonfinite value splits its own trace; no values are zero-filled.
 * The controller may delegate pointer handling using data-sample-index.
 */
export function renderKinematicsChart({ id = 'chart', title = '', unit = '', xLabel = 'Input', rows = [], series = [], selectedIndex } = {}) {
  const data = Array.isArray(rows) ? rows : [], lines = Array.isArray(series) ? series : [];
  const selected = selectedIndex === undefined ? data.length - 1 : selectedIndex;
  const row = Number.isInteger(selected) && selected >= 0 && selected < data.length ? data[selected] : null;
  const stem = `kin-chart-${Array.from(String(id)).map(c => c.codePointAt(0).toString(16)).join('-') || 'empty'}`;
  const validRows = data.filter(item => finite(item?.input));
  const xs = validRows.map(item => item.input);
  const ys = validRows.flatMap(item => lines.map(line => item.values?.[line.id]).filter(finite));
  const xDomain = domain(xs), yDomain = domain(ys);
  const width = 640, height = 300, left = 100, right = 22, top = 20, bottom = 57;
  const plotWidth = width - left - right, plotHeight = height - top - bottom;
  const X = value => coord(left + plotWidth * ratio(value, xDomain));
  const Y = value => coord(top + plotHeight * (1 - ratio(value, yDomain)));
  const chartTitle = unitLabel(title, unit);
  const selectedInput = row && finite(row.input) ? `${xLabel}: ${formatKinematicsValue(row.input)}` : 'No sample selected';
  const values = lines.map(line => {
    const value = row && finite(row.input) ? row.values?.[line.id] : null;
    const shown = finite(value) ? unitLabel(formatKinematicsValue(value), unit) : 'Unavailable';
    return `<li data-series-id="${escape(line.id)}"><span class="kinematics-chart-key" aria-hidden="true"><svg width="23" height="10" viewBox="0 0 23 10"><line x1="0" y1="5" x2="23" y2="5" stroke="${escape(color(line.color))}" stroke-width="2.5"${dash(line.dash)}/></svg></span><span>${escape(line.label)}</span><strong>${escape(shown)}</strong></li>`;
  }).join('');
  const available = ys.length > 0;
  let svg = `<svg xmlns="http://www.w3.org/2000/svg" data-chart-id="${escape(id)}" viewBox="0 0 ${width} ${height}" width="100%" height="auto" style="display:block;max-width:100%;height:auto" role="img" aria-labelledby="${stem}-title ${stem}-description"><title id="${stem}-title">${escape(chartTitle)}</title><desc id="${stem}-description">${escape(available ? `${title} versus ${xLabel}. ${data.length} samples. Missing values are gaps. Selected values are listed above.` : `No finite results available for ${title}.`)}</desc>`;
  svg += `<rect x="${left}" y="${top}" width="${plotWidth}" height="${plotHeight}" fill="var(--surface, #fff)"/>`;
  if (available) {
    for (const value of ticks(yDomain)) svg += `<line x1="${left}" x2="${width - right}" y1="${Y(value)}" y2="${Y(value)}" stroke="var(--border, #e7e6e9)"/><text x="${left - 9}" y="${Y(value) + 4}" text-anchor="end" fill="var(--muted, #85838c)" font-size="13">${escape(tickLabel(value))}</text>`;
    for (const value of ticks(xDomain)) svg += `<line x1="${X(value)}" x2="${X(value)}" y1="${top}" y2="${height - bottom}" stroke="var(--border, #e7e6e9)"/><text x="${X(value)}" y="${height - bottom + 21}" text-anchor="middle" fill="var(--muted, #85838c)" font-size="13">${escape(tickLabel(value))}</text>`;
    if (yDomain[0] < 0 && yDomain[1] > 0) svg += `<line class="kinematics-chart-zero" x1="${left}" x2="${width - right}" y1="${Y(0)}" y2="${Y(0)}" stroke="var(--muted, #85838c)" stroke-opacity=".55"/>`;
    if (row && finite(row.input)) svg += `<line class="kinematics-chart-cursor" x1="${X(row.input)}" x2="${X(row.input)}" y1="${top}" y2="${height - bottom}" stroke="var(--text, #282a30)" stroke-opacity=".65" stroke-dasharray="3 4"/>`;
    for (const line of lines) {
      const stroke = escape(color(line.color));
      let path = '', previousValid = false, markers = '';
      data.forEach((item, index) => {
        const value = item?.values?.[line.id];
        if (!finite(item?.input) || !finite(value)) { previousValid = false; return; }
        const x = X(item.input), y = Y(value);
        path += `${previousValid ? 'L' : 'M'}${x},${y} `;
        previousValid = true;
        markers += `<circle class="kinematics-chart-point${index === selected ? ' selected' : ''}" data-series-id="${escape(line.id)}" data-sample-index="${index}" cx="${x}" cy="${y}" r="${index === selected ? 4.5 : 2}" fill="${stroke}"${index === selected ? ' stroke="var(--surface, #fff)" stroke-width="1.5"' : ''}><title>${escape(`${line.label}; ${xLabel}: ${formatKinematicsValue(item.input)}; ${unitLabel(title, unit)}: ${formatKinematicsValue(value)}`)}</title></circle>`;
      });
      if (path) svg += `<path class="kinematics-chart-trace" data-series-id="${escape(line.id)}" d="${path.trim()}" fill="none" stroke="${stroke}" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round"${dash(line.dash)}/>${markers}`;
    }
  } else {
    svg += `<text x="${left + plotWidth / 2}" y="${top + plotHeight / 2}" text-anchor="middle" fill="var(--muted, #85838c)" font-size="15">No available results</text>`;
  }
  svg += `<path d="M${left},${top} V${height - bottom} H${width - right}" fill="none" stroke="var(--muted, #85838c)"/><text x="${left + plotWidth / 2}" y="${height - 10}" text-anchor="middle" fill="var(--text, #282a30)" font-size="12">${escape(xLabel)}</text><text transform="translate(17 ${top + plotHeight / 2}) rotate(-90)" text-anchor="middle" fill="var(--text, #282a30)" font-size="12">${escape(unit || 'Value')}</text></svg>`;
  return `<figure class="kinematics-chart" data-chart-id="${escape(id)}"><figcaption class="kinematics-chart-heading"><h3>${escape(title)}</h3><span>${escape(selectedInput)}</span></figcaption><ul class="kinematics-chart-values" aria-label="Selected sample values">${values}</ul>${svg}<p class="kinematics-chart-note">${available ? 'Points are solved samples; gaps indicate unavailable results.' : 'Run a valid sweep to populate this chart.'}</p></figure>`;
}

/** RFC-style CSV quoting; strings that could become spreadsheet formulas stay text. */
function csvCell(value) {
  if (value === null || value === undefined || typeof value === 'number' && !finite(value)) return '';
  let text = String(value);
  if (typeof value === 'string' && /^[=+@\-\t\r]/.test(text)) text = `'${text}`;
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

/**
 * Export every input row without rounding or filling absent results.
 * channels: [{id,label,unit}]; metadata: string-keyed scalar values.
 * Leading # records contain format version, mode, input unit, metadata and IDs.
 * The header and data block then have columns: Input (unit), channel labels (unit).
 * CRLF line endings and escaped quotes/newlines are preserved.
 */
export function exportKinematicsCsv({ mode = '', inputLabel = 'Input', inputUnit = '', rows = [], channels = [], metadata = {} } = {}) {
  const fields = Array.isArray(channels) ? channels : [], data = Array.isArray(rows) ? rows : [];
  const records = [
    ['# Suspension Studio kinematics CSV', 1],
    ['# Mode', mode],
    ['# Input unit', inputUnit],
    ...Object.entries(metadata ?? {}).map(([key, value]) => [`# ${key}`, value]),
    ['# Channel IDs', 'input', ...fields.map(channel => channel.id)],
    [unitLabel(inputLabel, inputUnit), ...fields.map(channel => unitLabel(channel.title ? `${channel.label ?? channel.id} · ${channel.title}` : channel.label ?? channel.id, channel.unit))],
    ...data.map(row => [finite(row?.input) ? row.input : null, ...fields.map(channel => finite(row?.values?.[channel.id]) ? row.values[channel.id] : null)]),
  ];
  return `${records.map(record => record.map(csvCell).join(',')).join('\r\n')}\r\n`;
}
