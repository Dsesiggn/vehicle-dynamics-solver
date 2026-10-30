// The Python service is the only calculation source for the dashboard.
const SWEEP_URL = new URL('../../api/v1/kinematics/sweep', import.meta.url);
const finite = value => typeof value === 'number' && Number.isFinite(value);
const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);
const point = value => Array.isArray(value) && value.length === 3 && value.every(finite);
const nullableNumber = value => value === null || finite(value);
const text = value => typeof value === 'string' && value.trim().length > 0;
const modes = ['heave', 'pitch', 'roll', 'steering'];
const timeoutMessage = 'The Python kinematics engine did not respond within the allowed time. A hosted server may be waking up; wait a minute and run the sweep again.';

export class KinematicsApiError extends Error {
  constructor(code, message) { super(message); this.name = 'KinematicsApiError'; this.code = code; }
}
function invalid(message) { throw new KinematicsApiError('INVALID_RESPONSE', `${message} No calculated result has been retained.`); }

function validateCorner(pose, axle) {
  if (!record(pose) || typeof pose.ok !== 'boolean' || !nullableNumber(pose.error) || pose.error !== null && pose.error < 0) invalid('The engine returned invalid corner diagnostics.');
  if (!pose.ok) {
    if (!text(pose.reason) || pose.points !== undefined || pose.rotation !== undefined || pose.spindle !== undefined) invalid('An unsolved corner must carry a reason and no solved geometry.');
    return;
  }
  if (!finite(pose.error) || !record(pose.points) || !point(pose.rotation) || !point(pose.spindle) || Math.abs(Math.hypot(...pose.spindle) - 1) > 1e-6 ||
      !['camber', 'toe', 'steer'].every(key => finite(pose[key])) || !nullableNumber(pose.compression) || !text(pose.alignmentReference)) invalid('The engine returned incomplete corner geometry or wheel orientation.');
  const expected = Object.keys(axle?.hardpoints ?? {});
  if (!expected.length || Object.keys(pose.points).length !== expected.length || expected.some(key => !point(pose.points[key]))) invalid('The returned hardpoints do not match the requested suspension.');
  if (!record(pose.actuator) || typeof pose.actuator.ok !== 'boolean') invalid('The engine returned invalid actuator availability.');
  if (pose.actuator.ok ? !finite(pose.compression) : pose.compression !== null || !text(pose.actuator.reason)) invalid('Unavailable actuation must carry a reason and no compression value.');
}

export function validateKinematicsSweepResponse(data, request) {
  const sweep = request?.sweep;
  const expected = { model: 'rigid_suspension_v1', coordinate_system: 'SAE_J670_Z_DOWN', hardpoint_frame: 'AXLE_LOCAL', length_unit: 'mm', angle_unit: 'deg' };
  if (!record(data) || Object.entries(expected).some(([key, value]) => data[key] !== value)) throw new KinematicsApiError('MODEL_MISMATCH', 'The response does not match the required suspension model, coordinate frame or units.');
  if (!record(sweep) || !modes.includes(sweep.mode) || data.mode !== sweep.mode || data.input_unit !== (['pitch', 'roll'].includes(sweep.mode) ? 'deg' : 'mm') ||
      !finite(sweep.min) || !finite(sweep.max) || sweep.min >= sweep.max || !Number.isInteger(sweep.samples) || sweep.samples < 3 || sweep.samples > 81 ||
      !Array.isArray(data.rows) || data.rows.length !== sweep.samples) invalid('The engine returned an incorrect sweep mode, extent or sample count.');
  if (!Array.isArray(data.channels) || !data.channels.length || data.channels.length > 2000) invalid('The engine returned no usable channel definitions.');
  const ids = new Set(), plots = new Map();
  for (const channel of data.channels) {
    if (!record(channel) || !['id', 'plot', 'title', 'label', 'unit', 'group', 'entity', 'description'].every(key => text(channel[key])) || ids.has(channel.id) ||
        !['Alignment', 'Actuation', 'Centers', 'Steering', 'Dimensions', 'Diagnostics', 'Hardpoints', 'Anti-effects', 'Anti-roll bar'].includes(channel.group) || !/^(?:front|rear|vehicle)(?:\.(?:left|right))?$/.test(channel.entity)) invalid('The engine returned invalid or duplicate channel metadata.');
    const definition = `${channel.title}\n${channel.unit}`;
    if (plots.has(channel.plot) && plots.get(channel.plot) !== definition) invalid('A chart would mix incompatible titles or units.');
    plots.set(channel.plot, definition); ids.add(channel.id);
  }
  data.rows.forEach((row, index) => {
    const expectedInput = sweep.min + (sweep.max - sweep.min) * index / (sweep.samples - 1);
    if (!record(row) || !finite(row.input) || Math.abs(row.input - expectedInput) > 1e-9 * Math.max(1, Math.abs(expectedInput)) || !record(row.values) || !record(row.unavailable) ||
        Object.keys(row.values).length !== ids.size || Object.keys(row.values).some(key => !ids.has(key)) || Object.keys(row.unavailable).some(key => !ids.has(key))) invalid('The engine returned invalid, missing or out-of-order samples.');
    for (const id of ids) {
      if (!nullableNumber(row.values[id]) || row.values[id] === null && !text(row.unavailable[id]) || row.values[id] !== null && Object.hasOwn(row.unavailable, id)) invalid('Unavailable channels must be blank and include a reason.');
    }
    if (!record(row.poses) || !record(row.diagnostics) || !['maxResidualMm', 'maxConditionNumber'].every(key => nullableNumber(row.diagnostics[key]) && (row.diagnostics[key] === null || row.diagnostics[key] >= 0))) invalid('The engine returned invalid pose diagnostics.');
    for (const name of ['front', 'rear']) {
      const pose = row.poses[name];
      if (!record(pose) || typeof pose.ok !== 'boolean') invalid('The engine omitted an axle pose.');
      for (const side of ['left', 'right']) validateCorner(pose[side], request.design?.axles?.[name]);
      if (pose.ok !== (pose.left.ok && pose.right.ok)) invalid('The axle pose availability contradicts its corners.');
    }
  });
  if (!['assumptions', 'warnings'].every(key => Array.isArray(data[key]) && data[key].every(text)) || !Array.isArray(data.coverage) ||
      data.coverage.some(item => !record(item) || !text(item.family) || !['available', 'unavailable'].includes(item.status) || typeof item.reason !== 'string')) invalid('The engine omitted its model assumptions or channel coverage.');
  return data;
}

export async function requestKinematicsSweep(request, { signal, fetchImpl = globalThis.fetch, endpoint = SWEEP_URL, timeoutMs = 60000 } = {}) {
  const url = new URL(endpoint);
  if (!['http:', 'https:'].includes(url.protocol)) throw new KinematicsApiError('HTTP_REQUIRED', 'Open Suspension Studio through its Python application server. A file address cannot reach the calculation engine.');
  const controller = new AbortController(), cancel = () => controller.abort(signal?.reason);
  signal?.addEventListener('abort', cancel, { once: true });
  if (signal?.aborted) cancel();
  let timedOut = false;
  const timer = setTimeout(() => { timedOut = true; controller.abort(); }, timeoutMs);
  try {
    if (signal?.aborted) throw signal.reason || new DOMException('Request cancelled', 'AbortError');
    const response = await fetchImpl(url, { method: 'POST', headers: { 'Content-Type': 'application/json', Accept: 'application/json' }, body: JSON.stringify(request), signal: controller.signal, cache: 'no-store', credentials: 'same-origin' });
    if (signal?.aborted) throw signal.reason || new DOMException('Request cancelled', 'AbortError');
    if (timedOut) throw new KinematicsApiError('ENGINE_TIMEOUT', timeoutMessage);
    if (!response.ok) {
      if ([404, 405, 501].includes(response.status)) throw new KinematicsApiError('ENGINE_UNAVAILABLE', 'This address does not provide the Python kinematics API. Start the application server; a static file server cannot calculate this dashboard.');
      if (response.status === 422) throw new KinematicsApiError('INPUT_REJECTED', 'The Python engine rejected the sweep. Review the hardpoints, mode range, sample count and required rig reference coordinates.');
      throw new KinematicsApiError('ENGINE_ERROR', `The Python engine returned HTTP ${response.status}. Calculated results have been cleared.`);
    }
    if (!(response.headers.get('content-type') || '').includes('application/json')) throw new KinematicsApiError('ENGINE_UNAVAILABLE', 'This address returned a page instead of a Python calculation response. The hosted server may be waking up; wait a minute and retry.');
    let data;
    try { data = await response.json(); } catch { throw new KinematicsApiError('INVALID_RESPONSE', 'The Python engine returned unreadable data.'); }
    if (signal?.aborted) throw signal.reason || new DOMException('Request cancelled', 'AbortError');
    if (timedOut) throw new KinematicsApiError('ENGINE_TIMEOUT', timeoutMessage);
    return validateKinematicsSweepResponse(data, request);
  } catch (error) {
    if (signal?.aborted) throw error;
    if (timedOut) throw new KinematicsApiError('ENGINE_TIMEOUT', timeoutMessage);
    if (error instanceof KinematicsApiError) throw error;
    throw new KinematicsApiError('CONNECTION_FAILED', 'The Python kinematics engine could not be reached. Check the application server and run the sweep again.');
  } finally { clearTimeout(timer); signal?.removeEventListener('abort', cancel); }
}

/** Snapshot requests and reject late results even if a server ignores cancellation. */
export function createKinematicsSweepLoader({ request = requestKinematicsSweep, onState = () => {} } = {}) {
  let generation = 0, controller = null;
  return {
    cancel() { generation++; controller?.abort(); controller = null; onState({ status: 'idle', rows: [] }); },
    async load(inputs) {
      const current = ++generation;
      controller?.abort(); controller = new AbortController();
      const active = controller, snapshot = structuredClone(inputs);
      onState({ status: 'loading', rows: [] });
      try {
        const data = await request(snapshot, { signal: active.signal });
        if (current !== generation || active.signal.aborted) return { status: 'superseded' };
        onState({ status: 'ready', rows: data.rows, data });
        return { status: 'ready', data };
      } catch (error) {
        if (current !== generation || active.signal.aborted) return { status: 'superseded' };
        onState({ status: 'error', rows: [], error });
        return { status: 'error', error };
      } finally { if (current === generation) controller = null; }
    },
  };
}
