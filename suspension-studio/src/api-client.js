// Same application origin in both local and hosted modes. Never sends inputs to
// a third-party calculation service. From /suspension-studio/src/ -> /api/v1/.
const API_BASE = new URL('../../api/v1/', import.meta.url);

export class CalculationError extends Error {
  constructor(code, message) { super(message); this.name = 'CalculationError'; this.code = code; }
}

async function request(path, { signal, body } = {}) {
  if (!['http:', 'https:'].includes(API_BASE.protocol)) {
    throw new CalculationError('HTTP_REQUIRED', 'Open this workspace through the local application server. A file address cannot connect to the calculation engine.');
  }
  const controller = new AbortController();
  const cancel = () => controller.abort(signal?.reason);
  signal?.addEventListener('abort', cancel, { once: true });
  if (signal?.aborted) cancel();
  let timedOut = false;
  const timer = setTimeout(() => { timedOut = true; controller.abort(); }, 15000);
  try {
    const response = await fetch(new URL(path, API_BASE), {
      method: body ? 'POST' : 'GET',
      headers: body ? { 'Content-Type': 'application/json', Accept: 'application/json' } : { Accept: 'application/json' },
      body: body ? JSON.stringify(body) : undefined,
      signal: controller.signal,
      cache: 'no-store',
      credentials: 'same-origin',
    });
    if (!response.ok) {
      if (response.status === 404) throw new CalculationError('ENGINE_UNAVAILABLE', 'The calculation engine was not found at this address. Start the Python application server; a static-only server cannot calculate this benchmark.');
      if (response.status === 422) throw new CalculationError('INPUT_REJECTED', 'The calculation engine rejected these inputs. Check the dimensions and angle range.');
      throw new CalculationError('ENGINE_ERROR', `The calculation engine returned HTTP ${response.status}. No result has been retained.`);
    }
    const contentType = response.headers.get('content-type') || '';
    if (!contentType.includes('application/json')) throw new CalculationError('INVALID_RESPONSE', 'This address returned a page instead of a calculation response. A hosted server may be waking up; wait about a minute and retry. If this persists, check that this address runs the Python service.');
    try { return await response.json(); }
    catch { throw new CalculationError('INVALID_RESPONSE', 'The calculation engine returned unreadable data.'); }
  } catch (error) {
    if (timedOut) throw new CalculationError('ENGINE_TIMEOUT', 'The calculation engine did not respond within 15 seconds. A hosted server may be waking up; wait about a minute and retry. For a local session, check that the Python server is running.');
    if (signal?.aborted) throw error;
    if (error instanceof CalculationError) throw error;
    throw new CalculationError('CONNECTION_FAILED', 'The calculation engine could not be reached. Check that the application server is running, then try again.');
  } finally {
    clearTimeout(timer);
    signal?.removeEventListener('abort', cancel);
  }
}

export async function engineHealth(options = {}) {
  const data = await request('health', options);
  if (data.status !== 'ok') throw new CalculationError('ENGINE_NOT_READY', 'The calculation engine is not ready.');
  if (!Array.isArray(data.models) || !data.models.includes('ideal_ackermann_v1')) throw new CalculationError('MODEL_UNAVAILABLE', 'The calculation engine does not advertise the required ideal Ackermann model.');
  return data;
}

export async function calculateAckermann(inputs, options = {}) {
  const data = await request('ackermann', { ...options, body: inputs });
  const keys = ['inside_angle_deg', 'outside_angle_deg', 'left_heading_deg', 'right_heading_deg'];
  if (data.model !== 'ideal_ackermann_v1' || data.coordinate_system !== 'SAE_J670_Z_DOWN' || data.length_unit !== 'mm' || data.angle_unit !== 'deg' || !Array.isArray(data.samples) || data.samples.length !== inputs.inside_angles_deg.length) {
    throw new CalculationError('MODEL_MISMATCH', 'The engine response does not match the expected model, units, coordinate system, or sample count.');
  }
  for (let index = 0; index < data.samples.length; index += 1) {
    const sample = data.samples[index];
    if (!sample || !keys.every(key => typeof sample[key] === 'number' && Number.isFinite(sample[key])) || !['left', 'right', 'straight'].includes(sample.turn_direction) || Math.abs(sample.inside_angle_deg - inputs.inside_angles_deg[index]) > 1e-10) {
      throw new CalculationError('INVALID_SAMPLE', 'The calculation engine returned an invalid or out-of-order sample. No result has been retained.');
    }
  }
  return data;
}
