// Same-origin Python calculation service. No browser solver fallback.
const SWEEP_URL = new URL('../../api/v1/steering/sweep', import.meta.url);
const finite = value => typeof value === 'number' && Number.isFinite(value);
const record = value => value !== null && typeof value === 'object' && !Array.isArray(value);
const point = value => Array.isArray(value) && value.length === 3 && value.every(finite);
const nullableNumber = value => value === null || finite(value);
const nullablePoint = value => value === null || point(value);

export class SteeringApiError extends Error {
  constructor(code, message) { super(message); this.name = 'SteeringApiError'; this.code = code; }
}
function invalid(message) { throw new SteeringApiError('INVALID_RESPONSE', `${message} No calculated result has been retained.`); }

function validateConstruction(value, solved) {
  if (!record(value) || value.pose !== (solved ? 'solved' : 'static')) invalid('The engine returned an invalid construction pose.');
  for (const side of ['left', 'right']) {
    const item = value[side];
    if (!record(item) || !['axisFoot', 'planAxisPoint', 'outer', 'inner'].every(key => nullablePoint(item[key])) ||
      !['planAngleDeg', 'spatialAngleDeg'].every(key => nullableNumber(item[key]) && (item[key] === null || item[key] >= 0 && item[key] <= 180))) invalid('The engine returned malformed steering-arm geometry.');
  }
  const intersection = value.intersection;
  if (!record(intersection) || !['finite', 'parallel', 'coincident', 'undefined'].includes(intersection.status)) invalid('The engine returned an invalid arm-line intersection.');
  if (intersection.status === 'finite') {
    if (!point(intersection.point) || !finite(intersection.rearAxleOffsetMm)) invalid('The engine returned a nonfinite arm-line intersection.');
  } else if (intersection.point !== null || intersection.rearAxleOffsetMm !== null) invalid('An undefined intersection must not carry coordinates.');
}
function validateRow(row, expectedAngle) {
  if (!record(row) || typeof row.ok !== 'boolean' || row.steeringWheelDeg !== expectedAngle || !nullableNumber(row.rack)) invalid('The engine returned an invalid or out-of-order steering position.');
  validateConstruction(row.construction, row.ok);
  if (!row.ok) {
    if (typeof row.reason !== 'string' || !row.reason.trim() || row.left !== null || row.right !== null || row.metrics !== null) invalid('An unavailable position must include a reason and no solved values.');
    return;
  }
  if (!finite(row.rack)) invalid('The engine returned an invalid rack displacement.');
  for (const side of ['left', 'right']) {
    const item = row[side];
    if (!record(item) || item.ok !== true || !['headingDeg', 'toeInDeg', 'deltaToeInDeg', 'camberDeg', 'rotationDeg', 'tieLength', 'residual'].every(key => finite(item[key])) ||
      !['outer', 'inner', 'wheelCenter', 'spindle'].every(key => point(item[key])) || item.tieLength <= 0) invalid('The engine returned incomplete or nonfinite wheel geometry.');
  }
  const metrics = row.metrics;
  if (!record(metrics) || !['right', 'left', 'straight', 'mixed'].includes(metrics.turn) || ![null, 'left', 'right'].includes(metrics.insideSide) ||
    !['totalToeInDeg', 'deltaTotalToeInDeg'].every(key => finite(metrics[key])) ||
    !['insideDeg', 'outsideDeg', 'idealOutsideDeg', 'ackermannErrorDeg', 'ackermannPercent', 'cotResidual'].every(key => nullableNumber(metrics[key]))) invalid('The engine returned invalid steering metrics.');
}

export function validateSteeringSweepResponse(data, design) {
  const expected = {model:'fixed_axis_steering_v1',coordinate_system:'SAE_J670_Z_DOWN',hardpoint_frame:'AXLE_LOCAL',length_unit:'mm',angle_unit:'deg',step_deg:5};
  if (!record(data) || Object.entries(expected).some(([key,value]) => data[key] !== value)) throw new SteeringApiError('MODEL_MISMATCH', 'The Python response does not match the required steering model, coordinates, units, or 5° step.');
  const limit = design?.steeringLimitDeg;
  if (!finite(limit) || limit <= 0 || limit > 3600 || limit % 5 !== 0 || !Array.isArray(data.rows) || data.rows.length !== 2 * limit / 5 + 1) invalid('The engine returned an incorrect steering sweep extent or row count.');
  data.rows.forEach((row,index) => validateRow(row, -limit + 5 * index));
  return data;
}

export async function requestSteeringSweep(design, {signal,fetchImpl=globalThis.fetch,endpoint=SWEEP_URL,timeoutMs=15000} = {}) {
  const url = new URL(endpoint);
  if (!['http:', 'https:'].includes(url.protocol)) throw new SteeringApiError('HTTP_REQUIRED', 'Open Steering Studio through the Python application server. A file address cannot reach the calculation engine.');
  const controller = new AbortController(), cancel = () => controller.abort(signal?.reason);
  signal?.addEventListener('abort', cancel, {once:true});
  if (signal?.aborted) cancel();
  let timedOut = false;
  const timer = setTimeout(() => {timedOut=true;controller.abort();}, timeoutMs);
  try {
    if (signal?.aborted) throw signal.reason || new DOMException('Request cancelled', 'AbortError');
    const response = await fetchImpl(url, {method:'POST',headers:{'Content-Type':'application/json',Accept:'application/json'},body:JSON.stringify({design}),signal:controller.signal,cache:'no-store',credentials:'same-origin'});
    if (signal?.aborted) throw signal.reason || new DOMException('Request cancelled', 'AbortError');
    if (timedOut) throw new SteeringApiError('ENGINE_TIMEOUT', 'The Python calculation engine did not respond within 15 seconds. Check the server and retry.');
    if (!response.ok) {
      if ([404,405,501].includes(response.status)) throw new SteeringApiError('ENGINE_UNAVAILABLE', 'This address does not provide the Python steering API. A static server can open the editor but cannot calculate steering.');
      if (response.status===422) throw new SteeringApiError('INPUT_REJECTED', 'The Python engine rejected this geometry. Review the dimensions and coordinate inputs, then retry.');
      throw new SteeringApiError('ENGINE_ERROR', `The Python engine returned HTTP ${response.status}. Calculated results have been cleared.`);
    }
    if (!(response.headers.get('content-type')||'').includes('application/json')) throw new SteeringApiError('ENGINE_UNAVAILABLE', 'This address returned a page instead of the Python steering response. Open the Python application server.');
    let data;
    try {data=await response.json();} catch {throw new SteeringApiError('INVALID_RESPONSE', 'The Python engine returned unreadable data. Calculated results have been cleared.');}
    if (signal?.aborted) throw signal.reason || new DOMException('Request cancelled', 'AbortError');
    if (timedOut) throw new SteeringApiError('ENGINE_TIMEOUT', 'The Python calculation engine did not respond within 15 seconds. Check the server and retry.');
    return validateSteeringSweepResponse(data,design);
  } catch (error) {
    if (signal?.aborted) throw error;
    if (timedOut) throw new SteeringApiError('ENGINE_TIMEOUT', 'The Python calculation engine did not respond within 15 seconds. Check the server and retry.');
    if (error instanceof SteeringApiError) throw error;
    throw new SteeringApiError('CONNECTION_FAILED', 'The Python engine could not be reached. Check the application server and retry.');
  } finally {clearTimeout(timer);signal?.removeEventListener('abort',cancel);}
}

/** Abort saves work; generation checks also reject servers that finish after cancellation. */
export function createSteeringSweepLoader({request=requestSteeringSweep,onState=()=>{},debounceMs=150} = {}) {
  let generation=0,controller=null;
  return {
    cancel() {generation++;controller?.abort();controller=null;onState({status:'idle',rows:[]});},
    async load(design) {
      const current=++generation;controller?.abort();controller=new AbortController();
      const active=controller,snapshot=structuredClone(design);
      onState({status:'loading',rows:[]});
      try {
        if(debounceMs>0)await new Promise(resolve=>{
          const finish=()=>{clearTimeout(timer);active.signal.removeEventListener('abort',finish);resolve();};
          const timer=setTimeout(finish,debounceMs);
          active.signal.addEventListener('abort',finish,{once:true});
        });
        if(current!==generation||active.signal.aborted)return {status:'superseded'};
        const data=await request(snapshot,{signal:active.signal});
        if(current!==generation||active.signal.aborted)return {status:'superseded'};
        onState({status:'ready',rows:data.rows,data});return {status:'ready',data};
      }catch(error){
        if(current!==generation||active.signal.aborted)return {status:'superseded'};
        onState({status:'error',rows:[],error});return {status:'error',error};
      }finally{if(current===generation)controller=null;}
    },
  };
}
