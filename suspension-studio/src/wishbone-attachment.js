/** Numerical bounds for rigid wishbone transport, not manufacturing tolerances. */
export const WISHBONE_ATTACHMENT_NUMERICS = Object.freeze({
  minimumAxisLengthMm: 1e-7,
  minimumRadiusMm: 1e-7,
  ballReconstructionToleranceMm: 1e-3,
});

const vector = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
const sub = (a,b) => a.map((value,index) => value-b[index]);
const add = (a,b) => a.map((value,index) => value+b[index]);
const scale = (a,value) => a.map(component => component*value);
const dot = (a,b) => a.reduce((sum,value,index) => sum+value*b[index],0);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm = value => Math.hypot(...value);
const distance = (a,b) => norm(sub(a,b));

/**
 * Transport a rigidly attached pickup about the actual fore–aft pivot line.
 * Geometry is SAE axle-local mm. No coplanarity of the pickup is assumed.
 * See docs/ACTUATION-ATTACHMENT.md for the prior derivation and source review.
 * An off-circle solved ball is checked, never projected or overwritten.
 */
export function mapWishbonePickup(geometry,options={}) {
  const fail = reason => ({ok:false,reason:`Wishbone attachment: ${reason}`});
  for(const key of ['fore','aft','ballStatic','ballSolved','pickupStatic']) {
    if(!vector(geometry?.[key]))return fail(`${key} must contain three finite coordinates in mm.`);
  }
  const tolerance=options.reconstructionToleranceMm??WISHBONE_ATTACHMENT_NUMERICS.ballReconstructionToleranceMm;
  if(!Number.isFinite(tolerance)||tolerance<=0)return fail('reconstruction tolerance must be finite and positive in mm.');
  const {fore,aft,ballStatic,ballSolved,pickupStatic}=geometry;
  const axisVector=sub(aft,fore),axisLength=norm(axisVector);
  if(!Number.isFinite(axisLength)||axisLength<=WISHBONE_ATTACHMENT_NUMERICS.minimumAxisLengthMm)
    return fail('fore and aft pivots coincide or are too close to resolve the pivot axis.');
  const axis=scale(axisVector,1/axisLength),r0=sub(ballStatic,fore),r1=sub(ballSolved,fore);
  const v0=sub(r0,scale(axis,dot(axis,r0))),v1=sub(r1,scale(axis,dot(axis,r1)));
  const radius0=norm(v0),radius1=norm(v1);
  if(!Number.isFinite(radius0)||!Number.isFinite(radius1)||Math.min(radius0,radius1)<=WISHBONE_ATTACHMENT_NUMERICS.minimumRadiusMm)
    return fail('the static or solved ball joint lies on, or too close to, the pivot axis to resolve arm rotation.');
  // Common positive radial scaling cancels in atan2; normalized vectors avoid
  // unnecessarily large products without changing the signed rotation.
  const u0=scale(v0,1/radius0),u1=scale(v1,1/radius1);
  const angleRad=Math.atan2(dot(axis,cross(u0,u1)),dot(u0,u1));
  const rotate = value => {
    if(angleRad===0)return [...value];
    const cosine=Math.cos(angleRad),sine=Math.sin(angleRad);
    return add(add(scale(value,cosine),scale(cross(axis,value),sine)),scale(axis,dot(axis,value)*(1-cosine)));
  };
  const transport = point => angleRad===0?[...point]:add(fore,rotate(sub(point,fore)));
  const reconstructedBall=transport(ballStatic),reconstructionErrorMm=distance(reconstructedBall,ballSolved);
  if(!Number.isFinite(reconstructionErrorMm)||reconstructionErrorMm>tolerance)
    return fail(`solved ball joint differs from rigid arm rotation by ${reconstructionErrorMm} mm (limit ${tolerance} mm).`);
  const point=transport(pickupStatic);
  if(!vector(point))return fail('transported pickup coordinates exceed finite numerical range.');
  const rigidityErrorMm=Math.max(
    Math.abs(distance(point,fore)-distance(pickupStatic,fore)),
    Math.abs(distance(point,aft)-distance(pickupStatic,aft)),
    Math.abs(distance(point,ballSolved)-distance(pickupStatic,ballStatic)),
  );
  if(!Number.isFinite(rigidityErrorMm)||rigidityErrorMm>tolerance)
    return fail(`pickup rigidity residual ${rigidityErrorMm} mm exceeds the ${tolerance} mm limit.`);
  return {ok:true,point,angleRad,axis,reconstructionErrorMm,rigidityErrorMm};
}
