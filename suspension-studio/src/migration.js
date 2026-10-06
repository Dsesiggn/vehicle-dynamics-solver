import { clone, validateDesign, hardpointLabel, newHeaveTravel, newInterferenceSettings } from './model.js?v=0.2.8';
import { COORDINATE_SYSTEM, HARDPOINT_FRAME, LENGTH_UNIT, legacyToSAE } from './coordinates.js';

/** Pure, non-mutating import boundary. Never infer axes from coordinate signs. */
export function importDesign(value) {
  const design = clone(value), migrated = design?.version === 1;
  if (migrated) {
    if (['coordinateSystem', 'hardpointFrame', 'lengthUnit'].some(key => Object.hasOwn(design, key))) {
      throw Error('Ambiguous version 1 coordinate metadata. Import an original Studio v1 file or a complete v2 file.');
    }
    if (Object.hasOwn(design, 'obstacles')) throw Error('Version 1 files do not support obstacle geometry. Import the original suspension, then add packaging envelopes in SAE coordinates.');
    for (const a of Object.values(design.axles || {})) {
      if (a && Object.hasOwn(a, 'antiRollBar')) throw Error('Version 1 files do not support ARB points. Import the original suspension, then enter ARB points in SAE coordinates.');
      if (!a?.hardpoints || typeof a.hardpoints !== 'object') throw Error('Version 1 design has missing hardpoints.');
      for (const [key, point] of Object.entries(a.hardpoints)) {
        if (!Array.isArray(point) || point.length !== 3 || !point.every(Number.isFinite)) throw Error(`Invalid version 1 ${hardpointLabel(key)} coordinates.`);
        a.hardpoints[key] = legacyToSAE(point);
      }
    }
    Object.assign(design, { version: 2, coordinateSystem: COORDINATE_SYSTEM, hardpointFrame: HARDPOINT_FRAME, lengthUnit: LENGTH_UNIT });
  }
  // Older v1/v2 designs predate the heave checker; their travel starts at the
  // explicit application default. A supplied malformed value is rejected below.
  if (design && typeof design === 'object' && !Object.hasOwn(design, 'heaveTravel')) design.heaveTravel = newHeaveTravel();
  if (design && typeof design === 'object' && !Object.hasOwn(design, 'interference')) design.interference = newInterferenceSettings();
  // Width is a user-supplied tread measurement. Older designs provide no
  // evidence for it; preserve their hardpoints and mark each width unknown.
  if (design?.axles && typeof design.axles === 'object') {
    for (const a of Object.values(design.axles)) {
      if (a && typeof a === 'object' && !Array.isArray(a) && !Object.hasOwn(a,'tireTreadWidth')) a.tireTreadWidth = null;
    }
  }
  const errors = validateDesign(design);
  if (errors.length) throw Error(errors.join(' '));
  return { design, migrated };
}

export const STORAGE = 'suspension-studio.designs.v2';
export const LEGACY_STORAGE = 'suspension-studio.designs.v1';

/** Read v1 only until v2 is first saved. Preserve unreadable entries and original v1 storage. */
export function readLibrary(storage) {
  const saved = [], rejected = [];
  let migrated = 0;
  const raw = storage.getItem(STORAGE) ?? storage.getItem(LEGACY_STORAGE) ?? '[]';
  const values = JSON.parse(raw);
  if (!Array.isArray(values)) throw Error('Saved library is not an array.');
  for (const value of values) {
    try { const result = importDesign(value); saved.push(result.design); if (result.migrated) migrated++; }
    catch { rejected.push(value); }
  }
  return { saved, rejected, migrated };
}
