import { clone, validateDesign } from './model.js';
import { COORDINATE_SYSTEM, HARDPOINT_FRAME, LENGTH_UNIT, legacyToSAE } from './coordinates.js';

/** Pure, non-mutating import boundary. Never infer axes from coordinate signs. */
export function importDesign(value) {
  const design = clone(value), migrated = design?.version === 1;
  if (migrated) {
    if (['coordinateSystem', 'hardpointFrame', 'lengthUnit'].some(key => Object.hasOwn(design, key))) {
      throw Error('Ambiguous version 1 coordinate metadata. Import an original Studio v1 file or a complete v2 file.');
    }
    for (const a of Object.values(design.axles || {})) {
      if (!a?.hardpoints || typeof a.hardpoints !== 'object') throw Error('Version 1 design has missing hardpoints.');
      for (const [key, point] of Object.entries(a.hardpoints)) {
        if (!Array.isArray(point) || point.length !== 3 || !point.every(Number.isFinite)) throw Error(`Invalid version 1 ${key} coordinates.`);
        a.hardpoints[key] = legacyToSAE(point);
      }
    }
    Object.assign(design, { version: 2, coordinateSystem: COORDINATE_SYSTEM, hardpointFrame: HARDPOINT_FRAME, lengthUnit: LENGTH_UNIT });
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
