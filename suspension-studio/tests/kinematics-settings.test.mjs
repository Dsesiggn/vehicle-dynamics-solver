import test from 'node:test';
import assert from 'node:assert/strict';
import {newDesign} from '../src/model.js';
import {importDesign} from '../src/migration.js';
import {newKinematicsSettings,completeKinematicsSettings,validateKinematicsSettings} from '../src/kinematics-settings.js';

test('Unknown physical measurements stay null and existing v2 geometry survives settings round-trip',()=>{
  const design=newDesign(), before=structuredClone(design.axles);
  design.kinematics=newKinematicsSettings();
  assert.equal(design.kinematics.axles.front.staticCamberDeg,null);
  assert.deepEqual(design.kinematics.axles.rear.rockerAxis,[null,null,null]);
  assert.deepEqual(validateKinematicsSettings(design.kinematics),[]);
  const loaded=importDesign(JSON.parse(JSON.stringify(design))).design;
  assert.deepEqual(loaded.axles,before);
  assert.deepEqual(loaded.kinematics,design.kinematics);
});

test('Optional added inputs complete earlier settings without changing entered measurements',()=>{
  const older=newKinematicsSettings();delete older.forceReference;
  for(const axle of Object.values(older.axles))for(const key of ['arbModel','arbAttachment','brakeReaction','driveReaction'])delete axle[key];
  older.axles.front.staticCamberDeg=-1.5;const before=structuredClone(older);
  const completed=completeKinematicsSettings(older);
  assert.equal(completed.axles.front.staticCamberDeg,-1.5);
  assert.equal(completed.forceReference.cgHeightMm,null);
  assert.equal(completed.axles.front.arbAttachment,null);
  completed.axles.front.rockerAxis[0]=1;
  assert.deepEqual(older,before);
});

test('Invalid physical inputs are rejected while explicit zeros retain their meanings',()=>{
  const valid=newKinematicsSettings();valid.axles.front.staticCamberDeg=0;valid.forceReference.frontDriveFraction=0;
  assert.deepEqual(validateKinematicsSettings(valid),[]);
  for(const change of [s=>s.forceReference.cgHeightMm=0,s=>s.forceReference.frontBrakeFraction=1.1,s=>s.axles.front.rockerAxis=[0,0,0],s=>s.axles.front.staticCamberDeg='0',s=>s.axles.front.arbAttachment='guessed']){
    const s=structuredClone(valid);change(s);assert.ok(validateKinematicsSettings(s).length);
  }
});
