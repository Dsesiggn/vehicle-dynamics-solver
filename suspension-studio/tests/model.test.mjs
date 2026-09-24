import test from 'node:test';
import assert from 'node:assert/strict';
import {runChecks} from './checks.js';
for(const result of runChecks())test(result.name,()=>assert.ok(result.ok,result.error));
