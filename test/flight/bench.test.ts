import { describe, expect, it } from 'vitest';
import { createAircraft } from '@/aircraft/aircraft';
import type { AircraftInput } from '@/aircraft/aircraft';
import { PHYSICS_DT } from '@/core/loop';

function neutral(): AircraftInput {
  return {
    roll: 0,
    pitch: 0,
    yaw: 0,
    throttle: 0,
    brakeLeft: 0,
    brakeRight: 0,
    toggleGear: false,
    toggleFlapsUp: false,
    toggleFlapsDown: false,
    startEngines: false,
  };
}

describe('bench', () => {
  it('times the step', () => {
    const a = createAircraft();
    const input = neutral();
    input.startEngines = true;
    input.brakeLeft = 1;
    input.brakeRight = 1;
    const t0 = Date.now();
    let started = 0;
    for (let i = 0; i < Math.round(200 / PHYSICS_DT); i++) {
      if (a.state.engines.every((e) => e.state === 'idle')) {
        started = i;
        break;
      }
      a.fixedUpdate(input, PHYSICS_DT);
    }
    const t1 = Date.now();
    console.log('start took', started * PHYSICS_DT, 's sim,', t1 - t0, 'ms real');

    input.startEngines = false;
    input.brakeLeft = 0;
    input.brakeRight = 0;
    input.throttle = 1;
    a.state.body.position.set(0, 0, -6000);
    a.state.body.velocity.set(200, 0, 0);
    const t2 = Date.now();
    const steps = Math.round(60 / PHYSICS_DT);
    for (let i = 0; i < steps; i++) {
      a.fixedUpdate(input, PHYSICS_DT);
    }
    const t3 = Date.now();
    console.log('60 s of flight:', t3 - t2, 'ms real, per sim second', (t3 - t2) / 60, 'ms');
    console.log('speed', a.state.totals.trueAirspeed, 'alt', -a.state.body.position.z);
    expect(steps).toBeGreaterThan(0);
  });
});
