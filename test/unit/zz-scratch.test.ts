import { describe, expect, it } from 'vitest';

import type { AircraftInput } from '@/aircraft/aircraft';
import { createAircraft } from '@/aircraft/aircraft';
import { PHYSICS_DT } from '@/core/loop';

const DT = PHYSICS_DT;

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

function finite(a: ReturnType<typeof createAircraft>): boolean {
  const b = a.state.body;
  return (
    Number.isFinite(b.position.x) &&
    Number.isFinite(b.position.z) &&
    Number.isFinite(b.velocity.z) &&
    Number.isFinite(b.orientation.w) &&
    Number.isFinite(b.angularVelocity.y)
  );
}

describe('scratch', () => {
  it('underground repro', () => {
    const a = createAircraft();
    a.spawnOnRunway();
    a.state.body.position.set(0, 0, 50);
    const input = neutral();
    for (let i = 0; i < 60; i++) {
      a.fixedUpdate(input, DT);
      if (!finite(a)) {
        console.log('NaN at step', i);
        break;
      }
      if (i < 10 || i % 5 === 0) {
        console.log(
          i,
          'z',
          a.state.body.position.z.toExponential(3),
          'vz',
          a.state.body.velocity.z.toExponential(3),
          'w',
          a.state.body.angularVelocity.length().toExponential(3),
          'load',
          a.state.gear.legs[1].load.toExponential(3),
        );
      }
    }
    expect(true).toBe(true);
  });

  it('parked alpha', () => {
    const a = createAircraft();
    const input = neutral();
    for (let i = 0; i < 240 * 5; i++) a.fixedUpdate(input, DT);
    const alphas = a.assembly.surfaces.map((s) => (s.result.alpha * 180) / Math.PI);
    console.log('parked alpha deg', alphas.slice(0, 4).map((v) => v.toFixed(2)).join(' '));
    console.log('speed', a.assembly.surfaces[0].result.speed.toExponential(3));
    expect(true).toBe(true);
  });

  it('idle creep, brakes off', () => {
    const a = createAircraft();
    const input = neutral();
    input.startEngines = true;
    input.brakeLeft = 1;
    input.brakeRight = 1;
    for (let i = 0; i < Math.round(200 / DT); i++) {
      if (a.state.engines.every((e) => e.state === 'idle')) break;
      a.fixedUpdate(input, DT);
    }
    input.startEngines = false;
    for (let i = 0; i < Math.round(5 / DT); i++) a.fixedUpdate(input, DT);
    console.log('idle thrust per engine', a.state.engines[0].thrust.toFixed(1));
    input.brakeLeft = 0;
    input.brakeRight = 0;
    const start = a.state.body.position.x;
    for (let i = 0; i < Math.round(10 / DT); i++) a.fixedUpdate(input, DT);
    console.log(
      'idle roll in 10 s',
      (a.state.body.position.x - start).toFixed(3),
      'speed',
      a.state.body.velocity.x.toFixed(4),
    );
    expect(true).toBe(true);
  });

  it('brake hold at full power', () => {
    const a = createAircraft();
    const input = neutral();
    input.startEngines = true;
    input.brakeLeft = 1;
    input.brakeRight = 1;
    for (let i = 0; i < Math.round(200 / DT); i++) {
      if (a.state.engines.every((e) => e.state === 'idle')) break;
      a.fixedUpdate(input, DT);
    }
    input.startEngines = false;
    const ramp = Math.round(20 / DT);
    for (let i = 0; i < ramp; i++) {
      input.throttle = i / ramp;
      a.fixedUpdate(input, DT);
    }
    input.throttle = 1;
    for (let i = 0; i < Math.round(20 / DT); i++) a.fixedUpdate(input, DT);
    console.log(
      'full thrust per engine',
      a.state.engines[0].thrust.toFixed(1),
      a.state.engines[0].state,
      a.state.engines[0].rpm.toFixed(0),
    );
    const start = a.state.body.position.x;
    for (let i = 0; i < Math.round(10 / DT); i++) a.fixedUpdate(input, DT);
    console.log(
      'creep in 10 s',
      (a.state.body.position.x - start).toFixed(3),
      'speed',
      a.state.body.velocity.x.toFixed(4),
      'slip',
      a.state.gear.legs[1].slipRatio.toFixed(4),
    );
    expect(true).toBe(true);
  });
});
