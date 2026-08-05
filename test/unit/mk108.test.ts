/**
 * Tests of the four MK 108 of src/weapons/mk108.ts.
 *
 * The tests cover the cyclic rate, the ammunition, the recoil force and the
 * moments the recoil makes. They run in Node with no GPU.
 *
 *
 * THE RECOIL, WORKED OUT BY HAND
 *
 * One round throws a 0.330 kg shell at 505 m/s and about 26 g of propellant gas
 * at 1.25 times that speed:
 *
 *   impulse = 0.330 * 505 + 0.026 * 1.25 * 505 = 166.65 + 16.41 = 183.06 N s
 *
 * At 650 rounds per minute, which is 10.833 rounds per second, the mean force
 * of one gun is
 *
 *   1983.2 N per gun, and 7932.7 N for four.
 *
 * At the loaded mass of 6396 kg that is 1.240 m/s2, which is 0.126 g. The two
 * Jumo 004 make about 6 kN of thrust at 800 km/h, so a full burst takes more
 * than the engines give at that speed.
 */

import { Vector3 } from 'three';
import { describe, expect, it } from 'vitest';

import { AMMUNITION_ROUNDS, FUEL_CAPACITY, me262Mass } from '@/aircraft/me262/mass';
import { G0 } from '@/math/units';
import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import { createMassProperties, createState, stepRK4 } from '@/physics/rigidbody';
import {
  CYCLIC_RATE,
  MUZZLE_VELOCITY,
  RECOIL_FORCE_PER_GUN,
  RECOIL_IMPULSE,
  ROUNDS_PER_SECOND,
  SHELL_MASS,
  SHOT_INTERVAL,
  createBattery,
  muzzleFlash,
  resetBattery,
  updateBattery,
} from '@/weapons/mk108';

const PHYSICS_DT = 1 / 240;

/** Runs the battery for a time with the trigger held, and counts the rounds. */
function holdTrigger(seconds: number, dt = PHYSICS_DT): { battery: ReturnType<typeof createBattery>; fired: number } {
  const battery = createBattery();
  let fired = 0;
  const steps = Math.round(seconds / dt);
  for (let i = 0; i < steps; i++) {
    fired += updateBattery(battery, true, dt);
  }
  return { battery, fired };
}

describe('the ammunition load', () => {
  it('carries 100 rounds for each upper gun and 80 for each lower gun', () => {
    const battery = createBattery();
    expect(battery.guns.length).toBe(4);
    expect(battery.guns[0].capacity).toBe(100);
    expect(battery.guns[1].capacity).toBe(100);
    expect(battery.guns[2].capacity).toBe(80);
    expect(battery.guns[3].capacity).toBe(80);
  });

  it('holds the 360 rounds that the mass model weighs', () => {
    const battery = createBattery();
    expect(battery.rounds).toBe(AMMUNITION_ROUNDS);
    expect(battery.rounds).toBe(360);
  });
});

describe('the cyclic rate', () => {
  it('fires 10.83 rounds per second from one gun, which is 650 per minute', () => {
    expect(ROUNDS_PER_SECOND).toBeCloseTo(CYCLIC_RATE / 60, 12);
    expect(ROUNDS_PER_SECOND).toBeCloseTo(10.8333, 4);
    expect(SHOT_INTERVAL).toBeCloseTo(0.09231, 5);
  });

  it('fires 43 rounds per second from all four guns', () => {
    // Two seconds of fire, which is short of the 7.4 s the lower boxes hold.
    const { fired } = holdTrigger(2);
    const expected = 4 * 2 * ROUNDS_PER_SECOND;
    expect(fired).toBeGreaterThanOrEqual(Math.floor(expected));
    // Each gun is ready when the trigger goes down, so the first press fires
    // one round from each of the four at once. That is four rounds of credit.
    expect(fired).toBeLessThanOrEqual(Math.ceil(expected) + 4);
  });

  it('does not follow the step rate', () => {
    // The same burst at 240 Hz and at 120 Hz must fire the same rounds.
    const fast = holdTrigger(3, 1 / 240).fired;
    const slow = holdTrigger(3, 1 / 120).fired;
    expect(Math.abs(fast - slow)).toBeLessThanOrEqual(1);
  });

  it('fires one round from each gun on the first step of a press', () => {
    const battery = createBattery();
    expect(updateBattery(battery, true, PHYSICS_DT)).toBe(4);
    expect(battery.rounds).toBe(356);
  });

  it('holds the guns ready while the trigger is up', () => {
    const battery = createBattery();
    for (let i = 0; i < 240; i++) updateBattery(battery, false, PHYSICS_DT);
    expect(battery.rounds).toBe(360);
    expect(updateBattery(battery, true, PHYSICS_DT)).toBe(4);
  });
});

describe('the ammunition running out', () => {
  it('empties the lower pair first, because they hold 80 rounds and not 100', () => {
    const battery = createBattery();
    // 80 rounds at 10.833 per second is 7.38 s. Run to 8 s.
    for (let i = 0; i < Math.round(8 / PHYSICS_DT); i++) {
      updateBattery(battery, true, PHYSICS_DT);
    }
    expect(battery.guns[2].rounds).toBe(0);
    expect(battery.guns[3].rounds).toBe(0);
    expect(battery.guns[0].rounds).toBeGreaterThan(0);
    expect(battery.guns[1].rounds).toBeGreaterThan(0);
  });

  it('fires exactly 360 rounds and then no more', () => {
    const battery = createBattery();
    let fired = 0;
    for (let i = 0; i < Math.round(20 / PHYSICS_DT); i++) {
      fired += updateBattery(battery, true, PHYSICS_DT);
    }
    expect(fired).toBe(360);
    expect(battery.rounds).toBe(0);
  });

  it('empties the boxes in about nine seconds of fire', () => {
    const battery = createBattery();
    let time = 0;
    while (battery.rounds > 0 && time < 30) {
      updateBattery(battery, true, PHYSICS_DT);
      time += PHYSICS_DT;
    }
    // 100 rounds at 10.833 per second is 9.2 s.
    expect(time).toBeGreaterThan(8.5);
    expect(time).toBeLessThan(10);
  });

  it('does nothing at all when every gun is empty', () => {
    const battery = createBattery();
    for (let i = 0; i < Math.round(20 / PHYSICS_DT); i++) {
      updateBattery(battery, true, PHYSICS_DT);
    }
    expect(battery.rounds).toBe(0);

    let fired = 0;
    for (let i = 0; i < 240; i++) fired += updateBattery(battery, true, PHYSICS_DT);
    expect(fired).toBe(0);
    expect(battery.roundsFired).toBe(0);
    // An empty gun makes no recoil either.
    expect(battery.recoil.force.length()).toBe(0);
    expect(battery.recoil.moment.length()).toBe(0);
    for (const gun of battery.guns) expect(gun.running).toBe(false);
  });

  it('fills every box again on a reset', () => {
    const battery = createBattery();
    for (let i = 0; i < 480; i++) updateBattery(battery, true, PHYSICS_DT);
    expect(battery.rounds).toBeLessThan(360);
    resetBattery(battery);
    expect(battery.rounds).toBe(360);
    expect(battery.recoil.force.length()).toBe(0);
  });
});

describe('the recoil', () => {
  it('takes 183 N s out of the gun for every round', () => {
    expect(RECOIL_IMPULSE).toBeCloseTo(183.06, 2);
    // The shell alone carries 166.65 N s of it.
    expect(SHELL_MASS * MUZZLE_VELOCITY).toBeCloseTo(166.65, 2);
  });

  it('makes 1983 N per gun and 7933 N from four', () => {
    expect(RECOIL_FORCE_PER_GUN).toBeCloseTo(1983.2, 1);
    const battery = createBattery();
    updateBattery(battery, true, PHYSICS_DT);
    expect(battery.recoil.force.x).toBeCloseTo(-4 * RECOIL_FORCE_PER_GUN, 6);
    expect(battery.recoil.force.x).toBeCloseTo(-7932.7, 1);
  });

  it('pushes along body minus x, which is straight back', () => {
    const battery = createBattery();
    updateBattery(battery, true, PHYSICS_DT);
    expect(battery.recoil.force.x).toBeLessThan(0);
    expect(battery.recoil.force.y).toBeCloseTo(0, 9);
    expect(battery.recoil.force.z).toBeCloseTo(0, 9);
  });

  it('slows a 6396 kg aircraft at 1.24 m/s2, which is 0.126 g', () => {
    const mass = me262Mass(FUEL_CAPACITY).mass;
    expect(mass).toBeCloseTo(6396, 0);
    const battery = createBattery();
    updateBattery(battery, true, PHYSICS_DT);
    const deceleration = -battery.recoil.force.x / mass;
    expect(deceleration).toBeCloseTo(1.24, 2);
    expect(deceleration / G0).toBeCloseTo(0.126, 3);
  });

  it('really decelerates a rigid body that carries only the gun wrench', () => {
    // The proof that the wrench works when src/aircraft/aircraft.ts adds it.
    // No gravity, no aerodynamics, no thrust: only the recoil.
    const mass = me262Mass(FUEL_CAPACITY);
    const properties = createMassProperties(mass.mass, mass.inertia);
    const state: RigidBodyState = createState();
    state.velocity.set(250, 0, 0);

    const battery = createBattery();
    const source = (_stage: RigidBodyState, _time: number, out: Wrench): void => {
      out.force.copy(battery.recoil.force);
      out.moment.copy(battery.recoil.moment);
    };

    const steps = Math.round(1 / PHYSICS_DT);
    for (let i = 0; i < steps; i++) {
      updateBattery(battery, true, PHYSICS_DT);
      stepRK4(state, properties, source, i * PHYSICS_DT, PHYSICS_DT);
    }

    // One second of fire at 1.24 m/s2 takes 1.24 m/s off the aircraft.
    expect(state.velocity.x).toBeLessThan(250);
    expect(250 - state.velocity.x).toBeCloseTo(1.24, 1);
    // The guns sit above the center of gravity, so a burst raises the nose.
    expect(state.angularVelocity.y).toBeGreaterThan(0);
  });
});

describe('the pitching moment of the recoil', () => {
  it('puts both pairs above the center of gravity, because the engines hang below it', () => {
    const battery = createBattery();
    // Body z points DOWN, so a negative z is above the center of gravity.
    expect(battery.guns[0].position.z).toBeCloseTo(-0.2933, 4);
    expect(battery.guns[2].position.z).toBeCloseTo(-0.0233, 4);
    // The muzzles stand ahead of the center of gravity.
    expect(battery.guns[0].position.x).toBeCloseTo(5.14, 4);
    expect(battery.guns[2].position.x).toBeCloseTo(4.98, 4);
  });

  it('makes a nose up moment from the lower pair, of the size their arm gives', () => {
    const battery = createBattery();
    // Empty the upper pair, so only the lower pair runs.
    battery.guns[0].rounds = 0;
    battery.guns[1].rounds = 0;
    updateBattery(battery, true, PHYSICS_DT);

    expect(battery.guns[0].running).toBe(false);
    expect(battery.guns[2].running).toBe(true);

    // M = r x F with F along minus x gives My = -z * |F|. The lower guns sit
    // 23.3 mm above the center of gravity, so My is positive, which is nose up.
    const arm = -battery.guns[2].position.z;
    expect(battery.recoil.moment.y).toBeCloseTo(2 * arm * RECOIL_FORCE_PER_GUN, 6);
    expect(battery.recoil.moment.y).toBeGreaterThan(0);
    expect(battery.recoil.moment.y).toBeCloseTo(92.4, 1);
  });

  it('makes a larger nose up moment from the upper pair, which sits higher', () => {
    const upper = createBattery();
    upper.guns[2].rounds = 0;
    upper.guns[3].rounds = 0;
    updateBattery(upper, true, PHYSICS_DT);
    expect(upper.recoil.moment.y).toBeCloseTo(1163.3, 1);

    const lower = createBattery();
    lower.guns[0].rounds = 0;
    lower.guns[1].rounds = 0;
    updateBattery(lower, true, PHYSICS_DT);
    expect(upper.recoil.moment.y).toBeGreaterThan(lower.recoil.moment.y);
  });

  it('makes no yaw moment while both guns of a pair still fire', () => {
    const battery = createBattery();
    updateBattery(battery, true, PHYSICS_DT);
    expect(battery.recoil.moment.z).toBeCloseTo(0, 9);
  });

  it('makes a yaw moment when one gun of a pair is empty', () => {
    const battery = createBattery();
    battery.guns[0].rounds = 0; // the upper LEFT gun
    updateBattery(battery, true, PHYSICS_DT);
    // The upper RIGHT gun still pushes, on its own, 85 mm right of the center
    // line. Mz = y * |F|, so the moment is positive and the nose yaws toward
    // the gun that still fires. A rearward pull on the right side carries the
    // right side aft, which swings the nose right.
    expect(battery.recoil.moment.z).toBeGreaterThan(0);
    expect(battery.recoil.moment.z).toBeCloseTo(0.085 * RECOIL_FORCE_PER_GUN, 6);
  });
});

describe('the muzzle flash', () => {
  it('is bright on the step of the shot and dark a tenth of a second later', () => {
    const battery = createBattery();
    updateBattery(battery, true, PHYSICS_DT);
    expect(muzzleFlash(battery.guns[0])).toBeGreaterThan(0.5);
    for (let i = 0; i < Math.round(0.1 / PHYSICS_DT); i++) {
      updateBattery(battery, false, PHYSICS_DT);
    }
    expect(muzzleFlash(battery.guns[0])).toBe(0);
  });
});

describe('the gun geometry', () => {
  it('puts every muzzle ahead of the center of gravity and inside the nose', () => {
    const battery = createBattery();
    for (const gun of battery.guns) {
      expect(gun.position.x).toBeGreaterThan(4.5);
      expect(gun.position.x).toBeLessThan(5.5);
      expect(Math.abs(gun.position.y)).toBeLessThan(0.12);
      expect(Math.abs(gun.position.z)).toBeLessThan(0.35);
    }
  });

  it('puts the pair of one row on either side of the center line', () => {
    const battery = createBattery();
    expect(battery.guns[0].position.y).toBeCloseTo(-battery.guns[1].position.y, 9);
    expect(battery.guns[2].position.y).toBeCloseTo(-battery.guns[3].position.y, 9);
    const span = new Vector3().subVectors(battery.guns[1].position, battery.guns[0].position);
    expect(span.length()).toBeCloseTo(0.17, 6);
  });
});
