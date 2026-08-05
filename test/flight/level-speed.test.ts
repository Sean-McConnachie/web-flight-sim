/**
 * Maximum level speed, and the elevator travel over a fuel burn.
 *
 * THE MEASUREMENT. The maximum level speed is the speed at which the drag of
 * level flight equals the thrust of a full throttle. Two independent methods
 * measure it here and the test compares them.
 *
 *   1. trimMaxLevelSpeed of src/aircraft/trim.ts solves the three trim equations
 *      with the throttle held at its stop and the speed free. It reaches the
 *      answer in five Newton steps and it has no ground under it, so it can work
 *      at a true sea level.
 *   2. The harness flies the aircraft at a full throttle with the altitude hold
 *      engaged and waits for the speed to stop changing.
 *
 * The two must agree. If they do not, one of them is wrong, and a target that
 * only one method reaches proves nothing.
 *
 * WHY THE FLIGHT RUNS AT 200 m AND NOT AT SEA LEVEL. The ground plane of
 * src/physics/contact.ts sits at NED z = 0, which is the same height the
 * reference data calls sea level. An aircraft flown at zero altitude drags its
 * nacelles along the ground. The flight therefore runs at 200 m, where the
 * density is 2 percent lower, and the trim solver reports both altitudes so the
 * reader can see what those 200 m cost.
 */

import { afterAll, describe, expect, it } from 'vitest';

import { trimLevelFlight, trimMaxLevelSpeed } from '@/aircraft/trim';
import type { TrimCondition } from '@/aircraft/trim';
import { FUEL_CAPACITY, me262Mass } from '@/aircraft/me262/mass';
import { MAC } from '@/aircraft/me262/geometry';
import { kmhToMs, msToKmh, toDeg } from '@/math/units';
import {
  createFlightTest,
  describeSample,
  flyUntilSteady,
  note,
  passed,
  placeInAir,
  printReport,
  record,
  steadyCriteria,
} from './harness';

/** Maximum level speed at sea level. CONVENTIONS section 8, firm. */
const TARGET_SEA_LEVEL = kmhToMs(827); // m/s
/** Maximum level speed at 6000 m. CONVENTIONS section 8, firm. */
const TARGET_AT_6000 = kmhToMs(870); // m/s
/** docs/validation.md gives 5 percent for both. */
const SPEED_TOLERANCE = 0.05;

/** The altitude the low level run flies at. See the module comment. */
const LOW_ALTITUDE = 200; // m

const clean: TrimCondition = {
  altitude: 0,
  speed: 240,
  flapSetting: 'up',
  gearDown: false,
  fuelMass: FUEL_CAPACITY,
};

afterAll(() => {
  printReport('LEVEL SPEED');
});

describe('maximum level speed', () => {
  it('holds the published speed at sea level', () => {
    const trim = trimMaxLevelSpeed({ ...clean, altitude: 0 });
    expect(trim.converged).toBe(true);
    const m = record({
      name: 'max level speed, sea level',
      measured: msToKmh(trim.speed),
      target: msToKmh(TARGET_SEA_LEVEL),
      tolerance: SPEED_TOLERANCE,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: `Mach ${trim.mach.toFixed(3)}, thrust ${trim.thrust.toFixed(0)} N, alpha ${toDeg(trim.alpha).toFixed(2)} deg`,
    });
    expect(passed(m)).toBe(true);
  });

  it('holds the published speed at 6000 m', () => {
    const trim = trimMaxLevelSpeed({ ...clean, altitude: 6000 });
    expect(trim.converged).toBe(true);
    const m = record({
      name: 'max level speed, 6000 m',
      measured: msToKmh(trim.speed),
      target: msToKmh(TARGET_AT_6000),
      tolerance: SPEED_TOLERANCE,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: `Mach ${trim.mach.toFixed(3)}, thrust ${trim.thrust.toFixed(0)} N`,
    });
    expect(passed(m)).toBe(true);
  });

  it('flies at the speed the trim solver reports', () => {
    const trim = trimMaxLevelSpeed({ ...clean, altitude: LOW_ALTITUDE });
    expect(trim.converged).toBe(true);

    const test = createFlightTest();
    placeInAir(test, {
      altitude: LOW_ALTITUDE,
      // The run starts two percent slow, so the last part of the acceleration is
      // the part that decides the answer.
      speed: trim.speed * 0.98,
      pitch: trim.pitch,
      flapSetting: 'up',
      gearDown: false,
    });
    test.command.altitude = LOW_ALTITUDE;
    test.command.throttle = 1;
    test.command.trimElevator = trim.elevator;

    const result = flyUntilSteady(test, 400, steadyCriteria({ speedSlope: 0.01 }));
    note(`flight: ${describeSample(result.mean)}`);
    note(
      `steady=${result.steady} after ${result.seconds.toFixed(0)} s, ` +
        `speed slope ${result.speedSlope.toExponential(2)} m/s2`,
    );
    for (const engine of test.aircraft.state.engines) {
      expect(engine.state).toBe('running');
      expect(engine.damage).toBe(0);
    }
    expect(result.steady).toBe(true);

    const m = record({
      name: 'trim against flight, 200 m',
      measured: msToKmh(result.mean.speed),
      target: msToKmh(trim.speed),
      tolerance: 0.02,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: 'the solver and the aircraft must agree, whatever the target says',
    });
    expect(passed(m)).toBe(true);
  });

  it('flies at the speed the trim solver reports at 6000 m', () => {
    const trim = trimMaxLevelSpeed({ ...clean, altitude: 6000 });
    const test = createFlightTest();
    placeInAir(test, {
      altitude: 6000,
      speed: trim.speed * 0.98,
      pitch: trim.pitch,
      flapSetting: 'up',
      gearDown: false,
    });
    test.command.altitude = 6000;
    test.command.throttle = 1;
    test.command.trimElevator = trim.elevator;

    const result = flyUntilSteady(test, 400, steadyCriteria({ speedSlope: 0.01 }));
    note(`flight: ${describeSample(result.mean)}`);
    expect(result.steady).toBe(true);
    const m = record({
      name: 'trim against flight, 6000 m',
      measured: msToKmh(result.mean.speed),
      target: msToKmh(trim.speed),
      tolerance: 0.02,
      toleranceKind: 'fraction',
      unit: 'km/h',
    });
    expect(passed(m)).toBe(true);
  });
});

describe('center of gravity travel over a fuel burn', () => {
  it('reports the elevator change from full fuel to empty', () => {
    const altitude = 3000;
    const speed = 160;
    const full = trimLevelFlight({ ...clean, altitude, speed, fuelMass: FUEL_CAPACITY });
    const empty = trimLevelFlight({ ...clean, altitude, speed, fuelMass: 0 });
    expect(full.converged).toBe(true);
    expect(empty.converged).toBe(true);

    const cgFull = me262Mass(FUEL_CAPACITY).cgFromNose;
    const cgEmpty = me262Mass(0).cgFromNose;
    const travel = cgEmpty - cgFull;
    note(
      `center of gravity: full ${cgFull.toFixed(4)} m, empty ${cgEmpty.toFixed(4)} m, ` +
        `travel ${(travel * 1000).toFixed(1)} mm, ${((travel / MAC) * 100).toFixed(2)} percent of the chord`,
    );
    note(
      `full fuel  ${full.mass.toFixed(0)} kg: elevator ${full.elevator.toFixed(4)}, alpha ${toDeg(full.alpha).toFixed(2)} deg`,
    );
    note(
      `no fuel    ${empty.mass.toFixed(0)} kg: elevator ${empty.elevator.toFixed(4)}, alpha ${toDeg(empty.alpha).toFixed(2)} deg`,
    );

    // The elevator change has no published target. The test records it for bead
    // b33 and checks only that it stays small, because a fuel burn that needs a
    // large part of the elevator travel is an aircraft a pilot cannot trim.
    const change = empty.elevator - full.elevator;
    record({
      name: 'elevator change, full fuel to empty',
      measured: change,
      target: 0,
      tolerance: 0.25,
      toleranceKind: 'absolute',
      unit: 'command',
      note:
        `center of gravity moves ${(travel * 1000).toFixed(0)} mm ` +
        `${travel > 0 ? 'aft' : 'forward'}. No published target. ` +
        'A positive change means the empty aircraft needs more nose up elevator.',
    });
    expect(Math.abs(change)).toBeLessThan(0.25);
  });
});
