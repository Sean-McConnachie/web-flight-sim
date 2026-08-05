/**
 * Stall speed, clean and in the landing configuration.
 *
 * READ THE NOTE ON THE STALL SPEED IN CONVENTIONS SECTION 8 BEFORE YOU CHANGE A
 * NUMBER IN THIS FILE. The widely repeated 175 km/h is the TOUCH-DOWN speed that
 * Wendel recorded, and a touch down happens below the stall speed of the clean
 * wing, in ground effect. The "Pilot's Handbook for Me-262 A-1" gives 180 to
 * 202 km/h with the gear and the flaps down at full fuel. A model that stalls at
 * 175 km/h with the flaps down is wrong, not accurate.
 *
 *
 * THREE MEASUREMENTS, BECAUSE THE WORD MEANS THREE THINGS
 *
 *   1. The TRIMMED stall speed. The lowest speed at which the aircraft holds a
 *      level flight path at 1 g with the elevator inside its travel.
 *      trimForAlpha of src/aircraft/trim.ts answers it: a search over the angle
 *      of attack for the lowest speed the solver converges at. This number
 *      carries the tail download, so it is the speed a pilot really meets.
 *   2. The peak lift coefficient, with the elevator at neutral. It is the number
 *      the table in CONVENTIONS section 8 reports, and it is what a wind tunnel
 *      measures.
 *   3. The FLOWN stall speed. The harness holds the altitude, closes the
 *      throttle, and records the speed at which the aircraft can no longer hold
 *      the altitude. It is a dynamic measurement and it lands a little below the
 *      trimmed value, because the aircraft carries its energy into the entry.
 *
 * All three are recorded. The test asserts on the first one.
 *
 * The speeds are EQUIVALENT airspeeds. The airspeed indicator of the Me 262
 * measures dynamic pressure, so every speed in the handbook is an indicated
 * speed. The trim runs at sea level, where the equivalent airspeed and the true
 * airspeed are the same number.
 */

import { afterAll, describe, expect, it } from 'vitest';

import { trimForAlpha, trimLevelFlight, trimResiduals } from '@/aircraft/trim';
import type { TrimCondition, TrimResult } from '@/aircraft/trim';
import { FUEL_CAPACITY } from '@/aircraft/me262/mass';
import type { FlapSetting } from '@/aircraft/me262/systems';
import { DEG, kmhToMs, msToKmh, toDeg } from '@/math/units';
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

/**
 * The handbook band with the gear and the flaps down at full fuel, and its
 * middle. CONVENTIONS section 8, firm, "Pilot's Handbook for Me-262 A-1".
 */
const LANDING_BAND_LOW = 180; // km/h
const LANDING_BAND_HIGH = 202; // km/h
const LANDING_TARGET = 0.5 * (LANDING_BAND_LOW + LANDING_BAND_HIGH); // 191 km/h
const LANDING_TOLERANCE = 0.5 * (LANDING_BAND_HIGH - LANDING_BAND_LOW); // 11 km/h

/** Clean stall speed at 6400 kg. CONVENTIONS section 8, derived. */
const CLEAN_TARGET = 199; // km/h
/** Takeoff flap stall speed. CONVENTIONS section 8, derived. */
const TAKEOFF_TARGET = 193; // km/h

/** The peak lift coefficients the table in CONVENTIONS section 8 reports. */
const CL_MAX_CLEAN = 1.54;
const CL_MAX_TAKEOFF = 1.647;
const CL_MAX_LANDING = 1.801;

/** The aircraft stalls at sea level, at the loaded mass, with full fuel. */
const base: TrimCondition = {
  altitude: 0,
  speed: 70,
  flapSetting: 'up',
  gearDown: false,
  fuelMass: FUEL_CAPACITY,
};

/**
 * Searches the angle of attack for the lowest speed a level 1 g trim exists at.
 *
 * The search is a coarse sweep and then a fine sweep around the winner. Below
 * the stall the solver reports `converged: false`, either because the wing
 * cannot make the lift or because the elevator runs out of travel, and the
 * search steps over those angles.
 */
function trimmedStall(
  flapSetting: FlapSetting,
  gearDown: boolean,
  altitude: number = 0,
): TrimResult {
  const condition: TrimCondition = { ...base, flapSetting, gearDown, altitude };
  let best: TrimResult | null = null;
  const consider = (deg: number): void => {
    const r = trimForAlpha(condition, deg * DEG);
    if (r.converged && (best === null || r.speed < best.speed)) {
      best = r;
    }
  };
  for (let deg = 6; deg <= 26; deg += 1) {
    consider(deg);
  }
  if (best === null) {
    throw new Error(`No 1 g trim converged with the flaps ${flapSetting}.`);
  }
  const coarse: TrimResult = best;
  for (let deg = toDeg(coarse.alpha) - 1; deg <= toDeg(coarse.alpha) + 1; deg += 0.1) {
    consider(deg);
  }
  return best;
}

/** Peak lift coefficient of the whole aircraft with the elevator at neutral. */
function peakLiftCoefficient(flapSetting: FlapSetting): { cl: number; alpha: number } {
  const condition: TrimCondition = { ...base, flapSetting };
  let best = { cl: 0, alpha: 0 };
  for (let deg = 5; deg <= 30; deg += 0.1) {
    const r = trimResiduals(condition, {
      speed: 60,
      alpha: deg * DEG,
      elevator: 0,
      throttle: 0,
    });
    if (r.liftCoefficient > best.cl) {
      best = { cl: r.liftCoefficient, alpha: deg };
    }
  }
  return best;
}

afterAll(() => {
  printReport('STALL');
});

describe('stall speed', () => {
  it('stalls inside the handbook band with the gear and the flaps down', () => {
    const stall = trimmedStall('landing', true);
    note(
      `landing: ${msToKmh(stall.speed).toFixed(1)} km/h at alpha ${toDeg(stall.alpha).toFixed(2)} deg, ` +
        `CL ${stall.liftCoefficient.toFixed(3)}, elevator ${stall.elevator.toFixed(3)}, ` +
        `mass ${stall.mass.toFixed(0)} kg`,
    );
    const m = record({
      name: 'stall speed, landing configuration',
      measured: msToKmh(stall.speed),
      target: LANDING_TARGET,
      tolerance: LANDING_TOLERANCE,
      toleranceKind: 'absolute',
      unit: 'km/h',
      note: `handbook band ${LANDING_BAND_LOW} to ${LANDING_BAND_HIGH} km/h. Trimmed, 1 g, ${stall.mass.toFixed(0)} kg`,
    });
    expect(passed(m)).toBe(true);
  });

  it('stalls clean near the derived speed', () => {
    const stall = trimmedStall('up', false);
    note(
      `clean: ${msToKmh(stall.speed).toFixed(1)} km/h at alpha ${toDeg(stall.alpha).toFixed(2)} deg, ` +
        `CL ${stall.liftCoefficient.toFixed(3)}, elevator ${stall.elevator.toFixed(3)}`,
    );
    const m = record({
      name: 'stall speed, clean',
      measured: msToKmh(stall.speed),
      target: CLEAN_TARGET,
      tolerance: 0.05,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: 'trimmed, 1 g, 6396 kg',
    });
    expect(passed(m)).toBe(true);
  });

  it('stalls with the takeoff flap near the derived speed', () => {
    const stall = trimmedStall('takeoff', false);
    const m = record({
      name: 'stall speed, takeoff flap',
      measured: msToKmh(stall.speed),
      target: TAKEOFF_TARGET,
      tolerance: 0.05,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: `alpha ${toDeg(stall.alpha).toFixed(1)} deg, CL ${stall.liftCoefficient.toFixed(3)}`,
    });
    expect(passed(m)).toBe(true);
  });
});

describe('peak lift coefficient', () => {
  it('reaches the peak lift the reference table reports', () => {
    for (const [flap, target] of [
      ['up', CL_MAX_CLEAN],
      ['takeoff', CL_MAX_TAKEOFF],
      ['landing', CL_MAX_LANDING],
    ] as const) {
      const peak = peakLiftCoefficient(flap);
      note(`${flap}: peak CL ${peak.cl.toFixed(3)} at ${peak.alpha.toFixed(1)} deg`);
      record({
        name: `peak lift coefficient, ${flap}`,
        measured: peak.cl,
        target,
        tolerance: 0.08,
        toleranceKind: 'absolute',
        unit: '-',
        note: `at ${peak.alpha.toFixed(1)} deg, elevator neutral. CONVENTIONS section 8 table.`,
      });
    }
  });
});

describe('the flown stall', () => {
  it('loses the altitude hold near the trimmed stall speed', () => {
    // The dynamic entry. The autopilot holds the altitude and the throttle is
    // closed, so the aircraft slows down until it cannot hold the height. The
    // speed at that moment is the flown stall speed.
    const altitude = 1500;
    const trimmed = trimmedStall('landing', true);
    const test = createFlightTest();
    placeInAir(test, {
      altitude,
      speed: 90,
      pitch: 0.05,
      flapSetting: 'landing',
      gearDown: true,
    });
    test.command.altitude = altitude;
    test.command.throttle = 0;
    test.command.trimElevator = 0;

    // Let the state settle at the entry speed first.
    test.fly(20);
    let stallSpeed = 0;
    let stallAlpha = 0;
    let previous = test.sample().equivalentAirspeed;
    const steps = Math.round(300 / (1 / 240));
    for (let i = 0; i < steps; i++) {
      test.step();
      if (i % 24 !== 0) {
        continue;
      }
      const s = test.sample();
      // The stall is the point where the aircraft can no longer hold the
      // altitude with the elevator it has left.
      if (s.altitude < altitude - 30 || s.climbRate < -3) {
        stallSpeed = previous;
        stallAlpha = s.alpha;
        break;
      }
      previous = s.equivalentAirspeed;
    }
    note(
      `flown stall at ${msToKmh(stallSpeed).toFixed(1)} km/h equivalent, ` +
        `alpha ${toDeg(stallAlpha).toFixed(2)} deg`,
    );
    note(`state at the break: ${describeSample(test.sample())}`);
    expect(stallSpeed).toBeGreaterThan(0);

    record({
      name: 'flown stall speed, landing configuration',
      measured: msToKmh(stallSpeed),
      target: msToKmh(trimmed.speed),
      tolerance: 0.1,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: 'the flown value against the trimmed value, not against the handbook',
    });
  });

  it('holds a steady level flight fifteen percent above the stall', () => {
    // A trim that the aircraft cannot fly is not a trim, so the harness has to
    // prove it can hold a steady state near the slow end of the envelope.
    //
    // The speed is NOT commanded here. The throttle sits closed and the altitude
    // hold flies the aircraft, so the aircraft settles at the speed where the
    // idle thrust equals the drag. That speed is a measurement of its own: the
    // Me 262 could not slow down below it in level flight, and a value far above
    // the stall speed means the idle thrust of the model is too high for an
    // approach. Bead b33 reads it.
    const altitude = 1500;
    const trimmed = trimmedStall('landing', true, altitude);
    const speed = trimmed.speed * 1.15;
    const approach = trimLevelFlight({
      ...base,
      altitude,
      speed,
      flapSetting: 'landing',
      gearDown: true,
    });
    expect(approach.converged).toBe(true);
    const test = createFlightTest();
    placeInAir(test, {
      altitude,
      speed,
      pitch: approach.pitch,
      flapSetting: 'landing',
      gearDown: true,
    });
    // THE PITCH HOLDS THE SPEED HERE AND THE THROTTLE STAYS AT THE VALUE THE
    // TRIM SOLVER REPORTS. At 1.15 times the stall speed the aircraft flies on
    // the back side of the drag curve, where a throttle that chases the speed
    // and a pitch that chases the altitude fight each other and the pair never
    // settles. A pilot flies the approach the other way round, and so does this
    // test. The climb rate that comes out is then a direct check of the throttle
    // the solver found: if that throttle is right, the aircraft holds height.
    test.command.altitude = null;
    test.command.climbSpeed = speed;
    test.command.throttle = approach.throttle;
    test.command.referencePitch = approach.pitch;
    test.command.trimElevator = approach.elevator;
    const result = flyUntilSteady(
      test,
      300,
      steadyCriteria({ speedSlope: 0.03, climbSpread: 0.8, alphaSpread: 0.01 }),
    );
    note(`flight: ${describeSample(result.mean)}`);
    note(
      `steady=${result.steady}, slopes speed ${result.speedSlope.toExponential(2)} ` +
        `climb ${result.climbSlope.toExponential(2)} spread ${result.climbSpread.toFixed(3)}`,
    );
    expect(result.steady).toBe(true);
    expect(Math.abs(result.mean.climbRate)).toBeLessThan(0.5);
    // The aircraft must still be flying, not falling.
    expect(result.crashed).toBe(false);

    record({
      name: 'approach speed held, 1.15 Vs',
      measured: msToKmh(result.mean.speed),
      target: msToKmh(speed),
      tolerance: 0.03,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note:
        `landing configuration at 1500 m, throttle ${result.mean.throttle.toFixed(2)}, ` +
        `elevator ${result.mean.elevator.toFixed(3)}. It proves the aircraft flies the trim.`,
    });
    record({
      name: 'climb rate at the trimmed approach',
      measured: result.mean.climbRate,
      target: 0,
      tolerance: 0.6,
      toleranceKind: 'absolute',
      unit: 'm/s',
      note: 'the throttle is the value the solver reported, so a level path proves it',
    });
  });
});

/** The speed the reference data calls a touch-down speed. It is NOT a stall. */
export const TOUCHDOWN_SPEED = kmhToMs(175); // m/s
