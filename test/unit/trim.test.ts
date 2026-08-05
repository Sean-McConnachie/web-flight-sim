/**
 * The trim solver.
 *
 * These tests check the SOLVER, not the aircraft. A number that comes out of
 * them says that the three residuals really reached zero, that a condition with
 * no answer is reported as having no answer, and that the same question always
 * gets the same answer. The flight tests in test/flight compare the aircraft
 * against the published data.
 */

import { describe, expect, it } from 'vitest';

import {
  trimForAlpha,
  trimLevelFlight,
  trimMaxLevelSpeed,
  trimResiduals,
  trimSteadyClimb,
} from '@/aircraft/trim';
import type { TrimCondition } from '@/aircraft/trim';
import { ELEVATOR_LIMIT } from '@/aircraft/aircraft';
import { FUEL_CAPACITY } from '@/aircraft/me262/mass';
import { DEG, G0, toDeg } from '@/math/units';

/** One cruise condition, used by most of the tests. The altitude is cheap to reuse. */
const cruise: TrimCondition = {
  altitude: 3000,
  speed: 180,
  flapSetting: 'up',
  gearDown: false,
  fuelMass: FUEL_CAPACITY,
};

describe('trimLevelFlight', () => {
  it('reaches a residual far below one part per million of the weight', () => {
    const r = trimLevelFlight(cruise);
    expect(r.converged).toBe(true);
    expect(r.residual).toBeLessThan(1e-8);
    expect(r.iterations).toBeLessThan(15);
  });

  it('holds the flight path level and the pitch equal to the angle of attack', () => {
    const r = trimLevelFlight(cruise);
    expect(r.flightPathAngle).toBe(0);
    expect(r.climbRate).toBe(0);
    expect(r.pitch).toBeCloseTo(r.alpha, 12);
  });

  it('balances the lift against the weight and the thrust against the drag', () => {
    const r = trimLevelFlight(cruise);
    // The wind axis lift carries the weight less the component the thrust holds,
    // which is the thrust times the sine of the angle of attack.
    const weight = r.mass * G0;
    expect(r.lift + r.thrust * Math.sin(r.alpha)).toBeCloseTo(weight, 0);
    expect(r.thrust * Math.cos(r.alpha)).toBeCloseTo(r.drag, 0);
  });

  it('leaves no pitching moment at the answer it reports', () => {
    const r = trimLevelFlight(cruise);
    const check = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: r.alpha,
      elevator: r.elevator,
      throttle: r.throttle,
    });
    expect(Math.abs(check.moment)).toBeLessThan(1e-8);
    expect(Math.abs(check.alongPath)).toBeLessThan(1e-8);
    expect(Math.abs(check.acrossPath)).toBeLessThan(1e-8);
  });

  it('gives the same answer every time it is asked', () => {
    // THE REGRESSION TEST FOR A REAL FAULT. assembly.evaluate solves the induced
    // angle from the separation state the LAST call left behind, so one call is
    // not a function of the state it receives. The solver runs the aerodynamics
    // to its own fixed point to remove that memory. Without the fixed point the
    // same condition gave different answers depending on what ran before it, and
    // the numerical Jacobian was noise.
    const first = trimLevelFlight(cruise);
    // Two very different conditions in between, to leave the model in another
    // state than the one the first call left.
    trimLevelFlight({ ...cruise, speed: 70, flapSetting: 'landing', gearDown: true });
    trimLevelFlight({ ...cruise, speed: 240 });
    const second = trimLevelFlight(cruise);
    expect(second.alpha).toBeCloseTo(first.alpha, 9);
    expect(second.elevator).toBeCloseTo(first.elevator, 9);
    expect(second.throttle).toBeCloseTo(first.throttle, 9);
  });

  it('needs a higher angle of attack at a lower speed', () => {
    const slow = trimLevelFlight({ ...cruise, speed: 120 });
    const fast = trimLevelFlight({ ...cruise, speed: 220 });
    expect(slow.converged).toBe(true);
    expect(fast.converged).toBe(true);
    expect(slow.alpha).toBeGreaterThan(fast.alpha);
    expect(slow.liftCoefficient).toBeGreaterThan(fast.liftCoefficient);
  });

  it('keeps the elevator inside its travel and reports a command, not an angle', () => {
    const r = trimLevelFlight(cruise);
    expect(Math.abs(r.elevator)).toBeLessThanOrEqual(1);
    // The command times ELEVATOR_LIMIT is the deflection the aerodynamics sees.
    expect(Math.abs(r.elevator * ELEVATOR_LIMIT)).toBeLessThanOrEqual(ELEVATOR_LIMIT);
  });

  it('reports no trim below the stall speed instead of a wrong one', () => {
    // 45 m/s is 162 km/h, well below any stall speed of this aircraft.
    const r = trimLevelFlight({ ...cruise, speed: 45 });
    expect(r.converged).toBe(false);
    expect(r.residual).toBeGreaterThan(1e-3);
  });

  it('reports no trim above the thrust limit instead of a wrong one', () => {
    // 300 m/s at 3000 m is far past what two Jumo 004 can push.
    const r = trimLevelFlight({ ...cruise, speed: 300 });
    expect(r.converged).toBe(false);
    expect(r.throttle).toBe(1);
    expect(r.atLimit).toBe(true);
  });

  it('needs more thrust at a load factor of two than at one', () => {
    const oneG = trimLevelFlight(cruise);
    const twoG = trimLevelFlight({ ...cruise, loadFactor: 2 });
    expect(twoG.converged).toBe(true);
    expect(twoG.alpha).toBeGreaterThan(oneG.alpha);
    expect(twoG.throttle).toBeGreaterThan(oneG.throttle);
    // The lift of a 2 g pull up is twice the weight, less what the thrust holds.
    expect(twoG.lift + twoG.thrust * Math.sin(twoG.alpha)).toBeCloseTo(2 * twoG.mass * G0, 0);
  });

  it('needs a lower angle of attack with the flaps down', () => {
    const clean = trimLevelFlight({ ...cruise, speed: 100 });
    const flapped = trimLevelFlight({ ...cruise, speed: 100, flapSetting: 'landing' });
    expect(clean.converged).toBe(true);
    expect(flapped.converged).toBe(true);
    expect(flapped.alpha).toBeLessThan(clean.alpha);
  });
});

describe('trimForAlpha', () => {
  it('holds the angle of attack it was given and solves the speed', () => {
    const r = trimForAlpha(cruise, 6 * DEG);
    expect(r.converged).toBe(true);
    expect(toDeg(r.alpha)).toBeCloseTo(6, 9);
    expect(r.flightPathAngle).toBe(0);
    expect(r.speed).toBeGreaterThan(50);
    expect(r.speed).toBeLessThan(200);
  });

  it('reports a lower speed at a higher angle of attack', () => {
    const low = trimForAlpha(cruise, 4 * DEG);
    const high = trimForAlpha(cruise, 10 * DEG);
    expect(low.converged).toBe(true);
    expect(high.converged).toBe(true);
    expect(high.speed).toBeLessThan(low.speed);
  });

  it('balances the lift against the weight at the speed it found', () => {
    const r = trimForAlpha(cruise, 8 * DEG);
    expect(r.lift + r.thrust * Math.sin(r.alpha)).toBeCloseTo(r.mass * G0, 0);
  });

  it('reports no trim past the stall angle of attack', () => {
    const r = trimForAlpha(cruise, 30 * DEG);
    expect(r.converged).toBe(false);
  });
});

describe('trimSteadyClimb', () => {
  it('climbs at a full throttle and descends at a closed one', () => {
    const full = trimSteadyClimb({ ...cruise, speed: 150 }, 1);
    const idle = trimSteadyClimb({ ...cruise, speed: 150 }, 0);
    expect(full.converged).toBe(true);
    expect(idle.converged).toBe(true);
    expect(full.climbRate).toBeGreaterThan(5);
    expect(idle.climbRate).toBeLessThan(0);
  });

  it('reports the climb rate as the speed times the sine of the path angle', () => {
    const r = trimSteadyClimb({ ...cruise, speed: 150 }, 1);
    expect(r.climbRate).toBeCloseTo(r.speed * Math.sin(r.flightPathAngle), 9);
    expect(r.pitch).toBeCloseTo(r.alpha + r.flightPathAngle, 12);
  });

  it('carries less lift in a climb, because the weight leans back', () => {
    const climb = trimSteadyClimb({ ...cruise, speed: 150 }, 1);
    const weight = climb.mass * G0;
    // A steady climb needs L = W cos(gamma), less the thrust component.
    const expected = weight * Math.cos(climb.flightPathAngle);
    expect(climb.lift + climb.thrust * Math.sin(climb.alpha)).toBeCloseTo(expected, 0);
    expect(climb.lift).toBeLessThan(weight);
  });

  it('climbs less well at a higher altitude', () => {
    const low = trimSteadyClimb({ ...cruise, altitude: 3000, speed: 150 }, 1);
    const high = trimSteadyClimb({ ...cruise, altitude: 9000, speed: 150 }, 1);
    expect(low.converged).toBe(true);
    expect(high.converged).toBe(true);
    expect(high.climbRate).toBeLessThan(low.climbRate);
  });
});

describe('trimMaxLevelSpeed', () => {
  it('puts the throttle at its stop and the thrust on the drag', () => {
    const r = trimMaxLevelSpeed(cruise);
    expect(r.converged).toBe(true);
    expect(r.throttle).toBe(1);
    expect(r.thrust * Math.cos(r.alpha)).toBeCloseTo(r.drag, 0);
    expect(r.thrust).toBeCloseTo(r.thrustAvailable, 6);
  });

  it('finds a speed no level trim can pass', () => {
    const max = trimMaxLevelSpeed(cruise);
    // One meter per second faster needs more thrust than the engines have.
    const past = trimLevelFlight({ ...cruise, speed: max.speed + 1 });
    expect(past.converged).toBe(false);
    // One meter per second slower is a trim with the throttle off its stop.
    const under = trimLevelFlight({ ...cruise, speed: max.speed - 1 });
    expect(under.converged).toBe(true);
    expect(under.throttle).toBeLessThan(1);
  });
});

describe('trimResiduals', () => {
  it('reports a nose up moment when the elevator moves nose up', () => {
    const down = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: 0.03,
      elevator: -0.5,
      throttle: 0.5,
    });
    const up = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: 0.03,
      elevator: 0.5,
      throttle: 0.5,
    });
    expect(up.momentCoefficient).toBeGreaterThan(down.momentCoefficient);
  });

  it('reports more lift at a higher angle of attack', () => {
    const low = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: 0.02,
      elevator: 0,
      throttle: 0,
    });
    const high = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: 0.08,
      elevator: 0,
      throttle: 0,
    });
    expect(high.lift).toBeGreaterThan(low.lift);
    expect(high.drag).toBeGreaterThan(low.drag);
  });

  it('is a static stability check: the moment falls as the angle of attack grows', () => {
    // dCm / dalpha must be negative for a statically stable aircraft.
    const low = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: 0.02,
      elevator: 0,
      throttle: 0,
    });
    const high = trimResiduals(cruise, {
      speed: cruise.speed,
      alpha: 0.06,
      elevator: 0,
      throttle: 0,
    });
    expect(high.momentCoefficient).toBeLessThan(low.momentCoefficient);
  });
});
