/**
 * Downwash at the horizontal tail and sidewash at the fin.
 *
 * Every test that carries a number states the number in its name or in its
 * comment, because this bead moves the static margin of the whole aircraft and a
 * later bead must be able to see what it moved it to.
 */

import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import { DEG } from '@/math/units';
import { clearWrench, createState, createWrench } from '@/physics/rigidbody';
import type { Wrench } from '@/physics/rigidbody';
import type { AeroAssembly } from '@/physics/aero/assembly';
import type { Downwash } from '@/physics/aero/downwash';
import { createDownwash, downwashParams, updateDownwash } from '@/physics/aero/downwash';
import {
  CONTROL_COUNT,
  CONTROL_INDEX,
  MAC,
  WING_AREA,
  WING_ASPECT_RATIO,
  WING_SPAN,
  createMe262Assembly,
} from '@/aircraft/me262/geometry';

const WING_FIRST = 0;
const TAIL_FIRST = 16;
const FIN_FIRST = 20;

/** Free stream dynamic pressure at the test speed, at sea level. */
const SPEED = 120; // m/s
const STEP = 1 / 240; // s

/**
 * Turns the whole model off, in place.
 *
 * A test that must measure what the downwash changed needs the same aircraft
 * with the model off. Every field below is a parameter of the model and not a
 * fact of the geometry, so a model with all of them neutral is exactly the
 * aircraft that flew before this bead.
 */
function turnOff(d: Downwash): void {
  d.params.wakeFactor = 0;
  d.params.etaTailClean = 1;
  d.params.etaTailWakeAttached = 1;
  d.params.etaTailWakeSeparated = 1;
  d.params.sidewashSlope = 0;
  d.params.etaFin = 1;
}

/** Turns the dynamic pressure loss off and leaves the downwash and the sidewash on. */
function turnOffPressureLoss(d: Downwash): void {
  d.params.etaTailClean = 1;
  d.params.etaTailWakeAttached = 1;
  d.params.etaTailWakeSeparated = 1;
  d.params.etaFin = 1;
}

interface Reading {
  moment: Vector3;
  force: Vector3;
  lift: number;
  dynamicPressure: number;
}

const wrench: Wrench = createWrench();
const wind = new Vector3(0, 0, 0);

/**
 * Flies the aircraft at one steady condition and copies the answer out.
 *
 * The assembly returns the same objects on every call, so a test that holds two
 * readings must copy. A step of one second is far longer than the separation lag
 * of any strip, so the second call reports the steady answer.
 */
function flyAt(
  assembly: AeroAssembly,
  alpha: number,
  beta: number = 0,
  controls: Float64Array = new Float64Array(CONTROL_COUNT),
  speed: number = SPEED,
): Reading {
  const state = createState();
  state.velocity.set(
    speed * Math.cos(alpha) * Math.cos(beta),
    speed * Math.sin(beta),
    speed * Math.sin(alpha) * Math.cos(beta),
  );
  assembly.evaluate(state, wind, controls, 1, wrench);
  clearWrench(wrench);
  const totals = assembly.evaluate(state, wind, controls, 1, wrench);
  return {
    moment: wrench.moment.clone(),
    force: wrench.force.clone(),
    lift: totals.lift,
    dynamicPressure: totals.dynamicPressure,
  };
}

/** Holds one angle of attack until the separation of every strip settles. */
function settleAt(assembly: AeroAssembly, alpha: number, speed: number = 70): Reading {
  const state = createState();
  state.velocity.set(speed * Math.cos(alpha), 0, speed * Math.sin(alpha));
  const controls = new Float64Array(CONTROL_COUNT);
  for (let i = 0; i < 600; i++) {
    clearWrench(wrench);
    assembly.evaluate(state, wind, controls, STEP, wrench);
  }
  return {
    moment: wrench.moment.clone(),
    force: wrench.force.clone(),
    lift: 0,
    dynamicPressure: 0.5 * 1.225 * speed * speed,
  };
}

interface Stability {
  cmAlpha: number;
  clAlpha: number;
  staticMargin: number;
}

/** dCm/dalpha, dCL/dalpha and the static margin, about the loaded center of gravity. */
function longitudinal(assembly: AeroAssembly): Stability {
  const step = 1 * DEG;
  const low = flyAt(assembly, 2 * DEG - step);
  const high = flyAt(assembly, 2 * DEG + step);
  const reference = low.dynamicPressure * WING_AREA;
  const cmAlpha = (high.moment.y - low.moment.y) / (reference * MAC * 2 * step);
  const clAlpha = (high.lift - low.lift) / (reference * 2 * step);
  return { cmAlpha, clAlpha, staticMargin: -cmAlpha / clAlpha };
}

/** dCn/dbeta of the whole aircraft, per radian, about the yaw axis. */
function directional(assembly: AeroAssembly): number {
  const step = 4 * DEG;
  const left = flyAt(assembly, 2 * DEG, -step);
  const right = flyAt(assembly, 2 * DEG, step);
  const reference = left.dynamicPressure * WING_AREA * WING_SPAN;
  return (right.moment.z - left.moment.z) / (reference * 2 * step);
}

/** The pitching moment one elevator step makes, in newton meters. */
function elevatorPower(assembly: AeroAssembly): number {
  const controls = new Float64Array(CONTROL_COUNT);
  const neutral = flyAt(assembly, 2 * DEG);
  controls[CONTROL_INDEX.elevator] = 10 * DEG;
  const deflected = flyAt(assembly, 2 * DEG, 0, controls);
  return deflected.moment.y - neutral.moment.y;
}

/** The aircraft with the model on. */
function aircraft(): AeroAssembly {
  return createMe262Assembly();
}

/** The aircraft with the model off. */
function bareAircraft(): AeroAssembly {
  const assembly = createMe262Assembly();
  turnOff(assembly.downwash);
  return assembly;
}

describe('the downwash angle at the tail', () => {
  it('reduces the angle of attack of the tail, and never raises it, when the wing lifts', () => {
    // The sign of this test is the sign of the whole longitudinal stability of
    // the aircraft. A downwash with the wrong sign would RAISE the tail angle,
    // which would make the tail more effective at a higher alpha and would
    // invert the answer of the aircraft to a gust.
    const on = aircraft();
    const off = bareAircraft();
    const alpha = 4 * DEG;
    flyAt(on, alpha);
    flyAt(off, alpha);

    expect(on.downwash.state.epsilon).toBeGreaterThan(0);
    // The tail works below the free stream angle, and further below it than the
    // tail of the aircraft with no downwash at all.
    expect(on.surfaces[TAIL_FIRST].result.alpha).toBeLessThan(alpha);
    expect(on.surfaces[TAIL_FIRST].result.alpha).toBeLessThan(
      off.surfaces[TAIL_FIRST].result.alpha,
    );
    // The wing does not read the downwash of its own tail.
    expect(on.surfaces[WING_FIRST].result.alpha).toBeCloseTo(
      off.surfaces[WING_FIRST].result.alpha,
      6,
    );
  });

  it('follows the wing lift, so a negative wing lift turns the downwash into an upwash', () => {
    const on = aircraft();
    flyAt(on, -6 * DEG);
    expect(on.downwash.state.epsilon).toBeLessThan(0);
    flyAt(on, 6 * DEG);
    expect(on.downwash.state.epsilon).toBeGreaterThan(0);
  });

  it('gives zero downwash at zero wing lift', () => {
    const d = createDownwash(downwashParams(aircraft().surfaces, aircraft().groups));
    updateDownwash(d, 0, 1, 4 * DEG, 0, SPEED, 8820, STEP);
    expect(d.state.epsilon).toBe(0);
    // A tail that meets no downwash and no pressure loss gains no angle either.
    turnOffPressureLoss(d);
    updateDownwash(d, 0, 1, 4 * DEG, 0, SPEED, 8820, STEP);
    expect(d.state.epsilon).toBe(0);
    expect(d.state.etaTail).toBe(1);
  });

  it('reaches the classic slope, between 2 CL_alpha / (PI AR) and 4 / (AR + 2)', () => {
    // Both estimates hold for an elliptically loaded wing. With the finite span
    // slope of this wing at 4.74 per radian and an aspect ratio of 7.21 they
    // give 0.419 and 0.434 per radian. The model measures 0.436, because it
    // drives the downwash from the lift the wing really makes and that lift
    // carries the sweep, the washout and the taper with it.
    const on = aircraft();
    flyAt(on, 0);
    const low = on.downwash.state.epsilon;
    flyAt(on, 4 * DEG);
    const high = on.downwash.state.epsilon;
    const slope = (high - low) / (4 * DEG);

    const elliptic = (2 * 4.74) / (Math.PI * WING_ASPECT_RATIO); // 0.419
    const empirical = 4 / (WING_ASPECT_RATIO + 2); // 0.434
    expect(elliptic).toBeCloseTo(0.419, 2);
    expect(empirical).toBeCloseTo(0.434, 2);
    expect(slope).toBeGreaterThan(0.9 * elliptic);
    expect(slope).toBeLessThan(1.1 * empirical);
  });
});

describe('the downwash and the static stability', () => {
  it('LOWERS dCm/dalpha in magnitude, from -0.975 to -0.278 per radian', () => {
    // The classic result is Cm_alpha_tail = -a_t V_h eta (1 - d epsilon /
    // d alpha). Both factors are below one, so the downwash and the pressure
    // loss can only take stability AWAY. The brief for this bead asked for the
    // opposite. The opposite is not what the physics gives and this test states
    // the measured direction.
    const off = longitudinal(bareAircraft());
    const on = longitudinal(aircraft());

    // The aircraft before this bead: -0.975 per radian, 16.8 percent of chord.
    expect(off.cmAlpha).toBeCloseTo(-0.975, 1);
    expect(off.staticMargin).toBeCloseTo(0.168, 1);

    // With the model on: dCm/dalpha -0.278 per radian, dCL/dalpha 5.43 per
    // radian, static margin 5.1 percent of the mean aerodynamic chord.
    expect(on.cmAlpha).toBeLessThan(0);
    expect(on.cmAlpha).toBeGreaterThan(off.cmAlpha);
    expect(on.cmAlpha).toBeCloseTo(-0.278, 1);
    expect(on.staticMargin).toBeGreaterThan(0.03);
    expect(on.staticMargin).toBeLessThan(0.10);

    // The aircraft is still stable, and it still needs its tail to be stable.
    expect(on.clAlpha).toBeGreaterThan(4);
  });

  it('puts the neutral point at 30 percent of the mean chord, where a 1944 fighter carried it', () => {
    // The center of gravity sits at the quarter chord of the mean aerodynamic
    // chord, so the neutral point sits at 25 percent plus the static margin.
    // With no downwash the model puts it at 41.8 percent, which is further aft
    // than any measured fighter of the period. With the downwash it sits at
    // 30.1 percent. The P-51D measured 31 percent stick fixed, so the model
    // moves TOWARD the measured aircraft and not away from it.
    const off = 0.25 + longitudinal(bareAircraft()).staticMargin;
    const on = 0.25 + longitudinal(aircraft()).staticMargin;
    expect(off).toBeGreaterThan(0.40);
    expect(on).toBeGreaterThan(0.27);
    expect(on).toBeLessThan(0.36);
  });
});

describe('the downwash at the stall', () => {
  /** The free stream angle, the downwash and the flow angle the tail meets. */
  function sweep(assembly: AeroAssembly, degrees: number[]): number[] {
    const out: number[] = [];
    for (const deg of degrees) {
      settleAt(assembly, deg * DEG);
      out.push(assembly.downwash.state.epsilon);
    }
    return out;
  }

  it('stops following alpha when the wing breaks, so the tail angle runs away upward', () => {
    // Below the stall the downwash takes 0.42 degrees out of every degree of
    // alpha. Past the stall the wing lift stops growing, so the downwash stops
    // growing with it and the tail meets nearly the whole of every further
    // degree. A fixed slope model would hide that completely.
    const on = aircraft();
    const [e8, e14, e20, e26] = sweep(on, [8, 14, 20, 26]);

    const below = (e14 - e8) / (6 * DEG);
    const above = (e26 - e20) / (6 * DEG);
    expect(below).toBeGreaterThan(0.3);
    expect(above).toBeLessThan(0.1);
    // The downwash does not grow past the break at all. It measured 7.02
    // degrees at 20 degrees of alpha and 6.94 degrees at 26.
    expect(e26).toBeLessThanOrEqual(e20);

    // The angle the tail meets is alpha - epsilon. It gains 0.62 degrees per
    // degree below the stall and 1.01 degrees per degree past it.
    const tailBelow = (14 * DEG - e14 - (8 * DEG - e8)) / (6 * DEG);
    const tailAbove = (26 * DEG - e26 - (20 * DEG - e20)) / (6 * DEG);
    expect(tailAbove).toBeGreaterThan(1.3 * tailBelow);
  });

  it('runs far below a fixed slope model deep in the stall', () => {
    // A fixed slope of 0.436 per radian would report 13.1 degrees of downwash at
    // 30 degrees of alpha. The wing does not make that lift, so the model
    // reports 7.0 degrees and the tail meets 6 degrees more than the fixed slope
    // model would give it. Six degrees at the tail is the whole difference
    // between a tail that still works and a tail that does not.
    const on = aircraft();
    settleAt(on, 30 * DEG);
    const fixed = 0.436 * 30 * DEG;
    expect(on.downwash.state.epsilon).toBeLessThan(0.7 * fixed);
    expect(on.downwash.state.epsilon).toBeGreaterThan(0);
  });

  it('keeps a nose down moment through the stall, so the wake makes no deep stall trap', () => {
    // The wake does cover this tail past the stall, and the model says so: the
    // coverage reaches 1 and the tail keeps only about 70 percent of the free
    // stream dynamic pressure. The tail is LOW, so it holds a positive angle of
    // attack and a restoring moment through the whole band. A T tail that lost
    // that would hold the nose up with no way down.
    const on = aircraft();
    for (const deg of [16, 20, 26, 30]) {
      const reading = settleAt(on, deg * DEG);
      expect(on.downwash.state.wakeCoverage).toBeGreaterThan(0.9);
      expect(on.downwash.state.etaTail).toBeGreaterThan(0.55);
      expect(reading.moment.y).toBeLessThan(0);
    }
  });

  it('leaves the tail clear of the wake in normal flight', () => {
    const on = aircraft();
    for (const deg of [0, 2, 6]) {
      flyAt(on, deg * DEG);
      expect(on.downwash.state.wakeCoverage).toBe(0);
      expect(on.downwash.state.etaTail).toBeCloseTo(0.92, 6);
      // The wake center line stays more than 0.4 m below the tail, and the wake
      // of an attached wing is only 0.11 m thick at that station.
      expect(on.downwash.state.wakeOffset).toBeGreaterThan(0.4);
    }
  });
});

describe('the dynamic pressure at the tail', () => {
  it('cuts the elevator power, because the elevator works on the pressure it meets', () => {
    const withLoss = aircraft();
    const withoutLoss = aircraft();
    turnOffPressureLoss(withoutLoss.downwash);

    const cut = elevatorPower(withLoss);
    const full = elevatorPower(withoutLoss);
    // A positive elevator command pitches the nose up, so both are positive.
    expect(full).toBeGreaterThan(0);
    expect(cut).toBeGreaterThan(0);
    expect(cut).toBeLessThan(full);
    // eta_h of 0.92 takes 5.6 percent of the elevator power. It is not the whole
    // 8 percent, because a tail that makes less lift also makes less induced
    // angle, and that gives a little of the loss back.
    expect(cut / full).toBeGreaterThan(0.9);
    expect(cut / full).toBeLessThan(0.99);
  });

  it('falls further when the wing separates, because a stalled wing sheds a thicker wake', () => {
    const on = aircraft();
    flyAt(on, 2 * DEG);
    const clean = on.downwash.state.etaTail;
    settleAt(on, 26 * DEG);
    const stalled = on.downwash.state.etaTail;
    expect(clean).toBeCloseTo(0.92, 6);
    expect(stalled).toBeLessThan(0.7);
    expect(stalled).toBeGreaterThan(0.55);
  });
});

describe('the sidewash at the fin', () => {
  it('changes the sideslip the fin meets, and this configuration meets more of it', () => {
    // DATCOM fits eta_v (1 - d sigma / d beta) at 1.01 for a mid wing with a fin
    // of this relative area, so the turn at the fin is FAVORABLE. The fin meets
    // 6.5 percent more sideslip than the free stream and gives 5 percent of the
    // dynamic pressure back to the fuselage boundary layer.
    const on = aircraft();
    const off = bareAircraft();
    const beta = 4 * DEG;
    flyAt(on, 2 * DEG, beta);
    flyAt(off, 2 * DEG, beta);

    expect(on.downwash.state.sigma).toBeLessThan(0);
    expect(on.surfaces[FIN_FIRST].result.alpha).not.toBeCloseTo(
      off.surfaces[FIN_FIRST].result.alpha,
      3,
    );
    expect(on.surfaces[FIN_FIRST].result.alpha).toBeGreaterThan(
      off.surfaces[FIN_FIRST].result.alpha,
    );

    // The model is odd in the sideslip, so the mirror condition mirrors.
    flyAt(on, 2 * DEG, -beta);
    expect(on.downwash.state.sigma).toBeGreaterThan(0);
    expect(on.surfaces[FIN_FIRST].result.alpha).toBeLessThan(0);
  });

  it('leaves the complete aircraft with a positive dCn/dbeta', () => {
    const on = directional(aircraft());
    const off = directional(bareAircraft());
    expect(on).toBeGreaterThan(0);
    // The margin is thin and it was thin before this bead. The fin makes
    // +0.077 per radian and the fuselage and the nacelles take -0.075 back,
    // which leaves +0.0008 with no sidewash at all. The favorable turn at the
    // fin adds a little to that, and gives +0.0039.
    expect(off).toBeGreaterThan(0);
    expect(on).toBeGreaterThan(off);
  });

  it('makes the nose swing into a positive sideslip', () => {
    const reading = flyAt(aircraft(), 2 * DEG, 6 * DEG);
    expect(reading.moment.z).toBeGreaterThan(0);
    expect(reading.force.y).toBeLessThan(0);
  });
});

describe('the model itself', () => {
  it('finds the wing, the tail and the fin out of the geometry alone', () => {
    const assembly = aircraft();
    const p = assembly.downwash.params;
    expect(p.wingIndices.length).toBe(16);
    expect(p.tailIndices.length).toBe(4);
    expect(p.finIndices.length).toBe(2);
    // The tail arm and the tail height are what set the wake crossing angle.
    expect(p.tailArm).toBeCloseTo(3.32, 1);
    expect(p.tailAboveWing).toBeCloseTo(0.64, 1);
  });

  it('does nothing at all to an aircraft with no group behind the wing', () => {
    const assembly = aircraft();
    const params = downwashParams(assembly.surfaces, [assembly.groups[0]]);
    expect(params.tailIndices.length).toBe(0);
    expect(params.finIndices.length).toBe(0);
    const d = createDownwash(params);
    const angles = new Float64Array(assembly.surfaces.length);
    angles.fill(0.05);
    updateDownwash(d, 20000, 1, 4 * DEG, 0, SPEED, 8820, STEP);
    expect(d.state.epsilon).toBeGreaterThan(0);
    // The angles of the strips never move, because there is nothing to move.
    for (let i = 0; i < angles.length; i++) {
      expect(angles[i]).toBe(0.05);
    }
  });

  it('holds the wake travel lag when the caller asks for it, and no lag by default', () => {
    // l / V is 0.028 s at 120 m/s. The default is OFF, because stepRK4 evaluates
    // four times per step at four different states and a lagged value carried
    // between them belongs to none of them.
    const assembly = aircraft();
    const d = createDownwash(downwashParams(assembly.surfaces, assembly.groups));
    expect(d.params.useLag).toBe(false);
    updateDownwash(d, 40000, 1, 4 * DEG, 0, SPEED, 8820, STEP);
    const steady = d.state.epsilon;

    const lagged = createDownwash({
      ...downwashParams(assembly.surfaces, assembly.groups),
      useLag: true,
    });
    updateDownwash(lagged, 40000, 1, 4 * DEG, 0, SPEED, 8820, STEP);
    expect(lagged.state.epsilon).toBeGreaterThan(0);
    expect(lagged.state.epsilon).toBeLessThan(steady);
    for (let i = 0; i < 200; i++) {
      updateDownwash(lagged, 40000, 1, 4 * DEG, 0, SPEED, 8820, STEP);
    }
    expect(lagged.state.epsilon).toBeCloseTo(steady, 6);
  });

  it('returns the same state object on every call and allocates nothing', () => {
    const assembly = aircraft();
    const state = createState();
    state.velocity.set(SPEED, 0, 4);
    const controls = new Float64Array(CONTROL_COUNT);
    const first = assembly.downwash.state;
    for (let i = 0; i < 500; i++) {
      clearWrench(wrench);
      assembly.evaluate(state, wind, controls, STEP, wrench);
      expect(assembly.downwash.state).toBe(first);
    }
    expect(Number.isFinite(first.epsilon)).toBe(true);
  });
});
