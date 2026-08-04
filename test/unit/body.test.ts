import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import { createBody, evaluateBody } from '@/physics/aero/body';
import type { Body, BodyDef } from '@/physics/aero/body';
import { clearWrench, createWrench } from '@/physics/rigidbody';
import type { Wrench } from '@/physics/rigidbody';
import { DEG } from '@/math/units';
import { SEA_LEVEL_DENSITY } from '@/physics/atmosphere';

const RHO = SEA_LEVEL_DENSITY;
const SPEED_OF_SOUND = 340.294; // m/s, ISA sea level
const ZERO = new Vector3(0, 0, 0);

/**
 * The Me-262 fuselage, at the center of gravity.
 *
 * Length 9.5 m of fuselage inside an aircraft of 10.60 m. Maximum width 1.5 m,
 * from the triangular section of the Me-262. The volume, the side area and the
 * frontal area follow from those two numbers with the usual shape factors of a
 * fighter fuselage. Every one is an estimate. Bead b17 owns the real set.
 */
function fuselage(over: Partial<BodyDef> = {}): BodyDef {
  return {
    name: 'fuselage',
    position: new Vector3(0, 0, 0),
    length: 9.5,
    maxDiameter: 1.5,
    volume: 8.4,
    sideArea: 10,
    frontalArea: 1.5,
    axialDragCoefficient: 0.08,
    crossFlowDragCoefficient: 1.2,
    munkFactor: 0.47,
    ...over,
  };
}

function run(
  b: Body,
  velocity: Vector3,
  options: { omega?: Vector3; wind?: Vector3 } = {},
): Wrench {
  const out = createWrench();
  clearWrench(out);
  evaluateBody(
    b,
    velocity,
    options.omega ?? ZERO,
    options.wind ?? ZERO,
    RHO,
    SPEED_OF_SOUND,
    out,
  );
  return out;
}

function flightVelocity(speed: number, alpha: number, beta: number = 0): Vector3 {
  return new Vector3(
    speed * Math.cos(alpha) * Math.cos(beta),
    speed * Math.sin(beta),
    speed * Math.sin(alpha) * Math.cos(beta),
  );
}

describe('the Munk moment is destabilizing', () => {
  it('makes a nose UP moment on a bare fuselage at a positive angle of attack', () => {
    const b = createBody(fuselage());
    const w = run(b, flightVelocity(100, 6 * DEG));
    expect(w.moment.y).toBeGreaterThan(0);
  });

  it('makes a nose DOWN moment at a negative angle of attack', () => {
    const b = createBody(fuselage());
    const w = run(b, flightVelocity(100, -6 * DEG));
    expect(w.moment.y).toBeLessThan(0);
  });

  it('grows with the angle of attack, so d(moment)/d(alpha) is positive', () => {
    const b = createBody(fuselage());
    const low = run(b, flightVelocity(100, 2 * DEG)).moment.y;
    const high = run(b, flightVelocity(100, 8 * DEG)).moment.y;
    expect(high).toBeGreaterThan(low);
  });

  it('matches k rho V^2 volume sin(2 alpha)', () => {
    const b = createBody(fuselage());
    const speed = 120;
    const alpha = 7 * DEG;
    const w = run(b, flightVelocity(speed, alpha));
    const expected = 0.47 * RHO * speed * speed * 8.4 * Math.sin(2 * alpha);
    expect(w.moment.y).toBeCloseTo(expected, 6);
  });

  it('peaks at 45 degrees and vanishes at 90 degrees, as sin(2 alpha) does', () => {
    const b = createBody(fuselage());
    const at45 = run(b, flightVelocity(100, 45 * DEG)).moment.y;
    const at30 = run(b, flightVelocity(100, 30 * DEG)).moment.y;
    const at90 = run(b, flightVelocity(100, 90 * DEG)).moment.y;
    expect(at45).toBeGreaterThan(at30);
    expect(Math.abs(at90)).toBeLessThan(1e-6 * at45);
  });

  it('turns the nose further into a positive sideslip, which is a negative yaw moment', () => {
    // A positive sideslip puts the nose left of the flight path. The Munk moment
    // wants the body broadside, so it turns the nose further left.
    const b = createBody(fuselage());
    const w = run(b, flightVelocity(100, 0, 6 * DEG));
    expect(w.moment.z).toBeLessThan(0);
    const expected = -0.47 * RHO * 100 * 100 * 8.4 * Math.sin(2 * (6 * DEG));
    expect(w.moment.z).toBeCloseTo(expected, 6);
  });

  it('vanishes with the volume, so a body of no volume has no Munk moment', () => {
    const b = createBody(fuselage({ volume: 0 }));
    const w = run(b, flightVelocity(100, 10 * DEG));
    expect(w.moment.y).toBeCloseTo(0, 9);
  });
});

describe('axial drag', () => {
  it('opposes the motion in pure forward flight', () => {
    const b = createBody(fuselage());
    const w = run(b, flightVelocity(100, 0));
    expect(w.force.x).toBeCloseTo(-0.5 * RHO * 100 * 100 * 1.5 * 0.08, 6);
    expect(w.force.y).toBeCloseTo(0, 9);
    expect(w.force.z).toBeCloseTo(0, 9);
    expect(w.moment.length()).toBeCloseTo(0, 9);
  });

  it('still opposes the motion when the body flies backwards', () => {
    const b = createBody(fuselage());
    const w = run(b, new Vector3(-80, 0, 0));
    expect(w.force.x).toBeGreaterThan(0);
  });

  it('grows above the critical Mach number', () => {
    const b = createBody(fuselage());
    const slow = -run(b, flightVelocity(0.6 * SPEED_OF_SOUND, 0)).force.x;
    const fast = -run(b, flightVelocity(0.86 * SPEED_OF_SOUND, 0)).force.x;
    const square = (0.86 / 0.6) ** 2;
    // The wave drag adds on top of the growth with the square of the speed.
    expect(fast).toBeGreaterThan(1.2 * square * slow);
  });
});

describe('cross flow drag', () => {
  it('grows with the square of the sine of the incidence', () => {
    const b = createBody(fuselage({ axialDragCoefficient: 0, volume: 0, frontalArea: 0 }));
    const at10 = -run(b, flightVelocity(100, 10 * DEG)).force.z;
    const at20 = -run(b, flightVelocity(100, 20 * DEG)).force.z;
    // The cross flow force is normal to the AXIS of the body, not normal to the
    // flow, so no cosine resolves it. The cross velocity is V sin(i) and the
    // force follows its square, which makes the law exactly sin^2.
    const ratio = Math.sin(20 * DEG) ** 2 / Math.sin(10 * DEG) ** 2;
    expect(at20 / at10).toBeCloseTo(ratio, 9);
    const q = 0.5 * RHO * 100 * 100;
    expect(at20).toBeCloseTo(
      q * 10 * 1.2 * b.crossFlowFactor * Math.sin(20 * DEG) ** 2,
      6,
    );
  });

  it('reaches the full cylinder drag at 90 degrees of incidence', () => {
    const b = createBody(fuselage({ axialDragCoefficient: 0, volume: 0, frontalArea: 0 }));
    const w = run(b, new Vector3(0, 0, 100));
    // The fineness ratio of 6.33 cuts the coefficient of an infinite cylinder.
    const expected = 0.5 * RHO * 100 * 100 * 10 * 1.2 * b.crossFlowFactor;
    expect(-w.force.z).toBeCloseTo(expected, 6);
    expect(b.crossFlowFactor).toBeGreaterThan(0.7);
    expect(b.crossFlowFactor).toBeLessThan(0.85);
  });

  it('opposes the cross velocity on both axes at the same time', () => {
    const b = createBody(fuselage());
    const w = run(b, flightVelocity(100, 10 * DEG, 10 * DEG));
    expect(w.force.y).toBeLessThan(0);
    expect(w.force.z).toBeLessThan(0);
  });

  it('cuts the coefficient more for a short body than for a long one', () => {
    const short = createBody(fuselage({ length: 3, maxDiameter: 1.5 }));
    const long = createBody(fuselage({ length: 30, maxDiameter: 1.5 }));
    expect(short.crossFlowFactor).toBeLessThan(long.crossFlowFactor);
  });
});

describe('slender body lift', () => {
  it('lifts at a positive angle of attack, which is a negative z force', () => {
    const b = createBody(fuselage({ crossFlowDragCoefficient: 0, axialDragCoefficient: 0 }));
    const w = run(b, flightVelocity(100, 5 * DEG));
    expect(w.force.z).toBeLessThan(0);
    const q = 0.5 * RHO * 100 * 100;
    const expected = 2 * q * b.baseArea * Math.sin(5 * DEG) * Math.cos(5 * DEG);
    // The Prandtl-Glauert factor at Mach 0.29 grows it a little.
    expect(-w.force.z / expected).toBeGreaterThan(1);
    expect(-w.force.z / expected).toBeLessThan(1.06);
  });

  it('pushes left at a positive sideslip, the same sense a fin takes', () => {
    const b = createBody(fuselage({ crossFlowDragCoefficient: 0, axialDragCoefficient: 0 }));
    const w = run(b, flightVelocity(100, 0, 5 * DEG));
    expect(w.force.y).toBeLessThan(0);
  });

  it('is small next to the cross flow drag at a large angle of attack', () => {
    const full = createBody(fuselage());
    const noCross = createBody(fuselage({ crossFlowDragCoefficient: 0 }));
    const a = -run(full, flightVelocity(100, 20 * DEG)).force.z;
    const b = -run(noCross, flightVelocity(100, 20 * DEG)).force.z;
    expect(b).toBeLessThan(0.25 * a);
  });
});

describe('the local flow of a body', () => {
  it('makes no force when the aircraft moves with the air mass', () => {
    const b = createBody(fuselage());
    const velocity = flightVelocity(100, 8 * DEG);
    const w = run(b, velocity, { wind: velocity.clone() });
    expect(w.force.length()).toBeCloseTo(0, 9);
    expect(w.moment.length()).toBeCloseTo(0, 9);
  });

  it('adds omega cross r, so a body behind the center of gravity meets a sideslip of its own', () => {
    // omega x r for a body at (-4, 0, 0) under a yaw rate r is (0, -4 r, 0). A
    // positive yaw rate swings the tail left, so the tail meets air from the
    // left and the drag on it pushes back to the right.
    const tailCone = createBody(fuselage({ name: 'tail cone', position: new Vector3(-4, 0, 0) }));
    const still = run(tailCone, flightVelocity(100, 0));
    expect(still.force.y).toBeCloseTo(0, 9);
    expect(still.moment.z).toBeCloseTo(0, 9);
    const yawing = run(tailCone, flightVelocity(100, 0), { omega: new Vector3(0, 0, 0.3) });
    // The lateral speed is the rate times the arm, so the local sideslip is
    // negative and its size is 0.3 * 4 / 100.
    expect(tailCone.result.beta).toBeCloseTo(Math.asin(-1.2 / Math.hypot(100, 1.2)), 12);
    expect(yawing.force.y).toBeGreaterThan(0);
  });

  it('damps a yaw rate through the arm alone, once the Munk moment is taken away', () => {
    // With no volume there is no Munk moment, so only the force on the arm is
    // left. That force opposes the rate, which is the damping the nacelles and
    // the tail cone really contribute.
    const tailCone = createBody(
      fuselage({ name: 'tail cone', position: new Vector3(-4, 0, 0), volume: 0 }),
    );
    const yawing = run(tailCone, flightVelocity(100, 0), { omega: new Vector3(0, 0, 0.3) });
    expect(yawing.moment.z).toBeLessThan(0);
    const yawingBack = run(tailCone, flightVelocity(100, 0), { omega: new Vector3(0, 0, -0.3) });
    expect(yawingBack.moment.z).toBeGreaterThan(0);
  });

  it('stays destabilizing in yaw under a yaw rate, which is why the aircraft needs a fin', () => {
    // The Munk moment of the whole fuselage beats the damping of its own arm.
    // A bare fuselage therefore diverges in yaw at a yaw rate as well as at a
    // sideslip. The fin has to beat both.
    const body = createBody(fuselage({ position: new Vector3(-4, 0, 0) }));
    const yawing = run(body, flightVelocity(100, 0), { omega: new Vector3(0, 0, 0.3) });
    expect(yawing.moment.z).toBeGreaterThan(0);
  });

  it('adds omega cross r, so a yaw rate changes the axial speed of a side nacelle', () => {
    // omega x r for a nacelle at (0, 2.5, 0) under a yaw rate is (-2.5 r, 0, 0),
    // which is purely axial. A positive yaw rate sweeps the right nacelle
    // backward, so it slows down and its axial drag falls. It makes no side
    // force at all, and a test that asked for one would be asking for the wrong
    // physics.
    const nacelle = createBody(fuselage({ name: 'nacelle', position: new Vector3(0, 2.5, 0) }));
    const still = run(nacelle, flightVelocity(100, 0));
    const yawing = run(nacelle, flightVelocity(100, 0), { omega: new Vector3(0, 0, 0.3) });
    expect(yawing.force.y).toBeCloseTo(0, 12);
    expect(-yawing.force.x).toBeLessThan(-still.force.x);
    expect(nacelle.result.alpha).toBeCloseTo(0, 12);
  });

  it('carries the arm of its force about the center of gravity', () => {
    const b = createBody(fuselage({ position: new Vector3(0.5, 0, 0), volume: 0 }));
    const w = run(b, flightVelocity(100, 10 * DEG));
    const arm = new Vector3(0.5, 0, 0).cross(b.result.force.clone());
    expect(w.moment.x).toBeCloseTo(arm.x, 9);
    expect(w.moment.y).toBeCloseTo(arm.y, 9);
    expect(w.moment.z).toBeCloseTo(arm.z, 9);
    // A nose down load ahead of the center of gravity is a nose up moment.
    expect(w.moment.y).toBeGreaterThan(0);
  });
});

describe('symmetry and safety', () => {
  it('makes no side force and no roll or yaw moment in pure forward flight', () => {
    const b = createBody(fuselage({ position: new Vector3(0.4, 0, 0.1) }));
    const w = run(b, flightVelocity(150, 4 * DEG));
    expect(w.force.y).toBeCloseTo(0, 9);
    expect(w.moment.x).toBeCloseTo(0, 9);
    expect(w.moment.z).toBeCloseTo(0, 9);
  });

  it('mirrors the pitch answer onto the yaw answer for a body of revolution', () => {
    const b = createBody(fuselage({ sideArea: 10, frontalArea: 1.5 }));
    const pitch = run(b, flightVelocity(100, 12 * DEG));
    const yaw = run(b, flightVelocity(100, 0, 12 * DEG));
    expect(yaw.force.y).toBeCloseTo(pitch.force.z, 6);
    expect(yaw.moment.z).toBeCloseTo(-pitch.moment.y, 6);
  });

  it('makes no force and no moment at rest', () => {
    const b = createBody(fuselage());
    const w = run(b, ZERO);
    expect(w.force.length()).toBe(0);
    expect(w.moment.length()).toBe(0);
    expect(Number.isFinite(b.result.beta)).toBe(true);
  });

  it('rejects a body with no length or no diameter', () => {
    expect(() => createBody(fuselage({ length: 0 }))).toThrow();
    expect(() => createBody(fuselage({ maxDiameter: 0 }))).toThrow();
  });
});

describe('evaluateBody allocates nothing', () => {
  it('returns the same result objects and the same vectors on every call', () => {
    const b = createBody(fuselage());
    const out = createWrench();
    const result = b.result;
    const force = b.result.force;
    const moment = b.result.moment;
    for (let i = 0; i < 1000; i++) {
      clearWrench(out);
      evaluateBody(
        b,
        flightVelocity(100, (i % 30) * DEG),
        ZERO,
        ZERO,
        RHO,
        SPEED_OF_SOUND,
        out,
      );
    }
    expect(b.result).toBe(result);
    expect(b.result.force).toBe(force);
    expect(b.result.moment).toBe(moment);
    expect(Number.isFinite(out.force.z)).toBe(true);
  });
});
