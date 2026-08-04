import { describe, expect, it } from 'vitest';
import { Matrix3, Quaternion, Vector3 } from 'three';
import { G0, toRad } from '@/math/units';
import {
  airspeedBody,
  bodyToWorld,
  clearWrench,
  createMassProperties,
  createState,
  createWrench,
  flowAngles,
  stepRK4,
  worldToBody,
} from '@/physics/rigidbody';
import type { RigidBodyState, Wrench, WrenchSource } from '@/physics/rigidbody';

const DT = 1 / 240; // s, the fixed physics step

/** Makes a diagonal inertia tensor in body axes. */
function diagonalInertia(ixx: number, iyy: number, izz: number): Matrix3 {
  return new Matrix3().set(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);
}

/** Checks a value against a known answer with a relative tolerance. */
function expectRelative(actual: number, expected: number, tolerance: number): void {
  const error = Math.abs(actual - expected) / Math.max(Math.abs(expected), 1e-30);
  expect(
    error,
    `expected ${actual} to be within ${tolerance} of ${expected}, relative error ${error}`,
  ).toBeLessThan(tolerance);
}

/** Runs the body for a number of steps from time zero. */
function run(
  state: RigidBodyState,
  mass: ReturnType<typeof createMassProperties>,
  source: WrenchSource,
  steps: number,
): void {
  for (let i = 0; i < steps; i++) {
    stepRK4(state, mass, source, i * DT, DT);
  }
}

const GRAVITY_WORLD = new Vector3(0, 0, G0); // NED, down is positive
const scratch = new Vector3();

/** Makes a source that adds weight only, as the caller must. */
function gravityOnly(massKg: number): WrenchSource {
  return (state: RigidBodyState, _time: number, out: Wrench): void => {
    clearWrench(out);
    worldToBody(state.orientation, GRAVITY_WORLD, scratch).multiplyScalar(massKg);
    out.force.copy(scratch);
  };
}

describe('rigid body translation', () => {
  it('a body with weight only falls half g t squared in 10 s', () => {
    const massKg = 6396; // kg, the loaded mass of the Me-262
    const mass = createMassProperties(massKg, diagonalInertia(12000, 25000, 35000));
    const state = createState();
    const seconds = 10;
    run(state, mass, gravityOnly(massKg), Math.round(seconds / DT));

    // Down is positive z in the world NED frame.
    expectRelative(state.position.z, 0.5 * G0 * seconds * seconds, 1e-6);
    expectRelative(state.velocity.z, G0 * seconds, 1e-6);
    expect(Math.abs(state.position.x)).toBeLessThan(1e-9);
    expect(Math.abs(state.position.y)).toBeLessThan(1e-9);
  });

  it('a constant force along body x with a level attitude gives v equal to F t over m', () => {
    const massKg = 6396;
    const force = 17600; // N, both Jumo 004 engines at full static thrust
    const mass = createMassProperties(massKg, diagonalInertia(12000, 25000, 35000));
    const state = createState();
    const seconds = 10;
    const source: WrenchSource = (_state, _time, out) => {
      clearWrench(out);
      out.force.set(force, 0, 0);
    };
    run(state, mass, source, Math.round(seconds / DT));

    // A level attitude makes body x the same as world north.
    expectRelative(state.velocity.x, (force * seconds) / massKg, 1e-9);
    expectRelative(state.position.x, (0.5 * force * seconds * seconds) / massKg, 1e-9);
    expect(Math.abs(state.velocity.z)).toBeLessThan(1e-12);
  });
});

describe('rigid body rotation', () => {
  it('a constant moment about a principal axis gives omega equal to M t over I', () => {
    const ixx = 12000; // kg m^2
    const moment = 3000; // N m
    const mass = createMassProperties(6396, diagonalInertia(ixx, 25000, 35000));
    const state = createState();
    const seconds = 5;
    const source: WrenchSource = (_state, _time, out) => {
      clearWrench(out);
      out.moment.set(moment, 0, 0);
    };
    run(state, mass, source, Math.round(seconds / DT));

    expectRelative(state.angularVelocity.x, (moment * seconds) / ixx, 1e-9);
    expect(Math.abs(state.angularVelocity.y)).toBeLessThan(1e-12);
    expect(Math.abs(state.angularVelocity.z)).toBeLessThan(1e-12);

    // The rotation stays about the world x axis, so the roll angle comes from
    // the x part and the w part of the quaternion.
    const angle = 2 * Math.atan2(state.orientation.x, state.orientation.w);
    expectRelative(angle, (0.5 * moment * seconds * seconds) / ixx, 1e-9);
  });

  it('torque free motion holds the angular momentum in the world frame and the kinetic energy', () => {
    // An asymmetric tensor makes the gyroscopic term work hard.
    const inertia = diagonalInertia(12000, 25000, 35000);
    const mass = createMassProperties(6396, inertia);
    const state = createState();
    state.angularVelocity.set(0.9, -0.4, 0.7);

    const momentum = new Vector3();
    const worldMomentum = (s: RigidBodyState, out: Vector3): Vector3 => {
      out.copy(s.angularVelocity).applyMatrix3(inertia);
      return bodyToWorld(s.orientation, out, out);
    };
    const energy = (s: RigidBodyState): number => {
      momentum.copy(s.angularVelocity).applyMatrix3(inertia);
      return 0.5 * momentum.dot(s.angularVelocity);
    };

    const startMomentum = worldMomentum(state, new Vector3());
    const startEnergy = energy(state);

    const source: WrenchSource = (_state, _time, out) => {
      clearWrench(out);
    };
    run(state, mass, source, Math.round(20 / DT));

    const endMomentum = worldMomentum(state, new Vector3());
    expectRelative(endMomentum.x, startMomentum.x, 1e-8);
    expectRelative(endMomentum.y, startMomentum.y, 1e-8);
    expectRelative(endMomentum.z, startMomentum.z, 1e-8);
    expectRelative(endMomentum.length(), startMomentum.length(), 1e-8);
    expectRelative(energy(state), startEnergy, 1e-8);
  });

  it('a spin about the intermediate axis flips, so the intermediate part changes sign', () => {
    // Ixx < Iyy < Izz, so the y axis is the intermediate axis.
    const mass = createMassProperties(6396, diagonalInertia(12000, 25000, 35000));
    const state = createState();
    const spin = 6; // rad/s about the intermediate axis
    state.angularVelocity.set(1e-4, spin, 0);
    const source: WrenchSource = (_state, _time, out) => {
      clearWrench(out);
    };

    let flipped = false;
    const steps = Math.round(30 / DT);
    for (let i = 0; i < steps && !flipped; i++) {
      stepRK4(state, mass, source, i * DT, DT);
      if (state.angularVelocity.y < -0.5 * spin) {
        flipped = true;
      }
    }
    expect(flipped, 'the spin about the intermediate axis did not flip').toBe(true);
    expect(state.angularVelocity.y).toBeLessThan(0);
  });

  it('the orientation quaternion stays a unit quaternion over 100000 steps', () => {
    const mass = createMassProperties(6396, diagonalInertia(12000, 25000, 35000));
    const state = createState();
    state.angularVelocity.set(0.3, -0.7, 1.1);
    const source: WrenchSource = (_state, _time, out) => {
      clearWrench(out);
      out.moment.set(0, 200, 0);
    };
    run(state, mass, source, 100000);

    expect(Math.abs(state.orientation.length() - 1)).toBeLessThan(1e-12);
  });
});

describe('frame helpers', () => {
  it('bodyToWorld and worldToBody invert each other', () => {
    const q = new Quaternion().setFromAxisAngle(new Vector3(0.3, 0.5, 0.81).normalize(), 0.7);
    const v = new Vector3(12, -3, 4);
    const world = bodyToWorld(q, v, new Vector3());
    const back = worldToBody(q, world, new Vector3());
    expect(back.distanceTo(v)).toBeLessThan(1e-12);
  });

  it('the airspeed vector removes the wind and rotates into body axes', () => {
    const state = createState();
    state.velocity.set(100, 0, 0); // 100 m/s to the north
    const wind = new Vector3(20, 0, 0); // a 20 m/s tailwind from the south
    const air = airspeedBody(state, wind, new Vector3());
    expect(air.x).toBeCloseTo(80, 12);
    expect(air.y).toBeCloseTo(0, 12);
    expect(air.z).toBeCloseTo(0, 12);
  });
});

describe('flow angles', () => {
  it('pure forward flight gives an angle of attack and a sideslip of zero', () => {
    const angles = flowAngles(new Vector3(180, 0, 0));
    expect(angles.alpha).toBe(0);
    expect(angles.beta).toBe(0);
    expectRelative(angles.speed, 180, 1e-12);
  });

  it('a positive body z part of the airspeed gives a positive angle of attack', () => {
    // Positive z is down, so the air comes from below. That is a positive alpha.
    const angles = flowAngles(new Vector3(180, 0, 18));
    expect(angles.alpha).toBeGreaterThan(0);
    expectRelative(angles.alpha, Math.atan2(18, 180), 1e-12);
  });

  it('a positive body y part of the airspeed gives a positive sideslip', () => {
    // Positive y is right, so the air comes from the right. That is a positive beta.
    const angles = flowAngles(new Vector3(180, 18, 0));
    expect(angles.beta).toBeGreaterThan(0);
    expectRelative(angles.beta, Math.asin(18 / Math.hypot(180, 18)), 1e-12);
  });

  it('a nose up attitude in level flight gives an angle of attack of the same size', () => {
    // A rotation about the body y axis raises the nose, as CONVENTIONS 3.1 says.
    const pitch = toRad(10);
    const state = createState();
    state.orientation.setFromAxisAngle(new Vector3(0, 1, 0), pitch);
    state.velocity.set(180, 0, 0); // level flight to the north
    const air = airspeedBody(state, new Vector3(0, 0, 0), new Vector3());
    const angles = flowAngles(air);
    expectRelative(angles.alpha, pitch, 1e-9);
    expect(Math.abs(angles.beta)).toBeLessThan(1e-12);
  });

  it('a standing aircraft reports no angle of attack and no sideslip', () => {
    const angles = flowAngles(new Vector3(0, 0, 0));
    expect(angles.alpha).toBe(0);
    expect(angles.beta).toBe(0);
    expect(angles.speed).toBe(0);
  });

  it('flowAngles writes into the result the caller gives', () => {
    const out = { alpha: 1, beta: 1, speed: 1 };
    const result = flowAngles(new Vector3(100, 0, 0), out);
    expect(result).toBe(out);
    expect(out.alpha).toBe(0);
    expect(out.speed).toBe(100);
  });
});

describe('mass properties', () => {
  it('the inverse inertia times the inertia is the identity', () => {
    const inertia = new Matrix3().set(12000, -500, 300, -500, 25000, -200, 300, -200, 35000);
    const mass = createMassProperties(6396, inertia);
    const product = mass.inertia.clone().multiply(mass.inverseInertia);
    const identity = new Matrix3();
    for (let i = 0; i < 9; i++) {
      expect(Math.abs(product.elements[i] - identity.elements[i])).toBeLessThan(1e-12);
    }
  });

  it('a singular inertia tensor throws', () => {
    expect(() => createMassProperties(6396, diagonalInertia(12000, 25000, 0))).toThrow(/singular/);
    expect(() => createMassProperties(0, diagonalInertia(1, 2, 3))).toThrow(/mass/);
  });

  it('an empty wrench holds no force and no moment', () => {
    const w = createWrench();
    expect(w.force.length()).toBe(0);
    expect(w.moment.length()).toBe(0);
  });
});
