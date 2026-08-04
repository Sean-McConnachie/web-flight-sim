import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import { NACA_0009, NACA_0011 } from '@/physics/aero/airfoil';
import type { AeroCoefficients } from '@/physics/aero/airfoil';
import { STALL_NACA_0009, STALL_NACA_0011 } from '@/physics/aero/stall';
import { createSurface, estimateSurfaceLoad, evaluateSurface } from '@/physics/aero/surface';
import type { Surface, SurfaceDef } from '@/physics/aero/surface';
import { clearWrench, createWrench } from '@/physics/rigidbody';
import type { Wrench } from '@/physics/rigidbody';
import { DEG } from '@/math/units';
import { SEA_LEVEL_DENSITY } from '@/physics/atmosphere';
import { createMachCorrection, machCorrection } from '@/physics/aero/compressibility';

// Standard day at sea level. The tests hold the atmosphere still, so the
// aerodynamics is the only thing under test.
const RHO = SEA_LEVEL_DENSITY;
const SPEED_OF_SOUND = 340.294; // m/s, ISA sea level
const STEP = 1 / 240; // s, the flight step of the model
const NO_CONTROLS = new Float64Array(4);
const ZERO = new Vector3(0, 0, 0);

/**
 * One strip of the Me-262 wing, with every field at a neutral value. The tests
 * change one field at a time, so each test states one physical fact.
 * Span 12.51 m, area 21.7 m2, aspect ratio 7.21. CONVENTIONS section 8.
 */
function strip(over: Partial<SurfaceDef> = {}): SurfaceDef {
  return {
    name: 'strip',
    position: new Vector3(0, 3, 0),
    span: 1,
    chord: 1.735,
    area: 1.735,
    incidence: 0,
    dihedral: 0,
    sweep: 0,
    airfoil: NACA_0011,
    stall: STALL_NACA_0011,
    aspectRatio: 7.21,
    oswaldEfficiency: 0.85,
    controlIndex: -1,
    controlEffectiveness: 0,
    flapIndex: -1,
    flapEffectiveness: 0,
    flapClMaxDelta: 0,
    flapAlphaDelta: 0,
    hasSlat: false,
    slatAlphaDelta: 0,
    slatDeployAlpha: 0,
    ...over,
  };
}

/** Runs one strip at a flow and returns the wrench it made. */
function run(
  s: Surface,
  velocity: Vector3,
  options: {
    omega?: Vector3;
    wind?: Vector3;
    controls?: Float64Array;
    induced?: number;
    dt?: number;
    steps?: number;
  } = {},
): Wrench {
  const out = createWrench();
  const steps = options.steps ?? 1;
  for (let i = 0; i < steps; i++) {
    clearWrench(out);
    evaluateSurface(
      s,
      velocity,
      options.omega ?? ZERO,
      options.wind ?? ZERO,
      RHO,
      SPEED_OF_SOUND,
      options.controls ?? NO_CONTROLS,
      options.induced ?? 0,
      options.dt ?? STEP,
      out,
    );
  }
  return out;
}

/**
 * The Prandtl-Glauert lift growth of compressibility.ts at one speed. Every
 * comparison against a raw section table needs it, because a strip carries the
 * Mach correction and the table does not.
 */
function liftScale(speed: number, sweep: number = 0): number {
  const correction = createMachCorrection();
  machCorrection(speed / SPEED_OF_SOUND, sweep, correction);
  return correction.clScale;
}

/** The velocity of the aircraft through the air, at an angle of attack. */
function flightVelocity(speed: number, alpha: number, beta: number = 0): Vector3 {
  return new Vector3(
    speed * Math.cos(alpha) * Math.cos(beta),
    speed * Math.sin(beta),
    speed * Math.sin(alpha) * Math.cos(beta),
  );
}

describe('the sign of the strip force', () => {
  it('makes lift up, which is a negative body z force, at a positive angle of attack', () => {
    const s = createSurface(strip());
    const w = run(s, flightVelocity(100, 5 * DEG), { steps: 200 });
    expect(w.force.z).toBeLessThan(0);
    expect(s.result.cl).toBeGreaterThan(0);
  });

  it('makes lift down at a negative angle of attack', () => {
    const s = createSurface(strip());
    const w = run(s, flightVelocity(100, -5 * DEG), { steps: 200 });
    expect(w.force.z).toBeGreaterThan(0);
    expect(s.result.cl).toBeLessThan(0);
  });

  it('makes drag backwards, which is a negative body x force, in forward flight', () => {
    const s = createSurface(strip());
    const w = run(s, flightVelocity(100, 0), { steps: 200 });
    expect(w.force.x).toBeLessThan(0);
    expect(w.force.y).toBeCloseTo(0, 9);
    expect(w.force.z).toBeCloseTo(0, 9);
  });

  it('matches the section table at zero induced angle', () => {
    const alpha = 4 * DEG;
    const s = createSurface(strip());
    run(s, flightVelocity(100, alpha), { steps: 400 });
    const section: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
    NACA_0011.sample(alpha, section);
    // The lagged separation point has settled, so the dynamic ratio is 1 and the
    // static table is in charge, as stall.ts asks. The only thing left on top of
    // the table is the Prandtl-Glauert lift growth of compressibility.ts.
    expect(s.result.cl).toBeCloseTo(section.cl * liftScale(100), 4);
    expect(s.result.cd).toBeCloseTo(section.cd, 4);
    const q = 0.5 * RHO * 100 * 100;
    expect(-s.result.force.z).toBeCloseTo(
      q * 1.735 * (s.result.cl * Math.cos(alpha) + s.result.cd * Math.sin(alpha)),
      3,
    );
  });
});

describe('the local flow of a strip', () => {
  it('reads the built in incidence as an angle of attack in level flight', () => {
    const s = createSurface(strip({ incidence: 3 * DEG }));
    run(s, flightVelocity(100, 0), { steps: 200 });
    expect(s.result.alpha).toBeCloseTo(3 * DEG, 9);
  });

  it('subtracts the wind, so a tailwind of the same speed makes no force', () => {
    const s = createSurface(strip());
    const velocity = flightVelocity(100, 5 * DEG);
    const w = run(s, velocity, { wind: velocity.clone(), steps: 10 });
    expect(w.force.length()).toBeCloseTo(0, 9);
  });

  it('adds omega cross r, so a positive roll rate raises the angle of the right strip', () => {
    const s = createSurface(strip({ position: new Vector3(0, 4, 0) }));
    run(s, flightVelocity(100, 2 * DEG), { steps: 200 });
    const level = s.result.alpha;
    run(s, flightVelocity(100, 2 * DEG), { omega: new Vector3(0.3, 0, 0), steps: 200 });
    expect(s.result.alpha).toBeGreaterThan(level);
    // The rise is the rate times the arm over the speed, to first order.
    expect(s.result.alpha - level).toBeCloseTo((0.3 * 4) / 100, 3);
  });

  it('adds omega cross r, so a positive pitch rate raises the angle of a tail strip', () => {
    const s = createSurface(strip({ position: new Vector3(-4.5, 1, 0) }));
    run(s, flightVelocity(100, 2 * DEG), { steps: 200 });
    const level = s.result.alpha;
    run(s, flightVelocity(100, 2 * DEG), { omega: new Vector3(0, 0.2, 0), steps: 200 });
    expect(s.result.alpha - level).toBeCloseTo((0.2 * 4.5) / 100, 3);
  });
});

describe('roll damping and pitch damping are emergent', () => {
  /** A symmetric pair of strips, one on each side. */
  function pair(over: Partial<SurfaceDef> = {}): Surface[] {
    return [
      createSurface(strip({ name: 'right', position: new Vector3(0, 4, 0), ...over })),
      createSurface(strip({ name: 'left', position: new Vector3(0, -4, 0), ...over })),
    ];
  }

  function runPair(surfaces: Surface[], velocity: Vector3, omega: Vector3): Wrench {
    const out = createWrench();
    for (let i = 0; i < 200; i++) {
      clearWrench(out);
      for (const s of surfaces) {
        evaluateSurface(
          s,
          velocity,
          omega,
          ZERO,
          RHO,
          SPEED_OF_SOUND,
          NO_CONTROLS,
          0,
          STEP,
          out,
        );
      }
    }
    return out;
  }

  it('gives no roll moment at zero roll rate', () => {
    const w = runPair(pair(), flightVelocity(100, 4 * DEG), ZERO);
    expect(w.moment.x).toBeCloseTo(0, 9);
  });

  it('opposes a positive roll rate with a negative roll moment', () => {
    const w = runPair(pair(), flightVelocity(100, 4 * DEG), new Vector3(0.4, 0, 0));
    expect(w.moment.x).toBeLessThan(0);
  });

  it('opposes a negative roll rate with a positive roll moment', () => {
    const w = runPair(pair(), flightVelocity(100, 4 * DEG), new Vector3(-0.4, 0, 0));
    expect(w.moment.x).toBeGreaterThan(0);
  });

  it('opposes a positive pitch rate with a nose down moment on a tail', () => {
    const tail = pair({ position: new Vector3(-4.5, 1.2, 0) });
    tail[1].def.position.set(-4.5, -1.2, 0);
    const level = runPair(tail, flightVelocity(100, 0), ZERO).moment.y;
    const pitching = runPair(tail, flightVelocity(100, 0), new Vector3(0, 0.3, 0)).moment.y;
    expect(pitching).toBeLessThan(level);
    expect(pitching).toBeLessThan(0);
  });
});

describe('sweep', () => {
  it('cuts the lift of a strip by the cosine of the sweep angle', () => {
    const alpha = 3 * DEG;
    const plain = createSurface(strip({ sweep: 0 }));
    const swept = createSurface(strip({ sweep: 18.5 * DEG }));
    run(plain, flightVelocity(100, alpha), { steps: 400 });
    run(swept, flightVelocity(100, alpha), { steps: 400 });
    const ratio = swept.result.force.z / plain.result.force.z;
    expect(ratio).toBeCloseTo(Math.cos(18.5 * DEG), 2);
  });

  it('raises the section angle of attack by one over the cosine of the sweep angle', () => {
    const alpha = 6 * DEG;
    const swept = createSurface(strip({ sweep: 18.5 * DEG }));
    run(swept, flightVelocity(100, alpha), { steps: 10 });
    expect(swept.result.alpha).toBeCloseTo(Math.atan(Math.tan(alpha) / Math.cos(18.5 * DEG)), 6);
  });

  it('gives the windward panel more lift than the leeward panel in a sideslip', () => {
    // Positive sideslip puts the air on the right side of the aircraft, so the
    // right panel is windward. It meets less effective sweep and makes more
    // lift. That is the dihedral effect of a swept wing.
    const velocity = flightVelocity(100, 4 * DEG, 8 * DEG);
    const right = createSurface(strip({ position: new Vector3(0, 4, 0), sweep: 18.5 * DEG }));
    const left = createSurface(strip({ position: new Vector3(0, -4, 0), sweep: 18.5 * DEG }));
    run(right, velocity, { steps: 400 });
    run(left, velocity, { steps: 400 });
    expect(-right.result.force.z).toBeGreaterThan(-left.result.force.z);
  });
});

describe('dihedral', () => {
  it('lifts both tips for one positive angle, so both normals lean outboard', () => {
    const right = createSurface(strip({ position: new Vector3(0, 4, 0), dihedral: 6 * DEG }));
    const left = createSurface(strip({ position: new Vector3(0, -4, 0), dihedral: 6 * DEG }));
    const velocity = flightVelocity(100, 4 * DEG);
    run(right, velocity, { steps: 200 });
    run(left, velocity, { steps: 200 });
    // Mirror images make mirrored side forces and the same lift.
    expect(right.result.force.z).toBeCloseTo(left.result.force.z, 6);
    expect(right.result.force.y).toBeCloseTo(-left.result.force.y, 6);
    expect(right.result.force.y).toBeLessThan(0);
  });

  it('raises the angle of the windward panel in a sideslip', () => {
    const velocity = flightVelocity(100, 4 * DEG, 8 * DEG);
    const right = createSurface(strip({ position: new Vector3(0, 4, 0), dihedral: 6 * DEG }));
    const left = createSurface(strip({ position: new Vector3(0, -4, 0), dihedral: 6 * DEG }));
    run(right, velocity, { steps: 200 });
    run(left, velocity, { steps: 200 });
    expect(right.result.alpha).toBeGreaterThan(left.result.alpha);
  });

  it('turns a strip of 90 degrees of dihedral into a fin that weathercocks', () => {
    // A fin behind the center of gravity must push the nose back into the wind.
    const fin = createSurface(
      strip({ name: 'fin', position: new Vector3(-4.5, 0, -1), dihedral: 90 * DEG }),
    );
    const w = run(fin, flightVelocity(100, 0, 6 * DEG), { steps: 200 });
    // Positive sideslip means the nose sits left of the flight path. The fin
    // force goes left and the yaw moment goes nose right, which is positive.
    expect(w.force.y).toBeLessThan(0);
    expect(w.moment.z).toBeGreaterThan(0);
  });
});

describe('the induced angle', () => {
  it('takes the induced angle off the section angle of attack', () => {
    const alpha = 6 * DEG;
    const induced = 1.5 * DEG;
    const s = createSurface(strip());
    run(s, flightVelocity(100, alpha), { induced, steps: 400 });
    expect(s.result.alpha).toBeCloseTo(alpha - induced, 6);
  });

  it('leans the lift vector back, which is the induced drag, and adds no drag term', () => {
    const alpha = 8 * DEG;
    const induced = 2 * DEG;
    const s = createSurface(strip());
    run(s, flightVelocity(100, alpha), { induced, steps: 400 });
    const withInduced = s.result;
    const lift = -withInduced.force.z * Math.cos(alpha) + withInduced.force.x * Math.sin(alpha);
    const drag = -(withInduced.force.x * Math.cos(alpha) + withInduced.force.z * Math.sin(alpha));
    const profile = withInduced.dynamicPressure * 1.735 * withInduced.cd;
    // The drag above the profile drag is the lift times the induced angle.
    const expected = lift * Math.tan(induced);
    expect(Math.abs((drag - profile) / expected - 1)).toBeLessThan(1e-3);
  });

  it('divides the induced angle by the cosine of the sweep, as it does the angle of attack', () => {
    const alpha = 6 * DEG;
    const induced = 2 * DEG;
    const sweep = 18.5 * DEG;
    const s = createSurface(strip({ sweep }));
    run(s, flightVelocity(100, alpha), { induced, steps: 10 });
    const geometric = Math.atan(Math.tan(alpha) / Math.cos(sweep));
    expect(s.result.alpha).toBeCloseTo(geometric - induced / Math.cos(sweep), 6);
  });
});

describe('the control and the flap', () => {
  it('moves the zero lift angle by the effectiveness times the deflection', () => {
    const controls = new Float64Array([10 * DEG, 0, 0, 0]);
    const s = createSurface(strip({ controlIndex: 0, controlEffectiveness: -0.45 }));
    run(s, flightVelocity(100, 0), { controls, steps: 400 });
    // A negative effectiveness with a positive deflection makes lift, which is
    // the usual sign of a trailing edge surface that deflects down.
    expect(s.result.alpha).toBeCloseTo(0.45 * 10 * DEG, 9);
    expect(s.result.cl).toBeGreaterThan(0);
    expect(s.result.force.z).toBeLessThan(0);
  });

  it('adds the flap shift to the control shift', () => {
    const controls = new Float64Array([10 * DEG, 20 * DEG, 0, 0]);
    const s = createSurface(
      strip({
        controlIndex: 0,
        controlEffectiveness: -0.45,
        flapIndex: 1,
        flapEffectiveness: -0.5,
      }),
    );
    run(s, flightVelocity(100, 0), { controls, steps: 10 });
    expect(s.result.alpha).toBeCloseTo(0.45 * 10 * DEG + 0.5 * 20 * DEG, 9);
  });

  it('loses control power at a high Mach number', () => {
    const controls = new Float64Array([10 * DEG, 0, 0, 0]);
    const s = createSurface(strip({ controlIndex: 0, controlEffectiveness: -0.45 }));
    run(s, flightVelocity(100, 0), { controls, steps: 10 });
    const slow = s.result.alpha;
    run(s, flightVelocity(0.86 * SPEED_OF_SOUND, 0), { controls, steps: 10 });
    expect(s.result.alpha).toBeLessThan(0.6 * slow);
  });
});

describe('the slat', () => {
  const slatted = strip({
    airfoil: NACA_0009,
    stall: STALL_NACA_0009,
    hasSlat: true,
    slatAlphaDelta: 6 * DEG,
    slatDeployAlpha: 8 * DEG,
  });
  const plain = { ...slatted, hasSlat: false };

  it('stays shut and changes nothing below the deploy angle', () => {
    const a = createSurface(slatted);
    const b = createSurface(plain);
    run(a, flightVelocity(100, 5 * DEG), { steps: 400 });
    run(b, flightVelocity(100, 5 * DEG), { steps: 400 });
    expect(a.result.slatOpen).toBe(false);
    expect(a.result.cl).toBeCloseTo(b.result.cl, 12);
  });

  it('opens above the deploy angle and holds the flow on', () => {
    const a = createSurface(slatted);
    const b = createSurface(plain);
    run(a, flightVelocity(100, 15 * DEG), { steps: 400 });
    run(b, flightVelocity(100, 15 * DEG), { steps: 400 });
    expect(a.result.slatOpen).toBe(true);
    // The plain section is past its 13 degree stall. The slatted one is not.
    expect(a.result.separation).toBeGreaterThan(b.result.separation);
    expect(a.result.cl).toBeGreaterThan(b.result.cl);
  });

  it('raises the peak lift and the angle of the peak by about the slat delta', () => {
    const a = createSurface(slatted);
    const b = createSurface(plain);
    let peakA = 0;
    let peakB = 0;
    let angleA = 0;
    let angleB = 0;
    for (let deg = 0; deg <= 30; deg += 0.25) {
      run(a, flightVelocity(100, deg * DEG), { steps: 300 });
      run(b, flightVelocity(100, deg * DEG), { steps: 300 });
      if (a.result.cl > peakA) {
        peakA = a.result.cl;
        angleA = deg;
      }
      if (b.result.cl > peakB) {
        peakB = b.result.cl;
        angleB = deg;
      }
    }
    expect(angleA - angleB).toBeGreaterThan(4);
    expect(angleA - angleB).toBeLessThan(8);
    expect(peakA).toBeGreaterThan(peakB);
  });

  it('adds drag when it opens', () => {
    const a = createSurface(slatted);
    const b = createSurface(plain);
    run(a, flightVelocity(100, 12 * DEG), { steps: 400 });
    run(b, flightVelocity(100, 12 * DEG), { steps: 400 });
    expect(a.result.cd).toBeGreaterThan(b.result.cd);
  });
});

describe('dynamic stall', () => {
  it('holds more lift than the static table while the angle of attack rises fast', () => {
    const section: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
    const s = createSurface(strip({ chord: 2 }));
    // Start well below the stall with the flow settled, then jump past it.
    run(s, flightVelocity(60, 6 * DEG), { steps: 500 });
    run(s, flightVelocity(60, 17 * DEG), { steps: 1, dt: 1 / 240 });
    NACA_0011.sample(17 * DEG, section);
    const steady = section.cl * liftScale(60);
    expect(s.result.cl).toBeGreaterThan(steady);
    // The lag runs out and the static table takes charge again.
    run(s, flightVelocity(60, 17 * DEG), { steps: 500 });
    expect(s.result.cl).toBeCloseTo(steady, 4);
  });

  it('freezes the separation point when dt is zero', () => {
    const s = createSurface(strip());
    run(s, flightVelocity(100, 2 * DEG), { steps: 200 });
    const settled = s.result.separation;
    run(s, flightVelocity(100, 20 * DEG), { dt: 0, steps: 5 });
    expect(s.result.separation).toBeCloseTo(settled, 12);
  });
});

describe('the moment of a strip', () => {
  it('carries the arm of the force about the center of gravity', () => {
    const s = createSurface(strip({ position: new Vector3(-4.5, 2, 0) }));
    run(s, flightVelocity(100, 5 * DEG), { steps: 200 });
    const arm = new Vector3(-4.5, 2, 0).cross(s.result.force.clone());
    // The section moment about the quarter chord is small next to the arm term,
    // so the two must agree on every axis to within that section moment.
    const sectionMoment = s.result.dynamicPressure * 1.735 * 1.735 * s.result.cm;
    expect(s.result.moment.x).toBeCloseTo(arm.x, 6);
    expect(s.result.moment.y - arm.y).toBeCloseTo(sectionMoment, 6);
    expect(s.result.moment.z).toBeCloseTo(arm.z, 6);
  });

  it('makes a nose down moment when a tail strip behind the center of gravity lifts', () => {
    const s = createSurface(strip({ position: new Vector3(-4.5, 1, 0) }));
    run(s, flightVelocity(100, 5 * DEG), { steps: 200 });
    expect(s.result.force.z).toBeLessThan(0);
    expect(s.result.moment.y).toBeLessThan(0);
  });
});

describe('the linear load estimate', () => {
  it('matches the full evaluation in attached flow', () => {
    const alpha = 4 * DEG;
    const s = createSurface(strip());
    run(s, flightVelocity(100, alpha), { steps: 400 });
    const load = { lift: 0, slope: 0 };
    estimateSurfaceLoad(
      s,
      flightVelocity(100, alpha),
      ZERO,
      ZERO,
      RHO,
      SPEED_OF_SOUND,
      NO_CONTROLS,
      load,
    );
    const lift = s.result.dynamicPressure * 1.735 * s.result.cl;
    expect(load.lift).toBeCloseTo(lift, 3);
    expect(load.slope).toBeGreaterThan(0);
  });

  it('falls with the separation point past the stall', () => {
    const s = createSurface(strip());
    const load = { lift: 0, slope: 0 };
    run(s, flightVelocity(100, 4 * DEG), { steps: 400 });
    estimateSurfaceLoad(
      s,
      flightVelocity(100, 4 * DEG),
      ZERO,
      ZERO,
      RHO,
      SPEED_OF_SOUND,
      NO_CONTROLS,
      load,
    );
    const attachedSlope = load.slope;
    run(s, flightVelocity(100, 25 * DEG), { steps: 400 });
    estimateSurfaceLoad(
      s,
      flightVelocity(100, 25 * DEG),
      ZERO,
      ZERO,
      RHO,
      SPEED_OF_SOUND,
      NO_CONTROLS,
      load,
    );
    expect(load.slope).toBeLessThan(0.5 * attachedSlope);
  });
});

describe('evaluateSurface allocates nothing', () => {
  it('returns the same result objects and the same vectors on every call', () => {
    const s = createSurface(strip());
    const out = createWrench();
    const result = s.result;
    const force = s.result.force;
    const moment = s.result.moment;
    const lastForce = s.state.lastForce;
    for (let i = 0; i < 1000; i++) {
      clearWrench(out);
      evaluateSurface(
        s,
        flightVelocity(100, (i % 20) * DEG),
        ZERO,
        ZERO,
        RHO,
        SPEED_OF_SOUND,
        NO_CONTROLS,
        0,
        STEP,
        out,
      );
    }
    expect(s.result).toBe(result);
    expect(s.result.force).toBe(force);
    expect(s.result.moment).toBe(moment);
    expect(s.state.lastForce).toBe(lastForce);
    expect(Number.isFinite(out.force.x)).toBe(true);
    expect(Number.isFinite(out.moment.y)).toBe(true);
  });
});
