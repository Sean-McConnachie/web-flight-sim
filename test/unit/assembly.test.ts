import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import { NACA_0009, NACA_0011, blendAirfoils } from '@/physics/aero/airfoil';
import type { Airfoil } from '@/physics/aero/airfoil';
import { STALL_NACA_0009, STALL_NACA_0011 } from '@/physics/aero/stall';
import type { StallParams } from '@/physics/aero/stall';
import { createMachCorrection, machCorrection } from '@/physics/aero/compressibility';
import { createAssembly } from '@/physics/aero/assembly';
import type { AeroAssembly, AeroTotals, GroupDef } from '@/physics/aero/assembly';
import type { SurfaceDef } from '@/physics/aero/surface';
import type { BodyDef } from '@/physics/aero/body';
import { clearWrench, createState, createWrench } from '@/physics/rigidbody';
import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import { lerp } from '@/math/tables';
import { DEG } from '@/math/units';
import { SEA_LEVEL_DENSITY } from '@/physics/atmosphere';

// The Me-262 A-1a. CONVENTIONS section 8, confidence: firm.
const SPAN = 12.51; // m
const WING_AREA = 21.7; // m2
const ASPECT_RATIO = 7.21;
const SWEEP = 18.5 * DEG; // rad
const MEAN_CHORD = WING_AREA / SPAN; // m, 1.735
const OSWALD = 0.85; // estimate, typical of a wing of this planform

const SPEED_OF_SOUND = 340.294; // m/s, ISA sea level
const RHO = SEA_LEVEL_DENSITY;
const STEP = 1 / 240; // s
const ZERO = new Vector3(0, 0, 0);

// The control array of the tests. 0 aileron, 1 elevator, 2 rudder.
const AILERON = 0;
const ELEVATOR = 1;
const RUDDER = 2;
const NEUTRAL = new Float64Array(3);

/** The Prandtl-Glauert lift growth of compressibility.ts at one speed. */
function liftScale(speed: number, sweep: number = 0): number {
  const correction = createMachCorrection();
  machCorrection(speed / SPEED_OF_SOUND, sweep, correction);
  return correction.clScale;
}

/**
 * The two dimensional lift curve slope of a section, read from its own table
 * over the same small angle band the wing tests use. This is the honest a0 for
 * the finite span comparison: it already holds the small loss that the
 * separation function makes below the stall.
 */
function sectionSlope(airfoil: Airfoil): number {
  const low = { cl: 0, cd: 0, cm: 0 };
  const high = { cl: 0, cd: 0, cm: 0 };
  airfoil.sample(1 * DEG, low);
  airfoil.sample(3 * DEG, high);
  return (high.cl - low.cl) / (2 * DEG);
}

/** The finite span lift curve slope of lifting line theory. */
function finiteSpanSlope(a0: number, aspectRatio: number, oswald: number): number {
  return a0 / (1 + a0 / (Math.PI * oswald * aspectRatio));
}

interface WingOptions {
  strips?: number; // per side
  sweep?: number;
  span?: number;
  area?: number;
  taper?: number;
  /** Washout at the tip, rad. A negative value lowers the tip incidence. */
  twist?: number;
  dihedral?: number;
  /** One section for the whole wing. Without it the wing blends root to tip. */
  airfoil?: Airfoil;
  stall?: StallParams;
  aspectRatio?: number;
  oswald?: number;
  /** Quarter chord of the root, meters ahead of the center of gravity. */
  x?: number;
  controlIndex?: number;
  /** Fraction of the semi span where the control starts. */
  controlFrom?: number;
  controlEffectiveness?: number;
  /** Fraction of the semi span where the slats start. Leave it out for none. */
  slatFrom?: number;
  name?: string;
}

interface Part {
  surfaces: SurfaceDef[];
  group: Omit<GroupDef, 'surfaceIndices'>;
}

/**
 * Builds one lifting surface out of strips, left half and right half together.
 *
 * The strips sample the semi span at the middle of each band, so the areas add
 * up to the reference area exactly for a linear taper. Each strip sits at its
 * own quarter chord, which the sweep carries aft.
 */
function wing(options: WingOptions = {}): Part {
  const n = options.strips ?? 8;
  const span = options.span ?? SPAN;
  const area = options.area ?? WING_AREA;
  const taper = options.taper ?? 0.5;
  const sweep = options.sweep ?? 0;
  const twist = options.twist ?? 0;
  const dihedral = options.dihedral ?? 0;
  const aspectRatio = options.aspectRatio ?? (span * span) / area;
  const oswald = options.oswald ?? OSWALD;
  const x = options.x ?? 0;
  const name = options.name ?? 'wing';

  const meanChord = area / span;
  const rootChord = (2 * meanChord) / (1 + taper);
  const width = span / 2 / n;
  const surfaces: SurfaceDef[] = [];

  for (const side of [1, -1]) {
    for (let i = 0; i < n; i++) {
      const eta = (i + 0.5) / n; // fraction of the semi span
      const chord = lerp(rootChord, rootChord * taper, eta);
      const y = side * eta * (span / 2);
      const controlled =
        options.controlIndex !== undefined && eta >= (options.controlFrom ?? 0);
      const slatted = options.slatFrom !== undefined && eta >= options.slatFrom;
      surfaces.push({
        name: `${name}-${side > 0 ? 'r' : 'l'}${i}`,
        position: new Vector3(x - Math.abs(y) * Math.tan(sweep), y, 0),
        span: width,
        chord,
        area: chord * width,
        incidence: twist * eta,
        dihedral,
        sweep,
        airfoil:
          options.airfoil ?? blendAirfoils(NACA_0011, NACA_0009, eta, `${name}-section-${i}`),
        stall:
          options.stall ??
          ({
            a1: lerp(STALL_NACA_0011.a1, STALL_NACA_0009.a1, eta),
            s1: lerp(STALL_NACA_0011.s1, STALL_NACA_0009.s1, eta),
            s2: lerp(STALL_NACA_0011.s2, STALL_NACA_0009.s2, eta),
            tf: STALL_NACA_0011.tf,
          } satisfies StallParams),
        aspectRatio,
        oswaldEfficiency: oswald,
        // A control on the right half and on the left half must work against
        // each other for an aileron and together for an elevator. The sign of
        // the effectiveness carries that, and side carries the mirror.
        controlIndex: controlled ? (options.controlIndex as number) : -1,
        controlEffectiveness: controlled
          ? (options.controlEffectiveness ?? 0.45) * (options.controlIndex === AILERON ? side : 1)
          : 0,
        flapIndex: -1,
        flapEffectiveness: 0,
        flapClMaxDelta: 0,
        flapAlphaDelta: 0,
        hasSlat: slatted,
        slatAlphaDelta: slatted ? 6 * DEG : 0,
        slatDeployAlpha: slatted ? 8 * DEG : 0,
      });
    }
  }

  return { surfaces, group: { name, aspectRatio, oswaldEfficiency: oswald, area } };
}

/**
 * The Me-262 fuselage. Length 9.5 m inside an aircraft of 10.60 m, maximum
 * width 1.5 m. The volume, the side area and the frontal area follow with the
 * usual shape factors. Every one is an estimate. Bead b17 owns the real set.
 */
function fuselage(over: Partial<BodyDef> = {}): BodyDef {
  return {
    name: 'fuselage',
    position: new Vector3(0.3, 0, 0),
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

/** The horizontal tail. Area 3.7 m2 on an arm of 4.5 m, from the general layout. */
function tailplane(): Part {
  return wing({
    name: 'tail',
    strips: 3,
    span: 4.2,
    area: 3.7,
    taper: 0.6,
    sweep: 10 * DEG,
    x: -4.5,
    airfoil: NACA_0009,
    stall: STALL_NACA_0009,
    oswald: 0.8,
    controlIndex: ELEVATOR,
    controlEffectiveness: -0.5,
  });
}

/** The fin. A strip set with 90 degrees of dihedral is a fin. */
function fin(): Part {
  const n = 3;
  const area = 2.0;
  const height = 1.8;
  const width = height / n;
  const surfaces: SurfaceDef[] = [];
  for (let i = 0; i < n; i++) {
    const eta = (i + 0.5) / n;
    const chord = area / height;
    surfaces.push({
      name: `fin-${i}`,
      position: new Vector3(-4.6 - eta * height * Math.tan(20 * DEG), 0, -eta * height),
      span: width,
      chord,
      area: chord * width,
      incidence: 0,
      dihedral: 90 * DEG,
      sweep: 20 * DEG,
      airfoil: NACA_0009,
      stall: STALL_NACA_0009,
      // A fin works against the fuselage, so its effective aspect ratio is
      // higher than its geometric one. Confidence: estimate.
      aspectRatio: (height * height) / area,
      oswaldEfficiency: 0.7,
      controlIndex: RUDDER,
      controlEffectiveness: -0.45,
      flapIndex: -1,
      flapEffectiveness: 0,
      flapClMaxDelta: 0,
      flapAlphaDelta: 0,
      hasSlat: false,
      slatAlphaDelta: 0,
      slatDeployAlpha: 0,
    });
  }
  return {
    surfaces,
    group: {
      name: 'fin',
      aspectRatio: (height * height) / area,
      oswaldEfficiency: 0.7,
      area,
    },
  };
}

/** Joins parts and bodies into one assembly, with the group indices worked out. */
function assemble(parts: Part[], bodies: BodyDef[] = []): AeroAssembly {
  const surfaces: SurfaceDef[] = [];
  const groups: GroupDef[] = [];
  for (const part of parts) {
    const start = surfaces.length;
    for (const s of part.surfaces) {
      surfaces.push(s);
    }
    const indices: number[] = [];
    for (let i = start; i < surfaces.length; i++) {
      indices.push(i);
    }
    groups.push({ ...part.group, surfaceIndices: indices });
  }
  return createAssembly(surfaces, bodies, groups);
}

/** A state at one speed, angle of attack and sideslip, level at sea level. */
function flightState(
  speed: number,
  alpha: number,
  beta: number = 0,
  omega: Vector3 = ZERO,
): RigidBodyState {
  const state = createState();
  // The orientation is the identity, so the body frame and the world frame
  // agree and the world velocity is the body velocity.
  state.velocity.set(
    speed * Math.cos(alpha) * Math.cos(beta),
    speed * Math.sin(beta),
    speed * Math.sin(alpha) * Math.cos(beta),
  );
  state.angularVelocity.copy(omega);
  return state;
}

const wrench: Wrench = createWrench();

/**
 * One frozen reading of the model.
 *
 * The assembly reuses its totals object and its result objects on every call, so
 * a test that compares two conditions must copy what it needs before it runs the
 * next one. The allocation test below holds that reuse on purpose.
 */
interface Snapshot {
  alpha: number;
  beta: number;
  mach: number;
  dynamicPressure: number;
  trueAirspeed: number;
  lift: number;
  drag: number;
  sideForce: number;
  force: Vector3;
  moment: Vector3;
}

/**
 * Runs the assembly to a settled condition and copies the answer out. The
 * separation point of every strip lags, so a test that wants the steady answer
 * must hold the state still until the lag runs out.
 */
function settle(
  assembly: AeroAssembly,
  state: RigidBodyState,
  controls: Float64Array = NEUTRAL,
  steps: number = 400,
): Snapshot {
  let last: AeroTotals | undefined;
  for (let i = 0; i < steps; i++) {
    clearWrench(wrench);
    last = assembly.evaluate(state, ZERO, controls, STEP, wrench);
  }
  const totals = last as AeroTotals;
  return {
    alpha: totals.alpha,
    beta: totals.beta,
    mach: totals.mach,
    dynamicPressure: totals.dynamicPressure,
    trueAirspeed: totals.trueAirspeed,
    lift: totals.lift,
    drag: totals.drag,
    sideForce: totals.sideForce,
    force: wrench.force.clone(),
    moment: wrench.moment.clone(),
  };
}

describe('the finite span lift curve slope', () => {
  // A plain wing: one section, no sweep, no twist, no taper effects on the
  // section. The only thing between the two dimensional slope and the measured
  // slope is the induced angle.
  const plain = () =>
    assemble([
      wing({
        strips: 10,
        sweep: 0,
        taper: 1,
        airfoil: NACA_0011,
        stall: STALL_NACA_0011,
        aspectRatio: ASPECT_RATIO,
        oswald: OSWALD,
      }),
    ]);

  function measureSlope(assembly: AeroAssembly, speed: number, sweep: number = 0): number {
    const low = settle(assembly, flightState(speed, 1 * DEG));
    const high = settle(assembly, flightState(speed, 3 * DEG));
    const q = 0.5 * RHO * speed * speed;
    return (high.lift - low.lift) / (q * WING_AREA * 2 * DEG) + 0 * sweep;
  }

  it('reaches the lifting line value a0 / (1 + a0 / (PI e AR)), not the two dimensional value', () => {
    const speed = 100;
    const a0 = sectionSlope(NACA_0011) * liftScale(speed);
    const theory = finiteSpanSlope(a0, ASPECT_RATIO, OSWALD);
    const measured = measureSlope(plain(), speed);
    expect(Math.abs(measured / theory - 1)).toBeLessThan(0.1);
    // The two dimensional slope is 37 percent higher. The test must not pass by
    // accident, so check that the model is nowhere near it.
    expect(measured).toBeLessThan(0.8 * a0);
  });

  it('reaches the swept value, which carries an extra cosine of the sweep angle', () => {
    const speed = 100;
    const swept = assemble([
      wing({
        strips: 10,
        sweep: SWEEP,
        taper: 1,
        airfoil: NACA_0011,
        stall: STALL_NACA_0011,
        aspectRatio: ASPECT_RATIO,
        oswald: OSWALD,
      }),
    ]);
    // Simple sweep theory cuts the section slope by the cosine of the sweep,
    // because the normal dynamic pressure falls by the square of the cosine and
    // the normal angle of attack grows by one over the cosine.
    const a0 = sectionSlope(NACA_0011) * liftScale(speed, SWEEP) * Math.cos(SWEEP);
    const theory = finiteSpanSlope(a0, ASPECT_RATIO, OSWALD);
    const measured = measureSlope(swept, speed, SWEEP);
    expect(Math.abs(measured / theory - 1)).toBeLessThan(0.1);
  });

  it('falls as the aspect ratio falls, as lifting line theory says', () => {
    const speed = 100;
    const slopes: number[] = [];
    for (const aspectRatio of [4, 7.21, 12]) {
      const span = Math.sqrt(aspectRatio * WING_AREA);
      const narrow = assemble([
        wing({
          strips: 10,
          sweep: 0,
          taper: 1,
          span,
          airfoil: NACA_0011,
          stall: STALL_NACA_0011,
          aspectRatio,
          oswald: OSWALD,
        }),
      ]);
      const low = settle(narrow, flightState(speed, 1 * DEG));
      const high = settle(narrow, flightState(speed, 3 * DEG));
      const q = 0.5 * RHO * speed * speed;
      slopes.push((high.lift - low.lift) / (q * WING_AREA * 2 * DEG));
    }
    expect(slopes[0]).toBeLessThan(slopes[1]);
    expect(slopes[1]).toBeLessThan(slopes[2]);
    const a0 = sectionSlope(NACA_0011) * liftScale(speed);
    expect(Math.abs(slopes[0] / finiteSpanSlope(a0, 4, OSWALD) - 1)).toBeLessThan(0.1);
    expect(Math.abs(slopes[2] / finiteSpanSlope(a0, 12, OSWALD) - 1)).toBeLessThan(0.1);
  });
});

describe('induced drag', () => {
  const plain = () =>
    assemble([
      wing({
        strips: 10,
        sweep: 0,
        taper: 1,
        airfoil: NACA_0011,
        stall: STALL_NACA_0011,
        aspectRatio: ASPECT_RATIO,
        oswald: OSWALD,
      }),
    ]);

  it('matches CL^2 / (PI e AR) at a moderate lift coefficient', () => {
    const speed = 100;
    const assembly = plain();
    const q = 0.5 * RHO * speed * speed;
    const zero = settle(assembly, flightState(speed, 0));
    const lifting = settle(assembly, flightState(speed, 7 * DEG));
    const cl = lifting.lift / (q * WING_AREA);
    const measured = (lifting.drag - zero.drag) / (q * WING_AREA);
    const theory = (cl * cl) / (Math.PI * OSWALD * ASPECT_RATIO);
    expect(cl).toBeGreaterThan(0.5);
    expect(Math.abs(measured / theory - 1)).toBeLessThan(0.15);
  });

  it('grows with the square of the lift coefficient', () => {
    const speed = 100;
    const assembly = plain();
    const q = 0.5 * RHO * speed * speed;
    const zero = settle(assembly, flightState(speed, 0));
    const low = settle(assembly, flightState(speed, 4 * DEG));
    const high = settle(assembly, flightState(speed, 8 * DEG));
    const clLow = low.lift / (q * WING_AREA);
    const clHigh = high.lift / (q * WING_AREA);
    const dragLow = low.drag - zero.drag;
    const dragHigh = high.drag - zero.drag;
    expect(dragHigh / dragLow).toBeCloseTo((clHigh * clHigh) / (clLow * clLow), 0);
  });

  it('vanishes at zero lift, so the drag at zero alpha is profile drag alone', () => {
    const speed = 100;
    const assembly = plain();
    const zero = settle(assembly, flightState(speed, 0));
    expect(zero.lift).toBeCloseTo(0, 6);
    expect(zero.drag).toBeGreaterThan(0);
  });
});

describe('the complete aircraft', () => {
  function me262(options: { slats?: boolean; tail?: boolean; body?: boolean } = {}): AeroAssembly {
    const parts: Part[] = [
      wing({
        strips: 8,
        sweep: SWEEP,
        twist: -2 * DEG,
        dihedral: 5 * DEG,
        x: 0.8,
        controlIndex: AILERON,
        controlFrom: 0.6,
        aspectRatio: ASPECT_RATIO,
        oswald: OSWALD,
        ...(options.slats === true ? { slatFrom: 0.5 } : {}),
      }),
    ];
    if (options.tail !== false) {
      parts.push(tailplane());
      parts.push(fin());
    }
    return assemble(parts, options.body === false ? [] : [fuselage()]);
  }

  it('makes no side force, no roll moment and no yaw moment in pure forward flight', () => {
    const assembly = me262();
    const w = settle(assembly, flightState(150, 4 * DEG));
    expect(w.sideForce).toBeCloseTo(0, 6);
    expect(w.force.y).toBeCloseTo(0, 6);
    expect(w.moment.x).toBeCloseTo(0, 6);
    expect(w.moment.z).toBeCloseTo(0, 6);
    // The symmetry must not come from a model that makes no force at all.
    expect(w.lift).toBeGreaterThan(1000);
  });

  it('makes lift up, which is a negative body z force, at a positive angle of attack', () => {
    const assembly = me262();
    const w = settle(assembly, flightState(150, 5 * DEG));
    expect(w.force.z).toBeLessThan(0);
    expect(w.lift).toBeGreaterThan(0);
    const down = settle(assembly, flightState(150, -5 * DEG));
    expect(down.force.z).toBeGreaterThan(0);
    expect(down.lift).toBeLessThan(0);
  });

  it('reports the free stream angles and the Mach number of the state', () => {
    const assembly = me262();
    const totals = settle(assembly, flightState(200, 6 * DEG, 3 * DEG), NEUTRAL, 5);
    expect(totals.alpha).toBeCloseTo(6 * DEG, 6);
    expect(totals.beta).toBeCloseTo(3 * DEG, 6);
    expect(totals.trueAirspeed).toBeCloseTo(200, 6);
    expect(totals.mach).toBeCloseTo(200 / SPEED_OF_SOUND, 6);
    expect(totals.dynamicPressure).toBeCloseTo(0.5 * RHO * 200 * 200, 3);
  });

  it('is statically stable in pitch, with a negative dCm/dalpha', () => {
    const assembly = me262();
    const speed = 150;
    const q = 0.5 * RHO * speed * speed;
    const low = settle(assembly, flightState(speed, 0)).moment.y;
    const high = settle(assembly, flightState(speed, 4 * DEG)).moment.y;
    const slope = (high - low) / (q * WING_AREA * MEAN_CHORD * 4 * DEG);
    expect(slope).toBeLessThan(0);
    // A real fighter sits near -0.5 to -1.5 per radian at this center of
    // gravity. A model far outside that band would fly nothing like one.
    expect(slope).toBeLessThan(-0.3);
    expect(slope).toBeGreaterThan(-3);
  });

  it('loses its pitch stability when the tail comes off, so the tail is what makes it', () => {
    const speed = 150;
    const q = 0.5 * RHO * speed * speed;
    const tailless = me262({ tail: false });
    const low = settle(tailless, flightState(speed, 0)).moment.y;
    const high = settle(tailless, flightState(speed, 4 * DEG)).moment.y;
    const slope = (high - low) / (q * WING_AREA * MEAN_CHORD * 4 * DEG);
    expect(slope).toBeGreaterThan(0);
  });

  it('weathercocks, so a positive sideslip makes a nose right yaw moment', () => {
    const assembly = me262();
    const w = settle(assembly, flightState(150, 2 * DEG, 6 * DEG));
    expect(w.moment.z).toBeGreaterThan(0);
  });

  it('has a stable dihedral effect, so a positive sideslip rolls it left', () => {
    const assembly = me262();
    const w = settle(assembly, flightState(150, 2 * DEG, 6 * DEG));
    expect(w.moment.x).toBeLessThan(0);
  });

  it('answers a positive aileron with a positive roll moment', () => {
    const assembly = me262();
    const controls = new Float64Array([12 * DEG, 0, 0]);
    const w = settle(assembly, flightState(150, 3 * DEG), controls);
    expect(w.moment.x).toBeGreaterThan(0);
  });

  it('answers a positive rudder with a yaw moment', () => {
    const assembly = me262();
    const controls = new Float64Array([0, 0, 10 * DEG]);
    const w = settle(assembly, flightState(150, 3 * DEG), controls);
    expect(Math.abs(w.moment.z)).toBeGreaterThan(100);
  });
});

describe('rate damping is emergent, out of omega cross r alone', () => {
  function me262(): AeroAssembly {
    return assemble(
      [
        wing({
          strips: 8,
          sweep: SWEEP,
          twist: -2 * DEG,
          dihedral: 5 * DEG,
          x: 0.8,
          aspectRatio: ASPECT_RATIO,
          oswald: OSWALD,
        }),
        tailplane(),
        fin(),
      ],
      [fuselage()],
    );
  }

  it('opposes a positive roll rate with no control input at all', () => {
    const assembly = me262();
    const still = settle(assembly, flightState(150, 4 * DEG)).moment.x;
    const rolling = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(1.0, 0, 0)),
    ).moment.x;
    expect(still).toBeCloseTo(0, 6);
    expect(rolling).toBeLessThan(0);
    const rollingBack = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(-1.0, 0, 0)),
    ).moment.x;
    expect(rollingBack).toBeGreaterThan(0);
    expect(rollingBack).toBeCloseTo(-rolling, 3);
  });

  it('grows the roll damping with the roll rate', () => {
    const assembly = me262();
    const slow = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(0.5, 0, 0)),
    ).moment.x;
    const fast = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(1.5, 0, 0)),
    ).moment.x;
    expect(fast).toBeLessThan(slow);
  });

  it('opposes a positive pitch rate with a nose down moment', () => {
    const assembly = me262();
    const still = settle(assembly, flightState(150, 4 * DEG)).moment.y;
    const pitching = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(0, 0.4, 0)),
    ).moment.y;
    expect(pitching).toBeLessThan(still);
    const pitchingDown = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(0, -0.4, 0)),
    ).moment.y;
    expect(pitchingDown).toBeGreaterThan(still);
  });

  it('opposes a positive yaw rate, because the fin beats the Munk moment', () => {
    const assembly = me262();
    const still = settle(assembly, flightState(150, 4 * DEG)).moment.z;
    const yawing = settle(
      assembly,
      flightState(150, 4 * DEG, 0, new Vector3(0, 0, 0.4)),
    ).moment.z;
    expect(still).toBeCloseTo(0, 6);
    expect(yawing).toBeLessThan(0);
  });
});

describe('the stall', () => {
  function stallWing(slats: boolean): AeroAssembly {
    return assemble([
      wing({
        strips: 8,
        sweep: SWEEP,
        twist: -2 * DEG,
        x: 0.8,
        controlIndex: AILERON,
        controlFrom: 0.6,
        aspectRatio: ASPECT_RATIO,
        oswald: OSWALD,
        ...(slats ? { slatFrom: 0.5 } : {}),
      }),
    ]);
  }

  /** The lift of one half of the wing, in newtons. */
  function halfLift(assembly: AeroAssembly, side: 'r' | 'l'): number {
    let sum = 0;
    for (let i = 0; i < assembly.surfaces.length; i++) {
      if (assembly.surfaces[i].def.name.includes(`-${side}`)) {
        sum += -assembly.surfaces[i].result.force.z;
      }
    }
    return sum;
  }

  it('makes one wing carry less lift than the other in a sideslip near the stall', () => {
    const assembly = stallWing(false);
    const speed = 70;
    settle(assembly, flightState(speed, 17 * DEG, 8 * DEG), NEUTRAL, 600);
    const right = halfLift(assembly, 'r');
    const left = halfLift(assembly, 'l');
    // A positive sideslip puts the air on the right, so the right half is
    // windward. It meets less effective sweep, so its section angle is lower and
    // it holds its flow while the left half breaks.
    expect(right).toBeGreaterThan(left);
    expect((right - left) / right).toBeGreaterThan(0.05);
  });

  it('rolls off, and the roll off is stronger at the stall than below it', () => {
    const assembly = stallWing(false);
    const speed = 70;
    const q = 0.5 * RHO * speed * speed;
    const scale = q * WING_AREA * SPAN;

    const low = settle(assembly, flightState(speed, 4 * DEG, 8 * DEG), NEUTRAL, 600);
    const lowRoll = Math.abs(low.moment.x) / scale;

    const stalled = settle(assembly, flightState(speed, 17 * DEG, 8 * DEG), NEUTRAL, 600);
    const stalledRoll = Math.abs(stalled.moment.x) / scale;

    expect(lowRoll).toBeGreaterThan(0);
    // The break of one wing before the other is the roll off. It is emergent:
    // no code puts a roll moment in at the stall.
    expect(stalledRoll).toBeGreaterThan(1.5 * lowRoll);
    expect(stalled.moment.x).toBeLessThan(0);
  });

  it('breaks the root before the tip, because the wing carries washout', () => {
    const assembly = stallWing(false);
    settle(assembly, flightState(70, 15 * DEG), NEUTRAL, 600);
    const root = assembly.surfaces[0].result.separation;
    const tip = assembly.surfaces[7].result.separation;
    expect(root).toBeLessThan(tip);
  });

  it('keeps the aileron alive past the root stall when the wing carries slats', () => {
    // The strip that decides this is the one with the aileron DOWN, on the left
    // half. A down aileron raises the local angle of attack, so that strip is
    // the first one to break, and when it breaks the aileron reverses. Index 15
    // is the outermost left strip and index 0 is the innermost right strip.
    const speed = 70;
    const alpha = 21 * DEG;
    const controls = new Float64Array([14 * DEG, 0, 0]);

    const withSlats = stallWing(true);
    const withoutSlats = stallWing(false);
    const slatted = settle(withSlats, flightState(speed, alpha), controls, 600);
    const plain = settle(withoutSlats, flightState(speed, alpha), controls, 600);

    // The root has really broken in both wings, so this is a real post stall
    // condition and not a test that passes below the stall.
    expect(withSlats.surfaces[0].result.separation).toBeLessThan(0.4);
    expect(withoutSlats.surfaces[0].result.separation).toBeLessThan(0.4);

    // The slat has opened on the down aileron strip and holds its flow on.
    expect(withSlats.surfaces[15].result.slatOpen).toBe(true);
    expect(withoutSlats.surfaces[15].result.slatOpen).toBe(false);
    expect(withSlats.surfaces[15].result.separation).toBeGreaterThan(
      2 * withoutSlats.surfaces[15].result.separation,
    );

    // The plain wing has lost the argument: its aileron REVERSES, because the
    // down aileron strip is stalled and the extra deflection only adds drag and
    // takes lift away. The slatted wing still rolls the way the pilot asked.
    // This is the whole reason the Me-262 carried slats.
    expect(plain.moment.x).toBeLessThan(0);
    expect(slatted.moment.x).toBeGreaterThan(0);
    expect(slatted.moment.x).toBeGreaterThan(2 * Math.abs(plain.moment.x));
  });

  it('loses lift past the stall, so the lift curve turns over', () => {
    const assembly = stallWing(false);
    const speed = 70;
    const q = 0.5 * RHO * speed * speed;
    let peak = 0;
    let peakAngle = 0;
    for (let deg = 2; deg <= 26; deg += 1) {
      const totals = settle(assembly, flightState(speed, deg * DEG), NEUTRAL, 300);
      const cl = totals.lift / (q * WING_AREA);
      if (cl > peak) {
        peak = cl;
        peakAngle = deg;
      }
    }
    // A wing of this section and this washout peaks near 1.2 to 1.5, between 13
    // and 20 degrees.
    expect(peak).toBeGreaterThan(1.0);
    expect(peak).toBeLessThan(1.7);
    expect(peakAngle).toBeGreaterThan(11);
    expect(peakAngle).toBeLessThan(22);
  });
});

describe('the fuselage inside the assembly', () => {
  it('is destabilizing on its own, with a nose up moment at a positive alpha', () => {
    // A bare fuselage, with no lifting surface at all. The group list is empty.
    const assembly = createAssembly([], [fuselage({ position: new Vector3(0, 0, 0) })], []);
    const w = settle(assembly, flightState(150, 6 * DEG), NEUTRAL, 5);
    expect(w.moment.y).toBeGreaterThan(0);
  });

  it('makes the wing and fuselage pair less stable than the wing on its own', () => {
    const speed = 150;
    const q = 0.5 * RHO * speed * speed;
    const wingOnly = assemble([wing({ strips: 8, sweep: SWEEP, x: 0.8 })]);
    const withBody = assemble(
      [wing({ strips: 8, sweep: SWEEP, x: 0.8 })],
      [fuselage({ position: new Vector3(0, 0, 0) })],
    );
    function slope(assembly: AeroAssembly): number {
      const low = settle(assembly, flightState(speed, 0)).moment.y;
      const high = settle(assembly, flightState(speed, 4 * DEG)).moment.y;
      return (high - low) / (q * WING_AREA * MEAN_CHORD * 4 * DEG);
    }
    expect(slope(withBody)).toBeGreaterThan(slope(wingOnly));
  });
});

describe('the tail inside the assembly', () => {
  it('makes a nose down restoring moment at a positive angle of attack', () => {
    const assembly = assemble([tailplane()]);
    const w = settle(assembly, flightState(150, 5 * DEG));
    expect(w.force.z).toBeLessThan(0);
    expect(w.moment.y).toBeLessThan(0);
  });

  it('makes a nose up moment at a negative angle of attack', () => {
    const assembly = assemble([tailplane()]);
    const w = settle(assembly, flightState(150, -5 * DEG));
    expect(w.moment.y).toBeGreaterThan(0);
  });
});

describe('the induced angle solve', () => {
  it('gives each group its own angle, so the tail does not read the wing lift', () => {
    const assembly = assemble([
      wing({ strips: 6, sweep: 0, taper: 1, airfoil: NACA_0011, stall: STALL_NACA_0011 }),
      tailplane(),
    ]);
    settle(assembly, flightState(150, 5 * DEG));
    // Both groups run below the free stream angle, because both carry lift and
    // both make their own downwash. The tail has the lower aspect ratio, so it
    // makes MORE downwash and sits further below. That is the whole point of
    // solving the angle per parent surface: one shared angle could not do it.
    const wingAlpha = assembly.surfaces[0].result.alpha;
    const tailAlpha = assembly.surfaces[12].result.alpha;
    expect(wingAlpha).toBeLessThan(5 * DEG);
    expect(tailAlpha).toBeLessThan(5 * DEG);
    expect(tailAlpha).toBeLessThan(wingAlpha);
    // The wing group and the tail group really did get different angles.
    expect(Math.abs(tailAlpha - wingAlpha)).toBeGreaterThan(0.2 * DEG);
  });

  it('gives a strip in no group its own finite span correction from its def', () => {
    const single = wing({ strips: 1, sweep: 0, taper: 1, airfoil: NACA_0011, stall: STALL_NACA_0011 });
    const assembly = createAssembly(single.surfaces, [], []);
    settle(assembly, flightState(150, 5 * DEG));
    expect(assembly.surfaces[0].result.alpha).toBeLessThan(5 * DEG);
    expect(assembly.surfaces[0].result.alpha).toBeGreaterThan(0);
  });

  it('rejects a group that names a surface that does not exist', () => {
    const parts = wing({ strips: 2 });
    expect(() =>
      createAssembly(parts.surfaces, [], [{ ...parts.group, surfaceIndices: [99] }]),
    ).toThrow();
  });
});

describe('the debug sample', () => {
  it('reports one sample per strip, with the position, the force and the stall angle', () => {
    const assembly = assemble([wing({ strips: 4, slatFrom: 0.5 })]);
    settle(assembly, flightState(150, 5 * DEG));
    const samples = assembly.sampleForDebug();
    expect(samples.length).toBe(assembly.surfaces.length);
    for (let i = 0; i < samples.length; i++) {
      expect(samples[i].name).toBe(assembly.surfaces[i].def.name);
      expect(samples[i].position.equals(assembly.surfaces[i].def.position)).toBe(true);
      expect(samples[i].force.equals(assembly.surfaces[i].result.force)).toBe(true);
      expect(samples[i].alpha).toBe(assembly.surfaces[i].result.alpha);
      expect(samples[i].stallAlpha).toBeGreaterThan(0);
    }
  });

  it('raises the stall angle of a strip whose slat has opened', () => {
    const assembly = assemble([wing({ strips: 4, slatFrom: 0.5 })]);
    settle(assembly, flightState(70, 16 * DEG), NEUTRAL, 600);
    const samples = assembly.sampleForDebug();
    const root = samples[0];
    const tip = samples[3];
    expect(assembly.surfaces[3].result.slatOpen).toBe(true);
    expect(tip.stallAlpha).toBeGreaterThan(root.stallAlpha);
  });
});

describe('evaluate allocates nothing', () => {
  it('returns the same objects and the same vectors on every one of 1000 calls', () => {
    // Verified by construction as well: no `new`, no object literal and no array
    // literal appears in evaluate, evaluateSurface or evaluateBody. Every
    // scratch vector lives in module scope or in the closure of createAssembly,
    // and every one of them is built before the first call. This test holds that
    // property, because a fresh allocation would break the identity of the
    // objects the caller reads.
    const assembly = assemble(
      [wing({ strips: 8, sweep: SWEEP, twist: -2 * DEG }), tailplane(), fin()],
      [fuselage()],
    );
    const state = flightState(150, 4 * DEG, 2 * DEG, new Vector3(0.2, 0.1, 0.05));
    const out = createWrench();

    clearWrench(out);
    const first = assembly.evaluate(state, ZERO, NEUTRAL, STEP, out);
    const perSurface = first.perSurface;
    const forces = perSurface.map((r) => r.force);
    const moments = perSurface.map((r) => r.moment);
    const samples = assembly.sampleForDebug();
    const samplePositions = samples.map((s) => s.position);

    for (let i = 0; i < 1000; i++) {
      clearWrench(out);
      const totals = assembly.evaluate(state, ZERO, NEUTRAL, STEP, out);
      expect(totals).toBe(first);
      expect(totals.perSurface).toBe(perSurface);
    }
    expect(assembly.sampleForDebug()).toBe(samples);
    for (let i = 0; i < perSurface.length; i++) {
      expect(perSurface[i].force).toBe(forces[i]);
      expect(perSurface[i].moment).toBe(moments[i]);
      expect(samples[i].position).toBe(samplePositions[i]);
    }
    expect(Number.isFinite(out.force.z)).toBe(true);
    expect(Number.isFinite(out.moment.y)).toBe(true);
  });

  it('adds into the wrench the caller passes, so gravity and thrust can share it', () => {
    const assembly = assemble([wing({ strips: 4 })]);
    const state = flightState(150, 4 * DEG);
    const out = createWrench();
    // dt of zero freezes the separation lag, so the two calls see exactly the
    // same state and their forces must add exactly.
    clearWrench(out);
    assembly.evaluate(state, ZERO, NEUTRAL, 0, out);
    const once = out.force.z;
    assembly.evaluate(state, ZERO, NEUTRAL, 0, out);
    expect(out.force.z).toBeCloseTo(2 * once, 6);
  });
});
