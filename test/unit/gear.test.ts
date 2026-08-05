import { describe, expect, it } from 'vitest';
import { Matrix3, Vector3 } from 'three';

import { G0, kmhToMs, toRad } from '@/math/units';
import {
  BRAKE_FADE_FULL_TEMPERATURE,
  BRAKE_FADE_START_TEMPERATURE,
  MAX_BRAKE_TORQUE,
  ME262_NOSE_LOAD_FRACTION,
  ME262_STATIC_CG_HEIGHT,
  NOSE_STEER_LIMIT,
  TIRE_PEAK_SLIP_ANGLE,
  TIRE_PEAK_SLIP_RATIO,
  brakeFade,
  createMe262Gear,
  me262GearLegs,
  tireLateralMu,
  tireLongitudinalMu,
} from '@/physics/gear';
import type { LandingGear } from '@/physics/gear';
import {
  clearWrench,
  createMassProperties,
  createState,
  createWrench,
  worldToBody,
} from '@/physics/rigidbody';
import type { RigidBodyState, Wrench } from '@/physics/rigidbody';

const DT = 1 / 240; // s, the fixed physics step

/** Loaded mass of the Me 262 A-1a. CONVENTIONS section 8, firm. */
const MASS = 6396; // kg
const WEIGHT = MASS * G0; // N

const LEGS = me262GearLegs();
const NOSE = 0;
const MAIN_LEFT = 1;
const MAIN_RIGHT = 2;

/**
 * Inertia tensor of the loaded aircraft, in body axes about the center of
 * gravity. The values come from src/aircraft/me262/mass.ts at full fuel, rounded
 * to four figures. The test copies them instead of importing them, so a change
 * in the mass model cannot silently change what this file measures.
 */
const INERTIA = new Matrix3().set(14660, 0, -550, 0, 23214, 0, -550, 0, 34950);
const MASS_PROPERTIES = createMassProperties(MASS, INERTIA);

/** Depth of the unloaded contact patch of one leg below the center of gravity. */
function extendedDepth(index: number): number {
  const def = LEGS[index];
  return def.position.z + def.restLength + def.wheelRadius;
}

interface RunOptions {
  /** Height of the center of gravity above the ground at the start, m. */
  height: number;
  /** Rate of descent at the start, m/s. Positive falls. */
  sink?: number;
  /** Ground speed along body x at the start, m/s. */
  forward?: number;
  seconds: number;
  gearPosition?: number;
  steering?: number;
  brakeLeft?: number;
  brakeRight?: number;
  /** A steady force along body x, N. It stands for the thrust of the engines. */
  thrust?: number;
  /** True holds the state still, so only the gear state moves. */
  frozen?: boolean;
}

interface RunResult {
  gear: LandingGear;
  state: RigidBodyState;
  wrench: Wrench;
  /** Height of the center of gravity above the ground at each step, m. */
  height: number[];
  peakLoad: number[];
  peakStroke: number[];
}

/**
 * Flies the aircraft with gravity and the gear only.
 *
 * The integrator is semi implicit Euler, not the RK4 of src/physics/rigidbody.ts.
 * The gear holds its own wheel spin and brake state, so it must see each step one
 * time. The step is the same 1/240 s the simulator runs, and the test only asks
 * whether the aircraft settles, bounces or bottoms, so the order of the
 * integrator does not change the answer.
 *
 * There is no aerodynamic force at all. That is the hard case for the gear: a
 * real aircraft still carries most of its weight on the wing when the wheels
 * touch, so the loads below are higher than a real touchdown makes.
 */
function run(options: RunOptions): RunResult {
  const gear = createMe262Gear();
  const state = createState();
  const wrench = createWrench();
  const gravityWorld = new Vector3(0, 0, WEIGHT);
  const gravityBody = new Vector3();
  const acceleration = new Vector3();
  const angularAcceleration = new Vector3();
  const inertiaTimesOmega = new Vector3();
  const gyroscopic = new Vector3();

  state.position.set(0, 0, -options.height);
  state.velocity.set(options.forward ?? 0, 0, options.sink ?? 0);

  const height: number[] = [];
  const peakLoad = [0, 0, 0];
  const peakStroke = [0, 0, 0];
  const steps = Math.round(options.seconds / DT);

  for (let i = 0; i < steps; i++) {
    clearWrench(wrench);
    gear.update(
      state,
      options.gearPosition ?? 1,
      options.steering ?? 0,
      options.brakeLeft ?? 0,
      options.brakeRight ?? 0,
      DT,
      wrench,
    );
    for (let k = 0; k < 3; k++) {
      peakLoad[k] = Math.max(peakLoad[k], gear.legs[k].load);
      peakStroke[k] = Math.max(peakStroke[k], gear.legs[k].compression);
    }
    if (!options.frozen) {
      worldToBody(state.orientation, gravityWorld, gravityBody);
      wrench.force.add(gravityBody);
      wrench.force.x += options.thrust ?? 0;
      acceleration.copy(wrench.force).applyQuaternion(state.orientation).multiplyScalar(1 / MASS);
      state.velocity.addScaledVector(acceleration, DT);
      state.position.addScaledVector(state.velocity, DT);

      inertiaTimesOmega.copy(state.angularVelocity).applyMatrix3(MASS_PROPERTIES.inertia);
      gyroscopic.crossVectors(state.angularVelocity, inertiaTimesOmega);
      angularAcceleration
        .copy(wrench.moment)
        .sub(gyroscopic)
        .applyMatrix3(MASS_PROPERTIES.inverseInertia);
      state.angularVelocity.addScaledVector(angularAcceleration, DT);

      const q = state.orientation;
      const ox = state.angularVelocity.x;
      const oy = state.angularVelocity.y;
      const oz = state.angularVelocity.z;
      q.set(
        q.x + 0.5 * (q.w * ox + q.y * oz - q.z * oy) * DT,
        q.y + 0.5 * (q.w * oy - q.x * oz + q.z * ox) * DT,
        q.z + 0.5 * (q.w * oz + q.x * oy - q.y * ox) * DT,
        q.w + 0.5 * (-q.x * ox - q.y * oy - q.z * oz) * DT,
      ).normalize();
    }
    height.push(-state.position.z);
  }
  return { gear, state, wrench, height, peakLoad, peakStroke };
}

/** Reads the gear one time at a fixed state. */
function probe(
  configure: (state: RigidBodyState) => void,
  steering = 0,
  brakeLeft = 0,
  brakeRight = 0,
  steps = 1,
): { gear: LandingGear; wrench: Wrench; state: RigidBodyState } {
  const gear = createMe262Gear();
  const state = createState();
  const wrench = createWrench();
  state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
  configure(state);
  for (let i = 0; i < steps; i++) {
    clearWrench(wrench);
    gear.update(state, 1, steering, brakeLeft, brakeRight, DT, wrench);
  }
  return { gear, wrench, state };
}

// ---------------------------------------------------------------------------
// The layout
// ---------------------------------------------------------------------------

describe('Me 262 gear layout', () => {
  it('places the wheels where the render model draws them', () => {
    // src/render/models/me262.ts puts its ground line 1.33 m below the fuselage
    // reference plane, and src/aircraft/me262/mass.ts puts the center of gravity
    // 0.1333 m below that plane. The contact patch therefore sits 1.1967 m below
    // the center of gravity when the aircraft stands at rest.
    expect(ME262_STATIC_CG_HEIGHT).toBeCloseTo(1.1967, 4);
    // Nose wheel 3.58 m forward of the center of gravity, main wheels 0.32 m
    // aft of it and 1.18 m out from the plane of symmetry.
    expect(LEGS[NOSE].position.x).toBeCloseTo(3.58, 6);
    expect(LEGS[NOSE].position.y).toBe(0);
    expect(LEGS[MAIN_LEFT].position.x).toBeCloseTo(-0.32, 6);
    expect(LEGS[MAIN_LEFT].position.y).toBeCloseTo(-1.18, 6);
    expect(LEGS[MAIN_RIGHT].position.y).toBeCloseTo(1.18, 6);
    expect(LEGS[NOSE].wheelRadius).toBeCloseTo(0.33, 6);
    expect(LEGS[MAIN_LEFT].wheelRadius).toBeCloseTo(0.42, 6);
  });

  it('brakes the main wheels only and steers the nose wheel only', () => {
    expect(LEGS[NOSE].braked).toBe(false);
    expect(LEGS[NOSE].steerable).toBe(true);
    expect(LEGS[MAIN_LEFT].braked).toBe(true);
    expect(LEGS[MAIN_RIGHT].braked).toBe(true);
    expect(LEGS[MAIN_LEFT].steerable).toBe(false);
    expect(LEGS[MAIN_RIGHT].steerable).toBe(false);
  });
});

// ---------------------------------------------------------------------------
// The strut
// ---------------------------------------------------------------------------

describe('oleo strut', () => {
  it('stiffens as it compresses, so it is not a linear spring', () => {
    // The gas force at the end of the travel is (1 - 0.9) ^ -1.3 = 19.9 times
    // the force at full extension. A linear spring that carried the aircraft at
    // rest would give only 1 / 0.55 = 1.8 times the static force at the stop.
    const gear = createMe262Gear();
    const def = LEGS[MAIN_LEFT];
    const state = createState();
    const wrench = createWrench();
    const sample = (stroke: number): number => {
      // Sink the aircraft until the strut reaches the wanted stroke. The tire
      // adds its own deflection, so the search runs on the height.
      let low = 0;
      let high = 1;
      for (let i = 0; i < 40; i++) {
        const guess = 0.5 * (low + high);
        state.position.set(0, 0, -(extendedDepth(MAIN_LEFT) - guess));
        clearWrench(wrench);
        gear.update(state, 1, 0, 0, 0, DT, wrench);
        if (gear.legs[MAIN_LEFT].compression > stroke) {
          high = guess;
        } else {
          low = guess;
        }
      }
      return gear.legs[MAIN_LEFT].load;
    };
    const atQuarter = sample(0.25 * def.maxTravel);
    const atHalf = sample(0.5 * def.maxTravel);
    const atThreeQuarters = sample(0.75 * def.maxTravel);
    const atFull = sample(0.99 * def.maxTravel);
    // A LINEAR spring gives the same force step over every equal step of stroke.
    // The gas spring gives a bigger step each time, which is what stiffening
    // means. The third quarter of the travel adds more than twice what the
    // second quarter added.
    const secondQuarter = atHalf - atQuarter;
    const thirdQuarter = atThreeQuarters - atHalf;
    const lastQuarter = atFull - atThreeQuarters;
    expect(thirdQuarter / secondQuarter).toBeGreaterThan(2);
    expect(lastQuarter / thirdQuarter).toBeGreaterThan(2);
    // The strut reaches nearly twenty times its extended force at the stop.
    expect(atFull / def.springGas).toBeGreaterThan(15);
    expect(atFull / atHalf).toBeGreaterThan(6);
  });

  it('damps compression far harder than rebound, and neither is zero', () => {
    for (const def of LEGS) {
      expect(def.dampingRebound).toBeGreaterThan(0);
      expect(def.dampingCompression).toBeGreaterThan(2 * def.dampingRebound);
    }
  });
});

// ---------------------------------------------------------------------------
// Static equilibrium. This is the test that matters most.
// ---------------------------------------------------------------------------

describe('static equilibrium at 6396 kg', () => {
  const result = run({ height: ME262_STATIC_CG_HEIGHT + 0.001, seconds: 40 });
  const legs = result.gear.legs;
  const total = legs[NOSE].load + legs[MAIN_LEFT].load + legs[MAIN_RIGHT].load;

  it('carries the whole weight of the aircraft and nothing more', () => {
    // A spring rate that is too soft sinks the aircraft through the runway. One
    // that is too stiff launches it off. Both show up here first.
    expect(total / WEIGHT).toBeCloseTo(1, 3);
    // `run` adds gravity into the same wrench, so the sum of the two must come
    // out at zero. The aircraft then stands still.
    expect(Math.abs(result.wrench.force.z)).toBeLessThan(0.002 * WEIGHT);
    expect(Math.abs(result.wrench.moment.y)).toBeLessThan(0.002 * WEIGHT);
    expect(Math.abs(result.state.velocity.z)).toBeLessThan(1e-3);
  });

  it('puts about 8 percent of the weight on the nose leg', () => {
    // The moment balance about the center of gravity gives 0.32 / 3.90 = 8.2
    // percent, which is what bead b17 reports.
    expect(ME262_NOSE_LOAD_FRACTION).toBeCloseTo(0.0821, 4);
    expect(legs[NOSE].load / total).toBeGreaterThan(0.075);
    expect(legs[NOSE].load / total).toBeLessThan(0.09);
    // The two main legs share the rest evenly, because the aircraft is level.
    expect(legs[MAIN_LEFT].load).toBeCloseTo(legs[MAIN_RIGHT].load, 3);
  });

  it('stands on a sensible fraction of the available travel', () => {
    for (let i = 0; i < 3; i++) {
      const fraction = legs[i].compression / LEGS[i].maxTravel;
      // Low enough that a taxi bump has stroke to work in, high enough that the
      // stiff end of the gas curve is still there for a landing.
      expect(fraction).toBeGreaterThan(0.35);
      expect(fraction).toBeLessThan(0.7);
    }
    expect(legs[MAIN_LEFT].compression).toBeCloseTo(0.154, 3);
  });

  it('parks the center of gravity on the ground line of the render model', () => {
    expect(-result.state.position.z).toBeCloseTo(ME262_STATIC_CG_HEIGHT, 3);
  });
});

// ---------------------------------------------------------------------------
// Touchdown
// ---------------------------------------------------------------------------

describe('touchdown', () => {
  /** Drops the aircraft level, from the height where the main tires just touch. */
  function drop(sink: number, seconds = 2.5): RunResult {
    return run({ height: extendedDepth(MAIN_LEFT) + 0.002, sink, seconds });
  }

  it('does not bottom the strut at a 3 m/s sink rate', () => {
    const result = drop(3);
    expect(result.peakStroke[MAIN_LEFT]).toBeLessThan(LEGS[MAIN_LEFT].maxTravel);
    expect(result.peakStroke[NOSE]).toBeLessThan(LEGS[NOSE].maxTravel);
    // The strut uses most of its travel, which is where a landing gear should
    // work. A strut that used a quarter of its stroke would be far too stiff.
    expect(result.peakStroke[MAIN_LEFT] / LEGS[MAIN_LEFT].maxTravel).toBeGreaterThan(0.6);
    // Peak load near 2.7 times the weight, with no lift on the wing at all.
    expect(result.peakLoad[MAIN_LEFT]).toBeLessThan(2 * WEIGHT);
  });

  it('bottoms the main strut between 6.4 and 6.5 m/s', () => {
    // The sink rate that bottoms the strut. Below it the gas spring and the
    // damper hold the stroke inside the travel. Above it the aircraft hits the
    // hard stop, the load trebles and the tires burst.
    expect(drop(6.4).peakStroke[MAIN_LEFT]).toBeLessThan(LEGS[MAIN_LEFT].maxTravel);
    expect(drop(6.5).peakStroke[MAIN_LEFT]).toBeGreaterThan(LEGS[MAIN_LEFT].maxTravel);
  });

  it('does not pogo, so each bounce is far smaller than the one before', () => {
    const result = drop(3, 8);
    const peaks: number[] = [];
    for (let i = 1; i < result.height.length - 1; i++) {
      const h = result.height[i];
      if (h > result.height[i - 1] && h >= result.height[i + 1]) {
        peaks.push(h - ME262_STATIC_CG_HEIGHT);
      }
    }
    expect(peaks.length).toBeGreaterThan(1);
    // The aircraft rebounds once and the next rebound is more than five times
    // smaller. With no rebound damping at all this ratio would be near one.
    expect(peaks[0]).toBeGreaterThan(0.05);
    expect(peaks[1]).toBeLessThan(0.2 * peaks[0]);
    expect(peaks[2]).toBeLessThan(0.2 * peaks[1]);
    // It settles back to the height it parks at.
    expect(result.height[result.height.length - 1]).toBeCloseTo(ME262_STATIC_CG_HEIGHT, 3);
  });

  it('bursts a tire on a hard arrival and leaves a normal landing alone', () => {
    const normal = drop(3);
    expect(normal.gear.legs.some((leg) => leg.burst)).toBe(false);
    expect(normal.peakLoad[MAIN_LEFT]).toBeLessThan(LEGS[MAIN_LEFT].burstLoad);

    const hard = drop(7);
    expect(hard.gear.legs[MAIN_LEFT].burst).toBe(true);
    expect(hard.gear.legs[MAIN_RIGHT].burst).toBe(true);
    expect(hard.peakLoad[MAIN_LEFT]).toBeGreaterThan(LEGS[MAIN_LEFT].burstLoad);
  });

  it('settles on the low wheel first when one wing is down', () => {
    // The gear positions are body frame offsets, so the attitude decides where
    // each wheel is. Four degrees of right bank has to load the right leg more.
    const { gear, wrench } = probe((state) => {
      state.orientation.setFromAxisAngle(new Vector3(1, 0, 0), toRad(4));
    });
    expect(gear.legs[MAIN_RIGHT].load).toBeGreaterThan(2 * gear.legs[MAIN_LEFT].load);
    // The roll moment pushes back toward level, so it is negative.
    expect(wrench.moment.x).toBeLessThan(0);
  });

  it('lifts the nose wheel clear during the rotation', () => {
    const { gear } = probe((state) => {
      state.orientation.setFromAxisAngle(new Vector3(0, 1, 0), toRad(6));
    });
    expect(gear.legs[NOSE].onGround).toBe(false);
    expect(gear.legs[NOSE].load).toBe(0);
    expect(gear.legs[MAIN_LEFT].onGround).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// The tire
// ---------------------------------------------------------------------------

describe('tire friction', () => {
  it('peaks at an intermediate slip and grips less when the wheel locks', () => {
    const peak = tireLongitudinalMu(TIRE_PEAK_SLIP_RATIO);
    const locked = Math.abs(tireLongitudinalMu(-1));
    const small = tireLongitudinalMu(0.2 * TIRE_PEAK_SLIP_RATIO);
    // Rises to the peak.
    expect(small).toBeLessThan(peak);
    // Falls after it. A locked wheel is the whole reason the curve has to fall.
    expect(locked).toBeLessThan(peak);
    expect(locked / peak).toBeLessThan(0.9);
    expect(locked / peak).toBeGreaterThan(0.6);
    // Nothing else on the curve beats the peak.
    for (let slip = -2; slip <= 2; slip += 0.01) {
      expect(Math.abs(tireLongitudinalMu(slip))).toBeLessThan(peak + 1e-9);
    }
  });

  it('gives the lateral force its own curve, with its own peak', () => {
    const peak = tireLateralMu(TIRE_PEAK_SLIP_ANGLE);
    expect(peak).toBeGreaterThan(0.5);
    expect(peak).toBeLessThan(tireLongitudinalMu(TIRE_PEAK_SLIP_RATIO));
    expect(tireLateralMu(0.2 * TIRE_PEAK_SLIP_ANGLE)).toBeLessThan(peak);
    // A tire sliding straight sideways grips less than one at its peak angle.
    expect(tireLateralMu(Math.PI / 2)).toBeLessThan(peak);
    // The curve is odd, so it always opposes the slide.
    expect(tireLateralMu(-TIRE_PEAK_SLIP_ANGLE)).toBeCloseTo(-peak, 12);
  });

  it('skids on a full brake at a light wheel load and takes longer to stop', () => {
    // The tire curve exists to make this true. A firm application holds the tire
    // near the peak of its curve. A full application asks for more than the tire
    // can give, the wheel stops turning, and the tire slides at the FALLING end
    // of the curve. The aircraft then needs more runway, not less.
    //
    // THE RUN CARRIES 40 PERCENT OF THE WEIGHT ON THE WHEELS. Bead b33 took
    // MAX_BRAKE_TORQUE down to the value the run up of the pilot notes measures,
    // so the brake is now WEAKER than the tire at the full weight and a full
    // pedal cannot lock a wheel there. It can lock one early in a landing roll,
    // where the wing still carries most of the weight and the tire has little
    // load to grip with. That is the condition below.
    //
    // THE RUN STARTS AT 40 m/s SO THAT THE PACK STAYS COLD. This test measures
    // the TIRE. A stop from 60 m/s with no wing lift at all puts 5.7 MJ into
    // each pack, which is past BRAKE_FADE_FULL of src/physics/gear.ts, and a
    // pack that has lost half its torque asks the tire for less than a locked
    // tire gives. The brake would then win the comparison for a reason that has
    // nothing to do with the tire curve. The test below this one measures that
    // second effect on its own.
    // The share of the weight the WHEELS carry. The wing carries the rest.
    const wheelShare = 0.4;
    function stop(command: number): { distance: number; worstSlip: number } {
      const gear = createMe262Gear();
      const state = createState();
      const wrench = createWrench();
      const gravityWorld = new Vector3(0, 0, wheelShare * WEIGHT);
      const gravityBody = new Vector3();
      const acceleration = new Vector3();
      state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
      state.velocity.set(40, 0, 0);
      let start = 0;
      let worstSlip = 0;
      for (let i = 0; i < 240 * 60 && state.velocity.x > 0.5; i++) {
        // Roll for half a second first, so the wheels are turning when the
        // brakes go on. A wheel that is already stopped locks whatever the
        // pilot does.
        const braking = i >= 120;
        if (i === 120) {
          start = state.position.x;
        }
        clearWrench(wrench);
        gear.update(state, 1, 0, braking ? command : 0, braking ? command : 0, DT, wrench);
        if (braking) {
          worstSlip = Math.min(worstSlip, gear.legs[MAIN_LEFT].slipRatio);
        }
        worldToBody(state.orientation, gravityWorld, gravityBody);
        wrench.force.add(gravityBody);
        acceleration.copy(wrench.force).applyQuaternion(state.orientation).multiplyScalar(1 / MASS);
        state.velocity.addScaledVector(acceleration, DT);
        state.position.x += state.velocity.x * DT;
      }
      return { distance: state.position.x - start, worstSlip };
    }
    // THE FIRM PEDAL FOLLOWS THE BRAKE CONSTANT AND THE WHEEL LOAD. Below is
    // the pedal that would put the tire exactly at the peak of its curve at this
    // load, and the firm application sits one percent under it. A hard coded
    // fraction went stale the last time the constant moved.
    const mainStaticLoad = 0.5 * (1 - ME262_NOSE_LOAD_FRACTION) * wheelShare * WEIGHT;
    const lockPedal =
      (tireLongitudinalMu(TIRE_PEAK_SLIP_RATIO) * mainStaticLoad * LEGS[MAIN_LEFT].wheelRadius) /
      MAX_BRAKE_TORQUE;
    expect(lockPedal).toBeLessThan(1);
    const firm = stop(0.99 * lockPedal);
    const full = stop(1);
    // The firm application keeps the wheel turning near the peak of the curve.
    expect(firm.worstSlip).toBeGreaterThan(-0.5);
    // The full application locks it.
    expect(full.worstSlip).toBeLessThan(-0.9);
    // And it costs runway.
    expect(full.distance).toBeGreaterThan(1.1 * firm.distance);
  });
});

// ---------------------------------------------------------------------------
// Steering and braking
// ---------------------------------------------------------------------------

describe('ground steering', () => {
  it('turns the nose wheel no further than 30 degrees', () => {
    expect(NOSE_STEER_LIMIT).toBeCloseTo(toRad(30), 12);
    const right = probe((state) => state.velocity.set(15, 0, 0), 1);
    // The wheel plane sits 30 degrees right of the path, so the contact patch
    // slides 30 degrees to the left of the wheel.
    expect(right.gear.legs[NOSE].slipAngle).toBeCloseTo(-NOSE_STEER_LIMIT, 4);
    // A command past the stop changes nothing.
    const past = probe((state) => state.velocity.set(15, 0, 0), 4);
    expect(past.gear.legs[NOSE].slipAngle).toBeCloseTo(right.gear.legs[NOSE].slipAngle, 9);
  });

  it('yaws the nose toward the side the nose wheel points', () => {
    const right = probe((state) => state.velocity.set(15, 0, 0), 1);
    const left = probe((state) => state.velocity.set(15, 0, 0), -1);
    // A positive yaw moment about body z moves the nose right.
    expect(right.wrench.moment.z).toBeGreaterThan(1000);
    expect(left.wrench.moment.z).toBeLessThan(-1000);
    expect(right.wrench.moment.z).toBeCloseTo(-left.wrench.moment.z, 6);
    // The side force acts at the nose wheel, so it points right as well.
    expect(right.wrench.force.y).toBeGreaterThan(0);
  });

  it('yaws toward the braked wheel under differential braking', () => {
    const leftBrake = probe((state) => state.velocity.set(25, 0, 0), 0, 1, 0, 60);
    const rightBrake = probe((state) => state.velocity.set(25, 0, 0), 0, 0, 1, 60);
    // The left brake drags the left wing back, so the nose swings LEFT and the
    // yaw moment is negative.
    expect(leftBrake.wrench.moment.z).toBeLessThan(-5000);
    expect(rightBrake.wrench.moment.z).toBeGreaterThan(5000);
    expect(leftBrake.wrench.moment.z).toBeCloseTo(-rightBrake.wrench.moment.z, 6);
    // Only the braked wheel slips. The free wheel rolls. The slip is small,
    // because MAX_BRAKE_TORQUE now sits below what the tire can pass at the
    // static load, so a full pedal holds the wheel turning near the front of
    // the curve instead of locking it. See the note on that constant.
    expect(leftBrake.gear.legs[MAIN_LEFT].slipRatio).toBeLessThan(-0.01);
    expect(leftBrake.gear.legs[MAIN_RIGHT].slipRatio).toBeCloseTo(0, 2);
    // The nose wheel has no brake, so it never slips under a brake command.
    expect(leftBrake.gear.legs[NOSE].slipRatio).toBeCloseTo(0, 2);
    // Both cases still slow the aircraft down.
    expect(leftBrake.wrench.force.x).toBeLessThan(0);
  });
});

// ---------------------------------------------------------------------------
// Standing still. Bead b54.
// ---------------------------------------------------------------------------

describe('the aircraft standing on the ground', () => {
  /** Thrust of both Jumo 004 at full power, N. CONVENTIONS section 8, firm. */
  const FULL_THRUST = 2 * 8800;
  /** Thrust of both engines at idle, N. src/aircraft/me262/engine.ts reports 230. */
  const IDLE_THRUST = 2 * 230;

  it('holds the aircraft on the brakes against both engines at full power', () => {
    // The pilot notes ask for a run up against the brakes. The numbers say the
    // aircraft can do it. The two main tires can pass 0.8 * 57.6 = 46 kN and the
    // brakes can react 12000 / 0.42 = 28.6 kN each, against 17.6 kN of thrust,
    // so both have room to spare. A slip based tire alone could not hold it,
    // because slip is a speed and at rest there is none.
    const result = run({
      height: ME262_STATIC_CG_HEIGHT + 0.001,
      seconds: 10,
      thrust: FULL_THRUST,
      brakeLeft: 1,
      brakeRight: 1,
    });
    // It NODS. The drag acts at the contact patch, 1.20 m below the center of
    // gravity, so 17.6 kN of thrust makes 21 kN m of nose down moment, the nose
    // leg takes twice its parked load and the center of gravity moves a few
    // centimeters forward. That is what a real aircraft does on a run up. Then
    // it stops: the speed at the end is a thousandth of the 0.06 m/s the model
    // used to creep at for ever.
    expect(result.state.position.x).toBeLessThan(0.05);
    expect(Math.abs(result.state.velocity.x)).toBeLessThan(1e-3);
  });

  it('rolls away once the thrust beats what the brakes can hold', () => {
    // The hold is not a freeze. Ask for more than the tires can pass and the
    // aircraft slides, or the model would hold it against anything at all.
    const result = run({
      height: ME262_STATIC_CG_HEIGHT + 0.001,
      seconds: 10,
      thrust: 60000,
      brakeLeft: 1,
      brakeRight: 1,
    });
    expect(result.state.position.x).toBeGreaterThan(5);
  });

  it('does not roll at idle thrust with the brakes off', () => {
    // Rolling resistance is 0.02 of the weight, which is 1254 N. The two engines
    // make 461 N at idle. The aircraft must stand still, and it did not: a
    // rolling resistance that faded to zero over a 0.3 m/s band let it roll at
    // 0.12 m/s for ever, which is a meter every ten seconds.
    const result = run({
      height: ME262_STATIC_CG_HEIGHT + 0.001,
      seconds: 10,
      thrust: IDLE_THRUST,
    });
    expect(result.state.position.x).toBeLessThan(0.05);
    expect(result.state.velocity.x).toBeLessThan(0.01);
  });

  it('rolls at idle once the thrust beats the rolling resistance', () => {
    // Above 1254 N the aircraft has to move, brakes or no brakes.
    const result = run({
      height: ME262_STATIC_CG_HEIGHT + 0.001,
      seconds: 10,
      thrust: 4000,
    });
    expect(result.state.position.x).toBeGreaterThan(5);
    expect(result.state.velocity.x).toBeGreaterThan(1);
  });
});

// ---------------------------------------------------------------------------
// The brakes. Bead b63.
//
// THIS FILE HOLDS THE ONLY BRAKE FADE MODEL OF THE PROJECT.
// src/aircraft/me262/systems.ts held a second one and multiplied the pilot
// command by its own fade before it handed the command to `update`, which then
// multiplied the torque by this one. Both applied, so the brakes faded about
// twice as fast as either model meant. systems.ts now passes the raw command.
// ---------------------------------------------------------------------------

/** What one landing roll on the brakes leaves behind. */
interface RollResult {
  /** Distance from the first brake application to the stop, m. */
  distance: number;
  /** Highest pack temperature of one main brake, K. */
  peakTemperature: number;
  /** Torque the pack still gives at that temperature, as a fraction of cold. */
  fadeAtPeak: number;
  /** Lowest slip ratio the left main reached. -1 is a locked wheel. */
  worstSlip: number;
}

/**
 * Rolls the aircraft out from a touch down at 175 km/h and brakes it to a stop.
 *
 * There is no aerodynamic force here, so the wheels carry the whole weight from
 * the first meter and every joule of the kinetic energy goes into the tires and
 * the brakes. That is the HARDEST case the brakes can meet: a real roll leaves
 * part of the weight on the wing and part of the energy in the airframe drag.
 */
function landingRoll(command: number, wheelShare = 1): RollResult {
  const gear = createMe262Gear();
  const state = createState();
  const wrench = createWrench();
  const gravityWorld = new Vector3(0, 0, wheelShare * WEIGHT);
  const gravityBody = new Vector3();
  const acceleration = new Vector3();
  state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
  state.velocity.set(kmhToMs(175), 0, 0);
  let start = 0;
  let peakTemperature = 0;
  let worstSlip = 0;
  for (let i = 0; i < 240 * 120 && state.velocity.x > 0.5; i++) {
    // One second of free roll first, so the wheels are turning when the pedal
    // goes down. A wheel that never turned locks whatever the pilot does.
    const braking = i >= 240;
    if (i === 240) {
      start = state.position.x;
    }
    clearWrench(wrench);
    gear.update(state, 1, 0, braking ? command : 0, braking ? command : 0, DT, wrench);
    peakTemperature = Math.max(peakTemperature, gear.legs[MAIN_LEFT].brakeTemp);
    if (braking && state.velocity.x > 5) {
      worstSlip = Math.min(worstSlip, gear.legs[MAIN_LEFT].slipRatio);
    }
    worldToBody(state.orientation, gravityWorld, gravityBody);
    wrench.force.add(gravityBody);
    acceleration.copy(wrench.force).applyQuaternion(state.orientation).multiplyScalar(1 / MASS);
    state.velocity.addScaledVector(acceleration, DT);
    state.position.x += state.velocity.x * DT;
  }
  return {
    distance: state.position.x - start,
    peakTemperature,
    fadeAtPeak: brakeFade(peakTemperature),
    worstSlip,
  };
}

/** The pedal that puts the tire exactly at the peak of its curve at rest. */
function lockPedal(wheelShare = 1): number {
  const mainStaticLoad = 0.5 * (1 - ME262_NOSE_LOAD_FRACTION) * wheelShare * WEIGHT;
  return (
    (tireLongitudinalMu(TIRE_PEAK_SLIP_RATIO) * mainStaticLoad * LEGS[MAIN_LEFT].wheelRadius) /
    MAX_BRAKE_TORQUE
  );
}

describe('brake heat and fade', () => {
  it('holds the cold torque below the lining temperature and half of it above', () => {
    // One curve, one place. The lining of 1944 is asbestos and resin. It holds
    // its friction to about 200 C, breaks down as the binder chars, and keeps
    // about half of the cold value when it is fully gone.
    expect(brakeFade(288.15)).toBe(1);
    expect(brakeFade(BRAKE_FADE_START_TEMPERATURE)).toBe(1);
    expect(brakeFade(BRAKE_FADE_FULL_TEMPERATURE)).toBeCloseTo(0.5, 12);
    expect(brakeFade(2000)).toBeCloseTo(0.5, 12);
    // Halfway between the two anchors the curve sits halfway through the fade.
    const middle = 0.5 * (BRAKE_FADE_START_TEMPERATURE + BRAKE_FADE_FULL_TEMPERATURE);
    expect(brakeFade(middle)).toBeCloseTo(0.75, 12);
    // It never rises with temperature.
    let previous = 1;
    for (let t = 288.15; t < 1200; t += 5) {
      const value = brakeFade(t);
      expect(value).toBeLessThanOrEqual(previous + 1e-12);
      previous = value;
    }
  });

  it('fades one fifth of the brake over a full landing roll from touch down', () => {
    // 6396 kg at 48.6 m/s carries 7.6 MJ, so each of the two packs takes up to
    // 3.8 MJ. At BRAKE_HEAT_CAPACITY that is a rise near 270 K, which lands the
    // pack inside the fade band. THE ME 262 WAS KNOWN FOR WEAK BRAKES AND LONG
    // LANDING RUNS, so a model whose pack never reaches the band at all would
    // carry a fade that never acts.
    const firm = landingRoll(0.95 * lockPedal());
    expect(firm.peakTemperature).toBeGreaterThan(BRAKE_FADE_START_TEMPERATURE);
    expect(firm.peakTemperature).toBeLessThan(BRAKE_FADE_FULL_TEMPERATURE);
    // The pack keeps between 70 and 90 percent of its cold torque at the stop.
    expect(firm.fadeAtPeak).toBeGreaterThan(0.7);
    expect(firm.fadeAtPeak).toBeLessThan(0.9);
    // The wheel still turns, which is what makes the heat in the pack.
    expect(firm.worstSlip).toBeGreaterThan(-0.5);
  });

  it('puts the energy of a locked wheel into the tire and leaves the pack cold', () => {
    // A locked pack no longer slides against the disc, so it takes no heat at
    // all. The tire takes it instead. That is why a pilot who locks a wheel
    // ruins a tire and keeps his brakes.
    //
    // THE ROLL CARRIES 40 PERCENT OF THE WEIGHT ON THE WHEELS, because that is
    // where this brake can still lock a wheel. See the note on MAX_BRAKE_TORQUE
    // and the tire test above.
    const locked = landingRoll(1, 0.4);
    expect(locked.worstSlip).toBeLessThan(-0.9);
    // The pack takes the energy of the spin down and nothing after it, which is
    // 35 K here against the 253 K of a roll where the wheel keeps turning.
    expect(locked.peakTemperature - 288.15).toBeLessThan(50);
    expect(locked.fadeAtPeak).toBe(1);
  });

  it('loses to a locked wheel once the pack has faded to half its torque', () => {
    // A pack at BRAKE_FADE_FULL asks the tire for 0.5 * 0.8 = 0.40, and a locked
    // tire slides at 0.76 * 0.8 = 0.61. The brake is then the weaker of the two.
    // This is the reason the tire test above starts at 40 m/s and not at 60.
    const cold = tireLongitudinalMu(TIRE_PEAK_SLIP_RATIO) * brakeFade(288.15);
    const hot = tireLongitudinalMu(TIRE_PEAK_SLIP_RATIO) * brakeFade(BRAKE_FADE_FULL_TEMPERATURE);
    const locked = Math.abs(tireLongitudinalMu(-1));
    expect(cold).toBeGreaterThan(locked);
    expect(hot).toBeLessThan(locked);
  });

  it('heats the pack and loses torque as it heats', () => {
    // Hold the aircraft at 60 m/s and drag the brakes. The wheel keeps turning,
    // so the pack slides against the disc and takes the energy. The pedal is
    // full, because MAX_BRAKE_TORQUE now sits below the tire at this load and a
    // full pedal no longer locks the wheel.
    const gear = createMe262Gear();
    const state = createState();
    const wrench = createWrench();
    state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
    state.velocity.set(60, 0, 0);
    let coldForce = 0;
    for (let i = 0; i < 240 * 25; i++) {
      clearWrench(wrench);
      gear.update(state, 1, 0, 1, 1, DT, wrench);
      if (i === 240) {
        coldForce = -wrench.force.x;
      }
    }
    const hotForce = -wrench.force.x;
    expect(gear.legs[MAIN_LEFT].brakeTemp).toBeGreaterThan(800);
    expect(hotForce).toBeLessThan(0.7 * coldForce);
    // The nose wheel has no brake, so its pack never heats.
    expect(gear.legs[NOSE].brakeTemp).toBeCloseTo(288.15, 4);
  });

  it('cools the pack back toward the air when the brake comes off', () => {
    const gear = createMe262Gear();
    const state = createState();
    const wrench = createWrench();
    state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
    state.velocity.set(60, 0, 0);
    for (let i = 0; i < 240 * 10; i++) {
      clearWrench(wrench);
      gear.update(state, 1, 0, 0.5, 0.5, DT, wrench);
    }
    const hot = gear.legs[MAIN_LEFT].brakeTemp;
    expect(hot).toBeGreaterThan(400);
    for (let i = 0; i < 240 * 300; i++) {
      clearWrench(wrench);
      gear.update(state, 1, 0, 0, 0, DT, wrench);
    }
    expect(gear.legs[MAIN_LEFT].brakeTemp).toBeLessThan(0.5 * (hot - 288.15) + 288.15);
  });
});

// ---------------------------------------------------------------------------
// The wrench contract
// ---------------------------------------------------------------------------

describe('the wrench contract', () => {
  it('makes no force at all with the gear retracted', () => {
    const gear = createMe262Gear();
    const state = createState();
    const wrench = createWrench();
    // Sit the aircraft where the wheels would be well under the runway.
    state.position.set(0, 0, -0.5);
    clearWrench(wrench);
    gear.update(state, 0, 0, 1, 1, DT, wrench);
    expect(wrench.force.length()).toBe(0);
    expect(wrench.moment.length()).toBe(0);
    expect(gear.anyOnGround).toBe(false);
    for (const leg of gear.legs) {
      expect(leg.load).toBe(0);
      expect(leg.onGround).toBe(false);
    }
  });

  it('leaves the wrench unchanged with every wheel above the ground', () => {
    const gear = createMe262Gear();
    const state = createState();
    const wrench = createWrench();
    state.position.set(0, 0, -50);
    state.velocity.set(80, 0, 2);
    wrench.force.set(11, -22, 33);
    wrench.moment.set(-44, 55, -66);
    gear.update(state, 1, 0.5, 1, 1, DT, wrench);
    expect(wrench.force.toArray()).toEqual([11, -22, 33]);
    expect(wrench.moment.toArray()).toEqual([-44, 55, -66]);
    expect(gear.anyOnGround).toBe(false);
  });

  it('adds into the wrench instead of writing over it', () => {
    // One wrench collects the aerodynamics, the thrust, gravity and the gear, so
    // update must never clear what is already there.
    const gear = createMe262Gear();
    const state = createState();
    const one = createWrench();
    const two = createWrench();
    state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
    state.velocity.set(20, 3, 1);
    gear.update(state, 1, 0.3, 0.4, 0.1, DT, one);
    gear.reset();
    gear.update(state, 1, 0.3, 0.4, 0.1, DT, two);
    gear.reset();
    gear.update(state, 1, 0.3, 0.4, 0.1, DT, two);
    expect(two.force.x).toBeCloseTo(2 * one.force.x, 6);
    expect(two.force.z).toBeCloseTo(2 * one.force.z, 6);
    expect(two.moment.z).toBeCloseTo(2 * one.moment.z, 6);
  });

  it('reports contact and clears every leg on reset', () => {
    const hard = run({ height: extendedDepth(MAIN_LEFT) + 0.002, sink: 7, seconds: 1 });
    expect(hard.gear.anyOnGround).toBe(true);
    expect(hard.gear.legs[MAIN_LEFT].burst).toBe(true);
    hard.gear.reset();
    expect(hard.gear.anyOnGround).toBe(false);
    for (const leg of hard.gear.legs) {
      expect(leg.burst).toBe(false);
      expect(leg.load).toBe(0);
      expect(leg.compression).toBe(0);
      expect(leg.wheelSpeed).toBe(0);
      expect(leg.brakeTemp).toBeCloseTo(288.15, 6);
    }
  });
});
