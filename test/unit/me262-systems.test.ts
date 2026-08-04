/**
 * The moving parts of the Me 262 A-1a, and the flap physics they drive.
 *
 * The first group of tests is the reason bead b19 touched
 * src/physics/aero/surface.ts. A flap that moves the zero lift angle alone gives
 * the wing the SAME peak lift at a lower angle, so lowering the flap made the
 * aircraft stall sooner and no slower. That is backwards. The first two tests
 * check the fix from both sides, on the section and on the whole aircraft.
 *
 * Every stall speed below is worked out at 6400 kg at sea level, which is the
 * mass CONVENTIONS section 8 quotes the published stall speed at.
 */

import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import { DEG, kmhToMs, msToKmh } from '@/math/units';
import { SEA_LEVEL_DENSITY } from '@/physics/atmosphere';
import { clearWrench, createState, createWrench } from '@/physics/rigidbody';
import { NACA_0011 } from '@/physics/aero/airfoil';
import { STALL_NACA_0011 } from '@/physics/aero/stall';
import { createSurface, evaluateSurface } from '@/physics/aero/surface';
import type { SurfaceDef } from '@/physics/aero/surface';
import {
  CONTROL_COUNT,
  CONTROL_INDEX,
  FLAP_LANDING_ANGLE,
  FLAP_TAKEOFF_ANGLE,
  SLAT_DEPLOY_ALPHA,
  WING_AREA,
  createMe262Assembly,
} from '@/aircraft/me262/geometry';
import { FUEL_CAPACITY, me262Mass } from '@/aircraft/me262/mass';
import {
  BRAKE_TORQUE_MAX,
  FLAP_TRAVEL_TIME,
  GEAR_LIMIT_SPEED,
  GEAR_TRAVEL_TIME,
  SLAT_CLOSE_ALPHA,
  SLAT_OPEN_ALPHA,
  SLAT_TRAVEL_TIME,
  createMe262Systems,
  flapLimitSpeed,
  gearDragAreaAt,
} from '@/aircraft/me262/systems';

/** The mass CONVENTIONS section 8 quotes the stall speed at. */
const STALL_MASS = 6400; // kg

/** One peak of a lift curve: the coefficient and the angle it appears at. */
interface Peak {
  clMax: number;
  alphaDeg: number;
  wingClMax: number;
  wingAlphaDeg: number;
}

/**
 * Sweeps the angle of attack of the whole aircraft and returns the peak lift
 * coefficient, once for the aircraft and once for the sixteen wing strips.
 *
 * The wing figure exists because the tail and the two bodies also make lift, and
 * the flap acts on the wing alone. The two peaks therefore sit at two angles,
 * and the wing one shows the flap effect without the tail in the way.
 *
 * The sweep evaluates every angle twice with a one second step. The separation
 * lag of a strip runs out in about a twentieth of a second, so the second call
 * reports the steady answer.
 */
function sweep(flapDeflection: number, speed = 55): Peak {
  const assembly = createMe262Assembly();
  const controls = new Float64Array(CONTROL_COUNT);
  controls[CONTROL_INDEX.flap] = flapDeflection;
  const state = createState();
  const wrench = createWrench();
  const wind = new Vector3();
  const peak: Peak = { clMax: 0, alphaDeg: 0, wingClMax: 0, wingAlphaDeg: 0 };

  for (let deg = 0; deg <= 30; deg += 0.05) {
    const alpha = deg * DEG;
    state.velocity.set(speed * Math.cos(alpha), 0, speed * Math.sin(alpha));
    clearWrench(wrench);
    assembly.evaluate(state, wind, controls, 1, wrench);
    clearWrench(wrench);
    const totals = assembly.evaluate(state, wind, controls, 1, wrench);
    const reference = totals.dynamicPressure * WING_AREA;

    const cl = totals.lift / reference;
    if (cl > peak.clMax) {
      peak.clMax = cl;
      peak.alphaDeg = deg;
    }

    // The wind axis lift of the sixteen wing strips alone.
    let wingLift = 0;
    for (let i = 0; i < 16; i++) {
      const f = assembly.surfaces[i].result.force;
      wingLift += -(-f.x * Math.sin(alpha) + f.z * Math.cos(alpha));
    }
    const wingCl = wingLift / reference;
    if (wingCl > peak.wingClMax) {
      peak.wingClMax = wingCl;
      peak.wingAlphaDeg = deg;
    }
  }
  return peak;
}

/** The level flight stall speed of a wing loading at a peak lift coefficient. */
function stallSpeedKmh(clMax: number, mass = STALL_MASS): number {
  return msToKmh(
    Math.sqrt((2 * mass * 9.80665) / (SEA_LEVEL_DENSITY * WING_AREA * clMax)),
  );
}

/** One test strip. Only the flap fields change between the tests below. */
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

/** Sweeps one strip and returns the peak of its section lift curve. */
function sectionPeak(def: SurfaceDef, deflection: number): { cl: number; alphaDeg: number } {
  const surface = createSurface(def);
  const controls = new Float64Array(CONTROL_COUNT);
  controls[CONTROL_INDEX.flap] = deflection;
  const wrench = createWrench();
  const zero = new Vector3();
  const velocity = new Vector3();
  let cl = 0;
  let alphaDeg = 0;
  for (let deg = -5; deg <= 35; deg += 0.05) {
    const alpha = deg * DEG;
    velocity.set(60 * Math.cos(alpha), 0, 60 * Math.sin(alpha));
    // Two calls with a one second step leave no separation lag behind.
    for (let k = 0; k < 2; k++) {
      clearWrench(wrench);
      evaluateSurface(surface, velocity, zero, zero, SEA_LEVEL_DENSITY, 340.294, controls, 0, 1, wrench);
    }
    if (surface.result.cl > cl) {
      cl = surface.result.cl;
      alphaDeg = deg;
    }
  }
  return { cl, alphaDeg };
}

/** The wing strip flap numbers of src/aircraft/me262/geometry.ts, on one strip. */
const FLAPPED_STRIP = strip({
  flapIndex: CONTROL_INDEX.flap,
  flapEffectiveness: -0.26,
  flapClMaxDelta: 1.2 / FLAP_LANDING_ANGLE,
  flapAlphaDelta: (1.2 * DEG) / FLAP_LANDING_ANGLE,
});

describe('the flap raises the peak lift of its section and stalls it earlier', () => {
  it('a slotted flap at 50 degrees adds about 1.2 to the section peak lift', () => {
    const clean = sectionPeak(FLAPPED_STRIP, 0);
    const flapped = sectionPeak(FLAPPED_STRIP, FLAP_LANDING_ANGLE);
    // Raymer gives 1.3 for the section peak increment of a slotted flap and
    // Hoerner gives 1.0 to 1.3. The model asks for 1.2 at full travel.
    expect(flapped.cl - clean.cl).toBeGreaterThan(1.1);
    expect(flapped.cl - clean.cl).toBeLessThan(1.3);
  });

  it('the flapped section stalls 3 to 5 degrees before the clean section', () => {
    const clean = sectionPeak(FLAPPED_STRIP, 0);
    const flapped = sectionPeak(FLAPPED_STRIP, FLAP_LANDING_ANGLE);
    // A NACA 23012 with a 25.7 percent slotted flap at 40 degrees peaks at 12.5
    // degrees against 16 degrees clean. The model must show the same fall.
    expect(clean.alphaDeg - flapped.alphaDeg).toBeGreaterThan(3);
    expect(clean.alphaDeg - flapped.alphaDeg).toBeLessThan(5);
  });

  it('the flapped section keeps the straight lift line of the clean section', () => {
    // The whole of the flap lift below the stall is the zero lift shift, so the
    // flapped section must sit on clAlpha times the angle above its own zero
    // lift angle. The peak terms move the curve and give the straight part back,
    // so they may not change that line. The one percent that is left is the
    // trailing edge separation of the clean section, which is already a little
    // below one at these angles.
    const surface = createSurface(FLAPPED_STRIP);
    const controls = new Float64Array(CONTROL_COUNT);
    controls[CONTROL_INDEX.flap] = FLAP_LANDING_ANGLE;
    const wrench = createWrench();
    const zero = new Vector3();
    const shift = 0.26 * FLAP_LANDING_ANGLE; // rad, the zero lift shift
    for (const deg of [-10, -8, -6, -4, -2, 0, 2]) {
      const alpha = deg * DEG;
      const velocity = new Vector3(60 * Math.cos(alpha), 0, 60 * Math.sin(alpha));
      for (let k = 0; k < 2; k++) {
        clearWrench(wrench);
        evaluateSurface(surface, velocity, zero, zero, SEA_LEVEL_DENSITY, 340.294, controls, 0, 1, wrench);
      }
      const straight = NACA_0011.clAlpha * (alpha + shift);
      // The band is 5 percent. The strip carries the Prandtl-Glauert growth of
      // compressibility.ts, which adds 1.6 percent at 60 m/s, and the trailing
      // edge separation of the section takes about 1 percent off.
      expect(surface.result.cl / straight).toBeGreaterThan(0.95);
      expect(surface.result.cl / straight).toBeLessThan(1.05);
    }
  });

  it('a strip with no flap deflection gives the clean answer to the last digit', () => {
    // This is the guard on the clean aircraft. The new fields may not move one
    // count of lift while the flap is up.
    const flapFields = createSurface(FLAPPED_STRIP);
    const plain = createSurface(strip());
    const controls = new Float64Array(CONTROL_COUNT);
    const wrench = createWrench();
    const zero = new Vector3();
    for (const deg of [0, 5, 10, 15, 20, 25]) {
      const alpha = deg * DEG;
      const velocity = new Vector3(60 * Math.cos(alpha), 0, 60 * Math.sin(alpha));
      for (let k = 0; k < 2; k++) {
        clearWrench(wrench);
        evaluateSurface(flapFields, velocity, zero, zero, SEA_LEVEL_DENSITY, 340.294, controls, 0, 1, wrench);
        clearWrench(wrench);
        evaluateSurface(plain, velocity, zero, zero, SEA_LEVEL_DENSITY, 340.294, controls, 0, 1, wrench);
      }
      expect(flapFields.result.cl).toBe(plain.result.cl);
    }
  });
});

describe('the flap raises the maximum lift coefficient of the aircraft', () => {
  const clean = sweep(0);
  const landing = sweep(FLAP_LANDING_ANGLE);
  const takeoff = sweep(FLAP_TAKEOFF_ANGLE);

  it('the landing flap raises the peak lift coefficient above the clean value', () => {
    expect(landing.clMax).toBeGreaterThan(clean.clMax);
    expect(landing.wingClMax).toBeGreaterThan(clean.wingClMax);
  });

  it('the landing flap brings the stall on at a lower angle of attack', () => {
    expect(landing.alphaDeg).toBeLessThan(clean.alphaDeg);
    // The wing carries the whole of the flap, so it shows the fall clearly. The
    // tail and the two bodies keep working past the wing peak and move the
    // aircraft peak a little later than the wing peak in both configurations.
    expect(clean.wingAlphaDeg - landing.wingAlphaDeg).toBeGreaterThan(0.5);
  });

  it('the take off flap sits between the clean wing and the landing wing', () => {
    expect(takeoff.clMax).toBeGreaterThan(clean.clMax);
    expect(takeoff.clMax).toBeLessThan(landing.clMax);
    expect(takeoff.alphaDeg).toBeLessThan(clean.alphaDeg);
    expect(takeoff.alphaDeg).toBeGreaterThan(landing.alphaDeg);
  });

  it('the clean aircraft holds its peak lift coefficient near 1.5 and stalls near 200 km/h', () => {
    // CONVENTIONS section 8 records 1.63 and 194 km/h from bead b17. The model
    // moved a little when bead b18 added the downwash at the tail. The flap work
    // of bead b19 changes nothing while the flap is up, which the section test
    // above proves to the last digit.
    expect(clean.clMax).toBeGreaterThan(1.4);
    expect(clean.clMax).toBeLessThan(1.75);
    const speed = stallSpeedKmh(clean.clMax);
    expect(speed).toBeGreaterThan(190);
    expect(speed).toBeLessThan(210);
  });

  it('the landing configuration stalls inside the 180 to 202 km/h band of the handbook', () => {
    // "Stalling speed with full fuel load, landing gear and flaps down:
    // 202 km/hr (125 mph)" and "The airplane stalls at 180 to 202 km/hr".
    // Source: "Pilot's Handbook for Me-262 A-1", speed and range table, and the
    // approach and landing section. The published 175 km/h of CONVENTIONS
    // section 8 is the TOUCH DOWN speed of the Wendel notes, which is a lower
    // number than any stall speed in level flight.
    const speed = stallSpeedKmh(landing.clMax);
    expect(landing.clMax).toBeGreaterThan(1.6);
    expect(speed).toBeGreaterThan(178);
    expect(speed).toBeLessThan(204);
  });
});

describe('the flap and the gear take their travel time', () => {
  it('the flap needs the whole travel time and moves less than 2 percent in one step', () => {
    const systems = createMe262Systems();
    systems.commandFlaps('landing');
    systems.update(0, 60, 0, 1 / 60);
    expect(systems.state.flapPosition).toBeGreaterThan(0);
    expect(systems.state.flapPosition).toBeLessThan(0.02);

    // Just before the travel time the flap is not down yet.
    for (let t = 1 / 60; t < FLAP_TRAVEL_TIME - 0.5; t += 1 / 60) {
      systems.update(0, 60, 0, 1 / 60);
    }
    expect(systems.state.flapPosition).toBeLessThan(1);
    for (let t = 0; t < 1; t += 1 / 60) {
      systems.update(0, 60, 0, 1 / 60);
    }
    expect(systems.state.flapPosition).toBe(1);
  });

  it('the take off setting stops at 20 of the 50 degrees of travel', () => {
    const systems = createMe262Systems();
    systems.commandFlaps('takeoff');
    for (let t = 0; t < FLAP_TRAVEL_TIME; t += 1 / 60) {
      systems.update(0, 60, 0, 1 / 60);
    }
    expect(systems.state.flapPosition).toBeCloseTo(FLAP_TAKEOFF_ANGLE / FLAP_LANDING_ANGLE, 6);
    expect(systems.flapDeflection()).toBeCloseTo(FLAP_TAKEOFF_ANGLE, 9);
  });

  it('the gear needs its travel time and the nose wheel follows the main gear', () => {
    const systems = createMe262Systems();
    systems.commandGear(false);
    for (let t = 0; t < GEAR_TRAVEL_TIME + 1; t += 1 / 60) {
      systems.update(0, 100, 0, 1 / 60);
    }
    expect(systems.state.gearPosition).toBe(0);

    systems.commandGear(true);
    systems.update(0, 100, 0, 1 / 60);
    expect(systems.state.gearPosition).toBeLessThan(0.02);
    // Halfway through the cycle the mains are down and the nose wheel is not.
    for (let t = 1 / 60; t < GEAR_TRAVEL_TIME / 2; t += 1 / 60) {
      systems.update(0, 100, 0, 1 / 60);
    }
    expect(systems.mainGearPosition()).toBeGreaterThan(systems.noseGearPosition());
    expect(systems.state.gearPosition).toBeLessThan(1);
    for (let t = 0; t < GEAR_TRAVEL_TIME / 2 + 1; t += 1 / 60) {
      systems.update(0, 100, 0, 1 / 60);
    }
    expect(systems.state.gearPosition).toBe(1);
    expect(systems.noseGearPosition()).toBe(1);
  });

  it('the gear drags when it is down and carries an extra bump in transit', () => {
    // A gear that is up drags nothing. A gear that is down carries the whole
    // gear. A gear in transit carries the open doors and the broadside leg as
    // well, so it drags MORE than the straight line between the two ends.
    expect(gearDragAreaAt(0)).toBe(0);
    expect(gearDragAreaAt(1)).toBeGreaterThan(0.3);
    expect(gearDragAreaAt(0.5)).toBeGreaterThan(0.5 * gearDragAreaAt(1));
    expect(gearDragAreaAt(0.5)).toBeGreaterThan(0.75 * gearDragAreaAt(1));
    expect(gearDragAreaAt(0.25)).toBeGreaterThan(0.25 * gearDragAreaAt(1));
  });
});

describe('the automatic slat', () => {
  it('opens from the angle of attack alone, with no pilot command', () => {
    const systems = createMe262Systems();
    // The interface carries no slat command, so this is the only way to open it.
    for (let t = 0; t < 2; t += 1 / 60) {
      systems.update(SLAT_OPEN_ALPHA + 1 * DEG, 60, 0, 1 / 60);
    }
    expect(systems.state.slatPosition).toBe(1);
  });

  it('takes its travel time and does not snap open', () => {
    const systems = createMe262Systems();
    systems.update(SLAT_OPEN_ALPHA + 1 * DEG, 60, 0, 1 / 60);
    expect(systems.state.slatPosition).toBeGreaterThan(0);
    expect(systems.state.slatPosition).toBeLessThan(0.1);
    for (let t = 1 / 60; t < SLAT_TRAVEL_TIME - 0.05; t += 1 / 60) {
      systems.update(SLAT_OPEN_ALPHA + 1 * DEG, 60, 0, 1 / 60);
    }
    expect(systems.state.slatPosition).toBeLessThan(1);
  });

  it('holds open between the two thresholds, which is the hysteresis', () => {
    const systems = createMe262Systems();
    for (let t = 0; t < 2; t += 1 / 60) {
      systems.update(SLAT_OPEN_ALPHA + 1 * DEG, 60, 0, 1 / 60);
    }
    // An angle below the open threshold but above the close threshold must not
    // shut the slat.
    const between = 0.5 * (SLAT_OPEN_ALPHA + SLAT_CLOSE_ALPHA);
    for (let t = 0; t < 2; t += 1 / 60) {
      systems.update(between, 60, 0, 1 / 60);
    }
    expect(systems.state.slatPosition).toBe(1);
    // Below the close threshold it runs back in.
    for (let t = 0; t < 2; t += 1 / 60) {
      systems.update(SLAT_CLOSE_ALPHA - 1 * DEG, 60, 0, 1 / 60);
    }
    expect(systems.state.slatPosition).toBe(0);
  });

  it('does not chatter when the angle of attack sits on the threshold', () => {
    const systems = createMe262Systems();
    for (let i = 0; i < 500; i++) {
      systems.update(SLAT_OPEN_ALPHA, 60, 0, 1 / 240);
    }
    expect(systems.state.slatPosition).toBe(0);
    // The same angle after the slat is open leaves it open.
    for (let t = 0; t < 2; t += 1 / 60) {
      systems.update(SLAT_OPEN_ALPHA + 1 * DEG, 60, 0, 1 / 60);
    }
    for (let i = 0; i < 500; i++) {
      systems.update(SLAT_OPEN_ALPHA, 60, 0, 1 / 240);
    }
    expect(systems.state.slatPosition).toBe(1);
  });

  it('opens at the angle the outer wing strips of the aerodynamics use', () => {
    expect(SLAT_OPEN_ALPHA).toBe(SLAT_DEPLOY_ALPHA);
    expect(SLAT_CLOSE_ALPHA).toBeLessThan(SLAT_OPEN_ALPHA);
  });
});

describe('overspeed damage', () => {
  it('the gear takes damage above the 400 km/h limit and none below it', () => {
    const fast = createMe262Systems();
    for (let t = 0; t < 5; t += 1 / 60) {
      fast.update(0, GEAR_LIMIT_SPEED * 1.3, 0, 1 / 60);
    }
    expect(fast.state.damage.gear).toBeGreaterThan(0);

    const slow = createMe262Systems();
    for (let t = 0; t < 5; t += 1 / 60) {
      slow.update(0, GEAR_LIMIT_SPEED * 0.99, 0, 1 / 60);
    }
    expect(slow.state.damage.gear).toBe(0);
  });

  it('a gear that is up and locked takes no damage at any speed', () => {
    const systems = createMe262Systems();
    systems.commandGear(false);
    for (let t = 0; t < GEAR_TRAVEL_TIME + 1; t += 1 / 60) {
      systems.update(0, 100, 0, 1 / 60);
    }
    const damaged = systems.state.damage.gear;
    for (let t = 0; t < 5; t += 1 / 60) {
      systems.update(0, kmhToMs(800), 0, 1 / 60);
    }
    expect(systems.state.damage.gear).toBe(damaged);
  });

  it('the flap limit falls as the flap goes down, and a fast flap takes damage', () => {
    expect(flapLimitSpeed(0)).toBe(Number.POSITIVE_INFINITY);
    expect(flapLimitSpeed(1)).toBeLessThan(flapLimitSpeed(FLAP_TAKEOFF_ANGLE / FLAP_LANDING_ANGLE));

    const systems = createMe262Systems();
    systems.commandFlaps('landing');
    for (let t = 0; t < FLAP_TRAVEL_TIME + 1; t += 1 / 60) {
      systems.update(0, kmhToMs(250), 0, 1 / 60);
    }
    expect(systems.state.damage.flap).toBe(0);
    for (let t = 0; t < 5; t += 1 / 60) {
      systems.update(0, kmhToMs(500), 0, 1 / 60);
    }
    expect(systems.state.damage.flap).toBeGreaterThan(0);
  });

  it('a destroyed actuator stops moving', () => {
    const systems = createMe262Systems();
    systems.commandFlaps('landing');
    for (let t = 0; t < 200; t += 1 / 60) {
      systems.update(0, kmhToMs(900), 0, 1 / 60);
    }
    expect(systems.state.damage.flap).toBe(1);
    const stuck = systems.state.flapPosition;
    systems.commandFlaps('up');
    for (let t = 0; t < 10; t += 1 / 60) {
      systems.update(0, kmhToMs(200), 0, 1 / 60);
    }
    expect(systems.state.flapPosition).toBe(stuck);
  });
});

describe('the wheel brakes', () => {
  it('give a torque that follows the command, on each side on its own', () => {
    const systems = createMe262Systems();
    systems.setBrakes(1, 0);
    systems.update(0, 20, 0, 1 / 60);
    expect(systems.brakeTorqueLeft()).toBeCloseTo(BRAKE_TORQUE_MAX, 6);
    expect(systems.brakeTorqueRight()).toBe(0);
  });

  it('fade as the heat builds and come back as the brake cools', () => {
    const systems = createMe262Systems();
    systems.setBrakes(1, 1);
    // A landing roll from 60 m/s, held for half a minute.
    for (let t = 0; t < 30; t += 1 / 60) {
      systems.update(0, 60, 0, 1 / 60);
    }
    const hot = systems.brakeTorqueLeft();
    expect(systems.brakeHeatLeft()).toBeGreaterThan(100);
    expect(hot).toBeLessThan(BRAKE_TORQUE_MAX);

    // Let go of the brakes and let the wheel cool.
    systems.setBrakes(0, 0);
    for (let t = 0; t < 600; t += 1 / 60) {
      systems.update(0, 0, 0, 1 / 60);
    }
    systems.setBrakes(1, 1);
    systems.update(0, 20, 0, 1 / 60);
    expect(systems.brakeTorqueLeft()).toBeGreaterThan(hot);
  });

  it('make no heat while the wheel is still inside the wing', () => {
    const systems = createMe262Systems();
    systems.commandGear(false);
    for (let t = 0; t < GEAR_TRAVEL_TIME + 1; t += 1 / 60) {
      systems.update(0, 100, 0, 1 / 60);
    }
    systems.setBrakes(1, 1);
    for (let t = 0; t < 10; t += 1 / 60) {
      systems.update(0, 100, 0, 1 / 60);
    }
    expect(systems.brakeHeatLeft()).toBe(0);
  });
});

describe('the fuel system', () => {
  it('burns the fuel at the rate the engines ask for and stops at empty', () => {
    const systems = createMe262Systems();
    expect(systems.state.fuelMass).toBe(FUEL_CAPACITY);
    for (let t = 0; t < 10; t += 1 / 60) {
      systems.update(0, 100, 2, 1 / 60);
    }
    expect(systems.state.fuelMass).toBeCloseTo(FUEL_CAPACITY - 20, 6);
    for (let t = 0; t < 3000; t += 1) {
      systems.update(0, 100, 2, 1);
    }
    expect(systems.state.fuelMass).toBe(0);
  });

  it('empties the rear auxiliary tank first, so the balance moves forward first', () => {
    // src/aircraft/me262/mass.ts owns the four tanks and their burn order. The
    // pilot notes tell the pilot to burn the rear auxiliary tank first, because
    // the aircraft is tail heavy while that tank is full. The center of gravity
    // must therefore move FORWARD over the first 498 kg and AFT after it.
    const full = me262Mass(FUEL_CAPACITY).cgFromNose;
    const auxEmpty = me262Mass(FUEL_CAPACITY - 498).cgFromNose;
    const mainsHalf = me262Mass(FUEL_CAPACITY - 498 - 700).cgFromNose;
    expect(auxEmpty).toBeLessThan(full);
    expect(mainsHalf).toBeGreaterThan(auxEmpty);
  });

  it('feeds a moving center of gravity into the mass model as it burns', () => {
    const systems = createMe262Systems();
    const start = me262Mass(systems.state.fuelMass);
    for (let t = 0; t < 600; t += 1) {
      systems.update(0, 200, 0.83, 1);
    }
    const later = me262Mass(systems.state.fuelMass);
    expect(later.mass).toBeLessThan(start.mass);
    expect(later.cgFromNose).toBeLessThan(start.cgFromNose);
  });
});

describe('the control array the systems write', () => {
  it('writes the flap deflection in radians and the slat as a fraction', () => {
    const systems = createMe262Systems();
    const controls = new Float64Array(CONTROL_COUNT);
    systems.commandFlaps('landing');
    for (let t = 0; t < FLAP_TRAVEL_TIME + 1; t += 1 / 60) {
      systems.update(SLAT_OPEN_ALPHA + 2 * DEG, 60, 0, 1 / 60);
    }
    systems.writeControls(controls);
    expect(controls[CONTROL_INDEX.flap]).toBeCloseTo(FLAP_LANDING_ANGLE, 9);
    expect(controls[CONTROL_INDEX.slat]).toBe(1);
    // The systems touch no other channel.
    expect(controls[CONTROL_INDEX.aileron]).toBe(0);
    expect(controls[CONTROL_INDEX.elevator]).toBe(0);
    expect(controls[CONTROL_INDEX.rudder]).toBe(0);
  });

  it('drives the aerodynamics: the written flap makes more lift than a flap that is up', () => {
    const systems = createMe262Systems();
    const controls = new Float64Array(CONTROL_COUNT);
    const assembly = createMe262Assembly();
    const state = createState();
    const wrench = createWrench();
    const wind = new Vector3();
    const alpha = 6 * DEG;
    state.velocity.set(60 * Math.cos(alpha), 0, 60 * Math.sin(alpha));

    systems.writeControls(controls);
    clearWrench(wrench);
    assembly.evaluate(state, wind, controls, 1, wrench);
    clearWrench(wrench);
    const up = assembly.evaluate(state, wind, controls, 1, wrench).lift;

    systems.commandFlaps('landing');
    for (let t = 0; t < FLAP_TRAVEL_TIME + 1; t += 1 / 60) {
      systems.update(alpha, 60, 0, 1 / 60);
    }
    systems.writeControls(controls);
    clearWrench(wrench);
    assembly.evaluate(state, wind, controls, 1, wrench);
    clearWrench(wrench);
    const down = assembly.evaluate(state, wind, controls, 1, wrench).lift;

    expect(down).toBeGreaterThan(up);
  });
});
