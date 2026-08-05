/**
 * Structural limits and failures of the Me 262 A-1a.
 *
 * The tests check three things that no other test in the project can check.
 *
 *   1. The numbers. The limit load, the ultimate load, and how both fall as the
 *      aircraft gets heavier.
 *   2. The difference between the two bounds. Past the limit load the wing
 *      keeps flying and keeps a permanent set. Past the ultimate load it goes.
 *   3. That every failure changes a FORCE. A test that only reads a flag proves
 *      nothing, so every failure below is measured on the wrench, on the
 *      control array, or on the engine.
 */

import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import { DEG, G0, kmhToMs, msToKmh } from '@/math/units';
import { createState, createWrench } from '@/physics/rigidbody';
import type { Wrench } from '@/physics/rigidbody';
import {
  BRAKE_FADE_FULL_TEMPERATURE,
  ME262_STATIC_CG_HEIGHT,
  createMe262Gear,
  me262GearLegs,
} from '@/physics/gear';
import { CONTROL_COUNT, CONTROL_INDEX, createMe262Assembly } from '@/aircraft/me262/geometry';
import { FUEL_CAPACITY, LOADED_MASS, MAX_TAKEOFF_MASS, EMPTY_MASS } from '@/aircraft/me262/mass';
import type { LoadLimits, StructureInput } from '@/aircraft/me262/limits';
import {
  AIRFRAME_LIMIT_SPEED,
  FAILED_PANEL_LEFT,
  FIRE_BURN_THROUGH_TIME,
  FIRE_STRENGTH_LOSS,
  LIMIT_LOAD_NEGATIVE,
  LIMIT_LOAD_POSITIVE,
  MAX_LIMIT_SCALE,
  RELIGHT_FIRE_ALTITUDE,
  ULTIMATE_FACTOR,
  ULTIMATE_LOAD_POSITIVE,
  UNPORT_LOW_FUEL_MASS,
  UNPORT_TIME,
  UNUSABLE_FUEL,
  createStructure,
  loadLimits,
  unportLoadFactor,
} from '@/aircraft/me262/limits';
import { createAircraft } from '@/aircraft/aircraft';
import type { AircraftInput } from '@/aircraft/aircraft';

const DT = 1 / 240;

/** One structure input, with the values a settled cruise carries. */
function input(over: Partial<StructureInput> = {}): StructureInput {
  return {
    loadFactor: 1,
    rollRate: 0,
    mass: LOADED_MASS,
    trueAirspeed: 200,
    altitude: 3000,
    fuelMass: FUEL_CAPACITY,
    onGround: false,
    engineFire: [false, false],
    engineLightOff: [false, false],
    tireBurst: [false, false, false],
    brakeTemperature: [288, 288, 288],
    ...over,
  };
}

/** The pilot holds nothing. */
function pilot(over: Partial<AircraftInput> = {}): AircraftInput {
  return {
    roll: 0,
    pitch: 0,
    yaw: 0,
    throttle: 0,
    brakeLeft: 0,
    brakeRight: 0,
    toggleGear: false,
    toggleFlapsUp: false,
    toggleFlapsDown: false,
    startEngines: false,
    ...over,
  };
}

/**
 * Cranks and lights both engines with the procedure of the pilot notes, then
 * runs them up against the brakes. The lever stays closed through the start,
 * exactly as the notes ask.
 */
function startEngines(aircraft: ReturnType<typeof createAircraft>): void {
  const held = pilot({ startEngines: true, brakeLeft: 1, brakeRight: 1 });
  for (let i = 0; i < Math.round(200 / DT); i++) {
    if (aircraft.state.engines.every((e) => e.state === 'idle')) {
      break;
    }
    aircraft.fixedUpdate(held, DT);
  }
  // The notes say to move the lever slowly. A slam from idle surges the
  // compressor and puts the flame out, which src/aircraft/me262/engine.ts
  // already models. The lever therefore takes fifteen seconds.
  const full = pilot({ throttle: 0, brakeLeft: 1, brakeRight: 1 });
  for (let i = 0; i < Math.round(25 / DT); i++) {
    full.throttle = Math.min(1, (i * DT) / 15);
    aircraft.fixedUpdate(full, DT);
  }
}

function emptyLimits(): LoadLimits {
  return { limitPositive: 0, limitNegative: 0, ultimatePositive: 0, ultimateNegative: 0 };
}

describe('the load factor limits of a 1944 fighter', () => {
  it('carries the German stress group H 5 limit of 7 g at the loaded mass', () => {
    const limits = loadLimits(LOADED_MASS, emptyLimits());
    expect(limits.limitPositive).toBeCloseTo(7, 6);
    expect(limits.limitNegative).toBeCloseTo(-3, 6);
  });

  it('puts the ultimate load 1.8 times over the limit load, which is 12.6 g', () => {
    const limits = loadLimits(LOADED_MASS, emptyLimits());
    expect(limits.ultimatePositive / limits.limitPositive).toBeCloseTo(ULTIMATE_FACTOR, 6);
    expect(limits.ultimatePositive).toBeCloseTo(12.6, 6);
    expect(limits.ultimateNegative).toBeCloseTo(-5.4, 6);
    // The IL-2 Great Battles data set gives this aircraft 12.5 g of maximum
    // load factor, which is a breaking value. The two agree to one percent.
    expect(Math.abs(limits.ultimatePositive - 12.5) / 12.5).toBeLessThan(0.01);
  });

  it('holds the same breaking LOAD at every mass, so the limit falls as the aircraft fills', () => {
    const loaded = loadLimits(LOADED_MASS, emptyLimits());
    const heavy = loadLimits(MAX_TAKEOFF_MASS, emptyLimits());
    // A structure carries newtons. The load factor is what changes.
    const loadedForce = loaded.ultimatePositive * LOADED_MASS * G0;
    const heavyForce = heavy.ultimatePositive * MAX_TAKEOFF_MASS * G0;
    expect(heavyForce).toBeCloseTo(loadedForce, 3);
    expect(heavy.limitPositive).toBeLessThan(loaded.limitPositive);
    expect(heavy.limitPositive).toBeCloseTo(6.28, 2);
  });

  it('caps the limit of a light aircraft, because the wing is not the only structure', () => {
    const empty = loadLimits(EMPTY_MASS, emptyLimits());
    // The inverse mass law alone would give 11.8 g at the empty mass.
    expect((LIMIT_LOAD_POSITIVE * LOADED_MASS) / EMPTY_MASS).toBeGreaterThan(11);
    expect(empty.limitPositive).toBeCloseTo(LIMIT_LOAD_POSITIVE * MAX_LIMIT_SCALE, 6);
  });
});

describe('the limit load and the ultimate load do two different things', () => {
  it('leaves the wing whole and permanently bent after one pull past the limit load', () => {
    const structure = createStructure();
    // A pull to 8 g at the loaded mass, held for a quarter of a second.
    for (let i = 0; i < 60; i++) {
      structure.update(input({ loadFactor: 8 }), DT);
    }
    expect(structure.state.wingFailure).toBe('none');
    // The set runs from zero at 7 g to one at 12.6 g, so 8 g uses 18 percent
    // of the band the structure has between yield and failure.
    expect(structure.state.wingStrain).toBeCloseTo(1 / 5.6, 6);
    // The bent wing is weaker, so every later limit is lower than 7 g.
    expect(structure.state.limits.limitPositive).toBeLessThan(LIMIT_LOAD_POSITIVE);
    expect(structure.state.limits.limitPositive).toBeGreaterThan(6);
  });

  it('keeps the permanent set after the load goes away, because a set is permanent', () => {
    const structure = createStructure();
    for (let i = 0; i < 60; i++) {
      structure.update(input({ loadFactor: 8 }), DT);
    }
    const bent = structure.state.wingStrain;
    for (let i = 0; i < 240; i++) {
      structure.update(input({ loadFactor: 1 }), DT);
    }
    expect(structure.state.wingStrain).toBeCloseTo(bent, 12);
  });

  it('reports nothing at all inside the limit load', () => {
    const structure = createStructure();
    for (let i = 0; i < 240; i++) {
      structure.update(input({ loadFactor: 6.5 }), DT);
    }
    expect(structure.state.wingStrain).toBe(0);
    expect(structure.state.wingFailure).toBe('none');
    expect(structure.events.count).toBe(0);
  });

  it('breaks the wing past the ultimate load, on the side the aircraft rolls into', () => {
    const right = createStructure();
    right.update(input({ loadFactor: 13, rollRate: 0.8 }), DT);
    expect(right.state.wingFailure).toBe('right');

    const left = createStructure();
    left.update(input({ loadFactor: 13, rollRate: -0.8 }), DT);
    expect(left.state.wingFailure).toBe('left');
  });

  it('breaks the wing on the negative side at -5.4 g', () => {
    const held = createStructure();
    for (let i = 0; i < 60; i++) {
      held.update(input({ loadFactor: -4 }), DT);
    }
    expect(held.state.wingFailure).toBe('none');
    expect(held.state.wingStrain).toBeGreaterThan(0);

    const broken = createStructure();
    broken.update(input({ loadFactor: -6 }), DT);
    expect(broken.state.wingFailure).toBe('left');
  });

  it('breaks the wing of a full aircraft at a load factor the same aircraft survives light', () => {
    // 10 g at the maximum takeoff mass is past the 8.79 g the wing has left
    // once the first step of that pull has bent it. The same 10 g at the loaded
    // mass only bends the wing, and it settles there.
    const heavy = createStructure();
    const light = createStructure();
    for (let i = 0; i < 240; i++) {
      heavy.update(input({ loadFactor: 10, mass: MAX_TAKEOFF_MASS }), DT);
      light.update(input({ loadFactor: 10, mass: LOADED_MASS }), DT);
    }
    expect(heavy.state.wingFailure).not.toBe('none');
    expect(light.state.wingFailure).toBe('none');
    expect(light.state.wingStrain).toBeGreaterThan(0.5);
  });

  it('walks a wing that is held past the limit load into a failure', () => {
    // 12 g sits under the 12.6 g of the sound wing. The set the first step
    // takes lowers the ultimate load, the next step takes more, and the wing
    // reaches its own weakened ultimate load in a fraction of a second.
    const structure = createStructure();
    let steps = 0;
    while (structure.state.wingFailure === 'none' && steps < 240) {
      structure.update(input({ loadFactor: 12 }), DT);
      steps++;
    }
    expect(structure.state.wingFailure).not.toBe('none');
    expect(steps * DT).toBeLessThan(0.5);
  });

  it('says what happened, in words the pilot can act on', () => {
    const structure = createStructure();
    structure.update(input({ loadFactor: 13 }), DT);
    expect(structure.events.raised).toBe(true);
    expect(structure.events.message).toContain('LEFT WING FAILED');
    expect(structure.events.count).toBe(1);
  });
});

describe('a bent wing and a jammed control change what the stick does', () => {
  it('changes nothing while the airframe is sound', () => {
    const structure = createStructure();
    const controls = new Float64Array(CONTROL_COUNT);
    controls[CONTROL_INDEX.aileron] = 0.25;
    structure.applyControls(controls);
    expect(controls[CONTROL_INDEX.aileron]).toBe(0.25);
  });

  it('takes half the aileron away and leaves a standing deflection after a full set', () => {
    const structure = createStructure();
    // Drive the permanent set to its full value.
    structure.update(input({ loadFactor: 12.5, rollRate: 0.5 }), DT);
    expect(structure.state.wingStrain).toBeGreaterThan(0.9);
    expect(structure.state.strainSide).toBe(1);

    const controls = new Float64Array(CONTROL_COUNT);
    controls[CONTROL_INDEX.aileron] = 0.2;
    structure.applyControls(controls);
    // Half of the command is gone and the bias is on top of what is left.
    expect(controls[CONTROL_INDEX.aileron]).toBeLessThan(0.2);
    expect(controls[CONTROL_INDEX.aileron]).toBeGreaterThan(0.1);

    // With the stick central the aircraft still rolls toward the bent panel.
    const central = new Float64Array(CONTROL_COUNT);
    structure.applyControls(central);
    expect(central[CONTROL_INDEX.aileron]).toBeGreaterThan(0.04);
  });

  it('holds a jammed aileron where it jammed, whatever the pilot asks for', () => {
    const structure = createStructure();
    const controls = new Float64Array(CONTROL_COUNT);
    controls[CONTROL_INDEX.aileron] = 0.3;
    structure.applyControls(controls);

    structure.state.aileronJammed = true;
    for (const command of [0, -0.35, 0.35]) {
      controls[CONTROL_INDEX.aileron] = command;
      structure.applyControls(controls);
      expect(controls[CONTROL_INDEX.aileron]).toBeCloseTo(0.3, 12);
    }
  });
});

describe('the airframe overspeed at the 950 km/h placard', () => {
  it('takes no damage under the placard speed', () => {
    const structure = createStructure();
    for (let i = 0; i < 2400; i++) {
      structure.update(input({ trueAirspeed: kmhToMs(940) }), DT);
    }
    expect(structure.state.overspeed).toBe(0);
    expect(structure.events.count).toBe(0);
  });

  it('warns at once and then builds damage with the excess dynamic pressure', () => {
    const structure = createStructure();
    structure.update(input({ trueAirspeed: kmhToMs(960) }), DT);
    expect(structure.events.message).toContain('AIRFRAME OVERSPEED');
    const slow = structure.state.overspeed;

    const fast = createStructure();
    fast.update(input({ trueAirspeed: kmhToMs(1100) }), DT);
    expect(fast.state.overspeed).toBeGreaterThan(4 * slow);
    expect(msToKmh(AIRFRAME_LIMIT_SPEED)).toBeCloseTo(950, 6);
  });

  it('jams the aileron when the damage is complete, and the aileron then answers nothing', () => {
    const structure = createStructure();
    const controls = new Float64Array(CONTROL_COUNT);
    let steps = 0;
    while (!structure.state.aileronJammed && steps < 240 * 60) {
      controls[CONTROL_INDEX.aileron] = 0.1;
      structure.applyControls(controls);
      structure.update(input({ trueAirspeed: kmhToMs(1200) }), DT);
      steps++;
    }
    expect(structure.state.aileronJammed).toBe(true);
    // A held 1200 km/h destroys the airframe in a few seconds and not at once.
    expect(steps * DT).toBeGreaterThan(2);
    expect(steps * DT).toBeLessThan(20);

    controls[CONTROL_INDEX.aileron] = -0.35;
    structure.applyControls(controls);
    expect(controls[CONTROL_INDEX.aileron]).toBeCloseTo(0.1, 12);
  });
});

describe('a failed wing panel changes the forces and not only a flag', () => {
  it('names the six strips outboard of the nacelle', () => {
    // The engine center line sits at 2.05 m and the boundary between strip 1
    // and strip 2 sits at 2.39 m. The panel is what is outboard of it.
    expect(FAILED_PANEL_LEFT).toEqual([2, 3, 4, 5, 6, 7]);
  });

  it('takes the lift of that panel off the aircraft and rolls it toward the loss', () => {
    const assembly = createMe262Assembly();
    const structure = createStructure();
    const controls = new Float64Array(CONTROL_COUNT);
    const state = createState();
    state.position.set(0, 0, -3000);
    // 5 degrees of angle of attack at 180 m/s, which is a 3 g pull. A wing
    // fails in a pull and not in a glide, so the test measures it in one.
    state.velocity.set(180 * Math.cos(5 * DEG), 0, 180 * Math.sin(5 * DEG));
    const whole: Wrench = createWrench();
    const broken: Wrench = createWrench();
    const wind = new Vector3();

    assembly.evaluateSteady(state, wind, controls, whole);
    structure.update(input({ loadFactor: 13 }), DT);
    expect(structure.state.wingFailure).toBe('left');
    assembly.evaluateSteady(state, wind, controls, broken);
    structure.applyWingFailure(assembly.surfaces, broken);

    // The lift is a negative body z force, so a lost panel makes it smaller.
    // The six strips carry more than a fifth of the lift of the aircraft.
    expect(whole.force.z).toBeLessThan(0);
    expect(broken.force.z).toBeGreaterThan(0.8 * whole.force.z);
    // The whole aircraft is symmetric, so it makes no roll moment at all.
    expect(Math.abs(whole.moment.x)).toBeLessThan(1);
    // The right wing is on its own now. A positive roll moment puts the right
    // wing DOWN, so the loss of the left panel gives a large negative moment.
    // 170 kN m against a roll inertia near 12000 kg m2 is 14 rad/s2.
    expect(broken.moment.x).toBeLessThan(-100000);
  });

  it('does nothing at all while the wing is whole', () => {
    const assembly = createMe262Assembly();
    const structure = createStructure();
    const wrench: Wrench = createWrench();
    wrench.force.set(1, 2, 3);
    wrench.moment.set(4, 5, 6);
    structure.applyWingFailure(assembly.surfaces, wrench);
    expect(wrench.force.toArray()).toEqual([1, 2, 3]);
    expect(wrench.moment.toArray()).toEqual([4, 5, 6]);
  });

  it('rolls the whole aircraft when the panel goes, through fixedUpdate', () => {
    const aircraft = createAircraft();
    aircraft.spawnOnRunway();
    aircraft.state.body.position.set(0, 0, -3000);
    aircraft.state.body.velocity.set(200 * Math.cos(5 * DEG), 0, 200 * Math.sin(5 * DEG));
    for (let i = 0; i < 4; i++) {
      aircraft.fixedUpdate(pilot(), DT);
    }
    expect(Math.abs(aircraft.wrench.moment.x)).toBeLessThan(200);

    // Break the left panel and fly one more step.
    aircraft.structure.update(input({ loadFactor: 13, trueAirspeed: 200 }), DT);
    aircraft.state.body.velocity.set(200 * Math.cos(5 * DEG), 0, 200 * Math.sin(5 * DEG));
    aircraft.state.body.angularVelocity.set(0, 0, 0);
    aircraft.fixedUpdate(pilot(), DT);
    expect(aircraft.structure.state.wingFailure).toBe('left');
    expect(aircraft.wrench.moment.x).toBeLessThan(-100000);
    // The aircraft is now rolling left, which is toward the missing panel, and
    // one step of 1/240 s already gives it a real roll rate.
    expect(aircraft.state.body.angularVelocity.x).toBeLessThan(-0.02);
  });
});

describe('the engine fire and what it costs', () => {
  it('burns through the mount in 25 seconds and then asks for the shutdown', () => {
    const structure = createStructure();
    let time = 0;
    while (!structure.state.engineShutdown[0] && time < 60) {
      structure.update(input({ engineFire: [true, false] }), DT);
      time += DT;
    }
    expect(time).toBeGreaterThan(FIRE_BURN_THROUGH_TIME - 0.1);
    expect(time).toBeLessThan(FIRE_BURN_THROUGH_TIME + 0.1);
    // The other engine is untouched, so the aircraft still flies.
    expect(structure.state.engineShutdown[1]).toBe(false);
    expect(structure.events.message).toContain('LEFT ENGINE LOST');
  });

  it('leaves the wing beside the fire weaker, so the pilot must fly gently', () => {
    const structure = createStructure();
    for (let i = 0; i < Math.ceil((FIRE_BURN_THROUGH_TIME + 1) / DT); i++) {
      structure.update(input({ engineFire: [true, false] }), DT);
    }
    expect(structure.state.panelStrength[0]).toBeCloseTo(1 - FIRE_STRENGTH_LOSS, 6);
    expect(structure.state.limits.ultimatePositive).toBeCloseTo(
      ULTIMATE_LOAD_POSITIVE * (1 - FIRE_STRENGTH_LOSS),
      6,
    );
    // The burned side is the one that goes first, whatever the roll rate says.
    structure.update(input({ loadFactor: 8, rollRate: 0.8 }), DT);
    expect(structure.state.wingFailure).toBe('left');
  });

  it('lights a fire on a relight above 4 km and not below it', () => {
    const high = createStructure();
    high.update(input({ engineLightOff: [false, true], altitude: RELIGHT_FIRE_ALTITUDE + 500 }), DT);
    expect(high.state.fireTime[1]).toBeGreaterThan(0);
    expect(high.events.message).toContain('RIGHT ENGINE FIRE');

    const low = createStructure();
    low.update(input({ engineLightOff: [false, true], altitude: RELIGHT_FIRE_ALTITUDE - 500 }), DT);
    expect(low.state.fireTime[1]).toBe(0);
    expect(low.events.count).toBe(0);
  });

  it('really stops the engine of the aircraft, through fixedUpdate', () => {
    const aircraft = createAircraft();
    aircraft.spawnOnRunway();
    startEngines(aircraft);
    expect(aircraft.state.engines[0].thrust).toBeGreaterThan(1000);

    // The fire burns through. limits.ts runs the handbook drill on the caller.
    aircraft.structure.state.engineShutdown[0] = true;
    for (let i = 0; i < 240; i++) {
      aircraft.fixedUpdate(pilot({ throttle: 1, brakeLeft: 1, brakeRight: 1 }), DT);
    }
    expect(aircraft.state.engines[0].thrust).toBe(0);
    expect(aircraft.state.engines[0].state).toBe('off');
    // The other engine keeps running, which is the whole point.
    expect(aircraft.state.engines[1].thrust).toBeGreaterThan(1000);
  });
});

describe('the fuel feed', () => {
  it('uncovers the pickup at zero g on a low tank and takes the feed away after one second', () => {
    const structure = createStructure();
    let time = 0;
    while (structure.state.fuelAvailable && time < 5) {
      structure.update(input({ loadFactor: -0.1, fuelMass: 100 }), DT);
      time += DT;
    }
    expect(time).toBeGreaterThan(UNPORT_TIME - 0.05);
    expect(time).toBeLessThan(UNPORT_TIME + 0.05);
    expect(structure.events.message).toContain('FUEL FEED LOST');
  });

  it('gives the feed back when the pilot pulls positive g again', () => {
    const structure = createStructure();
    for (let i = 0; i < 480; i++) {
      structure.update(input({ loadFactor: -0.1, fuelMass: 100 }), DT);
    }
    expect(structure.state.fuelAvailable).toBe(false);
    for (let i = 0; i < 240; i++) {
      structure.update(input({ loadFactor: 1, fuelMass: 100 }), DT);
    }
    expect(structure.state.fuelAvailable).toBe(true);
  });

  it('holds the feed through a normal push over that lasts less than a second', () => {
    const structure = createStructure();
    for (let i = 0; i < 120; i++) {
      structure.update(input({ loadFactor: 0.2, fuelMass: 100 }), DT);
    }
    expect(structure.state.fuelAvailable).toBe(true);
  });

  it('never uncovers the pickup of a full tank, whatever the load factor is', () => {
    // A full tank holds fuel against the pump from every side. The dive entry
    // of test/flight/mach.test.ts pushes to -1.34 g for three seconds and the
    // engines must run through all of it.
    const structure = createStructure();
    expect(unportLoadFactor(FUEL_CAPACITY)).toBe(Number.NEGATIVE_INFINITY);
    for (let i = 0; i < 240 * 4; i++) {
      structure.update(input({ loadFactor: -1.34 }), DT);
    }
    expect(structure.state.fuelAvailable).toBe(true);
  });

  it('needs more g to hold the feed as the tanks empty', () => {
    expect(unportLoadFactor(UNPORT_LOW_FUEL_MASS)).toBeCloseTo(0, 6);
    expect(unportLoadFactor(0)).toBeCloseTo(0.5, 6);
    expect(unportLoadFactor(75)).toBeGreaterThan(0.2);
  });

  it('cannot pump the last 40 kg out of the corners of the tanks', () => {
    const structure = createStructure();
    structure.update(input({ fuelMass: UNUSABLE_FUEL - 1 }), DT);
    expect(structure.state.fuelAvailable).toBe(false);
    expect(structure.events.message).toContain('TANKS DRY');
  });

  it('flames the engines out when the feed goes, through fixedUpdate', () => {
    const aircraft = createAircraft();
    aircraft.spawnOnRunway();
    startEngines(aircraft);
    expect(aircraft.state.engines[0].state).toBe('running');
    // Almost dry. The pickup then passes air at any normal load factor.
    aircraft.state.systems.state.fuelMass = UNUSABLE_FUEL - 1;
    for (let i = 0; i < 240 * 3; i++) {
      aircraft.fixedUpdate(pilot({ throttle: 1, brakeLeft: 1, brakeRight: 1 }), DT);
    }
    expect(aircraft.structure.state.fuelAvailable).toBe(false);
    expect(aircraft.state.engines[0].state).toBe('flameout');
    expect(aircraft.state.engines[1].state).toBe('flameout');
  });
});

describe('the tire and the brake of src/physics/gear.ts', () => {
  it('bursts a main tire on an arrival that is far too hard', () => {
    const gear = createMe262Gear();
    const state = createState();
    const wrench: Wrench = createWrench();
    state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
    state.velocity.set(0, 0, 8); // 8 m/s of sink, which no gear survives
    for (let i = 0; i < 60; i++) {
      wrench.force.set(0, 0, 0);
      wrench.moment.set(0, 0, 0);
      gear.update(state, 1, 0, 0, 0, DT, wrench);
      state.position.z += state.velocity.z * DT;
    }
    expect(gear.legs[1].burst || gear.legs[2].burst).toBe(true);
  });

  it('pulls the aircraft toward the burst tire on the roll', () => {
    const wrench: Wrench = createWrench();
    const run = (burstLeft: boolean): number => {
      const gear = createMe262Gear();
      const state = createState();
      state.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
      state.velocity.set(40, 0, 0);
      if (burstLeft) {
        gear.legs[1].burst = true;
      }
      let yaw = 0;
      for (let i = 0; i < 240; i++) {
        wrench.force.set(0, 0, 0);
        wrench.moment.set(0, 0, 0);
        gear.update(state, 1, 0, 0, 0, DT, wrench);
        yaw = wrench.moment.z;
      }
      return yaw;
    };
    const even = run(false);
    const pulling = run(true);
    // A positive yaw moment turns the nose right, so the left tire must give a
    // negative one. The burst leg drags nine times as hard as a whole tire.
    expect(Math.abs(even)).toBeLessThan(100);
    expect(pulling).toBeLessThan(-2000);
  });

  it('tells the pilot about a burst tire and about a brake pack that has faded', () => {
    const structure = createStructure();
    structure.update(input({ onGround: true, tireBurst: [false, true, false] }), DT);
    expect(structure.events.message).toContain('LEFT TIRE BURST');

    structure.update(
      input({
        onGround: true,
        brakeTemperature: [288, BRAKE_FADE_FULL_TEMPERATURE + 5, 288],
      }),
      DT,
    );
    expect(structure.events.message).toContain('BRAKES FADED');
  });

  it('leaves the wing load check to the gear while the aircraft is on the ground', () => {
    // The tire is the fuse on the ground and gear.ts owns it. A load factor
    // that would break the wing in the air breaks nothing on the runway.
    const structure = createStructure();
    structure.update(input({ loadFactor: 13, onGround: true }), DT);
    expect(structure.state.wingFailure).toBe('none');
    expect(structure.state.wingStrain).toBe(0);
  });

  it('keeps the three legs of me262GearLegs in the order this module reads', () => {
    const legs = me262GearLegs();
    expect(legs[0].name).toBe('nose');
    expect(legs[1].position.y).toBeLessThan(0);
    expect(legs[2].position.y).toBeGreaterThan(0);
  });
});

describe('the structure of a whole aircraft', () => {
  it('reports every limit and no damage after a spawn', () => {
    const aircraft = createAircraft();
    aircraft.spawnOnRunway();
    const s = aircraft.structure.state;
    expect(s.wingFailure).toBe('none');
    expect(s.wingStrain).toBe(0);
    expect(s.overspeed).toBe(0);
    expect(s.fuelAvailable).toBe(true);
    expect(s.limits.limitPositive).toBeCloseTo(LIMIT_LOAD_POSITIVE, 6);
    expect(s.limits.limitNegative).toBeCloseTo(LIMIT_LOAD_NEGATIVE, 6);
  });

  it('clears every failure on the next spawn', () => {
    const aircraft = createAircraft();
    aircraft.structure.update(input({ loadFactor: 13 }), DT);
    aircraft.structure.state.aileronJammed = true;
    aircraft.spawnOnRunway();
    expect(aircraft.structure.state.wingFailure).toBe('none');
    expect(aircraft.structure.state.aileronJammed).toBe(false);
    expect(aircraft.structure.events.count).toBe(0);
  });

  it('carries the failure message out on the event bus', () => {
    const aircraft = createAircraft();
    aircraft.spawnOnRunway();
    const seen: string[] = [];
    aircraft.events.on('failure', (event) => seen.push(event.message));
    aircraft.state.body.position.set(0, 0, -3000);
    aircraft.state.body.velocity.set(kmhToMs(1100), 0, 0);
    for (let i = 0; i < 8; i++) {
      aircraft.fixedUpdate(pilot(), DT);
    }
    expect(seen.length).toBeGreaterThan(0);
    expect(seen[0]).toContain('AIRFRAME OVERSPEED');
  });

  it('holds an external wrench for the gun recoil of bead b67', () => {
    // Two aircraft at the same state, so nothing but the recoil differs.
    const clean = createAircraft();
    const firing = createAircraft();
    for (const aircraft of [clean, firing]) {
      aircraft.spawnOnRunway();
      aircraft.state.body.position.set(0, 0, -3000);
      aircraft.state.body.velocity.set(200, 0, 0);
    }
    firing.externalWrench.force.set(-8000, 0, 0);
    firing.externalWrench.moment.set(0, 3000, 0);
    clean.fixedUpdate(pilot(), DT);
    firing.fixedUpdate(pilot(), DT);
    expect(firing.wrench.force.x).toBeCloseTo(clean.wrench.force.x - 8000, 6);
    expect(firing.wrench.moment.y).toBeCloseTo(clean.wrench.moment.y + 3000, 6);
  });
});
