/**
 * The assembled Me 262.
 *
 * These tests check the JOINS, not the parts. Every part already has its own
 * test file. What can only go wrong here is the sum: gravity added twice or not
 * at all, a gear force with the wrong sign, a thrust that makes no yaw moment,
 * a mass that never follows the fuel, or an allocation inside the step.
 */

import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';

import type { Aircraft, AircraftInput } from '@/aircraft/aircraft';
import { createAircraft } from '@/aircraft/aircraft';
import { ENGINE_POSITION_RIGHT } from '@/aircraft/me262/geometry';
import { FUEL_CAPACITY, LOADED_MASS } from '@/aircraft/me262/mass';
import { PHYSICS_DT } from '@/core/loop';
import { G0 } from '@/math/units';
import { ME262_STATIC_CG_HEIGHT } from '@/physics/gear';

const DT = PHYSICS_DT;

/** Static stroke of every leg at the design mass, m. Bead b17 reports it. */
const STATIC_STROKE = 0.154;

function neutral(): AircraftInput {
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
  };
}

function run(a: Aircraft, input: AircraftInput, seconds: number): void {
  const steps = Math.round(seconds / DT);
  for (let i = 0; i < steps; i++) {
    a.fixedUpdate(input, DT);
  }
}

/** Cranks and lights both engines with the procedure of the pilot notes. */
function startEngines(a: Aircraft, input: AircraftInput): void {
  input.startEngines = true;
  input.brakeLeft = 1;
  input.brakeRight = 1;
  const steps = Math.round(200 / DT);
  for (let i = 0; i < steps; i++) {
    if (a.state.engines.every((e) => e.state === 'idle')) break;
    a.fixedUpdate(input, DT);
  }
  input.startEngines = false;
  run(a, input, 5);
}

// ---------------------------------------------------------------------------

describe('the aircraft on its gear', () => {
  it('spawns with the center of gravity at the parked height of the gear model', () => {
    const a = createAircraft();
    expect(a.state.body.position.x).toBe(0);
    expect(a.state.body.position.y).toBe(0);
    expect(-a.state.body.position.z).toBeCloseTo(ME262_STATIC_CG_HEIGHT, 6);
    expect(-a.state.body.position.z).toBeCloseTo(1.1967, 4);
    expect(a.state.mass.mass).toBeCloseTo(LOADED_MASS, 0);
  });

  it('stays inside a few centimeters of the spawn point for 10 seconds', () => {
    // This is the test that catches a gravity sign error, a gear sign error and
    // a double count of either one. An aircraft that sinks, drifts or shakes
    // fails it at once.
    const a = createAircraft();
    const input = neutral();
    const spawn = a.state.body.position.clone();
    run(a, input, 10);
    const moved = a.state.body.position.clone().sub(spawn).length();
    expect(moved).toBeLessThan(0.02);
    expect(a.state.body.velocity.length()).toBeLessThan(0.01);
    expect(a.state.body.angularVelocity.length()).toBeLessThan(0.01);
    expect(a.state.onGround).toBe(true);
  });

  it('reports no angle of attack anywhere while it stands still', () => {
    // Bead b54. Every strip used to report the atan2 of two numbers near zero,
    // which is noise, and the force arrows painted a parked aircraft deep stall
    // red. surface.ts answers zero below its own flow speed now, so the whole
    // aircraft reads zero and the slats stay shut.
    const a = createAircraft();
    const input = neutral();
    run(a, input, 10);
    for (const surface of a.assembly.surfaces) {
      expect(surface.result.alpha).toBe(0);
      expect(surface.result.beta).toBe(0);
      expect(surface.result.slatOpen).toBe(false);
    }
    expect(a.state.totals.alpha).toBe(0);
    expect(a.state.systems.state.slatPosition).toBe(0);
  });

  it('carries the whole weight on the struts, so the total force at rest is zero', () => {
    const a = createAircraft();
    const input = neutral();
    run(a, input, 5);
    const weight = a.state.mass.mass * G0;
    // The wrench holds gravity, the gear and the aerodynamics together. At rest
    // the three must cancel to far below one newton in a 62 kN weight.
    expect(Math.abs(a.wrench.force.z)).toBeLessThan(1e-3 * weight);
    expect(Math.abs(a.wrench.force.x)).toBeLessThan(1e-3 * weight);
    expect(Math.abs(a.wrench.moment.y)).toBeLessThan(1e-3 * weight);
    expect(a.state.loadFactor).toBeCloseTo(1, 6);

    // The three struts share the weight in the ratio the layout fixes, and each
    // one sits at the static stroke of the design.
    let load = 0;
    for (const leg of a.state.gear.legs) {
      expect(leg.onGround).toBe(true);
      expect(leg.compression).toBeCloseTo(STATIC_STROKE, 3);
      load += leg.load;
    }
    expect(load).toBeCloseTo(weight, 0);
  });
});

describe('gravity', () => {
  it('is applied exactly one time, so free fall accelerates at G0', () => {
    const a = createAircraft();
    const input = neutral();
    // High up, at rest, gear down but far from the ground. The airspeed is
    // zero, so the aerodynamics makes no force, and the engines are off, so
    // gravity is the only force left.
    a.state.body.position.set(0, 0, -5000);
    a.state.body.velocity.set(0, 0, 0);
    a.state.body.angularVelocity.set(0, 0, 0);

    a.fixedUpdate(input, DT);
    // One step of a constant acceleration gives exactly a * dt. The stages
    // reach 0.04 m/s, so the drag of the fall costs a few parts in a million.
    expect(a.state.body.velocity.z / DT).toBeCloseTo(G0, 4);
    expect(a.state.body.velocity.x).toBeCloseTo(0, 7);
    expect(a.state.body.velocity.y).toBeCloseTo(0, 7);
    expect(a.state.loadFactor).toBeCloseTo(0, 6);
    expect(a.state.onGround).toBe(false);

    // Half a second later the fall has reached 5 m/s and the drag of the
    // airframe takes 0.4 percent off the acceleration. Nothing else may.
    const before = a.state.body.velocity.z;
    run(a, input, 0.5);
    const mean = (a.state.body.velocity.z - before) / 0.5;
    expect(mean).toBeGreaterThan(0.99 * G0);
    expect(mean).toBeLessThan(G0);
  });

  it('does not double count when the aircraft stands still on the ground', () => {
    // A caller that adds gravity in the source AND lets something else add it
    // reads a load factor of two at rest. This is the cheap check for it.
    const a = createAircraft();
    run(a, neutral(), 2);
    expect(a.state.loadFactor).toBeCloseTo(1, 6);
  });
});

describe('the engines', () => {
  it('makes a yaw moment toward the dead engine when one engine quits', () => {
    const a = createAircraft();
    const input = neutral();
    startEngines(a, input);

    // Away from the ground and at rest in the air, so the gear and the
    // aerodynamics both make nothing and the wrench holds thrust and gravity
    // alone. Gravity acts at the center of gravity and makes no moment.
    a.state.body.position.set(0, 0, -3000);
    a.state.body.velocity.set(0, 0, 0);
    a.state.body.angularVelocity.set(0, 0, 0);
    a.state.body.orientation.copy(new Quaternion());
    input.brakeLeft = 0;
    input.brakeRight = 0;

    // The LEFT engine quits. The right engine then pushes on the right hand
    // side, so the nose swings LEFT, which is a negative yaw moment.
    a.state.engines[0].shutdown();
    a.fixedUpdate(input, DT);
    const thrustRight = a.state.engines[1].thrust;
    expect(a.state.engines[0].thrust).toBe(0);
    expect(thrustRight).toBeGreaterThan(0);
    expect(a.wrench.moment.z).toBeLessThan(0);
    expect(a.wrench.moment.z).toBeCloseTo(-ENGINE_POSITION_RIGHT.y * thrustRight, 6);
    expect(a.wrench.force.x).toBeCloseTo(thrustRight, 6);
  });

  it('makes the mirror image moment when the other engine quits', () => {
    const a = createAircraft();
    const input = neutral();
    startEngines(a, input);
    a.state.body.position.set(0, 0, -3000);
    a.state.body.velocity.set(0, 0, 0);
    a.state.body.angularVelocity.set(0, 0, 0);
    a.state.body.orientation.copy(new Quaternion());
    input.brakeLeft = 0;
    input.brakeRight = 0;

    a.state.engines[1].shutdown();
    a.fixedUpdate(input, DT);
    expect(a.state.engines[1].thrust).toBe(0);
    expect(a.state.engines[0].thrust).toBeGreaterThan(0);
    expect(a.wrench.moment.z).toBeGreaterThan(0);
  });

  it('makes no yaw moment while both engines run together', () => {
    const a = createAircraft();
    const input = neutral();
    startEngines(a, input);
    a.state.body.position.set(0, 0, -3000);
    a.state.body.velocity.set(0, 0, 0);
    a.state.body.angularVelocity.set(0, 0, 0);
    a.state.body.orientation.copy(new Quaternion());
    input.brakeLeft = 0;
    input.brakeRight = 0;
    a.fixedUpdate(input, DT);
    expect(a.wrench.moment.z).toBeCloseTo(0, 9);
  });
});

describe('the fuel', () => {
  it('burns off, which drops the mass and moves the center of gravity', () => {
    const a = createAircraft();
    const input = neutral();
    startEngines(a, input);
    input.brakeLeft = 1;
    input.brakeRight = 1;

    const fuelBefore = a.state.systems.state.fuelMass;
    const massBefore = a.state.mass.mass;
    const cgBefore = a.state.mass.cgFromNose;
    expect(fuelBefore).toBeGreaterThan(0);
    expect(fuelBefore).toBeLessThanOrEqual(FUEL_CAPACITY);

    // Two minutes at the idle stop, which burns a few kilograms.
    run(a, input, 120);

    expect(a.state.systems.state.fuelMass).toBeLessThan(fuelBefore);
    expect(a.state.mass.mass).toBeLessThan(massBefore);
    // The mass properties are rebuilt every MASS_UPDATE_FUEL kilograms, so the
    // mass follows the fuel to inside that step.
    const burned = fuelBefore - a.state.systems.state.fuelMass;
    expect(Math.abs(a.state.mass.mass - (massBefore - burned))).toBeLessThan(2.5);
    // The rear auxiliary tank sits at station 7.8 m, well behind the center of
    // gravity at 5.76 m, and the fuel system empties it first. The balance
    // therefore moves FORWARD as the fuel goes.
    expect(a.state.mass.cgFromNose).toBeLessThan(cgBefore);
  });
});

describe('the step', () => {
  it('allocates nothing, so every object a caller reads keeps its identity', () => {
    // Verified by construction as well: fixedUpdate holds no `new`, no object
    // literal and no array literal. me262Mass does allocate, and the aircraft
    // calls it only when the fuel has moved by two kilograms, which no engine
    // off run reaches. This test holds the identity property, because a fresh
    // allocation would break every reference the render code keeps.
    const a = createAircraft();
    const input = neutral();
    const state = a.state;
    const body = state.body;
    const position = body.position;
    const velocity = body.velocity;
    const orientation = body.orientation;
    const angularVelocity = body.angularVelocity;
    const totals = state.totals;
    const perSurface = totals.perSurface;
    const inertia = state.mass.inertia;
    const controls = a.controls;
    const wrench = a.wrench;
    const force = wrench.force;
    const atmosphere = a.atmosphere;
    const samples = a.assembly.sampleForDebug();

    for (let i = 0; i < 1000; i++) {
      a.fixedUpdate(input, DT);
    }

    expect(a.state).toBe(state);
    expect(a.state.body).toBe(body);
    expect(a.state.body.position).toBe(position);
    expect(a.state.body.velocity).toBe(velocity);
    expect(a.state.body.orientation).toBe(orientation);
    expect(a.state.body.angularVelocity).toBe(angularVelocity);
    expect(a.state.totals).toBe(totals);
    expect(a.state.totals.perSurface).toBe(perSurface);
    expect(a.state.mass.inertia).toBe(inertia);
    expect(a.controls).toBe(controls);
    expect(a.wrench).toBe(wrench);
    expect(a.wrench.force).toBe(force);
    expect(a.atmosphere).toBe(atmosphere);
    expect(a.assembly.sampleForDebug()).toBe(samples);
    expect(Number.isFinite(a.state.body.position.z)).toBe(true);
  });

  it('writes the pilot commands into the control array that the assembly reads', () => {
    const a = createAircraft();
    const input = neutral();
    input.roll = 1;
    input.pitch = -0.5;
    input.yaw = 0.25;
    a.fixedUpdate(input, DT);
    // CONTROL_INDEX: aileron 0, elevator 1, rudder 2, flap 3, slat 4.
    expect(a.controls[0]).toBeGreaterThan(0);
    expect(a.controls[1]).toBeLessThan(0);
    expect(a.controls[2]).toBeGreaterThan(0);
    // The flap starts up and the slat starts shut.
    expect(a.controls[3]).toBe(0);
    expect(a.controls[4]).toBe(0);
  });

  it('spawns back onto the threshold after a flight', () => {
    const a = createAircraft();
    const input = neutral();
    a.state.body.position.set(1000, 200, -2000);
    a.state.body.velocity.set(180, 4, -6);
    run(a, input, 1);
    a.spawnOnRunway();
    expect(a.state.body.position.x).toBe(0);
    expect(a.state.body.position.y).toBe(0);
    expect(-a.state.body.position.z).toBeCloseTo(ME262_STATIC_CG_HEIGHT, 6);
    expect(a.state.body.velocity.length()).toBe(0);
    expect(a.state.systems.state.fuelMass).toBe(FUEL_CAPACITY);
    expect(a.state.systems.state.gearPosition).toBe(1);
    expect(a.state.systems.state.flapPosition).toBe(0);
    for (const engine of a.state.engines) {
      expect(engine.state).toBe('off');
      expect(engine.thrust).toBe(0);
    }
    const spawn = new Vector3().copy(a.state.body.position);
    run(a, input, 5);
    expect(a.state.body.position.clone().sub(spawn).length()).toBeLessThan(0.02);
  });
});
