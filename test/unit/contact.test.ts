/**
 * The airframe against the ground, and what the model does when it fails.
 *
 * Bead b53. Before this file existed the only part of the aircraft that the
 * ground could push on was the tire contact patch. An arrival that carried the
 * center of gravity below the ground plane drove the gear far past its hard
 * stop, which is 4 MN/m of structure, and the integrator diverged. Fifty meters
 * under the runway, at rest, the state held no number at all after 5 steps.
 *
 * Three things stop that now, and each test below holds one of them.
 */

import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';

import type { AircraftInput } from '@/aircraft/aircraft';
import { createAircraft } from '@/aircraft/aircraft';
import { PHYSICS_DT } from '@/core/loop';
import { G0 } from '@/math/units';
import { MAX_GROUND_LOAD_FACTOR, limitContactWrench } from '@/physics/contact';
import {
  ME262_STATIC_CG_HEIGHT,
  createMe262Gear,
  me262ContactPoints,
  me262GearLegs,
} from '@/physics/gear';
import { clearWrench, createState, createWrench } from '@/physics/rigidbody';
import type { RigidBodyState } from '@/physics/rigidbody';

const DT = PHYSICS_DT;

/** Loaded mass and weight of the Me 262 A-1a. CONVENTIONS section 8, firm. */
const MASS = 6396; // kg
const WEIGHT = MASS * G0; // N

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

/** True while every number of one state is finite. */
function finite(s: RigidBodyState): boolean {
  return [
    s.position.x,
    s.position.y,
    s.position.z,
    s.velocity.x,
    s.velocity.y,
    s.velocity.z,
    s.orientation.x,
    s.orientation.y,
    s.orientation.z,
    s.orientation.w,
    s.angularVelocity.x,
    s.angularVelocity.y,
    s.angularVelocity.z,
  ].every((v) => Number.isFinite(v));
}

interface CrashResult {
  /** The step at which the state first held a number that is not finite, or -1. */
  firstBadStep: number;
  /** The largest distance from the runway threshold the aircraft ever reached, m. */
  peakDistance: number;
  /** The largest speed it ever reached, m/s. */
  peakSpeed: number;
  diverged: boolean;
}

/**
 * Puts the aircraft in one state and steps it, with no pilot input at all.
 *
 * The whole aircraft runs, not the gear alone: the aerodynamics, the engines and
 * the mass model all see the same states, and the original divergence ran
 * through all of them.
 */
function crash(place: (a: ReturnType<typeof createAircraft>) => void, seconds: number): CrashResult {
  const a = createAircraft();
  a.spawnOnRunway();
  place(a);
  const input = neutral();
  const steps = Math.round(seconds / DT);
  const out: CrashResult = { firstBadStep: -1, peakDistance: 0, peakSpeed: 0, diverged: false };
  for (let i = 0; i < steps; i++) {
    a.fixedUpdate(input, DT);
    if (!finite(a.state.body)) {
      out.firstBadStep = i;
      break;
    }
    out.peakDistance = Math.max(out.peakDistance, a.state.body.position.length());
    out.peakSpeed = Math.max(out.peakSpeed, a.state.body.velocity.length());
  }
  out.diverged = a.state.diverged;
  return out;
}

// ---------------------------------------------------------------------------
// Where the points sit
// ---------------------------------------------------------------------------

describe('the airframe contact points', () => {
  const points = me262ContactPoints();
  const byName = new Map(points.map((p) => [p.name, p.position]));

  it('covers the nose, the belly, both nacelles, both wing tips and the tail', () => {
    expect(points.map((p) => p.name)).toEqual([
      'nose',
      'belly',
      'nacelle left',
      'nacelle right',
      'wing tip left',
      'wing tip right',
      'tail',
    ]);
  });

  it('stands every point clear of the runway while the aircraft parks', () => {
    // The tire contact patch sits 1.1967 m below the center of gravity, and the
    // center of gravity parks at that height. Any airframe point deeper than
    // that would hold the aircraft off its own wheels.
    for (const point of points) {
      expect(point.position.z).toBeLessThan(ME262_STATIC_CG_HEIGHT);
    }
    const a = createAircraft();
    const input = neutral();
    for (let i = 0; i < Math.round(5 / DT); i++) {
      a.fixedUpdate(input, DT);
    }
    expect(a.state.contacts.anyOnGround).toBe(false);
    for (const state of a.state.contacts.points) {
      expect(state.onGround).toBe(false);
      expect(state.load).toBe(0);
    }
  });

  it('puts the nacelles lowest, so a gear up arrival lands on them first', () => {
    // This is what the Me 262 really did. The two engines hang below the wing
    // and they take a belly landing before the fuselage reaches the ground.
    const nacelle = byName.get('nacelle left') as Vector3;
    const belly = byName.get('belly') as Vector3;
    expect(nacelle.z).toBeGreaterThan(belly.z);
    for (const point of points) {
      expect(point.position.z).toBeLessThanOrEqual(nacelle.z);
    }
    // Clear of the runway by 0.37 m with the gear down and the struts loaded.
    expect(ME262_STATIC_CG_HEIGHT - nacelle.z).toBeCloseTo(0.375, 2);
  });

  it('holds the aircraft up when it lies on its belly with no gear', () => {
    // Gear up, on the ground. The airframe points are the only thing left.
    const a = createAircraft();
    a.spawnOnRunway();
    a.state.systems.state.gearPosition = 0;
    a.state.systems.commandGear(false);
    a.state.body.position.set(0, 0, -0.9);
    const input = neutral();
    for (let i = 0; i < Math.round(10 / DT); i++) {
      a.fixedUpdate(input, DT);
    }
    expect(finite(a.state.body)).toBe(true);
    // It rests on the nacelles and it does not sink into the runway.
    expect(a.state.contacts.anyOnGround).toBe(true);
    expect(a.state.onGround).toBe(true);
    expect(Math.abs(a.state.body.velocity.z)).toBeLessThan(0.05);
    const nacelleLoad =
      a.state.contacts.points[2].load + a.state.contacts.points[3].load;
    expect(nacelleLoad).toBeGreaterThan(0.3 * WEIGHT);
  });

  it('scrapes a belly landing to a stop instead of gliding on', () => {
    // The friction of metal on concrete is 0.7, so a slide decelerates at about
    // that many g. A model with no airframe friction would coast for ever.
    const a = createAircraft();
    a.spawnOnRunway();
    a.state.systems.state.gearPosition = 0;
    a.state.systems.commandGear(false);
    a.state.body.position.set(0, 0, -0.9);
    a.state.body.velocity.set(50, 0, 0);
    const input = neutral();
    for (let i = 0; i < Math.round(20 / DT); i++) {
      a.fixedUpdate(input, DT);
    }
    expect(finite(a.state.body)).toBe(true);
    expect(a.state.body.velocity.x).toBeLessThan(1);
    // 50 m/s at about half a g gives a slide of a few hundred meters.
    expect(a.state.body.position.x).toBeGreaterThan(100);
    expect(a.state.body.position.x).toBeLessThan(600);
  });
});

// ---------------------------------------------------------------------------
// The bound on the force
// ---------------------------------------------------------------------------

describe('the ground force cap', () => {
  it('scales the force and the moment together and leaves a small wrench alone', () => {
    const w = createWrench();
    w.force.set(300, 400, 0);
    w.moment.set(0, 1000, 0);
    expect(limitContactWrench(w, 1000)).toBe(1);
    expect(w.force.x).toBe(300);
    expect(limitContactWrench(w, 250)).toBeCloseTo(0.5, 12);
    expect(w.force.length()).toBeCloseTo(250, 9);
    // The moment takes the same factor, because a contact moment is r x F.
    expect(w.moment.y).toBeCloseTo(500, 9);
  });

  it('never lets the gear ask for more than the airframe could survive', () => {
    // Fifty meters under the runway the gas spring and the 4 MN/m hard stop
    // together ask for 600 MN, which is ten thousand times the weight. No fixed
    // step can integrate that.
    const gear = createMe262Gear();
    const state = createState();
    const wrench = createWrench();
    state.position.set(0, 0, 50);
    clearWrench(wrench);
    gear.update(state, 1, 0, 0, 0, DT, wrench);
    expect(wrench.force.length()).toBeLessThanOrEqual(MAX_GROUND_LOAD_FACTOR * WEIGHT * 1.0001);
    expect(wrench.force.length()).toBeCloseTo(MAX_GROUND_LOAD_FACTOR * WEIGHT, 0);
    // The legs still report the load they really carry, so the tires burst.
    expect(gear.legs[1].load).toBeGreaterThan(MAX_GROUND_LOAD_FACTOR * WEIGHT);
  });

  it('clips nothing on the hardest arrival the gear survives', () => {
    // A 7 m/s drop from the height where the tires just touch bottoms both
    // struts and bursts both main tires. It peaks at 11.7 g, which is inside the
    // cap, so the cap changes no arrival that the gear model still answers for.
    const legs = me262GearLegs();
    const touchdown = legs[1].position.z + legs[1].restLength + legs[1].wheelRadius;
    const gear = createMe262Gear();
    const state = createState();
    const wrench = createWrench();
    const acceleration = new Vector3();
    state.position.set(0, 0, -(touchdown + 0.002));
    state.velocity.set(0, 0, 7);
    let peak = 0;
    for (let i = 0; i < Math.round(1 / DT); i++) {
      clearWrench(wrench);
      gear.update(state, 1, 0, 0, 0, DT, wrench);
      peak = Math.max(peak, wrench.force.length());
      wrench.force.z += WEIGHT;
      acceleration.copy(wrench.force).multiplyScalar(1 / MASS);
      state.velocity.addScaledVector(acceleration, DT);
      state.position.addScaledVector(state.velocity, DT);
    }
    expect(gear.legs[1].burst).toBe(true);
    expect(peak / WEIGHT).toBeGreaterThan(8);
    expect(peak / WEIGHT).toBeLessThan(MAX_GROUND_LOAD_FACTOR);
  });
});

// ---------------------------------------------------------------------------
// The three crashes that used to kill the simulator
// ---------------------------------------------------------------------------

describe('a hard crash leaves the state finite', () => {
  it('survives the aircraft placed 50 m under the runway at rest', () => {
    // The original report, step for step. The state held no number at all after
    // 24 steps, and the simulator was dead until the page reloaded.
    const result = crash((a) => {
      a.state.body.position.set(0, 0, 50);
    }, 5);
    expect(result.firstBadStep).toBe(-1);
    expect(result.peakSpeed).toBeLessThan(1000);
    expect(result.peakDistance).toBeLessThan(1e4);
  });

  it('survives an inverted impact', () => {
    // Upside down, 5 m up, coming down at 40 m/s. The wing tips and the tail
    // reach the ground first and the gear is pointing at the sky.
    const result = crash((a) => {
      a.state.body.position.set(0, 0, -5);
      a.state.body.velocity.set(60, 0, 40);
      a.state.body.orientation.setFromAxisAngle(new Vector3(1, 0, 0), Math.PI);
      a.state.body.angularVelocity.set(2, 0, 0);
    }, 10);
    expect(result.firstBadStep).toBe(-1);
    expect(result.peakSpeed).toBeLessThan(1000);
  });

  it('survives a very high sink rate onto the wheels', () => {
    // 150 m/s straight down is five times the rate that bursts both main tires.
    const result = crash((a) => {
      a.state.body.position.set(0, 0, -3);
      a.state.body.velocity.set(0, 0, 150);
    }, 5);
    expect(result.firstBadStep).toBe(-1);
    expect(result.peakSpeed).toBeLessThan(1000);
  });

  it('survives a nose down dive into the runway at 250 m/s', () => {
    const result = crash((a) => {
      a.state.body.position.set(0, 0, -20);
      a.state.body.orientation.setFromAxisAngle(new Vector3(0, 1, 0), -Math.PI / 3);
      a.state.body.velocity.set(125, 0, 216);
    }, 5);
    expect(result.firstBadStep).toBe(-1);
  });
});

// ---------------------------------------------------------------------------
// The guard of last resort
// ---------------------------------------------------------------------------

describe('the divergence guard', () => {
  it('holds the aircraft and reports when a step produces a value that is not finite', () => {
    const a = createAircraft();
    const input = neutral();
    const reports: string[] = [];
    a.events.on('diverged', (event) => reports.push(event.message));

    // Force the failure the guard exists for. Nothing the model does now
    // reaches this state, and the guard has to work when something does.
    a.state.body.velocity.x = Number.NaN;
    const held = a.state.body.position.clone();
    a.fixedUpdate(input, DT);

    expect(a.state.diverged).toBe(true);
    expect(reports.length).toBe(1);
    expect(reports[0]).toContain('Press R');
    // It holds the aircraft at the last state that was finite, at rest.
    expect(finite(a.state.body)).toBe(true);
    expect(a.state.body.position.distanceTo(held)).toBeLessThan(1e-9);
    expect(a.state.body.velocity.length()).toBe(0);
  });

  it('does nothing at all while it is held, and flies again after a spawn', () => {
    const a = createAircraft();
    const input = neutral();
    let reports = 0;
    a.events.on('diverged', () => {
      reports += 1;
    });
    a.state.body.angularVelocity.y = Number.POSITIVE_INFINITY;
    a.fixedUpdate(input, DT);
    const held = a.state.body.position.clone();

    // A held aircraft ignores the pilot and reports one time, not once a step.
    input.throttle = 1;
    for (let i = 0; i < 100; i++) {
      a.fixedUpdate(input, DT);
    }
    expect(reports).toBe(1);
    expect(a.state.body.position.distanceTo(held)).toBe(0);

    // R calls spawnOnRunway, and the aircraft flies again.
    a.spawnOnRunway();
    expect(a.state.diverged).toBe(false);
    for (let i = 0; i < Math.round(5 / DT); i++) {
      a.fixedUpdate(neutral(), DT);
    }
    expect(finite(a.state.body)).toBe(true);
    expect(-a.state.body.position.z).toBeCloseTo(ME262_STATIC_CG_HEIGHT, 2);
    expect(a.state.onGround).toBe(true);
  });
});
