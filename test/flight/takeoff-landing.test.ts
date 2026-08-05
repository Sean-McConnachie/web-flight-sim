/**
 * Takeoff ground run, and the landing roll.
 *
 * THE MASS. The published takeoff run of 1100 m belongs to the maximum takeoff
 * mass of 7130 kg. The mass model of src/aircraft/me262/mass.ts carries the
 * empty aircraft, the pilot, the ammunition, the equipment and the internal
 * fuel, and that sum reaches 6396 kg. The remaining 734 kg of the published
 * maximum are external stores, and no bead has added them, so the model cannot
 * be loaded to 7130 kg at all.
 *
 * The test therefore measures at 6396 kg and scales the target. A takeoff run
 * follows the square of the weight: the lift off speed follows the square root
 * of the weight, the distance follows the square of that speed, and the
 * acceleration follows one over the weight, so
 *
 *   s(W) = s(W_ref) * (W / W_ref)^2
 *
 * which gives 1100 * (6396 / 7130)^2 = 885 m. Bead b25 measured 603 to 716 m
 * against that scaled value.
 *
 * THE PROCEDURE. Brakes on, the throttle advanced at the rate the surge margin
 * allows, and the rotor given time to settle. Then the brakes come off and the
 * distance runs from that point. The elevator stays neutral until the rotation
 * speed, which the test takes as 1.1 times the trimmed stall speed with the
 * takeoff flap. The run ends when the last wheel leaves the ground.
 *
 * The landing roll has no published target in the reference set. The test
 * measures a MAXIMUM EFFORT stop: full brakes one second after the touch down.
 * Bead b33 re-specified the target, because the first estimate read the long
 * landing runs of the type as a maximum effort stop, and the two are different
 * measurements. See the note above the record call.
 */

import { afterAll, describe, expect, it } from 'vitest';

import { trimForAlpha } from '@/aircraft/trim';
import type { TrimCondition, TrimResult } from '@/aircraft/trim';
import { FUEL_CAPACITY, MAX_TAKEOFF_MASS } from '@/aircraft/me262/mass';
import { DEG, kmhToMs, msToKmh, toDeg } from '@/math/units';
import {
  DT,
  createFlightTest,
  note,
  passed,
  placeOnRunway,
  printReport,
  record,
} from './harness';

/** Takeoff run at 7130 kg. CONVENTIONS section 8, firm. */
const PUBLISHED_RUN = 1100; // m
/** The mass the model can really carry. */
const MODEL_MASS = 6396; // kg
/** The published run, scaled to the mass the model can carry. */
const SCALED_RUN = PUBLISHED_RUN * (MODEL_MASS / MAX_TAKEOFF_MASS) ** 2; // 885 m

/** Touch-down speed. CONVENTIONS section 8, firm, and it is NOT a stall speed. */
const TOUCHDOWN_SPEED = kmhToMs(175); // m/s

const base: TrimCondition = {
  altitude: 0,
  speed: 70,
  flapSetting: 'takeoff',
  gearDown: true,
  fuelMass: FUEL_CAPACITY,
};

/** The lowest speed a level 1 g trim exists at, with one flap setting. */
function trimmedStall(condition: TrimCondition): TrimResult {
  let best: TrimResult | null = null;
  for (let deg = 8; deg <= 24; deg += 0.25) {
    const r = trimForAlpha(condition, deg * DEG);
    if (r.converged && (best === null || r.speed < best.speed)) {
      best = r;
    }
  }
  if (best === null) {
    throw new Error('No 1 g trim converged for the rotation speed.');
  }
  return best;
}

afterAll(() => {
  printReport('TAKEOFF AND LANDING');
});

describe('the takeoff run', () => {
  it('lifts off inside the published ground run', () => {
    const stall = trimmedStall(base);
    const rotate = 1.1 * stall.speed;
    note(
      `takeoff flap stall ${msToKmh(stall.speed).toFixed(1)} km/h, ` +
        `rotation at ${msToKmh(rotate).toFixed(1)} km/h`,
    );

    const test = createFlightTest();
    placeOnRunway(test, 'takeoff');
    const input = test.input;
    input.brakeLeft = 1;
    input.brakeRight = 1;
    input.pitch = 0;
    input.roll = 0;
    input.yaw = 0;

    // The lever moves at the rate the pilot notes allow and the rotor is given
    // time to settle before the brakes come off. A Jumo 004 needs 8 to 10 s from
    // idle to full power, and a takeoff that starts before that is a takeoff of
    // a different aircraft.
    for (let i = 0; i < Math.round(25 / DT); i++) {
      input.throttle = Math.min(1, input.throttle + 0.1 * DT);
      test.flyOpenLoop(DT);
    }
    const engines = test.aircraft.state.engines;
    note(
      `at the brake release: rpm ${engines.map((e) => e.rpm.toFixed(0)).join('/')}, ` +
        `thrust ${engines.map((e) => e.thrust.toFixed(0)).join('/')} N, ` +
        `mass ${test.aircraft.state.mass.mass.toFixed(0)} kg`,
    );
    for (const engine of engines) {
      expect(engine.state).toBe('running');
      // The spool costs a little creep damage, because the acceleration schedule
      // runs the turbine close to its limit. Anything above one percent means
      // the lever moved too fast and the measurement is of a damaged engine.
      expect(engine.damage).toBeLessThan(0.01);
    }

    input.brakeLeft = 0;
    input.brakeRight = 0;
    const startX = test.aircraft.state.body.position.x;
    let liftOffDistance = 0;
    let liftOffSpeed = 0;
    let rotateDistance = 0;
    const limit = Math.round(120 / DT);
    for (let i = 0; i < limit; i++) {
      const s = test.sample();
      // The elevator is neutral until the rotation speed and then goes back to
      // raise the nose. The value is what a pilot uses: enough to rotate in
      // about three seconds, not enough to over rotate.
      if (s.speed >= rotate) {
        if (rotateDistance === 0) {
          rotateDistance = s.distance - startX;
        }
        input.pitch = 0.45;
      }
      // The rudder steers the nose wheel. The run is symmetric, so it stays at
      // zero, and the aileron holds the wings level against nothing.
      test.flyOpenLoop(DT);
      if (!test.aircraft.state.onGround && test.sample().speed > 40) {
        liftOffDistance = test.sample().distance - startX;
        liftOffSpeed = test.sample().speed;
        break;
      }
    }
    note(
      `rotation at ${rotateDistance.toFixed(0)} m, lift off at ${liftOffDistance.toFixed(0)} m ` +
        `and ${msToKmh(liftOffSpeed).toFixed(1)} km/h, ` +
        `pitch ${toDeg(test.sample().pitch).toFixed(2)} deg`,
    );
    expect(liftOffDistance).toBeGreaterThan(0);

    const m = record({
      name: 'takeoff ground run',
      measured: liftOffDistance,
      target: SCALED_RUN,
      tolerance: 0.1,
      toleranceKind: 'fraction',
      unit: 'm',
      note:
        `at ${MODEL_MASS} kg. The published ${PUBLISHED_RUN} m is at ${MAX_TAKEOFF_MASS} kg, ` +
        'scaled by the square of the weight. The model cannot be loaded past its internal fuel.',
    });
    record({
      name: 'lift off speed',
      measured: msToKmh(liftOffSpeed),
      target: msToKmh(1.15 * stall.speed),
      tolerance: 0.1,
      toleranceKind: 'fraction',
      unit: 'km/h',
      note: 'no published value. 1.15 times the stall speed is the usual lift off speed.',
    });
    expect(passed(m)).toBe(true);
  });
});

describe('the landing roll', () => {
  it('stops from the touch-down speed', () => {
    const test = createFlightTest();
    placeOnRunway(test, 'landing');
    const body = test.aircraft.state.body;
    body.velocity.set(TOUCHDOWN_SPEED, 0, 0);
    const input = test.input;
    input.throttle = 0;
    input.pitch = 0;
    input.roll = 0;
    input.yaw = 0;
    input.brakeLeft = 0;
    input.brakeRight = 0;

    const startX = body.position.x;
    // One second on the runway before the brakes go on, which is the time the
    // nose wheel takes to come down after a touch down.
    test.flyOpenLoop(1);
    input.brakeLeft = 1;
    input.brakeRight = 1;
    let distance = 0;
    for (let i = 0; i < Math.round(120 / DT); i++) {
      test.flyOpenLoop(DT);
      const s = test.sample();
      if (s.speed < 1) {
        distance = s.distance - startX;
        break;
      }
    }
    const legs = test.aircraft.state.gear.legs;
    note(
      `landing roll ${distance.toFixed(0)} m from ${msToKmh(TOUCHDOWN_SPEED).toFixed(0)} km/h, ` +
        `brake temperature ${legs.map((l) => l.brakeTemp.toFixed(0)).join('/')} K, ` +
        `tires ${legs.map((l) => (l.burst ? 'burst' : 'whole')).join('/')}`,
    );
    expect(distance).toBeGreaterThan(0);

    // BEAD b33 RE-SPECIFIED THIS TARGET. IT WAS 800 m WITH A BAND OF 400 m.
    //
    // No landing roll appears in the reference set, so both the old target and
    // the new one are estimates. The old one took the long landing runs the
    // type is known for and read them as a maximum effort stop. THE TWO ARE NOT
    // THE SAME MEASUREMENT, in the same way that the touch-down speed of
    // CONVENTIONS section 8 is not a stall speed.
    //
    // A roll of 800 m from 48.6 m/s is a mean deceleration of 0.15 g, which is
    // 9.4 kN of braking force, which is 1975 N m at each main wheel. The pilot
    // notes ask the SAME BRAKES to hold the aircraft against 17.6 kN of thrust
    // during the run up, and that needs 3700 N m. One brake cannot give half of
    // what it is documented to hold. The old target and the run up contradict
    // each other, and the run up is the documented one.
    //
    // The band therefore comes from the brake the run up measures. See
    // MAX_BRAKE_TORQUE of src/physics/gear.ts.
    //
    //   cold  2 * 4200 / 0.42 = 20.0 kN, which is 0.32 g, and the airframe drag
    //         and the rolling resistance take it to 0.36 g. That is 334 m.
    //   hot   the pack takes 3.6 MJ over the roll, reaches 541 K and gives 0.87
    //         of its cold torque, which is 0.29 g. That is 415 m.
    //   plus  about 49 m of free roll, while the nose wheel comes down.
    //
    // So a maximum effort roll runs 383 to 464 m. The band below is wider than
    // that on both sides. Under 300 m the brake beats the tire, the wheels lock
    // and the aircraft skids, which is what the model did before bead b33. Over
    // 540 m the brake is too weak to hold the run up.
    record({
      name: 'landing roll from touch down',
      measured: distance,
      target: 420,
      tolerance: 120,
      toleranceKind: 'absolute',
      unit: 'm',
      note:
        'ESTIMATE, not published, re-specified by bead b33. Full brakes from ' +
        '175 km/h on dry concrete. The band comes from the brake that holds the ' +
        'documented run up, cold at 0.36 g and faded at 0.29 g.',
    });
  });
});
