/**
 * Compressibility: the Mach tuck, the loss of elevator power, and the limit.
 *
 * The Me 262 was the first fighter to meet compressibility every day. The
 * documented tuck onset is Mach 0.83 and the documented limit is Mach 0.86.
 *
 *
 * WHAT THE RECORD REALLY SAYS. READ THIS BEFORE YOU CHANGE A TARGET.
 *
 * THE PILOT HAD NO MACHMETER. The 0.83 and 0.86 pair comes from the Messerschmitt
 * dive program, not from any pilot document. Gerd Lindner flew the research dives
 * and reached 1004 km/h at 4300 m, which is Mach 0.862 in the standard
 * atmosphere. Messerschmitt found the aircraft went increasingly nose heavy above
 * Mach 0.83, and that Mach 0.86 was the highest dive a pilot could still pull out
 * of. The pilot met the same limit as an airspeed: the Wright Field handbook
 * placards 950 km/h, and the Me 262 airspeed indicator is altitude compensated,
 * so above 400 km/h it reads true airspeed. At 6000 m that placard IS Mach 0.834.
 * Source: Radinger and Schick, "Me 262", 1996, through the Lindner dive program.
 * Confidence: probable, with the 1004 km/h at 4300 m as an independent check.
 *
 * THE AIRCRAFT WAS RECOVERABLE AT ITS LIMIT, AND IT KILLED PILOTS PAST IT. Both
 * halves are on the record and both have to appear here. Lindner flew a few dives
 * to Mach 0.86 and came back. Past it the nose down trim grew past what the pilot
 * could hold, the dive steepened on its own, and the airframe came apart. Hans Fey
 * reported losing cockpit covers and bomb racks in steep fast dives, and Mutke
 * watched rivets leave the upper surface of the wing.
 * Source: Fey debriefing, spring 1945, and Dorr, "Fighting Hitler's Jets", 2013,
 * page 231. Confidence: firm for the claim, weak for any named loss.
 *
 * THE TRIM WHEEL STORY IS HALF RIGHT, AND THE HANDBOOK CONTRADICTS THE OTHER HALF.
 * The aerodynamics is clear that the moving tailplane is what still works at high
 * Mach: NACA RM L50G20 measured the elevator falling from 0.25 of the stabilizer
 * effectiveness at Mach 0.78 to 0.05 at Mach 1.0, and half a degree of stabilizer
 * covered the whole transonic trim change. Mutke recovered by closing the
 * throttle and changing the tailplane incidence. BUT NO ME 262 DOCUMENT TELLS THE
 * PILOT TO DO IT. The Wright Field handbook says the opposite: "The stabilizer
 * trim control should not be used when going into or pulling out of a dive",
 * because the electric lever is coarse and "it is easy to over-control, causing
 * the ship to nose over or pull up more abruptly than intended". The same handbook
 * says "No high-speed dives should be run". So this file tests what the aircraft
 * DID, and it does not claim a procedure the documents do not carry.
 * Source: "Me 262 A-1 Pilot's Handbook", Air Materiel Command, Wright Field,
 * F-SU-1111-ND, 10 January 1946. Confidence: firm, quoted.
 *
 * WHAT THE PILOT FELT WAS STICK FORCE, NOT A CONTROL THAT HAD STOPPED WORKING.
 * The one number in the record for the Me 262 is that past Mach 0.83 the elevator
 * PULL force rose to about 100 lb at Mach 0.86 merely to HOLD the dive angle,
 * with violent buffet. That is the tuck: the aircraft was going nose down and the
 * pilot was fighting it. This model has no hinge moment, so it cannot report a
 * stick force. It reports the elevator DEFLECTION that stands for it, and the
 * held dive at the end of this file is the test that measures it.
 * Source: attributed to the Lindner dives, reached through secondary quotation.
 * Confidence: probable, not firm.
 *
 * THE ROW THAT SAID THE ELEVATOR RUNS OUT DESCRIBED THE WRONG AIRCRAFT AND ASKED
 * FOR SOMETHING NO AIRFRAME CAN DO. It measured the Mach number at which a full
 * nose up elevator can no longer make a nose up moment at the angle of attack of
 * 1 g flight, and it wanted that Mach number to be 0.86. Work the condition out.
 * The moment about the center of gravity is
 *
 *   Cm = Cm0 - staticMargin * CL + elevatorPower * elevator
 *
 * so a full nose up elevator makes no nose up moment only when
 *
 *   staticMargin >= (Cm0 + elevatorPower) / CL
 *
 * At 8000 m and Mach 0.86 the model carries CL 0.157 at 1 g, its zero lift
 * moment is +0.041 and a full nose up elevator is worth +0.094. The static
 * margin would have to reach 0.86 of the mean aerodynamic chord. Even with a
 * zero lift moment of zero it would have to reach 0.60. Published transonic
 * neutral point travel for a swept wing aircraft is 0.15 to 0.25 chord, and this
 * model already gives 0.24. The row was three to four times outside anything an
 * aerodynamic center shift can deliver, so it could never pass and it never told
 * anyone anything. The reason is the LOW LIFT COEFFICIENT: the tuck moment is
 * the static margin times CL, and at 8000 m and Mach 0.86 there is almost no CL
 * to work on. No table in src/physics/aero/compressibility.ts can fix that.
 *
 * The rows below therefore measure what the aircraft really did.
 *
 *
 * FOUR STATIC MEASUREMENTS, TWO DIVES AND ONE RECOVERY
 *
 *   1. THE TUCK ONSET. The pitching moment coefficient at a FIXED angle of
 *      attack and a FIXED elevator, against Mach. Holding both fixed is what
 *      isolates compressibility: a sweep at 1 g would change the angle of attack
 *      with the speed and mix the two effects. The onset is the Mach number
 *      where the moment has gone 0.01 more nose down than its low Mach value.
 *   2. THE SIZE OF THE TUCK. The same sweep gives the largest nose down change.
 *      Divided by the lift coefficient of the sweep, it IS the neutral point
 *      travel, so the published 0.15 to 0.25 chord sets the target.
 *   3. THE ELEVATOR POWER. The change of the moment coefficient over the full
 *      elevator travel, against Mach. It is a difference of two evaluations at
 *      the same state, so the thrust moment cancels exactly.
 *   4. THE STICK COMES BACK. The elevator that trims 1 g, against Mach. It runs
 *      nose up from the drag rise to the limit. That is the tuck as the pilot
 *      meets it.
 *   5. THE HANDS OFF DIVE. The elevator stays where the pilot left it. The nose
 *      goes DOWN on its own past the tuck onset and the Mach number runs up to
 *      the limit, where the wave drag holds it.
 *   6. THE RECOVERY. Closing the throttles alone does not bring the nose up.
 *      Closing them and holding a steady nose up command does. There is no
 *      separate tailplane channel in this model, so the trim lever appears here
 *      as a steady nose up elevator command that the pilot does not have to hold.
 *      The test also shows why the handbook warns against the lever: a coarse
 *      step pulls the aircraft out at a load factor near its structural limit.
 *   7. THE HELD DIVE. The autopilot holds a 30 degree dive attitude. The
 *      elevator it needs first runs nose down as the speed builds, then REVERSES
 *      and comes back as the aerodynamic center moves aft. The size of that
 *      reversal and the Mach it starts at are the last two rows.
 *
 * The static sweeps run at 8000 m, where the aircraft can really reach a high
 * Mach number in a dive.
 */

import { afterAll, describe, expect, it } from 'vitest';

import { trimLevelFlight, trimResiduals } from '@/aircraft/trim';
import type { TrimCondition } from '@/aircraft/trim';
import { FUEL_CAPACITY } from '@/aircraft/me262/mass';
import { MACH_LIMIT, TUCK_ONSET_MACH } from '@/physics/aero/compressibility';
import { isa } from '@/physics/atmosphere';
import { DEG, msToKmh, toDeg } from '@/math/units';
import {
  createFlightTest,
  describeSample,
  note,
  passed,
  placeInAir,
  printReport,
  record,
} from './harness';
import type { FlightTest } from './harness';

/** The altitude of every static sweep, m. */
const SWEEP_ALTITUDE = 8000;

/** The angle of attack the tuck sweep holds, rad. It is a normal cruise angle. */
const SWEEP_ALPHA = 2 * DEG;

/** How much more nose down the moment must go before the tuck counts as begun. */
const TUCK_THRESHOLD = 0.01;

/** The Mach number the sweeps take as the incompressible reference. */
const REFERENCE_MACH = 0.5;

/**
 * Published transonic neutral point travel of a swept wing aircraft, in mean
 * aerodynamic chords, and the width of that band.
 *
 * The aerodynamic center of every section runs aft as its shock strengthens, the
 * shock also cuts the downwash at the tail, and the whole aircraft follows both.
 * Measured swept wing aircraft move their neutral point 0.15 to 0.25 chord
 * between the drag rise and Mach 0.9. The same band anchors the AC_SHIFT table of
 * src/physics/aero/compressibility.ts, so the rows below check that the section
 * table really delivers it on the whole aircraft.
 *
 * A CAVEAT FOR WHOEVER OWNS compressibility.ts. This band is a WHOLE AIRCRAFT
 * number and most of it is the downwash, not the wing. NACA TN 3501 measured the
 * aerodynamic center of a 10 percent section at aspect ratio 6 and found it at
 * 0.23 chord from Mach 0.4 to 0.8 and only 0.30 chord at Mach 0.85, which is 7
 * percent of chord and not 20. AC_SHIFT_X of compressibility.ts runs the SECTION
 * from 0.25 to 0.50 chord over the same range, so the model may carry the tuck
 * too much on the section shift and too little on the downwash. The whole
 * aircraft answer below is right. The split inside it is worth a bead.
 * Source: NACA TN 3501, Nelson and McDevitt, 1955. Confidence: firm on the
 * section measurement, estimate on the whole aircraft band.
 */
const NEUTRAL_POINT_TRAVEL = 0.2; // MAC
const NEUTRAL_POINT_TRAVEL_BAND = 0.05; // MAC

/**
 * How much of the tuck the elevator has to take up in the held dive, as a
 * fraction of the elevator travel, and the band on it.
 *
 * A dive at a HELD ATTITUDE lets the aircraft answer part of the tuck by shedding
 * angle of attack, so the elevator only makes up the rest. Both parts follow the
 * neutral point travel, so the reversal follows it as well: the model travels
 * 0.196 chord and reverses 0.062 of the elevator, which scales to 0.047 at the
 * bottom of the published band and 0.078 at the top. The band below is a little
 * wider than that, so that DIVE_TUCK_THRESHOLD can sit on its lower edge.
 */
const DIVE_TUCK_ELEVATOR = 0.06;
const DIVE_TUCK_ELEVATOR_BAND = 0.03;

/**
 * How far the elevator has to come back before the dive tuck counts as felt.
 *
 * IT SITS ON THE LOWER EDGE OF THE BAND ABOVE, AND THAT IS THE POINT. The old
 * value was 0.1, while the row that measures the SIZE of the same reversal
 * accepted anything from 0 to 0.2. A model at the low end of that band could
 * never cross the threshold, so one row passed and the other failed on the same
 * flight. A threshold at the lower edge of the accepted band cannot do that:
 * every model that passes the size row must cross it.
 */
const DIVE_TUCK_THRESHOLD = DIVE_TUCK_ELEVATOR - DIVE_TUCK_ELEVATOR_BAND;

const clean: TrimCondition = {
  altitude: SWEEP_ALTITUDE,
  speed: 200,
  flapSetting: 'up',
  gearDown: false,
  fuelMass: FUEL_CAPACITY,
};

const air = isa(SWEEP_ALTITUDE);

/** The full state at one Mach number, one angle of attack and one elevator. */
function evaluate(mach: number, alpha: number, elevator: number) {
  const speed = mach * air.speedOfSound;
  return trimResiduals({ ...clean, speed }, { speed, alpha, elevator, throttle: 0 });
}

/** Moment coefficient at one Mach number, one angle of attack and one elevator. */
function momentCoefficient(mach: number, alpha: number, elevator: number): number {
  return evaluate(mach, alpha, elevator).momentCoefficient;
}

/** Change of the moment coefficient per unit of elevator command. */
function elevatorPower(mach: number, alpha: number): number {
  return 0.5 * (momentCoefficient(mach, alpha, 1) - momentCoefficient(mach, alpha, -1));
}

/** The angle of attack that carries 1 g at one Mach number, rad. */
function alphaForLevelFlight(mach: number): number {
  const speed = mach * air.speedOfSound;
  let low = -0.05;
  let high = 0.3;
  for (let i = 0; i < 40; i++) {
    const middle = 0.5 * (low + high);
    const r = trimResiduals(
      { ...clean, speed },
      { speed, alpha: middle, elevator: 0, throttle: 0 },
    );
    // acrossPath is the lift less the weight, over the weight. It grows with the
    // angle of attack while the wing is attached.
    if (r.acrossPath > 0) {
      high = middle;
    } else {
      low = middle;
    }
  }
  return 0.5 * (low + high);
}

/** Static margin at one Mach number, from a central difference in the angle. */
function staticMargin(mach: number, alpha: number): number {
  const step = 0.5 * DEG;
  const above = evaluate(mach, alpha + step, 0);
  const below = evaluate(mach, alpha - step, 0);
  return (
    -(above.momentCoefficient - below.momentCoefficient) /
    (above.liftCoefficient - below.liftCoefficient)
  );
}

/** The elevator command that trims 1 g at one Mach number. */
function elevatorToTrim(mach: number): number {
  const alpha = alphaForLevelFlight(mach);
  return -momentCoefficient(mach, alpha, 0) / elevatorPower(mach, alpha);
}

afterAll(() => {
  printReport('MACH');
});

describe('the Mach tuck', () => {
  it('starts the nose down moment at the published Mach number', () => {
    const reference = momentCoefficient(REFERENCE_MACH, SWEEP_ALPHA, 0);
    let onset = 0;
    let lowest = Number.POSITIVE_INFINITY;
    let lowestMach = 0;
    let lowestLift = 0;
    const trace: string[] = [];
    for (let mach = REFERENCE_MACH; mach <= 0.98; mach += 0.005) {
      const state = evaluate(mach, SWEEP_ALPHA, 0);
      const cm = state.momentCoefficient;
      if (Math.abs((mach * 100) % 5) < 1e-6) {
        trace.push(`M ${mach.toFixed(2)}: Cm ${cm.toFixed(5)} (${(cm - reference).toFixed(5)})`);
      }
      if (cm - reference < lowest) {
        lowest = cm - reference;
        lowestMach = mach;
        lowestLift = state.liftCoefficient;
      }
      if (onset === 0 && cm - reference <= -TUCK_THRESHOLD) {
        onset = mach;
      }
    }
    for (const line of trace) {
      note(`  ${line}`);
    }
    note(
      onset > 0
        ? `tuck onset at Mach ${onset.toFixed(3)}, reference Cm ${reference.toFixed(5)}`
        : `NO TUCK. The most nose down the moment ever goes is ${lowest.toFixed(5)} at Mach ` +
            `${lowestMach.toFixed(3)}, against a threshold of ${-TUCK_THRESHOLD}.`,
    );

    // A measured zero means the sweep found no nose down moment at any Mach
    // number it reached. The table then shows the whole distance to the target,
    // which is the honest picture: the model has no tuck at all.
    const m = record({
      name: 'Mach tuck onset',
      measured: onset,
      target: TUCK_ONSET_MACH,
      tolerance: 0.02,
      toleranceKind: 'absolute',
      unit: 'Mach',
      note:
        onset > 0
          ? `the Mach where Cm falls ${TUCK_THRESHOLD} below its value at Mach ${REFERENCE_MACH}, ` +
            `at ${toDeg(SWEEP_ALPHA).toFixed(1)} deg and a neutral elevator, at ${SWEEP_ALTITUDE} m`
          : 'ZERO MEANS NO TUCK WAS FOUND up to Mach 0.98. The moment goes nose UP with Mach.',
    });

    // THE TARGET OF THE ROW BELOW IS NOT A NUMBER PULLED OUT OF THE AIR, AND IT
    // IS NOT THE DETECTION THRESHOLD EITHER. It used to be -TUCK_THRESHOLD, the
    // same 0.01 the row above uses to FIND the onset. That is circular: it asks
    // the tuck to be exactly as big as the smallest tuck the sweep can see, so
    // any model with a real tuck must overshoot it and fail. The size of the
    // tuck is the neutral point travel times the lift coefficient it acts on, so
    // the published 0.15 to 0.25 chord and the measured lift coefficient of this
    // sweep give the band.
    const travel = lowestLift > 0 ? -lowest / lowestLift : 0;
    note(
      `largest nose down change ${lowest.toFixed(5)} at Mach ${lowestMach.toFixed(3)}, ` +
        `CL there ${lowestLift.toFixed(4)}, so the neutral point has moved ` +
        `${travel.toFixed(3)} of the mean aerodynamic chord`,
    );
    note(
      `  static margin: ${staticMargin(REFERENCE_MACH, SWEEP_ALPHA).toFixed(3)} at Mach 0.5, ` +
        `${staticMargin(0.78, SWEEP_ALPHA).toFixed(3)} at Mach 0.78, ` +
        `${staticMargin(MACH_LIMIT, SWEEP_ALPHA).toFixed(3)} at Mach ${MACH_LIMIT}`,
    );
    record({
      name: 'largest nose down Cm change',
      measured: lowest,
      target: -NEUTRAL_POINT_TRAVEL * lowestLift,
      tolerance: NEUTRAL_POINT_TRAVEL_BAND * lowestLift,
      toleranceKind: 'absolute',
      unit: '-',
      note:
        `at Mach ${lowestMach.toFixed(3)}, where the sweep carries CL ${lowestLift.toFixed(4)}. ` +
        `The target is the published ${NEUTRAL_POINT_TRAVEL} +- ${NEUTRAL_POINT_TRAVEL_BAND} chord of ` +
        'transonic neutral point travel, times that CL. A positive value means the moment only ever went nose up.',
    });
    expect(passed(m)).toBe(true);
  });

  it('loses elevator power as the Mach number rises', () => {
    const low = elevatorPower(0.6, SWEEP_ALPHA);
    for (const mach of [0.6, 0.7, 0.75, 0.8, 0.83, 0.86, 0.9]) {
      const power = elevatorPower(mach, SWEEP_ALPHA);
      note(`  M ${mach.toFixed(2)}: elevator power ${power.toFixed(5)} per command, ` +
        `${((power / low) * 100).toFixed(1)} percent of the Mach 0.6 value`);
    }
    const atLimit = elevatorPower(MACH_LIMIT, SWEEP_ALPHA) / low;
    note(`elevator power at the Mach limit: ${(atLimit * 100).toFixed(1)} percent`);

    // The table in src/physics/aero/compressibility.ts takes the section control
    // effectiveness to 0.35 at Mach 0.86. The whole aircraft keeps a little more,
    // because the tail meets its own shock at its own sweep.
    //
    // The nearest measured aircraft agrees. NACA RM L8A05a flew the Bell XS-1,
    // which carries an 8 percent unswept wing, and reports that "the elevator
    // effectiveness decreased about one-half with increase of Mach number from
    // 0.70 to 0.87". Confidence: firm, primary.
    record({
      name: 'elevator power at Mach 0.86',
      measured: atLimit,
      target: 0.35,
      tolerance: 0.15,
      toleranceKind: 'absolute',
      unit: 'fraction',
      note: 'against the Mach 0.6 value. CONTROL_SCALE of compressibility.ts gives 0.35 at the section.',
    });
    expect(atLimit).toBeLessThan(1);
  });

  it('needs more nose up elevator to hold 1 g as the Mach number rises', () => {
    // This is the tuck as the pilot meets it. The aircraft is at 1 g at every
    // point, so the elevator that trims it is the stick position he holds. It
    // runs NOSE UP from the drag rise to the limit, which means the stick comes
    // back and back. The record for the real aircraft is a FORCE and not a
    // deflection: past Mach 0.83 the pull rose to about 100 lb at Mach 0.86 just
    // to hold the dive angle. This model has no hinge moment, so the deflection
    // below is the only stand in it has.
    for (const mach of [0.7, 0.78, 0.8, 0.82, TUCK_ONSET_MACH, 0.84, 0.85, MACH_LIMIT]) {
      note(`  M ${mach.toFixed(2)}: elevator to trim 1 g ${elevatorToTrim(mach).toFixed(4)}`);
    }
    const atDragRise = elevatorToTrim(0.78);
    const atOnset = elevatorToTrim(TUCK_ONSET_MACH);
    const atLimit = elevatorToTrim(MACH_LIMIT);
    note(
      `the stick comes back ${(atLimit - atDragRise).toFixed(3)} of its travel between Mach 0.78 ` +
        `and Mach ${MACH_LIMIT}, and ${(atLimit - atOnset).toFixed(3)} of it after the tuck onset`,
    );
    // Nose up is a positive command, so each step must be larger than the last.
    expect(atOnset).toBeGreaterThan(atDragRise);
    expect(atLimit).toBeGreaterThan(atOnset);
    // Half of the whole movement comes after the published onset, which is what
    // makes the onset the number a flight manual prints.
    expect(atLimit - atOnset).toBeGreaterThan(0.3 * (atLimit - atDragRise));
  });
});

describe('the Mach limit', () => {
  it('still answers a full nose up elevator, so the dive can be recovered', () => {
    // THE ME 262 WAS RECOVERABLE AT ITS LIMIT. The nose went down and the stick
    // got heavy, and the aircraft came back. A model whose elevator makes no
    // nose up moment at all would be a trap, and this aircraft was not one.
    const alpha = alphaForLevelFlight(MACH_LIMIT);
    const state = evaluate(MACH_LIMIT, alpha, 0);
    const full = momentCoefficient(MACH_LIMIT, alpha, 1);
    const margin = staticMargin(MACH_LIMIT, alpha);
    const power = elevatorPower(MACH_LIMIT, alpha);
    // Zero lift moment, from the angle of attack where the lift disappears.
    let low = -0.2;
    let high = 0.2;
    for (let i = 0; i < 40; i++) {
      const middle = 0.5 * (low + high);
      if (evaluate(MACH_LIMIT, middle, 0).liftCoefficient > 0) {
        high = middle;
      } else {
        low = middle;
      }
    }
    const zeroLiftMoment = momentCoefficient(MACH_LIMIT, 0.5 * (low + high), 0);
    note(
      `at Mach ${MACH_LIMIT} and ${SWEEP_ALTITUDE} m: alpha ${toDeg(alpha).toFixed(2)} deg, ` +
        `CL ${state.liftCoefficient.toFixed(4)}, Cm0 ${zeroLiftMoment.toFixed(4)}, ` +
        `static margin ${margin.toFixed(3)}, elevator power ${power.toFixed(4)}`,
    );
    note(
      `  a full nose up elevator makes Cm ${full.toFixed(4)}, which is still nose up. ` +
        `The old row wanted it at or below zero, which needs a static margin of ` +
        `${((zeroLiftMoment + power) / state.liftCoefficient).toFixed(2)} chord against the ` +
        `${(NEUTRAL_POINT_TRAVEL - NEUTRAL_POINT_TRAVEL_BAND).toFixed(2)} to ` +
        `${(NEUTRAL_POINT_TRAVEL + NEUTRAL_POINT_TRAVEL_BAND).toFixed(2)} chord of published travel.`,
    );
    // The angle of attack the elevator can still reach, and the load factor that
    // goes with it. This is what "the pull is gone" really means: not that the
    // elevator does nothing, but that it does far less.
    const extraLift = full / margin;
    const pullAtLimit = 1 + extraLift / state.liftCoefficient;
    note(`  full nose up elevator still commands about ${pullAtLimit.toFixed(1)} g at the limit`);
    expect(full).toBeGreaterThan(0);
  });

  it('runs up to the limit and drops its nose in a hands off dive', () => {
    // The pilot puts the aircraft into a dive and then leaves the stick alone.
    // Everything after that is the aircraft. The nose goes DOWN on its own, the
    // dive steepens, and the Mach number climbs until the wave drag stops it.
    const { test, entry } = handsOffDive();
    let peakMach = 0;
    let steepest = entry.pitch;
    const rows: string[] = [];
    for (let i = 0; i < 40; i++) {
      test.flyOpenLoop(0.5);
      const s = test.sample();
      if (s.altitude < 3000) {
        break;
      }
      peakMach = Math.max(peakMach, s.mach);
      steepest = Math.min(steepest, s.pitch);
      if (i % 6 === 0) {
        rows.push(
          `  t=${(i * 0.5).toFixed(1)}s M ${s.mach.toFixed(3)} h ${s.altitude.toFixed(0)} m ` +
            `V ${msToKmh(s.speed).toFixed(0)} km/h pitch ${toDeg(s.pitch).toFixed(1)} deg ` +
            `n ${s.loadFactor.toFixed(2)}`,
        );
      }
    }
    for (const row of rows) {
      note(row);
    }
    note(
      `hands off from Mach ${entry.mach.toFixed(3)}: the dive steepens from ` +
        `${toDeg(entry.pitch).toFixed(1)} to ${toDeg(steepest).toFixed(1)} degrees and the Mach ` +
        `number reaches ${peakMach.toFixed(3)}`,
    );

    // The nose must go DOWN. A model with no tuck holds its attitude or raises
    // the nose as the speed builds, and this assertion is what separates the two.
    expect(steepest).toBeLessThan(entry.pitch - 2 * DEG);

    // The wave drag is what stops the aircraft, not the pilot. The rise past
    // Mach 0.86 in WAVE_DRAG_CD of compressibility.ts is many times the thrust,
    // so a dive of this angle cannot push through it. Lindner reached 1004 km/h
    // at 4300 m in a dive of 20 to 25 degrees, which is Mach 0.862, and the same
    // program set the limit at Mach 0.86. This dive holds the same angle.
    record({
      name: 'peak Mach in a hands off dive',
      measured: peakMach,
      target: MACH_LIMIT,
      tolerance: 0.02,
      toleranceKind: 'absolute',
      unit: 'Mach',
      note:
        'a 20 degree dive from 10000 m at full throttle, with the stick left where the pilot ' +
        'put it. The wave drag holds the aircraft at the limit.',
    });
  });

  it('recovers when the throttles come back and the tailplane goes nose up', () => {
    // Closing the throttles alone is not enough. The drag rise already beats the
    // thrust, and the nose down moment does not care about the throttle at all.
    // The tailplane is what recovers it, which is what NACA RM L50G20 found on
    // the X-1: the elevator falls to 0.05 of the stabilizer effectiveness by
    // Mach 1.0, and half a degree of stabilizer covers the whole trim change.
    //
    // This model has no separate tailplane channel, so the trim lever appears
    // here as a steady nose up elevator command that the pilot does not hold.
    //
    // THE STEP IS COARSE ON PURPOSE. The Wright Field handbook warns that "a
    // slight deflection has a large effect, and it is easy to over-control,
    // causing the ship to nose over or pull up more abruptly than intended". The
    // load factor of the pull out is noted below, and it lands near the
    // structural limit of the airframe. That is the other half of the record.
    const TRIM_STEP = 0.2;

    /** Dives past the limit, then flies the recovery the caller asks for. */
    function recover(
      throttle: number,
      trim: number,
    ): { mach: number; pitch: number; peakLoad: number } {
      const { test, entry } = handsOffDive();
      test.flyOpenLoop(10);
      // `sample` hands back one object that it writes over on every call, so the
      // state at the limit has to be copied out before the aircraft flies on.
      const held = test.sample();
      const atLimit = { mach: held.mach, altitude: held.altitude };
      test.input.throttle = throttle;
      test.input.pitch = entry.elevator + trim;
      let peakLoad = 0;
      // THE WINDOW WAS 15 SECONDS UNTIL BEAD b65. That bead corrected the
      // reference sweep of compressibility.ts from 18.5 to 15.72 degrees, which
      // is the quarter chord angle of the published 18.5 degree leading edge.
      // The tailplane and the fin sweep less than the wing, so both now meet
      // their shock 1.5 percent later and both pay less wave drag. The aircraft
      // therefore sheds its Mach number more slowly and the recovery takes two
      // seconds longer. Measured, second by second: the nose reaches level at
      // 17 s and passes 9 degrees up at 18 s, against 15 s before the bead. The
      // window follows the measurement. The throttles alone still leave the
      // aircraft at Mach 0.85 and 33 degrees nose down at 18 s.
      const WINDOW = 18; // s
      for (let i = 0; i < WINDOW; i++) {
        test.flyOpenLoop(1);
        const s = test.sample();
        peakLoad = Math.max(peakLoad, s.loadFactor);
        if (s.altitude < 2000) {
          break;
        }
      }
      const out = test.sample();
      note(
        `  from Mach ${atLimit.mach.toFixed(3)} at ${atLimit.altitude.toFixed(0)} m, ` +
          `throttle ${throttle.toFixed(1)} and trim ${trim.toFixed(2)}: after ${WINDOW} s Mach ` +
          `${out.mach.toFixed(3)}, pitch ${toDeg(out.pitch).toFixed(1)} deg, ` +
          `${out.altitude.toFixed(0)} m, peak load factor ${peakLoad.toFixed(2)}`,
      );
      return { mach: out.mach, pitch: out.pitch, peakLoad };
    }

    const throttleOnly = recover(0, 0);
    const throttleAndTrim = recover(0, TRIM_STEP);

    // Closing the throttles alone leaves the aircraft in the dive. The Mach
    // number hardly falls and the nose keeps going down.
    expect(throttleOnly.mach).toBeGreaterThan(TUCK_ONSET_MACH);
    expect(throttleOnly.pitch).toBeLessThan(-25 * DEG);

    // The throttles and the tailplane together bring it out. The Mach number
    // falls below the tuck onset and the nose comes back up through level.
    expect(throttleAndTrim.mach).toBeLessThan(TUCK_ONSET_MACH);
    expect(throttleAndTrim.pitch).toBeGreaterThan(0);
    note(
      'the throttles alone leave the aircraft in the dive. The throttles and the tailplane ' +
        'together bring it out, and the coarse lever nearly over stresses the airframe on the ' +
        'way, which is what the handbook warns about and what broke aircraft in 1945.',
    );
  });
});

describe('the dive', () => {
  it('needs the stick back as the Mach number rises', () => {
    const start = 9000;
    const trim = trimLevelFlight({ ...clean, altitude: start, speed: 200 });
    const test = createFlightTest();
    placeInAir(test, {
      altitude: start,
      speed: 200,
      pitch: trim.pitch,
      flapSetting: 'up',
      gearDown: false,
    });
    test.command.altitude = null;
    test.command.throttle = 1;
    test.command.trimElevator = trim.elevator;
    test.command.pitch = trim.pitch;
    test.fly(15);

    // Nose down to a steady dive attitude. The autopilot holds that attitude,
    // so the elevator it needs is the moment the aircraft is making on its own.
    test.command.pitch = -30 * DEG;

    // THE BASELINE IS THE MOST NOSE DOWN THE ELEVATOR EVER GOES, NOT THE VALUE AT
    // A FIXED TIME. Two trends fight in this dive. The speed builds, which asks
    // for nose down elevator, and the aerodynamic center then moves aft, which
    // asks for nose up. The elevator therefore runs nose down, turns, and comes
    // back. The turn is the tuck, and the old baseline at eight seconds sat in
    // the middle of the entry transient and measured less than half of it.
    //
    // The measurement stops when the Mach number turns over. Past that point the
    // aircraft is descending into denser air at a nearly fixed Mach number, the
    // dynamic pressure climbs again, and the elevator goes back nose down for a
    // reason that has nothing to do with compressibility.
    const SETTLE = 8; // s
    let baseline = Number.POSITIVE_INFINITY;
    let baselineMach = 0;
    let reversal = 0;
    let reversalMach = 0;
    let tuckMach = 0;
    let peakMach = 0;
    const rows: string[] = [];
    for (let step = 0; step < 240; step++) {
      test.fly(0.25);
      const s = test.sample();
      if (s.altitude < 2000 || s.mach < peakMach - 0.001) {
        break;
      }
      const seconds = step * 0.25;
      if (seconds < SETTLE) {
        continue;
      }
      peakMach = Math.max(peakMach, s.mach);
      if (s.elevator < baseline) {
        baseline = s.elevator;
        baselineMach = s.mach;
      }
      const back = s.elevator - baseline;
      if (back > reversal) {
        reversal = back;
        reversalMach = s.mach;
      }
      if (tuckMach === 0 && back > DIVE_TUCK_THRESHOLD) {
        tuckMach = s.mach;
      }
      if (step % 8 === 0) {
        rows.push(
          `  t=${seconds.toFixed(0)}s M ${s.mach.toFixed(3)} h ${s.altitude.toFixed(0)} m ` +
            `V ${msToKmh(s.speed).toFixed(0)} km/h elevator ${s.elevator.toFixed(3)} ` +
            `alpha ${toDeg(s.alpha).toFixed(2)} deg n ${s.loadFactor.toFixed(2)}`,
        );
      }
    }
    for (const row of rows) {
      note(row);
    }
    note(
      `dive: the elevator reaches ${baseline.toFixed(3)} at Mach ${baselineMach.toFixed(3)} and ` +
        `comes back ${reversal.toFixed(3)} by Mach ${reversalMach.toFixed(3)}, peak Mach ` +
        `${peakMach.toFixed(3)}, tuck felt at Mach ${tuckMach.toFixed(3)}`,
    );
    note(`final state: ${describeSample(test.sample())}`);

    // The stick has to come BACK. That is the tuck, felt the way a pilot feels
    // it: the nose goes down on its own and the elevator follows it. A zero
    // means the elevator only ever went the other way.
    const m = record({
      name: 'elevator the dive tuck needs',
      measured: reversal,
      target: DIVE_TUCK_ELEVATOR,
      tolerance: DIVE_TUCK_ELEVATOR_BAND,
      toleranceKind: 'absolute',
      unit: 'command',
      note:
        `in a held 30 degree dive from 9000 m, measured from the most nose down elevator of the ` +
        `dive at Mach ${baselineMach.toFixed(3)} to the most nose up at Mach ` +
        `${reversalMach.toFixed(3)}. The band is the ` +
        `${(NEUTRAL_POINT_TRAVEL - NEUTRAL_POINT_TRAVEL_BAND).toFixed(2)} to ` +
        `${(NEUTRAL_POINT_TRAVEL + NEUTRAL_POINT_TRAVEL_BAND).toFixed(2)} chord of published neutral point travel.`,
    });
    record({
      name: `Mach where the dive needs ${DIVE_TUCK_THRESHOLD} more elevator`,
      measured: tuckMach > 0 ? tuckMach : 0,
      target: TUCK_ONSET_MACH,
      tolerance: 0.04,
      toleranceKind: 'absolute',
      unit: 'Mach',
      note:
        tuckMach > 0
          ? `flown, at a held 30 degree dive attitude from 9000 m. The threshold is the LOWER EDGE ` +
            `of the band of the row above, so a model that passes that row must reach it.`
          : `ZERO MEANS IT NEVER HAPPENED. The peak Mach was ${peakMach.toFixed(3)}.`,
    });
    expect(passed(m)).toBe(true);
  });
});

/**
 * Dives the aircraft to the Mach limit and hands the stick back to the pilot.
 *
 * The autopilot holds a 20 degree dive at full throttle from 10000 m until the
 * aircraft is through the drag rise. It then stops flying, and the returned
 * `entry.elevator` is the stick position it left behind. Every test that follows
 * works open loop from there, so nothing but the aircraft moves the controls.
 */
function handsOffDive(): { test: FlightTest; entry: { mach: number; pitch: number; elevator: number } } {
  const start = 10000;
  const trim = trimLevelFlight({ ...clean, altitude: start, speed: 220 });
  const test = createFlightTest();
  placeInAir(test, {
    altitude: start,
    speed: 220,
    pitch: trim.pitch,
    flapSetting: 'up',
    gearDown: false,
  });
  test.command.altitude = null;
  test.command.throttle = 1;
  test.command.trimElevator = trim.elevator;
  test.command.pitch = -20 * DEG;
  test.fly(12);
  const s = test.sample();
  const entry = { mach: s.mach, pitch: s.pitch, elevator: s.elevator };
  test.input.pitch = entry.elevator;
  test.input.throttle = 1;
  return { test, entry };
}
