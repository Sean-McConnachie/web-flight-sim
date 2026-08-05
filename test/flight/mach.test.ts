/**
 * Compressibility: the Mach tuck, the loss of elevator power, and the limit.
 *
 * The Me 262 was the first fighter to meet compressibility every day. The
 * documented tuck onset is Mach 0.83 and the documented limit is Mach 0.86. Past
 * the limit the nose dropped and the elevator went light, so the pilot could not
 * pull out. src/physics/aero/compressibility.ts holds the five effects that make
 * that behavior, and this file measures them on the whole aircraft.
 *
 *
 * THREE STATIC MEASUREMENTS AND ONE DIVE
 *
 *   1. THE TUCK ONSET. The pitching moment coefficient at a FIXED angle of
 *      attack and a FIXED elevator, against Mach. Holding both fixed is what
 *      isolates compressibility: a sweep at 1 g would change the angle of attack
 *      with the speed and mix the two effects. The onset is the Mach number
 *      where the moment has gone 0.01 more nose down than its low Mach value.
 *   2. THE ELEVATOR POWER. The change of the moment coefficient over the full
 *      elevator travel, against Mach. It is a difference of two evaluations at
 *      the same state, so the thrust moment cancels exactly.
 *   3. THE LIMIT. The Mach number at which the elevator, held full nose up at
 *      the angle of attack of 1 g flight, can no longer make a nose up moment.
 *      Past that point the aircraft cannot be recovered with the elevator, which
 *      is what the word limit means.
 *   4. THE DIVE. The harness dives the aircraft at a fixed pitch attitude with
 *      the autopilot holding that attitude. The elevator the autopilot needs is
 *      the tuck the pilot feels. It rises as the Mach number rises.
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

/** The altitude of every static sweep, m. */
const SWEEP_ALTITUDE = 8000;

/** The angle of attack the tuck sweep holds, rad. It is a normal cruise angle. */
const SWEEP_ALPHA = 2 * DEG;

/** How much more nose down the moment must go before the tuck counts as begun. */
const TUCK_THRESHOLD = 0.01;

/** The Mach number the sweeps take as the incompressible reference. */
const REFERENCE_MACH = 0.5;

const clean: TrimCondition = {
  altitude: SWEEP_ALTITUDE,
  speed: 200,
  flapSetting: 'up',
  gearDown: false,
  fuelMass: FUEL_CAPACITY,
};

const air = isa(SWEEP_ALTITUDE);

/** Moment coefficient at one Mach number, one angle of attack and one elevator. */
function momentCoefficient(mach: number, alpha: number, elevator: number): number {
  const speed = mach * air.speedOfSound;
  return trimResiduals({ ...clean, speed }, { speed, alpha, elevator, throttle: 0 })
    .momentCoefficient;
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

afterAll(() => {
  printReport('MACH');
});

describe('the Mach tuck', () => {
  it('starts the nose down moment at the published Mach number', () => {
    const reference = momentCoefficient(REFERENCE_MACH, SWEEP_ALPHA, 0);
    let onset = 0;
    let lowest = Number.POSITIVE_INFINITY;
    let lowestMach = 0;
    const trace: string[] = [];
    for (let mach = REFERENCE_MACH; mach <= 0.98; mach += 0.005) {
      const cm = momentCoefficient(mach, SWEEP_ALPHA, 0);
      if (Math.abs((mach * 100) % 5) < 1e-6) {
        trace.push(`M ${mach.toFixed(2)}: Cm ${cm.toFixed(5)} (${(cm - reference).toFixed(5)})`);
      }
      if (cm - reference < lowest) {
        lowest = cm - reference;
        lowestMach = mach;
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
    record({
      name: 'largest nose down Cm change',
      measured: lowest,
      target: -TUCK_THRESHOLD,
      tolerance: 0.005,
      toleranceKind: 'absolute',
      unit: '-',
      note: `at Mach ${lowestMach.toFixed(3)}. A positive value means the moment only ever went nose up.`,
    });
    expect(passed(m)).toBe(true);
  });

  it('loses elevator power as the Mach number rises', () => {
    const power = (mach: number): number =>
      0.5 * (momentCoefficient(mach, SWEEP_ALPHA, 1) - momentCoefficient(mach, SWEEP_ALPHA, -1));
    const low = power(0.6);
    for (const mach of [0.6, 0.7, 0.75, 0.8, 0.83, 0.86, 0.9]) {
      note(`  M ${mach.toFixed(2)}: elevator power ${power(mach).toFixed(5)} per command, ` +
        `${((power(mach) / low) * 100).toFixed(1)} percent of the Mach 0.6 value`);
    }
    const atLimit = power(MACH_LIMIT) / low;
    note(`elevator power at the Mach limit: ${(atLimit * 100).toFixed(1)} percent`);

    // The table in src/physics/aero/compressibility.ts takes the section control
    // effectiveness to 0.35 at Mach 0.86. The whole aircraft keeps a little more,
    // because the tail meets its own shock at its own sweep.
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

  it('runs out of elevator at the Mach limit', () => {
    // At each Mach number the aircraft sits at the angle of attack of 1 g and the
    // pilot holds the elevator full nose up. The limit is where that no longer
    // makes a nose up moment.
    let limit = 0;
    for (let mach = 0.6; mach <= 0.99; mach += 0.005) {
      const alpha = alphaForLevelFlight(mach);
      const cm = momentCoefficient(mach, alpha, 1);
      if (Math.abs((mach * 100) % 5) < 1e-6) {
        note(`  M ${mach.toFixed(2)}: alpha ${toDeg(alpha).toFixed(2)} deg, Cm at full nose up ${cm.toFixed(5)}`);
      }
      if (limit === 0 && cm <= 0) {
        limit = mach;
      }
    }
    note(limit > 0 ? `elevator authority runs out at Mach ${limit.toFixed(3)}` : 'the elevator never runs out below Mach 0.99');

    record({
      name: 'Mach limit, elevator authority',
      measured: limit > 0 ? limit : 0.99,
      target: MACH_LIMIT,
      tolerance: 0.02,
      toleranceKind: 'absolute',
      unit: 'Mach',
      note:
        limit > 0
          ? 'the Mach where a full nose up elevator no longer makes a nose up moment at 1 g'
          : 'the elevator still holds at Mach 0.99, so the model has no limit here',
    });
  });
});

describe('the dive', () => {
  it('needs more and more nose up elevator as the Mach number rises', () => {
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
    let baseline = 0;
    let tuckMach = 0;
    let peakMach = 0;
    let peakElevator = -1;
    const rows: string[] = [];
    for (let i = 0; i < 60; i++) {
      test.fly(1);
      const s = test.sample();
      if (s.altitude < 2000) {
        break;
      }
      // The baseline is the elevator the aircraft needs once the dive is steady
      // and before the Mach number has grown.
      if (i === 8) {
        baseline = s.elevator;
      }
      if (i >= 8) {
        if (s.mach > peakMach) {
          peakMach = s.mach;
          peakElevator = s.elevator;
        }
        if (tuckMach === 0 && s.elevator - baseline > 0.1) {
          tuckMach = s.mach;
        }
        if (i % 5 === 0) {
          rows.push(
            `  t=${i}s M ${s.mach.toFixed(3)} h ${s.altitude.toFixed(0)} m ` +
              `V ${msToKmh(s.speed).toFixed(0)} km/h elevator ${s.elevator.toFixed(3)} ` +
              `pitch ${toDeg(s.pitch).toFixed(1)} deg`,
          );
        }
      }
    }
    for (const row of rows) {
      note(row);
    }
    note(
      `dive: baseline elevator ${baseline.toFixed(3)}, peak Mach ${peakMach.toFixed(3)} ` +
        `with elevator ${peakElevator.toFixed(3)}, tuck felt at Mach ${tuckMach.toFixed(3)}`,
    );
    note(`final state: ${describeSample(test.sample())}`);

    // The aircraft should need MORE nose up elevator at the top of the dive than
    // it did at the start. That is the tuck, felt the way a pilot feels it: the
    // nose goes down on its own and the stick has to come back. A negative
    // change means the model does the opposite.
    const m = record({
      name: 'elevator change at the peak dive Mach',
      measured: peakElevator - baseline,
      target: 0.1,
      tolerance: 0.1,
      toleranceKind: 'absolute',
      unit: 'command',
      note:
        `peak Mach ${peakMach.toFixed(3)} in a held 30 degree dive from 9000 m. ` +
        'A POSITIVE value is a tuck. A negative value means the nose rises on its own.',
    });
    record({
      name: 'Mach where the dive needs 0.1 more elevator',
      measured: tuckMach > 0 ? tuckMach : 0,
      target: TUCK_ONSET_MACH,
      tolerance: 0.04,
      toleranceKind: 'absolute',
      unit: 'Mach',
      note:
        tuckMach > 0
          ? 'flown, at a held 30 degree dive attitude from 9000 m'
          : `ZERO MEANS IT NEVER HAPPENED. The peak Mach was ${peakMach.toFixed(3)}.`,
    });
    expect(passed(m)).toBe(true);
  });
});
