/**
 * Hoehenmesser, the altimeter.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 A-1a carried the Hoehenmesser Fl.22320, the same aneroid
 * altimeter that the Bf 109 and the Fw 190 used. Dealer and museum records
 * give a range to 10000 m, and the two sliding bugs on the rim carry the marks
 * E for QFE and F for QFF.
 * Source: dealer and museum records of the Fl.22320, confidence: firm on the
 * fit, firm on the range near 10000 m.
 *
 * TWO HANDS, AND WHICH IS WHICH
 *
 * The real instrument has TWO hands, and this one has two as well.
 *
 *   The LONG hand turns one full circle every 1000 m. The outer scale carries
 *   ten numerals, so one numeral is 100 m.
 *   The SHORT hand turns one full circle over the whole range. The inner scale
 *   carries the numerals in kilometers.
 *
 * The service ceiling of this aircraft is 11450 m, so the range here runs to
 * 12000 m and the ceiling stays on the dial. That is 2000 m past the Fl.22320
 * face and it is a deliberate CHANGE, because an altimeter that pegs below the
 * ceiling of its own aircraft is useless.
 *
 * The long hand needs no wrap logic. The law repeats every 1000 m by taking
 * the remainder, and the lag runs on the WHOLE height and not on the
 * remainder, so a climb through 1000 m carries the hand smoothly past twelve
 * o'clock instead of unwinding the long way round.
 */

import type { DialLaw } from './dial';
import { dialAngle, linearDial, tickValues } from './dial';
import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { GAUGE_Z, pointNeedle } from './parts';
import type { NeedleLag } from './lag';
import { createLag, stepLag } from './lag';
import type { TelemetrySample } from '@/ui/debug-overlay';
import {
  DIAL_COLOR,
  DIAL_STYLE,
  LARGE_FACE_PIXELS,
  caption,
  drawFace,
  fillFace,
  luminousDots,
  numeralRow,
  tickRow,
} from './draw';

/** Height the long hand covers in one turn, m. */
export const ALTIMETER_HAND_RANGE = 1000;

/** Height the short hand covers in one turn, m. Read the module comment. */
export const ALTIMETER_FULL_RANGE = 12000;

/** The long hand. One turn is 1000 m, and zero stands at twelve o'clock. */
export const ALTIMETER_HAND_LAW: DialLaw = linearDial(0, ALTIMETER_HAND_RANGE, 0, 360);

/** The short hand. One turn is the whole range. */
export const ALTIMETER_KILOMETER_LAW: DialLaw = linearDial(0, ALTIMETER_FULL_RANGE, 0, 360);

/**
 * Needle time constant, s.
 *
 * An aneroid stack drives a heavier train than an airspeed capsule, and the
 * static line adds its own delay. The value is an ESTIMATE.
 */
export const ALTIMETER_LAG = 0.55;

/** The height the hands show, m, from one telemetry sample. */
export function altimeterReading(sample: TelemetrySample): number {
  // CONVENTIONS section 3.2: height above the ground is MINUS the world z.
  return -sample.state.position.z;
}

/** The clockwise angle of the LONG hand at one height in meters. */
export function altimeterHandAngle(altitude: number): number {
  // The remainder repeats the scale every turn. A negative height, which the
  // aircraft reaches on its wheels at a field below the datum, must wind the
  // hand back and not jump it forward, so the remainder is brought positive.
  const wrapped =
    ((altitude % ALTIMETER_HAND_RANGE) + ALTIMETER_HAND_RANGE) % ALTIMETER_HAND_RANGE;
  return dialAngle(ALTIMETER_HAND_LAW, wrapped);
}

/** The clockwise angle of the SHORT hand at one height in meters. */
export function altimeterKilometerAngle(altitude: number): number {
  return dialAngle(ALTIMETER_KILOMETER_LAW, altitude);
}

function paint(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    fillFace(f);
    tickRow(
      f,
      ALTIMETER_HAND_LAW,
      tickValues(0, 950, 50),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      ALTIMETER_HAND_LAW,
      tickValues(0, 900, 100),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      ALTIMETER_HAND_LAW,
      tickValues(0, 900, 100),
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v / 100)),
    );
    // The inner kilometer scale that the short hand reads.
    tickRow(
      f,
      ALTIMETER_KILOMETER_LAW,
      tickValues(0, 11000, 1000),
      0.055,
      0.03,
      DIAL_COLOR.dim,
    );
    luminousDots(f, ALTIMETER_HAND_LAW, [0], 0.885, 0.032);
    caption(f, 'Hoehe m', 0, -0.30, DIAL_STYLE.caption * 0.9, DIAL_COLOR.dim);
    caption(f, 'x100', 0, -0.47, DIAL_STYLE.caption * 0.85, DIAL_COLOR.dim);
  });
}

export function createAltimeter(parts: GaugeParts): Instrument {
  parts.addFace(paint());
  // The SHORT hand sits under the long one, so the long hand always reads
  // clear. It is wider, which is how a pilot tells the two apart at a glance.
  const kilometerHand = parts.addNeedle({
    name: 'kilometer',
    length: 0.52,
    tail: 0.14,
    width: 0.085,
    color: 0xd8d2c0,
    z: GAUGE_Z.lower,
  });
  const hand = parts.addNeedle({
    name: 'hundred',
    length: 0.88,
    tail: 0.17,
    width: 0.045,
    color: 0xf0ebdb,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.09, 0x1a1c1e);

  const lag: NeedleLag = createLag(ALTIMETER_LAG, 0);

  return {
    update(sample: TelemetrySample, _readout: CockpitReadout, dt: number): void {
      // The lag runs on the WHOLE height. Read the module comment.
      const altitude = stepLag(lag, altimeterReading(sample), dt);
      pointNeedle(hand, altimeterHandAngle(altitude));
      pointNeedle(kilometerHand, altimeterKilometerAngle(altitude));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
