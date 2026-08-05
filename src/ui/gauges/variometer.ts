/**
 * Variometer, the vertical speed indicator.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 carried the Variometer Fl.22386, which reads 30 m/s each way. The
 * face marks 5, 10, 20 and 30 on both sides, with the word Steigt for the
 * climb half and Sinkt for the descent half. The instrument hangs on the
 * static line through a calibrated leak and a compensation bottle.
 * Source: dealer and museum records of the Fl.22386, which name the Me 262
 * among its aircraft, confidence: firm.
 *
 *
 * THE DIAL LAW
 *
 * The marks are NOT evenly spaced, and the face makes that plain: the arc from
 * 0 to 5 is as long as the arc from 20 to 30. A vertical speed indicator
 * measures the rate that pressure leaks out of its case, and the gearing gives
 * fine resolution where a pilot needs it, which is near zero.
 *
 * Zero stands at nine o'clock, climb runs clockwise over the top, and descent
 * runs anticlockwise under the bottom. The two ends meet at three o'clock with
 * a gap between them, which is the layout of every vertical speed indicator
 * ever built.
 *
 *   0 m/s   at  -90 deg
 *   5 m/s   at  -35 deg      -5 m/s  at -145 deg
 *   10 m/s  at    0 deg     -10 m/s  at -180 deg
 *   20 m/s  at  +45 deg     -20 m/s  at -225 deg
 *   30 m/s  at  +70 deg     -30 m/s  at -250 deg
 *
 * The spacing is a RECONSTRUCTION from photographs of the face, confidence:
 * estimate. The range and the marked values are firm.
 *
 *
 * WHY THE LAG IS LONG
 *
 * The calibrated leak IS the lag. A real vertical speed indicator takes a
 * second or two to settle, and a pilot flies it by leading the needle. A
 * needle that answered at once would be a different instrument.
 */

import type { DialLaw } from './dial';
import { dialAngle, tableDial } from './dial';
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

/** The dial law of the Variometer. Read the module comment for its shape. */
export const VARIOMETER_LAW: DialLaw = tableDial(
  [-30, -20, -10, -5, 0, 5, 10, 20, 30],
  [-250, -225, -180, -145, -90, -35, 0, 45, 70],
);

/**
 * Needle time constant, s. The calibrated leak of a real instrument settles in
 * one to two seconds. The value is an ESTIMATE at the fast end of that band.
 */
export const VARIOMETER_LAG = 1.2;

/** The rate the needle shows, m/s, from one telemetry sample. */
export function variometerReading(sample: TelemetrySample): number {
  // CONVENTIONS section 3.2: a climb is MINUS the world z velocity.
  return -sample.state.velocity.z;
}

/** The clockwise needle angle at one rate in m/s. Both stops clamp. */
export function variometerAngle(rate: number): number {
  return dialAngle(VARIOMETER_LAW, rate);
}

const MARKED = [-30, -20, -10, -5, 5, 10, 20, 30];

function paint(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    fillFace(f);
    tickRow(
      f,
      VARIOMETER_LAW,
      [-25, -15, -7.5, -2.5, 2.5, 7.5, 15, 25],
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(f, VARIOMETER_LAW, MARKED, DIAL_STYLE.majorTick, DIAL_STYLE.majorWidth, DIAL_COLOR.mark);
    // Zero carries a longer and brighter mark, because a pilot reads this
    // instrument as "above zero or below zero" before anything else.
    tickRow(f, VARIOMETER_LAW, [0], 0.24, 0.055, DIAL_COLOR.mark);
    numeralRow(
      f,
      VARIOMETER_LAW,
      MARKED,
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.abs(v)),
    );
    luminousDots(f, VARIOMETER_LAW, [0], 0.885, 0.032);
    caption(f, 'Steigt', 0.30, 0.36, DIAL_STYLE.caption * 0.8, DIAL_COLOR.dim);
    caption(f, 'Sinkt', 0.30, -0.40, DIAL_STYLE.caption * 0.8, DIAL_COLOR.dim);
    caption(f, 'm/s', -0.30, -0.06, DIAL_STYLE.caption, DIAL_COLOR.dim);
  });
}

export function createVariometer(parts: GaugeParts): Instrument {
  parts.addFace(paint());
  const needle = parts.addNeedle({
    name: 'variometer',
    length: 0.86,
    tail: 0.17,
    width: 0.05,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.09, 0x1a1c1e);

  const lag: NeedleLag = createLag(VARIOMETER_LAG, 0);

  return {
    update(sample: TelemetrySample, _readout: CockpitReadout, dt: number): void {
      const rate = stepLag(lag, variometerReading(sample), dt);
      pointNeedle(needle, variometerAngle(rate));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
