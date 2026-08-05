/**
 * Borduhr, the aircraft clock. It fills the spare 57 mm bezel of the bottom
 * row of the flight group.
 *
 *
 * THE REAL INSTRUMENT
 *
 * Every Luftwaffe aircraft carried a Borduhr on the main panel, and the Me 262
 * is no exception. Junghans, Kienzle and Hanhart all built them, and the
 * fighter versions carried an eight day movement with a sweep second hand.
 * Section 3 of src/render/models/cockpit.ts already names this bezel a clock,
 * so this module fills the bezel that file left.
 * Source: photographs of restored A-1a panels. Confidence: firm on the fit,
 * estimate on the face.
 *
 * WHAT IT SHOWS. The clock reads the time since the last spawn and not the
 * time of day, because the simulator holds no time of day. The hour hand
 * therefore starts at twelve. Say so rather than invent a launch hour.
 *
 * NO LAG. A clock is the one instrument on this panel with no needle lag at
 * all. An escapement steps and it never overshoots, so a lag here would be a
 * fault and not a feature. The second hand steps in fifths of a second, as a
 * five beat movement does.
 */

import type { DialLaw } from './dial';
import { dialAngle, linearDial, tickValues } from './dial';
import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { GAUGE_Z, pointNeedle } from './parts';
import type { TelemetrySample } from '@/ui/debug-overlay';
import {
  DIAL_COLOR,
  DIAL_STYLE,
  SMALL_FACE_PIXELS,
  caption,
  drawFace,
  fillFace,
  luminousDots,
  numeralRow,
  tickRow,
} from './draw';

const SECONDS_PER_MINUTE = 60;
const SECONDS_PER_HOUR = 3600;
const SECONDS_PER_HALF_DAY = 12 * SECONDS_PER_HOUR;

/** The minute scale. One turn is 60 minutes, and it also serves the seconds. */
export const CLOCK_MINUTE_LAW: DialLaw = linearDial(0, 60, 0, 360);

/** The hour scale. One turn is 12 hours. */
export const CLOCK_HOUR_LAW: DialLaw = linearDial(0, 12, 0, 360);

/** Beats of the movement in one second. A five beat train steps five times. */
const BEATS_PER_SECOND = 5;

/** The clockwise angle of the second hand at one elapsed time in seconds. */
export function clockSecondAngle(time: number): number {
  const stepped = Math.floor(time * BEATS_PER_SECOND) / BEATS_PER_SECOND;
  return dialAngle(CLOCK_MINUTE_LAW, stepped % SECONDS_PER_MINUTE);
}

/** The clockwise angle of the minute hand at one elapsed time in seconds. */
export function clockMinuteAngle(time: number): number {
  return dialAngle(CLOCK_MINUTE_LAW, (time % SECONDS_PER_HOUR) / SECONDS_PER_MINUTE);
}

/** The clockwise angle of the hour hand at one elapsed time in seconds. */
export function clockHourAngle(time: number): number {
  return dialAngle(CLOCK_HOUR_LAW, (time % SECONDS_PER_HALF_DAY) / SECONDS_PER_HOUR);
}

function paint(): HTMLCanvasElement {
  return drawFace(SMALL_FACE_PIXELS, (f) => {
    fillFace(f);
    tickRow(
      f,
      CLOCK_MINUTE_LAW,
      tickValues(0, 59, 1),
      0.07,
      0.022,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      CLOCK_MINUTE_LAW,
      tickValues(0, 55, 5),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      CLOCK_HOUR_LAW,
      [12, 3, 6, 9],
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral * 1.1,
      DIAL_COLOR.mark,
      (v) => String(v),
    );
    luminousDots(f, CLOCK_HOUR_LAW, [12, 3, 6, 9], 0.88, 0.032);
    caption(f, 'Borduhr', 0, -0.42, DIAL_STYLE.caption * 0.9, DIAL_COLOR.dim);
  });
}

export function createClock(parts: GaugeParts): Instrument {
  parts.addFace(paint());
  const hourHand = parts.addNeedle({
    name: 'hour',
    length: 0.52,
    tail: 0.10,
    width: 0.09,
    color: 0xe8e2d0,
    z: GAUGE_Z.lower,
  });
  const minuteHand = parts.addNeedle({
    name: 'minute',
    length: 0.82,
    tail: 0.12,
    width: 0.055,
    color: 0xf0ebdb,
    z: GAUGE_Z.needle,
  });
  // The sweep second hand stands on the topmost layer, and its own tail
  // covers the roots of the other two. A hub cap on top of it would need one
  // more layer, and section 2 of src/ui/gauges/parts.ts has no room for it.
  const secondHand = parts.addNeedle({
    name: 'second',
    length: 0.90,
    tail: 0.22,
    width: 0.024,
    color: 0xd2582f,
    z: GAUGE_Z.hub,
  });

  return {
    update(sample: TelemetrySample, _readout: CockpitReadout, _dt: number): void {
      const time = sample.loop.simTime;
      pointNeedle(hourHand, clockHourAngle(time));
      pointNeedle(minuteHand, clockMinuteAngle(time));
      pointNeedle(secondHand, clockSecondAngle(time));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
