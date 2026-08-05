/**
 * Fahrtmesser, the airspeed indicator.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 A-1a carried the Fahrtmesser Fl.22245, which reads 0 to 1000
 * km/h. The same instrument went into the Me 163, the Do 335 and the He 219,
 * and it was built between 1944 and 1945. The earlier Fl.22231 reads only to
 * 800 km/h and would peg in a dive of this aircraft, because the placard
 * speed alone is 950 km/h true.
 * Source: dealer and museum records of the Fl.22245, confidence: firm on the
 * range, firm on the fit to the Me 262.
 *
 *
 * THE DIAL LAW
 *
 * The capsule of a pitot instrument follows the DYNAMIC PRESSURE, and the
 * dynamic pressure follows the square of the speed. The gearing of a real
 * instrument straightens most of that curve out, but it never straightens the
 * bottom of the scale, so every German face of this class crowds the first
 * hundred kilometers per hour into a short arc.
 *
 * The law here is a three knot table with that shape:
 *
 *   0 km/h    at -155 deg
 *   100 km/h  at -128 deg
 *   1000 km/h at +155 deg
 *
 * The first 100 km/h therefore take 27 degrees and the next 100 take 31. The
 * shape is a RECONSTRUCTION from photographs of the face, confidence:
 * estimate. The range above it is firm.
 *
 *
 * WHAT IT READS
 *
 * A pitot static instrument reads the EQUIVALENT airspeed, not the true
 * airspeed. The two are the same at sea level and they part above it, and at
 * 6000 m the aircraft that makes 870 km/h true shows about 640 km/h on this
 * dial. That gap is not an error. It is what the pilot really saw.
 */

import { msToKmh } from '@/math/units';

import type { DialLaw } from './dial';
import { dialAngle, tableDial, tickValues } from './dial';
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

/** The dial law of the Fahrtmesser. Read the module comment for its source. */
export const AIRSPEED_LAW: DialLaw = tableDial([0, 100, 1000], [-155, -128, 155]);

/**
 * Needle time constant, s.
 *
 * A capsule instrument answers a gust in about a third of a second. The value
 * is an ESTIMATE, chosen so the needle follows a hard pull without a step and
 * still shows the shake of a stall.
 */
export const AIRSPEED_LAG = 0.35;

/** The speed the needle shows, km/h, from one telemetry sample. */
export function airspeedReading(sample: TelemetrySample): number {
  return msToKmh(sample.equivalentAirspeed);
}

/** The clockwise needle angle at one speed in km/h. Both stops clamp. */
export function airspeedAngle(speedKmh: number): number {
  return dialAngle(AIRSPEED_LAW, speedKmh);
}

function paint(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    fillFace(f);
    tickRow(
      f,
      AIRSPEED_LAW,
      tickValues(0, 1000, 20),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      AIRSPEED_LAW,
      tickValues(0, 1000, 100),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      AIRSPEED_LAW,
      tickValues(100, 1000, 100),
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v / 100)),
    );
    // The luminous dots of a radium paint. A pilot finds the scale by them in
    // the dark, and they are what makes the face read as 1944 and not as new.
    luminousDots(f, AIRSPEED_LAW, [200, 400, 600, 800], 0.885, 0.028);
    caption(f, 'km/h', 0, -0.34, DIAL_STYLE.caption, DIAL_COLOR.dim);
    caption(f, 'x100', 0, -0.52, DIAL_STYLE.caption * 0.85, DIAL_COLOR.dim);
  });
}

export function createAirspeed(parts: GaugeParts): Instrument {
  parts.addFace(paint());
  const needle = parts.addNeedle({
    name: 'airspeed',
    length: 0.86,
    tail: 0.17,
    width: 0.05,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.09, 0x1a1c1e);

  const lag: NeedleLag = createLag(AIRSPEED_LAG, 0);

  return {
    update(sample: TelemetrySample, _readout: CockpitReadout, dt: number): void {
      const speedKmh = stepLag(lag, airspeedReading(sample), dt);
      pointNeedle(needle, airspeedAngle(speedKmh));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
