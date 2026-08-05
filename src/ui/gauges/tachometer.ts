/**
 * Drehzahlmesser, the rotor speed indicator. One per engine.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 carried two identical turbine tachometers on the engine group at
 * the right of the panel. The gauge reads to 12000 rpm, although the Jumo 004
 * never passes 8700, and dealer records say it was only accurate to about
 * 9000. The Australian War Memorial holds one from an Me 262 and gives its
 * scale as 2000 to 14000 rpm, so more than one face existed. A Drehzahlgeber
 * Fl.20230-1 on the engine drives it.
 * Source: Australian War Memorial REL/10140, and dealer records of the
 * Fl.20230-1 generator. Confidence: firm on the 12000 rpm face, firm on the
 * generator.
 *
 *
 * THE DANGER BAND, AND WHY IT IS MARKED
 *
 * DANGER_BAND_RPM of src/aircraft/me262/engine.ts is 6000 rpm. Below it the
 * compressor has almost no surge margin, so a lever pushed forward drives the
 * fuel-air ratio straight past the surge line and the engine bangs, stalls, or
 * burns a turbine. The pilot notes of the aircraft say to move the lever
 * slowly under that speed, and this red band is that instruction on the dial.
 *
 * The band starts at zero and ends at 6000, so an engine at its idle of 3000
 * rpm sits INSIDE the band. That is not a fault of the drawing. A Jumo 004 at
 * idle really is in the part of its range where a fast lever kills it.
 *
 * A green arc runs from the danger band to the maximum, and a red line marks
 * 8700 rpm, which is the maximum rotor speed of the engine.
 */

import { radPerSecToRpm } from '@/math/units';
import { DANGER_BAND_RPM, MAX_RPM } from '@/aircraft/me262/engine';

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
  SMALL_FACE_PIXELS,
  band,
  caption,
  drawFace,
  fillFace,
  numeralRow,
  tickRow,
} from './draw';

/** The end of the printed scale, rpm. Read the module comment. */
export const TACHOMETER_MAX_RPM = 12000;

/** The dial law. Even spacing over 300 degrees, which the real face shows. */
export const TACHOMETER_LAW: DialLaw = linearDial(0, TACHOMETER_MAX_RPM, -150, 300);

/**
 * Needle time constant, s. An eddy current tachometer is quick, and the rotor
 * of a Jumo 004 takes 8 to 10 s from idle to full power, so the needle never
 * sets the pace. The value is an ESTIMATE.
 */
export const TACHOMETER_LAG = 0.30;

/** The rotor speed the needle shows, rpm, from one engine. */
export function tachometerReading(rotorSpeed: number): number {
  // CONVENTIONS section 2: rad/s inside the model, rpm only on the gauge.
  return radPerSecToRpm(rotorSpeed);
}

/** The clockwise needle angle at one rotor speed in rpm. Both stops clamp. */
export function tachometerAngle(rpm: number): number {
  return dialAngle(TACHOMETER_LAW, rpm);
}

function paint(): HTMLCanvasElement {
  return drawFace(SMALL_FACE_PIXELS, (f) => {
    fillFace(f);
    // The surge band, and the normal band above it.
    band(f, TACHOMETER_LAW, 0, DANGER_BAND_RPM, 0.90, 0.10, DIAL_COLOR.red);
    band(f, TACHOMETER_LAW, DANGER_BAND_RPM, MAX_RPM, 0.90, 0.055, '#4e7f56');
    tickRow(
      f,
      TACHOMETER_LAW,
      tickValues(0, 12000, 500),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      TACHOMETER_LAW,
      tickValues(0, 12000, 2000),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      TACHOMETER_LAW,
      tickValues(0, 12000, 2000),
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v / 1000)),
    );
    // The maximum rotor speed.
    tickRow(f, TACHOMETER_LAW, [MAX_RPM], 0.22, 0.05, DIAL_COLOR.red);
    caption(f, 'U/min', 0, -0.32, DIAL_STYLE.caption, DIAL_COLOR.dim);
    caption(f, 'x1000', 0, -0.52, DIAL_STYLE.caption * 0.85, DIAL_COLOR.dim);
  });
}

/** Build one tachometer. `engine` is the index into the readout engine list. */
export function createTachometer(parts: GaugeParts, engine: number): Instrument {
  parts.addFace(paint());
  const needle = parts.addNeedle({
    name: 'rotor',
    length: 0.86,
    tail: 0.16,
    width: 0.055,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.10, 0x1a1c1e);

  const lag: NeedleLag = createLag(TACHOMETER_LAG, 0);

  return {
    update(_sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      // An engine that is not in the list reads zero, so a one engine build
      // still runs and the needle rests on its stop.
      const source = readout.engines[engine];
      const target = source === undefined ? 0 : tachometerReading(source.rotorSpeed);
      pointNeedle(needle, tachometerAngle(stepLag(lag, target, dt)));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
