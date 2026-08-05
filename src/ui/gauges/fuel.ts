/**
 * Kraftstoffvorrat, the fuel contents gauge.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 A-1a carried a fuel contents gauge on the left of the panel, with
 * a selector that read one tank at a time. The aircraft holds four cells: two
 * main cells of 900 liters, a rear cell of 600 liters, and a small forward
 * cell of 170 liters.
 * Source: photographs of restored A-1a panels and the aircraft fuel system.
 * Confidence: firm on the fit, estimate on the face.
 *
 * THE SCALE IS IN KILOGRAMS, AND THAT IS A CHANGE. A German gauge of 1944
 * reads LITERS. The flight model tracks fuel MASS, because mass is what moves
 * the center of gravity and what the engines burn, and src/math/units.ts owns
 * no fuel density to turn one into the other with. Rather than invent a
 * density in src/ui, this dial reads kilograms and says so on its face. The
 * needle is then the number the model really holds.
 *
 * The tanks hold 2133 kg when full, which is FUEL_CAPACITY of
 * src/aircraft/me262/mass.ts. The scale runs to 2200 kg so the full mark sits
 * inside the dial and not on the stop.
 *
 * The red band is the last 200 kg. Two engines at full power burn 0.71 kg/s,
 * so 200 kg is about five minutes of combat power and about fifteen minutes of
 * a careful let down.
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
  band,
  caption,
  drawFace,
  fillFace,
  luminousDots,
  numeralRow,
  tickRow,
} from './draw';

/** The end of the printed scale, kg. Read the module comment. */
export const FUEL_SCALE_MAX = 2200;

/** Where the red band ends, kg. */
export const FUEL_LOW_MARK = 200;

/** The dial law. Even spacing over 270 degrees, empty at seven o'clock. */
export const FUEL_LAW: DialLaw = linearDial(0, FUEL_SCALE_MAX, -135, 270);

/**
 * Needle time constant, s.
 *
 * A float in a tank swings with every movement of the aircraft, so a contents
 * gauge is damped hard on purpose. It is the second slowest needle on this
 * panel. The value is an ESTIMATE.
 */
export const FUEL_LAG = 4;

/** The clockwise needle angle at one fuel load in kg. Both stops clamp. */
export function fuelAngle(fuelMass: number): number {
  return dialAngle(FUEL_LAW, fuelMass);
}

function paint(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    fillFace(f);
    band(f, FUEL_LAW, 0, FUEL_LOW_MARK, 0.90, 0.075, DIAL_COLOR.red);
    tickRow(
      f,
      FUEL_LAW,
      tickValues(0, 2200, 100),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      FUEL_LAW,
      tickValues(0, 2000, 500),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      FUEL_LAW,
      tickValues(0, 2000, 500),
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v / 100)),
    );
    luminousDots(f, FUEL_LAW, [0], 0.885, 0.030);
    caption(f, 'Kraftstoff', 0, 0.44, DIAL_STYLE.caption * 0.85, DIAL_COLOR.dim);
    caption(f, 'kg', 0, -0.30, DIAL_STYLE.caption, DIAL_COLOR.dim);
    caption(f, 'x100', 0, -0.48, DIAL_STYLE.caption * 0.85, DIAL_COLOR.dim);
  });
}

export function createFuel(parts: GaugeParts): Instrument {
  parts.addFace(paint());
  const needle = parts.addNeedle({
    name: 'fuel',
    length: 0.86,
    tail: 0.16,
    width: 0.055,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.09, 0x1a1c1e);

  const lag: NeedleLag = createLag(FUEL_LAG, 0);

  return {
    update(_sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      pointNeedle(needle, fuelAngle(stepLag(lag, readout.fuelMass, dt)));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
