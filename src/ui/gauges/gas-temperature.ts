/**
 * Abgastemperatur, the gas temperature indicator. One per engine.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 carried the electric temperature gauge Fl.20338, which reads 300
 * to 1000 degrees Celsius. The same gauge went into the Me 163, the Ar 234 and
 * the He 162. A thermocouple in the gas path drives it, and a thermocouple
 * needs no power of its own.
 * Source: dealer and museum records of the Fl.20338, which name the Me 262.
 * Confidence: firm on the fit, firm on the 300 to 1000 range.
 *
 * THE SCALE HERE STARTS AT ZERO, AND THAT IS A CHANGE. A cold engine is at the
 * air temperature, which is 15 degrees at sea level, and on the real face that
 * needle rests against the bottom stop with nothing to read. Screenshot one of
 * this bead is a parked aircraft with both engines off, so the scale is
 * extended down to zero and the needle stays on the dial. Say so rather than
 * hide it.
 *
 *
 * THE LIMIT, AND WHERE IT COMES FROM
 *
 * TURBINE_INLET_TEMPERATURE_LIMIT of src/aircraft/me262/engine.ts is 1100 K,
 * which is 827 degrees Celsius, and the engine model charges creep damage
 * above it. Full power gives 1015 K, which is 742 degrees, so the amber band
 * from 742 to 827 is the whole margin the pilot has.
 *
 * The engine model measures at the TURBINE INLET. A real Fl.20338 in an
 * Me 262 measured further downstream and read lower. This dial therefore
 * carries the limit OF THE MODEL and not the limit off a 1944 placard. A
 * needle that agrees with the damage the engine really takes is worth more
 * than a needle that agrees with a number this simulator does not use.
 */

import { kelvinToCelsius } from '@/math/units';
import { TURBINE_INLET_TEMPERATURE_LIMIT } from '@/aircraft/me262/engine';

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

/** The end of the printed scale, degrees Celsius. */
export const GAS_TEMPERATURE_MAX = 1000;

/** The limit line, degrees Celsius. It is 1100 K, which the engine model owns. */
export const GAS_TEMPERATURE_LIMIT_C = kelvinToCelsius(TURBINE_INLET_TEMPERATURE_LIMIT);

/** Where the amber band starts, degrees Celsius. Full power gives 1015 K. */
const GAS_TEMPERATURE_CAUTION_C = kelvinToCelsius(1015);

/** The dial law. Even spacing over 270 degrees. */
export const GAS_TEMPERATURE_LAW: DialLaw = linearDial(0, GAS_TEMPERATURE_MAX, -135, 270);

/**
 * Needle time constant, s.
 *
 * A thermocouple sits inside a heavy probe, and the probe must warm up before
 * the junction can read the gas. It is the SLOWEST needle on this panel by a
 * long way, and a pilot who watches the temperature on a fast opening lever
 * finds out too late. That is a real property of the engine and not a fault of
 * the gauge. The value is an ESTIMATE.
 */
export const GAS_TEMPERATURE_LAG = 2.5;

/** The temperature the needle shows, degrees Celsius, from one engine. */
export function gasTemperatureReading(kelvin: number): number {
  return kelvinToCelsius(kelvin);
}

/** The clockwise needle angle at one temperature in Celsius. Both stops clamp. */
export function gasTemperatureAngle(celsius: number): number {
  return dialAngle(GAS_TEMPERATURE_LAW, celsius);
}

function paint(): HTMLCanvasElement {
  return drawFace(SMALL_FACE_PIXELS, (f) => {
    fillFace(f);
    band(
      f,
      GAS_TEMPERATURE_LAW,
      GAS_TEMPERATURE_CAUTION_C,
      GAS_TEMPERATURE_LIMIT_C,
      0.90,
      0.075,
      DIAL_COLOR.amber,
    );
    band(
      f,
      GAS_TEMPERATURE_LAW,
      GAS_TEMPERATURE_LIMIT_C,
      GAS_TEMPERATURE_MAX,
      0.90,
      0.075,
      DIAL_COLOR.red,
    );
    tickRow(
      f,
      GAS_TEMPERATURE_LAW,
      tickValues(0, 1000, 50),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      GAS_TEMPERATURE_LAW,
      tickValues(0, 1000, 200),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      GAS_TEMPERATURE_LAW,
      tickValues(0, 1000, 200),
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v / 100)),
    );
    // The limit itself, as a line across the whole scale.
    tickRow(f, GAS_TEMPERATURE_LAW, [GAS_TEMPERATURE_LIMIT_C], 0.24, 0.05, DIAL_COLOR.red);
    caption(f, 'Abgas C', 0, -0.30, DIAL_STYLE.caption, DIAL_COLOR.dim);
    caption(f, 'x100', 0, -0.50, DIAL_STYLE.caption * 0.85, DIAL_COLOR.dim);
  });
}

/** Build one gas temperature gauge. `engine` indexes the readout engine list. */
export function createGasTemperature(parts: GaugeParts, engine: number): Instrument {
  parts.addFace(paint());
  const needle = parts.addNeedle({
    name: 'gas-temperature',
    length: 0.86,
    tail: 0.16,
    width: 0.055,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.10, 0x1a1c1e);

  const lag: NeedleLag = createLag(GAS_TEMPERATURE_LAG, 0);

  return {
    update(_sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      const source = readout.engines[engine];
      const target =
        source === undefined ? 0 : gasTemperatureReading(source.gasTemperature);
      pointNeedle(needle, gasTemperatureAngle(stepLag(lag, target, dt)));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
