/**
 * Doppeldruckmesser, the twin pressure gauge for fuel and oil. One per engine.
 * It fills the two spare 57 mm bezels of the bottom row of the engine group.
 *
 *
 * THE REAL INSTRUMENT
 *
 * Section 3 of src/render/models/cockpit.ts already names the bottom row of
 * the engine group a fuel and oil pressure gauge, and a twin pressure gauge on
 * one dial is standard German practice: two needles of different color over
 * one scale in atmospheres. Source: photographs of restored A-1a panels.
 * Confidence: firm on the fit, estimate on the face.
 *
 *
 * WHERE THE TWO NUMBERS COME FROM, AND WHY THEY ARE HONEST ESTIMATES
 *
 * THE FLIGHT MODEL HOLDS NO PRESSURE AT ALL. src/aircraft/me262/engine.ts
 * models the rotor, the thrust, the gas temperature and the fuel flow, and it
 * models no pump and no oil system. Nothing here can therefore read a real
 * pressure, and nothing here invents one and calls it firm.
 *
 * What this gauge shows is the SENDER, which is a plain function of two values
 * the model does hold:
 *
 *   The FUEL pressure follows the fuel flow. A gear pump at a fixed delivery
 *   raises the line pressure with the flow it has to push, so the needle is
 *   the flow over MAX_FUEL_FLOW of the engine module.
 *   The OIL pressure follows the rotor speed, because the oil pump hangs off
 *   the same shaft. It rises fast at the bottom of the range and then flattens
 *   off, so the needle follows the square root of the rotor speed fraction.
 *
 * Both curves are ESTIMATES with no source. They are here because a dead dial
 * in a cockpit reads as a fault, and because both needles then say something
 * true: an engine that is turning has oil pressure and an engine that is off
 * has none.
 *
 * The numerals on the face run from 0 to 10, which is the working range of a
 * German pressure gauge in atmospheres. The dial law maps a FRACTION of the
 * full scale, so no pressure unit conversion is involved and CONVENTIONS
 * section 2 has nothing to convert.
 */

import { clamp } from '@/math/tables';
import { MAX_FUEL_FLOW, OMEGA_MAX } from '@/aircraft/me262/engine';

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
  caption,
  drawFace,
  fillFace,
  numeralRow,
  tickRow,
} from './draw';

/** The end of the printed scale, in atmospheres. */
export const PRESSURE_SCALE_MAX = 10;

/** The dial law, in atmospheres. Even spacing over 270 degrees. */
export const PRESSURE_LAW: DialLaw = linearDial(0, PRESSURE_SCALE_MAX, -135, 270);

/** Time constant of both needles, s. A bourdon tube with a restrictor. */
export const PRESSURE_LAG = 0.6;

/** Fuel pressure at the full flow of one engine, atmospheres. ESTIMATE. */
const FUEL_PRESSURE_AT_MAX_FLOW = 8.5;

/** Oil pressure at the maximum rotor speed, atmospheres. ESTIMATE. */
const OIL_PRESSURE_AT_MAX_RPM = 6.5;

/** Fuel pressure at one fuel flow in kg/s, atmospheres. Read the comment. */
export function fuelPressure(fuelFlow: number): number {
  return clamp(fuelFlow / MAX_FUEL_FLOW, 0, 1) * FUEL_PRESSURE_AT_MAX_FLOW;
}

/** Oil pressure at one rotor speed in rad/s, atmospheres. Read the comment. */
export function oilPressure(rotorSpeed: number): number {
  return Math.sqrt(clamp(rotorSpeed / OMEGA_MAX, 0, 1)) * OIL_PRESSURE_AT_MAX_RPM;
}

/** The clockwise needle angle at one pressure. Both stops clamp. */
export function pressureAngle(atmospheres: number): number {
  return dialAngle(PRESSURE_LAW, atmospheres);
}

function paint(): HTMLCanvasElement {
  return drawFace(SMALL_FACE_PIXELS, (f) => {
    fillFace(f);
    tickRow(
      f,
      PRESSURE_LAW,
      tickValues(0, 10, 0.5),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      PRESSURE_LAW,
      tickValues(0, 10, 2),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    numeralRow(
      f,
      PRESSURE_LAW,
      tickValues(0, 10, 2),
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v)),
    );
    // The two needle colors, named on the face. K is Kraftstoff, the fuel, and
    // S is Schmierstoff, the oil.
    caption(f, 'K', -0.30, -0.30, DIAL_STYLE.caption * 1.1, '#e8e2d0');
    caption(f, 'S', 0.30, -0.30, DIAL_STYLE.caption * 1.1, '#d2582f');
    caption(f, 'at', 0, -0.52, DIAL_STYLE.caption, DIAL_COLOR.dim);
  });
}

/** Build one twin pressure gauge. `engine` indexes the readout engine list. */
export function createPressure(parts: GaugeParts, engine: number): Instrument {
  parts.addFace(paint());
  const oilNeedle = parts.addNeedle({
    name: 'oil',
    length: 0.62,
    tail: 0.14,
    width: 0.07,
    color: 0xd2582f,
    z: GAUGE_Z.lower,
  });
  const fuelNeedle = parts.addNeedle({
    name: 'fuel',
    length: 0.86,
    tail: 0.16,
    width: 0.05,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.10, 0x1a1c1e);

  const fuelLag: NeedleLag = createLag(PRESSURE_LAG, 0);
  const oilLag: NeedleLag = createLag(PRESSURE_LAG, 0);

  return {
    update(_sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      const source = readout.engines[engine];
      const fuel = source === undefined ? 0 : fuelPressure(source.fuelFlow);
      const oil = source === undefined ? 0 : oilPressure(source.rotorSpeed);
      pointNeedle(fuelNeedle, pressureAngle(stepLag(fuelLag, fuel, dt)));
      pointNeedle(oilNeedle, pressureAngle(stepLag(oilLag, oil, dt)));
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
