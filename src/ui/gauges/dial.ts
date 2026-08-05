/**
 * The law of a dial, and the tick values that go with it.
 *
 *
 * 1. THE ANGLE CONVENTION
 *
 * A dial law turns a value into a needle angle. The angle is measured
 * CLOCKWISE from the twelve o'clock mark, in radians. Twelve o'clock is zero,
 * three o'clock is +pi/2, and nine o'clock is -pi/2.
 *
 * The face disc of src/render/models/cockpit.ts puts local +x to the right of
 * the dial and local +y at the top, and it points local +z at the pilot. A
 * needle that stands on local +y therefore reaches the value of the law when
 * its `rotation.z` is MINUS the angle. src/ui/gauges/parts.ts writes that one
 * sign, and no instrument module writes it again.
 *
 *
 * 2. WHY A TABLE AND NOT A FORMULA
 *
 * Almost no real instrument is linear. A vertical speed indicator crowds its
 * last ten meters per second into a few degrees, and an airspeed indicator
 * compresses everything under about 100 km/h, because the capsule follows the
 * dynamic pressure and the dynamic pressure follows the square of the speed.
 *
 * A `Table1D` of src/math/tables.ts states that shape as a list of knots. The
 * same table then draws the tick marks, so a tick and the needle can never
 * disagree. `lookup1d` clamps at both ends, which is what a needle does when it
 * reaches its stop, so the clamp needs no code of its own.
 *
 * This module is pure math. It touches no DOM and no renderer.
 */

import type { Table1D } from '@/math/tables';
import { lookup1d, table1d } from '@/math/tables';
import { DEG } from '@/math/units';

/**
 * One dial law. `min` and `max` are the ends of the printed scale. A value
 * outside them drives the needle onto the stop and no further.
 */
export interface DialLaw {
  /** Value on x, clockwise needle angle in radians on y. */
  readonly table: Table1D;
  /** First value of the printed scale. */
  readonly min: number;
  /** Last value of the printed scale. */
  readonly max: number;
}

/**
 * Build a law from a list of knots. `values` must increase. `anglesDeg` holds
 * the clockwise angle of each knot, in degrees from twelve o'clock.
 */
export function tableDial(values: readonly number[], anglesDeg: readonly number[]): DialLaw {
  const x = values.slice();
  const y = anglesDeg.map((a) => a * DEG);
  return { table: table1d(x, y), min: x[0], max: x[x.length - 1] };
}

/**
 * Build an even law. The needle stands at `startDeg` at `min` and sweeps
 * `sweepDeg` clockwise to reach `max`. A negative sweep runs anticlockwise.
 */
export function linearDial(
  min: number,
  max: number,
  startDeg: number,
  sweepDeg: number,
): DialLaw {
  return tableDial([min, max], [startDeg, startDeg + sweepDeg]);
}

/** The clockwise needle angle of one value, in radians. Both ends clamp. */
export function dialAngle(law: DialLaw, value: number): number {
  return lookup1d(law.table, value);
}

/**
 * The value list of a set of tick marks, from `from` to `to` every `step`.
 *
 * The loop counts in whole steps and multiplies, because adding a step over
 * and over collects a rounding error that moves the last tick off its mark.
 */
export function tickValues(from: number, to: number, step: number): number[] {
  const count = Math.round((to - from) / step);
  const out: number[] = [];
  for (let i = 0; i <= count; i++) out.push(from + i * step);
  return out;
}
