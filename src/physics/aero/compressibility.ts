/**
 * Mach number effects on the section coefficients.
 *
 * The Me-262 was the first fighter to meet compressibility every day. Its
 * documented tuck onset is Mach 0.83 and its documented limit is Mach 0.86.
 * Past the limit the nose dropped, the elevator went light, and the pilot could
 * not pull out. The correct recovery was to close the throttles and wind the
 * trim wheel back, not to pull harder. This module holds the five effects that
 * produce that behavior.
 *
 *   clScale      Prandtl-Glauert lift growth below the critical Mach number.
 *   cdAdd        Wave drag rise above the critical Mach number.
 *   acShift      The center of pressure moves back. This is the Mach tuck.
 *   controlScale The control surfaces lose authority.
 *   clMaxScale   Shock induced separation takes the peak lift away.
 *
 * Sweep. The Me-262 carries 18.5 degrees of sweep at the quarter chord. Simple
 * sweep theory says that only the velocity component normal to the quarter chord
 * line drives the section pressures, so every shock driven effect answers to the
 * normal Mach number M cos(sweep), not to the free stream Mach number.
 *
 * What sweep relieves: the critical Mach number, the wave drag rise, the
 * aerodynamic center shift, the loss of control power, and the loss of peak
 * lift. All five are section pressure effects and all five move to a higher free
 * stream Mach number.
 *
 * What sweep does not relieve:
 *   - It delays the drag rise. It does not remove it. The rise still comes.
 *   - It does not help a surface with a different sweep. The caller passes the
 *     sweep of the surface it works on. The Me-262 tailplane has less sweep than
 *     the wing, so the tail meets its own shock first. That is one reason the
 *     elevator goes light before the wing loses lift.
 *   - Simple sweep theory assumes a wing of infinite span. A real wing of aspect
 *     ratio 7.21 with a fuselage at the root and two tips gets less than the full
 *     benefit. SWEEP_RELIEF_EXPONENT lets bead b33 reduce the relief.
 *
 * Every constant below carries the free stream Mach number at the reference
 * sweep, because that is the number the flight manual and the test reports use.
 * The module converts each anchor to a normal Mach number when it builds the
 * table. Bead b33 tunes these anchors against the level speed and dive data.
 *
 * This module is a pure coefficient source. It imports only from src/math.
 */

import { lookup1d, table1d } from '@/math/tables';
import type { Table1D } from '@/math/tables';
import { DEG } from '@/math/units';

/** The five Mach corrections an aerodynamic element applies. */
export interface MachCorrection {
  /** Multiplies the lift curve slope. 1.0 below Mach 0.3. */
  clScale: number;
  /** Adds to the section drag coefficient. 0 below the critical Mach number. */
  cdAdd: number;
  /** The center of pressure in chord fractions from the leading edge. 0.25 below the tuck. */
  acShift: number;
  /** Multiplies control surface effectiveness. 1.0 below Mach 0.75. */
  controlScale: number;
  /** Multiplies the peak lift coefficient. 1.0 below Mach 0.75. */
  clMaxScale: number;
}

// The quarter chord sweep of the Me-262 wing. Source: docs/CONVENTIONS.md
// section 8, confidence: firm. Every Mach anchor below is a free stream value at
// this sweep.
export const REFERENCE_SWEEP = 18.5 * DEG; // rad

// The exponent of the sweep relief. A value of 1 gives full simple sweep theory.
// A value of 0 removes the relief. Bead b33 can lower it if the model reaches
// its drag rise too late.
export const SWEEP_RELIEF_EXPONENT = 1.0;

// The free stream Mach number where the wave drag starts to rise.
//
// Reasoning. The maximum level speed at 6000 m is 870 km/h, that is 241.7 m/s.
// The speed of sound at 6000 m in the standard atmosphere is 316.4 m/s, so that
// speed is Mach 0.764. The drag rise must stay clear of that point, or the
// aircraft cannot reach its published speed. An onset at Mach 0.78 leaves a
// margin of 0.016 in Mach. The onset also agrees with the section: an 11 percent
// symmetric section at low lift has a critical Mach near 0.75, and 18.5 degrees
// of sweep raise that to 0.79 in free stream terms.
// Confidence: derived from firm speed data.
export const CRITICAL_MACH = 0.78;

// The free stream Mach number where the center of pressure starts to move back.
// Source: docs/CONVENTIONS.md section 8, confidence: firm.
//
// The brief for this bead said the shift runs from Mach 0.80 to Mach 0.90. The
// reference table gives a firm tuck onset of Mach 0.83, so the model starts the
// shift there. The table reaches 0.45 chord at Mach 0.90 as the brief asked.
export const TUCK_ONSET_MACH = 0.83;

// The free stream Mach number where the aircraft goes out of control.
// Source: docs/CONVENTIONS.md section 8, confidence: firm.
export const MACH_LIMIT = 0.86;

// Wave drag rise. Free stream Mach against the drag coefficient the shock adds.
//
// Anchor by anchor:
//   0.78  0.0000  The onset. Level flight at 870 km/h and 6000 m sits at Mach
//                 0.764, below this point, so the published speed stays reachable.
//   0.80  0.0015  A small rise. About 700 N of extra drag at 6000 m. The engines
//                 still have this much left, so the aircraft can pass 0.80 in a
//                 shallow dive.
//   0.82  0.0060  About 2900 N of extra drag at 6000 m. Two Jumo 004 at that
//                 altitude and speed give about 7000 N in total, so level flight
//                 runs out of thrust near here.
//   0.84  0.0180  About 9100 N. Level flight is gone. Only a dive gets past.
//   0.86  0.0420  The documented limit. The extra drag alone is three times the
//                 available thrust, so the aircraft needs a steep dive to hold it.
//   0.88  0.0800  Past the limit. The dive angle must keep growing.
//   0.92  0.1600  Deep in the rise.
//   1.00  0.2400  The plateau of the rise for a thin swept wing.
// Confidence: estimate, anchored on the firm level speed and limit Mach numbers.
export const WAVE_DRAG_MACH: readonly number[] = [0.78, 0.8, 0.82, 0.84, 0.86, 0.88, 0.92, 1.0];
export const WAVE_DRAG_CD: readonly number[] = [
  0.0, 0.0015, 0.006, 0.018, 0.042, 0.08, 0.16, 0.24,
];

// Aerodynamic center shift, in chord fractions from the leading edge. The shock
// on the upper surface moves the load aft, which makes a nose down moment that
// grows with Mach. This is the tuck.
//
// The shift reaches 0.45 chord at Mach 0.90. On a chord of 2.4 m that is
// 0.48 m behind the quarter chord. Confidence: estimate.
export const AC_SHIFT_MACH: readonly number[] = [0.83, 0.85, 0.86, 0.88, 0.9, 1.0];
export const AC_SHIFT_X: readonly number[] = [0.25, 0.3, 0.34, 0.4, 0.45, 0.48];

// Control effectiveness. A shock ahead of the hinge line cuts the pressure the
// surface can change, so the stick moves the aircraft less and less.
//
// The factor reaches 0.35 at Mach 0.86. The elevator loses its authority at the
// same Mach number where the tuck needs it most. That is the trap the Me-262
// pilots met. Confidence: estimate.
export const CONTROL_MACH: readonly number[] = [0.75, 0.78, 0.8, 0.82, 0.84, 0.86, 0.9, 1.0];
export const CONTROL_SCALE: readonly number[] = [1.0, 0.95, 0.88, 0.76, 0.58, 0.35, 0.2, 0.15];

// Peak lift against Mach. The shock separates the boundary layer behind it, so
// the wing breaks at a lower angle of attack as Mach grows. Without this table
// the pilot could pull 7 g at Mach 0.86, which the real aircraft could not do.
// Confidence: estimate.
export const CL_MAX_MACH: readonly number[] = [0.75, 0.8, 0.83, 0.86, 0.9, 1.0];
export const CL_MAX_SCALE: readonly number[] = [1.0, 0.92, 0.82, 0.68, 0.5, 0.4];

// The lowest value the Prandtl-Glauert factor 1 - M^2 may take.
//
// The Prandtl-Glauert rule holds only below the critical Mach number. Above it
// the rule runs to infinity at Mach 1, which no aircraft does. The floor of 0.25
// caps the lift growth at 2.0 and the cap starts at a normal Mach of 0.866. The
// clMaxScale table above then holds the usable lift down, so the cap never lets
// the pilot pull more than the real aircraft could.
export const PG_BETA_SQUARED_FLOOR = 0.25;

// The reference sweep turns each free stream anchor into a normal Mach number.
const REFERENCE_RELIEF = Math.cos(REFERENCE_SWEEP);

/** Converts free stream Mach anchors at the reference sweep into normal Mach anchors. */
function toNormalMach(freeStream: readonly number[]): number[] {
  const out = new Array<number>(freeStream.length);
  for (let i = 0; i < freeStream.length; i++) {
    out[i] = freeStream[i] * REFERENCE_RELIEF;
  }
  return out;
}

const WAVE_DRAG_TABLE: Table1D = table1d(toNormalMach(WAVE_DRAG_MACH), WAVE_DRAG_CD.slice());
const AC_SHIFT_TABLE: Table1D = table1d(toNormalMach(AC_SHIFT_MACH), AC_SHIFT_X.slice());
const CONTROL_TABLE: Table1D = table1d(toNormalMach(CONTROL_MACH), CONTROL_SCALE.slice());
const CL_MAX_TABLE: Table1D = table1d(toNormalMach(CL_MAX_MACH), CL_MAX_SCALE.slice());

/** Creates a correction that holds the neutral, low speed values. */
export function createMachCorrection(): MachCorrection {
  return { clScale: 1, cdAdd: 0, acShift: 0.25, controlScale: 1, clMaxScale: 1 };
}

/**
 * Writes the five Mach corrections into out and returns out. The function
 * allocates nothing.
 *
 * mach is the free stream Mach number of the element. sweep is the quarter chord
 * sweep of the surface the element belongs to, in radians. A negative sweep and
 * a negative Mach number both give the same result as their magnitude.
 */
export function machCorrection(mach: number, sweep: number, out: MachCorrection): MachCorrection {
  const m = mach < 0 ? -mach : mach;
  const cosSweep = Math.abs(Math.cos(sweep));
  const relief =
    SWEEP_RELIEF_EXPONENT === 1 ? cosSweep : Math.pow(cosSweep, SWEEP_RELIEF_EXPONENT);
  const normalMach = m * relief;

  let betaSquared = 1 - normalMach * normalMach;
  if (betaSquared < PG_BETA_SQUARED_FLOOR) {
    betaSquared = PG_BETA_SQUARED_FLOOR;
  }
  out.clScale = 1 / Math.sqrt(betaSquared);
  out.cdAdd = lookup1d(WAVE_DRAG_TABLE, normalMach);
  out.acShift = lookup1d(AC_SHIFT_TABLE, normalMach);
  out.controlScale = lookup1d(CONTROL_TABLE, normalMach);
  out.clMaxScale = lookup1d(CL_MAX_TABLE, normalMach);
  return out;
}
