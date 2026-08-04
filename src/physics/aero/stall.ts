/**
 * Trailing edge separation and dynamic stall.
 *
 * The static airfoil tables in src/physics/aero/airfoil.ts hold the coefficients
 * of a section that sits at a fixed angle in a steady stream. A real wing never
 * does that. When the pilot pulls, the flow needs time to separate, so the wing
 * carries more lift than the static table promises and then loses it late. That
 * lag is dynamic stall. It is the reason a fast pull gives a sharp break and a
 * slow pull gives a soft one.
 *
 * The model has two parts.
 *
 *   1. A steady separation point f, from 1 fully attached to 0.04 fully
 *      separated. This is the Kirchhoff and Helmholtz picture of a section with
 *      the flow attached over the front f of the chord.
 *   2. A first order lag that carries f toward its steady value. The lag runs in
 *      the non-dimensional time of the section, so a short chord at high speed
 *      settles fast and a long chord at low speed settles slowly.
 *
 * Source of the whole model: Leishman and Beddoes, "A Semi-Empirical Model for
 * Dynamic Stall", Journal of the American Helicopter Society 34(3), 1989.
 * Confidence: firm.
 *
 * How to use this module. The Kirchhoff law holds near the stall only. It bends
 * the attached curve over and it puts the peak in the right place, but a few
 * degrees past the peak it starts to climb again, because a trailing edge model
 * cannot describe a fully separated section. Do not read lift from
 * kirchhoffLift at a large angle of attack. Read the static coefficients from
 * the airfoil table, which hands over to the flat plate law, and apply the
 * unsteady part of this module as a ratio:
 *
 *   dynamic = static * kirchhoffLift(..., fLagged) / kirchhoffLift(..., fSteady)
 *
 * The ratio is 1 in steady flow at any angle, so the static table stays in
 * charge. The ratio leaves 1 only while alpha moves, which is the effect this
 * module exists to give.
 *
 * This module is a pure coefficient source. It imports only from src/math.
 */

import { clamp } from '@/math/tables';
import { DEG } from '@/math/units';

/** The state of one aerodynamic element. The caller keeps it between steps. */
export interface StallState {
  /** The lagged separation point, 1 attached and 0.04 fully separated. */
  f: number;
}

/** The fit of the separation function and the lag of one section. */
export interface StallParams {
  /** The break angle. Above it the separation point falls fast. rad. */
  readonly a1: number;
  /** The shape constant below the break angle. rad. */
  readonly s1: number;
  /** The shape constant above the break angle. rad. */
  readonly s2: number;
  /** The lag constant in semi-chords of travel. Dimensionless. */
  readonly tf: number;
}

// The lag constant of the separation point, in semi-chords of travel. Leishman
// and Beddoes fitted Tf = 3.0 to oscillating NACA 0012 data and used the same
// value across the Mach range of the tests. The non-dimensional time is
// s = 2 V t / c, so a lag of Tf in s is a lag of Tf c / (2 V) in seconds.
// Source: Leishman and Beddoes, 1989, table of model constants. Confidence: firm.
export const DEFAULT_TF = 3.0;

// The lowest speed the lag may use, in meters per second.
//
// The time constant is Tf c / (2 V). At V = 0 it is infinite, so f would freeze
// at whatever value it held when the aircraft stopped. A parked aircraft would
// then keep a stale separation state and would answer the first gust with the
// wrong lift. The floor keeps the time constant finite and lets f relax to its
// steady value on the ground. At 1 m/s and a chord of 2 m the constant is 3 s,
// which is slow but bounded. The floor never acts in flight, because the stall
// speed of the Me-262 is near 49 m/s.
export const MIN_LAG_SPEED = 1.0; // m/s

/** Creates the state of one element. A new element starts with attached flow. */
export function createStallState(): StallState {
  return { f: 1 };
}

/**
 * Returns the steady separation point at the given angle of attack.
 *
 * The function is even in alpha, because both Me-262 sections are symmetric.
 * The two branches meet at f = 0.7 at the break angle.
 */
export function steadySeparation(alpha: number, p: StallParams): number {
  const a = Math.abs(alpha);
  if (a <= p.a1) {
    return 1 - 0.3 * Math.exp((a - p.a1) / p.s1);
  }
  return 0.04 + 0.66 * Math.exp((p.a1 - a) / p.s2);
}

/**
 * Steps the lagged separation point of one element and returns the new value.
 *
 * alphaSteady is the angle of attack that sets the steady target, in radians.
 * chord and speed give the non-dimensional time of the section.
 *
 * The step uses the exact solution of the first order lag over dt, not an Euler
 * step. The exact form stays stable at any dt, which matters because the flight
 * step may run at 240 Hz while the time constant of a short chord at high speed
 * falls below one millisecond.
 */
export function updateSeparation(
  s: StallState,
  alphaSteady: number,
  chord: number,
  speed: number,
  dt: number,
  p: StallParams,
): number {
  const target = steadySeparation(alphaSteady, p);
  if (!(dt > 0)) {
    return s.f;
  }
  const v = Math.max(speed, MIN_LAG_SPEED);
  const tau = (p.tf * chord) / (2 * v);
  if (!(tau > 0)) {
    // A zero chord has no flow to convect, so the element follows the steady value.
    s.f = target;
    return s.f;
  }
  const gain = 1 - Math.exp(-dt / tau);
  s.f += (target - s.f) * gain;
  return s.f;
}

/**
 * Kirchhoff lift with a separated trailing edge.
 *
 * At f = 1 the law gives the full attached slope. At f = 0.04 it gives about
 * 36 percent of it, which is the residual lift of a fully separated section.
 */
export function kirchhoffLift(
  clAlpha: number,
  alpha: number,
  alphaZeroLift: number,
  f: number,
): number {
  const root = (1 + Math.sqrt(clamp(f, 0, 1))) * 0.5;
  return clAlpha * root * root * (alpha - alphaZeroLift);
}

// The center of pressure in attached flow and in fully separated flow, in chord
// fractions from the leading edge. These match src/physics/aero/airfoil.ts.
const X_CP_ATTACHED = 0.25;
const X_CP_SEPARATED = 0.5;

/**
 * Returns the center of pressure of the element, in chord fractions from the
 * leading edge. Attached flow holds it at the quarter chord. Full separation
 * moves it to the middle of the chord, which makes the nose down break of a
 * stall.
 *
 * The map is linear in f. The floor of f at 0.04 stops it a little short of the
 * middle, at 0.49 chord.
 */
export function separationCenterOfPressure(f: number): number {
  return X_CP_ATTACHED + (X_CP_SEPARATED - X_CP_ATTACHED) * (1 - clamp(f, 0, 1));
}

/**
 * Fits a1, s1, and s2 so that the Kirchhoff law peaks at alphaStall with the
 * value clMax.
 *
 * Method. Write g(f) = ((1 + sqrt(f)) / 2)^2, so the Kirchhoff law is
 * cl = clAlpha g(f) alpha. Two conditions fix the fit.
 *
 *   Value.  g at the peak is clMax / (clAlpha alphaStall), which inverts to
 *           sqrt(f) = 2 sqrt(g) - 1.
 *   Slope.  d(cl)/d(alpha) = 0 at the peak, so
 *           df/dalpha = -g / (alphaStall g'(f)), with g'(f) = (1 + sqrt(f)) / (4 sqrt(f)).
 *
 * The fitted f at the peak always falls below 0.7, so the upper branch of the
 * separation function holds there. That branch has df/dalpha = -(f - 0.04) / s2,
 * which gives s2, and the branch itself then gives a1.
 *
 * The last constant is s1. The two branches have slopes -0.3/s1 and -0.66/s2 at
 * the break angle, so s1 = s2 / 2.2 makes the slope of f continuous.
 *
 * src/physics/aero/airfoil.ts holds the same fit. The two modules stay separate
 * because both must import only from src/math.
 */
export function fitStallParams(
  clAlpha: number,
  alphaStall: number,
  clMax: number,
  tf: number = DEFAULT_TF,
): StallParams {
  const g = clMax / (clAlpha * alphaStall);
  const sqrtF = 2 * Math.sqrt(g) - 1;
  const f = sqrtF * sqrtF;
  if (!(f > 0.05) || !(f < 0.7)) {
    throw new Error(
      `The section does not fit the Kirchhoff law. The separation point at the ` +
        `stall came out as ${f.toFixed(4)}, which must lie between 0.05 and 0.7.`,
    );
  }
  const gPrime = (1 + sqrtF) / (4 * sqrtF);
  const slope = -g / (alphaStall * gPrime);
  const s2 = -(f - 0.04) / slope;
  const a1 = alphaStall + s2 * Math.log((f - 0.04) / 0.66);
  return { a1, s1: s2 / 2.2, s2, tf };
}

// The two Me-262 sections. The lift curve slope, the stall angle, and the peak
// lift match src/physics/aero/airfoil.ts, so the unsteady model and the static
// tables agree at zero pitch rate. test/unit/stall.test.ts checks that.
//
// The fit gives a break angle of about 12.6 degrees for the tip section and
// about 13.4 degrees for the root section, with s2 near 5.5 and 5.9 degrees.
// The root section breaks later and holds a wider band, which is what a thicker
// nose does. The root therefore stalls after the tip on a wing that twists, and
// the aileron keeps its bite through the break.
const SLOPE_0009 = 2 * Math.PI * (1 + 0.77 * 0.09); // per rad
const SLOPE_0011 = 2 * Math.PI * (1 + 0.77 * 0.11); // per rad

export const STALL_NACA_0009: StallParams = fitStallParams(SLOPE_0009, 13 * DEG, 1.25);
export const STALL_NACA_0011: StallParams = fitStallParams(SLOPE_0011, 15 * DEG, 1.35);
