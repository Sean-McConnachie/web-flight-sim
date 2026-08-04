/**
 * Two dimensional airfoil coefficients over the full circle of angle of attack.
 *
 * A blade element model needs lift, drag, and moment at any angle, not only in
 * the small attached range. A spinning aircraft, a tail in a deep stall, and a
 * wing in a tail slide all see angles far outside the normal range. This module
 * builds one continuous curve from -PI to +PI for each section.
 *
 * The curve joins three regions.
 *
 *   1. Attached flow. A Kirchhoff lift law with a trailing edge separation
 *      point. The law is linear near zero and bends over near the stall.
 *   2. Separated flow. The curve blends down from the attached law. The drag
 *      rises fast. The center of pressure moves back from the quarter chord.
 *   3. Flat plate. Past about 25 degrees the section acts as a flat plate.
 *      The normal force coefficient is 2 sin(alpha), which gives
 *      cl = 2 sin(alpha) cos(alpha) and cd = 2 sin(alpha)^2.
 *
 * Every join uses smoothstep, so the curve and its slope stay continuous. A step
 * in the force at a join would shake the aircraft apart.
 *
 * Both Me-262 sections are symmetric, so the whole curve is odd in alpha for
 * lift and moment and even in alpha for drag. The builder makes the positive
 * half and mirrors it.
 *
 * The builder samples each section into a Table1D once. The flight step reads
 * the table with lookupCyclic. This keeps the cost of one element to three
 * interpolations and no allocation.
 *
 * This module is a pure coefficient source. It imports only from src/math.
 */

import { lerp, lookupCyclic, smoothstep, table1d } from '@/math/tables';
import type { Table1D } from '@/math/tables';
import { DEG } from '@/math/units';

/** Lift, drag, and moment coefficient of a section. The moment is about the quarter chord. */
export interface AeroCoefficients {
  cl: number;
  cd: number;
  cm: number;
}

export interface Airfoil {
  readonly name: string;
  readonly clAlpha: number; // 2D lift curve slope, per radian
  readonly alphaZeroLift: number; // rad
  readonly alphaStall: number; // rad, positive side
  readonly cdMin: number;
  readonly thickness: number; // t / c
  /** alpha in radians, valid over the full circle. Writes into out. */
  sample(alpha: number, out: AeroCoefficients): AeroCoefficients;
}

/** The input of makeSymmetricAirfoil. Only the first five fields are necessary. */
export interface AirfoilSpec {
  readonly name: string;
  readonly thickness: number; // t / c
  readonly clMax: number; // peak lift coefficient of the positive side
  readonly alphaStall: number; // rad, the angle of the peak
  readonly cdMin: number; // drag coefficient at zero lift
  /** Per radian. The default comes from thin airfoil theory with a thickness correction. */
  readonly clAlpha?: number;
  /** rad. Above this angle the section follows the flat plate law. */
  readonly alphaFlat?: number;
}

const TWO_PI = 2 * Math.PI;

// Thin airfoil theory gives a section lift curve slope of 2 PI per radian for a
// plate of zero thickness. Thickness turns the slope up. Abbott and von
// Doenhoff, "Theory of Wing Sections", give the correction as a factor of
// (1 + 0.77 t/c). Source: Abbott and von Doenhoff, section 4.3, confidence: firm.
//
// Wind tunnel sections measure a slope near 0.107 per degree, that is 6.13 per
// radian, because the boundary layer thickens the effective section near the
// trailing edge and takes lift away. The separation function f below carries
// that loss, so the model keeps the inviscid slope here and lets f bend the
// curve over before the stall.
const THICKNESS_SLOPE_FACTOR = 0.77;

/** Returns the inviscid 2D lift curve slope of a section of the given thickness. */
export function thinAirfoilSlope(thickness: number): number {
  return TWO_PI * (1 + THICKNESS_SLOPE_FACTOR * thickness);
}

// The angle band between the lift peak and full flat plate flow. The section
// loses the last of its attached flow across this band. Source: 360 degree
// section data of the type Sandia published for symmetric sections shows the
// lift minimum between 20 and 30 degrees. Confidence: estimate.
const FLAT_PLATE_BAND = 12 * DEG; // rad

// The normal force coefficient of a flat plate at 90 degrees. The flat plate law
// cn = 2 sin(alpha) gives 2.0 here. Measured plates peak between 1.8 and 2.0, so
// the law runs a little high and the model accepts that.
// Source: Hoerner, "Fluid Dynamic Drag", chapter 3. Confidence: firm.
const CD_FLAT_PLATE_90 = 2.0;

// The center of pressure of a fully separated section, in chord fractions from
// the leading edge. A flat plate in separated flow carries its load near the
// middle. Source: Hoerner, "Fluid Dynamic Lift", chapter 4. Confidence: firm.
const X_CP_SEPARATED = 0.5;

// The aerodynamic center of a thin section in attached flow, in chord fractions.
const X_CP_ATTACHED = 0.25;

// The drag of a section that flies backwards is higher than the drag of the same
// section that flies forwards. The sharp trailing edge leads and the round nose
// trails, which makes a wide wake. The factor multiplies cdMin near 180 degrees.
// Confidence: estimate. The aircraft only reaches this range in a tumble.
const REVERSED_DRAG_FACTOR = 4;

// Drag polar of the attached region. The section drag is
//   cd = cdMin + K_POLAR cl^2 + K_BUCKET max(0, |cl| - CL_BUCKET)^2
// The first quadratic term is the drag creep of the whole polar. The second term
// starts at the edge of the low drag bucket, where the transition point jumps
// forward. Both Me-262 sections put maximum thickness at 35 or 40 percent chord,
// so they hold laminar flow over the front of the section and do show a bucket.
// The constants match the published polar of the NACA 0009 at Reynolds 3e6,
// where cd is 0.0055 at cl = 0 and about 0.0105 at cl = 1.0.
// Source: Abbott and von Doenhoff, appendix IV. Confidence: firm.
const K_POLAR = 0.0025;
const K_BUCKET = 0.0045;
const CL_BUCKET = 0.2;

// The fraction of the stall angle where the center of pressure and the drag rise
// start to leave their attached values. Separation reaches the trailing edge
// before the lift peaks, so both effects start below the peak.
const SEPARATION_ONSET_FRACTION = 0.8;

// The angle step of the sampled table, in radians.
//
// Justification. The sharpest feature of the curve is the lift peak at the
// stall. Its curvature is about 0.03 per square degree for these sections. A
// linear table with step h misses a peak by |curvature| h^2 / 8, which is
// 0.001 in cl at a step of 0.5 degrees. The published spread of section data is
// about 0.05 in cl, so the table error is 50 times smaller than the data it
// holds. A step of 0.5 degrees also puts a knot exactly on 13, 15, 45, and 90
// degrees, which are the angles the tests check.
//
// The cost is 721 knots for each of the three coefficients of each section,
// which is 17 kilobytes for each section. That is cheap next to one extra
// interpolation for every element of every step.
const ALPHA_STEP = 0.5 * DEG; // rad
const HALF_TURN_STEPS = Math.round(Math.PI / ALPHA_STEP); // 360

/**
 * The fit of the Leishman and Beddoes separation function to one section, plus
 * the angles that drive the blends.
 */
interface CurveFit {
  clAlpha: number;
  cdMin: number;
  alphaStall: number;
  alphaFlat: number;
  a1: number; // break angle of the separation function, rad
  s1: number; // shape constant below the break angle, rad
  s2: number; // shape constant above the break angle, rad
  separationOnset: number; // rad
}

/**
 * Steady trailing edge separation point, 1 attached and 0.04 fully separated.
 * This is the standard Leishman and Beddoes form.
 * Source: Leishman and Beddoes, "A Semi-Empirical Model for Dynamic Stall",
 * Journal of the American Helicopter Society, 1989. Confidence: firm.
 *
 * The same form and the same fit appear in src/physics/aero/stall.ts, which owns
 * the unsteady side of the model. The two modules stay separate because both
 * must import only from src/math.
 */
function separationPoint(absAlpha: number, a1: number, s1: number, s2: number): number {
  if (absAlpha <= a1) {
    return 1 - 0.3 * Math.exp((absAlpha - a1) / s1);
  }
  return 0.04 + 0.66 * Math.exp((a1 - absAlpha) / s2);
}

/**
 * Fits the break angle a1 and the shape constants s1 and s2 so that the
 * Kirchhoff lift law peaks at alphaStall with the value clMax.
 *
 * Method. The Kirchhoff law is cl = clAlpha g(f) alpha with
 * g(f) = ((1 + sqrt(f)) / 2)^2. Two conditions fix the fit.
 *
 *   Value.  clAlpha g(f) alphaStall = clMax, which inverts to
 *           sqrt(f) = 2 sqrt(g) - 1 with g = clMax / (clAlpha alphaStall).
 *   Slope.  d(cl)/d(alpha) = 0 at the peak, which gives
 *           df/dalpha = -g / (alphaStall g'(f)) with g'(f) = (1 + sqrt(f)) / (4 sqrt(f)).
 *
 * The peak always falls above the break angle, so the second branch of the
 * separation function holds. That branch gives df/dalpha = -(f - 0.04) / s2,
 * which fixes s2, and then the branch itself fixes a1.
 *
 * The last free constant is s1. The choice s1 = s2 / 2.2 makes the slope of f
 * continuous at the break angle, because the two branches have slopes -0.3/s1
 * and -0.66/s2 there.
 */
function fitSeparation(clAlpha: number, alphaStall: number, clMax: number): {
  a1: number;
  s1: number;
  s2: number;
} {
  const g = clMax / (clAlpha * alphaStall);
  const sqrtF = 2 * Math.sqrt(g) - 1;
  const f = sqrtF * sqrtF;
  if (!(f > 0.05) || !(f < 0.7)) {
    throw new Error(
      `The airfoil spec does not fit the Kirchhoff law. The separation point at ` +
        `the stall came out as ${f.toFixed(4)}, which must lie between 0.05 and 0.7. ` +
        `Check clMax against clAlpha times alphaStall.`,
    );
  }
  const gPrime = (1 + sqrtF) / (4 * sqrtF);
  const slope = -g / (alphaStall * gPrime); // df/dalpha at the peak
  const s2 = -(f - 0.04) / slope;
  const a1 = alphaStall + s2 * Math.log((f - 0.04) / 0.66);
  return { a1, s1: s2 / 2.2, s2 };
}

/**
 * Writes the coefficients of the positive half of the curve. absAlpha runs from
 * 0 to PI. The caller mirrors the result onto the negative half.
 */
function positiveHalf(absAlpha: number, fit: CurveFit, out: AeroCoefficients): void {
  const sinA = Math.sin(absAlpha);
  const cosA = Math.cos(absAlpha);

  // Region 1 and region 2. Kirchhoff lift with trailing edge separation.
  const f = separationPoint(absAlpha, fit.a1, fit.s1, fit.s2);
  const root = (1 + Math.sqrt(f)) * 0.5;
  const clAttached = fit.clAlpha * root * root * absAlpha;

  // Region 3. Flat plate. The normal force is 2 sin(alpha) and carries no axial
  // part, so cl = 2 sin cos and cd = 2 sin^2.
  const clFlat = CD_FLAT_PLATE_90 * sinA * cosA;

  // The lift blend starts at the peak, so the peak keeps its fitted value.
  const wLift = smoothstep(fit.alphaStall, fit.alphaFlat, absAlpha);
  const cl = lerp(clAttached, clFlat, wLift);

  // Attached drag polar with a low drag bucket.
  const overBucket = Math.max(0, Math.abs(clAttached) - CL_BUCKET);
  const cdAttached =
    fit.cdMin + K_POLAR * clAttached * clAttached + K_BUCKET * overBucket * overBucket;

  // Separated drag. The cos^2 term keeps a viscous floor at 0 and at 180 degrees
  // and vanishes with zero slope at 90 degrees, so it adds no kink there.
  const floorFactor = cosA >= 0 ? 1 : REVERSED_DRAG_FACTOR;
  const cdSeparated = CD_FLAT_PLATE_90 * sinA * sinA + fit.cdMin * cosA * cosA * floorFactor;

  // The drag rise and the movement of the center of pressure both start before
  // the lift peak, because separation reaches the trailing edge first. One
  // weight drives both.
  const wSeparated = smoothstep(fit.separationOnset, fit.alphaFlat, absAlpha);
  const cd = lerp(cdAttached, cdSeparated, wSeparated);

  // The center of pressure moves back from the quarter chord to the middle of
  // the chord as the flow separates. A load behind the quarter chord makes a
  // nose down moment, so cm goes negative.
  const xCp = X_CP_ATTACHED + (X_CP_SEPARATED - X_CP_ATTACHED) * wSeparated;
  const cn = cl * cosA + cd * sinA;
  const cm = -(xCp - X_CP_ATTACHED) * cn;

  out.cl = cl;
  out.cd = cd;
  out.cm = cm;
}

/** Joins three tables and the scalar data into an Airfoil. */
function tabulatedAirfoil(
  name: string,
  clAlpha: number,
  alphaStall: number,
  cdMin: number,
  thickness: number,
  clTable: Table1D,
  cdTable: Table1D,
  cmTable: Table1D,
): Airfoil {
  return {
    name,
    clAlpha,
    alphaZeroLift: 0,
    alphaStall,
    cdMin,
    thickness,
    sample(alpha: number, out: AeroCoefficients): AeroCoefficients {
      out.cl = lookupCyclic(clTable, alpha, TWO_PI);
      out.cd = lookupCyclic(cdTable, alpha, TWO_PI);
      out.cm = lookupCyclic(cmTable, alpha, TWO_PI);
      return out;
    },
  };
}

/** Builds the shared angle axis of every table, from -PI to +PI. */
function buildAngleAxis(): number[] {
  const n = 2 * HALF_TURN_STEPS + 1;
  const x = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    x[i] = (i - HALF_TURN_STEPS) * ALPHA_STEP;
  }
  return x;
}

/**
 * Builds a symmetric section from its published data. The section has zero
 * camber, so the zero lift angle is zero and the moment about the quarter chord
 * is zero in attached flow.
 */
export function makeSymmetricAirfoil(spec: AirfoilSpec): Airfoil {
  const clAlpha = spec.clAlpha ?? thinAirfoilSlope(spec.thickness);
  const alphaFlat = spec.alphaFlat ?? spec.alphaStall + FLAT_PLATE_BAND;
  const fitted = fitSeparation(clAlpha, spec.alphaStall, spec.clMax);
  const fit: CurveFit = {
    clAlpha,
    cdMin: spec.cdMin,
    alphaStall: spec.alphaStall,
    alphaFlat,
    a1: fitted.a1,
    s1: fitted.s1,
    s2: fitted.s2,
    separationOnset: SEPARATION_ONSET_FRACTION * spec.alphaStall,
  };

  const x = buildAngleAxis();
  const n = x.length;
  const cl = new Array<number>(n);
  const cd = new Array<number>(n);
  const cm = new Array<number>(n);
  const scratch: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };

  for (let i = 0; i < n; i++) {
    const alpha = x[i];
    positiveHalf(Math.abs(alpha), fit, scratch);
    // Lift and moment are odd in alpha. Drag is even.
    const sign = alpha < 0 ? -1 : 1;
    cl[i] = sign * scratch.cl;
    cd[i] = scratch.cd;
    cm[i] = sign * scratch.cm;
  }

  return tabulatedAirfoil(
    spec.name,
    clAlpha,
    spec.alphaStall,
    spec.cdMin,
    spec.thickness,
    table1d(x, cl),
    table1d(x, cd),
    table1d(x, cm),
  );
}

/**
 * Builds the section of a wing strip between a root section and a tip section.
 * At t = 0 the result matches a. At t = 1 it matches b.
 *
 * The function resamples both sections onto the shared angle axis and blends the
 * samples. The result is one Airfoil with its own tables, so a strip still costs
 * one interpolation for each coefficient.
 */
export function blendAirfoils(a: Airfoil, b: Airfoil, t: number, name: string): Airfoil {
  const x = buildAngleAxis();
  const n = x.length;
  const cl = new Array<number>(n);
  const cd = new Array<number>(n);
  const cm = new Array<number>(n);
  const sa: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
  const sb: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };

  for (let i = 0; i < n; i++) {
    a.sample(x[i], sa);
    b.sample(x[i], sb);
    cl[i] = lerp(sa.cl, sb.cl, t);
    cd[i] = lerp(sa.cd, sb.cd, t);
    cm[i] = lerp(sa.cm, sb.cm, t);
  }

  return tabulatedAirfoil(
    name,
    lerp(a.clAlpha, b.clAlpha, t),
    lerp(a.alphaStall, b.alphaStall, t),
    lerp(a.cdMin, b.cdMin, t),
    lerp(a.thickness, b.thickness, t),
    table1d(x, cl),
    table1d(x, cd),
    table1d(x, cm),
  );
}

// Me-262 tip section, NACA 00009-1.1-40. The model treats it as a plain NACA
// 0009. The modified nose radius and the maximum thickness at 40 percent chord
// change the drag bucket and the critical Mach number, not the shape of the lift
// curve.
//
// Section data at Reynolds 3e6: cdMin 0.0055 and clMax 1.25 at 13 degrees.
// The published NACA 0009 reaches clMax 1.35 at Reynolds 6e6 and about 1.25 at
// Reynolds 3e6, which confirms the values the brief gave. Minimum drag of a
// smooth NACA 0009 is 0.0052 at Reynolds 6e6 and 0.0055 at Reynolds 3e6, which
// also confirms the brief. Source: Abbott and von Doenhoff, appendix IV.
// Confidence: firm.
export const NACA_0009: Airfoil = makeSymmetricAirfoil({
  name: 'NACA 0009',
  thickness: 0.09,
  clMax: 1.25,
  alphaStall: 13 * DEG,
  cdMin: 0.0055,
});

// Me-262 root section, NACA 00011-0.825-35. The model treats it as a plain NACA
// 0011.
//
// Abbott and von Doenhoff publish the 0009, the 0012, and the 0015, but not the
// 0011. The 0012 reaches clMax 1.55 at Reynolds 6e6 with cdMin 0.0060, and the
// 0009 reaches 1.35 with cdMin 0.0052. Linear interpolation on thickness puts
// the 0011 at clMax 1.48 and cdMin 0.0057 at Reynolds 6e6. The drop from
// Reynolds 6e6 to 3e6 takes about 0.10 off clMax and adds about 0.0003 to cdMin,
// which gives clMax 1.38 and cdMin 0.0060. The brief gave 1.35 and 0.0060, which
// sits inside that estimate, so the model keeps the brief. The stall angle of 15
// degrees follows the thicker nose of the 0011.
// Confidence: derived from firm 0009 and 0012 data.
export const NACA_0011: Airfoil = makeSymmetricAirfoil({
  name: 'NACA 0011',
  thickness: 0.11,
  clMax: 1.35,
  alphaStall: 15 * DEG,
  cdMin: 0.006,
});
