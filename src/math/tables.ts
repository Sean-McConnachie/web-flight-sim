/**
 * Lookup tables and scalar helpers.
 *
 * The aerodynamic model reads these tables 240 times per second for each
 * element. Two rules follow from that rate. A lookup allocates nothing. A lookup
 * finds its interval with a binary search, not with a scan.
 *
 * A table builder copies the input into a Float64Array and checks the shape. A
 * lookup trusts the table and does no check.
 *
 * This module is pure math. It imports nothing.
 */

export interface Table1D {
  readonly x: Float64Array;
  readonly y: Float64Array;
}

export interface Table2D {
  readonly x: Float64Array; // columns
  readonly y: Float64Array; // rows
  readonly z: Float64Array; // row major, index = iy * x.length + ix
}

/**
 * Returns the index i of the interval that holds v, with x[i] <= v < x[i + 1].
 * The caller must first clamp v into the range x[0] to x[n - 1]. The search
 * keeps the invariant x[lo] <= v < x[hi].
 */
function findInterval(x: Float64Array, v: number): number {
  let lo = 0;
  let hi = x.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (x[mid] <= v) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

/** Throws if the values do not increase strictly. The name goes in the message. */
function checkStrictlyIncreasing(values: readonly number[], name: string): void {
  for (let i = 1; i < values.length; i++) {
    // The negated test also catches a NaN, because every compare with a NaN is false.
    if (!(values[i] > values[i - 1])) {
      throw new Error(
        `${name} must increase strictly. Index ${i - 1} holds ${values[i - 1]} ` +
          `and index ${i} holds ${values[i]}.`,
      );
    }
  }
}

/**
 * Builds a one dimensional table. The knots in x must increase strictly. The
 * arrays x and y must have the same length.
 */
export function table1d(x: number[], y: number[]): Table1D {
  if (x.length === 0) {
    throw new Error('table1d needs one knot at least. The array x is empty.');
  }
  if (x.length !== y.length) {
    throw new Error(
      `table1d needs x and y of the same length. x holds ${x.length} values ` +
        `and y holds ${y.length} values.`,
    );
  }
  checkStrictlyIncreasing(x, 'table1d x');
  return { x: Float64Array.from(x), y: Float64Array.from(y) };
}

/**
 * Reads the table at x with linear interpolation. The result clamps to the first
 * value below the first knot and to the last value above the last knot.
 */
export function lookup1d(t: Table1D, x: number): number {
  const xs = t.x;
  const ys = t.y;
  const last = xs.length - 1;
  if (x <= xs[0]) {
    return ys[0];
  }
  if (x >= xs[last]) {
    return ys[last];
  }
  const i = findInterval(xs, x);
  const x0 = xs[i];
  const f = (x - x0) / (xs[i + 1] - x0);
  const y0 = ys[i];
  return y0 + (ys[i + 1] - y0) * f;
}

/**
 * Reads a cyclic table at x. The function wraps x into the table range modulo
 * the period, then interpolates. A 360 degree airfoil table uses this function.
 *
 * The last knot joins the first knot again at x[0] + period. The function
 * interpolates across that seam. If the table already spans the full period,
 * that is x[n - 1] equals x[0] + period, the wrap keeps x below the last knot
 * and the seam never runs.
 */
export function lookupCyclic(t: Table1D, x: number, period: number): number {
  const xs = t.x;
  const ys = t.y;
  const last = xs.length - 1;
  if (last === 0) {
    return ys[0];
  }
  const first = xs[0];
  // Wrap into the half open range first to first + period.
  const offset = x - first;
  const wrapped = first + (offset - Math.floor(offset / period) * period);
  const end = xs[last];
  if (wrapped >= end) {
    const span = first + period - end;
    if (span <= 0) {
      return ys[last];
    }
    const f = (wrapped - end) / span;
    const yEnd = ys[last];
    return yEnd + (ys[0] - yEnd) * f;
  }
  const i = findInterval(xs, wrapped);
  const x0 = xs[i];
  const f = (wrapped - x0) / (xs[i + 1] - x0);
  const y0 = ys[i];
  return y0 + (ys[i + 1] - y0) * f;
}

/**
 * Builds a two dimensional table. The caller gives z as z[iy][ix], that is one
 * inner array for each row y. The builder flattens z in row major order. The
 * knots in x and in y must increase strictly.
 */
export function table2d(x: number[], y: number[], z: number[][]): Table2D {
  if (x.length === 0 || y.length === 0) {
    throw new Error('table2d needs one knot at least on each axis.');
  }
  if (z.length !== y.length) {
    throw new Error(
      `table2d needs one row of z for each knot in y. y holds ${y.length} knots ` +
        `and z holds ${z.length} rows.`,
    );
  }
  checkStrictlyIncreasing(x, 'table2d x');
  checkStrictlyIncreasing(y, 'table2d y');
  const nx = x.length;
  const flat = new Float64Array(nx * y.length);
  for (let iy = 0; iy < y.length; iy++) {
    const row = z[iy];
    if (row.length !== nx) {
      throw new Error(
        `table2d needs one z value for each knot in x. Row ${iy} holds ` +
          `${row.length} values and x holds ${nx} knots.`,
      );
    }
    for (let ix = 0; ix < nx; ix++) {
      flat[iy * nx + ix] = row[ix];
    }
  }
  return { x: Float64Array.from(x), y: Float64Array.from(y), z: flat };
}

/**
 * Reads the table at x and y with bilinear interpolation. The result clamps on
 * both axes.
 */
export function lookup2d(t: Table2D, x: number, y: number): number {
  const xs = t.x;
  const ys = t.y;
  const z = t.z;
  const nx = xs.length;
  const lastX = nx - 1;
  const lastY = ys.length - 1;

  let ix = 0;
  let fx = 0;
  if (lastX > 0) {
    if (x >= xs[lastX]) {
      ix = lastX - 1;
      fx = 1;
    } else if (x > xs[0]) {
      ix = findInterval(xs, x);
      fx = (x - xs[ix]) / (xs[ix + 1] - xs[ix]);
    }
  }

  let iy = 0;
  let fy = 0;
  if (lastY > 0) {
    if (y >= ys[lastY]) {
      iy = lastY - 1;
      fy = 1;
    } else if (y > ys[0]) {
      iy = findInterval(ys, y);
      fy = (y - ys[iy]) / (ys[iy + 1] - ys[iy]);
    }
  }

  // A single knot axis makes both corners on that axis the same cell.
  const stepX = lastX > 0 ? 1 : 0;
  const stepY = lastY > 0 ? nx : 0;
  const i00 = iy * nx + ix;
  const i10 = i00 + stepX;
  const i01 = i00 + stepY;
  const i11 = i01 + stepX;

  const z00 = z[i00];
  const z01 = z[i01];
  const lowRow = z00 + (z[i10] - z00) * fx;
  const highRow = z01 + (z[i11] - z01) * fx;
  return lowRow + (highRow - lowRow) * fy;
}

/** Returns the linear blend of a and b. At t equal to 0 the result is a. */
export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/** Returns the position of v between a and b. This is the inverse of lerp. */
export function inverseLerp(a: number, b: number, v: number): number {
  return (v - a) / (b - a);
}

/** Limits v to the range lo to hi. */
export function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/**
 * Returns a smooth step from 0 to 1 between edge0 and edge1. The slope is zero
 * at both edges, so a blend with this function makes no kink.
 */
export function smoothstep(edge0: number, edge1: number, x: number): number {
  const t = clamp((x - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

/**
 * Wraps an angle in radians into the range -PI to PI. The lower limit is part of
 * the range. The upper limit is not, so PI gives -PI.
 */
export function wrapPi(angle: number): number {
  const twoPi = 2 * Math.PI;
  let a = (angle + Math.PI) % twoPi;
  if (a < 0) {
    a += twoPi;
  }
  return a - Math.PI;
}
