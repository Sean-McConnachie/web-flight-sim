/**
 * Deterministic random numbers.
 *
 * The world scatter, the turbulence, and any test that needs noise use this
 * module. The same seed gives the same sequence on every run and on every
 * machine. Do not call Math.random in code that must repeat.
 */

const TWO_PI = 2 * Math.PI;

/** 2 to the power 32. The generator divides by this value to reach [0, 1). */
const UINT32_SPAN = 4294967296;

/**
 * Builds a mulberry32 generator. The generator returns a number in [0, 1).
 *
 * mulberry32 holds one 32 bit word of state. It passes the gjrand test suite
 * and it costs about ten machine operations per draw.
 * Source: Tommy Ettinger, public domain reference code, confidence: firm.
 */
export function mulberry32(seed: number): () => number {
  let state = seed >>> 0;
  return function next(): number {
    state = (state + 0x6d2b79f5) >>> 0;
    let word = state;
    word = Math.imul(word ^ (word >>> 15), word | 1);
    word ^= word + Math.imul(word ^ (word >>> 7), word | 61);
    return ((word ^ (word >>> 14)) >>> 0) / UINT32_SPAN;
  };
}

export interface Rng {
  /** Returns a number in [0, 1). */
  next(): number;

  /** Returns a number in [lo, hi). */
  range(lo: number, hi: number): number;

  /** Returns a whole number in [lo, hi). `hi` must be larger than `lo`. */
  int(lo: number, hi: number): number;

  /** Returns one item of `items`. The array must hold at least one item. */
  pick<T>(items: readonly T[]): T;

  /** Returns an angle in [0, 2 pi), in radians. */
  angle(): number;

  /** Returns a normal value with mean 0 and standard deviation 1. */
  gaussian(): number;
}

/** Builds the random number source that the whole application shares. */
export function createRng(seed: number): Rng {
  const next = mulberry32(seed);

  // The Box-Muller transform makes two normal values at a time. The second
  // value waits here until the next call.
  let spare = 0;
  let spareIsReady = false;

  return {
    next,

    range(lo: number, hi: number): number {
      return lo + next() * (hi - lo);
    },

    int(lo: number, hi: number): number {
      const value = Math.floor(lo + next() * (hi - lo));
      // Rounding at the top of a wide span can reach `hi`. Hold the result
      // inside the half open range.
      return value < hi ? value : hi - 1;
    },

    pick<T>(items: readonly T[]): T {
      if (items.length === 0) {
        throw new RangeError('pick needs an array with at least one item.');
      }
      return items[Math.floor(next() * items.length)];
    },

    angle(): number {
      return next() * TWO_PI;
    },

    gaussian(): number {
      if (spareIsReady) {
        spareIsReady = false;
        return spare;
      }
      // The log of zero is not finite, so skip a draw of exactly zero.
      let unit = next();
      while (unit === 0) unit = next();
      const radius = Math.sqrt(-2 * Math.log(unit));
      const theta = TWO_PI * next();
      spare = radius * Math.sin(theta);
      spareIsReady = true;
      return radius * Math.cos(theta);
    },
  };
}
