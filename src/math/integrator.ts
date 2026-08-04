/**
 * Classical Runge-Kutta 4 over a state in a Float64Array.
 *
 * The rigid body has its own integrator, because its state holds a quaternion
 * that needs a normalization. This one covers the simple checks in the flight
 * test harness, such as a first order lag or a point mass.
 *
 * The caller holds the scratch, so a step allocates nothing.
 *
 * This module is pure math. It imports nothing.
 */

/** Writes the derivative of y at time t into dy. */
export type Derivative = (t: number, y: Float64Array, dy: Float64Array) => void;

/** Work arrays for one state size. Make one and use it for every step. */
export interface Rk4Scratch {
  readonly k1: Float64Array;
  readonly k2: Float64Array;
  readonly k3: Float64Array;
  readonly k4: Float64Array;
  readonly stage: Float64Array;
}

/** Makes the scratch for a state of n values. */
export function createRk4Scratch(n: number): Rk4Scratch {
  if (!Number.isInteger(n) || n <= 0) {
    throw new Error(`createRk4Scratch needs a positive whole state size. It got ${n}.`);
  }
  return {
    k1: new Float64Array(n),
    k2: new Float64Array(n),
    k3: new Float64Array(n),
    k4: new Float64Array(n),
    stage: new Float64Array(n),
  };
}

/**
 * Steps y forward by dt. The function changes y in place.
 *
 * The scratch must have the same length as y.
 */
export function rk4(
  f: Derivative,
  t: number,
  y: Float64Array,
  dt: number,
  scratch: Rk4Scratch,
): void {
  const n = y.length;
  const { k1, k2, k3, k4, stage } = scratch;
  if (k1.length !== n) {
    throw new Error(
      `rk4 needs a scratch of the same length as the state. The state holds ${n} values ` +
        `and the scratch holds ${k1.length}.`,
    );
  }
  const half = 0.5 * dt;

  f(t, y, k1);
  for (let i = 0; i < n; i++) {
    stage[i] = y[i] + half * k1[i];
  }

  f(t + half, stage, k2);
  for (let i = 0; i < n; i++) {
    stage[i] = y[i] + half * k2[i];
  }

  f(t + half, stage, k3);
  for (let i = 0; i < n; i++) {
    stage[i] = y[i] + dt * k3[i];
  }

  f(t + dt, stage, k4);
  const sixth = dt / 6;
  for (let i = 0; i < n; i++) {
    y[i] += sixth * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
}
