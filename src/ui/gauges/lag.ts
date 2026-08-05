/**
 * The mass and the damping of a needle.
 *
 *
 * WHY A NEEDLE MUST LAG
 *
 * A needle that jumps to its value reads as a printed number, not as a dial. A
 * real pointer hangs on a hairspring and it drags through air or through oil,
 * so it takes time to arrive. The pilot reads that movement as much as the
 * position: a needle that creeps says one thing and a needle that swings says
 * another.
 *
 * Every instrument here uses a FIRST ORDER lag, which is the simplest system
 * that has memory:
 *
 *   dx/dt = (target - x) / tau
 *
 * The step below is the exact solution of that equation over one frame, for a
 * target that holds still across the frame:
 *
 *   x <- x + (target - x) * (1 - exp(-dt / tau))
 *
 * The exact form matters. The plain Euler form `x += (target - x) * dt / tau`
 * OVERSHOOTS as soon as `dt` passes `tau`, and a frame of 33 ms against a
 * needle of 25 ms is a case this project really meets. The factor
 * `1 - exp(-dt / tau)` never leaves the range 0 to 1, so the needle can never
 * pass its target, whatever the frame time. The unit test states that fact.
 *
 * A first order lag reaches 63 percent of a step in one time constant and 95
 * percent in three. It never oscillates, so a needle never rings. A real
 * instrument does ring a little. The extra state is not worth its cost here.
 *
 * This module is pure math. It touches no DOM and no renderer.
 */

/** The state of one lagging needle. */
export interface NeedleLag {
  /** Where the needle stands now, in the unit of the instrument. */
  value: number;
  /** Time constant, s. A larger value makes a slower needle. */
  timeConstant: number;
  /** False until the first step. The first step puts the needle on its target. */
  settled: boolean;
}

/** Build one lag. A needle starts on its first target, not at zero. */
export function createLag(timeConstant: number, initial = 0): NeedleLag {
  return { value: initial, timeConstant, settled: false };
}

/** Put a needle straight onto a value and forget where it was. */
export function resetLag(lag: NeedleLag, value: number): void {
  lag.value = value;
  lag.settled = true;
}

/**
 * Advance one needle by one frame and return where it now stands.
 *
 * The FIRST call snaps the needle onto the target. Without that snap every
 * needle would sweep up from zero when the pilot enters the cockpit, which no
 * instrument does. A time constant of zero or less also snaps.
 */
export function stepLag(lag: NeedleLag, target: number, dt: number): number {
  if (!lag.settled || lag.timeConstant <= 0 || !(dt > 0)) {
    lag.value = target;
    lag.settled = true;
    return lag.value;
  }
  lag.value += (target - lag.value) * (1 - Math.exp(-dt / lag.timeConstant));
  return lag.value;
}

/**
 * Advance one needle that runs around a circle, such as a compass card.
 *
 * The needle takes the SHORT way round. Without the wrap a card that passes
 * north would run the whole way back through south, which is 359 degrees of
 * travel for one degree of change.
 */
export function stepLagWrapped(
  lag: NeedleLag,
  target: number,
  period: number,
  dt: number,
): number {
  if (!lag.settled || lag.timeConstant <= 0 || !(dt > 0)) {
    lag.value = target;
    lag.settled = true;
    return lag.value;
  }
  let error = target - lag.value;
  // Bring the error into the range of half a period each way, which is the
  // short way round.
  error -= Math.round(error / period) * period;
  lag.value += error * (1 - Math.exp(-dt / lag.timeConstant));
  // Hold the state inside one period, so it cannot grow without bound. The
  // line above can carry the value half a period past each end at most, so one
  // correction is always enough.
  if (lag.value >= period) lag.value -= period;
  else if (lag.value < 0) lag.value += period;
  return lag.value;
}
