/**
 * Fixed step loop.
 *
 * The physics runs at a constant rate. The renderer runs at the rate of the
 * display. This module keeps the two apart. Each frame it steps the physics a
 * whole number of times, then it gives the renderer the interpolation factor
 * `alpha`. The renderer blends the last physics state with the current one, so
 * the picture stays smooth at any frame rate.
 *
 * The loop uses `requestAnimationFrame` and `performance.now` as defaults only.
 * The caller can pass a different clock and a different scheduler. A test in
 * Node uses that to drive the loop with no browser.
 *
 * Read docs/CONVENTIONS.md before you change this file.
 */

import { config } from './config';

/** Physics steps per second. */
export const PHYSICS_HZ = config.physics.rate;

/** Length of one physics step, in seconds. */
export const PHYSICS_DT = 1 / PHYSICS_HZ;

/** Largest real time the loop simulates in one frame, in seconds. */
const ACCUMULATOR_CAP = config.physics.accumulatorCap;

/** Weight of the newest sample in the smoothed statistics. */
const SMOOTHING = 0.1;

export interface LoopCallbacks {
  /**
   * Runs one physics step. `dt` is always PHYSICS_DT. `time` is the simulated
   * time at the start of the step, in seconds.
   */
  fixedUpdate(dt: number, time: number): void;

  /**
   * Draws one frame. `alpha` is the position between the last physics state
   * and the current one, from 0 to 1. `frameDt` is the real time since the
   * last frame, in seconds.
   */
  render(alpha: number, frameDt: number): void;
}

export interface LoopStats {
  /** Smoothed frames per second. */
  fps: number;

  /** Number of physics steps in the last frame. */
  physicsStepsLastFrame: number;

  /** Total real time the cap threw away, in seconds. */
  droppedTime: number;

  /** Smoothed time of all physics steps in one frame, in milliseconds. */
  fixedUpdateMs: number;

  /** Smoothed time of the render call, in milliseconds. */
  renderMs: number;

  /** Time the physics has simulated, in seconds. */
  simTime: number;
}

/** Returns the current time in milliseconds. */
export type TimeSource = () => number;

/** Asks for one frame and returns a handle that cancels it. */
export type FrameScheduler = (frame: () => void) => number;

/** Cancels a frame that the scheduler gave a handle for. */
export type FrameCanceller = (handle: number) => void;

export interface LoopOptions {
  /** Defaults to `performance.now`. */
  now?: TimeSource;

  /** Defaults to `requestAnimationFrame`. */
  schedule?: FrameScheduler;

  /** Defaults to `cancelAnimationFrame`. */
  cancel?: FrameCanceller;
}

export interface Loop {
  start(): void;
  stop(): void;
  readonly stats: LoopStats;
}

// The default sources stay inside a function body. Node has no
// requestAnimationFrame, and a body runs only when the caller keeps the
// default.
const defaultNow: TimeSource = () => performance.now();
const defaultSchedule: FrameScheduler = (frame) => requestAnimationFrame(frame);
const defaultCancel: FrameCanceller = (handle) => {
  cancelAnimationFrame(handle);
};

function smooth(previous: number, sample: number): number {
  return previous + (sample - previous) * SMOOTHING;
}

export function createLoop(callbacks: LoopCallbacks, options: LoopOptions = {}): Loop {
  const now = options.now ?? defaultNow;
  const schedule = options.schedule ?? defaultSchedule;
  const cancel = options.cancel ?? defaultCancel;

  const stats: LoopStats = {
    fps: 0,
    physicsStepsLastFrame: 0,
    droppedTime: 0,
    fixedUpdateMs: 0,
    renderMs: 0,
    simTime: 0,
  };

  let running = false;
  let handle = 0;

  /** Whole physics steps since the loop first started. */
  let steps = 0;

  /** Real time, in seconds, that matches step zero. */
  let origin = 0;

  /** Real time, in seconds, of the last frame. */
  let lastFrame = 0;

  function frame(): void {
    if (!running) return;
    // Ask for the next frame first. The loop then survives a callback that
    // throws.
    handle = schedule(frame);

    const nowSec = now() / 1000;
    let frameDt = nowSec - lastFrame;
    // A clock that stands still, that steps back, or that gives NaN must not
    // reach the rest of the frame.
    if (!(frameDt > 0)) frameDt = 0;
    lastFrame = nowSec;

    if (frameDt > 0) {
      const sample = 1 / frameDt;
      stats.fps = stats.fps === 0 ? sample : smooth(stats.fps, sample);
    }

    // The accumulator is the real time that the physics has not simulated yet.
    // The code finds it from the absolute clock instead of a running sum, so
    // rounding error does not build up over a long session.
    let elapsed = nowSec - origin;
    const accumulator = elapsed - steps * PHYSICS_DT;
    if (accumulator > ACCUMULATOR_CAP) {
      const dropped = accumulator - ACCUMULATOR_CAP;
      // Move the origin forward. The dropped time never becomes simulated time.
      origin += dropped;
      elapsed -= dropped;
      stats.droppedTime += dropped;
    }

    // Find the step count for this frame before the first step. A slow
    // fixedUpdate then cannot add more work to the same frame.
    const target = Math.floor(elapsed / PHYSICS_DT);
    const stepsStartMs = now();
    let stepsThisFrame = 0;
    while (steps < target) {
      callbacks.fixedUpdate(PHYSICS_DT, steps * PHYSICS_DT);
      steps += 1;
      stepsThisFrame += 1;
    }
    // A count times the step length holds no drift. A running sum does.
    stats.simTime = steps * PHYSICS_DT;
    stats.physicsStepsLastFrame = stepsThisFrame;
    stats.fixedUpdateMs = smooth(stats.fixedUpdateMs, now() - stepsStartMs);

    let alpha = (elapsed - steps * PHYSICS_DT) / PHYSICS_DT;
    if (alpha < 0) alpha = 0;
    else if (alpha > 1) alpha = 1;

    const renderStartMs = now();
    callbacks.render(alpha, frameDt);
    stats.renderMs = smooth(stats.renderMs, now() - renderStartMs);
  }

  return {
    start(): void {
      if (running) return;
      running = true;
      const nowSec = now() / 1000;
      lastFrame = nowSec;
      // Hold the step count over a stop. The real time of the stop does not
      // become simulated time and does not count as dropped time.
      origin = nowSec - steps * PHYSICS_DT;
      handle = schedule(frame);
    },

    stop(): void {
      if (!running) return;
      running = false;
      cancel(handle);
      handle = 0;
    },

    stats,
  };
}
