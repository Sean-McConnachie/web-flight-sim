import { describe, expect, it } from 'vitest';
import type { Loop } from '@/core/loop';
import { createLoop, PHYSICS_DT, PHYSICS_HZ } from '@/core/loop';
import { config } from '@/core/config';

/**
 * A fake clock and a fake frame scheduler. The test moves the clock by hand, so
 * the loop runs in Node with no browser and with no real time.
 */
interface FakeHost {
  now(): number;
  schedule(frame: () => void): number;
  cancel(handle: number): void;
  /** Moves the clock to `atMs` and runs the frame that waits. */
  frameAt(atMs: number): void;
}

function createFakeHost(): FakeHost {
  let timeMs = 0;
  let pending: (() => void) | null = null;
  let nextHandle = 0;

  return {
    now: () => timeMs,
    schedule(frame: () => void): number {
      pending = frame;
      nextHandle += 1;
      return nextHandle;
    },
    cancel(): void {
      pending = null;
    },
    frameAt(atMs: number): void {
      timeMs = atMs;
      const frame = pending;
      pending = null;
      if (frame !== null) frame();
    },
  };
}

interface Probe {
  host: FakeHost;
  loop: Loop;
  /** Simulated time that every fixedUpdate call received. */
  stepTimes: number[];
  /** Interpolation factor that every render call received. */
  alphas: number[];
}

function createProbe(): Probe {
  const host = createFakeHost();
  const stepTimes: number[] = [];
  const alphas: number[] = [];
  const loop = createLoop(
    {
      fixedUpdate(dt: number, time: number): void {
        if (dt !== PHYSICS_DT) throw new Error('fixedUpdate must always get the fixed step length.');
        stepTimes.push(time);
      },
      render(alpha: number): void {
        alphas.push(alpha);
      },
    },
    { now: host.now, schedule: host.schedule, cancel: host.cancel },
  );
  return { host, loop, stepTimes, alphas };
}

describe('fixed step loop', () => {
  it('the physics rate is 240 Hz and the step length is its inverse', () => {
    expect(PHYSICS_HZ).toBe(240);
    expect(PHYSICS_DT).toBe(1 / 240);
  });

  it('one second of wall time runs exactly 240 physics steps at 60 frames per second', () => {
    const probe = createProbe();
    probe.loop.start();
    for (let i = 1; i <= 60; i += 1) probe.host.frameAt((i * 1000) / 60);
    probe.loop.stop();

    expect(probe.stepTimes.length).toBe(PHYSICS_HZ);
    expect(probe.loop.stats.simTime).toBeCloseTo(1, 9);
    expect(probe.loop.stats.droppedTime).toBe(0);
  });

  it('one second of wall time runs exactly 240 physics steps at any frame rate', () => {
    for (const frameRate of [24, 30, 72, 100, 120, 144, 240, 360]) {
      const probe = createProbe();
      probe.loop.start();
      for (let i = 1; i <= frameRate; i += 1) probe.host.frameAt((i * 1000) / frameRate);
      probe.loop.stop();

      expect(probe.stepTimes.length).toBe(PHYSICS_HZ);
    }
  });

  it('a single three second frame simulates only the 0.25 s cap and drops the rest', () => {
    const probe = createProbe();
    probe.loop.start();
    probe.host.frameAt(3000);
    probe.loop.stop();

    const cappedSteps = config.physics.accumulatorCap * PHYSICS_HZ;
    expect(cappedSteps).toBe(60);
    expect(probe.stepTimes.length).toBe(cappedSteps);
    expect(probe.loop.stats.physicsStepsLastFrame).toBe(cappedSteps);
    expect(probe.loop.stats.droppedTime).toBeCloseTo(3 - config.physics.accumulatorCap, 9);
  });

  it('the loop catches up after the cap fires and drops no more time', () => {
    const probe = createProbe();
    probe.loop.start();
    probe.host.frameAt(3000);
    const droppedAfterStall = probe.loop.stats.droppedTime;
    for (let i = 1; i <= 60; i += 1) probe.host.frameAt(3000 + (i * 1000) / 60);
    probe.loop.stop();

    // 60 steps for the capped frame, then 240 steps for the second that follows.
    expect(probe.stepTimes.length).toBe(60 + PHYSICS_HZ);
    expect(probe.loop.stats.droppedTime).toBe(droppedAfterStall);
  });

  it('alpha stays inside 0 to 1 over irregular frames', () => {
    const probe = createProbe();
    probe.loop.start();

    // Gaps that do not divide the step length, plus one long stall.
    const gaps = [7.3, 16.7, 4.1, 33.4, 3000, 1.2, 21.9, 8.8, 250, 12.5];
    let timeMs = 0;
    for (let i = 0; i < 200; i += 1) {
      timeMs += gaps[i % gaps.length];
      probe.host.frameAt(timeMs);
    }
    probe.loop.stop();

    expect(probe.alphas.length).toBe(200);
    for (const alpha of probe.alphas) {
      expect(alpha).toBeGreaterThanOrEqual(0);
      expect(alpha).toBeLessThanOrEqual(1);
    }
  });

  it('simulated time advances by one step length per step with no drift over 10000 steps', () => {
    const target = 10000;
    const probe = createProbe();
    probe.loop.start();
    // 100 ms per frame stays under the accumulator cap, so no frame drops time.
    for (let i = 1; probe.stepTimes.length < target; i += 1) probe.host.frameAt(i * 100);
    probe.loop.stop();

    expect(probe.loop.stats.droppedTime).toBe(0);
    expect(probe.stepTimes.length).toBeGreaterThanOrEqual(target);

    let worstTime = 0;
    let worstGap = 0;
    for (let i = 0; i < target; i += 1) {
      worstTime = Math.max(worstTime, Math.abs(probe.stepTimes[i] - i * PHYSICS_DT));
      if (i > 0) {
        worstGap = Math.max(worstGap, Math.abs(probe.stepTimes[i] - probe.stepTimes[i - 1] - PHYSICS_DT));
      }
    }
    expect(worstTime).toBeLessThan(1e-9);
    expect(worstGap).toBeLessThan(1e-9);

    const steps = probe.stepTimes.length;
    expect(Math.abs(probe.loop.stats.simTime - steps * PHYSICS_DT)).toBeLessThan(1e-9);
  });

  it('stop ends the loop and start carries the simulated time forward', () => {
    const probe = createProbe();
    probe.loop.start();
    for (let i = 1; i <= 60; i += 1) probe.host.frameAt((i * 1000) / 60);
    probe.loop.stop();

    const stepsBefore = probe.stepTimes.length;
    probe.host.frameAt(5000);
    expect(probe.stepTimes.length).toBe(stepsBefore);

    // A pause of four seconds is not simulated time and is not dropped time.
    probe.loop.start();
    for (let i = 1; i <= 60; i += 1) probe.host.frameAt(5000 + (i * 1000) / 60);
    probe.loop.stop();

    expect(probe.loop.stats.droppedTime).toBe(0);
    expect(probe.stepTimes.length).toBe(stepsBefore + PHYSICS_HZ);
    expect(probe.stepTimes[stepsBefore]).toBeCloseTo(stepsBefore * PHYSICS_DT, 9);
    expect(probe.loop.stats.simTime).toBeCloseTo(2, 9);
  });

  it('the statistics report the frame rate and the phase times', () => {
    const probe = createProbe();
    probe.loop.start();
    for (let i = 1; i <= 120; i += 1) probe.host.frameAt((i * 1000) / 60);
    probe.loop.stop();

    expect(probe.loop.stats.fps).toBeCloseTo(60, 6);
    expect(probe.loop.stats.physicsStepsLastFrame).toBeGreaterThan(0);
    expect(probe.stepTimes.length).toBe(2 * PHYSICS_HZ);
    // The fake clock does not move inside a frame, so both phases read zero.
    expect(probe.loop.stats.fixedUpdateMs).toBe(0);
    expect(probe.loop.stats.renderMs).toBe(0);
  });
});
