import { describe, expect, it } from 'vitest';
import {
  DEFAULT_TF,
  MIN_LAG_SPEED,
  STALL_NACA_0009,
  STALL_NACA_0011,
  createStallState,
  fitStallParams,
  kirchhoffLift,
  separationCenterOfPressure,
  steadySeparation,
  updateSeparation,
} from '@/physics/aero/stall';
import type { StallParams } from '@/physics/aero/stall';
import { NACA_0009, NACA_0011 } from '@/physics/aero/airfoil';
import type { Airfoil } from '@/physics/aero/airfoil';
import { DEG, toDeg } from '@/math/units';

const sections: ReadonlyArray<{ name: string; params: StallParams; airfoil: Airfoil; clMax: number }> =
  [
    { name: 'NACA 0009', params: STALL_NACA_0009, airfoil: NACA_0009, clMax: 1.25 },
    { name: 'NACA 0011', params: STALL_NACA_0011, airfoil: NACA_0011, clMax: 1.35 },
  ];

describe('the steady separation point', () => {
  for (const s of sections) {
    it(`${s.name} is fully attached well below the break angle`, () => {
      expect(steadySeparation(0, s.params)).toBeGreaterThan(0.99);
      expect(steadySeparation(3 * DEG, s.params)).toBeGreaterThan(0.99);
      expect(steadySeparation(0, s.params)).toBeLessThanOrEqual(1);
    });

    it(`${s.name} reaches its floor of 0.04 well above the break angle`, () => {
      expect(steadySeparation(60 * DEG, s.params)).toBeCloseTo(0.04, 3);
      expect(steadySeparation(90 * DEG, s.params)).toBeCloseTo(0.04, 4);
    });

    it(`${s.name} falls monotonically between the two limits`, () => {
      let previous = steadySeparation(0, s.params);
      for (let deg = 0.05; deg <= 60; deg += 0.05) {
        const f = steadySeparation(deg * DEG, s.params);
        expect(f).toBeLessThanOrEqual(previous + 1e-12);
        previous = f;
      }
      // The fall is real, not flat.
      expect(steadySeparation(20 * DEG, s.params)).toBeLessThan(0.4);
    });

    it(`${s.name} is even in the angle of attack`, () => {
      for (const deg of [1, 7, 13, 20, 45]) {
        expect(steadySeparation(-deg * DEG, s.params)).toBeCloseTo(
          steadySeparation(deg * DEG, s.params),
          14,
        );
      }
    });

    it(`${s.name} joins its two branches at 0.7 with the same slope`, () => {
      expect(steadySeparation(s.params.a1, s.params)).toBeCloseTo(0.7, 12);
      const h = 1e-6;
      const atBreak = steadySeparation(s.params.a1, s.params);
      const slopeBelow = (atBreak - steadySeparation(s.params.a1 - h, s.params)) / h;
      const slopeAbove = (steadySeparation(s.params.a1 + h, s.params) - atBreak) / h;
      expect(slopeAbove).toBeCloseTo(slopeBelow, 4);
      expect(slopeBelow).toBeLessThan(0);
    });
  }
});

describe('the Kirchhoff lift law', () => {
  it('gives the attached lift when the flow is fully attached', () => {
    const clAlpha = NACA_0009.clAlpha;
    for (const deg of [-8, -2, 0, 3, 9]) {
      const alpha = deg * DEG;
      expect(kirchhoffLift(clAlpha, alpha, 0, 1)).toBeCloseTo(clAlpha * alpha, 12);
    }
  });

  it('keeps about 36 percent of the slope when the flow is fully separated', () => {
    const clAlpha = NACA_0009.clAlpha;
    const alpha = 10 * DEG;
    const separated = kirchhoffLift(clAlpha, alpha, 0, 0.04);
    // ((1 + sqrt(0.04)) / 2)^2 = 0.36
    expect(separated / (clAlpha * alpha)).toBeCloseTo(0.36, 6);
  });

  it('shifts with the zero lift angle', () => {
    const clAlpha = 6.28;
    expect(kirchhoffLift(clAlpha, 2 * DEG, 2 * DEG, 1)).toBeCloseTo(0, 12);
  });

  for (const s of sections) {
    it(`${s.name} reproduces the peak lift and the stall angle of the static table`, () => {
      // The Kirchhoff law holds near the stall only. The search stops 3 degrees
      // past the peak, where the static table hands over to the flat plate law.
      const lastDeg = toDeg(s.airfoil.alphaStall) + 3;
      let clMax = -Infinity;
      let alphaDeg = 0;
      for (let deg = 0; deg <= lastDeg; deg += 0.02) {
        const alpha = deg * DEG;
        const cl = kirchhoffLift(
          s.airfoil.clAlpha,
          alpha,
          s.airfoil.alphaZeroLift,
          steadySeparation(alpha, s.params),
        );
        if (cl > clMax) {
          clMax = cl;
          alphaDeg = deg;
        }
      }
      expect(clMax).toBeGreaterThan(s.clMax * 0.95);
      expect(clMax).toBeLessThan(s.clMax * 1.05);
      expect(Math.abs(alphaDeg - toDeg(s.airfoil.alphaStall))).toBeLessThan(1);
    });
  }
});

describe('the fit of the separation constants', () => {
  it('places the break angle below the stall angle for both sections', () => {
    expect(toDeg(STALL_NACA_0009.a1)).toBeLessThan(13);
    expect(toDeg(STALL_NACA_0009.a1)).toBeGreaterThan(10);
    expect(toDeg(STALL_NACA_0011.a1)).toBeLessThan(15);
    expect(toDeg(STALL_NACA_0011.a1)).toBeGreaterThan(11);
  });

  it('keeps the slope of the separation point continuous at the break angle', () => {
    // The two branches have slopes -0.3/s1 and -0.66/s2, so s1 = s2 / 2.2.
    for (const s of sections) {
      expect(0.3 / s.params.s1).toBeCloseTo(0.66 / s.params.s2, 9);
    }
  });

  it('gives the root section a later break than the tip section', () => {
    expect(STALL_NACA_0011.a1).toBeGreaterThan(STALL_NACA_0009.a1);
  });

  it('refuses a peak the law cannot reach', () => {
    expect(() => fitStallParams(6.28, 13 * DEG, 3.0)).toThrow();
    expect(() => fitStallParams(6.28, 13 * DEG, 0.4)).toThrow();
  });

  it('uses the published lag constant by default', () => {
    expect(DEFAULT_TF).toBe(3);
    expect(STALL_NACA_0009.tf).toBe(3);
  });
});

describe('dynamic stall', () => {
  const CHORD = 2.0; // m
  const SPEED = 60; // m/s
  const DT = 1 / 240; // s
  const params = STALL_NACA_0009;
  const clAlpha = NACA_0009.clAlpha;

  /** Settles the state at one angle, then ramps to another over rampTime. */
  function rampTo(startDeg: number, endDeg: number, rampTime: number) {
    const state = createStallState();
    for (let i = 0; i < 1000; i++) {
      updateSeparation(state, startDeg * DEG, CHORD, SPEED, DT, params);
    }
    for (let t = 0; t < rampTime; t += DT) {
      const deg = startDeg + ((endDeg - startDeg) * t) / rampTime;
      updateSeparation(state, deg * DEG, CHORD, SPEED, DT, params);
    }
    return state;
  }

  it('settles on the steady value when the angle of attack holds', () => {
    const state = createStallState();
    for (let i = 0; i < 1000; i++) {
      updateSeparation(state, 10 * DEG, CHORD, SPEED, DT, params);
    }
    expect(state.f).toBeCloseTo(steadySeparation(10 * DEG, params), 9);
  });

  it('keeps the separation point above its steady value during a pull', () => {
    const state = rampTo(10, 20, 0.2);
    const steady = steadySeparation(20 * DEG, params);
    expect(state.f).toBeGreaterThan(steady);
  });

  it('makes the lift overshoot the steady lift during a pull', () => {
    const state = rampTo(10, 20, 0.2);
    const alpha = 20 * DEG;
    const laggedLift = kirchhoffLift(clAlpha, alpha, 0, state.f);
    const steadyLift = kirchhoffLift(clAlpha, alpha, 0, steadySeparation(alpha, params));
    expect(laggedLift).toBeGreaterThan(steadyLift);
    // The overshoot is worth about 15 percent at this pull rate.
    expect(laggedLift / steadyLift).toBeGreaterThan(1.05);
  });

  it('lets the overshoot decay when the angle of attack holds', () => {
    const state = rampTo(10, 20, 0.2);
    const alpha = 20 * DEG;
    const steady = steadySeparation(alpha, params);
    const overshootAtBreak = state.f - steady;
    expect(overshootAtBreak).toBeGreaterThan(0.05);
    for (let i = 0; i < 12; i++) {
      updateSeparation(state, alpha, CHORD, SPEED, DT, params);
    }
    const overshootLater = state.f - steady;
    expect(overshootLater).toBeGreaterThan(0);
    expect(overshootLater).toBeLessThan(overshootAtBreak * 0.5);
    for (let i = 0; i < 240; i++) {
      updateSeparation(state, alpha, CHORD, SPEED, DT, params);
    }
    expect(state.f).toBeCloseTo(steady, 6);
  });

  it('gives a lift ratio of one in steady flow at any angle', () => {
    // This is how an element must use the module. The ratio leaves 1 only while
    // the angle of attack moves, so the static table stays in charge.
    const state = createStallState();
    for (const deg of [2, 10, 16, 30, 60]) {
      const alpha = deg * DEG;
      for (let i = 0; i < 2000; i++) {
        updateSeparation(state, alpha, CHORD, SPEED, DT, params);
      }
      const steady = steadySeparation(alpha, params);
      const ratio =
        kirchhoffLift(clAlpha, alpha, 0, state.f) / kirchhoffLift(clAlpha, alpha, 0, steady);
      expect(ratio).toBeCloseTo(1, 8);
    }
  });

  it('holds the center of pressure forward during the overshoot', () => {
    const state = rampTo(10, 20, 0.2);
    const steady = steadySeparation(20 * DEG, params);
    expect(separationCenterOfPressure(state.f)).toBeLessThan(
      separationCenterOfPressure(steady),
    );
  });

  it('settles faster at a higher speed', () => {
    const slow = rampTo(10, 20, 0.2);
    const state = createStallState();
    const fastSpeed = 240; // m/s
    for (let i = 0; i < 1000; i++) {
      updateSeparation(state, 10 * DEG, CHORD, fastSpeed, DT, params);
    }
    for (let t = 0; t < 0.2; t += DT) {
      const deg = 10 + (10 * t) / 0.2;
      updateSeparation(state, deg * DEG, CHORD, fastSpeed, DT, params);
    }
    const steady = steadySeparation(20 * DEG, params);
    expect(state.f - steady).toBeLessThan(slow.f - steady);
  });
});

describe('the low speed guard', () => {
  const params = STALL_NACA_0009;

  it('keeps the separation point finite at zero speed', () => {
    const state = createStallState();
    for (let i = 0; i < 100; i++) {
      updateSeparation(state, 20 * DEG, 2, 0, 1 / 240, params);
    }
    expect(Number.isFinite(state.f)).toBe(true);
    expect(state.f).toBeLessThan(1);
    expect(state.f).toBeGreaterThan(0);
  });

  it('relaxes to the steady value on the ground given enough time', () => {
    const state = createStallState();
    const target = steadySeparation(20 * DEG, params);
    for (let i = 0; i < 240 * 60; i++) {
      updateSeparation(state, 20 * DEG, 2, 0, 1 / 240, params);
    }
    expect(state.f).toBeCloseTo(target, 6);
  });

  it('treats any speed below the floor as the floor', () => {
    const slow = createStallState();
    const stopped = createStallState();
    updateSeparation(slow, 20 * DEG, 2, MIN_LAG_SPEED * 0.5, 1 / 240, params);
    updateSeparation(stopped, 20 * DEG, 2, 0, 1 / 240, params);
    expect(slow.f).toBeCloseTo(stopped.f, 14);
  });

  it('leaves the state alone on a zero time step', () => {
    const state = createStallState();
    const before = state.f;
    updateSeparation(state, 20 * DEG, 2, 100, 0, params);
    expect(state.f).toBe(before);
  });
});

describe('the center of pressure', () => {
  it('sits at the quarter chord with attached flow', () => {
    expect(separationCenterOfPressure(1)).toBeCloseTo(0.25, 12);
  });

  it('reaches the middle of the chord with fully separated flow', () => {
    expect(separationCenterOfPressure(0)).toBeCloseTo(0.5, 12);
    expect(separationCenterOfPressure(0.04)).toBeCloseTo(0.49, 12);
  });

  it('moves back monotonically as the flow separates', () => {
    let previous = separationCenterOfPressure(1);
    for (let f = 1; f >= 0; f -= 0.01) {
      const x = separationCenterOfPressure(f);
      expect(x).toBeGreaterThanOrEqual(previous - 1e-12);
      previous = x;
    }
  });
});
