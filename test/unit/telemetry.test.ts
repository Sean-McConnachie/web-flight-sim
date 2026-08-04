import { describe, expect, it } from 'vitest';

import {
  attitudeAngles,
  fixedWidth,
  findOverlayField,
  OVERLAY_GROUPS,
  VALUE_WIDTH,
} from '@/ui/debug-overlay';
import type { OverlayField, TelemetrySample } from '@/ui/debug-overlay';
import { createRingBuffer } from '@/ui/telemetry-graph';
import type { Rgb } from '@/render/force-arrows';
import { stallColor } from '@/render/force-arrows';
import { kelvinToCelsius, msToKmh, paToHpa, toDeg, toRad } from '@/math/units';
import { isa } from '@/physics/atmosphere';
import { createState } from '@/physics/rigidbody';

/**
 * Tests for the debug overlay, the telemetry chart and the force arrows.
 *
 * The three modules touch the DOM and the renderer, so these tests only drive
 * the pure parts: the number formatting, the field table with its unit
 * conversions, the ring buffer of the chart, and the stall color ramp. The
 * drawing itself needs an eye, not an assertion.
 */

function makeSample(): TelemetrySample {
  return {
    loop: {
      fps: 0,
      physicsStepsLastFrame: 0,
      droppedTime: 0,
      fixedUpdateMs: 0,
      renderMs: 0,
      simTime: 0,
    },
    state: createState(),
    alpha: 0,
    beta: 0,
    loadFactor: 1,
    trueAirspeed: 0,
    equivalentAirspeed: 0,
    mach: 0,
    dynamicPressure: 0,
    atmosphere: isa(0),
  };
}

/** Look up one printed line and fail the test when the panel lost it. */
function field(group: string, label: string, unit?: string): OverlayField {
  const found = findOverlayField(group, label, unit);
  if (found === undefined) {
    throw new Error(`the overlay has no field "${label}" in the group "${group}"`);
  }
  return found;
}

describe('fixed width number formatting', () => {
  it('prints the same column width for -1234.5, 0 and 0.001', () => {
    const a = fixedWidth(-1234.5, VALUE_WIDTH, 2);
    const b = fixedWidth(0, VALUE_WIDTH, 2);
    const c = fixedWidth(0.001, VALUE_WIDTH, 2);
    expect(a.length).toBe(VALUE_WIDTH);
    expect(b.length).toBe(VALUE_WIDTH);
    expect(c.length).toBe(VALUE_WIDTH);
    expect(a.trim()).toBe('-1234.50');
    expect(b.trim()).toBe('0.00');
    expect(c.trim()).toBe('0.00');
  });

  it('holds the column width over the whole range a flight can reach', () => {
    const values = [
      0, 1, -1, 0.004, -0.004, 9.995, -9.995, 175, -1234.5, 60000, -60000, 101325, -101325,
      1e12, -1e12, Number.NaN, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY,
    ];
    for (const decimals of [0, 1, 2, 3, 4]) {
      for (const value of values) {
        expect(fixedWidth(value, VALUE_WIDTH, decimals).length).toBe(VALUE_WIDTH);
      }
    }
  });

  it('prints a value that rounds to zero without a minus sign', () => {
    // "-0.00" reads as a fault in the sign, and it is not one.
    expect(fixedWidth(-0.0001, VALUE_WIDTH, 2).trim()).toBe('0.00');
  });

  it('keeps the integer part and drops decimals when the column is too small', () => {
    // The digits that matter are the large ones. 1234.5678 in a column of six
    // must print 1234.6 and never 234.57.
    expect(fixedWidth(1234.5678, 6, 4).trim()).toBe('1234.6');
  });
});

describe('overlay display units match src/math/units.ts', () => {
  it('prints altitude as minus the NED z position', () => {
    const s = makeSample();
    s.state.position.set(120, -30, -1500);
    expect(field('state', 'north').read(s)).toBe(120);
    expect(field('state', 'east').read(s)).toBe(-30);
    // CONVENTIONS section 3.2. A positive z is below the ground.
    expect(field('state', 'altitude').read(s)).toBe(1500);
  });

  it('prints the climb rate as minus the NED z velocity', () => {
    const s = makeSample();
    s.state.velocity.set(200, 0, -20);
    expect(field('state', 'climb').read(s)).toBe(20);
    expect(field('state', 'ground', 'km/h').read(s)).toBeCloseTo(msToKmh(200), 9);
  });

  it('prints airspeed in kilometers per hour through msToKmh', () => {
    const s = makeSample();
    // The clean stall speed of the Me-262 is 175 km/h indicated.
    s.trueAirspeed = 200;
    s.equivalentAirspeed = 175;
    expect(field('state', 'tas', 'km/h').read(s)).toBeCloseTo(msToKmh(200), 9);
    expect(field('state', 'tas', 'm/s').read(s)).toBe(200);
    expect(field('state', 'eas', 'km/h').read(s)).toBeCloseTo(msToKmh(175), 9);
  });

  it('prints angles in degrees through toDeg', () => {
    const s = makeSample();
    s.alpha = toRad(12);
    s.beta = toRad(-3);
    s.state.angularVelocity.set(toRad(45), toRad(-10), toRad(6));
    expect(field('flow', 'alpha').read(s)).toBeCloseTo(12, 9);
    expect(field('flow', 'beta').read(s)).toBeCloseTo(-3, 9);
    expect(field('attitude', 'roll rate').read(s)).toBeCloseTo(45, 9);
    expect(field('attitude', 'pitch rate').read(s)).toBeCloseTo(-10, 9);
    expect(field('attitude', 'yaw rate').read(s)).toBeCloseTo(6, 9);
  });

  it('prints the air state in Celsius and hectopascals', () => {
    const s = makeSample();
    s.atmosphere = isa(6000);
    expect(field('air', 'temp').read(s)).toBeCloseTo(kelvinToCelsius(s.atmosphere.temperature), 9);
    expect(field('air', 'pressure').read(s)).toBeCloseTo(paToHpa(s.atmosphere.pressure), 9);
    expect(field('air', 'density').read(s)).toBe(s.atmosphere.density);
    // The standard atmosphere at 6000 m holds about 0.66 kg/m3 and -24 C.
    expect(field('air', 'density').read(s)).toBeCloseTo(0.6601, 3);
    expect(field('air', 'temp').read(s)).toBeCloseTo(-24, 0);
  });

  it('reads every field of every group without a fault', () => {
    const s = makeSample();
    s.state.position.set(10, 20, -3000);
    s.state.velocity.set(180, 4, -6);
    for (const group of OVERLAY_GROUPS) {
      for (const line of group.fields) {
        expect(Number.isFinite(line.read(s))).toBe(true);
      }
    }
  });
});

describe('attitude from the NED quaternion', () => {
  const angles = { roll: 0, pitch: 0, heading: 0 };

  it('reads a level heading of due west as 270 degrees and not as -90', () => {
    const s = makeSample();
    // A yaw of -90 degrees about the down axis points the nose west.
    const half = toRad(-45);
    s.state.orientation.set(0, 0, Math.sin(half), Math.cos(half));
    expect(field('attitude', 'heading').read(s)).toBeCloseTo(270, 6);
  });

  it('reads a nose up pitch as positive', () => {
    // A positive rotation about the body y axis raises the nose, because the
    // world z axis points down. CONVENTIONS section 3.
    const half = toRad(5);
    const q = createState().orientation.set(0, Math.sin(half), 0, Math.cos(half));
    expect(toDeg(attitudeAngles(q, angles).pitch)).toBeCloseTo(10, 6);
    expect(angles.roll).toBeCloseTo(0, 9);
  });

  it('reads a right wing down roll as positive', () => {
    const half = toRad(15);
    const q = createState().orientation.set(Math.sin(half), 0, 0, Math.cos(half));
    expect(toDeg(attitudeAngles(q, angles).roll)).toBeCloseTo(30, 6);
  });
});

describe('telemetry chart ring buffer', () => {
  it('keeps the last N samples in order after it wraps', () => {
    const buffer = createRingBuffer(4);
    for (let i = 1; i <= 7; i++) buffer.push(i);
    expect(buffer.length).toBe(4);
    expect(buffer.at(0)).toBe(4);
    expect(buffer.at(1)).toBe(5);
    expect(buffer.at(2)).toBe(6);
    expect(buffer.at(3)).toBe(7);
  });

  it('reports the samples in order before it is full', () => {
    const buffer = createRingBuffer(4);
    buffer.push(11);
    buffer.push(12);
    expect(buffer.length).toBe(2);
    expect(buffer.at(0)).toBe(11);
    expect(buffer.at(1)).toBe(12);
    expect(Number.isNaN(buffer.at(2))).toBe(true);
  });

  it('allocates nothing after construction', () => {
    const buffer = createRingBuffer(8);
    const store = buffer.values;
    const keys = Object.keys(buffer).sort().join(',');
    for (let i = 0; i < 10000; i++) buffer.push(Math.sin(i));
    // The store is the same object and the same size, so no push grew it and no
    // push replaced it. The object gained no field either.
    expect(buffer.values).toBe(store);
    expect(buffer.values.length).toBe(8);
    expect(buffer.capacity).toBe(8);
    expect(buffer.length).toBe(8);
    expect(Object.keys(buffer).sort().join(',')).toBe(keys);
    expect(buffer.at(7)).toBeCloseTo(Math.sin(9999), 12);
  });

  it('clears back to an empty buffer', () => {
    const buffer = createRingBuffer(3);
    buffer.push(1);
    buffer.push(2);
    buffer.clear();
    expect(buffer.length).toBe(0);
    buffer.push(9);
    expect(buffer.at(0)).toBe(9);
  });

  it('refuses a capacity that is not a positive whole number', () => {
    expect(() => createRingBuffer(0)).toThrow();
    expect(() => createRingBuffer(-1)).toThrow();
    expect(() => createRingBuffer(2.5)).toThrow();
  });
});

describe('angle of attack color scale', () => {
  // A stall angle near the value of a NACA four digit section at this Reynolds
  // number. The scale reads the ratio, so the exact angle does not matter.
  const STALL = toRad(15);

  function sample(ratio: number): Rgb {
    const out: Rgb = { r: 0, g: 0, b: 0 };
    return stallColor(ratio * STALL, STALL, out);
  }

  /** A red hue. Red leads, and both other parts stay far below it. */
  function isRed(c: Rgb): boolean {
    return c.r > 0.5 && c.g < 0.3 * c.r && c.b < 0.3 * c.r;
  }

  it('runs cool far below the stall angle', () => {
    const c = sample(0);
    expect(c.b).toBeGreaterThan(c.r);
    expect(c.b).toBeGreaterThan(0.9);
    expect(isRed(c)).toBe(false);
  });

  it('runs yellow in the warning band below the stall angle', () => {
    const c = sample(0.8);
    expect(c.r).toBeGreaterThan(0.9);
    expect(c.g).toBeGreaterThan(0.7);
    expect(c.b).toBeLessThan(0.2);
    expect(isRed(c)).toBe(false);
  });

  it('crosses to red at the stall angle and not before it', () => {
    expect(isRed(sample(0.9))).toBe(false);
    expect(isRed(sample(0.95))).toBe(false);
    expect(isRed(sample(1))).toBe(true);
    // Past the stall the red goes dark, and it stays red.
    expect(isRed(sample(1.2))).toBe(true);
    expect(isRed(sample(3))).toBe(true);
    expect(sample(1.2).r).toBeLessThan(sample(1).r);
  });

  it('changes continuously with the angle of attack', () => {
    const previous: Rgb = { r: 0, g: 0, b: 0 };
    const current: Rgb = { r: 0, g: 0, b: 0 };
    const steps = 2000;
    let worst = 0;
    stallColor(0, STALL, previous);
    for (let i = 1; i <= steps; i++) {
      const ratio = (i / steps) * 2;
      stallColor(ratio * STALL, STALL, current);
      worst = Math.max(
        worst,
        Math.abs(current.r - previous.r),
        Math.abs(current.g - previous.g),
        Math.abs(current.b - previous.b),
      );
      previous.r = current.r;
      previous.g = current.g;
      previous.b = current.b;
    }
    // A jump at a stop would show here as a step of the size of that jump.
    expect(worst).toBeLessThan(0.01);
  });

  it('reads the size of the angle, so a negative stall colors the same', () => {
    const positive = sample(1.1);
    const negative: Rgb = { r: 0, g: 0, b: 0 };
    stallColor(-1.1 * STALL, STALL, negative);
    expect(negative.r).toBeCloseTo(positive.r, 12);
    expect(negative.g).toBeCloseTo(positive.g, 12);
    expect(negative.b).toBeCloseTo(positive.b, 12);
  });

  it('gives a finite color when the stall angle is missing', () => {
    const out: Rgb = { r: 0, g: 0, b: 0 };
    stallColor(0.2, 0, out);
    expect(Number.isFinite(out.r)).toBe(true);
    expect(Number.isFinite(out.g)).toBe(true);
    expect(Number.isFinite(out.b)).toBe(true);
  });
});
