import { describe, expect, it } from 'vitest';
import {
  clamp,
  inverseLerp,
  lerp,
  lookup1d,
  lookup2d,
  lookupCyclic,
  smoothstep,
  table1d,
  table2d,
  wrapPi,
} from '@/math/tables';

describe('table1d and lookup1d', () => {
  // A lift curve shape. The slope is 2 per unit between -1 and 1.
  const t = table1d([-2, -1, 0, 1, 3], [-1.5, -2, 0, 2, 1]);

  it('the lookup returns the knot value at every knot', () => {
    expect(lookup1d(t, -2)).toBe(-1.5);
    expect(lookup1d(t, -1)).toBe(-2);
    expect(lookup1d(t, 0)).toBe(0);
    expect(lookup1d(t, 1)).toBe(2);
    expect(lookup1d(t, 3)).toBe(1);
  });

  it('the lookup interpolates linearly between two knots', () => {
    expect(lookup1d(t, 0.5)).toBeCloseTo(1, 12);
    expect(lookup1d(t, -0.25)).toBeCloseTo(-0.5, 12);
    // Between 1 and 3 the value falls from 2 to 1.
    expect(lookup1d(t, 2)).toBeCloseTo(1.5, 12);
    expect(lookup1d(t, 2.5)).toBeCloseTo(1.25, 12);
  });

  it('the lookup clamps below the first knot and above the last knot', () => {
    expect(lookup1d(t, -50)).toBe(-1.5);
    expect(lookup1d(t, -2.0001)).toBe(-1.5);
    expect(lookup1d(t, 3.0001)).toBe(1);
    expect(lookup1d(t, 1e6)).toBe(1);
  });

  it('the binary search finds the right interval in a long table', () => {
    const x: number[] = [];
    const y: number[] = [];
    for (let i = 0; i < 501; i++) {
      x.push(i * 0.5);
      y.push(i * 0.5 * 3 + 7);
    }
    const long = table1d(x, y);
    // The table holds a straight line, so every sample must sit on that line.
    for (const sample of [0, 0.25, 17.3, 118.75, 249.9, 250]) {
      expect(lookup1d(long, sample)).toBeCloseTo(sample * 3 + 7, 9);
    }
  });

  it('a table with one knot returns that knot everywhere', () => {
    const single = table1d([4], [9]);
    expect(lookup1d(single, -1)).toBe(9);
    expect(lookup1d(single, 4)).toBe(9);
    expect(lookup1d(single, 100)).toBe(9);
  });
});

describe('lookupCyclic', () => {
  // A table over the full circle in degrees. The end knot repeats the first one.
  const full = table1d([-180, -90, 0, 90, 180], [0, 1, 0, -1, 0]);

  it('a sample at 190 degrees wraps to -170 degrees', () => {
    // -170 sits one ninth of the way from -180 to -90, so the value is 1 / 9.
    expect(lookupCyclic(full, 190, 360)).toBeCloseTo(1 / 9, 12);
    expect(lookupCyclic(full, -170, 360)).toBeCloseTo(1 / 9, 12);
  });

  it('the wrap holds for a sample many periods away', () => {
    expect(lookupCyclic(full, 45, 360)).toBeCloseTo(-0.5, 12);
    expect(lookupCyclic(full, 45 + 360 * 3, 360)).toBeCloseTo(-0.5, 12);
    expect(lookupCyclic(full, 45 - 360 * 5, 360)).toBeCloseTo(-0.5, 12);
    expect(lookupCyclic(full, -190, 360)).toBeCloseTo(-1 / 9, 12);
  });

  it('the value at the seam matches the value at both ends of the table', () => {
    expect(lookupCyclic(full, 180, 360)).toBeCloseTo(0, 12);
    expect(lookupCyclic(full, -180, 360)).toBeCloseTo(0, 12);
    expect(lookupCyclic(full, 360, 360)).toBeCloseTo(0, 12);
  });

  it('a table without the repeated end knot interpolates across the seam', () => {
    // The last knot at 270 joins the first knot at 360, which is 0 again.
    const open = table1d([0, 90, 180, 270], [0, 1, 2, 3]);
    expect(lookupCyclic(open, 315, 360)).toBeCloseTo(1.5, 12);
    expect(lookupCyclic(open, -45, 360)).toBeCloseTo(1.5, 12);
    expect(lookupCyclic(open, 350, 360)).toBeCloseTo(3 - (3 * 80) / 90, 12);
    expect(lookupCyclic(open, 360, 360)).toBeCloseTo(0, 12);
    expect(lookupCyclic(open, 720, 360)).toBeCloseTo(0, 12);
  });

  it('a cyclic table in radians wraps at two pi', () => {
    const rad = table1d([-Math.PI, 0, Math.PI], [0, 4, 0]);
    expect(lookupCyclic(rad, Math.PI / 2, 2 * Math.PI)).toBeCloseTo(2, 12);
    expect(lookupCyclic(rad, (5 * Math.PI) / 2, 2 * Math.PI)).toBeCloseTo(2, 12);
  });

  it('a cyclic table with one knot returns that knot', () => {
    const single = table1d([0], [7]);
    expect(lookupCyclic(single, 123, 360)).toBe(7);
  });
});

describe('table2d and lookup2d', () => {
  // z[iy][ix]. Row 0 is y = 0. Row 1 is y = 20.
  const t = table2d([0, 10], [0, 20], [
    [0, 10],
    [20, 40],
  ]);

  it('the lookup returns the corner value at every corner', () => {
    expect(lookup2d(t, 0, 0)).toBe(0);
    expect(lookup2d(t, 10, 0)).toBe(10);
    expect(lookup2d(t, 0, 20)).toBe(20);
    expect(lookup2d(t, 10, 20)).toBe(40);
  });

  it('the lookup interpolates linearly along each edge of a cell', () => {
    expect(lookup2d(t, 5, 0)).toBeCloseTo(5, 12);
    expect(lookup2d(t, 5, 20)).toBeCloseTo(30, 12);
    expect(lookup2d(t, 0, 10)).toBeCloseTo(10, 12);
    expect(lookup2d(t, 10, 10)).toBeCloseTo(25, 12);
  });

  it('the value at the center of a cell matches the hand computed bilinear value', () => {
    // fx = 0.5 and fy = 0.5. Low row gives 5. High row gives 30. Blend gives 17.5.
    expect(lookup2d(t, 5, 10)).toBeCloseTo(17.5, 12);
    // fx = 0.25 and fy = 0.75. Low row gives 2.5. High row gives 25. Blend gives 19.375.
    expect(lookup2d(t, 2.5, 15)).toBeCloseTo(19.375, 12);
  });

  it('the lookup clamps outside the grid on both axes', () => {
    expect(lookup2d(t, -100, -100)).toBe(0);
    expect(lookup2d(t, 100, 100)).toBe(40);
    expect(lookup2d(t, -100, 20)).toBe(20);
    expect(lookup2d(t, 5, -100)).toBeCloseTo(5, 12);
    expect(lookup2d(t, 100, 10)).toBeCloseTo(25, 12);
  });

  it('the lookup picks the right cell in a grid with uneven knots', () => {
    const uneven = table2d([0, 1, 3], [0, 2], [
      [0, 2, 6],
      [10, 12, 16],
    ]);
    // At x = 2 the fraction is 0.5 in the second cell. Low row gives 4. High row
    // gives 14. At y = 1 the fraction is 0.5, so the answer is 9.
    expect(lookup2d(uneven, 2, 1)).toBeCloseTo(9, 12);
    expect(lookup2d(uneven, 1, 0)).toBe(2);
    expect(lookup2d(uneven, 3, 2)).toBe(16);
  });

  it('the builder stores z in row major order', () => {
    expect(Array.from(t.z)).toEqual([0, 10, 20, 40]);
  });
});

describe('scalar helpers', () => {
  it('lerp and inverseLerp are inverse operations', () => {
    expect(lerp(2, 6, 0)).toBe(2);
    expect(lerp(2, 6, 1)).toBe(6);
    expect(lerp(2, 6, 0.25)).toBeCloseTo(3, 12);
    expect(inverseLerp(2, 6, 3)).toBeCloseTo(0.25, 12);
    expect(inverseLerp(2, 6, lerp(2, 6, 0.7))).toBeCloseTo(0.7, 12);
  });

  it('clamp limits a value to the given range', () => {
    expect(clamp(5, 0, 1)).toBe(1);
    expect(clamp(-5, 0, 1)).toBe(0);
    expect(clamp(0.5, 0, 1)).toBe(0.5);
    expect(clamp(-3, -2, 2)).toBe(-2);
  });

  it('smoothstep runs from 0 to 1 and has a flat slope at both edges', () => {
    expect(smoothstep(0, 1, -1)).toBe(0);
    expect(smoothstep(0, 1, 0)).toBe(0);
    expect(smoothstep(0, 1, 0.5)).toBeCloseTo(0.5, 12);
    expect(smoothstep(0, 1, 1)).toBe(1);
    expect(smoothstep(0, 1, 2)).toBe(1);
    // The slope is flat at the edges, so the step near an edge stays small.
    expect(smoothstep(0, 1, 0.01)).toBeLessThan(0.001);
    expect(smoothstep(10, 20, 15)).toBeCloseTo(0.5, 12);
  });

  it('wrapPi puts an angle into the range -PI to PI', () => {
    expect(wrapPi(0)).toBe(0);
    expect(wrapPi(1)).toBeCloseTo(1, 12);
    expect(wrapPi(-1)).toBeCloseTo(-1, 12);
    // The upper limit is not part of the range, so PI gives -PI.
    expect(wrapPi(Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(wrapPi(-Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(wrapPi(3 * Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(wrapPi(1.5 * Math.PI)).toBeCloseTo(-0.5 * Math.PI, 12);
    expect(wrapPi(-1.5 * Math.PI)).toBeCloseTo(0.5 * Math.PI, 12);
    expect(wrapPi(100 * Math.PI + 0.25)).toBeCloseTo(0.25, 9);
  });
});

describe('table builders reject bad input', () => {
  it('table1d throws when x does not increase strictly', () => {
    expect(() => table1d([0, 1, 1, 2], [0, 1, 2, 3])).toThrow(/increase strictly/);
    expect(() => table1d([0, 5, 2], [0, 1, 2])).toThrow(/increase strictly/);
    expect(() => table1d([0, Number.NaN], [0, 1])).toThrow(/increase strictly/);
  });

  it('table1d throws when x and y have different lengths', () => {
    expect(() => table1d([0, 1, 2], [0, 1])).toThrow(/same length/);
    expect(() => table1d([], [])).toThrow(/one knot at least/);
  });

  it('table2d throws when an axis does not increase strictly', () => {
    expect(() =>
      table2d([0, 0], [0, 1], [
        [1, 2],
        [3, 4],
      ]),
    ).toThrow(/increase strictly/);
    expect(() =>
      table2d([0, 1], [1, 0], [
        [1, 2],
        [3, 4],
      ]),
    ).toThrow(/increase strictly/);
  });

  it('table2d throws when the shape of z does not match the axes', () => {
    expect(() => table2d([0, 1], [0, 1], [[1, 2]])).toThrow(/one row of z for each knot in y/);
    expect(() =>
      table2d([0, 1], [0, 1], [
        [1, 2],
        [3, 4, 5],
      ]),
    ).toThrow(/one z value for each knot in x/);
    expect(() => table2d([], [0], [[]])).toThrow(/one knot at least/);
  });
});
