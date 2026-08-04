import { describe, expect, it } from 'vitest';
import { createRng, mulberry32 } from '@/core/prng';

function draw(next: () => number, count: number): number[] {
  const values: number[] = [];
  for (let i = 0; i < count; i += 1) values.push(next());
  return values;
}

describe('deterministic random numbers', () => {
  it('mulberry32 returns values inside 0 to 1', () => {
    const next = mulberry32(20260804);
    for (let i = 0; i < 10000; i += 1) {
      const value = next();
      expect(value).toBeGreaterThanOrEqual(0);
      expect(value).toBeLessThan(1);
    }
  });

  it('the same seed gives the same first 100 values', () => {
    const first = draw(mulberry32(262), 100);
    const second = draw(mulberry32(262), 100);
    expect(second).toEqual(first);

    const rngA = createRng(262);
    const rngB = createRng(262);
    expect(draw(() => rngB.next(), 100)).toEqual(draw(() => rngA.next(), 100));
  });

  it('the same seed gives the same sequence from every method', () => {
    function sample(seed: number): number[] {
      const rng = createRng(seed);
      return [
        rng.next(),
        rng.range(-3, 9),
        rng.int(0, 100),
        rng.angle(),
        rng.gaussian(),
        rng.gaussian(),
        rng.next(),
      ];
    }
    expect(sample(7)).toEqual(sample(7));
  });

  it('different seeds give different sequences', () => {
    const first = draw(mulberry32(1), 100);
    const second = draw(mulberry32(2), 100);
    expect(second).not.toEqual(first);

    let same = 0;
    for (let i = 0; i < first.length; i += 1) {
      if (first[i] === second[i]) same += 1;
    }
    expect(same).toBe(0);
  });

  it('range stays inside its bounds over 10000 draws', () => {
    const rng = createRng(99);
    for (let i = 0; i < 10000; i += 1) {
      const value = rng.range(-12.5, 4.25);
      expect(value).toBeGreaterThanOrEqual(-12.5);
      expect(value).toBeLessThan(4.25);
    }
  });

  it('int stays inside its bounds and returns whole numbers over 10000 draws', () => {
    const rng = createRng(1234);
    const seen = new Set<number>();
    for (let i = 0; i < 10000; i += 1) {
      const value = rng.int(-5, 12);
      expect(Number.isInteger(value)).toBe(true);
      expect(value).toBeGreaterThanOrEqual(-5);
      expect(value).toBeLessThan(12);
      seen.add(value);
    }
    // Every value of the range appears at least one time.
    expect(seen.size).toBe(17);
  });

  it('angle stays inside 0 to two pi', () => {
    const rng = createRng(5);
    for (let i = 0; i < 10000; i += 1) {
      const value = rng.angle();
      expect(value).toBeGreaterThanOrEqual(0);
      expect(value).toBeLessThan(2 * Math.PI);
    }
  });

  it('pick returns an item of the array and reaches every item', () => {
    const rng = createRng(41);
    const items = ['pine', 'birch', 'oak', 'hangar'] as const;
    const seen = new Set<string>();
    for (let i = 0; i < 1000; i += 1) {
      const item = rng.pick(items);
      expect(items).toContain(item);
      seen.add(item);
    }
    expect(seen.size).toBe(items.length);
    expect(() => rng.pick([])).toThrow(RangeError);
  });

  it('gaussian has a mean near 0 and a standard deviation near 1 over 100000 draws', () => {
    const rng = createRng(2026);
    const count = 100000;
    let sum = 0;
    let sumOfSquares = 0;
    for (let i = 0; i < count; i += 1) {
      const value = rng.gaussian();
      sum += value;
      sumOfSquares += value * value;
    }
    const mean = sum / count;
    const variance = sumOfSquares / count - mean * mean;
    const deviation = Math.sqrt(variance);

    // The standard error of the mean is 1 / sqrt(100000), near 0.0032. A bound
    // of 0.02 is more than six times that error.
    expect(Math.abs(mean)).toBeLessThan(0.02);
    expect(Math.abs(deviation - 1)).toBeLessThan(0.02);
  });
});
