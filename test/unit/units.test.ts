import { describe, expect, it } from 'vitest';
import {
  DEG,
  G0,
  celsiusToKelvin,
  ftToM,
  hpaToPa,
  kelvinToCelsius,
  kgfToN,
  kmhToMs,
  ktToMs,
  mToFt,
  msToKmh,
  msToKt,
  nToKgf,
  paToHpa,
  radPerSecToRpm,
  rpmToRadPerSec,
  toDeg,
  toRad,
} from '@/math/units';

type Convert = (v: number) => number;

interface Pair {
  readonly name: string;
  readonly forward: Convert;
  readonly back: Convert;
  readonly sample: number;
}

// One sample for each pair. The sample is a value the simulator sees in flight.
const PAIRS: readonly Pair[] = [
  { name: 'radian and degree', forward: toDeg, back: toRad, sample: 0.35 },
  { name: 'meter per second and kilometer per hour', forward: msToKmh, back: kmhToMs, sample: 241.7 },
  { name: 'meter per second and knot', forward: msToKt, back: ktToMs, sample: 241.7 },
  { name: 'meter and foot', forward: mToFt, back: ftToM, sample: 6000 },
  { name: 'radian per second and rpm', forward: radPerSecToRpm, back: rpmToRadPerSec, sample: 911.06 },
  { name: 'kelvin and degree Celsius', forward: kelvinToCelsius, back: celsiusToKelvin, sample: 288.15 },
  { name: 'newton and kilogram-force', forward: nToKgf, back: kgfToN, sample: 8800 },
  { name: 'pascal and hectopascal', forward: paToHpa, back: hpaToPa, sample: 101325 },
];

describe('unit conversions', () => {
  for (const pair of PAIRS) {
    it(`the round trip between ${pair.name} returns the same value`, () => {
      expect(pair.back(pair.forward(pair.sample))).toBeCloseTo(pair.sample, 9);
      expect(pair.forward(pair.back(pair.sample))).toBeCloseTo(pair.sample, 9);
    });
  }

  it('one degree is pi divided by 180 radians', () => {
    expect(DEG).toBe(Math.PI / 180);
    expect(toRad(1)).toBe(DEG);
    expect(toDeg(Math.PI)).toBeCloseTo(180, 12);
    expect(toRad(90)).toBeCloseTo(Math.PI / 2, 12);
  });

  it('100 km/h is 27.7778 m/s', () => {
    expect(kmhToMs(100)).toBeCloseTo(27.7778, 4);
    expect(msToKmh(27.7778)).toBeCloseTo(100, 3);
  });

  it('one knot is 0.514444 m/s', () => {
    expect(ktToMs(1)).toBeCloseTo(0.514444, 5);
    expect(ktToMs(1)).toBe(1852 / 3600);
    expect(msToKt(1)).toBeCloseTo(1.943844, 5);
  });

  it('one meter is 3.28084 ft', () => {
    expect(mToFt(1)).toBeCloseTo(3.28084, 5);
    expect(ftToM(1)).toBe(0.3048);
  });

  it('the service ceiling of 11450 m is 37566 ft', () => {
    expect(mToFt(11450)).toBeCloseTo(37565.6, 1);
  });

  it('the maximum rotor speed of 8700 rpm is 911.06 rad/s', () => {
    expect(rpmToRadPerSec(8700)).toBeCloseTo(911.06, 2);
    expect(radPerSecToRpm((8700 * 2 * Math.PI) / 60)).toBeCloseTo(8700, 9);
    expect(radPerSecToRpm(Math.PI * 2)).toBeCloseTo(60, 12);
  });

  it('standard gravity is 9.80665 m/s2 and one kilogram-force is that many newtons', () => {
    expect(G0).toBe(9.80665);
    expect(kgfToN(1)).toBe(G0);
    expect(nToKgf(8800)).toBeCloseTo(897.35, 2);
  });

  it('the ISA sea level temperature of 288.15 K is 15 C', () => {
    expect(kelvinToCelsius(288.15)).toBeCloseTo(15, 12);
    expect(celsiusToKelvin(15)).toBeCloseTo(288.15, 12);
    expect(kelvinToCelsius(0)).toBe(-273.15);
  });

  it('the ISA sea level pressure of 101325 Pa is 1013.25 hPa', () => {
    expect(paToHpa(101325)).toBeCloseTo(1013.25, 10);
    expect(hpaToPa(1013.25)).toBeCloseTo(101325, 8);
  });

  it('a conversion keeps the sign of a negative value', () => {
    expect(msToKmh(-10)).toBeCloseTo(-36, 12);
    expect(toDeg(-Math.PI)).toBeCloseTo(-180, 12);
  });
});
