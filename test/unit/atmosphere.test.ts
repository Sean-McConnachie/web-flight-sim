import { describe, expect, it } from 'vitest';
import {
  RHO0,
  SEA_LEVEL_DENSITY,
  createAtmosphereSample,
  dynamicPressure,
  equivalentAirspeed,
  isa,
  machNumber,
  trueAirspeed,
} from '@/physics/atmosphere';
import type { AtmosphereSample } from '@/physics/atmosphere';

/** Checks a value against a table entry to four significant figures. */
function expectRelative(actual: number, expected: number, tolerance = 1e-4): void {
  const error = Math.abs(actual - expected) / Math.abs(expected);
  expect(
    error,
    `expected ${actual} to be within ${tolerance} of ${expected}, relative error ${error}`,
  ).toBeLessThan(tolerance);
}

interface TableRow {
  readonly altitude: number; // m
  readonly temperature: number; // K
  readonly pressure: number; // Pa
  readonly density: number; // kg/m^3
  readonly speedOfSound?: number; // m/s
}

// Published values of the International Standard Atmosphere, ISO 2533:1975.
const TABLE: readonly TableRow[] = [
  { altitude: 0, temperature: 288.15, pressure: 101325, density: 1.225, speedOfSound: 340.29 },
  { altitude: 5000, temperature: 255.65, pressure: 54019, density: 0.73612, speedOfSound: 320.53 },
  { altitude: 6000, temperature: 249.15, pressure: 47181, density: 0.65970, speedOfSound: 316.43 },
  { altitude: 11000, temperature: 216.65, pressure: 22632, density: 0.36392, speedOfSound: 295.07 },
  { altitude: 15000, temperature: 216.65, pressure: 12045, density: 0.19367 },
];

describe('standard atmosphere', () => {
  for (const row of TABLE) {
    it(`temperature, pressure and density at ${row.altitude} m match the standard atmosphere`, () => {
      const s = isa(row.altitude);
      expectRelative(s.temperature, row.temperature);
      expectRelative(s.pressure, row.pressure);
      expectRelative(s.density, row.density);
      if (row.speedOfSound !== undefined) {
        expectRelative(s.speedOfSound, row.speedOfSound);
      }
    });
  }

  it('sea level density matches the exported constant', () => {
    expect(SEA_LEVEL_DENSITY).toBe(1.225);
    expect(SEA_LEVEL_DENSITY).toBe(RHO0);
    expectRelative(isa(0).density, SEA_LEVEL_DENSITY);
  });

  it('the two layers join with no step in pressure at 11000 m', () => {
    const below = isa(11000 - 1e-6);
    const above = isa(11000 + 1e-6);
    expectRelative(above.pressure, below.pressure, 1e-9);
    expectRelative(above.temperature, below.temperature, 1e-9);
  });

  it('pressure falls with every step of altitude up to 20000 m', () => {
    let previous = isa(-1000).pressure;
    for (let altitude = -900; altitude <= 20000; altitude += 100) {
      const p = isa(altitude).pressure;
      expect(p, `pressure at ${altitude} m`).toBeLessThan(previous);
      previous = p;
    }
    expect(previous).toBeGreaterThan(0);
  });

  it('density below the datum is above the sea level value and stays finite', () => {
    for (const altitude of [-0.5, -5, -100, -500]) {
      const s = isa(altitude);
      expect(Number.isFinite(s.temperature), `temperature at ${altitude} m`).toBe(true);
      expect(Number.isFinite(s.pressure), `pressure at ${altitude} m`).toBe(true);
      expect(Number.isFinite(s.density), `density at ${altitude} m`).toBe(true);
      expect(Number.isFinite(s.speedOfSound), `speed of sound at ${altitude} m`).toBe(true);
      expect(Number.isFinite(s.dynamicViscosity), `viscosity at ${altitude} m`).toBe(true);
      expect(s.density).toBeGreaterThan(SEA_LEVEL_DENSITY);
    }
  });

  it('dynamic viscosity at sea level matches the Sutherland law', () => {
    // 1.458e-6 * 288.15^1.5 / (288.15 + 110.4) = 1.7894e-5 Pa s.
    expectRelative(isa(0).dynamicViscosity, 1.7894e-5);
  });

  it('isa writes into the sample the caller gives and returns that same sample', () => {
    const sample: AtmosphereSample = createAtmosphereSample();
    const result = isa(6000, sample);
    expect(result).toBe(sample);
    expectRelative(sample.density, 0.65970);
    expect(sample.altitude).toBe(6000);
  });
});

describe('flow quantities', () => {
  it('Mach number is the speed over the local speed of sound', () => {
    const s = isa(6000);
    // 870 km/h is 241.67 m/s, the maximum level speed at 6000 m.
    expectRelative(machNumber(241.67, s.speedOfSound), 241.67 / 316.43, 1e-3);
    expect(machNumber(100, 0)).toBe(0);
  });

  it('dynamic pressure is half rho V squared', () => {
    expectRelative(dynamicPressure(1.225, 100), 0.5 * 1.225 * 10000, 1e-12);
  });

  it('equivalent airspeed equals true airspeed at sea level', () => {
    expectRelative(equivalentAirspeed(200, SEA_LEVEL_DENSITY), 200, 1e-12);
  });

  it('equivalent airspeed is below true airspeed at altitude', () => {
    const s = isa(6000);
    const eas = equivalentAirspeed(241.67, s.density);
    expect(eas).toBeLessThan(241.67);
    // EAS = TAS * sqrt(rho / rho0) = 241.67 * sqrt(0.65970 / 1.225).
    expectRelative(eas, 241.67 * Math.sqrt(0.6597 / 1.225), 1e-4);
  });

  it('the equivalent airspeed and true airspeed conversions invert each other', () => {
    for (const altitude of [0, 3000, 6000, 11000, 15000]) {
      const density = isa(altitude).density;
      const tas = 180;
      const back = trueAirspeed(equivalentAirspeed(tas, density), density);
      expectRelative(back, tas, 1e-12);
    }
  });

  it('the conversions report zero for a density of zero', () => {
    expect(equivalentAirspeed(200, 0)).toBe(0);
    expect(trueAirspeed(200, 0)).toBe(0);
  });
});
