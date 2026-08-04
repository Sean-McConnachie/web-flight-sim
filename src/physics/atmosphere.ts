/**
 * The International Standard Atmosphere, from sea level to 20000 m.
 *
 * The model holds two layers. The troposphere runs from sea level to 11000 m
 * with a constant temperature lapse rate. The tropopause runs from 11000 m to
 * 20000 m at a constant temperature. The Me-262 service ceiling is 11450 m, so
 * the aircraft stays inside the first layer for almost all of a flight.
 *
 * Below sea level the model keeps the troposphere equation. The gear can
 * compress and put the center of gravity below the datum. A NaN at that point
 * would be hard to find, so the equation must stay finite there.
 *
 * The physics step reads this model 240 times per second. Pass an out sample to
 * isa to make the call allocate nothing.
 *
 * This module is pure math. It imports the standard gravity constant only.
 */

import { G0 } from '@/math/units';

// Sea level temperature of the standard atmosphere.
// Source: ISO 2533:1975, confidence: firm.
export const T0 = 288.15; // K

// Sea level pressure of the standard atmosphere.
// Source: ISO 2533:1975, confidence: firm.
export const P0 = 101325; // Pa

// Sea level density of the standard atmosphere. The value follows from T0, P0
// and R_AIR through the gas law.
// Source: ISO 2533:1975, confidence: firm.
export const RHO0 = 1.225; // kg/m^3

/**
 * The same value as RHO0. The shared contract in CONVENTIONS section 6 names the
 * sea level density SEA_LEVEL_DENSITY, so both names point at one number.
 */
export const SEA_LEVEL_DENSITY = RHO0; // kg/m^3

// Specific gas constant of dry air.
// Source: ISO 2533:1975, confidence: firm.
export const R_AIR = 287.05287; // J/(kg K)

// Ratio of the specific heats of dry air. The value holds while the air stays
// far below the dissociation temperature.
// Source: ISO 2533:1975, confidence: firm.
export const GAMMA = 1.4;

// Temperature lapse rate in the troposphere. The temperature falls as the
// altitude rises, so the value is negative.
// Source: ISO 2533:1975, confidence: firm.
export const L = -0.0065; // K/m

// Top of the troposphere and base of the tropopause.
// Source: ISO 2533:1975, confidence: firm.
export const TROPOPAUSE_ALTITUDE = 11000; // m

// Temperature of the tropopause. The layer is isothermal.
// Source: ISO 2533:1975, confidence: firm.
export const TROPOPAUSE_TEMPERATURE = 216.65; // K

// Top of the range this model covers. Above this altitude the temperature rises
// again in the real atmosphere, and this model still reports the tropopause.
// Source: ISO 2533:1975, confidence: firm.
export const MODEL_CEILING = 20000; // m

// Sutherland constants for the viscosity of air.
// Source: White, "Viscous Fluid Flow", 3rd edition, table 1-2, confidence: firm.
const SUTHERLAND_B = 1.458e-6; // kg/(m s K^0.5)
const SUTHERLAND_S = 110.4; // K

// Exponent of the pressure equation in the troposphere. The value comes from the
// hydrostatic equation with a linear temperature profile.
const PRESSURE_EXPONENT = -G0 / (L * R_AIR);

// Pressure at the base of the tropopause. The value carries over from the
// troposphere equation, so the two layers join with no step.
const TROPOPAUSE_PRESSURE = P0 * Math.pow(TROPOPAUSE_TEMPERATURE / T0, PRESSURE_EXPONENT); // Pa

export interface AtmosphereSample {
  altitude: number; // m
  temperature: number; // K
  pressure: number; // Pa
  density: number; // kg/m^3
  speedOfSound: number; // m/s
  dynamicViscosity: number; // Pa s
}

/** Makes one sample that the caller can pass to isa again and again. */
export function createAtmosphereSample(): AtmosphereSample {
  return {
    altitude: 0,
    temperature: T0,
    pressure: P0,
    density: RHO0,
    speedOfSound: Math.sqrt(GAMMA * R_AIR * T0),
    dynamicViscosity: (SUTHERLAND_B * T0 * Math.sqrt(T0)) / (T0 + SUTHERLAND_S),
  };
}

/**
 * Reads the standard atmosphere at a geopotential altitude in meters.
 *
 * The function writes into out when the caller gives one, and allocates nothing
 * on that path. Without out the function makes a new sample.
 */
export function isa(altitude: number, out?: AtmosphereSample): AtmosphereSample {
  const sample = out !== undefined ? out : createAtmosphereSample();

  let temperature: number;
  let pressure: number;
  if (altitude < TROPOPAUSE_ALTITUDE) {
    // The same equation runs below sea level. It stays finite there.
    temperature = T0 + L * altitude;
    pressure = P0 * Math.pow(temperature / T0, PRESSURE_EXPONENT);
  } else {
    temperature = TROPOPAUSE_TEMPERATURE;
    pressure =
      TROPOPAUSE_PRESSURE *
      Math.exp((-G0 * (altitude - TROPOPAUSE_ALTITUDE)) / (R_AIR * TROPOPAUSE_TEMPERATURE));
  }

  sample.altitude = altitude;
  sample.temperature = temperature;
  sample.pressure = pressure;
  sample.density = pressure / (R_AIR * temperature);
  sample.speedOfSound = Math.sqrt(GAMMA * R_AIR * temperature);
  // Sutherland law. The power of 1.5 becomes one multiply and one square root.
  sample.dynamicViscosity =
    (SUTHERLAND_B * temperature * Math.sqrt(temperature)) / (temperature + SUTHERLAND_S);
  return sample;
}

/** Returns the Mach number for a speed and the local speed of sound. */
export function machNumber(speed: number, speedOfSound: number): number {
  if (speedOfSound <= 0) {
    return 0;
  }
  return speed / speedOfSound;
}

/** Returns the dynamic pressure in pascals. */
export function dynamicPressure(density: number, speed: number): number {
  return 0.5 * density * speed * speed;
}

/**
 * Converts a true airspeed to an equivalent airspeed, with EAS = TAS * sqrt(rho / rho0).
 *
 * The Me-262 airspeed indicator reads a value close to the equivalent airspeed,
 * because the instrument measures dynamic pressure. Every speed in the pilot
 * notes is therefore an indicated speed, not a true speed. The clean stall speed
 * of 175 km/h is one of them. The flight tests compare against indicated speed,
 * so they must convert with this function first.
 */
export function equivalentAirspeed(trueAirspeed: number, density: number): number {
  if (density <= 0) {
    return 0;
  }
  return trueAirspeed * Math.sqrt(density / RHO0);
}

/** Converts an equivalent airspeed back to a true airspeed. */
export function trueAirspeed(equivalentAirspeed: number, density: number): number {
  if (density <= 0) {
    return 0;
  }
  return equivalentAirspeed * Math.sqrt(RHO0 / density);
}
