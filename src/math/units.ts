/**
 * Unit conversions.
 *
 * The model holds every value in SI units. Use these functions only at the edge
 * of the model, where a gauge, a data sheet, or a test uses another unit. Never
 * write a conversion factor such as 3.6 anywhere else in the code.
 *
 * This module is pure math. It imports nothing.
 */

/** Radians in one degree. */
export const DEG = Math.PI / 180;

/** Degrees in one radian. */
const RAD = 180 / Math.PI;

// Standard gravity, the value the CGPM fixed in 1901. The kilogram-force and the
// ISA both use it. Source: BIPM SI Brochure, confidence: firm.
export const G0 = 9.80665; // m/s^2

// The international foot. The value is exact since the 1959 agreement.
const METER_PER_FOOT = 0.3048; // m

// The international nautical mile. The value is exact by definition.
const METER_PER_NAUTICAL_MILE = 1852; // m

const SECOND_PER_HOUR = 3600; // s

const METER_PER_KILOMETER = 1000; // m

// One knot is one nautical mile per hour.
const MS_PER_KNOT = METER_PER_NAUTICAL_MILE / SECOND_PER_HOUR; // m/s

// One kilometer per hour in meters per second.
const MS_PER_KMH = METER_PER_KILOMETER / SECOND_PER_HOUR; // m/s

// One revolution in radians.
const RAD_PER_REVOLUTION = 2 * Math.PI; // rad

const SECOND_PER_MINUTE = 60; // s

// The offset between the kelvin scale and the Celsius scale. The value is exact
// by definition of the Celsius scale.
const KELVIN_AT_ZERO_CELSIUS = 273.15; // K

// One hectopascal in pascals. The hectopascal is the unit on an altimeter
// subscale and on a weather chart.
const PA_PER_HPA = 100; // Pa

/** Converts an angle from radians to degrees. */
export function toDeg(rad: number): number {
  return rad * RAD;
}

/** Converts an angle from degrees to radians. */
export function toRad(deg: number): number {
  return deg * DEG;
}

/** Converts a speed from meters per second to kilometers per hour. */
export function msToKmh(v: number): number {
  return v / MS_PER_KMH;
}

/** Converts a speed from kilometers per hour to meters per second. */
export function kmhToMs(v: number): number {
  return v * MS_PER_KMH;
}

/** Converts a speed from meters per second to knots. */
export function msToKt(v: number): number {
  return v / MS_PER_KNOT;
}

/** Converts a speed from knots to meters per second. */
export function ktToMs(v: number): number {
  return v * MS_PER_KNOT;
}

/** Converts a length from meters to feet. */
export function mToFt(v: number): number {
  return v / METER_PER_FOOT;
}

/** Converts a length from feet to meters. */
export function ftToM(v: number): number {
  return v * METER_PER_FOOT;
}

/** Converts an angular rate from radians per second to revolutions per minute. */
export function radPerSecToRpm(w: number): number {
  return (w * SECOND_PER_MINUTE) / RAD_PER_REVOLUTION;
}

/** Converts an angular rate from revolutions per minute to radians per second. */
export function rpmToRadPerSec(rpm: number): number {
  return (rpm * RAD_PER_REVOLUTION) / SECOND_PER_MINUTE;
}

/** Converts a temperature from kelvin to degrees Celsius. */
export function kelvinToCelsius(k: number): number {
  return k - KELVIN_AT_ZERO_CELSIUS;
}

/** Converts a temperature from degrees Celsius to kelvin. */
export function celsiusToKelvin(c: number): number {
  return c + KELVIN_AT_ZERO_CELSIUS;
}

/** Converts a force from newtons to kilograms-force. */
export function nToKgf(n: number): number {
  return n / G0;
}

/** Converts a force from kilograms-force to newtons. */
export function kgfToN(kgf: number): number {
  return kgf * G0;
}

/** Converts a pressure from pascals to hectopascals. */
export function paToHpa(pa: number): number {
  return pa / PA_PER_HPA;
}

/** Converts a pressure from hectopascals to pascals. */
export function hpaToPa(hpa: number): number {
  return hpa * PA_PER_HPA;
}
