import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';

import { DEG, kelvinToCelsius, msToKmh, radPerSecToRpm, toDeg } from '@/math/units';
import { isa } from '@/physics/atmosphere';
import { createState } from '@/physics/rigidbody';
import {
  DANGER_BAND_RPM,
  IDLE_RPM,
  MAX_FUEL_FLOW,
  MAX_RPM,
  OMEGA_MAX,
  TURBINE_INLET_TEMPERATURE_LIMIT,
} from '@/aircraft/me262/engine';
import type { TelemetrySample } from '@/ui/debug-overlay';

import { GAUGE_NAMES } from '@/ui/gauges';
import { dialAngle, linearDial, tableDial, tickValues } from '@/ui/gauges/dial';
import { createLag, stepLag, stepLagWrapped } from '@/ui/gauges/lag';
import type { CockpitReadout } from '@/ui/gauges/readout';
import {
  AIRSPEED_LAG,
  AIRSPEED_LAW,
  airspeedAngle,
  airspeedReading,
} from '@/ui/gauges/airspeed';
import {
  ALTIMETER_FULL_RANGE,
  ALTIMETER_HAND_RANGE,
  altimeterHandAngle,
  altimeterKilometerAngle,
  altimeterReading,
} from '@/ui/gauges/altimeter';
import { VARIOMETER_LAW, variometerAngle, variometerReading } from '@/ui/gauges/variometer';
import {
  BALL_LAG,
  STANDARD_RATE,
  TURN_FULL_SCALE,
  TURN_LAG,
  ballPosition,
  turnAngle,
  turnReading,
} from '@/ui/gauges/turn-slip';
import { compassCardRotation, compassReading } from '@/ui/gauges/compass';
import {
  HORIZON_BANK_LAG,
  HORIZON_ERECTION_LIMIT,
  HORIZON_PITCH_LAG,
  erectionTarget,
  horizonBankTarget,
  horizonOffset,
  horizonPitchTarget,
} from '@/ui/gauges/horizon';
import {
  TACHOMETER_MAX_RPM,
  tachometerAngle,
  tachometerReading,
} from '@/ui/gauges/tachometer';
import {
  GAS_TEMPERATURE_LIMIT_C,
  GAS_TEMPERATURE_MAX,
  gasTemperatureAngle,
  gasTemperatureReading,
} from '@/ui/gauges/gas-temperature';
import { FUEL_LOW_MARK, FUEL_SCALE_MAX, fuelAngle } from '@/ui/gauges/fuel';
import { clockHourAngle, clockMinuteAngle, clockSecondAngle } from '@/ui/gauges/clock';
import { homingCourse, homingStrength } from '@/ui/gauges/homing';
import {
  PRESSURE_SCALE_MAX,
  fuelPressure,
  oilPressure,
  pressureAngle,
} from '@/ui/gauges/pressure';

/**
 * Tests for the live cockpit instruments.
 *
 * Every face is painted on a 2D canvas, and this harness runs in Node with no
 * DOM at all. The tests therefore drive only the parts that hold no browser:
 * the dial law of each instrument, the needle lag, and the unit conversions.
 * Nothing here builds a mesh or opens a canvas.
 */

const TWO_PI = Math.PI * 2;

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

function makeReadout(): CockpitReadout {
  return {
    engines: [
      { rotorSpeed: 0, gasTemperature: 288.15, fuelFlow: 0 },
      { rotorSpeed: 0, gasTemperature: 288.15, fuelFlow: 0 },
    ],
    fuelMass: 0,
    lateralAcceleration: 0,
    longitudinalAcceleration: 0,
  };
}

// ---------------------------------------------------------------------------
// The dial law itself
// ---------------------------------------------------------------------------

describe('the dial law', () => {
  it('places an even law at its start, its middle and its end', () => {
    const law = linearDial(0, 100, -90, 180);
    expect(dialAngle(law, 0)).toBeCloseTo(-90 * DEG, 12);
    expect(dialAngle(law, 50)).toBeCloseTo(0, 12);
    expect(dialAngle(law, 100)).toBeCloseTo(90 * DEG, 12);
  });

  it('holds the needle on its stop past both ends of the scale', () => {
    const law = linearDial(0, 100, -90, 180);
    expect(dialAngle(law, -1000)).toBe(dialAngle(law, 0));
    expect(dialAngle(law, 1e9)).toBe(dialAngle(law, 100));
  });

  it('interpolates inside one segment of a table law', () => {
    const law = tableDial([0, 10, 100], [0, 90, 180]);
    expect(dialAngle(law, 5)).toBeCloseTo(45 * DEG, 12);
    expect(dialAngle(law, 55)).toBeCloseTo(135 * DEG, 12);
  });

  it('places the last tick exactly on the end of the range', () => {
    const values = tickValues(0, 1000, 20);
    expect(values.length).toBe(51);
    expect(values[50]).toBe(1000);
  });
});

// ---------------------------------------------------------------------------
// The lag
// ---------------------------------------------------------------------------

describe('the needle lag', () => {
  it('puts the needle on its first target with no sweep from zero', () => {
    const lag = createLag(0.5, 0);
    expect(stepLag(lag, 400, 1 / 60)).toBe(400);
  });

  it('reaches 63 percent of a step in one time constant', () => {
    const lag = createLag(0.5, 0);
    stepLag(lag, 0, 1 / 60);
    stepLag(lag, 1, 0.5);
    expect(lag.value).toBeCloseTo(1 - Math.exp(-1), 12);
  });

  it('reaches its target and settles there', () => {
    const lag = createLag(0.4, 0);
    stepLag(lag, 0, 1 / 60);
    // Ten seconds is twenty five time constants, so nothing measurable is left.
    for (let i = 0; i < 300; i++) stepLag(lag, 250, 2 / 60);
    expect(lag.value).toBeCloseTo(250, 6);
  });

  it('never passes its target, whatever the frame time', () => {
    // A frame far longer than the time constant is the case that breaks the
    // plain Euler form. The exact form must still stop at the target.
    for (const dt of [0.001, 1 / 60, 0.5, 4, 60]) {
      const lag = createLag(0.05, 0);
      stepLag(lag, 0, 1 / 60);
      let previous = lag.value;
      for (let i = 0; i < 20; i++) {
        const value = stepLag(lag, 100, dt);
        expect(value).toBeLessThanOrEqual(100);
        expect(value).toBeGreaterThanOrEqual(previous);
        previous = value;
      }
    }
  });

  it('never passes a target below it either', () => {
    const lag = createLag(0.05, 0);
    stepLag(lag, 100, 1 / 60);
    let previous = lag.value;
    for (let i = 0; i < 20; i++) {
      const value = stepLag(lag, -40, 1);
      expect(value).toBeGreaterThanOrEqual(-40);
      expect(value).toBeLessThanOrEqual(previous);
      previous = value;
    }
  });

  it('takes the short way round a wrapped scale', () => {
    // A card at 350 degrees that must reach 10 degrees moves 20 degrees
    // FORWARD, not 340 degrees back.
    const lag = createLag(1, 0);
    stepLagWrapped(lag, 350 * DEG, TWO_PI, 1 / 60);
    stepLagWrapped(lag, 10 * DEG, TWO_PI, 0.1);
    const moved = lag.value - 350 * DEG;
    expect(moved).toBeGreaterThan(0);
    expect(toDeg(moved)).toBeLessThan(20);
  });

  it('holds a wrapped needle inside one turn', () => {
    const lag = createLag(0.2, 0);
    stepLagWrapped(lag, 5 * DEG, TWO_PI, 1 / 60);
    for (let i = 0; i < 200; i++) stepLagWrapped(lag, 355 * DEG, TWO_PI, 1 / 60);
    expect(lag.value).toBeGreaterThanOrEqual(0);
    expect(lag.value).toBeLessThan(TWO_PI);
    expect(toDeg(lag.value)).toBeCloseTo(355, 3);
  });
});

// ---------------------------------------------------------------------------
// Fahrtmesser, the airspeed indicator
// ---------------------------------------------------------------------------

describe('the Fahrtmesser', () => {
  it('reads 0 to 1000 km/h, which is the range of the Fl.22245', () => {
    expect(AIRSPEED_LAW.min).toBe(0);
    expect(AIRSPEED_LAW.max).toBe(1000);
  });

  it('places the needle at the two ends and at the knot of the law', () => {
    expect(toDeg(airspeedAngle(0))).toBeCloseTo(-155, 9);
    expect(toDeg(airspeedAngle(100))).toBeCloseTo(-128, 9);
    expect(toDeg(airspeedAngle(1000))).toBeCloseTo(155, 9);
  });

  it('places the needle in the middle of the upper segment', () => {
    // 550 km/h is halfway from 100 to 1000, so it is halfway from -128 to 155.
    expect(toDeg(airspeedAngle(550))).toBeCloseTo((-128 + 155) / 2, 9);
  });

  it('crowds the first hundred, as a dynamic pressure capsule does', () => {
    const first = airspeedAngle(100) - airspeedAngle(0);
    const second = airspeedAngle(200) - airspeedAngle(100);
    expect(first).toBeLessThan(second);
  });

  it('holds the needle on its stop past both ends', () => {
    expect(airspeedAngle(-40)).toBe(airspeedAngle(0));
    expect(airspeedAngle(1400)).toBe(airspeedAngle(1000));
  });

  it('reads the EQUIVALENT airspeed in km/h, through src/math/units.ts', () => {
    const sample = makeSample();
    sample.trueAirspeed = 241.7;
    sample.equivalentAirspeed = 177.8;
    expect(airspeedReading(sample)).toBe(msToKmh(177.8));
    expect(airspeedReading(sample)).toBeCloseTo(640.1, 1);
  });

  it('lags by a third of a second, which a capsule instrument does', () => {
    expect(AIRSPEED_LAG).toBeGreaterThan(0.2);
    expect(AIRSPEED_LAG).toBeLessThan(0.6);
  });
});

// ---------------------------------------------------------------------------
// Hoehenmesser, the altimeter
// ---------------------------------------------------------------------------

describe('the Hoehenmesser', () => {
  it('turns the long hand one full circle every 1000 m', () => {
    expect(ALTIMETER_HAND_RANGE).toBe(1000);
    expect(altimeterHandAngle(0)).toBeCloseTo(0, 12);
    expect(altimeterHandAngle(250)).toBeCloseTo(Math.PI / 2, 12);
    expect(altimeterHandAngle(500)).toBeCloseTo(Math.PI, 12);
  });

  it('brings the long hand back to the top at every kilometer', () => {
    expect(altimeterHandAngle(1000)).toBeCloseTo(0, 12);
    expect(altimeterHandAngle(7000)).toBeCloseTo(0, 12);
    expect(altimeterHandAngle(7250)).toBeCloseTo(altimeterHandAngle(250), 12);
  });

  it('winds the long hand back below the datum instead of jumping it', () => {
    expect(altimeterHandAngle(-250)).toBeCloseTo(altimeterHandAngle(750), 12);
  });

  it('turns the short hand once over the whole range', () => {
    expect(ALTIMETER_FULL_RANGE).toBe(12000);
    expect(altimeterKilometerAngle(0)).toBeCloseTo(0, 12);
    expect(altimeterKilometerAngle(6000)).toBeCloseTo(Math.PI, 12);
    expect(altimeterKilometerAngle(12000)).toBeCloseTo(TWO_PI, 12);
  });

  it('keeps the service ceiling of 11450 m on the dial', () => {
    // A short hand that pegs below the ceiling of its own aircraft is useless,
    // so the range runs past it. CONVENTIONS section 8 gives 11450 m.
    expect(altimeterKilometerAngle(11450)).toBeLessThan(altimeterKilometerAngle(12000));
    expect(altimeterKilometerAngle(11450)).toBeGreaterThan(altimeterKilometerAngle(11000));
  });

  it('holds the short hand on its stop past the top of the range', () => {
    expect(altimeterKilometerAngle(20000)).toBe(altimeterKilometerAngle(12000));
    expect(altimeterKilometerAngle(-500)).toBe(altimeterKilometerAngle(0));
  });

  it('reads MINUS the world z, as CONVENTIONS section 3.2 demands', () => {
    const sample = makeSample();
    sample.state.position.set(120, -30, -6000);
    expect(altimeterReading(sample)).toBe(6000);
  });
});

// ---------------------------------------------------------------------------
// Variometer, the vertical speed indicator
// ---------------------------------------------------------------------------

describe('the Variometer', () => {
  it('reads 30 m/s each way, which is the range of the Fl.22386', () => {
    expect(VARIOMETER_LAW.min).toBe(-30);
    expect(VARIOMETER_LAW.max).toBe(30);
  });

  it('puts zero at nine o clock, with climb clockwise over the top', () => {
    expect(toDeg(variometerAngle(0))).toBeCloseTo(-90, 9);
    expect(variometerAngle(5)).toBeGreaterThan(variometerAngle(0));
    expect(variometerAngle(-5)).toBeLessThan(variometerAngle(0));
  });

  it('places the needle at both ends of the scale', () => {
    expect(toDeg(variometerAngle(30))).toBeCloseTo(70, 9);
    expect(toDeg(variometerAngle(-30))).toBeCloseTo(-250, 9);
  });

  it('crowds the top of the scale, as a real face does', () => {
    const nearZero = variometerAngle(5) - variometerAngle(0);
    const nearFull = variometerAngle(30) - variometerAngle(20);
    expect(nearFull).toBeLessThan(nearZero);
  });

  it('interpolates between two marked values', () => {
    // 15 m/s sits halfway from 10 to 20, so its angle sits halfway as well.
    expect(toDeg(variometerAngle(15))).toBeCloseTo((0 + 45) / 2, 9);
  });

  it('holds the needle on its stop past both ends', () => {
    expect(variometerAngle(90)).toBe(variometerAngle(30));
    expect(variometerAngle(-90)).toBe(variometerAngle(-30));
  });

  it('reads MINUS the world z velocity, so a climb is positive', () => {
    const sample = makeSample();
    sample.state.velocity.set(200, 0, -18);
    expect(variometerReading(sample)).toBe(18);
  });

  it('shows the published sea level climb of 20 m/s on the dial', () => {
    expect(variometerAngle(20)).toBeLessThan(variometerAngle(30));
    expect(variometerAngle(20)).toBeGreaterThan(variometerAngle(10));
  });
});

// ---------------------------------------------------------------------------
// Wendezeiger, the turn and slip indicator
// ---------------------------------------------------------------------------

describe('the Wendezeiger needle', () => {
  it('reads the yaw rate in degrees per second', () => {
    const sample = makeSample();
    sample.state.angularVelocity.set(0.2, -0.1, 0.05236);
    expect(turnReading(sample)).toBe(toDeg(0.05236));
    expect(turnReading(sample)).toBeCloseTo(3, 3);
  });

  it('stands upright at zero and leans right for a right turn', () => {
    expect(turnAngle(0)).toBeCloseTo(0, 12);
    expect(turnAngle(STANDARD_RATE)).toBeGreaterThan(0);
    expect(turnAngle(-STANDARD_RATE)).toBeLessThan(0);
  });

  it('reaches its stop at twice the standard rate', () => {
    expect(TURN_FULL_SCALE).toBe(2 * STANDARD_RATE);
    expect(toDeg(turnAngle(TURN_FULL_SCALE))).toBeCloseTo(32, 9);
    expect(toDeg(turnAngle(-TURN_FULL_SCALE))).toBeCloseTo(-32, 9);
  });

  it('puts the standard rate mark halfway out', () => {
    expect(turnAngle(STANDARD_RATE)).toBeCloseTo(turnAngle(TURN_FULL_SCALE) / 2, 12);
  });

  it('holds the needle on its stop in a hard turn', () => {
    expect(turnAngle(40)).toBe(turnAngle(TURN_FULL_SCALE));
    expect(turnAngle(-40)).toBe(turnAngle(-TURN_FULL_SCALE));
  });

  it('answers faster than the ball, because a rate gyro is quicker', () => {
    expect(TURN_LAG).toBeLessThan(BALL_LAG);
  });
});

describe('the Wendezeiger ball', () => {
  it('centers when the lateral specific force is zero', () => {
    expect(ballPosition(0)).toBeCloseTo(0, 12);
  });

  it('falls toward the low wing in a steady slip', () => {
    // A steady right wing low slip gives a NEGATIVE body y specific force, and
    // the ball must then go RIGHT, which is the low side. Read section 2 of
    // src/ui/gauges/turn-slip.ts.
    const bank = 20 * DEG;
    const lateral = -9.80665 * Math.sin(bank);
    expect(ballPosition(lateral)).toBeCloseTo(Math.sin(bank), 9);
    expect(ballPosition(lateral)).toBeGreaterThan(0);
  });

  it('reaches its stop at one g of side force', () => {
    expect(ballPosition(-9.80665)).toBeCloseTo(1, 9);
    expect(ballPosition(9.80665)).toBeCloseTo(-1, 9);
    expect(ballPosition(-40)).toBe(1);
    expect(ballPosition(40)).toBe(-1);
  });

  it('takes ONE input, and that input is an acceleration', () => {
    // The fault this test exists for is a ball driven by the rudder pedal. The
    // function must have one argument, and the readout must carry no rudder,
    // no yaw command and no sideslip for anyone to reach for.
    expect(ballPosition.length).toBe(1);
    const readout = makeReadout();
    const keys = Object.keys(readout);
    for (const forbidden of ['rudder', 'yaw', 'pedal', 'sideslip', 'beta']) {
      expect(keys).not.toContain(forbidden);
    }
  });

  it('does NOT move with a rudder deflection that makes no side force', () => {
    // A full rudder deflection held in a steady balanced turn shows up as a
    // yaw rate and a sideslip, and it leaves the specific force alone. The
    // needle must swing and the ball must not.
    const sample = makeSample();
    sample.state.angularVelocity.set(0, 0, 0.1);
    sample.beta = 12 * DEG;
    const readout = makeReadout();
    expect(turnAngle(turnReading(sample))).not.toBe(0);
    expect(ballPosition(readout.lateralAcceleration)).toBeCloseTo(0, 12);
  });

  it('DOES move with a side force that no rudder made', () => {
    const readout: CockpitReadout = { ...makeReadout(), lateralAcceleration: -9.80665 / 2 };
    expect(ballPosition(readout.lateralAcceleration)).toBeCloseTo(0.5, 9);
  });
});

// ---------------------------------------------------------------------------
// Kompass, the repeater card
// ---------------------------------------------------------------------------

describe('the Kompass', () => {
  it('reads the heading of the aircraft in the range 0 to one turn', () => {
    const sample = makeSample();
    sample.state.orientation.copy(
      new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), 90 * DEG),
    );
    expect(toDeg(compassReading(sample))).toBeCloseTo(90, 9);
  });

  it('turns the card by PLUS the heading, so the number reaches the index', () => {
    // The card carries Ost a quarter turn clockwise from Nord, so it must turn
    // a quarter turn ANTICLOCKWISE to bring Ost to the top.
    expect(compassCardRotation(90 * DEG)).toBeCloseTo(90 * DEG, 12);
    expect(compassCardRotation(0)).toBe(0);
  });

  it('reads north as zero and not as a full turn', () => {
    const sample = makeSample();
    expect(compassReading(sample)).toBe(0);
  });
});

// ---------------------------------------------------------------------------
// Wendehorizont, the gyro artificial horizon
// ---------------------------------------------------------------------------

describe('the Wendehorizont', () => {
  it('puts the horizon in the middle of the aperture in level flight', () => {
    expect(horizonOffset(0)).toBeCloseTo(0, 12);
  });

  it('drops the horizon below the middle with the nose up', () => {
    expect(horizonOffset(10 * DEG)).toBeCloseTo(-Math.sin(10 * DEG), 12);
    expect(horizonOffset(10 * DEG)).toBeLessThan(0);
    expect(horizonOffset(-10 * DEG)).toBeGreaterThan(0);
  });

  it('follows the sine law of a drum, so the ends crowd together', () => {
    const lowDegrees = Math.abs(horizonOffset(10 * DEG)) - Math.abs(horizonOffset(0));
    const highDegrees = Math.abs(horizonOffset(80 * DEG)) - Math.abs(horizonOffset(70 * DEG));
    expect(highDegrees).toBeLessThan(lowDegrees);
    expect(horizonOffset(90 * DEG)).toBeCloseTo(-1, 12);
  });

  it('erects to the true vertical when nothing accelerates', () => {
    expect(erectionTarget(0)).toBeCloseTo(0, 12);
  });

  it('tips toward a sustained acceleration, which is the gyro error', () => {
    // One meter per second squared is about a tenth of a g, and the vanes then
    // settle 5.8 degrees off the true vertical, which is inside the stop.
    const error = erectionTarget(1);
    expect(error).toBeGreaterThan(0);
    expect(error).toBeCloseTo(Math.atan2(1, 9.80665), 12);
    expect(toDeg(error)).toBeCloseTo(5.82, 2);
  });

  it('stops the erection error at its mechanical limit', () => {
    expect(erectionTarget(500)).toBe(HORIZON_ERECTION_LIMIT);
    expect(erectionTarget(-500)).toBe(-HORIZON_ERECTION_LIMIT);
    expect(toDeg(HORIZON_ERECTION_LIMIT)).toBeCloseTo(6, 9);
  });

  it('adds the erection error on top of the true attitude', () => {
    const sample = makeSample();
    sample.state.orientation.copy(
      new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), 8 * DEG),
    );
    expect(toDeg(horizonPitchTarget(sample, 0))).toBeCloseTo(8, 6);
    expect(toDeg(horizonPitchTarget(sample, 2 * DEG))).toBeCloseTo(10, 6);
    expect(toDeg(horizonBankTarget(sample, 0))).toBeCloseTo(0, 6);
  });

  it('answers quicker in bank than in pitch, and lags in both', () => {
    expect(HORIZON_BANK_LAG).toBeLessThan(HORIZON_PITCH_LAG);
    expect(HORIZON_BANK_LAG).toBeGreaterThan(0);
  });
});

// ---------------------------------------------------------------------------
// Drehzahlmesser, the rotor speed indicator
// ---------------------------------------------------------------------------

describe('the Drehzahlmesser', () => {
  it('turns rad/s into rpm exactly as src/math/units.ts does', () => {
    expect(tachometerReading(OMEGA_MAX)).toBe(radPerSecToRpm(OMEGA_MAX));
    expect(tachometerReading(OMEGA_MAX)).toBeCloseTo(MAX_RPM, 9);
    expect(tachometerReading(0)).toBe(0);
  });

  it('reads 0 to 12000 rpm on an even scale over 300 degrees', () => {
    expect(TACHOMETER_MAX_RPM).toBe(12000);
    expect(toDeg(tachometerAngle(0))).toBeCloseTo(-150, 9);
    expect(toDeg(tachometerAngle(6000))).toBeCloseTo(0, 9);
    expect(toDeg(tachometerAngle(12000))).toBeCloseTo(150, 9);
  });

  it('holds the needle on its stop past both ends', () => {
    expect(tachometerAngle(-500)).toBe(tachometerAngle(0));
    expect(tachometerAngle(20000)).toBe(tachometerAngle(12000));
  });

  it('puts the idle speed inside the marked danger band', () => {
    // The band runs from zero to 6000 rpm, and idle is 3000, so an idling
    // engine really does sit in the band a fast lever kills it in.
    expect(IDLE_RPM).toBeLessThan(DANGER_BAND_RPM);
    expect(tachometerAngle(IDLE_RPM)).toBeLessThan(tachometerAngle(DANGER_BAND_RPM));
    expect(tachometerAngle(IDLE_RPM)).toBeGreaterThan(tachometerAngle(0));
  });

  it('keeps the maximum rotor speed on the dial', () => {
    expect(MAX_RPM).toBeLessThan(TACHOMETER_MAX_RPM);
    expect(tachometerAngle(MAX_RPM)).toBeLessThan(tachometerAngle(TACHOMETER_MAX_RPM));
  });
});

// ---------------------------------------------------------------------------
// Abgastemperatur, the gas temperature indicator
// ---------------------------------------------------------------------------

describe('the Abgastemperatur', () => {
  it('turns kelvin into Celsius exactly as src/math/units.ts does', () => {
    expect(gasTemperatureReading(1100)).toBe(kelvinToCelsius(1100));
    expect(gasTemperatureReading(288.15)).toBeCloseTo(15, 9);
  });

  it('reads 0 to 1000 C on an even scale over 270 degrees', () => {
    expect(GAS_TEMPERATURE_MAX).toBe(1000);
    expect(toDeg(gasTemperatureAngle(0))).toBeCloseTo(-135, 9);
    expect(toDeg(gasTemperatureAngle(500))).toBeCloseTo(0, 9);
    expect(toDeg(gasTemperatureAngle(1000))).toBeCloseTo(135, 9);
  });

  it('holds the needle on its stop past both ends', () => {
    expect(gasTemperatureAngle(-200)).toBe(gasTemperatureAngle(0));
    expect(gasTemperatureAngle(1600)).toBe(gasTemperatureAngle(1000));
  });

  it('marks the limit the engine model really charges damage above', () => {
    expect(GAS_TEMPERATURE_LIMIT_C).toBe(kelvinToCelsius(TURBINE_INLET_TEMPERATURE_LIMIT));
    expect(GAS_TEMPERATURE_LIMIT_C).toBeCloseTo(826.85, 2);
    expect(GAS_TEMPERATURE_LIMIT_C).toBeLessThan(GAS_TEMPERATURE_MAX);
  });

  it('keeps a cold engine on the dial and not on the bottom stop', () => {
    const cold = gasTemperatureAngle(gasTemperatureReading(288.15));
    expect(cold).toBeGreaterThan(gasTemperatureAngle(0));
  });
});

// ---------------------------------------------------------------------------
// Kraftstoffvorrat, the fuel contents gauge
// ---------------------------------------------------------------------------

describe('the Kraftstoffvorrat', () => {
  it('reads 0 to 2200 kg on an even scale over 270 degrees', () => {
    expect(FUEL_SCALE_MAX).toBe(2200);
    expect(toDeg(fuelAngle(0))).toBeCloseTo(-135, 9);
    expect(toDeg(fuelAngle(1100))).toBeCloseTo(0, 9);
    expect(toDeg(fuelAngle(2200))).toBeCloseTo(135, 9);
  });

  it('keeps the full tanks of 2133 kg inside the dial', () => {
    expect(fuelAngle(2133)).toBeLessThan(fuelAngle(FUEL_SCALE_MAX));
    expect(fuelAngle(2133)).toBeGreaterThan(fuelAngle(2000));
  });

  it('puts the low fuel band at the bottom of the scale', () => {
    expect(fuelAngle(FUEL_LOW_MARK)).toBeLessThan(fuelAngle(FUEL_SCALE_MAX / 2));
  });

  it('holds the needle on its stop past both ends', () => {
    expect(fuelAngle(-10)).toBe(fuelAngle(0));
    expect(fuelAngle(9999)).toBe(fuelAngle(2200));
  });
});

// ---------------------------------------------------------------------------
// Borduhr, the clock
// ---------------------------------------------------------------------------

describe('the Borduhr', () => {
  it('turns the hour hand once every twelve hours', () => {
    expect(clockHourAngle(0)).toBeCloseTo(0, 12);
    expect(clockHourAngle(3 * 3600)).toBeCloseTo(Math.PI / 2, 12);
    expect(clockHourAngle(12 * 3600)).toBeCloseTo(0, 9);
  });

  it('turns the minute hand once every hour', () => {
    expect(clockMinuteAngle(15 * 60)).toBeCloseTo(Math.PI / 2, 12);
    expect(clockMinuteAngle(3600)).toBeCloseTo(0, 9);
  });

  it('steps the second hand in fifths, as a five beat movement does', () => {
    expect(clockSecondAngle(10.15)).toBe(clockSecondAngle(10.0));
    expect(clockSecondAngle(10.25)).not.toBe(clockSecondAngle(10.0));
    expect(clockSecondAngle(15)).toBeCloseTo(Math.PI / 2, 12);
  });
});

// ---------------------------------------------------------------------------
// AFN 2, the homing indicator
// ---------------------------------------------------------------------------

describe('the AFN 2', () => {
  it('centers the course pointer when the beacon is dead ahead', () => {
    const sample = makeSample();
    // The beacon sits at the world origin, so a station keeping aircraft to
    // the south of it on a north heading is pointing straight at it.
    sample.state.position.set(-5000, 0, -1000);
    expect(homingCourse(sample)).toBeCloseTo(0, 9);
  });

  it('tells the pilot to turn right when the beacon is to the right', () => {
    const sample = makeSample();
    sample.state.position.set(-5000, -5000, -1000);
    expect(homingCourse(sample)).toBeGreaterThan(0);
  });

  it('holds the course pointer on its stop past the scale', () => {
    const sample = makeSample();
    sample.state.position.set(5000, 0, -1000);
    expect(Math.abs(homingCourse(sample))).toBe(1);
  });

  it('shows a full signal near the beacon and none far from it', () => {
    const near = makeSample();
    near.state.position.set(-500, 0, -100);
    expect(homingStrength(near)).toBe(1);

    const far = makeSample();
    far.state.position.set(-40000, 0, -6000);
    expect(homingStrength(far)).toBe(0);

    const middle = makeSample();
    middle.state.position.set(-8000, 0, -3000);
    expect(homingStrength(middle)).toBeGreaterThan(0);
    expect(homingStrength(middle)).toBeLessThan(1);
  });
});

// ---------------------------------------------------------------------------
// Doppeldruckmesser, the fuel and oil pressure gauge
// ---------------------------------------------------------------------------

describe('the Doppeldruckmesser', () => {
  it('reads nothing at all with the engine stopped', () => {
    expect(fuelPressure(0)).toBe(0);
    expect(oilPressure(0)).toBe(0);
  });

  it('follows the fuel flow up to the maximum of the engine', () => {
    expect(fuelPressure(MAX_FUEL_FLOW)).toBeCloseTo(8.5, 9);
    expect(fuelPressure(MAX_FUEL_FLOW / 2)).toBeCloseTo(4.25, 9);
    expect(fuelPressure(MAX_FUEL_FLOW * 4)).toBe(fuelPressure(MAX_FUEL_FLOW));
  });

  it('follows the square root of the rotor speed, so it rises early', () => {
    expect(oilPressure(OMEGA_MAX)).toBeCloseTo(6.5, 9);
    expect(oilPressure(OMEGA_MAX / 4)).toBeCloseTo(3.25, 9);
    expect(oilPressure(OMEGA_MAX * 2)).toBe(oilPressure(OMEGA_MAX));
  });

  it('reads 0 to 10 at on an even scale over 270 degrees', () => {
    expect(PRESSURE_SCALE_MAX).toBe(10);
    expect(toDeg(pressureAngle(0))).toBeCloseTo(-135, 9);
    expect(toDeg(pressureAngle(5))).toBeCloseTo(0, 9);
    expect(toDeg(pressureAngle(10))).toBeCloseTo(135, 9);
    expect(pressureAngle(20)).toBe(pressureAngle(10));
  });
});

// ---------------------------------------------------------------------------
// The panel as a whole
// ---------------------------------------------------------------------------

describe('the panel', () => {
  it('fills all fifteen bezels of the virtual cockpit', () => {
    expect(GAUGE_NAMES.length).toBe(15);
    for (const name of [
      'airspeed',
      'artificialHorizon',
      'altimeter',
      'variometer',
      'turnSlip',
      'compass',
      'fuel',
      'clock',
      'homing',
      'rpmLeft',
      'rpmRight',
      'gasTemperatureLeft',
      'gasTemperatureRight',
      'enginePressureLeft',
      'enginePressureRight',
    ]) {
      expect(GAUGE_NAMES).toContain(name);
    }
  });

  it('names each bezel one time only', () => {
    expect(new Set(GAUGE_NAMES).size).toBe(GAUGE_NAMES.length);
  });
});
