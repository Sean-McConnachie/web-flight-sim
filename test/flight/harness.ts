/**
 * The flight test harness.
 *
 * It builds the whole Me 262, with NO renderer and no browser, trims it, flies a
 * defined maneuver with an autopilot, and measures what the aircraft did. Every
 * flight test in this directory uses it. Vitest runs the tests in Node through
 * `npm run test:flight`.
 *
 *
 * 1. WHY THERE IS AN AUTOPILOT AT ALL
 *
 * A performance number is only a number when the aircraft is STEADY. A rate of
 * climb read while the aircraft still trades speed for height is not a rate of
 * climb, it is a phase of a phugoid. Bead b25 measured a climb rate that hunted
 * between 13 and 48 m/s with a crude controller, and no part of that range is
 * the answer.
 *
 * The harness therefore holds the aircraft with a cascade of damped loops and
 * PROVES the state is steady before it records anything.
 *
 *   pitch      A rate command inner loop. The outer loop asks for a pitch rate,
 *              the inner loop drives the elevator with that rate error and with
 *              an integral term. The elevator starts at the value the trim
 *              solver reports, so the loop only has to hold the error, not find
 *              the trim.
 *   altitude   Altitude error, to a climb rate command, to a flight path angle
 *              command, to a pitch command. The pitch command carries the
 *              measured angle of attack through a slow filter, so the loop does
 *              not fight the angle of attack it needs.
 *   speed      Either the throttle holds the speed and the pitch holds the
 *              altitude, or the throttle is fixed and the PITCH holds the speed.
 *              The second mode is the climb: a climb at a full throttle is the
 *              speed the pilot holds, and the climb angle is the answer.
 *   lateral    The aileron holds the wings level from the bank angle and the
 *              roll rate. The rudder holds the sideslip at zero. Both stay near
 *              zero in a symmetric test, and both do the work in the engine out
 *              test.
 *
 *
 * 2. HOW THE HARNESS KNOWS THE STATE IS STEADY
 *
 * `flyUntilSteady` samples the state every SAMPLE_INTERVAL and keeps a window of
 * the last `window` seconds. It fits a straight line to each signal over that
 * window by least squares and it also measures the peak to peak spread. The
 * state counts as steady when the SLOPE of the speed, the slope of the climb
 * rate and the SPREAD of the climb rate are all inside the criteria at the same
 * time. The value it reports is the MEAN over that window, so a small remaining
 * oscillation averages out instead of landing in the answer.
 *
 * A measurement whose window never meets the criteria comes back with
 * `steady: false`. The test then fails on that flag, not on the number, because
 * a number taken during a transient is worthless.
 *
 *
 * 3. THE REPORT TABLE
 *
 * Every test records its result with `record`. `printReport` writes one table of
 * measurement, target, tolerance, result and delta. Bead b33 runs
 * `npm run test:flight` one time and reads the whole picture.
 *
 * The table goes to `process.stdout` and not to `console.log`, because the
 * default Vitest reporter swallows the console and passes the raw stream
 * through.
 *
 *
 * 4. THE SEPARATION RULE
 *
 * CONVENTIONS section 4. Nothing here imports src/render, src/ui or src/input,
 * and nothing touches a browser API. That is the whole reason the flight model
 * is separate from the renderer, and this file is the thing that reason exists
 * for.
 */

import { Quaternion, Vector3 } from 'three';

import type { Aircraft, AircraftInput } from '@/aircraft/aircraft';
import { createAircraft } from '@/aircraft/aircraft';
import { FUEL_CAPACITY } from '@/aircraft/me262/mass';
import { CONTROL_INDEX, FLAP_LANDING_ANGLE } from '@/aircraft/me262/geometry';
import type { FlapSetting } from '@/aircraft/me262/systems';
import { flapSettingPosition } from '@/aircraft/me262/systems';
import { PHYSICS_DT } from '@/core/loop';
import { ME262_STATIC_CG_HEIGHT } from '@/physics/gear';
import { clamp } from '@/math/tables';
import { msToKmh, toDeg } from '@/math/units';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/** The step of every flight test. It is the step the real loop uses. */
export const DT = PHYSICS_DT;

/** Time between two telemetry samples, s. */
const SAMPLE_INTERVAL = 0.05;

/**
 * Altitude under which a run counts as a crash, m.
 *
 * The ground plane of src/physics/contact.ts sits at zero altitude and the
 * airframe contact points of src/physics/gear.ts reach about a meter below the
 * center of gravity, so an aircraft below this height is already scraping.
 */
const GROUND_CLEARANCE = 5; // m

/** Longest engine start, s. The Jumo 004 needs about 80 s from cold to idle. */
const START_TIMEOUT = 200;

// --- Autopilot gains. Every one is tuned against this airframe. -------------

/**
 * Reference dynamic pressure of the pitch gains, Pa.
 *
 * The elevator makes a moment that follows the dynamic pressure, so one fixed
 * gain gives a loop that is slow at 400 km/h and unstable at 900 km/h. At sea
 * level the dynamic pressure runs from 8 kPa at the stall to 38 kPa at the
 * maximum level speed, which is a factor of five in the loop gain. The two pitch
 * gains below therefore carry a factor of REFERENCE_PRESSURE over the dynamic
 * pressure of the moment, so the loop keeps the same bandwidth everywhere the
 * aircraft flies. The bound stops the scale from running away at a very low
 * dynamic pressure.
 */
const REFERENCE_PRESSURE = 10000; // Pa, about 127 m/s at sea level
const GAIN_SCALE_MIN = 0.15;
const GAIN_SCALE_MAX = 4;

/** Pitch attitude error to a pitch rate command, 1/s. */
const PITCH_TO_RATE = 0.9;
/** Largest pitch rate the outer loop asks for, rad/s. */
const PITCH_RATE_LIMIT = 0.12;
/** Pitch rate error to elevator command, s. */
const RATE_TO_ELEVATOR = 4.0;
/** Integral of the pitch rate error to elevator command. It removes the bias. */
const RATE_INTEGRAL = 2.5;
/** Bound of the pitch integral, in elevator command units. */
const PITCH_INTEGRAL_LIMIT = 0.8;

/** Altitude error to a climb rate command, 1/s, and the bound of that command. */
const ALTITUDE_TO_CLIMB = 0.12;
const CLIMB_COMMAND_LIMIT = 6; // m/s
/** Climb rate error to a flight path angle command, s/m. */
const CLIMB_TO_GAMMA = 0.02;
/** Bound of the flight path angle command of the altitude hold, rad. */
const GAMMA_COMMAND_LIMIT = 0.25;
/** Time constant of the angle of attack filter the pitch command carries, s. */
const ALPHA_FILTER_TIME = 2.0;

/**
 * Largest rate the throttle lever may move at, per second.
 *
 * THE HARNESS MUST FLY THE LEVER LIKE A PILOT. A slam from idle to full power
 * drives the fuel-air ratio of a Jumo 004 past its surge line, because the
 * airflow of a 3000 rpm rotor is a third of the airflow at full power. The
 * engine model of src/aircraft/me262/engine.ts holds that fault: the compressor
 * stalls, the engine bangs, and two seconds later it flames out with permanent
 * turbine damage. A harness that slams the lever measures a dead aircraft.
 *
 * The published spool from idle to full power is 8 to 10 s, so a lever that
 * takes 10 s to cross its travel asks for nothing the engine cannot follow.
 * Source: CONVENTIONS section 8, idle to full power. Confidence: firm.
 */
const THROTTLE_RATE = 0.1; // per second

/**
 * Surge margin at which the lever stops advancing.
 *
 * The rate limit alone is enough at any rotor speed above the danger band. This
 * guard is the second line: while the margin of either engine falls under the
 * value below, the lever holds where it is and waits for the rotor.
 */
const SURGE_GUARD = 0.15;

/** Speed error to throttle, and its integral. */
const SPEED_TO_THROTTLE = 0.05; // per m/s
const SPEED_THROTTLE_INTEGRAL = 0.01; // per m/s per s
const THROTTLE_INTEGRAL_LIMIT = 0.6;

/** Speed error to a pitch command, rad per m/s, and its integral. */
const SPEED_TO_PITCH = 0.006;
const SPEED_PITCH_INTEGRAL = 0.0015;
const SPEED_PITCH_INTEGRAL_LIMIT = 0.25; // rad
/** Bound of the pitch command of the speed hold, rad. */
const SPEED_PITCH_LIMIT = 0.7; // rad

/** Bank angle and roll rate to aileron. */
const ROLL_TO_AILERON = 1.6;
const ROLL_RATE_TO_AILERON = 0.9;

/**
 * Sideslip and yaw rate to rudder.
 *
 * THE SIGN OF THE FIRST ONE IS THE ONE TO READ TWICE. A positive sideslip means
 * the airspeed vector points to the right of the nose, so the nose has to swing
 * RIGHT to catch it, and a positive rudder command yaws the nose right. The
 * rudder therefore follows the sideslip with a PLUS. A minus fights the
 * weathercock stability of the fin, and this aircraft has so little of it that
 * the loop then diverges from nothing but rounding noise, with a time constant
 * near 0.7 s. Bead b49 is open on that weak directional stability, and this gain
 * is where a test first meets it.
 */
const SIDESLIP_TO_RUDDER = 3.0;
const YAW_RATE_TO_RUDDER = 2.0;

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

/** One sample of the state. Every value is in SI units. */
export interface Sample {
  time: number;
  altitude: number;
  /** True airspeed, m/s. */
  speed: number;
  /** Equivalent airspeed, m/s. */
  equivalentAirspeed: number;
  mach: number;
  /** Positive up, m/s. */
  climbRate: number;
  alpha: number;
  beta: number;
  roll: number;
  pitch: number;
  yaw: number;
  /** Body roll, pitch and yaw rates, rad/s. */
  p: number;
  q: number;
  r: number;
  loadFactor: number;
  elevator: number;
  aileron: number;
  rudder: number;
  throttle: number;
  /** Ground distance from the origin along the runway, m. */
  distance: number;
  onGround: boolean;
}

function createSample(): Sample {
  return {
    time: 0,
    altitude: 0,
    speed: 0,
    equivalentAirspeed: 0,
    mach: 0,
    climbRate: 0,
    alpha: 0,
    beta: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
    p: 0,
    q: 0,
    r: 0,
    loadFactor: 1,
    elevator: 0,
    aileron: 0,
    rudder: 0,
    throttle: 0,
    distance: 0,
    onGround: false,
  };
}

/** Euler angles of an orientation, in the NED frame of CONVENTIONS section 3.2. */
const bodyX = new Vector3();
const bodyY = new Vector3();
const bodyZ = new Vector3();

export function attitude(q: Quaternion): { roll: number; pitch: number; yaw: number } {
  bodyX.set(1, 0, 0).applyQuaternion(q);
  bodyY.set(0, 1, 0).applyQuaternion(q);
  bodyZ.set(0, 0, 1).applyQuaternion(q);
  return {
    pitch: Math.asin(clamp(-bodyX.z, -1, 1)),
    yaw: Math.atan2(bodyX.y, bodyX.x),
    roll: Math.atan2(bodyY.z, bodyZ.z),
  };
}

// ---------------------------------------------------------------------------
// The autopilot
// ---------------------------------------------------------------------------

/**
 * What the autopilot holds. A field that is null leaves that channel alone, and
 * the fixed command below it goes through instead.
 */
export interface AutopilotCommand {
  /** Altitude to hold, m. The pitch channel flies it. */
  altitude: number | null;
  /** Speed to hold with the THROTTLE, m/s. */
  speed: number | null;
  /** Speed to hold with the PITCH, m/s. The throttle then stays fixed. */
  climbSpeed: number | null;
  /** Pitch attitude to hold, rad. It wins over the altitude and the climb speed. */
  pitch: number | null;
  /** Throttle to use when no speed hold owns the channel, 0 to 1. */
  throttle: number;
  /** True while the aileron holds the wings level. */
  wingsLevel: boolean;
  /** True while the rudder holds the sideslip at zero. */
  holdSideslip: boolean;
  /** Fixed aileron command, used when wingsLevel is false. */
  aileron: number;
  /** Fixed rudder command, used when holdSideslip is false. */
  rudder: number;
  /** Feed forward elevator, from the trim solver. */
  trimElevator: number;
  /** Feed forward throttle, from the trim solver. */
  trimThrottle: number;
  /** Reference pitch of the speed hold, rad. */
  referencePitch: number;
}

export function createCommand(): AutopilotCommand {
  return {
    altitude: null,
    speed: null,
    climbSpeed: null,
    pitch: null,
    throttle: 0,
    wingsLevel: true,
    holdSideslip: true,
    aileron: 0,
    rudder: 0,
    trimElevator: 0,
    trimThrottle: 0,
    referencePitch: 0,
  };
}

interface AutopilotState {
  pitchIntegral: number;
  throttleIntegral: number;
  speedPitchIntegral: number;
  alphaFilter: number;
  started: boolean;
}

// ---------------------------------------------------------------------------
// The test aircraft
// ---------------------------------------------------------------------------

export interface FlightTest {
  readonly aircraft: Aircraft;
  readonly input: AircraftInput;
  readonly command: AutopilotCommand;
  /** Simulated time since the aircraft was built, s. */
  time: number;
  /** Reads the state into a sample and returns it. The sample is reused. */
  sample(): Sample;
  /** Runs one physics step with the autopilot. */
  step(): void;
  /** Runs for a number of seconds with the autopilot. */
  fly(seconds: number): void;
  /** Runs for a number of seconds with NO autopilot. The input passes through. */
  flyOpenLoop(seconds: number): void;
  /** Resets the integrators of the autopilot. */
  resetAutopilot(): void;
}

function neutralInput(): AircraftInput {
  return {
    roll: 0,
    pitch: 0,
    yaw: 0,
    throttle: 0,
    brakeLeft: 0,
    brakeRight: 0,
    toggleGear: false,
    toggleFlapsUp: false,
    toggleFlapsDown: false,
    startEngines: false,
  };
}

/**
 * Builds one aircraft and starts both engines on the ground.
 *
 * The start follows the procedure of the pilot notes, which the aircraft model
 * already implements: crank with the starter, let the fuel cock open with the
 * rotor, and wait for a settled idle. It takes about 80 s of simulated time.
 */
export function createFlightTest(): FlightTest {
  const aircraft = createAircraft();
  const input = neutralInput();
  const command = createCommand();
  const current = createSample();
  const state: AutopilotState = {
    pitchIntegral: 0,
    throttleIntegral: 0,
    speedPitchIntegral: 0,
    alphaFilter: 0,
    started: false,
  };
  let time = 0;

  function readSample(): Sample {
    const body = aircraft.state.body;
    const angles = attitude(body.orientation);
    current.time = time;
    current.altitude = -body.position.z;
    current.speed = aircraft.state.totals.trueAirspeed;
    current.equivalentAirspeed = aircraft.equivalentAirspeed;
    current.mach = aircraft.state.totals.mach;
    current.climbRate = -body.velocity.z;
    current.alpha = aircraft.state.totals.alpha;
    current.beta = aircraft.state.totals.beta;
    current.roll = angles.roll;
    current.pitch = angles.pitch;
    current.yaw = angles.yaw;
    current.p = body.angularVelocity.x;
    current.q = body.angularVelocity.y;
    current.r = body.angularVelocity.z;
    current.loadFactor = aircraft.state.loadFactor;
    current.elevator = input.pitch;
    current.aileron = input.roll;
    current.rudder = input.yaw;
    current.throttle = input.throttle;
    current.distance = body.position.x;
    current.onGround = aircraft.state.onGround;
    return current;
  }

  /** Writes the four pilot commands for one step. */
  function autopilot(dt: number): void {
    const s = readSample();
    const speed = Math.max(s.speed, 1);

    // The angle of attack the pitch command carries. The filter is slow, so the
    // outer loop does not chase the short period of the aircraft.
    if (!state.started) {
      state.alphaFilter = s.alpha;
      state.started = true;
    }
    state.alphaFilter += (s.alpha - state.alphaFilter) * clamp(dt / ALPHA_FILTER_TIME, 0, 1);

    // --- The pitch channel ------------------------------------------------
    let pitchCommand: number;
    if (command.pitch !== null) {
      pitchCommand = command.pitch;
    } else if (command.climbSpeed !== null) {
      // The pitch holds the SPEED. A nose that is too low reads a speed that is
      // too high, so the error raises the nose.
      const error = speed - command.climbSpeed;
      state.speedPitchIntegral = clamp(
        state.speedPitchIntegral + SPEED_PITCH_INTEGRAL * error * dt,
        -SPEED_PITCH_INTEGRAL_LIMIT,
        SPEED_PITCH_INTEGRAL_LIMIT,
      );
      pitchCommand = clamp(
        command.referencePitch + SPEED_TO_PITCH * error + state.speedPitchIntegral,
        -SPEED_PITCH_LIMIT,
        SPEED_PITCH_LIMIT,
      );
    } else if (command.altitude !== null) {
      const climbCommand = clamp(
        ALTITUDE_TO_CLIMB * (command.altitude - s.altitude),
        -CLIMB_COMMAND_LIMIT,
        CLIMB_COMMAND_LIMIT,
      );
      // THE MEASURED FLIGHT PATH ANGLE MUST NOT ENTER THIS COMMAND. A command
      // that carries the angle it measures follows the aircraft instead of
      // holding it: the nose rises, the path follows, the command rises with it,
      // and the loop has no restoring term at all. That loop diverges with a
      // time constant near 0.6 s from nothing but rounding noise, and it takes
      // 90 s to grow out of the last digit of a double. The two terms below are
      // a feed forward from the COMMANDED climb rate and a damping term on the
      // climb rate error, so the equilibrium of a level hold is a pitch equal to
      // the angle of attack, which is level flight.
      const gammaCommand = clamp(
        climbCommand / speed + CLIMB_TO_GAMMA * (climbCommand - s.climbRate),
        -GAMMA_COMMAND_LIMIT,
        GAMMA_COMMAND_LIMIT,
      );
      pitchCommand = state.alphaFilter + gammaCommand;
    } else {
      pitchCommand = s.pitch;
    }

    const rateCommand = clamp(
      PITCH_TO_RATE * (pitchCommand - s.pitch),
      -PITCH_RATE_LIMIT,
      PITCH_RATE_LIMIT,
    );
    const rateError = rateCommand - s.q;
    // The gain schedule. See REFERENCE_PRESSURE.
    const gain = clamp(
      REFERENCE_PRESSURE / Math.max(aircraft.state.totals.dynamicPressure, 1),
      GAIN_SCALE_MIN,
      GAIN_SCALE_MAX,
    );
    const raw =
      command.trimElevator +
      gain * RATE_TO_ELEVATOR * rateError +
      gain * RATE_INTEGRAL * state.pitchIntegral;
    const elevator = clamp(raw, -1, 1);
    // Anti windup. The integral only grows while the elevator is off its stop,
    // or while the error pushes it back toward the range.
    if (Math.abs(raw) < 1 || raw * rateError < 0) {
      state.pitchIntegral = clamp(
        state.pitchIntegral + rateError * dt,
        -PITCH_INTEGRAL_LIMIT,
        PITCH_INTEGRAL_LIMIT,
      );
    }
    input.pitch = elevator;

    // --- The throttle channel ---------------------------------------------
    let wantedThrottle: number;
    if (command.speed !== null) {
      const error = command.speed - speed;
      const rawThrottle =
        command.trimThrottle + SPEED_TO_THROTTLE * error + state.throttleIntegral;
      if (rawThrottle > 0 && rawThrottle < 1) {
        state.throttleIntegral = clamp(
          state.throttleIntegral + SPEED_THROTTLE_INTEGRAL * error * dt,
          -THROTTLE_INTEGRAL_LIMIT,
          THROTTLE_INTEGRAL_LIMIT,
        );
      }
      wantedThrottle = clamp(rawThrottle, 0, 1);
    } else {
      wantedThrottle = clamp(command.throttle, 0, 1);
    }
    // The lever rate. See THROTTLE_RATE and SURGE_GUARD.
    let thin = false;
    for (const engine of aircraft.state.engines) {
      if (engine.surgeMargin < SURGE_GUARD) {
        thin = true;
      }
    }
    const travel = THROTTLE_RATE * dt;
    const lever = clamp(
      wantedThrottle,
      input.throttle - travel,
      thin ? input.throttle : input.throttle + travel,
    );
    input.throttle = clamp(lever, 0, 1);

    // --- The lateral channels ---------------------------------------------
    // The lateral gains carry the same schedule as the pitch gains, for the same
    // reason: the aileron and the rudder both make a moment that follows the
    // dynamic pressure.
    if (command.wingsLevel) {
      input.roll = clamp(gain * (-ROLL_TO_AILERON * s.roll - ROLL_RATE_TO_AILERON * s.p), -1, 1);
    } else {
      input.roll = clamp(command.aileron, -1, 1);
    }
    if (command.holdSideslip) {
      input.yaw = clamp(gain * (SIDESLIP_TO_RUDDER * s.beta - YAW_RATE_TO_RUDDER * s.r), -1, 1);
    } else {
      input.yaw = clamp(command.rudder, -1, 1);
    }
  }

  const api: FlightTest = {
    aircraft,
    input,
    command,
    get time(): number {
      return time;
    },
    set time(value: number) {
      time = value;
    },
    sample: readSample,
    step(): void {
      autopilot(DT);
      aircraft.fixedUpdate(input, DT);
      time += DT;
    },
    fly(seconds: number): void {
      const steps = Math.round(seconds / DT);
      for (let i = 0; i < steps; i++) {
        api.step();
      }
    },
    flyOpenLoop(seconds: number): void {
      const steps = Math.round(seconds / DT);
      for (let i = 0; i < steps; i++) {
        aircraft.fixedUpdate(input, DT);
        time += DT;
      }
    },
    resetAutopilot(): void {
      state.pitchIntegral = 0;
      state.throttleIntegral = 0;
      state.speedPitchIntegral = 0;
      state.started = false;
    },
  };

  // The start. The brakes hold the aircraft while the engines light.
  input.startEngines = true;
  input.brakeLeft = 1;
  input.brakeRight = 1;
  const steps = Math.round(START_TIMEOUT / DT);
  for (let i = 0; i < steps; i++) {
    if (aircraft.state.engines.every((e) => e.state === 'idle' || e.state === 'running')) {
      break;
    }
    aircraft.fixedUpdate(input, DT);
    time += DT;
  }
  input.startEngines = false;
  api.flyOpenLoop(3);
  input.brakeLeft = 0;
  input.brakeRight = 0;
  return api;
}

// ---------------------------------------------------------------------------
// Placing the aircraft
// ---------------------------------------------------------------------------

export interface AirStart {
  altitude: number;
  /** True airspeed, m/s. */
  speed: number;
  /** Pitch attitude, rad. Use the value the trim solver reports. */
  pitch: number;
  /** Flight path angle, rad. It sets the direction of the velocity. */
  flightPathAngle?: number;
  flapSetting: FlapSetting;
  gearDown: boolean;
  /** Fuel on board, kg. It defaults to a full load. */
  fuelMass?: number;
}

const startOrientation = new Quaternion();
const pitchAxis = new Vector3(0, 1, 0);

/**
 * Puts the aircraft in the air at a state, with the flap and the gear already
 * at the position the setup asks for.
 *
 * The flap and the gear both move at a rate in the real machine, so the harness
 * writes the POSITION as well as the command. Without the position the aircraft
 * would fly the first eight seconds of every test with the flap still traveling.
 */
export function placeInAir(test: FlightTest, setup: AirStart): void {
  const body = test.aircraft.state.body;
  const gamma = setup.flightPathAngle !== undefined ? setup.flightPathAngle : 0;
  body.position.set(0, 0, -setup.altitude);
  body.velocity.set(setup.speed * Math.cos(gamma), 0, -setup.speed * Math.sin(gamma));
  startOrientation.setFromAxisAngle(pitchAxis, setup.pitch);
  body.orientation.copy(startOrientation);
  body.angularVelocity.set(0, 0, 0);

  const systems = test.aircraft.state.systems;
  systems.commandFlaps(setup.flapSetting);
  systems.state.flapPosition = flapSettingPosition(setup.flapSetting);
  systems.commandGear(setup.gearDown);
  systems.state.gearPosition = setup.gearDown ? 1 : 0;
  systems.state.fuelMass = setup.fuelMass !== undefined ? setup.fuelMass : FUEL_CAPACITY;
  test.aircraft.controls[CONTROL_INDEX.flap] = systems.state.flapPosition * FLAP_LANDING_ANGLE;
  test.resetAutopilot();
}

/**
 * Puts the aircraft on the runway threshold at rest, with the engines still
 * running, and sets the flap.
 */
export function placeOnRunway(test: FlightTest, flapSetting: FlapSetting): void {
  const body = test.aircraft.state.body;
  body.position.set(0, 0, -ME262_STATIC_CG_HEIGHT);
  body.velocity.set(0, 0, 0);
  body.orientation.set(0, 0, 0, 1);
  body.angularVelocity.set(0, 0, 0);
  const systems = test.aircraft.state.systems;
  systems.commandFlaps(flapSetting);
  systems.state.flapPosition = flapSettingPosition(flapSetting);
  systems.commandGear(true);
  systems.state.gearPosition = 1;
  test.aircraft.controls[CONTROL_INDEX.flap] = systems.state.flapPosition * FLAP_LANDING_ANGLE;
  test.resetAutopilot();
}

// ---------------------------------------------------------------------------
// Steady state detection
// ---------------------------------------------------------------------------

export interface SteadyCriteria {
  /** Length of the window the fit runs over, s. */
  window: number;
  /** Largest acceptable slope of the speed, m/s2. */
  speedSlope: number;
  /** Largest acceptable slope of the climb rate, m/s2. */
  climbSlope: number;
  /** Largest acceptable peak to peak spread of the climb rate, m/s. */
  climbSpread: number;
  /** Largest acceptable peak to peak spread of the angle of attack, rad. */
  alphaSpread: number;
  /**
   * Largest acceptable peak to peak spread of the throttle lever.
   *
   * The lever moves at THROTTLE_RATE, so it needs ten seconds to cross its
   * travel. Without this test a run can look steady while the engine is still
   * spooling: the speed slope passes through zero on the way, the window fits a
   * flat line, and the harness records a speed the aircraft was only passing
   * through. That fault cost 40 km/h on the first maximum speed run.
   */
  throttleSpread: number;
  /** Shortest run before a window may count as steady, s. */
  minSeconds: number;
}

export function steadyCriteria(overrides?: Partial<SteadyCriteria>): SteadyCriteria {
  return {
    window: 10,
    speedSlope: 0.02,
    climbSlope: 0.05,
    climbSpread: 0.5,
    alphaSpread: 0.004,
    throttleSpread: 0.005,
    minSeconds: 15,
    ...overrides,
  };
}

export interface SteadyResult {
  steady: boolean;
  /** True when the run ended on the ground. A crash is never a measurement. */
  crashed: boolean;
  /** Simulated seconds the wait took. */
  seconds: number;
  /** Mean of every signal over the window. */
  mean: Sample;
  speedSlope: number;
  climbSlope: number;
  climbSpread: number;
  alphaSpread: number;
  throttleSpread: number;
}

/** Least squares slope of one signal against time. */
function slope(times: readonly number[], values: readonly number[]): number {
  const n = times.length;
  if (n < 3) {
    return Number.POSITIVE_INFINITY;
  }
  let meanTime = 0;
  let meanValue = 0;
  for (let i = 0; i < n; i++) {
    meanTime += times[i];
    meanValue += values[i];
  }
  meanTime /= n;
  meanValue /= n;
  let top = 0;
  let bottom = 0;
  for (let i = 0; i < n; i++) {
    const dt = times[i] - meanTime;
    top += dt * (values[i] - meanValue);
    bottom += dt * dt;
  }
  return bottom > 0 ? top / bottom : Number.POSITIVE_INFINITY;
}

function spread(values: readonly number[]): number {
  let lo = Number.POSITIVE_INFINITY;
  let hi = Number.NEGATIVE_INFINITY;
  for (const value of values) {
    lo = Math.min(lo, value);
    hi = Math.max(hi, value);
  }
  return hi - lo;
}

function mean(values: readonly number[]): number {
  let sum = 0;
  for (const value of values) {
    sum += value;
  }
  return values.length > 0 ? sum / values.length : 0;
}

/**
 * Flies with the autopilot until the state is steady, or until `maxSeconds` runs
 * out. It reports the window mean of every signal and the three fit values that
 * decided the answer.
 */
export function flyUntilSteady(
  test: FlightTest,
  maxSeconds: number,
  criteria: SteadyCriteria = steadyCriteria(),
  onSample?: (s: Sample) => void,
): SteadyResult {
  const count = Math.max(4, Math.round(criteria.window / SAMPLE_INTERVAL));
  const keys: (keyof Sample)[] = [
    'time',
    'altitude',
    'speed',
    'equivalentAirspeed',
    'mach',
    'climbRate',
    'alpha',
    'beta',
    'roll',
    'pitch',
    'yaw',
    'p',
    'q',
    'r',
    'loadFactor',
    'elevator',
    'aileron',
    'rudder',
    'throttle',
    'distance',
  ];
  const history = new Map<string, number[]>();
  for (const key of keys) {
    history.set(key, []);
  }

  const stepsPerSample = Math.max(1, Math.round(SAMPLE_INTERVAL / DT));
  const totalSteps = Math.round(maxSeconds / DT);
  const start = test.time;
  let steady = false;
  let crashed = false;
  let speedSlopeValue = Number.POSITIVE_INFINITY;
  let climbSlopeValue = Number.POSITIVE_INFINITY;
  let climbSpreadValue = Number.POSITIVE_INFINITY;
  let alphaSpreadValue = Number.POSITIVE_INFINITY;
  let throttleSpreadValue = Number.POSITIVE_INFINITY;

  for (let i = 0; i < totalSteps; i++) {
    test.step();
    if (i % stepsPerSample !== stepsPerSample - 1) {
      continue;
    }
    const s = test.sample();
    // A CRASH IS NEVER A MEASUREMENT. An aircraft that reaches the ground stops
    // moving, and every slope and every spread then reads zero, so the fit would
    // call the wreck the steadiest state of the whole run.
    if (s.onGround || s.altitude < GROUND_CLEARANCE) {
      crashed = true;
      break;
    }
    if (onSample !== undefined) {
      onSample(s);
    }
    for (const key of keys) {
      const list = history.get(key) as number[];
      const value = s[key];
      list.push(typeof value === 'number' ? value : 0);
      if (list.length > count) {
        list.shift();
      }
    }
    const times = history.get('time') as number[];
    if (times.length < count) {
      continue;
    }
    speedSlopeValue = slope(times, history.get('speed') as number[]);
    climbSlopeValue = slope(times, history.get('climbRate') as number[]);
    climbSpreadValue = spread(history.get('climbRate') as number[]);
    alphaSpreadValue = spread(history.get('alpha') as number[]);
    throttleSpreadValue = spread(history.get('throttle') as number[]);
    if (
      test.time - start >= criteria.minSeconds &&
      Math.abs(speedSlopeValue) <= criteria.speedSlope &&
      Math.abs(climbSlopeValue) <= criteria.climbSlope &&
      climbSpreadValue <= criteria.climbSpread &&
      alphaSpreadValue <= criteria.alphaSpread &&
      throttleSpreadValue <= criteria.throttleSpread
    ) {
      steady = true;
      break;
    }
  }

  const average = createSample();
  for (const key of keys) {
    const list = history.get(key) as number[];
    if (list.length > 0) {
      // The sample record holds numbers and one boolean. Only the numbers are
      // averaged, and onGround comes from the last state.
      (average as unknown as Record<string, number>)[key] = mean(list);
    }
  }
  average.onGround = test.sample().onGround;

  return {
    steady: steady && !crashed,
    crashed,
    seconds: test.time - start,
    mean: average,
    speedSlope: speedSlopeValue,
    climbSlope: climbSlopeValue,
    climbSpread: climbSpreadValue,
    alphaSpread: alphaSpreadValue,
    throttleSpread: throttleSpreadValue,
  };
}

// ---------------------------------------------------------------------------
// The report table
// ---------------------------------------------------------------------------

export type ToleranceKind = 'fraction' | 'absolute';

export interface Measurement {
  /** What was measured, in words. */
  name: string;
  measured: number;
  /** The published value from CONVENTIONS section 8. */
  target: number;
  /** Half width of the acceptance band. */
  tolerance: number;
  toleranceKind: ToleranceKind;
  unit: string;
  /** Anything the reader needs to judge the number. */
  note?: string;
}

const results: Measurement[] = [];

/** Records one measurement and returns it, so a test can assert on it. */
export function record(m: Measurement): Measurement {
  results.push(m);
  return m;
}

/** True while the measurement sits inside its tolerance band. */
export function passed(m: Measurement): boolean {
  const band = m.toleranceKind === 'fraction' ? Math.abs(m.target) * m.tolerance : m.tolerance;
  return Math.abs(m.measured - m.target) <= band + 1e-12;
}

/** The signed distance from the target, as a fraction of the target. */
export function relativeError(m: Measurement): number {
  return m.target !== 0 ? (m.measured - m.target) / m.target : 0;
}

function pad(text: string, width: number): string {
  return text.length >= width ? text : text + ' '.repeat(width - text.length);
}

function padLeft(text: string, width: number): string {
  return text.length >= width ? text : ' '.repeat(width - text.length) + text;
}

function format(value: number): string {
  const size = Math.abs(value);
  if (size >= 1000) {
    return value.toFixed(0);
  }
  if (size >= 10) {
    return value.toFixed(1);
  }
  if (size >= 1) {
    return value.toFixed(2);
  }
  return value.toFixed(3);
}

/**
 * Writes the table of every measurement this test file recorded.
 *
 * The table goes to the raw output stream, because the default Vitest reporter
 * hides console output and passes the stream through.
 */
export function printReport(title: string): void {
  if (results.length === 0) {
    return;
  }
  const lines: string[] = [];
  lines.push('');
  lines.push(`=== ${title} =======================================================`);
  lines.push(
    `${pad('measurement', 38)}${padLeft('measured', 11)}${padLeft('target', 11)}` +
      `${padLeft('band', 10)}  ${pad('unit', 6)}${padLeft('delta', 10)}  result`,
  );
  for (const m of results) {
    const band =
      m.toleranceKind === 'fraction'
        ? `+-${(m.tolerance * 100).toFixed(0)}%`
        : `+-${format(m.tolerance)}`;
    const delta = m.measured - m.target;
    const percent = m.target !== 0 ? ` (${(relativeError(m) * 100).toFixed(1)}%)` : '';
    lines.push(
      `${pad(m.name, 38)}${padLeft(format(m.measured), 11)}${padLeft(format(m.target), 11)}` +
        `${padLeft(band, 10)}  ${pad(m.unit, 6)}${padLeft(format(delta), 10)}${percent}  ` +
        `${passed(m) ? 'PASS' : 'FAIL'}`,
    );
    if (m.note !== undefined) {
      lines.push(`${pad('', 38)}note: ${m.note}`);
    }
  }
  const failures = results.filter((m) => !passed(m)).length;
  lines.push(`${results.length} measurements, ${results.length - failures} pass, ${failures} fail`);
  lines.push('');
  process.stdout.write(`${lines.join('\n')}\n`);
  results.length = 0;
}

/** Writes one line of diagnostic text. Tests use it for values with no target. */
export function note(text: string): void {
  process.stdout.write(`    ${text}\n`);
}

/** A short line of the state, for a diagnostic trace. */
export function describeSample(s: Sample): string {
  return (
    `t=${s.time.toFixed(1)} h=${s.altitude.toFixed(0)} V=${msToKmh(s.speed).toFixed(1)} km/h ` +
    `M=${s.mach.toFixed(3)} roc=${s.climbRate.toFixed(2)} a=${toDeg(s.alpha).toFixed(2)} ` +
    `th=${toDeg(s.pitch).toFixed(2)} e=${s.elevator.toFixed(3)} thr=${s.throttle.toFixed(2)} ` +
    `n=${s.loadFactor.toFixed(2)}`
  );
}
