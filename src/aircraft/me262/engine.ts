/**
 * Junkers Jumo 004 B-1 turbojet.
 *
 * The engine holds one state variable that matters: the rotor speed. The model
 * integrates the rotor with a net torque balance and a real polar moment of
 * inertia:
 *
 *   J * d(omega)/dt = Q_turbine - Q_compressor - Q_friction + Q_starter
 *
 * The model does not use a first order lag on the rotor speed. The slow spool of
 * this engine is not a time constant. It is a fact of the gas path. At a low
 * rotor speed the compressor pressure ratio is near one, so the turbine has
 * almost no pressure drop to work with. The turbine then makes little more
 * torque than the compressor absorbs, and almost nothing is left to accelerate
 * the rotor. The same mechanism gives all three behaviors that the pilot feels:
 *
 *   1. Eight to ten seconds from idle to full power.
 *   2. A fast response above 6000 rpm, where the pressure ratio is high.
 *   3. A thin surge margin below 6000 rpm, where the airflow is small and any
 *      extra fuel drives the fuel-air ratio past the surge line.
 *
 * The gas path runs in this order on each step:
 *
 *   inlet -> compressor -> combustor -> turbine -> rotor -> surge -> thrust
 *
 * Thrust does not come from the gas path. It comes from a two dimensional table
 * against Mach number and pressure altitude, scaled by a strong function of the
 * rotor speed fraction. Bead b33 tunes the table anchors against the level speed
 * targets, so every anchor is a named exported constant.
 *
 * Frames follow CONVENTIONS section 3. The engine position is in body axes and
 * is measured from the center of gravity, so the airframe gets a yaw moment when
 * the two engines make different thrust.
 *
 * The update allocates nothing. Every scratch value is a field of the engine or
 * a local number.
 *
 * This module obeys the separation rule. It imports Vector3 from the Three.js
 * core and nothing else from the renderer.
 */

import { Vector3 } from 'three';
import {
  GAMMA,
  P0,
  RHO0,
  T0,
  createAtmosphereSample,
  isa,
  machNumber,
} from '@/physics/atmosphere';
import { clamp, lerp, lookup1d, lookup2d, table1d, table2d } from '@/math/tables';
import type { Table1D, Table2D } from '@/math/tables';
import { radPerSecToRpm, rpmToRadPerSec } from '@/math/units';

// ---------------------------------------------------------------------------
// Reference data. Every constant carries a source and a confidence mark.
// ---------------------------------------------------------------------------

/** Maximum static thrust at sea level, one engine.
 *  Source: Kay, "Junkers Aircraft and Engines", confidence: firm. */
export const MAX_THRUST_SL_STATIC = 8800; // N

/** Maximum rotor speed. Source: Jumo 004 B-1 data sheet, confidence: firm. */
export const MAX_RPM = 8700;

/** Idle rotor speed. Source: Me-262 pilot notes, confidence: firm. */
export const IDLE_RPM = 3000;

/** Top of the throttle danger band. Below this speed the surge margin is thin.
 *  Source: Me-262 pilot notes, confidence: firm. */
export const DANGER_BAND_RPM = 6000;

/** Air mass flow at full power, sea level static.
 *  Source: Jumo 004 B-1 data sheet, confidence: firm. */
export const MASS_FLOW_MAX = 21.2; // kg/s

/** Compressor pressure ratio at full power. Eight axial stages.
 *  Source: Jumo 004 B-1 data sheet, confidence: firm. */
export const PRESSURE_RATIO_MAX = 3.14;

/**
 * Rotor polar moment of inertia. ESTIMATED, confidence: low.
 *
 * Method. Two independent checks agree on the same range.
 *
 * 1. Mass and geometry. The engine dry mass is 719 kg. The rotor is the welded
 *    eight stage compressor drum, the shaft, and the single turbine disc. A
 *    rotor mass near 175 kg is the usual fraction for an axial machine of this
 *    size. The drum carries almost all of its mass at the outer radius, and the
 *    compressor tip diameter is 0.64 m, so a radius of gyration near 0.245 m
 *    follows. J = 175 * 0.245^2 = 10.5 kg m^2.
 * 2. Energy and time. The spool from idle to full power stores
 *    0.5 * J * (omega_max^2 - omega_idle^2). With J = 10.5 that is 3.8 MJ. Over
 *    8.5 seconds the mean surplus power is 450 kW, which is 15 percent of the
 *    3.04 MW that the compressor absorbs at full power. That fraction is right
 *    for an engine with this pressure ratio and this turbine.
 *
 * The value below is the fitted value. The fit ran the model from a settled
 * idle, with the lever advanced as fast as the surge margin allows, and moved J
 * until the rotor reached 95 percent of maximum speed inside the published 8 to
 * 10 second window. The fitted value gives 8.46 s.
 */
export const ROTOR_INERTIA = 10.5; // kg m^2

/**
 * Turbine inlet temperature limit. ESTIMATED, confidence: medium.
 *
 * Method. The gas path model gives 1015 K at full power from the published mass
 * flow, pressure ratio and fuel flow, and that value agrees with the published
 * turbine entry temperature near 1030 K. The turbine blades were hollow
 * Cromadur, a chrome manganese steel with no nickel, and they crept badly above
 * that point. The limit sits 85 K above the full power value, so steady full
 * power does no damage and a hard acceleration crosses the line.
 */
export const TURBINE_INLET_TEMPERATURE_LIMIT = 1100; // K

/**
 * Fuel flow at full power. DERIVED, confidence: medium.
 *
 * Method. The published specific fuel consumption is 1.4 kg per kilogram-force
 * per hour, which is 3.97e-5 kg/(N s). At 8800 N that gives 0.349 kg/s. The gas
 * path model needs 0.355 kg/s to hold 8700 rpm, which is the same number within
 * the error of the efficiencies. The fuel valve reaches a higher flow than this
 * so that the rotor speed governor has something to cut back.
 */
export const FUEL_FLOW_AT_MAX_POWER = 0.355; // kg/s

/** Maximum fuel valve flow. The governor holds the rotor at MAX_RPM below it. */
export const MAX_FUEL_FLOW = 0.4; // kg/s

/**
 * Fuel flow at idle. DERIVED from the torque balance at 3000 rpm,
 * confidence: low. The value is 221 kg/h, which sits inside the 190 to 250 kg/h
 * range that the wartime handling notes give for ground idle. The model settles
 * at 3046 rpm with it, against the published 3000 rpm.
 */
export const IDLE_FUEL_FLOW = 0.0613; // kg/s

/**
 * Lower heating value of J2 synthetic diesel. ESTIMATED, confidence: medium.
 * J2 was a coal derived gas oil close to a light diesel, so the value comes from
 * diesel fuel, 42.5 to 43.0 MJ/kg.
 */
export const FUEL_HEATING_VALUE = 42.8e6; // J/kg

/**
 * Usable internal fuel of the Me-262 A-1a. ESTIMATED, confidence: medium.
 * Two 900 liter main tanks, one 170 liter rear tank and one 600 liter forward
 * tank give 2570 liters. J2 has a density near 0.83 kg/liter, so the mass is
 * 2133 kg. The engine model does not burn this store. The fuel system bead owns
 * it. The value sits here because the endurance follows from the fuel flow.
 */
export const FUEL_CAPACITY = 2133; // kg

/** Riedel starter output, 10 hp in the intake bullet. Source: firm. */
export const STARTER_POWER = 7457; // W

/** Starter torque at low rotor speed. The value gives 6.8 kW at 500 rpm, which
 *  matches the 10 hp of the Riedel through its gearbox. ESTIMATED, medium. */
export const STARTER_MAX_TORQUE = 130; // N m

/** Rotor speed at which the starter torque falls to zero. The value is chosen so
 *  that the starter alone holds the rotor near STARTER_TARGET_RPM against the
 *  compressor and the bearings. ESTIMATED, confidence: low. */
export const STARTER_ZERO_TORQUE_RPM = 1200;

/** The Riedel starter drives the rotor toward this speed before light off. */
export const STARTER_TARGET_RPM = 800;

// ---------------------------------------------------------------------------
// Gas path constants.
// ---------------------------------------------------------------------------

/** Specific heat of air at constant pressure, cold end. Confidence: firm. */
const CP_AIR = 1005; // J/(kg K)

/** Specific heat of the combustion gas, hot end. Confidence: firm. */
const CP_GAS = 1148; // J/(kg K)

/** Ratio of specific heats of the hot gas. Confidence: firm. */
const GAMMA_GAS = 1.333;

/** Exponent of the isentropic relation on the cold side. */
const K_AIR = (GAMMA - 1) / GAMMA;

/** Exponent of the isentropic relation on the hot side. */
const K_GAS = (GAMMA_GAS - 1) / GAMMA_GAS;

/** Total pressure recovery of the intake. ESTIMATED, confidence: medium. */
export const INLET_PRESSURE_RECOVERY = 0.98;

/** Combustor total pressure loss at full corrected flow. ESTIMATED, medium.
 *  The loss follows the square of the corrected flow, so it almost vanishes at
 *  a low rotor speed. That is what lets the engine run at all near idle, where
 *  the compressor pressure ratio is only 1.13. */
export const COMBUSTOR_PRESSURE_LOSS = 0.03;

/** Combustion efficiency of the six straight through chambers. ESTIMATED. */
const COMBUSTION_EFFICIENCY = 0.95;

/** Turbine isentropic efficiency, single axial stage. ESTIMATED, medium. */
const TURBINE_EFFICIENCY = 0.85;

/** Highest gas temperature the model reports. A rich mixture cannot pass it. */
const MAX_FLAME_TEMPERATURE = 2400; // K

/** Bearing and accessory drag. Q = constant + linear * omega. ESTIMATED, low.
 *  The pair gives 58 N m at full speed, which is 1.7 percent of the compressor
 *  torque. */
const FRICTION_TORQUE_CONSTANT = 8; // N m
const FRICTION_TORQUE_LINEAR = 0.055; // N m s

/** Rotor speed floor of the torque divide. Below it the torques are negligible. */
const OMEGA_FLOOR = rpmToRadPerSec(30); // rad/s

/** Maximum rotor speed in radians per second. */
export const OMEGA_MAX = rpmToRadPerSec(MAX_RPM);

/** Idle rotor speed in radians per second. */
export const OMEGA_IDLE = rpmToRadPerSec(IDLE_RPM);

// ---------------------------------------------------------------------------
// Component maps. The x axis of each map is the corrected speed fraction,
// nc = (omega / OMEGA_MAX) / sqrt(theta), where theta is the inlet total
// temperature over the standard sea level temperature.
// ---------------------------------------------------------------------------

/** Compressor pressure ratio against corrected speed. The rise follows the
 *  square of the blade speed, so PR - 1 goes as nc^2.8. The end point is the
 *  published 3.14. Confidence: shape estimated, end point firm. */
const COMPRESSOR_PRESSURE_RATIO = table1d(
  [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
  [1.0, 1.01, 1.04, 1.09, 1.17, 1.3, 1.5, 1.78, 2.15, 2.6, 3.14],
);

/** Corrected mass flow against corrected speed, as a fraction of MASS_FLOW_MAX.
 *  Confidence: shape estimated, end point firm. */
const COMPRESSOR_CORRECTED_FLOW = table1d(
  [0, 0.2, 0.4, 0.6, 0.8, 1.0],
  [0, 0.16, 0.34, 0.55, 0.79, 1.0],
);

/** Compressor isentropic efficiency against corrected speed. The low value at
 *  low speed is the reason the engine spools slowly. ESTIMATED, confidence:
 *  low, end point medium. */
const COMPRESSOR_EFFICIENCY = table1d(
  [0, 0.1, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0],
  [0.45, 0.5, 0.54, 0.58, 0.63, 0.68, 0.745, 0.78],
);

/**
 * Share of the available pressure drop that the turbine takes, on a logarithmic
 * measure. The rest goes to the propelling nozzle.
 *
 * At full power the nozzle is close to critical and takes 44 percent of the
 * drop, which gives a turbine pressure ratio of 1.85. Near idle the nozzle
 * passes a small flow and takes almost nothing, so the turbine takes the whole
 * drop. ESTIMATED from the flow matching of a fixed area nozzle,
 * confidence: low.
 */
const TURBINE_PRESSURE_SHARE = table1d(
  [0, 0.3, 0.5, 0.7, 0.85, 1.0],
  [1.0, 0.995, 0.97, 0.88, 0.72, 0.5633],
);

/**
 * Surge line, as the fuel-air ratio the compressor can swallow at a corrected
 * speed. The margin is thin at a low speed and wide at a high speed. That is the
 * whole danger of this engine.
 *
 * The steady running line sits at 0.0095 at idle and 0.0165 at full power. The
 * fastest safe acceleration sits near 0.022, which is also the ratio that holds
 * the turbine inlet temperature at its limit. A throttle slam at idle asks for
 * 0.065, three times the limit, because the airflow of a 3000 rpm rotor is only
 * a third of the airflow at full power. The same slam at 7000 rpm asks for
 * 0.024 against a limit of 0.028, because the airflow is already there. That
 * difference is the whole danger of this engine.
 *
 * ESTIMATED, confidence: low. The shape follows the acceleration schedule of a
 * single spool turbojet.
 */
export const SURGE_FUEL_AIR_LIMIT = table1d(
  [0, 0.15, 0.25, 0.35, 0.5, 0.65, 0.8, 1.0],
  [0.0235, 0.0235, 0.0235, 0.0235, 0.0245, 0.026, 0.028, 0.03],
);

/**
 * Start schedule of the fuel control, against the physical speed fraction. The
 * value multiplies the flow that the lever asks for.
 *
 * The control senses the rotor speed. Below 1300 rpm the pump cannot make the
 * pressure to deliver the full flow, so the factor falls below one. That is what
 * keeps a normal light off cool. Between 1700 and 2600 rpm the control enriches,
 * so that the engine can accelerate itself to idle against a compressor that is
 * still working badly. At idle speed and above the factor is one and the lever
 * alone sets the flow.
 *
 * The factor multiplies the lever, so the pilot can still beat it. A lever above
 * the idle stop during a start makes a hot start.
 *
 * The curve holds the fuel-air ratio near 0.020 through the start, which is
 * above the running line and below the surge line. ESTIMATED, confidence: low.
 */
const START_FUEL_SCHEDULE = table1d(
  [0, 0.05, 0.092, 0.15, 0.2, 0.25, 0.3, 0.345, 1.0],
  [0, 0.28, 0.51, 0.85, 1.1, 1.3, 1.2, 1.0, 1.0],
);

/** Fuel valve flow against the throttle lever. The curve is convex, so the first
 *  part of the lever travel adds little fuel. The pilot needs that part of the
 *  travel to work inside the danger band. */
const THROTTLE_FUEL_SCHEDULE = table1d(
  [0, 0.25, 0.5, 0.75, 1.0],
  [IDLE_FUEL_FLOW, 0.105, 0.17, 0.27, MAX_FUEL_FLOW],
);

// ---------------------------------------------------------------------------
// Thrust model. Bead b33 tunes every constant in this block.
// ---------------------------------------------------------------------------

/** Mach knots of the thrust table. */
export const THRUST_TABLE_MACH: readonly number[] = [0, 0.2, 0.4, 0.6, 0.75, 0.9];

/** Pressure altitude knots of the thrust table, in meters. */
export const THRUST_TABLE_ALTITUDE: readonly number[] = [0, 2000, 4000, 6000, 8000, 10000, 12000];

/**
 * Thrust factor against Mach number, one value for each knot of
 * THRUST_TABLE_MACH. The net thrust of a turbojet with a pressure ratio near
 * three falls first, because the momentum drag of the intake grows faster than
 * the gross thrust, then recovers as the ram pressure rises. TUNED, not sourced.
 */
export const THRUST_MACH_FACTOR: readonly number[] = [1.0, 0.94, 0.9, 0.885, 0.89, 0.93];

/**
 * Exponent of the altitude lapse. The thrust falls as (density ratio) to this
 * power. A momentum analysis of a fixed rotor speed gives thrust proportional to
 * delta / sqrt(theta), which is the density ratio to the power 1.12 at 6000 m.
 * The engine also runs at a higher corrected speed in cold air, which gives some
 * of that back. TUNED, not sourced. Start at 1.0.
 */
export const THRUST_ALTITUDE_EXPONENT = 1.0;

/**
 * Rotor speed fraction below which the engine makes no net thrust. The jet
 * velocity falls to the intake velocity there. TUNED.
 */
export const THRUST_ZERO_SPEED_FRACTION = 0.12;

/**
 * Exponent of the thrust against the rotor speed fraction, above the zero point.
 * The offset makes the local exponent steeper than the value itself. At idle the
 * local exponent is 2.6 * 0.345 / 0.225 = 4.0, so the curve is steeper than a
 * cube law where it matters. TUNED.
 */
export const THRUST_SPEED_EXPONENT = 2.6;

/** Thrust that is left during a compressor stall. The flow breaks down and the
 *  jet pipe pressure collapses. TUNED, confidence: low. */
export const STALL_THRUST_FACTOR = 0.1;

/** Thrust that full turbine damage removes. TUNED, confidence: low. */
export const DAMAGE_THRUST_LOSS = 0.6;

/** Builds the thrust table in newtons at maximum rotor speed. The build runs one
 *  time at module load, not in the update. */
function buildThrustTable(): Table2D {
  const sample = createAtmosphereSample();
  const rows: number[][] = [];
  for (let iy = 0; iy < THRUST_TABLE_ALTITUDE.length; iy++) {
    isa(THRUST_TABLE_ALTITUDE[iy], sample);
    const lapse = Math.pow(sample.density / RHO0, THRUST_ALTITUDE_EXPONENT);
    const row: number[] = [];
    for (let ix = 0; ix < THRUST_TABLE_MACH.length; ix++) {
      row.push(MAX_THRUST_SL_STATIC * lapse * THRUST_MACH_FACTOR[ix]);
    }
    rows.push(row);
  }
  return table2d(THRUST_TABLE_MACH as number[], THRUST_TABLE_ALTITUDE as number[], rows);
}

/** Net thrust at maximum rotor speed, in newtons, against Mach and altitude. */
export const THRUST_TABLE: Table2D = buildThrustTable();

/** Returns the thrust fraction that the rotor speed fraction gives. */
export function thrustSpeedFraction(speedFraction: number): number {
  const above = clamp(
    (speedFraction - THRUST_ZERO_SPEED_FRACTION) / (1 - THRUST_ZERO_SPEED_FRACTION),
    0,
    1,
  );
  return Math.pow(above, THRUST_SPEED_EXPONENT);
}

// ---------------------------------------------------------------------------
// Failure and start constants.
// ---------------------------------------------------------------------------

/** A stall that lasts longer than this ends in a flame out. Source: the pilot
 *  notes tell the pilot to close the throttle at once. Confidence: firm on the
 *  behavior, estimated on the time. */
export const STALL_FLAMEOUT_TIME = 2.0; // s

/** The surge margin must climb back above this value to clear a stall. */
export const SURGE_RECOVERY_MARGIN = 0.1;

/** Time between two surge bangs while the compressor keeps stalling. */
export const SURGE_BANG_INTERVAL = 0.35; // s

/** Air mass flow that is left during a stall, as a fraction of the clean flow. */
const STALL_FLOW_FRACTION = 0.6;

/** Combustion efficiency during a stall. The rest of the fuel burns in the jet
 *  pipe and makes the bang. */
const STALL_BURN_EFFICIENCY = 0.5;

/** Torque that the turbine still makes during a stall, as a fraction. */
const STALL_TURBINE_FACTOR = 0.35;

/** Below this rotor speed a lit engine cannot hold its flame. ESTIMATED. */
export const MIN_SUSTAIN_RPM = 1200;

/** Below this fuel-air ratio the flame goes out lean. ESTIMATED. */
const LEAN_BLOWOUT_FUEL_AIR = 0.004;

/** The rotor must turn at least this fast before the fuel lights. ESTIMATED. */
export const LIGHT_OFF_MIN_RPM = 500;

/** Time from the fuel cock to the flame. Fuel that arrives before the rotor is
 *  ready pools in the chambers over this time. ESTIMATED. */
export const LIGHT_OFF_DELAY = 0.4; // s

/** Time after a light off in which an over temperature counts as a hot start. */
export const HOT_START_WINDOW = 10; // s

/** Time over which pooled fuel burns after a light off. ESTIMATED. */
const POOL_BURN_TIME = 2.5; // s

/** Largest pool of unburned fuel the chambers hold. ESTIMATED. */
const MAX_POOLED_FUEL = 0.8; // kg

/** Damage rate at the temperature limit. Full damage takes five hours there.
 *  The published time between overhaul is 10 to 25 hours, and the engine spent
 *  most of that time below the limit. ESTIMATED, confidence: low. */
export const DAMAGE_RATE_AT_LIMIT = 1 / (5 * 3600); // 1/s

/** Temperature rise that multiplies the damage rate by e. The creep of a steel
 *  turbine blade climbs this fast with temperature. ESTIMATED, low. */
export const DAMAGE_TEMPERATURE_SCALE = 40; // K

/** Highest damage rate. The blades crept, they did not fail in one instant, so
 *  even a surge at 2000 K needs five seconds to ruin the turbine. ESTIMATED. */
export const DAMAGE_MAX_RATE = 0.2; // 1/s

/** Damage rate while the engine burns. */
const FIRE_DAMAGE_RATE = 0.1; // 1/s

/** Gas temperature that starts a fire when it lasts. ESTIMATED. */
export const FIRE_TEMPERATURE = 1900; // K

/** Time above FIRE_TEMPERATURE that starts a fire. It is longer than
 *  STALL_FLAMEOUT_TIME, so a plain surge flames out and does not burn. */
export const FIRE_TIME = 3.0; // s

/** A pool of fuel larger than this, lit by hot metal, starts a jet pipe fire. */
const FIRE_POOL_FUEL = 0.5; // kg

/** Turbine efficiency that full damage removes. */
const DAMAGE_TURBINE_LOSS = 0.15;

/** Rotor speed of a windmilling engine, against true airspeed. ESTIMATED from
 *  the fixed flow coefficient of an unlit turbomachine, confidence: low.
 *  At 139 m/s the free windmill speed is 1390 rpm, which is 16 percent of the
 *  maximum. The rotor settles a little below it, because the bearings take a
 *  part of the ram torque. */
export const WINDMILL_RPM_PER_MS = 10;

/** Torque that drives the unlit rotor toward its windmill speed, for each radian
 *  per second below that speed, at sea level density and at full ram flow. The
 *  value is large enough that the rotor settles within 15 percent of the
 *  windmill speed against the motoring drag. ESTIMATED, confidence: low. */
const WINDMILL_TORQUE_GAIN = 4.0; // N m s

/** Airspeed that gives the unlit engine its full ram flow. ESTIMATED. */
const WINDMILL_REFERENCE_SPEED = 150; // m/s

/** Motoring drag of the unlit rotor, for each radian per second, at sea level
 *  density. The drag holds the rotor near 800 rpm against the starter and lets
 *  it coast down from full speed in about 25 seconds with no airflow. Both
 *  numbers match the handling notes. ESTIMATED, confidence: low. */
const MOTORING_DRAG = 0.36; // N m s

/** Lowest windmill rotor speed that can carry a relight. ESTIMATED. */
export const RELIGHT_MIN_RPM = 1100;

/** Lowest true airspeed that windmills the rotor fast enough for a relight.
 *  DERIVED from RELIGHT_MIN_RPM and WINDMILL_RPM_PER_MS. The relight test uses
 *  the rotor speed, not this speed, so the window comes out of the rotor. */
export const RELIGHT_MIN_SPEED = RELIGHT_MIN_RPM / WINDMILL_RPM_PER_MS; // m/s, 396 km/h

/** Highest true airspeed for a relight. Above it the flame blows out. */
export const RELIGHT_MAX_SPEED = 250; // m/s, about 900 km/h

/** The throttle must sit below this value before a relight. */
export const RELIGHT_MAX_THROTTLE = 0.05;

/** Rotor speed governor droop. The fuel falls to a quarter over this many rpm
 *  above MAX_RPM. The B-1 fuel control held the rotor speed this way. */
export const OVERSPEED_DROOP_RPM = 300;

/** Time constant of the fuel valve and the fuel line. */
const FUEL_VALVE_TIME = 0.15; // s

/** Time constant of the gas temperature that the gauge and the metal see. */
const TEMPERATURE_LAG = 0.35; // s

/** Longest internal step of the rotor integration. */
const MAX_SUBSTEP = 0.005; // s

// ---------------------------------------------------------------------------
// Public types.
// ---------------------------------------------------------------------------

export type EngineState =
  | 'off'
  | 'starter'
  | 'lightOff'
  | 'idle'
  | 'running'
  | 'stall'
  | 'flameout'
  | 'fire';

export interface EngineInput {
  throttle: number; // 0..1, the pilot lever
  fuelCockOpen: boolean;
  starterEngaged: boolean;
  altitude: number; // m
  mach: number;
  airspeed: number; // m/s, true
  density: number; // kg/m^3
  fuelAvailable: boolean;
}

/**
 * One shot flags and running counts for the audio and the head up display.
 *
 * A flag holds for the one update that made it. A count only grows, so a reader
 * that runs slower than the physics can compare the count against its own copy
 * and miss nothing.
 */
export interface EngineEvents {
  surgeBang: boolean;
  flameout: boolean;
  hotStart: boolean;
  lightOff: boolean;
  fire: boolean;
  surgeBangCount: number;
  flameoutCount: number;
  hotStartCount: number;
}

export interface Engine {
  readonly state: EngineState;
  readonly rpm: number;
  readonly rotorSpeed: number; // rad/s
  readonly thrust: number; // N
  readonly gasTemperature: number; // K
  readonly fuelFlow: number; // kg/s
  readonly surgeMargin: number; // negative means surging
  readonly damage: number; // 0..1, permanent
  readonly position: Vector3; // body axes, from the CG
  readonly events: EngineEvents;
  update(input: EngineInput, dt: number): void;
  shutdown(): void;
  reset(): void;
}

/** Returns true while a flame burns in the chambers. */
function isLit(state: EngineState): boolean {
  return (
    state === 'lightOff' ||
    state === 'idle' ||
    state === 'running' ||
    state === 'stall' ||
    state === 'fire'
  );
}

// ---------------------------------------------------------------------------
// The engine.
// ---------------------------------------------------------------------------

class Jumo004 implements Engine {
  state: EngineState = 'off';
  rpm = 0;
  rotorSpeed = 0;
  thrust = 0;
  gasTemperature = T0;
  fuelFlow = 0;
  surgeMargin = 1;
  damage = 0;
  readonly position: Vector3;
  readonly events: EngineEvents = {
    surgeBang: false,
    flameout: false,
    hotStart: false,
    lightOff: false,
    fire: false,
    surgeBangCount: 0,
    flameoutCount: 0,
    hotStartCount: 0,
  };

  /** Scratch for the atmosphere. The engine holds one sample and reuses it. */
  private readonly air = createAtmosphereSample();

  /** Corrected speed fraction of the last step. The thrust curve reads it. */
  private correctedSpeed = 0;

  /** Time since the last light off, in seconds. A negative value means that the
   *  window for a hot start has closed. */
  private sinceLight = -1;

  private hotStartLatched = false;

  private stallTimer = 0; // s
  private cockTimer = 0; // s, time the fuel cock has been open
  private bangTimer = 0; // s
  private fireTimer = 0; // s
  private poolBurnTimer = 0; // s
  private pooledFuel = 0; // kg
  private poolBurnMass = 0; // kg
  private shutdownLatched = false;

  constructor(position: Vector3) {
    this.position = position.clone();
  }

  update(input: EngineInput, dt: number): void {
    const events = this.events;
    events.surgeBang = false;
    events.flameout = false;
    events.hotStart = false;
    events.lightOff = false;
    events.fire = false;
    if (!(dt > 0)) {
      return;
    }

    // The atmosphere and the inlet do not change inside one update.
    isa(input.altitude, this.air);
    const ambientTemperature = this.air.temperature;
    const ambientPressure = this.air.pressure;
    const mach =
      input.mach > 0 ? input.mach : machNumber(input.airspeed, this.air.speedOfSound);
    // Total conditions behind a normal shock free intake.
    const ram = 1 + 0.5 * (GAMMA - 1) * mach * mach;
    const inletTemperature = ambientTemperature * ram;
    // The intake loses part of the ram pressure rise, not part of the ambient
    // pressure. At rest the inlet total pressure equals the ambient pressure.
    const inletPressure =
      ambientPressure * (1 + (Math.pow(ram, 1 / K_AIR) - 1) * INLET_PRESSURE_RECOVERY);
    const theta = inletTemperature / T0;
    const delta = inletPressure / P0;
    const nozzleRam = inletPressure / ambientPressure;

    const steps = Math.max(1, Math.ceil(dt / MAX_SUBSTEP));
    const h = dt / steps;
    for (let i = 0; i < steps; i++) {
      this.step(input, h, theta, delta, inletTemperature, nozzleRam);
    }

    this.rpm = radPerSecToRpm(this.rotorSpeed);
    this.updateThrust(mach, input.altitude);
  }

  /** One internal step of the gas path, the rotor and the state machine. */
  private step(
    input: EngineInput,
    h: number,
    theta: number,
    delta: number,
    inletTemperature: number,
    nozzleRam: number,
  ): void {
    const omega = this.rotorSpeed;
    const speedFraction = omega / OMEGA_MAX;
    const correctedSpeed = speedFraction / Math.sqrt(theta);
    this.correctedSpeed = correctedSpeed;
    // A fire breaks the gas path the same way a stall does. The flame sits in
    // the jet pipe, so the flow falls and the turbine gets little work.
    const stalled = this.state === 'stall' || this.state === 'fire';
    const lit = isLit(this.state);

    // --- Compressor -------------------------------------------------------
    const pressureRatio = lookup1d(COMPRESSOR_PRESSURE_RATIO, correctedSpeed);
    const flowFraction = lookup1d(COMPRESSOR_CORRECTED_FLOW, correctedSpeed);
    const efficiency = lookup1d(COMPRESSOR_EFFICIENCY, correctedSpeed);
    let airFlow = (MASS_FLOW_MAX * flowFraction * delta) / Math.sqrt(theta);
    if (stalled) {
      airFlow *= STALL_FLOW_FRACTION;
    }
    const compressorWork =
      (CP_AIR * inletTemperature * (Math.pow(pressureRatio, K_AIR) - 1)) / efficiency;
    const deliveryTemperature = inletTemperature + compressorWork / CP_AIR;
    const divisor = omega > OMEGA_FLOOR ? omega : OMEGA_FLOOR;
    const compressorTorque = (airFlow * compressorWork) / divisor;

    // --- Fuel -------------------------------------------------------------
    this.cockTimer = input.fuelCockOpen && input.fuelAvailable ? this.cockTimer + h : 0;
    const cockDelivers =
      input.fuelCockOpen && input.fuelAvailable && !this.shutdownLatched;
    if (cockDelivers) {
      const lever = lookup1d(THROTTLE_FUEL_SCHEDULE, clamp(input.throttle, 0, 1));
      const start = lookup1d(START_FUEL_SCHEDULE, speedFraction);
      // The barostatic unit of the B-1 cuts the fuel with the inlet pressure, so
      // the fuel-air ratio holds as the aircraft climbs.
      const governor = clamp(1 - (this.rpm - MAX_RPM) / OVERSPEED_DROOP_RPM, 0.25, 1);
      const command = lever * start * delta * governor;
      // The metering valve and the fuel line take time to answer the lever.
      this.fuelFlow = lerp(this.fuelFlow, command, clamp(h / FUEL_VALVE_TIME, 0, 1));
      if (this.fuelFlow < 1e-6) {
        this.fuelFlow = 0;
      }
    } else {
      // The cock is a shutoff valve, not a metering valve. It stops the flow at
      // once. A closed cock therefore means exactly zero fuel flow.
      this.fuelFlow = 0;
    }

    // --- Combustor --------------------------------------------------------
    let heat = 0;
    if (lit) {
      const burn = stalled ? STALL_BURN_EFFICIENCY : COMBUSTION_EFFICIENCY;
      heat = this.fuelFlow * FUEL_HEATING_VALUE * burn;
      if (this.poolBurnTimer > 0) {
        heat += (this.poolBurnMass / POOL_BURN_TIME) * FUEL_HEATING_VALUE * burn;
        this.poolBurnTimer -= h;
      }
    } else {
      // Unburned fuel gathers in the chambers and in the jet pipe.
      this.pooledFuel = Math.min(MAX_POOLED_FUEL, this.pooledFuel + this.fuelFlow * h);
    }
    const gasFlow = airFlow + this.fuelFlow;
    const turbineInlet =
      gasFlow > 1e-3
        ? Math.min(MAX_FLAME_TEMPERATURE, deliveryTemperature + heat / (gasFlow * CP_GAS))
        : deliveryTemperature;

    // --- Turbine ----------------------------------------------------------
    // The available pressure drop runs from the combustor exit to the ambient
    // static pressure. The loss of the combustor follows the square of the
    // corrected flow, so it nearly vanishes near idle.
    const loss = COMBUSTOR_PRESSURE_LOSS * flowFraction * flowFraction;
    const available = Math.log(Math.max(1, pressureRatio * (1 - loss) * nozzleRam));
    const share = lookup1d(TURBINE_PRESSURE_SHARE, correctedSpeed);
    const logTurbine = share * available;
    let turbineEfficiency = TURBINE_EFFICIENCY * (1 - DAMAGE_TURBINE_LOSS * this.damage);
    if (stalled) {
      turbineEfficiency *= STALL_TURBINE_FACTOR;
    }
    // Only a lit engine drives the rotor from the gas path. The unlit rotor uses
    // the windmill model below, because the maps do not hold with no flame.
    const turbineWork = lit
      ? turbineEfficiency * CP_GAS * turbineInlet * (1 - Math.exp(-K_GAS * logTurbine))
      : 0;
    const turbineTorque = (gasFlow * turbineWork) / divisor;

    // --- Other torques ----------------------------------------------------
    const friction = FRICTION_TORQUE_CONSTANT + FRICTION_TORQUE_LINEAR * omega;
    let starterTorque = 0;
    if (input.starterEngaged && !this.shutdownLatched) {
      starterTorque =
        STARTER_MAX_TORQUE * clamp(1 - this.rpm / STARTER_ZERO_TORQUE_RPM, 0, 1);
    }

    // --- Rotor ------------------------------------------------------------
    // A lit rotor follows the gas path. An unlit rotor does not. With no flame
    // the quasi steady maps do not hold: the blades sit at the wrong incidence,
    // the compressor stalls and the machine pumps badly. The maps would then
    // report a drag near 2000 N m at full speed and stop the rotor in five
    // seconds, where a real engine coasts down for half a minute. The unlit
    // rotor therefore follows a windmill characteristic and a motoring drag.
    let net: number;
    if (lit) {
      net = turbineTorque + starterTorque - compressorTorque - friction;
    } else {
      const speed = Math.abs(input.airspeed);
      const densityRatio = clamp(input.density / RHO0, 0, 2);
      const target = rpmToRadPerSec(WINDMILL_RPM_PER_MS * speed);
      const ramFlow = clamp(speed / WINDMILL_REFERENCE_SPEED, 0, 1);
      // The ram drives the rotor up to the windmill speed and no further.
      const drive =
        WINDMILL_TORQUE_GAIN * densityRatio * ramFlow * Math.max(0, target - omega);
      const motoring = MOTORING_DRAG * densityRatio * omega;
      net = drive + starterTorque - motoring - friction;
    }
    let next = omega + (net / ROTOR_INERTIA) * h;
    if (next < 0) {
      next = 0;
    }
    this.rotorSpeed = next;
    this.rpm = radPerSecToRpm(next);

    // --- Surge margin -----------------------------------------------------
    const limit = lookup1d(SURGE_FUEL_AIR_LIMIT, correctedSpeed);
    const fuelAir = lit && airFlow > 1e-3 ? this.fuelFlow / airFlow : 0;
    this.surgeMargin = lit ? clamp(1 - fuelAir / limit, -2, 1) : 1;

    // --- Temperature, damage and the state machine ------------------------
    // The gauge and the turbine metal both lag the gas. The damage law reads the
    // lagged value, so one short spike does not creep the blades.
    this.gasTemperature = lerp(
      this.gasTemperature,
      lit ? turbineInlet : deliveryTemperature,
      clamp(h / TEMPERATURE_LAG, 0, 1),
    );
    this.accumulateDamage(h);
    this.stepState(input, h, fuelAir);
  }

  /** Adds creep damage while the gas temperature sits above the limit. */
  private accumulateDamage(h: number): void {
    const over = this.gasTemperature - TURBINE_INLET_TEMPERATURE_LIMIT;
    let rate = 0;
    if (over > 0) {
      rate = DAMAGE_RATE_AT_LIMIT * Math.exp(over / DAMAGE_TEMPERATURE_SCALE);
    }
    if (this.state === 'fire') {
      rate += FIRE_DAMAGE_RATE;
    }
    if (rate > 0) {
      this.damage = clamp(this.damage + Math.min(rate, DAMAGE_MAX_RATE) * h, 0, 1);
    }
  }

  /** Runs the state machine and raises the events. */
  private stepState(input: EngineInput, h: number, fuelAir: number): void {
    const events = this.events;
    const lit = isLit(this.state);
    const cockOpen = input.fuelCockOpen && input.fuelAvailable && !this.shutdownLatched;
    // The starter holds the rotor turning, so the flame lives below the speed at
    // which the engine could hold it alone. That is what a start is.
    const cranking = input.starterEngaged && !this.shutdownLatched;

    // A fire holds until the pilot shuts the engine down.
    if (this.state === 'fire') {
      return;
    }

    // A hot start is an over temperature in the first seconds after a light
    // off. The state can be stall by then, because one cause makes both.
    if (this.sinceLight >= 0) {
      this.sinceLight += h;
      if (this.sinceLight > HOT_START_WINDOW) {
        this.sinceLight = -1;
      } else if (
        !this.hotStartLatched &&
        this.gasTemperature > TURBINE_INLET_TEMPERATURE_LIMIT
      ) {
        this.hotStartLatched = true;
        events.hotStart = true;
        events.hotStartCount++;
      }
    }

    // Fire from a long over temperature or from a pool of fuel on hot metal.
    if (this.gasTemperature > FIRE_TEMPERATURE) {
      this.fireTimer += h;
    } else {
      this.fireTimer = 0;
    }
    const poolFire =
      !lit && cockOpen && this.pooledFuel > FIRE_POOL_FUEL && this.gasTemperature > 900;
    if (this.fireTimer >= FIRE_TIME || poolFire) {
      this.state = 'fire';
      events.fire = true;
      return;
    }

    switch (this.state) {
      case 'off':
        if (input.starterEngaged && !this.shutdownLatched) {
          this.state = 'starter';
        }
        break;

      case 'starter':
        if (!input.starterEngaged) {
          this.state = 'off';
        } else if (this.canLight(input)) {
          this.light(events);
        }
        break;

      case 'flameout':
        // An air start needs the throttle at idle and a windmill speed inside
        // the relight window.
        if (
          cockOpen &&
          input.throttle <= RELIGHT_MAX_THROTTLE &&
          input.airspeed <= RELIGHT_MAX_SPEED &&
          (this.rpm >= RELIGHT_MIN_RPM || input.starterEngaged) &&
          this.rpm >= LIGHT_OFF_MIN_RPM &&
          this.cockTimer >= LIGHT_OFF_DELAY
        ) {
          this.light(events);
        }
        break;

      case 'lightOff':
        if (!cockOpen) {
          this.flameOut(events);
        } else if (this.surgeMargin < 0) {
          // Too much fuel during a start stalls the compressor as surely as a
          // throttle slam does.
          this.state = 'stall';
          this.stallTimer = 0;
          this.bangTimer = SURGE_BANG_INTERVAL;
        } else if (this.rpm >= IDLE_RPM * 0.95) {
          this.state = input.throttle > RELIGHT_MAX_THROTTLE ? 'running' : 'idle';
        }
        break;

      case 'idle':
      case 'running':
        if (
          !cockOpen ||
          (this.rpm < MIN_SUSTAIN_RPM && !cranking) ||
          fuelAir < LEAN_BLOWOUT_FUEL_AIR
        ) {
          this.flameOut(events);
        } else if (this.surgeMargin < 0) {
          this.state = 'stall';
          this.stallTimer = 0;
          this.bangTimer = SURGE_BANG_INTERVAL;
        } else {
          this.state =
            input.throttle > RELIGHT_MAX_THROTTLE || this.rpm > IDLE_RPM * 1.05
              ? 'running'
              : 'idle';
        }
        break;

      case 'stall':
        if (!cockOpen || (this.rpm < MIN_SUSTAIN_RPM && !cranking)) {
          this.flameOut(events);
          break;
        }
        this.stallTimer += h;
        this.bangTimer += h;
        if (this.bangTimer >= SURGE_BANG_INTERVAL) {
          this.bangTimer = 0;
          events.surgeBang = true;
          events.surgeBangCount++;
        }
        if (this.surgeMargin > SURGE_RECOVERY_MARGIN) {
          this.state = this.rpm > IDLE_RPM * 1.05 ? 'running' : 'idle';
          this.stallTimer = 0;
        } else if (this.stallTimer >= STALL_FLAMEOUT_TIME) {
          this.flameOut(events);
        }
        break;
    }
  }

  /** Returns true when the fuel in the chambers can light. */
  private canLight(input: EngineInput): boolean {
    return (
      input.fuelCockOpen &&
      input.fuelAvailable &&
      !this.shutdownLatched &&
      this.rpm >= LIGHT_OFF_MIN_RPM &&
      this.cockTimer >= LIGHT_OFF_DELAY &&
      this.fuelFlow > 0
    );
  }

  /** Lights the flame and starts the pooled fuel burning. */
  private light(events: EngineEvents): void {
    this.state = 'lightOff';
    this.sinceLight = 0;
    this.hotStartLatched = false;
    this.poolBurnMass = this.pooledFuel;
    this.pooledFuel = 0;
    this.poolBurnTimer = this.poolBurnMass > 0 ? POOL_BURN_TIME : 0;
    events.lightOff = true;
  }

  /** Puts the flame out. */
  private flameOut(events: EngineEvents): void {
    this.state = 'flameout';
    this.stallTimer = 0;
    events.flameout = true;
    events.flameoutCount++;
  }

  /** Writes the thrust from the table, the rotor speed and the damage. */
  private updateThrust(mach: number, altitude: number): void {
    if (!isLit(this.state)) {
      this.thrust = 0;
      return;
    }
    let value = lookup2d(THRUST_TABLE, mach, altitude);
    // The curve reads the corrected speed, not the physical speed. A turbojet
    // sits at the same point of its characteristic at the same corrected speed,
    // and the table already carries the altitude lapse. Reading the physical
    // speed would count the altitude twice.
    value *= thrustSpeedFraction(this.correctedSpeed);
    value *= 1 - DAMAGE_THRUST_LOSS * this.damage;
    if (this.state === 'stall') {
      value *= STALL_THRUST_FACTOR;
    }
    if (this.state === 'fire') {
      value *= STALL_THRUST_FACTOR;
    }
    this.thrust = value;
  }

  /** Closes the fuel cock and stops the engine. The rotor coasts down. */
  shutdown(): void {
    this.shutdownLatched = true;
    this.state = 'off';
    this.fuelFlow = 0;
    this.stallTimer = 0;
    this.poolBurnTimer = 0;
    this.pooledFuel = 0;
    this.surgeMargin = 1;
    this.thrust = 0;
  }

  /** Returns the engine to the cold state. The damage clears with it. */
  reset(): void {
    this.state = 'off';
    this.rotorSpeed = 0;
    this.rpm = 0;
    this.thrust = 0;
    this.gasTemperature = T0;
    this.fuelFlow = 0;
    this.surgeMargin = 1;
    this.damage = 0;
    this.stallTimer = 0;
    this.cockTimer = 0;
    this.bangTimer = 0;
    this.fireTimer = 0;
    this.poolBurnTimer = 0;
    this.poolBurnMass = 0;
    this.pooledFuel = 0;
    this.sinceLight = -1;
    this.hotStartLatched = false;
    this.shutdownLatched = false;
    const events = this.events;
    events.surgeBang = false;
    events.flameout = false;
    events.hotStart = false;
    events.lightOff = false;
    events.fire = false;
    events.surgeBangCount = 0;
    events.flameoutCount = 0;
    events.hotStartCount = 0;
  }
}

/** Makes one Jumo 004 B-1 at a position in body axes, measured from the CG. */
export function createJumo004(position: Vector3): Engine {
  return new Jumo004(position);
}

/** The tables are part of the public surface so that bead b33 can read them. */
export type { Table1D, Table2D };
