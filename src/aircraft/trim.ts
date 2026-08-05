/**
 * The trim solver of the Messerschmitt Me 262 A-1a.
 *
 * A trim is the set of control positions that holds a steady flight condition.
 * Steady means that the linear acceleration, the angular acceleration and the
 * angular rate all keep the value the maneuver asks for, and that nothing in the
 * state changes with time. For level flight all three are zero.
 *
 *
 * 1. THE PROBLEM IS COUPLED, SO THE SOLVER SOLVES IT COUPLED
 *
 * Three equations must hold at the same time.
 *
 *   along the flight path        thrust - drag - the weight component = 0
 *   across the flight path       lift - the weight component = the maneuver
 *   about the pitch axis         the total pitching moment = 0
 *
 * Three unknowns answer them. A sequential search that sets the angle of attack
 * from the lift, then the elevator from the moment, then the throttle from the
 * drag, has to run the loop again, because the elevator changed the lift and the
 * throttle changed the pitching moment through the thrust line. That loop
 * converges slowly where the three couplings are weak and it does not converge
 * at all where one of them is strong. Near the stall the lift no longer grows
 * with the angle of attack, so the first step of the sequence has no answer.
 * Near the Mach limit the elevator loses its power, so the second step has no
 * answer either. Both are exactly the conditions the flight tests measure.
 *
 * The solver therefore builds ONE residual vector of three normalized numbers
 * and drives it to zero with a Newton step on a numerical Jacobian. Each Newton
 * step needs one residual and six more for the central difference Jacobian. A
 * backtracking line search protects the step where the Jacobian is nearly
 * singular, which is what the stall and the Mach limit both look like.
 *
 * WHAT A CALLER GETS WHEN NO TRIM EXISTS. Below the stall speed no angle of
 * attack makes enough lift, and above the thrust limit no throttle makes enough
 * thrust. The solver reports `converged: false` and the residual it reached. It
 * never reports a wrong answer as a right one. A caller that searches for the
 * stall speed or the maximum level speed uses that flag as its test.
 *
 *
 * 2. THE THREE MODES
 *
 *   trimLevelFlight     The flight path is level and the speed is given. The free
 *                       values are the angle of attack, the elevator and the
 *                       throttle.
 *   trimForAlpha        The angle of attack is given and the flight path is
 *                       level. The free values are the SPEED, the elevator and
 *                       the throttle.
 *   trimSteadyClimb     The throttle and the speed are given. The free values
 *                       are the angle of attack, the flight path angle and the
 *                       elevator. This is the rate of climb and the service
 *                       ceiling, with no small angle assumption and with the
 *                       thrust component along the path included.
 *   trimMaxLevelSpeed   The throttle is full and the flight path is level. The
 *                       free values are the speed, the angle of attack and the
 *                       elevator. This is the maximum level speed in one solve.
 *
 * All four run the same Newton core over the same residual. Only the list of
 * free values changes.
 *
 * WHY THE SPEED IS THE FREE VALUE OF trimForAlpha. The lift of a steady straight
 * flight must equal the load factor times the weight, whatever the flight path
 * angle is: the weight component across the path and the path curvature term
 * cancel exactly. The angle of attack and the speed together fix the lift, so an
 * angle of attack that is given fixes the speed and nothing else can answer that
 * equation. A solver that freed the flight path angle instead would work on a
 * singular column and report a wrong answer with confidence.
 *
 *
 * 3. THE MODEL THIS FILE FLIES
 *
 * The solver builds its own aerodynamic assembly, its own mass model and its own
 * engine, from the same modules the aircraft builds them from. It does not step
 * the rigid body at all. It evaluates the wrench one time for each residual.
 *
 * THE RESIDUAL HAS TO BE A FUNCTION, AND assembly.evaluateSteady GIVES ONE.
 * A numerical Jacobian is the residual at two states that differ by one part in
 * a hundred thousand. Anything the model remembers between two calls therefore
 * lands in the Jacobian, and a Jacobian of memory is noise. The aerodynamics
 * used to remember two things, and bead b61 closed both inside the assembly.
 *
 *   The lag states. src/physics/aero/surface.ts carries a separation lag and
 *   src/physics/aero/downwash.ts carries a downwash lag. Both now use the exact
 *   solution of a first order system, and evaluateSteady steps both with a dt of
 *   Infinity, so both reach their steady value exactly and carry nothing over.
 *
 *   The induced angle pass. assembly.evaluate solved the induced angle from a
 *   linear estimate that read the SEPARATION STATE the last evaluation left
 *   behind. One call was therefore not a function of the state it received, and
 *   near the stall the same state gave two answers that differed by a factor of
 *   four. This file had to iterate evaluate to its own fixed point to get a
 *   Jacobian that meant anything. assembly.evaluate now owns that fixed point,
 *   so one call is enough.
 *
 * THE ENGINE. The thrust of a Jumo 004 follows the rotor speed, and the rotor
 * needs about eight seconds to answer the lever. A trim needs the SETTLED
 * thrust. The solver runs the engine model of src/aircraft/me262/engine.ts to
 * its steady rotor speed over a grid of lever positions and airspeeds, one time
 * for each altitude, and reads the grid inside the Newton loop. The grid is
 * cached per altitude.
 *
 * The grid holds the speed as well as the lever, and not the lever alone,
 * because the speed is a FREE value in two of the four modes. A thrust that
 * jumped at a speed step would stop the Newton step at the size of the jump. The
 * bilinear grid is continuous in both directions, so the residual is continuous
 * and the solver can drive it to the tolerance.
 *
 * THE LANDING GEAR. `gearDown` puts the flat plate area of gearDragAreaAt of
 * src/aircraft/me262/systems.ts into the residual, at
 * ME262_GEAR_DRAG_POSITION of src/physics/gear.ts, exactly as
 * src/aircraft/aircraft.ts does it. THE TWO MUST AGREE. This solver held no gear
 * drag at all while the flight model held none either, which bead b32 recorded
 * as a known gap. Bead b59 closed the gap in the flight model, and a solver left
 * behind would hand the harness an approach throttle that is 0.7 kN short. The
 * aircraft then sinks at 0.6 m/s on a trim that says it holds height.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Vector3 } from 'three';

import { clamp, lookup2d, table2d } from '@/math/tables';
import type { Table2D } from '@/math/tables';
import { G0 } from '@/math/units';
import type { AtmosphereSample } from '@/physics/atmosphere';
import { createAtmosphereSample, isa, machNumber } from '@/physics/atmosphere';
import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import {
  airspeedBody,
  bodyToWorld,
  clearWrench,
  createState,
  createWrench,
  worldToBody,
} from '@/physics/rigidbody';
import { ME262_GEAR_DRAG_POSITION } from '@/physics/gear';
import type { AeroAssembly } from '@/physics/aero/assembly';
import { ELEVATOR_LIMIT, MIN_GEAR_DRAG_SPEED } from '@/aircraft/aircraft';
import type { Engine, EngineInput, EngineState } from '@/aircraft/me262/engine';
import { createJumo004 } from '@/aircraft/me262/engine';
import {
  CONTROL_COUNT,
  CONTROL_INDEX,
  ENGINE_POSITION_LEFT,
  ENGINE_POSITION_RIGHT,
  FLAP_LANDING_ANGLE,
  MAC,
  WING_AREA,
  createMe262Assembly,
} from '@/aircraft/me262/geometry';
import { me262Mass } from '@/aircraft/me262/mass';
import type { FlapSetting } from '@/aircraft/me262/systems';
import { flapSettingPosition, gearDragAreaAt } from '@/aircraft/me262/systems';

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

export interface TrimCondition {
  /** Pressure altitude, m. */
  altitude: number;
  /** True airspeed, m/s. */
  speed: number;
  flapSetting: FlapSetting;
  /** Carried to the result. See the module comment on the landing gear. */
  gearDown: boolean;
  /** Fuel on board, kg. It sets the mass, the balance and the inertia. */
  fuelMass: number;
  /**
   * Load factor of the maneuver, as an accelerometer reads it.
   *
   * IT DEFAULTS TO cos(flight path angle), WHICH IS A STRAIGHT PATH, and that is
   * 1 in level flight. A steady straight climb carries a lift of W cos(gamma),
   * not W, because the weight leans back along the path. A trim that forced 1 g
   * in a climb would be a slowly curving pull up and not a steady climb at all.
   *
   * A value the caller gives is a steady symmetric pull up: the flight path
   * bends upward at g (n - cos gamma) and the body pitch rate is that
   * acceleration over the speed. The pitch rate goes into the state the
   * aerodynamics reads, so the pitch damping of the tail appears in the trim, as
   * it must.
   */
  loadFactor?: number;
}

export interface TrimResult {
  /** Pilot command, -1 to 1. It is what AircraftInput.pitch takes. */
  elevator: number;
  /** Pilot command, 0 to 1. It is what AircraftInput.throttle takes. */
  throttle: number;
  /** Angle of attack, rad. */
  alpha: number;
  /** Pitch attitude, rad. It is alpha plus the flight path angle. */
  pitch: number;
  converged: boolean;
  /** The largest normalized residual the solver reached. Zero is a trim. */
  residual: number;
  iterations: number;

  // The members below are additions to the bead b31 contract. A caller that
  // measures a performance number needs them, and none can be derived from the
  // members above without flying the model again.

  /**
   * True airspeed of the trim, m/s.
   *
   * It repeats `condition.speed` in every mode that holds the speed. It is the
   * value the solver found in trimForAlpha and in trimMaxLevelSpeed.
   */
  speed: number;
  /** Flight path angle, rad. Positive is a climb. Level flight reports 0. */
  flightPathAngle: number;
  /** Rate of climb of this trim, m/s. It is speed times sin(flightPathAngle). */
  climbRate: number;
  /** Total thrust of the two engines at this throttle, N. */
  thrust: number;
  /** Total thrust of the two engines at full throttle in this air, N. */
  thrustAvailable: number;
  /** Wind axis drag, N. */
  drag: number;
  /** Wind axis lift, N. */
  lift: number;
  /** Lift coefficient on the wing reference area. */
  liftCoefficient: number;
  /** Total mass at this fuel load, kg. */
  mass: number;
  mach: number;
  /** True while a free value sits on a bound, which is why no trim was found. */
  atLimit: boolean;
}

// ---------------------------------------------------------------------------
// Solver constants
// ---------------------------------------------------------------------------

/**
 * The solver stops when every normalized residual is below this value.
 *
 * A hundredth of a part per million of the weight is 0.0006 N on a 62 kN
 * aircraft, and 0.001 N m of pitching moment. The solver reaches it, and it
 * usually reaches 1e-12, because the aerodynamic fixed point below makes the
 * residual a real function of the three unknowns and Newton then doubles its
 * digits at every step. A trim that does not exist stops six orders of magnitude
 * above this value, so the flag separates the two with no room for doubt.
 */
const RESIDUAL_TOLERANCE = 1e-8;

/** Largest number of Newton steps. A good condition needs four or five. */
const MAX_ITERATIONS = 40;

/** Central difference steps of the numerical Jacobian, in the unit of each value. */
const JACOBIAN_STEP = 1e-5;

/**
 * The central difference step of the speed, m/s.
 *
 * The thrust grid is bilinear in the speed, so a step of a millimeter per second
 * would read the slope of one cell and lose every digit of it to rounding. A
 * hundredth of a meter per second is small against the curvature of the drag and
 * large against the rounding of a 60 kN force.
 */
const JACOBIAN_SPEED_STEP = 0.01;

/** Halvings of the Newton step in the backtracking line search. */
const LINE_SEARCH_STEPS = 8;

/** Largest change of one value in one Newton step. */
const MAX_ANGLE_STEP = 0.15; // rad
const MAX_COMMAND_STEP = 0.5;
const MAX_SPEED_STEP = 40; // m/s

/** Bounds of the five values. A value on its bound means the trim does not exist. */
const ALPHA_MIN = -0.35; // rad
const ALPHA_MAX = 0.45; // rad
const GAMMA_LIMIT = 1.3; // rad
const SPEED_MIN = 30; // m/s
const SPEED_MAX = 400; // m/s

/** Lever positions and airspeeds of the settled thrust grid. */
const THROTTLE_KNOTS = 16;
const SPEED_KNOTS: readonly number[] = [0, 60, 120, 180, 240, 300, 360];

/**
 * Rotor speed at which the fuel cock of the start procedure opens, rpm.
 *
 * DUPLICATED from COCK_OPEN_RPM of src/aircraft/aircraft.ts, which holds it as a
 * private constant. The engine model charges a hot start to a cock that is open
 * from rest, so the settled thrust table must follow the same procedure the
 * aircraft follows.
 */
const FUEL_COCK_OPEN_RPM = 700;

/** Steps of the engine settle, and the longest settle. */
const ENGINE_STEP = 0.02; // s
const ENGINE_START_TIME = 120; // s
const ENGINE_SETTLE_TIME = 60; // s

/** The rotor is settled when it moves less than this in half a second. */
const ENGINE_SETTLE_RPM = 0.05;

/**
 * Largest rate the lever may move at while the grid is built, per second.
 *
 * THE SOLVER MUST MOVE THE LEVER LIKE A PILOT, for the same reason the flight
 * test harness must. A step of the lever at a low rotor speed drives the
 * fuel-air ratio past the surge line of the Jumo 004, the compressor stalls, and
 * the engine model charges permanent turbine damage for it. A grid built with a
 * stepped lever then reports a thrust that is 10 percent low, and every trim
 * that reads it is wrong by that much.
 *
 * Source: CONVENTIONS section 8, idle to full power in 8 to 10 s.
 */
const LEVER_RATE = 0.1; // per second

/** Surge margin at which the lever stops advancing. */
const LEVER_SURGE_GUARD = 0.15;

/** Below this speed a trim has no meaning, because the dynamic pressure is zero. */
const MIN_TRIM_SPEED = 1; // m/s


// ---------------------------------------------------------------------------
// The settled thrust table
// ---------------------------------------------------------------------------

/** True while a flame burns in the chambers. It repeats the test in engine.ts. */
function isLit(state: EngineState): boolean {
  return (
    state === 'lightOff' ||
    state === 'idle' ||
    state === 'running' ||
    state === 'stall' ||
    state === 'fire'
  );
}

/**
 * Moves the lever to `target` at the rate a pilot uses and then runs the engine
 * until its rotor speed stops moving.
 *
 * The lever holds where it is while the surge margin is thin, so the rotor gets
 * the airflow before the fuel. See LEVER_RATE.
 */
function settleEngine(
  engine: Engine,
  input: EngineInput,
  target: number,
  seconds: number,
): void {
  const steps = Math.round(seconds / ENGINE_STEP);
  const check = Math.round(0.5 / ENGINE_STEP);
  const travel = LEVER_RATE * ENGINE_STEP;
  let last = engine.rpm;
  for (let i = 0; i < steps; i++) {
    const rising = engine.surgeMargin >= LEVER_SURGE_GUARD;
    input.throttle = clamp(
      target,
      input.throttle - travel,
      rising ? input.throttle + travel : input.throttle,
    );
    // The fuel cock of the start procedure. See FUEL_COCK_OPEN_RPM.
    input.fuelCockOpen = engine.rpm >= FUEL_COCK_OPEN_RPM || isLit(engine.state);
    engine.update(input, ENGINE_STEP);
    if (i % check === check - 1) {
      if (input.throttle === target && Math.abs(engine.rpm - last) < ENGINE_SETTLE_RPM) {
        return;
      }
      last = engine.rpm;
    }
  }
}

/**
 * Builds the settled thrust of ONE engine over the lever and the airspeed, at
 * one altitude.
 *
 * The lever runs upward through the knots inside one speed row, so each settle
 * starts from the rotor speed of the knot below it and only has to cover one
 * step of the spool.
 */
function buildThrustGrid(altitude: number, sample: AtmosphereSample): Table2D {
  const lever: number[] = [];
  for (let i = 0; i < THROTTLE_KNOTS; i++) {
    lever.push(i / (THROTTLE_KNOTS - 1));
  }

  const rows: number[][] = [];
  for (const speed of SPEED_KNOTS) {
    const engine = createJumo004(ENGINE_POSITION_RIGHT);
    const input: EngineInput = {
      throttle: 0,
      fuelCockOpen: false,
      starterEngaged: false,
      altitude,
      mach: 0,
      airspeed: 0,
      density: sample.density,
      fuelAvailable: true,
    };
    const mach = machNumber(speed, sample.speedOfSound);

    const row: number[] = [];
    for (let i = 0; i < THROTTLE_KNOTS; i++) {
      // EVERY KNOT IS SET UP AT REST AND THEN FLOWN UP TO THE SPEED OF THE ROW.
      //
      // Two faults of the engine model make that order the only one that works.
      // A cold engine cannot light above RELIGHT_MAX_SPEED, which is 250 m/s, so
      // a row that cranked at its own speed would never light at all. And a lit
      // engine at a low lever and a high speed blows out LEAN, because the ram
      // gives the compressor a third more air while the lever holds the fuel: at
      // 300 m/s the fuel-air ratio at idle falls to 0.0038 against a blowout
      // limit of 0.004. That blowout is real and the grid must show it, but an
      // engine that dies at the first knot would carry a dead row into every
      // knot above it. The reset below gives each knot its own engine state, so
      // a zero in the grid means that this lever cannot hold a flame at this
      // speed, and nothing else.
      input.airspeed = 0;
      input.mach = 0;
      if (!isLit(engine.state)) {
        engine.reset();
        input.starterEngaged = true;
        input.throttle = 0;
        const startSteps = Math.round(ENGINE_START_TIME / ENGINE_STEP);
        for (let k = 0; k < startSteps; k++) {
          if (engine.state === 'idle' || engine.state === 'running') {
            break;
          }
          input.fuelCockOpen = engine.rpm >= FUEL_COCK_OPEN_RPM || isLit(engine.state);
          engine.update(input, ENGINE_STEP);
        }
        input.starterEngaged = false;
      }
      settleEngine(engine, input, lever[i], ENGINE_SETTLE_TIME);
      input.airspeed = speed;
      input.mach = mach;
      settleEngine(engine, input, lever[i], ENGINE_SETTLE_TIME);
      // A lever that gave less thrust than the lever below it would put a
      // negative slope in the Newton step. The engine model is monotone in the
      // lever, so the guard only acts where a low lever blew the flame out.
      row.push(i > 0 ? Math.max(engine.thrust, row[i - 1]) : engine.thrust);
    }
    rows.push(row);
  }
  return table2d(lever, SPEED_KNOTS as number[], rows);
}

/** The thrust grids already built, keyed by the altitude. */
const thrustGrids = new Map<number, Table2D>();

function thrustGrid(altitude: number, sample: AtmosphereSample): Table2D {
  const key = Math.round(altitude);
  const found = thrustGrids.get(key);
  if (found !== undefined) {
    return found;
  }
  const built = buildThrustGrid(altitude, sample);
  thrustGrids.set(key, built);
  return built;
}

/** Settled thrust of BOTH engines at one lever position and one airspeed, N. */
function thrustAt(grid: Table2D, throttle: number, speed: number): number {
  return 2 * lookup2d(grid, throttle, speed);
}

/**
 * Clears the settled thrust cache. A test that measures the cost of a trim needs
 * it. Nothing else does.
 */
export function clearTrimCache(): void {
  thrustGrids.clear();
}

// ---------------------------------------------------------------------------
// The model the solver evaluates
// ---------------------------------------------------------------------------


/** The five values a mode can free. The residual reads all five. */
interface TrimVariables {
  /** True airspeed, m/s. */
  speed: number;
  alpha: number;
  gamma: number;
  elevator: number;
  throttle: number;
}

type TrimVariableName = 'speed' | 'alpha' | 'gamma' | 'elevator' | 'throttle';

/** One assembly, one wrench and one state, reused by every call of this module. */
const assembly: AeroAssembly = createMe262Assembly();
const controls = new Float64Array(CONTROL_COUNT);
const state: RigidBodyState = createState();
const wrench: Wrench = createWrench();
const air: AtmosphereSample = createAtmosphereSample();
const wind = new Vector3(0, 0, 0);
const pitchAxis = new Vector3(0, 1, 0);
const forceWorld = new Vector3();
const gravityWorld = new Vector3();
const gravityBody = new Vector3();
const thrustForce = new Vector3();
const thrustMoment = new Vector3();
const stageAirspeed = new Vector3();
const gearDragForce = new Vector3();
const gearDragMoment = new Vector3();
const residual = [0, 0, 0];
const trial = [0, 0, 0];
const jacobian = [0, 0, 0, 0, 0, 0, 0, 0, 0];
const step = [0, 0, 0];

/** What one residual evaluation produced, beside the residual itself. */
interface TrimSample {
  lift: number;
  drag: number;
  thrust: number;
  dynamicPressure: number;
}

const sampleOut: TrimSample = { lift: 0, drag: 0, thrust: 0, dynamicPressure: 0 };

/** The lower and the upper bound of one free value. */
function bounds(name: TrimVariableName): readonly [number, number] {
  if (name === 'speed') {
    return [SPEED_MIN, SPEED_MAX];
  }
  if (name === 'alpha') {
    return [ALPHA_MIN, ALPHA_MAX];
  }
  if (name === 'gamma') {
    return [-GAMMA_LIMIT, GAMMA_LIMIT];
  }
  if (name === 'elevator') {
    return [-1, 1];
  }
  return [0, 1];
}

/** The largest change one Newton step may make to one free value. */
function stepLimit(name: TrimVariableName): number {
  if (name === 'speed') {
    return MAX_SPEED_STEP;
  }
  return name === 'alpha' || name === 'gamma' ? MAX_ANGLE_STEP : MAX_COMMAND_STEP;
}

/** The central difference step of one free value, in the unit of that value. */
function jacobianStep(name: TrimVariableName): number {
  return name === 'speed' ? JACOBIAN_SPEED_STEP : JACOBIAN_STEP;
}

/** Reads one value out of the variable record. */
function get(v: TrimVariables, name: TrimVariableName): number {
  if (name === 'speed') {
    return v.speed;
  }
  if (name === 'alpha') {
    return v.alpha;
  }
  if (name === 'gamma') {
    return v.gamma;
  }
  if (name === 'elevator') {
    return v.elevator;
  }
  return v.throttle;
}

/** Writes one value into the variable record. */
function set(v: TrimVariables, name: TrimVariableName, value: number): void {
  if (name === 'speed') {
    v.speed = value;
  } else if (name === 'alpha') {
    v.alpha = value;
  } else if (name === 'gamma') {
    v.gamma = value;
  } else if (name === 'elevator') {
    v.elevator = value;
  } else {
    v.throttle = value;
  }
}

/**
 * Writes the three normalized residuals of one candidate into `out`.
 *
 *   out[0]  force along the flight path, over the weight
 *   out[1]  force across the flight path less the maneuver, over the weight
 *   out[2]  pitching moment, over the weight times the mean aerodynamic chord
 *
 * The function also writes the wind axis lift, the drag and the thrust of the
 * same evaluation into `sampleOut`, because the caller reports them and they are
 * already computed.
 */
function evaluateResidual(
  v: TrimVariables,
  condition: TrimCondition,
  mass: number,
  grid: Table2D,
  out: number[],
): void {
  const speed = v.speed;
  // The default load factor is the one a straight path carries. See the note on
  // TrimCondition.loadFactor.
  const loadFactor =
    condition.loadFactor !== undefined ? condition.loadFactor : Math.cos(v.gamma);
  const weight = mass * G0;

  // The state of the maneuver. The flight path lies in the vertical plane and
  // the aircraft has no bank and no sideslip, so the pitch attitude is the sum
  // of the angle of attack and the flight path angle.
  const pitch = v.alpha + v.gamma;
  state.position.set(0, 0, -condition.altitude);
  state.velocity.set(speed * Math.cos(v.gamma), 0, -speed * Math.sin(v.gamma));
  state.orientation.setFromAxisAngle(pitchAxis, pitch);
  // The body pitch rate of the maneuver. A steady straight flight has none.
  state.angularVelocity.set(0, (G0 * (loadFactor - Math.cos(v.gamma))) / speed, 0);

  controls[CONTROL_INDEX.aileron] = 0;
  controls[CONTROL_INDEX.elevator] = v.elevator * ELEVATOR_LIMIT;
  controls[CONTROL_INDEX.rudder] = 0;
  controls[CONTROL_INDEX.flap] = flapSettingPosition(condition.flapSetting) * FLAP_LANDING_ANGLE;
  controls[CONTROL_INDEX.slat] = 0;

  // ONE CALL IS ENOUGH. BEAD b61.
  //
  // assembly.evaluateSteady drives the separation lag of every strip and the
  // downwash lag to the steady value of THIS state, and it closes the induced
  // angle against them inside the call. The wrench it returns is therefore a
  // function of the three unknowns and of nothing the solver did before it. This
  // file used to run evaluate again until the wrench stopped moving, because the
  // induced angle pass read the separation state the last call left behind. See
  // the module comment.
  clearWrench(wrench);
  const totals = assembly.evaluateSteady(state, wind, controls, wrench);

  // The drag of the landing gear, at the gear and not at the center of gravity.
  // src/aircraft/aircraft.ts applies the same force in the same way, and the two
  // must agree or a trim with the gear down is a trim of another aircraft.
  const gearDragArea = gearDragAreaAt(condition.gearDown ? 1 : 0);
  if (gearDragArea > 0 && totals.trueAirspeed > MIN_GEAR_DRAG_SPEED) {
    airspeedBody(state, wind, stageAirspeed);
    gearDragForce
      .copy(stageAirspeed)
      .multiplyScalar((-totals.dynamicPressure * gearDragArea) / totals.trueAirspeed);
    wrench.force.add(gearDragForce);
    gearDragMoment.crossVectors(ME262_GEAR_DRAG_POSITION, gearDragForce);
    wrench.moment.add(gearDragMoment);
  }

  // Thrust. Each engine pushes along body x at its own position, so the moment
  // of the low thrust line appears with no special case.
  const total = thrustAt(grid, v.throttle, speed);
  thrustForce.set(0.5 * total, 0, 0);
  wrench.force.x += total;
  thrustMoment.crossVectors(ENGINE_POSITION_LEFT, thrustForce);
  wrench.moment.add(thrustMoment);
  thrustMoment.crossVectors(ENGINE_POSITION_RIGHT, thrustForce);
  wrench.moment.add(thrustMoment);

  // Gravity, in body axes at the attitude of this candidate.
  gravityWorld.set(0, 0, weight);
  worldToBody(state.orientation, gravityWorld, gravityBody);
  wrench.force.add(gravityBody);

  bodyToWorld(state.orientation, wrench.force, forceWorld);

  // The two axes of the vertical plane. The path axis runs along the velocity
  // and the other one runs across it, upward.
  const cg = Math.cos(v.gamma);
  const sg = Math.sin(v.gamma);
  const alongPath = forceWorld.x * cg - forceWorld.z * sg;
  const acrossPath = -forceWorld.x * sg - forceWorld.z * cg;

  out[0] = alongPath / weight;
  out[1] = (acrossPath - weight * (loadFactor - cg)) / weight;
  out[2] = wrench.moment.y / (weight * MAC);

  sampleOut.lift = totals.lift;
  sampleOut.drag = totals.drag;
  sampleOut.thrust = total;
  sampleOut.dynamicPressure = totals.dynamicPressure;
}

/**
 * The merit of a residual vector, which is the square of its Euclidean length.
 *
 * The line search works on this value and not on the largest single residual.
 * The largest single residual lets the solver trade a smaller pitching moment
 * for a larger force error and call that an improvement, which walks the search
 * away from the answer where no trim exists.
 */
function merit(r: readonly number[]): number {
  return r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
}

/** The largest absolute value of a residual vector. */
function norm(r: readonly number[]): number {
  return Math.max(Math.abs(r[0]), Math.abs(r[1]), Math.abs(r[2]));
}

/**
 * Solves a 3 by 3 system with Gauss elimination and partial pivoting. The
 * matrix is row major. It returns false when the matrix is singular, which is
 * what a stalled wing or a lost elevator looks like.
 */
function solve3(a: readonly number[], b: readonly number[], out: number[]): boolean {
  // One augmented matrix of three rows and four columns. The fourth column is
  // the right hand side, so a row swap carries it with the row.
  const m = [
    a[0], a[1], a[2], b[0],
    a[3], a[4], a[5], b[1],
    a[6], a[7], a[8], b[2],
  ];
  for (let column = 0; column < 3; column++) {
    let pivot = column;
    let best = Math.abs(m[column * 4 + column]);
    for (let row = column + 1; row < 3; row++) {
      const value = Math.abs(m[row * 4 + column]);
      if (value > best) {
        best = value;
        pivot = row;
      }
    }
    if (best < 1e-14) {
      return false;
    }
    if (pivot !== column) {
      for (let k = 0; k < 4; k++) {
        const swap = m[column * 4 + k];
        m[column * 4 + k] = m[pivot * 4 + k];
        m[pivot * 4 + k] = swap;
      }
    }
    for (let row = column + 1; row < 3; row++) {
      const factor = m[row * 4 + column] / m[column * 4 + column];
      if (factor === 0) {
        continue;
      }
      for (let k = column; k < 4; k++) {
        m[row * 4 + k] -= factor * m[column * 4 + k];
      }
    }
  }
  for (let row = 2; row >= 0; row--) {
    let sum = m[row * 4 + 3];
    for (let k = row + 1; k < 3; k++) {
      sum -= m[row * 4 + k] * out[k];
    }
    out[row] = sum / m[row * 4 + row];
  }
  return true;
}

/**
 * The Newton core. It drives the three residuals to zero over the three free
 * values in `free` and leaves every other value where the caller put it.
 */
function newton(
  condition: TrimCondition,
  free: readonly TrimVariableName[],
  start: TrimVariables,
): TrimResult {
  const fuel = Math.max(0, condition.fuelMass);
  const mass = me262Mass(fuel).mass;
  isa(condition.altitude, air);
  const grid = thrustGrid(condition.altitude, air);

  const v: TrimVariables = { ...start };
  v.speed = Math.max(v.speed, MIN_TRIM_SPEED);
  evaluateResidual(v, condition, mass, grid, residual);
  let error = merit(residual);
  let lift = sampleOut.lift;
  let drag = sampleOut.drag;
  let thrust = sampleOut.thrust;
  let pressure = sampleOut.dynamicPressure;
  let iterations = 0;

  for (let iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
    if (norm(residual) <= RESIDUAL_TOLERANCE) {
      break;
    }
    iterations = iteration + 1;

    // The Jacobian, by central differences. Six residual evaluations.
    let singular = false;
    for (let column = 0; column < 3; column++) {
      const name = free[column];
      const base = get(v, name);
      const [lo, hi] = bounds(name);
      // The step stays inside the bounds, so a value that sits on a bound still
      // reports the one sided slope that points back into the range.
      const h = Math.min(jacobianStep(name), 0.5 * (hi - lo));
      const up = Math.min(hi, base + h);
      const down = Math.max(lo, base - h);
      if (up - down < 1e-12) {
        singular = true;
        break;
      }
      set(v, name, up);
      evaluateResidual(v, condition, mass, grid, trial);
      const r0 = trial[0];
      const r1 = trial[1];
      const r2 = trial[2];
      set(v, name, down);
      evaluateResidual(v, condition, mass, grid, trial);
      const scale = 1 / (up - down);
      jacobian[0 * 3 + column] = (r0 - trial[0]) * scale;
      jacobian[1 * 3 + column] = (r1 - trial[1]) * scale;
      jacobian[2 * 3 + column] = (r2 - trial[2]) * scale;
      set(v, name, base);
    }
    if (singular) {
      break;
    }
    const base: TrimVariables = { ...v };

    /**
     * Takes the longest step along `direction` that lowers the merit, and
     * reports whether it found one.
     *
     * The step limit holds the first step of a poor first guess inside the range
     * where the model is still smooth. The halvings after it are the line
     * search. A full step is right where the model is linear, and near the stall
     * and near the Mach limit it is not.
     */
    const tryDirection = (direction: readonly number[]): boolean => {
      let scale = 1;
      for (let i = 0; i < 3; i++) {
        const limit = stepLimit(free[i]);
        if (Math.abs(direction[i]) > limit) {
          scale = Math.min(scale, limit / Math.abs(direction[i]));
        }
      }
      for (let attempt = 0; attempt < LINE_SEARCH_STEPS; attempt++) {
        const lambda = scale * Math.pow(0.5, attempt);
        let moved = false;
        for (let i = 0; i < 3; i++) {
          const name = free[i];
          const [lo, hi] = bounds(name);
          const next = clamp(get(base, name) + lambda * direction[i], lo, hi);
          if (next !== get(base, name)) {
            moved = true;
          }
          set(v, name, next);
        }
        if (!moved) {
          continue;
        }
        evaluateResidual(v, condition, mass, grid, trial);
        const next = merit(trial);
        if (next < error) {
          residual[0] = trial[0];
          residual[1] = trial[1];
          residual[2] = trial[2];
          error = next;
          lift = sampleOut.lift;
          drag = sampleOut.drag;
          thrust = sampleOut.thrust;
          pressure = sampleOut.dynamicPressure;
          return true;
        }
      }
      for (const name of free) {
        set(v, name, get(base, name));
      }
      return false;
    };

    let accepted = false;
    if (solve3(jacobian, [-residual[0], -residual[1], -residual[2]], step)) {
      accepted = tryDirection(step);
    }
    if (!accepted) {
      // THE FALLBACK. A Newton step needs a Jacobian that is not nearly
      // singular. At the stall the lift stops answering the angle of attack, and
      // at the Mach limit the moment stops answering the elevator, so the matrix
      // loses a rank and the Newton direction points almost anywhere.
      //
      // The steepest descent direction of the merit cannot do that. It is
      // -J^T r, which is a descent direction for every J that is not exactly
      // zero. The length below is the Cauchy point, which is the exact minimum
      // along that direction of the linear model. The step is slower than a
      // Newton step and it never fails, so the solver walks the last part of a
      // hard condition and stops on the bound that really blocks the trim.
      const gradient = [0, 0, 0];
      for (let i = 0; i < 3; i++) {
        gradient[i] = -(
          jacobian[0 * 3 + i] * residual[0] +
          jacobian[1 * 3 + i] * residual[1] +
          jacobian[2 * 3 + i] * residual[2]
        );
      }
      let gg = 0;
      for (let i = 0; i < 3; i++) {
        gg += gradient[i] * gradient[i];
      }
      let jgg = 0;
      for (let row = 0; row < 3; row++) {
        const value =
          jacobian[row * 3 + 0] * gradient[0] +
          jacobian[row * 3 + 1] * gradient[1] +
          jacobian[row * 3 + 2] * gradient[2];
        jgg += value * value;
      }
      if (gg > 0 && jgg > 0) {
        const cauchy = gg / jgg;
        for (let i = 0; i < 3; i++) {
          gradient[i] *= cauchy;
        }
        accepted = tryDirection(gradient);
      }
    }
    if (!accepted) {
      // Nothing goes downhill. The candidate is the best this solver reaches,
      // and the residual reports how far from a trim it is.
      evaluateResidual(v, condition, mass, grid, residual);
      break;
    }
  }

  let atLimit = false;
  for (const name of free) {
    const [lo, hi] = bounds(name);
    const value = get(v, name);
    if (value <= lo + 1e-9 || value >= hi - 1e-9) {
      atLimit = true;
    }
  }

  const reached = norm(residual);
  return {
    speed: v.speed,
    elevator: v.elevator,
    throttle: v.throttle,
    alpha: v.alpha,
    pitch: v.alpha + v.gamma,
    converged: reached <= RESIDUAL_TOLERANCE,
    residual: reached,
    iterations,
    flightPathAngle: v.gamma,
    climbRate: v.speed * Math.sin(v.gamma),
    thrust,
    thrustAvailable: thrustAt(grid, 1, v.speed),
    drag,
    lift,
    liftCoefficient: pressure > 0 ? lift / (pressure * WING_AREA) : 0,
    mass,
    mach: machNumber(v.speed, air.speedOfSound),
    atLimit,
  };
}

/**
 * A first guess that costs nothing.
 *
 * The angle of attack comes from the lift equation with the lift curve slope of
 * the whole aircraft. It does not have to be right. It only has to put the first
 * Newton step inside the range where the model is smooth.
 */
function firstGuess(condition: TrimCondition, mass: number, density: number): TrimVariables {
  // The first guess starts level, so the straight path load factor is 1 there.
  const loadFactor = condition.loadFactor !== undefined ? condition.loadFactor : 1;
  const speed = Math.max(condition.speed, MIN_TRIM_SPEED);
  const q = 0.5 * density * speed * speed;
  const cl = q > 0 ? (loadFactor * mass * G0) / (q * WING_AREA) : 0;
  // 4.8 per radian is the finite span slope of the wing, and the zero lift angle
  // of this wing at its rigging incidence is near -1.5 degrees.
  const alpha = clamp(cl / 4.8 - 0.026, ALPHA_MIN, ALPHA_MAX);
  return { speed, alpha, gamma: 0, elevator: 0, throttle: 0.7 };
}

// ---------------------------------------------------------------------------
// The four modes
// ---------------------------------------------------------------------------

/**
 * Solves the angle of attack, the elevator and the throttle that hold a level
 * flight path at the speed of the condition.
 */
export function trimLevelFlight(condition: TrimCondition): TrimResult {
  isa(condition.altitude, air);
  const mass = me262Mass(Math.max(0, condition.fuelMass)).mass;
  const start = firstGuess(condition, mass, air.density);
  return newton(condition, ['alpha', 'elevator', 'throttle'], start);
}

/**
 * Solves the speed, the elevator and the throttle that hold a level flight path
 * at a fixed angle of attack. `condition.speed` is the first guess.
 *
 * A search over the angle of attack for the LOWEST speed this function reports
 * is the trimmed stall speed of the aircraft, with the tail load included.
 */
export function trimForAlpha(condition: TrimCondition, alpha: number): TrimResult {
  isa(condition.altitude, air);
  const mass = me262Mass(Math.max(0, condition.fuelMass)).mass;
  const start = firstGuess(condition, mass, air.density);
  start.alpha = clamp(alpha, ALPHA_MIN, ALPHA_MAX);
  return newton(condition, ['speed', 'elevator', 'throttle'], start);
}

/**
 * Solves the angle of attack, the flight path angle and the elevator at a fixed
 * throttle and a fixed speed. The result is the steady climb the aircraft holds
 * at that lever position, with the thrust component along the path and the lower
 * lift of a climb both included.
 *
 * This is the rate of climb and the service ceiling.
 */
export function trimSteadyClimb(condition: TrimCondition, throttle: number): TrimResult {
  isa(condition.altitude, air);
  const mass = me262Mass(Math.max(0, condition.fuelMass)).mass;
  const start = firstGuess(condition, mass, air.density);
  start.throttle = clamp(throttle, 0, 1);
  // A first guess of the climb angle from the excess thrust. The Newton step
  // needs only the sign and the size of it.
  const grid = thrustGrid(condition.altitude, air);
  const thrust = thrustAt(grid, start.throttle, start.speed);
  start.gamma = clamp((thrust - 0.2 * mass * G0) / (mass * G0), -0.5, 0.5);
  return newton(condition, ['alpha', 'gamma', 'elevator'], start);
}

/**
 * Solves the speed, the angle of attack and the elevator of level flight at a
 * full throttle. The speed it reports is the maximum level speed at this
 * altitude and this mass. `condition.speed` is the first guess.
 */
export function trimMaxLevelSpeed(condition: TrimCondition): TrimResult {
  isa(condition.altitude, air);
  const mass = me262Mass(Math.max(0, condition.fuelMass)).mass;
  const start = firstGuess(condition, mass, air.density);
  start.throttle = 1;
  return newton(condition, ['speed', 'alpha', 'elevator'], start);
}

/**
 * Evaluates the three residuals at one candidate, with no solve at all.
 *
 * Bead b33 and the flight tests read it to see the pitching moment, the lift and
 * the drag of a state the solver never has to reach: a dive past the Mach limit,
 * a wing past the stall, or an elevator at its stop. `speed` and `alpha` come
 * from the arguments, and the flight path angle is level.
 */
export function trimResiduals(
  condition: TrimCondition,
  values: { speed: number; alpha: number; elevator: number; throttle: number; gamma?: number },
): {
  alongPath: number;
  acrossPath: number;
  moment: number;
  lift: number;
  drag: number;
  thrust: number;
  pitchingMoment: number;
  momentCoefficient: number;
  liftCoefficient: number;
} {
  isa(condition.altitude, air);
  const mass = me262Mass(Math.max(0, condition.fuelMass)).mass;
  const grid = thrustGrid(condition.altitude, air);
  const v: TrimVariables = {
    speed: Math.max(values.speed, MIN_TRIM_SPEED),
    alpha: values.alpha,
    gamma: values.gamma !== undefined ? values.gamma : 0,
    elevator: values.elevator,
    throttle: values.throttle,
  };
  evaluateResidual(v, condition, mass, grid, trial);
  const q = sampleOut.dynamicPressure;
  return {
    alongPath: trial[0],
    acrossPath: trial[1],
    moment: trial[2],
    lift: sampleOut.lift,
    drag: sampleOut.drag,
    thrust: sampleOut.thrust,
    pitchingMoment: trial[2] * mass * G0 * MAC,
    momentCoefficient: q > 0 ? (trial[2] * mass * G0 * MAC) / (q * WING_AREA * MAC) : 0,
    liftCoefficient: q > 0 ? sampleOut.lift / (q * WING_AREA) : 0,
  };
}
