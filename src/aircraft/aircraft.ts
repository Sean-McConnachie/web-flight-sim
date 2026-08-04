/**
 * The whole Messerschmitt Me 262 A-1a, as one object.
 *
 * Nine modules already hold the physics. This file holds none of it. It joins
 * the aerodynamic assembly, the two engines, the landing gear, the systems and
 * the mass model onto one rigid body, and it steps that body.
 *
 *
 * 1. THE FORCE SUM
 *
 * Every step builds ONE wrench in BODY axes, about the center of gravity, in
 * this order:
 *
 *   1. clear
 *   2. aerodynamics   src/physics/aero/assembly.ts, which ADDS into the wrench
 *   3. thrust         each engine along body +x, AT its own nacelle position
 *   4. landing gear   src/physics/gear.ts, which ADDS into the wrench
 *   5. gravity        m * G0 on the world z axis, rotated into body axes
 *
 * GRAVITY. stepRK4 of src/physics/rigidbody.ts applies nothing on its own, so
 * this file adds it, one time, inside the wrench source. It is added in the
 * source and not before, because the rotation into body axes must use the
 * orientation of the STAGE that the integrator is evaluating.
 *
 * THRUST. The force of one engine goes in at the position of that engine, so
 * r x F gives the engine out yaw moment with no special case. An engine at
 * y = 2.05 m at 8800 N makes 18 kN m of yaw, and nothing in this file says so.
 *
 *
 * 2. WHAT RUNS ONE TIME PER STEP AND WHAT RUNS FOUR TIMES
 *
 * stepRK4 calls the wrench source four times, at four different stage states.
 * A model that carries internal state cannot run four times, because the four
 * calls are not four steps in time.
 *
 *   engines      ONE time, before the step. The rotor is a state variable, and
 *                the thrust is then constant over the four stages.
 *   landing gear ONE time, before the step. The wheel spin, the brake heat and
 *                the tire slip are all state. gear.ts says the strut, the tire
 *                and the friction are pure functions of the state, so the same
 *                wrench is correct in every stage.
 *   systems      ONE time, before the step. Flap, gear and slat travel.
 *   aerodynamics FOUR times, which is what the assembly is built for. The
 *                induced angle is solved in closed form inside each call, so
 *                evaluate is a pure function of the state it receives.
 *
 * THE SEPARATION LAG. evaluateSurface carries one first order lag per strip and
 * integrates it with the dt it receives. Four calls would run that lag four
 * times as fast. This file therefore passes the real dt on the FIRST stage and
 * zero on the other three. updateSeparation of src/physics/aero/stall.ts leaves
 * its state alone at dt = 0, so the lag advances exactly one step per step and
 * all four stages then read the same separation state.
 *
 *
 * 3. ALLOCATION
 *
 * fixedUpdate allocates nothing. Every scratch vector, the wrench, the engine
 * input record and the mass state all live in the closure of createAircraft.
 *
 * The one exception is me262Mass, which builds a fresh Matrix3 on every call.
 * The mass properties are therefore rebuilt only when the fuel on board has
 * moved by MASS_UPDATE_FUEL kilograms, which is about every three seconds at
 * full power and never with the engines off. The result is copied into the
 * mass state that this file owns, so every object a caller reads keeps its
 * identity for the life of the aircraft.
 *
 *
 * 4. THE SEPARATION RULE
 *
 * CONVENTIONS section 4. This file is physics. It imports no renderer, no DOM
 * and no browser API, and it does not import src/input. The pilot command
 * arrives as AircraftInput, which is the subset of ControlInput of
 * src/input/bindings.ts that the flight model reads. src/main.ts does the
 * wiring, and a ControlInput satisfies AircraftInput with no conversion.
 */

import { Matrix3, Quaternion, Vector3 } from 'three';

import { clamp } from '@/math/tables';
import { G0 } from '@/math/units';
import type { AtmosphereSample } from '@/physics/atmosphere';
import {
  createAtmosphereSample,
  equivalentAirspeed,
  isa,
  machNumber,
} from '@/physics/atmosphere';
import type { AeroAssembly, AeroTotals } from '@/physics/aero/assembly';
import type { LandingGear } from '@/physics/gear';
import { ME262_STATIC_CG_HEIGHT, createMe262Gear } from '@/physics/gear';
import type { FlowAngles, MassProperties, RigidBodyState, Wrench } from '@/physics/rigidbody';
import {
  addWrench,
  airspeedBody,
  clearWrench,
  createMassProperties,
  createState,
  createWrench,
  flowAngles,
  stepRK4,
  worldToBody,
} from '@/physics/rigidbody';
import type { Engine, EngineInput, EngineState } from '@/aircraft/me262/engine';
import { createJumo004 } from '@/aircraft/me262/engine';
import {
  CONTROL_COUNT,
  CONTROL_INDEX,
  ENGINE_POSITION_LEFT,
  ENGINE_POSITION_RIGHT,
  createMe262Assembly,
} from '@/aircraft/me262/geometry';
import type { MassState } from '@/aircraft/me262/mass';
import { FUEL_CAPACITY, me262Mass } from '@/aircraft/me262/mass';
import type { FlapSetting, Me262Systems } from '@/aircraft/me262/systems';
import { createMe262Systems } from '@/aircraft/me262/systems';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/**
 * Threshold of the runway and heading of the takeoff run.
 *
 * DUPLICATED from RUNWAY_THRESHOLD_NED and RUNWAY_HEADING_RAD of
 * src/world/runway.ts. That file draws the pavement and imports three/webgpu,
 * and CONVENTIONS section 4 stops src/aircraft from importing the renderer, so
 * the two numbers appear here as well. Both are zero, because CONVENTIONS
 * section 3.2 puts the NED origin on the threshold and the run points north.
 */
const RUNWAY_THRESHOLD_NORTH = 0; // m
const RUNWAY_THRESHOLD_EAST = 0; // m
const RUNWAY_HEADING = 0; // rad

/**
 * Control travel limits, in radians.
 *
 * DUPLICATED from ME262_POSE of src/render/models/me262.ts, for the same reason
 * the runway numbers are duplicated above. The surface that moves on the screen
 * must be the surface that makes the moment, so the two files must agree.
 * Confidence: estimate, from a three view. Bead b33 tunes them against the
 * published roll rate and the stick force per g.
 */
export const AILERON_LIMIT = 0.35; // rad, 20.1 deg
export const ELEVATOR_LIMIT = 0.44; // rad, 25.2 deg
export const RUDDER_LIMIT = 0.44; // rad, 25.2 deg

/**
 * Fuel change that forces a rebuild of the mass properties, kg.
 *
 * me262Mass allocates, so it cannot run inside the step. Two engines at full
 * power burn 0.71 kg/s, so this value rebuilds the tensor about every three
 * seconds. The mass error between two rebuilds is 2 kg out of 6396, which is
 * 0.03 percent.
 */
const MASS_UPDATE_FUEL = 2; // kg

/**
 * Wing strips whose local angle of attack drives the slat MECHANISM.
 *
 * me262Surfaces of src/aircraft/me262/geometry.ts puts the left wing at 0 to 7
 * and the right wing at 8 to 15, root first. Strip 5 sits at 5.49 m from the
 * plane of symmetry, which is inside the 3.00 m to 6.02 m slat span. The
 * aerodynamic slat of each strip opens on its own local angle in surface.ts.
 * This pair only drives the position that the render model and the gauge show.
 */
const SLAT_ALPHA_LEFT_INDEX = 5;
const SLAT_ALPHA_RIGHT_INDEX = 13;

/**
 * Airspeed below which the slat mechanism reads no angle of attack, m/s.
 *
 * SurfaceResult.alpha is atan2 of the two components of the local flow. At rest
 * both components are near zero, so the value is numerical noise: a parked
 * aircraft reports 91 degrees on every strip, because the tiny settling motion
 * of the struts is the whole flow. Without this gate the mechanism reads that
 * noise and runs the slats out while the aircraft stands on the runway.
 *
 * The value sits far below any speed at which the slat can matter. The slat of
 * the Me 262 opens near a lift coefficient of 0.68, which is 66 m/s at the
 * loaded mass, so nothing is lost.
 */
const MIN_SLAT_SPEED = 10; // m/s

/**
 * Rotor speed at which the fuel cock of one engine opens, rpm.
 *
 * THE FUEL COCK IS PART OF THE START PROCEDURE, NOT OF THE ENGINE. The pilot
 * notes say to crank with the Riedel starter first and to open the fuel cock
 * only once the rotor turns. A cock that is open from rest pools unburned fuel
 * in the six chambers, lights it all at LIGHT_OFF_MIN_RPM where the airflow is
 * almost nothing, and makes a HOT START. src/aircraft/me262/engine.ts models
 * that fault, and it charges permanent creep damage for it. This value is the
 * procedure, and it is why the model never sees the fault.
 *
 * Source: "Pilot's Handbook for Me-262 A-1", starting procedure, and
 * STARTER_TARGET_RPM of src/aircraft/me262/engine.ts. Confidence: firm on the
 * procedure, estimate on the value.
 */
const COCK_OPEN_RPM = 700;

/** The three flap settings, in the order the flap lever runs through them. */
const FLAP_ORDER: readonly FlapSetting[] = ['up', 'takeoff', 'landing'];

/** The world down axis. The spawn heading turns about it. */
const DOWN_AXIS = new Vector3(0, 0, 1);

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/**
 * What the flight model reads from the pilot.
 *
 * The shape is a subset of ControlInput of src/input/bindings.ts, field for
 * field, so a ControlInput passes straight in. This file cannot import that
 * type, because CONVENTIONS section 4 stops src/aircraft from importing
 * src/input. Keep the two shapes together by hand.
 */
export interface AircraftInput {
  /** -1 to 1. Positive rolls right. */
  roll: number;
  /** -1 to 1. Positive pitches the nose UP. */
  pitch: number;
  /** -1 to 1. Positive yaws the nose right. It also steers the nose wheel. */
  yaw: number;
  /** 0 to 1, both engines. */
  throttle: number;
  /** 0 to 1. */
  brakeLeft: number;
  /** 0 to 1. */
  brakeRight: number;
  /** True for one step. It toggles the landing gear. */
  toggleGear: boolean;
  /** True for one step. It moves the flap lever one setting up. */
  toggleFlapsUp: boolean;
  /** True for one step. It moves the flap lever one setting down. */
  toggleFlapsDown: boolean;
  /** HELD, not an edge. It engages the starter of both engines. */
  startEngines: boolean;
}

export interface AircraftState {
  body: RigidBodyState;
  totals: AeroTotals;
  engines: readonly Engine[];
  systems: Me262Systems;
  mass: MassState;
  onGround: boolean;
  /** Body z specific force over standard gravity. Level flight reads 1. */
  loadFactor: number;
  // The two members below are additions to the bead b25 contract. The render
  // model needs the wheel spin and the strut stroke, and neither can be derived
  // from the members above.
  gear: LandingGear;
}

export interface Aircraft {
  readonly state: AircraftState;
  /** Runs one physics step. `dt` is always PHYSICS_DT. */
  fixedUpdate(input: AircraftInput, dt: number): void;
  /** Puts the aircraft on the threshold, at rest, engines off, gear down. */
  spawnOnRunway(): void;
  /** The deflection array that the assembly reads. CONTROL_INDEX names it. */
  readonly controls: Float64Array;

  // The members below are additions to the bead b25 contract. The debug view
  // and the force arrows read them, and none of them can be derived from the
  // state above on its own.

  /** The aerodynamic assembly. sampleForDebug of it feeds the force arrows. */
  readonly assembly: AeroAssembly;
  /** The air the aircraft flies in now. The debug overlay reads it. */
  readonly atmosphere: AtmosphereSample;
  /** Equivalent airspeed, m/s. The overlay and the systems limits read it. */
  readonly equivalentAirspeed: number;
  /**
   * The total body wrench of the FIRST Runge-Kutta stage of the last step,
   * gravity included. It is the force and the moment at the state the caller
   * can see, which is what a test and a debug view both want.
   */
  readonly wrench: Wrench;
  /** Simulated time since the last spawn, s. */
  readonly time: number;
}

// ---------------------------------------------------------------------------
// The aircraft
// ---------------------------------------------------------------------------

/** True while a flame burns in the chambers of one engine. */
function isLit(state: EngineState): boolean {
  return (
    state === 'lightOff' ||
    state === 'idle' ||
    state === 'running' ||
    state === 'stall' ||
    state === 'fire'
  );
}

export function createAircraft(): Aircraft {
  const assembly = createMe262Assembly();
  const gear = createMe262Gear();
  const systems = createMe262Systems();
  const engines: readonly Engine[] = [
    createJumo004(ENGINE_POSITION_LEFT),
    createJumo004(ENGINE_POSITION_RIGHT),
  ];

  const body: RigidBodyState = createState();
  const controls = new Float64Array(CONTROL_COUNT);

  // The mass state and the mass properties both live for the life of the
  // aircraft. writeMass copies into them, so their identity never changes.
  const massState: MassState = {
    mass: 0,
    cgFromNose: 0,
    inertia: new Matrix3(),
    fuelMass: 0,
  };
  const massProperties: MassProperties = createMassProperties(
    1,
    new Matrix3().set(1, 0, 0, 0, 1, 0, 0, 0, 1),
  );
  /** The fuel the mass properties were last built at, kg. */
  let massFuel = Number.NEGATIVE_INFINITY;

  /**
   * The totals this file owns. assembly.evaluate returns its own object, and
   * that object holds the values of the LAST stage the integrator ran. The
   * numbers below are copied on the FIRST stage instead, so they describe the
   * state the caller can see. perSurface points at the live strip results, so
   * the force arrows and the debug view see the last stage there.
   */
  const totals: AeroTotals = {
    alpha: 0,
    beta: 0,
    mach: 0,
    dynamicPressure: 0,
    trueAirspeed: 0,
    lift: 0,
    drag: 0,
    sideForce: 0,
    perSurface: assembly.surfaces.map((s) => s.result),
  };

  const state: AircraftState = {
    body,
    totals,
    engines,
    systems,
    mass: massState,
    onGround: false,
    loadFactor: 1,
    gear,
  };

  // --- Scratch. fixedUpdate allocates nothing. ---
  const atmosphere: AtmosphereSample = createAtmosphereSample();
  const flow: FlowAngles = { alpha: 0, beta: 0, speed: 0 };
  const airspeed = new Vector3();
  const wind = new Vector3(0, 0, 0);
  const gearWrench: Wrench = createWrench();
  const stepWrench: Wrench = createWrench();
  const thrustForce = new Vector3();
  const thrustMoment = new Vector3();
  const gravityWorld = new Vector3();
  const gravityBody = new Vector3();
  const spawnHeading = new Quaternion();
  const engineInput: EngineInput = {
    throttle: 0,
    fuelCockOpen: true,
    starterEngaged: false,
    altitude: 0,
    mach: 0,
    airspeed: 0,
    density: 0,
    fuelAvailable: true,
  };

  let simTime = 0;
  let stepDt = 0;
  /** True while the integrator runs its first stage of the current step. */
  let firstStage = false;
  let eas = 0;
  let gearDown = true;
  let flapIndex = 0;

  /** Rebuilds the mass state and the mass properties at one fuel load. */
  function writeMass(fuel: number): void {
    const m = me262Mass(fuel);
    massState.mass = m.mass;
    massState.cgFromNose = m.cgFromNose;
    massState.inertia.copy(m.inertia);
    massState.fuelMass = m.fuelMass;
    massProperties.mass = m.mass;
    massProperties.inertia.copy(m.inertia);
    massProperties.inverseInertia.copy(m.inertia).invert();
    massFuel = fuel;
  }

  /**
   * The wrench source. stepRK4 calls it four times per step, at four stage
   * states. Read section 1 and section 2 of the module comment before you
   * change the order of the five blocks below.
   */
  function source(stage: RigidBodyState, _time: number, out: Wrench): void {
    // 1. The wrench arrives cleared. stepRK4 clears it before every stage.

    // 2. Aerodynamics. The assembly ADDS into out and returns the totals. Only
    // the first stage advances the separation lag. See section 2.
    const stageTotals = assembly.evaluate(stage, wind, controls, firstStage ? stepDt : 0, out);

    // 3. Thrust. Each engine pushes along body +x at its own position, so the
    // r x F below is the whole engine out yaw moment.
    for (let i = 0; i < engines.length; i++) {
      const engine = engines[i];
      thrustForce.set(engine.thrust, 0, 0);
      out.force.add(thrustForce);
      thrustMoment.crossVectors(engine.position, thrustForce);
      out.moment.add(thrustMoment);
    }

    // 4. The landing gear. The wrench was built one time before the step.
    addWrench(out, gearWrench);

    // 5. Gravity. stepRK4 adds none of its own. The rotation must use the
    // orientation of THIS stage, which is why gravity sits here and not in
    // fixedUpdate.
    gravityWorld.set(0, 0, massProperties.mass * G0);
    worldToBody(stage.orientation, gravityWorld, gravityBody);
    out.force.add(gravityBody);

    if (firstStage) {
      totals.alpha = stageTotals.alpha;
      totals.beta = stageTotals.beta;
      totals.mach = stageTotals.mach;
      totals.dynamicPressure = stageTotals.dynamicPressure;
      totals.trueAirspeed = stageTotals.trueAirspeed;
      totals.lift = stageTotals.lift;
      totals.drag = stageTotals.drag;
      totals.sideForce = stageTotals.sideForce;
      stepWrench.force.copy(out.force);
      stepWrench.moment.copy(out.moment);
      // The load factor is what an accelerometer reads, so it is the specific
      // force with GRAVITY TAKEN OUT. Level flight reads 1 and free fall
      // reads 0.
      state.loadFactor = -(out.force.z - gravityBody.z) / (massProperties.mass * G0);
      firstStage = false;
    }
  }

  const api: Aircraft = {
    state,
    controls,
    assembly,
    atmosphere,
    wrench: stepWrench,

    get equivalentAirspeed(): number {
      return eas;
    },

    get time(): number {
      return simTime;
    },

    spawnOnRunway(): void {
      // CONVENTIONS section 3.2. The threshold is the NED origin, at ground
      // level, and the center of gravity stands ME262_STATIC_CG_HEIGHT above
      // it once the struts carry the weight. Altitude is minus the world z.
      body.position.set(
        RUNWAY_THRESHOLD_NORTH,
        RUNWAY_THRESHOLD_EAST,
        -ME262_STATIC_CG_HEIGHT,
      );
      body.velocity.set(0, 0, 0);
      body.angularVelocity.set(0, 0, 0);
      spawnHeading.setFromAxisAngle(DOWN_AXIS, RUNWAY_HEADING);
      body.orientation.copy(spawnHeading);

      gear.reset();
      for (const engine of engines) {
        engine.reset();
      }

      // The systems drive every position at a rate, so a spawn writes the
      // position and the command together. Gear down, flaps up, slats shut.
      gearDown = true;
      flapIndex = 0;
      systems.commandGear(true);
      systems.commandFlaps('up');
      systems.setBrakes(0, 0);
      systems.state.gearPosition = 1;
      systems.state.flapPosition = 0;
      systems.state.slatPosition = 0;
      systems.state.brakeLeft = 0;
      systems.state.brakeRight = 0;
      systems.state.fuelMass = FUEL_CAPACITY;

      controls.fill(0);
      writeMass(FUEL_CAPACITY);
      clearWrench(stepWrench);
      clearWrench(gearWrench);
      state.onGround = true;
      state.loadFactor = 1;
      totals.alpha = 0;
      totals.beta = 0;
      totals.mach = 0;
      totals.dynamicPressure = 0;
      totals.trueAirspeed = 0;
      totals.lift = 0;
      totals.drag = 0;
      totals.sideForce = 0;
      eas = 0;
      simTime = 0;
      isa(ME262_STATIC_CG_HEIGHT, atmosphere);
    },

    fixedUpdate(input: AircraftInput, dt: number): void {
      if (!(dt > 0)) {
        return;
      }
      stepDt = dt;

      // --- The pilot commands the systems --------------------------------
      if (input.toggleGear) {
        gearDown = !gearDown;
        systems.commandGear(gearDown);
      }
      if (input.toggleFlapsDown && flapIndex < FLAP_ORDER.length - 1) {
        flapIndex += 1;
        systems.commandFlaps(FLAP_ORDER[flapIndex]);
      }
      if (input.toggleFlapsUp && flapIndex > 0) {
        flapIndex -= 1;
        systems.commandFlaps(FLAP_ORDER[flapIndex]);
      }
      systems.setBrakes(input.brakeLeft, input.brakeRight);

      // --- The air the aircraft flies in ---------------------------------
      const altitude = -body.position.z;
      isa(altitude, atmosphere);
      airspeedBody(body, wind, airspeed);
      flowAngles(airspeed, flow);
      const trueAirspeed = flow.speed;
      eas = equivalentAirspeed(trueAirspeed, atmosphere.density);
      const mach = machNumber(trueAirspeed, atmosphere.speedOfSound);

      // --- The systems ----------------------------------------------------
      // The slat MECHANISM follows the local angle of the outer wing. The two
      // strips report the value of the last step, which is one step of lag on
      // a mechanism that takes half a second to run out.
      const outerAlpha =
        trueAirspeed > MIN_SLAT_SPEED
          ? 0.5 *
            (assembly.surfaces[SLAT_ALPHA_LEFT_INDEX].result.alpha +
              assembly.surfaces[SLAT_ALPHA_RIGHT_INDEX].result.alpha)
          : 0;
      let fuelFlow = 0;
      for (const engine of engines) {
        fuelFlow += engine.fuelFlow;
      }
      systems.update(outerAlpha, eas, fuelFlow, dt);

      // --- The control deflections ----------------------------------------
      // CONTROL_INDEX names each channel and every entry holds RADIANS. The
      // sign of each one follows the module comment of
      // src/aircraft/me262/geometry.ts.
      controls[CONTROL_INDEX.aileron] = clamp(input.roll, -1, 1) * AILERON_LIMIT;
      controls[CONTROL_INDEX.elevator] = clamp(input.pitch, -1, 1) * ELEVATOR_LIMIT;
      controls[CONTROL_INDEX.rudder] = clamp(input.yaw, -1, 1) * RUDDER_LIMIT;
      // The flap deflection and the slat position come from the systems, so the
      // aerodynamics sees the part where the mechanism has really moved to.
      systems.writeControls(controls);

      // --- The engines, one time per step ---------------------------------
      engineInput.throttle = clamp(input.throttle, 0, 1);
      engineInput.starterEngaged = input.startEngines;
      engineInput.altitude = altitude;
      engineInput.mach = mach;
      engineInput.airspeed = trueAirspeed;
      engineInput.density = atmosphere.density;
      engineInput.fuelAvailable = systems.state.fuelMass > 0;
      for (const engine of engines) {
        // The fuel cock of THIS engine. See COCK_OPEN_RPM. A lit engine keeps
        // its cock open, so a flame out in the air can still relight while the
        // rotor windmills above the same speed.
        engineInput.fuelCockOpen = engine.rpm >= COCK_OPEN_RPM || isLit(engine.state);
        engine.update(engineInput, dt);
      }

      // --- The landing gear, one time per step ----------------------------
      // The rudder pedals steer the nose wheel. That is how this aircraft
      // turns at taxi speed, together with the differential brake.
      clearWrench(gearWrench);
      gear.update(
        body,
        systems.state.gearPosition,
        clamp(input.yaw, -1, 1),
        systems.state.brakeLeft,
        systems.state.brakeRight,
        dt,
        gearWrench,
      );
      state.onGround = gear.anyOnGround;

      // --- The step -------------------------------------------------------
      firstStage = true;
      stepRK4(body, massProperties, source, simTime, dt);
      simTime += dt;

      // --- The mass, as the fuel burns ------------------------------------
      if (Math.abs(systems.state.fuelMass - massFuel) >= MASS_UPDATE_FUEL) {
        writeMass(systems.state.fuelMass);
      }
    },
  };

  api.spawnOnRunway();
  return api;
}
