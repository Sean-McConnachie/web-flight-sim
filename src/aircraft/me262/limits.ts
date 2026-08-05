/**
 * Structural limits of the Messerschmitt Me 262 A-1a, and the failures that
 * follow when the pilot crosses one.
 *
 * The module holds no aerodynamics and no ground model. It reads what the
 * aircraft did over one step, decides what the airframe kept and what it lost,
 * and reports the answer in three forms:
 *
 *   state           what is broken and how weak the airframe is now
 *   applyControls   what a bent wing and a jammed control do to the commands
 *   applyWingFailure what a wing that left the aircraft takes off the wrench
 *
 * src/aircraft/aircraft.ts calls all three. Nothing else needs this file.
 *
 *
 * 1. LIMIT LOAD AND ULTIMATE LOAD ARE TWO DIFFERENT THINGS
 *
 * A structure of 1944 was built to a LIMIT load, which it must carry with no
 * permanent set, and to an ULTIMATE load, which it must carry for three seconds
 * without breaking. The German requirement set the ultimate load at 1.8 times
 * the limit load. The British and the American requirements used 1.5.
 *
 * The two bounds do two different things in this model, and that difference is
 * the whole point of the file.
 *
 *   past the LIMIT load     The wing yields. It keeps flying, and it keeps the
 *                           set. `wingStrain` records it, the aileron loses
 *                           part of its power, the aircraft rolls toward the
 *                           bent panel, and every later limit is lower.
 *   past the ULTIMATE load  The panel outboard of the nacelle leaves the
 *                           aircraft. It makes no lift, and the wing that is
 *                           left rolls the aircraft hard toward the loss.
 *
 * A pilot who pulls 8 g one time therefore lands a bent aircraft that flies
 * one wing low. A pilot who pulls 13 g does not land it.
 *
 *
 * 2. WHAT THE AIRCRAFT WAS ALLOWED TO DO
 *
 * The handbook placards this aircraft AWAY from its structural limits, exactly
 * as it does with Mach number. "No spins are to be attempted with this
 * airplane." "No acrobatics are to be performed." There is therefore no
 * published g placard to read, and the limit has to come from the requirement
 * the airframe was built to.
 * Source: "Me 262 A-1 Pilot's Handbook", Wright Field F-SU-1111-ND, section 2,
 * spins and permissible acrobatics. Confidence: firm.
 *
 *
 * 3. THE FAILURES THIS FILE MODELS ARE THE ONES THE TYPE HAD
 *
 * Every failure below carries a document, and none of them is invented.
 *
 *   engine fire        "In the event that power units should begin to burn,
 *                      pull starter handles, re-ignite power plant and then
 *                      repeat above shutting down process." The drill for a
 *                      burning engine is to shut that engine down.
 *   fire on a relight  "Do not start above 4 km (13,100 ft) because of fire
 *                      hazard." A relight high up lights a pool of fuel in the
 *                      jet pipe instead of a clean flame.
 *   one engine out     "The aircraft will fly at 450 to 500 km/hr on one jet
 *                      unit." An engine lost to a fire is survivable, so this
 *                      file never turns one into a loss of the aircraft.
 *   tire blowout       "Keep the airplane straight by using corresponding
 *                      throttles and applying opposite brakes until the
 *                      airplane comes to a stop." A burst tire pulls the
 *                      aircraft to its own side.
 *   landing weight     "In case of a very short flight in which the fuel has
 *                      not all been used from the auxiliary tanks, use caution
 *                      in landing as the allowable landing weight is exceeded
 *                      due to the fuel load." Mass costs margin.
 * Source for all five: the same handbook, section 4, emergency operating
 * instructions, and section 3. Confidence: firm.
 *
 *
 * 4. WHERE THE GROUND FAILURES LIVE
 *
 * They do not live here. src/physics/gear.ts already bursts a tire past
 * `burstLoad` and already makes the pull of the burst leg, because the burst
 * changes the grip and the rolling resistance of ONE wheel and only that file
 * holds the tire. The same file holds the only brake fade model. This module
 * reads both and tells the pilot. It computes no ground force at all.
 *
 * The wing load check runs IN THE AIR only. On the ground the load path runs
 * through the gear and into the fuselage, and the tire is the fuse there by
 * design. See BURST_LOAD_FRACTION of src/physics/gear.ts.
 *
 *
 * 5. ALLOCATION
 *
 * `update` allocates nothing on a normal step. It builds no string and no
 * object while the airframe is whole. A failure raises one message, which is a
 * constant string, so even the failure step allocates nothing.
 *
 * This module is pure physics. It imports the Three.js core math classes only,
 * through the types it reads.
 */

import { clamp } from '@/math/tables';
import { kmhToMs } from '@/math/units';
import type { Surface } from '@/physics/aero/surface';
import type { Wrench } from '@/physics/rigidbody';
import { BRAKE_FADE_FULL_TEMPERATURE } from '@/physics/gear';
import { CONTROL_INDEX } from '@/aircraft/me262/geometry';
import { LOADED_MASS } from '@/aircraft/me262/mass';

// ---------------------------------------------------------------------------
// Load factor.
// ---------------------------------------------------------------------------

/**
 * Limit load factor of the sound airframe at the design mass, in g.
 *
 * The German requirement sorted an airframe into a stress group
 * (Beanspruchungsgruppe). The highest group, H 5, covered the single seat
 * fighter and asked for a limit load factor of 7 g. CONVENTIONS section 8
 * carries the same +7 g and marks it estimated. The two agree.
 *
 * Source: the German stress groups as summarized in the load factor threads of
 * ww2aircraft.net and secretprojects.co.uk, against CONVENTIONS section 8.
 * Confidence: medium. The group is firm for a fighter of the period. The
 * placement of THIS aircraft in that group is the estimate.
 */
export const LIMIT_LOAD_POSITIVE = 7; // g

/**
 * Limit load factor on the negative side, in g.
 *
 * CONVENTIONS section 8 gives -3 g and marks it estimated. The ratio to the
 * positive limit is 0.43, which is where the fighters of the period sat: the
 * P-51D carried +8 and -4, and the requirement of the day asked for 0.4 to 0.5
 * of the positive case. Confidence: estimate.
 */
export const LIMIT_LOAD_NEGATIVE = -3; // g

/**
 * Ratio of the ultimate load to the limit load.
 *
 * The German requirement used 1.8 where the British and the American
 * requirements used 1.5. The product, 7 * 1.8 = 12.6 g, has an independent
 * check: the IL-2 Great Battles data set, which works from captured
 * Messerschmitt material, gives the Me 262 a maximum load factor of 12.5 g.
 * The two agree to one percent, and 12.5 g is a breaking value, not a placard.
 *
 * Source: the German safety factor of 1.8, and the IL-2 Me 262 A data sheet at
 * aergistal.github.io. Confidence: medium.
 */
export const ULTIMATE_FACTOR = 1.8;

/** Load factor that breaks the sound wing at the design mass, in g. */
export const ULTIMATE_LOAD_POSITIVE = LIMIT_LOAD_POSITIVE * ULTIMATE_FACTOR; // 12.6 g
export const ULTIMATE_LOAD_NEGATIVE = LIMIT_LOAD_NEGATIVE * ULTIMATE_FACTOR; // -5.4 g

/**
 * Mass the limit load factors above apply at, kg.
 *
 * A structure carries a LOAD, in newtons, and not a load factor. The limit
 * therefore falls as the aircraft gets heavier: a full aircraft has less margin
 * than an empty one. The reference is the published loaded mass, because that
 * is the mass a fighter of the period was stressed at.
 * Source: CONVENTIONS section 8, loaded mass, firm.
 */
export const LOAD_LIMIT_DESIGN_MASS = LOADED_MASS; // 6396 kg

/**
 * Largest value the mass scaling may reach.
 *
 * The inverse mass law would give an empty aircraft 11.8 g, and no airframe
 * works that way. The wing bending case scales with the mass, and the fittings,
 * the control surfaces and the tail do not, so one of them takes over as the
 * mass falls. The cap puts that takeover at 8.75 g. Confidence: estimate.
 */
export const MAX_LIMIT_SCALE = 1.25;

/**
 * Fraction of its strength the wing loses at a full permanent set.
 *
 * A wing that has yielded is not the wing that left the factory. The value is
 * the fall of both the limit load and the ultimate load at `wingStrain` of 1.
 * Confidence: estimate.
 */
export const STRAIN_STRENGTH_LOSS = 0.3;

/** The same fall at a full airframe overspeed damage. Confidence: estimate. */
export const OVERSPEED_STRENGTH_LOSS = 0.3;

/**
 * Fraction of its strength the wing loses when a fire burns through the
 * nacelle beside it.
 *
 * The main spar runs through the nacelle bay. A fire that burns long enough to
 * take the engine mount has been at the spar cap as well, so that side of the
 * wing carries far less than it did. Confidence: estimate.
 */
export const FIRE_STRENGTH_LOSS = 0.4;

// ---------------------------------------------------------------------------
// Airspeed.
// ---------------------------------------------------------------------------

/**
 * Highest true airspeed the airframe takes.
 *
 * The handbook table of section 3 gives the maximum speed in a 20 to 30 degree
 * dive as 950 km/h TRUE. CONVENTIONS section 8 records the same number as the
 * placard. The value is a true airspeed and not an equivalent airspeed, so the
 * check reads the true airspeed, unlike the gear and the flap checks of
 * src/aircraft/me262/systems.ts, which read the equivalent airspeed because a
 * hinge load follows the dynamic pressure.
 * Source: "Me 262 A-1 Pilot's Handbook", section 3, speed and range.
 * Confidence: firm.
 */
export const AIRFRAME_LIMIT_SPEED = kmhToMs(950); // m/s true

/**
 * Time in which the airframe reaches full damage at twice the limit dynamic
 * pressure.
 *
 * The law and the value both follow DAMAGE_TIME of
 * src/aircraft/me262/systems.ts, so the airframe, the flap and the gear all
 * take damage the same way. Twice the limit pressure is 41 percent over the
 * limit speed, which is 1343 km/h, and no Me 262 ever reached it. A pilot who
 * sits 10 percent over the placard needs 48 seconds to destroy the aircraft.
 * Confidence: estimate.
 */
export const OVERSPEED_DAMAGE_TIME = 10; // s

// WHAT A FULL AIRFRAME OVERSPEED DAMAGE BREAKS.
//
// The handbook reports no flutter in a dive and gives the reason: the tailplane
// sits high, clear of the wing wake. It reports nothing about the aileron. An
// unbalanced aileron on a wing whose skin has started to work is the classic
// high speed failure of the period, and it is a failure the pilot can survive,
// because the aircraft still turns on the rudder. The model therefore jams the
// AILERON and leaves the elevator alone.
// Source: the same handbook, section 2, diving. Confidence: estimate.

// ---------------------------------------------------------------------------
// Handling of a bent wing.
// ---------------------------------------------------------------------------

/**
 * Aileron power a full permanent set removes.
 *
 * A wing that has yielded carries a twist. The twist works against the aileron
 * at the same station, so the surface moves the same angle and makes less roll.
 * Confidence: estimate.
 */
export const STRAIN_AILERON_LOSS = 0.5;

/**
 * Standing aileron deflection a full permanent set makes, rad.
 *
 * The two wings no longer match, so the aircraft rolls with the stick central.
 * The model carries the mismatch on the aileron channel, which is the standard
 * way to write a rigging asymmetry into a strip model: it makes the same roll
 * couple that the two mismatched panels make. The value is 0.05 rad against the
 * 0.35 rad of full aileron, so a fully bent wing eats 14 percent of the roll
 * control and the pilot flies with the stick out of center. Confidence:
 * estimate.
 */
export const STRAIN_AILERON_BIAS = 0.05; // rad

// ---------------------------------------------------------------------------
// Fire.
// ---------------------------------------------------------------------------

/**
 * Altitude above which a relight lights a fire instead of an engine.
 *
 * The fuel does not atomize in thin cold air, so it pools in the chambers and
 * in the jet pipe and then burns where it lies.
 * Source: "Me 262 A-1 Pilot's Handbook", section 3, to start during flight:
 * "Do not start above 4 km (13,100 ft) because of fire hazard."
 * Confidence: firm.
 */
export const RELIGHT_FIRE_ALTITUDE = 4000; // m

/**
 * Time a fire needs to burn through the engine mount and the spar beside it.
 *
 * src/aircraft/me262/engine.ts already ruins the turbine in ten seconds of
 * fire, which costs 60 percent of the thrust. The mount and the wing structure
 * take longer. At the end of this time the model runs the handbook drill and
 * shuts that engine down, which puts the fire out and leaves the aircraft on
 * one engine. The pilot then has an aircraft that the handbook says flies.
 * Confidence: estimate.
 */
export const FIRE_BURN_THROUGH_TIME = 25; // s

// ---------------------------------------------------------------------------
// Fuel feed.
// ---------------------------------------------------------------------------

/**
 * Fuel the pumps cannot reach, kg.
 *
 * The pickup sits above the bottom of the tank and the last of the fuel lies in
 * the corners. Two percent of the 2133 kg capacity is the usual allowance for
 * an aircraft of this size, which is 43 kg. The model carries 40 kg.
 * Confidence: estimate.
 */
export const UNUSABLE_FUEL = 40; // kg

/**
 * Fuel under which the pickup can uncover at all, kg.
 *
 * A FULL TANK NEVER UNPORTS. The tanks are not pressurized and each booster
 * pump sits at the bottom of its tank, but a tank that is nearly full holds
 * fuel against the pump whatever the aircraft does, because there is no free
 * surface for the fuel to run away over. Only a tank with a shallow puddle in
 * it can leave the pump dry. 150 kg is 180 liters over four tanks, which is a
 * puddle a few centimeters deep in each one.
 *
 * THE FLIGHT TESTS MEASURED THE OTHER RULE AND FOUND IT WRONG. A rule with no
 * fuel term took the feed away from a FULL aircraft in the dive entry of
 * test/flight/mach.test.ts, where the autopilot pushes over to -1.34 g for
 * three seconds. The engines flamed out, and src/aircraft/me262/engine.ts
 * cannot relight at a high throttle, so the whole dive ran with no thrust and
 * the peak Mach row moved from 0.857 to 0.850. A full tank that starves is
 * wrong physics, and the measurement is what showed it.
 * Confidence: estimate.
 */
export const UNPORT_LOW_FUEL_MASS = 150; // kg

/**
 * Load factor under which the pickup uncovers, at UNPORT_LOW_FUEL_MASS.
 *
 * At zero g the puddle leaves the floor of the tank and the pump passes air.
 * The threshold climbs as the last of the fuel goes, because a smaller puddle
 * runs off the pump at a smaller push.
 * Confidence: firm on the mechanism, estimate on the values.
 */
export const UNPORT_LOAD_FACTOR = 0; // g

/** The same threshold with almost nothing in the tanks. Confidence: estimate. */
export const UNPORT_LOW_FUEL_LOAD_FACTOR = 0.5; // g

/** Time under the threshold that empties the line and stops the flame. */
export const UNPORT_TIME = 1.0; // s

/** Time over the threshold that fills the line again. */
export const FEED_RECOVERY_TIME = 0.5; // s

// ---------------------------------------------------------------------------
// The wing panel that fails.
// ---------------------------------------------------------------------------

/**
 * The strips of the panel that leaves the aircraft, left wing and right wing.
 *
 * me262Surfaces of src/aircraft/me262/geometry.ts puts the left wing at 0 to 7
 * and the right wing at 8 to 15, root first, on cosine spaced stations. The
 * boundary between strip 1 and strip 2 sits at 2.39 m from the plane of
 * symmetry and the engine center line sits at 2.05 m, so strips 2 to 7 are the
 * panel OUTBOARD OF THE NACELLE. That is where a wing of this layout breaks: a
 * failure inboard of the nacelle takes the engine and the fuselage with it, and
 * there is no aircraft left to fly.
 *
 * The panel carries the whole aileron, which runs from 4.00 m to 5.98 m, so the
 * side that fails also loses its roll control with no special case anywhere.
 */
export const FAILED_PANEL_LEFT: readonly number[] = [2, 3, 4, 5, 6, 7];
export const FAILED_PANEL_RIGHT: readonly number[] = [10, 11, 12, 13, 14, 15];

// ---------------------------------------------------------------------------
// Public types.
// ---------------------------------------------------------------------------

/** Which wing panel is gone. */
export type WingSide = 'none' | 'left' | 'right';

/** The four load factor bounds of the airframe as it is now, in g. */
export interface LoadLimits {
  /** Positive load factor with no permanent set. */
  limitPositive: number;
  /** Negative load factor with no permanent set. */
  limitNegative: number;
  /** Positive load factor that breaks the wing. */
  ultimatePositive: number;
  /** Negative load factor that breaks the wing. */
  ultimateNegative: number;
}

/** What the structure reads from the aircraft over one step. */
export interface StructureInput {
  /** Body z load factor, g. Level flight reads 1. */
  loadFactor: number;
  /** Roll rate, rad/s. Positive rolls the right wing down. It picks the side. */
  rollRate: number;
  /** Total mass now, kg. */
  mass: number;
  /** True airspeed, m/s. The airframe placard is a true airspeed. */
  trueAirspeed: number;
  /** Altitude above sea level, m. */
  altitude: number;
  /** Fuel on board, kg. */
  fuelMass: number;
  /** True while any wheel or any part of the airframe touches the ground. */
  onGround: boolean;
  /** True while the engine of that side burns. Left first. */
  engineFire: boolean[];
  /** True on the update in which the engine of that side lights. Left first. */
  engineLightOff: boolean[];
  /** True while the tire of that leg is burst. Nose, left main, right main. */
  tireBurst: boolean[];
  /** Brake pack temperature of each leg, K, in the same order. */
  brakeTemperature: number[];
}

/** What is broken, and how weak the airframe is now. */
export interface StructureState {
  /** The four bounds, with every loss the airframe has taken already in them. */
  limits: LoadLimits;
  /** 0 sound, 1 the wing carries the whole permanent set it can carry. */
  wingStrain: number;
  /** -1 the left panel is bent, +1 the right panel is bent, 0 neither. */
  strainSide: number;
  /** 0 sound, 1 the skin and the rivets of the airframe are gone. */
  overspeed: number;
  /** Which wing panel left the aircraft. */
  wingFailure: WingSide;
  /** True while the aileron answers nothing. */
  aileronJammed: boolean;
  /** The deflection the jammed aileron holds, rad. */
  jammedAileron: number;
  /** Time each nacelle has burned, s. Left first. */
  fireTime: number[];
  /** True while the caller must shut that engine down. Left first. */
  engineShutdown: boolean[];
  /** Strength that is left in each wing panel, 0 to 1. Left first. */
  panelStrength: number[];
  /** True while the fuel system delivers fuel to the engines. */
  fuelAvailable: boolean;
  /** Largest and smallest load factor since the last reset, g. */
  peakPositive: number;
  peakNegative: number;
}

/**
 * One message for the pilot.
 *
 * The shape follows EngineEvents of src/aircraft/me262/engine.ts. `raised`
 * holds for the one update that made it, and `count` only grows, so a reader
 * that runs slower than the physics can compare the count against its own copy
 * and miss nothing.
 */
export interface StructureEvents {
  raised: boolean;
  /** What the pilot must read. It is empty while `raised` is false. */
  message: string;
  count: number;
}

export interface Structure {
  readonly state: StructureState;
  readonly events: StructureEvents;
  /** Runs one time per physics step, with the load factor of the last step. */
  update(input: StructureInput, dt: number): void;
  /**
   * Writes the damage into the control array, AFTER the caller has written the
   * pilot commands into it. A jammed control stops answering. A bent wing takes
   * part of the aileron and leaves a standing deflection behind.
   */
  applyControls(controls: Float64Array): void;
  /**
   * Takes the load of a failed wing panel back off the wrench.
   *
   * The caller runs it inside the wrench source, after
   * AeroAssembly.evaluate has added every strip. Each SurfaceResult holds
   * exactly the force and the moment that evaluateSurface added, so the
   * subtraction removes that panel exactly.
   *
   * WHAT THIS DOES NOT DO. The induced angle of the wing group still comes from
   * a whole wing, because only src/physics/aero/assembly.ts can solve a group
   * that has lost half of one side. The error is second order against the lift
   * that has gone.
   */
  applyWingFailure(surfaces: readonly Surface[], out: Wrench): void;
  reset(): void;
}

// ---------------------------------------------------------------------------
// The messages. Every one is a constant, so a failure allocates nothing.
// ---------------------------------------------------------------------------

const MESSAGE_STRAIN =
  'WING OVERLOAD. The aircraft went past its limit load factor and the wing ' +
  'carries a permanent set. The aileron is weaker, the aircraft rolls with the ' +
  'stick central, and every load limit is lower than it was.';

const MESSAGE_FAIL_LEFT =
  'LEFT WING FAILED. The panel outboard of the left nacelle is gone. The ' +
  'aircraft rolls left. Center the controls, close the throttle and leave the ' +
  'aircraft.';

const MESSAGE_FAIL_RIGHT =
  'RIGHT WING FAILED. The panel outboard of the right nacelle is gone. The ' +
  'aircraft rolls right. Center the controls, close the throttle and leave the ' +
  'aircraft.';

const MESSAGE_OVERSPEED =
  'AIRFRAME OVERSPEED. The aircraft is past 950 km/h true, which is the ' +
  'placard of the handbook. Reduce the speed before the structure gives.';

const MESSAGE_AILERON_JAM =
  'AILERON JAMMED. The overspeed took the aileron. Turn with the rudder and ' +
  'land as soon as possible.';

const MESSAGE_FIRE_LEFT =
  'LEFT ENGINE FIRE. The fire burns through the mount in about 25 seconds and ' +
  'the engine then stops. The aircraft flies on one engine above 260 km/h.';

const MESSAGE_FIRE_RIGHT =
  'RIGHT ENGINE FIRE. The fire burns through the mount in about 25 seconds and ' +
  'the engine then stops. The aircraft flies on one engine above 260 km/h.';

const MESSAGE_BURNED_LEFT =
  'LEFT ENGINE LOST. The fire is out and the engine is dead. The left wing is ' +
  'weak, so hold the load factor low. Bank toward the good engine.';

const MESSAGE_BURNED_RIGHT =
  'RIGHT ENGINE LOST. The fire is out and the engine is dead. The right wing ' +
  'is weak, so hold the load factor low. Bank toward the good engine.';

const MESSAGE_FUEL_LOST =
  'FUEL FEED LOST. The pump passes air, so both engines stop. Hold positive g, ' +
  'close the throttle, and relight below 900 km/h.';

const MESSAGE_TANKS_DRY =
  'TANKS DRY. The pumps cannot lift the last of the fuel out of the corners of ' +
  'the tanks, so both engines stop. Glide, and land straight ahead.';

const MESSAGE_FUEL_BACK =
  'FUEL FEED RESTORED. The throttle must sit closed before either engine can ' +
  'light again.';

const MESSAGE_TIRE_NOSE =
  'NOSE TIRE BURST. Hold the aircraft straight on the rudder and brake gently.';

const MESSAGE_TIRE_LEFT =
  'LEFT TIRE BURST. The aircraft pulls left. Hold it straight with the right ' +
  'brake until it stops.';

const MESSAGE_TIRE_RIGHT =
  'RIGHT TIRE BURST. The aircraft pulls right. Hold it straight with the left ' +
  'brake until it stops.';

const MESSAGE_BRAKE_FADE =
  'BRAKES FADED. The pack is past 400 C and it keeps half of its torque. Let ' +
  'the aircraft roll and let the brakes cool.';

// ---------------------------------------------------------------------------
// Pure functions. The tests read these directly.
// ---------------------------------------------------------------------------

/**
 * Writes the load factor bounds of a SOUND airframe at one mass.
 *
 * The structure carries a load, so the bound falls as the mass rises. At the
 * maximum takeoff mass of 7130 kg the positive limit is 6.28 g against the
 * 7.00 g of the loaded mass. MAX_LIMIT_SCALE caps the other end.
 */
export function loadLimits(mass: number, out: LoadLimits): LoadLimits {
  const scale = Math.min(MAX_LIMIT_SCALE, LOAD_LIMIT_DESIGN_MASS / Math.max(1, mass));
  out.limitPositive = LIMIT_LOAD_POSITIVE * scale;
  out.limitNegative = LIMIT_LOAD_NEGATIVE * scale;
  out.ultimatePositive = ULTIMATE_LOAD_POSITIVE * scale;
  out.ultimateNegative = ULTIMATE_LOAD_NEGATIVE * scale;
  return out;
}

/**
 * Returns the damage a part takes over dt at a true airspeed past its limit.
 *
 * The law is the one overspeedDamage of src/aircraft/me262/systems.ts uses. The
 * load on the skin follows the dynamic pressure, which follows the square of
 * the speed, so the function works with the square of the speed ratio.
 */
export function overspeedDamage(speed: number, limit: number, dt: number): number {
  if (!(speed > limit) || !(limit > 0)) {
    return 0;
  }
  return (((speed * speed) / (limit * limit) - 1) * dt) / OVERSPEED_DAMAGE_TIME;
}

/**
 * Returns the load factor under which the fuel pickup uncovers, at one fuel
 * state.
 *
 * A tank with more than UNPORT_LOW_FUEL_MASS in it holds the pump under fuel at
 * any load factor, so the function reports the negative infinity of a tank that
 * cannot unport at all. Below that the threshold runs from zero g to
 * UNPORT_LOW_FUEL_LOAD_FACTOR as the last of the fuel goes.
 */
export function unportLoadFactor(fuelMass: number): number {
  if (fuelMass > UNPORT_LOW_FUEL_MASS) {
    return Number.NEGATIVE_INFINITY;
  }
  const low = 1 - clamp(fuelMass / UNPORT_LOW_FUEL_MASS, 0, 1);
  return UNPORT_LOAD_FACTOR + (UNPORT_LOW_FUEL_LOAD_FACTOR - UNPORT_LOAD_FACTOR) * low;
}

// ---------------------------------------------------------------------------
// The structure.
// ---------------------------------------------------------------------------

/** Builds the structure of one aircraft. Everything starts sound. */
export function createStructure(): Structure {
  const state: StructureState = {
    limits: {
      limitPositive: LIMIT_LOAD_POSITIVE,
      limitNegative: LIMIT_LOAD_NEGATIVE,
      ultimatePositive: ULTIMATE_LOAD_POSITIVE,
      ultimateNegative: ULTIMATE_LOAD_NEGATIVE,
    },
    wingStrain: 0,
    strainSide: 0,
    overspeed: 0,
    wingFailure: 'none',
    aileronJammed: false,
    jammedAileron: 0,
    fireTime: [0, 0],
    engineShutdown: [false, false],
    panelStrength: [1, 1],
    fuelAvailable: true,
    peakPositive: 1,
    peakNegative: 1,
  };

  const events: StructureEvents = { raised: false, message: '', count: 0 };

  /** The bounds of the sound airframe at this mass, before any damage. */
  const sound: LoadLimits = {
    limitPositive: LIMIT_LOAD_POSITIVE,
    limitNegative: LIMIT_LOAD_NEGATIVE,
    ultimatePositive: ULTIMATE_LOAD_POSITIVE,
    ultimateNegative: ULTIMATE_LOAD_NEGATIVE,
  };

  // Latches. Each message goes out one time.
  let strainLatched = false;
  let overspeedLatched = false;
  let brakeLatched = false;
  const fireLatched = [false, false];
  const tireLatched = [false, false, false];

  let unportTimer = 0; // s under the threshold
  let feedTimer = 0; // s over the threshold

  /** Raises one message. The last one of a step wins, and none is lost. */
  function raise(message: string): void {
    events.raised = true;
    events.message = message;
    events.count++;
  }

  /** Which side takes the damage of an overload. */
  function loadedSide(rollRate: number): number {
    // In a rolling pull the DOWN going panel works at the higher local angle of
    // attack, so it carries the greater load. A positive roll rate puts the
    // right wing down. With no roll rate at all the model picks the left panel,
    // so a test repeats.
    if (rollRate > 1e-3) {
      return 1;
    }
    if (rollRate < -1e-3) {
      return -1;
    }
    return state.strainSide !== 0 ? state.strainSide : -1;
  }

  /** Breaks the panel of one side and reports it. */
  function failWing(side: number): void {
    if (state.wingFailure !== 'none') {
      return;
    }
    state.wingFailure = side > 0 ? 'right' : 'left';
    raise(side > 0 ? MESSAGE_FAIL_RIGHT : MESSAGE_FAIL_LEFT);
  }

  const api: Structure = {
    state,
    events,

    update(input: StructureInput, dt: number): void {
      events.raised = false;
      events.message = '';
      if (!(dt > 0)) {
        return;
      }

      const n = input.loadFactor;
      if (n > state.peakPositive) {
        state.peakPositive = n;
      }
      if (n < state.peakNegative) {
        state.peakNegative = n;
      }

      // --- What the airframe can take now ---------------------------------
      // The permanent set and the overspeed damage act on the whole airframe.
      // A fire acts on one side, so the weaker panel decides what the aircraft
      // as a whole may pull.
      loadLimits(input.mass, sound);
      const airframe =
        (1 - STRAIN_STRENGTH_LOSS * state.wingStrain) *
        (1 - OVERSPEED_STRENGTH_LOSS * state.overspeed);
      const weakest = Math.min(state.panelStrength[0], state.panelStrength[1]);
      const strength = airframe * weakest;
      state.limits.limitPositive = sound.limitPositive * strength;
      state.limits.limitNegative = sound.limitNegative * strength;
      state.limits.ultimatePositive = sound.ultimatePositive * strength;
      state.limits.ultimateNegative = sound.ultimateNegative * strength;

      // --- Limit load and ultimate load -----------------------------------
      // IN THE AIR ONLY. See section 4 of the module comment: the tire is the
      // fuse on the ground and src/physics/gear.ts owns it.
      if (!input.onGround && state.wingFailure === 'none') {
        const limit = n >= 0 ? state.limits.limitPositive : state.limits.limitNegative;
        const ultimate = n >= 0 ? state.limits.ultimatePositive : state.limits.ultimateNegative;
        const past = n >= 0 ? n > ultimate : n < ultimate;
        const yielding = n >= 0 ? n > limit : n < limit;
        if (past) {
          // The weak panel goes first. With two sound panels the roll decides.
          const side =
            state.panelStrength[0] < state.panelStrength[1]
              ? -1
              : state.panelStrength[1] < state.panelStrength[0]
                ? 1
                : loadedSide(input.rollRate);
          failWing(side);
        } else if (yielding) {
          // A permanent set follows the PEAK of the excursion and not the time
          // it lasted. Yield is not a rate. The set runs from zero at the limit
          // load to one at the ultimate load, so a wing that reaches the
          // ultimate load has used the whole of the band the structure has.
          //
          // THE BAND IS THE ONE OF THE SOUND AIRFRAME AT THIS MASS, and the
          // yield test above is the one of the airframe as it is now. A weak
          // wing therefore starts to take a new set earlier, and the SIZE of
          // the set still follows how far past the design load the aircraft
          // went. A model that measured the size on the weakened band would
          // feed on itself: each step would lower the band, the next step would
          // read a larger set, and one excursion a real wing survives would
          // walk itself to a failure inside a tenth of a second.
          const soundLimit = n >= 0 ? sound.limitPositive : sound.limitNegative;
          const soundUltimate = n >= 0 ? sound.ultimatePositive : sound.ultimateNegative;
          const set = clamp((n - soundLimit) / (soundUltimate - soundLimit), 0, 1);
          if (set > state.wingStrain) {
            state.wingStrain = set;
            state.strainSide = loadedSide(input.rollRate);
            if (!strainLatched) {
              strainLatched = true;
              raise(MESSAGE_STRAIN);
            }
          }
        }
      }

      // --- Airframe overspeed ---------------------------------------------
      if (input.trueAirspeed > AIRFRAME_LIMIT_SPEED) {
        if (!overspeedLatched) {
          overspeedLatched = true;
          raise(MESSAGE_OVERSPEED);
        }
        const before = state.overspeed;
        state.overspeed = clamp(
          before + overspeedDamage(input.trueAirspeed, AIRFRAME_LIMIT_SPEED, dt),
          0,
          1,
        );
        if (state.overspeed >= 1 && before < 1 && !state.aileronJammed) {
          state.aileronJammed = true;
          raise(MESSAGE_AILERON_JAM);
        }
      }

      // --- Fire ------------------------------------------------------------
      for (let i = 0; i < state.fireTime.length; i++) {
        if (state.engineShutdown[i]) {
          continue;
        }
        // Two sources, and the handbook carries both. The engine model raises
        // its own fire. A relight above 4 km lights the pool of fuel that the
        // thin air left unburned.
        const started =
          input.engineFire[i] ||
          (input.engineLightOff[i] && input.altitude > RELIGHT_FIRE_ALTITUDE);
        if (started && state.fireTime[i] === 0) {
          if (!fireLatched[i]) {
            fireLatched[i] = true;
            raise(i === 0 ? MESSAGE_FIRE_LEFT : MESSAGE_FIRE_RIGHT);
          }
        }
        if (started || state.fireTime[i] > 0) {
          state.fireTime[i] += dt;
        }
        if (state.fireTime[i] >= FIRE_BURN_THROUGH_TIME) {
          // The handbook drill. The engine goes off, the fire goes out with it,
          // and the aircraft flies home on the other one.
          state.engineShutdown[i] = true;
          state.panelStrength[i] *= 1 - FIRE_STRENGTH_LOSS;
          raise(i === 0 ? MESSAGE_BURNED_LEFT : MESSAGE_BURNED_RIGHT);
        }
      }

      // --- The fuel feed ----------------------------------------------------
      const dry = input.fuelMass <= UNUSABLE_FUEL;
      const covered = n >= unportLoadFactor(input.fuelMass);
      if (covered && !dry) {
        unportTimer = 0;
        feedTimer += dt;
      } else {
        unportTimer += dt;
        feedTimer = 0;
      }
      if (state.fuelAvailable && (unportTimer >= UNPORT_TIME || dry)) {
        state.fuelAvailable = false;
        raise(dry ? MESSAGE_TANKS_DRY : MESSAGE_FUEL_LOST);
      } else if (!state.fuelAvailable && feedTimer >= FEED_RECOVERY_TIME) {
        state.fuelAvailable = true;
        raise(MESSAGE_FUEL_BACK);
      }

      // --- What the ground model already broke ------------------------------
      for (let i = 0; i < tireLatched.length && i < input.tireBurst.length; i++) {
        if (input.tireBurst[i] && !tireLatched[i]) {
          tireLatched[i] = true;
          raise(
            i === 0 ? MESSAGE_TIRE_NOSE : i === 1 ? MESSAGE_TIRE_LEFT : MESSAGE_TIRE_RIGHT,
          );
        }
      }
      if (!brakeLatched) {
        for (let i = 0; i < input.brakeTemperature.length; i++) {
          if (input.brakeTemperature[i] >= BRAKE_FADE_FULL_TEMPERATURE) {
            brakeLatched = true;
            raise(MESSAGE_BRAKE_FADE);
            break;
          }
        }
      }
    },

    applyControls(controls: Float64Array): void {
      if (controls.length <= CONTROL_INDEX.aileron) {
        return;
      }
      if (state.aileronJammed) {
        controls[CONTROL_INDEX.aileron] = state.jammedAileron;
        return;
      }
      if (state.wingStrain > 0) {
        controls[CONTROL_INDEX.aileron] =
          controls[CONTROL_INDEX.aileron] * (1 - STRAIN_AILERON_LOSS * state.wingStrain) +
          state.strainSide * STRAIN_AILERON_BIAS * state.wingStrain;
      }
      // The value the aileron would hold if it jammed on the next step.
      state.jammedAileron = controls[CONTROL_INDEX.aileron];
    },

    applyWingFailure(surfaces: readonly Surface[], out: Wrench): void {
      if (state.wingFailure === 'none') {
        return;
      }
      const panel = state.wingFailure === 'left' ? FAILED_PANEL_LEFT : FAILED_PANEL_RIGHT;
      for (let i = 0; i < panel.length; i++) {
        const index = panel[i];
        if (index >= surfaces.length) {
          continue;
        }
        const result = surfaces[index].result;
        out.force.sub(result.force);
        out.moment.sub(result.moment);
      }
    },

    reset(): void {
      state.limits.limitPositive = LIMIT_LOAD_POSITIVE;
      state.limits.limitNegative = LIMIT_LOAD_NEGATIVE;
      state.limits.ultimatePositive = ULTIMATE_LOAD_POSITIVE;
      state.limits.ultimateNegative = ULTIMATE_LOAD_NEGATIVE;
      state.wingStrain = 0;
      state.strainSide = 0;
      state.overspeed = 0;
      state.wingFailure = 'none';
      state.aileronJammed = false;
      state.jammedAileron = 0;
      state.fuelAvailable = true;
      state.peakPositive = 1;
      state.peakNegative = 1;
      for (let i = 0; i < state.fireTime.length; i++) {
        state.fireTime[i] = 0;
        state.engineShutdown[i] = false;
        state.panelStrength[i] = 1;
        fireLatched[i] = false;
      }
      for (let i = 0; i < tireLatched.length; i++) {
        tireLatched[i] = false;
      }
      strainLatched = false;
      overspeedLatched = false;
      brakeLatched = false;
      unportTimer = 0;
      feedTimer = 0;
      events.raised = false;
      events.message = '';
      events.count = 0;
    },
  };

  return api;
}
