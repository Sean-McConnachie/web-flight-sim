/**
 * Landing gear, tires and brakes.
 *
 * The module holds three oleo pneumatic legs, a slip based tire, a nose wheel
 * steering channel and an independent brake on each main wheel. It reads the
 * rigid body state, tests each wheel against the flat ground of
 * src/physics/contact.ts, and ADDS the ground wrench into the wrench the caller
 * already collects from the aerodynamics, the thrust and gravity.
 *
 *
 * 1. THE STRUT
 *
 * A leg is a gas spring in series with a tire spring, with a damper across the
 * gas spring and a hard stop at the end of the travel.
 *
 *   F_gas(s) = springGas * (1 - GAS_FILL * s / maxTravel) ^ -POLYTROPIC_INDEX
 *
 * That is the polytropic law of the sealed air chamber of a real oleo. The
 * piston sweeps the chamber, the volume falls, and the pressure rises as the
 * inverse of the volume raised to the polytropic index. GAS_FILL says how much
 * of the chamber the piston sweeps over the full travel, so the force at the end
 * of the travel is (1 - GAS_FILL) ^ -POLYTROPIC_INDEX times the force at full
 * extension. With GAS_FILL = 0.9 and POLYTROPIC_INDEX = 1.3 that ratio is 19.9,
 * which is the compression ratio a single stage oleo really works over.
 *
 * A LINEAR spring is wrong here and it feels wrong. A linear spring that carries
 * the aircraft at rest is far too soft at the end of the stroke, so it bottoms
 * on any firm landing, and it returns all of its energy, so the aircraft bounces
 * back to nearly the height it fell from. The gas curve stiffens as it
 * compresses, so it meets a hard landing with a rising force instead of a wall.
 *
 * The damper is ASYMMETRIC. dampingCompression is much larger than
 * dampingRebound. The oleo meters the fluid through a large orifice while the
 * strut closes and through a small one while it opens, so it swallows the
 * energy of the touchdown and gives little of it back. Without the asymmetry the
 * aircraft pogos down the runway.
 *
 * The strut carries a PRELOAD. The gas force at full extension is not zero, so a
 * light load deflects the tire only and never moves the piston. That is what a
 * real oleo does, and it is what makes small bumps feel like tire bumps.
 *
 *
 * 2. THE TIRE
 *
 * The tire is a vertical spring in series with the strut, and a slip based
 * friction pair. The vertical spring is separate from the strut because it
 * matters. It carries the first part of the touchdown impulse before the piston
 * moves at all, and on a small bump it is the only thing that moves.
 *
 * The friction uses the Pacejka magic formula with the curvature term set to
 * zero:
 *
 *   mu(x) = D * sin(C * atan(B * x))
 *
 * D is the peak, C sets how far the curve falls after the peak, and B sets how
 * fast it rises. The curve RISES to a peak and then FALLS. That shape is the
 * whole point. A wheel at the optimum slip grips harder than a locked wheel, so
 * a hard brake application that locks a wheel LOSES braking and the aircraft
 * skids. A linear or a saturating model cannot show that.
 *
 * The longitudinal curve and the lateral curve are separate, with different
 * peaks and different shapes, because a tire is not isotropic. A friction
 * ellipse then limits the two together, so a wheel that is already braking at
 * the limit has no grip left to steer with.
 *
 *
 * 3. THE BRAKES
 *
 * The main wheels brake, the nose wheel does not, and the left and the right
 * brake take separate commands. Differential braking is how this aircraft steers
 * at low speed. The brake makes a torque on the wheel, the wheel slows, the slip
 * ratio grows, and the tire curve decides how much of that reaches the ground.
 * Nothing shortcuts the tire.
 *
 * The brake heats up from its own friction power and cools toward the ambient
 * air. Above BRAKE_FADE_START the friction coefficient of the pack falls, so a
 * long drag down the runway costs braking. The model is one temperature per
 * wheel, which is enough to punish a pilot who rides the brakes.
 *
 *
 * 4. FRAMES
 *
 * Every leg position is a body axis offset from the center of gravity, with x
 * forward, y right and z down, as CONVENTIONS section 3.1 states. The attitude
 * of the aircraft therefore decides where each wheel really is, which is what
 * gives a crosswind landing one wheel first and what lifts the nose wheel during
 * the rotation. src/physics/contact.ts owns that step.
 *
 * The strut axis is the BODY z axis. The real legs rake a little, and the model
 * ignores the rake and places each vertical strut through its own axle. The
 * error is a few degrees of stroke direction and it is far below the error in
 * the oleo constants.
 *
 *
 * 5. COST AND STATE
 *
 * The gear holds state: the wheel spin, the brake temperature and the burst
 * flag. `update` integrates that state with the `dt` it receives, so the caller
 * runs it ONE time per physics step. A caller that drives stepRK4 must add the
 * gear wrench outside the four stages, or accept that the wheel spin integrates
 * four times. The strut, the tire and the friction are all pure functions of the
 * state, so the wrench itself is correct in any stage.
 *
 * `update` allocates nothing. Every scratch vector sits in module scope.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Vector3 } from 'three';

import { clamp, smoothstep } from '@/math/tables';
import { DEG, G0 } from '@/math/units';
import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import type { ContactSample } from '@/physics/contact';
import { addContactWrench, createContactSample, sampleContact } from '@/physics/contact';

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/** The fixed description of one leg. Nothing here changes while the sim runs. */
export interface GearLegDef {
  name: string;
  /** Top of the strut, body axes, m from the center of gravity. */
  position: Vector3;
  /** Distance from `position` to the axle at full extension, m. */
  restLength: number;
  /** Stroke from full extension to the hard stop, m. */
  maxTravel: number;
  /** Gas spring force at full extension, N. It is the strut preload. */
  springGas: number;
  /** Damping while the strut closes, N s / m. */
  dampingCompression: number;
  /** Damping while the strut opens, N s / m. */
  dampingRebound: number;
  /** Unloaded tire radius, m. */
  wheelRadius: number;
  /** Vertical tire rate, N / m. */
  tireStiffness: number;
  /** True for the nose leg. The steering command turns it. */
  steerable: boolean;
  /** True for the main legs. They carry the brakes. */
  braked: boolean;
  /** Vertical load that bursts the tire, N. */
  burstLoad: number;
}

/** What one leg is doing right now. */
export interface GearLegState {
  /** Strut stroke, m. Zero at full extension. */
  compression: number;
  /** Rate of the strut stroke, m/s. Positive while the strut closes. */
  compressionRate: number;
  /** Vertical load through the leg, N. Never negative. */
  load: number;
  onGround: boolean;
  /** (wheel speed - ground speed) / reference speed. -1 is a locked wheel. */
  slipRatio: number;
  /** Angle between the wheel plane and the path of the contact patch, rad. */
  slipAngle: number;
  /** Speed of the tread at the contact patch, m/s. It is omega * radius. */
  wheelSpeed: number;
  burst: boolean;
  /** Brake pack temperature, K. */
  brakeTemp: number;
}

export interface LandingGear {
  readonly legs: readonly GearLegState[];
  /**
   * Adds the ground wrench of every leg into `out`, in BODY axes.
   *
   * `gearPosition` is 0 when the gear is up and 1 when it is down and locked.
   * `steering` runs from -1 to 1, and positive turns the nose wheel RIGHT.
   * `brakeLeft` and `brakeRight` run from 0 to 1.
   */
  update(
    state: RigidBodyState,
    gearPosition: number,
    steering: number,
    brakeLeft: number,
    brakeRight: number,
    dt: number,
    out: Wrench,
  ): void;
  readonly anyOnGround: boolean;
  reset(): void;
}

// ---------------------------------------------------------------------------
// Oleo constants
// ---------------------------------------------------------------------------

/**
 * Fraction of the gas column the piston sweeps over the full travel.
 *
 * The force at the hard stop is (1 - GAS_FILL) ^ -POLYTROPIC_INDEX times the
 * force at full extension. At 0.9 that ratio is 19.9, which is the range a
 * single stage oleo pneumatic strut works over.
 * Source: Currey, "Aircraft Landing Gear Design", chapter 5. Confidence: firm
 * on the form of the law, estimate on the value.
 */
const GAS_FILL = 0.9;

/**
 * Polytropic index of the trapped gas.
 *
 * A slow compression is near isothermal at 1.0 and a landing stroke is near
 * adiabatic at 1.4. A landing takes about a fifth of a second, so the value sits
 * between the two. Source: Currey, chapter 5. Confidence: estimate.
 */
const POLYTROPIC_INDEX = 1.3;

/**
 * Rate of the metal on metal stop at the end of the travel, N / m.
 *
 * The stop is not a spring, it is structure. The value is high enough that the
 * strut stops and low enough that the step stays stable at 240 Hz.
 */
const BOTTOM_STOP_STIFFNESS = 4e6; // N/m

/**
 * Largest damper force, as a multiple of the gas preload of the same leg.
 *
 * A real oleo carries a relief valve so that a very fast closure cannot spike
 * the load without limit. The cap does the same job and it holds the step
 * stable if the aircraft arrives far outside the design case.
 */
const DAMPER_RELIEF_RATIO = 30;

/**
 * Tire damping, as a time constant against the tire rate. A tire returns about
 * 90 percent of the energy it takes, so its damping is light.
 * Confidence: estimate.
 */
const TIRE_DAMPING_TIME = 0.006; // s

/** Iterations of the bisection that splits the deflection over the two springs. */
const SPLIT_ITERATIONS = 20;

/**
 * Smallest vertical component of the strut axis that still carries a load. Past
 * about 70 degrees of bank or pitch the leg cannot hold the aircraft up, and the
 * model reports no contact instead of dividing by a very small number.
 */
const MIN_STRUT_VERTICAL = 0.3;

/** Below this `gearPosition` the leg is inside the bay and makes no force. */
const GEAR_DOWN_THRESHOLD = 1e-4;

// ---------------------------------------------------------------------------
// Tire constants
// ---------------------------------------------------------------------------

/**
 * Peak longitudinal friction coefficient on dry concrete.
 * Source: ESDU 71025, braking friction of aircraft tires. Confidence: firm.
 */
const LONG_PEAK_MU = 0.8;

/**
 * Shape factor of the longitudinal curve. It fixes how far the curve falls after
 * the peak: the value at unbounded slip is sin(C * pi / 2) times the peak, which
 * is 0.757 at C = 1.45. A locked tire really does lose about a fifth of its grip.
 */
const LONG_SHAPE = 1.45;

/** Slip ratio at the peak. Aircraft tires peak near 0.10 to 0.15. */
const LONG_PEAK_SLIP = 0.12;

/** Peak lateral friction coefficient. A tire corners a little worse than it brakes. */
const LAT_PEAK_MU = 0.75;

/** Shape factor of the lateral curve. The lateral tail falls less than the long one. */
const LAT_SHAPE = 1.3;

/** Slip angle at the peak of the lateral curve. Confidence: estimate. */
const LAT_PEAK_ANGLE = 8 * DEG; // rad

/** Stiffness factors that place the peak of each magic formula curve. */
const LONG_STIFFNESS = Math.tan(Math.PI / (2 * LONG_SHAPE)) / LONG_PEAK_SLIP;
const LAT_STIFFNESS = Math.tan(Math.PI / (2 * LAT_SHAPE)) / LAT_PEAK_ANGLE;

/**
 * Floor under the speed in the slip denominators, m/s.
 *
 * The slip ratio and the slip angle both divide by the ground speed, so both go
 * to infinity as the aircraft stops. The floor holds them finite, and it also
 * sets the STIFFNESS of the friction at low speed, which is what fixes the
 * value. Near zero slip the friction acts as a damper of
 *
 *   c = (d mu / d slip) * load / floor
 *
 * and an explicit step is stable only while c * dt * (1/m + r^2 / I) stays below
 * one, where r is the height of the contact patch below the center of gravity.
 * At 240 Hz, at the design weight, a floor of 1 m/s gives a loop gain of 1.04
 * and the aircraft chatters between two steps forever. A floor of 3 m/s gives
 * 0.35 and it settles.
 *
 * The floor costs nothing above 3 m/s, because the real ground speed then wins.
 * Below it a locked wheel still reaches its full sliding friction at 3 m/s and
 * its peak friction at 0.36 m/s, so the brakes still stop the aircraft.
 */
const SLIP_REFERENCE_SPEED = 3; // m/s

/** Same floor for the slip angle. The lateral curve is as stiff as the other. */
const SLIP_ANGLE_REFERENCE_SPEED = 3; // m/s

/** Rolling resistance of a tire on concrete. Source: Roskam, part V. */
const ROLLING_RESISTANCE = 0.02;

/** Speed over which the rolling resistance reaches its full value, m/s. */
const ROLL_BLEND_SPEED = 0.3; // m/s

/**
 * Creep band of the brake, m/s.
 *
 * Coulomb friction has one sign on one side of zero speed and the other sign on
 * the other side. A fixed step lands on one side, then the other, and the force
 * chatters at full size forever. The band takes the braking force smoothly to
 * zero over the last 0.2 m/s instead, so the aircraft coasts to a stop and
 * stays there.
 */
const CREEP_SPEED = 0.2; // m/s

/**
 * What a burst tire does. The carcass is gone, so the rim rides on the runway.
 * The grip falls, the drag climbs and the leg goes hard. Confidence: estimate.
 */
const BURST_GRIP_FACTOR = 0.55;
const BURST_ROLLING_RESISTANCE = 0.18;
const BURST_TIRE_STIFFNESS_FACTOR = 3;

/**
 * Polar moment of one wheel about its axle, as I = WHEEL_INERTIA_FACTOR * r^4.
 *
 * A wheel and a tire scale with the square of the radius in mass and with the
 * square of the radius again in the arm, so the moment goes as the fourth power.
 * The factor gives 4.0 kg m2 for the 0.42 m main wheel and 1.5 kg m2 for the
 * 0.33 m nose wheel. Confidence: estimate.
 */
const WHEEL_INERTIA_FACTOR = 128; // kg / m2

/** Bearing drag of a free wheel, as a spin down time constant. */
const WHEEL_SPIN_DOWN_TIME = 20; // s

// ---------------------------------------------------------------------------
// Brake constants
// ---------------------------------------------------------------------------

/**
 * Brake torque of one main wheel at a full command and a cold pack, N m.
 *
 * The wheel must be able to lock. A main leg carries 28.8 kN at rest, so the
 * tire can pass 0.8 * 28.8 = 23.0 kN, which is 9.7 kN m at the 0.42 m radius.
 * The value above that lets the pilot lock the wheel and lose grip, which is the
 * behavior the tire curve exists to show. Confidence: estimate.
 */
const MAX_BRAKE_TORQUE = 12000; // N m

/**
 * Heat capacity of one brake pack, J / K. A steel disc pack and its carrier of
 * about 45 kg, at 550 J per kg per K. A hard stop from 70 m/s puts about 8 MJ
 * into each main brake, which is a rise near 320 K. Confidence: estimate.
 */
const BRAKE_HEAT_CAPACITY = 25000; // J/K

/** Cooling conductance of one brake pack to the air, W / K. */
const BRAKE_COOLING = 210; // W/K

/** Air temperature the brake cools toward, K. ISA sea level. */
const AMBIENT_TEMPERATURE = 288.15; // K

/** Temperature where the pack starts to fade and where the fade is complete. */
const BRAKE_FADE_START = 575; // K
const BRAKE_FADE_FULL = 950; // K

/** Fraction of the cold torque the pack loses at BRAKE_FADE_FULL. */
const BRAKE_FADE_DEPTH = 0.65;

// ---------------------------------------------------------------------------
// Me 262 A-1a gear geometry
// ---------------------------------------------------------------------------

/**
 * Nose tip to center of gravity, m, and the height of the center of gravity
 * above the fuselage reference plane, m.
 *
 * DUPLICATED from CG_OFFSET_FROM_NOSE and CG_HEIGHT_FROM_DATUM of
 * src/aircraft/me262/mass.ts. CONVENTIONS section 4 keeps src/physics below
 * src/aircraft, so the two numbers appear here as well. The unit test asserts
 * the literal values so that the files cannot drift apart in silence.
 */
const CG_STATION = 5.76; // m
const CG_HEIGHT = -0.1333; // m

/**
 * The gear layout, DUPLICATED from src/render/models/me262.ts. That file draws
 * the aircraft, and CONVENTIONS section 4 stops the physics from importing it.
 * The render frame has x right, y up and z aft of the wing root quarter chord,
 * so a station is `z + CG_STATION` and a height is `y`.
 *
 *   NOSE_AXLE       (0, -1.00, aft(2.18))    NOSE_WHEEL_RADIUS   0.33
 *   MAIN_AXLE       (+-1.18, -0.91, 0.32)    MAIN_WHEEL_RADIUS   0.42
 *   NOSE_TRUNNION   (0, -0.42, aft(2.25))
 *   MAIN_TRUNNION   (+-1.18, -0.14, 0.52)
 *   GROUND_Y        -1.33
 *
 * Both wheels touch GROUND_Y, so the render model stands level on a flat plane.
 */
const NOSE_WHEEL_RADIUS = 0.33; // m
const MAIN_WHEEL_RADIUS = 0.42; // m
const NOSE_AXLE_STATION = 2.18; // m aft of the nose tip
const MAIN_AXLE_STATION = CG_STATION + 0.32; // m aft of the nose tip
const MAIN_TRACK_HALF = 1.18; // m
const NOSE_TRUNNION_HEIGHT = -0.42; // m above the reference plane
const MAIN_TRUNNION_HEIGHT = -0.14; // m above the reference plane
const RENDER_GROUND_HEIGHT = -1.33; // m above the reference plane

/**
 * Depth of the tire contact patch below the center of gravity when the aircraft
 * stands at rest, m. It is the ground line of the render model carried into body
 * axes, so the physics parks the aircraft exactly where the model draws it.
 */
const STATIC_CONTACT_DEPTH = CG_HEIGHT - RENDER_GROUND_HEIGHT; // 1.1967 m

/** Design mass of the aircraft. Source: CONVENTIONS section 8, firm. */
const DESIGN_MASS = 6396; // kg

/** Design weight, N. */
const DESIGN_WEIGHT = DESIGN_MASS * G0;

/** Body x of each contact patch. The strut is vertical, so it is the axle x. */
const NOSE_CONTACT_X = CG_STATION - NOSE_AXLE_STATION; // +3.58 m
const MAIN_CONTACT_X = CG_STATION - MAIN_AXLE_STATION; // -0.32 m

/**
 * Static load share of the nose leg.
 *
 * A moment balance about the center of gravity gives
 * NOSE_CONTACT_X / (NOSE_CONTACT_X - MAIN_CONTACT_X) = 0.32 / 3.90 = 0.0821.
 * Bead b17 reports about 8 percent from the same balance. A nose share below
 * 6 percent gives no steering grip and one above 20 percent needs a very strong
 * nose leg, so the layout sits where a tricycle layout should sit.
 */
const NOSE_LOAD_FRACTION = -MAIN_CONTACT_X / (NOSE_CONTACT_X - MAIN_CONTACT_X);

/** Static load of the nose leg and of ONE main leg, N. */
const NOSE_STATIC_LOAD = NOSE_LOAD_FRACTION * DESIGN_WEIGHT;
const MAIN_STATIC_LOAD = 0.5 * (1 - NOSE_LOAD_FRACTION) * DESIGN_WEIGHT;

/**
 * Full stroke of each leg, m. A fighter oleo of this size runs 0.2 m to 0.3 m.
 * The stroke has to swallow the energy of a 3 m/s touchdown inside the design
 * load factor, and these values do. Confidence: estimate.
 */
const NOSE_TRAVEL = 0.28; // m
const MAIN_TRAVEL = 0.28; // m

/**
 * Where the strut sits at rest, as a fraction of the full stroke.
 *
 * The aircraft has to sit low enough that a taxi bump has stroke to work in, and
 * high enough that the gas curve still has its stiff end left for a landing. At
 * 0.55 the gas force still climbs by a factor of 8 before the strut bottoms.
 */
const STATIC_STROKE_FRACTION = 0.55;

/**
 * Vertical rate of each tire, N / m.
 *
 * The A-1a ran a 840 x 300 main tire and a 660 x 160 nose tire. A tire of that
 * size deflects about a third of its section height at its rated load, which
 * gives these rates. Source: sized from Roskam part IV tire tables.
 * Confidence: estimate.
 */
const NOSE_TIRE_STIFFNESS = 300000; // N/m
const MAIN_TIRE_STIFFNESS = 700000; // N/m

/**
 * Damping of each leg, as a fraction of critical damping at the static point.
 *
 * The compression value sets the peak load of a landing. Too little and the
 * strut bottoms, too much and the leg spikes in the first centimeters of stroke
 * and never uses the rest of it. At 0.55 a 3 m/s touchdown uses about three
 * quarters of the travel, which is where a landing gear should work.
 *
 * The rebound value is far smaller, which is the asymmetry a real oleo carries.
 * It is still large enough that the aircraft does not pogo: the pair together
 * cut the height of each bounce by more than ten to one.
 */
const COMPRESSION_DAMPING_RATIO = 0.55;
const REBOUND_DAMPING_RATIO = 0.18;

/**
 * Vertical load that bursts a tire, as a fraction of the gas force at the hard
 * stop of the same leg.
 *
 * The tire is the fuse. It fails a little before the strut runs out of travel,
 * so a hard arrival costs a tire and not the airframe. The reference is the
 * strut bottoming force rather than the static load, because the nose leg and
 * the main legs see very different dynamic multiples of their static load and
 * one fraction of the bottoming force fits both.
 * Confidence: estimate.
 */
const BURST_LOAD_FRACTION = 0.75;

/** Nose wheel steering limit. */
const MAX_STEER_ANGLE = 30 * DEG; // rad

// ---------------------------------------------------------------------------
// The strut curve
// ---------------------------------------------------------------------------

/** Force of the gas spring at stroke `s`, N. Includes the hard stop. */
function strutForce(def: GearLegDef, s: number): number {
  if (s <= 0) {
    return def.springGas;
  }
  const swept = Math.min(s, def.maxTravel);
  let force = def.springGas * Math.pow(1 - (GAS_FILL * swept) / def.maxTravel, -POLYTROPIC_INDEX);
  if (s > def.maxTravel) {
    force += BOTTOM_STOP_STIFFNESS * (s - def.maxTravel);
  }
  return force;
}

/** Slope of the gas curve at stroke `s`, N / m. */
function strutRate(def: GearLegDef, s: number): number {
  const swept = Math.min(Math.max(s, 0), def.maxTravel);
  const scale = GAS_FILL / def.maxTravel;
  let rate =
    def.springGas *
    POLYTROPIC_INDEX *
    scale *
    Math.pow(1 - scale * swept, -(POLYTROPIC_INDEX + 1));
  if (s > def.maxTravel) {
    rate += BOTTOM_STOP_STIFFNESS;
  }
  return rate;
}

/**
 * Splits a total deflection over the strut and the tire.
 *
 * The two springs sit in series, so they carry the same force and their
 * deflections add. The gas curve is not invertible in closed form, so the split
 * comes out of a bisection. The strut force rises with the stroke and the tire
 * force falls with it, so the difference crosses zero one time and the bisection
 * cannot miss it.
 *
 * The strut preload gives the interesting case first: while the tire force stays
 * below the preload the piston has not moved at all, and the whole deflection is
 * in the tire.
 */
function splitDeflection(def: GearLegDef, tireRate: number, deflection: number): number {
  if (deflection <= 0) {
    return 0;
  }
  if (tireRate * deflection <= def.springGas) {
    return 0;
  }
  let lo = 0;
  let hi = deflection;
  for (let i = 0; i < SPLIT_ITERATIONS; i++) {
    const mid = 0.5 * (lo + hi);
    if (strutForce(def, mid) - tireRate * (deflection - mid) > 0) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return 0.5 * (lo + hi);
}

// ---------------------------------------------------------------------------
// The tire curve
// ---------------------------------------------------------------------------

/**
 * Pacejka magic formula with the curvature factor E set to zero.
 *
 *   mu(x) = peak * sin(shape * atan(stiffness * x))
 *
 * The function is odd, it rises to `peak` at x = tan(pi / (2 * shape)) /
 * stiffness, and it falls to sin(shape * pi / 2) * peak far past the peak.
 * Source: Pacejka, "Tire and Vehicle Dynamics", chapter 4.
 */
export function magicFormula(x: number, peak: number, shape: number, stiffness: number): number {
  return peak * Math.sin(shape * Math.atan(stiffness * x));
}

/**
 * Turns a wanted friction coefficient back into the slip that gives it, on the
 * RISING side of the curve.
 *
 * The magic formula inverts in closed form:
 *
 *   mu = D sin(C atan(B x))   ->   x = tan(asin(mu / D) / C) / B
 *
 * asin only answers on the rising side, which is the side the wheel settles on.
 * The falling side is unstable: a wheel there loses grip as it slips more, so it
 * runs away to a full lock. `mu` above the peak has no answer at all, and that
 * is exactly the condition that locks the wheel.
 */
function magicFormulaInverse(mu: number, peak: number, shape: number, stiffness: number): number {
  return Math.tan(Math.asin(clamp(mu / peak, 0, 1)) / shape) / stiffness;
}

/** Longitudinal friction coefficient at a slip ratio. Positive drives forward. */
export function tireLongitudinalMu(slipRatio: number): number {
  return magicFormula(slipRatio, LONG_PEAK_MU, LONG_SHAPE, LONG_STIFFNESS);
}

/** Lateral friction coefficient at a slip angle, in radians. */
export function tireLateralMu(slipAngle: number): number {
  return magicFormula(slipAngle, LAT_PEAK_MU, LAT_SHAPE, LAT_STIFFNESS);
}

/** Slip ratio where the longitudinal curve peaks. The test reads it. */
export const TIRE_PEAK_SLIP_RATIO = LONG_PEAK_SLIP;

/** Slip angle where the lateral curve peaks, rad. */
export const TIRE_PEAK_SLIP_ANGLE = LAT_PEAK_ANGLE;

/** Nose wheel steering limit, rad. The user interface reads it. */
export const NOSE_STEER_LIMIT = MAX_STEER_ANGLE;

// ---------------------------------------------------------------------------
// The gear
// ---------------------------------------------------------------------------

/** Makes one leg state at full extension, cold and whole. */
function createLegState(): GearLegState {
  return {
    compression: 0,
    compressionRate: 0,
    load: 0,
    onGround: false,
    slipRatio: 0,
    slipAngle: 0,
    wheelSpeed: 0,
    burst: false,
    brakeTemp: AMBIENT_TEMPERATURE,
  };
}

class Gear implements LandingGear {
  readonly legs: GearLegState[];
  private readonly defs: readonly GearLegDef[];
  private grounded = false;

  constructor(defs: readonly GearLegDef[]) {
    this.defs = defs;
    this.legs = defs.map(() => createLegState());
  }

  get anyOnGround(): boolean {
    return this.grounded;
  }

  reset(): void {
    for (const leg of this.legs) {
      leg.compression = 0;
      leg.compressionRate = 0;
      leg.load = 0;
      leg.onGround = false;
      leg.slipRatio = 0;
      leg.slipAngle = 0;
      leg.wheelSpeed = 0;
      leg.burst = false;
      leg.brakeTemp = AMBIENT_TEMPERATURE;
    }
    this.grounded = false;
  }

  update(
    state: RigidBodyState,
    gearPosition: number,
    steering: number,
    brakeLeft: number,
    brakeRight: number,
    dt: number,
    out: Wrench,
  ): void {
    const down = clamp(gearPosition, 0, 1);
    const steer = clamp(steering, -1, 1);
    this.grounded = false;
    for (let i = 0; i < this.defs.length; i++) {
      const def = this.defs[i];
      // The left main reads brakeLeft and the right main reads brakeRight. The
      // nose leg has no brake at all.
      const command = def.braked ? clamp(def.position.y < 0 ? brakeLeft : brakeRight, 0, 1) : 0;
      this.updateLeg(def, this.legs[i], state, down, steer, command, dt, out);
      if (this.legs[i].onGround) {
        this.grounded = true;
      }
    }
  }

  private updateLeg(
    def: GearLegDef,
    leg: GearLegState,
    state: RigidBodyState,
    down: number,
    steer: number,
    brakeCommand: number,
    dt: number,
    out: Wrench,
  ): void {
    // The pack fades from its own temperature, so the fade uses the value the
    // last step left behind.
    const fade = 1 - BRAKE_FADE_DEPTH * smoothstep(BRAKE_FADE_START, BRAKE_FADE_FULL, leg.brakeTemp);
    const brakeTorque = MAX_BRAKE_TORQUE * brakeCommand * fade;
    const wheelInertia = WHEEL_INERTIA_FACTOR * Math.pow(def.wheelRadius, 4);

    if (down <= GEAR_DOWN_THRESHOLD) {
      // The leg is in the bay. It touches nothing and it adds nothing.
      leg.compression = 0;
      leg.compressionRate = 0;
      leg.load = 0;
      leg.onGround = false;
      leg.slipRatio = 0;
      leg.slipAngle = 0;
      leg.wheelSpeed = 0;
      updateBrakeTemperature(leg, 0, 0, dt);
      return;
    }

    // The contact patch at full extension, before the ground pushes back.
    const extension = down * (def.restLength + def.wheelRadius);
    contactBody.set(def.position.x, def.position.y, def.position.z + extension);
    sampleContact(state, contactBody, sample);

    // The strut works along the body z axis, which the attitude tilts.
    strutAxis.set(0, 0, 1).applyQuaternion(state.orientation);
    const vertical = strutAxis.z;

    if (sample.depth <= 0 || vertical < MIN_STRUT_VERTICAL) {
      leg.compression = 0;
      leg.compressionRate = 0;
      leg.load = 0;
      leg.onGround = false;
      leg.slipRatio = 0;
      leg.slipAngle = 0;
      // A free wheel keeps turning. The brake can still stop it in the air, so a
      // pilot who holds the brakes touches down on a locked wheel.
      let free = leg.wheelSpeed * Math.exp(-dt / WHEEL_SPIN_DOWN_TIME);
      const stopped = ((brakeTorque * def.wheelRadius) / wheelInertia) * dt;
      free = free > 0 ? Math.max(0, free - stopped) : Math.min(0, free + stopped);
      leg.wheelSpeed = free;
      updateBrakeTemperature(leg, brakeTorque, leg.wheelSpeed / def.wheelRadius, dt);
      return;
    }

    // --- The vertical leg ---------------------------------------------------

    const penetration = sample.depth / vertical;
    const tireRate = def.tireStiffness * (leg.burst ? BURST_TIRE_STIFFNESS_FACTOR : 1);
    const stroke = splitDeflection(def, tireRate, penetration);
    const tireDeflection = penetration - stroke;
    const springLoad = stroke > 0 ? strutForce(def, stroke) : tireRate * tireDeflection;

    // The whole leg closes at this rate. The strut and the tire share it in the
    // ratio of their compliances, so a stiff tire sends nearly all of it to the
    // strut and a strut that has not cracked open sends none of it.
    const closureRate = sample.velocity.z / vertical;
    const legRate = strutRate(def, stroke);
    const strutShare = stroke > 0 ? tireRate / (tireRate + legRate) : 0;
    const compressionRate = closureRate * strutShare;
    leg.compressionRate = compressionRate;
    leg.compression = stroke;

    const damping = compressionRate > 0 ? def.dampingCompression : def.dampingRebound;
    const relief = DAMPER_RELIEF_RATIO * def.springGas;
    const damperForce = clamp(damping * compressionRate, -relief, relief);
    // The tire has light damping of its own. It is the only damping there is
    // while the piston stands still, so a small bump still settles.
    const tireDamperForce = TIRE_DAMPING_TIME * def.tireStiffness * (closureRate - compressionRate);

    let load = springLoad + damperForce + tireDamperForce;
    if (load < 0) {
      load = 0;
    }
    leg.load = load;
    leg.onGround = true;
    if (load > def.burstLoad) {
      leg.burst = true;
    }

    // Move the contact patch up to the ground and take the moment arm from
    // there. The patch is where the force really acts.
    contactBody.z -= penetration;

    if (load <= 0) {
      leg.slipRatio = 0;
      leg.slipAngle = 0;
      updateBrakeTemperature(leg, brakeTorque, leg.wheelSpeed / def.wheelRadius, dt);
      return;
    }

    sampleContact(state, contactBody, sample);

    // --- The wheel plane ----------------------------------------------------

    const steerAngle = def.steerable ? steer * MAX_STEER_ANGLE : 0;
    wheelForward.set(Math.cos(steerAngle), Math.sin(steerAngle), 0).applyQuaternion(state.orientation);
    wheelForward.z = 0;
    const planar = wheelForward.length();

    worldForce.set(0, 0, -load);

    if (planar > 1e-6) {
      wheelForward.multiplyScalar(1 / planar);
      // Turn the forward direction 90 degrees about world down to get the right
      // hand direction of the wheel. In NED that map is (x, y) -> (-y, x).
      wheelRight.set(-wheelForward.y, wheelForward.x, 0);

      const forwardSpeed = sample.velocity.dot(wheelForward);
      const lateralSpeed = sample.velocity.dot(wheelRight);
      const slipReference = Math.max(Math.abs(forwardSpeed), SLIP_REFERENCE_SPEED);

      const grip = leg.burst ? BURST_GRIP_FACTOR : 1;
      const longPeak = LONG_PEAK_MU * grip;
      const latPeak = LAT_PEAK_MU * grip;

      // --- The wheel spin ---------------------------------------------------
      //
      // I * domega/dt = -F_long * r - T_brake, which in tread speed is
      //
      //   d(tread)/dt = -mu(slip) * load * r^2 / I - T_brake * r / I
      //
      // That equation is very stiff. A free 4 kg m2 wheel reaches free rolling
      // in about sixty microseconds and the step is 1/240 s, four thousand times
      // longer. An explicit step rings, and a wheel that lags one step behind
      // answers a CHANGE in ground speed with a large force, which acts as a
      // phantom mass of over a tonne and drives the aircraft into a limit cycle.
      //
      // The fix uses the fact that the equation HAS a fixed point and that no
      // trajectory can cross one. `settledTread` is that fixed point: the slip
      // where the ground force exactly balances the brake torque. The step runs
      // forward as usual and then clamps at the fixed point. Far from it the
      // wheel spins up at the right rate, which keeps the spin up drag of a
      // touchdown. At it the wheel sits exactly where the brake asks, which
      // gives the right braking force at any step size.
      const wheelToGround = (def.wheelRadius * def.wheelRadius) / wheelInertia;
      // The creep band takes the brake smoothly out as the aircraft stops.
      // Coulomb friction has one sign each side of zero speed, so without the
      // band the force chatters at full size between two steps forever.
      const creep = clamp(forwardSpeed / CREEP_SPEED, -1, 1);
      // Friction coefficient the brake asks the tire for. The creep band scales
      // the ASK, not the answer, so the fixed point below stays exact.
      const demand = (brakeTorque / (def.wheelRadius * load)) * Math.abs(creep);
      const settledTread =
        demand >= longPeak
          ? // The brake beats the best the tire can give. The wheel stops
            // turning and the tire slides. This is the skid.
            -forwardSpeed
          : -Math.sign(creep) *
            magicFormulaInverse(demand, longPeak, LONG_SHAPE, LONG_STIFFNESS) *
            slipReference;

      let tread = leg.wheelSpeed - forwardSpeed;
      const spinAcceleration =
        -wheelToGround *
        load *
        (magicFormula(tread / slipReference, longPeak, LONG_SHAPE, LONG_STIFFNESS) +
          demand * Math.sign(creep));
      const stepped = tread + spinAcceleration * dt;
      tread = (tread - settledTread) * (stepped - settledTread) < 0 ? settledTread : stepped;

      // A brake resists rotation. It stops the wheel, and it can never turn it
      // backward. The clamp says so. Without it a brake torque past what the
      // tire can pass keeps driving a stopped wheel the wrong way, and the slip
      // ratio runs away to minus ten.
      let spin = tread + forwardSpeed;
      if (forwardSpeed > 0) {
        spin = Math.max(0, spin);
      } else if (forwardSpeed < 0) {
        spin = Math.min(0, spin);
      }
      leg.wheelSpeed = spin;
      tread = spin - forwardSpeed;

      const slipRatio = tread / slipReference;
      const slipAngle = Math.atan2(
        lateralSpeed,
        Math.max(Math.abs(forwardSpeed), SLIP_ANGLE_REFERENCE_SPEED),
      );
      leg.slipRatio = slipRatio;
      leg.slipAngle = slipAngle;

      let muLong = magicFormula(slipRatio, longPeak, LONG_SHAPE, LONG_STIFFNESS);
      // The friction opposes the lateral slide, so the sign turns over.
      let muLat = -magicFormula(slipAngle, latPeak, LAT_SHAPE, LAT_STIFFNESS);

      // Friction ellipse. A tire that already brakes at its limit has nothing
      // left to steer with.
      const combined = Math.hypot(muLong / longPeak, muLat / latPeak);
      if (combined > 1) {
        muLong /= combined;
        muLat /= combined;
      }

      // Rolling resistance rides on top. It is not slip, it is the tread and the
      // carcass working themselves.
      const rolling = leg.burst ? BURST_ROLLING_RESISTANCE : ROLLING_RESISTANCE;
      muLong -= rolling * clamp(forwardSpeed / ROLL_BLEND_SPEED, -1, 1);

      worldForce.addScaledVector(wheelForward, muLong * load);
      worldForce.addScaledVector(wheelRight, muLat * load);
    } else {
      leg.slipRatio = 0;
      leg.slipAngle = 0;
    }

    updateBrakeTemperature(leg, brakeTorque, leg.wheelSpeed / def.wheelRadius, dt);
    addContactWrench(state, contactBody, worldForce, out);
  }
}

/**
 * Heats the brake pack from its own friction power and cools it toward the air.
 *
 * A LOCKED wheel makes no heat in the brake at all, because the pack no longer
 * slides. The energy goes into the tire instead. That is right, and it is why a
 * pilot who locks a wheel ruins the tire and not the brake.
 */
function updateBrakeTemperature(
  leg: GearLegState,
  brakeTorque: number,
  wheelRate: number,
  dt: number,
): void {
  const power = brakeTorque * Math.abs(wheelRate);
  const cooling = BRAKE_COOLING * (leg.brakeTemp - AMBIENT_TEMPERATURE);
  leg.brakeTemp += ((power - cooling) / BRAKE_HEAT_CAPACITY) * dt;
  if (leg.brakeTemp < AMBIENT_TEMPERATURE) {
    leg.brakeTemp = AMBIENT_TEMPERATURE;
  }
}

/** Builds a gear from a list of legs. */
export function createLandingGear(defs: readonly GearLegDef[]): LandingGear {
  return new Gear(defs);
}

// ---------------------------------------------------------------------------
// The Me 262 gear
// ---------------------------------------------------------------------------

/** Gas force at full extension that puts the static load at the design stroke. */
function gasPreload(staticLoad: number, maxTravel: number): number {
  const stroke = STATIC_STROKE_FRACTION * maxTravel;
  return staticLoad / Math.pow(1 - (GAS_FILL * stroke) / maxTravel, -POLYTROPIC_INDEX);
}

/**
 * Builds one leg from the load it has to carry.
 *
 * The damping comes out of the leg itself. The strut slope and the tire rate at
 * the static point give the series rate of the leg, the static load gives the
 * mass that leg carries, and the two give a critical damping value. The leg then
 * takes a fraction of that value in each direction. A leg tuned this way holds
 * the same feel whatever load it carries, and nothing has to be tuned twice.
 *
 * `restLength` is not free. It is the length that puts the contact patch of the
 * loaded leg exactly on the ground line of the render model, so the physics
 * parks the aircraft where src/render/models/me262.ts draws it.
 */
function buildLeg(
  name: string,
  contactX: number,
  contactY: number,
  trunnionHeight: number,
  wheelRadius: number,
  maxTravel: number,
  tireStiffness: number,
  staticLoad: number,
  steerable: boolean,
  braked: boolean,
): GearLegDef {
  const springGas = gasPreload(staticLoad, maxTravel);
  const staticStroke = STATIC_STROKE_FRACTION * maxTravel;
  const staticTire = staticLoad / tireStiffness;

  // Top of the strut, in body axes, from the render trunnion height.
  const topZ = CG_HEIGHT - trunnionHeight;
  // Depth of the unloaded contact patch below the center of gravity.
  const unloadedDepth = STATIC_CONTACT_DEPTH + staticStroke + staticTire;
  const restLength = unloadedDepth - wheelRadius - topZ;

  const def: GearLegDef = {
    name,
    position: new Vector3(contactX, contactY, topZ),
    restLength,
    maxTravel,
    springGas,
    dampingCompression: 0,
    dampingRebound: 0,
    wheelRadius,
    tireStiffness,
    steerable,
    braked,
    burstLoad: BURST_LOAD_FRACTION * springGas * Math.pow(1 - GAS_FILL, -POLYTROPIC_INDEX),
  };

  const seriesRate = 1 / (1 / strutRate(def, staticStroke) + 1 / tireStiffness);
  const carried = staticLoad / G0; // kg the leg holds up
  const critical = 2 * Math.sqrt(seriesRate * carried);
  def.dampingCompression = COMPRESSION_DAMPING_RATIO * critical;
  def.dampingRebound = REBOUND_DAMPING_RATIO * critical;
  return def;
}

/** The three legs of the Me 262 A-1a, nose first, then left main, then right. */
export function me262GearLegs(): GearLegDef[] {
  return [
    buildLeg(
      'nose',
      NOSE_CONTACT_X,
      0,
      NOSE_TRUNNION_HEIGHT,
      NOSE_WHEEL_RADIUS,
      NOSE_TRAVEL,
      NOSE_TIRE_STIFFNESS,
      NOSE_STATIC_LOAD,
      true,
      false,
    ),
    buildLeg(
      'main left',
      MAIN_CONTACT_X,
      -MAIN_TRACK_HALF,
      MAIN_TRUNNION_HEIGHT,
      MAIN_WHEEL_RADIUS,
      MAIN_TRAVEL,
      MAIN_TIRE_STIFFNESS,
      MAIN_STATIC_LOAD,
      false,
      true,
    ),
    buildLeg(
      'main right',
      MAIN_CONTACT_X,
      MAIN_TRACK_HALF,
      MAIN_TRUNNION_HEIGHT,
      MAIN_WHEEL_RADIUS,
      MAIN_TRAVEL,
      MAIN_TIRE_STIFFNESS,
      MAIN_STATIC_LOAD,
      false,
      true,
    ),
  ];
}

/** Builds the landing gear of the Me 262 A-1a. */
export function createMe262Gear(): LandingGear {
  return createLandingGear(me262GearLegs());
}

/**
 * Height of the center of gravity above the ground when the aircraft stands at
 * rest at the design mass, m. A test and a scenario loader both need it.
 */
export const ME262_STATIC_CG_HEIGHT = STATIC_CONTACT_DEPTH;

/** Share of the weight the nose leg carries at rest. */
export const ME262_NOSE_LOAD_FRACTION = NOSE_LOAD_FRACTION;

// Scratch held in module scope. The step allocates nothing.
const sample: ContactSample = createContactSample();
const contactBody = new Vector3();
const strutAxis = new Vector3();
const wheelForward = new Vector3();
const wheelRight = new Vector3();
const worldForce = new Vector3();
