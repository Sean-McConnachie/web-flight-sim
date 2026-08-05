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
 * A slip curve alone cannot hold a PARKED aircraft. Slip is a speed, so at zero
 * speed the curve returns zero force whatever the brake does, and the aircraft
 * then creeps forward under thrust until the slip catches up with it. A real
 * tire does not creep. It sticks, and the carcass twists instead. STICK_SLIP_LENGTH
 * below models that twist, and it is what lets the pilot run the engines up
 * against the brakes.
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
 * THIS FILE HOLDS THE ONLY FADE MODEL OF THE PROJECT, and `update` takes the RAW
 * pilot command. See the note above BRAKE_HEAT_CAPACITY.
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
import { clearWrench, createWrench } from '@/physics/rigidbody';
import type { AirframeContact, ContactPointDef, ContactSample } from '@/physics/contact';
import {
  MAX_GROUND_LOAD_FACTOR,
  addContactWrench,
  createAirframeContact,
  createContactSample,
  limitContactWrench,
  reversalLimitMu,
  sampleContact,
} from '@/physics/contact';

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
  /**
   * Elastic twist of the tire of a HELD wheel, m. See STICK_SLIP_LENGTH. It is
   * zero whenever the tire rolls or slides.
   */
  stickOffset: number;
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

/**
 * Speed below which the tire of a HELD wheel twists instead of sliding, m/s.
 *
 * Above this speed the wheel rolls or slides and the slip curve owns the force.
 * Below it the tread of a braked wheel stays where the ground put it. The value
 * is far below any speed a pilot taxis at, so nothing above it changes.
 */
const STICK_SPEED = 0.3; // m/s

/**
 * Twist of the tire carcass that reaches full grip, m.
 *
 * A tire that grips without sliding carries its load through the shear of the
 * tread and the carcass. The deflection at the limit of grip is about one
 * percent of the rolling radius on a tire of this size, which is the value
 * below. It sets the stiffness of the hold: the two main tires together give
 * 0.8 * 57.6 kN / 0.01 m, which is 4.6 MN/m, so the two engines at full power
 * move the parked aircraft 3.8 mm and no further.
 * Source: Pacejka, "Tire and Vehicle Dynamics", the brush model and the
 * longitudinal slip stiffness. Confidence: estimate.
 */
const STICK_SLIP_LENGTH = 0.01; // m

/** Damping of the stick, as a fraction of critical damping at that stiffness. */
const STICK_DAMPING_RATIO = 0.7;

/**
 * Damping of the stick as a friction coefficient per unit speed, s / m.
 *
 * The stick is a spring of `LONG_PEAK_MU * load / STICK_SLIP_LENGTH` against the
 * `load / G0` kilograms the leg holds up, so its critical damping is
 * `2 sqrt(k m)`, and the load cancels out of the coefficient. Without the
 * damper the spring is undamped, and a fixed step that holds the ground force
 * over the whole step feeds energy into an undamped spring.
 *
 * The value comes out at 4.0 s/m. At the design weight the two main legs then
 * damp at 230 kN per m/s, and the criterion recorded under
 * SLIP_REFERENCE_SPEED, `c * dt * (1/m + r^2/I) < 1`, gives 0.21 at 240 Hz.
 */
const STICK_DAMPING =
  2 * STICK_DAMPING_RATIO * Math.sqrt(LONG_PEAK_MU / (STICK_SLIP_LENGTH * G0));

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
 * ONE FILE HOLDS THIS NUMBER AND IT IS THIS ONE.
 * src/aircraft/me262/systems.ts carried a second value of 4500 N m for its
 * gauge and its heat model. That value never reached a wheel, because the
 * torque that makes a force is the one below, so the two disagreed by a factor
 * of 2.7 in silence. systems.ts now imports this constant.
 *
 * THE RUN UP MEASURES THIS BRAKE. The pilot notes ask the brakes to hold the
 * aircraft against both engines at full power, which is 17.6 kN, and the same
 * notes call the brakes weak. Two braked wheels share the thrust, so each
 * contact patch passes 8.8 kN, and at the 0.42 m rolling radius that is
 * 3.7 kN m. The value below sits just over that bound: 4.2 kN m holds 20.0 kN
 * against 17.6 kN, a margin of 14 percent. A brake that a pilot calls weak and
 * that just holds the run up is a brake near its bound, not one far above it.
 *
 * WHY BEAD b33 TOOK IT DOWN FROM 10000 N m. The old value came from a design
 * rule of the JET AGE: size the brake past the tire, so that full pedal reaches
 * the whole of the grip and an anti-skid unit modulates it. The Me 262 carried
 * no anti-skid, and 10 kN m would hold 47.6 kN, which is 2.7 times the thrust of
 * both engines. No pilot calls such a brake weak.
 *
 * The old value also killed the fade model below. A brake past the tire locks
 * the wheel at the first touch of the pedal. A locked wheel does not turn, so it
 * slides no lining against a drum and the pack takes NO energy at all: bead b33
 * measured 288 K at the start of a full brake landing roll and 294 K at the end
 * of it. The whole 7.6 MJ went into the tire, the roll ran 281 m at a mean of
 * 0.52 g, and the aircraft stopped harder than a modern jet with anti-skid.
 *
 * At 4200 N m the wheel keeps turning at a slip of 0.025, the pack takes
 * 3.6 MJ, it heats from 288 K to 541 K, and the fade takes the deceleration from
 * 0.36 g at the start of the roll to 0.29 g at the end. The roll runs 389 m.
 * That is the aircraft the pilot notes describe.
 *
 * Source: Wendel notes, paragraph 2, for the run up and for the weak brakes.
 * Confidence: estimate, bounded below by the run up.
 */
export const MAX_BRAKE_TORQUE = 4200; // N m

/**
 * THIS FILE HOLDS THE ONLY BRAKE FADE MODEL. src/aircraft/me262/systems.ts ran a
 * second one of its own, with its own heat capacity, its own cooling time and
 * its own fade curve, and BOTH applied: systems.ts multiplied the pilot command
 * by its own fade before it handed the command to `update` below, which then
 * multiplied the torque by this one. The brakes therefore faded about twice as
 * fast as either model meant them to. src/physics sits below src/aircraft and
 * cannot import upward, which is the same reason MAX_BRAKE_TORQUE lives here, so
 * this file keeps the model and systems.ts now passes the raw pilot command.
 *
 * Heat capacity of one brake pack, J / K.
 *
 * The Me 262 braked on a drum inside each 840 by 300 mm main wheel. The metal
 * that takes the heat of one stop is the drum, the shoes and the hub, about
 * 25 kg of steel, at a mean 500 J per kg per K over the working range.
 *
 * SIZE CHECK. The aircraft touches down at 175 km/h, which is 48.6 m/s, and at
 * 6396 kg it carries 7.6 MJ. The two brakes take at most half of that each, so
 * one pack takes 3.8 MJ and rises about 300 K. That puts a full effort landing
 * roll INSIDE the fade band below, which is what the type was known for: weak
 * brakes and a long landing run. A pack of twice this size would never reach the
 * band at all, and the fade would be a model that never acts.
 * Source: sized from the wheel of the A-1a. Confidence: estimate.
 */
const BRAKE_HEAT_CAPACITY = 12500; // J/K

/**
 * Cooling conductance of one brake pack to the air, W / K.
 *
 * A drum of about 0.5 m2 of wetted area in the wheel well passes 30 to 50 W/K to
 * the air on the roll, and it radiates a further 10 W/K at 700 K. The value is
 * one number for both, so the pack cools with a time constant of 280 s and
 * reaches the air again in about ten minutes. Confidence: estimate.
 */
const BRAKE_COOLING = 45; // W/K

/** Air temperature the brake cools toward, K. ISA sea level. */
const AMBIENT_TEMPERATURE = 288.15; // K

/**
 * Temperature where the pack starts to fade and where the fade is complete.
 *
 * The lining of 1944 is an asbestos and resin compound. Its friction holds to
 * about 200 C, falls as the resin binder starts to break down, and is gone by
 * about 400 C, where the binder has charred. The two anchors below are those
 * temperatures in kelvin.
 * Source: the fade behavior of asbestos resin friction linings.
 * Confidence: estimate on the values, firm on the shape.
 */
const BRAKE_FADE_START = 475; // K
const BRAKE_FADE_FULL = 675; // K

/**
 * Fraction of the cold torque the pack loses at BRAKE_FADE_FULL.
 *
 * A charred organic lining keeps about half of its cold friction. A larger depth
 * would say the brake stops working, and it does not: it gets weak.
 * Confidence: estimate.
 */
const BRAKE_FADE_DEPTH = 0.5;

/**
 * Fraction of the cold brake torque a pack at this temperature still gives.
 *
 * The gauge, the tests and `update` all read this one function, so no second
 * copy of the curve can appear anywhere.
 */
export function brakeFade(temperature: number): number {
  return 1 - BRAKE_FADE_DEPTH * smoothstep(BRAKE_FADE_START, BRAKE_FADE_FULL, temperature);
}

/** The two anchors of the fade curve, K. The unit test reads them. */
export const BRAKE_FADE_START_TEMPERATURE = BRAKE_FADE_START;
export const BRAKE_FADE_FULL_TEMPERATURE = BRAKE_FADE_FULL;

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
    stickOffset: 0,
    burst: false,
    brakeTemp: AMBIENT_TEMPERATURE,
  };
}

class Gear implements LandingGear {
  readonly legs: GearLegState[];
  private readonly defs: readonly GearLegDef[];
  private readonly maxForce: number;
  private grounded = false;

  constructor(defs: readonly GearLegDef[]) {
    this.defs = defs;
    this.legs = defs.map(() => createLegState());
    // The gas force of each leg at the design stroke IS the load that leg
    // carries at rest, because gasPreload inverts exactly that relation. The
    // sum is therefore the design weight, and the gear needs no other file to
    // tell it what the aircraft weighs.
    let weight = 0;
    for (const def of defs) {
      weight += strutForce(def, STATIC_STROKE_FRACTION * def.maxTravel);
    }
    this.maxForce = MAX_GROUND_LOAD_FACTOR * weight;
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
      leg.stickOffset = 0;
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
    // The legs collect into a wrench of their own, so that the cap below acts on
    // what the GEAR made and not on what the caller already holds.
    clearWrench(gearWrench);
    for (let i = 0; i < this.defs.length; i++) {
      const def = this.defs[i];
      // The left main reads brakeLeft and the right main reads brakeRight. The
      // nose leg has no brake at all.
      const command = def.braked ? clamp(def.position.y < 0 ? brakeLeft : brakeRight, 0, 1) : 0;
      this.updateLeg(def, this.legs[i], state, down, steer, command, dt, gearWrench);
      if (this.legs[i].onGround) {
        this.grounded = true;
      }
    }
    // The hard stop at the end of the travel is structure, not a spring, and it
    // is stiff enough to break the step on its own. See MAX_GROUND_LOAD_FACTOR
    // of src/physics/contact.ts for why the cap sits where it sits.
    limitContactWrench(gearWrench, this.maxForce);
    out.force.add(gearWrench.force);
    out.moment.add(gearWrench.moment);
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
    // last step left behind. `brakeCommand` is the RAW pilot command. Nothing
    // above this file fades it first. See BRAKE_HEAT_CAPACITY.
    const brakeTorque = MAX_BRAKE_TORQUE * brakeCommand * brakeFade(leg.brakeTemp);
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
      leg.stickOffset = 0;
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
      leg.stickOffset = 0;
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
      leg.stickOffset = 0;
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

      // --- The stick --------------------------------------------------------
      //
      // Everything above works on a slip, and a slip is a SPEED. At zero speed
      // it is zero, so the tire above makes no force at all however hard the
      // brake grips, and the aircraft creeps forward under thrust until the slip
      // catches up. A real tire does not creep. The tread stays where the ground
      // put it and the carcass twists, so the leg holds through an elastic
      // OFFSET instead of through a slip.
      //
      // The brake is what makes the hold possible, because it is the brake that
      // stops the wheel from turning the twist away. The hold is therefore the
      // smaller of what the tire can pass and what the brake torque can react,
      // and with no brake at all there is no hold and the wheel rolls free.
      //
      // The two Jumo 004 at full power make 17.6 kN. The main tires can pass
      // 0.8 * 57.6 = 46 kN and the brakes can react 10000 / 0.42 = 23.8 kN per
      // wheel, so both have room to spare and the aircraft stands still. That is
      // the run up the pilot notes ask for.
      const holdMu = Math.min(longPeak, brakeTorque / (def.wheelRadius * load));
      const stickWeight =
        holdMu > 0 ? 1 - smoothstep(0.5 * STICK_SPEED, STICK_SPEED, Math.abs(forwardSpeed)) : 0;
      let muStick = 0;
      if (stickWeight > 0) {
        // A held wheel does not turn, so what is left of the wheel speed goes
        // out with the same blend. The offset then collects the whole of the
        // travel of the aircraft, which is what a twist is.
        leg.wheelSpeed *= 1 - stickWeight;
        leg.stickOffset += (forwardSpeed - leg.wheelSpeed) * dt;
        const maxOffset = (STICK_SLIP_LENGTH * holdMu) / longPeak;
        leg.stickOffset = clamp(leg.stickOffset, -maxOffset, maxOffset);
        muStick =
          -stickWeight *
          ((longPeak * leg.stickOffset) / STICK_SLIP_LENGTH + STICK_DAMPING * forwardSpeed);
      } else {
        leg.stickOffset = 0;
      }
      tread = leg.wheelSpeed - forwardSpeed;

      const slipRatio = tread / slipReference;
      const slipAngle = Math.atan2(
        lateralSpeed,
        Math.max(Math.abs(forwardSpeed), SLIP_ANGLE_REFERENCE_SPEED),
      );
      leg.slipRatio = slipRatio;
      leg.slipAngle = slipAngle;

      let muLong = magicFormula(slipRatio, longPeak, LONG_SHAPE, LONG_STIFFNESS) + muStick;
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
      //
      // It is COULOMB, so it holds its full value down to zero speed and it
      // opposes the travel on both sides of it. A model that faded it out over a
      // speed band instead could not hold a parked aircraft: the resistance is
      // 0.02 * 62.7 kN = 1254 N and the two engines make 461 N at idle, so the
      // aircraft must stand still, and a 0.3 m/s band let it roll at 0.12 m/s
      // for ever. reversalLimitMu is the only thing that takes the value down,
      // and it only does so where the force would drive the wheel BACKWARD
      // inside one step.
      const rolling = leg.burst ? BURST_ROLLING_RESISTANCE : ROLLING_RESISTANCE;
      muLong -= Math.sign(forwardSpeed) * Math.min(rolling, reversalLimitMu(forwardSpeed, dt));

      worldForce.addScaledVector(wheelForward, muLong * load);
      worldForce.addScaledVector(wheelRight, muLat * load);
    } else {
      leg.slipRatio = 0;
      leg.slipAngle = 0;
      leg.stickOffset = 0;
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

/**
 * Drag share of the two main legs, against the nose leg.
 *
 * The frontal area of a tire is its diameter times its width. The two 840 x 300
 * main tires give 0.504 m2 and the 660 x 160 nose tire gives 0.106 m2, so the
 * mains carry 0.83 of the frontal area of the three wheels. The legs and the
 * doors follow the same order of size, so one fraction covers the whole
 * assembly. Confidence: estimate.
 */
const MAIN_GEAR_DRAG_SHARE = 0.83;

/** Body z of the top of each leg. buildLeg works out the same value. */
const MAIN_LEG_TOP_Z = CG_HEIGHT - MAIN_TRUNNION_HEIGHT;
const NOSE_LEG_TOP_Z = CG_HEIGHT - NOSE_TRUNNION_HEIGHT;

/**
 * Where the drag of the extended gear acts, body axes, m from the center of
 * gravity.
 *
 * The point is the drag weighted mean of the three legs. Along body x it is the
 * mean of the two contact patches, because the wheels and the doors sit there.
 * Along body z it is the mean of the mid height of each leg, because the wheel
 * sits at the foot of the leg and the strut and the door spread up from it.
 *
 * THE HEIGHT IS WHY THIS CONSTANT EXISTS. The gear hangs 0.63 m below the center
 * of gravity, so its drag makes a NOSE DOWN moment that grows with the square of
 * the speed. The pilot feels that moment when the gear comes down. A model that
 * puts the gear drag at the center of gravity shows none of it.
 * Confidence: estimate.
 */
export const ME262_GEAR_DRAG_POSITION = new Vector3(
  MAIN_GEAR_DRAG_SHARE * MAIN_CONTACT_X + (1 - MAIN_GEAR_DRAG_SHARE) * NOSE_CONTACT_X,
  0,
  MAIN_GEAR_DRAG_SHARE * 0.5 * (MAIN_LEG_TOP_Z + STATIC_CONTACT_DEPTH) +
    (1 - MAIN_GEAR_DRAG_SHARE) * 0.5 * (NOSE_LEG_TOP_Z + STATIC_CONTACT_DEPTH),
);

// ---------------------------------------------------------------------------
// The Me 262 airframe contact points
// ---------------------------------------------------------------------------

/**
 * Turns a station and a height into a body axis position, exactly as
 * bodyPosition of src/aircraft/me262/geometry.ts does. A station runs aft from
 * the nose tip and a height runs up from the fuselage reference plane.
 */
function airframePoint(name: string, station: number, y: number, height: number): ContactPointDef {
  return { name, position: new Vector3(CG_STATION - station, y, CG_HEIGHT - height) };
}

/**
 * The seven points of the airframe that the ground can push on.
 *
 * Every number comes from the geometry the rest of the project already holds:
 * FUSELAGE_SECTIONS of src/aircraft/me262/mass.ts for the fuselage underside,
 * the plan form of src/aircraft/me262/geometry.ts for the wing tips, and the
 * nacelle of the same file. CONVENTIONS section 4 keeps src/physics below
 * src/aircraft, so the values appear here in the same DUPLICATED form the gear
 * geometry above already uses.
 *
 *   name        station   y        height   depth below the CG
 *   nose        0.00      0        +0.045   -0.18   the tip, and its underside
 *   belly       5.10      0        -0.810   +0.68   the lowest fuselage section
 *   nacelle     5.81      +-2.05   -0.955   +0.82   the bottom of the cowling
 *   wing tip    6.94      +-6.255  +0.248   -0.38   the tip, dihedral included
 *   tail        10.35     0        -0.050   -0.08   the aftmost low structure
 *
 * Read the last column against the 1.1967 m that the tire contact patch sits
 * below the center of gravity. Every point stands clear when the aircraft parks.
 * The nacelles are the lowest of the seven, 0.37 m above the runway with the
 * gear down, so a gear up arrival lands on the two nacelles first. That is what
 * the Me 262 really did, and it is why the type survived so many belly landings.
 *
 * The angles follow from the same table. The tail strikes at 16.7 degrees of
 * pitch about the main axle and a wing tip strikes at 17.3 degrees of bank, both
 * far outside anything the aircraft does on a normal takeoff or landing.
 */
export function me262ContactPoints(): ContactPointDef[] {
  return [
    airframePoint('nose', 0, 0, 0.045),
    airframePoint('belly', 5.1, 0, -0.81),
    airframePoint('nacelle left', 5.81, -2.05, -0.955),
    airframePoint('nacelle right', 5.81, 2.05, -0.955),
    airframePoint('wing tip left', 6.943, -6.255, 0.248),
    airframePoint('wing tip right', 6.943, 6.255, 0.248),
    airframePoint('tail', 10.35, 0, -0.05),
  ];
}

/** Builds the airframe contact set of the Me 262 A-1a. */
export function createMe262AirframeContact(): AirframeContact {
  return createAirframeContact(me262ContactPoints(), DESIGN_WEIGHT);
}

// Scratch held in module scope. The step allocates nothing.
const sample: ContactSample = createContactSample();
const gearWrench: Wrench = createWrench();
const contactBody = new Vector3();
const strutAxis = new Vector3();
const wheelForward = new Vector3();
const wheelRight = new Vector3();
const worldForce = new Vector3();
