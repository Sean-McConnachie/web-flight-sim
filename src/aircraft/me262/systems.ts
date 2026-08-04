/**
 * The moving parts of the Messerschmitt Me 262 A-1a: flaps, landing gear,
 * automatic leading edge slats, wheel brakes and the fuel system.
 *
 * Nothing here computes an aerodynamic force. The module holds the POSITION of
 * every moving part, moves each one at the rate the real machine moved it, and
 * hands the two positions the aerodynamics needs to the control array of
 * src/physics/aero/assembly.ts. The forces stay where they belong, in
 * src/physics/aero.
 *
 *
 * NOTHING SNAPS
 *
 * Every part of this module moves at a rate. A flap that jumps from up to down
 * in one step gives the aircraft a step in lift and a step in pitching moment,
 * which no airframe can make and which the integrator answers with a jolt. The
 * flap of this aircraft takes about eight seconds and the landing gear takes
 * about fifteen. Both are slow enough to fly through.
 *
 *
 * THE SLAT HAS NO PILOT CONTROL
 *
 * The slat of the Me 262 carried no actuator and no cockpit lever. Low pressure
 * at the nose of the wing pulled it out against a spring, and the airload pushed
 * it back in when the angle of attack fell. This module therefore drives the
 * slat from the local angle of attack of the outer wing alone. The caller passes
 * that angle into update. There is no commandSlats function, because the real
 * aircraft had no such command.
 *
 * The threshold carries HYSTERESIS. The slat opens at SLAT_DEPLOY_ALPHA of
 * src/aircraft/me262/geometry.ts and shuts again 2 degrees below it. Without the
 * band a slat sitting at the threshold would open and shut at 240 Hz, which is
 * both wrong and loud. The band is what the rail friction and the changed
 * pressure field of the open slot give in the real mechanism.
 *
 * WHERE THE SLAT MAKES ITS LIFT. The aerodynamic slat lives in
 * src/physics/aero/surface.ts, which opens the slat of each outer strip from the
 * local angle of attack of THAT strip. This module does not drive it. The
 * position here is the position of the mechanism, for the render model, for the
 * gauge and for a later bead that models a slat that jams. The two agree in
 * steady flight and differ for the fraction of a second the mechanism takes to
 * run out.
 *
 *
 * WHAT THE CALLER MUST DO WITH THE GEAR DRAG
 *
 * gearDragArea reports an equivalent flat plate area in square meters. The
 * flight model multiplies it by the dynamic pressure to get a drag force along
 * the relative wind. The area PEAKS IN TRANSIT, because the doors stand open
 * across the flow while the leg is halfway down.
 *
 *
 * ALLOCATION
 *
 * update allocates nothing. Every value lives in the state object or in a
 * closure variable. The state object and its damage record are built one time.
 *
 * This module is pure physics. It imports nothing from the renderer and touches
 * no browser API.
 */

import { clamp, smoothstep } from '@/math/tables';
import { DEG, kmhToMs } from '@/math/units';
import {
  CONTROL_INDEX,
  FLAP_LANDING_ANGLE,
  FLAP_TAKEOFF_ANGLE,
  SLAT_DEPLOY_ALPHA,
} from '@/aircraft/me262/geometry';
import { FUEL_CAPACITY } from '@/aircraft/me262/mass';

// ---------------------------------------------------------------------------
// Flaps.
// ---------------------------------------------------------------------------

/**
 * Where each setting puts the flap, as a fraction of the full travel.
 *
 * The flap carries graduations at 0, 10, 20, 30, 40 and 50 degrees on its upper
 * surface and the 20 degree take off setting is marked in red. The model uses
 * the three settings a pilot selects: up, take off and landing.
 * Source: "Pilot's Handbook for Me-262 A-1", section 2, wing flaps.
 * Confidence: firm.
 */
const FLAP_TAKEOFF_FRACTION = FLAP_TAKEOFF_ANGLE / FLAP_LANDING_ANGLE; // 0.4

/**
 * Time the flap needs for the whole travel, from up to the landing setting.
 *
 * The flap runs on the same hydraulic pump as the landing gear, and Wendel
 * reports that pump as too small for the job. No document in the reference set
 * times the flap. Eight seconds for 50 degrees is 6.25 degrees per second, which
 * matches the pace of a pilot who watches the graduations on the flap and lets
 * go of the button at the setting he wants.
 * Source: estimate from the pilot notes. Confidence: low.
 */
export const FLAP_TRAVEL_TIME = 8; // s

/**
 * Highest equivalent airspeed the flap takes at the landing setting and at the
 * take off setting.
 *
 * No wartime document in the reference set gives a flap limit. Wendel writes
 * only that "the high speed of the aircraft easily tempts one to lower the
 * undercarriage or flaps whilst travelling too fast and this leads to damage".
 * The IL-2 and War Thunder data sets, which both work from captured
 * Messerschmitt material, use 380 km/h at the landing setting and about 530 km/h
 * at the take off setting. The model takes both. Confidence: low.
 */
export const FLAP_LIMIT_SPEED_LANDING = kmhToMs(380); // m/s EAS
export const FLAP_LIMIT_SPEED_TAKEOFF = kmhToMs(530); // m/s EAS

// ---------------------------------------------------------------------------
// Landing gear.
// ---------------------------------------------------------------------------

/**
 * Time the gear needs for the whole travel.
 *
 * The emergency compressed air system puts the main gear down in 2 to 3 seconds
 * and the nose wheel down in 5 to 10 seconds. The handbook and Wendel both say
 * the normal hydraulic system is far slower than that, because the 18 litre per
 * minute pump is too small and drives the flaps as well. Fifteen seconds is the
 * model value.
 * Source: "Pilot's Handbook for Me-262 A-1", emergency operation, and the
 * Wendel notes, paragraph 4. Confidence: estimate.
 */
export const GEAR_TRAVEL_TIME = 15; // s

/**
 * Highest equivalent airspeed at which the gear may come down.
 *
 * The handbook says "Do not lower landing gear above 4000 km/hr (248 mph)". The
 * metric value carries an obvious extra zero. 248 mph is 400 km/h, so the limit
 * is 400 km/h.
 * Source: "Pilot's Handbook for Me-262 A-1", approach and landing.
 * Confidence: firm.
 */
export const GEAR_LIMIT_SPEED = kmhToMs(400); // m/s EAS

/**
 * Equivalent flat plate drag area of the gear when it is fully down.
 *
 * A tricycle gear of this size on a 21.7 m2 wing adds about 0.020 to the zero
 * lift drag coefficient of the aircraft, which is 0.43 m2 of flat plate area.
 * Source: Hoerner, "Fluid Dynamic Drag", chapter 13, landing gear.
 * Confidence: estimate.
 */
export const GEAR_DRAG_AREA = 0.45; // m2

/**
 * Extra drag area at the middle of the travel.
 *
 * The doors stand open across the flow and the leg hangs broadside while the
 * gear is halfway down, so a gear in transit drags MORE than a gear that is
 * locked down. The bump is a third of the down value at the middle of the
 * travel and it is zero at both ends. Confidence: estimate.
 */
export const GEAR_TRANSIT_DRAG_AREA = 0.15; // m2

/**
 * How the nose gear and the main gear share the travel.
 *
 * The nose wheel comes down very much later than the main gear, because the same
 * small pump feeds both and the nose leg works against the airstream. The mains
 * run over the first part of the cycle and the nose runs over the last part.
 * Source: Wendel notes, paragraph 4, and the handbook. Confidence: estimate.
 */
const MAIN_GEAR_SPAN = 0.55;
const NOSE_GEAR_START = 0.35;

// ---------------------------------------------------------------------------
// Slats.
// ---------------------------------------------------------------------------

/**
 * The angle of attack band of the slat mechanism.
 *
 * The slat opens at the angle the strips of src/physics/aero/surface.ts use and
 * shuts 2 degrees below it. The band stops the mechanism from opening and
 * shutting at the step rate when the angle of attack sits on the threshold.
 * Confidence: estimate.
 *
 * A cross check on the open angle. The handbook says the slots open at 300 km/h
 * in a glide and at 450 km/h in a climb or a turn. At 6400 kg and 300 km/h in a
 * 1 g glide the aircraft carries a lift coefficient of 0.68, and at 450 km/h it
 * reaches the same coefficient at 2.2 g. One angle of attack explains both
 * numbers, which is what an angle driven mechanism must do.
 */
export const SLAT_OPEN_ALPHA = SLAT_DEPLOY_ALPHA; // rad
export const SLAT_HYSTERESIS = 2 * DEG; // rad
export const SLAT_CLOSE_ALPHA = SLAT_DEPLOY_ALPHA - SLAT_HYSTERESIS; // rad

/**
 * Time the slat needs to run out along its rails.
 *
 * The slat is small, it carries no actuator, and the airload that pulls it out
 * grows fast above the threshold. Half a second is the model value.
 * Confidence: estimate.
 */
export const SLAT_TRAVEL_TIME = 0.5; // s

// ---------------------------------------------------------------------------
// Wheel brakes.
// ---------------------------------------------------------------------------

/**
 * Largest braking torque of one main wheel.
 *
 * The pilot notes require the brakes to hold the aircraft against both engines
 * at 8500 rpm, which is about 15 kN of thrust. Two main wheels of 0.42 m rolling
 * radius need 3150 N m each to do that. The model carries 4500 N m, which leaves
 * the margin a serviceable brake had.
 * Source: Wendel notes, paragraph 2. Confidence: estimate.
 */
export const BRAKE_TORQUE_MAX = 4500; // N m

/** Rolling radius of the main wheel, 840 by 300 mm tire. Confidence: firm. */
export const WHEEL_RADIUS = 0.42; // m

/**
 * Heat capacity of one brake, wheel and leg together.
 *
 * A full stop from 60 m/s at 6400 kg puts 11.5 MJ into the two brakes. With
 * 30 kJ per kelvin in each one, that stop raises each brake by about 190 K,
 * which is where the fade below starts. The Me 262 was known for weak brakes and
 * long landing runs, so the model must show that. Confidence: estimate.
 */
export const BRAKE_HEAT_CAPACITY = 30000; // J/K

/** Time constant of the brake cooling to the outside air. Confidence: estimate. */
export const BRAKE_COOL_TIME = 120; // s

/** Where the fade starts and where it reaches its full depth, K above ambient. */
const BRAKE_FADE_ONSET = 200; // K
const BRAKE_FADE_FULL = 500; // K

/** The fraction of the torque a fully hot brake loses. Confidence: estimate. */
const BRAKE_FADE_DEPTH = 0.5;

// ---------------------------------------------------------------------------
// Damage.
// ---------------------------------------------------------------------------

/**
 * Time in which a part reaches full damage at twice its limit dynamic pressure.
 *
 * The hinge load follows the dynamic pressure, so the model drives the damage
 * with the excess of the dynamic pressure over the limit value, not with the
 * excess of the speed. Ten seconds at twice the limit pressure, which is 41
 * percent over the limit speed, destroys the part. Confidence: estimate.
 */
export const DAMAGE_TIME = 10; // s

// ---------------------------------------------------------------------------
// The public interface.
// ---------------------------------------------------------------------------

export type FlapSetting = 'up' | 'takeoff' | 'landing';

export interface SystemsState {
  /** 0 up, 1 at the 50 degree landing setting. */
  flapPosition: number;
  /** 0 up and locked, 1 down and locked. */
  gearPosition: number;
  /** 0 shut, 1 fully out. */
  slatPosition: number;
  /** 0 to 1, the braking the left wheel gives NOW, after the heat fade. */
  brakeLeft: number;
  /** 0 to 1, the braking the right wheel gives NOW, after the heat fade. */
  brakeRight: number;
  /** Fuel on board, kg. */
  fuelMass: number;
  /** 0 sound, 1 destroyed. */
  damage: { flap: number; gear: number };
}

export interface Me262Systems {
  readonly state: SystemsState;
  commandFlaps(s: FlapSetting): void;
  commandGear(down: boolean): void;
  /** Both values are pilot commands from 0 to 1. The fade acts on top of them. */
  setBrakes(left: number, right: number): void;
  update(outerWingAlpha: number, equivalentAirspeed: number, fuelFlow: number, dt: number): void;
  /** Writes the flap deflection and the slat position into the aero control array. */
  writeControls(controls: Float64Array): void;

  // The members below are additions to the bead b19 contract. The ground model,
  // the render model and the gauges need them, and none of them can be derived
  // from the state above on its own.

  /** Equivalent flat plate area of the gear, m2. It peaks in transit. */
  gearDragArea(): number;
  /** 0 up, 1 down. The mains lead the nose wheel. */
  mainGearPosition(): number;
  /** 0 up, 1 down. The nose wheel follows the mains. */
  noseGearPosition(): number;
  /** Braking torque of one wheel, N m, after the fade. */
  brakeTorqueLeft(): number;
  brakeTorqueRight(): number;
  /** Brake temperature above the outside air, K. */
  brakeHeatLeft(): number;
  brakeHeatRight(): number;
  /** The flap deflection the aerodynamics sees, rad. */
  flapDeflection(): number;
}

/** The flap position of one setting, as a fraction of the full travel. */
export function flapSettingPosition(setting: FlapSetting): number {
  if (setting === 'landing') {
    return 1;
  }
  if (setting === 'takeoff') {
    return FLAP_TAKEOFF_FRACTION;
  }
  return 0;
}

/**
 * Returns the equivalent airspeed the flap takes at one position.
 *
 * The limit falls from the take off value to the landing value over the part of
 * the travel that lies past the take off setting. A flap that is up takes any
 * speed, so the function reports the infinity of a part with no limit.
 */
export function flapLimitSpeed(position: number): number {
  if (position <= 0) {
    return Number.POSITIVE_INFINITY;
  }
  const past = clamp(
    (position - FLAP_TAKEOFF_FRACTION) / (1 - FLAP_TAKEOFF_FRACTION),
    0,
    1,
  );
  return FLAP_LIMIT_SPEED_TAKEOFF + (FLAP_LIMIT_SPEED_LANDING - FLAP_LIMIT_SPEED_TAKEOFF) * past;
}

/**
 * Returns the equivalent flat plate drag area of the gear at one position.
 *
 * The first term is the gear itself, which grows with the travel. The second
 * term is the doors and the broadside leg, which appear only while the gear
 * moves. The parabola 4 p (1 - p) is 1 at the middle of the travel and 0 at both
 * ends, so a locked gear carries the first term alone.
 */
export function gearDragAreaAt(position: number): number {
  const p = clamp(position, 0, 1);
  return GEAR_DRAG_AREA * p + GEAR_TRANSIT_DRAG_AREA * 4 * p * (1 - p);
}

/** Returns the fraction of the brake torque that a brake at this heat still gives. */
export function brakeFade(heat: number): number {
  return 1 - BRAKE_FADE_DEPTH * smoothstep(BRAKE_FADE_ONSET, BRAKE_FADE_FULL, heat);
}

/**
 * Builds the systems of one aircraft. The tanks start full, the gear starts
 * down, the flap starts up and the slat starts shut.
 */
export function createMe262Systems(): Me262Systems {
  const state: SystemsState = {
    flapPosition: 0,
    gearPosition: 1,
    slatPosition: 0,
    brakeLeft: 0,
    brakeRight: 0,
    fuelMass: FUEL_CAPACITY,
    damage: { flap: 0, gear: 0 },
  };

  let flapTarget = 0;
  let gearTarget = 1;
  let slatTarget = 0;
  let brakeCommandLeft = 0;
  let brakeCommandRight = 0;
  let heatLeft = 0;
  let heatRight = 0;

  /** Moves a position toward its target at a rate and returns the new value. */
  const drive = (position: number, target: number, rate: number, dt: number): number => {
    const step = rate * dt;
    if (target > position) {
      return Math.min(target, position + step);
    }
    return Math.max(target, position - step);
  };

  /**
   * Returns the damage a part takes over dt.
   *
   * The load on a hinge follows the dynamic pressure, which follows the square
   * of the speed. The function therefore works with the square of the speed
   * ratio and reports zero below the limit.
   */
  const overspeedDamage = (speed: number, limit: number, dt: number): number => {
    if (!(speed > limit) || !Number.isFinite(limit)) {
      return 0;
    }
    const excess = (speed * speed) / (limit * limit) - 1;
    return (excess * dt) / DAMAGE_TIME;
  };

  const api: Me262Systems = {
    state,

    commandFlaps(setting: FlapSetting): void {
      flapTarget = flapSettingPosition(setting);
    },

    commandGear(down: boolean): void {
      gearTarget = down ? 1 : 0;
    },

    setBrakes(left: number, right: number): void {
      brakeCommandLeft = clamp(left, 0, 1);
      brakeCommandRight = clamp(right, 0, 1);
    },

    update(
      outerWingAlpha: number,
      equivalentAirspeed: number,
      fuelFlow: number,
      dt: number,
    ): void {
      if (!(dt > 0)) {
        return;
      }

      // A damaged actuator runs slow and a destroyed one holds where it is.
      const flapRate = (1 - state.damage.flap) / FLAP_TRAVEL_TIME;
      const gearRate = (1 - state.damage.gear) / GEAR_TRAVEL_TIME;
      state.flapPosition = drive(state.flapPosition, flapTarget, flapRate, dt);
      state.gearPosition = drive(state.gearPosition, gearTarget, gearRate, dt);

      // The slat answers the angle of attack alone. The band between the two
      // thresholds holds the target where it is, so the mechanism cannot chatter.
      const alpha = Math.abs(outerWingAlpha);
      if (alpha > SLAT_OPEN_ALPHA) {
        slatTarget = 1;
      } else if (alpha < SLAT_CLOSE_ALPHA) {
        slatTarget = 0;
      }
      state.slatPosition = drive(state.slatPosition, slatTarget, 1 / SLAT_TRAVEL_TIME, dt);

      // Overspeed. The flap limit follows the flap position, and the gear takes
      // damage only while it is out of the wing.
      state.damage.flap = clamp(
        state.damage.flap +
          overspeedDamage(equivalentAirspeed, flapLimitSpeed(state.flapPosition), dt),
        0,
        1,
      );
      if (state.gearPosition > 0) {
        state.damage.gear = clamp(
          state.damage.gear + overspeedDamage(equivalentAirspeed, GEAR_LIMIT_SPEED, dt),
          0,
          1,
        );
      }

      // The brakes. A wheel that is still inside the wing makes no heat, so the
      // model gates the heat on the main gear position. The heat rises with the
      // work the brake does, which is the torque times the wheel speed.
      const rolling = api.mainGearPosition() >= 1 ? Math.max(equivalentAirspeed, 0) : 0;
      const fadeLeft = brakeFade(heatLeft);
      const fadeRight = brakeFade(heatRight);
      state.brakeLeft = brakeCommandLeft * fadeLeft;
      state.brakeRight = brakeCommandRight * fadeRight;
      const workLeft = (state.brakeLeft * BRAKE_TORQUE_MAX * rolling) / WHEEL_RADIUS;
      const workRight = (state.brakeRight * BRAKE_TORQUE_MAX * rolling) / WHEEL_RADIUS;
      heatLeft += (workLeft / BRAKE_HEAT_CAPACITY - heatLeft / BRAKE_COOL_TIME) * dt;
      heatRight += (workRight / BRAKE_HEAT_CAPACITY - heatRight / BRAKE_COOL_TIME) * dt;
      heatLeft = Math.max(0, heatLeft);
      heatRight = Math.max(0, heatRight);

      // The fuel. src/aircraft/me262/mass.ts owns the four tanks and the order
      // they empty in, which puts the rear auxiliary tank first and the two main
      // tanks together after it. This module holds the TOTAL and hands it to
      // me262Mass, so one file owns the tank table and the balance can never
      // disagree with the fuel state.
      state.fuelMass = clamp(state.fuelMass - Math.max(0, fuelFlow) * dt, 0, FUEL_CAPACITY);
    },

    writeControls(controls: Float64Array): void {
      if (controls.length > CONTROL_INDEX.flap) {
        controls[CONTROL_INDEX.flap] = state.flapPosition * FLAP_LANDING_ANGLE;
      }
      // The slat channel carries a FRACTION and not an angle. No strip reads it,
      // because surface.ts opens the slat of each strip from the local angle of
      // attack of that strip. The slat of this aircraft has no hinge angle to
      // report, so the fraction is the only number the mechanism has.
      if (controls.length > CONTROL_INDEX.slat) {
        controls[CONTROL_INDEX.slat] = state.slatPosition;
      }
    },

    gearDragArea(): number {
      return gearDragAreaAt(state.gearPosition);
    },

    mainGearPosition(): number {
      return clamp(state.gearPosition / MAIN_GEAR_SPAN, 0, 1);
    },

    noseGearPosition(): number {
      return clamp((state.gearPosition - NOSE_GEAR_START) / (1 - NOSE_GEAR_START), 0, 1);
    },

    brakeTorqueLeft(): number {
      return state.brakeLeft * BRAKE_TORQUE_MAX;
    },

    brakeTorqueRight(): number {
      return state.brakeRight * BRAKE_TORQUE_MAX;
    },

    brakeHeatLeft(): number {
      return heatLeft;
    },

    brakeHeatRight(): number {
      return heatRight;
    },

    flapDeflection(): number {
      return state.flapPosition * FLAP_LANDING_ANGLE;
    },
  };

  return api;
}
