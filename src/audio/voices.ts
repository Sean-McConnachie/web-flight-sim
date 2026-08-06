/**
 * The map from the flight state to the parameters of every sound voice.
 *
 * BEAD 1g7.
 *
 * THIS FILE HOLDS NO BROWSER API. It names no `AudioContext`, no `window` and
 * no `document`, so the unit tests run it in Node with the rest of the model.
 * CONVENTIONS section 4 lets src/audio use the browser, and every other file in
 * this directory does. This one does not, because a sound law that no test can
 * measure is a sound law nobody can check.
 *
 * The rest of src/audio is a set of Web Audio nodes with no arithmetic in them.
 * They read the records below and write the numbers into an `AudioParam`. All
 * of the reasoning lives here.
 *
 *
 * 1. WHY THE PROJECT SYNTHESIZES INSTEAD OF PLAYING A RECORDING
 *
 * The simulator ships no audio file. It draws the runway markings into a
 * canvas, it builds the aircraft out of primitives, and it makes the flight
 * model out of twenty two strips instead of one table of coefficients. The
 * sound follows the same rule. A recording of a jet is one engine at one power
 * setting at one distance, and every other condition is a fade of that one
 * sample. A synthesized engine answers the rotor speed, the thrust and the
 * distance, because those are the values it is built from.
 *
 * The gain the project pays is honesty. The gain it loses is the exact timbre
 * of a Jumo 004, which nothing but a recording can give.
 *
 *
 * 2. THE THREE SOURCES OF A TURBOJET
 *
 * A turbojet is not one sound. Jet acoustics splits it into three sources, and
 * each one wins in a different part of the range:
 *
 *   jet mixing noise   The shear layer behind the nozzle. It follows the eighth
 *                      power of the jet velocity, so it is everything at full
 *                      power and nothing at idle.
 *   core noise         The combustion. It follows the fuel flow, and it is what
 *                      is left when the jet velocity is low.
 *   turbomachinery     The blade passing tone of the compressor. It is a pure
 *                      tone, and it is the first thing a listener hears when
 *                      the rotor starts to turn.
 *
 * This file builds all three. That is the reason the engine sounds different at
 * idle, at half power and at full power, and not just louder.
 *
 * The eighth power law ALONE would be wrong. It gives 63 dB between idle and
 * full power, and a real engine gives near 30. The law is not at fault. Jet
 * mixing noise really does fall that fast, and at idle it has stopped being the
 * source you hear. The other two sources are what hold the idle level up.
 *
 *
 * 3. WHAT IS FIRM AND WHAT IS ESTIMATED
 *
 * Every law below carries its source. Two numbers are estimates, and both are
 * marked: the blade count of the first compressor stage, and the nozzle
 * diameter. No published source in this project gives either one. Section 8 of
 * CONVENTIONS says to estimate a missing number, mark the estimate, and say how
 * it was made. Both estimates change the pitch of a sound and nothing else. No
 * flight test measures them.
 */

import {
  MASS_FLOW_MAX,
  MAX_THRUST_SL_STATIC,
  OMEGA_MAX,
} from '@/aircraft/me262/engine';
import { clamp, smoothstep } from '@/math/tables';

// ---------------------------------------------------------------------------
// 1. The compressor tone
// ---------------------------------------------------------------------------

/**
 * Rotor blades on the first compressor stage.
 *
 * The tone a listener hears from an axial compressor is the BLADE PASSING
 * frequency of the first stage. It is the stage nearest the inlet, so it is the
 * one that radiates forward out of the intake with nothing in the way.
 *
 * No source in this project gives the blade count of the Jumo 004. The estimate
 * comes from the plan form. The compressor is 8 stages inside a casing of about
 * 0.65 m, so the first stage runs at a mean radius near 0.24 m and a mean
 * circumference near 1.5 m. An axial stage of that period carries a blade pitch
 * close to its chord, and the chord at that radius is about 55 mm. That gives
 * 27 blades.
 *
 * The check on the estimate is the tone it produces. At the maximum rotor speed
 * of 8700 rpm it gives 3.9 kHz, and the whine of a small early turbojet sits
 * between 2 and 5 kHz. Confidence: estimated.
 */
export const COMPRESSOR_BLADES = 27;

/**
 * Blade passing frequency of the first compressor stage, Hz.
 *
 * It is the shaft frequency times the blade count. `rotorSpeed` is in rad/s,
 * which CONVENTIONS section 2 fixes as the unit the model holds.
 */
export function bladePassingFrequency(rotorSpeed: number): number {
  return (Math.abs(rotorSpeed) / (2 * Math.PI)) * COMPRESSOR_BLADES;
}

/**
 * Level of the compressor tone, 0 to 1.
 *
 * A compressor blade is a lifting surface in a flow. Its unsteady loading
 * radiates as a DIPOLE, and Curle gives a dipole an acoustic power that follows
 * the sixth power of the velocity. Sound pressure is the square root of the
 * power, so the amplitude follows the third power of the blade tip speed. The
 * tip speed follows the rotor speed, so this is a cube of the speed fraction.
 * Source: Curle, "The influence of solid boundaries upon aerodynamic sound",
 * Proc. R. Soc. A 231 (1955). Confidence: firm on the law.
 *
 * At the 3000 rpm idle it reads 0.041, which is 28 dB below full power. That
 * matches the faint idle whine of a real engine.
 */
export function compressorToneGain(rotorSpeed: number): number {
  const fraction = clamp(Math.abs(rotorSpeed) / OMEGA_MAX, 0, 1);
  return fraction * fraction * fraction;
}

// ---------------------------------------------------------------------------
// 2. The jet
// ---------------------------------------------------------------------------

/**
 * Air the compressor swallows at one rotor speed, kg/s.
 *
 * The corrected mass flow of a fixed geometry compressor rises with the
 * corrected speed, and over the top half of the range that rise is close to
 * linear. The audio only needs the flow to find the jet velocity, and the jet
 * velocity only matters where the jet is loud, which is the top of the range.
 * Confidence: firm on the form, estimated on the linearity at low speed.
 *
 * The floor stops a division by zero at rest. It is small enough that the gate
 * in `engineVoice` has already silenced the jet before it can act.
 */
export function compressorMassFlow(rotorSpeed: number): number {
  const fraction = clamp(Math.abs(rotorSpeed) / OMEGA_MAX, 0, 1);
  return Math.max(MASS_FLOW_MAX * fraction, 0.05);
}

/**
 * Speed of the gas that leaves the nozzle, m/s.
 *
 * A turbojet with a fully expanded nozzle makes a thrust of `mdot (Vj - V0)`,
 * where `V0` is the speed of the aircraft through the air. Turn that around for
 * the jet velocity. Source: the momentum equation. Confidence: firm.
 *
 * At full power, at rest, at sea level, it gives 8800 / 21.2 = 415 m/s.
 */
export function jetVelocity(thrust: number, rotorSpeed: number, trueAirspeed: number): number {
  return Math.max(trueAirspeed + thrust / compressorMassFlow(rotorSpeed), 0);
}

/**
 * Jet velocity at full power, at rest, at sea level, m/s.
 *
 * It is the reference the jet noise gain is measured against, and it is DERIVED
 * from two numbers the engine model already holds. It is not a new number.
 */
export const JET_VELOCITY_REFERENCE = MAX_THRUST_SL_STATIC / MASS_FLOW_MAX;

/**
 * Level of the jet mixing noise, 0 to 1.
 *
 * Lighthill showed that the acoustic power of a free jet follows the EIGHTH
 * power of the jet velocity. Sound pressure is the square root of the power, so
 * the amplitude follows the fourth power.
 * Source: Lighthill, "On sound generated aerodynamically I: General theory",
 * Proc. R. Soc. A 211 (1952), page 564. Confidence: firm.
 *
 * Read section 2 of the module comment before you soften this exponent. The
 * steep fall is correct. It is the reason a jet at idle sounds like a different
 * machine from the same jet at full power, and the core noise and the
 * compressor tone are what hold the idle level up.
 */
export function jetMixingGain(jetSpeed: number): number {
  const ratio = clamp(jetSpeed / JET_VELOCITY_REFERENCE, 0, 1);
  const square = ratio * ratio;
  return square * square;
}

/**
 * Diameter of the jet pipe at the nozzle, m.
 *
 * The engine is 0.76 m across the accessory case. The jet pipe behind the
 * turbine is narrower than that, and the Jumo 004 carried a movable bullet in
 * the nozzle which set the exit area. 0.55 m is the diameter of the pipe at the
 * plane of the exit, read off a general arrangement drawing.
 *
 * It sets the peak frequency of the jet noise and nothing else.
 * Confidence: estimated.
 */
export const NOZZLE_DIAMETER = 0.55; // m

/**
 * Strouhal number at which the noise of a subsonic jet peaks.
 *
 * The peak of the mixing noise spectrum sits near `f D / Vj = 0.2` for a cold
 * subsonic jet. Source: standard jet acoustics, and it is the value SAE ARP876
 * builds its spectrum around. Confidence: firm.
 */
export const JET_STROUHAL = 0.2;

/** Frequency at which the jet noise peaks, Hz. */
export function jetPeakFrequency(jetSpeed: number): number {
  return (JET_STROUHAL * jetSpeed) / NOZZLE_DIAMETER;
}

/**
 * How far above the spectral peak the roar filter opens.
 *
 * The peak at full power sits at 151 Hz. A low pass placed there would leave
 * only the rumble, and that is the WRONG answer for this listener. The low
 * frequency dominance of jet noise is a FAR FIELD result. Our listener sits in
 * the cockpit or 26 m behind the tail, which is the near field of a 0.55 m
 * nozzle, and the near field carries the high frequencies that have not yet
 * been absorbed by the air.
 *
 * The propagation stage takes those high frequencies off again as the listener
 * moves away. That is where the far field answer comes from, and it comes from
 * the distance rather than from a constant here.
 */
export const ROAR_BANDWIDTH_FACTOR = 12;

// ---------------------------------------------------------------------------
// 3. The core, the starter and the fire
// ---------------------------------------------------------------------------

/**
 * Fuel flow that counts as full core noise, kg/s.
 *
 * It is the flow at maximum power, which the engine model already holds.
 */
export const CORE_FUEL_FLOW_REFERENCE = 0.355; // kg/s

/**
 * Level of the combustion noise, 0 to 1.
 *
 * Core noise comes from the unsteady heat release in the chambers and from the
 * turbulence it leaves behind. Its power follows the heat the burners add,
 * which follows the fuel flow. The exponent is the estimate here. A square root
 * of the flow holds the idle level near a third of the full level, which is the
 * band a real engine sits in.
 * Confidence: firm on the source, estimated on the exponent.
 */
export function coreNoiseGain(fuelFlow: number): number {
  return Math.sqrt(clamp(fuelFlow / CORE_FUEL_FLOW_REFERENCE, 0, 1));
}

/**
 * Firings of the Riedel starter for one turn of the main rotor.
 *
 * The Riedel RBA is a two stroke flat twin in the nose of the nacelle. A two
 * stroke fires on every revolution of its own crank, and it has two cylinders,
 * so it fires twice per crank revolution. It drives the main rotor through a
 * reduction, and the ratio of the two is the estimate. A ratio of 5 puts the
 * Riedel near 4000 rpm while it cranks the main rotor to the 800 rpm that the
 * handbook calls for, which is a working speed for a small two stroke.
 *
 * The product is 10 firings per turn of the main rotor, so the putt of the
 * starter runs at 133 Hz at the light off speed. Confidence: estimated.
 */
export const STARTER_FIRINGS_PER_ROTOR_TURN = 10;

/** Firing frequency of the Riedel starter, Hz. */
export function starterFrequency(rotorSpeed: number): number {
  return (Math.abs(rotorSpeed) / (2 * Math.PI)) * STARTER_FIRINGS_PER_ROTOR_TURN;
}

// ---------------------------------------------------------------------------
// 4. One engine
// ---------------------------------------------------------------------------

/** What one engine voice reads. Every field is a member of `Engine`. */
export interface EngineVoiceInput {
  /** rad/s. */
  rotorSpeed: number;
  /** N, of this engine alone. */
  thrust: number;
  /** kg/s. */
  fuelFlow: number;
  /** m/s, true. The jet velocity is measured against the air, not the ground. */
  trueAirspeed: number;
  /** True while a flame burns in the chambers. */
  lit: boolean;
  /** True while the Riedel turns the rotor. */
  starterRunning: boolean;
  /** True while the nacelle burns. */
  onFire: boolean;
}

/** What one engine voice writes into its nodes. */
export interface EngineVoiceParameters {
  /** Hz, the blade passing tone. */
  whineFrequency: number;
  /** 0 to 1. */
  whineGain: number;
  /** 0 to 1, the jet mixing noise. */
  roarGain: number;
  /** Hz, the low pass on the roar. */
  roarCutoff: number;
  /** 0 to 1, the combustion. */
  coreGain: number;
  /** Hz, the low pass on the core noise. */
  coreCutoff: number;
  /** Hz, the firing rate of the Riedel. */
  starterFrequency: number;
  /** 0 to 1. */
  starterGain: number;
  /** 0 to 1, a jet pipe fire. */
  fireGain: number;
}

/** An empty parameter record, so no frame allocates one. */
export function createEngineVoiceParameters(): EngineVoiceParameters {
  return {
    whineFrequency: 20,
    whineGain: 0,
    roarGain: 0,
    roarCutoff: 200,
    coreGain: 0,
    coreCutoff: 200,
    starterFrequency: 20,
    starterGain: 0,
    fireGain: 0,
  };
}

/**
 * Rotor speed under which the tone is silent, rad/s.
 *
 * It is about 100 rpm. Under it the tone is below 45 Hz and the gain law has
 * already taken it to nothing, so the gate only stops an oscillator from being
 * asked for a frequency near zero.
 */
const WHINE_GATE = 10; // rad/s

/** Lowest frequency any oscillator of this file is ever given, Hz. */
export const MINIMUM_FREQUENCY = 20;

/**
 * Fills the parameters of one engine voice. It allocates nothing.
 */
export function engineVoice(
  input: EngineVoiceInput,
  out: EngineVoiceParameters,
): EngineVoiceParameters {
  const running = Math.abs(input.rotorSpeed) > WHINE_GATE;

  // The compressor tone.
  out.whineFrequency = Math.max(bladePassingFrequency(input.rotorSpeed), MINIMUM_FREQUENCY);
  out.whineGain = running ? compressorToneGain(input.rotorSpeed) : 0;

  // The jet. A rotor that windmills with no flame still pushes air through the
  // engine, but it makes no jet, so the thrust it reports is what decides.
  const speed = jetVelocity(input.thrust, input.rotorSpeed, input.trueAirspeed);
  out.roarGain = input.lit ? jetMixingGain(speed) : 0;
  out.roarCutoff = clamp(ROAR_BANDWIDTH_FACTOR * jetPeakFrequency(speed), 200, 12000);

  // The combustion. It is the source that holds the idle level up.
  out.coreGain = input.lit ? coreNoiseGain(input.fuelFlow) : 0;
  // Core noise is a low frequency source. It sits an octave over the jet peak
  // and it does not open up with power the way the mixing noise does.
  out.coreCutoff = clamp(2 * jetPeakFrequency(speed) + 120, 120, 1400);

  // The Riedel. It stops when the main rotor passes the cutout, and the engine
  // model has already dropped `starterRunning` by then.
  out.starterFrequency = Math.max(starterFrequency(input.rotorSpeed), MINIMUM_FREQUENCY);
  out.starterGain = input.starterRunning && running ? 1 : 0;

  out.fireGain = input.onFire ? 1 : 0;
  return out;
}

// ---------------------------------------------------------------------------
// 5. The airframe
// ---------------------------------------------------------------------------

/**
 * Airspeed that counts as full wind noise, m/s.
 *
 * 250 m/s is 900 km/h, which is just under the 950 km/h placard of section 8 of
 * CONVENTIONS. The loudest wind the aircraft can legally make therefore reads
 * near one.
 */
export const WIND_SPEED_REFERENCE = 250; // m/s

/**
 * Level of the airframe noise, 0 to 1.
 *
 * The skin of an aircraft in a turbulent boundary layer is a rigid surface in a
 * flow, and Curle gives that a dipole power which follows the sixth power of
 * the speed. The amplitude is the square root, so it follows the third power.
 * Source: Curle, Proc. R. Soc. A 231 (1955). Confidence: firm.
 *
 * This is also the law behind the rule of thumb that cabin noise rises by 18 dB
 * for a doubling of the speed, because 20 log10(8) is 18.1.
 */
export function windGain(trueAirspeed: number): number {
  const ratio = clamp(trueAirspeed / WIND_SPEED_REFERENCE, 0, 1);
  return ratio * ratio * ratio;
}

/**
 * Thickness of the boundary layer that sets the wind noise frequency, m.
 *
 * The noise of a turbulent boundary layer peaks near the speed over the
 * thickness. The layer over the canopy of a 10.6 m fuselage runs to a few
 * centimeters. 0.04 m puts the peak at 6.2 kHz at the placard speed and at
 * 1.2 kHz at the approach, which is the way the rush of air over a canopy
 * changes with speed. Confidence: estimated.
 */
export const BOUNDARY_LAYER_THICKNESS = 0.04; // m

/** Where the wind noise low pass sits, Hz. */
export function windCutoff(trueAirspeed: number): number {
  return clamp(Math.abs(trueAirspeed) / BOUNDARY_LAYER_THICKNESS, 200, 12000);
}

/**
 * Angle of attack at which the buffet starts, rad.
 *
 * Section 8 of CONVENTIONS gives the clean maximum lift at 20.3 degrees. A wing
 * starts to shed separated flow over the tail several degrees before it reaches
 * that angle, and the shaking IS the stall warning of an aircraft with no stick
 * shaker. The Me-262 has none. Confidence: estimated on the margin, firm on the
 * angle it is measured back from.
 */
export const BUFFET_ALPHA_ONSET = 0.244; // rad, 14.0 deg

/** Angle of attack at which the buffet is fully developed, rad. */
export const BUFFET_ALPHA_FULL = 0.354; // rad, 20.3 deg

/**
 * Rate at which the airframe shakes in the buffet, Hz.
 *
 * The wake off a separated wing arrives at the tail as a train of eddies, and
 * the airframe answers on its first bending mode. A fighter of this size sits
 * near 15 Hz. Confidence: estimated.
 */
export const BUFFET_FREQUENCY = 15; // Hz

/**
 * Mach number at which the shock buffet starts.
 *
 * Section 8 of CONVENTIONS gives a tuck onset of 0.83 and a limit of 0.86. The
 * shock that makes the tuck also separates the flow behind it, and the pilot
 * feels that before the trim change. Confidence: derived from the tuck onset.
 */
export const MACH_BUFFET_ONSET = 0.8;

/** Mach number at which the shock buffet is fully developed. */
export const MACH_BUFFET_FULL = 0.86;

/** What the airframe voice reads. */
export interface AirframeVoiceInput {
  /** m/s, true. */
  trueAirspeed: number;
  /** rad, free stream. */
  alpha: number;
  /** Free stream Mach number. */
  mach: number;
  /** 0 up and locked, 1 down and locked. */
  gearPosition: number;
  /** 0 up, 1 at the landing setting. */
  flapPosition: number;
}

/** What the airframe voice writes. */
export interface AirframeVoiceParameters {
  /** 0 to 1. */
  windGain: number;
  /** Hz. */
  windCutoff: number;
  /** 0 to 1, the depth of the shake on the wind noise. */
  buffetDepth: number;
  /** Hz. */
  buffetFrequency: number;
}

/** An empty parameter record. */
export function createAirframeVoiceParameters(): AirframeVoiceParameters {
  return { windGain: 0, windCutoff: 200, buffetDepth: 0, buffetFrequency: BUFFET_FREQUENCY };
}

/**
 * How much noise the gear and the flaps add over the clean airframe.
 *
 * A landing gear leg is a bluff body in the free stream and it is the loudest
 * thing on an airliner on the approach. The flap edge is the second. Both are
 * scaled against the clean skin noise, so they follow the same speed law.
 * Confidence: estimated on the two fractions, firm on the fact that the gear
 * dominates the flap.
 */
export const GEAR_NOISE_FRACTION = 0.9;
export const FLAP_NOISE_FRACTION = 0.35;

/**
 * How far the bluff body noise of the gear pulls the low pass down.
 *
 * The eddies off a wheel and a strut are the size of the wheel, so they are far
 * bigger than the eddies in the boundary layer and they arrive lower in
 * frequency. Confidence: estimated.
 */
export const BLUFF_BODY_CUTOFF_FACTOR = 0.55;

/** Fills the parameters of the airframe voice. It allocates nothing. */
export function airframeVoice(
  input: AirframeVoiceInput,
  out: AirframeVoiceParameters,
): AirframeVoiceParameters {
  const clean = windGain(input.trueAirspeed);
  const bluff = GEAR_NOISE_FRACTION * input.gearPosition + FLAP_NOISE_FRACTION * input.flapPosition;
  out.windGain = clamp(clean * (1 + bluff), 0, 1);
  // A gear that hangs in the stream drops the pitch of the rush of air, so the
  // low pass moves down as the legs come out.
  out.windCutoff =
    windCutoff(input.trueAirspeed) * (1 - BLUFF_BODY_CUTOFF_FACTOR * clamp(bluff, 0, 1));

  // Two separate things shake the airframe, and the deeper of the two wins.
  const stall = smoothstep(BUFFET_ALPHA_ONSET, BUFFET_ALPHA_FULL, Math.abs(input.alpha));
  const shock = smoothstep(MACH_BUFFET_ONSET, MACH_BUFFET_FULL, input.mach);
  out.buffetDepth = Math.max(stall, shock);
  // A shock buffet is faster than a stall buffet, because the eddies behind a
  // shock are smaller than the eddies off a stalled wing.
  out.buffetFrequency = BUFFET_FREQUENCY * (1 + shock);
  return out;
}

// ---------------------------------------------------------------------------
// 6. The tires on the ground
// ---------------------------------------------------------------------------

/**
 * Tread blocks that pass the contact patch for one turn of a main wheel.
 *
 * The rumble of a tire on a hard surface is the tread pattern beating against
 * the ground. The main tire of the Me-262 is 840 by 300 mm, so it turns once in
 * 2.64 m. A tread of that period carries a block every 40 mm or so, which gives
 * 66 blocks. Confidence: estimated.
 */
export const TREAD_BLOCKS = 66;

/** Radius of a main wheel, m. It matches WHEEL_RADII of src/main.ts. */
export const MAIN_WHEEL_RADIUS = 0.42; // m

/** Rate at which the tread beats the ground, Hz. */
export function treadFrequency(groundSpeed: number): number {
  return (Math.abs(groundSpeed) / (2 * Math.PI * MAIN_WHEEL_RADIUS)) * TREAD_BLOCKS;
}

/**
 * Load through the legs that counts as a full rumble, N.
 *
 * It is the weight of the loaded aircraft, 6396 kg times standard gravity. A
 * wheel that carries the whole aircraft makes the loudest rumble it can make.
 */
export const ROLLING_LOAD_REFERENCE = 6396 * 9.80665; // N

/**
 * Speed over the ground that counts as a full rumble, m/s.
 *
 * 60 m/s is 216 km/h, which is over the touch down speed of 175 km/h and under
 * the speed the aircraft leaves the ground at. The tire rumble is therefore at
 * its loudest through the whole of the take off run.
 */
export const ROLLING_SPEED_REFERENCE = 60; // m/s

/** What the rolling voice reads. */
export interface RollingVoiceInput {
  /** m/s, the speed of the tread at the contact patch. */
  wheelSpeed: number;
  /** N, the sum of the load through every leg that touches. */
  wheelLoad: number;
  /** True while any wheel touches. */
  onGround: boolean;
  /**
   * Largest slip ratio of any leg, as an absolute value. A locked wheel reads
   * 1 and it makes the tire squeal.
   */
  slip: number;
}

/** What the rolling voice writes. */
export interface RollingVoiceParameters {
  /** 0 to 1. */
  rumbleGain: number;
  /** Hz, the beat of the tread. */
  rumbleFrequency: number;
  /** 0 to 1, a tire that slides instead of rolling. */
  squealGain: number;
}

/** An empty parameter record. */
export function createRollingVoiceParameters(): RollingVoiceParameters {
  return { rumbleGain: 0, rumbleFrequency: MINIMUM_FREQUENCY, squealGain: 0 };
}

/**
 * Slip ratio under which a tire rolls and makes no squeal.
 *
 * src/physics/gear.ts reports a slip ratio where -1 is a locked wheel. A tire
 * builds its friction over the first few percent of slip with no noise, and it
 * only sings when it slides. Confidence: estimated.
 */
export const SQUEAL_SLIP_ONSET = 0.15;

/** Fills the parameters of the rolling voice. It allocates nothing. */
export function rollingVoice(
  input: RollingVoiceInput,
  out: RollingVoiceParameters,
): RollingVoiceParameters {
  if (!input.onGround) {
    out.rumbleGain = 0;
    out.squealGain = 0;
    return out;
  }
  const load = clamp(input.wheelLoad / ROLLING_LOAD_REFERENCE, 0, 1);
  const speed = clamp(Math.abs(input.wheelSpeed) / ROLLING_SPEED_REFERENCE, 0, 1);
  // A wheel that carries no load makes no sound whatever its speed, and a wheel
  // that stands still makes none whatever its load. The product gives both.
  out.rumbleGain = load * speed;
  out.rumbleFrequency = Math.max(treadFrequency(input.wheelSpeed), MINIMUM_FREQUENCY);
  // A locked wheel that slides at speed is what squeals. A locked wheel that
  // stands still is silent, so the speed is in this term as well.
  out.squealGain = smoothstep(SQUEAL_SLIP_ONSET, 1, clamp(input.slip, 0, 1)) * speed * load;
  return out;
}

// ---------------------------------------------------------------------------
// 7. Propagation
// ---------------------------------------------------------------------------

/**
 * Distance at which a source is at its reference level, m.
 *
 * It is the 26 m of the chase view, which is the view the simulator starts in.
 * The mix is therefore set in the view a pilot sees first, and the inverse
 * distance law works out from there.
 *
 * It is also the floor of the law. Inside it the gain holds at one, which stops
 * a source from going to infinity as the camera arrives on top of it. The
 * cockpit view sits well inside the floor and reads one.
 */
export const REFERENCE_DISTANCE = 26; // m

/**
 * Level of one source at a distance, 0 to 1.
 *
 * Sound spreads over the surface of a sphere, so the intensity falls with the
 * square of the distance and the pressure falls with the distance itself.
 * Source: spherical spreading. Confidence: firm.
 */
export function distanceGain(distance: number): number {
  return REFERENCE_DISTANCE / Math.max(distance, REFERENCE_DISTANCE);
}

/**
 * How fast the air takes the high frequencies off, Hz m.
 *
 * Air absorbs sound, and it absorbs the high frequencies far faster than the
 * low ones. That is the reason a jet overhead cracks and the same jet at a
 * kilometer rumbles. A one pole low pass whose corner falls with the distance
 * is the cheapest form that has the same behavior.
 *
 * The constant is the product of the corner frequency and the distance. At
 * 400000 Hz m the corner sits at 20 kHz at 20 m, which does nothing, at 2 kHz
 * at 200 m, and at 400 Hz at a kilometer. The last of those is the rumble.
 *
 * A real absorption curve depends on the frequency, the humidity and the
 * temperature, and ISO 9613-1 gives it. This is a fit to the shape of that
 * curve and not a calculation of it. Confidence: estimated.
 */
export const ABSORPTION_CONSTANT = 400_000; // Hz m

/** Where the air absorption low pass sits, Hz. */
export function absorptionCutoff(distance: number): number {
  return clamp(ABSORPTION_CONSTANT / Math.max(distance, 1), 300, 20_000);
}

/**
 * Longest propagation delay the delay line can hold, s.
 *
 * A Web Audio delay line fixes its maximum when it is built. Three seconds is
 * about a kilometer, and at a kilometer the distance gain has already taken the
 * source to 2.6 percent and the absorption has taken everything over 400 Hz.
 * A source further away than that is not audible, so the cap costs nothing.
 */
export const MAXIMUM_DELAY = 3; // s

/** What the propagation stage reads. */
export interface PropagationInput {
  /** m, from the listener to the source. */
  distance: number;
  /** m/s, in the air the sound travels through. */
  speedOfSound: number;
}

/** What the propagation stage writes. */
export interface PropagationParameters {
  /** 0 to 1. */
  gain: number;
  /** Hz. */
  cutoff: number;
  /** s. */
  delay: number;
}

/** An empty parameter record. */
export function createPropagationParameters(): PropagationParameters {
  return { gain: 1, cutoff: 20_000, delay: 0 };
}

/**
 * Fills the propagation parameters. It allocates nothing.
 *
 * THE DELAY IS NOT A DETAIL. It is where the Doppler shift comes from.
 *
 * A variable delay line IS a Doppler shift, exactly and with no separate
 * calculation. Hold a source at a fixed delay and it plays at its own pitch.
 * Make the delay grow at a rate `k` and every wave front arrives `1 + k` times
 * further apart, so the pitch falls by that factor. The delay is the distance
 * over the speed of sound, so `k` is the radial speed over the speed of sound,
 * and `1 / (1 + v / c)` is the Doppler formula for a moving source.
 *
 * `dopplerFactor` below states the same identity, and the unit test measures
 * the two against each other.
 */
export function propagation(
  input: PropagationInput,
  out: PropagationParameters,
): PropagationParameters {
  const distance = Math.max(input.distance, 0);
  out.gain = distanceGain(distance);
  out.cutoff = absorptionCutoff(distance);
  out.delay = clamp(distance / Math.max(input.speedOfSound, 1), 0, MAXIMUM_DELAY);
  return out;
}

/**
 * Pitch factor of a source that moves away at `radialSpeed`.
 *
 * A positive `radialSpeed` moves the source AWAY from the listener and lowers
 * the pitch. Source: the classical Doppler shift of a moving source and a
 * listener at rest. Confidence: firm.
 */
export function dopplerFactor(radialSpeed: number, speedOfSound: number): number {
  const c = Math.max(speedOfSound, 1);
  // A source that closes faster than sound would divide by zero and then turn
  // the sign over. The aircraft cannot reach that speed, and the clamp says so.
  return c / Math.max(c + radialSpeed, 0.05 * c);
}

/**
 * Pitch factor that a delay line which ramps at `delayRate` produces.
 *
 * `delayRate` is in seconds of delay per second of time. It is the identity in
 * the comment of `propagation`, written as code so a test can hold the two
 * against each other.
 */
export function delayLineDopplerFactor(delayRate: number): number {
  return 1 / Math.max(1 + delayRate, 0.05);
}

// ---------------------------------------------------------------------------
// 8. The cockpit
// ---------------------------------------------------------------------------

/**
 * Where the canopy low pass sits, Hz.
 *
 * The pilot sits under a hood of plexiglass with the engines out on the wings.
 * The hood takes the top off everything that arrives from outside, and it takes
 * more off the higher frequencies. 1800 Hz leaves the compressor tone in place
 * as a muffled hum and takes the edge off the jet. Confidence: estimated.
 */
export const CANOPY_CUTOFF = 1800; // Hz

/**
 * How much louder the wind is inside the cockpit than it is outside.
 *
 * Nothing is louder in an unpressurized cockpit than the air over the hood
 * seals. The listener is on the other side of a 3 mm skin from it, and the
 * outside views are 26 m away in clean air. Confidence: estimated.
 */
export const COCKPIT_WIND_GAIN = 2.2;
