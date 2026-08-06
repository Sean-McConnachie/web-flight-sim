/**
 * The landing gear, the flaps, the tires and the ground.
 *
 * BEAD 5nl.
 *
 * Everything in this file is a MECHANISM and not a flow. The engine and the
 * airframe make their sound out of moving air. These make theirs out of metal
 * moving against metal, rubber against concrete, and one hydraulic pump.
 *
 *
 * 1. WHAT IS CONTINUOUS AND WHAT IS AN EVENT
 *
 * continuous   the tire rumble, the tire squeal, the hydraulic pump, the scrape
 *              of an airframe that is on the ground it should not be on.
 * events       the lock of a gear leg, the detent of a flap lever, a wheel that
 *              arrives on the runway, a tire that bursts.
 *
 * An event has to be caught on the physics step that made it, because a frame
 * can hold four steps and can miss one that lasts a single step. src/audio/
 * sound.ts does that catching. This file only plays what it is told.
 *
 *
 * 2. THE TOUCH DOWN
 *
 * The strength of a touch down is the RATE the strut closes at and not the
 * speed the aircraft arrives at. A greaser and an arrival both stop the same
 * mass, and the difference between them is entirely how fast the oleo takes it.
 * src/physics/gear.ts reports `compressionRate` for each leg, which is that
 * rate, so the sound follows the landing a pilot actually made.
 */

import { FLAP_TRAVEL_TIME, GEAR_TRAVEL_TIME } from '@/aircraft/me262/systems';
import type { RollingVoiceInput, RollingVoiceParameters } from '@/audio/voices';
import { createRollingVoiceParameters, rollingVoice } from '@/audio/voices';
import { createNoiseLoop, playBurst, playTone } from '@/audio/noise';
import { clamp } from '@/math/tables';

/** How fast a smoothed parameter follows the frame, s. */
const SMOOTHING = 0.04;

// --- The mix trims -------------------------------------------------------

const RUMBLE_TRIM = 0.4;
/** The tonal part of the rumble, which is the tread pattern itself. */
const TREAD_TRIM = 0.12;
const SQUEAL_TRIM = 0.3;
const PUMP_TRIM = 0.16;
const SCRAPE_TRIM = 0.55;

/** Fastest the gear position can move, 1/s. It is one travel in 15 s. */
const GEAR_RATE_FULL = 1 / GEAR_TRAVEL_TIME;

/** Fastest the flap position can move, 1/s. */
const FLAP_RATE_FULL = 1 / FLAP_TRAVEL_TIME;

/**
 * Where the hydraulic pump sits, Hz.
 *
 * A gear pump is a set of teeth that mesh, so it makes a tone at the tooth
 * meshing rate with a hiss of fluid over it. 210 Hz is the tone of a small
 * aircraft pump. Confidence: estimated.
 */
const PUMP_FREQUENCY = 210;

/** What the mechanical voice reads every frame. */
export interface MechanicalVoiceInput {
  /** The wheels. src/audio/voices.ts turns it into a rumble and a squeal. */
  rolling: RollingVoiceInput;
  /** Rate the gear position moves at, 1/s, as an absolute value. */
  gearRate: number;
  /** Rate the flap position moves at, 1/s, as an absolute value. */
  flapRate: number;
  /**
   * How hard the airframe drags on the ground, 0 to 1.
   *
   * It is the slide speed of every airframe contact point that touches, against
   * the load through it. A belly landing runs it to one.
   */
  scrape: number;
}

export interface MechanicalVoice {
  update(input: MechanicalVoiceInput): void;
  /** A leg reaches its stop. `down` is true at the down and locked end. */
  gearLock(down: boolean): void;
  /** The flap lever reaches a detent. */
  flapDetent(): void;
  /** A wheel arrives on the runway. `strength` is 0 to 1 from the oleo rate. */
  touchdown(strength: number): void;
  /** A tire lets go. */
  tireBurst(): void;
  dispose(): void;
}

export function createMechanicalVoice(
  context: BaseAudioContext,
  destination: AudioNode,
): MechanicalVoice {
  const parameters: RollingVoiceParameters = createRollingVoiceParameters();

  // --- The tires ----------------------------------------------------------
  // The broad part. A low pass that opens with the speed, so a taxi is a rumble
  // and a take off run is a roar.
  const rumbleFilter = context.createBiquadFilter();
  rumbleFilter.type = 'lowpass';
  rumbleFilter.frequency.value = 200;
  rumbleFilter.Q.value = 0.7;
  const rumbleGain = context.createGain();
  rumbleGain.gain.value = 0;
  createNoiseLoop(context, 'pink').connect(rumbleFilter);
  rumbleFilter.connect(rumbleGain);
  rumbleGain.connect(destination);

  // The tonal part. The tread blocks beat the ground at a rate that rises with
  // the speed, and a band pass on the same noise is that beat.
  const treadFilter = context.createBiquadFilter();
  treadFilter.type = 'bandpass';
  treadFilter.frequency.value = 200;
  treadFilter.Q.value = 3;
  const treadGain = context.createGain();
  treadGain.gain.value = 0;
  createNoiseLoop(context, 'white').connect(treadFilter);
  treadFilter.connect(treadGain);
  treadGain.connect(destination);

  // --- The squeal ---------------------------------------------------------
  // A tire that slides sings, because the rubber grips and lets go many times a
  // second. A narrow band high up is that song.
  const squealFilter = context.createBiquadFilter();
  squealFilter.type = 'bandpass';
  squealFilter.frequency.value = 1150;
  squealFilter.Q.value = 9;
  const squealGain = context.createGain();
  squealGain.gain.value = 0;
  createNoiseLoop(context, 'white').connect(squealFilter);
  squealFilter.connect(squealGain);
  squealGain.connect(destination);

  // --- The hydraulic pump -------------------------------------------------
  const pumpOsc = context.createOscillator();
  pumpOsc.type = 'sawtooth';
  pumpOsc.frequency.value = PUMP_FREQUENCY;
  const pumpFilter = context.createBiquadFilter();
  pumpFilter.type = 'bandpass';
  pumpFilter.frequency.value = 900;
  pumpFilter.Q.value = 1.6;
  const pumpGain = context.createGain();
  pumpGain.gain.value = 0;
  pumpOsc.connect(pumpFilter);
  // The fluid over the teeth.
  createNoiseLoop(context, 'pink').connect(pumpFilter);
  pumpFilter.connect(pumpGain);
  pumpGain.connect(destination);
  pumpOsc.start();

  // --- The scrape ---------------------------------------------------------
  const scrapeFilter = context.createBiquadFilter();
  scrapeFilter.type = 'highpass';
  scrapeFilter.frequency.value = 700;
  scrapeFilter.Q.value = 0.7;
  const scrapeGain = context.createGain();
  scrapeGain.gain.value = 0;
  createNoiseLoop(context, 'white').connect(scrapeFilter);
  scrapeFilter.connect(scrapeGain);
  scrapeGain.connect(destination);

  function ramp(param: AudioParam, value: number): void {
    param.setTargetAtTime(value, context.currentTime, SMOOTHING);
  }

  return {
    update(input: MechanicalVoiceInput): void {
      rollingVoice(input.rolling, parameters);

      ramp(rumbleGain.gain, parameters.rumbleGain * RUMBLE_TRIM);
      // The low pass follows the tread rate, so the character of the rumble
      // follows the speed and not just its level.
      ramp(rumbleFilter.frequency, clamp(parameters.rumbleFrequency, 60, 3000));
      ramp(treadFilter.frequency, clamp(parameters.rumbleFrequency, 60, 6000));
      ramp(treadGain.gain, parameters.rumbleGain * TREAD_TRIM);
      ramp(squealGain.gain, parameters.squealGain * SQUEAL_TRIM);

      // One pump drives the gear and the flaps, so the louder of the two wins.
      // Two circuits at one time do not make a second pump.
      const pump = Math.max(
        clamp(input.gearRate / GEAR_RATE_FULL, 0, 1),
        clamp(input.flapRate / FLAP_RATE_FULL, 0, 1),
      );
      ramp(pumpGain.gain, pump * PUMP_TRIM);

      ramp(scrapeGain.gain, clamp(input.scrape, 0, 1) * SCRAPE_TRIM);
    },

    gearLock(down: boolean): void {
      // A leg that goes down falls onto its lock under its own weight and the
      // air load. A leg that comes up is pulled onto its stop by the ram. The
      // first is heavier and lower than the second.
      playBurst(context, destination, {
        frequency: down ? 95 : 150,
        q: 1.1,
        gain: down ? 0.6 : 0.42,
        attack: 0.001,
        decay: down ? 0.2 : 0.13,
        color: 'pink',
      });
      playTone(context, destination, down ? 210 : 320, 0.2, 0.14);
    },

    flapDetent(): void {
      playTone(context, destination, 480, 0.1, 0.07);
    },

    touchdown(strength: number): void {
      const force = clamp(strength, 0, 1);
      // The oleo taking the load. It is the body of the sound.
      playBurst(context, destination, {
        frequency: 70 + 40 * force,
        q: 0.8,
        gain: 0.35 + 0.5 * force,
        attack: 0.002,
        decay: 0.16 + 0.14 * force,
        color: 'pink',
      });
      // The tire itself, which chirps as it goes from stopped to running.
      playBurst(context, destination, {
        frequency: 1250,
        q: 6,
        gain: 0.18 + 0.3 * force,
        attack: 0.004,
        decay: 0.1 + 0.16 * force,
      });
    },

    tireBurst(): void {
      playBurst(context, destination, {
        frequency: 220,
        q: 0.5,
        gain: 0.9,
        attack: 0.0005,
        decay: 0.28,
      });
    },

    dispose(): void {
      pumpOsc.stop();
      rumbleGain.disconnect();
      treadGain.disconnect();
      squealGain.disconnect();
      pumpGain.disconnect();
      scrapeGain.disconnect();
    },
  };
}
