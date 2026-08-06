/**
 * One Jumo 004, as sound.
 *
 * BEAD c2c.
 *
 *
 * 1. THE GRAPH
 *
 *   sawtooth at the blade tone -> low pass -> whine gain  ----\
 *   pink noise -> low pass at the jet band  -> roar gain  -----+-> pan -> out
 *   pink noise -> low pass at the core band -> core gain  ----/
 *   sawtooth at the Riedel rate -> resonant low pass -> starter gain
 *   pink noise -> band pass -> fire gain
 *
 * Section 2 of src/audio/voices.ts says why there are three continuous sources
 * and not one. Every number that reaches an `AudioParam` below comes out of
 * that file. This one holds no arithmetic beyond the mix trims.
 *
 *
 * 2. WHY NOTHING EVER STOPS
 *
 * A Web Audio source that stops is dead. It cannot start again. Every source
 * here therefore runs from the moment the voice is built until the page closes,
 * and the gain in front of it is what turns it on and off. A cold engine costs
 * five oscillators and buffers that all read into a gain of zero, which the
 * audio thread finishes in nanoseconds.
 *
 *
 * 3. WHY THE PARAMETERS ARE SMOOTHED
 *
 * The frame rate is not the audio rate. A gain that is written 60 times a
 * second and holds between the writes is a staircase, and a staircase in a gain
 * is a buzz at the frame rate. `setTargetAtTime` runs an exponential approach
 * on the audio thread instead, so the value moves smoothly between the frames.
 *
 * The time constant must be short enough to follow a real change. A surge takes
 * the rotor speed down in a fraction of a second, and 30 ms follows that.
 *
 *
 * 4. THE THROB OF TWO ENGINES
 *
 * Two engines at the same throttle never run at exactly the same speed, and the
 * two blade tones beat against each other. That slow throb is one of the things
 * that says TWIN to a listener. The flight model gives each engine its own
 * rotor speed, so most of the time the beat is there on its own. DETUNE holds a
 * floor under it for the case where the two speeds agree exactly.
 */

import type { EngineVoiceInput, EngineVoiceParameters } from '@/audio/voices';
import { createEngineVoiceParameters, engineVoice } from '@/audio/voices';
import { createNoiseLoop, playBurst, playTone } from '@/audio/noise';

/** How fast a smoothed parameter follows the frame, s. Section 3 says why. */
const SMOOTHING = 0.03;

/** A frequency follows faster than a gain. A slow tone sweep sounds like tape. */
const FREQUENCY_SMOOTHING = 0.02;

/**
 * How far the two engines are pulled apart, as a fraction of the frequency.
 *
 * At the 3.9 kHz blade tone this is 3.9 Hz between the two, which is a throb
 * near four times a second. Section 4 says why it is here. Confidence:
 * estimated, and chosen for the rate of the beat it makes.
 */
const DETUNE = 0.0005;

// --- The mix trims -------------------------------------------------------
// Every one of these is a level and not a law. src/audio/voices.ts gives each
// source a value from 0 to 1, and these say how loud a full one of each is
// against the others. Two engines run, so every number here is paid twice.

// `tools/audio-check.mjs` measured the first set of these against each other
// and against the wind. The engine at idle came out 23 dB UNDER the Riedel
// starter, which is the wrong way round by a wide margin. These are the
// corrected values, and the tool holds the result.
const WHINE_TRIM = 0.21;
const ROAR_TRIM = 0.9;
const CORE_TRIM = 0.45;

/**
 * The Riedel.
 *
 * It is low because the starter is a 10 hp two stroke and the thing it starts
 * is a turbojet. It sat at 0.22, which put it 7 dB OVER the slipstream at
 * 900 km/h and 23 dB over the engine it had just lit.
 */
const STARTER_TRIM = 0.04;
const FIRE_TRIM = 0.5;

/**
 * How far above the blade tone the whine filter opens.
 *
 * A saw tooth at the blade tone carries every harmonic of it. A real compressor
 * tone carries the fundamental and a little of the second and third, so the low
 * pass sits at two and a half times the tone and takes the rest off.
 */
const WHINE_BANDWIDTH = 2.5;

export interface EngineVoice {
  /** Writes one frame of flight state into the nodes. It allocates nothing. */
  update(input: EngineVoiceInput): void;
  /** A compressor stall. src/aircraft/me262/engine.ts fires one every 0.35 s. */
  bang(): void;
  /** The flame arrives in the chambers. */
  lightOff(): void;
  /** The flame goes out. */
  flameout(): void;
  dispose(): void;
}

export interface EngineVoiceOptions {
  /** 0 is the left engine and 1 is the right. It sets the detune and the pan. */
  index: number;
  /** -1 hard left, +1 hard right. The engines sit out on the wings. */
  pan: number;
}

export function createEngineVoice(
  context: BaseAudioContext,
  destination: AudioNode,
  options: EngineVoiceOptions,
): EngineVoice {
  const parameters: EngineVoiceParameters = createEngineVoiceParameters();
  // The two engines pull in opposite directions, so the beat is the full DETUNE
  // between them and not half of it.
  const detune = options.index % 2 === 0 ? 1 - DETUNE : 1 + DETUNE;

  // --- The output stage ---------------------------------------------------
  const out = context.createGain();
  out.gain.value = 1;
  const panner = context.createStereoPanner();
  panner.pan.value = options.pan;
  out.connect(panner);
  panner.connect(destination);

  // --- The compressor tone ------------------------------------------------
  const whineOsc = context.createOscillator();
  whineOsc.type = 'sawtooth';
  whineOsc.frequency.value = 20;
  const whineFilter = context.createBiquadFilter();
  whineFilter.type = 'lowpass';
  whineFilter.frequency.value = 200;
  // A little resonance at the corner puts an edge on the tone, which is what
  // makes a turbine read as a turbine and not as a hum.
  whineFilter.Q.value = 2;
  const whineGain = context.createGain();
  whineGain.gain.value = 0;
  whineOsc.connect(whineFilter);
  whineFilter.connect(whineGain);
  whineGain.connect(out);
  whineOsc.start();

  // --- The jet ------------------------------------------------------------
  const roarFilter = context.createBiquadFilter();
  roarFilter.type = 'lowpass';
  roarFilter.frequency.value = 200;
  roarFilter.Q.value = 0.6;
  const roarGain = context.createGain();
  roarGain.gain.value = 0;
  const roarNoise = createNoiseLoop(context, 'pink');
  roarNoise.connect(roarFilter);
  roarFilter.connect(roarGain);
  roarGain.connect(out);

  // --- The combustion -----------------------------------------------------
  const coreFilter = context.createBiquadFilter();
  coreFilter.type = 'lowpass';
  coreFilter.frequency.value = 120;
  coreFilter.Q.value = 1.2;
  const coreGain = context.createGain();
  coreGain.gain.value = 0;
  const coreNoise = createNoiseLoop(context, 'pink');
  coreNoise.connect(coreFilter);
  coreFilter.connect(coreGain);
  coreGain.connect(out);

  // --- The Riedel starter -------------------------------------------------
  // A two stroke twin is a train of pulses. A saw tooth through a low pass with
  // real resonance is that train, and the resonance is the exhaust pipe.
  const starterOsc = context.createOscillator();
  starterOsc.type = 'sawtooth';
  starterOsc.frequency.value = 20;
  const starterFilter = context.createBiquadFilter();
  starterFilter.type = 'lowpass';
  starterFilter.frequency.value = 400;
  // A resonance of 8 rang far more than a two stroke exhaust does, and it was
  // most of why the starter was too loud. 4 keeps the character and not the
  // gain.
  starterFilter.Q.value = 4;
  const starterGain = context.createGain();
  starterGain.gain.value = 0;
  starterOsc.connect(starterFilter);
  starterFilter.connect(starterGain);
  starterGain.connect(out);
  starterOsc.start();

  // --- The jet pipe fire --------------------------------------------------
  const fireFilter = context.createBiquadFilter();
  fireFilter.type = 'bandpass';
  fireFilter.frequency.value = 260;
  fireFilter.Q.value = 0.7;
  const fireGain = context.createGain();
  fireGain.gain.value = 0;
  const fireNoise = createNoiseLoop(context, 'pink');
  fireNoise.connect(fireFilter);
  fireFilter.connect(fireGain);
  fireGain.connect(out);

  function ramp(param: AudioParam, value: number, timeConstant: number): void {
    param.setTargetAtTime(value, context.currentTime, timeConstant);
  }

  return {
    update(input: EngineVoiceInput): void {
      engineVoice(input, parameters);

      ramp(whineOsc.frequency, parameters.whineFrequency * detune, FREQUENCY_SMOOTHING);
      ramp(
        whineFilter.frequency,
        Math.min(parameters.whineFrequency * WHINE_BANDWIDTH, 20_000),
        FREQUENCY_SMOOTHING,
      );
      ramp(whineGain.gain, parameters.whineGain * WHINE_TRIM, SMOOTHING);

      ramp(roarFilter.frequency, parameters.roarCutoff, FREQUENCY_SMOOTHING);
      ramp(roarGain.gain, parameters.roarGain * ROAR_TRIM, SMOOTHING);

      ramp(coreFilter.frequency, parameters.coreCutoff, FREQUENCY_SMOOTHING);
      ramp(coreGain.gain, parameters.coreGain * CORE_TRIM, SMOOTHING);

      ramp(starterOsc.frequency, parameters.starterFrequency, FREQUENCY_SMOOTHING);
      // The exhaust resonance rides three harmonics above the firing rate, so
      // the putt keeps its character as the starter speeds up.
      ramp(starterFilter.frequency, parameters.starterFrequency * 3 + 120, FREQUENCY_SMOOTHING);
      ramp(starterGain.gain, parameters.starterGain * STARTER_TRIM, SMOOTHING);

      ramp(fireGain.gain, parameters.fireGain * FIRE_TRIM, SMOOTHING);
    },

    bang(): void {
      // A compressor stall drives the flow back out of the intake. It is a hard
      // low frequency slam with a crack of white on the front of it.
      playBurst(context, out, {
        frequency: 140,
        q: 0.7,
        gain: 0.85,
        attack: 0.001,
        decay: 0.32,
        color: 'pink',
      });
      playBurst(context, out, {
        frequency: 1900,
        q: 1.4,
        gain: 0.4,
        attack: 0.001,
        decay: 0.07,
      });
    },

    lightOff(): void {
      // The fuel that has pooled in the chambers goes at one time. It is a soft
      // swell and not a crack, so it takes 60 ms to reach its peak.
      playBurst(context, out, {
        frequency: 190,
        q: 0.5,
        gain: 0.5,
        attack: 0.06,
        decay: 0.55,
        color: 'pink',
      });
    },

    flameout(): void {
      // The flame goes and the pressure in the jet pipe falls away. The rotor
      // then runs down on its own, which the whine already follows.
      playBurst(context, out, {
        frequency: 320,
        q: 0.8,
        gain: 0.35,
        attack: 0.004,
        decay: 0.45,
        color: 'pink',
      });
      playTone(context, out, 420, 0.12, 0.5, 'sine');
    },

    dispose(): void {
      whineOsc.stop();
      starterOsc.stop();
      out.disconnect();
      panner.disconnect();
    },
  };
}
