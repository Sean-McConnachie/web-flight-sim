/**
 * The air over the airframe, and the shake when the flow comes off it.
 *
 * BEAD 5nl.
 *
 *
 * 1. THE GRAPH
 *
 *   pink noise -> low pass at the boundary layer band -> wind gain -> out
 *   pink noise -> low pass at 180 Hz -> buffet gain --------------> out
 *   sine at the buffet rate -> depth -> the gain of BOTH of the above
 *
 * The last row is the important one. The buffet is not a sound that is added on
 * top of the wind. It is the wind ITSELF going up and down, because that is
 * what a wing that sheds its flow does to the air over the tail. An oscillator
 * that drives the `gain` AudioParam of a GainNode is amplitude modulation, and
 * amplitude modulation is what the pilot feels.
 *
 *
 * 2. WHY THE BUFFET MATTERS MORE THAN IT SOUNDS
 *
 * The Me-262 has no stick shaker and no stall warning horn. The ONLY warning a
 * pilot gets before the wing lets go is the airframe starting to shake. Until
 * this bead the simulator gave a pilot no warning at all, because a screen
 * cannot shake a seat.
 *
 * src/audio/voices.ts opens the buffet at 14 degrees and holds it fully open at
 * the 20.3 degrees where the clean wing reaches its maximum lift. A pilot who
 * listens now has the same six degrees of warning the real aircraft gives.
 *
 *
 * 3. THE COCKPIT
 *
 * Nothing in an unpressurized fighter is louder than the air over the hood
 * seals. The wind therefore gets a boost in the cockpit view and the engines
 * get a low pass, which is the opposite way round from every outside view. See
 * COCKPIT_WIND_GAIN and CANOPY_CUTOFF in src/audio/voices.ts.
 */

import type { AirframeVoiceInput, AirframeVoiceParameters } from '@/audio/voices';
import { COCKPIT_WIND_GAIN, airframeVoice, createAirframeVoiceParameters } from '@/audio/voices';
import { createNoiseLoop } from '@/audio/noise';

/** How fast a smoothed parameter follows the frame, s. */
const SMOOTHING = 0.04;

/** How loud the rush of air is at the 250 m/s reference speed. */
const WIND_TRIM = 0.34;

/** How loud the low frequency part of a fully developed buffet is. */
const BUFFET_TRIM = 0.3;

/**
 * How deep the buffet cuts into the wind noise, as a fraction.
 *
 * At 0.7 a fully developed buffet takes the rush of air from 30 percent to 170
 * percent of its steady level, four times a second. That is a shake nobody
 * misses, and it stops short of chopping the sound off, which would read as a
 * fault in the simulator instead of a warning from the aircraft.
 */
const BUFFET_MODULATION_DEPTH = 0.7;

/** Where the low frequency body of the buffet sits, Hz. */
const BUFFET_BODY_CUTOFF = 180;

export interface AirframeVoice {
  /**
   * Writes one frame of flight state into the nodes.
   *
   * `cockpit` is true while the listener sits under the hood. It only changes
   * the level of the wind, for the reason in section 3.
   */
  update(input: AirframeVoiceInput, cockpit: boolean): void;
  /**
   * The gain the wind node holds NOW, read back off the audio thread.
   *
   * It is a development hook for `tools/audio-check.mjs`, which cannot reach
   * inside this closure and needs to tell a wrong gain from a graph that never
   * connected. Nothing inside src reads it.
   */
  readonly windLevel: number;
  dispose(): void;
}

export function createAirframeVoice(
  context: BaseAudioContext,
  destination: AudioNode,
): AirframeVoice {
  const parameters: AirframeVoiceParameters = createAirframeVoiceParameters();

  // --- The rush of air ----------------------------------------------------
  const windFilter = context.createBiquadFilter();
  windFilter.type = 'lowpass';
  windFilter.frequency.value = 200;
  windFilter.Q.value = 0.5;
  const windGain = context.createGain();
  windGain.gain.value = 0;
  const windNoise = createNoiseLoop(context, 'pink');
  windNoise.connect(windFilter);
  windFilter.connect(windGain);
  windGain.connect(destination);

  // --- The body of the shake ----------------------------------------------
  const buffetFilter = context.createBiquadFilter();
  buffetFilter.type = 'lowpass';
  buffetFilter.frequency.value = BUFFET_BODY_CUTOFF;
  buffetFilter.Q.value = 1;
  const buffetGain = context.createGain();
  buffetGain.gain.value = 0;
  const buffetNoise = createNoiseLoop(context, 'pink');
  buffetNoise.connect(buffetFilter);
  buffetFilter.connect(buffetGain);
  buffetGain.connect(destination);

  // --- The shake itself ---------------------------------------------------
  // The oscillator runs at all times. `windDepth` and `buffetDepth` are what
  // decide whether it reaches anything, and both sit at zero in smooth air.
  const shake = context.createOscillator();
  shake.type = 'sine';
  shake.frequency.value = 15;
  const windDepth = context.createGain();
  windDepth.gain.value = 0;
  const buffetDepth = context.createGain();
  buffetDepth.gain.value = 0;
  shake.connect(windDepth);
  shake.connect(buffetDepth);
  // An oscillator that reaches an AudioParam ADDS to the value the automation
  // holds. That is amplitude modulation, and section 1 says why it is the
  // correct shape for a buffet.
  windDepth.connect(windGain.gain);
  buffetDepth.connect(buffetGain.gain);
  shake.start();

  function ramp(param: AudioParam, value: number): void {
    param.setTargetAtTime(value, context.currentTime, SMOOTHING);
  }

  return {
    update(input: AirframeVoiceInput, cockpit: boolean): void {
      airframeVoice(input, parameters);
      const trim = WIND_TRIM * (cockpit ? COCKPIT_WIND_GAIN : 1);
      const wind = parameters.windGain * trim;
      const buffet = parameters.buffetDepth * parameters.windGain * BUFFET_TRIM;

      ramp(windFilter.frequency, parameters.windCutoff);
      ramp(windGain.gain, wind);
      ramp(buffetGain.gain, buffet);

      ramp(shake.frequency, parameters.buffetFrequency);
      // The depth of the shake follows the level it is shaking, so a buffet in
      // thin air at low speed is quiet and one at the placard is not.
      ramp(windDepth.gain, wind * parameters.buffetDepth * BUFFET_MODULATION_DEPTH);
      ramp(buffetDepth.gain, buffet * BUFFET_MODULATION_DEPTH);
    },

    get windLevel(): number {
      return windGain.gain.value;
    },

    dispose(): void {
      shake.stop();
      windGain.disconnect();
      buffetGain.disconnect();
      windDepth.disconnect();
      buffetDepth.disconnect();
    },
  };
}
