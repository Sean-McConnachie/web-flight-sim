/**
 * Noise, and the one shot burst that every impact of the simulator is made of.
 *
 * BEAD c2c and BEAD 5nl.
 *
 *
 * 1. WHY THE BUFFERS ARE MADE ONE TIME
 *
 * A jet, the wind over a canopy and a tire on concrete are all NOISE with a
 * filter over it. Building that noise costs a few hundred thousand random
 * numbers, and building it every time a wheel touches would cost that on the
 * frame the wheel touches. Both buffers are therefore made one time, at the
 * first call, and every voice reads the same two.
 *
 *
 * 2. WHITE AND PINK
 *
 * White noise holds the same power in every hertz. It sounds like a hiss,
 * because a listener hears in octaves and every octave up holds twice the band
 * and so twice the power.
 *
 * Pink noise holds the same power in every OCTAVE. It is what wind, a jet and
 * running water all sound like, and it is the right source for almost
 * everything here. The one place white wins is a sharp impact, where the extra
 * top end is the crack of the report.
 *
 * The pink filter is the Kellet three pole fit. It follows a fall of 3 dB per
 * octave to inside 0.05 dB from 10 Hz to 20 kHz, which is far closer than
 * anything a listener can hear.
 * Source: Paul Kellet, on the music-dsp list, 1999. Confidence: firm.
 *
 *
 * 3. WHY THE NOISE IS SEEDED
 *
 * src/core/prng.ts holds the generator the world scatter uses. The noise takes
 * its numbers from the same generator with a fixed seed, so every run of the
 * simulator makes the same noise buffer. That is not for the ear, which cannot
 * tell one noise from another. It is so that a fault in a sound is the same
 * fault on the next run.
 */

import { mulberry32 } from '@/core/prng';

/**
 * Length of a noise buffer, s.
 *
 * A loop that is too short reads as a tone at the loop rate. Two seconds puts
 * that rate at 0.5 Hz, which is below hearing, and the buffer costs 384 kB at
 * a 48 kHz sample rate. Both buffers together cost under a megabyte.
 */
const NOISE_SECONDS = 2;

/** The seed of the noise. Section 3 of the module comment says why it is fixed. */
const NOISE_SEED = 0x4a756d6f; // "Jumo"

/**
 * Level both buffers are normalized to, as a root mean square.
 *
 * A LOUDNESS IS AN RMS AND NOT A PEAK. A peak is one sample out of ninety six
 * thousand, and it says nothing about how loud a noise sounds. Normalizing the
 * two buffers to the same PEAK left them at two different loudnesses, so a mix
 * trim of 0.3 meant one thing on the white buffer and another on the pink.
 *
 * The measured crest factor of the pink buffer is 4.3, so this correction is
 * worth about 2 dB. It is a correctness fix and not a loudness fix.
 *
 * 0.3 leaves 10 dB of room over the RMS for the peaks of a pink buffer, which
 * is what stops the normalization from clipping.
 */
const TARGET_RMS = 0.3;

/**
 * Where the sub audio wander of the pink filter is removed, Hz.
 *
 * Nothing under 20 Hz reaches a listener through a laptop speaker or a pair of
 * headphones. It still takes the headroom, so it goes.
 */
const PINK_HIGHPASS = 30; // Hz

/**
 * Scales a buffer so its root mean square is TARGET_RMS.
 *
 * A peak past one would clip on the way out, so the scale comes back down when
 * the normalized peak passes 0.99. That happens on the pink buffer, and it
 * leaves it at an RMS of 0.234 against the 0.300 of the white buffer.
 */
function normalize(data: Float32Array): void {
  let sum = 0;
  for (let i = 0; i < data.length; i++) sum += data[i] * data[i];
  const rms = Math.sqrt(sum / data.length);
  if (rms <= 0) return;
  let scale = TARGET_RMS / rms;
  let peak = 0;
  for (let i = 0; i < data.length; i++) {
    const magnitude = Math.abs(data[i]) * scale;
    if (magnitude > peak) peak = magnitude;
  }
  if (peak > 0.99) scale *= 0.99 / peak;
  for (let i = 0; i < data.length; i++) data[i] *= scale;
}

/** The two buffers, built at the first call and then held. */
interface NoiseBuffers {
  readonly white: AudioBuffer;
  readonly pink: AudioBuffer;
}

/** One set of buffers for each context. A context holds its own memory. */
const cache = new WeakMap<BaseAudioContext, NoiseBuffers>();

/** Builds both noise buffers. It runs one time for each audio context. */
function buildNoise(context: BaseAudioContext): NoiseBuffers {
  const length = Math.floor(context.sampleRate * NOISE_SECONDS);
  const random = mulberry32(NOISE_SEED);

  const white = context.createBuffer(1, length, context.sampleRate);
  const pink = context.createBuffer(1, length, context.sampleRate);
  const whiteData = white.getChannelData(0);
  const pinkData = pink.getChannelData(0);

  // The three poles of the Kellet fit.
  let b0 = 0;
  let b1 = 0;
  let b2 = 0;

  for (let i = 0; i < length; i++) {
    const sample = random() * 2 - 1;
    whiteData[i] = sample;

    b0 = 0.99765 * b0 + sample * 0.0990460;
    b1 = 0.96300 * b1 + sample * 0.2965164;
    b2 = 0.57000 * b2 + sample * 1.0526913;
    pinkData[i] = b0 + b1 + b2 + sample * 0.1848;
  }

  // Take the sub audio wander out of the pink buffer. It is a one pole high
  // pass, run in place. See PINK_HIGHPASS.
  const alpha = 1 / (1 + (2 * Math.PI * PINK_HIGHPASS) / context.sampleRate);
  let lastIn = pinkData[0];
  let lastOut = 0;
  for (let i = 0; i < length; i++) {
    const value = pinkData[i];
    lastOut = alpha * (lastOut + value - lastIn);
    lastIn = value;
    pinkData[i] = lastOut;
  }

  // Both buffers then carry the same LOUDNESS. Read the comment on TARGET_RMS
  // before you change either of these two lines.
  normalize(whiteData);
  normalize(pinkData);

  // The loop point is a join between two unrelated samples, which is a click.
  // Fade the last 10 ms into the first 10 ms so the join is continuous.
  const fade = Math.floor(context.sampleRate * 0.01);
  for (let i = 0; i < fade; i++) {
    const t = i / fade;
    whiteData[i] = whiteData[i] * t + whiteData[length - fade + i] * (1 - t);
    pinkData[i] = pinkData[i] * t + pinkData[length - fade + i] * (1 - t);
  }

  return { white, pink };
}

/** Returns the two noise buffers of one context, and builds them if needed. */
export function noiseBuffers(context: BaseAudioContext): NoiseBuffers {
  let buffers = cache.get(context);
  if (buffers === undefined) {
    buffers = buildNoise(context);
    cache.set(context, buffers);
  }
  return buffers;
}

/**
 * Starts a looping noise source that runs for the life of the page.
 *
 * A voice that is on all the time never stops its source. It holds the level at
 * zero instead. A Web Audio source that stops can never start again, so a voice
 * that stopped its source would have to build a new one every time the pilot
 * opened the throttle.
 */
export function createNoiseLoop(context: BaseAudioContext, color: 'white' | 'pink'): AudioNode {
  const source = context.createBufferSource();
  source.buffer = noiseBuffers(context)[color];
  source.loop = true;
  source.start();
  return source;
}

/**
 * How far ahead of now a one shot is scheduled, s.
 *
 * `context.currentTime` is the start of the render quantum the audio thread is
 * ALREADY WORKING ON. A burst anchored exactly there asks the audio thread to
 * play something it has partly gone past, so the front of the envelope is lost.
 * A gun report is mostly its front, and it can vanish.
 *
 * 20 ms puts the whole envelope in front of the audio thread. It is under the
 * 100 ms that a listener can tie to a picture, so nothing reads as late.
 */
const LOOKAHEAD = 0.02; // s

/**
 * Counts every burst, so that two bursts never read the same noise.
 *
 * The read point used to come from the start time alone. `currentTime` DOES NOT
 * ADVANCE inside one JavaScript task, so the four guns that fire on one physics
 * frame all got the same start time, the same read point, and therefore the
 * same samples. Four identical bursts add COHERENTLY. They came out four times
 * as loud as one and they sounded like one gun, not four.
 *
 * A counter separates them whatever the clock does.
 */
let burstCount = 0;

/** What one burst sounds like. Every impact of the simulator is one of these. */
export interface BurstOptions {
  /** Center of the band pass, Hz. It is the pitch of the knock. */
  frequency: number;
  /** How narrow the band is. A low Q is a thud and a high Q is a ring. */
  q: number;
  /** Peak level, 0 to 1. */
  gain: number;
  /** How long the burst takes to reach its peak, s. */
  attack: number;
  /** How long it takes to die away, s. */
  decay: number;
  /** White for a sharp crack, pink for a body blow. */
  color?: 'white' | 'pink';
  /** Where the burst starts, in context time. Zero means now. */
  when?: number;
}

/**
 * Plays one burst of noise and frees itself.
 *
 * THE DECAY IS AN EXPONENTIAL AND NOT A LINE. Every real impact dies away as an
 * exponential, because the energy that is left falls by a fixed FRACTION in
 * every unit of time. A linear fade sounds like a machine turning a sound off.
 *
 * `exponentialRampToValueAtTime` cannot reach zero, so the ramp goes to a value
 * near it and a `setValueAtTime` closes the door.
 */
export function playBurst(
  context: BaseAudioContext,
  destination: AudioNode,
  options: BurstOptions,
): void {
  const start =
    options.when !== undefined && options.when > 0
      ? options.when
      : context.currentTime + LOOKAHEAD;
  const source = context.createBufferSource();
  source.buffer = noiseBuffers(context)[options.color ?? 'white'];
  source.loop = true;
  // Every burst reads the buffer from a different place, so ten shots in a row
  // are ten different sounds out of one buffer. The counter and not the clock
  // separates them. See `burstCount`.
  burstCount++;
  const span = source.buffer.duration - options.attack - options.decay - 0.01;
  const offset = (burstCount * 0.6180339887 * span) % span;

  const band = context.createBiquadFilter();
  band.type = 'bandpass';
  band.frequency.value = options.frequency;
  band.Q.value = options.q;

  const envelope = context.createGain();
  envelope.gain.value = 0;
  envelope.gain.setValueAtTime(0, start);
  envelope.gain.linearRampToValueAtTime(options.gain, start + options.attack);
  envelope.gain.exponentialRampToValueAtTime(
    Math.max(options.gain, 1e-4) * 1e-3,
    start + options.attack + options.decay,
  );
  envelope.gain.setValueAtTime(0, start + options.attack + options.decay);

  source.connect(band);
  band.connect(envelope);
  envelope.connect(destination);

  source.start(start, Math.max(offset, 0));
  source.stop(start + options.attack + options.decay + 0.01);
  // A stopped source is dead for good, so it drops its own connections. Without
  // this the graph grows by three nodes for every round the guns fire.
  source.addEventListener('ended', () => {
    source.disconnect();
    band.disconnect();
    envelope.disconnect();
  });
}

/**
 * Plays one falling tone and frees itself.
 *
 * A burst of noise cannot make a ring. A gear lock, a case that hits the floor
 * and a shell that strikes metal all ring at a pitch, so they need an
 * oscillator and not a filter over noise.
 */
export function playTone(
  context: BaseAudioContext,
  destination: AudioNode,
  frequency: number,
  gain: number,
  decay: number,
  type: OscillatorType = 'triangle',
): void {
  const start = context.currentTime + LOOKAHEAD;
  const oscillator = context.createOscillator();
  oscillator.type = type;
  oscillator.frequency.setValueAtTime(frequency, start);
  // A struck object loses its stiffness as it rings, so the pitch falls a
  // little. A tone that holds its pitch reads as a signal and not as a knock.
  oscillator.frequency.exponentialRampToValueAtTime(frequency * 0.75, start + decay);

  const envelope = context.createGain();
  envelope.gain.setValueAtTime(0, start);
  envelope.gain.linearRampToValueAtTime(gain, start + 0.002);
  envelope.gain.exponentialRampToValueAtTime(Math.max(gain, 1e-4) * 1e-3, start + decay);
  envelope.gain.setValueAtTime(0, start + decay);

  oscillator.connect(envelope);
  envelope.connect(destination);
  oscillator.start(start);
  oscillator.stop(start + decay + 0.01);
  oscillator.addEventListener('ended', () => {
    oscillator.disconnect();
    envelope.disconnect();
  });
}
