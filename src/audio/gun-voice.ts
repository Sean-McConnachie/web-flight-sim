/**
 * The four MK 108, and what their shells hit.
 *
 * BEAD 5nl.
 *
 *
 * 1. WHY THE MK 108 SOUNDS THE WAY IT DOES
 *
 * The crews called it the Presslufthammer, the pneumatic hammer, and the name
 * is a description of the sound and not of the weapon. Three things make it.
 *
 *   The rate is SLOW. 650 rounds per minute over four guns is one round every
 *   23 ms, and a listener counts that instead of hearing a tone.
 *   The charge is SMALL. The Minengeschoss leaves at 505 m/s, where a rifle
 *   round leaves at over 800. A low muzzle velocity gives a thump and not a
 *   crack, because the blast never goes supersonic in the way a rifle does.
 *   The action is BLOWBACK. Nothing locks the breech. A heavy bolt runs the
 *   whole cycle on the gas pressure alone and it slams both ends of its travel,
 *   so the mechanism is as loud as the shot.
 *
 * The voice therefore builds three parts for every round: a low body, a mid
 * crack and a metallic clack from the bolt. src/weapons/mk108.ts holds the rate
 * and every round it fires arrives here.
 *
 *
 * 2. THE SHELL LANDS SOMEWHERE ELSE
 *
 * A muzzle report starts at the aircraft. An impact starts wherever the shell
 * came down, which src/weapons/ballistics.ts lets run to 2500 m. The two
 * therefore CANNOT share a propagation stage.
 *
 * The muzzle goes onto the aircraft bus with the engines. The impact is
 * scheduled into the future by its own distance over the speed of sound, and
 * that is not a trick. A shell that lands 1400 m away really is heard four
 * seconds after it lands, and a pilot who fires at a ground target and then
 * hears it three seconds later is hearing the correct answer.
 */

import { mulberry32 } from '@/core/prng';
import { distanceGain } from '@/audio/voices';
import { playBurst, playTone } from '@/audio/noise';

/**
 * The variation between one round and the next.
 *
 * Ten identical reports in a row read as a loop of one sample, which is the
 * thing this project does not do. A few percent on the pitch of each part is
 * enough to break it. The generator is seeded, so the variation is the same on
 * every run. See section 3 of src/audio/noise.ts.
 */
const SHOT_SEED = 0x4d4b3130; // "MK10"

/** How far the pitch of one round moves, as a fraction. */
const SHOT_VARIATION = 0.08;

/** Rounds in the air at one time that the voice will sound. */
const MAX_CONCURRENT_SHOTS = 6;

export interface GunVoice {
  /** One round leaves a barrel. */
  shot(): void;
  /**
   * One shell arrives. `distance` is from the LISTENER to the impact, in m.
   *
   * `destroyed` is true when the shell finished the target off, which is a
   * bigger event than a hit that did not.
   */
  impact(distance: number, speedOfSound: number, destroyed: boolean): void;
  dispose(): void;
}

export function createGunVoice(
  context: BaseAudioContext,
  /** The aircraft bus. The muzzle report rides with the aircraft. */
  destination: AudioNode,
  /** The master bus. An impact carries its own distance, so it goes direct. */
  worldDestination: AudioNode,
): GunVoice {
  const random = mulberry32(SHOT_SEED);
  /** Rounds this voice has already sounded in the current audio block. */
  let concurrent = 0;
  let concurrentTime = -1;

  /** A pitch factor near one, different for every round. */
  function vary(): number {
    return 1 + (random() * 2 - 1) * SHOT_VARIATION;
  }

  return {
    shot(): void {
      // Four guns can fire inside one frame. The ear cannot separate more than
      // a few reports that arrive together, and every one of them costs three
      // filters, so the rest are dropped rather than summed into a wall.
      //
      // `currentTime` does not advance inside one JavaScript task, so this is
      // in practice a cap per FRAME and not per audio block. On a frame that
      // holds two hundred physics steps that cap does real work. On a frame
      // that holds four it never fires, because four guns is under the cap.
      const now = context.currentTime;
      if (now !== concurrentTime) {
        concurrentTime = now;
        concurrent = 0;
      }
      if (concurrent >= MAX_CONCURRENT_SHOTS) return;
      concurrent++;

      const pitch = vary();

      // The body of the blast. A 30 mm at 505 m/s is a deep thump.
      //
      // THE LEVELS ARE HIGH ON PURPOSE. Four 30 mm cannon sit in the nose, a
      // meter from the pilot, behind nothing but an armored bulkhead. They are
      // louder than the slipstream at any speed the aircraft can reach, and
      // `tools/audio-check.mjs` measured them level with the wind before this.
      // The master compressor is what holds the peaks, and it is there so that
      // the loudest thing on the aircraft can be the loudest thing in the mix.
      playBurst(context, destination, {
        frequency: 105 * pitch,
        q: 0.8,
        gain: 0.95,
        attack: 0.0008,
        decay: 0.14,
        color: 'pink',
      });
      // The crack on the front of it. A wider band than the body, because a
      // blast wave is not a tone.
      playBurst(context, destination, {
        frequency: 700 * pitch,
        q: 0.7,
        gain: 0.6,
        attack: 0.0004,
        decay: 0.05,
      });
      // The bolt. Section 1 says why the mechanism is as loud as the round.
      playBurst(context, destination, {
        frequency: 2400 * pitch,
        q: 4,
        gain: 0.3,
        attack: 0.0004,
        decay: 0.03,
      });
    },

    impact(distance: number, speedOfSound: number, destroyed: boolean): void {
      const gain = distanceGain(Math.max(distance, 0));
      // Under a hundredth the report is below everything else in the mix, and
      // scheduling it would cost three filters for nothing.
      if (gain < 0.01) return;
      const delay = Math.max(distance, 0) / Math.max(speedOfSound, 1);
      const when = context.currentTime + delay;

      // The far the impact, the less of it arrives above a few hundred hertz.
      // Section 7 of src/audio/voices.ts holds the same law for the continuous
      // sources. A one shot only needs the shape of it.
      const far = Math.min(distance / 800, 1);
      playBurst(context, worldDestination, {
        frequency: (destroyed ? 90 : 130) * (1 - 0.4 * far),
        q: 0.6,
        gain: gain * (destroyed ? 0.9 : 0.5),
        attack: 0.002,
        decay: (destroyed ? 0.55 : 0.22) * (1 + far),
        color: 'pink',
        when,
      });
      // The shell case of a Minengeschoss is thin and it opens up. Near to, the
      // crack of that is most of what a listener hears.
      if (far < 0.7) {
        playBurst(context, worldDestination, {
          frequency: 1500,
          q: 1.2,
          gain: gain * 0.3 * (1 - far),
          attack: 0.0006,
          decay: 0.09,
          when,
        });
      }
      if (destroyed && far < 0.9) {
        // Metal that tears.
        playTone(context, worldDestination, 340, gain * 0.16, 0.7, 'sawtooth');
      }
    },

    dispose(): void {
      // Every sound of this voice frees itself when it ends. Nothing is held.
    },
  };
}
