/**
 * The sound system. It joins every voice onto one bus and drives them.
 *
 * BEAD a23. It is the composition root of src/audio, in the way that
 * src/main.ts is the composition root of the simulator.
 *
 *
 * 1. THE BUS
 *
 *   engine 0 -\
 *   engine 1 --+-> canopy -\
 *                           +-> aircraft -> delay -> absorption -> spread ->
 *   airframe ---------------|                                       -> pan ->
 *   gear and tires ---------|                                          master
 *   the four guns ----------/
 *
 *   shell impacts ------------------------------------------------> master
 *
 * Everything that is ON the aircraft shares one propagation stage, because
 * everything on the aircraft is in the same place. An impact is not, so it
 * carries its own distance. Section 2 of src/audio/gun-voice.ts says why.
 *
 * The canopy low pass is only on the engines. A gear leg locking is
 * STRUCTURE borne, so the pilot hears it through the airframe and not through
 * the hood, and the wind over the hood seals is louder inside than out.
 *
 *
 * 2. THE DELAY LINE IS THE DOPPLER SHIFT
 *
 * There is no pitch calculation anywhere in src/audio. There is one delay line,
 * it holds the distance over the speed of sound, and the ramp it takes to a new
 * value each frame IS the Doppler shift. Read `propagation` in
 * src/audio/voices.ts for the identity and the unit test that measures it.
 *
 * This is also what gives a fly by its arrival. An aircraft 340 m away is heard
 * where it was one second ago, so the sound reaches the fly by camera after the
 * aircraft has already gone past it. Nothing in this file arranges that. It
 * falls out of holding the delay at the correct value.
 *
 * THE ONE CASE THE DELAY LINE CAN NOT HANDLE is a jump. A respawn or a change
 * of view moves the listener hundreds of meters between two frames, and a delay
 * line that ramps across that would sweep the pitch by a factor of fifty. The
 * frame therefore checks the size of the change, and a jump gets a hard set and
 * a 120 ms duck instead of a ramp. See DELAY_SNAP_RATE, which is a RATE and not
 * a fixed step, and which says what happened when it was one.
 *
 *
 * 3. WHY THERE ARE TWO UPDATE CALLS
 *
 * `fixedUpdate` runs on every physics step at 240 Hz. `update` runs one time
 * per frame. A frame can hold four steps or none.
 *
 * Anything that lasts a single step must be caught in `fixedUpdate`, or a
 * frame will miss it. A surge bang, a round leaving a barrel and a shell
 * arriving are all one step long. src/render/weapons.ts collects the muzzle
 * flashes on the same schedule and for the same reason.
 *
 * Anything continuous is written in `update`, because writing a gain 240 times
 * a second buys nothing over writing it 60 times and letting the audio thread
 * smooth between them.
 */

import type { PerspectiveCamera } from 'three';
import { Vector3 } from 'three';

import type { Aircraft } from '@/aircraft/aircraft';
import type { Engine } from '@/aircraft/me262/engine';
import { SELF_SUSTAIN_RPM } from '@/aircraft/me262/engine';
import { createAirframeVoice } from '@/audio/airframe-voice';
import type { AudioBus } from '@/audio/context';
import { createAudioBus } from '@/audio/context';
import { createEngineVoice } from '@/audio/engine-voice';
import { createGunVoice } from '@/audio/gun-voice';
import type { MechanicalVoiceInput } from '@/audio/mechanical-voice';
import { createMechanicalVoice } from '@/audio/mechanical-voice';
import { playBurst, playTone } from '@/audio/noise';
import type {
  AirframeVoiceInput,
  EngineVoiceInput,
  PropagationParameters,
} from '@/audio/voices';
import {
  CANOPY_CUTOFF,
  MAXIMUM_DELAY,
  createPropagationParameters,
  propagation,
} from '@/audio/voices';
import { clamp } from '@/math/tables';
import { threeToNed } from '@/render/frames';
import type { Armament } from '@/weapons/armament';

/** How far the engines sit out on the wings, as a pan. */
const ENGINE_PAN = 0.45;

/** How fast a smoothed parameter follows the frame, s. */
const SMOOTHING = 0.04;

/**
 * How fast the delay may honestly change, in seconds of delay per second.
 *
 * THIS IS A RATE AND NOT A FIXED STEP. It WAS a fixed step of 50 ms, and that
 * was wrong in a way only `tools/audio-check.mjs` could catch.
 *
 * The delay is the distance over the speed of sound, so its rate of change is
 * the radial speed over the speed of sound. That is bounded, because the
 * aircraft flies below Mach 0.86 and the camera adds its own motion on top. A
 * rate of 1.5 covers the two together with room to spare.
 *
 * A fixed step compares a per frame change against a number that has no frame
 * in it. The old 50 ms step held on a 16 ms frame, where the aircraft moves
 * 4 m. It FAILED on a slow frame. A software rasterizer draws in 430 ms, and
 * the same aircraft covers 107 m in that time, which is six times the old step.
 * Every frame then looked like a jump, so the bus ducked on every frame and the
 * sound broke up on exactly the machines that are already struggling.
 */
const DELAY_SNAP_RATE = 1.5;

/**
 * The smallest change that can count as a jump, s.
 *
 * A frame time near zero would otherwise make every change a jump, which is the
 * same fault the rate above fixes, at the other end of the range.
 */
const DELAY_SNAP_FLOOR = 0.01; // s

/** How long the bus stays down after a jump, s. */
const DUCK_TIME = 0.12; // s

/**
 * Oleo closing rate that counts as an arrival, m/s.
 *
 * A greaser closes the strut at a few tenths of a meter per second and a firm
 * arrival at two to three. 3 m/s is therefore the loudest touch down the sound
 * gives, and everything over it is the same sound. Confidence: estimated.
 */
const TOUCHDOWN_RATE_FULL = 3; // m/s

/**
 * How hard the airframe can drag on the ground, N m/s.
 *
 * It is the weight of the loaded aircraft against a slide of 30 m/s, which is a
 * belly landing at 108 km/h. The scrape is at its loudest there.
 */
const SCRAPE_REFERENCE = 6396 * 9.80665 * 30;

/** True while a flame burns in the chambers. It matches isLit of aircraft.ts. */
function isLit(engine: Engine): boolean {
  const state = engine.state;
  return (
    state === 'lightOff' ||
    state === 'idle' ||
    state === 'running' ||
    state === 'stall' ||
    state === 'fire'
  );
}

/**
 * True while the Riedel turns the rotor.
 *
 * The engine does not report the starter, so this is a proxy from the two
 * states in which the handbook has it engaged. Read StartPhase in
 * src/aircraft/me262/engine.ts: the pilot pulls the handle at step 5 and
 * releases it at step 8, which is the moment the rotor passes the self sustain
 * speed inside the `lightOff` state.
 */
function starterRunning(engine: Engine): boolean {
  if (engine.state === 'starter') return true;
  return engine.state === 'lightOff' && engine.rpm < SELF_SUSTAIN_RPM;
}

/**
 * What the propagation stage is doing right now.
 *
 * A development hook for `tools/audio-check.mjs`, in the same spirit as the
 * `window.sim` handle of src/main.ts. Nothing inside src reads it.
 *
 * An analyser on the master bus reports ONE number, and a mix that is 26 dB
 * down has a dozen possible causes. These are the numbers that tell them apart.
 */
export interface SoundMetrics {
  /** m, from the listener to the aircraft. */
  distance: number;
  /** -1 to 1. */
  pan: number;
  /** The inverse distance gain the stage holds now. */
  spread: number;
  /** Hz, the air absorption corner. */
  cutoff: number;
  /** s. */
  delay: number;
  /**
   * The gain of the aircraft bus, read off the audio thread. A duck takes it
   * to zero, so a value under one says a jump just happened.
   */
  busGain: number;
  /** How many times the delay line has snapped since the page loaded. */
  snaps: number;
  /** The gain of the wind node, read off the audio thread. */
  windLevel: number;
  /** Frame time of the last update, s. The snap bound is a rate against it. */
  frameDt: number;
}

export interface SoundSystem {
  /** The master bus, or null when the browser has no Web Audio. */
  readonly bus: AudioBus | null;
  /** See SoundMetrics. It is a development hook and nothing in src reads it. */
  readonly metrics: SoundMetrics;
  /**
   * Catches everything that lasts one physics step. Section 3 says why it is
   * separate from `update`.
   */
  fixedUpdate(aircraft: Aircraft, armament: Armament): void;
  /**
   * Writes one frame of flight state into every voice.
   *
   * `aircraftRenderPosition` is where the frame DREW the aircraft, in the
   * render frame, so the sound and the picture agree to the same interpolation.
   */
  update(
    aircraft: Aircraft,
    camera: PerspectiveCamera,
    aircraftRenderPosition: Vector3,
    cockpit: boolean,
    dt: number,
  ): void;
  /** The airframe, an engine or a tire failed. src/main.ts already listens. */
  failure(): void;
  dispose(): void;
}

/**
 * A sound system that does nothing.
 *
 * A browser with no Web Audio must still fly. Every call below is a no-op, so
 * src/main.ts needs no test for the null case.
 */
function createNullSoundSystem(): SoundSystem {
  return {
    bus: null,
    metrics: {
      distance: 0,
      pan: 0,
      spread: 0,
      cutoff: 0,
      delay: 0,
      busGain: 0,
      snaps: 0,
      windLevel: 0,
      frameDt: 0,
    },
    fixedUpdate(): void {},
    update(): void {},
    failure(): void {},
    dispose(): void {},
  };
}

export function createSoundSystem(): SoundSystem {
  const master = createAudioBus();
  if (master === null) return createNullSoundSystem();
  // A second name, so that every closure below reads a type that cannot be
  // null. TypeScript does not carry the guard above into a hoisted function.
  const bus: AudioBus = master;
  const context = bus.context;

  // --- The propagation stage ----------------------------------------------
  const aircraftBus = context.createGain();
  aircraftBus.gain.value = 1;

  const delay = context.createDelay(MAXIMUM_DELAY);
  delay.delayTime.value = 0;

  const absorption = context.createBiquadFilter();
  absorption.type = 'lowpass';
  absorption.frequency.value = 20_000;
  absorption.Q.value = 0.5;

  const spread = context.createGain();
  spread.gain.value = 1;

  /**
   * The gain a jump takes down. It sits AFTER the delay line, and that is not
   * an accident.
   *
   * The duck used to act on `aircraftBus`, which is the INPUT of the delay
   * line. A delay line holds a tenth of a second of sound that has already gone
   * in, so moving its read point still throws that buffered sound at the
   * listener as a click. Ducking the input cannot reach it. Ducking the output
   * can.
   */
  const duckGain = context.createGain();
  duckGain.gain.value = 1;

  const panner = context.createStereoPanner();
  panner.pan.value = 0;

  aircraftBus.connect(delay);
  delay.connect(absorption);
  absorption.connect(duckGain);
  duckGain.connect(spread);
  spread.connect(panner);
  panner.connect(bus.destination);

  // --- The canopy ----------------------------------------------------------
  const canopy = context.createBiquadFilter();
  canopy.type = 'lowpass';
  canopy.frequency.value = 20_000;
  canopy.Q.value = 0.4;
  canopy.connect(aircraftBus);

  // --- The voices ----------------------------------------------------------
  const engines = [
    createEngineVoice(context, canopy, { index: 0, pan: -ENGINE_PAN }),
    createEngineVoice(context, canopy, { index: 1, pan: ENGINE_PAN }),
  ];
  const airframe = createAirframeVoice(context, aircraftBus);
  const mechanical = createMechanicalVoice(context, aircraftBus);
  const guns = createGunVoice(context, aircraftBus, bus.destination);

  // --- The scratch records. No frame allocates one. -------------------------
  const engineInput: EngineVoiceInput = {
    rotorSpeed: 0,
    thrust: 0,
    fuelFlow: 0,
    trueAirspeed: 0,
    lit: false,
    starterRunning: false,
    onFire: false,
  };
  const airframeInput: AirframeVoiceInput = {
    trueAirspeed: 0,
    alpha: 0,
    mach: 0,
    gearPosition: 0,
    flapPosition: 0,
  };
  const mechanicalInput: MechanicalVoiceInput = {
    rolling: { wheelSpeed: 0, wheelLoad: 0, onGround: false, slip: 0 },
    gearRate: 0,
    flapRate: 0,
    scrape: 0,
  };
  const propagationOut: PropagationParameters = createPropagationParameters();
  const toAircraft = new Vector3();
  const listenerNed = new Vector3();

  // --- What the last frame and the last step saw ---------------------------
  const surgeCounts = [0, 0];
  const flameoutCounts = [0, 0];
  const legOnGround: boolean[] = [];
  const legBurst: boolean[] = [];
  let previousGearPosition = -1;
  let previousFlapPosition = -1;
  let previousFlapRate = 0;
  let previousDelay = -1;
  let started = false;

  /**
   * Simulated time each of the two update calls last saw.
   *
   * A SPAWN MUST NOT MAKE A SOUND. `spawnOnRunway` puts the aircraft on its
   * wheels and drops the gear, both in one step. Every latch below then reads
   * that as a landing and a gear lock, so the page used to open with a thump
   * and a respawn used to answer with one.
   *
   * `Aircraft.time` runs from the last spawn, so a value that went DOWN is a
   * spawn. The two calls hold their own copy, because a respawn happens inside
   * the frame and each call has to catch it on its own next visit.
   */
  let fixedTime = -1;
  let frameTime = -1;

  /** True while a sound would reach nobody. It saves building nodes for it. */
  function silent(): boolean {
    return bus.muted || bus.volume <= 0 || bus.blocked;
  }

  function ramp(param: AudioParam, value: number): void {
    param.setTargetAtTime(value, context.currentTime, SMOOTHING);
  }

  /** Takes the output of the delay line down and brings it back. */
  function duck(): void {
    const now = context.currentTime;
    duckGain.gain.cancelScheduledValues(now);
    duckGain.gain.setValueAtTime(0, now);
    duckGain.gain.linearRampToValueAtTime(0, now + DUCK_TIME * 0.5);
    duckGain.gain.linearRampToValueAtTime(1, now + DUCK_TIME);
  }

  // The development hook. `update` writes into it and nothing in src reads it.
  const metrics: SoundMetrics = {
    distance: 0,
    pan: 0,
    spread: 1,
    cutoff: 20_000,
    delay: 0,
    busGain: 1,
    snaps: 0,
    windLevel: 0,
    frameDt: 0,
  };

  return {
    bus,
    metrics,

    fixedUpdate(aircraft: Aircraft, armament: Armament): void {
      // A spawn resets every latch WITHOUT playing what it finds. See the
      // comment on `fixedTime`.
      const spawned = aircraft.time < fixedTime;
      fixedTime = aircraft.time;
      if (spawned) {
        const legs = aircraft.state.gear.legs;
        for (let i = 0; i < legs.length; i++) {
          legOnGround[i] = legs[i].onGround;
          legBurst[i] = legs[i].burst;
        }
        for (let i = 0; i < aircraft.state.engines.length && i < 2; i++) {
          surgeCounts[i] = aircraft.state.engines[i].events.surgeBangCount;
          flameoutCounts[i] = aircraft.state.engines[i].events.flameoutCount;
        }
        return;
      }

      if (silent()) {
        // Every latch must still follow the state, or the first frame after an
        // unmute fires every event that happened while the sound was off. A
        // pilot who unmutes on the runway must not hear the landing they made
        // two minutes ago.
        for (let i = 0; i < aircraft.state.engines.length && i < 2; i++) {
          surgeCounts[i] = aircraft.state.engines[i].events.surgeBangCount;
          flameoutCounts[i] = aircraft.state.engines[i].events.flameoutCount;
        }
        const legs = aircraft.state.gear.legs;
        for (let i = 0; i < legs.length; i++) {
          legOnGround[i] = legs[i].onGround;
          legBurst[i] = legs[i].burst;
        }
        return;
      }

      // --- The engines ------------------------------------------------------
      for (let i = 0; i < aircraft.state.engines.length && i < engines.length; i++) {
        const events = aircraft.state.engines[i].events;
        // A count that only grows cannot be missed by a reader that runs
        // slower than the physics. See EngineEvents in engine.ts.
        if (events.surgeBangCount !== surgeCounts[i]) {
          surgeCounts[i] = events.surgeBangCount;
          engines[i].bang();
        }
        if (events.flameoutCount !== flameoutCounts[i]) {
          flameoutCounts[i] = events.flameoutCount;
          engines[i].flameout();
        }
        // The light off has no count, so it is read on the step that raised it.
        if (events.lightOff) engines[i].lightOff();
      }

      // --- The guns ---------------------------------------------------------
      const fired = armament.battery.roundsFired;
      for (let i = 0; i < fired; i++) guns.shot();

      // --- The shells that landed -------------------------------------------
      const speedOfSound = aircraft.atmosphere.speedOfSound;
      for (let i = 0; i < armament.impactCount; i++) {
        const impact = armament.impacts[i];
        guns.impact(impact.position.distanceTo(listenerNed), speedOfSound, impact.destroyed);
      }

      // --- The wheels -------------------------------------------------------
      const legs = aircraft.state.gear.legs;
      for (let i = 0; i < legs.length; i++) {
        const leg = legs[i];
        if (legOnGround[i] !== true && leg.onGround) {
          mechanical.touchdown(clamp(leg.compressionRate / TOUCHDOWN_RATE_FULL, 0, 1));
        }
        legOnGround[i] = leg.onGround;
        if (legBurst[i] !== true && leg.burst) mechanical.tireBurst();
        legBurst[i] = leg.burst;
      }
    },

    update(
      aircraft: Aircraft,
      camera: PerspectiveCamera,
      aircraftRenderPosition: Vector3,
      cockpit: boolean,
      dt: number,
    ): void {
      // The frame catches a spawn on its own. `respawn` of src/main.ts runs
      // INSIDE the frame, so the pose this call reads is already the new one.
      if (aircraft.time < frameTime) {
        previousGearPosition = -1;
        previousFlapPosition = -1;
        previousFlapRate = 0;
        // The aircraft went back to the threshold and the camera went with it.
        // That is a jump, and the delay line has to snap rather than sweep.
        previousDelay = -1;
      }
      frameTime = aircraft.time;
      if (silent()) {
        // Drop every running comparison instead of following it. The next
        // audible frame then primes them again from the state it finds. Without
        // this, a gear that travelled while the sound was off arrives as a lock
        // on the frame the pilot unmutes.
        previousGearPosition = -1;
        previousFlapPosition = -1;
        previousFlapRate = 0;
        previousDelay = -1;
        return;
      }

      // --- Where the listener stands ----------------------------------------
      // The camera pose of THIS frame, not the pose of the last one.
      camera.updateMatrixWorld();
      // The shell impacts arrive in the world frame, so the listener needs a
      // world position as well as a distance. src/render/frames.ts owns the
      // conversion, and CONVENTIONS section 3.3 says no other file may do it.
      threeToNed(camera.position, listenerNed);

      toAircraft.copy(aircraftRenderPosition);
      const distance = toAircraft.distanceTo(camera.position);
      // The same vector in the frame of the camera. Its x is how far to the
      // right the aircraft stands, so it is the pan.
      camera.worldToLocal(toAircraft);
      const length = toAircraft.length();
      const pan = length > 0.1 ? clamp(toAircraft.x / length, -1, 1) : 0;

      propagation(
        { distance, speedOfSound: aircraft.atmosphere.speedOfSound },
        propagationOut,
      );

      // --- The delay line ---------------------------------------------------
      const now = context.currentTime;
      // The bound is a RATE against the length of THIS frame. Read the comment
      // on DELAY_SNAP_RATE before you turn it back into a constant.
      const honest = DELAY_SNAP_RATE * Math.max(dt, 0) + DELAY_SNAP_FLOOR;
      const jumped =
        previousDelay < 0 || Math.abs(propagationOut.delay - previousDelay) > honest;
      if (jumped) {
        delay.delayTime.cancelScheduledValues(now);
        delay.delayTime.setValueAtTime(propagationOut.delay, now);
        if (started) {
          metrics.snaps++;
          duck();
        }
      } else {
        // A LINEAR ramp over exactly one frame. The rate of that ramp is the
        // radial speed over the speed of sound, so the pitch shift it produces
        // is the Doppler factor. Read section 2 of the module comment.
        delay.delayTime.linearRampToValueAtTime(propagationOut.delay, now + Math.max(dt, 0.001));
      }
      previousDelay = propagationOut.delay;
      started = true;

      ramp(absorption.frequency, propagationOut.cutoff);
      ramp(spread.gain, propagationOut.gain);
      ramp(panner.pan, pan);
      ramp(canopy.frequency, cockpit ? CANOPY_CUTOFF : 20_000);

      metrics.distance = distance;
      metrics.pan = pan;
      metrics.spread = spread.gain.value;
      metrics.cutoff = absorption.frequency.value;
      metrics.delay = delay.delayTime.value;
      metrics.busGain = duckGain.gain.value;
      metrics.windLevel = airframe.windLevel;
      metrics.frameDt = dt;

      // --- The engines ------------------------------------------------------
      const totals = aircraft.state.totals;
      for (let i = 0; i < aircraft.state.engines.length && i < engines.length; i++) {
        const engine = aircraft.state.engines[i];
        engineInput.rotorSpeed = engine.rotorSpeed;
        engineInput.thrust = engine.thrust;
        engineInput.fuelFlow = engine.fuelFlow;
        engineInput.trueAirspeed = totals.trueAirspeed;
        engineInput.lit = isLit(engine);
        engineInput.starterRunning = starterRunning(engine);
        engineInput.onFire = engine.state === 'fire';
        engines[i].update(engineInput);
      }

      // --- The airframe -----------------------------------------------------
      const systems = aircraft.state.systems.state;
      airframeInput.trueAirspeed = totals.trueAirspeed;
      airframeInput.alpha = totals.alpha;
      airframeInput.mach = totals.mach;
      airframeInput.gearPosition = systems.gearPosition;
      airframeInput.flapPosition = systems.flapPosition;
      airframe.update(airframeInput, cockpit);

      // --- The gear, the tires and the ground -------------------------------
      const legs = aircraft.state.gear.legs;
      let wheelSpeed = 0;
      let wheelLoad = 0;
      let slip = 0;
      for (const leg of legs) {
        wheelSpeed = Math.max(wheelSpeed, Math.abs(leg.wheelSpeed));
        wheelLoad += leg.load;
        slip = Math.max(slip, Math.abs(leg.slipRatio));
      }
      mechanicalInput.rolling.wheelSpeed = wheelSpeed;
      mechanicalInput.rolling.wheelLoad = wheelLoad;
      mechanicalInput.rolling.onGround = aircraft.state.gear.anyOnGround;
      mechanicalInput.rolling.slip = slip;

      let scrape = 0;
      for (const point of aircraft.state.contacts.points) {
        if (point.onGround) scrape += point.load * point.slideSpeed;
      }
      mechanicalInput.scrape = clamp(scrape / SCRAPE_REFERENCE, 0, 1);

      const step = Math.max(dt, 1e-4);
      if (previousGearPosition < 0) previousGearPosition = systems.gearPosition;
      if (previousFlapPosition < 0) previousFlapPosition = systems.flapPosition;
      const gearRate = Math.abs(systems.gearPosition - previousGearPosition) / step;
      const flapRate = Math.abs(systems.flapPosition - previousFlapPosition) / step;
      mechanicalInput.gearRate = gearRate;
      mechanicalInput.flapRate = flapRate;

      // A leg that arrives at either end of its travel drops onto its lock.
      if (
        gearRate > 0 &&
        (systems.gearPosition === 0 || systems.gearPosition === 1) &&
        previousGearPosition !== systems.gearPosition
      ) {
        mechanical.gearLock(systems.gearPosition === 1);
      }
      // The flap lever falls into a detent when the travel stops.
      if (previousFlapRate > 0 && flapRate === 0) mechanical.flapDetent();

      previousGearPosition = systems.gearPosition;
      previousFlapPosition = systems.flapPosition;
      previousFlapRate = flapRate;

      mechanical.update(mechanicalInput);
    },

    failure(): void {
      if (silent()) return;
      // Something structural let go. src/aircraft/me262/limits.ts raises this
      // for a bent wing, a panel that departed, a jammed aileron, an engine
      // fire and a burst tire, and every one of them is a bang followed by
      // metal that tears.
      playBurst(context, aircraftBus, {
        frequency: 130,
        q: 0.6,
        gain: 0.95,
        attack: 0.0006,
        decay: 0.5,
        color: 'pink',
      });
      playTone(context, aircraftBus, 620, 0.22, 0.8, 'sawtooth');
    },

    dispose(): void {
      for (const engine of engines) engine.dispose();
      airframe.dispose();
      mechanical.dispose();
      guns.dispose();
      aircraftBus.disconnect();
      delay.disconnect();
      absorption.disconnect();
      duckGain.disconnect();
      spread.disconnect();
      panner.disconnect();
      canopy.disconnect();
      bus.dispose();
    },
  };
}
