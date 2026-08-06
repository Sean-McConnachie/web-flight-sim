/**
 * The audio context, the master bus and the autoplay unlock.
 *
 * BEAD o7r.
 *
 *
 * 1. WHY THE SOUND CAN NOT SIMPLY START
 *
 * A browser starts every `AudioContext` in the `suspended` state and it only
 * lets the page resume one from inside a handler for a real user action. This
 * is a deliberate rule and there is no way around it. A page that makes a noise
 * at a person who did not ask for one is the thing the rule exists to stop.
 *
 * The simulator therefore builds the whole graph at load, leaves it suspended,
 * and resumes it on the FIRST key press, pointer press or touch. A pilot who
 * flies has already pressed a key, so the sound starts on its own and nobody
 * reads an instruction. A visitor who only watches hears nothing until they
 * touch the page, which is the correct answer.
 *
 * `blocked` reports the state to the SOUND button of src/ui/sound-button.ts, so
 * a pilot who wonders where the sound went can read it.
 *
 *
 * 2. THE MASTER BUS
 *
 *   every voice -> master gain -> compressor -> the speakers
 *
 * The compressor is not there for an effect. Four MK 108 firing over two
 * engines at full power against a stall buffet will sum past one, and a sum
 * past one clips. A clip is the harshest sound a computer can make. The
 * compressor holds the peaks down and gives the guns their punch back at the
 * same time.
 *
 * The master gain carries the volume AND the mute, because a mute is a volume
 * of zero. It ramps rather than jumps, since a gain that steps makes a click.
 *
 *
 * 3. WHAT THIS FILE DOES NOT DO
 *
 * It holds no arithmetic. Every law of the sound sits in src/audio/voices.ts,
 * which names no browser API so that the tests can measure it.
 */

/** How long the master gain takes to reach a new volume, s. A step clicks. */
const VOLUME_RAMP = 0.05;

/** Where the compressor starts to hold the peaks down, dB. */
const COMPRESSOR_THRESHOLD = -16;

/** How hard it holds them. 6 to 1 is firm without sounding pumped. */
const COMPRESSOR_RATIO = 6;

/** Where the volume and the mute are kept between visits. */
const STORAGE_KEY = 'hfs-sound';

/** Volume a first visit starts at. */
export const DEFAULT_VOLUME = 0.7;

/** What the page remembers about the sound. */
interface StoredSettings {
  volume: number;
  muted: boolean;
}

/**
 * Reads the stored settings, or the defaults.
 *
 * Local storage throws in a browser that has it turned off and in some private
 * windows. A sound setting is not worth a fatal error, so every path here
 * falls back to the default and says nothing.
 */
function readSettings(): StoredSettings {
  const fallback: StoredSettings = { volume: DEFAULT_VOLUME, muted: false };
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (raw === null) return fallback;
    const parsed: unknown = JSON.parse(raw);
    if (typeof parsed !== 'object' || parsed === null) return fallback;
    const record = parsed as Record<string, unknown>;
    const volume = record.volume;
    const muted = record.muted;
    return {
      volume: typeof volume === 'number' && volume >= 0 && volume <= 1 ? volume : DEFAULT_VOLUME,
      muted: muted === true,
    };
  } catch {
    return fallback;
  }
}

function writeSettings(settings: StoredSettings): void {
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
  } catch {
    // A browser that refuses to store a volume still plays the sound.
  }
}

/** The master bus. Every voice of src/audio connects to `destination`. */
export interface AudioBus {
  readonly context: AudioContext;
  /** What a voice connects to. It is never the raw speaker output. */
  readonly destination: AudioNode;
  /** True while the browser has not yet let the context run. */
  readonly blocked: boolean;
  /** 0 to 1. It survives a reload. */
  volume: number;
  /** A mute is a volume of zero that remembers the volume. */
  muted: boolean;
  /** Adds a listener for a change of `blocked`, `volume` or `muted`. */
  onChange(fn: () => void): () => void;
  /**
   * Asks the browser to start the context.
   *
   * It only works inside a handler for a real user action. The bus calls it on
   * the first key and the first touch on its own, and the SOUND button calls it
   * again, because a click on that button is the clearest request there is.
   */
  unlock(): void;
  dispose(): void;
}

/** True when this browser can make a sound at all. */
export function audioAvailable(): boolean {
  return typeof window !== 'undefined' && typeof window.AudioContext === 'function';
}

/**
 * Builds the master bus, or returns null when the browser has no Web Audio.
 *
 * A null return is not a fault. src/main.ts leaves the sound out and the
 * simulator flies in silence, exactly as it did before this bead.
 */
export function createAudioBus(): AudioBus | null {
  if (!audioAvailable()) return null;

  const settings = readSettings();
  // `interactive` asks for the shortest buffer the machine can hold. The
  // alternative, `playback`, adds latency to save power, and a gun that fires a
  // tenth of a second after the trigger reads as a fault.
  const context = new AudioContext({ latencyHint: 'interactive' });

  const master = context.createGain();
  master.gain.value = settings.muted ? 0 : settings.volume;

  const compressor = context.createDynamicsCompressor();
  compressor.threshold.value = COMPRESSOR_THRESHOLD;
  compressor.knee.value = 12;
  compressor.ratio.value = COMPRESSOR_RATIO;
  // A fast attack catches the front of a gun report. A slow release stops the
  // engine level from breathing up and down after every burst.
  compressor.attack.value = 0.003;
  compressor.release.value = 0.25;

  master.connect(compressor);
  compressor.connect(context.destination);

  const listeners: (() => void)[] = [];
  function announce(): void {
    for (const fn of listeners.slice()) fn();
  }

  let volume = settings.volume;
  let muted = settings.muted;
  let disposed = false;

  function applyGain(): void {
    const target = muted ? 0 : volume;
    const now = context.currentTime;
    // cancelAndHoldAtTime keeps the value the ramp had reached, so a second
    // change part way through the first one does not jump back to the start.
    master.gain.cancelScheduledValues(now);
    master.gain.setValueAtTime(master.gain.value, now);
    master.gain.linearRampToValueAtTime(target, now + VOLUME_RAMP);
  }

  function save(): void {
    writeSettings({ volume, muted });
  }

  // --- The unlock ---------------------------------------------------------
  // The listeners sit on the window in the CAPTURE phase, so a press that a
  // control of the page handles and stops still reaches them.
  const UNLOCK_EVENTS: readonly string[] = ['pointerdown', 'touchstart', 'keydown'];

  function removeUnlockListeners(): void {
    for (const name of UNLOCK_EVENTS) {
      window.removeEventListener(name, unlock, true);
    }
  }

  function unlock(): void {
    if (disposed) return;
    if (context.state === 'running') {
      removeUnlockListeners();
      return;
    }
    void context
      .resume()
      .then(() => {
        removeUnlockListeners();
        announce();
      })
      .catch(() => {
        // The browser refused. Another press may still work, so the listeners
        // stay where they are.
      });
  }

  for (const name of UNLOCK_EVENTS) {
    window.addEventListener(name, unlock, true);
  }
  // A page that a user reached by a link may already hold a gesture, and a
  // context that is already running must not wait for a second one.
  if (context.state === 'suspended') unlock();

  // The browser suspends a context on its own when the tab goes to the
  // background, and it reports that here.
  context.addEventListener('statechange', announce);

  const bus: AudioBus = {
    context,
    destination: master,
    get blocked(): boolean {
      return context.state !== 'running';
    },
    get volume(): number {
      return volume;
    },
    set volume(value: number) {
      volume = Math.min(Math.max(value, 0), 1);
      applyGain();
      save();
      announce();
    },
    get muted(): boolean {
      return muted;
    },
    set muted(value: boolean) {
      muted = value;
      applyGain();
      save();
      announce();
    },
    onChange(fn: () => void): () => void {
      listeners.push(fn);
      return () => {
        const index = listeners.indexOf(fn);
        if (index >= 0) listeners.splice(index, 1);
      };
    },
    unlock,
    dispose(): void {
      if (disposed) return;
      disposed = true;
      removeUnlockListeners();
      context.removeEventListener('statechange', announce);
      listeners.length = 0;
      void context.close().catch(() => {
        // A context that is already closed throws. Nothing to do about it.
      });
    },
  };

  return bus;
}
