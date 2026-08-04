/**
 * Xbox controller reader.
 *
 * The module reads the Gamepad API, shapes the stick values, and finds the
 * button edges. It reports the hardware. It does not decide what a stick means.
 * The binding table decides that.
 *
 * Sign note. The reader keeps the Gamepad API sign of every axis. A stick that
 * a person pulls back reports a positive Y. A flight control needs the other
 * sign for pitch. The binding table applies that sign, so this module stays a
 * plain description of the hardware.
 *
 *
 * THE STANDARD MAPPING
 *
 * The standard mapping names 17 buttons and 4 axes.
 *
 *   buttons  0 a, 1 b, 2 x, 3 y, 4 left bumper, 5 right bumper,
 *            6 left trigger, 7 right trigger, 8 back, 9 start,
 *            10 left stick, 11 right stick,
 *            12 dpad up, 13 dpad down, 14 dpad left, 15 dpad right, 16 guide
 *   axes     0 left x, 1 left y, 2 right x, 3 right y
 *
 * The two triggers are buttons in this mapping. Each one carries an analog
 * value from 0 to 1 in `button.value`.
 *
 *
 * HOW THE MODULE DETECTS THE OTHER LAYOUT
 *
 * Some browsers and some drivers do not produce the standard mapping. Firefox
 * with an Xbox pad on Linux is the common case. Those layouts report the
 * triggers as axes with a range of -1 to 1, where -1 is released.
 *
 * The test is `pad.mapping === 'standard'`. The specification only allows that
 * string when the layout matches the table above, so it is the direct answer,
 * not a guess about the pad name. The module adds one guard: a pad that claims
 * the standard mapping but reports fewer than 8 buttons cannot hold the trigger
 * buttons, so the module treats it as the other layout.
 *
 * In the other layout the module reads the common Linux driver order:
 *
 *   axes  0 left x, 1 left y, 2 left trigger, 3 right x, 4 right y,
 *         5 right trigger
 *
 * A trigger there runs from -1 to 1. The module maps it to 0 to 1 before the
 * dead zone. Buttons keep the standard index in this layout, because the
 * trigger difference is the one that appears in practice.
 *
 *
 * SHAPING
 *
 * The dead zone is radial for a stick pair, not per axis. A per axis dead zone
 * cuts a square hole out of the stick range. On a diagonal the hole then lets a
 * value through on one axis while it holds the other axis at zero, and the
 * input feels as if it snaps to the axes.
 *
 * After the dead zone the module rescales the value, so the curve starts again
 * at zero and still reaches 1 at full deflection. Then it applies the curve:
 *
 *   output = sign(x) * (expo * |x|^3 + (1 - expo) * |x|)
 *
 * The curve is continuous, it passes through the origin, and it gives 1 at an
 * input of 1 for every value of expo.
 */

import { config } from '@/core/config';

export type AxisName = 'leftX' | 'leftY' | 'rightX' | 'rightY' | 'leftTrigger' | 'rightTrigger';

export type ButtonName =
  | 'a'
  | 'b'
  | 'x'
  | 'y'
  | 'leftBumper'
  | 'rightBumper'
  | 'back'
  | 'start'
  | 'leftStick'
  | 'rightStick'
  | 'dpadUp'
  | 'dpadDown'
  | 'dpadLeft'
  | 'dpadRight'
  | 'guide';

export interface GamepadState {
  connected: boolean;
  id: string;
  /** Shaped value. A stick reads -1 to 1. A trigger reads 0 to 1. */
  axis(name: AxisName): number;
  /** The value before the dead zone and the curve. */
  rawAxis(name: AxisName): number;
  held(name: ButtonName): boolean;
  /** True only on the frame the button went down. */
  pressed(name: ButtonName): boolean;
  /** True only on the frame the button came up. */
  released(name: ButtonName): boolean;
  /** Analog value, 0 to 1. A digital button reports 0 or 1. */
  value(name: ButtonName): number;
}

export interface GamepadReader extends GamepadState {
  /** Read the hardware once. Call it one time per frame, before the logic. */
  poll(): void;
  dispose(): void;
}

export interface GamepadOptions {
  /** Size below which a stick reads as zero, from 0 to 1. */
  deadZone?: number;
  /** Curve strength, from 0 to 1. Zero keeps the axis linear. */
  expo?: number;
}

/** Order of the axis names inside the value arrays. */
const AXIS_ORDER: readonly AxisName[] = [
  'leftX',
  'leftY',
  'rightX',
  'rightY',
  'leftTrigger',
  'rightTrigger',
];

const AXIS_INDEX: Readonly<Record<AxisName, number>> = {
  leftX: 0,
  leftY: 1,
  rightX: 2,
  rightY: 3,
  leftTrigger: 4,
  rightTrigger: 5,
};

/** Button index in the standard mapping. */
const BUTTON_INDEX: Readonly<Record<ButtonName, number>> = {
  a: 0,
  b: 1,
  x: 2,
  y: 3,
  leftBumper: 4,
  rightBumper: 5,
  back: 8,
  start: 9,
  leftStick: 10,
  rightStick: 11,
  dpadUp: 12,
  dpadDown: 13,
  dpadLeft: 14,
  dpadRight: 15,
  guide: 16,
};

const BUTTON_ORDER: readonly ButtonName[] = [
  'a',
  'b',
  'x',
  'y',
  'leftBumper',
  'rightBumper',
  'back',
  'start',
  'leftStick',
  'rightStick',
  'dpadUp',
  'dpadDown',
  'dpadLeft',
  'dpadRight',
  'guide',
];

/** Index inside the local arrays, which is dense and holds no gaps. */
const BUTTON_SLOT: Readonly<Record<ButtonName, number>> = Object.fromEntries(
  BUTTON_ORDER.map((name, slot) => [name, slot]),
) as Record<ButtonName, number>;

/** A button below this value counts as up. The value matches the browser rule. */
const BUTTON_THRESHOLD = 0.5;

/** Trigger index inside the axes array of the other layout. */
const LEGACY_LEFT_TRIGGER_AXIS = 2;
const LEGACY_RIGHT_TRIGGER_AXIS = 5;
const LEGACY_RIGHT_X_AXIS = 3;
const LEGACY_RIGHT_Y_AXIS = 4;
const LEGACY_AXIS_COUNT = 6;

/** Button count a pad needs before it can hold the two trigger buttons. */
const STANDARD_TRIGGER_BUTTON_COUNT = 8;

/** Apply the curve. See the module comment for the shape. */
function applyExpo(value: number, expo: number): number {
  const size = Math.abs(value);
  const shaped = expo * size * size * size + (1 - expo) * size;
  return value < 0 ? -shaped : shaped;
}

/** Clamp to a closed range. */
function clamp(value: number, low: number, high: number): number {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

/**
 * Build the reader. The reader starts with no pad. It finds a pad on the first
 * `poll` after the browser reports one.
 */
export function createGamepadReader(options?: GamepadOptions): GamepadReader {
  const deadZone = clamp(options?.deadZone ?? config.input.deadZone, 0, 0.9);
  const expo = clamp(options?.expo ?? config.input.expo, 0, 1);

  const raw = new Float64Array(AXIS_ORDER.length);
  const shaped = new Float64Array(AXIS_ORDER.length);
  const buttonValue = new Float64Array(BUTTON_ORDER.length);
  const heldNow = new Uint8Array(BUTTON_ORDER.length);
  const heldBefore = new Uint8Array(BUTTON_ORDER.length);

  /** Index the browser gave the pad. -1 means no pad. */
  let padIndex = -1;

  const onConnected = (event: GamepadEvent): void => {
    // Take the first pad and keep it. A second pad does not steal the seat.
    if (padIndex === -1) padIndex = event.gamepad.index;
  };

  const onDisconnected = (event: GamepadEvent): void => {
    if (event.gamepad.index === padIndex) padIndex = -1;
  };

  if (typeof window !== 'undefined') {
    window.addEventListener('gamepadconnected', onConnected);
    window.addEventListener('gamepaddisconnected', onDisconnected);
  }

  /** Find the live pad. The browser hands out a fresh snapshot every poll. */
  function findPad(): Gamepad | null {
    if (typeof navigator === 'undefined' || typeof navigator.getGamepads !== 'function') {
      return null;
    }
    const pads = navigator.getGamepads();

    if (padIndex >= 0) {
      const pad = pads[padIndex] ?? null;
      if (pad !== null && pad.connected) return pad;
      padIndex = -1;
    }

    // The connect event does not fire when the page loads with a pad already
    // held. Scan once, so the pad still appears after a page reload.
    for (const pad of pads) {
      if (pad != null && pad.connected) {
        padIndex = pad.index;
        return pad;
      }
    }
    return null;
  }

  /** Read one raw axis, or 0 when the pad does not hold that index. */
  function readAxis(pad: Gamepad, index: number): number {
    return index < pad.axes.length ? pad.axes[index] : 0;
  }

  /** Read one raw button value, or 0 when the pad does not hold that index. */
  function readButton(pad: Gamepad, index: number): number {
    return index < pad.buttons.length ? pad.buttons[index].value : 0;
  }

  /** Shape a stick pair with a radial dead zone, then the curve. */
  function shapeStick(xSlot: number, ySlot: number): void {
    const x = raw[xSlot];
    const y = raw[ySlot];
    const size = Math.hypot(x, y);

    if (size <= deadZone || size === 0) {
      shaped[xSlot] = 0;
      shaped[ySlot] = 0;
      return;
    }

    // Rescale the whole pair, so the direction survives and the edge of the
    // dead zone maps to zero. Clamp, because a worn stick reports past 1.
    const rescaled = Math.min((size - deadZone) / (1 - deadZone), 1);
    const gain = rescaled / size;
    shaped[xSlot] = applyExpo(clamp(x * gain, -1, 1), expo);
    shaped[ySlot] = applyExpo(clamp(y * gain, -1, 1), expo);
  }

  /** Shape a trigger. A trigger is one dimension, so the dead zone is scalar. */
  function shapeTrigger(slot: number): void {
    const value = clamp(raw[slot], 0, 1);
    if (value <= deadZone) {
      shaped[slot] = 0;
      return;
    }
    shaped[slot] = applyExpo((value - deadZone) / (1 - deadZone), expo);
  }

  function clearAll(): void {
    raw.fill(0);
    shaped.fill(0);
    buttonValue.fill(0);
    heldNow.fill(0);
  }

  const api: GamepadReader = {
    connected: false,
    id: '',

    poll(): void {
      heldBefore.set(heldNow);

      const pad = findPad();
      if (pad === null) {
        api.connected = false;
        api.id = '';
        clearAll();
        return;
      }

      api.connected = true;
      api.id = pad.id;

      // See the module comment. The mapping string is the direct test. The
      // button count guards a pad that claims the standard mapping and then
      // does not hold the two trigger buttons.
      const standard =
        pad.mapping === 'standard' && pad.buttons.length >= STANDARD_TRIGGER_BUTTON_COUNT;

      raw[AXIS_INDEX.leftX] = readAxis(pad, 0);
      raw[AXIS_INDEX.leftY] = readAxis(pad, 1);

      if (standard) {
        raw[AXIS_INDEX.rightX] = readAxis(pad, 2);
        raw[AXIS_INDEX.rightY] = readAxis(pad, 3);
        raw[AXIS_INDEX.leftTrigger] = readButton(pad, 6);
        raw[AXIS_INDEX.rightTrigger] = readButton(pad, 7);
      } else if (pad.axes.length >= LEGACY_AXIS_COUNT) {
        raw[AXIS_INDEX.rightX] = readAxis(pad, LEGACY_RIGHT_X_AXIS);
        raw[AXIS_INDEX.rightY] = readAxis(pad, LEGACY_RIGHT_Y_AXIS);
        // A released trigger reads -1 in this layout. Move it to 0 to 1.
        raw[AXIS_INDEX.leftTrigger] = (readAxis(pad, LEGACY_LEFT_TRIGGER_AXIS) + 1) / 2;
        raw[AXIS_INDEX.rightTrigger] = (readAxis(pad, LEGACY_RIGHT_TRIGGER_AXIS) + 1) / 2;
      } else {
        // A short pad holds no triggers. Read the right stick where it usually
        // sits and leave the triggers at zero.
        raw[AXIS_INDEX.rightX] = readAxis(pad, 2);
        raw[AXIS_INDEX.rightY] = readAxis(pad, 3);
        raw[AXIS_INDEX.leftTrigger] = 0;
        raw[AXIS_INDEX.rightTrigger] = 0;
      }

      shapeStick(AXIS_INDEX.leftX, AXIS_INDEX.leftY);
      shapeStick(AXIS_INDEX.rightX, AXIS_INDEX.rightY);
      shapeTrigger(AXIS_INDEX.leftTrigger);
      shapeTrigger(AXIS_INDEX.rightTrigger);

      for (const name of BUTTON_ORDER) {
        const slot = BUTTON_SLOT[name];
        const value = readButton(pad, BUTTON_INDEX[name]);
        buttonValue[slot] = value;
        heldNow[slot] = value >= BUTTON_THRESHOLD ? 1 : 0;
      }
    },

    axis(name: AxisName): number {
      return shaped[AXIS_INDEX[name]];
    },

    rawAxis(name: AxisName): number {
      return raw[AXIS_INDEX[name]];
    },

    held(name: ButtonName): boolean {
      return heldNow[BUTTON_SLOT[name]] === 1;
    },

    pressed(name: ButtonName): boolean {
      const slot = BUTTON_SLOT[name];
      return heldNow[slot] === 1 && heldBefore[slot] === 0;
    },

    released(name: ButtonName): boolean {
      const slot = BUTTON_SLOT[name];
      return heldNow[slot] === 0 && heldBefore[slot] === 1;
    },

    value(name: ButtonName): number {
      return buttonValue[BUTTON_SLOT[name]];
    },

    dispose(): void {
      if (typeof window !== 'undefined') {
        window.removeEventListener('gamepadconnected', onConnected);
        window.removeEventListener('gamepaddisconnected', onDisconnected);
      }
      padIndex = -1;
      api.connected = false;
      api.id = '';
      clearAll();
      heldBefore.fill(0);
    },
  };

  return api;
}

/** The axis names in panel order. src/ui/input-debug.ts reads this list. */
export const AXIS_NAMES: readonly AxisName[] = AXIS_ORDER;

/** The button names in panel order. src/ui/input-debug.ts reads this list. */
export const BUTTON_NAMES: readonly ButtonName[] = BUTTON_ORDER;

/** True when the axis holds a trigger, which runs from 0 to 1, not -1 to 1. */
export function isTriggerAxis(name: AxisName): boolean {
  return name === 'leftTrigger' || name === 'rightTrigger';
}
