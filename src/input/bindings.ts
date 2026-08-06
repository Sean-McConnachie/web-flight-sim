/**
 * Control bindings.
 *
 * This module is the one place that maps hardware to a named action. No other
 * file reads a raw button or a raw axis. A module that wants the pitch command
 * reads `ControlInput.pitch`. It never asks which stick moved.
 *
 *
 * THE SIGN OF THE PITCH AXIS
 *
 * Read this part twice. A wrong sign here makes the aircraft unflyable.
 *
 * The Gamepad API reports a positive left stick Y when the pilot pulls the
 * stick back. src/input/gamepad.ts keeps that sign, because that module
 * describes the hardware and nothing more. This module owns the flight
 * meaning: `ControlInput.pitch` is positive when the nose goes UP. A pilot who
 * pulls the stick back wants the nose up, so the stick value passes through
 * with the sign it has. That is the flight convention, which is the opposite of
 * the screen convention where a stick that moves up is a positive value.
 * PITCH_STICK_SCALE below holds the sign, and the unit test asserts it.
 *
 *
 * THE BINDING TABLE
 *
 * DEFAULT_BINDINGS holds the whole map. The poll function reads that table. It
 * holds no hardware name of its own. A later remap screen can read the table,
 * write a new one, and pass it to `createInputSystem`. src/ui/controls-menu.ts
 * already reads the table and prints it, so the list a pilot sees can never
 * disagree with the list the code runs.
 *
 * Rules the table follows:
 *
 * - More than one binding can carry the same action. The system adds the value
 *   of each one and then clamps. The rudder uses this: the right trigger adds
 *   and the left trigger subtracts.
 * - `keys` holds one code, or a pair in the order [negative, positive].
 * - `touch` names one control of the on screen pad. src/input/touch.ts draws
 *   that pad. It is a third device and the table maps it the same way it maps
 *   the other two.
 * - A key code can carry a `Shift+` prefix. `Binding` has no modifier field, so
 *   the prefix carries the modifier inside the code string. When a code has a
 *   shifted binding, the unshifted binding of that same code stays off while
 *   shift is down. That keeps `F` for flaps down and `Shift+F` for flaps up
 *   apart, and it leaves every other key free to work while shift is down.
 * - `kind` says how to read the source. It does not say whether the action is
 *   an edge or a hold. That is a property of the action, not of the key, so
 *   EDGE_ACTIONS holds it.
 *
 *
 * CONTROL AUTHORITY
 *
 * The Me 262 has no power controls. The stick force of a surface follows the
 * hinge moment, which follows the dynamic pressure, so the deflection a pilot
 * can hold falls as the aircraft goes faster. A stick and a key both report a
 * full command whatever the speed, so without a scaling a keyboard pilot puts
 * FULL surface deflection on at 900 km/h and breaks the wing. `controlAuthority`
 * holds the law and the poll applies it to the three flight axes.
 *
 * This is an INPUT limit and not a flight model limit. It says how far the pilot
 * can move the stick, not what the surface does when it gets there. The flight
 * model, the trim solver and the flight tests all command the surface directly
 * and none of them reads this file, so none of them changes.
 */

import { config } from '@/core/config';
import { clamp } from '@/math/tables';
import type { AxisName, ButtonName, GamepadReader } from './gamepad';
import { AXIS_NAMES, BUTTON_NAMES, createGamepadReader } from './gamepad';
import type { KeyboardReader } from './keyboard';
import { createKeyboardReader, SHIFT_CODES } from './keyboard';
import type { TouchAxisName, TouchButtonName, TouchReader } from './touch';
import { TOUCH_AXIS_NAMES, TOUCH_BUTTON_NAMES, createNullTouchReader } from './touch';

export interface ControlInput {
  // Axes, all -1..1 unless stated
  roll: number;        // positive rolls right
  pitch: number;       // positive pitches the nose UP
  yaw: number;         // positive yaws the nose right
  throttle: number;    // 0..1, both engines
  brakeLeft: number;   // 0..1
  brakeRight: number;  // 0..1
  lookYaw: number;     // head or camera look
  lookPitch: number;
  // Edge actions, true for one frame
  toggleGear: boolean;
  toggleFlapsUp: boolean;
  toggleFlapsDown: boolean;
  cycleView: boolean;
  startEngines: boolean;   // held, not an edge
  fireCannon: boolean;     // held
  toggleMenu: boolean;
  toggleDebug: boolean;
  trimUp: boolean;         // held
  trimDown: boolean;       // held
  toggleHud: boolean;      // hides or shows every overlay panel
  respawn: boolean;        // puts the aircraft back on the runway threshold
  toggleFreeCamera: boolean;
  toggleSound: boolean;    // mutes the sound, or brings it back
}

/** Which device the pilot last used. The prompts and the pad follow it. */
export type ActiveDevice = 'gamepad' | 'keyboard' | 'touch';

export interface InputSystem {
  readonly state: ControlInput;
  poll(dt: number): void;
  readonly activeDevice: ActiveDevice;
  dispose(): void;
}

export interface Binding {
  action: keyof ControlInput;
  kind: 'axis' | 'button' | 'rate';
  gamepad?: string;
  keys?: [string, string] | [string];
  /** One control of the on screen pad. See src/input/touch.ts. */
  touch?: string;
  scale?: number;
}

export interface InputSystemOptions {
  /** Gamepad reader. The system builds the real one when this is absent. */
  gamepad?: GamepadReader;
  /** Keyboard reader. The system builds the real one when this is absent. */
  keyboard?: KeyboardReader;
  /**
   * On screen pad. The system builds a reader that reports nothing when this is
   * absent, so a desktop and a test both run the same code path at no cost.
   */
  touch?: TouchReader;
  /** The map to use. DEFAULT_BINDINGS when this is absent. */
  bindings?: readonly Binding[];
  /**
   * Reports true while the wheels touch the ground. The differential brake
   * rule reads it. The default reports false, so the triggers only steer the
   * rudder until the aircraft module supplies the real answer.
   */
  groundContact?: () => boolean;
  /**
   * Reports the dynamic pressure of the free stream, Pa. `controlAuthority`
   * reads it. The default reports zero, which is full authority, so a test that
   * checks a binding needs no aircraft.
   */
  dynamicPressure?: () => number;
}

/** Action names that hold a number. */
type NumberAction = {
  [K in keyof ControlInput]: ControlInput[K] extends number ? K : never;
}[keyof ControlInput];

/** Action names that hold a boolean. */
type BooleanAction = Exclude<keyof ControlInput, NumberAction>;

const NUMBER_ACTIONS = [
  'roll',
  'pitch',
  'yaw',
  'throttle',
  'brakeLeft',
  'brakeRight',
  'lookYaw',
  'lookPitch',
] as const satisfies readonly NumberAction[];

const BOOLEAN_ACTIONS = [
  'toggleGear',
  'toggleFlapsUp',
  'toggleFlapsDown',
  'cycleView',
  'startEngines',
  'fireCannon',
  'toggleMenu',
  'toggleDebug',
  'trimUp',
  'trimDown',
  'toggleHud',
  'respawn',
  'toggleFreeCamera',
  'toggleSound',
] as const satisfies readonly BooleanAction[];

/**
 * Actions that fire for one poll. Every other boolean action stays true while
 * the pilot holds the control. `Binding` cannot carry this, because the choice
 * belongs to the action and not to the key that drives it.
 */
export const EDGE_ACTIONS: ReadonlySet<keyof ControlInput> = new Set<keyof ControlInput>([
  'toggleGear',
  'toggleFlapsUp',
  'toggleFlapsDown',
  'cycleView',
  'toggleMenu',
  'toggleDebug',
  'toggleHud',
  'respawn',
  'toggleFreeCamera',
  'toggleSound',
]);

/**
 * Sign that carries the left stick Y to the pitch command. See the module
 * comment. A positive stick Y means the pilot pulled the stick back, and a
 * positive pitch means the nose goes up, so the sign stays.
 */
const PITCH_STICK_SCALE = 1;

/**
 * Time for the throttle to move from closed to full, in seconds.
 *
 * A D-pad and a key are on or off. They carry no position. The system turns
 * them into a rate and integrates that rate, so the lever moves at a human
 * speed.
 *
 * The value lives in src/core/config.ts, which holds every tuned number. Read
 * the comment there for why this rate limit is an input device concern and not
 * the engine spool model.
 */
const THROTTLE_SWEEP_TIME = config.input.throttleSweepTime;

/** Throttle change per second while the pilot holds the control. */
const THROTTLE_RATE = 1 / THROTTLE_SWEEP_TIME;

/**
 * Throttle at or below this value counts as idle for the taxi brake rule.
 */
const TAXI_IDLE_THROTTLE = 0.05;

/** An axis above this size counts as device activity. It clears stick noise. */
const DEVICE_ACTIVITY_THRESHOLD = 0.25;

/** A gamepad axis bound to a button action counts as down above this value. */
const AXIS_AS_BUTTON_THRESHOLD = 0.5;

/** Prefix that carries the shift modifier inside a key code. */
const SHIFT_PREFIX = 'Shift+';

/**
 * Dynamic pressure up to which the pilot reaches the full travel of a control,
 * Pa.
 *
 * 10 kPa is 456 km/h of equivalent airspeed. Below it the pilot has every
 * degree of every surface, so the takeoff, the approach, the stall and the
 * recovery from a stall all keep the control they had. The flight test harness
 * schedules its own pitch gains on the same 10 kPa, for the same reason: an
 * elevator moment follows the dynamic pressure.
 * Confidence: estimate. It is the stick force a pilot can hold, in the units
 * this file can measure it in.
 */
export const FULL_AUTHORITY_PRESSURE = 10000; // Pa

/**
 * Smallest part of the travel the pilot keeps, whatever the speed.
 *
 * The law below would run to zero in a dive and leave the pilot with no way to
 * recover. The floor is reached at 67 kPa, which is 1180 km/h of equivalent
 * airspeed, so no state the aircraft can reach meets it. It exists so that a
 * fault in the caller can never take the controls away.
 */
export const MIN_CONTROL_AUTHORITY = 0.15;

/**
 * Part of the full surface travel a full stick reaches, at one dynamic
 * pressure.
 *
 * BEAD b56, item 4. The hinge moment of a surface is the dynamic pressure times
 * the area times the chord times a coefficient that follows the deflection, and
 * the stick force is that moment through a fixed gearing. A pilot has a fixed
 * maximum pull, so the deflection the pilot can hold falls as ONE OVER the
 * dynamic pressure. That is the law below.
 *
 * WHAT IT DOES AND WHAT IT DOES NOT DO. A snatch to full stick at 3000 m and
 * Mach 0.75, held for three seconds, was measured against the flight model:
 *
 *   after 0.25 s   12.33 g before the scaling, 8.87 g with it
 *   peak           13.74 g before the scaling, 13.02 g with it
 *
 * The scaling therefore removes the STEP that a key press used to make, which
 * is what bead b56 reports. It does not cap the load factor of a pilot who
 * HOLDS the stick back. It cannot: above about 600 km/h the elevator of this
 * airframe drives the wing to its stall angle at a few degrees of deflection,
 * so no bound on the deflection alone keeps the aircraft inside the envelope.
 * The aircraft of 1944 behaved the same way, which is why its handbook
 * placards the pilot away from high speed dives and away from acrobatics
 * instead of giving a limit to fly to. The pilot now reads the warning that
 * src/ui/hud.ts prints and the failure that src/main.ts shows.
 */
export function controlAuthority(dynamicPressure: number): number {
  if (!(dynamicPressure > FULL_AUTHORITY_PRESSURE)) return 1;
  return clamp(FULL_AUTHORITY_PRESSURE / dynamicPressure, MIN_CONTROL_AUTHORITY, 1);
}

/**
 * The default map.
 *
 * Gamepad, standard mapping:
 *
 *   left stick X      roll
 *   left stick Y      pitch, stick back gives nose up
 *   left trigger      left rudder
 *   right trigger     right rudder, and the two brakes while the aircraft taxis
 *   right stick       look
 *   D-pad up, down    throttle, as a rate
 *   D-pad left, right flaps up, flaps down
 *   A                 gear
 *   B                 both wheel brakes, held
 *   Y                 view
 *   left bumper       engine start, held
 *   right bumper      cannon, held
 *   start             menu
 *
 * The flaps sit on the D-pad and not on X. A flap lever moves two ways. A pair
 * of D-pad keys shows that shape, and a single button with a modifier does not.
 * X stays free for a later action.
 *
 * Keyboard, the part that is not obvious:
 *
 *   Q, E              rudder, left and right
 *   Z, C              LEFT wheel brake and RIGHT wheel brake, one at a time
 *   B                 both wheel brakes
 *
 * BEAD b56. Z and C sit under Q and E, one key column each, so the brake of a
 * side is under the same finger as the rudder of that side. This aircraft turns
 * on the ground with the nose wheel AND with the differential brake, and B
 * drives both wheels together, so a keyboard pilot had no way to brake one side
 * before these two keys. The keys carry no taxi gate, unlike the gamepad
 * triggers below, because a key that does nothing else cannot fight the rudder.
 * A pilot can therefore hold the aircraft straight with them after touchdown.
 *
 * The on screen pad, drawn by src/input/touch.ts:
 *
 *   stick            roll and pitch, down the screen is nose up
 *   rudder           the bar over the stick
 *   THR + and THR -  throttle, as a rate
 *   the button block gear, both flap steps, both brakes, view, cannon, engine
 *                    start and respawn
 *   the top bar      the overlay panels, and the pad itself
 *
 * The pad has no control for the MENU. src/ui/controls-menu.ts draws its own
 * button, and that button stands on every device, so a second one here would
 * be the same action in two places on a phone and in one place on a desktop.
 *
 * The pad carries no look control, no debug level and no free camera. A phone
 * screen holds the controls that fly the aircraft and no more.
 */
export const DEFAULT_BINDINGS: readonly Binding[] = [
  // Stick axes.
  {
    action: 'roll',
    kind: 'axis',
    gamepad: 'leftX',
    keys: ['KeyA', 'KeyD'],
    touch: 'stickX',
    scale: 1,
  },
  {
    action: 'pitch',
    kind: 'axis',
    gamepad: 'leftY',
    keys: ['KeyW', 'KeyS'],
    touch: 'stickY',
    scale: PITCH_STICK_SCALE,
  },

  // Rudder. The two triggers make one axis: right minus left.
  { action: 'yaw', kind: 'axis', gamepad: 'rightTrigger', scale: 1 },
  { action: 'yaw', kind: 'axis', gamepad: 'leftTrigger', scale: -1 },
  { action: 'yaw', kind: 'axis', keys: ['KeyQ', 'KeyE'], touch: 'rudder', scale: 1 },

  // Look. A stick that moves up looks up, which is the screen convention. The
  // head is not a flight control, so it does not follow the pitch convention.
  { action: 'lookYaw', kind: 'axis', gamepad: 'rightX', scale: 1 },
  { action: 'lookPitch', kind: 'axis', gamepad: 'rightY', scale: -1 },

  // Throttle. Every one of these is a rate. See THROTTLE_SWEEP_TIME.
  { action: 'throttle', kind: 'rate', gamepad: 'dpadUp', touch: 'throttleUp', scale: 1 },
  { action: 'throttle', kind: 'rate', gamepad: 'dpadDown', touch: 'throttleDown', scale: -1 },
  { action: 'throttle', kind: 'rate', keys: ['PageDown', 'PageUp'], scale: 1 },
  { action: 'throttle', kind: 'rate', keys: ['Shift+Minus', 'Shift+Equal'], scale: 1 },

  // Wheel brakes. An axis binding is a differential brake and the taxi rule
  // gates it. A button binding is the full brake and nothing gates it.
  { action: 'brakeLeft', kind: 'axis', gamepad: 'leftTrigger', scale: 1 },
  { action: 'brakeRight', kind: 'axis', gamepad: 'rightTrigger', scale: 1 },
  { action: 'brakeLeft', kind: 'button', gamepad: 'b', keys: ['KeyB'], touch: 'brake', scale: 1 },
  { action: 'brakeRight', kind: 'button', gamepad: 'b', keys: ['KeyB'], touch: 'brake', scale: 1 },
  // One key per side. See the keyboard part of the comment above the table.
  { action: 'brakeLeft', kind: 'button', keys: ['KeyZ'], scale: 1 },
  { action: 'brakeRight', kind: 'button', keys: ['KeyC'], scale: 1 },

  // Buttons.
  { action: 'toggleGear', kind: 'button', gamepad: 'a', keys: ['KeyG'], touch: 'gear' },
  {
    action: 'toggleFlapsUp',
    kind: 'button',
    gamepad: 'dpadLeft',
    keys: ['Shift+KeyF'],
    touch: 'flapsUp',
  },
  {
    action: 'toggleFlapsDown',
    kind: 'button',
    gamepad: 'dpadRight',
    keys: ['KeyF'],
    touch: 'flapsDown',
  },
  { action: 'cycleView', kind: 'button', gamepad: 'y', keys: ['KeyV'], touch: 'view' },
  {
    action: 'startEngines',
    kind: 'button',
    gamepad: 'leftBumper',
    keys: ['Home'],
    touch: 'engineStart',
  },
  { action: 'fireCannon', kind: 'button', gamepad: 'rightBumper', keys: ['Space'], touch: 'fire' },

  // The controls menu sits on H, for HELP.
  //
  // It held F1 before, and F1 was the wrong key. A browser answers F1 with its
  // OWN help window, and nothing on this page can stop it: the keyboard reader
  // of src/input/keyboard.ts reads the event and never cancels it. A pilot who
  // pressed F1 therefore got the help of the browser over the top of the help
  // of the simulator.
  //
  // H is a letter key, so it needs no modifier and no function row. A phone
  // keyboard and a small laptop both have it. src/ui/controls-menu.ts also
  // draws a button, because a key nobody can see is a key nobody presses.
  //
  // Escape stays as the second key. It is the key a game player reaches for.
  { action: 'toggleMenu', kind: 'button', gamepad: 'start', keys: ['KeyH'] },
  { action: 'toggleMenu', kind: 'button', keys: ['Escape'] },

  { action: 'toggleDebug', kind: 'button', keys: ['F3'] },
  { action: 'trimUp', kind: 'button', keys: ['BracketRight'] },
  { action: 'trimDown', kind: 'button', keys: ['BracketLeft'] },

  // The three actions below moved out of src/main.ts and into this table. They
  // used to sit on a separate key listener, so the controls menu could not find
  // them and no pad could reach them. Every pilot control now has one home.
  //
  // The panel switch held H until the menu took that key. It moved to U, for
  // the USER INTERFACE it hides. U carries no other action, it needs no
  // modifier, and it sits under the same hand as the panels it clears.
  { action: 'toggleHud', kind: 'button', keys: ['KeyU'], touch: 'panels' },
  { action: 'respawn', kind: 'button', keys: ['KeyR'], touch: 'respawn' },
  { action: 'toggleFreeCamera', kind: 'button', keys: ['F2'] },

  // BEAD kz2. M is the key every media player in the world mutes on, so it is
  // the first key a person tries. src/ui/sound-button.ts draws a button for it
  // as well, because a phone has no M key and because a browser will not let
  // the sound start until somebody presses something.
  { action: 'toggleSound', kind: 'button', keys: ['KeyM'] },
];

const AXIS_NAME_SET: ReadonlySet<string> = new Set<string>(AXIS_NAMES);
const BUTTON_NAME_SET: ReadonlySet<string> = new Set<string>(BUTTON_NAMES);
const TOUCH_AXIS_SET: ReadonlySet<string> = new Set<string>(TOUCH_AXIS_NAMES);
const TOUCH_BUTTON_SET: ReadonlySet<string> = new Set<string>(TOUCH_BUTTON_NAMES);

function isAxisName(name: string): name is AxisName {
  return AXIS_NAME_SET.has(name);
}

function isButtonName(name: string): name is ButtonName {
  return BUTTON_NAME_SET.has(name);
}

function isTouchAxisName(name: string): name is TouchAxisName {
  return TOUCH_AXIS_SET.has(name);
}

function isTouchButtonName(name: string): name is TouchButtonName {
  return TOUCH_BUTTON_SET.has(name);
}

const NUMBER_ACTION_SET: ReadonlySet<keyof ControlInput> = new Set<keyof ControlInput>(
  NUMBER_ACTIONS,
);

function isNumberAction(action: keyof ControlInput): action is NumberAction {
  return NUMBER_ACTION_SET.has(action);
}

/** One key of a binding, with the modifier already read. */
interface CompiledKey {
  code: string;
  /** The key only works while shift is down. */
  needsShift: boolean;
  /** The key stops while shift is down, because a shifted binding owns it. */
  blockedByShift: boolean;
}

/** One binding with every name already resolved. The poll parses nothing. */
interface CompiledBinding {
  action: keyof ControlInput;
  kind: 'axis' | 'button' | 'rate';
  scale: number;
  axis: AxisName | null;
  button: ButtonName | null;
  touchAxis: TouchAxisName | null;
  touchButton: TouchButtonName | null;
  negativeKey: CompiledKey | null;
  positiveKey: CompiledKey | null;
  /** True when the action reads the press edge and not the hold. */
  useEdge: boolean;
  /** True when the action holds a number. */
  numeric: boolean;
}

function parseKey(spec: string, shiftedCodes: ReadonlySet<string>): CompiledKey {
  if (spec.startsWith(SHIFT_PREFIX)) {
    return { code: spec.slice(SHIFT_PREFIX.length), needsShift: true, blockedByShift: false };
  }
  return { code: spec, needsShift: false, blockedByShift: shiftedCodes.has(spec) };
}

/** Read every code that a shifted binding owns. */
function collectShiftedCodes(bindings: readonly Binding[]): ReadonlySet<string> {
  const codes = new Set<string>();
  for (const binding of bindings) {
    if (binding.keys === undefined) continue;
    for (const spec of binding.keys) {
      if (spec.startsWith(SHIFT_PREFIX)) codes.add(spec.slice(SHIFT_PREFIX.length));
    }
  }
  return codes;
}

function compile(bindings: readonly Binding[]): CompiledBinding[] {
  const shiftedCodes = collectShiftedCodes(bindings);
  const compiled: CompiledBinding[] = [];

  for (const binding of bindings) {
    let axis: AxisName | null = null;
    let button: ButtonName | null = null;

    if (binding.gamepad !== undefined) {
      if (isAxisName(binding.gamepad)) {
        axis = binding.gamepad;
      } else if (isButtonName(binding.gamepad)) {
        button = binding.gamepad;
      } else {
        throw new Error(`Unknown gamepad control in a binding: ${binding.gamepad}`);
      }
    }

    let touchAxis: TouchAxisName | null = null;
    let touchButton: TouchButtonName | null = null;

    if (binding.touch !== undefined) {
      if (isTouchAxisName(binding.touch)) {
        touchAxis = binding.touch;
      } else if (isTouchButtonName(binding.touch)) {
        touchButton = binding.touch;
      } else {
        throw new Error(`Unknown touch control in a binding: ${binding.touch}`);
      }
    }

    let negativeKey: CompiledKey | null = null;
    let positiveKey: CompiledKey | null = null;
    if (binding.keys !== undefined) {
      if (binding.keys.length === 2) {
        negativeKey = parseKey(binding.keys[0], shiftedCodes);
        positiveKey = parseKey(binding.keys[1], shiftedCodes);
      } else {
        positiveKey = parseKey(binding.keys[0], shiftedCodes);
      }
    }

    compiled.push({
      action: binding.action,
      kind: binding.kind,
      scale: binding.scale ?? 1,
      axis,
      button,
      touchAxis,
      touchButton,
      negativeKey,
      positiveKey,
      useEdge: EDGE_ACTIONS.has(binding.action),
      numeric: isNumberAction(binding.action),
    });
  }

  return compiled;
}

function createControlInput(): ControlInput {
  return {
    roll: 0,
    pitch: 0,
    yaw: 0,
    throttle: 0,
    brakeLeft: 0,
    brakeRight: 0,
    lookYaw: 0,
    lookPitch: 0,
    toggleGear: false,
    toggleFlapsUp: false,
    toggleFlapsDown: false,
    cycleView: false,
    startEngines: false,
    fireCannon: false,
    toggleMenu: false,
    toggleDebug: false,
    trimUp: false,
    trimDown: false,
    toggleHud: false,
    respawn: false,
    toggleFreeCamera: false,
    toggleSound: false,
  };
}

/**
 * Build the input system.
 *
 * The system owns the three readers. It polls them and it disposes them. A test
 * passes its own readers through `options` and needs no browser.
 */
export function createInputSystem(options?: InputSystemOptions): InputSystem {
  const gamepad: GamepadReader = options?.gamepad ?? createGamepadReader();
  const keyboard: KeyboardReader = options?.keyboard ?? createKeyboardReader();
  const touch: TouchReader = options?.touch ?? createNullTouchReader();
  const groundContact: () => boolean = options?.groundContact ?? (() => false);
  const dynamicPressure: () => number = options?.dynamicPressure ?? (() => 0);
  const compiled = compile(options?.bindings ?? DEFAULT_BINDINGS);

  const state = createControlInput();

  /** Sum of the axis kind bindings, one field per number action. */
  const axisSum = createNumberAccumulator();
  /** Sum of the button kind bindings that drive a number action. */
  const buttonSum = createNumberAccumulator();
  /** Sum of the rate kind bindings. Only the throttle uses it now. */
  const rateSum = createNumberAccumulator();
  const boolSum = createBooleanAccumulator();

  /** The throttle lever position. The poll integrates the rate into it. */
  let throttleLever = 0;
  let activeDevice: ActiveDevice = 'keyboard';

  // Lists for the device activity test. Only a bound control counts.
  const boundAxes: AxisName[] = [];
  const boundButtons: ButtonName[] = [];
  const boundCodes: string[] = [];
  const boundTouchAxes: TouchAxisName[] = [];
  const boundTouchButtons: TouchButtonName[] = [];
  for (const binding of compiled) {
    if (binding.axis !== null && !boundAxes.includes(binding.axis)) boundAxes.push(binding.axis);
    if (binding.button !== null && !boundButtons.includes(binding.button)) {
      boundButtons.push(binding.button);
    }
    if (binding.touchAxis !== null && !boundTouchAxes.includes(binding.touchAxis)) {
      boundTouchAxes.push(binding.touchAxis);
    }
    if (binding.touchButton !== null && !boundTouchButtons.includes(binding.touchButton)) {
      boundTouchButtons.push(binding.touchButton);
    }
    for (const key of [binding.negativeKey, binding.positiveKey]) {
      if (key !== null && !boundCodes.includes(key.code)) boundCodes.push(key.code);
    }
  }

  function shiftIsDown(): boolean {
    for (const code of SHIFT_CODES) {
      if (keyboard.held(code)) return true;
    }
    return false;
  }

  function keyIsOn(key: CompiledKey, edge: boolean, shift: boolean): boolean {
    if (key.needsShift && !shift) return false;
    if (key.blockedByShift && shift) return false;
    return edge ? keyboard.pressed(key.code) : keyboard.held(key.code);
  }

  /** Value of an axis binding or a rate binding, before the scale. */
  function readAxisSource(binding: CompiledBinding, shift: boolean): number {
    let value = 0;
    if (binding.axis !== null) value += gamepad.axis(binding.axis);
    if (binding.button !== null && gamepad.held(binding.button)) value += 1;
    if (binding.touchAxis !== null) value += touch.axis(binding.touchAxis);
    if (binding.touchButton !== null && touch.held(binding.touchButton)) value += 1;
    if (binding.positiveKey !== null && keyIsOn(binding.positiveKey, false, shift)) value += 1;
    if (binding.negativeKey !== null && keyIsOn(binding.negativeKey, false, shift)) value -= 1;
    return value;
  }

  /** True when a button binding is on. An edge action reads the press. */
  function readButtonSource(binding: CompiledBinding, shift: boolean): boolean {
    const edge = binding.useEdge;
    if (binding.button !== null) {
      if (edge ? gamepad.pressed(binding.button) : gamepad.held(binding.button)) return true;
    }
    if (binding.axis !== null && Math.abs(gamepad.axis(binding.axis)) >= AXIS_AS_BUTTON_THRESHOLD) {
      return true;
    }
    if (binding.touchButton !== null) {
      if (edge ? touch.pressed(binding.touchButton) : touch.held(binding.touchButton)) return true;
    }
    if (
      binding.touchAxis !== null &&
      Math.abs(touch.axis(binding.touchAxis)) >= AXIS_AS_BUTTON_THRESHOLD
    ) {
      return true;
    }
    if (binding.positiveKey !== null && keyIsOn(binding.positiveKey, edge, shift)) return true;
    if (binding.negativeKey !== null && keyIsOn(binding.negativeKey, edge, shift)) return true;
    return false;
  }

  function gamepadIsActive(): boolean {
    for (const name of boundAxes) {
      if (Math.abs(gamepad.axis(name)) > DEVICE_ACTIVITY_THRESHOLD) return true;
    }
    for (const name of boundButtons) {
      if (gamepad.held(name)) return true;
    }
    return false;
  }

  function keyboardIsActive(): boolean {
    for (const code of boundCodes) {
      if (keyboard.held(code)) return true;
    }
    return false;
  }

  function touchIsActive(): boolean {
    for (const name of boundTouchAxes) {
      if (Math.abs(touch.axis(name)) > DEVICE_ACTIVITY_THRESHOLD) return true;
    }
    for (const name of boundTouchButtons) {
      if (touch.held(name)) return true;
    }
    return false;
  }

  return {
    state,

    get activeDevice(): ActiveDevice {
      return activeDevice;
    },

    poll(dt: number): void {
      gamepad.poll();
      keyboard.poll();
      touch.poll();

      // The last device that moved owns the prompts. A frame that moves more
      // than one reports the LAST test that passed, and the order below is the
      // order of how clear the intent is. A hand on the keys is the clearest,
      // because a key needs no calibration and reports no noise. A finger on
      // the pad is next. A stick that rests off center is the least clear.
      if (gamepadIsActive()) activeDevice = 'gamepad';
      if (touchIsActive()) activeDevice = 'touch';
      if (keyboardIsActive()) activeDevice = 'keyboard';

      const shift = shiftIsDown();

      for (const action of NUMBER_ACTIONS) {
        axisSum[action] = 0;
        buttonSum[action] = 0;
        rateSum[action] = 0;
      }
      for (const action of BOOLEAN_ACTIONS) boolSum[action] = false;

      for (const binding of compiled) {
        if (binding.kind === 'button') {
          const on = readButtonSource(binding, shift);
          if (!on) continue;
          if (binding.numeric) {
            addNumber(buttonSum, binding.action, binding.scale);
          } else {
            setBoolean(boolSum, binding.action, true);
          }
          continue;
        }

        const value = readAxisSource(binding, shift) * binding.scale;
        if (!binding.numeric) {
          if (Math.abs(value) >= AXIS_AS_BUTTON_THRESHOLD) setBoolean(boolSum, binding.action, true);
          continue;
        }
        if (binding.kind === 'rate') {
          addNumber(rateSum, binding.action, value);
        } else {
          addNumber(axisSum, binding.action, value);
        }
      }

      // The three flight axes carry the control authority of this speed. See
      // controlAuthority. The look axes move the head and not a surface, so
      // they keep their full range.
      const authority = controlAuthority(dynamicPressure());
      state.roll = clamp(axisSum.roll, -1, 1) * authority;
      state.pitch = clamp(axisSum.pitch, -1, 1) * authority;
      state.yaw = clamp(axisSum.yaw, -1, 1) * authority;
      state.lookYaw = clamp(axisSum.lookYaw, -1, 1);
      state.lookPitch = clamp(axisSum.lookPitch, -1, 1);

      // The throttle is a lever, not a switch. Integrate the rate.
      const throttleRate = clamp(rateSum.throttle, -1, 1);
      throttleLever = clamp(throttleLever + throttleRate * THROTTLE_RATE * dt, 0, 1);
      state.throttle = throttleLever;

      // Differential braking.
      //
      // The two triggers carry the rudder in flight. They also carry one brake
      // each, because that is how this aircraft turns at taxi speed. The two
      // jobs cannot fight, because the taxi rule opens the brake path only
      // while the wheels touch the ground AND the throttle sits at idle. In
      // that state the rudder has almost no air over it, so nothing is lost.
      // The moment the pilot opens the throttle for the takeoff roll, the
      // brakes close and the triggers steer with the rudder alone. A brake that
      // stayed live in the takeoff roll would drag one wheel at full power,
      // which is how a taildragger-era aircraft leaves the runway sideways.
      //
      // The B button is a separate path. It is the full brake on both wheels
      // and no rule gates it, so the pilot always has a way to stop.
      const taxiBrakes = groundContact() && state.throttle <= TAXI_IDLE_THROTTLE;
      const leftDifferential = taxiBrakes ? clamp(axisSum.brakeLeft, 0, 1) : 0;
      const rightDifferential = taxiBrakes ? clamp(axisSum.brakeRight, 0, 1) : 0;
      state.brakeLeft = Math.max(leftDifferential, clamp(buttonSum.brakeLeft, 0, 1));
      state.brakeRight = Math.max(rightDifferential, clamp(buttonSum.brakeRight, 0, 1));

      for (const action of BOOLEAN_ACTIONS) state[action] = boolSum[action];
    },

    dispose(): void {
      gamepad.dispose();
      keyboard.dispose();
      touch.dispose();
      throttleLever = 0;
      const clean = createControlInput();
      for (const action of NUMBER_ACTIONS) state[action] = clean[action];
      for (const action of BOOLEAN_ACTIONS) state[action] = clean[action];
    },
  };
}

function createNumberAccumulator(): Record<NumberAction, number> {
  return {
    roll: 0,
    pitch: 0,
    yaw: 0,
    throttle: 0,
    brakeLeft: 0,
    brakeRight: 0,
    lookYaw: 0,
    lookPitch: 0,
  };
}

function createBooleanAccumulator(): Record<BooleanAction, boolean> {
  return {
    toggleGear: false,
    toggleFlapsUp: false,
    toggleFlapsDown: false,
    cycleView: false,
    startEngines: false,
    fireCannon: false,
    toggleMenu: false,
    toggleDebug: false,
    trimUp: false,
    trimDown: false,
    toggleHud: false,
    respawn: false,
    toggleFreeCamera: false,
    toggleSound: false,
  };
}

/** Add to a number accumulator. The action name comes from the table. */
function addNumber(
  target: Record<NumberAction, number>,
  action: keyof ControlInput,
  value: number,
): void {
  if (!isNumberAction(action)) return;
  target[action] += value;
}

/** Write a boolean accumulator. The action name comes from the table. */
function setBoolean(
  target: Record<BooleanAction, boolean>,
  action: keyof ControlInput,
  value: boolean,
): void {
  if (isNumberAction(action)) return;
  target[action] = value;
}
