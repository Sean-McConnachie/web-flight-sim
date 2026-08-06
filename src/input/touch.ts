/**
 * Touch controls.
 *
 * A phone has no keyboard and no gamepad. This module draws a set of controls
 * over the picture and reports them as a THIRD DEVICE. It reports hardware and
 * nothing more, the same way src/input/gamepad.ts and src/input/keyboard.ts do.
 * It does not decide what a control means. The binding table of
 * src/input/bindings.ts decides that, through the `touch` field of a `Binding`.
 *
 *
 * THE SIGN OF THE STICK
 *
 * `stickY` is positive when the thumb moves DOWN the screen. A thumb that moves
 * down pulls the stick back, and a stick that comes back raises the nose. The
 * value therefore carries the same meaning as the left stick Y of the gamepad,
 * so both devices share one line of the binding table and one sign. Read the
 * module comment of src/input/bindings.ts for why that sign is the flight
 * convention and not the screen convention.
 *
 *
 * WHY POINTER EVENTS AND NOT TOUCH EVENTS
 *
 * A pointer event carries a finger, a mouse and a pen with one code path, and
 * `setPointerCapture` keeps a finger attached to the control it started on. A
 * thumb that slides off the edge of the virtual stick therefore holds the stick,
 * the same way a real hand does. Touch events give none of that.
 *
 *
 * WHY EACH CONTROL SETS `touch-action: none`
 *
 * The browser reads a drag on a page as a scroll and two fingers as a zoom. Both
 * gestures cancel the pointer stream in the middle of a turn. `touch-action` of
 * `none` stops the browser from claiming the gesture, so the control keeps the
 * finger until it lifts.
 *
 *
 * WHAT THE PAD DOES NOT HOLD
 *
 * The pad has no look control and no free camera. A phone screen has room for
 * the controls that fly the aircraft and no more. A pilot on a phone changes the
 * view with the VIEW button and looks with the camera the view gives.
 */

/** Every axis the pad reports. The binding table names them. */
export type TouchAxisName = 'stickX' | 'stickY' | 'rudder';

export const TOUCH_AXIS_NAMES: readonly TouchAxisName[] = ['stickX', 'stickY', 'rudder'];

/** Every button the pad reports. The binding table names them. */
export type TouchButtonName =
  | 'throttleUp'
  | 'throttleDown'
  | 'gear'
  | 'flapsDown'
  | 'flapsUp'
  | 'brake'
  | 'view'
  | 'fire'
  | 'engineStart'
  | 'panels'
  | 'respawn';

export const TOUCH_BUTTON_NAMES: readonly TouchButtonName[] = [
  'throttleUp',
  'throttleDown',
  'gear',
  'flapsDown',
  'flapsUp',
  'brake',
  'view',
  'fire',
  'engineStart',
  'panels',
  'respawn',
];

export interface TouchReader {
  /**
   * True while the pad is on the screen and can report a finger. A pad that is
   * hidden reports false and every control reads zero.
   */
  readonly active: boolean;
  /** Value of one axis, -1 to 1. It reads zero when no finger holds it. */
  axis(name: TouchAxisName): number;
  /** True while a finger holds the button. */
  held(name: TouchButtonName): boolean;
  /** True only on the poll after the finger went down. */
  pressed(name: TouchButtonName): boolean;
  /** True only on the poll after the finger came up. */
  released(name: TouchButtonName): boolean;
  /** Read the events of the last frame. Call it one time per frame. */
  poll(): void;
  /** Draws the pad, or takes it off the screen. */
  visible: boolean;
  /**
   * Fills the bar of the throttle rocker, 0 to 1. The pad holds no lever of its
   * own, because the binding table integrates the rate. The caller therefore
   * hands the lever position over one time per frame.
   */
  setThrottle(value: number): void;
  dispose(): void;
}

/**
 * Reports whether this browser drives a touch screen.
 *
 * `pointer: coarse` is true for a finger and false for a mouse. `hover: none`
 * is true for a device that cannot hold a cursor over a control. A phone and a
 * tablet answer true to both. A laptop with a touch screen and a mouse answers
 * false, which is correct: that pilot has a keyboard.
 */
export function touchScreenAvailable(): boolean {
  if (typeof window === 'undefined' || typeof window.matchMedia !== 'function') return false;
  return window.matchMedia('(pointer: coarse)').matches && window.matchMedia('(hover: none)').matches;
}

/**
 * A reader that holds no pad and reports nothing.
 *
 * src/input/bindings.ts builds this one when the caller gives no pad, so a test
 * and a desktop both run the touch path with no DOM and no cost.
 */
export function createNullTouchReader(): TouchReader {
  return {
    active: false,
    axis: () => 0,
    held: () => false,
    pressed: () => false,
    released: () => false,
    poll(): void {
      // The null reader holds no state, so a poll has nothing to move.
    },
    visible: false,
    setThrottle(): void {
      // No bar exists.
    },
    dispose(): void {
      // Nothing to free.
    },
  };
}

// ---------------------------------------------------------------------------
// The layout
// ---------------------------------------------------------------------------

/** One round button of the block at the bottom right. */
interface ButtonSpec {
  readonly name: TouchButtonName;
  /** The word on the face. It is short, because the face is small. */
  readonly label: string;
  /** A press that fires one time reads the edge. A hold reads the hold. */
  readonly hold: boolean;
}

/**
 * The button block, read left to right and then down.
 *
 * The three columns hold the engine and the view on the left, the flaps and the
 * gear in the middle, and the cannon on the right. The throttle rocker stands
 * apart from this grid, because it is tall and it is the control a pilot holds
 * the longest.
 */
const BUTTON_GRID: readonly ButtonSpec[] = [
  { name: 'engineStart', label: 'ENG', hold: true },
  { name: 'flapsUp', label: 'FLAP UP', hold: false },
  { name: 'fire', label: 'FIRE', hold: true },
  { name: 'view', label: 'VIEW', hold: false },
  { name: 'flapsDown', label: 'FLAP DN', hold: false },
  { name: 'brake', label: 'BRAKE', hold: true },
  { name: 'respawn', label: 'RESET', hold: false },
  { name: 'gear', label: 'GEAR', hold: false },
];

/** The two halves of the throttle rocker. The upper half adds power. */
const THROTTLE_BUTTONS: readonly ButtonSpec[] = [
  { name: 'throttleUp', label: 'THR +', hold: true },
  { name: 'throttleDown', label: 'THR -', hold: true },
];

/**
 * The pill buttons of the top bar.
 *
 * The CONTROLS button of src/ui/controls-menu.ts and the SOUND button of
 * src/ui/sound-button.ts stand in the same row, to the LEFT of these. They
 * belong to those modules and not to this one, because both must also stand on
 * a desktop where this pad does not exist. The indent of the bar in the style
 * sheet below holds the room this row leaves for the pair.
 */
const BAR_BUTTONS: readonly ButtonSpec[] = [
  { name: 'panels', label: 'PANELS', hold: false },
];

/**
 * Every button the pad DRAWS, and every axis it draws.
 *
 * TOUCH_BUTTON_NAMES above says which names exist. These two say which names
 * reach the screen. A name that exists and never reaches the screen is a
 * control the pilot can never use, and the unit test holds the two lists
 * against each other to catch that.
 */
export const TOUCH_DRAWN_BUTTONS: readonly TouchButtonName[] = [
  ...BUTTON_GRID,
  ...THROTTLE_BUTTONS,
  ...BAR_BUTTONS,
].map((spec) => spec.name);

export const TOUCH_DRAWN_AXES: readonly TouchAxisName[] = ['stickX', 'stickY', 'rudder'];

const STYLE_ID = 'hfs-touch-style';

const CSS = `
.hfs-touch {
  position: absolute;
  inset: 0;
  pointer-events: none;
  user-select: none;
  -webkit-user-select: none;
  font: 600 12px/1 ui-monospace, 'DejaVu Sans Mono', monospace;
  letter-spacing: 0.06em;
  color: #d8ffe6;
  z-index: 5;
}
.hfs-touch-control {
  position: absolute;
  pointer-events: auto;
  touch-action: none;
  -webkit-tap-highlight-color: transparent;
  box-sizing: border-box;
  background: rgba(6, 14, 10, 0.42);
  border: 1px solid rgba(120, 200, 150, 0.35);
  color: inherit;
  display: grid;
  place-items: center;
  text-align: center;
}
.hfs-touch-control.down {
  background: rgba(120, 200, 150, 0.34);
  border-color: rgba(216, 255, 230, 0.8);
}

/* The virtual stick. It stands at the bottom left, under the left thumb. */
.hfs-touch-stick {
  left: max(12px, env(safe-area-inset-left));
  bottom: max(12px, env(safe-area-inset-bottom));
  /* A phone in landscape is 390 px tall, so vmin reads off the SHORT side and
     38vmin gives 148 px. The travel radius is half of that. A smaller face
     leaves a thumb too little room to hold a small deflection. */
  width: clamp(130px, 38vmin, 210px);
  height: clamp(130px, 38vmin, 210px);
  border-radius: 50%;
}
.hfs-touch-knob {
  position: absolute;
  width: 34%;
  height: 34%;
  border-radius: 50%;
  background: rgba(120, 200, 150, 0.45);
  border: 1px solid rgba(216, 255, 230, 0.75);
  pointer-events: none;
  will-change: transform;
}
.hfs-touch-stick-mark {
  position: absolute;
  bottom: 6%;
  font-size: 10px;
  color: #79b795;
  pointer-events: none;
}

/* The rudder bar. It sits over the stick, so one thumb reaches both. */
.hfs-touch-rudder {
  left: max(12px, env(safe-area-inset-left));
  /* It stands directly over the stick, so one thumb reaches both. The height of
     the stick is therefore part of this sum. */
  bottom: calc(max(12px, env(safe-area-inset-bottom)) + clamp(130px, 38vmin, 210px) + 12px);
  width: clamp(130px, 38vmin, 210px);
  height: clamp(34px, 7vmin, 50px);
  border-radius: 6px;
}
.hfs-touch-rudder-knob {
  position: absolute;
  width: 26%;
  height: 78%;
  border-radius: 4px;
  background: rgba(120, 200, 150, 0.45);
  border: 1px solid rgba(216, 255, 230, 0.75);
  pointer-events: none;
  will-change: transform;
}
.hfs-touch-rudder-mark {
  /* The bar is short and the knob sits in the middle of it, so the word stands
     OVER the bar and not inside it. */
  position: absolute;
  top: -13px;
  left: 2px;
  font-size: 10px;
  color: #79b795;
  pointer-events: none;
}

/* The button block at the bottom right. */
.hfs-touch-buttons {
  position: absolute;
  right: calc(max(12px, env(safe-area-inset-right)) + clamp(60px, 13vmin, 84px) + 10px);
  bottom: max(12px, env(safe-area-inset-bottom));
  display: grid;
  grid-template-columns: repeat(3, clamp(56px, 12vmin, 78px));
  gap: 8px;
  pointer-events: none;
}
.hfs-touch-button {
  position: relative;
  height: clamp(56px, 12vmin, 78px);
  border-radius: 50%;
  font-size: clamp(9px, 1.5vmin, 12px);
}

/* The throttle rocker. It is tall, because a pilot holds it for seconds. */
.hfs-touch-throttle {
  position: absolute;
  right: max(12px, env(safe-area-inset-right));
  bottom: max(12px, env(safe-area-inset-bottom));
  width: clamp(60px, 13vmin, 84px);
  height: clamp(180px, 38vmin, 260px);
  display: grid;
  grid-template-rows: 1fr 1fr;
  gap: 6px;
  pointer-events: none;
}
.hfs-touch-throttle-half {
  position: relative;
  border-radius: 8px;
  overflow: hidden;
  font-size: clamp(10px, 1.6vmin, 13px);
}
.hfs-touch-throttle-fill {
  position: absolute;
  left: 0;
  right: 0;
  bottom: 0;
  height: 0%;
  background: rgba(120, 200, 150, 0.3);
  pointer-events: none;
}
.hfs-touch-throttle-text {
  position: relative;
}

/* The top bar. It carries the panels switch and the collapse.

   The 224 px indent is the width of TWO buttons that stand first in this row
   and belong to other modules, with the 8 px gap this row uses between its own
   pills after each one. They are the CONTROLS button of
   src/ui/controls-menu.ts and the SOUND button of src/ui/sound-button.ts, and
   both are 104 px wide.

   Those two lead first because one of them leads a new pilot to every other
   control, and the other is the first thing anybody looks for when a page
   makes a noise. */
.hfs-touch-bar {
  position: absolute;
  left: calc(max(12px, env(safe-area-inset-left)) + 224px);
  top: max(12px, env(safe-area-inset-top));
  display: flex;
  gap: 8px;
  pointer-events: none;
}
.hfs-touch-pill {
  position: relative;
  height: 34px;
  min-width: 62px;
  padding: 0 10px;
  border-radius: 17px;
  font-size: 11px;
}
`;

/** Add the style sheet one time, whatever the number of pads. */
function ensureStyle(): void {
  if (document.getElementById(STYLE_ID) !== null) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = CSS;
  document.head.appendChild(style);
}

function makeDiv(className: string, parent: HTMLElement): HTMLDivElement {
  const el = document.createElement('div');
  el.className = className;
  parent.appendChild(el);
  return el;
}

/** Distance from the center of a control at which the axis reads 1. */
function travelRadius(rect: DOMRect): number {
  return Math.max(1, Math.min(rect.width, rect.height) / 2);
}

/**
 * Attaches a pointer to the control it started on.
 *
 * `setPointerCapture` throws `NotFoundError` when the pointer is already gone,
 * which happens when a finger lifts in the same frame it lands. The capture is
 * an improvement and not a requirement, so a failure must not stop the press.
 */
function capture(element: HTMLElement, pointerId: number): void {
  try {
    element.setPointerCapture(pointerId);
  } catch {
    // The pointer ended before the capture. The press still counts.
  }
}

function clampUnit(value: number): number {
  if (value > 1) return 1;
  if (value < -1) return -1;
  return value;
}

export interface TouchPadOptions {
  /** The pad starts on the screen when this is true. */
  visible?: boolean;
}

/**
 * Builds the pad and attaches it to `parent`.
 *
 * The pad reports nothing while it is hidden, and a hidden pad takes no pointer
 * event, so the picture under it keeps every gesture.
 */
export function createTouchPad(parent: HTMLElement, options?: TouchPadOptions): TouchReader {
  ensureStyle();

  const root = makeDiv('hfs-touch', parent);

  // --- The state the readers report --------------------------------------
  const axes = new Map<TouchAxisName, number>();
  /** Buttons a finger holds now. */
  const down = new Set<TouchButtonName>();
  /** Buttons that went down since the last poll. */
  const downLatch = new Set<TouchButtonName>();
  /** Buttons that came up since the last poll. */
  const upLatch = new Set<TouchButtonName>();
  /** The three sets of the current frame. `poll` fills them. */
  const heldNow = new Set<TouchButtonName>();
  const pressedNow = new Set<TouchButtonName>();
  const releasedNow = new Set<TouchButtonName>();

  /** Every listener the pad added, so `dispose` takes all of them off. */
  const cleanups: (() => void)[] = [];

  /**
   * One entry for each control, which drops the finger that control holds.
   *
   * Each control holds the id of its own pointer, and it refuses a new finger
   * while it holds one. A blur and a collapse both take a control off the
   * screen without a pointer up event, so that id would stay set for ever and
   * the control would never answer again. `clearAll` runs this list.
   */
  const resets: (() => void)[] = [];

  function press(name: TouchButtonName): void {
    if (down.has(name)) return;
    down.add(name);
    downLatch.add(name);
  }

  function release(name: TouchButtonName): void {
    if (!down.has(name)) return;
    down.delete(name);
    upLatch.add(name);
  }

  /**
   * Wires one element as a button.
   *
   * A finger that starts on the face holds the button until it lifts, wherever
   * it travels, because the element captures the pointer. A button whose action
   * fires one time still holds, and the binding table reads the press edge of
   * that hold. The `hold` field therefore only changes the words in the controls
   * menu, not the code path here.
   */
  function wireButton(element: HTMLElement, name: TouchButtonName): void {
    let pointer = -1;

    const onDown = (event: PointerEvent): void => {
      if (pointer !== -1) return;
      pointer = event.pointerId;
      capture(element, event.pointerId);
      element.classList.add('down');
      press(name);
      event.preventDefault();
    };

    const onUp = (event: PointerEvent): void => {
      if (event.pointerId !== pointer) return;
      pointer = -1;
      element.classList.remove('down');
      release(name);
      event.preventDefault();
    };

    element.addEventListener('pointerdown', onDown);
    element.addEventListener('pointerup', onUp);
    element.addEventListener('pointercancel', onUp);
    cleanups.push(() => {
      element.removeEventListener('pointerdown', onDown);
      element.removeEventListener('pointerup', onUp);
      element.removeEventListener('pointercancel', onUp);
    });
    resets.push(() => {
      pointer = -1;
      element.classList.remove('down');
    });
  }

  /**
   * Wires one element as an axis pair or as a single axis.
   *
   * The control has a fixed center. A finger reports the offset from that
   * center over the travel radius, and the axis returns to zero when the finger
   * lifts. A floating center would move the neutral point of the aircraft under
   * the pilot, so this pad does not use one.
   */
  function wireAxis(
    element: HTMLElement,
    knob: HTMLElement,
    xName: TouchAxisName | null,
    yName: TouchAxisName | null,
  ): void {
    let pointer = -1;

    const write = (x: number, y: number): void => {
      if (xName !== null) axes.set(xName, x);
      if (yName !== null) axes.set(yName, y);
      // The knob moves by half of its own box for each unit of travel, so it
      // stays inside the face at full deflection.
      const dx = xName === null ? 0 : x * 50;
      const dy = yName === null ? 0 : y * 50;
      knob.style.transform = `translate(${dx.toFixed(1)}%, ${dy.toFixed(1)}%)`;
    };

    const move = (event: PointerEvent): void => {
      const rect = element.getBoundingClientRect();
      const radius = travelRadius(rect);
      const x = clampUnit((event.clientX - (rect.left + rect.width / 2)) / radius);
      // A finger that moves DOWN the screen pulls the stick back. See the
      // module comment.
      const y = clampUnit((event.clientY - (rect.top + rect.height / 2)) / radius);
      write(x, y);
    };

    const onDown = (event: PointerEvent): void => {
      if (pointer !== -1) return;
      pointer = event.pointerId;
      capture(element, event.pointerId);
      element.classList.add('down');
      move(event);
      event.preventDefault();
    };

    const onMove = (event: PointerEvent): void => {
      if (event.pointerId !== pointer) return;
      move(event);
      event.preventDefault();
    };

    const onUp = (event: PointerEvent): void => {
      if (event.pointerId !== pointer) return;
      pointer = -1;
      element.classList.remove('down');
      write(0, 0);
      event.preventDefault();
    };

    element.addEventListener('pointerdown', onDown);
    element.addEventListener('pointermove', onMove);
    element.addEventListener('pointerup', onUp);
    element.addEventListener('pointercancel', onUp);
    cleanups.push(() => {
      element.removeEventListener('pointerdown', onDown);
      element.removeEventListener('pointermove', onMove);
      element.removeEventListener('pointerup', onUp);
      element.removeEventListener('pointercancel', onUp);
    });
    resets.push(() => {
      pointer = -1;
      element.classList.remove('down');
      write(0, 0);
    });
    write(0, 0);
  }

  // --- The stick ----------------------------------------------------------
  const stick = makeDiv('hfs-touch-control hfs-touch-stick', root);
  const stickKnob = makeDiv('hfs-touch-knob', stick);
  const stickMark = makeDiv('hfs-touch-stick-mark', stick);
  stickMark.textContent = 'STICK';
  wireAxis(stick, stickKnob, 'stickX', 'stickY');

  // --- The rudder ---------------------------------------------------------
  const rudder = makeDiv('hfs-touch-control hfs-touch-rudder', root);
  const rudderKnob = makeDiv('hfs-touch-rudder-knob', rudder);
  const rudderMark = makeDiv('hfs-touch-rudder-mark', rudder);
  rudderMark.textContent = 'RUDDER';
  wireAxis(rudder, rudderKnob, 'rudder', null);

  // --- The button block ---------------------------------------------------
  const buttons = makeDiv('hfs-touch-buttons', root);
  for (const spec of BUTTON_GRID) {
    const element = makeDiv('hfs-touch-control hfs-touch-button', buttons);
    element.textContent = spec.label;
    wireButton(element, spec.name);
  }

  // --- The throttle rocker ------------------------------------------------
  const throttle = makeDiv('hfs-touch-throttle', root);
  const throttleFills: HTMLDivElement[] = [];
  for (const spec of THROTTLE_BUTTONS) {
    const element = makeDiv('hfs-touch-control hfs-touch-throttle-half', throttle);
    const fill = makeDiv('hfs-touch-throttle-fill', element);
    const text = makeDiv('hfs-touch-throttle-text', element);
    text.textContent = spec.label;
    throttleFills.push(fill);
    wireButton(element, spec.name);
  }

  // --- The top bar --------------------------------------------------------
  const bar = makeDiv('hfs-touch-bar', root);
  const collapse = makeDiv('hfs-touch-control hfs-touch-pill', bar);
  const pills: HTMLDivElement[] = [];
  for (const spec of BAR_BUTTONS) {
    const element = makeDiv('hfs-touch-control hfs-touch-pill', bar);
    element.textContent = spec.label;
    pills.push(element);
    wireButton(element, spec.name);
  }

  /**
   * True while every control except the collapse pill is off the screen.
   *
   * A pilot who wants to see the aircraft needs a way to clear the pad, and the
   * way back must stay on the screen. The collapse pill is therefore the one
   * control the pad never hides while it is on.
   */
  let collapsed = false;
  /** Every part of the pad that the collapse pill hides. */
  const collapsible: HTMLElement[] = [stick, rudder, buttons, throttle, ...pills];

  function applyCollapse(): void {
    collapse.textContent = collapsed ? 'CONTROLS' : 'HIDE PAD';
    for (const element of collapsible) element.style.display = collapsed ? 'none' : '';
  }

  const onCollapse = (event: PointerEvent): void => {
    event.preventDefault();
    collapsed = !collapsed;
    applyCollapse();
    // A control that leaves the screen never sends its pointer up event, so
    // release every finger by hand. Without this the aircraft holds full
    // aileron after the pad closes.
    clearAll();
  };
  collapse.addEventListener('pointerdown', onCollapse);
  cleanups.push(() => {
    collapse.removeEventListener('pointerdown', onCollapse);
  });
  applyCollapse();

  /**
   * Drops every finger and centers every axis.
   *
   * The release goes through `upLatch`, so a caller that reads the edge still
   * sees one release for each button that was down. The `resets` list then
   * frees each control to take a new finger. See the comment on that list.
   */
  function clearAll(): void {
    for (const name of down) upLatch.add(name);
    down.clear();
    axes.clear();
    for (const reset of resets) reset();
  }

  // The browser stops the pointer up event when the window loses the focus, the
  // same way it stops the key up event. Release every control. See the BLUR
  // part of src/input/keyboard.ts.
  const onBlur = (): void => {
    clearAll();
  };
  if (typeof window !== 'undefined') {
    window.addEventListener('blur', onBlur);
    cleanups.push(() => {
      window.removeEventListener('blur', onBlur);
    });
  }

  let visible = options?.visible ?? false;
  root.style.display = visible ? '' : 'none';

  /** The last bar height written, so an equal value costs no layout pass. */
  let lastThrottle = -1;

  const api: TouchReader = {
    get active(): boolean {
      return visible && !collapsed;
    },

    get visible(): boolean {
      return visible;
    },

    set visible(value: boolean) {
      if (value === visible) return;
      visible = value;
      root.style.display = visible ? '' : 'none';
      if (!visible) clearAll();
    },

    axis(name: TouchAxisName): number {
      if (!visible || collapsed) return 0;
      return axes.get(name) ?? 0;
    },

    held(name: TouchButtonName): boolean {
      return heldNow.has(name);
    },

    pressed(name: TouchButtonName): boolean {
      return pressedNow.has(name);
    },

    released(name: TouchButtonName): boolean {
      return releasedNow.has(name);
    },

    poll(): void {
      pressedNow.clear();
      releasedNow.clear();
      heldNow.clear();
      for (const name of downLatch) pressedNow.add(name);
      for (const name of upLatch) releasedNow.add(name);
      for (const name of down) heldNow.add(name);
      downLatch.clear();
      upLatch.clear();
    },

    setThrottle(value: number): void {
      const level = clampUnit(value);
      if (Math.abs(level - lastThrottle) < 0.005) return;
      lastThrottle = level;
      const percent = `${(level * 100).toFixed(0)}%`;
      for (const fill of throttleFills) fill.style.height = percent;
    },

    dispose(): void {
      for (const off of cleanups) off();
      cleanups.length = 0;
      root.remove();
      down.clear();
      downLatch.clear();
      upLatch.clear();
      heldNow.clear();
      pressedNow.clear();
      releasedNow.clear();
      axes.clear();
    },
  };

  return api;
}
