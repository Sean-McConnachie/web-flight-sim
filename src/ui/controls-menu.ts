/**
 * The controls menu.
 *
 * The menu prints every control the pilot has, for all three devices. It reads
 * the binding table of src/input/bindings.ts and builds each row from it, so a
 * change to a binding changes this list on the next frame. A list written by
 * hand goes stale the first time somebody moves a key.
 *
 *
 * 1. WHAT THE MENU CANNOT READ
 *
 * The binding table holds the actions. It does not hold the words for them, and
 * it does not hold the mouse. Two tables in this file supply the rest:
 *
 * - ACTION_INFO gives one row of prose for each action. Its type is
 *   `Record<keyof ControlInput, ActionInfo>`, so the compiler refuses the file
 *   the moment somebody adds an action and forgets the words for it.
 * - EXTRA_NOTES holds the controls that reach the simulator around the binding
 *   table. The mouse is the whole of that list. src/render/cameras.ts owns it,
 *   because a mouse reports movement and a binding needs a position.
 *
 *
 * 2. THE ORDER INSIDE ONE CELL
 *
 * An action such as the rudder or the throttle has a control for each
 * direction. The label of the action names the two directions in one order, so
 * the chips must follow that same order. A binding with a NEGATIVE scale is the
 * first direction, and a key pair already arrives in the order
 * [negative, positive]. `deviceChips` sorts on that rule.
 *
 *
 * 3. THE NARROW WINDOW
 *
 * A phone holds four columns badly. Under 760 px each row becomes a stack, and
 * every cell grows a word in front of it that names its device. The rule sits in
 * one media query at the end of the style sheet.
 *
 * This file may touch the DOM. CONVENTIONS section 4 allows that under src/ui.
 * It holds no physics.
 */

import type { Binding, ControlInput } from '@/input/bindings';
import { DEFAULT_BINDINGS, EDGE_ACTIONS } from '@/input/bindings';

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/** The blocks of the list, in the order they print. */
export type ActionGroup = 'flight' | 'systems' | 'view' | 'simulator';

export const ACTION_GROUPS: readonly ActionGroup[] = ['flight', 'systems', 'view', 'simulator'];

export const GROUP_TITLES: Record<ActionGroup, string> = {
  flight: 'Flight controls',
  systems: 'Engine and systems',
  view: 'View and camera',
  simulator: 'Simulator',
};

/** The words for one action. See part 1. */
export interface ActionInfo {
  /** The name of the action. It names both directions when there are two. */
  readonly label: string;
  readonly group: ActionGroup;
  /** One line under the row, or nothing. It says what the action cannot do. */
  readonly note?: string;
}

/** One printed chip, which is one control of one device. */
export interface DeviceChip {
  readonly text: string;
  /** Negative for the first direction of the label. See part 2. */
  readonly scale: number;
}

/** One printed row of the list. */
export interface MenuRow {
  readonly action: keyof ControlInput;
  readonly label: string;
  readonly group: ActionGroup;
  readonly note: string;
  /** True while the action fires one time for each press. */
  readonly edge: boolean;
  readonly keyboard: readonly string[];
  readonly gamepad: readonly string[];
  readonly touch: readonly string[];
}

export interface ControlsMenu {
  /** True while the panel is on the screen. */
  visible: boolean;
  /** Turns the panel on when it is off, and off when it is on. */
  toggle(): void;
  dispose(): void;
}

/**
 * Width of the CONTROLS button, px.
 *
 * The top bar of the touch pad stands to the RIGHT of that button and indents
 * itself by this width plus its own 8 px gap. The number therefore appears in
 * the style sheet of src/input/touch.ts as well, and the comment there names
 * this constant. A pad may not import a panel, so the two cannot share it.
 */
export const MENU_BUTTON_WIDTH = 104;

// ---------------------------------------------------------------------------
// The words
// ---------------------------------------------------------------------------

/**
 * One row of prose for each action.
 *
 * The type is a complete record over `ControlInput`, so the compiler fails the
 * build when an action has no words. That is the whole reason for the type.
 */
export const ACTION_INFO: Record<keyof ControlInput, ActionInfo> = {
  roll: { label: 'Roll (left, right)', group: 'flight' },
  pitch: { label: 'Pitch (nose down, nose up)', group: 'flight' },
  yaw: { label: 'Rudder (left, right)', group: 'flight' },
  throttle: { label: 'Throttle (less, more)', group: 'flight' },
  brakeLeft: { label: 'Left wheel brake', group: 'flight' },
  brakeRight: { label: 'Right wheel brake', group: 'flight' },
  trimUp: {
    label: 'Trim (nose up)',
    group: 'flight',
    note: 'The aircraft has no stabilizer trim channel, so this key does nothing yet.',
  },
  trimDown: {
    label: 'Trim (nose down)',
    group: 'flight',
    note: 'The aircraft has no stabilizer trim channel, so this key does nothing yet.',
  },

  toggleGear: { label: 'Landing gear', group: 'systems' },
  toggleFlapsDown: { label: 'Flaps, one step down', group: 'systems' },
  toggleFlapsUp: { label: 'Flaps, one step up', group: 'systems' },
  startEngines: {
    label: 'Engine start, held',
    group: 'systems',
    note: 'Hold the control. The head up display prints the next step of the start.',
  },
  fireCannon: { label: 'Fire the four MK 108, held', group: 'systems' },

  cycleView: { label: 'Next view: cockpit, chase, orbit, flyby', group: 'view' },
  lookYaw: { label: 'Look, sideways', group: 'view' },
  lookPitch: { label: 'Look, up and down', group: 'view' },
  toggleFreeCamera: { label: 'Free camera, for development', group: 'view' },

  toggleMenu: { label: 'This menu', group: 'simulator' },
  toggleHud: { label: 'Hide or show every overlay panel', group: 'simulator' },
  toggleDebug: { label: 'Debug level: off, numbers, force arrows', group: 'simulator' },
  respawn: { label: 'Put the aircraft back on the runway', group: 'simulator' },
};

/** Controls that do not come through the binding table. See part 1. */
export const EXTRA_NOTES: readonly string[] = [
  'Mouse: hold the left button and move to look around. The view returns to center about half a second after the button comes up.',
  'Mouse wheel: it zooms the orbit view between 8 m and 400 m.',
  'Free camera: W, A, S, D, Q and E fly it. A Shift key makes it eight times faster.',
  'A key that a shifted binding claims goes dead while a Shift key is down. That is why F lowers the flaps and Shift F raises them.',
  'A control pair prints in the order of the words in front of it.',
];

/** The words for one gamepad control. */
const GAMEPAD_LABELS: Record<string, string> = {
  leftX: 'Left stick, sideways',
  leftY: 'Left stick, fore and aft',
  rightX: 'Right stick, sideways',
  rightY: 'Right stick, up and down',
  leftTrigger: 'Left trigger',
  rightTrigger: 'Right trigger',
  a: 'A',
  b: 'B',
  x: 'X',
  y: 'Y',
  leftBumper: 'Left bumper',
  rightBumper: 'Right bumper',
  back: 'Back',
  start: 'Start',
  leftStick: 'Left stick button',
  rightStick: 'Right stick button',
  dpadUp: 'D-pad up',
  dpadDown: 'D-pad down',
  dpadLeft: 'D-pad left',
  dpadRight: 'D-pad right',
};

/** The words for one control of the on screen pad. */
const TOUCH_LABELS: Record<string, string> = {
  stickX: 'Stick, sideways',
  stickY: 'Stick, fore and aft',
  rudder: 'Rudder bar',
  throttleUp: 'THR +',
  throttleDown: 'THR -',
  gear: 'GEAR',
  flapsDown: 'FLAP DN',
  flapsUp: 'FLAP UP',
  brake: 'BRAKE',
  view: 'VIEW',
  fire: 'FIRE',
  engineStart: 'ENG',
  panels: 'PANELS',
  respawn: 'RESET',
};

/** Key codes whose printed name is not the code with its prefix removed. */
const KEY_LABELS: Record<string, string> = {
  Escape: 'Esc',
  Space: 'Space',
  Enter: 'Enter',
  Tab: 'Tab',
  Backspace: 'Backspace',
  Minus: '-',
  Equal: '=',
  BracketLeft: '[',
  BracketRight: ']',
  Backslash: '\\',
  Semicolon: ';',
  Quote: "'",
  Backquote: '`',
  Comma: ',',
  Period: '.',
  Slash: '/',
  PageUp: 'Page Up',
  PageDown: 'Page Down',
  ArrowUp: 'Up',
  ArrowDown: 'Down',
  ArrowLeft: 'Left',
  ArrowRight: 'Right',
  ShiftLeft: 'Shift',
  ShiftRight: 'Shift',
};

/** Prefix that carries the shift modifier inside a key code. */
const SHIFT_PREFIX = 'Shift+';

/**
 * Turns a `KeyboardEvent.code` into the word a pilot reads.
 *
 * `Shift+KeyF` gives `Shift F`. `KeyW` gives `W`. `PageUp` gives `Page Up`. A
 * code with no entry in the table above loses its `Key` or `Digit` prefix and
 * then takes a space in front of each capital, so `CapsLock` gives `Caps Lock`.
 */
export function keyLabel(code: string): string {
  if (code.startsWith(SHIFT_PREFIX)) return `Shift ${keyLabel(code.slice(SHIFT_PREFIX.length))}`;
  const known = KEY_LABELS[code];
  if (known !== undefined) return known;
  if (code.startsWith('Key') && code.length === 4) return code.slice(3);
  if (code.startsWith('Digit') && code.length === 6) return code.slice(5);
  if (code.startsWith('Numpad')) return `Numpad ${code.slice(6)}`;
  // A function key is already one word with a number, so it must not split.
  if (/^F\d{1,2}$/.test(code)) return code;
  return code.replace(/([a-z])([A-Z])/g, '$1 $2');
}

/** The word for one gamepad control, or the raw name when it has none. */
export function gamepadLabel(name: string): string {
  return GAMEPAD_LABELS[name] ?? name;
}

/** The word for one control of the on screen pad, or the raw name. */
export function touchLabel(name: string): string {
  return TOUCH_LABELS[name] ?? name;
}

// ---------------------------------------------------------------------------
// The rows
// ---------------------------------------------------------------------------

/**
 * Sorts the chips of one cell and drops a repeat.
 *
 * A negative scale is the first direction of the label. See part 2. The sort
 * holds the table order inside each of the two groups, because `Array.sort` is
 * stable in every engine this project runs on.
 */
function deviceChips(chips: readonly DeviceChip[]): readonly string[] {
  const sorted = [...chips].sort((a, b) => (a.scale < 0 ? 0 : 1) - (b.scale < 0 ? 0 : 1));
  const out: string[] = [];
  for (const chip of sorted) {
    if (!out.includes(chip.text)) out.push(chip.text);
  }
  return out;
}

/**
 * Builds one row for each action of the binding table.
 *
 * The function is pure, so a test drives it with no DOM. An action with no
 * binding on any device still gets a row, because a pilot who cannot find a
 * control must be able to see that there is none.
 */
export function buildRows(bindings: readonly Binding[] = DEFAULT_BINDINGS): readonly MenuRow[] {
  const keyboard = new Map<string, DeviceChip[]>();
  const gamepad = new Map<string, DeviceChip[]>();
  const touch = new Map<string, DeviceChip[]>();

  const push = (map: Map<string, DeviceChip[]>, action: string, chip: DeviceChip): void => {
    const list = map.get(action);
    if (list === undefined) map.set(action, [chip]);
    else list.push(chip);
  };

  for (const binding of bindings) {
    const scale = binding.scale ?? 1;
    if (binding.keys !== undefined) {
      if (binding.keys.length === 2) {
        // The pair already arrives as [negative, positive].
        push(keyboard, binding.action, { text: keyLabel(binding.keys[0]), scale: -1 });
        push(keyboard, binding.action, { text: keyLabel(binding.keys[1]), scale: 1 });
      } else {
        push(keyboard, binding.action, { text: keyLabel(binding.keys[0]), scale });
      }
    }
    if (binding.gamepad !== undefined) {
      push(gamepad, binding.action, { text: gamepadLabel(binding.gamepad), scale });
    }
    if (binding.touch !== undefined) {
      push(touch, binding.action, { text: touchLabel(binding.touch), scale });
    }
  }

  const rows: MenuRow[] = [];
  for (const group of ACTION_GROUPS) {
    for (const key of Object.keys(ACTION_INFO) as (keyof ControlInput)[]) {
      const info = ACTION_INFO[key];
      if (info.group !== group) continue;
      rows.push({
        action: key,
        label: info.label,
        group,
        note: info.note ?? '',
        edge: EDGE_ACTIONS.has(key),
        keyboard: deviceChips(keyboard.get(key) ?? []),
        gamepad: deviceChips(gamepad.get(key) ?? []),
        touch: deviceChips(touch.get(key) ?? []),
      });
    }
  }
  return rows;
}

// ---------------------------------------------------------------------------
// The panel
// ---------------------------------------------------------------------------

const STYLE_ID = 'hfs-menu-style';

const CSS = `
/* The button that opens the panel.

   It stands FIRST in the top left row, above every other overlay, and it never
   leaves the screen while the panel is shut. A pilot who has just loaded the
   page has no way to know that a menu exists, and the panel switch of the H
   key cannot teach them, because a key nobody can see is a key nobody presses.

   It is 104 px wide, and MENU_BUTTON_WIDTH holds that number for the touch pad
   to indent by. The panel switch does NOT hide it: help that a pilot cannot
   reach is worse than a small button in the corner. */
.hfs-menu-open {
  position: absolute;
  left: max(12px, env(safe-area-inset-left));
  top: max(12px, env(safe-area-inset-top));
  width: 104px;
  height: 34px;
  z-index: 8;
  pointer-events: auto;
  touch-action: manipulation;
  -webkit-tap-highlight-color: transparent;
  cursor: pointer;
  box-sizing: border-box;
  padding: 0;
  border-radius: 17px;
  background: rgba(6, 14, 10, 0.72);
  border: 1px solid rgba(120, 200, 150, 0.55);
  color: #d8ffe6;
  font: 600 11px/1 ui-monospace, 'DejaVu Sans Mono', monospace;
  letter-spacing: 0.08em;
  text-shadow: 0 1px 3px rgba(0, 0, 0, 0.9);
}
.hfs-menu-open:hover {
  background: rgba(120, 200, 150, 0.3);
  border-color: rgba(216, 255, 230, 0.8);
}
.hfs-menu-scrim {
  position: absolute;
  inset: 0;
  display: none;
  pointer-events: auto;
  background: rgba(4, 8, 6, 0.72);
  z-index: 10;
  overflow: auto;
  -webkit-overflow-scrolling: touch;
  padding: 16px;
  box-sizing: border-box;
}
.hfs-menu-scrim.open { display: block; }
.hfs-menu-panel {
  margin: 0 auto;
  max-width: 940px;
  box-sizing: border-box;
  padding: 16px 18px 20px;
  background: rgba(6, 14, 10, 0.94);
  border: 1px solid rgba(120, 200, 150, 0.45);
  border-radius: 6px;
  color: #d8ffe6;
  font: 13px/1.55 ui-monospace, 'DejaVu Sans Mono', monospace;
  text-shadow: 0 1px 3px rgba(0, 0, 0, 0.9);
}
.hfs-menu-head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  border-bottom: 1px solid rgba(120, 200, 150, 0.3);
  padding-bottom: 10px;
  margin-bottom: 10px;
}
.hfs-menu-title {
  font-size: 16px;
  font-weight: 700;
  letter-spacing: 0.14em;
}
.hfs-menu-actions { display: flex; gap: 8px; flex-wrap: wrap; }
.hfs-menu-button {
  pointer-events: auto;
  touch-action: manipulation;
  -webkit-tap-highlight-color: transparent;
  cursor: pointer;
  padding: 7px 12px;
  border-radius: 4px;
  background: rgba(120, 200, 150, 0.16);
  border: 1px solid rgba(120, 200, 150, 0.5);
  color: #d8ffe6;
  font: inherit;
  font-size: 12px;
  letter-spacing: 0.08em;
  white-space: nowrap;
}
.hfs-menu-button:hover { background: rgba(120, 200, 150, 0.3); }
.hfs-menu-group {
  margin-top: 14px;
  color: #79b795;
  text-transform: uppercase;
  letter-spacing: 0.16em;
  font-size: 11px;
}
.hfs-menu-header,
.hfs-menu-row {
  display: grid;
  grid-template-columns: minmax(0, 1.5fr) minmax(0, 1fr) minmax(0, 1.15fr) minmax(0, 0.9fr);
  gap: 6px 12px;
  align-items: start;
  padding: 5px 0;
}
.hfs-menu-header {
  color: #79b795;
  font-size: 11px;
  text-transform: uppercase;
  letter-spacing: 0.12em;
  border-bottom: 1px solid rgba(120, 200, 150, 0.2);
}
.hfs-menu-row { border-bottom: 1px solid rgba(120, 200, 150, 0.1); }
.hfs-menu-cell { display: flex; flex-wrap: wrap; gap: 4px; }
.hfs-key {
  display: inline-block;
  padding: 1px 7px;
  border-radius: 3px;
  background: rgba(120, 200, 150, 0.14);
  border: 1px solid rgba(120, 200, 150, 0.4);
  font-size: 12px;
  white-space: nowrap;
}
.hfs-menu-none { color: #4c6b5c; }
.hfs-menu-note {
  grid-column: 1 / -1;
  color: #ffc857;
  font-size: 11px;
}
.hfs-menu-extra {
  margin-top: 14px;
  padding-top: 10px;
  border-top: 1px solid rgba(120, 200, 150, 0.3);
  color: #a8d8bd;
  font-size: 12px;
}
.hfs-menu-extra p { margin: 0 0 6px; }

@media (max-width: 760px) {
  .hfs-menu-header { display: none; }
  .hfs-menu-row {
    grid-template-columns: 1fr;
    padding: 8px 0;
  }
  .hfs-menu-cell::before {
    color: #79b795;
    font-size: 11px;
    letter-spacing: 0.1em;
    margin-right: 4px;
    align-self: center;
  }
  .hfs-menu-cell.keyboard::before { content: 'KEYS'; }
  .hfs-menu-cell.gamepad::before { content: 'PAD'; }
  .hfs-menu-cell.touch::before { content: 'TOUCH'; }
  .hfs-menu-panel { font-size: 12px; padding: 12px; }
}
`;

/** Add the style sheet one time, whatever the number of menus. */
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

/** Fills one device cell with a chip for each control, or with a dash. */
function fillCell(cell: HTMLDivElement, chips: readonly string[]): void {
  if (chips.length === 0) {
    const none = document.createElement('span');
    none.className = 'hfs-menu-none';
    none.textContent = '-';
    cell.appendChild(none);
    return;
  }
  for (const text of chips) {
    const chip = document.createElement('span');
    chip.className = 'hfs-key';
    chip.textContent = text;
    cell.appendChild(chip);
  }
}

export interface ControlsMenuOptions {
  /** Reports whether the on screen pad is on now. */
  touchVisible(): boolean;
  /** Turns the on screen pad on or off. The menu button calls it. */
  setTouchVisible(value: boolean): void;
  /** The map to print. DEFAULT_BINDINGS when this is absent. */
  bindings?: readonly Binding[];
}

/**
 * Builds the menu and attaches it to `parent`.
 *
 * The menu starts closed. It builds every row one time, because a binding table
 * does not change while the simulator runs. The pad button is the one part that
 * changes, and it rewrites one word.
 *
 * The menu does NOT stop the simulator. The aircraft keeps flying while the
 * panel is open, so a pilot who opens it in a turn comes back to the turn.
 */
export function createControlsMenu(
  parent: HTMLElement,
  options: ControlsMenuOptions,
): ControlsMenu {
  ensureStyle();

  // The button that opens the panel. See the style sheet for why it exists.
  const openButton = document.createElement('button');
  openButton.type = 'button';
  openButton.className = 'hfs-menu-open';
  openButton.textContent = 'CONTROLS';
  openButton.title = 'Open the controls menu. The H key does the same.';
  parent.appendChild(openButton);

  const scrim = makeDiv('hfs-menu-scrim', parent);
  const panel = makeDiv('hfs-menu-panel', scrim);

  const head = makeDiv('hfs-menu-head', panel);
  const title = makeDiv('hfs-menu-title', head);
  title.textContent = 'CONTROLS';
  const actions = makeDiv('hfs-menu-actions', head);

  const padButton = document.createElement('button');
  padButton.type = 'button';
  padButton.className = 'hfs-menu-button';
  actions.appendChild(padButton);

  const closeButton = document.createElement('button');
  closeButton.type = 'button';
  closeButton.className = 'hfs-menu-button';
  closeButton.textContent = 'CLOSE';
  actions.appendChild(closeButton);

  function refreshPadButton(): void {
    padButton.textContent = options.touchVisible() ? 'TOUCH PAD: ON' : 'TOUCH PAD: OFF';
  }
  refreshPadButton();

  // --- The list ------------------------------------------------------------
  const rows = buildRows(options.bindings);
  let lastGroup: ActionGroup | null = null;
  for (const row of rows) {
    if (row.group !== lastGroup) {
      lastGroup = row.group;
      const groupTitle = makeDiv('hfs-menu-group', panel);
      groupTitle.textContent = GROUP_TITLES[row.group];
      const header = makeDiv('hfs-menu-header', panel);
      for (const text of ['Action', 'Keyboard', 'Gamepad', 'Touch']) {
        makeDiv('', header).textContent = text;
      }
    }

    const element = makeDiv('hfs-menu-row', panel);
    makeDiv('hfs-menu-action', element).textContent = row.label;
    fillCell(makeDiv('hfs-menu-cell keyboard', element), row.keyboard);
    fillCell(makeDiv('hfs-menu-cell gamepad', element), row.gamepad);
    fillCell(makeDiv('hfs-menu-cell touch', element), row.touch);
    if (row.note !== '') makeDiv('hfs-menu-note', element).textContent = row.note;
  }

  const extra = makeDiv('hfs-menu-extra', panel);
  for (const line of EXTRA_NOTES) {
    const p = document.createElement('p');
    p.textContent = line;
    extra.appendChild(p);
  }

  // --- The wiring ----------------------------------------------------------
  let visible = false;

  function setVisible(value: boolean): void {
    if (value === visible) return;
    visible = value;
    scrim.classList.toggle('open', visible);
    // The scrim covers the button, and a tap that landed on it would reach the
    // scrim and shut the panel. That reads as a button that does the opposite
    // of what it says, so the button leaves the screen while the panel is open.
    openButton.style.display = visible ? 'none' : '';
    if (visible) {
      refreshPadButton();
      scrim.scrollTop = 0;
    }
  }

  // A button that keeps the focus takes the next Space or Enter, and Space is
  // the cannon. Every handler therefore drops the focus before it returns.
  const onPad = (event: Event): void => {
    event.preventDefault();
    padButton.blur();
    options.setTouchVisible(!options.touchVisible());
    refreshPadButton();
  };
  const onClose = (event: Event): void => {
    event.preventDefault();
    closeButton.blur();
    setVisible(false);
  };
  const onOpen = (event: Event): void => {
    event.preventDefault();
    openButton.blur();
    setVisible(true);
  };
  // A tap on the scrim closes the panel. A tap INSIDE the panel must not, so
  // the test reads the target and not the currentTarget.
  const onScrim = (event: Event): void => {
    if (event.target === scrim) setVisible(false);
  };

  padButton.addEventListener('click', onPad);
  closeButton.addEventListener('click', onClose);
  openButton.addEventListener('click', onOpen);
  scrim.addEventListener('click', onScrim);

  return {
    get visible(): boolean {
      return visible;
    },

    set visible(value: boolean) {
      setVisible(value);
    },

    toggle(): void {
      setVisible(!visible);
    },

    dispose(): void {
      padButton.removeEventListener('click', onPad);
      closeButton.removeEventListener('click', onClose);
      openButton.removeEventListener('click', onOpen);
      scrim.removeEventListener('click', onScrim);
      openButton.remove();
      scrim.remove();
    },
  };
}
