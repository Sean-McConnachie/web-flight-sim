/**
 * Controller debug panel.
 *
 * The panel shows every axis as a bar and every button as a lit box, so a
 * person can check the controller mapping by eye. Push a stick, watch the bar
 * that moves, and read the name beside it.
 *
 * Each axis row shows two numbers. The left number is the shaped value that the
 * simulator uses. The right number, in brackets, is the raw hardware value
 * before the dead zone and the curve. The pair makes the dead zone visible.
 *
 * A stick bar grows out of the center, because a stick reads -1 to 1. A trigger
 * bar grows from the left edge, because a trigger reads 0 to 1.
 *
 * The panel is plain DOM and CSS. It uses no framework.
 */

import type { AxisName, ButtonName, GamepadState } from '@/input/gamepad';
import { AXIS_NAMES, BUTTON_NAMES, isTriggerAxis } from '@/input/gamepad';

export interface InputDebugPanel {
  /** Copy the reader values into the panel. Call it one time per frame. */
  update(): void;
  dispose(): void;
  /** Set to false to hide the panel. The panel then does no work. */
  visible: boolean;
}

const STYLE_ID = 'hfs-input-debug-style';

const CSS = `
.hfs-idbg {
  position: absolute;
  top: 12px;
  right: 12px;
  width: 260px;
  padding: 10px 12px;
  box-sizing: border-box;
  background: rgba(11, 14, 18, 0.82);
  border: 1px solid #2a3340;
  border-radius: 4px;
  color: #dfe6ee;
  font: 11px/1.5 ui-monospace, 'DejaVu Sans Mono', monospace;
  pointer-events: none;
  user-select: none;
}
.hfs-idbg-title {
  font-weight: 700;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  color: #8fa6bd;
  margin-bottom: 2px;
}
.hfs-idbg-id {
  color: #6f8299;
  margin-bottom: 8px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.hfs-idbg-id.hfs-idbg-off { color: #b4603c; }
.hfs-idbg-row {
  display: grid;
  grid-template-columns: 74px 1fr;
  align-items: center;
  gap: 6px;
  margin-bottom: 3px;
}
.hfs-idbg-name { color: #a9bccf; }
.hfs-idbg-track {
  position: relative;
  height: 9px;
  background: #161c24;
  border: 1px solid #2a3340;
  overflow: hidden;
}
.hfs-idbg-center {
  position: absolute;
  top: 0;
  bottom: 0;
  left: 50%;
  width: 1px;
  background: #38455a;
}
.hfs-idbg-fill {
  position: absolute;
  top: 0;
  bottom: 0;
  left: 50%;
  width: 0;
  background: #4f9dd6;
}
.hfs-idbg-value {
  grid-column: 1 / -1;
  text-align: right;
  color: #6f8299;
  margin-top: -3px;
  margin-bottom: 2px;
}
.hfs-idbg-buttons {
  display: grid;
  grid-template-columns: repeat(5, 1fr);
  gap: 3px;
  margin-top: 8px;
}
.hfs-idbg-btn {
  padding: 3px 0;
  text-align: center;
  font-size: 9px;
  background: #161c24;
  border: 1px solid #2a3340;
  color: #6f8299;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.hfs-idbg-btn.hfs-idbg-on {
  background: #4f9dd6;
  border-color: #7cc0ef;
  color: #0b0e12;
}
`;

/** Short label for a button box. The full name does not fit in the grid. */
const BUTTON_LABEL: Readonly<Record<ButtonName, string>> = {
  a: 'A',
  b: 'B',
  x: 'X',
  y: 'Y',
  leftBumper: 'LB',
  rightBumper: 'RB',
  back: 'BACK',
  start: 'STRT',
  leftStick: 'LS',
  rightStick: 'RS',
  dpadUp: 'UP',
  dpadDown: 'DOWN',
  dpadLeft: 'LEFT',
  dpadRight: 'RGHT',
  guide: 'GUID',
};

interface AxisRow {
  readonly name: AxisName;
  readonly trigger: boolean;
  readonly fill: HTMLDivElement;
  readonly value: HTMLDivElement;
  lastText: string;
  lastLeft: string;
  lastWidth: string;
}

interface ButtonBox {
  readonly name: ButtonName;
  readonly box: HTMLDivElement;
  lastOn: boolean;
}

/** Add the style sheet one time, whatever the number of panels. */
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

/**
 * Build the panel and attach it to `parent`. The panel reads the state. It
 * never polls, so the caller must call `reader.poll()` first.
 */
export function createInputDebugPanel(parent: HTMLElement, reader: GamepadState): InputDebugPanel {
  ensureStyle();

  const root = makeDiv('hfs-idbg', parent);

  const title = makeDiv('hfs-idbg-title', root);
  title.textContent = 'gamepad';

  const idLine = makeDiv('hfs-idbg-id', root);
  idLine.textContent = 'no controller';
  idLine.classList.add('hfs-idbg-off');

  const rows: AxisRow[] = [];
  for (const name of AXIS_NAMES) {
    const row = makeDiv('hfs-idbg-row', root);

    const label = makeDiv('hfs-idbg-name', row);
    label.textContent = name;

    const track = makeDiv('hfs-idbg-track', row);
    const trigger = isTriggerAxis(name);
    if (!trigger) makeDiv('hfs-idbg-center', track);
    const fill = makeDiv('hfs-idbg-fill', track);
    if (trigger) fill.style.left = '0%';

    const value = makeDiv('hfs-idbg-value', row);
    value.textContent = '0.00 [0.00]';

    rows.push({
      name,
      trigger,
      fill,
      value,
      lastText: '0.00 [0.00]',
      lastLeft: '',
      lastWidth: '',
    });
  }

  const grid = makeDiv('hfs-idbg-buttons', root);
  const boxes: ButtonBox[] = [];
  for (const name of BUTTON_NAMES) {
    const box = makeDiv('hfs-idbg-btn', grid);
    box.textContent = BUTTON_LABEL[name];
    box.title = name;
    boxes.push({ name, box, lastOn: false });
  }

  let shownVisible = true;

  const api: InputDebugPanel = {
    visible: true,

    update(): void {
      if (api.visible !== shownVisible) {
        shownVisible = api.visible;
        root.style.display = shownVisible ? '' : 'none';
      }
      if (!shownVisible) return;

      const idText = reader.connected ? reader.id : 'no controller';
      if (idLine.textContent !== idText) {
        idLine.textContent = idText;
        idLine.classList.toggle('hfs-idbg-off', !reader.connected);
      }

      for (const row of rows) {
        const shaped = reader.axis(row.name);
        const raw = reader.rawAxis(row.name);

        let left: string;
        let width: string;
        if (row.trigger) {
          // A trigger runs from 0 to 1, so the bar grows from the left edge.
          left = '0%';
          width = `${Math.min(Math.max(shaped, 0), 1) * 100}%`;
        } else {
          // A stick runs from -1 to 1, so the bar grows out of the center.
          const clamped = Math.min(Math.max(shaped, -1), 1);
          const half = Math.abs(clamped) * 50;
          left = `${clamped >= 0 ? 50 : 50 - half}%`;
          width = `${half}%`;
        }
        if (left !== row.lastLeft) {
          row.fill.style.left = left;
          row.lastLeft = left;
        }
        if (width !== row.lastWidth) {
          row.fill.style.width = width;
          row.lastWidth = width;
        }

        const text = `${shaped.toFixed(2)} [${raw.toFixed(2)}]`;
        if (text !== row.lastText) {
          row.value.textContent = text;
          row.lastText = text;
        }
      }

      for (const entry of boxes) {
        const on = reader.held(entry.name);
        if (on !== entry.lastOn) {
          entry.box.classList.toggle('hfs-idbg-on', on);
          entry.lastOn = on;
        }
      }
    },

    dispose(): void {
      root.remove();
      // The style sheet stays. A second panel may still use it, and an unused
      // style sheet costs nothing.
    },
  };

  return api;
}
