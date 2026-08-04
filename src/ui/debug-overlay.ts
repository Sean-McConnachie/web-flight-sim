/**
 * Debug and telemetry overlay.
 *
 * This panel is a development instrument. Its job is to make a wrong flight
 * model visible. Every value it shows comes straight from the physics, in a
 * display unit, with no smoothing and no rounding beyond the printed digits.
 *
 * The panel reads one plain sample object. It never reaches into the physics,
 * the loop, or the renderer. The composition root fills the sample and calls
 * `update` one time per frame. That keeps this file free of any dependency on
 * how the aircraft is built.
 *
 * LAYOUT RULE
 *
 * A number that changes width makes the layout jump, and a jumping layout is
 * unreadable in flight. Three rules hold the columns still:
 *
 *   1. `fixedWidth` pads every value to a constant character count.
 *   2. The value cell uses `white-space: pre`, so the pad spaces survive.
 *   3. The row is a CSS grid with fixed column widths, in a monospace font
 *      with tabular figures.
 *
 * UNITS
 *
 * CONVENTIONS section 2 says the model holds SI units and only `src/ui`
 * converts. Every conversion here goes through `src/math/units.ts`. The read
 * function of each field returns the display value, not the SI value, so the
 * unit test can check the conversion without any DOM.
 *
 * This file may touch the DOM. CONVENTIONS section 4 allows that under
 * `src/ui`. It holds no physics.
 */

import type { Quaternion } from 'three';

import type { LoopStats } from '@/core/loop';
import type { AtmosphereSample } from '@/physics/atmosphere';
import type { RigidBodyState } from '@/physics/rigidbody';
import { kelvinToCelsius, msToKmh, paToHpa, toDeg } from '@/math/units';

/**
 * One frame of telemetry. The caller fills it. Every field holds an SI value,
 * except `loadFactor`, which is already a multiple of standard gravity.
 */
export interface TelemetrySample {
  loop: LoopStats;
  state: RigidBodyState;
  /** Angle of attack of the whole aircraft, rad. */
  alpha: number;
  /** Sideslip angle, rad. */
  beta: number;
  /** Load factor, in multiples of standard gravity. */
  loadFactor: number;
  /** True airspeed, m/s. */
  trueAirspeed: number;
  /** Equivalent airspeed, m/s. */
  equivalentAirspeed: number;
  mach: number;
  /** Dynamic pressure, Pa. */
  dynamicPressure: number;
  atmosphere: AtmosphereSample;
}

export interface DebugOverlay {
  /** Copy one sample into the panel. Call it one time per frame. */
  update(s: TelemetrySample): void;
  /** Set to false to hide the panel. The panel then does no work. */
  visible: boolean;
  dispose(): void;
}

/** Roll, pitch and heading of a NED attitude, in radians. */
export interface AttitudeAngles {
  /** Positive puts the right wing down. */
  roll: number;
  /** Positive raises the nose. */
  pitch: number;
  /** From north, through east, in the range 0 to 2 pi. */
  heading: number;
}

/** One printed line of the panel. `read` returns the value in display units. */
export interface OverlayField {
  readonly label: string;
  readonly unit: string;
  readonly decimals: number;
  read(s: TelemetrySample): number;
}

/** One labeled block of lines. */
export interface OverlayGroup {
  readonly title: string;
  readonly fields: readonly OverlayField[];
}

/** Character count of every value cell. It holds 999999.99 and a sign. */
export const VALUE_WIDTH = 10;

/**
 * Pad a number to a constant character count.
 *
 * The result is always `width` characters long, whatever the value, so a column
 * of these strings never changes width. The function first tries `toFixed`. A
 * value too large for the column loses decimal places one at a time, because
 * the integer part carries more meaning than the fraction. A value that still
 * does not fit prints an overflow mark instead of a wrong number.
 *
 * A value such as -0.001 prints as "0.00" and not "-0.00". The minus sign on a
 * value that rounds to zero reads as a fault when it is not one.
 */
export function fixedWidth(value: number, width: number, decimals: number): string {
  let text: string;
  if (Number.isNaN(value)) {
    text = 'nan';
  } else if (!Number.isFinite(value)) {
    text = value > 0 ? 'inf' : '-inf';
  } else {
    text = value.toFixed(decimals);
    if (Number(text) === 0) {
      text = (0).toFixed(decimals);
    }
    for (let d = decimals - 1; d >= 0 && text.length > width; d--) {
      text = value.toFixed(d);
    }
    if (text.length > width) {
      text = value < 0 ? '-ovf' : '+ovf';
    }
  }
  if (text.length > width) {
    // Only a column narrower than four characters reaches this line.
    text = text.slice(text.length - width);
  }
  return text.padStart(width, ' ');
}

/**
 * Read roll, pitch and heading from a NED attitude quaternion.
 *
 * The quaternion turns a body vector into a world NED vector, as
 * CONVENTIONS section 3 defines it. The angles follow the aerospace 3-2-1
 * sequence, so the attitude is heading about down, then pitch about the right
 * wing, then roll about the nose.
 *
 * The function writes into `out` and allocates nothing.
 */
export function attitudeAngles(q: Quaternion, out: AttitudeAngles): AttitudeAngles {
  const x = q.x;
  const y = q.y;
  const z = q.z;
  const w = q.w;

  out.roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

  // The clamp holds the argument of asin inside its range against rounding
  // error. Straight up and straight down both put the argument at the limit.
  let sinPitch = 2 * (w * y - z * x);
  if (sinPitch > 1) sinPitch = 1;
  else if (sinPitch < -1) sinPitch = -1;
  out.pitch = Math.asin(sinPitch);

  let heading = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  // A compass reads 0 to 360, so wrap the atan2 result out of its -pi to pi
  // range. North then prints as 0 and not as 360.
  if (heading < 0) heading += 2 * Math.PI;
  if (heading >= 2 * Math.PI) heading = 0;
  out.heading = heading;

  return out;
}

/** Scratch for the attitude rows. The read functions allocate nothing. */
const attitude: AttitudeAngles = { roll: 0, pitch: 0, heading: 0 };

/** Ground speed is the horizontal part of the world velocity, m/s. */
function groundSpeed(s: TelemetrySample): number {
  const v = s.state.velocity;
  return Math.hypot(v.x, v.y);
}

/**
 * Every printed line, in order.
 *
 * The `read` function of a field returns the display value. The unit test
 * drives this table with a made up sample, so it checks the conversions and the
 * signs with no DOM at all.
 */
export const OVERLAY_GROUPS: readonly OverlayGroup[] = [
  {
    title: 'loop',
    fields: [
      { label: 'fps', unit: '', decimals: 1, read: (s) => s.loop.fps },
      { label: 'steps', unit: '', decimals: 0, read: (s) => s.loop.physicsStepsLastFrame },
      { label: 'dropped', unit: 's', decimals: 3, read: (s) => s.loop.droppedTime },
      { label: 'fixed', unit: 'ms', decimals: 2, read: (s) => s.loop.fixedUpdateMs },
      { label: 'render', unit: 'ms', decimals: 2, read: (s) => s.loop.renderMs },
      { label: 'sim time', unit: 's', decimals: 1, read: (s) => s.loop.simTime },
    ],
  },
  {
    title: 'state',
    fields: [
      { label: 'north', unit: 'm', decimals: 1, read: (s) => s.state.position.x },
      { label: 'east', unit: 'm', decimals: 1, read: (s) => s.state.position.y },
      // CONVENTIONS section 3.2: altitude is minus the NED z, never plus.
      { label: 'altitude', unit: 'm', decimals: 1, read: (s) => -s.state.position.z },
      { label: 'v north', unit: 'm/s', decimals: 2, read: (s) => s.state.velocity.x },
      { label: 'v east', unit: 'm/s', decimals: 2, read: (s) => s.state.velocity.y },
      { label: 'climb', unit: 'm/s', decimals: 2, read: (s) => -s.state.velocity.z },
      { label: 'ground', unit: 'km/h', decimals: 1, read: (s) => msToKmh(groundSpeed(s)) },
      { label: 'tas', unit: 'km/h', decimals: 1, read: (s) => msToKmh(s.trueAirspeed) },
      { label: 'tas', unit: 'm/s', decimals: 2, read: (s) => s.trueAirspeed },
      { label: 'eas', unit: 'km/h', decimals: 1, read: (s) => msToKmh(s.equivalentAirspeed) },
      { label: 'mach', unit: '', decimals: 3, read: (s) => s.mach },
    ],
  },
  {
    title: 'attitude',
    fields: [
      {
        label: 'roll',
        unit: 'deg',
        decimals: 1,
        read: (s) => toDeg(attitudeAngles(s.state.orientation, attitude).roll),
      },
      {
        label: 'pitch',
        unit: 'deg',
        decimals: 1,
        read: (s) => toDeg(attitudeAngles(s.state.orientation, attitude).pitch),
      },
      {
        label: 'heading',
        unit: 'deg',
        decimals: 1,
        read: (s) => toDeg(attitudeAngles(s.state.orientation, attitude).heading),
      },
      { label: 'roll rate', unit: 'deg/s', decimals: 1, read: (s) => toDeg(s.state.angularVelocity.x) },
      { label: 'pitch rate', unit: 'deg/s', decimals: 1, read: (s) => toDeg(s.state.angularVelocity.y) },
      { label: 'yaw rate', unit: 'deg/s', decimals: 1, read: (s) => toDeg(s.state.angularVelocity.z) },
    ],
  },
  {
    title: 'flow',
    fields: [
      { label: 'alpha', unit: 'deg', decimals: 2, read: (s) => toDeg(s.alpha) },
      { label: 'beta', unit: 'deg', decimals: 2, read: (s) => toDeg(s.beta) },
      { label: 'load', unit: 'g', decimals: 2, read: (s) => s.loadFactor },
      { label: 'q bar', unit: 'Pa', decimals: 0, read: (s) => s.dynamicPressure },
    ],
  },
  {
    title: 'air',
    fields: [
      { label: 'density', unit: 'kg/m3', decimals: 4, read: (s) => s.atmosphere.density },
      {
        label: 'temp',
        unit: 'C',
        decimals: 1,
        read: (s) => kelvinToCelsius(s.atmosphere.temperature),
      },
      { label: 'pressure', unit: 'hPa', decimals: 1, read: (s) => paToHpa(s.atmosphere.pressure) },
      { label: 'sound', unit: 'm/s', decimals: 1, read: (s) => s.atmosphere.speedOfSound },
    ],
  },
];

/**
 * Find one field by its group title and its label. The unit test uses it. Two
 * fields share the label "tas", so the unit tells them apart.
 */
export function findOverlayField(
  group: string,
  label: string,
  unit?: string,
): OverlayField | undefined {
  for (const g of OVERLAY_GROUPS) {
    if (g.title !== group) continue;
    for (const field of g.fields) {
      if (field.label !== label) continue;
      if (unit !== undefined && field.unit !== unit) continue;
      return field;
    }
  }
  return undefined;
}

const STYLE_ID = 'hfs-debug-overlay-style';

const CSS = `
.hfs-dbg {
  position: absolute;
  top: 12px;
  left: 12px;
  padding: 8px 10px;
  box-sizing: border-box;
  background: rgba(11, 14, 18, 0.78);
  border: 1px solid #2a3340;
  border-radius: 4px;
  color: #dfe6ee;
  font: 11px/1.45 ui-monospace, 'DejaVu Sans Mono', monospace;
  font-variant-numeric: tabular-nums;
  pointer-events: none;
  user-select: none;
}
.hfs-dbg-title {
  margin-top: 6px;
  font-weight: 700;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  color: #8fa6bd;
}
.hfs-dbg-title:first-child { margin-top: 0; }
.hfs-dbg-row {
  display: grid;
  grid-template-columns: 78px 78px 44px;
  align-items: baseline;
}
.hfs-dbg-label { color: #a9bccf; }
.hfs-dbg-value {
  text-align: right;
  white-space: pre;
  color: #e8f0f8;
}
.hfs-dbg-unit {
  padding-left: 6px;
  color: #6f8299;
}
`;

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

/** One built row. `last` holds the printed text, so an equal value writes nothing. */
interface Row {
  readonly field: OverlayField;
  readonly cell: HTMLDivElement;
  last: string;
}

/**
 * Build the panel and attach it to `parent`.
 *
 * The panel starts visible. A later key binding sets `visible`. The panel does
 * no work while it is hidden.
 */
export function createDebugOverlay(parent: HTMLElement): DebugOverlay {
  ensureStyle();

  const root = makeDiv('hfs-dbg', parent);
  const rows: Row[] = [];

  for (const group of OVERLAY_GROUPS) {
    const title = makeDiv('hfs-dbg-title', root);
    title.textContent = group.title;

    for (const field of group.fields) {
      const row = makeDiv('hfs-dbg-row', root);

      const label = makeDiv('hfs-dbg-label', row);
      label.textContent = field.label;

      const cell = makeDiv('hfs-dbg-value', row);
      const blank = fixedWidth(0, VALUE_WIDTH, field.decimals);
      cell.textContent = blank;

      const unit = makeDiv('hfs-dbg-unit', row);
      unit.textContent = field.unit;

      rows.push({ field, cell, last: blank });
    }
  }

  let shownVisible = true;

  const api: DebugOverlay = {
    visible: true,

    update(s: TelemetrySample): void {
      if (api.visible !== shownVisible) {
        shownVisible = api.visible;
        root.style.display = shownVisible ? '' : 'none';
      }
      if (!shownVisible) return;

      for (const row of rows) {
        const text = fixedWidth(row.field.read(s), VALUE_WIDTH, row.field.decimals);
        // A write to textContent costs a layout pass. Most values change every
        // frame, but the loop counters do not, so the test is worth its cost.
        if (text !== row.last) {
          row.cell.textContent = text;
          row.last = text;
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
