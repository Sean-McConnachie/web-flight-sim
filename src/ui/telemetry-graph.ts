/**
 * Rolling strip chart of the telemetry.
 *
 * A single number tells you what the aircraft does now. It does not tell you
 * what the aircraft is about to do. A stall that develops, a phugoid that
 * builds, and a control loop that starts to ring all look the same in one
 * frame. They differ only in their shape over a few seconds. This chart shows
 * that shape.
 *
 * Each channel gets its own row with its own scale, so a value of 0.2 g and a
 * value of 700 km/h both fill their row. The current value prints on the row,
 * because a shape with no number cannot be compared against a reference.
 *
 * NO ALLOCATION ON THE DATA PATH
 *
 * The chart holds every channel in a ring buffer that it makes one time. A push
 * writes one number into a Float64Array and moves an index. A draw reads the
 * array and calls the canvas. Neither path makes an array, an object, or a
 * string. The value labels are the one exception, because canvas text needs a
 * string. The chart rebuilds a label 8 times per second and keeps it in
 * between. A label that changes 60 times per second is unreadable anyway.
 *
 * This file may touch the DOM. CONVENTIONS section 4 allows that under
 * `src/ui`. It holds no physics.
 */

import { msToKmh, toDeg } from '@/math/units';
import type { TelemetrySample } from './debug-overlay';

/** One traced value. `get` returns the value in the unit that `name` states. */
export interface GraphChannel {
  name: string;
  color: string;
  get(s: TelemetrySample): number;
  /**
   * True when zero has a meaning for this channel. The row then always holds
   * zero inside its range and draws a line there, so the sign is readable.
   */
  zeroLine?: boolean;
  /** Printed decimal places of the current value. Two by default. */
  decimals?: number;
}

export interface TelemetryGraph {
  /** Record one sample and redraw. Call it from the render callback. */
  push(s: TelemetrySample, time: number): void;
  /** Set to false to hide the chart. The chart then only records. */
  visible: boolean;
  dispose(): void;
}

/** A fixed size queue of numbers over a Float64Array that never grows. */
export interface RingBuffer {
  readonly capacity: number;
  /** Number of samples held, from 0 to the capacity. */
  readonly length: number;
  /** The backing store. It is the same object for the life of the buffer. */
  readonly values: Float64Array;
  /** Write one sample. The oldest sample falls out when the buffer is full. */
  push(v: number): void;
  /** Read a sample. Index 0 is the oldest one held. */
  at(i: number): number;
  clear(): void;
}

/**
 * Make a ring buffer.
 *
 * The buffer allocates its store and its own object one time, in this call.
 * Nothing after this call allocates. That matters because the chart pushes
 * every frame for the whole session, and a garbage collection pause shows up
 * as a stutter in the picture it draws.
 */
export function createRingBuffer(capacity: number): RingBuffer {
  if (!Number.isInteger(capacity) || capacity <= 0) {
    throw new Error(`createRingBuffer needs a positive whole capacity. It got ${capacity}.`);
  }
  const values = new Float64Array(capacity);
  let count = 0; // total pushes, which can pass the capacity
  let head = 0; // index of the next write

  return {
    capacity,
    values,

    get length(): number {
      return count < capacity ? count : capacity;
    },

    push(v: number): void {
      values[head] = v;
      head = head + 1 === capacity ? 0 : head + 1;
      count += 1;
    },

    at(i: number): number {
      const held = count < capacity ? count : capacity;
      if (i < 0 || i >= held) return Number.NaN;
      // The oldest sample sits at index 0 while the buffer fills, and at the
      // write head after it wraps.
      const start = count <= capacity ? 0 : head;
      let index = start + i;
      if (index >= capacity) index -= capacity;
      return values[index];
    },

    clear(): void {
      count = 0;
      head = 0;
      values.fill(0);
    },
  };
}

/** Seconds of history the chart shows. */
export const WINDOW_SECONDS = 10;

/**
 * Largest rate the chart records at. A display faster than this rate pushes
 * more often, and the extra samples add nothing that 10 s of history can show.
 */
const MAX_SAMPLE_HZ = 120;

/** Samples per channel. Ten seconds at the largest rate. */
const CAPACITY = WINDOW_SECONDS * MAX_SAMPLE_HZ;

/** Rate the value labels rebuild at. A faster label cannot be read. */
const LABEL_HZ = 8;

/** Width of the chart, in CSS pixels. */
const WIDTH = 360;

/** Height of one channel row, in CSS pixels. */
const ROW_HEIGHT = 46;

/** Padding around the rows, in CSS pixels. */
const PAD = 4;

/** Fraction of the row that stays empty above and below the trace. */
const RANGE_MARGIN = 0.08;

/**
 * The default channels.
 *
 * Angle of attack and load factor are the pair that shows a stall. Airspeed and
 * the climb rate are the pair that shows a phugoid, because a phugoid trades
 * one against the other with a quarter period of lag.
 */
export const DEFAULT_CHANNELS: readonly GraphChannel[] = [
  { name: 'alpha deg', color: '#6cc5ff', get: (s) => toDeg(s.alpha), zeroLine: true, decimals: 2 },
  { name: 'load g', color: '#ffb454', get: (s) => s.loadFactor, zeroLine: true, decimals: 2 },
  { name: 'tas km/h', color: '#7fdd7f', get: (s) => msToKmh(s.trueAirspeed), decimals: 1 },
  {
    name: 'climb m/s',
    color: '#ff7f9c',
    // CONVENTIONS section 3.2: the world z axis points down, so the climb rate
    // is minus the z velocity.
    get: (s) => -s.state.velocity.z,
    zeroLine: true,
    decimals: 2,
  },
];

const STYLE_ID = 'hfs-telemetry-graph-style';

const CSS = `
.hfs-graph {
  position: absolute;
  left: 12px;
  bottom: 12px;
  background: rgba(11, 14, 18, 0.78);
  border: 1px solid #2a3340;
  border-radius: 4px;
  pointer-events: none;
  user-select: none;
}
`;

function ensureStyle(): void {
  if (document.getElementById(STYLE_ID) !== null) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = CSS;
  document.head.appendChild(style);
}

/** Per channel state. The chart makes one of these for each channel. */
interface ChannelState {
  readonly channel: GraphChannel;
  readonly buffer: RingBuffer;
  readonly decimals: number;
  label: string;
}

/**
 * Build the chart and attach it to `parent`.
 *
 * The chart records while it is hidden, so a person who turns it on sees the
 * ten seconds that led to the event, not an empty box.
 */
export function createTelemetryGraph(
  parent: HTMLElement,
  channels: readonly GraphChannel[] = DEFAULT_CHANNELS,
): TelemetryGraph {
  ensureStyle();

  const height = channels.length * ROW_HEIGHT + 2 * PAD;

  const canvas = document.createElement('canvas');
  canvas.className = 'hfs-graph';
  canvas.style.width = `${WIDTH}px`;
  canvas.style.height = `${height}px`;
  const ratio = window.devicePixelRatio > 0 ? window.devicePixelRatio : 1;
  canvas.width = Math.round(WIDTH * ratio);
  canvas.height = Math.round(height * ratio);
  parent.appendChild(canvas);

  const ctx = canvas.getContext('2d');
  if (ctx !== null) {
    // Draw in CSS pixels. The transform holds for the life of the canvas.
    ctx.scale(ratio, ratio);
    ctx.font = '10px ui-monospace, "DejaVu Sans Mono", monospace';
    ctx.lineJoin = 'round';
  }

  const times = createRingBuffer(CAPACITY);
  const states: ChannelState[] = [];
  for (const channel of channels) {
    const decimals = channel.decimals !== undefined ? channel.decimals : 2;
    states.push({
      channel,
      buffer: createRingBuffer(CAPACITY),
      decimals,
      label: (0).toFixed(decimals),
    });
  }

  let lastSampleTime = Number.NEGATIVE_INFINITY;
  let lastLabelTime = Number.NEGATIVE_INFINITY;
  let shownVisible = true;

  // Scratch for the auto scale. The draw allocates nothing.
  let rangeMin = 0;
  let rangeMax = 0;

  /**
   * Find the smallest and the largest value of one channel inside the window.
   * The result goes into rangeMin and rangeMax.
   *
   * A row with no span, such as a load factor that holds exactly 1 g, would
   * divide by zero. The function opens a span around the value instead, so the
   * trace draws as a flat line through the middle of the row.
   */
  function findRange(state: ChannelState, now: number, includeZero: boolean): void {
    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    const held = state.buffer.length;
    for (let i = 0; i < held; i++) {
      if (now - times.at(i) > WINDOW_SECONDS) continue;
      const v = state.buffer.at(i);
      if (!Number.isFinite(v)) continue;
      if (v < min) min = v;
      if (v > max) max = v;
    }
    if (min > max) {
      min = 0;
      max = 0;
    }
    if (includeZero) {
      if (min > 0) min = 0;
      if (max < 0) max = 0;
    }
    let span = max - min;
    if (!(span > 0)) {
      // Open a span that scales with the value, so a steady 700 km/h and a
      // steady 0.001 rad both draw as a line through the middle.
      const size = Math.abs(max);
      span = size > 1e-9 ? size * 0.1 : 1;
      min -= span / 2;
      max += span / 2;
    } else {
      const margin = span * RANGE_MARGIN;
      min -= margin;
      max += margin;
    }
    rangeMin = min;
    rangeMax = max;
  }

  function draw(now: number): void {
    if (ctx === null) return;

    ctx.clearRect(0, 0, WIDTH, height);

    for (let row = 0; row < states.length; row++) {
      const state = states[row];
      const top = PAD + row * ROW_HEIGHT;
      const bottom = top + ROW_HEIGHT - 2;
      const inner = bottom - top;
      const zeroLine = state.channel.zeroLine === true;

      findRange(state, now, zeroLine);
      const span = rangeMax - rangeMin;
      const scale = span > 0 ? inner / span : 0;

      // Row frame.
      ctx.strokeStyle = 'rgba(42, 51, 64, 0.9)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, bottom + 1.5);
      ctx.lineTo(WIDTH, bottom + 1.5);
      ctx.stroke();

      if (zeroLine && rangeMin <= 0 && rangeMax >= 0) {
        const y = bottom - (0 - rangeMin) * scale;
        ctx.strokeStyle = 'rgba(120, 140, 165, 0.5)';
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(WIDTH, y);
        ctx.stroke();
      }

      // Trace. The newest sample sits at the right edge.
      ctx.strokeStyle = state.channel.color;
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      let started = false;
      const held = state.buffer.length;
      for (let i = 0; i < held; i++) {
        const age = now - times.at(i);
        if (age > WINDOW_SECONDS || age < 0) continue;
        const v = state.buffer.at(i);
        if (!Number.isFinite(v)) continue;
        const x = WIDTH * (1 - age / WINDOW_SECONDS);
        let y = bottom - (v - rangeMin) * scale;
        if (y < top) y = top;
        else if (y > bottom) y = bottom;
        if (started) ctx.lineTo(x, y);
        else {
          ctx.moveTo(x, y);
          started = true;
        }
      }
      if (started) ctx.stroke();

      ctx.fillStyle = '#8fa6bd';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillText(state.channel.name, 4, top + 2);

      ctx.fillStyle = state.channel.color;
      ctx.textAlign = 'right';
      ctx.fillText(state.label, WIDTH - 4, top + 2);
    }
  }

  const api: TelemetryGraph = {
    visible: true,

    push(s: TelemetrySample, time: number): void {
      if (api.visible !== shownVisible) {
        shownVisible = api.visible;
        canvas.style.display = shownVisible ? '' : 'none';
      }

      // Throttle the record rate. A frame that arrives faster than the record
      // rate adds no shape that ten seconds of history can hold.
      if (time - lastSampleTime < 1 / MAX_SAMPLE_HZ) return;
      lastSampleTime = time;

      times.push(time);
      for (const state of states) {
        state.buffer.push(state.channel.get(s));
      }

      if (!shownVisible) return;

      if (time - lastLabelTime >= 1 / LABEL_HZ) {
        lastLabelTime = time;
        for (const state of states) {
          const held = state.buffer.length;
          const current = held > 0 ? state.buffer.at(held - 1) : 0;
          state.label = current.toFixed(state.decimals);
        }
      }

      draw(time);
    },

    dispose(): void {
      canvas.remove();
      // The style sheet stays. A second chart may still use it.
    },
  };

  return api;
}
