/**
 * The shared drawing helper for every dial face.
 *
 *
 * 1. DRAW ONCE, TURN A NEEDLE EVERY FRAME
 *
 * Every face on this panel is painted ONE time, at start up, onto a 2D canvas.
 * The canvas becomes a texture and the texture never changes again. Only the
 * needle meshes move.
 *
 * The split is the whole point. A face carries hundreds of tick marks, arcs
 * and numerals, and none of them move. A canvas redrawn every frame would cost
 * one texture upload per gauge per frame, which is fifteen uploads of up to
 * 512 by 512 pixels at the frame rate, for a picture that is the same picture.
 * A needle is one mesh and one number in a matrix.
 *
 *
 * 2. THE ANGLE CONVENTION
 *
 * Every angle here is CLOCKWISE from twelve o'clock, in radians, which is the
 * convention of src/ui/gauges/dial.ts. On the canvas that gives
 *
 *   x = cx + r * sin(angle)
 *   y = cy - r * cos(angle)
 *
 * because the canvas y axis points DOWN.
 *
 *
 * 3. HOW LARGE THE MARKINGS MUST BE
 *
 * The eye of the pilot sits 0.66 m from the middle of the panel, and an 80 mm
 * case gives a face 0.082 m across. That face fills 7.1 degrees of the view.
 * The camera uses a 55 degree vertical field, so on a 1080 line picture the
 * whole face is only about 140 pixels across. A numeral must reach 9 pixels to
 * stay readable, which is 6.5 percent of the face.
 *
 * Every default in DIAL_STYLE below is set from that number. The markings come
 * out a little bolder than a real 1944 dial, and they have to. A face drawn to
 * true scale is correct and unreadable, and an unreadable instrument has
 * failed.
 *
 *
 * 4. THE PLATFORM FAULT THAT MAKES EVERY FACE BLANK
 *
 * CONVENTIONS section 6a. Chrome with `--use-angle=vulkan` returns transparent
 * pixels from a GPU 2D canvas, so every `fillRect` and every `fillText` here
 * reads back as zero and every face comes out empty. The launch command in the
 * README carries `--disable-accelerated-2d-canvas` for that reason. Do not
 * drop that flag.
 *
 * This file touches the DOM. CONVENTIONS section 4 allows that under src/ui.
 */

import { CanvasTexture, LinearFilter, SRGBColorSpace } from 'three/webgpu';

import type { DialLaw } from './dial';
import { dialAngle } from './dial';

// ---------------------------------------------------------------------------
// The look of a Luftwaffe dial of 1944
// ---------------------------------------------------------------------------

/**
 * The colors of the panel.
 *
 * A German instrument of the period carries a matte black face with off white
 * markings, a red limit line, and small luminous marks that a radium paint
 * gave. The off white is warm, because the paint aged and because a pure white
 * on pure black reads as a modern instrument.
 */
export const DIAL_COLOR = {
  face: '#0b0d0e',
  faceEdge: '#161a1c',
  mark: '#e7e2d2',
  minor: '#a8a396',
  dim: '#7d7a70',
  red: '#c33a2c',
  amber: '#d9a324',
  luminous: '#9fd6a8',
  sky: '#4d6b86',
  ground: '#6d5334',
  horizon: '#efe9d8',
  pointer: '#e8b23c',
} as const;

/**
 * The size of every marking, as a fraction of the FACE RADIUS. Read section 3
 * before you make one of them smaller.
 */
export const DIAL_STYLE = {
  /** Length of a major tick. */
  majorTick: 0.17,
  /** Width of a major tick. */
  majorWidth: 0.045,
  /** Length of a minor tick. */
  minorTick: 0.09,
  /** Width of a minor tick. */
  minorWidth: 0.026,
  /** Where the outer end of every tick sits. */
  tickOuter: 0.97,
  /** Height of a scale numeral. */
  numeral: 0.20,
  /** Where the middle of a numeral sits. */
  numeralRadius: 0.66,
  /** Height of a name or a unit word. */
  caption: 0.135,
} as const;

/** The font of every marking. A condensed grotesque is what the period used. */
const DIAL_FONT = "'DejaVu Sans Condensed', 'Arial Narrow', 'DejaVu Sans', sans-serif";

// ---------------------------------------------------------------------------
// The canvas
// ---------------------------------------------------------------------------

/** A face under the brush. Every length below is in canvas pixels. */
export interface FaceCanvas {
  readonly canvas: HTMLCanvasElement;
  readonly ctx: CanvasRenderingContext2D;
  /** Width and height of the square canvas. */
  readonly size: number;
  /** Middle of the face. */
  readonly cx: number;
  readonly cy: number;
  /** Radius of the visible face. */
  readonly radius: number;
}

/** Pixels across one large face. An 80 mm case then holds 6.2 pixels per mm. */
export const LARGE_FACE_PIXELS = 512;

/** Pixels across one small face. A 57 mm case then holds 5.6 pixels per mm. */
export const SMALL_FACE_PIXELS = 320;

/**
 * Open a square canvas and hand it to `paint`.
 *
 * The face fills the canvas edge to edge, so the texture wastes no pixels. The
 * caller draws in canvas pixels and uses the helpers below for anything that
 * follows the dial law.
 */
export function drawFace(size: number, paint: (f: FaceCanvas) => void): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d');
  if (ctx === null) {
    throw new Error('The browser gave no 2D context for a gauge face.');
  }
  const face: FaceCanvas = {
    canvas,
    ctx,
    size,
    cx: size / 2,
    cy: size / 2,
    radius: size / 2,
  };
  ctx.lineCap = 'butt';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  paint(face);
  return canvas;
}

/**
 * Open a canvas that is not square and hand the plain context to `paint`.
 *
 * The drum of the artificial horizon needs one. Its texture wraps around the
 * drum on x and runs along the drum axis on y, and the two need very different
 * resolutions, so a square canvas would waste most of its pixels.
 */
export function drawStrip(
  width: number,
  height: number,
  paint: (ctx: CanvasRenderingContext2D, width: number, height: number) => void,
): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  if (ctx === null) {
    throw new Error('The browser gave no 2D context for a gauge drum.');
  }
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  paint(ctx, width, height);
  return canvas;
}

/**
 * Turn a painted canvas into a texture.
 *
 * The texture is color, so it holds the sRGB color space. Both filters are
 * linear and there is no mip chain, because the face is never seen small
 * enough to alias and a mip chain would blur the thin ticks.
 *
 * THE ANISOTROPY STAYS AT ONE, AND THAT IS NOT A CHOICE. WebGPU only allows
 * `maxAnisotropy` above 1 when the magnify filter, the minify filter AND the
 * mipmap filter are all linear. This texture carries no mip chain, so its
 * mipmap filter is nearest, and any anisotropy above 1 makes the sampler fail
 * validation. The bind group of that draw then goes bad.
 *
 * The fault it makes looks nothing like a sampler fault. Whole dial faces come
 * out in dark radial wedges, a DIFFERENT set of dials in every frame, which
 * reads as a texture that was lost or as depth fighting. It cost an hour.
 */
export function faceTexture(canvas: HTMLCanvasElement, name: string): CanvasTexture {
  const texture = new CanvasTexture(canvas);
  texture.name = name;
  texture.colorSpace = SRGBColorSpace;
  texture.minFilter = LinearFilter;
  texture.magFilter = LinearFilter;
  texture.generateMipmaps = false;
  texture.anisotropy = 1;
  texture.needsUpdate = true;
  return texture;
}

// ---------------------------------------------------------------------------
// The brush
// ---------------------------------------------------------------------------

/** Canvas x of one point, at a clockwise angle and a radius in pixels. */
export function pointX(f: FaceCanvas, angle: number, radius: number): number {
  return f.cx + radius * Math.sin(angle);
}

/** Canvas y of one point. The canvas y axis points DOWN, so the sign turns. */
export function pointY(f: FaceCanvas, angle: number, radius: number): number {
  return f.cy - radius * Math.cos(angle);
}

/**
 * Paint the black disc of the face.
 *
 * The disc carries a small radial shade, because a real dial sits behind glass
 * and the glass never lights the face evenly. Without it the face reads as a
 * flat sticker.
 */
export function fillFace(f: FaceCanvas): void {
  const { ctx } = f;
  const shade = ctx.createRadialGradient(
    f.cx,
    f.cy - f.radius * 0.25,
    f.radius * 0.1,
    f.cx,
    f.cy,
    f.radius,
  );
  shade.addColorStop(0, DIAL_COLOR.faceEdge);
  shade.addColorStop(1, DIAL_COLOR.face);
  ctx.fillStyle = shade;
  ctx.beginPath();
  ctx.arc(f.cx, f.cy, f.radius, 0, Math.PI * 2);
  ctx.fill();
}

/** One tick mark, at a clockwise angle. Every length is a fraction of the radius. */
export function tick(
  f: FaceCanvas,
  angle: number,
  outer: number,
  length: number,
  width: number,
  color: string,
): void {
  const { ctx } = f;
  const rOuter = outer * f.radius;
  const rInner = (outer - length) * f.radius;
  ctx.strokeStyle = color;
  ctx.lineWidth = width * f.radius;
  ctx.beginPath();
  ctx.moveTo(pointX(f, angle, rInner), pointY(f, angle, rInner));
  ctx.lineTo(pointX(f, angle, rOuter), pointY(f, angle, rOuter));
  ctx.stroke();
}

/** A row of ticks, one per value of the law. */
export function tickRow(
  f: FaceCanvas,
  law: DialLaw,
  values: readonly number[],
  length: number,
  width: number,
  color: string,
): void {
  for (const value of values) {
    tick(f, dialAngle(law, value), DIAL_STYLE.tickOuter, length, width, color);
  }
}

/** A row of numerals, one per value of the law. */
export function numeralRow(
  f: FaceCanvas,
  law: DialLaw,
  values: readonly number[],
  radius: number,
  height: number,
  color: string,
  print: (value: number) => string,
): void {
  const { ctx } = f;
  ctx.fillStyle = color;
  ctx.font = `600 ${Math.round(height * f.radius)}px ${DIAL_FONT}`;
  for (const value of values) {
    const angle = dialAngle(law, value);
    const r = radius * f.radius;
    ctx.fillText(print(value), pointX(f, angle, r), pointY(f, angle, r));
  }
}

/** A colored band along the scale, between two values of the law. */
export function band(
  f: FaceCanvas,
  law: DialLaw,
  from: number,
  to: number,
  radius: number,
  width: number,
  color: string,
): void {
  const { ctx } = f;
  const start = dialAngle(law, from);
  const end = dialAngle(law, to);
  // The canvas measures its arc anticlockwise from three o'clock, so a
  // clockwise angle from twelve o'clock arrives as the angle less a quarter turn.
  ctx.strokeStyle = color;
  ctx.lineWidth = width * f.radius;
  ctx.beginPath();
  ctx.arc(f.cx, f.cy, radius * f.radius, start - Math.PI / 2, end - Math.PI / 2, false);
  ctx.stroke();
}

/** A radial line across the scale, such as a red limit mark. */
export function limitLine(
  f: FaceCanvas,
  law: DialLaw,
  value: number,
  outer: number,
  length: number,
  width: number,
  color: string,
): void {
  tick(f, dialAngle(law, value), outer, length, width, color);
}

/**
 * A word on the face. `x` and `y` run from -1 to 1 over the face, with +y up.
 */
export function caption(
  f: FaceCanvas,
  text: string,
  x: number,
  y: number,
  height: number,
  color: string,
): void {
  const { ctx } = f;
  ctx.fillStyle = color;
  ctx.font = `500 ${Math.round(height * f.radius)}px ${DIAL_FONT}`;
  ctx.fillText(text, f.cx + x * f.radius, f.cy - y * f.radius);
}

/**
 * The small luminous dots that a radium paint gave.
 *
 * They sit inside the tick ring, so a pilot can find the marks in the dark.
 * They are the one part of a 1944 face that a photograph never shows well, and
 * they are also what makes the face read as period and not as modern.
 */
export function luminousDots(
  f: FaceCanvas,
  law: DialLaw,
  values: readonly number[],
  radius: number,
  size: number,
): void {
  const { ctx } = f;
  ctx.fillStyle = DIAL_COLOR.luminous;
  for (const value of values) {
    const angle = dialAngle(law, value);
    const r = radius * f.radius;
    ctx.beginPath();
    ctx.arc(pointX(f, angle, r), pointY(f, angle, r), size * f.radius, 0, Math.PI * 2);
    ctx.fill();
  }
}
