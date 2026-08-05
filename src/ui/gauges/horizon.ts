/**
 * Wendehorizont, the gyro artificial horizon.
 *
 *
 * 1. HOW IT IS BUILT
 *
 * This is the one dial on the panel that a needle cannot do. The horizon must
 * move up and down in pitch AND turn in bank at the same time, so the moving
 * part is a DRUM behind a round aperture, exactly as the real instrument is
 * built.
 *
 * Three parts, from the back forward:
 *
 *   The DRUM. A cylinder whose axis lies across the dial. Sky above the
 *   horizon line, ground below it, and a pitch ladder painted on both. The
 *   drum turns about its own axis by the pitch angle. Section 3 of
 *   src/ui/gauges/parts.ts states why the cylinder is squashed on z, and
 *   section 4 of that file states where the zero of its texture sits.
 *
 *   The BANK GIMBAL. An empty pivot between the face and the drum. It turns
 *   about the line of sight by MINUS the roll angle. A drum on a gimbal is
 *   what the real instrument carries, and it is why the sky stays level while
 *   the case rolls with the aircraft.
 *
 *   The MASK. A flat ring in front of the drum, painted black, with the bank
 *   scale on it. It hides the corners of the drum, which reach past the round
 *   aperture whenever the gimbal turns. Without the mask the pilot sees the
 *   ends of the cylinder swing across the dial.
 *
 * A fixed aircraft symbol and a fixed bank index stand in front of the mask.
 * They belong to the CASE and not to the gyro, so they never move.
 *
 *
 * 2. THE DRUM LAW
 *
 * The horizon line stands at `R sin(pitch)` above the middle of the aperture,
 * because that is where a point on a cylinder is. The scale therefore crowds
 * together toward the top and the bottom of the aperture and it runs out of
 * travel at 90 degrees, which is what a real drum does and what a sliding card
 * cannot do. The aperture covers 0.90 of the drum radius, so the pilot sees
 * 64 degrees of ladder each way.
 *
 *
 * 3. THIS IS A GYRO, NOT THE TRUTH
 *
 * A gyro horizon is wrong in two ways that a pilot must know about, and both
 * are here.
 *
 * FIRST, IT LAGS. The gimbals have mass and friction, so the instrument
 * arrives after the aircraft does. Pitch lags by 0.45 s and bank by 0.30 s.
 * Bank is the quicker of the two, because the roll gimbal is the lighter one.
 *
 * SECOND, IT ERECTS TO THE APPARENT VERTICAL AND NOT TO THE REAL ONE. The
 * erection mechanism hangs on pendulous vanes, and a vane cannot tell gravity
 * from an acceleration. It feels the SPECIFIC FORCE. Hold an acceleration long
 * enough and the gyro slowly rolls its spin axis onto that false vertical, so
 * the instrument shows a climb that is not there. A long turn tips it in bank
 * the same way.
 *
 * The error is modeled as a first order state that chases
 * `atan(specific force / g)` with a time constant of 45 s and stops at 6
 * degrees. Real erection runs at a few degrees per minute and real instruments
 * carry a cut out that stops the vanes in a turn, so both numbers are of the
 * right order. Confidence: estimate, from the erection rates quoted for
 * vacuum driven horizons of the period.
 *
 * A take off run at 0.35 g for 25 s therefore leaves about two degrees of
 * false climb on the dial, and it bleeds back off over the next minute.
 */

import { DEG, G0 } from '@/math/units';
import { clamp } from '@/math/tables';
import type { AttitudeAngles, TelemetrySample } from '@/ui/debug-overlay';
import { attitudeAngles } from '@/ui/debug-overlay';

import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { GAUGE_Z } from './parts';
import type { NeedleLag } from './lag';
import { createLag, stepLag, stepLagWrapped } from './lag';
import { DIAL_COLOR, LARGE_FACE_PIXELS, caption, drawFace, drawStrip } from './draw';
import { Shape } from 'three/webgpu';

/** Radius of the drum, as a fraction of the face radius. */
const DRUM_RADIUS = 0.80;

/** Radius of the aperture, as a fraction of the DRUM radius. Read section 2. */
const APERTURE = 0.90;

/** Pitch lag and bank lag, s. Read section 3. */
export const HORIZON_PITCH_LAG = 0.45;
export const HORIZON_BANK_LAG = 0.30;

/** Time constant of the erection error, s. Read section 3. */
export const HORIZON_ERECTION_LAG = 45;

/** Largest erection error the vanes can build up, rad. Read section 3. */
export const HORIZON_ERECTION_LIMIT = 6 * DEG;

/**
 * The false vertical that a specific force builds.
 *
 * `along` is the body specific force along the axis the vane hangs across, in
 * m/s2. The vane settles where the apparent vertical points, which is
 * `atan(along / g)`, and the mechanism stops at HORIZON_ERECTION_LIMIT.
 */
export function erectionTarget(along: number): number {
  return clamp(Math.atan2(along, G0), -HORIZON_ERECTION_LIMIT, HORIZON_ERECTION_LIMIT);
}

/**
 * Where the horizon line stands in the aperture, as a fraction of the drum
 * radius, at one pitch angle. Positive is above the middle.
 *
 * This is the law of section 2. A nose up attitude puts the horizon BELOW the
 * middle, so the sign turns over.
 */
export function horizonOffset(pitch: number): number {
  return -Math.sin(pitch);
}

/** Scratch for the attitude. The read functions allocate nothing. */
const attitude: AttitudeAngles = { roll: 0, pitch: 0, heading: 0 };

/** The pitch the drum shows before its lag, rad. */
export function horizonPitchTarget(sample: TelemetrySample, erection: number): number {
  return attitudeAngles(sample.state.orientation, attitude).pitch + erection;
}

/** The bank the gimbal shows before its lag, rad. */
export function horizonBankTarget(sample: TelemetrySample, erection: number): number {
  return attitudeAngles(sample.state.orientation, attitude).roll + erection;
}

// ---------------------------------------------------------------------------
// The faces
// ---------------------------------------------------------------------------

/**
 * Pixels around the drum and along it.
 *
 * The two axes need very different resolutions. Around the drum the texture
 * carries the horizon line, which must stay sharp, so it gets 1024 pixels for
 * one turn. Along the drum it carries only the width of a ladder bar, which is
 * a slow shape, so 256 pixels are enough. The face itself holds 205 pixels per
 * drum radius, and 1024 pixels for one turn gives 163, which is close enough
 * that the eye does not see the join.
 */
const DRUM_PIXELS_AROUND = 1024;
const DRUM_PIXELS_ALONG = 256;

/**
 * The drum.
 *
 * Section 4 of src/ui/gauges/parts.ts fixes the mapping: canvas x carries the
 * ladder angle, from -180 degrees at the left edge to +180 at the right, and
 * canvas y runs ACROSS the drum.
 *
 * The ladder carries NO numerals. Ten degrees of ladder is only 8 pixels on
 * the screen at the eye distance of the pilot, so a numeral would be four
 * pixels tall and unreadable. Bars of two widths carry the same information
 * and they are what most drums of the period really show.
 */
function paintDrum(): HTMLCanvasElement {
  return drawStrip(DRUM_PIXELS_AROUND, DRUM_PIXELS_ALONG, (ctx, width, height) => {
    /** Canvas x of one ladder angle, in degrees. Zero sits in the middle. */
    const at = (deg: number): number => width / 2 + (deg * width) / 360;
    /** Canvas width of a span of ladder, in degrees. */
    const span = (deg: number): number => (deg * width) / 360;

    ctx.fillStyle = DIAL_COLOR.ground;
    ctx.fillRect(0, 0, width, height);
    ctx.fillStyle = DIAL_COLOR.sky;
    ctx.fillRect(at(0), 0, span(180), height);

    // The horizon line. It is the one mark on this drum that must be sharp.
    ctx.fillStyle = DIAL_COLOR.horizon;
    ctx.fillRect(at(-1.3), 0, span(2.6), height);

    // The ladder. A wide bar every 20 degrees and a narrow one between them.
    // The bars stop at 70 degrees, where the aperture runs out.
    for (let deg = -70; deg <= 70; deg += 10) {
      if (deg === 0) continue;
      const wide = deg % 20 === 0;
      // The canvas height covers the whole drum length, which is 2.1 radii.
      const halfHeight = ((wide ? 0.34 : 0.18) * height) / 2.1;
      ctx.fillStyle = DIAL_COLOR.horizon;
      ctx.fillRect(at(deg - 1.1), height / 2 - halfHeight, span(2.2), halfHeight * 2);
    }
  });
}

/**
 * The mask ring and the bank scale on it.
 *
 * The canvas covers the whole face square, because RingGeometry maps its uv
 * from the outer radius. Only the ring itself is ever sampled, so the middle
 * of the canvas is left alone.
 */
function paintMask(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    const ctx = f.ctx;
    ctx.fillStyle = DIAL_COLOR.face;
    ctx.beginPath();
    ctx.arc(f.cx, f.cy, f.radius, 0, Math.PI * 2);
    ctx.fill();

    // The bank scale. Marks at 10, 20 and 30 degrees each way, and a longer
    // mark at 60. Those are the angles a pilot rolls to on instruments.
    for (const deg of [-60, -30, -20, -10, 10, 20, 30, 60]) {
      const angle = deg * DEG;
      const long = Math.abs(deg) === 60 || Math.abs(deg) === 30;
      const outer = 0.985 * f.radius;
      const inner = (long ? 0.86 : 0.90) * f.radius;
      ctx.strokeStyle = DIAL_COLOR.mark;
      ctx.lineWidth = (long ? 0.035 : 0.024) * f.radius;
      ctx.beginPath();
      ctx.moveTo(f.cx + inner * Math.sin(angle), f.cy - inner * Math.cos(angle));
      ctx.lineTo(f.cx + outer * Math.sin(angle), f.cy - outer * Math.cos(angle));
      ctx.stroke();
    }
    // The wings level mark at the top.
    ctx.fillStyle = DIAL_COLOR.pointer;
    ctx.beginPath();
    ctx.moveTo(f.cx, f.cy - 0.845 * f.radius);
    ctx.lineTo(f.cx + 0.055 * f.radius, f.cy - 0.99 * f.radius);
    ctx.lineTo(f.cx - 0.055 * f.radius, f.cy - 0.99 * f.radius);
    ctx.closePath();
    ctx.fill();

    caption(f, 'Wendehorizont', 0, -0.90, 0.11, DIAL_COLOR.dim);
  });
}

/**
 * The fixed aircraft symbol, in face fractions, with +y up the dial.
 *
 * Two wing bars with a GAP between them, and a small square in the gap. The
 * gap matters: in level flight the symbol lands exactly on the horizon line,
 * and a solid bar there would hide the one line the pilot needs most.
 */
function symbolShapes(): Shape[] {
  const bar = (x0: number, x1: number, y0: number, y1: number): Shape => {
    const shape = new Shape();
    shape.moveTo(x0, y0);
    shape.lineTo(x1, y0);
    shape.lineTo(x1, y1);
    shape.lineTo(x0, y1);
    shape.closePath();
    return shape;
  };
  return [
    bar(-0.54, -0.16, -0.032, 0.032),
    bar(0.16, 0.54, -0.032, 0.032),
    bar(-0.05, 0.05, -0.05, 0.05),
  ];
}

export function createHorizon(parts: GaugeParts): Instrument {
  // Back to front: gimbal, drum, mask, symbol.
  const gimbal = parts.addPivot('bank');
  const drum = parts.addDrum(paintDrum(), DRUM_RADIUS, gimbal);
  parts.addRing(paintMask(), DRUM_RADIUS * APERTURE, 1, GAUGE_Z.mask);
  parts.addPlate('symbol', symbolShapes(), 0xe8b23c, GAUGE_Z.hub);

  const pitchLag: NeedleLag = createLag(HORIZON_PITCH_LAG, 0);
  const bankLag: NeedleLag = createLag(HORIZON_BANK_LAG, 0);
  const pitchErection: NeedleLag = createLag(HORIZON_ERECTION_LAG, 0);
  const bankErection: NeedleLag = createLag(HORIZON_ERECTION_LAG, 0);

  return {
    update(sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      // The vanes hang across the two body axes that the specific force acts
      // on. A push along the nose tips the pitch gimbal, and a side force tips
      // the roll gimbal.
      const pitchError = stepLag(
        pitchErection,
        erectionTarget(readout.longitudinalAcceleration),
        dt,
      );
      const bankError = stepLag(
        bankErection,
        erectionTarget(-readout.lateralAcceleration),
        dt,
      );

      const pitch = stepLagWrapped(
        pitchLag,
        horizonPitchTarget(sample, pitchError),
        Math.PI * 2,
        dt,
      );
      const bank = stepLagWrapped(
        bankLag,
        horizonBankTarget(sample, bankError),
        Math.PI * 2,
        dt,
      );

      // The drum brings the ladder value that equals the pitch angle under the
      // aircraft symbol. Section 4 of parts.ts proves that one line.
      drum.rotation.x = pitch;
      // A right bank must tilt the sky CLOCKWISE in the view of the pilot, and
      // a clockwise turn about the face +z axis is a NEGATIVE rotation.
      gimbal.rotation.z = -bank;
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
