/**
 * Wendezeiger mit Libelle, the turn and slip indicator.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 A-1a carried an electric Wendezeiger, of the Fl.22407 and
 * Fl.22412 family, on the second row of the blind flying group. It holds two
 * separate mechanisms in one case, and they measure two different things.
 * Source: dealer and museum records of the Luftwaffe Wendezeiger series,
 * confidence: firm on the fit, estimate on the exact part number.
 *
 *
 * 1. THE NEEDLE IS A RATE GYRO
 *
 * The needle hangs on a gyro that is free about one axis. It measures the RATE
 * the aircraft turns about its vertical axis and nothing else. The needle
 * therefore reads yaw rate, in degrees per second.
 *
 * The two index marks are the standard rate turn, which is 3 degrees per
 * second and 360 degrees in two minutes. Full travel is twice that, at 6
 * degrees per second. A jet turns much faster than a standard rate when the
 * pilot pulls, so the needle spends most of a fight on its stop, which is
 * exactly what it did in 1944.
 *
 *
 * 2. THE BALL IS NOT A RATE, AND IT IS NOT THE RUDDER
 *
 * The ball is a weight in a curved glass tube full of damping fluid. It is a
 * pendulum, so it hangs along the APPARENT vertical. The apparent vertical is
 * the direction opposite the SPECIFIC FORCE, which is the total force with
 * gravity taken out, over the mass.
 *
 * THE BALL DOES NOT READ RUDDER. A rudder deflection reaches the ball only
 * through the side force that the fin then makes, and that side force is
 * already inside `lateralAcceleration`. A gauge that read the pedal instead
 * would center in a fully developed engine out sideslip, where the real ball
 * sits hard against the glass. The unit test states that fact, because it is
 * the fault that this instrument is most often built with.
 *
 * The law is
 *
 *   ball = -lateral specific force / g
 *
 * with the ball to the RIGHT for a positive result. Check it on a steady
 * forward slip with the right wing low by an angle `phi`. Nothing accelerates,
 * so the specific force is minus the gravity in body axes, and the body y part
 * of gravity is `g sin(phi)`. The lateral specific force is then `-g sin(phi)`
 * and the ball reads `+sin(phi)`, which puts it toward the LOW wing. That is
 * where a real ball goes, and it is why the rule is "step on the ball".
 */

import { G0, toDeg } from '@/math/units';
import { clamp } from '@/math/tables';

import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { GAUGE_Z, pointNeedle } from './parts';
import type { NeedleLag } from './lag';
import { createLag, stepLag } from './lag';
import type { DialLaw } from './dial';
import { dialAngle, linearDial } from './dial';
import type { TelemetrySample } from '@/ui/debug-overlay';
import {
  DIAL_COLOR,
  DIAL_STYLE,
  LARGE_FACE_PIXELS,
  caption,
  drawFace,
  fillFace,
} from './draw';

/** A standard rate turn, deg/s. Three degrees a second is 360 in two minutes. */
export const STANDARD_RATE = 3;

/** Full needle travel, deg/s, and the angle it reaches. */
export const TURN_FULL_SCALE = 6;
const TURN_FULL_ANGLE = 32;

/** The needle law, in degrees per second of yaw rate. */
export const TURN_LAW: DialLaw = linearDial(
  -TURN_FULL_SCALE,
  TURN_FULL_SCALE,
  -TURN_FULL_ANGLE,
  2 * TURN_FULL_ANGLE,
);

/**
 * Needle time constant, s. A rate gyro is the quickest thing on the panel,
 * and only the oil in its damper slows it. The value is an ESTIMATE.
 */
export const TURN_LAG = 0.25;

/**
 * Ball time constant, s. The ball rolls in a heavy damping fluid, so it is
 * slower than the needle and it never rings. The value is an ESTIMATE.
 */
export const BALL_LAG = 0.45;

/** Half the travel of the ball in the tube, as a fraction of the face radius. */
const BALL_TRAVEL = 0.34;

/** Radius of the curved tube, as a fraction of the face radius. */
const TUBE_RADIUS = 1.55;

/** Where the middle of the tube sits below the face middle, in face fractions. */
const TUBE_CENTER_Y = -0.30;

/** The yaw rate the needle shows, deg/s, from one telemetry sample. */
export function turnReading(sample: TelemetrySample): number {
  // A positive body z rate moves the nose RIGHT. CONVENTIONS section 3.1.
  return toDeg(sample.state.angularVelocity.z);
}

/** The clockwise needle angle at one yaw rate in deg/s. Both stops clamp. */
export function turnAngle(rateDegPerSecond: number): number {
  return dialAngle(TURN_LAW, rateDegPerSecond);
}

/**
 * Where the ball stands, from -1 at the left stop to +1 at the right stop.
 *
 * The only input is the body y specific force in m/s2. Read section 2 for the
 * sign and for why the rudder is not an input.
 */
export function ballPosition(lateralAcceleration: number): number {
  return clamp(-lateralAcceleration / G0, -1, 1);
}

function paint(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    const ctx = f.ctx;
    fillFace(f);

    // --- The needle scale. Zero at the top, with the two rate marks. ------
    for (const rate of [-TURN_FULL_SCALE, -STANDARD_RATE, 0, STANDARD_RATE, TURN_FULL_SCALE]) {
      const standard = Math.abs(rate) === STANDARD_RATE;
      const angle = dialAngle(TURN_LAW, rate);
      const outer = 0.97 * f.radius;
      const inner = (rate === 0 ? 0.74 : standard ? 0.76 : 0.82) * f.radius;
      ctx.strokeStyle = standard ? DIAL_COLOR.luminous : DIAL_COLOR.mark;
      ctx.lineWidth = (standard ? 0.055 : 0.04) * f.radius;
      ctx.beginPath();
      ctx.moveTo(f.cx + inner * Math.sin(angle), f.cy - inner * Math.cos(angle));
      ctx.lineTo(f.cx + outer * Math.sin(angle), f.cy - outer * Math.cos(angle));
      ctx.stroke();
    }
    caption(f, 'L', -0.40, 0.54, DIAL_STYLE.caption, DIAL_COLOR.dim);
    caption(f, 'R', 0.40, 0.54, DIAL_STYLE.caption, DIAL_COLOR.dim);

    // --- The tube of the Libelle, and its two cage lines. ------------------
    const cy = f.cy - TUBE_CENTER_Y * f.radius;
    const tube = TUBE_RADIUS * f.radius;
    // The tube bends upward at both ends, so its center of curvature stands
    // ABOVE the glass. A ball then rolls back to the middle under gravity.
    const arcCenterY = cy - tube;
    const halfArc = Math.asin((BALL_TRAVEL * f.radius * 1.35) / tube);
    ctx.strokeStyle = '#191d20';
    ctx.lineWidth = 0.30 * f.radius;
    ctx.beginPath();
    ctx.arc(f.cx, arcCenterY, tube, Math.PI / 2 - halfArc, Math.PI / 2 + halfArc);
    ctx.stroke();

    ctx.strokeStyle = DIAL_COLOR.mark;
    ctx.lineWidth = 0.028 * f.radius;
    for (const side of [-1, 1]) {
      const x = f.cx + side * 0.115 * f.radius;
      ctx.beginPath();
      ctx.moveTo(x, cy - 0.155 * f.radius);
      ctx.lineTo(x, cy + 0.155 * f.radius);
      ctx.stroke();
    }

    caption(f, 'Wendezeiger', 0, 0.20, 0.115, DIAL_COLOR.dim);
  });
}

export function createTurnSlip(parts: GaugeParts): Instrument {
  parts.addFace(paint());
  const needle = parts.addNeedle({
    name: 'turn',
    length: 0.80,
    tail: 0.08,
    width: 0.055,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  parts.addHub(0.075, 0x1a1c1e);

  // The ball rides on a pivot at the CENTER OF CURVATURE of the tube, so it
  // swings along the glass instead of sliding across the face in a straight
  // line. That is the movement a real ball makes.
  const ballPivot = parts.addPivot('ball');
  ballPivot.position.set(0, (TUBE_CENTER_Y + TUBE_RADIUS) * parts.radius, 0);
  const ball = parts.addHub(0.115, 0xe8e2d0, GAUGE_Z.hub);
  ball.removeFromParent();
  ballPivot.add(ball);
  ball.position.set(0, -TUBE_RADIUS * parts.radius, GAUGE_Z.lower);

  const turnLag: NeedleLag = createLag(TURN_LAG, 0);
  const ballLag: NeedleLag = createLag(BALL_LAG, 0);

  /** Half the swing of the ball along the tube, rad. */
  const ballSwing = Math.asin(BALL_TRAVEL / TUBE_RADIUS);

  return {
    update(sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      const rate = stepLag(turnLag, turnReading(sample), dt);
      pointNeedle(needle, turnAngle(rate));

      const position = stepLag(ballLag, ballPosition(readout.lateralAcceleration), dt);
      // A positive result puts the ball to the RIGHT, and the pivot stands
      // above the ball, so the pivot turns CLOCKWISE, which is negative.
      ballPivot.rotation.z = -position * ballSwing;
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
