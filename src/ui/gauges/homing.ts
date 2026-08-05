/**
 * AFN 2, the homing indicator. It fills the second spare 57 mm bezel of the
 * bottom row.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Anzeigegeraet AFN 2, part number Ln 27002, is the display of the FuG 16
 * ZY radio set. It sits between the repeater compass and the altimeter on
 * every photograph of an Me 262 panel, and section 3 of
 * src/render/models/cockpit.ts already names this bezel for it.
 *
 * It carries TWO pointers, and neither one turns about the middle of the dial.
 *
 *   The VERTICAL pointer hangs from a pivot at the bottom of the case and its
 *   tip swings left and right. It shows which way to turn to reach the ground
 *   station. A pointer to the left says turn left.
 *   The HORIZONTAL pointer hangs from a pivot at the left of the case and its
 *   tip swings up and down. It shows the strength of the signal, which grows
 *   as the aircraft closes on the station and collapses as it passes over it.
 *
 * Source: dealer and museum records of the AFN 2, Ln 27002. Confidence: firm
 * on the two pointers and on the fit, estimate on the face.
 *
 *
 * WHAT DRIVES IT HERE
 *
 * The beacon stands at the RUNWAY THRESHOLD, which CONVENTIONS section 3.2
 * puts at the origin of the world frame. The two numbers below are DUPLICATED
 * from RUNWAY_THRESHOLD_NORTH and RUNWAY_THRESHOLD_EAST of
 * src/aircraft/aircraft.ts, which duplicates them from src/world/runway.ts for
 * the same reason: this file may not import either one.
 *
 * The course pointer shows the bearing to the beacon less the heading of the
 * aircraft, so it points at the beacon in the frame of the pilot. The strength
 * pointer follows an inverse square law in the range, clamped at both ends.
 * The strength law is an ESTIMATE. A real field strength also depends on the
 * height and on the aerial pattern, and this simulator models no radio.
 */

import { clamp } from '@/math/tables';
import { DEG } from '@/math/units';
import type { AttitudeAngles, TelemetrySample } from '@/ui/debug-overlay';
import { attitudeAngles } from '@/ui/debug-overlay';

import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { GAUGE_Z } from './parts';
import type { NeedleLag } from './lag';
import { createLag, stepLag } from './lag';
import { DIAL_COLOR, SMALL_FACE_PIXELS, caption, drawFace, fillFace } from './draw';

/** The beacon, in the world NED frame, m. Read the module comment. */
const BEACON_NORTH = 0;
const BEACON_EAST = 0;

/** Bearing error that puts the course pointer on its stop, rad. */
export const HOMING_FULL_SCALE = 30 * DEG;

/** Range at which the signal reads full, m. */
const HOMING_FULL_RANGE = 2000;

/** Range at which the signal reads nothing, m. */
const HOMING_NO_SIGNAL_RANGE = 30000;

/**
 * How far each pointer tip swings, rad.
 *
 * The value is bounded by GEOMETRY and not by taste. Both pointers pivot
 * OUTSIDE the face, so a long swing carries the tip past the bezel and over
 * the panel, where nothing would clip it. At this swing the tip and the root
 * both stay inside 0.82 of the face radius.
 */
const POINTER_SWING = 22 * DEG;

/** Where each pivot sits, as a fraction of the face radius, from the middle. */
const PIVOT_OFFSET = 1.15;

/** Where a pointer starts and ends, measured from its own pivot. */
const POINTER_ROOT = 0.55;
const POINTER_TIP = 1.75;

/** Time constant of both pointers, s. A moving coil meter is well damped. */
export const HOMING_LAG = 0.8;

/** Scratch for the heading. The read functions allocate nothing. */
const attitude: AttitudeAngles = { roll: 0, pitch: 0, heading: 0 };

/**
 * Where the course pointer stands, from -1 at the left stop to +1 at the
 * right. A positive value tells the pilot to turn right.
 */
export function homingCourse(sample: TelemetrySample): number {
  const north = BEACON_NORTH - sample.state.position.x;
  const east = BEACON_EAST - sample.state.position.y;
  if (north === 0 && east === 0) return 0;
  const bearing = Math.atan2(east, north);
  const heading = attitudeAngles(sample.state.orientation, attitude).heading;
  let error = bearing - heading;
  // Bring the error into the range of half a turn each way.
  error -= Math.round(error / (Math.PI * 2)) * Math.PI * 2;
  return clamp(error / HOMING_FULL_SCALE, -1, 1);
}

/**
 * Where the strength pointer stands, from 0 at the bottom stop to 1 at the
 * top. The law is an ESTIMATE. Read the module comment.
 */
export function homingStrength(sample: TelemetrySample): number {
  const north = BEACON_NORTH - sample.state.position.x;
  const east = BEACON_EAST - sample.state.position.y;
  const range = Math.hypot(north, east);
  if (range <= HOMING_FULL_RANGE) return 1;
  if (range >= HOMING_NO_SIGNAL_RANGE) return 0;
  // An inverse square law in the range, scaled so it reaches zero at the far
  // end instead of trailing off for ever.
  const near = (HOMING_FULL_RANGE / range) ** 2;
  const far = (HOMING_FULL_RANGE / HOMING_NO_SIGNAL_RANGE) ** 2;
  return clamp((near - far) / (1 - far), 0, 1);
}

function paint(): HTMLCanvasElement {
  return drawFace(SMALL_FACE_PIXELS, (f) => {
    const ctx = f.ctx;
    fillFace(f);
    // A cross of hair lines. The pilot flies both pointers onto it.
    ctx.strokeStyle = DIAL_COLOR.dim;
    ctx.lineWidth = 0.02 * f.radius;
    ctx.beginPath();
    ctx.moveTo(f.cx - 0.62 * f.radius, f.cy);
    ctx.lineTo(f.cx + 0.62 * f.radius, f.cy);
    ctx.moveTo(f.cx, f.cy - 0.62 * f.radius);
    ctx.lineTo(f.cx, f.cy + 0.62 * f.radius);
    ctx.stroke();

    // The scale marks each pointer runs against.
    ctx.strokeStyle = DIAL_COLOR.mark;
    ctx.lineWidth = 0.03 * f.radius;
    for (const side of [-1, 1]) {
      for (const step of [0.5, 1]) {
        const x = f.cx + side * step * 0.44 * f.radius;
        ctx.beginPath();
        ctx.moveTo(x, f.cy - 0.86 * f.radius);
        ctx.lineTo(x, f.cy - 0.70 * f.radius);
        ctx.stroke();
        const y = f.cy + side * step * 0.44 * f.radius;
        ctx.beginPath();
        ctx.moveTo(f.cx - 0.86 * f.radius, y);
        ctx.lineTo(f.cx - 0.70 * f.radius, y);
        ctx.stroke();
      }
    }
    caption(f, 'AFN 2', 0, -0.62, 0.15, DIAL_COLOR.dim);
  });
}

export function createHoming(parts: GaugeParts): Instrument {
  parts.addFace(paint());

  // The COURSE pointer hangs from a pivot below the dial, so its tip swings
  // across the top. That is how the real pointer is built.
  const coursePivot = parts.addPivot('course');
  coursePivot.position.set(0, -PIVOT_OFFSET * parts.radius, 0);
  // A NEGATIVE tail starts the blade away from its pivot, so the part of the
  // pointer that would stand outside the bezel is simply never built.
  const course = parts.addNeedle({
    name: 'course',
    length: POINTER_TIP,
    tail: -POINTER_ROOT,
    width: 0.045,
    color: 0xe8e2d0,
    z: GAUGE_Z.needle,
  });
  course.removeFromParent();
  coursePivot.add(course);
  course.position.set(0, 0, GAUGE_Z.needle);

  // The STRENGTH pointer hangs from a pivot to the left of the dial, so its
  // tip swings up and down. It stands a quarter turn from the course pointer.
  const strengthPivot = parts.addPivot('strength');
  strengthPivot.position.set(-PIVOT_OFFSET * parts.radius, 0, 0);
  strengthPivot.rotation.z = -Math.PI / 2;
  const strength = parts.addNeedle({
    name: 'strength',
    length: POINTER_TIP,
    tail: -POINTER_ROOT,
    width: 0.045,
    color: 0xd2582f,
    z: GAUGE_Z.lower,
  });
  strength.removeFromParent();
  strengthPivot.add(strength);
  strength.position.set(0, 0, GAUGE_Z.lower);

  const courseLag: NeedleLag = createLag(HOMING_LAG, 0);
  const strengthLag: NeedleLag = createLag(HOMING_LAG, 0);

  return {
    update(sample: TelemetrySample, _readout: CockpitReadout, dt: number): void {
      // The pivot stands below the tip, so a pointer to the RIGHT needs a
      // clockwise turn, which is negative about the face +z axis.
      course.rotation.z = -stepLag(courseLag, homingCourse(sample), dt) * POINTER_SWING;
      // The strength pointer reads 0 at the bottom and 1 at the top, so its
      // travel is offset by half of the swing.
      // The pivot of this pointer already carries a quarter turn, which
      // reverses the sense of its own rotation, so the sign is the other way.
      const level = stepLag(strengthLag, homingStrength(sample), dt);
      strength.rotation.z = (level * 2 - 1) * POINTER_SWING;
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
