/**
 * Frame conversion between the NED physics frame and the Three.js render frame.
 *
 * This module is the only place in the codebase that converts between the two
 * frames. Read docs/CONVENTIONS.md section 3 before you change it. No other
 * file may write the axis swap again.
 *
 * The module imports only the Three.js core math classes, so a test can run it
 * in Node with no GPU and no browser.
 *
 *
 * THE TWO FRAMES
 *
 * NED, from CONVENTIONS 3.2. +x is north, +y is east, +z is down. The frame is
 * right handed, because north cross east gives down.
 *
 * Three.js. +x is right, +y is up, -z is forward. The frame is right handed,
 * because x cross y gives z.
 *
 *
 * THE MAP, WITH THE IMAGE OF EACH NED BASIS VECTOR
 *
 *   north (1, 0, 0)  ->  ( 0,  0, -1)    north is forward, and forward is -z
 *   east  (0, 1, 0)  ->  ( 1,  0,  0)    east is right
 *   down  (0, 0, 1)  ->  ( 0, -1,  0)    down is the opposite of up
 *
 * In component form:
 *
 *   three.x =  ned.y
 *   three.y = -ned.z
 *   three.z = -ned.x
 *
 * The map keeps the handedness. Proof: in NED, north cross east gives down.
 * Take the images of north and east and cross them.
 *
 *   (0, 0, -1) cross (1, 0, 0) = (0, -1, 0)
 *
 * That result is the image of down, so the handedness survives. The matrix
 * determinant is +1. The map is a rotation and not a reflection.
 *
 *
 * DEVIATION FROM CONVENTIONS 3.3, AND THE REASON
 *
 * Section 3.3 gives "three.z = ned.x". That map has a determinant of -1. It is
 * a mirror and not a rotation. Two faults follow. First, a mirror reverses
 * every rotation direction, so a right roll shows on screen as a left roll.
 * Second, no quaternion can express a mirror, so the attitude conversion that
 * section 3.3 asks for cannot exist.
 *
 * This module uses "three.z = -ned.x" instead. The east row and the down row
 * stay the same as section 3.3. Only the sign of the north row changes. North
 * then points along -z, which is the direction the default Three.js camera
 * looks. Section 3.3 needs this one sign fix. The owner of the document must
 * apply it.
 *
 *
 * THE ROTATION MATRIX AND ITS QUATERNION
 *
 * The rotation matrix M holds the three basis images as its columns.
 *
 *         [  0   1   0 ]
 *   M  =  [  0   0  -1 ]
 *         [ -1   0   0 ]
 *
 * Read it back to check. M times (1, 0, 0) gives (0, 0, -1), the image of
 * north. M times (0, 1, 0) gives (1, 0, 0), the image of east. M times
 * (0, 0, 1) gives (0, -1, 0), the image of down.
 *
 * Convert M to a quaternion with the standard trace method.
 *
 *   trace(M) = 0
 *   w = sqrt(1 + trace(M)) / 2 = 1 / 2
 *   x = (m21 - m12) / (4 w) = ( 0 - (-1)) / 2 =  1 / 2
 *   y = (m02 - m20) / (4 w) = ( 0 - (-1)) / 2 =  1 / 2
 *   z = (m10 - m01) / (4 w) = ( 0 -   1 ) / 2 = -1 / 2
 *
 * So R = (x, y, z, w) = (1/2, 1/2, -1/2, 1/2). The length is 1, because
 * 4 times (1/2)^2 is 1. In axis and angle form, R turns 120 degrees about the
 * axis (1, 1, -1) / sqrt(3). The 120 degree turn about a diagonal axis is the
 * signed cyclic permutation of the three axes that the table above shows.
 *
 * The unit test asserts each of these three images as a literal.
 *
 *
 * THE ATTITUDE CONVERSION IS A CONJUGATION
 *
 * The attitude quaternion q_ned turns a body vector into a NED world vector.
 * The render code needs q_three, which turns the same body vector, already
 * mapped into the render frame, into a render world vector. Write M for the
 * map. For every body vector v the render code needs:
 *
 *   M (q_ned v) = q_three (M v)
 *
 * Substitute v = M^-1 u, where u runs over every render frame vector:
 *
 *   M q_ned M^-1 u = q_three u      for every u
 *
 * Therefore q_three = R q_ned R^-1. The conversion is a conjugation, not a
 * product. R is a unit quaternion, so R^-1 is the conjugate of R. The inverse
 * conversion is q_ned = R^-1 q_three R.
 *
 * A plain product R q_ned would also compile and would also look plausible on
 * screen for level flight. It is wrong. The unit test checks the conjugation
 * property over many random attitudes, because that property is the one that
 * catches the error.
 *
 *
 * WHAT THIS MEANS FOR THE AIRCRAFT MODEL
 *
 * The body axes of CONVENTIONS 3.1 map through M in the same way. The nose,
 * body +x, points along render -z. The right wing, body +y, points along
 * render +x. The floor direction, body +z, points along render -y. A model
 * with the nose along -z and the top along +y needs no extra turn.
 */

import type { Vector3 } from 'three';
import { Quaternion } from 'three';

/**
 * The frame map, written as a unit quaternion. See the derivation above.
 * R = (1/2, 1/2, -1/2, 1/2) in the Three.js (x, y, z, w) order.
 */
const NED_TO_THREE = new Quaternion(0.5, 0.5, -0.5, 0.5);

/** The conjugate of NED_TO_THREE, which is its inverse because it is a unit. */
const THREE_TO_NED = new Quaternion(-0.5, -0.5, 0.5, 0.5);

/**
 * Scratch quaternion. It lets a caller pass the same object as the input and
 * the output without loss. The module allocates it once.
 */
const scratch = new Quaternion();

/**
 * Convert a position, a velocity, or any other NED vector into the render
 * frame. The caller may pass the same object as `ned` and `out`.
 */
export function nedToThree(ned: Vector3, out: Vector3): Vector3 {
  return out.set(ned.y, -ned.z, -ned.x);
}

/**
 * Convert a render frame vector back into NED. This is the exact inverse of
 * `nedToThree`. The caller may pass the same object as `three` and `out`.
 */
export function threeToNed(three: Vector3, out: Vector3): Vector3 {
  return out.set(-three.z, three.x, -three.y);
}

/**
 * Convert a NED attitude quaternion into a render frame attitude quaternion.
 * The input turns a body vector into a NED world vector. The output turns the
 * same body vector, mapped into the render frame, into a render world vector.
 * The caller may pass the same object as `q` and `out`.
 */
export function nedQuatToThree(q: Quaternion, out: Quaternion): Quaternion {
  scratch.copy(q);
  return out.copy(NED_TO_THREE).multiply(scratch).multiply(THREE_TO_NED);
}

/**
 * Convert a render frame attitude quaternion back into NED. This is the exact
 * inverse of `nedQuatToThree`. The caller may pass the same object as `q` and
 * `out`.
 */
export function threeQuatToNed(q: Quaternion, out: Quaternion): Quaternion {
  scratch.copy(q);
  return out.copy(THREE_TO_NED).multiply(scratch).multiply(NED_TO_THREE);
}
