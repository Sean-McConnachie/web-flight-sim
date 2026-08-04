/**
 * Chase camera.
 *
 * Bead b34 owns the full camera set: the cockpit, the fly by, the wing view and
 * the tower. This file holds only what the first flight needs, which is one
 * camera that follows the aircraft from behind and above.
 *
 *
 * WHY THE CAMERA DOES NOT ROLL WITH THE AIRCRAFT
 *
 * A camera rigidly fixed to the airframe rolls the whole world when the pilot
 * rolls. That reads well in a cockpit and badly from outside, because the eye
 * then cannot tell a roll from a turn. This camera takes the NOSE DIRECTION of
 * the aircraft and keeps its own up vector on the world vertical. A roll then
 * shows as the aircraft turning inside a level frame, which is what a chase
 * aircraft really sees.
 *
 * The nose direction keeps its vertical part, so a climb puts the camera below
 * the aircraft and a dive puts it above. VERTICAL_DAMPING takes part of that
 * out, so a vertical climb does not put the camera under the tail.
 *
 *
 * THE LAG
 *
 * The camera moves toward the place it wants with an exponential lag of a fixed
 * half life, which is frame rate independent. The lag is what makes the
 * aircraft look fast: with no lag the aircraft sits still in the frame and only
 * the ground moves.
 *
 * This file may use the renderer. It holds no physics.
 */

import type { PerspectiveCamera } from 'three/webgpu';
import { Quaternion, Vector3 } from 'three/webgpu';

/** Distance behind the aircraft, in meters. */
const DEFAULT_DISTANCE = 26;

/** Height above the aircraft, in meters. */
const DEFAULT_HEIGHT = 6;

/** Point the camera looks at, in meters above the center of gravity. */
const LOOK_HEIGHT = 1.5;

/** How much of the vertical part of the nose direction the camera keeps. */
const VERTICAL_DAMPING = 0.45;

/** Half life of the position lag, in seconds. */
const POSITION_HALF_LIFE = 0.22;

/** Half life of the aim lag, in seconds. It is faster, so the aim leads. */
const AIM_HALF_LIFE = 0.09;

/** Render frame up. */
const UP = new Vector3(0, 1, 0);

export interface ChaseCamera {
  /**
   * Moves the camera. `position` and `orientation` are the render frame pose of
   * the aircraft, which src/render/frames.ts produces from the physics state.
   */
  update(position: Vector3, orientation: Quaternion, dt: number): void;
  /** Puts the camera at its place at once, with no lag. A respawn needs it. */
  snap(position: Vector3, orientation: Quaternion): void;
  distance: number;
  height: number;
}

/** Builds the chase camera. It writes the position and the quaternion of `camera`. */
export function createChaseCamera(camera: PerspectiveCamera): ChaseCamera {
  const nose = new Vector3();
  const wanted = new Vector3();
  const aim = new Vector3();
  const smoothedAim = new Vector3();
  const eye = new Vector3();
  let started = false;

  /** Writes the place the camera wants into `wanted` and the aim into `aim`. */
  function solve(position: Vector3, orientation: Quaternion, api: ChaseCamera): void {
    // The nose points along render -z. See src/render/frames.ts.
    nose.set(0, 0, -1).applyQuaternion(orientation);
    nose.y *= VERTICAL_DAMPING;
    if (nose.lengthSq() < 1e-6) {
      nose.set(0, 0, -1);
    }
    nose.normalize();
    wanted.copy(position).addScaledVector(nose, -api.distance).addScaledVector(UP, api.height);
    aim.copy(position).addScaledVector(UP, LOOK_HEIGHT);
  }

  const api: ChaseCamera = {
    distance: DEFAULT_DISTANCE,
    height: DEFAULT_HEIGHT,

    snap(position: Vector3, orientation: Quaternion): void {
      solve(position, orientation, api);
      camera.position.copy(wanted);
      smoothedAim.copy(aim);
      camera.up.copy(UP);
      camera.lookAt(smoothedAim);
      started = true;
    },

    update(position: Vector3, orientation: Quaternion, dt: number): void {
      if (!started) {
        api.snap(position, orientation);
        return;
      }
      solve(position, orientation, api);
      // An exponential lag with a half life is the same at any frame rate.
      const positionBlend = 1 - Math.pow(0.5, dt / POSITION_HALF_LIFE);
      const aimBlend = 1 - Math.pow(0.5, dt / AIM_HALF_LIFE);
      eye.copy(camera.position).lerp(wanted, positionBlend);
      camera.position.copy(eye);
      smoothedAim.lerp(aim, aimBlend);
      camera.up.copy(UP);
      camera.lookAt(smoothedAim);
    },
  };

  return api;
}
