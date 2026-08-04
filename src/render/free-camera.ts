/**
 * Debug fly camera.
 *
 * The camera lets a person inspect the scene before the aircraft exists. It is
 * a tool for development. It is not the flight camera.
 *
 * Controls:
 *   click        take the pointer lock
 *   mouse        look
 *   W and S      forward and back
 *   A and D      left and right
 *   Q and E      down and up, along the render world y axis
 *   shift        move fast
 *   escape       release the pointer lock, which the browser handles
 *
 * The camera works in the Three.js render frame, because it never talks to the
 * physics. Read docs/CONVENTIONS.md section 3.3. Up is +y here. This file does
 * not convert frames, so it does not break the rule that src/render/frames.ts
 * owns every conversion.
 */

import type { Camera } from 'three';
import { Euler, Vector3 } from 'three';

/** Base speed, in m/s. It crosses the 2400 m runway in about 30 s. */
const BASE_SPEED = 80;

/** Speed multiplier while shift is down. */
const FAST_MULTIPLIER = 8;

/** Radians of look per pixel of mouse movement. */
const LOOK_SENSITIVITY = 0.0022;

/**
 * Largest pitch, in radians, just under a quarter turn. The stop keeps the
 * camera from turning upside down at the poles.
 */
const MAX_PITCH = Math.PI / 2 - 0.001;

/** Smoothing half life, in seconds. It hides a single jerky frame. */
const SMOOTHING_HALF_LIFE = 0.05;

export interface FreeCamera {
  /** Advance the camera. `dt` is the frame time, in seconds. */
  update(dt: number): void;
  dispose(): void;
  /** Set to false to stop the camera from reading the input. */
  enabled: boolean;
}

/**
 * Build the fly camera. The camera reads its start attitude from the camera
 * object, so the caller may place the camera first.
 */
export function createFreeCamera(camera: Camera, domElement: HTMLElement): FreeCamera {
  const keys = new Set<string>();

  // Hold the look angles as numbers. A stored euler drifts after many frames.
  const startEuler = new Euler().setFromQuaternion(camera.quaternion, 'YXZ');
  let yaw = startEuler.y;
  let pitch = startEuler.x;

  let locked = false;

  const euler = new Euler(0, 0, 0, 'YXZ');
  const velocity = new Vector3();
  const wish = new Vector3();
  const forward = new Vector3();
  const right = new Vector3();
  const worldUp = new Vector3(0, 1, 0);

  const onPointerDown = (): void => {
    if (!api.enabled) return;
    if (document.pointerLockElement !== domElement) {
      // The browser rejects the request outside a user gesture, and it rejects
      // a second request while the first is still in flight. Swallow both.
      const request: unknown = domElement.requestPointerLock();
      if (request instanceof Promise) request.catch(() => undefined);
    }
  };

  const onPointerLockChange = (): void => {
    locked = document.pointerLockElement === domElement;
  };

  const onMouseMove = (event: MouseEvent): void => {
    if (!api.enabled || !locked) return;
    yaw -= event.movementX * LOOK_SENSITIVITY;
    pitch -= event.movementY * LOOK_SENSITIVITY;
    if (pitch > MAX_PITCH) pitch = MAX_PITCH;
    if (pitch < -MAX_PITCH) pitch = -MAX_PITCH;
  };

  const onKeyDown = (event: KeyboardEvent): void => {
    keys.add(event.code);
  };

  const onKeyUp = (event: KeyboardEvent): void => {
    keys.delete(event.code);
  };

  // A lost window focus leaves a key stuck down, so clear the set.
  const onBlur = (): void => {
    keys.clear();
  };

  domElement.addEventListener('pointerdown', onPointerDown);
  document.addEventListener('pointerlockchange', onPointerLockChange);
  document.addEventListener('mousemove', onMouseMove);
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('keyup', onKeyUp);
  window.addEventListener('blur', onBlur);

  const api: FreeCamera = {
    enabled: true,

    update(dt: number): void {
      if (!api.enabled || dt <= 0) return;

      euler.set(pitch, yaw, 0, 'YXZ');
      camera.quaternion.setFromEuler(euler);

      // Forward is -z in the render frame. Right is +x.
      forward.set(0, 0, -1).applyQuaternion(camera.quaternion);
      right.set(1, 0, 0).applyQuaternion(camera.quaternion);

      wish.set(0, 0, 0);
      if (keys.has('KeyW')) wish.add(forward);
      if (keys.has('KeyS')) wish.sub(forward);
      if (keys.has('KeyD')) wish.add(right);
      if (keys.has('KeyA')) wish.sub(right);
      if (keys.has('KeyE')) wish.add(worldUp);
      if (keys.has('KeyQ')) wish.sub(worldUp);

      // Normalize, so a diagonal is not faster than a straight line.
      if (wish.lengthSq() > 0) wish.normalize();

      const fast = keys.has('ShiftLeft') || keys.has('ShiftRight');
      wish.multiplyScalar(fast ? BASE_SPEED * FAST_MULTIPLIER : BASE_SPEED);

      // Exponential smoothing that does not depend on the frame rate.
      const blend = 1 - Math.pow(0.5, dt / SMOOTHING_HALF_LIFE);
      velocity.lerp(wish, blend);

      camera.position.addScaledVector(velocity, dt);
    },

    dispose(): void {
      domElement.removeEventListener('pointerdown', onPointerDown);
      document.removeEventListener('pointerlockchange', onPointerLockChange);
      document.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      window.removeEventListener('blur', onBlur);
      keys.clear();
      if (document.pointerLockElement === domElement) document.exitPointerLock();
    },
  };

  return api;
}
