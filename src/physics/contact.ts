/**
 * Ground contact kinematics.
 *
 * The module answers three questions for any point that is fixed to the
 * airframe: where is it in the world, how fast does it move, and how deep is it
 * below the ground. It then turns a world force at that point into a wrench in
 * BODY axes. src/physics/gear.ts uses it for each landing gear leg, and a later
 * bead can use it for a wing tip strike or for a belly landing.
 *
 *
 * THE GROUND
 *
 * The ground is one flat plane at NED z = 0, which is the runway threshold plane
 * of CONVENTIONS section 3.2. Altitude above the ground is -position.z, so a
 * point with a POSITIVE z is BELOW the ground. `depth` reports that number
 * directly.
 *
 *
 * WHY THE BODY POINT MATTERS
 *
 * A gear position is an offset from the center of gravity in body axes. The
 * attitude of the aircraft therefore decides where the wheel really is. A right
 * wing down attitude drops the right main wheel and lifts the left one, and a
 * nose up rotation lifts the nose wheel clear. Both effects come out of the one
 * line that rotates the body offset into the world frame. Nothing else in the
 * gear model has to know about attitude.
 *
 * The velocity of the point carries the same idea. A point that sits away from
 * the center of gravity moves with
 *
 *   v_point = v_cg + R(q) * (omega_body x r_body)
 *
 * so a roll rate gives the right main wheel a downward speed that the center of
 * gravity does not have. That term is what makes a crosswind landing settle onto
 * one wheel first.
 *
 *
 * COST
 *
 * The step runs 240 times per second. Every function here writes into an output
 * that the caller owns, and every scratch vector sits in module scope. The
 * module allocates nothing after load.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Vector3 } from 'three';

import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import { worldToBody } from '@/physics/rigidbody';

/** NED z of the flat ground plane. CONVENTIONS section 3.2 puts it at zero. */
export const GROUND_PLANE_Z = 0; // m

/**
 * Unit normal of the ground, in world NED. The ground pushes UP, and up is the
 * negative z direction of a north-east-down frame.
 */
export const GROUND_NORMAL = new Vector3(0, 0, -1);

/** Where a body fixed point sits, how fast it moves, and how deep it is. */
export interface ContactSample {
  /** World NED position of the point, m. */
  world: Vector3;
  /** World NED velocity of the point, m/s. */
  velocity: Vector3;
  /**
   * Depth below the ground plane, m. A POSITIVE value means the point has gone
   * through the ground, because NED z grows downward.
   */
  depth: number;
}

/** Makes an empty sample. Call it one time, outside the step. */
export function createContactSample(): ContactSample {
  return { world: new Vector3(), velocity: new Vector3(), depth: 0 };
}

/**
 * Fills `out` for one body fixed point.
 *
 * `bodyPoint` is an offset from the center of gravity in body axes, with x
 * forward, y right and z down.
 */
export function sampleContact(
  state: RigidBodyState,
  bodyPoint: Vector3,
  out: ContactSample,
): ContactSample {
  out.world.copy(bodyPoint).applyQuaternion(state.orientation).add(state.position);
  // v = v_cg + R(q) * (omega x r). The cross product is in body axes, so it
  // needs the same rotation the offset needs.
  leverVelocity.crossVectors(state.angularVelocity, bodyPoint).applyQuaternion(state.orientation);
  out.velocity.copy(state.velocity).add(leverVelocity);
  out.depth = out.world.z - GROUND_PLANE_Z;
  return out;
}

/**
 * Adds the wrench of one world force applied at one body fixed point.
 *
 * The force arrives in world NED, because the ground normal and the friction
 * directions live in the world frame. The wrench leaves in BODY axes, because
 * that is what src/physics/rigidbody.ts integrates. The moment is r x F about
 * the center of gravity, with both vectors in body axes.
 *
 * The function ADDS into `out`. It never clears it, so one wrench can collect
 * the aerodynamic force, the thrust, gravity and every gear leg.
 */
export function addContactWrench(
  state: RigidBodyState,
  bodyPoint: Vector3,
  worldForce: Vector3,
  out: Wrench,
): void {
  worldToBody(state.orientation, worldForce, forceBody);
  out.force.add(forceBody);
  momentBody.crossVectors(bodyPoint, forceBody);
  out.moment.add(momentBody);
}

// Scratch held in module scope. The step allocates nothing.
const leverVelocity = new Vector3();
const forceBody = new Vector3();
const momentBody = new Vector3();
