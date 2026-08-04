/**
 * Six degree of freedom rigid body, with a classical Runge-Kutta 4 integrator.
 *
 * Frames follow CONVENTIONS section 3. The position and the velocity are in the
 * world NED frame. The orientation quaternion rotates a body vector into the
 * world frame. The angular velocity, the force and the moment are in body axes,
 * with x forward, y right and z down.
 *
 * The state derivatives are:
 *
 *   dx/dt     = v
 *   dv/dt     = (1 / m) * R(q) * F_body
 *   dq/dt     = 0.5 * q (x) (0, omega_body)
 *   domega/dt = I^-1 * (M_body - omega x (I * omega))
 *
 * The omega x (I * omega) term makes the gyroscopic coupling and the
 * intermediate axis instability. Never drop it.
 *
 * GRAVITY: stepRK4 applies nothing on its own. The caller adds gravity to the
 * wrench, along with the aerodynamic force, the thrust and the gear force. The
 * caller works in body axes, so the caller must rotate the world gravity vector
 * into body axes first. See CONVENTIONS section 6.
 *
 * The step runs 240 times per second, so stepRK4 allocates nothing. Every
 * scratch vector, quaternion and derivative sits in module scope. One aircraft
 * steps at one time, and the physics runs on one thread, so the shared scratch
 * is safe. Never call stepRK4 from inside a WrenchSource.
 *
 * This module is pure math. It imports the Three.js core math classes only.
 */

import { Matrix3, Quaternion, Vector3 } from 'three';
import { clamp } from '@/math/tables';

export interface RigidBodyState {
  position: Vector3; // world NED, m
  velocity: Vector3; // world NED, m/s
  orientation: Quaternion; // rotates a body vector into the world frame
  angularVelocity: Vector3; // body axes, rad/s
}

export interface MassProperties {
  mass: number; // kg
  inertia: Matrix3; // body axes, about the CG, kg m^2
  inverseInertia: Matrix3;
}

/** Force and moment in BODY axes. The moment acts about the center of gravity. */
export interface Wrench {
  force: Vector3;
  moment: Vector3;
}

export type WrenchSource = (state: RigidBodyState, time: number, out: Wrench) => void;

/** The angle of attack, the sideslip angle and the airspeed magnitude. */
export interface FlowAngles {
  alpha: number; // rad
  beta: number; // rad
  speed: number; // m/s
}

/** The time derivative of one rigid body state. */
interface StateDerivative {
  velocity: Vector3; // d position / dt, world NED
  acceleration: Vector3; // d velocity / dt, world NED
  spin: Quaternion; // d orientation / dt, not a unit quaternion
  angularAcceleration: Vector3; // d omega / dt, body axes
}

/** Below this speed the flow angles have no meaning, so they report zero. */
const MIN_FLOW_SPEED = 1e-6; // m/s

/** Makes a state at the origin, level, at rest. */
export function createState(): RigidBodyState {
  return {
    position: new Vector3(0, 0, 0),
    velocity: new Vector3(0, 0, 0),
    orientation: new Quaternion(0, 0, 0, 1),
    angularVelocity: new Vector3(0, 0, 0),
  };
}

/** Makes an empty wrench. */
export function createWrench(): Wrench {
  return { force: new Vector3(0, 0, 0), moment: new Vector3(0, 0, 0) };
}

/**
 * Makes the mass properties and inverts the inertia tensor one time. The step
 * then needs no inversion. The tensor must be symmetric and positive definite.
 */
export function createMassProperties(mass: number, inertia: Matrix3): MassProperties {
  if (!(mass > 0) || !Number.isFinite(mass)) {
    throw new Error(`createMassProperties needs a positive finite mass. It got ${mass}.`);
  }
  const e = inertia.elements;
  for (let i = 0; i < 9; i++) {
    if (!Number.isFinite(e[i])) {
      throw new Error(
        `createMassProperties needs a finite inertia tensor. Element ${i} holds ${e[i]}.`,
      );
    }
  }
  const determinant = inertia.determinant();
  // Compare the determinant against the cube of the largest element. A raw test
  // against zero passes for a tensor that is singular within rounding error.
  let scale = 0;
  for (let i = 0; i < 9; i++) {
    const value = Math.abs(e[i]);
    if (value > scale) {
      scale = value;
    }
  }
  if (scale === 0 || Math.abs(determinant) < 1e-12 * scale * scale * scale) {
    throw new Error(
      `createMassProperties cannot invert a singular inertia tensor. The determinant is ` +
        `${determinant} and the largest element is ${scale}.`,
    );
  }
  const stored = inertia.clone();
  return { mass, inertia: stored, inverseInertia: stored.clone().invert() };
}

/** Adds one wrench into another. Both are in body axes. */
export function addWrench(target: Wrench, add: Wrench): void {
  target.force.add(add.force);
  target.moment.add(add.moment);
}

/** Sets a wrench to zero. */
export function clearWrench(w: Wrench): void {
  w.force.set(0, 0, 0);
  w.moment.set(0, 0, 0);
}

/** Rotates a body vector into the world frame. Writes into out and returns it. */
export function bodyToWorld(q: Quaternion, v: Vector3, out: Vector3): Vector3 {
  return out.copy(v).applyQuaternion(q);
}

/** Rotates a world vector into body axes. Writes into out and returns it. */
export function worldToBody(q: Quaternion, v: Vector3, out: Vector3): Vector3 {
  inverseQuat.copy(q).invert();
  return out.copy(v).applyQuaternion(inverseQuat);
}

/**
 * Returns the airspeed vector in body axes. The vector is the velocity of the
 * aircraft through the air mass, so in level forward flight the x component is
 * positive. The relative wind that the aircraft meets is the negative of this
 * vector. CONVENTIONS section 3.1 defines alpha and beta from this vector.
 *
 * The wind is the velocity of the air mass in the world NED frame.
 */
export function airspeedBody(state: RigidBodyState, wind: Vector3, out: Vector3): Vector3 {
  relativeVelocity.copy(state.velocity).sub(wind);
  return worldToBody(state.orientation, relativeVelocity, out);
}

/**
 * Returns the angle of attack, the sideslip angle and the airspeed magnitude
 * from an airspeed vector in body axes.
 *
 * alpha = atan2(w, u) and beta = asin(v / speed), as CONVENTIONS section 3.1
 * defines them. A positive w means the air comes from below, so alpha is
 * positive. A positive v means the air comes from the right, so beta is
 * positive. Both angles report zero when the aircraft stands still.
 */
export function flowAngles(airspeedBody: Vector3, out?: FlowAngles): FlowAngles {
  const result = out !== undefined ? out : { alpha: 0, beta: 0, speed: 0 };
  const speed = airspeedBody.length();
  result.speed = speed;
  if (speed < MIN_FLOW_SPEED) {
    result.alpha = 0;
    result.beta = 0;
    return result;
  }
  result.alpha = Math.atan2(airspeedBody.z, airspeedBody.x);
  // The clamp holds the argument of asin inside its range against rounding error.
  result.beta = Math.asin(clamp(airspeedBody.y / speed, -1, 1));
  return result;
}

// Scratch held in module scope. The step allocates nothing.
const inverseQuat = new Quaternion();
const relativeVelocity = new Vector3();
const initialState = createState();
const stageState = createState();
const stageWrench = createWrench();
const inertiaTimesOmega = new Vector3();
const gyroscopic = new Vector3();
const angularTerm = new Vector3();

function createDerivative(): StateDerivative {
  return {
    velocity: new Vector3(),
    acceleration: new Vector3(),
    spin: new Quaternion(0, 0, 0, 0),
    angularAcceleration: new Vector3(),
  };
}

const k1 = createDerivative();
const k2 = createDerivative();
const k3 = createDerivative();
const k4 = createDerivative();

/** Copies one state into another. */
function copyState(target: RigidBodyState, source: RigidBodyState): void {
  target.position.copy(source.position);
  target.velocity.copy(source.velocity);
  target.orientation.copy(source.orientation);
  target.angularVelocity.copy(source.angularVelocity);
}

/**
 * Writes the quaternion rate 0.5 * q (x) (0, omega) into out. The product is the
 * Hamilton product, and omega sits in body axes.
 */
function quaternionRate(q: Quaternion, omega: Vector3, out: Quaternion): void {
  const qx = q.x;
  const qy = q.y;
  const qz = q.z;
  const qw = q.w;
  const wx = omega.x;
  const wy = omega.y;
  const wz = omega.z;
  out.set(
    0.5 * (qw * wx + qy * wz - qz * wy),
    0.5 * (qw * wy - qx * wz + qz * wx),
    0.5 * (qw * wz + qx * wy - qy * wx),
    0.5 * (-qx * wx - qy * wy - qz * wz),
  );
}

/**
 * Reads the wrench at one stage and writes the state derivative into out. The
 * source sees the stage state, so a model that depends on velocity or on rate
 * integrates correctly.
 */
function evaluate(
  state: RigidBodyState,
  mass: MassProperties,
  source: WrenchSource,
  time: number,
  out: StateDerivative,
): void {
  clearWrench(stageWrench);
  source(state, time, stageWrench);

  out.velocity.copy(state.velocity);
  bodyToWorld(state.orientation, stageWrench.force, out.acceleration).multiplyScalar(1 / mass.mass);

  quaternionRate(state.orientation, state.angularVelocity, out.spin);

  // Euler equation. The gyroscopic term omega x (I * omega) stays.
  inertiaTimesOmega.copy(state.angularVelocity).applyMatrix3(mass.inertia);
  gyroscopic.crossVectors(state.angularVelocity, inertiaTimesOmega);
  angularTerm.copy(stageWrench.moment).sub(gyroscopic);
  out.angularAcceleration.copy(angularTerm).applyMatrix3(mass.inverseInertia);
}

/** Writes base + h * derivative into out. */
function advance(
  out: RigidBodyState,
  base: RigidBodyState,
  d: StateDerivative,
  h: number,
): void {
  out.position.copy(base.position).addScaledVector(d.velocity, h);
  out.velocity.copy(base.velocity).addScaledVector(d.acceleration, h);
  out.angularVelocity.copy(base.angularVelocity).addScaledVector(d.angularAcceleration, h);
  const q = base.orientation;
  const s = d.spin;
  out.orientation.set(q.x + s.x * h, q.y + s.y * h, q.z + s.z * h, q.w + s.w * h);
}

/**
 * Steps the state forward by dt with classical Runge-Kutta 4. The function
 * changes the state in place and allocates nothing.
 *
 * The source runs four times, one time for each stage. It must write the force
 * and the moment in body axes, and it must include gravity.
 *
 * The step normalizes the quaternion one time at the end. It does not normalize
 * inside a stage, because that would break the derivative that RK4 needs.
 */
export function stepRK4(
  state: RigidBodyState,
  mass: MassProperties,
  source: WrenchSource,
  time: number,
  dt: number,
): void {
  copyState(initialState, state);
  const half = 0.5 * dt;

  evaluate(initialState, mass, source, time, k1);

  advance(stageState, initialState, k1, half);
  evaluate(stageState, mass, source, time + half, k2);

  advance(stageState, initialState, k2, half);
  evaluate(stageState, mass, source, time + half, k3);

  advance(stageState, initialState, k3, dt);
  evaluate(stageState, mass, source, time + dt, k4);

  const sixth = dt / 6;
  combine(state.position, initialState.position, k1.velocity, k2.velocity, k3.velocity, k4.velocity, sixth);
  combine(
    state.velocity,
    initialState.velocity,
    k1.acceleration,
    k2.acceleration,
    k3.acceleration,
    k4.acceleration,
    sixth,
  );
  combine(
    state.angularVelocity,
    initialState.angularVelocity,
    k1.angularAcceleration,
    k2.angularAcceleration,
    k3.angularAcceleration,
    k4.angularAcceleration,
    sixth,
  );

  const q = initialState.orientation;
  state.orientation.set(
    q.x + sixth * (k1.spin.x + 2 * k2.spin.x + 2 * k3.spin.x + k4.spin.x),
    q.y + sixth * (k1.spin.y + 2 * k2.spin.y + 2 * k3.spin.y + k4.spin.y),
    q.z + sixth * (k1.spin.z + 2 * k2.spin.z + 2 * k3.spin.z + k4.spin.z),
    q.w + sixth * (k1.spin.w + 2 * k2.spin.w + 2 * k3.spin.w + k4.spin.w),
  );
  // The four stages leave the quaternion a little off unit length. One
  // normalization at the end of the step removes the drift.
  state.orientation.normalize();
}

/** Writes base + sixth * (a + 2 b + 2 c + d) into out. */
function combine(
  out: Vector3,
  base: Vector3,
  a: Vector3,
  b: Vector3,
  c: Vector3,
  d: Vector3,
  sixth: number,
): void {
  out.set(
    base.x + sixth * (a.x + 2 * b.x + 2 * c.x + d.x),
    base.y + sixth * (a.y + 2 * b.y + 2 * c.y + d.y),
    base.z + sixth * (a.z + 2 * b.z + 2 * c.z + d.z),
  );
}
