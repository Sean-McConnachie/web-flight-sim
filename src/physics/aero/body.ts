/**
 * The fuselage and the engine nacelles.
 *
 * A body is not a lifting surface. It carries no bound circulation worth the
 * name, and a lifting line model of it would be wrong. The model here holds the
 * three effects that a slender body in a flow really produces.
 *
 *
 * 1. CROSS FLOW DRAG
 *
 * The cross flow principle splits the velocity into a part along the axis of the
 * body and a part across it. The part across the body meets the body as a
 * cylinder in a stream and makes a drag of its own, with the side area as the
 * reference and a drag coefficient near 1.2. The cross velocity is V sin(i),
 * where i is the angle between the flow and the axis, so the force grows with
 * sin squared. That is where most of the normal force of a real fuselage at a
 * large angle comes from.
 *
 * The coefficient of an infinite cylinder is too high for a body of finite
 * length, because the flow escapes around the ends. The factor eta below cuts it
 * by the fineness ratio.
 *
 *
 * 2. SLENDER BODY LIFT
 *
 * Munk showed that the normal force per unit length of a slender body follows
 * the rate of change of the cross section area:
 *
 *   dN/dx = 2 q sin(alpha) cos(alpha) dS/dx
 *
 * The integral of that over a body that closes to a point is zero, which is the
 * d'Alembert result. A real fuselage does not close to a point, so what survives
 * is the base area. The model uses a base area taken from the frontal area, and
 * the potential flow moment of the same load is the Munk moment below.
 *
 *
 * 3. THE MUNK MOMENT
 *
 * The same integral taken as a moment does not vanish for a closed body. It
 * gives
 *
 *   M = k rho V^2 volume sin(2 alpha)
 *
 * and it acts NOSE UP at a positive angle of attack. A fuselage is
 * DESTABILIZING in pitch. It wants to turn broadside to the flow. The
 * horizontal tail exists to beat it. A model that drops this term gives an
 * aircraft that is far too stable, that needs a tail far too small, and whose
 * short period frequency does not match the real one. The same term acts in yaw
 * with the sideslip angle, and there the fin has to beat it.
 *
 * k is 0.5 for the ideal slender body, times the apparent mass factor
 * (k2 - k1), which reaches 0.94 at a fineness ratio of 8. munkFactor in the def
 * carries the product, so a value near 0.47 is right for a fuselage.
 * Source: Munk, "The Aerodynamic Forces on Airship Hulls", NACA Report 184,
 * 1924. Confidence: firm.
 *
 *
 * The axial drag holds the skin friction and the form drag of the body. It uses
 * the frontal area and its own coefficient, and it grows with the wave drag of
 * compressibility.ts above the critical Mach number.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Vector3 } from 'three';

import { clamp, lookup1d, table1d } from '@/math/tables';
import type { Table1D } from '@/math/tables';
import { machNumber } from '@/physics/atmosphere';
import type { MachCorrection } from '@/physics/aero/compressibility';
import { createMachCorrection, machCorrection } from '@/physics/aero/compressibility';
import type { Wrench } from '@/physics/rigidbody';

export interface BodyDef {
  name: string;
  /** Body axes, meters from the center of gravity, at the cross flow center of pressure. */
  position: Vector3;
  /** Meters, along the axis. */
  length: number;
  /** Meters. */
  maxDiameter: number;
  /** Cubic meters, the enclosed volume. The Munk moment follows it. */
  volume: number;
  /** Square meters, the area seen from the side. The cross flow drag uses it. */
  sideArea: number;
  /** Square meters, the area seen from the front. The axial drag uses it. */
  frontalArea: number;
  axialDragCoefficient: number;
  /** Near 1.2 for a rounded body. The fineness ratio cuts it. */
  crossFlowDragCoefficient: number;
  /** Near 0.47 for a fuselage. See the module comment. */
  munkFactor: number;
}

export interface BodyResult {
  /** rad, the local angle of attack of the body. */
  alpha: number;
  /** rad, the local sideslip of the body. */
  beta: number;
  /** Body axes, newtons. */
  force: Vector3;
  /** Body axes about the center of gravity, newton meters. */
  moment: Vector3;
}

export interface Body {
  readonly def: BodyDef;
  readonly result: BodyResult;
  /** The fixed numbers that createBody works out one time. Treat it as read only. */
  readonly crossFlowFactor: number;
  readonly baseArea: number;
}

// The cross flow drag of a cylinder of finite length, over the drag of an
// infinite cylinder, against the fineness ratio length / diameter. The flow
// escapes around the ends of a short body, so a short body pays less.
// Source: Hoerner, "Fluid Dynamic Drag", chapter 3, table of finite cylinders,
// as used by Jorgensen, NASA TN D-6996. Confidence: firm.
const FINENESS_RATIO: readonly number[] = [1, 2, 5, 10, 20, 40, 1000];
const CROSS_FLOW_FACTOR: readonly number[] = [0.6, 0.65, 0.74, 0.82, 0.9, 0.95, 1.0];
const CROSS_FLOW_TABLE: Table1D = table1d(FINENESS_RATIO.slice(), CROSS_FLOW_FACTOR.slice());

// The base area of a body, as a fraction of its maximum cross section area.
//
// Slender body theory leaves only the base area, because the integral of dS/dx
// over a closed body is zero. The Me-262 fuselage tapers from its maximum
// section to a tail cone that still carries the fin and the tailplane, so the
// section at the end is far from zero but far from the maximum. A fifth of the
// maximum section is representative of a fighter fuselage of that shape.
// Confidence: estimate. The term is small next to the cross flow drag at any
// angle above about six degrees.
const BASE_AREA_FRACTION = 0.2;

// The slender body normal force slope, per radian, on the base area. Munk gives
// exactly 2. Source: Munk, NACA Report 184, 1924. Confidence: firm.
const SLENDER_LIFT_SLOPE = 2;

// Below this speed the flow angles carry no information.
const MIN_FLOW_SPEED = 1e-6; // m/s

// Scratch held in module scope. The step allocates nothing.
const localVelocity = new Vector3();
const bodyForce = new Vector3();
const bodyMoment = new Vector3();
const armMoment = new Vector3();
const mach: MachCorrection = createMachCorrection();

/** Builds one body and works out its fixed numbers. */
export function createBody(def: BodyDef): Body {
  if (!(def.maxDiameter > 0) || !(def.length > 0)) {
    throw new Error(
      `Body ${def.name} needs a positive length and a positive diameter. It got ` +
        `${def.length} and ${def.maxDiameter}.`,
    );
  }
  return {
    def,
    result: { alpha: 0, beta: 0, force: new Vector3(), moment: new Vector3() },
    crossFlowFactor: lookup1d(CROSS_FLOW_TABLE, def.length / def.maxDiameter),
    baseArea: def.frontalArea * BASE_AREA_FRACTION,
  };
}

/**
 * Adds the force and the moment of one body into out, in body axes, and fills
 * the result of the body.
 *
 * The function adds into out. The caller clears the wrench one time and then
 * runs every element into it.
 */
export function evaluateBody(
  b: Body,
  velocityBody: Vector3,
  angularVelocity: Vector3,
  windBody: Vector3,
  density: number,
  speedOfSound: number,
  out: Wrench,
): void {
  const def = b.def;
  const r = b.result;

  // The same local velocity rule as a strip. See surface.ts.
  localVelocity.crossVectors(angularVelocity, def.position).add(velocityBody).sub(windBody);
  const u = localVelocity.x;
  const v = localVelocity.y;
  const w = localVelocity.z;
  const speed = localVelocity.length();
  const dynamicPressure = 0.5 * density * speed * speed;

  const alpha = Math.atan2(w, u);
  const beta = speed > MIN_FLOW_SPEED ? Math.asin(clamp(v / speed, -1, 1)) : 0;

  machCorrection(machNumber(speed, speedOfSound), 0, mach);

  // Axial drag. The signed square keeps the force against the motion when the
  // body flies backwards in a tumble.
  const axial =
    -0.5 *
    density *
    Math.abs(u) *
    u *
    def.frontalArea *
    (def.axialDragCoefficient + mach.cdAdd);

  // Cross flow drag. The cross velocity is V sin(i), so the force already grows
  // with sin squared and no sine appears in the code. The force opposes the
  // cross velocity, which puts it on the correct side in pitch and in yaw at
  // the same time.
  const crossSpeed = Math.sqrt(v * v + w * w);
  const crossPressure =
    0.5 * density * crossSpeed * def.sideArea * def.crossFlowDragCoefficient * b.crossFlowFactor;

  // Slender body lift on the base area. Positive alpha lifts, which is a
  // negative z force. Positive sideslip pushes to the left, the same sense a
  // fin takes. The moment of this load is the Munk moment below, so the force
  // acts at the reference point and adds no moment of its own.
  const slender = SLENDER_LIFT_SLOPE * dynamicPressure * b.baseArea * mach.clScale;
  const normalLift = slender * Math.sin(alpha) * Math.cos(alpha);
  const sideLift = slender * Math.sin(beta) * Math.cos(beta);

  bodyForce.set(
    axial,
    -crossPressure * v - sideLift,
    -crossPressure * w - normalLift,
  );

  // The Munk moment. Nose up in pitch at a positive angle of attack, and nose
  // further left in yaw at a positive sideslip. Both senses turn the body
  // broadside to the flow, which is what makes it destabilizing.
  const munk = def.munkFactor * density * speed * speed * def.volume;
  bodyMoment.set(0, munk * Math.sin(2 * alpha), -munk * Math.sin(2 * beta));
  bodyMoment.add(armMoment.crossVectors(def.position, bodyForce));

  out.force.add(bodyForce);
  out.moment.add(bodyMoment);

  r.alpha = alpha;
  r.beta = beta;
  r.force.copy(bodyForce);
  r.moment.copy(bodyMoment);
}
