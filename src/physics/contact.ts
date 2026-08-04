/**
 * Ground contact kinematics, and the contact of the airframe itself.
 *
 * The module answers three questions for any point that is fixed to the
 * airframe: where is it in the world, how fast does it move, and how deep is it
 * below the ground. It then turns a world force at that point into a wrench in
 * BODY axes. src/physics/gear.ts uses it for each landing gear leg, and
 * createAirframeContact below uses it for the nose, the tail, the two wing tips,
 * the two nacelles and the belly.
 *
 *
 * WHY THE AIRFRAME NEEDS CONTACT POINTS
 *
 * With three wheels and nothing else, the only part of the aircraft that the
 * ground can push on is the tire contact patch. An arrival that carries the
 * center of gravity below the ground plane therefore drives the gear far past
 * its hard stop, which is structure and not a spring, and no explicit step can
 * follow that force. The airframe points stop the aircraft before the gear ever
 * reaches that state, and they are what makes a belly landing scrape and slide
 * instead of explode.
 *
 *
 * WHY EVERY CONTACT FORCE IS BOUNDED
 *
 * A spring with no limit can always make a force that a fixed step cannot
 * integrate. Every force this module makes is therefore capped, one point at a
 * time and again over the sum, at a multiple of the weight of the aircraft. Past
 * that load the airframe has already failed, so the only duty left to the model
 * is to stay finite and to shed the energy. limitContactWrench does that job and
 * src/physics/gear.ts uses it as well.
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

import { G0 } from '@/math/units';
import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import { clearWrench, createWrench, worldToBody } from '@/physics/rigidbody';

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

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

/**
 * Largest ground force the model makes, as a multiple of the weight.
 *
 * CONVENTIONS section 8 gives the airframe a limit load factor of +7 g, and a
 * landing gear carries an ultimate load of 1.5 times its limit load, which is
 * 10.5 g. This value sits just above that pair. Below it nothing is clipped: the
 * hardest arrival the gear model survives, a 7 m/s drop that bottoms both struts
 * and bursts both main tires, peaks near 6 g. Above it the structure has already
 * failed, so no force the model reports can be right, and the only duty left is
 * to stay finite. A spring with no cap always breaks a fixed step in the end.
 */
export const MAX_GROUND_LOAD_FACTOR = 12;

/**
 * Fraction of the reversal limit that a friction force may use.
 *
 * Coulomb friction has one sign each side of zero speed. A step that applies it
 * at full size across zero chatters forever, and a step that fades it to zero
 * over a speed band cannot hold anything still. This module does neither. It
 * keeps the full coefficient at any speed and only stops the force from driving
 * the point BACKWARD inside one step: a point that carries `load` newtons holds
 * up `load / G0` kilograms, so the force that exactly kills its motion in one
 * step is `load / G0 * speed / dt`, which is `load * speed / (G0 * dt)` and
 * therefore a coefficient of `speed / (G0 * dt)`.
 *
 * The fraction is not one, because that force also acts on an arm. Near zero
 * speed the cap works as a damper of `c = load / (G0 * dt)`, and the criterion
 * that src/physics/gear.ts records, `c * dt * (1 / m + r^2 / I) < 1`, then reads
 * `1 + m r^2 / I`, which is above one. At the design mass, with the contact
 * patch 1.20 m below the center of gravity and a pitch inertia of 23214 kg m2,
 * `m r^2 / I` is 0.39, so a half share gives 0.70 and the step settles.
 */
const HOLD_MARGIN = 0.5;

/**
 * Returns the largest friction coefficient that cannot reverse the motion of a
 * contact point inside one step. See HOLD_MARGIN.
 */
export function reversalLimitMu(slideSpeed: number, dt: number): number {
  if (!(dt > 0)) {
    return Number.POSITIVE_INFINITY;
  }
  return (HOLD_MARGIN * Math.abs(slideSpeed)) / (G0 * dt);
}

/**
 * Scales `w` down until the force magnitude reaches `maxForce`, and returns the
 * factor it used. The moment takes the same factor, because every contact moment
 * is r x F over an arm the geometry already bounds.
 */
export function limitContactWrench(w: Wrench, maxForce: number): number {
  const force = w.force.length();
  if (!(force > maxForce)) {
    return 1;
  }
  const scale = maxForce / force;
  w.force.multiplyScalar(scale);
  w.moment.multiplyScalar(scale);
  return scale;
}

// ---------------------------------------------------------------------------
// The airframe contact set
// ---------------------------------------------------------------------------

/** One point of the airframe that the ground can push on. */
export interface ContactPointDef {
  name: string;
  /** Body axes, meters from the center of gravity. */
  position: Vector3;
}

/** What one airframe point is doing right now. */
export interface ContactPointState {
  onGround: boolean;
  /** Depth of the point below the ground plane, m. Zero while it is clear. */
  depth: number;
  /** Force normal to the ground, N. Never negative. */
  load: number;
  /** Speed of the point over the ground, m/s. It is what makes the scrape. */
  slideSpeed: number;
}

export interface AirframeContact {
  readonly defs: readonly ContactPointDef[];
  readonly points: readonly ContactPointState[];
  readonly anyOnGround: boolean;
  /** Adds the ground wrench of every point that touches into `out`, BODY axes. */
  update(state: RigidBodyState, dt: number, out: Wrench): void;
  reset(): void;
}

/**
 * How deep the points sink under the weight of the aircraft, m.
 *
 * The airframe is not a landing gear. It has no stroke, so the only thing this
 * number sets is how stiff the skin and the ground are together. Two centimeters
 * under the whole weight is stiff enough that the aircraft does not sink into the
 * runway and soft enough that the step follows it. Confidence: estimate.
 */
const CONTACT_DEFLECTION = 0.02; // m

/**
 * How many points carry the aircraft when it lies on its belly.
 *
 * The two nacelles and the belly make the tripod that a gear up landing of the
 * Me 262 really rests on. The number sizes the rate and the damping of ONE
 * point, so that the set together holds the weight at CONTACT_DEFLECTION.
 */
const BELLY_POINTS = 3;

/**
 * Damping of one point, as a fraction of critical damping against the mass it
 * carries. The airframe gives nothing back: a fuselage that struck the ground
 * and bounced would be wrong, and heavy damping is also what holds the step.
 */
const CONTACT_DAMPING_RATIO = 0.8;

/**
 * Friction of aluminum skin on dry concrete.
 *
 * A belly landing decelerates at about 0.5 to 0.7 g once the aircraft is down
 * and sliding, which is the coefficient below. It is far higher than a tire in
 * free roll, so a belly arrival stops in a short distance and does not glide.
 * Source: metal on concrete sliding pairs, Bowden and Tabor, "The Friction and
 * Lubrication of Solids". Confidence: estimate.
 */
const CONTACT_FRICTION = 0.7;

class Contacts implements AirframeContact {
  readonly defs: readonly ContactPointDef[];
  readonly points: ContactPointState[];
  private readonly stiffness: number;
  private readonly damping: number;
  private readonly maxPointForce: number;
  private readonly maxTotalForce: number;
  private grounded = false;

  constructor(defs: readonly ContactPointDef[], weight: number) {
    this.defs = defs;
    this.points = defs.map(() => ({ onGround: false, depth: 0, load: 0, slideSpeed: 0 }));
    this.stiffness = weight / (BELLY_POINTS * CONTACT_DEFLECTION);
    const carried = weight / (BELLY_POINTS * G0); // kg one point holds up
    this.damping = 2 * CONTACT_DAMPING_RATIO * Math.sqrt(this.stiffness * carried);
    // One point alone may not make the whole allowance, so that a single corner
    // cannot throw the aircraft while the rest of it is still clear.
    this.maxPointForce = 0.5 * MAX_GROUND_LOAD_FACTOR * weight;
    this.maxTotalForce = MAX_GROUND_LOAD_FACTOR * weight;
  }

  get anyOnGround(): boolean {
    return this.grounded;
  }

  reset(): void {
    for (const point of this.points) {
      point.onGround = false;
      point.depth = 0;
      point.load = 0;
      point.slideSpeed = 0;
    }
    this.grounded = false;
  }

  update(state: RigidBodyState, dt: number, out: Wrench): void {
    this.grounded = false;
    clearWrench(contactWrench);
    for (let i = 0; i < this.defs.length; i++) {
      const def = this.defs[i];
      const point = this.points[i];
      sampleContact(state, def.position, contactSample);
      if (contactSample.depth <= 0) {
        point.onGround = false;
        point.depth = 0;
        point.load = 0;
        point.slideSpeed = 0;
        continue;
      }

      // The ground pushes up, so the force is on the NEGATIVE world z. The
      // damper takes the closing speed, and the pair is clamped at both ends:
      // never a pull downward, never past the point allowance.
      let load = this.stiffness * contactSample.depth + this.damping * contactSample.velocity.z;
      if (load < 0) {
        load = 0;
      } else if (load > this.maxPointForce) {
        load = this.maxPointForce;
      }

      const slideX = contactSample.velocity.x;
      const slideY = contactSample.velocity.y;
      const slide = Math.hypot(slideX, slideY);
      point.onGround = true;
      point.depth = contactSample.depth;
      point.load = load;
      point.slideSpeed = slide;
      this.grounded = true;

      contactForce.set(0, 0, -load);
      if (slide > 0) {
        const mu = Math.min(CONTACT_FRICTION, reversalLimitMu(slide, dt));
        const drag = (mu * load) / slide;
        contactForce.x -= drag * slideX;
        contactForce.y -= drag * slideY;
      }
      addContactWrench(state, def.position, contactForce, contactWrench);
    }
    limitContactWrench(contactWrench, this.maxTotalForce);
    out.force.add(contactWrench.force);
    out.moment.add(contactWrench.moment);
  }
}

/**
 * Builds the airframe contact set of one aircraft.
 *
 * `weight` is the design weight in newtons. It sizes the rate, the damping and
 * every cap, so one set of constants fits any aircraft this model flies.
 */
export function createAirframeContact(
  defs: readonly ContactPointDef[],
  weight: number,
): AirframeContact {
  if (!(weight > 0)) {
    throw new Error(`createAirframeContact needs a positive weight. It got ${weight}.`);
  }
  return new Contacts(defs, weight);
}

// Scratch held in module scope. The step allocates nothing.
const leverVelocity = new Vector3();
const forceBody = new Vector3();
const momentBody = new Vector3();
const contactSample: ContactSample = createContactSample();
const contactForce = new Vector3();
const contactWrench: Wrench = createWrench();
