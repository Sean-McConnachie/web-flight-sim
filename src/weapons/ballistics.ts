/**
 * Exterior ballistics of a gun round.
 *
 * The module flies one shell through the real atmosphere under gravity and
 * aerodynamic drag. It knows nothing about the gun that fired it and nothing
 * about the thing it hits. src/weapons/mk108.ts supplies the shell, and
 * src/weapons/armament.ts joins the two.
 *
 *
 * 1. THE EQUATION
 *
 * A shell in free flight carries two accelerations, in world NED axes:
 *
 *   a = (0, 0, G0)  -  (rho * |v| / (2 * BC)) * v
 *
 * The first term is gravity, on the world down axis of CONVENTIONS 3.2. The
 * second is drag. It acts along the flight path and against it, and its size is
 * the dynamic pressure over the ballistic coefficient.
 *
 * The BALLISTIC COEFFICIENT used here is
 *
 *   BC = m / (Cd * A)          kg/m2
 *
 * with `m` the shell mass, `Cd` the drag coefficient and `A` the frontal area.
 * It is the form an engineer uses, not the sporting form that divides by the
 * mass of a standard bullet. A large BC means a shell that holds its speed.
 *
 * The density comes from src/physics/atmosphere.ts at the height of the shell,
 * so a round fired at 8000 m keeps its speed far better than the same round at
 * sea level. That is a real and large effect: the density at 8000 m is 43
 * percent of the sea level value, so the drag is 43 percent as well.
 *
 *
 * 2. THE CLOSED FORM THE TESTS CHECK AGAINST
 *
 * Flat fire, level bore, constant density. Write `k = rho / (2 * BC)`, which
 * has the unit of one over a meter. While the shell stays flat the vertical
 * speed is small against the horizontal speed, so |v| is the horizontal speed
 * and the horizontal equation closes on its own:
 *
 *   dvx/dt = -k vx^2   ->   vx(x) = v0 * exp(-k x)
 *   t(x)   = (exp(k x) - 1) / (k v0)
 *
 * The vertical equation is then LINEAR in the vertical speed, because the
 * horizontal speed is already known:
 *
 *   vx dvz/dx = g - k vx vz     ->     dvz/dx + k vz = g / vx
 *
 * With the integrating factor exp(k x) and vz(0) = 0 this gives
 *
 *   vz(x) = (g / (k v0)) * sinh(k x)
 *   drop  = (g / (2 k v0^2)) * ( (exp(2 k x) - 1) / (2 k)  -  x )
 *
 * `flatFireTime` and `flatFireDrop` return those two results. They are an
 * INDEPENDENT solution of the same physics, so test/unit/ballistics.test.ts
 * uses them to check the integrator, and it also asserts the numbers as
 * literals so that a change to both at once cannot pass in silence.
 *
 * The closed form drops the vertical drag term of order (vz/vx)^2. At 600 m the
 * MK 108 shell has vz/vx near 0.05, so that term is near 0.2 percent.
 *
 *
 * 3. THE INTEGRATOR
 *
 * Classic fourth order Runge-Kutta on position and velocity together. A round
 * lives about one and a half seconds and the drag time constant is of the same
 * size, so an explicit Euler step at 240 Hz would carry about half a percent of
 * error into the drop. RK4 puts the error below the width of the shell at any
 * step the loop can hand it, which is what lets the test compare against the
 * closed form to a tight tolerance.
 *
 * `stepProjectile` allocates nothing. Every scratch vector lives in module
 * scope. A burst from four guns puts about 65 rounds in the air at one time,
 * and each one takes one step per physics step.
 *
 *
 * 4. THE SEPARATION RULE
 *
 * CONVENTIONS section 4. This file is physics. It imports the Three.js core
 * math classes, the standard atmosphere and the standard gravity, and nothing
 * else. It runs in Node with no GPU and no browser.
 */

import type { Quaternion } from 'three';
import { Vector3 } from 'three';

import { G0 } from '@/math/units';
import type { AtmosphereSample } from '@/physics/atmosphere';
import { createAtmosphereSample, isa } from '@/physics/atmosphere';
import { bodyToWorld } from '@/physics/rigidbody';

// ---------------------------------------------------------------------------
// The shell
// ---------------------------------------------------------------------------

/** What the flight of one shell needs to know about it. */
export interface ProjectileSpec {
  /** Shell mass, kg. */
  readonly mass: number;
  /** Drag coefficient on the frontal area. */
  readonly dragCoefficient: number;
  /** Frontal area, m2. */
  readonly referenceArea: number;
  /** mass / (dragCoefficient * referenceArea), kg/m2. See section 1. */
  readonly ballisticCoefficient: number;
}

/**
 * Builds a shell from its mass, its drag coefficient and its caliber.
 *
 * The frontal area is the area of the circle of that caliber. A driving band
 * stands a little proud of the body, and the drag coefficient absorbs that.
 */
export function projectileSpec(
  mass: number,
  dragCoefficient: number,
  caliber: number,
): ProjectileSpec {
  const referenceArea = 0.25 * Math.PI * caliber * caliber;
  return {
    mass,
    dragCoefficient,
    referenceArea,
    ballisticCoefficient: mass / (dragCoefficient * referenceArea),
  };
}

// ---------------------------------------------------------------------------
// One round in the air
// ---------------------------------------------------------------------------

export interface Projectile {
  /** Where the round is now, world NED, m. */
  position: Vector3;
  /** How fast it goes, world NED, m/s. */
  velocity: Vector3;
  /**
   * Where the round was at the top of the last step, world NED, m.
   *
   * The hit test needs the whole segment and not the end of it. A round covers
   * about 3 m in one physics step and a fuel drum is 2.4 m across, so a test of
   * the end point alone would let a round pass through a drum and report
   * nothing.
   */
  start: Vector3;
  /** Time since the round left the barrel, s. */
  age: number;
  /** Path length since the round left the barrel, m. */
  distance: number;
  /** True while the round is in the air. */
  alive: boolean;
  /** Index of the gun that fired it. The tracer color follows the pair. */
  gun: number;
}

/** Makes one round, at rest and not in the air. */
export function createProjectile(): Projectile {
  return {
    position: new Vector3(),
    velocity: new Vector3(),
    start: new Vector3(),
    age: 0,
    distance: 0,
    alive: false,
    gun: 0,
  };
}

/**
 * Makes a fixed pool of rounds.
 *
 * Nothing allocates once the pool exists. Four MK 108 at 650 rounds per minute
 * put 43 rounds in the air every second, and a round lives under two seconds,
 * so a pool of 128 never runs dry in normal fire.
 */
export function createProjectilePool(capacity: number): Projectile[] {
  const pool: Projectile[] = [];
  for (let i = 0; i < capacity; i++) {
    pool.push(createProjectile());
  }
  return pool;
}

/** Returns a round that is not in the air, or undefined when the pool is full. */
export function spawnProjectile(pool: Projectile[]): Projectile | undefined {
  for (let i = 0; i < pool.length; i++) {
    if (!pool[i].alive) return pool[i];
  }
  return undefined;
}

// ---------------------------------------------------------------------------
// The launch
// ---------------------------------------------------------------------------

/** Scratch. The launch and the step allocate nothing. */
const muzzleWorld = new Vector3();
const muzzleVelocityBody = new Vector3();
const muzzleVelocityWorld = new Vector3();

/**
 * Puts one round in the air at the muzzle of a gun.
 *
 * THE ROUND LEAVES IN THE WORLD FRAME AND IT CARRIES THE AIRCRAFT WITH IT. A
 * shell fired forward from an aircraft at 900 km/h leaves the barrel at 250 m/s
 * plus the muzzle velocity, so it starts at 755 m/s over the ground. Anything
 * that launches the round in the body frame and forgets the aircraft velocity
 * gives a weapon that cannot hit a thing from a diving attack.
 *
 * The muzzle also moves with the rotation of the aircraft. A pitch rate of one
 * radian per second at 5 m ahead of the center of gravity swings the muzzle at
 * 5 m/s, which is one percent of the muzzle velocity and a real change of aim.
 * `omega x r` covers it.
 *
 * @param round        the round to launch, from `spawnProjectile`
 * @param position     center of gravity of the aircraft, world NED, m
 * @param orientation  attitude of the aircraft, body into world
 * @param velocity     velocity of the center of gravity, world NED, m/s
 * @param angularVelocity  body axes, rad/s
 * @param muzzle       muzzle position, body axes, m from the center of gravity
 * @param bore         unit vector along the bore, body axes
 * @param muzzleSpeed  speed of the shell over the barrel, m/s
 */
export function launchProjectile(
  round: Projectile,
  position: Vector3,
  orientation: Quaternion,
  velocity: Vector3,
  angularVelocity: Vector3,
  muzzle: Vector3,
  bore: Vector3,
  muzzleSpeed: number,
): void {
  bodyToWorld(orientation, muzzle, muzzleWorld);
  round.position.copy(position).add(muzzleWorld);
  round.start.copy(round.position);

  // The velocity of the muzzle itself, in body axes, plus the bore velocity.
  muzzleVelocityBody.crossVectors(angularVelocity, muzzle).addScaledVector(bore, muzzleSpeed);
  bodyToWorld(orientation, muzzleVelocityBody, muzzleVelocityWorld);
  round.velocity.copy(velocity).add(muzzleVelocityWorld);

  round.age = 0;
  round.distance = 0;
  round.alive = true;
}

// ---------------------------------------------------------------------------
// The step
// ---------------------------------------------------------------------------

/** Scratch for the integrator. */
const air: AtmosphereSample = createAtmosphereSample();
const accel = new Vector3();
const stagePosition = new Vector3();
const stageVelocity = new Vector3();
const k1v = new Vector3();
const k2v = new Vector3();
const k3v = new Vector3();
const k4v = new Vector3();
const k1a = new Vector3();
const k2a = new Vector3();
const k3a = new Vector3();
const k4a = new Vector3();
const moved = new Vector3();

/**
 * Writes the acceleration of a shell at one state, world NED, m/s2.
 *
 * The drag term is `-(rho |v| / (2 BC)) v`. Written that way it needs the speed
 * one time and no unit vector, so it holds up at a speed of zero.
 */
export function projectileAcceleration(
  spec: ProjectileSpec,
  position: Vector3,
  velocity: Vector3,
  out: Vector3,
): Vector3 {
  // CONVENTIONS 3.2: altitude is minus the world z.
  isa(-position.z, air);
  const speed = velocity.length();
  out.set(0, 0, G0);
  if (speed > 0) {
    out.addScaledVector(velocity, (-0.5 * air.density * speed) / spec.ballisticCoefficient);
  }
  return out;
}

/**
 * Flies one round forward by `dt`, with fourth order Runge-Kutta.
 *
 * The round keeps the position it held at the top of the step in `start`, so
 * the caller can test the whole segment against a target.
 */
export function stepProjectile(round: Projectile, spec: ProjectileSpec, dt: number): void {
  if (!round.alive || !(dt > 0)) return;

  round.start.copy(round.position);

  const half = 0.5 * dt;

  k1v.copy(round.velocity);
  projectileAcceleration(spec, round.position, round.velocity, k1a);

  stagePosition.copy(round.position).addScaledVector(k1v, half);
  stageVelocity.copy(round.velocity).addScaledVector(k1a, half);
  k2v.copy(stageVelocity);
  projectileAcceleration(spec, stagePosition, stageVelocity, k2a);

  stagePosition.copy(round.position).addScaledVector(k2v, half);
  stageVelocity.copy(round.velocity).addScaledVector(k2a, half);
  k3v.copy(stageVelocity);
  projectileAcceleration(spec, stagePosition, stageVelocity, k3a);

  stagePosition.copy(round.position).addScaledVector(k3v, dt);
  stageVelocity.copy(round.velocity).addScaledVector(k3a, dt);
  k4v.copy(stageVelocity);
  projectileAcceleration(spec, stagePosition, stageVelocity, k4a);

  const sixth = dt / 6;
  moved
    .copy(k1v)
    .addScaledVector(k2v, 2)
    .addScaledVector(k3v, 2)
    .add(k4v)
    .multiplyScalar(sixth);
  round.position.add(moved);

  accel
    .copy(k1a)
    .addScaledVector(k2a, 2)
    .addScaledVector(k3a, 2)
    .add(k4a)
    .multiplyScalar(sixth);
  round.velocity.add(accel);

  round.age += dt;
  round.distance += moved.length();
}

// ---------------------------------------------------------------------------
// The closed form. Section 2 of the module comment derives it.
// ---------------------------------------------------------------------------

/** The drag constant `k = rho / (2 BC)`, in one over a meter. */
export function dragConstant(spec: ProjectileSpec, density: number): number {
  return density / (2 * spec.ballisticCoefficient);
}

/**
 * Time of flight to a range, for flat fire in air of constant density.
 *
 *   t(x) = (exp(k x) - 1) / (k v0)
 */
export function flatFireTime(
  spec: ProjectileSpec,
  density: number,
  muzzleSpeed: number,
  range: number,
): number {
  const k = dragConstant(spec, density);
  if (k <= 0) return range / muzzleSpeed;
  return (Math.exp(k * range) - 1) / (k * muzzleSpeed);
}

/** Speed left at a range, for flat fire: `v(x) = v0 exp(-k x)`. */
export function flatFireSpeed(
  spec: ProjectileSpec,
  density: number,
  muzzleSpeed: number,
  range: number,
): number {
  return muzzleSpeed * Math.exp(-dragConstant(spec, density) * range);
}

/**
 * Drop below the bore line at a range, for flat fire in air of constant density.
 *
 *   drop(x) = (g / (2 k v0^2)) * ( (exp(2 k x) - 1) / (2 k) - x )
 *
 * The result carries the drag on the VERTICAL speed as well as on the forward
 * speed, which is why it is smaller than the free fall value `0.5 g t^2`. At
 * 600 m the MK 108 shell falls 10.6 m and free fall over the same time of
 * flight would be 12.7 m, so the vertical drag takes off a sixth of the drop.
 */
export function flatFireDrop(
  spec: ProjectileSpec,
  density: number,
  muzzleSpeed: number,
  range: number,
): number {
  const k = dragConstant(spec, density);
  if (k <= 0) {
    const t = range / muzzleSpeed;
    return 0.5 * G0 * t * t;
  }
  const scale = G0 / (2 * k * muzzleSpeed * muzzleSpeed);
  return scale * ((Math.exp(2 * k * range) - 1) / (2 * k) - range);
}
