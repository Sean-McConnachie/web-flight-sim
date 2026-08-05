/**
 * Ground targets, where they stand, and what a shell does to them.
 *
 * The module holds two halves, in the same way src/world/scatter.ts does.
 *
 *   THE PLACEMENT AND HIT HALF is this file. It reads sizes and returns
 *   positions, health and a segment against box test. It imports the Three.js
 *   core math classes, src/core/prng.ts and the PLACEMENT RULES of
 *   src/world/scatter.ts. It builds no geometry and no material, so
 *   test/unit/ballistics.test.ts runs it in Node with no GPU.
 *
 *   THE RENDER HALF is `createTargetField` of src/render/weapons.ts. It turns
 *   the list this file returns into meshes, and it changes the look of a target
 *   that burns. Nothing in this file knows that it exists.
 *
 *
 * WHERE THE TARGETS STAND
 *
 * The rules of src/world/scatter.ts already say where an object may stand. This
 * module calls `isClearOfAirfield`, which keeps every target off the runway,
 * off the strip around it, off the apron and out of the approach corridor at
 * both ends. It also calls `insideAirfieldBoundary` and rejects anything inside
 * it, because a target field is a place to attack and the home airfield is not.
 *
 * Two dispersal sites sit about 3.5 km out, one to the north east and one to
 * the south west, so a pilot finds one soon after takeoff in either direction.
 * Each holds a hangar, a line of four parked aircraft, a stack of fuel drums
 * and two trucks. The hangar is the large target that a burst cannot miss and
 * the drums are the small one that needs a steady approach.
 *
 * Every position comes from src/core/prng.ts at a fixed seed, so the target
 * field is the same on every run and on every machine, exactly as the buildings
 * and the trees are.
 *
 *
 * THE BOUNDING VOLUME
 *
 * One box per target, standing on the ground and turned about the down axis by
 * its heading. A box is enough. A shell of 30 mm that passes inside the box of
 * a parked fighter has hit the fighter, and the error at the wing tips costs
 * nothing that a pilot can see.
 *
 * `position` is the CENTER of the box, so `-position.z` is the half height and
 * the box stands on the ground plane.
 *
 *
 * HEALTH
 *
 * Health counts HITS OF 30 MM and nothing else. The MK 108 fires one shell and
 * one shell type, so a table of energies would carry no more information than a
 * count. One shell wrecks a fuel drum, two wreck a truck, three wreck a parked
 * fighter and twelve open a hangar.
 */

import { Vector3 } from 'three';

import type { Rng } from '@/core/prng';
import { createRng } from '@/core/prng';
import { insideAirfieldBoundary, isClearOfAirfield } from '@/world/scatter';

// ---------------------------------------------------------------------------
// The kinds of target
// ---------------------------------------------------------------------------

export type TargetKind = 'hangar' | 'parked-aircraft' | 'truck' | 'fuel-drums';

/** What every target of one kind shares. */
export interface TargetKindSpec {
  readonly kind: TargetKind;
  /**
   * Half sizes of the bounding box in the frame of the target, in meters.
   * `x` runs along the heading, `y` runs to the right of it, `z` is the half
   * height.
   */
  readonly halfExtent: Vector3;
  /** Hits of 30 mm the target takes before it is destroyed. */
  readonly health: number;
}

/**
 * The four kinds. The sizes are the real ones: a Luftwaffe field hangar is
 * about 40 m by 25 m, a Bf 109 is 9 m long over 10 m of span, an Opel Blitz is
 * 6 m long, and a stack of four 200 liter drums covers about 2.4 m square.
 * Confidence: medium on every size, and none of them changes the physics.
 */
export const TARGET_KINDS: Readonly<Record<TargetKind, TargetKindSpec>> = {
  hangar: {
    kind: 'hangar',
    halfExtent: new Vector3(20, 12.5, 4.5),
    health: 12,
  },
  'parked-aircraft': {
    kind: 'parked-aircraft',
    halfExtent: new Vector3(4.5, 5, 1.4),
    health: 3,
  },
  truck: {
    kind: 'truck',
    halfExtent: new Vector3(3, 1.2, 1.3),
    health: 2,
  },
  'fuel-drums': {
    kind: 'fuel-drums',
    halfExtent: new Vector3(1.2, 1.2, 0.9),
    health: 1,
  },
};

/** Damage that one 30 mm shell does. Read the note on health above. */
export const SHELL_DAMAGE = 1;

// ---------------------------------------------------------------------------
// One target
// ---------------------------------------------------------------------------

export interface Target {
  readonly kind: TargetKind;
  /** Center of the bounding box, world NED, m. `-position.z` is the half height. */
  readonly position: Vector3;
  /** Heading of the long axis, in radians, with 0 north. */
  readonly heading: number;
  /** Half sizes of the box in the frame of the target, m. */
  readonly halfExtent: Vector3;
  readonly maxHealth: number;
  /** Health left. It reaches zero when the target is destroyed. */
  health: number;
  /** True once the health reached zero. */
  destroyed: boolean;
  /** Hits taken. The render half spreads the damage over the model with it. */
  hits: number;
}

/**
 * Takes one hit off a target.
 *
 * The function returns true only for the hit that DESTROYS the target, so the
 * caller can start the fire and the smoke one time. A hit on a target that is
 * already destroyed still counts, and still returns false.
 */
export function applyHit(target: Target, damage: number = SHELL_DAMAGE): boolean {
  target.hits += 1;
  if (target.destroyed) return false;
  target.health -= damage;
  if (target.health <= 0) {
    target.health = 0;
    target.destroyed = true;
    return true;
  }
  return false;
}

/** Puts every target back to full health. A spawn calls it. */
export function resetTargets(targets: readonly Target[]): void {
  for (const target of targets) {
    target.health = target.maxHealth;
    target.destroyed = false;
    target.hits = 0;
  }
}

// ---------------------------------------------------------------------------
// The hit test
// ---------------------------------------------------------------------------

/** Scratch for the hit test. The module allocates it one time. */
const localFrom = new Vector3();
const localTo = new Vector3();
const localDirection = new Vector3();

/** Below this the segment counts as a point on that axis. */
const SEGMENT_EPSILON = 1e-9;

/**
 * Writes a world NED point into the frame of a target.
 *
 * The target frame turns about the world DOWN axis by its heading, so the
 * forward axis is `(cos h, sin h, 0)` and the right axis is `(-sin h, cos h, 0)`.
 * The down axis is the world down axis.
 */
function toTargetFrame(target: Target, world: Vector3, out: Vector3): Vector3 {
  const north = world.x - target.position.x;
  const east = world.y - target.position.y;
  const cos = Math.cos(target.heading);
  const sin = Math.sin(target.heading);
  return out.set(cos * north + sin * east, -sin * north + cos * east, world.z - target.position.z);
}

/**
 * True when the segment from `from` to `to` passes through the box of a target.
 *
 * The test is the standard slab test, run in the frame of the target. `out`, if
 * the caller gives one, receives the FIRST point of the segment that is inside
 * the box, in world NED. That is where the shell went off.
 *
 * THE TEST IS ON THE SEGMENT AND NOT ON THE END POINT. A round covers 3 m in
 * one physics step at 240 Hz and a fuel drum is 2.4 m across, so an end point
 * test would miss it about half the time.
 */
export function segmentHitsTarget(
  target: Target,
  from: Vector3,
  to: Vector3,
  out?: Vector3,
): boolean {
  toTargetFrame(target, from, localFrom);
  toTargetFrame(target, to, localTo);
  localDirection.copy(localTo).sub(localFrom);

  let enter = 0;
  let exit = 1;

  for (let axis = 0; axis < 3; axis++) {
    const half = target.halfExtent.getComponent(axis);
    const start = localFrom.getComponent(axis);
    const direction = localDirection.getComponent(axis);
    if (Math.abs(direction) < SEGMENT_EPSILON) {
      // The segment does not move on this axis, so it must already be inside
      // the slab of this axis.
      if (start < -half || start > half) return false;
      continue;
    }
    let near = (-half - start) / direction;
    let far = (half - start) / direction;
    if (near > far) {
      const swap = near;
      near = far;
      far = swap;
    }
    if (near > enter) enter = near;
    if (far < exit) exit = far;
    if (enter > exit) return false;
  }

  if (out !== undefined) {
    out.copy(from).lerp(to, enter);
  }
  return true;
}

// ---------------------------------------------------------------------------
// The placement
// ---------------------------------------------------------------------------

/** Degrees to radians. src/math/units.ts holds the same constant. */
const DEG = Math.PI / 180;

/** Seed of the target field. It is not the seed of src/world/scatter.ts. */
export const TARGET_SEED = 10830;

/** One dispersal site, in NED meters, with the heading its line faces. */
interface Site {
  readonly name: string;
  readonly north: number;
  readonly east: number;
  readonly headingDeg: number;
}

/**
 * The two sites.
 *
 * The north east site sits beside the middle of the runway and 3 km out, so it
 * is off no approach corridor at all. The south west site sits 1.5 km short of
 * the south threshold, which puts it inside the LENGTH of the south approach
 * corridor. The corridor is only 370 m wide there and the site is 2.6 km to the
 * side of it, so `isClearOfAirfield` passes it. The test checks that, and it
 * checks it with the rules of src/world/scatter.ts and not with these words.
 */
const SITES: readonly Site[] = [
  { name: 'north east dispersal', north: 1750, east: 3030, headingDeg: 60 },
  { name: 'south west dispersal', north: -1500, east: -2600, headingDeg: 240 },
];

/** One item of a site, placed in the frame of that site, in meters. */
interface SiteItem {
  readonly kind: TargetKind;
  /** Along the site heading. */
  readonly along: number;
  /** To the right of the site heading. */
  readonly across: number;
  /** Turn of the item away from the site heading, in degrees. */
  readonly turnDeg: number;
}

/**
 * The layout of one site. The hangar stands at the origin of the site, a line
 * of four aircraft is parked in front of it and across the line, the drums sit
 * behind the hangar where a fuel dump belongs, and the two trucks stand on the
 * road that runs past.
 */
const SITE_LAYOUT: readonly SiteItem[] = [
  { kind: 'hangar', along: 0, across: 0, turnDeg: 0 },
  { kind: 'parked-aircraft', along: 62, across: -45, turnDeg: 90 },
  { kind: 'parked-aircraft', along: 62, across: -15, turnDeg: 90 },
  { kind: 'parked-aircraft', along: 62, across: 15, turnDeg: 90 },
  { kind: 'parked-aircraft', along: 62, across: 45, turnDeg: 90 },
  { kind: 'fuel-drums', along: -34, across: -12, turnDeg: 0 },
  { kind: 'fuel-drums', along: -34, across: -6, turnDeg: 0 },
  { kind: 'fuel-drums', along: -34, across: 0, turnDeg: 0 },
  { kind: 'fuel-drums', along: -40, across: -9, turnDeg: 0 },
  { kind: 'fuel-drums', along: -40, across: -3, turnDeg: 0 },
  { kind: 'fuel-drums', along: -40, across: 3, turnDeg: 0 },
  { kind: 'truck', along: 30, across: 78, turnDeg: 8 },
  { kind: 'truck', along: 48, across: 80, turnDeg: 6 },
];

/** Jitter of an item inside a site, in meters, and of its turn, in degrees. */
const ITEM_JITTER = 2.5;
const ITEM_TURN_JITTER = 5;

/** Radius of a circle that covers the footprint of a target, in meters. */
export function targetFootprintRadius(spec: TargetKindSpec): number {
  return Math.hypot(spec.halfExtent.x, spec.halfExtent.y);
}

/** Builds one target of a kind, at a place and a heading. */
function makeTarget(kind: TargetKind, north: number, east: number, heading: number): Target {
  const spec = TARGET_KINDS[kind];
  return {
    kind,
    // The box stands on the ground, so its center sits one half height up and
    // the world down axis makes that negative. CONVENTIONS 3.2.
    position: new Vector3(north, east, -spec.halfExtent.z),
    heading,
    halfExtent: spec.halfExtent.clone(),
    maxHealth: spec.health,
    health: spec.health,
    destroyed: false,
    hits: 0,
  };
}

/** Places the items of one site, and drops any that breaks a placement rule. */
function placeSite(site: Site, rng: Rng, out: Target[]): void {
  const siteHeading = site.headingDeg * DEG;
  const cos = Math.cos(siteHeading);
  const sin = Math.sin(siteHeading);

  for (const item of SITE_LAYOUT) {
    const along = item.along + rng.range(-ITEM_JITTER, ITEM_JITTER);
    const across = item.across + rng.range(-ITEM_JITTER, ITEM_JITTER);
    // The site frame turns about the world down axis, the same way the target
    // frame of `toTargetFrame` does.
    const north = site.north + cos * along - sin * across;
    const east = site.east + sin * along + cos * across;
    const heading =
      siteHeading + (item.turnDeg + rng.range(-ITEM_TURN_JITTER, ITEM_TURN_JITTER)) * DEG;

    const spec = TARGET_KINDS[item.kind];
    const obstacle = {
      north,
      east,
      radius: targetFootprintRadius(spec),
      height: 2 * spec.halfExtent.z,
    };
    // Rule 1, rule 2 and the apron of src/world/scatter.ts, and then the
    // airfield boundary. A target field belongs outside the wire.
    if (!isClearOfAirfield(obstacle)) continue;
    if (insideAirfieldBoundary(obstacle)) continue;

    out.push(makeTarget(item.kind, north, east, heading));
  }
}

/**
 * Places every target of the world.
 *
 * `seed` exists so a test can prove that the field really follows the seed. The
 * application never passes it.
 */
export function placeTargets(seed: number = TARGET_SEED): Target[] {
  const rng = createRng(seed);
  const targets: Target[] = [];
  for (const site of SITES) {
    placeSite(site, rng, targets);
  }
  return targets;
}
