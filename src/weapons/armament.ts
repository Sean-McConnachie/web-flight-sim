/**
 * The armament of the aircraft, as one object.
 *
 * Three modules already hold the parts. This file holds none of them. It joins
 * the four guns of src/weapons/mk108.ts, the shell flight of
 * src/weapons/ballistics.ts and the ground targets of src/weapons/targets.ts,
 * and it steps them together.
 *
 * It exists so that src/main.ts needs six lines to wire the weapon. Every other
 * order of the work would put the loop over the rounds in the composition root.
 *
 *
 * THE ORDER OF ONE STEP
 *
 *   1. run the guns. Every round that leaves a barrel takes a round out of the
 *      pool and starts it at the muzzle, in the WORLD frame, with the velocity
 *      of the aircraft already added.
 *   2. fly every live round one step.
 *   3. test the SEGMENT each round covered against every target, then against
 *      the ground.
 *   4. retire a round that hit, that ran out of time, or that flew too far.
 *
 * A round that is fired in step 1 flies in step 2 of the same step, so it never
 * hangs at the muzzle for one frame.
 *
 *
 * THE HIT TEST IS LINEAR IN THE TARGETS
 *
 * The world holds 26 targets and the air holds at most 128 rounds, so a full
 * test is 3300 box tests per step at the very worst. Each one is a few
 * multiplies. A grid would cost more to keep than it saves, so there is none.
 *
 *
 * THE SEPARATION RULE
 *
 * This file is physics of the same kind that src/weapons/targets.ts is. It
 * imports no renderer and no DOM of its own. It does import
 * src/weapons/targets.ts, which reads the placement rules of
 * src/world/scatter.ts, and test/unit/scatter.test.ts already proves that
 * module runs in Node.
 */

import { Vector3 } from 'three';

import type { RigidBodyState, Wrench } from '@/physics/rigidbody';
import type { Projectile } from '@/weapons/ballistics';
import {
  createProjectilePool,
  launchProjectile,
  spawnProjectile,
  stepProjectile,
} from '@/weapons/ballistics';
import type { Battery, Gun } from '@/weapons/mk108';
import {
  BORE_DIRECTION,
  MK108_SHELL,
  MUZZLE_VELOCITY,
  createBattery,
  resetBattery,
  updateBattery,
} from '@/weapons/mk108';
import type { Target } from '@/weapons/targets';
import { applyHit, resetTargets, segmentHitsTarget } from '@/weapons/targets';

/**
 * Rounds the pool holds.
 *
 * Four guns at 650 rounds per minute put 43.3 rounds in the air every second,
 * and a round lives 1.8 s at the most, so 78 is the largest number that can be
 * in the air at one time. 128 leaves room and costs 128 small objects.
 */
export const MAX_ROUNDS = 128;

/** Impacts one step can report. A step never makes more than one per round. */
export const MAX_IMPACTS = 32;

/**
 * Longest a round stays in the air, in seconds.
 *
 * The shell has fallen 41 m at 1000 m and 130 m at 1700 m, so it is on the
 * ground long before this. The limit only catches a round fired straight up.
 */
export const MAX_FLIGHT_TIME = 6; // s

/** Longest path a round flies before it is retired, in meters. */
export const MAX_FLIGHT_DISTANCE = 2500; // m

/** Where one shell went off. */
export interface Impact {
  /** World NED, m. */
  readonly position: Vector3;
  /** Index into the target list, or -1 when the round hit the ground. */
  target: number;
  /** True when this hit destroyed the target. */
  destroyed: boolean;
}

export interface Armament {
  /** The four guns, their ammunition and their recoil wrench. */
  readonly battery: Battery;
  /** Every round of the pool. Only the ones with `alive` are in the air. */
  readonly rounds: readonly Projectile[];
  /** The ground targets. */
  readonly targets: readonly Target[];
  /** The impacts of the LAST step. Read `impactCount` for how many are real. */
  readonly impacts: readonly Impact[];
  readonly impactCount: number;
  /** Rounds left over the four guns. */
  readonly roundsLeft: number;
  /**
   * The recoil of the guns, BODY axes, about the center of gravity. It is the
   * same object as `battery.recoil` and it is valid after `fixedUpdate`.
   */
  readonly recoil: Wrench;
  /** Runs the guns and the rounds for one physics step. */
  fixedUpdate(state: RigidBodyState, trigger: boolean, dt: number): void;
  /** Fills the boxes, empties the air and rebuilds every target. */
  reset(): void;
}

export function createArmament(targets: readonly Target[]): Armament {
  const battery = createBattery();
  const rounds = createProjectilePool(MAX_ROUNDS);

  const impacts: Impact[] = [];
  for (let i = 0; i < MAX_IMPACTS; i++) {
    impacts.push({ position: new Vector3(), target: -1, destroyed: false });
  }
  let impactCount = 0;

  /** The state of the step the guns fire from. onShot reads it. */
  let firingState: RigidBodyState | undefined;

  const hitPoint = new Vector3();

  /** Reports one impact, if the buffer still has room. */
  function addImpact(position: Vector3, target: number, destroyed: boolean): void {
    if (impactCount >= MAX_IMPACTS) return;
    const impact = impacts[impactCount];
    impact.position.copy(position);
    impact.target = target;
    impact.destroyed = destroyed;
    impactCount += 1;
  }

  /** One round leaves one barrel. */
  function onShot(gun: Gun): void {
    if (firingState === undefined) return;
    const round = spawnProjectile(rounds);
    if (round === undefined) return;
    launchProjectile(
      round,
      firingState.position,
      firingState.orientation,
      firingState.velocity,
      firingState.angularVelocity,
      gun.position,
      BORE_DIRECTION,
      MUZZLE_VELOCITY,
    );
    round.gun = gun.index;
  }

  return {
    battery,
    rounds,
    targets,
    impacts,
    recoil: battery.recoil,

    get impactCount(): number {
      return impactCount;
    },

    get roundsLeft(): number {
      return battery.rounds;
    },

    fixedUpdate(state: RigidBodyState, trigger: boolean, dt: number): void {
      impactCount = 0;

      // 1. The guns. onShot puts every round in the air at the muzzle.
      firingState = state;
      updateBattery(battery, trigger, dt, onShot);
      firingState = undefined;

      // 2, 3 and 4. Fly, test, retire.
      for (const round of rounds) {
        if (!round.alive) continue;
        stepProjectile(round, MK108_SHELL, dt);

        // The segment the round covered this step, against every target.
        let hit = false;
        for (let i = 0; i < targets.length; i++) {
          if (!segmentHitsTarget(targets[i], round.start, round.position, hitPoint)) continue;
          const destroyed = applyHit(targets[i]);
          addImpact(hitPoint, i, destroyed);
          hit = true;
          break;
        }
        if (hit) {
          round.alive = false;
          continue;
        }

        // The ground. Altitude is minus the world z, so z >= 0 is at or below
        // it. Cut the segment where it crosses, so the burst sits on the grass
        // and not under it.
        if (round.position.z >= 0) {
          const above = -round.start.z;
          const below = round.position.z;
          const fraction = above + below > 0 ? above / (above + below) : 0;
          hitPoint.copy(round.start).lerp(round.position, fraction);
          hitPoint.z = 0;
          addImpact(hitPoint, -1, false);
          round.alive = false;
          continue;
        }

        if (round.age >= MAX_FLIGHT_TIME || round.distance >= MAX_FLIGHT_DISTANCE) {
          round.alive = false;
        }
      }
    },

    reset(): void {
      resetBattery(battery);
      resetTargets(targets);
      for (const round of rounds) round.alive = false;
      impactCount = 0;
    },
  };
}
