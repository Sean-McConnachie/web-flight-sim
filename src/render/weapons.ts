/**
 * The picture of the weapon: the targets on the ground, the tracers in the air,
 * the muzzle flashes and the bursts.
 *
 * This module is the RENDER HALF of the weapon. The other half is
 * src/weapons/targets.ts, which places the targets and tests the hits, and
 * src/weapons/armament.ts, which runs the guns and the rounds. Nothing under
 * src/weapons knows that this file exists, so the whole weapon runs in Node
 * with no GPU and the tests prove it there.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * TWO THINGS TO ATTACH, IN TWO DIFFERENT FRAMES
 *
 * `createTargetField` returns a root that stands in the WORLD, and
 * src/world/scene.ts hangs it under the world root.
 *
 * `createWeaponEffects` returns TWO roots. `root` holds the tracers and the
 * bursts, which stand in the world. `muzzleRoot` holds the four flashes, which
 * ride with the aircraft, so src/main.ts hangs it under the render node of the
 * model. A flash drawn in the world frame would lag the nose by one frame of
 * interpolation and would sit behind the muzzle at 250 m/s.
 *
 *
 * WHY ADDITIVE BLENDING AND A BLACK FADE
 *
 * A tracer, a flash and a burst are all light that is added to what is behind
 * them. With additive blending a color of black draws nothing at all, so the
 * fade of every effect is a fade of its instance COLOR toward black. That needs
 * no per instance alpha and no sorting, and one instanced mesh then draws every
 * burst in one call.
 *
 *
 * WHY THE TRACER IS A BOX AND NOT A LINE
 *
 * A line has one pixel of width at any distance, so a tracer 400 m away reads
 * as bright as one 10 m away. A box carries a real size, so it thins out with
 * distance the way a shell does. It also takes the same instanced draw that
 * src/render/force-arrows.ts already uses, so the whole burst is one call.
 */

import type { BufferGeometry, Object3D } from 'three/webgpu';
import {
  AdditiveBlending,
  BoxGeometry,
  Color,
  ConeGeometry,
  CylinderGeometry,
  DynamicDrawUsage,
  Group,
  InstancedMesh,
  Matrix4,
  Mesh,
  MeshBasicNodeMaterial,
  MeshStandardNodeMaterial,
  Quaternion,
  SphereGeometry,
  Vector3,
} from 'three/webgpu';

import { nedToThree } from '@/render/frames';
import type { Armament } from '@/weapons/armament';
import { MAX_ROUNDS } from '@/weapons/armament';
import { muzzleFlash } from '@/weapons/mk108';
import type { Target, TargetKind } from '@/weapons/targets';

// ---------------------------------------------------------------------------
// Shared scratch. The update allocates nothing.
// ---------------------------------------------------------------------------

const scratchMatrix = new Matrix4();
const scratchPosition = new Vector3();
const scratchDirection = new Vector3();
const scratchQuaternion = new Quaternion();
const scratchScale = new Vector3();
const scratchColor = new Color();
const HIDDEN = new Vector3(0, 0, 0);

/** The axis a unit box of this module runs along before it is turned. */
const MODEL_AXIS = new Vector3(0, 0, 1);

// ---------------------------------------------------------------------------
// The targets on the ground
// ---------------------------------------------------------------------------

/** Paint of each kind of target, before it burns. */
const TARGET_COLOR: Readonly<Record<TargetKind, number>> = {
  hangar: 0x8d8b80,
  'parked-aircraft': 0x5f6a4a,
  truck: 0x585d47,
  'fuel-drums': 0x6d6a4e,
};

/** Paint of anything that burned. */
const BURNT_COLOR = 0x1b1815;

/** How far a destroyed target leans over, in radians. */
const WRECK_TILT = 0.28;

/** How far a destroyed target settles into the ground, in meters. */
const WRECK_SINK = 0.35;

export interface TargetField {
  /** Root of every target, in the render frame. */
  root: Object3D;
  /** Move the fires and show any target that was destroyed since the last call. */
  update(targets: readonly Target[], time: number): void;
  dispose(): void;
}

/**
 * Builds the body of one target, standing on `y = 0` and facing `-z`.
 *
 * The half extents arrive in the frame of the target, where `x` runs along the
 * heading and `y` runs across it. The model frame of this project puts the
 * length along `z` and the width along `x`, so the two swap here.
 */
function buildTargetBody(target: Target, material: MeshStandardNodeMaterial): Mesh[] {
  const along = target.halfExtent.x;
  const across = target.halfExtent.y;
  const height = 2 * target.halfExtent.z;
  const parts: Mesh[] = [];

  const add = (
    geometry: BoxGeometry | CylinderGeometry,
    x: number,
    y: number,
    z: number,
  ): Mesh => {
    const mesh = new Mesh(geometry, material);
    mesh.position.set(x, y, z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    parts.push(mesh);
    return mesh;
  };

  switch (target.kind) {
    case 'hangar': {
      // A shed with a curved roof. The doors face the same way the site does.
      const wall = 0.62 * height;
      add(new BoxGeometry(2 * across, wall, 2 * along), 0, 0.5 * wall, 0);
      const roof = new CylinderGeometry(across, across, 2 * along, 14, 1, true, 0, Math.PI);
      roof.rotateZ(Math.PI / 2);
      roof.rotateX(Math.PI / 2);
      add(roof, 0, wall, 0);
      // The door frame, on the front wall.
      add(new BoxGeometry(1.4 * across, 0.8 * wall, 0.5), 0, 0.4 * wall, -along - 0.2);
      break;
    }
    case 'parked-aircraft': {
      // A fighter under a net: a body, a wing, a fin and a tailplane.
      const body = new CylinderGeometry(0.42, 0.24, 1.9 * along, 10);
      body.rotateX(Math.PI / 2);
      add(body, 0, 0.9, 0);
      add(new BoxGeometry(2 * across, 0.16, 0.42 * along), 0, 0.78, 0.1 * along);
      add(new BoxGeometry(0.12, 0.9, 0.5), 0, 1.35, 0.82 * along);
      add(new BoxGeometry(0.8 * across, 0.1, 0.32 * along), 0, 1.0, 0.86 * along);
      // The undercarriage holds it off the grass.
      add(new BoxGeometry(0.18, 0.75, 0.18), -0.35 * across, 0.38, -0.15 * along);
      add(new BoxGeometry(0.18, 0.75, 0.18), 0.35 * across, 0.38, -0.15 * along);
      break;
    }
    case 'truck': {
      add(new BoxGeometry(2 * across, 0.9, 1.25 * along), 0, 1.05, 0.28 * along);
      add(new BoxGeometry(1.7 * across, 0.85, 0.6 * along), 0, 1.0, -0.65 * along);
      add(new BoxGeometry(2 * across, 0.35, 2 * along), 0, 0.55, 0);
      for (const side of [-1, 1]) {
        for (const end of [-0.55, 0.45]) {
          const wheel = new CylinderGeometry(0.42, 0.42, 0.26, 10);
          wheel.rotateZ(Math.PI / 2);
          add(wheel, side * across, 0.42, end * along);
        }
      }
      break;
    }
    case 'fuel-drums': {
      // Four 200 liter drums, stood on end in a square.
      for (const x of [-0.5, 0.5]) {
        for (const z of [-0.5, 0.5]) {
          add(
            new CylinderGeometry(0.29, 0.29, height, 12),
            x * across,
            0.5 * height,
            z * along,
          );
        }
      }
      break;
    }
  }
  return parts;
}

export function createTargetField(targets: readonly Target[]): TargetField {
  const root = new Group();
  root.name = 'targets';

  const paint = new Map<TargetKind, MeshStandardNodeMaterial>();
  for (const kind of Object.keys(TARGET_COLOR) as TargetKind[]) {
    paint.set(
      kind,
      new MeshStandardNodeMaterial({ color: TARGET_COLOR[kind], roughness: 0.85, metalness: 0.1 }),
    );
  }
  const burnt = new MeshStandardNodeMaterial({
    color: BURNT_COLOR,
    roughness: 0.98,
    metalness: 0,
  });
  const flameMaterial = new MeshBasicNodeMaterial({
    color: 0xff7a24,
    transparent: true,
    blending: AdditiveBlending,
    depthWrite: false,
    fog: false,
  });

  /** What the field keeps for one target. */
  interface Entry {
    group: Group;
    parts: Mesh[];
    flame: Mesh;
    shownDestroyed: boolean;
  }

  const entries: Entry[] = [];
  const geometries: BufferGeometry[] = [];

  /** The paint of one kind. Every kind of TARGET_COLOR has one. */
  function paintOf(kind: TargetKind): MeshStandardNodeMaterial {
    const material = paint.get(kind);
    return material !== undefined ? material : burnt;
  }

  for (const target of targets) {
    const group = new Group();
    group.name = `target-${target.kind}`;
    // The box center sits one half height up, so the group stands on the
    // ground under it. src/render/frames.ts owns the axis swap.
    scratchPosition.set(target.position.x, target.position.y, 0);
    nedToThree(scratchPosition, group.position);
    // A model faces north with no turn, and a turn about render +y runs the
    // opposite way round from a NED heading. src/world/scatter.ts does the same.
    group.rotation.y = -target.heading;

    const parts = buildTargetBody(target, paintOf(target.kind));
    for (const part of parts) {
      group.add(part);
      geometries.push(part.geometry);
    }

    const flameGeometry = new ConeGeometry(1.6 * target.halfExtent.y, 5.5, 8, 1, true);
    geometries.push(flameGeometry);
    const flame = new Mesh(flameGeometry, flameMaterial);
    flame.position.y = 2.4;
    flame.visible = false;
    group.add(flame);

    root.add(group);
    entries.push({ group, parts, flame, shownDestroyed: false });
  }

  return {
    root,

    update(list: readonly Target[], time: number): void {
      for (let i = 0; i < entries.length && i < list.length; i++) {
        const entry = entries[i];
        if (list[i].destroyed && !entry.shownDestroyed) {
          entry.shownDestroyed = true;
          for (const part of entry.parts) part.material = burnt;
          // A wreck leans over and settles. The lean is fixed by the index, so
          // it is the same on every run.
          entry.group.rotation.z = i % 2 === 0 ? WRECK_TILT : -WRECK_TILT;
          entry.group.position.y -= WRECK_SINK;
          entry.flame.visible = true;
        } else if (!list[i].destroyed && entry.shownDestroyed) {
          // `resetTargets` put the target back, so the picture must follow it.
          // Without this a respawn leaves the wrecks of the last flight.
          entry.shownDestroyed = false;
          const intact = paintOf(list[i].kind);
          for (const part of entry.parts) part.material = intact;
          entry.group.rotation.z = 0;
          entry.group.position.y += WRECK_SINK;
          entry.flame.visible = false;
        }
        if (!entry.shownDestroyed) continue;
        // The fire flickers. Every wreck keeps its own phase.
        const flicker = 0.8 + 0.2 * Math.sin(time * 9 + i * 1.7);
        entry.flame.scale.set(flicker, 0.85 + 0.25 * flicker, flicker);
      }
    },

    dispose(): void {
      for (const geometry of geometries) geometry.dispose();
      for (const material of paint.values()) material.dispose();
      burnt.dispose();
      flameMaterial.dispose();
      root.removeFromParent();
      root.clear();
    },
  };
}

// ---------------------------------------------------------------------------
// The tracers, the flashes and the bursts
// ---------------------------------------------------------------------------

/** Length of the streak one round leaves, in meters. */
const TRACER_LENGTH = 16;

/**
 * Side of the streak, in meters.
 *
 * The shell is 30 mm across and the streak is 0.34 m, which is eleven times
 * that. The size is deliberate. A tracer is the glow of a burning composition
 * in the base of the shell, not the shell, and that glow is far wider than the
 * body. At 400 m a 30 mm box covers less than one pixel and the burst would
 * appear out of nothing, with no line of fire to read the aim from.
 */
const TRACER_WIDTH = 0.34;

/** Color of a tracer from the upper pair and from the lower pair. */
const TRACER_UPPER = new Color(1.9, 1.5, 0.7);
const TRACER_LOWER = new Color(1.9, 1.0, 0.4);

/** Bursts the effect can show at one time. */
const MAX_BURSTS = 48;

/** How long one burst lives, in seconds. */
const BURST_TIME = 0.45;

/** Radius a burst reaches on a target and on the ground, in meters. */
const BURST_RADIUS_TARGET = 3.4;
const BURST_RADIUS_GROUND = 1.7;

/** Size of the muzzle flash, in meters. */
const FLASH_LENGTH = 1.5;
const FLASH_RADIUS = 0.26;

export interface WeaponEffects {
  /** Tracers and bursts, in the render frame of the WORLD. */
  root: Object3D;
  /** The four muzzle flashes. Attach to the render node of the aircraft. */
  muzzleRoot: Object3D;
  /**
   * Take the impacts of one physics step. Call it from `fixedUpdate`, because
   * a frame can hold several steps and each one reports its own impacts.
   */
  collect(armament: Armament): void;
  /** Draw. `time` is the elapsed time of the world, in seconds. */
  update(armament: Armament, dt: number): void;
  dispose(): void;
}

export function createWeaponEffects(): WeaponEffects {
  // --- the tracers ---
  const tracerGeometry = new BoxGeometry(TRACER_WIDTH, TRACER_WIDTH, 1);
  // The box runs from the round backward, so its z spans 0 to 1 and the turn
  // below points +z along the path the round came from.
  tracerGeometry.translate(0, 0, 0.5);
  const beamMaterial = new MeshBasicNodeMaterial({
    vertexColors: true,
    transparent: true,
    blending: AdditiveBlending,
    depthWrite: false,
    fog: false,
  });
  const tracers = new InstancedMesh(tracerGeometry, beamMaterial, MAX_ROUNDS);
  tracers.instanceMatrix.setUsage(DynamicDrawUsage);
  tracers.frustumCulled = false;
  tracers.count = 0;
  for (let i = 0; i < MAX_ROUNDS; i++) tracers.setColorAt(i, TRACER_UPPER);

  // --- the bursts ---
  const burstGeometry = new SphereGeometry(1, 10, 8);
  const bursts = new InstancedMesh(burstGeometry, beamMaterial, MAX_BURSTS);
  bursts.instanceMatrix.setUsage(DynamicDrawUsage);
  bursts.frustumCulled = false;
  bursts.count = 0;
  for (let i = 0; i < MAX_BURSTS; i++) bursts.setColorAt(i, TRACER_UPPER);

  const root = new Group();
  root.name = 'weapon-effects';
  root.add(tracers);
  root.add(bursts);

  // --- the muzzle flashes ---
  const flashGeometry = new ConeGeometry(FLASH_RADIUS, FLASH_LENGTH, 7, 1, true);
  // A cone points along +y, and a flash points out of the nose, which is
  // render -z. Turn it and slide its base onto the muzzle.
  flashGeometry.rotateX(-Math.PI / 2);
  flashGeometry.translate(0, 0, -0.5 * FLASH_LENGTH);
  const flashes = new InstancedMesh(flashGeometry, beamMaterial, 4);
  flashes.instanceMatrix.setUsage(DynamicDrawUsage);
  flashes.frustumCulled = false;
  flashes.count = 4;
  const muzzleRoot = new Group();
  muzzleRoot.name = 'muzzle-flashes';
  muzzleRoot.add(flashes);

  /** One burst that is still alive. */
  interface Burst {
    position: Vector3;
    radius: number;
    age: number;
  }
  const burstPool: Burst[] = [];
  for (let i = 0; i < MAX_BURSTS; i++) {
    burstPool.push({ position: new Vector3(), radius: 1, age: BURST_TIME });
  }

  function addBurst(nedPosition: Vector3, radius: number): void {
    let oldest = 0;
    for (let i = 0; i < burstPool.length; i++) {
      if (burstPool[i].age >= BURST_TIME) {
        oldest = i;
        break;
      }
      if (burstPool[i].age > burstPool[oldest].age) oldest = i;
    }
    const burst = burstPool[oldest];
    nedToThree(nedPosition, burst.position);
    burst.radius = radius;
    burst.age = 0;
  }

  return {
    root,
    muzzleRoot,

    collect(armament: Armament): void {
      for (let i = 0; i < armament.impactCount; i++) {
        const impact = armament.impacts[i];
        // A hit on a target throws more than a hit on the grass, and the shell
        // that destroys a target throws the most.
        const radius = impact.target < 0
          ? BURST_RADIUS_GROUND
          : impact.destroyed
            ? 1.6 * BURST_RADIUS_TARGET
            : BURST_RADIUS_TARGET;
        addBurst(impact.position, radius);
      }
    },

    update(armament: Armament, dt: number): void {
      // --- the tracers ---
      let drawn = 0;
      for (const round of armament.rounds) {
        if (!round.alive || drawn >= MAX_ROUNDS) continue;
        nedToThree(round.position, scratchPosition);
        // The streak lies behind the round, so it runs against the velocity.
        nedToThree(round.velocity, scratchDirection);
        const speed = scratchDirection.length();
        if (speed <= 0) continue;
        scratchDirection.multiplyScalar(-1 / speed);
        scratchQuaternion.setFromUnitVectors(MODEL_AXIS, scratchDirection);
        // A round near the barrel has no room yet for a full streak.
        const length = Math.min(TRACER_LENGTH, Math.max(1, round.distance));
        scratchScale.set(1, 1, length);
        scratchMatrix.compose(scratchPosition, scratchQuaternion, scratchScale);
        tracers.setMatrixAt(drawn, scratchMatrix);
        tracers.setColorAt(drawn, round.gun < 2 ? TRACER_UPPER : TRACER_LOWER);
        drawn += 1;
      }
      tracers.count = drawn;
      tracers.instanceMatrix.needsUpdate = true;
      if (tracers.instanceColor !== null) tracers.instanceColor.needsUpdate = true;

      // --- the bursts ---
      scratchQuaternion.identity();
      let liveBursts = 0;
      for (const burst of burstPool) {
        if (burst.age >= BURST_TIME) continue;
        burst.age += dt;
        const life = Math.min(1, burst.age / BURST_TIME);
        // The ball opens fast and then fades. Additive blending makes a black
        // color draw nothing, so the fade is a fade of the color.
        const size = burst.radius * (0.35 + 0.65 * Math.sqrt(life));
        scratchScale.set(size, size, size);
        scratchMatrix.compose(burst.position, scratchQuaternion, scratchScale);
        bursts.setMatrixAt(liveBursts, scratchMatrix);
        const fade = (1 - life) * (1 - life);
        scratchColor.setRGB(fade, 0.55 * fade * fade, 0.16 * fade * fade);
        bursts.setColorAt(liveBursts, scratchColor);
        liveBursts += 1;
      }
      bursts.count = liveBursts;
      bursts.instanceMatrix.needsUpdate = true;
      if (bursts.instanceColor !== null) bursts.instanceColor.needsUpdate = true;

      // --- the muzzle flashes ---
      // The body frame maps into the render frame with the same swap the world
      // uses, so nedToThree carries a body position as well.
      for (let i = 0; i < armament.battery.guns.length && i < 4; i++) {
        const gun = armament.battery.guns[i];
        const bright = muzzleFlash(gun);
        if (bright <= 0) {
          scratchMatrix.compose(HIDDEN, scratchQuaternion, HIDDEN);
          flashes.setMatrixAt(i, scratchMatrix);
          continue;
        }
        nedToThree(gun.position, scratchPosition);
        const size = 0.7 + 0.5 * bright;
        scratchScale.set(size, size, 0.6 + 0.9 * bright);
        scratchMatrix.compose(scratchPosition, scratchQuaternion, scratchScale);
        flashes.setMatrixAt(i, scratchMatrix);
        scratchColor.setRGB(bright, 0.72 * bright, 0.3 * bright);
        flashes.setColorAt(i, scratchColor);
      }
      flashes.instanceMatrix.needsUpdate = true;
      if (flashes.instanceColor !== null) flashes.instanceColor.needsUpdate = true;
    },

    dispose(): void {
      tracerGeometry.dispose();
      burstGeometry.dispose();
      flashGeometry.dispose();
      beamMaterial.dispose();
      root.removeFromParent();
      root.clear();
      muzzleRoot.removeFromParent();
      muzzleRoot.clear();
    },
  };
}
