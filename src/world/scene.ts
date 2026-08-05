/**
 * World assembly.
 *
 * The module puts the sky, the ground, and the runway under one root object and
 * gives the caller one update call. It holds no physics and no aircraft.
 *
 * The world root sits in the render frame. Every position that comes from the
 * physics must pass through src/render/frames.ts first.
 */

import type { Object3D, Scene, Vector3, WebGPURenderer } from 'three/webgpu';
import { Group } from 'three/webgpu';

import { config } from '@/core/config';
import type { CloudLayer } from '@/render/clouds';
import { createClouds } from '@/render/clouds';
import { createGround } from '@/render/ground';
import type { ShadowRig } from '@/render/shadows';
import { createShadowRig } from '@/render/shadows';
import type { SkyBundle } from '@/render/sky';
import { createSky } from '@/render/sky';
import { createTargetField } from '@/render/weapons';
import type { Target } from '@/weapons/targets';
import { placeTargets } from '@/weapons/targets';
import { createRunway } from '@/world/runway';
import { createScatteredWorld } from '@/world/scatter';

export interface World {
  /** Root of every world object, in the render frame. */
  root: Object3D;

  /** The sky, the sun light, the environment map, and the fog. */
  sky: SkyBundle;

  /** The cloud deck. It reads the same sun and the same haze the sky uses. */
  clouds: CloudLayer;

  /** The cascade shadow rig of the sun. Set `enabled` to compare the cost. */
  shadows: ShadowRig;

  /**
   * The ground targets, in world NED. src/main.ts hands the same list to
   * src/weapons/armament.ts, so the box a shell tests against is the box the
   * model stands in.
   */
  targets: readonly Target[];

  /** Advance the world. `cameraPosition` is in the render frame. */
  update(dt: number, cameraPosition: Vector3): void;

  dispose(): void;
}

export function createWorld(renderer: WebGPURenderer, scene: Scene): World {
  const root = new Group();
  root.name = 'world';

  const sky = createSky(renderer, scene);
  const ground = createGround();
  const runway = createRunway();

  // The deck must come after the sky, because it takes the clouds of the sky
  // dome away. Read section 5 of src/render/clouds.ts.
  const clouds = createClouds(renderer, scene, sky.sky);

  root.add(sky.sky);
  root.add(sky.sun);
  // The light aims at its target, so the target needs a place in the graph.
  root.add(sky.sun.target);
  root.add(ground.mesh);
  root.add(runway.group);
  root.add(clouds.mesh);

  // Buildings and trees join here. Add their groups to `root` and call their
  // update from `update` below. Keep the sky first, so it stays the background.
  const scatter = createScatteredWorld();
  root.add(scatter.root);
  let scatterTime = 0;

  // The ground targets. src/weapons/targets.ts places them with the same clear
  // zone and approach corridor rules the scatter uses, so nothing stands where
  // the aircraft flies.
  const targets = placeTargets();
  const targetField = createTargetField(targets);
  root.add(targetField.root);

  scene.add(root);

  renderer.shadowMap.enabled = config.render.shadowsEnabled;

  // The rig raises the one shadow map of the sun to a cascade set. It reads the
  // light that createSky built, so it must come after that call.
  const shadows = createShadowRig(renderer, scene, sky.sun);

  // Offset from the light target to the light. It stays fixed while the sun
  // angles stay fixed, so the rig can slide with the camera and keep its angle.
  const rigOffset = sky.sun.position.clone().sub(sky.sun.target.position);

  return {
    root,
    sky,
    clouds,
    shadows,
    targets,

    update(dt: number, cameraPosition: Vector3): void {
      // The sky box is a background, so it rides with the camera. The shader
      // reads the view direction, so the move changes no color.
      sky.sky.position.copy(cameraPosition);

      // Sort the buildings and the trees into their levels of detail, and move
      // the wind in the trees.
      scatterTime += dt;
      scatter.update(cameraPosition, scatterTime);

      // Show any target that burned since the last frame, and move the fires.
      targetField.update(targets, scatterTime);

      // Keep the shadow volume over the camera. The rig angle does not change.
      rigOffset.copy(sky.sun.position).sub(sky.sun.target.position);
      sky.sun.target.position.set(cameraPosition.x, 0, cameraPosition.z);
      sky.sun.position.copy(sky.sun.target.position).add(rigOffset);

      // The cascade rig places the light of every cascade from here.
      shadows.update(cameraPosition, sky.sunDirection);

      // The deck rides with the camera and it reads the sun of this frame.
      clouds.update(dt, cameraPosition, sky.sun, sky.sunDirection);

      // Rebuilds the environment map only after `setSunAngles` moved the sun.
      sky.update(renderer, scene);
    },

    dispose(): void {
      clouds.dispose();
      shadows.dispose();
      targetField.dispose();
      scatter.dispose();
      runway.dispose();
      ground.dispose();
      sky.dispose();
      root.removeFromParent();
      root.clear();
    },
  };
}
