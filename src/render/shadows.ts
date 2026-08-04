/**
 * Cascaded shadow maps for the sun.
 *
 * One shadow map cannot hold a 20 km world. A map that covers the whole world
 * gives one texel per many meters, so the shadow of the aircraft turns into a
 * gray smear. A map that covers only the aircraft loses every far shadow.
 *
 * A cascade set solves this. The camera frustum is cut into slices along the
 * view axis. Each slice gets its own shadow map with its own tight frustum, so
 * the near slice holds many texels per meter and the far slice still reaches
 * the far buildings.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * WHICH CASCADE CODE
 *
 * Three.js ships two cascade modules.
 *
 * `three/addons/csm/CSM.js` drives the WebGL renderer. It patches the GLSL of
 * every material through `Material.onBeforeCompile` and it writes the chunk
 * text in `CSMShader.js`. `WebGPURenderer` builds its shaders from the node
 * graph, so no GLSL chunk of that kind exists and the patch does nothing. That
 * file cannot work here.
 *
 * `three/addons/csm/CSMShadowNode.js` is the node form. Its own header says it
 * works only with `WebGPURenderer`. It extends `ShadowBaseNode`, it builds the
 * cascade selection in TSL, and the light picks it up through
 * `light.shadow.shadowNode`. This module uses that file.
 *
 * Two details of that file matter for this project.
 *
 * 1. It reads `renderer.reversedDepthBuffer` when it builds its main frustum
 *    and it swaps the clip space near and far values when the flag is set.
 *    src/render/renderer.ts asks for the reversed buffer, so this matters. The
 *    file handles it, and it handles the WebGL 2 backend of `WebGPURenderer`
 *    through `renderer.coordinateSystem` as well.
 *
 * 2. It snaps the center of every cascade to whole texels of that cascade
 *    before it places the light. Without the snap the shadow edge crawls while
 *    the camera rolls down the runway. The snap is already in the file, so this
 *    module must not repeat it.
 *
 * The header of that file says "WebGPU only". Bead b51 asked if the node makes
 * the ground black on the WebGL 2 backend. A measurement says no. With the post
 * chain off, the same frame on the WebGPU backend and on the WebGL 2 backend
 * gives the same color in every pixel of the ground, and a hangar casts the
 * same shadow on both. So this module keeps the cascade set on both backends.
 * Do not route the WebGL 2 backend to the fallback below. The fallback is
 * worse, and it would answer a fault that is not there.
 *
 *
 * THE FALLBACK
 *
 * `createShadowRig` builds the cascade node inside a try block. If a later
 * version of the addon fails to build, the module falls back to two shadow
 * casting lights that share the sun direction. One holds a tight frustum near
 * the camera and one holds a wide frustum over the whole shadow distance. The
 * two lights split the sun intensity, so their sum matches the one sun.
 *
 * That fallback is worse than a cascade set. A surface that lies outside the
 * near frustum takes a shadow from the far light alone, so its shadow reaches
 * only half of the full darkness. Read the comment on `createDualFrustumRig`.
 *
 *
 * GPU MEMORY
 *
 * Every cascade holds one depth texture and one color target of
 * `config.render.shadowMapSize` on a side. At the default 2048 that is about
 * 32 MB for each cascade, so three cascades need about 96 MB. Set
 * `shadowsEnabled` to false, or lower `shadowMapSize` to 1024, to cut that.
 */

import { CSMShadowNode } from 'three/addons/csm/CSMShadowNode.js';
import type { Scene, WebGPURenderer } from 'three/webgpu';
import { DirectionalLight, Matrix4, Vector3 } from 'three/webgpu';

import { config } from '@/core/config';

/**
 * Number of cascades. Three slices keep the shadow under the aircraft sharp
 * and still reach `config.render.shadowDistance`. A fourth slice would add
 * another full shadow map for a small gain, so this module stops at three.
 */
const CASCADE_COUNT = 3;

/**
 * Distance the cascade light moves back from its slice, in meters. It must
 * clear every object that stands between the sun and the slice.
 */
const LIGHT_MARGIN = 120;

/** Near plane of every shadow camera, in meters. */
const SHADOW_CAMERA_NEAR = 1;

/**
 * Far plane of every shadow camera, in meters.
 *
 * The last cascade holds the widest box. At `shadowDistance` of 600 m that box
 * is about 1270 m across and its depth along the sun ray stays below 1200 m.
 * A larger far plane only wastes depth precision, because `shadow.bias` counts
 * in normalized depth and its world size grows with this range.
 */
const SHADOW_CAMERA_FAR = 2.5 * config.render.shadowDistance;

/** Distance of the sun light from its target, in meters. */
const RIG_DISTANCE = 3000;

/** Half width of the near frustum of the fallback rig, in meters. */
const NEAR_FRUSTUM_EXTENT = 60;

/** Up vector of the light basis. The code picks another one near the zenith. */
const LIGHT_UP = new Vector3(0, 1, 0);
const LIGHT_UP_ALTERNATE = new Vector3(0, 0, 1);

/** Largest safe value of the dot product of the sun ray and the up vector. */
const UP_PARALLEL_LIMIT = 0.99;

export interface ShadowRig {
  /**
   * Move the shadow volume. `cameraPosition` and `sunDirection` are both in the
   * render frame. `sunDirection` points the way the sunlight travels, so it
   * points away from the sun and toward the ground.
   */
  update(cameraPosition: Vector3, sunDirection: Vector3): void;

  /** Turn every shadow on or off. Use it to measure the cost of shadows. */
  enabled: boolean;

  readonly cascadeCount: number;
  readonly method: 'csm' | 'dual-frustum';
  dispose(): void;
}

/** Scratch values. The module allocates them one time. */
const lightBasis = new Matrix4();
const lightBasisInverse = new Matrix4();
const snapOrigin = new Vector3();
const snapCenter = new Vector3();
const snapTarget = new Vector3();

/** Give one directional light the frustum of a square shadow volume. */
function setShadowExtent(light: DirectionalLight, extent: number): void {
  light.shadow.camera.left = -extent;
  light.shadow.camera.right = extent;
  light.shadow.camera.top = extent;
  light.shadow.camera.bottom = -extent;
  light.shadow.camera.near = SHADOW_CAMERA_NEAR;
  light.shadow.camera.far = SHADOW_CAMERA_FAR;
  light.shadow.camera.updateProjectionMatrix();
}

/**
 * Place one directional light so that its shadow volume sits over `center` and
 * so that its origin lands on a whole texel of its own shadow map.
 *
 * The snap is the reason this function exists. The shadow map is a grid fixed
 * to the light. When the volume slides by a fraction of a texel, every depth
 * sample lands on a new part of the grid and the shadow edge shimmers. The fix
 * moves the volume only by whole texels.
 */
function placeSnapped(
  light: DirectionalLight,
  center: Vector3,
  sunDirection: Vector3,
  extent: number,
): void {
  const up =
    Math.abs(sunDirection.dot(LIGHT_UP)) > UP_PARALLEL_LIMIT ? LIGHT_UP_ALTERNATE : LIGHT_UP;

  // A look-at matrix with the eye at the origin holds rotation alone, so it
  // maps a light space direction into the render frame and back.
  snapOrigin.set(0, 0, 0);
  snapTarget.copy(sunDirection);
  lightBasis.lookAt(snapOrigin, snapTarget, up);
  lightBasisInverse.copy(lightBasis).invert();

  const texel = (2 * extent) / light.shadow.mapSize.width;

  snapCenter.copy(center).applyMatrix4(lightBasisInverse);
  snapCenter.x = Math.floor(snapCenter.x / texel) * texel;
  snapCenter.y = Math.floor(snapCenter.y / texel) * texel;
  snapCenter.applyMatrix4(lightBasis);

  light.target.position.copy(snapCenter);
  light.position.copy(snapCenter).addScaledVector(sunDirection, -RIG_DISTANCE);
  light.target.updateMatrixWorld();
  light.updateMatrixWorld();
}

/**
 * Build the cascade rig. Returns null when the addon fails to build, and the
 * caller then takes the fallback.
 */
function createCsmRig(
  renderer: WebGPURenderer,
  sun: DirectionalLight,
): ShadowRig | null {
  let csm: CSMShadowNode;
  try {
    csm = new CSMShadowNode(sun, {
      cascades: CASCADE_COUNT,
      // The cascade set covers the camera near plane out to this distance. The
      // far ground beyond it takes no shadow, which the fog already hides.
      maxFar: config.render.shadowDistance,
      // The practical split mixes the uniform split and the logarithmic split.
      // A pure logarithmic split wastes the first cascade on the first meters,
      // because the camera near plane sits at 0.3 m.
      mode: 'practical',
      lightMargin: LIGHT_MARGIN,
    });
  } catch {
    return null;
  }

  // The node clones `sun.shadow` one time for each cascade, so every value the
  // cascades need must sit on the sun before the first frame.
  sun.shadow.shadowNode = csm;

  let enabled: boolean = config.render.shadowsEnabled;
  const lastProjection = new Float64Array(4);

  return {
    cascadeCount: CASCADE_COUNT,
    method: 'csm',

    get enabled(): boolean {
      return enabled;
    },

    set enabled(value: boolean) {
      if (value === enabled) return;
      enabled = value;
      sun.castShadow = value;
      renderer.shadowMap.enabled = value;
    },

    update(cameraPosition: Vector3, sunDirection: Vector3): void {
      if (!enabled) return;

      // The cascade node reads the sun direction from the light and its target
      // on every frame, and it places its own cascade lights. So the rig only
      // has to hold the direction and stay near the camera, which keeps the
      // float values small.
      sun.target.position.set(cameraPosition.x, 0, cameraPosition.z);
      sun.position.copy(sun.target.position).addScaledVector(sunDirection, -RIG_DISTANCE);

      // The cascade splits come from the camera projection. The node reads the
      // projection one time, so a change of the field of view or of the aspect
      // needs a rebuild. Elements 0, 5, 10 and 14 carry the aspect, the field
      // of view and the depth range. They do not carry a view offset, so a
      // jittered projection does not force a rebuild every frame.
      const camera = csm.camera;
      if (camera === null) return;
      const e = camera.projectionMatrix.elements;
      if (
        lastProjection[0] !== e[0] ||
        lastProjection[1] !== e[5] ||
        lastProjection[2] !== e[10] ||
        lastProjection[3] !== e[14]
      ) {
        lastProjection[0] = e[0];
        lastProjection[1] = e[5];
        lastProjection[2] = e[10];
        lastProjection[3] = e[14];
        csm.updateFrustums();
      }
    },

    dispose(): void {
      sun.shadow.shadowNode = undefined;
      csm.dispose();
    },
  };
}

/**
 * Build the fallback rig of two shadow casting lights.
 *
 * Both lights point along the sun ray. The near light holds a small frustum
 * that follows the camera and gives a sharp contact shadow. The far light
 * holds a frustum of `config.render.shadowDistance` and catches the distant
 * buildings.
 *
 * The two lights split the sun intensity in half, because two lights of the
 * full intensity would light the scene twice. The split has one visible cost.
 * A surface outside the near frustum passes the frustum test of the near light,
 * which then reports full light, so its shadow only reaches half darkness. A
 * true cascade set has no such seam. This path exists only for the case where
 * the cascade addon fails to build.
 */
function createDualFrustumRig(
  renderer: WebGPURenderer,
  scene: Scene,
  sun: DirectionalLight,
): ShadowRig {
  const far = new DirectionalLight(0xffffff, 0);
  far.name = 'sun-far-shadow';
  far.castShadow = config.render.shadowsEnabled;
  far.shadow.mapSize.copy(sun.shadow.mapSize);
  far.shadow.bias = config.render.shadowBias;
  setShadowExtent(far, config.render.shadowDistance);
  setShadowExtent(sun, NEAR_FRUSTUM_EXTENT);

  const parent = sun.parent !== null ? sun.parent : scene;
  parent.add(far);
  parent.add(far.target);

  let enabled: boolean = config.render.shadowsEnabled;
  let sunIntensity = sun.intensity;

  return {
    cascadeCount: 2,
    method: 'dual-frustum',

    get enabled(): boolean {
      return enabled;
    },

    set enabled(value: boolean) {
      if (value === enabled) return;
      enabled = value;
      sun.castShadow = value;
      far.castShadow = value;
      renderer.shadowMap.enabled = value;
    },

    update(cameraPosition: Vector3, sunDirection: Vector3): void {
      // src/render/sky.ts writes the whole sun intensity when the sun moves.
      // Read it back, halve it, and give the other half to the far light.
      const total = sun.intensity + far.intensity;
      if (Math.abs(total - sunIntensity) > 1e-9) {
        sunIntensity = total;
        sun.intensity = 0.5 * total;
        far.intensity = 0.5 * total;
        far.color.copy(sun.color);
      }

      placeSnapped(sun, cameraPosition, sunDirection, NEAR_FRUSTUM_EXTENT);

      // The far volume rides at ground level under the camera, so it holds the
      // same ground for a climb and for a low pass.
      snapTarget.set(cameraPosition.x, 0, cameraPosition.z);
      placeSnapped(far, snapTarget, sunDirection, config.render.shadowDistance);
    },

    dispose(): void {
      far.shadow.dispose();
      far.target.removeFromParent();
      far.removeFromParent();
    },
  };
}

/**
 * Build the shadow rig for the sun.
 *
 * The function writes `renderer.shadowMap.enabled` and it writes the shadow
 * settings of `sun`. src/render/sky.ts already made the light and gave it the
 * first shadow settings. This module raises those settings to the cascade set,
 * so it must run after `createSky`.
 */
export function createShadowRig(
  renderer: WebGPURenderer,
  scene: Scene,
  sun: DirectionalLight,
): ShadowRig {
  // The cascade node adds one helper light for each cascade to the parent of
  // the sun. A sun with no parent would drop every cascade, so put it in the
  // scene before the first frame.
  if (sun.parent === null) {
    scene.add(sun);
    scene.add(sun.target);
  }

  renderer.shadowMap.enabled = config.render.shadowsEnabled;
  sun.castShadow = config.render.shadowsEnabled;
  sun.shadow.mapSize.set(config.render.shadowMapSize, config.render.shadowMapSize);

  // The cascade node multiplies this value by the index of the cascade, so the
  // wide far cascades take more bias than the tight near cascade.
  sun.shadow.bias = config.render.shadowBias;
  setShadowExtent(sun, config.render.shadowDistance);

  const csmRig = createCsmRig(renderer, sun);
  if (csmRig !== null) return csmRig;

  return createDualFrustumRig(renderer, scene, sun);
}
