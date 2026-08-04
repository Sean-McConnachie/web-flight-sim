/**
 * Sky dome, sun light, environment map, and fog.
 *
 * The module builds one sky dome, one directional light, one image based
 * lighting map, and the scene fog. All four read the same sun angle, so the
 * picture stays consistent when the sun moves.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * WHICH SKY
 *
 * Three.js ships `SkyMesh` in `examples/jsm/objects/SkyMesh.js`. That class is
 * the WebGPU form of the Preetham analytic daylight model, and it is already
 * written in TSL. This module uses it. No hand written scattering shader was
 * needed. `Sky.js` is the older WebGL form and this project does not use it.
 *
 *
 * WHY THE SKY DRAWS FIRST AND IGNORES DEPTH
 *
 * The vertex stage of `SkyMesh` writes `position.z = position.w`, which places
 * every sky fragment at normalized depth 1. With a normal depth buffer that is
 * the far plane. src/render/renderer.ts asks for a reversed depth buffer, where
 * depth 1 is the NEAR plane, so the sky would cover the world.
 *
 * The fix keeps the sky as a pure background. The sky material turns off both
 * the depth test and the depth write, and the mesh takes the first place in the
 * render order. The sky then draws before every other object and every other
 * object draws over it. The result is correct with either depth buffer.
 *
 *
 * WHY THE RENDER ORDER CHANGES SIGN
 *
 * `RenderList.sort` of Three.js sorts the opaque list by the render order and
 * then calls `reverse()` on the whole list when the camera holds a reversed
 * depth buffer. The reverse keeps the front to back order that early depth
 * rejection wants, because a reversed buffer also turns the depth values
 * around. It turns the render order around as well.
 *
 * So a render order of -1 puts the sky LAST with a reversed depth buffer. The
 * sky has no depth test, so it then paints over the ground, the runway and
 * every building on every frame. The picture becomes a plain sky dome.
 *
 * `SKY_RENDER_ORDER` therefore changes sign with `renderer.reversedDepthBuffer`.
 * The sky draws first with either buffer. Read the note on the depth buffer in
 * src/render/renderer.ts for the reason the project asks for the reversed one.
 *
 *
 * WHY THE CPU REPEATS THE SKY EQUATIONS
 *
 * The fog color and the sun light color must match the sky that the shader
 * draws. This module evaluates the same Preetham equations on the CPU, with the
 * same constants and the same 0.04 radiance scale that `SkyMesh` applies. The
 * renderer tone maps the fog color and the sky through the same curve, because
 * the fog mixes into the material output before tone mapping. So a fog color
 * taken from the horizon radiance lands on the same screen color as the sky
 * just above the horizon.
 */

import { SkyMesh } from 'three/addons/objects/SkyMesh.js';
import type { Object3D, RenderTarget, WebGPURenderer } from 'three/webgpu';
import {
  Color,
  DirectionalLight,
  FogExp2,
  LinearSRGBColorSpace,
  PMREMGenerator,
  Scene,
  Vector3,
} from 'three/webgpu';

import { config } from '@/core/config';
import { nedToThree } from '@/render/frames';

/**
 * Preetham model constants. Every value matches the value inside SkyMesh, so
 * the CPU result and the shader result agree.
 * Source: Preetham, Shirley and Smits, "A Practical Analytic Model for
 * Daylight", SIGGRAPH 1999, confidence: firm.
 */
const TOTAL_RAYLEIGH_R = 5.804542996261093e-6;
const TOTAL_RAYLEIGH_G = 1.3562911419845635e-5;
const TOTAL_RAYLEIGH_B = 3.0265902468824876e-5;
const MIE_CONST_R = 1.8399918514433978e14;
const MIE_CONST_G = 2.7798023919660528e14;
const MIE_CONST_B = 4.0790479543861094e14;
const CUTOFF_ANGLE = 1.6110731556870734; // rad, the earth shadow hack of the model
const STEEPNESS = 1.5;
const SUN_ENERGY = 1000;
const RAYLEIGH_ZENITH_LENGTH = 8.4e3; // m
const MIE_ZENITH_LENGTH = 1.25e3; // m
const THREE_OVER_SIXTEEN_PI = 0.05968310365946075;
const ONE_OVER_FOUR_PI = 0.07957747154594767;

/** Sky dome settings. A low turbidity gives the clear air of a good flying day. */
const SKY_TURBIDITY = 2;
const SKY_RAYLEIGH = 1;
const SKY_MIE_COEFFICIENT = 0.005;
const SKY_MIE_DIRECTIONAL_G = 0.8;

/** Cloud settings of SkyMesh. The clouds give the eye a height reference. */
const CLOUD_COVERAGE = 0.35;
const CLOUD_DENSITY = 0.4;
const CLOUD_ELEVATION = 0.5;

/** Radiance scale that SkyMesh applies to the Preetham result. */
const SKY_RADIANCE_SCALE = 0.04;

/** Radiance multiplier of the solar disc inside SkyMesh. */
const SUN_DISC_RADIANCE = 19000;

/**
 * Solid angle of the sun disc, in steradian. The angular diameter is 0.53 deg,
 * so the half angle is 0.004625 rad and the solid angle is pi times its square.
 * Source: standard solar angular diameter, confidence: firm.
 */
const SUN_SOLID_ANGLE = 6.72e-5;

/** Side of the sky box, in meters. The box follows the camera, so this is ample. */
const SKY_SIZE = 20000;

/**
 * Render order of the sky with a normal depth buffer. Every other object holds
 * the default order of 0, so a negative value draws the sky first. Read the
 * note on the sign above.
 */
const SKY_RENDER_ORDER = -1;

/** Elevation of the sample ring that gives the fog color, in degrees. */
const HORIZON_SAMPLE_ELEVATION_DEG = 0.75;

/** Number of azimuth samples in that ring. The fog holds one color for every view. */
const HORIZON_SAMPLE_COUNT = 16;

/** Side of the environment map cube face, in texels. */
const ENVIRONMENT_SIZE = 256;

/** Distance of the light rig from its target, in meters. */
const SUN_RIG_DISTANCE = 3000;

/** Default sun angles. A clear mid morning sun, in the southeast. */
const DEFAULT_SUN_ELEVATION_DEG = 45;
const DEFAULT_SUN_AZIMUTH_DEG = 135;

/** Sun elevation below which the sun light goes out, in degrees. */
const MIN_SUN_ELEVATION_DEG = 0.05;

/** Fog thickness the fit asks for at config.render.fogNear. */
const FOG_FRACTION_AT_NEAR = 0.03;

/** Fog thickness the fit asks for at config.render.fogFar. */
const FOG_FRACTION_AT_FAR = 0.98;

const DEG_TO_RAD = Math.PI / 180;

export interface SkyBundle {
  /** The sky dome. Keep it centered on the camera. */
  sky: Object3D;

  /**
   * The sun light. Add both the light and `sun.target` to the scene graph. The
   * light aims from its own position at the target.
   */
  sun: DirectionalLight;

  /**
   * Direction of travel of the sunlight, in the render frame. The vector points
   * away from the sun and toward the ground. Multiply it by -1 to get the
   * direction from a surface toward the sun.
   */
  sunDirection: Vector3;

  /** Move the sun. Elevation is degrees above the horizon. Azimuth is degrees
   * clockwise from north, so 90 is east and 180 is south. */
  setSunAngles(elevationDeg: number, azimuthDeg: number): void;

  /** Rebuild the environment map when the sun moved. Cheap on every other frame. */
  update(renderer: WebGPURenderer, scene: Scene): void;

  dispose(): void;
}

/**
 * Fog density of the exponential squared fog, in 1 per meter.
 *
 * The fog has one parameter and the configuration gives two distances. A fit to
 * `fogNear` alone leaves the far ground visible. A fit to `fogFar` alone makes
 * the near air too clear. The code takes the geometric mean of the two fits, so
 * both distances get close to the value the configuration asks for.
 *
 * The fog factor of the model is `1 - exp(-(density * distance)^2)`, so the
 * density that gives a factor `f` at a distance `d` is `sqrt(-ln(1 - f)) / d`.
 */
const FOG_DENSITY = Math.sqrt(
  (Math.sqrt(-Math.log(1 - FOG_FRACTION_AT_NEAR)) / config.render.fogNear) *
    (Math.sqrt(-Math.log(1 - FOG_FRACTION_AT_FAR)) / config.render.fogFar),
);

/**
 * Horizon color of the sky, in linear sRGB. src/render/ground.ts reads it, so
 * the far edge of the ground plane can fade into the same color that the fog
 * and the sky show there. The module updates the color in place when the sun
 * moves, and the uniform that reads it sees the new value on the next frame.
 */
export const horizonColor = new Color(1, 1, 1);

/** Scratch state of the Preetham model for the current sun elevation. */
let sunIntensity = 0;
let betaR0 = 0;
let betaR1 = 0;
let betaR2 = 0;
let betaM0 = 0;
let betaM1 = 0;
let betaM2 = 0;

/** Scratch output of `evaluateSky`. The module allocates it one time. */
const skySample = new Vector3();

/** Scratch vectors that hold the sun direction in both frames. */
const sunNed = new Vector3();
const toSunThree = new Vector3();

/**
 * Fill the scattering state for one sun elevation. The steps repeat the vertex
 * stage of SkyMesh.
 *
 * `sunfade` reads the y value of the sun position uniform, which this module
 * always sets to a unit vector. The exponent is then near zero, the clamp gives
 * zero, and `sunfade` is one. So the Rayleigh coefficient is the plain
 * `SKY_RAYLEIGH` value and the code writes that result directly.
 */
function setSunElevation(elevationRad: number): void {
  const zenithAngleCos = Math.min(1, Math.max(-1, Math.sin(elevationRad)));
  sunIntensity =
    SUN_ENERGY *
    Math.max(0, 1 - Math.exp(-(CUTOFF_ANGLE - Math.acos(zenithAngleCos)) / STEEPNESS));

  betaR0 = TOTAL_RAYLEIGH_R * SKY_RAYLEIGH;
  betaR1 = TOTAL_RAYLEIGH_G * SKY_RAYLEIGH;
  betaR2 = TOTAL_RAYLEIGH_B * SKY_RAYLEIGH;

  // c = 0.2 * turbidity * 10E-18, then totalMie = 0.434 * c * MieConst.
  const mie = 0.434 * (0.2 * SKY_TURBIDITY * 1e-17) * SKY_MIE_COEFFICIENT;
  betaM0 = mie * MIE_CONST_R;
  betaM1 = mie * MIE_CONST_G;
  betaM2 = mie * MIE_CONST_B;
}

/**
 * Optical depth factor for one view elevation. The formula avoids the
 * singularity at the horizon, exactly as the shader does.
 */
function opticalInverse(viewElevationRad: number): number {
  const zenithAngle = Math.acos(Math.max(0, Math.sin(viewElevationRad)));
  const zenithDeg = (zenithAngle * 180) / Math.PI;
  return 1 / (Math.cos(zenithAngle) + 0.15 * Math.pow(93.885 - zenithDeg, -1.253));
}

/**
 * Sky radiance for one view direction, after the 0.04 scale of SkyMesh. The
 * direction is given by its elevation and by its azimuth relative to the sun.
 * The solar disc term is left out, because the caller never samples the disc.
 */
function evaluateSky(
  sunElevationRad: number,
  viewElevationRad: number,
  relativeAzimuthRad: number,
  out: Vector3,
): Vector3 {
  const inverse = opticalInverse(viewElevationRad);
  const sR = RAYLEIGH_ZENITH_LENGTH * inverse;
  const sM = MIE_ZENITH_LENGTH * inverse;

  const fex0 = Math.exp(-(betaR0 * sR + betaM0 * sM));
  const fex1 = Math.exp(-(betaR1 * sR + betaM1 * sM));
  const fex2 = Math.exp(-(betaR2 * sR + betaM2 * sM));

  // Angle between the view direction and the sun, with both on the unit sphere.
  const cosTheta =
    Math.cos(viewElevationRad) * Math.cos(sunElevationRad) * Math.cos(relativeAzimuthRad) +
    Math.sin(viewElevationRad) * Math.sin(sunElevationRad);

  const c = cosTheta * 0.5 + 0.5;
  const rPhase = THREE_OVER_SIXTEEN_PI * (1 + c * c);
  const g2 = SKY_MIE_DIRECTIONAL_G * SKY_MIE_DIRECTIONAL_G;
  const mPhase =
    (ONE_OVER_FOUR_PI * (1 - g2)) /
    Math.pow(1 - 2 * SKY_MIE_DIRECTIONAL_G * cosTheta + g2, 1.5);

  const ratio0 = (betaR0 * rPhase + betaM0 * mPhase) / (betaR0 + betaM0);
  const ratio1 = (betaR1 * rPhase + betaM1 * mPhase) / (betaR1 + betaM1);
  const ratio2 = (betaR2 * rPhase + betaM2 * mPhase) / (betaR2 + betaM2);

  let lin0 = Math.pow(sunIntensity * ratio0 * (1 - fex0), 1.5);
  let lin1 = Math.pow(sunIntensity * ratio1 * (1 - fex1), 1.5);
  let lin2 = Math.pow(sunIntensity * ratio2 * (1 - fex2), 1.5);

  // The model desaturates the sky as the sun falls toward the horizon.
  const blend = Math.min(1, Math.max(0, Math.pow(1 - Math.sin(sunElevationRad), 5)));
  lin0 *= 1 - blend + Math.sqrt(sunIntensity * ratio0 * fex0) * blend;
  lin1 *= 1 - blend + Math.sqrt(sunIntensity * ratio1 * fex1) * blend;
  lin2 *= 1 - blend + Math.sqrt(sunIntensity * ratio2 * fex2) * blend;

  // Night sky term, then the constant lift that SkyMesh adds.
  return out.set(
    (lin0 + 0.1 * fex0) * SKY_RADIANCE_SCALE,
    (lin1 + 0.1 * fex1) * SKY_RADIANCE_SCALE + 0.0003,
    (lin2 + 0.1 * fex2) * SKY_RADIANCE_SCALE + 0.00075,
  );
}

/**
 * Mean radiance of a ring of directions just above the horizon. The fog holds
 * one color for every view direction, so the mean is the honest choice.
 */
function computeHorizonColor(sunElevationRad: number, out: Color): void {
  let r = 0;
  let g = 0;
  let b = 0;
  const viewElevation = HORIZON_SAMPLE_ELEVATION_DEG * DEG_TO_RAD;
  for (let i = 0; i < HORIZON_SAMPLE_COUNT; i += 1) {
    const azimuth = (2 * Math.PI * i) / HORIZON_SAMPLE_COUNT;
    evaluateSky(sunElevationRad, viewElevation, azimuth, skySample);
    r += skySample.x;
    g += skySample.y;
    b += skySample.z;
  }
  out.setRGB(
    r / HORIZON_SAMPLE_COUNT,
    g / HORIZON_SAMPLE_COUNT,
    b / HORIZON_SAMPLE_COUNT,
    LinearSRGBColorSpace,
  );
}

/**
 * Set the color and the strength of the sun light from the sun elevation.
 *
 * The color is the atmospheric transmittance along the sun ray, normalized so
 * its largest part is one. Red passes through more air than blue, so a low sun
 * turns warm on its own. No hand picked color ramp is needed.
 *
 * The strength is the irradiance of the solar disc that SkyMesh draws. The disc
 * radiance is `sunIntensity * 19000 * 0.04` and the disc covers
 * `SUN_SOLID_ANGLE` steradian, so their product is the irradiance. The light
 * then agrees with the disc that a pilot sees in the same picture.
 */
function setSunLight(sun: DirectionalLight, elevationDeg: number): void {
  if (elevationDeg <= MIN_SUN_ELEVATION_DEG) {
    sun.color.setRGB(1, 1, 1, LinearSRGBColorSpace);
    sun.intensity = 0;
    return;
  }

  const inverse = opticalInverse(elevationDeg * DEG_TO_RAD);
  const sR = RAYLEIGH_ZENITH_LENGTH * inverse;
  const sM = MIE_ZENITH_LENGTH * inverse;
  const fex0 = Math.exp(-(betaR0 * sR + betaM0 * sM));
  const fex1 = Math.exp(-(betaR1 * sR + betaM1 * sM));
  const fex2 = Math.exp(-(betaR2 * sR + betaM2 * sM));

  const peak = Math.max(fex0, fex1, fex2);
  if (peak <= 0) {
    sun.color.setRGB(1, 1, 1, LinearSRGBColorSpace);
    sun.intensity = 0;
    return;
  }

  sun.color.setRGB(fex0 / peak, fex1 / peak, fex2 / peak, LinearSRGBColorSpace);
  sun.intensity =
    sunIntensity * SUN_DISC_RADIANCE * SKY_RADIANCE_SCALE * SUN_SOLID_ANGLE * peak;
}

/**
 * Build the sky, the sun light, the environment map, and the fog. The function
 * writes `scene.fog` and `scene.environment`.
 */
export function createSky(renderer: WebGPURenderer, scene: Scene): SkyBundle {
  const sky = new SkyMesh();
  sky.scale.setScalar(SKY_SIZE);
  sky.turbidity.value = SKY_TURBIDITY;
  sky.rayleigh.value = SKY_RAYLEIGH;
  sky.mieCoefficient.value = SKY_MIE_COEFFICIENT;
  sky.mieDirectionalG.value = SKY_MIE_DIRECTIONAL_G;
  sky.cloudCoverage.value = CLOUD_COVERAGE;
  sky.cloudDensity.value = CLOUD_DENSITY;
  sky.cloudElevation.value = CLOUD_ELEVATION;

  // Read the module comment. The sky is a background, not an object with depth.
  sky.material.depthTest = false;
  sky.material.depthWrite = false;
  sky.renderOrder = renderer.reversedDepthBuffer ? -SKY_RENDER_ORDER : SKY_RENDER_ORDER;
  sky.frustumCulled = false;

  // The fog must not touch the sky. The fog reads the view space depth of the
  // real box surface, which is thousands of meters away, so the sky would take
  // a third of the fog color and the zenith would turn pale. The fog exists to
  // hide the ground, and the sky is what the ground fades into.
  sky.material.fog = false;

  const sun = new DirectionalLight(0xffffff, 1);
  sun.castShadow = config.render.shadowsEnabled;
  sun.shadow.mapSize.set(config.render.shadowMapSize, config.render.shadowMapSize);
  sun.shadow.bias = config.render.shadowBias;
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 2 * SUN_RIG_DISTANCE;
  sun.shadow.camera.left = -config.render.shadowDistance;
  sun.shadow.camera.right = config.render.shadowDistance;
  sun.shadow.camera.top = config.render.shadowDistance;
  sun.shadow.camera.bottom = -config.render.shadowDistance;
  sun.shadow.camera.updateProjectionMatrix();

  // The light aims from its own position at `sun.target`. The target must sit
  // in the scene graph as a sibling, never as a child of the light, because a
  // child at the light origin would give a direction of zero length. The caller
  // adds `sun.target` to the same parent as the light.
  sun.target.position.set(0, 0, 0);

  const fog = new FogExp2(0xffffff, FOG_DENSITY);
  scene.fog = fog;

  // The generator binds to one renderer. It is built on the first use, from the
  // renderer that the caller passes in, so `update` really reads its argument.
  let pmrem: PMREMGenerator | null = null;
  const environmentScene = new Scene();
  let environmentTarget: RenderTarget | null = null;

  const sunDirection = new Vector3(0, -1, 0);
  let elevationDeg = DEFAULT_SUN_ELEVATION_DEG;
  let azimuthDeg = DEFAULT_SUN_AZIMUTH_DEG;
  let sunMoved = true;

  function setSunAngles(nextElevationDeg: number, nextAzimuthDeg: number): void {
    elevationDeg = nextElevationDeg;
    azimuthDeg = nextAzimuthDeg;

    const elevationRad = elevationDeg * DEG_TO_RAD;
    const azimuthRad = azimuthDeg * DEG_TO_RAD;

    // Build the direction toward the sun in NED, then let src/render/frames.ts
    // map it into the render frame. No other file may write that map.
    sunNed.set(
      Math.cos(elevationRad) * Math.cos(azimuthRad),
      Math.cos(elevationRad) * Math.sin(azimuthRad),
      -Math.sin(elevationRad),
    );
    nedToThree(sunNed, toSunThree).normalize();

    sky.sunPosition.value.copy(toSunThree);
    sunDirection.copy(toSunThree).multiplyScalar(-1);
    sun.position.copy(toSunThree).multiplyScalar(SUN_RIG_DISTANCE);

    setSunElevation(elevationRad);
    setSunLight(sun, elevationDeg);
    computeHorizonColor(elevationRad, horizonColor);
    fog.color.copy(horizonColor);

    sunMoved = true;
  }

  /**
   * Render the sky alone into a cube map and pre-filter it. The map feeds every
   * PBR surface, so it matters more than the sky pixels themselves.
   *
   * The sky mesh moves into a private scene for the render, then goes back to
   * the parent it came from. The cube camera sits at the origin, so the box
   * moves there as well and the camera stays inside it.
   */
  function regenerateEnvironment(activeRenderer: WebGPURenderer, target: Scene): void {
    if (pmrem === null) pmrem = new PMREMGenerator(activeRenderer);

    const parent: Object3D | null = sky.parent;
    const previousX = sky.position.x;
    const previousY = sky.position.y;
    const previousZ = sky.position.z;

    sky.position.set(0, 0, 0);
    // The solar disc is a very small and very bright spot. It leaves ringing in
    // the pre-filtered map, and the directional light already carries it.
    sky.showSunDisc.value = 0;
    environmentScene.add(sky);

    const generated = pmrem.fromScene(environmentScene, 0, 1, 2 * SKY_SIZE, {
      size: ENVIRONMENT_SIZE,
    });

    if (environmentTarget !== null && environmentTarget !== generated) {
      environmentTarget.dispose();
    }
    environmentTarget = generated;
    target.environment = generated.texture;

    sky.showSunDisc.value = 1;
    sky.position.set(previousX, previousY, previousZ);
    if (parent !== null) parent.add(sky);
    else environmentScene.remove(sky);
  }

  setSunAngles(DEFAULT_SUN_ELEVATION_DEG, DEFAULT_SUN_AZIMUTH_DEG);
  regenerateEnvironment(renderer, scene);
  sunMoved = false;

  return {
    sky,
    sun,
    sunDirection,

    setSunAngles,

    update(activeRenderer: WebGPURenderer, activeScene: Scene): void {
      if (!sunMoved) return;
      sunMoved = false;
      regenerateEnvironment(activeRenderer, activeScene);
    },

    dispose(): void {
      if (scene.fog === fog) scene.fog = null;
      if (environmentTarget !== null && scene.environment === environmentTarget.texture) {
        scene.environment = null;
      }
      if (environmentTarget !== null) {
        environmentTarget.dispose();
        environmentTarget = null;
      }
      if (pmrem !== null) {
        pmrem.dispose();
        pmrem = null;
      }
      sky.geometry.dispose();
      sky.material.dispose();
      sky.removeFromParent();
      sun.shadow.dispose();
      sun.target.removeFromParent();
      sun.removeFromParent();
    },
  };
}
