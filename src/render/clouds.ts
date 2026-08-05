/**
 * The cloud layer.
 *
 * One box holds a deck of stratocumulus. A TSL fragment shader marches a ray
 * through the box and builds the color from the same sun and the same haze
 * that src/render/sky.ts already computes, so the cloud never disagrees with
 * the sky behind it.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * 1. WHY A RAY MARCH AND NOT SPRITES
 *
 * The pilot must read the deck from the ground, from below it, from inside it,
 * and from above it. Three methods were on the table.
 *
 * Billboard sprites are the cheapest. They also fail at the one moment that
 * matters most: a sprite is a flat card, so it turns as the aircraft passes
 * through the deck and the whole layer swims. A deck of sprites also needs a
 * sort from far to near, and section 3 below shows that the sort of this
 * renderer runs the wrong way round.
 *
 * A hybrid, volumetric near and billboards far, needs a join between two
 * models that light differently. The join shows as a ring around the aircraft.
 *
 * The ray march holds one model for every distance and for every view. It is
 * the most expensive of the three, so section 4 gives the cost and the limits
 * that hold it down.
 *
 *
 * 2. WHY THE BOX IS THE DECK AND WHY IT DRAWS ITS BACK FACE
 *
 * The box holds exactly the air between `CLOUD_BASE` and `CLOUD_TOP`, and it
 * follows the camera on the two horizontal axes.
 *
 * The material draws the BACK face alone. Two things follow.
 *
 * First, the shader runs one time for each pixel. A box with both faces would
 * run the same march two times for a camera outside the box, and the second
 * pass would blend the deck over itself.
 *
 * Second, the depth test then works. The fragment sits where the ray LEAVES
 * the deck, so the fragment is nearer than the ground under the deck whenever
 * the pilot looks down through the deck. The ground therefore does not hide the
 * cloud, and a building or a tree in front of the deck still does hide it.
 *
 * The one view that this loses is a level view from INSIDE the deck, where the
 * exit face stands 22 km away and the far ground can stand nearer. The haze at
 * 22 km already covers 89 percent of that pixel, so the loss does not show.
 *
 *
 * 3. THE RENDER ORDER AND THE REVERSED DEPTH BUFFER
 *
 * `RenderList.sort` sorts the transparent list from far to near and then calls
 * `reverse()` on it when the camera holds a reversed depth buffer. The order
 * of the list turns around and the render order turns around with it. Read
 * section 6a of docs/CONVENTIONS.md and the note in src/render/sky.ts.
 *
 * The deck is the FAR layer, so it must draw before the particles.
 * `CLOUD_RENDER_ORDER` is therefore smaller than the order of the particles,
 * and src/render/particles.ts reads it from here. Both change sign with
 * `renderer.reversedDepthBuffer`.
 *
 *
 * 4. THE COST AND WHAT IT BUYS
 *
 * The march takes `MARCH_STEPS` samples along the ray. The step length grows
 * with the distance, so the near air, which fills most of the screen, gets the
 * short steps.
 *
 * The shape comes from one 3D texture and not from noise in the shader. A
 * shader that builds four octaves of value noise needs 32 hash rounds for one
 * sample. The texture needs one fetch, and the card filters it for free.
 *
 * The light does NOT march toward the sun. A second march would multiply the
 * cost by four. The shader takes the depth from the sample to the top of the
 * deck along the sun ray, and it takes one more density sample toward the sun
 * to break the flat look. A deck is a layer, so the depth to the top is a good
 * measure of how much cloud stands between the sample and the sun. The result
 * is a bright top, a grey base, and a bright rim on the sunward side.
 *
 * What this leaves out, and it should be said plainly: the deck casts no
 * shadow on the ground, and it does not enter the environment map. A pilot
 * under a hole in the deck therefore gets the same light as a pilot under the
 * thickest part.
 *
 *
 * 5. WHY THE SKY DOME LOSES ITS OWN CLOUDS
 *
 * `SkyMesh` draws its own clouds. They sit at an infinite distance, they hold
 * no parallax, and they stay above the pilot even when the aircraft climbs
 * over them. Two cloud models in one picture disagree at every altitude, so
 * this module sets the cloud coverage of the sky dome to zero. It writes one
 * uniform of the sky and it changes no line of src/render/sky.ts.
 */

import type {
  DirectionalLight,
  Node,
  Object3D,
  Scene,
  WebGPURenderer,
} from 'three/webgpu';
import {
  BackSide,
  BoxGeometry,
  Data3DTexture,
  FogExp2,
  LinearFilter,
  Mesh,
  MeshBasicNodeMaterial,
  NormalBlending,
  RGBAFormat,
  RepeatWrapping,
  UnsignedByteType,
  Vector3,
} from 'three/webgpu';
import {
  Break,
  Fn,
  If,
  Loop,
  cameraPosition,
  exp,
  float,
  hash,
  max,
  min,
  mix,
  positionWorld,
  screenCoordinate,
  smoothstep,
  texture3D,
  uniform,
  vec2,
  vec3,
  vec4,
} from 'three/tsl';

import { horizonColor } from '@/render/sky';

/**
 * Base and top of the deck, in meters above the ground.
 *
 * A stratocumulus deck over northern Europe sits between 600 m and 2000 m. The
 * deck here stands higher, because the whole point of the layer is that the
 * pilot flies under it, through it, and over it in one sortie, and the Me-262
 * needs room under the deck to fly the circuit.
 */
const CLOUD_BASE = 2150;
const CLOUD_TOP = 3350;

/** Half side of the box, in meters. The far plane of the camera is 60 km. */
const CLOUD_HALF_SIDE = 26000;

/** Longest distance the march covers, in meters. */
const MAX_MARCH = 22000;

/** Samples the primary march takes. */
const MARCH_STEPS = 44;

/**
 * Growth of the step length along the ray.
 *
 * A constant step wastes its samples in the far air, where one sample already
 * covers many pixels. With this growth the first step is 0.6 percent of the
 * span and the last one is 6.5 percent.
 */
const STEP_GROWTH = 1.055;

/** Side of the noise texture, in texels. */
const NOISE_SIZE = 48;

/** World size of one repeat of the shape channel and the detail channel, m. */
const SHAPE_SCALE = 8200;
const MID_SCALE = 2900;
const DETAIL_SCALE = 420;

/**
 * Extinction of the cloud, per meter, at a density of one.
 *
 * A dense cumulus holds a visibility near 50 m, which is an extinction near
 * 0.06 per meter. A stratocumulus deck is thinner than that. At 0.018 per
 * meter the whole 1200 m of the deck reaches an optical depth of 21 where the
 * density is one, so the thick part is opaque and the edge stays soft.
 */
const EXTINCTION = 0.018;

/**
 * How far the shape signal spreads before the coverage threshold reads it.
 *
 * A sum of octaves of value noise crowds around 0.5 with a spread near 0.06.
 * A factor of 5 carries that spread over most of the range 0 to 1, so the
 * threshold can cut real holes. Read the note on `cloudField`.
 */
const SHAPE_CONTRAST = 5;

/** Width of the soft band at the coverage threshold. It softens every edge. */
const COVERAGE_BAND = 0.34;

/** Height a thin column reaches inside the deck, from 0 to 1. */
const THIN_COLUMN_TOP = 0.34;

/** Distance where the deck starts to fade out, in meters. Read the march. */
const FADE_START = 13000;

/** Single scattering albedo of a water cloud. Water absorbs almost nothing. */
const CLOUD_ALBEDO = 0.96;

/**
 * Gain that stands for the light that scattered more than one time.
 *
 * The march follows ONE scattering event. A real cloud sends most of its light
 * to the eye only after many events, so a single scattering march gives a
 * cloud that is far too dark.
 *
 * The value is set from the grey card of src/core/config.ts and not by eye. At
 * an exposure of 0.17 the ACES curve puts a radiance of 5.1 at 227 of 255,
 * which is where a sunlit cloud top must read next to fresh runway paint at
 * 212 and sunlit grass at 87. Do not change the exposure to make the cloud
 * look right. If the cloud is wrong, the cloud is wrong.
 */
const SUN_SCATTER_GAIN = 5;

/**
 * Part of the sun irradiance that reaches a sample after many bounces inside
 * the cloud around it. It is what stops a shadowed base from going black.
 */
const IN_CLOUD_AMBIENT = 0.05;

/** Radiance to irradiance for light that arrives from every direction. */
const ISOTROPIC = 1 / (4 * Math.PI);

/**
 * Irradiance of the sky hemisphere over the radiance at the horizon.
 *
 * src/render/sky.ts gives the horizon radiance, which is the BRIGHTEST part of
 * the sky. src/core/config.ts states that the whole hemisphere sends 3.46 onto
 * flat ground in the green channel at a sun elevation of 45 degrees, and the
 * horizon radiance measures 6.32 in the same channel. The ratio is 0.55.
 */
const SKY_IRRADIANCE_FACTOR = 0.55;

/**
 * The two lobes of the phase function.
 *
 * A single forward lobe makes the side of a cloud away from the sun far too
 * dark, because a real cloud spreads the light over every direction before it
 * leaves. The second lobe faces backward and it holds that side up. The
 * forward value matches the Mie term that src/render/sky.ts gives the sky, so
 * the haze and the cloud bend the light the same way.
 */
const PHASE_G = 0.62;
const PHASE_G_BACK = -0.35;
const PHASE_BACK_MIX = 0.35;

/** Distance toward the sun of the one extra density sample, m. */
const SUN_SAMPLE_DISTANCE = 260;

/** Longest path toward the sun the shadow term counts, m. */
const SUN_PATH_LIMIT = 950;

/** How much of the sky a sample sees, at the base and at the top of the deck. */
const SKY_VIEW_BASE = 0.3;
const SKY_VIEW_TOP = 1;

/** Speed of the deck over the ground, m/s. A gentle drift, not a gale. */
const WIND_SPEED = 5.5;

/** Wind direction, as a unit vector in the render frame, from the west. */
const WIND_X = 0.94;
const WIND_Z = 0.34;

/**
 * Part of the sky the deck covers, from 0 to 1.
 *
 * 0.52 gives a broken deck. The pilot sees blue between the cloud and the
 * ground under the holes, which is what makes a deck read as a deck and not as
 * a ceiling.
 */
const DEFAULT_COVERAGE = 0.52;

/** Render order of the deck with a NORMAL depth buffer. Read section 3. */
export const CLOUD_RENDER_ORDER = 1;

export interface CloudLayer {
  /** The deck, in the render frame of the WORLD. */
  mesh: Object3D;

  /** Base and top of the deck, in meters. The camera rig may want them. */
  readonly base: number;
  readonly top: number;

  /** Part of the sky the deck covers, from 0 to 1. */
  setCoverage(coverage: number): void;

  /**
   * Keep the box over the camera, move the wind, and take the sun.
   *
   * `sun` and `travel` both come from the bundle of src/render/sky.ts. `travel`
   * is the direction the sunlight moves, so it points away from the sun.
   */
  update(
    dt: number,
    cameraPositionRender: Vector3,
    sun: DirectionalLight,
    travel: Vector3,
  ): void;

  dispose(): void;
}

// ---------------------------------------------------------------------------
// The noise texture.
// ---------------------------------------------------------------------------

/**
 * Hash of one lattice point of the noise, from 0 to 1.
 *
 * The lattice index wraps, so the texture repeats with no seam. The mix is the
 * lowbias32 finalizer, which carries every input bit into every output bit.
 * Source: Wellons, "Prospecting for Hash Functions", 2018, confidence: firm.
 */
function latticeHash(x: number, y: number, z: number, period: number, seed: number): number {
  const ix = ((x % period) + period) % period;
  const iy = ((y % period) + period) % period;
  const iz = ((z % period) + period) % period;
  let h = (ix * 0x27d4eb2d) ^ (iy * 0x85ebca6b) ^ (iz * 0x165667b1) ^ (seed * 0x9e3779b1);
  h = Math.imul(h ^ (h >>> 16), 0x21f0aaad);
  h = Math.imul(h ^ (h >>> 15), 0xd35a2d97);
  h = h ^ (h >>> 15);
  return (h >>> 0) / 4294967296;
}

/** One octave of tiling value noise in three dimensions, from 0 to 1. */
function valueNoise3(
  x: number,
  y: number,
  z: number,
  period: number,
  seed: number,
): number {
  const fx = x * period;
  const fy = y * period;
  const fz = z * period;
  const ix = Math.floor(fx);
  const iy = Math.floor(fy);
  const iz = Math.floor(fz);
  const tx = fx - ix;
  const ty = fy - iy;
  const tz = fz - iz;
  // The smoothstep weights remove the lattice creases that a linear blend
  // shows. The same weights appear in src/render/ground.ts.
  const wx = tx * tx * (3 - 2 * tx);
  const wy = ty * ty * (3 - 2 * ty);
  const wz = tz * tz * (3 - 2 * tz);

  let result = 0;
  for (let dz = 0; dz < 2; dz++) {
    const bz = dz === 0 ? 1 - wz : wz;
    for (let dy = 0; dy < 2; dy++) {
      const by = dy === 0 ? 1 - wy : wy;
      for (let dx = 0; dx < 2; dx++) {
        const bx = dx === 0 ? 1 - wx : wx;
        result += bx * by * bz * latticeHash(ix + dx, iy + dy, iz + dz, period, seed);
      }
    }
  }
  return result;
}

/** Several octaves of tiling value noise, normalized to the range 0 to 1. */
function fbm3(
  x: number,
  y: number,
  z: number,
  basePeriod: number,
  octaves: number,
  seed: number,
): number {
  let value = 0;
  let amplitude = 0.5;
  let total = 0;
  let period = basePeriod;
  for (let o = 0; o < octaves; o++) {
    value += amplitude * valueNoise3(x, y, z, period, seed + o * 131);
    total += amplitude;
    amplitude *= 0.5;
    period *= 2;
  }
  return value / total;
}

/**
 * Build the 3D noise texture.
 *
 * Four channels hold four scales of the same kind of noise. The shader mixes
 * two of them for the shape and it takes a third away from the edge, so the
 * cloud never repeats on the scale that any one channel repeats on.
 */
function buildNoiseTexture(): Data3DTexture {
  const size = NOISE_SIZE;
  const data = new Uint8Array(size * size * size * 4);
  const inverse = 1 / size;
  let index = 0;
  for (let z = 0; z < size; z++) {
    const fz = z * inverse;
    for (let y = 0; y < size; y++) {
      const fy = y * inverse;
      for (let x = 0; x < size; x++) {
        const fx = x * inverse;
        data[index] = Math.round(255 * fbm3(fx, fy, fz, 3, 3, 11));
        data[index + 1] = Math.round(255 * fbm3(fx, fy, fz, 5, 2, 271));
        data[index + 2] = Math.round(255 * fbm3(fx, fy, fz, 8, 2, 733));
        data[index + 3] = Math.round(255 * fbm3(fx, fy, fz, 4, 2, 1583));
        index += 4;
      }
    }
  }

  const texture = new Data3DTexture(data, size, size, size);
  texture.format = RGBAFormat;
  texture.type = UnsignedByteType;
  texture.minFilter = LinearFilter;
  texture.magFilter = LinearFilter;
  texture.wrapS = RepeatWrapping;
  texture.wrapT = RepeatWrapping;
  texture.wrapR = RepeatWrapping;
  texture.needsUpdate = true;
  return texture;
}

// ---------------------------------------------------------------------------
// The layer.
// ---------------------------------------------------------------------------

/** The two uniforms of `SkyMesh` that this module writes. Read section 5. */
interface SkyCloudUniforms {
  cloudCoverage: { value: number };
}

/**
 * Read the cloud uniform of the sky dome, if the object is a `SkyMesh`.
 *
 * The bundle of src/render/sky.ts reports the dome as a plain `Object3D`, so
 * the test looks at the flag that the class sets on itself.
 */
function skyCloudUniforms(object: Object3D | null): SkyCloudUniforms | null {
  if (object === null) return null;
  const probe = object as unknown as {
    isSkyMesh?: boolean;
    cloudCoverage?: { value: number };
  };
  if (probe.isSkyMesh !== true) return null;
  if (probe.cloudCoverage === undefined) return null;
  return { cloudCoverage: probe.cloudCoverage };
}

export function createClouds(
  renderer: WebGPURenderer,
  scene: Scene,
  skyDome: Object3D | null,
): CloudLayer {
  const noise = buildNoiseTexture();

  const thickness = CLOUD_TOP - CLOUD_BASE;

  // --- The uniforms the frame writes ------------------------------------
  const coverage = uniform(DEFAULT_COVERAGE);
  const windOffset = uniform(new Vector3());
  const sunDirection = uniform(new Vector3(0, 1, 0));
  const sunIrradiance = uniform(new Vector3(1, 1, 1));
  const skyIrradiance = uniform(new Vector3(1, 1, 1));
  const fogDensity = uniform(0);
  const fogColor = uniform(new Vector3(1, 1, 1));

  /**
   * Density of the cloud at one point of the world, and the top of its column.
   *
   * `x` is the density, from 0 to 1. `y` is the height of the top of that
   * column inside the deck, from 0 to 1. The march needs the second value,
   * because the light term measures how deep the sample sits under the top of
   * its OWN column and not under the top of the whole box.
   *
   *
   * WHY THE SHAPE NEEDS THE CONTRAST TERM
   *
   * A sum of octaves of value noise is a sum of many small numbers, so its
   * values crowd around 0.5. Measured over the texture, the spread is 0.06.
   * A plain threshold on such a signal leaves a thin haze of density over the
   * whole sky, and the deck is 1200 m thick, so even a density of 0.05 takes
   * two thirds of the light out of a vertical ray. The first picture from
   * above showed exactly that: a milky sheet from edge to edge with no holes.
   *
   * `SHAPE_CONTRAST` spreads the signal over the whole range before the
   * threshold reads it. The deck then has real holes and real solid parts.
   */
  const cloudField = Fn(([point]: [Node<'vec3'>]) => {
    const height = point.y.sub(CLOUD_BASE).div(thickness);

    const drift = point.add(windOffset);
    const big = texture3D(noise, drift.mul(1 / SHAPE_SCALE), 0).r;
    const mid = texture3D(noise, drift.mul(1 / MID_SCALE).add(0.37), 0).g;
    const shape = big.mul(0.64).add(mid.mul(0.36));
    const spread = shape.sub(0.5).mul(SHAPE_CONTRAST).add(0.5);

    // The coverage is a threshold with a soft band, so an edge is soft and the
    // clear air is really clear. The threshold runs past both ends of the
    // spread signal, so a coverage of 0 leaves a clear sky and a coverage of 1
    // leaves a solid lid.
    const level = mix(float(1.45), float(-COVERAGE_BAND), coverage);
    const cover = smoothstep(level, level.add(COVERAGE_BAND), spread);

    // A thin column tops out low and a thick column fills the whole deck. This
    // is what gives the deck a lumpy top instead of a flat lid.
    const columnTop = mix(float(THIN_COLUMN_TOP), float(1), cover);

    // Flat under, round over.
    const rise = smoothstep(float(0), float(0.09), height);
    const fall = smoothstep(float(0), float(0.3), columnTop.sub(height));
    const density = cover.mul(rise).mul(fall).toVar();

    // The fine channel only matters where there is cloud to eat into, and the
    // base of a cloud is more ragged than its top.
    If(density.greaterThan(0.002), () => {
      const detail = texture3D(noise, drift.mul(1 / DETAIL_SCALE).add(0.13), 0).b;
      const bite = detail.mul(mix(float(0.38), float(0.1), height));
      density.assign(density.sub(bite).max(0));
    });

    return vec2(density, columnTop);
  });

  /**
   * The Henyey-Greenstein phase function.
   *
   * A water drop sends most of the light forward, so a cloud between the pilot
   * and the sun holds a bright rim. `PHASE_G` matches the value that
   * src/render/sky.ts gives the Mie term of the sky, so the two agree.
   */
  const henyeyGreenstein = Fn(([cosAngle, g]: [Node<'float'>, Node<'float'>]) => {
    const g2 = g.mul(g);
    const denominator = float(1).add(g2).sub(cosAngle.mul(g).mul(2)).max(1e-4);
    return float(1).sub(g2).mul(ISOTROPIC).div(denominator.pow(1.5));
  });

  const phase = Fn(([cosAngle]: [Node<'float'>]) => {
    const forward = henyeyGreenstein(cosAngle, float(PHASE_G));
    const backward = henyeyGreenstein(cosAngle, float(PHASE_G_BACK));
    return mix(forward, backward, float(PHASE_BACK_MIX));
  });

  const march = Fn(() => {
    const origin = cameraPosition;
    const direction = positionWorld.sub(cameraPosition).normalize().toVar();

    // The entry and the exit of the deck. A ray that runs almost level would
    // divide by zero, so the vertical part of the ray keeps a floor.
    const sign = direction.y.lessThan(0).select(float(-1), float(1));
    const stepY = direction.y.abs().max(1e-5).mul(sign);
    const tBase = float(CLOUD_BASE).sub(origin.y).div(stepY);
    const tTop = float(CLOUD_TOP).sub(origin.y).div(stepY);
    const tNear = min(tBase, tTop).max(0).toVar();
    const tFar = max(tBase, tTop).min(MAX_MARCH).toVar();

    const scattered = vec3(0, 0, 0).toVar();
    const transmittance = float(1).toVar();
    const depthSum = float(0).toVar();
    const depthWeight = float(0).toVar();

    If(tFar.greaterThan(tNear), () => {
      const span = tFar.sub(tNear);

      // The step grows along the ray. The sum of a geometric series fixes the
      // first step, so the whole set of steps covers the span exactly.
      const growthSum = (Math.pow(STEP_GROWTH, MARCH_STEPS) - 1) / (STEP_GROWTH - 1);
      const firstStep = span.div(growthSum);

      // A random offset per pixel moves the first sample. Without it the
      // samples line up across the picture and the deck shows contour rings.
      //
      // The offset is white noise. The interleaved gradient noise of Jimenez
      // was tried first, because it holds a lower error for the same range.
      // It reads far worse here: its pattern repeats every few pixels, and on
      // the soft edge of a cloud that repeat came out as concentric rings and
      // fingerprints. White noise leaves a fine grain instead, and with
      // `MARCH_STEPS` at its present value the grain is already faint.
      const dither = hash(screenCoordinate.x.mul(3).add(screenCoordinate.y.mul(1021)));

      const cosSun = direction.dot(sunDirection);
      const phaseTerm = phase(cosSun);

      const t = tNear.toVar();
      const step = firstStep.toVar();

      Loop(MARCH_STEPS, () => {
        const distance = t.add(step.mul(dither));
        const point = origin.add(direction.mul(distance));
        const field = cloudField(point).toVar();
        const density = field.x.toVar();

        If(density.greaterThan(0.002), () => {
          // How much cloud stands between this sample and the sun. The march
          // does not follow the sun ray. Read section 4. The top of the OWN
          // column of the sample fixes the path, so a sample near the top of a
          // low column stays bright and does not take the shadow of a tall one.
          const columnTopY = float(CLOUD_BASE).add(field.y.mul(thickness));
          const toTop = columnTopY
            .sub(point.y)
            .max(0)
            .div(max(sunDirection.y, float(0.12)))
            .min(SUN_PATH_LIMIT);
          const ahead = point.add(sunDirection.mul(SUN_SAMPLE_DISTANCE));
          const meanDensity = density.add(cloudField(ahead).x).mul(0.5);
          const sunTransmittance = exp(meanDensity.mul(toTop).mul(-EXTINCTION));

          const height = point.y.sub(CLOUD_BASE).div(thickness).clamp(0, 1);
          const skyView = mix(float(SKY_VIEW_BASE), float(SKY_VIEW_TOP), height);

          // The phase function already carries the 1 over 4 pi that turns an
          // irradiance into a radiance, so the direct term must NOT be divided
          // by pi again. The first build did divide it twice, and the cloud
          // then held no shape at all: every part of it read the same white.
          const direct = phaseTerm.mul(sunTransmittance).mul(SUN_SCATTER_GAIN);
          const bounced = float(IN_CLOUD_AMBIENT).mul(mix(float(0.5), float(1), height));
          const fromSun = sunIrradiance.mul(direct.add(bounced));
          const fromSky = skyIrradiance.mul(skyView).mul(ISOTROPIC);
          const source = fromSun.add(fromSky).mul(CLOUD_ALBEDO);

          // The analytic integral of a constant source over the step. It stays
          // correct when the step is long, which a Riemann sum does not.
          const stepTransmittance = exp(density.mul(step).mul(-EXTINCTION));
          const weight = float(1).sub(stepTransmittance).mul(transmittance);

          scattered.addAssign(source.mul(weight));
          depthSum.addAssign(distance.mul(weight));
          depthWeight.addAssign(weight);
          transmittance.mulAssign(stepTransmittance);
        });

        t.addAssign(step);
        step.mulAssign(STEP_GROWTH);

        // Nothing behind an opaque cloud can reach the eye.
        If(transmittance.lessThan(0.02), () => {
          Break();
        });
      });
    });

    const meanDistance = depthSum.div(depthWeight.max(1e-4));

    // The box and the march both stop at a fixed distance, and a stop leaves a
    // straight line across the picture where the deck ends. The first frame
    // from above showed that line. The alpha therefore falls to zero before
    // either limit, where the haze has already taken most of the light.
    const reach = smoothstep(float(FADE_START), float(MAX_MARCH), meanDistance).oneMinus();
    const alpha = float(1).sub(transmittance).mul(reach);

    // The haze of src/render/sky.ts covers the deck as well. The fog factor
    // uses the mean distance of the light that this pixel collected, which is
    // where the cloud really stands, and not the face of the box.
    const optical = fogDensity.mul(meanDistance);
    const haze = float(1).sub(exp(optical.mul(optical).negate())).clamp(0, 1);

    // The color is already multiplied by the alpha, so the haze color needs
    // the same treatment. Read the note on the blending below.
    const hazed = mix(scattered.mul(reach), fogColor.mul(alpha), haze);

    return vec4(hazed, alpha);
  });

  const material = new MeshBasicNodeMaterial();
  material.colorNode = march();
  material.side = BackSide;
  material.transparent = true;
  material.blending = NormalBlending;
  // The march builds a radiance that already carries its own alpha, which is
  // what a volume integral gives. Premultiplied blending takes that form
  // directly. Without it the shader would have to divide the color by an alpha
  // that can be very small, and the result would break up at the thin edge.
  material.premultipliedAlpha = true;
  material.depthWrite = false;
  material.depthTest = true;
  // The shader applies the haze itself, over the real distance of the cloud.
  material.fog = false;

  const geometry = new BoxGeometry(2 * CLOUD_HALF_SIDE, thickness, 2 * CLOUD_HALF_SIDE);
  const mesh = new Mesh(geometry, material);
  mesh.name = 'clouds';
  mesh.position.y = 0.5 * (CLOUD_BASE + CLOUD_TOP);
  mesh.frustumCulled = false;
  mesh.renderOrder = renderer.reversedDepthBuffer ? -CLOUD_RENDER_ORDER : CLOUD_RENDER_ORDER;
  mesh.castShadow = false;
  mesh.receiveShadow = false;

  // Read section 5. The dome keeps its sky and it loses its clouds.
  const domeClouds = skyCloudUniforms(skyDome);
  if (domeClouds !== null) domeClouds.cloudCoverage.value = 0;

  let elapsed = 0;

  return {
    mesh,
    base: CLOUD_BASE,
    top: CLOUD_TOP,

    setCoverage(next: number): void {
      coverage.value = Math.min(1, Math.max(0, next));
    },

    update(
      dt: number,
      cameraPositionRender: Vector3,
      sun: DirectionalLight,
      travel: Vector3,
    ): void {
      elapsed += dt;

      // The box rides with the camera, so the deck reaches the horizon from
      // every place. The noise reads the world position, so the cloud stays
      // where it was and only the wind moves it.
      mesh.position.x = cameraPositionRender.x;
      mesh.position.z = cameraPositionRender.z;

      // The shader adds this offset to the world position, so the deck moves
      // against the wind direction.
      windOffset.value.set(-WIND_X * WIND_SPEED * elapsed, 0, -WIND_Z * WIND_SPEED * elapsed);

      // The light travels away from the sun, so the direction toward the sun
      // is the opposite one.
      sunDirection.value.copy(travel).multiplyScalar(-1);

      // A surface that faces the sun receives the color times the strength.
      // src/render/sky.ts builds both from the radiance of the solar disc that
      // the sky shader draws, so the deck reads the same sun the pilot sees.
      sunIrradiance.value.set(
        sun.color.r * sun.intensity,
        sun.color.g * sun.intensity,
        sun.color.b * sun.intensity,
      );

      // The sky hemisphere sends pi times its mean radiance onto a flat
      // surface. The horizon color of src/render/sky.ts is that radiance.
      skyIrradiance.value.set(
        horizonColor.r * SKY_IRRADIANCE_FACTOR,
        horizonColor.g * SKY_IRRADIANCE_FACTOR,
        horizonColor.b * SKY_IRRADIANCE_FACTOR,
      );

      const fog = scene.fog;
      if (fog instanceof FogExp2) {
        fogDensity.value = fog.density;
        fogColor.value.set(fog.color.r, fog.color.g, fog.color.b);
      } else {
        fogDensity.value = 0;
        fogColor.value.set(horizonColor.r, horizonColor.g, horizonColor.b);
      }
    },

    dispose(): void {
      geometry.dispose();
      material.dispose();
      noise.dispose();
      mesh.removeFromParent();
    },
  };
}
