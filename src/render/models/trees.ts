/**
 * Trees.
 *
 * The module builds three tree shapes, in three sizes each, out of cones,
 * cylinders, and low polygon solids. A conifer is a trunk under stacked cones.
 * A broadleaf is a trunk under two clumps of foliage. A bush is foliage alone.
 * Each type carries three levels of detail, and the coarsest level is one solid
 * with fewer than ten triangles, because a tree at 3 km covers a few pixels.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * MODEL FRAME
 *
 * A tree stands on the ground plane. Its origin sits at `y = 0` at the center
 * of its trunk and it reaches up along `+y`. A tree has no front, so
 * src/world/scatter.ts turns each one by a random angle only to break the
 * repeat.
 *
 *
 * WHY THE WIND RUNS IN THE VERTEX STAGE
 *
 * A forest of 4000 trees cannot sway from the CPU. A CPU sway would rebuild
 * every instance matrix on every frame and would send the whole buffer to the
 * GPU 60 times a second, and it could still only turn a whole tree, not bend
 * one. The sway is a vertex program instead. It costs one sine per vertex and
 * nothing at all on the CPU, and `updateTreeWind` only writes one uniform.
 *
 * Two attributes and one accessor carry it:
 *
 * - `sway` is a weight from 0 at the foot of the trunk to 1 at the top of the
 *   crown. It rises as the square of the height, which is the shape of a
 *   cantilever under an even load. So the trunk stays put and the crown moves.
 * - The vertex position, read after the instance transform, gives the world
 *   position of the vertex. Its x and z set the phase of the wave, so two trees
 *   10 m apart sway out of step and the wood does not move as one body.
 * - The same position gives the height of the vertex above the ground. The
 *   sway is proportional to that height, so a 19 m conifer leans further than a
 *   2 m bush with no extra attribute and no extra material.
 *
 * The order matters and it is not free choice. `NodeMaterial.setupPosition`
 * applies the instance matrix to `positionLocal` first and reads
 * `material.positionNode` after that. So inside the position node
 * `positionLocal` already holds the instance transformed position. The group
 * root of src/render/instanced.ts sits at the render origin, so that position
 * is also the world position.
 */

import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import type { Material } from 'three/webgpu';
import {
  BufferAttribute,
  BufferGeometry,
  Color,
  ConeGeometry,
  CylinderGeometry,
  IcosahedronGeometry,
  MeshStandardNodeMaterial,
  OctahedronGeometry,
} from 'three/webgpu';
import {
  attribute,
  float,
  mx_noise_float,
  positionGeometry,
  positionLocal,
  sin,
  uniform,
  vec3,
} from 'three/tsl';

import type { LodLevel } from '@/render/instanced';

/** Name of the vertex attribute that holds the base color of a part. */
const PART_TINT = 'partTint';

/** Name of the vertex attribute that holds the sway weight of a vertex. */
const SWAY = 'sway';

/** Bark and foliage colors, as sRGB hex values. */
const BARK_CONIFER = 0x4a3b2e;
const BARK_BROADLEAF = 0x6b6055;
const NEEDLE_GREEN = 0x2f4626;
const LEAF_GREEN = 0x4a6330;
const SCRUB_GREEN = 0x5d6b34;

/** Roughness of bark and of foliage. Neither one shines. */
const TREE_ROUGHNESS = 0.94;

/** Feature size of the foliage noise, in meters, and its brightness swing. */
const FOLIAGE_NOISE_SCALE = 1.7;
const FOLIAGE_NOISE_STRENGTH = 0.16;

/**
 * Lean of the crown at full wind, as a fraction of the height of the vertex.
 * A value of 0.02 moves the top of a 19 m conifer by about 0.4 m, which is a
 * gentle breeze and not a gale.
 */
const WIND_STRENGTH = 0.022;

/** Radian per second of the main gust and of the faster ripple over it. */
const WIND_RATE = 0.9;
const WIND_RIPPLE_RATE = 2.1;

/**
 * Radian per meter of the phase across the ground. The value is 2 pi over a
 * wavelength of 34 m, so two trees at opposite ends of one grove lean in
 * opposite directions.
 */
const WIND_WAVE_NUMBER = (2 * Math.PI) / 34;

/** Direction the wind pushes, in the render frame. It is a unit vector. */
const WIND_DIRECTION_X = 0.82;
const WIND_DIRECTION_Z = 0.57;

/**
 * Time that drives the wind, in seconds. One uniform serves every tree
 * material, so `updateTreeWind` is one write however many types exist.
 */
const windTime = uniform(0);

export interface TreeType {
  name: string;
  levels: LodLevel[];
  material: Material;
  /** Radius of the crown, in meters. The scatter uses it to space the trees. */
  radius: number;
  /** Height of the top, in meters. */
  height: number;
}

/** Scratch color. The module allocates it one time. */
const scratchColor = new Color();

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/**
 * Give one part its color and its sway weight, and drop the index.
 *
 * The sway weight is the square of the height of the vertex over the height of
 * the tree, clamped to 1. `mergeGeometries` needs every input to hold the same
 * attributes and to agree on the index, so every part goes through here.
 */
function paint(geometry: BufferGeometry, colorHex: number, treeHeight: number): BufferGeometry {
  const flat = geometry.index !== null ? geometry.toNonIndexed() : geometry;
  if (flat !== geometry) geometry.dispose();

  const position = flat.getAttribute('position');
  const count = position.count;
  const tint = new Float32Array(count * 3);
  const sway = new Float32Array(count);

  scratchColor.setHex(colorHex);
  for (let i = 0; i < count; i += 1) {
    tint[i * 3 + 0] = scratchColor.r;
    tint[i * 3 + 1] = scratchColor.g;
    tint[i * 3 + 2] = scratchColor.b;
    const fraction = Math.min(1, Math.max(0, position.getY(i) / treeHeight));
    sway[i] = fraction * fraction;
  }

  flat.setAttribute(PART_TINT, new BufferAttribute(tint, 3));
  flat.setAttribute(SWAY, new BufferAttribute(sway, 1));
  return flat;
}

/** Join every part into one geometry and free the parts. */
function merge(parts: BufferGeometry[]): BufferGeometry {
  const merged: BufferGeometry | null = mergeGeometries(parts, false);
  if (merged === null) throw new Error('The tree parts do not share one attribute set.');
  for (const part of parts) part.dispose();
  return merged;
}

/** A trunk that stands on the ground. */
function trunk(
  radius: number,
  height: number,
  segments: number,
  colorHex: number,
  treeHeight: number,
): BufferGeometry {
  const geometry = new CylinderGeometry(radius * 0.62, radius, height, segments);
  return paint(geometry, colorHex, treeHeight).translate(0, height / 2, 0);
}

/** One cone of needles, with its base at `y`. */
function needleCone(
  radius: number,
  height: number,
  segments: number,
  y: number,
  colorHex: number,
  treeHeight: number,
): BufferGeometry {
  const geometry = new ConeGeometry(radius, height, segments);
  return paint(geometry, colorHex, treeHeight).translate(0, y + height / 2, 0);
}

/**
 * One clump of leaves. An icosahedron with no subdivision holds 20 triangles
 * and reads as a round mass of leaves once it is squashed a little.
 */
function clump(
  radius: number,
  flatten: number,
  x: number,
  y: number,
  z: number,
  colorHex: number,
  treeHeight: number,
): BufferGeometry {
  const geometry = new IcosahedronGeometry(radius, 0);
  geometry.scale(1, flatten, 1);
  geometry.computeVertexNormals();
  return paint(geometry, colorHex, treeHeight).translate(x, y, z);
}

// ---------------------------------------------------------------------------
// The three shapes
// ---------------------------------------------------------------------------

function coniferLevel(detail: number, height: number, radius: number): BufferGeometry {
  if (detail === 2) {
    // One cone with five sides. It holds ten triangles and it keeps the
    // silhouette that names a conifer.
    return needleCone(radius * 0.92, height * 0.86, 5, height * 0.14, NEEDLE_GREEN, height);
  }

  const segments = detail === 0 ? 8 : 5;
  const parts: BufferGeometry[] = [
    trunk(radius * 0.1, height * 0.34, detail === 0 ? 6 : 4, BARK_CONIFER, height),
    needleCone(radius, height * 0.46, segments, height * 0.22, NEEDLE_GREEN, height),
    needleCone(radius * 0.74, height * 0.4, segments, height * 0.5, NEEDLE_GREEN, height),
  ];
  if (detail === 0) {
    parts.push(needleCone(radius * 0.44, height * 0.3, segments, height * 0.74, NEEDLE_GREEN, height));
  }
  return merge(parts);
}

function broadleafLevel(detail: number, height: number, radius: number): BufferGeometry {
  if (detail === 2) {
    const geometry = new OctahedronGeometry(radius, 0);
    geometry.scale(1, height / (2 * radius), 1);
    geometry.computeVertexNormals();
    return paint(geometry, LEAF_GREEN, height).translate(0, height * 0.55, 0);
  }

  const trunkHeight = height * 0.42;
  const parts: BufferGeometry[] = [
    trunk(radius * 0.13, trunkHeight, detail === 0 ? 6 : 4, BARK_BROADLEAF, height),
  ];

  if (detail === 0) {
    parts.push(clump(radius * 0.78, 0.8, -radius * 0.22, height * 0.62, 0, LEAF_GREEN, height));
    parts.push(clump(radius * 0.62, 0.78, radius * 0.34, height * 0.74, radius * 0.16, LEAF_GREEN, height));
  } else {
    parts.push(clump(radius, 0.74, 0, height * 0.66, 0, LEAF_GREEN, height));
  }

  return merge(parts);
}

function bushLevel(detail: number, height: number, radius: number): BufferGeometry {
  if (detail === 2) {
    const geometry = new OctahedronGeometry(radius, 0);
    geometry.scale(1, height / (2 * radius), 1);
    geometry.computeVertexNormals();
    return paint(geometry, SCRUB_GREEN, height).translate(0, height * 0.5, 0);
  }

  if (detail === 1) {
    return clump(radius, height / (2 * radius), 0, height * 0.5, 0, SCRUB_GREEN, height);
  }

  const parts: BufferGeometry[] = [
    clump(radius * 0.72, 0.8, -radius * 0.3, height * 0.44, radius * 0.1, SCRUB_GREEN, height),
    clump(radius * 0.6, 0.8, radius * 0.32, height * 0.56, -radius * 0.14, SCRUB_GREEN, height),
  ];
  return merge(parts);
}

// ---------------------------------------------------------------------------
// The material
// ---------------------------------------------------------------------------

function createTreeMaterial(): MeshStandardNodeMaterial {
  const material = new MeshStandardNodeMaterial({ name: 'tree' });

  // A flat green mass looks like plastic. One noise in model coordinates breaks
  // it into light and dark leaves and it does not swim when the tree sways.
  const speckle = mx_noise_float(positionGeometry.mul(FOLIAGE_NOISE_SCALE))
    .mul(FOLIAGE_NOISE_STRENGTH)
    .add(1);

  material.colorNode = attribute<'vec3'>(PART_TINT, 'vec3').mul(speckle);
  material.roughnessNode = float(TREE_ROUGHNESS);
  material.metalnessNode = float(0);

  // Read the module comment. At this point `positionLocal` already carries the
  // instance transform, so it is the world position of the vertex.
  const point = positionLocal;
  const phase = point.x.mul(WIND_WAVE_NUMBER).add(point.z.mul(WIND_WAVE_NUMBER * 0.71));
  const gust = sin(windTime.mul(WIND_RATE).add(phase));
  const ripple = sin(windTime.mul(WIND_RIPPLE_RATE).add(phase.mul(1.7))).mul(0.34);
  const wave = gust.add(ripple);

  // The lean grows with the height of the vertex, so a tall tree leans further.
  const lean = attribute<'float'>(SWAY, 'float').mul(point.y).mul(WIND_STRENGTH);
  material.positionNode = point.add(
    vec3(WIND_DIRECTION_X, 0, WIND_DIRECTION_Z).mul(wave.mul(lean)),
  );

  return material;
}

/**
 * Move the wind. `time` is the elapsed time of the world, in seconds. The call
 * writes one uniform and does no other work.
 */
export function updateTreeWind(time: number): void {
  windTime.value = time;
}

// ---------------------------------------------------------------------------
// The registry
// ---------------------------------------------------------------------------

/**
 * Distance where each level of detail gives way to the next, in meters. The
 * last value is the distance where a tree stops being drawn. The fog of
 * src/render/sky.ts hides about a seventh of the scene at 6 km, so a wood must
 * still be there at 8 km or the edge of the wood would show.
 */
const TREE_LOD_DISTANCES = [250, 1000, 8000];

/** Size of one tree, in meters. */
interface TreeSize {
  suffix: string;
  height: number;
  radius: number;
}

const CONIFER_SIZES: TreeSize[] = [
  { suffix: 'small', height: 6, radius: 1.6 },
  { suffix: 'medium', height: 12, radius: 2.6 },
  { suffix: 'large', height: 19, radius: 3.6 },
];

const BROADLEAF_SIZES: TreeSize[] = [
  { suffix: 'small', height: 5, radius: 2.2 },
  { suffix: 'medium', height: 9, radius: 3.8 },
  { suffix: 'large', height: 14, radius: 5.5 },
];

const BUSH_SIZES: TreeSize[] = [
  { suffix: 'small', height: 1.2, radius: 0.9 },
  { suffix: 'medium', height: 2, radius: 1.5 },
  { suffix: 'large', height: 3.2, radius: 2.3 },
];

function makeType(
  shape: string,
  size: TreeSize,
  material: Material,
  build: (detail: number, height: number, radius: number) => BufferGeometry,
): TreeType {
  const levels: LodLevel[] = [];
  for (let detail = 0; detail < TREE_LOD_DISTANCES.length; detail += 1) {
    levels.push({
      geometry: build(detail, size.height, size.radius),
      distance: TREE_LOD_DISTANCES[detail],
    });
  }
  return {
    name: `${shape}-${size.suffix}`,
    levels,
    material,
    radius: size.radius,
    height: size.height,
  };
}

/** Build the conifer types, in three sizes. */
export function createConiferTypes(material: Material): TreeType[] {
  return CONIFER_SIZES.map((size) => makeType('conifer', size, material, coniferLevel));
}

/** Build the broadleaf types, in three sizes. */
export function createBroadleafTypes(material: Material): TreeType[] {
  return BROADLEAF_SIZES.map((size) => makeType('broadleaf', size, material, broadleafLevel));
}

/** Build the scrub bush types, in three sizes. */
export function createBushTypes(material: Material): TreeType[] {
  return BUSH_SIZES.map((size) => makeType('bush', size, material, bushLevel));
}

/**
 * Build every tree type. All nine types share one material, so the whole wood
 * needs one shader and one pipeline.
 */
export function createTreeTypes(): TreeType[] {
  const material = createTreeMaterial();
  return [
    ...createConiferTypes(material),
    ...createBroadleafTypes(material),
    ...createBushTypes(material),
  ];
}

/**
 * Color multiplier of one tree instance.
 *
 * A wood where every tree holds the same green reads as a print of one tree.
 * The hash of the instance index gives a value that never changes, so the color
 * of one tree stays fixed while the tree moves between levels of detail. The
 * spread stays under a fifth, because a wood is not a paint chart.
 */
export function treeTint(index: number, out: Color): void {
  // A cheap integer hash. It mixes the low bits into the high bits, so two
  // trees next to each other in the placement list do not share a shade.
  let hash = Math.imul(index + 1, 0x27d4eb2d) >>> 0;
  hash ^= hash >>> 15;
  const shade = ((hash >>> 8) & 0xffff) / 0xffff;
  const hue = ((hash >>> 24) & 0xff) / 0xff;

  // Brightness from 0.82 to 1.14, and a shift between a yellow green and a
  // blue green. Both are multipliers in the working color space.
  const level = 0.82 + shade * 0.32;
  out.r = level * (0.88 + hue * 0.3);
  out.g = level * (1.02 - hue * 0.06);
  out.b = level * (1.12 - hue * 0.34);
}

/** Free every geometry of the tree types, and the material they share. */
export function disposeTreeTypes(types: readonly TreeType[]): void {
  const materials = new Set<Material>();
  for (const type of types) {
    materials.add(type.material);
    for (const level of type.levels) level.geometry.dispose();
  }
  for (const material of materials) material.dispose();
}
