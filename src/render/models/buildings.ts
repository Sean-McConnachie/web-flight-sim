/**
 * Airfield buildings.
 *
 * The module builds seven building types out of boxes, prisms, and cylinders: a
 * hangar with a curved roof and large doors, a control tower with a glazed top,
 * two sizes of hut, a workshop, a fuel dump with cylindrical tanks, and an
 * aircraft revetment. Each type carries three or four levels of detail, and the
 * coarsest level is close to a single box.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * MODEL FRAME AND THE FRONT OF A BUILDING
 *
 * Every model stands on the ground plane, so its origin sits at `y = 0` at the
 * center of its footprint and the model reaches up along `+y`. The front of a
 * building, which is the wall with the doors, faces `-z`. North is `-z` in the
 * render frame, so a model with no turn faces north. src/world/scatter.ts turns
 * a model with `rotationY = -heading`, where `heading` is the NED heading of
 * the front in radians.
 *
 *
 * WHY ONE MATERIAL SERVES EVERY TYPE
 *
 * A hangar needs corrugated iron, concrete, brick, and glass. Four materials on
 * one mesh would need four draw calls for every level of detail of every type,
 * and the whole point of the instanced draw is one call per level.
 *
 * So the surface is a vertex attribute, not a material. Each part of a model
 * carries `partTint`, which holds its base color, and `partSurface`, which
 * holds its roughness, its metalness, and a flag that marks brick. One material
 * reads the three values and shades concrete, iron, brick, and glass in one
 * pass. All seven types then share one material and one shader.
 *
 * The instance color of src/render/instanced.ts multiplies the result, so 120
 * buildings do not read as 120 copies of one paint batch.
 */

import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import type { Material } from 'three/webgpu';
import {
  BoxGeometry,
  BufferAttribute,
  BufferGeometry,
  Color,
  ConeGeometry,
  CylinderGeometry,
  MeshStandardNodeMaterial,
} from 'three/webgpu';
import {
  abs,
  attribute,
  clamp,
  float,
  fract,
  mix,
  mx_noise_float,
  positionGeometry,
  sin,
  smoothstep,
} from 'three/tsl';

import type { LodLevel } from '@/render/instanced';

/** Name of the vertex attribute that holds the base color of a part. */
const PART_TINT = 'partTint';

/**
 * Name of the vertex attribute that holds the surface of a part. The x value is
 * the roughness, the y value is the metalness, and the z value is 1 on brick.
 */
const PART_SURFACE = 'partSurface';

/** Distance between the ribs of a corrugated iron sheet, in meters. */
const CORRUGATION_PITCH = 0.19;

/** Distance between the mortar lines of a brick wall, in meters. */
const BRICK_COURSE_PITCH = 0.3;

/** Feature size of the weather noise, in meters, and its brightness swing. */
const WEATHER_SCALE = 0.55;
const WEATHER_STRENGTH = 0.09;

/** Height that rain splash dirties, in meters. */
const SPLASH_HEIGHT = 1.6;

export interface BuildingType {
  name: string;
  levels: LodLevel[];
  material: Material;
  /** Size of the footprint in the model frame, in meters. */
  footprint: { x: number; z: number };
  /** Height of the highest point, in meters. */
  height: number;
}

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------

interface SurfaceStyle {
  /** Base color, as an sRGB hex value. */
  color: number;
  roughness: number;
  metalness: number;
  /** 1 marks brick, so the material can draw the mortar courses. */
  brick: number;
}

const CONCRETE: SurfaceStyle = { color: 0x9b988f, roughness: 0.88, metalness: 0, brick: 0 };
const IRON: SurfaceStyle = { color: 0x767b78, roughness: 0.52, metalness: 0.8, brick: 0 };
const BRICK: SurfaceStyle = { color: 0x8a5340, roughness: 0.93, metalness: 0, brick: 1 };
const GLASS: SurfaceStyle = { color: 0x33454d, roughness: 0.07, metalness: 0.25, brick: 0 };
const ROOFING: SurfaceStyle = { color: 0x4b4c47, roughness: 0.86, metalness: 0.05, brick: 0 };
const EARTH: SurfaceStyle = { color: 0x6b6047, roughness: 0.98, metalness: 0, brick: 0 };
const DOOR: SurfaceStyle = { color: 0x44514a, roughness: 0.62, metalness: 0.35, brick: 0 };

/** Scratch color. The module allocates it one time. */
const scratchColor = new Color();

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/**
 * Give one geometry the two part attributes and drop its index.
 *
 * `mergeGeometries` needs every input to hold the same attributes and to agree
 * on the index, so every part goes through this function first.
 */
function paint(geometry: BufferGeometry, style: SurfaceStyle): BufferGeometry {
  const flat = geometry.index !== null ? geometry.toNonIndexed() : geometry;
  if (flat !== geometry) geometry.dispose();

  const count = flat.getAttribute('position').count;
  const tint = new Float32Array(count * 3);
  const surface = new Float32Array(count * 3);

  scratchColor.setHex(style.color);
  for (let i = 0; i < count; i += 1) {
    tint[i * 3 + 0] = scratchColor.r;
    tint[i * 3 + 1] = scratchColor.g;
    tint[i * 3 + 2] = scratchColor.b;
    surface[i * 3 + 0] = style.roughness;
    surface[i * 3 + 1] = style.metalness;
    surface[i * 3 + 2] = style.brick;
  }

  flat.setAttribute(PART_TINT, new BufferAttribute(tint, 3));
  flat.setAttribute(PART_SURFACE, new BufferAttribute(surface, 3));
  return flat;
}

/** A box that stands on the ground, centered on `x` and `z`. */
function box(
  width: number,
  height: number,
  depth: number,
  style: SurfaceStyle,
  x = 0,
  y = 0,
  z = 0,
): BufferGeometry {
  return paint(new BoxGeometry(width, height, depth), style).translate(x, y + height / 2, z);
}

/** An upright cylinder that stands on `y`. */
function pipe(
  radiusTop: number,
  radiusBottom: number,
  height: number,
  segments: number,
  style: SurfaceStyle,
  x = 0,
  y = 0,
  z = 0,
): BufferGeometry {
  const geometry = new CylinderGeometry(radiusTop, radiusBottom, height, segments);
  return paint(geometry, style).translate(x, y + height / 2, z);
}

/** An upright cone that stands on `y`. */
function cone(
  radius: number,
  height: number,
  segments: number,
  style: SurfaceStyle,
  x = 0,
  y = 0,
  z = 0,
): BufferGeometry {
  return paint(new ConeGeometry(radius, height, segments), style).translate(x, y + height / 2, z);
}

/**
 * A half cylinder that lies with its axis along z and its open side down. The
 * shape is the curved roof of a hangar or of a Nissen hut.
 *
 * The half cylinder starts with its axis along y, so two quarter turns lay it
 * down and point the solid half up. A scale on y then flattens the arch to the
 * rise the caller asks for, and the normals are computed after the scale,
 * because a scale that is not uniform does not carry a normal.
 */
function barrel(
  halfWidth: number,
  rise: number,
  depth: number,
  segments: number,
  style: SurfaceStyle,
  y: number,
): BufferGeometry {
  const geometry = new CylinderGeometry(halfWidth, halfWidth, depth, segments, 1, false, 0, Math.PI);
  geometry.rotateZ(Math.PI / 2);
  geometry.rotateY(Math.PI / 2);
  geometry.scale(1, rise / halfWidth, 1);
  geometry.computeVertexNormals();
  return paint(geometry, style).translate(0, y, 0);
}

/** One point of an extrusion profile, in the model x and y axes. */
type ProfilePoint = readonly [number, number];

/**
 * Extrude a closed convex profile along the z axis.
 *
 * One call gives the walls, the gable, and the roof of a hut as a single shape
 * with flat faces and with an end wall at each end. The profile must run
 * counterclockwise in the x and y plane, and it must be convex, because each
 * end wall is one triangle fan from the first point.
 *
 * `edgeStyles` holds one surface for each edge, in profile order. The edge from
 * the last point back to the first point is the last entry.
 */
function extrudeProfile(
  profile: readonly ProfilePoint[],
  depth: number,
  edgeStyles: readonly SurfaceStyle[],
  capStyle: SurfaceStyle,
): BufferGeometry {
  const n = profile.length;
  if (n < 3) throw new RangeError('An extrusion profile needs at least three points.');
  if (edgeStyles.length !== n) throw new RangeError('One edge style per profile edge.');

  const half = depth / 2;
  const vertexCount = n * 6 + (n - 2) * 6;
  const positions = new Float32Array(vertexCount * 3);
  const normals = new Float32Array(vertexCount * 3);
  const uvs = new Float32Array(vertexCount * 2);
  const tints = new Float32Array(vertexCount * 3);
  const surfaces = new Float32Array(vertexCount * 3);

  let cursor = 0;

  function push(
    x: number,
    y: number,
    z: number,
    nx: number,
    ny: number,
    nz: number,
    u: number,
    v: number,
    style: SurfaceStyle,
  ): void {
    positions[cursor * 3 + 0] = x;
    positions[cursor * 3 + 1] = y;
    positions[cursor * 3 + 2] = z;
    normals[cursor * 3 + 0] = nx;
    normals[cursor * 3 + 1] = ny;
    normals[cursor * 3 + 2] = nz;
    uvs[cursor * 2 + 0] = u;
    uvs[cursor * 2 + 1] = v;
    scratchColor.setHex(style.color);
    tints[cursor * 3 + 0] = scratchColor.r;
    tints[cursor * 3 + 1] = scratchColor.g;
    tints[cursor * 3 + 2] = scratchColor.b;
    surfaces[cursor * 3 + 0] = style.roughness;
    surfaces[cursor * 3 + 1] = style.metalness;
    surfaces[cursor * 3 + 2] = style.brick;
    cursor += 1;
  }

  // The side faces. The profile runs counterclockwise, so the interior lies to
  // the left of every edge and the outward normal of an edge (dx, dy) is
  // (dy, -dx).
  let along = 0;
  for (let k = 0; k < n; k += 1) {
    const a = profile[k];
    const b = profile[(k + 1) % n];
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    const length = Math.hypot(dx, dy);
    if (length === 0) throw new RangeError('An extrusion profile holds a repeated point.');
    const nx = dy / length;
    const ny = -dx / length;
    const style = edgeStyles[k];
    const u0 = along;
    const u1 = along + length;
    along = u1;

    push(a[0], a[1], -half, nx, ny, 0, u0, 0, style);
    push(b[0], b[1], -half, nx, ny, 0, u1, 0, style);
    push(b[0], b[1], half, nx, ny, 0, u1, depth, style);

    push(a[0], a[1], -half, nx, ny, 0, u0, 0, style);
    push(b[0], b[1], half, nx, ny, 0, u1, depth, style);
    push(a[0], a[1], half, nx, ny, 0, u0, depth, style);
  }

  // The two end walls, as a fan from the first point of the profile.
  const first = profile[0];
  for (let k = 1; k < n - 1; k += 1) {
    const b = profile[k];
    const c = profile[k + 1];
    push(first[0], first[1], half, 0, 0, 1, first[0], first[1], capStyle);
    push(b[0], b[1], half, 0, 0, 1, b[0], b[1], capStyle);
    push(c[0], c[1], half, 0, 0, 1, c[0], c[1], capStyle);

    push(first[0], first[1], -half, 0, 0, -1, first[0], first[1], capStyle);
    push(c[0], c[1], -half, 0, 0, -1, c[0], c[1], capStyle);
    push(b[0], b[1], -half, 0, 0, -1, b[0], b[1], capStyle);
  }

  const geometry = new BufferGeometry();
  geometry.setAttribute('position', new BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new BufferAttribute(normals, 3));
  geometry.setAttribute('uv', new BufferAttribute(uvs, 2));
  geometry.setAttribute(PART_TINT, new BufferAttribute(tints, 3));
  geometry.setAttribute(PART_SURFACE, new BufferAttribute(surfaces, 3));
  return geometry;
}

/** Join every part into one geometry and free the parts. */
function merge(parts: BufferGeometry[]): BufferGeometry {
  const merged: BufferGeometry | null = mergeGeometries(parts, false);
  if (merged === null) throw new Error('The building parts do not share one attribute set.');
  for (const part of parts) part.dispose();
  return merged;
}

// ---------------------------------------------------------------------------
// The models
// ---------------------------------------------------------------------------

/**
 * Distance where each level of detail gives way to the next, in meters. The
 * last value is also the distance where a building stops being drawn. A 15 m
 * building at 12 km covers less than one pixel at any usable field of view.
 */
const BUILDING_LOD_DISTANCES = [400, 1200, 4000, 12000];

/** A revetment is low and flat, so it leaves the picture much sooner. */
const REVETMENT_LOD_DISTANCES = [300, 900, 2500, 6000];

/** Hangar sizes, in meters. */
const HANGAR_WIDTH = 34;
const HANGAR_DEPTH = 26;
const HANGAR_WALL = 6;
const HANGAR_RISE = 8;
const HANGAR_DOOR_WIDTH = 24;

function hangarLevel(detail: number): BufferGeometry {
  const parts: BufferGeometry[] = [];
  const halfWidth = HANGAR_WIDTH / 2;
  const halfDepth = HANGAR_DEPTH / 2;

  if (detail === 3) {
    return box(HANGAR_WIDTH, HANGAR_WALL + HANGAR_RISE * 0.7, HANGAR_DEPTH, IRON);
  }

  if (detail === 2) {
    parts.push(box(HANGAR_WIDTH, HANGAR_WALL, HANGAR_DEPTH, IRON));
    parts.push(barrel(halfWidth, HANGAR_RISE, HANGAR_DEPTH, 5, IRON, HANGAR_WALL));
    return merge(parts);
  }

  const segments = detail === 0 ? 12 : 8;

  // A concrete plinth keeps the iron sheet out of the mud.
  parts.push(box(HANGAR_WIDTH + 1, 0.4, HANGAR_DEPTH + 1, CONCRETE));

  // Side walls and back wall.
  parts.push(box(0.8, HANGAR_WALL, HANGAR_DEPTH, IRON, -halfWidth + 0.4, 0.4));
  parts.push(box(0.8, HANGAR_WALL, HANGAR_DEPTH, IRON, halfWidth - 0.4, 0.4));
  parts.push(box(HANGAR_WIDTH, HANGAR_WALL, 0.8, IRON, 0, 0.4, halfDepth - 0.4));

  // The front wall is two piers with the door opening between them.
  const pier = (HANGAR_WIDTH - HANGAR_DOOR_WIDTH) / 2;
  parts.push(box(pier, HANGAR_WALL, 0.8, IRON, -(halfWidth - pier / 2), 0.4, -halfDepth + 0.4));
  parts.push(box(pier, HANGAR_WALL, 0.8, IRON, halfWidth - pier / 2, 0.4, -halfDepth + 0.4));

  if (detail === 0) {
    // Four sliding door leaves, with a gap between them that reads as a joint.
    const leaf = HANGAR_DOOR_WIDTH / 4;
    for (let i = 0; i < 4; i += 1) {
      const x = -HANGAR_DOOR_WIDTH / 2 + leaf * (i + 0.5);
      parts.push(box(leaf - 0.12, HANGAR_WALL, 0.5, DOOR, x, 0.4, -halfDepth + 0.25));
    }
    // Two roof vents.
    parts.push(box(1.6, 0.9, 1.6, IRON, -6, HANGAR_WALL + HANGAR_RISE - 0.5, 0));
    parts.push(box(1.6, 0.9, 1.6, IRON, 6, HANGAR_WALL + HANGAR_RISE - 0.5, 0));
  } else {
    parts.push(box(HANGAR_DOOR_WIDTH, HANGAR_WALL, 0.5, DOOR, 0, 0.4, -halfDepth + 0.25));
  }

  parts.push(barrel(halfWidth, HANGAR_RISE, HANGAR_DEPTH, segments, IRON, HANGAR_WALL));
  return merge(parts);
}

/** Control tower sizes, in meters. */
const TOWER_SHAFT = 7;
const TOWER_CAB = 11;
const TOWER_SHAFT_HEIGHT = 12;
const TOWER_GLASS_HEIGHT = 2.8;
const TOWER_HEIGHT = 16;

function towerLevel(detail: number): BufferGeometry {
  if (detail === 3) return box(TOWER_CAB, TOWER_HEIGHT, TOWER_CAB, CONCRETE);

  const parts: BufferGeometry[] = [];

  if (detail === 2) {
    parts.push(box(TOWER_SHAFT, TOWER_SHAFT_HEIGHT, TOWER_SHAFT, CONCRETE));
    parts.push(box(TOWER_CAB, TOWER_HEIGHT - TOWER_SHAFT_HEIGHT, TOWER_CAB, GLASS, 0, TOWER_SHAFT_HEIGHT));
    return merge(parts);
  }

  parts.push(box(TOWER_SHAFT + 2, 1, TOWER_SHAFT + 2, CONCRETE));
  parts.push(box(TOWER_SHAFT, TOWER_SHAFT_HEIGHT - 1, TOWER_SHAFT, CONCRETE, 0, 1));
  // The cab floor overhangs the shaft, which is the shape that makes a control
  // tower read as a control tower from a long way off.
  parts.push(box(TOWER_CAB, 0.5, TOWER_CAB, CONCRETE, 0, TOWER_SHAFT_HEIGHT));
  parts.push(
    box(TOWER_CAB - 0.6, TOWER_GLASS_HEIGHT, TOWER_CAB - 0.6, GLASS, 0, TOWER_SHAFT_HEIGHT + 0.5),
  );
  parts.push(
    box(TOWER_CAB + 0.8, 0.4, TOWER_CAB + 0.8, CONCRETE, 0, TOWER_SHAFT_HEIGHT + 0.5 + TOWER_GLASS_HEIGHT),
  );

  if (detail === 0) {
    const railTop = TOWER_SHAFT_HEIGHT + 0.9 + TOWER_GLASS_HEIGHT;
    const railHalf = (TOWER_CAB + 0.8) / 2;
    parts.push(box(TOWER_CAB + 0.8, 0.9, 0.12, IRON, 0, railTop, -railHalf));
    parts.push(box(TOWER_CAB + 0.8, 0.9, 0.12, IRON, 0, railTop, railHalf));
    parts.push(box(0.12, 0.9, TOWER_CAB + 0.8, IRON, -railHalf, railTop));
    parts.push(box(0.12, 0.9, TOWER_CAB + 0.8, IRON, railHalf, railTop));
    parts.push(pipe(0.08, 0.12, 4.5, 5, IRON, 0, railTop + 0.9));
    parts.push(box(1.1, 2.2, 0.2, DOOR, 0, 0, -TOWER_SHAFT / 2 - 0.05));
    parts.push(box(1.2, 1.2, 0.2, GLASS, -2, 5, -TOWER_SHAFT / 2 - 0.05));
    parts.push(box(1.2, 1.2, 0.2, GLASS, 2, 5, -TOWER_SHAFT / 2 - 0.05));
  }

  return merge(parts);
}

/**
 * A hut with a gabled roof. One extrusion gives the two side walls, the two
 * roof slopes, and the end walls.
 */
function hutLevel(
  detail: number,
  width: number,
  depth: number,
  eaves: number,
  ridge: number,
  windows: number,
): BufferGeometry {
  const halfWidth = width / 2;
  const halfDepth = depth / 2;

  if (detail >= 2) return box(width, (eaves + ridge) / 2, depth, BRICK);

  const profile: ProfilePoint[] = [
    [-halfWidth, 0],
    [halfWidth, 0],
    [halfWidth, eaves],
    [0, ridge],
    [-halfWidth, eaves],
  ];
  const edges = [CONCRETE, BRICK, ROOFING, ROOFING, BRICK];
  const parts: BufferGeometry[] = [extrudeProfile(profile, depth, edges, BRICK)];

  parts.push(box(1.1, 2.1, 0.16, DOOR, -halfWidth + 1.4, 0, -halfDepth - 0.06));

  if (detail === 0) {
    for (let i = 0; i < windows; i += 1) {
      const step = depth / (windows + 1);
      const z = -halfDepth + step * (i + 1);
      parts.push(box(0.16, 1.1, 1.2, GLASS, -halfWidth - 0.06, 1.1, z));
      parts.push(box(0.16, 1.1, 1.2, GLASS, halfWidth + 0.06, 1.1, z));
    }
    parts.push(box(0.7, 1.4, 0.7, BRICK, halfWidth - 1.2, eaves + 0.6, halfDepth - 1.5));
  }

  return merge(parts);
}

/** Workshop sizes, in meters. A shallow iron roof over brick walls. */
function workshopLevel(detail: number): BufferGeometry {
  const width = 18;
  const depth = 12;
  const eaves = 5;
  const ridge = 6.4;

  if (detail === 3) return box(width, eaves, depth, BRICK);
  if (detail === 2) return box(width, (eaves + ridge) / 2, depth, BRICK);

  const halfWidth = width / 2;
  const halfDepth = depth / 2;
  const profile: ProfilePoint[] = [
    [-halfWidth, 0],
    [halfWidth, 0],
    [halfWidth, eaves],
    [0, ridge],
    [-halfWidth, eaves],
  ];
  const parts: BufferGeometry[] = [
    extrudeProfile(profile, depth, [CONCRETE, BRICK, IRON, IRON, BRICK], BRICK),
    box(4.2, 3.6, 0.2, DOOR, 0, 0, -halfDepth - 0.08),
  ];

  if (detail === 0) {
    for (let i = -1; i <= 1; i += 1) {
      parts.push(box(1.4, 1.6, 0.2, GLASS, i * 5, 2.6, halfDepth + 0.08));
    }
    parts.push(box(0.9, 2.2, 0.9, BRICK, halfWidth - 2, eaves + 0.9, 0));
  }

  return merge(parts);
}

/** Fuel dump sizes, in meters. */
const TANK_RADIUS = 3.6;
const TANK_HEIGHT = 7.2;
const FUEL_WIDTH = 26;
const FUEL_DEPTH = 18;
const FUEL_HEIGHT = 9.6;

function fuelDumpLevel(detail: number): BufferGeometry {
  if (detail === 3) return box(FUEL_WIDTH, FUEL_HEIGHT * 0.6, FUEL_DEPTH, CONCRETE);

  const parts: BufferGeometry[] = [];
  const halfWidth = FUEL_WIDTH / 2;
  const halfDepth = FUEL_DEPTH / 2;

  if (detail === 2) {
    parts.push(pipe(TANK_RADIUS, TANK_RADIUS, TANK_HEIGHT, 6, IRON, 0, 1.2));
    parts.push(box(FUEL_WIDTH, 1.8, FUEL_DEPTH, CONCRETE));
    return merge(parts);
  }

  const segments = detail === 0 ? 12 : 8;

  // The bund wall holds the fuel in if a tank splits. Four low concrete walls.
  parts.push(box(FUEL_WIDTH, 1.8, 0.6, CONCRETE, 0, 0, -halfDepth));
  parts.push(box(FUEL_WIDTH, 1.8, 0.6, CONCRETE, 0, 0, halfDepth));
  parts.push(box(0.6, 1.8, FUEL_DEPTH, CONCRETE, -halfWidth, 0));
  parts.push(box(0.6, 1.8, FUEL_DEPTH, CONCRETE, halfWidth, 0));

  for (const x of [-6.5, 6.5]) {
    parts.push(pipe(TANK_RADIUS, TANK_RADIUS, TANK_HEIGHT, segments, IRON, x, 1.2));
    parts.push(cone(TANK_RADIUS, 1.2, segments, IRON, x, 1.2 + TANK_HEIGHT));
  }

  if (detail === 0) {
    // The transfer line runs across the bund between the two tanks. The
    // cylinder starts upright, so a quarter turn about z lays it along x.
    const runner = paint(new CylinderGeometry(0.22, 0.22, 13, 5), IRON);
    runner.rotateZ(Math.PI / 2);
    runner.translate(0, 2.4, 0);
    parts.push(runner);
    parts.push(box(3, 2.6, 3, BRICK, 0, 0, halfDepth - 3));
  }

  return merge(parts);
}

/** Revetment sizes, in meters. Three earth banks around a concrete hardstand. */
const REVETMENT_WIDTH = 30;
const REVETMENT_DEPTH = 26;
const REVETMENT_HEIGHT = 4.5;

function revetmentLevel(detail: number): BufferGeometry {
  const halfWidth = REVETMENT_WIDTH / 2;
  const halfDepth = REVETMENT_DEPTH / 2;

  if (detail === 3) return box(REVETMENT_WIDTH, REVETMENT_HEIGHT * 0.5, REVETMENT_DEPTH, EARTH);

  const parts: BufferGeometry[] = [];

  if (detail === 2) {
    parts.push(box(REVETMENT_WIDTH, REVETMENT_HEIGHT, 5, EARTH, 0, 0, halfDepth - 2.5));
    parts.push(box(5, REVETMENT_HEIGHT, REVETMENT_DEPTH - 5, EARTH, -halfWidth + 2.5, 0, -2.5));
    parts.push(box(5, REVETMENT_HEIGHT, REVETMENT_DEPTH - 5, EARTH, halfWidth - 2.5, 0, -2.5));
    return merge(parts);
  }

  // The back bank runs across the model. Its profile is a trapezoid, which is
  // the shape an earth bank keeps when it settles.
  const backProfile: ProfilePoint[] = [
    [-halfWidth, 0],
    [halfWidth, 0],
    [halfWidth - 3.5, REVETMENT_HEIGHT],
    [-halfWidth + 3.5, REVETMENT_HEIGHT],
  ];
  parts.push(
    extrudeProfile(backProfile, 5, [EARTH, EARTH, EARTH, EARTH], EARTH).translate(
      0,
      0,
      halfDepth - 2.5,
    ),
  );

  // The two side banks run along the model, so they extrude the full depth.
  const sideProfile: ProfilePoint[] = [
    [-2.5, 0],
    [2.5, 0],
    [1.6, REVETMENT_HEIGHT],
    [-1.6, REVETMENT_HEIGHT],
  ];
  for (const x of [-halfWidth + 2.5, halfWidth - 2.5]) {
    parts.push(
      extrudeProfile(sideProfile, REVETMENT_DEPTH - 5, [EARTH, EARTH, EARTH, EARTH], EARTH).translate(
        x,
        0,
        -2.5,
      ),
    );
  }

  parts.push(box(REVETMENT_WIDTH - 8, 0.25, REVETMENT_DEPTH - 6, CONCRETE, 0, 0, -1));
  return merge(parts);
}

// ---------------------------------------------------------------------------
// The material
// ---------------------------------------------------------------------------

function createBuildingMaterial(): MeshStandardNodeMaterial {
  const material = new MeshStandardNodeMaterial({ name: 'building' });

  const tint = attribute<'vec3'>(PART_TINT, 'vec3');
  const surface = attribute<'vec3'>(PART_SURFACE, 'vec3');
  const metal = surface.y;
  const point = positionGeometry;

  // Corrugated iron. The ripple runs up the model, so it makes a horizontal rib
  // on a wall and a rib along the axis on a curved roof, which is how a sheet
  // of corrugated iron really lies on both.
  const ripple = sin(point.y.mul((2 * Math.PI) / CORRUGATION_PITCH)).mul(0.5).add(0.5);
  const ironShade = mix(float(1), mix(float(0.86), float(1.08), ripple), metal);

  // Brick courses. A mortar line is a thin darker band at every course height.
  const course = abs(fract(point.y.mul(1 / BRICK_COURSE_PITCH)).sub(0.5));
  const mortar = smoothstep(0, 0.1, course);
  const brickShade = mix(float(1), mix(float(0.76), float(1), mortar), surface.z);

  // Weather. One noise breaks the flat look of concrete, of paint, and of earth.
  const weather = mx_noise_float(point.mul(WEATHER_SCALE)).mul(WEATHER_STRENGTH).add(1);

  // Rain throws soil against the bottom of every wall. A wall that is clean
  // down to the grass reads as a toy.
  const splash = smoothstep(0, SPLASH_HEIGHT, point.y);
  const dirt = mix(float(0.7), float(1), splash);

  material.colorNode = tint.mul(ironShade).mul(brickShade).mul(weather).mul(dirt);
  material.roughnessNode = clamp(
    surface.x.add(ripple.sub(0.5).mul(0.07).mul(metal)).add(splash.oneMinus().mul(0.1)),
    0.04,
    1,
  );
  material.metalnessNode = metal;

  return material;
}

// ---------------------------------------------------------------------------
// The registry
// ---------------------------------------------------------------------------

function makeLevels(
  build: (detail: number) => BufferGeometry,
  distances: readonly number[],
): LodLevel[] {
  const levels: LodLevel[] = [];
  for (let detail = 0; detail < distances.length; detail += 1) {
    levels.push({ geometry: build(detail), distance: distances[detail] });
  }
  return levels;
}

/** Build the hangar. Its doors face `-z`. */
export function createHangarType(material: Material): BuildingType {
  return {
    name: 'hangar',
    levels: makeLevels(hangarLevel, BUILDING_LOD_DISTANCES),
    material,
    footprint: { x: HANGAR_WIDTH + 1, z: HANGAR_DEPTH + 1 },
    height: HANGAR_WALL + HANGAR_RISE,
  };
}

/** Build the control tower. Its door faces `-z`. */
export function createTowerType(material: Material): BuildingType {
  return {
    name: 'tower',
    levels: makeLevels(towerLevel, BUILDING_LOD_DISTANCES),
    material,
    footprint: { x: TOWER_CAB + 0.8, z: TOWER_CAB + 0.8 },
    height: TOWER_HEIGHT + 5.4,
  };
}

/** Build the small hut. Its door faces `-z`. */
export function createHutType(material: Material): BuildingType {
  return {
    name: 'hut',
    levels: makeLevels((detail) => hutLevel(detail, 10, 6, 2.8, 4.6, 2), [400, 1500, 5000]),
    material,
    footprint: { x: 10, z: 6 },
    height: 4.6,
  };
}

/** Build the barracks block. Its doors face `-z`. */
export function createBarracksType(material: Material): BuildingType {
  return {
    name: 'barracks',
    levels: makeLevels((detail) => hutLevel(detail, 24, 8, 3.4, 5.4, 4), BUILDING_LOD_DISTANCES),
    material,
    footprint: { x: 24, z: 8 },
    height: 5.4,
  };
}

/** Build the workshop. Its doors face `-z`. */
export function createWorkshopType(material: Material): BuildingType {
  return {
    name: 'workshop',
    levels: makeLevels(workshopLevel, BUILDING_LOD_DISTANCES),
    material,
    footprint: { x: 18, z: 12 },
    height: 6.4 + 2.2,
  };
}

/** Build the fuel dump. */
export function createFuelDumpType(material: Material): BuildingType {
  return {
    name: 'fuel-dump',
    levels: makeLevels(fuelDumpLevel, BUILDING_LOD_DISTANCES),
    material,
    footprint: { x: FUEL_WIDTH, z: FUEL_DEPTH },
    height: FUEL_HEIGHT,
  };
}

/** Build the aircraft revetment. Its opening faces `-z`. */
export function createRevetmentType(material: Material): BuildingType {
  return {
    name: 'revetment',
    levels: makeLevels(revetmentLevel, REVETMENT_LOD_DISTANCES),
    material,
    footprint: { x: REVETMENT_WIDTH, z: REVETMENT_DEPTH },
    height: REVETMENT_HEIGHT,
  };
}

/**
 * Build every building type. All types share one material, so the whole set
 * needs one shader and one pipeline.
 */
export function createBuildingTypes(): BuildingType[] {
  const material = createBuildingMaterial();
  return [
    createHangarType(material),
    createTowerType(material),
    createHutType(material),
    createBarracksType(material),
    createWorkshopType(material),
    createFuelDumpType(material),
    createRevetmentType(material),
  ];
}

/**
 * Color multiplier of one building instance.
 *
 * A row of 120 buildings in one exact shade reads as a print of one building.
 * The hash of the instance index gives a value that never changes, so the color
 * of one building stays fixed while the building moves between levels of
 * detail. The spread is small, because these are painted walls and not leaves.
 */
export function buildingTint(index: number, out: Color): void {
  // A cheap integer hash. It mixes the low bits into the high bits, so two
  // buildings next to each other in the placement list do not share a shade.
  let hash = Math.imul(index + 1, 0x9e3779b1) >>> 0;
  hash ^= hash >>> 13;
  const shade = ((hash >>> 8) & 0xffff) / 0xffff;
  const warm = ((hash >>> 24) & 0xff) / 0xff;

  const level = 0.86 + shade * 0.26;
  out.r = level * (0.95 + warm * 0.12);
  out.g = level;
  out.b = level * (1.05 - warm * 0.12);
}

/** Free every geometry of the building types, and the material they share. */
export function disposeBuildingTypes(types: readonly BuildingType[]): void {
  const materials = new Set<Material>();
  for (const type of types) {
    materials.add(type.material);
    for (const level of type.levels) level.geometry.dispose();
  }
  for (const material of materials) material.dispose();
}
