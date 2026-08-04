/**
 * Concrete runway, apron, and taxi strip.
 *
 * The runway lies along the north axis. North is `-z` in the render frame, so
 * the strip runs from the threshold at the origin toward `-z`.
 *
 *
 * WHERE THE THRESHOLD SITS, AND WHY
 *
 * CONVENTIONS 3.2 states that the NED origin sits at the runway threshold, at
 * ground level. The player spawns at the north facing threshold and rolls
 * north. So the threshold is the south end of the pavement, and it is the NED
 * origin itself:
 *
 *   RUNWAY_THRESHOLD_NED = { x: 0, y: 0, z: 0 }
 *
 * The value is not `runwayLength / 2` and it is not the far end. Any other
 * value would move the origin off the threshold and would break the rule that
 * altitude above the ground is `-position.z`. The pavement therefore covers
 * north 0 m to north 2400 m, and it is not centered on the origin.
 *
 * `RUNWAY_HEADING_RAD` is 0, because the takeoff run points north.
 *
 *
 * HOW THE MARKINGS ARE DRAWN, AND WHY
 *
 * The markings come from one canvas texture, not from thin boxes.
 *
 * Boxes would need a separate mesh or an instance buffer for every bar, every
 * dash, and every line. Each one would sit a fraction of a millimeter above the
 * concrete and would fight it for depth at 2000 m. The canvas holds all of them
 * in one image, on one mesh, with one draw call, and the mip chain fades the
 * paint out at range with no flicker. The runway is a flat rectangle, so a
 * texture fits it with no distortion.
 *
 * The slab joints are not in that image. A 2400 m strip gives only 1.7 texels
 * per meter along its length, so a 0.15 m joint would be a tenth of a texel.
 * The joints are computed in TSL from the world position instead, and their
 * width follows the screen space derivative. They stay crisp near the aircraft,
 * where they are the strongest speed cue during the 1100 m takeoff run, and
 * they fade to an even gray at range with no aliasing. src/render/ground.ts
 * uses the other method, a plain camera distance fade, for its grass detail.
 */

import type { Node, Object3D } from 'three/webgpu';
import {
  CanvasTexture,
  Group,
  Mesh,
  MeshStandardNodeMaterial,
  PlaneGeometry,
  Vector3,
} from 'three/webgpu';
import {
  Fn,
  color,
  dot,
  float,
  floor,
  fract,
  fwidth,
  max,
  min,
  mix,
  positionWorld,
  sin,
  smoothstep,
  texture,
  uv,
  vec2,
} from 'three/tsl';

import { config } from '@/core/config';
import { nedToThree } from '@/render/frames';

/**
 * Height of the pavement above the ground plane, in meters. The offset keeps
 * the concrete from fighting the grass for depth. It is far below the wheel
 * radius, so the physics can still treat the ground as `z = 0`.
 */
const RUNWAY_HEIGHT = 0.02;

/** Height of the apron and the taxi strip. They must lose to the runway. */
const APRON_HEIGHT = 0.012;

/** Side of one concrete slab, in meters. Real slabs run 4.5 m to 7.5 m. */
const SLAB_SIZE = 5;

/** Width of the dark band at a slab joint, in meters. */
const JOINT_WIDTH = 0.15;

/** Brightness swing between neighboring slabs, as a fraction. */
const SLAB_TINT_STRENGTH = 0.1;

/** Roughness of concrete and of the paint on top of it. */
const CONCRETE_ROUGHNESS = 0.85;
const PAINT_ROUGHNESS = 0.6;
const APRON_ROUGHNESS = 0.92;

/** Base and joint colors, as sRGB hex. */
const RUNWAY_CONCRETE = 0x8f8d86;
const RUNWAY_JOINT = 0x55534e;
const APRON_CONCRETE = 0x6d6b66;
const APRON_JOINT = 0x3f3e3a;
const PAINT_WHITE = 0xe8e6dc;

/**
 * Size of the marking image. The width gives 11.4 texels per meter across the
 * runway, which is enough for a 0.9 m line. The length gives 1.7 texels per
 * meter, which is enough for a 30 m bar.
 */
const MARKING_TEXTURE_WIDTH = 512;
const MARKING_TEXTURE_HEIGHT = 4096;

/** Anisotropic filter samples. The runway is always seen at a grazing angle. */
const MARKING_ANISOTROPY = 4;

/** Marking sizes, in meters. The layout follows the ICAO Annex 14 pattern. */
const EDGE_LINE_WIDTH = 0.9;
const EDGE_LINE_INSET = 0.6;
const CENTERLINE_WIDTH = 0.9;
const CENTERLINE_STRIPE = 30;
const CENTERLINE_GAP = 20;
const CENTERLINE_START = 12;
const THRESHOLD_BAR_START = 6;
const THRESHOLD_BAR_LENGTH = 30;
const THRESHOLD_BAR_WIDTH = 1.8;
const THRESHOLD_BAR_PITCH = 3.6;
const THRESHOLD_BAR_INNER = 1.8;
const THRESHOLD_BAR_PAIRS = 6;
const AIMING_POINT_DISTANCE = 300;
const AIMING_POINT_LENGTH = 45;
const AIMING_POINT_WIDTH = 6;
const AIMING_POINT_INNER = 11;
const TOUCHDOWN_LENGTH = 22.5;
const TOUCHDOWN_WIDTH = 3;
const TOUCHDOWN_INNER = 8;
const TOUCHDOWN_DISTANCES = [150, 450, 600];

/** Taxi strip and apron layout, in meters, in NED. */
const TAXIWAY_WIDTH = 23;
const TAXIWAY_EAST = 100;
const TAXIWAY_START = -40;
const TAXIWAY_END = 1000;
const LINK_NORTH = 40;
const APRON_NORTH_START = -40;
const APRON_NORTH_END = 160;
const APRON_EAST_START = 130;
const APRON_EAST_END = 330;

/**
 * Threshold of the runway, in the NED world frame, in meters. The point is the
 * NED origin, because CONVENTIONS 3.2 puts the origin at the threshold. The
 * aircraft spawns here and rolls north.
 */
export const RUNWAY_THRESHOLD_NED: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 };

/** Heading of the takeoff run, in radians. Zero is north. */
export const RUNWAY_HEADING_RAD = 0;

/** Scratch vectors. The module allocates them one time. */
const scratchNed = new Vector3();
const scratchThree = new Vector3();

/** Hash of a slab index, from 0 to 1. See src/render/ground.ts for the reason. */
const hashLattice = Fn(([cell]: [Node<'vec2'>]) => {
  return fract(sin(dot(cell, vec2(127.1, 311.7))).mul(43758.5453123));
});

/**
 * Draw the painted markings into an alpha mask. White paint sits in the color
 * channels and the coverage sits in alpha, so the material can lay the paint
 * over the concrete that TSL builds underneath.
 *
 * The image runs from the threshold at the bottom to the far end at the top,
 * because a texture with `flipY` puts v = 0 on the bottom row. The pattern is
 * the same at both ends, so the runway reads correctly from either direction.
 */
function drawMarkings(): HTMLCanvasElement {
  const length = config.world.runwayLength;
  const width = config.world.runwayWidth;

  const canvas = document.createElement('canvas');
  canvas.width = MARKING_TEXTURE_WIDTH;
  canvas.height = MARKING_TEXTURE_HEIGHT;

  const context = canvas.getContext('2d');
  if (context === null) throw new Error('The runway markings need a 2D canvas context.');
  // Hold the narrowed value, because a closure cannot keep the narrowing.
  const ctx: CanvasRenderingContext2D = context;

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#ffffff';

  const acrossScale = canvas.width / width;
  const alongScale = canvas.height / length;

  /**
   * Paint one rectangle. `start` is the distance north of the threshold of the
   * south edge of the rectangle. `offset` is the distance east of the
   * centerline of the center of the rectangle.
   */
  function paint(start: number, along: number, offset: number, across: number): void {
    const x = canvas.width / 2 + (offset - across / 2) * acrossScale;
    const y = canvas.height - (start + along) * alongScale;
    ctx.fillRect(x, y, across * acrossScale, along * alongScale);
  }

  /** Paint the same rectangle measured from each threshold. */
  function paintBothEnds(start: number, along: number, offset: number, across: number): void {
    paint(start, along, offset, across);
    paint(length - start - along, along, offset, across);
  }

  // Edge lines. They run the whole length, just inside the pavement edge.
  const edgeOffset = width / 2 - EDGE_LINE_INSET - EDGE_LINE_WIDTH / 2;
  paint(0, length, edgeOffset, EDGE_LINE_WIDTH);
  paint(0, length, -edgeOffset, EDGE_LINE_WIDTH);

  // Dashed centerline.
  const pitch = CENTERLINE_STRIPE + CENTERLINE_GAP;
  for (let s = CENTERLINE_START; s + CENTERLINE_STRIPE <= length - CENTERLINE_START; s += pitch) {
    paint(s, CENTERLINE_STRIPE, 0, CENTERLINE_WIDTH);
  }

  // Threshold bars, six pairs at each end.
  for (let i = 0; i < THRESHOLD_BAR_PAIRS; i += 1) {
    const inner = THRESHOLD_BAR_INNER + i * THRESHOLD_BAR_PITCH;
    const center = inner + THRESHOLD_BAR_WIDTH / 2;
    paintBothEnds(THRESHOLD_BAR_START, THRESHOLD_BAR_LENGTH, center, THRESHOLD_BAR_WIDTH);
    paintBothEnds(THRESHOLD_BAR_START, THRESHOLD_BAR_LENGTH, -center, THRESHOLD_BAR_WIDTH);
  }

  // Aiming point, one wide bar each side of the centerline.
  const aimingCenter = AIMING_POINT_INNER + AIMING_POINT_WIDTH / 2;
  paintBothEnds(AIMING_POINT_DISTANCE, AIMING_POINT_LENGTH, aimingCenter, AIMING_POINT_WIDTH);
  paintBothEnds(AIMING_POINT_DISTANCE, AIMING_POINT_LENGTH, -aimingCenter, AIMING_POINT_WIDTH);

  // Touchdown zone marks.
  const touchdownCenter = TOUCHDOWN_INNER + TOUCHDOWN_WIDTH / 2;
  for (const distance of TOUCHDOWN_DISTANCES) {
    paintBothEnds(distance, TOUCHDOWN_LENGTH, touchdownCenter, TOUCHDOWN_WIDTH);
    paintBothEnds(distance, TOUCHDOWN_LENGTH, -touchdownCenter, TOUCHDOWN_WIDTH);
  }

  return canvas;
}

/**
 * Build a concrete material. The slabs and their joints come from the world
 * position, so every piece of pavement shares one grid and the joints line up
 * where two pieces meet.
 */
function createConcreteMaterial(
  baseHex: number,
  jointHex: number,
  roughness: number,
  markings: CanvasTexture | null,
): MeshStandardNodeMaterial {
  const material = new MeshStandardNodeMaterial();
  material.metalness = 0;
  material.roughness = roughness;

  const slab = positionWorld.xz.mul(1 / SLAB_SIZE);
  const inSlab = fract(slab);

  // Distance to the nearest joint, in slab units, on each axis.
  const toJoint = min(inSlab, inSlab.oneMinus());

  // The derivative width is the size of one pixel in slab units. It widens the
  // transition as the surface moves away, so a joint thinner than a pixel turns
  // into an even gray instead of a flickering line.
  const pixel = fwidth(slab).max(float(1e-6));
  const halfJoint = float(JOINT_WIDTH / (2 * SLAB_SIZE));

  const jointX = smoothstep(halfJoint.sub(pixel.x), halfJoint.add(pixel.x), toJoint.x).oneMinus();
  const jointZ = smoothstep(halfJoint.sub(pixel.y), halfJoint.add(pixel.y), toJoint.y).oneMinus();
  const joint = max(jointX, jointZ);

  // Each slab was poured on its own day, so each one has its own shade.
  const tint = hashLattice(floor(slab)).sub(0.5).mul(SLAB_TINT_STRENGTH);
  const base = mix(color(baseHex), color(jointHex), joint).mul(tint.add(1));

  if (markings === null) {
    material.colorNode = base;
    material.roughnessNode = float(roughness);
    return material;
  }

  const paint = texture(markings, uv()).a;
  material.colorNode = mix(base, color(PAINT_WHITE), paint);
  material.roughnessNode = mix(float(roughness), float(PAINT_ROUGHNESS), paint);
  return material;
}

/**
 * Build one flat rectangle of pavement. The caller gives the center and the
 * size in NED, and src/render/frames.ts maps the center into the render frame.
 */
function createSlabMesh(
  northCenter: number,
  northExtent: number,
  eastCenter: number,
  eastExtent: number,
  height: number,
  material: MeshStandardNodeMaterial,
  name: string,
): Mesh {
  // PlaneGeometry lies in the xy plane. A quarter turn about x lays it flat,
  // and its local +y then points north, which is -z in the render frame.
  const geometry = new PlaneGeometry(eastExtent, northExtent);
  geometry.rotateX(-Math.PI / 2);

  scratchNed.set(northCenter, eastCenter, -height);
  nedToThree(scratchNed, scratchThree);

  const mesh = new Mesh(geometry, material);
  mesh.name = name;
  mesh.position.copy(scratchThree);
  mesh.receiveShadow = config.render.shadowsEnabled;
  mesh.castShadow = false;
  return mesh;
}

export function createRunway(): { group: Object3D; dispose(): void } {
  const length = config.world.runwayLength;
  const width = config.world.runwayWidth;

  const markings = new CanvasTexture(drawMarkings());
  markings.anisotropy = MARKING_ANISOTROPY;

  const runwayMaterial = createConcreteMaterial(
    RUNWAY_CONCRETE,
    RUNWAY_JOINT,
    CONCRETE_ROUGHNESS,
    markings,
  );
  const apronMaterial = createConcreteMaterial(
    APRON_CONCRETE,
    APRON_JOINT,
    APRON_ROUGHNESS,
    null,
  );

  const group = new Group();
  group.name = 'runway';

  // The threshold is the NED origin, so the strip reaches from 0 to the full
  // length and its center sits at half the length north of the origin.
  const strip = createSlabMesh(
    length / 2,
    length,
    0,
    width,
    RUNWAY_HEIGHT,
    runwayMaterial,
    'runway-strip',
  );
  group.add(strip);

  const taxiway = createSlabMesh(
    (TAXIWAY_START + TAXIWAY_END) / 2,
    TAXIWAY_END - TAXIWAY_START,
    TAXIWAY_EAST,
    TAXIWAY_WIDTH,
    APRON_HEIGHT,
    apronMaterial,
    'taxi-strip',
  );
  group.add(taxiway);

  // The link joins the runway edge to the taxi strip near the threshold.
  const linkEastStart = width / 2;
  const linkEastEnd = TAXIWAY_EAST + TAXIWAY_WIDTH / 2;
  const link = createSlabMesh(
    LINK_NORTH,
    TAXIWAY_WIDTH,
    (linkEastStart + linkEastEnd) / 2,
    linkEastEnd - linkEastStart,
    APRON_HEIGHT,
    apronMaterial,
    'taxi-link',
  );
  group.add(link);

  const apron = createSlabMesh(
    (APRON_NORTH_START + APRON_NORTH_END) / 2,
    APRON_NORTH_END - APRON_NORTH_START,
    (APRON_EAST_START + APRON_EAST_END) / 2,
    APRON_EAST_END - APRON_EAST_START,
    APRON_HEIGHT,
    apronMaterial,
    'apron',
  );
  group.add(apron);

  return {
    group,
    dispose(): void {
      for (const child of group.children) {
        if (child instanceof Mesh) child.geometry.dispose();
      }
      runwayMaterial.dispose();
      apronMaterial.dispose();
      markings.dispose();
      group.removeFromParent();
      group.clear();
    },
  };
}
