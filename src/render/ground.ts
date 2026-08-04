/**
 * Ground plane.
 *
 * One flat square of grass, centered on the origin, at `y = 0` in the render
 * frame. The material is written in TSL, so it runs on the WebGPU backend and
 * on the WebGL 2 fallback without a second code path.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * WHY THE COLOR IS NOISE AND NOT A CONSTANT
 *
 * A single green reads as plastic from 6000 m, and it gives the eye nothing to
 * measure speed against during the takeoff run. The color comes from three
 * octaves of value noise in world coordinates:
 *
 *   6000 m   large fields, so the land reads as terrain from the air
 *    420 m   patches, which is the scale a pilot sees on the approach
 *     11 m   grass detail, which is the speed cue near the ground
 *
 * The noise is a function of the world position, so it does not swim when the
 * camera moves and it does not repeat on a visible grid.
 *
 *
 * WHY THE FINE OCTAVE FADES OUT
 *
 * An 11 m feature covers less than one pixel past about 1500 m, so it turns
 * into a sparkling mess. The fine octave fades out with the distance from the
 * camera, from 120 m to 700 m. Distance was chosen over the derivative width
 * because the ground is one flat plane under a camera with a fixed field of
 * view, so distance already carries the pixel footprint, and one distance term
 * also drives the edge fade below. src/world/runway.ts uses the other method,
 * the derivative width, where the feature size is fixed by the concrete slabs.
 *
 *
 * WHY THE FAR EDGE FADES INTO THE HORIZON COLOR
 *
 * The plane stops at 20000 m. The fog is thick there but not complete, so a
 * hard line would show where the grass meets the sky. Near that edge the
 * material drops its diffuse color to zero and lifts its emissive color to the
 * horizon color that src/render/sky.ts computes. The surface then sends out
 * exactly the color of the sky behind it, and the join disappears. The fade
 * also needs distance from the camera, so ground under the aircraft never goes
 * pale, whatever part of the plane it sits on.
 */

import type { Node } from 'three/webgpu';
import { Mesh, MeshStandardNodeMaterial, PlaneGeometry } from 'three/webgpu';
import {
  Fn,
  cameraPosition,
  color,
  float,
  floor,
  fract,
  max,
  mix,
  positionWorld,
  smoothstep,
  uint,
  uniform,
  vec2,
} from 'three/tsl';

import { config } from '@/core/config';
import { horizonColor } from '@/render/sky';

/**
 * Segments per side of the plane.
 *
 * Nothing is displaced, and every shading term is computed per fragment, so two
 * triangles would give the same picture. The mesh still carries 64 segments,
 * which is 8192 triangles and one quad every 625 m. The cost is a few thousand
 * vertices, which no frame budget can measure, and it leaves a grid for a later
 * terrain height pass without a rebuild of the mesh.
 */
const GROUND_SEGMENTS = 64;

/** Feature size of each noise octave, in meters. */
const FIELD_SCALE = 6000;
const PATCH_SCALE = 420;
const DETAIL_SCALE = 11;

/** Distance where the fine octave still has full strength, in meters. */
const DETAIL_FULL_DISTANCE = 120;

/** Distance where the fine octave has gone, in meters. */
const DETAIL_GONE_DISTANCE = 700;

/** Brightness swing of the fine octave, as a fraction of the base color. */
const DETAIL_STRENGTH = 0.34;

/** Fraction of the half side where the edge fade starts and where it completes. */
const EDGE_FADE_START = 0.78;
const EDGE_FADE_END = 0.999;

/** Roughness limits. Grass scatters light, so both ends sit near 0.9. */
const ROUGHNESS_LOW = 0.86;
const ROUGHNESS_HIGH = 0.94;

/**
 * Odd multipliers that spread each lattice axis over the whole 32 bit range.
 * An odd multiplier is invertible in 32 bit arithmetic, so it loses no bit.
 * 0x85ebca6b is a MurmurHash3 constant and 0x27d4eb2d is a Wang hash constant.
 */
const HASH_AXIS_X = 0x27d4eb2d;
const HASH_AXIS_Y = 0x85ebca6b;

/**
 * Multipliers of the lowbias32 finalizer.
 * Source: Wellons, "Prospecting for Hash Functions", 2018, confidence: firm.
 * The finalizer carries every input bit into every output bit.
 */
const HASH_MIX_A = 0x21f0aaad;
const HASH_MIX_B = 0xd35a2d97;

/** Scale that maps a full 32 bit value to the range 0 to 1. It is 2 to the -32. */
const HASH_TO_UNIT = 2.3283064365386963e-10;

/**
 * Hash of a lattice point, from 0 to 1.
 *
 * The input is always the result of `floor`, so it is an exact integer and the
 * same cell always returns the same value.
 *
 *
 * WHY THE HASH USES INTEGER BITS AND NOT A SINE
 *
 * The usual one line hash is `fract(sin(dot(cell, k)) * 43758.5)`. It broke
 * this ground. The detail octave has a cell of 11 m over a world of 40000 m, so
 * the lattice index reaches 1800 and the argument of the sine reaches 500000.
 * A float32 number of that size holds steps of 0.06, and the sine unit of a GPU
 * reduces such an argument with a small number of bits. Two cells that touch
 * then get values that are too close, and the noise goes flat.
 *
 * A test frame 12 km from the origin showed the result. The grass lost its
 * texture and broke into flat steps with straight edges. The same frame at the
 * origin, where the index stays below 60, looked correct. So the fault grows
 * with the distance from the origin, which is the signature of a float limit.
 *
 * Integer arithmetic has no such limit. `int` holds every lattice index of this
 * world exactly, `uint` reads the same bits, and every multiply and every shift
 * below is exact and wraps at 32 bits by the rules of both WGSL and GLSL ES 3.
 * The hash therefore gives the same quality at the origin and at the far corner
 * of the world.
 *
 * Two other answers were rejected. A wrap of the lattice index into a small
 * domain bounds the index, but the ground then repeats and the eye finds that
 * repeat from the air. A rebase of the noise against the camera position also
 * bounds the index, but the pattern then swims under a moving aircraft, which
 * is the worst fault of the three from a cockpit.
 */
const hashLattice = Fn(([cell]: [Node<'vec2'>]) => {
  // A negative index maps to a large unsigned value. No information is lost,
  // because the two forms hold the same bits.
  const keyX = cell.x.toInt().toUint().mul(uint(HASH_AXIS_X));
  const keyY = cell.y.toInt().toUint().mul(uint(HASH_AXIS_Y));

  const h = keyX.bitXor(keyY).toVar();
  h.assign(h.bitXor(h.shiftRight(uint(16))).mul(uint(HASH_MIX_A)));
  h.assign(h.bitXor(h.shiftRight(uint(15))).mul(uint(HASH_MIX_B)));
  h.assign(h.bitXor(h.shiftRight(uint(15))));

  return h.toFloat().mul(HASH_TO_UNIT);
});

/** One octave of value noise, from 0 to 1, with a smooth first derivative. */
const valueNoise = Fn(([point]: [Node<'vec2'>]) => {
  const p = vec2(point).toVar();
  const cell = floor(p);
  const f = fract(p);
  // The smoothstep weights remove the lattice creases that a linear blend shows.
  const w = f.mul(f).mul(float(3).sub(f.mul(2)));

  const a = hashLattice(cell);
  const b = hashLattice(cell.add(vec2(1, 0)));
  const c = hashLattice(cell.add(vec2(0, 1)));
  const d = hashLattice(cell.add(vec2(1, 1)));

  return mix(mix(a, b, w.x), mix(c, d, w.x), w.y);
});

export function createGround(): { mesh: Mesh; dispose(): void } {
  const size = config.world.groundSize;
  const halfSize = size / 2;

  const geometry = new PlaneGeometry(size, size, GROUND_SEGMENTS, GROUND_SEGMENTS);
  // PlaneGeometry lies in the xy plane and faces +z. A quarter turn about x
  // lays it flat and points its face at +y, which is up in the render frame.
  geometry.rotateX(-Math.PI / 2);

  const material = new MeshStandardNodeMaterial();
  material.metalness = 0;
  material.roughness = 0.9;

  const groundXZ = positionWorld.xz;
  const cameraDistance = positionWorld.distance(cameraPosition);

  const field = valueNoise(groundXZ.mul(1 / FIELD_SCALE));
  const patch = valueNoise(groundXZ.mul(1 / PATCH_SCALE));
  const detail = valueNoise(groundXZ.mul(1 / DETAIL_SCALE));

  const detailWeight = smoothstep(
    float(DETAIL_FULL_DISTANCE),
    float(DETAIL_GONE_DISTANCE),
    cameraDistance,
  ).oneMinus();

  // Two greens carry the base, and a third dry tone breaks up the patches.
  const base = mix(color(0x3d5225), color(0x5f7a34), field);
  const patched = mix(base, color(0x86995a), patch.mul(0.55));
  const grass = patched.mul(detail.sub(0.5).mul(DETAIL_STRENGTH).mul(detailWeight).add(1));

  // A square plane has a square edge, so the distance to the edge is the larger
  // of the two axis distances and not the radius.
  const edgeDistance = max(positionWorld.x.abs(), positionWorld.z.abs());
  const edge = smoothstep(
    float(halfSize * EDGE_FADE_START),
    float(halfSize * EDGE_FADE_END),
    edgeDistance,
  ).mul(
    smoothstep(float(config.render.fogNear), float(config.render.fogFar * 0.6), cameraDistance),
  );

  const horizon = uniform(horizonColor);

  material.colorNode = grass.mul(edge.oneMinus());
  material.emissiveNode = horizon.mul(edge);
  material.roughnessNode = mix(float(ROUGHNESS_LOW), float(ROUGHNESS_HIGH), patch);

  const mesh = new Mesh(geometry, material);
  mesh.name = 'ground';
  mesh.receiveShadow = config.render.shadowsEnabled;
  mesh.castShadow = false;
  // The plane is far larger than the view, and its bounding sphere is huge, so
  // a frustum test only costs time.
  mesh.frustumCulled = false;
  mesh.position.set(0, 0, 0);

  return {
    mesh,
    dispose(): void {
      geometry.dispose();
      material.dispose();
      mesh.removeFromParent();
    },
  };
}
