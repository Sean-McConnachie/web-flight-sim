/**
 * GPU particles.
 *
 * One storage buffer set holds every particle. One TSL compute shader moves
 * them. The processor writes ten small uniform blocks per frame and nothing
 * else, so the traffic between the processor and the card does not grow with
 * the particle count.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * 1. WHY A COMPUTE SHADER AND NOT AN INSTANCED MESH
 *
 * src/render/weapons.ts writes one matrix per instance from the processor.
 * `InstancedMesh` then sends the whole matrix array to the card on every frame,
 * whatever `needsUpdate` holds, which is 64 bytes for each instance. At 4480
 * particles that is 287 kB per frame, or 34 MB each second at 120 frames.
 *
 * A storage buffer stays on the card. The compute shader reads it and writes
 * it, and the vertex stage reads the same buffer as an instanced attribute.
 * WebGPU gives such a buffer the STORAGE and the VERTEX use at the same time,
 * so no copy is needed. The frame carries about 700 bytes of uniforms.
 *
 *
 * 2. WHY THE PARTICLES RUN ON THE WebGPU BACKEND ONLY
 *
 * The WebGL 2 backend has no storage buffer. Three.js emulates the compute
 * pass with transform feedback and it emulates an indexed read with a pixel
 * buffer object, and `StorageInstancedBufferAttribute` states in its own
 * header that it works with `WebGPURenderer` alone. src/render/postfx.ts
 * already drops the ambient occlusion on that backend for a related reason.
 * This module builds nothing there. `enabled` then reads false.
 *
 *
 * 3. THE BUFFER LAYOUT
 *
 * Three vectors of four floats hold one particle. WGSL does not pack a vector
 * of three floats inside a storage buffer, so a vector of four costs no more
 * space than a vector of three and it carries one more value for free.
 *
 *   posAge   xyz  position in the render frame, m
 *            w    age from 0 to 1, where 1 is the end of the life
 *   velSize  xyz  velocity in the render frame, m/s
 *            w    radius now, m. A dead particle holds 0 and draws nothing.
 *   colAlpha rgb  color now, linear radiance
 *            w    the alpha the emitter held at the moment of the spawn
 *
 * The alpha of the spawn must stay with the particle. A contrail lives 26 s
 * and the aircraft can leave the cold air in that time. Without the memory the
 * whole trail would fade at once, which is the fault that a trail shows first.
 *
 *
 * 4. THE PARTICLE STAYS IN THE WORLD
 *
 * Every position sits in the render frame of the WORLD, and the draw object
 * stands at the origin with no rotation. A particle therefore does not follow
 * the aircraft after the spawn. This is what makes a contrail read as a trail
 * and not as a rigid rod behind the tailpipe.
 *
 * The spawn point is not the tailpipe of this frame. It is a point between the
 * tailpipe of the last frame and the tailpipe of this frame, chosen by a random
 * number. At 250 m/s and 120 frames each second the tailpipe moves 2.1 m per
 * frame, so without the spread the trail would come out as a row of beads.
 *
 *
 * 6. WHY A PARTICLE KEEPS ITS PLACE IN THE CYCLE
 *
 * Every particle of one effect holds a different starting age, so the block
 * spawns at an even rate and the trail comes out even.
 *
 * The first build lost that spread. A particle that reached the end of its life
 * while its emitter was SHUT had its age held at 1. A contrail emitter is shut
 * below 6000 m, so the whole block of 2048 stood at age 1 by the time the
 * aircraft reached the cold air. The emitter then opened and all 2048 spawned
 * on ONE frame. The trail came out as a single ball of ice 5 m long, and 26 s
 * later the whole ball died and the next ball took its place.
 *
 * The fix is one line. A particle that reaches the end of its life takes the
 * FRACTION of its age and not zero, and it does that whether the emitter is
 * open or shut. Each particle therefore holds its own place in the cycle for
 * as long as the flight lasts. An emitter that opens fills its trail from the
 * aircraft backward over one life, which is what a real trail does.
 *
 *
 * 5. THE RENDER ORDER AND THE REVERSED DEPTH BUFFER
 *
 * `RenderList.sort` sorts the transparent list from far to near and then calls
 * `reverse()` on it when the camera holds a reversed depth buffer. The list
 * then runs from near to far, which is the wrong order for alpha blending, and
 * the render order turns around with it. Read the note in section 6a of
 * docs/CONVENTIONS.md and the same note in src/render/sky.ts.
 *
 * The clouds must draw before the particles, because the clouds are the far
 * layer. `PARTICLE_RENDER_ORDER` is larger than `CLOUD_RENDER_ORDER`, and both
 * change sign with `renderer.reversedDepthBuffer`.
 */

import type { Node, Object3D, WebGPURenderer } from 'three/webgpu';
import {
  Color,
  Group,
  NormalBlending,
  Quaternion,
  Sprite,
  SpriteMaterial,
  SpriteNodeMaterial,
  Vector3,
  Vector4,
  WebGPUCoordinateSystem,
} from 'three/webgpu';
import {
  Fn,
  If,
  float,
  hash,
  instanceIndex,
  instancedArray,
  max,
  mix,
  smoothstep,
  sqrt,
  uint,
  uniform,
  uv,
  vec2,
  vec3,
  vec4,
} from 'three/tsl';

import type { Aircraft } from '@/aircraft/aircraft';
import { me262GearLegs } from '@/physics/gear';
import { CLOUD_RENDER_ORDER } from '@/render/clouds';
import { nedToThree } from '@/render/frames';
import type { Armament } from '@/weapons/armament';
import { muzzleFlash } from '@/weapons/mk108';

// ---------------------------------------------------------------------------
// How many particles each effect owns.
// ---------------------------------------------------------------------------

/** Particles for one tailpipe of hot exhaust. */
const EXHAUST_PER_PIPE = 384;

/** Particles for one tailpipe of contrail. */
const CONTRAIL_PER_PIPE = 1024;

/** Particles for one wheel of dust and touch-down smoke. */
const GROUND_PER_WHEEL = 256;

/** Particles for one burning engine. */
const FIRE_PER_ENGINE = 256;

/** Particles for the smoke of the four cannon. */
const GUN_COUNT = 384;

const EXHAUST_BASE = 0;
const EXHAUST_COUNT = 2 * EXHAUST_PER_PIPE;
const CONTRAIL_BASE = EXHAUST_BASE + EXHAUST_COUNT;
const CONTRAIL_COUNT = 2 * CONTRAIL_PER_PIPE;
const GROUND_BASE = CONTRAIL_BASE + CONTRAIL_COUNT;
const GROUND_COUNT = 3 * GROUND_PER_WHEEL;
const FIRE_BASE = GROUND_BASE + GROUND_COUNT;
const FIRE_COUNT = 2 * FIRE_PER_ENGINE;
const GUN_BASE = FIRE_BASE + FIRE_COUNT;

/** Every particle of every effect. */
export const PARTICLE_COUNT = GUN_BASE + GUN_COUNT;

/** Bytes the three storage buffers hold together. */
export const PARTICLE_BUFFER_BYTES = PARTICLE_COUNT * 3 * 4 * 4;

// ---------------------------------------------------------------------------
// Where each effect starts, in the frame of the render model.
// ---------------------------------------------------------------------------

/**
 * Tailpipe of the left engine, in the model frame of src/render/models/me262.
 *
 * The model puts the nacelle nose at z = -1.85 and the nacelle is 3.80 m long,
 * so the pipe mouth sits at z = 1.95. The span station is 2.05 m and the center
 * line sits 0.53 m under the wing datum. The numbers match the drawn nacelle,
 * so the smoke leaves the pipe that the pilot sees.
 */
const TAILPIPE_LEFT = new Vector3(-2.05, -0.53, 1.95);
const TAILPIPE_RIGHT = new Vector3(2.05, -0.53, 1.95);

/**
 * Muzzle group of the four MK 108, in the model frame.
 *
 * The four barrels stand in a square in the nose, so one point serves them all
 * at the distance a smoke puff is visible from.
 */
const MUZZLE_POINT = new Vector3(0, 0.05, -4.6);

// ---------------------------------------------------------------------------
// The behavior of each effect.
// ---------------------------------------------------------------------------

/**
 * Speed of the gas at the tailpipe of a Jumo 004 B, m/s.
 *
 * The engine takes 21.2 kg/s of air at the maximum rotor speed and it makes
 * 8.8 kN, so the jet leaves at about 8800 / 21.2 = 415 m/s above the flight
 * speed. The visible plume slows far faster than the gas does, so the emitter
 * uses a part of that speed and the drag term below takes the rest.
 * Source: Kay, "Junkers Aircraft and Engines", confidence: derived.
 */
const EXHAUST_SPEED = 90;

/** Altitude where a contrail starts and where it reaches its full strength, m. */
const CONTRAIL_START_ALTITUDE = 6000;
const CONTRAIL_FULL_ALTITUDE = 8200;

/**
 * How long a contrail holds, s.
 *
 * A trail in dry air breaks up in seconds and a trail in moist air holds for
 * minutes. The value is the middle of that band. It sets the length of the
 * trail as well: 26 s at 250 m/s draws 6.5 km of trail.
 */
const CONTRAIL_LIFE = 26;

/** Sink speed of the airframe that gives the largest touch-down puff, m/s. */
const TOUCHDOWN_FULL_SINK = 3;

/** How long the touch-down puff holds after the wheel meets the ground, s. */
const TOUCHDOWN_HOLD = 0.55;

/** Ground speed above which the wheels raise dust at their full strength, m/s. */
const DUST_FULL_SPEED = 40;

/** Largest step the compute shader takes in one frame, s. */
const MAX_STEP = 0.05;

/**
 * Distance an emitter can move in one frame before the move counts as a jump.
 *
 * At 250 m/s and 20 frames each second the tailpipe moves 12.5 m, so any move
 * larger than this is a respawn and not flight.
 */
const MAX_EMITTER_JUMP = 400;

/** Where the frame counter wraps. A float32 value holds an integer to 2^24. */
const FRAME_WRAP = 1000000;

/** Render order of the particles with a NORMAL depth buffer. Read section 5. */
const PARTICLE_RENDER_ORDER = CLOUD_RENDER_ORDER + 1;

// ---------------------------------------------------------------------------
// The public surface.
// ---------------------------------------------------------------------------

export interface ParticleSystem {
  /** Root of the particle draw, in the render frame of the WORLD. */
  root: Object3D;

  /** False when the backend cannot run the compute pass. Read section 2. */
  readonly enabled: boolean;

  /** Particles the system holds. The report and the debug view read it. */
  readonly count: number;

  /**
   * Move every particle by one frame.
   *
   * `position` and `orientation` are the pose the frame draws, in the render
   * frame, which src/main.ts already blends between two physics states.
   */
  update(
    aircraft: Aircraft,
    armament: Armament,
    position: Vector3,
    orientation: Quaternion,
    dt: number,
  ): void;

  dispose(): void;
}

/** One emitter: where it stands, where it stood, and what it throws out. */
interface Emitter {
  /** xyz spawn point of the last frame, w is the gate from 0 to 1. */
  prev: Vector4;
  /** xyz spawn point of this frame, w is the alpha of the spawn. */
  now: Vector4;
  /** xyz spawn velocity in the render frame, w is the random spread, m/s. */
  drift: Vector4;
  nodePrev: Node<'vec4'>;
  nodeNow: Node<'vec4'>;
  nodeDrift: Node<'vec4'>;
}

function createEmitter(): Emitter {
  const prev = new Vector4();
  const now = new Vector4();
  const drift = new Vector4();
  return {
    prev,
    now,
    drift,
    nodePrev: uniform(prev),
    nodeNow: uniform(now),
    nodeDrift: uniform(drift),
  };
}

/** What one kind of particle does over its life. */
interface KindSpec {
  /** First index of the kind inside the shared buffers. */
  base: number;
  /** Particles that belong to one emitter of this kind. */
  perEmitter: number;
  /** The emitters, in the order the index blocks follow. */
  emitters: Emitter[];
  /** Life, s. */
  life: number;
  /** Radius at the spawn and at the end of the life, m. */
  sizeStart: number;
  sizeEnd: number;
  /** How fast the air stops the particle, 1/s. */
  drag: number;
  /** Rise of a hot or a light particle, m/s^2. Gravity is already inside it. */
  lift: number;
  /** Color at the spawn and at the end of the life, linear radiance. */
  colorStart: Color;
  colorEnd: Color;
}

// ---------------------------------------------------------------------------
// Scratch. The frame allocates nothing.
// ---------------------------------------------------------------------------

const scratchPoint = new Vector3();
const scratchWorld = new Vector3();
const scratchAxis = new Vector3();
const scratchVelocity = new Vector3();
const scratchBody = new Vector3();

/** Last spawn point of every emitter, so the frame can join the two points. */
const lastPoint: Vector3[] = [];

/** Time left on the touch-down puff of each wheel, s. */
const touchdownTimer = [0, 0, 0];

/** Whether each wheel touched the ground on the last frame. */
const wheelWasDown = [false, false, false];

/**
 * Where each wheel stands, in the model frame.
 *
 * The leg positions come from src/physics/gear.ts, so the dust rises under the
 * wheel that carries the load. src/render/frames.ts owns the axis swap, and a
 * body position takes the same swap a world position takes. The height is not
 * used, because the dust rises from the ground and not from the axle.
 */
const wheelPoint: Vector3[] = me262GearLegs().map((leg) =>
  nedToThree(leg.position, new Vector3()),
);

/**
 * Report whether this backend can run the compute pass. Read section 2. The
 * test matches src/render/renderer.ts, which reads the coordinate system to
 * name the backend that really started.
 */
function supportsCompute(renderer: WebGPURenderer): boolean {
  return renderer.coordinateSystem === WebGPUCoordinateSystem;
}

export function createParticles(renderer: WebGPURenderer): ParticleSystem {
  const root = new Group();
  root.name = 'particles';

  if (!supportsCompute(renderer)) {
    return {
      root,
      enabled: false,
      count: 0,
      update(): void {
        // The backend cannot hold a storage buffer, so there is nothing to move.
      },
      dispose(): void {
        root.removeFromParent();
      },
    };
  }

  // --- The buffers ------------------------------------------------------
  const posAge = instancedArray(PARTICLE_COUNT, 'vec4');
  const velSize = instancedArray(PARTICLE_COUNT, 'vec4');
  const colAlpha = instancedArray(PARTICLE_COUNT, 'vec4');

  // --- The uniforms the frame writes -----------------------------------
  const stepUniform = uniform(0);
  const frameUniform = uniform(0);

  // --- The emitters -----------------------------------------------------
  // The two tailpipes carry the exhaust, the contrail and an engine fire, so
  // the three effects share one pair of emitters. The gate and the alpha of
  // each effect still differ, so each effect keeps its own pair.
  const exhaust = [createEmitter(), createEmitter()];
  const contrail = [createEmitter(), createEmitter()];
  const fire = [createEmitter(), createEmitter()];
  const dust = [createEmitter(), createEmitter(), createEmitter()];
  const gun = [createEmitter()];
  const allEmitters = [...exhaust, ...contrail, ...fire, ...dust, ...gun];
  for (let i = 0; i < allEmitters.length; i++) lastPoint.push(new Vector3());

  const kinds: KindSpec[] = [
    {
      // The hot exhaust. It leaves fast, it slows fast, and it rises.
      base: EXHAUST_BASE,
      perEmitter: EXHAUST_PER_PIPE,
      emitters: exhaust,
      life: 1.1,
      // The pipe mouth is 0.6 m across, so the first puff is that size. A
      // smaller start left a chain of separate dots behind the aircraft at
      // 215 m/s instead of one ribbon of smoke.
      sizeStart: 0.8,
      sizeEnd: 4.5,
      drag: 2.6,
      lift: 1.4,
      // Soot, and almost no color in it.
      //
      // The first build gave the plume the color of the hot gas, and the
      // aircraft trailed an orange ball across the runway. The second build
      // was still warm enough that the thin trail read as brown against the
      // grass and as pink against a cloud. A Jumo 004 in daylight shows dark
      // grey smoke. The small warm bias that is left only shows where the
      // plume is thick, right at the pipe.
      colorStart: new Color(0.46, 0.43, 0.41),
      colorEnd: new Color(0.32, 0.32, 0.32),
    },
    {
      // The contrail. Ice, so it is white, it hardly moves, and it holds.
      base: CONTRAIL_BASE,
      perEmitter: CONTRAIL_PER_PIPE,
      emitters: contrail,
      life: CONTRAIL_LIFE,
      sizeStart: 1.4,
      sizeEnd: 48,
      drag: 0.55,
      lift: -0.06,
      // Ice, lit by the sun with no cloud over it. The radiance is close to
      // the sunlit top of the deck in src/render/clouds.ts, which is what a
      // contrail reads as next to a cloud in the same picture.
      colorStart: new Color(4.7, 4.8, 5),
      colorEnd: new Color(4.2, 4.35, 4.6),
    },
    {
      // The dust and the touch-down smoke. It falls back to the ground.
      base: GROUND_BASE,
      perEmitter: GROUND_PER_WHEEL,
      emitters: dust,
      life: 1.9,
      sizeStart: 0.28,
      sizeEnd: 3.4,
      drag: 1.5,
      lift: -0.5,
      colorStart: new Color(1.55, 1.46, 1.34),
      colorEnd: new Color(1.15, 1.1, 1.05),
    },
    {
      // A jet pipe fire. The flame is bright and the soot behind it is dark.
      base: FIRE_BASE,
      perEmitter: FIRE_PER_ENGINE,
      emitters: fire,
      life: 2.4,
      sizeStart: 0.35,
      sizeEnd: 7.5,
      drag: 1.1,
      lift: 3.2,
      colorStart: new Color(14, 4.4, 0.8),
      colorEnd: new Color(0.1, 0.095, 0.09),
    },
    {
      // The smoke of the four cannon. It falls behind the nose at once.
      base: GUN_BASE,
      perEmitter: GUN_COUNT,
      emitters: gun,
      life: 0.85,
      sizeStart: 0.2,
      sizeEnd: 2.4,
      drag: 3.2,
      lift: 0.7,
      colorStart: new Color(2.2, 2.05, 1.8),
      colorEnd: new Color(0.9, 0.88, 0.85),
    },
  ];

  /**
   * Build the compute kernel of one kind.
   *
   * Each kind runs over its own block of the shared buffers, so the kernel
   * needs no test on the kind and every thread of one dispatch follows the
   * same path.
   */
  function buildKernel(spec: KindSpec) {
    const total = spec.perEmitter * spec.emitters.length;

    return Fn(() => {
      const local = instanceIndex;
      const index = local.add(uint(spec.base));

      // Which emitter of this kind owns the particle. The blocks are equal in
      // size, so one division gives the answer.
      const which = local.div(uint(spec.perEmitter));

      // Pick the uniforms of that emitter. A chain of selects costs three
      // instructions at most and it needs no branch.
      let prev = spec.emitters[0].nodePrev;
      let now = spec.emitters[0].nodeNow;
      let drift = spec.emitters[0].nodeDrift;
      for (let e = 1; e < spec.emitters.length; e++) {
        const match = which.equal(uint(e));
        prev = match.select(spec.emitters[e].nodePrev, prev);
        now = match.select(spec.emitters[e].nodeNow, now);
        drift = match.select(spec.emitters[e].nodeDrift, drift);
      }

      const a = posAge.element(index).toVar();
      const b = velSize.element(index).toVar();
      const c = colAlpha.element(index).toVar();

      // The age runs from 0 to 1 over the life, so one constant divides it.
      const age = a.w.add(stepUniform.mul(1 / spec.life)).toVar();

      // Four random numbers. `hash` truncates its seed to an unsigned integer,
      // so every seed below must be a whole number. The particle index takes
      // every eighth number and the frame counter fills the gaps, so a value
      // stays exact in a float of 32 bits.
      const key = local.toFloat().mul(8).add(frameUniform);
      const r0 = hash(key);
      const r1 = hash(key.add(1));
      const r2 = hash(key.add(2));
      const r3 = hash(key.add(3));

      If(age.greaterThanEqual(1), () => {
        // The particle keeps the FRACTION of its age, not zero. Read section 6.
        age.assign(age.fract());

        If(prev.w.greaterThan(0.001), () => {
          // The spawn point lies between the emitter of the last frame and the
          // emitter of this frame. Read section 4.
          a.xyz.assign(mix(prev.xyz, now.xyz, r0));
          // A random direction inside a cube is close enough to a sphere once
          // the drag has worked on it, and it costs three hashes.
          const spread = vec3(r1.sub(0.5), r2.sub(0.5), r3.sub(0.5)).mul(drift.w.mul(2));
          b.xyz.assign(drift.xyz.add(spread));
          c.w.assign(now.w);
        }).Else(() => {
          // The emitter is shut, so this turn of the cycle draws nothing. The
          // particle still keeps its place in the cycle.
          c.w.assign(0);
        });
      });

      // A particle that never took an alpha is not in the picture at all.
      const alive = c.w.greaterThan(0.0005);

      // The air slows the particle toward still air, and a hot or a light
      // particle rises. One explicit step is enough at this frame rate.
      const damping = max(float(0), float(1).sub(stepUniform.mul(spec.drag)));
      const velocity = b.xyz.mul(damping).add(vec3(0, spec.lift, 0).mul(stepUniform)).toVar();
      a.xyz.addAssign(velocity.mul(stepUniform));
      b.xyz.assign(velocity);

      // The radius grows with the square root of the age, which is how a puff
      // of gas spreads once its own speed has gone.
      const radius = mix(float(spec.sizeStart), float(spec.sizeEnd), sqrt(age));
      b.w.assign(alive.select(radius, float(0)));

      a.w.assign(age);
      const from = vec3(spec.colorStart.r, spec.colorStart.g, spec.colorStart.b);
      const to = vec3(spec.colorEnd.r, spec.colorEnd.g, spec.colorEnd.b);
      c.xyz.assign(mix(from, to, age.mul(age)));

      posAge.element(index).assign(a);
      velSize.element(index).assign(b);
      colAlpha.element(index).assign(c);
    })().compute(total);
  }

  const kernels = kinds.map(buildKernel);

  // Every particle starts dead and out of sight, with an age spread over the
  // whole life. Without the spread the first frame would spawn the whole block
  // at one moment and the effect would pulse.
  const initKernel = Fn(() => {
    posAge.element(instanceIndex).assign(vec4(0, -20000, 0, hash(instanceIndex.toFloat())));
    velSize.element(instanceIndex).assign(vec4(0, 0, 0, 0));
    colAlpha.element(instanceIndex).assign(vec4(0, 0, 0, 0));
  })().compute(PARTICLE_COUNT);
  void renderer.compute(initKernel);

  // --- The draw ---------------------------------------------------------
  const material = new SpriteNodeMaterial();
  material.transparent = true;
  material.blending = NormalBlending;
  material.depthWrite = false;
  material.depthTest = true;

  const readPos = posAge.toAttribute();
  const readVel = velSize.toAttribute();
  const readCol = colAlpha.toAttribute();

  material.positionNode = readPos.xyz;

  // A puff is never a perfect circle. One hash per particle stretches the quad
  // a little on one axis, which breaks the row of equal disks.
  const stretch = hash(instanceIndex.toFloat().mul(7).add(1));
  material.scaleNode = vec2(
    readVel.w.mul(float(0.82).add(stretch.mul(0.36))),
    readVel.w.mul(float(1.18).sub(stretch.mul(0.36))),
  );

  material.colorNode = readCol.xyz;

  // The fade is the same for every kind, so the compute pass does not have to
  // hold it. The particle opens over the first twelfth of its life and it
  // thins out after that.
  const life = readPos.w;
  const fadeIn = smoothstep(float(0), float(0.08), life);
  const fadeOut = float(1).sub(life).max(0);
  // The edge of the quad must be soft. `smoothstep` needs a rising pair of
  // edges, so the fall comes from `oneMinus` and not from a falling pair.
  const shape = uv().sub(0.5).length().mul(2);
  const soft = smoothstep(float(0.15), float(1), shape).oneMinus();
  material.opacityNode = readCol.w.mul(fadeIn).mul(fadeOut.mul(fadeOut)).mul(soft);

  // `Sprite` names its material `SpriteMaterial` in the TypeScript definitions
  // of three.js, and `SpriteNodeMaterial` reports `isSpriteMaterial` as a
  // boolean where the older class reports the literal true. The two types are
  // therefore not assignable, although the renderer takes either one. The cast
  // states what the renderer already accepts and it writes no code.
  const sprites = new Sprite(material as unknown as SpriteMaterial);
  sprites.count = PARTICLE_COUNT;
  sprites.frustumCulled = false;
  sprites.renderOrder = renderer.reversedDepthBuffer
    ? -PARTICLE_RENDER_ORDER
    : PARTICLE_RENDER_ORDER;
  root.add(sprites);

  let frame = 0;

  /** Write one emitter from a point in the world, a velocity and two scalars. */
  function setEmitter(
    slot: number,
    emitter: Emitter,
    point: Vector3,
    velocity: Vector3,
    spread: number,
    gate: number,
    alpha: number,
  ): void {
    const previous = lastPoint[slot];
    // A shut emitter that opens again must not draw a line from where it stood
    // when it shut. The first frame of a new burst spawns at one point.
    if (emitter.prev.w <= 0.001) previous.copy(point);
    // A respawn puts the aircraft somewhere else in one frame. Without this
    // test the spawn would spread a plume along the whole jump.
    if (previous.distanceToSquared(point) > MAX_EMITTER_JUMP * MAX_EMITTER_JUMP) {
      previous.copy(point);
    }
    emitter.prev.set(previous.x, previous.y, previous.z, gate);
    emitter.now.set(point.x, point.y, point.z, alpha);
    emitter.drift.set(velocity.x, velocity.y, velocity.z, spread);
    previous.copy(point);
  }

  /** Put a point of the model frame into the world, through the frame pose. */
  function toWorld(
    local: Vector3,
    position: Vector3,
    orientation: Quaternion,
    out: Vector3,
  ): Vector3 {
    return out.copy(local).applyQuaternion(orientation).add(position);
  }

  return {
    root,
    enabled: true,
    count: PARTICLE_COUNT,

    update(
      aircraft: Aircraft,
      armament: Armament,
      position: Vector3,
      orientation: Quaternion,
      dt: number,
    ): void {
      const step = Math.min(dt, MAX_STEP);
      stepUniform.value = step;
      frame = (frame + 1) % FRAME_WRAP;
      frameUniform.value = frame;

      const state = aircraft.state;
      const altitude = -state.body.position.z;
      // The velocity of the aircraft, in the render frame. Every plume leaves
      // the aircraft with this speed and the drag term takes it away.
      nedToThree(state.body.velocity, scratchVelocity);

      // The direction the jet blows, which is aft in the model frame.
      scratchAxis.set(0, 0, 1).applyQuaternion(orientation);

      // --- The two tailpipes ---------------------------------------------
      for (let side = 0; side < 2; side++) {
        const engine = state.engines[side];
        const pipe = side === 0 ? TAILPIPE_LEFT : TAILPIPE_RIGHT;
        toWorld(pipe, position, orientation, scratchWorld);

        // The Jumo idles at 3000 rpm and it runs to 8700 rpm. Power is what
        // makes soot, so the smoke follows the rotor speed above the idle.
        const power = Math.max(0, Math.min(1, (engine.rpm - 3000) / 5700));
        const burning =
          engine.state === 'running' ||
          engine.state === 'idle' ||
          engine.state === 'lightOff' ||
          engine.state === 'stall' ||
          engine.state === 'fire';

        // The exhaust. It leaves the pipe aft and it carries the aircraft with
        // it. A surge throws a much thicker cloud out of the pipe.
        const surge = engine.state === 'stall' ? 1 : 0;
        const sooty = burning ? 0.07 + 0.5 * power * power + 0.45 * surge : 0;
        scratchPoint
          .copy(scratchVelocity)
          .addScaledVector(scratchAxis, EXHAUST_SPEED * (0.35 + 0.65 * power));
        setEmitter(
          side,
          exhaust[side],
          scratchWorld,
          scratchPoint,
          6 + 8 * power,
          burning ? 1 : 0,
          Math.min(0.6, sooty),
        );

        // The contrail. The air must be cold and the engine must burn. The
        // trail forms a little behind the pipe, where the gas has cooled.
        const cold =
          (altitude - CONTRAIL_START_ALTITUDE) /
          (CONTRAIL_FULL_ALTITUDE - CONTRAIL_START_ALTITUDE);
        const humid = Math.max(0, Math.min(1, cold));
        const trail = burning ? humid * (0.25 + 0.75 * power) : 0;
        scratchPoint.copy(scratchWorld).addScaledVector(scratchAxis, 6);
        // A trail hardly moves after it forms. It only keeps a little of the
        // wake of the wing, which pushes it down and out.
        scratchBody.copy(scratchVelocity).multiplyScalar(0.06);
        setEmitter(
          2 + side,
          contrail[side],
          scratchPoint,
          scratchBody,
          1.4,
          trail > 0.02 ? 1 : 0,
          0.78 * trail,
        );

        // A jet pipe fire. src/aircraft/me262/engine.ts raises the state.
        const onFire = engine.state === 'fire';
        scratchPoint.copy(scratchVelocity).addScaledVector(scratchAxis, 26);
        setEmitter(
          4 + side,
          fire[side],
          scratchWorld,
          scratchPoint,
          4,
          onFire ? 1 : 0,
          onFire ? 0.9 : 0,
        );
      }

      // --- The three wheels ----------------------------------------------
      const legs = state.gear.legs;
      const groundSpeed = Math.hypot(state.body.velocity.x, state.body.velocity.y);
      for (let i = 0; i < 3; i++) {
        const leg = i < legs.length ? legs[i] : null;
        const onGround = leg !== null && leg.onGround;

        // A wheel that meets the ground throws a puff whose size follows the
        // sink rate. The timer holds the puff for a moment after the touch.
        if (onGround && !wheelWasDown[i]) {
          // The sink rate of the AIRFRAME at the moment of the touch, not the
          // stroke rate of the strut. The strut has not moved yet on the frame
          // the wheel first meets the ground, so the stroke rate reads near
          // zero there and the puff would never grow with a hard landing.
          // World z points down, so a descent is a positive z velocity.
          const sink = Math.max(0, state.body.velocity.z);
          touchdownTimer[i] = TOUCHDOWN_HOLD * Math.min(1, sink / TOUCHDOWN_FULL_SINK);
        }
        wheelWasDown[i] = onGround;
        touchdownTimer[i] = Math.max(0, touchdownTimer[i] - step);

        if (!onGround || leg === null) {
          setEmitter(6 + i, dust[i], scratchWorld, scratchPoint, 0, 0, 0);
          continue;
        }

        // A tire that slips throws far more than a tire that rolls. The slip
        // ratio is -1 for a locked wheel, so its size is what matters.
        const slip = Math.min(1, Math.abs(leg.slipRatio) + Math.abs(leg.slipAngle));
        const roll = Math.min(1, groundSpeed / DUST_FULL_SPEED);
        const puff = touchdownTimer[i] / TOUCHDOWN_HOLD;
        const strength = Math.min(1, 0.35 * roll + 0.9 * slip + 1.2 * puff);

        // The contact patch stands under the leg. The ground is flat, so the
        // height of the point is the height of the dust and not the height of
        // the axle. Only the two horizontal parts of the leg matter.
        toWorld(wheelPoint[i], position, orientation, scratchWorld);
        scratchWorld.y = 0.12;

        // The dust leaves the tire backward and upward.
        scratchPoint.copy(scratchVelocity).multiplyScalar(-0.14);
        scratchPoint.y += 1.6 + 3.5 * puff;
        setEmitter(
          6 + i,
          dust[i],
          scratchWorld,
          scratchPoint,
          1.2 + 2.5 * strength,
          strength > 0.02 ? 1 : 0,
          Math.min(0.6, 0.7 * strength),
        );
      }

      // --- The four cannon -------------------------------------------------
      let flash = 0;
      for (const barrel of armament.battery.guns) {
        const bright = muzzleFlash(barrel);
        if (bright > flash) flash = bright;
      }
      toWorld(MUZZLE_POINT, position, orientation, scratchWorld);
      // The smoke leaves the muzzle forward but the aircraft outruns it at
      // once, so it holds only a fifth of the speed of the aircraft.
      scratchPoint.copy(scratchVelocity).multiplyScalar(0.2);
      setEmitter(
        9,
        gun[0],
        scratchWorld,
        scratchPoint,
        3.5,
        flash > 0.02 ? 1 : 0,
        Math.min(0.5, 0.6 * flash),
      );

      void renderer.compute(kernels);
    },

    dispose(): void {
      material.dispose();
      root.removeFromParent();
      root.clear();
    },
  };
}
