/**
 * Instanced draw with distance based level of detail.
 *
 * One `InstancedGroup` draws many copies of one model. Each copy is an
 * instance. The group holds one `InstancedMesh` for each level of detail, and
 * every instance sits in exactly one of them, or in none when it is too far
 * away to see. So the whole group costs at most one draw call per level, not
 * one draw call per copy.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * WHY THE MATRICES ARE NOT REBUILT EVERY FRAME
 *
 * A world with 4000 trees holds 4000 matrices. A rebuild of all of them each
 * frame would cost 4000 matrix compositions and would send 256 kB to the GPU 60
 * times a second, for a picture that does not change. The group builds every
 * matrix one time, at build time, and keeps them in one array. After that a
 * frame only moves an instance from one level to another, and only when the
 * instance crosses a distance boundary. A frame where no instance crosses a
 * boundary sends nothing to the GPU.
 *
 * The move itself is a swap and a pop. Each level keeps its live instances in
 * the first `count` slots of its buffer. To remove an instance the group copies
 * the last live instance into the free slot and lowers the count. To add one it
 * writes into the slot just past the end and raises the count. Both cost a
 * fixed amount of work, whatever the instance count is.
 *
 *
 * HOW THE LEVEL BOUNDARY AVOIDS FLICKER
 *
 * An instance that sits exactly on a boundary would change level on every
 * frame, because the camera never holds perfectly still. Each frame would then
 * write two levels and the model would visibly pop back and forth.
 *
 * The group uses two boundaries instead of one. An instance moves to the
 * coarser level only after its distance passes the boundary by
 * `LOD_HYSTERESIS`, and it moves back to the finer level only after the
 * distance falls the same fraction below. The gap between the two boundaries is
 * the hysteresis band. An instance inside the band keeps the level it already
 * has, so it must travel the full width of the band before it can change back.
 * At a boundary of 250 m the band is 30 m wide, which no camera shake can
 * cross.
 *
 *
 * WHY FRUSTUM CULLING IS OFF
 *
 * A level holds instances that are spread over the whole world, so its bounding
 * sphere covers the whole world and the frustum test never rejects it. The near
 * levels are worse: their instances form a shell around the camera, so their
 * bounding sphere always contains the camera and always meets the frustum. The
 * test would cost time and would reject nothing. The distance test above
 * already removes every instance that is too far to see.
 */

import type { BufferGeometry, Material, Object3D } from 'three/webgpu';
import { Color, Group, InstancedMesh, Matrix4, Vector3 } from 'three/webgpu';

import { config } from '@/core/config';

/** One copy of a model, in the render frame. */
export interface InstancePlacement {
  /** Position of the model origin, in meters. */
  position: Vector3;

  /** Turn about the render y axis, in radians. */
  rotationY: number;

  /** Uniform scale. A value of 1 keeps the model at its built size. */
  scale: number;
}

/** One level of detail of a model. */
export interface LodLevel {
  /** Geometry of this level. The caller owns it and must dispose it. */
  geometry: BufferGeometry;

  /** Largest camera distance that still uses this level, in meters. */
  distance: number;
}

/**
 * Fill `out` with the color multiplier of one instance.
 *
 * The value multiplies the color of the material, so 1 in every channel leaves
 * the model unchanged. The values are plain linear multipliers, not colors, so
 * the function must write the fields directly and must not use `setHex`.
 *
 * `index` is the index of the instance in the placement array. It never
 * changes, so the color of one instance never changes, even after the instance
 * moves between levels of detail.
 */
export type InstanceTint = (index: number, out: Color) => void;

export interface InstancedGroup {
  /** Root of the level meshes, in the render frame. */
  root: Object3D;

  /** Number of draw calls the last `update` left for the color pass. */
  readonly drawCalls: number;

  /** Number of instances the group holds. */
  readonly instanceCount: number;

  /** Sort the instances into levels. `cameraPosition` is in the render frame. */
  update(cameraPosition: Vector3): void;

  /** Free the level meshes. The caller still owns the geometry and material. */
  dispose(): void;
}

/**
 * Half width of the hysteresis band, as a fraction of the boundary distance.
 * Six percent of 250 m is 15 m on each side. A camera must move 30 m to send an
 * instance back to the level it just left.
 */
const LOD_HYSTERESIS = 0.06;

/** Scratch objects. The module allocates them one time. */
const scratchMatrix = new Matrix4();
const scratchScale = new Vector3();
const scratchColor = new Color();

/** One level of detail, with the bookkeeping that the swap and pop needs. */
interface LevelState {
  mesh: InstancedMesh;

  /** Square of the distance where an instance leaves for the coarser level. */
  farSq: number;

  /** Square of the distance where an instance leaves for the finer level. */
  nearSq: number;

  /** Square of the plain boundary, with no hysteresis. The first pass uses it. */
  exactSq: number;

  /** Instance that sits in each slot, for the first `count` slots. */
  instanceOfSlot: Int32Array;

  /** Slot of each instance, or -1 when the instance is not in this level. */
  slotOfInstance: Int32Array;

  /** Number of live slots. */
  count: number;

  /** True when a slot changed since the last upload. */
  dirty: boolean;
}

/**
 * Build a group that draws every placement with the level of detail that suits
 * its distance from the camera.
 *
 * `levels` must hold at least one level. The function sorts them by distance,
 * so the caller may give them in any order. An instance farther than the
 * largest distance is not drawn at all.
 *
 * `tint` is optional. Without it every instance takes the plain material color.
 */
export function createInstancedGroup(
  levels: LodLevel[],
  material: Material,
  placements: readonly InstancePlacement[],
  tint?: InstanceTint,
): InstancedGroup {
  if (levels.length === 0) {
    throw new RangeError('An instanced group needs at least one level of detail.');
  }

  const sorted = levels.slice().sort((a, b) => a.distance - b.distance);
  const instanceCount = placements.length;
  const levelCount = sorted.length;

  const root = new Group();
  root.name = 'instanced-group';
  // The group sits at the render origin, so an instance matrix is already a
  // world matrix. src/render/models/trees.ts reads the vertex position in the
  // vertex stage and relies on that.
  root.position.set(0, 0, 0);

  // Every instance matrix, built one time. Slot i holds the matrix of the
  // instance with index i, whatever level that instance sits in.
  const matrices = new Float32Array(instanceCount * 16);
  for (let i = 0; i < instanceCount; i += 1) {
    const placement = placements[i];
    scratchMatrix.makeRotationY(placement.rotationY);
    scratchMatrix.scale(scratchScale.setScalar(placement.scale));
    scratchMatrix.setPosition(placement.position);
    scratchMatrix.toArray(matrices, i * 16);
  }

  // Every instance color multiplier, built one time, for the same reason.
  const tints = new Float32Array(instanceCount * 3);
  for (let i = 0; i < instanceCount; i += 1) {
    scratchColor.setRGB(1, 1, 1);
    if (tint !== undefined) tint(i, scratchColor);
    tints[i * 3 + 0] = scratchColor.r;
    tints[i * 3 + 1] = scratchColor.g;
    tints[i * 3 + 2] = scratchColor.b;
  }

  const states: LevelState[] = [];
  for (let k = 0; k < levelCount; k += 1) {
    const level = sorted[k];
    const mesh = new InstancedMesh(level.geometry, material, Math.max(instanceCount, 1));
    mesh.name = `lod-${k}`;
    mesh.count = 0;
    mesh.frustumCulled = false;

    // A level casts a shadow only when its near boundary is inside the shadow
    // volume. A level that starts beyond the volume can never write a texel, so
    // its instances would only cost time in the shadow pass.
    const nearDistance = k === 0 ? 0 : sorted[k - 1].distance;
    mesh.castShadow = config.render.shadowsEnabled && nearDistance <= config.render.shadowDistance;
    mesh.receiveShadow = config.render.shadowsEnabled;

    // Allocate the color buffer now. The material reads the buffer only when it
    // exists at the first build of the shader, and at that moment no instance
    // has a level yet.
    scratchColor.setRGB(1, 1, 1);
    for (let slot = 0; slot < instanceCount; slot += 1) mesh.setColorAt(slot, scratchColor);

    const slotOfInstance = new Int32Array(instanceCount);
    slotOfInstance.fill(-1);

    states.push({
      mesh,
      farSq: (level.distance * (1 + LOD_HYSTERESIS)) ** 2,
      nearSq: (level.distance * (1 - LOD_HYSTERESIS)) ** 2,
      exactSq: level.distance ** 2,
      instanceOfSlot: new Int32Array(instanceCount),
      slotOfInstance,
      count: 0,
      dirty: false,
    });

    root.add(mesh);
  }

  /** Level of each instance. `levelCount` means the instance is too far away. */
  const levelOfInstance = new Int32Array(instanceCount);
  levelOfInstance.fill(levelCount);

  let firstPass = true;
  let drawCalls = 0;

  /** Copy the matrix and the color of one instance into one slot of one level. */
  function writeSlot(state: LevelState, slot: number, instance: number): void {
    scratchMatrix.fromArray(matrices, instance * 16);
    state.mesh.setMatrixAt(slot, scratchMatrix);
    scratchColor.setRGB(tints[instance * 3], tints[instance * 3 + 1], tints[instance * 3 + 2]);
    state.mesh.setColorAt(slot, scratchColor);
    state.instanceOfSlot[slot] = instance;
    state.slotOfInstance[instance] = slot;
  }

  function addToLevel(level: number, instance: number): void {
    const state = states[level];
    writeSlot(state, state.count, instance);
    state.count += 1;
    state.dirty = true;
  }

  function removeFromLevel(level: number, instance: number): void {
    const state = states[level];
    const slot = state.slotOfInstance[instance];
    state.slotOfInstance[instance] = -1;
    state.count -= 1;
    // Fill the hole with the last live instance, so the live slots stay packed.
    if (slot !== state.count) writeSlot(state, slot, state.instanceOfSlot[state.count]);
    state.dirty = true;
  }

  function update(cameraPosition: Vector3): void {
    const cx = cameraPosition.x;
    const cy = cameraPosition.y;
    const cz = cameraPosition.z;

    for (let i = 0; i < instanceCount; i += 1) {
      const position = placements[i].position;
      const dx = position.x - cx;
      const dy = position.y - cy;
      const dz = position.z - cz;
      const distanceSq = dx * dx + dy * dy + dz * dz;

      const current = levelOfInstance[i];
      let next = current;

      if (firstPass) {
        // No instance has a level yet, so there is nothing to keep. Use the
        // plain boundary and let the hysteresis start from that state.
        next = 0;
        while (next < levelCount && distanceSq > states[next].exactSq) next += 1;
      } else {
        while (next < levelCount && distanceSq > states[next].farSq) next += 1;
        while (next > 0 && distanceSq < states[next - 1].nearSq) next -= 1;
      }

      if (next === current) continue;
      if (current < levelCount) removeFromLevel(current, i);
      if (next < levelCount) addToLevel(next, i);
      levelOfInstance[i] = next;
    }

    firstPass = false;

    drawCalls = 0;
    for (let k = 0; k < levelCount; k += 1) {
      const state = states[k];
      state.mesh.count = state.count;
      if (state.count > 0) drawCalls += 1;
      if (!state.dirty) continue;
      state.dirty = false;
      state.mesh.instanceMatrix.needsUpdate = true;
      if (state.mesh.instanceColor !== null) state.mesh.instanceColor.needsUpdate = true;
    }
  }

  return {
    root,

    get drawCalls(): number {
      return drawCalls;
    },

    get instanceCount(): number {
      return instanceCount;
    },

    update,

    dispose(): void {
      for (const state of states) {
        state.mesh.dispose();
        state.mesh.removeFromParent();
      }
      root.removeFromParent();
      root.clear();
    },
  };
}
