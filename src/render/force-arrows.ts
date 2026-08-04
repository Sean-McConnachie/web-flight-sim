/**
 * Per element force arrows.
 *
 * This is the instrument that turns an assumed stall into a confirmed one. The
 * aircraft model splits the wing, the tail and the fin into about thirty
 * elements. Each element makes its own force at its own local angle of attack.
 * A number in a panel cannot show which element lost its lift first. An arrow
 * at the element, colored by how close that element runs to its own stall, can.
 *
 * The caller attaches `root` to the render node of the aircraft. Inside that
 * node the axes are the body axes mapped through the frame map of
 * `frames.ts`, so a body vector converts with `nedToThree`. CONVENTIONS
 * section 3.3 states that the body axes map through the same matrix as the NED
 * axes. This file therefore writes no axis swap of its own.
 *
 *
 * ONE DRAW CALL
 *
 * Thirty elements at 60 frames per second is 1800 arrow updates per second. One
 * `InstancedMesh` holds every arrow, so the whole set costs one draw call. The
 * update writes a matrix and a color per instance into buffers that exist for
 * the life of the object. It allocates nothing.
 *
 *
 * THE LENGTH SCALE, AND WHY
 *
 * An aerodynamic force follows the dynamic pressure, which follows the square
 * of the speed. Between 200 km/h and 800 km/h the forces grow by a factor of
 * 16. A fixed number of meters per newton is unreadable over that range: it is
 * either a set of stubs at the low speed or a set of spears at the high speed.
 *
 * The scale here has two parts.
 *
 * 1. A reference force. The object holds a smoothed peak of the largest element
 *    force. The reference rises fast and falls slowly, like the limiter in an
 *    audio chain. Fast attack keeps the longest arrow from saturating when the
 *    load builds. Slow release keeps a sudden loss of force visible: when a
 *    wing stalls, the arrows collapse for about a second before the scale
 *    follows them down. A reference that tracked the peak instantly would
 *    rescale the picture in the same frame and hide the event.
 *
 * 2. A square root. The length is the maximum length times the square root of
 *    the force over the reference. The square root does two things. It undoes
 *    the square in the dynamic pressure, so the picture at 800 km/h looks like
 *    the picture at 200 km/h and not like a different aircraft. It also keeps a
 *    weak element visible: an element at one sixteenth of the peak force gets a
 *    quarter of the length instead of a sixteenth.
 *
 * The smoothing runs per update and not per second, so it depends on the frame
 * rate. That is on purpose. The alternative needs a time step in `update`,
 * which the shared signature does not carry, and the exact time constant of a
 * debug scale does not have to be right.
 *
 *
 * THE COLOR SCALE
 *
 * The color follows the ratio of the local angle of attack to the local stall
 * angle, not the angle itself. A tip element with a washout stalls at a
 * different angle from a root element, and the ratio makes both readable on one
 * scale.
 *
 *   ratio 0.0   deep blue     far below the stall
 *   ratio 0.5   cyan          working
 *   ratio 0.8   yellow        close, the warning band
 *   ratio 1.0   red           at the stall angle
 *   ratio 1.4   dark red      deep stall
 *
 * The ramp interpolates between the stops, so the color is continuous. It
 * crosses into red exactly at the stall angle. The renderer tone maps the
 * image, which shifts every color a little. These four stops stay apart from
 * each other through the tone mapper.
 *
 * This file may use the renderer. CONVENTIONS section 4 allows that under
 * `src/render`. It holds no physics.
 */

import type { Object3D } from 'three/webgpu';
import {
  BufferGeometry,
  Color,
  DoubleSide,
  DynamicDrawUsage,
  Float32BufferAttribute,
  Group,
  InstancedMesh,
  Matrix4,
  MeshBasicNodeMaterial,
  Quaternion,
  Vector3,
} from 'three/webgpu';

import { nedToThree } from './frames';

/** One element of the aerodynamic model, as the physics reports it. */
export interface ElementForceSample {
  /** Body axes, meters from the center of gravity. */
  position: Vector3;
  /** Body axes, newtons. */
  force: Vector3;
  /** Local angle of attack, rad. */
  alpha: number;
  /** Local stall angle, rad, on the positive side. */
  stallAlpha: number;
  /** Name of the element. The caller keeps it for its own labels and logs. */
  name: string;
}

export interface ForceArrows {
  /** Attach this to the render node of the aircraft. */
  root: Object3D;
  /**
   * Redraw the arrows from one set of samples. `scale` multiplies the adaptive
   * length, so 2 makes every arrow twice as long. It defaults to 1.
   */
  update(samples: readonly ElementForceSample[], scale?: number): void;
  visible: boolean;
  /** Draw the sum of the element forces at the center of gravity. */
  showResultant: boolean;
  dispose(): void;
}

/** Any object with red, green and blue parts, from 0 to 1. A `Color` fits. */
export interface Rgb {
  r: number;
  g: number;
  b: number;
}

/** Length of the longest element arrow, in meters. The span is 12.51 m. */
export const ARROW_MAX_LENGTH = 2.5;

/** Length of the longest resultant arrow, in meters. */
export const RESULTANT_MAX_LENGTH = 4;

/** Shaft radius of every arrow, in meters. It stays the same at every length. */
const SHAFT_RADIUS = 0.04;

/** Head radius over shaft radius. */
const HEAD_RADIUS_RATIO = 2.6;

/** Fraction of the arrow that the shaft fills. The head fills the rest. */
const HEAD_START = 0.78;

/** Sides of the shaft and the head. Eight reads as round at this size. */
const ARROW_SEGMENTS = 8;

/** Default number of element arrows the object makes room for. */
const DEFAULT_MAX_ELEMENTS = 32;

/** Blend toward a larger reference force, per update. */
const REFERENCE_ATTACK = 0.35;

/** Blend toward a smaller reference force, per update. */
const REFERENCE_RELEASE = 0.012;

/** Smallest reference force, in newtons. It keeps a division by zero out. */
const REFERENCE_FLOOR = 1;

/** A force below this size draws no arrow, in newtons. */
const FORCE_EPSILON = 1e-6;

/** Color of the resultant arrow. It sits outside the stall ramp on purpose. */
const RESULTANT_COLOR: Rgb = { r: 1, g: 0.25, b: 1 };

// The stall ramp. The three color arrays share the index of the ratio array.
const RAMP_RATIO = [0, 0.5, 0.8, 1, 1.4];
const RAMP_R = [0.15, 0.1, 1, 1, 0.55];
const RAMP_G = [0.45, 0.85, 0.85, 0.15, 0];
const RAMP_B = [1, 0.75, 0.1, 0.1, 0];

/**
 * Map a local angle of attack to a color, and write it into `out`.
 *
 * The function reads the size of the angle, so a large negative angle of attack
 * colors the same as a large positive one. An element that runs far past its
 * negative stall has separated flow as well.
 *
 * The ramp is piecewise linear over the stops above, so the color is continuous
 * in the angle. At the stall angle the color is the red stop exactly.
 */
export function stallColor(alpha: number, stallAlpha: number, out: Rgb): Rgb {
  const limit = stallAlpha > 1e-4 ? stallAlpha : 1e-4;
  const ratio = Math.abs(alpha) / limit;

  const last = RAMP_RATIO.length - 1;
  if (!(ratio > RAMP_RATIO[0])) {
    out.r = RAMP_R[0];
    out.g = RAMP_G[0];
    out.b = RAMP_B[0];
    return out;
  }
  if (ratio >= RAMP_RATIO[last]) {
    out.r = RAMP_R[last];
    out.g = RAMP_G[last];
    out.b = RAMP_B[last];
    return out;
  }
  let i = 0;
  while (i < last && ratio > RAMP_RATIO[i + 1]) i++;
  const lo = RAMP_RATIO[i];
  const hi = RAMP_RATIO[i + 1];
  const t = (ratio - lo) / (hi - lo);
  out.r = RAMP_R[i] + (RAMP_R[i + 1] - RAMP_R[i]) * t;
  out.g = RAMP_G[i] + (RAMP_G[i + 1] - RAMP_G[i]) * t;
  out.b = RAMP_B[i] + (RAMP_B[i + 1] - RAMP_B[i]) * t;
  return out;
}

/**
 * Build one arrow of unit length along +y, with its base at the origin.
 *
 * The instance matrix scales y by the arrow length and scales x and z by the
 * shaft radius, so the arrow grows along its own axis and keeps its width.
 *
 * The material carries no lighting, because a lit arrow would mix the light
 * into the color that carries the stall. The vertex colors hold a gray shade
 * instead. The material multiplies the shade, the instance color and the
 * material color together, so the shade gives the arrow a shape and the
 * instance color keeps its meaning.
 */
function createArrowGeometry(): BufferGeometry {
  const s = ARROW_SEGMENTS;
  const headRadius = HEAD_RADIUS_RATIO;
  const positions = new Float32Array((3 * s + 2) * 3);
  const colors = new Float32Array((3 * s + 2) * 3);

  const write = (index: number, x: number, y: number, z: number, shade: number): void => {
    positions[index * 3] = x;
    positions[index * 3 + 1] = y;
    positions[index * 3 + 2] = z;
    colors[index * 3] = shade;
    colors[index * 3 + 1] = shade;
    colors[index * 3 + 2] = shade;
  };

  for (let i = 0; i < s; i++) {
    const angle = (i / s) * Math.PI * 2;
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    write(i, cos, 0, sin, 0.55); // shaft base ring
    write(s + i, cos, HEAD_START, sin, 0.8); // shaft top ring
    write(2 * s + i, cos * headRadius, HEAD_START, sin * headRadius, 0.9); // head base ring
  }
  write(3 * s, 0, HEAD_START, 0, 0.7); // center of the head base
  write(3 * s + 1, 0, 1, 0, 1); // tip

  const index: number[] = [];
  for (let i = 0; i < s; i++) {
    const next = (i + 1) % s;
    // Shaft side.
    index.push(i, next, s + next);
    index.push(i, s + next, s + i);
    // Head base, seen from below.
    index.push(3 * s, 2 * s + next, 2 * s + i);
    // Head side.
    index.push(2 * s + i, 2 * s + next, 3 * s + 1);
  }

  const geometry = new BufferGeometry();
  geometry.setAttribute('position', new Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new Float32BufferAttribute(colors, 3));
  geometry.setIndex(index);
  return geometry;
}

// Scratch held in module scope. The update allocates nothing.
const UP = new Vector3(0, 1, 0);
/** The center of gravity in body axes. The resultant arrow starts here. */
const ORIGIN = new Vector3(0, 0, 0);
const arrowColor: Rgb = { r: 1, g: 1, b: 1 };
const scratchPosition = new Vector3();
const scratchDirection = new Vector3();
const scratchScale = new Vector3();
const scratchQuaternion = new Quaternion();
const scratchMatrix = new Matrix4();
const scratchColor = new Color();
const resultantForce = new Vector3();

/**
 * Build the arrow set.
 *
 * `maxElements` is the largest number of element arrows the set can draw. The
 * mesh holds one more instance for the resultant arrow at the center of
 * gravity. A sample list longer than `maxElements` loses its tail.
 */
export function createForceArrows(maxElements: number = DEFAULT_MAX_ELEMENTS): ForceArrows {
  const capacity = Math.max(1, Math.floor(maxElements)) + 1;

  const geometry = createArrowGeometry();
  const material = new MeshBasicNodeMaterial({
    color: 0xffffff,
    vertexColors: true,
    side: DoubleSide,
    // The arrows sit within a few meters of the camera on the aircraft. Fog on
    // a debug color would only lie about it.
    fog: false,
  });

  const mesh = new InstancedMesh(geometry, material, capacity);
  mesh.instanceMatrix.setUsage(DynamicDrawUsage);
  // The instance matrices move the arrows far outside the bounding sphere of
  // one unit arrow, so the culler would drop the whole set at some attitudes.
  mesh.frustumCulled = false;
  mesh.renderOrder = 10;
  mesh.count = 0;

  // Allocate the instance color buffer one time, here. The first setColorAt
  // makes it, and a call inside update would allocate on the first frame.
  scratchColor.setRGB(1, 1, 1);
  for (let i = 0; i < capacity; i++) {
    mesh.setColorAt(i, scratchColor);
  }

  const root = new Group();
  root.name = 'force-arrows';
  root.add(mesh);

  /** Smoothed peak of the largest element force, in newtons. */
  let elementReference = REFERENCE_FLOOR;

  /** Smoothed peak of the resultant force, in newtons. */
  let resultantReference = REFERENCE_FLOOR;

  /** Follow a peak with a fast attack and a slow release. */
  function follow(reference: number, peak: number): number {
    const rate = peak > reference ? REFERENCE_ATTACK : REFERENCE_RELEASE;
    const next = reference + (peak - reference) * rate;
    return next > REFERENCE_FLOOR ? next : REFERENCE_FLOOR;
  }

  /**
   * Write one instance. `position` and `force` are in body axes. Returns true
   * when the instance draws.
   */
  function writeArrow(
    index: number,
    position: Vector3,
    force: Vector3,
    magnitude: number,
    reference: number,
    maxLength: number,
    scale: number,
    color: Rgb,
  ): boolean {
    if (!(magnitude > FORCE_EPSILON)) return false;

    let ratio = magnitude / reference;
    if (ratio > 1) ratio = 1;
    const length = maxLength * Math.sqrt(ratio) * scale;
    if (!(length > 0)) return false;

    nedToThree(position, scratchPosition);
    nedToThree(force, scratchDirection).multiplyScalar(1 / magnitude);
    scratchQuaternion.setFromUnitVectors(UP, scratchDirection);
    scratchScale.set(SHAFT_RADIUS, length, SHAFT_RADIUS);
    scratchMatrix.compose(scratchPosition, scratchQuaternion, scratchScale);

    mesh.setMatrixAt(index, scratchMatrix);
    scratchColor.setRGB(color.r, color.g, color.b);
    mesh.setColorAt(index, scratchColor);
    return true;
  }

  const api: ForceArrows = {
    root,
    showResultant: true,

    get visible(): boolean {
      return root.visible;
    },

    set visible(value: boolean) {
      root.visible = value;
    },

    update(samples: readonly ElementForceSample[], scale: number = 1): void {
      if (!root.visible) return;

      const count = Math.min(samples.length, capacity - 1);

      // Pass one. Find the peak element force and the resultant, then move the
      // two references. The lengths of pass two need both.
      let peak = 0;
      resultantForce.set(0, 0, 0);
      for (let i = 0; i < count; i++) {
        const force = samples[i].force;
        resultantForce.add(force);
        const magnitude = force.length();
        if (magnitude > peak) peak = magnitude;
      }
      elementReference = follow(elementReference, peak);
      const resultantMagnitude = resultantForce.length();
      resultantReference = follow(resultantReference, resultantMagnitude);

      // Pass two. Write the instances. An element with no force writes nothing,
      // so the drawn instances stay packed at the front of the buffer.
      let drawn = 0;
      for (let i = 0; i < count; i++) {
        const sample = samples[i];
        stallColor(sample.alpha, sample.stallAlpha, arrowColor);
        const written = writeArrow(
          drawn,
          sample.position,
          sample.force,
          sample.force.length(),
          elementReference,
          ARROW_MAX_LENGTH,
          scale,
          arrowColor,
        );
        if (written) drawn += 1;
      }

      if (api.showResultant) {
        // The resultant acts at the center of gravity, which is the origin of
        // the body frame. CONVENTIONS section 3.1.
        const written = writeArrow(
          drawn,
          ORIGIN,
          resultantForce,
          resultantMagnitude,
          resultantReference,
          RESULTANT_MAX_LENGTH,
          scale,
          RESULTANT_COLOR,
        );
        if (written) drawn += 1;
      }

      mesh.count = drawn;
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor !== null) mesh.instanceColor.needsUpdate = true;
    },

    dispose(): void {
      root.removeFromParent();
      root.remove(mesh);
      mesh.dispose();
      geometry.dispose();
      material.dispose();
    },
  };

  return api;
}
