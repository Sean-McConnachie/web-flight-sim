/**
 * Exterior model of the Messerschmitt Me 262 A-1a.
 *
 * The module builds the model from primitives and from lofted surfaces. It
 * holds no physics and no animation. Bead b19 drives the control surfaces
 * through the named pivots that `createMe262Model` returns.
 *
 * This module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4.
 *
 *
 * 1. ORIENTATION
 *
 * The model lives in the render frame. src/render/frames.ts maps the physics
 * body frame into the render frame, and that map gives the orientation below.
 *
 *   body +x, out of the nose      ->  render -z
 *   body +y, out of the right wing ->  render +x
 *   body +z, through the floor    ->  render -y
 *
 * So the model points its nose along -z, puts the starboard wing along +x, and
 * puts up along +y. The belly faces -y. A model built this way needs no extra
 * turn between the physics and the renderer.
 *
 *
 * 2. WHERE THE ORIGIN SITS
 *
 * The origin of the model sits at the center of gravity. The physics turns the
 * aircraft about the center of gravity, so a model built about the nose or
 * about the centroid swims across the screen during a roll.
 *
 * The center of gravity sits at 25 percent of the mean aerodynamic chord.
 * The wing is a straight taper with a root chord of 2.40 m, a tip chord of
 * 1.07 m, and a span of 12.51 m. For that plan form:
 *
 *   taper ratio  = 1.07 / 2.40 = 0.4458
 *   mean chord   = (2/3) * 2.40 * (1 + L + L^2) / (1 + L) = 1.820 m
 *   span station = (b/6) * (1 + 2L) / (1 + L)             = 2.728 m
 *
 * The root quarter chord sits 4.992 m aft of the nose tip, so the root leading
 * edge sits 4.392 m aft of it. The LEADING EDGE sweeps the published 18.5 deg
 * and the QUARTER CHORD line therefore sweeps 15.72 deg, because the chord
 * falls from 2.40 m to 1.07 m over the 6.255 m semi span:
 *
 *   tan(sweep at c/4) = tan(18.5 deg) - 0.25 * (2.40 - 1.07) / 6.255
 *                     = 0.33460 - 0.05316 = 0.28144,  that is 15.72 deg.
 *
 * At the span station of the mean chord the quarter chord then sits
 * 4.992 + 0.28144 * 2.728 = 5.760 m aft of the nose tip, and the quarter chord
 * line passes through 25 percent of the mean chord.
 *
 * THE TABLE CALLED 18.5 DEG A QUARTER CHORD ANGLE UNTIL BEAD b65, AND IT IS A
 * LEADING EDGE ANGLE. This file drew the old angle from a root at 4.850 m until
 * bead b75. See the note on the sweep in docs/CONVENTIONS.md section 8.
 *
 * CG_OFFSET_FROM_NOSE is therefore 5.76 m aft of the nose tip. That is 54.3
 * percent of the 10.60 m length. A jet that carries its engines under the wing
 * and a long gun bay in the nose sits near that value.
 *
 *
 * 3. HOW THE FUSELAGE CROSS SECTION IS MADE
 *
 * The Me 262 fuselage is a rounded triangle, widest at the bottom and narrow
 * at the top. A plain cylinder reads as the wrong aircraft.
 *
 * The method: a custom lofted surface. `roundedTriangleProfile` builds one
 * normalized cross section as the smooth intersection of four half planes.
 * Each half plane gives one flat side. A p-norm blend with p = 5 replaces the
 * hard minimum, which rounds every corner by a fixed amount. The result is a
 * closed loop of points with a wide flat bottom, two sloped sides, and a small
 * flat top.
 *
 * `loft` then sweeps that one profile along the length of the fuselage. Each
 * station scales the profile by a half width, a half height, and a vertical
 * offset. A Catmull-Rom pass adds two rings between every pair of hand written
 * stations, so the silhouette holds no facets.
 *
 *
 * 4. PIVOT SIGN CONVENTIONS. THIS IS THE CONTRACT WITH BEAD b19
 *
 * Every pivot is an `Object3D` with an identity local transform. Its parent
 * carries the position and the base orientation, so bead b19 may write
 * `pivot.rotation.x = angle` without loss of the base frame.
 *
 * EVERY pivot turns about its own LOCAL +x AXIS. No pivot uses y or z.
 * Set only `rotation.x`. Leave `rotation.y` and `rotation.z` at zero.
 *
 * The local frames follow. "aft" means the model +z direction, "up" means the
 * model +y direction, and "starboard" means the model +x direction.
 *
 *   aileronLeft, aileronRight, flapLeft, flapRight, elevatorLeft,
 *   elevatorRight
 *     local +x runs along the hinge line and points toward starboard.
 *     local +y is up. local +z is aft.
 *     POSITIVE angle moves the trailing edge DOWN. Zero is faired.
 *     This holds on both wings and on both tail halves.
 *     A right roll command therefore needs aileronRight negative and
 *     aileronLeft positive.
 *     Flap range: 0 to about +0.87 rad. Slat and flap limits sit in
 *     ME262_POSE.
 *
 *   rudder
 *     local +x runs down the hinge line, from the fin tip toward the root.
 *     local +y points toward starboard. local +z is aft and down.
 *     POSITIVE angle moves the trailing edge to the LEFT, toward port.
 *     Zero is faired.
 *
 *   slatLeft, slatRight
 *     local +x runs along the hinge line and points toward PORT.
 *     local +y is up. local +z is forward.
 *     POSITIVE angle moves the slat FORWARD and DOWN, which is the deployed
 *     position. Zero is retracted and flush.
 *
 *   gearNose
 *     local +x points toward starboard. local +y is up. local +z is aft.
 *     POSITIVE angle swings the leg FORWARD, into the nose bay.
 *     Zero is down and locked. ME262_POSE.gearNoseRetracted is up.
 *
 *   gearLeft, gearRight
 *     local +x runs along the trunnion. On the right leg it points forward.
 *     On the left leg it points aft. local +y is up in both cases.
 *     POSITIVE angle swings the leg INBOARD, toward the wing root, on both
 *     sides. Zero is down and locked. ME262_POSE.gearMainRetracted is up.
 *
 *   gearDoorNose, gearDoorLeft, gearDoorRight
 *     local +x runs along the door hinge. local +y is up. The door body always
 *     lies on the local +z side of the hinge.
 *     POSITIVE angle swings the door DOWN and away from the skin, which opens
 *     it. Zero is closed and flush. ME262_POSE holds the open angles.
 *
 *   wheelNose, wheelLeft, wheelRight
 *     local +x runs along the axle and points toward PORT. local +y is up.
 *     local +z is forward.
 *     POSITIVE angle rolls the wheel FORWARD. The angle is free to run past
 *     2 pi. Each wheel pivot is a child of its leg pivot, so a retract carries
 *     the wheel with it.
 *
 *   canopy
 *     local +x points forward, along the starboard sill. local +y is up.
 *     local +z points toward starboard.
 *     POSITIVE angle lifts the port edge and swings the hood over to
 *     starboard, which is the side hinged opening of the A-1a.
 *     Zero is closed. ME262_POSE.canopyOpen is the open angle.
 *
 * `reset` writes zero to every pivot, with no exception. That gives the state
 * in which the aircraft spawns on the runway: every aerodynamic control faired,
 * flaps up, slats retracted, gear down and locked, gear doors closed and flush
 * with the skin, and the hood closed. A gear cycle opens the doors by driving
 * each door pivot toward ME262_POSE.gearDoorOpen and back to zero.
 *
 *
 * 4a. THE SLIDING GEAR LEG
 *
 * A pivot cannot show an oleo stroke, because a pivot only turns. Each leg
 * therefore carries one more part: a SLIDER. The slider is a plain Object3D
 * between the leg pivot and the moving half of the leg, and it holds a
 * position instead of a turn.
 *
 * Each leg splits into two members. The outer member is the gas cylinder. It
 * hangs from the leg pivot and it never moves. The inner member is the piston.
 * It hangs from the slider, together with the torque link and the wheel pivot,
 * so the axle always stays on the end of the piston. The cylinder is wider
 * than the piston and it is long enough to hide the head of the piston over
 * the whole stroke.
 *
 * `setGearCompression` drives the three sliders. Its argument is the STRUT
 * STROKE of GearLegState.compression of src/physics/gear.ts, in meters, and
 * zero is full extension. The slider runs along the leg axis, and the distance
 * is scaled so that the axle rises by exactly one meter for each meter of
 * stroke. The strut of the model rakes a few degrees and the strut of the
 * physics stands on the body z axis, so only that scale keeps the wheel of the
 * model on the wheel of the physics.
 *
 * The model is DRAWN at the static stroke of 0.154 m, which is where the
 * aircraft stands at rest at the design mass. `reset` therefore writes that
 * value and not zero, because the aircraft spawns parked and sitting on its
 * gear. Every wheel then touches the ground line at y = -1.33.
 *
 *
 * 5. REFERENCE DATA
 *
 * Every dimension comes from docs/CONVENTIONS.md section 8, confidence firm,
 * except the values marked as an estimate below.
 */

import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';
import type { Material } from 'three/webgpu';
import {
  BoxGeometry,
  BufferGeometry,
  CylinderGeometry,
  Float32BufferAttribute,
  Matrix4,
  Mesh,
  Object3D,
  Quaternion,
  Vector2,
  Vector3,
} from 'three/webgpu';

import { clamp, lerp } from '@/math/tables';
import { DEG } from '@/math/units';

import type { ModelMaterialSet } from './materials';
import {
  bakeModelSpaceAttributes,
  createModelMaterialSet,
  disposeModelMaterialSet,
} from './materials';

// ---------------------------------------------------------------------------
// Public contract
// ---------------------------------------------------------------------------

/** Named hinge points of the model. Read section 4 above before you use one. */
export interface Me262Pivots {
  aileronLeft: Object3D;
  aileronRight: Object3D;
  elevatorLeft: Object3D;
  elevatorRight: Object3D;
  rudder: Object3D;
  flapLeft: Object3D;
  flapRight: Object3D;
  slatLeft: Object3D;
  slatRight: Object3D;
  gearNose: Object3D;
  gearLeft: Object3D;
  gearRight: Object3D;
  gearDoorNose: Object3D;
  gearDoorLeft: Object3D;
  gearDoorRight: Object3D;
  wheelNose: Object3D;
  wheelLeft: Object3D;
  wheelRight: Object3D;
  canopy: Object3D;
}

export interface Me262Model {
  root: Object3D;
  pivots: Me262Pivots;
  /**
   * Sets the oleo stroke of the three legs, in meters. Zero is full extension
   * and ME262_GEAR_TRAVEL is the hard stop. Each value clamps to that band.
   * Read section 4a of the module comment.
   */
  setGearCompression(nose: number, left: number, right: number): void;
  /** Sets every control to its neutral position, and the gear to its rest. */
  reset(): void;
  dispose(): void;
}

/**
 * Distance from the nose tip to the center of gravity, in meters. The model
 * origin sits at the center of gravity, so the nose tip sits at
 * `z = -CG_OFFSET_FROM_NOSE` and the tail sits at `z = LENGTH - this value`.
 */
export const CG_OFFSET_FROM_NOSE = 5.76;

/**
 * End stops and preset angles, in radians. Bead b19 clamps to these values.
 * Every angle turns the matching pivot about its local +x axis.
 */
export const ME262_POSE = {
  /** Nose leg fully forward and up. */
  gearNoseRetracted: 1.62,
  /** Main leg fully inboard and up. */
  gearMainRetracted: 1.5,
  /** Nose bay door fully open. */
  gearDoorNoseOpen: 1.5,
  /** Main bay door fully open. */
  gearDoorOpen: 1.55,
  /** Flap at the take off setting, about 20 deg. */
  flapTakeoff: 0.35,
  /** Flap at the landing setting, about 50 deg. */
  flapLanding: 0.87,
  /** Slat fully out. The slat of the Me 262 opens by air load alone. */
  slatDeployed: 0.35,
  /** Aileron travel limit, up and down. */
  aileronLimit: 0.35,
  /** Elevator travel limit, up and down. */
  elevatorLimit: 0.44,
  /** Rudder travel limit, left and right. */
  rudderLimit: 0.44,
  /** Canopy fully open. */
  canopyOpen: 1.15,
} as const;

// ---------------------------------------------------------------------------
// Airframe dimensions
// ---------------------------------------------------------------------------

/** Overall length, in m. Source: CONVENTIONS section 8, confidence firm. */
const LENGTH = 10.6;

/** Span, in m. Source: CONVENTIONS section 8, confidence firm. */
const SPAN = 12.51;

const HALF_SPAN = SPAN / 2;

/**
 * Root chord and tip chord, in m. Estimated from the firm span of 12.51 m and
 * the firm area of 21.7 m2. A straight taper wing holds
 * S = (b / 2) * (cr + ct), so cr + ct = 3.469 m. The taper ratio of 0.446
 * comes from a three view of the A-1a, confidence estimate.
 */
const WING_ROOT_CHORD = 2.4;
const WING_TIP_CHORD = 1.07;

/**
 * Sweep of the QUARTER CHORD line, derived from the firm leading edge sweep.
 *
 * The published 18.5 degrees is the sweep of the LEADING EDGE. See the note on
 * the sweep in docs/CONVENTIONS.md section 8, and WING_SWEEP of
 * src/aircraft/me262/geometry.ts, which derives the same 15.72 degrees from the
 * plan form. The wing the eye sees has to be the wing that makes the moment.
 * Confidence: derived from firm data.
 */
const WING_SWEEP = 15.72 * DEG;

/**
 * Model z of the wing quarter chord at the plane of symmetry, in m.
 *
 * Bead b65 moved the root station from 4.850 m to 4.992 m, so that the quarter
 * chord line of the corrected sweep still passes through 25 percent of the mean
 * aerodynamic chord at the center of gravity. The drawn wing follows.
 */
const WING_ROOT_QUARTER_Z = 4.992 - CG_OFFSET_FROM_NOSE;

/** Wing thickness ratios. Source: CONVENTIONS section 8, firm. */
const WING_ROOT_THICKNESS = 0.11;
const WING_TIP_THICKNESS = 0.09;

/** Dihedral of the outer panel, and where it starts. Estimate from photos. */
const WING_DIHEDRAL = 3.5 * DEG;
const WING_DIHEDRAL_START = 2.2;

/** Wing incidence at the root, in rad. Estimate from a three view. */
const WING_ROOT_INCIDENCE = 1.5 * DEG;

/** Spanwise position of the engine center line, in m. Estimate from photos. */
const NACELLE_SPAN = 2.05;

/** Nacelle size, in m. Source: the task brief. */
const NACELLE_RADIUS = 0.425;
const NACELLE_LENGTH = 3.8;

/** Ground line of the model, in m. The wheels touch the ground at this y. */
const GROUND_Y = -1.33;

/** Angle above which `toCreasedNormals` keeps a hard edge, in rad. */
const CREASE_ANGLE = 50 * DEG;

/** Distance aft of the nose tip, turned into a model z coordinate. */
function aft(distanceFromNose: number): number {
  return distanceFromNose - CG_OFFSET_FROM_NOSE;
}

// ---------------------------------------------------------------------------
// Loft, the one surface builder
// ---------------------------------------------------------------------------

interface LoftOptions {
  /** Join the last point of every ring back to the first point. */
  closedProfile?: boolean;
  /** Join the last ring back to the first ring. */
  closedRings?: boolean;
  /** Close the first ring with a triangle fan. */
  capStart?: boolean;
  /** Close the last ring with a triangle fan. */
  capEnd?: boolean;
  /** Reverse every triangle. A mirrored part needs this. */
  flip?: boolean;
}

/**
 * Build a surface from a list of rings. Every ring holds the same number of
 * points. The surface joins ring `i` to ring `i + 1`.
 *
 * Winding rule: let `w` be the direction in which the rings advance, and let
 * `u` and `v` span the plane of one ring so that `u`, `v`, `w` is right handed.
 * A ring that runs counter clockwise from `u` toward `v` then gives outward
 * normals. Check the rule for every new part, or set `flip`.
 */
function loft(rings: Vector3[][], options: LoftOptions = {}): BufferGeometry {
  const ringCount = rings.length;
  const pointCount = rings[0].length;

  const positions: number[] = [];
  const uvs: number[] = [];
  for (let r = 0; r < ringCount; r++) {
    const ring = rings[r];
    for (let i = 0; i < pointCount; i++) {
      positions.push(ring[i].x, ring[i].y, ring[i].z);
      uvs.push(i / pointCount, ringCount > 1 ? r / (ringCount - 1) : 0);
    }
  }

  const triangles: number[] = [];
  const push = (a: number, b: number, c: number): void => {
    if (options.flip === true) triangles.push(a, c, b);
    else triangles.push(a, b, c);
  };

  const lastRing = options.closedRings === true ? ringCount : ringCount - 1;
  const lastPoint = options.closedProfile === true ? pointCount : pointCount - 1;
  for (let r = 0; r < lastRing; r++) {
    const r1 = (r + 1) % ringCount;
    for (let i = 0; i < lastPoint; i++) {
      const i1 = (i + 1) % pointCount;
      const a = r * pointCount + i;
      const b = r * pointCount + i1;
      const c = r1 * pointCount + i1;
      const d = r1 * pointCount + i;
      push(a, b, c);
      push(a, c, d);
    }
  }

  const addCap = (ringIndex: number, atStart: boolean): void => {
    const ring = rings[ringIndex];
    const center = new Vector3();
    for (const p of ring) center.add(p);
    center.divideScalar(pointCount);
    const centerIndex = positions.length / 3;
    positions.push(center.x, center.y, center.z);
    uvs.push(0.5, atStart ? 0 : 1);
    for (let i = 0; i < pointCount; i++) {
      const i1 = (i + 1) % pointCount;
      const a = ringIndex * pointCount + i;
      const b = ringIndex * pointCount + i1;
      // The start cap faces against the ring advance, so it winds the other way.
      if (atStart) push(centerIndex, b, a);
      else push(centerIndex, a, b);
    }
  };

  if (options.capStart === true) addCap(0, true);
  if (options.capEnd === true) addCap(ringCount - 1, false);

  const geometry = new BufferGeometry();
  geometry.setAttribute('position', new Float32BufferAttribute(positions, 3));
  geometry.setAttribute('uv', new Float32BufferAttribute(uvs, 2));
  geometry.setIndex(triangles);
  geometry.computeVertexNormals();
  return geometry;
}

/**
 * Turn a contour into a surface of revolution about the z axis. Each contour
 * point holds an axial position in `x` and a radius in `y`.
 */
function revolveZ(contour: Vector2[], segments: number, options: LoftOptions = {}): BufferGeometry {
  const rings = contour.map((point) => {
    const ring: Vector3[] = [];
    for (let i = 0; i < segments; i++) {
      const angle = (i / segments) * Math.PI * 2;
      ring.push(new Vector3(Math.cos(angle) * point.y, Math.sin(angle) * point.y, point.x));
    }
    return ring;
  });
  return loft(rings, { closedProfile: true, ...options });
}

/** Build a tapered rod between two model points. */
function rod(from: Vector3, to: Vector3, radiusFrom: number, radiusTo: number): BufferGeometry {
  const direction = new Vector3().subVectors(to, from);
  const length = direction.length();
  // CylinderGeometry stands along +y with its top at +height / 2, so turn +y
  // onto the direction of the rod and then slide it to the mid point.
  const geometry = new CylinderGeometry(radiusTo, radiusFrom, length, 10, 1, false);
  geometry.applyQuaternion(
    new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), direction.clone().normalize()),
  );
  geometry.translate(
    from.x + direction.x / 2,
    from.y + direction.y / 2,
    from.z + direction.z / 2,
  );
  return geometry;
}

// ---------------------------------------------------------------------------
// Cross section shapes
// ---------------------------------------------------------------------------

/**
 * Build the normalized fuselage cross section. Read section 3 of the module
 * comment for the method. The result runs counter clockwise, spans -1 to 1 in
 * x, and spans -1 to 1 in y.
 */
function roundedTriangleProfile(count: number): Vector2[] {
  // Four half planes: a wide flat bottom, two sloped sides, and a narrow top
  // deck. A plane holds a unit normal and a distance from the section center.
  //
  // The side planes run from the widest corner at (1.15, -0.30) to the deck
  // corner at (0.68, 0.72). Those two corners set how fast the section closes
  // in toward the top. An earlier set closed much faster and left a deck only
  // 0.21 m wide at the sill of the hood, which no cockpit can use. The corners
  // below hold the deck at 0.60 m across at the sill and still taper to 0.20 m
  // at the crown, so the section keeps its triangular read.
  const planes = [
    { nx: 0, ny: -1, d: 0.62 },
    { nx: 0.9082, ny: 0.4185, d: 0.9189 },
    { nx: -0.9082, ny: 0.4185, d: 0.9189 },
    { nx: 0, ny: 1, d: 0.72 },
  ];
  // A larger exponent gives sharper corners. Six gives a soft rounded corner.
  const exponent = 6;

  const raw: Vector2[] = [];
  for (let i = 0; i < count; i++) {
    const angle = (i / count) * Math.PI * 2;
    const ux = Math.cos(angle);
    const uy = Math.sin(angle);
    let sum = 0;
    for (const plane of planes) {
      const support = (plane.nx * ux + plane.ny * uy) / plane.d;
      if (support > 0) sum += Math.pow(support, exponent);
    }
    const radius = Math.pow(sum, -1 / exponent);
    raw.push(new Vector2(ux * radius, uy * radius));
  }

  let maxX = 0;
  let minY = Number.POSITIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const p of raw) {
    maxX = Math.max(maxX, Math.abs(p.x));
    minY = Math.min(minY, p.y);
    maxY = Math.max(maxY, p.y);
  }
  const centerY = (minY + maxY) / 2;
  const halfY = (maxY - minY) / 2;
  return raw.map((p) => new Vector2(p.x / maxX, (p.y - centerY) / halfY));
}

/** Half thickness of a symmetric NACA four digit section, as a chord fraction. */
function naca4HalfThickness(chordFraction: number, thickness: number): number {
  const x = Math.min(Math.max(chordFraction, 0), 1);
  return (
    (thickness / 0.2) *
    (0.2969 * Math.sqrt(x) -
      0.126 * x -
      0.3516 * x * x +
      0.2843 * x * x * x -
      0.1015 * x * x * x * x)
  );
}

/**
 * Build one closed airfoil loop between two chord fractions. The loop runs from
 * the front along the upper surface to the back, and then along the lower
 * surface back to the front. Cosine spacing puts more points at both ends.
 *
 * The real sections are NACA 00011-0.825-35 at the root and NACA 00009-1.1-40
 * at the tip. Those modified sections carry their maximum thickness at 35 and
 * 40 percent of the chord. The four digit form used here carries it at 30
 * percent. The difference is a few millimeters on the model and no eye sees it.
 */
function airfoilLoop(
  pointsPerSide: number,
  front: number,
  back: number,
  thickness: number,
): Vector2[] {
  const loop: Vector2[] = [];
  const station = (k: number): number => {
    const u = 0.5 * (1 - Math.cos((Math.PI * k) / (pointsPerSide - 1)));
    return front + (back - front) * u;
  };
  for (let k = 0; k < pointsPerSide; k++) {
    const x = station(k);
    loop.push(new Vector2(x, naca4HalfThickness(x, thickness)));
  }
  for (let k = pointsPerSide - 1; k >= 0; k--) {
    const x = station(k);
    loop.push(new Vector2(x, -naca4HalfThickness(x, thickness)));
  }
  return loop;
}

// ---------------------------------------------------------------------------
// Lifting surfaces
// ---------------------------------------------------------------------------

/**
 * One straight taper lifting surface. The wing, the tail plane, the fin, and
 * every movable surface share this description. A movable surface uses the
 * plan of its parent surface and only changes the station list, so the hinge
 * line and the parent surface always agree.
 */
interface SurfacePlan {
  /** Model position of the quarter chord point at the root. */
  rootQuarter: Vector3;
  /** Unit vector from the root toward the tip. */
  spanAxis: Vector3;
  /** Unit vector from the leading edge toward the trailing edge. */
  chordAxis: Vector3;
  /** Unit vector along positive section thickness. */
  normalAxis: Vector3;
  rootChord: number;
  tipChord: number;
  /** Distance from the root to the tip, in m. */
  spanLength: number;
  /** Aft shift of the quarter chord line per meter of span. */
  sweepTangent: number;
  /** Rise along `normalAxis` per meter of span, outboard of `riseStart`. */
  riseTangent: number;
  riseStart: number;
  rootThickness: number;
  tipThickness: number;
  /** Incidence in rad. A positive value raises the leading edge. */
  rootIncidence: number;
  tipIncidence: number;
}

/** One spanwise station of a lofted lifting surface. */
interface SurfaceStation {
  /** Distance from the root, in m. */
  span: number;
  /** Chord fraction where the section starts. Zero is the leading edge. */
  front: number;
  /** Chord fraction where the section ends. One is the trailing edge. */
  back: number;
  /** Shrink of the section about its mid chord. One keeps the full size. */
  scale?: number;
}

function planChord(plan: SurfacePlan, span: number): number {
  return lerp(plan.rootChord, plan.tipChord, span / plan.spanLength);
}

function planRise(plan: SurfacePlan, span: number): number {
  return span > plan.riseStart ? plan.riseTangent * (span - plan.riseStart) : 0;
}

/**
 * Model position of a point on a lifting surface. `chordFraction` runs from 0
 * at the leading edge to 1 at the trailing edge. `normalOffset` is a chord
 * fraction along the thickness direction.
 */
function surfacePoint(
  plan: SurfacePlan,
  span: number,
  chordFraction: number,
  normalOffset = 0,
): Vector3 {
  const chord = planChord(plan, span);
  const incidence = lerp(plan.rootIncidence, plan.tipIncidence, span / plan.spanLength);
  const along = (chordFraction - 0.25) * chord;
  const up = normalOffset * chord;
  const cos = Math.cos(incidence);
  const sin = Math.sin(incidence);
  return new Vector3()
    .copy(plan.rootQuarter)
    .addScaledVector(plan.spanAxis, span)
    .addScaledVector(plan.chordAxis, plan.sweepTangent * span + along * cos + up * sin)
    .addScaledVector(plan.normalAxis, planRise(plan, span) + up * cos - along * sin);
}

/** Build one ring of a lofted lifting surface. */
function surfaceRing(
  plan: SurfacePlan,
  station: SurfaceStation,
  pointsPerSide: number,
): Vector3[] {
  const fraction = station.span / plan.spanLength;
  const chord = planChord(plan, station.span);
  const thickness = lerp(plan.rootThickness, plan.tipThickness, fraction);
  const incidence = lerp(plan.rootIncidence, plan.tipIncidence, fraction);
  const scale = station.scale ?? 1;
  const rise = planRise(plan, station.span);
  const sweep = plan.sweepTangent * station.span;
  const cos = Math.cos(incidence);
  const sin = Math.sin(incidence);

  return airfoilLoop(pointsPerSide, station.front, station.back, thickness).map((p) => {
    // Shrink about the mid chord, then measure from the quarter chord.
    const along = ((p.x - 0.5) * scale + 0.25) * chord;
    const up = p.y * chord * scale;
    return new Vector3()
      .copy(plan.rootQuarter)
      .addScaledVector(plan.spanAxis, station.span)
      .addScaledVector(plan.chordAxis, sweep + along * cos + up * sin)
      .addScaledVector(plan.normalAxis, rise + up * cos - along * sin);
  });
}

/**
 * Loft a lifting surface. The winding follows from the three axes of the plan.
 * If `chordAxis` crossed with `spanAxis` points against `normalAxis`, the loop
 * runs the other way and the triangles need a flip.
 */
function liftingSurface(
  plan: SurfacePlan,
  stations: SurfaceStation[],
  pointsPerSide: number,
  options: LoftOptions = {},
): BufferGeometry {
  const handedness = new Vector3()
    .crossVectors(plan.chordAxis, plan.spanAxis)
    .dot(plan.normalAxis);
  const rings = stations.map((station) => surfaceRing(plan, station, pointsPerSide));
  return loft(rings, {
    closedProfile: true,
    capStart: true,
    capEnd: true,
    flip: handedness < 0,
    ...options,
  });
}

/** Mirror a plan onto the other side of the aircraft. */
function mirrorPlan(plan: SurfacePlan): SurfacePlan {
  return {
    ...plan,
    rootQuarter: new Vector3(-plan.rootQuarter.x, plan.rootQuarter.y, plan.rootQuarter.z),
    spanAxis: new Vector3(-plan.spanAxis.x, plan.spanAxis.y, plan.spanAxis.z),
    normalAxis: new Vector3(-plan.normalAxis.x, plan.normalAxis.y, plan.normalAxis.z),
  };
}

// ---------------------------------------------------------------------------
// The wing, the tail plane, and the fin
// ---------------------------------------------------------------------------

const WING_RIGHT: SurfacePlan = {
  rootQuarter: new Vector3(0, 0, WING_ROOT_QUARTER_Z),
  spanAxis: new Vector3(1, 0, 0),
  chordAxis: new Vector3(0, 0, 1),
  normalAxis: new Vector3(0, 1, 0),
  rootChord: WING_ROOT_CHORD,
  tipChord: WING_TIP_CHORD,
  spanLength: HALF_SPAN,
  sweepTangent: Math.tan(WING_SWEEP),
  riseTangent: Math.tan(WING_DIHEDRAL),
  riseStart: WING_DIHEDRAL_START,
  rootThickness: WING_ROOT_THICKNESS,
  tipThickness: WING_TIP_THICKNESS,
  rootIncidence: WING_ROOT_INCIDENCE,
  tipIncidence: 0,
};

/** Chord fraction of the flap hinge line and of the aileron hinge line. */
const FLAP_HINGE = 0.74;
const AILERON_HINGE = 0.72;
/** Chord fraction where the fixed wing starts behind the slat. */
const SLAT_HINGE = 0.12;

/**
 * Span limits of the movable surfaces, in m from the plane of symmetry. The
 * nacelle splits the flap into an inner panel and an outer panel. Both panels
 * share one hinge line, so both hang from one pivot.
 */
const FLAP_INNER = [0.62, 1.56] as const;
const FLAP_OUTER = [2.5, 3.38] as const;
const AILERON_SPAN = [4.0, 5.98] as const;
const SLAT_SPAN = [3.0, 6.02] as const;

/**
 * Station list of the fixed wing. The list must rise in span. Each break in a
 * chord fraction needs two stations 0.04 m apart, which makes a crisp step at
 * the end of a movable surface instead of a long ramp. The last four stations
 * shrink the section and round the tip.
 */
const WING_STATIONS: SurfaceStation[] = [
  { span: 0, front: 0, back: 1 },
  { span: 0.58, front: 0, back: 1 },
  { span: FLAP_INNER[0], front: 0, back: FLAP_HINGE },
  { span: FLAP_INNER[1], front: 0, back: FLAP_HINGE },
  { span: 1.6, front: 0, back: 1 },
  { span: 2.46, front: 0, back: 1 },
  { span: FLAP_OUTER[0], front: 0, back: FLAP_HINGE },
  { span: SLAT_SPAN[0] - 0.04, front: 0, back: FLAP_HINGE },
  { span: SLAT_SPAN[0], front: SLAT_HINGE, back: FLAP_HINGE },
  { span: FLAP_OUTER[1], front: SLAT_HINGE, back: FLAP_HINGE },
  { span: FLAP_OUTER[1] + 0.04, front: SLAT_HINGE, back: 1 },
  { span: AILERON_SPAN[0] - 0.04, front: SLAT_HINGE, back: 1 },
  { span: AILERON_SPAN[0], front: SLAT_HINGE, back: AILERON_HINGE },
  { span: AILERON_SPAN[1], front: SLAT_HINGE, back: AILERON_HINGE },
  { span: SLAT_SPAN[1], front: SLAT_HINGE, back: 1 },
  { span: SLAT_SPAN[1] + 0.04, front: 0, back: 1 },
  { span: 6.14, front: 0, back: 1 },
  { span: 6.19, front: 0, back: 1, scale: 0.8 },
  { span: 6.235, front: 0, back: 1, scale: 0.5 },
  { span: HALF_SPAN, front: 0, back: 1, scale: 0.18 },
];

/**
 * Fin root height above the model x z plane, and fin span from that root to the
 * tip, in m.
 *
 * The root sits at y = 0.50, inside the fuselage. A span of 2.00 m puts the tip
 * at y = 2.50, which is 3.83 m above the ground line at y = -1.33.
 *
 * THIS FILE ONCE DREW A FIN OF 1.67 m. It back-solved that span from an overall
 * height of 3.50 m, which CONVENTIONS section 8 then gave and marked firm. The
 * height was wrong. The National Air and Space Museum gives 12 ft 7 in, that is
 * 3.84 m, for the A-1a airframe it holds, and three other sources give 3.8 m to
 * 3.84 m. Section 8 now carries 3.83 m and a note on the error. The method was
 * right and the input was wrong, so the span follows the corrected input.
 *
 * src/aircraft/me262/geometry.ts flies the same fin, with FIN_SPAN = 2.00 m and
 * an effective aspect ratio of 1.97. The two files now agree.
 */
const FIN_ROOT_Y = 0.5;
const FIN_SPAN = 2.0;

/**
 * The fin chords and the fin sweep are estimates from a three view. They do not
 * change with the span. The chords give a fin area of 3.70 m2 over the 2.00 m
 * span, which is the area the DATCOM fit of src/aircraft/me262/geometry.ts
 * reads against. The quarter chord sweeps 30.7 deg and the leading edge 37.6
 * deg.
 */
const FIN: SurfacePlan = {
  rootQuarter: new Vector3(0, FIN_ROOT_Y, aft(8.1175)),
  spanAxis: new Vector3(0, 1, 0),
  chordAxis: new Vector3(0, 0, 1),
  normalAxis: new Vector3(1, 0, 0),
  rootChord: 2.55,
  tipChord: 1.15,
  spanLength: FIN_SPAN,
  sweepTangent: 0.594,
  riseTangent: 0,
  riseStart: 0,
  rootThickness: 0.1,
  tipThickness: 0.09,
  rootIncidence: 0,
  tipIncidence: 0,
};

/** Span of the rounded tip cap above the top of the rudder, in m. */
const FIN_TIP_CAP = 0.15;

/**
 * Chord fraction of the rudder hinge line, and the span it covers. The span
 * runs from the fin root, so 0.25 is 0.25 m above y = 0.50. The rudder stops
 * FIN_TIP_CAP below the tip, where the cap starts. RUDDER_SPAN of
 * src/aircraft/me262/geometry.ts holds the same two numbers.
 */
const RUDDER_HINGE = 0.62;
const RUDDER_SPAN = [0.25, FIN_SPAN - FIN_TIP_CAP] as const;

/**
 * Station list of the fin. The last three stations shrink the section and round
 * the tip over the 0.15 m of the cap.
 */
const FIN_STATIONS: SurfaceStation[] = [
  { span: 0, front: 0, back: 1 },
  { span: RUDDER_SPAN[0] - 0.04, front: 0, back: 1 },
  { span: RUDDER_SPAN[0], front: 0, back: RUDDER_HINGE },
  { span: RUDDER_SPAN[1], front: 0, back: RUDDER_HINGE },
  { span: FIN_SPAN - 0.09, front: 0, back: 1 },
  { span: FIN_SPAN - 0.04, front: 0, back: 1, scale: 0.68 },
  { span: FIN_SPAN, front: 0, back: 1, scale: 0.24 },
];

/**
 * The tailplane sits 0.20 m above the fin root, which is 10 percent of the fin
 * span. src/aircraft/me262/geometry.ts reads that same 10 percent into the
 * DATCOM end plate fit that gives the fin its effective aspect ratio, so the
 * mount stays where it is while the fin grows.
 */
const TAILPLANE_ROOT_Y = FIN_ROOT_Y + 0.2;

const TAILPLANE_RIGHT: SurfacePlan = {
  rootQuarter: new Vector3(0, TAILPLANE_ROOT_Y, aft(8.905)),
  spanAxis: new Vector3(1, 0, 0),
  chordAxis: new Vector3(0, 0, 1),
  normalAxis: new Vector3(0, 1, 0),
  rootChord: 1.42,
  tipChord: 0.78,
  spanLength: 1.8,
  sweepTangent: Math.tan(12 * DEG),
  riseTangent: 0,
  riseStart: 0,
  rootThickness: 0.09,
  tipThickness: 0.08,
  rootIncidence: 0,
  tipIncidence: 0,
};

const ELEVATOR_HINGE = 0.68;
const ELEVATOR_SPAN = [0.3, 1.7] as const;

const TAILPLANE_STATIONS: SurfaceStation[] = [
  { span: 0, front: 0, back: 1 },
  { span: ELEVATOR_SPAN[0] - 0.04, front: 0, back: 1 },
  { span: ELEVATOR_SPAN[0], front: 0, back: ELEVATOR_HINGE },
  { span: ELEVATOR_SPAN[1], front: 0, back: ELEVATOR_HINGE },
  { span: 1.74, front: 0, back: 1 },
  { span: 1.78, front: 0, back: 1, scale: 0.72 },
  { span: 1.8, front: 0, back: 1, scale: 0.26 },
];

// ---------------------------------------------------------------------------
// Fuselage stations
// ---------------------------------------------------------------------------

interface FuselageStation {
  /** Distance aft of the nose tip, in m. */
  t: number;
  halfWidth: number;
  halfHeight: number;
  /** Height of the section center above the model x z plane, in m. */
  centerY: number;
}

/**
 * Fuselage stations. The widest section is 1.11 m across and 1.56 m deep, which
 * matches a three view of the A-1a. Confidence estimate, because no drawing in
 * the reference set carries a section table.
 */
const FUSELAGE_STATIONS: FuselageStation[] = [
  { t: 0, halfWidth: 0.045, halfHeight: 0.055, centerY: 0.1 },
  { t: 0.25, halfWidth: 0.135, halfHeight: 0.165, centerY: 0.092 },
  { t: 0.6, halfWidth: 0.235, halfHeight: 0.288, centerY: 0.075 },
  { t: 1.1, halfWidth: 0.335, halfHeight: 0.422, centerY: 0.05 },
  { t: 1.75, halfWidth: 0.432, halfHeight: 0.56, centerY: 0.022 },
  { t: 2.5, halfWidth: 0.505, halfHeight: 0.68, centerY: 0 },
  { t: 3.3, halfWidth: 0.545, halfHeight: 0.755, centerY: -0.02 },
  { t: 4.2, halfWidth: 0.555, halfHeight: 0.78, centerY: -0.03 },
  { t: 5.1, halfWidth: 0.555, halfHeight: 0.78, centerY: -0.03 },
  { t: 6, halfWidth: 0.54, halfHeight: 0.765, centerY: -0.02 },
  { t: 6.9, halfWidth: 0.5, halfHeight: 0.72, centerY: 0 },
  { t: 7.8, halfWidth: 0.44, halfHeight: 0.64, centerY: 0.045 },
  { t: 8.6, halfWidth: 0.372, halfHeight: 0.552, centerY: 0.095 },
  { t: 9.4, halfWidth: 0.288, halfHeight: 0.442, centerY: 0.15 },
  { t: 10.1, halfWidth: 0.205, halfHeight: 0.34, centerY: 0.212 },
  { t: 10.35, halfWidth: 0.17, halfHeight: 0.288, centerY: 0.238 },
  { t: 10.55, halfWidth: 0.108, halfHeight: 0.196, centerY: 0.262 },
  { t: LENGTH, halfWidth: 0.048, halfHeight: 0.092, centerY: 0.272 },
];

function catmullRom(p0: number, p1: number, p2: number, p3: number, t: number): number {
  const t2 = t * t;
  const t3 = t2 * t;
  return (
    0.5 *
    (2 * p1 +
      (-p0 + p2) * t +
      (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
      (-p0 + 3 * p1 - 3 * p2 + p3) * t3)
  );
}

/** Add `steps - 1` smooth rings between every pair of hand written stations. */
function refineStations(stations: FuselageStation[], steps: number): FuselageStation[] {
  const out: FuselageStation[] = [];
  const n = stations.length;
  for (let i = 0; i < n - 1; i++) {
    const a = stations[Math.max(i - 1, 0)];
    const b = stations[i];
    const c = stations[i + 1];
    const d = stations[Math.min(i + 2, n - 1)];
    for (let s = 0; s < steps; s++) {
      const u = s / steps;
      out.push({
        t: catmullRom(a.t, b.t, c.t, d.t, u),
        halfWidth: Math.max(0.01, catmullRom(a.halfWidth, b.halfWidth, c.halfWidth, d.halfWidth, u)),
        halfHeight: Math.max(
          0.01,
          catmullRom(a.halfHeight, b.halfHeight, c.halfHeight, d.halfHeight, u),
        ),
        centerY: catmullRom(a.centerY, b.centerY, c.centerY, d.centerY, u),
      });
    }
  }
  out.push(stations[n - 1]);
  return out;
}

// ---------------------------------------------------------------------------
// Canopy sections
// ---------------------------------------------------------------------------

interface CanopyStation {
  /** Distance aft of the nose tip, in m. */
  t: number;
  halfWidth: number;
  /** Height of the crown above the sill, in m. */
  height: number;
  /** Height of the sill above the model x z plane, in m. */
  sillY: number;
  /**
   * Shape of the crown. One gives a half ellipse. A larger value gives a
   * pointed section, which is how a flat panel windscreen reads. A smaller
   * value gives a square section, which is how a framed hood reads.
   */
  sharpness: number;
}

/** Points on the crown arc of a canopy ring. The rest lie on the sill line. */
const CANOPY_CROWN_POINTS = 13;
const CANOPY_POINTS = 20;

/**
 * Build one cross section of the canopy.
 *
 * `crownOnly` leaves out the sill line and gives the crown arc alone. A hood is
 * OPEN at the bottom, so the glass shell and every frame hoop must use that
 * form. A closed section carries a flat glass floor at the sill and a solid bar
 * across the cockpit, and the pilot then reads the panel through both. The hoop
 * at station 3.76 belongs to the sliding hood, so such a bar would also sweep
 * through the chest of the pilot when the hood opens.
 *
 * The dorsal spine behind the hood is a solid fairing, not a hood. It keeps the
 * closed section, because the sill line is its floor and the fuselage skin
 * covers that floor.
 */
function canopyRing(station: CanopyStation, crownOnly = false): Vector3[] {
  const z = aft(station.t);
  const ring: Vector3[] = [];
  const shape = (value: number): number =>
    Math.sign(value) * Math.pow(Math.abs(value), station.sharpness);

  // The crown runs from the starboard sill over the top to the port sill, which
  // is counter clockwise in the x y plane. The rings advance aft, so that
  // direction gives outward normals.
  for (let i = 0; i < CANOPY_CROWN_POINTS; i++) {
    const angle = (i / (CANOPY_CROWN_POINTS - 1)) * Math.PI;
    ring.push(
      new Vector3(
        station.halfWidth * shape(Math.cos(angle)),
        station.sillY + station.height * Math.abs(shape(Math.sin(angle))),
        z,
      ),
    );
  }
  if (crownOnly) return ring;

  const floorPoints = CANOPY_POINTS - CANOPY_CROWN_POINTS;
  for (let i = 1; i <= floorPoints; i++) {
    const u = i / (floorPoints + 1);
    ring.push(new Vector3(station.halfWidth * (2 * u - 1), station.sillY, z));
  }
  return ring;
}

/**
 * The sill sits at y = 0.56, which is below the crown of the fuselage at
 * y = 0.75. The section of the fuselage is 0.33 half width at that height, so
 * the flat floor of the hood stays inside the skin and the glass grows out of
 * the deck with no step and no gap.
 */
const CANOPY_STATIONS: CanopyStation[] = [
  { t: 3.28, halfWidth: 0.235, height: 0.3, sillY: 0.56, sharpness: 1.45 },
  { t: 3.55, halfWidth: 0.285, height: 0.42, sillY: 0.56, sharpness: 1.3 },
  { t: 3.74, halfWidth: 0.31, height: 0.46, sillY: 0.56, sharpness: 1 },
  { t: 4.2, halfWidth: 0.315, height: 0.475, sillY: 0.56, sharpness: 0.88 },
  { t: 4.72, halfWidth: 0.3, height: 0.445, sillY: 0.555, sharpness: 0.88 },
  { t: 5.05, halfWidth: 0.255, height: 0.36, sillY: 0.55, sharpness: 0.92 },
];

const SPINE_STATIONS: CanopyStation[] = [
  { t: 5.05, halfWidth: 0.255, height: 0.36, sillY: 0.55, sharpness: 0.92 },
  { t: 5.55, halfWidth: 0.225, height: 0.28, sillY: 0.55, sharpness: 0.95 },
  { t: 6.2, halfWidth: 0.185, height: 0.19, sillY: 0.55, sharpness: 1 },
  { t: 6.9, halfWidth: 0.135, height: 0.13, sillY: 0.545, sharpness: 1 },
  { t: 7.5, halfWidth: 0.09, height: 0.09, sillY: 0.52, sharpness: 1 },
];

/** Blend two canopy stations, so a frame hoop can sit between two rings. */
function canopyAt(t: number): CanopyStation {
  const list = CANOPY_STATIONS;
  for (let i = 0; i < list.length - 1; i++) {
    if (t <= list[i + 1].t) {
      const u = (t - list[i].t) / (list[i + 1].t - list[i].t);
      const a = list[i];
      const b = list[i + 1];
      return {
        t,
        halfWidth: lerp(a.halfWidth, b.halfWidth, u),
        height: lerp(a.height, b.height, u),
        sillY: lerp(a.sillY, b.sillY, u),
        sharpness: lerp(a.sharpness, b.sharpness, u),
      };
    }
  }
  return { ...list[list.length - 1], t };
}

/**
 * Build one frame hoop of the canopy. The hoop is a thin band that follows the
 * crown arc and stands a little proud of the glass.
 *
 * The band runs from the starboard sill over the crown to the port sill and it
 * stops there, because a hoop of a real hood stops at the sill. `closedRings`
 * closes the rectangular cross section of the band. `closedProfile` stays off,
 * so the band gets no bar across the cockpit.
 */
function canopyHoop(t: number, halfDepth: number, thickness: number): BufferGeometry {
  const inner = canopyAt(t);
  const outer: CanopyStation = {
    ...inner,
    halfWidth: inner.halfWidth + thickness,
    height: inner.height + thickness,
    sillY: inner.sillY - thickness,
  };
  const rings = [
    canopyRing({ ...outer, t: t - halfDepth }, true),
    canopyRing({ ...outer, t: t + halfDepth }, true),
    canopyRing({ ...inner, t: t + halfDepth }, true),
    canopyRing({ ...inner, t: t - halfDepth }, true),
  ];
  return loft(rings, { closedProfile: false, closedRings: true });
}

// ---------------------------------------------------------------------------
// Nacelle contour
// ---------------------------------------------------------------------------

/**
 * Contour of one nacelle, as an axial position and a radius. The list walks the
 * outer skin from the intake lip to the jet pipe, and then walks the duct wall
 * back to the lip. `closedRings` joins the last point to the first and forms
 * the rounded lip, so the intake and the exhaust both stay open.
 */
const NACELLE_CONTOUR: Vector2[] = [
  new Vector2(0, 0.372),
  new Vector2(0.03, 0.392),
  new Vector2(0.09, 0.41),
  new Vector2(0.22, 0.421),
  new Vector2(0.7, NACELLE_RADIUS),
  new Vector2(1.5, NACELLE_RADIUS),
  new Vector2(2.3, 0.42),
  new Vector2(2.9, 0.402),
  new Vector2(3.4, 0.372),
  new Vector2(NACELLE_LENGTH, 0.338),
  new Vector2(NACELLE_LENGTH, 0.312),
  new Vector2(3.3, 0.318),
  new Vector2(2.6, 0.322),
  new Vector2(1.7, 0.322),
  new Vector2(1, 0.33),
  new Vector2(0.4, 0.336),
  new Vector2(0.14, 0.345),
  new Vector2(0.04, 0.358),
];

/** Front bearing fairing inside the intake, the bullet of the Jumo 004. */
const NACELLE_BULLET: Vector2[] = [
  new Vector2(0.02, 0.02),
  new Vector2(0.12, 0.088),
  new Vector2(0.28, 0.145),
  new Vector2(0.5, 0.172),
  new Vector2(0.85, 0.178),
  new Vector2(1.2, 0.178),
];

/** Exhaust cone, the movable body of the Jumo 004 jet pipe. */
const NACELLE_CONE: Vector2[] = [
  new Vector2(2.95, 0.155),
  new Vector2(3.3, 0.165),
  new Vector2(3.62, 0.14),
  new Vector2(3.88, 0.095),
];

/** Model z of the nacelle nose. The nacelle reaches well past the wing. */
const NACELLE_FRONT_Z = -1.85;
/** Height of the nacelle center line. The top touches the wing lower skin. */
const NACELLE_Y = -0.53;

// ---------------------------------------------------------------------------
// Landing gear layout
// ---------------------------------------------------------------------------

const NOSE_TRUNNION = new Vector3(0, -0.42, aft(2.25));
const NOSE_AXLE = new Vector3(0, -1, aft(2.18));
const NOSE_WHEEL_RADIUS = 0.33;
const NOSE_WHEEL_HALF_WIDTH = 0.075;

const MAIN_TRUNNION = new Vector3(1.18, -0.14, 0.52);
const MAIN_AXLE = new Vector3(1.18, -0.91, 0.32);
const MAIN_WHEEL_RADIUS = 0.42;
const MAIN_WHEEL_HALF_WIDTH = 0.14;

/**
 * Full oleo stroke of every leg, in m.
 *
 * DUPLICATED from NOSE_TRAVEL and MAIN_TRAVEL of src/physics/gear.ts, which
 * both hold 0.28 m. CONVENTIONS section 4 stops the physics from importing the
 * renderer and it stops this file from importing the physics, so the number
 * appears in both places. The two must stay equal, or the model shows a stroke
 * the aircraft does not have.
 */
export const ME262_GEAR_TRAVEL = 0.28;

/**
 * Oleo stroke at which the model is drawn, in m.
 *
 * STATIC_STROKE_FRACTION of src/physics/gear.ts is 0.55, so a parked aircraft
 * at the design mass of 6396 kg stands at 0.55 * 0.28 = 0.154 m of stroke on
 * all three legs. The wheels of the model touch GROUND_Y at that value, so this
 * is the value `reset` writes.
 */
export const ME262_GEAR_STATIC_COMPRESSION = 0.55 * ME262_GEAR_TRAVEL;

/**
 * Length of the gas cylinder of each leg, and the depth of the head of the
 * piston inside it at the static stroke. Both run down the leg axis from the
 * trunnion, in m.
 *
 * THE TIRE SETS THESE VALUES, NOT THE STRUT. A main leg is 0.796 m long and its
 * tire is 0.84 m across, so the tire hides everything below y = -0.49 and only
 * 0.35 m of leg shows at rest. The cylinder must therefore end high, or no part
 * of the piston ever shows.
 *
 * Three limits fight over the pair:
 *
 *   The cylinder must end above the top of the tire, or the split never shows.
 *   The head of the piston must stay inside the cylinder at full extension.
 *   The head must stay under the skin at the hard stop.
 *
 * The values below hold all three. At full extension the main piston keeps
 * 0.026 m of engagement and the nose piston keeps 0.025 m. At the hard stop the
 * main head stands 0.015 m above its trunnion, which the wing lower skin
 * covers, and the nose head stands 0.007 m above its trunnion, which sits well
 * inside the fuselage.
 *
 * The 0.28 m stroke is 48 percent of the length of the nose leg, so the nose
 * piston only leaves its cylinder once the wheel comes off the ground. On the
 * ground the nose stroke reads as the wheel rising into the belly, which is the
 * larger cue in any case.
 */
const NOSE_CYLINDER_LENGTH = 0.3;
const NOSE_PISTON_HEAD = 0.12;
const MAIN_CYLINDER_LENGTH = 0.3;
const MAIN_PISTON_HEAD = 0.115;

/**
 * The moving half of one landing gear leg.
 *
 * `node` carries the piston, the torque link and the wheel pivot. Its position
 * runs along `axis`, which is the leg axis written in the local frame of the
 * leg pivot and points from the axle toward the trunnion. `scale` turns a
 * vertical stroke into a distance along that axis.
 */
interface GearSlider {
  node: Object3D;
  axis: Vector3;
  scale: number;
}

/**
 * Build the slider of one leg. `trunnion` and `axle` are model points, and the
 * leg runs between them.
 */
function makeSlider(leg: Object3D, trunnion: Vector3, axle: Vector3): GearSlider {
  const up = new Vector3().subVectors(trunnion, axle).normalize();
  leg.updateWorldMatrix(true, false);
  // transformDirection drops the translation and normalizes, so this is the
  // same direction written in the frame of the leg pivot.
  const axis = up
    .clone()
    .transformDirection(new Matrix4().copy(leg.matrixWorld).invert());

  const node = new Object3D();
  node.name = `${leg.name}-slider`;
  leg.add(node);
  // A leg that rakes must slide further along its own axis than the wheel
  // rises, or the model shows less stroke than the physics computes.
  return { node, axis, scale: 1 / up.y };
}

/** Place one slider at a strut stroke, in m. */
function setSlider(slider: GearSlider, compression: number): void {
  const stroke =
    clamp(compression, 0, ME262_GEAR_TRAVEL) - ME262_GEAR_STATIC_COMPRESSION;
  slider.node.position.copy(slider.axis).multiplyScalar(stroke * slider.scale);
}

/** A point on the leg axis, `distance` meters below the trunnion. */
function alongLeg(trunnion: Vector3, axle: Vector3, distance: number): Vector3 {
  const down = new Vector3().subVectors(axle, trunnion).normalize();
  return new Vector3().copy(trunnion).addScaledVector(down, distance);
}

/** Build a wheel about the model x axis, centered on `axle`. */
function wheelGeometry(radius: number, halfWidth: number, hubRadius: number): BufferGeometry {
  const contour: Vector2[] = [
    new Vector2(-halfWidth, hubRadius),
    new Vector2(-halfWidth, radius * 0.62),
    new Vector2(-halfWidth * 0.93, radius * 0.85),
    new Vector2(-halfWidth * 0.62, radius * 0.97),
    new Vector2(0, radius),
    new Vector2(halfWidth * 0.62, radius * 0.97),
    new Vector2(halfWidth * 0.93, radius * 0.85),
    new Vector2(halfWidth, radius * 0.62),
    new Vector2(halfWidth, hubRadius),
  ];
  const geometry = revolveZ(contour, 20, { capStart: true, capEnd: true });
  // The revolve runs about z. Turn +z onto +x to put the axle across the model.
  geometry.rotateY(Math.PI / 2);
  return geometry;
}

// ---------------------------------------------------------------------------
// Build context
// ---------------------------------------------------------------------------

/** The moving half of the three legs. `setGearCompression` drives all three. */
interface GearSliders {
  nose: GearSlider;
  left: GearSlider;
  right: GearSlider;
}

interface BuildContext {
  root: Object3D;
  materials: ModelMaterialSet;
  geometries: BufferGeometry[];
  sliders: Partial<GearSliders>;
}

interface AttachOptions {
  castShadow?: boolean;
  receiveShadow?: boolean;
}

/**
 * Finish one geometry and hang it under `parent`.
 *
 * The geometry arrives in model coordinates. The function splits the hard
 * edges, writes the model space attributes that the materials read, and then
 * moves the geometry into the local frame of `parent`. The mesh itself keeps an
 * identity transform, so a later turn of a pivot moves the part correctly.
 */
function attach(
  context: BuildContext,
  parent: Object3D,
  geometry: BufferGeometry,
  material: Material,
  name: string,
  options: AttachOptions = {},
): Mesh {
  const creased = toCreasedNormals(geometry, CREASE_ANGLE);
  if (creased !== geometry) geometry.dispose();
  bakeModelSpaceAttributes(creased);

  parent.updateWorldMatrix(true, false);
  creased.applyMatrix4(new Matrix4().copy(parent.matrixWorld).invert());

  const mesh = new Mesh(creased, material);
  mesh.name = name;
  mesh.castShadow = options.castShadow ?? true;
  mesh.receiveShadow = options.receiveShadow ?? true;
  parent.add(mesh);
  context.geometries.push(creased);
  return mesh;
}

/**
 * Build a hinge. The function returns the inner object, which holds an identity
 * transform. Bead b19 writes `rotation.x` on that object. The outer object
 * holds the position and the base orientation, so the base frame survives.
 *
 * `xAxis` becomes the local +x axis, which is the axis of the turn. `yHint`
 * chooses the local +y axis. The local +z axis follows from the right hand
 * rule.
 */
function makePivot(
  parent: Object3D,
  name: string,
  origin: Vector3,
  xAxis: Vector3,
  yHint: Vector3,
): Object3D {
  const x = xAxis.clone().normalize();
  const z = new Vector3().crossVectors(x, yHint).normalize();
  const y = new Vector3().crossVectors(z, x);

  const world = new Matrix4().makeBasis(x, y, z).setPosition(origin);
  parent.updateWorldMatrix(true, false);
  const local = new Matrix4().copy(parent.matrixWorld).invert().multiply(world);

  const mount = new Object3D();
  mount.name = `${name}-mount`;
  local.decompose(mount.position, mount.quaternion, mount.scale);
  parent.add(mount);

  const pivot = new Object3D();
  pivot.name = name;
  mount.add(pivot);
  return pivot;
}

// ---------------------------------------------------------------------------
// Part builders
// ---------------------------------------------------------------------------

function buildFuselage(context: BuildContext): void {
  const profile = roundedTriangleProfile(32);
  const stations = refineStations(FUSELAGE_STATIONS, 2);
  const rings = stations.map((station) =>
    profile.map(
      (p) =>
        new Vector3(
          p.x * station.halfWidth,
          station.centerY + p.y * station.halfHeight,
          aft(station.t),
        ),
    ),
  );
  const geometry = loft(rings, { closedProfile: true, capStart: true, capEnd: true });
  attach(context, context.root, geometry, context.materials.airframe, 'fuselage');

  // The fairing behind the hood carries the spine down into the tail. The
  // fairing is solid, so it keeps the closed section.
  const spine = loft(
    SPINE_STATIONS.map((station) => canopyRing(station)),
    { closedProfile: true, capStart: true, capEnd: true },
  );
  attach(context, context.root, spine, context.materials.airframe, 'dorsal-spine');
}

function buildGunPorts(context: BuildContext): void {
  // Four MK 108 of 30 mm. Source: CONVENTIONS section 8, confidence firm.
  // Each barrel is a rod that starts inside the nose and leaves the skin where
  // the section becomes too small to hold it. The upper pair leaves the skin
  // about 150 mm ahead of the lower pair, which gives the staggered group of
  // four muzzles that the nose of the A-1a shows.
  const ports: Array<{ x: number; y: number; from: number; to: number }> = [
    { x: 0.085, y: 0.16, from: 0.1, to: 0.62 },
    { x: -0.085, y: 0.16, from: 0.1, to: 0.62 },
    { x: 0.09, y: -0.11, from: 0.26, to: 0.78 },
    { x: -0.09, y: -0.11, from: 0.26, to: 0.78 },
  ];
  for (let i = 0; i < ports.length; i++) {
    const port = ports[i];
    const geometry = rod(
      new Vector3(port.x, port.y, aft(port.from)),
      new Vector3(port.x, port.y, aft(port.to)),
      0.032,
      0.032,
    );
    attach(context, context.root, geometry, context.materials.darkMetal, `gun-port-${i}`);
  }
}

function buildWing(context: BuildContext, side: 1 | -1, pivots: Partial<Me262Pivots>): void {
  const plan = side === 1 ? WING_RIGHT : mirrorPlan(WING_RIGHT);
  const label = side === 1 ? 'right' : 'left';

  attach(
    context,
    context.root,
    liftingSurface(plan, WING_STATIONS, 14),
    context.materials.airframe,
    `wing-${label}`,
  );

  // --- Flaps. Two panels per side, split by the nacelle, one hinge line. ---
  const flapInner = surfacePoint(plan, FLAP_INNER[0], FLAP_HINGE);
  const flapOuter = surfacePoint(plan, FLAP_OUTER[1], FLAP_HINGE);
  const flapAxis = new Vector3().subVectors(flapOuter, flapInner);
  if (flapAxis.x < 0) flapAxis.negate();
  const flapPivot = makePivot(
    context.root,
    side === 1 ? 'flapRight' : 'flapLeft',
    flapInner,
    flapAxis,
    new Vector3(0, 1, 0),
  );
  for (const [index, range] of [FLAP_INNER, FLAP_OUTER].entries()) {
    const stations: SurfaceStation[] = [
      { span: range[0] + 0.02, front: FLAP_HINGE + 0.005, back: 1 },
      { span: range[1] - 0.02, front: FLAP_HINGE + 0.005, back: 1 },
    ];
    attach(
      context,
      flapPivot,
      liftingSurface(plan, stations, 8),
      context.materials.airframe,
      `flap-${label}-${index}`,
    );
  }

  // --- Aileron ---
  const aileronInner = surfacePoint(plan, AILERON_SPAN[0], AILERON_HINGE);
  const aileronOuter = surfacePoint(plan, AILERON_SPAN[1], AILERON_HINGE);
  const aileronAxis = new Vector3().subVectors(aileronOuter, aileronInner);
  if (aileronAxis.x < 0) aileronAxis.negate();
  const aileronPivot = makePivot(
    context.root,
    side === 1 ? 'aileronRight' : 'aileronLeft',
    aileronInner,
    aileronAxis,
    new Vector3(0, 1, 0),
  );
  attach(
    context,
    aileronPivot,
    liftingSurface(
      plan,
      [
        { span: AILERON_SPAN[0] + 0.02, front: AILERON_HINGE + 0.005, back: 1 },
        { span: AILERON_SPAN[1] - 0.02, front: AILERON_HINGE + 0.005, back: 1 },
      ],
      8,
    ),
    context.materials.airframe,
    `aileron-${label}`,
  );

  // --- Slat. The hinge sits below and behind the slat, so a positive turn
  // --- carries the slat forward and down along its arc.
  const slatInner = surfacePoint(plan, SLAT_SPAN[0], 0.35, -0.22);
  const slatOuter = surfacePoint(plan, SLAT_SPAN[1], 0.35, -0.22);
  const slatAxis = new Vector3().subVectors(slatOuter, slatInner);
  if (slatAxis.x > 0) slatAxis.negate();
  const slatPivot = makePivot(
    context.root,
    side === 1 ? 'slatRight' : 'slatLeft',
    slatInner,
    slatAxis,
    new Vector3(0, 1, 0),
  );
  attach(
    context,
    slatPivot,
    liftingSurface(
      plan,
      [
        { span: SLAT_SPAN[0] + 0.02, front: 0, back: SLAT_HINGE - 0.005 },
        { span: SLAT_SPAN[1] - 0.02, front: 0, back: SLAT_HINGE - 0.005 },
      ],
      8,
    ),
    context.materials.airframe,
    `slat-${label}`,
  );

  if (side === 1) {
    pivots.flapRight = flapPivot;
    pivots.aileronRight = aileronPivot;
    pivots.slatRight = slatPivot;
  } else {
    pivots.flapLeft = flapPivot;
    pivots.aileronLeft = aileronPivot;
    pivots.slatLeft = slatPivot;
  }
}

function buildTail(context: BuildContext, pivots: Partial<Me262Pivots>): void {
  attach(
    context,
    context.root,
    liftingSurface(FIN, FIN_STATIONS, 10),
    context.materials.airframe,
    'fin',
  );

  // The rudder hinge runs up the fin and aft. Its local +x points down, so a
  // positive turn carries the trailing edge to port.
  const rudderTop = surfacePoint(FIN, RUDDER_SPAN[1], RUDDER_HINGE);
  const rudderBottom = surfacePoint(FIN, RUDDER_SPAN[0], RUDDER_HINGE);
  const rudderPivot = makePivot(
    context.root,
    'rudder',
    rudderBottom,
    new Vector3().subVectors(rudderBottom, rudderTop),
    new Vector3(1, 0, 0),
  );
  attach(
    context,
    rudderPivot,
    liftingSurface(
      FIN,
      [
        { span: RUDDER_SPAN[0] + 0.02, front: RUDDER_HINGE + 0.005, back: 1 },
        { span: RUDDER_SPAN[1] - 0.02, front: RUDDER_HINGE + 0.005, back: 1 },
      ],
      8,
    ),
    context.materials.airframe,
    'rudder-surface',
  );
  pivots.rudder = rudderPivot;

  for (const side of [1, -1] as const) {
    const plan = side === 1 ? TAILPLANE_RIGHT : mirrorPlan(TAILPLANE_RIGHT);
    const label = side === 1 ? 'right' : 'left';
    attach(
      context,
      context.root,
      liftingSurface(plan, TAILPLANE_STATIONS, 10),
      context.materials.airframe,
      `tailplane-${label}`,
    );

    const inner = surfacePoint(plan, ELEVATOR_SPAN[0], ELEVATOR_HINGE);
    const outer = surfacePoint(plan, ELEVATOR_SPAN[1], ELEVATOR_HINGE);
    const axis = new Vector3().subVectors(outer, inner);
    if (axis.x < 0) axis.negate();
    const pivot = makePivot(
      context.root,
      side === 1 ? 'elevatorRight' : 'elevatorLeft',
      inner,
      axis,
      new Vector3(0, 1, 0),
    );
    attach(
      context,
      pivot,
      liftingSurface(
        plan,
        [
          { span: ELEVATOR_SPAN[0] + 0.02, front: ELEVATOR_HINGE + 0.005, back: 1 },
          { span: ELEVATOR_SPAN[1] - 0.02, front: ELEVATOR_HINGE + 0.005, back: 1 },
        ],
        8,
      ),
      context.materials.airframe,
      `elevator-${label}`,
    );
    if (side === 1) pivots.elevatorRight = pivot;
    else pivots.elevatorLeft = pivot;
  }
}

function buildNacelle(context: BuildContext, side: 1 | -1): void {
  const label = side === 1 ? 'right' : 'left';
  const place = (geometry: BufferGeometry): BufferGeometry => {
    geometry.translate(side * NACELLE_SPAN, NACELLE_Y, NACELLE_FRONT_Z);
    return geometry;
  };

  // The skin runs outside to the jet pipe and then forward inside the duct, so
  // both ends stay open and the lip is a fold.
  const skin = place(revolveZ(NACELLE_CONTOUR, 24, { closedRings: true }));
  attach(context, context.root, skin, context.materials.airframe, `nacelle-${label}`);

  // A bare metal band that wraps the intake lip. The band stands 5 mm proud of
  // the skin on the outside and 1 mm inside it, so the paint never shows
  // through.
  const lip = place(
    revolveZ(
      [
        new Vector2(0, 0.377),
        new Vector2(0.03, 0.397),
        new Vector2(0.09, 0.415),
        new Vector2(0.16, 0.421),
        new Vector2(0.16, 0.415),
        new Vector2(0.09, 0.409),
        new Vector2(0.03, 0.391),
        new Vector2(0, 0.371),
      ],
      24,
      { closedRings: true },
    ),
  );
  attach(context, context.root, lip, context.materials.bareMetal, `nacelle-lip-${label}`);

  const bullet = place(revolveZ(NACELLE_BULLET, 20, { capEnd: true }));
  attach(context, context.root, bullet, context.materials.bareMetal, `intake-bullet-${label}`);

  // The compressor face blocks the view straight through the engine. The last
  // contour point sits on the axis, so the ring closes into a flat disc.
  const face = place(
    revolveZ([new Vector2(1.3, 0.322), new Vector2(1.34, 0.322), new Vector2(1.34, 0)], 20),
  );
  attach(context, context.root, face, context.materials.darkMetal, `compressor-face-${label}`);

  const cone = place(revolveZ(NACELLE_CONE, 20, { capStart: true, capEnd: true }));
  attach(context, context.root, cone, context.materials.exhaust, `exhaust-cone-${label}`);

  // The last stretch of the duct wall runs hot, so it gets the tinted steel.
  const pipe = place(
    revolveZ(
      [
        new Vector2(2.6, 0.322),
        new Vector2(3.3, 0.318),
        new Vector2(NACELLE_LENGTH, 0.312),
        new Vector2(NACELLE_LENGTH, 0.3),
        new Vector2(3.3, 0.306),
        new Vector2(2.6, 0.31),
      ],
      24,
      { closedRings: true },
    ),
  );
  attach(context, context.root, pipe, context.materials.exhaust, `jet-pipe-${label}`);
}

function buildCanopy(context: BuildContext, pivots: Partial<Me262Pivots>): void {
  // Every part of the hood shell uses the crown arc alone. Read canopyRing.
  const crownRing = (station: CanopyStation): Vector3[] => canopyRing(station, true);

  // --- The windscreen is fixed. It holds the three flat front panels. ---
  // `capStart` closes the first ring. The fan runs over the sill chord, and
  // that fan IS the flat front panel of the windscreen. The shell keeps no
  // `capEnd`, because that disc would stand at station 3.74, straight in the
  // forward view of the pilot.
  const windscreen = loft(CANOPY_STATIONS.slice(0, 3).map(crownRing), {
    closedProfile: false,
    capStart: true,
  });
  attach(context, context.root, windscreen, context.materials.glass, 'windscreen-glass', {
    castShadow: false,
  });
  for (const [index, t] of [3.3, 3.72].entries()) {
    attach(
      context,
      context.root,
      canopyHoop(t, 0.022, 0.016),
      context.materials.frame,
      `windscreen-hoop-${index}`,
    );
  }

  // The A-1a carries a flat armored glass panel behind the windscreen.
  const armor = new BoxGeometry(0.4, 0.3, 0.028);
  armor.rotateX(0.52);
  armor.translate(0, 0.8, aft(3.52));
  attach(context, context.root, armor, context.materials.glass, 'windscreen-armor', {
    castShadow: false,
  });

  // --- The hood hinges on the starboard sill. Local +x points forward along
  // --- that sill, so a positive turn lifts the port edge and carries the hood
  // --- over to starboard.
  const pivot = makePivot(
    context.root,
    'canopy',
    new Vector3(0.315, 0.6, aft(4.2)),
    new Vector3(0, 0, -1),
    new Vector3(0, 1, 0),
  );

  // The hood carries no `capStart` for the same reason as the windscreen. Its
  // `capEnd` sits at station 5.05, behind the head of the pilot, where the
  // dorsal spine meets the hood.
  const hood = loft(CANOPY_STATIONS.slice(2).map(crownRing), {
    closedProfile: false,
    capEnd: true,
  });
  attach(context, pivot, hood, context.materials.glass, 'canopy-glass', { castShadow: false });

  for (const [index, t] of [3.76, 4.45, 5.02].entries()) {
    attach(
      context,
      pivot,
      canopyHoop(t, 0.022, 0.016),
      context.materials.frame,
      `canopy-hoop-${index}`,
    );
  }

  // Sill rails and one rail along the crown.
  for (const side of [1, -1] as const) {
    const rail = new BoxGeometry(0.05, 0.055, 1.28);
    rail.translate(side * 0.3, 0.63, aft(4.4));
    attach(context, pivot, rail, context.materials.frame, `canopy-sill-${side === 1 ? 'r' : 'l'}`);
  }
  const crown = new BoxGeometry(0.042, 0.04, 1.16);
  crown.translate(0, 1.03, aft(4.4));
  attach(context, pivot, crown, context.materials.frame, 'canopy-crown-rail');

  pivots.canopy = pivot;
}

function buildNoseGear(context: BuildContext, pivots: Partial<Me262Pivots>): void {
  const leg = makePivot(
    context.root,
    'gearNose',
    NOSE_TRUNNION,
    new Vector3(1, 0, 0),
    new Vector3(0, 1, 0),
  );
  // The gas cylinder hangs from the trunnion and it never moves. A gland at its
  // lower end marks where the piston leaves it.
  const cylinderEnd = alongLeg(NOSE_TRUNNION, NOSE_AXLE, NOSE_CYLINDER_LENGTH);
  attach(
    context,
    leg,
    rod(NOSE_TRUNNION, cylinderEnd, 0.06, 0.055),
    context.materials.bareMetal,
    'nose-cylinder',
  );
  attach(
    context,
    leg,
    rod(
      alongLeg(NOSE_TRUNNION, NOSE_AXLE, NOSE_CYLINDER_LENGTH - 0.035),
      cylinderEnd,
      0.068,
      0.068,
    ),
    context.materials.darkMetal,
    'nose-gland',
  );

  // Everything below hangs from the slider, so the whole lower leg shortens.
  const slider = makeSlider(leg, NOSE_TRUNNION, NOSE_AXLE);
  const pistonHead = alongLeg(NOSE_TRUNNION, NOSE_AXLE, NOSE_PISTON_HEAD);
  attach(
    context,
    slider.node,
    rod(pistonHead, NOSE_AXLE, 0.042, 0.042),
    context.materials.bareMetal,
    'nose-piston',
  );

  // A short torque link on the front of the leg. It rides on the piston, and it
  // still reaches the cylinder at full extension.
  const linkCenter = alongLeg(NOSE_TRUNNION, NOSE_AXLE, 0.145 + 0.17);
  const link = new BoxGeometry(0.02, 0.34, 0.05);
  link.translate(0, linkCenter.y, NOSE_AXLE.z - 0.075);
  attach(context, slider.node, link, context.materials.bareMetal, 'nose-scissor');

  const wheel = makePivot(
    slider.node,
    'wheelNose',
    NOSE_AXLE,
    new Vector3(-1, 0, 0),
    new Vector3(0, 1, 0),
  );
  const tire = wheelGeometry(NOSE_WHEEL_RADIUS, NOSE_WHEEL_HALF_WIDTH, 0.09);
  tire.translate(NOSE_AXLE.x, NOSE_AXLE.y, NOSE_AXLE.z);
  attach(context, wheel, tire, context.materials.rubber, 'nose-wheel');

  // The bay door hinges on the starboard edge of the bay and drops away.
  const doorHinge = new Vector3(0.17, -0.585, aft(2.95));
  const door = makePivot(
    context.root,
    'gearDoorNose',
    doorHinge,
    new Vector3(0, 0, 1),
    new Vector3(0, 1, 0),
  );
  const doorGeometry = new BoxGeometry(0.34, 0.025, 1.2);
  doorGeometry.translate(0, -0.585, aft(3.55));
  attach(context, door, doorGeometry, context.materials.airframe, 'nose-gear-door');

  pivots.gearNose = leg;
  pivots.wheelNose = wheel;
  pivots.gearDoorNose = door;
  context.sliders.nose = slider;
}

function buildMainGear(context: BuildContext, side: 1 | -1, pivots: Partial<Me262Pivots>): void {
  const label = side === 1 ? 'right' : 'left';
  const trunnion = new Vector3(side * MAIN_TRUNNION.x, MAIN_TRUNNION.y, MAIN_TRUNNION.z);
  const axle = new Vector3(side * MAIN_AXLE.x, MAIN_AXLE.y, MAIN_AXLE.z);

  // The trunnion of the right leg turns about a forward axis, and the trunnion
  // of the left leg turns about an aft axis. Both give an inboard retract for a
  // positive angle.
  const leg = makePivot(
    context.root,
    side === 1 ? 'gearRight' : 'gearLeft',
    trunnion,
    new Vector3(0, 0, -side),
    new Vector3(0, 1, 0),
  );
  // The gas cylinder hangs from the trunnion and it never moves. A gland at its
  // lower end marks where the piston leaves it, and the side brace lands just
  // above that gland.
  const cylinderEnd = alongLeg(trunnion, axle, MAIN_CYLINDER_LENGTH);
  attach(
    context,
    leg,
    rod(trunnion, cylinderEnd, 0.082, 0.074),
    context.materials.bareMetal,
    `main-cylinder-${label}`,
  );
  attach(
    context,
    leg,
    rod(alongLeg(trunnion, axle, MAIN_CYLINDER_LENGTH - 0.04), cylinderEnd, 0.092, 0.092),
    context.materials.darkMetal,
    `main-gland-${label}`,
  );

  const brace = rod(
    new Vector3(side * 0.72, -0.09, MAIN_TRUNNION.z),
    new Vector3(side * 1.13, -0.4, MAIN_TRUNNION.z - 0.09),
    0.035,
    0.03,
  );
  attach(context, leg, brace, context.materials.bareMetal, `main-brace-${label}`);

  // The piston and the wheel hang from the slider, so the leg shortens under
  // load and the axle stays on the end of the piston.
  const slider = makeSlider(leg, trunnion, axle);
  const pistonHead = alongLeg(trunnion, axle, MAIN_PISTON_HEAD);
  attach(
    context,
    slider.node,
    rod(pistonHead, axle, 0.058, 0.058),
    context.materials.bareMetal,
    `main-piston-${label}`,
  );

  const wheel = makePivot(
    slider.node,
    side === 1 ? 'wheelRight' : 'wheelLeft',
    axle,
    new Vector3(-1, 0, 0),
    new Vector3(0, 1, 0),
  );
  const tire = wheelGeometry(MAIN_WHEEL_RADIUS, MAIN_WHEEL_HALF_WIDTH, 0.12);
  tire.translate(axle.x, axle.y, axle.z);
  attach(context, wheel, tire, context.materials.rubber, `main-wheel-${label}`);

  // The bay door hinges on the outboard edge of the bay and drops away. The
  // hinge axis runs aft on the right wing and forward on the left wing, which
  // keeps a positive angle as the open direction on both sides.
  const doorHinge = new Vector3(side * 1.58, -0.105, 0.11);
  const door = makePivot(
    context.root,
    side === 1 ? 'gearDoorRight' : 'gearDoorLeft',
    doorHinge,
    new Vector3(0, 0, side),
    new Vector3(0, 1, 0),
  );
  const doorGeometry = new BoxGeometry(0.98, 0.025, 1.02);
  doorGeometry.translate(side * 1.09, -0.105, 0.11);
  attach(context, door, doorGeometry, context.materials.airframe, `main-gear-door-${label}`);

  if (side === 1) {
    pivots.gearRight = leg;
    pivots.wheelRight = wheel;
    pivots.gearDoorRight = door;
    context.sliders.right = slider;
  } else {
    pivots.gearLeft = leg;
    pivots.wheelLeft = wheel;
    pivots.gearDoorLeft = door;
    context.sliders.left = slider;
  }
}

function buildAerials(context: BuildContext): void {
  // Radio mast behind the hood.
  const mast = new BoxGeometry(0.022, 0.3, 0.09);
  mast.rotateX(-0.2);
  mast.translate(0, 0.83, aft(6.05));
  attach(context, context.root, mast, context.materials.frame, 'radio-mast');

  // Pitot boom on the right wing, outboard of the slat.
  const pitotRoot = surfacePoint(WING_RIGHT, 6.12, 0.05);
  const pitot = rod(
    pitotRoot.clone(),
    pitotRoot.clone().add(new Vector3(0, 0, -0.62)),
    0.018,
    0.012,
  );
  attach(context, context.root, pitot, context.materials.bareMetal, 'pitot-boom');
}

// ---------------------------------------------------------------------------
// The public builder
// ---------------------------------------------------------------------------

/**
 * Build the exterior model of the Me 262 A-1a.
 *
 * The caller adds `root` to a scene and drives the pivots. The caller must call
 * `dispose` when the model leaves the scene.
 */
export function createMe262Model(): Me262Model {
  const root = new Object3D();
  root.name = 'me262';

  const materials = createModelMaterialSet();
  const context: BuildContext = { root, materials, geometries: [], sliders: {} };
  const pivots: Partial<Me262Pivots> = {};

  buildFuselage(context);
  buildGunPorts(context);
  buildWing(context, 1, pivots);
  buildWing(context, -1, pivots);
  buildTail(context, pivots);
  buildNacelle(context, 1);
  buildNacelle(context, -1);
  buildCanopy(context, pivots);
  buildNoseGear(context, pivots);
  buildMainGear(context, 1, pivots);
  buildMainGear(context, -1, pivots);
  buildAerials(context);

  const complete = pivots as Me262Pivots;
  const sliders = context.sliders as GearSliders;

  function setGearCompression(nose: number, left: number, right: number): void {
    setSlider(sliders.nose, nose);
    setSlider(sliders.left, left);
    setSlider(sliders.right, right);
  }

  function reset(): void {
    // Neutral is zero on every pivot, with no exception. That gives the spawn
    // state on the runway: controls faired, flaps up, slats retracted, gear
    // down and locked, gear doors closed and flush, and the hood closed.
    // A gear cycle drives the doors through ME262_POSE.gearDoorOpen.
    for (const pivot of Object.values(complete)) pivot.rotation.set(0, 0, 0);
    // The gear is the one part that does not rest at zero. The aircraft spawns
    // parked, so all three legs carry their share of the weight.
    setGearCompression(
      ME262_GEAR_STATIC_COMPRESSION,
      ME262_GEAR_STATIC_COMPRESSION,
      ME262_GEAR_STATIC_COMPRESSION,
    );
  }

  function dispose(): void {
    for (const geometry of context.geometries) geometry.dispose();
    context.geometries.length = 0;
    disposeModelMaterialSet(materials);
    root.clear();
  }

  reset();

  return { root, pivots: complete, setGearCompression, reset, dispose };
}

/**
 * Height of the ground line below the model origin, in m. A ground handling
 * module can place the model with `position.y = -GROUND_Y` to stand it on a
 * flat surface. The value follows from the wheel radii and the leg lengths at
 * the static stroke, and it puts the fin tip 3.83 m above the ground, which
 * matches the corrected height of CONVENTIONS section 8.
 */
export const ME262_GROUND_CLEARANCE = -GROUND_Y;

/** Overall length of the model, in m. Kept so a caller can size a bounding box. */
export const ME262_LENGTH = LENGTH;
