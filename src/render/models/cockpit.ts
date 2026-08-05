/**
 * Virtual cockpit of the Messerschmitt Me 262 A-1a, and the Revi 16B sight.
 *
 * The module builds the interior from primitives, in the same style as the
 * exterior model of src/render/models/me262.ts. It holds no physics and no
 * animation. Bead b37 puts live faces and needles on the gauge discs, and
 * src/main.ts drives the named pivots.
 *
 * This module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4.
 *
 *
 * 1. ORIENTATION AND ORIGIN
 *
 * The cockpit shares the frame and the origin of the exterior model.
 *
 *   nose      -> model -z
 *   starboard -> model +x
 *   up        -> model +y
 *   origin    -> the center of gravity, 5.76 m aft of the nose tip
 *
 * `aft(t)` turns a distance aft of the nose tip into a model z value, exactly
 * as the exterior model does. The two files must agree on CG_OFFSET_FROM_NOSE,
 * so this file imports that constant instead of writing the number again.
 *
 *
 * 2. THE EYE POINT IS AN INPUT, NOT AN OUTPUT
 *
 * src/render/cameras.ts puts the eye of the pilot at COCKPIT_EYE_FORWARD and
 * COCKPIT_EYE_UP, which is 4.15 m aft of the nose tip and 0.82 m above the
 * model center line. This file imports both values and builds around them. It
 * never moves the eye.
 *
 * Three numbers of the exterior model bound the interior at that station:
 *
 *   The armored glass panel fills y = 0.663 to 0.937 over the stations 3.433
 *   to 3.607, so the panel and the glare shield must stay under it.
 *   The hood sill sits at y = 0.56 and the hood crown reaches y = 1.03.
 *   The fuselage skin gives a half width of 0.28 m at y = 0.65 and 0.41 m at
 *   y = 0.20, so the panel is narrow at the top and wide at the bottom.
 *
 * The camera near plane is 0.3 m. Every part therefore stands at least 0.35 m
 * from the eye point. The seat back and the gunsight body are the two parts
 * that come closest.
 *
 *
 * 3. GAUGE LAYOUT AND ITS REFERENCE
 *
 * The layout follows the instrument list of the "Pilot's Handbook for Me-262
 * A-1" together with photographs of the restored A-1a panels. The handbook
 * names the instruments but carries no dimensioned panel drawing, so the
 * position of each dial is a RECONSTRUCTION with a confidence of estimate.
 *
 * The panel holds three groups, which is the arrangement every photograph
 * shows:
 *
 *   Left and center, the flight group. Airspeed, artificial horizon and
 *   altimeter on the top row. Variometer, turn and slip, and the repeater
 *   compass on the second row. Fuel contents, clock and the AFN 2 homing
 *   indicator on the bottom row.
 *
 *   Right, the engine group. Two columns, one per engine, with the left engine
 *   in the left column. Rotor speed on the top row, gas temperature on the
 *   second row, and the fuel and oil pressure gauge on the bottom row.
 *
 * The flight instruments use an 80 mm case and the engine instruments use a
 * 57 mm case, which are the two standard Luftwaffe sizes.
 *
 *
 * 4. PIVOT SIGN CONVENTIONS. THIS IS THE CONTRACT WITH BEAD b37 AND main.ts
 *
 * Every pivot is an `Object3D` with an identity local transform. Its parent
 * carries the position and the base orientation, so a caller may write
 * `pivot.rotation.x` without loss of the base frame.
 *
 *   stick
 *     local axes are the model axes. local +x is starboard, +y is up, +z aft.
 *     The pivot sits on the floor, under the seat pan front edge.
 *     `rotation.x` POSITIVE pulls the grip AFT, which is the nose up command.
 *     `rotation.z` POSITIVE moves the grip to PORT, which is the left roll
 *     command. A right roll therefore needs a negative value.
 *     ME262_COCKPIT_TRAVEL.stickPitch and .stickRoll give the travel.
 *
 *   throttleLeft, throttleRight
 *     local +x points to PORT. local +y is up. local +z points FORWARD.
 *     `rotation.x` POSITIVE swings the lever FORWARD, which OPENS the engine.
 *     Zero is the idle stop. ME262_COCKPIT_TRAVEL.throttle is full open.
 *
 *   pedalLeft, pedalRight
 *     local axes are the model axes. The pedal hangs below the pivot.
 *     `rotation.x` POSITIVE pushes that pedal FORWARD, away from the pilot.
 *     A right rudder command therefore needs pedalRight positive and pedalLeft
 *     negative. ME262_COCKPIT_TRAVEL.pedal gives the travel of one pedal.
 *
 * No pivot uses `rotation.y`.
 *
 *
 * 5. HOW THE RETICLE FLOATS AT INFINITY
 *
 * A reflector sight collimates the reticle. The pilot sees the mark along a
 * direction that the SIGHT fixes, and the direction does not change when the
 * head moves. A mark painted on the glass behaves the other way, because a
 * head movement of 50 mm across a glass 0.43 m away swings the mark by 0.116
 * rad, which is 6.7 deg.
 *
 * The model gives the same effect with geometry alone. `reticle` is a child of
 * the cockpit root, and it sits RETICLE_RANGE meters ahead along the sight
 * axis. Every dimension of the mark is scaled by that range, so the mark holds
 * its angular size. The same head movement of 50 mm now swings the mark by
 * 5e-5 rad, which is 0.003 deg, and no eye reads that. The mark therefore
 * stays on the target while the head moves, which is what collimation gives.
 *
 * The reticle material writes no depth and reads no depth, and it adds its
 * light instead of covering what is behind. A real reticle is reflected light
 * that reaches the eye on top of the view, so the ground never hides it. The
 * material also switches the fog off, because 1 km of fog would erase a mark
 * that is only drawn at 1 km for the geometry.
 *
 *
 * 6. WHEN THE COCKPIT IS DRAWN
 *
 * The interior is invisible from outside the aircraft, so it must not cost a
 * draw call in the other three views. Two rules hold that:
 *
 *   src/main.ts calls `createMe262Cockpit` only when the pilot first enters
 *   the cockpit view. A flight that never uses that view builds no interior.
 *   `setVisible(false)` clears `visible` on the root. Three.js then skips the
 *   whole subtree in the render list and in the shadow pass.
 *
 *
 * 7. NO WORK AROUND FOR THE HOOD SHELL
 *
 * This file once held two depth only plates and a list of two exterior hoops
 * to hide. Both worked around a CLOSED hood shell in
 * src/render/models/me262.ts. Bead henri-flight-sim-37m opened that shell at
 * the sill, so the plates and the list are gone. Do not put them back.
 */

import type { Material } from 'three/webgpu';
import {
  AdditiveBlending,
  BoxGeometry,
  BufferGeometry,
  CatmullRomCurve3,
  CircleGeometry,
  Color,
  CylinderGeometry,
  DoubleSide,
  ExtrudeGeometry,
  Mesh,
  MeshBasicNodeMaterial,
  MeshStandardNodeMaterial,
  Object3D,
  Quaternion,
  RingGeometry,
  Shape,
  SphereGeometry,
  TubeGeometry,
  Vector2,
  Vector3,
} from 'three/webgpu';
import { color, float } from 'three/tsl';

import { lerp } from '@/math/tables';
import { DEG } from '@/math/units';
import { COCKPIT_EYE_FORWARD, COCKPIT_EYE_UP } from '@/render/cameras';

import { CG_OFFSET_FROM_NOSE } from './me262';

// ---------------------------------------------------------------------------
// Public contract
// ---------------------------------------------------------------------------

/** Named hinge points of the interior. Read section 4 before you use one. */
export interface Me262CockpitPivots {
  stick: Object3D;
  throttleLeft: Object3D;
  throttleRight: Object3D;
  pedalLeft: Object3D;
  pedalRight: Object3D;
}

export interface Me262Cockpit {
  root: Object3D;
  /**
   * The face disc of every gauge, by name. Each value is the `Mesh` of a flat
   * disc. Its local frame is the frame of the dial: the origin sits at the
   * center of the face, local +x runs to the right of the dial, local +y runs
   * to the top of the dial, and local +z points at the pilot. A needle that
   * bead b37 hangs on the disc therefore turns about local -z for a clockwise
   * sweep, and a canvas texture arrives the right way up with no extra turn.
   */
  gauges: Record<string, Object3D>;
  pivots: Me262CockpitPivots;
  /** The collimated mark of the gunsight. Read section 5. */
  reticle: Object3D;
  /** Shows or hides the whole interior. Read section 6. */
  setVisible(v: boolean): void;
  dispose(): void;
}

/**
 * Full travel of each pivot, in radians. A caller multiplies its command,
 * which runs from -1 to 1, by the value below. Every value is an estimate from
 * a cockpit photograph, because no handbook gives a control travel.
 */
export const ME262_COCKPIT_TRAVEL = {
  /** Stick, nose up to nose down. */
  stickPitch: 0.30,
  /** Stick, port to starboard. */
  stickRoll: 0.26,
  /** Throttle, idle stop to full open stop. */
  throttle: 0.62,
  /** One rudder pedal, from the neutral position. */
  pedal: 0.24,
} as const;

/**
 * Distance of the reticle plane ahead of the eye, in meters. Read section 5.
 * The value only sets the parallax, because every dimension of the mark scales
 * with it. One kilometer leaves an error of 0.003 deg for a 50 mm head move.
 */
const RETICLE_RANGE = 1000;

// ---------------------------------------------------------------------------
// Stations and heights
// ---------------------------------------------------------------------------

/** Distance aft of the nose tip, turned into a model z coordinate. */
function aft(distanceFromNose: number): number {
  return distanceFromNose - CG_OFFSET_FROM_NOSE;
}

/** The eye of the pilot in model coordinates. src/render/cameras.ts owns it. */
const EYE = new Vector3(0, COCKPIT_EYE_UP, aft(CG_OFFSET_FROM_NOSE - COCKPIT_EYE_FORWARD));

/** Floor of the cockpit, in m. The heel line of the pilot sits on it. */
const FLOOR_Y = -0.30;

/** Top of the seat pan, in m. The eye stands 0.80 m above it, as a seated eye does. */
const SEAT_PAN_Y = 0.02;

/** Station of the seat back plane, and of the front edge of the seat pan. */
const SEAT_BACK_T = 4.45;
const SEAT_FRONT_T = 4.05;

/** Height of the top of the side consoles, in m. */
const CONSOLE_Y = 0.28;

/**
 * Center of the instrument panel face, and the angle it leans.
 *
 * The panel leans its top forward by PANEL_TILT, so the face normal points up
 * and aft at the eye. The center sits 0.66 m from the eye. The top edge then
 * stands 16.8 deg below the horizon and the bottom edge 48.3 deg below it,
 * which is the geometry of every single seat fighter of the period.
 */
const PANEL_CENTER = new Vector3(0, 0.425, aft(3.585));
const PANEL_TILT = -17.8 * DEG;

/** Half height of the panel plate, in m. */
const PANEL_HALF_HEIGHT = 0.21;

/**
 * Half width of the panel plate at the top edge and at the bottom edge, in m.
 * The fuselage section closes in toward the deck, so a rectangle would cut
 * through the skin at the top corners. Read section 2.
 */
const PANEL_HALF_WIDTH_TOP = 0.29;
const PANEL_HALF_WIDTH_BOTTOM = 0.36;

/** Station of the reflector glass of the Revi 16B. */
const SIGHT_GLASS_T = 3.745;

/**
 * Body of the Revi 16B, as a station range, a radius and a height, in m.
 *
 * The body must stay AFT of station 3.607, where the armored glass of the
 * exterior model ends, and it must stay 0.3 m from the eye, which is the near
 * plane. The band between the two is narrow, so the body is short. The center
 * line sits 0.105 m under the eye, so the body blocks the view from 16 deg
 * below the horizon downward, which is where the glare shield already sits.
 */
const SIGHT_BODY_FRONT_T = 3.62;
const SIGHT_BODY_BACK_T = 3.77;
const SIGHT_BODY_RADIUS = 0.033;
const SIGHT_BODY_Y = 0.715;

// ---------------------------------------------------------------------------
// Hood sections
// ---------------------------------------------------------------------------

/**
 * One station of the hood, as src/render/models/me262.ts writes it.
 *
 * DUPLICATED from CANOPY_STATIONS of that file. The exterior model does not
 * export the table, and CONVENTIONS section 4 gives no shared home for it. The
 * interior frames must hug the glass of the exterior, so the two tables must
 * stay equal. A change there needs the same change here.
 */
interface HoodStation {
  t: number;
  halfWidth: number;
  height: number;
  sillY: number;
  sharpness: number;
}

const HOOD_STATIONS: HoodStation[] = [
  { t: 3.28, halfWidth: 0.235, height: 0.3, sillY: 0.56, sharpness: 1.45 },
  { t: 3.55, halfWidth: 0.285, height: 0.42, sillY: 0.56, sharpness: 1.3 },
  { t: 3.74, halfWidth: 0.31, height: 0.46, sillY: 0.56, sharpness: 1 },
  { t: 4.2, halfWidth: 0.315, height: 0.475, sillY: 0.56, sharpness: 0.88 },
  { t: 4.72, halfWidth: 0.3, height: 0.445, sillY: 0.555, sharpness: 0.88 },
  { t: 5.05, halfWidth: 0.255, height: 0.36, sillY: 0.55, sharpness: 0.92 },
];

/** Blend the hood table at one station. */
function hoodAt(t: number): HoodStation {
  const list = HOOD_STATIONS;
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
 * A point on the crown of the hood. `u` runs from 0 at the starboard sill,
 * over the top at 0.5, to 1 at the port sill. `inset` pulls the point inside
 * the glass, which is where an interior frame member sits.
 */
function hoodPoint(station: HoodStation, u: number, inset: number): Vector3 {
  const angle = u * Math.PI;
  const shape = (value: number): number =>
    Math.sign(value) * Math.pow(Math.abs(value), station.sharpness);
  const halfWidth = station.halfWidth - inset;
  const height = station.height - inset;
  return new Vector3(
    halfWidth * shape(Math.cos(angle)),
    station.sillY + height * Math.abs(shape(Math.sin(angle))),
    aft(station.t),
  );
}

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

/**
 * Paint of the interior. A Luftwaffe cockpit of 1944 is RLM 66 black grey,
 * which is darker than the RLM 02 of a wheel well. Confidence firm.
 */
const RLM_66_BLACK_GREY = 0x4b4e4d;

/**
 * Light that every interior surface gives off on its own.
 *
 * The fuselage closes over the cockpit and the sun casts a shadow over the
 * whole interior, so only the sky map lights it. A surface that faces inboard
 * or down then reads as pure black, and a black surface carries no shape. This
 * small emission stands for the light that a real cockpit bounces off its own
 * walls, which no direct light model gives. It is an ESTIMATE, tuned by eye
 * until the side wall keeps its shape in the picture.
 */
const INTERIOR_BOUNCE = 0x141719;

/** Every material the interior needs. */
interface CockpitMaterials {
  /** Painted structure: floor, walls, consoles, and the panel plate. */
  paint: MeshStandardNodeMaterial;
  /** Instrument bezels and other small dark satin metal. */
  bezel: MeshStandardNodeMaterial;
  /** The face of a dial. Bead b37 replaces this material or its color node. */
  gaugeFace: MeshStandardNodeMaterial;
  /** Bare steel of the stick shaft, the levers, and the pedals. */
  steel: MeshStandardNodeMaterial;
  /** Black rubber and bakelite of a grip or a knob. */
  grip: MeshStandardNodeMaterial;
  /** Seat cushion and the shoulder pad. */
  leather: MeshStandardNodeMaterial;
  /** Glass of the gunsight reflector. */
  sightGlass: MeshStandardNodeMaterial;
  /** The collimated mark. Read section 5. */
  reticle: MeshBasicNodeMaterial;
}

/** Gain of the reticle color. The mark must stay bright after tone mapping. */
const RETICLE_GAIN = 4;

function createCockpitMaterials(): CockpitMaterials {
  const paint = new MeshStandardNodeMaterial({
    name: 'cockpit-paint',
    color: new Color(RLM_66_BLACK_GREY),
    roughness: 0.86,
    metalness: 0.04,
    emissive: new Color(INTERIOR_BOUNCE),
  });

  const bezel = new MeshStandardNodeMaterial({
    name: 'cockpit-bezel',
    // A high metalness with a dark color reads as pure black wherever the sky
    // map gives no reflection, and a black surface carries no shape. The value
    // stays low for that reason.
    color: new Color(0x2b2e31),
    roughness: 0.46,
    metalness: 0.3,
    emissive: new Color(0x101315),
  });

  const gaugeFace = new MeshStandardNodeMaterial({
    name: 'cockpit-gauge-face',
    color: new Color(0x1b1e22),
    roughness: 0.9,
    metalness: 0,
    emissive: new Color(0x0a0c0e),
  });

  const steel = new MeshStandardNodeMaterial({
    name: 'cockpit-steel',
    color: new Color(0x8d9296),
    roughness: 0.4,
    metalness: 0.85,
    emissive: new Color(0x0c0e10),
  });

  const grip = new MeshStandardNodeMaterial({
    name: 'cockpit-grip',
    color: new Color(0x232527),
    roughness: 0.88,
    metalness: 0.06,
    emissive: new Color(0x0d0f11),
  });

  const leather = new MeshStandardNodeMaterial({
    name: 'cockpit-leather',
    color: new Color(0x574a3c),
    roughness: 0.95,
    metalness: 0,
    emissive: new Color(0x121013),
  });

  const sightGlass = new MeshStandardNodeMaterial({
    name: 'cockpit-sight-glass',
    color: new Color(0x9fb7a8),
    roughness: 0.04,
    metalness: 0,
    transparent: true,
    opacity: 0.22,
    side: DoubleSide,
  });

  // The mark adds its light and it reads no depth, so nothing hides it. The
  // fog must stay off, because the mark is drawn 1 km away for the geometry.
  const reticle = new MeshBasicNodeMaterial({
    name: 'cockpit-reticle',
    transparent: true,
    blending: AdditiveBlending,
    depthTest: false,
    depthWrite: false,
    side: DoubleSide,
    fog: false,
  });
  reticle.colorNode = color(0xff8c1a).mul(float(RETICLE_GAIN));

  return { paint, bezel, gaugeFace, steel, grip, leather, sightGlass, reticle };
}

function disposeCockpitMaterials(set: CockpitMaterials): void {
  set.paint.dispose();
  set.bezel.dispose();
  set.gaugeFace.dispose();
  set.steel.dispose();
  set.grip.dispose();
  set.leather.dispose();
  set.sightGlass.dispose();
  set.reticle.dispose();
}

// ---------------------------------------------------------------------------
// Build helpers
// ---------------------------------------------------------------------------

interface BuildContext {
  root: Object3D;
  materials: CockpitMaterials;
  geometries: BufferGeometry[];
  gauges: Record<string, Object3D>;
}

/**
 * Hang one geometry under `parent`. The geometry must already sit in the LOCAL
 * frame of `parent`, so the mesh keeps an identity transform.
 *
 * The interior needs no model space attributes, because no interior material
 * reads the camouflage or the panel lines. The geometry therefore goes to the
 * mesh as it is.
 */
function add(
  context: BuildContext,
  parent: Object3D,
  geometry: BufferGeometry,
  material: Material,
  name: string,
  position?: Vector3,
): Mesh {
  const mesh = new Mesh(geometry, material);
  mesh.name = name;
  if (position !== undefined) mesh.position.copy(position);
  // The interior sits inside a closed fuselage. It casts no useful shadow, and
  // a shadow pass over it only costs time.
  mesh.castShadow = false;
  mesh.receiveShadow = true;
  parent.add(mesh);
  context.geometries.push(geometry);
  return mesh;
}

/**
 * Build a hinge. The function returns the inner object, which holds an
 * identity transform. A caller writes `rotation.x` or `rotation.z` on it.
 *
 * `turnY` turns the local frame about the model +y axis. A half turn puts
 * local +x on model -x and local +z on model -z, which is what the throttle
 * levers need. Read section 4.
 */
function makePivot(parent: Object3D, name: string, origin: Vector3, turnY = 0): Object3D {
  const mount = new Object3D();
  mount.name = `${name}-mount`;
  mount.position.copy(origin);
  mount.rotation.y = turnY;
  parent.add(mount);

  const pivot = new Object3D();
  pivot.name = name;
  mount.add(pivot);
  return pivot;
}

/** Build a straight rod between two points, with a cap at each end. */
function rod(from: Vector3, to: Vector3, radius: number, sides = 10): BufferGeometry {
  const direction = new Vector3().subVectors(to, from);
  const length = direction.length();
  const geometry = new CylinderGeometry(radius, radius, length, sides, 1, false);
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

/** Build a box of the given size, centered on `center`. */
function slab(center: Vector3, width: number, height: number, depth: number): BufferGeometry {
  const geometry = new BoxGeometry(width, height, depth);
  geometry.translate(center.x, center.y, center.z);
  return geometry;
}

/**
 * Build a tapered tube on the model z axis, between two stations. `y` puts the
 * center line above the model center line.
 */
function taperZ(
  frontT: number,
  backT: number,
  radiusFront: number,
  radiusBack: number,
  y: number,
  sides = 14,
): BufferGeometry {
  const length = Math.abs(backT - frontT);
  // The cylinder stands on +y with the top at +height / 2, so a quarter turn
  // about x puts the top on +z, which is aft.
  const geometry = new CylinderGeometry(radiusBack, radiusFront, length, sides, 1, false);
  geometry.rotateX(Math.PI / 2);
  geometry.translate(0, y, aft((frontT + backT) / 2));
  return geometry;
}

/** Build a smooth tube through a list of points. */
function tube(points: Vector3[], radius: number, steps: number, sides = 6): BufferGeometry {
  return new TubeGeometry(new CatmullRomCurve3(points), steps, radius, sides, false);
}

/** Build one frame member that follows the crown of the hood at station `t`. */
function hoodFrame(t: number, inset: number, radius: number): BufferGeometry {
  const station = hoodAt(t);
  const points: Vector3[] = [];
  for (let i = 0; i <= 12; i++) points.push(hoodPoint(station, i / 12, inset));
  return tube(points, radius, 24, 6);
}

/** Build a closed outline with a round at every corner. */
function roundedShape(corners: Vector2[], radius: number): Shape {
  const shape = new Shape();
  const count = corners.length;
  const scratchIn = new Vector2();
  const scratchOut = new Vector2();
  for (let i = 0; i < count; i++) {
    const previous = corners[(i + count - 1) % count];
    const current = corners[i];
    const next = corners[(i + 1) % count];
    scratchIn.subVectors(current, previous).normalize();
    scratchOut.subVectors(next, current).normalize();
    const startX = current.x - scratchIn.x * radius;
    const startY = current.y - scratchIn.y * radius;
    const endX = current.x + scratchOut.x * radius;
    const endY = current.y + scratchOut.y * radius;
    if (i === 0) shape.moveTo(startX, startY);
    else shape.lineTo(startX, startY);
    shape.quadraticCurveTo(current.x, current.y, endX, endY);
  }
  shape.closePath();
  return shape;
}

// ---------------------------------------------------------------------------
// The instrument panel
// ---------------------------------------------------------------------------

/** One dial of the panel. `x` and `y` sit in the local frame of the panel. */
interface GaugePlace {
  name: string;
  x: number;
  y: number;
  /** Radius of the visible face, in m. */
  face: number;
}

/** Radius of the face of an 80 mm case and of a 57 mm case, in m. */
const LARGE_FACE = 0.041;
const SMALL_FACE = 0.03;

/** How far the bezel stands proud of the panel plate, in m. */
const BEZEL_DEPTH = 0.016;

/** Width of the bezel ring outside the face, in m. */
const BEZEL_WIDTH = 0.009;

/**
 * The panel. Read section 3 of the module comment for the reference and for
 * the confidence of each position.
 *
 * The three rows sit 0.116 m apart, which clears two 80 mm bezels. The flight
 * group fills the left three columns and the engine group fills the right two.
 */
const GAUGE_PLACES: GaugePlace[] = [
  // --- Flight group, top row ---
  { name: 'airspeed', x: -0.235, y: 0.118, face: LARGE_FACE },
  { name: 'artificialHorizon', x: -0.12, y: 0.118, face: LARGE_FACE },
  { name: 'altimeter', x: -0.005, y: 0.118, face: LARGE_FACE },
  // --- Flight group, second row ---
  { name: 'variometer', x: -0.235, y: 0.002, face: LARGE_FACE },
  { name: 'turnSlip', x: -0.12, y: 0.002, face: LARGE_FACE },
  { name: 'compass', x: -0.005, y: 0.002, face: LARGE_FACE },
  // --- Flight group, bottom row ---
  { name: 'fuel', x: -0.235, y: -0.125, face: LARGE_FACE },
  { name: 'clock', x: -0.12, y: -0.125, face: SMALL_FACE },
  { name: 'homing', x: -0.005, y: -0.125, face: SMALL_FACE },
  // --- Engine group. The left column reads the left engine. ---
  { name: 'rpmLeft', x: 0.145, y: 0.118, face: SMALL_FACE },
  { name: 'rpmRight', x: 0.245, y: 0.118, face: SMALL_FACE },
  { name: 'gasTemperatureLeft', x: 0.145, y: 0.002, face: SMALL_FACE },
  { name: 'gasTemperatureRight', x: 0.245, y: 0.002, face: SMALL_FACE },
  { name: 'enginePressureLeft', x: 0.145, y: -0.125, face: SMALL_FACE },
  { name: 'enginePressureRight', x: 0.245, y: -0.125, face: SMALL_FACE },
];

/** Build one dial: a raised bezel ring and a flat face disc inside it. */
function buildGauge(context: BuildContext, panel: Object3D, place: GaugePlace): void {
  const outer = place.face + BEZEL_WIDTH;

  // The wall of the bezel. Its normals point away from the axis, which is the
  // side the pilot sees.
  const wall = new CylinderGeometry(outer, outer, BEZEL_DEPTH, 20, 1, true);
  wall.rotateX(Math.PI / 2);
  wall.translate(place.x, place.y, BEZEL_DEPTH / 2);
  add(context, panel, wall, context.materials.bezel, `bezel-wall-${place.name}`);

  const rim = new RingGeometry(place.face, outer, 20);
  rim.translate(place.x, place.y, BEZEL_DEPTH);
  add(context, panel, rim, context.materials.bezel, `bezel-rim-${place.name}`);

  // The face keeps its own transform, so bead b37 can hang a needle on it and
  // the needle lands on the dial. Read the comment on `gauges`.
  const face = new CircleGeometry(place.face, 24);
  const mesh = add(
    context,
    panel,
    face,
    context.materials.gaugeFace,
    place.name,
    new Vector3(place.x, place.y, BEZEL_DEPTH - 0.003),
  );
  context.gauges[place.name] = mesh;
}

function buildPanel(context: BuildContext): void {
  const panel = new Object3D();
  panel.name = 'instrument-panel';
  panel.position.copy(PANEL_CENTER);
  panel.rotation.x = PANEL_TILT;
  context.root.add(panel);

  // The plate. The outline is a trapezoid, because the fuselage section closes
  // in toward the deck. Read section 2 of the module comment.
  const outline = roundedShape(
    [
      new Vector2(-PANEL_HALF_WIDTH_TOP, PANEL_HALF_HEIGHT),
      new Vector2(PANEL_HALF_WIDTH_TOP, PANEL_HALF_HEIGHT),
      new Vector2(PANEL_HALF_WIDTH_BOTTOM, -PANEL_HALF_HEIGHT),
      new Vector2(-PANEL_HALF_WIDTH_BOTTOM, -PANEL_HALF_HEIGHT),
    ],
    0.045,
  );
  const plate = new ExtrudeGeometry(outline, {
    depth: 0.022,
    bevelEnabled: false,
    curveSegments: 3,
  });
  // ExtrudeGeometry grows along +z from zero, so the front face must come back
  // to z = 0. Every dial then measures from the front face.
  plate.translate(0, 0, -0.022);
  add(context, panel, plate, context.materials.paint, 'panel-plate');

  for (const place of GAUGE_PLACES) buildGauge(context, panel, place);

  // A vertical rib splits the flight group from the engine group.
  add(
    context,
    panel,
    slab(new Vector3(0.07, 0, 0.004), 0.012, 2 * PANEL_HALF_HEIGHT - 0.02, 0.008),
    context.materials.bezel,
    'panel-divider',
  );

  // A row of toggle switches along the bottom edge of the panel.
  for (let i = 0; i < 6; i++) {
    const x = -0.155 + i * 0.062;
    const stem = new CylinderGeometry(0.004, 0.005, 0.024, 6);
    stem.rotateX(Math.PI / 2);
    stem.translate(x, -0.19, 0.012);
    add(context, panel, stem, context.materials.steel, `panel-switch-${i}`);
  }

  // The knee panel closes the gap under the main plate. Without it the pilot
  // looks past the panel, through the skin that the near plane clipped, and
  // out at the ground.
  add(
    context,
    panel,
    slab(new Vector3(0, -0.305, 0.03), 0.72, 0.21, 0.03),
    context.materials.paint,
    'knee-panel',
  );

  // The glare shield. It leans back over the panel and it holds the gunsight.
  // The aft edge stands at y = 0.685, which is aft of the armored glass, so
  // the two never meet. Read section 2 of the module comment.
  add(
    context,
    panel,
    slab(new Vector3(0, 0.215, 0.055), 0.58, 0.04, 0.13),
    context.materials.paint,
    'glare-shield',
  );
}

// ---------------------------------------------------------------------------
// The Revi 16B reflector gunsight
// ---------------------------------------------------------------------------

/**
 * Build the sight and its mark.
 *
 * The sight axis runs along model -z through the reflector glass at the height
 * of the eye, so the mark sits on the boresight of the airframe. Read section
 * 5 for the way the mark holds its direction while the head moves.
 */
function buildGunsight(context: BuildContext): Object3D {
  const sight = new Object3D();
  sight.name = 'revi-16b';
  context.root.add(sight);

  const bodyY = SIGHT_BODY_Y;

  // The bracket that carries the sight off the glare shield.
  add(
    context,
    sight,
    rod(new Vector3(0, 0.645, aft(3.63)), new Vector3(0, 0.7, aft(3.67)), 0.018, 8),
    context.materials.paint,
    'sight-bracket',
  );

  // The body. Its aft end tapers, so it does not read as a black disc on the
  // end of a pipe.
  add(
    context,
    sight,
    taperZ(SIGHT_BODY_FRONT_T, SIGHT_BODY_BACK_T, SIGHT_BODY_RADIUS, SIGHT_BODY_RADIUS, bodyY),
    context.materials.paint,
    'sight-body',
  );
  add(
    context,
    sight,
    taperZ(SIGHT_BODY_BACK_T, SIGHT_BODY_BACK_T + 0.05, SIGHT_BODY_RADIUS, 0.018, bodyY),
    context.materials.paint,
    'sight-body-taper',
  );

  // The lamp housing hangs under the front of the body.
  add(
    context,
    sight,
    slab(new Vector3(0, bodyY - 0.042, aft(3.68)), 0.058, 0.05, 0.085),
    context.materials.paint,
    'sight-lamp',
  );

  // Two collars break the plain tube, as the real body does.
  for (const [index, t] of [3.655, 3.755].entries()) {
    add(
      context,
      sight,
      taperZ(t - 0.006, t + 0.006, SIGHT_BODY_RADIUS + 0.005, SIGHT_BODY_RADIUS + 0.005, bodyY),
      context.materials.steel,
      `sight-collar-${index}`,
    );
  }

  // The neck that carries the glass off the top of the body.
  add(
    context,
    sight,
    slab(new Vector3(0, 0.752, aft(SIGHT_GLASS_T + 0.028)), 0.1, 0.03, 0.03),
    context.materials.paint,
    'sight-neck',
  );

  // The reflector glass. It leans its top toward the pilot by 41 deg, so the
  // beam of the lamp turns through about a right angle into the eye. The
  // center sits 0.018 m under the eye line, so the mark of section 5 falls
  // inside the glass with room above it.
  const glassTilt = 41 * DEG;
  const glassY = EYE.y - 0.018;
  const glass = new BoxGeometry(0.12, 0.12, 0.005);
  glass.rotateX(glassTilt);
  glass.translate(0, glassY, aft(SIGHT_GLASS_T));
  add(context, sight, glass, context.materials.sightGlass, 'sight-glass');

  // A light frame down each side of the glass, and a bar across its top.
  for (const side of [1, -1] as const) {
    const post = new BoxGeometry(0.007, 0.125, 0.009);
    post.rotateX(glassTilt);
    post.translate(side * 0.061, glassY, aft(SIGHT_GLASS_T));
    add(context, sight, post, context.materials.steel, `sight-glass-post-${side === 1 ? 'r' : 'l'}`);
  }
  const glassTop = new BoxGeometry(0.129, 0.008, 0.009);
  glassTop.rotateX(glassTilt);
  glassTop.translate(0, glassY + 0.06 * Math.cos(glassTilt), aft(SIGHT_GLASS_T) + 0.06 * Math.sin(glassTilt));
  add(context, sight, glassTop, context.materials.steel, 'sight-glass-top');

  // --- The collimated mark. Read section 5 of the module comment. ---------
  const reticle = new Object3D();
  reticle.name = 'reticle';
  reticle.position.set(0, EYE.y, EYE.z - RETICLE_RANGE);
  // The mark must draw after everything else and must never be hidden. A
  // renderOrder is not enough on its own, because the reversed depth buffer
  // flips that order. The material already reads no depth, so the order only
  // decides how the mark blends with the glass in front of it.
  reticle.renderOrder = 900;
  context.root.add(reticle);

  /** Turn an angle in milliradians into a size at the reticle range. */
  const mil = (value: number): number => (value / 1000) * RETICLE_RANGE;

  const ring = new RingGeometry(mil(48), mil(51), 72);
  add(context, reticle, ring, context.materials.reticle, 'reticle-ring');

  const pipper = new CircleGeometry(mil(1.6), 14);
  add(context, reticle, pipper, context.materials.reticle, 'reticle-pipper');

  // A bar to each side and a stem above. The three marks give the pilot a
  // reference for a deflection shot.
  for (const side of [1, -1] as const) {
    const bar = new BoxGeometry(mil(22), mil(3), mil(0.4));
    bar.translate(side * mil(33), 0, 0);
    add(context, reticle, bar, context.materials.reticle, `reticle-bar-${side === 1 ? 'r' : 'l'}`);
  }
  const stem = new BoxGeometry(mil(3), mil(22), mil(0.4));
  stem.translate(0, mil(33), 0);
  add(context, reticle, stem, context.materials.reticle, 'reticle-stem');

  return reticle;
}

// ---------------------------------------------------------------------------
// Structure, seat and consoles
// ---------------------------------------------------------------------------

/**
 * Floor, side walls and bulkheads.
 *
 * The side walls lean inboard, because the fuselage section closes in toward
 * the deck. A vertical wall would cut through the skin above y = 0.28.
 */
function buildStructure(context: BuildContext): void {
  add(
    context,
    context.root,
    slab(new Vector3(0, FLOOR_Y, aft(4.0)), 0.8, 0.02, 1.6),
    context.materials.paint,
    'cockpit-floor',
  );

  for (const side of [1, -1] as const) {
    // The wall reaches from the front bulkhead at station 3.16 to station 5.02,
    // so the two meet. The fuselage skin is a single sided surface and it lies
    // inside the 0.3 m near plane at this station, so an open side of the foot
    // well would show the ground through the skin.
    const wall = new BoxGeometry(0.02, 0.9, 1.86);
    // The wall runs from x = 0.412 at the floor to x = 0.324 at the sill. The
    // skin holds a half width of 0.49 at that height, so the wall stays inside.
    wall.rotateZ(side * 5.6 * DEG);
    wall.translate(side * 0.368, 0.13, aft(4.09));
    add(context, context.root, wall, context.materials.paint, `cockpit-wall-${side === 1 ? 'r' : 'l'}`);
  }

  // Frame ribs down each side. A cockpit wall of one flat plate reads as a
  // box, and the eye needs a repeat to judge the length of the space.
  for (const side of [1, -1] as const) {
    for (const [index, t] of [3.72, 4.16, 4.6].entries()) {
      const rib = new BoxGeometry(0.026, 0.78, 0.05);
      rib.rotateZ(side * 5.6 * DEG);
      rib.translate(side * 0.352, 0.17, aft(t));
      add(
        context,
        context.root,
        rib,
        context.materials.bezel,
        `cockpit-rib-${side === 1 ? 'r' : 'l'}-${index}`,
      );
    }
  }

  // The bulkhead ahead of the pedals closes the foot well.
  add(
    context,
    context.root,
    slab(new Vector3(0, 0.05, aft(3.16)), 0.78, 0.72, 0.02),
    context.materials.paint,
    'front-bulkhead',
  );

  // The armor plate behind the seat, and the deck behind it.
  add(
    context,
    context.root,
    slab(new Vector3(0, 0.16, aft(4.68)), 0.62, 0.94, 0.024),
    context.materials.paint,
    'rear-bulkhead',
  );
  add(
    context,
    context.root,
    slab(new Vector3(0, 0.5, aft(4.9)), 0.5, 0.02, 0.42),
    context.materials.paint,
    'rear-deck',
  );
}

function buildSeat(context: BuildContext): void {
  const seat = new Object3D();
  seat.name = 'seat';
  context.root.add(seat);

  const seatCenterT = (SEAT_FRONT_T + SEAT_BACK_T) / 2;
  add(
    context,
    seat,
    slab(new Vector3(0, SEAT_PAN_Y - 0.015, aft(seatCenterT)), 0.44, 0.03, SEAT_BACK_T - SEAT_FRONT_T),
    context.materials.paint,
    'seat-pan',
  );

  // The seat of the A-1a is a metal bucket that carries the parachute. The pad
  // stands for the parachute pack, which is what the pilot really sits on.
  add(
    context,
    seat,
    slab(new Vector3(0, SEAT_PAN_Y + 0.03, aft(seatCenterT)), 0.4, 0.06, 0.36),
    context.materials.leather,
    'seat-cushion',
  );

  // The back leans aft by 12 deg.
  const backTilt = 12 * DEG;
  const back = new BoxGeometry(0.44, 0.62, 0.03);
  back.rotateX(backTilt);
  back.translate(0, 0.32, aft(SEAT_BACK_T + 0.05));
  add(context, seat, back, context.materials.paint, 'seat-back');

  const backPad = new BoxGeometry(0.36, 0.5, 0.04);
  backPad.rotateX(backTilt);
  backPad.translate(0, 0.3, aft(SEAT_BACK_T + 0.005));
  add(context, seat, backPad, context.materials.leather, 'seat-back-pad');

  // The armored head rest above the seat back.
  const headrest = new BoxGeometry(0.24, 0.28, 0.026);
  headrest.rotateX(backTilt);
  headrest.translate(0, 0.78, aft(4.62));
  add(context, seat, headrest, context.materials.paint, 'seat-headrest');

  // Two side rails hold the seat on the floor.
  for (const side of [1, -1] as const) {
    add(
      context,
      seat,
      rod(
        new Vector3(side * 0.21, FLOOR_Y + 0.01, aft(SEAT_FRONT_T + 0.05)),
        new Vector3(side * 0.21, SEAT_PAN_Y, aft(SEAT_BACK_T - 0.05)),
        0.014,
        8,
      ),
      context.materials.steel,
      `seat-rail-${side === 1 ? 'r' : 'l'}`,
    );
  }

  // The shoulder straps run over the back and down to the pan.
  for (const side of [1, -1] as const) {
    const strap = new BoxGeometry(0.06, 0.46, 0.012);
    strap.rotateX(backTilt);
    strap.rotateZ(side * 7 * DEG);
    strap.translate(side * 0.1, 0.3, aft(SEAT_BACK_T - 0.02));
    add(context, seat, strap, context.materials.leather, `seat-strap-${side === 1 ? 'r' : 'l'}`);
  }
}

/**
 * The two side consoles, the throttle quadrant, and the small controls.
 *
 * The throttle sits on the left console, which is where every German fighter
 * of the period puts it.
 */
function buildConsoles(context: BuildContext, pivots: Partial<Me262CockpitPivots>): void {
  for (const side of [1, -1] as const) {
    const label = side === 1 ? 'right' : 'left';
    const body = new BoxGeometry(0.175, 0.24, 0.9);
    body.rotateZ(side * 5.6 * DEG);
    body.translate(side * 0.268, CONSOLE_Y - 0.12, aft(4.3));
    add(context, context.root, body, context.materials.paint, `console-${label}`);

    const top = new BoxGeometry(0.185, 0.016, 0.9);
    top.rotateZ(side * 5.6 * DEG);
    top.translate(side * 0.268, CONSOLE_Y, aft(4.3));
    add(context, context.root, top, context.materials.bezel, `console-top-${label}`);
  }

  // --- The right console carries switches and one small lever. -----------
  for (let i = 0; i < 5; i++) {
    const stem = new CylinderGeometry(0.005, 0.006, 0.03, 6);
    stem.translate(0.235 + (i % 2) * 0.055, CONSOLE_Y + 0.015, aft(4.05 + i * 0.13));
    add(context, context.root, stem, context.materials.steel, `right-switch-${i}`);
  }
  add(
    context,
    context.root,
    rod(
      new Vector3(0.27, CONSOLE_Y + 0.01, aft(4.62)),
      new Vector3(0.27, CONSOLE_Y + 0.15, aft(4.55)),
      0.008,
      8,
    ),
    context.materials.steel,
    'right-lever',
  );

  // --- The throttle quadrant on the left console. ------------------------
  add(
    context,
    context.root,
    slab(new Vector3(-0.3, CONSOLE_Y + 0.02, aft(4.06)), 0.12, 0.03, 0.28),
    context.materials.bezel,
    'throttle-base',
  );

  // A guard on the outboard side of the quadrant.
  add(
    context,
    context.root,
    slab(new Vector3(-0.365, CONSOLE_Y + 0.07, aft(4.06)), 0.012, 0.1, 0.28),
    context.materials.bezel,
    'throttle-guard',
  );

  for (const engine of ['Left', 'Right'] as const) {
    const outboard = engine === 'Left';
    const x = outboard ? -0.335 : -0.272;
    // The half turn about +y puts local +z on model -z, so a POSITIVE angle
    // swings the lever forward and opens the engine. Read section 4.
    const pivot = makePivot(
      context.root,
      `throttle${engine}`,
      new Vector3(x, CONSOLE_Y + 0.035, aft(4.14)),
      Math.PI,
    );
    add(
      context,
      pivot,
      rod(new Vector3(0, 0, 0), new Vector3(0, 0.19, 0.045), 0.009, 8),
      context.materials.steel,
      `throttle-lever-${engine.toLowerCase()}`,
    );
    const knob = new SphereGeometry(0.02, 12, 8);
    knob.translate(0, 0.2, 0.048);
    add(context, pivot, knob, context.materials.grip, `throttle-knob-${engine.toLowerCase()}`);
    if (engine === 'Left') pivots.throttleLeft = pivot;
    else pivots.throttleRight = pivot;
  }
}

// ---------------------------------------------------------------------------
// The control stick and the rudder pedals
// ---------------------------------------------------------------------------

function buildStick(context: BuildContext, pivots: Partial<Me262CockpitPivots>): void {
  const pivot = makePivot(context.root, 'stick', new Vector3(0, FLOOR_Y + 0.06, aft(3.95)));

  // The rubber boot over the base joint.
  const boot = new CylinderGeometry(0.035, 0.075, 0.11, 12);
  boot.translate(0, 0.055, 0);
  add(context, pivot, boot, context.materials.grip, 'stick-boot');

  const top = new Vector3(0, 0.4, 0.02);
  add(context, pivot, rod(new Vector3(0, 0.05, 0), top, 0.016, 10), context.materials.steel, 'stick-shaft');

  const gripTop = new Vector3(0, 0.53, 0.026);
  add(context, pivot, rod(top, gripTop, 0.027, 12), context.materials.grip, 'stick-grip');
  const cap = new SphereGeometry(0.027, 12, 8);
  cap.translate(gripTop.x, gripTop.y, gripTop.z);
  add(context, pivot, cap, context.materials.grip, 'stick-cap');

  // The trigger of the four MK 108 sits on the front of the grip.
  const trigger = new BoxGeometry(0.02, 0.05, 0.016);
  trigger.rotateX(-0.2);
  trigger.translate(0, 0.455, -0.024);
  add(context, pivot, trigger, context.materials.steel, 'stick-trigger');

  // The trigger guard stands around it.
  const guard = new BoxGeometry(0.05, 0.012, 0.03);
  guard.translate(0, 0.415, -0.024);
  add(context, pivot, guard, context.materials.grip, 'stick-trigger-guard');

  // The brake lever sits on the front of the grip, above the trigger.
  const brake = new BoxGeometry(0.038, 0.016, 0.026);
  brake.translate(0, 0.508, -0.022);
  add(context, pivot, brake, context.materials.steel, 'stick-brake-lever');

  pivots.stick = pivot;
}

function buildPedals(context: BuildContext, pivots: Partial<Me262CockpitPivots>): void {
  const hangerY = -0.02;
  const hangerT = 3.34;

  // The cross tube that carries both pedal hangers. It does not move.
  add(
    context,
    context.root,
    rod(
      new Vector3(-0.2, hangerY, aft(hangerT)),
      new Vector3(0.2, hangerY, aft(hangerT)),
      0.014,
      8,
    ),
    context.materials.steel,
    'pedal-cross-tube',
  );

  for (const side of [1, -1] as const) {
    const label = side === 1 ? 'Right' : 'Left';
    const pivot = makePivot(
      context.root,
      `pedal${label}`,
      new Vector3(side * 0.135, hangerY, aft(hangerT)),
    );

    add(
      context,
      pivot,
      rod(new Vector3(0, 0, 0), new Vector3(0, -0.22, 0.012), 0.012, 8),
      context.materials.steel,
      `pedal-hanger-${label.toLowerCase()}`,
    );

    // The plate leans back, so the sole of the boot lies flat on it.
    const plate = new BoxGeometry(0.09, 0.11, 0.022);
    plate.rotateX(-20 * DEG);
    plate.translate(0, -0.245, 0.028);
    add(context, pivot, plate, context.materials.steel, `pedal-plate-${label.toLowerCase()}`);

    // A heel stop under the plate.
    const stop = new BoxGeometry(0.09, 0.02, 0.05);
    stop.translate(0, -0.294, 0.05);
    add(context, pivot, stop, context.materials.steel, `pedal-stop-${label.toLowerCase()}`);

    if (side === 1) pivots.pedalRight = pivot;
    else pivots.pedalLeft = pivot;
  }
}

// ---------------------------------------------------------------------------
// The hood frame, seen from inside
// ---------------------------------------------------------------------------

/**
 * The frame members of the hood.
 *
 * The Me 262 hood carries heavy framing, and the frame is a large part of what
 * makes the view read as a cockpit. The exterior model draws a thin band on
 * the OUTSIDE of the glass. This function adds the inner members, which are
 * the ones a pilot looks at.
 *
 * The two hood hoops belong to the sliding half of the hood. They hang on the
 * cockpit root here, because the hood stays shut in this build. A bead that
 * opens the hood must move them onto the canopy pivot of the exterior model.
 */
function buildHoodFrame(context: BuildContext): void {
  const frame = new Object3D();
  frame.name = 'hood-frame';
  context.root.add(frame);

  const inset = 0.014;
  const members: Array<{ t: number; radius: number; name: string }> = [
    { t: 3.32, radius: 0.026, name: 'windscreen-bow' },
    { t: 3.71, radius: 0.023, name: 'windscreen-aft-bow' },
    { t: 4.45, radius: 0.018, name: 'hood-hoop-mid' },
    { t: 5.0, radius: 0.018, name: 'hood-hoop-aft' },
  ];
  for (const member of members) {
    add(
      context,
      frame,
      hoodFrame(member.t, inset, member.radius),
      context.materials.paint,
      member.name,
    );
  }

  // The two posts that split the flat front panel from the side panels.
  for (const side of [1, -1] as const) {
    const start = side === 1 ? 0.3 : 0.7;
    const end = side === 1 ? 0.26 : 0.74;
    const points: Vector3[] = [];
    for (let k = 0; k <= 4; k++) {
      const u = k / 4;
      points.push(hoodPoint(hoodAt(lerp(3.33, 3.7, u)), lerp(start, end, u), inset));
    }
    add(
      context,
      frame,
      tube(points, 0.016, 12, 6),
      context.materials.paint,
      `windscreen-post-${side === 1 ? 'r' : 'l'}`,
    );
  }

  // The sill rail runs down each side, from the windscreen to the aft hoop.
  for (const side of [1, -1] as const) {
    const points: Vector3[] = [];
    for (let k = 0; k <= 8; k++) {
      const t = lerp(3.3, 5.02, k / 8);
      points.push(hoodPoint(hoodAt(t), side === 1 ? 0 : 1, inset));
    }
    add(
      context,
      frame,
      tube(points, 0.021, 28, 6),
      context.materials.paint,
      `hood-sill-${side === 1 ? 'r' : 'l'}`,
    );
  }
}

// ---------------------------------------------------------------------------
// The public builder
// ---------------------------------------------------------------------------

/**
 * Build the interior of the Me 262 A-1a and the Revi 16B sight.
 *
 * The caller adds `root` under the root of the exterior model, so the interior
 * takes the pose of the aircraft with no extra work. The caller must call
 * `dispose` when the interior leaves the scene.
 *
 * The interior starts HIDDEN. src/main.ts builds it on the first entry into
 * the cockpit view and then calls `setVisible`. Read section 6.
 */
export function createMe262Cockpit(): Me262Cockpit {
  const root = new Object3D();
  root.name = 'me262-cockpit';
  root.visible = false;

  const materials = createCockpitMaterials();
  const context: BuildContext = { root, materials, geometries: [], gauges: {} };
  const pivots: Partial<Me262CockpitPivots> = {};

  buildStructure(context);
  buildSeat(context);
  buildConsoles(context, pivots);
  buildPanel(context);
  buildStick(context, pivots);
  buildPedals(context, pivots);
  buildHoodFrame(context);
  const reticle = buildGunsight(context);

  const complete = pivots as Me262CockpitPivots;
  // Every control rests at its neutral position. The throttle rests at the
  // idle stop, which is zero, because the aircraft spawns with the engines off.
  for (const pivot of Object.values(complete)) pivot.rotation.set(0, 0, 0);

  return {
    root,
    gauges: context.gauges,
    pivots: complete,
    reticle,

    setVisible(v: boolean): void {
      root.visible = v;
    },

    dispose(): void {
      for (const geometry of context.geometries) geometry.dispose();
      context.geometries.length = 0;
      disposeCockpitMaterials(materials);
      root.clear();
    },
  };
}
