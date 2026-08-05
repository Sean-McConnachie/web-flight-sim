/**
 * The moving parts of a dial: the face disc, the needles, and the masked drum.
 *
 *
 * 1. WHERE THE PARTS HANG
 *
 * `createMe262Cockpit` of src/render/models/cockpit.ts hands over one `Object3D`
 * per gauge. That object is the flat face disc inside the bezel. Its local
 * frame is the frame of the dial: the origin is the middle of the face, local
 * +x runs to the right of the dial, local +y runs to the top, and local +z
 * points at the pilot.
 *
 * Every part this module builds becomes a CHILD of that disc, so the interior
 * keeps the position and the lean of the panel and no instrument module ever
 * writes a panel coordinate.
 *
 *
 * 2. THE DEPTH BUDGET, WHICH IS 3 MILLIMETERS, AND THE GAP THAT IS 0.6
 *
 * cockpit.ts puts the bezel front at 16 mm above the panel plate and the face
 * disc at 13 mm. A part that stands more than 3 mm off the face therefore
 * breaks out through the glass line of its own bezel. GAUGE_Z holds the whole
 * budget. Only the hub cap leaves it, by 0.4 mm, and a small disc at the
 * middle of a dial meets nothing out there.
 *
 * That budget is why the artificial horizon needs the squash of section 3.
 *
 * THE SMALLEST GAP BETWEEN TWO LAYERS IS 0.6 mm, AND THAT IS MEASURED, NOT
 * CHOSEN. The first build of this file put the painted face 0.3 mm in front of
 * the disc that cockpit.ts already draws. The two fought for the depth buffer,
 * and the fault does NOT look like depth fighting: whole dial faces come out in
 * dark radial wedges, and a DIFFERENT set of dials breaks in every frame,
 * because the wedges follow the 24 triangles of the disc underneath. It reads
 * as a lost texture. Moving the face to 1.0 mm cleared it at once.
 *
 * The depth buffer of this project is REVERSED, which CONVENTIONS section 6a
 * covers, and a reversed buffer holds most of its precision near the camera.
 * That is not enough here. The panel stands 0.66 m from the eye and the far
 * plane is 60 km, so keep the gaps.
 *
 *
 * 3. THE SQUASHED DRUM
 *
 * A gyro horizon shows a drum, not a needle. The drum turns about the pitch
 * axis behind a round aperture, so the horizon line moves as `R sin(pitch)`
 * and the ends of the scale crowd together, exactly as the real instrument
 * does. A flat card that slides up and down cannot do that.
 *
 * A true drum of radius R stands R proud of the face, which is 32 mm on an
 * 80 mm case. Section 2 allows 3 mm. `addDrum` therefore hangs the drum under
 * a mount that is SCALED on z alone.
 *
 * The order is what makes it work. Three.js builds the world matrix as
 * `parent * child`, so a point runs through the drum rotation FIRST and the
 * squash SECOND. The squash touches z alone, and the pitch rotation carries
 * the texture in y, so the horizon line still moves as `R sin(pitch)` to the
 * last digit. The bank rotation sits between the two, about z, and a scale on
 * z commutes with a rotation about z, so it is untouched as well.
 *
 * The pilot looks at the drum from the front, and from the front a drum and a
 * squashed drum give the same picture. Only a view along the panel would tell
 * the two apart, and no view of this simulator reaches that angle.
 *
 *
 * 4. THE DRUM TEXTURE AND WHERE ITS ZERO SITS
 *
 * `CylinderGeometry` runs its u coordinate around the circumference, with
 * `theta = u * thetaLength + thetaStart`, and it puts the vertex at
 * `x = R sin(theta)`, `z = R cos(theta)`. `addDrum` turns the cylinder a
 * quarter turn about z, which carries the cylinder axis onto dial +x. A point
 * of the drum then sits at dial `y = R sin(theta)` and dial `z = R cos(theta)`.
 *
 * The point at `theta = 0` therefore faces the pilot. Turn the drum by `phi`
 * about dial +x and that point moves to `y = R sin(-phi)`, while the point at
 * `theta = phi` arrives at the middle of the aperture. The texture is painted
 * with theta AS the pitch ladder value, so setting the drum rotation to the
 * pitch angle puts that pitch value under the fixed aircraft symbol. Nothing
 * else is needed.
 *
 * `addDrum` starts the sweep at minus half a turn, so `theta = 0` lands in the
 * MIDDLE of the texture and not on its seam. Canvas x therefore carries the
 * ladder angle, from -180 degrees at the left edge to +180 at the right, and
 * canvas y runs across the drum.
 *
 * This file touches the renderer. CONVENTIONS section 4 allows that under
 * src/ui. It holds no physics.
 */

import type { Material, Texture } from 'three/webgpu';
import {
  BufferGeometry,
  CircleGeometry,
  Color,
  CylinderGeometry,
  Mesh,
  MeshStandardNodeMaterial,
  Object3D,
  RingGeometry,
  Shape,
  ShapeGeometry,
} from 'three/webgpu';
import { texture as textureNode } from 'three/tsl';

import { faceTexture } from './draw';

// ---------------------------------------------------------------------------
// The depth budget of section 2, in meters above the face disc
// ---------------------------------------------------------------------------

export const GAUGE_Z = {
  /** The painted face. It covers the plain disc that cockpit.ts built. */
  face: 0.001,
  /** The middle of the horizon drum. */
  drum: 0.0014,
  /** A second needle, such as the kilometer hand of the altimeter. */
  lower: 0.0018,
  /** The ring that masks the drum, and a fixed index over a moving card. */
  mask: 0.0022,
  /** The main needle. */
  needle: 0.0026,
  /** The hub cap, the fixed aircraft symbol, and a sweep second hand. */
  hub: 0.0034,
} as const;

/**
 * How much light a face gives off on its own.
 *
 * The fuselage closes over the cockpit and the sun never reaches the panel, so
 * a face lit only by the sky map reads as black. A real panel carried its own
 * ultraviolet lamps for exactly that reason. The value is an ESTIMATE, tuned
 * by eye until the numerals stay readable from the eye point in level flight.
 */
const FACE_EMISSION = 0.55;

/**
 * How much light a needle gives off on its own.
 *
 * A needle needs MORE than a face. The face carries its tick marks under the
 * same light, so a needle at the light of the face reads as one more mark. The
 * value is an ESTIMATE, set from a measurement: at the eye point a printed
 * tick mark reads 0.68 on the screen, and this value puts a needle above it.
 *
 * A flat material with no light of its own reads 0.34, which is HALF of the
 * tick mark beside it. That was the first attempt and it was wrong.
 */
const NEEDLE_EMISSION = 0.8;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/** One needle. Every length is a fraction of the face radius. */
export interface NeedleSpec {
  readonly name: string;
  /** Radius the tip reaches. */
  readonly length: number;
  /** Length of the counterweight tail, behind the hub. */
  readonly tail: number;
  /** Half width of the blade at the hub. */
  readonly width: number;
  readonly color: number;
  /** Height above the face. Use one of GAUGE_Z. */
  readonly z: number;
}

/**
 * The tools one instrument module needs. Every call adds a child of the face
 * disc and records what it made, so `dispose` can give it all back.
 */
export interface GaugeParts {
  /** The face disc that cockpit.ts built. Every part is a child of it. */
  readonly face: Object3D;
  /** Radius of the visible face, m. */
  readonly radius: number;
  /** Paint the whole face with one canvas. */
  addFace(canvas: HTMLCanvasElement, z?: number): Mesh;
  /** Paint a ring with one canvas. The canvas covers the WHOLE face square. */
  addRing(canvas: HTMLCanvasElement, inner: number, outer: number, z: number): Mesh;
  /** Build one needle. It stands up the dial at an angle of zero. */
  addNeedle(spec: NeedleSpec): Mesh;
  /** Build the hub cap that hides the needle roots. */
  addHub(radius: number, color: number, z?: number): Mesh;
  /** Build a plain colored plate, in face fractions, with +y up the dial. */
  addPlate(name: string, shape: Shape | Shape[], color: number, z: number): Mesh;
  /** Build the squashed drum of section 3. It returns the drum mesh. */
  addDrum(canvas: HTMLCanvasElement, radius: number, parent: Object3D): Mesh;
  /** Build an empty pivot under the face, or under another part. */
  addPivot(name: string, parent?: Object3D): Object3D;
  dispose(): void;
}

// ---------------------------------------------------------------------------
// The builder
// ---------------------------------------------------------------------------

/** Read the face radius off the disc that cockpit.ts built. */
function readFaceRadius(face: Object3D): number {
  if (face instanceof Mesh) {
    const geometry: BufferGeometry = face.geometry;
    if (geometry.boundingSphere === null) geometry.computeBoundingSphere();
    const sphere = geometry.boundingSphere;
    if (sphere !== null && sphere.radius > 0) return sphere.radius;
  }
  // No gauge should reach this line. The value is the 80 mm case of cockpit.ts.
  return 0.041;
}

/**
 * The outline of one needle, in meters, standing up the dial from the hub.
 *
 * A NEGATIVE tail starts the blade away from the pivot, which the two pointers
 * of the AFN 2 need. The shoulder is then held above the base, because a
 * shoulder below it would fold the outline over itself and no triangulator
 * survives that.
 */
function needleShape(spec: NeedleSpec, radius: number): Shape {
  const length = spec.length * radius;
  const base = -spec.tail * radius;
  const w = spec.width * radius;
  const shoulder = Math.max(length * 0.14, base + length * 0.03);
  const shape = new Shape();
  shape.moveTo(-w * 0.8, base);
  shape.lineTo(w * 0.8, base);
  shape.lineTo(w, shoulder);
  shape.lineTo(w * 0.34, length * 0.86);
  shape.lineTo(0, length);
  shape.lineTo(-w * 0.34, length * 0.86);
  shape.lineTo(-w, shoulder);
  shape.closePath();
  return shape;
}

export function createGaugeParts(face: Object3D, name: string): GaugeParts {
  const radius = readFaceRadius(face);
  const geometries: BufferGeometry[] = [];
  const materials: Material[] = [];
  const textures: Texture[] = [];
  const added: Object3D[] = [];

  /** A material that shows one painted canvas and gives light of its own. */
  function paintedMaterial(canvas: HTMLCanvasElement, part: string): MeshStandardNodeMaterial {
    const map = faceTexture(canvas, `gauge-${name}-${part}`);
    textures.push(map);
    const material = new MeshStandardNodeMaterial({
      name: `gauge-${name}-${part}`,
      roughness: 0.88,
      metalness: 0,
    });
    material.colorNode = textureNode(map);
    material.emissiveNode = textureNode(map).mul(FACE_EMISSION);
    materials.push(material);
    return material;
  }

  /** A material for a needle or a plate. Read NEEDLE_EMISSION. */
  function flatMaterial(color: number, part: string): MeshStandardNodeMaterial {
    const base = new Color(color);
    const material = new MeshStandardNodeMaterial({
      name: `gauge-${name}-${part}`,
      color: base,
      roughness: 0.7,
      metalness: 0,
      emissive: base.clone().multiplyScalar(NEEDLE_EMISSION),
    });
    materials.push(material);
    return material;
  }

  function attach(mesh: Mesh, parent: Object3D): Mesh {
    mesh.castShadow = false;
    mesh.receiveShadow = false;
    parent.add(mesh);
    added.push(mesh);
    return mesh;
  }

  return {
    face,
    radius,

    addFace(canvas: HTMLCanvasElement, z = GAUGE_Z.face): Mesh {
      const geometry = new CircleGeometry(radius, 64);
      geometries.push(geometry);
      const mesh = new Mesh(geometry, paintedMaterial(canvas, 'face'));
      mesh.name = `gauge-face-${name}`;
      mesh.position.z = z;
      return attach(mesh, face);
    },

    addRing(canvas: HTMLCanvasElement, inner: number, outer: number, z: number): Mesh {
      // RingGeometry maps its uv from the OUTER radius, so a canvas that covers
      // the whole face square lands on the ring with no offset of its own.
      const geometry = new RingGeometry(inner * radius, outer * radius, 72, 1);
      geometries.push(geometry);
      const mesh = new Mesh(geometry, paintedMaterial(canvas, 'mask'));
      mesh.name = `gauge-mask-${name}`;
      mesh.position.z = z;
      return attach(mesh, face);
    },

    addNeedle(spec: NeedleSpec): Mesh {
      const geometry = new ShapeGeometry(needleShape(spec, radius), 2);
      geometries.push(geometry);
      const mesh = new Mesh(geometry, flatMaterial(spec.color, spec.name));
      mesh.name = `gauge-needle-${name}-${spec.name}`;
      mesh.position.z = spec.z;
      return attach(mesh, face);
    },

    addHub(hubRadius: number, color: number, z = GAUGE_Z.hub): Mesh {
      const geometry = new CircleGeometry(hubRadius * radius, 20);
      geometries.push(geometry);
      const mesh = new Mesh(geometry, flatMaterial(color, 'hub'));
      mesh.name = `gauge-hub-${name}`;
      mesh.position.z = z;
      return attach(mesh, face);
    },

    addPlate(plateName: string, shape: Shape | Shape[], color: number, z: number): Mesh {
      const geometry = new ShapeGeometry(shape, 8);
      geometry.scale(radius, radius, 1);
      geometries.push(geometry);
      const mesh = new Mesh(geometry, flatMaterial(color, plateName));
      mesh.name = `gauge-plate-${name}-${plateName}`;
      mesh.position.z = z;
      return attach(mesh, face);
    },

    addDrum(canvas: HTMLCanvasElement, drumRadius: number, parent: Object3D): Mesh {
      const r = drumRadius * radius;
      // Section 2 gives the whole budget. The drum must stand no more than
      // this far off the face, so the squash follows from it and not the other
      // way round. Half of the room to the mask leaves the 0.6 mm gap that
      // section 2 demands where the two overlap.
      const room = (GAUGE_Z.mask - GAUGE_Z.drum) * 0.5;
      const squash = room / r;

      const mount = new Object3D();
      mount.name = `gauge-drum-mount-${name}`;
      mount.position.z = GAUGE_Z.drum;
      mount.scale.set(1, 1, squash);
      parent.add(mount);
      added.push(mount);

      // The drum is as long as it is wide, so a banked drum still covers the
      // round aperture at every corner.
      // `thetaStart` of minus half a turn puts the ladder zero at the MIDDLE
      // of the texture instead of on its seam. A horizon line painted across
      // the seam would be cut in half.
      const geometry = new CylinderGeometry(r, r, 2.1 * r, 96, 1, true, -Math.PI, Math.PI * 2);
      // A quarter turn about z carries the cylinder axis onto dial +x. Read
      // section 4 for what that does to the texture.
      geometry.rotateZ(Math.PI / 2);
      geometries.push(geometry);

      const mesh = new Mesh(geometry, paintedMaterial(canvas, 'drum'));
      mesh.name = `gauge-drum-${name}`;
      mesh.castShadow = false;
      mesh.receiveShadow = false;
      mount.add(mesh);
      added.push(mesh);
      return mesh;
    },

    addPivot(pivotName: string, parent?: Object3D): Object3D {
      const pivot = new Object3D();
      pivot.name = `gauge-pivot-${name}-${pivotName}`;
      (parent ?? face).add(pivot);
      added.push(pivot);
      return pivot;
    },

    dispose(): void {
      for (const object of added) object.removeFromParent();
      added.length = 0;
      for (const geometry of geometries) geometry.dispose();
      geometries.length = 0;
      for (const material of materials) material.dispose();
      materials.length = 0;
      for (const map of textures) map.dispose();
      textures.length = 0;
    },
  };
}

/**
 * Point one needle at a clockwise dial angle.
 *
 * The needle mesh stands on local +y, and the face frame turns anticlockwise
 * about local +z, so a CLOCKWISE angle needs the minus sign. This is the only
 * line in the panel that carries it. Read section 1 of src/ui/gauges/dial.ts.
 */
export function pointNeedle(needle: Object3D, clockwiseAngle: number): void {
  needle.rotation.z = -clockwiseAngle;
}
