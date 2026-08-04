/**
 * Shared node materials for the models.
 *
 * The aircraft model uses this module. The cockpit model and the building
 * models can use it later. Keep every paint color and every surface function
 * here, so that one change repaints the whole scene.
 *
 * This module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * MODEL SPACE ATTRIBUTES
 *
 * A camouflage pattern must stay fixed on the airframe. It must not move when
 * a control surface turns, and it must not move when the aircraft flies.
 * Neither the local position of a mesh nor the world position gives that.
 *
 * - The local position changes at every hinge, because a control surface is a
 *   child of a pivot and the pivot holds an offset and a turn.
 * - The world position changes at every frame, because the aircraft moves.
 *
 * So each geometry carries two extra attributes. `modelPosition` holds the
 * position of the vertex in the frame of the model root, and `modelNormal`
 * holds the normal of the vertex in the same frame. The build step writes both
 * values while the geometry still sits in model coordinates. Call
 * `bakeModelSpaceAttributes` at that moment. After that the builder may move
 * the geometry into the local frame of a pivot, because `applyMatrix4` changes
 * only the position, the normal, and the tangent.
 *
 *
 * COLOR SPACE
 *
 * Every color constant below is an sRGB hex value. `new Color(hex)` reads a
 * hex value as sRGB and converts it to the working color space, so the numbers
 * behave as a paint chart expects.
 */

import type { BufferGeometry, Node } from 'three/webgpu';
import { BufferAttribute, Color, DoubleSide, MeshStandardNodeMaterial, Vector3 } from 'three/webgpu';
import {
  abs,
  attribute,
  clamp,
  color,
  float,
  floor,
  fract,
  min,
  mix,
  mx_fractal_noise_float,
  mx_noise_float,
  smoothstep,
  step,
  vec3,
} from 'three/tsl';

// ---------------------------------------------------------------------------
// Model space attributes
// ---------------------------------------------------------------------------

/** Name of the vertex attribute that holds the position in model coordinates. */
export const MODEL_POSITION_ATTRIBUTE = 'modelPosition';

/** Name of the vertex attribute that holds the normal in model coordinates. */
export const MODEL_NORMAL_ATTRIBUTE = 'modelNormal';

const bakeScratch = new Vector3();

/**
 * Copy the position and the normal of every vertex into two extra attributes.
 *
 * Call this while the geometry still sits in model coordinates. The geometry
 * must already hold a `position` attribute and a `normal` attribute.
 */
export function bakeModelSpaceAttributes(geometry: BufferGeometry): void {
  const position = geometry.getAttribute('position');
  const normal = geometry.getAttribute('normal');
  const count = position.count;

  const modelPosition = new Float32Array(count * 3);
  const modelNormal = new Float32Array(count * 3);

  for (let i = 0; i < count; i++) {
    bakeScratch.fromBufferAttribute(position, i);
    modelPosition[i * 3 + 0] = bakeScratch.x;
    modelPosition[i * 3 + 1] = bakeScratch.y;
    modelPosition[i * 3 + 2] = bakeScratch.z;

    bakeScratch.fromBufferAttribute(normal, i);
    modelNormal[i * 3 + 0] = bakeScratch.x;
    modelNormal[i * 3 + 1] = bakeScratch.y;
    modelNormal[i * 3 + 2] = bakeScratch.z;
  }

  geometry.setAttribute(MODEL_POSITION_ATTRIBUTE, new BufferAttribute(modelPosition, 3));
  geometry.setAttribute(MODEL_NORMAL_ATTRIBUTE, new BufferAttribute(modelNormal, 3));
}

const modelPosition = attribute<'vec3'>(MODEL_POSITION_ATTRIBUTE, 'vec3');
const modelNormal = attribute<'vec3'>(MODEL_NORMAL_ATTRIBUTE, 'vec3');

// ---------------------------------------------------------------------------
// Paint colors
// ---------------------------------------------------------------------------

/**
 * Late war Luftwaffe paint. The values are the common modern renditions of the
 * RLM chart. A paint chip changes with the light and with the batch, so treat
 * each value as an estimate with a spread of about five percent.
 */
export const RLM_81_BROWN_VIOLET = 0x5f5642;
export const RLM_82_LIGHT_GREEN = 0x6f7a44;
export const RLM_76_LIGHT_BLUE = 0xa9b6ba;
export const RLM_66_BLACK_GREY = 0x3a3d3c;
export const RLM_02_GREY = 0x7d7f6c;
export const RLM_70_BLACK_GREEN = 0x2b3a2e;

/** Clean aluminium. The nacelle lips and the gear legs use this color. */
export const BARE_ALUMINIUM = 0x9aa0a4;

/** Cold steel of the exhaust cone, before the heat tint. */
export const COLD_STEEL = 0x8b8f93;

// ---------------------------------------------------------------------------
// Surface functions
// ---------------------------------------------------------------------------

/** Distance between the frame lines along the length of the aircraft, in m. */
const PANEL_PITCH_LENGTH = 0.78;

/** Distance between the stringer lines across the span, in m. */
const PANEL_PITCH_SPAN = 1.06;

/** Distance between the stringer lines up the side of the fuselage, in m. */
const PANEL_PITCH_HEIGHT = 0.55;

/** Half width of a panel line, in m. */
const PANEL_LINE_WIDTH = 0.014;

/**
 * Return 0 on a grid line and 1 away from a grid line. The grid lines sit at
 * every half step of `pitch`, measured along `coordinate`.
 */
function gridLine(coordinate: Node<'float'>, pitch: number): Node<'float'> {
  const distance = abs(fract(coordinate.div(pitch)).sub(0.5)).mul(pitch);
  return smoothstep(0, PANEL_LINE_WIDTH, distance);
}

/**
 * Return the panel line mask. The value is 0 on a panel line and 1 on clean
 * skin.
 *
 * The mask holds three grids. The first grid runs across the aircraft and
 * marks the frames. The second grid runs along the span and marks the wing
 * stringers. The third grid runs up the side and marks the fuselage stringers.
 * The model normal picks between the second grid and the third grid, because a
 * grid that cuts a surface at a shallow angle spreads into a wide smear.
 */
export function modelPanelLineMask(): Node<'float'> {
  const frames = gridLine(modelPosition.z, PANEL_PITCH_LENGTH);
  const spanwise = gridLine(modelPosition.x, PANEL_PITCH_SPAN);
  const vertical = gridLine(modelPosition.y, PANEL_PITCH_HEIGHT);

  // 1 on a surface that faces up or down. 0 on a surface that faces sideways.
  const flatness = smoothstep(0.35, 0.78, abs(modelNormal.y));
  return min(frames, mix(vertical, spanwise, flatness));
}

/**
 * Return a roughness value that changes from panel to panel.
 *
 * Real metal never holds one roughness over a whole airframe. Each panel comes
 * from a different sheet and each panel takes a different amount of polish. The
 * cell noise gives one constant value per panel. The fine noise breaks the flat
 * look inside a panel.
 */
export function modelPanelRoughness(base: number, spread: number): Node<'float'> {
  const cell = floor(modelPosition.z.div(PANEL_PITCH_LENGTH))
    .mul(0.61)
    .add(floor(modelPosition.x.div(PANEL_PITCH_SPAN)).mul(1.37))
    .add(floor(modelPosition.y.div(PANEL_PITCH_HEIGHT)).mul(2.11));
  const perPanel = mx_noise_float(vec3(cell, cell.mul(1.7), cell.mul(-0.6)));
  const fine = mx_noise_float(modelPosition.mul(5.5)).mul(0.4);
  return clamp(float(base).add(perPanel.add(fine).mul(spread)), 0.05, 1);
}

/**
 * Return the splinter camouflage color.
 *
 * The pattern comes from two straight ramps that run across the airframe at
 * two different angles. A step turns each ramp into a hard edge band. The
 * difference of the two bands gives an angular patch work, which is the look of
 * the late war splinter scheme. A slow noise bends the ramps a little, so the
 * patches do not repeat.
 *
 * The model normal chooses between the upper colors and the lower color. On the
 * side of the fuselage the normal points sideways, so the two schemes meet in a
 * soft band. A mottle of the darker upper color sits on top of that band, which
 * is the standard late war finish.
 */
export function modelSplinterCamouflage(): Node<'vec3'> {
  const p = modelPosition;
  const warp = mx_noise_float(p.mul(0.2)).mul(0.5);

  const rampA = p.z.mul(0.46).add(p.x.mul(0.23)).add(warp);
  const rampB = p.x.mul(0.4).sub(p.z.mul(0.19)).sub(warp.mul(0.7));
  const bandA = step(0.5, fract(rampA));
  const bandB = step(0.55, fract(rampB));
  // The two bands hold only 0 or 1, so the absolute difference is an XOR.
  const splinter = abs(bandA.sub(bandB));

  const upper = mix(color(RLM_81_BROWN_VIOLET), color(RLM_82_LIGHT_GREEN), splinter);
  const upperness = smoothstep(-0.14, 0.34, modelNormal.y);
  const base = mix(color(RLM_76_LIGHT_BLUE), upper, upperness);

  const sideness = float(1).sub(abs(modelNormal.y));
  const mottle = smoothstep(0.02, 0.5, mx_fractal_noise_float(p.mul(1.15), 3, 2, 0.5, 1));
  return mix(base, color(RLM_81_BROWN_VIOLET), mottle.mul(sideness).mul(0.55));
}

// ---------------------------------------------------------------------------
// The material set
// ---------------------------------------------------------------------------

/**
 * Every material that a model needs. One set serves one model. Two models may
 * share one set, because no material holds a per object value.
 */
export interface ModelMaterialSet {
  /** Painted airframe skin with splinter camouflage and panel lines. */
  airframe: MeshStandardNodeMaterial;
  /** Clean aluminium. The nacelle lips and the gear legs use it. */
  bareMetal: MeshStandardNodeMaterial;
  /** Heat tinted steel of the jet pipe and the exhaust cone. */
  exhaust: MeshStandardNodeMaterial;
  /** Near black metal of the gun ports and the engine faces. */
  darkMetal: MeshStandardNodeMaterial;
  /** Canopy glazing. */
  glass: MeshStandardNodeMaterial;
  /** Canopy frame and other small painted parts, in RLM 66. */
  frame: MeshStandardNodeMaterial;
  /** Tire rubber. */
  rubber: MeshStandardNodeMaterial;
  /** Wheel well and other interior surfaces, in RLM 02. */
  interior: MeshStandardNodeMaterial;
}

/** Build one material set. The caller must call `disposeModelMaterialSet`. */
export function createModelMaterialSet(): ModelMaterialSet {
  const panelMask = modelPanelLineMask();
  // A panel line reads as a narrow dark groove, not as a black stripe.
  const panelShade = mix(0.74, 1, panelMask);
  // A groove holds dirt, so it also reads as rougher than the skin beside it.
  const panelWear = float(1).sub(panelMask).mul(0.14);

  const airframe = new MeshStandardNodeMaterial({ name: 'model-airframe' });
  airframe.colorNode = modelSplinterCamouflage().mul(panelShade);
  airframe.roughnessNode = modelPanelRoughness(0.62, 0.13).add(panelWear);
  airframe.metalnessNode = float(0.05);

  const bareMetal = new MeshStandardNodeMaterial({ name: 'model-bare-metal' });
  bareMetal.colorNode = color(BARE_ALUMINIUM).mul(mix(0.82, 1, panelMask));
  bareMetal.roughnessNode = modelPanelRoughness(0.3, 0.17).add(panelWear);
  bareMetal.metalnessNode = float(1);

  // The jet pipe runs from straw at the front to blue and then to soot at the
  // back. The ramp uses the model z axis, which points aft.
  const exhaust = new MeshStandardNodeMaterial({ name: 'model-exhaust' });
  const heat = smoothstep(0.4, 2.1, modelPosition.z);
  const strawStage = mix(color(COLD_STEEL), color(0x8a6d3a), smoothstep(0, 0.55, heat));
  const blueStage = mix(strawStage, color(0x4a4a63), smoothstep(0.5, 0.85, heat));
  exhaust.colorNode = mix(blueStage, color(0x33302c), smoothstep(0.82, 1, heat));
  exhaust.roughnessNode = modelPanelRoughness(0.42, 0.2);
  exhaust.metalnessNode = float(0.95);

  const darkMetal = new MeshStandardNodeMaterial({
    name: 'model-dark-metal',
    color: new Color(0x1b1c1d),
    roughness: 0.5,
    metalness: 0.9,
    side: DoubleSide,
  });

  const glass = new MeshStandardNodeMaterial({
    name: 'model-glass',
    color: new Color(0x8fa6ab),
    roughness: 0.06,
    metalness: 0,
    transparent: true,
    opacity: 0.3,
    side: DoubleSide,
  });

  const frame = new MeshStandardNodeMaterial({ name: 'model-frame' });
  frame.colorNode = color(RLM_66_BLACK_GREY);
  frame.roughnessNode = modelPanelRoughness(0.68, 0.1);
  frame.metalnessNode = float(0.1);

  const rubber = new MeshStandardNodeMaterial({
    name: 'model-rubber',
    color: new Color(0x1f2021),
    roughness: 0.92,
    metalness: 0,
  });

  const interior = new MeshStandardNodeMaterial({
    name: 'model-interior',
    color: new Color(RLM_02_GREY),
    roughness: 0.8,
    metalness: 0.05,
    side: DoubleSide,
  });

  return { airframe, bareMetal, exhaust, darkMetal, glass, frame, rubber, interior };
}

/** Free the GPU resources of one material set. */
export function disposeModelMaterialSet(set: ModelMaterialSet): void {
  set.airframe.dispose();
  set.bareMetal.dispose();
  set.exhaust.dispose();
  set.darkMetal.dispose();
  set.glass.dispose();
  set.frame.dispose();
  set.rubber.dispose();
  set.interior.dispose();
}
