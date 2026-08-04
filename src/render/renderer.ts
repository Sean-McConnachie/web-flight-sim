/**
 * Renderer start up.
 *
 * This module builds the WebGPU renderer, the scene, and the camera. It picks
 * the backend, sets the tone mapping, and holds the resize handler.
 *
 * The module touches the browser, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * DEPTH BUFFER CHOICE
 *
 * The camera runs from 0.3 m to 60000 m. That is a range of 200000 to 1. A
 * normal depth buffer puts almost all of its precision near the camera, so the
 * far ground and the far terrain flicker.
 *
 * Three.js offers two answers, and this module uses the reversed depth buffer.
 * A reversed buffer maps the near plane to 1 and the far plane to 0. The float
 * depth format has its fine steps near 0, so the two curves cancel and the
 * relative precision stays almost constant over the whole range. It costs
 * nothing per fragment and it keeps early depth rejection.
 *
 * The other answer, the logarithmic depth buffer, writes the depth value in the
 * fragment shader. That write turns off early depth rejection on most hardware
 * and it costs speed.
 *
 * WebGPU always supports the reversed range. The WebGL 2 backend needs the
 * EXT_clip_control extension. This module probes for that extension before it
 * builds the renderer. If the extension is missing, the module asks for the
 * logarithmic buffer instead, because a slow correct image beats a fast broken
 * one. Three.js also turns the reversed buffer off by itself when the backend
 * cannot support it. Read `renderer.reversedDepthBuffer` for the real state.
 */

import {
  ACESFilmicToneMapping,
  PerspectiveCamera,
  Scene,
  SRGBColorSpace,
  WebGPUCoordinateSystem,
  WebGPURenderer,
} from 'three/webgpu';

import { config } from '@/core/config';

/** Vertical field of view, in degrees. */
const CAMERA_FOV = 55;

/** Near plane, in meters. It clears the nose of the aircraft. */
const CAMERA_NEAR = 0.3;

/** Far plane, in meters. The world is 40 km across, so the sky needs 60 km. */
const CAMERA_FAR = 60000;

/** Largest device pixel ratio the renderer uses. A 4K phone panel needs a cap. */
const MAX_PIXEL_RATIO = 2;

export interface RendererBundle {
  renderer: WebGPURenderer;
  scene: Scene;
  camera: PerspectiveCamera;
  canvas: HTMLCanvasElement;
  /** The backend that really started, after any fallback. */
  backend: 'webgpu' | 'webgl2';
  /** Match the drawing buffer and the camera aspect to the canvas. */
  setSize(): void;
  dispose(): void;
}

/**
 * Report whether this browser exposes WebGPU. The check only looks for the
 * entry point. A device request can still fail later, so `createRenderer`
 * also catches a failed start and falls back.
 */
export function isWebGPUAvailable(): boolean {
  if (typeof navigator === 'undefined') return false;
  // TypeScript does not ship a type for navigator.gpu without @webgpu/types,
  // which this project does not install. Read the field through a narrow cast.
  const gpu = (navigator as unknown as { gpu?: unknown }).gpu;
  return gpu !== undefined && gpu !== null;
}

/**
 * Report whether the WebGL 2 backend can move the depth range. The probe uses
 * a throwaway canvas, so the answer arrives before the renderer starts.
 */
function webgl2SupportsReversedDepth(): boolean {
  if (typeof document === 'undefined') return false;
  try {
    const probe = document.createElement('canvas');
    const gl = probe.getContext('webgl2');
    if (gl === null) return false;
    const extension: unknown = gl.getExtension('EXT_clip_control');
    const supported = extension !== null && extension !== undefined;
    gl.getExtension('WEBGL_lose_context');
    return supported;
  } catch {
    return false;
  }
}

/**
 * Build the renderer, the scene, and the camera, then wait for the backend to
 * start. The function throws when neither backend starts. The caller shows the
 * message.
 */
export async function createRenderer(canvas: HTMLCanvasElement): Promise<RendererBundle> {
  const wantWebGPU = isWebGPUAvailable();
  const reversedDepth = wantWebGPU || webgl2SupportsReversedDepth();

  const renderer = new WebGPURenderer({
    canvas,
    antialias: true,
    // The page paints a black background behind the canvas, so the drawing
    // buffer needs no alpha. An opaque buffer is faster.
    alpha: false,
    // Ask for the WebGL 2 backend at once when the browser has no WebGPU.
    // Without this flag the renderer tries WebGPU first and then falls back,
    // which prints an error in the console of every browser that lacks WebGPU.
    forceWebGL: !wantWebGPU,
    reversedDepthBuffer: reversedDepth,
    logarithmicDepthBuffer: !reversedDepth,
  });

  // The backend starts here. Every later call needs a started backend.
  await renderer.init();

  renderer.toneMapping = ACESFilmicToneMapping;
  renderer.toneMappingExposure = config.render.exposure;
  renderer.outputColorSpace = SRGBColorSpace;

  const scene = new Scene();

  const camera = new PerspectiveCamera(CAMERA_FOV, 1, CAMERA_NEAR, CAMERA_FAR);
  scene.add(camera);

  // The renderer swaps its backend when WebGPU fails to start, so read the
  // coordinate system after init. The WebGPU backend reports
  // WebGPUCoordinateSystem. The WebGL 2 backend reports WebGLCoordinateSystem.
  const backend: 'webgpu' | 'webgl2' =
    renderer.coordinateSystem === WebGPUCoordinateSystem ? 'webgpu' : 'webgl2';

  function setSize(): void {
    // The style sheet sizes the canvas, so read the size back from the canvas
    // and pass updateStyle as false. That keeps the renderer from fighting CSS.
    const width = canvas.clientWidth > 0 ? canvas.clientWidth : window.innerWidth;
    const height = canvas.clientHeight > 0 ? canvas.clientHeight : window.innerHeight;
    if (width === 0 || height === 0) return;

    renderer.setPixelRatio(Math.min(window.devicePixelRatio, MAX_PIXEL_RATIO));
    renderer.setSize(width, height, false);

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }

  const onResize = (): void => setSize();
  window.addEventListener('resize', onResize);

  setSize();

  function dispose(): void {
    window.removeEventListener('resize', onResize);
    renderer.dispose();
  }

  return {
    renderer,
    scene,
    camera,
    canvas,
    backend,
    setSize,
    dispose,
  };
}
