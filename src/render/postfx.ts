/**
 * Post processing chain.
 *
 * The chain draws the scene into a render target, works on that image, and
 * writes the result to the canvas. It replaces `renderer.render(scene, camera)`
 * in the frame loop.
 *
 * The module touches the renderer, so it lives under src/render. Read
 * docs/CONVENTIONS.md section 4. No physics belongs here.
 *
 *
 * WHICH NODES EXIST AT THIS VERSION
 *
 * Three.js 0.185.1 ships these display nodes under
 * `three/addons/tsl/display/`: AfterImage, AnaglyphPass, BilateralBlur,
 * BleachBypass, Bloom, boxBlur, ChromaticAberration, CRT, Denoise,
 * depthAwareBlend, DepthOfField, DotScreen, Film, FSR1, FXAA, GaussianBlur,
 * Godrays, GTAO, hashBlur, ImportanceSampledEnvironment, Lensflare, Lut3D,
 * MotionBlur, Outline, ParallaxBarrierPass, PixelationPass, radialBlur,
 * RecurrentDenoise, RetroPass, RGBShift, Sepia, Sharpen, SMAA, SobelOperator,
 * SSAAPass, SSGI, SSR, SSS, StereoCompositePass, StereoPass, TAAU,
 * TemporalReproject, TRAA and Transition.
 *
 * This chain uses GTAO for the ambient occlusion, SMAA and FXAA for the anti
 * aliasing, and Bloom for the glare. There is no `ao.js` file. The GTAO file
 * exports the function under the name `ao`.
 *
 *
 * WHERE THE TONE MAP HAPPENS
 *
 * src/render/renderer.ts sets `renderer.toneMapping` to ACES filmic. That is
 * the one place that names the curve and the exposure, and this module does
 * not change it.
 *
 * `RenderPipeline.render` sets `renderer.toneMapping` to `NoToneMapping` for
 * the whole time that it draws. So every render target inside the chain holds
 * linear high dynamic range color and no material tone maps its own output.
 * The chain then applies the curve one time, with `toneMapping(...)`, at the
 * point marked below. `outputColorTransform` is false, so the pipeline does not
 * add a second automatic tone map on top. The image is tone mapped exactly one
 * time on each path.
 *
 * When the quality is `off`, this module calls `renderer.render` and the
 * renderer applies its own tone mapping in the normal way. So both paths give
 * the same curve and the same exposure.
 *
 *
 * WHY THE ORDER IS NOT THE OBVIOUS ONE
 *
 * The chain runs: ambient occlusion, bloom, ACES tone map, anti aliasing,
 * vignette.
 *
 * Bloom must read linear high dynamic range color, because it selects the
 * bright part of the image by a threshold. After a tone map every value sits
 * below one and the threshold loses its meaning.
 *
 * SMAA and FXAA must run after the tone map. Both find an edge from the
 * difference between two colors, and both use a fixed threshold near 0.1. In
 * this scene the sun disc reaches a radiance near 300000 while sunlit grass
 * sits near 2. Every edge in that image passes a threshold of 0.1, so the
 * filter would mark the whole picture as an edge and blend a very bright pixel
 * into its dark neighbor. After the tone map the values sit in 0 to 1 and the
 * threshold means what its author meant. SMAA still runs before the sRGB
 * transfer, as its own header asks.
 *
 * A temporal filter such as TRAA would run before the bloom, because it works
 * on high dynamic range color and it needs the motion vectors of the frame.
 * This module does not use it. Read the note on `Quality` below.
 *
 *
 * THE REVERSED DEPTH BUFFER
 *
 * src/render/renderer.ts asks for a reversed depth buffer. The near plane then
 * maps to depth 1 and the far plane maps to depth 0.
 *
 * `PassNode` handles that. It gives its depth texture the float type when the
 * flag is set, and `Renderer` reverses the camera projection matrix, so the
 * inverse projection that GTAO uses stays correct.
 *
 * One line of GTAO does not handle it. `GTAONode` drops the background with
 * `depth.greaterThanEqual(1.0).discard()`, which is the far plane of a normal
 * buffer and the NEAR plane of a reversed buffer. With the reversed buffer
 * nothing is dropped, so GTAO runs on the sky, where the normal buffer holds
 * zero and `normalize` of a zero vector is not a number. This module therefore
 * masks the result itself. Read `backgroundMask` below.
 *
 *
 * THE AMBIENT OCCLUSION AT A DISTANCE
 *
 * GTAO looks for the horizon of the scene inside a sphere of `AO_RADIUS`
 * meters. It marches `AO_STEPS` samples out from the fragment, and it reads the
 * depth of the scene at the screen position of each sample. The first step of
 * the march is `AO_RADIUS / AO_STEPS` meters long.
 *
 * A step is a distance in the world, so its width on the screen falls with the
 * distance from the camera. When a step becomes shorter than one texel of the
 * occlusion target, the sample lands in the texel that it started from. The
 * march then reads its own depth again, which describes a surface that faces
 * the camera, and it reports occlusion where the scene is open.
 *
 * The ground is one large flat plane. At a grazing angle the depth of that
 * plane changes down the screen and not across it, so the false occlusion
 * follows the rows of the target. The pilot sees horizontal bands. A pilot
 * looks at the ground at a grazing angle on every approach and on every takeoff
 * roll, so this is the most common view in the simulator.
 *
 * The error is also large. On the far ground it took the occlusion factor to
 * 0.73, which darkened that part of the picture by more than a quarter, and it
 * darkened the WebGL 2 picture more than the WebGPU picture.
 *
 * `aoTrust` below removes the error. It measures the first step of the march in
 * texels of the occlusion target, and it fades the occlusion out where that
 * step no longer covers a texel. The measure is in texels and not in meters, so
 * it follows the drawing buffer size, the field of view, the radius, and the
 * resolution scale on its own. At a 1280 by 720 drawing buffer the occlusion
 * holds its full strength to 35 m and reaches zero at 58 m. A larger drawing
 * buffer carries it further.
 *
 * The chain keeps the occlusion where the march resolves the geometry, which is
 * the near field that holds the aircraft, the gear bays, and the walls of the
 * buildings. Measured at a grazing angle, the ground closer than 21 m keeps the
 * same occlusion to the last bit.
 *
 *
 * THE TWO BACKENDS
 *
 * The ambient occlusion runs on the WebGPU backend only. `GTAONode` sends its
 * projection to the shader in a `mat4` uniform, and a `mat4` uniform does not
 * reach the shader on the WebGL 2 backend of three.js 0.185.1. The pass then
 * works with a projection of zero and it writes an occlusion that has no
 * relation to the scene. Read `supportsAmbientOcclusion` below for the measured
 * numbers. The rest of the chain runs on both backends.
 *
 * For the same reason this module states the view depth of a fragment with
 * `perspectiveDepthToViewZ`, which needs the two planes of the camera and no
 * matrix. `getViewPosition` gives the same answer on WebGPU and zero on
 * WebGL 2.
 *
 *
 * GPU MEMORY
 *
 * The chain asks the scene pass for zero samples, so it holds no multisample
 * target. Without that line the scene target would take the four samples of
 * the renderer, which at a 3840 by 2160 drawing buffer with two color outputs
 * would need more than 500 MB on its own. The anti aliasing of this chain
 * replaces that multisample buffer.
 *
 * At `high` and at a 3840 by 2160 drawing buffer the chain holds about:
 * scene color and scene normal 133 MB, scene depth 33 MB, three SMAA targets
 * 200 MB, bloom mip chain about 22 MB, and the ambient occlusion target 2 MB.
 * At the more common 1920 by 1080 every number falls by four. `low` drops the
 * ambient occlusion and the three SMAA targets. `off` holds nothing. On the
 * WebGL 2 backend `high` also drops the scene normal target and the ambient
 * occlusion target, which saves about 68 MB at 3840 by 2160.
 */

import { ao } from 'three/addons/tsl/display/GTAONode.js';
import { bloom } from 'three/addons/tsl/display/BloomNode.js';
import { fxaa } from 'three/addons/tsl/display/FXAANode.js';
import { smaa } from 'three/addons/tsl/display/SMAANode.js';
import type { Node, PerspectiveCamera, Scene, WebGPURenderer } from 'three/webgpu';
import {
  ACESFilmicToneMapping,
  NoToneMapping,
  RenderPipeline,
  SRGBColorSpace,
  Vector2,
  WebGPUCoordinateSystem,
} from 'three/webgpu';
import {
  float,
  logarithmicDepthToViewZ,
  mix,
  mrt,
  normalView,
  output,
  pass,
  perspectiveDepthToViewZ,
  renderOutput,
  smoothstep,
  uniform,
  uv,
  vec4,
} from 'three/tsl';

/**
 * Radius of the ambient occlusion, in meters. It is a contact shadow, so it
 * only has to cover the gap under a wheel or the corner of a wall.
 */
const AO_RADIUS = 0.6;

/** Samples of the ambient occlusion. More samples cost more and look smoother. */
const AO_SAMPLES = 16;

/**
 * Steps that the occlusion march takes along one direction.
 *
 * GTAO turns its sample count into directions and steps. It takes 3 directions
 * while the count stays below 30, and it gives each direction
 * `(AO_SAMPLES + 2) / 3` steps. Step `j` sits at `AO_RADIUS * (j + 1) / steps`
 * meters from the fragment, so the first step of the march is the shortest one
 * and it sets the smallest distance that the march can resolve. Keep this value
 * in agreement with `AO_SAMPLES`.
 */
const AO_STEPS = 6;

/**
 * Resolution of the ambient occlusion, as a part of the drawing buffer. Half
 * on a side is a quarter of the pixels, and ambient occlusion is low frequency.
 */
const AO_RESOLUTION_SCALE = 0.5;

/**
 * Width of the first occlusion step, in texels of the occlusion target, at
 * which the chain uses the occlusion in full. Read the note on the occlusion
 * at a distance at the top of this file.
 */
const AO_TRUST_TEXELS = 1;

/**
 * Width of the first occlusion step, in texels of the occlusion target, at
 * which the chain drops the occlusion. Below this width the march reads the
 * center texel again for every sample, so its result carries no information.
 */
const AO_DROP_TEXELS = 0.6;

/** Smallest view depth the fade divides by, in meters. It only stops a zero. */
const AO_MIN_VIEW_DEPTH = 1e-4;

/**
 * Largest color value that enters the bloom, in linear radiance.
 *
 * The solar disc of src/render/sky.ts reaches a radiance near 300000. Without
 * a clamp the blur of that one point covers a large part of the sky in white.
 * The clamp keeps the sun as a bright glare and not as a white disc.
 */
const BLOOM_INPUT_CLAMP = 12;

/**
 * Bloom threshold, in linear radiance. A sunlit diffuse surface sits near 2 in
 * this scene, so a threshold of 2.5 leaves the ground and the aircraft skin
 * alone and passes only the sun and a bright metal highlight.
 */
const BLOOM_THRESHOLD = 2.5;

/** Bloom strength. A flight simulator needs a hint of glare, not a glow. */
const BLOOM_STRENGTH = 0.18;

/** Bloom radius, from 0 to 1. */
const BLOOM_RADIUS = 0.55;

/** Resolution of the bloom mip chain, as a part of the drawing buffer. */
const BLOOM_RESOLUTION_SCALE = 0.5;

/** Distance from the center of the image where the vignette starts, in uv. */
const VIGNETTE_START = 0.38;

/** Distance where the vignette reaches its full strength, in uv. */
const VIGNETTE_END = 0.78;

/** Largest part of the light the vignette takes away at the corner. */
const VIGNETTE_STRENGTH = 0.2;

/**
 * Depth value that counts as the background. A reversed buffer clears to 0 and
 * a normal buffer clears to 1.
 */
const DEPTH_EPSILON = 1e-6;

/**
 * Frames a retired chain stays alive before the module frees it.
 *
 * A quality change frees every render target of the old chain. The card is one
 * or two frames behind the processor, so it may still be reading those targets
 * at that moment. On this machine that killed the renderer: the page either
 * reloaded itself about a second later, or the whole tab stopped answering.
 * Bead b44 measured the fault. A chain that keeps its targets and draws through
 * `renderer.render` instead does not show it, so the free and not the draw is
 * the cause.
 *
 * The old chain therefore waits in a retired list and the frame loop frees it
 * later. Four frames is more than the depth of any queue here.
 */
const RETIRE_FRAMES = 4;

/**
 * Quality of the chain.
 *
 * - `off` draws with the renderer alone. Nothing is allocated.
 * - `low` runs bloom, the tone map, FXAA, and the vignette.
 * - `high` adds the ambient occlusion and it swaps FXAA for SMAA.
 *
 * There is no temporal setting. TRAA needs a velocity output and a jittered
 * projection, and its result can only be judged by eye in motion. Nobody has
 * seen this chain in motion yet, so this module ships the two filters that
 * hold no history and therefore cannot smear.
 */
export type Quality = 'off' | 'low' | 'high';

export interface PostChain {
  /** Draw one frame. It replaces `renderer.render(scene, camera)`. */
  render(): void;

  /** False turns the whole chain off and frees its targets. */
  enabled: boolean;

  setQuality(q: Quality): void;
  setSize(width: number, height: number): void;
  dispose(): void;
}

/** Anything the chain must free when the quality changes. */
interface Disposable {
  dispose(): void;
}

/** Anything the chain must resize when the canvas changes. */
interface Sizable {
  setSize(width: number, height: number): void;
}

interface BuiltChain {
  outputNode: Node;
  disposables: Disposable[];
  /**
   * Work that runs before each frame. It carries the values that the shader
   * cannot read for itself, such as the size of the drawing buffer, into the
   * uniforms of the chain.
   */
  updates: (() => void)[];
  /**
   * Only the scene pass belongs here.
   *
   * Every effect node reads the drawing buffer size in its own frame update
   * and resizes itself, so the chain must not call their `setSize`. A call
   * before the first frame throws, because `BloomNode.setSize` writes into the
   * blur materials that its `setup` has not built yet.
   */
  sizables: Sizable[];
}

/**
 * State the color shape of a node again.
 *
 * `SMAANode` and `FXAANode` both leave their node type open in the TypeScript
 * definitions of three.js, although the shader of each one returns a vec4. The
 * cast tells the compiler what the shader already states. It writes no code.
 */
function asColor(node: Node): Node<'vec4'> {
  return node as Node<'vec4'>;
}

/** Scratch value for the drawing buffer size. It keeps the frame free of waste. */
const _bufferSize = new Vector2();

/**
 * Report whether the ambient occlusion can run on this backend.
 *
 * `GTAONode` sends its projection and its inverse projection to the shader in
 * two `mat4` uniforms. A `mat4` uniform does not reach the shader on the
 * WebGL 2 backend of three.js 0.185.1, so the pass reads a projection of zero
 * and every view position it builds collapses to the origin. The occlusion that
 * it then writes has no relation to the scene. Measured against the WebGPU
 * backend on the same frame, it took a sunlit hangar roof to a factor of 0.31
 * and the doors to 0.13, where WebGPU held both above 0.99, and it darkened the
 * whole picture by about one third.
 *
 * The test matches src/render/renderer.ts, which reads the coordinate system to
 * name the backend that really started.
 */
function supportsAmbientOcclusion(renderer: WebGPURenderer): boolean {
  return renderer.coordinateSystem === WebGPUCoordinateSystem;
}

/**
 * Darken the corners of the image. The factor is one at the center and it
 * falls to `1 - VIGNETTE_STRENGTH` at the corner.
 */
function applyVignette(color: Node): Node<'vec4'> {
  const source = asColor(color);
  const distance = uv().sub(0.5).length();
  const falloff = smoothstep(VIGNETTE_START, VIGNETTE_END, distance);
  const factor = float(1).sub(falloff.mul(VIGNETTE_STRENGTH));
  return vec4(source.rgb.mul(factor), source.a);
}

/**
 * Build the chain for one quality. The caller owns the result and frees it
 * through `disposables`.
 */
function buildChain(
  renderer: WebGPURenderer,
  scene: Scene,
  camera: PerspectiveCamera,
  quality: 'low' | 'high',
): BuiltChain {
  const disposables: Disposable[] = [];
  const sizables: Sizable[] = [];
  const updates: (() => void)[] = [];

  // Zero samples. Read the note on GPU memory at the top of this file. The
  // anti aliasing of this chain does the work that the multisample buffer of
  // the renderer would do.
  const scenePass = pass(scene, camera, { samples: 0 });
  disposables.push(scenePass);
  sizables.push(scenePass);

  // The anti aliasing follows the quality alone. The ambient occlusion also
  // needs a backend that can carry its projection. Read
  // `supportsAmbientOcclusion` above.
  const useAmbientOcclusion = quality === 'high' && supportsAmbientOcclusion(renderer);

  if (useAmbientOcclusion) {
    // The ambient occlusion reads the view space normal of every fragment. The
    // second render target carries it out of the scene pass. No other pass
    // reads the normal, so the scene pass keeps a single output when the
    // occlusion does not run.
    scenePass.setMRT(mrt({ output, normal: normalView }));
  }

  const sceneColor = scenePass.getTextureNode('output');

  let litColor: Node<'vec4'> = vec4(sceneColor);

  if (useAmbientOcclusion) {
    const sceneNormal = scenePass.getTextureNode('normal');
    const sceneDepth = scenePass.getTextureNode('depth');

    const aoPass = ao(sceneDepth, sceneNormal, camera);
    aoPass.resolutionScale = AO_RESOLUTION_SCALE;
    aoPass.radius.value = AO_RADIUS;
    aoPass.samples.value = AO_SAMPLES;
    disposables.push(aoPass);

    // Read the note on the reversed depth buffer at the top of this file. The
    // background keeps a factor of one, so the sky never darkens and a value
    // that is not a number never leaves the ambient occlusion target.
    const backgroundMask = renderer.reversedDepthBuffer
      ? sceneDepth.r.lessThanEqual(float(DEPTH_EPSILON))
      : sceneDepth.r.greaterThanEqual(float(1 - DEPTH_EPSILON));

    // Focal length of the occlusion target, in texels of that target. A view
    // offset of one meter at a view depth of one meter covers this many texels.
    const aoFocalTexels = uniform(1);

    // The near and far planes are numbers, so the frame update carries them
    // over. Every uniform here holds one float. A `mat4` uniform does not
    // reach the shader on the WebGL 2 backend of this version, which is the
    // same fault that leaves GTAO without a projection on that backend. Read
    // the note on the two backends at the top of this file.
    const cameraNear = uniform(camera.near);
    const cameraFar = uniform(camera.far);

    updates.push(() => {
      renderer.getDrawingBufferSize(_bufferSize);
      const aoHeight = Math.round(_bufferSize.height * AO_RESOLUTION_SCALE);
      // The projection maps a half height of tan(fov / 2) at a view depth of
      // one meter onto a half height of aoHeight / 2 texels.
      aoFocalTexels.value = aoHeight / 2 / Math.tan((camera.fov * Math.PI) / 360);
      cameraNear.value = camera.near;
      cameraFar.value = camera.far;
    });

    // Depth of the fragment along the view axis, in meters. src/render/renderer
    // asks for a logarithmic buffer when it cannot get a reversed one, and the
    // two buffers hold a different curve, so each one needs its own inverse.
    // `perspectiveDepthToViewZ` reads `reversedDepthBuffer` for itself, so it
    // covers both the reversed buffer and the plain one.
    const viewZ = renderer.logarithmicDepthBuffer
      ? logarithmicDepthToViewZ(sceneDepth.r, cameraNear, cameraFar)
      : perspectiveDepthToViewZ(sceneDepth.r, cameraNear, cameraFar);
    const viewDepth = viewZ.negate().max(float(AO_MIN_VIEW_DEPTH));

    // Width of the first step of the occlusion march, in texels of the
    // occlusion target. Read the note on the occlusion at a distance above.
    const stepTexels = float(AO_RADIUS / AO_STEPS)
      .mul(aoFocalTexels)
      .div(viewDepth);

    // One where the march resolves the geometry, zero where it cannot.
    const aoTrust = smoothstep(float(AO_DROP_TEXELS), float(AO_TRUST_TEXELS), stepTexels);

    const measured = mix(float(1), aoPass.getTextureNode().r, aoTrust);
    const occlusion = backgroundMask.select(float(1), measured);
    litColor = vec4(sceneColor.rgb.mul(occlusion), sceneColor.a);
  }

  // Bloom. The clamp holds the solar disc down to a value that the blur can
  // spread without covering the sky.
  const bloomInput = vec4(litColor.rgb.clamp(0, BLOOM_INPUT_CLAMP), litColor.a);
  const bloomPass = bloom(bloomInput, BLOOM_STRENGTH, BLOOM_RADIUS, BLOOM_THRESHOLD);
  bloomPass.setResolutionScale(BLOOM_RESOLUTION_SCALE);
  disposables.push(bloomPass);

  // Bloom returns the glare alone, so the chain adds it to the image.
  const glare = litColor.add(bloomPass);

  // THE ONE TONE MAP. Read the note at the top of this file. Everything above
  // this line is linear high dynamic range. Everything below it sits in 0 to 1.
  const mapped = glare.toneMapping(ACESFilmicToneMapping);

  let aliased: Node;
  if (quality === 'high') {
    const smaaPass = smaa(mapped);
    disposables.push(smaaPass);
    aliased = smaaPass;
  } else {
    // FXAA asks for sRGB input, so the transfer runs before it. SMAA asks for
    // the opposite, so the two paths put the transfer in different places.
    const encoded = renderOutput(mapped, NoToneMapping, SRGBColorSpace);
    const fxaaPass = fxaa(encoded);
    disposables.push(fxaaPass);
    aliased = fxaaPass;
  }

  const shaded = applyVignette(aliased);

  // The tone map already ran, so this step only carries the working color space
  // to sRGB. The low path already did that above, so it stops here.
  const outputNode =
    quality === 'high' ? renderOutput(shaded, NoToneMapping, SRGBColorSpace) : shaded;

  return { outputNode, disposables, sizables, updates };
}

/**
 * Build the post processing chain.
 *
 * The chain starts at `high`. Call `setQuality('off')` to compare the cost, or
 * to fall back when the device is short of memory.
 */
export function createPostChain(
  renderer: WebGPURenderer,
  scene: Scene,
  camera: PerspectiveCamera,
): PostChain {
  // `PostProcessing` is the old name of this class and it prints a warning on
  // every start. `RenderPipeline` is the same class under its current name.
  const pipeline = new RenderPipeline(renderer);

  // False means the chain owns the tone map and the color space transfer. Read
  // the note at the top of this file.
  pipeline.outputColorTransform = false;

  let quality: Quality = 'high';
  let lastActiveQuality: 'low' | 'high' = 'high';
  let chain: BuiltChain | null = null;
  let width = 0;
  let height = 0;

  /** Chains that wait for the card to finish with them. Read RETIRE_FRAMES. */
  const retired: { frames: number; disposables: Disposable[] }[] = [];

  /** Put the live chain in the retired list. It is freed a few frames later. */
  function retire(): void {
    if (chain === null) return;
    retired.push({ frames: RETIRE_FRAMES, disposables: chain.disposables });
    chain = null;
  }

  /** Free every retired chain that the card has certainly finished with. */
  function sweepRetired(): void {
    for (let i = retired.length - 1; i >= 0; i -= 1) {
      retired[i].frames -= 1;
      if (retired[i].frames > 0) continue;
      for (const item of retired[i].disposables) item.dispose();
      retired.splice(i, 1);
    }
  }

  /** Free every retired chain at once. Only the shutdown path may call it. */
  function releaseAll(): void {
    retire();
    for (const item of retired) {
      for (const one of item.disposables) one.dispose();
    }
    retired.length = 0;
  }

  function rebuild(): void {
    retire();
    if (quality === 'off') return;

    chain = buildChain(renderer, scene, camera, quality);
    pipeline.outputNode = chain.outputNode;
    pipeline.needsUpdate = true;

    if (width > 0 && height > 0) {
      for (const item of chain.sizables) item.setSize(width, height);
    }
  }

  rebuild();

  return {
    render(): void {
      if (chain === null) {
        void renderer.render(scene, camera);
        sweepRetired();
        return;
      }
      for (const update of chain.updates) update();
      pipeline.render();
      sweepRetired();
    },

    get enabled(): boolean {
      return quality !== 'off';
    },

    set enabled(value: boolean) {
      if (value === (quality !== 'off')) return;
      this.setQuality(value ? lastActiveQuality : 'off');
    },

    setQuality(next: Quality): void {
      if (next === quality) return;
      quality = next;
      if (next !== 'off') lastActiveQuality = next;
      rebuild();
    },

    setSize(nextWidth: number, nextHeight: number): void {
      width = nextWidth;
      height = nextHeight;
      if (chain === null) return;
      // Read the note on `sizables`. Every effect node sizes itself.
      for (const item of chain.sizables) item.setSize(nextWidth, nextHeight);
    },

    dispose(): void {
      releaseAll();
      pipeline.dispose();
    },
  };
}
