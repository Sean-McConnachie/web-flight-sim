/**
 * Composition root.
 *
 * This file starts the simulator. It builds the renderer, the world, the input
 * and the aircraft, then it starts the fixed step loop.
 *
 * IT IS THE ONLY PLACE THAT JOINS THE PHYSICS TO THE PICTURE. CONVENTIONS
 * section 4 stops src/aircraft from importing anything under src/render,
 * src/ui or src/input, so every wire between the two sides runs through here.
 *
 *
 * THE TWO RATES
 *
 * src/core/loop.ts steps the physics at a fixed rate and draws at the rate of
 * the display. The two rarely line up, so the loop reports `alpha`, which is
 * where the frame falls between the last physics state and the current one.
 * This file keeps the pose of the PREVIOUS step and blends the two poses with
 * `alpha`. Without that blend the aircraft steps across the screen at 240 Hz
 * against a camera that moves at 60 Hz, and the picture judders.
 *
 *
 * THE CONTROL SURFACES
 *
 * The pivots follow the CONTROL ARRAY that the aerodynamics reads, not the raw
 * stick. The surface that moves on the screen is then the surface that makes
 * the moment. Section 4 of src/render/models/me262.ts fixes the sign of every
 * pivot, and each line below carries the reason for its sign.
 *
 * Read docs/CONVENTIONS.md before you change this file. Frame conversion
 * happens only in src/render/frames.ts.
 */

import { Quaternion, Vector3 } from 'three';

import type { Aircraft, AircraftInput } from '@/aircraft/aircraft';
import {
  AILERON_LIMIT,
  ELEVATOR_LIMIT,
  RUDDER_LIMIT,
  createAircraft,
} from '@/aircraft/aircraft';
import { createLoop } from '@/core/loop';
import type { InputSystem } from '@/input/bindings';
import { createInputSystem } from '@/input/bindings';
import { clamp } from '@/math/tables';
import { equivalentAirspeed } from '@/physics/atmosphere';
import { createChaseCamera } from '@/render/cameras';
import { createForceArrows } from '@/render/force-arrows';
import { nedQuatToThree, nedToThree } from '@/render/frames';
import { createFreeCamera } from '@/render/free-camera';
import type { Me262Pivots } from '@/render/models/me262';
import { ME262_POSE, createMe262Model } from '@/render/models/me262';
import { createPostChain } from '@/render/postfx';
import { createRenderer, isWebGPUAvailable } from '@/render/renderer';
import type { TelemetrySample } from '@/ui/debug-overlay';
import { createDebugOverlay } from '@/ui/debug-overlay';
import { createTelemetryGraph } from '@/ui/telemetry-graph';
import { createWorld } from '@/world/scene';

/** Where the free camera starts, in NED, when the pilot turns it on. */
const FREE_CAMERA_OFFSET_NED = new Vector3(-40, -25, -14);

/** Radius of each wheel, in meters, in the leg order of src/physics/gear.ts. */
const WHEEL_RADII = [0.33, 0.42, 0.42];

/**
 * How wide the gear doors stand while the leg travels.
 *
 * The parabola 4 p (1 - p) is 1 at the middle of the travel and 0 at both ends,
 * and the factor of 1.6 holds the doors fully open over the middle two thirds
 * of the cycle. A locked gear, up or down, has its doors shut and flush.
 * src/aircraft/me262/systems.ts uses the same parabola for the drag of the
 * doors, so the picture and the drag agree.
 */
function doorOpening(gearPosition: number): number {
  return clamp(1.6 * 4 * gearPosition * (1 - gearPosition), 0, 1);
}

function showFatal(message: string): void {
  const el = document.getElementById('fatal');
  if (el === null) return;
  el.style.display = 'grid';
  el.textContent = message;
}

function requireElement<T extends HTMLElement>(id: string): T {
  const el = document.getElementById(id);
  if (el === null) throw new Error(`The page has no element with the id "${id}".`);
  return el as T;
}

/**
 * Drives all nineteen pivots of the model from the aircraft.
 *
 * `dt` is the frame time. Only the wheel spin needs it, because a wheel angle
 * is the integral of a rate and no other pivot is.
 */
function driveModel(
  pivots: Me262Pivots,
  aircraft: Aircraft,
  wheelAngles: number[],
  dt: number,
): void {
  const controls = aircraft.controls;
  const systems = aircraft.state.systems.state;

  // --- The aerodynamic controls ---------------------------------------
  // A positive aileron command rolls RIGHT, so the right aileron must go UP
  // and the left aileron must go DOWN. A positive pivot angle puts the
  // trailing edge DOWN on both wings, so the right side takes the minus sign.
  const aileron = clamp(controls[0] / AILERON_LIMIT, -1, 1) * ME262_POSE.aileronLimit;
  pivots.aileronLeft.rotation.x = aileron;
  pivots.aileronRight.rotation.x = -aileron;

  // A positive elevator command pitches the nose UP, which needs the elevator
  // trailing edge UP, which is a negative pivot angle.
  const elevator = -clamp(controls[1] / ELEVATOR_LIMIT, -1, 1) * ME262_POSE.elevatorLimit;
  pivots.elevatorLeft.rotation.x = elevator;
  pivots.elevatorRight.rotation.x = elevator;

  // A positive rudder command yaws the nose RIGHT, which needs the rudder
  // trailing edge to the RIGHT. A positive pivot angle carries it to the LEFT,
  // so the sign turns over.
  pivots.rudder.rotation.x = -clamp(controls[2] / RUDDER_LIMIT, -1, 1) * ME262_POSE.rudderLimit;

  // --- The systems -----------------------------------------------------
  const flap = systems.flapPosition * ME262_POSE.flapLanding;
  pivots.flapLeft.rotation.x = flap;
  pivots.flapRight.rotation.x = flap;

  const slat = systems.slatPosition * ME262_POSE.slatDeployed;
  pivots.slatLeft.rotation.x = slat;
  pivots.slatRight.rotation.x = slat;

  // The mains lead the nose leg, so each one reads its own share of the cycle.
  // A pivot angle of zero is down and locked, so the retracted end is one.
  const nose = aircraft.state.systems.noseGearPosition();
  const main = aircraft.state.systems.mainGearPosition();
  pivots.gearNose.rotation.x = (1 - nose) * ME262_POSE.gearNoseRetracted;
  pivots.gearLeft.rotation.x = (1 - main) * ME262_POSE.gearMainRetracted;
  pivots.gearRight.rotation.x = (1 - main) * ME262_POSE.gearMainRetracted;

  const doors = doorOpening(systems.gearPosition);
  pivots.gearDoorNose.rotation.x = doors * ME262_POSE.gearDoorNoseOpen;
  pivots.gearDoorLeft.rotation.x = doors * ME262_POSE.gearDoorOpen;
  pivots.gearDoorRight.rotation.x = doors * ME262_POSE.gearDoorOpen;

  // --- The wheels -------------------------------------------------------
  // GearLegState.wheelSpeed is the speed of the tread at the contact patch, so
  // the angular rate is that speed over the radius. A positive pivot angle
  // rolls the wheel FORWARD.
  const legs = aircraft.state.gear.legs;
  for (let i = 0; i < legs.length && i < WHEEL_RADII.length; i++) {
    wheelAngles[i] = (wheelAngles[i] + (legs[i].wheelSpeed / WHEEL_RADII[i]) * dt) % (Math.PI * 2);
  }
  pivots.wheelNose.rotation.x = wheelAngles[0];
  pivots.wheelLeft.rotation.x = wheelAngles[1];
  pivots.wheelRight.rotation.x = wheelAngles[2];

  // The hood stays shut. Bead b34 opens it with the cockpit view.
  pivots.canopy.rotation.x = 0;
}

/** Copies the pilot command out of the input system, without the look axes. */
function readInput(input: InputSystem): AircraftInput {
  return input.state;
}

async function main(): Promise<void> {
  const canvas = requireElement<HTMLCanvasElement>('canvas');
  const overlay = requireElement<HTMLDivElement>('overlay');

  const bundle = await createRenderer(canvas);
  const { renderer, scene, camera } = bundle;


  const world = createWorld(renderer, scene);

  const post = createPostChain(renderer, scene, camera);
  post.setSize(canvas.width, canvas.height);
  window.addEventListener('resize', () => post.setSize(canvas.width, canvas.height));

  // --- The aircraft ------------------------------------------------------
  const aircraft = createAircraft();
  aircraft.spawnOnRunway();

  const model = createMe262Model();
  scene.add(model.root);
  const wheelAngles = [0, 0, 0];

  const arrows = createForceArrows(aircraft.assembly.surfaces.length);
  model.root.add(arrows.root);
  arrows.visible = false;

  // --- The input ---------------------------------------------------------
  // The differential brake only works while the wheels touch the ground, so
  // the input system asks the aircraft.
  const input = createInputSystem({ groundContact: () => aircraft.state.onGround });

  // --- The cameras -------------------------------------------------------
  const chase = createChaseCamera(camera);
  const freeCamera = createFreeCamera(camera, canvas);
  freeCamera.enabled = false;
  let useChase = true;

  // --- The debug view ----------------------------------------------------
  const debug = createDebugOverlay(overlay);
  // The overlay panel and the chart both place themselves at the left of their
  // parent. A tall panel on a short window then covers the chart, so the chart
  // gets its own corner. The chart still positions itself inside this box.
  const graphCorner = document.createElement('div');
  graphCorner.style.position = 'absolute';
  graphCorner.style.right = '0';
  graphCorner.style.bottom = '0';
  graphCorner.style.width = '440px';
  graphCorner.style.height = '260px';
  graphCorner.style.pointerEvents = 'none';
  overlay.appendChild(graphCorner);
  const graph = createTelemetryGraph(graphCorner);
  // F3 steps through the three debug levels: nothing, the numbers, and the
  // numbers with the per element force arrows.
  let debugLevel = 1;
  const applyDebugLevel = (): void => {
    debug.visible = debugLevel > 0;
    graph.visible = debugLevel > 0;
    arrows.visible = debugLevel > 1;
  };
  applyDebugLevel();

  const telemetry: TelemetrySample = {
    loop: loopStats(),
    state: aircraft.state.body,
    alpha: 0,
    beta: 0,
    loadFactor: 1,
    trueAirspeed: 0,
    equivalentAirspeed: 0,
    mach: 0,
    dynamicPressure: 0,
    atmosphere: aircraft.atmosphere,
  };

  // --- The render pose ---------------------------------------------------
  // The loop steps the physics a whole number of times per frame and reports
  // where the frame falls between the last two states. Keep both.
  const previousPosition = new Vector3().copy(aircraft.state.body.position);
  const previousOrientation = new Quaternion().copy(aircraft.state.body.orientation);
  const blendedPosition = new Vector3();
  const blendedOrientation = new Quaternion();
  const renderPosition = new Vector3();
  const renderOrientation = new Quaternion();
  const freeCameraStart = new Vector3();

  // The edge actions fire for one POLL, and one frame can hold several polls or
  // none. The flags below carry an edge from the physics step to the frame that
  // acts on it, so no press is lost and none is read twice.
  let pendingCycleView = false;
  let pendingToggleDebug = false;

  /** Puts the aircraft back on the threshold and the camera behind it. */
  function respawn(): void {
    aircraft.spawnOnRunway();
    previousPosition.copy(aircraft.state.body.position);
    previousOrientation.copy(aircraft.state.body.orientation);
    nedToThree(aircraft.state.body.position, renderPosition);
    nedQuatToThree(aircraft.state.body.orientation, renderOrientation);
    chase.snap(renderPosition, renderOrientation);
    wheelAngles[0] = 0;
    wheelAngles[1] = 0;
    wheelAngles[2] = 0;
  }

  window.addEventListener('keydown', (event: KeyboardEvent) => {
    if (event.code === 'KeyR' && !event.repeat) respawn();
  });

  const loop = createLoop({
    fixedUpdate(dt: number): void {
      input.poll(dt);
      if (input.state.cycleView) pendingCycleView = true;
      if (input.state.toggleDebug) pendingToggleDebug = true;
      previousPosition.copy(aircraft.state.body.position);
      previousOrientation.copy(aircraft.state.body.orientation);
      aircraft.fixedUpdate(readInput(input), dt);
    },

    render(alpha: number, frameDt: number): void {
      // The pose the frame draws sits between the last two physics states.
      blendedPosition.copy(previousPosition).lerp(aircraft.state.body.position, alpha);
      blendedOrientation.copy(previousOrientation).slerp(aircraft.state.body.orientation, alpha);
      nedToThree(blendedPosition, renderPosition);
      nedQuatToThree(blendedOrientation, renderOrientation);
      model.root.position.copy(renderPosition);
      model.root.quaternion.copy(renderOrientation);

      driveModel(model.pivots, aircraft, wheelAngles, frameDt);

      if (pendingCycleView) {
        pendingCycleView = false;
        useChase = !useChase;
        freeCamera.enabled = !useChase;
        if (!useChase) {
          // Put the free camera where it can see the aircraft, then let it fly.
          nedToThree(FREE_CAMERA_OFFSET_NED, freeCameraStart);
          camera.position.copy(renderPosition).add(freeCameraStart);
          camera.up.set(0, 1, 0);
          camera.lookAt(renderPosition);
        } else {
          chase.snap(renderPosition, renderOrientation);
        }
      }
      if (pendingToggleDebug) {
        pendingToggleDebug = false;
        debugLevel = (debugLevel + 1) % 3;
        applyDebugLevel();
      }

      if (useChase) {
        chase.update(renderPosition, renderOrientation, frameDt);
      } else {
        freeCamera.update(frameDt);
      }

      // The arrows hang under the model, so their frame is the body frame
      // mapped through frames.ts. See src/render/force-arrows.ts.
      if (arrows.visible) {
        arrows.update(aircraft.assembly.sampleForDebug());
      }

      const totals = aircraft.state.totals;
      telemetry.loop = loop.stats;
      telemetry.alpha = totals.alpha;
      telemetry.beta = totals.beta;
      telemetry.loadFactor = aircraft.state.loadFactor;
      telemetry.trueAirspeed = totals.trueAirspeed;
      telemetry.equivalentAirspeed = equivalentAirspeed(
        totals.trueAirspeed,
        aircraft.atmosphere.density,
      );
      telemetry.mach = totals.mach;
      telemetry.dynamicPressure = totals.dynamicPressure;
      debug.update(telemetry);
      graph.push(telemetry, loop.stats.simTime);

      world.update(frameDt, camera.position);
      post.render();
    },
  });

  respawn();
  loop.start();
}

/** An empty statistics record, so the telemetry object is complete at once. */
function loopStats(): TelemetrySample['loop'] {
  return {
    fps: 0,
    physicsStepsLastFrame: 0,
    droppedTime: 0,
    fixedUpdateMs: 0,
    renderMs: 0,
    simTime: 0,
  };
}

main().catch((error: unknown) => {
  const backendHint = isWebGPUAvailable()
    ? 'The browser reports WebGPU, so the device request failed.'
    : 'The browser has no WebGPU, so the renderer tried WebGL 2 and that failed too.';
  showFatal(`The simulator failed to start.\n\n${backendHint}\n\n${String(error)}`);
});
