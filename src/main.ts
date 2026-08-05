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
import { G0 } from '@/math/units';
import { equivalentAirspeed } from '@/physics/atmosphere';
import type { Wrench } from '@/physics/rigidbody';
import { worldToBody } from '@/physics/rigidbody';
import { createArmament } from '@/weapons/armament';
import { createCameraRig } from '@/render/cameras';
import { createForceArrows } from '@/render/force-arrows';
import { nedQuatToThree, nedToThree } from '@/render/frames';
import { createFreeCamera } from '@/render/free-camera';
import type { Me262Pivots } from '@/render/models/me262';
import type { Me262Cockpit } from '@/render/models/cockpit';
import { ME262_COCKPIT_TRAVEL, createMe262Cockpit } from '@/render/models/cockpit';
import { ME262_POSE, createMe262Model } from '@/render/models/me262';
import { createParticles } from '@/render/particles';
import { createPostChain } from '@/render/postfx';
import { createRenderer, isWebGPUAvailable } from '@/render/renderer';
import { createWeaponEffects } from '@/render/weapons';
import type { TelemetrySample } from '@/ui/debug-overlay';
import { createDebugOverlay } from '@/ui/debug-overlay';
import type { CockpitGauges } from '@/ui/gauges';
import { createMe262Gauges } from '@/ui/gauges';
import { createHud } from '@/ui/hud';
import { createTelemetryGraph } from '@/ui/telemetry-graph';
import { createWorld } from '@/world/scene';

/** Where the free camera starts, in NED, when the pilot turns it on. */
const FREE_CAMERA_OFFSET_NED = new Vector3(-40, -25, -14);

/**
 * Where the recoil of the guns goes into the flight model.
 *
 * src/weapons/mk108.ts builds a BODY wrench about the center of gravity, and
 * src/aircraft/aircraft.ts must add it to the wrench of every Runge-Kutta
 * stage, next to the thrust and the gear drag. The hook is one optional member
 * on `Aircraft` and one line inside its wrench source:
 *
 *   in the Aircraft interface     externalWrench: Wrench;
 *   in createAircraft             const externalWrench: Wrench = createWrench();
 *   in the api object             externalWrench,
 *   in source(), after block 4    addWrench(out, externalWrench);
 *
 * Until that hook lands the member is not there, `externalWrench` reads
 * undefined, and the block below does nothing. The guns still fire and the
 * rounds still fly. `wrench` is a member `Aircraft` already has, and it is here
 * so that TypeScript does not read this type as a weak type with nothing in
 * common with `Aircraft`.
 */
interface RecoilTarget {
  readonly wrench: Wrench;
  readonly externalWrench?: Wrench;
}

/** What the head up display reads from one engine. The HUD type is readonly. */
interface EngineReadoutFields {
  rpm: number;
  gasTemperature: number;
  state: string;
}

/**
 * What the cockpit dials read from one engine. The gauge type is readonly.
 *
 * The rotor speed goes over in RADIANS PER SECOND. CONVENTIONS section 2 says
 * the model holds rad/s and only the gauge shows rpm, so the conversion sits
 * inside src/ui/gauges/tachometer.ts and not here.
 */
interface EngineGaugeFields {
  rotorSpeed: number;
  gasTemperature: number;
  fuelFlow: number;
}

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

/** A panel that shows one message across the middle of the picture. */
interface Banner {
  show(text: string): void;
  hide(): void;
}

/**
 * Builds the divergence banner.
 *
 * The flight model of bead b53 stops the aircraft when a state value stops
 * being a finite number, and it says so on `aircraft.events`. A simulator that
 * freezes without a word looks like a fault of the machine, so the pilot must
 * read what happened and how to carry on.
 */
function createBanner(parent: HTMLElement): Banner {
  const root = document.createElement('div');
  root.style.position = 'absolute';
  root.style.top = '38%';
  root.style.left = '50%';
  root.style.transform = 'translate(-50%, -50%)';
  root.style.maxWidth = '620px';
  root.style.padding = '14px 20px';
  root.style.display = 'none';
  root.style.background = 'rgba(24, 6, 4, 0.86)';
  root.style.border = '1px solid #ff6b5e';
  root.style.borderRadius = '4px';
  root.style.color = '#ffd9d4';
  root.style.font = '14px/1.6 ui-monospace, "DejaVu Sans Mono", monospace';
  root.style.textAlign = 'center';
  // The message carries its own line breaks, so the box must keep them.
  root.style.whiteSpace = 'pre-line';
  root.style.pointerEvents = 'none';
  root.style.userSelect = 'none';
  parent.appendChild(root);
  return {
    show(text: string): void {
      root.textContent = text;
      root.style.display = 'block';
    },
    hide(): void {
      root.style.display = 'none';
    },
  };
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

/**
 * Drives the five pivots of the virtual cockpit from the aircraft.
 *
 * Section 4 of src/render/models/cockpit.ts fixes the sign of every pivot. The
 * stick and the pedals follow the CONTROL ARRAY and not the raw stick, for the
 * same reason the exterior surfaces do.
 */
function driveCockpit(cockpit: Me262Cockpit, aircraft: Aircraft, throttle: number): void {
  const controls = aircraft.controls;
  const travel = ME262_COCKPIT_TRAVEL;

  // A positive elevator command pitches the nose UP. A positive stick angle
  // pulls the grip AFT, which is the same direction, so the sign holds.
  cockpit.pivots.stick.rotation.x =
    clamp(controls[1] / ELEVATOR_LIMIT, -1, 1) * travel.stickPitch;
  // A positive aileron command rolls RIGHT. A positive stick angle moves the
  // grip to PORT, so the sign turns over.
  cockpit.pivots.stick.rotation.z =
    -clamp(controls[0] / AILERON_LIMIT, -1, 1) * travel.stickRoll;

  // A positive rudder command yaws the nose RIGHT, which pushes the RIGHT
  // pedal forward. A positive pedal angle is forward on both sides.
  const rudder = clamp(controls[2] / RUDDER_LIMIT, -1, 1) * travel.pedal;
  cockpit.pivots.pedalRight.rotation.x = rudder;
  cockpit.pivots.pedalLeft.rotation.x = -rudder;

  // One throttle axis drives both levers. The axis runs from 0 to 1.
  const lever = clamp(throttle, 0, 1) * travel.throttle;
  cockpit.pivots.throttleLeft.rotation.x = lever;
  cockpit.pivots.throttleRight.rotation.x = lever;
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

  // The interior is invisible from outside, so it is built on the FIRST entry
  // into the cockpit view and hidden in every other view. A flight that never
  // uses that view therefore never pays for it. See cockpit.ts section 6.
  let cockpit: Me262Cockpit | null = null;
  // The live dials of that interior. They are built with it and they only run
  // while it is on screen, for the same reason.
  let gauges: CockpitGauges | null = null;

  const arrows = createForceArrows(aircraft.assembly.surfaces.length);
  model.root.add(arrows.root);
  arrows.visible = false;

  // --- The guns ----------------------------------------------------------
  // The armament owns the four MK 108, the rounds in the air and the hits. It
  // reads the SAME target list the world drew, so the box a shell tests against
  // is the box the model stands in.
  const armament = createArmament(world.targets);
  const effects = createWeaponEffects();
  scene.add(effects.root);
  // The flashes ride with the aircraft. In the world frame they would sit
  // behind the muzzle at 250 m/s and lag by one frame of interpolation.
  model.root.add(effects.muzzleRoot);
  const recoilTarget: RecoilTarget = aircraft;

  // --- The particles ------------------------------------------------------
  // The exhaust, the contrails, the wheel dust and an engine fire. Every one
  // of them stands in the WORLD, so the root hangs off the scene and not off
  // the model. A trail that hung off the model would follow the aircraft.
  const particles = createParticles(renderer);
  scene.add(particles.root);

  // --- The input ---------------------------------------------------------
  // The differential brake only works while the wheels touch the ground, so
  // the input system asks the aircraft.
  const input = createInputSystem({ groundContact: () => aircraft.state.onGround });

  // --- The cameras -------------------------------------------------------
  // V on the keyboard and Y on the gamepad step through the four flight views.
  // F2 turns the free camera on and off. The free camera is a development tool,
  // so it sits on a function key and not on the view key.
  const rig = createCameraRig(camera);
  const freeCamera = createFreeCamera(camera, canvas);
  freeCamera.enabled = false;
  let useFreeCamera = false;

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

  // --- The head up display -----------------------------------------------
  // The display shows only in the outside views. The cockpit view has its own
  // instruments, which bead b37 builds.
  const hud = createHud(overlay);
  const readoutEngines: EngineReadoutFields[] = aircraft.state.engines.map(() => ({
    rpm: 0,
    gasTemperature: 0,
    state: 'off',
  }));
  // The display reads this record every frame, so it is built one time and
  // written in place. Its shape is the AircraftReadout of src/ui/hud.ts.
  const readout = {
    engines: readoutEngines,
    throttle: 0,
    fuelMass: 0,
    gearPosition: 0,
    flapPosition: 0,
    rounds: armament.roundsLeft,
  };

  // --- The cockpit dials -------------------------------------------------
  // ONE interface carries every value that reaches a dial. Its shape is the
  // CockpitReadout of src/ui/gauges/readout.ts, and the telemetry sample above
  // carries the rest. Nothing under src/ui reaches into the physics itself.
  const gaugeEngines: EngineGaugeFields[] = aircraft.state.engines.map(() => ({
    rotorSpeed: 0,
    gasTemperature: 0,
    fuelFlow: 0,
  }));
  const gaugeReadout = {
    engines: gaugeEngines,
    fuelMass: 0,
    lateralAcceleration: 0,
    longitudinalAcceleration: 0,
  };
  // Scratch for the specific force below. The frame allocates nothing.
  const gaugeGravityWorld = new Vector3();
  const gaugeGravityBody = new Vector3();

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

  // --- The divergence banner ---------------------------------------------
  // The flight model reports a state that is no longer a finite number. It then
  // holds the aircraft still until a respawn, so the pilot needs to read the
  // reason and the way out.
  const banner = createBanner(overlay);
  aircraft.events.on('diverged', (event) => {
    // The model writes the reason and the way out, so this line only adds the
    // time. Two prints of one instruction read as two instructions.
    banner.show(`THE FLIGHT MODEL DIVERGED AT ${event.time.toFixed(1)} s\n\n${event.message}`);
  });

  /** Puts the aircraft back on the threshold and the camera behind it. */
  function respawn(): void {
    banner.hide();
    aircraft.spawnOnRunway();
    armament.reset();
    previousPosition.copy(aircraft.state.body.position);
    previousOrientation.copy(aircraft.state.body.orientation);
    nedToThree(aircraft.state.body.position, renderPosition);
    nedQuatToThree(aircraft.state.body.orientation, renderOrientation);
    rig.snap();
    wheelAngles[0] = 0;
    wheelAngles[1] = 0;
    wheelAngles[2] = 0;
  }

  /** Hands the camera to the free camera, or takes it back. */
  function toggleFreeCamera(): void {
    useFreeCamera = !useFreeCamera;
    freeCamera.enabled = useFreeCamera;
    if (useFreeCamera) {
      // Put the free camera where it can see the aircraft, then let it fly.
      nedToThree(FREE_CAMERA_OFFSET_NED, freeCameraStart);
      camera.position.copy(renderPosition).add(freeCameraStart);
      camera.up.set(0, 1, 0);
      camera.lookAt(renderPosition);
    } else {
      rig.snap();
    }
  }

  window.addEventListener('keydown', (event: KeyboardEvent) => {
    if (event.repeat) return;
    if (event.code === 'KeyR') respawn();
    if (event.code === 'F2') toggleFreeCamera();
  });

  const loop = createLoop({
    fixedUpdate(dt: number): void {
      input.poll(dt);
      if (input.state.cycleView) pendingCycleView = true;
      if (input.state.toggleDebug) pendingToggleDebug = true;
      previousPosition.copy(aircraft.state.body.position);
      previousOrientation.copy(aircraft.state.body.orientation);

      // The guns run BEFORE the flight model, so the recoil of this step is in
      // place when the wrench source of the step reads it.
      armament.fixedUpdate(aircraft.state.body, input.state.fireCannon, dt);
      const recoil = recoilTarget.externalWrench;
      if (recoil !== undefined) {
        recoil.force.copy(armament.recoil.force);
        recoil.moment.copy(armament.recoil.moment);
      }
      effects.collect(armament);

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

      // The struts telescope. me262GearLegs() orders the legs nose, main left,
      // main right, which matches the call.
      model.setGearCompression(
        aircraft.state.gear.legs[0].compression,
        aircraft.state.gear.legs[1].compression,
        aircraft.state.gear.legs[2].compression,
      );

      if (pendingCycleView) {
        pendingCycleView = false;
        rig.cycle();
      }
      if (pendingToggleDebug) {
        pendingToggleDebug = false;
        debugLevel = (debugLevel + 1) % 3;
        applyDebugLevel();
      }

      if (useFreeCamera) {
        freeCamera.update(frameDt);
      } else {
        // The rig takes the NED pose and converts it itself, so this file hands
        // over the blended physics pose and not the render pose.
        rig.update(
          blendedPosition,
          blendedOrientation,
          aircraft.state.loadFactor,
          aircraft.state.totals.trueAirspeed,
          input.state.lookYaw,
          input.state.lookPitch,
          frameDt,
        );
      }

      // --- The virtual cockpit -----------------------------------------
      // The view is chosen above, so this test reads the view of THIS frame.
      const inCockpit = !useFreeCamera && rig.mode === 'cockpit';
      if (inCockpit && cockpit === null) {
        cockpit = createMe262Cockpit();
        model.root.add(cockpit.root);
        // The dials paint every face one time, here. Nothing after this line
        // uploads a texture. See src/ui/gauges/index.ts.
        gauges = createMe262Gauges(cockpit.gauges);
      }
      if (cockpit !== null) {
        cockpit.setVisible(inCockpit);
        if (inCockpit) driveCockpit(cockpit, aircraft, input.state.throttle);
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

      // The head up display belongs to the outside views only.
      const systems = aircraft.state.systems.state;
      for (let i = 0; i < readoutEngines.length; i++) {
        const engine = aircraft.state.engines[i];
        readoutEngines[i].rpm = engine.rpm;
        readoutEngines[i].gasTemperature = engine.gasTemperature;
        readoutEngines[i].state = engine.state;
      }
      readout.throttle = input.state.throttle;
      readout.fuelMass = systems.fuelMass;
      readout.gearPosition = systems.gearPosition;
      readout.flapPosition = systems.flapPosition;
      readout.rounds = armament.roundsLeft;

      // --- The cockpit dials --------------------------------------------
      // They only run while the interior is on screen. A needle nobody can see
      // costs nothing to leave where it was.
      if (inCockpit && gauges !== null) {
        for (let i = 0; i < gaugeEngines.length; i++) {
          const engine = aircraft.state.engines[i];
          gaugeEngines[i].rotorSpeed = engine.rotorSpeed;
          gaugeEngines[i].gasTemperature = engine.gasTemperature;
          gaugeEngines[i].fuelFlow = engine.fuelFlow;
        }
        gaugeReadout.fuelMass = systems.fuelMass;
        // The slip ball and the erection error of the gyro horizon both hang
        // on the SPECIFIC FORCE, which is the total body force with gravity
        // taken out, over the mass. src/aircraft/aircraft.ts builds its own
        // load factor from the z part of the same difference.
        const mass = aircraft.state.mass.mass;
        gaugeGravityWorld.set(0, 0, mass * G0);
        worldToBody(aircraft.state.body.orientation, gaugeGravityWorld, gaugeGravityBody);
        gaugeReadout.lateralAcceleration =
          (aircraft.wrench.force.y - gaugeGravityBody.y) / mass;
        gaugeReadout.longitudinalAcceleration =
          (aircraft.wrench.force.x - gaugeGravityBody.x) / mass;
        gauges.update(telemetry, gaugeReadout, frameDt);
      }

      // The tracers, the bursts and the four muzzle flashes.
      effects.update(armament, frameDt);

      // The compute pass that moves every particle. It runs before the draw,
      // so the frame draws the positions of this frame.
      particles.update(aircraft, armament, renderPosition, renderOrientation, frameDt);
      hud.visible = rig.mode !== 'cockpit';
      hud.update(telemetry, readout);

      world.update(frameDt, camera.position);
      post.render();
    },
  });

  // A handle for the development tools. The screenshot harness reads the
  // backend, places the aircraft, and steps the views through it. Nothing
  // inside src reads it.
  (window as unknown as { sim: unknown }).sim = {
    bundle,
    scene,
    camera,
    world,
    aircraft,
    input,
    rig,
    hud,
    loop,
    armament,
    particles,
    post,
  };

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
