/**
 * The camera set.
 *
 * Four views, one rig. `V` on the keyboard and `Y` on the gamepad step through
 * them in the order cockpit, chase, orbit, fly by.
 *
 *   cockpit  the eye of the pilot, inside the hood, looking over the nose.
 *   chase    behind and above the aircraft, with the horizon level.
 *   orbit    a free turn about the aircraft at a settable distance.
 *   flyby    a fixed point in the world that watches the aircraft go past.
 *
 * The rig takes the pose of the aircraft in the NED physics frame and converts
 * it with src/render/frames.ts. No other file converts a frame. See
 * docs/CONVENTIONS.md section 3.
 *
 * This file may use the renderer and the browser. It holds no physics.
 *
 *
 * 1. WHY THE OUTSIDE VIEWS DO NOT ROLL
 *
 * A camera fixed to the airframe rolls the whole world when the pilot rolls.
 * That reads well from the cockpit, because the head of the pilot really does
 * roll with the aircraft. From outside it reads badly, because the eye then
 * cannot tell a roll from a turn, and a long roll makes a person sick. The
 * chase view, the orbit view and the fly by view all keep their up vector on
 * the world vertical. Only the cockpit rolls.
 *
 *
 * 2. WHERE THE EYE OF THE PILOT SITS
 *
 * COCKPIT_EYE_FORWARD and COCKPIT_EYE_UP put the eye 4.15 m aft of the nose tip
 * and 0.82 m above the model center line. Bead b36 builds the panel around this
 * point, so the two must agree.
 *
 * The numbers come from the model of src/render/models/me262.ts:
 *
 *   The origin sits at the center of gravity, 5.76 m aft of the nose tip.
 *   The hood runs from 3.74 m to 5.05 m aft of the nose tip. Its sill sits at
 *   y = 0.56 and its crown reaches y = 1.035 at the widest station.
 *   The fuselage skin at that station tops out at y = 0.75.
 *
 * An eye at y = 0.82 therefore stands 0.07 m clear of the skin and 0.2 m below
 * the crown of the hood, which is where a seated head sits. The station of
 * 4.15 m puts the eye between the armored glass at 3.52 m and the aft hoop at
 * 5.02 m.
 *
 * The view over the nose follows from those numbers. The deck ahead of the
 * windscreen tops out at y = 0.735, 0.85 m ahead of the eye, so the pilot sees
 * the ground 5.7 deg below the horizon over the nose. That is the value a
 * fighter of this shape gives.
 *
 *
 * 3. THE HEAD MOVES
 *
 * A head that is welded to the airframe reads as a photograph. Two effects move
 * it. Both are small, and both are meant to be felt and not seen.
 *
 *   G LOAD. The pilot compresses into the seat under g. The eye drops
 *   COCKPIT_G_TRAVEL meters for every g above 1 and rises under negative g.
 *   The travel is clamped, because a spine is not a spring with no end stop.
 *
 *   VIBRATION. The airframe shakes with the air load. The amplitude grows with
 *   speed, which is an estimate of the real dependence on dynamic pressure. The
 *   shake is the sum of three sines at frequencies that share no factor, so the
 *   pattern never repeats and the eye reads it as noise.
 *
 *
 * 4. THE LOOK AXES
 *
 * `lookYaw` and `lookPitch` arrive from src/input/bindings.ts. They hold the
 * right stick. This module also reads the mouse on its own, because a mouse
 * reports a movement and not a position, so it cannot pass through an axis.
 *
 * The stick is an ABSOLUTE position: a stick held right holds the head right.
 * The mouse is a RELATIVE addition, and it only moves the head while the pilot
 * holds the primary button. The addition decays back to zero once the button
 * comes up, so the head cannot stay stuck to one side.
 *
 * The orbit view is different. Its angles must STAY where the pilot puts them,
 * so there the stick drives a rate and the mouse drives an offset, and neither
 * one returns to a center.
 */

import { Euler, PerspectiveCamera, Quaternion, Vector3 } from 'three/webgpu';

import { clamp } from '@/math/tables';
import { nedQuatToThree, nedToThree } from '@/render/frames';

// ---------------------------------------------------------------------------
// The chase view
// ---------------------------------------------------------------------------

/** Distance behind the aircraft, in meters. */
const DEFAULT_DISTANCE = 26;

/** Height above the aircraft, in meters. */
const DEFAULT_HEIGHT = 6;

/** Point the camera looks at, in meters above the center of gravity. */
const LOOK_HEIGHT = 1.5;

/** How much of the vertical part of the nose direction the camera keeps. */
const VERTICAL_DAMPING = 0.45;

/** Half life of the position lag, in seconds. */
const POSITION_HALF_LIFE = 0.22;

/** Half life of the aim lag, in seconds. It is faster, so the aim leads. */
const AIM_HALF_LIFE = 0.09;

/** Half life of the speed estimate that feeds the lead term, in seconds. */
const VELOCITY_HALF_LIFE = 0.12;

/**
 * Largest lead, in meters. The lead needs 73 m at 230 m/s, so this value
 * covers every speed the aircraft reaches and still holds a teleport in check.
 */
const MAX_LEAD = 140;

/** Natural logarithm of two. It turns a half life into a time constant. */
const LN2 = Math.LN2;

/** Render frame up. */
const UP = new Vector3(0, 1, 0);

export interface ChaseCamera {
  /**
   * Moves the camera. `position` and `orientation` are the render frame pose of
   * the aircraft, which src/render/frames.ts produces from the physics state.
   */
  update(position: Vector3, orientation: Quaternion, dt: number): void;
  /** Puts the camera at its place at once, with no lag. A respawn needs it. */
  snap(position: Vector3, orientation: Quaternion): void;
  distance: number;
  height: number;
}

/**
 * Builds the chase camera. It writes the position and the quaternion of
 * `camera`.
 *
 * THE LAG. The camera moves toward the place it wants with an exponential lag
 * of a fixed half life, which is frame rate independent. The lag is what makes
 * the aircraft look fast: with no lag the aircraft sits still in the frame and
 * only the ground moves.
 *
 * The nose direction keeps part of its vertical component, so a climb puts the
 * camera below the aircraft and a dive puts it above. VERTICAL_DAMPING takes
 * the rest out, so a vertical climb does not put the camera under the tail.
 *
 * THE LEAD. A lag that chases a target which moves at a constant speed settles
 * at a CONSTANT error of speed times the time constant, and the time constant
 * is the half life over the natural logarithm of two. At 230 m/s that error is
 * 73 m, so the aircraft shrinks to a dot in level flight and the view is
 * useless. The camera therefore adds that same product as a lead on the place
 * it wants. The lead cancels the steady error and leaves the transient, so the
 * camera still falls back when the aircraft accelerates.
 */
export function createChaseCamera(camera: PerspectiveCamera): ChaseCamera {
  const nose = new Vector3();
  const wanted = new Vector3();
  const aim = new Vector3();
  const smoothedAim = new Vector3();
  const eye = new Vector3();
  const lastPosition = new Vector3();
  const velocity = new Vector3();
  const sampled = new Vector3();
  const lead = new Vector3();
  let started = false;

  /** Writes the place the camera wants into `wanted` and the aim into `aim`. */
  function solve(position: Vector3, orientation: Quaternion, api: ChaseCamera): void {
    // The nose points along render -z. See src/render/frames.ts.
    nose.set(0, 0, -1).applyQuaternion(orientation);
    nose.y *= VERTICAL_DAMPING;
    if (nose.lengthSq() < 1e-6) {
      nose.set(0, 0, -1);
    }
    nose.normalize();
    wanted.copy(position).addScaledVector(nose, -api.distance).addScaledVector(UP, api.height);
    wanted.add(leadFor(POSITION_HALF_LIFE));
    // The aim carries its own lead, for the same reason. Without it the aim
    // point settles behind the aircraft, and at speed it settles ON the camera,
    // where a lookAt has no direction left to use.
    aim.copy(position).addScaledVector(UP, LOOK_HEIGHT).add(leadFor(AIM_HALF_LIFE));
  }

  /** Writes the lead of one half life into `lead` and returns it. */
  function leadFor(halfLife: number): Vector3 {
    lead.copy(velocity).multiplyScalar(halfLife / LN2);
    if (lead.lengthSq() > MAX_LEAD * MAX_LEAD) lead.setLength(MAX_LEAD);
    return lead;
  }

  /** Follows the speed of the aircraft from the change of its position. */
  function trackVelocity(position: Vector3, dt: number): void {
    if (dt > 0) {
      sampled.subVectors(position, lastPosition).divideScalar(dt);
      const blend = 1 - Math.pow(0.5, dt / VELOCITY_HALF_LIFE);
      velocity.lerp(sampled, blend);
    }
    lastPosition.copy(position);
  }

  const api: ChaseCamera = {
    distance: DEFAULT_DISTANCE,
    height: DEFAULT_HEIGHT,

    snap(position: Vector3, orientation: Quaternion): void {
      velocity.set(0, 0, 0);
      lastPosition.copy(position);
      solve(position, orientation, api);
      camera.position.copy(wanted);
      smoothedAim.copy(aim);
      camera.up.copy(UP);
      camera.lookAt(smoothedAim);
      started = true;
    },

    update(position: Vector3, orientation: Quaternion, dt: number): void {
      if (!started) {
        api.snap(position, orientation);
        return;
      }
      trackVelocity(position, dt);
      solve(position, orientation, api);
      // An exponential lag with a half life is the same at any frame rate.
      const positionBlend = 1 - Math.pow(0.5, dt / POSITION_HALF_LIFE);
      const aimBlend = 1 - Math.pow(0.5, dt / AIM_HALF_LIFE);
      eye.copy(camera.position).lerp(wanted, positionBlend);
      camera.position.copy(eye);
      smoothedAim.lerp(aim, aimBlend);
      camera.up.copy(UP);
      // An aim point that lands on the camera leaves lookAt with no direction,
      // and it then writes an orientation that points anywhere. The aircraft is
      // always the safe answer.
      if (smoothedAim.distanceToSquared(camera.position) < 1) camera.lookAt(position);
      else camera.lookAt(smoothedAim);
    },
  };

  return api;
}

// ---------------------------------------------------------------------------
// Constants of the other three views
// ---------------------------------------------------------------------------

/** Eye of the pilot ahead of the center of gravity, body axes, m. See part 2. */
export const COCKPIT_EYE_FORWARD = 1.61;

/** Eye of the pilot above the center of gravity, m. See part 2. */
export const COCKPIT_EYE_UP = 0.82;

/** Drop of the eye per g above 1, in meters. Estimate from a seat and a spine. */
const COCKPIT_G_TRAVEL = 0.018;

/** End stop of the g travel, in meters, up and down. */
const COCKPIT_G_LIMIT = 0.05;

/** Shake of the airframe at rest, in meters. */
const VIBRATION_BASE = 0.0012;

/**
 * Growth of the shake with true airspeed, in meters per (m/s). The real drive
 * is the dynamic pressure. This linear form is an ESTIMATE that gives 7 mm at
 * 250 m/s, which is a shake a pilot feels and an eye does not read as a fault.
 */
const VIBRATION_PER_SPEED = 2.2e-5;

/** Angular part of the shake, in radians, at rest and per (m/s). */
const VIBRATION_ANGLE_BASE = 0.0008;
const VIBRATION_ANGLE_PER_SPEED = 8e-6;

/** Shake frequencies, in radians per second. They share no common factor. */
const VIBRATION_RATE_1 = 71;
const VIBRATION_RATE_2 = 111.2;
const VIBRATION_RATE_3 = 145.1;

/** End stops of the head, in radians. */
const MAX_HEAD_YAW = 2.4;
const MAX_HEAD_PITCH = 1.2;

/** Half life of the head lag, in seconds. */
const HEAD_HALF_LIFE = 0.07;

/** Half life of the return of the mouse offset once the button comes up, in s. */
const MOUSE_RETURN_HALF_LIFE = 0.5;

/** Radians of look per pixel of mouse movement. The free camera uses the same. */
const MOUSE_SENSITIVITY = 0.0022;

/** Start distance of the orbit view, in meters, and its two end stops. */
const ORBIT_DEFAULT_DISTANCE = 40;
const ORBIT_MIN_DISTANCE = 8;
const ORBIT_MAX_DISTANCE = 400;

/** Change of the orbit distance for one notch of the wheel. */
const ORBIT_ZOOM_STEP = 1.12;

/** Turn rate of the orbit view on the stick, in radians per second. */
const ORBIT_YAW_RATE = 1.6;
const ORBIT_PITCH_RATE = 1;

/** End stop of the orbit elevation, in radians. It stops short of the pole. */
const ORBIT_MAX_ELEVATION = 1.35;

/** Start elevation of the orbit view, in radians. */
const ORBIT_START_ELEVATION = 0.22;

/** Seconds of flight the fly by camera parks ahead of the aircraft. */
const FLYBY_LEAD_TIME = 3;

/** End stops of that lead, in meters. */
const FLYBY_MIN_LEAD = 90;
const FLYBY_MAX_LEAD = 700;

/** Distance of the park point to the side of the track, in meters. */
const FLYBY_LATERAL = 32;

/**
 * Height of the park point below the aircraft, in meters, and the floor it
 * never goes under. The camera sits a little low, so the aircraft passes above
 * it against the sky. That is the view that makes speed readable.
 */
const FLYBY_HEIGHT_BELOW = 6;
const FLYBY_MIN_HEIGHT = 8;

/**
 * How far past the camera the aircraft goes before the camera re-parks, in m.
 *
 * The value is about one second of flight at cruise speed, so the shot holds
 * while the aircraft goes away and only then cuts to the next park point.
 */
const FLYBY_PASS_MARGIN = 220;

/** Distance at which the camera re-parks even with no pass, in meters. */
const FLYBY_MAX_RANGE = 1200;

/** How much of the vertical part of the nose the park point follows. */
const FLYBY_VERTICAL_DAMPING = 0.35;

/**
 * Half life of the fly by aim lag, in seconds.
 *
 * The aim lag is what makes the pan read as a hand on a tripod. It must stay
 * short: the lag in meters is the speed times the time constant, so 0.035 s at
 * 250 m/s puts the aim 13 m behind the aircraft, which is a small part of the
 * frame even at the closest point of a pass.
 */
const FLYBY_AIM_HALF_LIFE = 0.035;

/** Field of view, near plane and far plane of a camera the rig builds itself. */
const DEFAULT_FOV = 55;
const DEFAULT_NEAR = 0.3;
const DEFAULT_FAR = 60000;

// ---------------------------------------------------------------------------
// Public contract
// ---------------------------------------------------------------------------

export type ViewMode = 'cockpit' | 'chase' | 'orbit' | 'flyby';

/** The order that `cycle` steps through. */
export const VIEW_ORDER: readonly ViewMode[] = ['cockpit', 'chase', 'orbit', 'flyby'];

export interface CameraRig {
  camera: PerspectiveCamera;
  /** The view now. A write moves to that view at once. */
  mode: ViewMode;
  /** Steps to the next view of VIEW_ORDER. */
  cycle(): void;
  /**
   * Moves the camera. The pose arrives in the NED physics frame, and this
   * module converts it. `speed` is the true airspeed in m/s. `lookYaw` and
   * `lookPitch` run from -1 to 1 and hold the right stick.
   */
  update(
    aircraftPositionNed: Vector3,
    aircraftOrientationNed: Quaternion,
    loadFactor: number,
    speed: number,
    lookYaw: number,
    lookPitch: number,
    dt: number,
  ): void;

  // The three members below are additions to the bead b34 contract. A respawn
  // needs the snap, the orbit view needs a settable distance, and the mouse
  // listeners need a way out.

  /** Drops every lag and every look angle, then places the camera at once. */
  snap(): void;
  /** Distance of the orbit view, in meters. */
  orbitDistance: number;
  dispose(): void;
}

// ---------------------------------------------------------------------------
// The rig
// ---------------------------------------------------------------------------

/**
 * Builds the camera set.
 *
 * The caller may pass the camera that the renderer already owns. With no
 * argument the rig builds its own, which is what a test does.
 */
export function createCameraRig(existing?: PerspectiveCamera): CameraRig {
  const camera =
    existing ?? new PerspectiveCamera(DEFAULT_FOV, 1, DEFAULT_NEAR, DEFAULT_FAR);
  const chase = createChaseCamera(camera);

  // --- Scratch. update allocates nothing. ---
  const position = new Vector3();
  const orientation = new Quaternion();
  const travel = new Vector3();
  const side = new Vector3();
  const offset = new Vector3();
  const aim = new Vector3();
  const parkPoint = new Vector3();
  const headEuler = new Euler(0, 0, 0, 'YXZ');
  const headQuaternion = new Quaternion();

  /** Eye of the pilot in the model frame. The body axes map through frames.ts. */
  const eyeModel = nedToThree(
    new Vector3(COCKPIT_EYE_FORWARD, 0, -COCKPIT_EYE_UP),
    new Vector3(),
  );

  let mode: ViewMode = 'chase';
  let needsSnap = true;

  // Head angles of the cockpit view, and the part of them that the mouse owns.
  let headYaw = 0;
  let headPitch = 0;
  let mouseYaw = 0;
  let mousePitch = 0;

  // Angles of the orbit view. They hold their place with no input.
  let orbitAzimuth = 0;
  let orbitElevation = ORBIT_START_ELEVATION;

  let flybyParked = false;
  let flybySide = 1;
  let vibrationTime = 0;

  // --- The mouse ---------------------------------------------------------
  // A mouse reports a movement, so it cannot arrive as an axis. The rig reads
  // it here and consumes the total in the next update.
  let mouseDx = 0;
  let mouseDy = 0;
  let mouseWheel = 0;
  let dragging = false;
  const hasWindow = typeof window !== 'undefined';

  const onMouseDown = (event: MouseEvent): void => {
    if (event.button === 0) dragging = true;
  };
  const onMouseUp = (event: MouseEvent): void => {
    if (event.button === 0) dragging = false;
  };
  const onMouseMove = (event: MouseEvent): void => {
    if (!dragging) return;
    mouseDx += event.movementX;
    mouseDy += event.movementY;
  };
  const onWheel = (event: WheelEvent): void => {
    mouseWheel += event.deltaY;
  };
  const onBlur = (): void => {
    dragging = false;
  };

  if (hasWindow) {
    window.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mouseup', onMouseUp);
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('wheel', onWheel, { passive: true });
    window.addEventListener('blur', onBlur);
  }

  // --- The four views ----------------------------------------------------

  /** One sine of the shake. The three rates share no common factor. */
  function shake(rate: number): number {
    return Math.sin(vibrationTime * rate);
  }

  function updateCockpit(loadFactor: number, speed: number, dt: number): void {
    vibrationTime += dt;
    const amplitude = VIBRATION_BASE + Math.abs(speed) * VIBRATION_PER_SPEED;
    const angleAmplitude =
      VIBRATION_ANGLE_BASE + Math.abs(speed) * VIBRATION_ANGLE_PER_SPEED;

    // The eye sits at a fixed point of the airframe, plus the two small moves
    // of part 3. Render +y is the top of the aircraft, so a positive g load
    // takes the eye down that axis.
    const sink = clamp(
      (loadFactor - 1) * COCKPIT_G_TRAVEL,
      -COCKPIT_G_LIMIT,
      COCKPIT_G_LIMIT,
    );
    offset.copy(eyeModel);
    offset.y -= sink;
    offset.x += amplitude * 0.6 * shake(VIBRATION_RATE_2);
    offset.y += amplitude * shake(VIBRATION_RATE_1);
    offset.applyQuaternion(orientation);
    camera.position.copy(position).add(offset);

    // The head turns inside the aircraft, so the head rotation multiplies the
    // attitude on the right. A positive turn about render +y looks LEFT, so a
    // stick pushed right needs the minus sign.
    headEuler.set(
      headPitch + angleAmplitude * shake(VIBRATION_RATE_1),
      headYaw + angleAmplitude * 0.5 * shake(VIBRATION_RATE_3),
      0,
      'YXZ',
    );
    headQuaternion.setFromEuler(headEuler);
    camera.quaternion.copy(orientation).multiply(headQuaternion);
    camera.up.copy(UP);
  }

  function updateOrbit(): void {
    const cosElevation = Math.cos(orbitElevation);
    offset.set(
      Math.sin(orbitAzimuth) * cosElevation,
      Math.sin(orbitElevation),
      Math.cos(orbitAzimuth) * cosElevation,
    );
    // An azimuth of zero puts the camera on render +z. The nose points along
    // render -z at a heading of north, so the view starts behind the aircraft.
    camera.position.copy(position).addScaledVector(offset, api.orbitDistance);
    camera.up.copy(UP);
    camera.lookAt(position);
  }

  /** Writes the direction of travel into `travel`, flattened toward level. */
  function solveTravel(): void {
    travel.set(0, 0, -1).applyQuaternion(orientation);
    travel.y *= FLYBY_VERTICAL_DAMPING;
    if (travel.lengthSq() < 1e-6) travel.set(0, 0, -1);
    travel.normalize();
  }

  /** Puts the park point ahead of the aircraft and to one side of its track. */
  function repark(speed: number): void {
    solveTravel();
    side.crossVectors(travel, UP);
    if (side.lengthSq() < 1e-6) side.set(1, 0, 0);
    side.normalize();

    const lead = clamp(Math.abs(speed) * FLYBY_LEAD_TIME, FLYBY_MIN_LEAD, FLYBY_MAX_LEAD);
    parkPoint
      .copy(position)
      .addScaledVector(travel, lead)
      .addScaledVector(side, FLYBY_LATERAL * flybySide);
    parkPoint.y = Math.max(position.y - FLYBY_HEIGHT_BELOW, FLYBY_MIN_HEIGHT);

    // The next park point goes to the other side, so two passes never look the
    // same.
    flybySide = -flybySide;
    flybyParked = true;
    camera.position.copy(parkPoint);
    aim.copy(position);
  }

  function updateFlyby(speed: number, dt: number): void {
    if (!flybyParked) {
      repark(speed);
    } else {
      solveTravel();
      offset.subVectors(position, camera.position);
      const passed = travel.dot(offset) > FLYBY_PASS_MARGIN;
      if (passed || offset.length() > FLYBY_MAX_RANGE) repark(speed);
    }

    // The camera holds its park point. Only the aim moves, with a short lag, so
    // a fast pass does not snap the head of the camera around.
    const blend = 1 - Math.pow(0.5, dt / FLYBY_AIM_HALF_LIFE);
    aim.lerp(position, blend);
    camera.position.copy(parkPoint);
    camera.up.copy(UP);
    camera.lookAt(aim);
  }

  /** Reads the look axes into the angles of the view that is running. */
  function readLook(lookYaw: number, lookPitch: number, dt: number): void {
    const dx = mouseDx;
    const dy = mouseDy;
    const wheel = mouseWheel;
    mouseDx = 0;
    mouseDy = 0;
    mouseWheel = 0;

    if (mode === 'orbit') {
      // The stick swings the camera to the side it points at. The mouse drags
      // the world the other way, which is what every orbit control does.
      orbitAzimuth += lookYaw * ORBIT_YAW_RATE * dt - dx * MOUSE_SENSITIVITY;
      orbitElevation = clamp(
        orbitElevation + lookPitch * ORBIT_PITCH_RATE * dt - dy * MOUSE_SENSITIVITY,
        -ORBIT_MAX_ELEVATION,
        ORBIT_MAX_ELEVATION,
      );
      if (wheel !== 0) {
        const factor = Math.pow(ORBIT_ZOOM_STEP, wheel / 100);
        api.orbitDistance = clamp(
          api.orbitDistance * factor,
          ORBIT_MIN_DISTANCE,
          ORBIT_MAX_DISTANCE,
        );
      }
      return;
    }

    // The mouse offset returns to zero once the pilot lets the button up, so
    // the head cannot stay stuck to one side.
    mouseYaw -= dx * MOUSE_SENSITIVITY;
    mousePitch -= dy * MOUSE_SENSITIVITY;
    if (!dragging) {
      const decay = Math.pow(0.5, dt / MOUSE_RETURN_HALF_LIFE);
      mouseYaw *= decay;
      mousePitch *= decay;
    }

    const wantedYaw = clamp(-lookYaw * MAX_HEAD_YAW + mouseYaw, -MAX_HEAD_YAW, MAX_HEAD_YAW);
    const wantedPitch = clamp(
      lookPitch * MAX_HEAD_PITCH + mousePitch,
      -MAX_HEAD_PITCH,
      MAX_HEAD_PITCH,
    );
    const blend = 1 - Math.pow(0.5, dt / HEAD_HALF_LIFE);
    headYaw += (wantedYaw - headYaw) * blend;
    headPitch += (wantedPitch - headPitch) * blend;
  }

  const api: CameraRig = {
    camera,
    orbitDistance: ORBIT_DEFAULT_DISTANCE,

    get mode(): ViewMode {
      return mode;
    },

    set mode(next: ViewMode) {
      if (next === mode) return;
      mode = next;
      api.snap();
    },

    cycle(): void {
      const index = VIEW_ORDER.indexOf(mode);
      api.mode = VIEW_ORDER[(index + 1) % VIEW_ORDER.length];
    },

    snap(): void {
      needsSnap = true;
      headYaw = 0;
      headPitch = 0;
      mouseYaw = 0;
      mousePitch = 0;
      flybyParked = false;
    },

    update(
      aircraftPositionNed: Vector3,
      aircraftOrientationNed: Quaternion,
      loadFactor: number,
      speed: number,
      lookYaw: number,
      lookPitch: number,
      dt: number,
    ): void {
      // One conversion per frame, for every view. See CONVENTIONS section 3.
      nedToThree(aircraftPositionNed, position);
      nedQuatToThree(aircraftOrientationNed, orientation);

      const step = dt > 0 ? dt : 0;
      readLook(lookYaw, lookPitch, step);

      if (mode === 'cockpit') {
        updateCockpit(loadFactor, speed, step);
      } else if (mode === 'chase') {
        if (needsSnap) chase.snap(position, orientation);
        else chase.update(position, orientation, step);
      } else if (mode === 'orbit') {
        updateOrbit();
      } else {
        updateFlyby(speed, step);
      }
      needsSnap = false;
    },

    dispose(): void {
      if (!hasWindow) return;
      window.removeEventListener('mousedown', onMouseDown);
      window.removeEventListener('mouseup', onMouseUp);
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('wheel', onWheel);
      window.removeEventListener('blur', onBlur);
    },
  };

  return api;
}
