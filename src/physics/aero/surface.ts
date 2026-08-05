/**
 * One lifting strip of a wing, a tail, or a fin.
 *
 * A strip is a piece of a lifting surface that is small enough that the flow
 * over it is close to uniform. The model gives each strip its own local air
 * velocity, its own angle of attack, and its own separation state. Every effect
 * that needs a spanwise difference then appears on its own: roll damping, the
 * dihedral effect, the roll off at the stall, and the bite that a slat keeps at
 * the wing tip after the root has stalled. None of those effects has code of its
 * own in this file.
 *
 *
 * FRAMES
 *
 * The body frame follows CONVENTIONS section 3.1, with x forward, y right, and
 * z down. The strip frame is the body frame turned by the incidence about the
 * span axis and then by the dihedral about the chord axis:
 *
 *   R_strip_to_body = Rx(-side * dihedral) * Ry(incidence)
 *
 * `side` is +1 for a strip at or right of the plane of symmetry and -1 for a
 * strip left of it. The sign makes one positive dihedral angle lift both tips,
 * so the geometry of bead b17 carries no sign of its own. A fin is a strip with
 * 90 degrees of dihedral. Its span axis then points up and its normal axis
 * points right, which turns the same code into a fin with no special case.
 *
 * The rotation is built one time in createSurface. The step reads it.
 *
 *
 * SWEEP
 *
 * Simple sweep theory says that only the velocity component normal to the
 * quarter chord line drives the section pressures. The component along that line
 * sweeps over the section and changes nothing. The model therefore splits the
 * strip velocity about the quarter chord line, takes the local angle of attack
 * and the local dynamic pressure from the normal component only, and resolves
 * the answer back into body axes.
 *
 * What this captures. The loss of lift with sweep, because the normal dynamic
 * pressure falls by cos^2 while the normal angle of attack grows by 1 / cos, so
 * the lift falls by cos. The rise of the effective Mach number of the section,
 * which compressibility.ts already handles. The dihedral effect of a swept wing
 * in a sideslip, because the windward panel meets less effective sweep than the
 * leeward panel and therefore makes more lift.
 *
 * What this does not capture. Spanwise flow inside the boundary layer, so the
 * model does not make tip stall from sweep on its own. The washout and the slat
 * of bead b17 hold the tip instead. It does not capture the bending of the
 * isobars near the root and near the tip, so the load near both ends is a little
 * wrong. It does not capture the change of the section that a real wing sees
 * when the sweep changes along the span, because one strip has one sweep angle.
 *
 * The strip area stays the planform area. That is exact, not an approximation:
 * the normal section has chord c cos(sweep) and the strip covers a length
 * span / cos(sweep) of the quarter chord line, so the two cosines cancel and
 * q_normal * planform area * cl is the true strip lift.
 *
 *
 * FLAPS
 *
 * A flap does three things to the section, and the model needs all three. It
 * moves the zero lift angle, it raises the peak lift, and it brings the stall
 * on at a lower geometric angle. A model with the first effect alone gives a
 * flapped wing the SAME peak lift as the clean wing at a lower angle, so the
 * flap makes the aircraft stall sooner and no better. That is backwards: the
 * pilot lowers the flap to fly slower.
 *
 * The measured picture, from a NACA 23012 section with a 25.7 percent slotted
 * flap at 40 degrees. The clean section peaks at 1.79 at 16 degrees. The
 * flapped section peaks at 2.85 at 12.5 degrees and carries 1.35 at zero
 * degrees. Read that in the frame the code works in, which is the angle AFTER
 * the zero lift shift: the flapped peak sits 9 degrees LATER than the clean
 * peak and 1.06 HIGHER. The whole of the 3.5 degree fall of the stall angle
 * comes from the zero lift shift of 12.9 degrees running ahead of that 9
 * degrees. Source: Abbott and von Doenhoff, "Theory of Wing Sections",
 * appendix, and NACA TN 664. Confidence: firm for the shape of the effect.
 *
 * The model follows that picture with two fields.
 *
 *   flapClMaxDelta   Raises the peak by this much. The code moves the section
 *                    curve toward a HIGHER angle and gives the linear part
 *                    back, which is the same mechanism the slat uses, so the
 *                    lift below the stall does not change by one count.
 *   flapAlphaDelta   Takes the stall angle down by this much MORE. The higher
 *                    circulation of a flapped section loads the nose of the
 *                    main element, so leading edge separation arrives a little
 *                    before the curve shift alone predicts. The code translates
 *                    the curve back and adds the peak the translation took, so
 *                    the peak keeps the value flapClMaxDelta names.
 *
 * The peak term rides a window that is zero while the flow is attached and zero
 * again once the section works as a flat plate. It therefore changes nothing in
 * the linear range, where the zero lift shift is already exact, and it does not
 * carry a flap increment into a fully separated section, which has no slot flow
 * left to keep.
 *
 * What the model does not do. StripGeometry.clMax still holds the CLEAN peak of
 * the section, so the Mach peak loss below works from the clean value. That
 * costs nothing here, because no aircraft flies with the flap down near the
 * drag rise Mach number.
 *
 *
 * THE CENTER OF PRESSURE
 *
 * Two effects move the load of a section along its chord, and the Mach tuck of
 * this aircraft is one of them.
 *
 *   The Mach shift. A shock on the upper surface carries the load aft. This is
 *   the tuck. compressibility.ts reports the new position as acShift, in chord
 *   fractions from the leading edge.
 *   The separation shift. A section that sheds its trailing edge flow carries
 *   its load back toward mid chord. Only the LAG of that move appears here,
 *   because the static section table already holds the steady part of it.
 *
 * THE MODEL MOVES THE POINT THE FORCE ACTS AT. It does not add a couple. The two
 * are not the same on a swept strip, and the difference is a factor of
 * cos^2(sweep), which is 0.90 on this wing.
 *
 * The reason. Simple sweep theory puts the load of the section a fraction x_cp
 * of the NORMAL chord behind the leading edge, so the line of the load runs
 * parallel to the quarter chord line at a perpendicular distance x_cp c_n. Two
 * parallel lines swept by an angle stand (perpendicular distance / cos sweep)
 * apart when they are measured STREAMWISE, and c_n / cos(sweep) is the
 * streamwise chord. The load at one span station therefore sits x_cp of the
 * STREAMWISE chord behind the leading edge, and the arm about the center of
 * gravity carries the streamwise chord and no cosine at all.
 *
 * A couple about the quarter chord line, built from the normal chord, loses one
 * cosine to the chord and a second one when its axis is resolved onto the pitch
 * axis of the aircraft. It also makes a rolling moment that a load moving aft at
 * a fixed span station does not make. The section moment of the airfoil table is
 * a true couple and keeps that treatment. The travel of the center of pressure
 * does not.
 *
 *
 * THE INDUCED ANGLE
 *
 * Strip theory with two dimensional section data makes no induced drag and
 * overstates the lift curve slope. The fix is one angle. The caller passes the
 * induced angle of the parent surface, the strip takes it off its local flow
 * angle, and the strip then builds the lift perpendicular to the flow that is
 * left. The lift vector leans back by the induced angle, and that lean IS the
 * induced drag. No drag term gets added by hand.
 *
 * src/physics/aero/assembly.ts owns the value of the angle, because the angle
 * belongs to the whole parent surface and not to one strip. This file only
 * applies it. The angle arrives as a free stream angle, so the strip divides it
 * by cos(sweep) to carry it into the normal plane, which is the same factor the
 * angle of attack takes.
 *
 *
 * ALLOCATION
 *
 * evaluateSurface runs about thirty times per step, four times per step inside
 * the Runge-Kutta stages, at 240 Hz. It allocates nothing. Every scratch vector
 * lives in module scope. One aircraft steps at one time and the physics runs on
 * one thread, so the shared scratch is safe.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Matrix3, Vector3 } from 'three';

import { clamp, smoothstep } from '@/math/tables';
import { DEG } from '@/math/units';
import { machNumber } from '@/physics/atmosphere';
import type { AeroCoefficients, Airfoil } from '@/physics/aero/airfoil';
import type { MachCorrection } from '@/physics/aero/compressibility';
import { createMachCorrection, machCorrection } from '@/physics/aero/compressibility';
import type { StallParams, StallState } from '@/physics/aero/stall';
import {
  createStallState,
  kirchhoffLift,
  separationCenterOfPressure,
  steadySeparation,
  updateSeparation,
} from '@/physics/aero/stall';
import type { Wrench } from '@/physics/rigidbody';

/** The geometry and the section data of one strip. The caller builds it one time. */
export interface SurfaceDef {
  name: string;
  /** Body axes, meters from the center of gravity, at the strip quarter chord. */
  position: Vector3;
  /** Meters, the strip width along the span. */
  span: number;
  /** Meters, the streamwise chord. */
  chord: number;
  /** Square meters, the planform area of the strip. */
  area: number;
  /** rad, built in twist plus rigging angle, positive leading edge up. */
  incidence: number;
  /** rad, positive tip up. The code takes the side from the sign of position.y. */
  dihedral: number;
  /** rad, quarter chord sweep, positive aft. */
  sweep: number;
  airfoil: Airfoil;
  stall: StallParams;
  /** Aspect ratio of the PARENT surface, for the finite span correction. */
  aspectRatio: number;
  /** Oswald efficiency of the PARENT surface. */
  oswaldEfficiency: number;
  /** Index into the control deflection array. Use -1 for none. */
  controlIndex: number;
  /**
   * d(alphaZeroLift) / d(deflection), the flap effectiveness tau.
   *
   * The magnitude is the classic tau, near 0.45 for a plain flap of 25 percent
   * chord. The sign follows from the definition: this value RAISES the zero lift
   * angle, so a positive value with a positive deflection LOWERS the lift. A
   * surface that must gain lift when its deflection goes positive therefore
   * takes a negative value here. Bead b17 owns the sign of every control.
   */
  controlEffectiveness: number;
  /** Index into the control deflection array for the flap. Use -1 for none. */
  flapIndex: number;
  /** d(alphaZeroLift) / d(flap deflection). Same sign rule as controlEffectiveness. */
  flapEffectiveness: number;
  /**
   * d(peak lift of the section) / d(flap deflection), per radian.
   *
   * A POSITIVE value raises the peak when the deflection goes positive, which is
   * the sign a trailing edge flap that lowers on a positive command takes. The
   * sign rule is therefore the OPPOSITE of controlEffectiveness, because this
   * field names a lift and that one names an angle.
   *
   * Scale the value by the fraction of the strip that the flap covers, exactly
   * as flapEffectiveness is scaled. See the FLAPS note in the module comment.
   */
  flapClMaxDelta: number;
  /**
   * Extra fall of the stall angle of the section, rad per radian of flap
   * deflection. A POSITIVE value brings the stall on EARLIER.
   *
   * This is the part of the fall that the zero lift shift and the peak rise do
   * not already give. See the FLAPS note in the module comment.
   */
  flapAlphaDelta: number;
  hasSlat: boolean;
  /** rad, how much the open slat raises the stall angle. */
  slatAlphaDelta: number;
  /** rad, the local angle of attack where the slat opens. */
  slatDeployAlpha: number;
}

/** The state of one strip. The caller keeps it between steps. */
export interface SurfaceState {
  stall: StallState;
  /** The section angle of attack of the last evaluation, rad. */
  lastAlpha: number;
  /** The body axis force of the last evaluation, newtons. */
  lastForce: Vector3;
}

/** What one evaluation produced. The debug view and the tests read it. */
export interface SurfaceResult {
  /** rad, the angle the section works at, after the control shift and the induced angle. */
  alpha: number;
  /** rad, the local sideslip of the strip, in the strip frame. */
  beta: number;
  /** Pa, from the velocity component normal to the quarter chord line. */
  dynamicPressure: number;
  /** m/s, the magnitude of the local air velocity. */
  speed: number;
  cl: number;
  cd: number;
  /**
   * Pitching moment of the strip about its quarter chord, on the STREAMWISE
   * chord. It holds the section couple and the travel of the center of pressure.
   * An unswept strip reports the section value, as it always did.
   */
  cm: number;
  /** The lagged separation point, 1 attached and 0.04 fully separated. */
  separation: number;
  slatOpen: boolean;
  /** Body axes, newtons. */
  force: Vector3;
  /** Body axes about the center of gravity, newton meters. */
  moment: Vector3;
}

/**
 * The fixed geometry that createSurface works out one time.
 *
 * This sits on the public object because the physics step must not allocate and
 * must not close over hidden state. Treat it as read only. Nothing outside this
 * module and assembly.ts needs it.
 */
export interface StripGeometry {
  /** Strip axes to body axes. */
  readonly toBody: Matrix3;
  /** Body axes to strip axes. */
  readonly toStrip: Matrix3;
  /** +1 for a strip at or right of the plane of symmetry, -1 for a strip left of it. */
  readonly side: number;
  readonly cosSweep: number;
  readonly sinSweep: number;
  /** The chord of the section normal to the quarter chord line, meters. */
  readonly normalChord: number;
  /** The peak lift coefficient of the section, read from the airfoil table one time. */
  readonly clMax: number;
  /** 1 / (PI * e * AR) of the parent surface, from the def. */
  readonly inducedFactor: number;
}

export interface Surface {
  readonly def: SurfaceDef;
  readonly state: SurfaceState;
  readonly result: SurfaceResult;
  readonly geometry: StripGeometry;
}

/** The linear load estimate that the induced angle solve of the assembly needs. */
export interface SurfaceLoad {
  /** Newtons of lift at zero induced angle. */
  lift: number;
  /** Newtons of lift lost per radian of induced angle. Never negative. */
  slope: number;
}

// The angle band over which a slat opens, above slatDeployAlpha. A real
// pressure operated slat runs out over about one degree of angle of attack. The
// band here is wider on purpose, so that the force stays smooth at 240 Hz.
// Confidence: estimate.
const SLAT_DEPLOY_BAND = 2 * DEG; // rad

// The section drag an open slat adds. The slat opens a slot, exposes its own
// surface, and thickens the effective nose. Hoerner gives 0.015 to 0.03 on the
// section drag coefficient for an extended leading edge device of this size.
// Source: Hoerner, "Fluid Dynamic Drag", chapter 6. Confidence: estimate.
const SLAT_OPEN_DRAG = 0.02;

// Where the flap peak term starts to come in, as a fraction of the stall angle
// of the clean section. The value matches SEPARATION_ONSET_FRACTION of
// airfoil.ts, which is the angle where the clean section starts to leave its
// straight line. Below it the zero lift shift already holds the whole of the
// flap lift, so the peak term must be zero there.
const FLAP_PEAK_ONSET = 0.8;

// The band above the stall angle over which the flap peak term fades out. The
// value matches FLAT_PLATE_BAND of airfoil.ts. A section that works as a flat
// plate has no slot flow left, so it keeps none of the flap increment.
const FLAP_PEAK_FADE_BAND = 12 * DEG; // rad

// The aerodynamic center of an attached section, in chord fractions. The Mach
// correction reports its shifted value against this one.
const X_AC_ATTACHED = 0.25;

// The lift coefficient where the Mach peak lift loss starts to act, as a
// fraction of the peak. Below it the shock has no separated flow to work on.
const CL_MAX_KNEE = 0.5;

/**
 * Below this speed the flow angles of a strip carry no information, m/s.
 *
 * alpha is atan2 of the two components of the local flow, and beta is the asin
 * of a third one over their length. At rest all three are the settling motion of
 * the struts and nothing else, so the ANGLE between them is numerical noise: a
 * parked aircraft reported 20 degrees on one strip and 12 on the next, from a
 * local speed of 2 nanometers per second. Anything that reads SurfaceResult.alpha
 * then paints a standing aircraft as a stalled one.
 *
 * The two angles therefore report ZERO below this speed, and not the value of
 * the last evaluation. Zero is what the flow really is: no flow, no angle. A
 * held value would be worse, because it would freeze whatever angle the aircraft
 * carried into the last moment before it stopped, and a wreck lying on the
 * runway would report the angle it hit the ground at for ever.
 *
 * The value costs nothing. At 1 m/s the dynamic pressure is 0.61 Pa, which is
 * about one newton over the whole aircraft, or a hundred-thousandth of its
 * weight. The rule matches flowAngles of src/physics/rigidbody.ts, which already
 * reports zero for the aircraft as a whole below a speed of its own.
 */
const MIN_FLOW_SPEED = 1; // m/s

// Scratch held in module scope. The step allocates nothing.
const localVelocity = new Vector3();
const stripVelocity = new Vector3();
const stripForce = new Vector3();
const stripMoment = new Vector3();
const bodyForce = new Vector3();
const bodyMoment = new Vector3();
const armMoment = new Vector3();
const cpOffset = new Vector3();
const coefficients: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
const mach: MachCorrection = createMachCorrection();

/**
 * Returns the Kirchhoff separation factor g(f) = ((1 + sqrt(f)) / 2)^2.
 *
 * kirchhoffLift returns clAlpha * g(f) * (alpha - alphaZeroLift), so a call with
 * a slope of 1 and an angle of 1 above the zero lift angle returns g(f) alone.
 * The ratio of two such calls is the lagged to steady ratio that stall.ts asks
 * for, with the angle factor already cancelled. The cancellation matters: a
 * direct ratio of two lift values divides zero by zero at the zero lift angle.
 */
function separationFactor(f: number): number {
  return kirchhoffLift(1, 1, 0, f);
}

/**
 * Returns the weight of the flap peak term at one table angle.
 *
 * The weight is zero while the clean section holds its straight line, one at
 * the peak of the clean section, and zero again once the section works as a
 * flat plate. Both joins use smoothstep, and smoothstep has zero slope at both
 * ends, so the weight has zero slope at the peak. The peak of the clean table
 * has zero slope as well, so the sum of the two still peaks exactly there.
 */
function flapPeakWindow(absAlphaTable: number, alphaStall: number): number {
  const rise = smoothstep(FLAP_PEAK_ONSET * alphaStall, alphaStall, absAlphaTable);
  const fade = smoothstep(alphaStall, alphaStall + FLAP_PEAK_FADE_BAND, absAlphaTable);
  return rise * (1 - fade);
}

/** Builds one strip and works out its fixed geometry. */
export function createSurface(def: SurfaceDef): Surface {
  if (!(def.area > 0)) {
    throw new Error(`Surface ${def.name} needs a positive area. It got ${def.area}.`);
  }
  if (!(def.chord > 0)) {
    throw new Error(`Surface ${def.name} needs a positive chord. It got ${def.chord}.`);
  }
  if (!(def.aspectRatio > 0) || !(def.oswaldEfficiency > 0)) {
    throw new Error(
      `Surface ${def.name} needs a positive aspect ratio and a positive Oswald ` +
        `efficiency. It got ${def.aspectRatio} and ${def.oswaldEfficiency}.`,
    );
  }

  const side = def.position.y < 0 ? -1 : 1;
  const dihedralAngle = -side * def.dihedral;
  const ci = Math.cos(def.incidence);
  const si = Math.sin(def.incidence);
  const cd = Math.cos(dihedralAngle);
  const sd = Math.sin(dihedralAngle);

  // The columns are the strip axes written in body axes. The product of
  // Rx(dihedralAngle) and Ry(incidence) gives them directly.
  const xs = new Vector3(ci, si * sd, -si * cd);
  const ys = new Vector3(0, cd, sd);
  const zs = new Vector3(si, -ci * sd, ci * cd);
  const toBody = new Matrix3().set(xs.x, ys.x, zs.x, xs.y, ys.y, zs.y, xs.z, ys.z, zs.z);
  const toStrip = toBody.clone().transpose();

  // The peak lift of the section. The Airfoil interface reports the stall angle
  // but not the peak value, so read the table one time here.
  const peak: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
  def.airfoil.sample(def.airfoil.alphaStall, peak);

  const cosSweep = Math.cos(def.sweep);
  return {
    def,
    state: { stall: createStallState(), lastAlpha: 0, lastForce: new Vector3() },
    result: {
      alpha: 0,
      beta: 0,
      dynamicPressure: 0,
      speed: 0,
      cl: 0,
      cd: 0,
      cm: 0,
      separation: 1,
      slatOpen: false,
      force: new Vector3(),
      moment: new Vector3(),
    },
    geometry: {
      toBody,
      toStrip,
      side,
      cosSweep,
      sinSweep: Math.sin(def.sweep),
      normalChord: def.chord * Math.abs(cosSweep),
      clMax: Math.abs(peak.cl),
      inducedFactor: 1 / (Math.PI * def.oswaldEfficiency * def.aspectRatio),
    },
  };
}

/**
 * Puts one strip back to the state createSurface left it in.
 *
 * The strip carries a lagged separation point between steps. A value that is not
 * finite can never leave that state on its own, because every later step reads
 * it, so a caller that has to recover from a diverged step must clear it. The
 * aircraft calls this on a respawn and after it catches a state that is not
 * finite. Nothing else needs it.
 */
export function resetSurface(s: Surface): void {
  s.state.stall.f = 1;
  s.state.lastAlpha = 0;
  s.state.lastForce.set(0, 0, 0);
  const r = s.result;
  r.alpha = 0;
  r.beta = 0;
  r.dynamicPressure = 0;
  r.speed = 0;
  r.cl = 0;
  r.cd = 0;
  r.cm = 0;
  r.separation = 1;
  r.slatOpen = false;
  r.force.set(0, 0, 0);
  r.moment.set(0, 0, 0);
}

/**
 * Writes the local air velocity of one strip into localVelocity, in body axes,
 * and then into stripVelocity, in strip axes.
 *
 * The velocity is the velocity of the strip through the air mass:
 *
 *   v_local = v_body + omega x r - wind_body
 *
 * This is the same sense that airspeedBody in rigidbody.ts uses, so forward
 * flight gives a positive x component. The omega x r term is the whole of the
 * rate damping of the aircraft. A positive roll rate moves the right wing down,
 * so that wing meets air from below, its angle of attack grows, and its lift
 * opposes the roll.
 */
function localFlow(
  position: Vector3,
  velocityBody: Vector3,
  angularVelocity: Vector3,
  windBody: Vector3,
  toStrip: Matrix3,
): void {
  localVelocity.crossVectors(angularVelocity, position).add(velocityBody).sub(windBody);
  stripVelocity.copy(localVelocity).applyMatrix3(toStrip);
}

/**
 * Returns the velocity component normal to the quarter chord line, in the plane
 * of the strip.
 *
 * The unit vector along the quarter chord line is (-side sin, cos, 0) in strip
 * axes. Removing that component from the strip velocity leaves
 * u_n * (cos, side sin, 0) + w * z, with u_n as returned here. The mirror on the
 * left panel matters: without the side factor both panels would answer a
 * sideslip in the same way and the swept wing would lose its dihedral effect.
 */
function normalChordwiseSpeed(g: StripGeometry): number {
  return stripVelocity.x * g.cosSweep + g.side * stripVelocity.y * g.sinSweep;
}

/**
 * Adds the force and the moment of one strip into out, in body axes, and fills
 * the result of the strip.
 *
 * inducedAngle is the downwash angle of the PARENT surface, in radians, in free
 * stream terms. The assembly works it out. Pass 0 to read the two dimensional
 * answer.
 *
 * The function adds into out. The caller clears the wrench one time and then
 * runs every element into it.
 */
export function evaluateSurface(
  s: Surface,
  velocityBody: Vector3,
  angularVelocity: Vector3,
  windBody: Vector3,
  density: number,
  speedOfSound: number,
  controls: Float64Array,
  inducedAngle: number,
  dt: number,
  out: Wrench,
): void {
  const def = s.def;
  const g = s.geometry;
  const r = s.result;

  localFlow(def.position, velocityBody, angularVelocity, windBody, g.toStrip);
  const speed = localVelocity.length();
  const un = normalChordwiseSpeed(g);
  const w = stripVelocity.z;
  const normalSpeedSquared = un * un + w * w;
  const normalSpeed = Math.sqrt(normalSpeedSquared);
  const q = 0.5 * density * normalSpeedSquared;

  // The angle of attack of the section normal to the quarter chord line. Below
  // MIN_FLOW_SPEED there is no flow to take an angle from, so it reports zero.
  const alphaGeometric = normalSpeed > MIN_FLOW_SPEED ? Math.atan2(w, un) : 0;
  const stripSpeed = stripVelocity.length();
  const beta =
    stripSpeed > MIN_FLOW_SPEED ? Math.asin(clamp(stripVelocity.y / stripSpeed, -1, 1)) : 0;

  machCorrection(machNumber(speed, speedOfSound), def.sweep, mach, def.airfoil.thickness);

  // A control and a flap move the zero lift angle of the section. The Mach
  // correction takes their power away as the shock reaches the hinge line.
  let alphaZeroLift = 0;
  let flapDeflection = 0;
  if (def.controlIndex >= 0 && def.controlIndex < controls.length) {
    alphaZeroLift += def.controlEffectiveness * controls[def.controlIndex] * mach.controlScale;
  }
  if (def.flapIndex >= 0 && def.flapIndex < controls.length) {
    flapDeflection = controls[def.flapIndex] * mach.controlScale;
    alphaZeroLift += def.flapEffectiveness * flapDeflection;
  }

  // The induced angle arrives in free stream terms and the strip works in the
  // normal plane, so it takes the same 1 / cos(sweep) that the angle of attack
  // takes. alphaFlow is the direction of the air that reaches the section, and
  // the force directions below use it. alphaSection adds the camber that the
  // control makes, and the section tables use that one.
  const alphaFlow = alphaGeometric - inducedAngle / g.cosSweep;
  const alphaSection = alphaFlow - alphaZeroLift;

  // A slat holds the flow on at a higher angle. The model shifts the whole
  // section curve toward a higher angle by slatAlphaDelta and gives the linear
  // part back, so the lift below the stall does not change and both the stall
  // angle and the peak lift rise by exactly the shift.
  let slatOpening = 0;
  let slatShift = 0;
  if (def.hasSlat) {
    slatOpening = smoothstep(
      def.slatDeployAlpha,
      def.slatDeployAlpha + SLAT_DEPLOY_BAND,
      Math.abs(alphaSection),
    );
    slatShift = (alphaSection < 0 ? -1 : 1) * def.slatAlphaDelta * slatOpening;
  }

  // The flap. See the FLAPS note in the module comment. The peak rise moves the
  // curve toward a higher angle, the extra stall angle fall moves it back, and
  // the peak term returns what that second move took off the peak.
  let flapShift = 0;
  let flapPeakAdd = 0;
  if (flapDeflection !== 0) {
    const side = alphaSection < 0 ? -1 : 1;
    const peakShift = (def.flapClMaxDelta * flapDeflection) / def.airfoil.clAlpha;
    const stallShift = def.flapAlphaDelta * flapDeflection;
    flapShift = side * (peakShift - stallShift);
    flapPeakAdd = side * def.airfoil.clAlpha * stallShift;
  }

  const alphaTable = alphaSection - slatShift - flapShift;

  def.airfoil.sample(alphaTable, coefficients);
  let cl = coefficients.cl + def.airfoil.clAlpha * (slatShift + flapShift);
  if (flapPeakAdd !== 0) {
    cl += flapPeakAdd * flapPeakWindow(Math.abs(alphaTable), def.airfoil.alphaStall);
  }
  cl *= mach.clScale;

  // Shock induced separation takes the peak away. The loss acts on the peak
  // only, so the linear part of the curve keeps its Prandtl-Glauert value.
  if (mach.clMaxScale < 1) {
    const peak = g.clMax * mach.clScale;
    const t = smoothstep(CL_MAX_KNEE * peak, peak, Math.abs(cl));
    cl *= 1 + (mach.clMaxScale - 1) * t;
  }

  // Dynamic stall. stall.ts asks for the lagged to steady ratio on top of the
  // static table, not for a replacement of it. The ratio is 1 in steady flow at
  // every angle, so the static table stays in charge.
  const steadyF = steadySeparation(alphaTable, def.stall);
  const laggedF = updateSeparation(s.state.stall, alphaTable, g.normalChord, normalSpeed, dt, def.stall);
  const steadyFactor = separationFactor(steadyF);
  cl *= steadyFactor > 1e-9 ? separationFactor(laggedF) / steadyFactor : 1;

  const cd = coefficients.cd + mach.cdAdd + SLAT_OPEN_DRAG * slatOpening;

  const cosFlow = Math.cos(alphaFlow);
  const sinFlow = Math.sin(alphaFlow);
  const cn = cl * cosFlow + cd * sinFlow;

  // WHERE THE LOAD ACTS. See the CENTER OF PRESSURE note in the module comment.
  // The Mach shift is the tuck. The separation shift is the unsteady part of the
  // nose down break, and it carries the lag only, because the static table
  // already holds the steady movement.
  const xCp =
    mach.acShift + separationCenterOfPressure(laggedF) - separationCenterOfPressure(steadyF);
  const cpTravel = (xCp - X_AC_ATTACHED) * def.chord; // m, aft of the quarter chord

  // The lift acts perpendicular to the flow that reaches the section, which the
  // induced angle has already tilted. The lean of that vector is the induced
  // drag. The drag acts along the relative wind, which is the negative of the
  // velocity of the strip through the air.
  const lift = q * def.area * cl;
  const drag = q * def.area * cd;
  const alongChord = lift * sinFlow - drag * cosFlow;
  const alongNormal = -lift * cosFlow - drag * sinFlow;
  stripForce.set(alongChord * g.cosSweep, alongChord * g.side * g.sinSweep, alongNormal);
  bodyForce.copy(stripForce).applyMatrix3(g.toBody);

  // The section moment turns about the quarter chord line. A positive cm is nose
  // up, which is a positive y moment when the sweep and the dihedral are zero.
  const pitching = q * def.area * g.normalChord * coefficients.cm;
  stripMoment.set(-g.side * g.sinSweep * pitching, g.cosSweep * pitching, 0);
  bodyMoment.copy(stripMoment).applyMatrix3(g.toBody);

  // The travel of the center of pressure. The load leaves the quarter chord and
  // acts cpTravel meters AFT of it, along the chord line of the strip, which is
  // the strip x axis. The chordwise part of the force runs along that same line
  // and therefore adds nothing here, so the whole force may go into the cross
  // product with no separate normal force term.
  cpOffset.set(-cpTravel, 0, 0).applyMatrix3(g.toBody);
  bodyMoment.add(armMoment.crossVectors(cpOffset, bodyForce));
  bodyMoment.add(armMoment.crossVectors(def.position, bodyForce));

  out.force.add(bodyForce);
  out.moment.add(bodyMoment);

  r.alpha = alphaSection;
  r.beta = beta;
  r.dynamicPressure = q;
  r.speed = speed;
  r.cl = cl;
  r.cd = cd;
  // The pitching moment of the whole strip about its quarter chord, on the
  // streamwise chord. The first term is the section couple, which turns about
  // the quarter chord LINE, so it reaches the pitch axis with one factor of
  // cos(sweep) and carries the normal chord, which holds a second one.
  r.cm = (g.cosSweep * g.cosSweep * coefficients.cm) - (xCp - X_AC_ATTACHED) * cn;
  r.separation = laggedF;
  r.slatOpen = slatOpening > 0;
  r.force.copy(bodyForce);
  r.moment.copy(bodyMoment);

  s.state.lastAlpha = alphaSection;
  s.state.lastForce.copy(bodyForce);
}

/**
 * Writes a linear estimate of the strip lift into out, without touching the
 * stall state and without reading the section tables.
 *
 * The estimate is lift(alphaInduced) = out.lift - out.slope * alphaInduced. The
 * assembly sums both terms over a parent surface and solves for the induced
 * angle in closed form. See the comment in assembly.ts for the choice.
 *
 * The estimate uses the Kirchhoff law with the separation point THE CALLER
 * PASSES. In attached flow that is exactly what evaluateSurface produces,
 * because the static table is the same Kirchhoff law and the lagged to steady
 * ratio cancels the steady factor. Above the flat plate blend the estimate runs
 * high, which pushes the induced angle a little high, near one degree at 30
 * degrees of angle of attack. Linear induced angle theory has no meaning there
 * in any case.
 *
 * WHY THE SEPARATION POINT IS AN ARGUMENT. It used to read s.state.stall.f,
 * which is the value the PREVIOUS call of evaluateSurface left behind. The
 * induced angle solve of assembly.ts then depended on what ran before it and not
 * on the state it received, and near the stall the same state gave two answers
 * that differed by a factor of four. That was bead b61. The assembly now hands
 * over the separation point that its own full pass will use, and it iterates the
 * pair to their common fixed point. See the module comment of assembly.ts.
 */
export function estimateSurfaceLoad(
  s: Surface,
  velocityBody: Vector3,
  angularVelocity: Vector3,
  windBody: Vector3,
  density: number,
  speedOfSound: number,
  controls: Float64Array,
  separation: number,
  out: SurfaceLoad,
): void {
  const def = s.def;
  const g = s.geometry;

  localFlow(def.position, velocityBody, angularVelocity, windBody, g.toStrip);
  const speed = localVelocity.length();
  const un = normalChordwiseSpeed(g);
  const w = stripVelocity.z;
  const q = 0.5 * density * (un * un + w * w);

  machCorrection(machNumber(speed, speedOfSound), def.sweep, mach, def.airfoil.thickness);

  // The two flap peak fields do not appear here. Both act on the peak of the
  // section and both are zero while the flow is attached, and this estimate is
  // the attached straight line that the induced angle solve needs.
  let alphaZeroLift = 0;
  if (def.controlIndex >= 0 && def.controlIndex < controls.length) {
    alphaZeroLift += def.controlEffectiveness * controls[def.controlIndex] * mach.controlScale;
  }
  if (def.flapIndex >= 0 && def.flapIndex < controls.length) {
    alphaZeroLift += def.flapEffectiveness * controls[def.flapIndex] * mach.controlScale;
  }

  // The same MIN_FLOW_SPEED rule that evaluateSurface follows. One rule, one
  // answer: the estimate must agree with the evaluation it is an estimate of.
  const alphaGeometric = Math.hypot(un, w) > MIN_FLOW_SPEED ? Math.atan2(w, un) : 0;
  const slope = def.airfoil.clAlpha * mach.clScale * separationFactor(separation);
  out.lift = q * def.area * slope * (alphaGeometric - alphaZeroLift);
  out.slope = (q * def.area * slope) / g.cosSweep;
}
