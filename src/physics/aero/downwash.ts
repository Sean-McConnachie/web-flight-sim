/**
 * Downwash at the horizontal tail and sidewash at the fin.
 *
 * The wing trails a vortex sheet. That sheet pushes the flow down at the tail,
 * so the tail meets a smaller angle of attack than the wing. This is the single
 * largest correction to the longitudinal stability and to the elevator power of
 * the aircraft. The fuselage and the wing also turn the flow sideways at the
 * fin, so the fin meets a sideslip of its own that is not the free stream one.
 *
 *
 * 1. WHERE THE DOWNWASH COMES FROM
 *
 * The classic estimate is
 *
 *   epsilon = epsilon0 + (d epsilon / d alpha) alpha
 *
 * with the slope of an elliptically loaded wing at 2 CL_alpha / (PI AR), or the
 * empirical 4 / (AR + 2). For the Me 262 wing, with a finite span lift curve
 * slope of 4.74 per radian and an aspect ratio of 7.21, the two give 0.419 and
 * 0.434 per radian. They agree to within four percent.
 *
 * THE CHOICE. This module uses the first form, and it drives it from the wing
 * LIFT that the assembly solved this step, not from a fixed slope times alpha:
 *
 *   epsilon = WAKE_FACTOR * CL_wing / (PI * AR_wing)
 *
 * Two reasons. First, a fixed slope is wrong past the stall, which is where the
 * downwash matters most. When the wing stalls its lift collapses, the downwash
 * collapses with it, and the tail suddenly meets a much larger angle of attack.
 * That is a real effect at the stall break and a fixed slope hides it. Second,
 * the flaps and the wing incidence enter the lift on their own, so epsilon0 and
 * the change of epsilon0 with flap angle come out of the model instead of going
 * into it as constants.
 *
 * WHERE THE WING LIFT COMES FROM. The induced angle solve of assembly.ts closes
 *
 *   alpha_i = CL / (PI e AR)
 *
 * exactly, so the lift coefficient the assembly settled on this step is
 * PI e AR alpha_i, with alpha_i the value the solve already wrote into the
 * induced angle array. No second lift estimate is necessary and no value from
 * the last step is necessary. The value is a function of the state that reached
 * evaluate, which is what the Runge-Kutta integrator needs.
 *
 *
 * 2. THE DYNAMIC PRESSURE AT THE TAIL
 *
 * The tail flies in the wake of the wing, so it meets less than the free stream
 * dynamic pressure. The model carries a tail efficiency eta_h. A strip cannot
 * take a dynamic pressure of its own: surface.ts builds the lift out of the
 * local flow and takes one angle from the caller. The model therefore turns the
 * pressure loss into the angle that leaves the same lift. groupAngle below holds
 * that step and its algebra.
 *
 * The angle the solve reports carries the elevator, the tailplane incidence and
 * the pitch rate with it, because all three are inside the lift that the solve
 * reported. One fixed angle could carry none of them, and the test that the tail
 * efficiency must cut the elevator power would then fail.
 *
 * WHAT THIS COSTS. The tail drag and the tail stall angle answer the equivalent
 * angle instead of the true dynamic pressure. Both errors are of the order of
 * (1 - eta), which is under a tenth of the tail angle, and neither drives the
 * handling.
 *
 *
 * 3. WHERE THE WAKE SITS
 *
 * The wake leaves the wing and travels aft along the local flow. In body axes
 * that path RISES at the angle alpha - epsilon, because the air moves aft and up
 * over an aircraft at a positive angle of attack. The Me 262 tailplane sits
 * 0.64 m above the wing chord plane and 3.32 m behind the wing quarter chord, so
 * the wake center line reaches the tail when
 *
 *   tan(alpha - epsilon) = 0.64 / 3.32,  that is  alpha - epsilon = 10.9 deg.
 *
 * The model measures the crossing at 18 degrees, and the wake of the broken wing
 * is thick enough to cover the tail from 14 degrees. THE WAKE THEREFORE DOES
 * COVER THIS TAIL, and it does so from the stall break upward. In normal flight
 * the tail stays clear. Below 8 degrees the wake center line runs more than
 * 0.39 m below the tail, and the wake of an attached wing reaches only 0.11 m.
 *
 * That is not a deep stall trap and the model does not build one. A trap needs a
 * tail that stays blanketed while the aircraft holds a high angle of attack with
 * no way to lower the nose, which is a T tail problem. The Me 262 tail sits low,
 * at the base of the fin, so the wake center line climbs PAST it above 18
 * degrees instead of sitting on it. The tail loses about a third of its dynamic
 * pressure and not all of it, and it holds a positive angle of attack and a
 * nose down moment at every angle the model was measured at, out to 30 degrees.
 * The published behavior matches: the aircraft buffeted, the nose dropped, and
 * the elevator went soft. The model gives all three and gives back no trap.
 *
 *
 * 4. SIDEWASH AT THE FIN
 *
 * The fin meets beta (1 - d sigma / d beta), and eta_v scales its dynamic
 * pressure. The fin strips carry 90 degrees of dihedral, so their local angle of
 * attack IS the local sideslip, and one added angle serves for both. The
 * pressure loss uses the same equivalent angle as the tail.
 *
 * For THIS aircraft the turn is favorable. The DATCOM fit at SIDEWASH_SLOPE
 * gives eta_v (1 - d sigma / d beta) = 1.01 for a mid wing with a fin of this
 * relative area, so the fin meets about 6 percent more sideslip than the free
 * stream and gives about 5 percent of it back to the boundary layer of the
 * fuselage. The pair very nearly cancels.
 *
 *
 * 5. THE LAG
 *
 * The wake needs l_tail / V seconds to reach the tail, near 0.03 s at 120 m/s.
 * The module holds the first order lag but leaves it OFF by default, because
 * stepRK4 calls evaluate four times per step at four different stage states. A
 * lagged value carried between those calls comes from another stage and breaks
 * the derivative that the integrator needs, exactly as the module comment of
 * assembly.ts states for the induced angle. The lag is 3 percent of the short
 * period and it changes no steady result. A caller that runs one evaluation per
 * step, such as a frequency response test, can turn it on.
 *
 * This module is pure physics. It imports no Three.js class at all.
 */

import { clamp, lerp, smoothstep } from '@/math/tables';
import type { GroupDef } from '@/physics/aero/assembly';
import type { Surface } from '@/physics/aero/surface';

/**
 * Ratio of the downwash at the tail to the induced angle at the wing.
 *
 * Lifting line theory gives 2 far behind the wing. One horseshoe vortex of this
 * span would give only 1 + x / sqrt(x^2 + s^2) = 1.47 at a tail arm of 3.32 m
 * and a semi span of 6.26 m, so the near field question is a real one. The
 * measurement answers it. The far field form gives a slope of 0.419 per radian
 * for this wing, and the empirical fit 4 / (AR + 2), which is a regression over
 * measured aircraft, gives 0.434. The two agree to four percent, so the far
 * field value is the one that matches the aircraft that were flown. A real wake
 * is a rolled up sheet and not one horseshoe, and it reaches its far field value
 * far sooner than one horseshoe does. Source: Prandtl lifting line theory,
 * Etkin, "Dynamics of Flight", chapter 3, and Roskam, "Airplane Design Part VI".
 * Confidence: firm.
 */
const WAKE_FACTOR = 2.0;

/**
 * Dynamic pressure ratio at a tail that sits clear of the wake core. The loss
 * carries the fuselage boundary layer and the far field of the wing wake.
 * Source: Perkins and Hage give 0.85 to 0.95 for a tail in this position, and
 * this tail sits high and clear of the nacelles. Confidence: estimate.
 */
const ETA_TAIL_CLEAN = 0.92;

/**
 * Dynamic pressure ratio at a tail inside the core of the wake, with the wing
 * attached and with the wing fully separated.
 *
 * An attached wing leaves a velocity deficit near 10 percent two chords behind
 * it, and a stalled wing leaves near 25 percent. The dynamic pressure follows
 * the square of the velocity: 0.90^2 = 0.81 and 0.75^2 = 0.56. Source: Hoerner,
 * "Fluid Dynamic Drag", chapter 3, wake surveys. Confidence: estimate.
 */
const ETA_TAIL_WAKE_ATTACHED = 0.8;
const ETA_TAIL_WAKE_SEPARATED = 0.6;

/**
 * Half thickness of the wake at the tail, as a fraction of the wing mean chord.
 * The attached value is the viscous wake of an attached section two chords
 * downstream. The separated value is the dead air behind a fully stalled wing,
 * which grows to the order of the chord itself. Source: Hoerner, "Fluid Dynamic
 * Drag", chapter 3. Confidence: estimate.
 */
const WAKE_HALF_THICKNESS_ATTACHED = 0.06;
const WAKE_HALF_THICKNESS_SEPARATED = 0.75;

/**
 * d(sigma) / d(beta) at the fin. A NEGATIVE value means the fin meets MORE
 * sideslip than the free stream, which is what this configuration gives.
 *
 * The wing and the fuselage turn the flow sideways before it reaches the fin.
 * DATCOM fits the pair of that turn and the fin dynamic pressure ratio over many
 * aircraft:
 *
 *   eta_v (1 - d sigma / d beta)
 *     = 0.724 + 3.06 (S_v / S) / (1 + cos sweep) + 0.4 z_w / d + 0.009 AR
 *
 * with z_w the height of the wing root quarter chord above the body center line
 * and d the depth of the body. For the Me 262 the fin area is 3.09 m2 against a
 * wing of 21.7 m2 and 18.5 degrees of sweep, which gives 0.224. The aspect ratio
 * term gives 0.065. The wing sits ON the body center line, so the third term is
 * zero and its sign never enters. The sum is 1.01.
 *
 * The turn at this fin is therefore FAVORABLE, and it very nearly cancels the
 * dynamic pressure loss. With eta_v at 0.95 the slope is (1 - 1.012 / 0.95),
 * which is -0.065. Source: USAF DATCOM 5.2.1.1, and Roskam, "Airplane Design
 * Part VI", equation 10.32. Confidence: estimate.
 */
const SIDEWASH_SLOPE = -0.065;

/**
 * Dynamic pressure ratio at the fin. The fin stands above the wing wake in every
 * symmetric condition, so it keeps more of the free stream than the tailplane.
 * The loss is the fuselage boundary layer alone. Source: Perkins and Hage.
 * Confidence: estimate.
 */
const ETA_FIN = 0.95;

/** The largest angle the model adds to any strip, in radians. */
const MAX_ADDED_ANGLE = 0.35;

/** Below this dynamic pressure the lift coefficient has no meaning. */
const MIN_PRESSURE = 1e-9; // Pa

/** Below this speed the wake travel time has no meaning. */
const MIN_LAG_SPEED = 1.0; // m/s

/** The separation point of a fully separated section. See stall.ts. */
const SEPARATION_FLOOR = 0.04;

/** The largest flow angle the wake path uses. Beyond it the tangent runs away. */
const MAX_WAKE_ANGLE = 1.3; // rad

/**
 * Everything the model needs. downwashParams builds it from the assembly, and a
 * test can build one by hand or copy one and change a field.
 */
export interface DownwashParams {
  /** Strip indices of the wing. The downwash follows their lift. */
  readonly wingIndices: readonly number[];
  /** Strip indices of the horizontal tail. Empty means no tail. */
  readonly tailIndices: readonly number[];
  /** Strip indices of the fin. Empty means no fin. */
  readonly finIndices: readonly number[];
  /** Reference area of the wing, square meters. */
  wingArea: number;
  /** Aspect ratio of the wing. */
  wingAspectRatio: number;
  /** Mean chord of the wing, meters. It sets the thickness of the wake. */
  wingMeanChord: number;
  /** PI e AR of the wing. It turns the solved induced angle back into CL. */
  wingClPerInducedAngle: number;
  /** Meters from the wing quarter chord to the tail quarter chord. */
  tailArm: number;
  /** Meters the tail sits above the wing chord plane. Positive is up. */
  tailAboveWing: number;
  /** D / K of the tail group. It turns the solved induced angle into the section angle. */
  tailLoadFactor: number;
  /** D / K of the fin group. */
  finLoadFactor: number;
  /** Ratio of the downwash at the tail to the induced angle at the wing. */
  wakeFactor: number;
  /** Dynamic pressure ratio at the tail, clear of the wake core. */
  etaTailClean: number;
  /** Dynamic pressure ratio at the tail, inside the wake of an attached wing. */
  etaTailWakeAttached: number;
  /** Dynamic pressure ratio at the tail, inside the wake of a separated wing. */
  etaTailWakeSeparated: number;
  /** Half thickness of the wake at the tail with the wing attached, meters. */
  wakeHalfThicknessAttached: number;
  /** Half thickness of the wake at the tail with the wing separated, meters. */
  wakeHalfThicknessSeparated: number;
  /** d(sigma) / d(beta) at the fin. */
  sidewashSlope: number;
  /** Dynamic pressure ratio at the fin. */
  etaFin: number;
  /** True lags the downwash by the wake travel time. See section 5 above. */
  useLag: boolean;
}

/** What the model worked out this step. It allocates nothing. */
export interface DownwashState {
  /** rad, the downwash angle at the tail, positive when the flow moves down. */
  epsilon: number;
  /** rad, the sidewash angle at the fin. */
  sigma: number;
  /** Dynamic pressure ratio at the tail this step. */
  etaTail: number;
  /** Dynamic pressure ratio at the fin this step. */
  etaFin: number;
  /** How much of the wake covers the tail. 0 is clear and 1 is inside the core. */
  wakeCoverage: number;
  /** Meters from the tail to the center line of the wake. Never negative. */
  wakeOffset: number;
  /** rad, the angle the model added to every tail strip. */
  tailAngle: number;
  /** rad, the angle the model added to every fin strip. */
  finAngle: number;
  /** rad, the lagged downwash. It equals epsilon when the lag is off. */
  laggedEpsilon: number;
}

export interface Downwash {
  readonly params: DownwashParams;
  readonly state: DownwashState;
}

/** Builds the model. Every allocation happens here. */
export function createDownwash(params: DownwashParams): Downwash {
  if (params.wingIndices.length > 0 && !(params.wingArea > 0 && params.wingAspectRatio > 0)) {
    throw new Error(
      `The downwash needs a positive wing area and aspect ratio. It got ` +
        `${params.wingArea} and ${params.wingAspectRatio}.`,
    );
  }
  return {
    params,
    state: {
      epsilon: 0,
      sigma: 0,
      etaTail: params.etaTailClean,
      etaFin: params.etaFin,
      wakeCoverage: 0,
      wakeOffset: params.tailAboveWing,
      tailAngle: 0,
      finAngle: 0,
      laggedEpsilon: 0,
    },
  };
}

/**
 * Works out the downwash angle, the sidewash angle and both dynamic pressure
 * ratios, and writes them into the state.
 *
 * wingLift is the lift of the WHOLE wing this step, in newtons.
 * wingSeparation is the area weighted separation point of the wing strips, 1
 * attached and 0.04 fully separated. alpha and beta are the free stream angles.
 */
export function updateDownwash(
  d: Downwash,
  wingLift: number,
  wingSeparation: number,
  alpha: number,
  beta: number,
  speed: number,
  dynamicPressure: number,
  dt: number,
): void {
  const p = d.params;
  const s = d.state;

  // The downwash follows the lift the wing really makes. At the stall that lift
  // collapses and the downwash collapses with it.
  const cl =
    dynamicPressure > MIN_PRESSURE && p.wingArea > 0
      ? wingLift / (dynamicPressure * p.wingArea)
      : 0;
  const steady = (p.wakeFactor * cl) / (Math.PI * p.wingAspectRatio);

  if (p.useLag && speed > MIN_LAG_SPEED && dt > 0) {
    const travel = p.tailArm / speed;
    s.laggedEpsilon += ((steady - s.laggedEpsilon) * dt) / (travel + dt);
  } else {
    s.laggedEpsilon = steady;
  }
  s.epsilon = s.laggedEpsilon;

  // The wake rises above the wing chord plane at the angle alpha - epsilon,
  // because the air moves aft and up over an aircraft at a positive angle of
  // attack and the downwash turns that path back down.
  const wakeAngle = clamp(alpha - s.epsilon, -MAX_WAKE_ANGLE, MAX_WAKE_ANGLE);
  const rise = p.tailArm * Math.tan(wakeAngle);
  s.wakeOffset = Math.abs(p.tailAboveWing - rise);

  // A separated wing sheds a far thicker and far slower wake than an attached
  // one, so the wing separation state sets the thickness.
  const separated = clamp(
    (1 - wingSeparation) / (1 - SEPARATION_FLOOR),
    0,
    1,
  );
  const halfThickness = lerp(
    p.wakeHalfThicknessAttached,
    p.wakeHalfThicknessSeparated,
    separated,
  );
  s.wakeCoverage =
    halfThickness > 0 ? 1 - smoothstep(halfThickness, 2 * halfThickness, s.wakeOffset) : 0;
  // The wake of an attached wing is thin AND mild. The wake of a separated wing
  // is thick AND slow. The separation state therefore sets both.
  const core = lerp(p.etaTailWakeAttached, p.etaTailWakeSeparated, separated);
  s.etaTail = lerp(p.etaTailClean, core, s.wakeCoverage);

  s.sigma = p.sidewashSlope * beta;
  s.etaFin = p.etaFin;
}

/**
 * Returns the angle to add to the induced angle of one group, so that the group
 * makes the lift it would make at the free stream angle A - flowAngle, with its
 * dynamic pressure scaled by eta.
 *
 * THE POINT OF THIS FUNCTION. The assembly solved the induced angle of the group
 * BEFORE the downwash arrived, from the lift at the full angle A. A smaller
 * angle makes less lift, and less lift makes a smaller induced angle. The model
 * cannot run the solve again, so it must add the angle that leaves the same
 * answer. Adding the flow angle raw would take the lift down by K epsilon, and
 * the true loss is only K epsilon D / (D + K). For the fin of this aircraft that
 * error is a factor of 2.4, because a fin of aspect ratio 1.5 loses more than
 * half of its angle to its own induced angle.
 *
 * The algebra. Write r = D / K, the load factor of the group. The solve gives
 * alpha_i = A K / (D + K), so the section angle is A - alpha_i = r alpha_i and
 * the full angle is A = (1 + r) alpha_i. The lift with the loss and with the
 * induced angle solved again is eta K (A - flow) / (1 + eta K / D). Setting that
 * equal to K (A - alpha_i - X) gives the line below. At eta = 1 it reduces to
 * X = flow r / (1 + r), and the section angle then comes out exactly right as
 * well, not only the lift.
 */
function groupAngle(
  loadFactor: number,
  inducedAngle: number,
  flowAngle: number,
  eta: number,
): number {
  const section = loadFactor * inducedAngle;
  const total = section + inducedAngle;
  return section - (eta * loadFactor * (total - flowAngle)) / (loadFactor + eta);
}

/**
 * Adds the downwash of the tail and the sidewash of the fin into the induced
 * angle array of the assembly, and fills the state.
 *
 * The two angles add, because surface.ts takes one angle off the local flow of
 * the strip. The function reads the wing lift and the wing separation out of the
 * strips the assembly already solved, so the caller passes no lift of its own.
 * It allocates nothing.
 */
export function applyDownwash(
  d: Downwash,
  surfaces: readonly Surface[],
  inducedAngles: Float64Array,
  alpha: number,
  beta: number,
  speed: number,
  dynamicPressure: number,
  dt: number,
): void {
  const p = d.params;
  const s = d.state;
  if (p.wingIndices.length === 0) {
    return;
  }

  // The solve of the assembly closes alpha_i = CL / (PI e AR), so the lift it
  // settled on this step comes straight back out of the angle it wrote.
  const wingCl = inducedAngles[p.wingIndices[0]] * p.wingClPerInducedAngle;
  const wingLift = wingCl * dynamicPressure * p.wingArea;

  let area = 0;
  let separation = 0;
  for (const index of p.wingIndices) {
    const stripArea = surfaces[index].def.area;
    area += stripArea;
    separation += stripArea * surfaces[index].state.stall.f;
  }

  updateDownwash(
    d,
    wingLift,
    area > 0 ? separation / area : 1,
    alpha,
    beta,
    speed,
    dynamicPressure,
    dt,
  );

  s.tailAngle = 0;
  if (p.tailIndices.length > 0) {
    s.tailAngle = clamp(
      groupAngle(p.tailLoadFactor, inducedAngles[p.tailIndices[0]], s.epsilon, s.etaTail),
      -MAX_ADDED_ANGLE,
      MAX_ADDED_ANGLE,
    );
    for (const index of p.tailIndices) {
      inducedAngles[index] += s.tailAngle;
    }
  }

  s.finAngle = 0;
  if (p.finIndices.length > 0) {
    s.finAngle = clamp(
      groupAngle(p.finLoadFactor, inducedAngles[p.finIndices[0]], s.sigma, s.etaFin),
      -MAX_ADDED_ANGLE,
      MAX_ADDED_ANGLE,
    );
    for (const index of p.finIndices) {
      inducedAngles[index] += s.finAngle;
    }
  }
}

/** The area weighted mean of one number over a set of strips. */
function meanOverGroup(
  surfaces: readonly Surface[],
  indices: readonly number[],
  read: (s: Surface) => number,
): number {
  let area = 0;
  let sum = 0;
  for (const index of indices) {
    const stripArea = surfaces[index].def.area;
    area += stripArea;
    sum += stripArea * read(surfaces[index]);
  }
  return area > 0 ? sum / area : 0;
}

/**
 * The lift a group makes per radian of angle, divided by the free stream
 * dynamic pressure.
 *
 * Simple sweep theory gives a strip the normal dynamic pressure q cos^2 and the
 * normal angle alpha / cos, so the two cosines leave one factor of cos on the
 * lift. That is the same K that estimateSurfaceLoad reports, written in free
 * stream terms.
 */
function groupSlope(surfaces: readonly Surface[], indices: readonly number[]): number {
  let sum = 0;
  for (const index of indices) {
    const def = surfaces[index].def;
    sum += def.area * def.airfoil.clAlpha * Math.abs(Math.cos(def.sweep));
  }
  return sum;
}

/** True when every strip of the group stands up, which makes the group a fin. */
function isVertical(surfaces: readonly Surface[], indices: readonly number[]): boolean {
  if (indices.length === 0) {
    return false;
  }
  for (const index of indices) {
    if (Math.abs(Math.cos(surfaces[index].def.dihedral)) > 0.5) {
      return false;
    }
  }
  return true;
}

/**
 * Reads the roles of the groups out of the geometry and returns the parameters.
 *
 * The roles come from the shape and the place of each group, not from its name.
 * The wing is the largest group that lies flat. The tail is the flat group
 * furthest aft of it. The fin is the group whose strips stand up. A model with
 * no wing, or with no group behind the wing, gets an empty parameter set and the
 * model then does nothing at all.
 */
export function downwashParams(
  surfaces: readonly Surface[],
  groups: readonly GroupDef[],
): DownwashParams {
  const empty: DownwashParams = {
    wingIndices: [],
    tailIndices: [],
    finIndices: [],
    wingArea: 0,
    wingAspectRatio: 0,
    wingMeanChord: 0,
    wingClPerInducedAngle: 0,
    tailArm: 0,
    tailAboveWing: 0,
    tailLoadFactor: 0,
    finLoadFactor: 0,
    wakeFactor: WAKE_FACTOR,
    etaTailClean: ETA_TAIL_CLEAN,
    etaTailWakeAttached: ETA_TAIL_WAKE_ATTACHED,
    etaTailWakeSeparated: ETA_TAIL_WAKE_SEPARATED,
    wakeHalfThicknessAttached: 0,
    wakeHalfThicknessSeparated: 0,
    sidewashSlope: SIDEWASH_SLOPE,
    etaFin: ETA_FIN,
    useLag: false,
  };

  let wing: GroupDef | undefined;
  let tail: GroupDef | undefined;
  let fin: GroupDef | undefined;
  for (const group of groups) {
    if (isVertical(surfaces, group.surfaceIndices)) {
      if (fin === undefined || group.area > fin.area) {
        fin = group;
      }
    } else if (wing === undefined || group.area > wing.area) {
      wing = group;
    }
  }
  if (wing === undefined) {
    return empty;
  }

  const wingX = meanOverGroup(surfaces, wing.surfaceIndices, (s) => s.def.position.x);
  const wingZ = meanOverGroup(surfaces, wing.surfaceIndices, (s) => s.def.position.z);
  for (const group of groups) {
    if (group === wing || isVertical(surfaces, group.surfaceIndices)) {
      continue;
    }
    const x = meanOverGroup(surfaces, group.surfaceIndices, (s) => s.def.position.x);
    if (x >= wingX) {
      continue;
    }
    if (tail === undefined) {
      tail = group;
      continue;
    }
    const best = meanOverGroup(surfaces, tail.surfaceIndices, (s) => s.def.position.x);
    if (x < best) {
      tail = group;
    }
  }

  const tailX =
    tail === undefined ? wingX : meanOverGroup(surfaces, tail.surfaceIndices, (s) => s.def.position.x);
  const tailZ =
    tail === undefined ? wingZ : meanOverGroup(surfaces, tail.surfaceIndices, (s) => s.def.position.z);
  const chord = meanOverGroup(surfaces, wing.surfaceIndices, (s) => s.def.chord);
  const tailSlope = tail === undefined ? 0 : groupSlope(surfaces, tail.surfaceIndices);
  const finSlope = fin === undefined ? 0 : groupSlope(surfaces, fin.surfaceIndices);

  return {
    wingIndices: wing.surfaceIndices,
    tailIndices: tail === undefined ? [] : tail.surfaceIndices,
    finIndices: fin === undefined ? [] : fin.surfaceIndices,
    wingArea: wing.area,
    wingAspectRatio: wing.aspectRatio,
    wingMeanChord: chord,
    wingClPerInducedAngle: Math.PI * wing.oswaldEfficiency * wing.aspectRatio,
    tailArm: wingX - tailX,
    // Body z points down, so a tail above the wing has the smaller z.
    tailAboveWing: wingZ - tailZ,
    tailLoadFactor:
      tail === undefined || tailSlope <= 0
        ? 0
        : (tail.area * Math.PI * tail.oswaldEfficiency * tail.aspectRatio) / tailSlope,
    finLoadFactor:
      fin === undefined || finSlope <= 0
        ? 0
        : (fin.area * Math.PI * fin.oswaldEfficiency * fin.aspectRatio) / finSlope,
    wakeFactor: WAKE_FACTOR,
    etaTailClean: ETA_TAIL_CLEAN,
    etaTailWakeAttached: ETA_TAIL_WAKE_ATTACHED,
    etaTailWakeSeparated: ETA_TAIL_WAKE_SEPARATED,
    wakeHalfThicknessAttached: WAKE_HALF_THICKNESS_ATTACHED * chord,
    wakeHalfThicknessSeparated: WAKE_HALF_THICKNESS_SEPARATED * chord,
    sidewashSlope: SIDEWASH_SLOPE,
    etaFin: ETA_FIN,
    useLag: false,
  };
}
