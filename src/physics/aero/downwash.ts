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
 * 5. THE TRANSONIC RANGE. BEAD b73
 *
 * Two things happen to this model above the critical Mach number, and the model
 * had neither of them until bead b73.
 *
 * THE WAKE STOPS TURNING THE FLOW DOWN. A shock stands on the upper surface of
 * the wing, the flow behind it starts to leave the surface, and the trailing
 * vortex sheet weakens. On a SWEPT wing the shock arrives at the ROOT first, so
 * the load leaves the root and moves outboard. The tail sits on the plane of
 * symmetry, where the downwash answers the INBOARD load far more than the
 * outboard load, so the downwash at the tail falls faster than the lift of the
 * whole wing falls. The tail then meets a larger angle of attack, makes more up
 * load, and a larger up load behind the center of gravity is a NOSE DOWN moment.
 * This is one of the two mechanisms every text gives for the Mach tuck, and it
 * is the one that moves the neutral point AFT through the (1 - d epsilon /
 * d alpha) term. WAKE_SCALE below holds it.
 *
 * THE TAIL LOSES DYNAMIC PRESSURE. The wake is a momentum deficit, so it deepens
 * with the DRAG of the wing. The wave drag of the drag rise is many times the
 * profile drag of the same wing, so the tail loses several times more dynamic
 * pressure at the Mach limit than it loses in cruise. This one works AGAINST the
 * tuck, because a tail with less dynamic pressure is a smaller tail, and it is
 * the reason the elevator goes light at the same time as the nose goes down.
 * WAKE_REFERENCE_DRAG below holds it.
 *
 * Both laws read the same shock Mach number that every table of
 * src/physics/aero/compressibility.ts reads, through shockMachNumber of that
 * module, so the wake meets the shock at the Mach number the wing meets it at.
 * Both are exactly neutral below the critical Mach number, so nothing below the
 * drag rise moves by one count.
 *
 *
 * 6. THE LAG
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
 * THE LAG USES THE EXACT SOLUTION OF THE FIRST ORDER SYSTEM, 1 - exp(-dt / T).
 * The earlier form dt / (T + dt) is the bilinear approximation. It NEVER reaches
 * one at any finite dt, so a caller that asked for a steady answer with a large
 * dt still carried a part of the call before it: at dt = 5 s and a wake travel of
 * 0.07 s the old form kept 1.4 percent of the last value. The exact form reaches
 * one to machine precision above about 40 travel times, and it reaches it exactly
 * at dt = Infinity, which is how assembly.evaluateSteady asks for the steady
 * answer. src/physics/aero/stall.ts already used the exact form. The two lags of
 * the model now agree.
 *
 * This module is pure physics. It imports no Three.js class at all.
 */

import { clamp, lerp, lookup1d, smoothstep, table1d } from '@/math/tables';
import type { Table1D } from '@/math/tables';
import type { GroupDef } from '@/physics/aero/assembly';
import type { MachCorrection } from '@/physics/aero/compressibility';
import {
  createMachCorrection,
  machCorrection,
  shockMachAnchors,
  shockMachNumber,
} from '@/physics/aero/compressibility';
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
 * What the wake still turns down, as a fraction of its low speed value, against
 * the free stream Mach number AT THE REFERENCE SWEEP of
 * src/physics/aero/compressibility.ts. See section 5 of the module comment.
 *
 * THE MECHANISM, AND WHY IT IS NOT ALREADY IN THE MODEL. This module drives the
 * downwash from the lift the wing really makes, so it already carries the
 * subsonic compressibility rule that DATCOM 4.4.1 gives, which is that
 * d epsilon / d alpha follows the lift curve slope of the wing. Measured, that
 * rule alone moves the slope of this wing from 0.550 to 0.555 between Mach 0.78
 * and Mach 0.86, because the Prandtl-Glauert growth and the shock loss of
 * SLOPE_LOSS_SCALE very nearly cancel. The real transonic fall is not a lift
 * curve slope effect at all. It is the shock separating the flow at the ROOT of
 * a swept wing, which takes the load away from exactly the span stations the
 * tail answers to and carries it outboard.
 *
 * Anchor by anchor. Every anchor is a free stream value at the REFERENCE SWEEP
 * and at the REFERENCE THICKNESS of compressibility.ts, exactly as every anchor
 * of that module is, so a surface with its own sweep and its own section reads
 * the table at its own place. The value is 1 to the critical Mach number,
 * because there is no shock below it. It falls slowly to Mach 0.82, where the
 * shock is still weak and stands well forward, then fast through 0.84 and 0.86,
 * where the shock is strong and the root sheds its load. It holds near 0.3 above
 * Mach 0.9, because a wake that turns nothing at all is not what any measurement
 * shows.
 *
 * WHAT THE MODEL GETS FROM IT. The innermost strip of the Me 262 wing is 10.8
 * percent thick against the 11 percent reference, so it meets its shock a little
 * later and reads 0.435 at Mach 0.86 where the anchor says 0.40. The slope
 * d epsilon / d alpha of the whole aircraft then falls from 0.562 to 0.249
 * between Mach 0.78 and Mach 0.86, at 8000 m, so (1 - d epsilon / d alpha) grows
 * from 0.44 to 0.75 and the tail term of the neutral point grows with it. That
 * is 0.161 m of the 0.426 m the neutral point travels. See the comment on
 * AC_SHIFT_X of compressibility.ts for the whole split.
 *
 * Source: the standard two term explanation of the Mach tuck, which is the aft
 * shift of the wing aerodynamic center AND the fall of the downwash at the tail.
 * Hurt, "Aerodynamics for Naval Aviators", NAVWEPS 00-80T-80, chapter 3, gives
 * both. The transonic measurement behind a swept wing is NACA RM L52J15,
 * Coppolino, 1952, "Effective downwash characteristics at transonic speeds of a
 * 6-percent-thick wing with 47 degrees of sweepback". Confidence: firm for the
 * direction and the mechanism, ESTIMATE for the size, anchored on the firm tuck
 * onset Mach number of 0.83.
 */
const WAKE_MACH: readonly number[] = [0.78, 0.8, 0.82, 0.84, 0.86, 0.88, 0.9, 1.0];
const WAKE_SCALE: readonly number[] = [1.0, 0.97, 0.88, 0.66, 0.4, 0.33, 0.3, 0.3];

/**
 * Wing profile drag coefficient that ETA_TAIL_CLEAN belongs to.
 *
 * A wake is a momentum deficit, and the deficit is the DRAG. Hoerner fits the
 * velocity deficit at the center of a plane wake as proportional to
 * sqrt(cd c / x), so the dynamic pressure loss at a fixed station behind the
 * wing grows with the SQUARE ROOT of the drag coefficient of the wing. Silverstein
 * and Katzoff give the same result in the form the tail needs: they chart the
 * dynamic pressure across the wake against the profile drag coefficient and the
 * distance behind the wing.
 *
 * The model therefore holds the loss at
 *
 *   1 - eta = (1 - ETA_TAIL_CLEAN) * sqrt(1 + cd_wave / WAKE_REFERENCE_DRAG)
 *
 * with cd_wave the wave drag the wing pays at this Mach number. The value below
 * is the profile drag coefficient of the Me 262 wing in cruise, from the section
 * tables, so the law returns ETA_TAIL_CLEAN exactly below the drag rise and the
 * calibration of that constant is untouched.
 *
 * Measured on this aircraft: the tail keeps 0.920 of the free stream to Mach
 * 0.78 and 0.817 of it at Mach 0.86. That is the other half of what a pilot met
 * at the limit, because a tail with less dynamic pressure has less elevator. It
 * takes 0.041 m of neutral point travel BACK, which is the honest sign: a
 * smaller tail is a less stable aircraft, and the aircraft turns nose down and
 * loses its elevator at the same time.
 *
 * Source: Silverstein and Katzoff, NACA Report 648, 1939, "Design charts for
 * predicting downwash angles and wake characteristics behind plain and flapped
 * wings", and Hoerner, "Fluid Dynamic Drag", chapter 3, wake surveys.
 * Confidence: firm for the square root law, estimate for the reference drag.
 */
const WAKE_REFERENCE_DRAG = 0.009;

/**
 * The lowest dynamic pressure ratio the wave drag law above may reach.
 *
 * The square root law has no upper bound, and the wave drag of the reference
 * section reaches 0.384 at Mach 1, which would take the ratio to 0.47. That is
 * below the value this module gives a tail sitting INSIDE the wake of a fully
 * separated wing, which cannot be right for a tail that is still in clear air.
 * The floor holds the law at the separated wake value. This aircraft cannot
 * reach the Mach number where the floor acts. Confidence: estimate.
 */
const ETA_TAIL_SHOCK_FLOOR = 0.6;

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
 * wing of 21.7 m2 and 15.72 degrees of QUARTER CHORD sweep, which gives 0.222.
 * The aspect ratio term gives 0.065. The wing sits ON the body center line, so
 * the third term is zero and its sign never enters. The sum is 1.01.
 *
 * BEAD b75 REDID THIS SUM AT THE CORRECTED SWEEP AND THE ANSWER DID NOT MOVE.
 * The line above read 18.5 degrees, which is the LEADING EDGE angle, and the
 * sweep term then gave 0.224 against the 0.222 it gives at the quarter chord
 * angle. The sum rounds to 1.01 either way, so SIDEWASH_SLOPE keeps its value.
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
 * The wake table of section 5, on the shock Mach scale of
 * src/physics/aero/compressibility.ts, and the scratch that reads the wave drag
 * of the wing. Both live in module scope, because the step allocates nothing.
 */
const WAKE_SCALE_TABLE: Table1D = table1d(shockMachAnchors(WAKE_MACH), WAKE_SCALE.slice());
const wakeCorrection: MachCorrection = createMachCorrection();

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
  /**
   * rad, the area weighted quarter chord sweep of the wing. Both transonic laws
   * of section 5 read it, because the sweep is what sets the shock Mach number.
   */
  wingSweep: number;
  /**
   * t/c of the wing section AT THE ROOT, which is the thickest section and the
   * one that meets its shock first. WAKE_SCALE reads it, because the load the
   * tail answers to is the load near the plane of symmetry.
   */
  wingRootThickness: number;
  /**
   * Area weighted t/c of the whole wing. The wave drag law of WAKE_REFERENCE_DRAG
   * reads it, because the wake carries the drag of the WHOLE wing and not the
   * drag of its thickest section.
   */
  wingMeanThickness: number;
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
  /** True lags the downwash by the wake travel time. See section 6 above. */
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
  /**
   * What the wake still turns down this step, as a fraction of its low speed
   * value. It is 1 below the drag rise. See section 5 of the module comment.
   */
  wakeScale: number;
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
      wakeScale: 1,
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
 * mach is the free stream Mach number, which both transonic laws of section 5
 * read. A Mach number below the drag rise leaves both laws neutral.
 */
export function updateDownwash(
  d: Downwash,
  wingLift: number,
  wingSeparation: number,
  alpha: number,
  beta: number,
  speed: number,
  mach: number,
  dynamicPressure: number,
  dt: number,
): void {
  const p = d.params;
  const s = d.state;

  // SECTION 5. The shock the WING meets. The root section is the thickest and
  // the least relieved, so it is the one that sheds its load first, and the tail
  // sits behind the root.
  s.wakeScale = lookup1d(
    WAKE_SCALE_TABLE,
    shockMachNumber(mach, p.wingSweep, p.wingRootThickness),
  );

  // The downwash follows the lift the wing really makes. At the stall that lift
  // collapses and the downwash collapses with it. Past the drag rise the shock
  // takes the turning power of the wake away as well.
  const cl =
    dynamicPressure > MIN_PRESSURE && p.wingArea > 0
      ? wingLift / (dynamicPressure * p.wingArea)
      : 0;
  const steady = (p.wakeFactor * s.wakeScale * cl) / (Math.PI * p.wingAspectRatio);

  if (p.useLag && speed > MIN_LAG_SPEED && dt > 0) {
    const travel = p.tailArm / speed;
    // The exact solution of the first order lag over dt. See section 6 above.
    s.laggedEpsilon += (steady - s.laggedEpsilon) * (travel > 0 ? 1 - Math.exp(-dt / travel) : 1);
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
  // SECTION 5. A wake is a momentum deficit and the deficit is the drag, so the
  // wave drag of the drag rise deepens the wake and the tail loses pressure with
  // it. The loss follows the square root of the drag coefficient. Below the drag
  // rise the wave drag is zero and this returns etaTailClean exactly.
  machCorrection(mach, p.wingSweep, wakeCorrection, p.wingMeanThickness);
  const clean = Math.max(
    1 - (1 - p.etaTailClean) * Math.sqrt(1 + wakeCorrection.cdAdd / WAKE_REFERENCE_DRAG),
    Math.min(p.etaTailClean, ETA_TAIL_SHOCK_FLOOR),
  );

  // The wake of an attached wing is thin AND mild. The wake of a separated wing
  // is thick AND slow. The separation state therefore sets both.
  const core = lerp(p.etaTailWakeAttached, p.etaTailWakeSeparated, separated);
  s.etaTail = lerp(clean, core, s.wakeCoverage);

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

/** Puts the model back to the state createDownwash left it in. */
export function resetDownwash(d: Downwash): void {
  const p = d.params;
  const s = d.state;
  s.epsilon = 0;
  s.sigma = 0;
  s.etaTail = p.etaTailClean;
  s.wakeScale = 1;
  s.etaFin = p.etaFin;
  s.wakeCoverage = 0;
  s.wakeOffset = p.tailAboveWing;
  s.tailAngle = 0;
  s.finAngle = 0;
  s.laggedEpsilon = 0;
}

/**
 * Adds the downwash of the tail and the sidewash of the fin into the induced
 * angle array of the assembly, and fills the state.
 *
 * The two angles add, because surface.ts takes one angle off the local flow of
 * the strip. The function reads the wing lift out of the induced angle the
 * assembly already solved, so the caller passes no lift of its own. It allocates
 * nothing.
 *
 * `separation` holds the separation point of every strip, in the order of
 * `surfaces`. IT IS AN ARGUMENT AND NOT A READ OF Surface.state. The thickness of
 * the wake follows the separation state of the wing, and the assembly must be
 * able to hand over the value that the SAME evaluation will use. A read of
 * Surface.state here would take the value that the evaluation BEFORE this one
 * left behind, which is the b61 defect. See the module comment of assembly.ts.
 */
export function applyDownwash(
  d: Downwash,
  surfaces: readonly Surface[],
  separation: Float64Array,
  inducedAngles: Float64Array,
  alpha: number,
  beta: number,
  speed: number,
  mach: number,
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
  let wingSeparation = 0;
  for (const index of p.wingIndices) {
    const stripArea = surfaces[index].def.area;
    area += stripArea;
    wingSeparation += stripArea * separation[index];
  }

  updateDownwash(
    d,
    wingLift,
    area > 0 ? wingSeparation / area : 1,
    alpha,
    beta,
    speed,
    mach,
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
    wingSweep: 0,
    wingRootThickness: 0.11,
    wingMeanThickness: 0.11,
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
  const sweep = meanOverGroup(surfaces, wing.surfaceIndices, (s) => Math.abs(s.def.sweep));
  const meanThickness = meanOverGroup(
    surfaces,
    wing.surfaceIndices,
    (s) => s.def.airfoil.thickness,
  );
  // The section nearest the plane of symmetry. It is the thickest, it meets its
  // shock first, and it is the one whose load the tail answers to.
  let rootIndex = wing.surfaceIndices[0];
  for (const index of wing.surfaceIndices) {
    if (Math.abs(surfaces[index].def.position.y) < Math.abs(surfaces[rootIndex].def.position.y)) {
      rootIndex = index;
    }
  }
  const rootThickness = surfaces[rootIndex].def.airfoil.thickness;
  const tailSlope = tail === undefined ? 0 : groupSlope(surfaces, tail.surfaceIndices);
  const finSlope = fin === undefined ? 0 : groupSlope(surfaces, fin.surfaceIndices);

  return {
    wingIndices: wing.surfaceIndices,
    tailIndices: tail === undefined ? [] : tail.surfaceIndices,
    finIndices: fin === undefined ? [] : fin.surfaceIndices,
    wingArea: wing.area,
    wingAspectRatio: wing.aspectRatio,
    wingMeanChord: chord,
    wingSweep: sweep,
    wingRootThickness: rootThickness,
    wingMeanThickness: meanThickness,
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
