/**
 * The aerodynamic elements of the Messerschmitt Me 262 A-1a.
 *
 * The file turns the plan form of the aircraft into the element list that
 * src/physics/aero/assembly.ts flies. Nothing here computes a force. Every
 * effect that a flight model needs comes out of the geometry and the section
 * data, and the aero modules already hold the physics.
 *
 *   left wing         8 strips, cosine spaced
 *   right wing        8 strips, the mirror of the left wing
 *   horizontal tail   4 strips, two on each side
 *   vertical fin      2 strips
 *   fuselage          1 body
 *   nacelles          2 bodies
 *                    --
 *                    25 elements
 *
 *
 * WHY COSINE SPACING
 *
 * The lift of a wing falls to zero at the tip over a short span, so the load
 * gradient is steepest there. A strip model with equal strips puts its last
 * station in the middle of that fall and reads the load too high. Cosine spacing
 * places the station boundaries at (b/2) sin(k pi / 16), which packs four of the
 * eight strips into the outer third of the semi span. The same spacing is the
 * one Glauert used for the lifting line series, and for the same reason.
 *
 * The outermost strip is only 0.12 m wide. That is not a mistake. It is the tip
 * itself, and it holds the part of the wing whose local angle of attack matters
 * most to the roll damping and to the stall.
 *
 *
 * WHERE THE ORIGIN SITS
 *
 * Every position is in body axes of CONVENTIONS section 3.1, measured from the
 * center of gravity, with x forward, y right and z down. The map from the
 * drawing frame is
 *
 *   x = CG_OFFSET_FROM_NOSE - station
 *   y = y
 *   z = CG_HEIGHT_FROM_DATUM - height
 *
 * src/aircraft/me262/mass.ts owns both constants and derives the second one from
 * the mass distribution. The engines hang below the reference plane, so the
 * center of gravity sits 0.133 m below it and the wing chord plane sits that far
 * above the center of gravity.
 *
 *
 * THE SIGN OF EVERY CONTROL
 *
 * SurfaceDef.controlEffectiveness is d(alphaZeroLift) / d(deflection). A
 * POSITIVE value raises the zero lift angle, which REMOVES lift. A surface that
 * must gain lift on a positive deflection therefore takes a NEGATIVE value. The
 * four signs below follow from the moment each control must make.
 *
 *   elevator  A positive command pitches the nose UP. The nose goes up when the
 *             tail load goes DOWN, so the tail must lose lift, so the value is
 *             POSITIVE on every tail strip.
 *   aileron   A positive command rolls RIGHT. The right wing must lose lift and
 *             the left wing must gain it, so the value is POSITIVE on the right
 *             strips and NEGATIVE on the left strips.
 *   rudder    A positive command yaws RIGHT. The fin normal points right, so its
 *             lift acts to the LEFT at a positive local angle, and a left force
 *             behind the center of gravity swings the nose right. The fin must
 *             therefore GAIN lift, so the value is NEGATIVE.
 *   flap      A positive command lowers the flap and ADDS lift, so the value is
 *             NEGATIVE.
 *
 * test/unit/me262-geometry.test.ts flies the assembled aircraft and checks all
 * four against the moment they make. Those four lines are the cheapest insurance
 * in the project.
 *
 *
 * PARTIAL CONTROL SPANS
 *
 * A control rarely fills a whole strip. The aileron runs from 4.00 m to 5.98 m
 * and strip 4 runs from 3.48 m to 4.42 m, so the aileron covers 45 percent of
 * that strip. The model scales the effectiveness of the strip by the covered
 * fraction. That is exact for the lift increment, because only the covered part
 * of the strip changes its camber, and it keeps the control power tied to the
 * real span of the surface instead of to the strip boundaries.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Vector3 } from 'three';

import { lerp } from '@/math/tables';
import { DEG } from '@/math/units';
import type { AeroAssembly, GroupDef } from '@/physics/aero/assembly';
import { createAssembly } from '@/physics/aero/assembly';
import type { Airfoil } from '@/physics/aero/airfoil';
import { NACA_0009, NACA_0011, blendAirfoils } from '@/physics/aero/airfoil';
import type { BodyDef } from '@/physics/aero/body';
import type { StallParams } from '@/physics/aero/stall';
import { STALL_NACA_0009, STALL_NACA_0011 } from '@/physics/aero/stall';
import type { SurfaceDef } from '@/physics/aero/surface';
import { CG_HEIGHT_FROM_DATUM, CG_OFFSET_FROM_NOSE, fuselageShape } from '@/aircraft/me262/mass';

// ---------------------------------------------------------------------------
// Plan form. CONVENTIONS section 8, and the three view that
// src/render/models/me262.ts works from.
// ---------------------------------------------------------------------------

/** Wing span. Source: CONVENTIONS section 8, confidence firm. */
export const WING_SPAN = 12.51; // m

/** Wing reference area. Source: CONVENTIONS section 8, confidence firm. */
export const WING_AREA = 21.7; // m2

/** Aspect ratio, b^2 / S. Source: CONVENTIONS section 8, derived. */
export const WING_ASPECT_RATIO = 7.21;

/** Sweep of the quarter chord line. Source: CONVENTIONS section 8, firm. */
export const WING_SWEEP = 18.5 * DEG; // rad

const HALF_SPAN = WING_SPAN / 2;
const TAN_SWEEP = Math.tan(WING_SWEEP);

/**
 * Root chord and tip chord. A straight taper wing holds S = (b / 2)(cr + ct),
 * so the firm span and the firm area fix the sum at 3.469 m. The taper ratio of
 * 0.446 comes from a three view. Confidence: derived from firm data.
 */
const ROOT_CHORD = 2.4; // m
const TIP_CHORD = 1.07; // m

/**
 * Mean aerodynamic chord of the straight taper wing.
 *   MAC = (2/3) cr (1 + L + L^2) / (1 + L), with L = ct / cr.
 */
export const MAC =
  ((2 / 3) *
    ROOT_CHORD *
    (1 + TIP_CHORD / ROOT_CHORD + (TIP_CHORD / ROOT_CHORD) ** 2)) /
  (1 + TIP_CHORD / ROOT_CHORD); // 1.820 m

/** Span station of the mean aerodynamic chord, (b/6)(1 + 2L)/(1 + L). */
export const MAC_SPAN_STATION =
  ((WING_SPAN / 6) * (1 + (2 * TIP_CHORD) / ROOT_CHORD)) / (1 + TIP_CHORD / ROOT_CHORD); // 2.728 m

/** Station of the wing root quarter chord, meters aft of the nose tip. */
const WING_ROOT_QUARTER_STATION = 4.85; // m

/** Dihedral of the outer panel, and the span station where it starts. Estimate. */
const WING_DIHEDRAL = 3.5 * DEG;
const DIHEDRAL_START = 2.2; // m
const TAN_DIHEDRAL = Math.tan(WING_DIHEDRAL);

/**
 * Rigging incidence at the root and at the tip.
 *
 * The 1.5 degree difference is WASHOUT. It makes the root work at a higher local
 * angle than the tip, so the root separates first, the nose drops before the
 * wing drops, and the aileron keeps its bite through the break. The automatic
 * slat on the outer wing does the same job from the other side. Estimate from a
 * three view, confidence: medium.
 */
const ROOT_INCIDENCE = 1.5 * DEG;
const TIP_INCIDENCE = 0;

/**
 * Oswald efficiency of the wing.
 *
 * The value is not a drag bookkeeping factor here. assembly.ts uses it to close
 * the induced angle, so it must be the value that reproduces the finite span
 * lift curve slope. The Helmbold formula gives a slope of 4.78 per radian for an
 * aspect ratio of 7.21. Matching that against the 6.5 per radian of the section
 * needs e = 0.80. Confidence: derived.
 */
const WING_OSWALD = 0.8;

// --- Movable surfaces, span limits in meters from the plane of symmetry -----
// The values repeat FLAP_INNER, FLAP_OUTER, AILERON_SPAN and SLAT_SPAN of
// src/render/models/me262.ts, so the surface that moves on the screen is the
// surface that makes the moment.

/** Inner flap panel. The nacelle splits the flap into two panels. */
const FLAP_INNER: readonly [number, number] = [0.62, 1.56];
/** Outer flap panel, outboard of the nacelle. */
const FLAP_OUTER: readonly [number, number] = [2.5, 3.38];
const AILERON_SPAN: readonly [number, number] = [4.0, 5.98];
const SLAT_SPAN: readonly [number, number] = [3.0, 6.02];

/**
 * Flap travel. The flap of the A-1a carries graduations at 0, 10, 20, 30, 40 and
 * 50 degrees on its upper surface, and the 20 degree take off setting is marked
 * in red. Source: "Pilot's Handbook for Me-262 A-1", section 2, wing flaps.
 * Confidence: firm.
 */
export const FLAP_TAKEOFF_ANGLE = 20 * DEG; // rad
export const FLAP_LANDING_ANGLE = 50 * DEG; // rad

/**
 * Flap effectiveness over the full travel.
 *
 * The same handbook calls the flap a HANDLEY-PAGE type, which is a slotted flap.
 * A slotted flap of 26 percent chord moves the zero lift angle by about 13
 * degrees at full travel: the NACA 23012 with a 25.7 percent slotted flap at 40
 * degrees carries 1.35 at zero angle of attack against 0.01 clean, which is a
 * shift of 12.9 degrees on a slope of 6.0 per radian.
 *
 * The classic tau for a hinge at 74 percent chord is 0.50, and a linear model
 * with that value would give 25 degrees of shift at the 50 degree landing
 * setting. The linear law is only true near zero deflection, so the model uses
 * the SECANT value that reproduces the measured shift at full travel:
 * 0.227 / 0.873 = 0.26.
 *
 * Source: Abbott and von Doenhoff, "Theory of Wing Sections", and NACA TN 664.
 * Confidence: derived from firm section data.
 */
const FLAP_TAU = 0.26;

/**
 * Peak lift the flap adds to its own section, per radian of flap deflection.
 *
 * Raymer gives 1.3 for the section peak increment of a slotted flap. Hoerner
 * gives 1.0 to 1.3 for the same device. The Me 262 flap is a plain slot with no
 * Fowler travel and a flap chord near 26 percent, which sits at the low end of
 * that band, so the model takes 1.2 at the 50 degree landing setting.
 *
 * Read the pair with FLAP_TAU. The zero lift shift gives 1.55 of extra lift in
 * the straight part of the curve and the peak rises by 1.2, so the section
 * stalls (1.55 - 1.20) / 6.8 = 2.9 degrees earlier. That is the mechanism the
 * measured sections show.
 *
 * Source: Raymer, "Aircraft Design: A Conceptual Approach", table of flap
 * increments, and Hoerner, "Fluid Dynamic Lift", chapter 5. Confidence:
 * estimate.
 */
const FLAP_CLMAX_DELTA = 1.2 / FLAP_LANDING_ANGLE; // per rad of deflection

/**
 * Extra fall of the stall angle of the flapped section, on top of the 2.9
 * degrees that the shift and the peak rise already give.
 *
 * Measured slotted flap sections stall 3 to 4 degrees before the clean section.
 * The rest of that fall comes from the higher circulation loading the nose of
 * the main element. The model carries 1.2 degrees at the landing setting, which
 * puts the total at 4.1 degrees. Confidence: estimate.
 */
const FLAP_ALPHA_DELTA = (1.2 * DEG) / FLAP_LANDING_ANGLE; // rad per rad

/**
 * Aileron effectiveness. The hinge sits at 72 percent chord, so the control
 * chord ratio is 0.28 and the theoretical tau is 0.52. The gap and the boundary
 * layer take about 15 percent of it. Source: Perkins and Hage, "Airplane
 * Performance, Stability and Control", flap effectiveness chart. Confidence:
 * derived. Bead b33 tunes it against the published roll rate.
 */
const AILERON_TAU = 0.44;

/** Elevator, hinge at 68 percent chord, control chord ratio 0.32, tau 0.56. */
const ELEVATOR_TAU = 0.45;

/** Rudder, hinge at 62 percent chord, control chord ratio 0.38, tau 0.61. */
const RUDDER_TAU = 0.48;

/**
 * How much the open slat raises the stall angle of its strip.
 *
 * A leading edge slat of the size the Me 262 carried moves the stall from about
 * 13 degrees to about 19 degrees on the section it covers. Source: Hoerner,
 * "Fluid Dynamic Lift", chapter 5. Confidence: estimate.
 */
const SLAT_ALPHA_DELTA = 6 * DEG;

/**
 * Local angle of attack at which the slat starts to open.
 *
 * The slat of the Me 262 had no actuator. The suction peak at the nose pulled it
 * out. surface.ts opens it over a 2 degree band above this angle, so the slat is
 * fully out at 10 degrees, three degrees below the stall of the tip section.
 * Confidence: estimate.
 *
 * src/aircraft/me262/systems.ts reads this value, so the mechanism the pilot
 * sees opens at the angle the aerodynamics uses.
 */
export const SLAT_DEPLOY_ALPHA = 8 * DEG;

// --- Tail plane, from the three view -----------------------------------------

/** Station of the tailplane root quarter chord. */
const TAIL_ROOT_QUARTER_STATION = 8.905; // m
/** Half width of the fuselage where the tailplane leaves it. */
const TAIL_ROOT_Y = 0.7; // m
/** Exposed span of one tailplane half. */
const TAIL_PANEL_SPAN = 1.8; // m
const TAIL_ROOT_CHORD = 1.42; // m
const TAIL_TIP_CHORD = 0.78; // m
const TAIL_SWEEP = 12 * DEG;
/** Height of the tailplane above the fuselage reference plane. */
const TAIL_HEIGHT = 0.7; // m
const ELEVATOR_SPAN: readonly [number, number] = [0.3, 1.7];

/**
 * Aspect ratio of the whole tailplane, gross span squared over gross area. The
 * gross span is 5.00 m and the gross area, carry through included, is 5.95 m2.
 */
const TAIL_ASPECT_RATIO = 4.2;
/** Matches the Helmbold slope of 3.97 per radian at that aspect ratio. */
const TAIL_OSWALD = 0.85;

/**
 * Tailplane incidence.
 *
 * The wing sits at 1.5 degrees to the fuselage datum and the tailplane sits at
 * zero, so the tail already works at 1.5 degrees less than the wing. The real
 * aircraft trimmed with an electric stabilizer, which bead b19 drives. The
 * model needs no built in offset.
 */
const TAIL_INCIDENCE = 0;

// --- Fin, from the three view -------------------------------------------------

const FIN_ROOT_QUARTER_STATION = 8.1175; // m
/** Height of the fin root above the fuselage reference plane. */
const FIN_ROOT_HEIGHT = 0.5; // m
const FIN_SPAN = 1.67; // m
const FIN_ROOT_CHORD = 2.55; // m
const FIN_TIP_CHORD = 1.15; // m
/** The fin quarter chord sweeps 30.7 degrees. Estimate from a three view. */
const FIN_TAN_SWEEP = 0.594;
const RUDDER_SPAN: readonly [number, number] = [0.25, 1.52];

/**
 * Effective aspect ratio of the fin.
 *
 * The geometric aspect ratio is 0.90. The fuselage below the fin and the
 * tailplane across its root both act as end plates and roughly halve the tip
 * loss, so the effective value is about 1.5 times the geometric one. Source:
 * USAF DATCOM, fin end plate effect. Confidence: estimate.
 */
const FIN_ASPECT_RATIO = 1.5;
const FIN_OSWALD = 0.85;

// --- Nacelles -----------------------------------------------------------------

/** Station of the nacelle intake lip and the nacelle length. */
const NACELLE_FRONT_STATION = 3.91; // m
const NACELLE_LENGTH = 3.8; // m
const NACELLE_RADIUS = 0.425; // m
/** Spanwise station of the engine center line. Estimate from photographs. */
const NACELLE_SPAN_STATION = 2.05; // m
/** Height of the engine center line below the reference plane. */
const NACELLE_HEIGHT = -0.53; // m

// ---------------------------------------------------------------------------
// Control channels.
// ---------------------------------------------------------------------------

/**
 * Index of each control inside the deflection array that
 * AeroAssembly.evaluate receives. Every entry holds a deflection in RADIANS,
 * with the sign that the module comment fixes.
 *
 * `slat` names a channel that no strip reads. surface.ts opens a slat from the
 * local angle of attack of its own strip, because the slat of the Me 262 had no
 * actuator. The index exists so that a caller sizes the array correctly and so
 * that a later bead can drive a locked or a failed slat through it.
 */
export const CONTROL_INDEX = {
  aileron: 0,
  elevator: 1,
  rudder: 2,
  flap: 3,
  slat: 4,
} as const;

/** Length the control deflection array must have. */
export const CONTROL_COUNT = 5;

// ---------------------------------------------------------------------------
// Engine mount positions, body axes, meters from the center of gravity.
// ---------------------------------------------------------------------------

/**
 * Where createJumo004 of src/aircraft/me262/engine.ts hangs each engine.
 *
 * The y offset of 2.05 m is what turns an asymmetric thrust into a yaw moment.
 * One engine out at full power on the other gives 8800 * 2.05 = 18 kN m of yaw,
 * which the fin and the rudder must hold. The z offset of 0.397 m puts the
 * thrust line BELOW the center of gravity, so opening the throttles raises the
 * nose. Both offsets follow from the geometry and from the mass model.
 */
export const ENGINE_POSITION_LEFT = new Vector3(
  CG_OFFSET_FROM_NOSE - (NACELLE_FRONT_STATION + NACELLE_LENGTH / 2),
  -NACELLE_SPAN_STATION,
  CG_HEIGHT_FROM_DATUM - NACELLE_HEIGHT,
);

export const ENGINE_POSITION_RIGHT = new Vector3(
  ENGINE_POSITION_LEFT.x,
  NACELLE_SPAN_STATION,
  ENGINE_POSITION_LEFT.z,
);

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

/** Local streamwise chord of the wing at a span station. */
function wingChord(y: number): number {
  return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * (y / HALF_SPAN);
}

/**
 * Fraction of the strip between `a0` and `a1` that the control between `b0` and
 * `b1` covers. The value scales the effectiveness of that strip.
 */
function coverage(a0: number, a1: number, b0: number, b1: number): number {
  const overlap = Math.min(a1, b1) - Math.max(a0, b0);
  return overlap > 0 ? overlap / (a1 - a0) : 0;
}

/**
 * Blends the separation fit of two sections.
 *
 * blendAirfoils blends the coefficient tables, and the fit of stall.ts must
 * follow them. The fit is a smooth function of the section, so a linear blend of
 * the break angle and of the two shape constants tracks the fit of the blended
 * section closely. The two Me 262 sections differ by only two degrees of stall
 * angle, so the error of the blend is far smaller than the spread of the
 * published section data.
 */
function blendStall(a: StallParams, b: StallParams, t: number): StallParams {
  return {
    a1: lerp(a.a1, b.a1, t),
    s1: lerp(a.s1, b.s1, t),
    s2: lerp(a.s2, b.s2, t),
    tf: lerp(a.tf, b.tf, t),
  };
}

/** Turns a station and a height into a body axis position. */
function bodyPosition(station: number, y: number, height: number): Vector3 {
  return new Vector3(CG_OFFSET_FROM_NOSE - station, y, CG_HEIGHT_FROM_DATUM - height);
}

/**
 * Boundaries of the eight cosine spaced wing strips of one side, in meters from
 * the plane of symmetry. The list runs from the root to the tip.
 */
export function wingStationBoundaries(): number[] {
  const out: number[] = [];
  for (let k = 0; k <= 8; k++) {
    out.push(HALF_SPAN * Math.sin((k * Math.PI) / 16));
  }
  return out;
}

// ---------------------------------------------------------------------------
// The elements.
// ---------------------------------------------------------------------------

/**
 * Builds the eight strips of one wing. `side` is -1 for the left wing and +1 for
 * the right wing. The two sides are exact mirrors, so the aircraft makes no roll
 * moment, no yaw moment and no side force at zero sideslip.
 */
function wingStrips(side: number): SurfaceDef[] {
  const boundaries = wingStationBoundaries();
  const out: SurfaceDef[] = [];
  for (let i = 0; i < 8; i++) {
    const inner = boundaries[i];
    const outer = boundaries[i + 1];
    const width = outer - inner;
    const center = 0.5 * (inner + outer);
    const chord = wingChord(center);
    const fraction = center / HALF_SPAN;

    const flapCover =
      coverage(inner, outer, FLAP_INNER[0], FLAP_INNER[1]) +
      coverage(inner, outer, FLAP_OUTER[0], FLAP_OUTER[1]);
    const aileronCover = coverage(inner, outer, AILERON_SPAN[0], AILERON_SPAN[1]);

    // The left panel and the right panel need opposite aileron signs, because a
    // roll to the right takes lift off the right wing and adds it to the left.
    const aileronSign = side >= 0 ? 1 : -1;

    const airfoil: Airfoil = blendAirfoils(
      NACA_0011,
      NACA_0009,
      fraction,
      `Me 262 wing ${(fraction * 100).toFixed(0)} percent semi span`,
    );

    out.push({
      name: `wing ${side < 0 ? 'left' : 'right'} ${i + 1}`,
      position: bodyPosition(
        WING_ROOT_QUARTER_STATION + TAN_SWEEP * center,
        side * center,
        TAN_DIHEDRAL * Math.max(0, center - DIHEDRAL_START),
      ),
      span: width,
      chord,
      area: width * chord,
      incidence: lerp(ROOT_INCIDENCE, TIP_INCIDENCE, fraction),
      dihedral: center > DIHEDRAL_START ? WING_DIHEDRAL : 0,
      sweep: WING_SWEEP,
      airfoil,
      stall: blendStall(STALL_NACA_0011, STALL_NACA_0009, fraction),
      aspectRatio: WING_ASPECT_RATIO,
      oswaldEfficiency: WING_OSWALD,
      controlIndex: aileronCover > 0 ? CONTROL_INDEX.aileron : -1,
      controlEffectiveness: aileronSign * AILERON_TAU * aileronCover,
      flapIndex: flapCover > 0 ? CONTROL_INDEX.flap : -1,
      flapEffectiveness: -FLAP_TAU * flapCover,
      // Both flap peak terms scale with the covered fraction, exactly as the
      // zero lift shift does. Only the covered part of the strip carries a slot.
      flapClMaxDelta: FLAP_CLMAX_DELTA * flapCover,
      flapAlphaDelta: FLAP_ALPHA_DELTA * flapCover,
      hasSlat: center > SLAT_SPAN[0],
      slatAlphaDelta: SLAT_ALPHA_DELTA,
      slatDeployAlpha: SLAT_DEPLOY_ALPHA,
    });
  }
  return out;
}

/** Builds the two strips of one tailplane half. */
function tailStrips(side: number): SurfaceDef[] {
  const out: SurfaceDef[] = [];
  const half = TAIL_PANEL_SPAN / 2;
  for (let i = 0; i < 2; i++) {
    const inner = i * half;
    const outer = inner + half;
    const center = 0.5 * (inner + outer);
    const fraction = center / TAIL_PANEL_SPAN;
    const chord = lerp(TAIL_ROOT_CHORD, TAIL_TIP_CHORD, fraction);
    const cover = coverage(inner, outer, ELEVATOR_SPAN[0], ELEVATOR_SPAN[1]);
    out.push({
      name: `tailplane ${side < 0 ? 'left' : 'right'} ${i + 1}`,
      position: bodyPosition(
        TAIL_ROOT_QUARTER_STATION + Math.tan(TAIL_SWEEP) * center,
        side * (TAIL_ROOT_Y + center),
        TAIL_HEIGHT,
      ),
      span: half,
      chord,
      area: half * chord,
      incidence: TAIL_INCIDENCE,
      dihedral: 0,
      sweep: TAIL_SWEEP,
      airfoil: NACA_0009,
      stall: STALL_NACA_0009,
      aspectRatio: TAIL_ASPECT_RATIO,
      oswaldEfficiency: TAIL_OSWALD,
      // A positive elevator command must REMOVE tail lift, so the sign is
      // positive. See the module comment.
      controlIndex: CONTROL_INDEX.elevator,
      controlEffectiveness: ELEVATOR_TAU * cover,
      flapIndex: -1,
      flapEffectiveness: 0,
      flapClMaxDelta: 0,
      flapAlphaDelta: 0,
      hasSlat: false,
      slatAlphaDelta: 0,
      slatDeployAlpha: 0,
    });
  }
  return out;
}

/**
 * Builds the two fin strips.
 *
 * A fin is a strip with 90 degrees of dihedral. createSurface then turns the
 * span axis up and the normal axis to the right, with no special case anywhere
 * in the force path. The strips sit on the plane of symmetry, so their side is
 * the right hand side and the dihedral turn is the one the code expects.
 */
function finStrips(): SurfaceDef[] {
  const out: SurfaceDef[] = [];
  const half = FIN_SPAN / 2;
  for (let i = 0; i < 2; i++) {
    const lower = i * half;
    const upper = lower + half;
    const center = 0.5 * (lower + upper);
    const chord = lerp(FIN_ROOT_CHORD, FIN_TIP_CHORD, center / FIN_SPAN);
    const cover = coverage(lower, upper, RUDDER_SPAN[0], RUDDER_SPAN[1]);
    out.push({
      name: `fin ${i + 1}`,
      position: bodyPosition(
        FIN_ROOT_QUARTER_STATION + FIN_TAN_SWEEP * center,
        0,
        FIN_ROOT_HEIGHT + center,
      ),
      span: half,
      chord,
      area: half * chord,
      incidence: 0,
      dihedral: Math.PI / 2,
      sweep: Math.atan(FIN_TAN_SWEEP),
      airfoil: NACA_0009,
      stall: STALL_NACA_0009,
      aspectRatio: FIN_ASPECT_RATIO,
      oswaldEfficiency: FIN_OSWALD,
      // A positive rudder command must ADD fin lift, so the sign is negative.
      controlIndex: CONTROL_INDEX.rudder,
      controlEffectiveness: -RUDDER_TAU * cover,
      flapIndex: -1,
      flapEffectiveness: 0,
      flapClMaxDelta: 0,
      flapAlphaDelta: 0,
      hasSlat: false,
      slatAlphaDelta: 0,
      slatDeployAlpha: 0,
    });
  }
  return out;
}

/**
 * Returns every lifting strip of the aircraft, in this order:
 * left wing 0 to 7, right wing 8 to 15, left tailplane 16 and 17, right
 * tailplane 18 and 19, fin 20 and 21. The strips of one panel run from the root
 * to the tip.
 */
export function me262Surfaces(): SurfaceDef[] {
  return [
    ...wingStrips(-1),
    ...wingStrips(1),
    ...tailStrips(-1),
    ...tailStrips(1),
    ...finStrips(),
  ];
}

/**
 * Returns the fuselage and the two nacelles.
 *
 * The fuselage numbers come from fuselageShape of src/aircraft/me262/mass.ts,
 * which integrates the same section table the mass model weighs. The shape the
 * aerodynamics sees and the shape the mass model weighs are therefore one shape.
 *
 * The Munk moment of the fuselage is what the tailplane has to beat. With a
 * volume of 9.3 m3 it gives dCm/dalpha near +0.44 per radian on its own, which
 * moves the aerodynamic center of the aircraft about 9 percent of the mean
 * aerodynamic chord forward. That is the range a fuselage of this fineness
 * really produces.
 */
export function me262Bodies(): BodyDef[] {
  const shape = fuselageShape();
  const nacelleFrontalArea = Math.PI * NACELLE_RADIUS * NACELLE_RADIUS;
  const bodies: BodyDef[] = [
    {
      name: 'fuselage',
      // The cross flow load rides with the side area, so the reference point is
      // the centroid of the side view.
      position: bodyPosition(shape.sideAreaStation, 0, 0.05),
      length: 10.6,
      maxDiameter: shape.maxDiameter,
      volume: shape.volume,
      sideArea: shape.sideArea,
      frontalArea: shape.frontalArea,
      // Parasite drag of a streamlined fuselage of fineness 8, on the frontal
      // area, with the canopy and the wing root fairing inside it.
      // Source: Hoerner, "Fluid Dynamic Drag", chapter 6. Confidence: estimate.
      axialDragCoefficient: 0.09,
      crossFlowDragCoefficient: 1.2,
      munkFactor: 0.47,
    },
  ];
  for (const side of [-1, 1]) {
    bodies.push({
      name: side < 0 ? 'nacelle left' : 'nacelle right',
      position: bodyPosition(
        NACELLE_FRONT_STATION + NACELLE_LENGTH / 2,
        side * NACELLE_SPAN_STATION,
        NACELLE_HEIGHT,
      ),
      length: NACELLE_LENGTH,
      maxDiameter: 2 * NACELLE_RADIUS,
      // The nacelle is a duct, not a closed body. Air passes through it, so its
      // apparent mass is far below that of a solid body of the same envelope.
      // The volume below is the envelope less the duct. Confidence: estimate.
      volume: 0.9,
      sideArea: NACELLE_LENGTH * 2 * 0.415,
      frontalArea: nacelleFrontalArea,
      // The intake momentum drag belongs to the engine model, which already
      // reports NET thrust. This coefficient carries the external skin friction
      // and the boat tail drag only. Confidence: estimate.
      axialDragCoefficient: 0.06,
      crossFlowDragCoefficient: 1.2,
      // Reduced from the 0.47 of a closed body, for the same through flow reason
      // as the volume. Confidence: estimate.
      munkFactor: 0.3,
    });
  }
  return bodies;
}

/**
 * Returns the three parent surfaces. assembly.ts solves one induced angle for
 * each group, because the downwash follows the lift of a whole surface and not
 * the lift of one strip.
 */
export function me262Groups(): GroupDef[] {
  const surfaces = me262Surfaces();
  const areaOf = (from: number, to: number): number => {
    let sum = 0;
    for (let i = from; i < to; i++) {
      sum += surfaces[i].area;
    }
    return sum;
  };
  const range = (from: number, to: number): number[] => {
    const out: number[] = [];
    for (let i = from; i < to; i++) {
      out.push(i);
    }
    return out;
  };
  return [
    {
      name: 'wing',
      surfaceIndices: range(0, 16),
      aspectRatio: WING_ASPECT_RATIO,
      oswaldEfficiency: WING_OSWALD,
      area: WING_AREA,
    },
    {
      name: 'horizontal tail',
      surfaceIndices: range(16, 20),
      aspectRatio: TAIL_ASPECT_RATIO,
      oswaldEfficiency: TAIL_OSWALD,
      area: areaOf(16, 20),
    },
    {
      name: 'fin',
      surfaceIndices: range(20, 22),
      aspectRatio: FIN_ASPECT_RATIO,
      oswaldEfficiency: FIN_OSWALD,
      area: areaOf(20, 22),
    },
  ];
}

/** Builds the complete aerodynamic assembly of the Me 262 A-1a. */
export function createMe262Assembly(): AeroAssembly {
  return createAssembly(me262Surfaces(), me262Bodies(), me262Groups());
}

// ---------------------------------------------------------------------------
// Reported geometry. The tests and bead b33 read these.
// ---------------------------------------------------------------------------

/**
 * Horizontal tail volume coefficient, V_h = l_h S_h / (S c).
 *
 * l_h is the distance from the center of gravity to the area weighted quarter
 * chord of the tailplane. The value comes out near 0.33, which sits in the
 * 0.30 to 0.60 band of a single seat fighter of the period.
 */
export function horizontalTailVolume(): number {
  const surfaces = me262Surfaces();
  let area = 0;
  let moment = 0;
  for (let i = 16; i < 20; i++) {
    area += surfaces[i].area;
    moment += surfaces[i].area * -surfaces[i].position.x;
  }
  return (moment / area) * (area / (WING_AREA * MAC));
}

/**
 * Vertical tail volume coefficient, V_v = l_v S_v / (S b).
 *
 * The value comes out near 0.032, which sits in the 0.03 to 0.05 band of a
 * fighter with two wing mounted engines to hold on one engine.
 */
export function verticalTailVolume(): number {
  const surfaces = me262Surfaces();
  let area = 0;
  let moment = 0;
  for (let i = 20; i < 22; i++) {
    area += surfaces[i].area;
    moment += surfaces[i].area * -surfaces[i].position.x;
  }
  return (moment / area) * (area / (WING_AREA * WING_SPAN));
}
