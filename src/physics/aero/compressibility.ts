/**
 * Mach number effects on the section coefficients.
 *
 * The Me-262 was the first fighter to meet compressibility every day. Its
 * documented tuck onset is Mach 0.83 and its documented limit is Mach 0.86.
 * Past the limit the nose dropped, the elevator went light, and the pilot could
 * not pull out. The correct recovery was to close the throttles and wind the
 * trim wheel back, not to pull harder. This module holds the five effects that
 * produce that behavior.
 *
 *   clScale      Prandtl-Glauert lift growth below the critical Mach number, and
 *                the fall of the lift curve slope above it.
 *   cdAdd        Wave drag rise above the critical Mach number.
 *   acShift      The center of pressure moves back. This is the Mach tuck.
 *   controlScale The control surfaces lose authority.
 *   clMaxScale   Shock induced separation takes the peak lift away.
 *
 * WHAT BEAD b58 CHANGED, AND WHY THE MODEL HAD NO TUCK BEFORE IT. Two of the
 * five effects were wrong in a way that hid the third.
 *
 *   The lift curve slope grew with the Prandtl-Glauert rule all the way to Mach
 *   0.9, where the rule has no meaning. Every surface then made 30 percent more
 *   lift at a fixed angle of attack at Mach 0.85 than at Mach 0.5. The downwash
 *   follows the wing lift, so the tail lost half a degree of angle, its load
 *   turned from a small up load into a down load, and a down load behind the
 *   center of gravity is a NOSE UP moment three times the size of the tuck.
 *   SLOPE_LOSS_SCALE holds the slope down where the shock really holds it down.
 *
 *   Every surface met the same shock at the same Mach number, whatever its
 *   thickness. The tailplane and the fin are 9 percent sections and the wing root
 *   is 11 percent, so the tail was carrying the wave drag of a wing. That drag
 *   acts 0.83 m ABOVE the center of gravity and made another large nose up
 *   moment. THICKNESS_MACH_RELIEF and WAVE_DRAG_THICKNESS_EXPONENT give a thin
 *   section its later and smaller shock.
 *
 * Sweep. The Me-262 carries 18.5 degrees of sweep at the LEADING EDGE, which is
 * 15.72 degrees at the quarter chord. Simple sweep theory says that only the
 * velocity component normal to the quarter chord line drives the section
 * pressures, so every shock driven effect answers to the normal Mach number
 * M cos(sweep), not to the free stream Mach number.
 *
 * What sweep relieves: the critical Mach number, the wave drag rise, the
 * aerodynamic center shift, the loss of control power, and the loss of peak
 * lift. All five are section pressure effects and all five move to a higher free
 * stream Mach number.
 *
 * What sweep does not relieve:
 *   - It delays the drag rise. It does not remove it. The rise still comes.
 *   - It does not help a surface with a different sweep. The caller passes the
 *     sweep of the surface it works on. The Me-262 tailplane has less sweep than
 *     the wing, so the tail meets its own shock first. That is one reason the
 *     elevator goes light before the wing loses lift.
 *   - Simple sweep theory assumes a wing of infinite span. A real wing of aspect
 *     ratio 7.21 with a fuselage at the root and two tips gets less than the full
 *     benefit. SWEEP_RELIEF_EXPONENT lets bead b33 reduce the relief.
 *
 * Thickness. A thick section makes the flow work harder over its crest, so it
 * meets its shock at a LOWER Mach number and pays a larger wave drag when it
 * does. The caller passes the t/c of the section, and it defaults to the
 * reference section of the tables. Sweep and thickness both act on the same
 * scale, so the two meet in one shock Mach number inside machCorrection.
 *
 * Every constant below carries the free stream Mach number at the reference
 * sweep AND at the reference thickness, because that is the number the flight
 * manual and the test reports use. The module converts each anchor to a normal
 * Mach number when it builds the table. Bead b33 tunes these anchors against the
 * level speed and dive data.
 *
 * This module is a pure coefficient source. It imports only from src/math.
 */

import { lookup1d, table1d } from '@/math/tables';
import type { Table1D } from '@/math/tables';
import { DEG } from '@/math/units';

/** The five Mach corrections an aerodynamic element applies. */
export interface MachCorrection {
  /** Multiplies the lift curve slope. 1.0 below Mach 0.3. */
  clScale: number;
  /** Adds to the section drag coefficient. 0 below the critical Mach number. */
  cdAdd: number;
  /** The center of pressure in chord fractions from the leading edge. 0.25 below the tuck. */
  acShift: number;
  /** Multiplies control surface effectiveness. 1.0 below Mach 0.75. */
  controlScale: number;
  /** Multiplies the peak lift coefficient. 1.0 below Mach 0.75. */
  clMaxScale: number;
}

// The QUARTER CHORD sweep of the Me-262 wing. Every Mach anchor below is a free
// stream value at this sweep.
//
// BEAD b65 CHANGED THIS NUMBER FROM 18.5 TO 15.72 DEGREES AND THE WING FEELS
// NOTHING. The published 18.5 degrees is the sweep of the LEADING EDGE, not of
// the quarter chord. See the note on the sweep in docs/CONVENTIONS.md section 8.
// The anchors are free stream values AT THIS SWEEP, so the wing reads every
// table at the free stream Mach number whatever this angle is, and only the
// surfaces with a DIFFERENT sweep move. The tailplane sweeps 12 degrees, so its
// shock used to arrive 3.1 percent early in free stream terms and now arrives
// 1.6 percent early. The fin sweeps 30.7 degrees and keeps a little more relief.
// Source: docs/CONVENTIONS.md section 8, derived from the firm leading edge
// sweep and the firm span and area.
export const REFERENCE_SWEEP = 15.72 * DEG; // rad

// The exponent of the sweep relief. A value of 1 gives full simple sweep theory.
// A value of 0 removes the relief. Bead b33 can lower it if the model reaches
// its drag rise too late.
export const SWEEP_RELIEF_EXPONENT = 1.0;

// The thickness ratio every Mach anchor below belongs to. It is the root section
// of the Me 262 wing, NACA 00011, which is the section that meets its shock
// first and therefore the one the aircraft answers to.
// Source: docs/CONVENTIONS.md section 8, confidence: firm.
export const REFERENCE_THICKNESS = 0.11;

// How far the shock moves up the Mach scale for each unit of thickness the
// section loses. A THIN SECTION MEETS ITS SHOCK LATER.
//
// The velocity over a section grows with its thickness, so a thick section
// reaches the speed of sound over its crest at a lower free stream Mach number.
// The measured critical Mach numbers of symmetric sections at a low lift
// coefficient run 0.80 at 6 percent, 0.76 at 9 percent, 0.72 at 12 percent, and
// 0.68 at 15 percent, which is a straight line of slope -1.33 against t/c.
// Source: Abbott and von Doenhoff, "Theory of Wing Sections", the high speed
// data, and the classic critical Mach chart it comes from. Confidence: firm for
// the shape, estimate for the slope.
//
// WHY THE MODEL NEEDS IT. This aircraft carries three different sections. The
// wing runs from 11 percent at the root to 9 percent at the tip, and both tail
// surfaces are 9 percent. Without this term the tailplane meets the same shock
// as the wing root, and its wave drag then acts 0.83 m ABOVE the center of
// gravity and makes a large nose up moment that hides the tuck. It also gives
// the wing ROOT the shock first, which is what a wing with washout and a thicker
// root really does. The root of a swept wing sits ahead of the tip, so a load
// that leaves the root moves aft, which is one more nose down term.
export const THICKNESS_MACH_RELIEF = 1.33;

// The exponent of the thickness in the wave drag. Transonic similarity gives the
// wave drag of a section as (t/c)^(5/3). A 9 percent section therefore pays 0.72
// of what an 11 percent section pays at the same shock strength.
// Source: transonic similarity, and Hoerner, "Fluid Dynamic Drag", chapter 15.
// Confidence: firm.
export const WAVE_DRAG_THICKNESS_EXPONENT = 5 / 3;

// The free stream Mach number where the wave drag starts to rise.
//
// Reasoning. The maximum level speed at 6000 m is 870 km/h, that is 241.7 m/s.
// The speed of sound at 6000 m in the standard atmosphere is 316.4 m/s, so that
// speed is Mach 0.764. The drag rise must stay clear of that point, or the
// aircraft cannot reach its published speed. An onset at Mach 0.78 leaves a
// margin of 0.016 in Mach. The onset also agrees with the section: an 11 percent
// symmetric section at low lift has a critical Mach near 0.75, and the 15.72
// degrees of quarter chord sweep raise that to 0.78 in free stream terms.
// Confidence: derived from firm speed data.
export const CRITICAL_MACH = 0.78;

// The free stream Mach number where the WHOLE AIRCRAFT starts to pitch nose
// down. Source: docs/CONVENTIONS.md section 8, confidence: firm.
//
// READ THIS WITH AC_SHIFT_MACH BELOW. The aerodynamic center of the section
// starts to move at the CRITICAL Mach number, because that is where the shock
// first stands on the surface. The published tuck onset is a different number:
// it is where the nose down moment of that movement has grown past everything
// that pulls the other way, so that the pilot feels the nose go down. The model
// therefore starts the shift at CRITICAL_MACH and reaches the documented onset
// on the whole aircraft. test/flight/mach.test.ts measures the second number,
// which is the one the flight manual gives.
export const TUCK_ONSET_MACH = 0.83;

// The free stream Mach number where the aircraft goes out of control.
// Source: docs/CONVENTIONS.md section 8, confidence: firm.
export const MACH_LIMIT = 0.86;

// Wave drag rise of the REFERENCE SECTION. Free stream Mach against the drag
// coefficient the shock adds. A thinner section pays less. See
// WAVE_DRAG_THICKNESS_EXPONENT.
//
// Anchor by anchor:
//   0.78  0.0000  The onset. Level flight at 870 km/h and 6000 m sits at Mach
//                 0.764, below this point, so the published speed stays reachable.
//   0.80  0.0024  A small rise. About 1100 N of extra drag at 6000 m. The engines
//                 still have this much left, so the aircraft can pass 0.80 in a
//                 shallow dive.
//   0.82  0.0096  About 4600 N of extra drag at 6000 m. Two Jumo 004 at that
//                 altitude and speed give about 7000 N in total, so level flight
//                 runs out of thrust near here.
//   0.84  0.0290  About 14 kN. Level flight is gone. Only a dive gets past.
//   0.86  0.0670  The documented limit. The extra drag alone is many times the
//                 available thrust, so the aircraft needs a steep dive to hold it.
//   0.88  0.1280  Past the limit. The dive angle must keep growing.
//   0.92  0.2560  Deep in the rise.
//   1.00  0.3840  The plateau of the rise for a thin swept wing.
//
// THE ANCHORS ARE 1.6 TIMES WHAT THEY WERE BEFORE BEAD b58, AND THE AIRCRAFT PAYS
// THE SAME DRAG. The reasoning above is written at the AIRCRAFT level, and it was
// written when every surface used this table with no thickness term. The
// tailplane, the fin and the outer wing are 9 percent sections, and they now meet
// their shock later and pay less of it. The aircraft level drag rise fell by a
// third, so the anchors carry that third back. The model measures 36 kN of drag
// at Mach 0.86 and 7000 m in a held dive, which is what it measured before.
// Confidence: estimate, anchored on the firm level speed and limit Mach numbers.
export const WAVE_DRAG_MACH: readonly number[] = [0.78, 0.8, 0.82, 0.84, 0.86, 0.88, 0.92, 1.0];
export const WAVE_DRAG_CD: readonly number[] = [
  0.0, 0.0024, 0.0096, 0.029, 0.067, 0.128, 0.256, 0.384,
];

// Aerodynamic center of the section, in chord fractions from the leading edge,
// against the free stream Mach number at the reference sweep. THIS TABLE IS THE
// MACH TUCK. Nothing else in the model makes one.
//
// The mechanism. Below the critical Mach number the load of a thin section sits
// at the quarter chord. A shock then stands on the upper surface, and the whole
// of the extra suction it holds sits BEHIND it. The load moves aft with the
// shock, and the shock walks toward the trailing edge as the Mach number rises.
// A section that carries its load at half chord instead of quarter chord has
// moved its aerodynamic center a quarter of a chord aft. On the 1.82 m mean
// aerodynamic chord of this wing that is 0.45 m, against a static margin of
// 0.05 chord, which is 0.09 m. The aircraft therefore goes from just stable to
// very stable, and a very stable aircraft in a dive holds the nose DOWN.
//
// WHERE THE SHIFT STARTS. It starts at CRITICAL_MACH, because that is where the
// shock appears. It does not start at TUCK_ONSET_MACH: the published onset is
// where the whole aircraft goes nose down, and the aircraft has to overcome its
// own nose up trend first. See the note on TUCK_ONSET_MACH.
//
// WHERE THE SHIFT STOPS. At half chord. A section in fully supersonic flow
// carries its load at mid chord, and no section carries it behind that point, so
// the table holds 0.5 from Mach 0.85 upward and never goes past it.
//
// Anchor by anchor. The value stays at the quarter chord to 0.78, runs aft as
// the shock strengthens, and reaches half chord at 0.85. What that gives on the
// whole aircraft: the neutral point moves 0.17 of the mean aerodynamic chord aft
// between Mach 0.78 and Mach 0.86, which takes the static margin from 0.05 to
// 0.22. Published transonic neutral point travel for a swept wing aircraft runs
// 0.15 to 0.25 chord, so the model sits inside that band. The measured onset on
// the whole aircraft is Mach 0.825, against the documented 0.83.
// Source: the transonic aerodynamic center travel of measured swept wing
// aircraft, and supersonic thin airfoil theory for the mid chord limit.
// Confidence: estimate, anchored on the firm onset Mach number.
//
//
// THIS TABLE IS AN AIRCRAFT LEVEL LUMPED PARAMETER. IT IS NOT A SECTION
// MEASUREMENT, AND NO MEASURED SECTION MOVES THIS FAR. READ BEAD b66.
//
// What the measured section does. NACA TN 3501, Nelson and McDevitt, June 1955,
// tested 22 rectangular NACA 63A0XX wings on the Ames transonic bump from Mach
// 0.40 to 1.10. Its figure 16 gives dCm/dCL about the quarter chord, so the
// aerodynamic center sits at 0.25 - dCm/dCL. For the aspect ratio 6 wing with
// the 10 percent section, which is the closest model in the report to this wing:
//
//   Mach 0.40 to 0.80   dCm/dCL near +0.01, so the center sits at 0.24 c
//   Mach 0.85           dCm/dCL near -0.07, so the center sits at 0.32 c
//   Mach 0.90           dCm/dCL near +0.22, a MOMENT REVERSAL, center at 0.03 c
//   Mach 0.95 to 1.10   dCm/dCL near -0.14, so the center sits at 0.39 c
//
// So the real section travels about 0.07 chord by Mach 0.85, it never reaches
// half chord anywhere in the tested range, and on the way it swings sharply
// FORWARD near Mach 0.90. This table asks for 0.25 chord of travel by Mach 0.85,
// monotone aft. The comparison is worse than the ratio of 3.6 those two numbers
// give, because TN 3501 tested an UNSWEPT wing, so its Mach number is a normal
// Mach number. At a free stream Mach of 0.85 the sweep of this wing leaves a
// normal Mach of 0.818, and TN 3501 reads no section travel at all there.
//
// WHERE THE WHOLE AIRCRAFT ANSWER COMES FROM, MEASURED. Between Mach 0.78 and
// Mach 0.86 at 8000 m the neutral point of the model moves from station 5.774 m
// to station 6.215 m, which is 0.441 m, or 0.242 of the mean aerodynamic chord.
// That travel splits as
//
//   0.416 m   94.3 percent   the section shift on the WING strips
//   0.012 m    2.7 percent   the section shift on the TAILPLANE strips, which
//                            lengthens the tail arm
//   0.013 m    3.0 percent   everything else
//
// The last row was measured by flattening this table to 0.25 everywhere, which
// leaves 0.014 m of travel. So the model builds the whole of its Mach tuck out
// of the section shift, and almost nothing out of the parts a real aircraft uses.
//
// WHY THAT IS A DEFECT EVEN THOUGH THE TOTAL IS RIGHT. The travel of 0.242 chord
// sits inside the published 0.15 to 0.25 chord band for a swept wing aircraft,
// and test/flight/mach.test.ts measures the tuck onset at 0.825 against the
// documented 0.83. The total is not the question. The split is. On a real
// aircraft most of the travel comes from the tail: the tailplane meets its own
// shock, loses lift curve slope and dynamic pressure, and the wake behind a wing
// that is losing lift stops turning the flow down, so the downwash slope falls
// and the tail meets more angle. This model has none of that working. Measured
// over the same Mach range, the downwash slope d(epsilon)/d(alpha) RISES from
// 0.550 to 0.555, which is the wrong way, the tail dynamic pressure ratio holds
// at 0.920 exactly, and the lift curve slope of the tailplane falls by 1.2
// percent. Three effects that should carry the tuck carry none of it.
//
// WHAT THAT COSTS. A model that gets the right total from the wrong parts drifts
// when anything near it changes. Any bead that touches the tail arm, the tail
// area, the tail section, the downwash model or the wake will move the tuck by
// the wrong amount, because the tuck does not answer to any of them here.
//
// WHY THE TABLE STILL STANDS. Correcting the split is not a change to this
// table. It needs a Mach dependent downwash slope and a tail dynamic pressure
// that falls through the drag rise, both of which live in
// src/physics/aero/downwash.ts, and then a fresh fit of this table against the
// firm tuck onset. Bead b66 was asked to measure the split and to leave the
// total alone unless the evidence said the total was wrong. The evidence says
// the total is right and the parts are wrong. The measurement is written down
// here so that the next bead starts from it.
export const AC_SHIFT_MACH: readonly number[] = [0.78, 0.8, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 1.0];
export const AC_SHIFT_X: readonly number[] = [
  0.25, 0.29, 0.36, 0.43, 0.48, 0.5, 0.5, 0.5, 0.5,
];

// Control effectiveness. A shock ahead of the hinge line cuts the pressure the
// surface can change, so the stick moves the aircraft less and less.
//
// The elevator loses its authority at the same Mach number where the tuck needs
// it most. That is the trap the Me-262 pilots met. Confidence: estimate.
//
// WHY THESE ANCHORS ARE NOT ROUND NUMBERS. THIS IS THE ONE TABLE THAT DOES NOT
// BELONG TO THE WING. The anchor of the fit is the ELEVATOR, and the elevator
// sits on the TAILPLANE, which sweeps 12 degrees and carries a 9 percent
// section. Every anchor here is still a free stream value at the reference
// sweep, so the tail, with its own sweep and thickness, meets each one a little
// later than the wing does.
//
// Bead b65 corrected the reference sweep from 18.5 to 15.72 degrees. That gave
// EVERY surface with a different sweep a new place on the Mach scale, and the
// tail gained 1.5 percent. Measured, the elevator kept 0.624 of its low speed
// power at Mach 0.86 where it kept 0.435 before, which is outside the band the
// flight test holds. The anchors below therefore carry the same factor back:
// each one is the value it held before b65 times cos(18.5 deg) / cos(15.72 deg),
// which is 0.9852. The elevator then loses the same authority at the same free
// stream Mach number as before, which is the documented behavior and the whole
// point of the fit. The ailerons and the rudder read the table 1.5 percent
// earlier than they did, and no published measurement holds either one.
export const CONTROL_MACH: readonly number[] = [
  0.7389, 0.7684, 0.7881, 0.8079, 0.8276, 0.8473, 0.8867, 0.9852,
];
export const CONTROL_SCALE: readonly number[] = [1.0, 0.95, 0.88, 0.76, 0.58, 0.35, 0.2, 0.15];

// Peak lift against Mach. The shock separates the boundary layer behind it, so
// the wing breaks at a lower angle of attack as Mach grows. Without this table
// the pilot could pull 7 g at Mach 0.86, which the real aircraft could not do.
// Confidence: estimate.
export const CL_MAX_MACH: readonly number[] = [0.75, 0.8, 0.83, 0.86, 0.9, 1.0];
export const CL_MAX_SCALE: readonly number[] = [1.0, 0.92, 0.82, 0.68, 0.5, 0.4];

// The lowest value the Prandtl-Glauert factor 1 - M^2 may take. The rule runs to
// infinity at Mach 1, and the floor of 0.25 caps its growth at 2.0 from a normal
// Mach of 0.866 upward. SHOCK_SLOPE_LOSS below is what really holds the slope
// down. The floor only stops the arithmetic from running away.
export const PG_BETA_SQUARED_FLOOR = 0.25;

// What the section keeps of its Prandtl-Glauert lift curve slope, against the
// free stream Mach number at the reference sweep.
//
// THE PRANDTL-GLAUERT RULE HOLDS BELOW THE CRITICAL MACH NUMBER AND NOWHERE
// ELSE. It is a linear, shock free result. Above the critical Mach number a
// shock stands on the upper surface, the pressure behind it no longer answers
// the angle of attack, and the boundary layer starts to leave the surface at the
// foot of the shock. Measured section data all show the same shape: the slope
// climbs with the rule, rounds over within about 0.05 Mach of the critical
// point, and then falls. The rule alone gives a slope that keeps climbing to
// twice its low speed value, which no section does.
//
// WHY THE MODEL NEEDS THIS TABLE, AND NOT ONLY FOR THE SLOPE. Every surface
// answers this factor, so the rule alone raises the wing lift at a FIXED angle
// of attack by 30 percent between Mach 0.5 and Mach 0.85. The downwash follows
// the wing lift, so the angle at the tail falls by nearly half a degree, the
// tail load turns from a small up load into a down load, and a down load behind
// the center of gravity is a NOSE UP moment. Bead b58 measured that moment at
// three times the size of the nose down moment of the aerodynamic center shift,
// so the model had no tuck at all. The tuck is a real effect that the rule was
// hiding, not a missing term.
//
// Anchor by anchor. The value is 1 up to the critical Mach number, because the
// rule is right there. It holds the slope flat from 0.78 to 0.84, which puts the
// peak of the slope just above the critical point where the measurements put it.
// Above 0.84 the shock is strong and the slope falls, to two thirds of the peak
// at Mach 0.90.
// Source: Abbott and von Doenhoff, "Theory of Wing Sections", the high speed
// section data, and Hoerner, "Fluid Dynamic Lift", chapter 15, which gives the
// same shape for the lift curve slope through the drag rise.
// Confidence: estimate, anchored on the firm tuck onset and limit Mach numbers.
export const SLOPE_LOSS_MACH: readonly number[] = [0.78, 0.82, 0.84, 0.86, 0.88, 0.9, 1.0];
export const SLOPE_LOSS_SCALE: readonly number[] = [1.0, 0.935, 0.9, 0.83, 0.74, 0.65, 0.55];

// The reference sweep turns each free stream anchor into a normal Mach number.
const REFERENCE_RELIEF = Math.cos(REFERENCE_SWEEP);

/** Converts free stream Mach anchors at the reference sweep into normal Mach anchors. */
function toNormalMach(freeStream: readonly number[]): number[] {
  const out = new Array<number>(freeStream.length);
  for (let i = 0; i < freeStream.length; i++) {
    out[i] = freeStream[i] * REFERENCE_RELIEF;
  }
  return out;
}

const WAVE_DRAG_TABLE: Table1D = table1d(toNormalMach(WAVE_DRAG_MACH), WAVE_DRAG_CD.slice());
const AC_SHIFT_TABLE: Table1D = table1d(toNormalMach(AC_SHIFT_MACH), AC_SHIFT_X.slice());
const CONTROL_TABLE: Table1D = table1d(toNormalMach(CONTROL_MACH), CONTROL_SCALE.slice());
const CL_MAX_TABLE: Table1D = table1d(toNormalMach(CL_MAX_MACH), CL_MAX_SCALE.slice());
const SLOPE_LOSS_TABLE: Table1D = table1d(toNormalMach(SLOPE_LOSS_MACH), SLOPE_LOSS_SCALE.slice());

/** Creates a correction that holds the neutral, low speed values. */
export function createMachCorrection(): MachCorrection {
  return { clScale: 1, cdAdd: 0, acShift: 0.25, controlScale: 1, clMaxScale: 1 };
}

/**
 * Writes the five Mach corrections into out and returns out. The function
 * allocates nothing.
 *
 * mach is the free stream Mach number of the element. sweep is the quarter chord
 * sweep of the surface the element belongs to, in radians. thickness is the t/c
 * of its section, and it defaults to the reference section of the tables. A
 * negative sweep and a negative Mach number both give the same result as their
 * magnitude.
 *
 * The three inputs meet in ONE number, the shock Mach number. The sweep and the
 * thickness both say how hard the flow has to work over the section, so both
 * move every shock driven effect up or down the same Mach scale. The
 * Prandtl-Glauert factor is not one of those effects: it is a pure Mach number
 * result of the linear equations, so it answers the normal Mach number alone.
 */
export function machCorrection(
  mach: number,
  sweep: number,
  out: MachCorrection,
  thickness: number = REFERENCE_THICKNESS,
): MachCorrection {
  const m = mach < 0 ? -mach : mach;
  const cosSweep = Math.abs(Math.cos(sweep));
  const relief =
    SWEEP_RELIEF_EXPONENT === 1 ? cosSweep : Math.pow(cosSweep, SWEEP_RELIEF_EXPONENT);
  const normalMach = m * relief;
  // A thin section meets its shock at a higher Mach number, so the tables are
  // read lower down. A section thicker than the reference reads them higher up.
  const shockMach = normalMach - THICKNESS_MACH_RELIEF * (REFERENCE_THICKNESS - thickness);
  const dragScale = Math.pow(thickness / REFERENCE_THICKNESS, WAVE_DRAG_THICKNESS_EXPONENT);

  let betaSquared = 1 - normalMach * normalMach;
  if (betaSquared < PG_BETA_SQUARED_FLOOR) {
    betaSquared = PG_BETA_SQUARED_FLOOR;
  }
  out.clScale = lookup1d(SLOPE_LOSS_TABLE, shockMach) / Math.sqrt(betaSquared);
  out.cdAdd = lookup1d(WAVE_DRAG_TABLE, shockMach) * dragScale;
  out.acShift = lookup1d(AC_SHIFT_TABLE, shockMach);
  out.controlScale = lookup1d(CONTROL_TABLE, shockMach);
  out.clMaxScale = lookup1d(CL_MAX_TABLE, shockMach);
  return out;
}
