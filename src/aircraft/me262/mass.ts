/**
 * Mass, balance and inertia of the Messerschmitt Me 262 A-1a.
 *
 * No source publishes the inertia tensor of this aircraft, so the module builds
 * one. The method is a LUMPED MASS MODEL. Every part of the aircraft becomes a
 * point mass with its own small radius of gyration, placed at the station where
 * that part really sits. The module then sums
 *
 *   I = sum( m * (r_own^2 + d^2) )
 *
 * over every lump, with the parallel axis theorem carrying each lump from its
 * own center to the center of gravity of the whole aircraft. The center of
 * gravity comes out of the same sum, so the balance and the inertia can never
 * disagree with each other.
 *
 *
 * FRAMES
 *
 * The module works in an AIRFRAME datum frame while it sums, and reports in body
 * axes. The datum frame follows the drawing:
 *
 *   station  meters aft of the nose tip, so it grows toward the tail
 *   y        meters right of the plane of symmetry, the same as body y
 *   height   meters above the fuselage reference plane, so it grows upward
 *
 * The fuselage reference plane is the plane through the wing root quarter chord.
 * src/render/models/me262.ts builds its model about the same plane, so a station
 * in this file and a station in that file mean the same point on the aircraft.
 *
 * The map into body axes of CONVENTIONS section 3.1, with x forward and z down,
 * is then
 *
 *   body x = cgStation - station
 *   body y = y
 *   body z = cgHeight - height
 *
 *
 * WHAT THE MODEL DOES NOT DO
 *
 * The aircraft has a plane of symmetry, so Ixy and Iyz are zero by construction
 * and the module does not compute them. Ixz is NOT zero, and the module does
 * compute it. The tail surfaces sit above the reference plane and the engines
 * hang below it, so the product of inertia is real and it couples roll and yaw.
 *
 *
 * COST
 *
 * me262Mass builds a fresh Matrix3. Do not call it inside the 240 Hz step. The
 * mass of the aircraft changes at the rate the fuel burns, which is a few
 * kilograms per second, so a call every tenth of a second is far more often than
 * the model needs. Cache the result.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Matrix3 } from 'three';

// ---------------------------------------------------------------------------
// Published masses. CONVENTIONS section 8.
// ---------------------------------------------------------------------------

/** Empty equipped mass, guns included. Source: CONVENTIONS section 8, firm. */
export const EMPTY_MASS = 3795; // kg

/** Normal loaded mass, full internal fuel. Source: CONVENTIONS section 8, firm. */
export const LOADED_MASS = 6396; // kg

/** Maximum takeoff mass. Source: CONVENTIONS section 8, firm. */
export const MAX_TAKEOFF_MASS = 7130; // kg

/**
 * Usable internal fuel mass.
 *
 * Four tanks hold 2570 liters. J2 was a coal derived gas oil with a density near
 * 0.83 kg per liter, so the mass is 2133 kg. The value repeats FUEL_CAPACITY of
 * src/aircraft/me262/engine.ts on purpose, and test/unit/me262-geometry.test.ts
 * checks that the two agree. Confidence: medium.
 */
export const FUEL_CAPACITY = 2133; // kg

/**
 * Distance from the nose tip to the center of gravity of the loaded aircraft.
 *
 * DUPLICATED from CG_OFFSET_FROM_NOSE in src/render/models/me262.ts, which
 * derives it from the plan form alone as 25 percent of the mean aerodynamic
 * chord. CONVENTIONS section 4 stops the physics from importing the renderer, so
 * the value appears twice. test/unit/me262-geometry.test.ts asserts the literal
 * 5.76 so that the two files cannot drift apart in silence.
 *
 * The lumped mass model below reaches the same point on its own, to within a
 * millimeter. Read the note on the fuel tank stations for what that agreement
 * cost.
 */
export const CG_OFFSET_FROM_NOSE = 5.76; // m

/**
 * Height of the loaded center of gravity above the fuselage reference plane.
 *
 * DERIVED from the lumped mass model. The value is negative because the two
 * engines carry 1700 kg half a meter below the reference plane and pull the
 * center of gravity down with them. src/render/models/me262.ts puts its origin
 * on the reference plane, so the render origin sits 0.133 m above the true
 * center of gravity. The difference is too small to see and too small to matter
 * to the moment arms, but src/aircraft/me262/geometry.ts uses the real value.
 */
export const CG_HEIGHT_FROM_DATUM = -0.1333; // m

/** Rounds of 30 mm ammunition at a full load. Source: A-1a loading notes, firm. */
export const AMMUNITION_ROUNDS = 360;

/**
 * Mass of a full ammunition load, links included.
 *
 * The MK 108 fired the 30x90RB round. A complete round weighs about 0.48 kg, and
 * the belt links and the boxes add about ten percent. 360 rounds therefore weigh
 * about 176 kg. Confidence: derived.
 */
export const AMMUNITION_MASS = 176; // kg

// ---------------------------------------------------------------------------
// Geometry the mass model needs. The same numbers appear in
// src/render/models/me262.ts and in src/aircraft/me262/geometry.ts.
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;

const SPAN = 12.51; // m, firm
const HALF_SPAN = SPAN / 2;
const ROOT_CHORD = 2.4; // m, derived from the firm span and area
const TIP_CHORD = 1.07; // m, derived
const TAN_SWEEP = Math.tan(18.5 * DEG); // quarter chord sweep, firm
const TAN_DIHEDRAL = Math.tan(3.5 * DEG); // estimate from photographs
const DIHEDRAL_START = 2.2; // m, the inner panel is flat
const FUSELAGE_LENGTH = 10.6; // m, firm

/** Station of the wing root quarter chord, meters aft of the nose tip. */
const WING_ROOT_QUARTER_STATION = 4.85; // m

/** Local streamwise chord of the wing at a span station. */
function wingChord(y: number): number {
  return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * (y / HALF_SPAN);
}

/**
 * Fuselage half width, half height and section center height against station.
 * The table repeats FUSELAGE_STATIONS of src/render/models/me262.ts, so the
 * mass of the shell follows the shape the eye sees. Confidence: estimate, taken
 * from a three view of the A-1a.
 */
const FUSELAGE_SECTIONS: readonly (readonly [number, number, number, number])[] = [
  [0, 0.045, 0.055, 0.1],
  [0.25, 0.135, 0.165, 0.092],
  [0.6, 0.235, 0.288, 0.075],
  [1.1, 0.335, 0.422, 0.05],
  [1.75, 0.432, 0.56, 0.022],
  [2.5, 0.505, 0.68, 0],
  [3.3, 0.545, 0.755, -0.02],
  [4.2, 0.555, 0.78, -0.03],
  [5.1, 0.555, 0.78, -0.03],
  [6, 0.54, 0.765, -0.02],
  [6.9, 0.5, 0.72, 0],
  [7.8, 0.44, 0.64, 0.045],
  [8.6, 0.372, 0.552, 0.095],
  [9.4, 0.288, 0.442, 0.15],
  [10.1, 0.205, 0.34, 0.212],
  [10.35, 0.17, 0.288, 0.238],
  [10.55, 0.108, 0.196, 0.262],
  [10.6, 0.048, 0.092, 0.272],
];

/**
 * Area of the rounded triangle section, as a fraction of the box that holds it.
 * roundedTriangleProfile in src/render/models/me262.ts encloses 0.798 of its
 * bounding box. The value comes from a numerical integration of that profile.
 */
export const FUSELAGE_SECTION_FILL = 0.798;

// ---------------------------------------------------------------------------
// The lumped mass model.
// ---------------------------------------------------------------------------

/**
 * One point mass of the model.
 *
 * `gyrationX`, `gyrationY` and `gyrationZ` are the radii of gyration of the lump
 * about its OWN center, in body axes. A lump that stands for a long or a wide
 * part must carry them. Without them a 1700 kg pair of engines, each 3.9 m long,
 * would contribute nothing at all to the pitch inertia about its own axis, and
 * the whole tensor would come out low.
 */
interface Lump {
  readonly mass: number; // kg
  readonly station: number; // m aft of the nose tip
  readonly y: number; // m right of the plane of symmetry
  readonly height: number; // m above the fuselage reference plane
  readonly gyrationX: number; // m
  readonly gyrationY: number; // m
  readonly gyrationZ: number; // m
  readonly name: string;
}

function lump(
  mass: number,
  station: number,
  y: number,
  height: number,
  gyrationX: number,
  gyrationY: number,
  gyrationZ: number,
  name: string,
): Lump {
  return { mass, station, y, height, gyrationX, gyrationY, gyrationZ, name };
}

// --- The empty aircraft, 3795 kg -------------------------------------------
//
// The group masses below follow a normal weight statement for a fighter of this
// size, and they sum to the firm empty mass exactly. Every one is an ESTIMATE
// except the bare engine mass and the gun mass.
//
//   wing group          760 kg   10.7 percent of the maximum takeoff mass
//   engine installation 1700 kg  two Jumo 004 B-1 at 719 kg, plus 131 kg each
//                                of nacelle, mount, plumbing and controls
//   fuselage structure  470 kg   shell, frames, longerons and the tail cone
//   tail group          165 kg   2.3 percent of the maximum takeoff mass
//   landing gear        310 kg   4.3 percent of the maximum takeoff mass
//   armament group      245 kg   four MK 108 at 58 kg, plus mounts and feed
//   systems             145 kg   flight controls, hydraulics, electrics
//                       -----
//                       3795 kg

/** Mass of the wing group, structure and movable surfaces together. */
const WING_GROUP_MASS = 760; // kg

/** Mass of the fuselage shell and frames, without the systems inside it. */
const FUSELAGE_STRUCTURE_MASS = 470; // kg

/** Installed mass of one engine, nacelle, mount and plumbing together. */
const ENGINE_INSTALLED_MASS = 850; // kg

/**
 * Station of the installed engine center of mass.
 *
 * The nacelle runs from station 3.91 to station 7.71. A mass breakdown of the
 * Jumo 004 B-1 puts the engine center of mass near half its length, because the
 * eight stage compressor forward and the turbine with its jet pipe aft nearly
 * balance. The installed unit sits a little further aft, because the rear of the
 * nacelle carries the jet pipe, the movable exhaust bullet and its actuator.
 * ESTIMATE, confidence: low. The value is the single largest lever on the
 * balance, so read the note on the center of gravity below.
 */
const ENGINE_STATION = 6.15; // m

/** Spanwise station of the engine center line. Estimate from photographs. */
const ENGINE_SPAN_STATION = 2.05; // m

/** Height of the engine center line. Estimate from photographs. */
const ENGINE_HEIGHT = -0.53; // m

/**
 * Fuel tank stations, in meters aft of the nose tip.
 *
 * The Me 262 A-1a carried 900 liters ahead of the cockpit, 900 liters behind it,
 * a 600 liter auxiliary tank in the rear fuselage, and a 170 liter tank under
 * the cockpit floor. No drawing in the reference set gives the station of any
 * one of them.
 *
 * THE STATIONS BELOW SIT AT THE AFT EDGE OF THE SPACE THE FUSELAGE OFFERS. That
 * choice is deliberate and it is the weakest number in this file. A layout with
 * the tanks half a meter further forward puts the loaded center of gravity near
 * 5.55 m, which is 11 percent of the mean aerodynamic chord ahead of the quarter
 * chord point and gives a static margin near 26 percent. The aircraft must
 * balance where the plan form says it balances, so the model holds the tanks
 * aft. The report of bead b17 records the tension.
 *
 * The wartime handling notes support the aft choice from the other side. Pilots
 * were told to burn the rear auxiliary tank first, because the aircraft was tail
 * heavy while that tank was full. burnOrder below follows that instruction.
 */
interface Tank {
  readonly capacity: number; // kg
  readonly station: number; // m
  readonly height: number; // m
  readonly gyrationX: number;
  readonly gyrationY: number;
  readonly gyrationZ: number;
  /** 0 empties first, then 1, then 2. The pilot burns the rear tank first. */
  readonly burnOrder: number;
  readonly name: string;
}

const TANKS: readonly Tank[] = [
  {
    capacity: 747, // 900 L
    station: 3.9,
    height: -0.12,
    gyrationX: 0.42,
    gyrationY: 0.6,
    gyrationZ: 0.42,
    burnOrder: 1,
    name: 'forward main tank',
  },
  {
    capacity: 747, // 900 L
    station: 6.5,
    height: -0.05,
    gyrationX: 0.4,
    gyrationY: 0.55,
    gyrationZ: 0.4,
    burnOrder: 1,
    name: 'rear main tank',
  },
  {
    capacity: 498, // 600 L
    station: 7.8,
    height: 0.1,
    gyrationX: 0.32,
    gyrationY: 0.55,
    gyrationZ: 0.32,
    burnOrder: 0,
    name: 'rear auxiliary tank',
  },
  {
    capacity: 141, // 170 L
    station: 4.6,
    height: -0.3,
    gyrationX: 0.25,
    gyrationY: 0.35,
    gyrationZ: 0.25,
    burnOrder: 2,
    name: 'cockpit floor tank',
  },
];

/** Number of spanwise pieces the wing structure breaks into, on each side. */
const WING_LUMP_COUNT = 60;

/**
 * Builds the wing structure lumps of one side.
 *
 * The mass of a wing per unit span follows the area of its structural box, which
 * is the local chord times the local thickness, times the chord again for the
 * skin that carries it. The weight therefore falls as chord squared times the
 * thickness ratio. The structural center of mass of a section with a front spar,
 * a rear spar, a skin and a movable surface sits near 46 percent of the chord.
 * ESTIMATE, confidence: medium.
 */
function wingLumps(side: number, out: Lump[]): void {
  let total = 0;
  const density: number[] = new Array<number>(WING_LUMP_COUNT);
  for (let i = 0; i < WING_LUMP_COUNT; i++) {
    const y = (HALF_SPAN * (i + 0.5)) / WING_LUMP_COUNT;
    const chord = wingChord(y);
    const thickness = 0.11 + (0.09 - 0.11) * (y / HALF_SPAN);
    density[i] = chord * chord * thickness;
    total += density[i];
  }
  for (let i = 0; i < WING_LUMP_COUNT; i++) {
    const y = (HALF_SPAN * (i + 0.5)) / WING_LUMP_COUNT;
    const chord = wingChord(y);
    const mass = (0.5 * WING_GROUP_MASS * density[i]) / total;
    const leadingEdge = WING_ROOT_QUARTER_STATION + TAN_SWEEP * y - 0.25 * chord;
    const station = leadingEdge + 0.46 * chord;
    const height = TAN_DIHEDRAL * Math.max(0, y - DIHEDRAL_START);
    // The piece is thin across the span, so its own roll gyration is small. Its
    // own pitch and yaw gyration follow the chord it covers.
    out.push(
      lump(mass, station, side * y, height, 0.05, 0.3 * chord, 0.3 * chord, 'wing'),
    );
  }
}

/**
 * Weight of the fuselage shell along its length.
 *
 * The skin and the frames follow the local perimeter. The aft bias adds the
 * carry through of the fin and of the tailplane, the frames that take the tail
 * loads, and the rear tank bay. ESTIMATE, confidence: low.
 */
const FUSELAGE_AFT_BIAS = 0.9;

function fuselageLumps(out: Lump[]): void {
  const count = FUSELAGE_SECTIONS.length - 1;
  const density: number[] = new Array<number>(count);
  let total = 0;
  for (let i = 0; i < count; i++) {
    const a = FUSELAGE_SECTIONS[i];
    const b = FUSELAGE_SECTIONS[i + 1];
    const length = b[0] - a[0];
    const station = 0.5 * (a[0] + b[0]);
    const perimeter = Math.PI * (0.5 * (a[1] + b[1]) + 0.5 * (a[2] + b[2]));
    density[i] = perimeter * length * (1 + (FUSELAGE_AFT_BIAS * station) / FUSELAGE_LENGTH);
    total += density[i];
  }
  for (let i = 0; i < count; i++) {
    const a = FUSELAGE_SECTIONS[i];
    const b = FUSELAGE_SECTIONS[i + 1];
    const length = b[0] - a[0];
    const station = 0.5 * (a[0] + b[0]);
    const halfWidth = 0.5 * (a[1] + b[1]);
    const halfHeight = 0.5 * (a[2] + b[2]);
    const centerHeight = 0.5 * (a[3] + b[3]);
    // A shell carries its mass on the wall, so its roll gyration is close to the
    // mean of the two half dimensions. The piece is short along x, so the piece
    // gyration in pitch and in yaw comes mostly from the section.
    const shell = Math.hypot(halfWidth, halfHeight) / Math.SQRT2;
    out.push(
      lump(
        (FUSELAGE_STRUCTURE_MASS * density[i]) / total,
        station,
        0,
        centerHeight,
        shell,
        Math.hypot(length / 3.5, halfHeight),
        Math.hypot(length / 3.5, halfWidth),
        'fuselage',
      ),
    );
  }
}

/** Builds every lump of the empty aircraft. The list is fixed. */
function buildEmptyLumps(): Lump[] {
  const out: Lump[] = [];
  wingLumps(1, out);
  wingLumps(-1, out);
  fuselageLumps(out);

  // Engines. The gyration values treat each unit as a 3.9 m tube of 0.85 m
  // diameter, so it holds a real pitch and yaw inertia of its own.
  for (const side of [-1, 1]) {
    out.push(
      lump(
        ENGINE_INSTALLED_MASS,
        ENGINE_STATION,
        side * ENGINE_SPAN_STATION,
        ENGINE_HEIGHT,
        0.3,
        1.05,
        1.05,
        'engine',
      ),
    );
  }

  // Tail group, 165 kg. The tailplane sits at station 9.35 and the fin center of
  // mass sits 1.17 m above the reference plane. Both stations follow the render
  // model geometry with the structural center of mass at 45 percent of chord.
  for (const side of [-1, 1]) {
    out.push(lump(50, 9.35, side * 0.85, 0.7, 0.5, 0.3, 0.55, 'horizontal tail'));
  }
  out.push(lump(65, 8.9, 0, 1.17, 0.5, 0.35, 0.5, 'vertical tail'));

  // Landing gear, 310 kg, at the retracted position. The aircraft flies with the
  // gear up, and the shift to the extended position moves the center of gravity
  // by less than 20 mm.
  out.push(lump(95, 2.0, 0, -0.3, 0.2, 0.35, 0.2, 'nose gear'));
  for (const side of [-1, 1]) {
    out.push(lump(107.5, 6.45, side * 0.75, -0.3, 0.25, 0.35, 0.25, 'main gear'));
  }

  // Armament group, 245 kg. Four MK 108 of 30 mm, plus the mounts and the feed.
  // The upper pair sits ahead of and above the lower pair, which is the
  // staggered muzzle group the nose of the A-1a shows.
  for (const side of [-1, 1]) {
    out.push(lump(61.25, 1.75, side * 0.085, 0.16, 0.06, 0.32, 0.32, 'cannon'));
    out.push(lump(61.25, 1.9, side * 0.09, -0.11, 0.06, 0.32, 0.32, 'cannon'));
  }

  // Systems, 145 kg. The control rods, the bellcranks, the trim motor, the
  // hydraulic lines and the electrical loom run from the cockpit to the tail, so
  // the group carries a long pitch gyration and a center of mass well aft.
  out.push(lump(145, 7.35, 0, 0.0, 0.35, 2.0, 0.6, 'systems'));

  return out;
}

const EMPTY_LUMPS: readonly Lump[] = buildEmptyLumps();

// --- The useful load, 2601 kg ----------------------------------------------

/** Pilot, seat and parachute. ESTIMATE, confidence: medium. */
const PILOT_LUMP = lump(100, 4.3, 0, 0.15, 0.28, 0.3, 0.28, 'pilot');

/**
 * Oil, radio, oxygen and the removable equipment.
 *
 * The item closes the published loaded mass. The empty mass, the pilot, a full
 * ammunition load and full internal fuel add up to 6204 kg, and the published
 * loaded mass is 6396 kg. The 192 kg that are left cover the engine oil, the
 * FuG 16ZY and FuG 25a radio sets, the oxygen bottles, the gun camera and the
 * ammunition boxes. DERIVED, confidence: low on the split, firm on the total.
 */
const EQUIPMENT_LUMP = lump(192, 7.2, 0, 0.2, 0.25, 0.6, 0.3, 'equipment');

/** Ammunition, at a full load. The boxes sit behind the gun bay. */
const AMMUNITION_STATION = 2.55; // m

/** Sum of every tank capacity. It must match FUEL_CAPACITY. */
const TANK_CAPACITY = TANKS.reduce((sum, tank) => sum + tank.capacity, 0);

/**
 * Writes the fuel mass of every tank for a given total, in the order the fuel
 * system feeds them.
 *
 * The tanks with the lowest burnOrder empty first. The rear auxiliary tank
 * therefore goes first, which is what the pilot notes tell the pilot to do,
 * because the aircraft is tail heavy while that tank is full.
 */
function fillTanks(fuelMass: number, out: number[]): void {
  let left = Math.max(0, Math.min(fuelMass, TANK_CAPACITY));
  for (let i = 0; i < TANKS.length; i++) {
    out[i] = 0;
  }
  // Fill against the burn order, so the last tank to empty is the first to fill.
  for (let order = 2; order >= 0; order--) {
    let groupCapacity = 0;
    for (const tank of TANKS) {
      if (tank.burnOrder === order) {
        groupCapacity += tank.capacity;
      }
    }
    if (groupCapacity <= 0) {
      continue;
    }
    const take = Math.min(left, groupCapacity);
    for (let i = 0; i < TANKS.length; i++) {
      if (TANKS[i].burnOrder === order) {
        out[i] = (take * TANKS[i].capacity) / groupCapacity;
      }
    }
    left -= take;
  }
}

// ---------------------------------------------------------------------------
// The public interface.
// ---------------------------------------------------------------------------

export interface MassState {
  /** Total mass, kg. */
  mass: number;
  /** Distance from the nose tip to the center of gravity, m. */
  cgFromNose: number;
  /** Inertia tensor in body axes about the center of gravity, kg m^2. */
  inertia: Matrix3;
  /** The fuel on board, kg. */
  fuelMass: number;
}

/** Scratch for fillTanks. me262Mass runs on one thread. */
const tankFill: number[] = new Array<number>(TANKS.length).fill(0);

/**
 * Returns the mass, the balance and the inertia tensor of the aircraft.
 *
 * `fuelMass` is the fuel on board in kilograms, clamped to FUEL_CAPACITY.
 * `ammunitionRounds` defaults to a full load of 360 rounds.
 *
 * The center of gravity and the inertia both follow the fuel, so a long flight
 * changes both. The tensor is symmetric and positive definite, and it satisfies
 * the triangle inequalities, so createMassProperties of
 * src/physics/rigidbody.ts accepts it at any fuel state.
 */
export function me262Mass(fuelMass: number, ammunitionRounds: number = AMMUNITION_ROUNDS): MassState {
  const fuel = Math.max(0, Math.min(fuelMass, FUEL_CAPACITY));
  const rounds = Math.max(0, Math.min(ammunitionRounds, AMMUNITION_ROUNDS));
  fillTanks(fuel, tankFill);

  let mass = 0;
  let momentStation = 0;
  let momentHeight = 0;

  const accumulate = (m: number, station: number, height: number): void => {
    mass += m;
    momentStation += m * station;
    momentHeight += m * height;
  };

  for (const item of EMPTY_LUMPS) {
    accumulate(item.mass, item.station, item.height);
  }
  accumulate(PILOT_LUMP.mass, PILOT_LUMP.station, PILOT_LUMP.height);
  accumulate(EQUIPMENT_LUMP.mass, EQUIPMENT_LUMP.station, EQUIPMENT_LUMP.height);
  const ammunitionMass = (AMMUNITION_MASS * rounds) / AMMUNITION_ROUNDS;
  accumulate(ammunitionMass, AMMUNITION_STATION, 0.05);
  for (let i = 0; i < TANKS.length; i++) {
    accumulate(tankFill[i], TANKS[i].station, TANKS[i].height);
  }

  const cgStation = momentStation / mass;
  const cgHeight = momentHeight / mass;

  let ixx = 0;
  let iyy = 0;
  let izz = 0;
  let ixz = 0;

  const addInertia = (
    m: number,
    station: number,
    y: number,
    height: number,
    gx: number,
    gy: number,
    gz: number,
  ): void => {
    // Body axes: x forward, y right, z down.
    const x = cgStation - station;
    const z = cgHeight - height;
    ixx += m * (y * y + z * z + gx * gx);
    iyy += m * (x * x + z * z + gy * gy);
    izz += m * (x * x + y * y + gz * gz);
    ixz += m * x * z;
  };

  for (const item of EMPTY_LUMPS) {
    addInertia(
      item.mass,
      item.station,
      item.y,
      item.height,
      item.gyrationX,
      item.gyrationY,
      item.gyrationZ,
    );
  }
  addInertia(
    PILOT_LUMP.mass,
    PILOT_LUMP.station,
    0,
    PILOT_LUMP.height,
    PILOT_LUMP.gyrationX,
    PILOT_LUMP.gyrationY,
    PILOT_LUMP.gyrationZ,
  );
  addInertia(
    EQUIPMENT_LUMP.mass,
    EQUIPMENT_LUMP.station,
    0,
    EQUIPMENT_LUMP.height,
    EQUIPMENT_LUMP.gyrationX,
    EQUIPMENT_LUMP.gyrationY,
    EQUIPMENT_LUMP.gyrationZ,
  );
  addInertia(ammunitionMass, AMMUNITION_STATION, 0, 0.05, 0.2, 0.3, 0.25);
  for (let i = 0; i < TANKS.length; i++) {
    const tank = TANKS[i];
    addInertia(
      tankFill[i],
      tank.station,
      0,
      tank.height,
      tank.gyrationX,
      tank.gyrationY,
      tank.gyrationZ,
    );
  }

  // CONVENTIONS section 6 and src/physics/rigidbody.ts read this matrix as the
  // inertia tensor itself, so the off diagonal terms carry the minus sign of the
  // products of inertia. The plane of symmetry makes Ixy and Iyz zero.
  const inertia = new Matrix3().set(ixx, 0, -ixz, 0, iyy, 0, -ixz, 0, izz);

  return { mass, cgFromNose: cgStation, inertia, fuelMass: fuel };
}

/**
 * Returns the height of the center of gravity above the fuselage reference
 * plane at a given fuel state. The value moves by about 40 mm over a full burn,
 * because the fuel sits close to the reference plane and the engines do not.
 */
export function me262CgHeight(fuelMass: number, ammunitionRounds: number = AMMUNITION_ROUNDS): number {
  const fuel = Math.max(0, Math.min(fuelMass, FUEL_CAPACITY));
  const rounds = Math.max(0, Math.min(ammunitionRounds, AMMUNITION_ROUNDS));
  fillTanks(fuel, tankFill);
  let mass = 0;
  let moment = 0;
  for (const item of EMPTY_LUMPS) {
    mass += item.mass;
    moment += item.mass * item.height;
  }
  mass += PILOT_LUMP.mass;
  moment += PILOT_LUMP.mass * PILOT_LUMP.height;
  mass += EQUIPMENT_LUMP.mass;
  moment += EQUIPMENT_LUMP.mass * EQUIPMENT_LUMP.height;
  const ammunitionMass = (AMMUNITION_MASS * rounds) / AMMUNITION_ROUNDS;
  mass += ammunitionMass;
  moment += ammunitionMass * 0.05;
  for (let i = 0; i < TANKS.length; i++) {
    mass += tankFill[i];
    moment += tankFill[i] * TANKS[i].height;
  }
  return moment / mass;
}

/**
 * Returns the volume, the side area and the frontal area of the fuselage shell,
 * and the station of the side area centroid.
 *
 * src/aircraft/me262/geometry.ts needs all four for the fuselage BodyDef. The
 * numbers come from the same section table the mass model uses, so the shape the
 * aerodynamics sees and the shape the mass model weighs are one shape.
 */
export function fuselageShape(): {
  volume: number;
  sideArea: number;
  frontalArea: number;
  sideAreaStation: number;
  maxDiameter: number;
} {
  let volume = 0;
  let sideArea = 0;
  let sideMoment = 0;
  let maxSection = 0;
  for (let i = 0; i < FUSELAGE_SECTIONS.length - 1; i++) {
    const a = FUSELAGE_SECTIONS[i];
    const b = FUSELAGE_SECTIONS[i + 1];
    const length = b[0] - a[0];
    const areaA = FUSELAGE_SECTION_FILL * 4 * a[1] * a[2];
    const areaB = FUSELAGE_SECTION_FILL * 4 * b[1] * b[2];
    volume += 0.5 * (areaA + areaB) * length;
    const side = 0.5 * (2 * a[2] + 2 * b[2]) * length;
    sideArea += side;
    sideMoment += side * 0.5 * (a[0] + b[0]);
    maxSection = Math.max(maxSection, areaA, areaB);
  }
  return {
    volume,
    sideArea,
    frontalArea: maxSection,
    sideAreaStation: sideMoment / sideArea,
    maxDiameter: Math.sqrt((4 * maxSection) / Math.PI),
  };
}
