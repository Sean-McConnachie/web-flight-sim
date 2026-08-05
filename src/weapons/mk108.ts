/**
 * The four Rheinmetall-Borsig MK 108 of the Me 262 A-1a.
 *
 * The module holds the shell, the four mounts, the cyclic rate, the ammunition
 * and the recoil. It fires no round of its own: `updateBattery` reports every
 * shot to a handler, and src/weapons/armament.ts turns each one into a round
 * through src/weapons/ballistics.ts.
 *
 *
 * 1. WHY THIS GUN FEELS THE WAY IT DOES
 *
 * The MK 108 was built to be cheap, short and light. It fires a 30 mm round
 * from a stamped steel case with a small propellant charge, and it fires it
 * SLOWLY. The muzzle velocity of 505 m/s is about two thirds of what an MG 151
 * of 20 mm gives and about half of what a fast anti-tank gun gives.
 *
 * Everything that matters about the weapon follows from that one number.
 *
 *   The shell lobs. It falls 41 m over the first 1000 m. A pilot who aims at a
 *   target 600 m away with the sight on the target hits 10 m below it.
 *   The time of flight is long. 1.6 s to 600 m. A bomber at 100 m/s moves 160 m
 *   across the sight in that time, so deflection shooting needs a large lead.
 *   The useful range is short. Under 400 m the drop is 4 m, which the Revi 16B
 *   sight can hold. Past that the pilot is guessing.
 *
 * The other half of the design pays for it. The shell weighs 330 g and about a
 * quarter of that is high explosive, because the Minengeschoss case is drawn
 * thin like a cartridge case instead of machined from bar. Four hits broke up a
 * heavy bomber and one hit was normally enough for a fighter. The gun therefore
 * carries very little ammunition, 100 rounds for each of the upper pair and 80
 * for each of the lower pair, which is nine seconds of fire.
 *
 *
 * 2. THE DRAG COEFFICIENT, AND HOW IT WAS SET
 *
 * The one published ballistic fact about this weapon is the drop: 41 m over the
 * first 1000 m. SHELL_DRAG_COEFFICIENT is set so that the model reproduces
 * that number, and nothing else about it is a free choice. The value comes out
 * at 0.74, which is high and which is right for the shape: the Minengeschoss
 * has a short blunt nose, no boat tail, and a driving band far forward.
 *
 * ONE CONSTANT COVERS THE WHOLE USEFUL RANGE, and that needs a word. The shell
 * leaves at Mach 1.48, crosses Mach 1 at 407 m and is at Mach 0.83 at 600 m. A
 * real drag curve rises through that crossing and falls away below it, so 0.74
 * is an effective mean over the passage and not a value at one Mach number. The
 * mean is anchored on a measured drop at 1000 m, which is past the crossing, so
 * the error it leaves inside 600 m is small. A Mach table would add numbers
 * that no source for this shell gives.
 *
 *
 * 3. THE RECOIL IS A MEAN FORCE AND NOT A TRAIN OF IMPULSES
 *
 * One round carries away 183 N s of momentum. A gun at 650 rounds per minute
 * fires one every 92 ms, and the physics step is 4.2 ms, so a per round impulse
 * would put 44 kN into the airframe for one step out of every twenty two.
 *
 * The module applies the MEAN force instead, which is the impulse times the
 * cyclic rate, 1983 N per gun. The reasons:
 *
 *   The mean is what the airframe and the pilot feel. The gun mount, the
 *   buffer springs and the mass of the aircraft all filter the hammer blow. The
 *   pilot feels a steady push, which is what the pilot reports say.
 *   A train of one step spikes is not the same signal at 240 Hz as at 120 Hz,
 *   so the deceleration would follow the frame rate. The mean does not.
 *   The total impulse over a burst is the same either way, so the speed the
 *   aircraft loses over a burst is correct.
 *
 * Four guns give 7.9 kN. The Jumo 004 pair make about 6 kN of thrust at
 * 800 km/h, so a full burst takes MORE than the engines give at that speed and
 * the aircraft slows at 1.24 m/s2. Over a five second burst that is 22 km/h.
 * The pilot notes on the type record exactly that.
 *
 *
 * 4. WHERE THE GUNS ARE
 *
 * The stations below repeat `buildGunPorts` of src/render/models/me262.ts and
 * the cannon lumps of src/aircraft/me262/mass.ts, so the muzzle that flashes on
 * the screen is the muzzle that makes the moment. The conversion into body axes
 * is the one of src/aircraft/me262/geometry.ts.
 *
 * Both pairs sit ABOVE the center of gravity, and that is not a mistake. The
 * two engines hang 0.53 m below the fuselage reference plane and pull the
 * center of gravity 0.133 m below it, so the lower pair still stands 23 mm
 * above the center of gravity while the upper pair stands 293 mm above it. A
 * rearward force above the center of gravity makes a NOSE UP moment, so a burst
 * lifts the nose a little, and the upper pair does most of it.
 *
 *
 * 5. THE SEPARATION RULE
 *
 * CONVENTIONS section 4. This file is physics. It imports the Three.js core
 * math classes, the mass model and the wrench of src/physics/rigidbody.ts, and
 * nothing else. No renderer, no DOM.
 */

import { Vector3 } from 'three';

import { CG_HEIGHT_FROM_DATUM, CG_OFFSET_FROM_NOSE } from '@/aircraft/me262/mass';
import type { Wrench } from '@/physics/rigidbody';
import { clearWrench, createWrench } from '@/physics/rigidbody';
import type { ProjectileSpec } from '@/weapons/ballistics';
import { projectileSpec } from '@/weapons/ballistics';

// ---------------------------------------------------------------------------
// The round, 30 x 90 RB
// ---------------------------------------------------------------------------

/**
 * Caliber of the MK 108, in meters.
 * Source: Rheinmetall-Borsig designation, 30 x 90 RB. Confidence: firm.
 */
export const CALIBER = 0.030; // m

/**
 * Mass of the Minengeschoss shell, in kilograms.
 *
 * The complete round weighs about 480 g and the shell alone weighs 330 g. About
 * 85 g of that is high explosive, which is a filler fraction no machined shell
 * of the day reached.
 * Source: MK 108 ammunition tables, confidence: firm.
 */
export const SHELL_MASS = 0.330; // kg

/**
 * Muzzle velocity, in meters per second.
 *
 * Published values run from 500 to 540 m/s over the several shell types. The
 * Minengeschoss, which is the heaviest, is the slowest. 505 m/s is the value
 * for it and it is the value the drop figure of section 2 belongs to.
 * Source: MK 108 data sheet, confidence: firm.
 */
export const MUZZLE_VELOCITY = 505; // m/s

/**
 * Drag coefficient of the shell on its frontal area.
 *
 * FITTED, not published. Section 2 of the module comment gives the method: the
 * value is the one that puts the drop at 1000 m on the published 41 m. It is
 * constant over the whole useful range, which the shape allows.
 * Confidence: derived from a firm drop figure.
 */
export const SHELL_DRAG_COEFFICIENT = 0.74;

/** The shell, ready for src/weapons/ballistics.ts. */
export const MK108_SHELL: ProjectileSpec = projectileSpec(
  SHELL_MASS,
  SHELL_DRAG_COEFFICIENT,
  CALIBER,
);

// ---------------------------------------------------------------------------
// The gun
// ---------------------------------------------------------------------------

/**
 * Cyclic rate of one gun, in rounds per minute.
 * Source: MK 108 data sheet, 600 to 660 rpm. Confidence: firm.
 */
export const CYCLIC_RATE = 650; // rounds per minute

/** Rounds one gun fires in one second. */
export const ROUNDS_PER_SECOND = CYCLIC_RATE / 60;

/** Time between two rounds from one gun, in seconds. */
export const SHOT_INTERVAL = 1 / ROUNDS_PER_SECOND;

/**
 * Propellant charge of one round, in kilograms.
 *
 * ESTIMATE. The case is a short drawn steel case of 90 mm and the muzzle
 * velocity is low, so the charge is small against the shell. 26 g gives a
 * charge to shell ratio near 8 percent, which is normal for a low velocity
 * aircraft cannon. Confidence: low. It changes the recoil by 9 percent.
 */
export const PROPELLANT_MASS = 0.026; // kg

/**
 * Mean speed of the propellant gas as it leaves the muzzle, as a multiple of
 * the muzzle velocity. The standard interior ballistics approximation is 1.25
 * for a gun of this length. Source: ordnance practice, confidence: medium.
 */
export const GAS_VELOCITY_FACTOR = 1.25;

/**
 * Momentum one round takes out of the gun, in newton seconds.
 *
 * The shell carries `m v` and the gas carries about `1.25 m_charge v`. Both
 * leave forward, so the gun takes the sum rearward.
 */
export const RECOIL_IMPULSE =
  SHELL_MASS * MUZZLE_VELOCITY + PROPELLANT_MASS * GAS_VELOCITY_FACTOR * MUZZLE_VELOCITY;

/**
 * Mean recoil force of one gun while it runs, in newtons.
 * Read section 3 of the module comment before you change this to an impulse.
 */
export const RECOIL_FORCE_PER_GUN = RECOIL_IMPULSE * ROUNDS_PER_SECOND;

/** Time the muzzle flash of one round stays visible, in seconds. */
export const MUZZLE_FLASH_TIME = 0.045; // s

// ---------------------------------------------------------------------------
// The four mounts
// ---------------------------------------------------------------------------

/** One gun port, in the airframe datum frame of src/aircraft/me262/mass.ts. */
interface GunPort {
  readonly name: string;
  /** Station of the muzzle, meters aft of the nose tip. */
  readonly station: number;
  /** Meters right of the plane of symmetry. */
  readonly y: number;
  /** Meters above the fuselage reference plane. */
  readonly height: number;
  /** Rounds in the box of this gun. */
  readonly capacity: number;
}

/**
 * The four gun ports. The upper pair leaves the skin 160 mm ahead of the lower
 * pair, which is the staggered muzzle group the nose of the A-1a shows.
 *
 * The ammunition split is 100 rounds for each of the upper pair and 80 for each
 * of the lower pair, which is 360 in total and matches AMMUNITION_ROUNDS of
 * src/aircraft/me262/mass.ts. Source: A-1a loading notes, confidence: firm.
 */
const GUN_PORTS: readonly GunPort[] = [
  { name: 'upper left', station: 0.62, y: -0.085, height: 0.16, capacity: 100 },
  { name: 'upper right', station: 0.62, y: 0.085, height: 0.16, capacity: 100 },
  { name: 'lower left', station: 0.78, y: -0.09, height: -0.11, capacity: 80 },
  { name: 'lower right', station: 0.78, y: 0.09, height: -0.11, capacity: 80 },
];

/** One gun, with its mount and its state. */
export interface Gun {
  readonly index: number;
  readonly name: string;
  /** Muzzle position, body axes, meters from the center of gravity. */
  readonly position: Vector3;
  /** Rounds the box holds when it is full. */
  readonly capacity: number;
  /** Rounds left. */
  rounds: number;
  /** Seconds of the current cycle that have run. A shot leaves at SHOT_INTERVAL. */
  timer: number;
  /** True while the trigger is down and this gun still has ammunition. */
  running: boolean;
  /** Seconds since this gun last fired. The flash reads it. */
  sinceShot: number;
}

/** Direction of every bore, body axes. The guns point straight ahead. */
export const BORE_DIRECTION = new Vector3(1, 0, 0);

/** The four guns, their ammunition and the wrench they make. */
export interface Battery {
  readonly guns: readonly Gun[];
  /**
   * The recoil of every gun that runs, in BODY axes about the center of
   * gravity. src/aircraft/aircraft.ts adds it to the wrench of the step.
   */
  readonly recoil: Wrench;
  /** Rounds left over the four guns. */
  rounds: number;
  /** Rounds that left the barrels during the last step. */
  roundsFired: number;
}

/** What `updateBattery` reports for every round that leaves a barrel. */
export type ShotHandler = (gun: Gun) => void;

/** Turns a station and a height into a body axis position. CONVENTIONS 3.1. */
function bodyPosition(station: number, y: number, height: number): Vector3 {
  return new Vector3(CG_OFFSET_FROM_NOSE - station, y, CG_HEIGHT_FROM_DATUM - height);
}

/** Builds the four guns, full and cold. */
export function createBattery(): Battery {
  const guns: Gun[] = GUN_PORTS.map((port, index) => ({
    index,
    name: port.name,
    position: bodyPosition(port.station, port.y, port.height),
    capacity: port.capacity,
    rounds: port.capacity,
    timer: SHOT_INTERVAL,
    running: false,
    sinceShot: MUZZLE_FLASH_TIME,
  }));
  const battery: Battery = {
    guns,
    recoil: createWrench(),
    rounds: 0,
    roundsFired: 0,
  };
  resetBattery(battery);
  return battery;
}

/** Fills every box and stops every gun. A spawn calls it. */
export function resetBattery(battery: Battery): void {
  let total = 0;
  for (const gun of battery.guns) {
    gun.rounds = gun.capacity;
    // A gun that has not fired is ready, so the first press fires at once.
    gun.timer = SHOT_INTERVAL;
    gun.running = false;
    gun.sinceShot = MUZZLE_FLASH_TIME;
    total += gun.rounds;
  }
  battery.rounds = total;
  battery.roundsFired = 0;
  clearWrench(battery.recoil);
}

/** Scratch. updateBattery allocates nothing. */
const recoilForce = new Vector3();
const recoilMoment = new Vector3();

/**
 * Runs the four guns for one step.
 *
 * `trigger` is the held state of the fire command, not an edge. Each gun keeps
 * its own cycle, so the four are not in step with each other and the sound and
 * the flash read as four guns and not as one.
 *
 * The function writes `battery.recoil` in BODY axes about the center of
 * gravity, calls `onShot` for every round that leaves a barrel, and returns the
 * number of rounds fired.
 */
export function updateBattery(
  battery: Battery,
  trigger: boolean,
  dt: number,
  onShot?: ShotHandler,
): number {
  clearWrench(battery.recoil);
  battery.roundsFired = 0;
  if (!(dt > 0)) return 0;

  let fired = 0;
  let left = 0;

  for (const gun of battery.guns) {
    gun.sinceShot += dt;

    const running = trigger && gun.rounds > 0;
    gun.running = running;

    if (!running) {
      // A released trigger leaves the gun ready. The bolt sits back and the
      // next press fires with no delay.
      gun.timer = SHOT_INTERVAL;
      left += gun.rounds;
      continue;
    }

    // The mean recoil of section 3, along body -x, AT the gun. The cross
    // product then gives the pitch and the yaw moments with no special case.
    recoilForce.set(-RECOIL_FORCE_PER_GUN, 0, 0);
    battery.recoil.force.add(recoilForce);
    recoilMoment.crossVectors(gun.position, recoilForce);
    battery.recoil.moment.add(recoilMoment);

    gun.timer += dt;
    while (gun.timer >= SHOT_INTERVAL && gun.rounds > 0) {
      gun.timer -= SHOT_INTERVAL;
      gun.rounds -= 1;
      gun.sinceShot = 0;
      fired += 1;
      if (onShot !== undefined) onShot(gun);
    }
    left += gun.rounds;
  }

  battery.rounds = left;
  battery.roundsFired = fired;
  return fired;
}

/**
 * Brightness of the muzzle flash of one gun, from 1 at the shot down to 0.
 * The render half reads it. It holds no physics.
 */
export function muzzleFlash(gun: Gun): number {
  if (gun.sinceShot >= MUZZLE_FLASH_TIME) return 0;
  return 1 - gun.sinceShot / MUZZLE_FLASH_TIME;
}
