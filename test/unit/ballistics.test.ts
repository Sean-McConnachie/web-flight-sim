/**
 * Tests of the shell flight of src/weapons/ballistics.ts.
 *
 * The tests run in Node with no GPU. Every number the integrator produces is
 * checked against the CLOSED FORM of section 2 of the module comment, and the
 * closed form itself is checked against numbers worked out by hand, so a change
 * to both at once cannot pass in silence.
 *
 *
 * THE HAND CALCULATION
 *
 * Shell mass 0.330 kg, caliber 0.030 m, so the frontal area is
 * 0.25 * pi * 0.030^2 = 7.0686e-4 m2. With a drag coefficient of 0.74 the
 * ballistic coefficient is
 *
 *   BC = 0.330 / (0.74 * 7.0686e-4) = 630.88 kg/m2
 *   k  = 1.225 / (2 * 630.88)       = 9.7086e-4 per meter
 *   k v0 = 9.7086e-4 * 505          = 0.49029 per second
 *
 * Time of flight, t(x) = (exp(k x) - 1) / (k v0):
 *
 *   200 m   (exp(0.19417) - 1) / 0.49029 = 0.2143 / 0.49029 = 0.4371 s
 *   400 m   (exp(0.38834) - 1) / 0.49029 = 0.4745 / 0.49029 = 0.9679 s
 *   600 m   (exp(0.58252) - 1) / 0.49029 = 0.7906 / 0.49029 = 1.6124 s
 *
 * Drop, drop(x) = (g / (2 k v0^2)) * ((exp(2 k x) - 1) / (2 k) - x), with
 * g / (2 k v0^2) = 9.80665 / (2 * 9.7086e-4 * 505^2) = 0.019804:
 *
 *   200 m   0.019804 * (244.38 - 200)   = 0.879 m
 *   400 m   0.019804 * (604.72 - 400)   = 4.055 m
 *   600 m   0.019804 * (1135.94 - 600)  = 10.617 m
 *
 * The published check point is the drop at 1000 m, which every source gives as
 * 41 m. The same equation gives 41.09 m, and that is where the drag coefficient
 * of 0.74 came from.
 */

import { Quaternion, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';

import { G0 } from '@/math/units';
import { RHO0, isa } from '@/physics/atmosphere';
import type { Projectile } from '@/weapons/ballistics';
import {
  createProjectile,
  dragConstant,
  flatFireDrop,
  flatFireSpeed,
  flatFireTime,
  launchProjectile,
  projectileSpec,
  stepProjectile,
} from '@/weapons/ballistics';
import { CALIBER, MK108_SHELL, MUZZLE_VELOCITY, SHELL_MASS } from '@/weapons/mk108';
import type { Target } from '@/weapons/targets';
import {
  TARGET_KINDS,
  applyHit,
  placeTargets,
  segmentHitsTarget,
  targetFootprintRadius,
} from '@/weapons/targets';
import { insideAirfieldBoundary, isClearOfAirfield } from '@/world/scatter';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

const NORTH = new Vector3(1, 0, 0);
const AT_REST = new Vector3(0, 0, 0);
const NO_SPIN = new Vector3(0, 0, 0);
const LEVEL = new Quaternion();

/** Fires one round due north from a gun that stands still at `altitude`. */
function fire(altitude: number, aircraftVelocity: Vector3 = AT_REST): Projectile {
  const round = createProjectile();
  launchProjectile(
    round,
    new Vector3(0, 0, -altitude),
    LEVEL,
    aircraftVelocity,
    NO_SPIN,
    AT_REST,
    NORTH,
    MUZZLE_VELOCITY,
  );
  return round;
}

/** Time of flight and drop at one range, from the integrator. */
function flyToRange(range: number, dt = 1 / 2000): { time: number; drop: number } {
  const round = fire(0);
  let lastNorth = 0;
  let lastDown = 0;
  let lastTime = 0;
  while (round.position.x < range && round.age < 10) {
    lastNorth = round.position.x;
    lastDown = round.position.z;
    lastTime = round.age;
    stepProjectile(round, MK108_SHELL, dt);
  }
  // Cut the last step at the exact range, so the answer does not depend on dt.
  const span = round.position.x - lastNorth;
  const fraction = span > 0 ? (range - lastNorth) / span : 0;
  return {
    time: lastTime + fraction * (round.age - lastTime),
    drop: lastDown + fraction * (round.position.z - lastDown),
  };
}

// ---------------------------------------------------------------------------

describe('the MK 108 shell', () => {
  it('has a ballistic coefficient of 631 kg/m2 from its mass, caliber and drag', () => {
    const area = 0.25 * Math.PI * CALIBER * CALIBER;
    expect(area).toBeCloseTo(7.0686e-4, 8);
    expect(MK108_SHELL.referenceArea).toBeCloseTo(area, 12);
    expect(MK108_SHELL.ballisticCoefficient).toBeCloseTo(630.88, 1);
    expect(dragConstant(MK108_SHELL, RHO0)).toBeCloseTo(9.7086e-4, 8);
  });

  it('falls 41 m over the first 1000 m, which is the published figure', () => {
    // This is the one measurement every source gives for this weapon, and the
    // drag coefficient of the shell was fitted to it.
    expect(flatFireDrop(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 1000)).toBeCloseTo(41.09, 1);
  });

  it('leaves at Mach 1.48 and falls through Mach 1 at about 400 m', () => {
    const sea = isa(0);
    expect(MUZZLE_VELOCITY / sea.speedOfSound).toBeCloseTo(1.484, 3);
    expect(flatFireSpeed(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 400)).toBeGreaterThan(
      sea.speedOfSound,
    );
    expect(flatFireSpeed(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 420)).toBeLessThan(
      sea.speedOfSound,
    );
    // At 600 m the shell is down to Mach 0.83, which is why a pilot had to be
    // close for the shell to arrive with any speed at all.
    expect(flatFireSpeed(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 600) / sea.speedOfSound).toBeCloseTo(
      0.829,
      3,
    );
  });
});

describe('the closed form of a flat fire trajectory', () => {
  it('matches the hand calculation of the time of flight at 200, 400 and 600 m', () => {
    expect(flatFireTime(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 200)).toBeCloseTo(0.4371, 3);
    expect(flatFireTime(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 400)).toBeCloseTo(0.9679, 3);
    expect(flatFireTime(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 600)).toBeCloseTo(1.6124, 3);
  });

  it('matches the hand calculation of the drop at 200, 400 and 600 m', () => {
    expect(flatFireDrop(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 200)).toBeCloseTo(0.879, 2);
    expect(flatFireDrop(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 400)).toBeCloseTo(4.055, 2);
    expect(flatFireDrop(MK108_SHELL, RHO0, MUZZLE_VELOCITY, 600)).toBeCloseTo(10.617, 2);
  });

  it('stays under free fall over the same time of flight, because drag acts down as well', () => {
    for (const range of [200, 400, 600]) {
      const time = flatFireTime(MK108_SHELL, RHO0, MUZZLE_VELOCITY, range);
      const freeFall = 0.5 * G0 * time * time;
      const drop = flatFireDrop(MK108_SHELL, RHO0, MUZZLE_VELOCITY, range);
      expect(drop).toBeLessThan(freeFall);
      // The vertical drag never takes off more than a fifth of the drop over
      // the useful range of this weapon.
      expect(drop).toBeGreaterThan(0.8 * freeFall);
    }
  });

  it('reduces to free fall over a flat trajectory when the air is empty', () => {
    const time = 1000 / MUZZLE_VELOCITY;
    expect(flatFireDrop(MK108_SHELL, 0, MUZZLE_VELOCITY, 1000)).toBeCloseTo(
      0.5 * G0 * time * time,
      6,
    );
    expect(flatFireTime(MK108_SHELL, 0, MUZZLE_VELOCITY, 1000)).toBeCloseTo(time, 9);
  });
});

describe('the integrator', () => {
  it('reaches 200, 400 and 600 m at the time the closed form gives', () => {
    // The density falls by 0.1 percent over the 10 m the shell drops, and the
    // closed form drops a term of order (vz/vx)^2, so a tenth of a percent is
    // the right band.
    for (const range of [200, 400, 600]) {
      const flown = flyToRange(range);
      const closed = flatFireTime(MK108_SHELL, RHO0, MUZZLE_VELOCITY, range);
      expect(flown.time).toBeCloseTo(closed, 3);
      expect(Math.abs(flown.time / closed - 1)).toBeLessThan(0.002);
    }
  });

  it('drops 0.88 m at 200 m, 4.05 m at 400 m and 10.6 m at 600 m', () => {
    const expected = [0.879, 4.055, 10.617];
    const ranges = [200, 400, 600];
    for (let i = 0; i < ranges.length; i++) {
      const flown = flyToRange(ranges[i]);
      expect(flown.drop).toBeCloseTo(expected[i], 2);
      expect(
        Math.abs(flown.drop / flatFireDrop(MK108_SHELL, RHO0, MUZZLE_VELOCITY, ranges[i]) - 1),
      ).toBeLessThan(0.01);
    }
  });

  it('gives the same answer at 240 Hz as at 2000 Hz', () => {
    // The loop hands the weapon a step of 1/240 s. RK4 must give the same
    // trajectory there as at eight times the rate.
    const coarse = flyToRange(600, 1 / 240);
    const fine = flyToRange(600, 1 / 2000);
    expect(coarse.drop).toBeCloseTo(fine.drop, 4);
    expect(coarse.time).toBeCloseTo(fine.time, 5);
  });

  it('falls at exactly g when the air is empty', () => {
    const noDrag = projectileSpec(SHELL_MASS, 0, CALIBER);
    const round = fire(0);
    for (let i = 0; i < 2000; i++) stepProjectile(round, noDrag, 1 / 2000);
    expect(round.age).toBeCloseTo(1, 9);
    expect(round.position.z).toBeCloseTo(0.5 * G0, 6);
    expect(round.position.x).toBeCloseTo(MUZZLE_VELOCITY, 6);
  });
});

describe('the muzzle', () => {
  it('adds the velocity of the aircraft to the muzzle velocity', () => {
    // A shell fired forward from an aircraft at 900 km/h leaves at 250 m/s plus
    // 505 m/s over the ground.
    const aircraft = new Vector3(250, 0, 0);
    const round = fire(0, aircraft);
    expect(round.velocity.x).toBeCloseTo(250 + MUZZLE_VELOCITY, 9);
    expect(round.velocity.y).toBeCloseTo(0, 9);
    expect(round.velocity.z).toBeCloseTo(0, 9);
  });

  it('carries the sideways velocity of the aircraft into the round', () => {
    const round = fire(0, new Vector3(0, 60, -12));
    expect(round.velocity.x).toBeCloseTo(MUZZLE_VELOCITY, 9);
    expect(round.velocity.y).toBeCloseTo(60, 9);
    expect(round.velocity.z).toBeCloseTo(-12, 9);
  });

  it('turns the bore with the aircraft', () => {
    // A 90 degree right turn about the world down axis points the nose east.
    const east = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2);
    const round = createProjectile();
    launchProjectile(
      round,
      new Vector3(0, 0, -100),
      east,
      AT_REST,
      NO_SPIN,
      new Vector3(5, 0, 0),
      NORTH,
      MUZZLE_VELOCITY,
    );
    expect(round.velocity.x).toBeCloseTo(0, 6);
    expect(round.velocity.y).toBeCloseTo(MUZZLE_VELOCITY, 6);
    // The muzzle sits 5 m ahead of the center of gravity, which is 5 m east.
    expect(round.position.y).toBeCloseTo(5, 6);
    expect(round.position.x).toBeCloseTo(0, 6);
  });

  it('swings with the pitch rate of the aircraft', () => {
    // A positive pitch rate raises the nose, so a muzzle 5 m ahead of the
    // center of gravity moves UP, which is a negative world z speed.
    const round = createProjectile();
    launchProjectile(
      round,
      new Vector3(0, 0, -100),
      LEVEL,
      AT_REST,
      new Vector3(0, 1, 0),
      new Vector3(5, 0, 0),
      NORTH,
      MUZZLE_VELOCITY,
    );
    expect(round.velocity.z).toBeCloseTo(-5, 6);
  });
});

describe('the atmosphere', () => {
  it('lets a round fired at 6000 m fly further than the same round at sea level', () => {
    // The density at 6000 m is 0.660 kg/m3 against 1.225 at sea level, so the
    // drag is 54 percent of the sea level value and the shell holds its speed.
    const high = fire(6000);
    const low = fire(0);
    for (let i = 0; i < 1200; i++) {
      stepProjectile(high, MK108_SHELL, 1 / 1000);
      stepProjectile(low, MK108_SHELL, 1 / 1000);
    }
    expect(high.position.x).toBeGreaterThan(low.position.x);
    // Over 1.2 s the difference is about 50 m, so it is not a rounding effect.
    // The high round also keeps more speed.
    expect(high.position.x - low.position.x).toBeGreaterThan(40);
    expect(high.velocity.length()).toBeGreaterThan(low.velocity.length());
  });

  it('reads the density at the height of the shell and not at the muzzle', () => {
    const air = isa(6000);
    expect(air.density).toBeCloseTo(0.6597, 4);
    // The closed form at the density of 6000 m must match the integrator there.
    const round = fire(6000);
    let lastNorth = 0;
    let lastTime = 0;
    while (round.position.x < 600) {
      lastNorth = round.position.x;
      lastTime = round.age;
      stepProjectile(round, MK108_SHELL, 1 / 2000);
    }
    const fraction = (600 - lastNorth) / (round.position.x - lastNorth);
    const time = lastTime + fraction * (round.age - lastTime);
    expect(time).toBeCloseTo(flatFireTime(MK108_SHELL, air.density, MUZZLE_VELOCITY, 600), 2);
  });
});

// ---------------------------------------------------------------------------
// Targets
// ---------------------------------------------------------------------------

/** One truck, standing north of the origin and facing north. */
function testTarget(): Target {
  const spec = TARGET_KINDS.truck;
  return {
    kind: 'truck',
    position: new Vector3(500, 0, -spec.halfExtent.z),
    heading: 0,
    halfExtent: spec.halfExtent.clone(),
    maxHealth: spec.health,
    health: spec.health,
    destroyed: false,
    hits: 0,
  };
}

describe('the hit test', () => {
  it('registers a hit for a round that passes through the target volume', () => {
    const target = testTarget();
    const point = new Vector3();
    const hit = segmentHitsTarget(
      target,
      new Vector3(490, 0, -1.3),
      new Vector3(510, 0, -1.3),
      point,
    );
    expect(hit).toBe(true);
    // The box reaches from 497 m to 503 m, so the round enters at 497 m.
    expect(point.x).toBeCloseTo(497, 6);
  });

  it('registers no hit for a round that passes beside the target', () => {
    const target = testTarget();
    expect(
      segmentHitsTarget(target, new Vector3(490, 4, -1.3), new Vector3(510, 4, -1.3)),
    ).toBe(false);
  });

  it('registers no hit for a round that passes over the target', () => {
    const target = testTarget();
    expect(
      segmentHitsTarget(target, new Vector3(490, 0, -3.5), new Vector3(510, 0, -3.5)),
    ).toBe(false);
  });

  it('registers no hit for a segment that stops short of the target', () => {
    const target = testTarget();
    expect(
      segmentHitsTarget(target, new Vector3(480, 0, -1.3), new Vector3(495, 0, -1.3)),
    ).toBe(false);
  });

  it('catches a round that would cross the whole target inside one step', () => {
    // A round at 755 m/s covers 3.1 m in one step at 240 Hz, and a fuel drum is
    // 2.4 m across. A test of the end point alone would miss it.
    const spec = TARGET_KINDS['fuel-drums'];
    const drum: Target = {
      kind: 'fuel-drums',
      position: new Vector3(300, 0, -spec.halfExtent.z),
      heading: 0,
      halfExtent: spec.halfExtent.clone(),
      maxHealth: spec.health,
      health: spec.health,
      destroyed: false,
      hits: 0,
    };
    const from = new Vector3(298.5, 0, -0.9);
    const to = new Vector3(301.6, 0, -0.9);
    expect(segmentHitsTarget(drum, from, to)).toBe(true);
  });

  it('turns the box with the heading of the target', () => {
    // The truck is 6 m long and 2.4 m wide, so its box reaches 3 m along its
    // heading and 1.2 m across it. A segment that runs north at 2 m east of the
    // center therefore hits the truck that faces EAST and misses the truck that
    // faces north, and only the heading tells the two apart.
    const facingNorth = testTarget();
    const facingEast: Target = { ...facingNorth, heading: Math.PI / 2 };
    const west = new Vector3(490, 2, -1.3);
    const east = new Vector3(510, 2, -1.3);
    expect(segmentHitsTarget(facingEast, west, east)).toBe(true);
    expect(segmentHitsTarget(facingNorth, west, east)).toBe(false);
    // At 4 m east both miss, because 4 m is outside the 3 m half length.
    expect(
      segmentHitsTarget(facingEast, new Vector3(490, 4, -1.3), new Vector3(510, 4, -1.3)),
    ).toBe(false);
    // At 1 m east both hit, because 1 m is inside the 1.2 m half width.
    expect(
      segmentHitsTarget(facingNorth, new Vector3(490, 1, -1.3), new Vector3(510, 1, -1.3)),
    ).toBe(true);
  });
});

describe('target health', () => {
  it('destroys a fuel drum with one hit and a truck with two', () => {
    const truck = testTarget();
    expect(applyHit(truck)).toBe(false);
    expect(truck.destroyed).toBe(false);
    expect(applyHit(truck)).toBe(true);
    expect(truck.destroyed).toBe(true);
    expect(truck.health).toBe(0);
  });

  it('reports the destroying hit one time only', () => {
    const truck = testTarget();
    applyHit(truck);
    expect(applyHit(truck)).toBe(true);
    expect(applyHit(truck)).toBe(false);
    expect(truck.hits).toBe(3);
  });
});

describe('the target field', () => {
  const targets = placeTargets();

  it('holds a hangar, parked aircraft, trucks and fuel drums', () => {
    const kinds = new Set(targets.map((target) => target.kind));
    expect(kinds.has('hangar')).toBe(true);
    expect(kinds.has('parked-aircraft')).toBe(true);
    expect(kinds.has('truck')).toBe(true);
    expect(kinds.has('fuel-drums')).toBe(true);
    expect(targets.length).toBeGreaterThan(20);
  });

  it('stands clear of the runway, the apron and both approach corridors', () => {
    for (const target of targets) {
      const spec = TARGET_KINDS[target.kind];
      const obstacle = {
        north: target.position.x,
        east: target.position.y,
        radius: targetFootprintRadius(spec),
        height: 2 * spec.halfExtent.z,
      };
      expect(isClearOfAirfield(obstacle)).toBe(true);
      expect(insideAirfieldBoundary(obstacle)).toBe(false);
    }
  });

  it('stands every target on the ground', () => {
    for (const target of targets) {
      expect(target.position.z).toBeCloseTo(-target.halfExtent.z, 9);
    }
  });

  it('rebuilds the same field from the same seed', () => {
    const again = placeTargets();
    expect(again.length).toBe(targets.length);
    for (let i = 0; i < targets.length; i++) {
      expect(again[i].kind).toBe(targets[i].kind);
      expect(again[i].position.x).toBeCloseTo(targets[i].position.x, 12);
      expect(again[i].position.y).toBeCloseTo(targets[i].position.y, 12);
      expect(again[i].heading).toBeCloseTo(targets[i].heading, 12);
    }
  });

  it('builds a different field from a different seed', () => {
    const other = placeTargets(1);
    let moved = 0;
    for (let i = 0; i < Math.min(other.length, targets.length); i++) {
      if (Math.abs(other[i].position.x - targets[i].position.x) > 1e-6) moved += 1;
    }
    expect(moved).toBeGreaterThan(0);
  });
});
