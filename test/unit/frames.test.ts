import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';

import { nedQuatToThree, nedToThree, threeQuatToNed, threeToNed } from '@/render/frames';

/**
 * Tests for the NED to Three.js frame conversion.
 *
 * The interesting test is the last one. A wrong attitude conversion, such as a
 * plain product in place of a conjugation, still passes a level flight check by
 * eye. It fails the conjugation property at once.
 */

const EPS = 1e-12;

/** A small deterministic generator, so a failure repeats on the next run. */
function makeRandom(seed: number): () => number {
  let s = seed >>> 0;
  return (): number => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** A vector with components over a range that a flight simulator really uses. */
function randomVector(rand: () => number, out: Vector3): Vector3 {
  return out.set((rand() - 0.5) * 60000, (rand() - 0.5) * 60000, (rand() - 0.5) * 20000);
}

/**
 * A uniform random unit quaternion. Source: Shoemake, "Uniform Random Rotations",
 * Graphics Gems III, 1992.
 */
function randomQuaternion(rand: () => number, out: Quaternion): Quaternion {
  const u1 = rand();
  const u2 = rand() * 2 * Math.PI;
  const u3 = rand() * 2 * Math.PI;
  const r1 = Math.sqrt(1 - u1);
  const r2 = Math.sqrt(u1);
  return out.set(r1 * Math.sin(u2), r1 * Math.cos(u2), r2 * Math.sin(u3), r2 * Math.cos(u3));
}

function expectVectorClose(actual: Vector3, x: number, y: number, z: number, eps = EPS): void {
  expect(actual.x).toBeCloseTo(x, 9);
  expect(actual.y).toBeCloseTo(y, 9);
  expect(actual.z).toBeCloseTo(z, 9);
  expect(Math.abs(actual.x - x) + Math.abs(actual.y - y) + Math.abs(actual.z - z)).toBeLessThan(eps);
}

describe('frame conversion between NED and the Three.js render frame', () => {
  it('north points along render -z, east along +x, and down along -y', () => {
    const out = new Vector3();

    // North. Three.js looks along -z, so north is forward.
    expectVectorClose(nedToThree(new Vector3(1, 0, 0), out), 0, 0, -1);
    // East.
    expectVectorClose(nedToThree(new Vector3(0, 1, 0), out), 1, 0, 0);
    // Down. Altitude is -ned.z, so down must give -y.
    expectVectorClose(nedToThree(new Vector3(0, 0, 1), out), 0, -1, 0);
  });

  it('the render basis vectors map back to north, east, and down', () => {
    const out = new Vector3();

    expectVectorClose(threeToNed(new Vector3(1, 0, 0), out), 0, 1, 0); // right is east
    expectVectorClose(threeToNed(new Vector3(0, 1, 0), out), 0, 0, -1); // up is minus down
    expectVectorClose(threeToNed(new Vector3(0, 0, 1), out), -1, 0, 0); // back is minus north
  });

  it('an aircraft 1000 m above the runway sits 1000 m up in the render frame', () => {
    // CONVENTIONS 3.2: altitude above the ground is -position.z.
    const out = new Vector3();
    expectVectorClose(nedToThree(new Vector3(0, 0, -1000), out), 0, 1000, 0);
  });

  it('a round trip through the render frame returns the same NED vector', () => {
    const rand = makeRandom(0x5eed);
    const ned = new Vector3();
    const three = new Vector3();
    const back = new Vector3();

    for (let i = 0; i < 2000; i++) {
      randomVector(rand, ned);
      nedToThree(ned, three);
      threeToNed(three, back);
      expectVectorClose(back, ned.x, ned.y, ned.z, 1e-9);
    }
  });

  it('a round trip through the NED frame returns the same render vector', () => {
    const rand = makeRandom(0xc0ffee);
    const three = new Vector3();
    const ned = new Vector3();
    const back = new Vector3();

    for (let i = 0; i < 2000; i++) {
      randomVector(rand, three);
      threeToNed(three, ned);
      nedToThree(ned, back);
      expectVectorClose(back, three.x, three.y, three.z, 1e-9);
    }
  });

  it('the conversion works in place, when the input and the output are one object', () => {
    const v = new Vector3(100, 200, 300);
    nedToThree(v, v);
    expectVectorClose(v, 200, -300, -100);
    threeToNed(v, v);
    expectVectorClose(v, 100, 200, 300);
  });

  it('the conversion keeps the length of a vector', () => {
    const rand = makeRandom(0xa11ce);
    const ned = new Vector3();
    const three = new Vector3();

    for (let i = 0; i < 500; i++) {
      randomVector(rand, ned);
      nedToThree(ned, three);
      expect(three.length()).toBeCloseTo(ned.length(), 6);
    }
  });

  it('the conversion is a rotation and not a mirror, so it keeps the cross product', () => {
    // A mirror would flip every cross product. A flipped frame turns a right
    // roll into a left roll on screen. Check the property directly.
    const rand = makeRandom(0xbeef);
    const a = new Vector3();
    const b = new Vector3();
    const crossThenMap = new Vector3();
    const mapA = new Vector3();
    const mapB = new Vector3();
    const mapThenCross = new Vector3();

    for (let i = 0; i < 500; i++) {
      randomVector(rand, a).normalize();
      randomVector(rand, b).normalize();

      crossThenMap.copy(a).cross(b);
      nedToThree(crossThenMap, crossThenMap);

      nedToThree(a, mapA);
      nedToThree(b, mapB);
      mapThenCross.copy(mapA).cross(mapB);

      expectVectorClose(
        mapThenCross,
        crossThenMap.x,
        crossThenMap.y,
        crossThenMap.z,
        1e-12,
      );
    }
  });
});

describe('attitude quaternion conversion between NED and the render frame', () => {
  it('a body vector keeps its place, whether the turn happens before or after the conversion', () => {
    // The property: nedToThree(q_ned * v_body) equals nedQuatToThree(q_ned)
    // applied to nedToThree(v_body). This is the property that matters. A
    // plain product in place of the conjugation fails it.
    const rand = makeRandom(0x1234abcd);

    const qNed = new Quaternion();
    const qThree = new Quaternion();
    const vBody = new Vector3();
    const turnedThenMapped = new Vector3();
    const mappedThenTurned = new Vector3();

    for (let i = 0; i < 2000; i++) {
      randomQuaternion(rand, qNed);
      randomVector(rand, vBody).normalize();

      // Turn in NED, then convert.
      turnedThenMapped.copy(vBody).applyQuaternion(qNed);
      nedToThree(turnedThenMapped, turnedThenMapped);

      // Convert, then turn in the render frame.
      nedQuatToThree(qNed, qThree);
      nedToThree(vBody, mappedThenTurned);
      mappedThenTurned.applyQuaternion(qThree);

      expectVectorClose(
        mappedThenTurned,
        turnedThenMapped.x,
        turnedThenMapped.y,
        turnedThenMapped.z,
        1e-12,
      );
    }
  });

  it('the same property holds for the inverse conversion', () => {
    const rand = makeRandom(0x99887766);

    const qThree = new Quaternion();
    const qNed = new Quaternion();
    const vRender = new Vector3();
    const turnedThenMapped = new Vector3();
    const mappedThenTurned = new Vector3();

    for (let i = 0; i < 2000; i++) {
      randomQuaternion(rand, qThree);
      randomVector(rand, vRender).normalize();

      turnedThenMapped.copy(vRender).applyQuaternion(qThree);
      threeToNed(turnedThenMapped, turnedThenMapped);

      threeQuatToNed(qThree, qNed);
      threeToNed(vRender, mappedThenTurned);
      mappedThenTurned.applyQuaternion(qNed);

      expectVectorClose(
        mappedThenTurned,
        turnedThenMapped.x,
        turnedThenMapped.y,
        turnedThenMapped.z,
        1e-12,
      );
    }
  });

  it('a round trip returns the same attitude quaternion', () => {
    const rand = makeRandom(0x2468ace0);
    const qNed = new Quaternion();
    const qThree = new Quaternion();
    const back = new Quaternion();

    for (let i = 0; i < 2000; i++) {
      randomQuaternion(rand, qNed);
      nedQuatToThree(qNed, qThree);
      threeQuatToNed(qThree, back);
      // q and -q are the same rotation, so compare the absolute dot product.
      expect(Math.abs(back.dot(qNed))).toBeCloseTo(1, 12);
    }
  });

  it('the conversion keeps the quaternion a unit, so it never scales the model', () => {
    const rand = makeRandom(0x13572468);
    const qNed = new Quaternion();
    const qThree = new Quaternion();

    for (let i = 0; i < 500; i++) {
      randomQuaternion(rand, qNed);
      nedQuatToThree(qNed, qThree);
      expect(qThree.length()).toBeCloseTo(1, 12);
    }
  });

  it('the level attitude puts the nose along render -z and the right wing along +x', () => {
    // Identity attitude in NED. The nose, body +x, points north.
    const qThree = new Quaternion();
    nedQuatToThree(new Quaternion(), qThree);

    const nose = new Vector3();
    const rightWing = new Vector3();
    const floor = new Vector3();

    nedToThree(new Vector3(1, 0, 0), nose).applyQuaternion(qThree);
    nedToThree(new Vector3(0, 1, 0), rightWing).applyQuaternion(qThree);
    nedToThree(new Vector3(0, 0, 1), floor).applyQuaternion(qThree);

    expectVectorClose(nose, 0, 0, -1);
    expectVectorClose(rightWing, 1, 0, 0);
    expectVectorClose(floor, 0, -1, 0);
  });

  it('a nose up pitch of 90 degrees in NED points the nose at render +y', () => {
    // A positive pitch rate raises the nose, CONVENTIONS 3.1. Pitch is a right
    // hand turn about the body y axis, which is east at the level attitude.
    // A right hand turn about east by +90 degrees takes north (+x) to (0, 0, -1).
    // That is straight up, because altitude is -ned.z.
    const east = new Vector3(0, 1, 0);
    const qNed = new Quaternion().setFromAxisAngle(east, Math.PI / 2);

    const qThree = new Quaternion();
    nedQuatToThree(qNed, qThree);

    const nose = new Vector3();
    nedToThree(new Vector3(1, 0, 0), nose).applyQuaternion(qThree);

    // Straight up in the render frame.
    expectVectorClose(nose, 0, 1, 0, 1e-12);
  });
});
