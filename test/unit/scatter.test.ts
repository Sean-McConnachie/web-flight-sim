/**
 * Tests of the world scatter.
 *
 * The tests run the placement half of src/world/scatter.ts with no renderer.
 * They rebuild the geometry of the clear zone and of the approach surface from
 * first principles, so a change to the module that breaks a rule fails here and
 * not in the air.
 */

import { describe, expect, it } from 'vitest';

import { config } from '@/core/config';
import { threeToNed } from '@/render/frames';
import { Vector3 } from 'three';

import type {
  BuildingFootprint,
  Obstacle,
  ScatterResult,
  TreeFootprint,
} from '@/world/scatter';
import {
  APPROACH_CLEARANCE,
  APPROACH_DIVERGENCE,
  APPROACH_INNER_HALF_WIDTH,
  APPROACH_LENGTH,
  RUNWAY_STRIP_END,
  RUNWAY_STRIP_HALF_WIDTH,
  footprintRadius,
  scatterWorld,
} from '@/world/scatter';

// ---------------------------------------------------------------------------
// Stand in type descriptors
// ---------------------------------------------------------------------------

/**
 * The sizes match src/render/models/buildings.ts and
 * src/render/models/trees.ts. The placement half never asks for geometry, so
 * plain numbers are enough and the test needs no GPU.
 */
const BUILDING_TYPES: BuildingFootprint[] = [
  { name: 'hangar', footprint: { x: 35, z: 27 }, height: 14 },
  { name: 'tower', footprint: { x: 11.8, z: 11.8 }, height: 21.4 },
  { name: 'hut', footprint: { x: 10, z: 6 }, height: 4.6 },
  { name: 'barracks', footprint: { x: 24, z: 8 }, height: 5.4 },
  { name: 'workshop', footprint: { x: 18, z: 12 }, height: 8.6 },
  { name: 'fuel-dump', footprint: { x: 26, z: 18 }, height: 9.6 },
  { name: 'revetment', footprint: { x: 30, z: 26 }, height: 4.5 },
];

const TREE_TYPES: TreeFootprint[] = [
  { name: 'conifer-small', radius: 1.6, height: 6 },
  { name: 'conifer-medium', radius: 2.6, height: 12 },
  { name: 'conifer-large', radius: 3.6, height: 19 },
  { name: 'broadleaf-small', radius: 2.2, height: 5 },
  { name: 'broadleaf-medium', radius: 3.8, height: 9 },
  { name: 'broadleaf-large', radius: 5.5, height: 14 },
  { name: 'bush-small', radius: 0.9, height: 1.2 },
  { name: 'bush-medium', radius: 1.5, height: 2 },
  { name: 'bush-large', radius: 2.3, height: 3.2 },
];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** One placed object, back in the NED frame that every rule is written in. */
interface PlacedObject extends Obstacle {
  typeName: string;
  scale: number;
}

const scratchThree = new Vector3();
const scratchNed = new Vector3();

/**
 * Turn a scatter result into a flat list in the NED frame.
 *
 * The scatter returns positions in the render frame, so the test sends them
 * back through src/render/frames.ts. That also proves the two conversions agree
 * on the world the scatter built.
 */
function toNedObjects(result: ScatterResult): PlacedObject[] {
  const objects: PlacedObject[] = [];

  for (const bucket of result.buildings) {
    const radius = footprintRadius(bucket.type.footprint);
    for (const placement of bucket.placements) {
      scratchThree.copy(placement.position);
      threeToNed(scratchThree, scratchNed);
      objects.push({
        typeName: bucket.type.name,
        scale: placement.scale,
        north: scratchNed.x,
        east: scratchNed.y,
        radius: radius * placement.scale,
        height: bucket.type.height * placement.scale,
      });
    }
  }

  for (const bucket of result.trees) {
    for (const placement of bucket.placements) {
      scratchThree.copy(placement.position);
      threeToNed(scratchThree, scratchNed);
      objects.push({
        typeName: bucket.type.name,
        scale: placement.scale,
        north: scratchNed.x,
        east: scratchNed.y,
        radius: bucket.type.radius * placement.scale,
        height: bucket.type.height * placement.scale,
      });
    }
  }

  return objects;
}

function countBuildings(result: ScatterResult): number {
  return result.buildings.reduce((sum, bucket) => sum + bucket.placements.length, 0);
}

function countTrees(result: ScatterResult): number {
  return result.trees.reduce((sum, bucket) => sum + bucket.placements.length, 0);
}

/** The one world that every test below reads. */
const world = scatterWorld(BUILDING_TYPES, TREE_TYPES);
const objects = toNedObjects(world);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('the scatter repeats exactly', () => {
  it('gives an identical placement list for the same seed', () => {
    const first = scatterWorld(BUILDING_TYPES, TREE_TYPES);
    const second = scatterWorld(BUILDING_TYPES, TREE_TYPES);

    expect(second.buildings.length).toBe(first.buildings.length);
    expect(second.trees.length).toBe(first.trees.length);

    let compared = 0;
    for (let b = 0; b < first.buildings.length; b += 1) {
      const a = first.buildings[b].placements;
      const c = second.buildings[b].placements;
      expect(c.length).toBe(a.length);
      for (let i = 0; i < a.length; i += 1) {
        expect(c[i].position.x).toBe(a[i].position.x);
        expect(c[i].position.y).toBe(a[i].position.y);
        expect(c[i].position.z).toBe(a[i].position.z);
        expect(c[i].rotationY).toBe(a[i].rotationY);
        expect(c[i].scale).toBe(a[i].scale);
        compared += 1;
      }
    }
    for (let t = 0; t < first.trees.length; t += 1) {
      const a = first.trees[t].placements;
      const c = second.trees[t].placements;
      expect(c.length).toBe(a.length);
      for (let i = 0; i < a.length; i += 1) {
        expect(c[i].position.x).toBe(a[i].position.x);
        expect(c[i].position.y).toBe(a[i].position.y);
        expect(c[i].position.z).toBe(a[i].position.z);
        expect(c[i].rotationY).toBe(a[i].rotationY);
        expect(c[i].scale).toBe(a[i].scale);
        compared += 1;
      }
    }

    expect(compared).toBe(countBuildings(first) + countTrees(first));
  });

  it('gives a different placement list for a different seed', () => {
    const other = scatterWorld(BUILDING_TYPES, TREE_TYPES, config.world.scatterSeed + 1);

    // Compare the trees, because the airfield buildings come from a fixed site
    // plan and only their small jitter follows the seed.
    let identical = 0;
    let compared = 0;
    for (let t = 0; t < world.trees.length; t += 1) {
      const a = world.trees[t].placements;
      const b = other.trees[t].placements;
      const shared = Math.min(a.length, b.length);
      for (let i = 0; i < shared; i += 1) {
        compared += 1;
        if (a[i].position.x === b[i].position.x && a[i].position.z === b[i].position.z) {
          identical += 1;
        }
      }
    }

    expect(compared).toBeGreaterThan(1000);
    expect(identical).toBe(0);
  });

  it('moves even the jitter of the airfield when the seed changes', () => {
    const other = scatterWorld(BUILDING_TYPES, TREE_TYPES, config.world.scatterSeed + 1);
    const a = world.buildings[0].placements[0];
    const b = other.buildings[0].placements[0];
    expect(a).toBeDefined();
    expect(b).toBeDefined();
    expect(a.position.x === b.position.x && a.position.z === b.position.z).toBe(false);
  });
});

describe('the runway strip stays clear', () => {
  it('holds no object inside the pavement and its margin', () => {
    const halfWidth = config.world.runwayWidth / 2 + 3 * config.world.runwayWidth;
    expect(RUNWAY_STRIP_HALF_WIDTH).toBeCloseTo(halfWidth, 9);
    expect(RUNWAY_STRIP_HALF_WIDTH).toBeGreaterThanOrEqual(150);
    expect(RUNWAY_STRIP_END).toBeGreaterThanOrEqual(60);

    for (const object of objects) {
      const insideAlong =
        object.north + object.radius > -RUNWAY_STRIP_END &&
        object.north - object.radius < config.world.runwayLength + RUNWAY_STRIP_END;
      const insideAcross = Math.abs(object.east) - object.radius < RUNWAY_STRIP_HALF_WIDTH;
      expect(
        insideAlong && insideAcross,
        `${object.typeName} at north ${object.north.toFixed(1)} east ${object.east.toFixed(1)}`,
      ).toBe(false);
    }
  });

  it('holds no object on the pavement itself', () => {
    for (const object of objects) {
      const onPavement =
        object.north > -object.radius &&
        object.north < config.world.runwayLength + object.radius &&
        Math.abs(object.east) < config.world.runwayWidth / 2 + object.radius;
      expect(onPavement).toBe(false);
    }
  });
});

describe('the approach corridor stays clear at both ends', () => {
  /**
   * Height of the 3 degree approach surface, rebuilt here from the angle. The
   * surface starts at the end of the strip, at ground level.
   */
  const slope = Math.tan((3 * Math.PI) / 180);

  it('keeps every object under the 3 degree surface off the south end', () => {
    for (const object of objects) {
      // Distance from the end of the strip to the nearest point of the object.
      const distance = -(object.north + object.radius) - RUNWAY_STRIP_END;
      if (distance < 0 || distance > APPROACH_LENGTH) continue;

      const halfWidth = APPROACH_INNER_HALF_WIDTH + APPROACH_DIVERGENCE * distance;
      if (Math.abs(object.east) - object.radius > halfWidth) continue;

      const surface = distance * slope;
      expect(
        object.height + APPROACH_CLEARANCE,
        `${object.typeName} ${distance.toFixed(0)} m off the south end`,
      ).toBeLessThanOrEqual(surface);
    }
  });

  it('keeps every object under the 3 degree surface off the north end', () => {
    for (const object of objects) {
      const distance =
        object.north - object.radius - config.world.runwayLength - RUNWAY_STRIP_END;
      if (distance < 0 || distance > APPROACH_LENGTH) continue;

      const halfWidth = APPROACH_INNER_HALF_WIDTH + APPROACH_DIVERGENCE * distance;
      if (Math.abs(object.east) - object.radius > halfWidth) continue;

      const surface = distance * slope;
      expect(
        object.height + APPROACH_CLEARANCE,
        `${object.typeName} ${distance.toFixed(0)} m off the north end`,
      ).toBeLessThanOrEqual(surface);
    }
  });

  it('leaves at least 29 m under the path an aircraft really flies', () => {
    // A 3 degree approach that aims 300 m in from the threshold crosses the
    // threshold at 300 * tan(3 deg). The path and the surface share one slope,
    // so the gap between them is the same at every distance.
    const aimingPointDistance = 300;
    const gap = (aimingPointDistance + RUNWAY_STRIP_END) * slope + APPROACH_CLEARANCE;
    expect(gap).toBeGreaterThan(29);

    for (const object of objects) {
      for (const distance of [
        -(object.north + object.radius) - RUNWAY_STRIP_END,
        object.north - object.radius - config.world.runwayLength - RUNWAY_STRIP_END,
      ]) {
        if (distance < 0 || distance > APPROACH_LENGTH) continue;
        const halfWidth = APPROACH_INNER_HALF_WIDTH + APPROACH_DIVERGENCE * distance;
        if (Math.abs(object.east) - object.radius > halfWidth) continue;

        const flownPath = (aimingPointDistance + RUNWAY_STRIP_END + distance) * slope;
        expect(flownPath - object.height).toBeGreaterThanOrEqual(29);
      }
    }
  });

  it('leaves the corridor long enough that no object can reach its far end', () => {
    // The surface at the end of the corridor is far above the tallest object,
    // so nothing beyond the corridor needs a test.
    const tallest = Math.max(...objects.map((object) => object.height));
    expect(APPROACH_LENGTH * slope).toBeGreaterThan(tallest + APPROACH_CLEARANCE);
  });
});

describe('no two objects overlap', () => {
  it('keeps every pair of footprints apart', () => {
    // The same uniform grid the scatter uses, so the check is not an O(n^2)
    // sweep over more than four thousand objects.
    const cell = 60;
    const buckets = new Map<string, number[]>();
    for (let i = 0; i < objects.length; i += 1) {
      const key = `${Math.floor(objects[i].north / cell)}:${Math.floor(objects[i].east / cell)}`;
      const bucket = buckets.get(key);
      if (bucket === undefined) buckets.set(key, [i]);
      else bucket.push(i);
    }

    let worstOverlap = 0;
    let checked = 0;
    for (let i = 0; i < objects.length; i += 1) {
      const a = objects[i];
      const cellNorth = Math.floor(a.north / cell);
      const cellEast = Math.floor(a.east / cell);
      for (let dn = -1; dn <= 1; dn += 1) {
        for (let de = -1; de <= 1; de += 1) {
          const bucket = buckets.get(`${cellNorth + dn}:${cellEast + de}`);
          if (bucket === undefined) continue;
          for (const j of bucket) {
            if (j <= i) continue;
            const b = objects[j];
            const distance = Math.hypot(a.north - b.north, a.east - b.east);
            worstOverlap = Math.max(worstOverlap, a.radius + b.radius - distance);
            checked += 1;
          }
        }
      }
    }

    expect(checked).toBeGreaterThan(objects.length);
    expect(worstOverlap).toBeLessThanOrEqual(0);
  });

  it('uses a grid cell wider than twice the largest footprint', () => {
    // The nine cell search above is only correct when this holds.
    const largest = Math.max(...objects.map((object) => object.radius));
    expect(2 * largest).toBeLessThan(60);
  });
});

describe('the counts match the configuration', () => {
  it('places every tree the configuration asks for', () => {
    expect(countTrees(world)).toBe(config.world.treeCount);
  });

  it('places every building the configuration asks for', () => {
    expect(countBuildings(world)).toBe(config.world.buildingCount);
  });

  it('builds the airfield from the site plan', () => {
    // The site plan puts three hangars and one control tower beside the runway.
    // A silent change that dropped them would leave the airfield unreadable.
    const hangars = world.buildings.find((bucket) => bucket.type.name === 'hangar');
    const towers = world.buildings.find((bucket) => bucket.type.name === 'tower');
    expect(hangars?.placements.length).toBeGreaterThanOrEqual(3);
    expect(towers?.placements.length).toBeGreaterThanOrEqual(1);
  });
});

describe('the world sits inside the ground plane', () => {
  it('keeps every object on the ground square', () => {
    const limit = config.world.groundSize / 2;
    for (const object of objects) {
      expect(Math.abs(object.north) + object.radius).toBeLessThan(limit);
      expect(Math.abs(object.east) + object.radius).toBeLessThan(limit);
    }
  });

  it('grows the wood denser toward the edge of the world', () => {
    // The grove radius follows the cube root of a flat draw, so the count of
    // trees in a ring grows as the square of the radius. Compare the inner half
    // of the world with the outer half by area.
    const half = 9000;
    let inner = 0;
    let outer = 0;
    for (const object of objects) {
      if (!object.typeName.includes('-')) continue;
      if (object.typeName.startsWith('hangar')) continue;
      const radius = Math.hypot(object.north, object.east);
      if (radius < half) inner += 1;
      else if (radius < 18000) outer += 1;
    }
    const innerArea = Math.PI * half * half;
    const outerArea = Math.PI * (18000 * 18000 - half * half);
    expect(outer / outerArea).toBeGreaterThan(inner / innerArea);
  });
});
