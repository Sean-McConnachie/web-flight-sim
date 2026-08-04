/**
 * Deterministic placement of every building and every tree in the world.
 *
 * The module holds two halves. `scatterWorld` is the placement half. It reads
 * only the size of each type and returns a list of positions, so a test can run
 * it in Node with no GPU and no browser. `createScatteredWorld` is the render
 * half. It builds the models, hands the placements to the instanced draw of
 * src/render/instanced.ts, and gives the caller one update call.
 *
 * The same seed must give the same world on every run and on every machine, so
 * every random value comes from src/core/prng.ts. No call to `Math.random`
 * belongs here.
 *
 *
 * WHERE THE RUNWAY IS
 *
 * The NED origin is the runway threshold, so the pavement runs from north 0 m
 * to north 2400 m. It is not centered on the origin. Read the module comment of
 * src/world/runway.ts before you change any number below.
 *
 *
 * RULE 1, THE CLEAR ZONE
 *
 * Nothing stands on the pavement and nothing stands in the strip around it. The
 * margin comes from the runway width: it is three times the width from the
 * pavement edge, which is 135 m, so the strip reaches 157.5 m from the
 * centerline. ICAO Annex 14 asks for 150 m for a precision approach runway of
 * code 3 or 4, so this margin covers the standard. The strip also reaches
 * 67.5 m beyond each end of the pavement, over the 60 m that Annex 14 asks for.
 *
 *
 * RULE 2, THE APPROACH CORRIDOR
 *
 * This is the rule that makes the simulator usable or unusable, so here is the
 * whole calculation.
 *
 * An approach surface starts at the end of the strip, at ground level, at each
 * end of the runway. It rises at 3 degrees, which is a slope of
 * tan(3 deg) = 0.05241. Its inner edge is as wide as the strip, 157.5 m each
 * side of the centerline, and it spreads by 15 percent each side as it goes
 * out, again the Annex 14 figure. It runs 3000 m, where it is already 157 m
 * high and no object can reach it.
 *
 * An object is rejected when its top plus 10 m of clearance is above that
 * surface, at the distance of the nearest point of its footprint. So the
 * shortest legal distance from the end of the strip is:
 *
 *   a 3 m bush     (3 + 10) / 0.05241 =  248 m
 *   a 14 m tree    (14 + 10) / 0.05241 = 458 m
 *   a 21 m tower   (21 + 10) / 0.05241 = 591 m
 *
 * Now compare that surface with the path an aircraft really flies. A 3 degree
 * approach aims at a touchdown point 300 m in from the threshold, so it crosses
 * the threshold at 300 * 0.05241 = 15.7 m, which is the standard threshold
 * crossing height. At a distance `d` beyond the end of the strip the aircraft
 * is therefore at:
 *
 *   (300 + 67.5 + d) * 0.05241 = 19.3 + 0.05241 * d
 *
 * The surface at the same point is 0.05241 * d. The two have the same slope, so
 * the flown path is 19.3 m above the surface everywhere, and the 10 m clearance
 * puts every object at least 29.3 m below the aircraft. That margin holds all
 * the way down to the threshold.
 *
 * The corridor is checked at BOTH ends, because the runway is used in both
 * directions. The takeoff climb needs no separate rule: the Me-262 climbs at
 * 20 m/s at about 90 m/s, which is a gradient near 12 degrees, so anything that
 * clears the 3 degree approach surface clears the climb by a wide margin.
 *
 *
 * RULE 3, THE LAYOUT
 *
 * The hangars and the control tower stand east of the runway and face it, on
 * the far side of the apron. The workshops, the barracks, and the huts sit
 * behind them, and the fuel dump sits behind those, because a fuel dump next to
 * a hangar is how an airfield loses both. The revetments line the west side.
 * Everything else is countryside: small villages and lone farms from 1.5 km to
 * 9 km out. Trees keep away from the airfield and grow in groves, and the
 * groves get denser toward the edge of the world.
 *
 *
 * RULE 4, NO OVERLAP
 *
 * A test of every new object against every placed object would cost 8 million
 * comparisons for 4000 trees. The module uses a uniform grid instead. Each cell
 * is wider than twice the largest footprint, so a new object can only meet an
 * object in one of the nine cells around it. The cost is then a fixed amount of
 * work for each object.
 */

import { Vector3 } from 'three';
import type { Object3D } from 'three/webgpu';
import { Group } from 'three/webgpu';

import { config } from '@/core/config';
import type { Rng } from '@/core/prng';
import { createRng } from '@/core/prng';
import { nedToThree } from '@/render/frames';
import type { InstancePlacement, InstancedGroup } from '@/render/instanced';
import { createInstancedGroup } from '@/render/instanced';
import type { BuildingType } from '@/render/models/buildings';
import {
  buildingTint,
  createBuildingTypes,
  disposeBuildingTypes,
} from '@/render/models/buildings';
import type { TreeType } from '@/render/models/trees';
import {
  createTreeTypes,
  disposeTreeTypes,
  treeTint,
  updateTreeWind,
} from '@/render/models/trees';

// ---------------------------------------------------------------------------
// The geometry of the clear zone and of the approach corridor
// ---------------------------------------------------------------------------

/** Degrees to radians. src/math/units.ts holds the same constant. */
const DEG = Math.PI / 180;

/**
 * Clear margin from the edge of the pavement, in meters. The value is three
 * runway widths. Read rule 1 in the module comment.
 */
const STRIP_MARGIN = 3 * config.world.runwayWidth;

/** Half width of the clear strip, measured from the centerline, in meters. */
export const RUNWAY_STRIP_HALF_WIDTH = config.world.runwayWidth / 2 + STRIP_MARGIN;

/** Length of the clear strip beyond each end of the pavement, in meters. */
export const RUNWAY_STRIP_END = 1.5 * config.world.runwayWidth;

/** Angle of the approach surface, in radians. */
export const GLIDE_SLOPE_RAD = 3 * DEG;

/** Slope of the approach surface, as a rise over a run. */
export const GLIDE_SLOPE = Math.tan(GLIDE_SLOPE_RAD);

/** Half width of the approach surface at its inner edge, in meters. */
export const APPROACH_INNER_HALF_WIDTH = RUNWAY_STRIP_HALF_WIDTH;

/** Spread of the approach surface, as a half width gained per meter out. */
export const APPROACH_DIVERGENCE = 0.15;

/** Length of the approach surface, in meters. */
export const APPROACH_LENGTH = 3000;

/** Clearance an object must keep below the approach surface, in meters. */
export const APPROACH_CLEARANCE = 10;

/**
 * The apron and its margin, in NED meters. The numbers follow the apron of
 * src/world/runway.ts with 20 m added on every side. Change them together.
 */
const APRON_KEEP_OUT = { northMin: -60, northMax: 180, eastMin: 110, eastMax: 350 };

/**
 * The boundary fence of the airfield, in NED meters.
 *
 * Inside the fence the grass is mown and the only buildings are the ones the
 * site plan puts there. No wood grows on an airfield and no village stands on
 * one. So a grove and a countryside building must both stay outside this
 * rectangle, while the fixed site plan works inside it.
 *
 * The rectangle reaches 500 m past each end of the pavement and it is wider on
 * the east side, where the apron and the flight line are.
 */
const AIRFIELD_BOUNDARY = {
  northMin: -500,
  northMax: config.world.runwayLength + 500,
  eastMin: -500,
  eastMax: 750,
};

/** Half side of the ground plane that still holds an object, in meters. */
const WORLD_LIMIT = config.world.groundSize / 2 - 500;

/** One object on the ground, seen from above, in the NED world frame. */
export interface Obstacle {
  /** Distance north of the runway threshold, in meters. */
  north: number;

  /** Distance east of the runway centerline, in meters. */
  east: number;

  /** Radius of a circle that covers the footprint, in meters. */
  radius: number;

  /** Height of the highest point above the ground, in meters. */
  height: number;
}

/** Radius of a circle that covers a rectangular footprint, in meters. */
export function footprintRadius(footprint: { x: number; z: number }): number {
  return 0.5 * Math.hypot(footprint.x, footprint.z);
}

/** True when any part of the object reaches into the runway strip. */
export function insideRunwayStrip(obstacle: Obstacle): boolean {
  const { north, east, radius } = obstacle;
  return (
    north + radius > -RUNWAY_STRIP_END &&
    north - radius < config.world.runwayLength + RUNWAY_STRIP_END &&
    Math.abs(east) - radius < RUNWAY_STRIP_HALF_WIDTH
  );
}

/** Height of the approach surface at `distance` out from the end of the strip. */
export function approachSurfaceHeight(distance: number): number {
  return distance * GLIDE_SLOPE;
}

/** Half width of the approach surface at `distance` out from the end of the strip. */
export function approachHalfWidth(distance: number): number {
  return APPROACH_INNER_HALF_WIDTH + APPROACH_DIVERGENCE * distance;
}

/**
 * Distance of the nearest point of the object from the end of the strip, at
 * each of the two runway ends, in meters. A negative value means the object is
 * beside the runway and not off an end.
 */
export function approachDistances(obstacle: Obstacle): [number, number] {
  const south = -(obstacle.north + obstacle.radius) - RUNWAY_STRIP_END;
  const north = obstacle.north - obstacle.radius - config.world.runwayLength - RUNWAY_STRIP_END;
  return [south, north];
}

/**
 * True when the object reaches into the approach corridor of either end. Read
 * rule 2 in the module comment for the whole calculation.
 */
export function breaksApproachSurface(obstacle: Obstacle): boolean {
  const distances = approachDistances(obstacle);
  for (const distance of distances) {
    if (distance < 0 || distance > APPROACH_LENGTH) continue;
    if (Math.abs(obstacle.east) - obstacle.radius > approachHalfWidth(distance)) continue;
    if (obstacle.height + APPROACH_CLEARANCE > approachSurfaceHeight(distance)) return true;
  }
  return false;
}

/** True when any part of the object reaches into the apron and its margin. */
export function insideApron(obstacle: Obstacle): boolean {
  const { north, east, radius } = obstacle;
  return (
    north + radius > APRON_KEEP_OUT.northMin &&
    north - radius < APRON_KEEP_OUT.northMax &&
    east + radius > APRON_KEEP_OUT.eastMin &&
    east - radius < APRON_KEEP_OUT.eastMax
  );
}

/** True when any part of the object reaches inside the airfield boundary. */
export function insideAirfieldBoundary(obstacle: Obstacle): boolean {
  const { north, east, radius } = obstacle;
  return (
    north + radius > AIRFIELD_BOUNDARY.northMin &&
    north - radius < AIRFIELD_BOUNDARY.northMax &&
    east + radius > AIRFIELD_BOUNDARY.eastMin &&
    east - radius < AIRFIELD_BOUNDARY.eastMax
  );
}

/** True when the object may stand where it is, before the overlap test. */
export function isClearOfAirfield(obstacle: Obstacle): boolean {
  if (Math.abs(obstacle.north) > WORLD_LIMIT) return false;
  if (Math.abs(obstacle.east) > WORLD_LIMIT) return false;
  if (insideRunwayStrip(obstacle)) return false;
  if (insideApron(obstacle)) return false;
  if (breaksApproachSurface(obstacle)) return false;
  return true;
}

// ---------------------------------------------------------------------------
// The overlap grid
// ---------------------------------------------------------------------------

/**
 * Side of one grid cell, in meters. It must be at least twice the largest
 * footprint radius plus the gap, so that two objects that meet always share a
 * cell or sit in cells that touch.
 */
const CELL_SIZE = 60;

/** Smallest gap between two footprints, in meters. */
const MIN_GAP = 1.5;

/** Offset that keeps a cell index positive, and the stride of the key. */
const CELL_ORIGIN = 1024;
const CELL_STRIDE = 4096;

interface OverlapGrid {
  /** True when a new object at this place touches nothing already placed. */
  fits(north: number, east: number, radius: number): boolean;
  add(north: number, east: number, radius: number): void;
}

function createOverlapGrid(): OverlapGrid {
  const cells = new Map<number, number[]>();
  const norths: number[] = [];
  const easts: number[] = [];
  const radii: number[] = [];

  function key(cellNorth: number, cellEast: number): number {
    return (cellNorth + CELL_ORIGIN) * CELL_STRIDE + (cellEast + CELL_ORIGIN);
  }

  return {
    fits(north: number, east: number, radius: number): boolean {
      const cellNorth = Math.floor(north / CELL_SIZE);
      const cellEast = Math.floor(east / CELL_SIZE);
      for (let dn = -1; dn <= 1; dn += 1) {
        for (let de = -1; de <= 1; de += 1) {
          const bucket = cells.get(key(cellNorth + dn, cellEast + de));
          if (bucket === undefined) continue;
          for (const index of bucket) {
            const gap = radius + radii[index] + MIN_GAP;
            const dNorth = north - norths[index];
            const dEast = east - easts[index];
            if (dNorth * dNorth + dEast * dEast < gap * gap) return false;
          }
        }
      }
      return true;
    },

    add(north: number, east: number, radius: number): void {
      const index = norths.length;
      norths.push(north);
      easts.push(east);
      radii.push(radius);
      const id = key(Math.floor(north / CELL_SIZE), Math.floor(east / CELL_SIZE));
      const bucket = cells.get(id);
      if (bucket === undefined) cells.set(id, [index]);
      else bucket.push(index);
    },
  };
}

// ---------------------------------------------------------------------------
// The placement half
// ---------------------------------------------------------------------------

/**
 * What the placement needs to know about a building type. `BuildingType` of
 * src/render/models/buildings.ts holds all of it, and more. The placement half
 * asks for no geometry and no material, so it runs with no renderer.
 */
export interface BuildingFootprint {
  name: string;
  footprint: { x: number; z: number };
  height: number;
}

/** What the placement needs to know about a tree type. */
export interface TreeFootprint {
  name: string;
  radius: number;
  height: number;
}

export interface ScatterResult<
  B extends BuildingFootprint = BuildingFootprint,
  T extends TreeFootprint = TreeFootprint,
> {
  buildings: { type: B; placements: InstancePlacement[] }[];
  trees: { type: T; placements: InstancePlacement[] }[];
}

/** Scratch vector for the frame conversion. The module allocates it one time. */
const scratchNed = new Vector3();

/**
 * Build one placement in the render frame.
 *
 * `heading` is the NED heading that the front of the model faces, in radians,
 * with 0 north. A model is built facing render `-z`, which is north, and a turn
 * about render `+y` runs the opposite way round from a NED heading, so the
 * render turn is the negative of the heading. src/render/frames.ts owns the
 * position conversion.
 */
function makePlacement(
  north: number,
  east: number,
  heading: number,
  scale: number,
): InstancePlacement {
  scratchNed.set(north, east, 0);
  const position = new Vector3();
  nedToThree(scratchNed, position);
  return { position, rotationY: -heading, scale };
}

/** One bucket of placements, with the size that every one of them shares. */
interface Bucket {
  radius: number;
  height: number;
  placements: InstancePlacement[];
}

/** A row of the fixed site plan of the airfield. */
interface Site {
  type: string;
  north: number;
  east: number;
  headingDeg: number;
}

/**
 * The airfield site plan, in NED meters. A heading of 270 faces west, which is
 * toward the runway from the east side. Read rule 3 in the module comment.
 */
const AIRFIELD_SITES: Site[] = [
  { type: 'hangar', north: 235, east: 252, headingDeg: 270 },
  { type: 'hangar', north: 310, east: 252, headingDeg: 270 },
  { type: 'hangar', north: 385, east: 252, headingDeg: 270 },
  { type: 'tower', north: 206, east: 196, headingDeg: 270 },
  { type: 'workshop', north: 250, east: 335, headingDeg: 270 },
  { type: 'workshop', north: 320, east: 335, headingDeg: 270 },
  { type: 'barracks', north: 440, east: 330, headingDeg: 270 },
  { type: 'barracks', north: 480, east: 330, headingDeg: 270 },
  { type: 'barracks', north: 520, east: 330, headingDeg: 270 },
  { type: 'hut', north: 430, east: 420, headingDeg: 270 },
  { type: 'hut', north: 460, east: 420, headingDeg: 270 },
  { type: 'hut', north: 490, east: 420, headingDeg: 270 },
  { type: 'hut', north: 520, east: 420, headingDeg: 270 },
  { type: 'hut', north: 550, east: 420, headingDeg: 270 },
  { type: 'hut', north: 580, east: 420, headingDeg: 270 },
  { type: 'fuel-dump', north: 660, east: 300, headingDeg: 0 },
  { type: 'fuel-dump', north: 660, east: 380, headingDeg: 0 },
  { type: 'revetment', north: 520, east: -245, headingDeg: 90 },
  { type: 'revetment', north: 660, east: -245, headingDeg: 90 },
  { type: 'revetment', north: 800, east: -245, headingDeg: 90 },
  { type: 'revetment', north: 1020, east: -255, headingDeg: 90 },
  { type: 'revetment', north: 1160, east: -255, headingDeg: 90 },
  { type: 'revetment', north: 1300, east: -255, headingDeg: 90 },
];

/** Jitter of a site, in meters, and of its heading, in degrees. */
const SITE_JITTER = 4;
const SITE_HEADING_JITTER = 2;

/** Types that a village is built from, most common first. */
const VILLAGE_TYPES = ['hut', 'hut', 'hut', 'barracks', 'workshop'];

/** Distance of a village from the runway threshold, in meters. */
const VILLAGE_MIN_RADIUS = 1500;
const VILLAGE_MAX_RADIUS = 9000;

/** Buildings in one village, and how far they spread from its center. */
const VILLAGE_MIN_SIZE = 3;
const VILLAGE_MAX_SIZE = 11;
const VILLAGE_SPREAD = 70;

/** Scale of a building and of a tree. Nothing is built to one exact size. */
const BUILDING_SCALE_LOW = 0.94;
const BUILDING_SCALE_HIGH = 1.09;
const TREE_SCALE_LOW = 0.78;
const TREE_SCALE_HIGH = 1.24;

/** Distance of a grove from the runway threshold, in meters. */
const GROVE_MIN_RADIUS = 900;
const GROVE_MAX_RADIUS = 18000;

/** Trees in one grove, and how far they spread from its center. */
const GROVE_MIN_SIZE = 16;
const GROVE_MAX_SIZE = 64;
const GROVE_MIN_SPREAD = 45;
const GROVE_MAX_SPREAD = 130;

/** Tries before the module gives up on one object. */
const PLACEMENT_TRIES = 8;

/** Groves and villages built beyond the target, so rejection cannot starve it. */
const CLUSTER_SURPLUS = 1.5;

/**
 * Group the types by the part of the name before the first dash, in the order
 * the names appear. A conifer wood is mostly conifers, so a grove picks one
 * group and then picks a size inside it.
 */
function groupByShape(names: readonly string[]): number[][] {
  const order: string[] = [];
  const groups: number[][] = [];
  for (let i = 0; i < names.length; i += 1) {
    const shape = names[i].split('-')[0];
    let slot = order.indexOf(shape);
    if (slot < 0) {
      slot = order.length;
      order.push(shape);
      groups.push([]);
    }
    groups[slot].push(i);
  }
  return groups;
}

/** Weights of the sizes inside one shape group, from the smallest up. */
const SIZE_WEIGHTS = [0.34, 0.42, 0.24];

function pickSize(rng: Rng, group: readonly number[]): number {
  if (group.length !== SIZE_WEIGHTS.length) return rng.pick(group);
  const draw = rng.next();
  let sum = 0;
  for (let i = 0; i < SIZE_WEIGHTS.length; i += 1) {
    sum += SIZE_WEIGHTS[i];
    if (draw < sum) return group[i];
  }
  return group[group.length - 1];
}

/**
 * Place every building and every tree.
 *
 * The result holds one bucket for each type, in the order the types came in.
 * Every position is already in the render frame, so the caller can hand the
 * placements straight to src/render/instanced.ts.
 *
 * `seed` exists so a test can prove that the world really follows the seed. The
 * application never passes it and takes the value from the configuration.
 */
export function scatterWorld<B extends BuildingFootprint, T extends TreeFootprint>(
  buildingTypes: readonly B[],
  treeTypes: readonly T[],
  seed: number = config.world.scatterSeed,
): ScatterResult<B, T> {
  const rng = createRng(seed);
  const grid = createOverlapGrid();

  const buildingBuckets: Bucket[] = buildingTypes.map((type) => ({
    radius: footprintRadius(type.footprint),
    height: type.height,
    placements: [],
  }));
  const treeBuckets: Bucket[] = treeTypes.map((type) => ({
    radius: type.radius,
    height: type.height,
    placements: [],
  }));

  /**
   * Try to stand one object at one place. The function returns false and
   * changes nothing when the place breaks a rule.
   */
  function tryPlace(
    bucket: Bucket,
    north: number,
    east: number,
    heading: number,
    scale: number,
    respectBoundary: boolean,
  ): boolean {
    const radius = bucket.radius * scale;
    const obstacle = { north, east, radius, height: bucket.height * scale };
    if (!isClearOfAirfield(obstacle)) return false;
    if (respectBoundary && insideAirfieldBoundary(obstacle)) return false;
    if (!grid.fits(north, east, radius)) return false;
    grid.add(north, east, radius);
    bucket.placements.push(makePlacement(north, east, heading, scale));
    return true;
  }

  // --- The buildings of the airfield, from the fixed site plan ---

  const buildingIndexByName = new Map<string, number>();
  buildingTypes.forEach((type, index) => {
    if (!buildingIndexByName.has(type.name)) buildingIndexByName.set(type.name, index);
  });

  let buildingsPlaced = 0;
  for (const site of AIRFIELD_SITES) {
    if (buildingsPlaced >= config.world.buildingCount) break;
    const index = buildingIndexByName.get(site.type);
    if (index === undefined) continue;
    const north = site.north + rng.range(-SITE_JITTER, SITE_JITTER);
    const east = site.east + rng.range(-SITE_JITTER, SITE_JITTER);
    const heading =
      (site.headingDeg + rng.range(-SITE_HEADING_JITTER, SITE_HEADING_JITTER)) * DEG;
    const scale = rng.range(BUILDING_SCALE_LOW, BUILDING_SCALE_HIGH);
    // The site plan works inside the boundary, because these are the buildings
    // of the airfield itself.
    if (tryPlace(buildingBuckets[index], north, east, heading, scale, false)) {
      buildingsPlaced += 1;
    }
  }

  // --- The buildings of the countryside, in villages and lone farms ---

  const villageIndices = VILLAGE_TYPES.map((name) => buildingIndexByName.get(name)).filter(
    (index): index is number => index !== undefined,
  );

  if (villageIndices.length > 0) {
    const remaining = config.world.buildingCount - buildingsPlaced;
    const averageSize = (VILLAGE_MIN_SIZE + VILLAGE_MAX_SIZE) / 2;
    const villageCount = Math.ceil((remaining / averageSize) * CLUSTER_SURPLUS);

    for (let v = 0; v < villageCount; v += 1) {
      if (buildingsPlaced >= config.world.buildingCount) break;

      // A village center, spread over the countryside with no radial bias.
      // Buildings follow roads, and roads run everywhere.
      const angle = rng.angle();
      const radius = Math.sqrt(
        rng.range(VILLAGE_MIN_RADIUS ** 2, VILLAGE_MAX_RADIUS ** 2),
      );
      const centerNorth = radius * Math.cos(angle);
      const centerEast = radius * Math.sin(angle);
      // Every building of one village faces the same way, as they would along
      // one street.
      const street = rng.angle();
      const size = rng.int(VILLAGE_MIN_SIZE, VILLAGE_MAX_SIZE + 1);

      for (let i = 0; i < size; i += 1) {
        if (buildingsPlaced >= config.world.buildingCount) break;
        const bucket = buildingBuckets[rng.pick(villageIndices)];
        const scale = rng.range(BUILDING_SCALE_LOW, BUILDING_SCALE_HIGH);
        const heading = street + rng.range(-0.12, 0.12);
        for (let attempt = 0; attempt < PLACEMENT_TRIES; attempt += 1) {
          const north = centerNorth + rng.gaussian() * VILLAGE_SPREAD;
          const east = centerEast + rng.gaussian() * VILLAGE_SPREAD;
          if (tryPlace(bucket, north, east, heading, scale, true)) {
            buildingsPlaced += 1;
            break;
          }
        }
      }
    }
  }

  // --- The trees, in groves ---

  const shapeGroups = groupByShape(treeTypes.map((type) => type.name));

  if (shapeGroups.length > 0) {
    const averageSize = (GROVE_MIN_SIZE + GROVE_MAX_SIZE) / 2;
    const groveCount = Math.ceil((config.world.treeCount / averageSize) * CLUSTER_SURPLUS);
    let treesPlaced = 0;

    for (let g = 0; g < groveCount; g += 1) {
      if (treesPlaced >= config.world.treeCount) break;

      // The radius follows the cube root of a flat draw. The count of groves in
      // a ring then grows as the square of the radius, while the area of the
      // ring only grows as the radius, so the density of groves grows in
      // proportion to the radius. The wood thickens toward the edge of the
      // world, which is rule 3.
      const angle = rng.angle();
      const radius =
        GROVE_MIN_RADIUS + (GROVE_MAX_RADIUS - GROVE_MIN_RADIUS) * Math.cbrt(rng.next());
      const centerNorth = radius * Math.cos(angle);
      const centerEast = radius * Math.sin(angle);

      const dominant = shapeGroups[rng.int(0, shapeGroups.length)];
      const spread = rng.range(GROVE_MIN_SPREAD, GROVE_MAX_SPREAD);
      const size = rng.int(GROVE_MIN_SIZE, GROVE_MAX_SIZE + 1);

      for (let i = 0; i < size; i += 1) {
        if (treesPlaced >= config.world.treeCount) break;
        // Most of a grove is its own kind. The rest keeps it from looking
        // planted by a machine.
        const group = rng.next() < 0.72 ? dominant : shapeGroups[rng.int(0, shapeGroups.length)];
        const bucket = treeBuckets[pickSize(rng, group)];
        const scale = rng.range(TREE_SCALE_LOW, TREE_SCALE_HIGH);
        const heading = rng.angle();
        for (let attempt = 0; attempt < PLACEMENT_TRIES; attempt += 1) {
          const north = centerNorth + rng.gaussian() * spread;
          const east = centerEast + rng.gaussian() * spread;
          if (tryPlace(bucket, north, east, heading, scale, true)) {
            treesPlaced += 1;
            break;
          }
        }
      }
    }
  }

  return {
    buildings: buildingTypes.map((type, index) => ({
      type,
      placements: buildingBuckets[index].placements,
    })),
    trees: treeTypes.map((type, index) => ({
      type,
      placements: treeBuckets[index].placements,
    })),
  };
}

// ---------------------------------------------------------------------------
// The render half
// ---------------------------------------------------------------------------

export interface ScatteredWorld {
  /** Root of every building and every tree, in the render frame. */
  root: Object3D;

  /** Number of draw calls the last update left, over every group. */
  readonly drawCalls: number;

  /**
   * Sort every instance into its level of detail and move the wind.
   * `cameraPosition` is in the render frame and `time` is the elapsed time of
   * the world, in seconds.
   */
  update(cameraPosition: Vector3, time: number): void;

  dispose(): void;
}

/**
 * Build the models, place them, and put them under one root.
 *
 * The caller must call `update` on every frame, because an instance only takes
 * a level of detail when the update sorts it into one.
 */
export function createScatteredWorld(): ScatteredWorld {
  const buildingTypes: BuildingType[] = createBuildingTypes();
  const treeTypes: TreeType[] = createTreeTypes();
  const result = scatterWorld(buildingTypes, treeTypes);

  const root = new Group();
  root.name = 'scatter';
  const groups: InstancedGroup[] = [];

  for (const bucket of result.buildings) {
    if (bucket.placements.length === 0) continue;
    const group = createInstancedGroup(
      bucket.type.levels,
      bucket.type.material,
      bucket.placements,
      buildingTint,
    );
    group.root.name = `buildings-${bucket.type.name}`;
    groups.push(group);
    root.add(group.root);
  }

  for (const bucket of result.trees) {
    if (bucket.placements.length === 0) continue;
    const group = createInstancedGroup(
      bucket.type.levels,
      bucket.type.material,
      bucket.placements,
      treeTint,
    );
    group.root.name = `trees-${bucket.type.name}`;
    groups.push(group);
    root.add(group.root);
  }

  // Sort the instances one time from the runway threshold, so the first frame
  // is not empty while the caller waits for its first update.
  const start = new Vector3(0, 0, 0);
  for (const group of groups) group.update(start);

  return {
    root,

    get drawCalls(): number {
      let total = 0;
      for (const group of groups) total += group.drawCalls;
      return total;
    },

    update(cameraPosition: Vector3, time: number): void {
      updateTreeWind(time);
      for (const group of groups) group.update(cameraPosition);
    },

    dispose(): void {
      for (const group of groups) group.dispose();
      disposeBuildingTypes(buildingTypes);
      disposeTreeTypes(treeTypes);
      root.removeFromParent();
      root.clear();
    },
  };
}
