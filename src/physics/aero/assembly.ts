/**
 * The aerodynamic assembly. It sums every strip and every body into one wrench.
 *
 * The assembly owns three things that no single element can own.
 *
 *
 * 1. THE INDUCED ANGLE, PER PARENT SURFACE
 *
 * The downwash of a wing follows the lift of the WHOLE wing, not the lift of one
 * strip. The groups exist for that reason: the left wing and the right wing form
 * the wing group, the two tailplane halves form the tail group, and the fin is
 * its own group. Each group carries its own aspect ratio, its own Oswald
 * efficiency, and its own reference area.
 *
 * The elliptical loading approximation gives
 *
 *   alpha_i = CL / (PI * e * AR)
 *
 * and CL follows the lift, which follows alpha_i. That is one equation in one
 * unknown, and the model must close it.
 *
 * THE CHOICE. This module runs a cheap first pass and then solves the loop in
 * closed form, inside the step. It does not carry the value over from the
 * previous step.
 *
 * The cost. estimateSurfaceLoad works out the local flow of a strip and reads no
 * section table and no stall state. It is about 40 percent of the cost of a full
 * strip evaluation, so the whole model costs about 1.4 evaluations per strip per
 * call. At thirty strips, four Runge-Kutta stages, and 240 Hz that is about
 * 40000 estimate calls per second, which is a few hundred microseconds.
 *
 * What that cost buys. First, accuracy: the estimate is the same Kirchhoff law
 * that the section tables hold, so in attached flow the solve is exact and the
 * wing reaches its finite span lift curve slope with no iteration and no lag.
 * Second, and more important, correctness under RK4. stepRK4 in rigidbody.ts
 * calls the wrench source four times per step, at four DIFFERENT stage states.
 * A value carried over from the previous call would come from another stage with
 * another velocity, which breaks the derivative that RK4 needs and makes the
 * model depend on the internals of the integrator. The closed form solve keeps
 * evaluate a function of the state that reaches it.
 *
 * What it costs in accuracy. The estimate is linear in the section, so above the
 * flat plate blend it runs high and the induced angle comes out about one degree
 * high at 30 degrees of angle of attack. Linear induced angle theory has no
 * meaning that far past the stall, so the model accepts it.
 *
 * The closed form. Write the group lift as L(alpha_i) = L0 - K alpha_i, which
 * estimateSurfaceLoad reports term by term. Then
 *
 *   alpha_i = CL / (PI e AR)  with  CL = (L0 - K alpha_i) / (q S)
 *
 * which rearranges to
 *
 *   alpha_i = L0 / (q S PI e AR + K)
 *
 * with no iteration at all.
 *
 *
 * 1a. THE STATE THIS MODULE READS, AND THE FIXED POINT. BEAD b61
 *
 * The estimate above needs the lift curve slope of every strip, and that slope
 * carries the SEPARATION POINT of the strip. The separation point is a state
 * variable: src/physics/aero/stall.ts lags it toward its steady value, which is
 * the dynamic stall of the model and is correct physics.
 *
 * THE DEFECT THIS DESIGN REPLACES. estimateSurfaceLoad used to read that state
 * out of the strip. The value it found was the one the PREVIOUS call of evaluate
 * had left behind, from another state, another stage of the integrator, or
 * another candidate of the trim solver. evaluate was therefore not a function of
 * the arguments it received. The same state gave two answers that differed by a
 * factor of four near the stall, and the trim solver of src/aircraft/trim.ts had
 * to iterate evaluate to its own fixed point to get a Jacobian that meant
 * anything.
 *
 * WHAT evaluate IS NOW. Write x for the separation point of every strip at the
 * moment of the call, u for the rigid body state, the wind and the controls, and
 * dt for the step. The full pass lags x over dt and builds the forces from the
 * lagged value, so both the induced angle solve and the full pass must use
 *
 *   x_lagged = lag(x, dt, alphaTable(x_lagged))
 *
 * which is one equation in one unknown, exactly like the induced angle itself.
 * evaluate iterates the pair to their common fixed point, so the wrench it
 * returns is a function of (u, x, dt) and of nothing else. Two calls with the
 * same u and dt = 0 return the same wrench to the last bit, because dt = 0
 * leaves x where it is.
 *
 * The cost. In attached flow the separation point is already 1 and the first
 * pass moves it by nothing, so the loop stops after one full pass and the model
 * costs what it always did. Near the stall it takes two or three.
 *
 * HOW A CALLER CONTROLS x.
 *
 *   dt = 0                 x does not move. evaluate is then a pure function of
 *                          the state it receives, which is what the four stages
 *                          of a Runge-Kutta step need.
 *   evaluateSteady         Drives every lag to the steady value of THIS state
 *                          inside the call, so the answer does not depend on x at
 *                          all. A trim solver and a static sweep want this one.
 *   reset                  Puts x back to fully attached flow. A respawn and a
 *                          recovery from a state that is not finite need it.
 *
 *
 * 2. THE FREE STREAM
 *
 * The assembly reads the standard atmosphere at the altitude of the aircraft and
 * hands the density and the speed of sound to every element. It reports the free
 * stream angle of attack, the sideslip, the Mach number, and the wind axis lift,
 * drag, and side force for the tests and for the debug panel.
 *
 *
 * 3. THE ORDER OF THE ELEMENTS
 *
 * src/physics/aero/downwash.ts owns the downwash at the tail and the sidewash at
 * the fin. This file marks the one place where those angles enter, between the
 * induced angle solve and the full pass, and implements none of the model.
 *
 * This module is pure physics. It imports the Three.js core math classes only.
 */

import { Vector3 } from 'three';

import { clamp } from '@/math/tables';
import type { AtmosphereSample } from '@/physics/atmosphere';
import { createAtmosphereSample, dynamicPressure, isa, machNumber } from '@/physics/atmosphere';
import type { Body, BodyDef } from '@/physics/aero/body';
import { createBody, evaluateBody } from '@/physics/aero/body';
import type { Downwash } from '@/physics/aero/downwash';
import {
  applyDownwash,
  createDownwash,
  downwashParams,
  resetDownwash,
} from '@/physics/aero/downwash';
import type { Surface, SurfaceDef, SurfaceLoad, SurfaceResult } from '@/physics/aero/surface';
import {
  createSurface,
  estimateSurfaceLoad,
  evaluateSurface,
  resetSurface,
} from '@/physics/aero/surface';
import type { FlowAngles, RigidBodyState, Wrench } from '@/physics/rigidbody';
import { addWrench, clearWrench, createWrench, flowAngles, worldToBody } from '@/physics/rigidbody';

/** One parent surface. The induced angle is worked out over the whole group. */
export interface GroupDef {
  name: string;
  /** Indices into the surface array that createAssembly received. */
  surfaceIndices: readonly number[];
  /** Aspect ratio of the whole parent surface, both halves together. */
  aspectRatio: number;
  oswaldEfficiency: number;
  /** Reference area of the whole parent surface, square meters. */
  area: number;
}

/** What one evaluation of the whole aircraft produced. */
export interface AeroTotals {
  /** rad, free stream, from the airspeed vector in body axes. */
  alpha: number;
  /** rad, free stream. */
  beta: number;
  mach: number;
  /** Pa, free stream. */
  dynamicPressure: number;
  /** m/s. */
  trueAirspeed: number;
  /** Newtons, wind axes. Positive lift points up. */
  lift: number;
  /** Newtons, wind axes. Positive drag opposes the motion. */
  drag: number;
  /** Newtons, wind axes. Positive side force points right. */
  sideForce: number;
  perSurface: readonly SurfaceResult[];
}

/**
 * One element, as the debug view wants it.
 *
 * The shape matches ElementForceSample in src/render/force-arrows.ts field for
 * field. This file cannot import that type, because CONVENTIONS section 4 stops
 * the physics from importing the renderer. Keep the two shapes together by hand.
 */
export interface ElementForceSample {
  /** Body axes, meters from the center of gravity. */
  position: Vector3;
  /** Body axes, newtons. */
  force: Vector3;
  /** Local angle of attack, rad. */
  alpha: number;
  /** Local stall angle, rad, on the positive side. An open slat raises it. */
  stallAlpha: number;
  name: string;
}

export interface AeroAssembly {
  readonly surfaces: readonly Surface[];
  readonly bodies: readonly Body[];
  readonly groups: readonly GroupDef[];
  /** The downwash at the tail and the sidewash at the fin. Bead b18 owns it. */
  readonly downwash: Downwash;
  /**
   * Adds the aerodynamic force and moment of the whole aircraft into out, in
   * body axes about the center of gravity, and returns the totals.
   *
   * The function adds into out, so the caller can put gravity, thrust, and the
   * gear into the same wrench. It allocates nothing.
   */
  evaluate(
    state: RigidBodyState,
    wind: Vector3,
    controls: Float64Array,
    dt: number,
    out: Wrench,
  ): AeroTotals;
  /**
   * The same evaluation with every lag driven to the steady value of this state.
   *
   * The separation lag of src/physics/aero/stall.ts and the downwash lag of
   * src/physics/aero/downwash.ts both use the exact solution of a first order
   * system, so a step of Infinity reaches the steady value exactly and carries
   * nothing over from the call before it. The answer is therefore a function of
   * the arguments alone, which is what a trim solver and a static sweep need.
   *
   * The call LEAVES the lag states at that steady value. See section 1a.
   */
  evaluateSteady(
    state: RigidBodyState,
    wind: Vector3,
    controls: Float64Array,
    out: Wrench,
  ): AeroTotals;
  /**
   * Puts every lag state back to the value createAssembly left it in: attached
   * flow on every strip and no downwash. See section 1a.
   */
  reset(): void;
  /** Returns the same array and the same vectors on every call. Surfaces only. */
  sampleForDebug(): readonly ElementForceSample[];
}

// The largest induced angle the model accepts, in radians. Twenty degrees of
// downwash is already far past anything a real wing makes. The clamp only ever
// acts when the linear estimate runs away at a very low dynamic pressure.
const MAX_INDUCED_ANGLE = 0.35; // rad

// Below this dynamic pressure the induced angle has no meaning and reports zero.
const MIN_SOLVE_PRESSURE = 1e-9;

/**
 * Passes of the separation fixed point of section 1a, and the movement of the
 * separation point that ends the loop.
 *
 * The separation point runs from 1 to 0.04, so a tolerance of 1e-9 is one part
 * in a thousand million of its range.
 *
 * Measured, at the 240 Hz flight step. Settled cruise takes ONE pass on every
 * step, because the separation point is already where the lag wants it and the
 * first pass moves it by nothing. A snap pull from 2 degrees to 20 degrees
 * averages 3.5 passes and peaks at 6. The same pull to 30 degrees averages 4.4
 * and peaks at 7. The cost of the model in cruise is unchanged, at 22
 * microseconds per call.
 */
const MAX_SEPARATION_PASSES = 8;
const SEPARATION_TOLERANCE = 1e-9;

/** Builds the whole assembly. Every allocation of the model happens here. */
export function createAssembly(
  surfaceDefs: SurfaceDef[],
  bodyDefs: BodyDef[],
  groupDefs: GroupDef[],
): AeroAssembly {
  const surfaces: Surface[] = surfaceDefs.map(createSurface);
  const bodies: Body[] = bodyDefs.map(createBody);
  const groups: GroupDef[] = groupDefs.slice();

  for (const group of groups) {
    if (!(group.area > 0) || !(group.aspectRatio > 0) || !(group.oswaldEfficiency > 0)) {
      throw new Error(
        `Group ${group.name} needs a positive area, aspect ratio and Oswald ` +
          `efficiency. It got ${group.area}, ${group.aspectRatio} and ` +
          `${group.oswaldEfficiency}.`,
      );
    }
    for (const index of group.surfaceIndices) {
      if (!Number.isInteger(index) || index < 0 || index >= surfaces.length) {
        throw new Error(
          `Group ${group.name} names surface index ${index}, and the assembly ` +
            `holds ${surfaces.length} surfaces.`,
        );
      }
    }
  }

  // Every strip that no group claims. Such a strip uses the aspect ratio and the
  // Oswald efficiency of its own def, so a single surface still gets a finite
  // span correction when a caller evaluates it on its own.
  const grouped = new Uint8Array(surfaces.length);
  for (const group of groups) {
    for (const index of group.surfaceIndices) {
      grouped[index] = 1;
    }
  }
  const ungrouped: number[] = [];
  for (let i = 0; i < surfaces.length; i++) {
    if (grouped[i] === 0) {
      ungrouped.push(i);
    }
  }

  const inducedAngles = new Float64Array(surfaces.length);
  // The separation point of every strip at the moment of the call, and the
  // lagged value the passes of section 1a work with. Both are written before
  // they are read, so their starting contents never reach a result.
  const entrySeparation = new Float64Array(surfaces.length);
  const passSeparation = new Float64Array(surfaces.length);
  // Bead b18. The downwash reads the roles of the groups out of their geometry.
  const downwash = createDownwash(downwashParams(surfaces, groups));
  const load: SurfaceLoad = { lift: 0, slope: 0 };
  // One reusable index array for a strip that belongs to no group.
  const singleIndex: number[] = [0];
  // The angular velocity of the state that evaluate received. solveInduced reads
  // it here, which keeps that call free of one more parameter.
  let angularVelocity = new Vector3();
  const atmosphere: AtmosphereSample = createAtmosphereSample();
  const flow: FlowAngles = { alpha: 0, beta: 0, speed: 0 };
  const total: Wrench = createWrench();
  const velocityBody = new Vector3();
  const windBody = new Vector3();
  const airspeed = new Vector3();

  const perSurface: SurfaceResult[] = surfaces.map((s) => s.result);
  const totals: AeroTotals = {
    alpha: 0,
    beta: 0,
    mach: 0,
    dynamicPressure: 0,
    trueAirspeed: 0,
    lift: 0,
    drag: 0,
    sideForce: 0,
    perSurface,
  };

  const debugSamples: ElementForceSample[] = surfaces.map((s) => ({
    position: s.def.position.clone(),
    force: new Vector3(),
    alpha: 0,
    stallAlpha: s.def.airfoil.alphaStall,
    name: s.def.name,
  }));

  /** Solves the induced angle of one set of strips and writes it into every one. */
  function solveInduced(
    indices: readonly number[],
    area: number,
    aspectRatio: number,
    oswald: number,
    freeStreamPressure: number,
    controls: Float64Array,
  ): void {
    let lift = 0;
    let slope = 0;
    for (const index of indices) {
      estimateSurfaceLoad(
        surfaces[index],
        velocityBody,
        angularVelocity,
        windBody,
        atmosphere.density,
        atmosphere.speedOfSound,
        controls,
        passSeparation[index],
        load,
      );
      lift += load.lift;
      slope += load.slope;
    }
    const denominator = freeStreamPressure * area * Math.PI * oswald * aspectRatio + slope;
    const angle =
      denominator > MIN_SOLVE_PRESSURE
        ? clamp(lift / denominator, -MAX_INDUCED_ANGLE, MAX_INDUCED_ANGLE)
        : 0;
    for (const index of indices) {
      inducedAngles[index] = angle;
    }
  }

  const api: AeroAssembly = {
    surfaces,
    bodies,
    groups,
    downwash,

    evaluate(
      state: RigidBodyState,
      wind: Vector3,
      controls: Float64Array,
      dt: number,
      out: Wrench,
    ): AeroTotals {
      angularVelocity = state.angularVelocity;

      // The elements take the body velocity and the body wind on their own,
      // because each one adds omega x r before it subtracts the wind.
      worldToBody(state.orientation, state.velocity, velocityBody);
      worldToBody(state.orientation, wind, windBody);
      airspeed.copy(velocityBody).sub(windBody);

      // CONVENTIONS section 3.2. The altitude is the negative of the world z.
      isa(-state.position.z, atmosphere);

      flowAngles(airspeed, flow);
      const speed = flow.speed;
      const freeStreamPressure = dynamicPressure(atmosphere.density, speed);
      // The downwash model reads it. Both transonic laws of section 5 of
      // src/physics/aero/downwash.ts answer the shock the WING meets.
      const freeStreamMach = machNumber(speed, atmosphere.speedOfSound);

      // SECTION 1a. The separation point of every strip at the moment of the
      // call. Every pass below starts again from this value, so the loop moves
      // toward the fixed point of one lag over one dt and never over two.
      for (let i = 0; i < surfaces.length; i++) {
        entrySeparation[i] = surfaces[i].state.stall.f;
        passSeparation[i] = entrySeparation[i];
      }
      const entryEpsilon = downwash.state.laggedEpsilon;

      for (let pass = 0; pass < MAX_SEPARATION_PASSES; pass++) {
        for (let i = 0; i < surfaces.length; i++) {
          surfaces[i].state.stall.f = entrySeparation[i];
        }
        downwash.state.laggedEpsilon = entryEpsilon;

        // Pass one. The induced angle of every parent surface.
        inducedAngles.fill(0);
        for (const group of groups) {
          solveInduced(
            group.surfaceIndices,
            group.area,
            group.aspectRatio,
            group.oswaldEfficiency,
            freeStreamPressure,
            controls,
          );
        }
        for (const index of ungrouped) {
          const def = surfaces[index].def;
          singleIndex[0] = index;
          solveInduced(
            singleIndex,
            def.area,
            def.aspectRatio,
            def.oswaldEfficiency,
            freeStreamPressure,
            controls,
          );
        }

        // DOWNWASH AND SIDEWASH, BEAD b18.
        //
        // The tail flies in the wake of the wing and the fin flies in the
        // sidewash of the wing and the fuselage. Both change the angle that
        // those surfaces meet. src/physics/aero/downwash.ts owns that model. It
        // adds its angle into inducedAngles here, after the induced angle solve
        // and before the full pass below, because the two angles add.
        applyDownwash(
          downwash,
          surfaces,
          passSeparation,
          inducedAngles,
          flow.alpha,
          flow.beta,
          speed,
          freeStreamMach,
          freeStreamPressure,
          dt,
        );

        // Pass two. Every element, with its induced angle. evaluateSurface lags
        // the separation point of every strip over dt and writes it back.
        clearWrench(total);
        for (let i = 0; i < surfaces.length; i++) {
          evaluateSurface(
            surfaces[i],
            velocityBody,
            state.angularVelocity,
            windBody,
            atmosphere.density,
            atmosphere.speedOfSound,
            controls,
            inducedAngles[i],
            dt,
            total,
          );
        }

        // How far the separation point the full pass produced sits from the one
        // the induced angle solve assumed. Zero means the two agree and the pass
        // is the fixed point of section 1a.
        let moved = 0;
        for (let i = 0; i < surfaces.length; i++) {
          const produced = surfaces[i].state.stall.f;
          const step = Math.abs(produced - passSeparation[i]);
          if (step > moved) {
            moved = step;
          }
          passSeparation[i] = produced;
        }
        if (moved <= SEPARATION_TOLERANCE) {
          break;
        }
      }

      for (let i = 0; i < bodies.length; i++) {
        evaluateBody(
          bodies[i],
          velocityBody,
          state.angularVelocity,
          windBody,
          atmosphere.density,
          atmosphere.speedOfSound,
          total,
        );
      }

      // The wind axis triad. CONVENTIONS section 3.1 fixes the body axes, and
      // these three rows are the standard map from them into wind axes. The
      // lift comes out positive when the force points up, that is toward
      // negative body z.
      const ca = Math.cos(flow.alpha);
      const sa = Math.sin(flow.alpha);
      const cb = Math.cos(flow.beta);
      const sb = Math.sin(flow.beta);
      const f = total.force;
      totals.alpha = flow.alpha;
      totals.beta = flow.beta;
      totals.mach = freeStreamMach;
      totals.dynamicPressure = freeStreamPressure;
      totals.trueAirspeed = speed;
      totals.drag = -(f.x * ca * cb + f.y * sb + f.z * sa * cb);
      totals.sideForce = -f.x * ca * sb + f.y * cb - f.z * sa * sb;
      totals.lift = -(-f.x * sa + f.z * ca);

      addWrench(out, total);
      return totals;
    },

    evaluateSteady(
      state: RigidBodyState,
      wind: Vector3,
      controls: Float64Array,
      out: Wrench,
    ): AeroTotals {
      // A step of Infinity drives the exact first order lag of stall.ts and of
      // downwash.ts to its steady value in one pass, so nothing of the call
      // before this one survives into the answer. See section 1a.
      return api.evaluate(state, wind, controls, Number.POSITIVE_INFINITY, out);
    },

    reset(): void {
      for (const surface of surfaces) {
        resetSurface(surface);
      }
      resetDownwash(downwash);
    },

    sampleForDebug(): readonly ElementForceSample[] {
      for (let i = 0; i < surfaces.length; i++) {
        const s = surfaces[i];
        const sample = debugSamples[i];
        sample.force.copy(s.result.force);
        sample.alpha = s.result.alpha;
        sample.stallAlpha =
          s.def.airfoil.alphaStall + (s.result.slatOpen ? s.def.slatAlphaDelta : 0);
      }
      return debugSamples;
    },
  };

  return api;
}
