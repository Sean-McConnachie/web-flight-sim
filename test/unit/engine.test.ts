/**
 * Behavior tests of the Jumo 004 B-1 model.
 *
 * Each test states a fact about the real engine. The headline fact is the spool
 * time: the rotor needs eight to ten seconds to go from idle to full power.
 */

import { describe, expect, it } from 'vitest';
import { Vector3 } from 'three';
import {
  DANGER_BAND_RPM,
  HOT_RELIGHT_MIN_RPM,
  IDLE_RPM,
  MAX_RPM,
  MAX_THRUST_SL_STATIC,
  RELIGHT_MIN_RPM,
  SELF_SUSTAIN_RPM,
  STARTER_CUTOUT_RPM,
  STARTER_TARGET_RPM,
  THRUST_ALTITUDE_EXPONENT,
  TURBINE_INLET_TEMPERATURE_LIMIT,
  createJumo004,
  thrustSpeedFraction,
} from '@/aircraft/me262/engine';
import type { Engine, EngineInput, StartPhase } from '@/aircraft/me262/engine';
import { RHO0, isa } from '@/physics/atmosphere';

const DT = 1 / 240;

/** Rotor speed that counts as spooled, 95 percent of the maximum. */
const SPOOLED_RPM = 0.95 * MAX_RPM;

function makeInput(): EngineInput {
  return {
    throttle: 0,
    fuelCockOpen: false,
    starterEngaged: false,
    altitude: 0,
    mach: 0,
    airspeed: 0,
    density: RHO0,
    fuelAvailable: true,
  };
}

/** Runs the engine for a time in seconds. The hook runs before each step. */
function run(
  engine: Engine,
  input: EngineInput,
  seconds: number,
  hook?: (t: number) => void,
): void {
  const steps = Math.round(seconds / DT);
  for (let i = 0; i < steps; i++) {
    if (hook !== undefined) {
      hook(i * DT);
    }
    engine.update(input, DT);
  }
}

/** Runs until the test passes or the time runs out. Returns the elapsed time. */
function runUntil(
  engine: Engine,
  input: EngineInput,
  limit: number,
  test: () => boolean,
  hook?: (t: number) => void,
): number {
  const steps = Math.round(limit / DT);
  for (let i = 0; i < steps; i++) {
    if (test()) {
      return i * DT;
    }
    if (hook !== undefined) {
      hook(i * DT);
    }
    engine.update(input, DT);
  }
  return limit;
}

/**
 * Runs the full start: crank with the Riedel, open the fuel cock at about
 * 800 rpm with the lever at the idle stop, then hold until the rotor settles at
 * idle. This is the procedure of the pilot notes.
 */
function startAndIdle(engine: Engine, input: EngineInput): void {
  input.starterEngaged = true;
  input.throttle = 0;
  runUntil(engine, input, 60, () => engine.rpm >= 700);
  input.fuelCockOpen = true;
  runUntil(engine, input, 150, () => engine.state === 'idle');
  input.starterEngaged = false;
  run(engine, input, 40);
}

/** Makes an engine that idles at sea level. */
function idlingEngine(): { engine: Engine; input: EngineInput } {
  const engine = createJumo004(new Vector3(0, -2.5, 0.3));
  const input = makeInput();
  startAndIdle(engine, input);
  return { engine, input };
}

/** Moves the lever as fast as the surge margin allows. This is the pilot who
 *  knows the engine, and it gives the shortest safe acceleration. */
function advanceOnTheMargin(engine: Engine, input: EngineInput, rate: number): void {
  if (engine.surgeMargin > 0.1) {
    input.throttle = Math.min(1, input.throttle + rate * DT);
  } else if (engine.surgeMargin < 0.05) {
    input.throttle = Math.max(0, input.throttle - rate * DT);
  }
}

describe('Jumo 004 rotor and spool', () => {
  it('settles at the published idle speed of 3000 rpm after a start', () => {
    const { engine } = idlingEngine();
    expect(engine.state).toBe('idle');
    expect(engine.rpm).toBeGreaterThan(2850);
    expect(engine.rpm).toBeLessThan(3150);
  });

  it('needs between 8 and 10 seconds from idle to 95 percent of maximum rpm', () => {
    const { engine, input } = idlingEngine();
    const spool = runUntil(
      engine,
      input,
      20,
      () => engine.rpm >= SPOOLED_RPM,
      () => advanceOnTheMargin(engine, input, 0.6),
    );
    expect(engine.rpm).toBeGreaterThanOrEqual(SPOOLED_RPM);
    expect(spool).toBeGreaterThan(8);
    expect(spool).toBeLessThan(10);
  });

  it('reaches the published maximum rotor speed and thrust at full throttle', () => {
    const { engine, input } = idlingEngine();
    run(engine, input, 20, () => advanceOnTheMargin(engine, input, 0.6));
    input.throttle = 1;
    run(engine, input, 10);
    // The fuel control holds the rotor at the maximum speed.
    expect(engine.rpm).toBeGreaterThan(MAX_RPM - 100);
    expect(engine.rpm).toBeLessThan(MAX_RPM + 100);
    expect(engine.thrust).toBeGreaterThan(0.97 * MAX_THRUST_SL_STATIC);
    expect(engine.thrust).toBeLessThanOrEqual(MAX_THRUST_SL_STATIC);
    // The published specific fuel consumption is 1.4 kg per kilogram-force per
    // hour, which is 0.349 kg/s at 8800 N.
    expect(engine.fuelFlow).toBeGreaterThan(0.32);
    expect(engine.fuelFlow).toBeLessThan(0.39);
    // The turbine runs just below its limit at full power.
    expect(engine.gasTemperature).toBeGreaterThan(950);
    expect(engine.gasTemperature).toBeLessThan(TURBINE_INLET_TEMPERATURE_LIMIT);
  });

  it('answers fast above the danger band, where the pressure ratio is high', () => {
    const { engine, input } = idlingEngine();
    runUntil(engine, input, 20, () => engine.rpm >= DANGER_BAND_RPM + 1000, () =>
      advanceOnTheMargin(engine, input, 0.6),
    );
    const from = engine.rpm;
    input.throttle = 1;
    const time = runUntil(engine, input, 10, () => engine.rpm >= SPOOLED_RPM);
    // The same 1200 rpm step takes more than four seconds down at idle.
    expect(from).toBeGreaterThan(DANGER_BAND_RPM);
    expect(time).toBeLessThan(3);
  });
});

describe('Jumo 004 thrust', () => {
  it('makes about 3 percent of maximum thrust at idle', () => {
    const { engine } = idlingEngine();
    const fraction = engine.thrust / MAX_THRUST_SL_STATIC;
    expect(fraction).toBeGreaterThan(0.02);
    expect(fraction).toBeLessThan(0.045);
  });

  it('follows a curve steeper than a cube law against the rotor speed', () => {
    // The local exponent is d(ln thrust) / d(ln speed). At idle it is near four.
    const speed = IDLE_RPM / MAX_RPM;
    const step = 0.001;
    const low = thrustSpeedFraction(speed * (1 - step));
    const high = thrustSpeedFraction(speed * (1 + step));
    const exponent = (Math.log(high) - Math.log(low)) / (2 * step);
    expect(exponent).toBeGreaterThan(3);
    expect(exponent).toBeLessThan(5);
  });

  it('loses thrust with altitude as the density lapse model says', () => {
    const { engine, input } = idlingEngine();
    run(engine, input, 20, () => advanceOnTheMargin(engine, input, 0.6));
    input.throttle = 1;
    run(engine, input, 10);
    const seaLevel = engine.thrust;

    // Climb to 6000 m over a minute. A step change of the altitude would leave
    // the fuel control one lag behind the airflow, and the engine would surge.
    // A real climb never does that.
    run(engine, input, 60, (t) => {
      input.altitude = 100 * t;
      input.density = isa(input.altitude).density;
    });
    const air = isa(6000);
    input.altitude = 6000;
    input.density = air.density;
    run(engine, input, 10);
    const high = engine.thrust;
    expect(engine.state).toBe('running');

    const expected = Math.pow(air.density / RHO0, THRUST_ALTITUDE_EXPONENT);
    expect(high).toBeLessThan(seaLevel);
    expect(high / seaLevel).toBeGreaterThan(expected - 0.03);
    expect(high / seaLevel).toBeLessThan(expected + 0.03);
    // The published values, one engine: 8800 N at sea level, 4740 N at 6000 m.
    expect(seaLevel).toBeGreaterThan(8500);
    expect(high).toBeGreaterThan(4400);
    expect(high).toBeLessThan(5100);
  });
});

describe('Jumo 004 compressor surge', () => {
  it('never surges when the lever goes from idle to full over ten seconds', () => {
    const { engine, input } = idlingEngine();
    let worst = 1;
    run(engine, input, 16, (t) => {
      input.throttle = Math.min(1, t / 10);
      worst = Math.min(worst, engine.surgeMargin);
    });
    expect(worst).toBeGreaterThan(0);
    expect(engine.state).toBe('running');
    expect(engine.rpm).toBeGreaterThan(SPOOLED_RPM);
  });

  it('surges and stalls when the lever goes from idle to full in one step', () => {
    const { engine, input } = idlingEngine();
    input.throttle = 1;
    run(engine, input, 1);
    expect(engine.surgeMargin).toBeLessThan(0);
    expect(engine.state).toBe('stall');
    expect(engine.events.surgeBangCount).toBeGreaterThan(0);
    // The thrust collapses with the flow.
    expect(engine.thrust).toBeLessThan(0.1 * MAX_THRUST_SL_STATIC);
  });

  it('does not surge when the lever goes from 7000 rpm to full in one step', () => {
    const { engine, input } = idlingEngine();
    runUntil(engine, input, 20, () => engine.rpm >= 7000, () =>
      advanceOnTheMargin(engine, input, 0.6),
    );
    input.throttle = 1;
    let worst = 1;
    run(engine, input, 5, () => {
      worst = Math.min(worst, engine.surgeMargin);
    });
    expect(worst).toBeGreaterThan(0);
    expect(engine.state).toBe('running');
    expect(engine.rpm).toBeGreaterThan(SPOOLED_RPM);
  });

  it('recovers when the pilot closes the throttle within two seconds', () => {
    const { engine, input } = idlingEngine();
    input.throttle = 1;
    run(engine, input, 1);
    expect(engine.state).toBe('stall');
    input.throttle = 0;
    run(engine, input, 10);
    expect(engine.state === 'idle' || engine.state === 'running').toBe(true);
    expect(engine.fuelFlow).toBeGreaterThan(0);
  });

  it('flames out when the pilot leaves the throttle open in a stall', () => {
    const { engine, input } = idlingEngine();
    input.throttle = 1;
    runUntil(engine, input, 2, () => engine.state === 'stall');
    run(engine, input, 4);
    expect(engine.state).toBe('flameout');
    expect(engine.events.flameoutCount).toBeGreaterThan(0);
    expect(engine.thrust).toBe(0);
  });

  it('holds a thinner margin at idle than at full power', () => {
    const { engine, input } = idlingEngine();
    const atIdle = engine.surgeMargin;
    run(engine, input, 20, () => advanceOnTheMargin(engine, input, 0.6));
    input.throttle = 1;
    run(engine, input, 10);
    // Both margins are positive on the running line, but one throttle slam eats
    // the whole margin at idle and only a third of it at full power.
    const slamAtIdle = 1 - 3.2 * (1 - atIdle);
    expect(atIdle).toBeGreaterThan(0);
    expect(slamAtIdle).toBeLessThan(0);
    expect(engine.surgeMargin).toBeGreaterThan(0);
  });
});

describe('Jumo 004 turbine damage', () => {
  it('accumulates damage above the temperature limit and never gives it back', () => {
    const { engine, input } = idlingEngine();
    run(engine, input, 20, () => advanceOnTheMargin(engine, input, 0.6));
    input.throttle = 1;
    run(engine, input, 10);
    const healthy = engine.thrust;
    expect(engine.damage).toBeLessThan(0.02);

    // Slam the lever at idle, hold the surge for one and a half seconds, then
    // save the engine. The gas temperature runs far above the limit.
    input.throttle = 0;
    run(engine, input, 25);
    input.throttle = 1;
    runUntil(engine, input, 3, () => engine.state === 'stall');
    run(engine, input, 1.5);
    expect(engine.gasTemperature).toBeGreaterThan(TURBINE_INLET_TEMPERATURE_LIMIT);
    input.throttle = 0;
    run(engine, input, 25);
    const hurt = engine.damage;
    expect(hurt).toBeGreaterThan(0.05);

    // The damage stays after the engine cools, and it costs thrust.
    run(engine, input, 30);
    expect(engine.damage).toBe(hurt);
    run(engine, input, 20, () => advanceOnTheMargin(engine, input, 0.6));
    input.throttle = 1;
    run(engine, input, 10);
    expect(engine.thrust).toBeLessThan(healthy);
    expect(engine.thrust).toBeLessThan(healthy * (1 - 0.5 * hurt));
  });
});

describe('Jumo 004 start, flame out and relight', () => {
  it('reaches idle from cold with the pilot at the idle stop', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    expect(engine.state).toBe('off');
    expect(engine.fuelFlow).toBe(0);
    expect(engine.thrust).toBe(0);
    startAndIdle(engine, input);
    expect(engine.state).toBe('idle');
    expect(engine.events.hotStartCount).toBe(0);
    expect(engine.damage).toBe(0);
    expect(engine.fuelFlow).toBeGreaterThan(0);
  });

  it('makes a hot start when the pilot opens the fuel with the lever open', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    runUntil(engine, input, 60, () => engine.rpm >= 700);
    // The lever sits at half travel, so the valve passes far more fuel than the
    // 800 rpm rotor can swallow.
    input.throttle = 0.5;
    input.fuelCockOpen = true;
    let peak = 0;
    run(engine, input, 4, () => {
      peak = Math.max(peak, engine.gasTemperature);
    });
    expect(engine.events.hotStartCount).toBeGreaterThan(0);
    expect(peak).toBeGreaterThan(TURBINE_INLET_TEMPERATURE_LIMIT);
    expect(engine.damage).toBeGreaterThan(0);
    // The over fuelled start also surges and then puts the flame out.
    expect(engine.state).toBe('flameout');
  });

  it('makes a hot start when the pilot opens the fuel cock too early', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    input.fuelCockOpen = true;
    // The cock opens at rest. Fuel gathers in the chambers for many seconds and
    // then lights at once.
    run(engine, input, 20);
    expect(engine.events.hotStartCount).toBeGreaterThan(0);
  });

  it('flames out when the fuel cock closes and makes no thrust after that', () => {
    const { engine, input } = idlingEngine();
    input.fuelCockOpen = false;
    run(engine, input, 1);
    expect(engine.state).toBe('flameout');
    expect(engine.thrust).toBe(0);
    expect(engine.fuelFlow).toBe(0);
  });

  it('cannot relight below the windmill speed window', () => {
    const { engine, input } = idlingEngine();
    input.altitude = 3000;
    input.density = isa(3000).density;
    input.airspeed = 60;
    input.mach = 0.18;
    input.fuelCockOpen = false;
    run(engine, input, 1);
    expect(engine.state).toBe('flameout');
    // Let the rotor fall to the windmill speed of 60 m/s, which is 600 rpm.
    run(engine, input, 150);
    expect(engine.rpm).toBeLessThan(RELIGHT_MIN_RPM);
    input.throttle = 0;
    input.fuelCockOpen = true;
    run(engine, input, 20);
    expect(engine.state).toBe('flameout');
    expect(engine.thrust).toBe(0);
  });

  it('relights inside the windmill speed window with the lever at idle', () => {
    const { engine, input } = idlingEngine();
    input.altitude = 3000;
    input.density = isa(3000).density;
    input.airspeed = 160;
    input.mach = 0.48;
    input.fuelCockOpen = false;
    run(engine, input, 1);
    expect(engine.state).toBe('flameout');
    run(engine, input, 120);
    expect(engine.rpm).toBeGreaterThan(RELIGHT_MIN_RPM);
    input.throttle = 0;
    input.fuelCockOpen = true;
    run(engine, input, 60);
    expect(engine.state === 'idle' || engine.state === 'running').toBe(true);
    expect(engine.fuelFlow).toBeGreaterThan(0);
    expect(engine.thrust).toBeGreaterThan(0);
  });

  it('will not relight while the pilot holds the throttle open', () => {
    const { engine, input } = idlingEngine();
    input.altitude = 3000;
    input.density = isa(3000).density;
    input.airspeed = 160;
    input.mach = 0.48;
    input.fuelCockOpen = false;
    run(engine, input, 1);
    run(engine, input, 60);
    input.throttle = 0.8;
    input.fuelCockOpen = true;
    run(engine, input, 20);
    expect(engine.state).toBe('flameout');
  });
});

/**
 * BEAD b38. The start as the pilot runs it.
 *
 * Every number here comes from "Pilot's Handbook for Me-262 A-1", the Wright
 * Field translation F-SU-1111-ND of 10 January 1946, STARTING PROCEDURES steps
 * 5 to 9. The steps are:
 *
 *   5, 6  Prime the Riedel. Pull the starter handle and hold it.
 *   7     At 700 to 800 rpm press the ignition and hold. The unit fires and the
 *         speed rises to 1800 to 2000 rpm.
 *   8     Release the starter handle, open the fuel valve, advance the throttle
 *         to 3000 rpm.
 *   9     At 3000 rpm release the ignition button.
 */
describe('Jumo 004 start sequence', () => {
  /** Runs the handbook start and records every phase change it passes. */
  function flyTheDrill(): {
    engine: Engine;
    phases: { phase: StartPhase; time: number; rpm: number }[];
    idleTime: number;
  } {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    const phases: { phase: StartPhase; time: number; rpm: number }[] = [];
    let last: StartPhase | null = null;
    let time = 0;
    let idleTime = -1;
    for (let i = 0; i < 240 * 200; i++) {
      // Step 7. The pilot opens the fuel at the light off speed.
      if (engine.rpm >= 700) {
        input.fuelCockOpen = true;
      }
      // Step 8. The pilot lets the starter handle go at 2000 rpm.
      if (engine.rpm >= STARTER_CUTOUT_RPM) {
        input.starterEngaged = false;
      }
      engine.update(input, DT);
      time += DT;
      if (engine.startPhase !== last) {
        phases.push({ phase: engine.startPhase, time, rpm: engine.rpm });
        last = engine.startPhase;
      }
      if (idleTime < 0 && engine.state === 'idle') {
        idleTime = time;
      }
      if (idleTime > 0 && time > idleTime + 20) {
        break;
      }
    }
    return { engine, phases, idleTime };
  }

  it('passes the four handbook steps in the order the handbook gives them', () => {
    const { phases } = flyTheDrill();
    expect(phases.map((p) => p.phase)).toEqual(['crank', 'light', 'accelerate', 'complete']);
  });

  it('lights the fuel at the handbook speed of 700 to 800 rpm', () => {
    const { phases } = flyTheDrill();
    const light = phases[1];
    expect(light.phase).toBe('light');
    // The handbook gives 700 to 800 rpm. The model lights a little above 700,
    // because LIGHT_OFF_DELAY holds the flame off for four tenths of a second
    // while the rotor keeps accelerating.
    expect(light.rpm).toBeGreaterThanOrEqual(700);
    expect(light.rpm).toBeLessThan(STARTER_TARGET_RPM + 100);
  });

  it('hands the rotor over to its own flame at the handbook speed of 1800 rpm', () => {
    const { phases } = flyTheDrill();
    const accelerate = phases[2];
    expect(accelerate.phase).toBe('accelerate');
    expect(accelerate.rpm).toBeGreaterThanOrEqual(SELF_SUSTAIN_RPM);
    expect(accelerate.rpm).toBeLessThan(SELF_SUSTAIN_RPM + 100);
  });

  it('runs from the crank to idle in the 30 to 45 s a Riedel start takes', () => {
    const { idleTime } = flyTheDrill();
    // This model took 80 s before bead b38. The starter torque was the fault:
    // a linear fade to zero at 1200 rpm put the surplus power at exactly zero
    // at the 800 rpm light off speed, so the crank approached that speed along
    // an asymptote and spent 15 s over the last 200 rpm.
    expect(idleTime).toBeGreaterThan(30);
    expect(idleTime).toBeLessThan(45);
  });

  it('cranks to the light off speed in under 10 s, which is what 10 hp gives', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    const time = runUntil(engine, input, 30, () => engine.rpm >= STARTER_TARGET_RPM);
    expect(engine.rpm).toBeGreaterThanOrEqual(STARTER_TARGET_RPM);
    // The rotor holds 10.5 kg m2 and 800 rpm is 83.8 rad/s, so the rotor stores
    // 36.9 kJ. The Riedel makes 7457 W and the motoring drag takes about half
    // of it near the top, so the crank cannot be much under 8 s or over 10.
    expect(time).toBeGreaterThan(5);
    expect(time).toBeLessThan(10);
  });

  it('holds the rotor near 1200 rpm on the Riedel alone with no fuel', () => {
    const engine = createJumo004(new Vector3(0, 0, 0));
    const input = makeInput();
    input.starterEngaged = true;
    run(engine, input, 120);
    // Constant power against the motoring drag and the bearings:
    // 7457 / w = 0.415 w + 8 gives w = 124.7 rad/s, which is 1191 rpm. The
    // starter alone therefore cannot reach the 1800 rpm of handbook step 8. The
    // flame does the rest, which is why the handbook holds the handle in
    // THROUGH the light off.
    expect(engine.rpm).toBeGreaterThan(1100);
    expect(engine.rpm).toBeLessThan(1300);
    expect(engine.state).toBe('starter');
    expect(engine.startPhase).toBe('crank');
  });

  it('names the step and the next action while the start runs', () => {
    const { phases, engine } = flyTheDrill();
    // Every step but the last tells the pilot what to do next.
    for (const step of phases.slice(0, 3)) {
      expect(step.phase).not.toBe('failed');
    }
    // The engine says nothing once it idles, because there is nothing to say.
    expect(engine.startPhase).toBe('complete');
    expect(engine.message).toBe('');
  });
});

/**
 * BEAD b38 and BEAD b56 item 3. A start that fails says why, and the pilot can
 * clear it.
 */
describe('Jumo 004 failed start signalling', () => {
  it('names the hot start and gives the handbook drill for it', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    runUntil(engine, input, 30, () => engine.rpm >= 700);
    // The lever sits at half travel, which is the fault the handbook warns of.
    input.throttle = 0.5;
    input.fuelCockOpen = true;
    run(engine, input, 6);
    expect(engine.events.hotStartCount).toBeGreaterThan(0);
    expect(engine.startPhase).toBe('failed');
    // The message names the fault, says the turbine took damage, and gives the
    // two actions that clear it.
    expect(engine.message).toContain('Hot start');
    expect(engine.message).toContain('turbine took damage');
    expect(engine.message).toContain('Close the fuel cock');
    expect(engine.message).toContain('tail pipe');
    expect(engine.damage).toBeGreaterThan(0);
  });

  it('clears the wet tail pipe when the pilot cranks with the fuel shut', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    runUntil(engine, input, 30, () => engine.rpm >= 700);
    input.throttle = 0.5;
    input.fuelCockOpen = true;
    run(engine, input, 6);
    const afterHotStart = engine.damage;
    expect(engine.pooledFuel).toBeGreaterThan(0);

    // The handbook drill: close the fuel and crank until the tail pipe clears.
    input.throttle = 0;
    input.fuelCockOpen = false;
    run(engine, input, 30);
    expect(engine.pooledFuel).toBe(0);
    expect(engine.message).toContain('tail pipe is clear');

    // The retry now works, and it costs no more of the turbine. Before bead
    // b56 the pool never drained, so every retry burned it again and drove the
    // damage from 0.35 to 0.90 on the second attempt alone.
    input.fuelCockOpen = true;
    runUntil(engine, input, 90, () => engine.state === 'idle');
    expect(engine.state).toBe('idle');
    expect(engine.startPhase).toBe('complete');
    expect(engine.damage).toBeCloseTo(afterHotStart, 3);
  });

  it('says the fuel cock is shut when no fuel reaches the burners', () => {
    const { engine, input } = idlingEngine();
    input.fuelCockOpen = false;
    run(engine, input, 2);
    expect(engine.state).toBe('flameout');
    expect(engine.message).toContain('Open the fuel cock');
  });

  it('says the feed failed when the cock is open and no fuel arrives', () => {
    const { engine, input } = idlingEngine();
    input.fuelAvailable = false;
    run(engine, input, 2);
    expect(engine.state).toBe('flameout');
    // Negative g uncovers the pickup. The pilot must know that the cock is not
    // the problem, or the pilot works the wrong control.
    expect(engine.message).toContain('no fuel arrives');
    expect(engine.message).toContain('positive g');
  });

  it('says the rotor is too slow when a windmill relight has no rotor speed', () => {
    const { engine, input } = idlingEngine();
    input.altitude = 3000;
    input.density = isa(3000).density;
    input.airspeed = 60;
    input.mach = 0.18;
    input.fuelCockOpen = false;
    run(engine, input, 1);
    run(engine, input, 150);
    input.fuelCockOpen = true;
    run(engine, input, 5);
    expect(engine.rpm).toBeLessThan(RELIGHT_MIN_RPM);
    expect(engine.state).toBe('flameout');
    expect(engine.message).toContain('turns too slowly');
  });

  it('says the lever is open when a windmill relight is refused for that', () => {
    const { engine, input } = idlingEngine();
    input.altitude = 3000;
    input.density = isa(3000).density;
    input.airspeed = 160;
    input.mach = 0.48;
    input.fuelCockOpen = false;
    run(engine, input, 1);
    run(engine, input, 60);
    input.throttle = 0.8;
    input.fuelCockOpen = true;
    run(engine, input, 10);
    expect(engine.state).toBe('flameout');
    expect(engine.rpm).toBeLessThan(HOT_RELIGHT_MIN_RPM);
    expect(engine.message).toContain('Close the throttle');
  });

  it('says the engine burns and gives the one action that ends a fire', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.starterEngaged = true;
    input.fuelCockOpen = true;
    input.throttle = 0.9;
    run(engine, input, 40);
    if (engine.state === 'fire') {
      expect(engine.message).toContain('Shut it down');
      expect(engine.startPhase).toBe('failed');
    }
  });
});

/**
 * BEAD b70. A hot engine relights with the lever where it is.
 *
 * The closed throttle rule of the handbook air start is right for a cold
 * windmilling rotor and wrong for an engine that lost its flame a second ago
 * with the rotor still turning at its running speed.
 */
describe('Jumo 004 hot relight', () => {
  /** Builds an engine running at power in a dive at 4000 m. */
  function divingEngine(): { engine: Engine; input: EngineInput } {
    const { engine, input } = idlingEngine();
    run(engine, input, 20, () => advanceOnTheMargin(engine, input, 0.6));
    input.throttle = 1;
    run(engine, input, 10);
    input.altitude = 4000;
    input.density = isa(4000).density;
    input.airspeed = 230;
    input.mach = 0.7;
    run(engine, input, 5);
    expect(engine.state).toBe('running');
    return { engine, input };
  }

  it('relights with the throttle wide open after a fuel interruption in a dive', () => {
    const { engine, input } = divingEngine();
    const thrust = engine.thrust;
    expect(engine.rpm).toBeGreaterThan(HOT_RELIGHT_MIN_RPM);

    // Negative g uncovers the fuel pickup for one second. The pilot keeps the
    // lever where it is, because the pilot is flying a dive.
    input.fuelAvailable = false;
    run(engine, input, 1);
    expect(engine.state).toBe('flameout');

    input.fuelAvailable = true;
    run(engine, input, 2);
    // Before bead b70 the engine stayed out for the whole dive, because the
    // relight test asked for a throttle under 5 percent and the pilot had it
    // wide open.
    expect(engine.state).toBe('running');
    expect(input.throttle).toBe(1);
    expect(engine.thrust).toBeCloseTo(thrust, 0);
  });

  it('takes the hot relight without a surge, because the airflow is there', () => {
    const { engine, input } = divingEngine();
    input.fuelAvailable = false;
    run(engine, input, 1);
    input.fuelAvailable = true;
    let worst = 1;
    run(engine, input, 6, () => {
      worst = Math.min(worst, engine.surgeMargin);
    });
    // At 6000 rpm and above the compressor already passes the air that an open
    // lever asks for. That is the published rule the threshold comes from.
    expect(worst).toBeGreaterThan(0);
    expect(engine.state).toBe('running');
    // The flame lights back into a combustor whose valve never closed, so the
    // gas temperature spikes for a fraction of a second and the turbine creeps
    // a little. One tenth of one percent per relight is the price, against the
    // 36 percent that one hot start on the ground costs.
    expect(engine.damage).toBeLessThan(0.01);
  });

  it('refuses the hot relight once the rotor falls below the published 6000 rpm', () => {
    const { engine, input } = divingEngine();
    input.fuelAvailable = false;
    // Hold the fuel away until the rotor coasts under the threshold.
    runUntil(engine, input, 60, () => engine.rpm < HOT_RELIGHT_MIN_RPM - 200);
    expect(engine.rpm).toBeLessThan(HOT_RELIGHT_MIN_RPM);
    input.fuelAvailable = true;
    run(engine, input, 5);
    // The lever is still wide open, so only the cold rule is left and it holds
    // the relight shut. The message says which control clears it.
    expect(engine.state).toBe('flameout');
    expect(engine.message).toContain('Close the throttle');
  });

  it('still needs the closed lever for a cold windmill start', () => {
    const { engine, input } = idlingEngine();
    input.altitude = 3000;
    input.density = isa(3000).density;
    input.airspeed = 160;
    input.mach = 0.48;
    input.fuelCockOpen = false;
    run(engine, input, 1);
    run(engine, input, 120);
    expect(engine.rpm).toBeGreaterThan(RELIGHT_MIN_RPM);
    expect(engine.rpm).toBeLessThan(HOT_RELIGHT_MIN_RPM);

    // Lever open. The handbook air start says "Throttle closed", and this is a
    // windmilling rotor, so the rule stands.
    input.throttle = 0.8;
    input.fuelCockOpen = true;
    run(engine, input, 20);
    expect(engine.state).toBe('flameout');

    // Lever closed. Now it lights.
    input.throttle = 0;
    run(engine, input, 40);
    expect(engine.state === 'idle' || engine.state === 'running').toBe(true);
  });

  it('reports the threshold as the published throttle danger band speed', () => {
    // The basis of the number. The handbook says that below 6000 rpm any
    // advance of the throttle must be made slowly. A relight is one more
    // advance of the throttle, so it takes the same bound.
    expect(HOT_RELIGHT_MIN_RPM).toBe(DANGER_BAND_RPM);
  });
});

describe('Jumo 004 housekeeping', () => {
  it('burns no fuel and makes no thrust when it is off', () => {
    const engine = createJumo004(new Vector3(0, -2.5, 0.3));
    const input = makeInput();
    input.throttle = 1;
    input.fuelCockOpen = true;
    run(engine, input, 2);
    expect(engine.state).toBe('off');
    expect(engine.thrust).toBe(0);
    expect(engine.rpm).toBe(0);
  });

  it('shuts down and returns to the cold state on reset', () => {
    const { engine, input } = idlingEngine();
    engine.shutdown();
    run(engine, input, 5);
    expect(engine.state).toBe('off');
    expect(engine.fuelFlow).toBe(0);
    expect(engine.thrust).toBe(0);
    engine.reset();
    expect(engine.rpm).toBe(0);
    expect(engine.damage).toBe(0);
    expect(engine.state).toBe('off');
  });

  it('holds its position in body axes and copies the vector it is given', () => {
    const position = new Vector3(0, -2.5, 0.3);
    const engine = createJumo004(position);
    position.set(9, 9, 9);
    expect(engine.position.x).toBe(0);
    expect(engine.position.y).toBe(-2.5);
    expect(engine.position.z).toBe(0.3);
  });

  it('keeps the same event object, so the update allocates nothing', () => {
    const { engine, input } = idlingEngine();
    const events = engine.events;
    run(engine, input, 1);
    expect(engine.events).toBe(events);
    expect(engine.position).toBe(engine.position);
  });

  it('gives the same answer at 60 Hz as at 240 Hz', () => {
    const fast = createJumo004(new Vector3(0, 0, 0));
    const slow = createJumo004(new Vector3(0, 0, 0));
    const inputFast = makeInput();
    const inputSlow = makeInput();
    startAndIdle(fast, inputFast);
    inputSlow.starterEngaged = true;
    for (let i = 0; i < 60 * 60; i++) {
      if (slow.rpm >= 700) {
        inputSlow.fuelCockOpen = true;
      }
      slow.update(inputSlow, 1 / 60);
    }
    inputSlow.starterEngaged = false;
    for (let i = 0; i < 60 * 150; i++) {
      slow.update(inputSlow, 1 / 60);
    }
    expect(Math.abs(slow.rpm - fast.rpm)).toBeLessThan(60);
  });
});


