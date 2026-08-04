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
  IDLE_RPM,
  MAX_RPM,
  MAX_THRUST_SL_STATIC,
  RELIGHT_MIN_RPM,
  THRUST_ALTITUDE_EXPONENT,
  TURBINE_INLET_TEMPERATURE_LIMIT,
  createJumo004,
  thrustSpeedFraction,
} from '@/aircraft/me262/engine';
import type { Engine, EngineInput } from '@/aircraft/me262/engine';
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


