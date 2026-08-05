import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';

import {
  HUD_ALERTS,
  HUD_FIELDS,
  engineMessageText,
  findHudField,
  fixedText,
  flapSpeedLevel,
  fuelLevel,
  gasTemperatureLevel,
  gearSpeedLevel,
  loadFactorLevel,
  machLevel,
  rotorLevel,
  worstLevel,
} from '@/ui/hud';
import type { HudContext } from '@/ui/hud';
import type { TelemetrySample } from '@/ui/debug-overlay';
import {
  COCKPIT_EYE_FORWARD,
  COCKPIT_EYE_UP,
  VIEW_ORDER,
  createCameraRig,
} from '@/render/cameras';
import { DANGER_BAND_RPM, TURBINE_INLET_TEMPERATURE_LIMIT } from '@/aircraft/me262/engine';
import {
  LIMIT_LOAD_NEGATIVE,
  LIMIT_LOAD_POSITIVE,
  STRAIN_STRENGTH_LOSS,
  loadLimits,
} from '@/aircraft/me262/limits';
import { MAX_TAKEOFF_MASS } from '@/aircraft/me262/mass';
import { GEAR_LIMIT_SPEED } from '@/aircraft/me262/systems';
import { kmhToMs, msToKmh, toDeg } from '@/math/units';
import { isa } from '@/physics/atmosphere';
import { createState } from '@/physics/rigidbody';

/**
 * Tests for the head up display and for the camera set.
 *
 * Both modules touch the DOM or the renderer, so these tests drive only the
 * parts that hold no browser: the field table with its unit conversions, every
 * warning threshold, and the geometry of the four views.
 */

function makeSample(): TelemetrySample {
  return {
    loop: {
      fps: 0,
      physicsStepsLastFrame: 0,
      droppedTime: 0,
      fixedUpdateMs: 0,
      renderMs: 0,
      simTime: 0,
    },
    state: createState(),
    alpha: 0,
    beta: 0,
    loadFactor: 1,
    trueAirspeed: 0,
    equivalentAirspeed: 0,
    mach: 0,
    dynamicPressure: 0,
    atmosphere: isa(0),
  };
}

/** The readout with every field writable, so a test can set one up. */
interface MutableReadout {
  engines: Array<{ rpm: number; gasTemperature: number; state: string; message: string }>;
  throttle: number;
  fuelMass: number;
  gearPosition: number;
  flapPosition: number;
  rounds: number;
  loadLimits: { limitPositive: number; limitNegative: number };
}

function makeReadout(): MutableReadout {
  return {
    engines: [
      { rpm: 0, gasTemperature: 288.15, state: 'off', message: '' },
      { rpm: 0, gasTemperature: 288.15, state: 'off', message: '' },
    ],
    throttle: 0,
    fuelMass: 2133,
    gearPosition: 1,
    flapPosition: 0,
    rounds: 360,
    loadLimits: {
      limitPositive: LIMIT_LOAD_POSITIVE,
      limitNegative: LIMIT_LOAD_NEGATIVE,
    },
  };
}

const STILL: HudContext = { throttleMoving: false, throttleRising: false };
const ADVANCING: HudContext = { throttleMoving: true, throttleRising: true };
const CLOSING: HudContext = { throttleMoving: true, throttleRising: false };

describe('the fixed width of the display', () => {
  it('pads a word to a constant character count', () => {
    expect(fixedText('UP', 4)).toBe('  UP');
    expect(fixedText('DOWN', 4)).toBe('DOWN');
    expect(fixedText('', 4)).toBe('    ');
  });

  it('cuts a word that is too long, so the column never grows', () => {
    expect(fixedText('flameout', 4)).toHaveLength(4);
  });

  it('gives every printed cell of the display a constant width', () => {
    const sample = makeSample();
    const readout = makeReadout();
    for (const field of HUD_FIELDS) {
      const value = field.read(sample, readout);
      const text =
        field.text === undefined
          ? value.toFixed(field.decimals).padStart(field.width, ' ')
          : fixedText(field.text(value, readout), field.width);
      expect(text.length).toBeGreaterThanOrEqual(field.width);
    }
  });
});

describe('the field table of the display', () => {
  it('prints the true airspeed in kilometers per hour', () => {
    const sample = makeSample();
    sample.trueAirspeed = 200;
    const field = findHudField('speed');
    expect(field?.read(sample, makeReadout())).toBeCloseTo(msToKmh(200), 6);
    expect(field?.read(sample, makeReadout())).toBeCloseTo(720, 6);
  });

  it('prints the altitude as minus the NED z, never plus', () => {
    const sample = makeSample();
    sample.state.position.z = -3000;
    expect(findHudField('altitude')?.read(sample, makeReadout())).toBe(3000);
  });

  it('prints the climb rate as minus the NED z velocity', () => {
    const sample = makeSample();
    sample.state.velocity.z = -20;
    expect(findHudField('climb')?.read(sample, makeReadout())).toBe(20);
  });

  it('prints the heading of a north attitude as zero degrees', () => {
    const sample = makeSample();
    expect(findHudField('heading')?.read(sample, makeReadout())).toBeCloseTo(0, 6);
  });

  it('prints the heading of an east attitude as ninety degrees', () => {
    const sample = makeSample();
    // A heading turns about the NED down axis, which is +z.
    sample.state.orientation.setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2);
    expect(findHudField('heading')?.read(sample, makeReadout())).toBeCloseTo(90, 4);
  });

  it('prints the angle of attack in degrees', () => {
    const sample = makeSample();
    sample.alpha = 0.2;
    expect(findHudField('alpha')?.read(sample, makeReadout())).toBeCloseTo(toDeg(0.2), 6);
  });

  it('prints the gas temperature of each engine in degrees Celsius', () => {
    const sample = makeSample();
    const readout = makeReadout();
    readout.engines[0].gasTemperature = 1015;
    expect(findHudField('egt-1')?.read(sample, readout)).toBeCloseTo(1015 - 273.15, 6);
  });

  it('prints the gear and the flap as words', () => {
    const gear = findHudField('gear');
    const flap = findHudField('flap');
    const readout = makeReadout();
    expect(gear?.text?.(1, readout)).toBe('DOWN');
    expect(gear?.text?.(0, readout)).toBe('UP');
    expect(gear?.text?.(0.5, readout)).toBe('TRAV');
    expect(flap?.text?.(0, readout)).toBe('UP');
    expect(flap?.text?.(1, readout)).toBe('LAND');
  });

  it('shows the ammunition of the four MK 108', () => {
    const sample = makeSample();
    expect(findHudField('ammo')?.read(sample, makeReadout())).toBe(360);
  });
});

describe('the warning thresholds', () => {
  it('takes the higher of two levels', () => {
    expect(worstLevel('normal', 'caution')).toBe('caution');
    expect(worstLevel('caution', 'warning')).toBe('warning');
    expect(worstLevel('normal', 'normal')).toBe('normal');
  });

  it('leaves a rotor at idle alone while the lever does not move', () => {
    expect(rotorLevel(3000, STILL)).toBe('normal');
  });

  it('warns in the surge band while the lever advances', () => {
    expect(rotorLevel(DANGER_BAND_RPM - 1, ADVANCING)).toBe('warning');
    expect(rotorLevel(DANGER_BAND_RPM - 1, CLOSING)).toBe('caution');
  });

  it('leaves a rotor above the surge band alone at any lever movement', () => {
    expect(rotorLevel(DANGER_BAND_RPM, ADVANCING)).toBe('normal');
    expect(rotorLevel(8700, ADVANCING)).toBe('normal');
  });

  it('leaves a dead rotor alone, because it can not surge', () => {
    expect(rotorLevel(0, ADVANCING)).toBe('normal');
  });

  it('warns above the turbine inlet temperature limit', () => {
    expect(gasTemperatureLevel(TURBINE_INLET_TEMPERATURE_LIMIT + 1)).toBe('warning');
    expect(gasTemperatureLevel(1070)).toBe('caution');
    // 1015 K is the full power value, so it must raise nothing.
    expect(gasTemperatureLevel(1015)).toBe('normal');
  });

  it('warns over the gear limit speed and only with the gear out', () => {
    expect(gearSpeedLevel(GEAR_LIMIT_SPEED + 1, 1)).toBe('warning');
    expect(gearSpeedLevel(GEAR_LIMIT_SPEED + 1, 0)).toBe('normal');
    expect(gearSpeedLevel(kmhToMs(300), 1)).toBe('normal');
  });

  it('warns over the flap limit speed and only with the flaps down', () => {
    // The landing setting takes 380 km/h EAS. See src/aircraft/me262/systems.ts.
    expect(flapSpeedLevel(kmhToMs(420), 1)).toBe('warning');
    expect(flapSpeedLevel(kmhToMs(420), 0)).toBe('normal');
    expect(flapSpeedLevel(kmhToMs(200), 1)).toBe('normal');
  });

  it('warns past the load factor limits of plus seven and minus three', () => {
    expect(loadFactorLevel(7.5)).toBe('warning');
    expect(loadFactorLevel(-3.5)).toBe('warning');
    expect(loadFactorLevel(6.5)).toBe('caution');
    expect(loadFactorLevel(1)).toBe('normal');
  });

  it('warns at 6.28 g at the maximum takeoff mass, where the limit really is', () => {
    // src/aircraft/me262/limits.ts scales the limit with the mass. At 7130 kg
    // the positive limit is 6.28 g, so 6.5 g is a WARNING and not a caution.
    const heavy = loadLimits(MAX_TAKEOFF_MASS, {
      limitPositive: 0,
      limitNegative: 0,
      ultimatePositive: 0,
      ultimateNegative: 0,
    });
    expect(heavy.limitPositive).toBeCloseTo(6.28, 2);
    expect(loadFactorLevel(6.5, heavy.limitPositive, heavy.limitNegative)).toBe('warning');
    expect(loadFactorLevel(5.7, heavy.limitPositive, heavy.limitNegative)).toBe('caution');
    expect(loadFactorLevel(1, heavy.limitPositive, heavy.limitNegative)).toBe('normal');
  });

  it('warns lower once the wing carries a permanent set', () => {
    // A full permanent set costs the airframe STRAIN_STRENGTH_LOSS of its
    // strength, so the limit falls from 7.00 g to 4.90 g.
    const bent = LIMIT_LOAD_POSITIVE * (1 - STRAIN_STRENGTH_LOSS);
    expect(bent).toBeCloseTo(4.9, 6);
    expect(loadFactorLevel(5, bent, LIMIT_LOAD_NEGATIVE * (1 - STRAIN_STRENGTH_LOSS))).toBe(
      'warning',
    );
    // The same 5 g raises nothing at all on a sound airframe.
    expect(loadFactorLevel(5)).toBe('normal');
  });

  it('prints one engine message even when both engines raise it', () => {
    const readout = makeReadout();
    expect(engineMessageText(readout.engines)).toBe('');
    readout.engines[0].message = 'OPEN THE FUEL COCK.';
    readout.engines[1].message = 'OPEN THE FUEL COCK.';
    expect(engineMessageText(readout.engines)).toBe('ENG 1 2  OPEN THE FUEL COCK.');
  });

  it('prints one line per engine when the two engines say different things', () => {
    const readout = makeReadout();
    readout.engines[0].message = 'THE TAIL PIPE IS WET.';
    readout.engines[1].message = 'CLOSE THE LEVER.';
    expect(engineMessageText(readout.engines)).toBe(
      'ENG 1  THE TAIL PIPE IS WET.\nENG 2  CLOSE THE LEVER.',
    );
  });

  it('an engine with nothing to say adds no line', () => {
    const readout = makeReadout();
    readout.engines[1].message = 'CLOSE THE LEVER.';
    expect(engineMessageText(readout.engines)).toBe('ENG 2  CLOSE THE LEVER.');
  });

  it('the G LIMIT alert follows the limits the readout carries', () => {
    const sample = makeSample();
    const readout = makeReadout();
    const alert = HUD_ALERTS.find((a) => a.key === 'load');
    sample.loadFactor = 5;
    expect(alert?.level(sample, readout, STILL)).toBe('normal');
    readout.loadLimits.limitPositive = 4.9;
    expect(alert?.level(sample, readout, STILL)).toBe('warning');
  });

  it('warns on low fuel', () => {
    expect(fuelLevel(90)).toBe('warning');
    expect(fuelLevel(200)).toBe('caution');
    expect(fuelLevel(2133)).toBe('normal');
  });

  it('warns at the Mach limit and cautions at the tuck onset', () => {
    expect(machLevel(0.87)).toBe('warning');
    expect(machLevel(0.84)).toBe('caution');
    expect(machLevel(0.7)).toBe('normal');
  });

  it('raises no alert at all in steady level flight', () => {
    const sample = makeSample();
    sample.trueAirspeed = 200;
    sample.equivalentAirspeed = 200;
    sample.mach = 0.6;
    const readout = makeReadout();
    readout.gearPosition = 0;
    readout.engines[0].rpm = 8000;
    readout.engines[1].rpm = 8000;
    for (const alert of HUD_ALERTS) {
      expect(alert.level(sample, readout, STILL)).toBe('normal');
    }
  });

  it('raises the gear overspeed alert on a fast pass with the gear down', () => {
    const sample = makeSample();
    sample.equivalentAirspeed = kmhToMs(600);
    const readout = makeReadout();
    const alert = HUD_ALERTS.find((a) => a.key === 'gear-speed');
    expect(alert?.level(sample, readout, STILL)).toBe('warning');
  });
});

describe('the camera set', () => {
  it('steps through the four views in order', () => {
    const rig = createCameraRig();
    expect(VIEW_ORDER).toEqual(['cockpit', 'chase', 'orbit', 'flyby']);
    rig.mode = 'cockpit';
    for (const wanted of ['chase', 'orbit', 'flyby', 'cockpit']) {
      rig.cycle();
      expect(rig.mode).toBe(wanted);
    }
    rig.dispose();
  });

  it('puts the cockpit eye ahead of and above the center of gravity', () => {
    const rig = createCameraRig();
    rig.mode = 'cockpit';
    const position = new Vector3(0, 0, -1000);
    const orientation = new Quaternion();
    rig.update(position, orientation, 1, 0, 0, 0, 1 / 60);
    // NED (0, 0, -1000) is render (0, 1000, 0). The eye sits forward along
    // render -z and up along render +y. The shake moves it by a millimeter.
    expect(rig.camera.position.x).toBeCloseTo(0, 2);
    expect(rig.camera.position.y).toBeCloseTo(1000 + COCKPIT_EYE_UP, 2);
    expect(rig.camera.position.z).toBeCloseTo(-COCKPIT_EYE_FORWARD, 2);
    rig.dispose();
  });

  it('rolls the cockpit view with the aircraft', () => {
    const rig = createCameraRig();
    rig.mode = 'cockpit';
    const orientation = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2);
    rig.update(new Vector3(0, 0, -1000), orientation, 1, 0, 0, 0, 1 / 60);
    // A ninety degree right roll takes the up of the head onto the horizontal.
    const up = new Vector3(0, 1, 0).applyQuaternion(rig.camera.quaternion);
    expect(Math.abs(up.y)).toBeLessThan(0.05);
    rig.dispose();
  });

  it('keeps the chase view level through a roll', () => {
    const rig = createCameraRig();
    rig.mode = 'chase';
    const orientation = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2);
    rig.update(new Vector3(0, 0, -1000), orientation, 1, 100, 0, 0, 1 / 60);
    // The right of a camera that does not roll stays in the horizontal plane.
    const right = new Vector3(1, 0, 0).applyQuaternion(rig.camera.quaternion);
    expect(Math.abs(right.y)).toBeLessThan(1e-6);
    rig.dispose();
  });

  it('holds the orbit view at its distance and points it at the aircraft', () => {
    const rig = createCameraRig();
    rig.mode = 'orbit';
    rig.orbitDistance = 60;
    const position = new Vector3(100, 200, -1500);
    rig.update(position, new Quaternion(), 1, 200, 0, 0, 1 / 60);
    // Render position of NED (100, 200, -1500) is (200, 1500, -100).
    const target = new Vector3(200, 1500, -100);
    expect(rig.camera.position.distanceTo(target)).toBeCloseTo(60, 4);
    const forward = new Vector3(0, 0, -1).applyQuaternion(rig.camera.quaternion);
    const toTarget = target.clone().sub(rig.camera.position).normalize();
    expect(forward.dot(toTarget)).toBeCloseTo(1, 4);
    rig.dispose();
  });

  it('parks the fly by camera ahead of the aircraft and re-parks after a pass', () => {
    const rig = createCameraRig();
    rig.mode = 'flyby';
    const position = new Vector3(0, 0, -300);
    const orientation = new Quaternion();
    rig.update(position, orientation, 1, 200, 0, 0, 1 / 60);
    const first = rig.camera.position.clone();
    // The aircraft heads north, which is render -z, so the park point is ahead
    // of it on that axis.
    expect(first.z).toBeLessThan(-100);

    // Fly past the park point. The camera must move to a new one, again ahead.
    position.x = 2000;
    rig.update(position, orientation, 1, 200, 0, 0, 1 / 60);
    const second = rig.camera.position.clone();
    expect(second.distanceTo(first)).toBeGreaterThan(100);
    expect(second.z).toBeLessThan(-2000);
    rig.dispose();
  });
});
