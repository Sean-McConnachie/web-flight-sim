import { describe, expect, it } from 'vitest';

import {
  ABSORPTION_CONSTANT,
  BUFFET_ALPHA_FULL,
  BUFFET_ALPHA_ONSET,
  COMPRESSOR_BLADES,
  JET_STROUHAL,
  JET_VELOCITY_REFERENCE,
  MACH_BUFFET_FULL,
  MACH_BUFFET_ONSET,
  MAIN_WHEEL_RADIUS,
  MAXIMUM_DELAY,
  MINIMUM_FREQUENCY,
  NOZZLE_DIAMETER,
  REFERENCE_DISTANCE,
  ROLLING_LOAD_REFERENCE,
  ROLLING_SPEED_REFERENCE,
  TREAD_BLOCKS,
  WIND_SPEED_REFERENCE,
  absorptionCutoff,
  airframeVoice,
  bladePassingFrequency,
  compressorMassFlow,
  compressorToneGain,
  coreNoiseGain,
  createAirframeVoiceParameters,
  createEngineVoiceParameters,
  createPropagationParameters,
  createRollingVoiceParameters,
  delayLineDopplerFactor,
  distanceGain,
  dopplerFactor,
  engineVoice,
  jetMixingGain,
  jetPeakFrequency,
  jetVelocity,
  propagation,
  rollingVoice,
  windCutoff,
  windGain,
} from '@/audio/voices';
import type { AirframeVoiceInput, EngineVoiceInput, RollingVoiceInput } from '@/audio/voices';
import { MASS_FLOW_MAX, MAX_THRUST_SL_STATIC, OMEGA_MAX } from '@/aircraft/me262/engine';
import { rpmToRadPerSec } from '@/math/units';

/** Turns an amplitude ratio into decibels. Sound pressure uses 20 log10. */
function decibels(ratio: number): number {
  return 20 * Math.log10(ratio);
}

/** An engine at rest, at sea level, with nothing running. */
function coldEngine(): EngineVoiceInput {
  return {
    rotorSpeed: 0,
    thrust: 0,
    fuelFlow: 0,
    trueAirspeed: 0,
    lit: false,
    starterRunning: false,
    onFire: false,
  };
}

/** An engine at full power, at rest, at sea level. */
function fullPowerEngine(): EngineVoiceInput {
  return {
    rotorSpeed: OMEGA_MAX,
    thrust: MAX_THRUST_SL_STATIC,
    fuelFlow: 0.355,
    trueAirspeed: 0,
    lit: true,
    starterRunning: false,
    onFire: false,
  };
}

/** An airframe with the gear up, the flaps up and no buffet. */
function cleanAirframe(): AirframeVoiceInput {
  return { trueAirspeed: 0, alpha: 0, mach: 0, gearPosition: 0, flapPosition: 0 };
}

function parkedWheels(): RollingVoiceInput {
  return { wheelSpeed: 0, wheelLoad: ROLLING_LOAD_REFERENCE, onGround: true, slip: 0 };
}

describe('the compressor tone', () => {
  it('the blade passing tone at 8700 rpm is 3.9 kHz', () => {
    // 8700 rpm is 145 shaft revolutions per second, and every one of them
    // carries 27 blades past the inlet.
    expect(bladePassingFrequency(OMEGA_MAX)).toBeCloseTo(145 * COMPRESSOR_BLADES, 0);
    expect(bladePassingFrequency(OMEGA_MAX)).toBeGreaterThan(2000);
    expect(bladePassingFrequency(OMEGA_MAX)).toBeLessThan(5000);
  });

  it('the tone frequency is proportional to the rotor speed', () => {
    const full = bladePassingFrequency(OMEGA_MAX);
    expect(bladePassingFrequency(OMEGA_MAX / 2)).toBeCloseTo(full / 2, 6);
    expect(bladePassingFrequency(0)).toBe(0);
  });

  it('the tone at the 3000 rpm idle is 28 dB below full power', () => {
    // Curle gives a dipole an amplitude that follows the cube of the speed, so
    // the ratio is (3000 / 8700) ^ 3 and that is 60 log10(0.345).
    const idle = compressorToneGain(rpmToRadPerSec(3000));
    expect(decibels(idle / compressorToneGain(OMEGA_MAX))).toBeCloseTo(-27.7, 1);
  });

  it('halving the rotor speed takes 18 dB off the tone', () => {
    const ratio = compressorToneGain(OMEGA_MAX / 2) / compressorToneGain(OMEGA_MAX);
    expect(decibels(ratio)).toBeCloseTo(-18.06, 2);
  });
});

describe('the jet', () => {
  it('the jet velocity at full static power is the thrust over the mass flow', () => {
    // A turbojet makes mdot (Vj - V0), and V0 is zero at rest.
    expect(compressorMassFlow(OMEGA_MAX)).toBeCloseTo(MASS_FLOW_MAX, 6);
    expect(jetVelocity(MAX_THRUST_SL_STATIC, OMEGA_MAX, 0)).toBeCloseTo(415.09, 1);
    expect(JET_VELOCITY_REFERENCE).toBeCloseTo(415.09, 1);
  });

  it('the aircraft speed adds to the jet velocity', () => {
    const still = jetVelocity(MAX_THRUST_SL_STATIC, OMEGA_MAX, 0);
    expect(jetVelocity(MAX_THRUST_SL_STATIC, OMEGA_MAX, 200)).toBeCloseTo(still + 200, 6);
  });

  it('the mixing noise follows the eighth power of the jet velocity', () => {
    // Lighthill gives the POWER an eighth power law. The amplitude is the
    // square root of the power, so halving the velocity takes 24 dB off it.
    const full = jetMixingGain(JET_VELOCITY_REFERENCE);
    const half = jetMixingGain(JET_VELOCITY_REFERENCE / 2);
    expect(full).toBeCloseTo(1, 6);
    expect(half).toBeCloseTo(1 / 16, 6);
    expect(decibels(half / full)).toBeCloseTo(-24.08, 2);
  });

  it('the jet noise peaks at a Strouhal number of 0.2', () => {
    const speed = 415;
    expect(jetPeakFrequency(speed)).toBeCloseTo((JET_STROUHAL * speed) / NOZZLE_DIAMETER, 6);
    // The peak of a 0.55 m nozzle at full power is a low rumble near 150 Hz.
    expect(jetPeakFrequency(JET_VELOCITY_REFERENCE)).toBeCloseTo(151, 0);
  });

  it('the core noise holds a third of its level at idle', () => {
    // The eighth power law alone would take the idle engine to silence. The
    // combustion is the source that is left, and it falls far more slowly.
    expect(coreNoiseGain(0.355)).toBeCloseTo(1, 6);
    const idle = coreNoiseGain(0.0613);
    expect(idle).toBeGreaterThan(0.3);
    expect(idle).toBeLessThan(0.5);
  });
});

describe('one engine voice', () => {
  it('a cold engine is silent on every source', () => {
    const out = engineVoice(coldEngine(), createEngineVoiceParameters());
    expect(out.whineGain).toBe(0);
    expect(out.roarGain).toBe(0);
    expect(out.coreGain).toBe(0);
    expect(out.starterGain).toBe(0);
    expect(out.fireGain).toBe(0);
  });

  it('no oscillator is ever asked for a frequency near zero', () => {
    // A cold engine has a rotor speed of zero, and both the tone and the
    // starter read that speed. An oscillator at 0 Hz is a stuck value.
    const out = engineVoice(coldEngine(), createEngineVoiceParameters());
    expect(out.whineFrequency).toBeGreaterThanOrEqual(MINIMUM_FREQUENCY);
    expect(out.starterFrequency).toBeGreaterThanOrEqual(MINIMUM_FREQUENCY);
  });

  it('a rotor that windmills with no flame makes a tone and no jet', () => {
    const input = coldEngine();
    input.rotorSpeed = rpmToRadPerSec(1500);
    input.trueAirspeed = 150;
    const out = engineVoice(input, createEngineVoiceParameters());
    expect(out.whineGain).toBeGreaterThan(0);
    expect(out.roarGain).toBe(0);
    expect(out.coreGain).toBe(0);
  });

  it('the starter only sounds while it turns the rotor', () => {
    const input = coldEngine();
    input.rotorSpeed = rpmToRadPerSec(800);
    input.starterRunning = true;
    const out = engineVoice(input, createEngineVoiceParameters());
    expect(out.starterGain).toBe(1);
    // The handbook cranks to 800 rpm before the pilot presses the ignition.
    // A two stroke twin through the reduction then putts near 133 Hz.
    expect(out.starterFrequency).toBeCloseTo(133, 0);

    input.starterRunning = false;
    expect(engineVoice(input, createEngineVoiceParameters()).starterGain).toBe(0);
  });

  it('the roar opens up with power and the low pass opens with it', () => {
    const idle = engineVoice(
      { ...fullPowerEngine(), rotorSpeed: rpmToRadPerSec(3000), thrust: 480, fuelFlow: 0.0613 },
      createEngineVoiceParameters(),
    );
    const full = engineVoice(fullPowerEngine(), createEngineVoiceParameters());
    expect(full.roarGain).toBeGreaterThan(idle.roarGain * 20);
    expect(full.roarCutoff).toBeGreaterThan(idle.roarCutoff);
    expect(full.roarCutoff).toBeCloseTo(1811, 0);
  });

  it('the engine at idle still makes a sound, because the combustion does', () => {
    const idle = engineVoice(
      { ...fullPowerEngine(), rotorSpeed: rpmToRadPerSec(3000), thrust: 480, fuelFlow: 0.0613 },
      createEngineVoiceParameters(),
    );
    // The mixing noise has all but gone at idle. Read section 2 of the module
    // comment. The core and the tone are what a pilot hears at the holding
    // point, and both must be well clear of silence.
    expect(idle.roarGain).toBeLessThan(0.01);
    expect(idle.coreGain).toBeGreaterThan(0.3);
    expect(idle.whineGain).toBeGreaterThan(0.03);
  });

  it('a burning nacelle raises the fire voice', () => {
    const input = fullPowerEngine();
    input.onFire = true;
    expect(engineVoice(input, createEngineVoiceParameters()).fireGain).toBe(1);
  });
});

describe('the airframe', () => {
  it('the wind noise follows the sixth power law of a dipole', () => {
    // Curle gives the POWER a sixth power law, so the amplitude follows the
    // cube and a doubling of the speed adds 18 dB.
    const fast = windGain(WIND_SPEED_REFERENCE);
    const half = windGain(WIND_SPEED_REFERENCE / 2);
    expect(fast).toBeCloseTo(1, 6);
    expect(decibels(fast / half)).toBeCloseTo(18.06, 2);
  });

  it('a parked aircraft hears no wind', () => {
    const out = airframeVoice(cleanAirframe(), createAirframeVoiceParameters());
    expect(out.windGain).toBe(0);
    expect(out.buffetDepth).toBe(0);
  });

  it('the rush of air rises in pitch with the speed', () => {
    expect(windCutoff(50)).toBeLessThan(windCutoff(250));
    // The peak follows the speed over the thickness of the boundary layer.
    expect(windCutoff(250)).toBeCloseTo(6250, 0);
  });

  it('the gear and the flaps add noise and drop its pitch', () => {
    const clean = cleanAirframe();
    clean.trueAirspeed = 100;
    const clear = airframeVoice(clean, createAirframeVoiceParameters());
    const clearGain = clear.windGain;
    const clearCutoff = clear.windCutoff;

    const dirty = { ...clean, gearPosition: 1, flapPosition: 1 };
    const out = airframeVoice(dirty, createAirframeVoiceParameters());
    // A leg and a wheel in the free stream are the loudest part of an airframe
    // on the approach, and they are a bluff body, so they arrive lower down.
    expect(out.windGain).toBeGreaterThan(clearGain);
    expect(out.windCutoff).toBeLessThan(clearCutoff);
  });

  it('the buffet warns before the wing stalls', () => {
    const input = cleanAirframe();
    input.trueAirspeed = 100;

    input.alpha = BUFFET_ALPHA_ONSET - 0.01;
    expect(airframeVoice(input, createAirframeVoiceParameters()).buffetDepth).toBe(0);

    // The buffet IS the stall warning of an aircraft with no stick shaker, so
    // it must be well developed before the wing reaches its maximum lift.
    input.alpha = (BUFFET_ALPHA_ONSET + BUFFET_ALPHA_FULL) / 2;
    expect(airframeVoice(input, createAirframeVoiceParameters()).buffetDepth).toBeCloseTo(0.5, 2);

    input.alpha = BUFFET_ALPHA_FULL;
    expect(airframeVoice(input, createAirframeVoiceParameters()).buffetDepth).toBe(1);
  });

  it('the buffet answers a negative angle of attack as well', () => {
    const input = cleanAirframe();
    input.trueAirspeed = 100;
    input.alpha = -BUFFET_ALPHA_FULL;
    expect(airframeVoice(input, createAirframeVoiceParameters()).buffetDepth).toBe(1);
  });

  it('the shock buffet arrives before the Mach tuck and shakes faster', () => {
    const input = cleanAirframe();
    input.trueAirspeed = 250;

    input.mach = MACH_BUFFET_ONSET - 0.01;
    const quiet = airframeVoice(input, createAirframeVoiceParameters());
    expect(quiet.buffetDepth).toBe(0);
    const slowRate = quiet.buffetFrequency;

    // Section 8 of CONVENTIONS puts the tuck onset at 0.83. The shock that
    // makes the tuck separates the flow first, so the shake comes first.
    input.mach = 0.83;
    expect(airframeVoice(input, createAirframeVoiceParameters()).buffetDepth).toBeGreaterThan(0.3);

    input.mach = MACH_BUFFET_FULL;
    const loud = airframeVoice(input, createAirframeVoiceParameters());
    expect(loud.buffetDepth).toBe(1);
    expect(loud.buffetFrequency).toBeGreaterThan(slowRate);
  });
});

describe('the tires', () => {
  it('a wheel in the air is silent', () => {
    const input = parkedWheels();
    input.wheelSpeed = 60;
    input.onGround = false;
    const out = rollingVoice(input, createRollingVoiceParameters());
    expect(out.rumbleGain).toBe(0);
    expect(out.squealGain).toBe(0);
  });

  it('a wheel that stands still is silent, whatever load it carries', () => {
    const out = rollingVoice(parkedWheels(), createRollingVoiceParameters());
    expect(out.rumbleGain).toBe(0);
  });

  it('a wheel that carries no load is silent, whatever speed it turns at', () => {
    const input = parkedWheels();
    input.wheelSpeed = ROLLING_SPEED_REFERENCE;
    input.wheelLoad = 0;
    expect(rollingVoice(input, createRollingVoiceParameters()).rumbleGain).toBe(0);
  });

  it('the tread beats the ground once for every block that passes', () => {
    // One turn of a 0.42 m wheel covers 2.64 m and carries 66 blocks past.
    const speed = 2 * Math.PI * MAIN_WHEEL_RADIUS;
    const input = parkedWheels();
    input.wheelSpeed = speed;
    const out = rollingVoice(input, createRollingVoiceParameters());
    expect(out.rumbleFrequency).toBeCloseTo(TREAD_BLOCKS, 6);
  });

  it('a locked wheel squeals and a rolling wheel does not', () => {
    const input = parkedWheels();
    input.wheelSpeed = ROLLING_SPEED_REFERENCE;
    input.slip = 0;
    expect(rollingVoice(input, createRollingVoiceParameters()).squealGain).toBe(0);

    input.slip = 1;
    expect(rollingVoice(input, createRollingVoiceParameters()).squealGain).toBeCloseTo(1, 6);
  });

  it('a locked wheel that stands still is silent', () => {
    const input = parkedWheels();
    input.slip = 1;
    expect(rollingVoice(input, createRollingVoiceParameters()).squealGain).toBe(0);
  });
});

describe('propagation through the air', () => {
  it('the level halves for every doubling of the distance', () => {
    // Sound spreads over a sphere, so the pressure follows one over the radius.
    expect(distanceGain(REFERENCE_DISTANCE)).toBeCloseTo(1, 6);
    expect(distanceGain(2 * REFERENCE_DISTANCE)).toBeCloseTo(0.5, 6);
    expect(distanceGain(4 * REFERENCE_DISTANCE)).toBeCloseTo(0.25, 6);
    expect(decibels(distanceGain(2 * REFERENCE_DISTANCE))).toBeCloseTo(-6.02, 2);
  });

  it('the level holds inside the reference distance', () => {
    // The cockpit view sits on top of the source. Without the floor the gain
    // would run away to infinity there.
    expect(distanceGain(0)).toBeCloseTo(1, 6);
    expect(distanceGain(1)).toBeCloseTo(1, 6);
  });

  it('the air takes the high frequencies off with distance', () => {
    expect(absorptionCutoff(20)).toBe(20_000);
    expect(absorptionCutoff(200)).toBeCloseTo(ABSORPTION_CONSTANT / 200, 6);
    // A jet at a kilometer is a rumble and nothing else.
    expect(absorptionCutoff(1000)).toBeCloseTo(400, 6);
    expect(absorptionCutoff(20)).toBeGreaterThan(absorptionCutoff(2000));
  });

  it('the delay is the distance over the speed of sound', () => {
    const out = propagation({ distance: 343, speedOfSound: 343 }, createPropagationParameters());
    expect(out.delay).toBeCloseTo(1, 6);
  });

  it('the delay stops at the length of the delay line', () => {
    const far = propagation(
      { distance: 100_000, speedOfSound: 340 },
      createPropagationParameters(),
    );
    expect(far.delay).toBe(MAXIMUM_DELAY);
    // Nothing is audible out there anyway, which is what makes the cap safe.
    expect(far.gain).toBeLessThan(0.001);
  });

  it('a ramped delay line IS the Doppler shift', () => {
    // This is the identity the whole propagation stage rests on. A source that
    // moves away at v makes its delay grow at v / c, and a delay line whose
    // delay grows at that rate lowers the pitch by exactly the Doppler factor.
    // No separate pitch calculation exists anywhere in src/audio.
    const c = 340;
    for (const radialSpeed of [-250, -120, -30, 0, 30, 120, 250]) {
      const delayRate = radialSpeed / c;
      expect(delayLineDopplerFactor(delayRate)).toBeCloseTo(dopplerFactor(radialSpeed, c), 10);
    }
  });

  it('a source that closes raises the pitch and one that recedes lowers it', () => {
    const c = 340;
    expect(dopplerFactor(0, c)).toBeCloseTo(1, 6);
    expect(dopplerFactor(-170, c)).toBeCloseTo(2, 6);
    expect(dopplerFactor(170, c)).toBeCloseTo(2 / 3, 6);
  });

  it('a fly by at 250 m/s moves the tone over two and a half octaves', () => {
    // The aircraft passes the fixed fly by camera at 900 km/h, which is Mach
    // 0.735. The 3.9 kHz compressor tone arrives at 14.8 kHz on the way in and
    // at 2.3 kHz on the way out. The whole drop lands in the second the
    // aircraft passes, and it is the sound a fast fly by makes.
    const c = 340;
    const tone = bladePassingFrequency(OMEGA_MAX);
    expect(tone * dopplerFactor(-250, c)).toBeCloseTo(14_790, -1);
    expect(tone * dopplerFactor(250, c)).toBeCloseTo(2256, -1);
    expect(dopplerFactor(-250, c) / dopplerFactor(250, c)).toBeCloseTo(6.56, 2);
  });

  it('the closing shift runs away as the source reaches the speed of sound', () => {
    // This is not a fault. A source that catches its own sound is a sonic boom,
    // and the classical formula says so by dividing by zero. The clamp holds
    // the factor at 20, which no state of this aircraft can reach.
    const c = 340;
    expect(dopplerFactor(-339, c)).toBeGreaterThan(10);
    expect(dopplerFactor(-c, c)).toBeCloseTo(20, 6);
    expect(Number.isFinite(dopplerFactor(-2 * c, c))).toBe(true);
  });
});
