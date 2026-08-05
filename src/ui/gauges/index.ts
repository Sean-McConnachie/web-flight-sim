/**
 * The live instrument panel of the Me 262 A-1a.
 *
 *
 * WHAT THIS FILE DOES
 *
 * `createMe262Cockpit` of src/render/models/cockpit.ts builds fifteen bezels
 * with a flat face disc in each one. This file gives every one of those discs
 * a painted face and moving needles, and then drives them.
 *
 * The list below is the whole panel. A name that cockpit.ts does not carry is
 * skipped in silence, so the two files can move apart by one gauge without a
 * crash in flight.
 *
 *
 * THE COST RULES
 *
 * Every face is painted ONE time here, at build time. Nothing after that
 * uploads a texture. `update` writes one rotation per needle and allocates
 * nothing.
 *
 * `update` runs only while the COCKPIT view runs. src/main.ts builds the whole
 * interior on the first entry into that view and hides it in every other view,
 * and it calls `update` only when the interior is on screen. Section 6 of
 * cockpit.ts holds the same rule for the interior itself.
 *
 * This file touches the renderer and the DOM. CONVENTIONS section 4 allows
 * that under src/ui. It holds no physics.
 */

import type { Object3D } from 'three/webgpu';

import type { TelemetrySample } from '@/ui/debug-overlay';

import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { createGaugeParts } from './parts';

import { createAirspeed } from './airspeed';
import { createAltimeter } from './altimeter';
import { createClock } from './clock';
import { createCompass } from './compass';
import { createFuel } from './fuel';
import { createGasTemperature } from './gas-temperature';
import { createHoming } from './homing';
import { createHorizon } from './horizon';
import { createPressure } from './pressure';
import { createTachometer } from './tachometer';
import { createTurnSlip } from './turn-slip';
import { createVariometer } from './variometer';

export type { CockpitReadout, EngineGaugeReadout } from './readout';

export interface CockpitGauges {
  /**
   * Move every needle. Call it one time per frame, and ONLY while the cockpit
   * view is on screen.
   */
  update(sample: TelemetrySample, readout: CockpitReadout, dt: number): void;
  dispose(): void;
}

/**
 * How each bezel of cockpit.ts is filled.
 *
 * The German name of each instrument sits in the comment, and the English name
 * beside it, because the face carries the German unit words that the pilot
 * really read.
 */
const PANEL: ReadonlyArray<{ name: string; build: (parts: GaugeParts) => Instrument }> = [
  // --- Flight group, top row -------------------------------------------
  /** Fahrtmesser, the airspeed indicator. */
  { name: 'airspeed', build: (p) => createAirspeed(p) },
  /** Wendehorizont, the gyro artificial horizon. */
  { name: 'artificialHorizon', build: (p) => createHorizon(p) },
  /** Hoehenmesser, the altimeter. */
  { name: 'altimeter', build: (p) => createAltimeter(p) },

  // --- Flight group, second row ----------------------------------------
  /** Variometer, the vertical speed indicator. */
  { name: 'variometer', build: (p) => createVariometer(p) },
  /** Wendezeiger mit Libelle, the turn and slip indicator. */
  { name: 'turnSlip', build: (p) => createTurnSlip(p) },
  /** Kompass, the repeater of the remote reading compass. */
  { name: 'compass', build: (p) => createCompass(p) },

  // --- Flight group, bottom row ----------------------------------------
  /** Kraftstoffvorrat, the fuel contents gauge. */
  { name: 'fuel', build: (p) => createFuel(p) },
  /** Borduhr, the aircraft clock. */
  { name: 'clock', build: (p) => createClock(p) },
  /** AFN 2, the homing indicator of the FuG 16 ZY. */
  { name: 'homing', build: (p) => createHoming(p) },

  // --- Engine group. The left column reads the LEFT engine. ------------
  /** Drehzahlmesser, the rotor speed indicator. */
  { name: 'rpmLeft', build: (p) => createTachometer(p, 0) },
  { name: 'rpmRight', build: (p) => createTachometer(p, 1) },
  /** Abgastemperatur, the gas temperature indicator. */
  { name: 'gasTemperatureLeft', build: (p) => createGasTemperature(p, 0) },
  { name: 'gasTemperatureRight', build: (p) => createGasTemperature(p, 1) },
  /** Doppeldruckmesser, the fuel and oil pressure gauge. */
  { name: 'enginePressureLeft', build: (p) => createPressure(p, 0) },
  { name: 'enginePressureRight', build: (p) => createPressure(p, 1) },
];

/** The name of every gauge this panel fills. The unit test reads it. */
export const GAUGE_NAMES: readonly string[] = PANEL.map((entry) => entry.name);

/**
 * Build the live panel on the gauge discs of the virtual cockpit.
 *
 * `gauges` is the record that `createMe262Cockpit` returns. The caller keeps
 * the returned object for as long as the cockpit lives, and calls `dispose`
 * when the cockpit goes.
 */
export function createMe262Gauges(gauges: Record<string, Object3D>): CockpitGauges {
  const built: Instrument[] = [];

  for (const entry of PANEL) {
    const face = gauges[entry.name];
    // A bezel the interior does not carry is skipped. Read the module comment.
    if (face === undefined) continue;
    built.push(entry.build(createGaugeParts(face, entry.name)));
  }

  return {
    update(sample: TelemetrySample, readout: CockpitReadout, dt: number): void {
      for (const instrument of built) instrument.update(sample, readout, dt);
    },
    dispose(): void {
      for (const instrument of built) instrument.dispose();
      built.length = 0;
    },
  };
}
