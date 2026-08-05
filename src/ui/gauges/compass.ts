/**
 * Kompass, the repeater of the remote reading compass.
 *
 *
 * THE REAL INSTRUMENT
 *
 * The Me 262 A-1a did not carry a magnetic compass on the main panel. It
 * carried the REPEATER of a Patin remote reading compass, which sits between
 * the altimeter and the AFN 2 homing indicator on every photograph of the
 * panel. The master unit stands far aft, away from the guns and the wiring,
 * and it drives this dial through a follow up motor.
 * Source: photographs of restored A-1a panels, confidence: firm on the fit,
 * estimate on the face.
 *
 *
 * A CARD, NOT A NEEDLE
 *
 * This dial turns a CARD and holds its index still, which is what a repeater
 * does. The pilot reads the heading against a fixed mark at the top, so the
 * number under the mark is the heading and nothing has to be added or taken
 * away.
 *
 * The card turns by PLUS the heading. On a heading of 090 the card must bring
 * O, for Ost, up to the index. O is painted a quarter turn CLOCKWISE from N on
 * the card, so the card itself must turn a quarter turn ANTICLOCKWISE, and an
 * anticlockwise turn about the face +z axis is a positive rotation.
 *
 *
 * WHY THE LAG IS THE LONGEST ON THE PANEL
 *
 * A follow up motor drives a geared card through a slipping clutch. It is the
 * slowest thing on the panel, and a pilot who rolls out of a turn watches the
 * card creep the last few degrees. A card that snapped would give away that
 * this is a repeater of a number and not a repeater of a compass.
 */

import { DEG } from '@/math/units';
import type { AttitudeAngles, TelemetrySample } from '@/ui/debug-overlay';
import { attitudeAngles } from '@/ui/debug-overlay';

import type { Instrument } from './instrument';
import type { CockpitReadout } from './readout';
import type { GaugeParts } from './parts';
import { GAUGE_Z } from './parts';
import type { NeedleLag } from './lag';
import { createLag, stepLagWrapped } from './lag';
import type { DialLaw } from './dial';
import { linearDial, tickValues } from './dial';
import {
  DIAL_COLOR,
  DIAL_STYLE,
  LARGE_FACE_PIXELS,
  caption,
  drawFace,
  fillFace,
  luminousDots,
  numeralRow,
  tickRow,
} from './draw';
import { Shape } from 'three/webgpu';

/** The card. A heading in DEGREES stands at that many degrees clockwise. */
export const COMPASS_CARD_LAW: DialLaw = linearDial(0, 360, 0, 360);

/**
 * Card time constant, s. A follow up motor with a slipping clutch is slow. The
 * value is an ESTIMATE, tuned so a roll out shows the card catching up.
 */
export const COMPASS_LAG = 1.5;

/** Scratch for the heading. The read function allocates nothing. */
const attitude: AttitudeAngles = { roll: 0, pitch: 0, heading: 0 };

/** The heading the card shows, rad, in the range 0 to 2 pi. */
export function compassReading(sample: TelemetrySample): number {
  return attitudeAngles(sample.state.orientation, attitude).heading;
}

/**
 * The rotation of the card at one heading, rad, ANTICLOCKWISE about face +z.
 * A heading of 90 degrees turns the card a quarter turn. Read the module
 * comment for the sign.
 */
export function compassCardRotation(heading: number): number {
  return heading;
}

/** The German cardinal letters. Ost is east and Sued is south. */
const CARDINALS: ReadonlyArray<{ at: number; text: string }> = [
  { at: 0, text: 'N' },
  { at: 90, text: 'O' },
  { at: 180, text: 'S' },
  { at: 270, text: 'W' },
];

function paintCard(): HTMLCanvasElement {
  return drawFace(LARGE_FACE_PIXELS, (f) => {
    fillFace(f);
    tickRow(
      f,
      COMPASS_CARD_LAW,
      tickValues(0, 355, 5),
      0.065,
      0.022,
      DIAL_COLOR.minor,
    );
    tickRow(
      f,
      COMPASS_CARD_LAW,
      tickValues(0, 350, 10),
      DIAL_STYLE.minorTick,
      DIAL_STYLE.minorWidth,
      DIAL_COLOR.mark,
    );
    tickRow(
      f,
      COMPASS_CARD_LAW,
      tickValues(0, 330, 30),
      DIAL_STYLE.majorTick,
      DIAL_STYLE.majorWidth,
      DIAL_COLOR.mark,
    );
    // The numerals run in tens of degrees, which is how every compass card of
    // the period is marked. A full 090 would not fit on a card this small.
    numeralRow(
      f,
      COMPASS_CARD_LAW,
      [30, 60, 120, 150, 210, 240, 300, 330],
      DIAL_STYLE.numeralRadius,
      DIAL_STYLE.numeral * 0.9,
      DIAL_COLOR.mark,
      (v) => String(Math.round(v / 10)),
    );
    for (const cardinal of CARDINALS) {
      const angle = cardinal.at * DEG;
      const r = DIAL_STYLE.numeralRadius;
      caption(
        f,
        cardinal.text,
        r * Math.sin(angle),
        r * Math.cos(angle),
        DIAL_STYLE.numeral * 1.15,
        cardinal.at === 0 ? DIAL_COLOR.luminous : DIAL_COLOR.mark,
      );
    }
    luminousDots(f, COMPASS_CARD_LAW, [0], 0.885, 0.034);
  });
}

/** The fixed index at the top of the dial, in face fractions. */
function indexShape(): Shape {
  const shape = new Shape();
  shape.moveTo(0, 0.90);
  shape.lineTo(0.075, 1.02);
  shape.lineTo(-0.075, 1.02);
  shape.closePath();
  return shape;
}

export function createCompass(parts: GaugeParts): Instrument {
  // The CARD carries the face, and the card turns. Nothing else on this dial
  // moves, so the index and the hair line hang straight on the face disc.
  const card = parts.addFace(paintCard());
  parts.addPlate('index', indexShape(), 0xe8b23c, GAUGE_Z.mask);

  const lag: NeedleLag = createLag(COMPASS_LAG, 0);

  return {
    update(sample: TelemetrySample, _readout: CockpitReadout, dt: number): void {
      const heading = stepLagWrapped(lag, compassReading(sample), Math.PI * 2, dt);
      card.rotation.z = compassCardRotation(heading);
    },
    dispose(): void {
      parts.dispose();
    },
  };
}
