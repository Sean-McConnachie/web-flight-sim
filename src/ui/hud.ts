/**
 * Head up display of the outside views.
 *
 * The panel shows what the pilot must know to fly the aircraft and to keep it
 * whole: speed, height, attitude load, both engines, the fuel, the gear, the
 * flaps and the ammunition. The cockpit view does not use it, because the
 * cockpit has real instruments. Bead b37 builds those.
 *
 *
 * 1. THE LAYOUT RULE
 *
 * A number that changes width makes the layout jump, and a jumping layout is
 * unreadable in flight. The same three rules that hold the debug overlay still
 * hold here, and this file uses the same `fixedWidth` function:
 *
 *   1. `fixedWidth` pads every value to a constant character count.
 *   2. The value cell uses `white-space: pre`, so the pad spaces survive.
 *   3. Every row is a grid with fixed column widths, in a monospace font with
 *      tabular figures.
 *
 *
 * 2. THE WARNINGS
 *
 * A pilot needs to see a problem before it becomes a failure. Every value that
 * can break the aircraft carries a level function. The level colors the cell,
 * and a matching alert prints across the top of the screen in words.
 *
 * Two levels: CAUTION for a value that is close to a limit, and WARNING for a
 * value that is past it. Section 4 below lists every threshold with its source.
 *
 * The rotor speed warning needs to know whether the lever is moving, because
 * the surge band is only dangerous while the fuel flow changes. This file holds
 * the last throttle position and reports the movement in a HudContext. Nothing
 * else in the display carries state between frames.
 *
 *
 * 3. WHERE THE VALUES COME FROM
 *
 * Everything that the debug overlay already carries arrives in a
 * `TelemetrySample`, so the display makes no second path into the physics. The
 * engines, the systems and the ammunition are not in that sample, so they
 * arrive in an `AircraftReadout`. That type is STRUCTURAL: it names the fields
 * the display reads and imports no physics type. The composition root fills it.
 *
 * CONVENTIONS section 2 says the model holds SI units and only src/ui converts.
 * Every conversion here goes through src/math/units.ts.
 *
 * This file may touch the DOM. CONVENTIONS section 4 allows that under src/ui.
 * It holds no physics.
 */

import { kelvinToCelsius, msToKmh, toDeg } from '@/math/units';
import { DANGER_BAND_RPM, TURBINE_INLET_TEMPERATURE_LIMIT } from '@/aircraft/me262/engine';
import { GEAR_LIMIT_SPEED, flapLimitSpeed } from '@/aircraft/me262/systems';
import type { AttitudeAngles, TelemetrySample } from '@/ui/debug-overlay';
import { attitudeAngles, fixedWidth } from '@/ui/debug-overlay';

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/** How close a value sits to its limit. */
export type HudLevel = 'normal' | 'caution' | 'warning';

/** What the display reads from one engine. */
export interface EngineReadout {
  /** Rotor speed, rev/min. */
  readonly rpm: number;
  /** Gas temperature at the turbine inlet, K. */
  readonly gasTemperature: number;
  /** Name of the engine state, such as "running". The display only prints it. */
  readonly state: string;
}

/**
 * What the display reads from the aircraft, less the part that the telemetry
 * sample already carries. The type is structural on purpose. See part 3.
 */
export interface AircraftReadout {
  readonly engines: readonly EngineReadout[];
  /** Throttle lever, 0 to 1. Both engines share it. */
  readonly throttle: number;
  /** Fuel on board, kg. */
  readonly fuelMass: number;
  /** 0 up and locked, 1 down and locked. A value between the two is travel. */
  readonly gearPosition: number;
  /** 0 up, 1 at the 50 degree landing setting. */
  readonly flapPosition: number;
  /** Rounds of 30 mm left, over all four guns. */
  readonly rounds: number;
}

/** What the level functions need and no single frame value can carry. */
export interface HudContext {
  /** True while the throttle lever moves. */
  throttleMoving: boolean;
  /** True while it moves toward more power. */
  throttleRising: boolean;
}

/** One printed cell. `read` returns the display value, not the SI value. */
export interface HudField {
  readonly key: string;
  readonly label: string;
  readonly unit: string;
  readonly decimals: number;
  /** Character count of the value cell. */
  readonly width: number;
  /** Which block of the display holds the cell. */
  readonly block: 'flight' | 'systems';
  read(s: TelemetrySample, a: AircraftReadout): number;
  /** Prints a word in place of the number. The word is padded to `width`. */
  text?(value: number, a: AircraftReadout): string;
  level?(s: TelemetrySample, a: AircraftReadout, c: HudContext): HudLevel;
}

/** One line of the alert strip. */
export interface HudAlert {
  readonly key: string;
  readonly text: string;
  level(s: TelemetrySample, a: AircraftReadout, c: HudContext): HudLevel;
}

export interface Hud {
  /** Copy one frame into the display. Call it one time per frame. */
  update(s: TelemetrySample, aircraft: AircraftReadout): void;
  /** Set to false to hide the display. It then does no work. */
  visible: boolean;
  dispose(): void;
}

// ---------------------------------------------------------------------------
// 4. The thresholds
// ---------------------------------------------------------------------------

/**
 * Rotor speed below which the engine has almost no surge margin.
 * DANGER_BAND_RPM of src/aircraft/me262/engine.ts holds the value of 6000 rpm.
 * Source: Me-262 pilot notes, confidence firm.
 *
 * A rotor below MIN_LIVE_RPM is not running at all, so it raises nothing. The
 * value sits well under the idle speed of 3000 rpm and over a windmilling
 * rotor at rest.
 */
const MIN_LIVE_RPM = 1200;

/**
 * Gas temperature that raises a caution, K. The limit itself is
 * TURBINE_INLET_TEMPERATURE_LIMIT, which is 1100 K. Full power gives 1015 K, so
 * a caution at 1060 K sits between the two and never shows in steady flight.
 */
const GAS_TEMPERATURE_CAUTION = 1060; // K

/**
 * Load factor limits. Source: CONVENTIONS section 8, confidence estimated.
 * The caution sits at 85 percent of each limit.
 */
const LOAD_LIMIT_POSITIVE = 7;
const LOAD_LIMIT_NEGATIVE = -3;
const LOAD_CAUTION_FRACTION = 0.85;

/**
 * Fuel that raises a caution and a warning, kg. Two engines at full power burn
 * 0.71 kg/s, so 250 kg is about six minutes of full power and 100 kg is about
 * two. Confidence: estimated from the fuel flow of the engine model.
 */
const FUEL_CAUTION = 250; // kg
const FUEL_WARNING = 100; // kg

/**
 * Mach limits. Source: CONVENTIONS section 8, confidence firm. Tuck starts at
 * 0.83 and 0.86 is the limit.
 */
const MACH_CAUTION = 0.83;
const MACH_WARNING = 0.86;

/**
 * Gear travel that counts as "out in the airstream". A door that is a little
 * open already takes the load, so the limit speed applies over the whole
 * travel and not only at the locked down end.
 */
const GEAR_OUT = 0.02;

/** Flap travel that counts as down, for the same reason. */
const FLAP_OUT = 0.02;

// ---------------------------------------------------------------------------
// The level functions. Every one is pure, so a test drives it with no DOM.
// ---------------------------------------------------------------------------

/** The higher of two levels. */
export function worstLevel(a: HudLevel, b: HudLevel): HudLevel {
  if (a === 'warning' || b === 'warning') return 'warning';
  if (a === 'caution' || b === 'caution') return 'caution';
  return 'normal';
}

/**
 * Level of one rotor speed.
 *
 * The band below 6000 rpm is dangerous only while the fuel flow changes. A
 * rotor that sits at idle with a still lever surges at nothing, so a still
 * lever reports normal. A lever that moves raises a caution, and a lever that
 * moves toward MORE power raises a warning, because that is the direction that
 * drives the fuel-air ratio past the surge line.
 */
export function rotorLevel(rpm: number, c: HudContext): HudLevel {
  if (rpm < MIN_LIVE_RPM || rpm >= DANGER_BAND_RPM) return 'normal';
  if (!c.throttleMoving) return 'normal';
  return c.throttleRising ? 'warning' : 'caution';
}

/** Level of one gas temperature, in kelvin. */
export function gasTemperatureLevel(kelvin: number): HudLevel {
  if (kelvin > TURBINE_INLET_TEMPERATURE_LIMIT) return 'warning';
  if (kelvin > GAS_TEMPERATURE_CAUTION) return 'caution';
  return 'normal';
}

/** Level of the airspeed against the gear limit. `eas` is in m/s. */
export function gearSpeedLevel(eas: number, gearPosition: number): HudLevel {
  if (gearPosition <= GEAR_OUT) return 'normal';
  if (eas > GEAR_LIMIT_SPEED) return 'warning';
  if (eas > GEAR_LIMIT_SPEED * 0.95) return 'caution';
  return 'normal';
}

/** Level of the airspeed against the flap limit. `eas` is in m/s. */
export function flapSpeedLevel(eas: number, flapPosition: number): HudLevel {
  if (flapPosition <= FLAP_OUT) return 'normal';
  const limit = flapLimitSpeed(flapPosition);
  if (eas > limit) return 'warning';
  if (eas > limit * 0.95) return 'caution';
  return 'normal';
}

/** Level of the load factor against the airframe limits. */
export function loadFactorLevel(n: number): HudLevel {
  if (n > LOAD_LIMIT_POSITIVE || n < LOAD_LIMIT_NEGATIVE) return 'warning';
  if (n > LOAD_LIMIT_POSITIVE * LOAD_CAUTION_FRACTION) return 'caution';
  if (n < LOAD_LIMIT_NEGATIVE * LOAD_CAUTION_FRACTION) return 'caution';
  return 'normal';
}

/** Level of the fuel on board, in kg. */
export function fuelLevel(kg: number): HudLevel {
  if (kg <= FUEL_WARNING) return 'warning';
  if (kg <= FUEL_CAUTION) return 'caution';
  return 'normal';
}

/** Level of the Mach number against the tuck onset and the limit. */
export function machLevel(mach: number): HudLevel {
  if (mach >= MACH_WARNING) return 'warning';
  if (mach >= MACH_CAUTION) return 'caution';
  return 'normal';
}

/** The worst rotor level over every engine. */
function anyRotorLevel(a: AircraftReadout, c: HudContext): HudLevel {
  let level: HudLevel = 'normal';
  for (const engine of a.engines) level = worstLevel(level, rotorLevel(engine.rpm, c));
  return level;
}

/** The worst gas temperature level over every engine. */
function anyGasTemperatureLevel(a: AircraftReadout): HudLevel {
  let level: HudLevel = 'normal';
  for (const engine of a.engines) {
    level = worstLevel(level, gasTemperatureLevel(engine.gasTemperature));
  }
  return level;
}

// ---------------------------------------------------------------------------
// The printed cells
// ---------------------------------------------------------------------------

/** Scratch for the heading row. The read functions allocate nothing. */
const attitude: AttitudeAngles = { roll: 0, pitch: 0, heading: 0 };

/** Word of one gear position. */
function gearText(position: number): string {
  if (position >= 1) return 'DOWN';
  if (position <= 0) return 'UP';
  return 'TRAV';
}

/** Word of one flap position. The three lever settings are up, take off, land. */
function flapText(position: number): string {
  if (position <= 0) return 'UP';
  if (position >= 1) return 'LAND';
  return 'T/O';
}

/** One engine cell. The index picks the engine and the closure holds it. */
function engineField(
  index: number,
  key: string,
  label: string,
  unit: string,
  decimals: number,
  width: number,
  read: (e: EngineReadout) => number,
  level?: (e: EngineReadout, c: HudContext) => HudLevel,
  text?: (e: EngineReadout) => string,
): HudField {
  /** An engine that is not in the list reads zero, so a one engine test runs. */
  const pick = (a: AircraftReadout): EngineReadout | undefined => a.engines[index];
  return {
    key,
    label,
    unit,
    decimals,
    width,
    block: 'systems',
    read: (_s, a) => {
      const engine = pick(a);
      return engine === undefined ? 0 : read(engine);
    },
    text:
      text === undefined
        ? undefined
        : (_v, a) => {
            const engine = pick(a);
            return engine === undefined ? '--' : text(engine);
          },
    level:
      level === undefined
        ? undefined
        : (_s, a, c) => {
            const engine = pick(a);
            return engine === undefined ? 'normal' : level(engine, c);
          },
  };
}

/**
 * Every printed cell, in the order it appears.
 *
 * The flight block runs down the right edge of the screen. The systems block
 * runs across the bottom in four columns, so the order below reads left to
 * right and then down.
 */
export const HUD_FIELDS: readonly HudField[] = [
  // --- The flight block ---------------------------------------------------
  {
    key: 'speed',
    label: 'speed',
    unit: 'km/h',
    decimals: 0,
    width: 4,
    block: 'flight',
    read: (s) => msToKmh(s.trueAirspeed),
  },
  {
    key: 'eas',
    label: 'eas',
    unit: 'km/h',
    decimals: 0,
    width: 4,
    block: 'flight',
    read: (s) => msToKmh(s.equivalentAirspeed),
    level: (s, a) =>
      worstLevel(
        gearSpeedLevel(s.equivalentAirspeed, a.gearPosition),
        flapSpeedLevel(s.equivalentAirspeed, a.flapPosition),
      ),
  },
  {
    key: 'mach',
    label: 'mach',
    unit: '',
    decimals: 3,
    width: 5,
    block: 'flight',
    read: (s) => s.mach,
    level: (s) => machLevel(s.mach),
  },
  {
    key: 'altitude',
    label: 'alt',
    unit: 'm',
    decimals: 0,
    width: 5,
    block: 'flight',
    // CONVENTIONS section 3.2: altitude is minus the NED z, never plus.
    read: (s) => -s.state.position.z,
  },
  {
    key: 'climb',
    label: 'v/s',
    unit: 'm/s',
    decimals: 1,
    width: 6,
    block: 'flight',
    read: (s) => -s.state.velocity.z,
  },
  {
    key: 'alpha',
    label: 'aoa',
    unit: 'deg',
    decimals: 1,
    width: 5,
    block: 'flight',
    read: (s) => toDeg(s.alpha),
  },
  {
    key: 'load',
    label: 'load',
    unit: 'g',
    decimals: 2,
    width: 5,
    block: 'flight',
    read: (s) => s.loadFactor,
    level: (s) => loadFactorLevel(s.loadFactor),
  },
  {
    key: 'heading',
    label: 'hdg',
    unit: 'deg',
    decimals: 0,
    width: 3,
    block: 'flight',
    read: (s) => toDeg(attitudeAngles(s.state.orientation, attitude).heading),
  },

  // --- The systems block --------------------------------------------------
  engineField(0, 'engine-1', 'eng 1', '', 0, 8, () => 0, undefined, (e) => e.state),
  engineField(0, 'rpm-1', 'rpm 1', 'rpm', 0, 4, (e) => e.rpm, (e, c) => rotorLevel(e.rpm, c)),
  engineField(0, 'egt-1', 'egt 1', 'C', 0, 4, (e) => kelvinToCelsius(e.gasTemperature), (e) =>
    gasTemperatureLevel(e.gasTemperature),
  ),
  {
    key: 'throttle',
    label: 'thr',
    unit: '%',
    decimals: 0,
    width: 3,
    block: 'systems',
    read: (_s, a) => a.throttle * 100,
  },

  engineField(1, 'engine-2', 'eng 2', '', 0, 8, () => 0, undefined, (e) => e.state),
  engineField(1, 'rpm-2', 'rpm 2', 'rpm', 0, 4, (e) => e.rpm, (e, c) => rotorLevel(e.rpm, c)),
  engineField(1, 'egt-2', 'egt 2', 'C', 0, 4, (e) => kelvinToCelsius(e.gasTemperature), (e) =>
    gasTemperatureLevel(e.gasTemperature),
  ),
  {
    key: 'fuel',
    label: 'fuel',
    unit: 'kg',
    decimals: 0,
    width: 4,
    block: 'systems',
    read: (_s, a) => a.fuelMass,
    level: (_s, a) => fuelLevel(a.fuelMass),
  },

  {
    key: 'gear',
    label: 'gear',
    unit: '',
    decimals: 0,
    width: 4,
    block: 'systems',
    read: (_s, a) => a.gearPosition,
    text: (v) => gearText(v),
    level: (s, a) => gearSpeedLevel(s.equivalentAirspeed, a.gearPosition),
  },
  {
    key: 'flap',
    label: 'flap',
    unit: '',
    decimals: 0,
    width: 4,
    block: 'systems',
    read: (_s, a) => a.flapPosition,
    text: (v) => flapText(v),
    level: (s, a) => flapSpeedLevel(s.equivalentAirspeed, a.flapPosition),
  },
  {
    key: 'ammo',
    label: 'ammo',
    unit: '',
    decimals: 0,
    width: 3,
    block: 'systems',
    read: (_s, a) => a.rounds,
  },
];

/**
 * Every alert of the strip, in the order of how fast it can break the
 * aircraft.
 */
export const HUD_ALERTS: readonly HudAlert[] = [
  {
    key: 'gas-temperature',
    text: 'TURBINE TEMP',
    level: (_s, a) => anyGasTemperatureLevel(a),
  },
  {
    key: 'rotor',
    text: 'SURGE BAND',
    level: (_s, a, c) => anyRotorLevel(a, c),
  },
  {
    key: 'gear-speed',
    text: 'GEAR OVERSPEED',
    level: (s, a) => gearSpeedLevel(s.equivalentAirspeed, a.gearPosition),
  },
  {
    key: 'flap-speed',
    text: 'FLAP OVERSPEED',
    level: (s, a) => flapSpeedLevel(s.equivalentAirspeed, a.flapPosition),
  },
  {
    key: 'load',
    text: 'G LIMIT',
    level: (s) => loadFactorLevel(s.loadFactor),
  },
  {
    key: 'mach',
    text: 'MACH LIMIT',
    level: (s) => machLevel(s.mach),
  },
  {
    key: 'fuel',
    text: 'FUEL LOW',
    level: (_s, a) => fuelLevel(a.fuelMass),
  },
];

/** Find one cell by its key. The unit test uses it. */
export function findHudField(key: string): HudField | undefined {
  for (const field of HUD_FIELDS) {
    if (field.key === key) return field;
  }
  return undefined;
}

/** Pad a word to a constant character count, so the column never moves. */
export function fixedText(text: string, width: number): string {
  const cut = text.length > width ? text.slice(0, width) : text;
  return cut.padStart(width, ' ');
}

// ---------------------------------------------------------------------------
// The throttle movement test
// ---------------------------------------------------------------------------

/**
 * Throttle change that counts as movement.
 *
 * src/core/config.ts sweeps the lever from closed to full in a few seconds, so
 * one frame at 60 Hz moves it by about 0.007. This value sits far below that
 * and far above the noise of a stick that rests against its center.
 */
const THROTTLE_MOVE_EPSILON = 5e-4;

/**
 * Frames that the movement flag holds after the lever stops.
 *
 * The flag is what turns the surge band caution on. A lever that moves in steps
 * would blink the caution on and off at the frame rate, which no eye can read.
 * The hold is counted in frames and not in seconds, because `update` receives
 * no time step.
 */
const THROTTLE_MOVE_HOLD = 30;

// ---------------------------------------------------------------------------
// The panel
// ---------------------------------------------------------------------------

const STYLE_ID = 'hfs-hud-style';

const CSS = `
.hfs-hud {
  position: absolute;
  color: #b6f0c8;
  font: 13px/1.5 ui-monospace, 'DejaVu Sans Mono', monospace;
  font-variant-numeric: tabular-nums;
  letter-spacing: 0.02em;
  text-shadow: 0 1px 3px rgba(0, 0, 0, 0.95);
  pointer-events: none;
  user-select: none;
}
.hfs-hud-flight {
  top: 12px;
  right: 12px;
  padding: 8px 10px;
  background: rgba(6, 14, 10, 0.42);
  border: 1px solid rgba(120, 200, 150, 0.35);
  border-radius: 4px;
}
.hfs-hud-systems {
  left: 12px;
  /* The telemetry chart of bead b26 owns the bottom right corner. It is 440 px
     wide, so the bar stops short of it and the two never overlap. */
  right: 448px;
  bottom: 12px;
  padding: 8px 10px;
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  column-gap: 14px;
  background: rgba(6, 14, 10, 0.42);
  border: 1px solid rgba(120, 200, 150, 0.35);
  border-radius: 4px;
}
.hfs-hud-alerts {
  top: 12px;
  left: 50%;
  transform: translateX(-50%);
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 3px;
  font-size: 15px;
  font-weight: 700;
  letter-spacing: 0.12em;
}
.hfs-hud-alert {
  display: none;
  padding: 2px 12px;
  border-radius: 3px;
  background: rgba(10, 6, 2, 0.62);
}
.hfs-hud-alert.caution {
  display: block;
  color: #ffc857;
  border: 1px solid rgba(255, 200, 87, 0.7);
}
.hfs-hud-alert.warning {
  display: block;
  color: #ff6b5e;
  border: 1px solid rgba(255, 107, 94, 0.85);
}
.hfs-hud-cell {
  display: grid;
  grid-template-columns: 52px auto 38px;
  align-items: baseline;
}
.hfs-hud-label {
  color: #79b795;
  text-transform: uppercase;
  font-size: 11px;
}
.hfs-hud-value {
  text-align: right;
  white-space: pre;
  color: #d8ffe6;
}
.hfs-hud-value.caution { color: #ffc857; }
.hfs-hud-value.warning { color: #ff6b5e; }
.hfs-hud-unit {
  padding-left: 6px;
  color: #5f9077;
  font-size: 11px;
}
`;

/** Add the style sheet one time, whatever the number of displays. */
function ensureStyle(): void {
  if (document.getElementById(STYLE_ID) !== null) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = CSS;
  document.head.appendChild(style);
}

function makeDiv(className: string, parent: HTMLElement): HTMLDivElement {
  const el = document.createElement('div');
  el.className = className;
  parent.appendChild(el);
  return el;
}

/** One built cell. `last` holds the printed text, so an equal value writes nothing. */
interface Row {
  readonly field: HudField;
  readonly cell: HTMLDivElement;
  last: string;
  lastLevel: HudLevel;
}

/** One built alert line. */
interface AlertRow {
  readonly alert: HudAlert;
  readonly element: HTMLDivElement;
  lastLevel: HudLevel;
}

/**
 * Build the display and attach it to `parent`.
 *
 * The display starts visible. The composition root sets `visible` from the
 * camera view, because the cockpit uses its own instruments.
 */
export function createHud(parent: HTMLElement): Hud {
  ensureStyle();

  const root = makeDiv('', parent);
  root.style.position = 'absolute';
  root.style.inset = '0';
  root.style.pointerEvents = 'none';

  const flight = makeDiv('hfs-hud hfs-hud-flight', root);
  const systems = makeDiv('hfs-hud hfs-hud-systems', root);
  const alerts = makeDiv('hfs-hud hfs-hud-alerts', root);

  const rows: Row[] = [];
  for (const field of HUD_FIELDS) {
    const row = makeDiv('hfs-hud-cell', field.block === 'flight' ? flight : systems);

    const label = makeDiv('hfs-hud-label', row);
    label.textContent = field.label;

    const cell = makeDiv('hfs-hud-value', row);
    const blank =
      field.text === undefined ? fixedWidth(0, field.width, field.decimals) : fixedText('', field.width);
    cell.textContent = blank;

    const unit = makeDiv('hfs-hud-unit', row);
    unit.textContent = field.unit;

    rows.push({ field, cell, last: blank, lastLevel: 'normal' });
  }

  const alertRows: AlertRow[] = [];
  for (const alert of HUD_ALERTS) {
    const element = makeDiv('hfs-hud-alert', alerts);
    element.textContent = alert.text;
    alertRows.push({ alert, element, lastLevel: 'normal' });
  }

  // State that lives between frames. See the throttle movement test above.
  let lastThrottle = 0;
  let moveHold = 0;
  let rising = false;
  const context: HudContext = { throttleMoving: false, throttleRising: false };

  let shownVisible = true;

  const api: Hud = {
    visible: true,

    update(s: TelemetrySample, aircraft: AircraftReadout): void {
      if (api.visible !== shownVisible) {
        shownVisible = api.visible;
        root.style.display = shownVisible ? '' : 'none';
      }
      if (!shownVisible) return;

      const change = aircraft.throttle - lastThrottle;
      lastThrottle = aircraft.throttle;
      if (Math.abs(change) > THROTTLE_MOVE_EPSILON) {
        moveHold = THROTTLE_MOVE_HOLD;
        rising = change > 0;
      } else if (moveHold > 0) {
        moveHold -= 1;
      }
      context.throttleMoving = moveHold > 0;
      context.throttleRising = context.throttleMoving && rising;

      for (const row of rows) {
        const field = row.field;
        const value = field.read(s, aircraft);
        const text =
          field.text === undefined
            ? fixedWidth(value, field.width, field.decimals)
            : fixedText(field.text(value, aircraft), field.width);
        // A write to textContent costs a layout pass, so only a change writes.
        if (text !== row.last) {
          row.cell.textContent = text;
          row.last = text;
        }
        const level = field.level === undefined ? 'normal' : field.level(s, aircraft, context);
        if (level !== row.lastLevel) {
          row.cell.className = level === 'normal' ? 'hfs-hud-value' : `hfs-hud-value ${level}`;
          row.lastLevel = level;
        }
      }

      for (const row of alertRows) {
        const level = row.alert.level(s, aircraft, context);
        if (level !== row.lastLevel) {
          row.element.className =
            level === 'normal' ? 'hfs-hud-alert' : `hfs-hud-alert ${level}`;
          row.lastLevel = level;
        }
      }
    },

    dispose(): void {
      root.remove();
      // The style sheet stays. A second display may still use it, and an unused
      // style sheet costs nothing.
    },
  };

  return api;
}
