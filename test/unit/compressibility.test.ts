import { describe, expect, it } from 'vitest';
import {
  CRITICAL_MACH,
  MACH_LIMIT,
  REFERENCE_SWEEP,
  TUCK_ONSET_MACH,
  createMachCorrection,
  machCorrection,
} from '@/physics/aero/compressibility';
import type { MachCorrection } from '@/physics/aero/compressibility';
import { toDeg } from '@/math/units';

const out: MachCorrection = createMachCorrection();
const other: MachCorrection = createMachCorrection();

/** Returns the free stream Mach number where the wave drag first appears. */
function dragRiseOnset(sweep: number): number {
  const probe = createMachCorrection();
  for (let mach = 0.5; mach <= 1.2; mach += 0.001) {
    machCorrection(mach, sweep, probe);
    if (probe.cdAdd > 0) {
      return mach;
    }
  }
  return Infinity;
}

describe('the reference constants', () => {
  it('match the published Me-262 numbers', () => {
    expect(TUCK_ONSET_MACH).toBe(0.83);
    expect(MACH_LIMIT).toBe(0.86);
    expect(CRITICAL_MACH).toBeLessThan(TUCK_ONSET_MACH);
    expect(toDeg(REFERENCE_SWEEP)).toBeCloseTo(18.5, 9);
  });

  it('leave the published level speed at 6000 m free of wave drag', () => {
    // 870 km/h is 241.7 m/s. The speed of sound at 6000 m is 316.4 m/s, so the
    // aircraft cruises at Mach 0.764.
    machCorrection(0.764, REFERENCE_SWEEP, out);
    expect(out.cdAdd).toBe(0);
    expect(out.acShift).toBe(0.25);
    expect(out.clMaxScale).toBeGreaterThan(0.95);
  });
});

describe('the Prandtl-Glauert lift scale', () => {
  it('is one at Mach zero', () => {
    machCorrection(0, 0, out);
    expect(out.clScale).toBe(1);
    machCorrection(0, REFERENCE_SWEEP, out);
    expect(out.clScale).toBe(1);
  });

  it('follows one over the square root of one minus Mach squared', () => {
    for (const mach of [0.2, 0.4, 0.6, 0.7]) {
      machCorrection(mach, 0, out);
      expect(out.clScale).toBeCloseTo(1 / Math.sqrt(1 - mach * mach), 12);
    }
  });

  it('stays finite at Mach one', () => {
    machCorrection(1, 0, out);
    expect(Number.isFinite(out.clScale)).toBe(true);
    expect(out.clScale).toBeLessThan(3);
    expect(out.clScale).toBeGreaterThan(1);
  });

  it('grows with Mach up to the critical Mach number', () => {
    let previous = 0;
    for (let mach = 0; mach <= CRITICAL_MACH; mach += 0.01) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.clScale).toBeGreaterThanOrEqual(previous - 1e-12);
      previous = out.clScale;
    }
  });

  it('peaks near the critical Mach number and then falls, as the shock takes over', () => {
    // The Prandtl-Glauert rule is a shock free result. Above the critical Mach
    // number a shock stands on the section, the pressure behind it stops
    // answering the angle of attack, and the measured lift curve slope rounds
    // over and falls. See SLOPE_LOSS_SCALE of compressibility.ts.
    let peak = 0;
    let peakMach = 0;
    for (let mach = 0; mach <= 1.0; mach += 0.005) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      if (out.clScale > peak) {
        peak = out.clScale;
        peakMach = mach;
      }
    }
    expect(peakMach).toBeGreaterThan(CRITICAL_MACH);
    expect(peakMach).toBeLessThan(TUCK_ONSET_MACH + 0.03);
    machCorrection(1.0, REFERENCE_SWEEP, out);
    expect(out.clScale).toBeLessThan(0.85 * peak);
  });

  it('gives a thin section a later peak than a thick one', () => {
    // The tailplane is 9 percent thick and the wing root is 11 percent. The thin
    // section meets its shock at a higher Mach number and keeps its slope longer.
    machCorrection(0.86, REFERENCE_SWEEP, out, 0.09);
    machCorrection(0.86, REFERENCE_SWEEP, other, 0.11);
    expect(out.clScale).toBeGreaterThan(other.clScale);
  });
});

describe('the wave drag rise', () => {
  it('is zero below the critical Mach number', () => {
    for (let mach = 0; mach <= CRITICAL_MACH; mach += 0.005) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.cdAdd).toBe(0);
    }
  });

  it('rises monotonically above the critical Mach number', () => {
    let previous = 0;
    for (let mach = CRITICAL_MACH; mach <= 1.0; mach += 0.002) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.cdAdd).toBeGreaterThanOrEqual(previous - 1e-15);
      previous = out.cdAdd;
    }
    expect(previous).toBeGreaterThan(0.2);
  });

  it('is large enough at the limit Mach number to stop level flight', () => {
    // Two Jumo 004 give about 7 kN in total at 6000 m at high speed. The dynamic
    // pressure at Mach 0.86 and 6000 m is 24.4 kPa, and the wing area is 21.7 m2,
    // so a cdAdd of 0.042 alone costs about 22 kN. Level flight cannot hold it.
    machCorrection(MACH_LIMIT, REFERENCE_SWEEP, out);
    expect(out.cdAdd).toBeGreaterThan(0.03);
    const extraDrag = 24_400 * 21.7 * out.cdAdd; // N
    expect(extraDrag).toBeGreaterThan(3 * 7000);
  });
});

describe('the aerodynamic center shift', () => {
  it('holds the quarter chord below the critical Mach number', () => {
    // The load moves when a shock stands on the section, so the shift starts at
    // the critical Mach number and not at the published tuck onset. The tuck
    // onset is where the WHOLE AIRCRAFT turns nose down, which comes later.
    // test/flight/mach.test.ts measures that one.
    for (let mach = 0; mach <= CRITICAL_MACH; mach += 0.005) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.acShift).toBeCloseTo(0.25, 12);
    }
  });

  it('has moved a third of the way to mid chord at the published tuck onset', () => {
    machCorrection(TUCK_ONSET_MACH, REFERENCE_SWEEP, out);
    expect(out.acShift).toBeGreaterThan(0.33);
    expect(out.acShift).toBeLessThan(0.47);
  });

  it('reaches mid chord by the Mach limit and stops there', () => {
    // A section in fully supersonic flow carries its load at mid chord. The
    // model does not put the aerodynamic center behind that point at any Mach
    // number, because no section does.
    machCorrection(MACH_LIMIT, REFERENCE_SWEEP, out);
    expect(out.acShift).toBeCloseTo(0.5, 6);
    for (let mach = MACH_LIMIT; mach <= 1.2; mach += 0.005) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.acShift).toBeLessThanOrEqual(0.5 + 1e-12);
    }
  });

  it('comes later on a thin section than on a thick one', () => {
    machCorrection(0.84, REFERENCE_SWEEP, out, 0.09);
    machCorrection(0.84, REFERENCE_SWEEP, other, 0.11);
    expect(out.acShift).toBeLessThan(other.acShift);
  });

  it('grows monotonically with Mach', () => {
    let previous = 0;
    for (let mach = 0; mach <= 1.0; mach += 0.002) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.acShift).toBeGreaterThanOrEqual(previous - 1e-15);
      previous = out.acShift;
    }
  });
});

describe('the loss of control power', () => {
  it('is one at Mach 0.75 and below', () => {
    for (let mach = 0; mach <= 0.75; mach += 0.005) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.controlScale).toBe(1);
    }
  });

  it('falls monotonically above Mach 0.75', () => {
    let previous = 1;
    for (let mach = 0.76; mach <= 1.0; mach += 0.001) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.controlScale).toBeLessThan(previous);
      previous = out.controlScale;
    }
  });

  it('reaches about 0.35 at the limit Mach number', () => {
    machCorrection(MACH_LIMIT, REFERENCE_SWEEP, out);
    expect(out.controlScale).toBeCloseTo(0.35, 6);
  });

  it('leaves the elevator weak where the tuck is strong', () => {
    // The trap that killed Me-262 pilots. At the limit Mach the nose down moment
    // is near its worst and the elevator has about a third of its authority.
    machCorrection(MACH_LIMIT, REFERENCE_SWEEP, out);
    expect(out.acShift).toBeGreaterThan(0.3);
    expect(out.controlScale).toBeLessThan(0.5);
  });
});

describe('the loss of peak lift', () => {
  it('is one at Mach 0.75 and below', () => {
    for (let mach = 0; mach <= 0.75; mach += 0.005) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.clMaxScale).toBe(1);
    }
  });

  it('falls monotonically above Mach 0.75', () => {
    let previous = 1;
    for (let mach = 0.76; mach <= 1.0; mach += 0.001) {
      machCorrection(mach, REFERENCE_SWEEP, out);
      expect(out.clMaxScale).toBeLessThan(previous);
      previous = out.clMaxScale;
    }
    expect(previous).toBeLessThan(0.5);
  });

  it('stops the pilot from pulling full lift at the limit Mach number', () => {
    machCorrection(MACH_LIMIT, REFERENCE_SWEEP, out);
    expect(out.clMaxScale).toBeLessThan(0.75);
    expect(out.clMaxScale).toBeGreaterThan(0.5);
  });
});

describe('sweep relief', () => {
  it('moves the drag rise to a higher free stream Mach number', () => {
    const straight = dragRiseOnset(0);
    const swept = dragRiseOnset(REFERENCE_SWEEP);
    expect(swept).toBeGreaterThan(straight);
    // The relief is the cosine of the sweep, so 18.5 degrees is worth about
    // 5 percent in Mach number.
    expect(swept / straight).toBeCloseTo(1 / Math.cos(REFERENCE_SWEEP), 2);
  });

  it('gives less drag at the same Mach number than a straight wing', () => {
    machCorrection(0.8, 0, out);
    machCorrection(0.8, REFERENCE_SWEEP, other);
    expect(other.cdAdd).toBeLessThan(out.cdAdd);
    expect(other.controlScale).toBeGreaterThan(out.controlScale);
    expect(other.clMaxScale).toBeGreaterThan(out.clMaxScale);
    expect(other.acShift).toBeLessThanOrEqual(out.acShift);
  });

  it('removes every Mach effect at 90 degrees of sweep', () => {
    machCorrection(0.95, Math.PI / 2, out);
    expect(out.clScale).toBeCloseTo(1, 12);
    expect(out.cdAdd).toBe(0);
    expect(out.acShift).toBe(0.25);
    expect(out.controlScale).toBe(1);
    expect(out.clMaxScale).toBe(1);
  });
});

describe('continuity and the call contract', () => {
  // A step in any correction would show up as a jolt in the cockpit. The test
  // sweeps Mach 0 to 1.2 in steps of 0.0005 and bounds every neighbor pair.
  const MAX_STEP = 0.01;

  for (const sweep of [0, REFERENCE_SWEEP]) {
    it(`every correction is continuous from Mach 0 to 1.2 at ${toDeg(sweep).toFixed(1)} degrees of sweep`, () => {
      machCorrection(0, sweep, other);
      for (let mach = 0.0005; mach <= 1.2; mach += 0.0005) {
        machCorrection(mach, sweep, out);
        expect(Math.abs(out.clScale - other.clScale)).toBeLessThan(MAX_STEP);
        expect(Math.abs(out.cdAdd - other.cdAdd)).toBeLessThan(MAX_STEP);
        expect(Math.abs(out.acShift - other.acShift)).toBeLessThan(MAX_STEP);
        expect(Math.abs(out.controlScale - other.controlScale)).toBeLessThan(MAX_STEP);
        expect(Math.abs(out.clMaxScale - other.clMaxScale)).toBeLessThan(MAX_STEP);
        other.clScale = out.clScale;
        other.cdAdd = out.cdAdd;
        other.acShift = out.acShift;
        other.controlScale = out.controlScale;
        other.clMaxScale = out.clMaxScale;
      }
    });
  }

  it('writes into the caller object and returns it', () => {
    const target = createMachCorrection();
    const returned = machCorrection(0.85, REFERENCE_SWEEP, target);
    expect(returned).toBe(target);
    expect(target.cdAdd).toBeGreaterThan(0);
  });

  it('treats a negative Mach number and a negative sweep as their magnitudes', () => {
    machCorrection(0.85, REFERENCE_SWEEP, out);
    machCorrection(-0.85, -REFERENCE_SWEEP, other);
    expect(other.clScale).toBe(out.clScale);
    expect(other.cdAdd).toBe(out.cdAdd);
    expect(other.acShift).toBe(out.acShift);
    expect(other.controlScale).toBe(out.controlScale);
    expect(other.clMaxScale).toBe(out.clMaxScale);
  });

  it('starts from the neutral low speed values', () => {
    const fresh = createMachCorrection();
    expect(fresh).toEqual({
      clScale: 1,
      cdAdd: 0,
      acShift: 0.25,
      controlScale: 1,
      clMaxScale: 1,
    });
  });
});
