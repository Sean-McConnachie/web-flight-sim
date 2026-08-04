import { describe, expect, it } from 'vitest';
import {
  NACA_0009,
  NACA_0011,
  blendAirfoils,
  makeSymmetricAirfoil,
  thinAirfoilSlope,
} from '@/physics/aero/airfoil';
import type { AeroCoefficients, Airfoil } from '@/physics/aero/airfoil';
import { DEG, toDeg } from '@/math/units';

/** A scratch result. Every test reuses it, as the flight step does. */
const out: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
const other: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };

/** Returns the peak lift of the positive side and the angle of the peak, in degrees. */
function findPeak(airfoil: Airfoil): { clMax: number; alphaDeg: number } {
  let clMax = -Infinity;
  let alphaDeg = 0;
  const probe: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
  for (let deg = 0; deg <= 60; deg += 0.05) {
    airfoil.sample(deg * DEG, probe);
    if (probe.cl > clMax) {
      clMax = probe.cl;
      alphaDeg = deg;
    }
  }
  return { clMax, alphaDeg };
}

const sections: ReadonlyArray<{ airfoil: Airfoil; clMax: number; stallDeg: number }> = [
  { airfoil: NACA_0009, clMax: 1.25, stallDeg: 13 },
  { airfoil: NACA_0011, clMax: 1.35, stallDeg: 15 },
];

describe('symmetric section shape', () => {
  for (const s of sections) {
    it(`${s.airfoil.name} makes no lift and no moment at zero angle of attack`, () => {
      s.airfoil.sample(0, out);
      expect(out.cl).toBe(0);
      expect(Math.abs(out.cm)).toBeLessThan(1e-9);
      expect(out.cd).toBeCloseTo(s.airfoil.cdMin, 12);
      expect(s.airfoil.alphaZeroLift).toBe(0);
    });

    it(`${s.airfoil.name} has odd lift and moment and even drag`, () => {
      for (let deg = 0.5; deg <= 180; deg += 0.5) {
        s.airfoil.sample(deg * DEG, out);
        s.airfoil.sample(-deg * DEG, other);
        expect(other.cl).toBeCloseTo(-out.cl, 12);
        expect(other.cd).toBeCloseTo(out.cd, 12);
        expect(other.cm).toBeCloseTo(-out.cm, 12);
      }
    });

    it(`${s.airfoil.name} reads the same value one full turn later`, () => {
      for (const deg of [-170, -30, 0, 7, 13, 95, 176]) {
        s.airfoil.sample(deg * DEG, out);
        s.airfoil.sample(deg * DEG + 2 * Math.PI, other);
        expect(other.cl).toBeCloseTo(out.cl, 9);
        expect(other.cd).toBeCloseTo(out.cd, 9);
        expect(other.cm).toBeCloseTo(out.cm, 9);
      }
    });
  }
});

describe('attached flow', () => {
  for (const s of sections) {
    it(`${s.airfoil.name} holds its declared lift curve slope within 5 percent near zero`, () => {
      const step = 1 * DEG;
      s.airfoil.sample(step, out);
      s.airfoil.sample(-step, other);
      const slope = (out.cl - other.cl) / (2 * step);
      expect(slope).toBeGreaterThan(s.airfoil.clAlpha * 0.95);
      expect(slope).toBeLessThan(s.airfoil.clAlpha * 1.05);
    });
  }

  it('the declared slope is the thin airfoil value with the thickness correction', () => {
    // Thin airfoil theory gives 2 PI. The correction is (1 + 0.77 t/c).
    expect(NACA_0009.clAlpha).toBeCloseTo(2 * Math.PI * 1.0693, 6);
    expect(NACA_0011.clAlpha).toBeCloseTo(2 * Math.PI * 1.0847, 6);
    expect(thinAirfoilSlope(0)).toBeCloseTo(2 * Math.PI, 12);
  });
});

describe('the stall', () => {
  for (const s of sections) {
    it(`${s.airfoil.name} peaks at ${s.clMax} within 5 percent`, () => {
      const peak = findPeak(s.airfoil);
      expect(peak.clMax).toBeGreaterThan(s.clMax * 0.95);
      expect(peak.clMax).toBeLessThan(s.clMax * 1.05);
    });

    it(`${s.airfoil.name} peaks at ${s.stallDeg} degrees within 2 degrees`, () => {
      const peak = findPeak(s.airfoil);
      expect(Math.abs(peak.alphaDeg - s.stallDeg)).toBeLessThan(2);
      expect(toDeg(s.airfoil.alphaStall)).toBeCloseTo(s.stallDeg, 9);
    });

    it(`${s.airfoil.name} carries about 0.02 of drag at the lift peak`, () => {
      // Published section polars at Reynolds 3e6 give cd between 0.018 and 0.025
      // at maximum lift.
      s.airfoil.sample(s.airfoil.alphaStall, out);
      expect(out.cd).toBeGreaterThan(0.015);
      expect(out.cd).toBeLessThan(0.03);
    });

    it(`${s.airfoil.name} makes a nose down moment above the stall`, () => {
      s.airfoil.sample(s.airfoil.alphaStall + 5 * DEG, out);
      expect(out.cm).toBeLessThan(-0.02);
      // The moment stays near zero in attached flow.
      s.airfoil.sample(5 * DEG, other);
      expect(Math.abs(other.cm)).toBeLessThan(0.005);
    });
  }
});

describe('the flat plate limit', () => {
  for (const s of sections) {
    it(`${s.airfoil.name} makes no lift and a drag of 2.0 at 90 degrees`, () => {
      s.airfoil.sample(90 * DEG, out);
      expect(Math.abs(out.cl)).toBeLessThan(0.01);
      expect(out.cd).toBeCloseTo(2.0, 2);
    });

    it(`${s.airfoil.name} makes a lift of 1.0 at 45 degrees`, () => {
      // The flat plate law cl = 2 sin(alpha) cos(alpha) gives sin(90) = 1.
      s.airfoil.sample(45 * DEG, out);
      expect(out.cl).toBeCloseTo(1.0, 2);
      expect(out.cd).toBeCloseTo(1.0, 2);
    });
  }
});

describe('drag over the full circle', () => {
  for (const s of sections) {
    it(`${s.airfoil.name} keeps drag positive at every angle`, () => {
      for (let deg = -180; deg <= 180; deg += 0.1) {
        s.airfoil.sample(deg * DEG, out);
        expect(out.cd).toBeGreaterThan(0);
      }
    });

    it(`${s.airfoil.name} has its least drag at zero angle of attack`, () => {
      s.airfoil.sample(0, out);
      const cdAtZero = out.cd;
      expect(cdAtZero).toBeCloseTo(s.airfoil.cdMin, 12);
      for (let deg = -180; deg <= 180; deg += 0.1) {
        s.airfoil.sample(deg * DEG, out);
        expect(out.cd).toBeGreaterThanOrEqual(cdAtZero - 1e-12);
      }
    });
  }
});

describe('continuity of the joins', () => {
  // The curve joins an attached region, a separated region, and a flat plate
  // region. A step in the force at a join would shake the aircraft apart. The
  // test sweeps the full circle in steps of 0.1 degrees. No neighbor pair may
  // differ by more than these bounds.
  const MAX_STEP_CL = 0.02;
  const MAX_STEP_CD = 0.01;
  const MAX_STEP_CM = 0.005;

  for (const s of sections) {
    it(`${s.airfoil.name} has no step in cl, cd, or cm from -PI to PI`, () => {
      let worstCl = 0;
      let worstCd = 0;
      let worstCm = 0;
      s.airfoil.sample(-180 * DEG, other);
      for (let deg = -180 + 0.1; deg <= 180; deg += 0.1) {
        s.airfoil.sample(deg * DEG, out);
        worstCl = Math.max(worstCl, Math.abs(out.cl - other.cl));
        worstCd = Math.max(worstCd, Math.abs(out.cd - other.cd));
        worstCm = Math.max(worstCm, Math.abs(out.cm - other.cm));
        other.cl = out.cl;
        other.cd = out.cd;
        other.cm = out.cm;
      }
      expect(worstCl).toBeLessThan(MAX_STEP_CL);
      expect(worstCd).toBeLessThan(MAX_STEP_CD);
      expect(worstCm).toBeLessThan(MAX_STEP_CM);
    });
  }
});

describe('the builder and the blend', () => {
  it('the builder keeps the declared section data', () => {
    const section = makeSymmetricAirfoil({
      name: 'test 0010',
      thickness: 0.1,
      clMax: 1.3,
      alphaStall: 14 * DEG,
      cdMin: 0.0058,
    });
    expect(section.name).toBe('test 0010');
    expect(section.thickness).toBe(0.1);
    expect(section.cdMin).toBe(0.0058);
    expect(section.alphaZeroLift).toBe(0);
    expect(toDeg(section.alphaStall)).toBeCloseTo(14, 9);
    const peak = findPeak(section);
    expect(peak.clMax).toBeCloseTo(1.3, 2);
    expect(peak.alphaDeg).toBeCloseTo(14, 0);
  });

  it('the builder refuses a peak the Kirchhoff law cannot reach', () => {
    // A peak of 2.5 at 13 degrees needs more lift than the attached slope gives.
    expect(() =>
      makeSymmetricAirfoil({
        name: 'too strong',
        thickness: 0.09,
        clMax: 2.5,
        alphaStall: 13 * DEG,
        cdMin: 0.0055,
      }),
    ).toThrow();
    // A peak of 0.5 at 13 degrees needs more separation than the law allows.
    expect(() =>
      makeSymmetricAirfoil({
        name: 'too weak',
        thickness: 0.09,
        clMax: 0.5,
        alphaStall: 13 * DEG,
        cdMin: 0.0055,
      }),
    ).toThrow();
  });

  it('a blend at zero and at one matches the two source sections', () => {
    const atRoot = blendAirfoils(NACA_0011, NACA_0009, 0, 'root copy');
    const atTip = blendAirfoils(NACA_0011, NACA_0009, 1, 'tip copy');
    for (const deg of [0, 5, 13, 15, 25, 45, 90, -20]) {
      atRoot.sample(deg * DEG, out);
      NACA_0011.sample(deg * DEG, other);
      expect(out.cl).toBeCloseTo(other.cl, 12);
      expect(out.cd).toBeCloseTo(other.cd, 12);
      atTip.sample(deg * DEG, out);
      NACA_0009.sample(deg * DEG, other);
      expect(out.cl).toBeCloseTo(other.cl, 12);
      expect(out.cd).toBeCloseTo(other.cd, 12);
    }
  });

  it('a blend at one half lies between the root section and the tip section', () => {
    const mid = blendAirfoils(NACA_0011, NACA_0009, 0.5, 'mid span');
    expect(mid.thickness).toBeCloseTo(0.1, 12);
    expect(toDeg(mid.alphaStall)).toBeCloseTo(14, 9);
    const rootCoefficients: AeroCoefficients = { cl: 0, cd: 0, cm: 0 };
    for (let deg = -180; deg <= 180; deg += 1) {
      mid.sample(deg * DEG, out);
      NACA_0011.sample(deg * DEG, rootCoefficients);
      NACA_0009.sample(deg * DEG, other);
      expect(out.cl).toBeCloseTo((rootCoefficients.cl + other.cl) / 2, 12);
      expect(out.cd).toBeCloseTo((rootCoefficients.cd + other.cd) / 2, 12);
      expect(out.cm).toBeCloseTo((rootCoefficients.cm + other.cm) / 2, 12);
    }
  });
});

describe('the sample contract', () => {
  it('sample writes into the caller object and returns it', () => {
    const target: AeroCoefficients = { cl: 99, cd: 99, cm: 99 };
    const returned = NACA_0009.sample(6 * DEG, target);
    expect(returned).toBe(target);
    expect(target.cl).toBeGreaterThan(0.5);
    expect(target.cd).toBeLessThan(0.02);
  });
});
