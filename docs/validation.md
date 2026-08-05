# Validation

This document holds the flight test targets that prove the model is correct.
Each target comes from the reference data in `docs/aircraft-me262.md`. A flight
test in `test/flight/` measures one target and compares the result against a
tolerance.

**A target is not sacred. A target that cannot be reached, or that measures
itself, is a bug in the target.** This project has re-specified two targets and
three whole rows, each time with the evidence written down. The sections below
record those changes. A target can move for a good reason, or it can move to
make a test pass. A reader must be able to tell the two apart.

## How to run a flight test

```sh
npm run test:flight   # 26 tests, 31 measurements, about 25 s
npm run test:unit     # 846 tests, about 11 s
npm run typecheck     # the TypeScript compiler, no emit
```

Everything runs in Node with no GPU and no browser. `vite.config.ts` sets the
test environment to `node` for every test file. That is possible only because of
the separation rule of `docs/CONVENTIONS.md` section 4, and it is the reason
that rule exists. `docs/architecture.md` gives the layer rules in full.

Each flight test file prints a table at the end of its run. The columns are the
measured value, the target, the band, the delta and the result. A note under
each row states the condition of that measurement.

## Target table, current

Measured on the current model. All 31 rows pass.

### Level speed

| Test | Target | Band | Measured |
| --- | --- | --- | --- |
| Maximum level speed, sea level | 827 km/h | 5 pct | 823.2 |
| Maximum level speed, 6000 m | 870 km/h | 5 pct | 838.0 |
| Trim against flight, 200 m | the flight | 2 pct | 823.9 against 825.7 |
| Trim against flight, 6000 m | the flight | 2 pct | 833.9 against 838.0 |
| Elevator change, full fuel to empty | 0 | 0.250 | -0.014 |

### Climb, ceiling and static stability

| Test | Target | Band | Measured |
| --- | --- | --- | --- |
| Rate of climb, sea level | 20 m/s | 10 pct | 21.0 |
| Trim against flight, climb | the flight | 8 pct | 16.2 against 16.0 |
| Time to 6000 m | 6.8 min, see below | 10 pct | 7.07 |
| Service ceiling | 11450 m | 5 pct | 11137 |
| Directional stability Cn_beta | 0.040 per rad | 0.020 | 0.043 |
| Dihedral effect Cl_beta | -0.080 per rad | 0.060 | -0.139 |
| Rudder to hold one engine out | 0 | 1.00 | 0.131 |
| Sideslip held on one engine | 0 deg | 5.00 | 2.39 |

### Stall

| Test | Target | Band | Measured |
| --- | --- | --- | --- |
| Stall speed, landing configuration | 191 km/h | 11 km/h | 182.0 |
| Stall speed, clean | 199 km/h | 5 pct | 197.9 |
| Stall speed, takeoff flap | 193 km/h | 5 pct | 191.1 |
| Peak lift coefficient, flaps up | 1.54 | 0.080 | 1.58 |
| Peak lift coefficient, takeoff flap | 1.65 | 0.080 | 1.67 |
| Peak lift coefficient, landing flap | 1.80 | 0.080 | 1.80 |
| Flown stall speed, landing | the trim | 10 pct | 175.3 against 182.0 |
| Approach speed held | 1.15 Vs, see below | 3 pct | 222.9 against 225 |
| Climb rate at the trimmed approach | 0 m/s | 0.600 | -0.142 |

### Takeoff and landing

| Test | Target | Band | Measured |
| --- | --- | --- | --- |
| Takeoff ground run | 885.2 m | 10 pct | 806.9 |
| Lift off speed | 219.3 km/h | 10 pct | 218.0 |
| Landing roll from touch down | 420 m | 120 m | 390.3 |

### Mach

| Test | Target | Band | Measured |
| --- | --- | --- | --- |
| Mach tuck onset | 0.830 | 0.020 | 0.825 |
| Largest nose down Cm change | -0.066 | 0.017 | -0.066 |
| Elevator power at Mach 0.86 | 0.350 | 0.150 | 0.379 |
| Peak Mach in a hands off dive | 0.860 | 0.020 | 0.860 |
| Elevator the dive tuck needs | 0.060 | 0.030 | 0.054 |
| Mach where the dive needs 0.03 more elevator | 0.830 | 0.040 | 0.836 |

Four checks that a reader might expect here are UNIT tests and not flight tests.
Free fall against `g0` and the standard atmosphere density at 6000 m both live
in `test/unit/`. The static thrust per engine and the idle to full power spool
time live there too. They belong there, because each one has a known closed form
answer and needs no aircraft.

## Test method for each target

**Maximum level speed.** `trimMaxLevelSpeed` solves for the speed at full
throttle in level flight, in one Newton solve. A second row then FLIES the
aircraft with the autopilot and compares the flown speed against the solved
speed. That second row exists to catch a solver that has drifted away from the
model it claims to describe.

**Rate of climb and the ceiling.** `trimSteadyClimb` at full throttle over a
grid of altitudes, with no small angle assumption and with the thrust component
along the path included. The time to height is the trapezoid integral of `dh`
over the best climb rate in 1000 m steps. The ceiling is the height where the
best climb falls to 0.5 m/s.

**Stall speed.** A search over the angle of attack for the lowest speed at which
a 1 g level trim converges. The solver reports `converged: false` when no trim
exists. That flag IS the stall test itself. A separate row then flies the
aircraft into the break with the autopilot and reports the flown speed.

**Static stability.** A sideslip sweep at a fixed speed, with the moment
coefficients read off directly. `Cn_beta` and `Cl_beta` come from the slope.

**Takeoff and landing.** A full ground roll from the brake release, and a full
brake roll from touch down. Both run through the gear model, with the tire, the
brake torque and the brake fade acting.

**Time to 6000 m.** The 6.8 minute target is the published climb time of the
type. It is not in the reference table of `docs/aircraft-me262.md`, because that
table carries the sea level climb rate and the ceiling instead.

**The approach speed.** `Vs` is the TRIMMED stall speed of the landing
configuration at 1500 m, which the same test measures at 195.7 km/h. The target
is therefore 225 km/h and it moves with the model.

**The size of the tuck.** The target is 0.2 chord of neutral point travel times
the lift coefficient the sweep carries. The lift coefficient is measured, so the
target moves with the LIFT of the model and not with its MOMENT. That is why the
target and the measurement agree to three decimals. It is not a band fitted to
the answer.

**The Mach rows.** Four static sweeps at 8000 m and three flown dives. The
static sweeps hold the angle of attack and the elevator FIXED. A sweep at 1 g
would otherwise change the angle of attack with the speed and mix the two
effects.

## Tolerance policy

A band comes from the confidence of the target and from nothing else.

- A firm published number takes 5 percent, or 10 percent where the published
number is itself a range or depends on a technique.
- A derived number takes the width its inputs give it.
- An estimated number takes the width of the published class band.
- A row that compares the solver against the flown aircraft takes 2 percent,
because both are the same model and any gap is a defect.

**A band must never be set to the value the model happens to give.** Where the
model sits outside a band, there are two correct answers. Fix the model, or
re-specify the target with evidence and say so in this document.

## Handling a target that the model cannot meet

Three questions, in this order.

1. **Is the target right?** Check the source. Three numbers marked firm in the
reference table have now been wrong. `docs/aircraft-me262.md` records all three.
2. **Does the target measure what it says?** A target that uses the same
threshold to find an effect and to size it is circular. A target that asks for
something no airframe of the class can do is unreachable. Three rows in this
project were one or the other.
3. **Is the model wrong?** Only then. Change the model, and record which
measurement moved which constant.

## Targets this project RE-SPECIFIED, with the evidence

### The landing roll, from 800 m plus or minus 400 to 420 m plus or minus 120

The old target read the long landing runs of the type as a maximum effort stop.
A roll of 800 m is 0.15 g, which is 1975 N m at each wheel. **The pilot notes
measure the same brakes at 3700 N m during the engine run up.** The old target
therefore asked the model to reproduce a landing where the pilot used barely
half of the brake he had. That is a long landing and not a maximum effort stop.

The new band comes from that brake. A cold pack stops in 334 m and a faded pack
in 415 m. Both add 49 m of free roll before the nose wheel is down, which gives
383 m and 464 m. The band of 420 m plus or minus 120 m covers that pair with
room for the flare speed. The model measures 390 m, with the brake pack reaching
542 K and both tires whole.

### The directional stability, from 0.10 plus or minus 0.05 to 0.040 plus or minus 0.020

The old target was the class band of a SINGLE ENGINE fighter, and this aircraft
is not one. The element split of the model reads:

| Element | Cn_beta contribution |
| --- | --- |
| Fin | +0.1174 |
| Fuselage | -0.0669 |
| Nacelles | -0.0078 |
| Wing | +0.0013 |
| Total | +0.0441 |

**The fin is not weak.** A P-51D fin gives about +0.13 on the same measure. Two
things take the difference.

1. The fuselage holds 9.3 m3 over a reference of 21.7 m2 by 12.51 m. Its Munk
moment takes -0.067 back, against -0.054 for the Mustang.
2. The engines on the wing pull the center of gravity to 54 percent of the
fuselage length. The fin arm over the span is then 0.23 against 0.42.

A fin that reached 0.10 would need 4.9 m2, and no photograph supports it. The
same fin holds the single engine minimum speed of 300 km/h that the pilot notes
give. The aircraft also snakes, which is what little yaw stiffness gives.

## The three Mach rows that were rewritten

Three rows measured the tuck badly. One was unreachable and two were circular.
The replacements measure what the aircraft really did.

### 1. The elevator runs out at Mach 0.86. UNREACHABLE

The row measured the Mach number at which a full nose up elevator can no longer
make a nose up moment. The condition is the angle of attack of 1 g flight. The
row wanted that Mach number to be 0.86. Work the condition out. The moment about
the center of gravity is

```
Cm = Cm0 - staticMargin * CL + elevatorPower * elevator
```

so a full nose up elevator makes no nose up moment only when

```
staticMargin >= (Cm0 + elevatorPower) / CL
```

At 8000 m and Mach 0.86 the model carries `CL` 0.157 at 1 g. Its zero lift
moment is +0.041 and a full nose up elevator is worth +0.094. **The static
margin would have to reach 0.86 of the mean aerodynamic chord.** Even with a
zero lift moment of zero it would have to reach 0.60. Published transonic
neutral point travel for a swept wing aircraft is 0.15 to 0.25 chord, and this
model already gives 0.234.

The row was three to four times outside anything an aerodynamic center shift can
deliver. It could never pass and it never told anybody anything. The cause is
the LOW LIFT COEFFICIENT: the tuck moment is the static margin times `CL`. At
8000 m and Mach 0.86 there is almost no `CL` to work on.

The record also says the row described the wrong aircraft. Lindner flew dives to
Mach 0.86 and came back. The pull force rose to about 100 lb merely to hold the
dive angle, with violent buffet. That is a heavy stick and not a control that
had stopped working.

**The replacement** asserts that a full nose up elevator still makes a nose up
moment at the limit. It reports the load factor that goes with it, 3.7 g, as a
note and not as a table row. The model has no hinge moment, so it cannot report
the stick force that the pilot actually felt.

### 2. The size of the tuck. CIRCULAR

The target was `-TUCK_THRESHOLD`, which is the same 0.01 that the row above uses
to FIND the tuck onset. That asks the tuck to be exactly as big as the smallest
tuck the sweep can see. Any model with a real tuck must therefore overshoot and
fail.

**The replacement** takes the size of the tuck as the neutral point travel times
the lift coefficient it acts on. The published 0.15 to 0.25 chord and the
measured lift coefficient of the sweep then give the band, with no threshold in
it at all.

### 3. The Mach where the dive needs more elevator. CIRCULAR AGAINST ITS OWN PAIR

The threshold was 0.1 of the elevator travel, while the row that measures the
SIZE of the same reversal accepted anything from 0 to 0.2. **A model at the low
end of that band could never cross the threshold. One row passed and the other
failed on the same flight.**

**The replacement** puts the threshold on the LOWER EDGE of the accepted band of
its pair, at 0.03. Every model that passes the size row must now cross it.

## Result log

### The tuning pass

`npm run test:flight` measures 31 numbers. Before this pass 22 passed. After it
all 31 pass.

| Measurement | Before | After | Target | Band |
| --- | --- | --- | --- | --- |
| max level speed, sea level | 929.9 | 827.2 | 827 km/h | 5 pct |
| max level speed, 6000 m | 897.6 | 842.6 | 870 km/h | 5 pct |
| rate of climb, sea level | 24.0 | 21.1 | 20 m/s | 10 pct |
| time to 6000 m | 5.95 | 7.03 | 6.8 min | 10 pct |
| service ceiling | 15434 | 11176 | 11450 m | 5 pct |
| landing roll from touch down | 281.3 | 389.0 | 420 m | 120 m |
| directional stability Cn_beta | 0.043 | 0.043 | 0.040 1/rad | 0.020 |

Three constants moved, and each one names the measurement that moved it:

| Constant | Old | New | Mark | The measurement that moved it |
| --- | --- | --- | --- | --- |
| fuselage `axialDragCoefficient` | 0.09 | 0.13 | estimate | max level speed at sea level, 12.4 pct fast |
| nacelle `axialDragCoefficient` | 0.06 | 0.10 | estimate | the same measurement |
| `MAX_BRAKE_TORQUE` | 10000 | 4200 N m | estimate | landing roll, 281 m against a 400 m floor |

**THE DRAG.** The model builds its parasite drag from smooth parts, so it missed
interference drag and excrescence drag. The measured flat plate area was 0.341
m2, which is CD0 0.0157. Three independent numbers ask for 0.44 m2. A wetted
area of 107 m2 at a turbulent friction coefficient of 0.0026 gives 0.354 m2, and
the standard allowances take it to 0.44 m2. The sea level speed of 827 km/h
against the 14.2 kN the engine model makes asks for 0.440 m2. The published
ceiling asks for CD0 near 0.020 through the minimum drag. The two body
coefficients now carry the whole allowance, because the strips carry published
section data that this project does not bend. `docs/flight-model.md` gives the
whole build up.

**THE BRAKE.** The old value came from a jet age design rule. The full story is
in `docs/aircraft-me262.md` under the landing gear.

**What the tuning pass did NOT change, and why.** `THRUST_ALTITUDE_EXPONENT`
stays at 1.0. The ceiling looked like a thrust fault and it is a drag fault. At
the exponent of 1.12 that the momentum analysis gives, the published ceiling
would need CD0 0.0145, and the sea level speed needs 0.0204. The two cannot both
hold, and the sea level speed is the firmer number. `LONG_PEAK_MU` stays at 0.8,
because that constant carries a firm mark and a source, ESDU 71025. No wave drag
constant moved.

### The sweep correction

The sweep correction of `docs/aircraft-me262.md` moved several rows after the
tuning pass. Every strip meets a higher normal dynamic pressure, so the whole
wing carries 2.8 percent more peak lift.

| Measurement | Before the correction | Now |
| --- | --- | --- |
| max level speed, sea level | 827.2 | 823.2 |
| max level speed, 6000 m | 842.6 | 838.0 |
| service ceiling | 11176 | 11137 |
| stall speed, clean | 201.3 | 197.9 |
| stall speed, landing | 181.9 | 182.0 |
| elevator power at Mach 0.86 | 0.435 | 0.445 |
| landing roll from touch down | 389.0 | 390.3 |

Two constants answered the correction. The flap peak lift increment came down
from 1.2 to 1.1. That keeps the landing stall speed inside the handbook band of
180 to 202 km/h. The elevator control power anchors each moved by
`cos(18.5 deg) / cos(15.72 deg)`, which is 0.9852. The elevator therefore still
loses the same authority at the same FREE STREAM Mach number.

### The Mach tuck split correction

The refit of the section shift table, against the two new downwash laws, moved
the four Mach rows that read the tail. It moved nothing else.

| Measurement | Before | Now | Target | Band |
| --- | --- | --- | --- | --- |
| Mach tuck onset | 0.825 | 0.825 | 0.830 | 0.020 |
| largest nose down Cm change | -0.059 | -0.066 | -0.066 | 0.017 |
| elevator power at Mach 0.86 | 0.445 | 0.379 | 0.350 | 0.150 |
| elevator the dive tuck needs | 0.071 | 0.054 | 0.060 | 0.030 |
| Mach where the dive needs 0.03 more | 0.829 | 0.836 | 0.830 | 0.040 |

The elevator power row moved because the tail now loses dynamic pressure through
the drag rise. It sits closer to the target than it did.

## Known gaps

**The maximum speed gains too little with altitude.** Tracked as bead ole. The
published pair is 827 km/h at sea level and 870 km/h at 6000 m, a gain of 43
km/h. The model gives 823 and 838, a gain of 15 km/h. Both rows pass their 5
percent bands, so the fault is the TREND and not either point. The cause is the
thrust lapse and `docs/engine-jumo004.md` gives the analysis. The service
ceiling row is the constraint on any fix, because it falls with a steeper lapse.

**A body reads the wave drag table of an 11 percent wing at zero sweep.**
Tracked as bead 7el. A fuselage starts its drag rise at Mach 0.751 and pays the
full section value on its frontal area. No row depends on it today. The 6000 m
level speed settles at Mach 0.736, which is below that onset. It will matter
again to any work that touches the level speed at height or the dive.
`docs/flight-model.md` has the measurement.

**The tire friction has no speed term.** Tracked as bead fw3. A locked wheel
keeps 82 percent of the peak coefficient at any speed, and measured aircraft
tires keep about half of it at 95 kt. The brake limits the landing roll today,
so the tire is no longer the lever and no row moves. `docs/flight-model.md` has
the numbers.

**Three rows carry no published Me-262 target.** Each one says so in its own
note, and each measures the model against a rule of thumb or against itself.

1. **Lift off speed.** The target is 1.15 times the measured stall speed, which
is the usual lift off speed of the class.
2. **Takeoff ground run.** The published 1100 m is at 7130 kg, and the model
cannot be loaded past its internal fuel. The target scales that run by the
square of the weight, to 885.2 m at 6396 kg.
3. **Landing roll.** The section above derives the band from the brake torque
that the pilot notes measure.

Four more rows compare the trim solver against the flown aircraft. They have no
external target at all, and that is deliberate. Both sides are the same model,
so any gap between them is a defect and not a modeling choice.

## What this table cannot catch

**Every Mach row passed on a split that no real aircraft had.** That is the most
useful single fact in this document.

The total neutral point travel measured 0.2364 of the mean chord, inside the
published 0.15 to 0.25 band. The tuck onset measured 0.825 against the
documented 0.83. Every Mach row passed. A separate measurement then split the
travel by part. It found 97.2 percent of it on the wing section shift, where
NACA TN 3501 measures about a quarter of that. The downwash slope was moving the
WRONG WAY and the tail dynamic pressure ratio was not moving at all.

A model can hold every published number and still work by the wrong mechanism.
No target in this table could have found that, because no published Me-262
number reports the split. It took a measurement that nobody had asked for. The
correction landed, the split is now 64.9 percent on the wing, and the total
moved from 0.2364 to 0.2339 of the mean chord. `docs/flight-model.md` records
the before and the after in full.

## Sources

Every target comes from `docs/aircraft-me262.md`, which names the source of each
one, apart from the three rows listed above and the four solver rows.
