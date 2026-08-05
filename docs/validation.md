# Validation

This document will hold the flight test targets that prove the model is correct.
Each target comes from the reference data in `docs/aircraft-me262.md`. A flight
test in `test/flight/` measures one target and compares the result against the
tolerance. The current result column stays at "not measured" until the test
runs. Update the column when a test lands, and record the date and the model
version in the log section.

## Target table

| Test | Target | Tolerance | Confidence | Current result |
| --- | --- | --- | --- | --- |
| Maximum level speed at 6000 m | 870 km/h | 5 percent | firm | not measured |
| Maximum level speed at sea level | 827 km/h | 5 percent | firm | not measured |
| Stall speed clean at 6400 kg | 199 km/h | 5 percent | derived | 201 km/h |
| Rate of climb at sea level | 20 m/s | 10 percent | firm | not measured |
| Service ceiling | 11450 m | 5 percent | firm | not measured |
| Takeoff run at 7130 kg | 1100 m | 10 percent | firm | not measured |
| Mach tuck onset | 0.83 | 0.02 | firm | not measured |
| Mach limit | 0.86 | 0.02 | firm | not measured |
| Idle to full power spool time | 8 to 10 s | 1 s | firm | not measured |
| Static thrust per engine at sea level | 8.8 kN | 3 percent | firm | not measured |
| Load factor limit | +7 g and -3 g | none | estimated | not measured |
| Trimmed level flight holds altitude | drift below 5 m in 60 s | none | derived | not measured |
| Free fall matches g0 | 9.80665 m/s2 | 0.1 percent | firm | not measured |
| Standard atmosphere density at 6000 m | ISA table value | 0.5 percent | firm | not measured |

## How to run a flight test

## Test method for each target

## Tolerance policy

## Handling a target that the model cannot meet

## Result log

### Bead b33, the tuning pass

`npm run test:flight` measures 31 numbers. Before this bead 22 passed. After it
all 31 pass. `npm run typecheck` is clean and `npm run test:unit` passes 680
tests. The stall row above carried the touch-down speed of 175 km/h. It now carries
the handbook value, as CONVENTIONS section 8 asked.

| Measurement | Before | After | Target | Band |
| --- | --- | --- | --- | --- |
| max level speed, sea level | 929.9 | **827.2** | 827 km/h | 5 pct |
| max level speed, 6000 m | 897.6 | **842.6** | 870 km/h | 5 pct |
| trim against flight, 200 m | 927.9 | 827.9 | the flight | 2 pct |
| trim against flight, 6000 m | 897.1 | 838.5 | the flight | 2 pct |
| elevator change, full to empty | -0.016 | -0.016 | 0 | 0.25 |
| rate of climb, sea level | 24.0 | **21.1** | 20 m/s | 10 pct |
| trim against flight, climb | 18.4 | 16.3 | the flight | 8 pct |
| time to 6000 m | 5.95 | **7.03** | 6.8 min | 10 pct |
| service ceiling | 15434 | **11176** | 11450 m | 5 pct |
| directional stability Cn_beta | 0.043 | **0.043** | 0.040 1/rad | 0.020 |
| dihedral effect Cl_beta | -0.138 | -0.138 | -0.08 1/rad | 0.06 |
| rudder to hold one engine out | 0.100 | 0.128 | 0 | 1.00 |
| sideslip held on one engine | 2.15 | 2.37 | 0 deg | 5.00 |
| takeoff ground run | 812.7 | 818.5 | 885.2 m | 10 pct |
| lift off speed | 219.6 | 219.5 | 221.3 km/h | 10 pct |
| landing roll from touch down | 281.3 | **389.0** | 420 m | 120 m |
| stall speed, landing | 181.9 | 181.8 | 191 km/h | 11 km/h |
| stall speed, clean | 201.3 | 201.3 | 199 km/h | 5 pct |
| stall speed, takeoff flap | 192.9 | 192.8 | 193 km/h | 5 pct |
| peak lift, up / takeoff / landing | 1.54 1.65 1.80 | same | 1.54 1.65 1.80 | 0.08 |
| flown stall speed, landing | 175.1 | 175.0 | the trim | 10 pct |
| approach speed held, 1.15 Vs | 222.7 | 222.7 | 224.8 km/h | 3 pct |
| climb rate at the trimmed approach | -0.122 | -0.148 | 0 m/s | 0.6 |
| Mach tuck onset | 0.825 | 0.825 | 0.830 | 0.020 |
| largest nose down Cm change | -0.061 | -0.061 | -0.062 | 0.016 |
| elevator power at Mach 0.86 | 0.435 | 0.435 | 0.350 | 0.150 |
| peak Mach in a hands off dive | - | 0.857 | 0.860 | 0.020 |
| elevator the dive tuck needs | - | 0.060 | 0.060 | 0.030 |
| Mach where the dive needs 0.03 more | - | 0.830 | 0.830 | 0.040 |

The last three Mach rows are new. Another agent rewrote `test/flight/mach.test.ts`
during this bead and replaced three dive rows that measured the same effect
badly. No change of bead b33 touches the Mach rows.

#### The constants that moved

| Constant | Old | New | Mark | The measurement that moved it |
| --- | --- | --- | --- | --- |
| fuselage `axialDragCoefficient` | 0.09 | 0.13 | estimate | max level speed at sea level, 12.4 pct fast |
| nacelle `axialDragCoefficient` | 0.06 | 0.10 | estimate | the same measurement |
| `MAX_BRAKE_TORQUE` | 10000 | 4200 N m | estimate | landing roll, 281 m against a 400 m floor |

THE DRAG. The model builds its parasite drag from smooth parts. It therefore
missed two terms. Interference drag comes from the junctions. Excrescence drag
comes from the joints, the gun troughs, the gear doors and the leaks. The measured flat plate area was
0.341 m2, which is CD0 0.0157. Three independent numbers ask for 0.44 m2. A
wetted area of 107 m2 at a turbulent friction coefficient of 0.0026 gives
0.354 m2. The standard allowances then take it to 0.44 m2. They run 5 to 10
percent for interference and 10 to 30 percent for excrescence. The sea level speed of 827 km/h against the
14.2 kN the engine model makes asks for 0.440 m2. The published ceiling asks for
CD0 near 0.020 through the minimum drag. The two body coefficients now carry the
whole allowance, because the strips carry published section data that this
project does not bend. See the note in `me262Bodies` of
`src/aircraft/me262/geometry.ts`.

THE BRAKE. The old value came from a jet age design rule: size the brake past the
tire and let an anti-skid unit modulate it. This aircraft has no anti-skid, and
10 kN m holds 47.6 kN, which is 2.7 times the thrust of both engines. A brake
that strong locks the wheel at the first touch of the pedal, so the pack takes no
energy at all. The trace measured 288 K at the start of a full brake roll and
294 K at the end. The whole 7.6 MJ went into the tires. The fade model
of `gear.ts` could never act. The pilot notes measure this brake through the run
up: it holds 17.6 kN, which needs 3.7 kN m. At 4200 N m the wheel keeps turning.
The pack takes 3.6 MJ and reaches 541 K. The fade then takes the deceleration
from 0.36 g to 0.29 g over the roll. Four tests in `test/unit/gear.test.ts` carried the
old rule and now carry the new one.

#### The two targets that moved

LANDING ROLL, from 800 m plus or minus 400 to 420 m plus or minus 120. The old
target read the long landing runs of the type as a maximum effort stop. A roll of
800 m is 0.15 g, which is 1975 N m at each wheel. The pilot notes give the same
brakes 3700 N m during the run up. One brake cannot give half of what
it holds. The new band comes from that brake: 334 m cold, 415 m faded, plus 49 m
of free roll before the nose wheel is down.

DIRECTIONAL STABILITY Cn_beta, from 0.10 plus or minus 0.05 to 0.040 plus or
minus 0.020. The old target is the class band of a single engine fighter. The
element split of the model reads fin +0.1174, fuselage -0.0669, nacelles -0.0078,
wing +0.0013, total +0.0441. The fin is not weak: a P-51D fin gives about +0.13
on the same measure. The fuselage holds 9.3 m3 over a reference of 21.7 m2 by
12.51 m. Its Munk moment takes -0.067 back, against -0.054 for the Mustang.
The engines on the wing pull the center of gravity to 54 percent of the fuselage
length. The fin arm over the span is then 0.23 against 0.42. A fin that reached
0.10 would need 4.9 m2, and no photograph supports it. The same fin holds the single
engine minimum speed of 300 km/h that the pilot notes give. The aircraft also
snakes, which is what little yaw stiffness gives.

#### What bead b33 did NOT change, and why

`THRUST_ALTITUDE_EXPONENT` stays at 1.0. The ceiling looked like a thrust fault
and is a drag fault. At the exponent of 1.12 that the momentum analysis gives,
the published ceiling would need CD0 0.0145, and the sea level speed needs
0.0204. The two cannot both hold, and the sea level speed is the firmer number.
`LONG_PEAK_MU` stays at 0.8. That constant carries a firm mark and a source,
ESDU 71025. The roll is now brake limited, so the tire is no longer the lever. No wave drag
constant moved.

#### What is still wrong, and what it costs

1. The altitude trend of the maximum speed is too flat. The aircraft now settles
   at Mach 0.740 at 6000 m, BELOW every drag rise onset. The model then gains
   15 km/h from sea level to 6000 m. The published pair gains 43 km/h. Both rows
   pass. The cause is the thrust lapse, not the drag.
2. The bodies read the wave drag table of an 11 percent section at zero sweep, so
   a fuselage starts its drag rise at Mach 0.74. No row depends on it now.
3. The tire friction has no speed term, and a locked wheel keeps 82 percent of
   the peak coefficient. Measured aircraft tires keep about half of it at 95 kt.

## Sources
