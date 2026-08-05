# Messerschmitt Me-262 A-1a

This document holds every number that describes the aircraft, with the source
and the confidence mark of each one. `docs/CONVENTIONS.md` section 8 is the
authority when the two disagree, and it carries the notes on the numbers that
later work corrected.

**Do not invent a number that is missing. Estimate it, mark the estimate, and
record the method.** That rule is the reason this project found three wrong
numbers in its own reference table. Read the section on those three before you
trust any row below.

## Reference data

| Item | Value | Confidence |
| --- | --- | --- |
| Span | 12.51 m | firm |
| Wing area | 21.7 m2 | firm |
| Aspect ratio | 7.21 | derived |
| Length | 10.60 m | firm |
| Height | 3.83 m | firm, corrected, see below |
| Sweep at leading edge | 18.5 deg | firm, corrected, see below |
| Sweep at quarter chord | 15.72 deg | derived |
| Root airfoil | NACA 00011-0.825-35 | firm |
| Tip airfoil | NACA 00009-1.1-40 | firm |
| Empty mass | 3795 kg | firm |
| Loaded mass | 6396 kg | firm |
| Maximum takeoff mass | 7130 kg | firm |
| Thrust per engine, static, sea level | 8.8 kN | firm |
| Maximum rotor speed | 8700 rpm | firm |
| Idle rotor speed | 3000 rpm | firm |
| Idle to full power | 8 to 10 s | firm |
| Throttle danger band | below 6000 rpm | firm |
| Maximum level speed at 6000 m | 870 km/h | firm |
| Maximum level speed at sea level | 827 km/h | firm |
| Stall speed, gear and flaps down, full fuel | 180 to 202 km/h | firm, pilot handbook |
| Stall speed at 6400 kg, clean | about 199 km/h | derived, see below |
| Touch-down speed | 175 km/h | firm, this is NOT a stall speed |
| Rate of climb at sea level | 20 m/s | firm |
| Service ceiling | 11450 m | firm |
| Mach tuck onset | 0.83 | firm |
| Mach limit | 0.86 | firm |
| Takeoff run at 7130 kg | about 1100 m | firm |
| Load factor limit | +7 g and -3 g | estimated |
| Armament | 4 x MK 108, 30 mm | firm |

No source publishes the inertia tensor. The model builds one, and the section
below gives the values and the method.

## Three numbers in this table were WRONG

Three rows above once carried a value that a later measurement disagreed with.
The table marked every one of them FIRM at the time. **The record of how each
one was caught matters more than the corrected number. The method is what will
catch the fourth one.**

Each was caught the same way. A measurement disagreed with the table, somebody
checked the table instead of bending the model, and the table lost.

### The height. It was 3.50 m and the sources give 3.8 to 3.84 m

The table gave 3.50 m and marked it firm. The National Air and Space Museum
gives 12 ft 7 in, which is 3.84 m, for the A-1a airframe it holds. Three other
sources give 3.8 m to 3.84 m.

**The error did real damage before anybody found it.** The render model back
solved the fin span from the overall height, so the fin came out at 1.67 m
instead of 2.00 m. The small fin then gave the aircraft almost no directional
stability. Later work then adjusted its own fin estimate to match the 3.50 m
figure, which is the correct method used on a bad input.

Two unrelated measurements caught it.

1. The fin span read off a three view disagreed with the fin span the height
implied.
2. Full rudder could not hold one dead engine below 373 km/h. The pilot notes
warn against single engine flight below 300 km/h. With the 2.00 m fin the
aircraft holds down to 305 km/h.

The geometry now works the other way round. The ground line sits 1.33 m below
the fuselage reference plane and the fin root sits 0.5 m above it. A fin span of
2.00 m therefore puts the tip 3.83 m above the ground. The height is now an
OUTPUT of the fin span and not an input to it. The reference table carries that
3.83 m and not the 3.84 m of the museum, because 3.83 m is what this geometry
really builds. The two agree to 10 mm.

### The stall speed. 175 km/h is a touch-down speed

The widely repeated 175 km/h is NOT a stall speed. It is the touch-down speed
that Wendel recorded. A pilot touches down below the stall speed of the free air
wing, in ground effect, so the two numbers measure different things.

The "Pilot's Handbook for Me-262 A-1" gives the real numbers. With full fuel,
the gear down and the flaps down, the aircraft stalls at 202 km/h. That is the
HEAVIEST case. The same handbook gives 180 to 202 km/h across the load and
configuration range, so 180 km/h is the light end of the same band.

A model that stalls at 175 km/h with the flaps down is WRONG, not accurate. The
flight test measures against the handbook band. The flare row is a separate row
and it compares the flown speed against the trimmed speed, not against the
handbook.

At 6400 kg the wing loading is 2894 N/m2. The model gives:

| Configuration | Peak lift coefficient | Stall speed |
| --- | --- | --- |
| Clean | 1.583 at 20.3 deg | 198 km/h |
| Takeoff flap, 20 deg | 1.673 at 20.0 deg | 191 km/h |
| Landing flap, 50 deg | 1.797 at 19.5 deg | 182 km/h |

### The sweep. 18.5 degrees is the LEADING EDGE, not the quarter chord

The table gave 18.5 degrees as the sweep at the QUARTER CHORD and marked it
firm. It is the sweep of the LEADING EDGE.

Every source that gives the number gives it as a leading edge angle. Wikipedia
states a "shallow leading edge sweep of 18.5 degrees" and cites Loftin, "Quest
for Performance", NASA SP-468. The design history of the same article gives the
same angle. Messerschmitt swept the outer wing to 18.5 degrees on 1 March 1940.
The heavier engine had moved the center of gravity aft. That paragraph then
contrasts the panel with the inboard LEADING EDGE, which stayed straight until
the sixth prototype. No source gives 18.5 degrees at the quarter chord.

The plan form then fixes the quarter chord angle. The chord falls from 2.40 m at
the root to 1.07 m at the tip over a 6.255 m semi span, so

```
tan(sweep at c/4) = tan(18.5 deg) - 0.25 * (2.40 - 1.07) / 6.255
                  = 0.33460 - 0.05316 = 0.28144, that is 15.72 degrees
```

The two angles differ by 2.8 degrees, so the choice is not a rounding question.
Reading 18.5 degrees as a quarter chord angle puts the leading edge at 21.2
degrees, which no source gives.

**What the error cost, and how the model answered.** The model puts the center
of gravity at 25 percent of the mean aerodynamic chord. It had found that point
with the wrong angle, so the wing and the balance disagreed. With the wing root
quarter chord held at station 4.85 m, the correct sweep puts 25 percent of the
mean chord at station 5.618 m. That point sits 0.145 m AHEAD of the center of
gravity the model carries. Measured at 6000 m, the static margin falls from +3.4
percent of the mean chord to -4.2 percent. **The aircraft would be statically
UNSTABLE.**

Only two numbers can take up the 0.145 m: the root station or the center of
gravity. Three other places fix the center of gravity. The root station was an
ESTIMATE read off a three view. On that drawing 0.145 m is 1.4 percent of the
length of the aircraft, which is inside the reading error. The estimate is
therefore the one that moved. The root quarter chord station is now 4.992 m and
the model DERIVES it. The quarter chord line therefore passes through 25 percent
of the mean chord at the center of gravity by construction.

The static margin at sea level then reads 4.80 percent of the mean chord, where
it read 5.13 percent before.

The correction also cost a second number. Every strip now meets a higher normal
dynamic pressure, so the wing carries 2.8 percent more peak lift. The landing
stall speed fell to 179.2 km/h. That is below the handbook floor of 180. The
model answered with the flap, not with the wing. The peak lift the flap adds to
its own section came down from 1.2 to 1.1. Hoerner gives 1.0 to 1.3 for a
slotted flap, and a plain slot with no Fowler travel belongs at the low end.
Measured at each candidate value: 179.2 km/h at 1.2, 182.0 at 1.1, 183.5 at
1.05, and 184.9 at 1.0.

### What the three have in common

**A confidence mark is only as good as the source it names.** The table marked
all three rows firm, and none of them named a source. A firm mark on an angle
must also say WHERE the angle sits. A firm mark on a speed must say what the
speed measures.

**If a number in this table fights a measurement, check the number before you
change the model.** That is the rule that found all three.

## Geometry

Every position in the model is in body axes from the center of gravity, with x
forward, y right and z down. The map from a drawing station is

```
x = 5.76 - station
y = y
z = -0.1333 - height
```

where the height runs upward from the fuselage reference plane, which is the
plane through the wing root quarter chord. The center of gravity sits 0.133 m
BELOW that plane, because the two engines hang half a meter under it.

| Item | Value | Confidence |
| --- | --- | --- |
| Fuselage length | 10.60 m | firm |
| Center of gravity, from the nose | 5.760 m | derived, three files hold it |
| Center of gravity, above the reference plane | -0.1333 m | derived from the mass model |
| Ground line, below the reference plane | 1.33 m | from the render model |
| Fuselage volume | 9.3 m3 | derived from the section table |
| Nacelle front station | 3.91 m | from a three view |
| Nacelle length | 3.80 m | from a three view |
| Nacelle radius | 0.425 m | from a three view |
| Engine center line, span station | 2.05 m | estimate from photographs |
| Engine center line, below the plane | 0.53 m | estimate from photographs |

The fuselage section table holds 18 stations, each with a half width, a half
height and a center height. It is an ESTIMATE taken from a three view of the
A-1a. The same table serves the mass model and the aerodynamic body. The shape
the aerodynamics feels and the shape the mass model weighs are one shape.

The thrust line sits 0.397 m BELOW the center of gravity, so opening the
throttles raises the nose. One engine at full power with the other out gives
`8800 * 2.05 = 18 kN m` of yaw.

## Wing planform and twist

| Item | Value | Confidence |
| --- | --- | --- |
| Span | 12.51 m | firm |
| Area | 21.7 m2 | firm |
| Root chord | 2.40 m | derived, see below |
| Tip chord | 1.07 m | derived, see below |
| Mean aerodynamic chord | 1.820 m | derived |
| Span station of the mean chord | 2.728 m | derived |
| Sweep at the leading edge | 18.5 deg | firm |
| Sweep at the quarter chord | 15.72 deg | derived |
| Root quarter chord station | 4.992 m | derived, see the sweep note |
| Dihedral of the outer panel | 3.5 deg | estimate |
| Span station where the dihedral starts | 2.20 m | estimate |
| Root incidence | 1.5 deg | estimate from a three view |
| Tip incidence | 0 deg | estimate, so the washout is 1.5 deg |
| Oswald efficiency | 0.80 | derived |

A straight taper wing holds `S = (b / 2)(cr + ct)`, so the firm span and the
firm area fix the sum of the two chords at 3.469 m. The taper ratio of 0.446
comes from a three view, and the pair then follows.

The Oswald efficiency is not a free number. The Helmbold formula gives a lift
curve slope of 4.78 per radian at an aspect ratio of 7.21. Matching that against
the 6.5 per radian of the section needs `e = 0.80`.

**The washout buys less than it looks like it should.** At the clean peak lift
the three inboard strips sit at a separation point of 0.33 and the five slatted
strips sit at 0.84. With the washout taken to zero the same two numbers are 0.36
and 0.82. The SLAT holds the outer wing, and not the twist.

The wing is cut into 8 cosine spaced strips per side, with the boundaries at
`(b/2) sin(k PI / 16)`. The widths run 1.220, 1.173, 1.081, 0.948, 0.778, 0.578,
0.356 and 0.120 m from the root to the tip. `docs/flight-model.md` explains the
spacing.

### Tailplane and fin

| Item | Value | Confidence |
| --- | --- | --- |
| Tailplane root quarter chord station | 8.905 m | from a three view |
| Tailplane exposed span, one side | 1.80 m | from a three view |
| Tailplane root chord | 1.42 m | from a three view |
| Tailplane tip chord | 0.78 m | from a three view |
| Tailplane sweep at the quarter chord | 12 deg | from a three view |
| Tailplane height above the reference plane | 0.70 m | from a three view |
| Tailplane gross span | 5.00 m | from a three view |
| Tailplane gross area, carry through included | 5.95 m2 | derived |
| Tailplane aspect ratio | 4.2 | derived |
| Tailplane incidence | 0 deg | see below |
| Fin root quarter chord station | 8.1175 m | from a three view |
| Fin root height above the plane | 0.50 m | from a three view |
| Fin span | 2.00 m | derived from the corrected height |
| Fin root chord | 2.55 m | from a three view |
| Fin tip chord | 1.15 m | from a three view |
| Fin quarter chord sweep | 30.7 deg | estimate from a three view |
| Fin area | 3.70 m2 | derived from the strips |
| Fin effective aspect ratio | 1.97 | derived, see below |

The tailplane sits at ZERO incidence and the wing sits at 1.5 degrees to the
fuselage datum. The tail therefore already works at 1.5 degrees less than the
wing, which is the offset a fixed tailplane would otherwise need. The real
aircraft trimmed with an electric stabilizer on top of that.

**The fin effective aspect ratio is 1.97 against a geometric 1.08.** The
fuselage and the tailplane both act as end plates. The DATCOM build up is

```
A_eff = [A_v(B) / A_v] * A_v * (1 + K_H * (A_v(HB) / A_v(B) - 1))
```

The first factor comes from the fin span over the body depth, 2.00 / 1.11 =
1.80, which gives 1.65. The tailplane ratio is 1.10 and `K_H` comes from the
gross tailplane area over the fin area, 5.95 / 3.70 = 1.61, which gives 1.08.
The product is 1.97. The Helmbold form at that aspect ratio and the 22.7 degree
half chord sweep gives a fin lift curve slope of 2.56 per radian. The assembled
model measures 2.9 per radian. The two agree to 13 percent. Source: USAF DATCOM
5.3.1.1, as fitted in Mason, "Stability and Control Derivative Estimation and
Engine-Out Analysis". Confidence: derived.

**The tail volume coefficients.** The horizontal is 0.33, which sits in the 0.30
to 0.60 band of a single seat fighter of the period. The vertical is 0.040 on
the convention this model uses, where the fin area starts at the root. Raymer
and Roskam quote 0.04 to 0.07 for a fighter with the fin area carried to the
fuselage center line. The same aircraft reports 0.049 on that convention.

**The tail arm is short for a fighter, and that is a real feature of this
layout.** The fin arm over the span is 0.23 against 0.42 for a Mustang. The two
engines hang on the wing and pull the center of gravity to 54 percent of the
fuselage length instead of 45 percent. This is why the measured `Cn_beta` is
0.043 and not the 0.10 of a single engine fighter.

## Control surface sizes and travel

Spans, in meters from the plane of symmetry:

| Surface | Span |
| --- | --- |
| Inner flap panel | 0.62 to 1.56 |
| Outer flap panel | 2.50 to 3.38 |
| Aileron | 4.00 to 5.98 |
| Slat, outer panel only | 3.00 to 6.02 |
| Elevator, from the tailplane root | 0.30 to 1.70 |
| Rudder, from the fin root | 0.25 to 1.85 |

The nacelle splits the flap into two panels and cuts it to 29 percent of the
span. This aircraft therefore gains less from its flaps than a single engine
fighter gains.

Travel:

| Surface | Full travel | Confidence |
| --- | --- | --- |
| Aileron | 0.35 rad, 20.1 deg | estimate, from a three view |
| Elevator | 0.44 rad, 25.2 deg | estimate, from a three view |
| Rudder | 0.44 rad, 25.2 deg | estimate, from a three view |
| Flap, takeoff | 20 deg | firm, pilot handbook |
| Flap, landing | 50 deg | firm, pilot handbook |
| Nose wheel steering | 30 deg | estimate |

The flap of the A-1a carried graduations at 0, 10, 20, 30, 40 and 50 degrees on
its upper surface. A red mark shows the 20 degree takeoff setting. Source:
"Pilot's Handbook for Me-262 A-1", section 2, wing flaps. Confidence: firm.

Effectiveness, as the flap effectiveness `tau`:

| Surface | Hinge at | Chord ratio | Theoretical tau | Model tau |
| --- | --- | --- | --- | --- |
| Aileron | 72 pct chord | 0.28 | 0.52 | 0.44 |
| Elevator | 68 pct chord | 0.32 | 0.56 | 0.45 |
| Rudder | 62 pct chord | 0.38 | 0.61 | 0.48 |

The gap and the boundary layer take 15 to 21 percent off the theoretical value.
Source: Perkins and Hage, flap effectiveness chart. Confidence: derived.

The slat raises the stall angle of the section it covers by 6 degrees. It opens
at a local angle of attack of 8 degrees. Both are estimates. The 6 degrees comes
from Hoerner: a leading edge slat of this size moves the section stall from
about 13 degrees to about 19 degrees. The 8 degrees has an independent check.
The handbook says the slots open at 300 km/h in a glide and at 450 km/h in a
climb or a turn. At 6400 kg and 300 km/h in a 1 g glide the aircraft carries a
lift coefficient of 0.68. At 450 km/h it reaches the same coefficient at 2.2 g.
One angle of attack explains both numbers.

## Mass and balance

| Item | Value | Confidence |
| --- | --- | --- |
| Empty mass | 3795 kg | firm |
| Loaded mass | 6396 kg | firm |
| Maximum takeoff mass | 7130 kg | firm |
| Fuel capacity | 2133 kg | medium, derived from 2570 liters |
| Ammunition, 360 rounds | 176 kg | derived |
| Dry mass, no fuel, full ammunition | 4263 kg | derived |

The fuel capacity is 2570 liters over four tanks. J2 was a coal derived gas oil
with a density near 0.83 kg per liter, so the mass is 2133 kg.

The model derives the ammunition mass. The MK 108 fired the 30x90RB round. A
complete round weighs about 0.48 kg, so 360 rounds are 173 kg. The model carries
176 kg, which leaves 3 kg for the belt links and the boxes. **The source comment
allows ten percent for those, and ten percent would give 190 kg.** The model
uses 176 kg, so the stated allowance does not match the number it produced.

### The weight statement

**Every group is an ESTIMATE except the bare engine mass and the gun mass.** The
groups follow a normal weight statement for a fighter of this size, and they sum
to the firm empty mass exactly.

| Group | Mass | The method |
| --- | --- | --- |
| Wing group | 760 kg | 10.7 percent of the maximum takeoff mass |
| Engine installation | 1700 kg | two Jumo 004 B-1 at 719 kg, plus 131 kg each of nacelle, mount, plumbing and controls |
| Fuselage structure | 470 kg | shell, frames, longerons and the tail cone |
| Tail group | 165 kg | 2.3 percent of the maximum takeoff mass |
| Landing gear | 310 kg | 4.3 percent of the maximum takeoff mass |
| Armament group | 245 kg | four MK 108 at 58 kg, plus mounts and feed |
| Systems | 145 kg | flight controls, hydraulics, electrics |
| Total | 3795 kg | the firm empty mass |

Two further lumps carry the loaded mass:

- **Pilot, seat and parachute, 100 kg** at station 4.3 m. Estimate, medium
confidence.
- **Equipment, 192 kg** at station 7.2 m. This item CLOSES the published loaded
mass. The empty mass, the pilot, full ammunition and full fuel add up to 6204
kg, and the published loaded mass is 6396 kg. The 192 kg that are left cover the
engine oil and the FuG 16ZY and FuG 25a radio sets. They also cover the oxygen
bottles, the gun camera and the ammunition boxes. Derived, with low confidence
on the split and firm confidence on the total.

**The single largest lever on the balance is the engine station, and it is an
estimate with LOW confidence.** The nacelle runs from station 3.91 m to station
7.71 m. A mass breakdown of the Jumo 004 B-1 puts the engine center of mass near
half its length. The eight stage compressor forward and the turbine with its jet
pipe aft nearly balance. The installed unit sits a little further aft, because
the rear of the nacelle carries the jet pipe, the movable exhaust bullet and its
actuator. The model uses station 6.15 m.

### The center of gravity

The center of gravity is 5.760 m from the nose at the loaded mass. THREE files
carry that number, because `docs/CONVENTIONS.md` section 4 stops the physics
from importing the renderer:

- `src/render/models/me262.ts` derives it from the plan form alone, as 25
percent of the mean aerodynamic chord.
- `src/aircraft/me262/mass.ts` repeats it, and its lumped mass model reaches the
same point on its own to within a millimeter.
- `src/physics/gear.ts` repeats it again.

A unit test asserts the literal 5.76 in all of them, so the three cannot drift
apart in silence.

The center of gravity moves 3.6 mm over a full fuel burn, which is 0.20 percent
of the mean chord. The elevator to trim changes by 0.014 of its travel between
full and empty. The tank stations were chosen to give that small travel, and a
flight test row holds it.

## The fuel tanks, and the weakest number in the mass model

| Tank | Capacity | Station | Burn order |
| --- | --- | --- | --- |
| Forward main | 747 kg, 900 liters | 3.90 m | second |
| Rear main | 747 kg, 900 liters | 6.50 m | second |
| Rear auxiliary | 498 kg, 600 liters | 7.80 m | first |
| Cockpit floor | 141 kg, 170 liters | 4.60 m | last |

Pilots were told to burn the rear auxiliary tank first, because the aircraft was
tail heavy while that tank was full. The burn order follows that instruction.

**No drawing in the reference set gives the station of any one tank.** The four
stations above sit at the AFT edge of the space the fuselage offers. That choice
is deliberate and it is the weakest number in the mass model. A layout with the
tanks half a meter further forward puts the loaded center of gravity near 5.55
m. That point is 11 percent of the mean chord ahead of the quarter chord point.
It gives a static margin near 26 percent. The aircraft has to balance where the
plan form says it balances, so the model holds the tanks aft.

40 kg of the 2133 kg is unusable, which is the usual two percent allowance. The
fuel unports under negative g. The model takes the feed away below 0 g with full
tanks and below 0.5 g with almost empty tanks. That fuel term is not decoration.
A rule with NO fuel term took the feed away from a FULL aircraft. That happened
in the dive entry of the Mach test, where the autopilot pushes over to -1.34 g
for three seconds. The engines flamed out and the peak Mach row moved from 0.857
to 0.850.

## Inertia tensor and how the estimate came about

**No source publishes the inertia tensor of this aircraft, so the model builds
one.** The method is a LUMPED MASS MODEL:

```
I = sum over lumps of  m * (r_own^2 + d^2)
```

with the parallel axis theorem carrying each lump to the whole aircraft center
of gravity. The CENTER OF GRAVITY comes out of the same sum, so the balance and
the inertia can never disagree with each other.

`Ixy` and `Iyz` are zero by symmetry and the model does not compute them. The
model does compute `Ixz`, and it is not small, because the tail stands above the
engines.

At the loaded mass of 6396 kg with full fuel:

| Term | Value |
| --- | --- |
| Ixx | 14660 kg m2 |
| Iyy | 23214 kg m2 |
| Izz | 34950 kg m2 |
| Ixz | 550 kg m2 |

At the dry mass of 4263 kg the three diagonal terms fall to 14306, 17252 and
29384 kg m2. Burning the fuel lowers every moment of inertia, and a unit test
checks that it does.

The wing lumps follow a structural law and not a uniform spread. The mass of a
wing per unit span follows the area of its structural box. That area is the
local chord times the local thickness, times the chord again for the skin that
carries it. The weight therefore falls as the chord squared times the thickness
ratio. A section carries a front spar, a rear spar, a skin and a movable
surface. Its structural center of mass sits near 46 percent of the chord.

The fuselage lumps follow the local perimeter, with a 0.9 aft bias. The bias
stands for the fin carry through, the tailplane carry through, the frames that
take the tail loads, and the rear tank bay. That bias is an estimate with low
confidence.

### How the model checks the tensor

**Against the radii of gyration of a fighter of the same size.** The tensor is
an estimate, so it cannot be checked against a published Me-262 number. It can
be checked against the SHAPE that fighters of this class hold.

| Ratio | The model | The P-51D | The accepted band |
| --- | --- | --- | --- |
| kx / span | 0.121 | 0.141 | 0.09 to 0.16 |
| ky / length | 0.180 | 0.181 | 0.14 to 0.22 |

The Me-262 holds no fuel in its wings, so its roll radius should run LOWER than
the Mustang, and it does. The pitch radius should match, and it does to one part
in 180.

Five more unit tests check that the tensor is a valid tensor at all. They test
exact symmetry, a product of inertia above 100 kg m2 at full fuel, and Sylvester
positive definiteness including `Ixx Izz - Ixz^2 > 0`. They also test the three
triangle inequalities and successful inversion.

## Landing gear geometry

| Item | Value | Confidence |
| --- | --- | --- |
| Nose axle station | 2.18 m | from the render model |
| Main axle station | 6.08 m | from the render model |
| Main track, half | 1.18 m | from the render model |
| Nose wheel radius | 0.33 m | from the 660 x 160 tire |
| Main wheel radius | 0.42 m | from the 840 x 300 tire |
| Center of gravity height at rest | 1.1967 m | derived from the ground line |
| Nose load share at rest | 8.2 percent | derived from the moment balance |
| Strut travel, each leg | 0.28 m | estimate |
| Static stroke | 55 percent of travel | estimate |
| Nose tire rate | 300 kN/m | estimate |
| Main tire rate | 700 kN/m | estimate |
| Brake torque per main wheel | 4200 N m | derived, see below |

The nose load share follows from one moment balance: `0.32 / 3.90 = 0.0821`. A
nose share below 6 percent gives no steering grip. A share above 20 percent
needs a very strong nose leg. This layout sits where a tricycle layout should
sit.

The strut travel of 0.28 m is an estimate. A fighter oleo of this size runs 0.2
to 0.3 m. The stroke has to swallow the energy of a 3 m/s touchdown inside the
design load factor. It does, using about three quarters of the travel.

The tire rates come from the Roskam part IV tire tables. A tire of this size
deflects about a third of its section height at its rated load.

**The brake torque is a corrected number and the correction was large.** The old
value of 10000 N m came from a jet age design rule. The rule is to size the
brake past the tire and let an anti-skid unit modulate it. This aircraft has no
anti-skid, and 10 kN m holds 47.6 kN, which is 2.7 times the thrust of both
engines. A brake that strong locks the wheel at the first touch of the pedal, so
the pack takes no energy at all. The trace measured 288 K at the start of a full
brake roll and 294 K at the end. The whole 7.6 MJ went into the tires and the
fade model could never act.

The pilot notes measure this brake through the run up: it holds 17.6 kN, which
needs 3.7 kN m. At 4200 N m the wheel keeps turning. The pack takes 3.6 MJ and
reaches 542 K. The fade takes the deceleration from 0.36 g to 0.29 g over the
roll.

## Fuel system and systems timings

| Item | Value | Confidence |
| --- | --- | --- |
| Flap travel time, full range | 8 s | low, estimate |
| Flap limit speed, landing setting | 380 km/h EAS | low |
| Flap limit speed, takeoff setting | 530 km/h EAS | low |
| Gear travel time | 15 s | estimate |
| Gear limit speed | 400 km/h EAS | firm |
| Gear drag area, down | 0.45 m2 | estimate |
| Gear drag area, extra in transit | 0.15 m2 | estimate |
| Slat travel time | 0.5 s | estimate |
| Slat hysteresis band | 2 deg | estimate |

**The gear limit speed needed a reading, not a guess.** The handbook says "Do
not lower landing gear above 4000 km/hr (248 mph)". The metric value carries an
obvious extra zero. 248 mph is 400 km/h, so the limit is 400 km/h. Confidence:
firm.

**The gear and the flap timings both come from the same small pump.** The
emergency compressed air system puts the main gear down in 2 to 3 seconds and
the nose wheel down in 5 to 10 seconds. The handbook and Wendel both say the
normal hydraulic system is far slower. The 18 liter per minute pump is too small
and drives the flaps as well. The model uses 15 seconds for the gear and 8
seconds for the flap. Eight seconds for 50 degrees is 6.25 degrees per second.
That pace matches a pilot who watches the graduations on the flap and lets go of
the button at the setting he wants.

The nose wheel comes down very much later than the mains. The same pump feeds
both legs in series, and the nose leg works against the airstream. The mains run
over the first 55 percent of the cycle and the nose runs over the last 65
percent.

**No wartime document in the reference set gives a flap limit speed.** Wendel
writes only:

> "the high speed of the aircraft easily tempts one to lower the undercarriage
> or flaps whilst travelling too fast and this leads to damage".

The IL-2 and War Thunder data sets both work from captured Messerschmitt
material, and both use 380 km/h at the landing setting. At the takeoff setting
they give about 530 km/h. The model takes both, with LOW confidence.

## Limits and placards

**The handbook placards this aircraft AWAY from its structural limits.** It says
"No spins are to be attempted with this airplane", "No acrobatics are to be
performed", and "No high-speed dives should be run". There is therefore no
published g placard to read.

| Limit | Value | Confidence |
| --- | --- | --- |
| Limit load factor, positive | +7 g at 6396 kg | medium |
| Limit load factor, negative | -3 g at 6396 kg | estimate |
| Ultimate factor | 1.8 | medium |
| Ultimate load factor | +12.6 g | derived |
| Airframe limit speed | 950 km/h TRUE | firm |
| Never start an engine above | 4000 m | firm |

The German requirement sorted an airframe into a stress group. The highest
group, H 5, covered the single seat fighter and asked for a limit load factor of
7 g. The group is firm for a fighter of the period, and the placement of THIS
aircraft in that group is the estimate.

The negative limit is 0.43 of the positive one, which is where the fighters of
the period sat. The P-51D carried +8 and -4, and the requirement of the day
asked for 0.4 to 0.5 of the positive case.

**The ultimate factor of 1.8 has an independent check.** The German requirement
used 1.8 where the British and the American requirements used 1.5. The product,
`7 * 1.8 = 12.6 g`, agrees with the IL-2 Great Battles data set. That set works
from captured Messerschmitt material and gives the Me-262 a maximum load factor
of 12.5 g. The two agree to one percent, and 12.5 g is a breaking value and not
a placard.

**A structure carries a LOAD, in newtons, and not a load factor.** The limit
therefore falls as the aircraft gets heavier. At the maximum takeoff mass of
7130 kg the positive limit is 6.28 g. A cap of 1.25 holds the rise as the
aircraft gets lighter. The wing bending case scales with the mass. The fittings,
the control surfaces and the tail do not. One of those takes over as the mass
falls, and the cap puts that takeover at 8.75 g.

The airframe limit speed is a TRUE airspeed. The handbook table of section 3
gives the maximum speed in a 20 to 30 degree dive as 950 km/h true. The Me-262
airspeed indicator is altitude compensated, so above 400 km/h it reads true
airspeed. At 6000 m that placard is Mach 0.834.

**A full airframe overspeed jams the AILERON and leaves the elevator alone.** An
unbalanced aileron on a wing whose skin has started to work is the classic high
speed failure of the period. The pilot can survive that failure, because the
aircraft still turns on the rudder. Confidence: estimate.

## Estimated numbers and the method behind each one

This is the complete list of estimates in the aircraft definition. Every one
carries the method that produced it.

| Number | Value | Confidence | Method |
| --- | --- | --- | --- |
| Wing dihedral and its start station | 3.5 deg from 2.20 m | estimate | from photographs and a three view |
| Root and tip incidence | 1.5 deg and 0 deg | medium | from a three view, and measured to buy little at the stall |
| Slat span | 3.00 to 6.02 m | medium | Stapfer, pages 31 and 36, outer panel only, see `docs/flight-model.md` |
| Fin quarter chord sweep | 30.7 deg | estimate | from a three view |
| Nacelle span station and height | 2.05 m and 0.53 m | estimate | from photographs |
| Nacelle volume | 0.9 m3 | estimate | the envelope less the duct, because a nacelle is not a closed body |
| Nacelle Munk factor | 0.30 | estimate | reduced from the 0.47 of a closed body, for the same through flow reason |
| Fuselage axial drag | 0.13 | estimate | 0.09 for a smooth body of fineness 8, plus the airframe allowance |
| Nacelle axial drag | 0.10 | estimate | 0.06 for a clean duct, plus the airframe allowance |
| Fuselage base area fraction | 0.20 | estimate | a fifth of the maximum section, for a fuselage that ends in a tail cone |
| Flap peak lift increment | 1.1 at 50 deg | estimate | Raymer gives 1.3 and Hoerner 1.0 to 1.3 for a slotted flap, and a plain slot belongs low |
| Flap extra stall angle fall | 1.2 deg at 50 deg | estimate | measured sections stall 3 to 4 deg early, and the shift gives the rest |
| Flap drag constant | 0.85 | estimate | half the 1.7 Hoerner fits to a split flap, checked three ways at 40 deg |
| Slat stall angle shift | 6 deg | estimate | Hoerner, a slat of this size moves the section stall from 13 to 19 deg |
| Slat deploy angle | 8 deg | estimate | checked against the handbook 300 km/h glide and 450 km/h turn |
| Control travel limits | see the table | estimate | from a three view |
| Fuselage section table | 18 stations | estimate | from a three view of the A-1a |
| Weight statement, seven groups | see the table | estimate | fighter class fractions that sum to the firm empty mass |
| Engine station | 6.15 m | low | half the nacelle length, moved aft for the jet pipe and the bullet |
| Wing lump distribution | chord squared times thickness | medium | the structural box area, with the mass center at 46 pct chord |
| Fuselage aft bias | 0.9 | low | the carry through of the fin and tailplane and the rear tank bay |
| Pilot lump | 100 kg at 4.3 m | medium | pilot, seat and parachute |
| Equipment lump | 192 kg at 7.2 m | low on the split | it closes the firm loaded mass |
| Tank stations | see the table | low | the aft edge of the space the fuselage offers, held there by the balance |
| Strut travel | 0.28 m | estimate | a fighter oleo of this size runs 0.2 to 0.3 m |
| Tire rates | 300 and 700 kN/m | estimate | Roskam part IV, a third of the section height at the rated load |
| Load factor limits | +7 and -3 g | medium and estimate | the German H 5 stress group, and the 0.43 ratio of the period |
| Mass scaling cap | 1.25 | estimate | the non wing cases take over as the mass falls |
| Overspeed damage time | 10 s | estimate | full damage at twice the limit dynamic pressure |
| Fire burn through time | 25 s | estimate | longer than the 10 s the engine model needs to ruin its turbine |
| Unusable fuel | 40 kg | estimate | two percent of capacity is the usual allowance |
| Unporting thresholds | 0 g full, 0.5 g near empty | firm mechanism, estimate values | a smaller puddle runs off the pump at a smaller push |
| Flap and gear timings | 8 s and 15 s | low and estimate | one 18 liter per minute pump feeds both |
| Flap limit speeds | 380 and 530 km/h | low | two modern data sets built from captured material |
| Gear drag areas | 0.45 and 0.15 m2 | estimate | Hoerner chapter 13, and a third of the down value at mid travel |

## Sources

- "Pilot's Handbook for Me-262 A-1", Air Materiel Command, Wright Field,
F-SU-1111-ND, 10 January 1946. The handbook. Firm and quotable.
- Fritz Wendel, handling notes. The touch-down speed and the hydraulic pump.
- Kay, "Junkers Aircraft and Engines". Engine data.
- Jumo 004 B-1 data sheet. Thrust, rotor speed, mass flow, pressure ratio.
- Abbott and von Doenhoff, "Theory of Wing Sections". Section data.
- Hoerner, "Fluid Dynamic Drag" and "Fluid Dynamic Lift". Bodies, flaps, slats.
- Munk, "The Aerodynamic Forces on Airship Hulls", NACA Report 184, 1924.
- Leishman and Beddoes, 1989. Dynamic stall.
- NACA TN 664. The flapped section polar.
- NACA TN 3501, Nelson and McDevitt, 1955. Transonic section aerodynamic center.
- USAF DATCOM. The fin effective aspect ratio and the sidewash.
- Perkins and Hage, "Airplane Performance, Stability and Control".
- Roskam, "Airplane Design", parts IV and VI.
- Loftin, "Quest for Performance", NASA SP-468. The leading edge sweep.
- Mason, "Stability and Control Derivative Estimation and Engine-Out Analysis".
- Raymer, "Aircraft Design: A Conceptual Approach". Flap increments.
- IL-2 Great Battles and War Thunder data sets, both built from captured
Messerschmitt material. The ultimate load factor and the flap limit speeds.
- Stapfer, "Messerschmitt Me 262", pages 31 and 36. The slat panels.
- Radinger and Schick, "Me 262", 1996. The Lindner dive program.
- Dorr, "Fighting Hitler's Jets", 2013. The high Mach losses.
- The National Air and Space Museum A-1a airframe. The overall height.
- ESDU 71025. The tire friction peak.

## Known gaps

None of the numbers in this document carries an open issue today.

The three files that once held the old 18.5 degree sweep now hold the corrected
15.72 degrees. `src/render/models/me262.ts` draws the quarter chord line at
15.72 degrees, rooted at station 4.992 m. It also draws the corrected 2.00 m fin
with a 0.25 to 1.85 m rudder. `src/aircraft/me262/mass.ts` puts its wing lumps
on the same line as the aerodynamic strips, which moved the computed center of
gravity by 2.4 mm. The render model bounding box still measures 12.510 by 3.830
by 10.600 m. **The aircraft on the screen is now the aircraft that flies.**

Four numbers in this document carry LOW confidence and would move a real result
if a better source appeared. They are listed here so that nobody has to find
them again.

1. **The four fuel tank stations.** No drawing gives them. The model holds them
at the aft edge of the space the fuselage offers. The aircraft has to balance
where the plan form says it balances. Half a meter forward would give a static
margin near 26 percent.
2. **The engine station of 6.15 m.** It is the single largest lever on the
balance, and it is an estimate from the nacelle length.
3. **The two flap limit speeds.** No wartime document gives one. Both numbers
come from two modern data sets built from captured material.
4. **The equipment lump of 192 kg.** The total is firm, because it closes the
published loaded mass. The split across the oil, the radios, the oxygen and the
gun camera is a guess.
