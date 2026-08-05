# Flight model

This document describes the component based aerodynamic model of the aircraft.
It lists every force and every moment that the model applies. For each one it
gives the frame, the method, and the source of the numbers. It does not repeat
the reference data, which lives in `docs/aircraft-me262.md`. All values here use
the SI units of `docs/CONVENTIONS.md` section 2, and all body axis signs follow
section 3.

## Model overview

The model builds the total force and moment out of separate parts. It carries no
table of aircraft level coefficients, and it has no `Cl_p` term, no `Cn_beta`
term and no `Cm_q` term written anywhere. Those numbers come OUT of the model
and go into the flight tests as measurements.

The aircraft carries 25 aerodynamic elements:

| Group | Elements |
| --- | --- |
| Left wing | 8 strips, cosine spaced |
| Right wing | 8 strips, the mirror of the left wing |
| Horizontal tail | 4 strips, two on each side |
| Vertical fin | 2 strips |
| Fuselage | 1 body |
| Nacelles | 2 bodies |

A strip is a piece of a lifting surface that is small enough that the flow over
it is close to uniform. Each strip has its own local air velocity, its own angle
of attack, its own Mach number and its own separation state. A body is not a
lifting surface and takes a different model. `src/aircraft/me262/geometry.ts`
builds all 25 from the plan form.

**Why cosine spacing.** The lift of a wing falls to zero at the tip over a short
span, so the load gradient is steepest there. Equal strips would put the last
station in the middle of that fall and read the load too high. The station
boundaries sit at `(b/2) sin(k PI / 16)`, which packs four of the eight strips
into the outer third of the semi span. Glauert used the same spacing for the
lifting line series, for the same reason. The outermost strip is only 0.12 m
wide, and that is the tip itself.

**What comes out of the model rather than going into it.** This list is the
reason the model works this way.

- **Roll damping.** Each strip takes `omega x r` into its local velocity. A
positive roll rate moves the right wing down. That wing meets air from below,
its angle of attack grows, and its lift opposes the roll. No damping coefficient
appears anywhere in the code.
- **Pitch damping and yaw damping.** The same term, on the tail arm and the fin
arm.
- **The dihedral effect of a swept wing.** In a sideslip the windward panel
meets less effective sweep than the leeward panel, so it makes more lift. The
sweep split of each strip produces it.
- **Asymmetric stall and the roll off.** The strips stall one at a time. The
wing has washout and the outer panels carry slats. The root therefore goes
first. The two panels are never in exactly the same state, so the break has a
roll in it.
- **The aileron keeping its bite through the stall.** The slat holds the outer
strips attached after the inner strips have gone. That is the whole reason the
real aircraft carried slats, and the model reproduces it without a rule that
says so.
- **The nose down break.** The center of pressure of a separated section moves
aft. The wing lift collapse takes the downwash with it, so the tail meets a
larger angle.

## Force and moment budget

`src/aircraft/aircraft.ts` sums one wrench in body axes about the center of
gravity. Every stage of the integrator receives a cleared wrench and fills it in
this order:

1. **Aerodynamics.** `assembly.evaluate` runs all 22 strips and all 3 bodies.
2. **Structural failure.** A departed wing panel removes its strips.
3. **Landing gear drag.** A flat plate area at a fixed body point, when the gear
is out and the aircraft is moving.
4. **Thrust.** One force along body x per engine, at the engine position, so the
moment arm of an engine failure comes out on its own.
5. **Gun recoil.** An external wrench that the armament writes.
6. **Ground reactions.** The gear and the airframe contact points, built once
per step and capped.
7. **Gravity.** `mass * G0` on the world z axis, rotated into body axes.

`stepRK4` applies nothing on its own. The caller adds gravity, as
`docs/CONVENTIONS.md` section 6 states.

## Airfoil model

`src/physics/aero/airfoil.ts` builds one continuous curve from -PI to +PI for
each section. A spinning aircraft, a tail in a deep stall and a wing in a tail
slide all reach angles far outside the normal range. A table that stops at 20
degrees is not enough.

Three regions join with `smoothstep`, so the curve and its slope stay
continuous. A step in the force at a join would shake the aircraft apart.

1. **Attached flow.** A Kirchhoff lift law with a trailing edge separation
point. It is linear near zero and it bends over near the stall.
2. **Separated flow.** The curve blends down. The drag rises fast and the center
of pressure moves back from the quarter chord toward mid chord.
3. **Flat plate.** Past about 25 degrees the normal force is `2 sin(alpha)`, so
`cl = 2 sin cos` and `cd = 2 sin^2`.

The section lift curve slope is `2 PI (1 + 0.77 t/c)`, which is thin airfoil
theory with the thickness correction of Abbott and von Doenhoff. That is the
INVISCID slope. Measured sections read about 6.13 per radian, because the
boundary layer thickens near the trailing edge and takes lift away. The
separation function carries that loss, so the model keeps the inviscid slope and
lets the separation point bend the curve over before the stall.

The attached drag polar carries a low drag bucket. Both Me-262 sections put
their maximum thickness at 35 or 40 percent chord and hold laminar flow over the
front.

Both sections are symmetric, so the builder makes the positive half and mirrors
it. The tables hold 721 sample points at a step of 0.5 degrees. A linear table
with step `h` misses a peak by `curvature * h^2 / 8`, which is 0.001 in `cl`
here, against a published spread of about 0.05. The table error is 50 times
smaller than the data it holds.

The two sections:

| Section | Peak lift | Stall angle | Minimum drag | Source |
| --- | --- | --- | --- | --- |
| NACA 0009, the tip | 1.25 | 13 deg | 0.0055 | Abbott and von Doenhoff, appendix IV, firm |
| NACA 0011, the root | 1.35 | 15 deg | 0.0060 | derived from the 0009 and the 0012, see below |

Abbott and von Doenhoff publish the 0009, the 0012 and the 0015, and not the
0011. Interpolation on thickness puts the 0011 at a peak of 1.48 and a minimum
drag of 0.0057 at Reynolds 6e6. The fall to Reynolds 3e6 takes about 0.10 off
the peak and adds about 0.0003 to the drag, which gives 1.38 and 0.0060. The
model uses 1.35 and 0.0060, which sits inside that estimate.

Each wing strip gets its own section, blended between the root and the tip by
span station.

## Lifting surface model

`src/physics/aero/surface.ts` evaluates one strip. The step for one strip:

1. **Local flow.** `v_local = v_body + omega x r - wind_body`. This is where the
whole rate damping of the aircraft comes from.
2. **The sweep split.** Simple sweep theory says that only the velocity
component normal to the quarter chord line drives the section pressures. The
strip takes its angle of attack and its dynamic pressure from that component
alone, and resolves the answer back into body axes.
3. **The Mach correction**, from the free stream Mach number, the sweep of the
strip and the thickness of its section.
4. **The control shift and the flap shift**, which move the zero lift angle.
5. **The induced angle**, which the assembly supplies.
6. **The slat shift**, if the strip has a slat and the slat is open.
7. **The section tables**, then the Mach lift scale and the peak lift loss.
8. **The dynamic stall ratio**, from the lagged separation point.
9. **The forces**, built perpendicular to and along the flow that reaches the
section.
10. **The moment**, the section couple plus the arm of the load.

**The strip area stays the planform area, and that is exact.** The normal
section has chord `c cos(sweep)`, and the strip covers a length
`span / cos(sweep)` of the quarter chord line. The two cosines cancel, so
`q_normal * planform area * cl` is the true strip lift.

**The center of pressure moves, and the model moves the POINT the force acts
at.** It does not add a couple. On a swept strip the two differ by a factor of
`cos^2(sweep)`, which is 0.93 on this wing. Simple sweep theory puts the load
`x_cp` of the NORMAL chord behind the leading edge. The line of the load
therefore runs parallel to the quarter chord line. Two parallel swept lines
stand `(perpendicular distance / cos sweep)` apart, measured streamwise, and
`c_n / cos(sweep)` is the streamwise chord. The arm about the center of gravity
therefore carries the streamwise chord and no cosine at all. The section moment
of the airfoil table IS a true couple and keeps the other treatment.

**What the sweep split captures.** The loss of lift with sweep, because the
normal dynamic pressure falls by `cos^2` while the normal angle grows by
`1 / cos`, so the lift falls by `cos`. The rise of the effective section Mach
number. The dihedral effect of the swept wing in a sideslip.

**What it does not capture.** Spanwise flow inside the boundary layer, so the
model does not make tip stall from sweep on its own. The washout and the slats
hold the tip instead. It does not capture the bending of the isobars near the
root and near the tip. The load near both ends is therefore a little wrong.

The flap needs three LIFT effects and the model carries all three. They are the
zero lift shift, a rise of the peak lift, and an extra fall of the stall angle.
A model with the first alone gives the flapped wing the SAME peak lift at a
lower angle. The flap then makes the aircraft stall sooner and no better. That
is backwards. A fourth field carries the drag, and it is not one of the three.
It adds the profile drag of the flap, as `flapCdDelta * sin^2(deflection)`. The
reason is that a shift along a drag polar can move `cd` DOWN. A pilot must never
gain speed by lowering a large drag device. The drag term reads the RAW
deflection and not the Mach scaled one. The reason is that a flap that lost its
lift power to a shock still hangs in the flow.

## The induced angle

Strip theory with two dimensional section data makes no induced drag and reads
the lift curve slope too high. The fix is one angle. The assembly passes the
induced angle of the parent surface. The strip takes it off its local flow
angle. The strip then builds the lift perpendicular to the flow that is left.
The lean of that lift vector IS the induced drag. The model adds no drag term by
hand.

The elliptical loading approximation gives

```
alpha_i = CL / (PI * e * AR)
```

and `CL` follows the lift, which follows `alpha_i`. That is one equation in one
unknown, and the model has to close it.

**The model closes it in closed form, inside the step, on every call.** It runs
a cheap linear pass over the strips, which reports the group lift as
`L(alpha_i) = L0 - K alpha_i`, and then solves

```
alpha_i = L0 / (q S PI e AR + K)
```

with no iteration.

**Why a value carried over from the previous call would be wrong.** `stepRK4`
calls the wrench source FOUR times per step, at four different stage states,
with four different velocities. A value carried over would come from another
stage, so the derivative that Runge-Kutta needs would be built from a mixture of
states. The model would then depend on the internals of the integrator. The
closed form solve keeps `evaluate` a function of the state that reaches it.

**The second fixed point, and the defect it replaced.** The linear pass needs
the lift curve slope of each strip. That slope carries the SEPARATION POINT,
which is a state variable with a lag. The pass used to read that state out of
the strip, which is the value the PREVIOUS full evaluation left behind. Near the
stall the same state then gave two answers that differed by a factor of four.
The trim solver then had to iterate the whole model to its own fixed point. Only
then did it get a Jacobian that meant anything.

The assembly now hands the linear pass the separation point that its own full
pass will use. It then iterates the pair to their common fixed point. Write `x`
for the separation point of every strip at the moment of the call. The full pass
lags `x` over `dt` and builds the forces from the lagged value, so both parts
must satisfy

```
x_lagged = lag(x, dt, alphaTable(x_lagged))
```

The wrench is therefore a function of the state, the entry separation point and
`dt`, and of nothing else. Two calls with the same state and `dt = 0` return the
same wrench to the last bit.

The cost is measured. In attached flow the separation point is already where the
lag wants it. The loop therefore stops after one full pass. The model costs what
it always did, at 22 microseconds per call. A snap pull from 2 degrees to 20
degrees averages 3.5 passes and peaks at 6. The same pull to 30 degrees averages
4.4 and peaks at 7. The loop stops at 8 passes or at a movement of 1e-9.

Three ways a caller controls the state:

| Call | What happens to the lag states |
| --- | --- |
| `evaluate` with `dt = 0` | they do not move, which is what a Runge-Kutta stage needs |
| `evaluateSteady` | a step of Infinity drives every lag to its steady value |
| `reset` | back to fully attached flow, for a respawn or a recovery |

## Wing, tail, and fin surfaces

The wing groups carry their own aspect ratio, Oswald efficiency and reference
area. The downwash of a wing follows the lift of the WHOLE wing and not the lift
of one strip. The left wing and the right wing form one group. The two tailplane
halves form a second. The fin is a third.

A fin is a strip with 90 degrees of dihedral, so its span axis points up and its
normal axis points right. That turns the same code into a fin with no special
case. Its local angle of attack IS the local sideslip.

The sign of each control follows from the moment it has to make.
`controlEffectiveness` is `d(alphaZeroLift) / d(deflection)`, so a POSITIVE
value removes lift. The elevator is positive on every tail strip, because a nose
up command needs the tail load to go down. The aileron is positive on the right
strips and negative on the left. The rudder is negative, because a right yaw
needs the fin to gain lift. A unit test flies the assembled aircraft and checks
all four against the moment they make.

A control rarely fills a whole strip. The aileron runs from 4.00 m to 5.98 m.
Strip 4 runs from 3.48 m to 4.42 m, so the aileron covers 45 percent of that
strip. The model scales the effectiveness of the strip by the covered fraction.
That is exact for the lift increment, because only the covered part of the strip
changes its camber.

## Downwash and sidewash

`src/physics/aero/downwash.ts` owns the largest single correction to the
longitudinal stability and to the elevator power.

**Where the downwash comes from.** The model drives it from the wing LIFT that
the assembly solved this step, and not from a fixed slope times the angle of
attack:

```
epsilon = WAKE_FACTOR * CL_wing / (PI * AR_wing)
```

`WAKE_FACTOR` is 2.0, which is the lifting line value far behind the wing. For
this wing the far field form gives a slope of 0.419 per radian and the empirical
fit `4 / (AR + 2)` gives 0.434. The two agree to four percent. The empirical fit
is a regression over measured aircraft, so the far field value is the one those
aircraft support.

Two reasons for driving it from the lift. A fixed slope is wrong past the stall,
which is where the downwash has its largest effect. When the wing stalls, its
lift collapses, the downwash collapses with it, and the tail suddenly meets a
much larger angle of attack. Second, the flaps and the wing incidence enter the
lift on their own. `epsilon0` and its change with flap angle therefore come out
of the model instead of going into it as constants.

**The dynamic pressure at the tail.** A strip cannot take a dynamic pressure of
its own. It builds the lift from its local flow and takes one angle from the
caller. The model therefore turns the pressure loss into the ANGLE that leaves
the same lift. That step needs care. The assembly solved the induced angle of
the group BEFORE the downwash arrived. A smaller angle makes less lift, which
makes a smaller induced angle. Adding the raw flow angle would take the lift
down by `K epsilon`, and the true loss is only `K epsilon D / (D + K)`. For this
fin that error is a factor of 2.4. This fin has an effective aspect ratio of
1.97. A fin that short loses more than half of its angle to its own induced
angle.

Below the drag rise the tail efficiency is 0.92 clear of the wake. It falls to
0.80 inside the wake of an attached wing. It falls to 0.60 inside the wake of a
separated wing.

**Both the downwash slope and the tail efficiency answer the shock.** These two
laws are half of the Mach tuck, and the section below explains why.

A wake table scales `WAKE_FACTOR` against the SHOCK Mach number, which is the
same number every compressibility table reads. The table is 1 up to the critical
Mach number, because no shock stands on the wing below it. Above it the shock
separates the flow at the root, that part of the wing stops turning the flow
down, and the wake weakens. The tail then meets a LARGER angle of attack. The
measured slope of this model runs from 0.562 at Mach 0.78 to 0.249 at Mach 0.86.

The clean tail efficiency falls with the wave drag the wing pays:

```
1 - eta = (1 - ETA_TAIL_CLEAN) * sqrt(1 + cd_wave / WAKE_REFERENCE_DRAG)
```

`WAKE_REFERENCE_DRAG` is 0.009, which is the profile drag the clean value
belongs to. The law therefore returns 0.92 exactly below the drag rise, and it
reaches 0.817 at Mach 0.86. A floor of 0.60 stops it running away.

The two laws pull opposite ways, and BOTH are correct. A weaker wake turns the
aircraft nose down. A tail with less dynamic pressure is a smaller tail, so it
takes stability away at the same time. That pair is the trap the pilots met.

**Where the wake sits.** The wake leaves the wing and travels aft along the
local flow, so in body axes it RISES at the angle `alpha - epsilon`. The AREA
WEIGHTED tailplane quarter chord sits 0.64 m above the wing chord plane and 3.32
m behind the area weighted wing quarter chord. Those are strip means and not the
root stations of `docs/aircraft-me262.md`. The wake center line therefore
reaches the tail at `alpha - epsilon = 10.9 degrees`. The model measures the
crossing at 18 degrees, and the thick wake of a broken wing covers the tail from
14 degrees.

**The wake covers this tail, and it does not make a trap.** A deep stall trap
needs a tail that stays blanketed while the aircraft holds a high angle with no
way to lower the nose. That is a T tail problem. This tail sits low, at the base
of the fin. The wake center line climbs PAST it above 18 degrees rather than
sitting on it. The tail loses about a third of its dynamic pressure and not all
of it. It holds a positive angle of attack and a nose down moment at every angle
out to 30 degrees. The published behavior matches: the aircraft buffeted, the
nose dropped, and the elevator went soft. Source: the Wright Field handbook,
section 2, and the Lindner dive reports through Radinger and Schick.

**Sidewash at the fin.** The fin meets `beta (1 - d sigma / d beta)`. The DATCOM
fit gives `eta_v (1 - d sigma / d beta) = 1.01` for a mid wing with a fin of
this relative area. The turn at this fin is therefore FAVORABLE, and it very
nearly cancels the dynamic pressure loss. With `eta_v` at 0.95 the sidewash
slope is -0.065.

**The lag is off by default.** The wake needs `l_tail / V` seconds to reach the
tail, near 0.03 s at 120 m/s. A lagged value carried between the four
Runge-Kutta stages would break the derivative in exactly the way the induced
angle would. The lag is 3 percent of the short period and it changes no steady
result. A caller that runs one evaluation per step can turn it on.

## Fuselage and interference

`src/physics/aero/body.ts` holds three effects. A body carries no bound
circulation worth the name, so a lifting line model of it would be wrong.

1. **Cross flow drag.** The cross flow principle splits the velocity into a part
along the axis and a part across it. The cross part meets the body as a cylinder
in a stream, with the side area as the reference and a coefficient near 1.2. The
cross velocity is `V sin(i)`, so the force already grows with the square of the
sine. A finite body pays less than an infinite cylinder, because the flow
escapes around the ends, and a table on the fineness ratio carries that. This is
where most of the normal force of a real fuselage at a large angle comes from.
2. **Slender body lift.** Munk showed that the normal force per unit length
follows the rate of change of the cross section area. The integral over a body
that closes to a point is zero, so what survives is the base area. The model
takes a base area of a fifth of the maximum section. That value is
representative of a fighter fuselage that ends in a tail cone. That fraction is
an estimate, and the term is small next to the cross flow drag above about six
degrees.
3. **The Munk moment.** The same integral taken as a moment does NOT vanish:

```
M = k rho V^2 volume sin(2 alpha)
```

**It acts NOSE UP at a positive angle of attack. A fuselage is DESTABILIZING in
pitch and it wants to turn broadside to the flow.** The horizontal tail exists
to beat it. A model that drops this term gives an aircraft that is far too
stable. That aircraft would need a tail far too small, and its short period
frequency does not match the real one. The same term acts in yaw with the
sideslip angle, and there the fin has to beat it.

`k` is 0.5 for the ideal slender body, times the apparent mass factor
`(k2 - k1)`, which reaches 0.94 at a fineness ratio of 8. The product is near
0.47 for this fuselage. Source: Munk, NACA Report 184, 1924. Confidence: firm.

The size of it matters. With a volume of 9.3 m3 the fuselage alone gives
`dCm/dalpha` near +0.44 per radian. That moves the aerodynamic center of the
aircraft about 9 percent of the mean chord forward. In yaw it takes -0.067 of
directional stability back, against -0.054 for a P-51D. That is why the measured
`Cn_beta` of this aircraft is 0.043 and not the 0.10 of a single engine fighter.
`docs/validation.md` carries the whole element split.

## Drag build up

The model builds parasite drag from its parts. Each strip pays the published
profile drag of its own section, and each body pays its own axial coefficient.

**A real airframe pays more than the sum of its smooth parts, and the model
books that difference on the two body coefficients.** Measured at the maximum
level speed at sea level, as an equivalent flat plate area `f = CD0 * 21.7`:

| Part | Flat plate area |
| --- | --- |
| Wing | 0.113 m2 |
| Tailplane | 0.021 m2 |
| Fin | 0.015 m2 |
| Fuselage | 0.124 m2 |
| Nacelles | 0.068 m2 |
| Total | 0.341 m2, that is CD0 0.0157 |

Two terms are missing from that sum. Interference drag appears where the wing
meets the fuselage, where the nacelle meets the wing, and where the tail meets
the fuselage. Excrescence drag comes from the skin joints, the rivets, the four
gun troughs and the gear doors. It also comes from the aerial, the control
surface gaps and the air that leaks through the airframe. The standard buildup
adds 5 to 10 percent for the first and 10 to 30 percent for the second. A 1944
production airframe sits at the top of the second band.

Three independent numbers agree on 0.44 m2, which is CD0 0.0203.

1. The wetted area is near 107 m2. A turbulent flat plate at the flight Reynolds
number of 3e7 gives a friction coefficient of 0.0026, and the usual form factors
give 0.354 m2. The two allowances take it to 0.44 m2.
2. At 827 km/h at sea level the engine model makes 14.2 kN, so the aircraft must
pay 0.440 m2 to sit at that speed.
3. The published service ceiling of 11450 m needs CD0 near 0.020 through the
minimum drag.

**Why the two bodies carry the whole allowance.** Most of the allowance really
belongs to the wing and the tail. The drag of a strip comes from the section
tables of Abbott and von Doenhoff. Those tables hold firm published data of a
smooth model. This project does not bend published section data to hold an
aircraft level correction. The two body coefficients are estimates at the
aircraft level, so they are the honest place to book it. Read them as the
parasite drag of the body PLUS the interference and the excrescence drag of the
whole aircraft. The values are 0.13 for the fuselage and 0.10 for each nacelle.

Induced drag is not a term. It is the lean of the lift vector of every strip.

## Ground effect

**The model has none.** No strip changes its induced angle or its section data
near the ground, and no test measures a flare.

The touch-down speed of 175 km/h that the sources repeat IS a ground effect
number. A pilot touches down below the stall speed of the free air wing, in
ground effect. The model reaches 175 km/h in the flare by a different route,
through the trimmed approach. `docs/validation.md` records that row against the
trim and not against the handbook. Add a ground effect model before you trust
any landing distance the model gives, or any flare that starts above about half
a span.

## Compressibility and Mach effects

The Me-262 was the first fighter to meet compressibility every day. Its
documented tuck onset is Mach 0.83 and its documented limit is Mach 0.86.
`src/physics/aero/compressibility.ts` holds five effects.

| Effect | What it does |
| --- | --- |
| `clScale` | Prandtl-Glauert growth below the critical Mach, then the fall of the slope above it |
| `cdAdd` | the wave drag rise |
| `acShift` | the center of pressure moves aft, which is the tuck |
| `controlScale` | the control surfaces lose authority |
| `clMaxScale` | shock induced separation takes the peak lift away |

**Sweep, thickness and one shock Mach number.** Only the velocity component
normal to the quarter chord line drives the section pressures, so every shock
driven effect answers to `M cos(sweep)`. A thick section makes the flow work
harder over its crest, so it meets its shock at a lower Mach number. It then
pays a larger wave drag. Measured critical Mach numbers of symmetric sections
run 0.80 at 6 percent and 0.76 at 9 percent. They run 0.72 at 12 percent and
0.68 at 15 percent. That is a straight line of slope -1.33 against `t/c`. Sweep
and thickness both meet in one shock Mach number.

Every anchor in the tables is a FREE STREAM value at the reference sweep of
15.72 degrees and the reference thickness of 11 percent. That is the number a
flight manual gives. The module converts each anchor into a normal Mach number
when it builds the table. The wave drag then carries a factor of `(t/c)^(5/3)`
from transonic similarity, so a 9 percent section pays 0.72 of what an 11
percent section pays.

**Why the thickness term is not optional.** This aircraft carries three
sections. Without the term the tailplane meets the same shock as the wing root.
Its wave drag then acts 0.83 m ABOVE the center of gravity. That makes a large
NOSE UP moment that hides the tuck.

**Why the Prandtl-Glauert rule needed a ceiling.** The rule is a linear, shock
free result and it holds below the critical Mach number and nowhere else. Left
alone it raises the lift of every surface at a fixed angle of attack by 30
percent between Mach 0.5 and Mach 0.85. The downwash follows the wing lift, so
the tail lost half a degree of angle. Its load turned from a small up load into
a down load. A down load behind the center of gravity is a NOSE UP moment three
times the size of the tuck. The model therefore had NO tuck at all. The tuck was
a real effect that the rule was hiding, not a missing term. `SLOPE_LOSS_SCALE`
holds the slope flat from Mach 0.78 to 0.84, where the measurements put the
peak. It then takes the slope to two thirds of the peak by Mach 0.90.

**The Mach tuck itself.** Below the critical Mach number the load of a thin
section sits at the quarter chord. A shock then stands on the upper surface, and
the whole of the extra suction it holds sits BEHIND it. The load moves aft with
the shock, and the shock walks toward the trailing edge as the Mach number
rises. The table runs the section aerodynamic center from 0.25 chord at Mach
0.78 to 0.425 chord at Mach 0.86. It reaches 0.50 chord only at Mach 1, and it
never goes past that point. A section in fully supersonic flow carries its load
at mid chord.

**The section shift is only one of the two causes, and the model carries both.**
The other is the fall of the downwash at the tail, above.

The shift starts at the CRITICAL Mach number, because that is where the shock
appears. It does not start at the published tuck onset. The published onset is
where the nose down moment has grown past everything that pulls the other way.
At that point the pilot feels the nose go down. The model starts the shift at
0.78 and reaches the documented onset on the whole aircraft. Measured, the onset
is Mach 0.825 against the documented 0.83.

The elevator loses its authority at the same Mach number where the tuck needs it
most. That is the trap the Me-262 pilots met. The control table is the one table
that does NOT belong to the wing. Its anchor is the elevator, which sits on the
tailplane, and the tailplane sweeps 12 degrees and carries a 9 percent section.

### Where the tuck really comes from, measured

The total was right long before the split was. This is the record of how the
model was measured and then corrected, because the method matters more than the
numbers.

Measured between Mach 0.78 and Mach 0.86 at 8000 m, the neutral point moves
0.4257 m, which is 0.2339 of the mean aerodynamic chord. Published transonic
neutral point travel for a swept wing aircraft runs 0.15 to 0.25 chord, so the
total sits inside the band. The flight test measures the tuck onset at 0.825
against the documented 0.83.

**The total was right and the parts were wrong.** Flattening the section shift
table on one group of strips at a time gives the split.

| Source | Before | After |
| --- | --- | --- |
| Shift on the WING strips | 0.4183 m, 97.2 pct | 0.2764 m, 64.9 pct |
| Shift on the TAILPLANE strips | 0.0100 m, 2.3 pct | 0.0140 m, 3.3 pct |
| Everything else | 0.0019 m, 0.4 pct | 0.1353 m, 31.8 pct |
| Total | 0.4302 m, 0.2364 MAC | 0.4257 m, 0.2339 MAC |

The standard two term account of the Mach tuck gives it two causes, and the
second one was missing. Source: Hoerner, "Fluid Dynamic Lift", chapter 15, and
the transonic chapters of Perkins and Hage. The first is the aft shift of the
section. The second is the fall of the downwash at the tail. Before the
correction the downwash slope of this model ROSE from 0.550 to 0.555 through the
same Mach range. The tail dynamic pressure ratio held at 0.920 exactly. Three
effects that should carry a tuck carried none of it.

The measurement that showed the section shift was too large is NACA TN 3501,
Nelson and McDevitt, June 1955. It tested 22 rectangular NACA 63A0XX wings on
the Ames transonic bump. The aspect ratio 6 wing with the 10 percent section is
the closest model in the report to this wing. On it the section aerodynamic
center sits at 0.24 chord to Mach 0.80 and reaches 0.32 chord at Mach 0.85. It
then swings sharply FORWARD near Mach 0.90, and it reaches 0.39 chord only above
Mach 0.95. So the real section travels about 0.07 chord by Mach 0.85, and the
old table asked for 0.25 chord, monotone aft. The comparison is worse than that
ratio of 3.6. TN 3501 tested an UNSWEPT wing, so its Mach number is already a
normal Mach number.

The correction added the two downwash laws of the section above. It then
refitted the section table down against them, from 0.50 to 0.425 at the Mach
limit. The whole aircraft total did not move.

The last row of the table splits further. The fall of the downwash slope gives
+0.1608 m and the fall of the tail dynamic pressure ratio gives -0.0412 m. The
second one is NEGATIVE on purpose, for the reason the downwash section gives.

## Stall and post stall behavior

`src/physics/aero/stall.ts` holds the unsteady part. The static tables hold a
section at a fixed angle in a steady stream, and a real wing never does that.
When the pilot pulls, the flow needs time to separate. The wing carries more
lift than the static table promises and then loses it late. That lag is dynamic
stall, and it is why a fast pull gives a sharp break and a slow pull gives a
soft one.

Two parts:

1. **A steady separation point `f`**, from 1 fully attached to 0.04 fully
separated. This is the Kirchhoff and Helmholtz picture of a section with the
flow attached over the front `f` of the chord.
2. **A first order lag** that carries `f` toward its steady value, in the non
dimensional time of the section. The time constant is `Tf c / (2 V)`, so a short
chord at high speed settles fast and a long chord at low speed settles slowly.
`Tf` is 3.0 semi-chords, which Leishman and Beddoes fitted to oscillating NACA
0012 data.

The step uses the EXACT solution of the first order lag, `1 - exp(-dt / tau)`,
and not an Euler step. The exact form stays stable at any step size, and it
reaches the steady value exactly at a step of Infinity. That last property is
what lets the trim solver ask for a steady answer that carries nothing over from
the call before it.

**The lag enters as a RATIO and not as a replacement.** The Kirchhoff law is
right near the stall and wrong a few degrees past it, where a trailing edge
model starts to climb again. The model reads the static coefficients from the
airfoil table, which hands over to the flat plate law, and applies

```
dynamic = static * g(f_lagged) / g(f_steady)
```

The ratio is 1 in steady flow at every angle, so the static table stays in
charge. It leaves 1 only while the angle moves.

The fit of `a1`, `s1` and `s2` comes from two conditions: the Kirchhoff law must
peak at `alphaStall` with the value `clMax`, and its slope must be zero there.
The break angle comes out near 12.6 degrees for the tip section and 13.4 degrees
for the root section. The root breaks later and holds a wider band, which is
what a thicker nose does.

**The slat.** The model shifts the whole section curve toward a higher angle by
`slatAlphaDelta` and gives the linear part back. The lift below the stall
therefore does not change by one count. The stall angle rises by exactly the
shift, and the peak lift rises by the section lift curve slope times the shift.
An open slat adds 0.02 of section drag. That is the middle of the 0.015 to 0.03
that Hoerner gives for a leading edge device of this size. The slat opens over a
band of 2 degrees. A real pressure operated slat runs out over about one degree.
The band here is wider on purpose, so that the force stays smooth at 240 Hz.

At the clean peak lift the three inboard strips sit at a separation point of
0.33 and the five slatted strips sit at 0.84. That gap is the aileron keeping
its bite through the break.

## Landing gear and ground reactions

`src/physics/gear.ts` holds three oleo pneumatic legs, a slip based tire, a nose
wheel steering channel and an independent brake on each main wheel.

**The strut** is a gas spring in series with a tire spring, with an asymmetric
damper and a hard stop:

```
F_gas(s) = springGas * (1 - GAS_FILL * s / maxTravel) ^ -POLYTROPIC_INDEX
```

That is the polytropic law of the sealed air chamber of a real oleo. The fill is
0.9 and the index is 1.3. The force at the end of the travel is then 19.9 times
the force at full extension. That is the compression ratio a single stage oleo
really works over. A LINEAR spring is wrong here and it feels wrong. A linear
spring that carries the aircraft at rest is far too soft at the end of the
stroke. It therefore bottoms on any firm landing. It also returns all of its
energy, so the aircraft bounces back to nearly the height it fell from.

The damper meters fluid through a large orifice while the strut closes. It
meters through a small orifice while the strut opens. It therefore swallows the
touchdown energy and gives little back. Without the asymmetry the aircraft pogos
down the runway.

**The tire** uses the Pacejka magic formula with the curvature term at zero:

```
mu(x) = D * sin(C * atan(B * x))
```

The curve RISES to a peak and then FALLS. That shape is the whole point. A wheel
at the optimum slip grips harder than a locked wheel. A hard brake that locks a
wheel LOSES braking, and the aircraft skids. The longitudinal curve and the
lateral curve are separate. A friction ellipse then limits the two together. A
wheel already braking at the limit has no grip left to steer with.

A slip curve alone cannot hold a PARKED aircraft. The slip ratio has no meaning
at zero rolling speed, so the curve returns no force there. The model carries a
stick slip length that stands for the twist of the carcass. That length is what
lets the pilot run the engines up against the brakes.

**The brakes.** Each main wheel takes 4200 N m at full command and the nose
wheel has none. The brake makes a torque on the wheel, the wheel slows, the slip
grows, and the tire curve decides how much reaches the ground. Nothing shortcuts
the tire. The pack heats from its own friction power, cools toward the ambient
air, and fades by half between 475 K and 675 K.

**The airframe contact points.** With three wheels and nothing else, the only
part of the aircraft the ground can push on is a tire contact patch. An arrival
that carries the center of gravity below the ground plane would then drive the
gear far past its hard stop. That stop is structure and not a spring. Contact
points on the nose, the tail, both wing tips, both nacelles and the belly stop
the aircraft before the gear reaches that state. They are what makes a belly
landing scrape and slide instead of explode.

The model caps every contact force at 12 times the weight over the sum, and at
half of that at any one point. Past that load the airframe has already failed,
so the only duty left to the model is to stay finite and to shed the energy.

**Frequency.** The gear holds state, so `update` runs ONCE per physics step and
outside the four Runge-Kutta stages. The strut, the tire and the friction are
all pure functions of the state, so the wrench itself is correct in any stage.

## Mass, center of gravity, and inertia

`src/aircraft/me262/mass.ts` owns the mass model. `docs/aircraft-me262.md` gives
the numbers and the method behind each estimate. Two facts belong here, because
the aerodynamics depends on them.

The center of gravity sits 0.133 m BELOW the reference plane, because the
engines hang below it. Every element position in
`src/aircraft/me262/geometry.ts` runs from that point, so the wing chord plane
sits that far above the center of gravity.

The fuselage that the aerodynamics sees and the fuselage that the mass model
weighs are ONE shape. `fuselageShape` integrates the section table, and both the
volume of the Munk moment and the mass of the fuselage come out of the same
integral.

## Numerical integration

`src/physics/rigidbody.ts` holds a classical Runge-Kutta 4 integrator over the
six degree of freedom state. The derivatives:

```
dx/dt     = v
dv/dt     = (1 / m) * R(q) * F_body
dq/dt     = 0.5 * q (x) (0, omega_body)
domega/dt = I^-1 * (M_body - omega x (I * omega))
```

The `omega x (I * omega)` term makes the gyroscopic coupling and the
intermediate axis instability. Never drop it.

The step runs 240 times per second and allocates nothing. Every scratch vector,
quaternion and derivative sits in module scope. One aircraft steps at one time
and the physics runs on one thread, so the shared scratch is safe.

The aircraft keeps one copy of the last good state. If the integrated state
comes back with a value that is not finite, the aircraft restores that copy. It
zeroes the velocities, resets every strip separation state, and raises a
`diverged` event. Nothing recovers from a state that is not finite on its own,
because every later step reads it.

## Trim procedure

`src/aircraft/trim.ts` finds the control positions that hold a steady flight
condition. Three equations must hold at the same time:

```
along the flight path    thrust - drag - the weight component = 0
across the flight path   lift - the weight component = the maneuver
about the pitch axis     the total pitching moment = 0
```

**The solver solves them coupled, because they are coupled.** A sequential
search sets the angle of attack from the lift, then the elevator from the
moment, then the throttle from the drag. It has to run the loop again, because
the elevator changed the lift. The throttle also changed the moment through the
thrust line. That loop converges slowly where the couplings are weak and not at
all where one is strong. Near the stall the lift no longer grows with the angle
of attack, so the first step has no answer. Near the Mach limit the elevator
loses its power, so the second step has no answer. Both are exactly the
conditions the flight tests measure.

The solver therefore builds ONE residual vector of three normalized numbers and
drives it to zero with a Newton step on a numerical Jacobian. A backtracking
line search protects the step where the Jacobian is nearly singular.

Four modes share that core:

| Mode | Given | Free |
| --- | --- | --- |
| `trimLevelFlight` | speed, level path | angle of attack, elevator, throttle |
| `trimForAlpha` | angle of attack, level path | SPEED, elevator, throttle |
| `trimSteadyClimb` | throttle, speed | angle of attack, path angle, elevator |
| `trimMaxLevelSpeed` | full throttle, level path | speed, angle of attack, elevator |

**When no trim exists, the solver says so.** Below the stall speed no angle of
attack makes enough lift. Above the thrust limit no throttle makes enough
thrust. The solver reports `converged: false` and the residual it reached. It
never reports a wrong answer as a right one. The tests that search for the stall
speed and for the maximum level speed use that flag as their test.

A numerical Jacobian is the residual at two states that differ by one part in a
hundred thousand. Anything the model remembers between two calls therefore lands
in the Jacobian. A Jacobian of memory is noise. `evaluateSteady` is what makes
the residual a function. It drives both lag states to their steady value inside
the call.

The engine needs the SETTLED thrust, because a rotor takes about eight seconds
to answer the lever. The solver runs the engine model to its steady rotor speed
over a grid of lever positions and airspeeds, once per altitude. It reads the
grid inside the Newton loop. The grid holds the speed as well as the lever. The
speed is a free value in two of the four modes. A thrust that jumped at a speed
step would stop the Newton step at the size of the jump.

## Known limits of the model

**No ground effect.** See the section above.

**No hinge moment and no stick force.** The model reports a deflection. The one
number in the Me-262 record for the high Mach elevator is a PULL FORCE. It is
about 100 lb at Mach 0.86 merely to hold the dive angle. The model cannot report
it. The flight tests use the deflection that stands for it.

**No stabilizer trim channel.** The real aircraft trimmed with a moving
tailplane, and that surface is what still works at high Mach. The Mach recovery
test stands in for it with a steady elevator command.

**No spanwise boundary layer flow.** The sweep split cannot make tip stall on
its own. The washout and the slats hold the tip instead.

**One sweep angle per strip.** A real wing changes its sweep along the span, and
this one does: the inboard leading edge stayed straight on the early aircraft.

**The model carries no engine gyroscopic moment.** Two rotors of about 8700 rpm
turn the same way. The moment they make in a fast pitch or yaw is real and the
model does not carry it.

## Known gaps

**The section shift of the Mach tuck still runs above the measured section.**
This carries no issue, because no measurement in this project depends on it.

The section table now reaches 0.425 chord at Mach 0.86. NACA TN 3501 measures
0.32 chord at Mach 0.85 on an unswept wing. That is 0.175 chord of travel
against a measured 0.07. The split is now 64.9 percent on the wing section
shift. On a real aircraft the tail carries more than that. The model therefore
still leans on the section more than it should.

Two things make that acceptable today. The whole aircraft total is right and
inside the published band. The tail now carries 31.8 percent of the travel
through two laws that answer the shock. A change to the tail arm, the tail area
or the wake therefore moves the tuck in the right direction now. It also moves
it by roughly the right amount. Before the correction it moved almost nothing.

**The inboard slat panel is missing and the sources disagree.** Tracked as bead
b74. The A-1a carried three unconnected slat sections on each wing, over the
full span. The model carries only the outer panels.

The strip grid cannot represent the inboard panel honestly. A strip takes the
whole 6 degree stall shift or none of it, because `SurfaceDef` has no slat
coverage fraction. The inboard panel runs from 0.55 m to 1.60 m. The first
cosine strip runs from 0.00 m to 1.22 m.

The measured effect is large. Outer panels only gives a landing stall of 182.0
km/h. Moving the outer panel inboard to the nacelle gives 173.1 km/h. Adding the
inboard panel as well gives 160.4 km/h, which is 20 km/h below the handbook
floor of 180 km/h.

**The sources disagree with each other and that has to be settled before
anything else.** Stapfer gives 160 to 170 km/h for the slatted aircraft, which
is where the model already lands with all three panels. The pilot handbook gives
180 to 202 km/h. Work out which configuration each number describes before
calibrating anything against either.

**A body reads the wave drag table of an 11 percent wing at zero sweep.**
Tracked as bead 7el. `evaluateBody` calls the Mach correction with a sweep of
zero and no thickness. Every body then reads the table of the REFERENCE SECTION,
which is the wing root. The anchors carry the cosine of the reference sweep. A
fuselage therefore starts its drag rise at a free stream Mach of 0.751. It pays
the full section value on its frontal area. A slender fuselage of fineness 8
does not do that. Its drag rise starts near Mach 0.9 and it is smaller when it
comes.

Measurement gives the cost. At Mach 0.788 the fuselage and the two nacelles
added 0.0023 to CD. The wing added 0.0002 over the same step. The bodies
therefore made 92 percent of the whole drag rise the aircraft felt there. No
test row depends on it today. The 6000 m level speed settles at Mach 0.736,
below the 0.751 onset. It will matter again to any work that touches the level
speed at height or the dive.

**The tire friction has no speed term and a locked wheel grips too well.**
Tracked as bead fw3. The longitudinal shape constant is 1.45, so the magic
formula gives a locked wheel 82 percent of the peak coefficient at a slip of -1.
With a peak of 0.8 a skidding tire therefore holds 0.656 at any speed. Measured
aircraft tires on dry concrete hold about half of the peak at 95 kt. The value
falls with ground speed on the peak side as well. The brake limits the landing
roll today, so the tire is no longer the lever, but the tire is still wrong.
