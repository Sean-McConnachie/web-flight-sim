# Junkers Jumo 004 B-1 engine model

This document describes the model of the turbojet engine. It gives the gas path,
the torque balance on the rotor, the fuel schedule, and the start procedure. It
also gives the failure modes that the pilot must respect. The engine of this
aircraft needs slow and careful throttle work, so this document states the rules
that the model enforces. The reference numbers live in `docs/aircraft-me262.md`.

`src/aircraft/me262/engine.ts` holds the model. `createJumo004(position)` builds
one engine and the aircraft builds two.

## Engine overview

The Jumo 004 B-1 is a single spool axial turbojet. Eight compressor stages, six
straight through combustion chambers, one turbine stage, and a variable area
nozzle with a movable bullet. It makes 8.8 kN of static thrust at sea level at
8700 rpm.

**The model is a gas path and a torque balance, and not a lag on the rotor
speed.** That choice is the whole of this document. Everything a pilot has to
respect about this engine comes out of one equation:

```
J * d(omega)/dt = Q_turbine - Q_compressor - Q_friction + Q_starter
```

A first order lag on the rotor speed would give the right spool time and nothing
else. It would not give a slow start, a fast response at high rotor speed, or a
thin surge margin at low rotor speed. It would also not give a hot start, a
flame out, or a hung start. The torque balance gives all of them from the same
equation.

## State variables and units

| Name | Unit | Meaning |
| --- | --- | --- |
| `state` | - | `off`, `starter`, `lightOff`, `idle`, `running`, `stall`, `flameout`, `fire` |
| `rotorSpeed` | rad/s | the state that the torque balance integrates |
| `rpm` | rpm | the same value, for the gauge |
| `thrust` | N | net thrust along body x |
| `gasTemperature` | K | turbine inlet temperature, lagged by 0.35 s |
| `fuelFlow` | kg/s | delivered flow, lagged by 0.15 s |
| `surgeMargin` | - | 1 is clear and a negative value means the compressor is surging |
| `damage` | - | 0 to 1, permanent |
| `pooledFuel` | kg | unburned fuel lying in the chambers and the jet pipe |

`docs/CONVENTIONS.md` section 2 says that the rotor speed is a rad/s inside the
model and an rpm only on the gauge. The model holds both, because the published
data, the handbook procedures and the pilot notes are all in rpm.

`update` runs once per physics step and sub-steps internally at 5 ms or less.
The answer at 60 Hz and the answer at 240 Hz agree to within 60 rpm.

## The gas path

The stations run inlet, compressor, combustor, turbine, and then the nozzle as a
share of the pressure drop.

**Inlet.** The ram rise is `1 + 0.5 (gamma - 1) M^2`, and the pressure recovery
of the duct is 0.98. The inlet temperature and pressure give the two corrected
flow groups `theta` and `delta`, which drive the rest of the path.

**Compressor.** Three maps against the CORRECTED rotor speed give the pressure
ratio, the corrected mass flow and the isentropic efficiency:

| Corrected speed | Pressure ratio | Flow fraction | Efficiency |
| --- | --- | --- | --- |
| 0.2 | 1.04 | 0.16 | 0.54 |
| 0.4 | 1.17 | 0.34 | 0.63 |
| 0.6 | 1.50 | 0.55 | 0.68 |
| 0.8 | 2.15 | 0.79 | 0.745 |
| 1.0 | 3.14 | 1.00 | 0.78 |

The end points are firm. The pressure ratio at full power is 3.14 and the mass
flow is 21.2 kg/s, both from the Jumo 004 B-1 data sheet. The shape between them
is an estimate. `PR - 1` runs close to the 2.8 power of the corrected speed,
which is the usual shape for an axial compressor.

**Combustor.** The fuel adds `fuelFlow * 42.8 MJ/kg * 0.95` of heat to the gas
flow. The heating value is an estimate: J2 was a coal derived gas oil, and light
diesel runs 42.5 to 43.0 MJ/kg. The turbine inlet temperature comes out near
1015 K at full power, and the published turbine entry temperature is near 1030
K. The blades were hollow Cromadur, a chrome manganese steel with no nickel, so
the limit is low. The model puts the limit at 1100 K, which is 85 K above the
full power value.

**Turbine and nozzle.** The model splits the available log pressure drop between
the turbine and the nozzle, with the share against the corrected speed. At full
power the nozzle is close to critical and takes 44 percent of the drop, which
leaves a turbine pressure ratio of 1.85. The share table is an estimate from the
flow matching of a fixed area nozzle, and it is the weakest table in the file.

**Thrust does NOT come out of the gas path.** It comes from a two dimensional
table against Mach number and altitude, scaled by the corrected rotor speed:

```
thrust = table(mach, altitude) * thrustSpeedFraction(correctedSpeed)
         * (1 - 0.6 * damage) * stallFactor
```

The model builds the table once, from 8800 N at sea level, a Mach factor, and a
lapse on the density ratio. `thrustSpeedFraction` reads the CORRECTED speed and
not the physical speed. Reading the physical speed would count the altitude
twice.

## Rotor spool dynamics

The torque balance, in code:

```ts
const compressorTorque = (airFlow * compressorWork) / divisor;
const turbineTorque = (gasFlow * turbineWork) / divisor;
const friction = FRICTION_TORQUE_CONSTANT + FRICTION_TORQUE_LINEAR * omega;
net = turbineTorque + starterTorque - compressorTorque - friction;
this.rotorSpeed = omega + (net / ROTOR_INERTIA) * h;
```

`divisor` is the rotor speed with a floor, because torque is power over speed
and a stopped rotor would divide by zero.

**One mechanism, three behaviors.** All three come out of the same three lines
above, and none of them has code of its own.

**The slow spool, 8.46 s from idle to 95 percent.** At a low rotor speed the
compressor pressure ratio is near one, so the turbine has almost no pressure
drop to work on. The turbine torque then barely exceeds the compressor torque,
and what is left over has to accelerate 10.5 kg m2 of rotor. The compressor
efficiency is also lowest there, at 0.54 against 0.78 at full speed. More of the
work the rotor puts in comes back as heat and less as pressure. The published
figure is 8 to 10 seconds and the model gives 8.46 seconds.

**The fast response above 6000 rpm.** The same equation, at the other end. The
pressure ratio is high, the turbine has a large drop, and the surplus torque is
large. A slam from 7000 rpm to full reaches 95 percent in under 3 seconds. The
same step of 1200 rpm takes more than 4 seconds down at idle.

**The thin surge margin below 6000 rpm.** The fuel schedule delivers roughly
what the LEVER asks for, and the airflow follows the ROTOR. At a low rotor speed
the airflow is small, so the same lever movement gives a far higher fuel to air
ratio.

| Rotor speed | Airflow | Fuel to air at full lever | The surge line there |
| --- | --- | --- | --- |
| 3000 rpm | about 7 kg/s | 0.065 | 0.0235 |
| 6000 rpm | 13.9 kg/s | 0.021 | 0.0265 |
| 7000 rpm | - | 0.024 | 0.028 |

A slam at idle asks for three times the limit. The same slam at 7000 rpm sits
inside it. That is why the pilot notes place the danger band below 6000 rpm, and
the model reproduces the band rather than enforcing it.

**The steady running line sits at 0.0095 at idle and 0.0165 at full power, and
the fastest safe acceleration sits near 0.022.** The gap between the running
line and the surge line is what the pilot has to spend. At idle there is almost
none of it.

**The rotor inertia is an estimate and this is how it came about.** Two
independent checks, then a fit.

1. Mass and geometry. The engine dry mass is 719 kg. The rotor is the welded
eight stage compressor drum, the shaft and one turbine disc, which comes to
about 175 kg. The compressor tip diameter is 0.64 m, so the radius of gyration
is near 0.245 m. That gives `175 * 0.245^2 = 10.5 kg m2`.
2. Energy and time. The stored energy between idle and full speed is 3.8 MJ.
Over 8.5 seconds that needs a mean surplus power of 450 kW. That is 15 percent
of the 3.04 MW the compressor absorbs at full power. That is a believable
surplus.
3. The fit. The model ran from a settled idle, with the lever advanced as fast
as the surge margin allows. The inertia moved until the rotor reached 95 percent
of maximum speed inside the published 8 to 10 second window.

The value is 10.5 kg m2 and the confidence is low. The two checks agree with the
fit, which is the reason to trust it at all.

The friction pair gives 58 N m at full speed, which is 1.7 percent of the
compressor torque. It is an estimate.

**An unlit rotor does not use the compressor maps.** The maps are steady state
maps of a running engine, and they would report a drag near 2000 N m at full
speed. A real engine coasts down for half a minute. The unlit rotor therefore
uses a windmill drive toward 10 rpm per meter per second of airspeed. It also
uses a motoring drag that lets the rotor coast down from full speed in about 25
seconds in still air. Against the starter that drag balances near 1191 rpm, as
the start section below states. All four constants are estimates with low
confidence.

## Fuel flow schedule

```
fuelFlow = lever(throttle) * enrich * delta * governor
```

Then a first order lag of 0.15 s, which is the fuel valve.

- **`lever`** is a convex table from 0.0613 kg/s at closed to 0.4 kg/s at full.
The first part of the travel adds little fuel, which is what the real unit did.
- **`enrich`** is the start enrichment. It blends in below idle and it holds the
fuel to air ratio near 0.020 through the start.
- **`delta`** is the inlet pressure ratio. This is the barostatic unit of the
B-1, which cut the fuel back with altitude.
- **`governor`** is the rotor speed droop. It cuts the fuel to a quarter over
300 rpm above the maximum speed.

**There is no acceleration limiter.** No part of the schedule clips against the
surge line. The surge line is a DIAGNOSTIC, and crossing it puts the engine into
a compressor stall. The pilot is the limiter. That is exactly the aircraft the
Me-262 was. The flight test that measures the spool time has to advance the
lever on the margin by hand.

The idle flow of 0.0613 kg/s is 221 kg/h, which sits inside the 190 to 250 kg/h
range the handling notes give. It comes from the torque balance at 3000 rpm,
with low confidence, and the model settles at 3046 rpm with it.

## The fuel consumption is EMERGENT

**This is the strongest evidence in the project that the engine model is
physical rather than fitted.**

Nothing in `engine.ts` sets a specific fuel consumption. The fuel valve table
ends at 0.4 kg/s, which is deliberately HIGHER than the engine needs. The rotor
speed governor then has something to cut back. The torque balance then decides
where the engine settles, and nothing else does.

Follow the chain at sea level, static, at full lever:

1. `theta` and `delta` are both 1, so the fuel command is `0.4 * governor`.
2. The rotor settles where the turbine torque equals the compressor torque plus
the friction. That fixes the rotor speed.
3. The flight test measures 8733 rpm and 8797 N at the brake release.
4. At 8733 rpm the governor is 0.89, so the fuel flow is 0.356 kg/s.
5. 8797 N is 897.0 kgf, so the consumption is 1.43 kg per kgf per hour.

The measurement from the run that first built this engine model gives 1.427.

**The published figure is 1.4 kg per kgf per hour.** The model is 2 percent
high, and nobody set a constant anywhere in the file to reach it. The unit test
asserts a band of 0.32 to 0.39 kg/s, which is a consumption band of 1.28 to
1.57. The test does not pin the number either.

A model fitted to the published consumption would prove nothing about the
compressor map, the turbine efficiency, the combustor loss or the rotor inertia.
A model that reaches the published consumption THROUGH those four numbers has
tested all of them at once.

`FUEL_FLOW_AT_MAX_POWER = 0.355` sits in the file as a comment on this result.
No code reads it. Do not mistake it for an input.

## Altitude and Mach corrections

| Constant | Value | Mark |
| --- | --- | --- |
| `THRUST_ALTITUDE_EXPONENT` | 1.0 | tuned, not sourced |
| `THRUST_MACH_FACTOR` | 1.0, 0.94, 0.90, 0.885, 0.89, 0.93 at Mach 0, 0.2, 0.4, 0.6, 0.75, 0.9 | tuned, not sourced |
| `THRUST_ZERO_SPEED_FRACTION` | 0.12 | tuned |
| `THRUST_SPEED_EXPONENT` | 2.6 | tuned |

The thrust falls as the density ratio raised to the altitude exponent. A
momentum analysis at a fixed rotor speed gives thrust proportional to
`delta / sqrt(theta)`, which is the density ratio to the power 1.12 at 6000 m.
The engine also runs at a higher CORRECTED speed in cold air, which gives some
of that back. The model holds the exponent at 1.0. A unit test asserts more than
8500 N at sea level and 4400 to 5100 N at 6000 m. It compares those against 8800
N and 4740 N. The 4740 N carries no source in the reference table, so read it as
a class figure and not as a measurement.

The Mach factor falls first and then recovers. The momentum drag of the intake
grows faster than the gross thrust up to about Mach 0.6. Above that the ram
pressure rise takes over.

`delta` also scales the fuel flow and the mass flow, so altitude enters the gas
path on its own and not only through the thrust table.

## Start sequence

The model follows the Wright Field handbook, F-SU-1111-ND, starting procedures
steps 5 to 9. `startPhase` reports which step the engine is on, and `message`
carries one imperative line for the pilot.

| Phase | Handbook step | What happens |
| --- | --- | --- |
| `cold` | before 5 | nothing turns |
| `crank` | 5 and 6 | the Riedel starter turns the rotor toward 800 rpm |
| `light` | 7 | the pilot opens the fuel cock and presses ignition |
| `accelerate` | 8 | past 1800 rpm the pilot releases the starter handle |
| `complete` | 9 | the engine reaches idle at 3000 rpm |
| `failed` | - | a hot start, a wet tail pipe, or a fire |

**The Riedel RBA starter** is a two cylinder two stroke petrol engine of 10 hp
at 6000 rpm. It sits in the intake bullet and drives through a reduction gear
and a dog clutch. The model gives it CONSTANT POWER of 7457 W, with a torque
limit of 130 N m below about 550 rpm. The clutch lets go at 2000 rpm.

The earlier model was a linear fade to zero at 1200 rpm. It gave 43 N m at 800
rpm, which is 3.6 kW, and the motoring drag at that speed is also 3.6 kW. The
crank then approached 800 rpm along an asymptote and took 15 seconds. The
constant power form gives 89 N m and 7.5 kW at the same speed. The crank then
takes 5 to 10 seconds, which is what the handbook describes.

**The starter alone cannot reach 1800 rpm.** Constant power against the motoring
drag balances near 1191 rpm. The rotor passes 1800 rpm only after the fuel
lights and the turbine starts to help. That is why step 8 comes after step 7.

The model works from this handbook quote. "When jet unit has reached 700 to 800
rpm, press ignition button on right side of throttle and hold." Then: "The speed
will increase to 1800 to 2000 rpm at this time. Release starter handle."

**Light off** needs the fuel cock open, fuel available, at least 500 rpm, and
0.4 s since the cock opened. Fuel that arrives before the rotor is ready POOLS
in the chambers. A pool up to 0.8 kg can gather, and it burns off over 2.5
seconds when the engine finally lights. That is the hot start.

A crank with the cock shut PURGES the pool, at 0.008 kg of fuel per kg of air.
The handbook drill is the reason: "The tail pipe should be wiped clean of any
injected fuel before repeating starting process. Otherwise a fire may start."
The model reports "the tail pipe is clear" when the pool falls below 0.02 kg. A
retry after a clean purge adds no damage at all.

A cold start takes 30 to 45 seconds from the first crank to idle.

## Shutdown

`shutdown()` closes the fuel cock, sets the state to `off`, and zeroes the
thrust and the fuel flow. The rotor coasts down on the motoring drag and the
friction.

The shutdown LATCHES. It blocks the fuel, the starter and any relight. Only
`reset()` clears it. That is what a fuel cock does. It is what the structure
model uses to shut down an engine that has burned through its mount.

## Flame out and the throttle danger band

Four causes of a flame out, and the model carries all four:

1. The fuel cock closes, the tanks run dry, or the fuel unports under negative g.
2. The rotor falls below 1200 rpm while nothing is cranking.
3. The fuel to air ratio falls below 0.004, which is a lean blow out.
4. A compressor stall that lasts more than 2 seconds.

**The compressor stall.** When the fuel to air ratio passes the surge line the
state goes to `stall`. The airflow falls to 0.6, the burn efficiency in the
chamber falls to 0.5, and the turbine efficiency falls to 0.35. The rest of the
fuel burns in the jet pipe and makes the bang, which the model raises every 0.35
second. The thrust falls to a tenth.

**Closing the throttle recovers it.** The margin comes back above 0.1, and the
engine returns to idle or to running. A pilot who leaves the lever where it is
flames out after 2 seconds. The unit tests measure both.

**The relight rules.** A cold relight needs the throttle at or below 0.05 and an
airspeed at or below 250 m/s. It also needs a windmilling rotor above 1100 rpm,
which is 110 m/s. The handbook drill is the source: "Throttle closed ... Advance
throttle slowly to idling position."

A HOT relight is different. Above 6000 rpm the engine relights with the throttle
where it is, and it costs less than 0.01 of damage. That threshold is the danger
band of the pilot notes, used the other way round. Above the band the fuel to
air ratio at a full lever sits inside the surge line. A relight there is safe.
One hot start on the ground costs 36 percent of the engine, and a hot relight in
the air costs a tenth of one percent.

## Overspeed and over temperature

**Overspeed.** The fuel schedule handles it alone. The governor droop cuts the
fuel to a quarter over 300 rpm above the maximum, so the rotor cannot run away.
There is no overspeed failure.

**Over temperature** costs the turbine. The damage rate is exponential in the
excess over the 1100 K limit:

```
rate = DAMAGE_RATE_AT_LIMIT * exp(over / 40 K)
```

`DAMAGE_RATE_AT_LIMIT` is one over five hours, so an engine sitting exactly at
its limit takes five hours to ruin. The published time between overhaul was 10
to 25 hours, so five hours at the limit is the right order. The scale of 40 K
means that 160 K over the limit is 55 times faster. The rate caps at 0.2 per
second, so even a surge at 2000 K needs 5 seconds to ruin the turbine.

The damage law reads the LAGGED gas temperature, with a lag of 0.35 s, so one
short spike does not creep the blades.

Damage is permanent. It costs 60 percent of the thrust and 15 percent of the
turbine efficiency at full damage. Only a reset clears it.

## Engine failure modes

| Mode | Trigger | Effect |
| --- | --- | --- |
| Hot start | gas temperature over the limit within 10 s of the light | damage, usually a flame out |
| Wet tail pipe | fuel pools with no light | a pool fire risk, and the handbook purge |
| Compressor stall | fuel to air above the surge line | bangs, a tenth of the thrust, a flame out after 2 s |
| Flame out | four causes, above | no thrust, a windmilling rotor |
| Fire | gas temperature over 1900 K for 3 s, or a pool fire | damage at 0.1 per second, and the state never leaves it |

A fire is absorbing. Only `shutdown()` or `reset()` ends it. The structure model
of `src/aircraft/aircraft.ts` runs the handbook drill. It shuts the engine down
after 25 seconds of fire, because by then the mount and the wing spar have been
in the flame.

The events the engine raises are one shot flags plus monotonic counts. A flag
holds for the one update that made it. A count only grows, so a reader that runs
slower than the physics can compare the count against its own copy and miss
nothing.

## Gauge outputs

The engine exposes `state`, `rpm`, `rotorSpeed`, `thrust`, `gasTemperature`,
`fuelFlow`, `surgeMargin`, `damage`, `position`, `events`, `startPhase`,
`message` and `pooledFuel`.

The cockpit reads four of them:

| Gauge | Source | Notes |
| --- | --- | --- |
| Tachometer | `rotorSpeed` | the value crosses in rad/s and the dial converts, green band 6000 to 8700 rpm, red at 8700 |
| Gas temperature | `gasTemperature` | limit mark at 1100 K, plus 2.5 s of gauge lag on top of the 0.35 s of the model |
| Fuel pressure | `fuelFlow` | a stand in, scaled to 8.5 of the 10 unit dial at full flow |
| Oil pressure | `rotorSpeed` | a stand in, the square root of the speed fraction |

The head up display reads `rpm`, `gasTemperature`, `state` and `message`. The
message is the pilot facing line and it is empty once the start is complete.

## Estimated numbers and the method behind each one

| Constant | Value | Confidence | Method |
| --- | --- | --- | --- |
| `ROTOR_INERTIA` | 10.5 kg m2 | low | two independent checks and then a fit to the 8 to 10 s window, see above |
| `TURBINE_INLET_TEMPERATURE_LIMIT` | 1100 K | medium | 85 K above the 1015 K the gas path gives at full power, against a published entry temperature near 1030 K |
| `FUEL_HEATING_VALUE` | 42.8 MJ/kg | medium | J2 was a coal derived gas oil, taken as light diesel at 42.5 to 43.0 MJ/kg |
| `STARTER_MAX_TORQUE` | 130 N m | medium | the torque the drive passes where the constant power curve reaches it |
| `INLET_PRESSURE_RECOVERY` | 0.98 | medium | a short straight duct |
| `COMBUSTOR_PRESSURE_LOSS` | 0.03 | medium | the loss follows the square of the corrected flow |
| `COMBUSTION_EFFICIENCY` | 0.95 | medium | six straight through chambers |
| `TURBINE_EFFICIENCY` | 0.85 | medium | one axial stage |
| friction pair | 8 N m and 0.055 N m s | low | the pair gives 58 N m at full speed, 1.7 percent of the compressor torque |
| compressor maps | see the table | shape estimated, end points firm | the end points are the published pressure ratio and mass flow |
| `TURBINE_PRESSURE_SHARE` | see above | low | flow matching of a fixed area nozzle, 44 percent of the drop at full power |
| `SURGE_FUEL_AIR_LIMIT` | 0.0235 to 0.030 | low | the acceleration schedule shape of a single spool turbojet |
| `START_FUEL_SCHEDULE` | see the file | low | it holds the fuel to air ratio near 0.020 through the start |
| `IDLE_DROOP_RPM` | 80 rpm | low | 150 rpm settles at 3143 rpm, 40 rpm is still climbing after 30 s, 80 rpm settles at 3080 rpm in 5 s |
| `POOL_PURGE_PER_KG_AIR` | 0.008 | low | a ceiling of 0.019 from the fuel and air at 400 rpm, a floor from clearing 0.8 kg in 43 s at 1191 rpm |
| `DAMAGE_RATE_AT_LIMIT` | 1 in 5 hours | low | against a published time between overhaul of 10 to 25 hours |
| `WINDMILL_RPM_PER_MS` | 10 | low | the fixed flow coefficient of an unlit turbomachine, 1390 rpm at 139 m/s |
| `MOTORING_DRAG` | 0.36 N m s | low | it gives a 25 s coast down from full speed in still air |
| `IDLE_FUEL_FLOW` | 0.0613 kg/s | low, derived | the torque balance at 3000 rpm, and 221 kg/h sits in the 190 to 250 kg/h band |
| `STALL_THRUST_FACTOR` | 0.1 | low, tuned | no source |
| `DAMAGE_THRUST_LOSS` | 0.6 | low, tuned | no source |

Firm and sourced: 8.8 kN static thrust, 8700 rpm maximum, 21.2 kg/s mass flow,
and 3.14 pressure ratio. Also firm and sourced: 7457 W of starter power, 3000
rpm idle, and the 6000 rpm danger band. The 2000 rpm starter cutout, the 800 rpm
crank target and the 1800 rpm self sustaining point are firm and sourced too.

## Known gaps

**The thrust does not lapse fast enough with altitude.** Tracked as bead ole.
The published pair is 827 km/h at sea level and 870 km/h at 6000 m, a gain of 43
km/h. The model gives 823 km/h and 838 km/h, a gain of 15 km/h. Both rows pass
their 5 percent bands, so the fault is the TREND and not either point.

The cause is the lapse exponent. A jet with an exponent of 1 holds the same
maximum speed at every height, because the speed squared goes as thrust over
density. The published pair asks for an exponent near 0.84 on its own. The model
also multiplies the thrust by `thrustSpeedFraction` of the CORRECTED rotor
speed. That costs 12 percent at sea level at Mach 0.68 and nothing at 6000 m,
where the clamp holds it at 1. That second term already leans the other way and
it is what gives the 15 km/h.

The service ceiling is the constraint on any fix. It passes today at 11137 m and
it falls with any steeper lapse. `docs/validation.md` records why the ceiling
looked like a thrust fault and turned out to be a drag fault.

**Two thrust chains, one engine.** The gas path decides where the rotor settles
and the thrust table decides what that rotor speed is worth. The two agree
today, because the table takes the same 8800 N anchor. A change to the
compressor map would move the settled rotor speed without moving the table.
