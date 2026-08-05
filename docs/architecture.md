# Architecture

This document describes how the modules of the simulator fit together. It names
each layer, states what the layer owns, and states which way the dependencies
point. It records the fixed step rate of the physics loop and the render path
that runs on top of it. It holds no physics equations. Those live in
`docs/flight-model.md`. Read `docs/CONVENTIONS.md` section 4 first, because the
separation rule shapes every choice here.

## Module map

| Directory | What it owns |
| --- | --- |
| `src/math/` | units, table lookup, interpolation |
| `src/core/` | the tunable constants, the fixed step loop, the event emitter, the seeded random source |
| `src/physics/` | rigid body, standard atmosphere, ground contact, landing gear, aerodynamics |
| `src/physics/aero/` | airfoil tables, one lifting strip, one body, the assembly, stall, downwash, compressibility |
| `src/aircraft/` | the Me-262 airframe, its mass model, its engines, its systems, the trim solver |
| `src/world/` | runway, ground scatter, scene assembly |
| `src/render/` | renderer, cameras, models, sky, clouds, particles, post processing |
| `src/ui/` | head up display, cockpit gauges, controls menu, debug overlay, telemetry graph |
| `src/input/` | keyboard, gamepad, the on screen touch pad, the binding table |
| `src/weapons/` | the MK 108 cannon, ballistics, ground targets |

`src/main.ts` builds every part and joins them. It is the only file that knows
about all of them.

## Layer rules and allowed imports

`src/math/` and `src/core/` import nothing from the project. `src/physics/`
imports `src/math/`. `src/aircraft/` imports `src/physics/`, `src/math/` and
`src/core/`. Nothing under `src/physics/` or `src/aircraft/` imports the
renderer, the DOM, or any browser interface.

The rule comes from `docs/CONVENTIONS.md` section 4. The reason is the flight
test harness. `test/flight/` runs the whole flight model in Node, with no GPU
and no browser, and `vite.config.ts` sets the test environment to `node` for
every test file. A physics module that reached for the renderer would stop that
harness from running at all. A test that cannot run cannot prove the model is
correct, so the whole validation record of `docs/validation.md` rests on this
one rule.

The rule has a price and the code pays it in the open.
`src/aircraft/aircraft.ts` carries its own copy of the runway threshold position
and of the three control surface travel limits. It may not import the runway
module or the render model. It therefore repeats all four numbers, each with a
comment naming the file they come from. The `AircraftInput` interface is a hand
kept subset of `ControlInput` of `src/input/bindings.ts` for the same reason.

`src/render/`, `src/ui/` and `src/input/` may use the renderer and the browser.
They hold no physics. Two lines in `src/render/particles.ts` read the aircraft
state and the gear leg positions. The rule allows that direction, because the
arrow points from the renderer toward the physics and never back.

## The three coordinate frames

The frames are the most common source of bugs in a flight simulator.
`docs/CONVENTIONS.md` section 3 is the authority. The short form:

- **Body frame.** Origin at the center of gravity, `+x` forward out of the nose,
`+y` right, `+z` down. All aerodynamics, thrust and gear forces use it. Positive
lift is a NEGATIVE z force.
- **World frame.** North-East-Down, with the origin at the runway threshold at
ground level. The rigid body position, velocity and gravity use it. Altitude
above the ground is `-position.z` and never `position.z`.
- **Render frame.** Three.js uses Y up, with `-z` forward.

One module converts between the world frame and the render frame:
`src/render/frames.ts`. It exports `nedToThree`, `threeToNed`, `nedQuatToThree`
and `threeQuatToNed`. No other file writes the conversion.

The vector map is `three.x = ned.y`, `three.y = -ned.z`, `three.z = -ned.x`. The
sign on the last row matters. Both frames are right handed, so the map must have
a determinant of `+1`. The map `three.z = ned.x` has a determinant of `-1` and
is a mirror. A mirror reverses every rotation, so a right roll would render as a
left roll, and no quaternion can express it.

The attitude conversion is a conjugation and not the same swap applied twice:

```
q_three = R * q_ned * inverse(R)
```

`R` is `(0.5, 0.5, -0.5, 0.5)` in `(x, y, z, w)` order, which is a turn of 120
degrees about the axis `(1, 1, -1) / sqrt(3)`. The header of `frames.ts` derives
it.

## The main loop

`src/core/loop.ts` exports `createLoop(callbacks, options)`. The caller passes
two callbacks. `fixedUpdate(dt, time)` advances the simulation by one fixed
step. `render(alpha, frameDt)` draws one frame.

The time source and the frame scheduler are both injectable through
`LoopOptions`. The defaults are `performance.now` and `requestAnimationFrame`,
and both sit inside function bodies, so Node never evaluates either name. A unit
test drives the loop with a fake clock through the same two hooks.

## Fixed step physics and variable rate render

The physics runs at a fixed rate. `PHYSICS_HZ` is 240 and `PHYSICS_DT` is 1 /
240 second. `src/core/config.ts` holds the rate.

A fixed step is not a preference. The aerodynamic model carries lag states, and
the Runge-Kutta integrator needs four evaluations of one derivative. Both give a
different answer at a different step size. A variable step would make the
aircraft fly differently on a fast machine and on a slow one.

The render runs at whatever rate the browser gives. It receives `alpha`, the
fraction of one physics step that has passed since the last one. `src/main.ts`
copies the position and the orientation at the top of every `fixedUpdate`, then
blends the old pose against the new one with `lerp` and `slerp` at `alpha`. The
blended pose goes through `frames.ts` and onto the render model and the camera
rig. Nothing else interpolates. The control surfaces, the gear compression and
the wheel angles all read the current state.

## Time, step size, and catch up behavior

The loop does not add up frame times. It reads the clock once per frame, works
out `elapsed` since the origin, and takes the whole step count from that one
number:

```
target = floor(elapsed / PHYSICS_DT)
while (steps < target) fixedUpdate(PHYSICS_DT, steps * PHYSICS_DT)
```

The count comes out BEFORE the first step of the frame. A slow `fixedUpdate`
therefore cannot add more work to the same frame, which is the classic way a
fixed step loop falls over.

The accumulator cap is 0.25 second, which is 60 physics steps. Past that point
the loop moves the origin forward and adds the difference to
`stats.droppedTime`. Dropped real time never becomes simulated time. The
simulation clock is `steps * PHYSICS_DT`, so it carries no floating point drift
over a long session.

A stalled, backward or invalid clock reports a frame time of zero rather than a
negative or an infinite one.

## Data flow from input to render

```
input.poll(dt)
  -> ControlInput
     -> armament.fixedUpdate      guns fire and write their recoil wrench
        -> aircraft.fixedUpdate   systems, engines, gear, then stepRK4
           -> AircraftState
              -> render(alpha)    pose blend, gauges, head up display, particles
```

The guns run BEFORE the flight model, so the recoil wrench is already in place
when the wrench source of the aircraft reads it.

`aircraft.fixedUpdate` runs its parts in a fixed order. It reads the pilot
commands for the gear and the flaps, then samples the atmosphere. It updates the
structure, the systems and the control deflections. It updates both engines and
builds the ground wrench. Only then does it step the rigid body.

The frequency contract inside one step matters. The engines, the landing gear,
the airframe contact points and the aircraft systems all update ONCE per step,
before the integration. The aerodynamics runs FOUR times, once for each
Runge-Kutta stage. The separation lag of the aerodynamics advances on the first
stage only. The other three stages pass a step of zero. That leaves the lag
state where it is, and each stage stays a pure function of the state it
received.

## State ownership

| State | Owner |
| --- | --- |
| position, velocity, orientation, angular velocity | `src/physics/rigidbody.ts`, held by the aircraft |
| separation point of each strip | `src/physics/aero/surface.ts`, one per strip |
| downwash lag | `src/physics/aero/downwash.ts` |
| rotor speed, gas temperature, engine mode | `src/aircraft/me262/engine.ts`, one per engine |
| wheel spin, brake temperature, tire burst | `src/physics/gear.ts` |
| fuel mass, flap position, gear position, slat position | `src/aircraft/me262/systems.ts` |
| failed parts and the load history | the structure model in `src/aircraft/aircraft.ts` |
| simulated time and the step count | `src/core/loop.ts` |
| camera pose, gauge needles, particle lifetimes | `src/render/` and `src/ui/` |

Every physics module holds its state in an explicit object that the caller
passes in. No physics module holds hidden module level state, apart from scratch
vectors. The step allocates nothing, because it runs 240 times per second.

The aircraft keeps one copy of the last good state. If the integrated state
comes back with a value that is not finite, the aircraft restores that copy. It
zeroes the velocities, resets every strip, raises a `diverged` event, and stops
stepping. The banner in `src/main.ts` then tells the pilot to press R.

## Coordinate frame conversion at the render edge

The conversion happens in one place, in the `render` callback of `src/main.ts`,
and it uses `src/render/frames.ts`. The physics never sees a render frame value.
The renderer never sees a raw NED value, apart from the camera rig, which takes
the NED pose and converts inside itself.

## Startup and shutdown

`main()` builds the parts in one order. Renderer, world, post processing chain,
aircraft, aircraft render model, force arrows, armament and weapon effects. Then
particles, input, cameras, debug overlay, telemetry, head up display, gauges,
banners, and finally the loop. The cockpit interior and its gauges build lazily,
on the first entry into the cockpit view.

The build is asynchronous, because the renderer asks the browser for a WebGPU
device. If any part throws, `main().catch` writes a message into the fatal
element of the page and names the graphics backend it found.

There is no shutdown path. `src/main.ts` never calls `loop.stop()` and never
calls any `dispose` method, although most render modules provide one. The page
lives until the tab closes. This is a gap and not a design choice. It costs
nothing today and it would cost a leak in a page that built the simulator more
than one time.

## The Node flight test harness

`test/flight/harness.ts` is the second host of the flight model. It never builds
a loop and never touches a browser interface. `FlightTest.step()` calls the
autopilot, then `aircraft.fixedUpdate(input, DT)`, with `DT` taken from
`PHYSICS_DT`, and adds the step to its own clock. The model that flies in the
test is the same model that flies in the browser, at the same step size.

The harness provides:

- `createFlightTest()`, which builds the aircraft and runs the engine start
procedure until both engines idle.
- `placeInAir` and `placeOnRunway`, which set the starting state.
- An autopilot with a gain schedule on dynamic pressure. It flies pitch,
altitude, climb speed, heading and sideslip.
- `flyUntilSteady`, which samples 20 signals every 0.05 second. It fits a least
squares slope to the speed and to the climb rate. It reports the window mean as
soon as every slope and every spread sits inside its criterion.
- `record`, `passed` and `printReport`, which build the measurement table that
`docs/validation.md` carries.

The trim solver is separate, in `src/aircraft/trim.ts`. It builds its own
assembly, its own mass model and its own engine, and it never steps the rigid
body. Several flight tests compare the answer of the solver against the answer
of the flown aircraft. Those rows exist to catch a solver that drifts away from
the model it claims to describe.

## Where to start reading the code

1. `docs/CONVENTIONS.md`, sections 2, 3 and 4. Units, frames, and the separation
rule. Everything else assumes them.
2. `src/core/config.ts`. Every tunable number of the host, in one table.
3. `src/core/loop.ts` with `test/unit/loop.test.ts`. The two rate contract.
4. `src/render/frames.ts`. The only axis swap in the project, with its
derivation in the header.
5. `src/aircraft/aircraft.ts`. The module comment is the force sum contract.
6. `src/physics/rigidbody.ts`, then `src/physics/aero/assembly.ts`, then
`src/physics/aero/surface.ts`. This is the flight model itself.
7. `test/flight/harness.ts` and `src/aircraft/trim.ts`. How the tests prove the
model.
8. The renderer last: `src/render/renderer.ts`, then `src/world/scene.ts`, then
`src/render/postfx.ts`.

Read `docs/CONVENTIONS.md` section 6a before you debug anything on the render
side. It lists the platform faults that cost earlier work several hours each.

## Known gaps

**No shutdown path.** See the startup section above. Nothing calls `dispose`,
and `src/main.ts` adds three window listeners that it never removes.

**Two size paths.** `post.setSize` reads the drawing buffer size of the canvas
and `renderer.setSize` reads the layout size. The two agree today. They would
disagree on a display with a device pixel ratio above one.

**A package cycle between the world and the weapons.** `src/world/scene.ts`
imports the target list from `src/weapons/targets.ts`, and
`src/weapons/targets.ts` imports two airfield boundary tests from
`src/world/scatter.ts`. No single pair of files forms a cycle, so the bundler is
happy, but the two directories now depend on each other.
