# Conventions

Read this file before you write any code in this repository. Every rule here is
binding. The rules exist so that many separate modules fit together without a
later rewrite.

---

## 1. Language of all prose

All prose uses ASD-STE100 Simplified Technical English. This covers documents,
README text, code comments, commit text, and every string the user can see.

- Active voice. "The parser reads the file", not "the file is read by the parser".
- Short words. Use, not `utilize`. Start, not `initiate`. Make sure, not `ensure`.
- One instruction per sentence. 20 words maximum for an instruction. 25 for a
  description.
- No contractions. Write "does not", not `doesn't`.
- No semicolons in prose. Write two sentences. Code keeps its semicolons.
- No marketing adjectives. No `seamless`, `robust`, `powerful`, `cutting-edge`.
- One name for one thing. If the code calls it a `surface`, the document calls it
  a surface, not a panel and not a strip.
- American spelling.
- No emoji anywhere.

---

## 2. Units

All internal values use SI units. No exceptions inside `src/physics`,
`src/aircraft`, or `src/world`.

| Quantity | Unit |
| --- | --- |
| Length | meter |
| Speed | meter per second |
| Mass | kilogram |
| Force | newton |
| Moment | newton meter |
| Angle | radian |
| Angular rate | radian per second |
| Temperature | kelvin |
| Pressure | pascal |
| Density | kilogram per cubic meter |
| Time | second |
| Rotor speed | radian per second inside the model, rpm only on the gauge |

Convert to display units only in `src/ui`. Use `src/math/units.ts` for every
conversion. Never write a magic number such as `3.6` in place of a conversion.

A variable name carries its unit when the unit is not the SI default for that
quantity. Example: `speedKmh`, `altitudeFt`, `rpm`. A plain name such as `speed`
always means meters per second.

---

## 3. Coordinate frames

This is the most common source of bugs in a flight simulator. Read this section
twice.

### 3.1 Body frame, used by all physics

Standard aerospace body axes, with the origin at the center of gravity.

- `+x` points forward, out of the nose.
- `+y` points right, out of the starboard wing.
- `+z` points down, through the floor.

Consequences you must remember:

- Positive lift is a **negative** z force.
- A positive roll rate `p` rolls the right wing down.
- A positive pitch rate `q` raises the nose.
- A positive yaw rate `r` moves the nose right.
- Angle of attack: `alpha = atan2(w, u)`, where `u` is the x component and `w` is
  the z component of the airspeed vector in body axes.
- Sideslip: `beta = asin(v / V)`, where `v` is the y component.

### 3.2 World frame, used by the rigid body and the world

North-East-Down. The origin sits at the runway threshold, at ground level.

- `+x` points north, along the runway centerline.
- `+y` points east.
- `+z` points down.
- Altitude above the ground is `-position.z`. It is never `position.z`.
- Gravity is `+9.80665` on the world z axis.

### 3.3 Render frame

Three.js uses Y up and a right-handed frame with `-z` forward. The renderer needs
a conversion. One module owns it: `src/render/frames.ts`. No other file converts
between frames.

The conversion from NED to the Three.js frame is:

```
three.x =  ned.y      (east  -> right)
three.y = -ned.z      (down  -> up)
three.z = -ned.x      (north -> forward, into the screen)
```

The sign on the last row matters. NED is right-handed, because north cross east
gives down. The Three.js frame is also right-handed. A map between two
right-handed frames must have a determinant of `+1`. The map `three.z = ned.x`
has a determinant of `-1`. That map is a mirror, so it reverses every rotation
direction and a right roll renders as a left roll. No quaternion can express a
mirror, so the attitude conversion would be impossible.

The attitude conversion is a conjugation, not the same swap applied twice:

```
q_three = R * q_ned * inverse(R)
```

`R` is the rotation that carries the NED basis to the render basis. Its basis
images are `N -> (0, 0, -1)`, `E -> (1, 0, 0)`, and `D -> (0, -1, 0)`. The matrix
trace is 0, so `1 + 2*cos(angle) = 0` and the angle is 120 degrees. The axis is
`(1, 1, -1) / sqrt(3)`. This gives `R = (0.5, 0.5, -0.5, 0.5)` in `(x, y, z, w)`
order.

`frames.ts` exports `nedToThree`, `threeToNed`, `nedQuatToThree`, and
`threeQuatToNed`. Use them. Do not write the conversion again anywhere else.

---

## 4. The separation rule

Files under these paths must never import the renderer, the DOM, or any browser
API:

- `src/math/`
- `src/physics/`
- `src/aircraft/`
- `src/world/data/`

These files may import **only** the Three.js core math classes:

```ts
import { Vector3, Quaternion, Matrix3, Matrix4, Euler } from 'three';
```

They must not import `three/webgpu`, `three/tsl`, or `three/addons`. They must not
touch `window`, `document`, `performance`, or `navigator`.

The reason: the flight test harness runs the whole flight model in Node with no
GPU and no browser. If the physics touches the renderer, that harness cannot run.
A test that cannot run cannot prove the model is correct.

Files under `src/render/`, `src/ui/`, and `src/input/` may use the renderer and
the browser. They must not contain physics.

---

## 5. Code style

- TypeScript strict mode. No `any`. Use `unknown` and narrow it.
- `verbatimModuleSyntax` is on. Import types with `import type { X } from '...'`.
- File names use kebab-case. `aero-surface.ts`, not `AeroSurface.ts`.
- Types and interfaces use PascalCase. Functions and variables use camelCase.
- Physical constants use SCREAMING_SNAKE_CASE and carry a comment with the source.
- Prefer pure functions. Hold state in an explicit object that the caller passes in.
- Allocate no objects inside the physics step. Reuse scratch vectors held in module
  scope or in the state object. The step runs 240 times per second.
- Import from `@/...` for anything outside the current directory.
- Export one clear public surface per module. Do not export scratch variables.

### Comments

Write a comment when the reason is not obvious from the code. Do not restate the
code. Every physical constant gets a source. Example:

```ts
// Jumo 004 B-1 maximum static thrust at sea level, 8.8 kN per engine.
// Source: Kay, "Junkers Aircraft and Engines", confidence: firm.
export const MAX_THRUST_PER_ENGINE = 8800; // N
```

---

## 6. Shared contracts

Implement to these signatures. They are fixed. If you need to change one, say so
in your report instead of changing it.

### `src/math/tables.ts`

```ts
export interface Table1D { readonly x: Float64Array; readonly y: Float64Array }
export function table1d(x: number[], y: number[]): Table1D;
export function lookup1d(t: Table1D, x: number): number;          // clamped ends
export function lookupCyclic(t: Table1D, x: number, period: number): number;

export interface Table2D {
  readonly x: Float64Array;   // columns
  readonly y: Float64Array;   // rows
  readonly z: Float64Array;   // row major, index = iy * x.length + ix
}
export function table2d(x: number[], y: number[], z: number[][]): Table2D;
export function lookup2d(t: Table2D, x: number, y: number): number; // bilinear, clamped

export function lerp(a: number, b: number, t: number): number;
export function clamp(v: number, lo: number, hi: number): number;
export function smoothstep(edge0: number, edge1: number, x: number): number;
```

### `src/math/units.ts`

```ts
export const DEG = Math.PI / 180;
export function toDeg(rad: number): number;
export function toRad(deg: number): number;
export function msToKmh(v: number): number;
export function kmhToMs(v: number): number;
export function msToKt(v: number): number;
export function mToFt(v: number): number;
export function radPerSecToRpm(w: number): number;
export function rpmToRadPerSec(rpm: number): number;
export const G0 = 9.80665;  // m/s^2, standard gravity
```

### `src/physics/atmosphere.ts`

```ts
export interface AtmosphereSample {
  altitude: number;         // m
  temperature: number;      // K
  pressure: number;         // Pa
  density: number;          // kg/m^3
  speedOfSound: number;     // m/s
  dynamicViscosity: number; // Pa s
}
export function isa(altitude: number, out?: AtmosphereSample): AtmosphereSample;
export const SEA_LEVEL_DENSITY: number;  // 1.225
```

### `src/physics/rigidbody.ts`

```ts
export interface RigidBodyState {
  position: Vector3;        // world NED, m
  velocity: Vector3;        // world NED, m/s
  orientation: Quaternion;  // rotates a body vector into the world frame
  angularVelocity: Vector3; // body axes, rad/s
}
export interface MassProperties {
  mass: number;             // kg
  inertia: Matrix3;         // body axes, about the CG, kg m^2
  inverseInertia: Matrix3;
}
/** Force and moment in BODY axes. The moment acts about the center of gravity. */
export interface Wrench { force: Vector3; moment: Vector3 }

export type WrenchSource = (state: RigidBodyState, time: number, out: Wrench) => void;

export function createState(): RigidBodyState;
export function createMassProperties(mass: number, inertia: Matrix3): MassProperties;
export function stepRK4(
  state: RigidBodyState, mass: MassProperties, source: WrenchSource,
  time: number, dt: number,
): void;                    // steps the state in place
export function addWrench(target: Wrench, add: Wrench): void;
export function clearWrench(w: Wrench): void;
```

`stepRK4` integrates gravity as part of the caller supplied wrench, or the caller
adds it after. State the choice you made in the module comment. The current
choice: **the caller adds gravity**. `stepRK4` applies nothing on its own.

### `src/physics/aero/airfoil.ts`

```ts
export interface AeroCoefficients { cl: number; cd: number; cm: number }

export interface Airfoil {
  readonly name: string;
  readonly clAlpha: number;        // 2D lift curve slope, per radian
  readonly alphaZeroLift: number;  // rad
  readonly alphaStall: number;     // rad, positive side
  readonly cdMin: number;
  readonly thickness: number;      // t / c
  /** alpha in radians, valid over the full circle. Writes into out. */
  sample(alpha: number, out: AeroCoefficients): AeroCoefficients;
}
```

---

## 6a. Known platform faults

These cost hours to find. Read them before you debug a render problem.

**Three.js 0.185.1, WebGL2, mat4 uniforms.** A `mat4` uniform does not reach the
shader on the WebGL2 backend. `getViewPosition` therefore returns zero there. A
private `Matrix4` copy fails in the same way. Use a form that needs only scalar
uniforms. `perspectiveDepthToViewZ` works, and it handles the reversed depth
buffer on its own.

**Three.js reversed depth buffer and render order.** `RenderList.sort` reverses
the opaque list when the camera uses a reversed depth buffer. This also flips
`renderOrder`. A `renderOrder` of -1 therefore draws LAST, not first. Read
`renderer.reversedDepthBuffer` and flip the sign yourself.

**Three.js reversed depth buffer and the TRANSPARENT list.** The same `reverse()`
runs on `transparent` and on `transparentDoublePass`, not on the opaque list
alone. Three.js sorts the transparent list from far to near, which is the order
that alpha blending needs, and then it turns that order around. Two transparent
objects therefore blend in the WRONG order with this buffer. Give every
transparent object an explicit `renderOrder` and change its sign with
`renderer.reversedDepthBuffer`, the same way the sky does. Additive blending
does not care, because addition does not depend on the order.

**Three.js `SpriteNodeMaterial` and the `Sprite` type.** `Sprite` names its
material `SpriteMaterial` in the TypeScript definitions, and `SpriteNodeMaterial`
reports `isSpriteMaterial` as a boolean where the older class reports the
literal `true`. The two types are not assignable, although the renderer takes
either one. Cast through `unknown` and say why.

**Three.js timestamp queries.** `trackTimestamp` is a constructor parameter of
the backend, so it looks like a choice you must make before the renderer starts.
You can still turn it on later. Three.js asks the adapter for every feature it
supports, so the device already carries `timestamp-query`, and the query pool
builds itself on the first pass after
`renderer.backend.trackTimestamp = true`. Then `resolveTimestampsAsync('render')`
fills `renderer.info.render.timestamp` with the real time on the card, in
milliseconds. This is the only honest way to measure a render cost on a machine
where the compositor holds `requestAnimationFrame` at 1 Hz.

**Chrome on this machine, hybrid graphics.** `/dev/dri/card0` is the NVIDIA and
`/dev/dri/renderD128` is the AMD. Chrome allocates on one device and imports on
the other, then reports `VK_ERROR_OUT_OF_DEVICE_MEMORY`, which is not a memory
fault at all. The README gives the launch command that fixes it.

**Chrome with `--use-angle=vulkan`.** The GPU 2D canvas returns transparent
pixels. A `fillRect` reads back as zero, so any canvas texture comes out blank.
Add `--disable-accelerated-2d-canvas`.

**Headless Chrome.** It downgrades this project to the WebGL2 backend without
saying so. Use headed Chrome for the WebGPU path. Always check `backend` on the
`RendererBundle` before you trust a screenshot.

**Freeing a render target in flight kills the device.** A call to
`postfx.setQuality('off')` used to kill the renderer every time, and a switch
between `low` and `high` killed it now and then. Setting `shadows.enabled` to
false does the same. Hold a retired chain for a few frames before you free it.

---

## 6b. How to measure on this machine

Three agents wasted time on bad measurements. Read this before you profile.

**The frame rate is meaningless here.** The desktop session gives the browser
window no frame callbacks, so Chrome holds `requestAnimationFrame` at 1 Hz.
Pump the loop with a TIMER, not a `MessageChannel`. A message channel overruns
the timestamp query pool and drops whole passes.

**Use GPU timestamp queries.** `trackTimestamp` can be turned on after `init`.
Three.js asks the adapter for every feature, so the device already holds
`timestamp-query`. The pool builds when it is first needed.

**Never compare two runs minutes apart.** The clock of this card follows the
load. The same frame reads 1.9 ms at 190 frames per second and 6.6 ms at 40.
Compare the share of each pass INSIDE one frame.

**`renderer.info.render` goes stale on this backend.** Do not use a draw call
count as evidence without a second source.

**Measure the processor as well as the card.** On this project the card was
never the limit. A timer around `post.render()` found 5.0 ms of processor time
against 1.9 ms of card time, and 107 draws of nothing per frame.

**One claim that turned out to be FALSE, recorded so nobody repeats it.** An
early report said Three.js re-uploads every instance matrix each frame, at
287 kB per frame. It does not, at least at version 0.185.1. `Attributes.update`
gates on the attribute version, and `InstancedMesh` does not set
`DynamicDrawUsage`. A still frame sends 2 uploads and 2.1 kB, and none of it is
instance matrices. Measured by wrapping `backend.updateAttribute`.

---

## 7. Tests

- Vitest. Unit tests go in `test/unit/<module>.test.ts`.
- Flight tests go in `test/flight/`.
- A test states the physical fact it checks, in the test name.
  Good: `'density at 6000 m matches the standard atmosphere'`.
  Bad: `'isa works'`.
- Test any equation that has a known answer. Free fall, constant torque,
  standard atmosphere values, and the flat plate limit all have known answers.
- Run `npm run test:unit` before you report your work as complete.
- Run `npm run typecheck` before you report your work as complete.

---

## 8. Reference data for the Me-262 A-1a

Use these numbers. Every one carries a confidence mark. Do not invent a number
that is missing. If a number is missing, estimate it, mark the estimate, and say
how you estimated it.

| Item | Value | Confidence |
| --- | --- | --- |
| Span | 12.51 m | firm |
| Wing area | 21.7 m2 | firm |
| Aspect ratio | 7.21 | derived |
| Length | 10.60 m | firm |
| Height | 3.83 m | firm, corrected, see note |
| Sweep at leading edge | 18.5 deg | firm, corrected, see note |
| Sweep at quarter chord | 15.72 deg | derived, see note |
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
| Stall speed at 6400 kg, clean | about 199 km/h | derived, see note |
| Touch-down speed | 175 km/h | firm, this is NOT a stall speed |
| Rate of climb at sea level | 20 m/s | firm |
| Service ceiling | 11450 m | firm |
| Mach tuck onset | 0.83 | firm |
| Mach limit | 0.86 | firm |
| Takeoff run at 7130 kg | about 1100 m | firm |
| Inertia tensor | see src/aircraft/me262/mass.ts | estimated, validated against the P-51D |
| Load factor limit | +7 g and -3 g | estimated |
| Armament | 4 x MK 108, 30 mm | firm |

### Note on the Mach limit and the recovery

Earlier text in this project said the documented recovery was to reduce thrust
and use the trim wheel. That is NOT documented procedure, and the primary
source says something close to the opposite.

The Wright Field handbook F-SU-1111-ND is dated 10 January 1946. It states that
the pilot should not use the stabilizer trim control when entering or leaving a
dive. The reason it gives is that the control is easy to over-control.

No Me-262 pilot document mentions Mach number at all. The placard is 950 km/h
true airspeed, which is Mach 0.834 at 6000 m. The same placard says that no
high-speed dives should be run.

The thrust and stabilizer technique is supported by the aerodynamics, and by
Mutke's single first-person account. It is not in the handbook. Write it that
way. The aircraft was placarded away from the problem, not given a procedure
for it.

### Note on the height, and a warning about this table

This table first gave the height as 3.50 m and marked it firm. That was WRONG.
The National Air and Space Museum gives 12 ft 7 in, which is 3.84 m, for the
A-1a airframe it holds. Three other sources give 3.8 m to 3.84 m.

The error did real damage before anyone found it. The render model back-solved
the fin span from the height, so the fin came out at 1.67 m instead of 2.00 m.
The small fin then gave the aircraft almost no directional stability. One agent
even adjusted its fin estimate to match the 3.50 m figure, which is the correct
method used on a bad input.

Two unrelated facts caught it. First, the fin span from a three-view disagreed.
Second, full rudder could not hold one dead engine below 373 km/h. The pilot
notes warn against single-engine flight below 300 km/h. With the correct fin
the aircraft holds down to 305 km/h.

The lesson for this table: a confidence mark is only as good as its source. If
a number here fights a measurement, check the number before you change the
model.

### Note on the sweep, and the third wrong number in this table

This table gave 18.5 degrees as the sweep at the QUARTER CHORD and marked it
firm. That was WRONG. The 18.5 degrees is the sweep of the LEADING EDGE.

The sources agree on the leading edge. Wikipedia states a "shallow leading edge
sweep of 18.5 degrees" and cites Loftin, "Quest for Performance", NASA SP-468.
The design history of the same article gives the same angle. It says that
Messerschmitt swept the outer wing to 18.5 degrees on 1 March 1940. The reason
was the heavier engine, which moved the center of gravity aft. That paragraph
then contrasts the panel with the inboard LEADING EDGE, which stayed straight
until the sixth prototype. No source gives 18.5 degrees at the quarter chord.

The plan form then fixes the quarter chord angle. The chord falls from 2.40 m at
the root to 1.07 m at the tip over a 6.255 m semi span, so

    tan(sweep at c/4) = tan(18.5 deg) - 0.25 * (2.40 - 1.07) / 6.255
                      = 0.33460 - 0.05316 = 0.28144,  that is 15.72 degrees.

The two angles differ by 2.8 degrees. Reading 18.5 degrees as a quarter chord
angle would put the leading edge at 21.2 degrees, which no source gives.

Here is what the error cost. The model put the center of gravity at 25 percent
of the mean aerodynamic chord. It found that point with the wrong angle, so the
wing and the balance disagree.

Hold the wing root quarter chord at station 4.85 m. The correct sweep then puts
25 percent of the mean chord at station 5.618 m. That point sits 0.145 m AHEAD
of the center of gravity the model carries. Measured at 6000 m, the static
margin falls from +3.4 percent of the mean chord to -4.2 percent. The aircraft
would be statically UNSTABLE.

Only two numbers can take up the 0.145 m. The model moves the wing root station
from 4.85 m to 4.992 m and holds the center of gravity. The root station was an
estimate read off a three view. Three other places fix the center of gravity.
They are the mass model, the render model, and a unit test.

The static margin at sea level then reads 4.80 percent of the mean chord. It
read 5.13 percent before the correction.

The lesson repeats the one below. Three numbers marked firm in this table have
now been wrong. They are the height, the stall speed, and the reference line of
the sweep. A number is only as good as the source it names. A firm mark on an
angle must also say WHERE the angle sits.

### Note on the stall speed

The widely repeated figure of 175 km/h is NOT a stall speed. It is the
touch-down speed that Wendel recorded. A pilot touches down below the stall
speed of the clean wing, in ground effect, so the two numbers are different
measurements of different things.

The "Pilot's Handbook for Me-262 A-1" gives the real numbers. With full fuel,
the landing gear down, and the flaps down, the aircraft stalls at 202 km/h. The
same handbook states that the aircraft stalls between 180 and 202 km/h.

At 6400 kg the wing loading is 2894 N/m2. The model gives:

| Configuration | Maximum lift coefficient | Stall speed |
| --- | --- | --- |
| Clean | 1.583 at 20.3 deg | 198 km/h |
| Takeoff flap, 20 deg | 1.673 at 20.0 deg | 191 km/h |
| Landing flap, 50 deg | 1.797 at 19.5 deg | 182 km/h |

Bead b65 moved this table. The sweep correction leaves every strip a higher
normal dynamic pressure. The wing then carries 2.8 percent more peak lift, and
the landing stall speed fell to 179 km/h. That is below the handbook floor.

The model answers with the flap. The peak lift the flap adds to its own section
comes down from 1.2 to 1.1. Hoerner gives 1.0 to 1.3 for a slotted flap, and a
plain slot with no Fowler travel belongs at the low end. The landing row then
returns to the 182 km/h it held before the correction.

The landing number sits inside the handbook band of 180 to 202 km/h. Bead b33
must test against the handbook band, not against 175 km/h. A model that stalls
at 175 km/h with the flaps down is wrong, not accurate.

The engine nacelles cut the flap to 29 percent of the span. This aircraft
therefore gains less from its flaps than a single-engine fighter gains.

---

## 9. How to report your work

When you finish, report:

1. The files you created or changed.
2. The public exports you added, with their signatures.
3. The test command you ran and its result.
4. Any contract in section 6 that you had to change, and why.
5. Any number you estimated, and how.

Do not report work as complete if the type check fails or a test fails.
