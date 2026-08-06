# Controls

This document describes how the simulator turns pilot input into a control
surface deflection and an engine command. It lists every input device, every
axis and every key binding. It states the shape of each curve that maps a raw
axis to a command. The physics of the control surfaces belongs in
`docs/flight-model.md` and not here.

`src/input/bindings.ts` holds the whole map. It is the only file that joins a
piece of hardware to an action.

## Input devices

Three devices, and the simulator accepts all of them at the same time.

- **A gamepad with the standard mapping.** The text below names the buttons the
way an Xbox controller labels them. `src/input/gamepad.ts` also handles the
older layout that Firefox on Linux reports. In that layout the triggers arrive
as axes 2 and 5 rather than as buttons 6 and 7.
- **The keyboard.** `src/input/keyboard.ts` reads `KeyboardEvent.code`, which is
the physical key. The map therefore works on a keyboard of any layout.
- **The on screen pad.** `src/input/touch.ts` draws a virtual stick, a rudder
bar, a throttle rocker and a block of buttons. It is a device like the other
two, and the binding table maps it through the `touch` field of a `Binding`.

The input system reports which device the pilot last used, as `activeDevice`. A
gamepad axis must pass 0.25, or a bound gamepad button must go down, before the
pad counts as active. A touch axis takes the same 0.25 test. Any bound key
counts at once. If more than one device moves in the same poll, the keyboard
wins, then the touch pad, then the gamepad. A hand on the keys is the clearest
statement of intent, because a key reports no noise.

The keyboard reader drops the automatic repeat of the operating system, so a
held key raises one press edge and not twenty. It also releases every held key
when the window loses focus. Without that rule a pilot who changed windows in a
turn would come back to full aileron.

## Axis map

The gamepad map:

| Control | Action | Sense |
| --- | --- | --- |
| Left stick, sideways | roll | right is positive |
| Left stick, fore and aft | pitch | back is positive, which raises the nose |
| Right stick | look | it moves the head, not the aircraft |
| Right trigger | yaw and the right brake | |
| Left trigger | yaw the other way, and the left brake | |
| D-pad up and down | throttle, as a rate | |
| D-pad left and right | flaps up and flaps down | |
| A | landing gear | |
| B | both brakes | |
| Y | change the view | |
| Left bumper | engine start, held | |
| Right bumper | fire the cannon, held | |
| Start | menu | |

The X button, the back button and both stick buttons carry no action. They stay
free for later work.

The rudder is the DIFFERENCE of the two triggers. The right trigger yaws right
and the left trigger yaws left, so a pilot who pulls both gets no rudder and two
brakes. That is the same thing a pair of real rudder pedals with toe brakes
gives.

## Key bindings

| Key | Action |
| --- | --- |
| A and D | roll left and roll right |
| W and S | nose down and nose up |
| Q and E | rudder left and rudder right |
| Page Up and Page Down | throttle up and throttle down |
| Shift and equal, Shift and minus | throttle up and throttle down |
| Z | left brake |
| C | right brake |
| B | both brakes |
| G | landing gear |
| F | flaps one step down |
| Shift and F | flaps one step up |
| V | change the view |
| Home | engine start, held |
| Space | fire the cannon, held |
| H, or Escape | the controls menu |
| U | hide or show every overlay panel |
| M | mute the sound, or bring it back |
| R | respawn on the runway |
| F2 | the developer free camera |
| F3 | the debug level |
| Left and right square bracket | trim down and trim up |

Any key that a shifted binding claims goes dead while a Shift key is down. That
is why `F` lowers the flaps and `Shift F` raises them, with no risk of both.

`src/input/bindings.ts` holds every key in the table above. R and F2 used to sit
on a separate key listener inside `src/main.ts`, and the controls menu could
therefore not find them. They are rows of the binding table now, as the
`respawn` and `toggleFreeCamera` actions.

## The on screen pad

`src/input/touch.ts` draws the pad. It starts on the screen when the browser
reports `pointer: coarse` and `hover: none`, which a phone and a tablet both
report and a desktop does not. Two other ways turn it on: the TOUCH PAD button
of the controls menu, and the query string `?touch=1` on the address. `?touch=0`
turns it off.

| Control | Where it stands | Action |
| --- | --- | --- |
| Stick | bottom left | roll and pitch |
| Rudder bar | over the stick | rudder |
| THR + and THR - | bottom right, tall | throttle, as a rate |
| ENG | button block | engine start, held |
| FLAP UP and FLAP DN | button block | flaps, one step each |
| GEAR | button block | landing gear |
| BRAKE | button block | both wheel brakes |
| VIEW | button block | change the view |
| FIRE | button block | fire the cannon, held |
| RESET | button block | respawn on the runway |
| PANELS | top bar | hide or show every overlay panel |
| HIDE PAD | top bar | collapse the pad to this one button |

The pad has no control for the menu. The CONTROLS button of the next section
stands in the same top row, and it stands on every device. A control here would
put one action in two places on a phone and in one place on a desktop.

**A finger down the screen pulls the stick back.** The stick reports a positive
Y when the thumb moves toward the bottom of the screen. The gamepad left stick
reports the same sign when the pilot pulls it back. The two devices therefore
share one row of the binding table.

The stick and the rudder bar both have a FIXED center and both return to zero
when the finger lifts. A floating center would move the neutral point of the
aircraft under the pilot.

Each control captures the pointer that starts on it. A thumb that slides off the
edge of the stick still holds the stick, the way a real hand does. Every control
sets `touch-action: none`, so the browser cannot read a drag as a scroll or two
fingers as a zoom. The page itself sets `user-scalable=no` for the same reason.

The pad carries no look control, no debug level and no free camera. A phone
screen holds the controls that fly the aircraft and no more.

The pad reports nothing while it is hidden or collapsed. A hidden pad also takes
no pointer event, so the picture under it keeps every gesture. A collapse or a
window that loses the focus releases every control, the same way the keyboard
reader releases every key on blur.

## The controls menu

`src/ui/controls-menu.ts` builds the panel. Four controls open it: the CONTROLS
button, the H key, the Escape key and the gamepad Start button.

**The CONTROLS button stands in the top left corner of every view.** A pilot who
has just loaded the page has no way to know that a menu exists. A key alone
cannot teach them, because a key nobody can see is a key nobody presses. The
button leaves the screen while the panel is open, and the panel switch does not
hide it. Help a pilot cannot reach is worse than a small button in a corner.

**H is the key, and F1 is not.** F1 held the menu for one day and F1 was the
wrong key. A browser answers F1 with its OWN help window, and this page cannot
stop it: the reader in `src/input/keyboard.ts` reads the event and never cancels
it. A pilot who pressed F1 therefore got two help windows, one over the other. H
is a letter key, so it needs no modifier and no function row.

The panel switch moved to U when the menu took H. See the section below.

The panel prints one row for each action, with the controls of all three devices
side by side. It builds every row from `DEFAULT_BINDINGS`, so a change to a
binding changes the printed list on the next start. A list written by hand goes
stale the first time somebody moves a key.

A unit test holds every keyboard code against every other one. Two actions on
one key is an error. The one pair that shares a key on purpose is named in that
test. B applies both wheel brakes, so it stands on `brakeLeft` and on
`brakeRight`.

Two things the binding table cannot supply live in that file. `ACTION_INFO`
gives the words for each action. Its type is a complete record over
`ControlInput`, so the compiler refuses the file when an action has no words.
`EXTRA_NOTES` holds the mouse, because a mouse reports movement and a binding
needs a position.

A control pair prints in the order of the words in front of it. The label
`Rudder (left, right)` therefore prints the left trigger first. The rule is the
sign of the scale. A binding with a negative scale is the first direction, and a
key pair already arrives in the order `[negative, positive]`.

**The menu does not stop the simulator.** The aircraft keeps flying while the
panel is open, so a pilot who opens it in a turn comes back to the turn.

## Hiding the panels

U on the keyboard, and the PANELS button of the pad, raise the `toggleHud`
action. It hides the head up display, the debug overlay and the telemetry chart
together, and it shows all of them again. A phone screen is small and the panels
cover the aircraft, so the pilot needs one control that clears the picture.

U stands for the USER INTERFACE it hides. The action held H first, and the menu
then took that key. A pilot looks for help far more often than a pilot clears
the glass. U carries no other action and it needs no modifier.

The CONTROLS button and the SOUND button both stay on the screen through this
action. Every other panel goes.

It is a separate switch from the debug level of F3. The debug level chooses
WHICH instruments run. This switch chooses whether any of them draws, so the
pilot comes back to the same instruments that were on before.

The debug overlay and the telemetry chart start OFF on a touch screen. They are
a development instrument, they take the whole left edge and the whole bottom
right corner, and a phone has neither to spare.

The head up display has its own rule for a narrow or a short window. The systems
bar leaves the bottom edge, which belongs to the pad, and stands at the top left
in two columns. It only moves while the debug overlay and the chart are both
off, because those two own the corners it moves into. `Hud.debugPanelsVisible`
carries that answer from `src/main.ts`.

## The sound

M raises the `toggleSound` action, which mutes the sound and brings it back. It
is the key every media player in the world mutes on, so it is the first key a
person tries.

The SOUND button stands beside the CONTROLS button in every view. It exists for
three reasons.

A phone has no M key while the simulator runs.

A browser holds every `AudioContext` shut until the person acts, and a page that
starts in silence looks broken rather than muted. The button says which of the
two it is, and a click on it is exactly the act the browser waits for.

A person who lands on a page that makes a noise looks for the way to stop it
before anything else. That control must be visible.

The button reads `ENABLE SOUND` while the browser holds the context shut,
`SOUND ON` while it runs, and `SOUND OFF` after a mute. The controls menu
carries the master volume on a slider. Both settings survive a reload.

The mute is on a key and on a button because a pilot who wants silence wants it
at once. The volume is in a panel because it is set one time and then left
alone.

`docs/audio.md` says what every sound is made of.

## Response curves and dead zones

A gamepad stick pair takes a RADIAL dead zone. The reader takes the length of
the pair, removes the dead zone from that length, and rescales what is left. A
per axis dead zone would leave a square hole around the center. A stick pushed
to a corner would then answer differently from a stick pushed straight. The dead
zone is 0.06 of full travel.

Both sticks and both triggers then take the same shaping curve:

```
output = sign(x) * (expo * |x|^3 + (1 - expo) * |x|)
```

`expo` is 0.4. The curve passes through the origin, it reaches exactly 1 at full
travel for any value of `expo`, and it is smooth at the center. It gives fine
control near neutral and full authority at the stop.

A trigger takes a plain dead zone on its 0 to 1 range and then the same curve.

**A key gives full deflection at once.** There is no ramp and no self centering
on the roll, pitch and yaw keys. A press adds the whole 1 and a release removes
it in the same poll. This is a real difference between the devices and a pilot
on the keyboard should know it.

The on screen pad takes NO dead zone and NO expo curve. It reports the offset of
the finger from the center of the control over the travel radius, clamped to the
range -1 to 1. A finger holds a position on glass, so there is no spring to
center and no noise to remove.

The throttle is the one input with memory. Every throttle binding is a RATE and
not a position. The binding table clamps the summed rate to plus or minus 1. The
lever then moves at 0.5 per second, so a full sweep from closed to open takes 2
seconds. The lever holds its value between polls and clamps to the 0 to 1 range.

## Control surface travel limits

`src/aircraft/aircraft.ts` turns a command of plus or minus 1 into a deflection:

| Surface | Full travel |
| --- | --- |
| Aileron | 0.35 rad, that is 20.1 degrees |
| Elevator | 0.44 rad, that is 25.2 degrees |
| Rudder | 0.44 rad, that is 25.2 degrees |

The flap and the slat are not pilot commands in this sense. The systems model of
`src/aircraft/me262/systems.ts` drives both, and it takes time to move them.

## The control authority law

The Me-262 has no powered controls. The stick force of an unboosted control
grows with the dynamic pressure. A pilot at 800 km/h therefore cannot pull the
stick as far back as the same pilot at 300 km/h. A key press or a stick pushed
to its stop carries no force at all, so the simulator has to supply that limit
itself.

`controlAuthority` is the law:

```
authority = 1                       at or below 10 kPa
authority = 10 kPa / q              above it, with a floor of 0.15
```

10 kPa is about 456 km/h of equivalent airspeed. The floor of 0.15 arrives at 67
kPa, which is about 1180 km/h. The aircraft cannot reach that speed, so the
floor never acts in flight. It exists so that a fault in the caller cannot take
the controls away.

The law scales exactly three fields, after the clamp: `roll`, `pitch` and `yaw`.
It does not scale the look axes, the throttle, either brake, or any boolean.

**THIS IS AN INPUT LIMIT AND NOT A FLIGHT MODEL LIMIT.** It says how far the
pilot can move the stick. It says nothing about what the surface does when it
gets there. The trim solver of `src/aircraft/trim.ts` and every flight test in
`test/flight/` command the surfaces directly, and none of them imports the input
layer. A deflection that the flight model receives from a test is the deflection
the model applies.

Two measurements say what the law fixed and what it did not fix. A snatch to
full stick at 3000 m and Mach 0.75 reached 12.33 g after a quarter of a second
before the law. With the law it reached 8.87 g. The peak over a three second
pull was 13.74 g before and 13.02 g with it. So the law removes the STEP that a
key press used to make. It does not cap a pilot who holds the stick back. See
the known gaps below.

## Trim

The two square bracket keys produce a `trimUp` and a `trimDown` command, and the
reader treats both as held rather than edge triggered.

**Nothing consumes them.** The aircraft has no stabilizer trim channel, and
`AircraftInput` carries no trim field. The two keys do nothing today. Do not
confuse them with `src/aircraft/trim.ts`, which is an offline solver that finds
a trimmed flight condition for the tests. That solver has no pilot interface.

This is a gap, and the list below holds it.

## Flaps, gear, and brakes

The gear command is a toggle on an edge. The systems model then runs the gear
through its travel time.

The flap steps one detent per edge through `up`, `takeoff` and `landing`. The
index clamps at both ends, so a pilot who presses F four times ends at `landing`
and not back at `up`.

**The differential brake rule.** Two paths reach the brakes and the larger of
the two wins.

1. **The trigger path takes a gate.** The left trigger drives the left brake,
and the right trigger drives the right brake, with the full analog range. Both
work only while the aircraft is ON THE GROUND and the throttle is at or below
0.05.
2. **The button path takes no gate.** B and the gamepad B button apply both
brakes at full. Z applies the left brake and C applies the right brake. These
work at any speed and at any throttle setting.

The reason for the gate on the triggers is the same trigger pair driving the
rudder. At idle on the ground the rudder has almost no air over it, so a trigger
that also brakes costs nothing. The moment the throttle passes idle, the brakes
close and the triggers steer with rudder alone. A brake that stayed live in the
takeoff roll would drag one wheel at full power.

The Z and the C keys carry no gate, because a key that does nothing else cannot
fight the rudder. A pilot on the keyboard therefore keeps differential braking
through the whole landing roll.

The yaw command also steers the nose wheel, to a maximum of 30 degrees.

Downstream, `src/physics/gear.ts` takes the RAW brake command. Each main wheel
carries 4200 N m of brake torque at full command, and the nose wheel has no
brake. The brake pack heats up, and it fades by half between 475 K and 675 K.
See `docs/flight-model.md` for the tire and the brake.

## Throttle handling rules

The input layer enforces four rules.

1. Every throttle binding is a rate. There is no absolute throttle axis.
2. The summed rate clamps to plus or minus 1, so two throttle sources held at
once do not double the sweep speed.
3. The lever moves at 0.5 per second and clamps to the 0 to 1 range.
4. The control authority law does not touch the throttle.

The sweep time of 2 seconds shapes the DEVICE and not the engine. The Jumo 004
takes 8 to 10 seconds to spool from idle to full power. It surges if the pilot
opens the lever too fast below 6000 rpm. Both of those live in
`src/aircraft/me262/engine.ts`, and `docs/engine-jumo004.md` describes them. The
input rate exists so that the engine punishes a pilot who slams the lever,
without punishing a pilot who holds a digital button down.

## Camera and view controls

V on the keyboard, Y on the gamepad, or VIEW on the pad steps through four
views: cockpit, chase, orbit and flyby. The simulator starts in chase.

The right stick moves the view. In the cockpit, the chase and the flyby views it
is an absolute head position. The limits are 2.4 rad of yaw and 1.2 rad of
pitch, with a short smoothing time. In the orbit view it is a rate instead, at
1.6 rad per second in yaw and 1 rad per second in pitch.

The mouse works through `src/render/cameras.ts` and not through the binding
table, because a mouse reports movement and not position. Hold the left button
to look around. The offset returns to center about half a second after the
button comes up, except in the orbit view, where it stays. The wheel zooms the
orbit view between 8 m and 400 m, and it starts at 40 m.

F2 turns on a developer free camera. While it is on, W, A, S, D, Q and E fly the
camera and a Shift key makes it eight times faster.

F3 steps the debug level through three settings: nothing, the overlay and the
telemetry graph, and then the force arrows as well.

## Input to command pipeline

```
gamepad, keyboard and touch readers
  -> dead zone and expo, gamepad only
     -> the binding table sums every source into one action
        -> throttle rate integration, held between polls
        -> controlAuthority scales roll, pitch and yaw
        -> the taxi gate decides the two brake commands
           -> ControlInput
              -> AircraftInput, a subset with the same field names
                 -> control deflections, in radians
```

`ControlInput` carries 8 numbers and 14 booleans. The axes run from -1 to 1,
apart from the throttle and the two brakes, which run from 0 to 1. Positive
pitch raises the nose, positive roll rolls right, and positive yaw moves the
nose right.

`AircraftInput` is a strict subset of `ControlInput`, field for field, so
`src/main.ts` passes one straight in as the other with no conversion. The twelve
fields the aircraft does not read go to the camera rig and the guns. They also go
to the controls menu, the panels, the debug level, the respawn and the mute. Two
of those twelve fields go to nothing at all, and both are trim.

## Known gaps

**The control authority law does not cap a pilot who holds the stick.** It
removes the step that a key press makes and nothing more. Tracked as bead 4rq. A
held command sweep at 3000 m measures the peak load factor of a three second
pull:

| Speed | Dynamic pressure | The command that reaches the limit |
| --- | --- | --- |
| 400 km/h | 5612 Pa | full command peaks at 3.09 g |
| 500 km/h | 8769 Pa | full command peaks at 4.96 g |
| 600 km/h | 12627 Pa | 0.30 of the command peaks at 6.85 g |
| 700 km/h | 17186 Pa | 0.05 of the command peaks at 6.03 g |
| 800 km/h and above | above 22 kPa | no command above zero stays under 7 g |

Five percent of the elevator is 1.3 degrees, and at 700 km/h that gives 6 g. No
bound on the deflection can do both jobs. It cannot leave the pilot able to
maneuver at 500 km/h and also hold the aircraft inside the envelope at 800 km/h.
The classic result says that the elevator angle per g is nearly independent of
speed for a rigid stable aircraft. This model gives an angle per g that falls
much faster. The fault is therefore more likely to sit in the pitch damping, the
elevator power, or the damping of the short period. It is less likely to sit in
the input layer. This is not a display fault. The head up display already warns
at the live limit and the structure model already breaks the aircraft.

**The two trim keys do nothing.** The aircraft has no stabilizer trim channel. A
Me-262 pilot trimmed with a moving tailplane, and that channel is the one the
Mach recovery of `docs/validation.md` needs. The flight test stands in for it
with a steady elevator command.

**The on screen pad has no look control.** A pilot on a phone changes the view
with the VIEW button and takes the view the camera rig gives. A second stick for
the head would leave no room for the button block.

**The pad cannot be moved or resized.** The layout is fixed in the style sheet
of `src/input/touch.ts`. A pilot with small hands or a left hand throttle has no
way to change it.

**The bindings cannot be remapped.** `createInputSystem` already takes a table,
and the controls menu already reads one, so the parts are in place. Nothing
writes a new table and nothing stores one.
