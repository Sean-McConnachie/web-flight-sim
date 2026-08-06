# Sound

Every sound in this simulator is built at run time from the flight state. The
project ships no audio file.

That is the same rule the rest of the project follows. The flight model builds
its forces from twenty two strips instead of one table of coefficients. The
runway markings are drawn into a canvas. The sound is made the same way, out of
the numbers the model already holds.

## Why not a recording

A recording of a jet is one engine, at one power setting, at one distance, in
one direction. Every other condition has to be a fade between samples.

A synthesized engine answers the rotor speed, the thrust and the distance,
because those are the values it is built from. Open the throttle and the sound
changes in three separate ways at once, because three separate sources answer
the throttle differently.

The cost is the exact timbre of a Jumo 004. Only a recording can give that, and
no recording of one exists.

## Where the code is

| File | Content |
| --- | --- |
| `src/audio/voices.ts` | every law, and no browser API at all |
| `src/audio/context.ts` | the audio context, the master bus, the mute |
| `src/audio/noise.ts` | the white and pink noise, and the one shot burst |
| `src/audio/engine-voice.ts` | one Jumo 004 |
| `src/audio/airframe-voice.ts` | the wind and the buffet |
| `src/audio/mechanical-voice.ts` | the gear, the flaps, the tires, the ground |
| `src/audio/gun-voice.ts` | the four MK 108 and the shell impacts |
| `src/audio/sound.ts` | the bus, the propagation stage and the two updates |

`src/audio/voices.ts` names no `AudioContext`, no `window` and no `document`.
`test/unit/audio-voices.test.ts` therefore runs it in Node with the rest of the
model, and it measures 38 facts about the sound. A law that no test can measure
is a law nobody can check.

Every other file in `src/audio` is a graph of Web Audio nodes with no arithmetic
in it. Those files read the records that `voices.ts` fills and write the numbers
into an `AudioParam`.

## The three sources of a turbojet

A turbojet is not one sound. Jet acoustics splits it into three sources, and
each one wins in a different part of the range.

| Source | What makes it | Law | Where it wins |
| --- | --- | --- | --- |
| Jet mixing noise | the shear layer behind the nozzle | the eighth power of the jet velocity | full power |
| Core noise | the combustion in the chambers | the fuel flow | idle |
| Compressor tone | the blades passing the inlet | the cube of the tip speed | from the first turn of the rotor |

**Jet mixing noise.** Lighthill showed that the acoustic power of a free jet
follows the eighth power of the jet velocity. Sound pressure is the square root
of the power, so the amplitude follows the fourth power. Source: Lighthill, "On
sound generated aerodynamically I", Proc. R. Soc. A 211 (1952), page 564.

The jet velocity comes from the momentum equation. A turbojet makes a thrust of
`mdot (Vj - V0)`, so `Vj = V0 + T / mdot`. At full power, at rest, at sea level,
that gives 8800 / 21.2, which is 415 m/s.

**Why the eighth power law is not enough.** It gives 63 dB between idle and full
power. A real engine gives near 30. The law is not wrong. Jet mixing noise
really does fall that fast, and at idle it has stopped being the source anybody
hears. The other two sources hold the idle level up.

That is the reason this file models three sources and not one. A single source
with a softened exponent would sound the same at every power setting, only
louder.

**The compressor tone.** A compressor blade is a lifting surface in a flow. Its
unsteady loading radiates as a dipole, and Curle gives a dipole a power that
follows the sixth power of the speed. The amplitude is the square root of that,
so it follows the cube of the blade tip speed. Source: Curle, "The influence of
solid boundaries upon aerodynamic sound", Proc. R. Soc. A 231 (1955).

The pitch of the tone is the blade passing frequency. It is the shaft frequency
times the blade count of the first compressor stage. At the maximum rotor speed
of 8700 rpm it reads 3.9 kHz.

## The airframe

The wind noise follows the same Curle dipole law as the compressor tone. The
skin of an aircraft in a turbulent boundary layer is a rigid surface in a flow.
The amplitude therefore follows the cube of the speed, and a doubling of the
speed adds 18 dB.

The landing gear and the flaps add to it. A gear leg is a bluff body in the free
stream, and it is the loudest part of an airliner on the approach. The eddies it
sheds are the size of the wheel, so they arrive lower in frequency than the
eddies in the boundary layer. Lower the gear and the rush of air gets louder and
drops in pitch.

## The buffet is the stall warning

The Me-262 has no stick shaker and no stall warning horn. The only warning a
pilot gets before the wing lets go is the airframe starting to shake.

Until the sound arrived, this simulator gave a pilot no warning at all. A screen
cannot shake a seat.

The buffet starts at 14 degrees of angle of attack. It is fully developed at the
20.3 degrees where the clean wing reaches its maximum lift. A pilot who listens
now has the same six degrees of warning the real aircraft gives.

A second buffet arrives at Mach 0.80. The shock that makes the Mach tuck
separates the flow behind it first, so the shake comes before the trim change.
It shakes faster than the stall buffet, because the eddies behind a shock are
smaller than the eddies off a stalled wing.

The buffet is not a sound added on top of the wind. It is the wind itself going
up and down, which is what a wing that sheds its flow does to the air over the
tail. An oscillator drives the gain of the wind, and that is amplitude
modulation.

## Distance, and why the delay line matters

Everything on the aircraft shares one propagation stage:

```
the voices -> delay -> air absorption -> duck -> spread -> pan -> master bus
```

The duck sits AFTER the delay line and not before it. Its job is to suppress the
click a delay line makes when its read point moves. A delay line holds a tenth
of a second of sound that has already gone in. Ducking the input cannot reach
that sound. Ducking the output can.

**Spread.** Sound goes out over the surface of a sphere, so the pressure falls
with the distance. The level halves for every doubling of the distance. The
reference is 26 m, which is the distance of the chase view, so the mix is set in
the view the simulator starts in.

**Air absorption.** Air takes the high frequencies off far faster than the low
ones. That is why a jet overhead cracks and the same jet at a kilometer rumbles.
A one pole low pass whose corner falls with the distance has the same behavior.
The corner sits at 20 kHz at 20 m, at 2 kHz at 200 m, and at 400 Hz at a
kilometer.

**The delay.** The delay holds the distance over the speed of sound. This is the
most important line in the whole system, and it does two things that nothing
else does.

First, it makes a fly by arrive late. A listener hears an aircraft 340 m away
where it stood one second ago. The sound therefore reaches the fly by camera
after the aircraft has gone past it.

Second, and this is the part that matters: **a variable delay line IS the
Doppler shift.** There is no pitch calculation anywhere in `src/audio`.

Hold a source at a fixed delay and it plays at its own pitch. Make the delay
grow at a rate `k` and every wave front arrives `1 + k` times further apart, so
the pitch falls by that factor. The delay is the distance over the speed of
sound, so `k` is the radial speed over the speed of sound. That makes the factor
`1 / (1 + v / c)`, which is the Doppler formula for a moving source.

The frame ramps the delay to its new value over exactly one frame. The rate of
that ramp is the radial speed over the speed of sound. The Doppler shift then
falls out of holding one number at the correct value.

`test/unit/audio-voices.test.ts` measures the two forms against each other over
seven radial speeds and holds them to ten decimal places.

The effect is large. The aircraft passes the fly by camera at 900 km/h, which is
Mach 0.735. The 3.9 kHz compressor tone arrives at 14.8 kHz on the way in and at
2.3 kHz on the way out. That whole drop lands in the second the aircraft passes.

**The one case a delay line cannot handle is a jump.** A respawn or a change of
view moves the listener hundreds of meters between two frames. A ramp across
that would sweep the pitch by a factor of fifty. The frame therefore measures
the size of the change. A jump gets a hard set and a 120 ms duck instead of a
ramp.

## The cockpit

Nothing in an unpressurized fighter is louder than the air over the hood seals.
The wind therefore gets a boost inside the cockpit, and the engines get a low
pass at 1800 Hz.

The low pass is only on the engines. A gear leg that locks is structure borne,
so the pilot hears it through the airframe and not through the hood.

## What each sound answers

| Sound | What drives it |
| --- | --- |
| Compressor whine | rotor speed, through the blade passing frequency |
| Jet roar | thrust and rotor speed, through the jet velocity |
| Combustion rumble | fuel flow |
| Riedel starter | rotor speed while the starter is engaged |
| Surge bang | `EngineEvents.surgeBangCount` |
| Light off, flameout | `EngineEvents` |
| Jet pipe fire | the engine state |
| Wind | true airspeed, gear position, flap position |
| Stall buffet | angle of attack |
| Mach buffet | Mach number |
| Tire rumble | wheel speed and the load through the legs |
| Tire squeal | slip ratio and wheel speed |
| Hydraulic pump | the rate the gear or the flaps travel at |
| Gear lock, flap detent | the travel reaching an end stop |
| Touch down | the rate the oleo closes at |
| Airframe scrape | the slide speed and load of every airframe contact point |
| MK 108 | every round that leaves a barrel |
| Shell impact | the distance from the listener to where it landed |
| Structural failure | the `failure` event of the aircraft |

## The shell lands somewhere else

A muzzle report starts at the aircraft. An impact starts wherever the shell came
down, which the ballistics lets run to 2500 m. The two cannot share a
propagation stage.

The muzzle goes onto the aircraft bus with the engines. The impact is scheduled
into the future by its own distance over the speed of sound. A pilot who fires
at a ground target and hears it three seconds later is hearing the correct
answer.

## The two update calls

`fixedUpdate` runs on every physics step at 240 Hz. `update` runs one time per
frame. A frame can hold four steps or none.

Anything that lasts one step must be caught in `fixedUpdate`, or a frame will
miss it. A surge bang, a round leaving a barrel and a shell arriving are all one
step long.

Anything continuous is written in `update`. Writing a gain 240 times a second
buys nothing over writing it 60 times and letting the audio thread smooth
between them.

## Why the sound does not start on its own

A browser starts every `AudioContext` suspended. It only lets a page resume one
from inside a handler for a real user action. There is no way around this, and
it is a good rule. A page that makes a noise at a person who did not ask for one
is what the rule exists to stop.

The simulator builds the whole graph at load, leaves it suspended, and resumes
it on the first key press, pointer press or touch. A pilot who flies has already
pressed a key, so the sound starts on its own.

The SOUND button at the top left says which state the sound is in. It reads
`ENABLE SOUND` while the browser holds the context shut, `SOUND ON` while it
runs and `SOUND OFF` after a mute.

## The controls

| Action | Control |
| --- | --- |
| Mute, or bring the sound back | the M key, or the SOUND button |
| Master volume | the slider in the controls menu |

The volume and the mute both survive a reload. The mute is on a key and on a
button because a pilot who wants silence wants it at once. The volume is in a
panel because it is set one time and then left alone.

## How to measure it

```sh
npm run build
npx vite preview --port 4173 --strictPort &
npm run audio-check
```

`tools/audio-check.mjs` starts headless Chrome, taps the master bus with an
`AnalyserNode`, and measures the level that comes out of it.

This tool exists because the unit tests cannot catch a whole class of fault. A
voice that is never connected, a gain that is never written and an oscillator
that never starts all pass the unit suite. They all fail here.

It runs the real engine start drill and not a state set by hand. It holds the
starter, waits for the rotor to reach idle, opens the throttle, and checks that
the level rises at each step. It fires the guns and checks for a report. It
presses M and checks for silence.

Headless Chrome has no audio device, so it runs the graph into a null sink. The
audio thread still runs at the real sample rate. The analyser still sees the
samples, so the measurement is real. Nobody hears it, which is the point of a
headless run.

### What it caught before it ever passed

The tool found five faults that no unit test could reach. All five are fixed.

**Four guns fired as one.** Each burst picked its read point in the noise buffer
from the clock. `currentTime` DOES NOT ADVANCE inside one JavaScript task. The
four guns that fire on one physics frame therefore got the same read point and
the same samples. Four identical bursts add coherently. They came out four times
as loud as one gun and they sounded like one gun. A counter picks the read point
now.

**A one shot could lose its own attack.** `currentTime` is the start of the
render quantum the audio thread is ALREADY working on. A burst anchored there
asks for something that thread has partly gone past. A gun report is mostly its
front, so the front went missing. Every one shot is scheduled 20 ms ahead now.

**The jump test had no frame time in it.** It compared the change of delay in
ONE FRAME against a fixed 50 ms. That bound holds at 60 frames a second. The
aircraft moves 4 m between frames there. A software rasterizer draws a frame in
430 ms, and the same aircraft covers 107 m in that time. Every frame then looked
like a jump. The bus ducked on every frame, so the sound broke up on exactly the
machines that already struggle. The bound is a RATE now.

**The duck sat on the wrong side of the delay line.** Its whole job is to
suppress the click a delay line makes when its read point moves. It acted on the
INPUT of that line, and a delay line holds a tenth of a second of sound that has
already gone in. Ducking the input cannot reach that. It ducks the output now.

**A spawn made a landing noise.** `spawnOnRunway` puts the aircraft on its
wheels and drops the gear in one step. Every latch read that as an arrival and a
gear lock, so the page opened with a thump and every respawn answered with one.
A spawn now resets each latch without playing what it finds.

### What it taught about measuring

Two of the early failures were faults in the TOOL, and both are worth recording
because both looked exactly like a broken mix.

The measurement window ran far longer than it asked for. A `setTimeout` of 25 ms
lands on the next frame. At 430 ms a frame, a window that asks for 700 ms of
samples takes twelve seconds. The aircraft is untrimmed, so twelve seconds at
250 m/s is a loop, a stall and a decay to about 56 m/s. The tail of every window
measured a different aircraft from the one the test placed. Every level came out
30 dB low, and the sound was correct the whole time. The window now pins the
state before each read.

The volume and the mute survive a reload, and the browser profile survives a
whole RUN. A run that ended on the mute check started the next run muted, and
every level read as digital silence. The tool now sets a known volume first.

A third failure was not a fault in either. The power check held Page Up for four
seconds from idle, and the engine surged and flamed out, so the level fell
instead of rising. That is the danger band of the Jumo 004 working exactly as
`src/aircraft/me262/engine.ts` builds it. The test was flying the aircraft
badly. It opens the lever in small steps under 6000 rpm now, and it checks that
the engine is still lit at the top.

The measurements also SET the mix. The engine at idle came out 23 dB under the
Riedel starter that had just lit it. That is the wrong way round by a wide
margin. The trims in `src/audio/engine-voice.ts` are the corrected values, and
the tool holds them.

The lesson from both: when a measurement disagrees with the design by 30 dB,
suspect the measurement first. `SoundMetrics` exists for that. One number off an
analyser has a dozen possible causes. The distance, the spread, the duck and the
wind gain together say which one it is.

## Numbers that are estimated

Section 8 of `docs/CONVENTIONS.md` says to estimate a missing number, mark the
estimate, and say how it was made. The four below set the PITCH of a sound, and
they are the ones a reader is most likely to want to check. Others set a level
or a filter corner, and the module comment marks each one where it stands.

No flight test measures any of them. A wrong value here changes what the
simulator sounds like and nothing else.

| Number | Value | How it was estimated |
| --- | --- | --- |
| Blade count of the first compressor stage | 27 | mean radius 0.24 m, blade pitch close to the 55 mm chord |
| Jet pipe diameter at the nozzle | 0.55 m | read off a general arrangement drawing |
| Riedel firings for one turn of the main rotor | 10 | a two stroke twin at a reduction near 5 to 1 |
| Tread blocks on a main tire | 66 | an 840 by 300 mm tire with a block every 40 mm |

The check on the blade count is the tone it makes. At 8700 rpm it gives 3.9 kHz,
and the whine of a small early turbojet sits between 2 and 5 kHz.

## Known gaps

**There is no front to back cue.** The panning is equal power between two
channels. A listener can tell left from right and cannot tell in front from
behind. A `PannerNode` with an HRTF would fix it and would cost far more than
every other voice together.

**The engine pan does not follow the aircraft.** Each engine sits at a fixed
place in the stereo field. A roll through 90 degrees should carry one nacelle
over the other, and it does not. The pan of each voice is set one time, and the
aircraft as a whole carries only one pan of its own.

**Nothing echoes off the ground.** A real aircraft near the ground gets a
reflection a few milliseconds after the direct sound. That comb filter is part
of what a low pass sounds like.

**The air absorption is a fit and not a calculation.** ISO 9613-1 gives the real
curve, and it depends on the frequency, the humidity and the temperature. This
is a one pole low pass fitted to the shape of that curve.

**A departed wing panel changes no sound.** The engine voice follows the engine
model, and the engine model keeps running after the panel outboard of it has
gone. The airframe voice does not answer the failure either, although a wing
that has torn off makes a great deal of noise.
