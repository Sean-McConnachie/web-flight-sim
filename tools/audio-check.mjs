/**
 * Measures the sound of the simulator in a real browser.
 *
 * Run it with the preview server already up:
 *
 *   npm run build
 *   npx vite preview --port 4173 --strictPort &
 *   node tools/audio-check.mjs
 *
 *
 * WHY THIS TOOL EXISTS
 *
 * test/unit/audio-voices.test.ts measures every LAW of the sound, and it runs
 * in Node because src/audio/voices.ts names no browser API. It therefore proves
 * that the numbers are right and it proves nothing at all about the graph that
 * plays them. A voice that is never connected, a gain that is never written and
 * an oscillator that never starts all pass that suite.
 *
 * This tool taps the master bus with a meter that runs on the AUDIO THREAD and
 * MEASURES the level that comes out of it. A silent engine fails here. A gun
 * that fires and makes no report fails here. Nothing else in the project can
 * catch either fault.
 *
 * It is a test and not a demonstration. Every check below stops the run with an
 * error when the level is wrong.
 *
 *
 * WHAT HEADLESS CHROME DOES WITH THE SOUND
 *
 * There is no audio device, so Chrome runs the graph into a null sink. The
 * audio thread still runs at the real sample rate, the meter still sees every
 * sample, and `currentTime` still advances. The measurement is therefore real.
 * Nobody hears it, which is the point of a headless run.
 *
 * `--autoplay-policy=no-user-gesture-required` takes the unlock out of the way.
 * The unlock itself is a browser rule and not project code, so there is nothing
 * here to test about it.
 */

import { spawn } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const PAGE_URL = process.env.SIM_URL ?? 'http://localhost:4173/';
const PORT = 9334;

/** Time the page needs to build the world and reach a steady frame, ms. */
const SETTLE_MS = 6000;

/** Time a change of flight state needs to reach the audio thread, ms. */
const SHORT_MS = 900;

/**
 * How long the engine start drill is given before the run gives up, ms.
 *
 * It is generous because SIMULATED time runs far slower than real time here.
 * `config.physics.accumulatorCap` lets one frame simulate at most 0.25 s, and a
 * software rasterizer draws a frame in 0.83 s. The clock of the aircraft
 * therefore runs at about a third of the clock on the wall, and the handbook
 * start takes 25 s on the aircraft clock.
 */
const START_TIMEOUT_MS = 240_000;

function wait(ms) {
  return new Promise((done) => setTimeout(done, ms));
}

async function findPageTarget() {
  for (let attempt = 0; attempt < 60; attempt++) {
    try {
      const response = await fetch(`http://127.0.0.1:${PORT}/json/list`);
      const targets = await response.json();
      const page = targets.find((t) => t.type === 'page' && t.webSocketDebuggerUrl);
      if (page !== undefined) return page;
    } catch {
      // The browser is not listening yet.
    }
    await wait(250);
  }
  throw new Error('Chrome never opened its debugging port.');
}

function createClient(url) {
  const socket = new WebSocket(url);
  const pending = new Map();
  const events = [];
  let nextId = 1;

  const ready = new Promise((done, fail) => {
    socket.addEventListener('open', () => done());
    socket.addEventListener('error', () => fail(new Error('The debugger socket failed.')));
  });

  socket.addEventListener('message', (event) => {
    const message = JSON.parse(event.data);
    if (message.id === undefined) {
      events.push(message);
      return;
    }
    const entry = pending.get(message.id);
    if (entry === undefined) return;
    pending.delete(message.id);
    if (message.error !== undefined) entry.fail(new Error(message.error.message));
    else entry.done(message.result);
  });

  return {
    ready,
    events,
    send(method, params = {}) {
      const id = nextId++;
      return new Promise((done, fail) => {
        pending.set(id, { done, fail });
        socket.send(JSON.stringify({ id, method, params }));
      });
    },
    close() {
      socket.close();
    },
  };
}

/** Formats a level as decibels against full scale, which is how a mix reads. */
function dbfs(rms) {
  if (rms <= 0) return '-inf dBFS';
  return `${(20 * Math.log10(rms)).toFixed(1)} dBFS`;
}

async function main() {
  const chrome = spawn(
    'google-chrome',
    [
      '--headless=new',
      `--remote-debugging-port=${PORT}`,
      '--remote-allow-origins=*',
      '--user-data-dir=/tmp/hfs-audio-profile',
      '--disable-gpu',
      '--use-gl=angle',
      '--use-angle=swiftshader',
      '--disable-accelerated-2d-canvas',
      // The whole reason a headless run can measure anything.
      '--autoplay-policy=no-user-gesture-required',
      '--window-size=1280,720',
      PAGE_URL,
    ],
    { stdio: 'ignore' },
  );

  const target = await findPageTarget();
  const cdp = createClient(target.webSocketDebuggerUrl);
  await cdp.ready;
  await cdp.send('Page.enable');
  await cdp.send('Runtime.enable');
  await cdp.send('Log.enable');

  async function evaluate(expression) {
    const result = await cdp.send('Runtime.evaluate', {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (result.exceptionDetails !== undefined) {
      throw new Error(`The page threw: ${result.exceptionDetails.text}`);
    }
    return result.result.value;
  }

  async function pressKey(code, key) {
    await cdp.send('Input.dispatchKeyEvent', { type: 'keyDown', code, key });
    await cdp.send('Input.dispatchKeyEvent', { type: 'keyUp', code, key });
  }

  async function holdKey(code, key) {
    await cdp.send('Input.dispatchKeyEvent', { type: 'keyDown', code, key });
  }

  async function releaseKey(code, key) {
    await cdp.send('Input.dispatchKeyEvent', { type: 'keyUp', code, key });
  }

  console.log(`page: ${PAGE_URL}`);
  await wait(SETTLE_MS);

  const backend = await evaluate('window.sim.bundle.backend');
  console.log(`backend: ${backend}`);

  // --- The tap -------------------------------------------------------------
  // The meter hangs off the MASTER GAIN, which is what every voice connects to.
  // That is the mix before the compressor, so the numbers below are what the
  // voices made and not what the compressor left of it.
  //
  // IT ALSO FORCES A KNOWN VOLUME AND MUTE. src/audio/context.ts keeps both in
  // local storage so they survive a reload, and this profile directory survives
  // a whole RUN. A run that ended on the mute check therefore started the next
  // run muted, and every level below read as digital silence. That is the
  // persistence working correctly and the harness reading it wrongly.
  //
  // Full volume, and not the 0.7 a first visit gets, so two runs are
  // comparable.
  //
  // THE METER RUNS ON THE AUDIO THREAD, in an `AudioWorkletNode`. An
  // `AnalyserNode` cannot do this job here and the reason is worth writing
  // down.
  //
  // An analyser holds a WINDOW of recent history, and something on the main
  // thread has to read it. This main thread is blocked for most of every
  // frame, so a read lands one time per frame and no more. The largest window
  // the API allows is 32768 samples, which is 683 ms. On the live site a frame
  // takes 1400 ms. The window therefore cannot reach back to the last frame,
  // and the guns fire in a burst that lasts 140 ms, so every read fell in the
  // silence between the bursts and the guns read 26 dB too quiet.
  //
  // A worklet sees EVERY sample. It keeps a running peak and a running mean
  // square, and the main thread asks for them whenever it gets a turn. No
  // sample can fall between two reads, whatever the frame rate does.
  const tapped = await evaluate(`(async () => {
    const bus = window.sim.sound.bus;
    if (bus === null) return 'no Web Audio in this browser';
    bus.muted = false;
    bus.volume = 1;
    const context = bus.context;

    const code = \`
      class Meter extends AudioWorkletProcessor {
        constructor() {
          super();
          this.peak = 0; this.sum = 0; this.count = 0;
          this.port.onmessage = () => {
            this.port.postMessage({
              peak: this.peak,
              rms: this.count > 0 ? Math.sqrt(this.sum / this.count) : 0,
            });
            this.peak = 0; this.sum = 0; this.count = 0;
          };
        }
        process(inputs) {
          const channel = inputs[0] && inputs[0][0];
          if (channel !== undefined) {
            for (let i = 0; i < channel.length; i++) {
              const v = channel[i];
              const m = v < 0 ? -v : v;
              if (m > this.peak) this.peak = m;
              this.sum += v * v;
              this.count++;
            }
          }
          return true;
        }
      }
      registerProcessor('hfs-meter', Meter);
    \`;
    const url = URL.createObjectURL(new Blob([code], { type: 'application/javascript' }));
    await context.audioWorklet.addModule(url);
    const meter = new AudioWorkletNode(context, 'hfs-meter');
    bus.destination.connect(meter);
    // A node whose output reaches nothing may never be pulled. A silent gain to
    // the speakers guarantees the graph runs it and adds nothing audible.
    const sink = context.createGain();
    sink.gain.value = 0;
    meter.connect(sink);
    sink.connect(context.destination);

    window.__readMeter = () =>
      new Promise((done) => {
        meter.port.onmessage = (event) => done(event.data);
        meter.port.postMessage('read');
      });
    return context.state;
  })()`);
  if (tapped !== 'running') {
    throw new Error(`The audio context is "${tapped}". It must be running to measure anything.`);
  }
  console.log('tap: on the master gain, context running');

  /**
   * Reads the loudest level over a window, and the level a long way into it.
   *
   * A gun report is a peak that lasts 40 ms and an engine is a level that
   * holds, so one number cannot describe both. `peak` catches the report.
   * `steady` is the mean of the last third of the window, which the peak of a
   * one shot has already left.
   */
  /**
   * `hold` is JavaScript that runs INSIDE the page before every read.
   *
   * IT IS NOT OPTIONAL FOR A MEASUREMENT IN THE AIR, and this cost a long
   * detour to work out. A software rasterizer draws a frame in 430 ms, so a
   * `setTimeout` of 25 ms lands on the next frame and a window that asks for
   * 700 ms of samples takes TWELVE SECONDS of real time.
   *
   * The aircraft is untrimmed. Twelve seconds at 250 m/s is a loop, a stall and
   * a decay to about 56 m/s, so the tail of the window measured a different
   * aircraft from the one the test placed. Every level came out 30 dB low and
   * the sound was correct the whole time.
   *
   * `hold` pins the state on every read. What comes back is then the state the
   * test asked for.
   */
  async function measure(ms = 700, hold = '') {
    const steps = Math.max(Math.round(ms / 60), 8);
    return await evaluate(`(async () => {
      // Throw away everything the meter holds from before the window.
      await window.__readMeter();
      for (let i = 0; i < ${steps}; i++) {
        ${hold}
        await new Promise((r) => setTimeout(r, 25));
      }
      const m = await window.__readMeter();
      return {
        // The mean square over the WHOLE window. It is the right measure for a
        // source that runs all the time.
        rms: m.rms,
        // The largest single sample in the window. It is the right measure for
        // a one shot, which a mean over a window mostly full of gaps buries.
        sampleMax: m.peak,
      };
    })()`);
  }

  /**
   * Places the aircraft, level, at a speed and a height, and moves the camera
   * with it.
   *
   * THE SNAP IS NOT OPTIONAL. The chase camera lags the aircraft on purpose,
   * and a teleport leaves it hundreds of meters behind. The spread of
   * src/audio/voices.ts then reads that distance and takes 30 dB off
   * everything, so a measurement without the snap measures the camera lag and
   * not the sound.
   *
   * The aircraft is also UNTRIMMED, so it pulls up hard at any real speed. The
   * caller places it again a moment before it measures, for the same reason
   * tools/screenshot.mjs does.
   */
  function placeExpression(speed, altitude) {
    return `(() => {
      const body = window.sim.aircraft.state.body;
      body.position.set(1200, 0, ${-altitude});
      body.velocity.set(${speed}, 0, 0);
      body.orientation.identity();
      body.angularVelocity.set(0, 0, 0);
      window.sim.rig.snap();
    })();`;
  }

  async function place(speed, altitude) {
    await evaluate(placeExpression(speed, altitude));
  }

  /**
   * Places the aircraft, lets the sound settle, then measures while HOLDING it
   * there. Read the comment on `measure` for why the hold is what makes this
   * work at all.
   */
  async function measurePlaced(speed, altitude, ms = 600) {
    const hold = placeExpression(speed, altitude);
    await place(speed, altitude);
    // The gains of src/audio smooth over 40 ms and the frame may be far slower
    // than that, so give the ramps a few frames before the window opens.
    await wait(SHORT_MS);
    return await measure(ms, hold);
  }

  /**
   * Prints what the propagation stage is doing.
   *
   * An analyser reports ONE number, and a mix that is 26 dB down has a dozen
   * possible causes. `SoundMetrics` of src/audio/sound.ts is what tells them
   * apart, and this line is the reason that hook exists.
   */
  async function metrics(label) {
    const m = await evaluate('window.sim.sound.metrics');
    console.log(
      `  ${label}: distance ${m.distance.toFixed(1)} m, spread ${m.spread.toFixed(3)}, ` +
        `duck ${m.busGain.toFixed(3)}, wind ${m.windLevel.toFixed(4)}, ` +
        `delay ${(m.delay * 1000).toFixed(0)} ms, cutoff ${m.cutoff.toFixed(0)} Hz, ` +
        `snaps ${m.snaps}, frame ${(m.frameDt * 1000).toFixed(0)} ms`,
    );
    return m;
  }

  const failures = [];
  function check(name, pass, detail) {
    console.log(`${pass ? 'PASS' : 'FAIL'}  ${name}  ${detail}`);
    if (!pass) failures.push(`${name}: ${detail}`);
  }

  // --- 1. Silence ----------------------------------------------------------
  // The aircraft is parked with the engines off. Nothing moves, so nothing in
  // the whole graph has anything to say.
  //
  // It uses `spawnOnRunway` and NOT `place`. `place` writes the position of the
  // center of gravity, so an altitude of zero puts the wheels and the belly
  // UNDER the runway. The ground model then throws the aircraft out, and the
  // scrape and the touch down that follow are the correct answer to a silly
  // question. The spawn puts it on its wheels the way the simulator does.
  await evaluate('window.sim.aircraft.spawnOnRunway()');
  await wait(SHORT_MS * 3);
  const silence = await measure(400);
  check(
    'a parked aircraft with the engines off is silent',
    silence.rms < 0.002,
    `${dbfs(silence.rms)}`,
  );

  // --- 2. The wind ---------------------------------------------------------
  // 250 m/s is 900 km/h, which is the reference speed of the Curle law in
  // src/audio/voices.ts. The wind alone must carry the mix there.
  const fast = await measurePlaced(250, 1400);
  await metrics('at 900 km/h');
  check(
    'the rush of air at 900 km/h is loud on its own',
    fast.rms > 0.03,
    `${dbfs(fast.rms)}, against ${dbfs(silence.rms)} at rest`,
  );

  // --- 3. The wind follows the speed ---------------------------------------
  // The law is a cube of the speed, so half the speed is 18 dB down. The
  // measurement is against the graph and not against the law, so the check is
  // only that it fell a long way.
  const half = await measurePlaced(125, 1400);
  check(
    'halving the speed takes the wind a long way down',
    half.rms < fast.rms * 0.5,
    `${dbfs(half.rms)} at 450 km/h, against ${dbfs(fast.rms)} at 900`,
  );

  // --- 4. The guns ---------------------------------------------------------
  // The guns fire from the PARKED aircraft, against the silence of check 1.
  //
  // Every other source is off there, so whatever the analyser reads is the
  // guns and nothing else. A test in the air measures the guns AGAINST the
  // wind, and the wind follows the cube of the speed, so the answer says more
  // about the speed the test chose than about the guns.
  await evaluate('window.sim.aircraft.spawnOnRunway()');
  await wait(SHORT_MS * 3);
  const beforeGuns = await measure(400);
  const ammoBefore = await evaluate('window.sim.armament.roundsLeft');
  await holdKey('Space', ' ');
  await wait(300);
  const firing = await measure(600);
  await releaseKey('Space', ' ');
  const ammoAfter = await evaluate('window.sim.armament.roundsLeft');
  // Two separate facts. A quiet gun and a gun that never fired look the same
  // on the analyser, and only one of them is a fault in the sound.
  check(
    'the trigger reaches the guns',
    ammoAfter < ammoBefore,
    `${ammoBefore - ammoAfter} rounds left the barrels`,
  );
  // A gun is a one shot, so it answers the largest SAMPLE and not a mean over
  // a window that mostly holds the gaps between the reports.
  check(
    'the four MK 108 make a report',
    firing.sampleMax > 0.1,
    `loudest sample ${dbfs(firing.sampleMax)} while firing, ` +
      `against ${dbfs(beforeGuns.sampleMax)} parked and silent`,
  );

  // --- 5. The mute ---------------------------------------------------------
  const muteHold = placeExpression(250, 1400);
  await place(250, 1400);
  await wait(SHORT_MS);
  await pressKey('KeyM', 'm');
  await wait(SHORT_MS);
  const muted = await measure(400, muteHold);
  check('the M key mutes everything', muted.sampleMax < 0.001, `${dbfs(muted.sampleMax)}`);
  const buttonSaysOff = await evaluate(
    `document.querySelector('.hfs-sound-open').textContent`,
  );
  check(
    'the SOUND button reads the mute back',
    buttonSaysOff === 'SOUND OFF',
    `the button reads "${buttonSaysOff}"`,
  );

  await pressKey('KeyM', 'm');
  await wait(SHORT_MS);
  const unmuted = await measure(500, muteHold);
  check(
    'the M key brings the sound back',
    unmuted.rms > 0.01,
    `${dbfs(unmuted.rms)}`,
  );

  // --- 6. The engines ------------------------------------------------------
  // This is the real start drill and not a state set by hand. Hold the starter
  // with the throttle closed, exactly as the handbook says, and wait for the
  // rotor to reach idle. It also proves that the start makes a sound, which is
  // the one part of the flight the pilot cannot see on any instrument.
  await evaluate('window.sim.aircraft.spawnOnRunway()');
  await wait(SHORT_MS);
  const parked = await measure(400);

  await holdKey('Home', 'Home');
  const deadline = Date.now() + START_TIMEOUT_MS;
  let cranking = null;
  let rpm = 0;
  while (Date.now() < deadline) {
    const state = await evaluate(`(() => {
      const e = window.sim.aircraft.state.engines[0];
      return { rpm: e.rpm, state: e.state, sim: window.sim.loop.stats.simTime };
    })()`);
    rpm = state.rpm;
    if (cranking === null && state.rpm > 300 && state.rpm < 1600) {
      cranking = await measure(400);
    }
    if (state.state === 'idle' || state.state === 'running') break;
    await wait(500);
  }
  await releaseKey('Home', 'Home');

  if (cranking !== null) {
    check(
      'the Riedel starter is heard while it cranks the rotor',
      cranking.rms > parked.rms * 3,
      `${dbfs(cranking.rms)} on the starter, against ${dbfs(parked.rms)} parked`,
    );
  } else {
    check('the Riedel starter is heard while it cranks the rotor', false, 'the crank was missed');
  }

  await wait(SHORT_MS);
  const idle = await measure();
  check(
    'the engine at idle makes a sound',
    idle.rms > parked.rms * 4,
    `${dbfs(idle.rms)} at ${rpm.toFixed(0)} rpm, against ${dbfs(parked.rms)} with it off`,
  );

  // --- 7. Power ------------------------------------------------------------
  // OPEN THE LEVER SLOWLY. The first version of this test held Page Up for
  // four seconds from idle, and the engine surged and flamed out. That was not
  // a fault in the simulator. It is the danger band of the Jumo 004, which
  // src/aircraft/me262/engine.ts reproduces on purpose, and the README warns
  // about it in as many words. The test was flying the aircraft badly.
  //
  // Under 6000 rpm the lever moves in small steps and waits for the rotor.
  // Over it, a slam is safe and the lever moves freely.
  const spoolDeadline = Date.now() + 200_000;
  let power = { rpm: 0, state: 'off', throttle: 0 };
  while (Date.now() < spoolDeadline) {
    power = await evaluate(`(() => {
      const e = window.sim.aircraft.state.engines[0];
      return {
        rpm: e.rpm,
        state: e.state,
        thrust: e.thrust,
        throttle: window.sim.input.state.throttle,
      };
    })()`);
    if (power.rpm > 7200) break;
    // The engine let go. Stop, and let the check below report it.
    if (power.state !== 'idle' && power.state !== 'running') break;
    const gentle = power.rpm < 6000;
    await holdKey('PageUp', 'PageUp');
    await wait(gentle ? 220 : 900);
    await releaseKey('PageUp', 'PageUp');
    await wait(gentle ? 2200 : 700);
  }

  check(
    'a slow lever keeps the engine lit through the danger band',
    power.state === 'running' || power.state === 'idle',
    `state "${power.state}" at ${power.rpm.toFixed(0)} rpm, lever ${power.throttle.toFixed(2)}`,
  );

  const loud = await measure(500);
  check(
    'the engine gets much louder with power',
    loud.rms > idle.rms * 2.5,
    `${dbfs(loud.rms)} at ${power.rpm.toFixed(0)} rpm, against ${dbfs(idle.rms)} at idle`,
  );

  // --- 7. Nothing threw ----------------------------------------------------
  const errors = cdp.events
    .filter((e) => e.method === 'Log.entryAdded' && e.params.entry.level === 'error')
    .map((e) => `${e.params.entry.text} ${e.params.entry.url ?? ''}`)
    // A browser asks for /favicon.ico on its own and the project ships none.
    // That 404 belongs to no code here and it says nothing about the sound.
    .filter((text) => !text.includes('favicon'));
  check('the page logged no error', errors.length === 0, errors.join(' | ') || 'none');

  cdp.close();
  chrome.kill();

  if (failures.length > 0) {
    console.error(`\n${failures.length} check(s) failed:`);
    for (const line of failures) console.error(`  ${line}`);
    process.exitCode = 1;
    return;
  }
  console.log('\nevery check passed');
}

main().catch((error) => {
  console.error(String(error));
  process.exitCode = 1;
  process.exit(1);
});

void ROOT;
