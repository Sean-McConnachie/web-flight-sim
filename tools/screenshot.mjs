/**
 * Takes the screenshots that README.md shows.
 *
 * Run it with the preview server already up:
 *
 *   npm run build
 *   npx vite preview --port 4173 --strictPort &
 *   node tools/screenshot.mjs
 *
 * The script starts headless Chrome, drives the page over the Chrome DevTools
 * Protocol, and writes one picture for each shot into docs/images/. `shoot`
 * below says which shots come out as a JPEG and which come out as a PNG.
 *
 *
 * WHY IT DRIVES THE PAGE AND DOES NOT ONLY POINT A CAMERA
 *
 * The interesting pictures need a state that a fresh page does not hold: the
 * cockpit view, an aircraft in the air, and the controls menu open. The
 * protocol gives three tools for that. `Runtime.evaluate` reaches `window.sim`,
 * which src/main.ts exposes for exactly this. `Input.dispatchMouseEvent` presses
 * a real button at real coordinates. `Input.dispatchKeyEvent` presses a real
 * key. The last two also PROVE the control works, which a screenshot of a
 * state set by hand would not.
 *
 *
 * WHAT THE PICTURES ARE NOT
 *
 * Headless Chrome quietly drops this project to the WebGL 2 backend, and this
 * script asks for the software rasterizer on top of that. CONVENTIONS section
 * 6a records both. The pictures are therefore honest about the layout, the
 * instruments and the controls. They are NOT a measure of what the WebGPU path
 * looks like on a real card, and they are not a frame rate.
 */

import { spawn } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const OUT_DIR = resolve(ROOT, 'docs/images');
const PAGE_URL = process.env.SIM_URL ?? 'http://localhost:4173/';
const PORT = 9333;

/** Time the software rasterizer needs to draw a settled frame, ms. */
const SETTLE_MS = 6000;

/** Time a small change needs, such as a panel that opens, ms. */
const SHORT_MS = 1500;

function wait(ms) {
  return new Promise((done) => setTimeout(done, ms));
}

/** Asks the browser for its list of targets until one answers. */
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

/** A minimal Chrome DevTools Protocol client over one web socket. */
function createClient(url) {
  const socket = new WebSocket(url);
  const pending = new Map();
  let nextId = 1;

  const ready = new Promise((done, fail) => {
    socket.addEventListener('open', () => done());
    socket.addEventListener('error', () => fail(new Error('The debugger socket failed.')));
  });

  socket.addEventListener('message', (event) => {
    const message = JSON.parse(event.data);
    const entry = pending.get(message.id);
    if (entry === undefined) return;
    pending.delete(message.id);
    if (message.error !== undefined) entry.fail(new Error(message.error.message));
    else entry.done(message.result);
  });

  return {
    ready,
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

async function main() {
  mkdirSync(OUT_DIR, { recursive: true });

  const chrome = spawn(
    'google-chrome',
    [
      '--headless=new',
      `--remote-debugging-port=${PORT}`,
      '--remote-allow-origins=*',
      '--user-data-dir=/tmp/hfs-shot-profile',
      '--disable-gpu',
      '--use-gl=angle',
      '--use-angle=swiftshader',
      // CONVENTIONS section 6a: the GPU 2D canvas returns transparent pixels on
      // the Vulkan ANGLE path, and every gauge face is a 2D canvas.
      '--disable-accelerated-2d-canvas',
      '--hide-scrollbars',
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

  /** Runs an expression in the page and returns its value. */
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

  /** Sets the viewport. The renderer reads the canvas size back from CSS. */
  async function setViewport(width, height) {
    await cdp.send('Emulation.setDeviceMetricsOverride', {
      width,
      height,
      deviceScaleFactor: 1,
      mobile: false,
    });
  }

  /**
   * Writes one picture.
   *
   * A picture of the WORLD goes out as a JPEG. The sky and the ground are long
   * smooth gradients, and a PNG of one of them runs to 450 kB. A palette of 256
   * colors brings that down but it bands every gradient, which reads as a fault
   * in the renderer that is not there.
   *
   * A picture of a PANEL goes out as a PNG. It is mostly flat color and small
   * monospace text, so a PNG is both smaller AND sharp, and JPEG rings around
   * the letters.
   */
  async function shoot(name, kind = 'world') {
    const png = kind === 'panel';
    const shot = await cdp.send('Page.captureScreenshot', {
      format: png ? 'png' : 'jpeg',
      ...(png ? {} : { quality: 90 }),
    });
    const file = resolve(OUT_DIR, `${name}.${png ? 'png' : 'jpg'}`);
    writeFileSync(file, Buffer.from(shot.data, 'base64'));
    console.log(`wrote ${file}`);
  }

  /** Presses one key by its physical code, the way a pilot presses it. */
  async function pressKey(code, key) {
    await cdp.send('Input.dispatchKeyEvent', { type: 'keyDown', code, key });
    await cdp.send('Input.dispatchKeyEvent', { type: 'keyUp', code, key });
  }

  /**
   * Holds the left mouse button and drags, the way a pilot looks around.
   *
   * src/render/cameras.ts owns the mouse, because a mouse reports movement and
   * a binding needs a position. The head returns to center about half a second
   * after the button comes up, so the caller holds the button over the shot and
   * releases it after.
   */
  async function lookDrag(fromX, fromY, byX, byY) {
    await cdp.send('Input.dispatchMouseEvent', {
      type: 'mousePressed',
      x: fromX,
      y: fromY,
      button: 'left',
      buttons: 1,
      clickCount: 1,
    });
    const steps = 12;
    for (let i = 1; i <= steps; i++) {
      await cdp.send('Input.dispatchMouseEvent', {
        type: 'mouseMoved',
        x: fromX + (byX * i) / steps,
        y: fromY + (byY * i) / steps,
        button: 'left',
        buttons: 1,
      });
      await wait(16);
    }
  }

  async function releaseMouse(x, y) {
    await cdp.send('Input.dispatchMouseEvent', {
      type: 'mouseReleased',
      x,
      y,
      button: 'left',
      clickCount: 1,
    });
  }

  /** Clicks the middle of the element that the selector names. */
  async function clickElement(selector) {
    const box = await evaluate(`(() => {
      const el = document.querySelector(${JSON.stringify(selector)});
      if (el === null) return null;
      const r = el.getBoundingClientRect();
      return { x: r.left + r.width / 2, y: r.top + r.height / 2 };
    })()`);
    if (box === null) throw new Error(`No element matches ${selector}.`);
    for (const type of ['mousePressed', 'mouseReleased']) {
      await cdp.send('Input.dispatchMouseEvent', {
        type,
        x: box.x,
        y: box.y,
        button: 'left',
        clickCount: 1,
      });
    }
  }

  // Wait for the simulator to build itself and draw a settled frame.
  await setViewport(1280, 720);
  for (let attempt = 0; attempt < 80; attempt++) {
    if ((await evaluate('typeof window.sim === "object" && window.sim !== null')) === true) break;
    await wait(500);
  }
  await wait(SETTLE_MS);

  console.log(`backend: ${await evaluate('window.sim.bundle.backend')}`);

  // --- 1. The aircraft on the runway, in the chase view ------------------
  await shoot('chase');

  // --- 2. The controls menu ----------------------------------------------
  // The CLICK is the test. A screenshot of `menu.visible = true` would prove
  // the panel draws and would prove nothing about the button.
  await clickElement('.hfs-menu-open');
  await wait(SHORT_MS);
  if ((await evaluate('window.sim.menu.visible')) !== true) {
    throw new Error('The CONTROLS button did not open the menu.');
  }
  await shoot('controls-menu', 'panel');

  // The H key must shut the same panel. That is the other half of the binding.
  await pressKey('KeyH', 'h');
  await wait(SHORT_MS);
  if ((await evaluate('window.sim.menu.visible')) !== false) {
    throw new Error('The H key did not shut the menu.');
  }

  // --- 3. The aircraft in the air -----------------------------------------
  //
  // THE AIRCRAFT IS NOT TRIMMED, so it cannot be placed and left. The elevator
  // reads the pilot command, and there is no pilot, so the aircraft holds the
  // angle of attack its own balance gives and climbs away from any speed that
  // does not match. 190 m/s reached 3.5 g in four seconds and filled the
  // picture with sky.
  //
  // Two answers. 120 m/s is close to the speed the balance holds by itself, so
  // the drift is mild. And `level` runs again a moment BEFORE each shot, so
  // only the last two seconds of drift reach the picture.
  const level = `(() => {
    const body = window.sim.aircraft.state.body;
    body.position.set(1200, 0, -1400);
    body.velocity.set(120, 0, 0);
    body.orientation.identity();
    body.angularVelocity.set(0, 0, 0);
  })()`;

  // The panel switch clears the glass. It is the U key now, because the menu
  // took H. The dials of the cockpit sit under the debug overlay without it.
  await pressKey('KeyU', 'u');
  await wait(SHORT_MS);
  if ((await evaluate('window.sim.hud.visible')) !== false) {
    throw new Error('The U key did not hide the panels.');
  }

  // The ORBIT view holds the aircraft in the middle of the picture, whatever
  // the aircraft does. The flyby view does not: it stands at a fixed point and
  // lets the aircraft pass, so an aircraft that is placed by hand lands a
  // kilometer away from it and the shot comes back empty.
  // G retracts the gear. The aircraft spawned on the runway, so the legs are
  // down and locked, and an aircraft in the air with its gear down reads as a
  // mistake. The systems model runs the legs through their travel time, so the
  // shot waits for that.
  await evaluate(level);
  await pressKey('KeyG', 'g');
  await evaluate(`(() => {
    window.sim.rig.mode = 'orbit';
    window.sim.rig.orbitDistance = 20;
    window.sim.rig.snap();
  })()`);
  await wait(SETTLE_MS);
  await evaluate(level);
  // The orbit view keeps the mouse offset after the button comes up, so the
  // drag turns the camera to a three quarter view and leaves it there. Dead
  // astern hides the whole plan form of the wing.
  await lookDrag(640, 360, 260, -40);
  await releaseMouse(900, 320);
  await wait(SHORT_MS);
  await evaluate(level);
  await wait(SHORT_MS);
  await shoot('flight');

  // --- 4. The cockpit ------------------------------------------------------
  // The interior builds itself on the FIRST frame in this view, with its
  // fifteen live dials. See section 6 of src/render/models/cockpit.ts. The
  // taller viewport shows more of the panel, because the field of view is
  // vertical.
  await setViewport(1280, 800);
  await evaluate(level);
  await evaluate("window.sim.rig.mode = 'cockpit'; window.sim.rig.snap();");
  await wait(SETTLE_MS);
  await evaluate(level);
  // The eye point of the pilot sits above the panel, so the dials fall under
  // the bottom of the picture from the neutral head position. The drag lowers
  // the head, which is what a real pilot does to read them.
  await lookDrag(640, 400, 0, 150);
  await wait(SHORT_MS);
  await shoot('cockpit');
  await releaseMouse(640, 550);

  // --- 5. The phone, with the on screen pad -------------------------------
  // A second load carries the query string, because the pad reads it at start.
  await setViewport(844, 390);
  await cdp.send('Page.navigate', { url: `${PAGE_URL}?touch=1` });
  await wait(1000);
  for (let attempt = 0; attempt < 80; attempt++) {
    if ((await evaluate('typeof window.sim === "object" && window.sim !== null')) === true) break;
    await wait(500);
  }
  await wait(SETTLE_MS);
  if ((await evaluate('window.sim.touch.visible')) !== true) {
    throw new Error('The pad did not appear with ?touch=1.');
  }
  await shoot('touch-controls');

  cdp.close();
  chrome.kill();
}

main().catch((error) => {
  console.error(String(error));
  process.exit(1);
});
