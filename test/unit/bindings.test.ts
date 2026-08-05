import { describe, expect, it } from 'vitest';
import type { AxisName, ButtonName, GamepadReader } from '@/input/gamepad';
import type { KeyboardReader } from '@/input/keyboard';
import type { ControlInput, InputSystem } from '@/input/bindings';
import {
  FULL_AUTHORITY_PRESSURE,
  MIN_CONTROL_AUTHORITY,
  controlAuthority,
  createInputSystem,
  DEFAULT_BINDINGS,
} from '@/input/bindings';
import type { TouchAxisName, TouchButtonName, TouchReader } from '@/input/touch';
import {
  TOUCH_AXIS_NAMES,
  TOUCH_BUTTON_NAMES,
  TOUCH_DRAWN_AXES,
  TOUCH_DRAWN_BUTTONS,
} from '@/input/touch';

/**
 * A fake gamepad and a fake keyboard. The test writes the hardware by hand, so
 * the binding layer runs in Node with no browser and with no controller.
 *
 * Both fakes hold the edge the same way the real readers hold it. `poll` moves
 * the current set into the previous set, so `pressed` is true on one poll only.
 */

interface FakeGamepad extends GamepadReader {
  setAxis(name: AxisName, value: number): void;
  setButton(name: ButtonName, down: boolean): void;
}

function createFakeGamepad(): FakeGamepad {
  const axes = new Map<AxisName, number>();
  const hardware = new Set<ButtonName>();
  let now = new Set<ButtonName>();
  let before = new Set<ButtonName>();

  return {
    connected: true,
    id: 'fake pad',
    poll(): void {
      before = now;
      now = new Set(hardware);
    },
    axis: (name) => axes.get(name) ?? 0,
    rawAxis: (name) => axes.get(name) ?? 0,
    held: (name) => now.has(name),
    pressed: (name) => now.has(name) && !before.has(name),
    released: (name) => !now.has(name) && before.has(name),
    value: (name) => (now.has(name) ? 1 : 0),
    dispose(): void {
      hardware.clear();
      now.clear();
      before.clear();
    },
    setAxis(name, value): void {
      axes.set(name, value);
    },
    setButton(name, down): void {
      if (down) hardware.add(name);
      else hardware.delete(name);
    },
  };
}

interface FakeKeyboard extends KeyboardReader {
  setKey(code: string, down: boolean): void;
}

function createFakeKeyboard(): FakeKeyboard {
  const hardware = new Set<string>();
  let now = new Set<string>();
  let before = new Set<string>();

  return {
    poll(): void {
      before = now;
      now = new Set(hardware);
    },
    held: (code) => now.has(code),
    pressed: (code) => now.has(code) && !before.has(code),
    released: (code) => !now.has(code) && before.has(code),
    dispose(): void {
      hardware.clear();
      now.clear();
      before.clear();
    },
    setKey(code, down): void {
      if (down) hardware.add(code);
      else hardware.delete(code);
    },
  };
}

/**
 * A fake on screen pad. It holds the same edge shape the two fakes above hold,
 * so the touch path runs in Node with no DOM and no finger.
 */
interface FakeTouch extends TouchReader {
  setAxis(name: TouchAxisName, value: number): void;
  setButton(name: TouchButtonName, down: boolean): void;
}

function createFakeTouch(): FakeTouch {
  const axes = new Map<TouchAxisName, number>();
  const hardware = new Set<TouchButtonName>();
  let now = new Set<TouchButtonName>();
  let before = new Set<TouchButtonName>();

  return {
    active: true,
    visible: true,
    poll(): void {
      before = now;
      now = new Set(hardware);
    },
    axis: (name) => axes.get(name) ?? 0,
    held: (name) => now.has(name),
    pressed: (name) => now.has(name) && !before.has(name),
    released: (name) => !now.has(name) && before.has(name),
    setThrottle(): void {
      // The fake draws no bar.
    },
    dispose(): void {
      hardware.clear();
      now.clear();
      before.clear();
      axes.clear();
    },
    setAxis(name, value): void {
      axes.set(name, value);
    },
    setButton(name, down): void {
      if (down) hardware.add(name);
      else hardware.delete(name);
    },
  };
}

interface Rig {
  pad: FakeGamepad;
  keys: FakeKeyboard;
  touch: FakeTouch;
  input: InputSystem;
  /** Reports the wheels on the ground. The test writes it. */
  onGround: { value: boolean };
  /** Polls one frame of the given length, 1/60 s by default. */
  frame(dt?: number): ControlInput;
}

const FRAME = 1 / 60;

function createRig(): Rig {
  const pad = createFakeGamepad();
  const keys = createFakeKeyboard();
  const touch = createFakeTouch();
  const onGround = { value: false };
  const input = createInputSystem({
    gamepad: pad,
    keyboard: keys,
    touch,
    groundContact: () => onGround.value,
  });
  return {
    pad,
    keys,
    touch,
    input,
    onGround,
    frame(dt = FRAME): ControlInput {
      input.poll(dt);
      return input.state;
    },
  };
}

describe('the pitch axis sign', () => {
  it('a stick pulled back gives a positive pitch command, which is nose up', () => {
    const rig = createRig();
    // src/input/gamepad.ts keeps the Gamepad API sign. A stick pulled back
    // reports a positive Y. Nose up is a positive pitch command.
    rig.pad.setAxis('leftY', 1);
    expect(rig.frame().pitch).toBeGreaterThan(0);
    expect(rig.frame().pitch).toBeCloseTo(1, 9);
  });

  it('a stick pushed forward gives a negative pitch command, which is nose down', () => {
    const rig = createRig();
    rig.pad.setAxis('leftY', -1);
    expect(rig.frame().pitch).toBeCloseTo(-1, 9);
  });

  it('the S key gives nose up and the W key gives nose down', () => {
    const rig = createRig();
    rig.keys.setKey('KeyS', true);
    expect(rig.frame().pitch).toBeCloseTo(1, 9);
    rig.keys.setKey('KeyS', false);
    rig.keys.setKey('KeyW', true);
    expect(rig.frame().pitch).toBeCloseTo(-1, 9);
  });
});

describe('the roll axis and the yaw axis', () => {
  it('a stick to the right gives a positive roll command', () => {
    const rig = createRig();
    rig.pad.setAxis('leftX', 1);
    expect(rig.frame().roll).toBeCloseTo(1, 9);
    rig.pad.setAxis('leftX', -1);
    expect(rig.frame().roll).toBeCloseTo(-1, 9);
  });

  it('the D key rolls right and the A key rolls left', () => {
    const rig = createRig();
    rig.keys.setKey('KeyD', true);
    expect(rig.frame().roll).toBeCloseTo(1, 9);
    rig.keys.setKey('KeyD', false);
    rig.keys.setKey('KeyA', true);
    expect(rig.frame().roll).toBeCloseTo(-1, 9);
  });

  it('the right trigger gives a positive yaw and the left trigger gives a negative yaw', () => {
    const rig = createRig();
    rig.pad.setAxis('rightTrigger', 1);
    expect(rig.frame().yaw).toBeCloseTo(1, 9);

    rig.pad.setAxis('rightTrigger', 0);
    rig.pad.setAxis('leftTrigger', 1);
    expect(rig.frame().yaw).toBeCloseTo(-1, 9);
  });

  it('two equal triggers give no yaw, because the rudder is their difference', () => {
    const rig = createRig();
    rig.pad.setAxis('leftTrigger', 0.7);
    rig.pad.setAxis('rightTrigger', 0.7);
    expect(rig.frame().yaw).toBeCloseTo(0, 9);
  });

  it('the E key yaws right and the Q key yaws left', () => {
    const rig = createRig();
    rig.keys.setKey('KeyE', true);
    expect(rig.frame().yaw).toBeCloseTo(1, 9);
    rig.keys.setKey('KeyE', false);
    rig.keys.setKey('KeyQ', true);
    expect(rig.frame().yaw).toBeCloseTo(-1, 9);
  });
});

describe('the throttle is a lever with a rate, not a switch', () => {
  it('one 16 ms frame of D-pad up moves the throttle by about 0.008, not to full', () => {
    const rig = createRig();
    rig.pad.setButton('dpadUp', true);
    const state = rig.frame(0.016);
    expect(state.throttle).toBeCloseTo(0.008, 9);
    expect(state.throttle).toBeLessThan(0.01);
  });

  it('two seconds of D-pad up moves the throttle over the full range', () => {
    const rig = createRig();
    rig.pad.setButton('dpadUp', true);
    let state = rig.input.state;
    // 120 frames of 1/60 s is two seconds of simulated time.
    for (let i = 0; i < 120; i += 1) state = rig.frame();
    expect(state.throttle).toBeCloseTo(1, 6);
  });

  it('one second of D-pad up reaches half of the range', () => {
    const rig = createRig();
    rig.pad.setButton('dpadUp', true);
    let state = rig.input.state;
    for (let i = 0; i < 60; i += 1) state = rig.frame();
    expect(state.throttle).toBeCloseTo(0.5, 6);
  });

  it('the throttle clamps at 1 and at 0', () => {
    const rig = createRig();
    rig.pad.setButton('dpadUp', true);
    for (let i = 0; i < 300; i += 1) rig.frame();
    expect(rig.input.state.throttle).toBe(1);

    rig.pad.setButton('dpadUp', false);
    rig.pad.setButton('dpadDown', true);
    for (let i = 0; i < 300; i += 1) rig.frame();
    expect(rig.input.state.throttle).toBe(0);
  });

  it('the Page Up key and the Page Down key move the same lever', () => {
    const rig = createRig();
    rig.keys.setKey('PageUp', true);
    for (let i = 0; i < 60; i += 1) rig.frame();
    expect(rig.input.state.throttle).toBeCloseTo(0.5, 6);

    rig.keys.setKey('PageUp', false);
    rig.keys.setKey('PageDown', true);
    for (let i = 0; i < 30; i += 1) rig.frame();
    expect(rig.input.state.throttle).toBeCloseTo(0.25, 6);
  });

  it('the shifted equal key raises the lever at the same rate', () => {
    const rig = createRig();
    rig.keys.setKey('ShiftLeft', true);
    rig.keys.setKey('Equal', true);
    for (let i = 0; i < 60; i += 1) rig.frame();
    expect(rig.input.state.throttle).toBeCloseTo(0.5, 6);
  });
});

describe('edge actions and held actions', () => {
  it('the gear button reports true on exactly one poll while the pilot holds it', () => {
    const rig = createRig();
    rig.pad.setButton('a', true);
    expect(rig.frame().toggleGear).toBe(true);
    expect(rig.frame().toggleGear).toBe(false);
    expect(rig.frame().toggleGear).toBe(false);

    rig.pad.setButton('a', false);
    expect(rig.frame().toggleGear).toBe(false);
    rig.pad.setButton('a', true);
    expect(rig.frame().toggleGear).toBe(true);
    expect(rig.frame().toggleGear).toBe(false);
  });

  it('the G key reports true on exactly one poll while the pilot holds it', () => {
    const rig = createRig();
    rig.keys.setKey('KeyG', true);
    expect(rig.frame().toggleGear).toBe(true);
    expect(rig.frame().toggleGear).toBe(false);
  });

  it('the engine start button stays true for every poll, because it is a hold', () => {
    const rig = createRig();
    rig.pad.setButton('leftBumper', true);
    expect(rig.frame().startEngines).toBe(true);
    expect(rig.frame().startEngines).toBe(true);
    rig.pad.setButton('leftBumper', false);
    expect(rig.frame().startEngines).toBe(false);
  });

  it('the cannon stays true for every poll, because it is a hold', () => {
    const rig = createRig();
    rig.keys.setKey('Space', true);
    expect(rig.frame().fireCannon).toBe(true);
    expect(rig.frame().fireCannon).toBe(true);
  });

  it('the F key lowers the flaps and the shifted F key raises them', () => {
    const rig = createRig();
    rig.keys.setKey('KeyF', true);
    let state = rig.frame();
    expect(state.toggleFlapsDown).toBe(true);
    expect(state.toggleFlapsUp).toBe(false);

    rig.keys.setKey('KeyF', false);
    rig.frame();
    rig.keys.setKey('ShiftLeft', true);
    rig.keys.setKey('KeyF', true);
    state = rig.frame();
    expect(state.toggleFlapsUp).toBe(true);
    expect(state.toggleFlapsDown).toBe(false);
  });

  it('the D-pad carries the two flap steps', () => {
    const rig = createRig();
    rig.pad.setButton('dpadRight', true);
    expect(rig.frame().toggleFlapsDown).toBe(true);
    rig.pad.setButton('dpadRight', false);
    rig.pad.setButton('dpadLeft', true);
    expect(rig.frame().toggleFlapsUp).toBe(true);
  });

  it('the trim keys are holds, so the pilot can run the trim wheel', () => {
    const rig = createRig();
    rig.keys.setKey('BracketRight', true);
    expect(rig.frame().trimUp).toBe(true);
    expect(rig.frame().trimUp).toBe(true);
    expect(rig.frame().trimDown).toBe(false);
  });
});

describe('the wheel brakes', () => {
  it('the B button holds both brakes on, in the air and on the ground', () => {
    const rig = createRig();
    rig.pad.setButton('b', true);
    const state = rig.frame();
    expect(state.brakeLeft).toBe(1);
    expect(state.brakeRight).toBe(1);
  });

  it('a trigger drives one brake while the aircraft taxis at idle', () => {
    const rig = createRig();
    rig.onGround.value = true;
    rig.pad.setAxis('leftTrigger', 1);
    const state = rig.frame();
    expect(state.brakeLeft).toBe(1);
    expect(state.brakeRight).toBe(0);
    // The trigger still drives the rudder. On the ground at idle the rudder
    // has almost no air over it, so the two jobs do not fight.
    expect(state.yaw).toBeCloseTo(-1, 9);
  });

  it('a trigger drives no brake in the air, so it cannot fight the rudder', () => {
    const rig = createRig();
    rig.onGround.value = false;
    rig.pad.setAxis('rightTrigger', 1);
    const state = rig.frame();
    expect(state.brakeRight).toBe(0);
    expect(state.yaw).toBeCloseTo(1, 9);
  });

  it('a trigger drives no brake on the ground above idle power', () => {
    const rig = createRig();
    rig.onGround.value = true;
    rig.pad.setAxis('rightTrigger', 1);
    rig.pad.setButton('dpadUp', true);
    // One second of D-pad up carries the lever well past idle.
    for (let i = 0; i < 60; i += 1) rig.frame();
    expect(rig.input.state.throttle).toBeGreaterThan(0.05);
    expect(rig.input.state.brakeRight).toBe(0);
  });

  it('the Z key brakes the left wheel alone and the C key brakes the right', () => {
    const rig = createRig();
    rig.keys.setKey('KeyZ', true);
    let state = rig.frame();
    expect(state.brakeLeft).toBe(1);
    expect(state.brakeRight).toBe(0);

    rig.keys.setKey('KeyZ', false);
    rig.keys.setKey('KeyC', true);
    state = rig.frame();
    expect(state.brakeLeft).toBe(0);
    expect(state.brakeRight).toBe(1);
  });

  it('the two brake keys work in the air and at full power, unlike the triggers', () => {
    // A key drives nothing else, so no taxi rule gates it. The pilot needs the
    // differential brake through the whole landing roll, not only at idle.
    const rig = createRig();
    rig.onGround.value = false;
    rig.keys.setKey('KeyZ', true);
    expect(rig.frame().brakeLeft).toBe(1);
  });

  it('the B key still brakes both wheels together', () => {
    const rig = createRig();
    rig.keys.setKey('KeyB', true);
    const state = rig.frame();
    expect(state.brakeLeft).toBe(1);
    expect(state.brakeRight).toBe(1);
  });
});

describe('the control authority', () => {
  it('a full command reaches the full travel up to 10 kPa', () => {
    expect(controlAuthority(0)).toBe(1);
    expect(controlAuthority(FULL_AUTHORITY_PRESSURE)).toBe(1);
  });

  it('the travel falls as one over the dynamic pressure, as a stick force does', () => {
    expect(controlAuthority(2 * FULL_AUTHORITY_PRESSURE)).toBeCloseTo(0.5, 9);
    expect(controlAuthority(4 * FULL_AUTHORITY_PRESSURE)).toBeCloseTo(0.25, 9);
  });

  it('the travel never falls under the floor, so a dive is always recoverable', () => {
    expect(controlAuthority(1e9)).toBe(MIN_CONTROL_AUTHORITY);
  });

  it('the three flight axes lose travel at speed and the look axes do not', () => {
    // 40 kPa is 900 km/h of equivalent airspeed. See FULL_AUTHORITY_PRESSURE.
    const pad = createFakeGamepad();
    const keys = createFakeKeyboard();
    const input = createInputSystem({
      gamepad: pad,
      keyboard: keys,
      dynamicPressure: () => 40000,
    });
    pad.setAxis('leftY', 1);
    pad.setAxis('leftX', 1);
    pad.setAxis('rightTrigger', 1);
    pad.setAxis('rightY', -1);
    input.poll(FRAME);
    expect(input.state.pitch).toBeCloseTo(0.25, 9);
    expect(input.state.roll).toBeCloseTo(0.25, 9);
    expect(input.state.yaw).toBeCloseTo(0.25, 9);
    expect(input.state.lookPitch).toBeCloseTo(1, 9);
  });

  it('a parked aircraft keeps every degree of every surface', () => {
    const rig = createRig();
    rig.pad.setAxis('leftY', 1);
    expect(rig.frame().pitch).toBeCloseTo(1, 9);
  });
});

describe('the active device', () => {
  it('the active device follows the device that moved last', () => {
    const rig = createRig();
    rig.frame();
    expect(rig.input.activeDevice).toBe('keyboard');

    rig.pad.setAxis('leftX', 1);
    rig.frame();
    expect(rig.input.activeDevice).toBe('gamepad');

    // A stick back at rest keeps the pad active until a key moves.
    rig.pad.setAxis('leftX', 0);
    rig.frame();
    expect(rig.input.activeDevice).toBe('gamepad');

    rig.keys.setKey('KeyA', true);
    rig.frame();
    expect(rig.input.activeDevice).toBe('keyboard');

    rig.keys.setKey('KeyA', false);
    rig.pad.setButton('a', true);
    rig.frame();
    expect(rig.input.activeDevice).toBe('gamepad');
  });
});

describe('the binding table', () => {
  it('the table carries every action of the control input', () => {
    const bound = new Set<string>();
    for (const binding of DEFAULT_BINDINGS) bound.add(binding.action);

    const rig = createRig();
    for (const action of Object.keys(rig.input.state)) {
      expect(bound.has(action)).toBe(true);
    }
  });

  it('every binding names a source', () => {
    for (const binding of DEFAULT_BINDINGS) {
      const named =
        binding.gamepad !== undefined || binding.keys !== undefined || binding.touch !== undefined;
      expect(named).toBe(true);
    }
  });
});

describe('the on screen pad', () => {
  it('the stick drives roll and pitch, and a thumb down the screen raises the nose', () => {
    const rig = createRig();
    rig.touch.setAxis('stickX', 1);
    // A finger below the center of the stick reports a positive Y, which pulls
    // the stick back. See the module comment of src/input/touch.ts.
    rig.touch.setAxis('stickY', 1);
    const state = rig.frame();
    expect(state.roll).toBeCloseTo(1, 9);
    expect(state.pitch).toBeCloseTo(1, 9);
  });

  it('the rudder bar drives yaw both ways', () => {
    const rig = createRig();
    rig.touch.setAxis('rudder', -1);
    expect(rig.frame().yaw).toBeCloseTo(-1, 9);
    rig.touch.setAxis('rudder', 0.5);
    expect(rig.frame().yaw).toBeCloseTo(0.5, 9);
  });

  it('the throttle rocker moves the lever at the rate of the other devices', () => {
    const rig = createRig();
    rig.touch.setButton('throttleUp', true);
    // One second of the upper half opens the lever by one sweep time.
    for (let i = 0; i < 60; i++) rig.frame();
    const opened = rig.input.state.throttle;
    expect(opened).toBeGreaterThan(0.1);

    rig.touch.setButton('throttleUp', false);
    rig.touch.setButton('throttleDown', true);
    for (let i = 0; i < 60; i++) rig.frame();
    expect(rig.input.state.throttle).toBeLessThan(opened);
  });

  it('a button that carries an edge action fires one time for each press', () => {
    const rig = createRig();
    rig.touch.setButton('gear', true);
    expect(rig.frame().toggleGear).toBe(true);
    // The finger stays down. The action must not fire again.
    expect(rig.frame().toggleGear).toBe(false);
    rig.touch.setButton('gear', false);
    rig.frame();
    rig.touch.setButton('gear', true);
    expect(rig.frame().toggleGear).toBe(true);
  });

  it('a button that carries a held action stays true while the finger is down', () => {
    const rig = createRig();
    rig.touch.setButton('fire', true);
    expect(rig.frame().fireCannon).toBe(true);
    expect(rig.frame().fireCannon).toBe(true);
    rig.touch.setButton('fire', false);
    expect(rig.frame().fireCannon).toBe(false);
  });

  it('the BRAKE button drives both wheels at any speed and at any throttle', () => {
    const rig = createRig();
    rig.onGround.value = false;
    rig.touch.setButton('brake', true);
    const state = rig.frame();
    expect(state.brakeLeft).toBeCloseTo(1, 9);
    expect(state.brakeRight).toBeCloseTo(1, 9);
  });

  it('a finger on the pad claims the active device', () => {
    const rig = createRig();
    rig.frame();
    expect(rig.input.activeDevice).toBe('keyboard');
    rig.touch.setAxis('stickX', 1);
    rig.frame();
    expect(rig.input.activeDevice).toBe('touch');
    // A hand back on the keys wins over a finger that rests on the glass.
    rig.keys.setKey('KeyA', true);
    rig.frame();
    expect(rig.input.activeDevice).toBe('keyboard');
  });

  it('the pad draws every control it names, and the table binds every one', () => {
    // A name that the pad never draws is a control no pilot can reach. A name
    // the table never binds is a control that does nothing.
    expect([...TOUCH_DRAWN_BUTTONS].sort()).toEqual([...TOUCH_BUTTON_NAMES].sort());
    expect([...TOUCH_DRAWN_AXES].sort()).toEqual([...TOUCH_AXIS_NAMES].sort());

    const bound = new Set<string>();
    for (const binding of DEFAULT_BINDINGS) {
      if (binding.touch !== undefined) bound.add(binding.touch);
    }
    for (const name of TOUCH_AXIS_NAMES) expect(bound.has(name)).toBe(true);
    for (const name of TOUCH_BUTTON_NAMES) expect(bound.has(name)).toBe(true);
  });

  it('an unknown touch control in a binding fails at build time', () => {
    expect(() =>
      createInputSystem({
        gamepad: createFakeGamepad(),
        keyboard: createFakeKeyboard(),
        bindings: [{ action: 'roll', kind: 'axis', touch: 'nose wheel' }],
      }),
    ).toThrow(/Unknown touch control/);
  });

  it('the system runs with no pad at all', () => {
    // A desktop passes no pad. Every touch read must then be a quiet zero.
    const input = createInputSystem({
      gamepad: createFakeGamepad(),
      keyboard: createFakeKeyboard(),
    });
    input.poll(FRAME);
    expect(input.state.roll).toBe(0);
    expect(input.state.toggleGear).toBe(false);
    input.dispose();
  });
});
