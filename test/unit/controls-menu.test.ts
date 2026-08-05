import { describe, expect, it } from 'vitest';

import type { Binding, ControlInput } from '@/input/bindings';
import { DEFAULT_BINDINGS } from '@/input/bindings';
import {
  ACTION_GROUPS,
  ACTION_INFO,
  buildRows,
  gamepadLabel,
  keyLabel,
  touchLabel,
} from '@/ui/controls-menu';

/**
 * The menu builds its rows from the binding table. These tests drive the pure
 * part of that build, so the whole list runs in Node with no DOM.
 */

describe('keyLabel', () => {
  it('a letter key prints the letter alone', () => {
    expect(keyLabel('KeyW')).toBe('W');
    expect(keyLabel('KeyG')).toBe('G');
  });

  it('a digit key prints the digit alone', () => {
    expect(keyLabel('Digit1')).toBe('1');
  });

  it('a shifted code prints the modifier in front', () => {
    expect(keyLabel('Shift+KeyF')).toBe('Shift F');
    expect(keyLabel('Shift+Equal')).toBe('Shift =');
  });

  it('a punctuation key prints the mark and not the name', () => {
    expect(keyLabel('BracketLeft')).toBe('[');
    expect(keyLabel('BracketRight')).toBe(']');
    expect(keyLabel('Minus')).toBe('-');
  });

  it('a two word code takes a space, and a function key does not', () => {
    expect(keyLabel('PageUp')).toBe('Page Up');
    expect(keyLabel('CapsLock')).toBe('Caps Lock');
    expect(keyLabel('F2')).toBe('F2');
    expect(keyLabel('F12')).toBe('F12');
  });
});

describe('the device labels', () => {
  it('every gamepad control of the table has a word', () => {
    for (const binding of DEFAULT_BINDINGS) {
      if (binding.gamepad === undefined) continue;
      // A name with no entry falls back to the raw name, so the two differ
      // exactly when the word exists.
      expect(gamepadLabel(binding.gamepad)).not.toBe(binding.gamepad);
    }
  });

  it('every touch control of the table has a word', () => {
    for (const binding of DEFAULT_BINDINGS) {
      if (binding.touch === undefined) continue;
      expect(touchLabel(binding.touch)).not.toBe(binding.touch);
    }
  });
});

describe('buildRows', () => {
  it('it builds one row for each action of the control input', () => {
    const rows = buildRows();
    const actions = new Set(rows.map((row) => row.action));
    for (const key of Object.keys(ACTION_INFO) as (keyof ControlInput)[]) {
      expect(actions.has(key)).toBe(true);
    }
    expect(rows.length).toBe(Object.keys(ACTION_INFO).length);
  });

  it('the rows arrive in group order', () => {
    const rows = buildRows();
    const seen: string[] = [];
    for (const row of rows) {
      if (seen[seen.length - 1] !== row.group) seen.push(row.group);
    }
    expect(seen).toEqual([...ACTION_GROUPS]);
  });

  it('a key pair prints in the order of the words of the label', () => {
    const rows = buildRows();
    const pitch = rows.find((row) => row.action === 'pitch');
    expect(pitch?.label).toBe('Pitch (nose down, nose up)');
    // The table holds ['KeyW', 'KeyS'] as [negative, positive], and a negative
    // pitch is nose down.
    expect(pitch?.keyboard).toEqual(['W', 'S']);
  });

  it('a control with a negative scale prints before the one with a positive scale', () => {
    const rows = buildRows();
    const yaw = rows.find((row) => row.action === 'yaw');
    // The left trigger carries a scale of -1 and it is rudder LEFT, which is
    // the first word of the label.
    expect(yaw?.label).toBe('Rudder (left, right)');
    expect(yaw?.gamepad).toEqual(['Left trigger', 'Right trigger']);

    const throttle = rows.find((row) => row.action === 'throttle');
    expect(throttle?.label).toBe('Throttle (less, more)');
    expect(throttle?.gamepad).toEqual(['D-pad down', 'D-pad up']);
    expect(throttle?.touch).toEqual(['THR -', 'THR +']);
  });

  it('a repeated control prints one time', () => {
    const bindings: Binding[] = [
      { action: 'toggleGear', kind: 'button', keys: ['KeyG'] },
      { action: 'toggleGear', kind: 'button', keys: ['KeyG'] },
    ];
    const row = buildRows(bindings).find((r) => r.action === 'toggleGear');
    expect(row?.keyboard).toEqual(['G']);
  });

  it('an action with no binding still gets a row and prints nothing for it', () => {
    const row = buildRows([]).find((r) => r.action === 'roll');
    expect(row).toBeDefined();
    expect(row?.keyboard).toEqual([]);
    expect(row?.gamepad).toEqual([]);
    expect(row?.touch).toEqual([]);
  });

  it('it reports which actions fire one time for each press', () => {
    const rows = buildRows();
    expect(rows.find((row) => row.action === 'toggleGear')?.edge).toBe(true);
    expect(rows.find((row) => row.action === 'fireCannon')?.edge).toBe(false);
  });

  it('every action of the binding table has a row', () => {
    // The two lists come from different files. This test is what stops a new
    // action from reaching the code and never reaching the menu.
    const rows = buildRows();
    const actions = new Set(rows.map((row) => row.action as string));
    for (const binding of DEFAULT_BINDINGS) {
      expect(actions.has(binding.action)).toBe(true);
    }
  });
});
