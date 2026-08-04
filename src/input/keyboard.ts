/**
 * Keyboard reader.
 *
 * The module has the same shape as the gamepad reader. It reports the hardware
 * and it finds the key edges. It does not decide what a key means. The binding
 * table in src/input/bindings.ts decides that.
 *
 *
 * WHY THE MODULE READS `code` AND NOT `key`
 *
 * `KeyboardEvent.code` names the physical key. `KeyboardEvent.key` names the
 * character that the layout puts on that key. On an AZERTY layout the key that
 * sits where QWERTY holds W reports `code` of `KeyW` and `key` of `z`. A map
 * built on `key` moves under the pilot when the layout changes. A map built on
 * `code` keeps the same three keys under the same three fingers.
 *
 *
 * EDGES
 *
 * The reader latches a key down event and a key up event. `poll` moves the
 * latch into the frame result and then clears the latch. A key that goes down
 * and comes up between two polls still reports one press and one release, so a
 * fast tap is never lost.
 *
 * The reader drops an event that carries `repeat`. The operating system sends
 * the repeat after the key is already down, so the repeat is not a new edge.
 *
 *
 * BLUR
 *
 * The browser stops the key up event when the window loses the focus. A pilot
 * who changes window in a turn comes back to an aircraft that still holds full
 * left roll. On blur the reader releases every key that is down.
 */

/** The two shift keys. The binding table reads them as a modifier. */
export const SHIFT_CODES: readonly string[] = ['ShiftLeft', 'ShiftRight'];

export interface KeyboardReader {
  held(code: string): boolean;
  /** True only on the poll after the key went down. */
  pressed(code: string): boolean;
  /** True only on the poll after the key came up. */
  released(code: string): boolean;
  /** Read the events of the last frame. Call it one time per frame. */
  poll(): void;
  dispose(): void;
}

/** The fields the reader needs from an event. */
interface KeyFields {
  code: string;
  repeat: boolean;
}

/**
 * Read the two fields, or return null when the event does not carry them.
 * The listener signature gives an `Event`, so the reader narrows it here.
 */
function readKeyFields(event: Event): KeyFields | null {
  const candidate = event as Partial<KeyboardEvent>;
  if (typeof candidate.code !== 'string' || candidate.code.length === 0) return null;
  return { code: candidate.code, repeat: candidate.repeat === true };
}

/**
 * Build the reader. The reader listens on `target`, which is the window by
 * default. A test passes its own `EventTarget` and needs no browser.
 */
export function createKeyboardReader(target?: EventTarget): KeyboardReader {
  const source: EventTarget | null =
    target ?? (typeof window !== 'undefined' ? window : null);

  /** Keys the hardware holds down now. The events keep this set current. */
  const down = new Set<string>();
  /** Keys that went down since the last poll. */
  const downLatch = new Set<string>();
  /** Keys that came up since the last poll. */
  const upLatch = new Set<string>();

  /** The three sets of the current frame. `poll` fills them. */
  const heldNow = new Set<string>();
  const pressedNow = new Set<string>();
  const releasedNow = new Set<string>();

  const onKeyDown = (event: Event): void => {
    const fields = readKeyFields(event);
    if (fields === null) return;
    // The operating system repeat is not a new edge. The key is already down.
    if (fields.repeat) return;
    down.add(fields.code);
    downLatch.add(fields.code);
  };

  const onKeyUp = (event: Event): void => {
    const fields = readKeyFields(event);
    if (fields === null) return;
    down.delete(fields.code);
    upLatch.add(fields.code);
  };

  const onBlur = (): void => {
    // Release every key. See the module comment.
    for (const code of down) upLatch.add(code);
    down.clear();
  };

  if (source !== null) {
    source.addEventListener('keydown', onKeyDown);
    source.addEventListener('keyup', onKeyUp);
    source.addEventListener('blur', onBlur);
  }

  // The window carries the blur event when the target is a document or an
  // element. Listen there as well, so the release always runs.
  const windowSource: EventTarget | null =
    typeof window !== 'undefined' && window !== source ? window : null;
  if (windowSource !== null) windowSource.addEventListener('blur', onBlur);

  return {
    poll(): void {
      pressedNow.clear();
      releasedNow.clear();
      heldNow.clear();

      for (const code of downLatch) pressedNow.add(code);
      for (const code of upLatch) releasedNow.add(code);
      for (const code of down) heldNow.add(code);

      downLatch.clear();
      upLatch.clear();
    },

    held(code: string): boolean {
      return heldNow.has(code);
    },

    pressed(code: string): boolean {
      return pressedNow.has(code);
    },

    released(code: string): boolean {
      return releasedNow.has(code);
    },

    dispose(): void {
      if (source !== null) {
        source.removeEventListener('keydown', onKeyDown);
        source.removeEventListener('keyup', onKeyUp);
        source.removeEventListener('blur', onBlur);
      }
      if (windowSource !== null) windowSource.removeEventListener('blur', onBlur);
      down.clear();
      downLatch.clear();
      upLatch.clear();
      heldNow.clear();
      pressedNow.clear();
      releasedNow.clear();
    },
  };
}
