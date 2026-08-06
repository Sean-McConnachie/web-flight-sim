/**
 * The SOUND button. It stands beside the CONTROLS button in every view.
 *
 * BEAD kz2.
 *
 *
 * 1. WHY A BUTTON AND NOT ONLY A KEY
 *
 * The M key mutes and unmutes. That is enough on a desktop and it is nothing at
 * all on a phone, which has no M key while the simulator is running.
 *
 * The button also solves a second problem that no key can solve. A browser
 * holds every `AudioContext` shut until the person acts, and a page that starts
 * in silence looks broken rather than muted. The button SAYS which of the two
 * it is, and a click on it is exactly the act the browser is waiting for.
 *
 *
 * 2. THE THREE STATES
 *
 *   ENABLE SOUND   The browser has not let the context start. A click starts
 *                  it. The border pulses, because this is the one state where
 *                  the pilot has to do something.
 *   SOUND ON       Everything runs.
 *   SOUND OFF      The pilot muted it, or the tab went to the background.
 *
 *
 * 3. WHY THE PANEL SWITCH DOES NOT HIDE IT
 *
 * U clears every overlay panel, and this button survives it, in the same way
 * the CONTROLS button does. A control that a person cannot find is worse than a
 * small button in a corner, and the first thing anybody looks for in a page
 * that makes a noise is the way to stop it.
 */

const STYLE_ID = 'hfs-sound-style';

/**
 * Width of the button, px.
 *
 * The CONTROLS button of src/ui/controls-menu.ts is the same width and stands
 * to the LEFT of this one. The top bar of the touch pad stands to the right of
 * both and indents itself past the pair. The style sheet of src/input/touch.ts
 * names this file for that reason.
 */
export const SOUND_BUTTON_WIDTH = 104;

const CSS = `
.hfs-sound-open {
  position: absolute;
  /* 104 px for the CONTROLS button and an 8 px gap. */
  left: calc(max(12px, env(safe-area-inset-left)) + 112px);
  top: max(12px, env(safe-area-inset-top));
  width: ${String(SOUND_BUTTON_WIDTH)}px;
  height: 34px;
  z-index: 8;
  pointer-events: auto;
  touch-action: manipulation;
  -webkit-tap-highlight-color: transparent;
  cursor: pointer;
  box-sizing: border-box;
  padding: 0;
  border-radius: 17px;
  background: rgba(6, 14, 10, 0.72);
  border: 1px solid rgba(120, 200, 150, 0.55);
  color: #d8ffe6;
  font: 600 11px/1 ui-monospace, 'DejaVu Sans Mono', monospace;
  letter-spacing: 0.08em;
  text-shadow: 0 1px 3px rgba(0, 0, 0, 0.9);
}
.hfs-sound-open:hover {
  background: rgba(120, 200, 150, 0.3);
  border-color: rgba(216, 255, 230, 0.8);
}
.hfs-sound-open.muted {
  color: #9aa8a0;
  border-color: rgba(120, 200, 150, 0.25);
}
/* The one state that asks the pilot for something. Section 2 says why. */
.hfs-sound-open.blocked {
  color: #ffe9a8;
  border-color: rgba(255, 200, 90, 0.85);
  animation: hfs-sound-pulse 1.8s ease-in-out infinite;
}
@keyframes hfs-sound-pulse {
  0%, 100% { box-shadow: 0 0 0 0 rgba(255, 200, 90, 0); }
  50% { box-shadow: 0 0 0 4px rgba(255, 200, 90, 0.22); }
}
@media (prefers-reduced-motion: reduce) {
  .hfs-sound-open.blocked { animation: none; }
}
`;

function ensureStyle(): void {
  if (document.getElementById(STYLE_ID) !== null) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = CSS;
  document.head.appendChild(style);
}

/** What the button reads and writes. It is the part of the bus it needs. */
export interface SoundButtonTarget {
  /** True while the browser has not let the audio context start. */
  readonly blocked: boolean;
  muted: boolean;
  /** Asks the browser to start the context. It only works inside a gesture. */
  unlock(): void;
  /** Adds a listener for a change of state. The return removes it. */
  onChange(fn: () => void): () => void;
}

export interface SoundButton {
  /** Does what a press of the button does. The M key calls it. */
  toggle(): void;
  /**
   * Takes the button off the screen.
   *
   * The controls menu sets it while its panel is open. The panel covers the
   * whole picture, so a button that stood in front of it would take a press
   * that belongs to the panel, and one that stood BEHIND it would be a dimmed
   * shape poking out from an edge. The CONTROLS button of
   * src/ui/controls-menu.ts leaves the screen for the same reason.
   *
   * The M key still mutes while the panel is open, and the panel carries the
   * master volume, so nothing is out of reach.
   */
  hidden: boolean;
  dispose(): void;
}

export function createSoundButton(
  parent: HTMLElement,
  target: SoundButtonTarget,
): SoundButton {
  ensureStyle();

  const button = document.createElement('button');
  button.type = 'button';
  button.className = 'hfs-sound-open';
  parent.appendChild(button);

  function refresh(): void {
    const blocked = target.blocked;
    const muted = target.muted;
    button.classList.toggle('blocked', blocked && !muted);
    button.classList.toggle('muted', muted);
    if (muted) {
      button.textContent = 'SOUND OFF';
      button.title = 'The sound is muted. The M key does the same as this button.';
      return;
    }
    if (blocked) {
      button.textContent = 'ENABLE SOUND';
      button.title = 'The browser holds the sound shut until you act. Click here to start it.';
      return;
    }
    button.textContent = 'SOUND ON';
    button.title = 'Mute the sound. The M key does the same as this button.';
  }

  function toggle(): void {
    // A click is a gesture, so it is the moment the browser accepts. Ask for
    // the unlock first, whatever else this press does.
    target.unlock();
    if (target.blocked && !target.muted) {
      // The press was the unlock. It must not also mute, or the pilot has to
      // press the button twice to hear anything.
      refresh();
      return;
    }
    target.muted = !target.muted;
    refresh();
  }

  const onClick = (event: Event): void => {
    event.preventDefault();
    // A button that keeps the focus takes the next Space, and Space is the
    // cannon. See the same note in src/ui/controls-menu.ts.
    button.blur();
    toggle();
  };
  button.addEventListener('click', onClick);
  const offChange = target.onChange(refresh);
  refresh();

  // A write to `style` every frame is a write the browser has to consider, so
  // the setter only touches the DOM when the answer changes.
  let hidden = false;

  return {
    toggle,

    get hidden(): boolean {
      return hidden;
    },

    set hidden(value: boolean) {
      if (value === hidden) return;
      hidden = value;
      button.style.display = hidden ? 'none' : '';
    },

    dispose(): void {
      button.removeEventListener('click', onClick);
      offChange();
      button.remove();
    },
  };
}
