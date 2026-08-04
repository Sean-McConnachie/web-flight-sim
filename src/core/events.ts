/**
 * Typed event bus.
 *
 * The caller gives a map type. Each key of the map is an event name. The value
 * of that key is the payload type of the event.
 *
 * ```ts
 * interface SimEvents { crash: { speed: number }; pause: boolean }
 * const bus = createEmitter<SimEvents>();
 * const off = bus.on('crash', (event) => report(event.speed));
 * bus.emit('crash', { speed: 82 });
 * off();
 * ```
 *
 * The bus holds no state of the simulator. It only carries messages between
 * modules that must not import each other.
 */

export type Handler<T> = (payload: T) => void;

export interface Emitter<M> {
  /** Adds a handler. The returned function removes it again. */
  on<K extends keyof M>(key: K, fn: Handler<M[K]>): () => void;

  /** Adds a handler that runs one time. The returned function removes it. */
  once<K extends keyof M>(key: K, fn: Handler<M[K]>): () => void;

  /** Removes a handler that `on` or `once` added. */
  off<K extends keyof M>(key: K, fn: Handler<M[K]>): void;

  /** Calls every handler of `key` with `payload`. */
  emit<K extends keyof M>(key: K, payload: M[K]): void;
}

// The store keeps the payload type out of the record. Each public method knows
// the payload type from the key, so it converts at the boundary only.
type StoredHandler = (payload: unknown) => void;

interface Registration {
  /** The emitter calls this function. */
  readonly run: StoredHandler;

  /** The caller gave this function. `off` matches on this value. */
  readonly given: StoredHandler;
}

export function createEmitter<M>(): Emitter<M> {
  const byKey = new Map<keyof M, Registration[]>();

  function listFor(key: keyof M): Registration[] {
    let list = byKey.get(key);
    if (list === undefined) {
      list = [];
      byKey.set(key, list);
    }
    return list;
  }

  function remove(key: keyof M, given: StoredHandler): void {
    const list = byKey.get(key);
    if (list === undefined) return;
    const index = list.findIndex((entry) => entry.given === given);
    if (index < 0) return;
    list.splice(index, 1);
    if (list.length === 0) byKey.delete(key);
  }

  return {
    on<K extends keyof M>(key: K, fn: Handler<M[K]>): () => void {
      const given = fn as StoredHandler;
      listFor(key).push({ run: given, given });
      return () => {
        remove(key, given);
      };
    },

    once<K extends keyof M>(key: K, fn: Handler<M[K]>): () => void {
      const given = fn as StoredHandler;
      const run: StoredHandler = (payload) => {
        remove(key, given);
        given(payload);
      };
      listFor(key).push({ run, given });
      return () => {
        remove(key, given);
      };
    },

    off<K extends keyof M>(key: K, fn: Handler<M[K]>): void {
      remove(key, fn as StoredHandler);
    },

    emit<K extends keyof M>(key: K, payload: M[K]): void {
      const list = byKey.get(key);
      if (list === undefined || list.length === 0) return;
      // Walk a copy. A handler can then add or remove a handler during the
      // emit. The change reaches the next emit, not the emit in progress.
      const snapshot = list.slice();
      for (const entry of snapshot) entry.run(payload);
    },
  };
}
