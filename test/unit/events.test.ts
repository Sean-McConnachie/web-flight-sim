import { describe, expect, it } from 'vitest';
import type { Handler } from '@/core/events';
import { createEmitter } from '@/core/events';

interface TestEvents {
  tick: number;
  crash: { speed: number };
  pause: boolean;
}

describe('typed event bus', () => {
  it('a handler receives the payload of its own key only', () => {
    const bus = createEmitter<TestEvents>();
    const ticks: number[] = [];
    const crashes: number[] = [];

    bus.on('tick', (value) => ticks.push(value));
    bus.on('crash', (event) => crashes.push(event.speed));

    bus.emit('tick', 3);
    bus.emit('crash', { speed: 82.5 });
    bus.emit('pause', true);

    expect(ticks).toEqual([3]);
    expect(crashes).toEqual([82.5]);
  });

  it('an emit with no handler does nothing', () => {
    const bus = createEmitter<TestEvents>();
    expect(() => bus.emit('tick', 1)).not.toThrow();
  });

  it('every handler of one key runs in the order of the subscriptions', () => {
    const bus = createEmitter<TestEvents>();
    const order: string[] = [];
    bus.on('tick', () => order.push('first'));
    bus.on('tick', () => order.push('second'));
    bus.on('tick', () => order.push('third'));

    bus.emit('tick', 1);
    expect(order).toEqual(['first', 'second', 'third']);
  });

  it('the function that on returns removes the handler', () => {
    const bus = createEmitter<TestEvents>();
    const seen: number[] = [];
    const off = bus.on('tick', (value) => seen.push(value));

    bus.emit('tick', 1);
    off();
    bus.emit('tick', 2);

    expect(seen).toEqual([1]);
  });

  it('off removes the handler that on added', () => {
    const bus = createEmitter<TestEvents>();
    const seen: number[] = [];
    const handler: Handler<number> = (value) => seen.push(value);

    bus.on('tick', handler);
    bus.emit('tick', 1);
    bus.off('tick', handler);
    bus.emit('tick', 2);

    expect(seen).toEqual([1]);
  });

  it('off removes one handler and leaves the others', () => {
    const bus = createEmitter<TestEvents>();
    const seen: string[] = [];
    const first: Handler<number> = () => seen.push('first');
    bus.on('tick', first);
    bus.on('tick', () => seen.push('second'));

    bus.off('tick', first);
    bus.emit('tick', 1);

    expect(seen).toEqual(['second']);
  });

  it('a once handler runs one time only', () => {
    const bus = createEmitter<TestEvents>();
    const seen: number[] = [];
    bus.once('tick', (value) => seen.push(value));

    bus.emit('tick', 1);
    bus.emit('tick', 2);
    bus.emit('tick', 3);

    expect(seen).toEqual([1]);
  });

  it('off removes a once handler before it runs', () => {
    const bus = createEmitter<TestEvents>();
    const seen: number[] = [];
    const handler: Handler<number> = (value) => seen.push(value);

    bus.once('tick', handler);
    bus.off('tick', handler);
    bus.emit('tick', 1);

    expect(seen).toEqual([]);
  });

  it('a handler that removes itself does not stop the handlers after it', () => {
    const bus = createEmitter<TestEvents>();
    const seen: string[] = [];

    const off = bus.on('tick', () => {
      seen.push('first');
      off();
    });
    bus.on('tick', () => seen.push('second'));
    bus.on('tick', () => seen.push('third'));

    bus.emit('tick', 1);
    expect(seen).toEqual(['first', 'second', 'third']);

    bus.emit('tick', 2);
    expect(seen).toEqual(['first', 'second', 'third', 'second', 'third']);
  });

  it('a handler that removes another handler does not change the emit in progress', () => {
    const bus = createEmitter<TestEvents>();
    const seen: string[] = [];

    bus.on('tick', () => {
      seen.push('first');
      offSecond();
    });
    const offSecond = bus.on('tick', () => seen.push('second'));
    bus.on('tick', () => seen.push('third'));

    // The emit walks a copy, so the removed handler still runs this one time.
    bus.emit('tick', 1);
    expect(seen).toEqual(['first', 'second', 'third']);

    bus.emit('tick', 2);
    expect(seen).toEqual(['first', 'second', 'third', 'first', 'third']);
  });

  it('a handler that subscribes during an emit runs from the next emit', () => {
    const bus = createEmitter<TestEvents>();
    const seen: string[] = [];

    const off = bus.on('tick', () => {
      seen.push('outer');
      bus.once('tick', () => seen.push('inner'));
    });

    bus.emit('tick', 1);
    expect(seen).toEqual(['outer']);

    off();
    bus.emit('tick', 2);
    expect(seen).toEqual(['outer', 'inner']);
  });
});
