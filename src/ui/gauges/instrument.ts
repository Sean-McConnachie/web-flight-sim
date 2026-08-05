/**
 * What every instrument of the panel looks like from outside.
 *
 * One module builds one instrument. The module paints its face one time, hangs
 * its needles on the face disc, and then moves nothing but those needles. The
 * panel of src/ui/gauges/index.ts holds the list and calls `update` one time
 * per frame, and only while the cockpit view runs.
 */

import type { TelemetrySample } from '@/ui/debug-overlay';

import type { CockpitReadout } from './readout';

export interface Instrument {
  /** Move the needles of this frame. `dt` is the frame time in seconds. */
  update(sample: TelemetrySample, readout: CockpitReadout, dt: number): void;
  /** Give back every geometry, material and texture the instrument made. */
  dispose(): void;
}
