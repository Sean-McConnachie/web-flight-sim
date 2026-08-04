import { describe, expect, it, afterAll } from 'vitest';
describe('out', () => {
  it('prints', () => {
    console.log('LOGLINE');
    console.error('ERRLINE');
    process.stdout.write('RAWSTDOUT\n');
    process.stderr.write('RAWSTDERR\n');
    expect(1).toBe(1);
  });
  afterAll(() => {
    console.log('AFTERALL-LOG');
    process.stdout.write('AFTERALL-RAW\n');
  });
});
