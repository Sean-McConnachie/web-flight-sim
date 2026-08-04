import { defineConfig } from 'vitest/config';
import { fileURLToPath } from 'node:url';

export default defineConfig({
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  server: {
    port: 5173,
    open: false,
  },
  build: {
    target: 'esnext',
    sourcemap: true,
  },
  test: {
    include: ['test/**/*.test.ts'],
    environment: 'node',
    testTimeout: 60_000,
  },
});
