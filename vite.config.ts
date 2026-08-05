import { defineConfig } from 'vitest/config';
import { fileURLToPath } from 'node:url';

export default defineConfig({
  // BEAD xq3. GitHub Pages serves a project site from /<repo>/ and not from the
  // root. A relative base makes every asset path relative to index.html, so the
  // same bundle runs at the root, under a repository name, and from a file on
  // disk. The alternative is to write the repository name here, and that breaks
  // the moment somebody forks the project or renames it.
  base: './',
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
