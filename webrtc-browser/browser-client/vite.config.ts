import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    port: 3000,
    host: true // Allow external connections
  }
});

