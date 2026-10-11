import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

const backendPort = process.env.ARIA_DASHBOARD_PORT || '8000';

export default defineConfig({
  plugins: [react()],
  server: {
    port: 3000,
    proxy: {
      '/api': {
        target: `http://localhost:${backendPort}`,
        changeOrigin: true,
      },
      '/ws': {
        target: `ws://localhost:${backendPort}`,
        ws: true,
      },
    },
  },
  build: {
    outDir: 'dist',
  },
});
