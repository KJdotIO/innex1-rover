import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

const roverApiTarget = process.env.ROVER_API_TARGET;

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5174,
    strictPort: false,
    proxy: roverApiTarget
      ? {
          "/rover": {
            target: roverApiTarget,
            changeOrigin: true,
            secure: false,
            rewrite: (path) => path.replace(/^\/rover/, "")
          }
        }
      : undefined
  }
});
