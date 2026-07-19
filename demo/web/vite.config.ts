import { defineConfig } from "vite";

export default defineConfig({
  publicDir: "../../zig-out/bin/demo-assets",
  build: {
    outDir: "../../zig-out/demo",
    emptyOutDir: true,
    target: "es2022"
  }
});
