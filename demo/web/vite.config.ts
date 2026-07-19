import { defineConfig } from "vite";

const installPrefix = (globalThis as { process?: { env: Record<string, string | undefined> } }).process?.env.GRAVITY_INSTALL_PREFIX ?? "../../zig-out";

export default defineConfig({
  publicDir: `${installPrefix}/bin/demo-assets`,
  build: {
    outDir: `${installPrefix}/demo`,
    emptyOutDir: true,
    target: "es2022"
  }
});
