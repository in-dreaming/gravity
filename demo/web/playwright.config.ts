import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "tests",
  timeout: 30_000,
  use: { baseURL: "http://127.0.0.1:5173", headless: true },
  webServer: {
    command: "zig build demo-run -j1",
    cwd: "../..",
    port: 5173,
    reuseExistingServer: false,
    timeout: 30_000
  }
});
