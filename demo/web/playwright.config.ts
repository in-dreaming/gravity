import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "tests",
  workers: 1,
  timeout: 60_000,
  expect: { timeout: 10_000 },
  use: {
    baseURL: "http://127.0.0.1:5173",
    headless: true,
    viewport: { width: 1440, height: 900 },
    deviceScaleFactor: 1,
    launchOptions: { args: ["--use-angle=swiftshader-webgl", "--enable-unsafe-swiftshader"] }
  },
  webServer: {
    command: "zig build demo-run -j1",
    cwd: "../..",
    port: 5173,
    reuseExistingServer: true,
    timeout: 30_000
  }
});
