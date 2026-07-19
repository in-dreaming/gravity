import { createHash } from "node:crypto";
import { execFileSync } from "node:child_process";
import { readFileSync, writeFileSync, existsSync, mkdirSync } from "node:fs";
import { dirname, resolve } from "node:path";
import process from "node:process";

const root = resolve(import.meta.dirname, "..");
const manifest = JSON.parse(readFileSync(resolve(root, "package.json"), "utf8"));
const stampPath = resolve(root, "../../.zig-cache/demo-install.stamp");
const runPnpm = (args, options = {}) => process.platform === "win32"
  ? execFileSync(process.env.ComSpec ?? "cmd.exe", ["/d", "/s", "/c", ["pnpm", ...args].join(" ")], options)
  : execFileSync("pnpm", args, options);
const nodeVersion = process.version;
const pnpmVersion = runPnpm(["--version"], { encoding: "utf8" }).trim();
if (nodeVersion !== `v${manifest.engines.node}` || pnpmVersion !== manifest.engines.pnpm) {
  throw new Error(`demo requires Node ${manifest.engines.node} and pnpm ${manifest.engines.pnpm}; found ${nodeVersion} and ${pnpmVersion}`);
}
const hash = createHash("sha256")
  .update(readFileSync(resolve(root, "package.json")))
  .update(readFileSync(resolve(root, "pnpm-lock.yaml")))
  .update(nodeVersion)
  .update(pnpmVersion)
  .digest("hex");

const installed = existsSync(resolve(root, "node_modules/.modules.yaml"));
const current = existsSync(stampPath) ? readFileSync(stampPath, "utf8").trim() : "";
if (!installed || current !== hash) {
  runPnpm(["install", "--frozen-lockfile"], { cwd: root, stdio: "inherit" });
  mkdirSync(dirname(stampPath), { recursive: true });
  writeFileSync(stampPath, `${hash}\n`);
  console.log(`demo dependencies installed (${nodeVersion}, pnpm ${pnpmVersion})`);
} else {
  console.log(`demo dependencies unchanged (${hash.slice(0, 12)})`);
}
