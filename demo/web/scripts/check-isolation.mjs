import { readdirSync, readFileSync } from "node:fs";
import { extname, resolve } from "node:path";

const repo = resolve(import.meta.dirname, "../../..");
const sourceRoot = resolve(repo, "demo/web/src");

function files(path) {
  return readdirSync(path, { withFileTypes: true }).flatMap(entry => {
    const child = resolve(path, entry.name);
    return entry.isDirectory() ? files(child) : [child];
  });
}

for (const path of files(sourceRoot).filter(path => extname(path) === ".ts" && !path.endsWith("abi.generated.ts"))) {
  const text = readFileSync(path, "utf8");
  if (/(?:\bas\s+|:\s*|<)any\b/.test(text)) throw new Error(`explicit any escape in ${path}`);
  if (/Math\.(?:sin|cos|tan|sqrt|hypot|atan|acos|asin|round|trunc)/.test(text)) throw new Error(`physics-like numeric implementation in ${path}`);
}

const manifest = readFileSync(resolve(repo, "build.zig.zon"), "utf8");
if (/"demo"/.test(manifest)) throw new Error("core package manifest includes demo");
for (const path of files(resolve(repo, "src")).filter(path => extname(path) === ".zig")) {
  if (readFileSync(path, "utf8").includes("demo/")) throw new Error(`core source references demo: ${path}`);
}
const build = readFileSync(resolve(repo, "build.zig"), "utf8");
if (!build.includes('"src/jobs/wasm_serial.zig"') || !build.includes("target.result.cpu.arch == .wasm32")) throw new Error("WASM serial adapter selection drift");
console.log("verified strict wrapper surface and core/package demo isolation");
