import { readFileSync } from "node:fs";
import { resolve } from "node:path";

const wasmPath = process.argv[2];
if (wasmPath === undefined) throw new Error("missing WASM path");
const repo = resolve(import.meta.dirname, "../../..");
const baseline = JSON.parse(readFileSync(resolve(repo, "tests/abi/abi-baseline-v1.json"), "utf8"));
const bytes = readFileSync(wasmPath);
const module = await WebAssembly.compile(bytes);
const imports = WebAssembly.Module.imports(module);
if (imports.length !== 0) throw new Error(`WASM must be freestanding; imports: ${imports.map(value => `${value.module}.${value.name}`).join(", ")}`);
const exports = WebAssembly.Module.exports(module).map(value => value.name);
for (const required of ["memory", ...baseline.exports]) {
  if (!exports.includes(required)) throw new Error(`missing WASM export ${required}`);
}
const allowed = new Set(["memory", ...baseline.exports]);
const unexpected = exports.filter(name => !allowed.has(name));
if (unexpected.length !== 0) throw new Error(`unexpected WASM exports: ${unexpected.join(", ")}`);
const embedded = bytes.toString("latin1").toLowerCase();
for (const forbidden of ["spindle", "host_dispatcher", "pthread", "atomic.wait"]) {
  if (embedded.includes(forbidden)) throw new Error(`forbidden WASM graph marker ${forbidden}`);
}
console.log(`verified ${baseline.exports.length} Gravity exports; zero imports; serial-only graph`);
