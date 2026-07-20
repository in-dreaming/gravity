import { readFile, readdir } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import path from "node:path";

const root = process.cwd();
const requiredDocs = [
  "README.md", "LICENSE", "SECURITY.md", "THIRD_PARTY_NOTICES.md",
  "docs/api/c-abi-v1.md", "docs/api/zig-module.md",
  "docs/formats/asset-format-v1.md", "docs/formats/geometry-asset-tlv-v1.md", "docs/formats/c-abi-v1.schema.json",
  "docs/integration/c-abi.md", "docs/integration/determinism.md", "docs/integration/web-demo.md",
  "docs/release/limits.md", "docs/release/checklist.md", "docs/release/task-audit.md",
  "docs/security/fuzzing.md", "docs/security/sbom.spdx.json", "docs/security/threat-model.md"
];
const forbidden = /\b(?:TODO|FIXME|XXX|HACK|mock|stub)\b|(?:test|describe|it)\.skip\s*\(/i;
const excludedDirectories = new Set(["node_modules", "bin", "obj", "test-results", "playwright-report"]);

async function text(name) { return readFile(path.join(root, name), "utf8"); }
function git(args) { return execFileSync("git", args, { cwd: root, encoding: "utf8" }).trim(); }

async function scan(directory) {
  const findings = [];
  const visit = async current => {
    const items = (await readdir(path.join(root, current), { withFileTypes: true })).sort((a, b) => a.name.localeCompare(b.name, "en"));
    for (const item of items) {
      if (excludedDirectories.has(item.name)) continue;
      const relative = `${current}/${item.name}`;
      if (relative === "tools/qualification_audit.mjs") continue;
      if (item.isDirectory()) await visit(relative);
      else if (item.isFile() && /\.(?:zig|ts|tsx|js|mjs|c|cpp|h)$/.test(item.name)) {
        const lines = (await text(relative)).split(/\r?\n/);
        lines.forEach((line, index) => { if (forbidden.test(line)) findings.push(`${relative}:${index + 1}:${line.trim()}`); });
      }
    }
  };
  await visit(directory);
  return findings;
}

for (const name of requiredDocs) await text(name);
for (let task = 0; task <= 27; task += 1) {
  const prefix = `task-${task.toString().padStart(2, "0")}`;
  const entries = await readdir(path.join(root, "docs/tasks"));
  if (!entries.some(name => name.startsWith(prefix) && name.endsWith(".md"))) throw new Error(`missing authoritative task record: ${prefix}`);
}
await text("docs/tasks/task-22a-spindle-executor-adoption.md");
const taskAudit = await text("docs/release/task-audit.md");
for (let task = 0; task <= 27; task += 1) {
  const id = task.toString().padStart(2, "0");
  if (!new RegExp(`^\\| ${id} \\|`, "m").test(taskAudit)) throw new Error(`Task audit coverage missing: ${id}`);
}
if (!/^\| 22A \|/m.test(taskAudit)) throw new Error("Task audit coverage missing: 22A");
for (const report of [
  "docs/tasks/task-23-scaling-report.md",
  "docs/tasks/task-24-performance-report.md",
  "docs/tasks/task-25-security-report.md",
  "docs/tasks/task-26-demo-wasm-typescript-report.md",
  "docs/tasks/task-27-demo-three-react-cases-report.md",
  "docs/tasks/task-28-product-qualification-report.md",
]) await text(report);

const findings = [...await scan("src"), ...await scan("tools"), ...await scan("tests"), ...await scan("demo/web/src"), ...await scan("demo/web/tests")];
if (findings.length !== 0) throw new Error(`unfinished-code audit failed:\n${findings.join("\n")}`);

const zon = await text("build.zig.zon");
const version = await text("src/version.zig");
const demo = JSON.parse(await text("demo/web/package.json"));
const sbom = JSON.parse(await text("docs/security/sbom.spdx.json"));
if (!zon.includes('.version = "1.0.0"') || !version.includes('package_version = "1.0.0"') || demo.version !== "1.0.0") throw new Error("package version drift");
const gravity = sbom.packages.find(value => value.SPDXID === "SPDXRef-Package-Gravity");
if (gravity?.versionInfo !== "1.0.0") throw new Error("SBOM Gravity version drift");
const spindleGitlink = git(["rev-parse", "HEAD:third_party/spindle"]);
const spindleCheckout = git(["-C", "third_party/spindle", "rev-parse", "HEAD"]);
if (spindleGitlink !== spindleCheckout) throw new Error(`Spindle checkout drift: gitlink ${spindleGitlink}, checkout ${spindleCheckout}`);
const spindle = sbom.packages.find(value => value.SPDXID === "SPDXRef-Package-Spindle");
if (spindle?.versionInfo !== spindleGitlink || !spindle?.downloadLocation?.endsWith(`@${spindleGitlink}`)) throw new Error("SBOM Spindle pin drift");
if (!(await text("THIRD_PARTY_NOTICES.md")).includes(spindleGitlink)) throw new Error("third-party notice Spindle pin drift");
if (git(["-C", "third_party/spindle", "status", "--porcelain"]) !== "") throw new Error("Spindle checkout is dirty");
for (const [name, packageVersion] of [["React", "19.2.7"], ["React DOM", "19.2.7"], ["Three.js", "0.185.1"], ["Vite", "7.3.1"], ["TypeScript", "5.9.3"], ["Playwright Test", "1.61.1"]]) {
  if (!sbom.packages.some(value => value.name === name && value.versionInfo === packageVersion)) throw new Error(`SBOM dependency drift: ${name}`);
}
const workflow = await text(".github/workflows/ci.yml");
for (const target of ["windows-x86_64", "windows-aarch64", "linux-x86_64", "linux-aarch64", "macos-x86_64", "macos-aarch64"]) if (!workflow.includes(target)) throw new Error(`CI target missing: ${target}`);
const allowedGenerated = /^(?:[ MADRCU?!]{1,2}\s+)?tests\/abi\/csharp\/(?:bin|obj)\//;
const unexpectedChanges = git(["status", "--porcelain=v1", "--untracked-files=all"])
  .split(/\r?\n/).filter(Boolean).filter(line => !allowedGenerated.test(line.replaceAll("\\", "/")));
if (unexpectedChanges.length !== 0) throw new Error(`qualification checkout is dirty:\n${unexpectedChanges.join("\n")}`);
console.log(`qualification audit: Task 00-27 documents and audit coverage, ${requiredDocs.length} product documents, clean source/checkouts, versions, Spindle ${spindleGitlink}, SBOM and six native targets verified`);
