import { createHash } from "node:crypto";
import { execFileSync } from "node:child_process";
import { chmod, lstat, mkdir, mkdtemp, readFile, readdir, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";

const nativeTargets = ["windows-x86_64", "windows-aarch64", "linux-x86_64", "linux-aarch64", "macos-x86_64", "macos-aarch64"];
const excluded = new Set([".git", ".zig-cache", "zig-out", "node_modules", "playwright-report", "test-results", "bin", "obj"]);
const generatedDirectory = /^(?:\.zig-cache|\.zig-global|zig-out)/;

function octal(value, width) {
  const text = value.toString(8);
  if (text.length > width - 1) throw new Error(`tar numeric field overflow: ${value}`);
  return `${"0".repeat(width - 1 - text.length)}${text}\0`;
}

function tarName(name) {
  const bytes = Buffer.byteLength(name);
  if (bytes <= 100) return { name, prefix: "" };
  for (let index = name.lastIndexOf("/"); index > 0; index = name.lastIndexOf("/", index - 1)) {
    const prefix = name.slice(0, index);
    const leaf = name.slice(index + 1);
    if (Buffer.byteLength(prefix) <= 155 && Buffer.byteLength(leaf) <= 100) return { name: leaf, prefix };
  }
  throw new Error(`tar path is too long: ${name}`);
}

function copyText(header, offset, width, value) {
  const bytes = Buffer.from(value);
  if (bytes.length > width) throw new Error(`tar field is too long: ${value}`);
  bytes.copy(header, offset);
}

function tarHeader(entry) {
  const header = Buffer.alloc(512);
  const split = tarName(entry.name);
  copyText(header, 0, 100, split.name);
  copyText(header, 100, 8, octal(0o644, 8));
  copyText(header, 108, 8, octal(0, 8));
  copyText(header, 116, 8, octal(0, 8));
  copyText(header, 124, 12, octal(entry.data.length, 12));
  copyText(header, 136, 12, octal(0, 12));
  header.fill(0x20, 148, 156);
  header[156] = "0".charCodeAt(0);
  copyText(header, 257, 6, "ustar\0");
  copyText(header, 263, 2, "00");
  copyText(header, 265, 32, "root");
  copyText(header, 297, 32, "root");
  copyText(header, 345, 155, split.prefix);
  let sum = 0;
  for (const byte of header) sum += byte;
  const checksum = `${sum.toString(8).padStart(6, "0")}\0 `;
  copyText(header, 148, 8, checksum);
  return header;
}

async function normalizeStaticArchive(disk) {
  const temporary = await mkdtemp(path.join(tmpdir(), "gravity-release-ar-"));
  try {
    const input = path.join(temporary, "input.a");
    const output = path.join(temporary, "normalized.a");
    await writeFile(input, await readFile(disk));
    const archivedNames = execFileSync("zig", ["ar", "t", input], { encoding: "utf8" }).split(/\r?\n/).filter(Boolean);
    const members = archivedNames.map(name => name.split(/[\\/]/).at(-1));
    if (new Set(members).size !== members.length) throw new Error(`static archive has duplicate member basenames: ${disk}`);
    execFileSync("zig", ["ar", "x", input], { cwd: temporary, stdio: "pipe" });
    await Promise.all(members.map(member => chmod(path.join(temporary, member), 0o644)));
    execFileSync("zig", ["ar", "rcsD", output, ...members], { cwd: temporary, stdio: "pipe" });
    return await readFile(output);
  } finally {
    await rm(temporary, { recursive: true, force: true });
  }
}

async function addTree(entries, diskRoot, archiveRoot, normalizeArchives = false) {
  const names = (await readdir(diskRoot, { withFileTypes: true })).sort((a, b) => a.name.localeCompare(b.name, "en"));
  for (const item of names) {
    if (excluded.has(item.name) || generatedDirectory.test(item.name)) continue;
    const disk = path.join(diskRoot, item.name);
    const archive = `${archiveRoot}/${item.name}`.replaceAll("\\", "/");
    if (item.isDirectory()) await addTree(entries, disk, archive, normalizeArchives);
    else if (item.isFile()) {
      if (normalizeArchives && item.name.endsWith(".pdb")) continue;
      const isStaticArchive = item.name.endsWith(".a") || item.name.endsWith("_static.lib");
      entries.push({ name: archive, data: normalizeArchives && isStaticArchive ? await normalizeStaticArchive(disk) : await readFile(disk) });
    }
    else if (item.isSymbolicLink()) throw new Error(`release input may not contain symlinks: ${disk}`);
  }
}

async function addFile(entries, disk, archive) {
  const info = await lstat(disk);
  if (!info.isFile()) throw new Error(`release input is not a file: ${disk}`);
  entries.push({ name: archive.replaceAll("\\", "/"), data: await readFile(disk) });
}

async function writeTar(destination, entries) {
  entries.sort((a, b) => a.name.localeCompare(b.name, "en"));
  const chunks = [];
  for (const entry of entries) {
    chunks.push(tarHeader(entry), entry.data);
    const padding = (512 - (entry.data.length % 512)) % 512;
    if (padding !== 0) chunks.push(Buffer.alloc(padding));
  }
  chunks.push(Buffer.alloc(1024));
  await writeFile(destination, Buffer.concat(chunks));
}

async function common(entries, root) {
  for (const name of ["LICENSE", "SECURITY.md", "THIRD_PARTY_NOTICES.md", "include/gravity.h", "docs/api/c-abi-v1.md", "docs/integration/c-abi.md", "docs/release/limits.md", "docs/security/sbom.spdx.json"]) {
    await addFile(entries, path.join(root, name), name);
  }
}

async function generate(prefix, version, commit) {
  const root = process.cwd();
  const status = execFileSync("git", ["status", "--porcelain", "--untracked-files=no"], { cwd: root, encoding: "utf8" });
  if (status !== "") throw new Error("release generation requires clean tracked source and submodule worktrees");
  const untracked = execFileSync("git", ["ls-files", "--others", "--exclude-standard"], { cwd: root, encoding: "utf8" }).split(/\r?\n/).filter(Boolean);
  const unexpected = untracked.filter(name => !/(?:^|\/)(?:\.zig-cache[^/]*|\.zig-global[^/]*|zig-out[^/]*)\//.test(name.replaceAll("\\", "/")));
  if (unexpected.length !== 0) throw new Error(`release generation found untracked source:\n${unexpected.join("\n")}`);
  const actualCommit = execFileSync("git", ["rev-parse", "HEAD"], { cwd: root, encoding: "utf8" }).trim();
  if (actualCommit !== commit) throw new Error(`release commit mismatch: ${commit} != ${actualCommit}`);
  const output = path.join(prefix, "release");
  await rm(output, { recursive: true, force: true });
  await mkdir(output, { recursive: true });
  const packages = [];
  const emit = async (name, entries) => {
    const filename = `gravity-${version}-${name}.tar`;
    const destination = path.join(output, filename);
    await writeTar(destination, entries);
    const bytes = await readFile(destination);
    packages.push({ file: filename, bytes: bytes.length, sha256: createHash("sha256").update(bytes).digest("hex") });
  };

  const source = [];
  for (const name of ["build.zig", "build.zig.zon", ".zigversion", ".gitmodules", "README.md", "CONTRIBUTING.md", "LICENSE", "SECURITY.md", "THIRD_PARTY_NOTICES.md"]) await addFile(source, path.join(root, name), name);
  for (const name of ["src", "include", "tools", "tests", "docs", "demo", "third_party/spindle"]) await addTree(source, path.join(root, name), name);
  await emit("source", source);

  for (const target of nativeTargets) {
    const entries = [];
    await common(entries, root);
    await addTree(entries, path.join(prefix, "abi", target, "lib"), "lib", true);
    await emit(target, entries);
  }

  const wasm = [];
  await common(wasm, root);
  await addFile(wasm, path.join(prefix, "bin", "gravity.wasm"), "bin/gravity.wasm");
  await emit("wasm32-freestanding", wasm);

  const demo = [];
  await common(demo, root);
  await addTree(demo, path.join(prefix, "demo"), "demo");
  await addTree(demo, path.join(prefix, "bin", "demo-assets"), "demo-assets");
  await emit("demo", demo);

  const manifest = { schema: "gravity.release.v1", version, commit, zig: "0.16.0", generatedUtc: "1970-01-01T00:00:00Z", packages };
  await writeFile(path.join(output, "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`);
  await writeFile(path.join(output, "SHA256SUMS"), `${packages.map(item => `${item.sha256}  ${item.file}`).join("\n")}\n`);
  console.log(`release ${version}: ${packages.length} deterministic packages, manifest and SHA256SUMS`);
}

async function verify(directory) {
  const manifest = JSON.parse(await readFile(path.join(directory, "manifest.json"), "utf8"));
  if (manifest.schema !== "gravity.release.v1" || !Array.isArray(manifest.packages) || manifest.packages.length !== 9) throw new Error("invalid release manifest");
  for (const item of manifest.packages) {
    const bytes = await readFile(path.join(directory, item.file));
    const digest = createHash("sha256").update(bytes).digest("hex");
    if (bytes.length !== item.bytes || digest !== item.sha256) throw new Error(`release checksum mismatch: ${item.file}`);
  }
  const expected = `${manifest.packages.map(item => `${item.sha256}  ${item.file}`).join("\n")}\n`;
  if (await readFile(path.join(directory, "SHA256SUMS"), "utf8") !== expected) throw new Error("SHA256SUMS does not match manifest");
  console.log(`verified ${manifest.packages.length} release packages and SHA-256 checksums`);
}

async function compare(first, second) {
  await verify(first);
  await verify(second);
  const left = JSON.parse(await readFile(path.join(first, "manifest.json"), "utf8"));
  const right = JSON.parse(await readFile(path.join(second, "manifest.json"), "utf8"));
  if (JSON.stringify(left) !== JSON.stringify(right)) throw new Error("release manifests are not reproducible");
  for (const item of left.packages) {
    const a = await readFile(path.join(first, item.file));
    const b = await readFile(path.join(second, item.file));
    if (!a.equals(b)) throw new Error(`release package is not byte-identical: ${item.file}`);
  }
  console.log(`reproducible release: ${left.packages.length} byte-identical packages`);
}

const [mode, ...args] = process.argv.slice(2);
if (mode === "--generate" && args.length === 3) await generate(...args);
else if (mode === "--verify" && args.length === 1) await verify(args[0]);
else if (mode === "--compare" && args.length === 2) await compare(...args);
else throw new Error("usage: release.mjs --generate <prefix> <version> <commit> | --verify <release-dir> | --compare <release-dir-a> <release-dir-b>");
