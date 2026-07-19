import { Gravity } from "./wasm/gravity";
import { runAbiSmoke } from "./smoke";

const output = document.querySelector<HTMLOutputElement>("#status");
if (output === null) throw new Error("missing status output");

try {
  const gravity = await Gravity.load("/gravity.wasm");
  const report = runAbiSmoke(gravity);
  output.value = `ABI ${gravity.abiVersion} hash ${report.hash}`;
  output.dataset.hash = report.hash;
  output.dataset.pages = report.pages.toString();
  output.dataset.tick = report.tick;
  output.dataset.ready = "true";
} catch (error: unknown) {
  output.value = error instanceof Error ? error.message : String(error);
  output.dataset.ready = "false";
}
