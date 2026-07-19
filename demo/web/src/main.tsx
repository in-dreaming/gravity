import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./ui/App";
import { DemoController } from "./physics/controller";
import { runAbiSmoke } from "./smoke";
import { Gravity } from "./wasm/gravity";
import "./ui/app.css";

const output = document.querySelector<HTMLOutputElement>("#status");
const rootElement = document.querySelector<HTMLElement>("#root");
if (output === null || rootElement === null) throw new Error("missing demo root");

async function loadAsset(path: string): Promise<Uint8Array> {
  const response = await fetch(path);
  if (!response.ok) throw new Error(`failed to load ${path}: ${response.status}`);
  return new Uint8Array(await response.arrayBuffer());
}

try {
  const gravity = await Gravity.load("/gravity.wasm");
  const report = runAbiSmoke(gravity);
  const assets = await Promise.all(["/hull.grav", "/mesh.grav", "/height.grav", "/compound.grav"].map(loadAsset));
  const controller = new DemoController(gravity, gravity.createAssetStore(assets));
  globalThis.gravityDemo = controller;
  output.value = `ABI ${gravity.abiVersion} hash ${report.hash}`;
  output.dataset.hash = report.hash;
  output.dataset.pages = report.pages.toString();
  output.dataset.tick = report.tick;
  output.dataset.ready = "true";
  createRoot(rootElement).render(<StrictMode><App controller={controller} /></StrictMode>);
  globalThis.addEventListener("beforeunload", () => controller.dispose(), { once: true });
} catch (error: unknown) {
  output.value = error instanceof Error ? error.message : String(error);
  output.dataset.ready = "false";
  rootElement.textContent = output.value;
}
