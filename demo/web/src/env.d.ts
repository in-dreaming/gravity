import type { DemoController } from "./physics/controller";

declare global {
  var gravityDemo: DemoController | undefined;
}

export {};
