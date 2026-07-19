import { expect, test, type Page } from "@playwright/test";

const expected = {
  "stack-pyramid": [4, "5ede974e91341090464b89f92347c2cd"],
  "material-ramp": [4, "21bb6174ef8fc090fe0186ad1575acd2"],
  "newton-cradle": [4, "de5e65cedb4928e5b68968231563bad7"],
  "joint-gallery": [4, "5ee81fecac78ac45387cbe8618acabde"],
  ragdoll: [4, "70dff34395111e11f6af91ad4d2c6fef"],
  "hull-compound": [4, "e6ca20bb55d98fae6b4ed042b97f308e"],
  "dynamic-mesh": [4, "4859f9b6e51c3143203a27dea96592be"],
  "height-field": [4, "9b01aac6f537bca84fb23c66472b6d9e"],
  ccd: [4, "7de0f5cd05411dc6d0d90c3987f2965c"],
  queries: [4, "1483074bce75f17737c1d2615e06df05"],
  "planar-2d": [4, "1f4c8353daba9b65a7c7ed2d870aeb55"],
  "sleep-wake": [36, "d1a7e33d11edf57fd8826c83abe787f7"],
  rollback: [8, "87d8d73dfdcb6b7b87ba61dcc48eafbd"],
  determinism: [4, "e624782f796d619da67d1e50a0df2c5b"],
  stress: [4, "c258557689232338d5f025fbc4b2d1c5"]
} as const;

async function ready(page: Page) {
  await page.goto("/");
  await expect(page.locator("#status")).toHaveAttribute("data-ready", "true");
}

test("all 15 cases run through the formal ABI and reset to their qualification hash", async ({ page }) => {
  await ready(page);
  const results = await page.evaluate((qualification) => {
    const demo = globalThis.gravityDemo;
    if (demo === undefined) throw new Error("demo controller is unavailable");
    return Object.entries(qualification).map(([id, [tick, hash]]) => {
      demo.selectCase(id);
      demo.runTo(tick);
      const first = demo.view();
      demo.reset();
      demo.runTo(tick);
      const reset = demo.view();
      return { id, expected: hash, first: first.hash, reset: reset.hash, tick: reset.tick, deterministic: reset.deterministic };
    });
  }, expected);
  expect(results).toHaveLength(15);
  for (const result of results) {
    expect(result.first, `${result.id} first run`).toBe(result.expected);
    expect(result.reset, `${result.id} reset`).toBe(result.expected);
    expect(result.deterministic, `${result.id} dual-world parity`).toBe(true);
  }
});

test("pause, single-step, renderer interpolation, sleep/wake and rollback are exact", async ({ page }) => {
  await ready(page);
  await page.getByRole("button", { name: "Step" }).click();
  await expect(page.getByText("Tick 1", { exact: true })).toBeVisible();
  const steppedHash = await page.getByTestId("world-hash").textContent();
  await page.waitForTimeout(150);
  await expect(page.getByText("Tick 1", { exact: true })).toBeVisible();
  await expect(page.getByTestId("world-hash")).toHaveText(steppedHash ?? "");

  await page.getByLabel("Classic case").selectOption("sleep-wake");
  const sleepState = await page.evaluate(() => {
    const demo = globalThis.gravityDemo!;
    demo.runTo(36);
    const asleep = demo.view();
    demo.applyImpulse("2.5");
    demo.singleStep();
    return { asleep: asleep.stats.awakeBodyCount, awake: demo.view().stats.awakeBodyCount };
  });
  expect(sleepState.asleep).toBe(0);
  expect(sleepState.awake).toBeGreaterThan(0);

  await page.getByLabel("Classic case").selectOption("rollback");
  const replay = await page.evaluate(() => {
    const demo = globalThis.gravityDemo!;
    demo.runTo(8);
    demo.injectLateInput();
    return demo.view();
  });
  expect(replay.tick).toBe(8);
  expect(replay.rollbackStatus).toContain("replay matched");
  expect(replay.deterministic).toBe(true);
});

test("case switching reaches a stable WASM high-water mark and the controls are accessible", async ({ page }) => {
  await ready(page);
  await expect(page.getByRole("complementary", { name: "Simulation controls" })).toBeVisible();
  await expect(page.getByRole("main", { name: "Three.js physics viewport" })).toBeVisible();
  await expect(page.getByRole("complementary", { name: "Diagnostics" })).toBeVisible();
  await expect(page.getByLabel("Classic case")).toHaveValue("stack-pyramid");
  await expect(page.getByRole("button", { name: "Run" })).toBeEnabled();
  await expect(page.getByLabel("Impulse")).toHaveValue("2.5");

  const pages = await page.evaluate((ids) => {
    const demo = globalThis.gravityDemo!;
    const cycles: number[] = [];
    for (let cycle = 0; cycle < 3; cycle += 1) {
      for (const id of ids) demo.selectCase(id);
      cycles.push(demo.view().memoryPages);
    }
    return cycles;
  }, Object.keys(expected));
  expect(pages[1]).toBe(pages[0]);
  expect(pages[2]).toBe(pages[1]);
});

test("fixed software-rendered overview remains perceptually stable", async ({ page }) => {
  await ready(page);
  await page.evaluate(() => globalThis.gravityDemo!.runTo(4));
  await expect(page.getByTestId("world-hash")).toHaveText(expected["stack-pyramid"][1]);
  await expect(page).toHaveScreenshot("stack-overview.png", { animations: "disabled", maxDiffPixelRatio: 0.04 });
});
