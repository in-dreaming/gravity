import { expect, test } from "@playwright/test";

test("official WASM ABI survives growth, batches, snapshots, and repeated disposal", async ({ page }) => {
  await page.goto("/");
  const status = page.locator("#status");
  await expect(status).toHaveAttribute("data-ready", "true");
  await expect(status).toHaveAttribute("data-hash", "4336297d3f06a9c557e75aea2a839853");
  await expect(status).toHaveAttribute("data-tick", "1");
  await expect(status).toContainText("ABI 1");
});
