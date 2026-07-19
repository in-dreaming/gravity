import type { AssetStore, CommandInput, GravityId, QueryHit, World } from "../wasm/gravity";

export type VisualKind = "sphere" | "box" | "capsule" | "hull" | "compound" | "mesh" | "height";
export type Visual = Readonly<{ body: GravityId; kind: VisualKind; size: readonly [number, number, number]; color: number; wireframe?: boolean }>;
export type DebugLine = Readonly<{ from: readonly [number, number, number]; to: readonly [number, number, number]; color: number }>;
export type DebugPoint = Readonly<{ at: readonly [number, number, number]; color: number }>;
export type QueryDebug = Readonly<{ lines: readonly DebugLine[]; points: readonly DebugPoint[]; hits: readonly QueryHit[] }>;

export type CaseRuntime = {
  world: World;
  mirror?: World;
  visuals: Visual[];
  focus: readonly [number, number, number];
  startupCommands: CommandInput[];
  queryDebug?: QueryDebug;
};

export type DemoCase = Readonly<{
  id: string;
  title: string;
  category: string;
  description: string;
  seed: string;
  expectedTick: number;
  expectedHash: string;
  build(store: AssetStore): CaseRuntime;
}>;
