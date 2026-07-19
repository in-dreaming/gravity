import { ABI } from "../wasm/abi.generated";
import type { AssetStore, BodyState, CommandInput, Gravity, WorldStats } from "../wasm/gravity";
import { ZERO, decimalToRaw, transform } from "./fixed";
import { CASES, caseById } from "../cases/catalog";
import type { CaseRuntime, DemoCase, QueryDebug, Visual } from "../cases/types";

const STEP_MS = 1000 / 60;
const PHASE_NAMES = ["prevalidate", "commit", "integrate", "broadphase", "narrowphase", "islands", "solve", "ccd", "sleep", "events", "hash"] as const;

export type DemoView = Readonly<{
  ready: boolean;
  caseId: string;
  title: string;
  description: string;
  paused: boolean;
  tick: number;
  hash: string;
  mirrorHash: string;
  deterministic: boolean;
  stats: WorldStats;
  phaseVisits: readonly Readonly<{ name: string; visits: number }>[];
  stepMilliseconds: number;
  rollbackStatus: string;
  expectedStatus: string;
  memoryPages: number;
  visuals: readonly Visual[];
  previousBodies: readonly BodyState[];
  bodies: readonly BodyState[];
  queryDebug?: QueryDebug;
  focus: readonly [number, number, number];
}>;

type Listener = (view: DemoView) => void;

const EMPTY_STATS: WorldStats = { bodyCount: 0, colliderCount: 0, jointCount: 0, awakeBodyCount: 0, contactCount: 0, broadPairCount: 0, eventCount: 0, workerCount: 1, phaseVisits: [] };

export class DemoController {
  private runtime: CaseRuntime | null = null;
  private definition: DemoCase = CASES[0] as DemoCase;
  private listeners = new Set<Listener>();
  private paused = true;
  private accumulator = 0;
  private previousTimestamp = 0;
  private previousBodies: readonly BodyState[] = [];
  private bodies: readonly BodyState[] = [];
  private stats: WorldStats = EMPTY_STATS;
  private hash = "";
  private mirrorHash = "";
  private lastStepMilliseconds = 0;
  private rollbackStatus = "No late input injected";
  private snapshots = new Map<number, Uint8Array>();
  private history = new Map<number, readonly CommandInput[]>();
  private pending: CommandInput[] = [];
  private nextSequence = 1000;

  constructor(readonly gravity: Gravity, readonly store: AssetStore) {
    this.selectCase(this.definition.id);
  }

  subscribe(listener: Listener): () => void {
    this.listeners.add(listener);
    listener(this.view());
    return () => { this.listeners.delete(listener); };
  }

  selectCase(id: string): void {
    this.disposeRuntime();
    this.definition = caseById(id);
    this.runtime = this.definition.build(this.store);
    this.paused = true;
    this.accumulator = 0;
    this.previousTimestamp = 0;
    this.rollbackStatus = "No late input injected";
    this.snapshots.clear();
    this.history.clear();
    this.pending = [];
    this.nextSequence = 1000;
    this.refreshBodies();
    this.snapshots.set(0, this.world().snapshot());
    this.refreshDiagnostics();
    this.emit();
  }

  reset(): void { this.selectCase(this.definition.id); }
  setPaused(value: boolean): void { this.paused = value; this.emit(); }
  togglePaused(): void { this.setPaused(!this.paused); }

  singleStep(): void {
    this.stepOnce();
    this.emit();
  }

  runTo(targetTick: number): void {
    while (Number(this.world().tick()) < targetTick) this.stepOnce();
    this.emit();
  }

  applyImpulse(canonicalDecimal: string): void {
    const target = this.world().bodyStates().find(value => value.bodyType === ABI.enums.bodyType.dynamic);
    if (target === undefined) throw new Error("case has no dynamic body");
    this.pending.push({ type: ABI.enums.commandType.impulseAtPoint, phasePriority: 0, issuer: 0xd3, sequence: this.nextSequence, body: target.id, first: { x: decimalToRaw(canonicalDecimal), y: 0n, z: 0n }, second: target.transform.position, transform: transform(), dofLocks: 0 });
    this.nextSequence += 1;
  }

  injectLateInput(): void {
    const currentTick = Number(this.world().tick());
    if (currentTick < 3) this.runTo(3);
    const finalTick = Number(this.world().tick());
    const targetTick = finalTick - 3;
    const source = this.snapshots.get(targetTick);
    if (source === undefined) throw new Error(`missing rollback snapshot ${targetTick}`);
    const authorityRuntime = this.definition.build(this.store);
    const authority = authorityRuntime.world;
    try {
      this.world().loadSnapshot(source);
      authority.loadSnapshot(source);
      const target = this.world().bodyStates().find(value => value.bodyType === ABI.enums.bodyType.dynamic);
      if (target === undefined) throw new Error("rollback case has no dynamic body");
      const late: CommandInput = { type: ABI.enums.commandType.impulseAtPoint, phasePriority: 0, issuer: 0x1a7e, sequence: this.nextSequence, body: target.id, first: { x: decimalToRaw("2.5"), y: 0n, z: 0n }, second: target.transform.position, transform: transform(), dofLocks: 0 };
      this.nextSequence += 1;
      for (let tick = targetTick; tick < finalTick; tick += 1) {
        const original = this.history.get(tick) ?? [];
        const commands = tick === targetTick ? [...original, late] : [...original];
        this.world().step(commands);
        authority.step(commands);
        this.history.set(tick, commands);
        this.snapshots.set(tick + 1, this.world().snapshot());
      }
      const authoritativeHash = authority.hash();
      const replayHash = this.world().hash();
      if (authoritativeHash !== replayHash) throw new Error(`rollback mismatch ${replayHash} != ${authoritativeHash}`);
      this.rollbackStatus = `Late input at Tick ${targetTick}; replay matched ${replayHash}`;
    } finally {
      authorityRuntime.mirror?.dispose();
      authority.dispose();
    }
    this.refreshBodies();
    this.refreshDiagnostics();
    this.emit();
  }

  advance(timestamp: number): number {
    if (this.previousTimestamp === 0) this.previousTimestamp = timestamp;
    const elapsed = timestamp - this.previousTimestamp;
    this.previousTimestamp = timestamp;
    if (!this.paused) {
      this.accumulator += elapsed > 250 ? 250 : elapsed;
      let steps = 0;
      while (this.accumulator >= STEP_MS && steps < 4) {
        this.stepOnce();
        this.accumulator -= STEP_MS;
        steps += 1;
      }
      if (steps !== 0) this.emit();
    }
    return this.accumulator / STEP_MS;
  }

  view(): DemoView {
    const tick = this.runtime === null ? 0 : Number(this.world().tick());
    const expectedStatus = this.definition.expectedHash === "" ? "capturing qualification hash" : tick < this.definition.expectedTick ? `expected hash at Tick ${this.definition.expectedTick}` : this.hash === this.definition.expectedHash ? "expected hash matched" : `expected ${this.definition.expectedHash}`;
    const base = {
      ready: this.runtime !== null, caseId: this.definition.id, title: this.definition.title, description: this.definition.description, paused: this.paused, tick, hash: this.hash, mirrorHash: this.mirrorHash,
      deterministic: this.mirrorHash === "" || this.mirrorHash === this.hash, stats: this.stats, phaseVisits: PHASE_NAMES.map((name, index) => ({ name, visits: this.stats.phaseVisits[index] ?? 0 })), stepMilliseconds: this.lastStepMilliseconds,
      rollbackStatus: this.rollbackStatus, expectedStatus, memoryPages: this.gravity.memoryPages, visuals: this.runtime?.visuals ?? [], previousBodies: this.previousBodies, bodies: this.bodies, focus: this.runtime?.focus ?? [0, 0, 0] as const
    };
    return this.runtime?.queryDebug === undefined ? base : { ...base, queryDebug: this.runtime.queryDebug };
  }

  dispose(): void {
    this.disposeRuntime();
    this.listeners.clear();
    this.store.dispose();
  }

  private world() {
    if (this.runtime === null) throw new Error("demo runtime is not initialized");
    return this.runtime.world;
  }

  private stepOnce(): void {
    const runtime = this.runtime;
    if (runtime === null) return;
    const tick = Number(runtime.world.tick());
    const commands = [...(tick === 0 ? runtime.startupCommands : []), ...this.pending];
    this.pending = [];
    this.previousBodies = this.bodies;
    const started = performance.now();
    runtime.world.step(commands);
    this.lastStepMilliseconds = performance.now() - started;
    runtime.mirror?.step(commands);
    this.history.set(tick, commands);
    this.snapshots.set(tick + 1, runtime.world.snapshot());
    if (this.snapshots.size > 121) this.snapshots.delete(tick - 120);
    this.refreshBodies();
    this.refreshDiagnostics();
  }

  private refreshBodies(): void {
    this.bodies = this.world().bodyStates();
    if (this.previousBodies.length === 0) this.previousBodies = this.bodies;
  }

  private refreshDiagnostics(): void {
    this.hash = this.world().hash();
    this.stats = this.world().stats();
    this.mirrorHash = this.runtime?.mirror?.hash() ?? "";
  }

  private disposeRuntime(): void {
    if (this.runtime === null) return;
    this.runtime.mirror?.dispose();
    this.runtime.world.dispose();
    this.runtime = null;
  }

  private emit(): void {
    const value = this.view();
    for (const listener of this.listeners) listener(value);
  }
}
