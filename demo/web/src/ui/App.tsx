import { useEffect, useRef, useState } from "react";
import { CASES } from "../cases/catalog";
import type { DemoController, DemoView } from "../physics/controller";
import { DemoScene } from "../renderer/scene";

type Props = Readonly<{ controller: DemoController }>;

export function App({ controller }: Props) {
  const [view, setView] = useState<DemoView>(() => controller.view());
  const [impulse, setImpulse] = useState("2.5");
  const [nativeReport, setNativeReport] = useState("No native scaling report loaded");
  const [error, setError] = useState("");
  const canvas = useRef<HTMLCanvasElement>(null);

  useEffect(() => controller.subscribe(setView), [controller]);
  useEffect(() => {
    const element = canvas.current;
    if (element === null) return;
    const scene = new DemoScene(element);
    let frame = 0;
    const draw = (timestamp: number) => {
      const alpha = controller.advance(timestamp);
      scene.render(controller.view(), alpha);
      frame = requestAnimationFrame(draw);
    };
    frame = requestAnimationFrame(draw);
    return () => { cancelAnimationFrame(frame); scene.dispose(); };
  }, [controller]);

  const act = (action: () => void) => {
    try { action(); setError(""); } catch (caught: unknown) { setError(caught instanceof Error ? caught.message : String(caught)); }
  };

  const loadNativeReport = async (file: File | undefined) => {
    if (file === undefined) return;
    const text = await file.text();
    setNativeReport(`${file.name}: ${text.slice(0, 800)}`);
  };

  return <div className="app-shell">
    <header className="topbar">
      <div><span className="eyebrow">Gravity / deterministic 3D</span><h1>{view.title}</h1><p>{view.description}</p></div>
      <div className={`determinism ${view.deterministic ? "ok" : "bad"}`}><span>Determinism</span><strong>{view.deterministic ? "LOCKED" : "MISMATCH"}</strong></div>
    </header>
    <aside className="sidebar" aria-label="Simulation controls">
      <label className="field"><span>Classic case</span><select aria-label="Classic case" value={view.caseId} onChange={event => act(() => controller.selectCase(event.target.value))}>{CASES.map(item => <option key={item.id} value={item.id}>{item.title}</option>)}</select></label>
      <div className="button-row">
        <button type="button" onClick={() => controller.togglePaused()}>{view.paused ? "Run" : "Pause"}</button>
        <button type="button" onClick={() => act(() => controller.singleStep())}>Step</button>
        <button type="button" onClick={() => act(() => controller.reset())}>Reset</button>
      </div>
      <label className="field"><span>Impulse (canonical decimal)</span><input aria-label="Impulse" value={impulse} pattern="-?[0-9]+(?:\.[0-9]+)?" onChange={event => setImpulse(event.target.value)} /></label>
      <button className="wide" type="button" onClick={() => act(() => controller.applyImpulse(impulse))}>Queue impulse</button>
      <button className="wide secondary" type="button" onClick={() => act(() => controller.injectLateInput())}>Inject late input + replay</button>
      <section className="case-index"><h2>Case matrix</h2>{[...new Set(CASES.map(item => item.category))].map(category => <div key={category}><span>{category}</span><b>{CASES.filter(item => item.category === category).length}</b></div>)}</section>
      <label className="file-field"><span>Native worker report</span><input aria-label="Native worker report" type="file" accept=".json,.csv,.txt,.md" onChange={event => void loadNativeReport(event.target.files?.[0])} /></label>
      <p className="native-report">{nativeReport}</p>
      {error === "" ? null : <p role="alert" className="error">{error}</p>}
    </aside>
    <main className="viewport" aria-label="Three.js physics viewport"><canvas ref={canvas} data-testid="viewport" /><div className="viewport-badge"><span>{view.paused ? "PAUSED" : "LIVE"}</span><b>Tick {view.tick}</b></div></main>
    <aside className="diagnostics" aria-label="Diagnostics">
      <section><span className="eyebrow">Canonical state</span><div className="hash" data-testid="world-hash">{view.hash}</div><p>{view.expectedStatus}</p></section>
      <section className="metrics">
        <Metric label="Bodies" value={view.stats.bodyCount} /><Metric label="Awake" value={view.stats.awakeBodyCount} /><Metric label="Contacts" value={view.stats.contactCount} /><Metric label="Joints" value={view.stats.jointCount} /><Metric label="Pairs" value={view.stats.broadPairCount} /><Metric label="Events" value={view.stats.eventCount} /><Metric label="Workers" value={view.stats.workerCount} /><Metric label="WASM pages" value={view.memoryPages} />
      </section>
      <section><div className="section-heading"><h2>Phase visits</h2><span>{view.stepMilliseconds.toFixed(3)} ms total</span></div><div className="phases">{view.phaseVisits.map(phase => <div key={phase.name}><span>{phase.name}</span><i style={{ width: `${phase.visits * 18}px` }} /><b>{phase.visits}</b></div>)}</div></section>
      <section><h2>Rollback timeline</h2><p data-testid="rollback-status">{view.rollbackStatus}</p><div className="timeline"><i /><i /><i className="active" /><i /><i /></div></section>
      <section><h2>Queries</h2><p>{view.queryDebug === undefined ? "Select the query case for hit visualization." : `${view.queryDebug.hits.length} canonical hits rendered.`}</p></section>
      <section><h2>Dual World</h2><div className="hash subtle">{view.mirrorHash === "" ? "Single authoritative World" : view.mirrorHash}</div></section>
    </aside>
  </div>;
}

function Metric({ label, value }: Readonly<{ label: string; value: number }>) {
  return <div><span>{label}</span><b>{value}</b></div>;
}
