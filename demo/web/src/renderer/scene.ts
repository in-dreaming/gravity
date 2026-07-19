import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { BodyState, GravityId } from "../wasm/gravity";
import { rawToNumber } from "../physics/fixed";
import type { DemoView } from "../physics/controller";
import type { Visual } from "../cases/types";

export class DemoScene {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(48, 1, 0.05, 500);
  private readonly controls: OrbitControls;
  private readonly objects = new Map<GravityId, THREE.Object3D>();
  private readonly debug = new THREE.Group();
  private caseId = "";
  private resizeObserver: ResizeObserver;

  constructor(private readonly canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(globalThis.devicePixelRatio > 2 ? 2 : globalThis.devicePixelRatio);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.scene.background = new THREE.Color(0x07111e);
    this.scene.fog = new THREE.Fog(0x07111e, 35, 95);
    this.camera.position.set(14, 10, 18);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 3, 0);
    const hemisphere = new THREE.HemisphereLight(0xb9e3ff, 0x17202c, 1.8);
    this.scene.add(hemisphere);
    const key = new THREE.DirectionalLight(0xffffff, 3.2);
    key.position.set(8, 16, 10);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    this.scene.add(key);
    const grid = new THREE.GridHelper(48, 48, 0x2b7da0, 0x173044);
    grid.position.y = 0.01;
    this.scene.add(grid, this.debug);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas.parentElement ?? canvas);
    this.resize();
  }

  render(view: DemoView, alpha: number): void {
    if (view.caseId !== this.caseId) this.rebuild(view);
    const previous = new Map(view.previousBodies.map(state => [state.id, state]));
    const current = new Map(view.bodies.map(state => [state.id, state]));
    for (const [id, object] of this.objects) {
      const next = current.get(id);
      if (next === undefined) { object.visible = false; continue; }
      object.visible = true;
      const prior = previous.get(id) ?? next;
      this.applyInterpolated(object, prior, next, alpha);
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose(): void {
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.clearObjects();
    this.clearGroup(this.debug);
    this.renderer.dispose();
  }

  private rebuild(view: DemoView): void {
    this.caseId = view.caseId;
    this.clearObjects();
    this.clearGroup(this.debug);
    for (const entry of view.visuals) {
      const object = this.createVisual(entry);
      this.objects.set(entry.body, object);
      this.scene.add(object);
    }
    for (const line of view.queryDebug?.lines ?? []) {
      const geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...line.from), new THREE.Vector3(...line.to)]);
      this.debug.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: line.color })));
    }
    for (const point of view.queryDebug?.points ?? []) {
      const marker = new THREE.Mesh(new THREE.SphereGeometry(0.12, 12, 8), new THREE.MeshBasicMaterial({ color: point.color }));
      marker.position.set(...point.at);
      this.debug.add(marker);
    }
    this.controls.target.set(...view.focus);
    this.controls.update();
  }

  private createVisual(entry: Visual): THREE.Object3D {
    const material = new THREE.MeshStandardMaterial({ color: entry.color, roughness: 0.58, metalness: 0.12, wireframe: entry.wireframe ?? false });
    let object: THREE.Object3D;
    if (entry.kind === "compound") {
      const group = new THREE.Group();
      for (const x of [-0.8, 0.8]) {
        const child = new THREE.Mesh(new THREE.TetrahedronGeometry(0.85, 0), material.clone());
        child.position.x = x;
        child.castShadow = true;
        group.add(child);
      }
      object = group;
    } else {
      const geometry = this.geometry(entry);
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      object = mesh;
    }
    object.scale.set(entry.size[0], entry.size[1], entry.size[2]);
    return object;
  }

  private geometry(entry: Visual): THREE.BufferGeometry {
    switch (entry.kind) {
      case "sphere": return new THREE.SphereGeometry(0.5, 24, 16);
      case "box": return new THREE.BoxGeometry(1, 1, 1);
      case "capsule": return new THREE.CapsuleGeometry(0.5, 1, 12, 20);
      case "hull": return new THREE.TetrahedronGeometry(0.65, 0);
      case "mesh": return new THREE.TetrahedronGeometry(0.65, 0);
      case "height": return new THREE.BoxGeometry(1, 0.12, 1, 5, 1, 5);
      case "compound": return new THREE.BoxGeometry(1, 1, 1);
    }
  }

  private applyInterpolated(object: THREE.Object3D, previous: BodyState, current: BodyState, alpha: number): void {
    const p0 = previous.transform.position;
    const p1 = current.transform.position;
    object.position.lerpVectors(new THREE.Vector3(rawToNumber(p0.x), rawToNumber(p0.y), rawToNumber(p0.z)), new THREE.Vector3(rawToNumber(p1.x), rawToNumber(p1.y), rawToNumber(p1.z)), alpha);
    const q0 = previous.transform.orientation;
    const q1 = current.transform.orientation;
    object.quaternion.slerpQuaternions(new THREE.Quaternion(rawToNumber(q0.x), rawToNumber(q0.y), rawToNumber(q0.z), rawToNumber(q0.w)), new THREE.Quaternion(rawToNumber(q1.x), rawToNumber(q1.y), rawToNumber(q1.z), rawToNumber(q1.w)), alpha);
  }

  private clearObjects(): void {
    for (const object of this.objects.values()) {
      this.scene.remove(object);
      object.traverse(child => {
        if (child instanceof THREE.Mesh) {
          child.geometry.dispose();
          if (Array.isArray(child.material)) for (const material of child.material) material.dispose(); else child.material.dispose();
        }
      });
    }
    this.objects.clear();
  }

  private clearGroup(group: THREE.Group): void {
    for (const child of [...group.children]) {
      group.remove(child);
      if (child instanceof THREE.Line || child instanceof THREE.Mesh) {
        child.geometry.dispose();
        if (Array.isArray(child.material)) for (const material of child.material) material.dispose(); else child.material.dispose();
      }
    }
  }

  private resize(): void {
    const parent = this.canvas.parentElement;
    const width = parent?.clientWidth ?? 1;
    const height = parent?.clientHeight ?? 1;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }
}
