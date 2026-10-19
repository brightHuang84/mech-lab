import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { DEFS, MATS, defaultMass, pitchRadius } from './parts.js';
import { RAPIER, Sim, analyze, planCoupling, V3, Q4, ptW, axW, ptL, axL, isForce } from './physics.js';
import { PRESETS } from './presets.js';
import { findAssembly, connectors, SNAP_PX, JNAME } from './assemble.js';
import { resizeModel, maxJointGap, anchorRef } from './resize.js';
import { installExtra, keyLabel } from './extra.js';
import { GEO } from './parts.js';
import { installAsm } from './asmpanel.js';
import { installMove } from './move.js';
import { geoMinY, groundFixModel, GROUND_TOL, GROUND_CLEAR } from './ground.js';

const $ = id => document.getElementById(id);
const tv3 = v => new THREE.Vector3(v.x, v.y, v.z), tq4 = q => new THREE.Quaternion(q.x, q.y, q.z, q.w);
/** lowest world y of a part's actual geometry (mesh vertices; analytic collision shapes as fallback) */
function partMinY(P) { return geoMinY(P, null, null, shapeMinY); }
/** lowest world y of a part's collision shapes (analytic) */
function shapeMinY(P) {
  const q = Q4(P.q), p = V3(P.p); let lo = Infinity;
  for (const s of DEFS[P.type].shapes(P.d)) {
    const c = V3(s.pos || [0, 0, 0]).applyQuaternion(q).add(p); let e = 0;
    if (s.k === 'box') { for (let k = 0; k < 3; k++) { const ax = new THREE.Vector3(k === 0 ? 1 : 0, k === 1 ? 1 : 0, k === 2 ? 1 : 0).applyQuaternion(q); e += Math.abs(ax.y) * s.h[k]; } }
    else if (s.k === 'cyl' || s.k === 'cap') { const ay = new THREE.Vector3(0, 0, 1).applyQuaternion(q).y; e = s.hh * Math.abs(ay) + s.r * Math.sqrt(Math.max(0, 1 - ay * ay)) + (s.k === 'cap' ? s.r : 0); }
    else if (s.k === 'ball') e = s.r;
    else if (s.pts) { for (let i = 0; i < s.pts.length; i += 3) lo = Math.min(lo, V3([s.pts[i], s.pts[i + 1], s.pts[i + 2]]).applyQuaternion(q).add(p).y); continue; }
    lo = Math.min(lo, c.y - e);
  }
  return lo;
}
const D2R = Math.PI / 180;
const JT = { revolute: '铰链', prismatic: '滑动副', fixed: '固定连接', spherical: '球铰', helical: '螺旋副', spring: '弹簧', damper: '阻尼器' };
const EDIT_T = ['revolute', 'prismatic', 'fixed', 'spherical', 'helical'];
const DRIVE_N = { speed: '速度', torque: '扭矩', servo: '位置(舵机)' };
const CT = { gear: '啮合', belt: '皮带', chain: '链条', screw: '螺旋副', gearbox: '变速箱联动', clutch: '离合器', ujoint: '万向节联动' };
const TOOLS = [
  ['select', '选择/移动', 'V'], ['revolute', '铰链', '旋转副'], ['prismatic', '滑动副', '移动副'], ['fixed', '固定连接', '焊接'], ['spherical', '球铰', '万向'],
  ['spring', '弹簧', 'k'], ['damper', '阻尼器', 'c'], ['gear', '啮合', '齿轮/齿条/蜗杆/锥齿'], ['belt', '皮带', '带轮'], ['chain', '链条', '链轮'], ['screw', '螺旋副', '丝杠-螺母'],
  ['gearbox', '变速箱联动', '比例'], ['clutch', '离合器', '可分离'], ['ujoint', '万向节联动', '1:1'],
];

class App {
  constructor() {
    this.M = { parts: [], joints: [], couplings: [], nid: 1 }; this.DEFS = DEFS; this._connectors = connectors; this._fa = findAssembly;
    this.mode = 'edit'; this.tool = 'select'; this.sel = null; this.undoS = []; this.redoS = []; this.init = null;
    this.speed = 1; this.gravity = true; this.snap = true; this.auto = true; this.objs = {}; this.sim = null; this.pick1 = null; this.armed = null;
    this.initThree(); this.initUI(); this.rebuild(); this.init = this.snapshot();
    this.clock = performance.now(); this.acc = 0; this.plotData = [];
    const loop = () => { requestAnimationFrame(loop); this.frame(); }; loop();
    this.msg('就绪：从左侧拖入零件；Ctrl/Shift+点击选中两个零件 → 右侧“装配”面板选连接点和连接类型 → 确定装配（J）；也可在“示例”中载入装配');
  }
  // ---------------- three ----------------
  initThree() {
    const cv = $('cv');
    const R = this.renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true, preserveDrawingBuffer: true });
    R.setPixelRatio(Math.min(2, window.devicePixelRatio || 1)); R.shadowMap.enabled = true; R.shadowMap.type = THREE.PCFSoftShadowMap;
    R.outputColorSpace = THREE.SRGBColorSpace; R.toneMapping = THREE.ACESFilmicToneMapping; R.toneMappingExposure = 1.05;
    const S = this.scene = new THREE.Scene(); S.background = new THREE.Color(0xdfe6ee); S.fog = new THREE.Fog(0xdfe6ee, 8, 25);
    this.cam = new THREE.PerspectiveCamera(42, 1, 0.01, 100); this.cam.position.set(1.2, 1.0, 1.8);
    S.add(new THREE.HemisphereLight(0xffffff, 0x8a94a6, 1.1));
    const dl = this.sun = new THREE.DirectionalLight(0xffffff, 2.2); dl.position.set(2.5, 4, 2.5); dl.castShadow = true;
    dl.shadow.mapSize.set(2048, 2048); Object.assign(dl.shadow.camera, { left: -2.5, right: 2.5, top: 2.5, bottom: -2.5, near: 0.5, far: 12 }); dl.shadow.bias = -0.0005; S.add(dl);
    const gnd = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: 0xc8d0da, roughness: 0.95 })); gnd.rotation.x = -Math.PI / 2; gnd.receiveShadow = true; gnd.userData.ground = true; S.add(gnd); this.groundMesh = gnd;
    const g1 = new THREE.GridHelper(10, 100, 0x94a3b8, 0xaab4c2); g1.position.y = 0.0005; g1.material.transparent = true; g1.material.opacity = 0.5; S.add(g1);
    const g2 = new THREE.GridHelper(10, 10, 0x64748b, 0x64748b); g2.position.y = 0.001; S.add(g2);
    S.add(new THREE.AxesHelper(0.15));
    this.root = new THREE.Group(); S.add(this.root); this.fx = new THREE.Group(); S.add(this.fx);
    const oc = this.orbit = new OrbitControls(this.cam, cv); oc.target.set(0.1, 0.3, 0); oc.enableDamping = true; oc.dampingFactor = 0.12; oc.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.ROTATE, RIGHT: THREE.MOUSE.PAN }; oc.touches = { ONE: null, TWO: THREE.TOUCH.DOLLY_ROTATE }; cv.addEventListener('mousedown', e => { if (e.button === 1) e.preventDefault(); }); cv.addEventListener('auxclick', e => { if (e.button === 1) e.preventDefault(); }); oc.update();
    const tc = this.tc = new TransformControls(this.cam, cv); tc.setSize(0.8);
    this.proxy = new THREE.Object3D(); S.add(this.proxy);
    S.add(tc.getHelper ? tc.getHelper() : tc);
    tc.addEventListener('dragging-changed', e => { oc.enabled = !e.value; if (e.value) this.dragStart(); else this.dragEnd(); });
    tc.addEventListener('objectChange', () => { if (this.drag) this.dragMove(this.proxy.position.clone(), this.proxy.quaternion.clone()); });
    this.applySnap();
    this.ray = new THREE.Raycaster();
    const ro = new ResizeObserver(() => this.resize()); ro.observe($('center')); this.resize();
    this.trace = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 4000), 3)), new THREE.LineBasicMaterial({ color: 0xdc2626 }));
    this.trace.frustumCulled = false; this.trace.geometry.setDrawRange(0, 0); S.add(this.trace); this.tracePts = [];
    this.arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.1, 0x16a34a, 0.03, 0.02); this.arrow.visible = false; S.add(this.arrow);
    this.hl = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), new THREE.MeshBasicMaterial({ color: 0xfacc15, depthTest: false })); this.hl.renderOrder = 10; this.hl.visible = false; S.add(this.hl);
    this.hl2 = this.hl.clone(); this.hl2.material = new THREE.MeshBasicMaterial({ color: 0xf97316, depthTest: false }); S.add(this.hl2);
    const cvs = cv;
    let down = null;
    cvs.addEventListener('pointerdown', e => this.ptrDown(e), { capture: true });
    window.addEventListener('pointermove', e => this.ptrMove(e)); window.addEventListener('pointerup', e => this.ptrUp(e));
    cvs.addEventListener('pointerdown', e => { down = { x: e.clientX, y: e.clientY, b: e.button }; });
    cvs.addEventListener('pointerup', e => { if (down && Math.hypot(e.clientX - down.x, e.clientY - down.y) < 5 && down.b === 0 && !this.tcDragged) this.click(e); down = null; this.tcDragged = false; });
    cvs.addEventListener('pointermove', e => this.hover(e));
    cvs.addEventListener('contextmenu', e => e.preventDefault());
    let rdown = null; cvs.addEventListener('pointerdown', e => { if (e.button === 2) rdown = { x: e.clientX, y: e.clientY }; }, { capture: true });
    window.addEventListener('pointerup', e => { if (e.button === 2 && rdown && Math.hypot(e.clientX - rdown.x, e.clientY - rdown.y) < 5) this.ctxMenu(e); if (e.button === 2) rdown = null; }, { capture: true });
    cvs.addEventListener('pointerdown', e => { if (e.button !== 2) this.hideCtx(); });
    cvs.addEventListener('wheel', e => this.wheel(e), { capture: true, passive: false });
    window.addEventListener('pointermove', e => { this._cursor = { x: e.clientX, y: e.clientY }; });
    window.addEventListener('keydown', e => { if (e.key === 'Alt') { this.altDown = true; e.preventDefault(); } if (e.key === 'Tab' && this.dragging()) { e.preventDefault(); e.stopPropagation(); this.cycleSnapType(e.shiftKey ? -1 : 1); } }, { capture: true });
    window.addEventListener('keyup', e => { if (e.key === 'Alt') this.altDown = false; });
    window.addEventListener('blur', () => this.altDown = false);
  }
  applySnap() { this.tc.setTranslationSnap(this.snap ? 0.005 : null); this.tc.setRotationSnap(this.snap ? 15 * D2R : null); }
  resize() { const c = $('center'), w = c.clientWidth, h = c.clientHeight; this.renderer.setSize(w, h, false); this.cam.aspect = w / Math.max(1, h); this.cam.updateProjectionMatrix(); }
  ndc(e) { const r = $('cv').getBoundingClientRect(); return new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1); }
  pickAt(e, o = {}) {
    this.scene.updateMatrixWorld(); this.ray.setFromCamera(this.ndc(e), this.cam);
    if (this.mode === 'edit' && !o.parts) for (const h of this.ray.intersectObjects([this.fx], true)) { let o = h.object; while (o && o.userData.jid == null) o = o.parent; if (o) { const j = this.M.joints.find(x => x.id === o.userData.jid); if (j && !isForce(j)) return { jid: j.id, point: h.point }; } }
    const hits = this.ray.intersectObjects([this.root, this.fx, this.groundMesh], true);
    for (const h of hits) { let o = h.object; while (o && o.userData.pid == null && o.userData.jid == null && !o.userData.ground) o = o.parent; if (!o) continue; if (o.userData.helper) continue; if (o.userData.pid && (this.part(o.userData.pid) || {}).hidden) continue; let normal = null; if (h.face && o.userData.pid) { normal = h.face.normal.clone().applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(h.object.matrixWorld)).normalize(); if (normal.dot(this.ray.ray.direction) > 0) normal.negate(); } return { pid: o.userData.pid, jid: o.userData.jid, ground: !!o.userData.ground, point: h.point, normal }; }
    return { ground: true, point: this.groundPoint(e) };
  }
  groundPoint(e) { this.ray.setFromCamera(this.ndc(e), this.cam); const p = new THREE.Vector3(); return this.ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), p) ? p : new THREE.Vector3(); }
  // ---------------- model helpers ----------------
  part(id) { return this.M.parts.find(p => p.id === id); }
  byName(n) { return this.M.parts.find(p => p.name === n); }
  newPart(type, pos, rot, over = {}) {
    const D = DEFS[type], id = 'p' + (this.M.nid++);
    const P = { id, type, name: this.autoName(type), p: [0, 0, 0], q: [0, 0, 0, 1], fixed: !!D.fixed, mat: D.mat, d: JSON.parse(JSON.stringify(D.d)), mass: 0 };
    if (D.motor) P.motor = Object.assign({}, D.motor);
    const o = Object.assign({}, over); if (o.d) { Object.assign(P.d, o.d); delete o.d; } if (o.motor) { P.motor = Object.assign(P.motor || {}, o.motor); delete o.motor; }
    Object.assign(P, o); P.mass = over.mass != null ? over.mass : defaultMass(P); if (over.mass != null) P.massManual = true;
    P.p = pos.slice ? [pos[0], pos[1], pos[2]] : [pos.x, pos.y, pos.z];
    if (rot && rot.length === 4) P.q = rot.slice(); else if (rot) P.q = new THREE.Quaternion().setFromEuler(new THREE.Euler(rot[0] * D2R, rot[1] * D2R, rot[2] * D2R)).toArray();
    this.M.parts.push(P); return P;
  }
  shortLabel(type) { return DEFS[type].label.split(/[\/(（]/)[0]; }
  autoName(type) { const L = this.shortLabel(type), re = new RegExp('^' + L.replace(/[.*+?^${}()|[\]\\]/g, '\\$&') + ' (\\d+)$'); let n = 0; for (const P of this.M.parts) { const m = re.exec(P.name || ''); if (m) n = Math.max(n, +m[1]); } return L + ' ' + (n + 1); }
  addJoint(type, A, B, anchorW, axisW, over = {}) {
    const W = V3(anchorW.toArray ? anchorW.toArray() : anchorW), n = V3(axisW.toArray ? axisW.toArray() : axisW).normalize();
    if (!isForce({ type }) && this.findDupJoint) { const d = this.findDupJoint(type, A, B, W); if (d) { this._dupJoint = d.id; return d; } } // never stack an identical joint twice
    const j = Object.assign({ id: 'j' + (this.M.nid++), type, a: A.id, b: B ? B.id : null, la: ptL(A, W).toArray(), lb: B ? ptL(B, W).toArray() : W.toArray(), axa: axL(A, n).toArray() }, over);
    if (type === 'spring') Object.assign(j, { k: 300, c: 0.5 }, over); if (type === 'damper') Object.assign(j, { c: 20 }, over);
    // parametric anchor reference (connector name + position along it) so a later resize can recompute the anchor (resize.js)
    if (!isForce(j) && !j.ref) { const ra = anchorRef(A, j.la), rb = B ? anchorRef(B, j.lb) : null; if (ra || rb) { j.ref = {}; if (ra) j.ref.a = ra; if (rb) j.ref.b = rb; } }
    this.M.joints.push(j); return j;
  }
  addCoupling(type, A, B, extra = {}) {
    const r = planCoupling(this.M, type, A, B, extra);
    if (typeof r === 'string') { this.msg('无法建立' + CT[type] + '：' + r, true); return null; }
    const c = Object.assign({ id: 'c' + (this.M.nid++), type, pa: A.id, pb: B.id, ratio: extra.ratio || 1, engaged: true, maxT: 20, reverse: false }, r, extra);
    this.M.couplings.push(c); return c;
  }
  snapshot() { return JSON.stringify(this.M); }
  commit() { if (this.mode === 'edit' && this.pruneAutoHinges) this.pruneAutoHinges(); if (this.mode === 'edit') this.groundFix(); this.undoS.push(this.lastSnap || this.snapshot()); if (this.undoS.length > 200) this.undoS.shift(); this.redoS = []; this.lastSnap = this.snapshot(); if (this.mode === 'edit') this.init = this.lastSnap; this.updButtons(); }
  beginEdit() { if (this.asm && !this._asmBusy) this.asmCancel(true); if (this.mode !== 'edit') this.reset(); this.lastSnap = this.snapshot(); }
  undo() { if (this.asm) return this.asmCancel(); if (this.mode !== 'edit') this.reset(); if (!this.undoS.length) return; this.redoS.push(this.snapshot()); this.M = JSON.parse(this.undoS.pop()); this.groundFix({ quiet: true }); this.lastSnap = this.snapshot(); this.init = this.lastSnap; this.sel = null; this.msel = new Set(); this.rebuild(); this.msg('已撤销'); }
  redo() { if (this.asm) this.asmCancel(true); if (this.mode !== 'edit') this.reset(); if (!this.redoS.length) return; this.undoS.push(this.snapshot()); this.M = JSON.parse(this.redoS.pop()); this.groundFix({ quiet: true }); this.lastSnap = this.snapshot(); this.init = this.lastSnap; this.sel = null; this.msel = new Set(); this.rebuild(); this.msg('已重做'); }
  // ---------------- scene build ----------------
  rebuild(keepProps) {
    this.groundNet('rebuild');
    this.root.clear(); this.fx.clear(); this.objs = {}; this.jobjs = {}; this.cobjs = {};
    for (const P of this.M.parts) {
      const g = DEFS[P.type].mesh(P.d, P); g.userData.pid = P.id; g.visible = !P.hidden; this.root.add(g); this.objs[P.id] = g; this.decorate(P, g);
      g.traverse(o => { if (o.isMesh) o.userData.baseEm = o.material.emissive ? o.material.emissive.getHex() : 0; });
    }
    for (const j of this.M.joints) { const o = this.jointMarker(j); if (o) { o.userData.jid = j.id; this.fx.add(o); this.jobjs[j.id] = o; } }
    for (const c of this.M.couplings) if (c.type === 'belt' || c.type === 'chain') { const o = { c, s: 0 }; this.cobjs[c.id] = o; }
    this.syncPoses(); this.updSel(); if (!keepProps) this.updProps(); this.updButtons(); this.updStatus(); this.showPoints(); this.renderOutliner(); if (this.asmPend && !this.asm && this.asmSyncSel) this.asmSyncSel();
  }
  jointMarker(j) {
    const g = new THREE.Group(), col = { revolute: 0xf97316, prismatic: 0x10b981, fixed: 0x6366f1, spherical: 0xec4899, helical: 0xa855f7, spring: 0x0ea5e9, damper: 0x334155 }[j.type];
    const m = new THREE.MeshStandardMaterial({ color: col, emissive: col, emissiveIntensity: 0.25, roughness: 0.5 });
    if (!isForce(j)) { m.depthTest = false; m.transparent = true; m.opacity = 0.9; g.renderOrder = 8; }
    if (j.type === 'revolute' || j.type === 'helical') { const c = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.05, 16), m); g.add(c); const r = new THREE.Mesh(new THREE.TorusGeometry(0.012, 0.0025, 8, 24), m); r.rotation.x = Math.PI / 2; g.add(r); }
    else if (j.type === 'prismatic') { const c = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.12, 8), m); g.add(c); for (const s of [-1, 1]) { const k = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.02, 12), m); k.position.y = s * 0.06; if (s < 0) k.rotation.x = Math.PI; g.add(k); } }
    else if (j.type === 'fixed') g.add(new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.014, 0.014), m));
    else if (j.type === 'spherical') g.add(new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), m));
    else if (j.type === 'spring') {
      class H extends THREE.Curve { getPoint(u, t = new THREE.Vector3()) { const a = u * 12 * Math.PI * 2, w = u < 0.06 || u > 0.94 ? 0 : 1; return t.set(0.018 * Math.cos(a) * w, u, 0.018 * Math.sin(a) * w); } }
      const s = new THREE.Mesh(new THREE.TubeGeometry(new H(), 400, 0.0022, 6, false), new THREE.MeshStandardMaterial({ color: 0x0ea5e9, metalness: 0.6, roughness: 0.3 })); s.castShadow = true; g.add(s); g.userData.stretch = s;
    } else if (j.type === 'damper') {
      const body = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 1, 20), m); body.castShadow = true; const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 1, 12), new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.8, roughness: 0.2 }));
      g.add(body); g.add(rod); g.userData.damper = { body, rod };
    }
    if (!isForce(j)) g.traverse(o => { if (o.isMesh) o.renderOrder = 8; });
    return g;
  }
  livePose(id) {
    const L = this.mode === 'edit' && this._lift && this._lift[id];
    if (this.dragPoses && this.dragPoses[id]) { const d = this.dragPoses[id]; return L ? { p: d.p.clone().setY(d.p.y + L), q: d.q } : d; }
    if (this.mode !== 'edit' && this.sim) return this.sim.partPose(id);
    const P = this.part(id); if (!P) return null; const o = { p: V3(P.p), q: Q4(P.q) }; if (L) o.p.y += L; return o;
  }
  syncPoses() {
    this.computeLiveLift();
    for (const P of this.M.parts) { const o = this.objs[P.id], ps = this.livePose(P.id); if (o && ps) { o.position.copy(ps.p); o.quaternion.copy(ps.q); } }
    const X = new THREE.Vector3(), Yv = new THREE.Vector3(0, 1, 0);
    const wpt = (pid, l) => { const ps = this.livePose(pid); return V3(l).applyQuaternion(ps.q).add(ps.p); };
    for (const j of this.M.joints) {
      const o = this.jobjs[j.id]; if (!o || !this.part(j.a)) continue;
      const pa = wpt(j.a, j.la), ps = this.livePose(j.a);
      if (isForce(j)) {
        const pb = j.b ? wpt(j.b, j.lb) : V3(j.lb), d = pb.clone().sub(pa), L = Math.max(1e-4, d.length());
        o.position.copy(pa); o.quaternion.setFromUnitVectors(Yv, d.clone().normalize());
        if (o.userData.stretch) o.userData.stretch.scale.set(1, L, 1);
        if (o.userData.damper) { const { body, rod } = o.userData.damper; body.scale.set(1, L * 0.55, 1); body.position.y = L * 0.275; rod.scale.set(1, L * 0.6, 1); rod.position.y = L * 0.7; }
      } else { o.position.copy(pa); X.copy(V3(j.axa)).applyQuaternion(ps.q); o.quaternion.setFromUnitVectors(Yv, X.normalize()); }
    }
    this.updLoops();
  }
  updLoops() {
    for (const id in this.cobjs) {
      const L = this.cobjs[id], c = L.c, A = this.part(c.pa), B = this.part(c.pb); if (!A || !B) continue;
      const pa = this.livePose(A.id), pb = this.livePose(B.id), n = new THREE.Vector3(0, 0, 1).applyQuaternion(pa.q);
      const rA = pitchRadius(A) + (c.type === 'belt' ? 0.002 : 0), rB = pitchRadius(B) + (c.type === 'belt' ? 0.002 : 0);
      const key = [pa.p.toArray(), pb.p.toArray(), n.toArray(), rA, rB].flat().map(v => v.toFixed(5)).join();
      if (key !== L.key) {
        L.key = key; if (L.mesh) { this.fx.remove(L.mesh); L.mesh.geometry.dispose(); } if (L.inst) this.fx.remove(L.inst);
        const u = pb.p.clone().sub(pa.p); u.addScaledVector(n, -u.dot(n)); const D = u.length(); if (D < rA + rB) continue; u.normalize(); const v = n.clone().cross(u);
        const phi = Math.acos(Math.max(-1, Math.min(1, (rA - rB) / D))), pts = [];
        const P2 = (c0, r, a) => c0.clone().addScaledVector(u, r * Math.cos(a)).addScaledVector(v, r * Math.sin(a));
        const cB = pa.p.clone().addScaledVector(u, D).addScaledVector(n, 0);
        const N = 64; for (let i = 0; i <= N; i++) pts.push(P2(pa.p, rA, phi + (2 * Math.PI - 2 * phi) * i / N));
        for (let i = 0; i <= N; i++) pts.push(P2(cB, rB, -phi + 2 * phi * i / N));
        // resample path by arc length
        const seg = []; let tot = 0; for (let i = 0; i < pts.length; i++) { const a = pts[i], b = pts[(i + 1) % pts.length]; const l = a.distanceTo(b); seg.push([a, b, l, tot]); tot += l; }
        L.tot = tot; L.at = s => { s = ((s % tot) + tot) % tot; for (const [a, b, l, t0] of seg) if (s <= t0 + l) { const t = l ? (s - t0) / l : 0; return { p: a.clone().lerp(b, t), d: b.clone().sub(a).normalize() }; } return { p: pts[0].clone(), d: u.clone() }; };
        L.n = n.clone();
        if (c.type === 'belt') {
          const curve = new THREE.CatmullRomCurve3(pts.filter((_, i) => i % 2 === 0), true);
          const sh = new THREE.Shape(); sh.moveTo(-0.0015, -0.009); sh.lineTo(0.0015, -0.009); sh.lineTo(0.0015, 0.009); sh.lineTo(-0.0015, 0.009);
          L.mesh = new THREE.Mesh(new THREE.TubeGeometry(curve, 200, 0.002, 4, true), new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.9 })); L.mesh.scale.set(1, 1, 1);
          L.mesh.castShadow = true; this.fx.add(L.mesh);
          const cnt = Math.floor(tot / 0.02); L.inst = new THREE.InstancedMesh(new THREE.BoxGeometry(0.004, 0.0045, 0.018), new THREE.MeshStandardMaterial({ color: 0x4b5563 }), cnt); L.cnt = cnt; L.pitch = tot / cnt;
        } else {
          const cnt = Math.round(tot / (B.d.p || 0.0127)); L.cnt = cnt; L.pitch = tot / cnt;
          L.inst = new THREE.InstancedMesh(new THREE.BoxGeometry(L.pitch * 1.05, 0.006, 0.006), new THREE.MeshStandardMaterial({ color: 0x52525b, metalness: 0.8, roughness: 0.35 }), cnt);
        }
        L.inst.castShadow = true; this.fx.add(L.inst);
      }
      if (!L.inst || !L.at) continue;
      const m = new THREE.Matrix4(), q = new THREE.Quaternion(), X = new THREE.Vector3(1, 0, 0), Zz = new THREE.Vector3(0, 0, 1), one = new THREE.Vector3(1, 1, 1);
      for (let i = 0; i < L.cnt; i++) { const a = L.at(L.s + i * L.pitch + L.pitch / 2); const b = new THREE.Matrix4().makeBasis(a.d, L.n.clone().cross(a.d), L.n); q.setFromRotationMatrix(b); m.compose(a.p, q, one); L.inst.setMatrixAt(i, m); }
      L.inst.instanceMatrix.needsUpdate = true; void X; void Zz;
    }
  }
  // ---------------- interaction ----------------
  hover(e) {
    if (this.asm && this.mode === 'edit' && !this.dragging()) { this.hl.visible = false; this.asmHover(e); return; }
    if (this.mode !== 'edit' || this.tool === 'select') { if (!this.pick1) this.hl.visible = false; return; }
    const h = this.pickAt(e);
    if (this.tool === 'mate' || this.tool === 'addcon') { if (h.pid) { const r = this.tool === 'mate' ? this.mateRef(h) : null; this.hl.position.copy(r ? ptW(this.part(h.pid), r.p) : h.point); this.hl.scale.setScalar(0.7); this.hl.visible = true; } else this.hl.visible = false; return; }
    if (h.pid) { const pt = this.nearestPt(h.pid, h.point); this.hl.position.copy(pt.w); this.hl.visible = true; } else this.hl.visible = false;
  }
  nearestPt(pid, w) {
    const P = this.part(pid), D = DEFS[P.type]; let best = null;
    for (const [i, pt] of D.pts(P.d).entries()) { const pw = ptW(P, pt.p), d = pw.distanceTo(w); if (!best || d < best.d) best = { d, i, pt, w: pw }; }
    return best;
  }
  click(e) {
    if (this.armed) { const gp = this.groundPoint(e); this.placePart(this.armed, gp); this.armed = null; document.querySelectorAll('.item.armed').forEach(x => x.classList.remove('armed')); return; }
    const mod = e.ctrlKey || e.metaKey || e.shiftKey;
    if (this.asm && this.mode === 'edit' && this.tool === 'select' && !mod) {
      if (performance.now() - (this._asmNoClick || 0) < 250) return; // end of a drag along a shaft / face
      if (this.asmClick(e)) return;
      const ha = this.pickAt(e, { parts: true }), pr = this.asm.pairs[this.asm.act];
      if (ha.pid && pr && (ha.pid === pr.a || ha.pid === pr.b)) { this.asmPickAt(ha); return; }
      if (!ha.pid) { this.msg('装配面板已打开：点击 A / B 零件可选取连接点；“确定装配”(Enter) 应用，“取消”(Esc) 关闭'); return; }
    }
    const h = this.pickAt(e);
    if (this.mode !== 'edit' || this.tool === 'select') {
      if (h.jid) this.select({ j: h.jid });
      else if (h.pid && (e.ctrlKey || e.metaKey || e.shiftKey)) { const ms = new Set(this.selIds()), g = this.groupOf(h.pid); if (ms.has(h.pid)) { g.forEach(id => ms.delete(id)); this.msel = ms; const nx = [...ms][0]; this.select(nx ? { p: nx } : null, true); } else { g.forEach(id => ms.add(id)); this.msel = ms; this.select({ p: h.pid }, true); } this.msg('已选中 ' + this.msel.size + ' 个零件（Ctrl/Shift+点击 增减；Ctrl+G 组合）'); this.asmAuto(); }
      else if (h.pid) { const g = this.groupOf(h.pid); if (g.length > 1) { this.msel = new Set(g); this.select({ p: h.pid }, true); } else this.select({ p: h.pid }); }
      else this.select(null);
      return;
    }
    this.toolClick(h);
  }
  toolClick(h) {
    const t = this.tool, isJ = !!JT[t];
    if (t === 'mate') return this.mateClick(h);
    if (t === 'addcon') return this.conClick(h);
    if (!this.pick1) {
      if (!h.pid) { this.msg('请先点击第一个零件' + (isJ ? '的连接点' : '')); return; }
      const pt = this.nearestPt(h.pid, h.point); this.pick1 = { pid: h.pid, pt, w: pt.w, hit: h.point.clone() };
      this.hl2.position.copy(pt.w); this.hl2.visible = true; this.msg('已选 ' + this.part(h.pid).name + (isJ ? '·' + pt.pt.n : '') + '，再点击第二个零件' + (isJ && t !== 'gear' ? '（点空白地面=连接到大地）' : ''));
      return;
    }
    const A = this.part(this.pick1.pid), B = h.pid ? this.part(h.pid) : null;
    if (B === A) { this.msg('请点击另一个零件', true); return; }
    this.beginEdit();
    if (isJ) this.makeJointBetween(t, A, this.pick1.pt, B, B ? this.nearestPt(B.id, h.point) : null);
    else { if (!B) { this.msg('需要点击第二个零件', true); return; } const c = this.addCoupling(t, A, B, t === 'gearbox' ? { ratio: this.gearboxRatio() } : {}); if (c) { this.commit(); this.rebuild(); this.msg('已建立' + CT[t] + '：' + A.name + ' → ' + B.name + '，传动比 ' + c.k.toFixed(4)); } }
    this.pick1 = null; this.hl2.visible = false;
  }
  gearboxRatio() { const gb = this.M.parts.find(p => p.type === 'gearbox'); return gb ? gb.d.ratio : 3; }
  makeJointBetween(t, A, ptA, B, ptB) {
    const W = ptW(A, ptA.pt.p); let axis = axW(A, ptA.pt.a);
    if (t === 'prismatic') axis = axW(A, DEFS[A.type].slideAxis || (A.type === 'follower' ? [0, 1, 0] : A.type === 'nut' || A.type === 'screw' ? [0, 0, 1] : [1, 0, 0]));
    if (B && ptB && !isForce({ type: t })) {
      const an = analyze(this.M), cB = an.comps[an.compOf[B.id]];
      if (!cB.anchored && an.compOf[B.id] !== an.compOf[A.id]) {
        // move B's whole free assembly so its connection point lands on A's (axes aligned for hinges/sliders)
        let rot = new THREE.Quaternion();
        if (t === 'revolute' || t === 'prismatic') { const aB = axW(B, ptB.pt.a), tgt = aB.dot(axis) >= 0 ? axis : axis.clone().negate(); rot.setFromUnitVectors(aB, tgt); }
        const pivot = ptW(B, ptB.pt.p);
        for (const P of cB.parts) { const p = V3(P.p).sub(pivot).applyQuaternion(rot).add(W); P.p = p.toArray(); P.q = rot.clone().multiply(Q4(P.q)).toArray(); }
      }
    }
    let j;
    if (isForce({ type: t })) { const pb = B ? ptW(B, ptB.pt.p) : this.pick1.hit.clone().setY(0); j = this.addJoint(t, A, null, W, axis); if (B) { j.b = B.id; j.lb = ptL(B, pb).toArray(); } else j.lb = pb.toArray(); j.L0 = +W.distanceTo(pb).toFixed(4); }
    else j = this.addJoint(t, A, B, W, axis);
    this.commit(); this.relaxAll(); this.rebuild(); this.select({ j: j.id });
    this.msg('已添加' + JT[t] + '：' + A.name + ' ↔ ' + (B ? B.name : '大地'));
    return j;
  }
  relaxAll() {
    // close any gaps: relax every dynamic body with joints in the edit world
    const s = new Sim(this.M, { edit: true }); let ch = false; if (s.maxGap() > 2e-4) { for (let i = 0; i < 40 && s.maxGap() > 1e-4; i++) s.relax(10); for (const P of this.M.parts) { const ps = s.partPose(P.id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); } ch = true; }
    s.free();
    if (this.mode === 'edit' && this.groundFix({ quiet: true }).dy > 0) ch = true;
    if (ch) { this.lastSnap = this.snapshot(); this.init = this.lastSnap; }
  }
  placePart(type, gp, over, asm) {
    this.beginEdit();
    const P = this.newPart(type, [0, 0, 0], null, over || {});
    if (asm && asm.q) P.q = asm.q.toArray();
    const g = DEFS[type].mesh(P.d, P); g.quaternion.copy(Q4(P.q)); g.updateMatrixWorld(true); const bb = new THREE.Box3().setFromObject(g);
    const sn = v => this.snap ? Math.round(v / 0.01) * 0.01 : v;
    void bb; P.p = [sn(gp.x), +(-geoMinY(P, [0, 0, 0], P.q, shapeMinY) + GROUND_CLEAR).toFixed(5), sn(gp.z)]; // exact geometry (a Box3 of a rotated wheel floats)
    let res = null;
    if (asm && asm.ray) { const r = findAssembly({ M: this.M, mover: P, p0: null, q0: Q4(P.q), ray: asm.ray, cursor: asm.cursor, exclude: new Set([P.id]), toScreen: v => this.toScreen(v), camPos: this.cam.position }); if (r.best) res = this.applyAssembly(r.best, P, [P.id]); }
    if (res) this.liftComp(P.id);
    this.commit(); this.rebuild(); this.select({ p: P.id }); if (this.noteRecent && type !== 'mesh') this.noteRecent(type);
    if (res) this.announce(res); else this.msg('已放置 ' + P.name + '：拖动零件靠近其它零件的连接点即可自动装配（按住 Alt 自由放置）');
    return P;
  }
  // ---------------- snap-to-assemble ----------------
  toScreen(v) { const r = $('cv').getBoundingClientRect(), p = v.clone().project(this.cam); return new THREE.Vector2((p.x + 1) / 2 * r.width + r.left, (1 - p.y) / 2 * r.height + r.top); }
  autoOn(ev) { return this.auto && !(ev && ev.altKey) && !this.altDown; }
  rayOf(ev) { this.ray.setFromCamera(this.ndc(ev), this.cam); return this.ray.ray.clone(); }
  /** move the moving set so `mover` gets the snapped pose, then create the joints (all inside the current edit step) */
  applyAssembly(best, mover, ids) {
    const p0 = V3(mover.p), q0 = Q4(mover.q).normalize(), dq = best.q.clone().multiply(q0.clone().invert());
    for (const id of ids) { const P = this.part(id); if (!P) continue; if (id === mover.id) { P.p = best.p.toArray(); P.q = best.q.toArray(); continue; } P.p = V3(P.p).sub(p0).applyQuaternion(dq).add(best.p).toArray(); P.q = dq.clone().multiply(Q4(P.q)).normalize().toArray(); }
    const nj = this.M.joints.length, nc = this.M.couplings.length;
    const jt = this.altType(best);
    const r = best.apply(this, jt);
    if (r == null && best.kind === 'gear') return null;
    this.snapAlt = 0;
    return { label: jt && jt !== best.kind ? JNAME[jt] + '（Tab 选择）' : best.label, marker: best.marker.clone(), ids: [mover.id, best.target.id], joints: this.M.joints.slice(nj).map(j => j.id), couplings: this.M.couplings.slice(nc).map(c => c.id), a: mover.name, b: best.target.name };
  }
  announce(res) {
    this.msg('✓ 已自动装配：' + res.a + ' → ' + res.b + '（' + res.label + '）。拖动零件可立即看到运动，按 ▶ 运行 / 试运行 查看效果');
    this.flash = { ids: res.ids, jids: res.joints, t0: performance.now(), marker: res.marker, label: '✓ ' + res.label };
    if (res.joints.length) this.select({ p: res.ids[0] });
  }
  showPreview(best, near, ghost) {
    const lbl = $('snapLbl'), ring = $('snapRing');
    if (ghost !== undefined) { if (this.ghostObj && this.ghostObj !== ghost) { this.scene.remove(this.ghostObj); } this.ghostObj = ghost; if (ghost && !ghost.parent) this.scene.add(ghost); }
    this._pv = [best, near];
    const c = best || (near && near.dist < SNAP_PX * 3 ? near : null);
    if (!c && this.rotInfo && this._cursor) { const cr = $('center').getBoundingClientRect(); lbl.style.display = 'block'; lbl.className = ''; lbl.textContent = this.rotInfo; lbl.style.left = (this._cursor.x - cr.left + 18) + 'px'; lbl.style.top = (this._cursor.y - cr.top - 34) + 'px'; ring.style.display = 'none'; this.hl.visible = false; this.snapTarget = null; return; }
    if (!c) { lbl.style.display = 'none'; ring.style.display = 'none'; this.hl.visible = false; this.snapTarget = null; return; }
    const s = this.toScreen(c.marker), cr = $('center').getBoundingClientRect();
    ring.style.display = 'block'; ring.style.left = (s.x - cr.left - SNAP_PX) + 'px'; ring.style.top = (s.y - cr.top - SNAP_PX) + 'px'; ring.style.width = ring.style.height = 2 * SNAP_PX + 'px';
    ring.className = best ? 'on' : '';
    lbl.style.display = 'block'; lbl.style.left = (s.x - cr.left + SNAP_PX * 0.7) + 'px'; lbl.style.top = (s.y - cr.top - SNAP_PX * 0.9) + 'px';
    lbl.className = best ? 'on' : ''; let txt = best ? '松开 → ' + c.label : '靠近 → ' + c.label;
    if (best && best.alts && best.alts.length > 1) { const cur = this.altType(best); txt = '松开 → ' + (cur === best.kind ? best.label : JNAME[cur]) + '   Tab: ' + best.alts.map(t => t === cur ? '[' + JNAME[t] + ']' : JNAME[t]).join(' / '); }
    if (this.rotInfo) txt += '   ' + this.rotInfo;
    lbl.textContent = txt;
    this.hl.position.copy(c.marker); this.hl.scale.setScalar(best ? 1.6 : 1); this.hl.visible = true; this.snapTarget = best;
  }
  clearPreview() { const had = !!this._palLift; this._palLift = null; this.showPreview(null, null, null); if (had) this.syncPoses(); }
  ghostFor(type, P) {
    if (!this._ghosts) this._ghosts = {};
    const key = type + JSON.stringify(P.d); if (this._ghosts[key]) return this._ghosts[key];
    const g = DEFS[type].mesh(P.d, P); g.traverse(o => { if (o.isMesh) { o.material = o.material.clone(); o.material.transparent = true; o.material.opacity = 0.55; o.material.emissive && o.material.emissive.setHex(0x2563eb); o.material.emissiveIntensity = 0.4; o.castShadow = false; } });
    return (this._ghosts[key] = g);
  }
  /** palette drag preview: returns the snap candidate (if any) */
  previewPalette(type, ev, over) {
    const D = DEFS[type], P = { id: '__new', type, d: Object.assign(JSON.parse(JSON.stringify(D.d)), over && over.d ? over.d : {}), mat: over && over.mat || D.mat, p: [0, 0, 0], q: [0, 0, 0, 1] };
    const g = this.ghostFor(type, P), bb = new THREE.Box3().setFromObject(DEFS[type].mesh(P.d, P)), gp = this.groundPoint(ev);
    let best = null, near = null;
    this._palEv = { type, over, ev: { clientX: ev.clientX, clientY: ev.clientY, altKey: ev.altKey } }; const q0 = this.palRot || new THREE.Quaternion(); P.q = q0.toArray();
    g.quaternion.copy(q0); g.position.set(0, 0, 0); g.updateMatrixWorld(true); const bbr = new THREE.Box3().setFromObject(g);
    if (this.autoOn(ev)) { const r = findAssembly({ M: this.M, mover: P, p0: null, q0, ray: this.rayOf(ev), cursor: new THREE.Vector2(ev.clientX, ev.clientY), exclude: new Set(), toScreen: v => this.toScreen(v), camPos: this.cam.position }); best = r.best; near = r.near; }
    void bb; if (best) { g.position.copy(best.p); g.quaternion.copy(best.q); } else { g.position.set(gp.x, -geoMinY(P, [0, 0, 0], q0.toArray(), shapeMinY) + GROUND_CLEAR, gp.z); g.quaternion.copy(q0); void bbr; }
    // snapped ghost (and the assembly it snaps onto) never below the floor: preview the lift the drop will apply
    this._palLift = null;
    if (best && best.target && this.part(best.target.id)) {
      const an = analyze(this.M), ids = an.comps[an.compOf[best.target.id]].parts.map(p => p.id);
      const lo = Math.min(geoMinY(P, best.p, best.q, shapeMinY), ...ids.map(id => partMinY(this.part(id))));
      if (lo < -GROUND_TOL) { const dy = -lo + GROUND_CLEAR; g.position.y += dy; this._palLift = {}; for (const id of ids) this._palLift[id] = dy; }
    }
    this.showPreview(best, near, g); this.syncPoses();
    return best;
  }
  // ---------------- direct pointer drag of parts ----------------
  ptrDown(e) {
    if (e.button === 1) { this.hideCtx(); return; } // middle button always orbits the camera (OrbitControls), even over a part
    if (e.button !== 0 || this.mode !== 'edit' || this.tool !== 'select' || this.armed || this.drag) return;
    if (this.asm && this.asmPtrDown(e)) return; // 装配 panel open: drag along the chosen shaft / face, never drag A / B away
    if (this.tc.axis) return; // gizmo handle under the cursor
    const h = this.pickAt(e); if (!h.pid) return;
    if (this.part(h.pid).locked) return; // locked: click selects, drag does nothing
    // modifiers are read at press time: Alt = move only this part (detach it), Ctrl/⌘ = drag along the joints (articulate), Shift = vertical
    this.orbit.enabled = false; this.pd = { pid: h.pid, x: e.clientX, y: e.clientY, hit: h.point.clone(), started: false, alt: !!(e.altKey || this.altDown), art: !!(e.ctrlKey || e.metaKey), vert: !!e.shiftKey };
  }
  ptrMove(e) {
    if (this.asmScrub) { this.asmScrubMove(e); return; }
    const D = this.pd; if (!D) return;
    if (!D.started) { if (Math.hypot(e.clientX - D.x, e.clientY - D.y) < 5) return; D.started = true; if (this.asm) this.asmCancel(true); this.directStart(e); }
    this.directMove(e);
  }
  ptrUp(e) { if (this.asmScrub) { this.asmScrubEnd(); return; } const D = this.pd; if (!D) return; this.pd = null; this.orbit.enabled = true; if (D.started) this.directEnd(D); } // (the canvas click handler already ran for this pointerup; setting tcDragged here used to swallow the NEXT click)
  directStart(e) {
    const D = this.pd; this.lastSnap = this.snapshot(); const multi = !D.alt && !!(this.msel && this.msel.size > 1 && this.msel.has(D.pid)); this.select({ p: D.pid }, multi); this.tc.detach(); this.snapAlt = 0; this.rotInfo = ''; this.orbit.enableZoom = false;
    D.start = this.posesOf(); this.dragPoses = {}; D.multi = multi;
    if (D.art) { // Ctrl/⌘+drag: the old constrained drag — pull the part along its hinges / sliders (pull hard to detach)
      const an = analyze(this.M), g = an.groupOf[D.pid], comp = an.comps[an.compOf[D.pid]];
      D.plane = new THREE.Plane().setFromNormalAndCoplanarPoint(this.cam.getWorldDirection(new THREE.Vector3()).negate(), D.hit);
      if (!comp.anchored || g.static) { this.directRigid(comp.parts.map(p => p.id), D.hit); return; }
      const s = D.sim = new Sim(this.M, { edit: true }), body = s.pb[D.pid].body;
      const cb = D.cursor = s.world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(D.hit.x, D.hit.y, D.hit.z));
      const gl = s.toBody(body, D.hit);
      D.spring = s.world.createImpulseJoint(RAPIER.JointData.spring(0, 600, 60, { x: 0, y: 0, z: 0 }, { x: gl.x, y: gl.y, z: gl.z }), cb, body, true);
      D.grabL = gl; D.body = body;
      this.msg('Ctrl+拖动：零件按连接约束运动（绕铰链转动 / 沿导轨滑动）；用力拉开可拆开');
      return;
    }
    let ids;
    if (D.alt) { ids = [D.pid]; const lk = this.lockedIn(ids); if (!lk.length) { D.detached = this.detachJoints(D.pid); if (D.detached) this.rebuild(true); } }
    else ids = this.moveSet(D.pid);
    const lk = this.lockedIn(ids); if (lk.length) { D.blocked = true; this.msg(lk.map(x => x.name).join('、') + ' 已锁定，装配体无法移动（在组件栏点 🔒 解锁，或 Alt+拖动 只移动这个零件）', true); return; }
    D.plane = this.dragPlane(D.hit, D.vert); D.ids = ids;
    this.directRigid(ids, D.hit);
    this.msg(D.alt ? (D.detached ? '已拆开 ' + this.part(D.pid).name + '（移除 ' + D.detached + ' 个连接），只移动这个零件；靠近连接点可重新装配' : '只移动 ' + this.part(D.pid).name + '（Alt）')
      : (ids.length > 1 ? '移动整个装配体（' + ids.length + ' 个零件，连接保持）' : '移动 ' + this.part(D.pid).name) + (D.vert ? ' · 竖直移动' : ' · 沿地面移动（Shift=竖直）') + ' · Alt+拖动=只移动此零件 · Ctrl+拖动=按连接约束拖动 · 方向键微调');
  }
  /** drag plane: horizontal through the grab point (ground-parallel); Shift = vertical plane facing the camera; edge-on views fall back to the view plane */
  dragPlane(hit, vert) {
    const dir = this.cam.getWorldDirection(new THREE.Vector3());
    if (vert) { const n = new THREE.Vector3(dir.x, 0, dir.z); if (n.lengthSq() < 1e-6) n.set(0, 0, 1); return new THREE.Plane().setFromNormalAndCoplanarPoint(n.normalize().negate(), hit); }
    if (Math.abs(dir.y) < 0.2) return new THREE.Plane().setFromNormalAndCoplanarPoint(dir.negate(), hit);
    return new THREE.Plane().setFromNormalAndCoplanarPoint(new THREE.Vector3(0, 1, 0), hit);
  }
  directRigid(ids, grabW) { const D = this.pd || this.dd; D.rigid = ids; D.grabOff = V3(this.part(D.pid).p).sub(grabW); D.q0 = Q4(this.part(D.pid).q); }
  directMove(e) {
    const D = this.pd; if (!D || !D.started || D.blocked) return; D.lastEv = { clientX: e.clientX, clientY: e.clientY, altKey: e.altKey };
    const T = new THREE.Vector3(); if (!this.rayOf(e).intersectPlane(D.plane, T)) return;
    if (D.sim) {
      D.cursor.setNextKinematicTranslation({ x: T.x, y: T.y, z: T.z }); D.sim.relax(6);
      for (const P of this.M.parts) this.dragPoses[P.id] = D.sim.partPose(P.id);
      const gw = D.sim.toWorld(D.body, D.grabL);
      if (gw.distanceTo(T) > 0.12) this.detachDuring(D, T);
    }
    if (D.rigid) {
      const pos = T.clone().add(D.grabOff), s0 = D.start[D.pid]; if (D.vert) { pos.x = s0.p.x; pos.z = s0.p.z; }
      const set = (p, q) => { const dq = q.clone().multiply(s0.q.clone().invert()); for (const id of D.rigid) { const s = D.start[id]; this.dragPoses[id] = { p: s.p.clone().sub(s0.p).applyQuaternion(dq).add(p), q: dq.clone().multiply(s.q) }; } };
      set(pos, D.q0); D.best = null;
      if (this.autoOn(e)) {
        const r = findAssembly({ M: this.M, mover: this.part(D.pid), p0: pos, q0: D.q0, ray: null, cursor: null, exclude: new Set(D.rigid), toScreen: v => this.toScreen(v), camPos: this.cam.position });
        D.best = r.best; if (r.best) set(r.best.p, r.best.q); this.showPreview(r.best, r.near);
      } else this.clearPreview();
    }
    this.syncPoses();
  }
  detachDuring(D, T) {
    const id = D.pid, before = this.M.joints.length + this.M.couplings.length;
    this.M.joints = this.M.joints.filter(j => j.a !== id && j.b !== id); const jids = new Set(this.M.joints.map(j => j.id));
    this.M.couplings = this.M.couplings.filter(c => c.pa !== id && c.pb !== id && jids.has(c.ja) && jids.has(c.jb));
    const ps = D.sim.partPose(id); D.sim.free(); D.sim = null;
    for (const P of this.M.parts) { if (this.dragPoses[P.id] && P.id !== id) { /* keep others at relaxed pose */ P.p = this.dragPoses[P.id].p.toArray(); P.q = this.dragPoses[P.id].q.toArray(); } }
    const P = this.part(id); P.p = ps.p.toArray(); P.q = ps.q.toArray();
    for (const Q of this.M.parts) D.start[Q.id] = { p: V3(Q.p), q: Q4(Q.q) };
    this.dragPoses = {};
    const an = analyze(this.M), comp = an.comps[an.compOf[id]];
    D.rigid = comp.parts.map(p => p.id); D.grabOff = V3(P.p).sub(T); D.q0 = Q4(P.q); D.detached = before - this.M.joints.length - this.M.couplings.length;
    this.rebuild(); this.msg('已拆开 ' + P.name + '（移除 ' + D.detached + ' 个连接）。松开放下，或拖到其它连接点重新装配');
  }
  directEnd(D) {
    let res = null;
    if (D.blocked) { this.rotInfo = ''; this.orbit.enableZoom = true; this.dragPoses = null; this.clearPreview(); this.syncPoses(); return; }
    if (D.rigid) {
      for (const id of D.rigid) { const ps = this.dragPoses[id]; if (!ps) continue; const P = this.part(id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); }
      this.fixWorldAnchors(D.start, D.rigid); // ground-fixed parts take their world anchors along
      if (D.best) { const P = this.part(D.pid); res = this.applyAssembly(D.best, P, []); if (res) this.liftComp(P.id); }
      else if (!D.art) this.groundLift(D.rigid);
    } else if (D.sim) {
      const s = D.sim; s.world.removeImpulseJoint(D.spring, true);
      for (let i = 0; i < 60 && s.maxGap() > 1e-4; i++) s.relax(5);
      for (const P of this.M.parts) { const ps = s.partPose(P.id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); }
      this.lastDragGap = s.maxGap(); s.free();
    }
    this.rotInfo = ''; this.orbit.enableZoom = true;
    this.dragPoses = null; this.clearPreview(); this.commit(); this.rebuild(); if (D.multi) this.select({ p: D.pid }, true); else this.select({ p: D.pid });
    if (res) this.announce(res); else if (D.detached) this.msg('已拆开并移动'); 
  }
  detach(pid) {
    if (this.mode !== 'edit') this.reset(); const P = this.part(pid); if (!P) return; this.beginEdit();
    const n0 = this.M.joints.length + this.M.couplings.length;
    this.M.joints = this.M.joints.filter(j => j.a !== pid && j.b !== pid); const jids = new Set(this.M.joints.map(j => j.id));
    this.M.couplings = this.M.couplings.filter(c => c.pa !== pid && c.pb !== pid && jids.has(c.ja) && jids.has(c.jb));
    const n = n0 - this.M.joints.length - this.M.couplings.length; this.commit(); this.rebuild(); this.select({ p: pid }); this.msg('已拆开 ' + P.name + '：移除 ' + n + ' 个连接/传动');
  }
  trialRun(sec = 3) { if (this.mode !== 'edit') this.reset(); this.startRun(); this.trialEnd = performance.now() + sec * 1000; this.msg('试运行 ' + sec + ' 秒…（结束后自动回到编辑状态）'); }
  select(s, keepMulti) { if (!keepMulti) this.msel = new Set(s && s.p ? [s.p] : []); this.sel = s; this.tracePts = []; this.plotData = []; this.updSel(); this.updProps(); if (this.asm || this.asmPend) this.asmSyncSel(); }
  updSel() {
    const ms = this.msel || new Set();
    for (const id in this.objs) { const on = (this.sel && this.sel.p === id) || ms.has(id), hv = this.hoverPid === id; this.objs[id].traverse(o => { if (o.isMesh && o.material.emissive) { o.material.emissive.setHex(hv ? 0xf59e0b : on ? 0x1d4ed8 : o.userData.baseEm); o.material.emissiveIntensity = hv ? 0.45 : on ? 0.35 : 1; } }); }
    this.olSync();
    // the gizmo is never a default state: it belongs to the part it was explicitly switched on for (G / Shift+G / button) and turns off when the selection changes
    if (this.gizmoOn && (!this.sel || !this.sel.p || this.gizKey() !== this._gizmoFor || this.asm || this.mode !== 'edit')) { this.gizmoOn = false; this._gizmoFor = null; if (this.updGizmoBtns && $('btnGizmo')) this.updGizmoBtns(); }
    const multi = !!(this.msel && this.msel.size > 1);
    if (this.gizmoOn && this.sel && this.sel.p && this.mode === 'edit' && this.tool === 'select' && this.part(this.sel.p) && !(multi ? this.lockedIn(this.moveSet(this.sel.p, { multi: true })).length : this.part(this.sel.p).locked)) {
      // multi-selection / group: the gizmo sits at the selection centre, world-aligned, and moves / rotates everything together
      if (multi) { this.proxy.position.copy(this.selCenter(this.moveSet(this.sel.p, { multi: true }))); this.proxy.quaternion.identity(); }
      else { const P = this.part(this.sel.p); this.proxy.position.copy(V3(P.p)); this.proxy.quaternion.copy(Q4(P.q)); }
      this.tc.attach(this.proxy);
    } else this.tc.detach();
  }
  gizKey() { return this.msel && this.msel.size > 1 ? 'm:' + this.selIds().sort().join(',') : (this.sel && this.sel.p) || null; }
  showPoints() {
    if (this.ptsGroup) this.scene.remove(this.ptsGroup);
    const g = this.ptsGroup = new THREE.Group(); this.scene.add(g);
    if (this.mode !== 'edit' || this.tool === 'select' || this.tool === 'addcon') return;
    const geo = new THREE.SphereGeometry(0.006, 10, 8), m = new THREE.MeshBasicMaterial({ color: 0x2563eb, depthTest: false, transparent: true, opacity: 0.8 });
    for (const P of this.M.parts) for (const pt of (this.tool === 'mate' ? connectors(P).filter(c => !c.line) : DEFS[P.type].pts(P.d))) { const s = new THREE.Mesh(geo, m); s.position.copy(ptW(P, pt.p)); s.renderOrder = 9; s.userData.helper = true; s.raycast = () => { }; g.add(s); }
  }
  setTool(t) { if (this.asm && t !== 'select') this.asmCancel(true); if (this.mateS) this.mateCancel(); this.tool = t; this.pick1 = null; this.mate1 = null; this.hl2.visible = false; document.querySelectorAll('.tool').forEach(b => b.classList.toggle('sel', b.dataset.t === t)); this.updSel(); this.showPoints(); if (this.showToolBar) this.showToolBar();
    if (t === 'mate') return this.msg('配合：先点击零件 A 上的面/连接点，再点击零件 B 上的面/连接点 → A 贴合到 B（法向相对），然后选择连接类型、偏移、角度');
    if (t === 'addcon') return this.msg('添加连接点：选择类型（孔/轴/安装面），在零件表面点击放置；Esc 完成');
    this.msg(t === 'select' ? '选择/移动：Ctrl/Shift+点击选两个零件 → 装配面板（J）；左键拖动零件移动，G 显示坐标轴' : (JT[t] ? JT[t] + '：依次点击两个零件的连接点（蓝点），第二下点地面=连接大地' : CT[t] + '：依次点击两个已用铰链/滑动副安装好的传动零件')); }
  // ---------------- edit drag (constraints kept) ----------------
  dragStart() {
    if (this.mode !== 'edit' || !this.sel || !this.sel.p) return;
    this.tcDragged = true; this.lastSnap = this.snapshot();
    const pid = this.sel.p, an = analyze(this.M), g = an.groupOf[pid], comp = an.comps[an.compOf[pid]];
    const start = {}; for (const P of this.M.parts) start[P.id] = { p: V3(P.p), q: Q4(P.q) };
    const multi = !!(this.msel && this.msel.size > 1);
    this.drag = { pid, start, dragPoses: {}, multi, ref: multi ? { p: this.proxy.position.clone(), q: this.proxy.quaternion.clone() } : start[pid] }; this.snapAlt = 0;
    if (multi) { this.drag.rigid = this.moveSet(pid, { multi: true }); } // gizmo on a multi-selection / group: everything moves / rotates about the selection centre
    else if (!comp.anchored || g.static) { this.drag.rigid = comp.parts.map(p => p.id); }
    else { this.drag.sim = new Sim(this.M, { edit: true, kinematic: pid }); }
    this.dragPoses = {};
  }
  dragMove(tp, tq) {
    const D = this.drag; if (!D) return; const s0 = D.ref || D.start[D.pid];
    if (D.rigid) {
      const dq = tq.clone().multiply(s0.q.clone().invert());
      for (const id of D.rigid) { const s = D.start[id]; this.dragPoses[id] = { p: s.p.clone().sub(s0.p).applyQuaternion(dq).add(tp), q: dq.clone().multiply(s.q) }; }
      D.best = null;
      if (this.autoOn()) { const ps = this.dragPoses[D.pid], r = findAssembly({ M: this.M, mover: this.part(D.pid), p0: ps.p, q0: ps.q, ray: null, cursor: null, exclude: new Set(D.rigid), toScreen: v => this.toScreen(v), camPos: this.cam.position }); D.best = r.best; this.showPreview(r.best, r.near); }
      if (!D.best) { const sn = this.findSnap(D.rigid); if (sn) { this.hl.position.copy(sn.to); this.hl.scale.setScalar(1); this.hl.visible = true; } else if (!this.snapTarget) this.hl.visible = false; }
    } else {
      const r = D.sim.pb[D.pid], bq = tq.clone().multiply(r.oq.clone().invert()), bp = tp.clone().sub(r.op.clone().applyQuaternion(bq));
      r.body.setNextKinematicTranslation({ x: bp.x, y: bp.y, z: bp.z }); r.body.setNextKinematicRotation({ x: bq.x, y: bq.y, z: bq.z, w: bq.w });
      D.sim.relax(8);
      for (const P of this.M.parts) this.dragPoses[P.id] = D.sim.partPose(P.id);
    }
    this.syncPoses();
  }
  findSnap(ids) {
    const set = new Set(ids); let best = null;
    for (const id of ids) { const P = this.part(id), ps = this.dragPoses[id]; for (const pt of DEFS[P.type].pts(P.d)) { const w = V3(pt.p).applyQuaternion(ps.q).add(ps.p);
      for (const Q of this.M.parts) { if (set.has(Q.id)) continue; for (const qt of DEFS[Q.type].pts(Q.d)) { const wq = ptW(Q, qt.p), d = w.distanceTo(wq); if (d < 0.015 && (!best || d < best.d)) best = { d, from: w, to: wq }; } } } }
    return best;
  }
  dragEnd() {
    const D = this.drag; if (!D) return; this.drag = null;
    if (D.rigid) {
      const sn = Object.keys(this.dragPoses).length && !D.best ? this.findSnap(D.rigid) : null;
      for (const id of D.rigid) { const ps = this.dragPoses[id]; if (!ps) continue; if (sn) ps.p.add(sn.to.clone().sub(sn.from)); const P = this.part(id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); }
      this.fixWorldAnchors(D.start, D.rigid);
      if (D.best) { D.res = this.applyAssembly(D.best, this.part(D.pid), D.rigid); if (D.res) this.liftComp(D.pid); }
      if (sn) this.msg('已吸附到连接点'); this.clearPreview();
    } else {
      const s = D.sim, r = s.pb[D.pid]; r.body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
      for (let i = 0; i < 60 && s.maxGap() > 1e-4; i++) s.relax(5);
      for (const P of this.M.parts) { const ps = s.partPose(P.id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); }
      this.lastDragGap = s.maxGap(); s.free();
    }
    this.dragPoses = null; this.commit(); this.rebuild(); if (D.res) this.announce(D.res);
  }
  // ---------------- snap type cycling (Tab) ----------------
  dragging() { return !!(this.palDrag || (this.pd && this.pd.started) || this.drag || this.md); }
  altType(best) { if (!best || !best.alts || best.alts.length < 2) return null; const n = best.alts.length; return best.alts[((this.snapAlt || 0) % n + n) % n]; }
  cycleSnapType(d) { this.snapAlt = (this.snapAlt || 0) + d; this.refreshDragPreview(); }
  refreshDragPreview() {
    if (this.palDrag && this._palEv) { this.previewPalette(this._palEv.type, this._palEv.ev, this._palEv.over); return; }
    if (this.pd && this.pd.started && this.pd.lastEv) { this.directMove(this.pd.lastEv); return; }
    if (this._pv) this.showPreview(this._pv[0], this._pv[1]);
  }
  // ---------------- rotation with the mouse wheel / middle button ----------------
  viewAxis() { const d = this.cam.getWorldDirection(new THREE.Vector3()), a = [Math.abs(d.x), Math.abs(d.y), Math.abs(d.z)], i = a.indexOf(Math.max(...a)); return ['X', 'Y', 'Z'][i]; }
  rotAxisName() { return this.rotAxis && this.rotAxis !== 'view' ? this.rotAxis : this.viewAxis(); }
  axisVec(n) { return new THREE.Vector3(n === 'X' ? 1 : 0, n === 'Y' ? 1 : 0, n === 'Z' ? 1 : 0); }
  wheel(e) {
    if (!this.dragging() || this.mode !== 'edit') return;
    e.preventDefault(); e.stopImmediatePropagation();
    const dl = e.deltaY || e.deltaX; if (!dl) return;
    if (e.shiftKey || e.ctrlKey) { const L = ['view', 'X', 'Y', 'Z'], i = L.indexOf(this.rotAxis || 'view'); this.rotAxis = L[(i + 1) % L.length]; this.rotInfo = '旋转轴 ' + this.rotAxisName() + (this.rotAxis === 'view' ? '(视角)' : '') + ' · 滚轮旋转 15°'; this.refreshDragPreview(); return; }
    this.rotateDrag((dl > 0 ? 1 : -1) * 15 * D2R);
  }
  rotateDrag(ang) {
    const ax = this.rotAxisName(), R = new THREE.Quaternion().setFromAxisAngle(this.axisVec(ax), ang);
    this.rotAcc = ((this.rotAcc || 0) + ang / D2R) % 360;
    if (this.palDrag) { this.palRot = R.clone().multiply(this.palRot || new THREE.Quaternion()); }
    const D = this.pd;
    if (D && D.started && D.rigid) { D.q0 = R.clone().multiply(D.q0); }
    else if (D && D.started && D.sim) {
      // jointed part: rotate about its hinge (if any) and let the linkage follow
      const s = D.sim, body = D.body, rec = s.rj.find(r => (r.bA === body || r.bB === body) && r.j.type === 'revolute');
      let axis = this.axisVec(ax), piv = tv3(body.translation());
      if (rec) { axis = s.jointAxisW(rec); piv = s.toWorld(rec.bA, rec.l1); }
      const Rq = new THREE.Quaternion().setFromAxisAngle(axis, ang), bq = Rq.clone().multiply(tq4(body.rotation())), bp = tv3(body.translation()).sub(piv).applyQuaternion(Rq).add(piv);
      body.setTranslation({ x: bp.x, y: bp.y, z: bp.z }, true); body.setRotation({ x: bq.x, y: bq.y, z: bq.z, w: bq.w }, true);
      const T = tv3(D.cursor.translation()), gl = s.toBody(body, T); s.world.removeImpulseJoint(D.spring, true);
      D.spring = s.world.createImpulseJoint(RAPIER.JointData.spring(0, 600, 60, { x: 0, y: 0, z: 0 }, { x: gl.x, y: gl.y, z: gl.z }), D.cursor, body, true); D.grabL = gl;
      s.relax(10); for (const P of this.M.parts) this.dragPoses[P.id] = s.partPose(P.id); this.syncPoses();
      this.rotInfo = '绕铰链旋转 ' + Math.round(this.rotAcc) + '°'; return;
    }
    this.rotInfo = '旋转轴 ' + ax + ' · ' + Math.round(this.rotAcc) + '°（Shift/Ctrl+滚轮换轴）';
    this.refreshDragPreview();
  }
  midStart(pid, e) {
    this.beginEdit(); this.select({ p: pid }); this.tc.detach();
    const an = analyze(this.M), g = an.groupOf[pid], comp = an.comps[an.compOf[pid]], ids = new Set(g.parts.map(p => p.id));
    const start = {}; for (const P of this.M.parts) start[P.id] = { p: V3(P.p), q: Q4(P.q) };
    const D = this.md = { pid, x: e.clientX, y: e.clientY, start, ang: 0 };
    const j = this.M.joints.find(j => j.type === 'revolute' && (ids.has(j.a) || ids.has(j.b)) && !(ids.has(j.a) && ids.has(j.b)));
    if (!comp.anchored || g.static) { D.rigid = comp.parts.map(p => p.id); D.axis = this.axisVec(this.rotAxisName()); D.piv = V3(this.part(pid).p); }
    else { D.sim = new Sim(this.M, { edit: true, kinematic: pid }); if (j) { const A = this.part(j.a); D.axis = axW(A, j.axa); D.piv = ptW(A, j.la); } else { D.axis = this.axisVec(this.rotAxisName()); D.piv = V3(this.part(pid).p); } }
    this.dragPoses = {}; D.hinge = !!(j && D.sim);
  }
  /** rotate the part started with midStart by `a` degrees (rigid assembly, or about its hinge when jointed) */
  midApply(a) {
    const D = this.md; D.ang = a;
    const R = new THREE.Quaternion().setFromAxisAngle(D.axis, a * D2R);
    const rot = id => { const s = D.start[id]; return { p: s.p.clone().sub(D.piv).applyQuaternion(R).add(D.piv), q: R.clone().multiply(s.q) }; };
    if (D.rigid) for (const id of D.rigid) this.dragPoses[id] = rot(id);
    else { const t = rot(D.pid), r = D.sim.pb[D.pid], bq = t.q.clone().multiply(r.oq.clone().invert()), bp = t.p.clone().sub(r.op.clone().applyQuaternion(bq));
      r.body.setNextKinematicTranslation({ x: bp.x, y: bp.y, z: bp.z }); r.body.setNextKinematicRotation({ x: bq.x, y: bq.y, z: bq.z, w: bq.w }); D.sim.relax(8);
      for (const P of this.M.parts) this.dragPoses[P.id] = D.sim.partPose(P.id); }
    this.syncPoses();
  }
  /** R key / rotate buttons: rotate the selected part 15° about the chosen axis (hinged parts turn about their hinge) */
  rotateSel(dir = 1, deg = 15) {
    if (this.mode !== 'edit') this.reset(); const pid = this.sel && this.sel.p, P = pid && this.part(pid);
    if (!P) { this.msg('先选中一个零件，再按 R 旋转（Shift+R 反向）'); return; }
    if (this.msel && this.msel.size > 1) return this.rotateSelBy(this.rotAxisName(), dir * deg);
    if (P.locked) { this.msg(P.name + ' 已锁定，无法旋转（在组件栏点 🔒 解锁）', true); return; }
    this.midStart(pid, { clientX: 0, clientY: 0 }); const hinge = !!this.md.hinge; this.midApply(dir * deg); this.midEnd();
    this.msg('已旋转 ' + P.name + ' ' + (dir * deg) + '°' + (hinge ? '（绕铰链）' : '（绕 ' + this.rotAxisName() + ' 轴）') + ' · R / Shift+R');
  }
  midEnd() {
    const D = this.md; this.md = null; this.orbit.enabled = true; this.rotInfo = '';
    if (D.rigid) { for (const id of D.rigid) { const ps = this.dragPoses[id]; if (ps) { const P = this.part(id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); } } this.fixWorldAnchors(D.start, D.rigid); }
    else if (D.sim) { const s = D.sim; s.pb[D.pid].body.setBodyType(RAPIER.RigidBodyType.Dynamic, true); for (let i = 0; i < 60 && s.maxGap() > 1e-4; i++) s.relax(5); for (const P of this.M.parts) { const ps = s.partPose(P.id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); } s.free(); }
    this.dragPoses = null; this.clearPreview(); if (Math.abs(D.ang) > 1e-6) this.commit(); this.rebuild(); this.select({ p: D.pid });
  }
  /** the assembly containing pid must not sink into the floor (free OR anchored — the floor is solid): lift it rigidly, with its ground anchors */
  liftComp(pid) { return groundFixModel(this.M, [pid]).dy; }
  /** post-edit ground hook: lift every assembly whose geometry is below y = 0 (relative poses / joints kept, ground anchors move along).
   *  o.only: restrict to the assemblies containing these ids; o.quiet: no toast. Returns { dy, lifted } */
  groundFix(o = {}) {
    if (!this.M || !this.M.parts.length) return { dy: 0, lifted: [] };
    const r = groundFixModel(this.M, o.only);
    if (r.dy > 0.001 && !o.quiet && !this._quietGround) this.toast('已抬升至地面以上（↑' + (r.dy * 1000).toFixed(1) + ' mm）');
    return r;
  }
  /** safety net on every rebuild / reset in edit mode: whatever path wrote the poses, nothing stays below the floor.
   *  The regular hooks (commit / previews / undo …) should already have lifted it — if this one has to act, it logs which path slipped through. */
  groundNet(where) {
    if (this.mode !== 'edit' || !this.M || !this.M.parts.length || this._groundNetOff) return 0;
    const r = groundFixModel(this.M);
    if (r.dy > 1e-6) { const names = [...new Set(r.lifted)].map(id => (this.part(id) || {}).name).slice(0, 6).join('、'); (this.groundLog = this.groundLog || []).push({ where, dy: r.dy, ids: r.lifted.slice(), t: Date.now() }); if (this.groundLog.length > 50) this.groundLog.shift(); console.warn('[groundFix safety net] ' + where + ': lifted ' + names + ' by ' + (r.dy * 1000).toFixed(2) + ' mm'); }
    return r.dy;
  }
  /** brief toast over the viewport */
  toast(t, ms = 1800) {
    let el = $('gToast'); const c = $('center'); if (!c) return;
    if (!el) { el = document.createElement('div'); el.id = 'gToast'; el.style.cssText = 'position:absolute;left:50%;top:14px;transform:translateX(-50%);background:rgba(15,23,42,.88);color:#fde68a;padding:6px 14px;border-radius:6px;font-size:13px;z-index:40;pointer-events:none;transition:opacity .3s;opacity:0'; c.appendChild(el); }
    el.textContent = t; el.style.opacity = '1'; this.lastToast = t; this.lastToastT = performance.now(); clearTimeout(this._toastT); this._toastT = setTimeout(() => { el.style.opacity = '0'; }, ms);
  }
  /** live (drag / palette) preview: per-part vertical offsets so the previewed assemblies stay above the floor (model untouched) */
  computeLiveLift() {
    this._lift = null;
    if (this.mode === 'edit' && this._palLift) { this._lift = this._palLift; return; }
    if (this.mode !== 'edit' || !this.dragPoses || !Object.keys(this.dragPoses).length) return;
    const an = analyze(this.M), dp = this.dragPoses, groups = new Map();
    const tgt = this.snapTarget && this.snapTarget.target ? this.snapTarget.target.id : null, mover = (this.pd && this.pd.pid) || (this.drag && this.drag.pid) || null;
    for (const id of Object.keys(dp)) { if (!this.part(id)) continue; const c = an.compOf[id]; if (!groups.has(c)) groups.set(c, new Set()); }
    if (tgt && mover && an.compOf[tgt] != null && an.compOf[mover] != null) { const a = an.compOf[mover], b = an.compOf[tgt]; groups.set(a, new Set([b])); groups.set(b, groups.get(b) || new Set()); }
    const out = {}, used = new Set();
    for (const [c, extra] of groups) {
      if (used.has(c)) continue; const cs = [c, ...extra]; cs.forEach(x => used.add(x));
      const ids = cs.flatMap(x => (an.comps[x] ? an.comps[x].parts.map(p => p.id) : []));
      let lo = Infinity; for (const id of ids) { const P = this.part(id), ps = dp[id]; lo = Math.min(lo, ps ? geoMinY(P, ps.p, ps.q, shapeMinY) : partMinY(P)); }
      if (isFinite(lo) && lo < -GROUND_TOL) { const dy = -lo + GROUND_CLEAR; for (const id of ids) out[id] = dy; }
    }
    if (Object.keys(out).length) this._lift = out;
  }
  // ---------------- joint editing ----------------
  jointById(id) { return this.M.joints.find(j => j.id === id); }
  editJoint(id, fn, msg) {
    if (this.mode !== 'edit') this.reset(); const j = this.jointById(id); if (!j) return;
    this.beginEdit(); fn(j); j.mod = true; this.commit(); this.relaxAll(); this.rebuild(); if (msg) this.msg(msg);
  }
  setJointType(id, t) {
    this.editJoint(id, j => {
      const old = j.type; j.type = t;
      if (t === 'helical' && j.lead == null) j.lead = 0.005;
      if (old !== t) { delete j.lim; }
    }, '连接已改为 ' + JT[t] + '（已修改，可撤销）');
  }
  setJointAxis(id, w) { this.editJoint(id, j => { const A = this.part(j.a); j.axa = axL(A, w.clone().normalize()).toArray(); }, '轴线已修改'); }
  flipJoint(id) { this.editJoint(id, j => { const A = this.part(j.a); j.axa = axL(A, axW(A, j.axa).negate()).toArray(); j.motorRev = !j.motorRev; if (j.lim) [j.lim.lo, j.lim.hi] = [-j.lim.hi, -j.lim.lo]; }, '已反转轴（电机/驱动方向随之反向）'); }
  deleteJoint(id) {
    if (this.mode !== 'edit') this.reset(); this.beginEdit(); this.M.joints = this.M.joints.filter(j => j.id !== id);
    const jids = new Set(this.M.joints.map(j => j.id)); this.M.couplings = this.M.couplings.filter(c => jids.has(c.ja) && jids.has(c.jb));
    if (this.sel && this.sel.j === id) this.sel = null; this.commit(); this.rebuild(); this.msg('已删除连接');
  }
  /** remove the joint and move the free side 3 cm away so the parts are visibly apart */
  splitJoint(id) {
    if (this.mode !== 'edit') this.reset(); const j = this.jointById(id); if (!j) return; this.beginEdit();
    this.M.joints = this.M.joints.filter(x => x.id !== id); const jids = new Set(this.M.joints.map(x => x.id)); this.M.couplings = this.M.couplings.filter(c => jids.has(c.ja) && jids.has(c.jb));
    const an = analyze(this.M), pick = [j.b, j.a].find(pid => pid && !an.comps[an.compOf[pid]].anchored) || null;
    if (pick && j.b) { const other = pick === j.a ? j.b : j.a, d = V3(this.part(pick).p).sub(V3(this.part(other).p)); if (d.length() < 1e-6) d.set(0, 1, 0); d.normalize().multiplyScalar(0.03); for (const P of an.comps[an.compOf[pick]].parts) P.p = V3(P.p).add(d).toArray(); }
    if (this.sel && this.sel.j === id) this.sel = { p: j.a }; this.commit(); this.rebuild(); this.msg('已拆开连接');
  }
  isMotorJ(j) { const A = this.part(j.a), B = j.b ? this.part(j.b) : null; return j.type === 'revolute' && ((A && A.type === 'motor') || (B && B.type === 'motor')) ? (A.type === 'motor' ? A : B) : null; }
  // ---------------- context menu ----------------
  ctxMenu(e) {
    if (this.mode !== 'edit') return; const h = this.pickAt(e); if (!h.pid && !h.jid) { this.hideCtx(); return; }
    const js = h.jid ? [this.jointById(h.jid)] : this.M.joints.filter(j => (j.a === h.pid || j.b === h.pid) && !isForce(j));
    const el = $('ctx'), nm = j => JT[j.type] + '：' + ((this.part(j.a) || {}).name || '?') + ' ↔ ' + (j.b ? (this.part(j.b) || {}).name : '大地');
    let html = h.pid ? `<div class="ch">${this.part(h.pid).name}</div>` : '';
    js.slice(0, 5).forEach(j => { html += `<div class="ch2">${nm(j)}</div>` + [['type:revolute', '改为铰链'], ['type:fixed', '改为固定'], ['type:prismatic', '改为滑动'], ['flip', '反转轴'], ['del', '删除连接'], ['split', '拆开']].filter(([a]) => a !== 'type:' + j.type && !(a === 'flip' && ['fixed', 'spherical'].includes(j.type))).map(([a, t]) => `<div class="ci" data-a="${a}" data-j="${j.id}">${t}</div>`).join(''); });
    if (!js.length) html += '<div class="ch2">（无连接）</div>';
    if (h.pid && js.length) html += `<div class="ci" data-a="detach" data-p="${h.pid}">拆开全部连接</div>`;
    if (h.pid) html += `<div class="ch2">编辑</div><div class="ci" data-a="gizmo" data-p="${h.pid}">${this.gizmoOn && this.tc.mode === 'translate' ? '隐藏坐标轴 (G)' : '显示移动坐标轴 (G)'}</div><div class="ci" data-a="gizmoR" data-p="${h.pid}">${this.gizmoOn && this.tc.mode === 'rotate' ? '隐藏旋转坐标轴' : '显示旋转坐标轴 (Shift+G)'}</div><div class="ci" data-a="mate" data-p="${h.pid}">🧲 从此处开始配合 (M)</div><div class="ci" data-a="addcon" data-p="${h.pid}">＋ 在此处添加连接点</div>`;
    if (h.pid && this.selIds().length > 1) html += `<div class="ci" data-a="group">组合所选 (Ctrl+G)</div><div class="ci" data-a="dupsel">复制所选 (Ctrl+D)</div>`;
    html += `<div class="ci" data-a="props" data-p="${h.pid || ''}" data-j="${h.jid || ''}">属性…</div>`;
    el.innerHTML = html; el.style.display = 'block'; const cr = $('center').getBoundingClientRect();
    el.style.left = (e.clientX - cr.left + 2) + 'px'; el.style.top = Math.min(e.clientY - cr.top + 2, cr.height - el.offsetHeight - 4) + 'px';
    if (h.jid) this.select({ j: h.jid }); else if (!(this.msel && this.msel.has(h.pid))) this.select({ p: h.pid });
    const hit = h;
    el.querySelectorAll('.ci').forEach(it => it.onclick = ev => {
      ev.stopPropagation(); const a = it.dataset.a, jid = it.dataset.j; this.hideCtx();
      if (a.startsWith('type:')) this.setJointType(jid, a.slice(5)); else if (a === 'flip') this.flipJoint(jid); else if (a === 'del') this.deleteJoint(jid); else if (a === 'split') this.splitJoint(jid);
      else if (a === 'gizmo') { this.select({ p: it.dataset.p }); this.toggleGizmo('translate'); } else if (a === 'gizmoR') { this.select({ p: it.dataset.p }); this.toggleGizmo('rotate'); }
      else if (a === 'mate') { this.setTool('mate'); this.mateClick(hit); } else if (a === 'addcon') { this.setTool('addcon'); this.conClick(hit); }
      else if (a === 'group') this.groupSel(); else if (a === 'dupsel') this.cloneSet(this.selIds(), 'copy');
      else if (a === 'detach') this.detach(it.dataset.p); else if (a === 'props') this.select(jid ? { j: jid } : { p: it.dataset.p });
    });
  }
  hideCtx() { const el = $('ctx'); if (el) el.style.display = 'none'; }
  // ---------------- run control ----------------
  startRun() {
    if (this.mode === 'run') return; if (this.asm) this.asmCancel(true);
    if (this.mode === 'edit') { this.init = this.snapshot(); if (this.sim) this.sim.free(); this.sim = new Sim(this.M, { gravity: this.gravity }); this.sim.keys = this.keys; this.tracePts = []; this.plotData = []; this.pick1 = null; for (const k in this.cobjs) this.cobjs[k].s = 0; }
    if (this.flash) { this.flash.t0 = -1e9; this.updFlash(performance.now()); }
    this.mode = 'run'; this.tc.detach(); this.showPoints(); this.updButtons(); this.updProps(); const bk = [...this.boundKeys()].map(keyLabel); this.msg('运行中：真实3D物理（重力、碰撞、摩擦、电机）' + (bk.length ? '　键盘控制：' + bk.join(' / ') : ''));
    if (this.asmUnits && this.asmUnits(this.selIds()).length >= 2) this.asmPending(this.asmUnits(this.selIds()));
  }
  pause() { if (this.mode !== 'run') return; this.mode = 'pause'; this.updButtons(); this.msg('已暂停：可“将当前运行状态设为初始状态”，或“重置”回编辑模式'); if (this.asmPend) this.asmPending(this.asmPend); }
  toggleRun() { if (this.mode === 'run') this.pause(); else this.startRun(); }
  reset() {
    if (this.sim) { this.sim.free(); this.sim = null; }
    this.M = JSON.parse(this.init || this.snapshot()); this.mode = 'edit'; this.tracePts = []; this.plotData = [];
    // the initial state may come from a run pose (设为初始状态: contacts sink a few mm into the floor) or an old file: the floor is solid in edit mode
    if (this.groundFix({ quiet: true }).dy > 0) this.init = this.snapshot(); this.lastSnap = this.snapshot();
    this.rebuild(); this.updButtons(); this.msg('已重置到初始状态（编辑模式，可继续放置零件）');
    if (this.asmPend) { this.asmPendClear(); this.asmAuto(); }
  }
  setInitial() { if (this.mode === 'pause' || this.mode === 'run') return this.setInitialFromRun(); this.init = this.snapshot(); this.msg('已将当前布局设为初始状态'); }
  setInitialFromRun() {
    if (!this.sim) return; if (this.mode === 'run') this.pause();
    for (const P of this.M.parts) { const ps = this.sim.partPose(P.id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); }
    groundFixModel(this.M); // run poses rest ON the floor with a little contact penetration (wheels, gear teeth): store them lifted
    this.init = this.snapshot(); this.undoS.push(this.lastSnap || this.init); if (this.undoS.length > 200) this.undoS.shift(); this.lastSnap = this.init;
    this.msg('已将当前运行姿态设为初始状态（速度清零），重置后回到此姿态');
  }
  simulate(sec) { if (!this.sim) return; const n = Math.round(sec / this.sim.h); for (let i = 0; i < n; i++) { this.sim.substep(); this.afterSub(); } this.syncPoses(); }
  afterSub() {
    const s = this.sim;
    for (const id in this.cobjs) { const L = this.cobjs[id], A = this.part(L.c.pa); const r = s.pb[A.id]; if (!r) continue; const n = new THREE.Vector3(0, 0, 1).applyQuaternion(s.partPose(A.id).q); const w = new THREE.Vector3().copy(r.body.angvel()).dot(n); L.s -= w * pitchRadius(A) * s.h * (L.dir || 1); }
  }
  frame() {
    const now = performance.now(), dt = Math.min(0.05, (now - this.clock) / 1000); this.clock = now;
    if (this.mode === 'run' && this.sim) {
      this.acc += dt * this.speed; let n = 0;
      while (this.acc >= this.sim.h && n < 400) { this.sim.substep(); this.afterSub(); this.acc -= this.sim.h; n++; }
      if (n >= 400) this.acc = 0;
      this.syncPoses(); this.updRun();
    } else if (!this.drag) this.syncPoses();
    if (this.trialEnd && now > this.trialEnd) { this.trialEnd = 0; if (this.mode !== 'edit') this.reset(); }
    this.updFlash(now);
    this.orbit.update(); if (this.asm) this.asmLabels(); this.renderer.render(this.scene, this.cam);
  }
  updFlash(now) {
    const F = this.flash, lbl = $('snapLbl'); if (!F) return;
    if (this.dragging()) { this.flash = null; this.updSel(); return; }
    const t = (now - F.t0) / 1000;
    if (t > 1.6) { this.flash = null; lbl.style.display = 'none'; lbl.className = ''; this.updSel(); for (const id of F.jids) { const o = this.jobjs[id]; if (o) o.scale.setScalar(1); } return; }
    const k = 0.5 + 0.5 * Math.sin(t * 14);
    for (const id of F.ids) { const o = this.objs[id]; if (o) o.traverse(m => { if (m.isMesh && m.material.emissive) { m.material.emissive.setHex(0x22c55e); m.material.emissiveIntensity = 0.15 + 0.5 * k * (1 - t / 1.6); } }); }
    for (const id of F.jids) { const o = this.jobjs[id]; if (o && !isForce(this.M.joints.find(j => j.id === id) || {})) o.scale.setScalar(1 + 1.5 * (1 - t / 1.6)); }
    if (this.mode === 'edit' && F.marker) { const s = this.toScreen(F.marker), cr = $('center').getBoundingClientRect(); lbl.style.display = 'block'; lbl.className = 'ok'; lbl.textContent = F.label; lbl.style.left = (s.x - cr.left + 14) + 'px'; lbl.style.top = (s.y - cr.top - 34) + 'px'; }
  }
  selPartForRun() { return this.sel && this.sel.p && this.sim && this.sim.pb[this.sel.p] ? this.sel.p : null; }
  readout(pid) {
    const s = this.sim, r = s.pb[pid], b = r.body, P = this.part(pid), ps = s.partPose(pid);
    const v = new THREE.Vector3().copy(b.velocityAtPoint(ps.p)), w = new THREE.Vector3().copy(b.angvel());
    const jr = s.rj.find(x => x.j.type === 'revolute' && (x.bA === b || x.bB === b));
    const n = jr ? s.jointAxisW(jr) : (w.length() > 1e-9 ? w.clone().normalize() : new THREE.Vector3(0, 0, 1));
    const wn = w.dot(n), m = b.isDynamic() ? b.mass() : 0;
    const last = this._last && this._last.pid === pid ? this._last : null, t = s.time;
    let F = 0, T = 0;
    if (last && t > last.t) { const dtt = t - last.t; F = m * v.clone().sub(last.v).length() / dtt; const I = b.isDynamic() ? n.dot(n.clone().applyMatrix3(s.bodyInertiaW(b))) : 0; T = I * (wn - last.wn) / dtt; }
    this._last = { pid, t, v: v.clone(), wn };
    const mot = s.motors.find(x => x.part.id === pid || x.db === b);
    return { v, speed: v.length(), w: wn, rpm: wn * 30 / Math.PI, F, T, mot, m, pos: ps.p };
  }
  updRun() {
    const s = this.sim; const E = s.energy();
    $('energy').style.display = 'block';
    $('energy').innerHTML = `t = ${s.time.toFixed(2)} s<br>动能 ${E.K.toFixed(3)} J<br>重力势能 ${E.U.toFixed(3)} J<br>弹性势能 ${E.S.toFixed(3)} J<br><b>总能量 ${E.total.toFixed(3)} J</b><br>最大连接间隙 ${(s.maxGap() * 1000).toFixed(3)} mm`;
    const pid = this.selPartForRun();
    if (!pid) { this.arrow.visible = false; $('readout').style.display = 'none'; $('plot').style.display = 'none'; return; }
    const R = this.readout(pid), P = this.part(pid);
    let html = `<b>${P.name}</b><br>速度 |v| = <b>${R.speed.toFixed(4)}</b> m/s<br>角速度 = <b>${R.rpm.toFixed(2)}</b> rpm (${R.w.toFixed(3)} rad/s)<br>合力 ≈ ${R.F.toFixed(2)} N　合力矩 ≈ ${R.T.toFixed(3)} N·m<br>位置 (${R.pos.toArray().map(v => (v * 1000).toFixed(1)).join(', ')}) mm`;
    if (R.mot) html += `<br>电机扭矩 <b>${R.mot.tau.toFixed(3)}</b> N·m　功率 <b>${R.mot.P.toFixed(2)}</b> W`;
    $('readout').innerHTML = html; $('readout').style.display = 'block';
    if (R.speed > 1e-4) { this.arrow.visible = true; this.arrow.position.copy(R.pos); this.arrow.setDirection(R.v.clone().normalize()); const l = Math.min(0.6, 0.05 + R.speed * 0.4); this.arrow.setLength(l, Math.min(0.04, l * 0.3), 0.015); } else this.arrow.visible = false;
    // trace
    const pts = DEFS[P.type].pts(P.d), ti = P.tracePt != null ? P.tracePt : (P.type === 'rod' ? 1 : 0), ps = s.partPose(pid);
    const tw = V3(pts[Math.min(ti, pts.length - 1)].p).applyQuaternion(ps.q).add(ps.p);
    const T = this.tracePts; if (!T.length || T[T.length - 1].distanceTo(tw) > 0.002) { T.push(tw); if (T.length > 4000) T.shift(); }
    const arr = this.trace.geometry.attributes.position; T.forEach((p, i) => arr.setXYZ(i, p.x, p.y, p.z)); arr.needsUpdate = true; this.trace.geometry.setDrawRange(0, T.length);
    // plot
    const pd = this.plotData; pd.push([s.time, R.rpm, R.speed]); while (pd.length && pd[0][0] < s.time - 10) pd.shift(); this.drawPlot();
  }
  drawPlot() {
    const c = $('plot'), g = c.getContext('2d'), W = c.width, H = c.height, pd = this.plotData; c.style.display = 'block';
    g.clearRect(0, 0, W, H); g.fillStyle = '#fff'; g.fillRect(0, 0, W, H); if (pd.length < 2) return;
    const useRpm = pd.some(d => Math.abs(d[1]) > 0.5), k = useRpm ? 1 : 2, vals = pd.map(d => d[k]);
    let lo = Math.min(...vals), hi = Math.max(...vals); const span = Math.max(hi - lo, Math.abs(hi) * 0.1, 0.02); const mid = (hi + lo) / 2; if (hi - lo < span) { hi = mid + span / 2; lo = mid - span / 2; } const t0 = pd[0][0], t1 = pd[pd.length - 1][0];
    g.strokeStyle = '#e5e7eb'; g.lineWidth = 1; for (let i = 0; i <= 4; i++) { g.beginPath(); g.moveTo(40, 20 + i * (H - 40) / 4); g.lineTo(W - 10, 20 + i * (H - 40) / 4); g.stroke(); }
    g.strokeStyle = '#2563eb'; g.lineWidth = 3; g.beginPath(); pd.forEach((d, i) => { const x = 40 + (d[0] - t0) / Math.max(1e-6, t1 - t0) * (W - 50), y = 20 + (hi - d[k]) / (hi - lo) * (H - 40); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke();
    g.fillStyle = '#374151'; g.font = '22px sans-serif'; g.fillText((useRpm ? '角速度 rpm' : '速度 m/s') + '  [' + lo.toFixed(2) + ', ' + hi.toFixed(2) + ']', 44, 18);
  }
  // ---------------- UI ----------------
  msg(t, warn) { $('stMsg').textContent = t; $('stMsg').style.color = warn ? '#fca5a5' : ''; if (warn) console.warn(t); }
  updStatus() { $('stInfo').textContent = `零件 ${this.M.parts.length}　连接 ${this.M.joints.length}　传动 ${this.M.couplings.length}`; }
  updButtons() {
    const m = this.mode; $('modeBadge').textContent = { edit: '编辑模式', run: '运行中', pause: '已暂停' }[m]; $('modeBadge').className = m === 'edit' ? '' : m;
    $('btnRun').disabled = m === 'run'; $('btnRun').textContent = m === 'pause' ? '▶ 继续' : '▶ 运行'; $('btnPause').disabled = m !== 'run'; $('btnReset').disabled = m === 'edit';
    $('btnSetInitRun').disabled = m === 'edit'; $('btnUndo').disabled = !this.undoS.length; $('btnRedo').disabled = !this.redoS.length;
    if (m === 'edit') { $('energy').style.display = 'none'; $('readout').style.display = 'none'; $('plot').style.display = 'none'; this.arrow.visible = false; this.trace.geometry.setDrawRange(0, 0); }
    $('side').classList.toggle('run', m !== 'edit'); this.updStatus();
  }
  initUI() {
    const mk = (cat, el) => { for (const [t, D] of Object.entries(DEFS)) if (D.cat === cat) { const d = document.createElement('div'); d.className = 'item'; d.dataset.type = t; d.innerHTML = `<span class="dot" style="background:#${new THREE.Color((MATS[D.mat] || MATS.steel).color).getHexString()}"></span>${D.label}`; el.appendChild(d); this.paletteDrag(d, t); } };
    mk('basic', $('palItems')); mk('trans', $('palTrans'));
    this.mkItem = (t, el) => { const D = DEFS[t], d = document.createElement('div'); d.className = 'item'; d.dataset.type = t; d.title = D.label; d.innerHTML = `<span class="dot" style="background:#${new THREE.Color((MATS[D.mat] || MATS.steel).color).getHexString()}"></span>${D.label}`; el.appendChild(d); this.paletteDrag(d, t); return d; };
    try { this.recent = JSON.parse(localStorage.getItem('mech3d-recent') || '[]').filter(t => DEFS[t]); } catch (e) { this.recent = []; }
    this.renderRecent();
    $('palSearch').oninput = () => this.filterPalette();
    $('palSearch').onkeydown = e => { if (e.key === 'Escape') { e.target.value = ''; this.filterPalette(); e.target.blur(); } if (e.key === 'Enter') { const it = [...document.querySelectorAll('#palItems .item, #palTrans .item')].find(x => x.style.display !== 'none'); if (it) { document.querySelectorAll('.item.armed').forEach(x => x.classList.remove('armed')); this.armed = it.dataset.type; it.classList.add('armed'); this.msg('点击场景中的位置放置 ' + DEFS[this.armed].label); e.target.blur(); } } };
    this.initOutliner();
    for (const [t, n, s] of TOOLS) { const b = document.createElement('div'); b.className = 'tool'; b.dataset.t = t; b.innerHTML = `${n}<small>${s}</small>`; b.onclick = () => this.setTool(t); $('palTools').appendChild(b); }
    this.setTool('select');
    PRESETS.forEach((p, i) => { const o = document.createElement('option'); o.value = i; o.textContent = p.name; $('selPreset').appendChild(o); });
    $('selPreset').onchange = e => { if (e.target.value !== '') this.loadPreset(+e.target.value); e.target.value = ''; };
    $('btnRun').onclick = () => this.startRun(); $('btnPause').onclick = () => this.pause(); $('btnReset').onclick = () => this.reset();
    $('btnSetInit').onclick = () => this.setInitial(); $('btnSetInitRun').onclick = () => this.setInitialFromRun();
    $('chkGravity').onchange = e => { this.gravity = e.target.checked; if (this.sim && !this.sim.edit) { this.sim.g = this.gravity ? 9.81 : 0; this.sim.world.gravity = { x: 0, y: -this.sim.g, z: 0 }; } };
    $('chkSnap').onchange = e => { this.snap = e.target.checked; this.applySnap(); };
    $('chkAuto').onchange = e => { this.auto = e.target.checked; this.msg(this.auto ? '自动连接：开（拖动零件靠近连接点即可自动装配）' : '自动连接：关（自由放置，可用“高级”连接工具手动连接）'); };
    $('tipClose').onclick = () => $('tipbar').style.display = 'none';
    $('selSpeed').onchange = e => this.speed = +e.target.value;
    $('btnUndo').onclick = () => this.undo(); $('btnRedo').onclick = () => this.redo();
    $('btnClear').onclick = () => { this.beginEdit(); this.M = { parts: [], joints: [], couplings: [], nid: this.M.nid }; this.sel = null; this.msel = new Set(); this.commit(); this.rebuild(); };
    $('btnSave').onclick = () => this.save(); $('btnLoad').onclick = () => $('fileIn').click(); $('fileIn').onchange = e => { const f = e.target.files[0]; if (f) f.text().then(t => this.loadJSON(t)); e.target.value = ''; };
    $('btnBom').onclick = () => this.download('机械装配BOM.csv', this.bomCSV(), 'text/csv;charset=utf-8'); $('btnPng').onclick = () => this.png();
    $('btnFit').onclick = () => this.fit();
    document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => this.view(b.dataset.view));
    $('btnMove').onclick = () => this.setGizmo('translate'); $('btnRot').onclick = () => this.setGizmo('rotate');
    window.addEventListener('keydown', e => this.key(e)); window.addEventListener('keyup', e => this.runKey(e, false));
    this.initExtra();
  }
  renderRecent() { const el = $('palRecent'); el.innerHTML = ''; for (const t of this.recent) this.mkItem(t, el); $('recentHdr').style.display = this.recent.length ? '' : 'none'; }
  noteRecent(t) { this.recent = [t, ...this.recent.filter(x => x !== t)].slice(0, 6); try { localStorage.setItem('mech3d-recent', JSON.stringify(this.recent)); } catch (e) { /* file:// without storage */ } this.renderRecent(); this.filterPalette(); }
  filterPalette() {
    const q = $('palSearch').value.trim().toLowerCase(); let n = 0;
    for (const [box, hdr] of [['palItems', 'basicHdr'], ['palTrans', 'transHdr'], ['palRecent', 'recentHdr']]) { let k = 0; for (const it of $(box).children) { const D = DEFS[it.dataset.type], ok = !q || D.label.toLowerCase().includes(q) || it.dataset.type.includes(q); it.style.display = ok ? '' : 'none'; if (ok) k++; } if (box !== 'palRecent') { $(hdr).style.display = k ? '' : 'none'; $(box).style.display = k ? '' : 'none'; n += k; } else if (this.recent.length) { $(hdr).style.display = k ? '' : 'none'; } }
    $('palNone').style.display = n ? 'none' : '';
  }
  // ---------------- outliner (组件栏) ----------------
  initOutliner() {
    this.olCollapsed = new Set(); this.olOpen = new Set(); this.msel = new Set();
    $('olSearch').oninput = () => this.renderOutliner();
    $('olToggle').onclick = () => { $('outliner').classList.toggle('min'); $('olToggle').textContent = $('outliner').classList.contains('min') ? '▸' : '▾'; };
    $('olList').addEventListener('mouseleave', () => this.olHover(null));
  }
  olHover(pid) { if (this.hoverPid === pid) return; this.hoverPid = pid; this.updSelColorsOnly(); }
  updSelColorsOnly() { const ms = this.msel || new Set(); for (const id in this.objs) { const on = (this.sel && this.sel.p === id) || ms.has(id), hv = this.hoverPid === id; this.objs[id].traverse(o => { if (o.isMesh && o.material.emissive) { o.material.emissive.setHex(hv ? 0xf59e0b : on ? 0x1d4ed8 : o.userData.baseEm); o.material.emissiveIntensity = hv ? 0.45 : on ? 0.35 : 1; } }); } }
  /** connected groups (any joint between two parts or coupling), in part order */
  olGroups() {
    const par = {}, f = x => par[x] === x ? x : (par[x] = f(par[x])); for (const P of this.M.parts) par[P.id] = P.id;
    const u = (a, b) => { if (par[a] && par[b]) par[f(a)] = f(b); };
    for (const j of this.M.joints) if (j.b) u(j.a, j.b); for (const c of this.M.couplings) u(c.pa, c.pb);
    const g = new Map(); for (const P of this.M.parts) { const r = f(P.id); if (!g.has(r)) g.set(r, []); g.get(r).push(P); }
    const asm = [], loose = []; for (const ps of g.values()) (ps.length > 1 ? asm : loose).push(...(ps.length > 1 ? [ps] : ps));
    return { asm, loose };
  }
  renderOutliner() {
    const el = $('olList'); if (!el) return; const q = $('olSearch').value.trim().toLowerCase(), st = el.scrollTop; el.innerHTML = '';
    const ICON = { base: '▣', rod: '━', shaft: '┃', wheel: '◯', block: '▭', plate: '▬', bearing: '⊓', caster: '⑂', ball: '●', wall: '▁', motor: 'Ⓜ', gear: '✲', rack: '☰', bevel: '◭', worm: '∿', wormwheel: '❂', pulley: '◎', sprocket: '✺', cam: '◐', follower: '│', screw: '⫶', nut: '⬡', flywheel: '⦿', gearbox: '▤', clutch: '⧉', ujoint: '✚', mesh: '◆' };
    const jOf = P => this.M.joints.filter(j => j.a === P.id || j.b === P.id), cOf = P => this.M.couplings.filter(c => c.pa === P.id || c.pb === P.id);
    const match = P => !q || (P.name || '').toLowerCase().includes(q) || DEFS[P.type].label.toLowerCase().includes(q);
    const nm = id => id ? ((this.part(id) || {}).name || '?') : '大地';
    const partRow = (P, box) => {
      const js = jOf(P), cs = cOf(P), n = js.length + cs.length, open = this.olOpen.has(P.id) || (!!q && false);
      const r = document.createElement('div'); r.className = 'olr' + (P.hidden ? ' hid' : '') + (P.locked ? ' lck' : ''); r.dataset.pid = P.id;
      r.innerHTML = `<span class="olc">${n ? (open ? '▾' : '▸') : ''}</span><span class="oli" style="color:#${new THREE.Color((MATS[P.mat] || MATS.steel).color).getHexString()}" title="${DEFS[P.type].label}">${ICON[P.type] || '■'}</span><span class="oln" title="双击改名">${P.name}</span>${n ? `<span class="olj" title="连接/传动数">🔗${n}</span>` : ''}<span class="olb ole" title="${P.hidden ? '显示' : '隐藏'}">${P.hidden ? '◌' : '👁'}</span><span class="olb oll" title="${P.locked ? '解锁' : '锁定（禁止移动）'}">${P.locked ? '🔒' : '🔓'}</span><span class="olb old" title="删除">✕</span>`;
      box.appendChild(r);
      r.onmouseenter = () => this.olHover(P.id);
      r.onclick = e => { const c = e.target.classList; if (c.contains('olc') && n) { this.olOpen.has(P.id) ? this.olOpen.delete(P.id) : this.olOpen.add(P.id); this.renderOutliner(); return; } if (c.contains('ole')) return this.setHidden(P.id, !P.hidden); if (c.contains('oll')) return this.setLocked(P.id, !P.locked); if (c.contains('old')) return this.deleteParts([P.id]); this.olClick(P.id, e); };
      r.ondblclick = e => { if (e.target.classList.contains('oln')) this.olRename(P.id, e.target); else if (!e.target.classList.contains('olb')) this.focusPart(P.id); };
      if (open) for (const j of js) { const d = document.createElement('div'); d.className = 'olr olsub'; d.dataset.jid = j.id; d.innerHTML = `<span class="jdot t-${j.type}"></span><span class="oln">${JT[j.type]} ↔ ${nm(j.a === P.id ? j.b : j.a)}</span>`; d.onclick = () => this.select({ j: j.id }); d.onmouseenter = () => { this.olHover(null); const o = this.jobjs[j.id]; if (o) o.scale.setScalar(2); }; d.onmouseleave = () => { const o = this.jobjs[j.id]; if (o) o.scale.setScalar(1); }; box.appendChild(d); }
      if (open) for (const c of cs) { const d = document.createElement('div'); d.className = 'olr olsub'; d.dataset.cid = c.id; d.innerHTML = `<span class="jdot t-gear"></span><span class="oln">${CT[c.type]} ${nm(c.pa)} → ${nm(c.pb)}</span>`; d.onclick = () => this.select({ c: c.id }); box.appendChild(d); }
    };
    const { asm, loose } = this.olGroups(); let shown = 0;
    asm.forEach((ps, i) => {
      const vis = ps.filter(match); if (!vis.length) return; const key = ps[0].id, col = this.olCollapsed.has(key) && !q;
      const nj = this.M.joints.filter(j => ps.some(P => P.id === j.a)).length;
      const h = document.createElement('div'); h.className = 'olg'; h.dataset.g = key; h.innerHTML = `<span class="olc">${col ? '▸' : '▾'}</span>装配体 ${i + 1}<span class="olgn">${ps.length} 零件 · ${nj} 连接</span>`;
      h.onclick = e => { if (e.ctrlKey || e.metaKey || e.shiftKey || !e.target.classList.contains('olc')) { if (!e.target.classList.contains('olc')) { this.msel = new Set(ps.map(P => P.id)); this.select({ p: ps[0].id }, true); return; } } this.olCollapsed.has(key) ? this.olCollapsed.delete(key) : this.olCollapsed.add(key); this.renderOutliner(); };
      el.appendChild(h); const box = document.createElement('div'); box.className = 'olbox'; el.appendChild(box); if (col) box.style.display = 'none';
      for (const P of vis) { partRow(P, box); shown++; }
    });
    const lv = loose.filter(match);
    if (lv.length) { const h = document.createElement('div'); h.className = 'olg'; h.innerHTML = `未连接零件<span class="olgn">${loose.length}</span>`; el.appendChild(h); const box = document.createElement('div'); box.className = 'olbox'; el.appendChild(box); for (const P of lv) { partRow(P, box); shown++; } }
    if (!shown) el.innerHTML = `<div class="hint2" style="padding:6px">${this.M.parts.length ? '没有匹配的零件' : '场景中还没有零件。从上方零件库拖入。'}</div>`;
    $('olCount').textContent = this.M.parts.length;
    el.scrollTop = st; this.olSync();
  }
  /** highlight the outliner rows of the current selection and scroll the primary one into view */
  olSync() {
    const el = $('olList'); if (!el) return; const ms = this.msel || new Set(), sp = this.sel && this.sel.p, sj = this.sel && this.sel.j, sc = this.sel && this.sel.c;
    if (sp && !this.olOpen.has(sp) && (this.M.joints.some(j => j.a === sp || j.b === sp)) && !el.querySelector(`[data-pid="${sp}"] + .olsub`)) { /* keep collapsed unless user opens */ }
    let prim = null;
    for (const r of el.querySelectorAll('.olr')) { const on = (r.dataset.pid && (r.dataset.pid === sp || ms.has(r.dataset.pid))) || (r.dataset.jid && r.dataset.jid === sj) || (r.dataset.cid && r.dataset.cid === sc); r.classList.toggle('sel', !!on); if (on && (r.dataset.pid === sp || r.dataset.jid === sj || r.dataset.cid === sc)) prim = r; }
    if (prim && prim !== this._olPrim) { const box = prim.parentElement; if (box && box.style.display === 'none') { box.style.display = ''; } prim.scrollIntoView({ block: 'nearest' }); }
    this._olPrim = prim;
  }
  olClick(pid, e) {
    if (e.ctrlKey || e.metaKey) { const ms = new Set(this.msel || []); if (this.sel && this.sel.p) ms.add(this.sel.p); if (ms.has(pid)) { ms.delete(pid); this.msel = ms; const nx = [...ms][0]; this.select(nx ? { p: nx } : null, true); } else { ms.add(pid); this.msel = ms; this.select({ p: pid }, true); } }
    else if (e.shiftKey && this.sel && this.sel.p) { const rows = [...$('olList').querySelectorAll('.olr[data-pid]')].map(r => r.dataset.pid), a = rows.indexOf(this.sel.p), b = rows.indexOf(pid); const ms = new Set(this.msel || []); if (a >= 0 && b >= 0) for (let i = Math.min(a, b); i <= Math.max(a, b); i++) ms.add(rows[i]); this.msel = ms; this.select({ p: pid }, true); }
    else this.select({ p: pid });
    if (this.msel && this.msel.size > 1) this.msg('已选中 ' + this.msel.size + ' 个零件（Del 删除全部）');
    if (e.ctrlKey || e.metaKey || e.shiftKey) this.asmAuto();
  }
  olRename(pid, span) {
    const P = this.part(pid); if (!P || this.mode !== 'edit') return; const inp = document.createElement('input'); inp.type = 'text'; inp.className = 'olin'; inp.value = P.name; span.replaceWith(inp); inp.focus(); inp.select();
    let done = false; const fin = ok => { if (done) return; done = true; const v = inp.value.trim(); if (ok && v && v !== P.name) { this.beginEdit(); P.name = v; this.commit(); this.updProps(); } this.renderOutliner(); };
    inp.onkeydown = e => { e.stopPropagation(); if (e.key === 'Enter') fin(true); if (e.key === 'Escape') fin(false); }; inp.onblur = () => fin(true); inp.onclick = e => e.stopPropagation(); inp.ondblclick = e => e.stopPropagation();
  }
  setHidden(pid, v) { if (this.asm) this.asmCancel(true); const P = this.part(pid); if (!P) return; if (v) P.hidden = true; else delete P.hidden; if (this.objs[pid]) this.objs[pid].visible = !v; this.lastSnap = this.snapshot(); if (this.mode === 'edit') this.init = this.lastSnap; this.renderOutliner(); this.msg((v ? '已隐藏 ' : '已显示 ') + P.name + (v ? '（仍参与物理仿真；在组件栏点 ◌ 显示）' : '')); }
  setLocked(pid, v) { if (this.asm) this.asmCancel(true); const P = this.part(pid); if (!P) return; if (v) P.locked = true; else delete P.locked; this.lastSnap = this.snapshot(); if (this.mode === 'edit') this.init = this.lastSnap; this.updSel(); this.renderOutliner(); this.msg((v ? '已锁定 ' : '已解锁 ') + P.name + (v ? '：不能拖动/旋转（仍可选中和编辑属性）' : '')); }
  deleteParts(ids) { if (this.mode !== 'edit') this.reset(); this.msel = new Set(ids); this.sel = { p: ids[0] }; this.deleteSel(); }
  focusPart(pid) {
    const o = this.objs[pid]; if (!o) return; const bb = new THREE.Box3().setFromObject(o); if (bb.isEmpty()) return;
    const c = bb.getCenter(new THREE.Vector3()), r = Math.max(0.04, bb.getSize(new THREE.Vector3()).length() / 2), dir = this.cam.position.clone().sub(this.orbit.target).normalize();
    this.orbit.target.copy(c); this.cam.position.copy(c).addScaledVector(dir, r / Math.sin(this.cam.fov * D2R / 2) * 1.6); this.orbit.update();
    if (!(this.sel && this.sel.p === pid)) this.select({ p: pid }); this.msg('视角已聚焦到 ' + this.part(pid).name);
  }
  setGizmo(m) { this.tc.setMode(m); if (this.updGizmoBtns) this.updGizmoBtns(); }
  key(e) {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
    const k = e.key.toLowerCase();
    if (this.runKey(e, true)) return;
    if (this.asm && k === 'enter') { e.preventDefault(); this.asmCommit(); return; }
    if (this.asm && k === 'escape') { this.asmCancel(); return; }
    if (this.asm && k === 'tab') { e.preventDefault(); this.asmSetFocus((this.asm.focus || 'a') === 'a' ? 'b' : 'a'); return; }
    if (k === 'j' && !e.ctrlKey && !e.metaKey && !e.altKey) { this.asmToggle(); return; }
    if (this.mode === 'edit' && ['arrowleft', 'arrowright', 'arrowup', 'arrowdown', 'pageup', 'pagedown'].includes(k) && !e.ctrlKey && !e.metaKey) { if (this.sel && this.sel.p) { e.preventDefault(); this.nudgeKey(k, e.shiftKey); } return; }
    if (e.code === 'Space') { e.preventDefault(); this.toggleRun(); }
    else if ((e.ctrlKey || e.metaKey) && k === 'g') { e.preventDefault(); if (e.shiftKey) this.ungroupSel(); else this.groupSel(); }
    else if (k === 'g' && !e.ctrlKey && !e.metaKey && !e.altKey) this.toggleGizmo(e.shiftKey ? 'rotate' : null);
    else if (k === 'm' && !e.ctrlKey && !e.metaKey) this.setTool(this.tool === 'mate' ? 'select' : 'mate');
    else if (k === 'enter' && this.mateS) this.mateCommit();
    else if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); e.shiftKey ? this.redo() : this.undo(); }
    else if ((e.ctrlKey || e.metaKey) && k === 'y') { e.preventDefault(); this.redo(); }
    else if ((e.ctrlKey || e.metaKey) && k === 's') { e.preventDefault(); this.save(); }
    else if ((e.ctrlKey || e.metaKey) && k === 'd') { e.preventDefault(); this.duplicate(); }
    else if (k === 'delete' || k === 'backspace') this.deleteSel();
    else if (k === 'escape' && this.mateS) this.mateCancel();
    else if (k === 'escape') { this.pick1 = null; this.mate1 = null; this.hl2.visible = false; this.armed = null; if (this.tool !== 'select') this.setTool('select'); else this.select(null); }
    else if (k === 'w' && this.mode === 'edit') this.setGizmo('translate'); else if (k === 'e' && this.mode === 'edit') this.setGizmo('rotate');
    else if (k === 'r' && !e.ctrlKey && !e.metaKey && !e.altKey) this.rotateSel(e.shiftKey ? -1 : 1);
    else if (k === 'f') { if (this.sel && this.sel.p) this.focusPart(this.sel.p); else this.fit(); }
  }
  paletteDrag(el, type, over) {
    el.addEventListener('pointerdown', e => {
      if (e.button !== 0) return;
      e.preventDefault(); this.palDrag = true; this.palRot = null; this.rotInfo = ''; this.snapAlt = 0; this.orbit.enableZoom = false; const gh = $('ghost'); gh.textContent = over && over.name ? over.name : DEFS[type].label; gh.style.display = 'block';
      const inCv = ev => { const r = $('cv').getBoundingClientRect(); return ev.clientX > r.left && ev.clientX < r.right && ev.clientY > r.top && ev.clientY < r.bottom; };
      const mv = ev => { gh.style.left = ev.clientX + 8 + 'px'; gh.style.top = ev.clientY + 8 + 'px'; if (inCv(ev) && this.mode === 'edit') { this.previewPalette(type, ev, over); gh.style.opacity = 0.35; } else { this.clearPreview(); gh.style.opacity = ''; } }; mv(e);
      const up = ev => { window.removeEventListener('pointermove', mv); window.removeEventListener('pointerup', up); gh.style.display = 'none'; this.clearPreview();
        const pq = this.palRot ? this.palRot.clone() : null; this.palDrag = false; this.palRot = null; this.rotInfo = ''; this.orbit.enableZoom = true; this._palEv = null;
        if (inCv(ev)) { if (this.mode !== 'edit') this.reset(); this.placePart(type, this.groundPoint(ev), over ? Object.assign(JSON.parse(JSON.stringify(over)), { name: this.uniqueName(over.name || '导入零件') }) : undefined, this.autoOn(ev) ? { ray: this.rayOf(ev), cursor: new THREE.Vector2(ev.clientX, ev.clientY), q: pq } : (pq ? { q: pq } : null)); }
        else if (el.contains(document.elementFromPoint(ev.clientX, ev.clientY))) { document.querySelectorAll('.item.armed').forEach(x => x.classList.remove('armed')); this.armed = type; el.classList.add('armed'); this.msg('点击场景中的位置放置 ' + DEFS[type].label); }
      };
      window.addEventListener('pointermove', mv); window.addEventListener('pointerup', up);
    });
  }
  deleteSel() {
    if (!this.sel || this.mode !== 'edit') return; this.beginEdit();
    if (this.sel.p) { const ids = new Set([this.sel.p, ...(this.msel || [])]); this.M.parts = this.M.parts.filter(p => !ids.has(p.id)); this.M.joints = this.M.joints.filter(j => !ids.has(j.a) && !ids.has(j.b)); }
    if (this.sel.j) this.M.joints = this.M.joints.filter(j => j.id !== this.sel.j);
    if (this.sel.c) this.M.couplings = this.M.couplings.filter(c => c.id !== this.sel.c);
    const jids = new Set(this.M.joints.map(j => j.id)); this.M.couplings = this.M.couplings.filter(c => jids.has(c.ja) && jids.has(c.jb) && this.part(c.pa) && this.part(c.pb));
    this.sel = null; this.msel = new Set(); this.commit(); this.rebuild(); this.msg('已删除');
  }
  duplicate() { if (this.mode === 'edit' && this.selIds().length > 1) return this.cloneSet(this.selIds(), 'copy'); if (!this.sel || !this.sel.p || this.mode !== 'edit') return; const P = this.part(this.sel.p); this.beginEdit(); const C = JSON.parse(JSON.stringify(P)); C.id = 'p' + (this.M.nid++); C.name = this.autoName(P.type); delete C.locked; delete C.hidden; C.p = [P.p[0] + 0.05, P.p[1], P.p[2] + 0.05]; this.M.parts.push(C); this.commit(); this.rebuild(); this.select({ p: C.id }); }
  // ---------------- property panel ----------------
  updProps() {
    const el = $('props'); el.innerHTML = ''; const ed = this.mode === 'edit';
    const row = (label, html, u = '') => { const d = document.createElement('div'); d.className = 'row'; d.innerHTML = `<label>${label}</label>${html}<span class="u">${u}</span>`; el.appendChild(d); return d; };
    const num = (label, val, u, set, step = 'any') => { const r = row(label, `<input type="number" step="${step}" value="${+(+val).toFixed(5)}" ${ed ? '' : 'disabled'}>`, u); if (set) r.querySelector('input').onchange = e => { this.beginEdit(); set(+e.target.value); this.commit(); this.rebuild(); }; return r; };
    if (this.sel && this.sel.p && this.msel && this.msel.size > 1) { this.multiPanel(el, this.selIds()); return; }
    if (!this.sel) {
      el.innerHTML = `<div class="title">装配概览</div><div style="color:#6b7280;line-height:1.6">零件 ${this.M.parts.length} 个，连接 ${this.M.joints.length} 个，传动 ${this.M.couplings.length} 个。<br>中键拖动=旋转视角，右键拖动=平移，滚轮=缩放，左键=选择/拖动零件。<b>装配：Ctrl/Shift+点击 选中两个或多个零件 → 右侧出现“🔧 装配”面板（也可按 J）→ 选连接点（下拉列表或直接点模型）和连接类型（固定/铰链/滑动/球铰/齿轮啮合/皮带/链条/螺旋）→ 确定装配。</b>选中零件只显示高亮，不显示坐标轴；左键直接拖动零件可移动（靠近连接点仍会自动吸附，可在顶栏关闭“自动连接”），R 旋转 15°（Shift+R 反向）；需要精确移动时按 G 显示坐标轴（换选零件后自动隐藏）。🧲 配合(M)：两次点击把零件面贴面装配。📥 导入零件：STL/OBJ/GLB。Ctrl/Shift+点击多选 → 组合 / 复制 / 镜像。</div>`;
      this.listJoints(el, this.M.joints, this.M.couplings); return;
    }
    if (this.sel.p) {
      const P = this.part(this.sel.p); if (!P) return; const D = DEFS[P.type];
      const t = document.createElement('div'); t.className = 'title'; t.textContent = D.label + '：' + P.name; el.appendChild(t);
      const rn = row('名称', `<input type="text" value="${P.name}" ${ed ? '' : 'disabled'}>`); rn.querySelector('input').onchange = e => { this.beginEdit(); P.name = e.target.value; this.commit(); this.updProps(); this.renderOutliner(); };
      const rm = row('材料', `<select ${ed ? '' : 'disabled'}>${Object.entries(MATS).map(([k, m]) => `<option value="${k}" ${k === P.mat ? 'selected' : ''}>${m.n}</option>`).join('')}</select>`);
      rm.querySelector('select').onchange = e => { this.beginEdit(); P.mat = e.target.value; delete P.color; if (!P.massManual) P.mass = defaultMass(P); this.commit(); this.rebuild(); };
      for (const [k, lab, kind] of D.dims) {
        // dimension fields: live preview while typing / spinning / scrubbing the label, one undo step on commit; joints stay matched (resize.js)
        const toUI = v => kind === 'len' ? v * 1000 : v, fromUI = v => kind === 'len' ? v / 1000 : kind === 'int' ? Math.max(3, Math.round(v)) : v;
        const r = num(lab, toUI(P.d[k]), kind === 'len' ? 'mm' : '', null, kind === 'int' ? 1 : 'any'), inp = r.querySelector('input'), lb = r.querySelector('label'), pid = P.id;
        inp.classList.add('dim'); inp.dataset.dim = k;
        const ok = v => v !== '' && isFinite(+v) && (kind === 'int' ? +v >= 3 : +v > 0);
        inp.onchange = null; inp.oninput = () => { if (ok(inp.value)) this.resizeLive(pid, { [k]: fromUI(+inp.value) }); };
        inp.onchange = () => { if (ok(inp.value)) this.resizeCommit(pid, { [k]: fromUI(+inp.value) }); else { this.resizeCancel(); this.updProps(); } };
        if (ed) { lb.classList.add('scrub'); lb.title = '左右拖动标签可连续调整（实时预览，松开后一次撤销）';
          lb.onpointerdown = e => { e.preventDefault(); lb.setPointerCapture(e.pointerId); const x0 = e.clientX, v0 = +inp.value, st = kind === 'int' ? 0.2 : Math.max(0.1, Math.abs(v0) * 0.005); let v = v0;
            lb.onpointermove = ev => { v = v0 + (ev.clientX - x0) * st; v = kind === 'int' ? Math.max(3, Math.round(v)) : Math.max(0.1, +v.toFixed(1)); inp.value = v; this.resizeLive(pid, { [k]: fromUI(v) }); };
            lb.onpointerup = () => { lb.onpointermove = lb.onpointerup = null; if (v !== v0) this.resizeCommit(pid, { [k]: fromUI(v) }); }; }; }
      }
      if (P.type === 'mesh') { const G = GEO[P.d.g], hint = document.createElement('div'); hint.className = 'hint2'; hint.textContent = G ? `导入：${G.name}　尺寸 ${G.box.map(x => (x * 2000 * (P.d.sc || 1)).toFixed(1)).join('×')} mm　${G.tris} 面　体积 ${(G.vol * 1e6 * (P.d.sc || 1) ** 3).toFixed(2)} cm³` : '缺少几何数据'; el.appendChild(hint);
        const rc = row('碰撞体', `<select ${ed ? '' : 'disabled'}><option value="hull">凸包（可运动）</option><option value="tri">三角网格（固定）</option></select>`); const cs = rc.querySelector('select'); cs.value = P.d.col || 'hull'; cs.onchange = e => { this.beginEdit(); P.d.col = e.target.value; if (P.d.col === 'tri') P.fixed = true; this.commit(); this.rebuild(); }; }
      num('质量', P.mass, 'kg', v => { P.mass = Math.max(1e-4, v); P.massManual = true; });
      if (ed) { const b = document.createElement('button'); b.className = 'small'; b.textContent = '按材料密度计算质量'; b.onclick = () => { this.beginEdit(); P.massManual = false; P.mass = defaultMass(P); this.commit(); this.rebuild(); }; el.appendChild(b); }
      const rf = row('固定于大地', `<input type="checkbox" ${P.fixed ? 'checked' : ''} ${ed ? '' : 'disabled'}>`); rf.querySelector('input').onchange = e => { this.beginEdit(); P.fixed = e.target.checked; this.commit(); this.rebuild(); };
      if (P.type === 'motor') {
        const r = row('驱动方式', `<select ${ed ? '' : ''}><option value="speed">转速</option><option value="torque">扭矩</option><option value="off">关闭</option></select>`); const s = r.querySelector('select'); s.value = P.motor.mode; s.onchange = e => { P.motor.mode = e.target.value; this.commit(); };
        const live = (label, key, u) => { const r = row(label, `<input type="number" step="any" value="${P.motor[key]}">`, u); r.querySelector('input').onchange = e => { P.motor[key] = +e.target.value; if (this.sim) { const sp = this.sim.M.parts.find(x => x.id === P.id); if (sp) sp.motor[key] = +e.target.value; } this.commit(); }; };
        live('转速', 'rpm', 'rpm'); live('扭矩(扭矩模式)', 'torque', 'N·m'); live('最大扭矩', 'maxT', 'N·m');
        const hint = document.createElement('div'); hint.className = 'hint2'; hint.textContent = '电机驱动与其相连的“铰链”，正方向为输出轴方向（右手定则）。'; el.appendChild(hint);
      }
      const pos = row('位置', `<input type="number" step="1" value="${(P.p[0] * 1000).toFixed(1)}"><input type="number" step="1" value="${(P.p[1] * 1000).toFixed(1)}"><input type="number" step="1" value="${(P.p[2] * 1000).toFixed(1)}">`, 'mm');
      pos.querySelectorAll('input').forEach((inp, i) => { inp.disabled = !ed; inp.onchange = e => { this.beginEdit(); P.p[i] = +e.target.value / 1000; this.commit(); this.rebuild(); }; });
      const eu = new THREE.Euler().setFromQuaternion(Q4(P.q));
      const rot = row('姿态(欧拉)', [eu.x, eu.y, eu.z].map(v => `<input type="number" step="15" value="${(v / D2R).toFixed(1)}">`).join(''), '°');
      rot.querySelectorAll('input').forEach(inp => { inp.disabled = !ed; inp.onchange = () => { this.beginEdit(); const v = [...rot.querySelectorAll('input')].map(x => +x.value * D2R); P.q = new THREE.Quaternion().setFromEuler(new THREE.Euler(v[0], v[1], v[2])).toArray(); this.commit(); this.rebuild(); }; });
      if (ed) { const rr = row('旋转 15°', `<select id="rotAx" style="flex:none;width:64px"><option value="view">视角轴</option><option value="X">X</option><option value="Y">Y</option><option value="Z">Z</option></select><button class="small" id="bRotM" title="Shift+R">−15°</button><button class="small" id="bRotP" title="R">+15°</button>`); const sa = rr.querySelector('#rotAx'); sa.value = this.rotAxis || 'view'; sa.onchange = e => { this.rotAxis = e.target.value; }; rr.querySelector('#bRotP').onclick = () => this.rotateSel(1); rr.querySelector('#bRotM').onclick = () => this.rotateSel(-1); }
      const pts = D.pts(P.d); const rt = row('轨迹点', `<select>${pts.map((p, i) => `<option value="${i}">${p.n}</option>`).join('')}</select>`); const ts = rt.querySelector('select'); ts.value = P.tracePt != null ? P.tracePt : (P.type === 'rod' ? 1 : 0); ts.onchange = e => { P.tracePt = +e.target.value; this.tracePts = []; };
      if (ed) { const nj = this.M.joints.filter(j => j.a === P.id || j.b === P.id).length + this.M.couplings.filter(c => c.pa === P.id || c.pb === P.id).length; const bb = document.createElement('div'); bb.innerHTML = `<button class="small run" id="bTrial">▶ 试运行 3 秒</button>${nj ? ' <button class="small" id="bDetach">拆开 (' + nj + ')</button>' : ''}<br><button class="small" id="bDup">复制 (Ctrl+D)</button> <button class="del" id="bDel">删除零件 (Del)</button>`; el.appendChild(bb); $('bDup').onclick = () => this.duplicate(); $('bDel').onclick = () => this.deleteSel(); $('bTrial').onclick = () => this.trialRun(3); if (nj) $('bDetach').onclick = () => this.detach(P.id); }
      { const h = document.createElement('h3'); h.textContent = '自定义连接点 (' + (P.cons ? P.cons.length : 0) + ')'; el.appendChild(h);
        (P.cons || []).forEach((c, i) => { const d = document.createElement('div'); d.className = 'jl'; d.innerHTML = `${({ hole: '孔', pin: '轴', mount: '安装面' })[c.role]}：${c.n} <span style="float:right;color:#dc2626" class="cdel" title="删除">✕</span>`; el.appendChild(d); d.querySelector('.cdel').onclick = () => this.removeConnector(P.id, i); });
        if (ed) { const b = document.createElement('button'); b.className = 'small'; b.textContent = '＋ 在表面添加连接点'; b.onclick = () => this.setTool('addcon'); el.appendChild(b); } }
      { const js = this.M.joints.filter(j => j.a === P.id || j.b === P.id), cs = this.M.couplings.filter(c => c.pa === P.id || c.pb === P.id);
        const h = document.createElement('h3'); h.textContent = '连接 (' + (js.length + cs.length) + ')'; h.id = 'connHdr'; el.appendChild(h);
        if (!js.length && !cs.length) { const d = document.createElement('div'); d.className = 'hint2'; d.textContent = '此零件还没有连接。把它拖到其它零件的连接点即可自动装配。'; el.appendChild(d); }
        for (const j of js) { if (isForce(j)) { const d = document.createElement('div'); d.className = 'jl'; d.textContent = `${JT[j.type]}：${(this.part(j.a) || {}).name} ↔ ${j.b ? (this.part(j.b) || {}).name : '大地'}`; d.onclick = () => this.select({ j: j.id }); el.appendChild(d); } else this.jointCard(el, j, js.length === 1, P.id); }
        for (const c of cs) this.couplingRow(el, c); }
    } else if (this.sel.j) {
      const j = this.M.joints.find(x => x.id === this.sel.j); if (!j) return;
      const A = this.part(j.a), B = j.b ? this.part(j.b) : null;
      el.innerHTML = `<div class="title">连接：${JT[j.type]}</div><div style="margin-bottom:6px">${A ? A.name : '?'} ↔ ${B ? B.name : '大地'}　<a href="#" id="selA">选中零件</a></div>`;
      $('selA').onclick = ev => { ev.preventDefault(); this.select({ p: j.a }); };
      if (isForce(j)) {
        if (j.type === 'spring') { num('刚度 k', j.k, 'N/m', v => j.k = v); num('自由长度', j.L0 * 1000, 'mm', v => j.L0 = v / 1000); num('阻尼 c', j.c || 0, 'N·s/m', v => j.c = v); }
        if (j.type === 'damper') num('阻尼 c', j.c, 'N·s/m', v => j.c = v);
        if (ed) { const b = document.createElement('button'); b.className = 'del'; b.textContent = '删除连接'; b.onclick = () => this.deleteJoint(j.id); el.appendChild(b); }
      } else this.jointCard(el, j, true);
      if (ed) { const t = document.createElement('button'); t.className = 'small run'; t.textContent = '▶ 试运行 3 秒'; t.onclick = () => this.trialRun(3); el.appendChild(t); }
    } else if (this.sel.c) {
      const c = this.M.couplings.find(x => x.id === this.sel.c); if (!c) return;
      el.innerHTML = `<div class="title">${CT[c.type]}</div><div>${this.part(c.pa).name} → ${this.part(c.pb).name}</div>`;
      row('传动比', `<span>ω₂/ω₁ = ${c.k.toFixed(5)}</span>`);
      if (c.type === 'gearbox') num('减速比', c.ratio, ':1', v => { c.ratio = v; c.k = Math.sign(c.k) / v; });
      if (c.type === 'clutch') { const r = row('结合', `<input type="checkbox" ${c.engaged ? 'checked' : ''}>`); r.querySelector('input').onchange = e => { c.engaged = e.target.checked; if (this.sim) { const sc = this.sim.couplings.find(x => x.c.id === c.id); if (sc) sc.c.engaged = c.engaged; } }; num('最大传递扭矩', c.maxT, 'N·m', v => c.maxT = v); }
      if (ed) { const d = document.createElement('div'); d.innerHTML = '<button class="small">方向反向</button> <button class="del">删除传动</button>'; el.appendChild(d); d.querySelector('.small').onclick = () => { this.beginEdit(); c.k = -c.k; c.reverse = !c.reverse; this.commit(); this.rebuild(); }; d.querySelector('.del').onclick = () => this.deleteSel(); }
    }
  }
  jointCard(el, j, open, fromPid) {
    const ed = this.mode === 'edit', A = this.part(j.a), B = j.b ? this.part(j.b) : null, other = fromPid ? (fromPid === j.a ? B : A) : B;
    const card = document.createElement('div'); card.className = 'jcard'; card.dataset.jid = j.id; el.appendChild(card);
    const dis = ed ? '' : 'disabled', mot = this.isMotorJ(j);
    const typeSel = `<select class="jtype" ${dis}>${EDIT_T.map(t => `<option value="${t}" ${t === j.type ? 'selected' : ''}>${JT[t]}</option>`).join('')}</select>`;
    card.innerHTML = `<div class="jhead"><span class="jdot t-${j.type}"></span>${typeSel}<span class="jwith">↔ ${other ? other.name : '大地'}</span>${j.mod ? '<span class="jmod">已修改</span>' : ''}</div>`;
    card.querySelector('.jtype').onchange = e => this.setJointType(j.id, e.target.value);
    card.querySelector('.jhead').onmouseenter = () => { const o = this.jobjs[j.id]; if (o) o.scale.setScalar(2); };
    card.querySelector('.jhead').onmouseleave = () => { const o = this.jobjs[j.id]; if (o) o.scale.setScalar(1); };
    const body = document.createElement('div'); body.className = 'jbody'; card.appendChild(body);
    const row = (label, html) => { const d = document.createElement('div'); d.className = 'jrow'; d.innerHTML = `<label>${label}</label>${html}`; body.appendChild(d); return d; };
    const axisd = ['revolute', 'prismatic', 'helical'].includes(j.type);
    if (axisd && A) {
      const n = axW(A, j.axa), near = ['X', 'Y', 'Z'].find((k, i) => Math.abs(Math.abs(n.getComponent(i)) - 1) < 1e-6) || 'C';
      const r = row('轴线', `<select class="jax" ${dis}>${['X', 'Y', 'Z'].map(k => `<option value="${k}" ${near === k ? 'selected' : ''}>${k}</option>`).join('')}<option value="C" ${near === 'C' ? 'selected' : ''}>自定义</option></select><button class="small jflip" ${dis}>反转</button><span class="jaxv">(${n.toArray().map(v => v.toFixed(2)).join(', ')})</span>`);
      r.querySelector('.jax').onchange = e => { if (e.target.value === 'C') { body.querySelector('.jcustom').style.display = ''; return; } const sgn = Math.sign(n.getComponent('XYZ'.indexOf(e.target.value))) || 1; this.setJointAxis(j.id, this.axisVec(e.target.value).multiplyScalar(sgn)); };
      r.querySelector('.jflip').onclick = () => this.flipJoint(j.id);
      const c = row('自定义轴', ['x', 'y', 'z'].map((k, i) => `<input type="number" step="0.1" value="${n.getComponent(i).toFixed(3)}" ${dis}>`).join('') + `<button class="small" ${dis}>应用</button>`); c.classList.add('jcustom'); if (near !== 'C') c.style.display = 'none';
      c.querySelector('button').onclick = () => { const v = new THREE.Vector3(...[...c.querySelectorAll('input')].map(x => +x.value)); if (v.length() > 1e-6) this.setJointAxis(j.id, v); };
    }
    const det = document.createElement('details'); det.className = 'jdet'; if (open) det.open = true; det.innerHTML = '<summary>限位 / 驱动(电机) / 弹簧阻尼 / 按键</summary>'; body.appendChild(det);
    const drow = (label, html) => { const d = document.createElement('div'); d.className = 'jrow'; d.innerHTML = `<label>${label}</label>${html}`; det.appendChild(d); return d; };
    const numIn = (v, cls) => `<input type="number" step="any" class="${cls}" value="${+(+v).toFixed(4)}" ${dis}>`;
    if (j.type === 'revolute' || j.type === 'prismatic') {
      const rv = j.type === 'revolute', k = rv ? 1 / D2R : 1000, u = rv ? '°' : 'mm', L = j.lim || { on: false, lo: rv ? -45 * D2R : -0.05, hi: rv ? 45 * D2R : 0.05 };
      const r = drow('限位', `<input type="checkbox" class="jlimon" ${L.on ? 'checked' : ''} ${dis}>${numIn(L.lo * k, 'jlo')}~${numIn(L.hi * k, 'jhi')}<span class="u">${u}</span>`);
      const upd = () => this.editJoint(j.id, jj => { jj.lim = { on: r.querySelector('.jlimon').checked, lo: +r.querySelector('.jlo').value / k, hi: +r.querySelector('.jhi').value / k }; }, '限位已修改');
      r.querySelectorAll('input').forEach(i => i.onchange = upd);
      if (mot) {
        const m = mot.motor || {}, r2 = drow('电机驱动', `<input type="checkbox" class="jmon" ${!j.noMotor ? 'checked' : ''} ${dis}>${numIn(m.rpm || 0, 'jrpm')}<span class="u">rpm</span>${numIn(m.maxT != null ? m.maxT : 20, 'jmt')}<span class="u">N·m</span>`);
        r2.querySelector('.jmon').onchange = e => this.editJoint(j.id, jj => { jj.noMotor = !e.target.checked; }, e.target.checked ? '电机驱动已启用' : '电机驱动已关闭（铰链自由转动）');
        r2.querySelector('.jrpm').onchange = e => this.editJoint(j.id, () => { mot.motor.rpm = +e.target.value; }, '电机转速已修改');
        r2.querySelector('.jmt').onchange = e => this.editJoint(j.id, () => { mot.motor.maxT = +e.target.value; }, '电机最大扭矩已修改');
        const r3 = drow('转向', `<label class="inl"><input type="checkbox" class="jrev" ${j.motorRev ? 'checked' : ''} ${dis}>反转</label>`); r3.querySelector('.jrev').onchange = e => this.editJoint(j.id, jj => { jj.motorRev = e.target.checked; }, '电机方向已修改');
      } else {
        const Dv = Object.assign({ on: false, mode: 'speed', v: rv ? 30 * Math.PI / 30 : 0.05, maxF: 10, pos: rv ? 25 * D2R : 0.02 }, j.drive || {}), kv = rv ? 30 / Math.PI : 1000, kp = rv ? 1 / D2R : 1000;
        const r2 = drow('驱动', `<input type="checkbox" class="jdon" ${Dv.on ? 'checked' : ''} ${dis}><select class="jdm" ${dis}>${Object.entries(DRIVE_N).map(([k, n]) => `<option value="${k}" ${k === Dv.mode ? 'selected' : ''}>${n}</option>`).join('')}</select>`);
        const r2b = drow(Dv.mode === 'servo' ? '目标' : Dv.mode === 'torque' ? '力/力矩' : '目标速度', Dv.mode === 'servo' ? `${numIn(Dv.pos * kp, 'jdp2')}<span class="u">${rv ? '°' : 'mm'}</span>` : Dv.mode === 'torque' ? `${numIn(Dv.tq != null ? Dv.tq : Dv.maxF, 'jdt')}<span class="u">${rv ? 'N·m' : 'N'}</span>` : `${numIn(Dv.v * kv, 'jdv')}<span class="u">${rv ? 'rpm' : 'mm/s'}</span>`);
        const r2c = drow('最大' + (rv ? '力矩' : '力'), `${numIn(Dv.maxF, 'jdf')}<span class="u">${rv ? 'N·m' : 'N'}</span>`);
        const r2d = drow('键盘', `<input type="text" class="jkp" readonly placeholder="正" value="${keyLabel(Dv.keyP)}" style="width:42px" ${dis}><input type="text" class="jkn" readonly placeholder="反" value="${keyLabel(Dv.keyN)}" style="width:42px" ${dis}><button class="small jkclr" ${dis}>清除</button>`);
        const hint = document.createElement('div'); hint.className = 'hint2'; hint.textContent = Dv.mode === 'servo' ? '舵机：运行时按住正/反键转到 ±目标角度，松开回到 0（未绑定按键时保持目标）。' : Dv.mode === 'torque' ? '扭矩模式：按住键施加 ± 力矩（未绑定按键时持续施加）。' : '速度模式：按住正/反键以 ± 目标速度驱动，松开滑行（未绑定按键时持续转动）。'; det.appendChild(hint);
        const save = () => this.editJoint(j.id, jj => { const o = Object.assign({}, Dv, { on: r2.querySelector('.jdon').checked, mode: r2.querySelector('.jdm').value, maxF: +r2c.querySelector('.jdf').value });
          const a = r2b.querySelector('input'); if (a.classList.contains('jdv')) o.v = +a.value / kv; else if (a.classList.contains('jdt')) o.tq = +a.value; else o.pos = +a.value / kp; jj.drive = o; }, '驱动已修改');
        [r2, r2b, r2c].forEach(r => r.querySelectorAll('input,select').forEach(i => i.onchange = save));
        const bindKey = (cls, prop) => { const inp = r2d.querySelector(cls); inp.onkeydown = e => { e.preventDefault(); e.stopPropagation(); if (e.code === 'Escape') return inp.blur(); this.editJoint(j.id, jj => { jj.drive = Object.assign({}, Dv, jj.drive || {}, { on: true, [prop]: e.code }); }, '已绑定按键 ' + keyLabel(e.code) + '（运行时按住控制此连接）'); }; inp.onfocus = () => { inp.value = '按键…'; }; inp.onblur = () => { inp.value = keyLabel(Dv[prop]); }; };
        bindKey('.jkp', 'keyP'); bindKey('.jkn', 'keyN');
        r2d.querySelector('.jkclr').onclick = () => this.editJoint(j.id, jj => { if (jj.drive) { delete jj.drive.keyP; delete jj.drive.keyN; } }, '已清除按键');
      }
      const r4 = drow('摩擦', `${numIn(j.fric || 0, 'jfr')}<span class="u">${rv ? 'N·m' : 'N'}</span>`); r4.querySelector('input').onchange = e => this.editJoint(j.id, jj => { jj.fric = Math.max(0, +e.target.value); }, '摩擦已修改');
      const r5 = drow('阻尼', `${numIn(j.damp || 0, 'jdp')}<span class="u">${rv ? 'N·m·s' : 'N·s/m'}</span>`); r5.querySelector('input').onchange = e => this.editJoint(j.id, jj => { jj.damp = Math.max(0, +e.target.value); }, '阻尼已修改');
      const r6 = drow('弹簧刚度', `${numIn(j.kc || 0, 'jkc')}<span class="u">${rv ? 'N·m/rad' : 'N/m'}</span>`); r6.querySelector('input').onchange = e => this.editJoint(j.id, jj => { jj.kc = Math.max(0, +e.target.value); }, '弹簧刚度已修改（弹簧+阻尼可做悬架 / 回正）');
      const r6b = drow('平衡位置', `${numIn((j.rest || 0) * k, 'jrest')}<span class="u">${u}</span>`); r6b.querySelector('input').onchange = e => this.editJoint(j.id, jj => { jj.rest = +e.target.value / k; }, '弹簧平衡位置已修改');
      if (rv) { const r7 = drow('', `<button class="small jsteer" ${dis}>设为转向销（竖直轴 + 回正）</button>`); r7.querySelector('button').onclick = () => this.editJoint(j.id, jj => { jj.axa = axL(A, new THREE.Vector3(0, 1, 0)).toArray(); jj.kc = jj.kc || 0.6; jj.damp = jj.damp || 0.08; jj.steer = true; }, '已设为转向铰链（竖直销）'); }
    } else if (j.type === 'helical') {
      const r = drow('导程', `${numIn((j.lead != null ? j.lead : 0.005) * 1000, 'jld')}<span class="u">mm/转</span>`); r.querySelector('input').onchange = e => this.editJoint(j.id, jj => { jj.lead = +e.target.value / 1000; }, '导程已修改');
    } else { const d = document.createElement('div'); d.className = 'hint2'; d.textContent = j.type === 'fixed' ? '固定连接：两零件成为一个刚体（无相对运动）。' : '球铰：三个转动自由度。'; det.appendChild(d); }
    if (ed) { const bb = document.createElement('div'); bb.className = 'jbtns'; bb.innerHTML = '<button class="del jdel">删除连接</button> <button class="small jsplit">拆开</button>'; body.appendChild(bb); bb.querySelector('.jdel').onclick = () => this.deleteJoint(j.id); bb.querySelector('.jsplit').onclick = () => this.splitJoint(j.id); }
  }
  couplingRow(el, c) {
    const ed = this.mode === 'edit', d = document.createElement('div'); d.className = 'jcard'; el.appendChild(d);
    d.innerHTML = `<div class="jhead"><span class="jdot t-gear"></span><b>${CT[c.type]}</b><span class="jwith">${(this.part(c.pa) || {}).name} → ${(this.part(c.pb) || {}).name}</span></div><div class="jrow"><label>${c.type === 'gear' ? '啮合' : '启用'}</label><input type="checkbox" class="con" ${!c.off ? 'checked' : ''} ${ed ? '' : 'disabled'}><span class="u">i=${c.k.toFixed(3)}</span><button class="small cdet">详情</button></div>`;
    d.querySelector('.con').onchange = e => { this.beginEdit(); c.off = !e.target.checked; this.commit(); this.rebuild(); this.msg((CT[c.type]) + (c.off ? ' 已断开' : ' 已接合')); };
    d.querySelector('.cdet').onclick = () => this.select({ c: c.id });
  }
  listJoints(el, js, cs) {
    if (!js.length && !cs.length) return; const h = document.createElement('h3'); h.textContent = '连接 / 传动'; el.appendChild(h);
    for (const j of js) { const d = document.createElement('div'); d.className = 'jl'; d.textContent = `${JT[j.type]}：${(this.part(j.a) || {}).name} ↔ ${j.b ? (this.part(j.b) || {}).name : '大地'}`; d.onclick = () => this.select({ j: j.id }); el.appendChild(d); }
    for (const c of cs) { const d = document.createElement('div'); d.className = 'jl'; d.textContent = `${CT[c.type]}：${(this.part(c.pa) || {}).name} → ${(this.part(c.pb) || {}).name}  i=${c.k.toFixed(3)}`; d.onclick = () => this.select({ c: c.id }); el.appendChild(d); }
  }
  // ---------------- presets / IO ----------------
  loadPreset(i) {
    if (this.asm) this.asmCancel(true);
    if (this.sim) { this.sim.free(); this.sim = null; } this.mode = 'edit'; const old = this.snapshot();
    this.M = { parts: [], joints: [], couplings: [], nid: 1 };
    const api = {
      P: (t, p, r, o) => this.newPart(t, p, r, o),
      J: (t, A, B, w, ax, o) => this.addJoint(t, A, B, w, ax, o),
      C: (t, A, B, e) => this.addCoupling(t, A, B, e),
      rod: (p1, p2, o = {}) => { const a = V3(p1), b = V3(p2), d = b.clone().sub(a); const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1, 0, 0), d.clone().normalize()); o.d = Object.assign({ L: d.length() }, o.d || {}); return this.newPart('rod', a.clone().add(b).multiplyScalar(0.5).toArray(), q.toArray(), o); },
    };
    if (PRESETS[i].app) { const u = this.undoS; this.undoS = []; this.lastSnap = this.snapshot(); this._quietGround = true; try { PRESETS[i].build(this); } finally { this.undoS = u; this._quietGround = false; } } else PRESETS[i].build(api);
    this.groundFix({ quiet: true }); this.undoS.push(old); if (this.undoS.length > 200) this.undoS.shift(); this.redoS = []; this.lastSnap = this.snapshot(); this.init = this.lastSnap; this.sel = null; this.msel = new Set();
    const tr = this.M.parts.find(p => p.trace); this.rebuild(); if (tr) this.select({ p: tr.id }); this.fit(); this.updButtons();
    this.msg('已载入示例：' + PRESETS[i].name + ' — ' + PRESETS[i].desc + '（按 Space 运行）');
  }
  save() { this.download('机械装配3D.json', JSON.stringify(this.saveData()), 'application/json'); this.msg('已保存 JSON'); }
  loadJSON(t) {
    if (this.asm) this.asmCancel(true);
    try { const o = typeof t === 'string' ? JSON.parse(t) : t, M = o.model || o; if (!M.parts) throw new Error('无 parts'); const gs = o.geos || M.geos; if (gs) Object.assign(GEO, gs); delete M.geos; const miss = M.parts.filter(P => P.type === 'mesh' && !GEO[P.d.g]).length; if (miss) this.msg('警告：' + miss + ' 个导入零件缺少几何数据', true); if (this.sim) { this.sim.free(); this.sim = null; } this.mode = 'edit'; this.undoS.push(this.snapshot()); if (this.undoS.length > 200) this.undoS.shift(); this.M = M; M.couplings = M.couplings || []; M.joints = M.joints || []; const gl = this.groundFix({ quiet: true }); this.lastSnap = this.snapshot(); this.init = this.lastSnap; this.sel = null; this.msel = new Set(); this.rebuild(); this.updButtons(); if (o.camera) { this.cam.position.fromArray(o.camera.p); this.orbit.target.fromArray(o.camera.t); } else this.fit(); this.msg('已读取装配：' + M.parts.length + ' 个零件' + (gl.dy > 0.001 ? '（低于地面的装配体已抬升至地面以上）' : '')); }
    catch (e) { this.msg('读取失败：' + e.message, true); }
  }
  bomCSV() {
    const q = s => '"' + String(s).replace(/"/g, '""') + '"', L = [];
    L.push(['序号', '名称', '类型', '材料', '质量(kg)', '尺寸', '固定', '位置(mm)'].map(q).join(','));
    this.M.parts.forEach((P, i) => { const D = DEFS[P.type]; L.push([i + 1, P.name, D.label, (MATS[P.mat] || {}).n, P.mass.toFixed(4), D.dims.map(([k, l, kd]) => l + '=' + (kd === 'len' ? (P.d[k] * 1000).toFixed(1) + 'mm' : P.d[k])).join(' '), P.fixed ? '是' : '否', P.p.map(v => (v * 1000).toFixed(1)).join(' ')].map(q).join(',')); });
    L.push(''); L.push(['连接', '类型', '零件A', '零件B', '参数'].map(q).join(','));
    for (const j of this.M.joints) L.push([j.id, JT[j.type], (this.part(j.a) || {}).name, j.b ? (this.part(j.b) || {}).name : '大地', j.type === 'spring' ? `k=${j.k} L0=${j.L0}` : j.type === 'damper' ? `c=${j.c}` : ''].map(q).join(','));
    for (const c of this.M.couplings) L.push([c.id, CT[c.type], (this.part(c.pa) || {}).name, (this.part(c.pb) || {}).name, 'i=' + c.k.toFixed(5)].map(q).join(','));
    L.push(''); L.push([q('合计质量(kg)'), this.M.parts.reduce((a, p) => a + p.mass, 0).toFixed(3)].join(','));
    return '\ufeff' + L.join('\r\n');
  }
  png() { this.renderer.render(this.scene, this.cam); const a = document.createElement('a'); a.href = this.renderer.domElement.toDataURL('image/png'); a.download = '机械装配3D.png'; a.click(); this.msg('已导出 PNG'); }
  download(name, text, type) { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type })); a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000); }
  fit() {
    const bb = new THREE.Box3(); for (const id in this.objs) bb.expandByObject(this.objs[id]); if (bb.isEmpty()) bb.set(new THREE.Vector3(-0.5, 0, -0.5), new THREE.Vector3(0.5, 0.5, 0.5));
    const c = bb.getCenter(new THREE.Vector3()), r = Math.max(0.25, bb.getSize(new THREE.Vector3()).length() / 2);
    const dir = this.cam.position.clone().sub(this.orbit.target).normalize(); if (!isFinite(dir.x)) dir.set(0.5, 0.4, 1).normalize();
    this.orbit.target.copy(c); this.cam.position.copy(c).addScaledVector(dir, r / Math.sin(this.cam.fov * D2R / 2) * 1.05); this.orbit.update();
  }
  view(v) {
    const dirs = { front: [0, 0, 1], top: [0, 1, 0.0001], side: [1, 0, 0], iso: [0.8, 0.6, 1] }, d = V3(dirs[v]).normalize(), dist = this.cam.position.distanceTo(this.orbit.target);
    this.cam.position.copy(this.orbit.target).addScaledVector(d, dist); this.cam.lookAt(this.orbit.target); this.orbit.update();
  }
  // ---------------- resizing (keeps connections matched) ----------------
  /** compute a resized model from a snapshot; changes = { dimKey: valueSI } */
  computeResize(snap, pid, changes) { const M0 = JSON.parse(snap), P = M0.parts.find(p => p.id === pid); if (!P) return { ok: false, msg: '零件不存在' }; return resizeModel(M0, pid, Object.assign({}, P.d, changes)); }
  /** live preview (no undo entry): always recomputed from the state before the edit started */
  resizeLive(pid, changes) {
    if (this.asm) this.asmCancel(true);
    if (this.mode !== 'edit') this.reset();
    if (!this._rs || this._rs.pid !== pid) this._rs = { pid, snap: this.snapshot() };
    const r = this.computeResize(this._rs.snap, pid, changes);
    this.M = r.ok ? r.M : JSON.parse(this._rs.snap); if (r.ok) this.groundFix({ quiet: true }); this.rebuild(true);
    if (!r.ok) this.msg(r.msg, true); else this.msg('调整尺寸（预览）：' + this.part(pid).name + (r.notes && r.notes.length ? '；' + r.notes.join('；') : ''));
    return r;
  }
  resizeCancel() { if (this._rs) { this.M = JSON.parse(this._rs.snap); this._rs = null; this.rebuild(); } }
  /** apply a resize as one undo step; returns the result ({ok, gap, strategy, msg}) */
  resizeCommit(pid, changes) {
    if (this.asm) this.asmCancel(true);
    if (this.mode !== 'edit') this.reset();
    const base = this._rs && this._rs.pid === pid ? this._rs.snap : this.snapshot(); this._rs = null;
    const r = this.computeResize(base, pid, changes);
    if (!r.ok) { this.M = JSON.parse(base); this.lastSnap = base; this.rebuild(); this.msg(r.msg, true); return r; }
    this.M = r.M; this.lastSnap = base; this.commit(); this.rebuild();
    this.msg('已修改 ' + this.part(pid).name + ' 的尺寸，连接已重新对齐（最大间隙 ' + (r.gap * 1000).toFixed(3) + ' mm）' + (r.notes && r.notes.length ? '；' + r.notes.join('；') : ''));
    return r;
  }
  /** test/API helper */
  resizePart(pid, changes) { return this.resizeCommit(pid, changes); }
  jointGapAll() { return maxJointGap(this.M); }
  // ---------------- test helpers ----------------
  editDrag(pid, pos, quat, steps = 10) {
    this.select({ p: pid }); this.dragStart(); const P = this.part(pid), p0 = V3(P.p), q0 = Q4(P.q), p1 = V3(pos), q1 = quat ? Q4(quat) : q0;
    for (let i = 1; i <= steps; i++) this.dragMove(p0.clone().lerp(p1, i / steps), q0.clone().slerp(q1, i / steps));
    this.dragEnd(); const s = new Sim(this.M, { edit: true }), g = s.maxGap(); s.free(); return g;
  }
  /** visual-connection audit: distance (m) from each joint anchor to the solid geometry of both parts, and parts left without any joint */
  connAudit() {
    const dist = (P, w) => { const l = ptL(P, w); let best = Infinity; for (const sh of DEFS[P.type].shapes(P.d)) { const x = l.clone().sub(V3(sh.pos || [0, 0, 0])); let d;
      if (sh.k === 'box') { const q = [Math.abs(x.x) - sh.h[0], Math.abs(x.y) - sh.h[1], Math.abs(x.z) - sh.h[2]]; d = Math.hypot(...q.map(v => Math.max(0, v))); }
      else if (sh.k === 'cyl') d = Math.hypot(Math.max(0, Math.hypot(x.x, x.y) - sh.r), Math.max(0, Math.abs(x.z) - sh.hh));
      else if (sh.k === 'ball') d = Math.max(0, x.length() - sh.r); else d = 0; best = Math.min(best, d); } return best; };
    const joints = [];
    for (const j of this.M.joints) { if (isForce(j)) continue; const A = this.part(j.a), B = j.b ? this.part(j.b) : null; if (!A) continue; const w = ptW(A, j.la);
      joints.push({ id: j.id, type: j.type, a: A.name, b: B ? B.name : '大地', gapA: dist(A, w), gapB: B ? dist(B, ptW(B, j.lb)) : 0, sep: B ? (() => { const d = ptW(B, j.lb).sub(w); if (j.type === 'prismatic' || j.type === 'helical') { const n = axW(A, j.axa); d.addScaledVector(n, -d.dot(n)); } return d.length(); })() : 0 }); }
    const used = new Set(); for (const j of this.M.joints) { used.add(j.a); if (j.b) used.add(j.b); }
    const loose = this.M.parts.filter(P => !used.has(P.id) && !P.fixed && !P.free).map(P => P.name);
    return { joints, loose, maxGap: joints.reduce((m, j) => Math.max(m, j.gapA, j.gapB, j.sep), 0) };
  }
  editGap() { const s = new Sim(this.M, { edit: true }), g = s.maxGap(); s.free(); return g; }
}

installExtra(App);
installAsm(App);
installMove(App, partMinY);
RAPIER.init().then(() => { window.app = new App(); window.dispatchEvent(new Event('app-ready')); }).catch(e => { console.error(e); document.body.insertAdjacentHTML('beforeend', '<div style="position:fixed;top:40%;left:30%;background:#fee;padding:20px">物理引擎加载失败: ' + e + '</div>'); });
