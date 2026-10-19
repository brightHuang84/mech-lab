// Assembly moving in edit mode: rigid moves of whole connected assemblies / multi-selections,
// world-anchor bookkeeping for ground-fixed parts, arrow-key nudge, numeric centre/rotation, ground lift.
import * as THREE from 'three';
import { DEFS } from './parts.js';
import { V3, Q4, analyze, isForce, axW, ptW, ptL, axL, planCoupling, Sim } from './physics.js';
import { groundFixModel } from './ground.js';

const D2R = Math.PI / 180;

export function installMove(App, partMinY) {
  const P = App.prototype;
  P._partMinY = partMinY;

  /** ids moved together when dragging `pid`:
   *  alt → only that part; otherwise every connected assembly (joints / couplings / fixed groups) touched by
   *  the multi-selection (if pid is in it) or by pid's 组合 group. Keeps every joint intact. */
  P.moveSet = function (pid, o = {}) {
    if (o.alt) return [pid];
    const an = analyze(this.M), ms = this.msel || new Set();
    const seeds = o.seeds || ((ms.size > 1 && (ms.has(pid) || o.multi)) ? [...ms, pid] : this.groupOf(pid));
    const ids = new Set();
    for (const s of seeds) { const c = an.comps[an.compOf[s]]; if (c) c.parts.forEach(p => ids.add(p.id)); else if (this.part(s)) ids.add(s); }
    return [...ids];
  };
  /** after moving parts rigidly: carry along the world-side anchors (lb) of joints that connect them to the ground */
  P.fixWorldAnchors = function (start, ids) {
    const set = new Set(ids); let n = 0;
    for (const j of this.M.joints) {
      if (j.b != null || !set.has(j.a) || !start[j.a]) continue; const Pp = this.part(j.a); if (!Pp) continue;
      const s = start[j.a], l = V3(j.lb).sub(s.p).applyQuaternion(s.q.clone().invert());
      const w = l.applyQuaternion(Q4(Pp.q)).add(V3(Pp.p)); if (w.distanceTo(V3(j.lb)) > 1e-12) { j.lb = w.toArray(); n++; }
    }
    return n;
  };
  P.posesOf = function () { const s = {}; for (const Pp of this.M.parts) s[Pp.id] = { p: V3(Pp.p), q: Q4(Pp.q) }; return s; };
  /** nothing may end up below the floor: lift the moved set (with its world anchors) so its lowest point is at y = 0 */
  //  (uses the parts' real geometry — mesh vertices — and lifts the whole connected assemblies touched by ids, see ground.js)
  P.groundLift = function (ids) { return groundFixModel(this.M, ids).dy; };
  /** detach a part from everything (no commit) */
  P.detachJoints = function (id) {
    const before = this.M.joints.length + this.M.couplings.length;
    this.M.joints = this.M.joints.filter(j => j.a !== id && j.b !== id); const jids = new Set(this.M.joints.map(j => j.id));
    this.M.couplings = this.M.couplings.filter(c => c.pa !== id && c.pb !== id && jids.has(c.ja) && jids.has(c.jb));
    return before - this.M.joints.length - this.M.couplings.length;
  };
  /** visual centre (bounding-box centre) of a set of parts */
  P.selCenter = function (ids = this.selIds()) {
    const bb = new THREE.Box3(), tmp = new THREE.Box3();
    for (const id of ids) { const o = this.objs[id]; if (!o) continue; o.updateMatrixWorld(true); tmp.setFromObject(o); if (!tmp.isEmpty()) bb.union(tmp); }
    if (bb.isEmpty()) { const c = new THREE.Vector3(); ids.forEach(id => c.add(V3(this.part(id).p))); return c.multiplyScalar(1 / Math.max(1, ids.length)); }
    return bb.getCenter(new THREE.Vector3());
  };
  P.lockedIn = function (ids) { return ids.map(id => this.part(id)).filter(Pp => Pp && Pp.locked); };
  /** rigidly transform the selection's assemblies: p' = R·(p − piv) + piv + T. One undo step. */
  P.xformSel = function (R, piv, T, o = {}) {
    if (this.mode !== 'edit') this.reset();
    const pid = this.sel && this.sel.p; if (!pid || !this.part(pid)) { this.msg('先选中零件或装配体', true); return null; }
    const ids = o.ids || this.moveSet(pid, { multi: true });
    const lk = this.lockedIn(ids); if (lk.length) { this.msg(lk.map(x => x.name).join('、') + ' 已锁定，无法移动（在组件栏点 🔒 解锁）', true); return null; }
    this.beginEdit(); const start = this.posesOf();
    for (const id of ids) { const Pp = this.part(id), s = start[id]; Pp.p = s.p.clone().sub(piv).applyQuaternion(R).add(piv).add(T).toArray(); Pp.q = R.clone().multiply(s.q).normalize().toArray(); }
    this.fixWorldAnchors(start, ids); if (o.lift) this.groundLift(ids);
    const keep = new Set(this.msel || []), sel = this.sel;
    this.commit(); this.rebuild(); this.msel = keep; this.sel = sel; this.updSel(); this.updProps();
    return ids;
  };
  P.moveSelBy = function (d, o = {}) { const ids = this.xformSel(new THREE.Quaternion(), new THREE.Vector3(), V3(d), o); if (ids && !o.quiet) this.msg(`已移动 ${ids.length} 个零件 Δ(${d.map(v => (v * 1000).toFixed(0)).join(', ')}) mm · 方向键 5 mm（Shift 50 mm）· PageUp/PageDown 上下`); return ids; };
  P.setSelCenter = function (axis, v) { const c = this.selCenter(this.moveSet(this.sel.p, { multi: true })), d = [0, 0, 0]; d[axis] = v - c.getComponent(axis); return this.moveSelBy(d, { quiet: true }); };
  P.rotateSelBy = function (axis, deg) {
    const ids0 = this.sel && this.sel.p ? this.moveSet(this.sel.p, { multi: true }) : []; if (!ids0.length) return null;
    const c = this.selCenter(ids0), ax = new THREE.Vector3(axis === 'X' ? 1 : 0, axis === 'Y' ? 1 : 0, axis === 'Z' ? 1 : 0);
    const ids = this.xformSel(new THREE.Quaternion().setFromAxisAngle(ax, deg * D2R), c, new THREE.Vector3(), { lift: true });
    if (ids) this.msg(`已将 ${ids.length} 个零件绕 ${axis} 轴旋转 ${deg}°（绕选中中心）`); return ids;
  };
  /** arrow keys: camera-relative horizontal moves snapped to the world X / Z axis; PageUp/PageDown vertical */
  P.nudgeKey = function (k, big) {
    const st = big ? 0.05 : 0.005, d = [0, 0, 0];
    if (k === 'pageup') d[1] = st; else if (k === 'pagedown') d[1] = -st;
    else {
      const f = this.cam.getWorldDirection(new THREE.Vector3()); f.y = 0; if (f.lengthSq() < 1e-8) f.set(0, 0, -1); f.normalize();
      const r = new THREE.Vector3().crossVectors(f, new THREE.Vector3(0, 1, 0));
      const snapAx = v => { const i = Math.abs(v.x) >= Math.abs(v.z) ? 0 : 2, o = [0, 0, 0]; o[i] = Math.sign(i === 0 ? v.x : v.z); return o; };
      const v = k === 'arrowright' ? snapAx(r) : k === 'arrowleft' ? snapAx(r.negate()) : k === 'arrowup' ? snapAx(f) : snapAx(f.negate());
      for (let i = 0; i < 3; i++) d[i] = v[i] * st;
    }
    return this.moveSelBy(d, { lift: k === 'pagedown' });
  };
  /** test/API helper: drag the gizmo on the current (multi-)selection by delta / rotation */
  P.gizmoDragSel = function (delta, rotQ, steps = 8) {
    if (!this.gizmoOn) this.toggleGizmo('translate'); this.updSel();
    const p0 = this.proxy.position.clone(), q0 = this.proxy.quaternion.clone(), p1 = p0.clone().add(V3(delta || [0, 0, 0])), q1 = rotQ ? Q4(rotQ).multiply(q0) : q0.clone();
    this.dragStart(); for (let i = 1; i <= steps; i++) { this.proxy.position.copy(p0.clone().lerp(p1, i / steps)); this.proxy.quaternion.copy(q0.clone().slerp(q1, i / steps)); this.dragMove(this.proxy.position.clone(), this.proxy.quaternion.clone()); }
    this.dragEnd(); return this.editGap();
  };

  // ---------------- connection hygiene (auto hinges, duplicates, visual separation) ----------------
  const GEARISH = new Set(['gear', 'bevel', 'pulley', 'sprocket', 'worm', 'wormwheel']);
  /** a ground hinge that was auto-added to a gear (mesh / belt helpers) becomes redundant once the gear sits on a shaft that has its own
   *  coaxial revolute support → remove it (couplings are re-planned onto the real support). If the gear is fixed to a shaft but the shaft has
   *  no support, the ground hinge is moved onto the shaft (shows as 轴 ↔ 大地 = bearing), same axis. Returns number of changes. */
  P.pruneAutoHinges = function () {
    let n = 0;
    for (const j of this.M.joints.slice()) {
      if (j.type !== 'revolute' || j.b != null || !this.M.joints.includes(j)) continue; const X = this.part(j.a); if (!X || !(j.auto || GEARISH.has(X.type))) continue;
      const an = analyze(this.M), g = an.groupOf[X.id]; if (!g || g.parts.length < 2) continue; const ids = new Set(g.parts.map(p => p.id));
      const axJ = axW(X, j.axa).normalize(), pJ = ptW(X, j.la);
      const onAxis = (p, ax) => { const d = pJ.clone().sub(p); d.addScaledVector(ax, -d.dot(ax)); return d.length() < 0.001; };
      const coax = r => { const R = this.part(r.a); if (!R) return false; const ax = axW(R, r.axa).normalize(); return Math.abs(ax.dot(axJ)) > 0.999 && onAxis(ptW(R, r.la), ax); };
      // the other support must really hold the group: to the ground, or to a ground-fixed (static) body
      const grounded = r => { if (r.b == null) return ids.has(r.a); const o = ids.has(r.a) ? r.b : ids.has(r.b) ? r.a : null; return !!o && !ids.has(o) && !!an.groupOf[o] && an.groupOf[o].static; };
      const other = this.M.joints.find(r => r !== j && r.type === 'revolute' && grounded(r) && coax(r));
      if (other) {
        const bj = JSON.stringify(this.M.joints), bc = JSON.stringify(this.M.couplings); let ok = true;
        this.M.joints = this.M.joints.filter(x => x !== j);
        for (const c of this.M.couplings) if (c.ja === j.id || c.jb === j.id) { const r = planCoupling(this.M, c.type, this.part(c.pa), this.part(c.pb), { ratio: c.ratio }); if (typeof r === 'string') { ok = false; break; } Object.assign(c, r); }
        if (ok) n++; else { this.M.joints = JSON.parse(bj); this.M.couplings = JSON.parse(bc); }
        continue;
      }
      if (GEARISH.has(X.type)) { // move the ground bearing from the gear onto the coaxial shaft it is fixed to
        const S = g.parts.find(Q => Q.id !== X.id && Q.type === 'shaft' && Math.abs(axW(Q, [0, 0, 1]).dot(axJ)) > 0.999 && onAxis(V3(Q.p), axW(Q, [0, 0, 1]).normalize()));
        if (S) { j.a = S.id; j.la = ptL(S, pJ).toArray(); j.axa = axL(S, axJ).toArray(); j.auto = true; n++; }
      }
    }
    return n;
  };
  /** same type + same pair + same anchor (±1 mm) */
  P.findDupJoint = function (type, A, B, W) {
    return this.M.joints.find(j => j.type === type && ((j.a === A.id && (j.b || null) === (B ? B.id : null)) || (B && j.a === B.id && j.b === A.id)) && this.part(j.a) && ptW(this.part(j.a), j.la).distanceTo(W) < 0.001) || null;
  };
  /** which side of a two-part connection may be moved: a free assembly, else one whose anchoring is only "fixed" flags (no ground joints), smaller first */
  P.pickMover = function (A, B) {
    const an = analyze(this.M), cA = an.comps[an.compOf[A.id]], cB = an.comps[an.compOf[B.id]];
    if (!cA || !cB || cA === cB) return null;
    const gj = c => this.M.joints.some(j => j.b == null && !isForce(j) && c.parts.some(p => p.id === j.a));
    if (!cA.anchored) return { side: 'a', comp: cA }; if (!cB.anchored) return { side: 'b', comp: cB };
    const ok = [['a', cA], ['b', cB]].filter(([, c]) => !gj(c)).sort((x, y) => x[1].parts.length - y[1].parts.length);
    return ok.length ? { side: ok[0][0], comp: ok[0][1] } : null;
  };
  /** after an assembly / mate: every new joint must sit on both parts (≤ 1 mm) and the edit gap must be < 0.5 mm.
   *  Tries one relax pass to correct; returns { ok, gap, sep, msg } */
  P.validateNew = function (jids, allow = 0) {
    const chk = () => { const au = this.connAudit(), js = au.joints.filter(x => jids.includes(x.id)), sep = js.reduce((m, x) => Math.max(m, x.gapA, x.gapB, x.sep), 0); const s = new Sim(this.M, { edit: true }), gap = s.maxGap(); s.free(); return { gap, sep }; };
    let r = chk(); if (r.gap >= 0.0005) { this.relaxAll(); r = chk(); }
    r.ok = r.gap < 0.0005 && r.sep <= 0.001 + allow;
    if (!r.ok) r.msg = r.sep > 0.001 + allow ? `连接点相距 ${(r.sep * 1000).toFixed(1)} mm（零件没有贴合）` : `连接间隙 ${(r.gap * 1000).toFixed(2)} mm 超过 0.5 mm`;
    return r;
  };
}
