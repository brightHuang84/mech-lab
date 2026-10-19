// 装配面板 (panel-based assembly): select 2+ parts -> pick connection points + connection type -> live preview -> 确定装配.
// Replaces relying on drag-to-align. Works on a snapshot: every option change re-applies the whole plan to a copy of the
// model (preview), 确定 commits it as ONE undo step, 取消 restores the snapshot.
import * as THREE from 'three';
import { DEFS, pitchRadius } from './parts.js';
import { V3, Q4, axW, analyze, isForce } from './physics.js';
import { connectors, meshCandidate, hasJoint, anyPerp } from './assemble.js';

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const mm1 = x => Math.round(x * 1000) / 1000;
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/"/g, '&quot;');
const SUF = { hole: '孔', hub: '轮毂孔', pin: '销', bore: '轴承孔', axleEnd: '轴端', axleMid: '轴中心', axleLine: '沿轴身·位置可调', holeLine: '沿杆身·位置可调', screwLine: '沿螺纹·位置可调', rail: '沿导轨·位置可调', mount: '面', slider: '滑块底面', shaftOut: '电机输出轴', motorBase: '电机底座', mountBase: '安装底面', kingpin: '转向销', ball: '球心', nut: '螺母孔', ref: '参考点' };
// connectors that never "stack" (an axle passes through, a base sits flush ...)
const NOSTACK = new Set(['axleLine', 'axleMid', 'axleEnd', 'shaftOut', 'screwLine', 'nut', 'rail', 'slider', 'motorBase', 'mountBase', 'kingpin', 'ball', 'ref']);
// connectors whose axis points out of the part (mate face-to-face)
const OUTR = new Set(['axleEnd', 'motorBase', 'mountBase', 'kingpin', 'slider']);
export const ATYPE = { fixed: '固定', revolute: '铰链(转动)', prismatic: '滑动', spherical: '球铰', gear: '齿轮啮合', belt: '皮带', chain: '链条', helical: '螺旋', keep: '保留现有连接（不新增）' };
const EDIT_T = { revolute: '铰链', prismatic: '滑动副', fixed: '固定连接', spherical: '球铰', helical: '螺旋副' };
const CTN = { gear: '啮合', belt: '皮带', chain: '链条', screw: '螺旋副', gearbox: '变速箱联动', clutch: '离合器', ujoint: '万向节联动' };
// smart defaults: unordered role pair -> [joint type, score]
const PRI = [
  ['hub', 'axleLine', 'fixed', 100], ['nut', 'screwLine', 'helical', 97], ['hub', 'shaftOut', 'revolute', 96], ['axleLine', 'bore', 'revolute', 96], ['axleMid', 'bore', 'revolute', 94],
  ['hub', 'bore', 'revolute', 92], ['slider', 'rail', 'prismatic', 92], ['hole', 'shaftOut', 'revolute', 92], ['hole', 'pin', 'revolute', 91], ['hole', 'hole', 'revolute', 90], ['axleEnd', 'bore', 'revolute', 90],
  ['motorBase', 'mount', 'fixed', 88], ['mountBase', 'mount', 'fixed', 88], ['hub', 'pin', 'revolute', 85], ['axleEnd', 'shaftOut', 'revolute', 85], ['hub', 'hole', 'revolute', 80], ['axleEnd', 'hole', 'revolute', 80], ['hole', 'bore', 'revolute', 80],
  ['kingpin', 'mount', 'revolute', 75], ['hole', 'holeLine', 'revolute', 70], ['axleEnd', 'holeLine', 'revolute', 66], ['hub', 'holeLine', 'revolute', 65], ['pin', 'holeLine', 'revolute', 65], ['ball', 'hole', 'fixed', 60],
  ['axleEnd', 'mount', 'fixed', 50], ['hub', 'mount', 'revolute', 45], ['hole', 'mount', 'revolute', 40], ['pin', 'mount', 'revolute', 40], ['pin', 'pin', 'fixed', 35], ['mount', 'mount', 'fixed', 30]];

/** every connection point of a part as an assembly entry (local frame) */
export function asmEntries(P) {
  const L = [];
  connectors(P).forEach((c, i) => {
    const e = { key: 'c' + i, role: c.plane ? 'mount' : c.role, custom: !!c.custom };
    if (c.plane) Object.assign(e, { kind: 'plane', p: c.p, n: c.a, u: c.u, hu: c.hu, hv: c.hv, out: true, stack: 0, sec: c.u, slide: c.u });
    else if (c.line && c.jax) Object.assign(e, { kind: 'line', p: c.p, dir: c.a, len: c.len, n: c.jax, out: false, stack: c.half || 0, slide: c.a });
    else if (c.line && c.role === 'rail') Object.assign(e, { kind: 'line', p: c.p, dir: c.a, len: c.len, n: c.nrm, out: true, stack: null, sec: c.a, slide: c.a });
    else if (c.line) Object.assign(e, { kind: 'line', p: c.p, dir: c.a, len: c.len, n: c.a, out: false, stack: null, slide: c.a });
    else { const out = c.side === 1 || OUTR.has(c.role); Object.assign(e, { kind: 'pt', p: c.p, n: c.a, out, stack: NOSTACK.has(c.role) ? null : out ? 0 : (c.half || 0), sec: ['slider', 'motorBase', 'mountBase'].includes(c.role) ? [1, 0, 0] : null, slide: c.role === 'slider' ? [1, 0, 0] : null }); }
    e.name = c.n + '（' + (c.custom ? '自定义' : '') + (SUF[e.role] || e.role) + '）';
    L.push(e);
  });
  const D = DEFS[P.type];
  if (D && D.pts) D.pts(P.d).forEach((pt, i) => {
    if (L.some(e => V3(e.p).distanceTo(V3(pt.p)) < 1e-3)) return;
    L.push({ key: 'r' + i, role: 'ref', kind: 'pt', p: pt.p, n: pt.a || [0, 0, 1], out: false, stack: null, name: pt.n + '（参考点）' });
  });
  return L;
}
function eLocal(e, prm = {}) {
  const p = V3(e.p);
  if (e.kind === 'line') p.addScaledVector(V3(e.dir).normalize(), prm.s || 0);
  else if (e.kind === 'plane') { const U = V3(e.u).normalize(), Vv = V3(e.n).normalize().cross(U).normalize(); p.addScaledVector(U, prm.u || 0).addScaledVector(Vv, prm.v || 0); }
  return p;
}
/** world point / axis / secondary axis / slide direction of an entry */
export function eWorld(P, e, prm) {
  const q = Q4(P.q), rot = a => a ? V3(a).applyQuaternion(q).normalize() : null;
  return { p: eLocal(e, prm).applyQuaternion(q).add(V3(P.p)), n: rot(e.n), sec: rot(e.sec), slide: rot(e.slide), dir: rot(e.dir), u: rot(e.u) };
}
function clampPrm(e, prm) {
  if (e.kind === 'line') return { s: clamp(prm.s || 0, -e.len / 2, e.len / 2) };
  if (e.kind === 'plane') return { u: clamp(prm.u || 0, -e.hu, e.hu), v: clamp(prm.v || 0, -e.hv, e.hv) };
  return {};
}
/** parameter on entry e (line/plane) closest to world point w */
function projPrm(P, e, w, margin = 0) {
  const g = eWorld(P, e, {}), d = w.clone().sub(g.p);
  if (e.kind === 'line') { const s = d.dot(g.dir), perp = d.clone().addScaledVector(g.dir, -s).length(); if (perp > 0.3) return { s: 0 }; const h = Math.max(0, e.len / 2 - margin); return { s: mm1(clamp(s, -h, h)) }; }
  if (e.kind === 'plane') { if (Math.abs(d.dot(g.n)) > 0.3) return { u: 0, v: 0 }; const V = g.n.clone().cross(g.u).normalize(), mg = 0.002; return { u: mm1(clamp(d.dot(g.u), -e.hu + mg, e.hu - mg)), v: mm1(clamp(d.dot(V), -e.hv + mg, e.hv - mg)) }; }
  return {};
}
/**
 * rigid transform that brings mover F's entry onto target T's entry.
 * o: { axisW (forced shared axis), flip, other (stack on the other side), off (m), ang (deg) }
 */
export function alignXf(T, te, tprm, F, fe, fprm, o = {}) {
  const tw = eWorld(T, te, tprm), fw = eWorld(F, fe, fprm);
  let Ax = tw.n.clone(); if (o.axisW) { Ax = o.axisW.clone().normalize(); if (Ax.dot(tw.n) < 0) Ax.negate(); }
  let D = te.out && fe.out ? Ax.clone().negate() : (fw.n.dot(Ax) >= 0 ? Ax.clone() : Ax.clone().negate());
  if (o.flip) D.negate();
  const R = new THREE.Quaternion().setFromUnitVectors(fw.n, D);
  if (tw.sec && fw.sec) {
    const s1 = fw.sec.clone().applyQuaternion(R), pa = tw.sec.clone().addScaledVector(D, -tw.sec.dot(D)), pb = s1.addScaledVector(D, -s1.dot(D));
    if (pa.length() > 1e-6 && pb.length() > 1e-6) { pa.normalize(); pb.normalize(); let best = null; for (const sg of [1, -1]) { const t = pa.clone().multiplyScalar(sg), a = Math.atan2(pb.clone().cross(t).dot(D), pb.dot(t)); if (best === null || Math.abs(a) < Math.abs(best) - 1e-9) best = a; } R.premultiply(new THREE.Quaternion().setFromAxisAngle(D, best)); }
  }
  if (o.ang) R.premultiply(new THREE.Quaternion().setFromAxisAngle(D, o.ang * D2R));
  let w;
  if (te.out) w = Ax.clone();
  else if (fe.out) w = fw.n.clone().applyQuaternion(R).negate();
  else { const side = Math.sign(fw.p.clone().sub(tw.p).dot(Ax)) || 1; w = Ax.clone().multiplyScalar(o.other ? -side : side); }
  const sa = te.stack, sb = fe.stack, stand = sa != null && sb != null && sa + sb > 1e-9 ? sa + sb + 0.0005 : 0;
  const L = tw.p.clone().addScaledVector(w, stand + (o.off || 0));
  return { R, pivot: fw.p.clone(), L, axis: Ax, D, w, tw, fw, stand };
}
function moveParts(parts, R, pivot, L, M) {
  for (const P of parts) { P.p = V3(P.p).sub(pivot).applyQuaternion(R).add(L).toArray(); P.q = R.clone().multiply(Q4(P.q)).normalize().toArray(); }
  // ground joints (b = null) store their world anchor in lb: carry it along so the moved assembly stays consistent
  if (M) { const ids = new Set(parts.map(P => P.id)); for (const j of M.joints) if (j.b == null && ids.has(j.a) && Array.isArray(j.lb)) j.lb = V3(j.lb).sub(pivot).applyQuaternion(R).add(L).toArray(); }
}
function ensureRot(app, P, axis, center) { if (!hasJoint(app.M, P.id, ['revolute'])) app.addJoint('revolute', P, null, center.clone(), axis.clone(), { auto: true }); }
const dummyCtx = { toScreen: v => new THREE.Vector2(v.x, v.y), cursor: null };
function ghostify(o) { if (!o) return; o.traverse(m => { if (m.isMesh && m.material) { m.material = m.material.clone(); m.material.transparent = true; m.material.opacity = 0.55; m.castShadow = false; } }); }

export function installAsm(App) {
  const P = App.prototype;
  P.asmEnts = function (pid) { const Pp = this.part(pid); return Pp ? asmEntries(Pp) : []; };
  P.asmEntry = function (pid, key) { return this.asmEnts(pid).find(e => e.key === key) || null; };
  P.asmPointW = function (pid, key, prm) { const e = this.asmEntry(pid, key), Pp = this.part(pid); return e && Pp ? eWorld(Pp, e, prm || {}) : null; };
  /** selection collapsed to one id per Ctrl+G group (the clicked part represents its group) */
  P.asmUnits = function (ids) {
    const out = [], seen = new Set(), sp = this.sel && this.sel.p;
    for (const id of ids) { const Pp = this.part(id); if (!Pp) continue; if (Pp.grp) { if (seen.has(Pp.grp)) continue; seen.add(Pp.grp); const g = ids.filter(x => (this.part(x) || {}).grp === Pp.grp); out.push(g.includes(sp) ? sp : id); } else out.push(id); }
    return out;
  };
  P.asmLinked = function (pr, M = this.M) { const s = new Set([pr.a, pr.b]); return M.joints.some(j => s.has(j.a) && s.has(j.b) && j.a !== j.b) || M.couplings.some(c => s.has(c.pa) && s.has(c.pb) && c.pa !== c.pb); };
  P.asmMeshOK = function (A, B) { try { return !!meshCandidate(dummyCtx, B, A, V3(B.p), Q4(B.q), V3(A.p), Q4(A.q)); } catch (e) { return false; } };
  P.asmTypes = function (pr) {
    const A = this.part(pr.a), B = this.part(pr.b), L = ['fixed', 'revolute', 'prismatic', 'spherical'];
    if (A && B && this.asmMeshOK(A, B)) L.push('gear');
    if (A && B && A.type === 'pulley' && B.type === 'pulley') L.push('belt');
    if (A && B && A.type === 'sprocket' && B.type === 'sprocket') L.push('chain');
    L.push('helical'); if (this.asmLinked(pr, this.asm ? JSON.parse(this.asm.snap) : this.M)) L.push('keep');
    return L;
  };
  /** smart default: most sensible point pair + connection type for parts pr.a (stays) and pr.b (moves) */
  P.asmDefault = function (pr) {
    const A = this.part(pr.a), B = this.part(pr.b);
    Object.assign(pr, { axis: 'auto', flip: false, other: false, off: 0, ang: 0, dist: null, pa: {}, pb: {}, ka: null, kb: null, type: 'fixed' });
    if (!A || !B) return pr;
    const EA = asmEntries(A), EB = asmEntries(B); let best = null;
    for (const ea of EA) for (const eb of EB) {
      let sc = 0, ty = 'fixed';
      for (const [r1, r2, t, s] of PRI) if (((ea.role === r1 && eb.role === r2) || (ea.role === r2 && eb.role === r1)) && s > sc) { sc = s; ty = t; }
      if (!sc) sc = (ea.role === 'ref' ? 0 : 2) + (eb.role === 'ref' ? 0 : 2);
      if (/^中点/.test(ea.name)) sc -= 5; if (/^中点/.test(eb.name)) sc -= 5;
      const a0 = eWorld(A, ea, {}).p, b0 = eWorld(B, eb, {}).p;
      const pa = projPrm(A, ea, b0, eb.stack || 0), pb = projPrm(B, eb, a0, ea.stack || 0);
      const wa = eWorld(A, ea, pa), wb = eWorld(B, eb, pb);
      sc -= wa.p.distanceTo(wb.p) * 10;
      if (ea.out && eb.out) sc += 8 * -wa.n.dot(wb.n); // faces already facing each other need the least re-orientation
      if (!best || sc > best.sc + 1e-9) best = { sc, ty, ea, eb, pa, pb };
    }
    if (best) Object.assign(pr, { ka: best.ea.key, kb: best.eb.key, pa: best.pa, pb: best.pb, type: best.ty });
    const types = this.asmTypes(pr);
    if (types.includes('keep')) pr.type = 'keep';
    else if (types.includes('gear')) pr.type = 'gear';
    else if (types.includes('belt')) pr.type = 'belt';
    else if (types.includes('chain')) pr.type = 'chain';
    else if ((A.type === 'screw' && B.type === 'nut') || (A.type === 'nut' && B.type === 'screw')) pr.type = 'helical';
    return pr;
  };
  /** apply one pair to the current model (moves the mover's connected group, creates the connection) */
  P.asmApplyPair = function (pr) {
    const A = this.part(pr.a), B = this.part(pr.b), info = { moved: [], jids: [], cids: [], note: '', mover: null };
    if (!A || !B) { info.err = '零件不存在'; return info; }
    if (pr.type === 'keep') { info.note = '保留现有连接，不移动'; return info; }
    const an = analyze(this.M), cA = an.comps[an.compOf[A.id]], cB = an.comps[an.compOf[B.id]], same = an.compOf[A.id] === an.compOf[B.id];
    const gj = c => this.M.joints.some(j => j.b == null && !isForce(j) && c.parts.some(p => p.id === j.a));
    const lone = c => c.parts.length === 1 && !this.M.joints.some(j => j.a === c.parts[0].id || j.b === c.parts[0].id);
    let mv = null;
    if (same) info.note = '两零件已在同一装配体中：不移动，直接建立连接（必须已经对齐）';
    else if (!cB.anchored || (lone(cB) && !gj(cB))) mv = 'b';
    else if (!cA.anchored) mv = 'a';
    else if (!gj(cB)) mv = 'b';
    else if (!gj(cA)) mv = 'a';
    else { mv = 'b'; info.note = '两侧都已连接到大地：整体移动 B 所在装配体（连同其接地连接）'; }
    if (mv === 'a') info.note = B.name + ' 所在装配体已固定，改为移动 ' + A.name;
    const F = mv === 'a' ? A : B, T = mv === 'a' ? B : A, comp = mv ? (mv === 'a' ? cA : cB) : null;
    const eA = asmEntries(A).find(e => e.key === pr.ka), eB = asmEntries(B).find(e => e.key === pr.kb);
    const nj = this.M.joints.length, nc = this.M.couplings.length, t = pr.type, nid0 = this.M.nid;
    if (t === 'gear') {
      const G = F, H = T, hq = Q4(H.q), hp = V3(H.p), nH = new THREE.Vector3(0, 0, 1).applyQuaternion(hq);
      // bevel pairs: the azimuth (方位角) is applied after meshCandidate snaps the pitch-plane direction (exact 90°, no residual tilt)
      const bev = G.type === 'bevel' && H.type === 'bevel', q0 = Q4(G.q).normalize();
      let p0 = V3(G.p); if (pr.ang && !bev) p0 = p0.sub(hp).applyAxisAngle(nH, pr.ang * D2R).add(hp);
      const c = meshCandidate(bev ? Object.assign({}, dummyCtx, { az: (pr.ang || 0) * D2R }) : dummyCtx, G, H, p0, q0, hp, hq);
      if (!c) { info.err = '这两个零件不能啮合（需模数相同的直齿轮 / 齿条 / 蜗杆蜗轮 / 锥齿轮）'; return info; }
      if (mv) moveParts(comp.parts, c.q.clone().multiply(q0.clone().invert()), V3(G.p), c.p.clone(), this.M);
      info.label = c.label; if (!c.apply(this)) info.err = '啮合失败（请检查两齿轮是否已用铰链安装）';
    } else if (t === 'belt' || t === 'chain') {
      const nT = axW(T, [0, 0, 1]), cT = V3(T.p), minD = pitchRadius(T) + pitchRadius(F) + 0.01;
      if (mv) {
        const nF = axW(F, [0, 0, 1]), R = new THREE.Quaternion().setFromUnitVectors(nF, nF.dot(nT) >= 0 ? nT : nT.clone().negate());
        let u = V3(F.p).sub(cT); u.addScaledVector(nT, -u.dot(nT)); const d0 = u.length(); if (d0 < 1e-6) u = V3(anyPerp(nT.toArray())); else u.normalize();
        if (pr.ang) u.applyAxisAngle(nT, pr.ang * D2R);
        const dist = pr.dist != null ? Math.max(minD, pr.dist) : Math.max(d0, minD + 0.04); info.dist = dist;
        moveParts(comp.parts, R, V3(F.p), cT.clone().addScaledVector(u, dist), this.M);
      } else info.dist = V3(F.p).distanceTo(cT);
      ensureRot(this, A, axW(A, [0, 0, 1]), V3(A.p)); ensureRot(this, B, axW(B, [0, 0, 1]), V3(B.p));
      if (!this.addCoupling(t, A, B)) info.err = '无法建立' + ATYPE[t];
      info.label = ATYPE[t] + '（中心距 ' + (info.dist * 1000).toFixed(1) + ' mm）';
    } else {
      if (!eA || !eB) { info.err = '请选择两个零件的连接点'; return info; }
      const te = mv === 'a' ? eB : eA, fe = mv === 'a' ? eA : eB, tprm = clampPrm(te, mv === 'a' ? pr.pb : pr.pa), fprm = clampPrm(fe, mv === 'a' ? pr.pa : pr.pb);
      const forced = pr.axis && pr.axis !== 'auto' ? this.axisVec(pr.axis) : null;
      const X = alignXf(T, te, tprm, F, fe, fprm, { axisW: t === 'prismatic' ? null : forced, flip: pr.flip, other: pr.other, off: pr.off, ang: pr.ang });
      if (mv) {
        if (fe.role && ['motorBase', 'mountBase'].includes(fe.role) && te.role === 'mount' && F.fixed && comp.parts.length === 1) F.fixed = false;
        moveParts(comp.parts, X.R, X.pivot, X.L, this.M);
      }
      const anchor = mv ? X.L.clone() : X.tw.p.clone(); info.stand = X.stand;
      let jax = X.axis.clone();
      if (t === 'prismatic') jax = forced || X.tw.slide || eWorld(F, fe, fprm).slide || X.axis.clone();
      if (t === 'helical' && ((A.type === 'screw' && B.type === 'nut') || (A.type === 'nut' && B.type === 'screw'))) {
        const S = A.type === 'screw' ? A : B, N = S === A ? B : A, sAx = axW(S, [0, 0, 1]);
        if (!hasJoint(this.M, S.id, ['revolute'])) this.addJoint('revolute', S, null, V3(S.p), sAx);
        if (!hasJoint(this.M, N.id, ['prismatic'])) this.addJoint('prismatic', N, null, anchor, sAx);
        if (!this.addCoupling('screw', S, N)) info.err = '无法建立螺旋副';
      } else { this._dupJoint = null; this.addJoint(t, A, B, anchor, jax, t === 'helical' ? { lead: 0.005 } : {}); if (this._dupJoint) { info.dup = true; info.note = '已有相同的连接，未重复添加'; this._dupJoint = null; } }
      info.label = ATYPE[t];
      const wa = eWorld(A, eA, clampPrm(eA, pr.pa)), wb = eWorld(B, eB, clampPrm(eB, pr.pb)); const d = wb.p.clone().sub(wa.p);
      info.gap = d.length(); info.radial = d.clone().addScaledVector(X.axis, -d.dot(X.axis)).length();
      // never join visually separated parts: if nothing could be moved, the two points must already coincide (stack / offset allowed)
      if (!mv && info.gap > 0.001 + (X.stand || 0) + Math.abs(pr.off || 0)) {
        this.M.joints.splice(nj); this.M.couplings.splice(nc); this.M.nid = nid0;
        info.err = `${A.name} 与 ${B.name} 相距 ${(info.gap * 1000).toFixed(1)} mm，且${same ? '已在同一装配体中' : '都已连接到大地'}无法移动对齐：未建立连接（请先拆开或移动其中一个）`;
      }
    }
    info.jids = this.M.joints.slice(nj).map(j => j.id); info.cids = this.M.couplings.slice(nc).map(c => c.id);
    if (mv) { info.moved = comp.parts.map(p => p.id); info.mover = F.id; const dy = this.liftComp(F.id); if (dy > 0.001) info.lifted = dy; }
    else if (!info.err) { const dy = this.liftComp(A.id); if (dy > 0.001) info.lifted = dy; }
    return info;
  };

  // ---------------- open / preview / commit / cancel ----------------
  P.asmOpen = function (ids) {
    ids = ids || this.selIds();
    const wasRun = this.mode !== 'edit'; this.asmPend = null;
    if (wasRun) this.reset();
    if (this.asm) this.asmCancel(true);
    if (this.mateS) this.mateCancel(); if (this.tool !== 'select') this.setTool('select');
    ids = this.asmUnits(ids || this.selIds());
    if (ids.length < 2) { this.msg('装配：请先用 Ctrl/Shift+点击（场景或组件栏）选中两个或以上零件，再按 J 或点“🔧 装配”', true); return null; }
    if (this.gizmoOn) { this.gizmoOn = false; this.updGizmoBtns(); }
    this._asmBusy = true; try { const ms = new Set(this.selIds()); ids.forEach(id => ms.add(id)); this.msel = ms; if (!this.sel || !this.sel.p || !ms.has(this.sel.p)) this.sel = { p: ids[ids.length - 1] }; } finally { this._asmBusy = false; }
    this.asm = { ids, mode: ids.length > 2 ? 'chain' : 'pair', snap: this.snapshot(), act: 0, hov: null, pairs: [], info: [] };
    this.asmBuildPairs(); this.asmPreview(true); this.asmReveal();
    const pr = this.asm.pairs[0];
    this.msg((wasRun ? '已停止仿真并回到初始状态（编辑模式）。' : '') + `装配：${this.part(pr.a).name} ⟵ ${this.part(pr.b).name}　已自动选择 ${ATYPE[pr.type]}，确认无误按“确定装配”（Enter），或在右侧面板 / 3D 模型上改选连接点`);
    return this.asm;
  };
  P.asmBase = function () { this.M = JSON.parse(this.asm.snap); };
  P.asmBuildPairs = function () {
    const S = this.asm; this.asmBase(); S.pairs = [];
    for (let i = 1; i < S.ids.length; i++) S.pairs.push(this.asmDefault({ a: S.mode === 'star' ? S.ids[0] : S.ids[i - 1], b: S.ids[i] }));
    S.act = 0;
  };
  P.asmPreview = function (full = true) {
    const S = this.asm; if (!S) return;
    this._asmBusy = true;
    try {
      this.M = JSON.parse(S.snap);
      S.info = S.pairs.map(pr => { try { return this.asmApplyPair(pr); } catch (e) { console.warn(e); return { err: String(e && e.message || e), moved: [], jids: [], cids: [] }; } });
      // ground hook (same as on commit): the previewed result never sits below the floor
      const gl = this.groundFix({ quiet: true }); S.lift = gl.dy;
      if (gl.dy > 0.001) for (const inf of S.info) if (inf.moved && inf.moved.some(id => gl.lifted.includes(id))) inf.lifted = gl.dy;
      this.rebuild(true);
      const mv = new Set(S.info.flatMap(i => i.moved)); for (const id of mv) ghostify(this.objs[id]);
    } finally { this._asmBusy = false; }
    this.asmMarkers(); if (full) this.asmRender(); else this.asmInfoUpd();
  };
  P.asmSet = function (i, patch) { const S = this.asm; if (!S || !S.pairs[i]) return null; Object.assign(S.pairs[i], patch); S.act = i; this.asmPreview(true); return S.info[i]; };
  P.asmSwap = function (i = this.asm.act) { const S = this.asm, pr = S.pairs[i]; this.asmBase(); const a = pr.a; pr.a = pr.b; pr.b = a; this.asmDefault(pr); this.asmPreview(true); };
  P.asmSetMode = function (m) { const S = this.asm; if (!S || S.ids.length < 3) return; S.mode = m; this.asmBuildPairs(); this.asmPreview(true); };
  P.asmCommit = function () {
    const S = this.asm; if (!S) return null;
    this.asmPreview(false);
    const info = S.info; let changed = this.snapshot() !== S.snap;
    this.asm = null; this.asmClear();
    if (changed) { // validate: new joints sit on both parts and the edit gap closes (< 0.5 mm), else revert
      const v = this.validateNew(info.flatMap(i => i.jids), Math.max(0, ...info.map(i => i.stand || 0), ...S.pairs.map(pr => Math.abs(pr.off || 0))));
      if (!v.ok) { this.M = JSON.parse(S.snap); this.lastSnap = S.snap; this.rebuild(); this.msg('装配已撤回：' + v.msg + '。请检查连接点 / 连接类型后重试', true); return { joints: [], couplings: [], errs: [v.msg], changed: false, reverted: true }; }
      this.lastSnap = S.snap; this.commit(); }
    this.rebuild();
    const last = S.pairs[S.pairs.length - 1], jids = info.flatMap(i => i.jids), cids = info.flatMap(i => i.cids), errs = info.filter(i => i.err).map(i => i.err);
    this.select({ p: last.b });
    if (changed) { const ids = [...new Set(S.pairs.flatMap(pr => [pr.a, pr.b]))], j0 = jids.length ? this.M.joints.find(j => j.id === jids[jids.length - 1]) : null;
      this.flash = { ids, jids, t0: performance.now(), marker: j0 ? V3(j0.la).applyQuaternion(Q4(this.part(j0.a).q)).add(V3(this.part(j0.a).p)) : V3(this.part(last.b).p), label: '✓ 已装配' }; }
    this.msg(changed ? `✓ 装配完成：新建 ${jids.length} 个连接、${cids.length} 个传动（可 Ctrl+Z 一步撤销）` + (errs.length ? '；问题：' + errs.join('；') : '') : (errs.length ? '未装配：' + errs.join('；') : '没有需要新建的连接'), !!errs.length);
    return { joints: jids, couplings: cids, errs, changed };
  };
  P.asmCancel = function (silent) {
    const S = this.asm; if (!S) return;
    this.asm = null; this.asmClear(); this.M = JSON.parse(S.snap); this.lastSnap = this.snapshot(); this.rebuild();
    if (!silent) this.msg('已取消装配（模型未改变）');
  };
  P.asmToggle = function () { if (this.asm) this.asmCancel(); else this.asmOpen(); };
  /** after a Ctrl/Shift multi-select: open the panel when 2+ parts are selected */
  P.asmAuto = function () {
    const u = this.asmUnits(this.selIds());
    if (u.length >= 2) { if (this.mode === 'edit') this.asmOpen(u); else this.asmPending(u); }
    else { if (this.asm) this.asmCancel(true); if (this.asmPend) this.asmPendClear(); }
  };
  /** run / pause mode: assembling needs the edit (initial) state -> show the panel with a clear "返回编辑并装配" button */
  P.asmPending = function (ids) {
    if (this.asm) this.asmCancel(true);
    this.asmPend = ids.slice(); const el = $('asmPanel'); if (!el) return;
    const nm = id => esc((this.part(id) || {}).name || '?'), t = this.sim ? this.sim.time : 0;
    el.innerHTML = `<div class="asmHead">🔧 装配 <small>${ids.length} 个零件</small><span id="asmX" title="关闭">✕</span></div>`
      + `<div class="asmPendMsg">已选中：<b>${ids.map(nm).join('、')}</b><br>仿真${this.mode === 'run' ? '正在运行' : '已暂停'}（t = ${t.toFixed(2)} s）。装配需要在编辑模式（初始状态）下进行。</div>`
      + `<button class="bigAsm" id="asmGoEdit">↩ 返回编辑并装配</button><div class="hint2">点击后停止仿真、重置到初始状态，并打开装配面板（选连接点和连接类型 → 确定装配）。</div>`;
    el.style.display = 'block'; el.classList.add('pend'); $('side').classList.add('asm'); $('btnAsm') && $('btnAsm').classList.add('on');
    el.querySelector('#asmX').onclick = () => this.asmPendClear();
    el.querySelector('#asmGoEdit').onclick = () => { const p = this.asmPend || ids; this.asmOpen(p); };
    this.asmReveal();
    this.msg('已选中 ' + ids.length + ' 个零件：仿真中不能直接装配，点右侧“返回编辑并装配”', true);
  };
  P.asmPendClear = function () { this.asmPend = null; const el = $('asmPanel'); if (el) el.classList.remove('pend'); if (!this.asm) this.asmClear(); };
  /** keep the panel at the top of the right column and scrolled into view */
  P.asmReveal = function () { const sd = $('side'), el = $('asmPanel'); if (!sd || !el) return; if (sd.firstElementChild !== el) sd.prepend(el); sd.scrollTop = 0; };
  /** called from select(): keep the open / pending panel in sync with the selection */
  P.asmSyncSel = function () {
    if (this.asm) return this.asmSelChanged();
    if (this.asmPend && !this._asmBusy) { const u = this.asmUnits(this.selIds()); if (u.length < 2) this.asmPendClear(); else if (u.length !== this.asmPend.length || u.some(x => !this.asmPend.includes(x))) this.asmPending(u); }
  };
  /** selection changed by something else: close (and revert) the panel */
  P.asmSelChanged = function () {
    const S = this.asm; if (!S || this._asmBusy) return;
    const u = this.asmUnits(this.selIds()); if (u.length !== S.ids.length || u.some(x => !S.ids.includes(x))) this.asmCancel(true);
  };
  /** edit / delete an existing joint between the selected parts from inside the panel */
  P.asmEditExisting = function (fn) {
    const S = this.asm; if (!S) return; this._asmBusy = true;
    try { this.M = JSON.parse(S.snap); this.lastSnap = S.snap; fn(); S.snap = this.snapshot(); } finally { this._asmBusy = false; }
    for (const pr of S.pairs) if (pr.type === 'keep' && !this.asmLinked(pr, JSON.parse(S.snap))) { this.asmBase(); this.asmDefault(pr); }
    this.asmPreview(true);
  };
  P.asmDelCoupling = function (cid) { this.beginEdit(); this.M.couplings = this.M.couplings.filter(c => c.id !== cid); this.commit(); this.rebuild(); };

  // ---------------- picking on the model ----------------
  /** nearest entry to a click on part A/B: discrete points win within ~12 mm, then lines (rod/shaft body), then faces */
  P.asmNearest = function (h) {
    const S = this.asm, pr = S.pairs[S.act], side = h.pid === pr.a ? 'a' : h.pid === pr.b ? 'b' : null; if (!side) return null;
    const Pp = this.part(h.pid); let best = null;
    for (const e of asmEntries(Pp)) {
      const g0 = eWorld(Pp, e, {}); let prm = {}, w, d;
      if (e.kind === 'line') { const hl = e.len / 2, s = mm1(clamp(h.point.clone().sub(g0.p).dot(g0.dir), -hl, hl)); prm = { s }; w = g0.p.clone().addScaledVector(g0.dir, s); d = w.distanceTo(h.point) + 0.006; }
      else if (e.kind === 'plane') { if (h.normal && h.normal.dot(g0.n) < 0.8) continue; const Vv = g0.n.clone().cross(g0.u).normalize(), dv = h.point.clone().sub(g0.p); prm = { u: mm1(clamp(dv.dot(g0.u), -e.hu, e.hu)), v: mm1(clamp(dv.dot(Vv), -e.hv, e.hv)) }; w = eWorld(Pp, e, prm).p; d = w.distanceTo(h.point) + 0.012; }
      else { w = g0.p; d = w.distanceTo(h.point) + (e.role === 'ref' ? 0.002 : 0); }
      if (!best || d < best.d) best = { d, e, prm, w, side };
    }
    return best;
  };
  P.asmPickAt = function (h) {
    const S = this.asm, pr = S.pairs[S.act], r = this.asmNearest(h); if (!r) return null;
    if (['gear', 'belt', 'chain', 'keep'].includes(pr.type)) { this.msg(ATYPE[pr.type] + '不需要选择连接点；如需按点连接，请先把连接类型改为 固定 / 铰链 等', true); return null; }
    pr['k' + r.side] = r.e.key; pr['p' + r.side] = r.prm; this.asmPreview(true);
    this.msg(`已选择 ${r.side === 'a' ? 'A' : 'B'}（${this.part(h.pid).name}）的连接点：${r.e.name}`);
    return r;
  };

  // ---------------- 3D markers ----------------
  P.asmClear = function () {
    if (this.asmFx) { this.scene.remove(this.asmFx); this.asmFx.traverse(o => { if (o.geometry) o.geometry.dispose(); }); this.asmFx = null; }
    const el = $('asmPanel'); if (el) { el.style.display = 'none'; el.innerHTML = ''; el.classList.remove('pend'); } this.asmPend = null;
    $('side') && $('side').classList.remove('asm');
    for (const id of ['asmLblA', 'asmLblB']) { const l = $(id); if (l) l.style.display = 'none'; }
    const b = $('btnAsm'); if (b) b.classList.remove('on');
    this.hl.visible = false;
  };
  P.asmMarkers = function () {
    if (this.asmFx) { this.scene.remove(this.asmFx); this.asmFx.traverse(o => { if (o.geometry) o.geometry.dispose(); }); }
    const S = this.asm; if (!S) { this.asmFx = null; return; }
    const g = this.asmFx = new THREE.Group(); this.scene.add(g);
    const pr = S.pairs[S.act]; if (!pr) return;
    const mat = (c, op = 0.95) => new THREE.MeshBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: op });
    const lmat = c => new THREE.LineBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.9 });
    const add = o => { o.renderOrder = 12; o.traverse(x => { x.renderOrder = 12; x.raycast = () => { }; x.userData.helper = true; }); g.add(o); return o; };
    const sph = (c, r, p, op) => { const m = add(new THREE.Mesh(new THREE.SphereGeometry(r, 14, 10), mat(c, op))); m.position.copy(p); return m; };
    const ptsMode = !['gear', 'belt', 'chain', 'keep'].includes(pr.type);
    for (const side of ['a', 'b']) {
      const Pp = this.part(pr[side]); if (!Pp) continue; const col = side === 'a' ? 0xf97316 : 0x2563eb, key = pr['k' + side];
      if (!ptsMode) { sph(col, 0.008, V3(Pp.p)); continue; }
      for (const e of asmEntries(Pp)) {
        const w = eWorld(Pp, e, {}), on = e.key === key;
        if (e.kind === 'line') { const a = w.p.clone().addScaledVector(w.dir, -e.len / 2), b = w.p.clone().addScaledVector(w.dir, e.len / 2); add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([a, b]), lmat(col))); }
        else if (e.kind === 'plane') { if (!on) { const m = sph(col, 0.0032, w.p, 0.6); void m; } else { const V = w.n.clone().cross(w.u).normalize(), c = [[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]].map(([x, y]) => w.p.clone().addScaledVector(w.u, x * e.hu).addScaledVector(V, y * e.hv).addScaledVector(w.n, 0.0005)); add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(c), lmat(col))); } }
        else sph(col, 0.0042, w.p, 0.75);
        if (on) {
          const c = eWorld(Pp, e, clampPrm(e, pr['p' + side])); sph(col, 0.009, c.p);
          const ring = add(new THREE.Mesh(new THREE.TorusGeometry(0.016, 0.0022, 8, 28), mat(col))); ring.position.copy(c.p); ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), c.n);
          const ar = new THREE.ArrowHelper(c.n, c.p, 0.05, col, 0.014, 0.009); ar.traverse(x => { if (x.material) { x.material.depthTest = false; x.material.transparent = true; } }); add(ar);
        }
      }
    }
    if (S.hov) { const Pp = this.part(pr[S.hov.side]), e = Pp && asmEntries(Pp).find(x => x.key === S.hov.key); if (e) { const prm = e.key === pr['k' + S.hov.side] ? clampPrm(e, pr['p' + S.hov.side]) : {}; sph(0xfacc15, 0.011, eWorld(Pp, e, prm).p); } }
    if (!ptsMode && pr.type !== 'keep') { const A = this.part(pr.a), B = this.part(pr.b); if (A && B) add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([V3(A.p), V3(B.p)]), lmat(0xfacc15))); }
  };
  /** DOM labels next to the chosen points (called every frame while the panel is open) */
  P.asmLabels = function () {
    const S = this.asm; if (!S) return; const pr = S.pairs[S.act]; if (!pr) return; const cr = $('center').getBoundingClientRect();
    for (const side of ['a', 'b']) {
      const id = side === 'a' ? 'asmLblA' : 'asmLblB'; let l = $(id);
      if (!l) { l = document.createElement('div'); l.id = id; l.className = 'asmLbl ' + side; $('center').appendChild(l); }
      const Pp = this.part(pr[side]); if (!Pp) { l.style.display = 'none'; continue; }
      const e = asmEntries(Pp).find(x => x.key === pr['k' + side]), pts = !['gear', 'belt', 'chain', 'keep'].includes(pr.type) && e;
      const w = pts ? eWorld(Pp, e, clampPrm(e, pr['p' + side])).p : V3(Pp.p), s = this.toScreen(w);
      l.textContent = (side === 'a' ? 'A ' : 'B ') + Pp.name + (pts ? ' · ' + e.name.replace(/（.*$/, '') : '');
      l.style.display = 'block'; l.style.left = (s.x - cr.left + (side === 'a' ? 16 : -16)) + 'px'; l.style.top = (s.y - cr.top + (side === 'a' ? -30 : 10)) + 'px';
      l.style.transform = side === 'a' ? '' : 'translateX(-100%)';
    }
  };

  // ---------------- panel UI ----------------
  P.asmInfoText = function (i) {
    const S = this.asm, pr = S.pairs[i], inf = S.info[i] || {}, nm = id => (this.part(id) || {}).name || '?'; const L = [];
    if (inf.err) L.push(`<span class="asmErr">⚠ ${esc(inf.err)}</span>`);
    if (inf.mover) L.push(`移动：<b>${esc(nm(inf.mover))}</b>${inf.moved.length > 1 ? `（连同 ${inf.moved.length - 1} 个已连接零件整体移动）` : ''}`);
    if (inf.note) L.push(esc(inf.note));
    if (inf.lifted) L.push('已抬升至地面以上（整个装配体 ↑' + (inf.lifted * 1000).toFixed(1) + ' mm，连接保持）');
    if (pr.type === 'gear' && inf.label) L.push(esc(inf.label) + '（自动计算：a = m(z₁+z₂)/2）');
    if ((pr.type === 'belt' || pr.type === 'chain') && inf.dist) L.push('中心距 ' + (inf.dist * 1000).toFixed(1) + ' mm');
    if (inf.stand) L.push('自动错开 ' + (inf.stand * 1000).toFixed(1) + ' mm（两零件厚度，避免重叠）');
    if (inf.radial != null && !inf.err) L.push('对齐后两点径向偏差 ' + (inf.radial * 1000).toFixed(2) + ' mm');
    return L.join('<br>');
  };
  P.asmInfoUpd = function () { const S = this.asm; if (!S) return; S.pairs.forEach((_, i) => { const el = document.querySelector(`#asmPanel [data-info="${i}"]`); if (el) el.innerHTML = this.asmInfoText(i); }); };
  P.asmRender = function () {
    const S = this.asm, el = $('asmPanel'); if (!el) return; if (!S) { el.style.display = 'none'; return; }
    const focus = document.activeElement && el.contains(document.activeElement) ? document.activeElement.id : null;
    const nm = id => esc((this.part(id) || {}).name || '?');
    $('side').classList.add('asm'); $('btnAsm') && $('btnAsm').classList.add('on'); el.classList.remove('pend');
    let h = `<div class="asmHead">🔧 装配 <small>${S.ids.length} 个零件</small><span id="asmX" title="取消 (Esc)">✕</span></div>`;
    if (S.ids.length > 2) h += `<div class="row"><label>方式</label><select id="asmMode"><option value="chain" ${S.mode === 'chain' ? 'selected' : ''}>依次连接 A→B→C…</option><option value="star" ${S.mode === 'star' ? 'selected' : ''}>全部连到第一个零件</option></select></div>`;
    S.pairs.forEach((pr, i) => {
      const act = i === S.act, types = this.asmTypes(pr), inf = S.info[i] || {};
      h += `<div class="apair ${act ? 'act' : ''}" data-i="${i}"><div class="aph"><span class="apn">${S.pairs.length > 1 ? (i + 1) + '. ' : ''}</span><span class="tagA">A</span>${nm(pr.a)} <span class="arr">⟵</span> <span class="tagB">B</span>${nm(pr.b)}<span class="apt">${ATYPE[pr.type]}${inf.err ? ' ⚠' : ''}</span></div>`;
      if (act) {
        const ptsMode = !['gear', 'belt', 'chain', 'keep'].includes(pr.type);
        h += `<div class="row"><label>连接类型</label><select id="asmType">${types.map(t => `<option value="${t}" ${t === pr.type ? 'selected' : ''}>${ATYPE[t]}</option>`).join('')}</select></div>`;
        for (const side of ['a', 'b']) {
          const Pp = this.part(pr[side]), ents = Pp ? asmEntries(Pp) : [], key = pr['k' + side], e = ents.find(x => x.key === key), prm = clampPrm(e || {}, pr['p' + side] || {}), S2 = side.toUpperCase();
          h += `<div class="asmPt"><div class="asmPtH"><span class="tag${S2}">${S2}</span> ${nm(pr[side])} 的连接点${side === 'b' ? '（移动到 A）' : ''}</div>`;
          h += `<select id="asmE${S2}" class="asmList" size="${Math.min(7, Math.max(3, ents.length))}" ${ptsMode ? '' : 'disabled'}>${ents.map(x => `<option value="${x.key}" ${x.key === key ? 'selected' : ''}>${esc(x.name)}</option>`).join('')}</select>`;
          if (ptsMode && e && e.kind === 'line') h += `<div class="row"><label>位置</label><input type="number" id="asmS${S2}" step="1" value="${(prm.s * 1000).toFixed(1)}"><span class="u">mm</span></div><div class="hint2">沿${e.role === 'axleLine' ? '轴' : e.role === 'holeLine' ? '杆' : e.role === 'screwLine' ? '螺纹' : '导轨'}方向距中心，范围 ±${(e.len * 500).toFixed(0)} mm</div>`;
          if (ptsMode && e && e.kind === 'plane') h += `<div class="row"><label>面内位置 U/V</label><input type="number" id="asmU${S2}" step="1" value="${(prm.u * 1000).toFixed(1)}"><input type="number" id="asmV${S2}" step="1" value="${(prm.v * 1000).toFixed(1)}"><span class="u">mm</span></div>`;
          h += `</div>`;
        }
        if (ptsMode) h += `<div class="row"><label>轴线</label><select id="asmAxis">${[['auto', '自动（按连接点）'], ['X', 'X'], ['Y', 'Y（竖直）'], ['Z', 'Z']].map(([k, n]) => `<option value="${k}" ${k === pr.axis ? 'selected' : ''}>${n}</option>`).join('')}</select><label class="inl"><input type="checkbox" id="asmFlip" ${pr.flip ? 'checked' : ''}>反向</label></div>`;
        if (ptsMode && inf.stand) h += `<div class="row"><label></label><label class="inl"><input type="checkbox" id="asmOther" ${pr.other ? 'checked' : ''}>放在另一侧</label></div>`;
        if (ptsMode) h += `<div class="row"><label>偏移</label><input type="number" id="asmOff" step="1" value="${(pr.off * 1000).toFixed(1)}"><span class="u">mm</span></div>`;
        if (pr.type === 'belt' || pr.type === 'chain') h += `<div class="row"><label>中心距</label><input type="number" id="asmDist" step="5" value="${((pr.dist != null ? pr.dist : inf.dist || 0) * 1000).toFixed(1)}"><span class="u">mm</span></div>`;
        if (pr.type !== 'keep') h += `<div class="row"><label>${ptsMode ? '旋转角度' : '方位角'}</label><input type="number" id="asmAng" step="15" value="${pr.ang || 0}"><span class="u">°</span></div>`;
        if (!ptsMode && pr.type !== 'keep') h += `<div class="hint2">${pr.type === 'gear' ? '齿轮啮合：自动计算中心距并让齿相互错开；方位角 = B 绕 A 轴线的位置。' : '皮带/链条：B 移到与 A 同一平面、轴线平行；可改中心距和方位角。'}未装轴的轮会自动加铰链。</div>`;
        h += `<div class="asmInfo" data-info="${i}">${this.asmInfoText(i)}</div><button class="small" id="asmSwap" title="交换 A/B：改为移动 A">⇄ 交换 A/B</button>`;
      } else h += `<div class="asmInfo" data-info="${i}">${this.asmInfoText(i)}</div>`;
      h += `</div>`;
    });
    // existing joints / couplings between the selected parts
    const M0 = JSON.parse(S.snap), sel = new Set(S.ids.flatMap(id => { const Pp = this.part(id); return Pp && Pp.grp ? M0.parts.filter(q => q.grp === Pp.grp).map(q => q.id) : [id]; }));
    const js = M0.joints.filter(j => j.b && sel.has(j.a) && sel.has(j.b) && !isForce(j)), cs = M0.couplings.filter(c => sel.has(c.pa) && sel.has(c.pb));
    const pn = id => esc((M0.parts.find(q => q.id === id) || {}).name || '?');
    if (js.length || cs.length) {
      h += `<div class="asmEx"><div class="asmExH">所选零件之间已有的连接（${js.length + cs.length}）</div>`;
      for (const j of js) h += `<div class="asmExR"><select class="asmJT" data-j="${j.id}">${Object.entries(EDIT_T).map(([k, n]) => `<option value="${k}" ${k === j.type ? 'selected' : ''}>${n}</option>`).join('')}</select><span>${pn(j.a)} ↔ ${pn(j.b)}</span><button class="small asmJF" data-j="${j.id}" title="反转轴">⇅</button><button class="small asmJD" data-j="${j.id}" title="删除连接">✕</button></div>`;
      for (const c of cs) h += `<div class="asmExR"><b>${CTN[c.type] || c.type}</b><span>${pn(c.pa)} → ${pn(c.pb)}</span><button class="small asmCD" data-c="${c.id}" title="删除传动">✕</button></div>`;
      h += `</div>`;
    }
    h += `<div class="asmBtns"><button class="small run" id="asmOk">✓ 确定装配 (Enter)</button><button class="small" id="asmNo">取消 (Esc)</button></div>`;
    h += `<div class="hint2">提示：悬停列表项会在模型上高亮该点（橙=A，蓝=B）；也可直接在 3D 中点击 A/B 零件选取连接点。B 半透明显示装配后的位置。</div>`;
    el.innerHTML = h; el.style.display = 'block';
    this.asmBind(el);
    if (focus && $(focus)) $(focus).focus();
  };
  P.asmBind = function (el) {
    const S = this.asm, pr = S.pairs[S.act], q = id => el.querySelector('#' + id), on = (id, ev, fn) => { const x = q(id); if (x) x[ev] = fn; };
    on('asmX', 'onclick', () => this.asmCancel()); on('asmNo', 'onclick', () => this.asmCancel()); on('asmOk', 'onclick', () => this.asmCommit());
    on('asmMode', 'onchange', e => this.asmSetMode(e.target.value));
    on('asmSwap', 'onclick', () => this.asmSwap());
    el.querySelectorAll('.apair').forEach(d => { if (!d.classList.contains('act')) d.onclick = () => { S.act = +d.dataset.i; S.hov = null; this.asmMarkers(); this.asmRender(); }; });
    if (pr) {
      on('asmType', 'onchange', e => { pr.type = e.target.value; this.asmPreview(true); });
      for (const side of ['a', 'b']) {
        const S2 = side.toUpperCase(), list = q('asmE' + S2);
        if (list) {
          list.onchange = () => { pr['k' + side] = list.value; const Pp = this.part(pr[side]), e = asmEntries(Pp).find(x => x.key === list.value), o = side === 'a' ? 'b' : 'a', Po = this.part(pr[o]), eo = asmEntries(Po).find(x => x.key === pr['k' + o]);
            const M1 = this.M; this.asmBase(); pr['p' + side] = eo ? projPrm(this.part(pr[side]), e, eWorld(this.part(pr[o]), eo, clampPrm(eo, pr['p' + o])).p, eo.stack || 0) : {}; this.M = M1; void Po; this.asmPreview(true); };
          list.onmouseover = ev => { const o = ev.target.closest('option'); if (o && (!S.hov || S.hov.key !== o.value || S.hov.side !== side)) { S.hov = { side, key: o.value }; this.asmMarkers(); } };
          list.onmouseleave = () => { S.hov = null; this.asmMarkers(); };
        }
        const live = (id, fn) => on(id, 'oninput', e => { const v = +e.target.value; if (e.target.value === '' || !isFinite(v)) return; fn(v); this.asmPreview(false); });
        live('asmS' + S2, v => { pr['p' + side] = { s: v / 1000 }; });
        live('asmU' + S2, v => { pr['p' + side] = Object.assign({}, pr['p' + side], { u: v / 1000 }); });
        live('asmV' + S2, v => { pr['p' + side] = Object.assign({}, pr['p' + side], { v: v / 1000 }); });
      }
      on('asmAxis', 'onchange', e => { pr.axis = e.target.value; this.asmPreview(true); });
      on('asmFlip', 'onchange', e => { pr.flip = e.target.checked; this.asmPreview(true); });
      on('asmOther', 'onchange', e => { pr.other = e.target.checked; this.asmPreview(true); });
      const live = (id, fn) => on(id, 'oninput', e => { const v = +e.target.value; if (e.target.value === '' || !isFinite(v)) return; fn(v); this.asmPreview(false); });
      live('asmOff', v => { pr.off = v / 1000; }); live('asmAng', v => { pr.ang = v; }); live('asmDist', v => { pr.dist = v / 1000; });
    }
    el.querySelectorAll('.asmJT').forEach(s => s.onchange = () => this.asmEditExisting(() => this.setJointType(s.dataset.j, s.value)));
    el.querySelectorAll('.asmJF').forEach(b => b.onclick = () => this.asmEditExisting(() => this.flipJoint(b.dataset.j)));
    el.querySelectorAll('.asmJD').forEach(b => b.onclick = () => this.asmEditExisting(() => this.deleteJoint(b.dataset.j)));
    el.querySelectorAll('.asmCD').forEach(b => b.onclick = () => this.asmEditExisting(() => this.asmDelCoupling(b.dataset.c)));
    el.onkeydown = e => { if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); this.asmCancel(); } else if (e.key === 'Enter') { e.preventDefault(); e.stopPropagation(); this.asmCommit(); } };
  };
}
