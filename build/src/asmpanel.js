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
const LONG = { rod: [1, 0, 0], shaft: [0, 0, 1], screw: [0, 0, 1], worm: [0, 0, 1], rack: [1, 0, 0] };
/** world long direction of a bar-like part (rod / shaft ...), null otherwise */
export function longDir(P) { const l = LONG[P.type]; return l ? V3(l).applyQuaternion(Q4(P.q)).normalize() : null; }
/** same entry, but its axis turned perpendicular to the bar (shafts: the connection axis is a cross axis, bodies stand side by side) */
function crossEnt(P, e) {
  const l = LONG[P.type]; if (!l || !e.n || Math.abs(V3(e.n).normalize().dot(V3(l))) < 0.9) return e;
  const n = l[2] ? [1, 0, 0] : [0, 0, 1], r = P.d.r || (P.d.t ? P.d.t / 2 : 0.005);
  return Object.assign({}, e, { n, out: false, stack: r, sec: null });
}
const DISC = ['wheel', 'gear', 'pulley', 'sprocket', 'flywheel'];
export const ATYPE = { fixed: '固定', revolute: '铰链(转动)', prismatic: '滑动', spherical: '球铰', gear: '齿轮啮合', belt: '皮带', chain: '链条', helical: '螺旋', keep: '保留现有连接（不新增）' };
const EDIT_T = { revolute: '铰链', prismatic: '滑动副', fixed: '固定连接', spherical: '球铰', helical: '螺旋副' };
const CTN = { gear: '啮合', belt: '皮带', chain: '链条', screw: '螺旋副', gearbox: '变速箱联动', clutch: '离合器', ujoint: '万向节联动' };
// smart defaults: unordered role pair -> [joint type, score]
const PRI = [
  ['hub', 'axleLine', 'fixed', 100], ['nut', 'screwLine', 'helical', 97], ['hub', 'shaftOut', 'revolute', 96], ['axleLine', 'bore', 'revolute', 96], ['axleMid', 'bore', 'revolute', 94],
  ['hub', 'bore', 'revolute', 92], ['slider', 'rail', 'prismatic', 92], ['hole', 'shaftOut', 'revolute', 92], ['hole', 'pin', 'revolute', 91], ['hole', 'hole', 'revolute', 90], ['axleEnd', 'bore', 'revolute', 90],
  ['motorBase', 'mount', 'fixed', 88], ['mountBase', 'mount', 'fixed', 88], ['hub', 'pin', 'revolute', 85], ['axleEnd', 'pin', 'revolute', 86], ['hub', 'axleEnd', 'fixed', 72], ['hub', 'axleMid', 'fixed', 71], ['axleEnd', 'shaftOut', 'revolute', 85], ['hub', 'hole', 'revolute', 80], ['axleEnd', 'hole', 'revolute', 80], ['hole', 'bore', 'revolute', 80],
  ['kingpin', 'mount', 'revolute', 75], ['hole', 'holeLine', 'revolute', 70], ['holeLine', 'holeLine', 'revolute', 55], ['axleEnd', 'holeLine', 'revolute', 66], ['hub', 'holeLine', 'revolute', 65], ['pin', 'holeLine', 'revolute', 65], ['ball', 'hole', 'fixed', 60],
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
  // flat faces of discs (wheel / gear / pulley ...): any point on the face, e.g. an eccentric crank pin
  if (DISC.includes(P.type) && P.d.t) {
    const R = Math.max(0.004, P.type === 'gear' ? pitchRadius(P) - 1.3 * P.d.m : pitchRadius(P) * 0.9), t = P.d.t;
    for (const [i, sg, nm] of [[0, 1, '正面'], [1, -1, '背面']]) L.push({ key: 'f' + i, role: 'mount', kind: 'plane', p: [0, 0, sg * t / 2], n: [0, 0, sg], u: [1, 0, 0], hu: R, hv: R, rad: R, out: true, stack: 0, sec: [1, 0, 0], slide: [1, 0, 0], disc: true, name: nm + '（面·任意点/偏心）' });
  }
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
  if (e.kind === 'plane') return discClamp(e, { u: clamp(prm.u || 0, -e.hu, e.hu), v: clamp(prm.v || 0, -e.hv, e.hv) });
  return {};
}
function discClamp(e, r) { if (e.rad) { const l = Math.hypot(r.u, r.v); if (l > e.rad) { r.u *= e.rad / l; r.v *= e.rad / l; } } return r; }
/** role-pair compatibility: { sc (0 = no rule), ty (suggested connection type) } */
export function pairScore(ra, rb) { let sc = 0, ty = null; for (const [r1, r2, t, s] of PRI) if (((ra === r1 && rb === r2) || (ra === r2 && rb === r1)) && s > sc) { sc = s; ty = t; } return { sc, ty }; }
export const shortName = e => e ? String(e.name).replace(/（.*$/, '') : '';
/** ends / middle of a line entry: names + parameter (hub flush with the end: margin = half thickness of the other part) */
export function linePosList(P, e, other) {
  const m = Math.min(e.len / 2, other && other.stack ? other.stack : 0), h = e.len / 2 - m;
  const nm = P.type === 'shaft' ? ['后端', '中点', '前端'] : P.type === 'rod' ? ['端孔A', '中点', '端孔B'] : ['-端', '中点', '+端'];
  return [[nm[0], mm1(-h)], [nm[1], 0], [nm[2], mm1(h)]];
}
/** parameter on entry e (line/plane) closest to world point w */
function projPrm(P, e, w, margin = 0) {
  const g = eWorld(P, e, {}), d = w.clone().sub(g.p);
  if (e.kind === 'line') { const s = d.dot(g.dir), perp = d.clone().addScaledVector(g.dir, -s).length(); if (perp > 0.3) return { s: 0 }; const h = Math.max(0, e.len / 2 - margin); return { s: mm1(clamp(s, -h, h)) }; }
  if (e.kind === 'plane') { if (Math.abs(d.dot(g.n)) > 0.3) return { u: 0, v: 0 }; const V = g.n.clone().cross(g.u).normalize(), mg = 0.002; return discClamp(e, { u: mm1(clamp(d.dot(g.u), -e.hu + mg, e.hu - mg)), v: mm1(clamp(d.dot(V), -e.hv + mg, e.hv - mg)) }); }
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
    Object.assign(pr, { axis: 'auto', flip: false, other: false, off: 0, ang: 0, rel: null, cross: false, dist: null, pa: {}, pb: {}, ka: null, kb: null, type: 'fixed', typeManual: false, sugg: [] });
    if (!A || !B) return pr;
    const EA = asmEntries(A), EB = asmEntries(B); let best = null; const sug = [];
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
      if (ea.role !== 'ref' && eb.role !== 'ref' && pairScore(ea.role, eb.role).sc) sug.push({ sc, type: ty, ka: ea.key, kb: eb.key, pa, pb });
    }
    // ranked suggestions (best per point pair, compatible pairs only)
    sug.sort((x, y) => y.sc - x.sc); const seen = new Set(); pr.sugg = [];
    for (const s of sug) { const k = s.ka + '|' + s.kb; if (seen.has(k)) continue; seen.add(k); pr.sugg.push(s); if (pr.sugg.length >= 5) break; }
    if (best) Object.assign(pr, { ka: best.ea.key, kb: best.eb.key, pa: best.pa, pb: best.pb, type: best.ty });
    const types = this.asmTypes(pr);
    if (types.includes('keep')) pr.type = 'keep';
    else if (types.includes('gear')) pr.type = 'gear';
    else if (types.includes('belt')) pr.type = 'belt';
    else if (types.includes('chain')) pr.type = 'chain';
    else if ((A.type === 'screw' && B.type === 'nut') || (A.type === 'nut' && B.type === 'screw')) pr.type = 'helical';
    for (const t of ['chain', 'belt', 'gear']) if (types.includes(t)) pr.sugg.unshift({ sc: 200, type: t });
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
      // rods / shafts: an entry whose axis runs along the bar (shaft body / end) is turned to a cross axis when crossing at an angle
      const wantX = pr.cross || (pr.rel != null && Math.abs(Math.sin(pr.rel * D2R)) > 1e-6);
      const teX = wantX ? crossEnt(T, te) : te, feX = wantX ? crossEnt(F, fe) : fe;
      const X = alignXf(T, teX, tprm, F, feX, fprm, { axisW: t === 'prismatic' ? null : forced, flip: pr.flip, other: pr.other, off: pr.off, ang: pr.ang });
      // explicit angle between the two bars (0 = parallel, 90 = perpendicular ...): rotate the mover about the joint axis through the joint point
      if (pr.rel != null) { const lt = longDir(T), lf = longDir(F);
        if (lt && lf) { const ax = X.axis.clone().normalize(), pa = lt.clone().addScaledVector(ax, -lt.dot(ax)), pb = lf.clone().applyQuaternion(X.R); pb.addScaledVector(ax, -pb.dot(ax));
          if (pa.length() > 1e-4 && pb.length() > 1e-4) { pa.normalize(); pb.normalize(); const cur = Math.atan2(pa.clone().cross(pb).dot(ax), pa.dot(pb)), want = (mv === 'a' ? -1 : 1) * pr.rel * D2R;
            X.R.premultiply(new THREE.Quaternion().setFromAxisAngle(ax, want - cur)); info.rel = pr.rel; } else info.relNA = true; } }
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
    this.asm = { ids, mode: ids.length > 2 ? 'chain' : 'pair', snap: this.snapshot(), act: 0, hov: null, pairs: [], info: [], focus: 'a' };
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
  /** apply ranked suggestion k (optionally with a quick position on a line entry) */
  P.asmApplySugg = function (k, side, pos) {
    const S = this.asm, pr = S && S.pairs[S.act], x = pr && pr.sugg && pr.sugg[k]; if (!x) return null;
    Object.assign(pr, { type: x.type, typeManual: false, flip: false, other: false, off: 0, ang: 0, rel: null, cross: false });
    if (x.ka) Object.assign(pr, { ka: x.ka, kb: x.kb, pa: Object.assign({}, x.pa), pb: Object.assign({}, x.pb) });
    if (side) pr['p' + side] = { s: pos };
    S.hov = null; S.hovHit = null; this.asmGhostClear(); this.asmPreview(true); return S.info[S.act];
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
  /** nearest entry to a ray hit on part A/B: discrete points win within ~12 mm, then lines (rod/shaft body), then faces */
  P.asmNearest = function (h) {
    const S = this.asm, pr = S.pairs[S.act], side = h.pid === pr.a ? 'a' : h.pid === pr.b ? 'b' : null; if (!side) return null;
    const Pp = this.part(h.pid); let best = null;
    for (const e of asmEntries(Pp)) {
      const g0 = eWorld(Pp, e, {}); let prm = {}, w, d;
      if (e.kind === 'line') { const hl = e.len / 2, s = mm1(clamp(h.point.clone().sub(g0.p).dot(g0.dir), -hl, hl)); prm = { s }; w = g0.p.clone().addScaledVector(g0.dir, s); d = w.distanceTo(h.point) + 0.006; }
      else if (e.kind === 'plane') { if (h.normal && h.normal.dot(g0.n) < 0.8) continue; const Vv = g0.n.clone().cross(g0.u).normalize(), dv = h.point.clone().sub(g0.p); prm = discClamp(e, { u: mm1(clamp(dv.dot(g0.u), -e.hu, e.hu)), v: mm1(clamp(dv.dot(Vv), -e.hv, e.hv)) }); w = eWorld(Pp, e, prm).p; d = w.distanceTo(h.point) + 0.012; }
      else { w = g0.p; d = w.distanceTo(h.point) + (e.role === 'ref' ? 0.002 : 0); }
      if (!best || d < best.d) best = { d, e, prm, w, side, pid: h.pid };
    }
    return best;
  };
  const ptsOf = pr => pr && !['gear', 'belt', 'chain', 'keep'].includes(pr.type);
  /** what is under the cursor while the panel is open (screen space first: big markers are easy to hit even when tiny on screen)
   *  1. discrete points (focus side 18 px, other side 11 px)  2. shaft / bar body lines (10 px)  3. face surfaces (ray hit)  4. nearest entry of the hit part */
  P.asmHit = function (ev) {
    const S = this.asm, pr = S && S.pairs[S.act]; if (!ptsOf(pr)) return null;
    const cx = ev.clientX, cy = ev.clientY, cam = this.cam, foc = S.focus || 'a'; let best = null;
    const onScr = w => { const v = w.clone().project(cam); return v.z < 1 && v.z > -1; };
    for (const side of ['a', 'b']) {
      const Pp = this.part(pr[side]); if (!Pp || Pp.hidden) continue; const f = side === foc;
      for (const e of asmEntries(Pp)) {
        if (e.kind !== 'pt') continue; const w = eWorld(Pp, e, {}).p; if (!onScr(w)) continue;
        const s = this.toScreen(w), d = Math.hypot(s.x - cx, s.y - cy), R = e.role === 'ref' ? 8 : f ? 18 : 11;
        const key = d + (f ? 0 : 7) + (e.role === 'ref' ? 6 : 0);
        if (d <= R && (!best || key < best.key)) best = { key, side, e, prm: {}, w, pid: Pp.id };
      }
    }
    if (best) return best;
    // then, per side (focus part first): shaft / bar body lines (10 px), the face under the cursor, the nearest entry to the ray hit
    const other = foc === 'a' ? 'b' : 'a';
    for (const side of [foc, other]) { const r = this.asmLineHit(ev, side, pr) || this.asmSurfHit(ev, side, pr); if (r) return r; }
    return null;
  };
  P.asmLineHit = function (ev, side, pr) {
    const Pp = this.part(pr[side]); if (!Pp || Pp.hidden) return null; const cam = this.cam; this.ray.setFromCamera(this.ndc(ev), cam); const ray = this.ray.ray; let best = null;
    for (const e of asmEntries(Pp)) {
      if (e.kind !== 'line') continue; const g = eWorld(Pp, e, {}), a0 = g.p.clone().addScaledVector(g.dir, -e.len / 2), a1 = g.p.clone().addScaledVector(g.dir, e.len / 2);
      const on = new THREE.Vector3(); ray.distanceSqToSegment(a0, a1, null, on); const pj = on.clone().project(cam); if (pj.z >= 1 || pj.z <= -1) continue;
      const sc = this.toScreen(on), d = Math.hypot(sc.x - ev.clientX, sc.y - ev.clientY); if (d > 10) continue;
      const s = mm1(clamp(on.clone().sub(g.p).dot(g.dir), -e.len / 2, e.len / 2));
      if (!best || d < best.key) best = { key: d, side, e, prm: { s }, w: g.p.clone().addScaledVector(g.dir, s), pid: Pp.id };
    }
    return best;
  };
  P.asmSurfHit = function (ev, side, pr) {
    const h = this.asmRayPart(ev, pr[side]); if (!h) return null; const Pp = this.part(h.pid);
    for (const e of asmEntries(Pp)) { // a face under the cursor: the exact clicked point on it
      if (e.kind !== 'plane') continue; const g = eWorld(Pp, e, {}); if (!h.normal || h.normal.dot(g.n) < 0.8 || Math.abs(h.point.clone().sub(g.p).dot(g.n)) > 0.003) continue;
      const V = g.n.clone().cross(g.u).normalize(), dv = h.point.clone().sub(g.p); if (e.rad && Math.hypot(dv.dot(g.u), dv.dot(V)) > e.rad + 0.002) continue;
      const prm = discClamp(e, { u: mm1(clamp(dv.dot(g.u), -e.hu, e.hu)), v: mm1(clamp(dv.dot(V), -e.hv, e.hv)) });
      return { key: 0, side, e, prm, w: eWorld(Pp, e, prm).p, pid: Pp.id };
    }
    return this.asmNearest(h);
  };
  /** first ray hit on one part only: { pid, point, normal } */
  P.asmRayPart = function (ev, pid) {
    const o = this.objs[pid], Pp = this.part(pid); if (!o || !Pp || Pp.hidden) return null; this.scene.updateMatrixWorld(); this.ray.setFromCamera(this.ndc(ev), this.cam);
    const h = this.ray.intersectObject(o, true).find(x => !x.object.userData.helper); if (!h) return null;
    let normal = null; if (h.face) { normal = h.face.normal.clone().applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(h.object.matrixWorld)).normalize(); if (normal.dot(this.ray.ray.direction) > 0) normal.negate(); }
    return { pid, point: h.point, normal };
  };
  /** is the press on the currently chosen line / face of side sd (screen space, generous) */
  P.asmOnChosen = function (ev, sd) {
    const S = this.asm, pr = S.pairs[S.act], Pp = this.part(pr[sd]), e = Pp && asmEntries(Pp).find(x => x.key === pr['k' + sd]); if (!e || e.kind === 'pt') return null;
    const g = eWorld(Pp, e, {});
    if (e.kind === 'line') { const a = this.toScreen(g.p.clone().addScaledVector(g.dir, -e.len / 2)), b = this.toScreen(g.p.clone().addScaledVector(g.dir, e.len / 2)), c = new THREE.Vector2(ev.clientX, ev.clientY), ab = b.clone().sub(a), t = clamp(c.clone().sub(a).dot(ab) / Math.max(1e-9, ab.lengthSq()), 0, 1);
      return a.clone().addScaledVector(ab, t).distanceTo(c) <= 12 ? e : null; }
    this.ray.setFromCamera(this.ndc(ev), this.cam); const x = new THREE.Vector3(); if (!this.ray.ray.intersectPlane(new THREE.Plane().setFromNormalAndCoplanarPoint(g.n, g.p), x)) return null;
    const V = g.n.clone().cross(g.u).normalize(), dv = x.sub(g.p), u = dv.dot(g.u), v = dv.dot(V); return (e.rad ? Math.hypot(u, v) <= e.rad : Math.abs(u) <= e.hu && Math.abs(v) <= e.hv) ? e : null;
  };
  const hitKey = r => r ? r.side + '|' + r.e.key + '|' + JSON.stringify(r.prm || {}) : '';
  /** the pair as it would be after picking hit r (auto connection type, shaft end / centre with a hub -> flush on the shaft body) */
  P.asmPickPlan = function (pr, r) {
    const o = r.side === 'a' ? 'b' : 'a', Po = this.part(pr[o]), eo = Po && asmEntries(Po).find(x => x.key === pr['k' + o]), Pp = this.part(pr[r.side]);
    let e = r.e, prm = r.prm || {}, note = '';
    if (eo && ['axleEnd', 'axleMid'].includes(e.role) && ['hub', 'hole', 'bore', 'nut'].includes(eo.role)) {
      const ln = asmEntries(Pp).find(x => x.role === 'axleLine');
      if (ln) { const h = Math.max(0, ln.len / 2 - (eo.stack || 0)), sg = e.role === 'axleMid' ? 0 : Math.sign(V3(e.p).sub(V3(ln.p)).dot(V3(ln.dir))) || 1; prm = { s: mm1(sg * h) }; note = sg ? '（齐平装在轴端）' : '（装在轴中点）'; e = ln; }
    }
    const np = Object.assign({}, pr, { ['k' + r.side]: e.key, ['p' + r.side]: prm });
    const ea = r.side === 'a' ? e : eo, eb = r.side === 'b' ? e : eo;
    if (!pr.typeManual && ea && eb) { const ps = pairScore(ea.role, eb.role); if (ps.sc && ps.ty) np.type = ps.ty; }
    return { np, e, prm, note, compat: ea && eb ? pairScore(ea.role, eb.role).sc : 0 };
  };
  /** click on the model: choose the connection point under the cursor */
  P.asmPickHit = function (r) {
    const S = this.asm, pr = S.pairs[S.act]; if (!r) return null;
    if (!ptsOf(pr)) { this.msg(ATYPE[pr.type] + '不需要选择连接点；如需按点连接，请先把连接类型改为 固定 / 铰链 等', true); return null; }
    const pl = this.asmPickPlan(pr, r); Object.assign(pr, pl.np);
    const nb = asmEntries(this.part(pr.b)).filter(x => x.role !== 'ref').length;
    if (r.side === 'a' && nb > 1) S.focus = 'b';
    S.hov = null; S.hovHit = null; this.asmGhostClear(); this.asmPreview(true);
    this.msg(`已选择 ${r.side === 'a' ? 'A' : 'B'}（${this.part(r.pid || pr[r.side]).name}）的连接点：${shortName(pl.e)}${pl.note} → ${ATYPE[pr.type]}` + (r.side === 'a' && nb > 1 ? '。下一步：点击 B 的连接点（蓝色）' : ''));
    return r;
  };
  P.asmPickAt = function (h) { const r = h && h.e ? h : this.asmNearest(h); return r ? this.asmPickHit(r) : null; };
  P.asmClick = function (ev) { const r = this.asmHit(ev); if (!r) return false; this.asmPickHit(r); return true; };
  P.asmSetFocus = function (f) { const S = this.asm; if (!S) return; S.focus = f; S.hov = null; S.hovHit = null; this.asmGhostClear(); this.asmMarkers(); this.asmRender(); };
  P.asmHover = function (ev) {
    const S = this.asm; if (!S || this.asmScrub) return;
    const r = this.asmHit(ev), k = hitKey(r), cv = $('cv'); if (cv) cv.style.cursor = r ? 'pointer' : '';
    if (k === S.hovK) return; S.hovK = k; S.hovHit = r; S.hov = r ? { side: r.side, key: r.e.key } : null; this.asmMarkers();
    clearTimeout(this._asmGT); this.asmGhostClear(); if (r) this._asmGT = setTimeout(() => { if (this.asm === S && S.hovK === k) this.asmGhost(r); }, 140);
  };
  // ---- hover preview: a green ghost of the moved parts at the position the hovered point would give (model untouched) ----
  P.asmGhostClear = function () { if (this.asmGhostFx) { this.scene.remove(this.asmGhostFx); this.asmGhostFx = null; } if (this.asm) this.asm.hovInfo = null; };
  P.asmTry = function (np) {
    const S = this.asm, M1 = this.M, i0 = S.act; let res = null; this._asmBusy = true;
    try { this.M = JSON.parse(S.snap); const infos = S.pairs.map((p, i) => { try { return this.asmApplyPair(i === i0 ? np : p); } catch (e) { return { err: String(e.message || e), moved: [] }; } });
      this.groundFix({ quiet: true }); const ids = [...new Set(infos.flatMap(x => x.moved || []))]; res = { info: infos[i0], poses: ids.map(id => { const Q = this.part(id); return { id, p: Q.p.slice(), q: Q.q.slice() }; }) };
    } finally { this.M = M1; this._asmBusy = false; }
    return res;
  };
  P.asmGhost = function (r) {
    const S = this.asm, pr = S.pairs[S.act], pl = this.asmPickPlan(pr, r);
    if (pl.np['k' + r.side] === pr['k' + r.side] && JSON.stringify(clampPrm(pl.e, pl.np['p' + r.side])) === JSON.stringify(clampPrm(pl.e, pr['p' + r.side] || {})) && pl.np.type === pr.type) return;
    const t = this.asmTry(pl.np); if (!t) return; S.hovInfo = { type: pl.np.type, err: t.info && t.info.err, note: pl.note, compat: pl.compat };
    const g = this.asmGhostFx = new THREE.Group(); g.userData.helper = true; this.scene.add(g);
    const gm = this._asmGM || (this._asmGM = new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.38, depthWrite: false }));
    for (const ps of t.poses) { const src = this.objs[ps.id]; if (!src) continue; const c = src.clone(true); c.traverse(m => { m.raycast = () => { }; m.userData = { helper: true }; if (m.isMesh) { m.material = gm; m.castShadow = false; m.renderOrder = 6; } }); c.position.set(...ps.p); c.quaternion.set(...ps.q); g.add(c); }
  };
  // ---- drag along a shaft / bar / face (stationary part) to slide the connection point ----
  P.asmPtrDown = function (ev) {
    const S = this.asm, pr = S && S.pairs[S.act]; if (!pr || ev.button !== 0) return false;
    const inf = S.info[S.act] || {};
    if (ptsOf(pr)) for (const sd of ['a', 'b']) { if (pr[sd] === inf.mover) continue; const e = this.asmOnChosen(ev, sd);
      if (e) { this.asmScrub = { side: sd, key: e.key, x: ev.clientX, y: ev.clientY, moved: false }; this.orbit.enabled = false; return true; } }
    const h = this.pickAt(ev, { parts: true }); return !!(h.pid && (h.pid === pr.a || h.pid === pr.b)); // never start a part drag on A / B (that used to close the panel)
  };
  P.asmScrubMove = function (ev) {
    const D = this.asmScrub, S = this.asm; if (!D || !S) return; const pr = S.pairs[S.act], Pp = this.part(pr[D.side]), e = Pp && asmEntries(Pp).find(x => x.key === D.key); if (!e) return;
    if (!D.moved && Math.hypot(ev.clientX - D.x, ev.clientY - D.y) < 4) return; D.moved = true;
    this.ray.setFromCamera(this.ndc(ev), this.cam); const ray = this.ray.ray, g = eWorld(Pp, e, {}); let prm = null;
    if (e.kind === 'line') { const a0 = g.p.clone().addScaledVector(g.dir, -e.len / 2), a1 = g.p.clone().addScaledVector(g.dir, e.len / 2), on = new THREE.Vector3(); ray.distanceSqToSegment(a0, a1, null, on); prm = { s: mm1(clamp(on.sub(g.p).dot(g.dir), -e.len / 2, e.len / 2)) }; }
    else { const x = new THREE.Vector3(); if (!ray.intersectPlane(new THREE.Plane().setFromNormalAndCoplanarPoint(g.n, g.p), x)) return; const V = g.n.clone().cross(g.u).normalize(), dv = x.sub(g.p); prm = discClamp(e, { u: mm1(clamp(dv.dot(g.u), -e.hu, e.hu)), v: mm1(clamp(dv.dot(V), -e.hv, e.hv)) }); }
    if (JSON.stringify(prm) === JSON.stringify(pr['p' + D.side])) return; pr['p' + D.side] = prm;
    if (!D.raf) D.raf = requestAnimationFrame(() => { D.raf = 0; if (this.asm) { this.asmPreview(false); this.asmSyncNums(); } });
  };
  P.asmScrubEnd = function () { const D = this.asmScrub; this.asmScrub = null; this.orbit.enabled = true; if (D && D.moved && this.asm) { this.asmPreview(true); D.ended = true; this._asmNoClick = performance.now(); } };
  /** keep the number / slider inputs in sync without re-rendering the panel (keeps a slider drag alive) */
  P.asmSyncNums = function () {
    const S = this.asm; if (!S) return; const pr = S.pairs[S.act];
    for (const side of ['a', 'b']) { const S2 = side.toUpperCase(), e = this.asmEntry(pr[side], pr['k' + side]); if (!e) continue; const prm = clampPrm(e, pr['p' + side] || {});
      const set = (id, v) => { const x = $(id); if (x && document.activeElement !== x) x.value = v; };
      if (e.kind === 'line') { set('asmS' + S2, (prm.s * 1000).toFixed(1)); set('asmSR' + S2, Math.round(prm.s * 1000)); set('asmSP' + S2, (50 + prm.s / e.len * 100).toFixed(0)); }
      if (e.kind === 'plane') { set('asmU' + S2, (prm.u * 1000).toFixed(1)); set('asmV' + S2, (prm.v * 1000).toFixed(1)); set('asmUR' + S2, Math.round(prm.u * 1000)); set('asmVR' + S2, Math.round(prm.v * 1000)); } }
  };

  // ---------------- 3D markers ----------------
  P.asmClear = function () {
    if (this.asmFx) { this.scene.remove(this.asmFx); this.asmFx.traverse(o => { if (o.geometry) o.geometry.dispose(); }); this.asmFx = null; }
    this.asmGhostClear(); clearTimeout(this._asmGT); this.asmScrub = null; if (this.orbit) this.orbit.enabled = true;
    const el = $('asmPanel'); if (el) { el.style.display = 'none'; el.innerHTML = ''; el.classList.remove('pend'); } this.asmPend = null;
    $('side') && $('side').classList.remove('asm');
    for (const id of ['asmLblA', 'asmLblB', 'asmLblH']) { const l = $(id); if (l) l.style.display = 'none'; }
    document.querySelectorAll('.asmTag').forEach(t => { t.style.display = 'none'; });
    const cv = $('cv'); if (cv) cv.style.cursor = '';
    for (const id in this.jobjs || {}) this.jobjs[id].visible = true;
    const b = $('btnAsm'); if (b) b.classList.remove('on');
    this.hl.visible = false;
  };
  /** sprite textures: filled dot (point), ring (chosen), square (face), all white -> tinted per side */
  P.asmTex = function (k) {
    const T = this._asmTex || (this._asmTex = {}); if (T[k]) return T[k];
    const c = document.createElement('canvas'); c.width = c.height = 64; const x = c.getContext('2d');
    x.lineWidth = 7; x.strokeStyle = '#fff'; x.fillStyle = '#fff';
    if (k === 'dot') { x.beginPath(); x.arc(32, 32, 24, 0, 7); x.fill(); x.fillStyle = 'rgba(0,0,0,.35)'; x.beginPath(); x.arc(32, 32, 9, 0, 7); x.fill(); x.globalCompositeOperation = 'destination-over'; x.fillStyle = 'rgba(15,23,42,.85)'; x.beginPath(); x.arc(32, 32, 30, 0, 7); x.fill(); }
    else if (k === 'ring') { x.fillStyle = 'rgba(15,23,42,.85)'; x.beginPath(); x.arc(32, 32, 30, 0, 7); x.fill(); x.fillStyle = '#fff'; x.beginPath(); x.arc(32, 32, 26, 0, 7); x.fill(); x.globalCompositeOperation = 'destination-out'; x.beginPath(); x.arc(32, 32, 15, 0, 7); x.fill(); x.globalCompositeOperation = 'source-over'; x.beginPath(); x.arc(32, 32, 8, 0, 7); x.fill(); }
    else { x.fillStyle = 'rgba(15,23,42,.8)'; x.fillRect(6, 6, 52, 52); x.fillStyle = '#fff'; x.fillRect(11, 11, 42, 42); x.globalCompositeOperation = 'destination-out'; x.fillRect(22, 22, 20, 20); }
    const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return (T[k] = t);
  };
  P.asmScaleSprites = function () {
    if (!this.asmFx) return; const H = Math.max(1, $('cv').clientHeight), k = 2 / (this.cam.projectionMatrix.elements[5] * H);
    this.asmFx.traverse(o => { if (o.isSprite && o.userData.px) o.scale.set(o.userData.px * k, o.userData.px * k, 1); });
  };
  P.asmMarkers = function () {
    if (this.asmFx) { this.scene.remove(this.asmFx); this.asmFx.traverse(o => { if (o.geometry && !o.isSprite) o.geometry.dispose(); if (o.material) o.material.dispose(); }); }
    const S = this.asm; if (!S) { this.asmFx = null; return; }
    const g = this.asmFx = new THREE.Group(); this.scene.add(g);
    const pr = S.pairs[S.act]; if (!pr) return;
    // only the two parts being connected show anything: joint markers of other parts are hidden while the panel is open
    for (const id in this.jobjs) { const j = this.M.joints.find(x => x.id === id); this.jobjs[id].visible = !!j && [j.a, j.b].some(x => x === pr.a || x === pr.b); }
    const mat = (c, op = 0.95) => new THREE.MeshBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: op });
    const lmat = c => new THREE.LineBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.95 });
    const add = (o, ro = 12) => { o.renderOrder = ro; o.traverse(x => { x.renderOrder = ro; x.raycast = () => { }; x.userData.helper = true; }); g.add(o); return o; };
    const spr = (tex, col, px, p, op = 1, ro = 13) => { const s = add(new THREE.Sprite(new THREE.SpriteMaterial({ map: this.asmTex(tex), color: col, depthTest: false, transparent: true, opacity: op, sizeAttenuation: false })), ro); s.position.copy(p); s.userData.px = px; return s; };
    const tube = (a, b, col, r, op) => { const d = b.clone().sub(a), L = d.length(); if (L < 1e-6) return; const m = add(new THREE.Mesh(new THREE.CylinderGeometry(r, r, L, 8, 1, true), mat(col, op)), 11); m.position.copy(a).add(b).multiplyScalar(0.5); m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize()); };
    const outline = (Pp, e, col, prm) => { const w = eWorld(Pp, e, {}), V = w.n.clone().cross(w.u).normalize(); let c;
      if (e.rad) c = Array.from({ length: 41 }, (_, i) => { const a = i / 40 * Math.PI * 2; return w.p.clone().addScaledVector(w.u, Math.cos(a) * e.rad).addScaledVector(V, Math.sin(a) * e.rad).addScaledVector(w.n, 0.0006); });
      else c = [[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]].map(([x, y]) => w.p.clone().addScaledVector(w.u, x * e.hu).addScaledVector(V, y * e.hv).addScaledVector(w.n, 0.0006));
      add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(c), lmat(col)));
      if (prm) { const q = eWorld(Pp, e, prm).p, r = Math.min(e.hu, e.hv, 0.03) * 0.6; for (const [d1, d2] of [[w.u, V]]) { add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([q.clone().addScaledVector(d1, -r), q.clone().addScaledVector(d1, r)]), lmat(col))); add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([q.clone().addScaledVector(d2, -r), q.clone().addScaledVector(d2, r)]), lmat(col))); } } };
    const ptsMode = ptsOf(pr), foc = S.focus || 'a', hv = S.hovHit;
    for (const side of ['a', 'b']) {
      const Pp = this.part(pr[side]); if (!Pp) continue; const col = side === 'a' ? 0xf97316 : 0x2563eb, key = pr['k' + side], f = side === foc;
      if (!ptsMode) { spr('ring', col, 22, V3(Pp.p)); continue; }
      for (const e of asmEntries(Pp)) {
        const w = eWorld(Pp, e, {}), on = e.key === key;
        if (e.kind === 'line') { const a = w.p.clone().addScaledVector(w.dir, -e.len / 2), b = w.p.clone().addScaledVector(w.dir, e.len / 2); tube(a, b, col, f ? 0.0022 : 0.0014, on ? 0.9 : f ? 0.55 : 0.35); }
        else if (e.kind === 'plane') { if (on) outline(Pp, e, col, null); else if (f) spr('sq', col, 12, w.p.clone().addScaledVector(w.n, 0.0006), 0.8, 12); }
        else if (e.role === 'ref') { if (f) spr('dot', 0x94a3b8, 9, w.p, 0.85, 12); }
        else spr('dot', col, f ? 17 : 11, w.p, f ? 1 : 0.7);
        if (on) {
          const c = eWorld(Pp, e, clampPrm(e, pr['p' + side])); spr('ring', col, 26, c.p, 1, 14);
          const ar = new THREE.ArrowHelper(c.n, c.p, 0.05, col, 0.014, 0.009); ar.traverse(x => { if (x.material) { x.material.depthTest = false; x.material.transparent = true; } }); add(ar);
        }
      }
    }
    if (ptsMode && hv) { const Pp = this.part(pr[hv.side]); if (Pp) { if (hv.e.kind === 'plane') outline(Pp, hv.e, 0xfacc15, hv.prm); if (hv.e.kind === 'line') { const w = eWorld(Pp, hv.e, {}); tube(w.p.clone().addScaledVector(w.dir, -hv.e.len / 2), w.p.clone().addScaledVector(w.dir, hv.e.len / 2), 0xfacc15, 0.0026, 0.7); } spr('ring', 0xfacc15, 30, eWorld(Pp, hv.e, clampPrm(hv.e, hv.prm || {})).p, 1, 15); } }
    else if (S.hov) { const Pp = this.part(pr[S.hov.side]), e = Pp && asmEntries(Pp).find(x => x.key === S.hov.key); if (e) { const prm = e.key === pr['k' + S.hov.side] ? clampPrm(e, pr['p' + S.hov.side]) : {}; if (e.kind === 'plane') outline(Pp, e, 0xfacc15, prm); spr('ring', 0xfacc15, 30, eWorld(Pp, e, prm).p, 1, 15); } }
    if (!ptsMode && pr.type !== 'keep') { const A = this.part(pr.a), B = this.part(pr.b); if (A && B) add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([V3(A.p), V3(B.p)]), lmat(0xfacc15))); }
    this.asmScaleSprites();
  };
  /** DOM labels (every frame while the panel is open): chosen A/B points, the hovered point (name + resulting connection), short tags on the focus part */
  P.asmLabels = function () {
    const S = this.asm; if (!S) return; const pr = S.pairs[S.act]; if (!pr) return; const ct = $('center'), cr = ct.getBoundingClientRect();
    this.asmScaleSprites();
    const lbl = (id, cls) => { let l = $(id); if (!l) { l = document.createElement('div'); l.id = id; l.className = 'asmLbl ' + cls; ct.appendChild(l); } return l; };
    const ptsMode = ptsOf(pr);
    for (const side of ['a', 'b']) {
      const l = lbl(side === 'a' ? 'asmLblA' : 'asmLblB', side);
      const Pp = this.part(pr[side]); if (!Pp) { l.style.display = 'none'; continue; }
      const e = asmEntries(Pp).find(x => x.key === pr['k' + side]), pts = ptsMode && e;
      const w = pts ? eWorld(Pp, e, clampPrm(e, pr['p' + side])).p : V3(Pp.p), s = this.toScreen(w);
      l.textContent = (side === 'a' ? 'A ' : 'B ') + Pp.name + (pts ? ' · ' + shortName(e) : '');
      l.style.display = 'block'; l.style.left = (s.x - cr.left + (side === 'a' ? 18 : -18)) + 'px'; l.style.top = (s.y - cr.top + (side === 'a' ? -32 : 12)) + 'px';
      l.style.transform = side === 'a' ? '' : 'translateX(-100%)';
    }
    // hover label: what is under the cursor and what clicking it would give
    const lh = lbl('asmLblH', 'h'), hv = S.hovHit;
    if (ptsMode && hv) {
      const Pp = this.part(pr[hv.side]), w = eWorld(Pp, hv.e, clampPrm(hv.e, hv.prm || {})).p, s = this.toScreen(w), pl = this.asmPickPlan(pr, hv), hi = S.hovInfo;
      let pos = ''; const prm = clampPrm(hv.e, hv.prm || {});
      if (hv.e.kind === 'line') pos = ' · 距中心 ' + (prm.s >= 0 ? '+' : '') + (prm.s * 1000).toFixed(0) + ' mm';
      if (hv.e.kind === 'plane') pos = ' · (' + (prm.u * 1000).toFixed(0) + ', ' + (prm.v * 1000).toFixed(0) + ') mm';
      const res = pl.compat ? `→ ${ATYPE[pl.np.type]}${pl.note}` : `→ ${ATYPE[pl.np.type]}（非常规组合）`;
      lh.innerHTML = `<b>${hv.side.toUpperCase()} ${esc(Pp.name)} · ${esc(shortName(hv.e))}</b>${esc(pos)}<br><span>${esc(res)}${hi && hi.err ? ' ⚠ ' + esc(hi.err) : ''}</span><br><small>点击选择${hi && !hi.err ? '（绿色 = 预览结果）' : ''}</small>`;
      lh.style.display = 'block'; lh.style.left = (s.x - cr.left + 22) + 'px'; lh.style.top = (s.y - cr.top + 18) + 'px';
    } else lh.style.display = 'none';
    // short name tags next to every point of the part being picked (focus side)
    const tags = [...ct.querySelectorAll('.asmTag')]; let n = 0;
    if (ptsMode) {
      const foc = S.focus || 'a', Pp = this.part(pr[foc]);
      const ents = Pp ? asmEntries(Pp).filter(e => e.kind === 'pt' && e.role !== 'ref' && e.key !== pr['k' + foc] && !(hv && hv.side === foc && hv.e.key === e.key)) : [];
      if (ents.length <= 12) for (const e of ents) {
        const v = eWorld(Pp, e, {}).p, pj = v.clone().project(this.cam); if (pj.z >= 1) continue; const s = this.toScreen(v);
        let t = tags[n]; if (!t) { t = document.createElement('div'); t.className = 'asmTag'; ct.appendChild(t); tags.push(t); }
        t.className = 'asmTag ' + foc; t.textContent = shortName(e); t.style.display = 'block'; t.style.left = (s.x - cr.left + 11) + 'px'; t.style.top = (s.y - cr.top - 9) + 'px'; n++;
      }
    }
    for (let i = n; i < tags.length; i++) tags[i].style.display = 'none';
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
    if (inf.rel != null) L.push('两杆夹角 ' + inf.rel + '°'); if (inf.relNA) L.push('两杆夹角：两杆方向与连接轴线平行，夹角不适用（可勾选“交叉连接”或改轴线）');
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
        const ptsMode = ptsOf(pr), foc = S.focus || 'a';
        h += `<div class="row"><label>连接类型</label><select id="asmType">${types.map(t => `<option value="${t}" ${t === pr.type ? 'selected' : ''}>${ATYPE[t]}</option>`).join('')}</select></div>`;
        // ranked suggestions: one click applies point pair + type
        const sg = (pr.sugg || []).filter(x => x.type === 'gear' || x.type === 'belt' || x.type === 'chain' ? types.includes(x.type) : true);
        if (sg.length) {
          h += `<div class="asmSug"><div class="asmSugH">⭐ 推荐连接 <small>按兼容性和距离排序 · 点击直接应用</small></div>`;
          sg.forEach((x, k) => {
            const on = x.ka ? (pr.ka === x.ka && pr.kb === x.kb && pr.type === x.type) : pr.type === x.type;
            let txt;
            if (!x.ka) txt = `${ATYPE[x.type]}（自动计算位置）`;
            else { const ea = this.asmEntry(pr.a, x.ka), eb = this.asmEntry(pr.b, x.kb); txt = `<span class="tagB">B</span>${esc(shortName(eb))} → <span class="tagA">A</span>${esc(shortName(ea))}`; }
            h += `<div class="sugR ${on ? 'on' : ''}" data-s="${k}"><span class="sn">${k + 1}</span><span class="st">${txt}</span><span class="sty">${ATYPE[x.type]}</span>`;
            if (x.ka) for (const side of ['a', 'b']) { const e = this.asmEntry(pr[side], x['k' + side]), o = this.asmEntry(pr[side === 'a' ? 'b' : 'a'], x['k' + (side === 'a' ? 'b' : 'a')]);
              if (e && e.kind === 'line') h += `<span class="sqp">${linePosList(this.part(pr[side]), e, o).map(([n, s]) => `<button class="mini sugPos" data-s="${k}" data-side="${side}" data-pos="${s}" title="${esc(shortName(e))}：${n}">${n}</button>`).join('')}</span>`; }
            h += `</div>`;
          });
          h += `</div>`;
        }
        if (ptsMode) h += `<div class="asmStep"><span>在 3D 中点击连接点：</span><button class="asmFoc a ${foc === 'a' ? 'on' : ''}" data-f="a">① A 点</button><button class="asmFoc b ${foc === 'b' ? 'on' : ''}" data-f="b">② B 点</button><small>Tab 切换</small></div>`;
        for (const side of ['a', 'b']) {
          const Pp = this.part(pr[side]), ents = Pp ? asmEntries(Pp) : [], key = pr['k' + side], e = ents.find(x => x.key === key), prm = clampPrm(e || {}, pr['p' + side] || {}), S2 = side.toUpperCase();
          const o = side === 'a' ? 'b' : 'a', eo = this.asmEntry(pr[o], pr['k' + o]);
          h += `<div class="asmPt ${ptsMode && foc === side ? 'foc' : ''}"><div class="asmPtH"><span class="tag${S2}">${S2}</span> ${nm(pr[side])}${side === 'b' ? '（移动到 A）' : ''}：<b>${ptsMode && e ? esc(shortName(e)) : '—'}</b></div>`;
          h += `<select id="asmE${S2}" class="asmList" size="${Math.min(5, Math.max(3, ents.length))}" ${ptsMode ? '' : 'disabled'}>${ents.map(x => { const c = eo && pairScore(x.role, eo.role).sc; return `<option value="${x.key}" ${x.key === key ? 'selected' : ''} class="${c ? 'ok' : x.role === 'ref' ? 'ref' : ''}">${c ? '✓ ' : ''}${esc(x.name)}</option>`; }).join('')}</select>`;
          if (ptsMode && e && e.kind === 'line') {
            const hl = Math.round(e.len * 500);
            h += `<div class="row"><label>位置</label><input type="range" id="asmSR${S2}" min="${-hl}" max="${hl}" step="1" value="${Math.round(prm.s * 1000)}"></div><div class="row"><label></label><input type="number" id="asmS${S2}" class="n" step="1" value="${(prm.s * 1000).toFixed(1)}"><span class="u">mm</span><input type="number" id="asmSP${S2}" class="n p" step="1" min="0" max="100" value="${(50 + prm.s / e.len * 100).toFixed(0)}" title="从一端起的百分比：0% = 一端，50% = 中点，100% = 另一端"><span class="u">%</span></div>`;
            h += `<div class="row qp"><label>快捷位置</label>${linePosList(Pp, e, eo).map(([n, s]) => `<button class="mini asmQP" data-side="${side}" data-s="${s}">${n}</button>`).join('')}</div>`;
            h += `<div class="hint2">沿${e.role === 'axleLine' ? '轴' : e.role === 'holeLine' ? '杆' : e.role === 'screwLine' ? '螺纹' : '导轨'}方向距中心（±${hl} mm）：拖滑块、输入数值，或在 3D 中沿${e.role === 'axleLine' ? '轴身' : '线'}拖动/点击</div>`;
          }
          if (ptsMode && e && e.kind === 'plane') {
            const ru = Math.round(e.hu * 1000), rv = Math.round(e.hv * 1000);
            h += `<div class="row"><label>面内 U</label><input type="range" id="asmUR${S2}" min="${-ru}" max="${ru}" step="1" value="${Math.round(prm.u * 1000)}"><input type="number" id="asmU${S2}" class="n" step="1" value="${(prm.u * 1000).toFixed(1)}"><span class="u">mm</span></div>`;
            h += `<div class="row"><label>面内 V</label><input type="range" id="asmVR${S2}" min="${-rv}" max="${rv}" step="1" value="${Math.round(prm.v * 1000)}"><input type="number" id="asmV${S2}" class="n" step="1" value="${(prm.v * 1000).toFixed(1)}"><span class="u">mm</span></div>`;
            h += `<div class="row qp"><label></label><button class="mini asmQC" data-side="${side}">⊙ 面中心</button></div><div class="hint2">${e.rad ? '圆面内任意点（偏心安装，半径 ≤ ' + Math.round(e.rad * 1000) + ' mm）' : '面上任意点'}：在 3D 中直接点击该面，或拖动</div>`;
          }
          h += `</div>`;
        }
        if (ptsMode) h += `<div class="row"><label>轴线</label><select id="asmAxis">${[['auto', '自动（按连接点）'], ['X', 'X'], ['Y', 'Y（竖直）'], ['Z', 'Z']].map(([k, n]) => `<option value="${k}" ${k === pr.axis ? 'selected' : ''}>${n}</option>`).join('')}</select><label class="inl"><input type="checkbox" id="asmFlip" ${pr.flip ? 'checked' : ''}>反向 ⇅</label></div>`;
        if (ptsMode && inf.stand) h += `<div class="row"><label></label><label class="inl"><input type="checkbox" id="asmOther" ${pr.other ? 'checked' : ''}>放在另一侧</label></div>`;
        if (ptsMode) h += `<div class="row"><label>偏移</label><input type="number" id="asmOff" step="1" value="${(pr.off * 1000).toFixed(1)}"><span class="u">mm</span><button class="mini asmOffB" data-d="-5">−5</button><button class="mini asmOffB" data-d="5">+5</button></div>`;
        if (pr.type === 'belt' || pr.type === 'chain') h += `<div class="row"><label>中心距</label><input type="number" id="asmDist" step="5" value="${((pr.dist != null ? pr.dist : inf.dist || 0) * 1000).toFixed(1)}"><span class="u">mm</span></div>`;
        const bars = ptsMode && LONG[(this.part(pr.a) || {}).type] && LONG[(this.part(pr.b) || {}).type];
        if (bars) h += `<div class="row"><label>两杆夹角</label><input type="number" id="asmRel" step="15" placeholder="自动" value="${pr.rel == null ? '' : pr.rel}"><span class="u">°</span></div><div class="row qp"><label></label>${[['平行', 0], ['45°', 45], ['90°', 90], ['自动', '']].map(([n, v]) => `<button class="mini asmRelB" data-v="${v}">${n}</button>`).join('')}</div><div class="row"><label></label><label class="inl"><input type="checkbox" id="asmCross" ${pr.cross ? 'checked' : ''}>交叉连接（连接轴垂直于杆身，两杆并排交叉）</label></div>`;
        if (pr.type !== 'keep') h += `<div class="row"><label>${ptsMode ? '旋转角度' : '方位角'}</label><input type="number" id="asmAng" step="15" value="${pr.ang || 0}"><span class="u">°</span></div><div class="row qp"><label></label>${[-90, -15, 15, 90].map(d => `<button class="mini asmAngB" data-d="${d}">${d > 0 ? '+' : ''}${d}°</button>`).join('')}</div>`;
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
    h += `<div class="hint2">提示：只显示这两个零件的连接点（橙=A，蓝=B，● 点 / ■ 面 / 线 = 轴身·杆身）。鼠标悬停显示名称和结果（绿色预览），点击即选；轴身、面上可点任意位置并拖动。B 半透明 = 装配后的位置。</div>`;
    el.innerHTML = h; el.style.display = 'block';
    this.asmBind(el);
    if (focus && $(focus)) $(focus).focus();
  };
  P.asmBind = function (el) {
    const S = this.asm, pr = S.pairs[S.act], q = id => el.querySelector('#' + id), on = (id, ev, fn) => { const x = q(id); if (x) x[ev] = fn; };
    on('asmX', 'onclick', () => this.asmCancel()); on('asmNo', 'onclick', () => this.asmCancel()); on('asmOk', 'onclick', () => this.asmCommit());
    on('asmMode', 'onchange', e => this.asmSetMode(e.target.value));
    on('asmSwap', 'onclick', () => this.asmSwap());
    el.querySelectorAll('.apair').forEach(d => { if (!d.classList.contains('act')) d.onclick = () => { S.act = +d.dataset.i; S.hov = null; S.hovHit = null; S.focus = 'a'; this.asmGhostClear(); this.asmMarkers(); this.asmRender(); }; });
    if (pr) {
      on('asmType', 'onchange', e => { pr.type = e.target.value; pr.typeManual = true; this.asmPreview(true); });
      for (const side of ['a', 'b']) {
        const S2 = side.toUpperCase(), list = q('asmE' + S2);
        if (list) {
          list.onchange = () => { pr['k' + side] = list.value; const Pp = this.part(pr[side]), e = asmEntries(Pp).find(x => x.key === list.value), o = side === 'a' ? 'b' : 'a', Po = this.part(pr[o]), eo = asmEntries(Po).find(x => x.key === pr['k' + o]);
            const M1 = this.M; this.asmBase(); pr['p' + side] = eo ? projPrm(this.part(pr[side]), e, eWorld(this.part(pr[o]), eo, clampPrm(eo, pr['p' + o])).p, eo.stack || 0) : {}; this.M = M1; void Po; if (!pr.typeManual && e && eo) { const ps = pairScore(side === 'a' ? e.role : eo.role, side === 'a' ? eo.role : e.role); if (ps.ty) pr.type = ps.ty; } this.asmPreview(true); };
          list.onmouseover = ev => { const o = ev.target.closest('option'); if (o && (!S.hov || S.hov.key !== o.value || S.hov.side !== side)) { S.hov = { side, key: o.value }; this.asmMarkers(); } };
          list.onmouseleave = () => { S.hov = null; this.asmMarkers(); };
        }
        const live = (id, fn) => on(id, 'oninput', e => { const v = +e.target.value; if (e.target.value === '' || !isFinite(v)) return; fn(v); this.asmPreview(false); });
        live('asmS' + S2, v => { pr['p' + side] = { s: v / 1000 }; }); on('asmS' + S2, 'onchange', () => this.asmSyncNums());
        live('asmU' + S2, v => { pr['p' + side] = Object.assign({}, pr['p' + side], { u: v / 1000 }); });
        live('asmV' + S2, v => { pr['p' + side] = Object.assign({}, pr['p' + side], { v: v / 1000 }); });
      }
      for (const side of ['a', 'b']) { const S2 = side.toUpperCase(), slide = (id, fn) => on(id, 'oninput', e => { fn(+e.target.value / 1000); this.asmPreview(false); this.asmSyncNums(); });
        slide('asmSR' + S2, v => { pr['p' + side] = { s: v }; }); slide('asmUR' + S2, v => { pr['p' + side] = Object.assign({}, pr['p' + side], { u: v }); }); slide('asmVR' + S2, v => { pr['p' + side] = Object.assign({}, pr['p' + side], { v }); }); }
      el.querySelectorAll('.asmQP').forEach(b => b.onclick = () => { pr['p' + b.dataset.side] = { s: +b.dataset.s }; this.asmPreview(true); });
      el.querySelectorAll('.asmQC').forEach(b => b.onclick = () => { pr['p' + b.dataset.side] = { u: 0, v: 0 }; this.asmPreview(true); });
      el.querySelectorAll('.asmFoc').forEach(b => b.onclick = () => this.asmSetFocus(b.dataset.f));
      el.querySelectorAll('.sugR').forEach(d => d.onclick = ev => { if (ev.target.closest('.sugPos')) return; this.asmApplySugg(+d.dataset.s); });
      el.querySelectorAll('.sugPos').forEach(b => b.onclick = () => this.asmApplySugg(+b.dataset.s, b.dataset.side, +b.dataset.pos));
      on('asmRel', 'oninput', e => { const v = e.target.value; if (v !== '' && !isFinite(+v)) return; pr.rel = v === '' ? null : +v; this.asmPreview(false); });
      el.querySelectorAll('.asmRelB').forEach(b => b.onclick = () => { pr.rel = b.dataset.v === '' ? null : +b.dataset.v; this.asmPreview(true); });
      on('asmCross', 'onchange', e => { pr.cross = e.target.checked; this.asmPreview(true); });
      for (const side of ['a', 'b']) on('asmSP' + side.toUpperCase(), 'oninput', ev => { const v = +ev.target.value; if (ev.target.value === '' || !isFinite(v)) return; const e = this.asmEntry(pr[side], pr['k' + side]); if (!e || e.kind !== 'line') return; pr['p' + side] = { s: mm1((clamp(v, 0, 100) / 100 - 0.5) * e.len) }; this.asmPreview(false); this.asmSyncNums(); });
      el.querySelectorAll('.asmAngB').forEach(b => b.onclick = () => { pr.ang = ((pr.ang || 0) + +b.dataset.d); this.asmPreview(true); });
      el.querySelectorAll('.asmOffB').forEach(b => b.onclick = () => { pr.off = mm1((pr.off || 0) + +b.dataset.d / 1000); this.asmPreview(true); });
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
