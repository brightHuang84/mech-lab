// Resizing a part while keeping every connection matched.
// resizeModel(M, pid, newD) works on a clone of the model and returns { ok, M, gap, strategy, msg, notes }.
import * as THREE from 'three';
import { DEFS, defaultMass, pitchRadius } from './parts.js';
import { Sim, analyze, planCoupling, V3, Q4, ptW, axW, ptL, isForce } from './physics.js';
import { geoMinY } from './ground.js';
import { connectors } from './assemble.js';

export const GAP_TOL = 0.0005; // 0.5 mm
// axis index along which a long part is "a bar": interior anchors keep their distance from the nearer end
const LONG = { shaft: 2, screw: 2, worm: 2, rod: 0, follower: 1, rack: 0 };
const GROUNDISH = new Set(['base', 'wall']);
// parts that make the assembly they belong to the 'staying' side when a part attached to it is resized
const STRUCT = new Set(['motor', 'plate', 'base', 'wall', 'bearing', 'gearbox', 'mesh']);

/** local axis-aligned box of a part: union of collision shapes and named connection points */
export function localBox(type, d) {
  const D = DEFS[type], mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  const add = (c, e) => { for (let i = 0; i < 3; i++) { mn[i] = Math.min(mn[i], c[i] - e[i]); mx[i] = Math.max(mx[i], c[i] + e[i]); } };
  for (const s of D.shapes(d)) {
    const c = s.pos || [0, 0, 0];
    if (s.k === 'box') add(c, s.h); else if (s.k === 'cyl') add(c, [s.r, s.r, s.hh]); else if (s.k === 'ball') add(c, [s.r, s.r, s.r]);
    else if (s.k === 'cap') add(c, [s.r, s.hh + s.r, s.r]); else if (s.pts) for (let i = 0; i < s.pts.length; i += 3) add([s.pts[i], s.pts[i + 1], s.pts[i + 2]], [0, 0, 0]);
  }
  for (const p of D.pts(d)) add(p.p, [0, 0, 0]);
  return { mn, mx };
}

/** map a local anchor of a resized part from old to new dimensions. keep=true: centre-relative (only clamp) */
export function remapPoint(type, oldD, newD, l, keep, keepAxis = -1) {
  const tol = 5e-4;
  if (!keep && keepAxis < 0) { const op = DEFS[type].pts(oldD), np = DEFS[type].pts(newD); for (let i = 0; i < op.length; i++) if (Math.hypot(l[0] - op[i].p[0], l[1] - op[i].p[1], l[2] - op[i].p[2]) < 2e-4) return np[i].p.slice(); }
  const o = localBox(type, oldD), n = localBox(type, newD), out = [];
  for (let i = 0; i < 3; i++) {
    const x = l[i], a0 = o.mn[i], b0 = o.mx[i], a1 = n.mn[i], b1 = n.mx[i];
    if (Math.abs(a0 - a1) < 1e-12 && Math.abs(b0 - b1) < 1e-12) { out.push(x); continue; }
    let y;
    if (keep || i === keepAxis) y = x;
    else if (x >= b0 - tol) y = b1 + (x - b0);
    else if (x <= a0 + tol) y = a1 + (x - a0);
    else if (LONG[type] === i) y = (x - a0 <= b0 - x) ? a1 + (x - a0) : b1 - (b0 - x);
    else y = (a1 + b1) / 2 + (x - (a0 + b0) / 2);
    // clamp inside the new size (a hinge pin kept along its own axis may stick out by up to 1 mm)
    if (x >= a0 - tol && x <= b0 + tol) y = i === keepAxis ? Math.max(a1 - 1e-3, Math.min(b1 + 1e-3, y)) : Math.max(a1, Math.min(b1, y));
    out.push(y);
  }
  return out;
}

/** lowest world y of the part's real geometry (mesh vertices; collision shapes as fallback) */
export function partMinY(P) { return geoMinY(P, null, null, shapeMinY); }
/** lowest world y of the collision shapes */
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

const anchorW = (M, j, side) => { const id = side === 'a' ? j.a : j.b; if (id == null) return V3(j.lb); const P = M.parts.find(p => p.id === id); return ptW(P, side === 'a' ? j.la : j.lb); };
const slides = j => j.type === 'prismatic' || j.type === 'helical';
/** joint separation (m); sliders only perpendicular to their axis. Includes fixed and world joints. */
export function jointGaps(M) {
  const out = [];
  for (const j of M.joints) {
    if (isForce(j)) continue; const A = M.parts.find(p => p.id === j.a); if (!A || (j.b != null && !M.parts.find(p => p.id === j.b))) continue;
    const d = anchorW(M, j, 'b').sub(anchorW(M, j, 'a'));
    if (slides(j)) { const n = axW(A, j.axa); d.addScaledVector(n, -d.dot(n)); }
    out.push({ id: j.id, gap: d.length() });
  }
  return out;
}
export const maxJointGap = M => jointGaps(M).reduce((m, g) => Math.max(m, g.gap), 0);

function translate(P, d) { P.p = V3(P.p).add(d).toArray(); }

/** breadth-first settle: anchored parts stay, every other part is translated so its joints coincide again */
function settle(M, pid, rootOrder, pre) {
  const byId = new Map(M.parts.map(p => [p.id, p])), edges = new Map();
  const add = (k, j) => { if (!edges.has(k)) edges.set(k, []); edges.get(k).push(j); };
  for (const j of M.joints) { if (isForce(j) || !byId.has(j.a) || (j.b != null && !byId.has(j.b))) continue; add(j.a, j); add(j.b == null ? 'W' : j.b, j); }
  // fixed joints first so rigid groups are carried together
  for (const l of edges.values()) l.sort((x, y) => (y.type === 'fixed') - (x.type === 'fixed'));
  const settled = new Set(['W', ...pre]);
  const visit = (root, force) => {
    if (settled.has(root) && !force) return; settled.add(root); const q = Array.isArray(root) ? root : [root];
    while (q.length) {
      const s = q.shift();
      for (const j of edges.get(s) || []) {
        const sSide = s === 'W' ? 'b' : (j.a === s ? 'a' : 'b'), other = sSide === 'a' ? (j.b == null ? 'W' : j.b) : j.a;
        if (settled.has(other)) continue;
        const d = anchorW(M, j, sSide).sub(anchorW(M, j, sSide === 'a' ? 'b' : 'a'));
        if (slides(j)) { const n = axW(byId.get(j.a), j.axa); d.addScaledVector(n, -d.dot(n)); }
        if (d.lengthSq() > 1e-16) translate(byId.get(other), d);
        settled.add(other); q.push(other);
      }
    }
  };
  visit(['W', ...pre], true); for (const r of rootOrder) visit(r); visit(pid);
}

/** parts reachable from start through joints, not entering static groups, blocked ids or the world.
 *  null if the start is static, touches a blocked part, or is driven by a motor (a motor's shaft cannot be re-anchored) */
function cluster(M, an, start, blocked) {
  const C = new Set(), q = [start]; if (an.groupOf[start].static || blocked.has(start)) return null; C.add(start);
  const byId = new Map(M.parts.map(p => [p.id, p]));
  while (q.length) {
    const s = q.shift();
    for (const j of M.joints) {
      if (isForce(j) || (j.a !== s && j.b !== s)) continue; const o = j.a === s ? j.b : j.a; if (o == null || C.has(o)) continue;
      if (blocked.has(o)) return null;
      if (an.groupOf[o] && an.groupOf[o].static) { if (byId.get(o).type === 'motor') return null; continue; }
      C.add(o); q.push(o);
    }
  }
  return C;
}
/** rigidly move a cluster; joints from the cluster to the outside (static parts / world) are re-anchored there ("the hole moves") */
function moveCluster(M, C, d, rot, piv) {
  const byId = new Map(M.parts.map(p => [p.id, p]));
  for (const id of C) { const P = byId.get(id); if (rot) { P.p = V3(P.p).sub(piv).applyQuaternion(rot).add(piv).toArray(); P.q = rot.clone().multiply(Q4(P.q)).toArray(); } if (d) translate(P, d); }
  for (const j of M.joints) {
    const ia = C.has(j.a), ib = j.b != null && C.has(j.b); if (ia === ib) continue;
    const w = ia ? ptW(byId.get(j.a), j.la) : ptW(byId.get(j.b), j.lb);
    if (ia) { if (j.b == null) j.lb = w.toArray(); else j.lb = ptL(byId.get(j.b), w).toArray(); }
    else j.la = ptL(byId.get(j.a), w).toArray();
    if (!ia) { /* the outside part is side A: keep its axis */ }
  }
}

const zAx = P => axW(P, [0, 0, 1]);
/** required translation of part H so that its mesh with G is exact again */
function meshDelta(G, H, oldRG) {
  const cG = V3(G.p), cH = V3(H.p);
  const plane = (v, n) => v.clone().addScaledVector(n, -v.dot(n));
  if (G.type === 'rack' || H.type === 'rack') {
    const R = G.type === 'rack' ? G : H, W = R === G ? H : G, Yr = axW(R, [0, 1, 0]);
    const s = cH.clone().sub(cG); void s;
    const pitch = V3(R.p).addScaledVector(Yr, R.d.h / 2), dist = V3(W.p).sub(pitch).dot(Yr), err = pitchRadius(W) - dist;
    return H === W ? Yr.multiplyScalar(err) : Yr.multiplyScalar(-err);
  }
  if ((G.type === 'worm') !== (H.type === 'worm')) {
    const Wm = G.type === 'worm' ? G : H, Wh = Wm === G ? H : G;
    let v = plane(plane(V3(Wh.p).sub(V3(Wm.p)), zAx(Wm)), zAx(Wh)); const L = v.length(); if (L < 1e-9) return null;
    const target = Wm.d.r + pitchRadius(Wh); v.normalize();
    return H === Wh ? v.multiplyScalar(target - L) : v.multiplyScalar(-(target - L));
  }
  if (G.type === 'gear' && H.type === 'gear') {
    const u = plane(cH.clone().sub(cG), zAx(G)), L = u.length(); if (L < 1e-9) return null;
    return u.normalize().multiplyScalar(pitchRadius(G) + pitchRadius(H) - L);
  }
  if (G.type === 'bevel' || H.type === 'bevel') return null;
  const u = plane(cH.clone().sub(cG), zAx(G)), L = u.length(); if (L < 1e-9 || oldRG == null) return null;
  return u.normalize().multiplyScalar(pitchRadius(G) - oldRG);
}
/** tooth phase for a spur pair (G stays, H turns about its own axis) */
function phaseRot(G, H) {
  const nG = zAx(G), nH = zAx(H); if (nG.dot(nH) < 0.9999) return null;
  const u = V3(H.p).sub(V3(G.p)); u.addScaledVector(nG, -u.dot(nG)); if (u.length() < 1e-9) return null; u.normalize();
  const ul = u.applyQuaternion(Q4(G.q).invert()), phi = Math.atan2(ul.y, ul.x), th = phi + Math.PI - (Math.PI - G.d.z * phi) / H.d.z;
  const want = Q4(G.q).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), th));
  return want.multiply(Q4(H.q).invert());
}

/** move the cluster of `start` by d; meshes from the moved cluster to other clusters cascade with the same offset */
function propagate(M, pid, start, blocked, d, done, phaseFrom) {
  const byId = id => M.parts.find(p => p.id === id), gearCs = M.couplings.filter(c => c.type === 'gear');
  const an = analyze(M), myGroup = an.groupOf[pid] ? an.groupOf[pid].parts.map(p => p.id) : [];
  const q = [{ start, blocked: new Set([...blocked, ...myGroup].filter(id => id !== start)), phase: phaseFrom }];
  while (q.length) {
    const it = q.shift(), C = cluster(M, an, it.start, it.blocked); if (!C) return false;
    if (d.length() > 1e-12) moveCluster(M, C, d);
    const others = gearCs.filter(o => !done.has(o.id) && (C.has(o.pa) !== C.has(o.pb)));
    if (it.phase && !gearCs.some(o => o.id !== it.phase.cid && (C.has(o.pa) || C.has(o.pb)))) { const G = byId(it.phase.g), H = byId(it.start); if (G.type === 'gear' && H.type === 'gear') { const R = phaseRot(G, H); if (R) moveCluster(M, C, null, R, V3(H.p)); } }
    for (const o of others) { done.add(o.id); const K = C.has(o.pa) ? o.pb : o.pa; q.push({ start: K, blocked: new Set([...C, ...myGroup]) }); }
  }
  return true;
}
function fixMeshes(M, pid, oldR, notes) {
  const byId = id => M.parts.find(p => p.id === id), done = new Set();
  for (const c of M.couplings.filter(c => c.type === 'gear' && (c.pa === pid || c.pb === pid))) {
    if (done.has(c.id)) continue; done.add(c.id);
    const G = byId(pid), H = byId(c.pa === pid ? c.pb : c.pa); if (!G || !H) continue;
    if (G.type === 'bevel' || H.type === 'bevel') { notes.push('锥齿轮啮合位置未自动调整'); continue; }
    if (G.d.m != null && H.d.m != null && Math.abs(G.d.m - H.d.m) > 1e-9) notes.push(`${G.name} 与 ${H.name} 模数不同，不能正确啮合`);
    const d = meshDelta(G, H, oldR); if (!d) continue;
    let ok = false;
    // prefer moving the partner side; otherwise move the resized part's side the other way
    for (const [start, blocked, sign, phase] of [[H.id, [G.id], 1, { g: G.id, cid: c.id }], [G.id, [H.id], -1, null]]) {
      const M2 = JSON.parse(JSON.stringify({ parts: M.parts, joints: M.joints, couplings: M.couplings })), dn = new Set(done);
      const an = analyze(M2), bl = an.groupOf[blocked[0]].parts.map(p => p.id);
      if (propagateFrom(M2, pid, start, bl, d.clone().multiplyScalar(sign), dn, phase)) { M.parts = M2.parts; M.joints = M2.joints; for (const x of dn) done.add(x); ok = true; break; }
    }
    if (!ok && d.length() > 1e-6) notes.push(`${H.name} 无法移动到新的中心距（两侧都被固定）`);
  }
}
function propagateFrom(M, pid, start, blocked, d, done, phase) {
  // when the resized part's own side moves, do not block its rigid group
  const an = analyze(M);
  if (an.groupOf[start].id === an.groupOf[pid].id) return propagateOwn(M, pid, start, blocked, d, done);
  return propagate(M, pid, start, blocked, d, done, phase);
}
function propagateOwn(M, pid, start, blocked, d, done) {
  const an = analyze(M), C = cluster(M, an, start, new Set(blocked)); if (!C) return false;
  if (d.length() > 1e-12) moveCluster(M, C, d);
  const gearCs = M.couplings.filter(c => c.type === 'gear');
  for (const o of gearCs) { if (done.has(o.id) || C.has(o.pa) === C.has(o.pb)) continue; done.add(o.id); if (!propagate(M, '__none__', C.has(o.pa) ? o.pb : o.pa, [...C], d, done)) return false; }
  return true;
}

function relax(M) {
  if (maxJointGap(M) < 1e-5) return;
  const s = new Sim(M, { edit: true });
  try { if (s.maxGap() > 1e-5) { let best = s.maxGap(), stall = 0; for (let i = 0; i < 200 && best > 1e-5 && stall < 8; i++) { s.relax(10); const g = s.maxGap(); if (g < best * 0.98) { best = g; stall = 0; } else stall++; } for (const P of M.parts) { if (s.A.groupOf[P.id].static) continue; /* static bodies never move (avoid float32 round-off) */ const ps = s.partPose(P.id); P.p = ps.p.toArray(); P.q = ps.q.toArray(); } } } finally { s.free(); }
}

function updateCouplings(M, pid, oldD) {
  const P = M.parts.find(p => p.id === pid);
  for (const c of M.couplings) {
    if (c.pa !== pid && c.pb !== pid) continue; if (!['gear', 'belt', 'chain', 'screw'].includes(c.type)) continue;
    const A = M.parts.find(p => p.id === c.pa), B = M.parts.find(p => p.id === c.pb); if (!A || !B) continue;
    const r = planCoupling(M, c.type, A, B, { ratio: c.ratio });
    if (typeof r === 'object' && r.ja === c.ja && r.jb === c.jb) { c.k = r.k; c.sa = r.sa; c.sb = r.sb; continue; }
    // fallback: scale the ratio with the radius change
    const old = Object.assign({}, P, { d: oldD }), f = pitchRadius(P) / pitchRadius(old);
    if (isFinite(f) && f > 0) c.k = c.pa === pid ? c.k * f : c.k / f;
  }
}

// ---------------- parametric anchor references ----------------
// Every joint stores, per side, which connector of the part it sits on: j.ref = { a: {n, k, off|s}, b: … } (n = connector name,
// k = 'pt' (point + axial offset, e.g. a hub stacked on an axle end) | 'line' (s = position along an axle / rod body) | 'plane').
// Written when the joint is created (addJoint: assembly panel, mate, snap, joint tools) and refreshed after a resize; resize uses it to
// recompute the anchor from the new dimensions (stale refs — the anchor no longer on that connector — are ignored and re-derived).
const nrm = a => { const v = V3(a); const l = v.length(); return l > 1e-12 ? v.multiplyScalar(1 / l) : v; };
function refCands(P) {
  const out = [];
  try { for (const c of connectors(P)) out.push(c); } catch (e) { /* no connectors */ }
  const D = DEFS[P.type]; if (D && D.pts) for (const pt of D.pts(P.d)) if (!out.some(c => c.n === pt.n)) out.push({ n: pt.n, p: pt.p, a: pt.a || [0, 0, 1] });
  return out;
}
/** which connector of part P (current dimensions) the local point l lies on, or null */
export function anchorRef(P, l) {
  const L = V3(l); let exact = null, line = null, off = null, plane = null;
  for (const c of refCands(P)) {
    const p = V3(c.p), a = nrm(c.a), d = L.clone().sub(p);
    if (c.plane) { if (Math.abs(d.dot(a)) < 1e-3 && !plane) plane = { n: c.n, k: 'plane' }; continue; }
    if (c.line) { const s = d.dot(a), perp = d.clone().addScaledVector(a, -s).length(); if (perp < 1e-3 && Math.abs(s) <= (c.len || 0) / 2 + 1e-3 && !line) line = { n: c.n, k: 'line', s }; continue; }
    const t = d.dot(a), perp = d.clone().addScaledVector(a, -t).length(); if (perp > 1e-3) continue;
    if (Math.abs(t) < 1e-4) { if (!exact) exact = { n: c.n, k: 'pt', off: 0 }; }
    else if (Math.abs(t) < 0.06 && (!off || Math.abs(t) < Math.abs(off.off))) off = { n: c.n, k: 'pt', off: t };
  }
  return exact || line || off || plane;
}
/** local point of a reference on part P with dimensions d (null: connector gone / plane) */
export function resolveRef(P, ref, d = P.d) {
  if (!ref || ref.k === 'plane') return null; const c = refCands(Object.assign({}, P, { d })).find(x => x.n === ref.n); if (!c || !!c.line !== (ref.k === 'line')) return null;
  return V3(c.p).addScaledVector(nrm(c.a), ref.k === 'line' ? ref.s : ref.off || 0).toArray();
}
/** the reference of joint side ('a' | 'b') if it still describes the stored anchor, else a freshly derived one */
function sideRef(P, j, side) {
  const l = side === 'a' ? j.la : j.lb, r = j.ref && j.ref[side];
  if (r) { const w = resolveRef(P, r); if (w && Math.hypot(w[0] - l[0], w[1] - l[1], w[2] - l[2]) < 1e-3) return r; }
  return anchorRef(P, l);
}
/** (re)write j.ref for every joint side on part ids (after creation / resize) */
export function refreshRefs(M, ids) {
  const set = ids ? new Set(ids) : null;
  for (const j of M.joints) {
    if (isForce(j)) continue;
    for (const side of ['a', 'b']) { const id = j[side]; if (id == null || (set && !set.has(id))) continue; const P = M.parts.find(p => p.id === id); if (!P) continue; const r = anchorRef(P, side === 'a' ? j.la : j.lb); j.ref = j.ref || {}; if (r) j.ref[side] = r; else delete j.ref[side]; }
  }
}

/**
 * 'param' strategy: the side that is held stays put.
 *  An anchor of the resized part is HELD when what it connects to (the rest of the model without this part, joints + gear/belt couplings)
 *  is anchored (ground joint / fixed part) or attaches to this part at two or more places (a frame, bearings, a meshing gear train back to
 *  the motor …). Held anchors keep their world position; if one of them sits at an end, that end stays and the part grows / shrinks at the
 *  other end. Free anchors: on a connector point (axle end, side pin …) → follow that point; at an end → follow the end; along the body of a
 *  bar (shaft / rod / screw …) → keep their distance from the nearer end; elsewhere → keep their distance from the side that stays.
 *  Returns false when held anchors sit at both ends (cannot be kept) → the older strategies are tried.
 */
function paramRemap(M0, an0, M, P, oldD, notes) {
  const pid = P.id, tol = 5e-4;
  if (an0.groupOf[pid] && an0.groupOf[pid].static) return false;
  // components of the rest of the model
  const par = {}, find = x => par[x] === x ? x : (par[x] = find(par[x])); for (const Q of M0.parts) if (Q.id !== pid) par[Q.id] = Q.id;
  const has = id => id != null && id !== pid && par[id] !== undefined, uni = (a, b) => { if (has(a) && has(b)) par[find(a)] = find(b); };
  for (const j of M0.joints) if (!isForce(j)) uni(j.a, j.b);
  for (const c of M0.couplings) uni(c.pa, c.pb);
  const anch = new Set();
  for (const Q of M0.parts) if (Q.id !== pid && (Q.fixed || (an0.groupOf[Q.id] && an0.groupOf[Q.id].static))) anch.add(find(Q.id));
  for (const j of M0.joints) if (!isForce(j) && j.b == null && has(j.a)) anch.add(find(j.a));
  const sides = []; // { j, side, l, comp }
  for (const j of M.joints) { if (isForce(j)) continue; for (const side of ['a', 'b']) { if (j[side] !== pid) continue; const o = side === 'a' ? j.b : j.a; sides.push({ j, side, l: (side === 'a' ? j.la : j.lb).slice(), comp: o == null ? 'W' : has(o) ? find(o) : 'self' }); } }
  const pos = {}; for (const s of sides) { (pos[s.comp] = pos[s.comp] || []).push(s.l); }
  const multi = c => { const L = pos[c] || []; return L.some(a => L.some(b => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) > 1e-3)); };
  // a component also holds when it carries structure (motor / plate / frame …) or a mesh / belt (moving it would disengage the partner)
  const heavy = new Set();
  for (const Q of M0.parts) if (has(Q.id) && STRUCT.has(Q.type)) heavy.add(find(Q.id));
  for (const c of M0.couplings) for (const id of [c.pa, c.pb]) if (has(id)) heavy.add(find(id));
  for (const s of sides) s.held = s.comp === 'W' || anch.has(s.comp) || heavy.has(s.comp) || (s.comp !== 'self' && multi(s.comp));
  const o = localBox(P.type, oldD), n = localBox(P.type, P.d), A = [], B = [], C1 = [];
  for (let i = 0; i < 3; i++) {
    const a0 = o.mn[i], b0 = o.mx[i], a1 = n.mn[i], b1 = n.mx[i], len = b1 - a1, c0 = (a0 + b0) / 2; C1.push((a1 + b1) / 2);
    if (Math.abs(a0 - a1) < 1e-12 && Math.abs(b0 - b1) < 1e-12) { A.push(a0); B.push(b0); continue; }
    const hu = sides.filter(s => s.held).map(s => s.l[i]), aPin = hu.some(u => u <= a0 + tol), bPin = hu.some(u => u >= b0 - tol);
    if (aPin && bPin) return false;
    let lo = aPin ? a0 : bPin ? b0 - len : c0 - len / 2;
    // held anchors only inside the body (e.g. a meshing bevel near one end): shift the new extent just enough to keep them on the part
    if (!aPin && !bPin && hu.length) { lo = Math.min(lo, Math.min(...hu)); lo = Math.max(lo, Math.max(...hu) - len); }
    A.push(lo); B.push(lo + len);
    if (hu.some(u => u < lo - 1e-3 || u > lo + len + 1e-3)) return false; // a held anchor would fall off the part
  }
  const cA = A.map((a, i) => (a + B[i]) / 2);
  for (const s of sides) {
    let y = null;
    if (!s.held) { const r = sideRef(Object.assign({}, P, { d: oldD }), s.j, s.side); if (r && r.k === 'pt') y = resolveRef(P, r); }
    if (!y) {
      y = [];
      for (let i = 0; i < 3; i++) {
        const u = s.l[i], a0 = o.mn[i], b0 = o.mx[i];
        if (Math.abs(a0 - n.mn[i]) < 1e-12 && Math.abs(b0 - n.mx[i]) < 1e-12) { y.push(u); continue; }
        let w = u;
        if (!s.held) {
          if (u >= b0 - tol) w = B[i] + (u - b0); else if (u <= a0 + tol) w = A[i] + (u - a0);
          else if (LONG[P.type] === i) w = (u - a0 <= b0 - u) ? A[i] + (u - a0) : B[i] - (b0 - u);
          if (u >= a0 - tol && u <= b0 + tol) w = Math.max(A[i], Math.min(B[i], w));
        }
        y.push(w - cA[i] + C1[i]);
      }
    }
    if (s.side === 'a') s.j.la = y; else s.j.lb = y;
  }
  // the part's origin moves so the held side stays where it is
  const sh = new THREE.Vector3(cA[0] - C1[0], cA[1] - C1[1], cA[2] - C1[2]);
  if (sh.lengthSq() > 1e-16) { P.p = V3(P.p).add(sh.applyQuaternion(Q4(P.q))).toArray(); for (const j of M.joints) if (j.a === pid && j.b == null) { /* world side stays */ } }
  if (sides.some(s => s.held)) notes.push('保持被约束的一侧不动');
  return true;
}
/** relative poses of meshing pairs (gear / belt / chain couplings) — must not drift when a third part is resized */
function meshPoses(M, skip) {
  const out = {};
  for (const c of M.couplings) { if (!['gear', 'belt', 'chain'].includes(c.type) || c.pa === skip || c.pb === skip) continue; const A = M.parts.find(p => p.id === c.pa), B = M.parts.find(p => p.id === c.pb); if (!A || !B) continue; out[c.id] = V3(B.p).sub(V3(A.p)).applyQuaternion(Q4(A.q).invert()); }
  return out;
}

/**
 * Resize part pid to dimensions newD, keeping all joints matched.
 * Strategies: smart (anchors follow the ends, except anchors spanning one rigid/anchored body, which keep their centre-relative position), then follow (all anchors follow).
 */
export function resizeModel(M0, pid, newD) {
  const an0 = analyze(M0), P0 = M0.parts.find(p => p.id === pid); if (!P0) return { ok: false, msg: '零件不存在' };
  const oldD = JSON.parse(JSON.stringify(P0.d)), oldR = DEFS[P0.type].pitchR ? pitchRadius(P0) : null;
  // spanning joints: >=2 joints of P into the same rigid group, or >=2 joints into static (ground-fixed) groups
  const mine = M0.joints.filter(j => !isForce(j) && (j.a === pid || j.b === pid)), oth = j => j.a === pid ? j.b : j.a;
  const cnt = {}; let nStatic = 0;
  for (const j of mine) { const o = oth(j); if (o == null) { nStatic++; continue; } const g = an0.groupOf[o]; if (!g || g.id === an0.groupOf[pid].id) continue; cnt[g.id] = (cnt[g.id] || 0) + 1; if (g.static) nStatic++; }
  const spanning = new Set(mine.filter(j => { const o = oth(j); if (o == null) return nStatic >= 2; const g = an0.groupOf[o]; return g && g.id !== an0.groupOf[pid].id && (cnt[g.id] >= 2 || (g.static && nStatic >= 2)); }).map(j => j.id));
  // local axis index of each hinge/slider axis on the resized part (pins along their own axis may keep their position: 'axial' strategy)
  const axisIdx = j => { if (!['revolute', 'prismatic', 'helical'].includes(j.type)) return -1; const A = M0.parts.find(p => p.id === j.a); const n = axW(A, j.axa).applyQuaternion(Q4(P0.q).invert()); const k = [0, 1, 2].find(i => Math.abs(Math.abs(n.getComponent(i)) - 1) < 1e-6); return k == null || k === LONG[P0.type] ? -1 : k; };
  const modes = ['param', ...(spanning.size ? ['smart'] : []), 'follow', 'axial'];
  const mesh0 = meshPoses(M0, pid);
  let best = null;
  for (const mode of modes) {
    const M = JSON.parse(JSON.stringify(M0)), P = M.parts.find(p => p.id === pid), notes = [];
    const wasOnGround = P.fixed && Math.abs(partMinY(P)) < 1e-3, y0 = partMinY(P);
    P.d = Object.assign({}, P.d, newD); if (!P.massManual) P.mass = defaultMass(P);
    if (mode === 'param') { if (!paramRemap(M0, an0, M, P, oldD, notes)) continue; }
    else for (const j of M.joints) {
      const keep = mode === 'smart' && spanning.has(j.id), ka = mode === 'axial' ? axisIdx(j) : -1;
      if (j.a === pid) j.la = remapPoint(P.type, oldD, P.d, j.la, keep, ka);
      if (j.b === pid) j.lb = remapPoint(P.type, oldD, P.d, j.lb, keep, ka);
      if (j.type === 'helical' && (j.a === pid || j.b === pid) && P.type === 'screw' && newD.lead != null && (j.lead == null ? Math.abs(oldD.lead - 0.005) < 1e-12 : Math.abs(j.lead - oldD.lead) < 1e-12)) j.lead = P.d.lead;
    }
    if (wasOnGround) P.p[1] += y0 - partMinY(P);
    // anchored roots: the resized part when it is fixed on the ground, then ground-like fixed parts, then other fixed parts
    const fixedIds = M.parts.filter(p => p.fixed).map(p => p.id);
    const order = [...(wasOnGround ? [pid] : []), ...fixedIds.filter(id => GROUNDISH.has(M.parts.find(p => p.id === id).type)), ...fixedIds];
    // ground-fixed parts outside the resized part's rigid group never move
    const myGroup = new Set(an0.groupOf[pid].parts.map(p => p.id)), pre = fixedIds.filter(id => !myGroup.has(id));
    settle(M, pid, order.filter(id => id !== pid || wasOnGround), pre);
    fixMeshes(M, pid, oldR, notes);
    relax(M);
    // free assemblies must not end up inside the floor
    const an = analyze(M), comp = an.comps[an.compOf[pid]];
    if (comp && !comp.anchored) { const lo = Math.min(...comp.parts.map(partMinY)); if (lo < -1e-4) for (const Q of comp.parts) Q.p[1] -= lo; }
    updateCouplings(M, pid, oldD); refreshRefs(M, [pid]);
    // meshes between other parts (e.g. a bevel pair on the resized shaft and its partner) must stay engaged
    const mesh1 = meshPoses(M, pid); let drift = 0; for (const k in mesh0) if (mesh1[k]) drift = Math.max(drift, mesh0[k].distanceTo(mesh1[k]));
    const gap = maxJointGap(M), r = { M, gap, drift, strategy: mode, notes: [...new Set(notes)] };
    if (drift > GAP_TOL) r.notes.push('啮合位置偏移 ' + (drift * 1000).toFixed(1) + ' mm');
    if (!best || gap + drift < best.gap + best.drift - 1e-6) best = r;
    if (gap < 1e-5 && drift < 1e-5) break;
  }
  if (!best) return { ok: false, msg: '无法修改尺寸' };
  const P = M0.parts.find(p => p.id === pid);
  if (best.gap < GAP_TOL && best.drift > GAP_TOL) return { ok: false, gap: best.gap, drift: best.drift, M: best.M, strategy: best.strategy, notes: best.notes, msg: `无法修改 ${P.name} 的尺寸：修改后齿轮/皮带啮合会脱开（偏移 ${(best.drift * 1000).toFixed(1)} mm——两侧被约束的连接点无法同时保持），已恢复原尺寸。可先拆开一个连接或移动零件后再修改。` };
  if (best.gap < GAP_TOL) { best.ok = true; return best; }
  return { ok: false, gap: best.gap, M: best.M, strategy: best.strategy, notes: best.notes, msg: `无法修改 ${P.name} 的尺寸：修改后连接无法全部重新对齐（最大间隙 ${(best.gap * 1000).toFixed(1)} mm，机构在该尺寸下无法装配），已恢复原尺寸。可先拆开一个连接再修改。` };
}
