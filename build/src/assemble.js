// Snap-to-assemble rules: connectors per part + rule table + mesh placement.
import * as THREE from 'three';
import { pitchRadius, DEFS } from './parts.js';
import { analyze } from './physics.js';

const v3 = a => new THREE.Vector3(a[0], a[1], a[2]);
const q4 = a => new THREE.Quaternion(a[0], a[1], a[2], a[3]);
const HUBS = ['wheel', 'gear', 'pulley', 'sprocket', 'flywheel', 'cam', 'wormwheel', 'clutch', 'bevel'];
export const SNAP_PX = 42;

/** typed connection points (local frame). side: 1 = on the face pointing along a, 0 = centred, 'auto' = either side */
export function connectors(P) {
  const base = baseConnectors(P);
  // user-placed connectors (any part, typically imported parts): 孔 hole / 轴 pin / 安装面 mount
  if (P.cons) for (const c of P.cons) {
    if (c.role === 'mount') base.push({ n: c.n, role: 'mount', plane: true, p: c.p, a: c.a, u: anyPerp(c.a), hu: 0.002, hv: 0.002, custom: true });
    else base.push({ n: c.n, role: c.role === 'pin' ? 'pin' : 'hole', p: c.p, a: c.a, side: 1, half: 0, custom: true });
  }
  return base;
}
export function anyPerp(a) { const v = v3(a).normalize(), r = Math.abs(v.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0); return r.sub(v.multiplyScalar(r.dot(v))).normalize().toArray(); }
/** the six faces of a centred box as planar mount connectors */
export function boxFaces(hx, hy, hz, names = ['顶面', '底面', '+X面', '-X面', '+Z面', '-Z面']) {
  return [
    { n: names[0], role: 'mount', plane: true, p: [0, hy, 0], a: [0, 1, 0], u: [1, 0, 0], hu: hx, hv: hz },
    { n: names[1], role: 'mount', plane: true, p: [0, -hy, 0], a: [0, -1, 0], u: [1, 0, 0], hu: hx, hv: hz },
    { n: names[2], role: 'mount', plane: true, p: [hx, 0, 0], a: [1, 0, 0], u: [0, 0, 1], hu: hz, hv: hy },
    { n: names[3], role: 'mount', plane: true, p: [-hx, 0, 0], a: [-1, 0, 0], u: [0, 0, 1], hu: hz, hv: hy },
    { n: names[4], role: 'mount', plane: true, p: [0, 0, hz], a: [0, 0, 1], u: [1, 0, 0], hu: hx, hv: hy },
    { n: names[5], role: 'mount', plane: true, p: [0, 0, -hz], a: [0, 0, -1], u: [1, 0, 0], hu: hx, hv: hy }];
}
function baseConnectors(P) {
  const d = P.d, t = P.type;
  if (t === 'mesh') { const b = DEFS.mesh.pts(d), h = [b[3].p[0], b[1].p[1], b[5].p[2]]; return boxFaces(h[0], h[1], h[2]); }
  if (t === 'base') { const y = d.h / 2 - Math.min(0.03, d.h / 4); return [
    { n: '轴销(前)', role: 'pin', p: [0, y, d.dp / 2], a: [0, 0, 1], side: 1 },
    { n: '轴销(后)', role: 'pin', p: [0, y, -d.dp / 2], a: [0, 0, -1], side: 1 },
    { n: '顶面', role: 'mount', plane: true, p: [0, d.h / 2, 0], a: [0, 1, 0], u: [1, 0, 0], hu: d.w / 2, hv: d.dp / 2 },
    { n: '顶面导轨', role: 'rail', line: true, p: [0, d.h / 2, 0], a: [1, 0, 0], nrm: [0, 1, 0], len: d.w },
    ...boxFaces(d.w / 2, d.h / 2, d.dp / 2).slice(2)]; }
  if (t === 'plate' || t === 'wall') {
    const c = [
      { n: '顶面', role: 'mount', plane: true, p: [0, d.h / 2, 0], a: [0, 1, 0], u: [1, 0, 0], hu: d.w / 2, hv: d.dp / 2 },
      { n: '导轨', role: 'rail', line: true, p: [0, d.h / 2, 0], a: [1, 0, 0], nrm: [0, 1, 0], len: d.w }];
    if (t === 'plate') {
      c.push(...boxFaces(d.w / 2, d.h / 2, d.dp / 2).slice(1));
      // side-face connection points (corners + evenly spaced along the long edges, plus the two end faces)
      const n = Math.max(1, Math.round(d.np != null ? d.np : 3)), m = Math.min(0.02, d.w / 4);
      for (let i = 0; i < n; i++) { const x = n === 1 ? 0 : -d.w / 2 + m + (d.w - 2 * m) * i / (n - 1);
        c.push({ n: '侧面' + (i + 1) + '(+Z)', role: 'pin', p: [x, 0, d.dp / 2], a: [0, 0, 1], side: 1 }, { n: '侧面' + (i + 1) + '(-Z)', role: 'pin', p: [x, 0, -d.dp / 2], a: [0, 0, -1], side: 1 }); }
      c.push({ n: '端面(+X)', role: 'pin', p: [d.w / 2, 0, 0], a: [1, 0, 0], side: 1 }, { n: '端面(-X)', role: 'pin', p: [-d.w / 2, 0, 0], a: [-1, 0, 0], side: 1 });
    }
    return c;
  }
  if (t === 'rod') return [
    { n: '端孔A', role: 'hole', p: [-d.L / 2, 0, 0], a: [0, 0, 1], half: d.t / 2, side: 'auto' },
    { n: '端孔B', role: 'hole', p: [d.L / 2, 0, 0], a: [0, 0, 1], half: d.t / 2, side: 'auto' },
    { n: '中点', role: 'hole', p: [0, 0, 0], a: [0, 0, 1], half: d.t / 2, side: 'auto' },
    { n: '杆身', role: 'holeLine', line: true, p: [0, 0, 0], a: [1, 0, 0], jax: [0, 0, 1], len: d.L, half: d.t / 2, side: 'auto' },
    { n: '正面', role: 'mount', plane: true, p: [0, 0, d.t / 2], a: [0, 0, 1], u: [1, 0, 0], hu: d.L / 2, hv: d.w / 2 },
    { n: '背面', role: 'mount', plane: true, p: [0, 0, -d.t / 2], a: [0, 0, -1], u: [1, 0, 0], hu: d.L / 2, hv: d.w / 2 }];
  if (t === 'bearing') return [
    { n: '底面', role: 'mountBase', p: [0, -d.H, 0], a: [0, -1, 0] },
    { n: '轴孔', role: 'bore', p: [0, 0, 0], a: [0, 0, 1], half: d.dp / 2, side: 'auto' }];
  if (t === 'caster') return [
    { n: '转向销', role: 'kingpin', p: [0, 0, 0], a: [0, -1, 0] },
    { n: '轮轴', role: 'bore', p: [d.R, d.H, 0], a: [0, 0, 1], half: 0, side: 0 }];
  if (t === 'shaft') return [
    { n: '中心', role: 'axleMid', p: [0, 0, 0], a: [0, 0, 1], len: d.L },
    { n: '前端', role: 'axleEnd', p: [0, 0, d.L / 2], a: [0, 0, 1] }, { n: '后端', role: 'axleEnd', p: [0, 0, -d.L / 2], a: [0, 0, -1] },
    { n: '轴身', role: 'axleLine', line: true, p: [0, 0, 0], a: [0, 0, 1], len: d.L }];
  if (t === 'screw') return [
    { n: '端A', role: 'axleEnd', p: [0, 0, -d.L / 2], a: [0, 0, -1] }, { n: '端B', role: 'axleEnd', p: [0, 0, d.L / 2], a: [0, 0, 1] },
    { n: '螺纹', role: 'screwLine', line: true, p: [0, 0, 0], a: [0, 0, 1], len: d.L }];
  if (t === 'worm') return [{ n: '端A', role: 'axleEnd', p: [0, 0, -d.L / 2], a: [0, 0, -1] }, { n: '端B', role: 'axleEnd', p: [0, 0, d.L / 2], a: [0, 0, 1] }];
  if (t === 'motor') return [
    { n: '输出轴', role: 'shaftOut', p: [0, 0, d.s * 0.6 + d.Ls / 2], a: [0, 0, 1], side: 0 },
    { n: '底座', role: 'motorBase', p: [0, -d.s * 0.53, 0], a: [0, -1, 0] }];
  if (HUBS.includes(t)) return [{ n: '中心孔', role: 'hub', p: [0, 0, 0], a: [0, 0, 1], half: (d.t || 0.02) / 2 }];
  if (t === 'ball') return [{ n: '球心', role: 'ball', p: [0, 0, 0], a: [0, 0, 1] }];
  if (t === 'block') return [{ n: '底面', role: 'slider', p: [0, -d.h / 2, 0], a: [0, -1, 0], len: d.w }, ...boxFaces(d.w / 2, d.h / 2, d.dp / 2)];
  if (t === 'nut') return [{ n: '中心孔', role: 'nut', p: [0, 0, 0], a: [0, 0, 1], len: d.L }];
  return [];
}

// moving role > target role : { j: joint type, label, align: 'any' | 'anti' }
const RULES = {
  'hole>pin': { j: 'revolute', label: '铰链' }, 'hole>hole': { j: 'revolute', label: '铰链' }, 'hole>shaftOut': { j: 'revolute', label: '铰链 (电机驱动)' },
  'pin>hole': { j: 'revolute', label: '铰链', align: 'any' },
  'hub>shaftOut': { j: 'revolute', label: '铰链 (电机驱动)' }, 'hub>pin': { j: 'revolute', label: '铰链' }, 'hub>hole': { j: 'revolute', label: '铰链' },
  'axleEnd>pin': { j: 'revolute', label: '铰链 (轴承)', align: 'anti' }, 'axleEnd>shaftOut': { j: 'revolute', label: '联轴 (电机驱动)', align: 'anti' },
  'shaftOut>hub': { j: 'revolute', label: '铰链 (电机驱动)' }, 'shaftOut>hole': { j: 'revolute', label: '铰链 (电机驱动)' },
  'motorBase>mount': { j: 'fixed', label: '固定连接 (电机座)', align: 'anti', unfix: true },
  'mountBase>mount': { j: 'fixed', label: '固定连接 (轴承座)', align: 'anti', unfix: true },
  'kingpin>mount': { j: 'revolute', label: '转向铰链 (竖直销)', align: 'anti', alts: ['revolute', 'fixed'], over: { kc: 0.6, damp: 0.08, steer: true } },
  'axleEnd>bore': { j: 'revolute', label: '铰链 (轴承)', side0: true }, 'axleMid>bore': { j: 'revolute', label: '铰链 (轴穿过轴承)', side0: true },
  'hub>bore': { j: 'revolute', label: '铰链 (轮装在轴承上)' }, 'hole>bore': { j: 'revolute', label: '铰链' },
  'ball>hole': { j: 'fixed', label: '固定连接' },
  // line targets
  // anywhere along a rod (5 mm grid)
  'hole>holeLine': { j: 'revolute', label: '铰链 (杆身任意位置)', weak: true }, 'hub>holeLine': { j: 'revolute', label: '铰链 (杆身任意位置)', weak: true },
  'pin>holeLine': { j: 'revolute', label: '铰链 (杆身任意位置)', weak: true }, 'shaftOut>holeLine': { j: 'revolute', label: '铰链 (电机驱动)', weak: true },
  'axleEnd>holeLine': { j: 'revolute', label: '轴插入杆身', weak: true, alts: ['revolute', 'fixed'] },
  // anywhere on any planar face (5 mm grid)
  'face>mount': { j: 'fixed', label: '面贴合', weak: true, alts: ['fixed', 'revolute', 'prismatic', 'spherical'] },
  'hub>mount': { j: 'revolute', label: '铰链 (装在面上)', weak: true, out: true, alts: ['revolute', 'fixed'] },
  'hole>mount': { j: 'revolute', label: '铰链 (装在面上)', weak: true, out: true, alts: ['revolute', 'fixed'] },
  'pin>mount': { j: 'revolute', label: '铰链 (装在面上)', weak: true, out: true, alts: ['revolute', 'fixed'] },
  'axleEnd>mount': { j: 'revolute', label: '轴垂直插在面上', weak: true, alts: ['revolute', 'fixed'] },
  'hub>axleLine': { j: 'fixed', label: '固定在轴上', alts: ['fixed', 'revolute'] }, 'slider>rail': { j: 'prismatic', label: '滑动副' }, 'nut>screwLine': { j: 'screw', label: '螺旋副' },
};

export const JNAME = { revolute: '铰链', prismatic: '滑动副', fixed: '固定连接', spherical: '球铰', helical: '螺旋副' };
const ALTS = { revolute: ['revolute', 'fixed', 'spherical', 'prismatic'], fixed: ['fixed', 'revolute', 'prismatic', 'spherical'], prismatic: ['prismatic', 'fixed', 'revolute'] };
export function altsOf(rule) { return rule.alts || ALTS[rule.j] || null; }

/** does the rigid group of part `id` have a joint of `kinds` to something outside the group (or the world)? */
export function hasJoint(M, id, kinds, an = analyze(M)) {
  const g = an.groupOf[id]; if (!g) return false; const ids = new Set(g.parts.map(p => p.id));
  return M.joints.some(j => kinds.includes(j.type) && (ids.has(j.a) || ids.has(j.b)) && !(ids.has(j.a) && j.b != null && ids.has(j.b)));
}

/**
 * ctx: { M, mover (part), p0, q0 (current pose, Vector3/Quaternion), ray (palette: cursor ray) | null, cursor (screen px Vector2) | null,
 *        exclude: Set of part ids that move with mover, toScreen(Vector3)->Vector2 }
 * returns best candidate { dist, p, q, label, marker, target, kind, apply(app) } or null; also `near` (closest target point even if out of range)
 */
export function findAssembly(ctx) {
  const { M, mover, exclude, toScreen } = ctx, q0 = ctx.q0.clone(), p0 = ctx.p0 ? ctx.p0.clone() : null;
  const cands = [];
  const mcs = connectors(mover);
  const mcWorld = mc => p0 ? v3(mc.p).applyQuaternion(q0).add(p0) : null;
  const sdist = (target, mcW) => { const s = toScreen(target); const ref = ctx.cursor ? ctx.cursor : toScreen(mcW); return s.distanceTo(ref); };
  const probeNear = (pt, mcW) => ctx.ray ? ctx.ray.closestPointToPoint(pt, new THREE.Vector3()) : mcW.clone();
  for (const T of M.parts) {
    if (exclude.has(T.id) || T.id === mover.id) continue;
    const tp = v3(T.p), tq = q4(T.q);
    for (const tc of connectors(T)) {
      const tW = v3(tc.p).applyQuaternion(tq).add(tp), aT = v3(tc.a).applyQuaternion(tq).normalize();
      for (const mc of mcs) {
        const rule = RULES[(mc.plane ? 'face' : mc.role) + '>' + tc.role]; if (!rule) continue;
        const mcW = mcWorld(mc), aM = v3(mc.a).applyQuaternion(q0).normalize();
        let q, anchor, jaxis = null, slideAx = null, jointAt = null, rot = 0;
        if (tc.plane) {
          // planar mount face: anchor = probe point projected onto the face (5 mm grid, clamped to the face)
          const U = v3(tc.u).applyQuaternion(tq).normalize(), V = aT.clone().cross(U).normalize(), pl = new THREE.Plane().setFromNormalAndCoplanarPoint(aT, tW);
          let pr = null; if (ctx.ray) { pr = new THREE.Vector3(); if (!ctx.ray.intersectPlane(pl, pr)) pr = ctx.ray.closestPointToPoint(tW, new THREE.Vector3()); } else pr = mcW.clone();
          const dv = pr.clone().sub(tW), mg = 0.004, g5 = x => Math.round(x / 0.005) * 0.005;
          const cu = Math.max(-tc.hu + mg, Math.min(tc.hu - mg, g5(dv.dot(U)))), cv = Math.max(-tc.hv + mg, Math.min(tc.hv - mg, g5(dv.dot(V))));
          anchor = tW.clone().addScaledVector(U, cu).addScaledVector(V, cv); slideAx = U;
          if (rule.out) { const dir = aM.dot(aT) >= 0 ? aT : aT.clone().negate(); q = new THREE.Quaternion().setFromUnitVectors(aM, dir).multiply(q0); jointAt = anchor.clone(); anchor.addScaledVector(aT, (mc.half || 0) + 0.0005); }
          else { q = new THREE.Quaternion().setFromUnitVectors(aM, aT.clone().negate()).multiply(q0); rot = Math.acos(Math.max(-1, Math.min(1, -aM.dot(aT)))); }
        } else if (tc.line && tc.jax) {
          // hole line along a bar: any position on a 5 mm grid, then behaves like a hole whose axis is jax
          const J = v3(tc.jax).applyQuaternion(tq).normalize(), hl = tc.len / 2, onSeg = new THREE.Vector3();
          const a0 = tW.clone().addScaledVector(aT, -hl), a1 = tW.clone().addScaledVector(aT, hl);
          if (ctx.ray) ctx.ray.distanceSqToSegment(a0, a1, null, onSeg); else new THREE.Line3(a0, a1).closestPointToPoint(mcW, true, onSeg);
          const sAl = Math.max(-hl, Math.min(hl, Math.round(onSeg.clone().sub(tW).dot(aT) / 0.005) * 0.005)), base = tW.clone().addScaledVector(aT, sAl);
          const dir = rule.align === 'anti' ? J.clone().negate() : (aM.dot(J) >= 0 ? J.clone() : J.clone().negate());
          q = new THREE.Quaternion().setFromUnitVectors(aM, dir).multiply(q0);
          const pr = probeNear(base, mcW); const side = Math.sign(pr.clone().sub(base).dot(J)) || 1;
          anchor = base.clone().addScaledVector(J, side * ((tc.half || 0) + (mc.half || 0) + 0.0005)); jaxis = J;
          if (rule.j === 'revolute' && mc.role === 'axleEnd') anchor = base.clone();
        } else if (tc.line) {
          // closest point on the target line segment
          const hl = Math.max(0, tc.len / 2 - (mc.len || (mc.half || 0) * 2) / 2), a0 = tW.clone().addScaledVector(aT, -hl), a1 = tW.clone().addScaledVector(aT, hl);
          const onSeg = new THREE.Vector3();
          if (ctx.ray) ctx.ray.distanceSqToSegment(a0, a1, null, onSeg); else new THREE.Line3(a0, a1).closestPointToPoint(mcW, true, onSeg);
          anchor = onSeg;
          if (tc.role === 'rail') {
            const nT = v3(tc.nrm).applyQuaternion(tq).normalize(), xM = new THREE.Vector3(1, 0, 0).applyQuaternion(q0);
            const X = xM.dot(aT) >= 0 ? aT.clone() : aT.clone().negate(), Z = X.clone().cross(nT);
            q = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(X, nT, Z));
          } else { const dir = aM.dot(aT) >= 0 ? aT : aT.clone().negate(); q = new THREE.Quaternion().setFromUnitVectors(aM, dir).multiply(q0); }
        } else {
          const dir = rule.align === 'anti' ? aT.clone().negate() : (aM.dot(aT) >= 0 ? aT.clone() : aT.clone().negate());
          q = new THREE.Quaternion().setFromUnitVectors(aM, dir).multiply(q0);
          let side = rule.side0 ? 0 : tc.side == null ? 0 : tc.side;
          if (side === 'auto') { const pr = probeNear(tW, mcW); side = Math.sign(pr.clone().sub(tW).dot(aT)) || 1; }
          anchor = tW.clone().addScaledVector(aT, side * ((tc.half || 0) + (mc.half || 0) + (side ? 0.0005 : 0)));
        }
        const p = anchor.clone().sub(v3(mc.p).applyQuaternion(q));
        const dist = sdist(anchor, mcW);
        const axis = (jaxis || aT).clone(), A = (jointAt || anchor).clone(), sAx = slideAx ? slideAx.clone() : null;
        const alts = altsOf(rule);
        cands.push({ dist, p, q, label: rule.label, marker: A, target: T, kind: rule.j, mc: mc.n, tc: tc.n, alts, weak: !!rule.weak, rot,
          apply: (app, jt) => {
            if (rule.unfix) mover.fixed = false;
            if (rule.j === 'screw') {
              const an = analyze(app.M);
              if (!hasJoint(app.M, T.id, ['revolute'], an)) app.addJoint('revolute', T, null, tW.clone(), axis);
              app.addJoint('prismatic', mover, null, A, axis);
              return app.addCoupling('screw', T, mover);
            }
            const t = jt || rule.j;
            return app.addJoint(t, T, mover, A, t === 'prismatic' && sAx && rule.j !== 'prismatic' ? sAx : axis, t === rule.j && rule.over ? Object.assign({}, rule.over) : {});
          } });
      }
    }
    // ---- gear meshes ----
    const mesh = meshCandidate(ctx, mover, T, p0, q0, tp, tq);
    if (mesh) cands.push(mesh);
  }
  let best = null, near = null;
  // nearest on screen; near-ties (same 8px bucket) go to the point closer to the camera
  // "anywhere" connectors (planes / bar lines) give way to discrete connection points within ~24 px
  const key = c => Math.floor((c.dist + (c.weak ? 24 : 0)) / 8) * 1000 + (ctx.camPos ? c.marker.distanceTo(ctx.camPos) : 0) + (c.rot || 0) * 0.3; // least re-orientation wins ties
  for (const c of cands) { if (!near || c.dist < near.dist) near = c; if (c.dist <= SNAP_PX && (!best || key(c) < key(best) - 1e-6)) best = c; }
  return { best, near, cands };
}

function ensureRot(app, P, axisW, center) { if (!hasJoint(app.M, P.id, ['revolute'])) app.addJoint('revolute', P, null, center.clone(), axisW.clone(), { auto: true }); }
function ensurePri(app, P, axisW, center) { if (!hasJoint(app.M, P.id, ['prismatic'])) app.addJoint('prismatic', P, null, center.clone(), axisW.clone()); }
const planeProj = (v, n) => v.clone().addScaledVector(n, -v.dot(n));

export const BEVEL_AZ_STEP = Math.PI / 12; // 15°
/** orthonormal frame (e1, e2) of the plane ⟂ n: e1 horizontal when n is not vertical, else world X projected */
export function pitchFrame(n) {
  let e1 = n.clone().cross(new THREE.Vector3(0, 1, 0)); if (e1.length() < 0.3) e1 = planeProj(new THREE.Vector3(1, 0, 0), n);
  e1.normalize(); const e2 = n.clone().cross(e1).normalize(); return { e1, e2 };
}
/** pose of bevel G meshing with H: local Z = nG exactly, teeth interleaved (zH·φ + zG·ψ ≡ π, φ = azimuth of nG in H's frame,
 *  ψ = azimuth of nH in G's frame), roll about nG chosen closest to G's current roll (q0) so its assembly turns the least */
export function bevelPose(G, H, nG, nH, hq, q0) {
  const Z = new THREE.Vector3(0, 0, 1);
  const ul = nG.clone().applyQuaternion(hq.clone().invert()), phi = Math.atan2(ul.y, ul.x), s = 2 * Math.PI / G.d.z;
  const qb = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(nH.clone(), nG.clone().cross(nH).normalize(), nG.clone())); // X→nH, Z→nG
  let psi = (Math.PI - H.d.z * phi) / G.d.z;
  if (q0) { // current roll of G: azimuth of nH in G's frame after re-aiming its axis onto nG
    const qa = new THREE.Quaternion().setFromUnitVectors(Z.clone().applyQuaternion(q0), nG).multiply(q0), xa = new THREE.Vector3(1, 0, 0).applyQuaternion(qa), ya = new THREE.Vector3(0, 1, 0).applyQuaternion(qa);
    const psi0 = Math.atan2(nH.dot(ya), nH.dot(xa)); psi += Math.round((psi0 - psi) / s) * s; }
  return qb.multiply(new THREE.Quaternion().setFromAxisAngle(Z, -psi)).normalize();
}
export function meshCandidate(ctx, G, H, p0, q0, hp, hq) {
  hq = hq.clone().normalize(); q0 = q0.clone().normalize();
  const Z = new THREE.Vector3(0, 0, 1), nH = Z.clone().applyQuaternion(hq).normalize(), toScreen = ctx.toScreen;
  // probe point: palette -> cursor ray hit with H's mid plane; existing part -> its current centre
  const probe = n => { if (p0) return p0.clone(); const pl = new THREE.Plane().setFromNormalAndCoplanarPoint(n, hp), x = new THREE.Vector3(); return ctx.ray.intersectPlane(pl, x) ? x : null; };
  const dist = (pt) => toScreen(pt).distanceTo(ctx.cursor ? ctx.cursor : toScreen(p0));
  const sameM = (a, b) => a.d.m != null && b.d.m != null && Math.abs(a.d.m - b.d.m) < 1e-9;
  const nG0 = Z.clone().applyQuaternion(q0);
  const orient = (zdir, ref) => { const q = new THREE.Quaternion().setFromUnitVectors(nG0, zdir).multiply(q0); return ref ? ref : q; };
  if (G.type === 'gear' && H.type === 'gear' && sameM(G, H)) {
    const X = probe(nH); if (!X) return null; let u = planeProj(X.clone().sub(hp), nH); if (u.length() < 1e-6) u = new THREE.Vector3(1, 0, 0).applyQuaternion(hq); u.normalize();
    const a = G.d.m * (G.d.z + H.d.z) / 2, c = hp.clone().addScaledVector(u, a);
    // phase so that teeth interleave: angle of u in H's frame
    const ul = u.clone().applyQuaternion(hq.clone().invert()), phi = Math.atan2(ul.y, ul.x);
    const thG = phi + Math.PI - (Math.PI - H.d.z * phi) / G.d.z;
    const q = hq.clone().multiply(new THREE.Quaternion().setFromAxisAngle(Z, thG));
    return { dist: dist(c), p: c, q, label: '啮合 (中心距 ' + (a * 1000).toFixed(1) + 'mm)', marker: c.clone(), target: H, kind: 'gear',
      apply: app => { ensureRot(app, H, nH, hp); ensureRot(app, G, nH, c); return app.addCoupling('gear', H, G); } };
  }
  const gearLike = t => t === 'gear';
  if ((gearLike(G.type) && H.type === 'rack') || (G.type === 'rack' && gearLike(H.type)) && sameM(G, H)) {
    if (!sameM(G, H)) return null;
    if (G.type === 'rack') { // rack moves under/around the gear
      const X = probe(nH); if (!X) return null; let u = planeProj(X.clone().sub(hp), nH); if (u.length() < 1e-6) u = new THREE.Vector3(0, -1, 0); u.normalize();
      const r = pitchRadius(H), c = hp.clone().addScaledVector(u, r + G.d.h / 2), Yr = u.clone().negate(), Xr = Yr.clone().cross(nH).normalize();
      const q = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(Xr, Yr, nH));
      return { dist: dist(c), p: c, q, label: '啮合 (齿条 + 滑动副)', marker: hp.clone().addScaledVector(u, r), target: H, kind: 'gear',
        apply: app => { ensureRot(app, H, nH, hp); ensurePri(app, G, Xr, c); return app.addCoupling('gear', H, G); } };
    }
    // gear moves onto the rack pitch line
    const Xr = new THREE.Vector3(1, 0, 0).applyQuaternion(hq), Yr = new THREE.Vector3(0, 1, 0).applyQuaternion(hq), pitch = hp.clone().addScaledVector(Yr, H.d.h / 2);
    const X = probe(nH); if (!X) return null; const s = Math.max(-H.d.L / 2, Math.min(H.d.L / 2, X.clone().sub(pitch).dot(Xr)));
    const r = pitchRadius(G), c = pitch.clone().addScaledVector(Xr, s).addScaledVector(Yr, r), q = orient(nH.dot(nG0) >= 0 ? nH : nH.clone().negate());
    return { dist: dist(c), p: c, q, label: '啮合 (齿轮齿条)', marker: pitch.clone().addScaledVector(Xr, s), target: H, kind: 'gear',
      apply: app => { ensurePri(app, H, Xr, hp); ensureRot(app, G, nH, c); return app.addCoupling('gear', G, H); } };
  }
  if ((G.type === 'worm' && H.type === 'wormwheel') || (G.type === 'wormwheel' && H.type === 'worm')) {
    if (!sameM(G, H)) return null;
    if (G.type === 'worm') {
      const X = probe(nH); if (!X) return null; let u = planeProj(X.clone().sub(hp), nH); if (u.length() < 1e-6) u = new THREE.Vector3(0, 1, 0); u.normalize();
      const R = pitchRadius(H), c = hp.clone().addScaledVector(u, R + G.d.r), aw = nH.clone().cross(u).normalize(), q = orient(aw.dot(nG0) >= 0 ? aw : aw.clone().negate());
      return { dist: dist(c), p: c, q, label: '啮合 (蜗杆蜗轮)', marker: c.clone(), target: H, kind: 'gear',
        apply: app => { ensureRot(app, H, nH, hp); ensureRot(app, G, aw, c); return app.addCoupling('gear', G, H); } };
    }
    const aW = nH; // worm axis
    const X = p0 ? p0.clone() : (() => { const x = new THREE.Vector3(); ctx.ray.closestPointToPoint(hp, x); return x; })();
    let v = planeProj(X.clone().sub(hp), aW); if (v.length() < 1e-6) v = new THREE.Vector3(0, -1, 0); v.normalize();
    const R = pitchRadius(G), c = hp.clone().addScaledVector(v, R + H.d.r), u = v.clone().negate(), nG = u.clone().cross(aW).normalize(), q = orient(nG);
    return { dist: dist(c), p: c, q, label: '啮合 (蜗杆蜗轮)', marker: c.clone(), target: H, kind: 'gear',
      apply: app => { ensureRot(app, H, aW, hp); ensureRot(app, G, nG, c); return app.addCoupling('gear', H, G); } };
  }
  if (G.type === 'bevel' && H.type === 'bevel' && sameM(G, H)) {
    // 90° bevel pair: common apex O on H's axis; G's axis nG lies exactly in H's pitch plane (nG ⟂ nH).
    // The azimuth of nG around nH comes from where G currently is, but G's centre is never exactly level with the apex
    // (ground lift, stacking offsets ...): taking that residual as-is tilted the second shaft by a few degrees.
    // -> snap the azimuth to a 15° grid of a stable world-derived frame (then add the panel's explicit azimuth ctx.az),
    //    and build G's orientation from (nG, nH) directly, not from a rotation of its current q0.
    const rG = pitchRadius(G), rH = pitchRadius(H), O = hp.clone().addScaledVector(nH, -rG);
    const X = p0 ? p0.clone() : (() => { const x = new THREE.Vector3(); ctx.ray.closestPointToPoint(O, x); return x; })();
    const { e1, e2 } = pitchFrame(nH);
    let nG = planeProj(X.clone().sub(O), nH); if (nG.length() < 1e-6) nG = e1.clone(); nG.normalize();
    const step = ctx.azStep != null ? ctx.azStep : BEVEL_AZ_STEP;
    let az = Math.atan2(nG.dot(e2), nG.dot(e1)); if (step > 0) az = Math.round(az / step) * step; az += ctx.az || 0;
    nG = e1.clone().multiplyScalar(Math.cos(az)).addScaledVector(e2, Math.sin(az)); nG.addScaledVector(nH, -nG.dot(nH)).normalize();
    const c = O.clone().addScaledVector(nG, rH), q = bevelPose(G, H, nG, nH, hq, q0);
    return { dist: dist(c), p: c, q, label: '啮合 (锥齿轮 90°)', marker: c.clone(), target: H, kind: 'gear',
      apply: app => { H.d.A = rG; G.d.A = rH; H.p = O.clone().addScaledVector(nH, rG).toArray(); ensureRot(app, H, nH, v3(H.p)); ensureRot(app, G, nG, c); return app.addCoupling('gear', H, G); } };
  }
  return null;
}
