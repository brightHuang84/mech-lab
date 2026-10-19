// Ground clamp: the floor (y = 0) is solid in edit mode. Every edit that changes poses ends in one post-edit hook (groundFix, called from
// commit / undo / redo / preset + JSON load / relax / previews) that lifts each connected assembly whose actual GEOMETRY (mesh vertices,
// not just the origin or the analytic collision shapes — gear teeth, motor feet, imported meshes …) reaches below the floor.
// Assemblies move rigidly (relative poses + joints intact); ground-anchored joint anchors (lb, b = null) move with them.
import * as THREE from 'three';
import { DEFS } from './parts.js';
import { analyze } from './physics.js';

export const GROUND_TOL = 0.0005;   // allowed penetration (m)
export const GROUND_CLEAR = 0.0005; // clearance after a lift (m) — same as palette placement
const cache = new Map();
/** local-frame vertex cloud (flat xyz) of a part's mesh; cached per type + dimensions */
export function localPts(P) {
  const key = P.type + '|' + JSON.stringify(P.d);
  let pts = cache.get(key); if (pts) return pts;
  const out = [];
  try {
    const g = DEFS[P.type].mesh(P.d, P); g.updateMatrixWorld(true); const v = new THREE.Vector3(), seen = new Set();
    g.traverse(o => {
      if (!o.isMesh || !o.geometry || !o.geometry.attributes.position || (o.userData && o.userData.helper)) return;
      let vis = true; for (let x = o; x && x !== g.parent; x = x.parent) if (x.visible === false) { vis = false; break; } if (!vis) return;
      const pa = o.geometry.attributes.position;
      for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(o.matrixWorld); const k = Math.round(v.x * 1e5) + ',' + Math.round(v.y * 1e5) + ',' + Math.round(v.z * 1e5); if (seen.has(k)) continue; seen.add(k); out.push(v.x, v.y, v.z); }
    });
  } catch (e) { /* fall back to the analytic shapes */ }
  pts = new Float32Array(out); if (out.length) { if (cache.size > 400) cache.clear(); cache.set(key, pts); } return pts;
}
/** lowest world y of the part's geometry at pose (p: [x,y,z] | Vector3, q: [x,y,z,w] | Quaternion); `fallback(P)` = analytic min */
export function geoMinY(P, p, q, fallback) {
  const pts = localPts(P); p = p || P.p; q = q || P.q;
  const px = p.isVector3 ? p.y : p[1], qx = q.isQuaternion ? q.x : q[0], qy = q.isQuaternion ? q.y : q[1], qz = q.isQuaternion ? q.z : q[2], qw = q.isQuaternion ? q.w : q[3];
  if (!pts.length) return fallback ? fallback(Object.assign({}, P, { p: p.isVector3 ? p.toArray() : p, q: q.isQuaternion ? q.toArray() : q })) : px;
  // second row of the rotation matrix → world y = r·v + p.y
  const r0 = 2 * (qx * qy + qz * qw), r1 = 1 - 2 * (qx * qx + qz * qz), r2 = 2 * (qy * qz - qx * qw);
  let lo = Infinity; for (let i = 0; i < pts.length; i += 3) { const y = r0 * pts[i] + r1 * pts[i + 1] + r2 * pts[i + 2]; if (y < lo) lo = y; }
  return lo + px;
}
/** lowest point of the whole model (geometry) */
export function modelMinY(M) { let lo = Infinity; for (const P of M.parts) lo = Math.min(lo, geoMinY(P)); return lo; }
/** lift a set of part ids of model M by dy (+ their ground-anchored joint anchors) */
export function liftIds(M, ids, dy) {
  const set = new Set(ids);
  for (const P of M.parts) if (set.has(P.id)) P.p = [P.p[0], P.p[1] + dy, P.p[2]];
  for (const j of M.joints) if (j.b == null && set.has(j.a) && Array.isArray(j.lb)) j.lb = [j.lb[0], j.lb[1] + dy, j.lb[2]];
}
/** the post-edit hook: lift every connected assembly (or only those containing `only` ids) whose geometry is below the floor.
 *  Returns { dy (max lift), lifted: [ids...] } */
export function groundFixModel(M, only) {
  const an = analyze(M), out = { dy: 0, lifted: [] }, want = only ? new Set(only) : null, done = new Set();
  for (const P of M.parts) {
    const cid = an.compOf[P.id]; if (done.has(cid)) continue; done.add(cid);
    const comp = an.comps[cid]; if (!comp) continue; if (want && !comp.parts.some(Q => want.has(Q.id))) continue;
    let lo = Infinity; for (const Q of comp.parts) lo = Math.min(lo, geoMinY(Q));
    if (!isFinite(lo) || lo >= -GROUND_TOL) continue;
    const dy = -lo + GROUND_CLEAR, ids = comp.parts.map(Q => Q.id); liftIds(M, ids, dy);
    out.dy = Math.max(out.dy, dy); out.lifted.push(...ids);
  }
  return out;
}
