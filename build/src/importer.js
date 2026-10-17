// 3D model import: STL (binary/ascii), OBJ, GLB / GLTF (embedded) -> compact, centred, decimated geometry record.
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { ConvexHull } from 'three/examples/jsm/math/ConvexHull.js';

export const UNITS = { mm: 0.001, cm: 0.01, m: 1, inch: 0.0254 };
export const UNIT_N = { mm: '毫米 mm', cm: '厘米 cm', m: '米 m', inch: '英寸 in' };
export const MAX_TRIS = 40000;

export function b64enc(typed) {
  const u = new Uint8Array(typed.buffer, typed.byteOffset, typed.byteLength); let s = '';
  for (let i = 0; i < u.length; i += 0x8000) s += String.fromCharCode.apply(null, u.subarray(i, i + 0x8000));
  return btoa(s);
}
export function b64dec(str, Type) { const s = atob(str), u = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i); return new Type(u.buffer); }

const ext = n => (n.split('.').pop() || '').toLowerCase();
export const defaultUnit = name => ['glb', 'gltf'].includes(ext(name)) ? 'm' : 'mm';
export const isModelFile = name => ['stl', 'obj', 'glb', 'gltf'].includes(ext(name));

/** parse a file into a flat triangle soup (Float32Array xyz per vertex, file units) */
export async function parseModel(name, buf) {
  const e = ext(name); let root;
  if (e === 'stl') { const g = new STLLoader().parse(buf); root = new THREE.Mesh(g); }
  else if (e === 'obj') { const txt = typeof buf === 'string' ? buf : new TextDecoder().decode(buf); root = new OBJLoader().parse(txt); }
  else if (e === 'glb' || e === 'gltf') {
    const data = e === 'gltf' && typeof buf !== 'string' ? new TextDecoder().decode(buf) : buf;
    root = await new Promise((res, rej) => new GLTFLoader().parse(data, '', g => res(g.scene), err => rej(err)));
  } else throw new Error('不支持的格式：' + e + '（支持 STL / OBJ / GLB / GLTF）');
  root.updateMatrixWorld(true);
  const out = []; const v = new THREE.Vector3();
  root.traverse(o => {
    if (!o.isMesh || !o.geometry || !o.geometry.attributes.position) return;
    const g = o.geometry, pa = g.attributes.position, ix = g.index, n = ix ? ix.count : pa.count;
    for (let i = 0; i + 2 < n; i += 3) for (let k = 0; k < 3; k++) { v.fromBufferAttribute(pa, ix ? ix.getX(i + k) : i + k).applyMatrix4(o.matrixWorld); out.push(v.x, v.y, v.z); }
  });
  if (!out.length) throw new Error('文件中没有三角网格');
  return new Float32Array(out);
}

export function soupBox(soup) { const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]; for (let i = 0; i < soup.length; i += 3) for (let k = 0; k < 3; k++) { const x = soup[i + k]; if (x < mn[k]) mn[k] = x; if (x > mx[k]) mx[k] = x; } return { mn, mx, size: mx.map((x, k) => x - mn[k]) }; }

/** weld (quantised) + optional vertex-clustering decimation; returns {pos, idx} */
function weld(soup, cell) {
  const map = new Map(), pos = [], idx = [], cnt = [];
  const key = (x, y, z) => Math.round(x / cell) + ',' + Math.round(y / cell) + ',' + Math.round(z / cell);
  const tri = [0, 0, 0];
  for (let i = 0; i < soup.length; i += 9) {
    for (let k = 0; k < 3; k++) {
      const x = soup[i + 3 * k], y = soup[i + 3 * k + 1], z = soup[i + 3 * k + 2], kk = key(x, y, z);
      let id = map.get(kk); if (id === undefined) { id = pos.length / 3; map.set(kk, id); pos.push(x, y, z); cnt.push(1); } else if (cell > 1e-6) { const c = ++cnt[id]; pos[3 * id] += (x - pos[3 * id]) / c; pos[3 * id + 1] += (y - pos[3 * id + 1]) / c; pos[3 * id + 2] += (z - pos[3 * id + 2]) / c; }
      tri[k] = id;
    }
    if (tri[0] !== tri[1] && tri[1] !== tri[2] && tri[0] !== tri[2]) idx.push(tri[0], tri[1], tri[2]);
  }
  return { pos: new Float32Array(pos), idx: new Uint32Array(idx) };
}

export function hullOf(points) {
  const h = new ConvexHull().setFromPoints(points), tris = [], set = new Map();
  for (const f of h.faces) { const ring = []; let e = f.edge; do { ring.push(e.head().point); e = e.next; } while (e !== f.edge); for (let i = 1; i + 1 < ring.length; i++) tris.push(ring[0], ring[i], ring[i + 1]); for (const p of ring) set.set(p, 1); }
  let vol = 0; for (let i = 0; i < tris.length; i += 3) vol += tris[i].dot(tris[i + 1].clone().cross(tris[i + 2])) / 6;
  return { pts: [...set.keys()], vol: Math.abs(vol) };
}

/**
 * soup (file units) -> geometry record in metres, centred on the bounding-box centre
 * returns { name, v, i, h (base64), vol (m^3), box: [hx,hy,hz], tris, src }
 */
export function buildGeo(soup, unit, name) {
  const s = UNITS[unit] || 0.001, bb = soupBox(soup), c = bb.mn.map((m, k) => (m + bb.mx[k]) / 2);
  const P = new Float32Array(soup.length); for (let i = 0; i < soup.length; i += 3) for (let k = 0; k < 3; k++) P[i + k] = (soup[i + k] - c[k]) * s;
  const diag = Math.hypot(...bb.size) * s || 1;
  let W = weld(P, diag * 1e-6), cell = diag / 400;
  while (W.idx.length / 3 > MAX_TRIS && cell < diag / 8) { W = weld(P, cell); cell *= 1.5; }
  // mesh volume (closed meshes); fall back to the hull volume when the mesh is open / inconsistent
  let mv = 0; const p = W.pos, ix = W.idx, A = new THREE.Vector3(), B = new THREE.Vector3(), C = new THREE.Vector3();
  for (let t = 0; t < ix.length; t += 3) { A.fromArray(p, 3 * ix[t]); B.fromArray(p, 3 * ix[t + 1]); C.fromArray(p, 3 * ix[t + 2]); mv += A.dot(B.cross(C)) / 6; }
  mv = Math.abs(mv);
  // simplified hull points: cluster vertices on a coarse grid, then convex hull, coarsen until <= 200 points
  let hc = diag / 40, hull;
  for (let it = 0; it < 6; it++) {
    const m = new Map(); for (let i = 0; i < p.length; i += 3) { const k = Math.round(p[i] / hc) + ',' + Math.round(p[i + 1] / hc) + ',' + Math.round(p[i + 2] / hc); if (!m.has(k)) m.set(k, new THREE.Vector3(p[i], p[i + 1], p[i + 2])); }
    hull = hullOf([...m.values()]); if (hull.pts.length <= 200) break; hc *= 1.6;
  }
  const hp = new Float32Array(hull.pts.length * 3); hull.pts.forEach((q, i) => q.toArray(hp, 3 * i));
  const vol = mv > hull.vol * 0.02 && mv < hull.vol * 1.05 ? mv : hull.vol;
  const half = bb.size.map(x => x * s / 2);
  return { name, v: b64enc(W.pos), i: b64enc(W.idx), h: b64enc(hp), vol, hullVol: hull.vol, box: half, tris: W.idx.length / 3 };
}

/** mirrored copy of a record (local x negated, triangle winding flipped) */
export function mirrorGeo(G) {
  const pos = b64dec(G.v, Float32Array).slice(), idx = b64dec(G.i, Uint32Array).slice(), h = b64dec(G.h, Float32Array).slice();
  for (let i = 0; i < pos.length; i += 3) pos[i] = -pos[i]; for (let i = 0; i < h.length; i += 3) h[i] = -h[i];
  for (let t = 0; t < idx.length; t += 3) { const a = idx[t + 1]; idx[t + 1] = idx[t + 2]; idx[t + 2] = a; }
  return Object.assign({}, G, { name: G.name + '(镜像)', v: b64enc(pos), i: b64enc(idx), h: b64enc(h) });
}
