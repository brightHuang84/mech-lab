import * as THREE from 'three';

export const MATS = {
  steel: { n: '钢', rho: 7850, mu: 0.5, e: 0.1, color: 0x9aa3ad, metal: 0.75, rough: 0.35 },
  alu: { n: '铝', rho: 2700, mu: 0.45, e: 0.1, color: 0xc9d1db, metal: 0.6, rough: 0.4 },
  brass: { n: '黄铜', rho: 8500, mu: 0.4, e: 0.1, color: 0xd4a843, metal: 0.8, rough: 0.3 },
  wood: { n: '木材', rho: 600, mu: 0.6, e: 0.2, color: 0xb07a45, metal: 0, rough: 0.8 },
  rubber: { n: '橡胶', rho: 1100, mu: 1.0, e: 0.3, color: 0x2b2f36, metal: 0, rough: 0.9 },
  plastic: { n: '塑料', rho: 1200, mu: 0.35, e: 0.3, color: 0x3b82f6, metal: 0, rough: 0.5 },
  iron: { n: '铸铁', rho: 7200, mu: 0.5, e: 0.1, color: 0x4b5563, metal: 0.5, rough: 0.6 },
};

const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0), Z = new THREE.Vector3(0, 0, 1);
const qZY = new THREE.Quaternion().setFromUnitVectors(Y, Z); // rotate Y-aligned geometry onto Z

function mat(part, colorOverride) {
  const m = MATS[part.mat] || MATS.steel;
  return new THREE.MeshStandardMaterial({ color: colorOverride != null ? colorOverride : (part.color != null ? part.color : m.color), metalness: m.metal, roughness: m.rough });
}
const darkMat = () => new THREE.MeshStandardMaterial({ color: 0x1f2937, metalness: 0.2, roughness: 0.7 });
function mesh(g, m, pos, q) { const o = new THREE.Mesh(g, m); o.castShadow = true; o.receiveShadow = true; if (pos) o.position.set(pos[0], pos[1], pos[2]); if (q) o.quaternion.copy(q); return o; }
function cylZ(r, len, seg = 32) { const g = new THREE.CylinderGeometry(r, r, len, seg); g.applyQuaternion(qZY); return g; }

export function gearShape(m, z, holeR) {
  const r = m * z / 2, ra = r + m, rf = Math.max(r - 1.25 * m, r * 0.4), s = 2 * Math.PI / z;
  const sh = new THREE.Shape();
  for (let i = 0; i < z; i++) {
    const t = i * s;
    const P = [[rf, t - 0.3 * s], [r, t - 0.2 * s], [ra, t - 0.12 * s], [ra, t + 0.12 * s], [r, t + 0.2 * s], [rf, t + 0.3 * s], [rf, t + 0.7 * s]];
    P.forEach(([rr, a], k) => { const x = rr * Math.cos(a), y = rr * Math.sin(a); if (i === 0 && k === 0) sh.moveTo(x, y); else sh.lineTo(x, y); });
  }
  if (holeR) { const h = new THREE.Path(); h.absarc(0, 0, holeR, 0, Math.PI * 2, true); sh.holes.push(h); }
  return sh;
}
function extrude(shape, t, segs = 2) { const g = new THREE.ExtrudeGeometry(shape, { depth: t, bevelEnabled: false, curveSegments: segs }); g.translate(0, 0, -t / 2); g.computeVertexNormals(); return g; }
function helixTube(r, pitch, len, wire, turnsSeg = 24) {
  const turns = len / pitch;
  class H extends THREE.Curve { getPoint(u, tgt = new THREE.Vector3()) { const a = u * turns * Math.PI * 2; return tgt.set(r * Math.cos(a), r * Math.sin(a), -len / 2 + u * len); } }
  return new THREE.TubeGeometry(new H(), Math.max(16, Math.ceil(turns * turnsSeg)), wire, 6, false);
}
// eccentric disc cam: circle of radius rb+h/2 whose centre is offset h/2 towards -X (lift = h)
export function camRadius(d, th) { const e = d.h / 2, R = d.rb + e, c = -e * Math.cos(th); return c + Math.sqrt(Math.max(0, R * R - e * e * Math.sin(th) ** 2)); }
function camPts2(d, n = 48) { const a = []; for (let i = 0; i < n; i++) { const th = i / n * Math.PI * 2, r = camRadius(d, th); a.push([r * Math.cos(th), r * Math.sin(th)]); } return a; }

// dims spec: [key, label, kind] kind: len (m, shown mm), int, num, deg
export const DEFS = {
  base: {
    label: '固定底座', cat: 'basic', mat: 'iron', fixed: true, d: { w: 0.12, h: 0.5, dp: 0.08 }, dims: [['w', '宽', 'len'], ['h', '高', 'len'], ['dp', '厚', 'len']],
    shapes: d => [{ k: 'box', h: [d.w / 2, d.h / 2, d.dp / 2] }],
    pts: d => [{ n: '顶部', p: [0, d.h / 2, 0], a: [0, 0, 1] }, { n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '前面', p: [0, 0, d.dp / 2], a: [0, 0, 1] }, { n: '前面上部', p: [0, d.h / 2 - 0.05, d.dp / 2], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.BoxGeometry(d.w, d.h, d.dp), mat(P))); g.add(mesh(new THREE.BoxGeometry(d.w * 1.6, 0.012, d.dp * 1.6), mat(P, 0x374151), [0, -d.h / 2 + 0.006, 0])); return g; }
  },
  rod: {
    label: '连杆', cat: 'basic', mat: 'alu', d: { L: 0.3, w: 0.025, t: 0.01 }, dims: [['L', '孔距', 'len'], ['w', '宽', 'len'], ['t', '厚', 'len']],
    shapes: d => [{ k: 'box', h: [d.L / 2, d.w / 2, d.t / 2] }],
    pts: d => [{ n: '端孔A', p: [-d.L / 2, 0, 0], a: [0, 0, 1] }, { n: '端孔B', p: [d.L / 2, 0, 0], a: [0, 0, 1] }, { n: '中点', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) {
      const g = new THREE.Group(), m = mat(P);
      g.add(mesh(new THREE.BoxGeometry(d.L, d.w, d.t), m));
      for (const s of [-1, 1]) { g.add(mesh(cylZ(d.w / 2, d.t, 24), m, [s * d.L / 2, 0, 0])); g.add(mesh(cylZ(d.w * 0.22, d.t * 1.05, 16), darkMat(), [s * d.L / 2, 0, 0])); }
      return g;
    }
  },
  shaft: {
    label: '轴', cat: 'basic', mat: 'steel', d: { L: 0.15, r: 0.006 }, dims: [['L', '长度', 'len'], ['r', '半径', 'len']],
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.L / 2 }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '前端', p: [0, 0, d.L / 2], a: [0, 0, 1] }, { n: '后端', p: [0, 0, -d.L / 2], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(cylZ(d.r, d.L, 20), mat(P))); g.add(mesh(new THREE.BoxGeometry(d.r * 0.5, d.r * 2.02, d.L * 0.6), darkMat())); return g; }
  },
  wheel: {
    label: '轮/圆盘', cat: 'basic', mat: 'rubber', d: { r: 0.08, t: 0.03 }, dims: [['r', '半径', 'len'], ['t', '厚', 'len']],
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.t / 2 }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '轮缘', p: [d.r * 0.75, 0, d.t / 2], a: [0, 0, 1] }],
    mesh(d, P) {
      const g = new THREE.Group(); g.add(mesh(cylZ(d.r, d.t, 40), mat(P)));
      g.add(mesh(cylZ(d.r * 0.55, d.t * 1.04, 32), mat(P, 0xc0c7d0)));
      for (let i = 0; i < 5; i++) { const a = i / 5 * Math.PI * 2; g.add(mesh(cylZ(d.r * 0.07, d.t * 1.08, 10), darkMat(), [Math.cos(a) * d.r * 0.33, Math.sin(a) * d.r * 0.33, 0])); }
      return g;
    }
  },
  block: {
    label: '滑块', cat: 'basic', mat: 'steel', d: { w: 0.08, h: 0.05, dp: 0.05 }, dims: [['w', '长', 'len'], ['h', '高', 'len'], ['dp', '宽', 'len']],
    shapes: d => [{ k: 'box', h: [d.w / 2, d.h / 2, d.dp / 2] }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '顶面', p: [0, d.h / 2, 0], a: [0, 1, 0] }, { n: '前面', p: [0, 0, d.dp / 2], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.BoxGeometry(d.w, d.h, d.dp), mat(P))); return g; }
  },
  plate: {
    label: '板', cat: 'basic', mat: 'wood', d: { w: 0.3, h: 0.02, dp: 0.2, np: 3 }, dims: [['w', '长', 'len'], ['h', '厚', 'len'], ['dp', '宽', 'len'], ['np', '每边侧面连接点数', 'int']],
    shapes: d => [{ k: 'box', h: [d.w / 2, d.h / 2, d.dp / 2] }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 1, 0] }, { n: '顶面', p: [0, d.h / 2, 0], a: [0, 1, 0] }, { n: '端A', p: [-d.w / 2, 0, 0], a: [0, 0, 1] }, { n: '端B', p: [d.w / 2, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.BoxGeometry(d.w, d.h, d.dp), mat(P))); return g; }
  },
  bearing: {
    label: '轴承座', cat: 'basic', mat: 'iron', d: { w: 0.045, H: 0.0424, dp: 0.02, r: 0.014 }, dims: [['w', '底宽', 'len'], ['H', '中心高', 'len'], ['dp', '厚', 'len'], ['r', '外圈半径', 'len']],
    // origin = bore centre; base at y = -H
    shapes: d => [{ k: 'box', h: [d.w / 2, d.H / 2, d.dp / 2], pos: [0, -d.H / 2, 0] }, { k: 'cyl', r: d.r, hh: d.dp / 2 }],
    pts: d => [{ n: '轴孔', p: [0, 0, 0], a: [0, 0, 1] }, { n: '底面', p: [0, -d.H, 0], a: [0, 1, 0] }],
    mesh(d, P) { const g = new THREE.Group(), m = mat(P, 0x475569); g.add(mesh(new THREE.BoxGeometry(d.w, d.H, d.dp), m, [0, -d.H / 2, 0])); g.add(mesh(cylZ(d.r, d.dp, 28), m)); g.add(mesh(cylZ(d.r * 0.62, d.dp * 1.06, 24), mat(P, 0xd1d5db))); g.add(mesh(cylZ(d.r * 0.32, d.dp * 1.1, 16), darkMat())); g.add(mesh(new THREE.BoxGeometry(d.w * 1.3, 0.005, d.dp * 1.2), m, [0, -d.H + 0.0025, 0])); return g; }
  },
  caster: {
    label: '转向叉(前轮)', cat: 'basic', mat: 'alu', d: { R: 0.1, H: 0.0424, top: 0.135, gw: 0.04 }, dims: [['R', '前伸距', 'len'], ['H', '轮轴高', 'len'], ['top', '叉顶高', 'len'], ['gw', '叉口宽', 'len']],
    // origin = bottom of the vertical steering pin (sits on a plate top); wheel axle at (R, H, 0) along Z
    shapes: d => [{ k: 'box', h: [0.018, 0.004, 0.018], pos: [0, 0.004, 0] }, { k: 'box', h: [0.007, d.top / 2, 0.007], pos: [0, d.top / 2, 0] }, { k: 'box', h: [d.R / 2 + 0.008, 0.005, d.gw / 2 + 0.006], pos: [d.R / 2, d.top, 0] },
      { k: 'box', h: [0.008, (d.top - d.H) / 2 + 0.006, 0.003], pos: [d.R, (d.top + d.H) / 2 - 0.006, d.gw / 2 + 0.003] }, { k: 'box', h: [0.008, (d.top - d.H) / 2 + 0.006, 0.003], pos: [d.R, (d.top + d.H) / 2 - 0.006, -d.gw / 2 - 0.003] }, { k: 'cyl', r: 0.005, hh: d.gw / 2 + 0.006, pos: [d.R, d.H, 0] }],
    pts: d => [{ n: '转向销', p: [0, 0, 0], a: [0, 1, 0] }, { n: '轮轴', p: [d.R, d.H, 0], a: [0, 0, 1] }],
    mesh(d, P) {
      const g = new THREE.Group(), m = mat(P);
      for (const s of DEFS.caster.shapes(d)) if (s.k === 'box') g.add(mesh(new THREE.BoxGeometry(s.h[0] * 2, s.h[1] * 2, s.h[2] * 2), m, s.pos));
      g.add(mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.012, 20), mat(P, 0x6b7280), [0, 0.006, 0]));
      g.add(mesh(cylZ(0.005, d.gw + 0.016, 12), mat(P, 0xd1d5db), [d.R, d.H, 0]));
      return g;
    }
  },
  ball: {
    label: '球/重物', cat: 'basic', mat: 'steel', collideSame: true, d: { r: 0.04 }, dims: [['r', '半径', 'len']],
    shapes: d => [{ k: 'ball', r: d.r }],
    pts: d => [{ n: '球心', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.SphereGeometry(d.r, 32, 20), mat(P))); return g; }
  },
  wall: {
    label: '地面/墙', cat: 'basic', mat: 'wood', fixed: true, ground: true, d: { w: 0.6, h: 0.04, dp: 0.3 }, dims: [['w', '长', 'len'], ['h', '高', 'len'], ['dp', '宽', 'len']],
    shapes: d => [{ k: 'box', h: [d.w / 2, d.h / 2, d.dp / 2] }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '顶面', p: [0, d.h / 2, 0], a: [0, 1, 0] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.BoxGeometry(d.w, d.h, d.dp), mat(P))); return g; }
  },
  motor: {
    label: '电机', cat: 'basic', mat: 'iron', fixed: true, d: { s: 0.08, Ls: 0.04 }, dims: [['s', '机身尺寸', 'len'], ['Ls', '轴伸长', 'len']],
    motor: { mode: 'speed', rpm: 30, torque: 1, maxT: 20 },
    shapes: d => [{ k: 'box', h: [d.s / 2, d.s * 0.53, d.s * 0.6] }, { k: 'cyl', r: d.s * 0.09, hh: d.Ls / 2, pos: [0, 0, d.s * 0.6 + d.Ls / 2] }],
    pts: d => [{ n: '输出轴', p: [0, 0, d.s * 0.6 + d.Ls], a: [0, 0, 1] }, { n: '机身', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) {
      const g = new THREE.Group(), s = d.s;
      g.add(mesh(cylZ(s * 0.48, s * 1.2, 32), mat(P, 0x1e3a8a)));
      g.add(mesh(new THREE.BoxGeometry(s * 1.0, s * 0.12, s * 1.1), mat(P, 0x374151), [0, -s * 0.47, 0]));
      g.add(mesh(cylZ(s * 0.5, s * 0.08, 32), mat(P, 0x9ca3af), [0, 0, s * 0.56]));
      g.add(mesh(cylZ(s * 0.09, d.Ls, 16), mat(P, 0xd1d5db), [0, 0, s * 0.6 + d.Ls / 2]));
      for (let i = 0; i < 8; i++) { const a = i / 8 * Math.PI * 2; g.add(mesh(new THREE.BoxGeometry(s * 0.06, s * 0.06, s * 1.0), mat(P, 0x1e40af), [Math.cos(a) * s * 0.49, Math.sin(a) * s * 0.49, -s * 0.05])); }
      return g;
    }
  },
  // ---------- transmission ----------
  gear: {
    label: '直齿轮', cat: 'trans', mat: 'steel', d: { m: 0.004, z: 20, t: 0.02 }, dims: [['m', '模数', 'len'], ['z', '齿数', 'int'], ['t', '齿宽', 'len']],
    pitchR: d => d.m * d.z / 2,
    shapes: d => [{ k: 'cyl', r: d.m * d.z / 2, hh: d.t / 2 }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); const r = d.m * d.z / 2; g.add(mesh(extrude(gearShape(d.m, d.z, Math.min(0.008, r * 0.3)), d.t), mat(P))); g.add(mesh(cylZ(r * 0.45, d.t * 1.3, 24), mat(P, 0x6b7280))); g.add(mesh(new THREE.BoxGeometry(r * 0.25, r * 0.06, d.t * 1.32), darkMat(), [r * 0.3, 0, 0])); return g; }
  },
  rack: {
    label: '齿条', cat: 'trans', mat: 'steel', d: { m: 0.004, L: 0.5, h: 0.03, t: 0.02 }, dims: [['m', '模数', 'len'], ['L', '长度', 'len'], ['h', '高(至节线)', 'len'], ['t', '齿宽', 'len']],
    shapes: d => [{ k: 'box', h: [d.L / 2, d.h / 2, d.t / 2] }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [1, 0, 0] }, { n: '节线', p: [0, d.h / 2, 0], a: [1, 0, 0] }],
    mesh(d, P) {
      const m = d.m, p = Math.PI * m, n = Math.floor(d.L / p), y0 = d.h / 2 - 1.25 * m, sh = new THREE.Shape();
      sh.moveTo(-d.L / 2, -d.h / 2); sh.lineTo(d.L / 2, -d.h / 2); sh.lineTo(d.L / 2, y0);
      for (let i = n - 1; i >= 0; i--) { const xc = -d.L / 2 + p / 2 + i * p + (d.L - n * p) / 2; sh.lineTo(xc + 0.3 * p, y0); sh.lineTo(xc + 0.13 * p, d.h / 2 + m); sh.lineTo(xc - 0.13 * p, d.h / 2 + m); sh.lineTo(xc - 0.3 * p, y0); }
      sh.lineTo(-d.L / 2, y0); sh.closePath();
      const g = new THREE.Group(); g.add(mesh(extrude(sh, d.t), mat(P))); return g;
    }
  },
  bevel: {
    label: '锥齿轮', cat: 'trans', mat: 'brass', d: { m: 0.004, z: 20, t: 0.015, A: 0.06 }, dims: [['m', '模数', 'len'], ['z', '齿数', 'int'], ['t', '齿宽', 'len'], ['A', '锥顶距(配对节圆半径)', 'len']],
    pitchR: d => d.m * d.z / 2,
    shapes: d => [{ k: 'cyl', r: d.m * d.z / 2 * (1 - d.t / 2 / d.A), hh: d.t / 2, pos: [0, 0, -d.t / 2] }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) {
      const r = d.m * d.z / 2, geo = new THREE.ExtrudeGeometry(gearShape(d.m, d.z, Math.min(0.006, r * 0.25)), { depth: d.t, bevelEnabled: false, curveSegments: 2 });
      geo.translate(0, 0, -d.t); const pa = geo.attributes.position;
      for (let i = 0; i < pa.count; i++) { const s = 1 + pa.getZ(i) / d.A; pa.setXY(i, pa.getX(i) * s, pa.getY(i) * s); }
      geo.computeVertexNormals();
      const g = new THREE.Group(); g.add(mesh(geo, mat(P))); g.add(mesh(cylZ(r * 0.35, d.t * 0.6, 20), mat(P, 0x6b7280), [0, 0, d.t * 0.3])); return g;
    }
  },
  worm: {
    label: '蜗杆', cat: 'trans', mat: 'steel', d: { m: 0.004, r: 0.015, L: 0.12, starts: 1 }, dims: [['m', '模数', 'len'], ['r', '节圆半径', 'len'], ['L', '长度', 'len'], ['starts', '头数', 'int']],
    pitchR: d => d.r,
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.L / 2 }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '端A', p: [0, 0, -d.L / 2], a: [0, 0, 1] }, { n: '端B', p: [0, 0, d.L / 2], a: [0, 0, 1] }],
    mesh(d, P) {
      const g = new THREE.Group(), lead = Math.PI * d.m * d.starts;
      g.add(mesh(cylZ(d.r - 1.1 * d.m, d.L, 24), mat(P)));
      for (let s = 0; s < d.starts; s++) { const h = helixTube(d.r, lead, d.L * 0.8, d.m * 0.75, 32); h.rotateZ(s / d.starts * Math.PI * 2); g.add(mesh(h, mat(P))); }
      g.add(mesh(cylZ(d.r * 0.4, d.L * 1.3, 16), mat(P, 0xd1d5db)));
      return g;
    }
  },
  wormwheel: {
    label: '蜗轮', cat: 'trans', mat: 'brass', d: { m: 0.004, z: 30, t: 0.02 }, dims: [['m', '模数', 'len'], ['z', '齿数', 'int'], ['t', '齿宽', 'len']],
    pitchR: d => d.m * d.z / 2,
    shapes: d => [{ k: 'cyl', r: d.m * d.z / 2, hh: d.t / 2 }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(), r = d.m * d.z / 2; g.add(mesh(extrude(gearShape(d.m, d.z, Math.min(0.008, r * 0.3)), d.t), mat(P))); g.add(mesh(cylZ(r * 0.4, d.t * 1.3, 24), mat(P, 0x6b7280))); g.add(mesh(new THREE.BoxGeometry(r * 0.25, r * 0.06, d.t * 1.32), darkMat(), [r * 0.3, 0, 0])); return g; }
  },
  pulley: {
    label: '带轮', cat: 'trans', mat: 'alu', d: { r: 0.05, t: 0.025 }, dims: [['r', '节圆半径', 'len'], ['t', '宽', 'len']],
    pitchR: d => d.r,
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.t / 2 }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) {
      const r = d.r, t = d.t, prof = [[0.004, -t / 2], [r * 1.12, -t / 2], [r * 1.12, -t * 0.38], [r * 0.97, -t * 0.25], [r * 0.97, t * 0.25], [r * 1.12, t * 0.38], [r * 1.12, t / 2], [0.004, t / 2]].map(([x, y]) => new THREE.Vector2(x, y));
      const geo = new THREE.LatheGeometry(prof, 48); geo.applyQuaternion(qZY);
      const g = new THREE.Group(); g.add(mesh(geo, mat(P)));
      for (let i = 0; i < 4; i++) { const a = i / 4 * Math.PI * 2; g.add(mesh(cylZ(r * 0.12, t * 1.02, 12), darkMat(), [Math.cos(a) * r * 0.55, Math.sin(a) * r * 0.55, 0])); }
      return g;
    }
  },
  sprocket: {
    label: '链轮', cat: 'trans', mat: 'steel', d: { z: 16, p: 0.0127, t: 0.008 }, dims: [['z', '齿数', 'int'], ['p', '链节距', 'len'], ['t', '厚', 'len']],
    pitchR: d => d.p / (2 * Math.sin(Math.PI / d.z)),
    shapes: d => [{ k: 'cyl', r: d.p / (2 * Math.sin(Math.PI / d.z)), hh: d.t / 2 }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const r = d.p / (2 * Math.sin(Math.PI / d.z)), m = 2 * r / d.z; const g = new THREE.Group(); g.add(mesh(extrude(gearShape(m * 1.0, d.z, Math.min(0.008, r * 0.25)), d.t), mat(P))); g.add(mesh(cylZ(r * 0.35, d.t * 2.2, 20), mat(P, 0x6b7280))); return g; }
  },
  cam: {
    label: '凸轮(偏心盘)', cat: 'trans', mat: 'steel', collideSame: true, d: { rb: 0.05, h: 0.03, t: 0.02 }, dims: [['rb', '基圆半径', 'len'], ['h', '升程', 'len'], ['t', '厚', 'len']],
    shapes: d => [{ k: 'cyl', r: d.rb + d.h / 2, hh: d.t / 2, pos: [-d.h / 2, 0, 0] }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const sh = new THREE.Shape(); camPts2(d, 96).forEach(([x, y], i) => i ? sh.lineTo(x, y) : sh.moveTo(x, y)); sh.closePath(); const h = new THREE.Path(); h.absarc(0, 0, 0.007, 0, Math.PI * 2, true); sh.holes.push(h); const g = new THREE.Group(); g.add(mesh(extrude(sh, d.t, 4), mat(P))); return g; }
  },
  follower: {
    label: '从动件', cat: 'trans', mat: 'steel', collideSame: true, d: { L: 0.25, r: 0.01 }, dims: [['L', '长度', 'len'], ['r', '半径', 'len']],
    shapes: d => [{ k: 'ball', r: d.r, pos: [0, -d.L / 2, 0] }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 1, 0] }, { n: '底端', p: [0, -d.L / 2 - d.r, 0], a: [0, 1, 0] }, { n: '顶端', p: [0, d.L / 2, 0], a: [0, 1, 0] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.CapsuleGeometry(d.r, d.L, 8, 20), mat(P))); g.add(mesh(new THREE.CylinderGeometry(d.r * 2.2, d.r * 2.2, d.r * 1.2, 20), mat(P, 0x6b7280), [0, d.L / 2, 0])); return g; }
  },
  screw: {
    label: '丝杠', cat: 'trans', mat: 'steel', d: { r: 0.01, L: 0.5, lead: 0.005 }, dims: [['r', '半径', 'len'], ['L', '长度', 'len'], ['lead', '导程', 'len']],
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.L / 2 }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '端A', p: [0, 0, -d.L / 2], a: [0, 0, 1] }, { n: '端B', p: [0, 0, d.L / 2], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(cylZ(d.r * 0.82, d.L, 20), mat(P))); g.add(mesh(helixTube(d.r * 0.85, d.lead, d.L * 0.98, d.r * 0.2, 16), mat(P, 0xb8c0ca))); return g; }
  },
  nut: {
    label: '螺母', cat: 'trans', mat: 'brass', d: { r: 0.022, L: 0.04 }, dims: [['r', '外接圆半径', 'len'], ['L', '长度', 'len']],
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.L / 2 }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }, { n: '顶面', p: [0, d.r * 0.87, 0], a: [0, 1, 0] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(cylZ(d.r, d.L, 6), mat(P))); g.add(mesh(cylZ(d.r * 0.45, d.L * 1.02, 16), darkMat())); return g; }
  },
  flywheel: {
    label: '飞轮', cat: 'trans', mat: 'iron', d: { r: 0.12, t: 0.03 }, dims: [['r', '半径', 'len'], ['t', '厚', 'len']],
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.t / 2 }],
    pts: d => [{ n: '中心孔', p: [0, 0, 0], a: [0, 0, 1] }, { n: '轮缘', p: [d.r * 0.85, 0, d.t / 2], a: [0, 0, 1] }],
    mesh(d, P) {
      const r = d.r, t = d.t, prof = [[0.006, -t * 0.4], [r * 0.25, -t * 0.4], [r * 0.3, -t * 0.15], [r * 0.78, -t * 0.15], [r * 0.8, -t / 2], [r, -t / 2], [r, t / 2], [r * 0.8, t / 2], [r * 0.78, t * 0.15], [r * 0.3, t * 0.15], [r * 0.25, t * 0.4], [0.006, t * 0.4]].map(([x, y]) => new THREE.Vector2(x, y));
      const geo = new THREE.LatheGeometry(prof, 64); geo.applyQuaternion(qZY); const g = new THREE.Group(); g.add(mesh(geo, mat(P)));
      for (let i = 0; i < 6; i++) { const a = i / 6 * Math.PI * 2; g.add(mesh(cylZ(r * 0.08, t * 0.32, 12), darkMat(), [Math.cos(a) * r * 0.55, Math.sin(a) * r * 0.55, 0])); }
      return g;
    }
  },
  gearbox: {
    label: '减速箱', cat: 'trans', mat: 'iron', fixed: true, d: { w: 0.16, h: 0.12, dp: 0.1, ratio: 3 }, dims: [['w', '长', 'len'], ['h', '高', 'len'], ['dp', '宽', 'len'], ['ratio', '减速比', 'num']],
    shapes: d => [{ k: 'box', h: [d.w / 2, d.h / 2, d.dp / 2] }],
    pts: d => [{ n: '输入轴', p: [-d.w / 2, 0, 0], a: [1, 0, 0] }, { n: '输出轴', p: [d.w / 2, 0, 0], a: [1, 0, 0] }, { n: '中心', p: [0, 0, 0], a: [1, 0, 0] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(new THREE.BoxGeometry(d.w, d.h, d.dp), mat(P, 0x475569))); for (const s of [-1, 1]) g.add(mesh(cylZ(d.h * 0.18, 0.02, 24), mat(P, 0x9ca3af), [s * d.w / 2, 0, 0], new THREE.Quaternion().setFromUnitVectors(Z, X))); for (let i = 0; i < 4; i++) g.add(mesh(new THREE.BoxGeometry(d.w * 0.9, 0.006, d.dp * 1.04), mat(P, 0x334155), [0, -d.h / 2 + (i + 1) * d.h / 5, 0])); return g; }
  },
  clutch: {
    label: '离合器', cat: 'trans', mat: 'steel', d: { r: 0.04, t: 0.04 }, dims: [['r', '半径', 'len'], ['t', '总厚', 'len']],
    shapes: d => [{ k: 'cyl', r: d.r, hh: d.t / 2 }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }, { n: '输入面', p: [0, 0, -d.t / 2], a: [0, 0, 1] }, { n: '输出面', p: [0, 0, d.t / 2], a: [0, 0, 1] }],
    mesh(d, P) { const g = new THREE.Group(); g.add(mesh(cylZ(d.r, d.t * 0.42, 32), mat(P), [0, 0, -d.t * 0.29])); g.add(mesh(cylZ(d.r, d.t * 0.42, 32), mat(P, 0xf59e0b), [0, 0, d.t * 0.29])); g.add(mesh(cylZ(d.r * 0.9, d.t * 0.12, 32), darkMat())); return g; }
  },
  ujoint: {
    label: '万向节', cat: 'trans', mat: 'steel', d: { s: 0.04 }, dims: [['s', '尺寸', 'len']],
    shapes: d => [{ k: 'ball', r: d.s / 2 }],
    pts: d => [{ n: '中心', p: [0, 0, 0], a: [0, 0, 1] }],
    mesh(d, P) { const s = d.s, g = new THREE.Group(), m = mat(P); g.add(mesh(new THREE.BoxGeometry(s, s * 0.18, s * 0.18), m)); g.add(mesh(new THREE.BoxGeometry(s * 0.18, s, s * 0.18), m)); for (const sg of [-1, 1]) { g.add(mesh(new THREE.BoxGeometry(s * 1.2, s * 0.12, s * 0.5), mat(P, 0x6b7280), [0, sg * s * 0.55, -s * 0.2])); g.add(mesh(new THREE.BoxGeometry(s * 0.12, s * 1.2, s * 0.5), mat(P, 0x6b7280), [sg * s * 0.55, 0, s * 0.2])); } return g; }
  },
};

export function shapeVolume(s) {
  if (s.vol) return s.vol;
  switch (s.k) { case 'box': return 8 * s.h[0] * s.h[1] * s.h[2]; case 'cyl': return Math.PI * s.r * s.r * 2 * s.hh; case 'ball': return 4 / 3 * Math.PI * s.r ** 3; case 'cap': return Math.PI * s.r * s.r * 2 * s.hh + 4 / 3 * Math.PI * s.r ** 3; }
  return 1e-4;
}
export function partVolume(P) { return DEFS[P.type].shapes(P.d).reduce((a, s) => a + shapeVolume(s), 0); }
export function defaultMass(P) { return +(partVolume(P) * (MATS[P.mat] || MATS.steel).rho).toPrecision(4); }
export function pitchRadius(P) { const D = DEFS[P.type]; return D.pitchR ? D.pitchR(P.d) : (P.d.r || 0.05); }
