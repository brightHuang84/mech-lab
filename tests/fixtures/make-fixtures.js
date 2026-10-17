// generates small test models: box (STL binary + ascii), L-bracket (OBJ), cylinder (GLB)
const fs = require('fs');
function boxTris(sx, sy, sz, ox = 0, oy = 0, oz = 0) {
  const x = [ox - sx / 2, ox + sx / 2], y = [oy - sy / 2, oy + sy / 2], z = [oz - sz / 2, oz + sz / 2], v = (i, j, k) => [x[i], y[j], z[k]];
  const q = (a, b, c, d) => [[a, b, c], [a, c, d]];
  return [...q(v(0,0,1), v(1,0,1), v(1,1,1), v(0,1,1)), ...q(v(1,0,0), v(0,0,0), v(0,1,0), v(1,1,0)), ...q(v(1,0,1), v(1,0,0), v(1,1,0), v(1,1,1)), ...q(v(0,0,0), v(0,0,1), v(0,1,1), v(0,1,0)), ...q(v(0,1,1), v(1,1,1), v(1,1,0), v(0,1,0)), ...q(v(0,0,0), v(1,0,0), v(1,0,1), v(0,0,1))];
}
const box = boxTris(40, 20, 10, 100, 50, 0); // offset on purpose (importer centres it)
const bin = Buffer.alloc(84 + 50 * box.length); bin.writeUInt32LE(box.length, 80);
box.forEach((t, i) => { const o = 84 + 50 * i; t.forEach((p, k) => p.forEach((c, m) => bin.writeFloatLE(c, o + 12 + 12 * k + 4 * m))); });
fs.writeFileSync(__dirname + '/box40x20x10.stl', bin);
fs.writeFileSync(__dirname + '/box40x20x10-ascii.stl', 'solid box\n' + box.map(t => ' facet normal 0 0 0\n  outer loop\n' + t.map(p => '   vertex ' + p.join(' ')).join('\n') + '\n  endloop\n endfacet').join('\n') + '\nendsolid box\n');
// OBJ L-bracket: two boxes (60x5x30 + 5x40x30) in mm
const L = [...boxTris(60, 5, 30, 0, 2.5, 0), ...boxTris(5, 40, 30, -27.5, 20, 0)];
let obj = '# L bracket\n'; L.forEach(t => t.forEach(p => obj += 'v ' + p.join(' ') + '\n')); L.forEach((t, i) => obj += `f ${3 * i + 1} ${3 * i + 2} ${3 * i + 3}\n`);
fs.writeFileSync(__dirname + '/bracket.obj', obj);
// GLB cylinder r=0.05 h=0.03 (metres), axis Y
const N = 32, pos = [], idx = []; const r = 0.05, h = 0.015;
for (let i = 0; i < N; i++) { const a = i / N * Math.PI * 2; pos.push(r * Math.cos(a), -h, r * Math.sin(a), r * Math.cos(a), h, r * Math.sin(a)); }
pos.push(0, -h, 0, 0, h, 0); const cb = 2 * N, ct = 2 * N + 1;
for (let i = 0; i < N; i++) { const j = (i + 1) % N, a = 2 * i, b = 2 * i + 1, c = 2 * j, d = 2 * j + 1; idx.push(a, b, d, a, d, c, cb, a, c, ct, d, b); }
const pb = Buffer.from(new Float32Array(pos).buffer), ib = Buffer.from(new Uint16Array(idx).buffer), pad = b => Buffer.concat([b, Buffer.alloc((4 - b.length % 4) % 4)]);
const binc = Buffer.concat([pad(pb), pad(ib)]);
const mn = [-r, -h, -r], mx = [r, h, r];
const json = { asset: { version: '2.0' }, scene: 0, scenes: [{ nodes: [0] }], nodes: [{ mesh: 0, name: 'wheel' }], meshes: [{ primitives: [{ attributes: { POSITION: 0 }, indices: 1 }] }],
  buffers: [{ byteLength: binc.length }], bufferViews: [{ buffer: 0, byteOffset: 0, byteLength: pb.length }, { buffer: 0, byteOffset: pad(pb).length, byteLength: ib.length }],
  accessors: [{ bufferView: 0, componentType: 5126, count: pos.length / 3, type: 'VEC3', min: mn, max: mx }, { bufferView: 1, componentType: 5123, count: idx.length, type: 'SCALAR' }] };
const jb = pad(Buffer.from(JSON.stringify(json)).toString().padEnd(Math.ceil(JSON.stringify(json).length / 4) * 4, ' ') && Buffer.from(JSON.stringify(json).padEnd(Math.ceil(JSON.stringify(json).length / 4) * 4, ' ')));
const head = Buffer.alloc(12); head.writeUInt32LE(0x46546C67, 0); head.writeUInt32LE(2, 4); head.writeUInt32LE(12 + 8 + jb.length + 8 + binc.length, 8);
const c0 = Buffer.alloc(8); c0.writeUInt32LE(jb.length, 0); c0.writeUInt32LE(0x4E4F534A, 4); const c1 = Buffer.alloc(8); c1.writeUInt32LE(binc.length, 0); c1.writeUInt32LE(0x004E4942, 4);
fs.writeFileSync(__dirname + '/wheel-r50.glb', Buffer.concat([head, c0, jb, c1, binc]));
console.log('fixtures written');
