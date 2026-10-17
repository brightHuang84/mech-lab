import RAPIER from '@dimforge/rapier3d-compat';
import * as THREE from 'three';
import { DEFS, MATS, shapeVolume, pitchRadius, geoData } from './parts.js';
export { RAPIER };

export const V3 = a => new THREE.Vector3(a[0], a[1], a[2]);
export const Q4 = a => new THREE.Quaternion(a[0], a[1], a[2], a[3]);
const XA = new THREE.Vector3(1, 0, 0);
const qZY = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1));
const rv = v => ({ x: v.x, y: v.y, z: v.z }), rq = q => ({ x: q.x, y: q.y, z: q.z, w: q.w });
const tv = v => new THREE.Vector3(v.x, v.y, v.z), tq = q => new THREE.Quaternion(q.x, q.y, q.z, q.w);
export const ptW = (P, l) => V3(l).applyQuaternion(Q4(P.q)).add(V3(P.p));
export const axW = (P, a) => V3(a).applyQuaternion(Q4(P.q)).normalize();
export const ptL = (P, w) => w.clone().sub(V3(P.p)).applyQuaternion(Q4(P.q).invert());
export const axL = (P, w) => w.clone().applyQuaternion(Q4(P.q).invert()).normalize();
export const isForce = j => j.type === 'spring' || j.type === 'damper';

/** groups (rigidly fixed sets) and components (connected through any joint) */
export function analyze(M) {
  const par = {}; const find = x => par[x] === x ? x : (par[x] = find(par[x]));
  M.parts.forEach(p => par[p.id] = p.id);
  const has = id => par[id] !== undefined;
  const worldFixed = new Set();
  for (const j of M.joints) if (j.type === 'fixed' && has(j.a)) { if (j.b == null) worldFixed.add(j.a); else if (has(j.b)) par[find(j.a)] = find(j.b); }
  const groups = {}, groupOf = {};
  for (const p of M.parts) { const r = find(p.id); (groups[r] = groups[r] || { id: r, parts: [], static: false }).parts.push(p); groupOf[p.id] = groups[r]; }
  for (const p of M.parts) if (p.fixed || worldFixed.has(p.id)) groupOf[p.id].static = true;
  const cp = {}; const cf = x => cp[x] === x ? x : (cp[x] = cf(cp[x]));
  Object.keys(groups).forEach(g => cp[g] = g);
  const link = (a, b) => { if (a != null && b != null && has(a) && has(b)) cp[cf(groupOf[a].id)] = cf(groupOf[b].id); };
  const anchoredJ = new Set();
  for (const j of M.joints) { if (j.b == null && !isForce(j)) anchoredJ.add(j.a); if (j.type !== 'fixed' && !isForce(j)) link(j.a, j.b); }
  for (const c of M.couplings) { const ja = M.joints.find(j => j.id === c.ja), jb = M.joints.find(j => j.id === c.jb); if (ja && jb) { link(ja.a, jb.a); } }
  const compOf = {}, comps = {};
  for (const p of M.parts) { const c = cf(groupOf[p.id].id); compOf[p.id] = c; (comps[c] = comps[c] || { id: c, parts: [], anchored: false }).parts.push(p); if (groupOf[p.id].static || anchoredJ.has(p.id)) comps[c].anchored = true; }
  return { groups, groupOf, compOf, comps };
}

function invInertia(body, out) {
  const ip = body.invPrincipalInertia(), fr = tq(body.principalInertiaLocalFrame()), R = tq(body.rotation()).multiply(fr);
  const m = new THREE.Matrix4().makeRotationFromQuaternion(R), e = m.elements;
  const r = [[e[0], e[4], e[8]], [e[1], e[5], e[9]], [e[2], e[6], e[10]]], d = [ip.x, ip.y, ip.z];
  const o = out || new THREE.Matrix3(), oe = o.elements;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let s = 0; for (let k = 0; k < 3; k++) s += r[i][k] * d[k] * r[j][k]; oe[j * 3 + i] = s; }
  return o;
}
function inertiaW(body) {
  const ip = body.principalInertia(), fr = tq(body.principalInertiaLocalFrame()), R = tq(body.rotation()).multiply(fr);
  const m = new THREE.Matrix4().makeRotationFromQuaternion(R), e = m.elements;
  const r = [[e[0], e[4], e[8]], [e[1], e[5], e[9]], [e[2], e[6], e[10]]], d = [ip.x, ip.y, ip.z];
  const o = new THREE.Matrix3(), oe = o.elements;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let s = 0; for (let k = 0; k < 3; k++) s += r[i][k] * d[k] * r[j][k]; oe[j * 3 + i] = s; }
  return o;
}

export class Sim {
  constructor(M, opt = {}) {
    this.M = M; this.edit = !!opt.edit; this.g = opt.gravity === false || this.edit ? 0 : 9.81;
    this.A = analyze(M);
    const W = this.world = new RAPIER.World({ x: 0, y: -this.g, z: 0 });
    W.numSolverIterations = 12; W.timestep = 1 / 240;
    try { const ip = W.integrationParameters; ip.normalizedAllowedLinearError = 0.0002; ip.contact_natural_frequency = 120; } catch (e) { }
    this.h = 1 / 240; this.time = 0;
    this.pb = {}; this.colPart = new Map(); this.bodies = [];
    // floor
    const fb = W.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, -0.5, 0));
    const fcd = RAPIER.ColliderDesc.cuboid(60, 0.5, 60).setFriction(0.9).setRestitution(0.1);
    if (this.edit) fcd.setCollisionGroups(0); else fcd.setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);
    const fc = W.createCollider(fcd, fb); this.colPart.set(fc.handle, { floor: true });
    this.worldBody = W.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    for (const g of Object.values(this.A.groups)) {
      const root = g.parts[0], gp = V3(root.p), gq = Q4(root.q), gqi = gq.clone().invert();
      const kin = opt.kinematic && g.parts.some(p => p.id === opt.kinematic);
      const bd = g.static ? RAPIER.RigidBodyDesc.fixed() : kin ? RAPIER.RigidBodyDesc.kinematicPositionBased() : RAPIER.RigidBodyDesc.dynamic();
      bd.setTranslation(gp.x, gp.y, gp.z).setRotation(rq(gq)).setCanSleep(false);
      if (!this.edit) { bd.setCcdEnabled(g.parts.some(p => p.type === 'ball' || p.type === 'wheel')); }
      const body = W.createRigidBody(bd);
      const rec = { body, group: g, static: g.static, motorTau: 0 };
      this.bodies.push(rec);
      for (const p of g.parts) {
        const D = DEFS[p.type], op = V3(p.p).sub(gp).applyQuaternion(gqi), oq = gqi.clone().multiply(Q4(p.q));
        this.pb[p.id] = { rec, body, op, oq, part: p };
        const shapes = D.shapes(p.d), vt = shapes.reduce((a, s) => a + shapeVolume(s), 0), mt = MATS[p.mat] || MATS.steel;
        for (const s of shapes) {
          let cd, sq = new THREE.Quaternion();
          if (s.k === 'box') cd = RAPIER.ColliderDesc.cuboid(...s.h);
          else if (s.k === 'cyl') { cd = RAPIER.ColliderDesc.cylinder(s.hh, s.r); sq = qZY.clone(); }
          else if (s.k === 'ball') cd = RAPIER.ColliderDesc.ball(s.r);
          else if (s.k === 'cap') cd = RAPIER.ColliderDesc.capsule(s.hh, s.r);
          else if (s.k === 'hull') cd = RAPIER.ColliderDesc.convexHull(new Float32Array(s.pts));
          else if (s.k === 'tri') { const c = geoData(s.gid); if (c) { const v = new Float32Array(c.pos.length); for (let i = 0; i < v.length; i++) v[i] = c.pos[i] * s.sc; cd = RAPIER.ColliderDesc.trimesh(v, c.idx); } }
          if (!cd) { cd = RAPIER.ColliderDesc.ball(0.01); }
          const sp = V3(s.pos || [0, 0, 0]).applyQuaternion(oq).add(op);
          cd.setTranslation(sp.x, sp.y, sp.z).setRotation(rq(oq.clone().multiply(sq)));
          cd.setMass(Math.max(1e-4, p.mass) * shapeVolume(s) / vt).setFriction(p.mu != null ? p.mu : mt.mu).setRestitution(p.e != null ? p.e : mt.e);
          if (this.edit) cd.setCollisionGroups(0); else cd.setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);
          const c = W.createCollider(cd, body); this.colPart.set(c.handle, { part: p, comp: this.A.compOf[p.id], group: g.id, D });
        }
      }
    }
    const self = this;
    this.hooks = {
      filterContactPair(c1, c2) {
        const a = self.colPart.get(c1), b = self.colPart.get(c2);
        if (!a || !b) return RAPIER.SolverFlags.COMPUTE_IMPULSE;
        if (a.floor || b.floor) return RAPIER.SolverFlags.COMPUTE_IMPULSE;
        if (a.group === b.group) return RAPIER.SolverFlags.EMPTY;
        if (a.D.ground || b.D.ground || a.comp !== b.comp || (a.D.collideSame && b.D.collideSame) || a.part.collide || b.part.collide) return RAPIER.SolverFlags.COMPUTE_IMPULSE;
        return RAPIER.SolverFlags.EMPTY;
      },
      filterIntersectionPair() { return true; }
    };
    // joints
    this.rj = []; this.drives = []; this.forces = []; this.motors = []; this.couplings = []; this.helix = [];
    for (const j of M.joints) {
      if (j.type === 'fixed') continue;
      const PA = M.parts.find(p => p.id === j.a), PB = j.b != null ? M.parts.find(p => p.id === j.b) : null;
      if (!PA || (j.b != null && !PB)) continue;
      const ra = this.pb[PA.id], rb = PB ? this.pb[PB.id] : null;
      const bA = ra.body, bB = rb ? rb.body : this.worldBody;
      const wA = ptW(PA, j.la), wB = PB ? ptW(PB, j.lb) : V3(j.lb);
      const l1 = this.toBody(bA, wA), l2 = this.toBody(bB, wB);
      if (isForce(j)) { this.forces.push({ j, bA, bB, l1, l2 }); continue; }
      if (bA === bB) continue;
      const nW = axW(PA, j.axa);
      const rec = { j, bA, bB, l1, l2, axA: nW.clone().applyQuaternion(tq(bA.rotation()).invert()) };
      if (!(bA.isFixed() && bB.isFixed())) {
        let data;
        if (j.type === 'revolute') data = RAPIER.JointData.revolute(rv(l1), rv(l2), { x: 1, y: 0, z: 0 });
        else if (j.type === 'prismatic') data = RAPIER.JointData.prismatic(rv(l1), rv(l2), { x: 1, y: 0, z: 0 });
        else if (j.type === 'helical') data = RAPIER.JointData.generic(rv(l1), rv(l2), { x: 1, y: 0, z: 0 }, 2 | 4 | 16 | 32);
        else data = RAPIER.JointData.spherical(rv(l1), rv(l2));
        const ij = W.createImpulseJoint(data, bA, bB, true);
        if (j.type !== 'spherical') {
          const Qj = new THREE.Quaternion().setFromUnitVectors(XA, nW);
          ij.setLocalFrame1(rv(l1), rq(tq(bA.rotation()).invert().multiply(Qj)));
          ij.setLocalFrame2(rv(l2), rq(tq(bB.rotation()).invert().multiply(Qj)));
        }
        ij.setContactsEnabled(false); rec.ij = ij;
        this.configJoint(rec, j, PA, PB);
      }
      this.rj.push(rec);
      if (j.type === 'helical' && !this.edit) this.helix.push({ rec, k: (j.lead != null ? j.lead : 0.005) / (2 * Math.PI) });
      // motor
      const mp = PA.type === 'motor' ? PA : PB && PB.type === 'motor' ? PB : null;
      if (mp && j.type === 'revolute' && !this.edit && !j.noMotor && !(j.drive && j.drive.on)) {
        const mb = this.pb[mp.id].body, db = mp === PA ? bB : bA;
        const mrec = { part: mp, j, mb, db, nL: V3([0, 0, 1]).applyQuaternion(this.pb[mp.id].oq), tau: 0, P: 0, ij: rec.ij, rec };
        const mz = V3([0, 0, 1]).applyQuaternion(Q4(mp.q)); mrec.dir = j.motorRev ? -1 : 1; mrec.sgn = Math.sign(mz.dot(nW) || 1) * (mp === PA ? 1 : -1) * mrec.dir;
        this.motors.push(mrec);
      }
    }
    if (!this.edit) for (const c of M.couplings) { const r = this.makeCoupling(c); if (r) this.couplings.push(r); }
  }
  /** per-joint options: limits (edit + run), drive / centering spring / damping / friction (run only) */
  configJoint(rec, j, PA, PB) {
    const ij = rec.ij; if (!(j.type === 'revolute' || j.type === 'prismatic') || !ij.setLimits) return;
    if (j.lim && j.lim.on) ij.setLimits(Math.min(j.lim.lo, j.lim.hi), Math.max(j.lim.lo, j.lim.hi));
    if (this.edit) return;
    const isMotorJ = (PA.type === 'motor' || (PB && PB.type === 'motor')) && j.type === 'revolute' && !j.noMotor;
    try {
      if (j.drive && j.drive.on) { ij.configureMotorModel(RAPIER.MotorModel.ForceBased); rec.drive = j.drive; rec.dkey = null; this.drives.push(rec); this.applyDrive(rec); }
      else if (isMotorJ) return;
      else if ((j.kc || 0) > 0 || (j.damp || 0) > 0) { ij.configureMotorModel(RAPIER.MotorModel.ForceBased); ij.configureMotor(j.rest || 0, 0, j.kc || 0, j.damp || 0); ij.setMotorMaxForce(1e6); }
      else if ((j.fric || 0) > 0) { ij.configureMotorModel(RAPIER.MotorModel.ForceBased); ij.configureMotorVelocity(0, 1e3); ij.setMotorMaxForce(j.fric); }
    } catch (e) { }
  }
  /** keyboard input of a joint drive: null = no keys bound (always on), else -1 / 0 / +1 */
  driveInput(D) { if (!D.keyP && !D.keyN) return null; const K = this.keys || new Set(); return (K.has(D.keyP) ? 1 : 0) - (K.has(D.keyN) ? 1 : 0); }
  /** built-in joint motor: speed / torque / servo (position) mode, optionally key-controlled */
  applyDrive(rec) {
    const D = rec.drive, ij = rec.ij, inp = this.driveInput(D), rv = rec.j.type === 'revolute', mode = D.mode || 'speed', maxF = D.maxF != null ? D.maxF : 10;
    let key;
    if (mode === 'servo') {
      const goal = (D.pos || 0) * (inp == null ? 1 : inp), rate = D.rate != null ? D.rate : (rv ? 3 : 0.2);
      if (rec.servoT == null) rec.servoT = inp == null ? goal : 0;
      const st = rate * this.h; rec.servoT += Math.max(-st, Math.min(st, goal - rec.servoT));
      key = 'p' + rec.servoT.toFixed(5);
      if (key !== rec.dkey) { ij.configureMotorPosition(rec.servoT, D.kp != null ? D.kp : (rv ? 60 : 3000), D.kd != null ? D.kd : (rv ? 3 : 150)); ij.setMotorMaxForce(maxF); }
    } else if (mode === 'torque') {
      const u = inp == null ? 1 : inp, t = (D.tq != null ? D.tq : maxF) * u; key = 't' + t;
      if (key !== rec.dkey) { ij.configureMotorVelocity(Math.sign(t) * 1e3, 1e4); ij.setMotorMaxForce(Math.abs(t)); }
    } else {
      const u = inp == null ? 1 : inp, v = (D.v || 0) * u; key = 's' + v;
      // released keys: coast (tiny drag) unless "brake" is set
      if (key !== rec.dkey) { ij.configureMotorVelocity(v, 1e4); ij.setMotorMaxForce(u === 0 ? maxF * (D.brake ? 1 : 0.02) : maxF); }
    }
    rec.dkey = key; rec.dIn = inp;
  }
  toBody(b, w) { return w.clone().sub(tv(b.translation())).applyQuaternion(tq(b.rotation()).invert()); }
  toWorld(b, l) { return l.clone().applyQuaternion(tq(b.rotation())).add(tv(b.translation())); }
  partPose(id) { const r = this.pb[id]; if (!r) return null; const bq = tq(r.body.rotation()); return { p: r.op.clone().applyQuaternion(bq).add(tv(r.body.translation())), q: bq.multiply(r.oq) }; }
  jointAxisW(rec) { return rec.axA.clone().applyQuaternion(tq(rec.bA.rotation())); }
  /** constraint rows: list of {b, lin, ang} giving the relative joint rate (B relative to A along the axis) */
  jointRow(rec) {
    const n = this.jointAxisW(rec);
    if (rec.j.type === 'revolute') return [{ b: rec.bA, lin: null, ang: n.clone().negate() }, { b: rec.bB, lin: null, ang: n.clone() }];
    const p = this.toWorld(rec.bA, rec.l1);
    // sliders cannot rotate relative to their guide: use pure translation (impulse through the centre of mass)
    return [{ b: rec.bA, lin: n.clone().negate(), ang: null }, { b: rec.bB, lin: n.clone(), ang: null }];
  }
  /** sign of a part relative to the joint: +1 if part is on side B */
  partSide(rec, partId) { const g = this.pb[partId].body; return g === rec.bB ? 1 : -1; }
  findJointFor(partId, kinds) {
    const g = this.pb[partId] && this.pb[partId].body; if (!g) return null;
    return this.rj.find(r => kinds.includes(r.j.type) && (r.bA === g || r.bB === g) && r.bA !== r.bB) || null;
  }
  makeCoupling(c) {
    if (c.off) return null;
    const ra = this.rj.find(r => r.j.id === c.ja), rb = this.rj.find(r => r.j.id === c.jb);
    if (!ra || !rb) return null;
    return { c, ra, rb, sa: c.sa, sb: c.sb, k: c.k, lam: 0 };
  }
  rate(row) { let s = 0; for (const e of row) { if (e.lin) s += e.lin.dot(tv(e.b.linvel())); if (e.ang) s += e.ang.dot(tv(e.b.angvel())); } return s; }
  solveRow(row, target, maxImp, accum) {
    // merge per body
    const m = new Map();
    for (const e of row) { if (!e.b.isDynamic()) continue; let o = m.get(e.b.handle); if (!o) m.set(e.b.handle, o = { b: e.b, lin: new THREE.Vector3(), ang: new THREE.Vector3() }); if (e.lin) o.lin.add(e.lin); if (e.ang) o.ang.add(e.ang); }
    let W = 0, r = 0; const list = [...m.values()];
    for (const o of list) { o.im = o.b.invMass(); o.I = invInertia(o.b); o.Ia = o.ang.clone().applyMatrix3(o.I); W += o.im * o.lin.lengthSq() + o.ang.dot(o.Ia); r += o.lin.dot(tv(o.b.linvel())) + o.ang.dot(tv(o.b.angvel())); }
    if (W < 1e-12) return 0;
    let lam = -(r - target) / W;
    if (maxImp != null) { const a0 = accum.v; accum.v = Math.max(-maxImp, Math.min(maxImp, a0 + lam)); lam = accum.v - a0; }
    for (const o of list) { if (o.im) o.b.setLinvel(rv(tv(o.b.linvel()).addScaledVector(o.lin, o.im * lam)), true); o.b.setAngvel(rv(tv(o.b.angvel()).addScaledVector(o.Ia, lam)), true); }
    return lam;
  }
  scaleRow(row, s) { return row.map(e => ({ b: e.b, lin: e.lin ? e.lin.clone().multiplyScalar(s) : null, ang: e.ang ? e.ang.clone().multiplyScalar(s) : null })); }
  substep() {
    const h = this.h;
    for (const r of this.bodies) if (r.body.isDynamic()) { r.body.resetForces(true); r.body.resetTorques(true); }
    for (const f of this.forces) {
      const pA = this.toWorld(f.bA, f.l1), pB = this.toWorld(f.bB, f.l2), d = pB.clone().sub(pA), L = d.length(); if (L < 1e-9) continue;
      const u = d.divideScalar(L), vr = tv(f.bB.velocityAtPoint(rv(pB))).sub(tv(f.bA.velocityAtPoint(rv(pA)))).dot(u);
      const F = (f.j.type === 'spring' ? (f.j.k || 0) * (L - f.j.L0) : 0) + (f.j.c || 0) * vr; f.F = F;
      if (f.bA.isDynamic()) f.bA.addForceAtPoint(rv(u.clone().multiplyScalar(F)), rv(pA), true);
      if (f.bB.isDynamic()) f.bB.addForceAtPoint(rv(u.clone().multiplyScalar(-F)), rv(pB), true);
    }
    for (const r of this.drives) this.applyDrive(r);
    const mrows = [];
    for (const m of this.motors) {
      const n = m.nL.clone().applyQuaternion(tq(m.mb.rotation())), mo = m.part.motor || {};
      m.n = n; m.row = [{ b: m.db, ang: n.clone() }, { b: m.mb, ang: n.clone().negate() }]; m.acc = { v: 0 };
      if (mo.mode === 'torque') { const t = (mo.torque || 0) * m.dir; m.tau = t; if (m.db.isDynamic()) m.db.addTorque(rv(n.clone().multiplyScalar(t)), true); if (m.mb.isDynamic()) m.mb.addTorque(rv(n.clone().multiplyScalar(-t)), true); }
      if (m.ij) {
        if (mo.mode === 'speed') { m.ij.configureMotorModel(RAPIER.MotorModel.ForceBased); m.ij.configureMotorVelocity(m.sgn * this.ramp() * (mo.rpm || 0) * Math.PI / 30, 1e4); m.ij.setMotorMaxForce(mo.maxT != null ? mo.maxT : 1e3); }
        else m.ij.setMotorMaxForce(0);
      }
      if (mo.mode === 'speed') mrows.push(m);
    }
    for (const c of this.couplings) { c.row = this.scaleRow(this.jointRow(c.ra), c.k * c.sa).concat(this.scaleRow(this.jointRow(c.rb), -c.sb)); c.acc = { v: 0 }; }
    for (let it = 0; it < 8; it++) {
      for (const c of this.couplings) { if (c.c.type === 'clutch' && !c.c.engaged) continue; this.solveRow(c.row, 0, c.c.type === 'clutch' ? (c.c.maxT || 50) * h : null, c.acc); }
    }
    for (const hx of this.helix) hx.row = this.helixRow(hx);
    for (let it = 0; it < 8; it++) for (const hx of this.helix) this.solveRow(hx.row, 0, null, { v: 0 });
    const pre = mrows.map(m => this.rate(m.row));
    if (!this.eq) this.eq = new RAPIER.EventQueue(false);
    this.world.step(this.eq, this.edit ? undefined : this.hooks);
    // motor torque: power balance of the whole system (dE/dt incl. coupling work) divided by speed, smoothed
    if (mrows.length) { const E = this.energy().total; if (this._E != null) { const P = (E - this._E) / h; for (const m of mrows) { const w = this.rate(m.row); const tq = Math.abs(w) > 1e-3 ? P / mrows.length / w : 0; m.tau = m.tau * 0.98 + 0.02 * Math.max(-(m.part.motor.maxT || 1e3), Math.min(m.part.motor.maxT || 1e3, tq)); } } this._E = E; }
    for (const m of this.motors) { m.w = this.rate(m.row); m.P = m.tau * m.w; }
    void pre;
    for (const m of mrows) { m.acc = { v: 0 }; m.row = [{ b: m.db, ang: m.nL.clone().applyQuaternion(tq(m.mb.rotation())) }, { b: m.mb, ang: m.nL.clone().applyQuaternion(tq(m.mb.rotation())).negate() }]; }
    for (let it = 0; it < 4; it++) for (const m of mrows) this.solveRow(m.row, m.dir * this.ramp() * (m.part.motor.rpm || 0) * Math.PI / 30, (m.part.motor.maxT != null ? m.part.motor.maxT : 1e3) * h, m.acc);
    for (let it = 0; it < 4; it++) for (const c of this.couplings) { if (c.c.type === 'clutch' && !c.c.engaged) continue; c.row = this.scaleRow(this.jointRow(c.ra), c.k * c.sa).concat(this.scaleRow(this.jointRow(c.rb), -c.sb)); this.solveRow(c.row, 0, null, { v: 0 }); }
    for (let it = 0; it < 4; it++) for (const hx of this.helix) { hx.row = this.helixRow(hx); this.solveRow(hx.row, 0, null, { v: 0 }); }
    this.time += h;
  }
  /** soft start for speed-controlled motors (0.3 s), avoids wheelies / impulsive starts */
  ramp() { return Math.min(1, this.time / 0.3 + 0.02); }
  /** helical joint: relative slide along the axis = lead/2π · relative rotation */
  helixRow(hx) { const n = this.jointAxisW(hx.rec), r = hx.rec; return [{ b: r.bA, lin: n.clone().negate(), ang: n.clone().multiplyScalar(hx.k) }, { b: r.bB, lin: n.clone(), ang: n.clone().multiplyScalar(-hx.k) }]; }
  step(dt) { const n = Math.max(1, Math.round(dt / this.h)); for (let i = 0; i < n; i++) this.substep(); }
  /** edit-mode relaxation: steps with zero velocities to close joint gaps */
  relax(n = 10) {
    this.world.timestep = 1 / 60;
    for (let i = 0; i < n; i++) { this.world.step(); for (const r of this.bodies) if (r.body.isDynamic()) { r.body.setLinvel({ x: 0, y: 0, z: 0 }, false); r.body.setAngvel({ x: 0, y: 0, z: 0 }, false); } }
    this.world.timestep = this.h;
  }
  gaps() {
    const out = [];
    for (const r of this.rj) { const a = this.toWorld(r.bA, r.l1), b = this.toWorld(r.bB, r.l2), d = b.sub(a); if (r.j.type === 'prismatic') { const n = this.jointAxisW(r); d.addScaledVector(n, -d.dot(n)); } out.push({ id: r.j.id, type: r.j.type, gap: d.length() }); }
    return out;
  }
  maxGap() { return this.gaps().reduce((m, g) => Math.max(m, g.gap), 0); }
  energy() {
    let K = 0, U = 0, S = 0;
    for (const r of this.bodies) { const b = r.body; if (!b.isDynamic()) continue; const m = b.mass(), v = tv(b.linvel()), w = tv(b.angvel()); K += 0.5 * m * v.lengthSq() + 0.5 * w.dot(w.clone().applyMatrix3(inertiaW(b))); U += m * this.g * b.worldCom().y; }
    for (const f of this.forces) if (f.j.type === 'spring') { const L = this.toWorld(f.bA, f.l1).distanceTo(this.toWorld(f.bB, f.l2)); S += 0.5 * (f.j.k || 0) * (L - f.j.L0) ** 2; }
    return { K, U, S, total: K + U + S };
  }
  bodyInertiaW(b) { return inertiaW(b); }
  free() { try { this.world.free(); } catch (e) { } }
}

/** compute coupling ratio from geometry; returns {k, sa, sb, ja, jb} or error string */
export function planCoupling(M, type, PA, PB, extra = {}) {
  const sim = new Sim(M, { edit: true });
  const kindsA = ['rack', 'nut'].includes(PA.type) ? ['prismatic'] : ['revolute'];
  const kindsB = ['rack', 'nut'].includes(PB.type) ? ['prismatic'] : ['revolute'];
  const ra = sim.findJointFor(PA.id, kindsA), rb = sim.findJointFor(PB.id, kindsB);
  if (!ra || !rb) { sim.free(); return (!ra ? PA.name : PB.name) + ' 需要先用' + (kindsA[0] === 'prismatic' && !ra || kindsB[0] === 'prismatic' && ra ? '滑动副' : '铰链') + '安装（有可转动/滑动的轴）'; }
  const sa = sim.partSide(ra, PA.id), sb = sim.partSide(rb, PB.id);
  const nA = sim.jointAxisW(ra), nB = sim.jointAxisW(rb); // q = ω_part·n for either side
  const cA = V3(PA.p), cB = V3(PB.p); let k = 1;
  const pitchPoint = (c, n, r, toward) => { const d = toward.clone().sub(c); d.addScaledVector(n, -d.dot(n)); if (d.length() < 1e-9) d.set(1, 0, 0).addScaledVector(n, -n.x); return c.clone().addScaledVector(d.normalize(), r); };
  const tang = (n, c, P) => n.clone().cross(P.clone().sub(c));
  const t = type;
  if (t === 'gear') {
    if (PA.type === 'worm' || PB.type === 'worm') {
      const W = PA.type === 'worm' ? PA : PB, G = W === PA ? PB : PA, nW = W === PA ? nA : nB, nG = W === PA ? nB : nA;
      const lead = Math.PI * W.d.m * W.d.starts, P = pitchPoint(V3(G.p), nG, pitchRadius(G), V3(W.p)), tg = tang(nG, V3(G.p), P);
      const kk = -lead / (2 * Math.PI) * tg.dot(nW) / tg.lengthSq();
      k = W === PA ? kk : 1 / kk;
    } else if (PA.type === 'rack' || PB.type === 'rack') {
      const R = PA.type === 'rack' ? PA : PB, G = R === PA ? PB : PA, nR = R === PA ? nA : nB, nG = R === PA ? nB : nA;
      const P = pitchPoint(V3(G.p), nG, pitchRadius(G), V3(R.p)), kk = tang(nG, V3(G.p), P).dot(nR); // v_rack = kk * w
      k = G === PA ? kk : 1 / kk;
    } else {
      const P = pitchPoint(cA, nA, pitchRadius(PA), cB), tA = tang(nA, cA, P), tB = tang(nB, cB, P);
      k = tA.dot(tB) / tB.lengthSq();
    }
  } else if (t === 'chain' && PA.d.z && PB.d.z) { k = PA.d.z / PB.d.z * Math.sign(nA.dot(nB) || 1); }
  else if (t === 'belt' || t === 'chain') { k = pitchRadius(PA) / pitchRadius(PB) * Math.sign(nA.dot(nB) || 1); }
  else if (t === 'screw') {
    const S = PA.type === 'screw' ? PA : PB, N = S === PA ? PB : PA, nS = S === PA ? nA : nB, nN = S === PA ? nB : nA;
    const kk = -S.d.lead / (2 * Math.PI) * nS.dot(nN); k = S === PA ? kk : 1 / kk;
  } else if (t === 'gearbox') { k = (1 / (extra.ratio || 1)) * Math.sign(nA.dot(nB) || 1); }
  else { k = Math.sign(nA.dot(nB) || 1); }
  sim.free();
  return { k, sa, sb, ja: ra.j.id, jb: rb.j.id };
}
