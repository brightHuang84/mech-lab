// Regression: 改变轴的长度，连接的点没有跟着改 — resizing a shaft keeps every connection made with the assembly panel / mate tool:
// parts at an end follow the end, the held side (motor / meshing bevel / frame / ground) stays put, meshes stay engaged, gaps < 0.5 mm,
// nothing below the floor; impossible resizes are refused and restored. Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  await p.evaluate(() => {
    const A = app, V = A.cam.position.constructor, Q = A.cam.quaternion.constructor;
    const meshMin = (P) => { const g = A.DEFS[P.type].mesh(P.d, P); g.position.set(...P.p); g.quaternion.set(...P.q); g.updateMatrixWorld(true); let m = Infinity; const v = new V(); g.traverse(o => { if (o.isMesh && o.geometry && o.geometry.attributes.position) { const pa = o.geometry.attributes.position; for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(o.matrixWorld); if (v.y < m) m = v.y; } } }); return m; };
    A._rz = {
      V, Q,
      clear() { if (A.asm) A.asmCancel(true); if (A.mateS) A.mateCancel(); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select'); if (A.gizmoOn) A.toggleGizmo(); document.getElementById('btnClear').click(); A.msel = new Set(); },
      pair(a, b, patch) { A.msel = new Set([a, b]); A.select({ p: b }, true); A.asmOpen([a, b]); if (patch) A.asmSet(0, patch); return A.asmCommit(); },
      /** axial position of part w's centre on shaft s (m) and its radial offset */
      axial(s, w) { const S = A.part(s), W = A.part(w), ax = new V(0, 0, 1).applyQuaternion(new Q(...S.q)), d = new V(...W.p).sub(new V(...S.p)), t = d.dot(ax); return { t, perp: d.addScaledVector(ax, -t).length(), half: S.d.L / 2 }; },
      /** wheel w sits at an end of shaft s: centre ± half thickness reaches the end (flush) or its centre is on the end (mate) */
      atEnd(s, w, mode) { const a = A._rz.axial(s, w), W = A.part(w), e = mode === 'centre' ? Math.abs(Math.abs(a.t) - a.half) : Math.abs(Math.abs(a.t) + W.d.t / 2 - a.half); return { err: e, perp: a.perp }; },
      rel(a, b) { const Pa = A.part(a), Pb = A.part(b); return new V(...Pb.p).sub(new V(...Pa.p)).applyQuaternion(new Q(...Pa.q).invert()); },
      lo() { let m = Infinity; for (const P of A.M.parts) m = Math.min(m, meshMin(P)); return m; },
      gap() { return A.jointGapAll() * 1000; },
            wpos(id) { return new V(...A.part(id).p); },
      hd(a, b) { return Math.hypot(a.x - b.x, a.z - b.z); }, // horizontal distance (the ground hook may lift the whole assembly)
    };
  });
  const TOL = -0.0005;
  // ---------- 1. panel: motor (fixed) + shaft + bevel + wheel, bevel mesh → second shaft + two wheels ----------
  const R1 = await p.evaluate(() => {
    const A = app, Z = A._rz; Z.clear(); const out = { steps: [] };
    const m = A.placePart('motor', { x: 0, y: 0, z: 0 });
    const s1 = A.placePart('shaft', { x: 0.3, y: 0, z: 0.3 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(m.id, s1.id);
    const bv = A.placePart('bevel', { x: 0.5, y: 0, z: 0.3 }); Z.pair(s1.id, bv.id, { pa: { s: -0.03 } });             // near the motor end
    const w1 = A.placePart('wheel', { x: 0.7, y: 0, z: 0.3 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s1.id, w1.id, { pa: { s: 0.03 } }); // flush with the front end
    const bv2 = A.placePart('bevel', { x: 0.5, y: 0, z: -0.4 }); const rm = Z.pair(bv.id, bv2.id); out.mesh = A.M.couplings.length;
    const s2 = A.placePart('shaft', { x: 0.3, y: 0, z: -0.5 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(bv2.id, s2.id, { pb: { s: 0 } });
    const w2 = A.placePart('wheel', { x: 0.7, y: 0, z: -0.5 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s2.id, w2.id, { pa: { s: 0.03 } });
    const w3 = A.placePart('wheel', { x: 0.7, y: 0, z: -0.8 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s2.id, w3.id, { pa: { s: -0.03 } });
    out.refs = A.M.joints.filter(j => j.ref && (j.ref.a || j.ref.b)).length + '/' + A.M.joints.length;
    out.refShaft = (A.M.joints.find(j => (j.a === s1.id && j.b === w1.id) || (j.b === s1.id && j.a === w1.id)) || {}).ref;
    const rel0 = Z.rel(bv.id, bv2.id), m0 = Z.wpos(m.id), b0 = Z.wpos(bv.id);
    out.build = { gap: Z.gap(), w1: Z.atEnd(s1.id, w1.id).err, w2: Z.atEnd(s2.id, w2.id).err, w3: Z.atEnd(s2.id, w3.id).err, lo: Z.lo() };
    for (const [sid, L, ws] of [[s1.id, 0.2, [w1.id]], [s1.id, 0.06, [w1.id]], [s2.id, 0.2, [w2.id, w3.id]], [s2.id, 0.06, [w2.id, w3.id]], [s1.id, 0.1, [w1.id]], [s2.id, 0.1, [w2.id, w3.id]]]) {
      const r = A.resizePart(sid, { L });
      out.steps.push({ s: A.part(sid).name, L, ok: r.ok, strat: r.strategy, gap: +Z.gap().toFixed(4), ends: ws.map(w => +(Z.atEnd(sid, w).err * 1000).toFixed(3)), perp: Math.max(...ws.map(w => Z.atEnd(sid, w).perp)) * 1000, mesh: +(Z.rel(bv.id, bv2.id).distanceTo(rel0) * 1000).toFixed(3), motor: +(Z.hd(Z.wpos(m.id), m0) * 1000).toFixed(3), bevel: +(Z.hd(Z.wpos(bv.id), b0) * 1000).toFixed(3), lo: Z.lo(), Lnow: A.part(sid).d.L });
    }
    out.ids = { m: m.id, s1: s1.id, s2: s2.id, w1: w1.id, bv: bv.id };
    return out;
  });
  const okStep = s => s.ok && s.gap < 0.5 && s.ends.every(e => e < 1) && s.perp < 0.5 && s.mesh < 0.5 && s.lo >= -0.0005;
  check('装配面板搭建（电机+轴+锥齿轮+轮，锥齿轮啮合→第二根轴+两个轮）：每个连接都记录了参数化连接点（连接点名+沿轴位置）', R1.mesh === 1 && R1.refs.split('/')[0] === R1.refs.split('/')[1] && R1.refShaft && R1.refShaft.a && R1.refShaft.a.k === 'line' && R1.build.gap < 0.5 && R1.build.w1 < 1e-3 && R1.build.w2 < 1e-3 && R1.build.w3 < 1e-3, JSON.stringify({ refs: R1.refs, ref: R1.refShaft, build: R1.build }));
  check('改轴长 100→200→60 mm（电机轴）：轮始终在轴端、锥齿轮与电机不动、啮合保持、间隙 < 0.5 mm、不入地', R1.steps.slice(0, 2).every(s => okStep(s) && s.motor < 0.5 && s.bevel < 0.5), JSON.stringify(R1.steps.slice(0, 2)));
  check('改轴长 100→200→60 mm（锥齿轮带动的第二根轴，两端各一个轮）：两轮都在轴端、啮合保持、间隙 < 0.5 mm', R1.steps.slice(2, 4).every(s => okStep(s)), JSON.stringify(R1.steps.slice(2, 4)));
  check('再改回 100 mm：两根轴上的轮仍在轴端，电机/锥齿轮位置不变', R1.steps.slice(4).every(s => okStep(s)) && R1.steps[4].motor < 0.5, JSON.stringify(R1.steps.slice(4)));
  // ---------- 2. impossible: bevel would fall off the motor shaft → refused, model restored ----------
  const R2 = await p.evaluate(ids => {
    const A = app, Z = A._rz, s1 = ids.s1; A.resizePart(s1, { L: 0.1 }); const before = A.snapshot(); const r = A.resizePart(s1, { L: 0.015 });
    return { ok: r.ok, msg: r.msg, same: A.snapshot() === before, gap: Z.gap() };
  }, R1.ids);
  check('改得过短（锥齿轮会脱离轴/啮合）→ 拒绝并恢复原尺寸，给出中文说明', !R2.ok && R2.same && /无法修改/.test(R2.msg || '') && R2.gap < 0.5, JSON.stringify(R2));
  // ---------- 1b. the bevel sits near the FREE end of the motor shaft (the old "ends follow" rule dragged it out of mesh) ----------
  const R1b = await p.evaluate(() => {
    const A = app, Z = A._rz; Z.clear();
    const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), s1 = A.placePart('shaft', { x: 0.3, y: 0, z: 0.3 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(m.id, s1.id);
    const bv = A.placePart('bevel', { x: 0.5, y: 0, z: 0.3 }); Z.pair(s1.id, bv.id, { pa: { s: 0.025 } });
    const bv2 = A.placePart('bevel', { x: 0.5, y: 0, z: -0.4 }); Z.pair(bv.id, bv2.id);
    const s2 = A.placePart('shaft', { x: 0.3, y: 0, z: -0.5 }, { d: { L: 0.12, r: 0.006 } }); Z.pair(bv2.id, s2.id, { pb: { s: 0 } });
    const rel0 = Z.rel(bv.id, bv2.id), b0 = Z.wpos(bv.id), out = [];
    for (const L of [0.2, 0.08, 0.1]) { const r = A.resizePart(s1.id, { L }); out.push({ L, ok: r.ok, strat: r.strategy, gap: +Z.gap().toFixed(4), mesh: +(Z.rel(bv.id, bv2.id).distanceTo(rel0) * 1000).toFixed(3), bevel: +(Z.hd(Z.wpos(bv.id), b0) * 1000).toFixed(3), lo: Z.lo() }); }
    return out;
  });
  check('锥齿轮在电机轴的自由端附近 → 改轴长 100→200→80→100 mm：锥齿轮留在原处、啮合不脱开（轴向另一端伸缩）', R1b.every(s => s.ok && s.gap < 0.5 && s.mesh < 0.5 && s.bevel < 0.5 && s.lo >= -0.0005), JSON.stringify(R1b));
  // ---------- 3. mate tool: motor + shaft (mated to the output) + wheel mated onto the shaft front end ----------
  const R3 = await p.evaluate(() => {
    const A = app, Z = A._rz, V = Z.V, Q = Z.Q; Z.clear(); const out = { steps: [] };
    const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), s = A.placePart('shaft', { x: 0.3, y: 0, z: 0.3 }, { d: { L: 0.1, r: 0.006 } }), w = A.placePart('wheel', { x: 0.6, y: 0, z: 0.3 }, { d: { r: 0.07, t: 0.03 } });
    const W = (P, l) => new V(...l).applyQuaternion(new Q(...P.q)).add(new V(...P.p)).toArray(), N = (P, a) => new V(...a).applyQuaternion(new Q(...P.q)).toArray();
    let M = A.part(m.id), S = A.part(s.id);
    // mate refs are local (point + normal on each part), as picked with the 🧲 tool
    A.mate({ pid: s.id, p: [0, 0, -S.d.L / 2], n: [0, 0, -1] }, { pid: m.id, p: A.DEFS.motor.pts(M.d)[0].p, n: [0, 0, 1] }, { type: 'revolute' });
    S = A.part(s.id);
    A.mate({ pid: w.id, p: [0, 0, 0], n: [0, 0, -1] }, { pid: s.id, p: [0, 0, S.d.L / 2], n: [0, 0, 1] }, { type: 'fixed' });
    out.joints = A.M.joints.length; out.ref = (A.M.joints.find(j => j.a === s.id || j.b === s.id) || {}).ref; out.gap0 = Z.gap(); out.end0 = Z.atEnd(s.id, w.id, 'centre').err;
    const m0 = Z.wpos(m.id);
    for (const L of [0.2, 0.06, 0.12]) { const r = A.resizePart(s.id, { L }); const e = Z.atEnd(s.id, w.id, 'centre'); out.steps.push({ L, ok: r.ok, strat: r.strategy, gap: +Z.gap().toFixed(4), end: +(e.err * 1000).toFixed(3), perp: e.perp * 1000, motor: +(Z.wpos(m.id).distanceTo(m0) * 1000).toFixed(3), lo: Z.lo() }); }
    // the motor-side end stays in the motor: the shaft's back end is still at the motor output
    const S2 = A.part(s.id), M2 = A.part(m.id), back = new V(...W(S2, [0, 0, -S2.d.L / 2])), out0 = new V(...W(M2, A.DEFS.motor.pts(M2.d)[0].p)); out.backErr = back.distanceTo(out0) * 1000;
    return out;
  });
  check('🧲 配合搭建（轴配到电机输出轴、轮配到轴前端）→ 改轴长 100→200→60→120 mm：轮跟着轴端、轴后端仍在电机输出端、间隙 < 0.5 mm', R3.joints === 2 && R3.gap0 < 0.5 && R3.end0 < 1e-3 && R3.steps.every(s => s.ok && s.gap < 0.5 && s.end < 1 && s.perp < 0.5 && s.motor < 0.5 && s.lo >= -0.0005) && R3.backErr < 1, JSON.stringify(R3));
  // ---------- 4. motor mounted on a free plate (panel): the plate + motor side stays, the free end with the wheel moves ----------
  const R4 = await p.evaluate(() => {
    const A = app, Z = A._rz; Z.clear(); const out = { steps: [] };
    const pl = A.placePart('plate', { x: 0, y: 0, z: 0 }, { d: { w: 0.4, h: 0.02, dp: 0.3, np: 3 } }), m = A.placePart('motor', { x: 0.5, y: 0, z: 0 }); Z.pair(pl.id, m.id); out.motorFree = !A.part(m.id).fixed;
    const s = A.placePart('shaft', { x: 0.5, y: 0, z: 0.5 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(m.id, s.id);
    const w = A.placePart('wheel', { x: 0.8, y: 0, z: 0.5 }, { d: { r: 0.09, t: 0.04 } }); Z.pair(s.id, w.id, { pa: { s: 0.03 } });
    const p0 = Z.wpos(pl.id), m0 = Z.wpos(m.id);
    for (const L of [0.2, 0.06, 0.1]) { const r = A.resizePart(s.id, { L }); out.steps.push({ L, ok: r.ok, strat: r.strategy, gap: +Z.gap().toFixed(4), end: +(Z.atEnd(s.id, w.id).err * 1000).toFixed(3), plate: +(Z.wpos(pl.id).distanceTo(p0) * 1000).toFixed(3), motor: +(Z.wpos(m.id).distanceTo(m0) * 1000).toFixed(3), lo: Z.lo() }); }
    return out;
  });
  check('电机装在板上（可移动的装配体）→ 改轴长 100→200→60→100 mm：板和电机不动、轮在轴端、间隙 < 0.5 mm、不入地', R4.motorFree && R4.steps.every(s => s.ok && s.gap < 0.5 && s.end < 1 && s.plate < 0.5 && s.motor < 0.5 && s.lo >= -0.0005), JSON.stringify(R4));
  // ---------- 5. live preview (dimension field dragging) uses the same rules ----------
  const R5 = await p.evaluate(ids => {
    const A = app, Z = A._rz; Z.clear();
    const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), s = A.placePart('shaft', { x: 0.3, y: 0, z: 0.3 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(m.id, s.id);
    const w = A.placePart('wheel', { x: 0.7, y: 0, z: 0.3 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s.id, w.id, { pa: { s: 0.03 } });
    const out = []; for (const L of [0.15, 0.25, 0.07]) { const r = A.resizeLive(s.id, { L }); out.push({ ok: r.ok, gap: +Z.gap().toFixed(4), end: +(Z.atEnd(s.id, w.id).err * 1000).toFixed(3) }); } A.resizeCommit(s.id, { L: 0.07 });
    out.push({ commit: true, gap: +Z.gap().toFixed(4), end: +(Z.atEnd(s.id, w.id).err * 1000).toFixed(3), L: A.part(s.id).d.L });
    return out;
  }, R1.ids);
  check('尺寸实时预览（拖动/输入中）与确定：轮始终跟随轴端', R5.slice(0, 3).every(s => s.ok && s.gap < 0.5 && s.end < 1) && R5[3].gap < 0.5 && R5[3].end < 1 && Math.abs(R5[3].L - 0.07) < 1e-9, JSON.stringify(R5));
  // ---------- screenshot: the car-like build after lengthening both shafts ----------
  await p.evaluate(() => {
    const A = app, Z = A._rz; Z.clear();
    const pl = A.placePart('plate', { x: 0, y: 0, z: 0 }, { d: { w: 0.4, h: 0.02, dp: 0.3, np: 3 } });
    const m = A.placePart('motor', { x: 0.5, y: 0, z: 0 }); Z.pair(pl.id, m.id); A.part(m.id).fixed = true;
    const s1 = A.placePart('shaft', { x: 0.3, y: 0, z: 0.3 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(m.id, s1.id);
    const bv = A.placePart('bevel', { x: 0.5, y: 0, z: 0.3 }); Z.pair(s1.id, bv.id, { pa: { s: -0.03 } });
    const w1 = A.placePart('wheel', { x: 0.7, y: 0, z: 0.3 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s1.id, w1.id, { pa: { s: 0.03 } });
    const bv2 = A.placePart('bevel', { x: 0.5, y: 0, z: -0.4 }); Z.pair(bv.id, bv2.id);
    const s2 = A.placePart('shaft', { x: 0.3, y: 0, z: -0.5 }, { d: { L: 0.1, r: 0.006 } }); Z.pair(bv2.id, s2.id, { pb: { s: 0 } });
    const w2 = A.placePart('wheel', { x: 0.7, y: 0, z: -0.5 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s2.id, w2.id, { pa: { s: 0.03 } });
    const w3 = A.placePart('wheel', { x: 0.7, y: 0, z: -0.8 }, { d: { r: 0.06, t: 0.04 } }); Z.pair(s2.id, w3.id, { pa: { s: -0.03 } });
    A.resizePart(s1.id, { L: 0.2 }); A.resizePart(s2.id, { L: 0.24 });
    A.select({ p: s2.id }); A.flash = null; const t = document.getElementById('gToast'); if (t) t.style.opacity = '0';
    A.view('iso'); A.fit(); A.msg('改轴长后：轴1 100→200 mm、轴2 100→240 mm，轮仍在轴端，锥齿轮啮合与电机位置保持，最大连接间隙 ' + Z.gap().toFixed(3) + ' mm');
  });
  await new Promise(r => setTimeout(r, 400)); await frame();
  await p.screenshot({ path: OUT + 'screenshot-resize-fix.png' }); console.log('saved screenshot-resize-fix.png');
  await p.evaluate(() => app._rz.clear());
};
