// Regression: 锥齿轮啮合后第二根轴歪了 ~3.5° — meshing two bevel gears where gear B already sits on a second shaft that carries a
// wheel (B's whole assembly moves with it). Through the 装配 panel, the 🧲 mate tool (B's assembly built with mate) and drag-snap:
// B's axis exactly ⟂ A's axis and lying exactly in the stable frame (no residual tilt), pitch cones share the apex, teeth interleaved,
// and B's shaft + wheel follow rigidly (same relative poses). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  await p.evaluate(() => {
    const A = app, V = A.cam.position.constructor, Q = A.cam.quaternion.constructor;
    const ax = P => new V(0, 0, 1).applyQuaternion(new Q(...P.q).normalize());
    const wrap = a => { a = (a + Math.PI) % (2 * Math.PI); if (a < 0) a += 2 * Math.PI; return a - Math.PI; };
    A._bp = {
      V, Q, ax,
      clear() { if (A.asm) A.asmCancel(true); if (A.mateS) A.mateCancel(); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select'); if (A.gizmoOn) A.toggleGizmo(); document.getElementById('btnClear').click(); A.msel = new Set(); },
      pair(a, b, patch) { A.msel = new Set([a, b]); A.select({ p: b }, true); A.asmOpen([a, b]); if (patch) A.asmSet(0, patch); return A.asmCommit(); },
      pl(t, x, z) { return A.placePart(t, { x, y: 0, z }).id; },
      /** gear A: motor (fixed) + shaft 1 + bevel 1, all through the panel */
      sideA() { const G = A._bp, m = G.pl('motor', 0, 0), s1 = G.pl('shaft', 0.3, 0), g1 = G.pl('bevel', 0.5, 0.2); G.pair(s1, m); G.pair(s1, g1); return { m, s1, g1 }; },
      /** rotate a set of parts rigidly (about the first part's centre) — e.g. a 3.5° residual tilt */
      tilt(ids, axis, deg) { A.beginEdit(); const R = new Q().setFromAxisAngle(new V(...axis).normalize(), deg * Math.PI / 180), c = new V(...A.part(ids[0]).p);
        for (const id of ids) { const P = A.part(id); P.p = new V(...P.p).sub(c).applyQuaternion(R).add(c).toArray(); P.q = R.clone().multiply(new Q(...P.q)).normalize().toArray(); } A.commit(); A.rebuild(); },
      rel(ref, ids) { const R = A.part(ref), qi = new Q(...R.q).normalize().invert(); return ids.map(id => { const P = A.part(id); return { p: new V(...P.p).sub(new V(...R.p)).applyQuaternion(qi), q: qi.clone().multiply(new Q(...P.q).normalize()) }; }); },
      /** all metrics of the bevel pair H (stays) / G (moved), G's group grp (shaft, wheel ...) with relative poses rel0 before */
      metrics(h, g, grp, rel0, s2) {
        const H = A.part(h), Gp = A.part(g), nH = ax(H), nG = ax(Gp), S2 = A.part(s2);
        const apex = P => new V(...P.p).addScaledVector(ax(P), -P.d.A);
        const loc = (P, w) => w.clone().applyQuaternion(new Q(...P.q).normalize().invert()), uH = loc(H, nG), uG = loc(Gp, nH);
        const phase = wrap(H.d.z * Math.atan2(uH.y, uH.x) + Gp.d.z * Math.atan2(uG.y, uG.x) - Math.PI) / (2 * Math.PI); // fraction of a tooth period
        const rel1 = A._bp.rel(g, grp);
        const rigid = Math.max(...rel0.map((r, i) => Math.max(r.p.distanceTo(rel1[i].p), 1 - Math.abs(r.q.dot(rel1[i].q)))));
        const sAx = ax(S2), wheelAxes = grp.map(id => Math.abs(Math.abs(ax(A.part(id)).dot(nG)) - 1));
        // residual tilt: B's axis must sit exactly on the snapped azimuth grid of the pitch plane (horizontal frame for a horizontal A)
        let e1 = nH.clone().cross(new V(0, 1, 0)); if (e1.length() < 0.3) e1 = new V(1, 0, 0).addScaledVector(nH, -nH.x); e1.normalize(); const e2 = nH.clone().cross(e1);
        const az = Math.atan2(nG.dot(e2), nG.dot(e1)) * 180 / Math.PI, azRes = Math.abs(az - Math.round(az / 15) * 15);
        return { dot: nH.dot(nG), shaftDot: nH.dot(sAx), sCoax: 1 - Math.abs(sAx.dot(nG)), nG: nG.toArray().map(x => +x.toFixed(6)), az: +az.toFixed(4), azRes, apex: apex(H).distanceTo(apex(Gp)),
          phase, rigid, wheelAx: Math.max(...wheelAxes), gap: A.editGap() * 1000, coupled: A.M.couplings.some(c => [c.pa, c.pb].includes(h) && [c.pa, c.pb].includes(g)) };
      },
    };
  });
  const R = {};
  // ---------- 1. panel: B (bevel 2) on shaft 2 with a wheel, B's assembly tilted 3.5° (the reported residual) ----------
  R.panel = await p.evaluate(() => {
    const A = app, G = A._bp; G.clear(); const a = G.sideA();
    const s2 = G.pl('shaft', -0.5, 0.5), g2 = G.pl('bevel', -0.3, 0.6), w = G.pl('wheel', -0.7, 0.6);
    G.pair(s2, g2, { pa: { s: 0.03 } }); G.pair(s2, w, { pa: { s: -0.04 } }); G.tilt([s2, g2, w], [0.3, 0.2, 1], 3.5);
    const grp = [s2, w], rel0 = G.rel(g2, grp); const r = G.pair(a.g1, g2);
    const out = G.metrics(a.g1, g2, grp, rel0, s2); out.errs = r.errs; out.changed = r.changed;
    // azimuth 30°: still exactly ⟂, exactly 30° on the pitch-plane grid
    A.undo(); const r2 = G.pair(a.g1, g2, { ang: 30 }); out.ang30 = G.metrics(a.g1, g2, grp, rel0, s2); out.ang30.changed = r2.changed;
    return out;
  });
  check('锥齿轮·装配面板：B 在带轮子的轴2 上（装配体歪 3.5°）→ 啮合后 B 轴与 A 轴严格垂直（|dot|<1e-4，无残余倾斜）、锥顶重合、齿相错、轴2+轮整体刚性随动；方位角 30° 同样严格', R.panel.changed && !R.panel.errs.length && Math.abs(R.panel.dot) < 1e-4 && Math.abs(R.panel.shaftDot) < 1e-4 && R.panel.sCoax < 1e-9 && R.panel.azRes < 1e-3 && R.panel.apex < 1e-6 && Math.abs(R.panel.phase) < 1e-6 && R.panel.rigid < 1e-9 && R.panel.wheelAx < 1e-9 && R.panel.gap < 0.5 && R.panel.coupled
    && R.panel.ang30.changed && Math.abs(R.panel.ang30.dot) < 1e-4 && Math.abs(Math.abs(R.panel.ang30.az - R.panel.az) - 30) < 1e-3 && R.panel.ang30.apex < 1e-6 && Math.abs(R.panel.ang30.phase) < 1e-6 && R.panel.ang30.rigid < 1e-9, JSON.stringify(R.panel));
  // ---------- 2. mate tool: B's assembly (shaft 2 + bevel 2 + wheel) built with 🧲 配合, tilted, then meshed in the panel ----------
  R.mate = await p.evaluate(() => {
    const A = app, G = A._bp; G.clear(); const a = G.sideA();
    const s2 = G.pl('shaft', -0.5, 0.5), g2 = G.pl('bevel', -0.3, 0.6), w = G.pl('wheel', -0.7, 0.6), L = A.part(s2).d.L;
    const j1 = A.mate({ pid: g2, p: [0, 0, 0], n: [0, 0, -1], name: '中心孔' }, { pid: s2, p: [0, 0, L / 2], n: [0, 0, 1], name: '前端' }, { type: 'fixed' });
    const j2 = A.mate({ pid: w, p: [0, 0, 0], n: [0, 0, 1], name: '中心孔' }, { pid: s2, p: [0, 0, -L / 2], n: [0, 0, -1], name: '后端' }, { type: 'fixed' });
    G.tilt([s2, g2, w], [1, 0.4, -0.2], -3.5);
    const grp = [s2, w], rel0 = G.rel(g2, grp); const r = G.pair(a.g1, g2);
    const out = G.metrics(a.g1, g2, grp, rel0, s2); out.mated = !!(j1 && j2); out.errs = r.errs; out.changed = r.changed;
    // and a mate on the already meshed assembly (another wheel onto shaft 2's free end... here: the gear side) keeps the pair exact
    const w2 = G.pl('wheel', 0.8, -0.6), rel1 = G.rel(g2, grp); const j3 = A.mate({ pid: w2, p: [0, 0, 0], n: [0, 0, 1], name: '中心孔' }, { pid: s2, p: [0, 0, 0], n: [0, 0, 1], name: '中心' }, { type: 'fixed' });
    out.after = G.metrics(a.g1, g2, grp, rel1, s2); out.after.mated = !!j3; out.after.w2Ax = 1 - Math.abs(G.ax(A.part(w2)).dot(G.ax(A.part(g2))));
    return out;
  });
  check('锥齿轮·🧲 配合：轴2+锥齿轮2+轮 用配合工具装成（再歪 3.5°）→ 面板啮合后严格垂直、锥顶重合、齿相错、整体刚性；之后再配合一个轮到轴2 上，啮合仍严格', R.mate.mated && R.mate.changed && !R.mate.errs.length && Math.abs(R.mate.dot) < 1e-4 && Math.abs(R.mate.shaftDot) < 1e-4 && R.mate.sCoax < 1e-9 && R.mate.azRes < 1e-3 && R.mate.apex < 1e-6 && Math.abs(R.mate.phase) < 1e-6 && R.mate.rigid < 1e-9 && R.mate.wheelAx < 1e-9 && R.mate.gap < 0.5 && R.mate.coupled
    && R.mate.after.mated && Math.abs(R.mate.after.dot) < 1e-4 && R.mate.after.apex < 1e-6 && Math.abs(R.mate.after.phase) < 1e-6 && R.mate.after.rigid < 1e-9 && R.mate.after.w2Ax < 1e-9 && R.mate.after.gap < 0.5, JSON.stringify(R.mate));
  // ---------- 3. drag-snap: B's assembly dragged (rigid) next to bevel 1 with a 3.5° tilt, released → snap mesh (findAssembly + applyAssembly) ----------
  R.snap = await p.evaluate(() => {
    const A = app, G = A._bp, V = G.V, Q = G.Q; G.clear(); const a = G.sideA();
    const s2 = G.pl('shaft', -0.5, 0.5), g2 = G.pl('bevel', -0.3, 0.6), w = G.pl('wheel', -0.7, 0.6);
    G.pair(s2, g2, { pa: { s: 0.03 } }); G.pair(s2, w, { pa: { s: -0.04 } });
    const grp = [s2, w], rel0 = G.rel(g2, grp), ids = [g2, s2, w];
    A.view('iso'); A.fit();
    // "drag": move the group rigidly so bevel 2 sits ~3 mm off the meshing spot, axis roughly along -X but tilted 3.5°
    const H = A.part(a.g1), nH = G.ax(H), rG = H.d.m * H.d.z / 2, O = new V(...H.p).addScaledVector(nH, -rG), rH = A.part(g2).d.m * A.part(g2).d.z / 2;
    const tgt = O.clone().add(new V(-rH - 0.003, 0.004, 0)), q0 = new Q(...A.part(g2).q).normalize(), want = new Q().setFromUnitVectors(new V(0, 0, 1), new V(-Math.cos(0.061), -Math.sin(0.061), 0)).multiply(q0);
    A.beginEdit(); const dq = want.clone().multiply(q0.clone().invert()), c0 = new V(...A.part(g2).p);
    for (const id of ids) { const P = A.part(id); P.p = new V(...P.p).sub(c0).applyQuaternion(dq).add(tgt).toArray(); P.q = dq.clone().multiply(new Q(...P.q)).normalize().toArray(); }
    const P = A.part(g2), r = A._fa({ M: A.M, mover: P, p0: new V(...P.p), q0: new Q(...P.q), ray: null, cursor: null, exclude: new Set(ids), toScreen: v => A.toScreen(v), camPos: A.cam.position });
    const gc = r.cands.filter(c => c.kind === 'gear' && c.target.id === a.g1).sort((x, y) => x.dist - y.dist)[0];
    const out = { found: !!gc, best: r.best ? r.best.kind : null, px: gc ? +gc.dist.toFixed(1) : null };
    if (gc) { const res = A.applyAssembly(gc, P, ids); if (res) A.liftComp(g2); A.commit(); A.rebuild(); out.res = !!res; }
    Object.assign(out, G.metrics(a.g1, g2, grp, rel0, s2)); return out;
  });
  check('锥齿轮·拖动吸附：带轴2+轮的锥齿轮2 歪 3.5° 拖到锥齿轮1 旁松开 → 吸附啮合后严格垂直、锥顶重合、齿相错、整体刚性随动', R.snap.found && R.snap.res && R.snap.px <= 42 && Math.abs(R.snap.dot) < 1e-4 && Math.abs(R.snap.shaftDot) < 1e-4 && R.snap.sCoax < 1e-9 && R.snap.azRes < 1e-3 && R.snap.apex < 1e-6 && Math.abs(R.snap.phase) < 1e-6 && R.snap.rigid < 1e-9 && R.snap.wheelAx < 1e-9 && R.snap.gap < 0.5 && R.snap.coupled, JSON.stringify(R.snap));
};
