// Regression: moving assemblies in edit mode (装配体没法移动) + panel/mate connection hygiene (这个配合也不对). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const settle = async () => { for (let i = 0; i < 4; i++) await frame(); };
  const drag = async (xy, dx, dy, mods = []) => { for (const m of mods) await p.keyboard.down(m); await p.mouse.move(xy.x, xy.y); await p.mouse.down(); for (let i = 1; i <= 10; i++) await p.mouse.move(xy.x + dx * i / 10, xy.y + dy * i / 10); await p.mouse.up(); for (const m of mods) await p.keyboard.up(m); await settle(); };
  await p.evaluate(() => {
    app._mv = {
      // the user's scene: 电机(固定) + 轴 + 锥齿轮 ×2 + 板, assembled with the 装配 panel; the plate is additionally 固定 to the ground (world joint)
      build() {
        const A = app; if (A.asm) A.asmCancel(true); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select'); if (A.gizmoOn) A.toggleGizmo();
        document.getElementById('btnClear').click();
        const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), s = A.placePart('shaft', { x: 0.3, y: 0, z: 0 }), g1 = A.placePart('bevel', { x: 0.5, y: 0, z: 0.2 }), pl = A.placePart('plate', { x: -0.4, y: 0, z: 0.3 }), g2 = A.placePart('bevel', { x: 0.2, y: 0, z: 0.4 });
        const ids = [m, s, g1, pl, g2].map(x => x.id); A.msel = new Set(ids); A.select({ p: ids[4] }, true); A.asmOpen(ids); A.asmCommit();
        A.beginEdit(); const P = A.part(pl.id); A.addJoint('fixed', P, null, new A._V(...P.p), [0, 1, 0]); A.commit(); A.rebuild();
        A.select(null); A.view('iso'); A.fit(); return ids;
      },
      poses(ids) { return ids.map(id => ({ p: app.part(id).p.slice(), q: app.part(id).q.slice() })); },
      deltas(a, b) { return a.map((x, i) => [0, 1, 2].map(k => b[i].p[k] - x.p[k])); },
      spread(d) { return Math.max(...d.flatMap(v => v.map((x, k) => Math.abs(x - d[0][k])))); },
      qdiff(a, b) { return Math.max(...a.map((x, i) => 1 - Math.abs(x.q.reduce((s, v, k) => s + v * b[i].q[k], 0)))); },
      worldOk() { const A = app; let e = 0; for (const j of A.M.joints) if (j.b == null && j.type === 'fixed') { const P = A.part(j.a), w = new A._V(...j.la).applyQuaternion(new A.cam.quaternion.constructor(...P.q)).add(new A._V(...P.p)); e = Math.max(e, w.distanceTo(new A._V(...j.lb))); } return e; },
      screen(pid) { const A = app, P = A.part(pid), s = A.toScreen(new A._V(...P.p)); for (let r = 0; r <= 90; r += 3) for (let a = 0; a < 6.28; a += 0.3) { const x = s.x + Math.cos(a) * r, y = s.y + Math.sin(a) * r, el = document.elementFromPoint(x, y); if (el && el.id === 'cv' && A.pickAt({ clientX: x, clientY: y }).pid === pid) return { x, y }; } return null; },
    };
  });
  const R = {};
  // ---------- 1. group-header selection, plate fixed to ground: left-drag on a part moves all 5 rigidly ----------
  let ids = await p.evaluate(() => app._mv.build()); await settle();
  const hdr = await p.evaluateHandle(() => [...document.querySelectorAll('#olList .olg')].find(h => /装配体/.test(h.textContent)));
  await hdr.asElement().click({ offset: { x: 60, y: 8 } }); await settle();
  R.sel = await p.evaluate(() => ({ n: app.msel.size, title: document.querySelector('#props .title').textContent, pos: document.querySelectorAll('#props .mPos').length, rot: !!document.getElementById('mRot') }));
  let pt = await p.evaluate(id => app._mv.screen(id), ids[0]); let b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids), snap: app.snapshot(), u: app.undoS.length }), ids);
  await drag(pt, 120, 50);
  R.drag1 = await p.evaluate((ids, b0) => { const P = app._mv.poses(ids), d = app._mv.deltas(b0.P, P); return { moved: Math.hypot(...d[0]), spread: app._mv.spread(d), dq: app._mv.qdiff(b0.P, P), dy: Math.max(...d.map(v => Math.abs(v[1]))), gap: app.editGap() * 1000, world: app._mv.worldOk(), joints: app.M.joints.length, undoSteps: app.undoS.length - b0.u, sel: app.msel.size }; }, ids, b0);
  R.undo1 = await p.evaluate(s => { app.undo(); return app.snapshot() === s; }, b0.snap);
  check('装配体移动：点组件栏“装配体 1”标题选中 5 个零件 → 右侧多选面板含 中心位置 X/Y/Z + 旋转', R.sel.n === 5 && /多选（5/.test(R.sel.title) && R.sel.pos === 3 && R.sel.rot, JSON.stringify(R.sel));
  check('装配体移动：左键拖动（板已固定到大地）→ 5 个零件同样平移、沿地面、连接有效、接地锚点跟随、一步撤销', R.drag1.moved > 0.02 && R.drag1.spread < 1e-9 && R.drag1.dq < 1e-12 && R.drag1.dy < 1e-9 && R.drag1.gap < 0.5 && R.drag1.world < 1e-9 && R.drag1.undoSteps === 1 && R.undo1, JSON.stringify(R.drag1) + ' undo=' + R.undo1);
  // ---------- 2. nothing selected: drag a single connected part → whole assembly ----------
  ids = await p.evaluate(() => app._mv.build()); await settle();
  pt = await p.evaluate(id => app._mv.screen(id), ids[4]); b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids) }), ids);
  await drag(pt, -100, 40);
  R.drag2 = await p.evaluate((ids, b0) => { const P = app._mv.poses(ids), d = app._mv.deltas(b0.P, P); return { moved: Math.hypot(...d[0]), spread: app._mv.spread(d), gap: app.editGap() * 1000, world: app._mv.worldOk(), joints: app.M.joints.length }; }, ids, b0);
  check('单个零件左键拖动（未选中，属于装配体）→ 整个装配体一起移动', R.drag2.moved > 0.02 && R.drag2.spread < 1e-9 && R.drag2.gap < 0.5 && R.drag2.world < 1e-9, JSON.stringify(R.drag2));
  // ---------- 3. Alt+drag: only that part (detached) ----------
  const nj0 = await p.evaluate(() => app.M.joints.length + app.M.couplings.length);
  pt = await p.evaluate(id => app._mv.screen(id), ids[4]); b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids) }), ids);
  await drag(pt, 90, 30, ['Alt']);
  R.alt = await p.evaluate((ids, b0, nj0) => { const P = app._mv.poses(ids), d = app._mv.deltas(b0.P, P); const own = app.M.joints.filter(j => j.a === ids[4] || j.b === ids[4]).length + app.M.couplings.filter(c => c.pa === ids[4] || c.pb === ids[4]).length;
    return { moved: Math.hypot(...d[4]), others: Math.max(...d.slice(0, 4).map(v => Math.hypot(...v))), own, removed: nj0 - app.M.joints.length - app.M.couplings.length, undo: (() => { const n = app.undoS.length; app.undo(); const back = app.M.joints.length + app.M.couplings.length; app.redo(); return back === nj0 && n > 0; })() }; }, ids, b0, nj0);
  check('Alt+左键拖动 → 只移动（并拆开）这个零件，其它零件不动，一步撤销恢复连接', R.alt.moved > 0.02 && R.alt.others < 1e-12 && R.alt.own === 0 && R.alt.removed > 0 && R.alt.undo, JSON.stringify(R.alt));
  // ---------- 4. gizmo on a multi-selection ----------
  ids = await p.evaluate(() => app._mv.build()); await settle();
  await p.evaluate(ids => { app.msel = new Set(ids); app.select({ p: ids[0] }, true); }, ids); await settle();
  await p.keyboard.press('KeyG'); await settle();
  R.giz = await p.evaluate(ids => { const A = app, c = A.selCenter(A.moveSet(ids[0], { multi: true })); return { on: A.gizmoOn, att: A.tc.object === A.proxy, atCenter: A.proxy.position.distanceTo(c), q: A.proxy.quaternion.w }; }, ids);
  R.giz2 = await p.evaluate(ids => { const A = app, m = A._mv, b = m.poses(ids); A.gizmoDragSel([0.1, 0, -0.05]); const a = m.poses(ids), d = m.deltas(b, a);
    const rel0 = b.map(x => [0, 1, 2].map(k => x.p[k] - b[0].p[k])), b2 = m.poses(ids); const Q = A.cam.quaternion.constructor, V = A._V; A.gizmoDragSel(null, new Q().setFromAxisAngle(new V(0, 1, 0), Math.PI / 2).toArray()); const a2 = m.poses(ids);
    const dist = (P) => P.map(x => Math.hypot(...[0, 1, 2].map(k => x.p[k] - P[0].p[k])));
    return { dx: d[0], spread: m.spread(d), gap: A.editGap() * 1000, world: m.worldOk(), rotKeepsShape: Math.max(...dist(b2).map((v, i) => Math.abs(v - dist(a2)[i]))), rotated: m.qdiff(b2, a2), stillOn: A.gizmoOn && A.tc.object === A.proxy, steps: A.undoS.length, rel0: rel0.length }; }, ids);
  check('G 坐标轴 on 多选/装配体：附着在选中中心（世界轴向），拖动 → 全部一起移动，旋转 → 一起绕中心旋转，连接有效', R.giz.on && R.giz.att && R.giz.atCenter < 1e-9 && Math.abs(R.giz.q - 1) < 1e-12 && Math.abs(R.giz2.dx[0] - 0.1) < 1e-9 && Math.abs(R.giz2.dx[2] + 0.05) < 1e-9 && R.giz2.spread < 1e-9 && R.giz2.gap < 0.5 && R.giz2.world < 1e-9 && R.giz2.rotKeepsShape < 1e-9 && R.giz2.rotated > 0.1 && R.giz2.stillOn, JSON.stringify({ g: R.giz, g2: R.giz2 }));
  await p.keyboard.press('KeyG'); await settle();
  // ---------- 5. arrow keys / PageUp nudge ----------
  R.nudge = {};
  b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids) }), ids);
  await p.keyboard.press('ArrowRight'); await settle();
  let d1 = await p.evaluate((ids, b0) => { const d = app._mv.deltas(b0.P, app._mv.poses(ids)); return { mag: Math.hypot(...d[0]), spread: app._mv.spread(d), y: d[0][1] }; }, ids, b0);
  b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids), u: app.undoS.length }), ids);
  await p.keyboard.down('Shift'); await p.keyboard.press('ArrowUp'); await p.keyboard.up('Shift'); await settle();
  let d2 = await p.evaluate((ids, b0) => { const d = app._mv.deltas(b0.P, app._mv.poses(ids)); return { mag: Math.hypot(...d[0]), spread: app._mv.spread(d), y: d[0][1], steps: app.undoS.length - b0.u }; }, ids, b0);
  b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids), snap: app.snapshot() }), ids);
  await p.keyboard.press('PageUp'); await settle();
  let d3 = await p.evaluate((ids, b0) => { const d = app._mv.deltas(b0.P, app._mv.poses(ids)); const r = { y: d[0][1], spread: app._mv.spread(d), world: app._mv.worldOk(), gap: app.editGap() * 1000 }; app.undo(); r.undo = app.snapshot() === b0.snap; return r; }, ids, b0);
  R.nudge = { d1, d2, d3 };
  check('方向键微调选中装配体：→ 5 mm、Shift+↑ 50 mm（沿世界轴）、PageUp 上移 5 mm，全部零件一起、每次一步撤销', Math.abs(d1.mag - 0.005) < 1e-9 && d1.spread < 1e-9 && d1.y === 0 && Math.abs(d2.mag - 0.05) < 1e-9 && d2.spread < 1e-9 && d2.steps === 1 && Math.abs(d3.y - 0.005) < 1e-9 && d3.spread < 1e-9 && d3.world < 1e-9 && d3.undo, JSON.stringify(R.nudge));
  // ---------- 6. multi-select panel: centre X field + rotation field ----------
  await p.evaluate(ids => { app.msel = new Set(ids); app.select({ p: ids[0] }, true); }, ids); await settle();
  await p.evaluate(() => { const i = document.querySelector('#props .mPos[data-i="0"]'); i.value = '123.4'; i.dispatchEvent(new Event('change')); }); await settle();
  R.panel = await p.evaluate(ids => { const A = app; const c = A.selCenter(A.moveSet(ids[0], { multi: true })); return { x: c.x, field: +document.querySelector('#props .mPos[data-i="0"]').value, sel: A.msel.size, gap: A.editGap() * 1000, world: A._mv.worldOk() }; }, ids);
  b0 = await p.evaluate(ids => ({ P: app._mv.poses(ids) }), ids);
  await p.evaluate(() => { const i = document.getElementById('mRot'); i.value = '90'; i.dispatchEvent(new Event('change')); }); await settle();
  R.panelRot = await p.evaluate((ids, b0) => { const P = app._mv.poses(ids); return { rotated: app._mv.qdiff(b0.P, P), gap: app.editGap() * 1000, world: app._mv.worldOk(), field: document.getElementById('mRot') && document.getElementById('mRot').value }; }, ids, b0);
  check('多选面板：中心位置 X 输入 123.4 mm → 选中中心 X = 123.4 mm；旋转 90° → 绕中心旋转，连接有效', Math.abs(R.panel.x - 0.1234) < 1e-9 && Math.abs(R.panel.field - 123.4) < 0.05 && R.panel.sel === 5 && R.panel.gap < 0.5 && R.panelRot.rotated > 0.1 && R.panelRot.gap < 0.5 && R.panelRot.world < 1e-9, JSON.stringify({ p: R.panel, r: R.panelRot }));
  // ---------- 7. the "stray blue cube": an assembly sunk below the floor shows only its (depth-test-free) joint marker; moving it lifts it back ----------
  ids = await p.evaluate(() => app._mv.build()); await settle();
  R.sunk = await p.evaluate(ids => { const A = app; for (const id of ids) A.part(id).p[1] -= 0.08; for (const j of A.M.joints) if (j.b == null) j.lb[1] -= 0.08; A.lastSnap = A.snapshot(); A.rebuild(); const fixedMarker = Object.values(A.jobjs).some(o => o.children[0] && o.children[0].geometry.type === 'BoxGeometry' && o.children[0].material.depthTest === false); return { fixedMarker }; }, ids);
  await p.evaluate(ids => { app.msel = new Set(ids); app.select({ p: ids[0] }, true); app.view('iso'); app.fit(); }, ids); await settle();
  pt = await p.evaluate(id => app._mv.screen(id), ids[0]);
  if (pt) await drag(pt, 60, 20); else await p.evaluate(() => app.nudgeKey('pagedown'));
  R.lift = await p.evaluate(ids => ({ minY: Math.min(...ids.map(id => app._partMinY(app.part(id)))), gap: app.editGap() * 1000, world: app._mv.worldOk(), picked: true }), ids);
  check('“蓝色小方块”= 埋在地面下的装配体的固定连接标记（不做深度测试）；移动后装配体自动抬回地面以上', R.sunk.fixedMarker && R.lift.minY > -0.0006 && R.lift.gap < 0.5 && R.lift.world < 1e-9, JSON.stringify({ s: R.sunk, l: R.lift, viaMouse: !!pt }));
  // screenshot: assembly selected via header, gizmo at the centre, move panel
  await p.evaluate(() => { const ids = app._mv.build(); app.msel = new Set(ids); app.select({ p: ids[0] }, true); app.toggleGizmo('translate'); app.view('iso'); app.fit(); }); await settle();
  await p.screenshot({ path: OUT + 'screenshot-move-assembly.png' }); await p.evaluate(() => { if (app.gizmoOn) app.toggleGizmo(); });

  // ================= 这个配合也不对: the user's bevel scene rebuilt through the 装配 panel + 🧲 配合 =================
  R.bev = await p.evaluate(() => {
    const A = app; document.getElementById('btnClear').click(); const log = [];
    const pl = (t, x, z) => A.placePart(t, { x, y: 0, z }).id;
    const wall = pl('wall', 0, 0), m1 = pl('motor', 0.5, 0.4), s1 = pl('shaft', -0.4, 0.4), g1 = pl('bevel', 0.6, -0.3), g2 = pl('bevel', -0.5, -0.4), s2 = pl('shaft', 0.8, 0.6), w1 = pl('wheel', -0.8, 0.5), m2 = pl('motor', 0.9, -0.6);
    const asm = (a, b) => { A.msel = new Set([a, b]); A.select({ p: b }, true); A.asmOpen([a, b]); const r = A.asmCommit(); log.push(r.errs.join(';')); return r; };
    asm(wall, m1); asm(s1, m1); asm(g1, s1); asm(g1, g2); asm(s2, g2); asm(w1, s2);
    const nBefore = A.M.joints.length; asm(wall, m1); const dupPanel = A.M.joints.length - nBefore;
    const refs = () => [{ pid: wall, p: [0.15, A.part(wall).d.h / 2, 0.05], n: [0, 1, 0], name: '表面点' }, { pid: m2, p: [0, -A.part(m2).d.s * 0.53, 0], n: [0, -1, 0], name: '表面点' }];
    let r = refs(); A.mate(r[0], r[1], { type: 'fixed' }); const n1 = A.M.joints.length; r = refs(); A.mate(r[0], r[1], { type: 'fixed' }); const dupMate = A.M.joints.length - n1;
    const V = A._V, Q = A.cam.quaternion.constructor, ax = P => new V(0, 0, 1).applyQuaternion(new Q(...P.q)), G1 = A.part(g1), G2 = A.part(g2);
    const apex = P => new V(...P.p).addScaledVector(ax(P), -P.d.A), rG1 = G1.d.m * G1.d.z / 2, rG2 = G2.d.m * G2.d.z / 2;
    // pitch circles touch: point on both
    const u = new V(...G2.p).sub(new V(...G1.p)); u.addScaledVector(ax(G1), -u.dot(ax(G1))); const P1 = new V(...G1.p).addScaledVector(u.normalize(), rG1);
    const pitchErr = Math.abs(P1.distanceTo(new V(...G2.p)) - rG2) + Math.abs(P1.clone().sub(new V(...G2.p)).dot(ax(G2)));
    const key = j => [j.type, [j.a, j.b || 'W'].sort().join('-')].join(':'); const keys = A.M.joints.map(key);
    const W = A.part(wall), M2 = A.part(m2);
    const out = { log, dupPanel, dupMate, uniqueJoints: new Set(keys).size === keys.length, gearWorldHinge: A.M.joints.some(j => j.b == null && (j.a === g1 || j.a === g2)), shaft2Bearing: A.M.joints.some(j => j.b == null && j.a === s2 && j.type === 'revolute'),
      m2Flush: Math.abs((M2.p[1] - M2.d.s * 0.53) - (W.p[1] + W.d.h / 2)), m2OnWall: Math.abs(M2.p[0] - W.p[0]) < W.d.w / 2 && Math.abs(M2.p[2] - W.p[2]) < W.d.dp / 2, m2Joint: A.M.joints.filter(j => [j.a, j.b].includes(m2) && [j.a, j.b].includes(wall)).length,
      bevelDot: ax(G1).dot(ax(G2)), apexErr: apex(G1).distanceTo(apex(G2)), pitchErr, audit: A.connAudit().maxGap * 1000, gap: A.editGap() * 1000, couplingOk: A.M.couplings.every(c => A.M.joints.some(j => j.id === c.ja) && A.M.joints.some(j => j.id === c.jb)) };
    A.startRun(); A.simulate(1); const w = n => { const b = A.sim.pb[n].body.angvel(), a = ax(A.part(n)); return b.x * a.x + b.y * a.y + b.z * a.z; }; out.runGap = A.sim.maxGap() * 1000; out.ratio = w(g2) / w(g1); A.reset();
    return out;
  });
  check('配合修复：电机2 → 地面1（🧲 配合）移到顶面上贴合、只建 1 个固定连接；再配合/再装配不重复', R.bev.m2Flush < 0.001 && R.bev.m2OnWall && R.bev.m2Joint === 1 && R.bev.dupMate === 0 && R.bev.dupPanel === 0 && R.bev.uniqueJoints, JSON.stringify({ m2Flush: R.bev.m2Flush, m2OnWall: R.bev.m2OnWall, m2Joint: R.bev.m2Joint, dupMate: R.bev.dupMate, dupPanel: R.bev.dupPanel }));
  check('锥齿轮 90° 啮合：轴线垂直、锥顶重合、节圆相切；锥齿轮上不留“↔大地”铰链（改为轴2 的支承），运行同步', Math.abs(R.bev.bevelDot) < 1e-6 && R.bev.apexErr < 1e-6 && R.bev.pitchErr < 1e-6 && !R.bev.gearWorldHinge && R.bev.shaft2Bearing && R.bev.couplingOk && R.bev.gap < 0.5 && R.bev.audit < 1 && R.bev.runGap < 1 && Math.abs(Math.abs(R.bev.ratio) - 1) < 0.02, JSON.stringify(R.bev));
  // auto ground hinge is removed once the gear is fixed to a shaft that has its own (coaxial) support; coupling re-planned
  R.prune = await p.evaluate(() => {
    const A = app; document.getElementById('btnClear').click();
    const pl = (t, x, z) => A.placePart(t, { x, y: 0, z }).id; const asm = (a, b) => { A.msel = new Set([a, b]); A.select({ p: b }, true); A.asmOpen([a, b]); return A.asmCommit(); };
    const m = pl('motor', 0, 0), s1 = pl('shaft', 0.3, 0), g1 = pl('gear', 0.5, 0.2), g2 = pl('gear', 0.6, -0.3), br = A.placePart('bearing', { x: -0.4, y: 0, z: 0.4 }, { fixed: true }).id, s2 = pl('shaft', -0.6, 0.5);
    asm(s1, m); asm(g1, s1); const hinge1 = A.M.joints.some(j => j.b == null && j.a === g1); asm(g1, g2); const auto2 = A.M.joints.some(j => j.b == null && j.a === g2 && j.auto);
    asm(s2, br); asm(g2, s2);
    const r = { hinge1, auto2, after: A.M.joints.filter(j => j.b == null && j.type === 'revolute').map(j => A.part(j.a).name), couplingOk: A.M.couplings.every(c => A.M.joints.some(j => j.id === c.ja) && A.M.joints.some(j => j.id === c.jb)), gap: A.editGap() * 1000 };
    A.startRun(); A.simulate(1); const V = A._V, Q = A.cam.quaternion.constructor, w = n => { const b = A.sim.pb[n].body.angvel(), a = new V(0, 0, 1).applyQuaternion(new Q(...A.part(n).q)); return b.x * a.x + b.y * a.y + b.z * a.z; }; r.ratio = w(g2) / w(g1); r.runGap = A.sim.maxGap() * 1000; A.reset();
    return r;
  });
  check('齿轮已装在有铰链支承的轴上 → 不加接地铰链；后来固定到有轴承支承的轴上 → 自动删除多余的接地铰链（传动改挂到轴承铰链）', !R.prune.hinge1 && R.prune.auto2 && !R.prune.after.some(n => /齿轮/.test(n)) && R.prune.couplingOk && R.prune.gap < 0.5 && R.prune.runGap < 1 && Math.abs(Math.abs(R.prune.ratio) - 1) < 0.02, JSON.stringify(R.prune));
  // refuse to join visually separated parts that cannot be moved
  R.refuse = await p.evaluate(() => {
    const A = app; document.getElementById('btnClear').click();
    const a = A.placePart('wall', { x: 0, y: 0, z: 0 }), b = A.placePart('wall', { x: 1.2, y: 0, z: 0.6 });
    A.beginEdit(); A.addJoint('fixed', A.part(a.id), null, new A._V(...a.p), [0, 1, 0]); A.addJoint('fixed', A.part(b.id), null, new A._V(...b.p), [0, 1, 0]); A.commit(); A.rebuild();
    const n0 = A.M.joints.length, pos = JSON.stringify(A.M.parts.map(P => P.p));
    const j = A.mate({ pid: a.id, p: [0, 0.02, 0], n: [0, 1, 0], name: 's' }, { pid: b.id, p: [0, -0.02, 0], n: [0, -1, 0], name: 's' }, { type: 'fixed' }); const mmsg = document.getElementById('stMsg').textContent;
    const r = { mate: !!j, mmsg, joints: A.M.joints.length - n0, unmoved: JSON.stringify(A.M.parts.map(P => P.p)) === pos };
    // panel: two rods already in the same assembly (both hinged to one fixed base) with far-apart holes → refused, nothing changes
    document.getElementById('btnClear').click(); const base = A.placePart('base', { x: 0, y: 0, z: 0 }), r1 = A.placePart('rod', { x: 0.4, y: 0, z: 0.3 }), r2 = A.placePart('rod', { x: -0.4, y: 0, z: 0.5 });
    const D = A.part(base.id); A.beginEdit(); A.addJoint('revolute', D, A.part(r1.id), new A._V(D.p[0], D.d.h, 0.05), [0, 0, 1]); A.addJoint('revolute', D, A.part(r2.id), new A._V(D.p[0], D.d.h * 0.5, -0.05), [0, 0, 1]); A.commit(); A.rebuild();
    const snap = A.snapshot(); A.msel = new Set([r1.id, r2.id]); A.select({ p: r2.id }, true); A.asmOpen([r1.id, r2.id]); A.asmSet(0, { type: 'revolute' }); const res = A.asmCommit();
    r.panelErr = res.errs.join(';') + (res.reverted ? ' reverted' : ''); r.panelSame = A.snapshot() === snap; r.panelMsg = document.getElementById('stMsg').textContent;
    return r;
  });
  check('两个都已接地且分开的零件：配合/装配拒绝建立连接并给出中文提示（不在分开的零件之间连线）', !R.refuse.mate && /无法配合/.test(R.refuse.mmsg) && R.refuse.joints === 0 && R.refuse.unmoved && /未建立连接|撤回/.test(R.refuse.panelErr + R.refuse.panelMsg) && R.refuse.panelSame, JSON.stringify(R.refuse));
  // screenshot of the fixed bevel scene
  await p.evaluate(() => {
    const A = app; document.getElementById('btnClear').click(); const pl = (t, x, z) => A.placePart(t, { x, y: 0, z }).id; const asm = (a, b) => { A.msel = new Set([a, b]); A.select({ p: b }, true); A.asmOpen([a, b]); A.asmCommit(); };
    const wall = pl('wall', 0, 0), m1 = pl('motor', 0.5, 0.4), s1 = pl('shaft', -0.4, 0.4), g1 = pl('bevel', 0.6, -0.3), g2 = pl('bevel', -0.5, -0.4), s2 = pl('shaft', 0.8, 0.6), w1 = pl('wheel', -0.8, 0.5), m2 = pl('motor', 0.9, -0.6);
    asm(wall, m1); asm(s1, m1); asm(g1, s1); asm(g1, g2); asm(s2, g2); asm(w1, s2);
    A.mate({ pid: wall, p: [-0.18, A.part(wall).d.h / 2, -0.08], n: [0, 1, 0], name: '表面点' }, { pid: m2, p: [0, -A.part(m2).d.s * 0.53, 0], n: [0, -1, 0], name: '表面点' }, { type: 'fixed' });
    A.select(null); A.flash = null; A.view('iso'); A.fit();
  }); await new Promise(r => setTimeout(r, 300)); await settle();
  await p.screenshot({ path: OUT + 'screenshot-bevel-fix.png' });
};
