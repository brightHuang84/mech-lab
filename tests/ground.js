// Regression: 装配的时候，零件会到地面下面 — after ANY pose-changing edit every part's real geometry stays at y >= 0
// (assembly panel apply + preview, mate, snap on drop + ghost, rotate R, resize, nudge, gizmo, group move, undo/redo,
// presets/examples, import, copy/mirror, negative Y in the position field). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const TOL = -0.0005;
  await p.evaluate(() => {
    const A = app, V = A.cam.position.constructor, Q = A.cam.quaternion.constructor, B3 = A.hl.geometry && A.hl.geometry.boundingBox ? A.hl.geometry.boundingBox.constructor : null;
    app._V = V;
    // independent measurement: fresh mesh of the part at its live (displayed) pose, every vertex in world space
    const meshMin = (P, ps) => { const g = A.DEFS[P.type].mesh(P.d, P); g.position.copy(ps.p); g.quaternion.copy(ps.q); g.updateMatrixWorld(true); let m = Infinity; const v = new V();
      g.traverse(o => { if (o.isMesh && o.geometry && o.geometry.attributes.position) { const pa = o.geometry.attributes.position; for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(o.matrixWorld); if (v.y < m) m = v.y; } } }); return m; };
    app._g = {
      meshMin,
      minAll(ids) { let lo = Infinity, who = null; for (const P of A.M.parts) { if (ids && !ids.includes(P.id)) continue; const m = meshMin(P, A.livePose(P.id)); if (m < lo) { lo = m; who = P.name; } } return { lo, who }; },
      // displayed objects (incl. ghosts) — scene graph bounding boxes, precise
      objMin(ids) { let lo = Infinity; const v = new V(); for (const id of ids) { const o = A.objs[id]; if (!o) continue; o.updateMatrixWorld(true); o.traverse(x => { if (x.isMesh && x.geometry && x.geometry.attributes.position && !(x.userData && x.userData.helper)) { const pa = x.geometry.attributes.position; for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(x.matrixWorld); if (v.y < lo) lo = v.y; } } }); } return lo; },
      rel(a, b) { const Pa = A.part(a), Pb = A.part(b), qa = new Q(...Pa.q), d = new V(...Pb.p).sub(new V(...Pa.p)).applyQuaternion(qa.clone().invert()), qr = qa.clone().invert().multiply(new Q(...Pb.q)); return [...d.toArray(), ...qr.toArray()]; },
      relDiff(r1, r2) { return Math.max(...r1.map((x, i) => i < 3 ? Math.abs(x - r2[i]) : 0), 1 - Math.abs(r1[3] * r2[3] + r1[4] * r2[4] + r1[5] * r2[5] + r1[6] * r2[6])); },
      clear() { if (A.asm) A.asmCancel(true); if (A.mateS) A.mateCancel(); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select'); if (A.gizmoOn) A.toggleGizmo(); document.getElementById('btnClear').click(); A.msel = new Set(); },
      // the user's scene: motor (fixed, on the ground) + shaft on its output (assembled with the panel)
      motorShaft() { const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), s = A.placePart('shaft', { x: 0.25, y: 0, z: 0 }); A.msel = new Set([m.id, s.id]); A.select({ p: s.id }, true); A.asmOpen([m.id, s.id]); A.asmCommit(); return [m.id, s.id]; },
      worst() { return app._g.minAll().lo; },
    };
  });
  const R = {};
  // ---------- 1. panel: big gear onto the motor shaft (B would go below the ground) + live preview + gear mesh ----------
  R.panel = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const [m, s] = G.motorShaft(); const rel0 = G.rel(m, s), axisY0 = A.part(s).p[1];
    const g1 = A.placePart('gear', { x: 0.3, y: 0, z: 0.25 }, { d: { m: 0.004, z: 36, t: 0.02 } }); // tip radius 76 mm > motor axis height 43 mm
    A.msel = new Set([s, g1.id]); A.select({ p: g1.id }, true); A.asmOpen([s, g1.id]);
    const pv = { mover: A.asm.info[0].mover === g1.id, lo: G.minAll().lo, ghost: G.objMin([g1.id, m, s]), info: document.querySelector('#asmPanel [data-info="0"]') ? document.querySelector('#asmPanel [data-info="0"]').textContent : '' };
    const r = A.asmCommit(); const after = G.minAll();
    const out = { pv, changed: r.changed, lo: after.lo, who: after.who, gap: A.editGap() * 1000, rel: G.relDiff(rel0, G.rel(m, s)), lift: A.part(s).p[1] - axisY0, joints: A.M.joints.length };
    // gear mesh via the panel: second gear meshes with the first (placed lower than its radius allows)
    const g2 = A.placePart('gear', { x: 0.55, y: 0, z: 0.3 }, { d: { m: 0.004, z: 50, t: 0.02 } });
    A.msel = new Set([g1.id, g2.id]); A.select({ p: g2.id }, true); A.asmOpen([g1.id, g2.id]); out.meshType = A.asm.pairs[0].type; out.meshPv = G.minAll().lo; const r2 = A.asmCommit();
    out.mesh = { changed: r2.changed, couplings: r2.couplings.length, lo: G.minAll().lo, gap: A.editGap() * 1000 };
    return out;
  });
  check('装配面板：大齿轮装到电机轴上（B 会到地面下）→ 预览与确定后所有零件几何最低点 ≥ 0，整体抬升、相对位姿/连接保持', R.panel.changed && R.panel.pv.lo >= TOL && R.panel.pv.ghost >= TOL && R.panel.lo >= TOL && R.panel.gap < 0.5 && R.panel.rel < 1e-9 && R.panel.lift > 0.02 && /已抬升至地面以上/.test(R.panel.pv.info), JSON.stringify(R.panel));
  check('装配面板：齿轮啮合（第二个大齿轮）→ 预览与确定后仍在地面以上，传动建立', R.panel.meshType === 'gear' && R.panel.meshPv >= TOL && R.panel.mesh.changed && R.panel.mesh.couplings === 1 && R.panel.mesh.lo >= TOL && R.panel.mesh.gap < 0.5, JSON.stringify({ t: R.panel.meshType, pv: R.panel.meshPv, mesh: R.panel.mesh }));
  // ---------- 1b. panel: the moving side B (motor + shaft, free) hangs below a gear lying flat → the joined group is lifted ----------
  R.panelB = await p.evaluate(() => {
    const A = app, G = A._g, V = A._V, Q = A.cam.quaternion.constructor; G.clear();
    const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), s = A.placePart('shaft', { x: 0.25, y: 0, z: 0 }); A.part(m.id).fixed = false; A.msel = new Set([m.id, s.id]); A.select({ p: s.id }, true); A.asmOpen([m.id, s.id]); A.asmCommit();
    const g = A.placePart('gear', { x: 0.3, y: 0, z: 0.3 }, { d: { m: 0.004, z: 30, t: 0.02 } }, { q: new Q().setFromAxisAngle(new V(1, 0, 0), -Math.PI / 2) }); // lying flat, hub axis vertical
    A.msel = new Set([g.id, s.id]); A.select({ p: s.id }, true); A.asmOpen([g.id, s.id]); const pr = A.asm.pairs[0]; const pv = { mover: A.asm.info[0].mover, lo: G.minAll().lo };
    const r = A.asmCommit(); const after = G.minAll();
    return { pv, b: s.id, changed: r.changed, lo: after.lo, who: after.who, gap: A.editGap() * 1000, gearY: A.part(g.id).p[1], type: pr.type };
  });
  check('装配面板：移动的 B 侧（电机+轴）会插到平放齿轮下方地面以下 → 整组抬升到地面以上', R.panelB.changed && R.panelB.pv.lo >= TOL && R.panelB.lo >= TOL && R.panelB.gap < 0.5 && R.panelB.gearY > 0.05, JSON.stringify(R.panelB));
  // ---------- 2. resize the larger gear (on the motor assembly) / a free gear standing on the ground ----------
  R.resize = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const [m, s] = G.motorShaft(); const g1 = A.placePart('gear', { x: 0.3, y: 0, z: 0.25 }, { d: { m: 0.004, z: 20, t: 0.02 } }); A.msel = new Set([s, g1.id]); A.select({ p: g1.id }, true); A.asmOpen([s, g1.id]); A.asmCommit();
    const r1 = A.resizePart(g1.id, { z: 48 }); const a = { msg: r1.msg, ok: r1.ok, lo: G.minAll().lo, gap: A.editGap() * 1000 };
    G.clear(); const g = A.placePart('gear', { x: 0, y: 0, z: 0 }); const r2 = A.resizePart(g.id, { z: 60 }); const lo = G.minAll().lo;
    const live = (() => { const g2 = A.placePart('gear', { x: 0.4, y: 0, z: 0 }); const rr = A.resizeLive(g2.id, { z: 70 }); const l = G.minAll([g2.id]).lo; A.resizeCancel(); return { ok: rr.ok, lo: l }; })();
    return { a, b: { ok: r2.ok, lo }, live };
  });
  check('调整尺寸：电机轴上的齿轮改大 / 地面上的齿轮改大（含实时预览）→ 不陷入地面、仍贴地', R.resize.a.ok && R.resize.a.lo >= TOL && R.resize.a.gap < 0.5 && R.resize.b.ok && R.resize.b.lo >= TOL && R.resize.b.lo < 0.002 && R.resize.live.lo >= TOL, JSON.stringify(R.resize));
  // ---------- 3. mate tool: gear hub onto the fixed motor's output shaft ----------
  R.mate = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const m = A.placePart('motor', { x: 0, y: 0, z: 0 }), g = A.placePart('gear', { x: 0.3, y: 0, z: 0.2 }, { d: { m: 0.004, z: 40, t: 0.02 } });
    A.mateStart({ pid: g.id, p: [0, 0, 0], n: [0, 0, 1] }, { pid: m.id, p: [0, 0, 0.088], n: [0, 0, 1] }, { type: 'revolute' }); const pv = G.minAll().lo; A.mateCommit();
    return { pv, lo: G.minAll().lo, joints: A.M.joints.filter(j => j.a === g.id || j.b === g.id).length, gap: A.editGap() * 1000 };
  });
  check('🧲 配合：齿轮配到电机输出轴（会陷入地面）→ 预览与确定后都在地面以上', R.mate.pv >= TOL && R.mate.lo >= TOL && R.mate.joints >= 1 && R.mate.gap < 0.5, JSON.stringify(R.mate));
  // ---------- 4. snap on palette drop (real mouse) — the snapped ghost stays above the ground too ----------
  const m4 = await p.evaluate(() => { const A = app; A._g.clear(); A.view('front'); const m = A.placePart('motor', { x: 0, y: 0, z: 0 }); A.cam.position.set(0.05, 0.12, 0.9); A.orbit.target.set(0.05, 0.05, 0.05); A.orbit.update(); A.select(null); return m.id; });
  await frame();
  const tip = await p.evaluate(id => { const A = app, P = A.part(id), w = new A._V(0, 0, 0.088).applyQuaternion(new A.cam.quaternion.constructor(...P.q)).add(new A._V(...P.p)), s = A.toScreen(w); return [s.x, s.y]; }, m4);
  const box = await (await p.$('.item[data-type="gear"]')).boundingBox();
  await p.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await p.mouse.down(); await p.mouse.move(tip[0] - 30, tip[1] + 20, { steps: 6 }); await p.mouse.move(tip[0], tip[1], { steps: 4 }); await frame();
  R.ghost = await p.evaluate(() => { const A = app, g = A.ghostObj; if (!g) return null; g.updateMatrixWorld(true); let lo = Infinity; const v = new A._V(); g.traverse(x => { if (x.isMesh && x.geometry) { const pa = x.geometry.attributes.position; for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(x.matrixWorld); lo = Math.min(lo, v.y); } } }); return { lo, snap: !!A.snapTarget, motor: A._g.minAll().lo }; });
  await p.mouse.up(); await frame();
  R.snap = await p.evaluate(m => { const A = app, G = A._g, g = A.M.parts[A.M.parts.length - 1]; return { type: g.type, joined: A.M.joints.some(j => (j.a === g.id && j.b === m) || (j.b === g.id && j.a === m)), lo: G.minAll().lo, gap: A.editGap() * 1000 }; }, m4);
  check('吸附装配（零件库拖入齿轮到电机输出轴）：拖动中半透明预览和松开后都在地面以上', !!R.ghost && R.ghost.snap && R.ghost.lo >= TOL && R.ghost.motor >= TOL && R.snap.type === 'gear' && R.snap.joined && R.snap.lo >= TOL && R.snap.gap < 0.5, JSON.stringify({ ghost: R.ghost, snap: R.snap }));
  // ---------- 5. rotate with R (real key) / multi-select rotate / nudge / gizmo (live) / group move ----------
  const rodId = await p.evaluate(() => { const A = app; A._g.clear(); A.view('iso'); A.fit(); const r = A.placePart('rod', { x: 0, y: 0, z: 0 }); A.rotAxis = 'Z'; A.select({ p: r.id }); return r.id; });
  const rot = []; for (let i = 0; i < 6; i++) { await p.evaluate(() => document.activeElement && document.activeElement.blur()); await p.keyboard.press('KeyR'); rot.push(await p.evaluate(() => app._g.minAll().lo)); }
  R.rot = await p.evaluate((rot, id) => { const A = app; const q = A.part(id).q; return { min: Math.min(...rot), steps: rot.length, tilt: 2 * Math.acos(Math.min(1, Math.abs(q[3]))) * 180 / Math.PI }; }, rot, rodId);
  check('R 键旋转（杆绕 Z 轴转 6×15° = 90°）→ 每一步都不陷入地面', R.rot.min >= TOL && Math.abs(R.rot.tilt - 90) < 1, JSON.stringify(R.rot));
  R.moves = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const [m, s] = G.motorShaft(); const g = A.placePart('gear', { x: 0.3, y: 0, z: 0.3 }); A.msel = new Set([s, g.id]); A.select({ p: g.id }, true); A.asmOpen([s, g.id]); A.asmCommit();
    const ids = [m, s, g.id], out = {}; A.msel = new Set(ids); A.select({ p: m }, true);
    A.rotateSelBy('X', 90); out.rotX = G.minAll().lo;
    for (let i = 0; i < 8; i++) A.nudgeKey('pagedown', true); out.nudge = G.minAll().lo;
    A.moveSelBy([0, -0.3, 0]); out.groupMove = G.minAll().lo;
    A.setSelCenter(1, -0.5); out.center = G.minAll().lo;
    // gizmo: live preview while dragging down, then release
    A.toggleGizmo('translate'); A.updSel(); A.dragStart(); const p0 = A.proxy.position.clone(); let live = Infinity; for (let i = 1; i <= 6; i++) { A.proxy.position.copy(p0.clone().add(new A._V(0, -0.05 * i, 0))); A.dragMove(A.proxy.position.clone(), A.proxy.quaternion.clone()); live = Math.min(live, G.minAll().lo, G.objMin(ids)); } A.dragEnd(); out.gizmoLive = live; out.gizmo = G.minAll().lo;
    if (A.gizmoOn) A.toggleGizmo();
    out.gap = A.editGap() * 1000; return out;
  });
  check('组移动/绕 X 旋转 90°/PageDown/中心 Y=-500mm/坐标轴拖动（含拖动中预览）→ 装配体始终在地面以上，连接保持', Object.entries(R.moves).every(([k, v]) => k === 'gap' ? v < 0.5 : v >= TOL), JSON.stringify(R.moves));
  // ---------- 6. negative Y in the position field → clamped + toast ----------
  R.field = await p.evaluate(() => {
    const A = app; A._g.clear(); const b = A.placePart('block', { x: 0, y: 0, z: 0 }); A.select({ p: b.id });
    const lab = [...document.querySelectorAll('#props *')].find(e => e.children.length === 0 && e.textContent.trim() === '位置'); const row = lab && lab.closest('div'); const inp = row ? row.querySelectorAll('input')[1] : null; if (!inp) return { err: 'no field' };
    A.lastToast = ''; inp.value = '-120'; inp.dispatchEvent(new Event('change')); const t = document.getElementById('gToast');
    return { y: A.part(b.id).p[1], lo: A._g.minAll().lo, toast: A.lastToast, shown: !!t && t.style.opacity === '1' };
  });
  check('位置 Y 输入 -120 mm → 仍抬回地面以上，并提示“已抬升至地面以上”', !R.field.err && R.field.lo >= TOL && R.field.lo < 0.002 && /已抬升至地面以上/.test(R.field.toast) && R.field.shown, JSON.stringify(R.field));
  // ---------- 7. copy / mirror below the ground ----------
  R.clone = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const [m, s] = G.motorShaft(); const ids = [m, s];
    const c = A.cloneSet(ids, 'copy', { off: [0.3, -0.2, 0] }); const a = G.minAll(c).lo;
    const mi = A.cloneSet(ids, 'mirror', { axis: 1, at: 0 }); const b = G.minAll(mi).lo; return { copy: a, mirror: b, all: G.minAll().lo, gap: A.editGap() * 1000 };
  });
  check('复制（向下偏移）/ 镜像（Y=0 平面，镜像到地下）→ 副本被抬升到地面以上', R.clone.copy >= TOL && R.clone.mirror >= TOL && R.clone.all >= TOL && R.clone.gap < 0.5, JSON.stringify(R.clone));
  // ---------- 8. undo / redo (incl. an old stored state that was below ground) ----------
  R.undo = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const [m, s] = G.motorShaft(); const g = A.placePart('gear', { x: 0.3, y: 0, z: 0.3 }, { d: { m: 0.004, z: 40, t: 0.02 } }); A.msel = new Set([s, g.id]); A.select({ p: g.id }, true); A.asmOpen([s, g.id]); A.asmCommit();
    A.resizePart(g.id, { z: 56 }); A.rotAxis = 'Y'; A.select({ p: g.id }); A.rotateSel(1, 30);
    // a stale (pre-fix) state in the undo history: everything sunk 0.1 m
    const bad = JSON.parse(A.snapshot()); bad.parts.forEach(P => P.p[1] -= 0.1); bad.joints.forEach(j => { if (j.b == null) j.lb[1] -= 0.1; }); A.undoS.push(JSON.stringify(bad)); A.redoS = [];
    const n = A.undoS.length, u = [], r = [];
    for (let i = 0; i < n; i++) { A.undo(); u.push(G.minAll().lo); }
    for (let i = 0; i < n; i++) { A.redo(); r.push(G.minAll().lo); }
    return { n, undoMin: Math.min(...u), redoMin: Math.min(...r), gap: A.editGap() * 1000, firstUndo: u[0] };
  });
  check('撤销/重做每一步（含历史中低于地面的旧状态）→ 所有零件都在地面以上', R.undo.n > 4 && R.undo.undoMin >= TOL && R.undo.redoMin >= TOL && R.undo.gap < 0.5, JSON.stringify(R.undo));
  // ---------- 9. import (JSON with sunk parts + a ground hinge): lifted with its world anchor ----------
  R.imp = await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const b = A.placePart('base', { x: 0, y: 0, z: 0 }), r = A.placePart('rod', { x: 0.3, y: 0, z: 0 }); A.addJoint('revolute', A.part(r.id), null, new A._V(...A.part(r.id).p), new A._V(0, 0, 1)); A.commit();
    const M = JSON.parse(A.snapshot()); M.parts.forEach(P => P.p[1] -= 0.25); M.joints.forEach(j => { if (j.b == null) j.lb[1] -= 0.25; });
    A.loadJSON(JSON.stringify({ model: M })); return { lo: G.minAll().lo, gap: A.editGap() * 1000, parts: A.M.parts.length };
  });
  check('读取 JSON（零件在地面下 + 接地铰链）→ 抬升到地面以上，接地锚点跟随（连接间隙 < 0.5 mm）', R.imp.parts === 2 && R.imp.lo >= TOL && R.imp.gap < 0.5, JSON.stringify(R.imp));
  // ---------- 10. every preset / example ----------
  R.pre = await p.evaluate(() => { const A = app, out = []; [...document.querySelectorAll('#selPreset option')].slice(1).forEach((o, i) => { A.loadPreset(i); const m = A._g.minAll(); out.push([o.textContent, +(m.lo * 1000).toFixed(2), m.who]); }); return out; });
  check('载入全部示例 → 所有零件几何最低点 ≥ 0', R.pre.length >= 10 && R.pre.every(x => x[1] >= TOL * 1000), JSON.stringify(R.pre.filter(x => x[1] < 1)));
  // ---------- screenshot: the user's scene (motor + shaft + two spur gears) assembled with the panel, all on/above the ground ----------
  await p.evaluate(() => {
    const A = app, G = A._g; G.clear(); const [m, s] = G.motorShaft();
    const g1 = A.placePart('gear', { x: 0.3, y: 0, z: 0.25 }, { d: { m: 0.004, z: 36, t: 0.02 } }); A.msel = new Set([s, g1.id]); A.select({ p: g1.id }, true); A.asmOpen([s, g1.id]); A.asmCommit();
    const g2 = A.placePart('gear', { x: 0.55, y: 0, z: 0.3 }, { d: { m: 0.004, z: 24, t: 0.02 } }); A.msel = new Set([g1.id, g2.id]); A.select({ p: g2.id }, true); A.asmOpen([g1.id, g2.id]); A.asmCommit();
    A.select(null); A.flash = null; const t = document.getElementById('gToast'); if (t) t.style.opacity = '0'; A.cam.position.set(0.42, 0.075, 0.55); A.orbit.target.set(0.05, 0.045, 0.1); A.orbit.update();
    A.msg('装配后整体抬升：电机、轴、两个齿轮的最低点 = ' + (G.minAll().lo * 1000).toFixed(2) + ' mm（地面 y = 0）');
  });
  await new Promise(r => setTimeout(r, 400)); await frame();
  await p.screenshot({ path: OUT + 'screenshot-ground-fix.png' }); console.log('saved screenshot-ground-fix.png');
  await p.evaluate(() => { app._g.clear(); });
};
