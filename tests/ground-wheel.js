// Regression 2: 装配的时候，还是会到地面下面 — wheel cases (large wheels, axis horizontal / rotated), plate + motor + shaft + wheels,
// palette drop with rotation, panel / mate / hinge-to-ground, resize, run → 设为初始状态 → 重置, and the render-time safety net. Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const TOL = -0.0005;
  await p.evaluate(() => {
    const A = app, V = A.cam.position.constructor, Q = A.cam.quaternion.constructor;
    const meshMin = (P, ps) => { const g = A.DEFS[P.type].mesh(P.d, P); g.position.copy(ps.p); g.quaternion.copy(ps.q); g.updateMatrixWorld(true); let m = Infinity; const v = new V(); g.traverse(o => { if (o.isMesh && o.geometry && o.geometry.attributes.position) { const pa = o.geometry.attributes.position; for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(o.matrixWorld); if (v.y < m) m = v.y; } } }); return m; };
    A._gw = {
      V, Q,
      clear() { if (A.asm) A.asmCancel(true); if (A.mateS) A.mateCancel(); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select'); if (A.gizmoOn) A.toggleGizmo(); document.getElementById('btnClear').click(); A.msel = new Set(); A.palRot = null; },
      pair(a, b, patch) { A.msel = new Set([a, b]); A.select({ p: b }, true); A.asmOpen([a, b]); if (patch) A.asmSet(0, patch); const pv = A._gw.all(); const r = A.asmCommit(); return { pv, r }; },
      // model (live pose) AND what is drawn (scene objects incl. the palette ghost)
      model() { let lo = Infinity, who = null; for (const P of A.M.parts) { const ps = A.livePose(P.id); if (!ps) continue; const m = meshMin(P, ps); if (m < lo) { lo = m; who = P.name; } } return { lo, who }; },
      drawn() { let lo = Infinity; const v = new V(), list = Object.values(A.objs); if (A.ghostObj && A.ghostObj.parent) list.push(A.ghostObj); for (const o of list) { if (!o.visible) continue; o.updateMatrixWorld(true); o.traverse(x => { if (x.isMesh && x.geometry && x.geometry.attributes.position && !(x.userData && x.userData.helper)) { const pa = x.geometry.attributes.position; for (let i = 0; i < pa.count; i++) { v.fromBufferAttribute(pa, i).applyMatrix4(x.matrixWorld); if (v.y < lo) lo = v.y; } } }); } return lo; },
      all() { return Math.min(A._gw.model().lo, A._gw.drawn()); },
      bigWheel(x, z, r = 0.14) { return A.placePart('wheel', { x, y: 0, z }, { d: { r, t: 0.05 } }, { q: new Q().setFromAxisAngle(new V(0, 1, 0), Math.PI / 2) }); }, // axis along world X
      /** the user's scene: plate + motor on it + shaft out of the motor + bevel + large wheel at the shaft end */
      car() { const G = A._gw, out = {}; const pl = A.placePart('plate', { x: 0, y: 0, z: 0 }, { d: { w: 0.3, h: 0.02, dp: 0.2, np: 3 } }); const m = A.placePart('motor', { x: 0.4, y: 0, z: 0 }); out.motor = G.pair(pl.id, m.id).pv;
        const s = A.placePart('shaft', { x: 0.4, y: 0, z: 0.4 }, { d: { L: 0.16, r: 0.006 } }); out.shaft = G.pair(m.id, s.id).pv;
        const bv = A.placePart('bevel', { x: 0.6, y: 0, z: 0.4 }); out.bevel = G.pair(s.id, bv.id, { pa: { s: 0.0 } }).pv;
        const w = G.bigWheel(0.8, 0.4); out.wheel = G.pair(s.id, w.id, { pa: { s: 0.05 } }).pv;
        return { ids: { pl: pl.id, m: m.id, s: s.id, bv: bv.id, w: w.id }, pv: out }; },
    };
  });
  const R = {};
  // ---------- 1. car: plate + motor + shaft + bevel + large wheel (axis horizontal), all via the panel: preview + commit ----------
  R.car = await p.evaluate(() => { const A = app, G = A._gw; G.clear(); const c = G.car(); A._gwCar = c.ids; return { pv: Math.min(...Object.values(c.pv)), lo: G.all(), gap: A.editGap() * 1000, wheelY: A.part(c.ids.w).p[1] }; });
  check('轮·场景：板+电机+轴+锥齿轮+大轮（R140，轴线水平）用装配面板装配 → 每一步预览与确定后都在地面以上', R.car.pv >= TOL && R.car.lo >= TOL && R.car.gap < 0.5 && R.car.wheelY > 0.13, JSON.stringify(R.car));
  // ---------- 2. a second large wheel onto the plate side pin (wheel would be centred at plate mid-height → sunk) ----------
  R.side = await p.evaluate(() => { const A = app, G = A._gw, c = A._gwCar; const w = G.bigWheel(-0.4, 0.3, 0.16); const ent = A.asmEnts(c.pl).find(e => /侧面1\(\+Z\)/.test(e.name)); const r = G.pair(c.pl, w.id, { ka: ent.key, kb: A.asmEnts(w.id)[0].key, type: 'revolute' }); return { pv: r.pv, lo: G.all(), changed: r.r.changed, gap: A.editGap() * 1000, plateLift: A.part(c.pl).p[1] }; });
  check('轮·大轮铰接到板侧面连接点（轮心=板中高 → 会陷入地面）→ 预览/确定后整车抬升，轮不入地', R.side.changed && R.side.pv >= TOL && R.side.lo >= TOL && R.side.gap < 0.5 && R.side.plateLift > 0.1, JSON.stringify(R.side));
  // ---------- 3. hinge a wheel to the ground at its centre, Y field = 0 (centre on the floor), mate onto the shaft end ----------
  R.misc = await p.evaluate(() => {
    const A = app, G = A._gw, out = {}; G.clear();
    const w = G.bigWheel(0, 0); A.makeJointBetween('revolute', A.part(w.id), { pt: A.DEFS.wheel.pts(A.part(w.id).d)[0] }, null, null); out.hinge = G.all();
    A.select({ p: w.id }); const lab = [...document.querySelectorAll('#props *')].find(e => e.children.length === 0 && e.textContent.trim() === '位置'); const inp = lab && lab.closest('div').querySelectorAll('input')[1];
    if (inp) { inp.value = '0'; inp.dispatchEvent(new Event('change')); } out.field = G.all(); out.fieldOk = !!inp;
    const m = A.placePart('motor', { x: 0.5, y: 0, z: 0 }), w2 = A.placePart('wheel', { x: 0.9, y: 0, z: 0 }, { d: { r: 0.2, t: 0.05 } });
    A.mateStart({ pid: w2.id, p: [0, 0, 0], n: [0, 0, -1] }, { pid: m.id, p: A.DEFS.motor.pts(A.part(m.id).d)[0].p, n: [0, 0, 1] }, { type: 'fixed' }); out.matePv = G.all(); A.mateCommit(); out.mate = G.all();
    out.gap = A.editGap() * 1000; return out;
  });
  check('轮·铰链↔大地 / 位置 Y=0（轮心在地面）/ 🧲 大轮配到电机输出轴（预览+确定）→ 都在地面以上', R.misc.fieldOk && R.misc.hinge >= TOL && R.misc.field >= TOL && R.misc.matePv >= TOL && R.misc.mate >= TOL && R.misc.gap < 0.5, JSON.stringify(R.misc));
  // ---------- 4. resize the wheel on the car (R 140 → 260) live + commit ----------
  R.rs = await p.evaluate(() => { const A = app, G = A._gw; G.clear(); const c = G.car(); A.resizeLive(c.ids.w, { r: 0.2 }); const live = G.all(); A.resizeCommit(c.ids.w, { r: 0.26 }); return { live, lo: G.all(), gap: A.editGap() * 1000 }; });
  check('轮·改大轮半径（实时预览 + 确定）→ 整车抬升，不入地', R.rs.live >= TOL && R.rs.lo >= TOL && R.rs.gap < 0.5, JSON.stringify(R.rs));
  // ---------- 5. run → pause → 将当前运行状态设为初始状态 → 重置 (contacts sink into the floor while running) ----------
  R.run = await p.evaluate(() => {
    const A = app, G = A._gw; G.clear(); const c = G.car(); const w = G.bigWheel(-0.5, 0.4, 0.12); const g = A.placePart('gear', { x: -0.5, y: 0, z: -0.3 }, { d: { m: 0.004, z: 40, t: 0.02 } });
    A.startRun(); A.simulate(2.5); const inRun = G.model().lo; A.pause(); A.setInitialFromRun(); const initLo = (() => { const M = JSON.parse(A.init); let lo = Infinity; for (const P of M.parts) lo = Math.min(lo, A._groundMinY ? A._groundMinY(P) : 0); return lo; })();
    A.reset(); const afterReset = G.all(); A.startRun(); A.simulate(0.5); A.reset(); const reset2 = G.all();
    return { inRun, afterReset, reset2, mode: A.mode, net: (A.groundLog || []).length };
  });
  check('轮·运行 → 暂停 → “将当前运行状态设为初始状态” → 重置：回到编辑模式后所有零件都在地面以上（运行中轮/齿轮压入地面几毫米不会被带回编辑）', R.run.mode === 'edit' && R.run.afterReset >= TOL && R.run.reset2 >= TOL, JSON.stringify(R.run));
  // ---------- 6. palette drop (real mouse) of a large wheel rotated with the mouse wheel during the drag ----------
  await p.evaluate(() => { const A = app; A._gw.clear(); A.view('iso'); A.cam.position.set(0.6, 0.5, 0.8); A.orbit.target.set(0, 0.05, 0); A.orbit.update(); });
  await frame();
  const cv = await (await p.$('#cv')).boundingBox(), box = await (await p.$('.item[data-type="wheel"]')).boundingBox();
  await p.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await p.mouse.down(); await p.mouse.move(cv.x + cv.width / 2 - 30, cv.y + cv.height / 2 + 20, { steps: 6 });
  for (let i = 0; i < 3; i++) await p.mouse.wheel({ deltaY: 100 });
  await p.mouse.move(cv.x + cv.width / 2, cv.y + cv.height / 2, { steps: 4 }); await frame();
  R.palGhost = await p.evaluate(() => ({ lo: app._gw.drawn(), ghost: !!(app.ghostObj && app.ghostObj.parent) }));
  await p.mouse.up(); await frame();
  R.pal = await p.evaluate(() => { const A = app, P = A.M.parts[A.M.parts.length - 1]; return { type: P && P.type, lo: A._gw.all(), tilt: P ? 2 * Math.acos(Math.min(1, Math.abs(P.q[3]))) * 180 / Math.PI : 0 }; });
  check('轮·零件库拖入轮并在拖动中用滚轮旋转 45° → 半透明预览与放下后都贴地、不入地', R.palGhost.ghost && R.palGhost.lo >= TOL && R.pal.type === 'wheel' && R.pal.lo >= TOL && R.pal.lo < 0.002 && R.pal.tilt > 10, JSON.stringify({ g: R.palGhost, p: R.pal }));
  // ---------- 7. safety net: a pose written below the floor by any path is lifted on the next rebuild (and logged) ----------
  R.net = await p.evaluate(() => { const A = app, G = A._gw; G.clear(); const c = G.car(); const n0 = (A.groundLog || []).length; A.part(c.ids.w).p[1] -= 0.2; A.part(c.ids.pl).p[1] -= 0.2; A.part(c.ids.m).p[1] -= 0.2; A.part(c.ids.s).p[1] -= 0.2; A.part(c.ids.bv).p[1] -= 0.2; A.rebuild(); return { lo: G.all(), logged: (A.groundLog || []).length - n0, gap: A.editGap() * 1000 }; });
  check('轮·兜底：任何途径把位姿写到地面下（直接改模型）→ 下一次重建即整体抬升并记录日志（console.warn）', R.net.lo >= TOL && R.net.logged === 1 && R.net.gap < 0.5, JSON.stringify(R.net));
  // ---------- screenshot: the user's scene, all on the ground ----------
  await p.evaluate(() => {
    const A = app, G = A._gw; G.clear(); const c = G.car();
    const w = G.bigWheel(-0.4, 0.3, 0.14); const ent = A.asmEnts(c.ids.pl).find(e => /侧面1\(\+Z\)/.test(e.name)); G.pair(c.ids.pl, w.id, { ka: ent.key, kb: A.asmEnts(w.id)[0].key, type: 'revolute' });
    A.select(null); A.flash = null; const t = document.getElementById('gToast'); if (t) t.style.opacity = '0'; A.view('iso'); A.fit(); A.cam.position.sub(A.orbit.target).applyAxisAngle(new G.V(0, 1, 0), 0.5).multiplyScalar(0.9).add(A.orbit.target); A.orbit.update();
    A.msg('板+电机+轴+锥齿轮+大轮（R140，装配面板），另一大轮铰接到板侧面：所有零件几何最低点 = ' + (G.all() * 1000).toFixed(2) + ' mm（地面 y = 0）');
  });
  await new Promise(r => setTimeout(r, 400)); await frame();
  await p.screenshot({ path: OUT + 'screenshot-ground-fix2.png' }); console.log('saved screenshot-ground-fix2.png');
  await p.evaluate(() => app._gw.clear());
};
