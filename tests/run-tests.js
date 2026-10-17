// Headless Chrome test-suite for mech-sandbox-3d. Run: node tests/run-tests.js
const puppeteer = require('/workspace/tools/node_modules/puppeteer-core');
const OUT = '/workspace/mech-sandbox-3d/';
(async () => {
  const b = await puppeteer.launch({ executablePath: '/usr/bin/google-chrome', headless: 'new', args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--window-size=1500,900'] });
  const p = await b.newPage(); await p.setViewport({ width: 1500, height: 900 });
  const errs = []; p.on('pageerror', e => errs.push('PAGEERR ' + e.message)); p.on('console', m => { if (m.type() === 'error') errs.push('CONSOLE ' + m.text()); });
  const url = 'file://' + OUT + 'index.html';
  await p.goto(url); await p.waitForFunction('window.app', { timeout: 60000 });
  const res = [];
  const check = (name, ok, info) => { res.push({ name, ok, info }); console.log((ok ? 'PASS ' : 'FAIL ') + name + '  ' + info); };
  check('opens via file:// URL, app initialised', true, url);
  const R = await p.evaluate(() => {
    const A = app, out = {};
    const pre = n => { const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === n) - 1; A.loadPreset(i); };
    const body = n => A.sim.pb[A.byName(n).id].body;
    const w = (n, ax) => { const v = body(n).angvel(); return v.x * ax[0] + v.y * ax[1] + v.z * ax[2]; };
    // pendulum
    pre('单摆'); A.startRun(); { const piv = 0; let prev = null; const cr = []; for (let i = 0; i < 240 * 12; i++) { A.sim.substep(); const x = body('摆锤').translation().x - piv; if (prev !== null && prev > 0 && x <= 0) cr.push(A.sim.time); prev = x; }
      out.pend = { T: (cr[cr.length - 1] - cr[0]) / (cr.length - 1), ideal: 2 * Math.PI * Math.sqrt(1 / 9.81), n: cr.length, gap: A.sim.maxGap() }; } A.reset();
    // crank slider
    pre('曲柄滑块'); A.startRun(); { A.simulate(1); let lo = 1e9, hi = -1e9; for (let i = 0; i < 240 * 3; i++) { A.sim.substep(); const x = body('滑块').translation().x; lo = Math.min(lo, x); hi = Math.max(hi, x); } out.crank = { stroke: hi - lo, rpm: w('曲柄', [0, 0, 1]) * 30 / Math.PI }; } A.reset();
    // gears
    pre('两级齿轮减速器'); A.startRun(); A.simulate(2); { let s = 0, s2 = 0, n = 0; for (let i = 0; i < 240; i++) { A.sim.substep(); s += w('齿轮2 z40', [0, 0, 1]) / w('齿轮1 z20', [0, 0, 1]); s2 += w('齿轮4 z40', [0, 0, 1]) / w('齿轮1 z20', [0, 0, 1]); n++; } out.gear = { r12: s / n, r14: s2 / n, rpm1: w('齿轮1 z20', [0, 0, 1]) * 30 / Math.PI }; } A.reset();
    pre('锥齿轮传动'); A.startRun(); A.simulate(2); out.bevel = { r: w('大锥齿轮 z30', [1, 0, 0]) / w('小锥齿轮 z20', [0, 0, 1]) }; A.reset();
    pre('蜗杆蜗轮'); A.startRun(); A.simulate(2); out.worm = { r: w('蜗轮 z30', [0, 0, 1]) / w('蜗杆', [1, 0, 0]), rpmWorm: w('蜗杆', [1, 0, 0]) * 30 / Math.PI }; A.reset();
    pre('皮带传动'); A.startRun(); A.simulate(2); out.belt = { r: w('从动带轮', [0, 0, 1]) / w('主动带轮', [0, 0, 1]) }; A.reset();
    pre('链传动'); A.startRun(); A.simulate(2); out.chain = { r: w('大链轮 z32', [0, 0, 1]) / w('小链轮 z16', [0, 0, 1]) }; A.reset();
    pre('齿轮齿条'); A.startRun(); A.simulate(1); out.rack = { v: body('齿条').linvel().x, wr: w('小齿轮 z20', [0, 0, 1]) * 0.04 }; A.reset();
    pre('丝杠螺母'); A.startRun(); A.simulate(2); out.screw = { v: body('螺母').linvel().x, ideal: 0.005 * w('丝杠', [1, 0, 0]) / (2 * Math.PI) }; A.reset();
    pre('凸轮-从动件'); A.startRun(); A.simulate(1); { let lo = 1e9, hi = -1e9; for (let i = 0; i < 240 * 4; i++) { A.sim.substep(); const y = body('从动件').translation().y; lo = Math.min(lo, y); hi = Math.max(hi, y); } out.cam = { stroke: hi - lo }; } A.reset();
    pre('简易小车'); A.startRun(); { const x0 = body('车架').translation().x; A.simulate(5); const t = body('车架').translation(); out.cart = { dx: t.x - x0, dz: t.z, y: t.y, v: body('车架').linvel().x, ideal: 60 / 60 * 2 * Math.PI * 0.06 }; } A.reset();
    pre('投石机'); A.startRun(); { let vm = 0; const x0 = body('石块').translation().x; for (let i = 0; i < 240 * 3; i++) { A.sim.substep(); const v = body('石块').linvel(); vm = Math.max(vm, Math.hypot(v.x, v.y, v.z)); } out.catapult = { vmax: vm, dx: body('石块').translation().x - x0, gap: A.sim.maxGap() }; } A.reset();
    // 15 s gap test on every preset
    out.gaps = []; const opts = [...document.querySelectorAll('#selPreset option')].slice(1);
    opts.forEach((o, i) => { A.loadPreset(i); A.startRun(); let g = 0; for (let k = 0; k < 15; k++) { A.simulate(1); g = Math.max(g, A.sim.maxGap()); } out.gaps.push([o.textContent, g * 1000]); A.reset(); });
    // edit drag keeps connections
    pre('四连杆'); { const c = A.byName('连杆'), c0 = c.p.slice(); const gp = A.editDrag(c.id, [c.p[0] + 0.01, c.p[1] + 0.03, c.p[2]]); out.drag4 = { gapAfter: gp * 1000, moved: Math.hypot(A.byName('连杆').p[0] - c0[0], A.byName('连杆').p[1] - c0[1]) }; }
    pre('单摆'); { const bl = A.byName('摆锤'); const g = A.editDrag(bl.id, [bl.p[0] + 0.25, bl.p[1] + 0.05, 0]); out.dragPend = { gap: g * 1000, ballx: A.byName('摆锤').p[0] }; }
    pre('简易小车'); { const ch = A.byName('车架'); const x0 = A.byName('左前轮').p[0]; const g = A.editDrag(ch.id, [ch.p[0] + 0.3, ch.p[1], ch.p[2] - 0.2]); out.dragCart = { gap: g * 1000, wheelMoved: A.byName('左前轮').p[0] - x0 }; }
    // reset / initial state
    pre('单摆'); { const p0 = A.byName('摆锤').p.slice(); A.startRun(); A.simulate(0.7); A.pause(); A.reset(); const p1 = A.byName('摆锤').p.slice(); out.reset1 = { d: Math.hypot(p1[0] - p0[0], p1[1] - p0[1]), mode: A.mode };
      A.startRun(); A.simulate(0.5); A.pause(); A.setInitialFromRun(); const ps = A.byName('摆锤').p.slice(); A.startRun(); A.simulate(0.6); A.reset(); const p2 = A.byName('摆锤').p.slice(); out.reset2 = { d: Math.hypot(p2[0] - ps[0], p2[1] - ps[1]), mode: A.mode, movedFromOrig: Math.hypot(ps[0] - p0[0], ps[1] - p0[1]) };
      A.startRun(); out.reset2.v0 = Math.hypot(...Object.values(body('摆锤').linvel())); A.reset(); }
    // edits update initial state; add part, run, reset keeps it
    { const n0 = A.M.parts.length; A.placePart('block', { x: 0.6, y: 0, z: 0.3 }); A.startRun(); A.simulate(0.3); A.reset(); out.editInit = { before: n0, after: A.M.parts.length, mode: A.mode };
      A.undo(); const nu = A.M.parts.length; A.redo(); out.undo = { afterUndo: nu, afterRedo: A.M.parts.length }; }
    // build from scratch through API: rod + base + revolute, ball fixed -> run
    { document.getElementById('btnClear').click(); const base = A.placePart('base', { x: 0, y: 0, z: 0 }); const rod = A.placePart('rod', { x: 0.3, y: 0, z: 0.2 });
      const D = A.part(base.id); const j = A.makeJointBetween('revolute', A.part(base.id), { pt: { p: [0, D.d.h / 2, 0], a: [0, 0, 1], n: '顶部' } }, A.part(rod.id), { pt: { p: [-0.15, 0, 0], a: [0, 0, 1], n: '端孔A' } });
      const g0 = A.editGap(); A.startRun(); A.simulate(3); out.scratch = { gapEdit: g0 * 1000, gapRun: A.sim.maxGap() * 1000, rodMoved: A.sim.partPose(rod.id).p.y < A.part(rod.id).p[1] - 0.01, joints: A.M.joints.length }; A.reset(); }
    // save / load / bom
    pre('两级齿轮减速器'); { const s = JSON.stringify({ model: A.M }); A.loadJSON(s); out.io = { parts: A.M.parts.length, couplings: A.M.couplings.length, bomLines: A.bomCSV().split('\r\n').length, bomHead: A.bomCSV().slice(1, 30) }; }
    return out;
  });
  const pc = (a, b) => Math.abs(a - b) / Math.abs(b) * 100;
  check('单摆周期 vs 2π√(L/g)', pc(R.pend.T, R.pend.ideal) < 2, `T=${R.pend.T.toFixed(4)}s ideal=${R.pend.ideal.toFixed(4)}s err=${pc(R.pend.T, R.pend.ideal).toFixed(2)}% (${R.pend.n} crossings)`);
  check('曲柄滑块行程 ≈ 2r = 200mm', pc(R.crank.stroke, 0.2) < 2, `stroke=${(R.crank.stroke * 1000).toFixed(2)}mm crank=${R.crank.rpm.toFixed(2)}rpm`);
  check('齿轮 z20→z40 速比 -0.5 (1%)', pc(R.gear.r12, -0.5) < 1, `ω2/ω1=${R.gear.r12.toFixed(5)}, 总速比 ω4/ω1=${R.gear.r14.toFixed(5)} (理论0.25), 输入${R.gear.rpm1.toFixed(1)}rpm`);
  check('锥齿轮 90° 速比 |20/30|', pc(Math.abs(R.bevel.r), 2 / 3) < 1, `ω2/ω1=${R.bevel.r.toFixed(5)} (轴线 Z→X)`);
  check('蜗杆蜗轮 1/30', pc(Math.abs(R.worm.r), 1 / 30) < 1, `ω轮/ω杆=${R.worm.r.toFixed(6)} 蜗杆${R.worm.rpmWorm.toFixed(1)}rpm`);
  check('皮带速比 +0.5 (同向)', pc(R.belt.r, 0.5) < 1, `ω2/ω1=${R.belt.r.toFixed(5)}`);
  check('链传动速比 +0.5', pc(R.chain.r, 0.5) < 1, `ω2/ω1=${R.chain.r.toFixed(5)}`);
  check('齿轮齿条 v = ω·r', pc(Math.abs(R.rack.v), Math.abs(R.rack.wr)) < 1, `v=${R.rack.v.toFixed(5)} ω·r=${R.rack.wr.toFixed(5)}`);
  check('丝杠 v = 导程·ω/2π', pc(Math.abs(R.screw.v), Math.abs(R.screw.ideal)) < 1, `v=${(R.screw.v * 1000).toFixed(3)}mm/s ideal=${(R.screw.ideal * 1000).toFixed(3)}mm/s`);
  check('凸轮升程 ≈ 30mm', pc(R.cam.stroke, 0.03) < 5, `stroke=${(R.cam.stroke * 1000).toFixed(2)}mm`);
  check('小车前进', R.cart.dx > 1.0, `5s 位移 ${R.cart.dx.toFixed(3)}m, v=${R.cart.v.toFixed(3)}m/s (纯滚动理论 ${R.cart.ideal.toFixed(3)}), 横偏 ${(R.cart.dz * 1000).toFixed(1)}mm`);
  check('投石机抛出', R.catapult.vmax > 2, `石块最大速度 ${R.catapult.vmax.toFixed(2)}m/s, 水平位移 ${R.catapult.dx.toFixed(2)}m`);
  for (const [n, g] of R.gaps) check('15s 后连接间隙 <1mm: ' + n, g < 1, `max gap ${g.toFixed(4)}mm`);
  check('编辑拖动(四连杆连杆)保持连接', R.drag4.gapAfter < 1 && R.drag4.moved > 0.005, `gap=${R.drag4.gapAfter.toFixed(4)}mm moved=${(R.drag4.moved * 1000).toFixed(1)}mm`);
  check('编辑拖动(单摆摆锤)保持连接', R.dragPend.gap < 1, `gap=${R.dragPend.gap.toFixed(4)}mm ball x=${R.dragPend.ballx.toFixed(3)}`);
  check('编辑拖动(小车整体)保持连接', R.dragCart.gap < 1 && Math.abs(R.dragCart.wheelMoved - 0.3) < 1e-6, `gap=${R.dragCart.gap.toFixed(4)}mm wheel moved=${R.dragCart.wheelMoved.toFixed(3)}`);
  check('重置回到初始状态(编辑模式)', R.reset1.d < 1e-9 && R.reset1.mode === 'edit', JSON.stringify(R.reset1));
  check('将当前运行状态设为初始状态 → 重置回到该姿态', R.reset2.d < 1e-9 && R.reset2.movedFromOrig > 0.01 && R.reset2.v0 < 1e-9, JSON.stringify(R.reset2));
  check('编辑更新初始状态（放置零件后运行再重置仍在）', R.editInit.after === R.editInit.before + 1 && R.editInit.mode === 'edit', JSON.stringify(R.editInit));
  check('撤销/重做', R.undo.afterUndo === R.editInit.before && R.undo.afterRedo === R.editInit.after, JSON.stringify(R.undo));
  check('自由装配：底座+连杆+铰链 → 运行', R.scratch.gapEdit < 1 && R.scratch.gapRun < 1 && R.scratch.rodMoved, JSON.stringify(R.scratch));
  check('保存/读取/BOM', R.io.parts === 9 && R.io.couplings === 2 && R.io.bomLines > 12, JSON.stringify(R.io));
  // ---- real mouse interactions: palette drag-drop + tool clicks ----
  await p.evaluate(() => document.getElementById('btnClear').click());
  const box = await (await p.$('.item[data-type="wheel"]')).boundingBox(); const cv = await (await p.$('#cv')).boundingBox();
  await p.mouse.move(box.x + 20, box.y + 10); await p.mouse.down(); await p.mouse.move(cv.x + cv.width / 2, cv.y + cv.height / 2, { steps: 8 }); await p.mouse.up();
  const nAfterDrop = await p.evaluate(() => app.M.parts.length);
  check('鼠标从零件库拖入场景', nAfterDrop === 1, 'parts=' + nAfterDrop);
  await p.keyboard.press('KeyE'); const mode = await p.evaluate(() => app.tc.mode); await p.keyboard.press('KeyW');
  check('快捷键 W/E 切换移动/旋转', mode === 'rotate', 'mode after E=' + mode);
  await p.keyboard.press('Space'); const m1 = await p.evaluate(() => app.mode); await p.keyboard.press('Space'); const m2 = await p.evaluate(() => app.mode);
  await p.keyboard.press('Escape'); await p.evaluate(() => app.reset());
  await p.evaluate(() => app.select({ p: app.M.parts[0].id })); await p.keyboard.press('Delete'); const nDel = await p.evaluate(() => app.M.parts.length);
  await p.keyboard.down('Control'); await p.keyboard.press('KeyZ'); await p.keyboard.up('Control'); const nUndo = await p.evaluate(() => app.M.parts.length);
  check('快捷键 Space/Delete/Ctrl+Z', m1 === 'run' && m2 === 'pause' && nDel === 0 && nUndo === 1, `space→${m1},${m2} del→${nDel} undo→${nUndo}`);
  // ---- direct snap-to-assemble (real mouse) ----
  await require('./assembly.js')(p, check, OUT);
  await require('./joints.js')(p, check, OUT);
  await require('./outliner.js')(p, check, OUT);
  await require('./resize.js')(p, check, OUT);
  await require('./controls.js')(p, check, OUT);
  await require('./features.js')(p, check, OUT);
  // screenshots
  const shot = async (name, js, t = 0, arg) => { await p.evaluate(js, arg); if (t) await new Promise(r => setTimeout(r, t)); await p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))); await p.screenshot({ path: OUT + name }); console.log('saved ' + name); };
  await shot('screenshot-edit-gizmo.png', () => { const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '四连杆') - 1; app.loadPreset(i); app.view('iso'); app.fit(); app.select({ p: app.byName('连杆').id }); }, 300);
  await shot('screenshot-gear-reducer-run.png', () => { const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '两级齿轮减速器') - 1; app.loadPreset(i); app.view('iso'); app.fit(); app.select({ p: app.byName('齿轮4 z40').id }); app.startRun(); app.simulate(1.5); app.updRun(); }, 1500);
  await shot('screenshot-cart-run.png', () => { app.reset(); const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '简易小车') - 1; app.loadPreset(i); app.view('iso'); app.fit(); app.startRun(); app.simulate(1.5); app.updRun(); const c = app.sim.partPose(app.byName('车架').id).p; const d = app.cam.position.clone().sub(app.orbit.target); app.orbit.target.copy(c); app.cam.position.copy(c).add(d.multiplyScalar(1.4)); app.orbit.update(); }, 300);
  await shot('screenshot-crank-slider-run.png', () => { app.reset(); const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '曲柄滑块') - 1; app.loadPreset(i); app.view('front'); app.fit(); app.startRun(); app.simulate(1.2); app.updRun(); }, 1500);
  for (const [n, nm] of [['锥齿轮传动', 'bevel'], ['蜗杆蜗轮', 'worm'], ['皮带传动', 'belt'], ['链传动', 'chain'], ['凸轮-从动件', 'cam'], ['丝杠螺母', 'screw'], ['投石机', 'catapult'], ['单摆', 'pendulum']])
    await shot('screenshot-' + nm + '.png', (n) => { app.reset(); const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === n) - 1; app.loadPreset(i); app.view('iso'); app.fit(); app.startRun(); app.simulate(n === '投石机' ? 0.6 : 1); app.updRun(); }, 600, n).catch(e => console.log(e.message));
  check('无控制台错误', errs.length === 0, errs.join(' | ') || 'none');
  const fs = require('fs'); fs.writeFileSync(OUT + 'tests/test-results.txt', res.map(r => (r.ok ? 'PASS ' : 'FAIL ') + r.name + '  ' + r.info).join('\n') + `\n\n${res.filter(r => r.ok).length}/${res.length} passed\n`);
  console.log(`${res.filter(r => r.ok).length}/${res.length} passed`);
  await b.close();
})();
