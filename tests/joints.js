// Joint editing, context menu, Tab type cycling, wheel / middle-button rotation, auto-orientation,
// hand-built tricycle and preset connection audit. Real mouse/keyboard events wherever feasible. Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const scr = w => p.evaluate(w => { const s = app.toScreen(new app._V(w[0], w[1], w[2])); return [s.x, s.y]; }, w);
  const setup = async (pos, tgt) => { await p.evaluate((pos, tgt) => { if (app.mode !== 'edit') app.reset(); document.getElementById('btnClear').click(); app.cam.position.set(...pos); app.orbit.target.set(...tgt); app.orbit.update(); app.select(null); app.rotAxis = 'view'; }, pos, tgt); await frame(); };
  const dropAt = async (type, xy, beforeUp) => {
    const box = await (await p.$(`.item[data-type="${type}"]`)).boundingBox();
    await p.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await p.mouse.down();
    await p.mouse.move(xy[0] - 30, xy[1] + 25, { steps: 6 }); await p.mouse.move(xy[0], xy[1], { steps: 4 }); await frame();
    if (beforeUp) await beforeUp();
    await p.mouse.up(); await frame();
    return p.evaluate(() => app.M.parts[app.M.parts.length - 1].id);
  };
  const mdrag = async (from, to, steps = 12, button = 'left', mods = []) => { for (const m of mods) await p.keyboard.down(m); await p.mouse.move(from[0], from[1]); await p.mouse.down({ button }); await p.mouse.move(to[0], to[1], { steps }); await frame(); await p.mouse.up({ button }); for (const m of mods) await p.keyboard.up(m); await frame(); };
  const conn = (id, name) => p.evaluate((id, name) => app._conn(app.part(id), name), id, name);
  const rodAng = rod => p.evaluate(rod => { const R = app.part(rod), a = app._conn(R, '端孔A'), b = app._conn(R, '端孔B'); return Math.atan2(b[1] - a[1], b[0] - a[0]) * 180 / Math.PI; }, rod);
  const jointOf = (a, b) => p.evaluate((a, b) => { const j = app.M.joints.find(j => (j.a === a && j.b === b) || (j.a === b && j.b === a)); return j ? { id: j.id, type: j.type, mod: !!j.mod } : null; }, a, b);

  // ---------- pendulum built by dragging ----------
  await setup([0.25, 0.45, 1.5], [0.15, 0.35, 0]);
  const base = await dropAt('base', await scr([0, 0, 0]));
  const rod = await dropAt('rod', (await scr(await conn(base, '轴销(前)'))).map((v, i) => v + (i ? 6 : -5)));
  const j0 = await jointOf(base, rod);
  // auto-orientation: rod hole axis parallel to the pin axis
  const ori = await p.evaluate((rod, base) => { const R = app.part(rod), B = app.part(base), Q = app.cam.quaternion.constructor; const zr = new app._V(0, 0, 1).applyQuaternion(new Q(...R.q)), zb = new app._V(0, 0, 1).applyQuaternion(new Q(...B.q)); return Math.abs(zr.dot(zb)); }, rod, base);
  // 1. select the rod -> property panel lists its connection; change hinge -> fixed in the dropdown
  await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  const panel = await p.evaluate(() => ({ hdr: (document.getElementById('connHdr') || {}).textContent, cards: document.querySelectorAll('#props .jcard').length, type: (document.querySelector('#props .jtype') || {}).value }));
  await p.select('#props .jtype', 'fixed'); await frame();
  const jf = await jointOf(base, rod);
  const a0 = await rodAng(rod); const endB = await scr(await conn(rod, '端孔B'));
  await mdrag(endB, [endB[0] - 60, endB[1] + 90]);
  const a1 = await rodAng(rod);
  const runFixed = await p.evaluate(rod => { app.startRun(); const y0 = app.sim.partPose(rod).p.y; app.simulate(1.5); const y1 = app.sim.partPose(rod).p.y; app.reset(); return Math.abs(y1 - y0) * 1000; }, rod);
  await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  const marked = await p.evaluate(() => !!document.querySelector('#props .jmod'));
  check('连接编辑：属性面板“连接”列表 → 铰链改为固定连接（拖动/运行不再转动）', j0 && j0.type === 'revolute' && panel.cards === 1 && panel.type === 'revolute' && jf.type === 'fixed' && Math.abs(a1 - a0) < 0.5 && runFixed < 0.5 && marked,
    JSON.stringify({ panel, before: j0 && j0.type, after: jf.type, dragAngChange: +(a1 - a0).toFixed(3), runMove_mm: +runFixed.toFixed(4), 已修改: marked }));
  check('吸附时自动对齐轴线：连杆孔轴 ∥ 轴销轴', ori > 0.9999, 'dot=' + ori.toFixed(6));
  // screenshot of the joint panel
  await p.evaluate(() => { const d = document.querySelector('#props .jdet'); if (d) d.open = true; }); await frame();
  await p.screenshot({ path: OUT + 'screenshot-joint-panel.png' });
  // undo restores the hinge, redo fixes again
  const ur = await p.evaluate((a, b) => { const t = () => app.M.joints.find(j => [j.a, j.b].includes(a) && [j.a, j.b].includes(b)).type; app.undo(); app.undo(); const u = t(); app.redo(); const r = t(); app.undo(); return { u, r, now: t() }; }, base, rod);
  // 2. change back to hinge via the dropdown: rotates again when dragged
  await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  await p.select('#props .jtype', 'revolute'); await frame();
  const jr = await jointOf(base, rod); const b0 = await rodAng(rod); const eb = await scr(await conn(rod, '端孔B'));
  await mdrag(eb, [eb[0] - 60, eb[1] + 110], 12, 'left', ['Control']); const b1 = await rodAng(rod);
  check('撤销/重做 每次连接修改 + 改回铰链后可再次转动', ur.u === 'revolute' && ur.r === 'fixed' && jr.type === 'revolute' && Math.abs(b1 - b0) > 15, JSON.stringify({ ur, back: jr.type, dragAngChange: +(b1 - b0).toFixed(2) }));
  // 3. change axis to Y and flip, via panel
  await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  await p.select('#props .jax', 'Y'); await frame();
  const axY = await p.evaluate((a, b) => { const j = app.M.joints.find(j => [j.a, j.b].includes(a) && [j.a, j.b].includes(b)), A = app.part(j.a); return new app._V(...j.axa).applyQuaternion(new app.cam.quaternion.constructor(...A.q)).toArray(); }, base, rod);
  await (await p.$('#props .jflip')).click(); await frame();
  const axF = await p.evaluate((a, b) => { const j = app.M.joints.find(j => [j.a, j.b].includes(a) && [j.a, j.b].includes(b)), A = app.part(j.a); return new app._V(...j.axa).applyQuaternion(new app.cam.quaternion.constructor(...A.q)).toArray(); }, base, rod);
  check('连接编辑：改轴线 (Y) 与反转', Math.abs(axY[1]) > 0.9999 && Math.abs(axF[1] + axY[1]) < 1e-6, JSON.stringify({ axY: axY.map(v => +v.toFixed(4)), flipped: axF.map(v => +v.toFixed(4)) }));
  await p.evaluate(() => { app.undo(); app.undo(); });
  // limits + joint drive via panel (drive the hinge at 60 rpm, check)
  await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  await p.evaluate(() => { const d = document.querySelector('#props .jdet'); if (d) d.open = true; }); await frame();
  await p.click('#props .jdon'); await frame();
  await p.evaluate(() => { const i = document.querySelector('#props .jdv'); i.value = 60; i.dispatchEvent(new Event('change')); }); await frame();
  const drv = await p.evaluate(rod => { app.startRun(); app.simulate(1); const b = app.sim.pb[rod].body, w = b.angvel(); const rpm = Math.hypot(w.x, w.y, w.z) * 30 / Math.PI; app.reset(); return rpm; }, rod);
  await p.evaluate(() => { app.undo(); app.undo(); });
  await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  await p.evaluate(() => { const d = document.querySelector('#props .jdet'); if (d) d.open = true; }); await frame();
  await p.click('#props .jlimon'); await frame();
  const lim = await p.evaluate(rod => { app.startRun(); let lo = 1e9, hi = -1e9; for (let i = 0; i < 240 * 3; i++) { app.sim.substep(); const ps = app.sim.partPose(rod), R = app.part(rod); const e = new app._V(R.d.L / 2, 0, 0).applyQuaternion(ps.q); const a = Math.atan2(e.y, e.x) * 180 / Math.PI; lo = Math.min(lo, a); hi = Math.max(hi, a); } app.reset(); return { lo, hi }; }, rod);
  await p.evaluate(() => app.undo());
  check('连接编辑：铰链驱动 60rpm、限位 ±45°', Math.abs(drv - 60) < 1 && lim.hi - lim.lo < 92, JSON.stringify({ rpm: +drv.toFixed(2), swing_deg: [+lim.lo.toFixed(1), +lim.hi.toFixed(1)] }));
  // 4. right-click context menu on the rod -> 改为固定
  const mid = await scr(await conn(rod, '中点'));
  await p.mouse.click(mid[0], mid[1], { button: 'right' }); await new Promise(r => setTimeout(r, 150)); await frame();
  const menu = await p.evaluate(() => { const m = document.getElementById('ctx'); return { vis: m.style.display, items: [...m.querySelectorAll('.ci')].map(x => x.textContent) }; });
  await p.screenshot({ path: OUT + 'screenshot-context-menu.png' });
  const fixItem = await p.evaluateHandle(() => [...document.querySelectorAll('#ctx .ci')].find(x => x.textContent === '改为固定'));
  await fixItem.asElement().click(); await frame();
  const jc = await jointOf(base, rod);
  // right-click on the joint marker -> 删除连接
  const jm = await p.evaluate(id => { const j = app.M.joints.find(x => x.id === id), A = app.part(j.a); const w = new app._V(...j.la).applyQuaternion(new app.cam.quaternion.constructor(...A.q)).add(new app._V(...A.p)); return w.toArray(); }, jc.id);
  const jms = await scr(jm);
  await p.mouse.click(jms[0], jms[1], { button: 'right' }); await new Promise(r => setTimeout(r, 150)); await frame();
  const menu2 = await p.evaluate(() => [...document.querySelectorAll('#ctx .ch2')].map(x => x.textContent));
  const delItem = await p.evaluateHandle(() => [...document.querySelectorAll('#ctx .ci')].find(x => x.textContent === '删除连接'));
  await delItem.asElement().click(); await frame();
  const nj = await p.evaluate(() => app.M.joints.length);
  check('右键菜单：零件 → 改为固定；连接标记 → 删除连接', menu.vis === 'block' && menu.items.includes('改为铰链') === false && jc.type === 'fixed' && menu2.length === 1 && nj === 0, JSON.stringify({ items: menu.items, after: jc.type, markerMenu: menu2, jointsAfterDelete: nj }));
  // 5. panel delete button
  await p.evaluate(() => app.undo()); await p.evaluate(rod => app.select({ p: rod }), rod); await frame();
  await p.click('#props .jdel'); await frame();
  check('属性面板“删除连接”按钮', await p.evaluate(() => app.M.joints.length) === 0, 'joints=0');

  // ---------- Tab cycling of the joint type during a snap ----------
  await setup([0.12, 0.75, 1.3], [0.1, 0.5, 0]);
  const b2 = await dropAt('base', await scr([0, 0, 0]));
  const mot = await dropAt('motor', await scr(await conn(b2, '顶面')));
  let lblTab = '';
  const g1 = await dropAt('gear', await scr(await conn(mot, '输出轴')), async () => { await p.keyboard.press('Tab'); await frame(); lblTab = await p.evaluate(() => document.getElementById('snapLbl').textContent); });
  const jt = await jointOf(mot, g1);
  check('吸附时按 Tab 切换连接类型（标签显示可选项）', /Tab/.test(lblTab) && /\[固定连接\]/.test(lblTab) && jt && jt.type === 'fixed', JSON.stringify({ label: lblTab, joint: jt && jt.type }));

  // ---------- auto-orientation: gear onto a tilted motor, wheel onto a skewed shaft ----------
  await setup([0.3, 0.7, 1.4], [0, 0.3, 0]);
  const ao = await p.evaluate(() => { const E = (x, y, z) => new app.cam.quaternion.constructor().setFromEuler(new app.cam.rotation.constructor(x * Math.PI / 180, y * Math.PI / 180, z * Math.PI / 180)).toArray();
    app.beginEdit(); const m = app.newPart('motor', [-0.25, 0.3, 0], null, {}); m.q = E(25, 40, 10); const s = app.newPart('shaft', [0.25, 0.35, 0], null, { fixed: true }); s.q = E(-35, 60, 20); app.commit(); app.rebuild(); return { m: m.id, s: s.id }; });
  const gA = await dropAt('gear', await scr(await conn(ao.m, '输出轴')));
  const wA = await dropAt('wheel', await scr(await conn(ao.s, '轴身')));
  const orient = await p.evaluate((ao, gA, wA) => { const Q = app.cam.quaternion.constructor, z = id => new app._V(0, 0, 1).applyQuaternion(new Q(...app.part(id).q)); return { gear: Math.abs(z(gA).dot(z(ao.m))), wheel: Math.abs(z(wA).dot(z(ao.s))), gj: app.M.joints.filter(j => [j.a, j.b].includes(gA)).map(j => j.type), wj: app.M.joints.filter(j => [j.a, j.b].includes(wA)).map(j => j.type) }; }, ao, gA, wA);
  check('吸附自动转向：齿轮→倾斜电机轴、轮→任意方向的轴（轴线自动对齐）', orient.gear > 0.9999 && orient.wheel > 0.9999 && orient.gj[0] === 'revolute' && orient.wj[0] === 'fixed', JSON.stringify(orient));

  // ---------- mouse wheel rotation during a palette drag ----------
  await setup([0.0, 1.4, 0.6], [0, 0, 0]);
  let wl = '';
  const mr = await dropAt('motor', await scr([0.2, 0, 0.1]), async () => {
    for (let i = 0; i < 4 && !(await p.evaluate(() => app.rotAxis === 'Y')); i++) { await p.keyboard.down('Shift'); await p.mouse.wheel({ deltaY: 100 }); await p.keyboard.up('Shift'); await frame(); }
    for (let i = 0; i < 6; i++) { await p.mouse.wheel({ deltaY: 100 }); await frame(); }
    wl = await p.evaluate(() => document.getElementById('snapLbl').textContent);
  });
  const mq = await p.evaluate(id => { const q = new app.cam.quaternion.constructor(...app.part(id).q), z = new app._V(0, 0, 1).applyQuaternion(q), y = new app._V(0, 1, 0).applyQuaternion(q); return { z: z.toArray().map(v => +v.toFixed(4)), y: y.toArray().map(v => +v.toFixed(4)) }; }, mr);
  check('拖动时滚轮旋转 15°/格（Shift+滚轮换轴，标签显示轴和角度）', /Y/.test(wl) && /90°/.test(wl) && Math.abs(Math.abs(mq.z[0]) - 1) < 1e-3 && Math.abs(mq.y[1] - 1) < 1e-3, JSON.stringify({ label: wl, motorZ: mq.z }));
  // ---------- view controls: middle-drag anywhere (even on a part) orbits the camera; wheel zooms; right-drag pans ----------
  const cxy = await scr(await p.evaluate(id => app.part(id).p, mr));
  const st0 = await p.evaluate(id => ({ q: app.part(id).q.slice(), p: app.part(id).p.slice(), cam: app.cam.position.toArray(), n: app.undoS.length }), mr);
  await mdrag(cxy, [cxy[0] + 75, cxy[1] + 20], 10, 'middle'); await new Promise(r => setTimeout(r, 300)); await frame();
  const st1 = await p.evaluate(id => ({ q: app.part(id).q.slice(), p: app.part(id).p.slice(), cam: app.cam.position.toArray(), n: app.undoS.length }), mr);
  const dq = st0.q.reduce((a, v, i) => a + Math.abs(v - st1.q[i]), 0), dp = st0.p.reduce((a, v, i) => a + Math.abs(v - st1.p[i]), 0), dcam = Math.hypot(...st0.cam.map((v, i) => v - st1.cam[i]));
  check('中键在零件上拖动 = 旋转视角（零件不移动/不旋转）', dq < 1e-12 && dp < 1e-12 && dcam > 0.01 && st1.n === st0.n, JSON.stringify({ partRot: dq, partMove: dp, camMoved: +dcam.toFixed(3), undoEntries: st1.n - st0.n }));
  const c0 = await p.evaluate(() => app.cam.position.toArray());
  const cv = await (await p.$('#cv')).boundingBox();
  await mdrag([cv.x + 60, cv.y + cv.height - 60], [cv.x + 60, cv.y + cv.height - 160], 10, 'middle'); await new Promise(r => setTimeout(r, 300));
  const c1 = await p.evaluate(() => app.cam.position.toArray());
  check('中键拖空白处 = 旋转视角', Math.hypot(...c0.map((v, i) => v - c1[i])) > 0.01, 'camMoved=' + Math.hypot(...c0.map((v, i) => v - c1[i])).toFixed(3));
  const d0 = await p.evaluate(() => app.cam.position.distanceTo(app.orbit.target));
  await p.mouse.move(cv.x + cv.width / 2, cv.y + cv.height / 2); for (let i = 0; i < 3; i++) { await p.mouse.wheel({ deltaY: 120 }); await frame(); } await new Promise(r => setTimeout(r, 300));
  const d1 = await p.evaluate(() => app.cam.position.distanceTo(app.orbit.target));
  check('未拖动零件时滚轮 = 缩放', Math.abs(d1 - d0) / d0 > 0.05, `dist ${d0.toFixed(3)} → ${d1.toFixed(3)}`);
  const t0 = await p.evaluate(() => app.orbit.target.toArray());
  await mdrag([cv.x + 80, cv.y + 120], [cv.x + 200, cv.y + 180], 10, 'right'); await new Promise(r => setTimeout(r, 300));
  const t1 = await p.evaluate(() => ({ t: app.orbit.target.toArray(), ctx: getComputedStyle(document.getElementById('ctx')).display }));
  check('右键拖空白处 = 平移（不弹出右键菜单）', Math.hypot(...t0.map((v, i) => v - t1.t[i])) > 0.005 && t1.ctx === 'none', JSON.stringify({ pan: +Math.hypot(...t0.map((v, i) => v - t1.t[i])).toFixed(4), ctx: t1.ctx }));
  // R key rotates the selected part 15° (Shift+R reverses)
  await p.evaluate(id => app.select({ p: id }), mr);
  const qr0 = await p.evaluate(id => app.part(id).q.slice(), mr);
  await p.keyboard.press('KeyR'); await frame(); await p.keyboard.press('KeyR'); await frame();
  const rdeg = await p.evaluate((id, q0) => { const Q = app.cam.quaternion.constructor; return new Q(...q0).angleTo(new Q(...app.part(id).q)) * 180 / Math.PI; }, mr, qr0);
  await p.keyboard.down('Shift'); await p.keyboard.press('KeyR'); await p.keyboard.up('Shift'); await frame();
  const rdeg2 = await p.evaluate((id, q0) => { const Q = app.cam.quaternion.constructor; return new Q(...q0).angleTo(new Q(...app.part(id).q)) * 180 / Math.PI; }, mr, qr0);
  check('R 键旋转选中零件 15°/次（Shift+R 反向）', Math.abs(rdeg - 30) < 0.01 && Math.abs(rdeg2 - 15) < 0.01, JSON.stringify({ afterRR: +rdeg.toFixed(3), afterShiftR: +rdeg2.toFixed(3) }));
  // R on a hinged rod rotates it about the hinge
  await setup([0.25, 0.45, 1.5], [0.15, 0.35, 0]);
  const b3 = await dropAt('base', await scr([0, 0, 0]));
  const r3 = await dropAt('rod', (await scr(await conn(b3, '轴销(前)'))).map((v, i) => v + (i ? 6 : -5)));
  const ra0 = await rodAng(r3);
  await p.evaluate(r => app.select({ p: r }), r3); for (let i = 0; i < 3; i++) { await p.keyboard.press('KeyR'); await frame(); }
  const hinge = await p.evaluate((r, b) => { const h = app._conn(app.part(r), '端孔A'), pw = app._conn(app.part(b), '轴销(前)'); return { err: Math.hypot(h[0] - pw[0], h[1] - pw[1]) * 1000, gap: app.editGap() * 1000, joints: app.M.joints.length }; }, r3, b3);
  const ra1 = await rodAng(r3);
  check('R 键旋转带铰链的零件：绕铰链转动（不脱开）', Math.abs(Math.abs(ra1 - ra0) - 45) < 1 && hinge.err < 0.5 && hinge.gap < 1 && hinge.joints === 1, JSON.stringify({ dAng: +(ra1 - ra0).toFixed(2), ...hinge }));

  // ---------- tricycle built from scratch by dragging only ----------
  await setup([0.55, 0.75, 0.75], [0, 0.03, 0]);
  const plate = await dropAt('plate', await scr([0, 0, 0]));
  const top = await p.evaluate(id => { const P = app.part(id); return P.p[1] + P.d.h / 2; }, plate);
  const m1 = await dropAt('motor', await scr([0.1, top, 0.05]));
  const m2 = await dropAt('motor', await scr([0.1, top, -0.05]), async () => {
    for (let i = 0; i < 4 && !(await p.evaluate(() => app.rotAxis === 'Y')); i++) { await p.keyboard.down('Shift'); await p.mouse.wheel({ deltaY: 100 }); await p.keyboard.up('Shift'); await frame(); }
    for (let i = 0; i < 12; i++) { await p.mouse.wheel({ deltaY: 100 }); await frame(); }
  });
  const w1 = await dropAt('wheel', await scr(await conn(m1, '输出轴')));
  const w2 = await dropAt('wheel', await scr(await conn(m2, '输出轴')));
  const top2 = await p.evaluate(id => { const P = app.part(id); return P.p[1] + P.d.h / 2; }, plate);
  const br = await dropAt('bearing', await scr([-0.12, top2, 0.04]));
  const sh = await dropAt('shaft', await scr(await conn(br, '轴孔')));
  const shMid = await p.evaluate((sh, br) => { const S = app.part(sh), b = app._conn(app.part(br), '轴孔'), a = app._conn(S, '前端'), c = app._conn(S, '后端'); const far = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) > Math.hypot(c[0] - b[0], c[1] - b[1], c[2] - b[2]) ? a : c; return [(far[0] * 2 + b[0]) / 3, (far[1] * 2 + b[1]) / 3, (far[2] * 2 + b[2]) / 3]; }, sh, br);
  const w3 = await dropAt('wheel', await scr(shMid));
  // right motor faces the other way: reverse its drive with the context menu (反转轴) on its wheel
  const w2c = await scr(await p.evaluate(id => app.part(id).p, w2));
  await p.mouse.click(w2c[0], w2c[1], { button: 'right' }); await new Promise(r => setTimeout(r, 150)); await frame();
  const flip = await p.evaluateHandle(() => [...document.querySelectorAll('#ctx .ci')].find(x => x.textContent === '反转轴'));
  if (flip.asElement()) { await flip.asElement().click(); await frame(); }
  const built = await p.evaluate(ids => { const J = app.M.joints, has = (a, b, t) => J.some(j => j.type === t && [j.a, j.b].includes(a) && [j.a, j.b].includes(b));
    return { parts: app.M.parts.length, joints: J.length, m1: has(ids.plate, ids.m1, 'fixed'), m2: has(ids.plate, ids.m2, 'fixed'), w1: has(ids.m1, ids.w1, 'revolute'), w2: has(ids.m2, ids.w2, 'revolute'), br: has(ids.plate, ids.br, 'fixed'), sh: has(ids.br, ids.sh, 'revolute'), w3: has(ids.sh, ids.w3, 'fixed'), rev: !!J.find(j => [j.a, j.b].includes(ids.w2) && j.motorRev), audit: +(app.connAudit().maxGap * 1000).toFixed(3), motorsFree: !app.part(ids.m1).fixed && !app.part(ids.m2).fixed }; }, { plate, m1, m2, w1, w2, br, sh, w3 });
  const run = await p.evaluate(plate => { app.startRun(); const p0 = app.sim.partPose(plate).p.clone(); let gap = 0, minUp = 1; for (let k = 0; k < 50; k++) { app.simulate(0.1); gap = Math.max(gap, app.sim.maxGap()); minUp = Math.min(minUp, new app._V(0, 1, 0).applyQuaternion(app.sim.partPose(plate).q).y); } const p1 = app.sim.partPose(plate).p; const r = { dist: Math.hypot(p1.x - p0.x, p1.z - p0.z), gap_mm: gap * 1000, minUp, dy: p1.y - p0.y }; return r; }, plate);
  await p.evaluate(() => { app.updRun(); }); await frame();
  await p.evaluate(() => app.reset());
  check('纯拖拽搭建三轮车（板+2电机+2轮+轴承座+轴+前轮），全部为真实连接', built.m1 && built.m2 && built.w1 && built.w2 && built.br && built.sh && built.w3 && built.rev && built.motorsFree && built.audit < 1, JSON.stringify(built));
  check('手搭三轮车运行 5s：前进 >1m、间隙 <1mm、不翻车', run.dist > 1 && run.gap_mm < 1 && run.minUp > 0.95, JSON.stringify({ dist_m: +run.dist.toFixed(3), gap_mm: +run.gap_mm.toFixed(4), minUpY: +run.minUp.toFixed(4) }));

  // ---------- presets: tricycle + cart run, connection audit for every preset ----------
  const pre = await p.evaluate(() => { const out = {}; const opts = [...document.querySelectorAll('#selPreset option')].slice(1);
    out.audit = opts.map((o, i) => { app.loadPreset(i); const a = app.connAudit(); return [o.textContent, +(a.maxGap * 1000).toFixed(3), a.loose, a.joints.length]; });
    for (const n of ['三轮车', '简易小车']) { const i = opts.findIndex(o => o.textContent === n); app.loadPreset(i); app.startRun(); const ch = app.byName('车架').id, p0 = app.sim.partPose(ch).p.clone(); let gap = 0, minUp = 1; for (let k = 0; k < 50; k++) { app.simulate(0.1); gap = Math.max(gap, app.sim.maxGap()); minUp = Math.min(minUp, new app._V(0, 1, 0).applyQuaternion(app.sim.partPose(ch).q).y); } const p1 = app.sim.partPose(ch).p; out[n] = { dist: +Math.hypot(p1.x - p0.x, p1.z - p0.z).toFixed(3), lateral_mm: +((p1.z - p0.z) * 1000).toFixed(1), gap_mm: +(gap * 1000).toFixed(4), minUp: +minUp.toFixed(4) }; app.reset(); }
    const cart = opts.findIndex(o => o.textContent === '简易小车'); app.loadPreset(cart); const J = app.M.joints, nm = id => app.part(id).name, t = (a, b) => (J.find(j => [nm(j.a), j.b && nm(j.b)].includes(a) && [nm(j.a), j.b && nm(j.b)].includes(b)) || {}).type;
    out.cartJ = { 左电机: t('车架', '左电机'), 右电机: t('车架', '右电机'), 左后轮: t('左电机', '左后轮'), 右后轮: t('右电机', '右后轮'), 左轴承: t('车架', '左轴承座'), 右轴承: t('车架', '右轴承座'), 前轴L: t('左轴承座', '前轴'), 前轴R: t('右轴承座', '前轴'), 左前轮: t('前轴', '左前轮'), 右前轮: t('前轴', '右前轮') };
    return out; });
  check('三轮车预设：5s 前进 >1m、间隙 <1mm、不翻车', pre['三轮车'].dist > 1 && pre['三轮车'].gap_mm < 1 && pre['三轮车'].minUp > 0.95, JSON.stringify(pre['三轮车']));
  check('简易小车预设：电机固定在车架、轴插入后轮、前轴由两个轴承座支承（全部为真实连接）', Object.values(pre.cartJ).every(Boolean) && pre.cartJ.左电机 === 'fixed' && pre.cartJ.左后轮 === 'revolute' && pre.cartJ.前轴L === 'revolute' && pre['简易小车'].dist > 1 && pre['简易小车'].minUp > 0.95, JSON.stringify({ joints: pre.cartJ, run: pre['简易小车'] }));
  for (const [n, g, loose, nj] of pre.audit) check('连接可视审计（锚点贴合两零件 <1mm，无漏连零件）: ' + n, g < 1 && loose.length === 0, `max contact gap ${g}mm, joints ${nj}${loose.length ? ', 未连接: ' + loose.join(',') : ''}`);

  // ---------- screenshots ----------
  const sel = n => [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === n) - 1;
  await p.evaluate(i => { app.loadPreset(i); app.view('iso'); app.fit(); app.startRun(); app.simulate(2); app.select({ p: app.byName('车架').id }); app.updRun(); const c = app.sim.partPose(app.byName('车架').id).p; app.orbit.target.copy(c); app.cam.position.set(c.x + 0.7, c.y + 0.55, c.z + 0.8); app.orbit.update(); }, await p.evaluate(sel, '三轮车'));
  await frame(); await p.screenshot({ path: OUT + 'screenshot-tricycle-run.png' });
  await p.evaluate(i => { app.reset(); app.loadPreset(i); app.select(null); const ch = app.byName('车架'); app.orbit.target.set(0, 0.05, 0.08); app.cam.position.set(0.05, 0.28, 0.5); app.orbit.update(); }, await p.evaluate(sel, '简易小车'));
  await frame(); await p.screenshot({ path: OUT + 'screenshot-cart-closeup.png' });
};
