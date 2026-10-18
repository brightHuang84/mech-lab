// Selection shows NO gizmo on any path + panel-based assembly (装配面板). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const scr = w => p.evaluate(w => { const s = app.toScreen(new app._V(w[0], w[1], w[2])); return [s.x, s.y]; }, w);
  const surf = async (pid, local) => scr(await p.evaluate((pid, l) => { const P = app.part(pid), Q = app.cam.quaternion.constructor; return new app._V(...l).applyQuaternion(new Q(...P.q)).add(new app._V(...P.p)).toArray(); }, pid, local));
  const click = async (xy, mods = []) => { for (const m of mods) await p.keyboard.down(m); await p.mouse.click(xy[0], xy[1]); for (const m of mods) await p.keyboard.up(m); await frame(); };
  const gz = () => p.evaluate(() => ({ obj: !!app.tc.object, vis: !!(app.tc.getHelper ? app.tc.getHelper().visible : app.tc.visible), on: !!app.gizmoOn }));
  const noGz = g => !g.obj && !g.vis && !g.on;
  const setup = async (pos = [0.55, 0.5, 0.8], tgt = [0.1, 0.02, 0.05]) => { await p.evaluate((pos, tgt) => { if (app.asm) app.asmCancel(true); if (app.mode !== 'edit') app.reset(); if (app.tool !== 'select') app.setTool('select'); document.getElementById('btnClear').click(); app.cam.position.set(...pos); app.orbit.target.set(...tgt); app.orbit.update(); app.select(null); }, pos, tgt); await frame(); };
  const presetIdx = n => p.evaluate(n => [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === n) - 1, n);

  // ======================= 1. selecting a part never shows the gizmo =======================
  const G = {};
  await setup();
  const g1 = await p.evaluate(() => { const a = app.placePart('gear', { x: 0, y: 0, z: 0 }), b = app.placePart('block', { x: 0.25, y: 0, z: 0.1 }); app.select(null); return [a.id, b.id]; });
  await frame(); G.canvas = await (async () => { await click(await surf(g1[0], [0, 0, 0.01])); const sel = await p.evaluate(id => app.sel && app.sel.p === id, g1[0]); return Object.assign(await gz(), { sel }); })();
  await (await p.$(`#olList .olr[data-pid="${g1[1]}"] .oln`)).click(); await frame();
  G.outliner = Object.assign(await gz(), { sel: await p.evaluate(id => app.sel && app.sel.p === id, g1[1]) });
  // palette add (real mouse drag from the library)
  { const box = await (await p.$('#palItems .item[data-type="shaft"]')).boundingBox(), cv = await (await p.$('#cv')).boundingBox(); await p.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await p.mouse.down(); await p.mouse.move(cv.x + 120, cv.y + cv.height - 120, { steps: 8 }); await p.mouse.up(); await frame();
    G.palette = Object.assign(await gz(), { sel: await p.evaluate(() => app.sel && app.sel.p === app.M.parts[app.M.parts.length - 1].id) }); }
  // preset load through the dropdown (selects its traced part), then a real click on a gear of the reducer
  { const i = await presetIdx('两级齿轮减速器'); await p.select('#selPreset', String(i)); await frame(); G.preset = await gz();
    await p.evaluate(() => { app.view('iso'); app.fit(); }); await frame(); const gid = await p.evaluate(() => app.byName('齿轮2 z40').id); await click(await surf(gid, [0, 0.06, 0]));
    G.presetClick = Object.assign(await gz(), { sel: await p.evaluate(id => app.sel && app.sel.p === id, gid) }); }
  await p.evaluate(() => { const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '单摆') - 1; app.loadPreset(i); }); await frame(); G.presetTrace = Object.assign(await gz(), { sel: await p.evaluate(() => !!(app.sel && app.sel.p)) });
  // after a drag-snap (real mouse): rod end dragged onto a base pin
  { await setup([0.1, 0.45, 1.0], [0.05, 0.25, 0]); const ids = await p.evaluate(() => { const b = app.placePart('base', { x: 0, y: 0, z: 0 }), r = app.placePart('rod', { x: 0.3, y: 0, z: 0.2 }); app.select(null); return [b.id, r.id]; }); await frame();
    const pin = await p.evaluate(id => { const B = app.part(id), c = app._connectors(B).find(c => c.n === '轴销(前)'); return [B.p[0] + c.p[0], B.p[1] + c.p[1], B.p[2] + c.p[2] + 0.006]; }, ids[0]);
    const from = await surf(ids[1], [-0.145, 0, 0.005]), to = await scr(pin); const j0 = await p.evaluate(() => app.M.joints.length);
    await p.mouse.move(from[0], from[1]); await p.mouse.down(); await p.mouse.move((from[0] + to[0]) / 2, (from[1] + to[1]) / 2, { steps: 6 }); await p.mouse.move(to[0], to[1], { steps: 6 }); await frame(); await p.mouse.up(); await frame();
    G.snap = Object.assign(await gz(), { joints: (await p.evaluate(() => app.M.joints.length)) - j0 }); }
  // after mate (API) and after undo
  { await setup(); const ids = await p.evaluate(() => { const a = app.placePart('block', { x: 0, y: 0, z: 0 }), b = app.placePart('block', { x: 0.2, y: 0, z: 0.1 }); return [a.id, b.id]; });
    await p.evaluate(ids => { app.mate({ pid: ids[1], p: [0, -0.025, 0], n: [0, -1, 0] }, { pid: ids[0], p: [0, 0.025, 0], n: [0, 1, 0] }, { type: 'fixed' }); app.select({ p: ids[1] }); }, ids); G.mate = await gz();
    await p.evaluate(() => app.undo()); await frame(); await p.evaluate(id => app.select({ p: id }), ids[0]); G.undo = await gz(); }
  // sticky check: G on part 1, then click part 2 -> gizmo gone; Ctrl+click multi-select -> gone
  { await setup(); const ids = await p.evaluate(() => { const a = app.placePart('block', { x: 0, y: 0, z: 0 }), b = app.placePart('block', { x: 0.2, y: 0, z: 0.1 }); app.select(null); return [a.id, b.id]; }); await frame();
    await click(await surf(ids[0], [0, 0.025, 0])); await p.keyboard.press('KeyG'); await frame(); const on1 = await gz();
    await click(await surf(ids[1], [0, 0.025, 0])); G.switchSel = Object.assign(await gz(), { wasOn: on1.obj && on1.vis });
    await click(await surf(ids[1], [0, 0.025, 0])); await p.keyboard.down('Shift'); await p.keyboard.press('KeyG'); await p.keyboard.up('Shift'); await frame(); const on2 = await gz();
    await click(await surf(ids[0], [0, 0.025, 0]), ['Control']); G.multi = Object.assign(await gz(), { wasOn: on2.obj, panel: await p.evaluate(() => !!app.asm) });
    await p.evaluate(() => app.asmCancel()); }
  const allNo = Object.entries(G).every(([k, g]) => noGz(g));
  check('选中零件无坐标轴：画布点击 / 组件栏 / 零件库拖入 / 载入示例 / 示例中点击齿轮', noGz(G.canvas) && G.canvas.sel && noGz(G.outliner) && G.outliner.sel && noGz(G.palette) && G.palette.sel && noGz(G.preset) && noGz(G.presetClick) && G.presetClick.sel && noGz(G.presetTrace), JSON.stringify({ canvas: G.canvas, outliner: G.outliner, palette: G.palette, preset: G.preset, presetClick: G.presetClick, presetTrace: G.presetTrace }));
  check('选中零件无坐标轴：吸附装配后 / 配合后 / 撤销后', noGz(G.snap) && G.snap.joints >= 1 && noGz(G.mate) && noGz(G.undo), JSON.stringify({ snap: G.snap, mate: G.mate, undo: G.undo }));
  check('坐标轴只在按 G/Shift+G 时出现，换选零件或多选后自动关闭', G.switchSel.wasOn && noGz(G.switchSel) && G.multi.wasOn && noGz(G.multi) && G.multi.panel && allNo, JSON.stringify({ switchSel: G.switchSel, multi: G.multi }));

  // ======================= 2. assembly panel =======================
  // open by Ctrl+click on the canvas
  await setup([0.45, 0.42, 0.7], [0.12, 0.02, 0.08]);
  const sg = await p.evaluate(() => { const s = app.placePart('shaft', { x: 0, y: 0, z: 0 }), g = app.placePart('gear', { x: 0.25, y: 0, z: 0.15 }); app.select(null); return [s.id, g.id]; }); await frame();
  await click(await surf(sg[0], [0, 0.006, 0])); await click(await surf(sg[1], [0, 0, 0.01]), ['Control']);
  const op1 = await p.evaluate(() => ({ open: !!app.asm && document.getElementById('asmPanel').style.display === 'block', n: app.asm && app.asm.pairs.length, a: app.asm && app.asm.pairs[0].a, b: app.asm && app.asm.pairs[0].b, lists: document.querySelectorAll('#asmPanel select.asmList').length, ok: !!document.getElementById('asmOk'), types: [...document.querySelectorAll('#asmType option')].map(o => o.textContent), gizmo: !!app.tc.object }));
  const dShaftGear = await p.evaluate(() => { const pr = app.asm.pairs[0]; return { type: pr.type, a: app.asmEntry(pr.a, pr.ka).name, b: app.asmEntry(pr.b, pr.kb).name }; });
  check('Ctrl+点击两个零件 → 右侧出现装配面板（A/B 连接点列表、连接类型、确定/取消），无坐标轴', op1.open && op1.n === 1 && op1.a === sg[0] && op1.b === sg[1] && op1.lists === 2 && op1.ok && op1.types.includes('固定') && op1.types.includes('铰链(转动)') && op1.types.includes('螺旋') && !op1.gizmo, JSON.stringify(op1));
  // hover an entry -> highlight; choose B point by list
  const lb = await p.$('#asmEB option'); const hb = await lb.boundingBox(); await p.mouse.move(hb.x + 10, hb.y + hb.height / 2); await frame();
  const hov = await p.evaluate(() => ({ hov: app.asm.hov, marks: app.asmFx ? app.asmFx.children.length : 0, lblA: document.getElementById('asmLblA') && document.getElementById('asmLblA').style.display }));
  check('悬停列表项 → 3D 模型上高亮该连接点（标记 + A/B 标签）', !!hov.hov && hov.hov.side === 'b' && hov.marks > 4 && hov.lblA === 'block', JSON.stringify(hov));
  // preview: B is moved (ghost) but the committed model is unchanged until 确定; 取消 restores
  const pv = await p.evaluate(ids => { const snap0 = app.asm.snap, g = app.part(ids[1]).p.slice(); const o = app.objs[ids[1]]; let tr = false; o.traverse(m => { if (m.isMesh && m.material.transparent && m.material.opacity < 0.9) tr = true; });
    app.asmSet(0, { type: 'revolute', off: 0.01, ang: 30 }); const moved = Math.hypot(...app.part(ids[1]).p.map((v, i) => v - JSON.parse(snap0).parts.find(P => P.id === ids[1]).p[i]));
    app.asmCancel(); return { ghost: tr, movedInPreview: moved > 0.05, same: app.snapshot() === snap0, undo: app.undoS.length, open: !!app.asm }; }, sg);
  check('实时预览：B 半透明移动到装配位置；取消后模型完全不变', pv.ghost && pv.movedInPreview && pv.same && !pv.open, JSON.stringify(pv));
  // J key reopens; Enter applies -> gear on shaft body, fixed, one undo step
  await p.evaluate(ids => { app.undoS.length = 0; app.msel = new Set(ids); app.sel = { p: ids[1] }; app.updSel(); }, sg); const snapBefore = await p.evaluate(() => app.snapshot()); await p.keyboard.press('KeyJ'); await frame();
  const j0 = await p.evaluate(s => ({ open: !!app.asm, undo: app.undoS.length, snap: s, previewOnly: app.asm && app.asm.snap === s }), snapBefore);
  await p.keyboard.press('Enter'); await frame();
  const ap = await p.evaluate((ids, pr0) => { const S = app.undoS.length, j = app.M.joints[app.M.joints.length - 1], sh = app.part(ids[0]), g = app.part(ids[1]);
    const ent = app.asmEnts(ids[0]).find(e => /^轴身/.test(e.name)), hub = app.asmEnts(ids[1]).find(e => /^中心孔/.test(e.name));
    const ga = new app._V(...g.p), sa = new app._V(...sh.p), ax = new app._V(0, 0, 1).applyQuaternion(new app.cam.quaternion.constructor(...sh.q)), gx = new app._V(0, 0, 1).applyQuaternion(new app.cam.quaternion.constructor(...g.q));
    const d = ga.clone().sub(sa); const radial = d.clone().addScaledVector(ax, -d.dot(ax)).length();
    return { open: !!app.asm, joint: j && j.type, a: j && j.a, b: j && j.b, undoSteps: S, radial, along: Math.abs(d.dot(ax)), axes: Math.abs(ax.dot(gx)), ent: !!ent, hub: !!hub, gizmo: !!app.tc.object }; }, sg, j0);
  check('默认：轴+齿轮 → 齿轮中心孔装在轴身上，固定连接', dShaftGear.type === 'fixed' && /^轴身/.test(dShaftGear.a) && /^中心孔/.test(dShaftGear.b), JSON.stringify(dShaftGear));
  check('J 打开面板，Enter 确定装配：建立固定连接，齿轮轴心与轴线重合(<0.5mm)、轴线平行，一步撤销，无坐标轴', j0.open && j0.previewOnly && !ap.open && ap.joint === 'fixed' && ap.a === sg[0] && ap.b === sg[1] && ap.radial < 0.0005 && ap.axes > 0.9999 && ap.undoSteps === j0.undo + 1 && !ap.gizmo, JSON.stringify(ap));
  await p.evaluate(() => app.undo()); await frame();
  const un = await p.evaluate(s => ({ same: app.snapshot() === s, joints: app.M.joints.length }), j0.snap);
  check('撤销一次 → 恢复装配前（位置与连接）', un.same && un.joints === 0, JSON.stringify(un));

  // defaults for other pairs + apply checks (API)
  const D = await p.evaluate(() => {
    const A = app, out = {};
    const mk = (specs) => { document.getElementById('btnClear').click(); const ps = specs.map(([t, x, z, o]) => A.placePart(t, { x, y: 0, z }, o)); A.select({ p: ps[0].id }); A.msel = new Set(ps.map(P => P.id)); A.select({ p: ps[ps.length - 1].id }, true); return ps; };
    const desc = pr => ({ type: pr.type, a: A.asmEntry(pr.a, pr.ka) && A.asmEntry(pr.a, pr.ka).name, b: A.asmEntry(pr.b, pr.kb) && A.asmEntry(pr.b, pr.kb).name });
    // two gears, same module -> mesh
    let ps = mk([['gear', 0, 0], ['gear', 0.3, 0.1, { d: { z: 40 } }]]); let S = A.asmOpen(); out.gears = desc(S.pairs[0]); out.gears.types = A.asmTypes(S.pairs[0]);
    let r = A.asmCommit(); const g1 = A.part(ps[0].id), g2 = A.part(ps[1].id); out.gears.dist = Math.hypot(g1.p[0] - g2.p[0], g1.p[1] - g2.p[1], g1.p[2] - g2.p[2]); out.gears.ideal = g1.d.m * (g1.d.z + g2.d.z) / 2; out.gears.couplings = A.M.couplings.length;
    A.startRun(); A.simulate(0.2); out.gears.runOk = !!A.sim; A.reset();
    // gear + gear different module -> no mesh offered
    ps = mk([['gear', 0, 0], ['gear', 0.3, 0.1, { d: { m: 0.003 } }]]); S = A.asmOpen(); out.gearsBadM = A.asmTypes(S.pairs[0]).includes('gear'); A.asmCancel(true);
    // shaft + bearing -> hinge through the bore
    ps = mk([['bearing', 0, 0], ['shaft', 0.3, 0.15]]); S = A.asmOpen(); out.shaftBearing = desc(S.pairs[0]); const pr = S.pairs[0]; r = A.asmCommit();
    { const wa = A.asmPointW(pr.a, pr.ka, pr.pa), wb = A.asmPointW(pr.b, pr.kb, pr.pb); out.shaftBearing.d = wa.p.distanceTo(wb.p); out.shaftBearing.axes = Math.abs(wa.n.dot(wb.n)); out.shaftBearing.joint = (A.M.joints[0] || {}).type; }
    // rod + rod -> hinge between end holes (stacked side by side)
    ps = mk([['rod', 0, 0], ['rod', 0.32, 0.2]]); S = A.asmOpen(); out.rods = desc(S.pairs[0]); const pr2 = S.pairs[0]; r = A.asmCommit();
    { const wa = A.asmPointW(pr2.a, pr2.ka, pr2.pa), wb = A.asmPointW(pr2.b, pr2.kb, pr2.pb), dv = wb.p.clone().sub(wa.p); out.rods.radial = dv.clone().addScaledVector(wa.n, -dv.dot(wa.n)).length(); out.rods.axes = Math.abs(wa.n.dot(wb.n));
      const j = A.M.joints[0]; out.rods.joint = j && j.type; out.rods.anchorGap = A.editGap(); }
    // wheel + shaft -> fixed on the shaft
    ps = mk([['shaft', 0, 0], ['wheel', 0.3, 0.2]]); S = A.asmOpen(); out.wheelShaft = desc(S.pairs[0]); A.asmCancel(true);
    // 3-part chain -> 2 joints; star mode offered
    ps = mk([['rod', 0, 0], ['rod', 0.32, 0.2], ['rod', 0.6, -0.2]]); S = A.asmOpen(); out.chain = { mode: S.mode, pairs: S.pairs.length, modeSel: !!document.getElementById('asmMode'), cards: document.querySelectorAll('#asmPanel .apair').length };
    out.chain.ab = [S.pairs[0].a === ps[0].id && S.pairs[0].b === ps[1].id, S.pairs[1].a === ps[1].id && S.pairs[1].b === ps[2].id];
    A.asmSetMode('star'); out.chain.star = S.pairs.map(pr => pr.a === ps[0].id); A.asmSetMode('chain');
    r = A.asmCommit(); out.chain.joints = A.M.joints.length; out.chain.types = A.M.joints.map(j => j.type); out.chain.gap = A.editGap(); out.chain.undo1 = (() => { const n = A.undoS.length; A.undo(); const k = A.M.joints.length; A.redo(); return { k, n }; })();
    // existing joints show in the panel and can be edited / deleted there
    S = A.asmOpen([ps[0].id, ps[1].id]); out.existing = { type: S.pairs[0].type, rows: document.querySelectorAll('#asmPanel .asmJT').length };
    const sel = document.querySelector('#asmPanel .asmJT'); sel.value = 'fixed'; sel.dispatchEvent(new Event('change'));
    out.existing.after = JSON.parse(A.asm.snap).joints.filter(j => j.a === ps[0].id && j.b === ps[1].id || j.a === ps[1].id && j.b === ps[0].id).map(j => j.type);
    document.querySelector('#asmPanel .asmJD').click(); out.existing.afterDel = JSON.parse(A.asm.snap).joints.length; out.existing.stillOpen = !!A.asm; out.existing.newDefault = A.asm && A.asm.pairs[0].type;
    A.asmCancel(true);
    return out;
  });
  check('默认：两个同模数齿轮 → 齿轮啮合，自动中心距 = m(z1+z2)/2，可运行', D.gears.type === 'gear' && D.gears.types.includes('gear') && Math.abs(D.gears.dist - D.gears.ideal) < 5e-4 && D.gears.couplings === 1 && D.gears.runOk && !D.gearsBadM, JSON.stringify({ gears: D.gears, badModuleOffersMesh: D.gearsBadM }));
  check('默认：轴+轴承座 → 铰链穿过轴承孔（点距 <0.5mm，轴线平行）', D.shaftBearing.type === 'revolute' && /^轴孔/.test(D.shaftBearing.a) && /^轴身/.test(D.shaftBearing.b) && D.shaftBearing.d < 5e-4 && D.shaftBearing.axes > 0.9999 && D.shaftBearing.joint === 'revolute', JSON.stringify(D.shaftBearing));
  check('默认：连杆+连杆 → 端孔铰链（孔轴线重合 <0.5mm、平行，自动错开厚度）', D.rods.type === 'revolute' && /^端孔/.test(D.rods.a) && /^端孔/.test(D.rods.b) && D.rods.radial < 5e-4 && D.rods.axes > 0.9999 && D.rods.joint === 'revolute' && D.rods.anchorGap < 5e-4, JSON.stringify(D.rods));
  check('默认：轮+轴 → 固定在轴身上', D.wheelShaft.type === 'fixed' && /^轴身/.test(D.wheelShaft.a) && /^中心孔/.test(D.wheelShaft.b), JSON.stringify(D.wheelShaft));
  check('3 个零件：依次连接 A-B、B-C → 2 个连接（可切换“全部连到第一个”），一步撤销', D.chain.mode === 'chain' && D.chain.pairs === 2 && D.chain.modeSel && D.chain.cards === 2 && D.chain.ab.every(Boolean) && D.chain.star.every(Boolean) && D.chain.joints === 2 && D.chain.gap < 5e-4 && D.chain.undo1.k === 0, JSON.stringify(D.chain));
  check('面板中显示所选零件间已有连接，可改类型 / 删除', D.existing.type === 'keep' && D.existing.rows === 1 && D.existing.after[0] === 'fixed' && D.existing.afterDel === 1 && D.existing.stillOpen && D.existing.newDefault === 'revolute', JSON.stringify(D.existing));

  // pick a point by clicking the model + outliner Ctrl multi-select + Esc cancels
  await setup([0.2, 0.55, 0.75], [0.15, 0.0, 0.1]);
  const rr = await p.evaluate(() => { const a = app.placePart('rod', { x: 0, y: 0, z: 0 }), b = app.placePart('rod', { x: 0.3, y: 0, z: 0.25 }); app.select(null); return [a.id, b.id]; }); await frame();
  await (await p.$(`#olList .olr[data-pid="${rr[0]}"] .oln`)).click(); await p.keyboard.down('Control'); await (await p.$(`#olList .olr[data-pid="${rr[1]}"] .oln`)).click(); await p.keyboard.up('Control'); await frame();
  const ol = await p.evaluate(() => ({ open: !!app.asm, snap: app.asm && app.asm.snap, ka: app.asm && app.asmEntry(app.asm.pairs[0].a, app.asm.pairs[0].ka).name }));
  await click(await surf(rr[0], [0, 0, 0.005]));
  const pk = await p.evaluate(() => ({ ka: app.asmEntry(app.asm.pairs[0].a, app.asm.pairs[0].ka).name, open: !!app.asm }));
  await p.keyboard.press('Escape'); await frame();
  const esc = await p.evaluate(s => ({ open: !!app.asm, same: app.snapshot() === s }), ol.snap);
  check('组件栏 Ctrl+点击多选 → 面板；在模型上点击 A 零件即可选连接点；Esc 取消且模型不变', ol.open && /^端孔/.test(ol.ka) && pk.open && /^中点/.test(pk.ka) && !esc.open && esc.same, JSON.stringify({ ol: { open: ol.open, ka: ol.ka }, pk, esc }));
  const tb = await p.evaluate(() => ({ btn: !!document.getElementById('btnAsm'), txt: (document.getElementById('btnAsm') || {}).textContent, help: /装配/.test(document.getElementById('tipbar').textContent) }));
  check('顶栏“🔧 装配”按钮 + 提示文字', tb.btn && /装配/.test(tb.txt) && tb.help, JSON.stringify(tb));

  // ======================= screenshots =======================
  await setup([0.36, 0.3, 0.55], [0.08, 0.03, 0.05]);
  await p.evaluate(() => { const s = app.placePart('shaft', { x: 0, y: 0, z: 0 }, { d: { L: 0.2 } }), g = app.placePart('gear', { x: 0.2, y: 0, z: 0.12 }); app.select({ p: s.id }); app.msel = new Set([s.id, g.id]); app.select({ p: g.id }, true); app.asmOpen(); document.getElementById('tipbar').style.display = 'none'; });
  { const o = await p.$('#asmEA option:nth-child(2)'); const bb = await o.boundingBox(); await p.mouse.move(bb.x + 20, bb.y + bb.height / 2); } await new Promise(r => setTimeout(r, 200)); await frame();
  await p.waitForFunction(() => !app.flash, { timeout: 4000 }).catch(() => {}); await frame(); await p.screenshot({ path: OUT + 'screenshot-assembly-panel.png' }); console.log('saved screenshot-assembly-panel.png');
  await p.evaluate(() => app.asmCancel(true));
  { const i = await presetIdx('两级齿轮减速器'); await p.evaluate(i => { app.loadPreset(i); app.view('iso'); app.fit(); app.select(null); }, i); await frame();
    const gid = await p.evaluate(() => app.byName('齿轮2 z40').id); await click(await surf(gid, [0, 0.06, 0])); await new Promise(r => setTimeout(r, 200)); await frame();
    const g = await gz(); check('截图 screenshot-select.png：选中齿轮仅高亮、无坐标轴', noGz(g) && await p.evaluate(id => app.sel && app.sel.p === id, gid), JSON.stringify(g));
    await p.screenshot({ path: OUT + 'screenshot-select.png' }); console.log('saved screenshot-select.png'); }
  // stale gizmo key from an old build is ignored / removed on startup
  await p.evaluate(() => { localStorage.setItem('mech3d-gizmo', 'on'); localStorage.setItem('mech3d-transformMode', 'translate'); });
  await p.reload(); await p.waitForFunction('window.app', { timeout: 60000 }); await p.evaluate(() => { app._V = app.cam.position.constructor; });
  const ls = await p.evaluate(() => { const i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '四连杆') - 1; app.loadPreset(i); app.select({ p: app.byName('连杆').id }); return { keys: Object.keys(localStorage).filter(k => /gizmo|transform/i.test(k)), gizmo: !!app.tc.object, on: app.gizmoOn }; });
  check('旧版本残留的坐标轴设置被清除，启动后选中零件仍无坐标轴', !ls.keys.length && !ls.gizmo && !ls.on, JSON.stringify(ls));
};
