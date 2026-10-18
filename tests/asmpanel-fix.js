// Regression: 装配 panel must appear for ANY 2+ part selection (outliner across assemblies, canvas, after run / pause,
// narrow viewport), at the top of the right column and inside the viewport. Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const vp0 = p.viewport();
  // two sub-assemblies: motor + shaft + gear each (like the user's report)
  const build = () => p.evaluate(() => {
    const A = app; if (A.asm) A.asmCancel(true); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select');
    document.getElementById('btnClear').click();
    const mk = (x, z) => { const m = A.placePart('motor', { x, y: 0.1, z }), s = A.placePart('shaft', { x: x + 0.2, y: 0.1, z }), g = A.placePart('gear', { x: x + 0.3, y: 0.1, z: z + 0.1 });
      A.msel = new Set([m.id, s.id, g.id]); A.select({ p: g.id }, true); A.asmOpen([m.id, s.id, g.id]); A.asmCommit(); return [m.id, s.id, g.id]; };
    const a1 = mk(0, 0), a2 = mk(0.6, 0.45); A.select(null); A.view('iso'); A.fit(); return { a1, a2 };
  });
  const inVP = id => p.evaluate(id => { const e = document.getElementById(id); if (!e || getComputedStyle(e).display === 'none') return { ok: false, why: 'hidden' }; const r = e.getBoundingClientRect(), W = innerWidth, H = innerHeight;
    const cx = Math.min(W - 2, Math.max(1, r.left + Math.min(r.width / 2, 40))), cy = Math.min(H - 2, Math.max(1, r.top + Math.min(r.height / 2, 12))), hit = document.elementFromPoint(cx, cy);
    return { ok: r.width > 0 && r.height > 0 && r.top >= 0 && r.left >= 0 && r.right <= W + 0.5 && r.top < H - 20 && !!hit && (hit === e || e.contains(hit)), top: Math.round(r.top), left: Math.round(r.left), right: Math.round(r.right), bottom: Math.round(r.bottom), W, H }; }, id);
  const olCtrl = async ids => { await (await p.$(`#olList .olr[data-pid="${ids[0]}"] .oln`)).click(); await p.keyboard.down('Control'); for (const id of ids.slice(1)) await (await p.$(`#olList .olr[data-pid="${id}"] .oln`)).click(); await p.keyboard.up('Control'); await frame(); };
  const state = () => p.evaluate(() => { const pl = document.getElementById('asmPanel'), side = document.getElementById('side'), plot = document.getElementById('plot'), ro = document.getElementById('readout');
    const pr = pl.getBoundingClientRect(), vis = e => getComputedStyle(e).display !== 'none';
    return { asm: !!app.asm, pend: app.asmPend ? app.asmPend.slice() : null, mode: app.mode, shown: pl.style.display === 'block' && pr.height > 0, first: side.firstElementChild === pl, scrollTop: side.scrollTop,
      abovePlot: !vis(plot) || pr.top < plot.getBoundingClientRect().top, aboveReadout: !vis(ro) || pr.top < ro.getBoundingClientRect().top,
      goEdit: !!document.getElementById('asmGoEdit') && /返回编辑并装配/.test(document.getElementById('asmGoEdit').textContent), bigBtn: !!document.querySelector('#props #mAsm') && /装配选中零件/.test(document.querySelector('#props #mAsm').textContent),
      type: app.asm && app.asm.pairs[0] && app.asm.pairs[0].type, moved: app.asm && app.asm.info[0] ? app.asm.info[0].moved.slice() : null }; });
  const R = {};
  for (const [W, H] of [[1024, 520], [1280, 720]]) {
    await p.setViewport({ width: W, height: H }); await frame(); await new Promise(r => setTimeout(r, 150));
    const k = W + 'x' + H, S = R[k] = {};
    // 1. edit mode, outliner Ctrl+click: gear of 装配体 1 + gear of 装配体 2
    let ids = await build(); await frame();
    S.groups = await p.evaluate(ids => { const g = [...document.querySelectorAll('#olList .olg')].map(h => h.textContent); return g.filter(t => /装配体/.test(t)).length; }, ids);
    await p.evaluate(() => { document.getElementById('side').scrollTop = 9999; });
    await olCtrl([ids.a1[2], ids.a2[2]]);
    S.edit = await state(); S.editVP = await inVP('asmPanel'); S.editOk = await inVP('asmOk'); S.btnAsm = await inVP('btnAsm');
    // apply -> mesh, assembly B moved as one rigid group
    S.apply = await p.evaluate(ids => { const A = app, M0 = JSON.parse(A.asm.snap), rel = (a, b, M) => { const f = id => M ? M.parts.find(x => x.id === id) : A.part(id), P = f(a), Q = f(b), q = new A.cam.quaternion.constructor(...P.q).invert(); return new A._V(...Q.p).sub(new A._V(...P.p)).applyQuaternion(q).toArray(); };
      const r0 = [rel(ids.a2[2], ids.a2[0], M0), rel(ids.a2[2], ids.a2[1], M0)], p0 = M0.parts.find(x => x.id === ids.a2[0]).p.slice(); const res = A.asmCommit();
      const r1 = [rel(ids.a2[2], ids.a2[0]), rel(ids.a2[2], ids.a2[1])], g1 = A.part(ids.a1[2]), g2 = A.part(ids.a2[2]);
      const rigid = Math.max(...r0.flatMap((v, i) => v.map((x, k) => Math.abs(x - r1[i][k]))));
      const out = { couplings: res.couplings.length, errs: res.errs, dist: Math.hypot(...g1.p.map((v, i) => v - g2.p[i])), ideal: g1.d.m * (g1.d.z + g2.d.z) / 2, rigid, motorMoved: Math.hypot(...A.part(ids.a2[0]).p.map((v, i) => v - p0[i])), gap: A.editGap() };
      A.startRun(); A.simulate(0.5); out.runOk = !!A.sim && A.sim.maxGap() < 1e-3; A.reset(); return out; }, ids);
    // 2. after run + pause: outliner Ctrl+click -> panel with "返回编辑并装配", click it (real mouse)
    ids = await build(); await p.evaluate(() => { app.startRun(); app.simulate(1.0); app.pause(); }); await frame();
    await olCtrl([ids.a1[2], ids.a2[2]]);
    S.pause = await state(); S.pauseVP = await inVP('asmPanel'); S.pauseBtnVP = await inVP('asmGoEdit');
    await (await p.$('#asmGoEdit')).click(); await frame();
    S.pauseAfter = await state(); S.pauseAfterVP = await inVP('asmPanel');
    await p.evaluate(() => app.asmCancel(true));
    // 3. while RUNNING: canvas Ctrl+click (selection works in run mode) -> panel; big 🔧 button in the multi-select props opens it
    ids = await build(); await p.evaluate(() => { app.startRun(); app.simulate(0.5); app.pause(); }); await frame();
    const pick = async pid => p.evaluate(pid => { const o = app.objs[pid]; o.updateMatrixWorld(true);
      for (let r = 0; r <= 0.03; r += 0.005) for (let a = 0; a < 6.28; a += 0.5) { const w = new app._V(Math.cos(a) * r, Math.sin(a) * r, 0.004).applyMatrix4(o.matrixWorld), s = app.toScreen(w); const el = document.elementFromPoint(s.x, s.y); if (el && el.id === 'cv' && app.pickAt({ clientX: s.x, clientY: s.y }).pid === pid) return [s.x, s.y]; } return null; }, pid);
    await p.evaluate(() => app.startRun()); // running again: poses frozen enough for a click? use pause-state coordinates while running slowly
    await p.evaluate(() => { app.speed = 0.0001; document.getElementById('tipbar').style.display = 'none'; });
    const c1 = await pick(ids.a1[2]), c2 = await pick(ids.a2[2]);
    S.canvasPick = !!(c1 && c2);
    if (c1 && c2) { await p.mouse.click(c1[0], c1[1]); await frame(); await p.keyboard.down('Control'); await p.mouse.click(c2[0], c2[1]); await p.keyboard.up('Control'); await frame(); }
    S.run = await state(); S.runVP = await inVP('asmPanel'); S.bigVP = await p.evaluate(() => { const e = document.querySelector('#props #mAsm'); if (!e) return null; e.scrollIntoView({ block: 'nearest' }); const r = e.getBoundingClientRect(); return { top: r.top, bottom: r.bottom, H: innerHeight }; });
    await (await p.$('#props #mAsm')).click(); await frame();
    S.runAfter = await state(); S.runAfterVP = await inVP('asmPanel');
    await p.evaluate(() => { app.asmCancel(true); app.speed = 1; document.getElementById('tipbar').style.display = ''; });
    // 4. 2 parts already selected, then ▶ run: panel stays (pending); ⟲ reset -> opens the real assembly panel automatically
    ids = await build(); await p.evaluate(ids => { app.msel = new Set([ids.a1[2], ids.a2[2]]); app.select({ p: ids.a2[2] }, true); app.asmCancel(true); app.startRun(); app.simulate(0.3); }, ids); await frame();
    S.startRun = await state(); await (await p.$('#btnReset')).click(); await frame(); S.afterReset = await state();
    await p.evaluate(() => app.asmCancel(true));
    // toolbar: 🔧 装配 button reachable and works (J-equivalent) at this width
    ids = await build(); await p.evaluate(ids => { app.msel = new Set([ids.a1[2], ids.a2[2]]); app.select({ p: ids.a2[2] }, true); app.asmCancel(true); }, ids);
    S.tbVP = await inVP('btnAsm'); await (await p.$('#btnAsm')).click(); await frame(); S.tbOpen = await state();
    if (W === 1024) { await p.evaluate(() => { document.getElementById('tipbar').style.display = 'none'; app.fit(); }); await p.waitForFunction(() => !app.flash, { timeout: 4000 }).catch(() => { }); await frame(); await p.screenshot({ path: OUT + 'screenshot-panel-fix.png' }); console.log('saved screenshot-panel-fix.png'); await p.evaluate(() => { document.getElementById('tipbar').style.display = ''; }); }
    await p.evaluate(() => app.asmCancel(true));
  }
  await p.setViewport(vp0); await frame();
  const top = s => s.shown && s.first && s.abovePlot && s.aboveReadout;
  for (const k of Object.keys(R)) {
    const S = R[k];
    check(`[${k}] 编辑模式：组件栏 Ctrl+点击 装配体1齿轮 + 装配体2齿轮 → 装配面板在右栏顶部且在视口内（确定按钮可见），默认齿轮啮合，B 整个装配体移动`, S.groups >= 2 && S.edit.asm && top(S.edit) && S.edit.scrollTop === 0 && S.editVP.ok && S.editOk.ok && S.edit.type === 'gear' && S.edit.moved && S.edit.moved.length === 3 && S.edit.bigBtn, JSON.stringify({ groups: S.groups, edit: S.edit, vp: S.editVP, ok: S.editOk }));
    check(`[${k}] 齿轮啮合确定：中心距 = m(z1+z2)/2，B 装配体（电机+轴+齿轮）整体刚性移动，连接间隙 <0.5mm，可运行`, S.apply.couplings === 1 && !S.apply.errs.length && Math.abs(S.apply.dist - S.apply.ideal) < 5e-4 && S.apply.rigid < 1e-6 && S.apply.motorMoved > 0.01 && S.apply.gap < 5e-4 && S.apply.runOk, JSON.stringify(S.apply));
    check(`[${k}] 运行后暂停：组件栏 Ctrl+点击跨装配体 → 面板显示“返回编辑并装配”（在视口内、位于速度曲线之上），多选属性含“🔧 装配选中零件”`, !S.pause.asm && S.pause.pend && S.pause.pend.length === 2 && top(S.pause) && S.pause.goEdit && S.pause.bigBtn && S.pauseVP.ok && S.pauseBtnVP.ok && S.pause.mode === 'pause', JSON.stringify({ pause: S.pause, vp: S.pauseVP, btn: S.pauseBtnVP }));
    check(`[${k}] 点击“返回编辑并装配” → 重置到编辑模式并打开装配面板（齿轮啮合）`, S.pauseAfter.asm && S.pauseAfter.mode === 'edit' && S.pauseAfter.type === 'gear' && top(S.pauseAfter) && S.pauseAfterVP.ok, JSON.stringify({ after: S.pauseAfter, vp: S.pauseAfterVP }));
    check(`[${k}] 运行中：画布 Ctrl+点击两个齿轮 → 面板（返回编辑并装配）；点多选属性里的“🔧 装配选中零件” → 打开装配面板`, S.canvasPick && S.run.pend && S.run.pend.length === 2 && S.run.mode === 'run' && top(S.run) && S.runVP.ok && S.run.bigBtn && S.runAfter.asm && S.runAfter.mode === 'edit' && S.runAfter.type === 'gear' && S.runAfterVP.ok, JSON.stringify({ pick: S.canvasPick, run: S.run, vp: S.runVP, big: S.bigVP, after: S.runAfter }));
    check(`[${k}] 已多选再运行 → 面板保持（提示返回编辑）；点“⟲ 重置” → 自动打开装配面板`, S.startRun.pend && S.startRun.shown && S.startRun.goEdit && S.afterReset.asm && S.afterReset.mode === 'edit' && top(S.afterReset), JSON.stringify({ startRun: S.startRun, afterReset: S.afterReset }));
    check(`[${k}] 顶栏换行：“🔧 装配”按钮在视口内可点击，点击打开面板`, S.btnAsm.ok && S.tbVP.ok && S.tbOpen.asm && top(S.tbOpen), JSON.stringify({ btn: S.tbVP, open: S.tbOpen.asm }));
  }
};
