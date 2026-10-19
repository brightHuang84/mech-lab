// Regression: easy connection-point selection in the 装配 panel — big screen-space markers only on the two parts, hover label +
// green ghost preview, click a point directly in 3D (shaft end -> flush, any point along a shaft / on a face / eccentric on a disc),
// drag along the chosen shaft, slider / mm / % inputs, quick end/middle buttons, ranked suggestions with one-click apply,
// fully manual rod-rod connections (any point, any type, angle between the bars). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const sleep = t => new Promise(r => setTimeout(r, t));
  await p.setViewport({ width: 1500, height: 900 }); await frame();
  await p.evaluate(() => {
    const A = app, V = A.cam.position.constructor, Q = A.cam.quaternion.constructor;
    A._pk = {
      V, Q,
      clear(pos = [0.32, 0.3, 0.42], tgt = [0, 0.05, 0]) { if (A.asm) A.asmCancel(true); if (A.mode !== 'edit') A.reset(); if (A.tool !== 'select') A.setTool('select'); if (A.gizmoOn) A.toggleGizmo(); document.getElementById('btnClear').click(); A.msel = new Set(); A.cam.position.set(...pos); A.orbit.target.set(...tgt); A.orbit.update(); A.cam.updateMatrixWorld(); },
      open(a, b) { A.select({ p: a }); A.msel = new Set([a, b]); A.select({ p: b }, true); return A.asmOpen([a, b]); },
      scr(w) { A.cam.updateMatrixWorld(); const s = A.toScreen(w); return [s.x, s.y]; },
      ent(pid, re) { return A.asmEnts(pid).find(e => re.test(e.name)); },
      entScr(pid, re, prm) { const e = A._pk.ent(pid, re); return A._pk.scr(A.asmPointW(pid, e.key, prm || {}).p); },
      local(pid, l) { const P = A.part(pid); return new V(...l).applyQuaternion(new Q(...P.q)).add(new V(...P.p)); },
      ax(pid, l) { return new V(...l).applyQuaternion(new Q(...A.part(pid).q)).normalize(); },
      noGz() { return !A.tc.object && !A.gizmoOn; },
      ground() { const r = A.groundFix({ quiet: true }); return r.dy; },
    };
  });
  const R = {};
  // ---------- 1. wheel + shaft: only A/B markers, other joint markers hidden, hover label + ghost, click shaft end -> flush ----------
  await p.evaluate(() => { const A = app, K = A._pk; K.clear();
    const b1 = A.placePart('block', { x: -0.3, y: 0, z: -0.3 }), b2 = A.placePart('block', { x: -0.3, y: 0, z: -0.15 }); A.msel = new Set([b1.id, b2.id]); A.select({ p: b2.id }, true); A.asmOpen([b1.id, b2.id]); A.asmCommit();
    const s = A.placePart('shaft', { x: 0, y: 0.05, z: 0 }, { d: { L: 0.2 } }), w = A.placePart('wheel', { x: 0.2, y: 0.05, z: 0 }); A.select(null); K.ids = { s: s.id, w: w.id };
    K.open(s.id, w.id); });
  await frame(); await frame();
  R.m = await p.evaluate(() => { const A = app, S = A.asm, other = Object.entries(A.jobjs).filter(([id]) => { const j = A.M.joints.find(x => x.id === id); return j && ![j.a, j.b].includes(A._pk.ids.s) && ![j.a, j.b].includes(A._pk.ids.w); });
    return { open: !!S, sprites: A.asmFx.children.filter(o => o.isSprite).length, otherHidden: other.length > 0 && other.every(([, o]) => !o.visible), tags: [...document.querySelectorAll('.asmTag')].filter(t => t.style.display === 'block').map(t => t.textContent), sug: S.pairs[0].sugg.length, sugRows: document.querySelectorAll('#asmPanel .sugR').length, foc: S.focus, gz: A._pk.noGz() }; });
  const fr = await p.evaluate(() => app._pk.entScr(app._pk.ids.s, /^前端/));
  await p.mouse.move(fr[0] - 30, fr[1] - 30); await p.mouse.move(fr[0], fr[1], { steps: 4 }); await sleep(320); await frame();
  R.h = await p.evaluate(() => { const A = app, S = A.asm, l = document.getElementById('asmLblH'); return { hov: S.hovHit && S.hovHit.e.name, lbl: l && l.style.display === 'block' ? l.textContent : '', ghost: !!(A.asmGhostFx && A.asmGhostFx.children.length), cursor: document.getElementById('cv').style.cursor, unchanged: S.pairs[0].pa.s }; });
  await p.mouse.click(fr[0], fr[1]); await frame();
  R.c = await p.evaluate(() => { const A = app, pr = A.asm.pairs[0]; return { ka: A.asmEntry(pr.a, pr.ka).name, s: pr.pa.s, type: pr.type, foc: A.asm.focus, open: !!A.asm }; });
  R.c1 = await p.evaluate(() => { const A = app, K = A._pk, r = A.asmCommit(), S = A.part(K.ids.s), W = A.part(K.ids.w), ax = K.ax(S.id, [0, 0, 1]), d = new K.V(...W.p).sub(new K.V(...S.p)), t = d.dot(ax);
    return { errs: r.errs, joint: (A.M.joints.find(j => r.joints.includes(j.id)) || {}).type, flush: Math.abs(t + W.d.t / 2 - S.d.L / 2), perp: d.addScaledVector(ax, -t).length(), ground: K.ground(), gz: K.noGz(), markersBack: Object.values(A.jobjs).every(o => o.visible) }; });
  // resize keeps the connection (wheel follows the end)
  R.rz = await p.evaluate(() => { const A = app, K = A._pk; A.resizeCommit(K.ids.s, { L: 0.3 }); const S = A.part(K.ids.s), W = A.part(K.ids.w), ax = K.ax(S.id, [0, 0, 1]), t = new K.V(...W.p).sub(new K.V(...S.p)).dot(ax); return { flush: Math.abs(t + W.d.t / 2 - S.d.L / 2), gap: A.editGap() }; });
  check('装配面板：只显示 A/B 两个零件的大号连接点（其他连接标记隐藏）、焦点零件显示中文点名，推荐列表，无坐标轴', R.m.open && R.m.sprites >= 4 && R.m.otherHidden && R.m.tags.includes('前端') && R.m.tags.includes('后端') && R.m.sug >= 2 && R.m.sugRows >= 2 && R.m.foc === 'a' && R.m.gz, JSON.stringify(R.m));
  check('3D 悬停连接点 → 中文标签 + 结果说明 + 绿色预览（模型未改），指针变手形', /前端/.test(R.h.hov) && /前端/.test(R.h.lbl) && /→/.test(R.h.lbl) && /固定/.test(R.h.lbl) && R.h.ghost && R.h.cursor === 'pointer', JSON.stringify(R.h));
  check('3D 点击轴“前端” + 轮 → 轮齐平装在轴端（轴身位置自动 = L/2 − t/2），固定，确定后连接/地面/无坐标轴正常', /^轴身/.test(R.c.ka) && Math.abs(R.c.s - 0.085) < 1e-6 && R.c.type === 'fixed' && R.c.open && R.c1.joint === 'fixed' && R.c1.flush < 5e-4 && R.c1.perp < 5e-4 && R.c1.ground < 0.001 && R.c1.gz && R.c1.markersBack && !R.c1.errs.length, JSON.stringify({ c: R.c, c1: R.c1 }));
  check('轴端装轮后改轴长 → 轮仍齐平在轴端，连接间隙 <0.5mm', R.rz.flush < 5e-4 && R.rz.gap < 5e-4, JSON.stringify(R.rz));

  // ---------- 2. gear on the shaft middle (quick button), slider, real drag along the shaft ----------
  await p.evaluate(() => { const A = app, K = A._pk; K.clear(); const s = A.placePart('shaft', { x: 0, y: 0.05, z: 0 }, { d: { L: 0.2 } }), g = A.placePart('gear', { x: 0.2, y: 0.05, z: 0.08 }); A.select(null); K.ids = { s: s.id, g: g.id }; K.open(s.id, g.id); });
  await frame();
  R.q = await p.evaluate(() => { const A = app, b = [...document.querySelectorAll('#asmPanel .asmQP')].find(x => x.textContent === '中点'); b.click(); const s1 = A.asm.pairs[0].pa.s;
    const r = document.getElementById('asmSRA'); r.value = '-40'; r.dispatchEvent(new Event('input')); const s2 = A.asm.pairs[0].pa.s, num = document.getElementById('asmSA').value, pct = document.getElementById('asmSPA').value, still = !!document.getElementById('asmSRA');
    const pc = document.getElementById('asmSPA'); pc.value = '75'; pc.dispatchEvent(new Event('input')); const s3 = A.asm.pairs[0].pa.s;
    b.isConnected; [...document.querySelectorAll('#asmPanel .asmQP')].find(x => x.textContent === '中点').click(); return { s1, s2, num, pct, still, s3, buttons: [...document.querySelectorAll('#asmPanel .asmQP')].map(x => x.textContent) }; });
  // drag along the (stationary) shaft body in 3D
  const d0 = await p.evaluate(() => app._pk.scr(app.asmPointW(app._pk.ids.s, app.asm.pairs[0].ka, { s: 0.004 }).p)), d1 = await p.evaluate(() => app._pk.scr(app.asmPointW(app._pk.ids.s, app.asm.pairs[0].ka, { s: -0.05 }).p));
  const cam0 = await p.evaluate(() => app.cam.position.toArray());
  await p.mouse.move(d0[0], d0[1]); await p.mouse.down(); await p.mouse.move((d0[0] + d1[0]) / 2, (d0[1] + d1[1]) / 2, { steps: 5 }); await p.mouse.move(d1[0], d1[1], { steps: 5 }); await frame(); await p.mouse.up(); await sleep(60); await frame();
  R.dr = await p.evaluate(c0 => { const A = app, pr = A.asm && A.asm.pairs[0]; return { open: !!A.asm, s: pr && pr.pa.s, num: document.getElementById('asmSA') && document.getElementById('asmSA').value, camSame: A.cam.position.toArray().every((v, i) => Math.abs(v - c0[i]) < 1e-9), gz: A._pk.noGz() }; }, cam0);
  R.g = await p.evaluate(() => { const A = app, K = A._pk; A.asmSet(0, { pa: { s: 0 } }); const r = A.asmCommit(), S = A.part(K.ids.s), G = A.part(K.ids.g), ax = K.ax(S.id, [0, 0, 1]), d = new K.V(...G.p).sub(new K.V(...S.p)); return { t: d.dot(ax), perp: d.addScaledVector(ax, -d.dot(ax)).length(), errs: r.errs, ground: K.ground() }; });
  check('轴身位置：快捷按钮 后端/中点/前端、滑块、mm 与 % 输入互相同步', R.q.s1 === 0 && Math.abs(R.q.s2 + 0.04) < 1e-9 && R.q.num === '-40.0' && R.q.pct === '30' && R.q.still && Math.abs(R.q.s3 - 0.05) < 1e-9 && R.q.buttons.join() === '后端,中点,前端', JSON.stringify(R.q));
  check('在 3D 中沿轴身拖动连接点（面板保持打开、视角不动、不拖走零件），齿轮装在轴中点', R.dr.open && R.dr.s < -0.03 && R.dr.s > -0.07 && R.dr.camSame && R.dr.gz && Math.abs(R.g.t) < 5e-4 && R.g.perp < 5e-4 && !R.g.errs.length && R.g.ground < 0.001, JSON.stringify({ dr: R.dr, g: R.g }));

  // ---------- 3. suggestions: shaft -> plate side pin (hinge), motor -> shaft (coupling), one click ----------
  R.sp = await p.evaluate(() => { const A = app, K = A._pk; K.clear(); const pl = A.placePart('plate', { x: 0, y: 0.05, z: 0 }), s = A.placePart('shaft', { x: 0.1, y: 0.05, z: 0.25 }); K.open(pl.id, s.id); const pr = A.asm.pairs[0];
    const rows = [...document.querySelectorAll('#asmPanel .sugR')].map(r => r.textContent); const sg = pr.sugg.map(x => A.asmEntry(pr.a, x.ka).role + '>' + A.asmEntry(pr.b, x.kb).role + ':' + x.type);
    document.querySelectorAll('#asmPanel .sugR')[1].click(); const t1 = A.asm.pairs[0].type, on1 = document.querySelectorAll('#asmPanel .sugR')[1].classList.contains('on');
    document.querySelectorAll('#asmPanel .sugR')[0].click(); const r = A.asmCommit(); const j = A.M.joints.find(x => r.joints.includes(x.id)); return { rows: rows.length, sg, t1, on1, joint: j && j.type, errs: r.errs, gap: A.editGap(), ground: K.ground() }; });
  R.ms = await p.evaluate(() => { const A = app, K = A._pk; K.clear(); const s = A.placePart('shaft', { x: 0, y: 0.06, z: 0 }), m = A.placePart('motor', { x: 0.25, y: 0, z: 0.1 }); K.open(s.id, m.id); const pr = A.asm.pairs[0];
    const sg = pr.sugg.map(x => A.asmEntry(pr.a, x.ka).role + '>' + A.asmEntry(pr.b, x.kb).role + ':' + x.type); document.querySelector('#asmPanel .sugR').click(); const r = A.asmCommit(); const j = A.M.joints.find(x => r.joints.includes(x.id)); return { sg, joint: j && j.type, errs: r.errs, gap: A.editGap(), ground: K.ground() }; });
  check('推荐连接：轴+板 → 轴端插板侧面销（铰链）排第一，点击行即应用并高亮', R.sp.rows >= 2 && R.sp.sg[0] === 'pin>axleEnd:revolute' && R.sp.on1 && R.sp.joint === 'revolute' && !R.sp.errs.length && R.sp.gap < 5e-4 && R.sp.ground < 0.001, JSON.stringify(R.sp));
  check('推荐连接：电机+轴 → 输出轴联轴（铰链/电机驱动），一键应用', R.ms.sg.some(x => /axleEnd>shaftOut:revolute/.test(x)) && R.ms.joint === 'revolute' && !R.ms.errs.length && R.ms.gap < 5e-4 && R.ms.ground < 0.001, JSON.stringify(R.ms));

  // ---------- 4. arbitrary point on a plate face (real click), eccentric point on a wheel face ----------
  await p.evaluate(() => { const A = app, K = A._pk; K.clear([0.05, 0.55, 0.45], [0, 0.03, 0]); const pl = A.placePart('plate', { x: 0, y: 0.05, z: 0 }), b = A.placePart('bearing', { x: 0.3, y: 0.05, z: 0.3 }); A.select(null); K.ids = { pl: pl.id, b: b.id }; K.open(pl.id, b.id); });
  await frame();
  const pt = await p.evaluate(() => app._pk.scr(app._pk.local(app._pk.ids.pl, [0.083, 0.01, 0.047])));
  await p.mouse.move(pt[0], pt[1], { steps: 3 }); await sleep(250); await frame(); await p.mouse.click(pt[0], pt[1]); await frame();
  R.pf = await p.evaluate(() => { const A = app, K = A._pk, pr = A.asm.pairs[0]; const o = { ka: A.asmEntry(pr.a, pr.ka).name, pa: pr.pa, kb: A.asmEntry(pr.b, pr.kb).name, type: pr.type };
    const r = A.asmCommit(), B = A.part(K.ids.b), base = K.local(B.id, [0, -B.d.H, 0]), pl = A.part(K.ids.pl), loc = base.clone().sub(new K.V(...pl.p)).applyQuaternion(new K.Q(...pl.q).invert()); o.loc = loc.toArray().map(v => +v.toFixed(4)); o.errs = r.errs; return o; });
  check('板面任意点：3D 点击板顶面某处 → 轴承座底面装到该点（误差 <1mm）', /^顶面/.test(R.pf.ka) && Math.abs(R.pf.pa.u - 0.083) < 0.0015 && Math.abs(R.pf.pa.v + 0.047) < 0.0015 && Math.abs(R.pf.loc[0] - 0.083) < 0.0015 && Math.abs(R.pf.loc[2] - 0.047) < 0.0015 && Math.abs(R.pf.loc[1] - 0.01) < 0.0006 && !R.pf.errs.length, JSON.stringify(R.pf));
  await p.evaluate(() => { const A = app, K = A._pk; K.clear([0.0, 0.12, 0.5], [0, 0.1, 0]); const w = A.placePart('wheel', { x: 0, y: 0.1, z: 0 }, { fixed: true }), r = A.placePart('rod', { x: 0.3, y: 0.1, z: 0.2 }); A.select(null); K.ids = { w: w.id, r: r.id }; K.open(w.id, r.id); });
  await frame();
  const ep = await p.evaluate(() => app._pk.scr(app._pk.local(app._pk.ids.w, [0.05, 0.0, 0.015])));
  await p.mouse.move(ep[0], ep[1], { steps: 3 }); await sleep(250); await frame(); await p.mouse.click(ep[0], ep[1]); await frame();
  R.ec = await p.evaluate(() => { const A = app, K = A._pk, pr = A.asm.pairs[0], o = { ka: A.asmEntry(pr.a, pr.ka).name, pa: pr.pa, type: pr.type };
    A.asmSet(0, { kb: K.ent(K.ids.r, /^端孔A/).key, pb: {} }); const r = A.asmCommit(), j = A.M.joints.find(x => r.joints.includes(x.id)); const w = A.part(K.ids.w);
    o.joint = j && j.type; o.ecc = j ? Math.hypot(j.la[0], j.la[1]) : -1; o.gap = A.editGap(); o.errs = r.errs; return o; });
  check('轮面偏心点：3D 点击轮正面偏心处 → 连杆端孔铰接在该偏心点（曲柄销）', /^正面/.test(R.ec.ka) && Math.abs(Math.hypot(R.ec.pa.u, R.ec.pa.v) - 0.05) < 0.002 && R.ec.joint === 'revolute' && Math.abs(R.ec.ecc - 0.05) < 0.002 && R.ec.gap < 5e-4 && !R.ec.errs.length, JSON.stringify(R.ec));

  // ---------- 5. rod-rod fully manual: any point, any type, angle between the bars ----------
  R.rr = await p.evaluate(() => { const A = app, K = A._pk, out = {};
    const ang = (a, b, axA, axB) => { const x = K.ax(a, axA), y = K.ax(b, axB); return Math.acos(Math.min(1, Math.abs(x.dot(y)))) * 180 / Math.PI; };
    const mk = () => { K.clear(); const a = A.placePart('rod', { x: 0, y: 0.05, z: 0 }), b = A.placePart('rod', { x: 0.1, y: 0.05, z: 0.3 }); K.open(a.id, b.id); return [a.id, b.id]; };
    const ptGap = (pr) => { const wa = A.asmPointW(pr.a, pr.ka, pr.pa).p, wb = A.asmPointW(pr.b, pr.kb, pr.pb).p, n = A.asmPointW(pr.a, pr.ka, pr.pa).n, d = wb.clone().sub(wa); return { d: d.length(), radial: d.addScaledVector(n, -d.dot(n)).length() }; };
    // (a) middle to middle, fixed (weld), 90°: a cross
    let [a, b] = mk(); A.asmSet(0, { ka: K.ent(a, /^中点/).key, kb: K.ent(b, /^中点/).key, type: 'fixed', typeManual: true, rel: 90 });
    out.panel = { rel: !!document.getElementById('asmRel'), relBtns: [...document.querySelectorAll('#asmPanel .asmRelB')].map(x => x.textContent), cross: !!document.getElementById('asmCross'), types: [...document.querySelectorAll('#asmType option')].map(o => o.value) };
    let pr = A.asm.pairs[0], r = A.asmCommit(); out.mid = Object.assign({ ang: ang(a, b, [1, 0, 0], [1, 0, 0]), joint: (A.M.joints[0] || {}).type, errs: r.errs, ground: K.ground() }, ptGap(pr));
    // (b) end of B to the middle of A, hinge, angle 30°, via the panel controls (type select = manual)
    [a, b] = mk(); { const ls = document.getElementById('asmEA'); ls.value = K.ent(a, /^中点/).key; ls.dispatchEvent(new Event('change')); const lb = document.getElementById('asmEB'); lb.value = K.ent(b, /^端孔A/).key; lb.dispatchEvent(new Event('change'));
      const ts = document.getElementById('asmType'); ts.value = 'revolute'; ts.dispatchEvent(new Event('change')); const rl = document.getElementById('asmRel'); rl.value = '30'; rl.dispatchEvent(new Event('input')); }
    pr = A.asm.pairs[0]; r = A.asmCommit(); out.endMid = Object.assign({ ang: ang(a, b, [1, 0, 0], [1, 0, 0]), joint: (A.M.joints[0] || {}).type, errs: r.errs, gap: A.editGap(), manual: pr.typeManual }, ptGap(pr));
    // (c) crossing at 45° at an arbitrary point on both bodies (A at 25 %, B at +40 mm), ball joint chosen manually
    [a, b] = mk(); { const body = K.ent(a, /^杆身/).key, bodyB = K.ent(b, /^杆身/).key; A.asmSet(0, { ka: body, pa: { s: 0 }, kb: bodyB, pb: { s: 0.04 } });
      const pc = document.getElementById('asmSPA'); pc.value = '25'; pc.dispatchEvent(new Event('input')); const ts = document.getElementById('asmType'); ts.value = 'spherical'; ts.dispatchEvent(new Event('change'));
      document.querySelector('#asmPanel .asmRelB[data-v="45"]').click(); }
    pr = A.asm.pairs[0]; out.crossPrm = { pa: pr.pa.s, pb: pr.pb.s }; r = A.asmCommit(); out.cross = Object.assign({ ang: ang(a, b, [1, 0, 0], [1, 0, 0]), joint: (A.M.joints[0] || {}).type, errs: r.errs, gap: A.editGap() }, ptGap(pr));
    // (d) rod on a rod body, slider (prismatic) along A, parallel (0°)
    [a, b] = mk(); A.asmSet(0, { ka: K.ent(a, /^杆身/).key, pa: { s: 0.05 }, kb: K.ent(b, /^中点/).key, pb: {}, type: 'prismatic', typeManual: true, rel: 0 }); pr = A.asm.pairs[0]; r = A.asmCommit();
    { const j = A.M.joints[0], axw = j ? K.ax(j.a, j.axa) : null; out.slide = Object.assign({ ang: ang(a, b, [1, 0, 0], [1, 0, 0]), joint: j && j.type, along: axw ? Math.abs(axw.dot(K.ax(a, [1, 0, 0]))) : 0, errs: r.errs }, ptGap(pr)); }
    // (e) shaft x shaft crossing at 90° (shaft bodies), hinge: side by side, perpendicular
    K.clear(); const s1 = A.placePart('shaft', { x: 0, y: 0.08, z: 0 }), s2 = A.placePart('shaft', { x: 0.2, y: 0.08, z: 0.1 }); K.open(s1.id, s2.id);
    A.asmSet(0, { ka: K.ent(s1.id, /^轴身/).key, pa: { s: 0.02 }, kb: K.ent(s2.id, /^轴身/).key, pb: { s: 0 }, type: 'revolute', typeManual: true, rel: 90 }); r = A.asmCommit();
    { const P1 = A.part(s1.id), P2 = A.part(s2.id), a1 = K.ax(s1.id, [0, 0, 1]), a2 = K.ax(s2.id, [0, 0, 1]), c1 = new K.V(...P1.p).addScaledVector(a1, 0.02), c2 = new K.V(...P2.p), n = a1.clone().cross(a2).normalize();
      out.shafts = { ang: ang(s1.id, s2.id, [0, 0, 1], [0, 0, 1]), sep: Math.abs(c2.clone().sub(c1).dot(n)), inPlane: c2.clone().sub(c1).addScaledVector(n, -c2.clone().sub(c1).dot(n)).length(), joint: (A.M.joints[0] || {}).type, errs: r.errs, gap: A.editGap(), ground: K.ground() }; }
    // (f) manual type sticks when the point is re-picked
    [a, b] = mk(); { const ts = document.getElementById('asmType'); ts.value = 'spherical'; ts.dispatchEvent(new Event('change')); A.asmPickHit({ side: 'a', e: K.ent(a, /^端孔B/), prm: {}, pid: a }); out.sticky = A.asm.pairs[0].type; A.asmCancel(true); }
    return out; });
  check('杆-杆手动：面板始终提供连接类型（固定/铰链/滑动/球铰…）、两杆夹角（平行/45°/90°/自定义）和交叉连接', ['fixed', 'revolute', 'prismatic', 'spherical'].every(t => R.rr.panel.types.includes(t)) && R.rr.panel.rel && R.rr.panel.cross && R.rr.panel.relBtns.join() === '平行,45°,90°,自动' && R.rr.sticky === 'spherical', JSON.stringify({ panel: R.rr.panel, sticky: R.rr.sticky }));
  check('杆-杆：中点对中点 固定（焊接）90° 十字', Math.abs(R.rr.mid.ang - 90) < 0.1 && R.rr.mid.radial < 5e-4 && R.rr.mid.joint === 'fixed' && !R.rr.mid.errs.length && R.rr.mid.ground < 0.001, JSON.stringify(R.rr.mid));
  check('杆-杆：B 端孔 → A 中点 铰链，夹角 30°（列表 + 类型下拉手动选择）', Math.abs(R.rr.endMid.ang - 30) < 0.1 && R.rr.endMid.radial < 5e-4 && R.rr.endMid.joint === 'revolute' && R.rr.endMid.manual && R.rr.endMid.gap < 5e-4 && !R.rr.endMid.errs.length, JSON.stringify(R.rr.endMid));
  check('杆-杆：杆身任意点（A 25% / B +40mm）45° 交叉，球铰', Math.abs(R.rr.crossPrm.pa + 0.075) < 1e-9 && Math.abs(R.rr.crossPrm.pb - 0.04) < 1e-9 && Math.abs(R.rr.cross.ang - 45) < 0.1 && R.rr.cross.radial < 5e-4 && R.rr.cross.joint === 'spherical' && R.rr.cross.gap < 5e-4 && !R.rr.cross.errs.length, JSON.stringify({ p: R.rr.crossPrm, c: R.rr.cross }));
  check('杆-杆：沿 A 杆身滑动副（平行），滑动方向沿杆', R.rr.slide.ang < 0.1 && R.rr.slide.joint === 'prismatic' && R.rr.slide.along > 0.999 && R.rr.slide.radial < 5e-4 && !R.rr.slide.errs.length, JSON.stringify(R.rr.slide));
  check('轴-轴：轴身任意点 90° 交叉铰链（两轴垂直、并排错开 r1+r2，不重叠）', Math.abs(R.rr.shafts.ang - 90) < 0.1 && Math.abs(R.rr.shafts.sep - 0.0125) < 6e-4 && R.rr.shafts.inPlane < 5e-4 && R.rr.shafts.joint === 'revolute' && R.rr.shafts.gap < 5e-4 && !R.rr.shafts.errs.length && R.rr.shafts.ground < 0.001, JSON.stringify(R.rr.shafts));

  // ---------- 6. focus (Tab), middle mouse only orbits (never picks) ----------
  await p.evaluate(() => { const A = app, K = A._pk; K.clear(); const s = A.placePart('shaft', { x: 0, y: 0.05, z: 0 }, { d: { L: 0.2 } }), w = A.placePart('wheel', { x: 0.2, y: 0.05, z: 0 }); A.select(null); K.ids = { s: s.id, w: w.id }; K.open(s.id, w.id); document.getElementById('cv').focus && document.body.focus(); });
  await frame(); await p.keyboard.press('Tab'); const f1 = await p.evaluate(() => app.asm.focus); await p.keyboard.press('Tab'); const f2 = await p.evaluate(() => app.asm.focus);
  const bk = await p.evaluate(() => app._pk.entScr(app._pk.ids.s, /^后端/)); const before = await p.evaluate(() => JSON.stringify(app.asm.pairs[0]) + app.cam.position.toArray().join());
  await p.mouse.move(bk[0], bk[1]); await p.mouse.down({ button: 'middle' }); await p.mouse.move(bk[0] + 60, bk[1] + 20, { steps: 6 }); await p.mouse.up({ button: 'middle' }); await frame();
  R.mm = await p.evaluate(b => { const A = app, pr = JSON.stringify(A.asm.pairs[0]); return { open: !!A.asm, samePair: pr === b.slice(0, pr.length), camMoved: !b.endsWith(A.cam.position.toArray().join()) }; }, before);
  check('Tab 切换 A/B 选点焦点；中键拖动只旋转视角、不选点', f1 === 'b' && f2 === 'a' && R.mm.open && R.mm.samePair && R.mm.camMoved, JSON.stringify({ f1, f2, mm: R.mm }));
  await p.evaluate(() => app.asmCancel(true));

  // ---------- screenshots of the new flow ----------
  const shot = async (n) => { await frame(); await sleep(120); await frame(); await p.screenshot({ path: OUT + n }); console.log('saved ' + n); };
  await p.evaluate(() => { const A = app, K = A._pk; K.clear([0.3, 0.26, 0.36], [0.0, 0.05, 0.0]); const s = A.placePart('shaft', { x: 0, y: 0.06, z: 0 }, { d: { L: 0.22 } }), w = A.placePart('wheel', { x: 0.2, y: 0.06, z: 0.05 }); A.select(null); K.ids = { s: s.id, w: w.id }; K.open(s.id, w.id); document.getElementById('tipbar').style.display = 'none'; });
  { const fr2 = await p.evaluate(() => app._pk.entScr(app._pk.ids.s, /^前端/)); await p.mouse.move(fr2[0] - 20, fr2[1]); await p.mouse.move(fr2[0], fr2[1], { steps: 3 }); await sleep(350); }
  await shot('screenshot-asm-pick-hover.png');
  { const fr2 = await p.evaluate(() => app._pk.entScr(app._pk.ids.s, /^前端/)); await p.mouse.click(fr2[0], fr2[1]); await p.mouse.move(10, 400); }
  await shot('screenshot-asm-pick-chosen.png');
  await p.evaluate(() => { const A = app, K = A._pk; K.clear([0.05, 0.42, 0.42], [0, 0.03, 0]); const pl = A.placePart('plate', { x: 0, y: 0.05, z: 0 }), s = A.placePart('shaft', { x: 0.1, y: 0.05, z: 0.25 }); A.select(null); K.ids = { pl: pl.id, s: s.id }; K.open(pl.id, s.id); });
  { const pt2 = await p.evaluate(() => app._pk.scr(app._pk.local(app._pk.ids.pl, [0.06, 0.01, -0.04]))); await p.mouse.move(pt2[0], pt2[1], { steps: 3 }); await sleep(350); }
  await shot('screenshot-asm-pick-plate.png');
  await p.evaluate(() => { const A = app, K = A._pk; K.clear([0.1, 0.5, 0.45], [0.02, 0.05, 0]); const a = A.placePart('rod', { x: 0, y: 0.05, z: 0 }), b = A.placePart('rod', { x: 0.1, y: 0.05, z: 0.3 }); A.select(null); K.open(a.id, b.id);
    A.asmSet(0, { ka: K.ent(a.id, /^杆身/).key, pa: { s: -0.06 }, kb: K.ent(b.id, /^中点/).key, pb: {}, type: 'revolute', typeManual: true, rel: 60 }); });
  await p.mouse.move(10, 400); await shot('screenshot-asm-pick-rods.png');
  await p.evaluate(() => { app.asmCancel(true); document.getElementById('tipbar').style.display = ''; });
};
