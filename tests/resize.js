// Resizing parts keeps connections matched (resize.js). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const mm = v => +(v * 1000).toFixed(4);
  // ---- 1. shaft in a fixed bearing, wheel fixed on the free end: L 100 -> 200 -> 60 ----
  const t1 = await p.evaluate(() => {
    const A = app; if (A.mode !== 'edit') A.reset(); document.getElementById('btnClear').click();
    const V = app._V, y = 0.0424;
    const br = A.newPart('bearing', [0, y, 0], null, { fixed: true }), sh = A.newPart('shaft', [0, y, 0.05], null, { d: { L: 0.1, r: 0.006 } }), wh = A.newPart('wheel', [0, y, 0.1], null, { d: { r: 0.03, t: 0.02 } });
    A.addJoint('revolute', br, sh, [0, y, 0], [0, 0, 1]); A.addJoint('fixed', sh, wh, [0, y, 0.1], [0, 0, 1]); A.commit(); A.rebuild();
    const st = () => ({ gap: mm(A.jointGapAll()), wheelZ: mm(A.part(wh.id).p[2]), backEnd: mm(A.part(sh.id).p[2] - A.part(sh.id).d.L / 2), bearing: A.part(br.id).p.map(mm), L: mm(A.part(sh.id).d.L) });
    const mm = v => +(v * 1000).toFixed(3); const out = { s0: st() }, u0 = A.undoS.length;
    out.r200 = A.resizePart(sh.id, { L: 0.2 }).ok; out.s200 = st();
    out.r60 = A.resizePart(sh.id, { L: 0.06 }).ok; out.s60 = st(); out.undoSteps = A.undoS.length - u0;
    A.undo(); out.u1 = st(); A.undo(); out.u2 = st(); A.redo(); out.redo = st();
    return out; });
  const ok1 = s => s.gap < 0.5 && Math.abs(s.backEnd) < 0.5 && Math.abs(s.wheelZ - s.L) < 0.5 && s.bearing[2] === 0;
  check('改轴长 100→200→60mm（轴承座固定，轮在轴端）：轴承端不动、轮随轴端移动、间隙 <0.5mm', t1.r200 && t1.r60 && ok1(t1.s200) && ok1(t1.s60) && t1.s200.wheelZ === 200 && t1.s60.wheelZ === 60, JSON.stringify({ s200: t1.s200, s60: t1.s60 }));
  check('改尺寸：每次修改一个撤销步，撤销/重做恢复尺寸与位置', t1.undoSteps === 2 && t1.u1.L === 200 && t1.u1.wheelZ === 200 && t1.u2.L === 100 && t1.u2.wheelZ === 100 && t1.redo.L === 200 && t1.u1.gap < 0.5 && t1.u2.gap < 0.5, JSON.stringify({ steps: t1.undoSteps, u1: t1.u1, u2: t1.u2, redo: t1.redo }));
  // ---- 1b. wheel part-way along the shaft keeps its distance from the anchored end; centre wheel stays centred ----
  const t1b = await p.evaluate(() => {
    const A = app; document.getElementById('btnClear').click(); const y = 0.0424;
    const br = A.newPart('bearing', [0, y, 0], null, { fixed: true }), sh = A.newPart('shaft', [0, y, 0.1], null, { d: { L: 0.2, r: 0.006 } }), w1 = A.newPart('wheel', [0, y, 0.03], null, { d: { r: 0.03, t: 0.01 } }), w2 = A.newPart('wheel', [0, y, 0.1], null, { d: { r: 0.03, t: 0.01 } }), w3 = A.newPart('wheel', [0, y, 0.17], null, { d: { r: 0.03, t: 0.01 } });
    A.addJoint('revolute', br, sh, [0, y, 0], [0, 0, 1]); for (const w of [w1, w2, w3]) A.addJoint('fixed', sh, w, w.p, [0, 0, 1]); A.commit(); A.rebuild();
    const r = A.resizePart(sh.id, { L: 0.3 }); const z = id => +(A.part(id).p[2] * 1000).toFixed(3);
    return { ok: r.ok, gap: A.jointGapAll() * 1000, w1: z(w1.id), w2: z(w2.id), w3: z(w3.id), sh: z(sh.id) }; });
  check('轴上中间位置的轮：保持到最近端的距离；中心处的轮保持居中', t1b.ok && t1b.gap < 0.5 && t1b.w1 === 30 && t1b.w2 === 150 && t1b.w3 === 270, JSON.stringify(t1b));
  // ---- 2. shaft between two bearings ----
  const t2 = await p.evaluate(() => {
    const A = app, y = 0.0424, out = {};
    const build = freeSecond => { document.getElementById('btnClear').click(); const b1 = A.newPart('bearing', [0, y, 0], null, { fixed: true }), b2 = A.newPart('bearing', [0, y, 0.1], null, { fixed: !freeSecond }), sh = A.newPart('shaft', [0, y, 0.05], null, { d: { L: 0.1, r: 0.006 } });
      A.addJoint('revolute', b1, sh, [0, y, 0], [0, 0, 1]); A.addJoint('revolute', b2, sh, [0, y, 0.1], [0, 0, 1]); A.commit(); A.rebuild(); return { b1, b2, sh }; };
    let s = build(false); let r = A.resizePart(s.sh.id, { L: 0.16 });
    out.bothFixed = { ok: r.ok, strategy: r.strategy, gap: A.jointGapAll() * 1000, b2z: A.part(s.b2.id).p[2], shz: A.part(s.sh.id).p[2] };
    const before = A.snapshot(), nU = A.undoS.length; r = A.resizePart(s.sh.id, { L: 0.05 });
    out.tooShort = { ok: r.ok, msg: r.msg, unchanged: A.snapshot() === before, noUndo: A.undoS.length === nU, gap: A.jointGapAll() * 1000, status: document.getElementById('stMsg').textContent };
    s = build(true); r = A.resizePart(s.sh.id, { L: 0.2 });
    out.oneFree = { ok: r.ok, gap: A.jointGapAll() * 1000, b1z: A.part(s.b1.id).p[2], b2z: A.part(s.b2.id).p[2] };
    return out; });
  check('两个固定轴承座之间的轴加长：轴承座不动，轴绕中心两端伸长', t2.bothFixed.ok && t2.bothFixed.gap < 0.5 && Math.abs(t2.bothFixed.b2z - 0.1) < 1e-9 && Math.abs(t2.bothFixed.shz - 0.05) < 1e-6, JSON.stringify(t2.bothFixed));
  check('两个固定轴承座之间的轴缩短到小于跨距：拒绝并给出中文提示，模型不变、无间隙', !t2.tooShort.ok && /无法修改/.test(t2.tooShort.msg) && t2.tooShort.unchanged && t2.tooShort.noUndo && t2.tooShort.gap < 0.5 && /无法修改/.test(t2.tooShort.status), JSON.stringify(t2.tooShort));
  check('一个固定、一个自由轴承座：加长后自由轴承座随轴端外移', t2.oneFree.ok && t2.oneFree.gap < 0.5 && Math.abs(t2.oneFree.b1z) < 1e-9 && Math.abs(t2.oneFree.b2z - 0.2) < 5e-4, JSON.stringify(t2.oneFree));
  // ---- 3. four-bar: coupler length change ----
  const t3 = await p.evaluate(() => {
    const A = app, i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '四连杆') - 1, out = {};
    A.loadPreset(i); const c = A.byName('连杆'), L0 = c.d.L, base = A.byName('机架').p.slice(), mot = A.byName('电机').p.slice();
    let r = A.resizePart(c.id, { L: L0 * 1.08 }); out.ok = { ok: r.ok, gap: A.jointGapAll() * 1000, L: A.byName('连杆').d.L / L0, baseFixed: A.byName('机架').p.every((v, k) => Math.abs(v - base[k]) < 1e-9) && A.byName('电机').p.every((v, k) => Math.abs(v - mot[k]) < 1e-9), audit: A.connAudit().maxGap * 1000 };
    A.startRun(); A.simulate(2); out.run = { gap: A.sim.maxGap() * 1000 }; A.reset();
    const snap = A.snapshot(); r = A.resizePart(A.byName('连杆').id, { L: 2 }); out.bad = { ok: r.ok, msg: r.msg, unchanged: A.snapshot() === snap, gap: A.jointGapAll() * 1000 };
    return out; });
  check('四连杆改连杆长度 +8%：机构重新求解姿态，机架/电机不动，间隙 <0.5mm，可运行', t3.ok.ok && t3.ok.gap < 0.5 && t3.ok.baseFixed && t3.ok.audit < 1 && t3.run.gap < 1, JSON.stringify(t3));
  check('四连杆改为不可装配的连杆长度：中文提示“无法修改”，保持原尺寸、无间隙', !t3.bad.ok && /无法修改.*已恢复原尺寸/.test(t3.bad.msg) && t3.bad.unchanged && t3.bad.gap < 0.5, JSON.stringify(t3.bad));
  // ---- 4. gear tooth count 20 -> 30 ----
  const t4 = await p.evaluate(() => {
    const A = app, i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '两级齿轮减速器') - 1;
    A.loadPreset(i); const g = n => A.byName(n), cd = (a, b) => Math.hypot(g(a).p[0] - g(b).p[0], g(a).p[1] - g(b).p[1]);
    const r = A.resizePart(g('齿轮1 z20').id, { z: 30 }), m = 0.004;
    const out = { ok: r.ok, gap: A.jointGapAll() * 1000, err12: Math.abs(cd('齿轮1 z20', '齿轮2 z40') - m * 70 / 2) * 1000, err34: Math.abs(cd('齿轮3 z20', '齿轮4 z40') - m * 60 / 2) * 1000, audit: A.connAudit().maxGap * 1000, motorStill: Math.abs(g('电机').p[0]) < 1e-12 };
    A.startRun(); A.simulate(2); const w = n => A.sim.pb[g(n).id].body.angvel().z; let s = 0, k = 0; for (let t = 0; t < 120; t++) { A.sim.substep(); s += w('齿轮2 z40') / w('齿轮1 z20'); k++; } out.ratio = s / k; out.runGap = A.sim.maxGap() * 1000; A.reset();
    A.undo(); out.undoErr = Math.abs(cd('齿轮1 z20', '齿轮2 z40') - m * 60 / 2) * 1000; out.zUndo = g('齿轮1 z20').d.z; return out; });
  check('齿轮齿数 20→30：啮合齿轮移动到精确中心距(两级联动)，速比 -0.75，撤销恢复', t4.ok && t4.err12 < 1e-6 && t4.err34 < 1e-6 && t4.gap < 0.5 && t4.audit < 1 && t4.motorStill && Math.abs(t4.ratio + 0.75) < 0.0075 && t4.runGap < 1 && t4.zUndo === 20 && t4.undoErr < 1e-6, JSON.stringify(t4));
  // ---- 4b. pulley radius: belt ratio follows ----
  const t4b = await p.evaluate(() => {
    const A = app, i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '皮带传动') - 1; A.loadPreset(i);
    const r = A.resizePart(A.byName('主动带轮').id, { r: 0.04 }), rB = A.byName('从动带轮').d.r; A.startRun(); A.simulate(2);
    const w = n => A.sim.pb[A.byName(n).id].body.angvel().z, ratio = w('从动带轮') / w('主动带轮'), gap = A.sim.maxGap() * 1000; A.reset(); return { ok: r.ok, ratio, ideal: 0.04 / rB, gap }; });
  check('带轮半径修改：皮带速比随之更新', t4b.ok && Math.abs(t4b.ratio - t4b.ideal) / t4b.ideal < 0.01 && t4b.gap < 1, JSON.stringify(t4b));
  // ---- 5. plate shrink clamps mounted motors / bearings ----
  const t5 = await p.evaluate(() => {
    const A = app, i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '简易小车') - 1, out = {};
    const inside = pid => A.M.joints.filter(j => j.a === pid || j.b === pid).every(j => { const l = j.a === pid ? j.la : j.lb, P = A.part(pid); return Math.abs(l[0]) <= P.d.w / 2 + 1e-9 && Math.abs(l[2]) <= P.d.dp / 2 + 1e-9 && Math.abs(Math.abs(l[1]) - P.d.h / 2) < 1e-9; });
    // hand-built: free plate with two motors near the corners
    document.getElementById('btnClear').click(); const pl = A.newPart('plate', [0, 0.1, 0], null, { d: { w: 0.4, h: 0.02, dp: 0.3 } });
    for (const [x, z] of [[-0.17, 0.12], [0.15, -0.1]]) { const m = A.newPart('motor', [x, 0.11 + 0.0424, z], null, { fixed: false }); A.addJoint('fixed', pl, m, [x, 0.11, z], [0, 1, 0]); }
    A.commit(); A.rebuild(); let r = A.resizePart(pl.id, { w: 0.2, dp: 0.1 });
    out.hand = { ok: r.ok, gap: A.jointGapAll() * 1000, inside: inside(pl.id), audit: A.connAudit().maxGap * 1000, motors: A.M.parts.filter(P => P.type === 'motor').map(P => [+(P.p[0] * 1000).toFixed(1), +(P.p[2] * 1000).toFixed(1)]) };
    r = A.resizePart(pl.id, { h: 0.05 }); out.thick = { ok: r.ok, gap: A.jointGapAll() * 1000, audit: A.connAudit().maxGap * 1000 };
    // cart preset: shorter chassis, motors and bearings move in with the ends
    A.loadPreset(i); const ch = A.byName('车架'); r = A.resizePart(ch.id, { w: ch.d.w * 0.5 });
    out.cart = { ok: r.ok, gap: A.jointGapAll() * 1000, inside: inside(ch.id), audit: A.connAudit().maxGap * 1000 };
    A.startRun(); A.simulate(2); out.run = A.sim.maxGap() * 1000; A.reset();
    // narrower than the front axle span: the axle between the two bearings cannot follow -> clean rejection
    const snap = A.snapshot(); r = A.resizePart(A.byName('车架').id, { dp: 0.1 }); out.narrow = { ok: r.ok, unchanged: A.snapshot() === snap, msg: r.msg };
    return out; });
  check('板缩小：板上的电机/轴承座连接点夹紧在板内，所有连接贴合（手搭板 + 简易小车）', t5.hand.ok && t5.hand.gap < 0.5 && t5.hand.inside && t5.hand.audit < 1 && t5.thick.ok && t5.thick.gap < 0.5 && t5.thick.audit < 1 && t5.cart.ok && t5.cart.gap < 0.5 && t5.cart.inside && t5.cart.audit < 1 && t5.run < 1, JSON.stringify(t5));
  check('板窄于前轴两轴承座跨距：拒绝修改（中文提示），模型不变', !t5.narrow.ok && t5.narrow.unchanged && /无法修改/.test(t5.narrow.msg), JSON.stringify(t5.narrow));
  // ---- 6. UI: live preview while typing / scrubbing, one undo step on commit ----
  await p.evaluate(() => { const A = app; document.getElementById('btnClear').click(); const y = 0.0424;
    const br = A.newPart('bearing', [0, y, 0], null, { fixed: true }), sh = A.newPart('shaft', [0, y, 0.05], null, { name: '测试轴', d: { L: 0.1, r: 0.006 } }), wh = A.newPart('wheel', [0, y, 0.1], null, { name: '测试轮', d: { r: 0.03, t: 0.02 } });
    A.addJoint('revolute', br, sh, [0, y, 0], [0, 0, 1]); A.addJoint('fixed', sh, wh, [0, y, 0.1], [0, 0, 1]); A.commit(); A.rebuild(); A.select({ p: sh.id }); });
  await frame();
  const u0 = await p.evaluate(() => app.undoS.length);
  const inp = await p.$('#props input.dim[data-dim="L"]');
  await inp.click(); await p.evaluate(() => document.querySelector('#props input.dim[data-dim="L"]').select()); await p.keyboard.type('150');
  const live = await p.evaluate(() => ({ wz: app.byName('测试轮').p[2], L: app.byName('测试轴').d.L, undo: app.undoS.length, gap: app.jointGapAll() * 1000, focus: document.activeElement && document.activeElement.dataset.dim }));
  await p.keyboard.press('Enter'); await frame();
  const com = await p.evaluate(() => ({ wz: app.byName('测试轮').p[2], undo: app.undoS.length, gap: app.jointGapAll() * 1000 }));
  check('属性面板输入尺寸：实时预览（输入时轮已移动、焦点保留、不产生撤销步），回车提交为一个撤销步', Math.abs(live.wz - 0.15) < 1e-6 && live.undo === u0 && live.focus === 'L' && live.gap < 0.5 && Math.abs(com.wz - 0.15) < 1e-6 && com.undo === u0 + 1 && com.gap < 0.5, JSON.stringify({ live, com, u0 }));
  await p.evaluate(() => app.select({ p: app.byName('测试轴').id })); await frame();
  const lab = await (await p.$('#props input.dim[data-dim="L"]')).evaluateHandle(i => i.closest('.row').querySelector('label'));
  const bb = await lab.boundingBox(); const u1 = await p.evaluate(() => app.undoS.length);
  await p.mouse.move(bb.x + 10, bb.y + bb.height / 2); await p.mouse.down(); await p.mouse.move(bb.x + 40, bb.y + bb.height / 2, { steps: 5 });
  const mid = await p.evaluate(() => ({ L: app.byName('测试轴').d.L, wz: app.byName('测试轮').p[2], undo: app.undoS.length, gap: app.jointGapAll() * 1000 }));
  await p.mouse.move(bb.x + 80, bb.y + bb.height / 2, { steps: 5 }); await p.mouse.up(); await frame();
  const end = await p.evaluate(() => ({ L: app.byName('测试轴').d.L, wz: app.byName('测试轮').p[2], undo: app.undoS.length, gap: app.jointGapAll() * 1000 }));
  check('拖动尺寸标签连续调整：拖动中实时更新且连接贴合，松开后只增加一个撤销步', mid.L > 0.15 && mid.undo === u1 && mid.gap < 0.5 && Math.abs(mid.wz - mid.L) < 5e-4 && end.L > mid.L && end.undo === u1 + 1 && end.gap < 0.5 && Math.abs(end.wz - end.L) < 5e-4, JSON.stringify({ mid, end }));
  // ---- 7. randomized resizes on every preset ----
  const t7 = await p.evaluate(() => {
    const A = app, opts = [...document.querySelectorAll('#selPreset option')].slice(1); let seed = 12345; const rnd = () => ((seed = (Math.imul(seed, 1103515245) + 12345) >>> 0) % 1000003) / 1000003;
    const res = [];
    opts.forEach((o, i) => { A.loadPreset(i); let n = 0, rej = 0, worst = 0, worstAudit = 0, bad = [];
      for (let k = 0; k < 4; k++) { const P = A.M.parts[Math.floor(rnd() * A.M.parts.length)], D = A.DEFS[P.type], dims = D.dims.filter(d => d[2] !== 'num'); if (!dims.length) continue; const [key, , kind] = dims[Math.floor(rnd() * dims.length)];
        const f = 0.75 + rnd() * 0.6, v = kind === 'int' ? Math.max(3, Math.round(P.d[key] * f)) : P.d[key] * f, snap = A.snapshot(); const r = A.resizePart(P.id, { [key]: v });
        if (r.ok) { n++; const g = A.jointGapAll() * 1000, a = A.connAudit().maxGap * 1000; worst = Math.max(worst, g); worstAudit = Math.max(worstAudit, a); if (g >= 0.5 || a >= 1) bad.push(P.name + '.' + key); }
        else { rej++; if (A.snapshot() !== snap) bad.push('reject-changed ' + P.name); } }
      res.push([o.textContent, n, rej, +worst.toFixed(4), +worstAudit.toFixed(3), bad]); });
    return res; });
  for (const [n, ok, rej, g, a, bad] of t7) check('随机改尺寸后连接仍贴合（关节间隙 <0.5mm，可视审计 <1mm）: ' + n, bad.length === 0, `applied ${ok}, rejected ${rej}, max gap ${g}mm, audit ${a}mm${bad.length ? ' BAD ' + bad.join(',') : ''}`);
};
