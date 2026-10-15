// Direct-assembly tests with real mouse events (palette drag-drop, part dragging). Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const sleep = t => new Promise(r => setTimeout(r, t));
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const scr = w => p.evaluate(w => { const s = app.toScreen(new app._V(w[0], w[1], w[2])); return [s.x, s.y]; }, w);
  await p.evaluate(() => { app._V = app.cam.position.constructor; });
  const setup = async (pos, tgt) => { await p.evaluate((pos, tgt) => { document.getElementById('btnClear').click(); app.reset && app.mode !== 'edit' && app.reset(); app.cam.position.set(...pos); app.orbit.target.set(...tgt); app.orbit.update(); app.select(null); }, pos, tgt); await frame(); };
  // drag a palette item onto a screen point with real mouse events; optional callback before release
  const dropAt = async (type, xy, beforeUp) => {
    const box = await (await p.$(`.item[data-type="${type}"]`)).boundingBox();
    await p.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await p.mouse.down();
    await p.mouse.move(xy[0] - 30, xy[1] + 25, { steps: 6 }); await p.mouse.move(xy[0], xy[1], { steps: 4 }); await frame();
    if (beforeUp) await beforeUp();
    await p.mouse.up(); await frame();
    return p.evaluate(() => app.M.parts[app.M.parts.length - 1].id);
  };
  const mdrag = async (from, to, steps = 12) => { await p.mouse.move(from[0], from[1]); await p.mouse.down(); await p.mouse.move(to[0], to[1], { steps }); await frame(); await p.mouse.up(); await frame(); };
  const conn = (id, name) => p.evaluate((id, name) => { const P = app.part(id); return app._conn(P, name); }, id, name);
  await p.evaluate(() => { app._conn = (P, name) => { const c = app._connectors(P).find(c => c.n === name); const v = new app._V(...c.p).applyQuaternion(new app.cam.quaternion.constructor(...P.q)).add(new app._V(...P.p)); return v.toArray(); }; });

  // ---------- 1. pendulum: base, then rod onto the pin (no joint tool) ----------
  await setup([0.25, 0.45, 1.5], [0.15, 0.35, 0]);
  const base = await dropAt('base', await scr([0, 0, 0]));
  const pin = await conn(base, '轴销(前)');
  let shotDone = false;
  const rod = await dropAt('rod', (await scr(pin)).map((v, i) => v + (i ? 6 : -5)), async () => {
    await p.screenshot({ path: OUT + 'screenshot-snap-preview.png' }); shotDone = true;
  });
  const pend = await p.evaluate((base, rod) => {
    const j = app.M.joints.find(j => j.type === 'revolute' && ((j.a === base && j.b === rod) || (j.a === rod && j.b === base)));
    const R = app.part(rod), B = app.part(base), pinW = app._conn(B, '轴销(前)'), hole = app._conn(R, '端孔A');
    const lbl = document.getElementById('stMsg').textContent;
    const out = { joints: app.M.joints.length, rev: !!j, holeErr: Math.hypot(hole[0] - pinW[0], hole[1] - pinW[1]) * 1000, zOff: (hole[2] - pinW[2]) * 1000, msg: lbl, undoDepth: app.undoS.length };
    // physical pendulum (uniform box bar, pivot at end, released horizontal => 90° amplitude)
    const L = R.d.L, w = R.d.w, d = L / 2, g = 9.81, T0 = 2 * Math.PI * Math.sqrt(((L * L + w * w) / 12 + d * d) / (g * d));
    let a = 1, b = Math.SQRT1_2; for (let i = 0; i < 20; i++) [a, b] = [(a + b) / 2, Math.sqrt(a * b)]; const K = Math.PI / (2 * a);
    out.ideal = T0 * 2 * K / Math.PI;
    app.startRun(); const piv = pinW[0]; let prev = null; const cr = [];
    for (let i = 0; i < 240 * 8; i++) { app.sim.substep(); const x = app.sim.partPose(rod).p.x - piv; if (prev !== null && prev > 0 && x <= 0) cr.push(app.sim.time); prev = x; }
    out.T = (cr[cr.length - 1] - cr[0]) / (cr.length - 1); out.n = cr.length; out.gap = app.sim.maxGap() * 1000; app.reset();
    return out;
  }, base, rod);
  const pe = Math.abs(pend.T - pend.ideal) / pend.ideal * 100;
  check('直接装配：拖底座+拖连杆到轴销 → 自动铰链', pend.rev && pend.joints === 1 && pend.holeErr < 0.01 && shotDone, JSON.stringify({ rev: pend.rev, holeErr_mm: +pend.holeErr.toFixed(4), zOff_mm: +pend.zOff.toFixed(2), msg: pend.msg }));
  check('直接装配的摆周期 (90°大摆角理论值, ±2%)', pe < 2 && pend.gap < 1, `T=${pend.T.toFixed(4)}s ideal=${pend.ideal.toFixed(4)}s err=${pe.toFixed(2)}% crossings=${pend.n} gap=${pend.gap.toFixed(4)}mm`);
  // undo = one step for place+joint
  const un = await p.evaluate(() => { const n0 = [app.M.parts.length, app.M.joints.length]; app.undo(); const n1 = [app.M.parts.length, app.M.joints.length]; app.redo(); const n2 = [app.M.parts.length, app.M.joints.length]; return { n0, n1, n2 }; });
  check('自动装配撤销为一步（零件+连接一起撤销/重做）', un.n1[0] === 1 && un.n1[1] === 0 && un.n2[0] === 2 && un.n2[1] === 1, JSON.stringify(un));
  // immediate response: drag rod free end with the mouse -> rotates about the hinge
  const endB = await conn(rod, '端孔B'), pinW = await conn(base, '轴销(前)');
  const sB = await scr(endB);
  const dbg0 = await p.evaluate(xy => { const e = { clientX: xy[0], clientY: xy[1] }; const h = app.pickAt(e); return { h: h.pid, axis: app.tc.axis, sel: app.sel }; }, sB);
  await mdrag(sB, [sB[0] - 60, sB[1] + 110]);
  const rot = await p.evaluate((rod, base) => { const R = app.part(rod), h = app._conn(R, '端孔A'), pw = app._conn(app.part(base), '轴销(前)'), e = app._conn(R, '端孔B'); return { ang: Math.atan2(e[1] - h[1], e[0] - h[0]) * 180 / Math.PI, gap: app.editGap() * 1000, pivErr: Math.hypot(h[0] - pw[0], h[1] - pw[1]) * 1000, msg: document.getElementById('stMsg').textContent, joints: app.M.joints.length }; }, rod, base);
  check('装配后立即可拖动：连杆绕铰链转动', rot.ang < -20 && rot.gap < 1 && rot.pivErr < 0.5 && rot.joints === 1, JSON.stringify(rot) + JSON.stringify(dbg0));
  void pinW;
  // strong pull detaches
  const e2 = await conn(rod, '端孔B'), h2 = await conn(rod, '端孔A'); const s2 = await scr(e2), sh = await scr(h2);
  const dir = [s2[0] - sh[0], s2[1] - sh[1]], dl = Math.hypot(...dir);
  await mdrag(s2, [s2[0] + dir[0] / dl * 300, s2[1] + dir[1] / dl * 300], 20);
  const det = await p.evaluate(() => ({ joints: app.M.joints.length, parts: app.M.parts.length, msg: document.getElementById('stMsg').textContent }));
  check('用力拉开 → 拆开连接', det.joints === 0 && det.parts === 2, JSON.stringify(det));
  const del = await p.evaluate((rod) => { app.undo(); const j0 = app.M.joints.length; app.select({ p: rod }); app.deleteSel(); return { before: j0, after: app.M.joints.length, parts: app.M.parts.length }; }, rod);
  check('删除零件同时删除其连接', del.before === 1 && del.after === 0 && del.parts === 1, JSON.stringify(del));

  // ---------- 2. gear train: base, motor on base, gear on motor, gear z40 next to it ----------
  await setup([0.12, 0.75, 1.3], [0.1, 0.5, 0]);
  const b2 = await dropAt('base', await scr([0, 0, 0]));
  const mot = await dropAt('motor', await scr(await conn(b2, '顶面')));
  const g1 = await dropAt('gear', await scr(await conn(mot, '输出轴')));
  await p.evaluate(() => { app._z0 = app.DEFS.gear.d.z; app.DEFS.gear.d.z = 40; });
  const mesh = await p.evaluate((g1) => { const G = app.part(g1); return [G.p[0] + 0.12, G.p[1] + 0.01, G.p[2]]; }, g1);
  const g2 = await dropAt('gear', (await scr(mesh)).map((v, i) => v + (i ? -8 : 10)));
  await p.evaluate(() => { app.DEFS.gear.d.z = app._z0; });
  const gt = await p.evaluate((mot, g1, g2, b2) => {
    const M = app.M, G1 = app.part(g1), G2 = app.part(g2), Mo = app.part(mot);
    const dist = Math.hypot(G1.p[0] - G2.p[0], G1.p[1] - G2.p[1], G1.p[2] - G2.p[2]);
    const out = { motorOnBase: M.joints.some(j => j.type === 'fixed' && [j.a, j.b].includes(mot) && [j.a, j.b].includes(b2)), g1rev: M.joints.some(j => j.type === 'revolute' && [j.a, j.b].includes(mot) && [j.a, j.b].includes(g1)),
      mesh: M.couplings.filter(c => c.type === 'gear').length, dist_mm: dist * 1000, ideal_mm: 0.004 * (20 + 40) / 2 * 1000, z2: G2.d.z, rpmSet: Mo.motor.rpm };
    app.startRun(); app.simulate(2); let s = 0, n = 0, w1s = 0;
    const w = id => { const v = app.sim.pb[id].body.angvel(); const q = app.sim.partPose(id).q; const z = new app._V(0, 0, 1).applyQuaternion(q); return v.x * z.x + v.y * z.y + v.z * z.z; };
    for (let i = 0; i < 240; i++) { app.sim.substep(); s += w(g2) / w(g1); w1s += w(g1); n++; }
    out.ratio = s / n; out.rpm1 = w1s / n * 30 / Math.PI; out.gap = app.sim.maxGap() * 1000; app.reset(); return out;
  }, mot, g1, g2, b2);
  check('直接装配：电机→底座、齿轮→电机轴、齿轮→齿轮 自动啮合(精确中心距)', gt.motorOnBase && gt.g1rev && gt.mesh === 1 && Math.abs(gt.dist_mm - gt.ideal_mm) < 0.01, JSON.stringify({ ...gt, ratio: undefined }));
  check('自动齿轮系运行：速比 -z1/z2 = -0.5，电机默认 30rpm', Math.abs(gt.ratio + 0.5) < 0.005 && Math.abs(Math.abs(gt.rpm1) - 30) < 0.6, `ω2/ω1=${gt.ratio.toFixed(5)} 输入 ${gt.rpm1.toFixed(2)} rpm gap=${gt.gap.toFixed(4)}mm`);
  // screenshot of the auto-built assembly running
  await p.evaluate((g2) => { app.select({ p: g2 }); app.startRun(); app.simulate(1.2); app.updRun(); }, g2); await sleep(400); await frame();
  await p.screenshot({ path: OUT + 'screenshot-auto-assembly-run.png' });
  await p.evaluate(() => app.reset());

  // ---------- 3. wheel onto shaft; shaft onto bearing pin ----------
  await setup([0.3, 0.5, 1.4], [0.1, 0.3, 0]);
  const b3 = await dropAt('base', await scr([0, 0, 0]));
  const sh3 = await dropAt('shaft', await scr(await conn(b3, '轴销(前)')));
  const shc = await p.evaluate(sh => { const S = app.part(sh); return [S.p[0], S.p[1], S.p[2] + 0.02]; }, sh3);
  const wh = await dropAt('wheel', await scr(shc));
  const wt = await p.evaluate((b3, sh3, wh) => { const M = app.M, W = app.part(wh), S = app.part(sh3); const ax = new app._V(0, 0, 1).applyQuaternion(new app.cam.quaternion.constructor(...S.q));
    const rel = new app._V(W.p[0] - S.p[0], W.p[1] - S.p[1], W.p[2] - S.p[2]); const off = rel.clone().addScaledVector(ax, -rel.dot(ax)).length();
    const r = { shaftRev: M.joints.some(j => j.type === 'revolute' && [j.a, j.b].includes(b3) && [j.a, j.b].includes(sh3)), wheelFixed: M.joints.some(j => j.type === 'fixed' && [j.a, j.b].includes(wh) && [j.a, j.b].includes(sh3)), radialOff_mm: off * 1000 };
    app.startRun(); app.simulate(1); r.gap = app.sim.maxGap() * 1000; app.reset(); return r; }, b3, sh3, wh);
  check('直接装配：轴→轴承座(铰链)，轮→轴(固定在轴上，同轴)', wt.shaftRev && wt.wheelFixed && wt.radialOff_mm < 0.01 && wt.gap < 1, JSON.stringify(wt));

  // ---------- 4. slider onto rail, then drag it: slides along the rail ----------
  await setup([0.0, 0.6, 1.3], [0.0, 0.05, 0]);
  const wall = await dropAt('wall', await scr([0, 0, 0]));
  const top = await conn(wall, '顶面');
  const blk = await dropAt('block', await scr([top[0] + 0.05, top[1], top[2]]));
  const sl0 = await p.evaluate((wall, blk) => { const B = app.part(blk); return { pri: app.M.joints.some(j => j.type === 'prismatic' && [j.a, j.b].includes(wall) && [j.a, j.b].includes(blk)), p: B.p.slice(), bottom: B.p[1] - B.d.h / 2, wallTop: app.part(wall).p[1] + app.part(wall).d.h / 2 }; }, wall, blk);
  const sb = await scr(sl0.p); await mdrag(sb, [sb[0] + 120, sb[1] - 40]);
  const sl1 = await p.evaluate(blk => app.part(blk).p.slice(), blk);
  const sl = { pri: sl0.pri, seat_mm: (sl0.bottom - sl0.wallTop) * 1000, dx_mm: (sl1[0] - sl0.p[0]) * 1000, dy_mm: (sl1[1] - sl0.p[1]) * 1000, dz_mm: (sl1[2] - sl0.p[2]) * 1000 };
  check('直接装配：滑块→导轨(滑动副)，拖动时沿导轨滑动', sl.pri && Math.abs(sl.seat_mm) < 0.01 && sl.dx_mm > 20 && Math.abs(sl.dy_mm) < 1 && Math.abs(sl.dz_mm) < 1, JSON.stringify(sl));

  // ---------- 5. lead screw: base, motor, screw on motor shaft, nut onto screw ----------
  await setup([1.2, 0.8, 0.5], [0, 0.5, 0.25]);
  const b5 = await dropAt('base', await scr([0, 0, 0]));
  const m5 = await dropAt('motor', await scr(await conn(b5, '顶面')));
  const sc = await dropAt('screw', await scr(await conn(m5, '输出轴')));
  const scMid = await p.evaluate(sc => { const S = app.part(sc); return [S.p[0], S.p[1], S.p[2] + 0.05]; }, sc);
  const nut = await dropAt('nut', await scr(scMid));
  const nt = await p.evaluate((m5, sc, nut) => { const M = app.M; const r = { screwRev: M.joints.some(j => j.type === 'revolute' && [j.a, j.b].includes(m5) && [j.a, j.b].includes(sc)), helix: M.couplings.filter(c => c.type === 'screw').length, nutPri: M.joints.some(j => j.type === 'prismatic' && j.a === nut) };
    app.startRun(); app.simulate(1); const wv = app.sim.pb[sc].body.angvel(), az = new app._V(0, 0, 1).applyQuaternion(app.sim.partPose(sc).q), w = wv.x * az.x + wv.y * az.y + wv.z * az.z, v = app.sim.pb[nut].body.linvel();
    r.v_mm_s = (v.x * az.x + v.y * az.y + v.z * az.z) * 1000; r.ideal_mm_s = app.part(sc).d.lead * w / (2 * Math.PI) * 1000; r.rpm = w * 30 / Math.PI; app.reset(); return r; }, m5, sc, nut);
  check('直接装配：丝杠→电机轴、螺母→丝杠 (螺旋副) 运行 v=导程·n', nt.screwRev && nt.helix === 1 && nt.nutPri && Math.abs(Math.abs(nt.v_mm_s) - Math.abs(nt.ideal_mm_s)) / Math.abs(nt.ideal_mm_s) < 0.01 && Math.abs(nt.ideal_mm_s) > 1, JSON.stringify(nt));

  // ---------- 6. Alt = free placement ----------
  await setup([0.25, 0.45, 1.5], [0.15, 0.35, 0]);
  const b6 = await dropAt('base', await scr([0, 0, 0]));
  const pin6 = await scr(await conn(b6, '轴销(前)'));
  await p.keyboard.down('Alt'); await dropAt('rod', pin6); await p.keyboard.up('Alt');
  const alt = await p.evaluate(() => ({ joints: app.M.joints.length, parts: app.M.parts.length }));
  check('按住 Alt 自由放置（不自动连接）', alt.joints === 0 && alt.parts === 2, JSON.stringify(alt));
  // trial run button
  const tr = await p.evaluate(async () => { app.select({ p: app.M.parts[1].id }); document.getElementById('bTrial').click(); const m = app.mode; await new Promise(r => setTimeout(r, 3400)); return { during: m, after: app.mode }; });
  check('“试运行 3 秒”按钮：运行后自动回到编辑', tr.during === 'run' && tr.after === 'edit', JSON.stringify(tr));
};
