// New features: gizmo on demand, 3D import (STL/OBJ/GLB) + 我的零件 + custom connectors, snap anywhere along rods / on any face,
// two-click 配合, joint editor (limits / spring / drive keys), multi-select + group + mirror, generic-part 4-wheel car. Used by run-tests.js
const fs = require('fs');
module.exports = async function (p, check, OUT) {
  const FX = __dirname + '/fixtures/';
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const scr = w => p.evaluate(w => { const s = app.toScreen(new app._V(w[0], w[1], w[2])); return [s.x, s.y]; }, w);
  const setup = async (pos, tgt) => { await p.evaluate((pos, tgt) => { if (app.mode !== 'edit') app.reset(); if (app.tool !== 'select') app.setTool('select'); document.getElementById('btnClear').click(); app.cam.position.set(...pos); app.orbit.target.set(...tgt); app.orbit.update(); app.select(null); app.gizmoOn = false; app.updGizmoBtns(); }, pos, tgt); await frame(); };
  const dropSel = async (sel, xy) => {
    const box = await (await p.$(sel)).boundingBox();
    await p.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await p.mouse.down();
    await p.mouse.move(xy[0] - 30, xy[1] + 25, { steps: 6 }); await p.mouse.move(xy[0], xy[1], { steps: 4 }); await frame();
    await p.mouse.up(); await frame();
    return p.evaluate(() => app.M.parts[app.M.parts.length - 1].id);
  };
  const click = async (xy, mods = []) => { for (const m of mods) await p.keyboard.down(m); await p.mouse.click(xy[0], xy[1]); for (const m of mods) await p.keyboard.up(m); await frame(); };
  const surf = async (pid, local) => scr(await p.evaluate((pid, l) => { const P = app.part(pid), Q = app.cam.quaternion.constructor; return new app._V(...l).applyQuaternion(new Q(...P.q)).add(new app._V(...P.p)).toArray(); }, pid, local));

  // ======================= 1. gizmo on demand =======================
  await setup([0.5, 0.5, 0.9], [0, 0.05, 0]);
  const g0 = await p.evaluate(() => { const P = app.placePart('block', { x: 0, y: 0, z: 0 }); app.select(null); return P.id; });
  await frame(); await click(await surf(g0, [0, 0.025, 0]));
  const gs1 = await p.evaluate(id => ({ sel: app.sel && app.sel.p === id, gizmo: !!app.tc.object }), g0);
  await p.keyboard.press('KeyG'); await frame();
  const gs2 = await p.evaluate(() => ({ gizmo: !!app.tc.object, mode: app.tc.mode, btn: document.getElementById('btnGizmo').classList.contains('on') }));
  await p.keyboard.press('KeyG'); await frame();
  const gs3 = await p.evaluate(() => !!app.tc.object);
  await p.keyboard.down('Shift'); await p.keyboard.press('KeyG'); await p.keyboard.up('Shift'); await frame();
  const gs4 = await p.evaluate(() => ({ gizmo: !!app.tc.object, mode: app.tc.mode }));
  await p.keyboard.down('Shift'); await p.keyboard.press('KeyG'); await p.keyboard.up('Shift'); await frame();
  // reselect another time: stays hidden (remembered off)
  await click([5 + (await (await p.$('#cv')).boundingBox()).x, 300]); await click(await surf(g0, [0, 0.025, 0]));
  const gs5 = await p.evaluate(() => !!app.tc.object);
  check('选中零件不显示坐标轴；G 显示/隐藏移动坐标轴，Shift+G 旋转坐标轴，默认保持关闭', gs1.sel && !gs1.gizmo && gs2.gizmo && gs2.mode === 'translate' && gs2.btn && !gs3 && gs4.gizmo && gs4.mode === 'rotate' && !gs5, JSON.stringify({ gs1, gs2, gs3, gs4, gs5 }));
  // right-click menu item
  const gxy = await surf(g0, [0, 0.025, 0]); await p.mouse.click(gxy[0], gxy[1], { button: 'right' }); await frame();
  const cm = await p.evaluate(() => [...document.querySelectorAll('#ctx .ci')].map(x => x.textContent));
  await p.evaluate(() => { const it = [...document.querySelectorAll('#ctx .ci')].find(x => x.dataset.a === 'gizmo'); it && it.click(); }); await frame();
  const gs6 = await p.evaluate(() => ({ gizmo: !!app.tc.object, mode: app.tc.mode }));
  await p.keyboard.press('KeyG');
  // left-drag still moves the part (no gizmo)
  const d0 = await p.evaluate(id => app.part(id).p.slice(), g0); const dxy = await surf(g0, [0, 0.025, 0]);
  await p.mouse.move(dxy[0], dxy[1]); await p.mouse.down(); await p.mouse.move(dxy[0] + 70, dxy[1] + 10, { steps: 8 }); await p.mouse.up(); await frame();
  const d1 = await p.evaluate(id => ({ p: app.part(id).p.slice(), gizmo: !!app.tc.object }), g0);
  check('右键菜单“显示移动坐标轴”；无坐标轴时左键拖动仍可移动零件', cm.some(t => t.includes('显示移动坐标轴')) && gs6.gizmo && gs6.mode === 'translate' && Math.hypot(d1.p[0] - d0[0], d1.p[2] - d0[2]) > 0.02 && !d1.gizmo, JSON.stringify({ menu: cm.filter(t => t.includes('坐标轴')), gs6, moved: +Math.hypot(d1.p[0] - d0[0], d1.p[2] - d0[2]).toFixed(4) }));

  // ======================= 2. 3D import =======================
  await setup([0.35, 0.3, 0.45], [0, 0.02, 0]);
  await p.evaluate(() => { localStorage.removeItem('mech3d-myparts'); app.renderMine(); });
  const imp = {};
  for (const [f, unitWant] of [['box40x20x10.stl', 'mm'], ['box40x20x10-ascii.stl', 'mm'], ['bracket.obj', 'mm'], ['wheel-r50.glb', 'm']]) {
    const n0 = await p.evaluate(() => app.M.parts.length);
    const inp = await p.$('#impIn'); await inp.uploadFile(FX + f);
    await p.waitForSelector('#impDlg', { timeout: 10000 });
    const dlg = await p.evaluate(() => ({ unit: document.getElementById('impU').value, size: document.getElementById('impSz').textContent }));
    await p.select('#impMat', 'alu'); await p.click('#impOk'); await p.waitForFunction(n => app.M.parts.length > n, {}, n0); await frame();
    imp[f] = await p.evaluate(() => { const P = app.M.parts[app.M.parts.length - 1]; return { id: P.id, type: P.type, mass: P.mass, half: app.DEFS.mesh.pts(P.d)[3].p[0], ol: !!document.querySelector(`#olList .olr[data-pid="${P.id}"]`) }; });
    imp[f].dlg = dlg; imp[f].unitOk = dlg.unit === unitWant;
  }
  const boxOk = r => r.type === 'mesh' && Math.abs(r.mass - 8e-6 * 2700) < 1e-4 && Math.abs(r.half - 0.02) < 1e-6 && r.unitOk && r.ol;
  const runImp = await p.evaluate(ids => { app.startRun(); const cols = ids.map(id => { let n = 0; for (const v of app.sim.colPart.values()) if (v.part && v.part.id === id) n++; return n; }); app.simulate(1.5); const ys = ids.map(id => app.sim.partPose(id).p.y); app.reset(); return { cols, ys }; }, Object.values(imp).map(r => r.id));
  check('导入 STL(二进制/ASCII)/OBJ/GLB：单位对话框(mm/米)、尺寸、质量=体积×密度、凸包碰撞体、出现在组件栏', boxOk(imp['box40x20x10.stl']) && boxOk(imp['box40x20x10-ascii.stl']) && Math.abs(imp['bracket.obj'].mass - 15e-6 * 2700) < 2e-4 && Math.abs(imp['wheel-r50.glb'].half - 0.05) < 1e-4 && imp['wheel-r50.glb'].unitOk && runImp.cols.every(c => c === 1) && runImp.ys.every(y => y > 0.003 && y < 0.08),
    JSON.stringify({ imp: Object.fromEntries(Object.entries(imp).map(([k, v]) => [k, { unit: v.dlg.unit, size: v.dlg.size, mass: +v.mass.toFixed(4), ol: v.ol }])), runImp }));
  // drag & drop a file onto the canvas
  const b64 = fs.readFileSync(FX + 'bracket.obj').toString('base64'), cvb = await (await p.$('#cv')).boundingBox();
  const nDrop0 = await p.evaluate(() => app.M.parts.length);
  await p.evaluate((b64, x, y) => { const s = atob(b64), u = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i); const dt = new DataTransfer(); dt.items.add(new File([u], 'dropped.obj')); document.getElementById('center').dispatchEvent(new DragEvent('drop', { dataTransfer: dt, clientX: x, clientY: y, bubbles: true, cancelable: true })); }, b64, cvb.x + cvb.width * 0.3, cvb.y + cvb.height * 0.6);
  await p.waitForSelector('#impDlg', { timeout: 10000 }); await p.click('#impOk'); await p.waitForFunction(n => app.M.parts.length > n, {}, nDrop0);
  const dropName = await p.evaluate(() => app.M.parts[app.M.parts.length - 1].name);
  // save -> wipe registry -> load ; undo / redo ; initial state
  const io = await p.evaluate(() => {
    const data = JSON.parse(JSON.stringify(app.saveData())), ng = Object.keys(data.geos).length, n = app.M.parts.length;
    for (const k of Object.keys(app.GEO)) delete app.GEO[k];
    app.loadJSON(JSON.stringify(data)); const meshes = app.M.parts.filter(P => P.type === 'mesh'); const vis = meshes.every(P => app.objs[P.id].children.length > 0 && app.objs[P.id].children[0].geometry.attributes.position.count > 8);
    app.startRun(); app.simulate(0.5); const ok = meshes.every(P => isFinite(app.sim.partPose(P.id).p.y)); app.reset(); const afterReset = app.M.parts.filter(P => P.type === 'mesh').length;
    const u0 = app.M.parts.length; app.deleteParts([meshes[0].id]); const ud = app.M.parts.length; app.undo(); const uu = app.M.parts.length; const back = app.M.parts.find(P => P.id === meshes[0].id);
    return { ng, n, loaded: app.M.parts.length, vis, ok, afterReset, del: [u0, ud, uu], backVol: back && app.GEO[back.d.g] ? app.GEO[back.d.g].vol : 0 };
  });
  check('拖放文件到场景导入；导入零件随 保存/读取(含几何数据)、撤销/重做、初始状态 保留', /dropped/.test(dropName) && io.ng >= 4 && io.loaded === io.n && io.vis && io.ok && io.afterReset === io.n && io.del[1] === io.del[0] - 1 && io.del[2] === io.del[0] && io.backVol > 0, JSON.stringify({ dropName, ...io }));
  // my parts library: listed + drag from palette creates a part
  const mineN = await p.evaluate(() => document.querySelectorAll('#palMine .item').length);
  const nm0 = await p.evaluate(() => app.M.parts.length);
  const mid = await dropSel('#palMine .item', [cvb.x + cvb.width * 0.7, cvb.y + cvb.height * 0.7]);
  const mine = await p.evaluate(id => { const P = app.part(id); return { type: P.type, g: P.d.g, stored: JSON.parse(localStorage.getItem('mech3d-myparts') || '[]').length }; }, mid);
  check('组件库“我的零件”：导入后自动加入(localStorage)，可从库中拖入复用', mineN >= 4 && mine.type === 'mesh' && mine.stored >= 4 && (await p.evaluate(() => app.M.parts.length)) === nm0 + 1, JSON.stringify({ mineN, ...mine }));

  // ======================= custom connectors on an imported part =======================
  await setup([0.0, 0.45, 0.25], [0, 0.0, 0]);
  const bracketB64 = fs.readFileSync(FX + 'box40x20x10.stl').toString('base64');
  const imId = await p.evaluate(async b64 => { const s = atob(b64), u = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i); const P = await app.importModel('plate.stl', u.buffer, { unit: 'mm', mat: 'alu', mine: false, at: new app._V(0, 0, 0) }); P.fixed = true; app.select(null); app.rebuild(); return P.id; }, bracketB64);
  await frame();
  await p.evaluate(() => app.setTool('addcon')); await frame();
  const bar = await p.evaluate(() => document.getElementById('tbar2').style.display);
  await click(await surf(imId, [0.012, 0.005, 0]));
  const con = await p.evaluate(id => { const P = app.part(id); return P.cons ? P.cons.map(c => ({ n: c.n, role: c.role, p: c.p, a: c.a })) : null; }, imId);
  await p.keyboard.press('Escape'); await frame();
  const conW = await p.evaluate(id => { const P = app.part(id), c = P.cons[0], Q = app.cam.quaternion.constructor; return new app._V(...c.p).applyQuaternion(new Q(...P.q)).add(new app._V(...P.p)).toArray(); }, imId);
  const wId = await dropSel('.item[data-type="wheel"]', await scr(conW));
  const cj = await p.evaluate((a, b) => { const j = app.M.joints.find(j => [j.a, j.b].includes(a) && [j.a, j.b].includes(b)); if (!j) return null; const W = app.part(b), Q = app.cam.quaternion.constructor; return { type: j.type, wheelAxisY: Math.abs(new app._V(0, 0, 1).applyQuaternion(new Q(...W.q)).y) }; }, imId, wId);
  check('“＋连接点”：在导入零件表面点击添加孔（轴线=表面法向），拖入轮子自动铰接到该孔', bar === 'flex' && con && con.length === 1 && con[0].role === 'hole' && con[0].a[1] > 0.99 && cj && cj.type === 'revolute' && cj.wheelAxisY > 0.99, JSON.stringify({ con, cj }));
  await p.screenshot({ path: OUT + 'screenshot-import.png' }).catch(() => { });

  // ======================= 3. snap anywhere along a rod / any plate face =======================
  await setup([0, 0.3, 1.0], [0, 0.3, 0]);
  const rodId = await p.evaluate(() => { const R = app.placePart('rod', { x: 0, y: 0, z: 0 }); R.p = [0, 0.3, 0]; R.q = [0, 0, 0, 1]; R.fixed = true; app.rebuild(); app.select(null); return R.id; });
  await frame();
  const wr = await dropSel('.item[data-type="gear"]', await scr([-0.047, 0.3, 0.005]));
  const rs = await p.evaluate((r, w) => { const j = app.M.joints.find(j => [j.a, j.b].includes(r) && [j.a, j.b].includes(w)); if (!j) return null; const l = j.a === r ? j.la : j.lb; return { type: j.type, x: l[0], onGrid: Math.abs(l[0] / 0.005 - Math.round(l[0] / 0.005)) < 1e-6 }; }, rodId, wr);
  check('沿连杆任意位置吸附（5mm 网格，非端孔/中点）：齿轮铰接在杆身 x≈-45mm', rs && rs.type === 'revolute' && Math.abs(rs.x + 0.045) < 0.0051 && Math.abs(rs.x) > 0.02 && Math.abs(Math.abs(rs.x) - 0.15) > 0.02 && rs.onGrid, JSON.stringify(rs));
  // plate: wheel anywhere on the side face (+Z), block flush on the top face
  await setup([0.25, 0.45, 0.9], [0, 0.3, 0]);
  const plId = await p.evaluate(() => { const P = app.placePart('plate', { x: 0, y: 0, z: 0 }, { d: { w: 0.3, h: 0.06, dp: 0.2, np: 3 } }); P.p = [0, 0.3, 0]; P.fixed = true; app.rebuild(); app.select(null); return P.id; });
  await frame();
  const pw = await dropSel('.item[data-type="wheel"]', await scr([0.075, 0.31, 0.1]));
  const ps = await p.evaluate((pl, w) => { const j = app.M.joints.find(j => [j.a, j.b].includes(pl) && [j.a, j.b].includes(w)), W = app.part(w); return j ? { type: j.type, wp: W.p.map(v => +v.toFixed(4)), ax: new app._V(...j.axa).toArray() } : null; }, plId, pw);
  check('板侧面任意位置装轮（铰链，轴线=面法向，轮贴在面外侧）', ps && ps.type === 'revolute' && Math.abs(Math.abs(ps.ax[2]) - 1) < 1e-6 && Math.abs(ps.wp[0] - 0.075) < 0.006 && ps.wp[2] > 0.11, JSON.stringify(ps));
  // face mating by dragging an imported part onto the top face (arbitrary position, flush)
  const tId = await dropSel('#palMine .item', await scr([-0.08, 0.33, -0.05]));
  const fm = await p.evaluate((pl, t) => { const j = app.M.joints.find(j => [j.a, j.b].includes(pl) && [j.a, j.b].includes(t)), T = app.part(t); const lo = app._partMinY ? 0 : 0; void lo; return j ? { type: j.type, y: T.p[1], x: T.p[0], z: T.p[2], qw: Math.abs(T.q[3]) } : null; }, plId, tId);
  const fmH = await p.evaluate(t => { const P = app.part(t); const Q = app.cam.quaternion.constructor, q = new Q(...P.q); const b = app.DEFS.mesh.pts(P.d); let lo = 1e9; for (const v of b) { const w = new app._V(...v.p).applyQuaternion(q); lo = Math.min(lo, w.y); } return lo; }, tId);
  check('面贴合：把导入零件拖到板顶面任意位置 → 固定连接，底面与顶面齐平（最小转动：保持原姿态平放）', fm && fm.type === 'fixed' && Math.abs(fm.y + fmH - 0.33) < 0.001 && fm.qw > 0.9999, JSON.stringify({ fm, bottomOffset: fmH }));

  // ======================= two-click 配合 tool =======================
  await setup([0.35, 0.5, 0.75], [0, 0.15, 0]);
  const mt = await p.evaluate(() => { const P = app.placePart('plate', { x: 0, y: 0, z: 0 }, { d: { w: 0.3, h: 0.02, dp: 0.2, np: 3 }, name: '底板' }); P.p = [0, 0.2, 0]; P.fixed = true; const B = app.placePart('block', { x: 0.25, y: 0, z: 0.2 }, { name: '块' }); app.rebuild(); app.select(null); return { pl: P.id, bl: B.id }; });
  await frame();
  await p.keyboard.press('KeyM'); await frame();
  await click(await surf(mt.bl, [0.0, 0.025, 0.0]));
  await click(await surf(mt.pl, [-0.06, 0.01, 0.04]));
  const mbar = await p.evaluate(() => ({ disp: document.getElementById('tbar2').style.display, has: !!document.getElementById('mtT') }));
  await p.screenshot({ path: OUT + 'screenshot-mate.png' });
  await p.select('#mtT', 'revolute'); await frame();
  await p.evaluate(() => { const i = document.getElementById('mtO'); i.value = 5; i.dispatchEvent(new Event('change')); }); await frame();
  await p.click('#mtOk'); await frame();
  const mr = await p.evaluate(m => { const j = app.M.joints.find(j => [j.a, j.b].includes(m.pl) && [j.a, j.b].includes(m.bl)), B = app.part(m.bl); return { j: j && j.type, y: B.p[1], x: B.p[0], z: B.p[2], undo: app.undoS.length, bar: document.getElementById('tbar2').style.display }; }, mt);
  check('🧲 两点配合工具：点 A 的面 → 点 B 的面 → 贴合（法向相对），工具条改为铰链 + 偏移 5mm → 确定', mbar.disp === 'flex' && mbar.has && mr.j === 'revolute' && Math.abs(mr.y - (0.21 + 0.005 + 0.025)) < 0.0015 && Math.abs(mr.x + 0.06) < 0.003 && Math.abs(mr.z - 0.04) < 0.003, JSON.stringify(mr));
  await p.keyboard.press('Escape');

  // ======================= joint editor: limits + spring + key-bound drive =======================
  await setup([0.3, 0.4, 0.8], [0, 0.2, 0]);
  const je = await p.evaluate(() => { const P = app.placePart('plate', { x: 0, y: 0, z: 0 }, { d: { w: 0.4, h: 0.02, dp: 0.1 } }); P.p = [0, 0.2, 0]; P.fixed = true; const B = app.placePart('block', { x: 0, y: 0, z: 0.3 }); B.p = [0, 0.235, 0]; const j = app.addJoint('prismatic', P, B, new app._V(0, 0.21, 0), new app._V(1, 0, 0)); app.commit(); app.rebuild(); app.select({ j: j.id }); return { j: j.id, b: B.id }; });
  await frame(); await p.evaluate(() => { const d = document.querySelector('#props .jdet'); if (d) d.open = true; });
  const setv = async (cls, v) => { await p.evaluate((cls, v) => { const i = document.querySelector('#props .' + cls); if (i.type === 'checkbox') i.checked = v; else i.value = v; i.dispatchEvent(new Event('change')); }, cls, v); await frame(); await p.evaluate(() => { const d = document.querySelector('#props .jdet'); if (d) d.open = true; }); };
  await setv('jlimon', true); await setv('jlo', -10); await setv('jhi', 10); await setv('jkc', 400); await setv('jdp', 8); await setv('jrest', 30);
  const jf = await p.evaluate(id => { const j = app.M.joints.find(x => x.id === id); return { lim: j.lim, kc: j.kc, damp: j.damp, rest: j.rest }; }, je.j);
  const jrun = await p.evaluate(m => { app.startRun(); const x0 = app.sim.partPose(m.b).p.x; app.simulate(2); const x = app.sim.partPose(m.b).p.x - x0; app.reset(); return x * 1000; }, je);
  check('连接编辑：滑动副限位 ±10mm + 弹簧刚度/阻尼/平衡位置(30mm) 可编辑 → 运行时被弹簧推到限位', jf.lim.on && Math.abs(jf.lim.lo + 0.01) < 1e-9 && Math.abs(jf.lim.hi - 0.01) < 1e-9 && jf.kc === 400 && jf.damp === 8 && Math.abs(jf.rest - 0.03) < 1e-9 && jrun > 8 && jrun < 11, JSON.stringify({ jf, travel_mm: +jrun.toFixed(3) }));
  // revolute: built-in motor with key binding (J / K)
  const jr = await p.evaluate(() => { app.reset(); document.getElementById('btnClear').click(); const B = app.placePart('base', { x: 0, y: 0, z: 0 }); const W = app.placePart('wheel', { x: 0, y: 0, z: 0.3 }); W.p = [0, B.d.h - 0.03, B.d.dp / 2 + 0.0155]; const j = app.addJoint('revolute', B, W, new app._V(0, B.d.h - 0.03, B.d.dp / 2), new app._V(0, 0, 1)); app.commit(); app.rebuild(); app.select({ j: j.id }); return { j: j.id, w: W.id }; });
  await frame(); await p.evaluate(() => { document.querySelector('#props .jdet').open = true; });
  await p.click('#props .jdon'); await frame(); await p.evaluate(() => { document.querySelector('#props .jdet').open = true; });
  await p.focus('#props .jkp'); await p.keyboard.press('KeyJ'); await frame(); await p.evaluate(() => { document.querySelector('#props .jdet').open = true; });
  await p.focus('#props .jkn'); await p.keyboard.press('KeyK'); await frame();
  const jd = await p.evaluate(id => { const j = app.M.joints.find(x => x.id === id); return Object.assign({}, j.drive); }, jr.j);
  await p.evaluate(() => document.activeElement.blur());
  await p.evaluate(() => app.startRun()); const wz = () => p.evaluate(w => app.sim.pb[w].body.angvel().z, jr.w);
  await p.evaluate(() => app.simulate(0.5)); const w0 = await wz();
  await p.keyboard.down('KeyJ'); await p.evaluate(() => app.simulate(0.5)); const w1 = await wz(); await p.keyboard.up('KeyJ');
  await p.keyboard.down('KeyK'); await p.evaluate(() => app.simulate(1.0)); const w2 = await wz(); await p.keyboard.up('KeyK');
  await p.evaluate(() => app.reset());
  check('连接编辑：铰链内置电机 + 绑定按键（J 正转 / K 反转，未按键不驱动）', jd.on && jd.keyP === 'KeyJ' && jd.keyN === 'KeyK' && Math.abs(w0) < 0.05 && w1 > 2.5 && w2 < -2.5, JSON.stringify({ drive: jd, w0: +w0.toFixed(3), wJ: +w1.toFixed(3), wK: +w2.toFixed(3) }));

  // ======================= 4-wheel car from generic parts (validation) =======================
  const car = await p.evaluate(() => {
    const A = app, i = [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent.startsWith('四轮小车')) - 1; A.loadPreset(i);
    const types = [...new Set(A.M.parts.map(P => P.type))], lw = A.byName('左前轮'), rw = A.byName('左前轮 镜像');
    const ch = () => A.sim.partPose(A.byName('车架').id), V = A.cam.position.constructor;
    const yaw = () => { const x = new V(1, 0, 0).applyQuaternion(ch().q); return Math.atan2(-x.z, x.x) * 180 / Math.PI; }, up = () => new V(0, 1, 0).applyQuaternion(ch().q).y;
    let nan = false, minUp = 1, gap = 0; const st = () => { const c = ch().p; return { x: c.x, z: c.z, yaw: yaw() }; };
    const run = (sec, keys) => { keys.forEach(k => A.setKey(k, true)); for (let t = 0; t < sec * 10; t++) { A.simulate(0.1); const c = ch().p; for (const P of A.M.parts) { const q = A.sim.partPose(P.id).p; if (!isFinite(q.x + q.y + q.z)) nan = true; } minUp = Math.min(minUp, up()); gap = Math.max(gap, A.sim.maxGap()); void c; } keys.forEach(k => A.setKey(k, false)); return st(); };
    A.startRun(); const y0 = ch().p.y; const s0 = run(1, []); const sag = (y0 - ch().p.y) * 1000;
    const sW = run(2, ['KeyW']); const sA = run(1.5, ['KeyW', 'KeyA']); const sD = run(3, ['KeyW', 'KeyD']);
    const r = { types, n: A.M.parts.length, mirrorZ: [lw.p[2], rw.p[2]], sag, fwd: sW.x - s0.x, yawA: sA.yaw - sW.yaw, yawD: sD.yaw - sA.yaw, minUp, nan, gap_mm: gap * 1000, t: A.sim.time };
    A.reset(); return r;
  });
  check('验证：仅用通用零件(板/滑块/轮)+配合+镜像 装出四轮小车，运行 7.5 s 无 NaN、W 前进、A/D 转向、悬架下沉、不翻车', !car.types.some(t => !['plate', 'block', 'wheel', 'wall'].includes(t)) && car.n === 12 && Math.abs(car.mirrorZ[0] + car.mirrorZ[1]) < 1e-6 && !car.nan && car.fwd > 0.8 && car.yawA > 15 && car.yawD < -15 && car.minUp > 0.95 && car.sag > 3 && car.gap_mm < 1, JSON.stringify(car));
  // multi-select (Ctrl+click in the viewport) -> group panel
  await p.evaluate(() => { app.view('iso'); app.fit(); }); await frame();
  const wl = await p.evaluate(() => app.byName('左后轮').id), wr2 = await p.evaluate(() => app.byName('车架').id);
  await click(await surf(wr2, [0, 0.01, 0])); 
  const grpSel = await p.evaluate(() => app.selIds().length);
  await click(await surf(wl, [0, 0, 0.016]), ['Control']);
  const ms = await p.evaluate(() => ({ n: app.selIds().length, panel: !!document.getElementById('mGrp') || !!document.getElementById('mUngrp'), mir: !!document.getElementById('mMir') }));
  check('多选：Ctrl+点击 视图中零件 → 多选面板（组合 / 复制 / 镜像复制）；点击已组合零件选中整组', grpSel === 1 && ms.n >= 3 && ms.panel && ms.mir, JSON.stringify({ grpSel, ...ms }));
  // car screenshot while driving
  await p.evaluate(() => { app.select(null); app.startRun(); app.setKey('KeyW', true); app.simulate(1.2); app.setKey('KeyA', true); app.simulate(0.6); app.setKey('KeyW', false); app.setKey('KeyA', false); app.updRun(); const c = app.sim.partPose(app.byName('车架').id).p; app.orbit.target.copy(c); app.cam.position.set(c.x + 0.55, c.y + 0.45, c.z + 0.75); app.orbit.update(); });
  await new Promise(r => setTimeout(r, 300)); await frame();
  await p.screenshot({ path: OUT + 'screenshot-car.png' });
  await p.evaluate(() => app.reset());
};
