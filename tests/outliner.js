// Outliner (组件栏) + palette search tests, real mouse events where feasible. Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const presetIdx = n => p.evaluate(n => [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === n) - 1, n);
  // every preset: outliner lists all parts
  const counts = await p.evaluate(() => { if (app.mode !== 'edit') app.reset(); const opts = [...document.querySelectorAll('#selPreset option')].slice(1);
    return opts.map((o, i) => { app.loadPreset(i); const rows = [...document.querySelectorAll('#olList .olr[data-pid]')].map(r => r.dataset.pid); return [o.textContent, rows.length, app.M.parts.length, new Set(rows).size === app.M.parts.length && app.M.parts.every(P => rows.includes(P.id)), +document.getElementById('olCount').textContent]; }); });
  for (const [n, rows, parts, all, badge] of counts) check('组件栏列出全部零件: ' + n, rows === parts && all && badge === parts, `rows=${rows} parts=${parts}`);
  // tricycle: click selects, double-click focuses, hover highlights
  await p.evaluate(i => { app.loadPreset(i); app.view('iso'); app.fit(); app.select(null); }, await presetIdx('三轮车')); await frame();
  const pid = await p.evaluate(() => app.byName('车架') ? app.byName('车架').id : app.M.parts[1].id);
  const row = async id => (await p.$(`#olList .olr[data-pid="${id}"] .oli`));
  const r1 = await row(pid); await r1.hover(); await frame();
  const hov = await p.evaluate(id => app.hoverPid === id, pid);
  await r1.click(); await frame();
  const sel = await p.evaluate(id => ({ sel: app.sel && app.sel.p === id, rowSel: document.querySelector(`#olList .olr[data-pid="${id}"]`).classList.contains('sel'), props: document.querySelector('#props .title').textContent, gizmo: !!app.tc.object }), pid);
  check('组件栏：悬停高亮 3D 零件，单击选中（属性面板 + 高亮行，无坐标轴）', hov && sel.sel && sel.rowSel && /车架|：/.test(sel.props) && !sel.gizmo, JSON.stringify({ hov, ...sel, note: '选中不显示坐标轴' }));
  const wheel = await p.evaluate(() => app.M.parts.find(P => P.type === 'wheel').id);
  const t0 = await p.evaluate(() => ({ t: app.orbit.target.toArray(), d: app.cam.position.distanceTo(app.orbit.target) }));
  await (await row(wheel)).click({ count: 2 }); await frame();
  const t1 = await p.evaluate(id => { const o = app.objs[id]; const p = new app._V(); o.getWorldPosition(p); return { t: app.orbit.target.toArray(), d: app.cam.position.distanceTo(app.orbit.target), toPart: app.orbit.target.distanceTo(p), sel: app.sel && app.sel.p === id }; }, wheel);
  check('组件栏：双击聚焦视角到零件', t1.toPart < 0.05 && t1.d < t0.d && t1.sel, JSON.stringify({ before: t0.d.toFixed(3), after: t1.d.toFixed(3), targetToPart: t1.toPart.toFixed(4) }));
  // 3D selection highlights + scrolls row into view
  const last = await p.evaluate(() => app.M.parts[app.M.parts.length - 1].id);
  await p.evaluate(() => { document.getElementById('olList').scrollTop = 0; });
  await p.evaluate(id => app.select({ p: id }), last); await frame();
  const vis = await p.evaluate(id => { const r = document.querySelector(`#olList .olr[data-pid="${id}"]`), L = document.getElementById('olList').getBoundingClientRect(), b = r.getBoundingClientRect(); return { sel: r.classList.contains('sel'), inView: b.top >= L.top - 1 && b.bottom <= L.bottom + 1 }; }, last);
  check('3D 中选中 → 组件栏对应行高亮并滚动到可见', vis.sel && vis.inView, JSON.stringify(vis));
  // joints listed under part, click selects joint
  await (await p.$(`#olList .olr[data-pid="${wheel}"] .olc`)).click(); await frame();
  const jrow = await p.$('#olList .olsub[data-jid]');
  let jsel = null; if (jrow) { await jrow.click(); await frame(); jsel = await p.evaluate(() => ({ j: app.sel && app.sel.j, title: document.querySelector('#props .title').textContent })); }
  check('组件栏：零件下列出连接，点击连接 → 选中并编辑', !!jsel && !!jsel.j && /连接/.test(jsel.title), JSON.stringify(jsel));
  // hide
  await p.evaluate(() => app.select(null));
  await (await p.$(`#olList .olr[data-pid="${wheel}"] .ole`)).click(); await frame();
  const hid = await p.evaluate(id => { const P = app.part(id), s = app.toScreen(new app._V(...P.p)); return { hidden: !!P.hidden, vis: app.objs[id].visible, pick: app.pickAt({ clientX: s.x, clientY: s.y }).pid !== id }; }, wheel);
  await (await p.$(`#olList .olr[data-pid="${wheel}"] .ole`)).click(); await frame();
  const shown = await p.evaluate(id => app.objs[id].visible && !app.part(id).hidden, wheel);
  check('组件栏：眼睛图标隐藏/显示（隐藏后不可点选）', hid.hidden && !hid.vis && hid.pick && shown, JSON.stringify({ ...hid, shownAgain: shown }));
  // lock: cannot drag / rotate
  await (await p.$(`#olList .olr[data-pid="${wheel}"] .oll`)).click(); await frame();
  const lk0 = await p.evaluate(id => ({ p: app.part(id).p.slice(), q: app.part(id).q.slice() }), wheel);
  const wxy = await p.evaluate(id => { const s = app.toScreen(new app._V(...app.part(id).p)); return [s.x, s.y]; }, wheel);
  await p.mouse.move(wxy[0], wxy[1]); await p.mouse.down(); await p.mouse.move(wxy[0] + 80, wxy[1] + 40, { steps: 8 }); await p.mouse.up(); await frame();
  await p.evaluate(id => { app.gizmoOn = true; app.select({ p: id }); }, wheel); await p.keyboard.press('KeyR'); await frame();
  const lk1 = await p.evaluate(id => ({ p: app.part(id).p.slice(), q: app.part(id).q.slice(), locked: !!app.part(id).locked, gizmo: !!app.tc.object }), wheel);
  const lkd = lk0.p.concat(lk0.q).reduce((a, v, i) => a + Math.abs(v - lk1.p.concat(lk1.q)[i]), 0);
  await p.evaluate(() => { app.gizmoOn = false; app.updSel(); });
  await (await p.$(`#olList .olr[data-pid="${wheel}"] .oll`)).click(); await frame();
  check('组件栏：锁定后不能拖动/旋转（无坐标轴）', lk1.locked && lkd < 1e-12 && !lk1.gizmo, JSON.stringify({ change: lkd, gizmo: lk1.gizmo }));
  // multi-select (Ctrl) + delete via button + undo sync + add sync
  const ids = await p.evaluate(() => app.M.parts.slice(0, 3).map(P => P.id));
  await (await row(ids[0])).click(); await p.keyboard.down('Control'); await (await row(ids[1])).click(); await p.keyboard.up('Control'); await frame();
  const ms = await p.evaluate(() => ({ n: app.msel.size, rows: document.querySelectorAll('#olList .olr.sel').length }));
  const n0 = await p.evaluate(() => app.M.parts.length);
  await (await p.$(`#olList .olr[data-pid="${ids[2]}"] .old`)).click(); await frame();
  const afterDel = await p.evaluate(() => ({ parts: app.M.parts.length, rows: document.querySelectorAll('#olList .olr[data-pid]').length }));
  await p.evaluate(() => app.undo()); await frame();
  const afterUndo = await p.evaluate(() => ({ parts: app.M.parts.length, rows: document.querySelectorAll('#olList .olr[data-pid]').length }));
  await p.evaluate(() => app.redo()); await p.evaluate(() => app.undo());
  await p.evaluate(() => app.placePart('gear', { x: 0.5, y: 0, z: 0.5 })); await frame();
  const afterAdd = await p.evaluate(() => { const P = app.M.parts[app.M.parts.length - 1]; const r = document.querySelector(`#olList .olr[data-pid="${P.id}"]`); return { parts: app.M.parts.length, rows: document.querySelectorAll('#olList .olr[data-pid]').length, name: P.name, sel: r && r.classList.contains('sel') }; });
  check('组件栏 Ctrl 多选', ms.n === 2 && ms.rows === 2, JSON.stringify(ms));
  check('组件栏与 删除/撤销/添加 同步（自动命名）', afterDel.parts === n0 - 1 && afterDel.rows === n0 - 1 && afterUndo.rows === n0 && afterAdd.rows === n0 + 1 && afterAdd.sel && /^直齿轮 \d+$/.test(afterAdd.name), JSON.stringify({ n0, afterDel, afterUndo, afterAdd }));
  // rename via double-click on the name
  const gid = await p.evaluate(() => app.M.parts[app.M.parts.length - 1].id);
  await (await p.$(`#olList .olr[data-pid="${gid}"] .oln`)).click({ count: 2 }); await frame();
  await p.keyboard.down('Control'); await p.keyboard.press('KeyA'); await p.keyboard.up('Control'); await p.keyboard.type('主齿轮'); await p.keyboard.press('Enter'); await frame();
  const rn = await p.evaluate(id => ({ name: app.part(id).name, row: document.querySelector(`#olList .olr[data-pid="${id}"] .oln`).textContent }), gid);
  check('组件栏：双击名称改名', rn.name === '主齿轮' && rn.row === '主齿轮', JSON.stringify(rn));
  // filter
  await p.click('#olSearch'); await p.keyboard.type('轮'); await frame();
  const flt = await p.evaluate(() => [...document.querySelectorAll('#olList .olr[data-pid]')].map(r => app.part(r.dataset.pid)).every(P => /轮/.test(P.name + app.DEFS[P.type].label)) && document.querySelectorAll('#olList .olr[data-pid]').length > 0);
  await p.evaluate(() => { const s = document.getElementById('olSearch'); s.value = ''; s.dispatchEvent(new Event('input')); });
  check('组件栏筛选框', flt, String(flt));
  // palette search + recent
  await p.click('#palSearch'); await p.keyboard.type('轴承'); await frame();
  const ps = await p.evaluate(() => [...document.querySelectorAll('#palItems .item, #palTrans .item')].filter(x => x.style.display !== 'none').map(x => x.dataset.type));
  await p.evaluate(() => { const s = document.getElementById('palSearch'); s.value = ''; s.dispatchEvent(new Event('input')); });
  const allT = await p.evaluate(() => { const shown = [...document.querySelectorAll('#palItems .item, #palTrans .item')].filter(x => x.offsetParent !== null).map(x => x.dataset.type); return { all: Object.keys(app.DEFS).filter(t => app.DEFS[t].cat !== 'import').every(t => shown.includes(t)), recent: [...document.querySelectorAll('#palRecent .item')].map(x => x.dataset.type) }; });
  check('零件库搜索（“轴承”→轴承座）+ 全部零件类型可见 + 最近使用', ps.length === 1 && ps[0] === 'bearing' && allT.all && allT.recent[0] === 'gear', JSON.stringify({ search: ps, ...allT }));
  // screenshot with the tricycle preset
  await p.evaluate(i => { app.loadPreset(i); app.view('iso'); app.fit(); const w = app.M.parts.find(P => P.type === 'motor'); app.olOpen.add(w.id); app.renderOutliner(); app.select({ p: w.id }); }, await presetIdx('三轮车'));
  await new Promise(r => setTimeout(r, 400)); await frame();
  await p.screenshot({ path: OUT + 'screenshot-outliner.png' });
};
