// View controls: only middle-drag orbits; left = select/drag parts. Used by run-tests.js
module.exports = async function (p, check, OUT) {
  const frame = () => p.evaluate(() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r))));
  const settle = async () => { for (let i = 0; i < 20; i++) await frame(); };
  const cam = () => p.evaluate(() => ({ q: app.cam.quaternion.toArray(), pos: app.cam.position.toArray(), parts: app.M.parts.map(P => P.p.slice()) }));
  const qd = (a, b) => 1 - Math.abs(a.q.reduce((s, v, i) => s + v * b.q[i], 0));
  const pmax = (a, b) => Math.max(...a.parts.map((x, i) => Math.hypot(x[0] - b.parts[i][0], x[1] - b.parts[i][1], x[2] - b.parts[i][2])));
  const drag = async (x, y, dx, dy, button) => { await p.mouse.move(x, y); await p.mouse.down({ button }); for (let i = 1; i <= 10; i++) await p.mouse.move(x + dx * i / 10, y + dy * i / 10); await p.mouse.up({ button }); await settle(); };
  const idx = await p.evaluate(() => [...document.querySelectorAll('#selPreset option')].findIndex(o => o.textContent === '三轮车') - 1);
  await p.evaluate(i => { if (app.mode !== 'edit') app.reset(); app.loadPreset(i); app.view('iso'); app.fit(); app.select(null); }, idx); await settle();
  const cfg = await p.evaluate(() => ({ L: app.orbit.mouseButtons.LEFT, M: app.orbit.mouseButtons.MIDDLE, R: app.orbit.mouseButtons.RIGHT, one: app.orbit.touches.ONE }));
  check('OrbitControls 映射：左键无、中键旋转、右键平移、单指不旋转', cfg.L == null && cfg.M === 0 && cfg.R === 2 && cfg.one == null, JSON.stringify(cfg));
  // empty spot: scan canvas for a point with no part
  const findEmpty = () => p.evaluate(() => { const c = app.renderer.domElement, r = c.getBoundingClientRect(); for (let y = r.top + 120; y < r.bottom; y += 20) for (let x = r.left + 30; x < r.right - 200; x += 20) if (document.elementFromPoint(x, y) === c && !app.pickAt({ clientX: x, clientY: y }).pid) return { x, y }; return null; });
  let empty = await findEmpty();
  let a = await cam(); await drag(empty.x, empty.y, 150, 60, 'left'); let b = await cam();
  check('左键拖动空白处不旋转视角', qd(a, b) < 1e-9 && pmax(a, b) < 1e-9, 'dq=' + qd(a, b));
  const pt = await p.evaluate(() => { const P = app.M.parts.find(P => !app.part(P.id).locked && P.type === 'wheel') || app.M.parts[0]; const s = app.toScreen(new app._V(...P.p)); for (let dx = -10; dx <= 10; dx += 2) for (let dy = -10; dy <= 10; dy += 2) if (app.pickAt({ clientX: s.x + dx, clientY: s.y + dy }).pid) return { x: s.x + dx, y: s.y + dy }; return { x: s.x, y: s.y }; });
  a = await cam(); await drag(pt.x, pt.y, 60, -40, 'left'); b = await cam();
  check('左键拖动零件：零件移动，视角不变', pmax(a, b) > 1e-3 && qd(a, b) < 1e-9, `moved=${pmax(a, b).toFixed(4)} dq=${qd(a, b)}`);
  for (const mode of ['edit', 'run']) {
    if (mode === 'run') await p.evaluate(() => { app.toggleRun ? app.toggleRun() : app.run(); });
    await settle(); if (mode === 'run') empty = await findEmpty(); a = await cam(); const target = mode === 'edit' ? pt : empty;
    await drag(target.x, target.y, 150, 50, 'middle'); b = await cam();
    check(`中键拖动旋转视角（${mode === 'edit' ? '编辑' : '运行'}模式${mode === 'edit' ? '，在零件上' : ''}）且不移动零件`, qd(a, b) > 1e-4 && (mode === 'run' || pmax(a, b) < 1e-9), `dq=${qd(a, b).toExponential(2)} moved=${pmax(a, b).toFixed(5)}`);
  }
  const prevented = await p.evaluate(() => { const ev = new MouseEvent('mousedown', { button: 1, bubbles: true, cancelable: true }); app.renderer.domElement.dispatchEvent(ev); return ev.defaultPrevented; });
  check('中键按下阻止浏览器自动滚动', prevented, '');
  await p.evaluate(() => { if (app.mode !== 'edit') app.reset(); }); await settle();
  await p.screenshot({ path: OUT + 'screenshot-controls.png' });
};
