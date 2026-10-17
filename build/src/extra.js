// Extra editor features mixed into App: on-demand gizmo, 3D import + 我的零件, custom connectors,
// two-click 配合 (mate) tool, multi-select / groups / copy / mirror, keyboard-driven joints in run mode.
import * as THREE from 'three';
import { DEFS, MATS, GEO, defaultMass } from './parts.js';
import { V3, Q4, ptW, axW, ptL, axL, isForce, analyze } from './physics.js';
import { connectors } from './assemble.js';
import { parseModel, buildGeo, soupBox, UNITS, UNIT_N, defaultUnit, isModelFile, mirrorGeo } from './importer.js';

const $ = id => document.getElementById(id);
const D2R = Math.PI / 180;
// local reflection plane (axis index) under which each built-in part is symmetric (used by mirror copy)
const SYM = { rod: 2, plate: 1, block: 2, base: 2, wall: 2, shaft: 0, wheel: 2, ball: 2, motor: 0, bearing: 2, caster: 2, gear: 2, rack: 2, bevel: 1, worm: 0, wormwheel: 2, pulley: 2, sprocket: 2, cam: 2, follower: 0, screw: 0, nut: 2, flywheel: 2, gearbox: 2, clutch: 0, ujoint: 0, mesh: 0 };
const ROLE_N = { hole: '孔', pin: '轴', mount: '安装面' };
export const keyLabel = c => !c ? '' : c.startsWith('Key') ? c.slice(3) : c.startsWith('Digit') ? c.slice(5) : ({ ArrowUp: '↑', ArrowDown: '↓', ArrowLeft: '←', ArrowRight: '→', Space: '空格' })[c] || c;
const reflectM = k => new THREE.Matrix4().makeScale(k === 0 ? -1 : 1, k === 1 ? -1 : 1, k === 2 ? -1 : 1);
const refl = (v, k) => { const a = Array.isArray(v) ? v.slice() : v.toArray(); a[k] = -a[k]; return a; };

export function installExtra(App) {
  const P = App.prototype;

  P.initExtra = function () {
    this.GEO = GEO; this.gizmoOn = false; this.keys = new Set(); this.mateType = 'fixed'; this.conRole = 'hole';
    // top-bar buttons
    const after = $('btnRot'), mk = (id, txt, title, cls = 'tb') => { const b = document.createElement('button'); b.className = cls; b.id = id; b.textContent = txt; b.title = title; return b; };
    const bG = mk('btnGizmo', '⊕ 坐标轴', '显示/隐藏移动坐标轴 (G)；选中零件时默认不显示，直接左键拖动零件即可移动');
    after.after(bG);
    const sep = document.createElement('span'); sep.className = 'sep'; bG.after(sep);
    const bM = mk('btnMate', '🧲 配合', '两点配合 (M)：先点零件A上的面/连接点，再点零件B上的面/连接点 → A 贴合到 B，再选连接类型/偏移/角度');
    const bC = mk('btnAddCon', '＋连接点', '在零件表面点击添加自定义连接点（孔/轴/安装面），用于导入零件的装配');
    const bI = mk('btnImport', '📥 导入零件', '导入 3D 模型：STL / OBJ / GLB / GLTF（也可直接把文件拖到场景中）');
    sep.after(bM); bM.after(bC); bC.after(bI);
    const fi = document.createElement('input'); fi.type = 'file'; fi.id = 'impIn'; fi.accept = '.stl,.obj,.glb,.gltf'; fi.style.display = 'none'; bI.after(fi);
    bG.onclick = () => this.toggleGizmo(); bM.onclick = () => this.setTool(this.tool === 'mate' ? 'select' : 'mate'); bC.onclick = () => this.setTool(this.tool === 'addcon' ? 'select' : 'addcon');
    bI.onclick = () => fi.click(); fi.onchange = e => { const f = e.target.files[0]; if (f) this.importFileObj(f); e.target.value = ''; };
    $('btnMove').onclick = () => this.toggleGizmo('translate'); $('btnRot').onclick = () => this.toggleGizmo('rotate');
    $('btnMove').classList.remove('on');
    // drag & drop files onto the canvas
    const c = $('center');
    c.addEventListener('dragover', e => { if (e.dataTransfer && [...e.dataTransfer.types].includes('Files')) { e.preventDefault(); e.dataTransfer.dropEffect = 'copy'; } });
    c.addEventListener('drop', e => { const f = e.dataTransfer && e.dataTransfer.files[0]; if (!f) return; e.preventDefault(); this.importFileObj(f, this.groundPoint(e)); });
    // floating tool option bar
    const tb = document.createElement('div'); tb.id = 'tbar2'; c.appendChild(tb);
    // my parts library
    const h = document.createElement('h3'); h.id = 'mineHdr'; h.textContent = '组件库：我的零件';
    const box = document.createElement('div'); box.className = 'pal'; box.id = 'palMine';
    $('palNone').before(h); h.after(box);
    this.renderMine();
    window.addEventListener('keyup', e => { if (this.keys.delete(e.code) && this.sim) { /* drive released */ } });
    window.addEventListener('blur', () => this.keys.clear());
  };

  // ---------------- gizmo on demand ----------------
  P.toggleGizmo = function (mode) {
    if (!mode) this.gizmoOn = !this.gizmoOn;
    else if (this.gizmoOn && this.tc.mode === mode) this.gizmoOn = false;
    else { this.tc.setMode(mode); this.gizmoOn = true; }
    this.updGizmoBtns(); this.updSel();
    this.msg(this.gizmoOn ? '已显示' + (this.tc.mode === 'rotate' ? '旋转' : '移动') + '坐标轴（G 隐藏；W 移动 / E 旋转）' : '已隐藏坐标轴：直接左键拖动零件移动，R 旋转 15°（G 显示坐标轴）');
  };
  P.updGizmoBtns = function () { const on = this.gizmoOn, m = this.tc.mode; $('btnGizmo').classList.toggle('on', on); $('btnMove').classList.toggle('on', on && m === 'translate'); $('btnRot').classList.toggle('on', on && m === 'rotate'); };

  // ---------------- 3D import ----------------
  P.importFileObj = function (f, at) {
    if (!isModelFile(f.name)) { this.msg('不支持的文件：' + f.name + '（支持 STL / OBJ / GLB / GLTF）', true); return; }
    f.arrayBuffer().then(buf => this.openImport(f.name, buf, at)).catch(e => this.msg('读取文件失败：' + e.message, true));
  };
  /** parse and show the units/material dialog */
  P.openImport = async function (name, buf, at) {
    let soup; try { this.msg('正在解析 ' + name + ' …'); soup = await parseModel(name, buf); } catch (e) { this.msg('导入失败：' + (e && e.message || e), true); return null; }
    return new Promise(res => this.importDialog(name, soup, at, res));
  };
  P.importDialog = function (name, soup, at, done) {
    const bb = soupBox(soup), du = defaultUnit(name), dlg = document.createElement('div'); dlg.id = 'impDlg';
    const autoUnit = () => { const mx = Math.max(...bb.size); let u = du; const sz = mx * UNITS[u]; if (sz > 5 || sz < 0.002) u = ['mm', 'cm', 'inch', 'm'].find(k => { const s = mx * UNITS[k]; return s >= 0.01 && s <= 3; }) || du; return u; };
    dlg.innerHTML = `<div class="dlgbox"><div class="title">导入零件：${name.replace(/</g, '&lt;')}</div>
      <div class="row"><label>三角面</label><span>${Math.round(soup.length / 9)}</span></div>
      <div class="row"><label>文件单位</label><select id="impU">${Object.keys(UNITS).map(k => `<option value="${k}">${UNIT_N[k]}</option>`).join('')}</select></div>
      <div class="row"><label>尺寸</label><span id="impSz"></span></div>
      <div class="row"><label>材料</label><select id="impMat">${Object.entries(MATS).map(([k, m]) => `<option value="${k}" ${k === 'plastic' ? 'selected' : ''}>${m.n}</option>`).join('')}</select></div>
      <div class="row"><label>碰撞体</label><select id="impCol"><option value="hull">凸包（可运动，推荐）</option><option value="tri">三角网格（精确外形，固定于大地）</option></select></div>
      <div class="row"><label></label><label style="width:auto"><input type="checkbox" id="impMine" checked> 加入“我的零件”</label></div>
      <div class="hint2">STL/OBJ 默认按毫米，GLB/GLTF 按米；尺寸异常时自动换算。质量 = 体积 × 材料密度。</div>
      <div style="text-align:right;margin-top:8px"><button class="small" id="impCancel">取消</button> <button class="small run" id="impOk">导入</button></div></div>`;
    document.body.appendChild(dlg);
    const u = dlg.querySelector('#impU'); u.value = autoUnit();
    const upd = () => { const s = UNITS[u.value]; dlg.querySelector('#impSz').textContent = bb.size.map(x => (x * s * 1000).toFixed(1)).join(' × ') + ' mm'; };
    u.onchange = upd; upd();
    const close = () => dlg.remove();
    dlg.querySelector('#impCancel').onclick = () => { close(); this.msg('已取消导入'); done(null); };
    dlg.querySelector('#impOk').onclick = () => { const o = { unit: u.value, mat: dlg.querySelector('#impMat').value, col: dlg.querySelector('#impCol').value, mine: dlg.querySelector('#impMine').checked, at }; close(); done(this.importSoup(name, soup, o)); };
  };
  /** headless / API import: opts { unit, mat, col, mine, at } -> part */
  P.importModel = async function (name, buf, opts = {}) { const soup = await parseModel(name, buf); return this.importSoup(name, soup, opts); };
  P.registerGeo = function (G) { const id = 'geo_' + Date.now().toString(36) + Math.random().toString(36).slice(2, 7); GEO[id] = G; return id; };
  P.importSoup = function (name, soup, o = {}) {
    if (this.mode !== 'edit') this.reset();
    const G = buildGeo(soup, o.unit || defaultUnit(name), name.replace(/\.[^.]+$/, '')), gid = this.registerGeo(G);
    const P = this.placeMesh(gid, o.at || this.freeSpot(G.box), { mat: o.mat || 'plastic', col: o.col || 'hull' });
    if (o.mine !== false) this.addMine(gid);
    const sz = G.box.map(x => (x * 2000).toFixed(1)).join('×');
    this.msg(`已导入 ${P.name}：${sz} mm，${G.tris} 三角面，体积 ${(G.vol * 1e6).toFixed(1)} cm³，质量 ${P.mass.toFixed(3)} kg（${P.d.col === 'tri' ? '三角网格碰撞体，固定' : '凸包碰撞体'}）。拖到其它零件的面上即可贴合；用“＋连接点”添加孔/轴`);
    return P;
  };
  P.placeMesh = function (gid, at, o = {}) {
    const G = GEO[gid], over = { d: { g: gid, sc: 1, col: o.col || 'hull' }, mat: o.mat || 'plastic', name: this.uniqueName(G.name || '导入零件') };
    if (over.d.col === 'tri') over.fixed = true;
    return this.placePart('mesh', at, over, o.asm);
  };
  /** a ground spot near the view centre that does not overlap existing parts */
  P.freeSpot = function (half) {
    const c = this.orbit.target.clone().setY(0), bb = new THREE.Box3(); for (const id in this.objs) bb.expandByObject(this.objs[id]);
    if (bb.isEmpty()) return c; const tb = new THREE.Box3(c.clone().sub(V3(half)), c.clone().add(V3(half))); tb.min.y = 0;
    if (!tb.intersectsBox(bb)) return c; return new THREE.Vector3(bb.max.x + half[0] + 0.05, 0, c.z);
  };
  P.uniqueName = function (base) { const names = new Set(this.M.parts.map(p => p.name)); if (!names.has(base)) return base; for (let i = 2; ; i++) if (!names.has(base + ' ' + i)) return base + ' ' + i; };
  // ---------------- 我的零件 (localStorage) ----------------
  P.mineList = function () { try { return JSON.parse(localStorage.getItem('mech3d-myparts') || '[]'); } catch (e) { return []; } };
  P.saveMine = function (L) { try { localStorage.setItem('mech3d-myparts', JSON.stringify(L)); return true; } catch (e) { this.msg('浏览器存储空间不足，未能保存到“我的零件”', true); return false; } };
  P.addMine = function (gid) { const L = this.mineList().filter(x => x.gid !== gid); L.unshift({ gid, geo: GEO[gid] }); this.saveMine(L.slice(0, 30)); this.renderMine(); };
  P.renderMine = function () {
    const el = $('palMine'); if (!el) return; el.innerHTML = ''; const L = this.mineList();
    for (const it of L) { if (!GEO[it.gid]) GEO[it.gid] = it.geo;
      const d = document.createElement('div'); d.className = 'item mine'; d.dataset.type = 'mesh'; d.dataset.gid = it.gid; d.title = it.geo.name + '（拖到场景）';
      d.innerHTML = `<span class="dot" style="background:#3b82f6"></span>${(it.geo.name || '零件').slice(0, 10)}<span class="mdel" title="从我的零件中移除">✕</span>`; el.appendChild(d);
      d.querySelector('.mdel').addEventListener('pointerdown', e => { e.stopPropagation(); e.preventDefault(); this.saveMine(this.mineList().filter(x => x.gid !== it.gid)); this.renderMine(); });
      this.paletteDrag(d, 'mesh', { d: { g: it.gid, sc: 1, col: 'hull' }, name: it.geo.name, mat: 'plastic' }); }
    $('mineHdr').style.display = L.length ? '' : 'none'; el.style.display = L.length ? '' : 'none';
  };

  // ---------------- custom connectors (添加连接点) ----------------
  P.conClick = function (h) {
    if (!h.pid) { this.msg('请点击零件表面放置连接点', true); return; }
    const A = this.part(h.pid), n = h.normal || new THREE.Vector3(0, 1, 0);
    const lp = ptL(A, h.point), la = axL(A, n), g = v => +(Math.round(v / 0.0005) * 0.0005).toFixed(4);
    this.beginEdit(); A.cons = A.cons || [];
    const nm = ROLE_N[this.conRole] + (A.cons.filter(c => c.role === this.conRole).length + 1);
    A.cons.push({ n: nm, role: this.conRole, p: lp.toArray().map(g), a: la.toArray().map(v => +v.toFixed(5)) });
    this.commit(); this.rebuild(); this.select({ p: A.id });
    this.msg(`已在 ${A.name} 上添加连接点“${nm}”（${ROLE_N[this.conRole]}，轴线沿表面法向）。拖动零件靠近即可自动装配`);
    return A.cons[A.cons.length - 1];
  };
  P.addConnector = function (pid, role, localP, localA, name) { const A = this.part(pid); this.beginEdit(); A.cons = A.cons || []; const c = { n: name || ROLE_N[role] + (A.cons.length + 1), role, p: localP, a: V3(localA).normalize().toArray() }; A.cons.push(c); this.commit(); this.rebuild(); return c; };
  P.removeConnector = function (pid, i) { const A = this.part(pid); if (!A || !A.cons) return; this.beginEdit(); A.cons.splice(i, 1); if (!A.cons.length) delete A.cons; this.commit(); this.rebuild(); };
  /** visual markers for user connectors (children of the part object, not pickable) */
  P.decorate = function (Pt, g) {
    if (!Pt.cons) return;
    for (const c of Pt.cons) {
      const col = c.role === 'pin' ? 0x0ea5e9 : c.role === 'mount' ? 0x22c55e : 0xf43f5e, m = new THREE.MeshBasicMaterial({ color: col, depthTest: false, transparent: true, opacity: 0.85 });
      const o = new THREE.Group(), t = new THREE.Mesh(c.role === 'mount' ? new THREE.BoxGeometry(0.012, 0.012, 0.0015) : new THREE.TorusGeometry(0.006, 0.0015, 6, 18), m), l = new THREE.Mesh(new THREE.CylinderGeometry(0.0008, 0.0008, 0.02, 6), m);
      l.rotation.x = Math.PI / 2; l.position.z = 0.01; o.add(t); o.add(l); o.position.fromArray(c.p); o.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), V3(c.a).normalize());
      o.traverse(x => { x.raycast = () => { }; x.renderOrder = 11; x.userData.helper = true; }); g.add(o);
    }
  };

  // ---------------- two-click mate tool (配合) ----------------
  /** reference on a part: nearest connector within 14 px, else the clicked surface point + normal */
  P.mateRef = function (h) {
    const A = this.part(h.pid), sc = this.toScreen(h.point); let best = null;
    for (const c of connectors(A)) { const w = ptW(A, c.p), d = this.toScreen(w).distanceTo(sc); if (d < 14 && (!best || d < best.d)) best = { d, c, w }; }
    if (best) return { pid: A.id, p: best.c.p.slice(), n: best.c.a.slice(), name: best.c.n };
    return { pid: A.id, p: ptL(A, h.point).toArray(), n: axL(A, h.normal || new THREE.Vector3(0, 1, 0)).toArray(), name: '表面点' };
  };
  P.mateClick = function (h) {
    if (!h.pid) { this.msg(this.mate1 ? '请点击第二个零件 B 的面或连接点' : '请点击零件 A 上要贴合的面或连接点', true); return; }
    if (!this.mate1) { this.mate1 = this.mateRef(h); this.hl2.position.copy(ptW(this.part(h.pid), this.mate1.p)); this.hl2.visible = true; this.msg(`配合：已选 ${this.part(h.pid).name}·${this.mate1.name}，再点击零件 B 上的目标面/连接点`); this.showToolBar(); return; }
    if (h.pid === this.mate1.pid) { this.msg('请点击另一个零件', true); return; }
    const r2 = this.mateRef(h), r1 = this.mate1; this.mate1 = null; this.hl2.visible = false;
    this.mateStart(r1, r2);
  };
  /** start an interactive mate (live preview, edit in the tool bar, 确定 commits) */
  P.mateStart = function (r1, r2, o = {}) {
    if (this.mode !== 'edit') this.reset();
    this.mateS = { snap: this.snapshot(), r1, r2, o: Object.assign({ type: this.mateType || 'fixed', offset: 0, angle: 0, flip: false }, o) };
    this.mateApply(); this.showToolBar();
  };
  P.mateApply = function () {
    const S = this.mateS; if (!S) return null; this.M = JSON.parse(S.snap);
    const j = this.mateCore(S.r1, S.r2, S.o); this.rebuild(true); this.updProps();
    if (j) this.msg(`配合预览：${this.part(S.r1.pid).name} → ${this.part(S.r2.pid).name}（${({ fixed: '固定', revolute: '铰链', prismatic: '滑动', spherical: '球铰' })[S.o.type]}）。在上方工具条调整偏移/角度后点“确定”`);
    return j;
  };
  P.mateCommit = function () { const S = this.mateS; if (!S) return; this.mateS = null; this.lastSnap = S.snap; this.commit(); this.relaxAll(); this.rebuild(); this.showToolBar(); this.msg('✓ 配合完成'); };
  P.mateCancel = function () { const S = this.mateS; if (!S) return; this.mateS = null; this.M = JSON.parse(S.snap); this.rebuild(); this.showToolBar(); this.msg('已取消配合'); };
  /** one-shot API: mate(refA, refB, {type, offset(m), angle(deg), flip}) -> joint */
  P.mate = function (r1, r2, o = {}) { this.mateStart(r1, r2, o); const j = this.M.joints[this.M.joints.length - 1]; this.mateCommit(); return this.M.joints.find(x => x.id === (j && j.id)) || null; };
  /** move the free side so A's point/normal is flush (normals opposed) with B's, then add the joint */
  P.mateCore = function (r1, r2, o) {
    const A = this.part(r1.pid), B = this.part(r2.pid); if (!A || !B) return null;
    const an = analyze(this.M), cA = an.comps[an.compOf[A.id]], cB = an.comps[an.compOf[B.id]], same = an.compOf[A.id] === an.compOf[B.id];
    let mv = null, from = r1, to = r2;
    if (!same && !cA.anchored) mv = cA; else if (!same && !cB.anchored) { mv = cB; from = r2; to = r1; }
    const F = this.part(from.pid), T = this.part(to.pid);
    const nT = axW(T, to.n), pT = ptW(T, to.p);
    if (mv) {
      const nF = axW(F, from.n), pF = ptW(F, from.p), want = o.flip ? nT.clone() : nT.clone().negate();
      const R = new THREE.Quaternion().setFromUnitVectors(nF, want); R.premultiply(new THREE.Quaternion().setFromAxisAngle(nT, (o.angle || 0) * D2R));
      const tgt = pT.clone().addScaledVector(nT, o.offset || 0);
      for (const Pp of mv.parts) { Pp.p = V3(Pp.p).sub(pF).applyQuaternion(R).add(tgt).toArray(); Pp.q = R.clone().multiply(Q4(Pp.q)).toArray(); if (Pp.type !== 'mesh' || !Pp.fixed) { } }
    } else if (!same) this.msg('两个零件都已固定/锚定：不移动，直接在 B 的点处建立连接', true);
    const type = o.type || 'fixed', axis = o.axis && o.axis !== 'n' ? new THREE.Vector3(o.axis === 'X' ? 1 : 0, o.axis === 'Y' ? 1 : 0, o.axis === 'Z' ? 1 : 0) : nT;
    const anchor = pT.clone().addScaledVector(nT, (o.offset || 0) / 2);
    const extra = Object.assign({}, o.joint || {});
    return this.addJoint(type, T, F, anchor, axis, extra);
  };
  P._anyPerpW = function (n) { const r = Math.abs(n.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0); return r.sub(n.clone().multiplyScalar(r.dot(n))).normalize().toArray(); };

  // ---------------- floating tool bar (mate / add connector) ----------------
  P.showToolBar = function () {
    const el = $('tbar2'); if (!el) return; const t = this.tool;
    ['btnMate', 'btnAddCon'].forEach(id => $(id) && $(id).classList.toggle('on', (id === 'btnMate' && t === 'mate') || (id === 'btnAddCon' && t === 'addcon')));
    if (this.mateS) {
      const o = this.mateS.o;
      el.innerHTML = `<b>配合</b> 类型 <select id="mtT">${[['fixed', '固定'], ['revolute', '铰链'], ['prismatic', '滑动'], ['spherical', '球铰']].map(([k, n]) => `<option value="${k}" ${k === o.type ? 'selected' : ''}>${n}</option>`).join('')}</select>
        轴线 <select id="mtX">${[['n', '法向'], ['X', 'X'], ['Y', 'Y(竖直)'], ['Z', 'Z']].map(([k, n]) => `<option value="${k}" ${k === (o.axis || 'n') ? 'selected' : ''}>${n}</option>`).join('')}</select>
        偏移 <input id="mtO" type="number" step="1" value="${((o.offset || 0) * 1000).toFixed(1)}" style="width:60px">mm 角度 <input id="mtA" type="number" step="15" value="${o.angle || 0}" style="width:56px">° <label><input type="checkbox" id="mtF" ${o.flip ? 'checked' : ''}>同向</label>
        <button class="small run" id="mtOk">确定</button><button class="small" id="mtNo">取消</button>`;
      el.style.display = 'flex';
      const rd = () => { o.type = $('mtT').value; this.mateType = o.type; o.axis = $('mtX').value; o.offset = +$('mtO').value / 1000; o.angle = +$('mtA').value; o.flip = $('mtF').checked; this.mateApply(); };
      ['mtT', 'mtX', 'mtO', 'mtA', 'mtF'].forEach(id => $(id).onchange = rd);
      $('mtOk').onclick = () => this.mateCommit(); $('mtNo').onclick = () => this.mateCancel();
      return;
    }
    if (t === 'mate') { el.innerHTML = `<b>🧲 配合</b>${this.mate1 ? '第 2 步：点击零件 B 的目标面 / 连接点' : '第 1 步：点击零件 A 上要贴合的面 / 连接点（蓝色连接点附近会自动吸附）'} <button class="small" id="mtExit">退出 (Esc)</button>`; el.style.display = 'flex'; $('mtExit').onclick = () => this.setTool('select'); return; }
    if (t === 'addcon') {
      el.innerHTML = `<b>＋连接点</b> 类型 <select id="cnR"><option value="hole">孔（可装轴/轮/杆孔）</option><option value="pin">轴（可套孔/轮毂）</option><option value="mount">安装面（面贴合点）</option></select> 点击零件表面放置（轴线=表面法向） <button class="small" id="cnExit">完成 (Esc)</button>`;
      el.style.display = 'flex'; $('cnR').value = this.conRole; $('cnR').onchange = e => this.conRole = e.target.value; $('cnExit').onclick = () => this.setTool('select'); return;
    }
    el.style.display = 'none'; el.innerHTML = '';
  };

  // ---------------- groups / multi-selection / copy / mirror ----------------
  P.selIds = function () { const s = new Set(this.msel || []); if (this.sel && this.sel.p) s.add(this.sel.p); return [...s].filter(id => this.part(id)); };
  P.groupOf = function (pid) { const Pp = this.part(pid); return Pp && Pp.grp ? this.M.parts.filter(q => q.grp === Pp.grp).map(q => q.id) : [pid]; };
  P.groupSel = function (ids = this.selIds()) {
    if (ids.length < 2) { this.msg('至少选中 2 个零件才能组合（Ctrl/Shift+点击多选）', true); return null; }
    this.beginEdit(); const g = 'g' + (this.M.nid++); for (const id of ids) this.part(id).grp = g; this.M.gnames = this.M.gnames || {};
    this.M.gnames[g] = '组合 ' + (Object.keys(this.M.gnames).length);
    this.commit(); this.rebuild(); this.msel = new Set(ids); this.select({ p: ids[0] }, true); this.msg('已组合 ' + ids.length + ' 个零件：点击其中任一零件会选中整组，可整体拖动 / 复制 / 镜像'); return g;
  };
  P.ungroupSel = function (ids = this.selIds()) { this.beginEdit(); for (const id of ids) delete this.part(id).grp; this.commit(); this.rebuild(); this.msg('已解除组合'); };
  /** duplicate (mode 'copy', o.off = [dx,dy,dz]) or mirror-copy (mode 'mirror', o.axis 0|1|2, o.at plane coordinate, o.ext copy joints to outside parts) */
  P.cloneSet = function (ids, mode, o = {}) {
    if (this.mode !== 'edit') this.reset(); ids = ids.filter(id => this.part(id)); if (!ids.length) return [];
    this.beginEdit();
    const set = new Set(ids), map = {}, tf = {}, grp = {}, k = o.axis != null ? o.axis : 2, at = o.at || 0, off = V3(o.off || [0.06, 0, 0.06]);
    const S = reflectM(k), rp = v => { const a = v.slice(); a[k] = 2 * at - a[k]; return a; };
    for (const id of ids) {
      const Pp = this.part(id), C = JSON.parse(JSON.stringify(Pp)); C.id = 'p' + (this.M.nid++); delete C.locked; delete C.hidden;
      C.name = mode === 'mirror' ? this.uniqueName(Pp.name + ' 镜像') : this.uniqueName(Pp.name + ' 副本');
      if (Pp.grp) C.grp = grp[Pp.grp] || (grp[Pp.grp] = 'g' + (this.M.nid++));
      if (mode === 'mirror') {
        const t = SYM[Pp.type] != null ? SYM[Pp.type] : 2; tf[id] = t;
        if (Pp.type === 'mesh') C.d.g = this.mirrorGeoId(Pp.d.g);
        C.p = rp(Pp.p);
        const m = new THREE.Matrix4().multiplyMatrices(S, new THREE.Matrix4().makeRotationFromQuaternion(Q4(Pp.q))).multiply(reflectM(t));
        C.q = new THREE.Quaternion().setFromRotationMatrix(m).normalize().toArray();
        if (C.cons) C.cons = C.cons.map(c => Object.assign({}, c, { p: refl(c.p, t), a: refl(c.a, t) }));
      } else C.p = V3(Pp.p).add(off).toArray();
      this.M.parts.push(C); map[id] = C.id;
    }
    if (this.M.gnames) for (const g in grp) this.M.gnames[grp[g]] = (this.M.gnames[g] || '组合') + (mode === 'mirror' ? ' 镜像' : ' 副本');
    const jmap = {};
    for (const j of this.M.joints.slice()) {
      const ia = set.has(j.a), ib = j.b != null && set.has(j.b); if (!ia && !ib) continue;
      const ext = !(ia && ib); if (ext && !(mode === 'mirror' && o.ext)) continue;
      const J = JSON.parse(JSON.stringify(j)); J.id = 'j' + (this.M.nid++); jmap[j.id] = J.id;
      J.a = ia ? map[j.a] : j.a; J.b = j.b == null ? null : (ib ? map[j.b] : j.b);
      if (mode === 'mirror') {
        const loc = (pid, l, inSet) => { if (inSet) return refl(l, tf[pid]); if (pid == null) return rp(l); const E = this.part(pid); return ptL(E, V3(rp(ptW(E, l).toArray()))).toArray(); };
        J.la = loc(j.a, j.la, ia); J.lb = loc(j.b, j.lb, ib);
        let a = ia ? refl(j.axa, tf[j.a]) : axL(this.part(j.a), V3(refl(axW(this.part(j.a), j.axa).toArray(), k))).toArray();
        if (j.type === 'revolute' || j.type === 'helical') { a = a.map(v => -v); if (j.lim) J.lim = { on: j.lim.on, lo: -j.lim.hi, hi: -j.lim.lo }; }
        J.axa = a;
      }
      this.M.joints.push(J);
    }
    for (const c of this.M.couplings.slice()) if (set.has(c.pa) && set.has(c.pb) && jmap[c.ja] && jmap[c.jb]) { const C = JSON.parse(JSON.stringify(c)); C.id = 'c' + (this.M.nid++); C.pa = map[c.pa]; C.pb = map[c.pb]; C.ja = jmap[c.ja]; C.jb = jmap[c.jb]; this.M.couplings.push(C); }
    this.commit(); if (mode === 'mirror' && o.ext) this.relaxAll(); else this.liftComp(map[ids[0]]); this.rebuild();
    const nids = ids.map(id => map[id]); this.msel = new Set(nids); this.select({ p: nids[0] }, true);
    this.msg((mode === 'mirror' ? '已镜像复制 ' : '已复制 ') + nids.length + ' 个零件、' + Object.keys(jmap).length + ' 个连接' + (mode === 'mirror' ? '（镜像平面 ' + 'XYZ'[k] + ' = ' + (at * 1000).toFixed(1) + ' mm）' : ''));
    return nids;
  };
  P.mirrorGeoId = function (g) { if (g.endsWith('~m')) return g.slice(0, -2); const id = g + '~m'; if (!GEO[id] && GEO[g]) GEO[id] = mirrorGeo(GEO[g]); return id; };
  /** default mirror plane: centre of the first outside part connected to the set (e.g. the chassis), else 0 */
  P.mirrorDefault = function (ids, k) {
    const set = new Set(ids);
    for (const j of this.M.joints) { const ia = set.has(j.a), ib = j.b != null && set.has(j.b); if (ia !== ib) { const o = ia ? j.b : j.a; if (o) return this.part(o).p[k]; } }
    return 0;
  };
  P.multiPanel = function (el, ids) {
    const ed = this.mode === 'edit', grouped = ids.every(id => this.part(id).grp) && new Set(ids.map(id => this.part(id).grp)).size === 1;
    const g = grouped ? this.part(ids[0]).grp : null, gn = g && this.M.gnames && this.M.gnames[g];
    el.innerHTML = `<div class="title">${grouped ? '组合：' + (gn || g) : '多选'}（${ids.length} 个零件）</div><div class="hint2">${ids.map(id => this.part(id).name).join('、')}</div>`;
    if (!ed) return;
    const d = document.createElement('div'); d.className = 'mpanel';
    d.innerHTML = `${grouped ? '<button class="small" id="mUngrp">解除组合</button>' : '<button class="small run" id="mGrp">组合 (Ctrl+G)</button>'} <button class="small" id="mDup">复制 (Ctrl+D)</button>
      <div class="row"><label>镜像复制</label><select id="mAx" style="flex:none;width:52px"><option value="2">Z</option><option value="0">X</option><option value="1">Y</option></select><input id="mAt" type="number" step="1" style="width:70px"><span class="u">mm</span></div>
      <div class="row"><label></label><label style="width:auto"><input type="checkbox" id="mExt" checked> 同时复制与外部零件的连接</label></div>
      <button class="small run" id="mMir">⇋ 镜像复制</button> <button class="del" id="mDel">删除全部</button>
      <div class="hint2">例：装好一侧车轮组件（轮 + 转向块 + 连接），组合后“镜像复制”到另一侧；镜像平面默认取所连接零件（如车架）的中心。</div>`;
    el.appendChild(d);
    const ax = d.querySelector('#mAx'), atI = d.querySelector('#mAt'), setAt = () => { atI.value = (this.mirrorDefault(ids, +ax.value) * 1000).toFixed(1); }; setAt(); ax.onchange = setAt;
    if (grouped) d.querySelector('#mUngrp').onclick = () => this.ungroupSel(ids); else d.querySelector('#mGrp').onclick = () => this.groupSel(ids);
    d.querySelector('#mDup').onclick = () => this.cloneSet(ids, 'copy');
    d.querySelector('#mMir').onclick = () => this.cloneSet(ids, 'mirror', { axis: +ax.value, at: +atI.value / 1000, ext: d.querySelector('#mExt').checked });
    d.querySelector('#mDel').onclick = () => this.deleteParts(ids);
  };

  // ---------------- run-mode keyboard input ----------------
  P.boundKeys = function () { const s = new Set(); for (const j of this.M.joints) if (j.drive && j.drive.on) { if (j.drive.keyP) s.add(j.drive.keyP); if (j.drive.keyN) s.add(j.drive.keyN); } return s; };
  P.runKey = function (e, down) {
    if (this.mode === 'edit') return false; const b = this.boundKeys(); if (!b.has(e.code)) return false;
    e.preventDefault(); if (down) this.keys.add(e.code); else this.keys.delete(e.code); return true;
  };
  /** test helper: press / release a bound key programmatically */
  P.setKey = function (code, on) { if (on) this.keys.add(code); else this.keys.delete(code); };

  // ---------------- save / load helpers ----------------
  P.usedGeos = function (M) { const o = {}; for (const Pp of M.parts) if (Pp.type === 'mesh' && GEO[Pp.d.g]) o[Pp.d.g] = GEO[Pp.d.g]; return o; };
  P.saveData = function () { const model = this.mode === 'edit' ? this.M : JSON.parse(this.init); return { app: 'mech-sandbox-3d', version: 2, model, geos: this.usedGeos(model), camera: { p: this.cam.position.toArray(), t: this.orbit.target.toArray() } }; };
}
