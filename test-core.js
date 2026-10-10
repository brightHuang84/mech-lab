// node test-core.js — 提取 index.html 中 CORE 段并数值验证公式
const fs = require('fs');
const html = fs.readFileSync(__dirname + '/index.html', 'utf8');
const m = html.match(/\/\/ ==CORE-BEGIN==([\s\S]*?)\/\/ ==CORE-END==/);
if (!m) throw new Error('CORE block not found');
const core = new Function(m[1] + '; return {kin, dyn, computeCurves, statsOf, validate, G0};')();
const {kin, dyn, computeCurves, statsOf, validate, G0} = core;
let pass = 0, fail = 0;
function check(name, ok, info){ (ok ? pass++ : fail++); console.log((ok ? 'PASS ' : 'FAIL ') + name + (info !== undefined ? '  ' + info : '')); }
const near = (a, b, tol) => Math.abs(a - b) <= tol*Math.max(1, Math.abs(a), Math.abs(b));
const D = Math.PI/180;
const cases = [{r:60,l:180},{r:30,l:35},{r:100,l:101},{r:10,l:500},{r:75.5,l:210.3}];

for (const {r,l} of cases){
  const tag = `[r=${r},l=${l}] `;
  // 1 位置公式 + 独立几何闭环 |B−C| = l
  let maxLoop = 0, maxForm = 0;
  for (let a = 0; a < 360; a += 0.5){
    const th = a*D, k = kin(r,l,th);
    const form = r*Math.cos(th) + Math.sqrt(l*l - r*r*Math.sin(th)**2);
    maxForm = Math.max(maxForm, Math.abs(k.x - form));
    const bx = r*Math.cos(th), by = r*Math.sin(th);
    maxLoop = Math.max(maxLoop, Math.abs(Math.hypot(k.x - bx, by) - l));
  }
  check(tag + '位置 = r cosθ + sqrt(l²−r² sin²θ)', maxForm < 1e-12, 'maxErr=' + maxForm.toExponential(2));
  check(tag + '几何闭环 |BC| = l', maxLoop < 1e-9, 'maxErr=' + maxLoop.toExponential(2));
  // 2 行程 = 2r,死点位置
  const c = computeCurves({r,l,rpm:100,m:1,F:0,mu:0,g:false,loadMode:'const'}, 720), s = statsOf(c);
  check(tag + '行程 = 2r', near(s.stroke, 2*r, 1e-12), `${s.stroke} vs ${2*r}`);
  check(tag + '外/内止点 x = l±r', near(s.xmax, l + r, 1e-12) && near(s.xmin, l - r, 1e-12));
  check(tag + '止点处速度为 0', Math.abs(kin(r,l,0).dx) < 1e-12 && Math.abs(kin(r,l,Math.PI).dx) < 1e-9);
  // 3 速度/加速度 vs 中心差分(解析 vs 数值)
  let ev = 0, ea = 0; const h = 1e-3;
  const d1 = (fn, th) => (-fn(th + 2*h) + 8*fn(th + h) - 8*fn(th - h) + fn(th - 2*h))/(12*h); // 五点中心差分
  for (let a = 3; a < 360; a += 7.3){
    const th = a*D, k = kin(r,l,th);
    const dnum = d1(t => kin(r,l,t).x, th);        // 位置 → 速度(数值)
    const d2num = d1(t => kin(r,l,t).dx, th);      // 解析速度 → 加速度(数值)
    const d2num2 = d1(t => d1(u => kin(r,l,u).x, t), th); // 位置二次数值微分(交叉验证)
    ev = Math.max(ev, Math.abs(dnum - k.dx)/Math.max(1, Math.abs(k.dx)));
    const rel = v => Math.abs(v - k.d2x)/Math.max(1, Math.abs(k.d2x));
    ea = Math.max(ea, rel(d2num), rel(d2num2));
    // 速度的另一种已知形式 dx/dθ = −r sin(θ+φ)/cosφ
    if (!near(k.dx, -r*Math.sin(th + k.phi)/k.cosphi, 1e-10)) { ev = 1; }
  }
  check(tag + 'dx/dθ 解析 = 数值微分', ev < 1e-7, 'relErr=' + ev.toExponential(2));
  check(tag + 'd²x/dθ² 解析 = 数值二阶差分', ea < 1e-7, 'relErr=' + ea.toExponential(2));
}

// 4 动力学:虚功/功率守恒  τ·ω = −T cosφ · v  (质量、载荷、摩擦均开启)
{
  const p = {r:60,l:180,rpm:300,m:2.5,F:80,mu:0.15,g:true,loadMode:'oppose'};
  let e = 0, e2 = 0;
  for (let a = 0; a < 360; a += 1.3){
    const d = dyn(p, a*D), k = kin(p.r, p.l, a*D);
    const pw1 = d.tau*d.w, pw2 = -d.T*k.cosphi*d.v;
    e = Math.max(e, Math.abs(pw1 - pw2)/Math.max(1, Math.abs(pw1)));
    // 滑块 x 向动力学残差:m a = −T cosφ + Fx + Ff
    const res = p.m*d.a - (-d.T*k.cosphi + d.Fx + d.Ff);
    // y 向:N + T sinφ = m g
    const resy = d.N + d.T*k.sinphi - p.m*G0;
    e2 = Math.max(e2, Math.abs(res), Math.abs(resy));
  }
  check('功率守恒 τω = −T cosφ v', e < 1e-9, 'relErr=' + e.toExponential(2));
  check('滑块 x/y 方向牛顿方程残差 ≈ 0', e2 < 1e-7, 'maxRes=' + e2.toExponential(2));
}
// 5 纯惯性(无载荷无摩擦无重力):一周期平均扭矩 = 0(能量守恒)
{
  const c = computeCurves({r:60,l:180,rpm:600,m:3,F:0,mu:0,g:false,loadMode:'const'}, 7200), s = statsOf(c);
  check('纯惯性:平均扭矩 = 0', Math.abs(s.taumean) < 1e-3*s.taupeak, `mean=${s.taumean.toExponential(2)} peak=${s.taupeak.toFixed(3)}`);
}
// 6 恒定方向载荷 F(沿 −x,保守力):平均扭矩 ≈ 0;阻碍运动型:平均扭矩 = 2 F r /π·(1/…) → 4 F r /(2π)
{
  const base = {r:50,l:200,rpm:120,m:0,F:100,mu:0,g:false};
  const s1 = statsOf(computeCurves({...base, loadMode:'const'}, 7200));
  check('恒定 −x 载荷:平均扭矩 = 0(保守力)', Math.abs(s1.taumean) < 2e-3*s1.taupeak, 'mean=' + s1.taumean.toExponential(2));
  const s2 = statsOf(computeCurves({...base, loadMode:'oppose'}, 7200));
  const expect = 4*base.F*(base.r/1000)/(2*Math.PI);
  check('阻碍型载荷:平均扭矩 = 4Fr/(2π)(每周期克服做功 F·2S)', near(s2.taumean, expect, 2e-3), `${s2.taumean.toFixed(5)} vs ${expect.toFixed(5)}`);
}
// 7 摩擦做功:μ·m·g 在 g 开启时一周期耗散 = μ m g · 2S → 平均扭矩 = 4 μ m g r/(2π)(小 φ 近似下非严格,允许 3%)
{
  const p = {r:50,l:400,rpm:60,m:5,F:0,mu:0.2,g:true,loadMode:'const'};
  const s = statsOf(computeCurves(p, 7200));
  const expect = 4*p.mu*p.m*G0*(p.r/1000)/(2*Math.PI);
  check('摩擦耗散:平均扭矩 ≈ 4μmg·r/(2π)', near(s.taumean, expect, 0.03), `${s.taumean.toFixed(5)} vs ${expect.toFixed(5)} (含 T sinφ 对 N 的修正)`);
}
// 8 准静态(转速≈0,无质量惯性影响):载荷 F 沿 −x ⇒ 杆受压 T = −F/cosφ
{
  const p = {r:50,l:200,rpm:0.001,m:1,F:100,mu:0,g:false,loadMode:'const'};
  const d = dyn(p, 90*D), k = kin(p.r, p.l, 90*D);
  check('准静态:T = −F/cosφ(压杆)', near(d.T, -100/k.cosphi, 1e-6), `T=${d.T.toFixed(4)} expect ${(-100/k.cosphi).toFixed(4)}`);
}
// 9 有效性检查
check('validate: l ≤ r 无效', !validate({r:50,l:50}).ok && !validate({r:50,l:40}).ok && validate({r:50,l:51}).ok);
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
