import * as THREE from 'three';
import { buildGenericCar } from './examples.js';
const D2R = Math.PI / 180;
// api: P(type, pos, eulerDeg|quat, over) -> part ; J(type, A, B|null, anchor, axis, over) ; C(type, A, B, extra) ; rod(p1,p2,over)
function circInter(p0, r0, p1, r1, upper) {
  const dx = p1[0] - p0[0], dy = p1[1] - p0[1], d = Math.hypot(dx, dy);
  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d), h = Math.sqrt(Math.max(0, r0 * r0 - a * a));
  const mx = p0[0] + a * dx / d, my = p0[1] + a * dy / d;
  const s1 = [mx + h * dy / d, my - h * dx / d], s2 = [mx - h * dy / d, my + h * dx / d];
  return (s1[1] > s2[1]) === !!upper ? s1 : s2;
}
const Zx = [0, 0, 1];
export const PRESETS = [
  { name: '单摆', desc: '底座 + 轻杆(铰链) + 重物球；T≈2π√(L/g)', build({ P, J, rod }) {
    const piv = [0, 1.3, 0], L = 1.0, th = 10 * D2R, e = [piv[0] + L * Math.sin(th), piv[1] - L * Math.cos(th), 0];
    const base = P('base', [0, 0.7, -0.044], null, { name: '支架', d: { w: 0.1, h: 1.4, dp: 0.08 } });
    const r = rod(piv, e, { name: '摆杆', d: { L, w: 0.012, t: 0.008 }, mass: 0.05 });
    const ball = P('ball', e, null, { name: '摆锤', d: { r: 0.04 }, trace: true });
    J('revolute', r, base, [piv[0], piv[1], -0.004], Zx); J('fixed', ball, r, e, Zx);
  } },
  { name: '曲柄滑块', desc: '电机 + 曲柄 + 连杆 + 滑块(滑动副)；行程 = 2×曲柄半径', build({ P, J, rod }) {
    const O = [0, 0.4, 0], r = 0.1, l = 0.3, a0 = 60 * D2R, B = [O[0] + r * Math.cos(a0), O[1] + r * Math.sin(a0), 0.012], S = [B[0] + Math.sqrt(l * l - (B[1] - O[1]) ** 2), O[1], 0.012];
    const mot = P('motor', [0, 0.4, -0.088], null, { name: '电机', motor: { mode: 'speed', rpm: 60, torque: 1, maxT: 30 } });
    J('fixed', P('base', [0, 0.3576 / 2, -0.088], null, { name: '电机座', d: { w: 0.1, h: 0.3576, dp: 0.1 } }), mot, [0, 0.3576, -0.088], [0, 1, 0]);
    const cr = rod([O[0], O[1], 0], [B[0], B[1], 0], { name: '曲柄', d: { L: r, w: 0.03, t: 0.012 } });
    const co = rod(B, S, { name: '连杆', d: { L: l, w: 0.025, t: 0.01 } });
    const sl = P('block', S, null, { name: '滑块', trace: true });
    const rail = P('plate', [0.3, 0.4 - 0.035, 0.012], null, { name: '导轨', fixed: true, d: { w: 0.5, h: 0.02, dp: 0.06 }, mat: 'steel' });
    J('revolute', mot, cr, [O[0], O[1], 0], Zx); J('revolute', cr, co, [B[0], B[1], 0.0065], Zx); J('revolute', co, sl, S, Zx); J('prismatic', sl, rail, [S[0], 0.375, S[2]], [1, 0, 0]);
  } },
  { name: '四连杆', desc: '曲柄摇杆机构（电机驱动曲柄）', build({ P, J, rod }) {
    const O2 = [0, 0.45], O4 = [0.4, 0.45], a = 0.12, b = 0.4, c = 0.3, th = 70 * D2R;
    const A = [O2[0] + a * Math.cos(th), O2[1] + a * Math.sin(th)], B = circInter(A, b, O4, c, true);
    const mot = P('motor', [O2[0], O2[1], -0.088], null, { name: '电机', motor: { mode: 'speed', rpm: 30, torque: 1, maxT: 30 } });
    J('fixed', P('base', [O2[0], 0.4076 / 2, -0.088], null, { name: '电机座', d: { w: 0.1, h: 0.4076, dp: 0.1 } }), mot, [O2[0], 0.4076, -0.088], [0, 1, 0]);
    const base = P('base', [O4[0], 0.25, -0.035], null, { name: '机架', d: { w: 0.08, h: 0.5, dp: 0.06 } });
    const cr = rod([...O2, 0], [...A, 0], { name: '曲柄', d: { L: a, w: 0.03, t: 0.012 } });
    const cp = rod([...A, 0.011], [...B, 0.011], { name: '连杆', d: { L: b, w: 0.025, t: 0.01 }, trace: true });
    const rk = rod([...O4, 0], [...B, 0], { name: '摇杆', d: { L: c, w: 0.025, t: 0.01 } });
    J('revolute', mot, cr, [...O2, 0], Zx); J('revolute', cr, cp, [...A, 0.006], Zx); J('revolute', cp, rk, [...B, 0.0055], Zx); J('revolute', rk, base, [...O4, -0.005], Zx);
  } },
  { name: '两级齿轮减速器', desc: 'z20→z40→(同轴z20)→z40，总速比 4:1', build({ P, J, C }) {
    const m = 0.004, y = 0.25, a = m * (20 + 40) / 2;
    const plate = P('base', [a, 0.21, -0.05], null, { name: '箱体背板', d: { w: 0.44, h: 0.42, dp: 0.02 } });
    const mot = P('motor', [0, y, -0.12], null, { name: '电机', motor: { mode: 'speed', rpm: 120, torque: 1, maxT: 50 }, d: { s: 0.07, Ls: 0.04 } });
    const s1 = P('shaft', [0, y, 0.0], null, { name: '输入轴', d: { L: 0.1, r: 0.006 } });
    const g1 = P('gear', [0, y, 0], [0, 0, 0], { name: '齿轮1 z20', d: { m, z: 20, t: 0.02 } });
    const s2 = P('shaft', [a, y, 0.01], null, { name: '中间轴', d: { L: 0.12, r: 0.006 } });
    const g2 = P('gear', [a, y, 0], [0, 0, 180 + 180 / 40], { name: '齿轮2 z40', d: { m, z: 40, t: 0.02 }, mat: 'brass' });
    const g3 = P('gear', [a, y, 0.03], [0, 0, 0], { name: '齿轮3 z20', d: { m, z: 20, t: 0.02 } });
    const s3 = P('shaft', [2 * a, y, 0.01], null, { name: '输出轴', d: { L: 0.12, r: 0.006 } });
    const g4 = P('gear', [2 * a, y, 0.03], [0, 0, 180 + 180 / 40], { name: '齿轮4 z40', d: { m, z: 40, t: 0.02 }, mat: 'brass' });
    J('fixed', g1, s1, [0, y, 0], Zx); J('fixed', g2, s2, [a, y, 0], Zx); J('fixed', g3, s2, [a, y, 0.03], Zx); J('fixed', g4, s3, [2 * a, y, 0.03], Zx);
    J('revolute', mot, s1, [0, y, -0.05], Zx); J('revolute', s2, plate, [a, y, -0.04], Zx); J('revolute', s3, plate, [2 * a, y, -0.04], Zx);
    C('gear', g1, g2); C('gear', g3, g4);
  } },
  { name: '锥齿轮传动', desc: '两轴相交 90°，z20 → z30', build({ P, J, C }) {
    const m = 0.004, z1 = 20, z2 = 30, r1 = m * z1 / 2, r2 = m * z2 / 2, O = [0, 0.3, 0];
    const c1 = [O[0], O[1], O[2] + r2], c2 = [O[0] + r1, O[1], O[2]];
    const g1 = P('bevel', c1, [0, 0, 0], { name: '小锥齿轮 z20', d: { m, z: z1, t: 0.016, A: r2 } });
    const g2 = P('bevel', c2, [0, 90, 0], { name: '大锥齿轮 z30', d: { m, z: z2, t: 0.016, A: r1 }, mat: 'steel' });
    // phase: gear2 gap toward +Z
    g2.q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 90 * D2R, 0)).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI + Math.PI / z2)).toArray();
    const s1 = P('shaft', [c1[0], c1[1], c1[2] + 0.05], null, { name: '输入轴', d: { L: 0.1, r: 0.006 } });
    const mot = P('motor', [c1[0], c1[1], c1[2] + 0.18], [0, 180, 0], { name: '电机', motor: { mode: 'speed', rpm: 60, torque: 1, maxT: 30 }, d: { s: 0.07, Ls: 0.04 } });
    { const hb = c1[1] - 0.53 * 0.07; J('fixed', P('base', [c1[0], hb / 2, c1[2] + 0.18], null, { name: '电机座', d: { w: 0.08, h: hb, dp: 0.09 } }), mot, [c1[0], hb, c1[2] + 0.18], [0, 1, 0]); }
    const s2 = P('shaft', [c2[0] + 0.06, c2[1], c2[2]], [0, 90, 0], { name: '输出轴', d: { L: 0.12, r: 0.006 } });
    const brg = P('base', [c2[0] + 0.14, c2[1] / 2 + 0.02, c2[2]], null, { name: '轴承座', d: { w: 0.04, h: c2[1] + 0.04, dp: 0.06 } });
    J('fixed', g1, s1, c1, Zx); J('fixed', g2, s2, c2, [1, 0, 0]);
    J('revolute', mot, s1, [c1[0], c1[1], c1[2] + 0.1], [0, 0, 1]); J('revolute', s2, brg, [c2[0] + 0.12, c2[1], c2[2]], [1, 0, 0]);
    C('gear', g1, g2);
  } },
  { name: '蜗杆蜗轮', desc: '单头蜗杆驱动 z30 蜗轮，速比 30:1', build({ P, J, C }) {
    const m = 0.004, z = 30, R = m * z / 2, rw = 0.015, y = 0.25, yw = y + R + rw;
    const plate = P('base', [0, 0.17, -0.05], null, { name: '箱体背板', d: { w: 0.3, h: 0.34, dp: 0.02 } });
    const wh = P('wormwheel', [0, y, 0], null, { name: '蜗轮 z30', d: { m, z, t: 0.02 } });
    const sw = P('shaft', [0, y, -0.015], null, { name: '蜗轮轴', d: { L: 0.07, r: 0.006 } });
    const wm = P('worm', [0, yw, 0], [0, 90, 0], { name: '蜗杆', d: { m, r: rw, L: 0.12, starts: 1 } });
    const mot = P('motor', [-0.06 - 0.04 - 0.042, yw, 0], [0, 90, 0], { name: '电机', motor: { mode: 'speed', rpm: 300, torque: 1, maxT: 20 }, d: { s: 0.07, Ls: 0.04 } });
    { const hb = yw - 0.53 * 0.07; J('fixed', P('base', [-0.142, hb / 2, 0], null, { name: '电机座', d: { w: 0.08, h: hb, dp: 0.08 } }), mot, [-0.142, hb, 0], [0, 1, 0]); }
    J('fixed', wh, sw, [0, y, 0], Zx); J('revolute', sw, plate, [0, y, -0.04], Zx);
    J('revolute', mot, wm, [-0.06, yw, 0], [1, 0, 0]);
    C('gear', wm, wh);
  } },
  { name: '皮带传动', desc: '小带轮 r50 → 大带轮 r100，同向，速比 2:1', build({ P, J, C }) {
    const y = 0.3;
    const mot = P('motor', [0, y, -0.088], null, { name: '电机', motor: { mode: 'speed', rpm: 120, torque: 1, maxT: 30 } });
    J('fixed', P('base', [0, (y - 0.0424) / 2, -0.088], null, { name: '电机座', d: { w: 0.1, h: y - 0.0424, dp: 0.1 } }), mot, [0, y - 0.0424, -0.088], [0, 1, 0]);
    const p1 = P('pulley', [0, y, 0], null, { name: '主动带轮', d: { r: 0.05, t: 0.025 } });
    const p2 = P('pulley', [0.4, y, 0], null, { name: '从动带轮', d: { r: 0.1, t: 0.025 } });
    const brg = P('base', [0.4, (y + 0.03) / 2, -0.0325], null, { name: '轴承座', d: { w: 0.06, h: y + 0.03, dp: 0.04 } });
    J('revolute', mot, p1, [0, y, 0], Zx); J('revolute', p2, brg, [0.4, y, -0.0125], Zx);
    C('belt', p1, p2);
  } },
  { name: '链传动', desc: '链轮 z16 → z32，同向，速比 2:1', build({ P, J, C }) {
    const y = 0.3;
    const mot = P('motor', [0, y, -0.088], null, { name: '电机', motor: { mode: 'speed', rpm: 60, torque: 1, maxT: 30 } });
    J('fixed', P('base', [0, (y - 0.0424) / 2, -0.088], null, { name: '电机座', d: { w: 0.1, h: y - 0.0424, dp: 0.1 } }), mot, [0, y - 0.0424, -0.088], [0, 1, 0]);
    const p1 = P('sprocket', [0, y, 0], null, { name: '小链轮 z16', d: { z: 16, p: 0.0127, t: 0.008 } });
    const p2 = P('sprocket', [0.35, y, 0], null, { name: '大链轮 z32', d: { z: 32, p: 0.0127, t: 0.008 } });
    const brg = P('base', [0.35, (y + 0.03) / 2, -0.024], null, { name: '轴承座', d: { w: 0.06, h: y + 0.03, dp: 0.04 } });
    J('revolute', mot, p1, [0, y, 0], Zx); J('revolute', p2, brg, [0.35, y, -0.004], Zx);
    C('chain', p1, p2);
  } },
  { name: '凸轮-从动件', desc: '电机驱动盘形凸轮，从动杆靠重力+预压弹簧保持接触（真实碰撞）', build({ P, J }) {
    const y = 0.25, rb = 0.05, h = 0.03, rTop = rb + h / 2, fr = 0.01, fl = 0.25;
    const mot = P('motor', [0, y, -0.088], null, { name: '电机', motor: { mode: 'speed', rpm: 30, torque: 1, maxT: 30 } });
    J('fixed', P('base', [0, (y - 0.0424) / 2, -0.088], null, { name: '电机座', d: { w: 0.1, h: y - 0.0424, dp: 0.1 } }), mot, [0, y - 0.0424, -0.088], [0, 1, 0]);
    const cam = P('cam', [0, y, 0], null, { name: '凸轮', d: { rb, h, t: 0.02 }, mu: 0.05, e: 0 });
    const fy = y + rTop + fr + fl / 2 + 0.001;
    const fo = P('follower', [0, fy, 0], null, { name: '从动件', d: { L: fl, r: fr }, mass: 0.3, trace: true, mu: 0.05, e: 0 });
    P('base', [0.06, 0.3, -0.03], null, { name: '导向架', d: { w: 0.04, h: 0.6, dp: 0.02 } });
    J('revolute', mot, cam, [0, y, 0], Zx); J('prismatic', fo, null, [0, fy - fl / 2, 0], [0, 1, 0]);
    const top = fy + fl / 2 + 0.008; J('spring', fo, null, [0, top, 0], [0, 1, 0], { lb: [0, 0.62, 0], k: 150, c: 0.5, L0: 0.62 - top + 0.06 });
  } },
  { name: '丝杠螺母', desc: '电机驱动丝杠，螺母沿导轨直线移动：v = 导程·n', build({ P, J, C }) {
    const y = 0.15, lead = 0.005;
    const mot = P('motor', [-0.25 - 0.04 - 0.042, y, 0], [0, 90, 0], { name: '电机', motor: { mode: 'speed', rpm: 120, torque: 1, maxT: 20 }, d: { s: 0.07, Ls: 0.04 } });
    { const hb = y - 0.53 * 0.07; J('fixed', P('base', [-0.332, hb / 2, 0], null, { name: '电机座', d: { w: 0.08, h: hb, dp: 0.08 } }), mot, [-0.332, hb, 0], [0, 1, 0]); }
    const sc = P('screw', [0, y, 0], [0, 90, 0], { name: '丝杠', d: { r: 0.01, L: 0.5, lead } });
    const nut = P('nut', [-0.1, y, 0], [0, 90, 0], { name: '螺母', trace: true });
    const tbl = P('block', [-0.1, y + 0.037, 0], null, { name: '工作台', d: { w: 0.08, h: 0.03, dp: 0.1 }, mat: 'alu' });
    P('base', [0, y - 0.04, 0], null, { name: '导轨底座', d: { w: 0.55, h: 0.02, dp: 0.12 }, mat: 'iron' });
    J('revolute', mot, sc, [-0.25, y, 0], [1, 0, 0]); J('fixed', tbl, nut, [-0.1, y + 0.022, 0], Zx);
    J('prismatic', nut, null, [-0.1, y, 0], [1, 0, 0]);
    C('screw', sc, nut);
  } },
  { name: '齿轮齿条', desc: '电机驱动小齿轮，齿条沿滑动副平移', build({ P, J, C }) {
    const m = 0.004, z = 20, r = m * z / 2, yr = 0.15, hr = 0.03, yg = yr + hr / 2 + r;
    const mot = P('motor', [0, yg, -0.088], null, { name: '电机', motor: { mode: 'speed', rpm: 20, torque: 1, maxT: 20 } });
    J('fixed', P('base', [0, (yg - 0.0424) / 2, -0.088], null, { name: '电机座', d: { w: 0.1, h: yg - 0.0424, dp: 0.1 } }), mot, [0, yg - 0.0424, -0.088], [0, 1, 0]);
    const g = P('gear', [0, yg, 0], [0, 0, -90], { name: '小齿轮 z20', d: { m, z, t: 0.02 } });
    const rk = P('rack', [0, yr, 0], null, { name: '齿条', d: { m, L: 0.6, h: hr, t: 0.02 }, trace: true });
    J('revolute', mot, g, [0, yg, 0], Zx); J('prismatic', rk, null, [0, yr, 0], [1, 0, 0]);
    C('gear', g, rk);
  } },
  { name: '简易小车', desc: '两台电机固定在车架上，输出轴直接插入后轮；前轮装在前轴上，前轴由两个固定在车架上的轴承座支承', build({ P, J }) {
    const R = 0.07, tw = 0.03, s = 0.06, Ls = 0.03, hc = 0.53 * s, xb = 0.15, W = 0.42, DP = 0.22, H = 0.02;
    const yTop = R - hc, zt = DP / 2 + 0.005 + tw / 2;           // axle height = plate top + motor shaft height
    const ch = P('plate', [0, yTop - H / 2, 0], null, { name: '车架', d: { w: W, h: H, dp: DP }, trace: true });
    const zm = zt - (0.6 * s + Ls / 2);                         // motor centre so the shaft stub sits in the wheel hub
    const mk = (z, side) => {
      const m = P('motor', [-xb, yTop + hc, side * zm], side > 0 ? null : [0, 180, 0], { name: side > 0 ? '左电机' : '右电机', fixed: false, motor: { mode: 'speed', rpm: 60, torque: 1, maxT: 5 }, d: { s, Ls }, mass: 0.5 });
      J('fixed', ch, m, [-xb, yTop, side * zm], [0, 1, 0]);
      const w = P('wheel', [-xb, R, side * zt], null, { name: side > 0 ? '左后轮' : '右后轮', d: { r: R, t: tw } });
      J('revolute', m, w, [-xb, R, side * zt], [0, 0, side], side > 0 ? { motorRev: true } : {});
      return m;
    };
    mk(zt, 1); mk(-zt, -1);
    // front axle in two bearing blocks
    const bH = R - yTop, ax = P('shaft', [xb, R, 0], null, { name: '前轴', d: { L: 2 * zt + 0.02, r: 0.006 } });
    for (const sd of [1, -1]) {
      const zb = sd * 0.075, b = P('bearing', [xb, R, zb], null, { name: sd > 0 ? '左轴承座' : '右轴承座', d: { w: 0.04, H: bH, dp: 0.02, r: 0.012 } });
      J('fixed', ch, b, [xb, yTop, zb], [0, 1, 0]); J('revolute', b, ax, [xb, R, zb], Zx);
      const w = P('wheel', [xb, R, sd * zt], null, { name: sd > 0 ? '左前轮' : '右前轮', d: { r: R, t: tw } });
      J('fixed', ax, w, [xb, R, sd * zt], Zx);
    }
  } },
  { name: '三轮车', desc: '板车架 + 两台电机直驱后轮 + 前轮装在转向叉(竖直转向销, 自动回正)上；全部由普通零件装配', build({ P, J }) {
    const R = 0.08, tw = 0.03, s = 0.08, Ls = 0.04, hc = 0.53 * s, W = 0.36, DP = 0.2, H = 0.02, xm = 0.1, xk = -0.12;
    const yTop = R - hc, zt = DP / 2 + 0.005 + tw / 2, zm = zt - (0.6 * s + Ls / 2);
    const ch = P('plate', [0, yTop - H / 2, 0], null, { name: '车架', d: { w: W, h: H, dp: DP }, trace: true });
    for (const sd of [1, -1]) {
      const m = P('motor', [xm, yTop + hc, sd * zm], sd > 0 ? null : [0, 180, 0], { name: sd > 0 ? '左电机' : '右电机', fixed: false, motor: { mode: 'speed', rpm: 60, torque: 1, maxT: 20 }, d: { s, Ls } });
      J('fixed', ch, m, [xm, yTop, sd * zm], [0, 1, 0]);
      const w = P('wheel', [xm, R, sd * zt], null, { name: sd > 0 ? '左后轮' : '右后轮', d: { r: R, t: tw } });
      J('revolute', m, w, [xm, R, sd * zt], [0, 0, sd], sd > 0 ? {} : { motorRev: true });
    }
    const fk = P('caster', [xk, yTop, 0], [0, 180, 0], { name: '转向叉', d: { R: 0.1, H: hc, top: 0.135, gw: 0.04 } });
    J('revolute', ch, fk, [xk, yTop, 0], [0, 1, 0], { kc: 0.6, damp: 0.08, steer: true });
    const fw = P('wheel', [xk - 0.1, R, 0], null, { name: '前轮', d: { r: R, t: tw } });
    J('revolute', fk, fw, [xk - 0.1, R, 0], Zx);
  } },
  { name: '四轮小车(通用零件装配)', app: true, desc: '只用板/滑块/轮 + 配合工具装配：滑动副+弹簧阻尼=悬架，铰链舵机=转向(A/D)，铰链电机=后驱(W/S)；左侧装好后镜像复制到右侧。运行后按 W/S/A/D 驾驶', build(app) { buildGenericCar(app); app.placePart('wall', { x: 0.9, y: 0, z: 0 }, { name: '减速带', d: { w: 0.04, h: 0.012, dp: 0.6 } }); } },
  { name: '投石机', desc: '配重下落带动长臂，把石块抛出（铰链+固定连接+碰撞）', build({ P, J, rod }) {
    const piv = [0, 0.75, 0], ang = -140 * D2R, dir = [Math.cos(ang), Math.sin(ang), 0], Ll = 0.7, Ls = 0.3;
    const longEnd = [piv[0] + dir[0] * Ll, piv[1] + dir[1] * Ll, 0], shortEnd = [piv[0] - dir[0] * Ls, piv[1] - dir[1] * Ls, 0];
    const fr = P('base', [0, 0.4, -0.045], null, { name: '支架', d: { w: 0.1, h: 0.8, dp: 0.06 } });
    const arm = rod(shortEnd, longEnd, { name: '抛臂', d: { L: Ll + Ls, w: 0.03, t: 0.03 }, mat: 'wood' });
    const cw = P('ball', [...shortEnd], null, { name: '配重', d: { r: 0.07 } });
    const nUp = [-dir[1] * -1 * 0 + Math.cos(ang + Math.PI / 2 + Math.PI), Math.sin(ang + Math.PI / 2 + Math.PI), 0]; // up-left normal
    const n2 = [Math.cos(ang - Math.PI / 2), Math.sin(ang - Math.PI / 2), 0];
    const up = n2[1] > 0 ? n2 : [-n2[0], -n2[1], 0];
    const cupC = [longEnd[0] + up[0] * 0.05, longEnd[1] + up[1] * 0.05, 0];
    const cup = P('plate', cupC, [0, 0, Math.atan2(up[1], up[0]) / D2R], { name: '抛斗挡板', d: { w: 0.1, h: 0.01, dp: 0.06 }, mat: 'wood' });
    const br = 0.03, bc = [longEnd[0] - dir[0] * (0.005 + br + 0.002) + up[0] * (0.015 + br + 0.002), longEnd[1] - dir[1] * (0.005 + br + 0.002) + up[1] * (0.015 + br + 0.002), 0];
    P('ball', bc, null, { name: '石块', d: { r: br }, mat: 'wood', trace: true, free: true });
    J('revolute', arm, fr, [piv[0], piv[1], -0.015], Zx); J('fixed', cw, arm, shortEnd, Zx); J('fixed', cup, arm, longEnd, Zx);
    void nUp;
  } },
];
