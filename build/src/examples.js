// Example scenes assembled ONLY with the same editor API a user drives through the UI
// (place generic parts, 🧲 配合, joint editor, group + mirror copy). No car-specific part types.
const D2R = Math.PI / 180;
/** four-wheel car: plate chassis, block suspension sliders (prismatic + spring/damper), block steering knuckles (revolute servo A/D), rubber wheels (rear: speed drive W/S) */
export function buildGenericCar(A, o = {}) {
  const W = o.W || 0.5, Dp = o.Dp || 0.24, H = 0.02, R = o.R || 0.06, T = 0.03, b = 0.03, xf = W / 2 - 0.06, xr = -W / 2 + 0.06;
  const ch = A.placePart('plate', { x: 0, y: 0, z: 0 }, { name: '车架', d: { w: W, h: H, dp: Dp, np: 3 }, mat: 'alu' });
  const blk = (name, h) => A.placePart('block', { x: 0, y: 0, z: 0.6 }, { name, d: { w: b, h, dp: b }, mat: 'steel' });
  const face = (P, n) => { const d = P.d, hz = (d.dp || d.t || 0) / 2; return { pid: P.id, p: [0, 0, n * (P.type === 'wheel' ? d.t / 2 : hz)], n: [0, 0, n] }; };
  const spring = j => A.editJoint(j.id, jj => { jj.kc = o.k || 1500; jj.damp = o.c || 60; jj.lim = { on: true, lo: -0.025, hi: 0.025 }; }, '悬架弹簧');
  // ---- front-left corner: slider block -> knuckle block -> wheel
  const s1 = blk('左前悬架块', 0.05);
  const jS1 = A.mate(face(s1, -1), { pid: ch.id, p: [xf, 0, Dp / 2], n: [0, 0, 1] }, { type: 'prismatic', axis: 'Y' }); spring(jS1);
  const k1 = blk('左前转向节', b);
  const jK1 = A.mate(face(k1, -1), face(s1, 1), { type: 'revolute', axis: 'Y' });
  A.editJoint(jK1.id, jj => { jj.lim = { on: true, lo: -30 * D2R, hi: 30 * D2R }; jj.drive = { on: true, mode: 'servo', pos: 22 * D2R, maxF: 30, keyP: 'KeyA', keyN: 'KeyD' }; }, '转向舵机');
  const w1 = A.placePart('wheel', { x: 0, y: 0, z: 0.9 }, { name: '左前轮', d: { r: R, t: T } });
  A.mate(face(w1, -1), face(k1, 1), { type: 'revolute' });
  // group the corner and mirror it to the right side (joints to the chassis are mirrored too)
  A.msel = new Set([s1.id, k1.id, w1.id]); A.groupSel([s1.id, k1.id, w1.id]);
  const fr = A.cloneSet([s1.id, k1.id, w1.id], 'mirror', { axis: 2, at: 0, ext: true });
  // a mirrored steering servo turns the mirrored way: flip its axis so both knuckles steer the same way
  const kR = fr[1], jKR = A.M.joints.find(j => (j.a === kR || j.b === kR) && j.type === 'revolute' && j.drive && j.drive.mode === 'servo'); if (jKR) A.flipJoint(jKR.id);
  // ---- rear-left corner: slider block -> driven wheel
  const s2 = blk('左后悬架块', 0.05);
  const jS2 = A.mate(face(s2, -1), { pid: ch.id, p: [xr, 0, Dp / 2], n: [0, 0, 1] }, { type: 'prismatic', axis: 'Y' }); spring(jS2);
  const w2 = A.placePart('wheel', { x: 0, y: 0, z: 0.9 }, { name: '左后轮', d: { r: R, t: T } });
  const jW2 = A.mate(face(w2, -1), face(s2, 1), { type: 'revolute' });
  // wheel joint axis = +Z: forward (+X) rolling is negative rotation
  A.editJoint(jW2.id, jj => { jj.drive = { on: true, mode: 'speed', v: -(o.v || 12), maxF: o.maxT || 4, keyP: 'KeyW', keyN: 'KeyS' }; }, '驱动电机');
  A.groupSel([s2.id, w2.id]);
  A.cloneSet([s2.id, w2.id], 'mirror', { axis: 2, at: 0, ext: true });
  A.liftComp(ch.id); A.commit(); A.rebuild();
  return ch;
}
