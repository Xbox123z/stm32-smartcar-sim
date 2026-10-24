// 用 JS 复刻 car-sim-core 的两个核心算法，验证「tick yield + 差速循迹」闭环真的成立。
// 本机缺 MSVC 链接器无法 cargo build，所以用它做算法等价性验证。

// ---------- 物理（对应 physics.rs） ----------
const CFG = { wheelBase: 0.16, motorTau: 0.08, maxWheel: 1.6, maxAccel: 3.0, drag: 0.35, offGrip: 0.45, sensorSpacing: 0.026, sensorOffset: 0.12 };
// 巡线判据用的是「那根 3cm 的引导线」，不是整个赛道宽度
const LINE_HALF = 0.015;

function makeTrack(n = 120, rx = 3, ry = 4.5, width = 0.55) {
  const pts = [];
  for (let i = 0; i < n; i++) { const t = i / n * Math.PI * 2; pts.push({ x: rx * Math.sin(t), y: ry * (1 - Math.cos(t)) }); }
  return { pts, width };
}
function nearest(tr, x, y) {
  let bestD = Infinity, bestS = 0, acc = 0;
  for (let i = 0; i < tr.pts.length - 1; i++) {
    const a = tr.pts[i], b = tr.pts[i + 1];
    const vx = b.x - a.x, vy = b.y - a.y, L2 = vx * vx + vy * vy;
    let t = L2 < 1e-12 ? 0 : ((x - a.x) * vx + (y - a.y) * vy) / L2;
    t = Math.max(0, Math.min(1, t));
    const px = a.x + t * vx, py = a.y + t * vy;
    const d = Math.hypot(x - px, y - py);
    if (d < bestD) { bestD = d; bestS = acc + t * Math.sqrt(L2); }
    acc += Math.sqrt(L2);
  }
  return { d: bestD, s: bestS };
}
function isOff(tr, x, y) { return nearest(tr, x, y).d > tr.width / 2; }
function onLine(tr, x, y) { return nearest(tr, x, y).d <= LINE_HALF; }
function trackLen(tr) { return nearest(tr, tr.pts[tr.pts.length - 1].x, tr.pts[tr.pts.length - 1].y).s; }

function readLinePos(tr, car, lastPos) {
  const sin = Math.sin(car.yaw), cos = Math.cos(car.yaw);
  let acc = 0, sum = 0;
  for (let i = 0; i < 5; i++) {
    const lat = (i - 2) * CFG.sensorSpacing;
    const fx = car.x + sin * CFG.sensorOffset + cos * lat;
    const fy = car.y + cos * CFG.sensorOffset - sin * lat;
    const on = onLine(tr, fx, fy);
    acc += (i - 2) * (on ? 1 : 0); sum += on ? 1 : 0;
  }
  // 完全丢线时沿用上次方向（对应 World::last_position）
  return sum === 0 ? lastPos : (acc / sum) / 2;
}

function makeWorld(tr) {
  const start = tr.pts[0];
  const nxt = tr.pts[1];
  return { track: tr, x: start.x, y: start.y, yaw: Math.atan2(nxt.x - start.x, nxt.y - start.y), v: 0, wl: 0, wr: 0, omega: 0, t: 0, dist: 0, off: 0, wasOff: false, startS: nearest(tr, start.x, start.y).s, maxProg: 0 };
}
function step(w, pwmL, pwmR, dt) {
  const tL = (pwmL / 1000) * CFG.maxWheel, tR = (pwmR / 1000) * CFG.maxWheel;
  const al = 1 - Math.exp(-dt / CFG.motorTau);
  w.wl += (tL - w.wl) * al; w.wr += (tR - w.wr) * al;
  const vLin = (w.wl + w.wr) / 2;
  const omega = Math.max(-3, Math.min(3, (w.wl - w.wr) / CFG.wheelBase * 0.5));
  const off = isOff(w.track, w.x, w.y);
  const grip = off ? CFG.offGrip : 1;
  let accel = Math.max(-CFG.maxAccel, Math.min(CFG.maxAccel, (vLin - w.v) / dt));
  let v = w.v + accel * dt * grip;
  v -= v * CFG.drag * dt;
  if (off) v -= v * 1.2 * dt;
  w.v = v; w.omega = omega * grip;
  w.yaw += w.omega * dt;
  const dx = w.v * Math.sin(w.yaw) * dt, dy = w.v * Math.cos(w.yaw) * dt;
  w.x += dx; w.y += dy; w.dist += Math.hypot(dx, dy);
  if (off && !w.wasOff) w.off++;
  w.wasOff = off;
  const s = nearest(w.track, w.x, w.y).s;
  const total = trackLen(w.track);
  let p = (s - w.startS) / total;
  while (p < -0.5) p += 1; while (p > 0.5) p -= 1;
  if (p < 0) p += 1;
  if (p > w.maxProg) w.maxProg = p;
  w.t += dt;
}

// ---------- 学生代码（对应 good_line_follower.c） ----------
const Kp = 300, Kd = 90, BASE = 500, MAXT = 350;
let lastPos = 0;
function control(w) {           // 一次 loop 体
  const pos = readLinePos(w.track, w, lastPos) * 2000;
  let turn = Kp * (pos / 2000) + Kd * ((pos - lastPos) / 2000);
  lastPos = pos;
  if (turn > MAXT) turn = MAXT; if (turn < -MAXT) turn = -MAXT;
  return [Math.round(BASE + turn), Math.round(BASE - turn)];
}

// ---------- 驱动器（对应 sim.rs 的 tick yield 循环） ----------
function run(limitS = 60, dt = 0.01, delayMs = 10) {
  const tr = makeTrack();
  const w = makeWorld(tr);
  const stepsPerTick = Math.round((delayMs / 1000) / dt);
  let steps = 0;
  const maxSteps = limitS / dt;
  let tele = [];
  while (steps < maxSteps) {
    const [l, r] = control(w);          // 执行到 HAL_Delay
    for (let i = 0; i < stepsPerTick && steps < maxSteps; i++) { step(w, l, r, dt); steps++; }
    if (steps % 10 === 0) tele.push({ t: +w.t.toFixed(3), prog: +w.maxProg.toFixed(4) });
    if (w.maxProg >= 0.995) return { result: 'finished', t: w.t, prog: w.maxProg, off: w.off, dist: w.dist, steps, tele: tele.length };
    if (w.off > 40) return { result: 'failed', reason: 'off track too much', off: w.off, prog: w.maxProg, steps };
  }
  return { result: 'timeout', prog: w.maxProg, off: w.off, dist: w.dist, steps };
}

// ---------- 断言 ----------
let pass = 0, fail = 0;
function ok(name, cond, extra = '') { if (cond) { pass++; console.log('  PASS ' + name); } else { fail++; console.log('  FAIL ' + name + ' ' + extra); } }

console.log('算法等价性验证（JS 复刻 car-sim-core）\n');

const r = run();
console.log('  结果:', JSON.stringify({ result: r.result, t: r.t && +r.t.toFixed(2), prog: +(r.prog * 100).toFixed(1) + '%', off: r.off, dist: r.dist && +r.dist.toFixed(2), steps: r.steps }), '\n');

ok('PD 循迹能跑完 oval 赛道', r.result === 'finished', JSON.stringify(r));
ok('进度 ≥ 99.5%', r.prog >= 0.995);
ok('行驶距离 ≈ 赛道周长', r.dist > 15 && r.dist < 40, 'dist=' + r.dist);
ok('出界次数很少 (<10)', r.off < 10, 'off=' + r.off);
ok('用时合理 (<60s)', r.t < 60, 't=' + r.t);
ok('遥测被采样', r.tele > 10, 'tele=' + r.tele);

// 不给 PWM -> 不动
{
  const tr = makeTrack(); const w = makeWorld(tr);
  for (let i = 0; i < 500; i++) step(w, 0, 0, 0.01);
  ok('无 PWM 小车不动', w.dist < 1e-6, 'dist=' + w.dist);
}
// 差速 -> 转向
{
  const tr = makeTrack(); const w = makeWorld(tr);
  for (let i = 0; i < 100; i++) step(w, 200, 800, 0.01);
  ok('左右轮速差导致转向', Math.abs(w.omega) > 0.01, 'omega=' + w.omega);
}
// 出界检测
{
  const tr = makeTrack(); const w = makeWorld(tr);
  // 横向偏移（垂直于航向）超过半宽才算真正出界
  const sin = Math.sin(w.yaw), cos = Math.cos(w.yaw);
  w.x += cos * 0.9; w.y -= sin * 0.9;
  step(w, 0, 0, 0.01);
  ok('横向偏出半宽判定出界', w.off === 1, 'off=' + w.off);
}
// 中心线上中间的传感器压到线
{
  const tr = makeTrack(); const w = makeWorld(tr);
  ok('起点处加权位置≈0', Math.abs(readLinePos(tr, w, 0)) < 0.3, String(readLinePos(tr, w, 0)));
  // 偏离线后应能读出非零误差（这是修复前失效的关键点）
  const sin0 = Math.sin(w.yaw), cos0 = Math.cos(w.yaw);
  const car = { x: w.x + cos0 * 0.02, y: w.y - sin0 * 0.02, yaw: w.yaw };
  const p = readLinePos(tr, car, 0);
  ok('偏离引导线时能读出非零误差', Math.abs(p) > 0.01, 'pos=' + p);
}
// 忙等：无 delay 时控制周期退化为 dt（证明 HAL_Delay 是推进物理的唯一途径）
{
  const tr = makeTrack(); const w = makeWorld(tr);
  const [l, r] = control(w);
  for (let i = 0; i < 500; i++) step(w, l, r, 0.01);   // 只跑 5 秒物理
  ok('5 秒物理无法跑完（说明需要反复 yield）', w.maxProg < 0.995, 'prog=' + w.maxProg);
}
// 确定性
{
  lastPos = 0; const a = run(20, 0.01, 10);
  lastPos = 0; const b = run(20, 0.01, 10);
  ok('同参数结果可复现', JSON.stringify(a) === JSON.stringify(b));
}

console.log('\n' + pass + ' passed, ' + fail + ' failed');
process.exit(fail ? 1 : 0);
