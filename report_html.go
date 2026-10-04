package trajectory

// reportHTML is the entire static report: markup, styles and vanilla
// JavaScript in one string. The only template substitution is the inlined
// scenario JSON. The file deliberately contains no backticks (Go raw string)
// and no external resource references.
const reportHTML = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>时间最优速度规划报告</title>
<style>
  :root {
    color-scheme: light;
    --page: #f9f9f7;
    --surface: #fcfcfb;
    --ink: #0b0b0b;
    --ink-2: #52514e;
    --muted: #898781;
    --grid: #e1e0d9;
    --axis: #c3c2b7;
    --border: rgba(11,11,11,0.10);
    --c-pos: #2a78d6;
    --c-vel: #eb6834;
    --c-acc: #1baf7a;
    --c-jerk: #4a3aa7;
    --c-critical: #d03b3b;
    --c-good: #0ca30c;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --page: #0d0d0d;
      --surface: #1a1a19;
      --ink: #ffffff;
      --ink-2: #c3c2b7;
      --muted: #898781;
      --grid: #2c2c2a;
      --axis: #383835;
      --border: rgba(255,255,255,0.10);
      --c-pos: #3987e5;
      --c-vel: #d95926;
      --c-acc: #199e70;
      --c-jerk: #9085e9;
      --c-critical: #e66767;
      --c-good: #0ca30c;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --page: #0d0d0d;
    --surface: #1a1a19;
    --ink: #ffffff;
    --ink-2: #c3c2b7;
    --muted: #898781;
    --grid: #2c2c2a;
    --axis: #383835;
    --border: rgba(255,255,255,0.10);
    --c-pos: #3987e5;
    --c-vel: #d95926;
    --c-acc: #199e70;
    --c-jerk: #9085e9;
    --c-critical: #e66767;
    --c-good: #0ca30c;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--page); color: var(--ink);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    font-size: 14px; line-height: 1.5;
  }
  header { padding: 20px 24px 8px; }
  h1 { font-size: 20px; margin: 0 0 4px; }
  .sub { color: var(--ink-2); margin: 0; }
  .wrap { padding: 12px 24px 40px; }
  .toolbar {
    display: flex; flex-wrap: wrap; gap: 10px; align-items: center;
    padding: 12px 0 16px;
  }
  select, button, input[type=range] {
    font: inherit; color: var(--ink);
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 10px;
  }
  button { cursor: pointer; min-width: 64px; }
  button:hover { border-color: var(--muted); }
  .spacer { flex: 1; }
  .layout {
    display: grid; gap: 16px;
    grid-template-columns: minmax(0, 1.5fr) minmax(320px, 1fr);
  }
  @media (max-width: 980px) { .layout { grid-template-columns: 1fr; } }
  .card {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 10px; padding: 14px 16px;
  }
  .card h2 { font-size: 14px; margin: 0 0 2px; font-weight: 600; }
  .card .hint { color: var(--muted); font-size: 12px; margin: 0 0 8px; }
  .chart-box { position: relative; margin-top: 6px; }
  canvas { display: block; width: 100%; }
  .tooltip {
    position: absolute; pointer-events: none; z-index: 5;
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 9px; font-size: 12px;
    box-shadow: 0 2px 10px rgba(0,0,0,0.12); display: none;
    white-space: nowrap;
  }
  .tooltip .row { display: flex; align-items: center; gap: 6px; }
  .dot { width: 8px; height: 8px; border-radius: 50%; flex: none; }
  .readout {
    display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px;
    margin: 10px 0 4px;
  }
  .stat { background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 8px 10px; }
  .stat .k { color: var(--muted); font-size: 11px; }
  .stat .v { font-weight: 600; font-size: 15px; }
  .controls { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  .controls input[type=range] { flex: 1; min-width: 160px; padding: 0; }
  .pill {
    display: inline-block; font-size: 12px; border-radius: 999px;
    padding: 2px 10px; border: 1px solid var(--border); color: var(--ink-2);
  }
  .pill.ok { color: var(--c-good); border-color: var(--c-good); }
  .pill.bad { color: var(--c-critical); border-color: var(--c-critical); }
  table { width: 100%; border-collapse: collapse; font-variant-numeric: tabular-nums; }
  th, td { text-align: right; padding: 5px 8px; border-bottom: 1px solid var(--grid); }
  th:first-child, td:first-child { text-align: left; }
  th { color: var(--muted); font-weight: 500; font-size: 12px; }
  .viol {
    border-left: 3px solid var(--c-critical); padding: 8px 10px;
    margin: 8px 0; background: color-mix(in srgb, var(--c-critical) 7%, transparent);
    border-radius: 0 6px 6px 0;
  }
  .viol b { color: var(--c-critical); }
  .note { color: var(--ink-2); font-size: 13px; margin: 6px 0 0; }
  .legend { display: flex; gap: 14px; flex-wrap: wrap; margin: 4px 0 2px; font-size: 12px; color: var(--ink-2); }
  .legend span { display: inline-flex; align-items: center; gap: 6px; }
  label.theme-toggle { color: var(--ink-2); font-size: 13px; display: inline-flex; gap:6px; align-items:center; }
</style>
</head>
<body>
<header>
  <h1>时间最优速度规划 — 约束耦合与分段连续性验证</h1>
  <p class="sub">梯形 / S 曲线速度规划 · 位置、速度、加速度曲线 · 沿几何路径逐帧回放 · 纯静态单文件（无框架、无第三方库）</p>
</header>
<div class="wrap">
  <div class="toolbar">
    <label for="sc">场景：</label>
    <select id="sc"></select>
    <span id="statusPill" class="pill"></span>
    <span class="spacer"></span>
    <label class="theme-toggle"><input type="checkbox" id="darkToggle"> 深色</label>
  </div>

  <div class="readout">
    <div class="stat"><div class="k">时间 t</div><div class="v" id="rT">0.000 s</div></div>
    <div class="stat"><div class="k">位置 s</div><div class="v" id="rS">0.000</div></div>
    <div class="stat"><div class="k">速度 v</div><div class="v" id="rV">0.000</div></div>
    <div class="stat"><div class="k">加速度 a</div><div class="v" id="rA">0.000</div></div>
  </div>

  <div class="layout">
    <div>
      <div class="card">
        <h2>位置 s(t) / 速度 v(t) / 加速度 a(t)</h2>
        <p class="hint">红色细线为约束限幅；红色三角为检测到的违例时刻。在任意图上移动鼠标可联动所有图与回放位置。</p>
        <div class="legend">
          <span><i class="dot" style="background:var(--c-pos)"></i>位置 s</span>
          <span><i class="dot" style="background:var(--c-vel)"></i>速度 v</span>
          <span><i class="dot" style="background:var(--c-acc)"></i>加速度 a</span>
          <span><i class="dot" style="background:var(--c-jerk)"></i>jerk（仅 S 曲线）</span>
          <span><i class="dot" style="background:var(--c-critical)"></i>约束限幅 / 违例</span>
        </div>
        <div class="chart-box"><canvas id="chS" height="150"></canvas><div class="tooltip" id="ttS"></div></div>
        <div class="chart-box"><canvas id="chV" height="150"></canvas><div class="tooltip" id="ttV"></div></div>
        <div class="chart-box"><canvas id="chA" height="150"></canvas><div class="tooltip" id="ttA"></div></div>
        <div class="chart-box" id="jerkWrap"><canvas id="chJ" height="120"></canvas><div class="tooltip" id="ttJ"></div></div>
      </div>
    </div>

    <div>
      <div class="card">
        <h2>沿路径回放</h2>
        <p class="hint" id="scNote"></p>
        <canvas id="pathCanvas" height="300"></canvas>
        <div class="controls" style="margin-top:10px">
          <button id="playBtn">播放</button>
          <select id="speedSel">
            <option value="0.25">0.25×</option>
            <option value="0.5">0.5×</option>
            <option value="1" selected>1×</option>
            <option value="2">2×</option>
            <option value="4">4×</option>
          </select>
          <input type="range" id="timeSlider" min="0" max="1000" value="0">
        </div>
        <p class="note" id="metaLine"></p>
      </div>

      <div class="card" style="margin-top:16px">
        <h2>分段（零长度退化段已跳过）</h2>
        <table id="segTable"><thead>
          <tr><th>阶段</th><th>t₀</th><th>t₁</th><th>时长</th><th>jerk</th></tr>
        </thead><tbody></tbody></table>
      </div>

      <div class="card" style="margin-top:16px">
        <h2>约束验证</h2>
        <div id="violBox"><p class="note">所有采样点均满足速度 / 加速度 / jerk 限幅与端点条件。</p></div>
      </div>
    </div>
  </div>
</div>

<script id="trajectory-data" type="application/json">{{.}}</script>
<script>
"use strict";
var DATA = JSON.parse(document.getElementById("trajectory-data").textContent);

/* ---------------- theme ---------------- */
var root = document.documentElement;
var darkMQ = window.matchMedia("(prefers-color-scheme: dark)");
function applyTheme() {
  var wantDark = document.getElementById("darkToggle").checked;
  root.setAttribute("data-theme", wantDark ? "dark" : "light");
}
document.getElementById("darkToggle").checked = darkMQ.matches;
document.getElementById("darkToggle").addEventListener("change", applyTheme);
darkMQ.addEventListener("change", function (e) {
  document.getElementById("darkToggle").checked = e.matches; applyTheme(); render();
});
function css(name) { return getComputedStyle(root).getPropertyValue(name).trim(); }

/* ---------------- math helpers ---------------- */
function niceStep(raw) {
  if (!isFinite(raw) || raw <= 0) return 1;
  var mag = Math.pow(10, Math.floor(Math.log(raw) / Math.LN10));
  var norm = raw / mag;
  var step;
  if (norm <= 1) step = 1; else if (norm <= 2) step = 2;
  else if (norm <= 5) step = 5; else step = 10;
  return step * mag;
}
function fmt(x, d) {
  if (!isFinite(x)) return "-";
  d = (d === undefined) ? 3 : d;
  if (Math.abs(x) >= 10000) return x.toExponential(2);
  return Number(x.toFixed(d)).toString();
}
function interp(samples, t) {
  var lo = 0, hi = samples.length - 1;
  if (t <= samples[0][0]) return samples[0];
  if (t >= samples[hi][0]) return samples[hi];
  while (hi - lo > 1) {
    var mid = (lo + hi) >> 1;
    if (samples[mid][0] <= t) lo = mid; else hi = mid;
  }
  var a = samples[lo], b = samples[hi], u = (t - a[0]) / (b[0] - a[0]);
  return [t, a[1] + (b[1]-a[1])*u, a[2] + (b[2]-a[2])*u, a[3] + (b[3]-a[3])*u];
}
function jerkAt(sc, t) {
  var segs = sc.segments;
  for (var i = 0; i < segs.length; i++) {
    if (t >= segs[i].t0 - 1e-12 && t < segs[i].t1 - 1e-9) return segs[i].jerk;
  }
  return segs.length ? segs[segs.length-1].jerk : 0;
}

/* ---------------- canvas chart ---------------- */
function Chart(canvas, tooltip, opts) {
  this.cv = canvas; this.ctx = canvas.getContext("2d"); this.tt = tooltip; this.opts = opts;
  var self = this;
  canvas.addEventListener("mousemove", function (e) {
    var r = canvas.getBoundingClientRect();
    var px = (e.clientX - r.left) * (canvas.width / r.width) ;
    var t = self.invX(px);
    if (t !== null && opts.onCursor) opts.onCursor(t, true);
    if (t !== null) self.showTip(e, t);
  });
  canvas.addEventListener("mouseleave", function () {
    self.tt.style.display = "none";
    if (opts.onCursor) opts.onCursor(current.t, false);
  });
}
Chart.prototype.invX = function (px) {
  if (!this.box) return null;
  var b = this.box;
  if (px < b.l || px > b.l + b.w) return null;
  var u = (px - b.l) / b.w;
  return this.x0 + u * (this.x1 - this.x0);
};
Chart.prototype.showTip = function (e, t) {
  var sc = current;
  var val;
  if (this.opts.yIndex === -99) val = jerkAt(sc, t);
  else val = interp(sc.samples, t)[this.opts.yIndex];
  var html = '<div class="row"><span class="dot" style="background:' + this.opts.color + '"></span>' +
    't = ' + fmt(t) + ' s · ' + this.opts.label + ' = <b>' + fmt(val) + '</b> ' + this.opts.unit + '</div>';
  if (this.opts.extra) html += this.opts.extra(t);
  this.tt.innerHTML = html;
  this.tt.style.display = "block";
  var box = this.cv.parentElement.getBoundingClientRect();
  var x = e.clientX - box.left + 12, y = e.clientY - box.top + 12;
  if (x + this.tt.offsetWidth > box.width) x = e.clientX - box.left - this.tt.offsetWidth - 12;
  this.tt.style.left = x + "px"; this.tt.style.top = y + "px";
};
Chart.prototype.draw = function (sc, cursorT) {
  var cv = this.cv, ctx = this.ctx;
  var dpr = window.devicePixelRatio || 1;
  var cssW = cv.clientWidth, cssH = cv.clientHeight || parseInt(cv.getAttribute("height"), 10);
  cv.width = Math.round(cssW * dpr); cv.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  var W = cssW, H = cssH;
  ctx.clearRect(0, 0, W, H);

  var ml = 56, mr = 14, mt = 10, mb = 22;
  var b = { l: ml, r: W - mr, t: mt, b: H - mb, w: W - ml - mr, h: H - mt - mb };
  this.box = b;

  var T = sc.duration;
  this.x0 = 0; this.x1 = T;
  var yLo = this.opts.yLo !== undefined ? this.opts.yLo(sc) : null;
  var yHi = this.opts.yHi !== undefined ? this.opts.yHi(sc) : null;
  var vals = sc.samples.map(function (q) { return q[this.opts.yIndex]; }, this);
  var dMin = Math.min.apply(null, vals), dMax = Math.max.apply(null, vals);
  if (yLo === null) yLo = Math.min(0, dMin);
  if (yHi === null) yHi = dMax;
  if (this.opts.limit) { yLo = Math.min(yLo, -this.opts.limit(sc)); yHi = Math.max(yHi, this.opts.limit(sc)); }
  if (yHi - yLo < 1e-12) { yHi += 1; yLo -= 1; }
  var pad = (yHi - yLo) * 0.08; yLo -= pad; yHi += pad;
  this.yLo = yLo; this.yHi = yHi;

  var X = function (t) { return b.l + (t / T) * b.w; };
  var Y = function (v) { return b.b - ((v - yLo) / (yHi - yLo)) * b.h; };

  // grid + ticks
  ctx.strokeStyle = css("--grid"); ctx.fillStyle = css("--muted");
  ctx.lineWidth = 1; ctx.font = "11px system-ui, sans-serif";
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  var yStep = niceStep((yHi - yLo) / 4);
  var yStart = Math.ceil(yLo / yStep) * yStep;
  for (var yv = yStart; yv <= yHi + yStep * 1e-9; yv += yStep) {
    var yy = Y(yv);
    ctx.beginPath(); ctx.moveTo(b.l, yy); ctx.lineTo(b.r, yy); ctx.stroke();
    ctx.fillText(fmt(yv, yStep < 1 ? 2 : 1), b.l - 7, yy);
  }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  var xStep = niceStep(T / 6);
  for (var xt = 0; xt <= T + xStep * 1e-9; xt += xStep) {
    var xx = X(Math.min(xt, T));
    ctx.strokeStyle = css("--grid");
    ctx.beginPath(); ctx.moveTo(xx, b.t); ctx.lineTo(xx, b.b); ctx.stroke();
    ctx.fillStyle = css("--muted");
    ctx.fillText(fmt(Math.min(xt, T)), xx, b.b + 5);
  }
  // axes
  ctx.strokeStyle = css("--axis");
  ctx.beginPath(); ctx.moveTo(b.l, b.t); ctx.lineTo(b.l, b.b); ctx.lineTo(b.r, b.b); ctx.stroke();
  // zero line emphasized
  if (yLo < 0 && yHi > 0) {
    ctx.strokeStyle = css("--axis");
    ctx.beginPath(); ctx.moveTo(b.l, Y(0)); ctx.lineTo(b.r, Y(0)); ctx.stroke();
  }

  // limit lines
  if (this.opts.limit) {
    var lim = this.opts.limit(sc);
    ctx.setLineDash([5, 4]); ctx.strokeStyle = css("--c-critical"); ctx.lineWidth = 1;
    [lim, -lim].forEach(function (lv) {
      if (lv > yLo && lv < yHi && (lv !== 0 || lim === 0)) {
        ctx.beginPath(); ctx.moveTo(b.l, Y(lv)); ctx.lineTo(b.r, Y(lv)); ctx.stroke();
      }
    });
    if (lim > yLo && lim < yHi) {
      ctx.setLineDash([]); ctx.fillStyle = css("--c-critical");
      ctx.textAlign = "left"; ctx.textBaseline = "bottom";
      ctx.fillText(this.opts.limitLabel + " " + fmt(lim), b.l + 4, Y(lim) - 1);
    }
    ctx.setLineDash([]);
  }

  // area wash + line
  var col = this.opts.color;
  ctx.beginPath();
  sc.samples.forEach(function (q, i) {
    var px = X(q[0]), py = Y(q[this.opts.yIndex]);
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  }, this);
  ctx.lineWidth = 2; ctx.strokeStyle = col; ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.stroke();

  // segment boundary guides
  ctx.strokeStyle = css("--grid");
  sc.segments.forEach(function (sg) {
    if (sg.t0 > 0 && sg.t0 < T) {
      ctx.beginPath(); ctx.moveTo(X(sg.t0), b.t); ctx.lineTo(X(sg.t0), b.b); ctx.stroke();
    }
  });

  // violation markers
  sc.violations.forEach(function (v) {
    if (!this.opts.violationFilter(v.type)) return;
    var vy;
    if (v.type === this.opts.violationType) vy = Y(Math.max(Math.min(v.value, yHi), yLo));
    else vy = b.t + 8;
    var vx = X(Math.max(0, Math.min(v.t, T)));
    ctx.fillStyle = css("--c-critical");
    ctx.beginPath();
    ctx.moveTo(vx, vy - 6); ctx.lineTo(vx + 5, vy + 3); ctx.lineTo(vx - 5, vy + 3);
    ctx.closePath(); ctx.fill();
  }, this);

  // cursor
  if (cursorT >= 0 && cursorT <= T) {
    var cx = X(cursorT);
    ctx.strokeStyle = css("--ink-2"); ctx.setLineDash([3, 3]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(cx, b.t); ctx.lineTo(cx, b.b); ctx.stroke();
    var q = interp(sc.samples, cursorT);
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(cx, Y(q[this.opts.yIndex]), 5, 0, 7);
    ctx.fillStyle = col; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = css("--surface"); ctx.stroke();
  }
  this.X = X; this.Y = Y;
};

/* ---------------- state ---------------- */
var current = null;
var cursor = { t: 0, active: false };
var playing = false, lastFrame = null;
var charts = [];

function buildCharts() {
  charts = [
    new Chart(document.getElementById("chS"), document.getElementById("ttS"), {
      yIndex: 1, label: "s", unit: "", color: css("--c-pos"),
      yLo: function () { return 0; },
      violationType: "none", violationFilter: function () { return false; },
      onCursor: setCursor
    }),
    new Chart(document.getElementById("chV"), document.getElementById("ttV"), {
      yIndex: 2, label: "v", unit: "", color: css("--c-vel"),
      yLo: function () { return 0; },
      limit: function (sc) { return sc.vMax; }, limitLabel: "限速",
      violationType: "velocity", violationFilter: function (ty) { return ty === "velocity"; },
      onCursor: setCursor
    }),
    new Chart(document.getElementById("chA"), document.getElementById("ttA"), {
      yIndex: 3, label: "a", unit: "", color: css("--c-acc"),
      limit: function (sc) { return sc.aMax; }, limitLabel: "限幅",
      violationType: "acceleration", violationFilter: function (ty) { return ty === "acceleration"; },
      onCursor: setCursor
    }),
    new Chart(document.getElementById("chJ"), document.getElementById("ttJ"), {
      yIndex: -99, label: "j", unit: "", color: css("--c-jerk"),
      limit: function (sc) { return sc.jMax || 0; }, limitLabel: "限幅",
      violationType: "jerk", violationFilter: function (ty) { return ty === "jerk"; },
      onCursor: setCursor
    })
  ];
}

/* jerk chart uses a step series built from segments */
var origDraw = Chart.prototype.draw;
function jerkSamples(sc) {
  if (!sc.jMax) return [[0, 0], [sc.duration, 0]];
  var out = [[0, sc.segments.length ? sc.segments[0].jerk : 0]];
  sc.segments.forEach(function (sg) {
    out.push([sg.t0, sg.jerk]);
    out.push([sg.t1 - 1e-9, sg.jerk]);
  });
  out.push([sc.duration, sc.segments.length ? sc.segments[sc.segments.length - 1].jerk : 0]);
  return out;
}

/* ---------------- render ---------------- */
function setCursor(t, active) {
  cursor.t = Math.max(0, Math.min(t, current.duration));
  cursor.active = active;
  if (!active) playing = false;
  syncReadout();
  drawAll();
  document.getElementById("timeSlider").value =
    Math.round((cursor.t / current.duration) * 1000);
}
function syncReadout() {
  var st = interp(current.samples, cursor.t);
  document.getElementById("rT").textContent = fmt(st[0]) + " s";
  document.getElementById("rS").textContent = fmt(st[1]);
  document.getElementById("rV").textContent = fmt(st[2]);
  document.getElementById("rA").textContent = fmt(st[3]);
}
function drawAll() {
  charts.forEach(function (ch) {
    if (ch.opts.yIndex === -99) {
      var saved = ch.opts.yIndex;
      var sc = current;
      // jerk step chart: temporarily swap sample source
      var origSamples = sc.samples;
      ch.opts._jerkMode = true;
      ch.opts.yIndex = 1;
      var fakeSc = Object.assign({}, sc, { samples: jerkSamples(sc) });
      ch.draw(fakeSc, cursor.t);
      ch.opts.yIndex = saved;
    } else {
      ch.draw(current, cursor.t);
    }
  });
  drawPath();
}
function render() {
  var scSel = document.getElementById("sc");
  current = DATA[scSel.selectedIndex];
  cursor.t = 0;
  document.getElementById("jerkWrap").style.display = current.jMax ? "block" : "none";
  document.getElementById("scNote").textContent = current.note || "";
  var pill = document.getElementById("statusPill");
  if (current.corrupted) {
    pill.className = "pill bad";
    pill.textContent = "检测到约束违例 ×" + current.violations.length;
  } else {
    pill.className = "pill ok";
    pill.textContent = "全部约束满足 ✓";
  }
  document.getElementById("metaLine").textContent =
    current.kind.toUpperCase() +
    (current.reach ? "（达到限速）" : "（未达限速·退化）") +
    " · 总时长 " + fmt(current.duration, 5) + " s" +
    " · 理论最优 " + fmt(current.theoryTime, 5) + " s" +
    " · 路径长 " + fmt(current.length) +
    " · vMax=" + fmt(current.vMax) + " aMax=" + fmt(current.aMax) +
    (current.jMax ? " jMax=" + fmt(current.jMax) : "");
  // segments
  var tb = document.querySelector("#segTable tbody");
  tb.innerHTML = "";
  current.segments.forEach(function (sg) {
    var tr = document.createElement("tr");
    tr.innerHTML = "<td>" + sg.name + "</td><td>" + fmt(sg.t0, 4) + "</td><td>" +
      fmt(sg.t1, 4) + "</td><td>" + fmt(sg.duration, 4) + "</td><td>" +
      (current.jMax ? fmt(sg.jerk) : "—") + "</td>";
    tr.style.cursor = "pointer";
    tr.addEventListener("click", function () { setCursor(sg.t0, true); });
    tb.appendChild(tr);
  });
  // violations
  var vb = document.getElementById("violBox");
  if (!current.violations.length) {
    vb.innerHTML = '<p class="note">所有采样点均满足速度 / 加速度 / ' +
      (current.jMax ? "jerk " : "") + '限幅与端点条件（起点 s=0、终点 s=L，端点速度匹配）。</p>';
  } else {
    vb.innerHTML = "";
    current.violations.forEach(function (v) {
      var d = document.createElement("div");
      d.className = "viol";
      var names = { velocity: "速度", acceleration: "加速度", jerk: "jerk", endpoint: "端点条件" };
      d.innerHTML = "<b>" + (names[v.type] || v.type) + "违例</b>：时间段 t∈[" +
        fmt(v.t0, 4) + ", " + fmt(v.t1, 4) + "] s，峰值时刻 t=" + fmt(v.t, 5) +
        " s，观测值 " + fmt(v.value, 5) + "，限幅 " + fmt(v.limit, 5);
      d.style.cursor = "pointer";
      d.addEventListener("click", function () { setCursor(v.t, true); });
      vb.appendChild(d);
    });
  }
  document.getElementById("timeSlider").value = 0;
  syncReadout();
  drawAll();
}

/* ---------------- path playback ---------------- */
var pathCv = document.getElementById("pathCanvas");
function drawPath() {
  var cv = pathCv, ctx = cv.getContext("2d");
  var dpr = window.devicePixelRatio || 1;
  var cssW = cv.clientWidth || 600, cssH = 300;
  cv.width = Math.round(cssW * dpr); cv.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);
  var pts = current.path;
  if (!pts || pts.length < 2) {
    ctx.fillStyle = css("--muted"); ctx.font = "12px system-ui";
    ctx.fillText("(无路径几何)", 12, 20);
    return;
  }
  var xs = pts.map(function (p) { return p[0]; }), ys = pts.map(function (p) { return p[1]; });
  var x0 = Math.min.apply(null, xs), x1 = Math.max.apply(null, xs);
  var y0 = Math.min.apply(null, ys), y1 = Math.max.apply(null, ys);
  var pad = 28;
  var sx = (cssW - 2 * pad) / Math.max(1e-9, x1 - x0);
  var sy = (cssH - 2 * pad) / Math.max(1e-9, y1 - y0);
  var sc = Math.min(sx, sy);
  var ox = (cssW - (x1 - x0) * sc) / 2 - x0 * sc;
  var oy = (cssH - (y1 - y0) * sc) / 2 + y1 * sc;
  var P = function (p) { return [p[0] * sc + ox, -p[1] * sc + oy]; };

  function polyPath() {
    ctx.beginPath();
    pts.forEach(function (p, i) {
      var q = P(p);
      if (i === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]);
    });
  }
  // full path
  polyPath();
  ctx.strokeStyle = css("--grid"); ctx.lineWidth = 6; ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.stroke();
  // traveled part: find how many vertices are within current s fraction
  var frac = 0;
  var stNow = interp(current.samples, cursor.t);
  frac = current.length > 0 ? stNow[1] / current.length : 0;
  frac = Math.max(0, Math.min(1, frac));
  // total polyline length equals scenario length (sampled polyline)
  var target = frac * current.length, acc = 0, cut = null;
  ctx.beginPath();
  var started = false;
  for (var i = 0; i < pts.length - 1; i++) {
    var a = pts[i], b = pts[i+1];
    var seg = Math.hypot(b[0]-a[0], b[1]-a[1]);
    var qa = P(a);
    if (acc + seg >= target - 1e-12) {
      var u = seg > 0 ? (target - acc) / seg : 0;
      var qx = a[0] + (b[0]-a[0])*u, qy = a[1] + (b[1]-a[1])*u;
      var qc = P([qx, qy]);
      if (!started) { ctx.moveTo(qa[0], qa[1]); started = true; }
      ctx.lineTo(qc[0], qc[1]);
      cut = qc;
      break;
    }
    if (!started) { ctx.moveTo(qa[0], qa[1]); started = true; }
    var qb = P(b);
    ctx.lineTo(qb[0], qb[1]);
    acc += seg;
  }
  if (cut === null) cut = P(pts[pts.length - 1]);
  ctx.strokeStyle = css("--c-pos"); ctx.lineWidth = 6; ctx.lineJoin = "round"; ctx.lineCap = "round";
  ctx.stroke();

  // start / end markers
  var ps = P(pts[0]), pe = P(pts[pts.length-1]);
  ctx.fillStyle = css("--ink-2");
  ctx.beginPath(); ctx.arc(ps[0], ps[1], 4, 0, 7); ctx.fill();
  ctx.strokeStyle = css("--ink-2"); ctx.lineWidth = 2;
  ctx.strokeRect(pe[0]-4, pe[1]-4, 8, 8);

  // moving marker
  ctx.fillStyle = css("--c-vel");
  ctx.beginPath(); ctx.arc(cut[0], cut[1], 7, 0, 7); ctx.fill();
  ctx.lineWidth = 2; ctx.strokeStyle = css("--surface"); ctx.stroke();
  ctx.fillStyle = css("--ink-2"); ctx.font = "11px system-ui"; ctx.textAlign = "left";
  ctx.fillText("起点", ps[0] + 8, ps[1] - 6);
  ctx.fillText("终点", pe[0] + 8, pe[1] - 6);
}

/* ---------------- playback loop ---------------- */
function frame(now) {
  if (!playing) { lastFrame = null; return; }
  if (lastFrame === null) lastFrame = now;
  var dt = (now - lastFrame) / 1000; lastFrame = now;
  var speed = parseFloat(document.getElementById("speedSel").value);
  cursor.t += dt * speed;
  if (cursor.t >= current.duration) {
    cursor.t = current.duration;
    playing = false;
    document.getElementById("playBtn").textContent = "播放";
  }
  document.getElementById("timeSlider").value = Math.round((cursor.t / current.duration) * 1000);
  syncReadout(); drawAll();
  if (playing) requestAnimationFrame(frame);
}
document.getElementById("playBtn").addEventListener("click", function () {
  if (cursor.t >= current.duration - 1e-9) cursor.t = 0;
  playing = !playing;
  this.textContent = playing ? "暂停" : "播放";
  if (playing) requestAnimationFrame(frame);
});
document.getElementById("timeSlider").addEventListener("input", function () {
  playing = false;
  document.getElementById("playBtn").textContent = "播放";
  cursor.t = (parseInt(this.value, 10) / 1000) * current.duration;
  syncReadout(); drawAll();
});

/* ---------------- bootstrap ---------------- */
(function init() {
  var sel = document.getElementById("sc");
  DATA.forEach(function (sc, i) {
    var o = document.createElement("option");
    o.textContent = sc.name;
    sel.appendChild(o);
  });
  // Optional deep links: report.html#sc=3&theme=light
  var params = {};
  location.hash.replace(/^#/, "").split("&").forEach(function (kv) {
    var p = kv.split("="); if (p[0]) params[p[0]] = decodeURIComponent(p[1] || "");
  });
  if (params.theme === "light" || params.theme === "dark") {
    document.getElementById("darkToggle").checked = params.theme === "dark";
  }
  if (params.sc !== undefined) {
    var idx = parseInt(params.sc, 10);
    if (idx >= 0 && idx < DATA.length) sel.selectedIndex = idx;
  }
  applyTheme();
  sel.addEventListener("change", render);
  buildCharts();
  render();
  if (params.t !== undefined) {
    setCursor(parseFloat(params.t) * DATA[sel.selectedIndex].duration, true);
  }
  window.addEventListener("resize", drawAll);
})();
</script>
</body>
</html>
`
