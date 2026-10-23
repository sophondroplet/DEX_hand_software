"""Self-contained HTML page for a motion trace: setpoint vs actual position and load per joint."""

from __future__ import annotations

import json
from pathlib import Path


def write_trace_html(payload: dict, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(payload, ensure_ascii=False).replace("</", "<\\/")
    path.write_text(_TEMPLATE.replace("__DATA__", data), encoding="utf-8")
    return path


_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Motion Trace</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --text-primary: #0b0b0b; --text-secondary: #52514e;
  --text-muted: #898781; --grid: #e1e0d9; --axis: #c3c2b7; --border: rgba(11,11,11,0.10);
  --series-1: #2a78d6; --series-2: #eb6834; --warning: #fab219;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --text-primary: #ffffff; --text-secondary: #c3c2b7;
    --text-muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
    --series-1: #3987e5; --series-2: #d95926;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --text-primary: #ffffff; --text-secondary: #c3c2b7;
  --text-muted: #898781; --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --series-1: #3987e5; --series-2: #d95926;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--text-primary);
  font: 14px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1120px; margin: 0 auto; padding: 24px 16px 48px; }
h1 { font-size: 22px; margin: 0 0 4px; }
h2 { font-size: 16px; margin: 0; }
.meta { color: var(--text-secondary); margin: 0 0 16px; }
.card { background: var(--surface); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin-bottom: 16px; }
.table-wrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; }
th, td { padding: 6px 10px; text-align: right; border-bottom: 1px solid var(--grid); white-space: nowrap; }
th { color: var(--text-secondary); font-weight: 600; }
th:nth-child(-n+2), td:nth-child(-n+2) { text-align: left; }
.flag::before { content: "▲ "; color: var(--warning); font-size: 10px; }
.legend { display: flex; gap: 16px; color: var(--text-secondary); margin: 0 0 12px; }
.legend span { display: inline-flex; align-items: center; gap: 6px; }
.legend i { width: 14px; height: 2px; border-radius: 1px; display: inline-block; }
.head { display: flex; flex-wrap: wrap; gap: 4px 12px; align-items: baseline; margin-bottom: 8px; }
.head p { margin: 0; color: var(--text-secondary); font-variant-numeric: tabular-nums; }
.charts { display: grid; grid-template-columns: 2fr 1fr; gap: 16px; position: relative; }
.charts > div { min-width: 0; }
@media (max-width: 720px) { .charts { grid-template-columns: 1fr; } }
.chart-title { color: var(--text-secondary); font-size: 12px; margin: 0 0 4px; }
.chart { width: 100%; height: 190px; display: block; outline: none; }
.chart text { fill: var(--text-muted); font-size: 11px; font-variant-numeric: tabular-nums; }
.chart text.label { fill: var(--text-secondary); }
.tooltip { position: absolute; pointer-events: none; background: var(--surface); border: 1px solid var(--border);
  border-radius: 6px; padding: 6px 10px; font-variant-numeric: tabular-nums; display: none; z-index: 2;
  box-shadow: 0 4px 12px rgba(0,0,0,.15); }
.tooltip .row { display: flex; align-items: center; gap: 6px; }
.tooltip i { width: 10px; height: 2px; border-radius: 1px; display: inline-block; }
.tooltip strong { color: var(--text-primary); }
.tooltip span { color: var(--text-secondary); }
footer { color: var(--text-muted); font-size: 12px; }
</style>
</head>
<body>
<main>
  <h1 id="title">Motion trace</h1>
  <p class="meta" id="meta"></p>
  <section class="card table-wrap">
    <table>
      <thead><tr><th>ID</th><th>Joint</th><th>Start</th><th>Goal</th><th>Arrived at</th><th>Final error</th>
        <th>Max tracking error</th><th>Max goal step</th><th>Stalled cycles</th><th>Max load</th></tr></thead>
      <tbody id="summary"></tbody>
    </table>
  </section>
  <div class="legend"><span><i style="background:var(--series-1)"></i>setpoint sent</span>
    <span><i style="background:var(--series-2)"></i>actual position</span></div>
  <div id="joints"></div>
  <footer id="footer"></footer>
</main>
<script type="application/json" id="trace-data">__DATA__</script>
<script>
(() => {
  const data = JSON.parse(document.getElementById("trace-data").textContent);
  const NS = "http://www.w3.org/2000/svg";
  const el = (tag, attrs = {}, text) => { const n = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v); if (text !== undefined) n.textContent = text; return n; };
  const svgEl = (tag, attrs = {}) => { const n = document.createElementNS(NS, tag);
    for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v); return n; };
  const fmt = (v, d = 0) => (v === null || v === undefined ? "-" : Number(v).toFixed(d));

  document.getElementById("title").textContent = `Motion trace · ${data.label}` + (data.stopped ? " (stopped)" : "");
  document.getElementById("meta").textContent =
    `${data.started_at} · ${data.port} · loop ${data.cycle_hz} Hz · speed register ${data.stream_speed_mode || "fixed"} · ` +
    `planned ${fmt(data.plan_duration_s, 2)} s · ` +
    `arrival = first sample after which the joint stays within ±${data.tolerance} steps of its goal`;
  document.getElementById("footer").textContent =
    `Position and goal register are read at the start of each cycle, so they trail the setpoint sent in the same row ` +
    `by one cycle. Max goal step = largest change of the sent goal between two cycles (stair height); stalled cycles = ` +
    `cycles where the goal advanced by >= 1 step but the measured position did not change. ` +
    `Per-sample data: ${data.csv_name}. Positions are in each joint's own servo steps ` +
    `(SCS0009 ~0.29 deg, HLS3606 ~0.09 deg; arrival tolerance scaled to the same angle). ` +
    `Load is the drive duty in % of maximum.`;

  const summary = document.getElementById("summary");
  const arrivals = data.joints.map((j) => j.summary.arrival_s).filter((v) => v !== null);
  const latest = arrivals.length ? Math.max(...arrivals) : null;
  for (const j of data.joints) {
    const s = j.summary, tr = el("tr");
    const cells = [String(j.id), j.name, fmt(s.start), fmt(s.goal), s.arrival_s === null ? "not reached" : `${fmt(s.arrival_s, 2)} s`,
      fmt(s.final_error), fmt(s.max_tracking_error), fmt(s.max_goal_step, 1),
      s.stalled_pct === null || s.stalled_pct === undefined ? "-" : `${fmt(s.stalled_pct, 0)} %`,
      s.max_load_pct === null ? "-" : `${fmt(s.max_load_pct, 1)} %`];
    cells.forEach((text, i) => {
      const td = el("td");
      const late = i === 4 && (s.arrival_s === null || (latest !== null && arrivals.length > 1 && s.arrival_s === latest));
      td.appendChild(late ? el("span", { class: "flag" }, text) : document.createTextNode(text));
      tr.appendChild(td);
    });
    summary.appendChild(tr);
  }

  function niceStep(span, n) { const raw = Math.max(span, 1e-9) / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    for (const f of [1, 2, 5, 10]) if (raw <= f * mag) return f * mag; return 10 * mag; }
  function extent(series, minPad) { let lo = Infinity, hi = -Infinity;
    for (const s of series) for (const v of s) if (v !== null) { if (v < lo) lo = v; if (v > hi) hi = v; }
    if (lo === Infinity) return { lo: 0, hi: 1, step: 0.5 };
    if (hi - lo < minPad) { const m = (hi + lo) / 2; lo = m - minPad / 2; hi = m + minPad / 2; }
    const step = niceStep(hi - lo, 4); return { lo: Math.floor(lo / step) * step, hi: Math.ceil(hi / step) * step, step }; }
  function nearest(t, x) { let lo = 0, hi = t.length - 1; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (t[m] < x) lo = m; else hi = m; }
    return Math.abs(t[lo] - x) <= Math.abs(t[hi] - x) ? lo : hi; }

  function chart(svg, t, series, opts) {
    while (svg.firstChild) svg.removeChild(svg.firstChild);
    const W = svg.clientWidth || 600, H = svg.clientHeight || 190, m = { l: 44, r: 12, t: 14, b: 24 };
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    const tMax = t.length ? Math.max(t[t.length - 1], 1e-6) : 1;
    const yd = extent(series.map((s) => s.values), opts.minPad);
    const xOf = (v) => m.l + (v / tMax) * (W - m.l - m.r);
    const yOf = (v) => H - m.b - ((v - yd.lo) / (yd.hi - yd.lo || 1)) * (H - m.t - m.b);
    for (let v = yd.lo; v <= yd.hi + yd.step / 2; v += yd.step) {
      svg.appendChild(svgEl("line", { x1: m.l, x2: W - m.r, y1: yOf(v), y2: yOf(v), stroke: v === yd.lo ? "var(--axis)" : "var(--grid)" }));
      const lab = svgEl("text", { x: m.l - 6, y: yOf(v) + 4, "text-anchor": "end" }); lab.textContent = Number(v.toFixed(opts.digits)); svg.appendChild(lab);
    }
    const xs = niceStep(tMax, 5);
    for (let v = 0; v <= tMax + 1e-9; v += xs) { const lab = svgEl("text", { x: xOf(v), y: H - 6, "text-anchor": "middle" });
      lab.textContent = `${Number(v.toFixed(2))} s`; svg.appendChild(lab); }
    for (const s of series) {
      let d = "", pen = false;
      s.values.forEach((v, i) => { if (v === null) { pen = false; return; } d += `${pen ? "L" : "M"}${xOf(t[i]).toFixed(1)},${yOf(v).toFixed(1)}`; pen = true; });
      svg.appendChild(svgEl("path", { d, fill: "none", stroke: s.color, "stroke-width": 2, "stroke-linejoin": "round", "stroke-linecap": "round" }));
    }
    if (opts.arrival !== null && opts.arrival !== undefined) {
      const i = nearest(t, opts.arrival), v = series[1].values[i];
      if (v !== null) {
        svg.appendChild(svgEl("circle", { cx: xOf(t[i]), cy: yOf(v), r: 4, fill: series[1].color, stroke: "var(--surface)", "stroke-width": 2 }));
        const lab = svgEl("text", { x: xOf(t[i]), y: Math.max(m.t + 8, yOf(v) - 10), "text-anchor": xOf(t[i]) > W - 90 ? "end" : "middle", class: "label" });
        lab.textContent = `arrived ${opts.arrival.toFixed(2)} s`; svg.appendChild(lab);
      }
    }
    const cross = svgEl("line", { y1: m.t, y2: H - m.b, stroke: "var(--axis)", visibility: "hidden" });
    svg.appendChild(cross);
    return { indexAt(cx) { const b = svg.getBoundingClientRect(); const px = ((cx - b.left) / b.width) * W;
        return nearest(t, ((px - m.l) / (W - m.l - m.r)) * tMax); },
      show(i) { const x = xOf(t[i]); cross.setAttribute("x1", x); cross.setAttribute("x2", x); cross.setAttribute("visibility", "visible"); return x / W; },
      hide() { cross.setAttribute("visibility", "hidden"); } };
  }

  const rows = [], host = document.getElementById("joints");
  for (const j of data.joints) {
    const card = el("section", { class: "card" }), head = el("div", { class: "head" });
    head.appendChild(el("h2", {}, `ID ${j.id} · ${j.name}`));
    head.appendChild(el("p", {}, `${fmt(j.summary.start)} → ${fmt(j.summary.goal)}, max tracking error ${fmt(j.summary.max_tracking_error)} steps`));
    card.appendChild(head);
    const charts = el("div", { class: "charts" });
    const mk = (title) => { const w = el("div"); w.appendChild(el("p", { class: "chart-title" }, title));
      const s = svgEl("svg", { class: "chart", tabindex: "0", role: "img", "aria-label": `${title}, ID ${j.id}` }); w.appendChild(s); charts.appendChild(w); return s; };
    const posSvg = mk("Position (steps): setpoint vs actual"), loadSvg = mk("Load (% drive duty)");
    const tip = el("div", { class: "tooltip" }); charts.appendChild(tip);
    card.appendChild(charts); host.appendChild(card);
    rows.push({ j, posSvg, loadSvg, tip, charts });
  }

  function render() {
    for (const { j, posSvg, loadSvg, tip, charts } of rows) {
      const pos = chart(posSvg, j.t, [{ values: j.setpoint, color: "var(--series-1)" }, { values: j.position, color: "var(--series-2)" }],
        { minPad: 6, digits: 0, arrival: j.summary.arrival_s });
      const load = chart(loadSvg, j.t, [{ values: j.load, color: "var(--series-2)" }], { minPad: 10, digits: 0 });
      let cur = 0;
      const showAt = (i, svg) => {
        cur = Math.max(0, Math.min(j.t.length - 1, i));
        const fp = pos.show(cur), fl = load.show(cur);
        tip.replaceChildren();
        tip.appendChild(el("span", {}, `t = ${j.t[cur].toFixed(3)} s`));
        const line = (v, label, color) => { const r = el("div", { class: "row" }); const k = el("i"); k.style.background = color;
          r.appendChild(k); r.appendChild(el("strong", {}, v)); r.appendChild(el("span", {}, label)); tip.appendChild(r); };
        line(fmt(j.setpoint[cur], 1), "setpoint", "var(--series-1)");
        line(fmt(j.position[cur]), "actual", "var(--series-2)");
        line(j.load[cur] === null ? "-" : `${fmt(j.load[cur], 1)} %`, "load", "var(--series-2)");
        if (j.speed && j.speed[cur] !== null && j.speed[cur] !== undefined) line(String(j.speed[cur]), "speed register", "var(--axis)");
        const box = charts.getBoundingClientRect(), a = svg.getBoundingClientRect();
        let left = a.left - box.left + (svg === posSvg ? fp : fl) * a.width + 12;
        if (left > box.width - 150) left -= 174;
        tip.style.left = `${Math.max(0, left)}px`; tip.style.top = `${a.top - box.top + 8}px`; tip.style.display = "block";
      };
      for (const [svg, c] of [[posSvg, pos], [loadSvg, load]]) {
        svg.onpointermove = (e) => { if (j.t.length) showAt(c.indexAt(e.clientX), svg); };
        svg.onpointerleave = svg.onblur = () => { pos.hide(); load.hide(); tip.style.display = "none"; };
        svg.onfocus = () => { if (j.t.length) showAt(cur, svg); };
        svg.onkeydown = (e) => { if (e.key === "ArrowRight") { showAt(cur + (e.shiftKey ? 10 : 1), svg); e.preventDefault(); }
          if (e.key === "ArrowLeft") { showAt(cur - (e.shiftKey ? 10 : 1), svg); e.preventDefault(); } };
      }
    }
  }
  render();
  let timer; window.addEventListener("resize", () => { clearTimeout(timer); timer = setTimeout(render, 120); });
})();
</script>
</body>
</html>
"""
