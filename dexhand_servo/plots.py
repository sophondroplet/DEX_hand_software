"""Self-contained HTML report for a monitor run (inline SVG, no external assets)."""

from __future__ import annotations

import json
from pathlib import Path

from .config import JOINT_MAP
from .monitor import MonitorResult, SeriesStats


def monitor_payload(
    result: MonitorResult, stats: dict[int, SeriesStats], jump_threshold: int, csv_name: str
) -> dict[str, object]:
    return {
        "meta": {
            "started_at": result.started_at,
            "port": result.port,
            "baudrate": result.baudrate,
            "rate_hz": result.rate_hz,
            "achieved_rate_hz": round(result.achieved_rate_hz, 1),
            "elapsed_s": round(result.elapsed_s, 2),
            "overruns": result.overruns,
            "interrupted": result.interrupted,
            "jump_threshold": jump_threshold,
            "csv_name": csv_name,
        },
        "servos": [
            {
                "id": servo_id,
                "joint": JOINT_MAP.get(servo_id, "-"),
                "model": model.label if (model := result.models.get(servo_id)) else "?",
                "t": [s.t for s in samples],
                "position": [s.position for s in samples],
                "voltage": [s.voltage for s in samples],
                "stats": stats[servo_id].to_dict(),
            }
            for servo_id, samples in result.series.items()
        ],
    }


def write_monitor_html(payload: dict[str, object], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    # "</" inside the JSON would close the <script> element early.
    data = json.dumps(payload, ensure_ascii=False).replace("</", "<\\/")
    path.write_text(_TEMPLATE.replace("__DATA__", data), encoding="utf-8")
    return path


_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Servo Monitor</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7;
  --surface: #fcfcfb;
  --text-primary: #0b0b0b;
  --text-secondary: #52514e;
  --text-muted: #898781;
  --grid: #e1e0d9;
  --axis: #c3c2b7;
  --border: rgba(11, 11, 11, 0.10);
  --series-1: #2a78d6;
  --warning: #fab219;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --page: #0d0d0d;
    --surface: #1a1a19;
    --text-primary: #ffffff;
    --text-secondary: #c3c2b7;
    --text-muted: #898781;
    --grid: #2c2c2a;
    --axis: #383835;
    --border: rgba(255, 255, 255, 0.10);
    --series-1: #3987e5;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d;
  --surface: #1a1a19;
  --text-primary: #ffffff;
  --text-secondary: #c3c2b7;
  --text-muted: #898781;
  --grid: #2c2c2a;
  --axis: #383835;
  --border: rgba(255, 255, 255, 0.10);
  --series-1: #3987e5;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--page);
  color: var(--text-primary);
  font: 14px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif;
}
main { max-width: 1120px; margin: 0 auto; padding: 24px 16px 48px; }
h1 { font-size: 22px; margin: 0 0 4px; }
h2 { font-size: 16px; margin: 0; }
.meta { color: var(--text-secondary); margin: 0 0 20px; }
.card {
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 16px;
  margin-bottom: 16px;
}
.table-wrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; }
th, td { padding: 6px 10px; text-align: right; border-bottom: 1px solid var(--grid); white-space: nowrap; }
th { color: var(--text-secondary); font-weight: 600; }
th:nth-child(-n+2), td:nth-child(-n+2) { text-align: left; }
.flag { display: inline-flex; align-items: center; gap: 4px; }
.flag::before { content: "▲"; color: var(--warning); font-size: 10px; }
.servo-head { display: flex; flex-wrap: wrap; align-items: baseline; gap: 4px 12px; margin-bottom: 8px; }
.servo-head p { margin: 0; color: var(--text-secondary); font-variant-numeric: tabular-nums; }
.charts { display: grid; grid-template-columns: 2fr 1fr; gap: 16px; position: relative; }
.charts > div { min-width: 0; }
@media (max-width: 720px) { .charts { grid-template-columns: 1fr; } }
.chart-title { color: var(--text-secondary); font-size: 12px; margin: 0 0 4px; }
.chart { width: 100%; height: 200px; display: block; outline: none; touch-action: pan-y; }
.chart:focus-visible { box-shadow: 0 0 0 2px var(--series-1); border-radius: 4px; }
.chart text { fill: var(--text-muted); font-size: 11px; font-variant-numeric: tabular-nums; }
.chart text.label { fill: var(--text-secondary); }
.tooltip {
  position: absolute;
  pointer-events: none;
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 6px;
  padding: 6px 10px;
  font-variant-numeric: tabular-nums;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  display: none;
  z-index: 2;
}
.tooltip .row { display: flex; align-items: center; gap: 6px; }
.tooltip .key { width: 10px; height: 2px; background: var(--series-1); border-radius: 1px; }
.tooltip strong { color: var(--text-primary); }
.tooltip span { color: var(--text-secondary); }
footer { color: var(--text-muted); font-size: 12px; }
</style>
</head>
<body>
<main>
  <h1>Servo monitor</h1>
  <p class="meta" id="meta"></p>
  <section class="card table-wrap">
    <table>
      <thead>
        <tr>
          <th>ID</th><th>Joint</th><th>Samples (failed)</th><th>Min</th><th>Max</th>
          <th>Spread steps / deg</th><th>Std</th><th>Max step</th><th id="jump-head">Jumps</th>
          <th>Voltage V</th><th>Temp max degC</th>
        </tr>
      </thead>
      <tbody id="summary"></tbody>
    </table>
  </section>
  <div id="servos"></div>
  <footer id="footer"></footer>
</main>
<script type="application/json" id="monitor-data">__DATA__</script>
<script>
(() => {
  const data = JSON.parse(document.getElementById("monitor-data").textContent);
  const meta = data.meta;
  const NS = "http://www.w3.org/2000/svg";

  const el = (tag, attrs = {}, text) => {
    const node = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
    if (text !== undefined) node.textContent = text;
    return node;
  };
  const svgEl = (tag, attrs = {}) => {
    const node = document.createElementNS(NS, tag);
    for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
    return node;
  };
  const fmt = (v, digits = 0) => (v === null || v === undefined ? "-" : Number(v).toFixed(digits));

  document.getElementById("meta").textContent =
    `${meta.started_at} · ${meta.port} @ ${meta.baudrate} bps · ${meta.elapsed_s} s at ` +
    `${meta.achieved_rate_hz} Hz (requested ${meta.rate_hz} Hz)` +
    (meta.interrupted ? " · stopped with Ctrl-C" : "") +
    (meta.overruns ? ` · ${meta.overruns} rate overruns` : "");
  document.getElementById("jump-head").textContent = `Jumps > ${meta.jump_threshold} (SCS0009 steps)`;
  document.getElementById("footer").textContent =
    `Read-only capture. Per-sample data: ${meta.csv_name}. ` +
    `Positions are in each servo's own steps; degrees use the nominal deg/step of its model ` +
    `(SCS0009: 300 deg over 1024 steps; HLS3606: 360 deg over 4096 steps). ` +
    `Jump thresholds are scaled to the same angle per model.`;

  const summary = document.getElementById("summary");
  for (const s of data.servos) {
    const st = s.stats;
    const tr = el("tr");
    const spread = st.position_spread === null ? "-" : `${st.position_spread} / ${fmt(st.spread_deg, 1)}`;
    const cells = [
      String(s.id), s.joint, `${st.ok} (${st.failed})`, fmt(st.position_min), fmt(st.position_max),
      spread, fmt(st.position_std, 2), fmt(st.max_step), String(st.jumps),
      st.voltage_min === null ? "-" : `${fmt(st.voltage_min, 1)}–${fmt(st.voltage_max, 1)}`,
      fmt(st.temperature_max),
    ];
    cells.forEach((text, i) => {
      const td = el("td");
      if ((i === 8 && st.jumps > 0) || (i === 2 && st.failed > 0)) {
        td.appendChild(el("span", { class: "flag" }, text));
      } else {
        td.textContent = text;
      }
      tr.appendChild(td);
    });
    summary.appendChild(tr);
  }

  function niceStep(span, count) {
    const raw = Math.max(span, 1e-9) / count;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    for (const f of [1, 2, 5, 10]) if (raw <= f * mag) return f * mag;
    return 10 * mag;
  }

  function domain(values, minPad) {
    const clean = values.filter((v) => v !== null);
    if (!clean.length) return { lo: 0, hi: 1, step: 0.5 };
    // Loop instead of Math.min(...arr): long captures exceed the engine's argument limit.
    let lo = Infinity, hi = -Infinity;
    for (const v of clean) { if (v < lo) lo = v; if (v > hi) hi = v; }
    if (hi - lo < minPad) { const mid = (hi + lo) / 2; lo = mid - minPad / 2; hi = mid + minPad / 2; }
    const step = niceStep(hi - lo, 4);
    return { lo: Math.floor(lo / step) * step, hi: Math.ceil(hi / step) * step, step };
  }

  function nearestIndex(t, x) {
    let lo = 0, hi = t.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (t[mid] < x) lo = mid; else hi = mid; }
    return Math.abs(t[lo] - x) <= Math.abs(t[hi] - x) ? lo : hi;
  }

  function buildChart(svg, t, y, opts) {
    while (svg.firstChild) svg.removeChild(svg.firstChild);
    const W = svg.clientWidth || 600, H = svg.clientHeight || 200;
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    const m = { l: 44, r: 12, t: 16, b: 24 };
    const tMax = t.length ? Math.max(t[t.length - 1], 1e-6) : 1;
    const yd = domain(y, opts.minPad);
    const xOf = (v) => m.l + (v / tMax) * (W - m.l - m.r);
    const yOf = (v) => H - m.b - ((v - yd.lo) / (yd.hi - yd.lo || 1)) * (H - m.t - m.b);

    for (let v = yd.lo; v <= yd.hi + yd.step / 2; v += yd.step) {
      const yy = yOf(v);
      svg.appendChild(svgEl("line", { x1: m.l, x2: W - m.r, y1: yy, y2: yy,
        stroke: v === yd.lo ? "var(--axis)" : "var(--grid)", "stroke-width": 1 }));
      const label = svgEl("text", { x: m.l - 6, y: yy + 4, "text-anchor": "end" });
      label.textContent = Number(v.toFixed(opts.digits));
      svg.appendChild(label);
    }
    const xStep = niceStep(tMax, 5);
    for (let v = 0; v <= tMax + 1e-9; v += xStep) {
      const label = svgEl("text", { x: xOf(v), y: H - 6, "text-anchor": "middle" });
      label.textContent = `${Number(v.toFixed(2))} s`;
      svg.appendChild(label);
    }

    let d = "", pen = false;
    for (let i = 0; i < t.length; i++) {
      if (y[i] === null) { pen = false; continue; }
      d += `${pen ? "L" : "M"}${xOf(t[i]).toFixed(1)},${yOf(y[i]).toFixed(1)}`;
      pen = true;
    }
    svg.appendChild(svgEl("path", { d, fill: "none", stroke: "var(--series-1)", "stroke-width": 2,
      "stroke-linejoin": "round", "stroke-linecap": "round" }));

    if (opts.labelExtremes) {
      let iMin = -1, iMax = -1;
      y.forEach((v, i) => {
        if (v === null) return;
        if (iMin < 0 || v < y[iMin]) iMin = i;
        if (iMax < 0 || v > y[iMax]) iMax = i;
      });
      const marks = iMin === iMax || (iMin >= 0 && y[iMin] === y[iMax])
        ? [[iMax, `${y[iMax]} (flat)`, -10]]
        : [[iMax, `max ${y[iMax]}`, -10], [iMin, `min ${y[iMin]}`, 18]];
      for (const [i, text, dy] of marks) {
        if (i < 0) continue;
        const x = xOf(t[i]), yy = yOf(y[i]);
        svg.appendChild(svgEl("circle", { cx: x, cy: yy, r: 4, fill: "var(--series-1)",
          stroke: "var(--surface)", "stroke-width": 2 }));
        const anchor = x > W - 80 ? "end" : x < m.l + 40 ? "start" : "middle";
        const label = svgEl("text", { x, y: Math.min(Math.max(yy + dy, m.t), H - m.b - 2),
          "text-anchor": anchor, class: "label" });
        label.textContent = text;
        svg.appendChild(label);
      }
    }

    const cross = svgEl("line", { y1: m.t, y2: H - m.b, stroke: "var(--axis)", "stroke-width": 1,
      visibility: "hidden" });
    const dot = svgEl("circle", { r: 4, fill: "var(--series-1)", stroke: "var(--surface)", "stroke-width": 2,
      visibility: "hidden" });
    svg.appendChild(cross);
    svg.appendChild(dot);
    return {
      indexAtClientX(clientX) {
        const box = svg.getBoundingClientRect();
        const px = ((clientX - box.left) / box.width) * W;
        return nearestIndex(t, ((px - m.l) / (W - m.l - m.r)) * tMax);
      },
      show(i) {
        const x = xOf(t[i]);
        cross.setAttribute("x1", x); cross.setAttribute("x2", x);
        cross.setAttribute("visibility", "visible");
        if (y[i] === null) dot.setAttribute("visibility", "hidden");
        else {
          dot.setAttribute("cx", x); dot.setAttribute("cy", yOf(y[i]));
          dot.setAttribute("visibility", "visible");
        }
        return x / W;  // crosshair position as a fraction of the chart width
      },
      hide() { cross.setAttribute("visibility", "hidden"); dot.setAttribute("visibility", "hidden"); },
    };
  }

  const rows = [];
  const host = document.getElementById("servos");
  for (const s of data.servos) {
    const st = s.stats;
    const card = el("section", { class: "card" });
    const head = el("div", { class: "servo-head" });
    head.appendChild(el("h2", {}, `ID ${s.id} · ${s.joint} · ${s.model}`));
    head.appendChild(el("p", {},
      st.position_spread === null ? "no clean samples"
        : `range ${st.position_min}–${st.position_max} (${st.position_spread} steps ≈ ${fmt(st.spread_deg, 1)} deg), ` +
          `max step ${fmt(st.max_step)}, ${st.jumps} jump(s) > ${st.jump_threshold}`));
    card.appendChild(head);

    const charts = el("div", { class: "charts" });
    const make = (title) => {
      const wrap = el("div");
      wrap.appendChild(el("p", { class: "chart-title" }, title));
      const svg = svgEl("svg", { class: "chart", tabindex: "0", role: "img", "aria-label": `${title}, ID ${s.id}` });
      wrap.appendChild(svg);
      charts.appendChild(wrap);
      return svg;
    };
    const posSvg = make("Present position (steps)");
    const voltSvg = make("Supply voltage at servo (V)");
    const tip = el("div", { class: "tooltip" });
    charts.appendChild(tip);
    card.appendChild(charts);
    host.appendChild(card);
    rows.push({ s, posSvg, voltSvg, tip, charts });
  }

  function render() {
    for (const row of rows) {
      const { s, posSvg, voltSvg, tip, charts } = row;
      const pos = buildChart(posSvg, s.t, s.position, { minPad: 4, digits: 0, labelExtremes: true });
      const volt = buildChart(voltSvg, s.t, s.voltage, { minPad: 0.4, digits: 1, labelExtremes: false });
      let current = -1;
      const showAt = (i, anchorSvg) => {
        current = Math.max(0, Math.min(s.t.length - 1, i));
        const fracPos = pos.show(current), fracVolt = volt.show(current);
        tip.replaceChildren();
        const line = (value, label) => {
          const r = el("div", { class: "row" });
          r.appendChild(el("i", { class: "key" }));
          r.appendChild(el("strong", {}, value));
          r.appendChild(el("span", {}, label));
          tip.appendChild(r);
        };
        tip.appendChild(el("span", {}, `t = ${s.t[current].toFixed(3)} s`));
        line(s.position[current] === null ? "no reply" : String(s.position[current]), "position");
        line(s.voltage[current] === null ? "-" : `${s.voltage[current].toFixed(1)} V`, "voltage");
        const box = charts.getBoundingClientRect(), a = anchorSvg.getBoundingClientRect();
        const frac = anchorSvg === posSvg ? fracPos : fracVolt;
        let left = a.left - box.left + frac * a.width + 12;
        if (left > box.width - 150) left -= 174;
        tip.style.left = `${Math.max(0, left)}px`;
        tip.style.top = `${a.top - box.top + 8}px`;
        tip.style.display = "block";
      };
      const hide = () => { pos.hide(); volt.hide(); tip.style.display = "none"; };
      for (const [svg, chart] of [[posSvg, pos], [voltSvg, volt]]) {
        svg.onpointermove = (e) => { if (s.t.length) showAt(chart.indexAtClientX(e.clientX), svg); };
        svg.onpointerleave = hide;
        svg.onblur = hide;
        svg.onfocus = () => { if (s.t.length) showAt(current < 0 ? 0 : current, svg); };
        svg.onkeydown = (e) => {
          if (!s.t.length) return;
          const stepSize = e.shiftKey ? 10 : 1;
          if (e.key === "ArrowRight") { showAt(current + stepSize, svg); e.preventDefault(); }
          if (e.key === "ArrowLeft") { showAt(current - stepSize, svg); e.preventDefault(); }
        };
      }
    }
  }

  render();
  let timer;
  window.addEventListener("resize", () => { clearTimeout(timer); timer = setTimeout(render, 120); });
})();
</script>
</body>
</html>
"""
