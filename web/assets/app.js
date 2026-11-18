"use strict";
// Sifted operator view.
//
// Two modes:
//  * With the launcher (./sifted): the page talks to its local control API
//    (/api/...) to run scenarios, replay recordings and show the service's
//    counters, and reads the track stream from the WebSocket the API names.
//  * Standalone (any static server, e.g. `python3 -m http.server` in web/):
//    a read-only view of a daemon's track stream, as before.
//      index.html?ws=ws://host:port/   (default ws://<this host>:8765/)
//
// The track stream is JSON "sifted.tracks" v2 (src/sifted/net/snapshot.hpp).
// Nothing is ever sent to the daemon.

const $ = id => document.getElementById(id);
const params = new URLSearchParams(location.search);
const SNAPSHOT_VERSION = 2;
const LEADER_S = 30;
const TRACE_KEEP = 64;     // decisions remembered per track in this page
const GATE_CLOSE = 0.75;   // d² / gate from which a measurement is flagged as close to the gate
const TRACE_ROUTINE = new Set(["associated", "competed", "near miss"]);  // per-measurement decisions
const COLORS = { confirmed: "#3ea6ff", coasting: "#f0b429", tentative: "#8b98a8" };
const store = {  // localStorage can be missing or throw (private mode, some embeds)
  get(k) { try { return localStorage.getItem(k); } catch { return null; } },
  set(k, v) { try { localStorage.setItem(k, v); } catch { /* ignore */ } },
};

const ui = {
  api: false,         // launcher present?
  config: null,
  status: null,
  lastSeq: -1,
  scenarios: [],
  selectedScenario: store.get("sifted.scenario") || "demo",
  speed: Number(store.get("sifted.speed")) || 1,
  wsUrl: params.get("ws") || `ws://${location.hostname || "localhost"}:8765/`,
  wsOpen: false,
  lastSnapshotAt: 0,
  tracksSeen: 0,
  selected: null,
};

function esc(s) {
  return String(s).replace(/[&<>"']/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;" }[c]));
}
function fmtTime(s) {
  s = Math.max(0, Math.round(s));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
}
function fmtNum(n) { return n == null ? "–" : Number(n).toLocaleString(); }

function toast(text, kind = "") {
  const el = document.createElement("div");
  el.className = `toast ${kind}`;
  el.textContent = text;
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), kind === "err" ? 8000 : 4500);
}

// ------------------------------------------------------------------ API

async function api(path, body) {
  const opts = body === undefined ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json", "X-Sifted": "1" },
    body: JSON.stringify(body),
  };
  const r = await fetch(path, opts);
  let data = null;
  try { data = await r.json(); } catch { /* non-JSON error page */ }
  if (!r.ok) throw new Error((data && data.error) || `${r.status} ${r.statusText}`);
  return data;
}

async function action(path, body, busyBtn, okText) {
  if (busyBtn) busyBtn.disabled = true;
  try {
    const st = await api(path, body);
    if (st && st.state) applyStatus(st);
    if (okText) toast(okText, "ok");
    return st;
  } catch (e) {
    toast(e.message, "err");
    return null;
  } finally {
    if (busyBtn) busyBtn.disabled = false;
  }
}

// ------------------------------------------------------------------ map

const map = L.map("map", { zoomControl: true, attributionControl: true }).setView([51.4779, -0.0015], 11);
L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: "&copy; <a href=\"https://www.openstreetmap.org/copyright\">OpenStreetMap</a> contributors",
}).on("tileerror", () => {
  ui.tileErrors = (ui.tileErrors || 0) + 1;
  if (ui.tileErrors > 4 && !ui.tileLoaded) $("mapNote").hidden = false;
}).on("tileload", () => { ui.tileLoaded = true; $("mapNote").hidden = true; }).addTo(map);

function setMapTheme(dark) {
  document.body.classList.toggle("dark-map", dark);
  $("themeBtn").textContent = dark ? "Light Map" : "Dark Map";
  store.set("sifted.mapTheme", dark ? "dark" : "light");
}
setMapTheme(store.get("sifted.mapTheme") !== "light");
$("themeBtn").onclick = () => setMapTheme(!document.body.classList.contains("dark-map"));

const layers = new Map();  // track id -> {dot, ring, leader, label}
const traces = new Map();  // track id -> {total, events: oldest first, each with its decision number n}
let fitted = false;
// Keep zooming out to new tracks until the user moves the map themselves.
let autoFit = true;
map.on("dragstart", () => { autoFit = false; });
map.getContainer().addEventListener("wheel", () => { autoFit = false; }, { passive: true });

// Offset (east, north) metres from (lat, lon), small-distance approximation for display.
function offset(lat, lon, east, north) {
  const r = 6378137;
  return [lat + (north / r) * 180 / Math.PI,
          lon + (east / (r * Math.cos(lat * Math.PI / 180))) * 180 / Math.PI];
}

function clearTracks() {
  for (const l of layers.values()) for (const k of ["dot", "ring", "leader", "label"]) map.removeLayer(l[k]);
  layers.clear();
  traces.clear();
  resetTruthView();
  ui.selected = null;
  renderWhy();
  $("rows").innerHTML = "";
  $("trackCount").textContent = "0";
  ui.tracksSeen = 0;
  fitted = false;
  autoFit = true;
  updateTracksEmpty();
}

function fitTracks() {
  if (layers.size === 0) return false;
  map.fitBounds(L.featureGroup([...layers.values()].map(l => l.ring)).getBounds().pad(0.15),
    { maxZoom: 14, paddingTopLeft: [40, 70], paddingBottomRight: [40, 80] });
  return true;
}
$("fitBtn").onclick = () => { if (!fitTracks()) toast("No tracks to show yet."); };

function selectTrack(id, pan) {
  ui.selected = id;
  for (const tr of $("rows").children) tr.classList.toggle("sel", Number(tr.dataset.id) === id);
  const l = layers.get(id);
  if (l && pan) { map.panTo(l.dot.getLatLng()); l.dot.openTooltip(); }
  renderWhy();
}

// ------------------------------------------------------------------ decision trace

// Each snapshot carries only a track's newest decisions. Their numbers (from
// trace_total) say exactly which ones are new, so the page can build a longer
// history than any single snapshot holds.
function mergeTrace(t) {
  if (!Array.isArray(t.trace)) return;
  let h = traces.get(t.id);
  if (!h || t.trace_total < h.total) { h = { total: 0, events: [] }; traces.set(t.id, h); }
  const first = t.trace_total - t.trace.length;
  t.trace.forEach((e, i) => { if (first + i >= h.total) h.events.push({ ...e, n: first + i }); });
  h.total = t.trace_total;
  // Routine decisions make way first, so a track's birth, manoeuvres and outages stay visible.
  while (h.events.length > TRACE_KEEP) {
    const oldest = h.events.findIndex(e => TRACE_ROUTINE.has(e.kind));
    h.events.splice(Math.max(oldest, 0), 1);
  }
}

function fmtStat(v) { return v >= 100 ? v.toFixed(0) : v.toFixed(1); }
function fmtAgo(ns) {
  const s = Math.max(0, ns / 1e9);
  return s < 0.05 ? "now" : s < 60 ? `${s.toFixed(1)} s` : fmtTime(s);
}
function traceSensor(id) {
  return ui.sensorNames && ui.sensorNames[id] ? sensorName(id) : `sensor ${esc(id)}`;
}
function traceLabel(e) {
  const s = traceSensor(e.sensor);
  const has = e.d2 != null && e.gate != null;
  const gated = has ? `d² ${fmtStat(e.d2)} / gate ${fmtStat(e.gate)}` : "";
  const nis = has ? `: innovation ${fmtStat(e.d2)} / threshold ${fmtStat(e.gate)}` : "";
  switch (e.kind) {
    case "initiated": return `Started from ${s}`;
    case "associated": return `Associated ${s}${has ? `: ${gated}` : ""}`;
    case "competed": return `Competed: measurement ${e.other ? `went to track ${esc(e.other)}` : "left unassigned"} (${s}${has ? `, ${gated}` : ""})`;
    case "near miss": return `Near miss from ${s}${has ? `: ${gated}` : ""}`;
    case "update rejected": return `Filter rejected update from ${s}${has ? `: ${gated}` : ""}`;
    case "confirmed": return "Confirmed";
    case "coasting": return "Coasting: no recent position measurements";
    case "recovered": return "Recovered: position measurements resumed";
    case "dropped": return `Dropped: ${esc(e.drop_reason)}`;
    case "manoeuvre start": return `Manoeuvre detected${nis}`;
    case "manoeuvre end": return `Manoeuvre ended${nis}`;
    default: return esc(e.kind);
  }
}
// How close a measurement came to the gate. Manoeuvre events carry a different statistic.
function gateCue(e) {
  if (e.d2 == null || !(e.gate > 0) || e.kind.startsWith("manoeuvre")) return "";
  const ratio = e.d2 / e.gate;
  const cls = ratio > 1 ? "out" : ratio >= GATE_CLOSE ? "close" : "";
  const tag = cls === "close" ? '<span class="badge warn">Close to Gate</span>' : cls === "out" ? '<span class="badge err">Outside Gate</span>' : "";
  return `<div class="gate ${cls}" title="d² is ${Math.round(ratio * 100)}% of the gate">` +
    `<span class="gate-meter"><i style="width:${Math.min(ratio, 1) * 100}%"></i></span>${tag}</div>`;
}

function renderWhy() {
  const id = ui.selected;
  const h = id == null ? null : traces.get(id);
  $("why").hidden = id == null;
  if (id == null) return;
  $("whyTitle").textContent = `Why Track ${id}`;
  $("whyNote").hidden = !ui.reported || ui.reported.has(id);
  $("whyNote").textContent = "No longer reported: dropped, or hidden by a filter. These are the last decisions received.";
  const events = h ? h.events : [];
  const list = $("whyList"), top = list.scrollTop;
  list.innerHTML = [...events].reverse().map(e => {
    return `<li class="${TRACE_ROUTINE.has(e.kind) ? "" : "key"}"><span class="at">${fmtAgo((ui.traceNow || 0) - e.time_ns)}</span>` +
      `<span class="what">${traceLabel(e)}</span>${gateCue(e)}</li>`;
  }).join("");
  list.scrollTop = top;
  $("whyFoot").textContent = events.length
    ? `Newest first · ${fmtNum(events.length)} of ${fmtNum(h.total)} decisions · times are before the latest update`
    : "No decisions received for this track yet.";
}
$("whyClose").onclick = () => selectTrack(null, false);

function render(snap) {
  if (snap.type !== "sifted.tracks" || snap.version !== SNAPSHOT_VERSION || !Array.isArray(snap.tracks)) {
    setConn("unsupported data", "err");
    return;
  }
  ui.lastSnapshotAt = Date.now();
  if (!snap.origin) {
    setConn("no map origin", "warn");
    return;
  }
  setConn(`live · t = ${(snap.time_ns / 1e9).toFixed(1)} s`, "ok");
  const showTentative = $("showTentative").checked;
  const seen = new Set();
  const rows = [];
  const tracks = [...snap.tracks].sort((a, b) => a.id - b.id);
  ui.reported = new Set(tracks.map(t => t.id));
  ui.traceNow = snap.time_ns;
  for (const id of traces.keys()) if (!ui.reported.has(id) && id !== ui.selected) traces.delete(id);
  for (const t of tracks) {
    mergeTrace(t);
    if (typeof t.lat !== "number" || typeof t.lon !== "number") continue;
    if (t.status === "tentative" && !showTentative) continue;
    seen.add(t.id);
    const color = COLORS[t.status] || "#e6edf3";
    const pos = [t.lat, t.lon];
    const end = offset(t.lat, t.lon, t.vx * LEADER_S, t.vy * LEADER_S);
    const speed = Math.hypot(t.vx, t.vy);
    const sensors = (t.sensors || []).map(id => sensorName(id)).join(", ") || "–";
    const tip = `<b>Track ${t.id}</b> · ${esc(t.status)}${t.manoeuvring ? " · manoeuvring" : ""}<br>` +
      `Uncertainty ${t.sigma_m.toFixed(1)} m (1σ)<br>Speed ${speed.toFixed(1)} m/s · ` +
      `${Math.round((Math.atan2(t.vx, t.vy) * 180 / Math.PI + 360) % 360)}°<br>` +
      `Sensors ${sensors}<br>${fmtNum(t.updates)} updates`;
    let l = layers.get(t.id);
    if (!l) {
      l = {
        ring: L.circle(pos, { radius: 1, weight: 1, fillOpacity: 0.1, interactive: false }).addTo(map),
        leader: L.polyline([pos, end], { weight: 2, opacity: 0.85, interactive: false }).addTo(map),
        dot: L.circleMarker(pos, { radius: 6, weight: 2, fillOpacity: 1 }).addTo(map),
        label: L.marker(pos, { interactive: false, keyboard: false,
          icon: L.divIcon({ className: "", html: "", iconSize: [0, 0] }) }).addTo(map),
      };
      l.dot.bindTooltip("", { direction: "top", className: "trk", offset: [0, -6] });
      l.dot.on("click", () => { selectTrack(t.id, false); showTab("tracks"); });
      layers.set(t.id, l);
      ui.tracksSeen++;
    }
    const dash = t.status === "coasting" ? "4 4" : null;
    l.tip = tip;
    l.sigma = t.sigma_m;
    l.dot.setLatLng(pos).setStyle({ ...trackDotStyle(t.id), fillColor: color });
    l.dot.setTooltipContent(tip + truthTip(t.id, t.sigma_m));
    l.ring.setLatLng(pos).setRadius(Math.max(t.sigma_m, 1)).setStyle({ color, fillColor: color, dashArray: dash });
    l.leader.setLatLngs([pos, end]).setStyle({ color, dashArray: dash });
    l.label.setLatLng(pos).setIcon(L.divIcon({
      className: "", iconSize: [0, 0],
      html: `<span style="position:absolute;left:9px;top:-7px;font:600 11px ui-monospace,monospace;color:#e6edf3;` +
        `text-shadow:0 0 3px #000,0 0 3px #000;white-space:nowrap">${t.id}</span>`,
    }));
    const stColor = t.status === "confirmed" ? "var(--confirmed)" : t.status === "coasting" ? "var(--coasting)" : "var(--tentative)";
    rows.push(`<tr class="${esc(t.status)}${ui.selected === t.id ? " sel" : ""}" data-id="${t.id}">` +
      `<td class="id">${t.id}</td>` +
      `<td><span class="st" style="--c:${stColor}"><i></i>${esc(t.status)}${t.manoeuvring ? " ↻" : ""}</span></td>` +
      `<td>${t.sigma_m.toFixed(1)}</td>${truthCellHtml(t.id, t.sigma_m)}<td>${speed.toFixed(0)} m/s</td>` +
      `<td title="Sensor ids: ${esc((t.sensors || []).join(", "))}">${(t.sensors || []).map(id => sensorName(id, false)).join(", ") || "–"}</td></tr>`);
  }
  for (const [id, l] of layers) {  // tracks that disappeared (dropped or filtered)
    if (!seen.has(id)) {
      for (const k of ["dot", "ring", "leader", "label"]) map.removeLayer(l[k]);
      layers.delete(id);
    }
  }
  $("rows").innerHTML = rows.join("");
  $("trackCount").textContent = String(layers.size);
  updateTracksEmpty();
  renderWhy();
  if ($("follow").checked && layers.size > 0) {
    const view = map.getBounds();
    const outside = [...layers.values()].some(l => !view.contains(l.dot.getLatLng()));
    if (!fitted || (autoFit && outside)) fitted = fitTracks();
  }
  updateOverlay();
  truthOnSnapshot(snap);
}

// ------------------------------------------------------------------ ground truth overlay

// A display aid for simulated runs and their replays. The launcher does the time
// alignment and matching (sim/sifted_sim/truth.py); this page only draws the answer.
const TRUTH_COLORS = { truth: "#e879f9", missed: "#ff5c5c", unmatched: "#ff7a45" };
const truth = {
  on: store.get("sifted.truth") === "1",
  run: null,            // run whose truth this page has asked about
  runGen: 0,            // discards the answer about a run no longer shown
  available: false,
  gate: null,           // match distance, from the launcher
  gen: 0,               // discards comparisons from before a reset
  busy: false,
  queued: null,         // newest snapshot waiting for a comparison
  lastSnap: null,
  lastT: null,
  pairs: {},            // target -> track, handed back with each request
  result: null,
  unmatched: new Set(),
  marks: new Map(),     // target id -> {mark, trail, state}
};

function truthShown() { return truth.on && truth.available; }
function truthVerdict(id) {
  return truthShown() && truth.result ? truth.result.tracks[id] || null : null;
}
function truthFlagged(id) { return truthShown() && truth.unmatched.has(id); }
function trackDotStyle(id) {
  return truthFlagged(id) ? { color: TRUTH_COLORS.unmatched, weight: 3 } : { color: "#0b1017", weight: 2 };
}
function errorInSigma(errorM, sigma) { return sigma > 0 ? `${(errorM / sigma).toFixed(1)}σ` : "–"; }
function truthTip(id, sigma) {
  const v = truthVerdict(id);
  if (v) return `<br>Error ${v.error_m.toFixed(1)} m (${errorInSigma(v.error_m, sigma)}) to target ${esc(v.target)}`;
  return truthFlagged(id) ? "<br>No true target within the match distance" : "";
}
function truthCell(id, sigma) {
  const v = truthVerdict(id);
  if (v) return { cls: "terr", text: v.error_m.toFixed(1), title: `${errorInSigma(v.error_m, sigma)} · target ${v.target}` };
  if (truthFlagged(id)) return { cls: "terr none", text: "None", title: "No true target within the match distance: possibly a false track" };
  return { cls: "terr", text: "–", title: "" };
}
function truthCellHtml(id, sigma) {
  const c = truthCell(id, sigma);
  return `<td class="${c.cls}" title="${esc(c.title)}">${esc(c.text)}</td>`;
}

// Tracks are drawn before their comparison returns, so refresh them once it has.
function refreshTruthTracks() {
  for (const [id, l] of layers) {
    l.dot.setStyle(trackDotStyle(id));
    if (l.tip) l.dot.setTooltipContent(l.tip + truthTip(id, l.sigma));
  }
  for (const tr of $("rows").children) {
    const id = Number(tr.dataset.id), td = tr.querySelector(".terr"), l = layers.get(id);
    if (!td || !l) continue;
    const c = truthCell(id, l.sigma);
    td.className = c.cls;
    td.textContent = c.text;
    td.title = c.title;
  }
}

function truthIcon(id, state) {
  return L.divIcon({ className: "", iconSize: [0, 0],
    html: `<div class="truth-mark ${esc(state)}"><i></i><b>T${esc(id)}</b></div>` });
}
function drawTruth(r) {
  const seen = new Set();
  for (const t of r.targets) {
    seen.add(t.id);
    const pos = [t.lat, t.lon];
    const color = t.state === "missed" ? TRUTH_COLORS.missed : TRUTH_COLORS.truth;
    let m = truth.marks.get(t.id);
    if (!m) {
      m = {
        trail: L.polyline(t.trail, { color, weight: 2, opacity: 0.8, interactive: false }).addTo(map),
        mark: L.marker(pos, { interactive: false, keyboard: false, icon: truthIcon(t.id, t.state) }).addTo(map),
        state: t.state,
      };
      truth.marks.set(t.id, m);
    }
    m.trail.setLatLngs(t.trail).setStyle({ color });
    m.mark.setLatLng(pos);
    if (m.state !== t.state) { m.mark.setIcon(truthIcon(t.id, t.state)); m.state = t.state; }
  }
  for (const [id, m] of truth.marks) {
    if (!seen.has(id)) { map.removeLayer(m.mark); map.removeLayer(m.trail); truth.marks.delete(id); }
  }
}

function renderTruthStats(r) {
  const unmatched = r.unmatched_tracks.length;
  const cells = [
    ["Mean Error", r.mean_error_m == null ? "–" : `${r.mean_error_m.toFixed(1)} m`],
    ["Targets Matched", `${r.matched} / ${r.present}`],
    ["Missed Targets", r.missed],
    ["Unmatched Tracks", unmatched],
  ];
  $("truthStats").innerHTML = cells.map(([k, v]) => `<div class="stat"><b>${esc(v)}</b><span>${esc(k)}</span></div>`).join("");
  $("truthNote").textContent = r.in_span
    ? `Display aid: targets are paired one-to-one with tracks within ${truth.gate} m. The scorecard and tests are the authoritative measure of accuracy.`
    : "The track stream's time is outside this run's ground truth, so nothing is compared.";
}

function updateTruthControls() {
  const btn = $("truthBtn"), shown = truthShown();
  btn.hidden = !truth.available;
  btn.textContent = truth.on ? "Hide Ground Truth" : "Show Ground Truth";
  btn.setAttribute("aria-pressed", String(truth.on));
  $("truthPanel").hidden = !shown;
  $("tracksTable").classList.toggle("truth-on", shown);
  for (const el of document.querySelectorAll(".truth-key")) el.hidden = !shown;
}

function resetTruthView({ keepSnapshot = false } = {}) {
  truth.gen++;
  truth.pairs = {};
  truth.result = null;
  truth.unmatched = new Set();
  truth.queued = null;
  truth.lastT = null;
  if (!keepSnapshot) truth.lastSnap = null;
  for (const m of truth.marks.values()) { map.removeLayer(m.mark); map.removeLayer(m.trail); }
  truth.marks.clear();
  $("truthStats").innerHTML = "";
  refreshTruthTracks();
}

function applyTruth(r) {
  if (!r.available) {
    truth.available = false;
    resetTruthView();
    updateTruthControls();
    return;
  }
  truth.pairs = r.pairs;
  truth.result = r;
  truth.unmatched = new Set(r.unmatched_tracks);
  drawTruth(r);
  refreshTruthTracks();
  renderTruthStats(r);
}

// One request at a time; snapshots that arrive meanwhile collapse to the newest.
async function pumpTruth() {
  if (truth.busy || !truthShown() || !truth.queued) return;
  const snap = truth.queued, gen = truth.gen;
  truth.queued = null;
  truth.busy = true;
  try {
    const tracks = snap.tracks.filter(t => Number.isFinite(t.x) && Number.isFinite(t.y))
      .map(t => ({ id: t.id, x: t.x, y: t.y, status: t.status }));
    const r = await api("/api/truth/compare", { run: truth.run, time_ns: snap.time_ns, origin: snap.origin, tracks, pairs: truth.pairs });
    if (gen === truth.gen) applyTruth(r);
  } catch (e) {
    console.warn("ground truth comparison failed:", e.message);
  } finally {
    truth.busy = false;
    pumpTruth();
  }
}

function truthOnSnapshot(snap) {
  if (truth.lastT != null && snap.time_ns < truth.lastT) resetTruthView();  // a replay restarted
  truth.lastT = snap.time_ns;
  truth.lastSnap = snap;
  truth.queued = snap;
  pumpTruth();
}

// Asks once per run whether it has truth (ADS-B and older recordings do not).
function syncTruthRun(a) {
  const id = a.id || null;
  if (id === truth.run) return;
  truth.run = id;
  truth.available = false;
  const gen = ++truth.runGen;
  resetTruthView();
  updateTruthControls();
  if (!id) return;
  api(`/api/truth?run=${encodeURIComponent(id)}`).then(r => {
    if (gen !== truth.runGen) return;
    truth.available = !!r.available;
    truth.gate = r.match_distance_m;
    updateTruthControls();
    truth.queued = truth.lastSnap;
    pumpTruth();
  }).catch(() => { if (gen === truth.runGen) truth.run = null; });  // the next status poll retries
}

$("truthBtn").onclick = () => {
  truth.on = !truth.on;
  store.set("sifted.truth", truth.on ? "1" : "0");
  resetTruthView({ keepSnapshot: true });
  updateTruthControls();
  truth.queued = truth.lastSnap;
  pumpTruth();
};

// Sensor ids -> readable names, from the running scenario's sensor table.
const KIND_NAMES = { range_bearing: "Radar", bearing_only: "Camera", geo_position: "ADS-B" };
function sensorName(id, withId = true) {
  const n = ui.sensorNames && ui.sensorNames[id];
  if (!n) return esc(id);
  return withId ? `${esc(n)} <span class="muted">${esc(id)}</span>` : esc(n);
}
function updateSensorNames(a) {
  ui.sensorNames = {};
  if (a.scenario === "adsb") { ui.sensorNames[100] = "ADS-B"; return; }
  const sc = ui.scenarios.find(s => s.name === a.scenario);
  if (sc) for (const x of sc.sensors) ui.sensorNames[x.id] = KIND_NAMES[x.kind] || x.kind;
}

function updateTracksEmpty() { $("tracksEmpty").hidden = layers.size > 0; }

$("rows").addEventListener("click", e => {
  const tr = e.target.closest("tr");
  if (tr) selectTrack(Number(tr.dataset.id), true);
});
$("showTentative").onchange = () => { /* applied on the next snapshot (several per second) */ };

// ------------------------------------------------------------------ track stream

function setConn(text, kind) {
  $("connText").textContent = text;
  $("conn").querySelector(".dot").className = `dot ${kind || ""}`;
}

const STREAM_STATES = ["starting", "running", "finished", "replaying", "live"];
function wantStream() { return !ui.api || STREAM_STATES.includes(ui.status && ui.status.state); }

let retry = 500, ws = null, wsTimer = null;
function connect() {
  clearTimeout(wsTimer);
  try { ws = new WebSocket(ui.wsUrl); } catch (e) { scheduleReconnect(); return; }
  ws.onopen = () => { retry = 500; ui.wsOpen = true; setConn("connected · waiting", "ok"); updateOverlay(); };
  ws.onmessage = ev => {
    try { render(JSON.parse(ev.data)); } catch (err) { setConn("bad message", "err"); console.error(err); }
  };
  ws.onclose = () => {
    const was = ui.wsOpen;
    ui.wsOpen = false;
    setConn(wantStream() ? "reconnecting…" : "idle", wantStream() ? "warn" : "");
    if (was) updateOverlay();
    scheduleReconnect();
  };
}
function scheduleReconnect() {
  clearTimeout(wsTimer);
  if (!wantStream()) return;  // nothing is serving tracks; applyStatus reconnects when it is
  wsTimer = setTimeout(connect, retry);
  retry = Math.min(retry * 2, ui.api ? 2000 : 10000);
}
function reconnectNow() {
  retry = 300;
  if (ws && ws.readyState <= 1) ws.close();  // onclose schedules the reconnect
  else if (wantStream()) connect();
}

// ------------------------------------------------------------------ overlay (empty states)

function showOverlay(title, text, { spin = false, demoBtn = false } = {}) {
  $("overlay").hidden = false;
  $("overlayTitle").textContent = title;
  $("overlayText").textContent = text;
  $("overlaySpin").hidden = !spin;
  $("overlayBtn").hidden = !demoBtn;
}

function updateOverlay() {
  if (layers.size > 0) { $("overlay").hidden = true; return; }
  if (!ui.api) {
    if (!ui.wsOpen) showOverlay("Waiting for the Fusion Service", `Trying ${ui.wsUrl} …`, { spin: true });
    else showOverlay("Connected", "No confirmed tracks yet. They appear once a target has been seen a few times.", { spin: true });
    return;
  }
  const st = ui.status ? ui.status.state : "idle";
  const a = (ui.status && ui.status.activity) || {};
  if (ui.config && !ui.config.built) {
    showOverlay("Almost There", "The fusion service isn't built yet. Run the installer, then start Sifted again.");
  } else if (st === "idle") {
    showOverlay("Nothing Running Yet", "Pick a scenario on the left and press Run, or start with the guided demo.", { demoBtn: true });
  } else if (st === "starting") {
    showOverlay("Starting…", "Launching the fusion service and the simulated sensors.", { spin: true });
  } else if (st === "live") {
    showOverlay("Waiting for Aircraft", ui.config && ui.config.adsb_authenticated
      ? "Polling OpenSky about every 20 s. Tracks appear after a few updates."
      : "Anonymous OpenSky access updates every few minutes, so the first tracks can take a while.", { spin: true });
  } else if (st === "running" || st === "replaying") {
    showOverlay("Waiting for Confirmed Tracks", "A track is shown once a radar has seen its target three times (a few seconds).", { spin: true });
  } else if (st === "finished" || st === "complete" || a.replay_done) {
    showOverlay(st === "complete" ? "Run Complete" : "All Frames Sent",
      st === "complete" ? "Replay it on the map, verify it bit-for-bit, or start a new run."
        : "Every track has coasted and dropped out. Finish the run to complete its recording.");
  } else {
    $("overlay").hidden = true;
  }
}
$("overlayBtn").onclick = () => runDemo();

// ------------------------------------------------------------------ run panel

function renderScenarios() {
  const el = $("scenarioList");
  if (!ui.scenarios.find(s => s.name === ui.selectedScenario) && ui.scenarios.length) ui.selectedScenario = ui.scenarios[0].name;
  el.innerHTML = ui.scenarios.map(s => {
    const kinds = [...new Set(s.sensors.map(x => x.kind === "range_bearing" ? "radar" : x.kind === "bearing_only" ? "camera" : x.kind))];
    const counts = kinds.map(k => `${s.sensors.filter(x => (x.kind === "range_bearing" ? "radar" : x.kind === "bearing_only" ? "camera" : x.kind) === k).length} ${k}`);
    return `<button class="scenario${s.name === ui.selectedScenario ? " on" : ""}" role="radio" aria-checked="${s.name === ui.selectedScenario}" data-name="${esc(s.name)}">` +
      `<div class="scenario-title">${esc(s.title)}${s.name === "demo" ? ' <span class="badge accent">Start Here</span>' : ""}</div>` +
      `<div class="scenario-desc">${esc(s.description)}</div>` +
      `<div class="scenario-meta">${fmtTime(s.duration_s)} · ${s.targets} target${s.targets === 1 ? "" : "s"} · ${counts.join(" + ")}</div></button>`;
  }).join("");
}
$("scenarioList").addEventListener("click", e => {
  const b = e.target.closest(".scenario");
  if (!b) return;
  ui.selectedScenario = b.dataset.name;
  store.set("sifted.scenario", ui.selectedScenario);
  renderScenarios();
});

function setSpeed(v) {
  ui.speed = v;
  store.set("sifted.speed", String(v));
  for (const b of $("speedSeg").children) b.classList.toggle("on", Number(b.dataset.v) === v);
}
$("speedSeg").addEventListener("click", e => { const b = e.target.closest("button"); if (b) setSpeed(Number(b.dataset.v)); });

function fillSelect(sel, items, hintEl, key) {
  sel.innerHTML = items.map(i => `<option value="${esc(i.id)}">${esc(i.title)}</option>`).join("");
  const saved = store.get(key);
  if (saved && items.find(i => i.id === saved)) sel.value = saved;
  const upd = () => { hintEl.textContent = (items.find(i => i.id === sel.value) || {}).description || ""; store.set(key, sel.value); };
  sel.onchange = upd;
  upd();
}

function runDemo() {
  ui.selectedScenario = "demo";
  renderScenarios();
  startRun("demo", 1, "outage", "none");
}
function startRun(scenario, speed, network, fault) {
  clearTracks();
  return action("/api/run", { scenario, speed, network, fault }, $("runBtn"));
}
$("runBtn").onclick = () => startRun(ui.selectedScenario, ui.speed, $("networkSel").value, $("faultSel").value);
$("stopBtn").onclick = $("nowStop").onclick = () => action("/api/stop", {}, null);
$("nowNew").onclick = () => { clearTracks(); action("/api/dismiss", {}, $("nowNew")); };

// ADS-B
function setBbox(b) { [$("bS").value, $("bN").value, $("bW").value, $("bE").value] = b.map(v => (+v).toFixed(2)); }
$("bboxFromMap").onclick = () => {
  const b = map.getBounds();
  setBbox([b.getSouth(), b.getNorth(), b.getWest(), b.getEast()]);
};
$("adsbBtn").onclick = async () => {
  const bbox = [$("bS").value, $("bN").value, $("bW").value, $("bE").value].map(Number);
  if (bbox.some(v => !Number.isFinite(v))) { toast("Enter all four edges of the area.", "err"); return; }
  clearTracks();
  const st = await action("/api/adsb", { bbox }, $("adsbBtn"));
  if (st) map.fitBounds([[bbox[0], bbox[2]], [bbox[1], bbox[3]]]);
};

// ------------------------------------------------------------------ "what to watch"

function watchItems(a) {
  const items = [];
  const D = a.duration_s || 0;
  if (a.scenario === "adsb") {
    return [
      [0, "Each aircraft's ADS-B report becomes one geodetic measurement; the tracker smooths them into tracks."],
      [0, "Updates are minutes or tens of seconds apart, so uncertainty circles grow between polls and shrink on each update."],
      [0, "Aircraft that stop reporting coast (amber) and are dropped after three minutes."],
    ];
  }
  if (a.scenario === "demo") {
    items.push([0, "Tracks appear a few seconds in: a track is shown once it is confirmed by 3 radar hits."]);
    items.push([5, "Two tracks converge in the middle: the crossing pair. Global nearest-neighbour assignment keeps their ids straight."]);
    items.push([20, "A new track in the east: a target entering radar coverage from outside."]);
    items.push([30, "Radar and camera both contribute (Sensors column): the camera's bearings from another site tighten the estimate."]);
    items.push([50, "The western track turns; its tooltip says \"manoeuvring\" while the filter adapts."]);
    items.push([95, "The south-western track leaves coverage: it coasts (amber, dashed), then is dropped."]);
  } else {
    items.push([0, "Tracks appear once confirmed by 3 radar hits. Turn on \"Show tentative tracks\" (Tracks tab) to see them earlier."]);
  }
  items.push([Math.min(10, D / 4), "Click a track on the map or in the Tracks tab to see why the system believes it: each association, near miss and lifecycle decision."]);
  if (a.network === "lossy" || a.network === "outage") {
    items.push([1, "The network loses, delays, reorders and duplicates frames: duplicates are suppressed and late frames put back in order (counters below)."]);
  }
  if (a.outage_at_s != null) {
    items.push([a.outage_at_s, "Link outage for 8 s: every track stops updating and coasts on prediction. Nothing is invented."]);
    items.push([a.outage_at_s + 8, "Recovery: tracks update again. Straight-flying targets keep their ids."]);
  }
  if (a.fault && a.fault !== "none") {
    const at = Math.round(D / 3);
    const what = { restart: "The radar reboots: its sequence numbers restart, and the tracks carry on undisturbed.",
      dropout: "The radar goes silent for 10 s: tracks coast, then recover when it returns.",
      stuck: "The radar freezes and repeats a stale scan for 8 s: watch the estimates hold rather than jump." }[a.fault];
    if (what) items.push([at, what]);
  }
  items.push([D, "All frames sent: tracks coast and drop out. Stop the run, then replay or verify it."]);
  return items.sort((x, y) => x[0] - y[0]);
}

function renderWatch(a, scenarioTime) {
  const items = watchItems(a);
  let cur = -1;
  items.forEach(([t], i) => { if (scenarioTime >= t) cur = i; });
  $("watchList").innerHTML = items.map(([t, text], i) =>
    `<li class="${i === cur ? "now" : i < cur ? "past" : ""}"><span class="at">${a.scenario === "adsb" ? "•" : fmtTime(t)}</span>${esc(text)}</li>`).join("");
}

// ------------------------------------------------------------------ status

const NET_TITLE = { perfect: "Perfect Network", lossy: "Lossy Network", outage: "Lossy + Outage" };
const FAULT_TITLE = { restart: "Radar Reboots", dropout: "Radar Goes Silent", stuck: "Radar Freezes" };

function applyStatus(st) {
  const prev = ui.status;
  ui.status = st;
  const a = st.activity || {};
  const state = st.state;

  // header pill
  const dot = $("sessionDot");
  const prog = $("sessionProgress");
  let text = "Ready", dotCls = "";
  if (state === "starting") { text = "Starting…"; dotCls = "busy"; }
  else if (state === "running") { text = `Running · ${a.title}`; dotCls = "busy"; }
  else if (state === "finished") { text = `Finished Sending · ${a.title}`; dotCls = "ok"; }
  else if (state === "complete") { text = `Run Complete · ${a.title}`; dotCls = "ok"; }
  else if (state === "replaying") { text = a.replay_done ? `Replay Finished · ${a.title}` : `Replaying · ${a.title}`; dotCls = a.replay_done ? "ok" : "busy"; }
  else if (state === "live") { text = "Live · OpenSky ADS-B"; dotCls = "busy"; }
  else if (state === "stopping") { text = "Stopping…"; dotCls = "warn"; }
  if (st.error) { dotCls = "err"; }
  $("sessionText").textContent = text;
  dot.className = `dot ${dotCls}`;
  const active = !["idle", "stopping"].includes(state);
  const done = state === "complete" || (state === "replaying" && a.replay_done);
  $("stopBtn").hidden = !active || done;
  const p = a.progress;
  prog.hidden = !(active && p != null);
  if (p != null) prog.firstElementChild.style.width = `${(p * 100).toFixed(1)}%`;

  // run panel
  if ($("setup").hidden !== active) $("tab-run").scrollTop = 0;
  $("setup").hidden = active;
  $("active").hidden = !active;
  if (active) renderActive(st);

  // tracks reset when a new activity starts or everything stops
  if (prev && prev.activity && (prev.activity.id !== a.id || (a.started !== prev.activity.started)) && state !== "starting") {
    if (!a.id) clearTracks();
  }
  if (prev && prev.state !== state && STREAM_STATES.includes(state) && !STREAM_STATES.includes(prev.state)) {
    reconnectNow();
  } else if (!ws && wantStream()) {
    connect();
  }
  if (prev && prev.state !== "idle" && state === "idle") clearTracks();

  if (st.error && (!prev || prev.error !== st.error)) toast(st.error, "err");
  if (prev && st.verify && st.verify.state && st.verify.state !== (prev.verify || {}).state) {
    if (st.verify.state === "identical") toast("Verified: the replay is bit-for-bit identical to the live run.", "ok");
    else if (st.verify.state === "different") toast("The replay differs from the live run.", "err");
    else if (st.verify.state === "error") toast(`Verify failed: ${st.verify.message}`, "err");
  }
  if (prev && a.replay_done && !(prev.activity || {}).replay_done && a.identical != null) {
    toast(a.identical ? "Replay finished: identical to the live run." : "Replay finished, but it differs from the live run.", a.identical ? "ok" : "err");
  }

  if (a.scenario !== ui.sensorScenario) { ui.sensorScenario = a.scenario; updateSensorNames(a); }
  syncTruthRun(a);
  renderLog(st.log || []);
  if (st.seq !== ui.lastSeq) { ui.lastSeq = st.seq; refreshRuns(); }
  if (!ui.wsOpen && !wantStream()) setConn("idle", "");
  document.body.classList.toggle("stale", ui.api && !wantStream() && layers.size > 0);
  updateOverlay();
}

function renderActive(st) {
  const a = st.activity || {};
  const state = st.state;
  const kind = { running: "Running Scenario", finished: "All Frames Sent", complete: "Run Complete",
    replaying: a.replay_done ? "Replay Finished" : "Replaying Recording", live: "Live Feed", starting: "Starting" }[state] || state;
  $("nowKind").textContent = kind;
  $("nowTitle").textContent = a.title || "…";
  const rate = a.rate;
  $("nowBadge").textContent = rate ? `${rate}×` : state === "live" ? "LIVE" : "";
  $("nowBadge").className = `badge ${state === "live" ? "err" : "accent"}`;
  const pb = $("nowProgress");
  const full = state === "finished" || a.replay_done;
  pb.classList.toggle("indeterminate", a.progress == null && !full && state !== "complete");
  pb.firstElementChild.style.width = full ? "100%" : a.progress != null ? `${(a.progress * 100).toFixed(1)}%` : "";
  const span = (state === "replaying" && a.recorded_s) || a.duration_s;
  const scenT = a.replay_done ? span : a.elapsed_s != null && rate ? a.elapsed_s * rate : a.elapsed_s || 0;
  $("nowMeta").innerHTML = span
    ? `<span>Scenario time ${fmtTime(Math.min(scenT, span))} / ${fmtTime(span)}</span><span>${fmtTime(a.elapsed_s || 0)} elapsed</span>`
    : `<span>${fmtTime(a.elapsed_s || 0)} elapsed</span>`;
  const chips = [];
  if (a.network) chips.push(`<span class="badge ${a.network === "perfect" ? "" : "warn"}">${esc(NET_TITLE[a.network] || a.network)}</span>`);
  if (a.fault && a.fault !== "none") chips.push(`<span class="badge warn">${esc(FAULT_TITLE[a.fault] || a.fault)}</span>`);
  if (state === "replaying") chips.push(`<span class="badge">From Recording</span>`);
  if (a.replay_done && a.identical != null) chips.push(`<span class="badge ${a.identical ? "ok" : "err"}">${a.identical ? "✓ Identical to Live Run" : "✗ Differs from Live Run"}</span>`);
  $("nowChips").innerHTML = chips.join("");
  const done = state === "complete" || (state === "replaying" && a.replay_done);
  const canReplay = (state === "finished" || done) && a.id && a.scenario !== "adsb";
  $("nowReplay").hidden = !canReplay;
  $("nowReplay").textContent = state === "replaying" ? "Replay Again" : "Replay on Map";
  $("nowVerify").hidden = !canReplay;
  $("nowStop").hidden = done;
  $("nowStop").textContent = state === "finished" ? "Finish Run" : "Stop";
  $("nowNew").hidden = !done;
  $("nowReplay").onclick = () => { clearTracks(); action("/api/replay", { id: a.id, pace: Number($("paceSel").value) }, $("nowReplay")); };
  $("nowVerify").onclick = () => action("/api/verify", { id: a.id }, $("nowVerify"));
  renderWatch(a, scenT);

  const s = st.stats || {};
  const cells = [
    ["Frames Received", s.received ?? s.frames], ["Measurements", s.measurements], ["Live Tracks", s.live_tracks],
    ["Duplicates Dropped", s.duplicates], ["Out of Order", s.out_of_order], ["Rejected", s.rejected != null ? s.rejected + (s.oversize || 0) : null],
  ];
  $("statsGrid").innerHTML = cells.map(([k, v]) => `<div class="stat"><b>${fmtNum(v)}</b><span>${esc(k)}</span></div>`).join("");
  const v = st.verify || {};
  const vr = $("nowVerifyResult");
  vr.hidden = !(v.id && v.id === a.id && v.state);
  if (!vr.hidden) renderVerify(vr, v);
}

function renderVerify(el, v) {
  el.className = "verify-result " + (v.state === "identical" ? "ok" : v.state === "running" ? "" : "err");
  el.textContent = v.state === "running" ? "Replaying the recording and comparing outputs…"
    : v.state === "identical" ? `✓ Bit-for-bit identical: ${fmtNum(v.lines)} output lines replayed in ${v.seconds < 0.1 ? "under 0.1" : v.seconds} s.`
    : v.state === "different" ? "✗ The replay's output differs from the live run."
    : `Verify failed: ${v.message}`;
}

function renderLog(entries) {
  const el = $("log");
  const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 4;
  el.innerHTML = entries.map(e => `<div class="${e.level === "error" ? "error" : ""}"><span class="t">${esc(e.t)}</span>${esc(e.text)}</div>`).join("");
  $("logCount").textContent = String(entries.length);
  if (atBottom) el.scrollTop = el.scrollHeight;
}

// ------------------------------------------------------------------ recordings

async function refreshRuns() {
  if (!ui.api) return;
  let runs;
  try { runs = await api("/api/runs"); } catch { return; }
  const cur = ui.status && ui.status.activity && ui.status.activity.id;
  const v = (ui.status && ui.status.verify) || {};
  $("runsEmpty").hidden = runs.length > 0;
  $("runList").innerHTML = runs.map(r => {
    const when = r.started ? new Date(r.started).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" }) : "";
    const chips = [];
    if (r.speed) chips.push(`<span class="badge">${r.speed}×</span>`);
    if (r.network) chips.push(`<span class="badge ${r.network === "perfect" ? "" : "warn"}">${esc(NET_TITLE[r.network] || r.network)}</span>`);
    if (r.fault && r.fault !== "none") chips.push(`<span class="badge warn">${esc(FAULT_TITLE[r.fault] || r.fault)}</span>`);
    if (r.scenario === "adsb") chips.push(`<span class="badge">ADS-B</span>`);
    if (!r.complete) chips.push(`<span class="badge">${r.id === cur ? "In Progress" : "Incomplete"}</span>`);
    chips.push(`<span class="badge">${fmtNum(r.size_kb)} KB</span>`);
    const vr = v.id === r.id && v.state ? `<div class="verify-result" data-v="${esc(r.id)}"></div>` : "";
    return `<div class="run${r.id === cur ? " cur" : ""}"><div class="run-top"><span class="run-title">${esc(r.title || r.scenario)}</span>` +
      `<span class="run-when">${esc(when)}</span></div><div class="chips">${chips.join("")}</div>` +
      (r.recording ? `<div class="row"><button class="btn btn-secondary btn-sm" data-act="replay" data-id="${esc(r.id)}">▶ Replay on Map</button>` +
        `<button class="btn btn-ghost btn-sm" data-act="verify" data-id="${esc(r.id)}" title="Replay as fast as possible and compare the output with the live run">Verify</button></div>` : "") +
      vr + `</div>`;
  }).join("");
  const el = $("runList").querySelector("[data-v]");
  if (el) renderVerify(el, v);
}
$("runList").addEventListener("click", e => {
  const b = e.target.closest("button[data-act]");
  if (!b) return;
  if (b.dataset.act === "replay") {
    clearTracks();
    action("/api/replay", { id: b.dataset.id, pace: Number($("paceSel").value) }, b).then(st => { if (st) showTab("run"); });
  } else {
    action("/api/verify", { id: b.dataset.id }, b);
  }
});

// ------------------------------------------------------------------ tabs, welcome, quit

function showTab(name) {
  for (const t of document.querySelectorAll(".tab")) t.classList.toggle("active", t.dataset.tab === name);
  for (const p of document.querySelectorAll(".panel")) p.hidden = p.id !== `tab-${name}`;
  if (name === "recordings") refreshRuns();
}
document.querySelector(".tabs").addEventListener("click", e => { const t = e.target.closest(".tab"); if (t) showTab(t.dataset.tab); });

function openWelcome() { $("welcome").hidden = false; }
function closeWelcome() { $("welcome").hidden = true; store.set("sifted.welcomed", "1"); }
$("helpBtn").onclick = openWelcome;
$("welcomeClose").onclick = closeWelcome;
$("welcomeDemo").onclick = () => { closeWelcome(); runDemo(); };
$("welcome").addEventListener("click", e => { if (e.target.id === "welcome") closeWelcome(); });
document.addEventListener("keydown", e => { if (e.key === "Escape" && !$("welcome").hidden) closeWelcome(); });

$("quitBtn").onclick = async () => {
  if (!confirm("Quit Sifted? This stops the fusion service and any run in progress.")) return;
  try { await api("/api/quit", {}); } catch { /* it is going away */ }
  clearInterval(ui.poll);
  showOverlay("Sifted Has Stopped", "You can close this tab. Start Sifted again from its shortcut or with ./sifted.");
  $("overlayBtn").hidden = true;
  $("sessionText").textContent = "Stopped";
  $("sessionDot").className = "dot";
  setConn("offline", "");
  ui.stopped = true;
  if (ws) { ws.onclose = null; ws.close(); }
  clearTimeout(wsTimer);
  // In Sifted's own window (Windows) or an app window this closes it; a normal
  // browser tab ignores it and keeps the "stopped" message.
  setTimeout(() => window.close(), 400);
};

// ------------------------------------------------------------------ start-up

async function poll() {
  if (ui.stopped) return;
  try {
    applyStatus(await api("/api/status"));
    ui.pollFailures = 0;
  } catch {
    ui.pollFailures = (ui.pollFailures || 0) + 1;
    if (ui.pollFailures === 3) {
      $("sessionText").textContent = "Lost contact with Sifted";
      $("sessionDot").className = "dot err";
      toast("Lost contact with the Sifted launcher. Is its window still open?", "err");
    }
  }
}

// Tell the launcher when this window closes: started from the desktop shortcut,
// Sifted stops once its last window is gone. (A reload polls again at once.)
window.addEventListener("pagehide", () => {
  if (ui.api && !ui.stopped && navigator.sendBeacon) navigator.sendBeacon("/api/bye", "");
});

async function init() {
  let cfg = null;
  try { cfg = await api("/api/config"); } catch { /* standalone */ }
  if (cfg && cfg.ws_url) {
    ui.api = true;
    ui.config = cfg;
    if (!params.get("ws")) ui.wsUrl = cfg.ws_url.replace("{host}", location.hostname || "127.0.0.1");
    $("quitBtn").hidden = false;
    $("notBuilt").hidden = cfg.built;
    $("runBtn").disabled = !cfg.built;
    fillSelect($("networkSel"), cfg.networks, $("networkHint"), "sifted.network");
    fillSelect($("faultSel"), cfg.faults, $("faultHint"), "sifted.fault");
    setBbox(cfg.adsb_bbox);
    setSpeed(ui.speed);
    try { ui.scenarios = await api("/api/scenarios"); } catch (e) { toast(`Couldn't load scenarios: ${e.message}`, "err"); }
    renderScenarios();
    await poll();
    ui.poll = setInterval(poll, 1000);
    if (!store.get("sifted.welcomed")) openWelcome();
  } else {
    // Standalone: a read-only view of whatever daemon is at ui.wsUrl.
    $("standalone").hidden = false;
    $("wsUrlText").textContent = ui.wsUrl;
    $("setup").hidden = true;
    $("tabRecBtn").hidden = true;
    $("session").style.visibility = "hidden";
    $("activity").hidden = true;
    $("welcomeDemo").hidden = true;
    $("welcomeClose").textContent = "Got It";
    showTab("tracks");
    updateOverlay();
  }
  connect();
}
init();
