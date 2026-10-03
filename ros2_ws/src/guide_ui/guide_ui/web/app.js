"use strict";
const $ = (id) => document.getElementById(id);
const state = { dest: [], trip: null, route: [], floor: null, floorData: {}, paused: false };
const ACTIVE = new Set(["routing", "driving", "waiting_for_student", "waiting_for_help", "lost"]);

const CONFIRM_LABELS = {
  elevator_call: "The doors are open",
  elevator_ride: "We're there, doors open",
  door: "I'm holding the door",
  blocked: "Try again",
  still_with_me: "Yes, I'm here",
};
const TITLES = {
  idle: "Ready", routing: "Planning the route", driving: "Follow me",
  waiting_for_student: "Waiting for you", waiting_for_help: "I need your help",
  lost: "Finding where I am", arrived: "Arrived", aborted: "Trip ended",
};

async function main() {
  const cfg = await (await fetch("api/config")).json();
  state.dest = await (await fetch("api/destinations")).json();
  const ros = new Ros(`ws://${location.hostname}:${cfg.rosbridge_port}`);
  ros.onstatus = (on) => { $("conn").textContent = on ? "connected" : "reconnecting"; $("conn").className = "pill " + (on ? "on" : "off"); };

  // Follow-aware: heartbeat every second while this page is open (spec section 9).
  setInterval(() => { if (!document.hidden) ros.publish("/guide/heartbeat", "std_msgs/msg/Empty", {}); }, cfg.heartbeat_s * 1000);

  ros.subscribe("/guide/state", "std_msgs/msg/String", (m) => onState(JSON.parse(m.data)));
  ros.subscribe("/guide/speech", "std_msgs/msg/String", (m) => say(m.data));
  ros.subscribe("/guide/route", "std_msgs/msg/String", (m) => { state.route = JSON.parse(m.data); draw(); });

  $("q").addEventListener("input", () => renderResults($("q").value));
  $("confirm").onclick = () => ros.publish("/guide/confirm", "std_msgs/msg/Empty", {});
  // The mission's own paused flag decides what the button does, so the phone,
  // the voice agent and a reloaded page always agree.
  $("pause").onclick = () => ros.publish("/guide/pause", "std_msgs/msg/Bool", { data: !state.paused });
  $("cancel").onclick = () => {
    if (state.trip && ACTIVE.has(state.trip.state)) ros.publish("/guide/cancel", "std_msgs/msg/Empty", {});
    showSearch();
  };
  state.go = (key) => {
    unlockSpeech();
    ros.publish("/guide/destination", "std_msgs/msg/String", { data: key });
    state.route = [];
    $("search-view").hidden = true; $("trip-view").hidden = false;
  };
  renderResults("");
}

function normalise(s) { return s.toUpperCase().replace(/[\s-]+/g, ""); }

function renderResults(q) {
  const n = normalise(q);
  const list = $("results");
  list.innerHTML = "";
  if (!n) return;
  const hits = state.dest.filter((d) => normalise(d.key).includes(n) || (d.name && d.name.toUpperCase().includes(q.toUpperCase()))).slice(0, 12);
  for (const d of hits) {
    const li = document.createElement("li");
    const b = document.createElement("button");
    b.textContent = d.key;
    const small = document.createElement("small");
    small.textContent = [d.type, d.name, d.floor].filter(Boolean).join(" · ");
    b.appendChild(small);
    b.onclick = () => state.go(d.key);
    li.appendChild(b); list.appendChild(li);
  }
}

function showSearch() { $("trip-view").hidden = true; $("search-view").hidden = false; $("q").value = ""; renderResults(""); }

function onState(s) {
  state.trip = s;
  // Show the trip while one is under way; a finished trip stays on screen
  // until the student taps "New trip", and never pulls them out of search.
  if (ACTIVE.has(s.state) && $("trip-view").hidden) { $("search-view").hidden = true; $("trip-view").hidden = false; }
  state.paused = !!s.paused;
  $("pause").textContent = state.paused ? "Resume" : "Pause";
  $("pause").hidden = !ACTIVE.has(s.state);
  $("cancel").textContent = ACTIVE.has(s.state) ? "End trip" : "New trip";
  $("status-title").textContent = TITLES[s.state] || s.state;
  $("status-detail").textContent = s.destination ? `To ${s.destination}${s.detail && s.detail !== s.destination ? " · " + s.detail.replaceAll("_", " ") : ""}` : "";
  $("status").className = "status " + ({ waiting_for_student: "waiting", waiting_for_help: "waiting", lost: "waiting", arrived: "done", aborted: "bad" }[s.state] || "");
  const label = CONFIRM_LABELS[s.detail];
  const needsConfirm = (s.state === "waiting_for_help" && label) || (s.state === "waiting_for_student" && s.detail === "still_with_me");
  $("confirm").hidden = !needsConfirm;
  if (label) $("confirm").textContent = label;
  if (s.floor !== state.floor) { state.floor = s.floor; loadFloor(s.floor); }
  state.pose = s.pose;
  draw();
}

async function loadFloor(f) {
  if (!f || state.floorData[f]) { draw(); return; }
  const r = await fetch(`api/floor/${encodeURIComponent(f)}`);
  if (r.ok) state.floorData[f] = await r.json();
  draw();
}

function draw() {
  const c = $("map"), g = c.getContext("2d");
  const css = getComputedStyle(document.documentElement);
  g.clearRect(0, 0, c.width, c.height);
  const fd = state.floorData[state.floor];
  let [x0, y0, x1, y1] = fd ? fd.bounds : [0, 0, 1, 1];
  const pts = state.route.filter((n) => n.floor === state.floor);
  if (!fd && pts.length) { x0 = Math.min(...pts.map((p) => p.x)); x1 = Math.max(...pts.map((p) => p.x)); y0 = Math.min(...pts.map((p) => p.y)); y1 = Math.max(...pts.map((p) => p.y)); }
  const pad = 20, s = Math.min((c.width - 2 * pad) / Math.max(x1 - x0, 1), (c.height - 2 * pad) / Math.max(y1 - y0, 1));
  const X = (x) => pad + (x - x0) * s, Y = (y) => c.height - pad - (y - y0) * s;
  if (fd) {
    g.strokeStyle = css.getPropertyValue("--corridor"); g.lineWidth = 6; g.lineCap = "round";
    g.beginPath(); for (const [a, b, cc, d] of fd.edges) { g.moveTo(X(a), Y(b)); g.lineTo(X(cc), Y(d)); } g.stroke();
  }
  if (pts.length > 1) {
    g.strokeStyle = css.getPropertyValue("--route"); g.lineWidth = 4;
    g.beginPath(); pts.forEach((p, i) => (i ? g.lineTo(X(p.x), Y(p.y)) : g.moveTo(X(p.x), Y(p.y)))); g.stroke();
    const end = pts[pts.length - 1];
    g.fillStyle = css.getPropertyValue("--route"); g.beginPath(); g.arc(X(end.x), Y(end.y), 7, 0, 7); g.fill();
  }
  if (state.pose) {
    const [px, py, yaw] = state.pose;
    g.fillStyle = css.getPropertyValue("--car");
    g.beginPath(); g.arc(X(px), Y(py), 8, 0, 7); g.fill();
    g.strokeStyle = g.fillStyle; g.lineWidth = 3;
    g.beginPath(); g.moveTo(X(px), Y(py)); g.lineTo(X(px) + 16 * Math.cos(yaw), Y(py) - 16 * Math.sin(yaw)); g.stroke();
  }
  if (state.floor && fd) { g.fillStyle = css.getPropertyValue("--muted"); g.font = "16px system-ui"; g.fillText(fd.name, 10, 20); }
}

// Spoken instructions with the browser's speech synthesis (spec section 9).
let speechUnlocked = false;
function unlockSpeech() {
  if (speechUnlocked || !("speechSynthesis" in window)) return;
  speechSynthesis.speak(new SpeechSynthesisUtterance(""));
  speechUnlocked = true;
}
function say(text) {
  $("said").textContent = text;
  if ("speechSynthesis" in window) { speechSynthesis.cancel(); speechSynthesis.speak(new SpeechSynthesisUtterance(text)); }
}

main().catch((e) => { $("conn").textContent = "error"; console.error(e); });
