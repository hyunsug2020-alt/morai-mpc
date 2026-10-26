"use strict";
const profile = document.body.dataset.profile;
const $ = id => document.getElementById(id);
const title = profile === "two_pc" ? "2 PC 대회용" : "1 PC용";
const views = ["original", "semantic", "crop", "red", "yellow", "green"];
const names = {UNKNOWN:"미확인", RED:"빨강", YELLOW:"노랑", GREEN:"초록", LEFT:"좌회전", RED_LEFT:"빨강 + 좌회전", GREEN_LEFT:"초록 + 좌회전"};
const phases = {BRAKE:"감속", HOLD:"정지 유지", ALLOW:"신호 통과 허용"};
const policies = {green:"초록 통과", red_left_only:"빨강+좌회전만 통과", ignore:"우회전 · 신호 무시"};
const fmt = value => typeof value === "number" && Number.isFinite(value) ? value.toFixed(1) : "–";
let latest = null, sending = false, connected = false, shownFrame = null, expiry = null;
document.title = `MORAI · ${title} 신호등 HSV`;
$("pageTitle").textContent = `${title} 신호등 · HSV`;
$("driveLink").href = profile === "two_pc" ? "/competition" : "/single-pc";
document.querySelector(`[data-profile-link="${profile}"]`).setAttribute("aria-current", "page");

function controls() {
  $("start").disabled = $("signalEnabled").disabled = sending || !connected || !!latest?.busy;
  $("stop").disabled = sending || !latest?.running;
}

function clearFrame() {
  clearTimeout(expiry); shownFrame = null;
  for (const view of views) { $(view).hidden = true; $(view).nextElementSibling.hidden = false; }
  $("rawState").textContent = $("stableState").textContent = "–";
  for (const slot of document.querySelectorAll("[data-slot]")) { slot.classList.toggle("on", false); slot.querySelector("small").textContent = "–"; }
  $("frameInfo").textContent = "실시간 영상 갱신 대기";
  if (latest?.available) { $("connection").textContent = "영상 갱신 대기"; $("connection").className = "badge"; }
}

function imageAt(url) {
  return new Promise((resolve, reject) => {
    const image = new Image();
    const timeout = setTimeout(() => { image.src = ""; reject(new Error("영상 수신 시간 초과")); }, 1200);
    image.onload = () => { clearTimeout(timeout); resolve(image); };
    image.onerror = () => { clearTimeout(timeout); reject(new Error("영상 갱신 대기")); };
    image.src = url;
  });
}

async function showFrame(state, requestStarted) {
  const perception = state.perception;
  if (!state.available || !perception) { clearFrame(); return; }
  if (shownFrame === perception.frame_id) return;
  const images = await Promise.all(views.map(view => imageAt(`/api/signal/image?${new URLSearchParams({profile,view,frame:perception.frame_id,slot:perception.slot})}`)));
  // Use the server's receipt age plus local elapsed time. The browser may be
  // on the other PC, whose wall clock is not synchronized with the drive PC.
  const remaining = (perception.config.stale_seconds-state.frame_age_s)*1000-(performance.now()-requestStarted);
  if (remaining <= 0) { clearFrame(); return; }
  clearTimeout(expiry);
  views.forEach((view, i) => { $(view).src = images[i].src; $(view).hidden = false; $(view).nextElementSibling.hidden = true; });
  shownFrame = perception.frame_id;
  expiry = setTimeout(clearFrame, remaining);
  $("rawState").textContent = names[perception.raw_state] || "미확인";
  $("stableState").textContent = `${names[perception.state] || "미확인"} · ${perception.stable_count}/${perception.config.stable_frames}`;
  for (const element of document.querySelectorAll("[data-slot]")) {
    const slot = perception.slots?.[element.dataset.slot];
    element.classList.toggle("on", !!slot?.on);
    element.querySelector("small").textContent = slot ? `${slot.largest_blob} / ${slot.minimum} px · ${slot.on ? "점등" : "꺼짐"}` : "해당 등 없음";
  }
  const config = perception.config;
  $("redRange").textContent = `H ${config.red_h[0]}–${config.red_h[1]} / ${config.red_h[2]}–${config.red_h[3]}`;
  $("yellowRange").textContent = `H ${config.yellow_h.join("–")}`;
  $("greenRange").textContent = `H ${config.green_h.join("–")}`;
  $("thresholds").textContent = `S ≥ ${config.s_min} · V ≥ ${config.v_min} · ${config.layout === "four" ? "4등식" : "3등식"}`;
  $("detectionReason").textContent = perception.reason || "판정 정보 대기";
  $("frameInfo").textContent = `${perception.camera_id}번 카메라 · 수신 후 ${fmt(state.frame_age_s)}초 · ${perception.target_box ? `같은 영역·색상 연속 ${perception.stable_count}프레임` : "선택 영역 없음"}`;
}

async function refresh() {
  const requestStarted = performance.now();
  try {
    const response = await fetch(`/api/signal/status?profile=${profile}`, {cache:"no-store"});
    const state = await response.json();
    if (!response.ok) throw new Error(state.error || "신호등 상태 수신 실패");
    connected = true; latest = state;
    if (state.running && typeof state.signal_enabled === "boolean") $("signalEnabled").checked = state.signal_enabled;
    $("settingNote").textContent = state.running ? `현재 주행: 신호등 ${state.signal_enabled ? "사용" : "미사용"}` : "신호등 사용 여부는 주행 시작 전에 선택";
    $("connection").textContent = state.reason; $("connection").className = `badge ${state.available ? "ok" : ""}`;
    const control = state.control || {};
    $("phase").textContent = control.active ? phases[control.phase] || "판정 대기" : "대기";
    $("distance").textContent = control.active ? `${fmt(control.distance_m)} m` : "–";
    $("lineInfo").textContent = control.permission_policy === "ignore" ? policies.ignore : control.active ?
      `${control.signal_index ? `${control.signal_index}번째 · ` : ""}${policies[control.permission_policy] || control.maneuver || "–"} · 정지선 ${control.stop_line_node_id || "–"}` : "선택된 신호 정지선 없음";
    $("controlReason").textContent = control.reason || "신호 제어 대기";
    controls();
    try { await showFrame(state, requestStarted); } catch (_) { clearFrame(); }
  } catch (error) {
    connected = false; clearFrame(); controls();
    $("connection").textContent = error.message; $("connection").className = "badge";
    $("phase").textContent = $("distance").textContent = "–";
  }
  setTimeout(refresh, 350);
}

async function action(endpoint) {
  sending = true; controls();
  $("message").classList.toggle("error", false);
  $("message").textContent = endpoint === "start" ? "주행 시작 요청 중" : "주행 중지 요청 중";
  try {
    const response = await fetch(`/api/drive/${endpoint}`, {method:"POST", headers:{"Content-Type":"application/json"},
      body:JSON.stringify({profile,options:{signal_enabled:$("signalEnabled").checked}})});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "주행 요청 실패");
    $("message").textContent = endpoint === "start" ? "선택한 신호 설정으로 주행 시작 요청 완료" : "주행 중지 요청 완료";
  } catch (error) {
    $("message").classList.toggle("error", true); $("message").textContent = error.message;
  } finally { sending = false; controls(); }
}
$("start").onclick = () => action("start");
$("stop").onclick = () => action("stop");
controls(); refresh();
