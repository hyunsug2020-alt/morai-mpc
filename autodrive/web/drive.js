"use strict";
const profile = document.body.dataset.profile;
const isCompetition = profile === "two_pc";
const title = isCompetition ? "2 PC 대회용" : "1 PC용";
const names = {front:"전방 카메라",left:"좌측 카메라",right:"우측 카메라",front_low:"보조 카메라",gps:"GPS",imu:"IMU",lidar:"LiDAR · 설정 15 Hz",status:"차량 상태",collision:"충돌 정보"};
const $ = id => document.getElementById(id);
const fmt = (value, digits=1) => typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "–";
let latest = null, sending = false, webReady = false;
document.title = `MORAI · ${title}`;
$("pageTitle").textContent = `${title} 주행 관제`;
$("launchTitle").textContent = `${title} 실행`;
$("start").textContent = `${title} 주행 시작`;
$("addressLine").textContent = isCompetition ? "MORAI PC 192.168.0.1 → 주행 PC 192.168.0.10" : "이 컴퓨터에서 MORAI + 주행 · 127.0.0.1";
$("setupNote").textContent = isCompetition ? "MORAI PC에서 대회용 센서·네트워크 JSON 각각 Load → Connect → Play 후 주행 시작." : "이 컴퓨터에서 MORAI 맵·시나리오를 열고 주행 시작. 싱글 PC용 센서·네트워크 사용.";
$("collect").hidden = $("labLink").hidden = isCompetition;
$("scenarioLink").href = `http://${location.hostname}:4173/`;
document.querySelector(`[data-page="${profile}"]`).setAttribute("aria-current", "page");
for (const kind of ["network", "sensor"]) {
  const a = document.createElement("a");
  a.href = `/api/preset?profile=${profile}&kind=${kind}`;
  a.textContent = `${title} ${kind === "network" ? "네트워크" : "센서"} JSON 다운로드`;
  $("downloads").append(a);
}
for (const [key, label] of Object.entries(names)) {
  const card = document.createElement("div"); card.className = "sensor"; card.dataset.sensor = key;
  const strong = document.createElement("strong"); strong.textContent = label;
  const small = document.createElement("small"); small.textContent = "수신 확인 중";
  card.append(strong, small); $("sensors").append(card);
}
function message(text, error=false) { $("message").textContent=text; $("message").classList.toggle("error",error); }
function buttons() {
  const busy = sending || !webReady || !!latest?.busy;
  $("start").disabled = $("collect").disabled = busy;
  $("signalEnabled").disabled = busy;
  $("stop").disabled = sending || !latest?.job?.running;
}
function render(state) {
  latest = state;
  const streams = state.sensor_input?.streams || {}, ros = state.ros || {};
  const received = Object.values(streams).filter(s=>s.connected).length;
  const driving = state.monitor_mode === "driving";
  const connected = received > 0 || (driving && ros.ego_hz > 0);
  $("connection").textContent = driving ? (ros.ego_hz > 0 ? `${title} 주행 · 측위 ${fmt(ros.ego_hz)} Hz` : "주행 입력 준비 중") : received ? `${title} · ${received}/${Object.keys(names).length} 입력 수신` : "MORAI 입력 수신 대기";
  $("connection").className = `connection ${connected ? "ok" : "wait"}`;
  for (const card of document.querySelectorAll("[data-sensor]")) {
    const key = card.dataset.sensor, input = streams[key];
    card.classList.toggle("ok", !!input?.connected);
    card.querySelector("small").textContent = driving ? "주행 프로세스에서 수신" : input?.connected ? (input.hz == null ? "수신 중" : `${fmt(input.hz)} Hz · 수신 중`) : input?.error ? "주소·포트 확인 필요" : "수신 대기";
  }
  $("inputNote").textContent = driving ? "주행 프로세스가 센서 수신을 담당하고 있음." : `${state.profile.simulator_ip}에서 ${state.profile.local_ip}로 오는 실시간 입력만 표시.`;
  $("speed").textContent = ros.ego ? `${fmt(ros.ego.speed_kmh)} km/h` : "–";
  $("progress").textContent = ros.path ? `${fmt(ros.path.progress_pct)}%` : "–";
  $("poseHz").textContent = ros.ego_hz > 0 ? `${fmt(ros.ego_hz)} Hz` : "–";
  $("controlHz").textContent = ros.ctrl_hz > 0 ? `${fmt(ros.ctrl_hz)} Hz` : "–";
  $("driveNote").textContent = driving ? "현재 실행의 측위·주행 데이터 표시." : "주행 시작 시 해당 구성의 주행 상태 표시.";
  const signal = state.traffic_signal || {};
  if (driving && typeof state.signal_enabled === "boolean") {
    $("signalStatus").textContent = !state.signal_enabled ? "신호등 미사용" : signal.active ?
      `신호등 ${signal.raw_state} · ${signal.phase} · 정지선 ${fmt(signal.distance_m)} m · ${signal.fresh ? "최신 영상" : "인지 대기"}` : "신호등 사용 · 다음 신호 구간 대기";
  } else {
    $("signalStatus").textContent = $("signalEnabled").checked ? "신호등 사용 · 시작 전 선택 가능" : "신호등 미사용 · 시작 전 선택 가능";
  }
  $("busyNote").hidden = !state.other_job;
  if (state.other_job) $("busyNote").textContent = `${state.other_job.profile === "single_pc" ? "1 PC용" : state.other_job.profile === "two_pc" ? "2 PC 대회용" : "다른"} 작업 실행 중. 해당 페이지에서 중지한 뒤 실행 가능.`;
  const job = state.job;
  $("jobTitle").textContent = job.running ? `${title} 실행 중` : job.action ? `${title} 종료 · 코드 ${job.exit_code ?? "–"}` : `${title} 실행 대기`;
  $("jobTime").textContent = job.running ? `${Math.floor(job.elapsed_seconds/60)}분 ${Math.floor(job.elapsed_seconds%60)}초` : "";
  const logs = $("logs"), bottom = logs.scrollTop + logs.clientHeight >= logs.scrollHeight - 10;
  logs.textContent = job.logs?.length ? job.logs.join("\n").replace(/\x1b\[[0-9;]*m/g, "") : "이 구성의 주행 로그가 여기에 표시됨.";
  if (bottom) logs.scrollTop = logs.scrollHeight;
  buttons(); images();
}
function images() {
  for (const card of document.querySelectorAll(".camera")) {
    const view = card.dataset.view, img = card.querySelector("img"), placeholder = card.querySelector(".placeholder");
    if (!latest?.camera_available?.[view]) { img.hidden=true; placeholder.hidden=false; continue; }
    img.onload = () => { img.hidden=false; placeholder.hidden=true; };
    img.onerror = () => { img.hidden=true; placeholder.hidden=false; };
    img.src = `/api/drive-camera?profile=${profile}&view=${view}&t=${Date.now()}`;
  }
}
async function refresh() {
  try {
    const response = await fetch(`/api/drive-status?profile=${profile}`, {cache:"no-store"});
    const state = await response.json(); if (!response.ok) throw new Error(state.error || "상태 수신 실패");
    webReady=true; render(state);
  } catch (error) {
    webReady=false; buttons();
    $("connection").textContent = "웹 상태 수신 실패 · 재연결 중"; $("connection").className="connection wait";
    for (const card of document.querySelectorAll(".camera")) { card.querySelector("img").hidden=true; card.querySelector(".placeholder").hidden=false; }
  }
  setTimeout(refresh, 1200);
}
async function action(endpoint, collect=false) {
  sending=true; buttons(); message(endpoint === "start" ? `${title} 시작 요청 중` : "중지 요청 중");
  try {
    const response = await fetch(`/api/drive/${endpoint}`, {method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({profile,collect,options:{signal_enabled:$("signalEnabled").checked}})});
    const result = await response.json(); if (!response.ok) throw new Error(result.error || "실행 요청 실패");
    message(endpoint === "start" ? `${title} 실행 요청 완료` : "중지 요청 완료");
  } catch (error) { message(error.message,true); }
  finally { sending=false; buttons(); }
}
$("start").onclick = () => action("start");
$("collect").onclick = () => action("start",true);
$("stop").onclick = () => action("stop");
$("signalEnabled").onchange = () => { if (latest) render(latest); };
buttons();
refresh();
