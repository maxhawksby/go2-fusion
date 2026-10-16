import { dashboardApi } from "./dashboard-api.js";
import { parseCommand } from "./command-parser.js";

const state = {
  mode: "mock", connected: false, commandEnabled: false, moving: null, log: [],
  cameraOnline: false, cameraFrameReady: false, cameraSource: null,
  obstacleAvoidance: null, obstaclePending: false,
};
const motionSpeeds = { forward: 0.2, back: -0.15, left: 0, right: 0 };
const motionLabels = { forward: "Moving forward", back: "Moving backward", left: "Turning left", right: "Turning right" };
const OBSTACLE_CONTROL_ENABLED = false;
const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const clock = $("#clock");
const logList = $("#command-log");
const interpretation = $("#interpretation");

function updateClock() {
  clock.textContent = new Date().toLocaleTimeString([], { hour12: false });
}
updateClock();
setInterval(updateClock, 1000);

function escapeHtml(value) {
  const node = document.createElement("div");
  node.textContent = value;
  return node.innerHTML;
}

function addLog(kind, message) {
  state.log.unshift({ kind, message, time: new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }) });
  state.log = state.log.slice(0, 8);
  logList.innerHTML = state.log.map((entry) => `
    <li><span class="log-kind ${entry.kind}">${entry.kind}</span><p>${escapeHtml(entry.message)}</p><time>${entry.time}</time></li>
  `).join("");
}

function showInterpretation(title, detail) {
  interpretation.innerHTML = `<strong>${escapeHtml(title)}</strong><span>${escapeHtml(detail)}</span>`;
}

function setSensor(name, online, onlineText = "Online", offlineText = "Offline") {
  const element = $(`[data-sensor="${name}"]`);
  if (!element) return;
  element.textContent = online ? onlineText : offlineText;
  element.closest(".metric").classList.toggle("online", online);
}

function renderObjects(objects) {
  const list = $("#object-list");
  if (!objects.length) {
    list.innerHTML = "<p>No remembered objects. Start the detector and put an object in view.</p>";
    return;
  }
  list.innerHTML = objects.slice(0, 8).map((object) => `
    <div class="object-chip"><strong>${escapeHtml(object.label)}</strong><small>${object.distance.toFixed(2)} m · ${object.bearing_deg.toFixed(1)}°</small></div>
  `).join("");
}

function renderObstacleControl(status = {}) {
  const toggle = $("#obstacle-toggle");
  const enabled = typeof status.enabled === "boolean" ? status.enabled : null;
  state.obstacleAvoidance = enabled;
  state.obstaclePending = Boolean(status.pending);
  toggle.classList.toggle("on", enabled === true);
  toggle.setAttribute("aria-checked", enabled === true ? "true" : "false");
  toggle.querySelector("b").textContent = !OBSTACLE_CONTROL_ENABLED ? "Later" : enabled === null ? "--" : enabled ? "On" : "Off";
  toggle.disabled = !OBSTACLE_CONTROL_ENABLED || state.mode !== "robot" || !state.commandEnabled || enabled === null || state.obstaclePending;
  $("#obstacle-status").textContent = !OBSTACLE_CONTROL_ENABLED
    ? "Unavailable while the team validates movement control"
    : status.error
    ? `Driver error: ${status.error}`
    : state.obstaclePending ? "Applying setting..."
      : enabled === null ? "Waiting for the robot driver..."
        : enabled ? "Used by later ROS movement commands" : "ROS movement commands use direct velocity";
}

function resetCamera(message = "Switch to Robot Mode to load the live feed.") {
  const feed = $("#camera-feed");
  feed.hidden = true;
  feed.removeAttribute("src");
  $("#camera-placeholder").hidden = false;
  $("#camera-placeholder span").textContent = message;
  $("#camera-source").textContent = "CAMERA OFFLINE";
}

async function refreshRobotState({ announce = false } = {}) {
  if (state.mode !== "robot") return;
  try {
    const [status, detection] = await Promise.all([dashboardApi.status(), dashboardApi.objects()]);
    state.connected = status.robot_connected;
    state.commandEnabled = status.command_enabled;
    state.cameraOnline = status.topics.camera;
    state.cameraFrameReady = Boolean(status.camera?.frame_ready);
    state.cameraSource = status.camera?.source || null;
    if (!state.cameraOnline) resetCamera("The camera topic is offline.");
    else if (!state.cameraFrameReady) resetCamera("Waiting for the bridge to encode a camera frame...");
    $("#connection-label").textContent = status.robot_connected ? "Robot heartbeat active" : "Bridge online · robot offline";
    $("#simulate-connect").textContent = "Refresh ROS";
    $(".robot-card").classList.toggle("connected", status.robot_connected);
    setSensor("camera", status.topics.camera, "Publishing", "Offline");
    setSensor("lidar", status.topics.lidar, "Publishing", "Offline");
    setSensor("imu", status.topics.imu, "Publishing", "Offline");
    setSensor("objects", status.topics.objects, `${detection.objects.length} detected`, "Detector offline");
    renderObjects(detection.objects);
    renderObstacleControl(status.obstacle_avoidance);
    const safety = status.command_enabled ? "Live actions enabled · movement disabled" : "Read-only Robot Mode · commands disabled";
    $("#safety-status").innerHTML = `<i class="safety-dot"></i> ${safety}`;
    if (announce) addLog("system", status.robot_connected ? "ROS bridge and robot heartbeat found" : "ROS bridge found; waiting for robot heartbeat");
  } catch (error) {
    state.connected = false;
    state.commandEnabled = false;
    state.cameraOnline = false;
    state.cameraFrameReady = false;
    $("#connection-label").textContent = "Dashboard bridge offline";
    $("#simulate-connect").textContent = "Retry bridge";
    $(".robot-card").classList.remove("connected");
    ["camera", "lidar", "imu", "objects"].forEach((name) => setSensor(name, false));
    $("#object-list").innerHTML = "<p>Start go2_dashboard_bridge inside the ROS container.</p>";
    renderObstacleControl();
    resetCamera("Start the dashboard bridge to load the camera.");
    $("#safety-status").innerHTML = '<i class="safety-dot"></i> Robot publishing unavailable';
    if (announce) addLog("unknown", error.message);
  }
}

function setMode(mode) {
  state.mode = mode;
  state.connected = false;
  state.commandEnabled = false;
  stopMotion("mode change", false);
  const robotMode = mode === "robot";
  $("#mode-toggle").classList.toggle("robot", robotMode);
  $("#mode-toggle span").textContent = robotMode ? "Robot mode" : "Mock mode";
  $(".robot-card .tag").textContent = robotMode ? "ROS BRIDGE" : "SIMULATED";
  if (robotMode) {
    $("#connection-label").textContent = "Checking dashboard bridge…";
    $("#simulate-connect").textContent = "Refresh ROS";
    addLog("system", "Robot Mode selected; movement remains disabled");
    refreshRobotState({ announce: true });
  } else {
    $("#connection-label").textContent = "Offline simulator";
    $("#simulate-connect").textContent = "Start simulation";
    $(".robot-card").classList.remove("connected");
    setSensor("camera", false, "Simulated", "Standby");
    setSensor("lidar", false, "Simulated", "Standby");
    setSensor("imu", false, "Simulated", "Standby");
    setSensor("objects", false, "Demo loaded", "Detector offline");
    $("#object-list").innerHTML = "<p>Switch to Robot Mode to read detections.</p>";
    renderObstacleControl();
    resetCamera();
    $("#safety-status").innerHTML = '<i class="safety-dot"></i> Robot publishing disabled';
    addLog("system", "Mock Mode selected");
  }
}

$("#mode-toggle").addEventListener("click", () => setMode(state.mode === "mock" ? "robot" : "mock"));
setInterval(() => refreshRobotState(), 1500);

const cameraFeed = $("#camera-feed");
cameraFeed.addEventListener("load", () => {
  cameraFeed.hidden = false;
  $("#camera-placeholder").hidden = true;
  $("#camera-source").textContent = state.cameraSource === "annotated" ? "LIVE · OBJECT OVERLAY" : "LIVE · RAW CAMERA";
});
cameraFeed.addEventListener("error", () => {
  cameraFeed.hidden = true;
  $("#camera-placeholder").hidden = false;
  $("#camera-placeholder span").textContent = "Waiting for a camera frame...";
});
setInterval(() => {
  if (state.mode !== "robot" || !state.cameraOnline || !state.cameraFrameReady) return;
  cameraFeed.src = dashboardApi.cameraUrl();
}, 300);

$("#simulate-connect").addEventListener("click", () => {
  if (state.mode === "robot") {
    refreshRobotState({ announce: true });
    return;
  }
  state.connected = !state.connected;
  $("#simulate-connect").textContent = state.connected ? "Stop simulation" : "Start simulation";
  $("#connection-label").textContent = state.connected ? "Simulator online" : "Offline simulator";
  $(".robot-card").classList.toggle("connected", state.connected);
  setSensor("camera", state.connected, "Simulated", "Standby");
  setSensor("lidar", state.connected, "Simulated", "Standby");
  setSensor("imu", state.connected, "Simulated", "Standby");
  setSensor("objects", state.connected, "Demo loaded", "Detector offline");
  addLog("system", state.connected ? "Mock simulation started" : "Mock simulation stopped");
});

function setMotion(direction, source = "control") {
  if (state.mode === "robot") {
    showInterpretation("Live movement disabled", "WASD needs its own twist-mux lane and a robot test.");
    if (state.moving !== "blocked") addLog("unknown", `Blocked live ${direction} request (${source})`);
    state.moving = "blocked";
    return;
  }
  if (state.moving === direction) return;
  state.moving = direction;
  $$("[data-motion]").forEach((button) => button.classList.toggle("active", button.dataset.motion === direction));
  $("#velocity-value").textContent = Math.abs(motionSpeeds[direction]).toFixed(2);
  $("#motion-label").textContent = motionLabels[direction];
  addLog("motion", `${motionLabels[direction]} (${source}, mock only)`);
}

function stopMotion(source = "control", log = true) {
  if (!state.moving && source !== "stop") return;
  state.moving = null;
  $$("[data-motion]").forEach((button) => button.classList.remove("active"));
  $("#velocity-value").textContent = "0.00";
  $("#motion-label").textContent = "Standing by";
  if (log) addLog("stop", `Stop (${source}${state.mode === "mock" ? ", mock only" : ""})`);
}

async function performAction(action, label, source) {
  showInterpretation(label, state.mode === "mock" ? `Recognized from ${source}; execution disabled` : `Sending from ${source}…`);
  if (state.mode === "mock") {
    addLog("action", `${label} (${source}, dry run)`);
    return;
  }
  try {
    await dashboardApi.action(action);
    showInterpretation(label, "Queued through ROS");
    addLog("action", `${label} queued through ROS`);
  } catch (error) {
    showInterpretation("Command blocked", error.message);
    addLog("unknown", error.message);
  }
}

async function interpret(raw, source = "typed") {
  if (state.mode === "robot") {
    showInterpretation("Interpreting command", "Sending to the ROS bridge…");
    try {
      const result = await dashboardApi.command(raw);
      const label = result.label || (result.type === "tts" ? `Say “${result.text}”` : "Command accepted");
      showInterpretation(label, "Queued through ROS");
      addLog(result.type === "tts" ? "speech" : "action", `${label} (${source})`);
    } catch (error) {
      showInterpretation("Command blocked", error.message);
      addLog("unknown", error.message);
    }
    return;
  }

  const command = parseCommand(raw);
  if (command.type === "unknown") {
    showInterpretation("Not recognized", `“${raw}”`);
    addLog("unknown", `Could not map “${raw}”`);
    return;
  }
  if (command.type === "motion") {
    if (state.mode === "robot") {
      showInterpretation("Live movement disabled", "Movement needs its own reviewed control lane.");
      addLog("unknown", `Blocked ${command.label.toLowerCase()} voice request`);
    } else {
      addLog("motion", `${command.label} (${source}, dry run)`);
      showInterpretation(command.label, `Recognized from ${source}; execution disabled`);
    }
    return;
  }
  if (command.type === "tts") {
    showInterpretation(command.label, state.mode === "mock" ? "Speech preview only" : "Sending to /tts…");
    if (state.mode === "mock") {
      addLog("action", `${command.label} (${source}, dry run)`);
      return;
    }
    try {
      await dashboardApi.tts(command.value);
      showInterpretation(command.label, "Queued on /tts");
      addLog("action", `${command.label} queued on /tts`);
    } catch (error) {
      showInterpretation("Speech blocked", error.message);
      addLog("unknown", error.message);
    }
    return;
  }
  await performAction(command.value, command.label, source);
}

const keyMap = { w: "forward", a: "left", s: "back", d: "right" };
const heldKeys = new Set();
window.addEventListener("keydown", (event) => {
  const direction = keyMap[event.key.toLowerCase()];
  if (!direction || event.repeat || event.target.matches("input")) return;
  event.preventDefault();
  heldKeys.add(event.key.toLowerCase());
  setMotion(direction, "keyboard");
});
window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (!keyMap[key]) return;
  heldKeys.delete(key);
  if (heldKeys.size === 0) stopMotion("key release", state.mode === "mock");
});
window.addEventListener("blur", () => { heldKeys.clear(); stopMotion("window focus lost", state.mode === "mock"); });

$$('[data-motion]').forEach((button) => {
  const start = (event) => { event.preventDefault(); setMotion(button.dataset.motion, "button"); };
  const stop = (event) => { event.preventDefault(); stopMotion("button release", state.mode === "mock"); };
  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointercancel", stop);
  button.addEventListener("pointerleave", (event) => { if (event.buttons) stop(event); });
});

$("#stop-button").addEventListener("click", () => {
  stopMotion("stop");
  if (state.mode === "robot") performAction("stop", "Request StopMove", "stop button");
});
$$('[data-action]').forEach((button) => button.addEventListener("click", () => performAction(button.dataset.action, button.dataset.label, "quick action")));
$("#obstacle-toggle").addEventListener("click", async () => {
  if (!OBSTACLE_CONTROL_ENABLED || state.mode !== "robot" || !state.commandEnabled || state.obstacleAvoidance === null || state.obstaclePending) return;
  const enabled = !state.obstacleAvoidance;
  state.obstaclePending = true;
  renderObstacleControl({ enabled: state.obstacleAvoidance, pending: true });
  try {
    await dashboardApi.obstacleAvoidance(enabled);
    showInterpretation("Obstacle avoidance", `${enabled ? "Enabling" : "Disabling"} through the robot driver...`);
    addLog("action", `Obstacle avoidance ${enabled ? "enable" : "disable"} requested`);
  } catch (error) {
    state.obstaclePending = false;
    renderObstacleControl({ enabled: state.obstacleAvoidance, error: error.message });
    showInterpretation("Setting blocked", error.message);
    addLog("unknown", error.message);
  }
});
$("#command-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const input = $("#command-input");
  interpret(input.value);
  input.value = "";
});
$("#clear-log").addEventListener("click", () => { state.log = []; logList.innerHTML = '<li class="empty-log">Commands will appear here.</li>'; });

const FOXGLOVE_URL = "https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A8765&openIn=desktop";
$$('[data-tool]').forEach((button) => button.addEventListener("click", () => {
  window.open(FOXGLOVE_URL, "_blank", "noopener,noreferrer");
  const message = button.dataset.tool === "mapping"
    ? "Opened Foxglove. Add a Map panel and select /map."
    : "Opened Foxglove at ws://localhost:8765 for camera, LiDAR, and other ROS topics.";
  addLog("tool", message);
}));

const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
const talkButton = $("#talk-button");
if (SpeechRecognition) {
  const recognition = new SpeechRecognition();
  recognition.lang = "en-US";
  recognition.interimResults = true;
  recognition.continuous = true;
  let listeningRequested = false;
  let recognitionActive = false;
  let readyTimer = null;
  let bufferElapsed = false;
  let readyForSpeech = false;
  let resultOffset = 0;
  let transcript = "";
  const MICROPHONE_BUFFER_MS = 3000;
  const BrowserAudioContext = window.AudioContext || window.webkitAudioContext;
  let audioContext;

  const playReadyTone = () => {
    if (!BrowserAudioContext) return;
    audioContext ||= new BrowserAudioContext();
    const oscillator = audioContext.createOscillator();
    const gain = audioContext.createGain();
    oscillator.frequency.value = 740;
    gain.gain.setValueAtTime(0.035, audioContext.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.001, audioContext.currentTime + 0.08);
    oscillator.connect(gain).connect(audioContext.destination);
    oscillator.start();
    oscillator.stop(audioContext.currentTime + 0.08);
  };

  $("#voice-support").textContent = "Press once to begin";

  const markReadyIfPossible = () => {
    if (!listeningRequested || !recognitionActive || !bufferElapsed || readyForSpeech) return;
    readyForSpeech = true;
    transcript = "";
    $("#command-input").value = "";
    talkButton.classList.remove("starting");
    talkButton.classList.add("listening");
    talkButton.querySelector("strong").textContent = "Stop and process";
    $("#voice-support").textContent = "Speak now; press again when finished";
    $("#listening-status").textContent = "Listening now...";
    $("#listening-status").classList.add("live");
    playReadyTone();
  };

  const startListening = () => {
    if (listeningRequested || recognitionActive) return;
    listeningRequested = true;
    bufferElapsed = false;
    readyForSpeech = false;
    resultOffset = 0;
    transcript = "";
    talkButton.classList.add("starting");
    talkButton.querySelector("strong").textContent = "Turning microphone on...";
    $("#voice-support").textContent = "Wait for the ready tone";
    $("#listening-status").textContent = "Opening microphone and starting 3-second ready period...";
    readyTimer = window.setTimeout(() => {
      readyTimer = null;
      bufferElapsed = true;
      markReadyIfPossible();
    }, MICROPHONE_BUFFER_MS);
    try {
      // Start Chrome immediately. The three-second ready period runs at the
      // same time, and early recognition results are discarded.
      recognition.start();
    } catch (error) {
      listeningRequested = false;
      window.clearTimeout(readyTimer);
      readyTimer = null;
      talkButton.classList.remove("starting");
      talkButton.querySelector("strong").textContent = "Start listening";
      $("#voice-support").textContent = "Press once to begin";
      $("#listening-status").textContent = "Microphone idle";
      addLog("unknown", `Microphone error: ${error.message}`);
    }
  };

  const stopListening = () => {
    if (!listeningRequested && !recognitionActive) return;
    listeningRequested = false;
    readyForSpeech = false;
    bufferElapsed = false;
    window.clearTimeout(readyTimer);
    readyTimer = null;
    if (recognitionActive) recognition.stop();
    else recognition.abort();
  };

  talkButton.addEventListener("click", () => {
    if (listeningRequested || recognitionActive) stopListening();
    else startListening();
  });

  recognition.addEventListener("start", () => {
    recognitionActive = true;
    talkButton.classList.add("listening");
    talkButton.querySelector("strong").textContent = "Microphone on · getting ready...";
    $("#voice-support").textContent = "Wait for the ready tone";
    $("#listening-status").textContent = "Microphone is on; finishing the 3-second ready period...";
    $("#listening-status").classList.add("live");
    markReadyIfPossible();
    if (!listeningRequested) recognition.stop();
  });
  recognition.addEventListener("end", () => {
    recognitionActive = false;
    listeningRequested = false;
    window.clearTimeout(readyTimer);
    readyTimer = null;
    bufferElapsed = false;
    readyForSpeech = false;
    talkButton.classList.remove("starting");
    talkButton.classList.remove("listening");
    talkButton.querySelector("strong").textContent = "Start listening";
    $("#voice-support").textContent = "Press once to begin";
    $("#listening-status").textContent = transcript ? `Heard: “${transcript}”` : "Microphone idle";
    $("#listening-status").classList.remove("live");
    if (transcript.trim()) interpret(transcript.trim(), "microphone");
    transcript = "";
  });
  recognition.addEventListener("result", (event) => {
    if (!readyForSpeech) {
      resultOffset = event.results.length;
      return;
    }
    transcript = [...event.results].slice(resultOffset).map((result) => result[0].transcript).join(" ").trim();
    $("#command-input").value = transcript;
    $("#listening-status").textContent = transcript ? `Hearing: “${transcript}”` : "Listening now...";
  });
  recognition.addEventListener("error", (event) => {
    talkButton.classList.remove("starting");
    if (event.error !== "aborted") addLog("unknown", `Microphone error: ${event.error}`);
  });
} else {
  $("#voice-support").textContent = "Use Chrome/Edge or type below";
  talkButton.disabled = true;
}
