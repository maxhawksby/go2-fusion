import { parseCommand } from "./command-parser.js";

const state = { connected: false, moving: null, log: [] };
const motionSpeeds = { forward: 0.2, back: -0.15, left: 0, right: 0 };
const motionLabels = { forward: "Moving forward", back: "Moving backward", left: "Turning left", right: "Turning right" };

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

function addLog(kind, message) {
  state.log.unshift({ kind, message, time: new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }) });
  state.log = state.log.slice(0, 8);
  logList.innerHTML = state.log.map((entry) => `
    <li><span class="log-kind ${entry.kind}">${entry.kind}</span><p>${escapeHtml(entry.message)}</p><time>${entry.time}</time></li>
  `).join("");
}

function escapeHtml(value) {
  const node = document.createElement("div");
  node.textContent = value;
  return node.innerHTML;
}

function setMotion(direction, source = "control") {
  if (state.moving === direction) return;
  state.moving = direction;
  $$("[data-motion]").forEach((button) => button.classList.toggle("active", button.dataset.motion === direction));
  $("#velocity-value").textContent = Math.abs(motionSpeeds[direction]).toFixed(2);
  $("#motion-label").textContent = motionLabels[direction];
  addLog("motion", `${motionLabels[direction]} (${source}, mock only)`);
}

function stopMotion(source = "control") {
  if (!state.moving && source !== "stop") return;
  state.moving = null;
  $$("[data-motion]").forEach((button) => button.classList.remove("active"));
  $("#velocity-value").textContent = "0.00";
  $("#motion-label").textContent = "Standing by";
  addLog("stop", `Stop (${source}, mock only)`);
}

function interpret(raw, source = "typed") {
  const command = parseCommand(raw);
  if (command.type === "unknown") {
    interpretation.innerHTML = `<strong>Not recognized</strong><span>“${escapeHtml(raw)}”</span>`;
    addLog("unknown", `Could not map “${raw}”`);
    return;
  }

  interpretation.innerHTML = `<strong>${command.label}</strong><span>Recognized from ${source}; execution disabled</span>`;
  if (command.type === "stop") stopMotion("voice command");
  else addLog(command.type, `${command.label} (${source}, dry run)`);
}

$("#simulate-connect").addEventListener("click", () => {
  state.connected = !state.connected;
  $("#simulate-connect").textContent = state.connected ? "Stop simulation" : "Start simulation";
  $("#connection-label").textContent = state.connected ? "Simulator online" : "Offline simulator";
  $(".connection-card").classList.toggle("connected", state.connected);
  $$('[data-sensor]').forEach((sensor) => {
    sensor.textContent = state.connected ? (sensor.dataset.sensor === "map" ? "Demo loaded" : "Simulated") : (sensor.dataset.sensor === "map" ? "No session" : "Standby");
    sensor.closest(".metric").classList.toggle("online", state.connected);
  });
  addLog("system", state.connected ? "Mock simulation started" : "Mock simulation stopped");
});

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
  if (heldKeys.size === 0) stopMotion("key release");
});
window.addEventListener("blur", () => { heldKeys.clear(); stopMotion("window focus lost"); });

$$('[data-motion]').forEach((button) => {
  const start = (event) => { event.preventDefault(); setMotion(button.dataset.motion, "button"); };
  const stop = (event) => { event.preventDefault(); stopMotion("button release"); };
  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointercancel", stop);
  button.addEventListener("pointerleave", (event) => { if (event.buttons) stop(event); });
});

$("#stop-button").addEventListener("click", () => stopMotion("stop"));
$$('[data-action]').forEach((button) => button.addEventListener("click", () => interpret(button.dataset.action, "quick action")));
$("#command-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const input = $("#command-input");
  interpret(input.value);
  input.value = "";
});
$("#clear-log").addEventListener("click", () => { state.log = []; logList.innerHTML = '<li class="empty-log">Commands will appear here.</li>'; });

$$('[data-tool]').forEach((button) => button.addEventListener("click", () => {
  const name = button.dataset.tool === "foxglove" ? "Foxglove connection at ws://localhost:8765" : "Mapping workspace";
  addLog("tool", `${name} selected (placeholder)`);
}));

const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
const talkButton = $("#talk-button");
if (SpeechRecognition) {
  const recognition = new SpeechRecognition();
  recognition.lang = "en-US";
  recognition.interimResults = false;
  recognition.continuous = false;
  $("#voice-support").textContent = "Browser microphone available";
  talkButton.addEventListener("click", () => recognition.start());
  recognition.addEventListener("start", () => { talkButton.classList.add("listening"); $("#voice-orb").classList.add("listening"); $("#voice-support").textContent = "Listening…"; });
  recognition.addEventListener("end", () => { talkButton.classList.remove("listening"); $("#voice-orb").classList.remove("listening"); $("#voice-support").textContent = "Browser microphone available"; });
  recognition.addEventListener("result", (event) => {
    const transcript = event.results[0][0].transcript;
    $("#command-input").value = transcript;
    interpret(transcript, "microphone");
  });
  recognition.addEventListener("error", (event) => addLog("unknown", `Microphone error: ${event.error}`));
} else {
  $("#voice-support").textContent = "Use Chrome/Edge or type below";
  talkButton.disabled = true;
}
