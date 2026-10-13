const API_BASE = "http://localhost:8080";

async function request(path, options = {}) {
  const response = await fetch(`${API_BASE}${path}`, {
    ...options,
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
  });
  let body = {};
  try {
    body = await response.json();
  } catch {
    body = { error: `Bridge returned HTTP ${response.status}` };
  }
  if (!response.ok) throw new Error(body.error || `Bridge returned HTTP ${response.status}`);
  return body;
}

export const dashboardApi = {
  status: () => request("/api/status"),
  objects: () => request("/api/objects"),
  cameraUrl: () => `${API_BASE}/api/camera.jpg?t=${Date.now()}`,
  action: (action) => request("/api/action", { method: "POST", body: JSON.stringify({ action }) }),
  obstacleAvoidance: (enabled) => request("/api/obstacle-avoidance", { method: "POST", body: JSON.stringify({ enabled }) }),
  command: (text) => request("/api/command", { method: "POST", body: JSON.stringify({ text }) }),
  tts: (text) => request("/api/tts", { method: "POST", body: JSON.stringify({ text }) }),
};
