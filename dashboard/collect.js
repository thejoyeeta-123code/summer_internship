(() => {
  const $ = (id) => document.getElementById(id);

  const SERVICE_UUID = "59462f12-9543-9999-12c8-58b459a2712d";
  const NOTIFY_UUID = "77776666-5555-4444-3333-222211110000";

  const FIXED_HEADER = "timestamp_ms,ax,ay,az,gx,gy,gz,accel_mag,gyro_mag,jerk,q_i,q_j,q_k,q_r,audio_rms,audio_peak,audio_zcr,crest_factor".split(",");
  const state = {
    device: null,
    server: null,
    notifyCharacteristic: null,
    connected: false,
    recording: false,
    buffer: "",
    rows: [],
    liveRows: [],
    sampleIndex: 0,
    firstRecordedAt: null,
    rateWindow: [],
    colors: ["#2854c5", "#0f8b8d", "#e89918", "#c93434", "#6f42c1", "#16844a", "#d63384", "#495057"]
  };

  const elements = {
    connectionStatus: $("connectionStatus"),
    connectBtn: $("connectBtn"),
    disconnectBtn: $("disconnectBtn"),
    subjectId: $("subjectId"),
    classActivityEmotion: $("classActivityEmotion"),
    recordBtn: $("recordBtn"),
    markerBtn: $("markerBtn"),
    sampleCount: $("sampleCount"),
    recordingState: $("recordingState"),
    sampleRate: $("sampleRate"),
    bufferedRows: $("bufferedRows"),
    lastPacket: $("lastPacket"),
    windowSize: $("windowSize"),
    plotFields: $("plotFields"),
    liveChart: $("liveChart"),
    chartLegend: $("chartLegend"),
    downloadCsvBtn: $("downloadCsvBtn"),
    clearBtn: $("clearBtn")
  };

  elements.connectBtn.addEventListener("click", connectBle);
  elements.disconnectBtn.addEventListener("click", disconnectBle);
  elements.recordBtn.addEventListener("click", toggleRecording);
  elements.markerBtn.addEventListener("click", addMarker);
  elements.downloadCsvBtn.addEventListener("click", downloadDataset);
  elements.clearBtn.addEventListener("click", clearDataset);
  elements.plotFields.addEventListener("change", drawChart);
  elements.windowSize.addEventListener("change", drawChart);

  drawChart();
  updateUi();

  async function connectBle() {
    if (!navigator.bluetooth) {
      alert("Web Bluetooth is not available in this browser. Use Chrome or Edge.");
      return;
    }

    try {
      state.device = await navigator.bluetooth.requestDevice({
        acceptAllDevices: true,
        optionalServices: [SERVICE_UUID]
      });
      state.device.addEventListener("gattserverdisconnected", onDisconnected);
      state.server = await state.device.gatt.connect();
      const service = await state.server.getPrimaryService(SERVICE_UUID);
      state.notifyCharacteristic = await service.getCharacteristic(NOTIFY_UUID);
      await state.notifyCharacteristic.startNotifications();
      state.notifyCharacteristic.addEventListener("characteristicvaluechanged", handleNotification);

      state.connected = true;
      updateUi();
    } catch (error) {
      console.error(error);
      alert(`BLE connection failed: ${error.message}`);
      updateUi();
    }
  }

  async function disconnectBle() {
    try {
      if (state.notifyCharacteristic) {
        state.notifyCharacteristic.removeEventListener("characteristicvaluechanged", handleNotification);
        try { await state.notifyCharacteristic.stopNotifications(); } catch (_) {}
      }
      if (state.device?.gatt?.connected) state.device.gatt.disconnect();
    } finally {
      onDisconnected();
    }
  }

  function onDisconnected() {
    state.connected = false;
    state.recording = false;
    state.notifyCharacteristic = null;
    updateUi();
  }

  function handleNotification(event) {
    const value = event.target.value;
    const chunk = new TextDecoder().decode(value);
    state.buffer += chunk;
    const lines = state.buffer.split(/\r?\n/);
    state.buffer = lines.pop() || "";
    for (const rawLine of lines) processLine(rawLine.trim());
  }

  function processLine(line) {
    if (!line) return;
    elements.lastPacket.textContent = line;
    const parsed = parsePacket(line);
    
    const sampleTime = Date.now();
    state.rateWindow.push(sampleTime);
    const cutoff = sampleTime - 3000;
    state.rateWindow = state.rateWindow.filter((t) => t >= cutoff);

    const liveRow = normalizeNumericFields(parsed);
    state.liveRows.push(liveRow);
    const maxWindow = Math.max(100, Number(elements.windowSize.value) || 250);
    if (state.liveRows.length > maxWindow) state.liveRows.splice(0, state.liveRows.length - maxWindow);

    if (state.recording) {
      if (!state.firstRecordedAt) state.firstRecordedAt = sampleTime;
      state.sampleIndex += 1;
      const labeledRow = {
        ...parsed,
        ...currentLabels(), 
        dashboard_sample_index: state.sampleIndex
      };
      state.rows.push(labeledRow);
    }

    updateUi();
    drawChart();
  }

  function parsePacket(line) {
    const parts = line.split(",").map((part) => part.trim());
    const row = {};
    parts.forEach((value, index) => {
      const key = FIXED_HEADER[index] || `field_${index + 1}`;
      row[key] = numericOrString(value);
    });
    return row;
  }

  function normalizeNumericFields(row) {
    const out = {};
    for (const [key, value] of Object.entries(row)) {
      const n = Number(value);
      if (Number.isFinite(n)) out[key] = n;
    }
    out.__x = state.liveRows.length;
    return out;
  }

  function currentLabels() {
    const [classId, activity, emotion] = elements.classActivityEmotion.value.split(":");
    return {
      subject_id: sanitizeLabel(elements.subjectId.value || "S01"),
      class_id: Number(classId),
      activity: sanitizeLabel(activity),
      emotion: sanitizeLabel(emotion)
    };
  }

  function toggleRecording() {
    state.recording = !state.recording;
    if (state.recording && !state.firstRecordedAt) state.firstRecordedAt = Date.now();
    updateUi();
  }

  function addMarker() {
    if (!state.recording) return;
    const marker = {
      ...currentLabels(),
      dashboard_sample_index: ++state.sampleIndex,
      marker: "manual_marker"
    };
    state.rows.push(marker);
    updateUi();
  }

  function updateUi() {
    const statusDotClass = state.recording ? "recording" : state.connected ? "online" : "offline";
    const deviceName = state.device?.name || "Unnamed BLE device";
    elements.connectionStatus.innerHTML = `
      <span class="status-dot ${statusDotClass}"></span>
      <div>
        <strong>${state.connected ? (state.recording ? "Recording" : "Connected") : "Disconnected"}</strong>
        <small>${state.connected ? escapeHtml(deviceName) : "No BLE device selected"}</small>
      </div>`;

    elements.connectBtn.disabled = state.connected;
    elements.disconnectBtn.disabled = !state.connected;
    elements.recordBtn.disabled = !state.connected;
    elements.markerBtn.disabled = !state.connected || !state.recording;
    elements.recordBtn.textContent = state.recording ? "Stop Recording" : "Start Recording";
    elements.recordBtn.classList.toggle("active", state.recording);

    elements.sampleCount.textContent = String(state.rows.length);
    elements.recordingState.textContent = state.recording ? "Yes" : "No";
    const hz = state.rateWindow.length > 1 ? (state.rateWindow.length - 1) / ((state.rateWindow.at(-1) - state.rateWindow[0]) / 1000) : 0;
    elements.sampleRate.textContent = `${Number.isFinite(hz) ? hz.toFixed(1) : "0"} Hz`;
    elements.bufferedRows.textContent = String(state.liveRows.length);
    elements.downloadCsvBtn.disabled = state.rows.length === 0;
  }

  function drawChart() {
    const canvas = elements.liveChart;
    const ctx = canvas.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.max(600, Math.floor(rect.width * dpr));
    canvas.height = Math.floor(420 * dpr);
    ctx.scale(dpr, dpr);

    const width = rect.width;
    const height = 420;
    const pad = { left: 58, right: 20, top: 28, bottom: 42 };
    ctx.clearRect(0, 0, width, height);
    drawGrid(ctx, width, height, pad);

    const fields = elements.plotFields.value.split(",").map((f) => f.trim()).filter(Boolean);
    const windowSize = Number(elements.windowSize.value) || 250;
    const data = state.liveRows.slice(-windowSize);

    elements.chartLegend.innerHTML = "";
    fields.forEach((field, index) => {
      const color = state.colors[index % state.colors.length];
      const item = document.createElement("span");
      item.className = "legend-item";
      item.innerHTML = `<span class="legend-dot" style="background:${color}"></span>${escapeHtml(field)}`;
      elements.chartLegend.appendChild(item);
    });

    const values = [];
    for (const row of data) {
      for (const field of fields) {
        if (Number.isFinite(row[field])) values.push(row[field]);
      }
    }
    if (!values.length) return;

    let minY = Math.min(...values);
    let maxY = Math.max(...values);
    if (minY === maxY) { minY -= 1; maxY += 1; }
    const margin = (maxY - minY) * 0.12;
    minY -= margin;
    maxY += margin;

    drawAxisLabels(ctx, width, height, pad, minY, maxY, data.length);

    fields.forEach((field, seriesIndex) => {
      ctx.beginPath();
      ctx.strokeStyle = state.colors[seriesIndex % state.colors.length];
      ctx.lineWidth = 2;
      let started = false;
      data.forEach((row, index) => {
        const value = row[field];
        if (!Number.isFinite(value)) return;
        const x = pad.left + (index / Math.max(1, data.length - 1)) * (width - pad.left - pad.right);
        const y = pad.top + (1 - ((value - minY) / (maxY - minY))) * (height - pad.top - pad.bottom);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    });
  }

  function drawGrid(ctx, width, height, pad) {
    ctx.save();
    ctx.strokeStyle = "#e6edf7";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const y = pad.top + i * ((height - pad.top - pad.bottom) / 5);
      ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke();
    }
    for (let i = 0; i <= 8; i++) {
      const x = pad.left + i * ((width - pad.left - pad.right) / 8);
      ctx.beginPath(); ctx.moveTo(x, pad.top); ctx.lineTo(x, height - pad.bottom); ctx.stroke();
    }
    ctx.strokeStyle = "#b8c6dd";
    ctx.beginPath(); ctx.moveTo(pad.left, pad.top); ctx.lineTo(pad.left, height - pad.bottom); ctx.lineTo(width - pad.right, height - pad.bottom); ctx.stroke();
    ctx.restore();
  }

  function drawAxisLabels(ctx, width, height, pad, minY, maxY, count) {
    ctx.save();
    ctx.fillStyle = "#667085";
    ctx.font = "12px system-ui";
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let i = 0; i <= 5; i++) {
      const value = maxY - i * ((maxY - minY) / 5);
      const y = pad.top + i * ((height - pad.top - pad.bottom) / 5);
      ctx.fillText(formatNumber(value), pad.left - 9, y);
    }
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.fillText(`${count} live samples`, width / 2, height - 28);
    ctx.restore();
  }

  function downloadDataset() {
    const [classId, activity] = elements.classActivityEmotion.value.split(":");
    const subjectId = sanitizeLabel(elements.subjectId.value || "S01");
    const act = sanitizeLabel(activity);
    const filename = `${subjectId}_${classId}_${act}.csv`;
    
    const csv = rowsToCsv(state.rows);
    const blob = new Blob([csv], { type: "text/csv;charset=utf-8" });
    
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }

  function rowsToCsv(rows) {
    const EXPORT_ORDER = [
      "timestamp_ms", "subject_id", "class_id", "activity", "emotion"
    ].concat(FIXED_HEADER.slice(1));
    const keys = Array.from(new Set([...EXPORT_ORDER, ...collectKeys(rows)]));
    const header = keys.map(csvEscape).join(",");
    const body = rows.map((row) => keys.map((key) => csvEscape(row[key])).join(",")).join("\n");
    return `${header}\n${body}`;
  }

  function clearDataset() {
    if (state.rows.length && !confirm("Clear all collected rows?")) return;
    state.rows = [];
    state.liveRows = [];
    state.sampleIndex = 0;
    state.firstRecordedAt = null;
    updateUi();
    drawChart();
  }

  function collectKeys(rows) {
    const seen = new Set();
    for (const row of rows) Object.keys(row).forEach((key) => seen.add(key));
    return Array.from(seen);
  }

  function sanitizeLabel(value) {
    return String(value || "").trim().replace(/\s+/g, "_").replace(/[^a-zA-Z0-9_\-]/g, "");
  }

  function numericOrString(value) {
    const n = Number(value);
    return value !== "" && Number.isFinite(n) ? n : value;
  }

  function csvEscape(value) {
    if (value === undefined || value === null) return "";
    const text = String(value);
    return /[",\n\r]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
  }

  function formatNumber(value) {
    const abs = Math.abs(value);
    if (abs >= 1000 || abs < 0.01) return value.toExponential(1);
    return value.toFixed(2);
  }

  function escapeHtml(text) {
    return String(text).replace(/[&<>'"]/g, (c) => ({"&":"&amp;","<":"&lt;",">":"&gt;","'":"&#39;","\"":"&quot;"}[c]));
  }
})();