(() => {
    const $ = (id) => document.getElementById(id);
  
    const SERVICE_UUID = "59462f12-9543-9999-12c8-58b459a2712d";
    const NOTIFY_UUID = "77776666-5555-4444-3333-222211110000";
  
    // Update these arrays based on the exact classes you trained your model on
    const CLASS_LABELS = ["Class 0", "Class 1", "Class 2", "Class 3", "Class 4", "Class 5", "Class 6", "Class 7"];
    
    // Inference Parameters
    const WINDOW_SIZE = 100; // Number of samples required to calculate features (2 seconds at 50Hz)
    const SLIDING_STEP = 100; // Slide window every 25 samples (Updates prediction every 0.5s)
  
    // The exact 17 metrics from your firmware payload
    const METRICS = [
      "ax", "ay", "az", "gx", "gy", "gz", "accel_mag", "gyro_mag", "jerk",
      "q_i", "q_j", "q_k", "q_r", "audio_rms", "audio_peak", "audio_zcr", "crest_factor"
    ];
  
    const state = {
      device: null,
      server: null,
      notifyCharacteristic: null,
      connected: false,
      buffer: "",
      rowBuffer: [],
      rateWindow: [],
      currentPred: "--",
      lastPred: "--"
    };
  
    const elements = {
      connectionStatus: $("connectionStatus"),
      connectBtn: $("connectBtn"),
      disconnectBtn: $("disconnectBtn"),
      lastPacket: $("lastPacket"),
      currentPrediction: $("currentPrediction"),
      lastPrediction: $("lastPrediction"),
      dataRate: $("dataRate")
    };
  
    let WasmModule = null; // Will hold our compiled C code
    
    // ---------------------------------------------------------
    // 1. INITIALIZE WEBASSEMBLY BEFORE ALLOWING BLE CONNECTION
    // ---------------------------------------------------------
    elements.connectBtn.disabled = true;
    elements.connectBtn.textContent = "Loading Engine...";

    if (typeof InitModel === 'function') {
        InitModel().then(loadedModule => {
            WasmModule = loadedModule;
            console.log("WASM Model loaded successfully!", WasmModule);
            elements.connectBtn.disabled = false;
            elements.connectBtn.textContent = "Connect BLE Device";
        }).catch(err => {
            console.error("Failed to load WASM module", err);
            elements.connectBtn.textContent = "Engine Load Failed";
        });
    } else {
        console.error("InitModel function not found. Did the Emscripten compile fail?");
    }

    elements.connectBtn.addEventListener("click", connectBLE);
    elements.disconnectBtn.addEventListener("click", disconnectBLE);
  
    // ---------------------------------------------------------
    // 2. BLE CONNECTION LOGIC
    // ---------------------------------------------------------
    async function connectBLE() {
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
        state.notifyCharacteristic.addEventListener("characteristicvaluechanged", handleNotifications);
        
        state.connected = true;
        updateUi();
      } catch (error) {
        console.error("BLE Connection failed", error);
      }
    }
  
    function disconnectBLE() {
      if (state.device && state.device.gatt.connected) {
        state.device.gatt.disconnect();
      }
    }
  
    function onDisconnected() {
      state.connected = false;
      state.device = null;
      state.server = null;
      state.notifyCharacteristic = null;
      updateUi();
    }
  
    function handleNotifications(event) {
      const decoder = new TextDecoder("utf-8");
      const chunk = decoder.decode(event.target.value);
      state.buffer += chunk;
      processBuffer();
      
      const now = performance.now();
      state.rateWindow.push(now);
      state.rateWindow = state.rateWindow.filter((t) => now - t <= 1000);
      elements.dataRate.textContent = `${state.rateWindow.length} Hz`;
    }
  
    function processBuffer() {
      const lines = state.buffer.split("\n");
      state.buffer = lines.pop(); // Keep incomplete line in buffer
  
      for (const line of lines) {
        const str = line.trim();
        if (!str) continue;
        elements.lastPacket.textContent = str;
        
        const parts = str.split(",");
        if (parts.length < 17) continue; 
  
        const row = {};
        // Read from the back of the array to guarantee alignment
        for (let i = 0; i < 17; i++) {
            const valIndex = parts.length - 17 + i; 
            row[METRICS[i]] = parseFloat(parts[valIndex]);
        }
  
        state.rowBuffer.push(row);
      }
  
      // When buffer fills, extract features and run inference
      if (state.rowBuffer.length >= WINDOW_SIZE) {
          const features = extractFeatures(state.rowBuffer);
          runInference(features);
          
          // Slide the window forward
          state.rowBuffer.splice(0, SLIDING_STEP);
      }
    }
  
    function extractFeatures(rows) {
      const stats = { mean: [], std: [], min: [], max: [], median: [], skew: [], kurt: [] };
  
      METRICS.forEach(metric => {
          const arr = rows.map(r => r[metric] || 0);
          const n = arr.length;
          
          const m = arr.reduce((a, b) => a + b, 0) / n;
          stats.mean.push(m);
  
          const variance = arr.reduce((a, b) => a + Math.pow(b - m, 2), 0) / n;
          const s = Math.sqrt(variance);
          stats.std.push(s);
  
          stats.min.push(Math.min(...arr));
          stats.max.push(Math.max(...arr));
  
          const sorted = [...arr].sort((a, b) => a - b);
          const mid = Math.floor(n / 2);
          stats.median.push(n % 2 === 0 ? (sorted[mid - 1] + sorted[mid]) / 2 : sorted[mid]);
  
          const skew = s === 0 ? 0 : (arr.reduce((a, b) => a + Math.pow(b - m, 3), 0) / n) / Math.pow(s, 3);
          stats.skew.push(skew);
  
          const kurt = s === 0 ? 0 : (arr.reduce((a, b) => a + Math.pow(b - m, 4), 0) / n) / Math.pow(s, 4);
          stats.kurt.push(kurt);
      });
  
      return [].concat(stats.mean, stats.std, stats.min, stats.max, stats.median, stats.skew, stats.kurt);
    }
  
    // ---------------------------------------------------------
    // 3. SECURE WEBASSEMBLY INFERENCE
    // ---------------------------------------------------------
    function runInference(features) {
      if (!WasmModule) {
          console.warn("WASM Module is not ready.");
          return;
      }
  
      // 1. Convert our 119 Javascript numbers into a native C Float Array
      const floatArray = new Float32Array(features);
      
      // 2. Trick: ccall expects a raw stream of bytes (8-bit). 
      // We wrap our float buffer in a Uint8Array so ccall can safely copy it.
      const byteArray = new Uint8Array(floatArray.buffer);
      
      // 3. Execute the C function using the bridge.
      // ccall will automatically malloc, write the memory, execute, and free!
      const predictedClassInt = WasmModule.ccall(
          'predict_random_forest', // Name of C function (ccall doesn't need the underscore)
          'number',                // What the C function returns
          ['array'],               // What we are passing to it
          [byteArray]              // The actual data
      );
  
      // 4. Update UI
      state.lastPred = state.currentPred;
      state.currentPred = CLASS_LABELS[predictedClassInt] || `Class ${predictedClassInt}`;
      elements.currentPrediction.textContent = state.currentPred;
      elements.lastPrediction.textContent = state.lastPred;
    }
  
    function updateUi() {
      elements.connectBtn.disabled = state.connected;
      elements.disconnectBtn.disabled = !state.connected;
  
      const dot = elements.connectionStatus.querySelector(".status-dot");
      const text = elements.connectionStatus.querySelector("strong");
      const sub = elements.connectionStatus.querySelector("small");
  
      if (state.connected) {
        dot.className = "status-dot online";
        text.textContent = "Connected";
        sub.textContent = state.device ? state.device.name : "Device active";
      } else {
        dot.className = "status-dot offline";
        text.textContent = "Disconnected";
        sub.textContent = "No BLE device selected";
        elements.dataRate.textContent = "0 Hz";
      }
    }
})();