/*
  Senior Design project
  Xiao ESP32-C6 — AP Web UI: Volume + Frequency + Mode(Continuous/Beep)
  Output: Two piezo-electric buzzers driven by LEDC PWM
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "BeepBaseball";
const char* AP_PASS = "SuperSecure420-69";

// -------------------- Piezo / LEDC Config --------------------

// Two piezo outputs (set to your actual GPIOs)
constexpr int BUZZER1_PIN = 2;
constexpr int BUZZER2_PIN = 3;

// Two independent PWM channels
constexpr int LEDC_CH1 = 0;
constexpr int LEDC_CH2 = 1;

constexpr int LEDC_RES_BITS = 10;
constexpr uint32_t LEDC_MAX_DUTY = (1u << LEDC_RES_BITS) - 1;

// Custom frequency range (single source of truth)
constexpr uint16_t FREQ_MIN_HZ = 1000;
constexpr uint16_t FREQ_MAX_HZ = 3000;
constexpr uint16_t FREQ_DEFAULT_HZ = 1500;

// -------------------- Battery --------------------
constexpr int   BATTERY_ADC_PIN = -1;
constexpr float ADC_REF_V      = 3.3f;
constexpr int   ADC_BITS       = 12;
constexpr float BAT_DIVIDER    = 2.0f;
constexpr float BATT_MIN_V     = 3.30f;
constexpr float BATT_MAX_V     = 4.20f;

// -------------------- Types / State --------------------
enum Mode : uint8_t { MODE_CONTINUOUS = 0, MODE_BEEP = 1 };

struct SystemState {
  uint16_t volume_pct;
  uint16_t freq_hz;
  Mode     mode;
  uint16_t rate_tenths;
  bool     muted;
  int8_t   batt_pct;
};

WebServer server(80);
static SystemState g_state { 50, FREQ_DEFAULT_HZ, MODE_CONTINUOUS, 20, true, -1 };

// Beep gating
static bool g_gate_on = false;
static uint32_t g_next_toggle_ms = 0;

// -------------------- Utils --------------------
static inline uint16_t clamp_u16(int v, int lo, int hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

static inline uint32_t halfPeriodMsFromRateTenths(uint16_t rate_tenths) {
  return (uint32_t)((5000u + (rate_tenths / 2u)) / rate_tenths);
}

static inline uint32_t volumePctToDuty(uint16_t vol_pct) {
  uint32_t max_half_duty = LEDC_MAX_DUTY / 2u;
  if (vol_pct == 0) return 0;
  if (vol_pct >= 100) return max_half_duty;
  return (max_half_duty * (uint32_t)vol_pct) / 100u;
}

// -------------------- Battery --------------------
int batteryPercent() {
  if (BATTERY_ADC_PIN < 0) return -1;
  analogReadResolution(ADC_BITS);
  int raw = analogRead(BATTERY_ADC_PIN);
  float v_adc  = (raw * ADC_REF_V) / ((1 << ADC_BITS) - 1);
  float v_batt = v_adc * BAT_DIVIDER;
  float pct    = (v_batt - BATT_MIN_V) / (BATT_MAX_V - BATT_MIN_V);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 1.0f) pct = 1.0f;
  return (int)lroundf(pct * 100.0f);
}

// -------------------- Dual Piezo Driver --------------------
void piezoInitDual() {
  ledcSetup(LEDC_CH1, FREQ_DEFAULT_HZ, LEDC_RES_BITS);
  ledcAttachPin(BUZZER1_PIN, LEDC_CH1);
  ledcWrite(LEDC_CH1, 0);

  ledcSetup(LEDC_CH2, FREQ_DEFAULT_HZ, LEDC_RES_BITS);
  ledcAttachPin(BUZZER2_PIN, LEDC_CH2);
  ledcWrite(LEDC_CH2, 0);
}

void piezoStartBoth(uint16_t freq_hz, uint32_t duty) {
  if (duty == 0) {
    ledcWrite(LEDC_CH1, 0);
    ledcWrite(LEDC_CH2, 0);
    return;
  }

  ledcSetup(LEDC_CH1, freq_hz, LEDC_RES_BITS);
  ledcWrite(LEDC_CH1, duty);

  ledcSetup(LEDC_CH2, freq_hz, LEDC_RES_BITS);
  ledcWrite(LEDC_CH2, duty);
}

void piezoStopBoth() {
  ledcWrite(LEDC_CH1, 0);
  ledcWrite(LEDC_CH2, 0);
}

void applyStateToPiezoDual(bool reset_beep_phase) {
  g_state.freq_hz = clamp_u16(g_state.freq_hz, FREQ_MIN_HZ, FREQ_MAX_HZ);
  g_state.rate_tenths = clamp_u16(g_state.rate_tenths, 5, 100);
  g_state.volume_pct = clamp_u16(g_state.volume_pct, 0, 100);

  uint32_t duty = (g_state.muted) ? 0 : volumePctToDuty(g_state.volume_pct);

  if (duty == 0) {
    g_gate_on = false;
    piezoStopBoth();
    return;
  }

  if (g_state.mode == MODE_CONTINUOUS) {
    g_gate_on = true;
    piezoStartBoth(g_state.freq_hz, duty);
    return;
  }

  if (reset_beep_phase) {
    g_gate_on = true;
    g_next_toggle_ms = millis() + halfPeriodMsFromRateTenths(g_state.rate_tenths);
    piezoStartBoth(g_state.freq_hz, duty);
  }
}

void piezoServiceDual() {
  if (g_state.muted || g_state.volume_pct == 0) {
    if (g_gate_on) {
      g_gate_on = false;
      piezoStopBoth();
    }
    return;
  }

  if (g_state.mode != MODE_BEEP) return;

  uint32_t now = millis();
  if ((int32_t)(now - g_next_toggle_ms) >= 0) {
    g_gate_on = !g_gate_on;
    g_next_toggle_ms = now + halfPeriodMsFromRateTenths(g_state.rate_tenths);

    uint32_t duty = volumePctToDuty(g_state.volume_pct);
    if (g_gate_on) piezoStartBoth(g_state.freq_hz, duty);
    else piezoStopBoth();
  }
}

// -------------------- HTML --------------------
String htmlPage() {
  String page = R"====(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1"/>
<title>ESP Piezo Control</title>
<style>
  :root { color-scheme: dark; --bg:#0b0f14; --panel:#121820; --accent:#4da3ff; --accent-2:#7fc7ff; --text:#e6edf3; --muted:#9aa7b3; --ok:#34d399; --warn:#f59e0b; }
  * { box-sizing:border-box; font-family: system-ui, -apple-system, Segoe UI, Roboto, Inter, sans-serif; }
  body { margin:0; background: radial-gradient(1200px 800px at 80% -20%, #14202c 0%, var(--bg) 55%) fixed, var(--bg); color:var(--text); }
  .wrap { max-width: 820px; margin: 4vh auto; padding: clamp(12px, 3vw, 24px); }
  .card { background: linear-gradient(180deg, rgba(255,255,255,.02), rgba(0,0,0,.2));
          border: 1px solid rgba(255,255,255,.06); border-radius: 16px; padding: clamp(14px, 3vw, 22px);
          box-shadow: 0 6px 20px rgba(0,0,0,.35); backdrop-filter: blur(6px); }
  h1 { font-size: clamp(1.05rem, 1.6rem, 1.35rem); margin: 0 0 10px; letter-spacing: .3px; display:flex; gap:10px; align-items:center; flex-wrap: wrap;}
  p  { margin: 0 0 14px; color: var(--muted); font-size: clamp(.9rem, 1rem, 1rem); }
  .row { display:grid; gap:12px; grid-template-columns: 1fr auto; align-items:center; margin:12px 0 18px;}
  @media (max-width:600px){ .row { grid-template-columns: 1fr; } }
  label { font-weight:600; color: var(--accent-2); font-size: clamp(.95rem, 1rem, 1.05rem);  }
  input[type="range"] { width: 100%; height: 36px; }
  .readout { min-width: 110px; background:#0f141b; color:var(--text);
             border:1px solid rgba(255,255,255,.08); border-radius:10px; padding:10px 12px; text-align:center; font-feature-settings:"tnum"; font-size:1rem; }
  .grid { display:grid; gap:16px; grid-template-columns: 1fr; }
  .btns { display:flex; gap:10px; flex-wrap:wrap; margin-top: 6px; }
  button, select { padding:12px 14px; border-radius:12px; border:1px solid rgba(255,255,255,.1);
                   background:#10161e; color:var(--text); cursor:pointer; font-weight:600; font-size:1rem; }
  .primary{ background: linear-gradient(180deg, #132235, #0f1a27); border-color: rgba(77,163,255,.35); }
  .good{ border-color: rgba(52,211,153,.35); }
  .warn{ border-color: rgba(245,158,11,.35); }
  .pill { display:inline-flex; align-items:center; gap:6px; padding:.25rem .6rem; border:1px solid rgba(255,255,255,.08);
          border-radius:999px; color: var(--muted); font-size:.9rem;}
  .footer { margin-top:12px; font-size:.95rem; color:var(--muted); display:flex; justify-content:space-between; gap:10px; flex-wrap:wrap;}
  .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace; color:#c4d1de;}
</style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>
        ESP Piezo Control (Dual)
        <span class="pill mono" id="status">Connecting...</span>
        <span class="pill mono">Battery: <span id="bat">n/a</span></span>
      </h1>
      <p>Controls <strong>Volume</strong>, <strong>Frequency</strong>, and <strong>Mode</strong> for two piezo buzzers.</p>

      <div class="grid">
        <div>
          <label for="vol">Volume</label>
          <div class="row">
            <input id="vol" type="range" min="0" max="100" value="50" step="1"/>
            <input id="volBox" class="readout" type="text" inputmode="numeric" value="50%" />
          </div>
        </div>

        <div>
          <label for="freq">Frequency</label>
          <div class="row">
            <input id="freq" type="range" min="{{FREQ_MIN}}" max="{{FREQ_MAX}}" value="{{FREQ_DEF}}" step="1"/>
            <input id="freqBox" class="readout" type="text" inputmode="numeric" value="{{FREQ_DEF}} Hz" />
          </div>
        </div>

        <div>
          <label for="mode">Mode</label>
          <div class="row">
            <select id="mode">
              <option value="0">Continuous</option>
              <option value="1">Beep</option>
            </select>
            <div id="beepControls" class="readout" style="display:flex; gap:10px; align-items:center; justify-content:space-between;">
              <span class="mono" style="opacity:.8;">Beep rate</span>
              <input id="rate" type="range" min="5" max="100" value="20" step="1" style="width: 180px;"/>
              <span id="rateLabel" class="mono">2.0 Hz</span>
            </div>
          </div>
        </div>

        <div class="btns">
          <button class="primary" id="apply">Apply</button>
          <button class="good" id="mute">Mute/Unmute</button>
          <button class="warn" id="stop">Stop</button>
        </div>
      </div>

      <div class="footer">
        <div>Frequency range: <span class="mono">{{FREQ_MIN}}–{{FREQ_MAX}} Hz</span></div>
        <div>Device: <span class="mono" id="devName">ESP32-C6</span></div>
      </div>
    </div>
  </div>

<script>
const $ = (id)=>document.getElementById(id);
const statusEl=$("status"), vol=$("vol"), volBox=$("volBox"), freq=$("freq"), freqBox=$("freqBox");
const modeSel=$("mode"), rate=$("rate"), rateLabel=$("rateLabel"), beepControls=$("beepControls"), bat=$("bat");
const applyBtn=$("apply"), muteBtn=$("mute"), stopBtn=$("stop");

let state = { vol:50, freq:{{FREQ_DEF}}, muted:false, mode:0, rate:20, bat:-1 };

function showBeepUI(){ beepControls.style.display = (state.mode===1) ? "flex" : "none"; }
function format(){
  volBox.value  = `${state.vol}%`;
  freqBox.value = `${state.freq} Hz`;
  vol.value     = state.vol;
  freq.value    = state.freq;
  modeSel.value = String(state.mode);
  rate.value    = state.rate;
  rateLabel.textContent = `${(state.rate/10).toFixed(1)} Hz`;
  bat.textContent = (state.bat>=0) ? `${state.bat}%` : "n/a";
  showBeepUI();
}
function clamp(n,min,max){ return Math.max(min, Math.min(max, n)); }
async function fetchJSON(url){ const r=await fetch(url,{cache:"no-store"}); if(!r.ok) throw new Error("net"); return await r.json(); }

async function init(){
  try{
    statusEl.textContent="Syncing…";
    const s = await fetchJSON("/get");
    state = s; format();
    statusEl.textContent = s.muted ? "Muted" : "Ready";
    setInterval(async ()=>{
      try { const s = await fetchJSON("/get"); state.bat = s.bat; bat.textContent = (state.bat>=0)?(`${state.bat}%`):"n/a"; } catch(_){}
    }, 5000);
  }catch(e){ statusEl.textContent="Offline?"; }
}

async function push(){
  const url = `/set?vol=${state.vol}&freq=${state.freq}&mode=${state.mode}&rate=${state.rate}`;
  try{
    statusEl.textContent="Updating…";
    const s = await fetchJSON(url);
    state = s; format();
    statusEl.textContent = s.muted ? "Muted" : "Ready";
  }catch(e){ statusEl.textContent="Error"; }
}

vol.addEventListener("input", ()=>{ state.vol=clamp(+vol.value,0,100); volBox.value=`${state.vol}%`; });
freq.addEventListener("input",()=>{ state.freq=clamp(+freq.value,{{FREQ_MIN}},{{FREQ_MAX}}); freqBox.value=`${state.freq} Hz`; });
volBox.addEventListener("change", ()=>{ const n=parseInt(volBox.value.replace(/[^\d]/g,"")||"0",10); state.vol=clamp(n,0,100); vol.value=state.vol; volBox.value=`${state.vol}%`; });
freqBox.addEventListener("change",()=>{ const n=parseInt(freqBox.value.replace(/[^\d]/g,"")||"0",10); state.freq=clamp(n,{{FREQ_MIN}},{{FREQ_MAX}}); freq.value=state.freq; freqBox.value=`${state.freq} Hz`; });

modeSel.addEventListener("change", ()=>{ state.mode=+modeSel.value|0; showBeepUI(); });
rate.addEventListener("input", ()=>{ state.rate = clamp(+rate.value,5,100); rateLabel.textContent = `${(state.rate/10).toFixed(1)} Hz`; });

applyBtn.addEventListener("click", ()=>{ state.muted=false; push(); });
muteBtn.addEventListener("click", async()=>{ try{ const s=await fetchJSON("/toggle_mute"); state=s; format(); statusEl.textContent=s.muted?"Muted":"Ready"; }catch(_){ statusEl.textContent="Error"; }});
stopBtn.addEventListener("click", async()=>{ try{ const s=await fetchJSON("/stop"); state=s; format(); statusEl.textContent="Stopped"; }catch(_){ statusEl.textContent="Error"; }});

init();
</script>
</body>
</html>)====";

  page.replace("{{FREQ_MIN}}", String(FREQ_MIN_HZ));
  page.replace("{{FREQ_MAX}}", String(FREQ_MAX_HZ));
  page.replace("{{FREQ_DEF}}", String(FREQ_DEFAULT_HZ));
  return page;
}

// -------------------- HTTP handlers --------------------
void sendState() {
  g_state.batt_pct = batteryPercent();
  String json;
  json.reserve(96);
  json += "{";
  json += "\"vol\":";   json += g_state.volume_pct;   json += ",";
  json += "\"freq\":";  json += g_state.freq_hz;      json += ",";
  json += "\"muted\":"; json += (g_state.muted ? "true" : "false"); json += ",";
  json += "\"mode\":";  json += (int)g_state.mode;    json += ",";
  json += "\"rate\":";  json += g_state.rate_tenths;  json += ",";
  json += "\"bat\":";   json += (int)g_state.batt_pct;
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() { server.send(200, "text/html; charset=utf-8", htmlPage()); }
void handleGet()  { sendState(); }

void handleSet() {
  if (server.hasArg("vol"))  g_state.volume_pct = clamp_u16(server.arg("vol").toInt(), 0, 100);
  if (server.hasArg("freq")) g_state.freq_hz    = clamp_u16(server.arg("freq").toInt(), FREQ_MIN_HZ, FREQ_MAX_HZ);
  if (server.hasArg("mode")) {
    int m = server.arg("mode").toInt();
    g_state.mode = (m == 1) ? MODE_BEEP : MODE_CONTINUOUS;
  }
  if (server.hasArg("rate")) g_state.rate_tenths = clamp_u16(server.arg("rate").toInt(), 5, 100);

  g_state.muted = false;

  applyStateToPiezoDual(true);
  sendState();
}

void handleToggleMute() {
  g_state.muted = !g_state.muted;
  applyStateToPiezoDual(true);
  sendState();
}

void handleStop() {
  g_state.muted = true;
  g_state.volume_pct = 0;
  applyStateToPiezoDual(true);
  sendState();
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  if (BATTERY_ADC_PIN >= 0) {
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogReadResolution(ADC_BITS);
    (void)analogRead(BATTERY_ADC_PIN);
  }

  piezoInitDual();
  applyStateToPiezoDual(true);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);
  Serial.printf("AP SSID: %s  PASS: %s  IP: %s\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());

  server.on("/",            HTTP_GET, handleRoot);
  server.on("/get",         HTTP_GET, handleGet);
  server.on("/set",         HTTP_GET, handleSet);
  server.on("/toggle_mute", HTTP_GET, handleToggleMute);
  server.on("/stop",        HTTP_GET, handleStop);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  piezoServiceDual();
}
