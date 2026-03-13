/* =============================================================
   AWSCR Dashboard — app.js
   Real-time IoT dashboard for the AWSCR water surface cleaning robot.

   Libraries in use:
     - Leaflet.js     : map rendering
     - Leaflet.Draw   : polygon drawing tool
     - MQTT.js        : real-time telemetry via HiveMQ WebSocket
   ============================================================= */

'use strict';

// ── Configuration ────────────────────────────────────────────────────────
const MQTT_BROKER  = 'wss://broker.hivemq.com:8884/mqtt';
const TOPIC_TELEM  = 'awscr/telemetry';
const TOPIC_CMD    = 'awscr/command';
const TOPIC_MISSION= 'awscr/mission';
const CLIENT_ID    = 'awscr_dashboard_' + Math.random().toString(16).substr(2, 8);

// Default map centre (Bangalore lake)
const DEFAULT_LAT  = 13.0827;
const DEFAULT_LON  = 77.5946;
const DEFAULT_ZOOM = 18;

// ── State ─────────────────────────────────────────────────────────────────
let mqttClient   = null;
let robotMarker  = null;
let trashMarkers = [];
let pathPolyline = null;
let missionPoly  = null;
let drawnItems   = null;
let drawControl  = null;
let polygonLayer = null;
let botPathCoords= [];
let totalWPs     = 1;
let currentWP    = 0;
let trashCount   = 0;
let missionStart = null;
let timerInterval= null;

// ── Map Initialization ────────────────────────────────────────────────────
const map = L.map('map', {
  center: [DEFAULT_LAT, DEFAULT_LON],
  zoom:   DEFAULT_ZOOM,
  zoomControl: true,
  attributionControl: false
});

// Dark water tile
L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
  attribution: '©OpenStreetMap, ©CARTO',
  subdomains: 'abcd',
  maxZoom: 22
}).addTo(map);

// Draw items layer
drawnItems = new L.FeatureGroup().addTo(map);

drawControl = new L.Control.Draw({
  draw: {
    polygon: {
      allowIntersection: false,
      showArea: true,
      shapeOptions: { color: '#3b82f6', fillOpacity: 0.12, weight: 2 }
    },
    polyline: false, circle: false, circlemarker: false,
    rectangle: false, marker: false
  },
  edit: { featureGroup: drawnItems }
});

map.on(L.Draw.Event.CREATED, (e) => {
  drawnItems.clearLayers();
  drawnItems.addLayer(e.layer);
  polygonLayer = e.layer;
  logMQTT('dashboard', 'Polygon drawn (' + polygonLayer.getLatLngs()[0].length + ' vertices)');
});

// ── Robot Marker ──────────────────────────────────────────────────────────
function createRobotIcon(heading_deg) {
  const rotateDeg = heading_deg || 0;
  return L.divIcon({
    className: '',
    html: `<div style="
      width:32px;height:32px;display:flex;align-items:center;justify-content:center;
      transform:rotate(${rotateDeg}deg);font-size:22px;
      filter: drop-shadow(0 0 8px rgba(34,197,94,0.9));
      transition: transform 0.4s ease;">🚤</div>`,
    iconSize: [32, 32],
    iconAnchor: [16, 16]
  });
}

// ── Trash Markers ─────────────────────────────────────────────────────────
const trashIcon = L.divIcon({
  className:'',
  html:'<div style="font-size:18px;filter:drop-shadow(0 0 6px #f97316)">♦</div>',
  iconSize:[18,18], iconAnchor:[9,9]
});

const collectedIcon = L.divIcon({
  className:'',
  html:'<div style="font-size:16px;opacity:0.4">✔</div>',
  iconSize:[16,16], iconAnchor:[8,8]
});

// ── MQTT Setup ────────────────────────────────────────────────────────────
function connectMQTT() {
  setBadge('mqtt-badge', '⬤ Connecting…', '#f59e0b');

  mqttClient = mqtt.connect(MQTT_BROKER, {
    clientId: CLIENT_ID,
    clean: true,
    reconnectPeriod: 3000
  });

  mqttClient.on('connect', () => {
    setBadge('mqtt-badge', '⬤ MQTT Online', '#22c55e');
    mqttClient.subscribe([TOPIC_TELEM], (err) => {
      if (!err) logMQTT('sys', 'Subscribed to ' + TOPIC_TELEM);
    });
  });

  mqttClient.on('disconnect', () => setBadge('mqtt-badge', '⬤ MQTT Offline', '#ef4444'));
  mqttClient.on('error', (err) => {
    setBadge('mqtt-badge', '⬤ MQTT Error', '#ef4444');
    logMQTT('error', err.message);
  });

  mqttClient.on('message', (topic, payload) => {
    try {
      const data = JSON.parse(payload.toString());
      handleTelemetry(data);
      logMQTT(topic, payload.toString().substring(0, 100));
    } catch(e) {}
  });
}

// ── Telemetry Handler ─────────────────────────────────────────────────────
function handleTelemetry(d) {
  // Position
  const lat = d.lat, lon = d.lon;
  if (lat && lon) {
    if (!robotMarker) {
      robotMarker = L.marker([lat, lon], {icon: createRobotIcon(d.hdg)}).addTo(map);
      robotMarker.bindPopup('');
    } else {
      robotMarker.setLatLng([lat, lon]);
      robotMarker.setIcon(createRobotIcon(d.hdg));
    }
    botPathCoords.push([lat, lon]);
    if (botPathCoords.length > 500) botPathCoords.shift();
    if (pathPolyline) map.removeLayer(pathPolyline);
    pathPolyline = L.polyline(botPathCoords, {
      color: '#22d3ee', weight: 2, opacity: 0.7,
      dashArray: '5, 4'
    }).addTo(map);
    robotMarker.getPopup().setContent(
      `<b>Lat:</b> ${lat.toFixed(6)}<br><b>Lon:</b> ${lon.toFixed(6)}<br>` +
      `<b>Hdg:</b> ${(d.hdg||0).toFixed(1)}°<br><b>Mode:</b> ${d.mode||'—'}`
    );
  }

  // Text fields
  setText('t-lat', lat  ? lat.toFixed(6)+'°' : '—');
  setText('t-lon', lon  ? lon.toFixed(6)+'°' : '—');
  setText('t-hdg', d.hdg  !== undefined ? d.hdg.toFixed(1)+'°' : '—°');
  setText('t-spd', d.speed!== undefined ? d.speed.toFixed(2)+' m/s' : '—');
  setText('t-mode', d.mode || 'IDLE');

  // Waypoints & progress
  if (d.n_wp)  totalWPs    = d.n_wp;
  if (d.wp !==undefined) currentWP = d.wp;
  const pct = totalWPs > 0 ? Math.round(100 * currentWP / totalWPs) : 0;
  setText('t-wp',  currentWP + ' / ' + totalWPs);
  setText('t-cov', pct + ' %');
  document.getElementById('progress-fill').style.width = pct + '%';

  // FSM badge colour
  const fsmBadge = document.getElementById('fsm-badge');
  fsmBadge.textContent = d.mode || 'IDLE';
  fsmBadge.className   = 'status-badge mode-badge ' + fsmModeClass(d.mode);

  // Ultrasonic sensors
  const uf = d.us_f, ul = d.us_l, ur = d.us_r;
  if (uf !== undefined) { setText('s-uf', uf.toFixed(0)+' cm'); setSensorBar('sb-uf', uf, 400); }
  if (ul !== undefined) { setText('s-ul', ul.toFixed(0)+' cm'); setSensorBar('sb-ul', ul, 400); }
  if (ur !== undefined) { setText('s-ur', ur.toFixed(0)+' cm'); setSensorBar('sb-ur', ur, 400); }

  // Trash events (if any new collections happen)
  if (d.trash_collected !== undefined && d.trash_collected > trashCount) {
    const delta  = d.trash_collected - trashCount;
    trashCount   = d.trash_collected;
    setText('t-trash', trashCount.toString());
    for (let i=0; i<delta; i++) addTrashLogEntry(d.lat, d.lon, new Date().toLocaleTimeString());
  }
  if (d.trash_collected !== undefined) setText('t-trash', d.trash_collected.toString());
}

// ── UI Helpers ────────────────────────────────────────────────────────────
function setText(id, val)    { const el=document.getElementById(id); if(el) el.textContent=val; }
function setBadge(id, txt, col) { const el=document.getElementById(id); if(!el)return; el.textContent=txt; el.style.borderColor=col; el.style.color=col; }

function setSensorBar(id, val, max) {
  const el = document.getElementById(id);
  if (!el) return;
  const pct    = Math.min(100, Math.max(0, (1 - val/max) * 100));
  el.style.width = pct + '%';
  el.style.background = val < 60 ? '#ef4444' : val < 150 ? '#eab308' : 'linear-gradient(90deg,#22c55e,#22d3ee)';
}

function fsmModeClass(mode) {
  if (!mode) return '';
  if (mode.includes('ATTACK'))  return 'attack';
  if (mode.includes('RETURN'))  return 'return';
  if (mode.includes('LAWNMOW')) return 'lawnmow';
  return '';
}

function addTrashLogEntry(lat, lon, time) {
  const log  = document.getElementById('trash-log');
  const entry= document.createElement('div');
  entry.className = 'trash-log-entry';
  entry.innerHTML = `<span class="dot"></span><span>${time} — collected @ ${lat.toFixed(5)},${lon.toFixed(5)}</span>`;
  log.prepend(entry);
  if (log.children.length > 20) log.removeChild(log.lastChild);
}

function logMQTT(topic, payload) {
  const log   = document.getElementById('mqtt-log');
  const entry = document.createElement('div');
  entry.className = 'mqtt-log-entry';
  const ts    = new Date().toLocaleTimeString('en-GB', {hour12:false});
  entry.innerHTML = `<span class="ts">[${ts}]</span> <span class="topic">${topic}</span> <span class="payload">${payload}</span>`;
  log.prepend(entry);
  if (log.children.length > 100) log.removeChild(log.lastChild);
}

// ── Mission Control Buttons ───────────────────────────────────────────────
document.getElementById('btn-draw').addEventListener('click', () => {
  new L.Draw.Polygon(map, drawControl.options.draw.polygon).enable();
});

document.getElementById('btn-clear-poly').addEventListener('click', () => {
  drawnItems.clearLayers();
  polygonLayer = null;
  logMQTT('ui', 'Polygon cleared');
});

document.getElementById('btn-start').addEventListener('click', () => {
  if (!polygonLayer) {
    alert('Please draw the operation area polygon first (✏ Draw Area).');
    return;
  }
  if (!mqttClient || !mqttClient.connected) {
    alert('MQTT not connected yet.'); return;
  }
  const latlngs = polygonLayer.getLatLngs()[0];
  const polygon  = latlngs.map(p => ({ lat: p.lat, lon: p.lng }));
  const payload  = JSON.stringify({ polygon, sweep_width_m: 0.6, speed_ms: 0.6 });
  mqttClient.publish(TOPIC_MISSION, payload, {qos:1}, () => {
    logMQTT(TOPIC_MISSION, 'Mission sent (' + polygon.length + ' vertices)');
  });

  // Start timer
  missionStart = Date.now();
  if (timerInterval) clearInterval(timerInterval);
  timerInterval = setInterval(() => {
    const elapsed = Math.floor((Date.now() - missionStart)/1000);
    const h=Math.floor(elapsed/3600).toString().padStart(2,'0');
    const m=Math.floor((elapsed%3600)/60).toString().padStart(2,'0');
    const s=(elapsed%60).toString().padStart(2,'0');
    document.getElementById('time-badge').textContent=`${h}:${m}:${s}`;
  }, 1000);
});

document.getElementById('btn-stop').addEventListener('click', () => {
  if (mqttClient && mqttClient.connected) {
    mqttClient.publish(TOPIC_CMD, 'stop', {qos:1});
    logMQTT(TOPIC_CMD, 'STOP command sent');
  }
  if (timerInterval) clearInterval(timerInterval);
});

// ── Demo Mode (no hardware) ───────────────────────────────────────────────
// Inject simulated telemetry every 1s for standalone demo
function runDemoMode() {
  let t=0, wp=0, totalWP=40;
  const lat0=DEFAULT_LAT, lon0=DEFAULT_LON;
  const step_lat=0.000008, step_lon=0.000010;
  let mode_cycle = ['LAWNMOWER','LAWNMOWER','LAWNMOWER','ATTACK','RETURN_TO_PATH','LAWNMOWER'];
  let mi=0;

  setInterval(() => {
    t++;
    wp = Math.min(wp+1, totalWP);
    const lat = lat0 + Math.sin(t*0.08)*0.00015 + (t%3)*step_lat;
    const lon = lon0 + (t*step_lon*0.5);
    const spd = 0.55 + 0.1*Math.sin(t);
    const hdg = (t*8) % 360;
    const mode = mode_cycle[Math.floor(t/6) % mode_cycle.length];
    const d = {
      lat, lon, hdg, speed: spd,
      mode, wp, n_wp: totalWP,
      us_f: 100 + 80*Math.sin(t*0.3),
      us_l: 150 + 60*Math.cos(t*0.2),
      us_r: 140 + 70*Math.sin(t*0.15+1),
      trash_collected: Math.floor(t/25)
    };
    handleTelemetry(d);
    if (t===1) map.setView([lat, lon], DEFAULT_ZOOM);
  }, 500);
}

// ── Init ───────────────────────────────────────────────────────────────────
connectMQTT();

// After 3s, if no real telemetry, start demo
setTimeout(() => {
  // Only start demo if no real telemetry has come in
  if (!robotMarker) {
    logMQTT('sys', 'No hardware detected — running demo mode');
    runDemoMode();
  }
}, 3000);
