#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

/* ==========================================
 * Balance Bot Control (ESP32) - nRF24L01 Version
 * ==========================================
 */

// ===================== USER CONFIG =====================
#define DEFAULT_SSID  "AIS0"    
#define DEFAULT_PASS  "Ford8589" 

#define LED_WIFI 2   
unsigned long blinkUntil = 0; 

static inline void ledOn()  { pinMode(LED_WIFI, OUTPUT); digitalWrite(LED_WIFI, HIGH); }
static inline void ledOff() { pinMode(LED_WIFI, OUTPUT); digitalWrite(LED_WIFI, LOW);  }

bool isJoystickActive = false;
unsigned long lastJoyTime = 0;
int currentJoyX = 0;
int currentJoyY = 0;

HardwareSerial STM(2);            
static const int STM_RX = 16;    
static const int STM_TX = 17;    
static const int STM_BAUD = 115200;

static const int GPS_RX = 26;    
static const int GPS_TX = 27;
HardwareSerial SerialGPS(1);      
TinyGPSPlus gps;

double savedLat = 0.0;
double savedLng = 0.0;

void feedGPS() {
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    if (gps.location.isValid()) {
      savedLat = gps.location.lat();
      savedLng = gps.location.lng();
    }
  }
}

// ===================== nRF24L01 Config =====================
#define CE_PIN    14
#define CSN_PIN   5

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"00001", "00002"}; 

// ===================== Compass & Waypoint =====================
QMC5883LCompass compass;
bool isNavigating = false;
float currentHeading = 0.0;

#define MAX_WP 50
double wpLat[MAX_WP];
double wpLng[MAX_WP];
int wpCount = 0;
int currentWpIndex = 0;

// ===================== BLE CONFIG =====================
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ===================== Web & Prefs =====================
WebServer server(80);
Preferences preferences;

struct Telemetry {
  uint32_t ms = 0;
  float pitch = 0; float gyro  = 0;
  float pos   = 0; float vel   = 0; float u = 0; int run = 0;   
};
Telemetry g;
static char line[256];
static int  idx = 0;

// ===================== FULL HTML PAGE =====================
const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<title>Robot Control + Routing</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet-routing-machine@latest/dist/leaflet-routing-machine.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://unpkg.com/leaflet-routing-machine@latest/dist/leaflet-routing-machine.js"></script>
<style>
  body{font-family:Arial;margin:0;padding:10px;background:#f4f4f4;text-align:center}
  .box{max-width:800px;margin:0 auto;background:white;padding:10px;border-radius:10px;box-shadow:0 2px 6px rgba(0,0,0,0.1)}
  .row{display:flex;gap:5px;flex-wrap:wrap;justify-content:center;margin-bottom:10px}
  .card{padding:5px 10px;border:1px solid #ddd;border-radius:5px;min-width:60px;background:#fff;font-size:12px}
  .big{font-size:16px;font-weight:bold;color:#007bff}
  
  #joystick-container { position: relative; width: 200px; height: 200px; margin: 10px auto; background: #eee; border-radius: 50%; border: 4px solid #ccc; touch-action: none; }
  #stick { position: absolute; width: 80px; height: 80px; background: linear-gradient(to bottom, #4facfe 0%, #00f2fe 100%); border-radius: 50%; top: 50%; left: 50%; transform: translate(-50%, -50%); box-shadow: 0 4px 10px rgba(0,0,0,0.3); }

  .btn-send { background: #28a745; color: white; border: none; padding: 8px 12px; border-radius: 5px; cursor: pointer; }
  .btn-route { background: #ff9800; color: white; border: none; padding: 8px 12px; border-radius: 5px; cursor: pointer; font-weight: bold; }
  
  #map { width: 100%; height: 300px; border-radius: 8px; margin-top: 15px; z-index: 1; border: 1px solid #ccc; }
  .leaflet-routing-container { display: none !important; }
  button{padding:8px 12px;border-radius:5px;border:1px solid #ccc;background:#eee;cursor:pointer;margin:2px}
</style>
</head>
<body>
<div class="box">
  <h3>Balance Robot Control</h3>
  <div style="font-size:12px;color:#555">IP: <span id="ipAddr">...</span> | GPS: <span id="gpsData">No Fix</span> (Sats: <span id="sats">0</span>)</div>
  
  <div id="joystick-container"><div id="stick"></div></div>
  
  <div class="row">
    <button onclick="sendCmdStr('S1')">START (S1)</button>
    <button onclick="sendCmdStr('S0')" style="color:red">STOP (S0)</button>
  </div>
  
  <div style="margin-top: 15px;">
     <p style="font-size: 13px; color: #555; margin-bottom: 5px;">📍 แตะบนแผนที่เพื่อสร้างเส้นทางนำทางอัตโนมัติ</p>
     <button class="btn-route" onclick="sendRoute()">🚀 SEND ROUTE</button>
     <button class="btn-send" style="background: #dc3545;" onclick="clearRoute()">🗑️ Clear</button>
  </div>
  <div id="map"></div>

  <hr>
  <div class="row">
    <div class="card">Pitch<div class="big" id="p">0.00</div></div>
    <div class="card">Speed<div class="big" id="v">0.00</div></div>
    <div class="card">Pos<div class="big" id="pos">0.00</div></div>
  </div>
</div>

<script>
var map = L.map('map').setView([13.7563, 100.5018], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

var botIcon = L.icon({
    iconUrl: 'https://cdn-icons-png.flaticon.com/512/622/622868.png',
    iconSize: [32, 32], iconAnchor: [16, 16]
});
var botMarker = L.marker([13.7563, 100.5018], {icon: botIcon}).addTo(map);

var firstLoc = true;
var botLatLng = null;
var routeCoords = []; 

var routingControl = L.Routing.control({
    waypoints: [],
    routeWhileDragging: true,
    addWaypoints: true,
    show: false,
    createMarker: function() { return null; }
}).addTo(map);

routingControl.on('routesfound', function(e) {
    var routes = e.routes;
    routeCoords = routes[0].coordinates; 
});

map.on('click', function(e) {
    if(botLatLng) {
        routingControl.setWaypoints([
            L.latLng(botLatLng.lat, botLatLng.lng),
            e.latlng 
        ]);
    } else {
        alert("Waiting for GPS signal...");
    }
});

function clearRoute() {
    routingControl.setWaypoints([]);
    routeCoords = [];
}

function sendRoute() {
    if(routeCoords.length === 0) { alert("Please click on map to set a route first!"); return; }
    var maxPts = 45;
    var step = Math.ceil(routeCoords.length / maxPts);
    var routeStr = "";
    for(var i=0; i<routeCoords.length; i+=step) {
        routeStr += routeCoords[i].lat.toFixed(6) + "," + routeCoords[i].lng.toFixed(6) + ";";
    }
    if(routeCoords.length > 0 && (routeCoords.length-1) % step !== 0) {
        var last = routeCoords[routeCoords.length-1];
        routeStr += last.lat.toFixed(6) + "," + last.lng.toFixed(6) + ";";
    }

    fetch('/route', { method: 'POST', body: routeStr })
      .then(r => r.text())
      .then(t => alert(t))
      .catch(e => alert("Error sending route"));
}

function sendCmdStr(str) { fetch('/tune?c=' + encodeURIComponent(str)); }

const stick = document.getElementById('stick');
const container = document.getElementById('joystick-container');
const maxDist = 60; let isDragging = false; let lastSend = 0;
function handleMove(event) {
  if (!isDragging) return;
  event.preventDefault();
  const touch = event.targetTouches ? event.targetTouches[0] : event;
  const rect = container.getBoundingClientRect();
  let x = touch.clientX - rect.left - rect.width / 2;
  let y = touch.clientY - rect.top - rect.height / 2;
  const dist = Math.sqrt(x*x + y*y);
  if (dist > maxDist) { x = (x / dist) * maxDist; y = (y / dist) * maxDist; }
  stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
  const valX = Math.round((x / maxDist) * 100);
  const valY = Math.round(-(y / maxDist) * 100);
  sendJoy(valX, valY);
}
function startDrag(e) { isDragging = true; handleMove(e); }
function endDrag() { isDragging = false; stick.style.transform = `translate(-50%, -50%)`; sendJoy(0, 0); }
function sendJoy(x, y) {
  const now = Date.now();
  if (now - lastSend < 80 && (x!==0 || y!==0)) return; 
  lastSend = now; fetch('/cmd?v=' + x + ',' + y).catch(e=>{});
}
container.addEventListener('mousedown', startDrag); document.addEventListener('mousemove', handleMove); document.addEventListener('mouseup', endDrag);
container.addEventListener('touchstart', startDrag, {passive: false}); container.addEventListener('touchmove', handleMove, {passive: false}); document.addEventListener('touchend', endDrag);

async function poll() {
  try {
    const r = await fetch('/data');
    const j = await r.json();
    document.getElementById('p').innerText = j.pitch.toFixed(2);
    document.getElementById('v').innerText = j.vel.toFixed(2);
    document.getElementById('pos').innerText = j.pos.toFixed(1);
    document.getElementById('ipAddr').innerText = j.ip;
    if(j.sats !== undefined) document.getElementById('sats').innerText = j.sats;

    if (j.lat !== 0 && j.lng !== 0) {
        document.getElementById('gpsData').innerText = j.lat.toFixed(6) + ", " + j.lng.toFixed(6);
        botLatLng = { lat: j.lat, lng: j.lng };
        var newLatLng = new L.LatLng(j.lat, j.lng);
        botMarker.setLatLng(newLatLng);
        if(firstLoc) { map.setView(newLatLng, 17); firstLoc = false; } 
    }
  } catch(e){}
  setTimeout(poll, 150); 
}
poll();
</script>
</body>
</html>
)HTML";

static bool parseLineToTelemetry(char* s, Telemetry &out) {
  char* tok[20] = {0}; int n = 0;
  tok[n++] = s;
  for (char* p=s; *p && n < 20; p++) { if (*p == ',') { *p = 0; tok[n++] = p+1; } }
  if (n < 3) return false;
  out.ms = strtoul(tok[0], NULL, 10); 
  out.pitch = strtof(tok[1], NULL); out.gyro = strtof(tok[2], NULL);
  if (n >= 7) { 
    out.pos = strtof(tok[3], NULL); out.vel = strtof(tok[4], NULL); 
    out.u = strtof(tok[5], NULL); out.run = strtol(tok[6], NULL, 10); 
  }
  return true;
}

static bool connectSTAFromPrefs() {
  preferences.begin("wifi-conf", true);
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("pass", "");
  preferences.end();
  if (ssid == "") { ssid = DEFAULT_SSID; pass = DEFAULT_PASS; }
  WiFi.mode(WIFI_STA); WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while(millis()-t0 < 10000) {
    if(WiFi.status() == WL_CONNECTED) { ledOn(); return true; }
    delay(100); ledOn(); 
  }
  return false;
}

// ===================== Web Handlers =====================
void handleRoot() { server.send(200, "text/html", PAGE); }
void handleData() {
  String json = "{";
  json += "\"pitch\":" + String(g.pitch) + ",";
  json += "\"vel\":" + String(g.vel) + ",";
  json += "\"pos\":" + String(g.pos) + ",";
  json += "\"lat\":" + String(savedLat, 6) + ",";
  json += "\"lng\":" + String(savedLng, 6) + ",";
  json += "\"sats\":" + String(gps.satellites.value()) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleCmd() { 
  if (server.hasArg("v")) { 
    isNavigating = false; 
    String val = server.arg("v"); 
    int comma = val.indexOf(',');
    if (comma > 0) {
        currentJoyX = val.substring(0, comma).toInt();
        currentJoyY = val.substring(comma + 1).toInt();
        lastJoyTime = millis();
    }
    server.send(200, "text/plain", "OK"); 
  } else server.send(400, "text/plain", "BAD"); 
}

void handleTune() { if (server.hasArg("c")) { String c = server.arg("c"); STM.println(c); server.send(200, "text/plain", "OK"); } else server.send(400, "text/plain", "BAD"); }

void handleRoute() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    wpCount = 0;
    int startIdx = 0;
    while (startIdx < body.length() && wpCount < MAX_WP) {
      int semiColonIdx = body.indexOf(';', startIdx);
      if (semiColonIdx == -1) semiColonIdx = body.length();
      String pair = body.substring(startIdx, semiColonIdx);
      int commaIdx = pair.indexOf(',');
      if (commaIdx > 0) {
        wpLat[wpCount] = pair.substring(0, commaIdx).toDouble();
        wpLng[wpCount] = pair.substring(commaIdx + 1).toDouble();
        wpCount++;
      }
      startIdx = semiColonIdx + 1;
    }
    currentWpIndex = 0; 
    if (wpCount > 0) isNavigating = true; 
    server.send(200, "text/plain", "Route OK");
  } else {
    server.send(400, "text/plain", "No Data");
  }
}

// ===================== BLE Handlers =====================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        String btCmd = String(rxValue.c_str());
        btCmd.trim();
        if (btCmd.startsWith("WIFI:")) {
           // Wifi Setup...
        } else {
           STM.println(btCmd);
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  STM.begin(STM_BAUD, SERIAL_8N1, STM_RX, STM_TX);
  
  pinMode(LED_WIFI, OUTPUT); 
  digitalWrite(LED_WIFI, HIGH);

  Wire.begin(21, 22);
  // หากไม่ได้ต่อเข็มทิศ ให้ comment บรรทัด compass.init() ออกชั่วคราว เพื่อป้องกันบอร์ดค้าง
  compass.init();

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Setup nRF24L01
  if (!radio.begin()) {
    Serial.println("nRF24 Error!");
  } else {
    Serial.println("nRF24 OK!");
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.openWritingPipe(address[1]); // หุ่นยนต์เขียนลงท่อ 1
    radio.openReadingPipe(1, address[0]); // หุ่นยนต์อ่านจากท่อ 0
    radio.startListening();
  }
  
  BLEDevice::init("BalanceBot_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  if(!connectSTAFromPrefs()) {
    Serial.println("WiFi STA Failed. Starting AP Mode...");
    WiFi.softAP("RobotControl");
  } else {
    Serial.println("WiFi Connected!");
  }
  
  ledOn();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd); 
  server.on("/tune", handleTune);
  server.on("/route", HTTP_POST, handleRoute); 
  server.begin();
}

void loop() {
  static unsigned long lastWifiCheck = 0;
  if (WiFi.getMode() == WIFI_STA && millis() - lastWifiCheck > 10000) {
      lastWifiCheck = millis();
      if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }

  feedGPS(); 
  server.handleClient();
  
  if (millis() < blinkUntil) {
     if ((millis() / 50) % 2) ledOff(); else ledOn();
  } else {
     ledOn(); 
  }

  if (!deviceConnected && oldDeviceConnected) { delay(500); pServer->startAdvertising(); oldDeviceConnected = deviceConnected; }
  if (deviceConnected && !oldDeviceConnected) { oldDeviceConnected = deviceConnected; }

  feedGPS(); 

  // ======================================================================
  // 🎮 JOYSTICK & NAVIGATION CONTROL LOOP
  // ======================================================================
  static unsigned long lastControlLoop = 0;
  
  if (millis() - lastJoyTime > 500) {
      isJoystickActive = false;
      currentJoyX = 0; currentJoyY = 0;
  } else {
      isJoystickActive = true;
  }

  if (millis() - lastControlLoop > 50) {
      lastControlLoop = millis();

      if (!isNavigating) { 
          if (!isJoystickActive) {
              STM.println("*0,0#");
              // 🔍 DEBUG: ปริ้นท์เพื่อให้รู้ว่าบอร์ดยังวนลูปและส่งค่าตอนอยู่นิ่งๆ (ปริ้นท์ทุก 1 วินาที)
              static unsigned long lastIdlePrint = 0;
              if (millis() - lastIdlePrint > 1000) {
                  Serial.println(">> Sent to STM32: *0,0# (Idle)");
                  lastIdlePrint = millis();
              }
          } else {
              // ====================================================
              // 🚀 1. สลับแกนตามต้องการ & ขยายความแรง (Amplify)
              // X จากรีโมท = เดินหน้า/ถอยหลัง (ดันสุดส่ง 100)
              // Y จากรีโมท = เลี้ยวซ้าย/ขวา (ดันสุดส่ง 100)
              // ====================================================
              int forwardCmd = currentJoyX * 12; // 100 * 12 = 1200 (องศาการเอียง ~18 องศา พุ่งแรงมาก)
              int turnCmd    = currentJoyY * 6;  // 100 * 6 = 600 (ความเร็วในการกลับตัว)

              // 🚀 ปลดล็อค Limit ให้กว้างขึ้น หุ่นจะได้ไม่อั้นความเร็ว
              if (forwardCmd > 1500) forwardCmd = 1500;
              if (forwardCmd < -1500) forwardCmd = -1500;
              if (turnCmd > 800) turnCmd = 800;
              if (turnCmd < -800) turnCmd = -800;

              // 🚀 2. นำไปคำนวณ Velocity Limit (กันหน้าทิ่มเมื่อวิ่งเร็วจัด)
              int finalSpeed = forwardCmd;
              const float MAX_SAFE_VEL = 40.0; 

              if (g.vel > MAX_SAFE_VEL && forwardCmd > 0) {
                  finalSpeed = forwardCmd - (int)((g.vel - MAX_SAFE_VEL) * 20.0);
                  if (finalSpeed < 0) finalSpeed = 0; 
              }
              else if (g.vel < -MAX_SAFE_VEL && forwardCmd < 0) {
                  finalSpeed = forwardCmd - (int)((g.vel + MAX_SAFE_VEL) * 20.0);
                  if (finalSpeed > 0) finalSpeed = 0;
              }
              
              // 🚀 3. ส่งคำสั่งไปหา STM32 
              // รูปแบบที่ STM32 รับคือ: *เดินหน้า,เลี้ยว# (แก้สลับตัวแปรให้ตรงกันแล้ว)
              STM.printf("*%d,%d#\n", finalSpeed, turnCmd);

              // 🔍 DEBUG: ปริ้นท์ยืนยัน
              Serial.printf(">> Sent to STM32: *%d,%d#\n", finalSpeed, turnCmd);
          }
      }
  }

  // --- Compass & Waypoint Auto Navigation Loop ---
  compass.read();
  currentHeading = compass.getAzimuth(); 

  if (isNavigating && gps.location.isValid() && wpCount > 0 && currentWpIndex < wpCount) {
     double targetLat = wpLat[currentWpIndex];
     double targetLng = wpLng[currentWpIndex];
     double distToTarget = TinyGPSPlus::distanceBetween(savedLat, savedLng, targetLat, targetLng);
     double targetBearing = TinyGPSPlus::courseTo(savedLat, savedLng, targetLat, targetLng);

     if (distToTarget < 2.0) { 
         currentWpIndex++;
         if (currentWpIndex >= wpCount) {
             isNavigating = false;
             STM.print("*0,0#\n"); 
         }
     } else {
         float headingError = targetBearing - currentHeading;
         if (headingError > 180) headingError -= 360;
         if (headingError < -180) headingError += 360;

         int baseSpeed = 20; 
         int turnSpeed = (int)(headingError * 0.8); 
         if(turnSpeed > 40) turnSpeed = 40;
         if(turnSpeed < -40) turnSpeed = -40;

         // แก้การส่งข้อมูลตอน Auto Nav ด้วย (เดินหน้า, เลี้ยว)
         STM.printf("*%d,%d#\n", baseSpeed, turnSpeed);
         delay(50); 
     }
  }

  feedGPS(); 

  // --- Heartbeat Logic สำหรับ nRF24L01 ---
  static unsigned long lastHBT = 0;
  if (millis() - lastHBT > 1000) {
      lastHBT = millis();
      char hbMsg[32];
      sprintf(hbMsg, "HB:%s", gps.location.isValid() ? "1" : "0");
      
      radio.stopListening();
      radio.write(&hbMsg, strlen(hbMsg) + 1);
      radio.startListening();
  }

  // --- nRF24L01 Receive ---
  if (radio.available()) {
    feedGPS(); 
    char packetBuffer[32] = {0};
    radio.read(&packetBuffer, sizeof(packetBuffer));
    String packet = String(packetBuffer);
    packet.trim();

    // ==========================================
    // 🔍 ปริ้นท์ค่าที่รับได้จาก nRF24L01 ออกทาง Monitor
    Serial.print("[nRF24 RX] Received: ");
    Serial.println(packet);
    // ==========================================

    if (packet.startsWith("C,")) {
       int first = packet.indexOf(',');
       String valStr = packet.substring(first + 1); 
       
       int comma = valStr.indexOf(',');
       if(comma > 0) {
           currentJoyX = valStr.substring(0, comma).toInt();
           currentJoyY = valStr.substring(comma + 1).toInt();
           lastJoyTime = millis();
       }
       isNavigating = false; 
    }
    else if (packet.equals("IP?")) {
       blinkUntil = millis() + 1000;
       String ipData = "IP:" + WiFi.localIP().toString();
       
       radio.stopListening();
       for(int i=0; i<3; i++) { // ส่งย้ำเผื่อข้อมูลตกหล่น
           radio.write(ipData.c_str(), ipData.length() + 1);
           delay(30);
       }
       radio.startListening();
    }
    else if (packet.equals("GET")) {
       blinkUntil = millis() + 1000;
       String gpsData;
       if (gps.location.isValid()) gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
       else if (savedLat != 0.0) gpsData = String(savedLat, 6) + "," + String(savedLng, 6);
       else gpsData = "NO_GPS";
       
       radio.stopListening();
       for(int i=0; i<3; i++) { 
           radio.write(gpsData.c_str(), gpsData.length() + 1);
           delay(30);
       }
       radio.startListening();
    }
  }

  feedGPS(); 

  // รับค่าจาก STM32 (Balance Controller)
  while (STM.available()) {
    char c = (char)STM.read();
    if (c == '\n') {
      line[idx] = 0;

      // ==========================================
      // 🔍 DEBUG: ให้ ESP32 ปริ้นท์สิ่งที่ STM32 ส่งกลับมา
      // (ปริ้นท์ทุกๆ 1 วินาที เพื่อไม่ให้ข้อความไหลเร็วเกินไป)
      static unsigned long lastStmPrint = 0;
      if (millis() - lastStmPrint > 1000) { 
          Serial.print("[From STM32] Telemetry: ");
          Serial.println(line);
          lastStmPrint = millis();
      }
      // ==========================================

      Telemetry t;
      if (parseLineToTelemetry(line, t)) { g = t; }
      idx = 0;
    } else if (c != '\r') {
      if (idx < 250) line[idx++] = c;
    }
  }

  feedGPS(); 
  while (Serial.available()) { char c = (char)Serial.read(); STM.write(c); }
}