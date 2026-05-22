/*
- ขยายรัศมีเป้าหมายการเดินเรือจาก 2.0 เป็น 3.5 เมตร (distToWp < 3.5)
- ปรับขนาด Marker เส้นทางในแผนที่จากรัศมี 4 เป็น 8 เพื่อให้เห็นชัดเจนบนถนน
*/
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
#include <ESP32Servo.h>

#define DEFAULT_SSID  "AIS0"    
#define DEFAULT_PASS  "Ford8589" 
#define LED_WIFI 2   

#define TRIG_PIN 12
#define ECHO_PIN 13

float frontDist_cm = 999.0f; 
unsigned long lastSonarMs = 0;

unsigned long blinkUntil = 0; 
unsigned long lastNrfRxTime = 0;

static inline void ledOn()  { 
  pinMode(LED_WIFI, OUTPUT); 
  digitalWrite(LED_WIFI, HIGH); 
}

static inline void ledOff() { 
  pinMode(LED_WIFI, OUTPUT); 
  digitalWrite(LED_WIFI, LOW);  
}

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

// 💡 ตัวแปรสำหรับชดเชยตำแหน่ง GPS (Offset) ปรับผ่านหน้าเว็บ
double latOffset = 0.000000; 
double lngOffset = 0.000000; 
const double OFFSET_STEP = 0.000010;

const int QMC5883P_ADDR = 0x2C; 
float x_offset = 1693.00; 
float y_offset = 1428.00;

float headingOffset = 275.0f; 

bool isNavigating = false; 
float currentHeading = 0.0; 
bool isAligningNorth = false; 
int lastRunState = 0;

float currentNavDist = 0.0f; 
float currentNavCourse = 0.0f; 

#define MAX_WP 50
double wpLat[MAX_WP]; 
double wpLng[MAX_WP]; 
int wpCount = 0; 
int currentWpIndex = 0;

void initCompass() { 
  Wire.beginTransmission(QMC5883P_ADDR); 
  Wire.write(0x0B); 
  Wire.write(0x01); 
  Wire.endTransmission(); 
  
  Wire.beginTransmission(QMC5883P_ADDR); 
  Wire.write(0x0A); 
  Wire.write(0x1D); 
  Wire.endTransmission(); 
}

float getCompassHeading() { 
  Wire.beginTransmission(QMC5883P_ADDR); 
  Wire.write(0x00); 
  Wire.endTransmission(false); 
  
  if (Wire.requestFrom(QMC5883P_ADDR, 7) == 7) { 
    uint8_t status = Wire.read(); 
    int16_t x = Wire.read() | (Wire.read() << 8); 
    int16_t y = Wire.read() | (Wire.read() << 8); 
    int16_t z = Wire.read() | (Wire.read() << 8); 
    
    float x_cal = x - x_offset; 
    float y_cal = y - y_offset; 
    
    static float filter_x = 0.0f;
    static float filter_y = 0.0f;
    
    if (filter_x == 0.0f && filter_y == 0.0f) {
        filter_x = x_cal;
        filter_y = y_cal;
    }
    
    const float COMPASS_ALPHA = 0.10f; 
    filter_x = (1.0f - COMPASS_ALPHA) * filter_x + COMPASS_ALPHA * x_cal;
    filter_y = (1.0f - COMPASS_ALPHA) * filter_y + COMPASS_ALPHA * y_cal;

    float heading = atan2(-filter_y, filter_x) * 180.0 / PI; 
    
    heading += headingOffset;
    while (heading < 0) heading += 360.0; 
    while (heading >= 360.0) heading -= 360.0; 
    
    return heading; 
  } 
  return currentHeading; 
}

void readSonar() {
  if (millis() - lastSonarMs > 100) { 
    lastSonarMs = millis();
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 15000); 
    
    if (duration == 0) {
      frontDist_cm = 999.0f; 
    } else {
      frontDist_cm = (duration * 0.0343f) / 2.0f;
    }
  }
}

#define OLED_ADDR 0x3C
uint8_t oledBuf[1024]; 
bool oledOK = false; 
uint32_t lastEyeMs = 0;

void oledCmd(uint8_t c) { 
  Wire.beginTransmission(OLED_ADDR); 
  Wire.write(0x00); 
  Wire.write(c); 
  Wire.endTransmission(); 
}

void oledInit() { 
  delay(80); 
  oledCmd(0xAE); oledCmd(0x02); oledCmd(0x10); oledCmd(0x40); 
  oledCmd(0xB0); oledCmd(0x81); oledCmd(0x80); oledCmd(0xA1); 
  oledCmd(0xA6); oledCmd(0xA8); oledCmd(0x3F); oledCmd(0xAD); 
  oledCmd(0x8B); oledCmd(0x33); oledCmd(0xC8); oledCmd(0xD3); 
  oledCmd(0x00); oledCmd(0xD5); oledCmd(0x80); oledCmd(0xD9); 
  oledCmd(0x1F); oledCmd(0xDA); oledCmd(0x12); oledCmd(0xDB); 
  oledCmd(0x40); oledCmd(0xAF); 
  memset(oledBuf, 0, sizeof(oledBuf)); 
  oledOK = true; 
}

void oledUpdate() { 
  if (!oledOK) return; 
  for (uint8_t page = 0; page < 8; page++) { 
    oledCmd(0xB0 | page); 
    oledCmd(0x02); 
    oledCmd(0x10); 
    for (uint8_t col = 0; col < 128; col += 16) { 
      Wire.beginTransmission(OLED_ADDR); 
      Wire.write(0x40); 
      for (uint8_t i = 0; i < 16; i++) {
        Wire.write(oledBuf[page * 128 + col + i]); 
      }
      Wire.endTransmission(); 
    } 
  } 
}

void oledSetPixel(int x, int y, bool on) { 
  if (x < 0 || x >= 128 || y < 0 || y >= 64) return; 
  if(on) {
    oledBuf[(y >> 3) * 128 + x] |= (1 << (y & 7)); 
  } else {
    oledBuf[(y >> 3) * 128 + x] &= ~(1 << (y & 7)); 
  }
}

void oledDrawLine(int x0, int y0, int x1, int y1, bool on) { 
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1; 
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
  int err = dx + dy, e2; 
  for (;;) { 
    oledSetPixel(x0, y0, on); 
    if (x0 == x1 && y0 == y1) break; 
    e2 = 2 * err; 
    if (e2 >= dy) { err += dy; x0 += sx; } 
    if (e2 <= dx) { err += dx; y0 += sy; } 
  } 
}

void drawEyeSimple(int cx, int cy, int r, int offY) { 
  int rr = r * r; 
  for(int dy = -r; dy <= r; dy++) { 
    for(int dx = -r; dx <= r; dx++) { 
      if(dx*dx + dy*dy <= rr) oledSetPixel(cx+dx, cy+dy, true); 
    } 
  } 
  int pr = r - 3; 
  int prr = pr * pr; 
  for(int dy = -pr; dy <= pr; dy++) { 
    for(int dx = -pr; dx <= pr; dx++) { 
      if(dx*dx + dy*dy <= prr) oledSetPixel(cx+dx, cy+dy+offY, false); 
    } 
  } 
}

void drawEyeCross(int cx, int cy, int s) { 
  for(int i = -s; i <= s; i++) { 
    oledSetPixel(cx+i, cy-i, 1); 
    oledSetPixel(cx+i, cy-i+1, 1); 
    oledSetPixel(cx+i, cy+i, 1); 
    oledSetPixel(cx+i, cy+i+1, 1); 
  } 
}

void drawNumber(int x, int y, int num) { 
  const uint8_t font[10][3] = { 
    {0x1F, 0x11, 0x1F}, {0x00, 0x1F, 0x00}, {0x1D, 0x15, 0x17}, 
    {0x15, 0x15, 0x1F}, {0x07, 0x04, 0x1F}, {0x17, 0x15, 0x1D}, 
    {0x1F, 0x15, 0x1D}, {0x01, 0x01, 0x1F}, {0x1F, 0x15, 0x1F}, 
    {0x17, 0x15, 0x1F} 
  }; 
  
  if (num == 0) { 
    for(int c=0; c<3; c++) {
      for(int r=0; r<5; r++) {
        if((font[0][c]>>r)&1) oledSetPixel(x+c, y+r, 1); 
      }
    }
    return; 
  } 
  
  int digits[10]; 
  int cnt = 0; 
  int temp = num; 
  while (temp > 0) { 
    digits[cnt++] = temp % 10; 
    temp /= 10; 
  } 
  
  int cx = x; 
  for (int i = cnt - 1; i >= 0; i--) { 
    int d = digits[i]; 
    for (int c = 0; c < 3; c++) { 
      for (int r = 0; r < 5; r++) { 
        if ((font[d][c] >> r) & 1) oledSetPixel(cx + c, y + r, 1); 
      } 
    } 
    cx += 4; 
  } 
}

void drawGPSStatus() { 
  oledSetPixel(2, 6, 1); oledSetPixel(3, 6, 1); oledSetPixel(4, 6, 1); 
  oledSetPixel(3, 5, 1); oledSetPixel(3, 4, 1); oledSetPixel(3, 3, 1); 
  oledSetPixel(1, 2, 1); oledSetPixel(2, 1, 1); oledSetPixel(3, 1, 1); 
  oledSetPixel(4, 1, 1); oledSetPixel(5, 2, 1); oledSetPixel(3, 2, 1); 
  
  if ((millis() / 500) % 2) oledSetPixel(3, 0, 1); 
  
  if (gps.charsProcessed() < 10 && millis() > 3000) { 
    if ((millis() / 250) % 2) drawEyeCross(12, 4, 2); 
  } else { 
    drawNumber(9, 2, gps.satellites.value()); 
    if (gps.location.isValid()) { 
      oledSetPixel(22, 4, 1); oledSetPixel(23, 5, 1); 
      oledSetPixel(24, 4, 1); oledSetPixel(25, 3, 1); 
      if ((millis() / 500) % 2) oledSetPixel(26, 2, 1); 
    } else { 
      int step = (millis() / 300) % 4; 
      if (step >= 1) oledSetPixel(22, 5, 1); 
      if (step >= 2) oledSetPixel(24, 5, 1); 
      if (step >= 3) oledSetPixel(26, 5, 1); 
    } 
  } 
}

void drawCompass(int cx, int cy, int r, float heading) { 
  oledSetPixel(cx, cy - r, 1); 
  oledSetPixel(cx, cy + r, 1); 
  oledSetPixel(cx - r, cy, 1); 
  oledSetPixel(cx + r, cy, 1); 
  float rad = (-90.0 - heading) * 3.14159265f / 180.0f; 
  int nx = cx + (int)(r * cos(rad)); 
  int ny = cy + (int)(r * sin(rad)); 
  oledDrawLine(cx, cy, nx, ny, 1); 
  oledSetPixel(cx, cy, 0); 
}

void updateEyesFromAngle(float ang, int runState) { 
  if(!oledOK) return; 
  memset(oledBuf, 0, sizeof(oledBuf)); 
  int offY = (int)(ang * 0.3f); 
  
  if(offY > 4) offY = 4; 
  if(offY < -4) offY = -4; 
  
  if(abs(ang) > 45.0f && runState == 0) { 
    drawEyeCross(40, 32, 10); 
    drawEyeCross(88, 32, 10); 
  } else { 
    drawEyeSimple(40, 32, 10, offY); 
    drawEyeSimple(88, 32, 10, offY); 
  } 
  
  drawGPSStatus(); 
  drawCompass(114, 12, 8, currentHeading); 
  oledUpdate(); 
}

void feedGPS() { 
  while (SerialGPS.available() > 0) { 
    char c = SerialGPS.read(); 
    gps.encode(c); 
    if (gps.location.isValid()) { 
      double newLat = gps.location.lat() + latOffset; 
      double newLng = gps.location.lng() + lngOffset; 
      
      if (savedLat == 0.0 && savedLng == 0.0) {
        savedLat = newLat;
        savedLng = newLng;
      } else {
        double distChange = TinyGPSPlus::distanceBetween(savedLat, savedLng, newLat, newLng);
        if (distChange > 1.0) { 
            savedLat = (0.85 * savedLat) + (0.15 * newLat);
            savedLng = (0.85 * savedLng) + (0.15 * newLng);
        }
      }
    } 
  } 
}

#define CE_PIN    14
#define CSN_PIN   5
RF24 radio(CE_PIN, CSN_PIN); 
const byte address[][6] = {"00001", "00002"}; 
bool nrfOK = false;

#define SERVO1_PIN 25 
#define SERVO2_PIN 4  

int trimTilt = 0;  // 💡 ปรับจาก 2 เป็น 4 เพื่อให้เงยหน้าขึ้นอีก 2 องศา

Servo tiltLeftServo; 
Servo tiltRightServo; 

float currentTiltAngle = 90.0; 
float targetTiltAngle  = 90.0; 
unsigned long lastServoMoveMs = 0; 
const int SERVO_SPEED_MS = 5; 
const float SERVO_STEP = 3.0; 

float baseAngleOffset = 3.0f; 
float cgMultiplier = 0.025f;   

void calculateAndSendCG() {
  float tiltWeight = (90.0f - currentTiltAngle) * 0.5f; 
  float cgOffset = tiltWeight * cgMultiplier; 
  float newTargetAngle = baseAngleOffset + cgOffset;
  STM.printf("A%.2f\n", newTargetAngle);
}

BLEServer *pServer = NULL; 
BLECharacteristic *pTxCharacteristic; 
bool deviceConnected = false; 
bool oldDeviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

WebServer server(80); 
Preferences preferences;

struct Telemetry {
  uint32_t ms = 0; 
  float pitch = 0; 
  float gyro = 0;
  float pos = 0; 
  float vel = 0; 
  float u = 0; 
  int run = 0;   
  int rxOk = 0; 
  float sp = 0; 
};

Telemetry g;
static char line[256]; 
static int idx = 0;
String lastCmdToSTM = "WAITING..."; 
unsigned long lastStmRxTime = 0; 

const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<title>Robot AI Command</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet-routing-machine@latest/dist/leaflet-routing-machine.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://unpkg.com/leaflet-routing-machine@latest/dist/leaflet-routing-machine.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  body { font-family: Arial; margin: 0; padding: 10px; background: #f4f4f4; text-align: center; }
  .box { max-width: 800px; margin: 0 auto; background: white; padding: 10px; border-radius: 10px; box-shadow: 0 2px 6px rgba(0,0,0,0.1); }
  .row { display: flex; gap: 5px; flex-wrap: wrap; justify-content: center; margin-bottom: 10px; }
  .card { padding: 5px 10px; border: 1px solid #ddd; border-radius: 5px; min-width: 60px; background: #fff; font-size: 12px; }
  .big { font-size: 16px; font-weight: bold; color: #007bff; }
  .dist-warning { color: #dc3545 !important; font-weight: bold; } 
  #joystick-container { position: relative; width: 200px; height: 200px; margin: 10px auto; background: #eee; border-radius: 50%; border: 4px solid #ccc; touch-action: none; }
  #stick { position: absolute; width: 80px; height: 80px; background: linear-gradient(to bottom, #4facfe 0%, #00f2fe 100%); border-radius: 50%; top: 50%; left: 50%; transform: translate(-50%, -50%); box-shadow: 0 4px 10px rgba(0,0,0,0.3); }
  .btn-send { background: #28a745; color: white; border: none; padding: 8px 12px; border-radius: 5px; cursor: pointer; font-weight: bold; }
  .btn-pause { background: #6c757d; color: white; border: none; padding: 8px 12px; border-radius: 5px; cursor: pointer; font-weight: bold; margin-bottom: 10px; width: 100%; max-width: 200px;}
  #map { width: 100%; height: 250px; border-radius: 8px; margin-top: 15px; z-index: 1; border: 1px solid #ccc; cursor: crosshair;}
  .leaflet-routing-container { display: none !important; }
  button { padding: 8px 12px; border-radius: 5px; border: 1px solid #ccc; background: #eee; cursor: pointer; margin: 2px; }
  .ai-box { background: #e3f2fd; border-radius: 8px; padding: 10px; margin-top: 15px; border: 1px solid #90caf9;}
  .ai-log { height: 100px; overflow-y: auto; background: white; padding: 8px; border-radius: 4px; font-size: 14px; text-align: left; margin-bottom: 10px; border: 1px solid #ccc;}
  .ai-log b { color: #007bff; }
  .ai-btn-mic { background: #673ab7; color: white; border: none; padding: 8px 15px; border-radius: 5px; cursor: pointer; font-weight: bold;}
  .ai-btn-auto { background: #17a2b8; color: white; border: none; padding: 8px 15px; border-radius: 5px; cursor: pointer; font-weight: bold; width: 100%; margin-top: 10px;}
</style>
</head>
<body>
<div class="box">
  <h3>Balance Robot Control</h3>
  <div style="font-size:12px;color:#555">IP: <span id="ipAddr">...</span> | GPS: <span id="gpsData">No Fix</span></div>
  
  <div id="joystick-container"><div id="stick"></div></div>
  
  <div class="row">
    <button onclick="sendCmdStr('S1')">START (S1)</button>
    <button onclick="sendCmdStr('S0')" style="color:red">STOP (S0)</button>
  </div>
  
  <div class="row">
    <div class="card">Pitch<div class="big" id="p">0.00</div></div>
    <div class="card">Speed<div class="big" id="v">0.00</div></div>
    <div class="card">PWM<div class="big" id="u">0.00</div></div>
    <div class="card">Heading<div class="big" id="yaw">0&deg;</div></div>
    <div class="card">Sonar<div class="big" id="dist">0 cm</div></div>
  </div>

  <hr>
  <div class="ai-box">
    <h4 style="margin:0 0 10px 0;">🤖 AI Copilot (ผู้ช่วยนำทาง)</h4>
    <div class="ai-log" id="aiLog">
       <i>ระบบ AI พร้อมทำงาน...</i><br>
       <small style="color:#888;">(สั่งงานด้วยเสียง เช่น "เดินหน้า", "ทิศไหน", "สถานะ")</small>
    </div>
    <div style="display:flex; gap:5px;">
        <input type="text" id="aiInput" placeholder="พิมพ์คำสั่ง..." style="flex:1; border-radius:5px; border:1px solid #ccc; padding:5px;">
        <button class="btn-send" onclick="sendAIText()">ส่ง</button>
        <button class="ai-btn-mic" onclick="startAIVoice()">🎤 พูด</button>
    </div>
    <button id="btnAiAuto" class="ai-btn-auto" onclick="toggleAiAuto()">🤖 เริ่มโหมดสำรวจอัตโนมัติ (AI AUTO)</button>
  </div>

  <hr>
  <div style="margin-top: 15px;">
     <p style="font-size: 14px; margin-bottom: 5px; color:#555;">📍 จิ้มจุดบนแผนที่ (ระบบจะวาดเส้นทางตามถนนและจุดไข่ปลาให้อัตโนมัติ)</p>
     <button class="btn-send" style="background: #ff9800;" onclick="sendRoute()">🚀 SEND ROUTE</button>
     <button class="btn-send" style="background: #dc3545;" onclick="clearRoute()">🗑️ Clear Map</button>
  </div>
  
  <hr>
  <h4>📍 ปรับชดเชยตำแหน่ง GPS (1 คลิก ≈ 1 เมตร)</h4>
  <div style="display:flex; flex-direction:column; align-items:center; gap:5px; margin-bottom:15px;">
     <button class="btn-send" style="width:50px; background:#17a2b8;" onclick="adjGPS('up')">▲</button>
     <div style="display:flex; gap:5px;">
         <button class="btn-send" style="width:50px; background:#17a2b8;" onclick="adjGPS('left')">◀</button>
         <button class="btn-send" style="width:50px; background:#6c757d;" onclick="adjGPS('reset')">RST</button>
         <button class="btn-send" style="width:50px; background:#17a2b8;" onclick="adjGPS('right')">▶</button>
     </div>
     <button class="btn-send" style="width:50px; background:#17a2b8;" onclick="adjGPS('down')">▼</button>
  </div>

  <div id="map"></div>

  <hr>
  <h4>🦾 Head Tilt Control</h4>
  <div class="row" style="flex-direction: column; align-items: center; font-size: 14px;">
     <label>ก้ม / เงย (S1, S2): <input type="range" id="s1" min="0" max="180" value="90" oninput="setSrv(1, this.value)" style="width:100%;"></label>
  </div>
</div>

<script>
window.robotDist = 999.0; 
window.robotYaw = 0;
window.lastNavCmd = "";
window.lastNavReportTime = 0;

var map = L.map('map').setView([13.7563, 100.5018], 15);
// 💡 เปลี่ยนมาใช้แผนที่จาก Google Maps (ลื่นไหลและอัปเดตใหม่กว่า)
L.tileLayer('https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}&hl=th', { maxZoom: 20, attribution: 'Google Maps' }).addTo(map);

var northControl = L.control({position: 'topright'});
northControl.onAdd = function(map) {
    var div = L.DomUtil.create('div', 'compass-control');
    div.innerHTML = '<div style="background: white; padding: 5px 10px; border-radius: 5px; border: 2px solid rgba(0,0,0,0.2); text-align: center; box-shadow: 0 1px 5px rgba(0,0,0,0.4);"><div style="color: #dc3545; font-weight: bold; font-size: 16px; margin-bottom: -4px;">N</div><div style="color: #343a40; font-size: 20px;">▲</div></div>';
    return div;
};
northControl.addTo(map);

var botIcon = L.divIcon({ 
  html: '<svg viewBox="0 0 24 24" fill="#007bff" stroke="white" stroke-width="2" width="32" height="32" id="botImg" style="transform: rotate(0deg); transition: transform 0.3s ease-out; filter: drop-shadow(0px 2px 4px rgba(0,0,0,0.5));"><path d="M12 2 L22 20 L12 17 L2 20 Z" /></svg>', 
  className: '', 
  iconSize: [32, 32], 
  iconAnchor: [16, 16] 
});
var botMarker = L.marker([13.7563, 100.5018], {icon: botIcon}).addTo(map);

var firstLoc = true; 
var botLatLng = null; 

var activeWaypoints = []; 
var activeMarkers = []; 

var routingControl = L.Routing.control({ 
  waypoints: [], 
  routeWhileDragging: true, 
  addWaypoints: true, 
  show: false, 
  createMarker: function() { return null; } 
}).addTo(map);

routingControl.on('routesfound', function(e) { 
  var routes = e.routes; 
  var fullRouteCoords = routes[0].coordinates; 
  
  activeMarkers.forEach(m => map.removeLayer(m));
  activeMarkers = [];
  activeWaypoints = [];

  var maxPts = 45; 
  var step = Math.ceil(fullRouteCoords.length / maxPts); 
  
  for(var i = 0; i < fullRouteCoords.length; i += step) {
    activeWaypoints.push(fullRouteCoords[i]);
    
    // 💡 ปรับรัศมีจุดบนแผนที่ให้ใหญ่ขึ้น (จาก 4 เป็น 8)
    var mk = L.circleMarker(fullRouteCoords[i], {
        radius: 8, color: '#ff9800', fillColor: '#ff9800', fillOpacity: 0.8
    }).addTo(map);
    activeMarkers.push(mk);
  } 
});

map.on('click', function(e) { 
  if(botLatLng) { 
    routingControl.setWaypoints([ L.latLng(botLatLng.lat, botLatLng.lng), e.latlng ]); 
  } else { 
    alert("Waiting for GPS..."); 
  } 
});

function clearRoute() { 
  routingControl.setWaypoints([]); 
  activeWaypoints = [];
  activeMarkers.forEach(m => map.removeLayer(m));
  activeMarkers = [];
  fetch('/route', { method: 'POST', body: "" }).catch(e=>{}); 
}

function sendRoute() { 
  if(activeWaypoints.length === 0) {
    alert("กรุณาจิ้มเป้าหมายบนแผนที่เพื่อสร้างเส้นทางก่อนครับ!");
    return;
  } 
  
  var routeStr = ""; 
  for(var i = 0; i < activeWaypoints.length; i++) {
    routeStr += activeWaypoints[i].lat.toFixed(6) + "," + activeWaypoints[i].lng.toFixed(6) + ";";
  } 
  
  fetch('/route', { method: 'POST', body: routeStr }).then(r=>r.text()).then(t=>{
      alert("✅ เริ่มภารกิจ! ส่งพิกัดเป้าหมาย " + activeWaypoints.length + " จุดตามเส้นทางให้หุ่นยนต์เรียบร้อย");
  }).catch(e=>{}); 
}

function sendCmdStr(str) { fetch('/tune?c=' + encodeURIComponent(str)); }
function setSrv(id, val) { fetch('/servo?id=' + id + '&val=' + val); }
function adjGPS(dir) { fetch('/offset?dir=' + dir); } // 💡 ฟังก์ชันส่งคำสั่งเลื่อนแผนที่

const stick = document.getElementById('stick'); 
const container = document.getElementById('joystick-container'); 
const maxDist = 60; 
let isDragging = false; 
let joyValX = 0; 
let joyValY = 0; 
let joyTimer = null;

function handleMove(event) { 
  if (!isDragging) return; 
  event.preventDefault(); 
  const touch = event.targetTouches ? event.targetTouches[0] : event; 
  const rect = container.getBoundingClientRect(); 
  let x = touch.clientX - rect.left - rect.width / 2; 
  let y = touch.clientY - rect.top - rect.height / 2; 
  
  if (Math.abs(x) > Math.abs(y)) { y = 0; } else { x = 0; } 
  
  const dist = Math.sqrt(x*x + y*y); 
  if (dist > maxDist) { 
    x = (x / dist) * maxDist; 
    y = (y / dist) * maxDist; 
  } 
  stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`; 
  joyValX = Math.round((x / maxDist) * 100); 
  joyValY = Math.round(-(y / maxDist) * 100); 
}

function startDrag(e) { 
  isDragging = true; 
  handleMove(e); 
  if(!joyTimer) joyTimer = setInterval(() => sendJoy(joyValX, joyValY), 150); 
}

function endDrag() { 
  isDragging = false; 
  stick.style.transform = `translate(-50%, -50%)`; 
  joyValX = 0; 
  joyValY = 0; 
  if(joyTimer){ clearInterval(joyTimer); joyTimer=null; } 
  sendJoy(0, 0); 
}

function sendJoy(x, y) { fetch('/cmd?v=' + x + ',' + y).catch(e=>{}); }

container.addEventListener('mousedown', startDrag); 
document.addEventListener('mousemove', handleMove); 
document.addEventListener('mouseup', endDrag); 
container.addEventListener('touchstart', startDrag, {passive: false}); 
container.addEventListener('touchmove', handleMove, {passive: false}); 
document.addEventListener('touchend', endDrag);

const synth = window.speechSynthesis;
let aiDriveTimer = null;
let isAiAutoDriving = false;
let aiAutoLoopTimer = null;
window.isReversing = false; 

function speakAI(text) {
    if (synth) {
        const ut = new SpeechSynthesisUtterance(text);
        ut.lang = 'th-TH';
        synth.speak(ut);
    }
}

function logAI(sender, msg) {
    const l = document.getElementById('aiLog');
    l.innerHTML += "<div style='margin-bottom: 5px;'><b>" + sender + ":</b> " + msg + "</div>";
    l.scrollTop = l.scrollHeight;
}

function sendAIText() {
    const input = document.getElementById('aiInput');
    if(input.value.trim() !== "") {
        processAICommand(input.value);
        input.value = "";
    }
}
document.getElementById('aiInput').addEventListener("keypress", function(event) {
  if (event.key === "Enter") {
    event.preventDefault();
    sendAIText();
  }
});

function startAIVoice() {
    const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    if(!SpeechRecognition) {
        alert("เบราว์เซอร์ของคุณไม่รองรับการสั่งงานด้วยเสียง");
        return;
    }
    const rec = new SpeechRecognition();
    rec.lang = 'th-TH';
    rec.start();
    logAI("System", "<i>กำลังฟังคำสั่งเสียงของคุณ...</i>");
    
    rec.onresult = (e) => {
        const text = e.results[0][0].transcript;
        processAICommand(text);
    };
}

function autoDrive(x, y, ms) {
    if(aiDriveTimer) clearInterval(aiDriveTimer);
    let elapsed = 0;
    sendCmdStr('S1'); 
    aiDriveTimer = setInterval(() => {
        sendJoy(x, y);
        elapsed += 150;
        if(elapsed >= ms) {
            clearInterval(aiDriveTimer);
            sendJoy(0, 0); 
        }
    }, 150);
}

function toggleAiAuto(forceStart = false) {
    if (forceStart) isAiAutoDriving = false; 
    
    isAiAutoDriving = !isAiAutoDriving;
    const btn = document.getElementById('btnAiAuto');
    
    if(isAiAutoDriving) {
        btn.style.background = "#dc3545"; 
        btn.innerText = "🛑 หยุดโหมดสำรวจอัตโนมัติ (STOP AI AUTO)";
        logAI("System", "เริ่มโหมดสำรวจอัตโนมัติ หุ่นยนต์จะเดินเล่นเอง");
        speakAI("เริ่มโหมดสำรวจอัตโนมัติ");
        sendCmdStr('S1'); 
        runAiAutoLoop();
    } else {
        btn.style.background = "#17a2b8"; 
        btn.innerText = "🤖 เริ่มโหมดสำรวจอัตโนมัติ (AI AUTO)";
        clearTimeout(aiAutoLoopTimer);
        if(aiDriveTimer) clearInterval(aiDriveTimer);
        sendJoy(0, 0);
        logAI("System", "หยุดโหมดสำรวจอัตโนมัติ");
        speakAI("หยุดระบบอัตโนมัติ");
    }
}

function runAiAutoLoop() {
    if(!isAiAutoDriving || window.isReversing) return; 

    let action = Math.floor(Math.random() * 4); 
    let duration = 0;

    if (window.robotDist < 30.0) {
        let avoidSpeed = Math.random() > 0.5 ? 35 : -35; 
        sendJoy(avoidSpeed, 0); 
        duration = 400 + Math.random() * 300; 
        logAI("AI Auto", "⚠️ ระวัง! ของขวางทาง กำลังเลี้ยวหลบ...");
    } 
    else {
        if (action === 0 || action === 1) { 
            sendJoy(0, 85); 
            duration = 1500 + Math.random() * 2000; 
            logAI("AI Auto", "กำลังเดินสำรวจไปข้างหน้า...");
        } else if (action === 2) { 
            sendJoy(-35, 0); 
            duration = 100; 
            logAI("AI Auto", "กำลังหันดูทางซ้าย...");
        } else if (action === 3) { 
            sendJoy(35, 0); 
            duration = 100; 
            logAI("AI Auto", "กำลังหันดูทางขวา...");
        }
    }

    aiAutoLoopTimer = setTimeout(() => {
        if(!isAiAutoDriving || window.isReversing) return;
        sendJoy(0, 0); 
        logAI("AI Auto", "หยุดประเมินพื้นที่...");
        
        aiAutoLoopTimer = setTimeout(runAiAutoLoop, 1500 + Math.random() * 1500); 
    }, duration);
}

function processAICommand(text) {
    logAI("คุณ", text);
    let t = text.toLowerCase();
    let reply = "ขออภัยครับ ไม่เข้าใจคำสั่ง";
    
    if(t.includes("ออโต้") || t.includes("สำรวจ") || t.includes("เดินเล่น") || t.includes("auto")) {
        toggleAiAuto(true);
        return; 
    }
    else if(t.includes("เดินหน้า") || t.includes("ไปข้างหน้า") || t.includes("หน้า")) {
        autoDrive(0, 70, 3000); 
        reply = "รับทราบ กำลังเดินหน้า 3 วินาทีครับ";
    } 
    else if(t.includes("ถอยหลัง") || t.includes("ถอย")) {
        autoDrive(0, -60, 3000); 
        reply = "กำลังถอยหลังครับ";
    } 
    else if(t.includes("หยุด") || t.includes("จอด") || t.includes("เบรค")) {
        if(aiDriveTimer) clearInterval(aiDriveTimer);
        sendCmdStr('S0'); sendJoy(0,0);
        reply = "หยุดการทำงานของหุ่นยนต์แล้วครับ";
    } 
    else if(t.includes("เลี้ยวซ้าย") || t.includes("หันซ้าย") || t.includes("ซ้าย")) {
        autoDrive(-35, 0, 800); 
        reply = "เลี้ยวซ้ายครับ";
    } 
    else if(t.includes("เลี้ยวขวา") || t.includes("หันขวา") || t.includes("ขวา")) {
        autoDrive(35, 0, 800); 
        reply = "เลี้ยวขวาครับ";
    } 
    else if(t.includes("ล้างแผนที่") || t.includes("ลบแผนที่") || t.includes("เคลียร์")) {
        clearRoute();
        reply = "ทำการเคลียร์เส้นทางบนแผนที่เรียบร้อยครับ";
    } 
    else if(t.includes("เริ่มทำงาน") || t.includes("สตาร์ท")) {
        sendCmdStr('S1');
        reply = "เปิดระบบทรงตัวแล้วครับ";
    }
    else if(t.includes("รายงาน") || t.includes("สถานะ")) {
        let txt = `ตอนนี้เข็มทิศหันไปทาง ${window.robotYaw} องศา `;
        if (window.robotDist < 100) txt += `และมีสิ่งกีดขวางในระยะ ${Math.round(window.robotDist)} เซนติเมตรครับ`;
        else txt += `ทางข้างหน้าโล่งปลอดภัยครับ`;
        reply = txt;
    }
    else if(t.includes("เข็มทิศ") || t.includes("ทิศอะไร") || t.includes("ทิศไหน")) {
        reply = `เข็มทิศกำลังชี้ไปที่ ${window.robotYaw} องศาครับ`;
    }
    else if(t.includes("สวัสดี") || t.includes("หวัดดี")) {
        reply = "สวัสดีครับ ผมคือหุ่นยนต์ AI ทรงตัว มีอะไรให้ผมช่วยไหมครับ";
    }
    
    logAI("AI", reply);
    speakAI(reply);
}

async function poll() {
  try {
    const r = await fetch('/data'); 
    const j = await r.json();
    
    document.getElementById('p').innerText = j.pitch.toFixed(2); 
    document.getElementById('v').innerText = j.vel.toFixed(2); 
    document.getElementById('u').innerText = j.u !== undefined ? j.u.toFixed(1) : "0";
    document.getElementById('ipAddr').innerText = j.ip; 
    
    window.robotYaw = j.yaw !== undefined ? Math.round(j.yaw) : 0;
    document.getElementById('yaw').innerText = window.robotYaw + "°";
    
    let distEl = document.getElementById('dist');
    if (j.dist !== undefined) {
        window.robotDist = j.dist;
        if (j.dist > 500) {
            distEl.innerText = "โล่ง";
            distEl.classList.remove('dist-warning');
        } else {
            distEl.innerText = j.dist.toFixed(0) + " cm";
            if (j.dist < 25.0) distEl.classList.add('dist-warning');
            else distEl.classList.remove('dist-warning');
        }
    }
    
    if (j.cmd !== window.lastNavCmd) {
        if (j.cmd.includes("Nav WP")) {
            if(window.lastNavCmd.includes("Idle") || window.lastNavCmd === "") {
               logAI("AI Copilot", "📍 เริ่มออกเดินทางตามเส้นทางจีพีเอส!");
            }
            window.lastNavReportTime = Date.now();
        } else if (j.cmd.includes("Nav Finish")) {
            logAI("AI Copilot", "🏁 เดินทางถึงเป้าหมายครบทุกจุดแล้วครับ!");
            speakAI("ถึงเป้าหมายสุดทางเรียบร้อยแล้วครับ");
        } else if (j.cmd.includes("Nav Wait GPS")) {
            logAI("AI Copilot", "⏳ สัญญาณ GPS อ่อน กำลังรอพิกัด...");
        } else if (j.cmd.includes("EMERGENCY REVERSE") && window.lastNavCmd && !window.lastNavCmd.includes("EMERGENCY")) {
            logAI("AI Copilot", "🚨 ระวัง! พบสิ่งกีดขวาง ระบบกำลังดึงรถถอยหลัง");
            speakAI("ระวังสิ่งกีดขวาง กำลังถอยหลัง");
        }
        window.lastNavCmd = j.cmd;
    }

    if (j.cmd && j.cmd.includes("Nav WP") && j.navDist > 0 && (Date.now() - (window.lastNavReportTime || 0) > 10000)) {
        let diffHeading = j.navCourse - j.yaw;
        while (diffHeading < -180) diffHeading += 360;
        while (diffHeading > 180) diffHeading -= 360;
        
        let turnText = "เดินตรงไป";
        if (diffHeading > 15) turnText = "กำลังแต่งพวงมาลัยไปทางขวา";
        else if (diffHeading < -15) turnText = "กำลังแต่งพวงมาลัยไปทางซ้าย";

        let report = `กำลังมุ่งหน้า เข็มทิศ ${Math.round(j.yaw)}° (เป้าหมาย ${Math.round(j.navCourse)}°) ${turnText}`;
        logAI("AI Copilot", "🧭 " + report);
        window.lastNavReportTime = Date.now();
    }

    if (window.robotDist < 15.0) {
        if (!window.isReversing) {
            window.isReversing = true;
            if(aiAutoLoopTimer) clearTimeout(aiAutoLoopTimer); 
            if(aiDriveTimer) clearInterval(aiDriveTimer);
            endDrag(); 
        }
    } else if (window.isReversing && window.robotDist > 20.0) {
        window.isReversing = false;
        if (isAiAutoDriving) aiAutoLoopTimer = setTimeout(runAiAutoLoop, 1000);
    }
    
    // 💡 แก้บั๊กภาพรถบนหน้าเว็บหมุนควงสว่าน
    let img = document.getElementById('botImg');
    if (img && j.yaw !== undefined) {
      let diff = j.yaw - (window.lastRawYaw || 0);
      while (diff < -180) diff += 360;
      while (diff > 180) diff -= 360;
      window.visualYaw = (window.visualYaw || 0) + diff;
      window.lastRawYaw = j.yaw;
      img.style.transform = 'rotate(' + window.visualYaw + 'deg)';
    }
    
    if (j.lat !== 0 && j.lng !== 0) { 
      document.getElementById('gpsData').innerText = j.lat.toFixed(6) + ", " + j.lng.toFixed(6); 
      botLatLng = { lat: j.lat, lng: j.lng }; 
      var newLatLng = new L.LatLng(j.lat, j.lng); 
      botMarker.setLatLng(newLatLng); 
      if(firstLoc) { map.setView(newLatLng, 17); firstLoc = false; } 
      
      if (j.cmd && j.cmd.includes("Nav WP")) {
          let wpMatch = j.cmd.match(/Nav WP(\d+)/);
          if (wpMatch) {
             let activeWp = parseInt(wpMatch[1]) - 1; 
             
             for (let i = 0; i < activeMarkers.length; i++) {
                 if (i < activeWp) {
                     if (map.hasLayer(activeMarkers[i])) map.removeLayer(activeMarkers[i]);
                 } else if (i === activeWp) {
                     activeMarkers[i].setStyle({color: '#28a745', fillColor: '#28a745', radius: 12}); // จุดปัจจุบันขยายใหญ่ขึ้นอีกนิด
                 }
             }
          }
      } else if (j.cmd && j.cmd.includes("Nav Finish")) {
          if (activeWaypoints.length > 0) clearRoute(); 
      }
    }
  } catch(e){}
  
  setTimeout(poll, 250); 
}
poll();
</script>
</body>
</html>
)HTML";

static bool parseLineToTelemetry(char* s, Telemetry &out) {
  char* tok[20] = {0}; 
  int n = 0; 
  tok[n++] = s;
  
  for (char* p = s; *p && n < 20; p++) { 
    if (*p == ',') { 
      *p = 0; 
      tok[n++] = p + 1; 
    } 
  }
  
  if (n < 3) return false;
  
  out.ms = strtoul(tok[0], NULL, 10); 
  out.pitch = strtof(tok[1], NULL); 
  out.gyro = strtof(tok[2], NULL);
  
  if (n >= 7) { 
    out.pos = strtof(tok[3], NULL); 
    out.vel = strtof(tok[4], NULL); 
    out.u = strtof(tok[5], NULL); 
    out.run = strtol(tok[6], NULL, 10); 
  }
  
  if (n >= 14) { 
    out.rxOk = strtol(tok[13], NULL, 10); 
  } 
  
  if (n >= 15) { 
    out.sp = strtof(tok[14], NULL); 
  }
  
  return true;
}

static bool connectSTAFromPrefs() {
  preferences.begin("wifi-conf", false); 
  String ssid = preferences.getString("ssid", ""); 
  String pass = preferences.getString("pass", ""); 
  preferences.end();
  
  if (ssid == "") { 
    ssid = DEFAULT_SSID; 
    pass = DEFAULT_PASS; 
  }
  
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid.c_str(), pass.c_str()); 
  uint32_t t0 = millis();
  
  while(millis() - t0 < 10000) { 
    if(WiFi.status() == WL_CONNECTED) { 
      ledOn(); 
      Serial.print("\n>>> WiFi Connected! IP Address: "); 
      Serial.println(WiFi.localIP()); 
      return true; 
    } 
    delay(100); 
    ledOn(); 
  } 
  return false;
}

void handleRoot() { 
  server.send(200, "text/html", PAGE); 
}

void handleData() {
  char jsonBuf[512]; 
  String ipStr = (WiFi.getMode() == WIFI_STA) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  
  snprintf(jsonBuf, sizeof(jsonBuf), 
           "{\"pitch\":%.2f,\"sp\":%.2f,\"vel\":%.2f,\"pos\":%.1f,\"u\":%.2f,\"yaw\":%.1f,\"lat\":%.6f,\"lng\":%.6f,\"sats\":%d,\"ip\":\"%s\",\"cmd\":\"%s\",\"dist\":%.1f,\"navDist\":%.1f,\"navCourse\":%.1f}",
           g.pitch, g.sp, g.vel, g.pos, g.u, currentHeading, savedLat, savedLng, gps.satellites.value(), ipStr.c_str(), lastCmdToSTM.c_str(), frontDist_cm, currentNavDist, currentNavCourse);
           
  server.send(200, "application/json", jsonBuf);
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
  } else {
    server.send(400, "text/plain", "BAD"); 
  }
}

void handleTune() { 
  if (server.hasArg("c")) { 
    String c = server.arg("c"); 
    STM.println(c); 
    server.send(200, "text/plain", "OK"); 
  } else {
    server.send(400, "text/plain", "BAD"); 
  }
}

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
    else isNavigating = false; 

    server.send(200, "text/plain", "Route OK"); 
  } else { 
    server.send(400, "text/plain", "No Data"); 
  } 
}

void handleServo() { 
  if (server.hasArg("id") && server.hasArg("val")) { 
    int id = server.arg("id").toInt(); 
    int val = server.arg("val").toInt(); 
    
    if (val < 0) val = 0; 
    if (val > 180) val = 180; 
    
    if (id == 1) { targetTiltAngle = val; } 
    
    server.send(200, "text/plain", "Servo OK"); 
  } else { 
    server.send(400, "text/plain", "Bad Request"); 
  } 
}

// 💡 ฟังก์ชันรับคำสั่งเลื่อนแผนที่จากหน้าเว็บ
void handleOffset() {
  if (server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (dir == "up") latOffset += OFFSET_STEP;
    else if (dir == "down") latOffset -= OFFSET_STEP;
    else if (dir == "right") lngOffset += OFFSET_STEP;
    else if (dir == "left") lngOffset -= OFFSET_STEP;
    else if (dir == "reset") { latOffset = 0.0; lngOffset = 0.0; }
    server.send(200, "text/plain", "Offset OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

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
      STM.println(btCmd); 
    } 
  } 
};

void setup() {
  Serial.begin(115200); 
  STM.begin(STM_BAUD, SERIAL_8N1, STM_RX, STM_TX);
  
  pinMode(LED_WIFI, OUTPUT); 
  digitalWrite(LED_WIFI, HIGH);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Wire.begin(21, 22); 
  Wire.setClock(400000); 
  
  oledInit(); 
  if(oledOK) updateEyesFromAngle(0.0f, 0); 
  
  initCompass(); 
  
  ESP32PWM::allocateTimer(0); 
  ESP32PWM::allocateTimer(1); 
  
  tiltLeftServo.setPeriodHertz(50); tiltLeftServo.attach(SERVO1_PIN, 500, 2400); 
  tiltRightServo.setPeriodHertz(50); tiltRightServo.attach(SERVO2_PIN, 500, 2400); 
  
  tiltLeftServo.write(90 + trimTilt); 
  tiltRightServo.write(180 - (90 + trimTilt)); 
  
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); 
  delay(5000);
  
  if (!radio.begin()) { 
    nrfOK = false; 
  } else { 
    if(radio.isChipConnected()) { 
      nrfOK = true; 
    } else { 
      nrfOK = false; 
    } 
    
    if (nrfOK) { 
      radio.setPALevel(RF24_PA_LOW); 
      radio.setDataRate(RF24_1MBPS); 
      radio.setChannel(100); 
      radio.setPayloadSize(32); 
      radio.setRetries(15, 15); 
      radio.openWritingPipe(address[1]); 
      radio.openReadingPipe(1, address[0]); 
      radio.startListening(); 
    } 
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
    WiFi.softAP("RobotControl"); 
  } 
  ledOn();
  
  server.on("/", handleRoot); 
  server.on("/data", handleData); 
  server.on("/cmd", handleCmd); 
  server.on("/tune", handleTune); 
  server.on("/route", HTTP_POST, handleRoute); 
  server.on("/servo", handleServo); 
  server.on("/offset", handleOffset); 
  server.begin();
  
  calculateAndSendCG();
}

void loop() {
  static unsigned long lastWifiCheck = 0; 
  if (WiFi.getMode() == WIFI_STA && millis() - lastWifiCheck > 10000) { 
    lastWifiCheck = millis(); 
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect(); 
  }
  
  readSonar();
  feedGPS(); 
  server.handleClient();
  
  if (millis() < blinkUntil) { 
    if ((millis() / 50) % 2) ledOff(); else ledOn(); 
  } else if (millis() - lastNrfRxTime < 3000) { 
    ledOn(); 
  } else { 
    if ((millis() / 500) % 2) ledOff(); else ledOn(); 
  }
  
  if (!deviceConnected && oldDeviceConnected) { 
    delay(500); 
    pServer->startAdvertising(); 
    oldDeviceConnected = deviceConnected; 
  } 
  
  if (deviceConnected && !oldDeviceConnected) { 
    oldDeviceConnected = deviceConnected; 
  }
  
  if (millis() - lastServoMoveMs > SERVO_SPEED_MS) {
    lastServoMoveMs = millis(); 
    bool armsMoved = false;
    
    if (currentTiltAngle != targetTiltAngle) { 
      if (abs(targetTiltAngle - currentTiltAngle) <= SERVO_STEP) {
        currentTiltAngle = targetTiltAngle; 
      } else if (currentTiltAngle < targetTiltAngle) {
        currentTiltAngle += SERVO_STEP; 
      } else if (currentTiltAngle > targetTiltAngle) {
        currentTiltAngle -= SERVO_STEP; 
      }
      
      int tiltTargetWithTrim = (int)currentTiltAngle + trimTilt; 
      if(tiltTargetWithTrim < 0) tiltTargetWithTrim = 0; 
      if(tiltTargetWithTrim > 180) tiltTargetWithTrim = 180; 
      
      tiltLeftServo.write(tiltTargetWithTrim); 
      tiltRightServo.write(180 - tiltTargetWithTrim); 
      armsMoved = true; 
    }
    
    if (armsMoved) { 
      calculateAndSendCG(); 
    }
  }
  
  static unsigned long lastControlLoop = 0; 
  static bool wasCommanding = false; 

  if (millis() - lastJoyTime > 800) { 
    isJoystickActive = false; 
    currentJoyX = 0; 
    currentJoyY = 0; 
  } else { 
    isJoystickActive = true; 
  }
  
  if (g.run == 1 && lastRunState == 0) { isAligningNorth = true; } 
  if (g.run == 0) { isAligningNorth = false; } 
  lastRunState = g.run;
  
  if (millis() - lastControlLoop > 50) {
      lastControlLoop = millis(); 
      char cmdBuf[64] = ""; 
      bool shouldSend = false;
      
      if (!isNavigating) { 
          if (abs(currentJoyX) > abs(currentJoyY)) currentJoyY = 0; 
          else if (abs(currentJoyY) > abs(currentJoyX)) currentJoyX = 0;
          
          if (isJoystickActive && (abs(currentJoyX) > 30 || abs(currentJoyY) > 30)) { 
            isAligningNorth = false; 
            int forwardCmd = 0; 
            int turnCmd = 0;  
            
            if (currentJoyY > 30) forwardCmd = map(currentJoyY, 30, 100, 3, 7); 
            else if (currentJoyY < -30) forwardCmd = map(currentJoyY, -100, -30, -7, -3); 
            
            // 💡 แก้ไข: เพิ่มแรงหมุนขั้นต่ำ (จาก 4 เป็น 12) เพื่อให้มีแรงสลัดความฝืดของล้อ
            if (currentJoyX > 30) turnCmd = map(currentJoyX, 30, 100, -12, -25); 
            else if (currentJoyX < -30) turnCmd = map(currentJoyX, -100, -30, 25, 12); 
            
            sprintf(cmdBuf, "*%d,%d,0#", forwardCmd, turnCmd); 
            lastCmdToSTM = String(cmdBuf) + " (Manual)";
            shouldSend = true;
            wasCommanding = true;
          } else if (isAligningNorth) { 
            float diff = 0.0f - currentHeading; 
            while (diff < -180.0f) diff += 360.0f; 
            while (diff > 180.0f) diff -= 360.0f; 
            
            if (abs(diff) > 15.0f) { 
              // 💡 ลดแรงหมุนตอนหันหาทิศเหนือลงจาก 12-25 เหลือ 8-18 ให้มันหันนิ่มๆ
              int autoTurn = map(abs(diff), 15, 180, 8, 18);  
              
              if (diff > 0) autoTurn = -autoTurn; 
              
              // 💡 ขยายมุมลดความเร็ว (Fine Turn) จาก 25 เป็น 35 องศา ให้มันเบรกล่วงหน้านานขึ้น
              int isFineTurn = (abs(diff) <= 35.0f) ? 1 : 0; 
              sprintf(cmdBuf, "*0,%d,%d#", autoTurn, isFineTurn); 
              lastCmdToSTM = String(cmdBuf) + " (Aligning N)";
              shouldSend = true;
              wasCommanding = true;
            } else { 
              isAligningNorth = false; 
              sprintf(cmdBuf, "*0,0,0#"); 
              lastCmdToSTM = "Idle";
              shouldSend = true;
              wasCommanding = false;
            }
          } else { 
            if (wasCommanding) {
                sprintf(cmdBuf, "*0,0,0#"); 
                lastCmdToSTM = "Idle";
                shouldSend = true;
                wasCommanding = false;
            } else {
                static unsigned long lastHeartbeat = 0;
                if (millis() - lastHeartbeat > 500) {
                    sprintf(cmdBuf, "*0,0,0#"); 
                    shouldSend = true;
                    lastHeartbeat = millis();
                }
            }
          }
      } else {
          if (gps.location.isValid() && currentWpIndex < wpCount) {
              double curLat = gps.location.lat();
              double curLng = gps.location.lng();
              double targetLat = wpLat[currentWpIndex];
              double targetLng = wpLng[currentWpIndex];

              double distToWp = TinyGPSPlus::distanceBetween(curLat, curLng, targetLat, targetLng);
              double targetCourse = TinyGPSPlus::courseTo(curLat, curLng, targetLat, targetLng);

              currentNavDist = (float)distToWp;
              currentNavCourse = (float)targetCourse;

              // 💡 ขยายรัศมีเป้าหมายจาก 2.0 เมตร เป็น 3.5 เมตร
              if (distToWp < 3.5) {
                  currentWpIndex++;
                  if (currentWpIndex >= wpCount) {
                      isNavigating = false; 
                      currentNavDist = 0.0f;
                      currentNavCourse = 0.0f;
                      sprintf(cmdBuf, "*0,0,0#");
                      lastCmdToSTM = "Nav Finish";
                      shouldSend = true;
                      wasCommanding = false;
                  }
              } else {
                  float headingError = targetCourse - currentHeading;
                  while (headingError < -180.0f) headingError += 360.0f;
                  while (headingError > 180.0f) headingError -= 360.0f;

                  int forwardCmd = 0;
                  int turnCmd = 0;
                  int isFineTurn = 0;

                  if (abs(headingError) > 15.0f) { 
                      // 💡 ตรงนี้ถ้ารถเลี้ยวตามจุดไข่ปลาแล้วแกว่งแรงไป ก็อาจจะลดลงคล้ายๆ Aligning North ก็ได้ครับ
                      turnCmd = map(abs(headingError), 15, 180, 12, 25);
                      
                      if (headingError > 0) turnCmd = -turnCmd; 
                      
                      if (abs(headingError) < 30.0f) isFineTurn = 1;

                      if (abs(headingError) < 25.0f) {
                          forwardCmd = 4; 
                      } else {
                          forwardCmd = 0; 
                      }
                  } else {
                      forwardCmd = 7; 
                  }

                  sprintf(cmdBuf, "*%d,%d,%d#", forwardCmd, turnCmd, isFineTurn);
                  lastCmdToSTM = String(cmdBuf) + " (Nav WP" + String(currentWpIndex+1) + ")";
                  shouldSend = true;
                  wasCommanding = true;
              }
          } else {
              currentNavDist = 0.0f;
              currentNavCourse = 0.0f;
              sprintf(cmdBuf, "*0,0,0#");
              lastCmdToSTM = "Nav Wait GPS";
              static unsigned long lastWait = 0;
              if (millis() - lastWait > 500) {
                  shouldSend = true;
                  lastWait = millis();
              }
          }
      }
      
      static bool wasEmergencyReversing = false;
      
      if (frontDist_cm < 15.0f) {
          sprintf(cmdBuf, "*-6,0,0#");
          lastCmdToSTM = "EMERGENCY REVERSE";
          shouldSend = true; 
          wasEmergencyReversing = true;
      } 
      else {
          if (wasEmergencyReversing) {
              sprintf(cmdBuf, "*0,0,0#");
              lastCmdToSTM = "REVERSE STOPPED";
              shouldSend = true;
              wasEmergencyReversing = false;
          }
          
          if (frontDist_cm < 25.0f && strlen(cmdBuf) > 0) {
              if (cmdBuf[1] != '-' && cmdBuf[1] != '0') { 
                  char* comma = strchr(cmdBuf, ',');
                  if (comma != NULL) {
                      char newCmd[64];
                      sprintf(newCmd, "*0%s", comma); 
                      strcpy(cmdBuf, newCmd);
                      lastCmdToSTM = "BLOCKED FORWARD";
                      shouldSend = true;
                  }
              }
          }
      }
      
      if (shouldSend) {
          STM.println(cmdBuf); 
      }
      
      currentHeading = getCompassHeading(); 
  }
  
  if (nrfOK) {
      static unsigned long lastHBT = 0; 
      if (millis() - lastHBT > 1000) { 
        lastHBT = millis(); 
        char hbMsg[32] = {0}; 
        bool stmLinkOk = (millis() - lastStmRxTime < 1000) && (g.rxOk == 1); 
        sprintf(hbMsg, "HB:%s,%s", gps.location.isValid() ? "1" : "0", stmLinkOk ? "1" : "0"); 
        
        radio.stopListening(); 
        bool ok = radio.write(hbMsg, 32); 
        radio.startListening(); 
      }
      
      if (radio.available()) { 
        lastNrfRxTime = millis(); 
        feedGPS(); 
        char packetBuffer[32] = {0}; 
        radio.read(&packetBuffer, sizeof(packetBuffer)); 
        String packet = String(packetBuffer); 
        packet.trim();
        
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
        } else if (packet.indexOf("IP?") >= 0) { 
          blinkUntil = millis() + 1000; 
          String ipStr = (WiFi.getMode() == WIFI_STA) ? WiFi.localIP().toString() : WiFi.softAPIP().toString(); 
          String ipData = "IP:" + ipStr; 
          char sendBuf[32] = {0}; 
          strncpy(sendBuf, ipData.c_str(), 31); 
          
          radio.stopListening(); 
          for(int i=0; i<3; i++) { 
            radio.write(sendBuf, 32); 
            delay(30); 
          } 
          radio.startListening(); 
        } else if (packet.indexOf("GET") >= 0) { 
          blinkUntil = millis() + 1000; 
          String gpsData; 
          if (gps.location.isValid()) {
            gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6); 
          } else if (savedLat != 0.0) {
            gpsData = String(savedLat, 6) + "," + String(savedLng, 6); 
          } else {
            gpsData = "NO_GPS"; 
          }
          
          char sendBuf[32] = {0}; 
          strncpy(sendBuf, gpsData.c_str(), 31); 
          radio.stopListening(); 
          for(int i=0; i<3; i++) { 
            radio.write(sendBuf, 32); 
            delay(30); 
          } 
          radio.startListening(); 
        }
      }
  } 
  
  while (STM.available()) { 
    char c = (char)STM.read(); 
    if (c == '\n') { 
      line[idx] = 0; 
      Telemetry t; 
      if (parseLineToTelemetry(line, t)) { 
        g = t; 
        g.pitch = -g.pitch; 
        lastStmRxTime = millis(); 
      } 
      idx = 0; 
    } else if (c != '\r') { 
      if (idx < 250) line[idx++] = c; 
    } 
  }
  
  while (Serial.available()) { 
    char c = (char)Serial.read(); 
    STM.write(c); 
  }
  
  if (oledOK && (millis() - lastEyeMs > 100)) { 
    lastEyeMs = millis(); 
    updateEyesFromAngle(g.pitch, g.run); 
  }
}