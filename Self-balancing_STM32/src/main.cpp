#include <Arduino.h>
#include <Wire.h>

// ================================================================
// 0) COMMUNICATION SETUP
// ================================================================
// ใช้ Serial3 (PB10=TX3, PB11=RX3) เชื่อมต่อกับ ESP32 สำหรับ STM32 Maple Core
#define SerialESP Serial3 

const int statePin = PB1;

// ================================================================
// 1) PINS DEFINITIONS
// ================================================================
// Motor Left
const int pwmL  = PA8;   
const int dirL  = PA0;   
const int enL   = PB12;  

// Motor Right
const int pwmR  = PB0;   
const int dirR  = PA4;   
const int enR   = PB13;  

const int ledPin = PC13; 

// Encoders
const int encLA = PA2;
const int encLB = PA3;
const int encRA = PA6;
const int encRB = PA7;

// ================================================================
// 2) LOOP TIMING & CONSTANTS
// ================================================================
const float PI_F          = 3.1415926f;
const float LOOP_HZ       = 500.0f;
const uint32_t LOOP_DT_US = (uint32_t)(1e6f / LOOP_HZ);
const float DT            = 1.0f / LOOP_HZ;

// ================================================================
// 3) IMU (MPU6050)
// ================================================================
const uint8_t MPU_ADDR = 0x68;

float pitchDeg    = 0.0f;
float gyroX_dps   = 0.0f;
float gyroBiasX   = 0.0f;
float gyroX_filt  = 0.0f;
const float GYRO_ALPHA = 0.4f; 
const float IMU_SIGN = -1.0f; 
const float CF_ALPHA = 0.95f; 

// ================================================================
// 4) ENCODERS / WHEEL PHYSICS
// ================================================================
volatile long encL_cnt = 0;
volatile long encR_cnt = 0;

long lastEncL_cnt = 0;
long lastEncR_cnt = 0;

float wheelPos_m     = 0.0f;
float wheelVel_mps   = 0.0f;

// Nidec 24H Direct Drive (100 Line * 2 edges = 200 ticks/rev)
const float COUNTS_PER_REV    = 200.0f; 
const float WHEEL_DIAMETER_M  = 0.065f; // ล้อขนาด 6.5 cm
const float WHEEL_CIRC_M      = PI_F * WHEEL_DIAMETER_M;
const float M_PER_COUNT       = WHEEL_CIRC_M / COUNTS_PER_REV; 

// ================================================================
// 5) FULL-STATE FEEDBACK GAINS (PID)
// ================================================================
float Kth   = 30.0f;   // (Angle P) 
float Kd    = 1.9f;    // (Angle D) 
float Kx    = 30.0f;   // (Position P) 
float Kv    = 9.5f;    // (Velocity D) 
float KiPos = 1.2f;    // (Position I) 

float posInt = 0.0f;
const float POS_INT_LIMIT = 100.0f; 

// Angle Offset: ค่านี้ดีมากตอนอยู่นิ่งๆและพักได้เบาๆ
float angleOffsetDeg = -1.70f; 

// ================================================================
// 6) SAFETY / AUTO ARM FLAGS
// ================================================================
bool runEnabled = false;

const float FALL_ANGLE_DEG = 45.0f; 
const float ARM_ANGLE_DEG  = 6.0f;
const float ARM_GYRO_DPS   = 15.0f;
const uint32_t ARM_MS      = 400;

uint32_t armStableStart = 0;
uint32_t fallDuration = 0; 

// ================================================================
// 7) CONTROL HELPERS & MOTOR VARS
// ================================================================
float lastU = 0.0f;
const float SLEW_MAX = 40000.0f;
const float MAX_U_HW = 255.0f; 

const float MIN_PWM = 15.0f;     
const float PWM_DEADZONE = 3.0f; 

const float BRAKE_ANGLE_DEAD = 2.0f;
const float BRAKE_VEL_LIMIT  = 0.02f;
float Bbrake = 0.1f;

const float SNAP_ANGLE_DEG = 5.0f;
const float SNAP_KV_EXTRA  = 3.5f; 

const bool INVERT_L = false;
const bool INVERT_R = true;

// Joystick Control Variables
float speedCmd = 0.0f;          // แทน rawTargetAngle
float turnSpeed = 0.0f;         

// Serial Parsing Buffers
char rxBuf[64];
int rxIdx = 0;

// ================================================================
// 8) OLED HELPERS (SSD1306 Direct I2C)
// ================================================================
#define OLED_ADDR 0x3C
uint8_t oledBuf[1024];
bool oledOK = false;
uint32_t lastEyeMs = 0;

void oledCmd(uint8_t c) { 
  Wire.beginTransmission(OLED_ADDR); Wire.write(0x00); Wire.write(c); Wire.endTransmission(); 
}

void oledInit() {
  delay(80);
  oledCmd(0xAE); oledCmd(0x20); oledCmd(0x00); oledCmd(0xB0); oledCmd(0xC8); 
  oledCmd(0x00); oledCmd(0x10); oledCmd(0x40); oledCmd(0x81); oledCmd(0x7F); 
  oledCmd(0xA1); oledCmd(0xA6); oledCmd(0xA8); oledCmd(0x3F); oledCmd(0xA4); 
  oledCmd(0xD3); oledCmd(0x00); oledCmd(0xD5); oledCmd(0x80); oledCmd(0xD9); 
  oledCmd(0xF1); oledCmd(0xDA); oledCmd(0x12); oledCmd(0xDB); oledCmd(0x40); 
  oledCmd(0x8D); oledCmd(0x14); oledCmd(0xAF);
  memset(oledBuf, 0, sizeof(oledBuf)); oledOK = true;
}

void oledUpdate() {
  if (!oledOK) return;
  for (uint8_t page = 0; page < 8; page++) {
    oledCmd(0xB0 | page); oledCmd(0x00); oledCmd(0x10);
    for (uint8_t col = 0; col < 128; col += 16) {
      Wire.beginTransmission(OLED_ADDR); Wire.write(0x40);
      for (uint8_t i = 0; i < 16; i++) Wire.write(oledBuf[page * 128 + col + i]);
      Wire.endTransmission();
    }
  }
}

void oledSetPixel(int x, int y, bool on) {
  if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
  if(on) oledBuf[(y >> 3) * 128 + x] |= (1 << (y & 7));
  else   oledBuf[(y >> 3) * 128 + x] &= ~(1 << (y & 7));
}

void drawEyeSimple(int cx, int cy, int r, int offY) {
  int rr=r*r;
  for(int dy=-r;dy<=r;dy++) {
    for(int dx=-r;dx<=r;dx++) {
      if(dx*dx+dy*dy<=rr) oledSetPixel(cx+dx, cy+dy, true);
    }
  }
  int pr=r-3; int prr=pr*pr;
  for(int dy=-pr;dy<=pr;dy++) {
    for(int dx=-pr;dx<=pr;dx++) {
      if(dx*dx+dy*dy<=prr) oledSetPixel(cx+dx, cy+dy+offY, false);
    }
  }
}

void drawEyeCross(int cx, int cy, int s) {
  for(int i=-s;i<=s;i++) { 
    oledSetPixel(cx+i,cy-i,1); oledSetPixel(cx+i,cy-i+1,1); 
    oledSetPixel(cx+i,cy+i,1); oledSetPixel(cx+i,cy+i+1,1); 
  }
}

void updateEyesFromAngle(float ang) {
  if(!oledOK) return;
  memset(oledBuf,0,sizeof(oledBuf));
  int offY = (int)(ang*0.3f); 
  if(offY>4) offY=4; if(offY<-4) offY=-4;
  
  if(abs(ang)>FALL_ANGLE_DEG) { 
    drawEyeCross(40,32,10); drawEyeCross(88,32,10); 
  } else { 
    drawEyeSimple(40,32,10,offY); drawEyeSimple(88,32,10,offY); 
  }
  oledUpdate();
}

// ================================================================
// 9) MPU6050 HELPERS
// ================================================================
inline void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

inline bool mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((int)MPU_ADDR, (int)len);
  if (got != (int)len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool mpuInit() {
  mpuWrite(0x6B, 0x00); delay(50);
  mpuWrite(0x1B, 0x00); mpuWrite(0x1C, 0x00); delay(10);
  Wire.beginTransmission(MPU_ADDR); return (Wire.endTransmission() == 0);
}

void mpuCalibrateGyro() {
  const int N = 1000; long sum = 0; int good = 0;
  for(int j=0; j<5; j++) { digitalWrite(ledPin, !digitalRead(ledPin)); delay(50); }
  for (int i = 0; i < N; i++) {
    uint8_t buf[2];
    if (mpuReadBytes(0x45, buf, 2)) {
      int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
      sum += gx; good++;
    }
    delay(2);
  }
  if (good < 10) { gyroBiasX = 0.0f; return; }
  gyroBiasX = ((float)sum / (float)good) / 131.0f;
  digitalWrite(ledPin, LOW); 
}

float getMedian(float a, float b, float c) {
  float middle;
  if ((a <= b) && (a <= c)) middle = (b <= c) ? b : c;
  else if ((b <= a) && (b <= c)) middle = (a <= c) ? a : c;
  else middle = (a <= b) ? a : b;
  return middle;
}

void mpuReadAngle(float dt) {
  uint8_t buf[6];
  if (!mpuReadBytes(0x3B, buf, 6)) return;
  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  if (!mpuReadBytes(0x45, buf, 2)) return;
  int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);

  float ax_g = (float)ax / 16384.0f;
  float az_g = (float)az / 16384.0f;
  float rawAccAngle = atan2f(-ax_g, az_g) * (180.0f / PI_F) * IMU_SIGN;
  float gyro_raw_dps = (((float)gx / 131.0f) - gyroBiasX) * IMU_SIGN;

  static float angHist[3] = {0,0,0};
  angHist[0] = angHist[1]; angHist[1] = angHist[2]; angHist[2] = rawAccAngle;
  float cleanAccAngle = getMedian(angHist[0], angHist[1], angHist[2]);

  gyroX_filt = (1.0f - GYRO_ALPHA) * gyroX_filt + GYRO_ALPHA * gyro_raw_dps;
  gyroX_dps = gyroX_filt; 
  if (dt < 0.0005f) dt = DT; if (dt > 0.02f) dt = DT;
  float pred = pitchDeg + gyroX_dps * dt;
  pitchDeg = CF_ALPHA * pred + (1.0f - CF_ALPHA) * cleanAccAngle;
}

// ================================================================
// 10) ENCODER INTERRUPT SERVICE ROUTINES
// ================================================================
void isrEncLA() { if (digitalRead(encLA) == digitalRead(encLB)) encL_cnt++; else encL_cnt--; }
void isrEncRA() { if (digitalRead(encRA) == digitalRead(encRB)) encR_cnt++; else encR_cnt--; }

// ================================================================
// 11) MOTOR DRIVER FUNCTIONS
// ================================================================
void setMotorRaw(int pwmPin, int dirPin, int enPin, int val, bool invertDir) {
  if (val == 0) {
    digitalWrite(enPin, LOW); analogWrite(pwmPin, 255); return;
  }
  digitalWrite(enPin, HIGH);
  bool dir = (val > 0);
  if (invertDir) dir = !dir;
  digitalWrite(dirPin, dir ? HIGH : LOW);
  int mag = abs(val);
  if (mag > 255) mag = 255;
  analogWrite(pwmPin, 255 - mag);
}

void brakeAll() {
  digitalWrite(enL, LOW); digitalWrite(enR, LOW);
  analogWrite(pwmL, 255); analogWrite(pwmR, 255);
}

void drive(float u, float turn) {
  float left = u + turn;
  float right = u - turn;

  if (abs(left) < PWM_DEADZONE) left = 0;
  else left += (left > 0) ? MIN_PWM : -MIN_PWM;

  if (abs(right) < PWM_DEADZONE) right = 0;
  else right += (right > 0) ? MIN_PWM : -MIN_PWM;

  float du = u - lastU;
  float maxStep = SLEW_MAX * DT;
  if (du > maxStep) du = maxStep; if (du < -maxStep) du = -maxStep;
  
  if (left > MAX_U_HW) left = MAX_U_HW; else if (left < -MAX_U_HW) left = -MAX_U_HW;
  if (right > MAX_U_HW) right = MAX_U_HW; else if (right < -MAX_U_HW) right = -MAX_U_HW;

  lastU += du; 
  if (lastU > MAX_U_HW) lastU = MAX_U_HW; if (lastU < -MAX_U_HW) lastU = -MAX_U_HW;

  float angDead = BRAKE_ANGLE_DEAD * Bbrake;
  float velDead = BRAKE_VEL_LIMIT  * Bbrake;
  
  if (abs(pitchDeg) < angDead && abs(wheelVel_mps) < velDead && abs(u) < 5.0f && abs(turn) < 2.0f) {
    brakeAll();
    lastU = 0.0f;
    return;
  }

  setMotorRaw(pwmL, dirL, enL, (int)left, INVERT_L);
  setMotorRaw(pwmR, dirR, enR, (int)right, INVERT_R);
}

// ================================================================
// 12) COMMAND & TELEMETRY
// ================================================================
float parseFloatFrom(const char* s) { return (float)atof(s); }

void processLine(char* line) {
  if (line[0] == '*') {
    String s = String(line);
    int comma = s.indexOf(',');
    int hash = s.indexOf('#');
    if (comma > 0 && hash > comma) {
      
      // ESP32 ส่งมาเป็น *แกนX(เดินหน้า),แกนY(เลี้ยว)#
      float rawX = s.substring(1, comma).toFloat();
      float rawY = s.substring(comma + 1, hash).toFloat();
      
      // ==========================================
      // 🚀 1. ควบคุมด้วย "ความเร็ว" 
      // เนื่องจากค่า X ที่มาจาก ESP32 ถูกคูณมาแรงแล้ว (เช่น 1200) 
      // เราจึงเอามาคูณให้เป็นความเร็วจริง (m/s) อีกที
      // ==========================================
      speedCmd = rawX * 0.001f; // ถ้า X=1200 ความเร็วเป้าหมายคือ 1.2 m/s
      
      // เลี้ยว: ถ้า Y=600 สั่งล้อสวนกันแรงๆ
      turnSpeed = rawY * 0.2f; 
    }
    return;
  }

  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32; 
  float val = parseFloatFrom(line + 1);

  switch(cmd) {
    case 'P': Kth = val; break;
    case 'D': Kd = val; break;
    case 'X': Kx = val; break;
    case 'V': Kv = val; break;
    case 'I': KiPos = val; break;
    case 'A': angleOffsetDeg = val; break;
    case 'C': 
      angleOffsetDeg = pitchDeg; 
      posInt = 0; wheelPos_m = 0; 
      break;
    case 'S': 
      if (val > 0) { runEnabled = true; } else { runEnabled = false; brakeAll(); }
      break;
  }
}

void readESP32() {
  while (SerialESP.available()) {
    char c = (char)SerialESP.read();
    if (c == '\r') continue;
    if (c == '\n' || c == '#') {
      if (c == '#') rxBuf[rxIdx++] = c;
      rxBuf[rxIdx] = 0;
      if (rxIdx > 0) processLine(rxBuf);
      rxIdx = 0;
    } else {
      if (rxIdx < 63) rxBuf[rxIdx++] = c;
    }
  }
}

void sendTelemetry() {
  static uint32_t lastTel = 0;
  if (millis() - lastTel > 50) { 
     lastTel = millis();
     SerialESP.print(millis()); SerialESP.print(",");
     SerialESP.print(pitchDeg, 2); SerialESP.print(",");
     SerialESP.print(gyroX_dps, 2); SerialESP.print(",");
     SerialESP.print(wheelPos_m, 4); SerialESP.print(",");
     SerialESP.print(wheelVel_mps, 4); SerialESP.print(",");
     SerialESP.print(lastU, 1); SerialESP.print(",");
     SerialESP.print(runEnabled ? 1 : 0); SerialESP.print(",");
     SerialESP.print(Kth, 1); SerialESP.print(",");
     SerialESP.print(Kd, 1); SerialESP.print(",");
     SerialESP.print(Kx, 1); SerialESP.print(",");
     SerialESP.print(Kv, 1); SerialESP.print(",");
     SerialESP.print(KiPos, 4); SerialESP.print(",");
     SerialESP.println(angleOffsetDeg, 2);
  }
}

// ================================================================
// 13) SETUP & MAIN LOOP
// ================================================================
void setup() {
  pinMode(pwmL, OUTPUT); pinMode(dirL, OUTPUT); pinMode(enL, OUTPUT);
  pinMode(pwmR, OUTPUT); pinMode(dirR, OUTPUT); pinMode(enR, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  pinMode(encLA, INPUT_PULLUP); pinMode(encLB, INPUT_PULLUP);
  pinMode(encRA, INPUT_PULLUP); pinMode(encRB, INPUT_PULLUP);
  pinMode(statePin, INPUT);

  analogWriteFrequency(20000); 

  attachInterrupt(digitalPinToInterrupt(encLA), isrEncLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRA), isrEncRA, CHANGE);

  brakeAll();
  digitalWrite(ledPin, HIGH); 

  Serial.begin(115200); 
  SerialESP.begin(115200);
  Wire.begin(); Wire.setClock(400000); 

  oledInit();
  if (oledOK) updateEyesFromAngle(0.0f);

  if (!mpuInit()) {
    while (1) { digitalWrite(ledPin, !digitalRead(ledPin)); delay(100); }
  }
  mpuCalibrateGyro();

  lastEncL_cnt = encL_cnt; lastEncR_cnt = encR_cnt;
  digitalWrite(ledPin, LOW); 
  lastEyeMs = millis();
}

void loop() {
  static uint32_t lastMicros = micros();
  uint32_t now = micros();
  
  if (now - lastMicros < LOOP_DT_US) return;
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0.0f) dt = DT;
  lastMicros = now;
  uint32_t nowMs = millis();

  // 1. Read IMU
  mpuReadAngle(dt);
  
  // 2. Read Encoders
  long curL = encL_cnt, curR = encR_cnt;
  float dPos = (0.5f * (float)((curL-lastEncL_cnt) + (curR-lastEncR_cnt))) * M_PER_COUNT;
  wheelPos_m += dPos;
  wheelVel_mps = 0.7f * wheelVel_mps + 0.3f * (dPos / dt); 
  lastEncL_cnt = curL; lastEncR_cnt = curR;

  // 3. Command
  readESP32();

  // 4. Safety Logic
  if (abs(pitchDeg) > FALL_ANGLE_DEG) {
    if (fallDuration == 0) fallDuration = nowMs;
    if (nowMs - fallDuration > 100) { 
      runEnabled = false; armStableStart = 0; brakeAll();
      wheelPos_m = 0; posInt = 0; lastU = 0; 
    }
  } else {
    fallDuration = 0;
    if (!runEnabled && abs(pitchDeg) < ARM_ANGLE_DEG && abs(gyroX_dps) < ARM_GYRO_DPS) {
      if (armStableStart == 0) armStableStart = nowMs;
      else if (nowMs - armStableStart > ARM_MS) {
         runEnabled = true; wheelPos_m = 0; posInt = 0; lastU = 0;
         digitalWrite(ledPin, LOW); speedCmd = 0;
      }
    } else { armStableStart = 0; }
  }

  // 5. PID Control Logic
  float u = 0.0f;
  if (runEnabled) {
    static bool wasMoving = false;
    bool isCmdActive = (abs(speedCmd) > 0.001f || abs(turnSpeed) > 0.05f);

    // ==========================================
    // 🚀 3. ปลดเชือก และ ขับเคลื่อน (Anti-Rubber Band)
    // ==========================================
    if (isCmdActive) {
        // หลอกให้หุ่นวิ่งตามตำแหน่งเป้าหมายที่เคลื่อนที่ไปเรื่อยๆ (หุ่นจะเทตัววิ่งอัตโนมัติ)
        wheelPos_m -= speedCmd * dt; 
        posInt = 0.0f; // ปิด I-term ตอนกำลังวิ่งเพื่อความสมูท
        wasMoving = true;
    } else {
        if (wasMoving) {
            // จังหวะเพิ่งปล่อยจอยสติ๊ก: รีเซ็ตตำแหน่งให้หยุดนิ่งตรงนี้ทันที (ป้องกันการเด้งกลับ)
            wheelPos_m = 0.0f; 
            posInt = 0.0f;
            wasMoving = false;
        }
        // ตอนอยู่นิ่ง: เปิด I-term ช่วยพยุงไม่ให้ไหล
        if (KiPos > 0.0f) {
            posInt += wheelPos_m * dt;
            posInt *= 0.99f; 
            posInt = constrain(posInt, -POS_INT_LIMIT, POS_INT_LIMIT);
        }
    }
    
    // 🚀 เพิ่ม Feedforward: สั่งให้หุ่น "ชิงเอียงตัว" ล่วงหน้าตามความเร็วที่สั่ง
    float targetAngleOffset = speedCmd * 2.5f; 
    
    float errTh = pitchDeg - (angleOffsetDeg + targetAngleOffset);
    float errVel = wheelVel_mps - speedCmd; // ลบความเร็วเป้าหมายออก เพื่อไม่ให้ระบบ PID ต้านการวิ่ง
    
    float uRaw = Kth*(errTh) + Kd*gyroX_dps + Kx*wheelPos_m + Kv*errVel + KiPos*posInt;
    
    // Snap-up assist
    if (abs(errTh) < SNAP_ANGLE_DEG && !isCmdActive) {
       uRaw -= SNAP_KV_EXTRA * wheelVel_mps;
    }
    
    u = uRaw;
    
    // 🚀 ส่งคำสั่งความเร็วมอเตอร์ + คำสั่งหมุน
    drive(u, turnSpeed);
  } else {
    brakeAll();
    lastU = 0;
  }

  // 6. Telemetry & OLED
  sendTelemetry();
  if (oledOK && (nowMs - lastEyeMs > 100)) {
      lastEyeMs = nowMs;
      if (!runEnabled) {
        if (abs(pitchDeg) > FALL_ANGLE_DEG) drawEyeCross(64, 32, 10);
        else updateEyesFromAngle(pitchDeg);
      }
  }
}