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

const float GYRO_ALPHA = 0.2f; 
// 💡 แยกเครื่องหมาย Gyro กับ Accel ออกจากกัน 
// (ถ้าเปลี่ยนแกนแล้วเซ็นเซอร์ตีกัน ให้มาสลับเครื่องหมาย 1.0f กับ -1.0f ตรงนี้)
const float ACC_SIGN = -1.0f;  
const float GYRO_SIGN = 1.0f; // 💡 ปรับกลับเป็น 1.0f ไว้เป็นค่าเริ่มต้นก่อน
const float CF_ALPHA = 0.95f; 

// 💡 ลบ const float IMU_OFFSET_DEG = -90.0f; ทิ้งไปเลย เพราะสูตรใหม่ฉลาดพอที่จะเซ็ต 0 เอง

// ================================================================
// 4) ENCODERS / WHEEL PHYSICS (อัปเดตเป็นหน่วย CM)
// ================================================================
volatile long encL_cnt = 0;
volatile long encR_cnt = 0;

long lastEncL_cnt = 0;
long lastEncR_cnt = 0;

float wheelPos_cm    = 0.0f; 
float wheelVel_cmps  = 0.0f; 

// Nidec 24H Direct Drive (100 Line * 2 edges = 200 ticks/rev)
const float COUNTS_PER_REV    = 90.0f; 
const float WHEEL_DIAMETER_CM = 8.0f; 
const float WHEEL_CIRC_CM     = PI_F * WHEEL_DIAMETER_CM;
const float CM_PER_COUNT      = WHEEL_CIRC_CM / COUNTS_PER_REV; 

// ================================================================
// 5) FULL-STATE FEEDBACK GAINS (PID)
// ================================================================

//ค่าที่ได้จาก LQR
float Kth   =42.5819f;   // (Angle P) พยุงตัว
float Kd    = 11.1448f;    // (Angle D) ต้านการแกว่ง
float Kx    = 0.8367f;   // (Position P) 
float Kv    = 4.1772f;    // (Velocity D)
float KiPos = 0.01f;    // (Position I) 


float posInt = 0.0f;
const float POS_INT_LIMIT = 100.0f; 

float angleOffsetDeg = 3.0f;  //6.0 ตอนไม่มีหัว

// 💡 ตัวแปรใหม่สำหรับบันทึกกราฟ Setpoint ไปแสดงผลบนหน้าเว็บ
float currentSetpoint = 0.0f; 

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

// 💡 ปรับความเร่ง (Slew Rate) กลับขึ้นมาเพื่อให้มอเตอร์กระชากตัวพยุงได้ทัน
const float SLEW_MAX = 15000.0f; 
const float MAX_U_HW = 255.0f; 

// 💡 ลดค่า MIN_PWM ลง ถ้ามอเตอร์เป็น Direct Drive มักจะไม่ต้องการแรงเตะเริ่มต้นเยอะ
// 💡 ปรับ MIN_PWM และ DEADZONE ขึ้นเล็กน้อย เพื่อให้มอเตอร์เอาชนะแรงเสียดทานตอนเริ่มหมุนสวนทางได้
const float MIN_PWM = 15.0f;     
const float PWM_DEADZONE = 5.0f; 

const float BRAKE_ANGLE_DEAD = 2.0f;
const float BRAKE_VEL_LIMIT  = 2.0f; 
float Bbrake = 0.1f;

const float SNAP_ANGLE_DEG = 5.0f;
const float SNAP_KV_EXTRA  = 1.2f; 

const bool INVERT_L = false;
const bool INVERT_R = true;

// Joystick Control Variables (ใส่ระบบ Target สำหรับทำ Soft-Start)
float targetSpeedCmd = 0.0f;
float targetTurnCmd  = 0.0f;
float speedCmd = 0.0f;          
float turnSpeed = 0.0f;         

// Serial Parsing Buffers
char rxBuf[64];
int rxIdx = 0;
uint32_t lastEspRxTime = 0; 

// ================================================================
// 8) MPU6050 HELPERS
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
    if (mpuReadBytes(0x47, buf, 2)) { // 💡 เปลี่ยนมาอ่าน Register 0x47 (Gyro Z)
      int16_t gz = (int16_t)((buf[0] << 8) | buf[1]);
      sum += gz; good++;
    }
    delay(2);
  }
  if (good < 10) { gyroBiasX = 0.0f; return; }
  gyroBiasX = ((float)sum / (float)good) / 131.0f;
  digitalWrite(ledPin, HIGH); // (Active HIGH) ไฟติด บอกว่าคาลิเบรตเสร็จแล้ว
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
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]); // 💡 อ่านแกน Y (ชี้ไปด้านหน้า-หลัง แทนแกน Z)
  if (!mpuReadBytes(0x47, buf, 2)) return;        // 💡 เปลี่ยนมาอ่าน Register 0x47 (Gyro Z)
  int16_t gz = (int16_t)((buf[0] << 8) | buf[1]);

  float ax_g = (float)ax / 16384.0f;
  float ay_g = (float)ay / 16384.0f;
  
  // 💡 สูตรใหม่! ใช้ atan2f(Y, X) จะได้ 0 องศาพอดีตอนที่บอร์ดตั้งตรง โดยไม่ต้องมี Offset มากวนใจ
  float rawAccAngle = atan2f(ay_g, ax_g) * (180.0f / PI_F);

  // 💡 ถ้าติดชิปกลับหัว (แกน X ชี้ขึ้นฟ้า) ค่ามันจะเป็น +-180 โค้ดนี้จะหักลบให้เหลือ 0 เองอัตโนมัติ
  if (rawAccAngle > 90.0f) {
    rawAccAngle -= 180.0f;
  } else if (rawAccAngle < -90.0f) {
    rawAccAngle += 180.0f;
  }
  
  rawAccAngle *= ACC_SIGN;

  // 💡 ใช้ GYRO_SIGN เฉพาะของ Gyroscope
  float gyro_raw_dps = (((float)gz / 131.0f) - gyroBiasX) * GYRO_SIGN;

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
// 9) ENCODER INTERRUPT SERVICE ROUTINES
// ================================================================
void isrEncLA() { if (digitalRead(encLA) == digitalRead(encLB)) encL_cnt++; else encL_cnt--; }
void isrEncRA() { if (digitalRead(encRA) == digitalRead(encRB)) encR_cnt++; else encR_cnt--; }

// ================================================================
// 10) MOTOR DRIVER FUNCTIONS
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
  
  if (abs(pitchDeg) < angDead && abs(wheelVel_cmps) < velDead && abs(u) < 5.0f && abs(turn) < 2.0f) {
    brakeAll();
    lastU = 0.0f;
    return;
  }

  setMotorRaw(pwmL, dirL, enL, (int)left, INVERT_L);
  setMotorRaw(pwmR, dirR, enR, (int)right, INVERT_R);
}

// ================================================================
// 11) COMMAND & TELEMETRY
// ================================================================
float parseFloatFrom(const char* s) { return (float)atof(s); }

void processLine(char* line) {
  // --- 💡 ส่วนประมวลผลคำสั่งจอยสติ๊กจาก ESP32 ---
  if (line[0] == '*') {
    
    lastEspRxTime = millis(); 

    char* comma = strchr(line, ',');
    char* hash = strchr(line, '#');
    char* comma2 = nullptr;

    if (comma != nullptr && hash != nullptr && hash > comma) {
      
      // ค้นหา Comma ตัวที่สอง (ถ้ามีแปลว่า ESP32 ส่งพารามิเตอร์ Fine Turn มาด้วย)
      comma2 = strchr(comma + 1, ',');
      if (comma2 != nullptr && comma2 >= hash) comma2 = nullptr;

      *comma = '\0';  
      *hash = '\0';  
      
      float forwardVal = (float)atoi(line + 1); 
      float turnVal = 0.0f;
      bool isFineTurn = false;
      
      if (comma2 != nullptr) {
          *comma2 = '\0';
          turnVal = (float)atoi(comma + 1); 
          if (atoi(comma2 + 1) == 1) isFineTurn = true;
      } else {
          turnVal = (float)atoi(comma + 1); 
      }
      
      // 💡 ลด Deadzone ที่เคยตัดคำสั่งทิ้ง (เดิม 5.0 ทำให้เลี้ยวเบาๆ หายไปหมด)
      if (abs(forwardVal) < 1.0f) forwardVal = 0.0f;
      if (abs(turnVal) < 1.0f) turnVal = 0.0f;
      
      targetSpeedCmd = forwardVal * 7.5f;  
      
      // 💡 เพิ่มตัวคูณแรงหมุนให้สูงทะลุแรงพยุงตัว (u) ล้อจะได้ถูกบังคับให้หมุนสวนทางกันจริงๆ
      if (isFineTurn) {
          targetTurnCmd = turnVal * 2.0f; // จูนเพิ่มเป็น 8
      } else {
          targetTurnCmd = turnVal * 4.0f; // 💡 จูนเพิ่มจาก 8 เป็น 18 (แม็กซ์ที่ 180) หมุนสะใจแน่นอน
      }
    }
    return;
  }

  // --- ส่วนประมวลผลคำสั่ง PID Tuning ---
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
      posInt = 0; wheelPos_cm = 0; 
      break;
    case 'S': 
      if (val > 0) { runEnabled = true; } else { runEnabled = false; brakeAll(); }
      break;
  }
}

void readESP32() {
  while (SerialESP.available()) {
    char c = (char)SerialESP.read();
    
    // ทิ้งอักขระที่ไม่จำเป็น
    if (c == '\r') continue;
    
    // 💡 ถ้าเจอตัว * ให้ล้าง Buffer ใหม่เสมอ (ตัดปัญหาข้อมูลขยะปนมาข้างหน้า)
    if (c == '*') {
      rxIdx = 0; 
    }
    
    // 💡 ถ้าเจอการจบแพ็กเก็ตด้วย \n (PID Command) หรือ # (Joystick Command) ให้ประมวลผลทันที
    if (c == '\n' || c == '#') {
      if (c == '#') rxBuf[rxIdx++] = c; // เก็บ # ไว้เช็คใน processLine
      
      rxBuf[rxIdx] = '\0'; // ปิดท้าย String
      
      if (rxIdx > 0) { 
        processLine(rxBuf);
      }
      rxIdx = 0; // เคลียร์ Index เตรียมรับข้อความใหม่
    } else {
      if (rxIdx < 63) rxBuf[rxIdx++] = c; // เก็บตัวอักษรลง Buffer
    }
  }
}

void sendTelemetry() {
  static uint32_t lastTel = 0;
  if (millis() - lastTel > 50) { 
     lastTel = millis();
     
     int espLinkActive = (millis() - lastEspRxTime < 1000) ? 1 : 0; 
     
     SerialESP.print(millis()); SerialESP.print(",");
     SerialESP.print(pitchDeg, 2); SerialESP.print(",");
     SerialESP.print(gyroX_dps, 2); SerialESP.print(",");
     SerialESP.print(wheelPos_cm, 2); SerialESP.print(","); 
     SerialESP.print(wheelVel_cmps, 2); SerialESP.print(","); 
     SerialESP.print(lastU, 1); SerialESP.print(",");
     SerialESP.print(runEnabled ? 1 : 0); SerialESP.print(",");
     SerialESP.print(Kth, 1); SerialESP.print(",");
     SerialESP.print(Kd, 1); SerialESP.print(",");
     SerialESP.print(Kx, 1); SerialESP.print(",");
     SerialESP.print(Kv, 1); SerialESP.print(",");
     SerialESP.print(KiPos, 4); SerialESP.print(",");
     SerialESP.print(angleOffsetDeg, 2); SerialESP.print(",");
     SerialESP.print(espLinkActive); SerialESP.print(",");
     
     // 💡 ส่งค่า Setpoint (พารามิเตอร์ที่ 14) ไปให้ ESP32 นำไปพล็อตกราฟ
     SerialESP.println(currentSetpoint, 2); 
  }
}

// ================================================================
// 12) SETUP & MAIN LOOP
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
  digitalWrite(ledPin, LOW); // (Active HIGH) สั่งปิดไฟตอนเริ่ม Setup

  Serial.begin(115200); 
  SerialESP.begin(115200);
  Wire.begin(); Wire.setClock(400000); 

  if (!mpuInit()) {
    while (1) { digitalWrite(ledPin, !digitalRead(ledPin)); delay(100); }
  }
  mpuCalibrateGyro();

  lastEncL_cnt = encL_cnt; lastEncR_cnt = encR_cnt;
  digitalWrite(ledPin, HIGH); // (Active HIGH) เปิดไฟค้างไว้พร้อมทำงาน
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
  long curL = encL_cnt;
  long curR = -encR_cnt; 
  float dPos = (0.5f * (float)((curL-lastEncL_cnt) + (curR-lastEncR_cnt))) * CM_PER_COUNT;
  wheelPos_cm += dPos;
  
  wheelVel_cmps = 0.8f * wheelVel_cmps + 0.2f * (dPos / dt); 
  
  lastEncL_cnt = curL; lastEncR_cnt = curR;

  // 3. Command
  readESP32();
  
  // ตัดระบบเซฟตี้ ถ้าสายหลุด ให้ลดค่าเป้าหมายลงเป็น 0 อย่างนุ่มนวล
  if (millis() - lastEspRxTime >= 1000) {
      targetSpeedCmd = 0.0f; 
      targetTurnCmd = 0.0f;
  }

  // 4. Safety Logic
  if (abs(pitchDeg) > FALL_ANGLE_DEG) {
    if (fallDuration == 0) fallDuration = nowMs;
    if (nowMs - fallDuration > 100) { 
      runEnabled = false; armStableStart = 0; brakeAll();
      wheelPos_cm = 0; posInt = 0; lastU = 0; 
      currentSetpoint = angleOffsetDeg; // รีเซ็ตกราฟ Setpoint เวลามันล้ม
    }
  } else {
    fallDuration = 0;
    if (!runEnabled && abs(pitchDeg) < ARM_ANGLE_DEG && abs(gyroX_dps) < ARM_GYRO_DPS) {
      if (armStableStart == 0) armStableStart = nowMs;
      else if (nowMs - armStableStart > ARM_MS) {
         runEnabled = true; wheelPos_cm = 0; posInt = 0; lastU = 0;
         digitalWrite(ledPin, HIGH); // (Active HIGH) เปิดไฟตอนเริ่มตั้งไข่
         targetSpeedCmd = 0; targetTurnCmd = 0; // รีเซ็ตคำสั่งตอนเริ่มตั้งไข่
      }
    } else { armStableStart = 0; }
  }

  // 5. PID Control Logic
  float u = 0.0f;
  if (runEnabled) {
    
    speedCmd  = 0.95f * speedCmd  + 0.05f * targetSpeedCmd;
    // 💡 ปรับให้การหมุน (Turn) ตอบสนองไวขึ้น ลดความเฉื่อย เพื่อให้หุ่นบิดตัวได้ทันทีที่สั่ง
    turnSpeed = 0.85f * turnSpeed + 0.15f * targetTurnCmd;
      
    bool isUserCommanding = (abs(targetSpeedCmd) > 0.1f || abs(targetTurnCmd) > 0.1f);
    
    // 💡 ตัวแปรใหม่: ใช้แยกแยะว่าการเคลื่อนที่นี้เกิดจาก "จอยสติ๊ก" หรือ "โดนผลักภายนอก"
    static bool userWasDriving = false;

    if (isUserCommanding) {
        // ถ้าผู้ใช้ดันจอย ให้จำไว้ว่าอยู่ในโหมด "กำลังขับขี่"
        userWasDriving = true;
    } else if (abs(wheelVel_cmps) < 5.0f) {
        // ถ้าปล่อยจอยแล้ว และรถเบรกจนความเร็วเกือบหยุดสนิท ถึงจะออกจากโหมดขับขี่
        userWasDriving = false;
    }

    if (userWasDriving) {
        // 🟢 โหมดขับขี่ / กำลังไถลเบรก: รีเซ็ตพิกัดทิ้งไปเรื่อยๆ เพื่อไม่ให้เกิดอาการหนังยาง
        wheelPos_cm = 0.0f; 
        posInt = 0.0f; 
    } else {
        // 🔴 โหมดรักษาตำแหน่ง (ยืนนิ่ง): ถ้ารถโดนผลัก พิกัด wheelPos_cm จะสะสมและดึงรถกลับมาที่เดิม!
        if (KiPos > 0.0f) {
            posInt += wheelPos_cm * dt;
            posInt *= 0.99f; 
            posInt = constrain(posInt, -POS_INT_LIMIT, POS_INT_LIMIT);
        }
    }
    
    float targetAngleOffset = speedCmd * 0.03f; 
    currentSetpoint = angleOffsetDeg + targetAngleOffset; 
    
    float errTh = pitchDeg - currentSetpoint;
    float errVel = wheelVel_cmps - speedCmd; 
    
    // คลายเบรกอัตโนมัติ 50% เฉพาะตอนที่ผู้ใช้ขับ หรือรถกำลังไถลเบรกจากการขับ
    if (userWasDriving) {
        errVel *= 0.3f; 
    }
    
    float uRaw = Kth*(errTh) + Kd*gyroX_dps + Kx*wheelPos_cm + Kv*errVel + KiPos*posInt;
    
    // ระบบกระชากให้รถหยุดนิ่งสนิท จะทำงานก็ต่อเมื่อไม่ได้อยู่ในโหมดขับขี่
    if (abs(errTh) < SNAP_ANGLE_DEG && !userWasDriving) {
       uRaw -= SNAP_KV_EXTRA * wheelVel_cmps;
    }
    
    static float uFiltered = 0.0f;
    uFiltered = 0.2f * uFiltered + 0.8f * uRaw; 
    
    u = uFiltered;
    
    drive(u, turnSpeed);
  } else {
    brakeAll();
    lastU = 0; 
    currentSetpoint = angleOffsetDeg; // รีเซ็ตกราฟ Setpoint เวลาหยุดทำงาน
  }

  // 6. Telemetry และ เช็คสายสัญญาณไฟ LED
  sendTelemetry();
  
  // 💡 เช็คสถานะการเชื่อมต่อสายไฟแบบเข้มงวด (แก้เป็น Active HIGH)
  if (millis() - lastEspRxTime < 500) {
      // 🟢 สายปกติ: ไฟจะกระพริบช้าๆ เป็นจังหวะชัดเจน (สว่างสลับดับ)
      digitalWrite(ledPin, (millis() / 150) % 2); 
  } else if (runEnabled) {
      // 🔴 หุ่นทำงานอยู่แต่สายหลุด -> ไฟดับค้าง (Active HIGH -> LOW = OFF)
      digitalWrite(ledPin, LOW); 
  } else {
      // 🔴 หุ่นล้ม/ไม่ทำงาน และสายหลุด -> ไฟติดค้าง (Active HIGH -> HIGH = ON)
      digitalWrite(ledPin, HIGH);  
  }
}