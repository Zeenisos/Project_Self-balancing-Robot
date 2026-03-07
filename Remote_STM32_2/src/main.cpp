#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <qrcode.h>
#include <nRF24L01.h>
#include <RF24.h>

/* ==========================================
 * nRF24L01 Remote + Joystick (COMPLETE FINAL)
 * ==========================================
 */

// ================= CONFIG =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// STM32 nRF24L01 Pins
#define CE_PIN    PA1
#define CSN_PIN   PA4

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"00001", "00002"}; // ท่อส่งข้อมูล (Pipe Addresses)

// Joystick Pins 
#define JOY_X_PIN PB0 
#define JOY_Y_PIN PB1
#define JOY_SW_PIN PB12

QRCode qrcode;
char rxBuffer[32]; // nRF24 รับได้สูงสุด 32 Bytes
char qrString[60];
unsigned long lastSendTime = 0;
unsigned long lastHeartbeat = 0; 
unsigned long requestTime = 0; 

bool showingQR = false; 
bool waitingReply = false; 
bool blinkState = false; 
bool isConnected = false;
bool gpsFix = false;

// ตัวแปร Calibrate
int centerX = 2048, centerY = 2048;

// ================= TUNING =================
#define JOY_RANGE 350 
#define DEADZONE 20  

void showMessage(const char* title, const char* subtitle) {
    showingQR = true; 
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20); 
    display.print(title); 
    display.setTextSize(1);
    display.setCursor(0, 45);
    display.print(subtitle); 
    display.display();
}

void showQRCode(const char* data, bool isWeb) {
    showingQR = true; 
    waitingReply = false;
    
    if (isWeb) {
        strcpy(qrString, data);
    } else {
        strcpy(qrString, "geo:");
        strcat(qrString, data);
    }
    
    uint8_t qrcodeData[qrcode_getBufferSize(2)];
    qrcode_initText(&qrcode, qrcodeData, 2, 0, qrString);
    int scale = 2; 
    int qrSizePx = qrcode.size * scale; 
    int y_offset = (SCREEN_HEIGHT - qrSizePx) / 2;
    int x_offset = (SCREEN_WIDTH - qrSizePx) / 2;
    
    display.clearDisplay();
    display.fillRect(x_offset - 4, y_offset - 4, qrSizePx + 8, qrSizePx + 8, SSD1306_WHITE);
    for (uint8_t y = 0; y < qrcode.size; y++) {
        for (uint8_t x = 0; x < qrcode.size; x++) {
            if (qrcode_getModule(&qrcode, x, y)) {
                display.fillRect(x_offset + (x * scale), y_offset + (y * scale), scale, scale, SSD1306_BLACK);
            }
        }
    }
    
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    if(isWeb) display.print("WEB IP"); else display.print("GPS MAP");
    display.display();
}

void setup() {
    Wire.begin(); 
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { while(1); }
    
    pinMode(JOY_SW_PIN, INPUT_PULLUP);
    pinMode(JOY_X_PIN, INPUT);
    pinMode(JOY_Y_PIN, INPUT);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println("Calibrating...");
    display.println("DO NOT TOUCH!");
    display.display();
    
    delay(1000); 
    long sumX = 0, sumY = 0;
    for(int i=0; i<50; i++) { 
        sumX += analogRead(JOY_X_PIN);
        sumY += analogRead(JOY_Y_PIN);
        delay(5);
    }
    centerX = sumY / 50;
    centerY = sumX / 50;
    
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Remote Ready");
    display.display();
    delay(500);

    // Setup nRF24L01
    if (!radio.begin()) {
        display.println("nRF24 Error!");
        display.display();
        while (1);
    }
    
    radio.setPALevel(RF24_PA_MAX);       // กำลังส่งสูงสุด
    radio.setDataRate(RF24_250KBPS);     // 250kbps เพื่อให้ได้ระยะทางไกลที่สุด
    radio.openWritingPipe(address[0]);   // รีโมทเขียนลงท่อ 0
    radio.openReadingPipe(1, address[1]);// รีโมทอ่านจากท่อ 1
    radio.startListening();              // เปิดโหมดรอรับข้อมูล

    display.clearDisplay();
}

void loop() {
    // 1. ตรวจสอบปุ่มกด (SW)
    if (digitalRead(JOY_SW_PIN) == LOW) {
        delay(50); 
        if (digitalRead(JOY_SW_PIN) == LOW) {
            
            while(digitalRead(JOY_SW_PIN) == LOW); // รอปล่อย
            
            bool doubleClick = false;
            long releaseTime = millis();
            while (millis() - releaseTime < 300) {
                if (digitalRead(JOY_SW_PIN) == LOW) {
                    doubleClick = true;
                    while(digitalRead(JOY_SW_PIN) == LOW);
                    break;
                }
            }

            waitingReply = true;
            requestTime = millis();
            showingQR = false;
            display.clearDisplay(); 
            display.setCursor(10, 30);

            radio.stopListening(); // หยุดฟังเพื่อส่งข้อมูล
            if (doubleClick) {
                const char text[] = "IP?";
                radio.write(&text, sizeof(text));
                display.print("Get Web IP...");
            } else {
                const char text[] = "GET";
                radio.write(&text, sizeof(text));
                display.print("Get GPS...");
            }
            radio.startListening(); // กลับมาโหมดฟัง

            display.display();
        }
    }

    // 2. ส่งค่า Joystick
    if (millis() - lastSendTime > 250) {
        lastSendTime = millis();

        int xVal = analogRead(JOY_X_PIN);
        int yVal = analogRead(JOY_Y_PIN);
        
        int diffX = -(xVal - centerX);
        int diffY = -(yVal - centerY);
        
        int mapX = constrain(map(diffX, -JOY_RANGE, JOY_RANGE, -100, 100), -100, 100);
        int mapY = constrain(map(diffY, -JOY_RANGE, JOY_RANGE, -100, 100), -100, 100);

        if (abs(mapX) < 90) mapX = 0; else mapX = (mapX > 0) ? 100 : -100;
        if (abs(mapY) < 90) mapY = 0; else mapY = (mapY > 0) ? 100 : -100;
        if (mapY != 0) { mapX = 0; } // MUTUAL EXCLUSION

        if (showingQR) {
            if (abs(mapX) > 50 || abs(mapY) > 50) {
                showingQR = false; 
                display.clearDisplay(); 
            }
        }

        if (!showingQR && !waitingReply) {
            
            // เตรียมข้อมูลเป็น String ชุดเดียวและส่ง
            char cmdStr[32];
            sprintf(cmdStr, "C,%d,%d", mapX, mapY);
            
            radio.stopListening();
            radio.write(&cmdStr, strlen(cmdStr) + 1); // +1 เพื่อรวมตัวปิด String (Null Terminator)
            radio.startListening();
            
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); 
            display.setCursor(0, 0); display.print("CMD: "); 
            
            if (millis() - lastHeartbeat < 3000) isConnected = true;
            else { isConnected = false; gpsFix = false; }

            blinkState = !blinkState;

            // Connection Dot
            if (isConnected) display.fillRect(120, 0, 6, 6, SSD1306_WHITE);
            else if (blinkState) display.fillRect(120, 0, 6, 6, SSD1306_WHITE);
            else display.fillRect(120, 0, 6, 6, SSD1306_BLACK);

            // GPS Status Dot
            if (isConnected && gpsFix) display.fillRect(120, 58, 6, 6, SSD1306_WHITE);
            else if (isConnected && blinkState) display.fillRect(120, 58, 6, 6, SSD1306_WHITE);
            else display.fillRect(120, 58, 6, 6, SSD1306_BLACK);

            display.setCursor(0, 15); display.print("X: "); display.print(mapX); display.print("   ");
            display.setCursor(0, 30); display.print("Y: "); display.print(mapY); display.print("   ");

            // Bar Graph (X Axis)
            display.fillRect(0, 45, 115, 19, SSD1306_BLACK); 
            display.drawRect(10, 50, 90, 10, SSD1306_WHITE); 
            int barLen = mapX / 2.5; 
            if (barLen > 0) display.fillRect(55, 52, barLen, 6, SSD1306_WHITE);
            else display.fillRect(55 + barLen, 52, -barLen, 6, SSD1306_WHITE);

            display.display();
        }
    }

    // 3. Timeout Logic
    if (waitingReply && millis() - requestTime > 3000) {
        waitingReply = false;
        showMessage("Time Out", "Try Again");
        delay(1000); 
        showingQR = false; 
    }

    // 4. รอรับข้อมูล nRF24L01
    if (radio.available()) {
        memset(rxBuffer, 0, sizeof(rxBuffer)); // ล้างค่าเก่า
        radio.read(&rxBuffer, sizeof(rxBuffer));

        lastHeartbeat = millis();

        if (strncmp(rxBuffer, "HB:", 3) == 0) {
            gpsFix = (rxBuffer[3] == '1');
        }
        else if (strncmp(rxBuffer, "IP:", 3) == 0) {
            showQRCode(&rxBuffer[3], true); 
        }
        else if (strcmp(rxBuffer, "NO_GPS") == 0) {
            showMessage("NO GPS", "Move Joy to Exit");
        } 
        else if (strchr(rxBuffer, ',') != NULL) {
            if (strncmp(rxBuffer, "C,", 2) != 0) { 
               showQRCode(rxBuffer, false); 
            }
        }
    }
}