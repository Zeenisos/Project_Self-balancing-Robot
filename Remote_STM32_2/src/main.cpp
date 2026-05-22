#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <qrcode.h>
#include <nRF24L01.h>
#include <RF24.h>

/* ==========================================
 * nRF24L01 Remote + Joystick (VERIFIED & TUNED)
 * ==========================================
 */

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// STM32 nRF24L01 Pins
#define CE_PIN    PA1
#define CSN_PIN   PA4

RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"00001", "00002"};

// Joystick Pins 
#define JOY_X_PIN PB0 
#define JOY_Y_PIN PB1
#define JOY_SW_PIN PB12
#define LED_PIN PA8 

QRCode qrcode;
char rxBuffer[32]; 
char qrString[60];
unsigned long lastSendTime = 0;
unsigned long lastHeartbeat = 0; 
unsigned long requestTime = 0; 

bool showingQR = false; 
bool waitingReply = false; 
bool blinkState = false; 
bool isConnected = false;
bool gpsFix = false;
bool botLinkOk = false; // เพิ่มตัวแปรเช็คสถานะสาย TX/RX ของหุ่นยนต์

// ตัวแปร Calibrate
int centerX = 512, centerY = 512;

#define DEADZONE 45  

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
    
    if (isWeb) strcpy(qrString, data);
    else { strcpy(qrString, "geo:"); strcat(qrString, data); }
    
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
    delay(1000); 

    Wire.begin(); 
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { while(1); }
    
    pinMode(JOY_SW_PIN, INPUT_PULLUP);
    pinMode(JOY_X_PIN, INPUT);
    pinMode(JOY_Y_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);

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
    centerX = sumX / 50; 
    centerY = sumY / 50;
    
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Init nRF24...");
    display.display();
    
    // ==========================================
    // ส่วนปรับปรุงการเชื่อมต่อ SPI สำหรับโมดูลที่เซนซิทีฟ
    // ==========================================
    SPI.begin();
    delay(500); // หน่วงเวลาให้ไฟ 3.3V นิ่ง และให้ชิป nRF24 บูทตัวเองให้เสร็จ

    bool nrfStarted = false;
    for(int i = 0; i < 5; i++) { // พยายามเชื่อมต่อ 5 ครั้ง
        if (radio.begin()) {
            nrfStarted = true;
            break;
        }
        display.print("."); // แสดงจุดไข่ปลาเวลากำลังพยายามเชื่อมต่อ
        display.display();
        delay(300);
    }

    if (!nrfStarted) {
        display.clearDisplay();
        display.setCursor(0, 20);
        display.println("nRF24 Error!");
        display.println("Check Wiring/Power");
        display.display();
        while (1) {
            digitalWrite(LED_PIN, HIGH); delay(100);
            digitalWrite(LED_PIN, LOW); delay(100);
        }
    }
    
    // --- สั่งล้างสมองชิป (Flush & Reset) ป้องกันอาการชิปค้างจากรอบที่แล้ว ---
    radio.powerDown();
    delay(50);
    radio.powerUp();
    delay(50);
    radio.flush_tx(); // ล้างข้อมูลขยะที่อาจจะค้างท่อตอนไฟตก
    radio.flush_rx();
    // -----------------------------------------------------------

    radio.setPALevel(RF24_PA_LOW);       // <--- ปรับความแรงขึ้นมาเป็น LOW
    radio.setDataRate(RF24_1MBPS);       // <--- แก้บั๊กชิปโคลน: เปลี่ยนเป็น 1 MBPS
    radio.setChannel(100);               
    radio.setPayloadSize(32); 
    radio.setRetries(15, 15); 
    
    radio.openWritingPipe(address[0]);   
    radio.openReadingPipe(1, address[1]);
    radio.startListening();              

    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Remote Ready");
    display.display();
    delay(500);
    
    display.clearDisplay();
    display.display();
}

void loop() {
    int currentX = analogRead(JOY_X_PIN);
    int currentY = analogRead(JOY_Y_PIN);
    bool isMoving = (abs(currentX - centerX) > DEADZONE) || (abs(currentY - centerY) > DEADZONE);

    if (digitalRead(JOY_SW_PIN) == LOW && !isMoving) { 
        delay(50); 
        currentX = analogRead(JOY_X_PIN);
        currentY = analogRead(JOY_Y_PIN);
        isMoving = (abs(currentX - centerX) > DEADZONE) || (abs(currentY - centerY) > DEADZONE);

        if (digitalRead(JOY_SW_PIN) == LOW && !isMoving) {
            
            while(digitalRead(JOY_SW_PIN) == LOW); 
            
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

            radio.stopListening(); 
            if (doubleClick) {
                char text[32] = "IP?";
                radio.write(&text, sizeof(text));
                display.print("Get Web IP...");
            } else {
                char text[32] = "GET";
                radio.write(&text, sizeof(text));
                display.print("Get GPS...");
            }
            radio.startListening(); 

            display.display();
        }
    }

    if (millis() - lastSendTime > 150) {
        lastSendTime = millis();

        int xVal = (analogRead(JOY_X_PIN) + analogRead(JOY_X_PIN) + analogRead(JOY_X_PIN)) / 3;
        int yVal = (analogRead(JOY_Y_PIN) + analogRead(JOY_Y_PIN) + analogRead(JOY_Y_PIN)) / 3;
        
        int mapX = 0;
        int mapY = 0;

        if (xVal > (centerX + DEADZONE)) {
            mapX = map(xVal, centerX + DEADZONE, 1023, 0, 100);
        } else if (xVal < (centerX - DEADZONE)) {
            mapX = map(xVal, 0, centerX - DEADZONE, -100, 0);
        }
        mapX = constrain(mapX, -100, 100);

        if (yVal > (centerY + DEADZONE)) {
            mapY = map(yVal, centerY + DEADZONE, 1023, 0, 100);
        } else if (yVal < (centerY - DEADZONE)) {
            mapY = map(yVal, 0, centerY - DEADZONE, -100, 0);
        }
        mapY = constrain(mapY, -100, 100);
        
        if (abs(mapY) > abs(mapX)) mapX = 0; 
        else if (abs(mapX) > abs(mapY)) mapY = 0; 

        if (mapX >= 80) mapX = 100;
        else if (mapX <= -80) mapX = -100;
        else mapX = 0;

        if (mapY >= 80) mapY = 100;
        else if (mapY <= -80) mapY = -100;
        else mapY = 0;

        if (showingQR) {
            if (abs(mapX) > 50 || abs(mapY) > 50) {
                showingQR = false; 
                display.clearDisplay(); 
            }
        }

        if (!showingQR && !waitingReply) {
            
            char cmdStr[32] = {0}; // <--- จองพื้นที่ 32 ไบต์ และเติม 0 ให้เต็ม
            sprintf(cmdStr, "C,%d,%d", mapX, mapY);
            
            radio.stopListening();
            radio.write(&cmdStr, sizeof(cmdStr)); // <--- ส่ง 32 ไบต์เต็ม (ไม่ใช่ strlen)
            radio.startListening();
            
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); 
            display.setCursor(0, 0); display.print("CMD: "); 
            
            if (millis() - lastHeartbeat < 3000) isConnected = true;
            else { isConnected = false; gpsFix = false; botLinkOk = false; }

            blinkState = !blinkState;

            if (isConnected) digitalWrite(LED_PIN, HIGH);
            else digitalWrite(LED_PIN, blinkState); 

            // ========================================================
            // วาดตัวอักษรและจุดไข่ปลาสถานะ 3 ชุด (RF, BOT, GPS) ให้อยู่ทางขวา
            // ========================================================
            display.setCursor(95, 0);  display.print("RF");
            display.setCursor(95, 15); display.print("BOT");
            display.setCursor(95, 30); display.print("GPS");

            // 1. Connection Dot (คลื่น nRF24L01)
            if (isConnected) display.fillRect(120, 0, 6, 6, SSD1306_WHITE);
            else if (blinkState) display.fillRect(120, 0, 6, 6, SSD1306_WHITE);
            else display.fillRect(120, 0, 6, 6, SSD1306_BLACK);

            // 2. Bot Link Dot (สาย TX/RX ระหว่าง ESP32 <-> STM32)
            if (isConnected && botLinkOk) display.fillRect(120, 15, 6, 6, SSD1306_WHITE);
            else if (isConnected && blinkState) display.fillRect(120, 15, 6, 6, SSD1306_WHITE);
            else display.fillRect(120, 15, 6, 6, SSD1306_BLACK);

            // 3. GPS Status Dot
            if (isConnected && gpsFix) display.fillRect(120, 30, 6, 6, SSD1306_WHITE);
            else if (isConnected && blinkState) display.fillRect(120, 30, 6, 6, SSD1306_WHITE);
            else display.fillRect(120, 30, 6, 6, SSD1306_BLACK);
            // ========================================================

            display.setCursor(0, 15); display.print("X: "); display.print(mapX); display.print("   ");
            display.setCursor(0, 30); display.print("Y: "); display.print(mapY); display.print("   ");

            display.fillRect(0, 45, 128, 19, SSD1306_BLACK); // ลบพื้นหลังของกราฟแท่งให้กว้างขึ้น
            display.drawRect(10, 50, 90, 10, SSD1306_WHITE); 
            int barLen = mapX / 2.5; 
            if (barLen > 0) display.fillRect(55, 52, barLen, 6, SSD1306_WHITE);
            else display.fillRect(55 + barLen, 52, -barLen, 6, SSD1306_WHITE);

            display.display();
        }
    }

    if (waitingReply && millis() - requestTime > 3000) {
        waitingReply = false;
        showMessage("Time Out", "Try Again");
        delay(1000); 
        showingQR = false; 
        display.clearDisplay(); 
    }

    if (radio.available()) {
        memset(rxBuffer, 0, sizeof(rxBuffer));
        radio.read(&rxBuffer, sizeof(rxBuffer));
        lastHeartbeat = millis();

        if (strncmp(rxBuffer, "HB:", 3) == 0) {
            gpsFix = (rxBuffer[3] == '1');
            // แกะสถานะ BOT ออกมาจาก Heartbeat (เช่น HB:1,1)
            if (rxBuffer[4] == ',' && rxBuffer[5] == '1') {
                botLinkOk = true;
            } else {
                botLinkOk = false;
            }
        }
        else if (strncmp(rxBuffer, "IP:", 3) == 0) showQRCode(&rxBuffer[3], true); 
        else if (strcmp(rxBuffer, "NO_GPS") == 0) showMessage("NO GPS", "Move Joy to Exit");
        else if (strchr(rxBuffer, ',') != NULL) {
            if (strncmp(rxBuffer, "C,", 2) != 0) showQRCode(rxBuffer, false); 
        }
    }
}