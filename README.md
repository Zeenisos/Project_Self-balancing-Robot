🤖 MyBalancingRobot (หุ่นยนต์สองล้อสมดุลอัจฉริยะ)

โปรเจกต์หุ่นยนต์สองล้อสมดุล (Self-Balancing Robot) ที่มาพร้อมกับระบบควบคุมที่หลากหลาย รองรับทั้งการบังคับผ่าน รีโมทคอนโทรลแบบสร้างเอง (Custom Remote) และการสั่งงานผ่าน Web Application พร้อมระบบนำทางด้วยพิกัด GPS และแสดงออกทางอารมณ์ผ่านจอ OLED

✨ ฟีเจอร์หลัก (Key Features)

⚖️ Self-Balancing: ทรงตัวอัตโนมัติด้วยเซ็นเซอร์ MPU6050 และสมการควบคุมแบบ PID (Full-State Feedback)

🌐 Web-based Control: ควบคุมหุ่นยนต์ผ่านหน้าเว็บ (ESP32 Web Server) พร้อมแผนที่ Leaflet.js

📍 GPS Waypoint Navigation: ระบบนำทางอัตโนมัติไปยังจุดหมายที่จิ้มบนแผนที่ (ใช้ GPS ร่วมกับเข็มทิศ QMC5883L)

🎮 Custom RF Remote: รีโมทคอนโทรลหน้าจอ OLED ส่งสัญญาณผ่าน nRF24L01 (2.4GHz) ระยะไกล

👀 Interactive OLED Eyes: จอแสดงผลใบหน้า/ดวงตาของหุ่นยนต์ที่ตอบสนองตามมุมเอียง (Pitch Angle)

🤖 Animatronic Head: ระบบ Servo คู่สำหรับหมุนหัว (Pan/Tilt) และขยับหู/ตา พร้อมโหมด Auto-Scan ส่ายหัวอัตโนมัติ

📂 โครงสร้างโปรเจกต์ (Project Structure)

โปรเจกต์นี้ถูกแบ่งออกเป็น 3 ส่วนหลัก (3 โฟลเดอร์ใน PlatformIO Workspace) ซึ่งทำงานร่วมกันแบบ Multi-MCU:

MYBALANCINGROBOT/
│
├── 📁 Self-balancing_STM32/    # [STM32] หัวใจหลักของระบบทรงตัว
│   └── ควบคุมมอเตอร์, อ่านค่า Encoder, คำนวณ PID, อ่านค่า MPU6050
│
├── 📁 communication_Esp32/     # [ESP32] ศูนย์กลางการสื่อสารและหน้าเว็บ
│   └── จัดการ Web Server, อ่าน GPS/Compass, รับค่า nRF24 จากรีโมท, สั่งงาน Servo
│   └── 💬 สื่อสารกับบอร์ดทรงตัว (STM32) ผ่าน Serial UART
│
└── 📁 Remote_STM32_2/          # [STM32] รีโมทคอนโทรล
    └── อ่านค่า Joystick, แสดงผลจอ OLED (QR Code/IP), ส่งข้อมูลผ่าน nRF24L01


⚙️ สถาปัตยกรรมการสื่อสาร (System Architecture)

ระบบประกอบด้วยไมโครคอนโทรลเลอร์ 3 ตัว ทำงานสอดประสานกัน:

Remote (STM32) ➜ ส่งข้อมูล Joystick ผ่านคลื่นวิทยุ nRF24L01 ➜ ไปยัง ESP32

ESP32 ➜ เป็นตัวกลางรับข้อมูลจาก Remote, ประมวลผล Web UI, ดูแล GPS/Servo ➜ ส่งคำสั่งความเร็ว (Speed/Turn) ผ่านสาย Serial (UART) ➜ ไปยัง บอร์ดสมดุล (STM32)

บอร์ดสมดุล (STM32) ➜ รับคำสั่งความเร็ว ➜ คำนวณ PID รักษาสมดุล ➜ สั่งขับมอเตอร์ ➜ ส่งข้อมูลสถานะ (Telemetry) กลับไปที่ ESP32

🛠️ ฮาร์ดแวร์ที่ใช้ (Hardware Requirements)

1. ส่วนตัวหุ่นยนต์ (Robot Body)

บอร์ด STM32 (Bluepill / Maple Core) x 1

เซ็นเซอร์วัดความเอียง MPU6050 x 1

มอเตอร์ DC พร้อม Encoder (เช่น Nidec 24H) x 2

บอร์ดขับมอเตอร์ (Motor Driver) x 1

2. ส่วนการสื่อสารและลูกเล่น (Comms & Head)

บอร์ด ESP32 x 1

โมดูลวิทยุ nRF24L01 x 1

โมดูล GPS (เช่น NEO-6M / M8N) x 1

โมดูลเข็มทิศ QMC5883L x 1

จอแสดงผล OLED 0.96" (I2C) x 1

Servo Motors x 4 (สำหรับส่วนหัวและหู)

3. ส่วนรีโมท (Remote Controller)

บอร์ด STM32 (Bluepill) x 1

โมดูลวิทยุ nRF24L01 x 1

จอแสดงผล OLED 0.96" (I2C) x 1

โมดูล Joystick Analog x 1

💻 การติดตั้งและคอมไพล์ (Installation & Setup)

โปรเจกต์นี้เขียนด้วย C++ ผ่านเฟรมเวิร์ก Arduino โดยใช้ PlatformIO IDE บน VS Code

Clone Repository นี้ลงเครื่องของคุณ:

git clone [https://github.com/YourUsername/MyBalancingRobot.git](https://github.com/YourUsername/MyBalancingRobot.git)


เปิดโฟลเดอร์ MYBALANCINGROBOT (หรือไฟล์ Project02_STM32.code-workspace) ด้วย VS Code

ตรวจสอบให้แน่ใจว่าได้ติดตั้ง Extension PlatformIO แล้ว

สลับโฟลเดอร์เพื่ออัปโหลดโค้ดทีละบอร์ด:

บอร์ดรีโมท: เปิดโฟลเดอร์ Remote_STM32_2 เสียบสาย ST-Link หรือ USB แล้วกด Upload

บอร์ดทรงตัว: เปิดโฟลเดอร์ Self-balancing_STM32 เสียบสายแล้วกด Upload

บอร์ด ESP32: เปิดโฟลเดอร์ communication_Esp32 ตั้งค่า WiFi SSID/Password ในโค้ด เสียบสายแล้วกด Upload

📚 Libraries ที่ต้องการ (Dependencies)

(PlatformIO จะดาวน์โหลดให้อัตโนมัติ หากตั้งค่า platformio.ini ไว้ถูกต้อง)

RF24 by TMRh20

Adafruit GFX Library & Adafruit SSD1306

qrcode by ricmoo

TinyGPSPlus

QMC5883LCompass

ESP32Servo

🚀 วิธีการใช้งาน (How to use)

📱 การใช้งานผ่าน Web UI

เปิดหุ่นยนต์ (ESP32 จะปล่อย WiFi ชื่อ RobotControl หากต่อเน็ตบ้านไม่ได้ หรือจะต่อกับ Router บ้านก็ได้)

ดู IP Address จากหน้าจอ OLED ของรีโมท (กดปุ่ม Joy 2 ครั้งเพื่อขอ IP สแกน QR Code เข้าเว็บได้เลย)

ในหน้าเว็บสามารถ:

บังคับทิศทางด้วย Virtual Joystick

จิ้มบนแผนที่เพื่อสร้างเส้นทาง (GPS Waypoint) แล้วกด SEND ROUTE

ควบคุมหันหัวซ้าย-ขวา หรือเปิดโหมด Auto Scan

🎮 การใช้งานผ่าน Remote

เปิดหุ่นยนต์ และเปิดรีโมท

รอให้ไฟ LED ที่รีโมทหยุดกระพริบ (แสดงว่าเชื่อมต่อ nRF24L01 สำเร็จ)

โยกจอยสติ๊กเพื่อบังคับหุ่นยนต์เดินหน้า/ถอยหลัง และเลี้ยว

กดปุ่มจอยสติ๊กค้างไว้เพื่อขอพิกัด GPS ปัจจุบันของหุ่นยนต์ (จะขึ้น QR Code สำหรับ Google Maps)

👨‍💻 ผู้พัฒนา (Author)

[ชื่อของคุณ] - Initial work - ลิงก์ GitHub ของคุณ

📜 License

โปรเจกต์นี้จัดทำขึ้นเพื่อการศึกษาและการทดลองส่วนตัว (สามารถปรับแก้ระบุ License เช่น MIT ได้ตามต้องการ)
