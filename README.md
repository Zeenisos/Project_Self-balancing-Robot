# 🤖 MyBalancingRobot (หุ่นยนต์สองล้อสมดุลอัจฉริยะ)

![Project Status](https://img.shields.io/badge/Status-Active-success)
![Platform](https://img.shields.io/badge/PlatformIO-VS%20Code-blue)
![MCU](https://img.shields.io/badge/MCU-STM32%20%7C%20ESP32-orange)

โปรเจกต์หุ่นยนต์สองล้อสมดุล (Self-Balancing Robot) ที่มาพร้อมกับระบบควบคุมที่หลากหลาย รองรับทั้งการบังคับผ่าน **รีโมทคอนโทรลแบบสร้างเอง (Custom Remote)** และการสั่งงานผ่าน **Web Application** พร้อมระบบนำทางด้วยพิกัด GPS และแสดงออกทางอารมณ์ผ่านจอ OLED

## ✨ ฟีเจอร์หลัก (Key Features)

* **⚖️ Self-Balancing:** ทรงตัวอัตโนมัติด้วยเซ็นเซอร์ MPU6050 และสมการควบคุมแบบ PID (Full-State Feedback)
* **🌐 Web-based Control:** ควบคุมหุ่นยนต์ผ่านหน้าเว็บ (ESP32 Web Server) พร้อมแผนที่ Leaflet.js
* **📍 GPS Waypoint Navigation:** ระบบนำทางอัตโนมัติไปยังจุดหมายที่จิ้มบนแผนที่ (ใช้ GPS ร่วมกับเข็มทิศ QMC5883L)
* **🎮 Custom RF Remote:** รีโมทคอนโทรลหน้าจอ OLED ส่งสัญญาณผ่าน nRF24L01 (2.4GHz) ระยะไกล
* **👀 Interactive OLED Eyes:** จอแสดงผลใบหน้า/ดวงตาของหุ่นยนต์ที่ตอบสนองตามมุมเอียง (Pitch Angle)
* **🤖 Animatronic Head:** ระบบ Servo คู่สำหรับหมุนหัว (Pan/Tilt) และขยับหู/ตา พร้อมโหมด Auto-Scan ส่ายหัวอัตโนมัติ

## 📂 โครงสร้างโปรเจกต์ (Project Structure)

โปรเจกต์นี้ถูกแบ่งออกเป็น 3 ส่วนหลัก (3 โฟลเดอร์ใน PlatformIO Workspace) ซึ่งทำงานร่วมกันแบบ Multi-MCU:

```text
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
