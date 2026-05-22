🤖 MyBalancingRobot (Smart Two-Wheeled Balancing Robot)

A self-balancing robot project featuring versatile control systems. It supports a Custom RF Remote Control, a Web Application, and an AI Copilot complete with GPS waypoint autonomous navigation, obstacle avoidance, and expressive OLED eyes.
[![วิดีโอสาธิตการทำงานของหุ่นยนต์](https://img.youtube.com/vi/ypHjd6-KG1s/0.jpg)](https://youtu.be/ypHjd6-KG1s?si=i2dr2yRV-hSurY5u)
✨ Key Features

⚖️ Self-Balancing: Auto-stabilization using an MPU6050 sensor and LQR/PID control loop (Full-State Feedback).

🌐 Web-based Control: Control the robot via a web interface (ESP32 Web Server) integrated with Leaflet.js maps.

📍 GPS Waypoint Navigation: Autonomous navigation to points clicked on the map (utilizing GPS and QMC5883L Compass).

🛑 Obstacle Avoidance: Integrated Ultrasonic sensor for collision prevention and emergency reversing.

🤖 AI Copilot & Voice Control: Voice-activated commands (Thai language supported) and an "AI Auto" autonomous exploration mode.

🎮 Custom RF Remote: Handheld controller with an OLED display, transmitting commands via a long-range nRF24L01 (2.4GHz) module.

👀 Interactive OLED Eyes: Facial expression displays on the robot that react dynamically to its current pitch angle and running state.

🦾 Dynamic Tilt System: Dual-servo system for head tilt movements and center-of-gravity (CG) compensation.

📂 Project Structure

This project is divided into 3 main parts (3 folders within the PlatformIO Workspace) that work together in a Multi-MCU ecosystem:

MYBALANCINGROBOT/
│
├── 📁 Self-balancing_STM32/    # [STM32] The Core Balancing System
│   └── Motor control, Encoder reading, PID calculation, MPU6050 processing
│
├── 📁 communication_Esp32/     # [ESP32] Communications & Web Hub
│   └── Web Server hosting, GPS/Compass parsing, Ultrasonic, nRF24 receiver, Servo driving, BLE Server
│   └── 💬 Communicates with the balancing board (STM32) via Serial UART
│
└── 📁 Remote_STM32_2/          # [STM32] The Custom Remote
    └── Analog Joystick reading, OLED display (QR Code/IP), nRF24L01 transmission


⚙️ System Architecture

The system consists of 3 microcontrollers working in harmony:

Remote (STM32) ➜ Sends Joystick data via nRF24L01 radio waves ➜ to the ESP32.

ESP32 ➜ Acts as the central hub receiving remote data, reading Ultrasonic/GPS/Compass, processing the Web UI & AI Copilot ➜ Sends speed/turn commands via Serial (UART) ➜ to the Balancing Board (STM32).

Balancing Board (STM32) ➜ Receives speed commands ➜ Calculates LQR/PID to maintain balance ➜ Drives motors ➜ Sends telemetry data back to the ESP32.

🛠️ Hardware Requirements

1. Robot Body (Balancing Core)

STM32 Board (Bluepill / Maple Core) x 1

MPU6050 IMU Sensor x 1

DC Motors with Encoders (e.g., Nidec 24H) x 2

Motor Driver Board x 1

2. Communications & Head System

ESP32 Board x 1

nRF24L01 Wireless Module x 1

GPS Module (e.g., NEO-6M / M8N) x 1

QMC5883L Compass Module x 1

Ultrasonic Sensor (HC-SR04) x 1

0.96" OLED Display (I2C) x 1

Servo Motors x 2 (For Tilt and CG compensation)

3. Remote Controller

STM32 Board (Bluepill) x 1

nRF24L01 Wireless Module x 1

0.96" OLED Display (I2C) x 1

Analog Joystick Module x 1

💻 Installation & Setup

This project is written in C++ using the Arduino Framework via PlatformIO IDE on VS Code.

Clone this repository to your local machine:

git clone [https://github.com/YourUsername/MyBalancingRobot.git](https://github.com/YourUsername/MyBalancingRobot.git)


Open the MYBALANCINGROBOT folder using VS Code.

Ensure you have the PlatformIO extension installed.

Switch between folders to upload the code to each specific board:

Remote Board: Open Remote_STM32_2, connect via ST-Link or USB, and click Upload.

Balancing Board: Open Self-balancing_STM32, connect, and click Upload.

ESP32 Board: Open communication_Esp32, configure your WiFi SSID/Password in the code, connect, and click Upload.

📚 Dependencies (Libraries)

(PlatformIO will download these automatically if platformio.ini is configured correctly)

RF24 by TMRh20

Adafruit GFX Library & Adafruit SSD1306

qrcode by ricmoo

TinyGPSPlus

QMC5883LCompass

ESP32Servo

BLEDevice (ESP32 Built-in)

🚀 How to Use

📱 Using the Web UI & AI Copilot

Turn on the robot (The ESP32 will broadcast a WiFi AP named RobotControl if it cannot connect to your home network).

Check the IP Address on the remote's OLED screen (Double-click the joystick button to request the IP and scan the QR Code to open the web app).

On the Web UI, you can:

Steer the robot using the Virtual Joystick.

Check real-time telemetry (Pitch, Speed, Heading, Obstacle Distance).

Tap on the map to set GPS waypoints, then click SEND ROUTE for autonomous navigation.

Use AI Copilot: Click the 🎤 Mic button to give voice commands (e.g., "เดินหน้า", "สำรวจ", "หยุด").

Activate AI Auto Mode: Let the robot roam freely and avoid obstacles on its own.

🎮 Using the Custom Remote

Turn on the robot, then turn on the remote.

Wait for the remote's LED to stop blinking (indicating a successful nRF24L01 connection).

Move the joystick to drive the robot forward/backward and turn.

Press and hold the joystick button to request the robot's current GPS coordinates (A QR Code linking to Google Maps will appear).

👨‍💻 Author

Aisoon GitHub: https://github.com/Zeenisos

📜 License

This project is created for educational and personal experimentation purposes.
