# 🚗 Self-Driving Car  
### Vision-Based Obstacle Detection and Lane-Aware Autonomous Driving System

---

## 📌 Project Overview

This project is an autonomous driving system implemented on a Raspberry Pi-based platform using computer vision and embedded control.

It combines:
- Lane detection for road following
- YOLO-based object detection for real-time environment understanding
- Rule-based decision making for obstacle avoidance
- Arduino-based motor control system

The system enables a small autonomous car to perceive its environment and make driving decisions in real time.

---

## 🎯 Objectives

- Detect and follow lane markings using computer vision
- Detect road objects using YOLO (traffic lights, stop signs, obstacles, etc.)
- Control vehicle steering and speed based on lane position
- Implement basic obstacle avoidance using rule-based logic
- Interface Raspberry Pi with Arduino for motor control

---

## 🧠 System Architecture


Camera (Raspberry Pi)
↓
Perception Layer
├── Lane Detection (OpenCV)
├── Object Detection (YOLO)
↓
Decision Layer (Rule-Based Logic)
↓
Control Signals
↓
Arduino Motor Controller
↓
DC Motors (Car Movement)


---

## 🧩 Features

### 🚦 Lane Detection
- Uses OpenCV (thresholding + Hough Transform)
- Calculates lane center
- Applies proportional control (P-controller) for steering

### 🧠 Object Detection
- YOLO model (ONNX format)
- Detects:
  - Toy cars
  - LEGO persons
  - Traffic lights
  - Stop signs
- Real-time bounding box visualization

### ⚙️ Control System
- Rule-based decision making:
  - Stop for red light / stop sign
  - Slow down or avoid obstacles
  - Follow lane otherwise

### 🔌 Hardware Integration
- Raspberry Pi handles vision + decision making
- Arduino handles motor PWM control
- Serial communication between both systems

---

## 🛠️ Hardware Used

- Raspberry Pi 5
- Arduino UNO
- Pi Camera Module
- DC Motors + Motor Driver (L298N)
- Ultrasonic Sensor (for obstacle distance - future integration)

---

## 💻 Software Stack

- Python 3
- OpenCV
- Ultralytics YOLO (ONNX inference)
- NumPy
- PySerial
- Arduino IDE (C++)

---

## 📁 Project Structure


self-driving-car/
│
├── src/
│ ├── main.py # Integrated system (lane + YOLO + control)
│ ├── lane_detection.py
│ ├── object_detection.py
│
├── arduino/
│ └── motor_control.ino # Arduino motor driver code
│
├── models/
│ └── best.onnx # YOLO trained model
│
├── docs/
│ ├── seminar1.pptx
│ ├── seminar2.pptx
│ └── report.pdf
│
├── demos/
│ ├── lane_demo.mp4
│ ├── yolo_demo.mp4
│
├── assets/
│ └── images/ # Screenshots for documentation
│
└── README.md


---

## ▶️ How to Run

### 1. Install dependencies
```bash
pip install ultralytics opencv-python numpy pyserial picamera2
2. Run object detection
python3 src/object_detection.py
3. Run lane detection
python3 src/lane_detection.py
4. Run full system (future)
python3 src/main.py
🔌 Arduino Communication Format

Raspberry Pi sends motor commands:

L<left_speed>R<right_speed>
Example: L120R140
📊 Current Status
✅ Lane detection working
✅ YOLO object detection working
✅ Serial communication with Arduino working
🔄 Full system integration in progress
🔜 Obstacle avoidance (ultrasonic sensor integration)
🚧 Future Improvements
Deep learning-based lane segmentation (instead of thresholding)
Reinforcement learning-based decision making
Sensor fusion (camera + ultrasonic + IMU)
Fully autonomous navigation on real roads
Speed control based on distance estimation
👨‍💻 Authors
Graduation Project Team
Esraa Bassiouny
Sara Islam
Amr Ibrahim
Ziad Yasser
Ziad Khaled
📍 Faculty of Computer and Information Science  
🎓 Graduation Project 2026
📜 License
This project is for academic purposes.