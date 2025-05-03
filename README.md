# 🛣️ Autonomous Navigation Car with Overhead Camera & ArUco Markers

An Arduino-powered autonomous vehicle that navigates in a controlled arena using an **overhead camera**, **ArUco marker-based localization**, and **path planning algorithms**. The vehicle receives motion instructions via serial/Bluetooth from a Python-based control system running on a host computer or Raspberry Pi.

---

## 🚀 Features

- 📷 Overhead view of entire arena using a single USB camera
- 🧠 Localization of robot and goal using ArUco markers
- 🗺️ Path planning with A* or Dijkstra algorithm
- 📡 Wireless/serial communication from host to Arduino
- ⚙️ Simple and modular Arduino motor control

---

## 🧰 Hardware Requirements

| Component            | Purpose                                |
|---------------------|----------------------------------------|
| Arduino Uno         | Core controller for robot              |
| USB Camera          | Mounted above arena for full visibility|
| Raspberry Pi / PC   | Runs image processing and path planning|
| ArUco Markers       | Placed on robot and goal for tracking  |
| L298N Motor Driver  | Drives the robot's motors              |
| Robot Chassis       | Mobile platform                        |
| HC-05 Bluetooth (opt)| For wireless command transfer         |

---

## 🧑‍💻 Software Requirements

- Python 3.7+
- OpenCV
- NumPy
- PySerial (for communication)

Install dependencies:
```bash
pip install opencv-python numpy pyserial

