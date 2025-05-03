# 🤖 Autonomous Navigation Car using Overhead Camera & ArUco Markers

A vision-based autonomous robot system that uses **overhead camera tracking**, **ArUco marker localization**, and **path planning algorithms** to navigate in a predefined arena. All the logic is handled by a single Python script (`Ardinuo.py`), making it simple and easy to deploy.

---

## 📂 Project Structure

├── ARD_code/ # Arduino motor driver code (if any)
├── ESP_code/ # ESP-related communication code (optional)
├── Ardinuo.py # 🧠 Main control script: detection + planning + communication
├── path_plan.ipynb # Notebook for standalone path planning experiments
├── plot_point.py # Utility to visualize or debug points
├── arena_image1.jpg # Arena reference image
├── README.md # This file


---

## ⚙️ Features

- 📸 Overhead camera tracks real-time robot and goal positions via ArUco markers
- 🔁 All-in-one Python script (`Ardinuo.py`) handles:
  - Marker detection
  - Robot and goal localization
  - Grid creation and path planning
  - Command encoding and transmission to robot
- 🧭 Path planning using A* or custom algorithms
- 🔌 Communicates with robot over serial/Bluetooth

---

## 🛠 Requirements

- Python 3.7+
- OpenCV (`cv2`)
- NumPy
- PySerial
- ArUco markers (printed and placed on robot/arena)

Install dependencies:
```bash
pip install opencv-python numpy pyserial

🚗 How to Run
Connect your Arduino via USB or Bluetooth.

Ensure the camera is mounted overhead with a clear view of the arena.

Launch the main script:

bash
Copy
Edit
python Ardinuo.py
The robot will localize itself and the goal, plan the path, and start moving!

🧠 Logic Overview
cv2.aruco detects markers on robot and target

Arena is divided into grid cells based on marker size

Shortest path is computed

Commands are sent to Arduino to follow the path

🧪 Extras
path_plan.ipynb: Visualizes and tests different path planning strategies

plot_point.py: Plots coordinates and marker points for debugging

arena_image1.jpg: Reference arena image used for mapping
