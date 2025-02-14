# 🤖 PSO-Based WiFi-Controlled Dobot Robotic Arm 🤖

This project uses **Particle Swarm Optimization (PSO)** for **inverse kinematics** to remotely control a **Dobot robotic arm** with **WiFi (ESP32)**. The system reads coordinates from a **camera**, optimizes joint angles with **PSO**, and sends them wirelessly to the **ESP32-controlled Dobot**.

## 🚀 Features
- 🎯 **PSO-Based Inverse Kinematics**: Determines optimal joint angles for precise positioning.
- 📡 **ESP32 WiFi Communication**: Communicates optimized joint angles to the robotic arm over WiFi.
- 🤖 **Dobot Integration**: Dynamically controls the Dobot robotic arm according to targets detected.
- 🔧 **Configurable Dimensions**: Users need to replace the hardcoded dimensions of the Dobot in the code with their own setup dimensions.
- 🛠 **Adjustable DH Parameters**: The Denavit-Hartenberg (DH) parameters need to be adjusted based on your own robotic arm for precise movement.

## 🛠️ Installation
### 1️⃣ Clone the Repository
```bash
git clone https://github.com/Aarya2004-ai/PSO_dobot_arm_control.git
cd PSO_dobot_arm_control
```

### 2️⃣ Create a Virtual Environment (Recommended)
```bash
python -m venv venv
source venv/bin/activate  # On Windows use `venv\Scripts\activate`
```

### 3️⃣ Install Dependencies
```bash
pip install -r requirements.txt
```

### 4️⃣ Configure ESP32 Firmware
1. Open `dobot_wifi.ino` in the **Arduino IDE**.
2. Configure your **WiFi credentials**.
3. Upload the firmware to your ESP32.

### 5️⃣ Run the PSO Optimization Script
```bash
python pso_wifi.py
```

## ⚡ How It Works
1. The **user manually enters the target coordinates** in the script.
2. The **PSO algorithm** computes the optimal joint angles.
3. The angles are **transmitted over WiFi** to the ESP32-controlled Dobot.
4. The **Dobot dynamically moves to the target position**.

## 🤝 Contributing
We appreciate contributions! Please submit pull requests.

## 📜 License
This project is under the MIT License.

---

## 🔮 Future Enhancements
- 🚧 Implement **collision detection** to prevent obstacles.
- 📡 Add **real-time feedback from the Dobot**.
- ⚡ Enhance **optimization speed** to enable quicker movement.

🤖 **This project closes the gap between AI-based optimization and actual robotic control!** 🚀🤖

