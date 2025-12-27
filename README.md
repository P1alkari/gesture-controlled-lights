# Hand Tracking LED Control with Arduino & Python

This project demonstrates a **real-time computer vision to hardware pipeline** where hand gestures detected using a webcam control LEDs on an Arduino via serial communication.

The goal of the project is to showcase skills in **Python, computer vision, serial communication, and embedded systems**, while keeping the system modular and easy to extend.

---

## 🚀 Features

* Real-time hand tracking using **MediaPipe**
* Robust finger-count detection with temporal smoothing
* Serial communication between Python and Arduino
* LED output mapped directly to finger count
* Clean and beginner-friendly project structure

---

## 🧠 How It Works

1. MediaPipe tracks one hand from the webcam
2. Finger states are detected and smoothed over time
3. The finger count is sent as a single character (`'0'–'5'`) via serial
4. Arduino reads the value and turns on the first **N LEDs**

---

## 🧰 Hardware Requirements

* Arduino Uno (or compatible)
* 5 LEDs
* 5 resistors (220–330Ω)
* Breadboard
* Jumper wires
* USB cable
* PC with webcam

---

## 🖥️ Software Requirements

### Python

* Python 3.9+
* OpenCV
* MediaPipe
* PySerial

Install dependencies:

```bash
pip install opencv-python mediapipe pyserial
```

### Arduino

* Arduino IDE

---

## ⚙️ Setup

### 1️⃣ Arduino

1. Open `arduino_leds.ino` in **Arduino IDE**
2. Connect the LEDs to the defined digital pins
3. Select correct **Board** and **COM port**
4. Upload the sketch

### 2️⃣ Python

1. Open `hand_tracking.py`
2. Set the correct serial port (e.g. `COM3` or `/dev/ttyUSB0`)
3. Run the script:

```bash
python hand_tracking.py
```

---

## 📁 Project Structure

```text
hand-tracking-led-control/
│── hand_tracking.py      # Python hand tracking + serial output
│── arduino_leds.ino      # Arduino LED control
│── README.md             # Project documentation
```

---

## 🎯 Learning Outcomes

* Computer vision with MediaPipe
* Gesture-based control systems
* Python ↔ Arduino serial communication
* Embedded systems basics
* Writing clean technical documentation

---

## 🔮 Possible Improvements

* PWM LED brightness control
* Gesture-based modes (e.g. fist = off)
* OLED display feedback
* Wireless communication (Bluetooth / ESP32)
* FPGA-based gesture decoding (future work)

---

## 📸 Demo
🎥 Short demo video showing real-time finger gesture control of LEDs:
 https://drive.google.com/file/d/1LWPpYbhF0TckNv7tTDFLE-7RGKTXTAIL/view?usp=sharing
  

---

## 📄 License

MIT License

---

⭐ If you find this project interesting, feel free to star the repository!
