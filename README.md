# ESP8266 LED Matrix Liquid Simulation

This project implements a **particle-based liquid simulation** on an **8×8 WS2812B LED matrix**, controlled by an **ESP8266** microcontroller.  
An **MPU6050 accelerometer + gyroscope** provides tilt input, so gravity in the simulation responds to how the board is held or rotated in space.  

The simulation is inspired by basic fluid/particle dynamics: multiple "particles" bounce around the grid, collide, and shift under gravity, creating the illusion of liquid flowing across the LED matrix.

![Demo_liquid_sim](https://github.com/user-attachments/assets/7d545c9a-4743-4de2-bf0d-7340781c30cc)

---

## ✨ Features
- **Interactive tilt control**: The MPU6050 continuously measures orientation. Tilt left/right/forward/backward to change the gravity vector.  
- **Particle physics**:
  - Velocity and position updates every frame
  - Edge collision with bounce
  - Particle–particle repulsion to avoid overlap (incompressibility effect)
- **Real-time rendering** at ~30 Hz using the FastLED library.
- **Custom LED mapping**: Supports both serpentine and rectangular XY mapping for flexible matrix layouts.

---

## 🛠️ Hardware Requirements
- **ESP8266** development board (tested on NodeMCU / Wemos D1 Mini)  
- **8×8 WS2812B RGB LED matrix**  
- **MPU6050 accelerometer/gyroscope module**  
- **LiPo battery + LiPo Rider (or USB power)**  

Optional: any other WS2812B-compatible LED grid can be adapted by changing `WIDTH` and `HEIGHT` in the code.  

---

## 📦 Software Requirements
- **Arduino IDE** (or PlatformIO) with ESP8266 board support installed.  
- Libraries required:  
  - [`Adafruit_MPU6050`](https://github.com/adafruit/Adafruit_MPU6050)  
  - [`Adafruit_Sensor`](https://github.com/adafruit/Adafruit_Sensor)  
  - [`FastLED`](https://github.com/FastLED/FastLED)  
  - Custom `"fl/leds.h"` and `"fl/xymap.h"` mapping utilities  

---

## ⚙️ How It Works
1. **Initialization**:  
   - MPU6050 is set up for acceleration/gyro readings.  
   - 30 particles are randomly placed on the grid with zero velocity.  

2. **Physics loop (30 Hz)**:  
   - Read accelerometer data, compute **pitch/roll**, and derive a 2D gravity vector.  
   - Update particle velocities and positions using basic Newtonian motion.  
   - Constrain particles inside grid bounds with edge bounce.  
   - Resolve inter-particle collisions by pushing them apart and exchanging velocity along the collision normal.  

3. **Rendering**:  
   - Each particle is mapped to its nearest LED cell.  
   - LED color is currently fixed (`CHSV(100, 255, 100)` → green).  
   - Frame is pushed to the matrix with `FastLED.show()`.  

The result is an animated blob of particles that **sloshes around** like water whenever you tilt the ESP8266.  

---

## 🚀 Getting Started

### 1. Install dependencies
In Arduino IDE, install the required libraries via **Library Manager**:  
- Adafruit MPU6050  
- Adafruit Unified Sensor  
- FastLED  

Make sure ESP8266 board definitions are installed via **Boards Manager**.

### 2. Flash the code
- Open the `.ino` file in Arduino IDE.  
- Select your ESP8266 board type and COM port.  
- Upload the sketch.  

### 3. Wire the hardware
- **LED matrix DIN** → ESP8266 pin D4 (GPIO2 in the code)  
- **MPU6050 SDA/SCL** → ESP8266 I²C pins (D2 = SDA, D1 = SCL on NodeMCU by default)  
- **Power**: provide 5 V to the LED matrix and 3.3 V to the MPU6050 (via regulator or LiPo Rider).  

### 4. Run it
- Open the serial monitor at 115200 baud to check sensor initialization.  
- Tilt the board — particles should “fall” in that direction on the LED matrix.  

---

## 🎨 Customization
- **Particle count**: adjust `Num_of_particles` in the code (note: too many will slow the ESP8266).  
- **LED mapping**: switch between `constructSerpentine` and `constructRectangularGrid` depending on your LED wiring.  
- **Color scheme**: change `CHSV(100, 255, 100)` to another HSV or RGB color.  
- **Physics parameters**:
  - Gravity strength is tied to accelerometer values.  
  - Collision softness is controlled by `minDist` and overlap factor.  

---

## 📜 License
This project is released under the **MIT License**. See [LICENSE](LICENSE) for details.  

---

## 🤝 Contributions
Pull requests, bug reports, and feature ideas are welcome. Examples of contributions:  
- Adding splash color effects  
- Optimizing performance for larger matrices  
- Porting to ESP32 or other microcontrollers  

---
