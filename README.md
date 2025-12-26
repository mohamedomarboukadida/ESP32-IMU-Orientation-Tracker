# ESP32 IMU Orientation Tracker

**Measure 3D orientation angles (pitch, roll, yaw) using ESP32 and MPU6050/HMC5883L sensors with Kalman filter.**

This project reads accelerometer, gyroscope, and magnetometer data to calculate real-time orientation. It uses Kalman filtering for accurate angle estimation and is pre-configured for Sousse, Tunisia with automatic sensor calibration.

---

## Applications
Perfect for projects requiring precise orientation tracking:
- **Robotics** - Self-balancing robots, navigation, robotic arms
- **Drones & RC** - Flight controllers, quadcopter stabilization
- **IoT Devices** - Smart sensors, motion detection systems
- **Camera Gimbals** - Video stabilization, smooth camera movement
- **Educational** - Learn sensor fusion, Kalman filtering, control systems

---

## Features
- Kalman filter for optimal sensor fusion
- Real-time pitch, roll, yaw measurement
- Automatic sensor calibration
- Pre-configured for Sousse, Tunisia (Declination: +3° 16' EAST)
- OLED display support

---

## Hardware

![Sensors Required](docs/Sensor%20Fusion%20(MPU6050%20+%20HMC5883L).jpg)

- ESP32 Development Board
- MPU6050 IMU Sensor
- HMC5883L Magnetometer (optional)

**Wiring:**

![Wiring Diagram](docs/Interfacing%20MPU6050%20+%20HMC5883L%20with%20ESP32.jpg)

| Sensor | ESP32 |
|--------|-------|
| VCC    | 3.3V  |
| GND    | GND   |
| SDA    | GPIO 21 |
| SCL    | GPIO 22 |

---

## Quick Start

1. **Install ESP32 Board Support**
   - Arduino IDE → **File** → **Preferences**
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - **Tools** → **Boards Manager** → Install **ESP32**

2. **Install Libraries**
   - Adafruit MPU6050
   - Adafruit HMC5883
   - Adafruit Unified Sensor

3. **Upload**
   - Open [main.ino](main.ino)
   - Select ESP32 board and port
   - Upload and open Serial Monitor (115200 baud)

---

## Examples
- `examples/basic_pitch_roll_yaw/` - Complementary filter
- `examples/kalman_filter_implementation/` - Kalman filter demo
- `examples/oled_display/` - Display on SSD1306 OLED

---

## Configuration

**Magnetic Declination:** Pre-configured for Sousse, Tunisia (+3° 16' EAST = 0.0570 radians)

*Why it matters:* The magnetometer reads magnetic north, not true north. Declination corrects this difference, which varies by location. For accurate navigation/heading, set your location's declination from [magnetic-declination.com](https://www.magnetic-declination.com/)

⚠️ **Using wrong declination:** Yaw/heading will be offset by the difference (pitch/roll unaffected). Critical for drones, navigation, and compass applications.

To change location, edit `DECLINATION_ANGLE` in [src/config.h](src/config.h)

**Calibration:** System auto-calibrates on startup
- Keep sensor stationary for MPU6050
- Rotate sensor in all directions for HMC5883L

---

## License
See [LICENSE](LICENSE) file for details.
