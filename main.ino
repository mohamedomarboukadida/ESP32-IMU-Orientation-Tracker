#include <Wire.h>
#include "src/config.h"
#include "src/sensors.h"

double pitch = 0, roll = 0, yaw = 0;
double anglePitch = 0, angleRoll = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("\n=== ESP32 IMU Orientation Tracker ===");
  Serial.println("Location: Sousse, Tunisia");
  Serial.println("Declination: +3° 16' EAST\n");
  
  MPU6050_init();
  HMC5883L_init();
  
  calibrate_MPU6050();
  calibrate_HMC5883L();
  
  lastTime = millis();
  Serial.println("=== System Ready ===\n");
}

void loop() {
  double ax, ay, az, gx, gy, gz, mx, my, mz;
  
  read_MPU6050(ax, ay, az, gx, gy, gz);
  read_HMC5883L(mx, my, mz);
  
  unsigned long currentTime = millis();
  double dt = (currentTime - lastTime) / 1000.0;
  if (dt == 0) dt = 0.001;
  lastTime = currentTime;
  
  double accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  double accelRoll = atan2(ay, az) * 180.0 / PI;
  
  pitch = Kalman_filter(anglePitch, gx, accelPitch, dt);
  roll = Kalman_filter(angleRoll, gy, accelRoll, dt);
  anglePitch = pitch;
  angleRoll = roll;
  
  yaw = atan2(my, mx) + DECLINATION_ANGLE;
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  yaw *= 180.0 / PI;
  
  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.print("° | Roll: ");
  Serial.print(roll, 2);
  Serial.print("° | Yaw: ");
  Serial.print(yaw, 2);
  Serial.println("°");
  
  delay(LOOP_DELAY_MS);
}
