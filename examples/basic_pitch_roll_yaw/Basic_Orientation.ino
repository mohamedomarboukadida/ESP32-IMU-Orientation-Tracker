#include <Wire.h>
#include "config.h"
#include "sensors.h"

double pitch = 0, roll = 0, yaw = 0;
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 100;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("ESP32 Pitch-Roll-Yaw IMU - Basic Orientation Example");
  Serial.println("=====================================================");
  
  Serial.println("\nInitializing IMU...");
  
  MPU6050_init();
  HMC5883L_init();
  
  Serial.println("IMU initialized successfully!");
  
  calibrate_MPU6050();
  calibrate_HMC5883L();
  
  Serial.println("\nStarting orientation readings...");
  Serial.println("Format: Pitch | Roll | Yaw (degrees)");
  Serial.println("=====================================\n");
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentTime;
    
    double ax, ay, az, gx, gy, gz, mx, my, mz;
    
    read_MPU6050(ax, ay, az, gx, gy, gz);
    read_HMC5883L(mx, my, mz);
    
    double accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    double accelRoll = atan2(ay, az) * 180.0 / PI;
    
    double alpha = 0.96;
    pitch = alpha * (pitch + gx * (UPDATE_INTERVAL / 1000.0)) + (1 - alpha) * accelPitch;
    roll = alpha * (roll + gy * (UPDATE_INTERVAL / 1000.0)) + (1 - alpha) * accelRoll;
    
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
  }
}