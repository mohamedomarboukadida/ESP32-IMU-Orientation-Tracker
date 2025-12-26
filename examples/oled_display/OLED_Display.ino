#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

float alpha = 0.98;
float roll = 0, pitch = 0, yaw = 0;

unsigned long previousTime = 0;

#define DECLINATION_ANGLE 0.0570

void setup() {
  Serial.begin(115200);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Initializing..."));
  display.display();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("MPU6050 Error!"));
    display.display();
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  if(!mag.begin()) {
    Serial.println("Failed to find HMC5883L chip");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("HMC5883L Error!"));
    display.display();
    while(1) {
      delay(10);
    }
  }
  
  Serial.println("Calibrating sensors...");
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Calibrating..."));
  display.display();
  
  calibrateSensors();
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Ready!"));
  display.display();
  delay(1000);
  
  previousTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  sensor_event_t magEvent;
  mag.getEvent(&magEvent);
  
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  
  float ax = a.acceleration.x - accelOffsetX;
  float ay = a.acceleration.y - accelOffsetY;
  float az = a.acceleration.z - accelOffsetZ;
  
  float gx = g.gyro.x - gyroOffsetX;
  float gy = g.gyro.y - gyroOffsetY;
  float gz = g.gyro.z - gyroOffsetZ;
  
  float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accelRoll = atan2(-ax, az) * 180.0 / PI;
  
  pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accelPitch;
  roll = alpha * (roll + gx * dt) + (1 - alpha) * accelRoll;
  
  float mx = magEvent.magnetic.x;
  float my = magEvent.magnetic.y;
  float mz = magEvent.magnetic.z;
  
  float rollRad = roll * PI / 180.0;
  float pitchRad = pitch * PI / 180.0;
  
  float magX = mx * cos(pitchRad) + mz * sin(pitchRad);
  float magY = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);
  
  yaw = atan2(-magY, magX) * 180.0 / PI;
  
  yaw += DECLINATION_ANGLE * 180.0 / PI;
  
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  
  display.print(F("Roll:  "));
  display.print(roll, 1);
  display.println(F(" deg"));
  
  display.print(F("Pitch: "));
  display.print(pitch, 1);
  display.println(F(" deg"));
  
  display.print(F("Yaw:   "));
  display.print(yaw, 1);
  display.println(F(" deg"));
  
  display.println();
  display.print(F("Temp:  "));
  display.print(temp.temperature, 1);
  display.println(F(" C"));
  
  display.display();
  
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);
  
  delay(50);
}

void calibrateSensors() {
  const int numSamples = 1000;
  float axSum = 0, aySum = 0, azSum = 0;
  float gxSum = 0, gySum = 0, gzSum = 0;
  
  for(int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    axSum += a.acceleration.x;
    aySum += a.acceleration.y;
    azSum += a.acceleration.z;
    
    gxSum += g.gyro.x;
    gySum += g.gyro.y;
    gzSum += g.gyro.z;
    
    delay(3);
  }
  
  accelOffsetX = axSum / numSamples;
  accelOffsetY = aySum / numSamples;
  accelOffsetZ = (azSum / numSamples) - 9.81;
  
  gyroOffsetX = gxSum / numSamples;
  gyroOffsetY = gySum / numSamples;
  gyroOffsetZ = gzSum / numSamples;
  
  Serial.println("Calibration complete!");
  Serial.print("Accel offsets: ");
  Serial.print(accelOffsetX); Serial.print(", ");
  Serial.print(accelOffsetY); Serial.print(", ");
  Serial.println(accelOffsetZ);
  Serial.print("Gyro offsets: ");
  Serial.print(gyroOffsetX); Serial.print(", ");
  Serial.print(gyroOffsetY); Serial.print(", ");
  Serial.println(gyroOffsetZ);
}
