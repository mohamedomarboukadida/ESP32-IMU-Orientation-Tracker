#include <Wire.h>
#include "sensors.h"
#include "config.h"

double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0;

static double angle = 0, bias = 0, rate = 0;
static double P[2][2] = {{0, 0}, {0, 0}};

void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void HMC5883L_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void calibrate_MPU6050() {
  Serial.println("Calibrating MPU6050...  Keep sensor stationary!");
  double ax, ay, az, gx, gy, gz;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    read_MPU6050(ax, ay, az, gx, gy, gz);
    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;
    gyroXOffset += gx;
    gyroYOffset += gy;
    gyroZOffset += gz;
    delay(10);
  }

  accelOffsetX /= CALIBRATION_SAMPLES;
  accelOffsetY /= CALIBRATION_SAMPLES;
  accelOffsetZ /= CALIBRATION_SAMPLES;
  gyroXOffset /= CALIBRATION_SAMPLES;
  gyroYOffset /= CALIBRATION_SAMPLES;
  gyroZOffset /= CALIBRATION_SAMPLES;

  accelOffsetZ -= 1.0;

  Serial.println("MPU6050 Calibration Complete!\n");
}

void calibrate_HMC5883L() {
  Serial.println("Calibrating HMC5883L...  Rotate sensor in all directions!");
  double mx, my, mz;

  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;

  unsigned long startTime = millis();
  int dotCount = 0;
  
  while (millis() - startTime < MAG_CALIBRATION_TIME) {
    read_HMC5883L(mx, my, mz);

    if (mx < magMinX) magMinX = mx;
    if (mx > magMaxX) magMaxX = mx;
    if (my < magMinY) magMinY = my;
    if (my > magMaxY) magMaxY = my;
    if (mz < magMinZ) magMinZ = mz;
    if (mz > magMaxZ) magMaxZ = mz;

    if ((millis() - startTime) / 1000 > dotCount) {
      Serial.print(".");
      dotCount++;
    }
    
    delay(100);
  }

  Serial.println("\nHMC5883L Calibration Complete!\n");
}

void read_MPU6050(double &ax, double &ay, double &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  ax = rawX / ACCEL_SCALE - accelOffsetX;
  ay = rawY / ACCEL_SCALE - accelOffsetY;
  az = rawZ / ACCEL_SCALE - accelOffsetZ;
}

void read_MPU6050(double &ax, double &ay, double &az, double &gx, double &gy, double &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();
  
  Wire.read();
  Wire.read();
  
  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  ax = rawAX / ACCEL_SCALE - accelOffsetX;
  ay = rawAY / ACCEL_SCALE - accelOffsetY;
  az = rawAZ / ACCEL_SCALE - accelOffsetZ;
  
  gx = rawGX / GYRO_SCALE - gyroXOffset;
  gy = rawGY / GYRO_SCALE - gyroYOffset;
  gz = rawGZ / GYRO_SCALE - gyroZOffset;
}

void read_HMC5883L(double &mx, double &my, double &mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Starting register for magnetometer
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);

  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();

  mx = (x - (magMinX + magMaxX) / 2.0) * MAG_SCALE;
  my = (y - (magMinY + magMaxY) / 2.0) * MAG_SCALE;
  mz = (z - (magMinZ + magMaxZ) / 2.0) * MAG_SCALE;
}

double Kalman_filter(double angleInput, double gyroRate, double accelAngle, double dt) {
  rate = gyroRate - bias;
  angleInput += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_ANGLE);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_BIAS * dt;

  double S = P[0][0] + R_MEASURE;
  double K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  double y = accelAngle - angleInput;
  angleInput += K[0] * y;
  bias += K[1] * y;

  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angleInput;
}