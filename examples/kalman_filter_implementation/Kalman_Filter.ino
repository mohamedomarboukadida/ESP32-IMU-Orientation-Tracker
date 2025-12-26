#include <Wire.h>

#define MPU6050_ADDR 0x68
#define HMC5883L_ADDR 0x1E

#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_GYRO_XOUT_H  0x43

#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0
#define MAG_SCALE 0.92

#define Q_ANGLE 0.001
#define Q_BIAS 0.003
#define R_MEASURE 0.03

#define DECLINATION_ANGLE 0.0570 

double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0;

double pitch = 0, roll = 0, yaw = 0;
double dt = 0.02;

double anglePitch = 0, biasPitch = 0, ratePitch = 0;
double angleRoll = 0, biasRoll = 0, rateRoll = 0;
double P_pitch[2][2] = {{0, 0}, {0, 0}};
double P_roll[2][2] = {{0, 0}, {0, 0}};

unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  MPU6050_init();
  HMC5883L_init();

  calibrate_MPU6050();
  calibrate_HMC5883L();

  lastTime = millis();
  Serial.println("\n=== Kalman Filter Orientation Tracker Ready ===\n");
}

void loop() {
  double ax, ay, az, gx, gy, gz, mx, my, mz;

  read_MPU6050(ax, ay, az, gx, gy, gz);
  read_HMC5883L(mx, my, mz);

  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  if (dt == 0) dt = 0.001;
  lastTime = currentTime;

  double accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  double accelRoll = atan2(ay, az) * 180.0 / PI;

  pitch = Kalman_filter_pitch(gx, accelPitch);
  roll = Kalman_filter_roll(gy, accelRoll);

  yaw = atan2(my, mx) + DECLINATION_ANGLE;
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  yaw *= 180.0 / PI;

  Serial.print("Pitch: "); Serial.print(pitch, 2); Serial.print("°  ");
  Serial.print("Roll: "); Serial.print(roll, 2); Serial.print("°  ");
  Serial.print("Yaw: "); Serial.print(yaw, 2); Serial.println("°");

  delay(20);
}

double Kalman_filter_pitch(double gyroRate, double accelAngle) {
  ratePitch = gyroRate - biasPitch;
  anglePitch += dt * ratePitch;

  P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_ANGLE);
  P_pitch[0][1] -= dt * P_pitch[1][1];
  P_pitch[1][0] -= dt * P_pitch[1][1];
  P_pitch[1][1] += Q_BIAS * dt;

  double S = P_pitch[0][0] + R_MEASURE;
  double K[2];
  K[0] = P_pitch[0][0] / S;
  K[1] = P_pitch[1][0] / S;

  double y = accelAngle - anglePitch;
  anglePitch += K[0] * y;
  biasPitch += K[1] * y;

  double P00_temp = P_pitch[0][0];
  double P01_temp = P_pitch[0][1];

  P_pitch[0][0] -= K[0] * P00_temp;
  P_pitch[0][1] -= K[0] * P01_temp;
  P_pitch[1][0] -= K[1] * P00_temp;
  P_pitch[1][1] -= K[1] * P01_temp;

  return anglePitch;
}

double Kalman_filter_roll(double gyroRate, double accelAngle) {
  rateRoll = gyroRate - biasRoll;
  angleRoll += dt * rateRoll;

  P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_ANGLE);
  P_roll[0][1] -= dt * P_roll[1][1];
  P_roll[1][0] -= dt * P_roll[1][1];
  P_roll[1][1] += Q_BIAS * dt;

  double S = P_roll[0][0] + R_MEASURE;
  double K[2];
  K[0] = P_roll[0][0] / S;
  K[1] = P_roll[1][0] / S;

  double y = accelAngle - angleRoll;
  angleRoll += K[0] * y;
  biasRoll += K[1] * y;

  double P00_temp = P_roll[0][0];
  double P01_temp = P_roll[0][1];

  P_roll[0][0] -= K[0] * P00_temp;
  P_roll[0][1] -= K[0] * P01_temp;
  P_roll[1][0] -= K[1] * P00_temp;
  P_roll[1][1] -= K[1] * P01_temp;

  return angleRoll;
}

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

  for (int i = 0; i < 100; i++) {
    read_MPU6050(ax, ay, az, gx, gy, gz);
    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;
    gyroXOffset += gx;
    gyroYOffset += gy;
    gyroZOffset += gz;
    delay(10);
  }

  accelOffsetX /= 100;
  accelOffsetY /= 100;
  accelOffsetZ = accelOffsetZ / 100 - 1.0;
  gyroXOffset /= 100;
  gyroYOffset /= 100;
  gyroZOffset /= 100;

  Serial.println("MPU6050 Calibration Complete!\n");
}

void calibrate_HMC5883L() {
  Serial.println("Calibrating HMC5883L...  Rotate sensor in all directions!");
  double mx, my, mz;

  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    read_HMC5883L(mx, my, mz);

    magMinX = min(magMinX, mx);
    magMaxX = max(magMaxX, mx);
    magMinY = min(magMinY, my);
    magMaxY = max(magMaxY, my);
    magMinZ = min(magMinZ, mz);
    magMaxZ = max(magMaxZ, mz);

    if ((millis() - startTime) % 1000 == 0) Serial.print(".");
    delay(100);
  }

  Serial.println("\nHMC5883L Calibration Complete!\n");
}

void read_MPU6050(double &ax, double &ay, double &az, double &gx, double &gy, double &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
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
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);

  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();

  mx = (x - (magMinX + magMaxX) / 2.0) * MAG_SCALE;
  my = (y - (magMinY + magMaxY) / 2.0) * MAG_SCALE;
  mz = (z - (magMinZ + magMaxZ) / 2.0) * MAG_SCALE;
}