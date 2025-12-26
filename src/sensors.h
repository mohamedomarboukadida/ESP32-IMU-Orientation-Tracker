#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"

extern double accelOffsetX, accelOffsetY, accelOffsetZ;
extern double gyroXOffset, gyroYOffset, gyroZOffset;
extern double magMinX, magMaxX, magMinY, magMaxY, magMinZ, magMaxZ;

void MPU6050_init();
void HMC5883L_init();
void calibrate_MPU6050();
void calibrate_HMC5883L();
void read_MPU6050(double &ax, double &ay, double &az);
void read_MPU6050(double &ax, double &ay, double &az, double &gx, double &gy, double &gz);
void read_HMC5883L(double &mx, double &my, double &mz);
double Kalman_filter(double angle, double gyroRate, double accelAngle, double dt);

#endif