/*
 * MPU6050.c
 *
 *  Created on: Apr 9, 2015
 *      Author: Ros9e
 */
#include"MPU6050.h"
#include "Kalman.h"

/************************************************************/

void Setup_MPU6050()
{
writeI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80);
SysCtlDelay(SysCtlClockGet()/600);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x07);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x00);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);
writeI2C(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
SysCtlDelay(SysCtlClockGet()/300);

accYsub = readI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H);
accZsub = readI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H);
accXsub = readI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H);
accY = ((accYsub<<8)|0);
accZ = ((accZsub<<8)|0);
accX = ((accXsub<<8)|0);
accXangle = (atan2(accY,accZ))*RAD_TO_DEG;
accYangle = (atan2(accZ,accX))*RAD_TO_DEG;
x_angle= accXangle;
y_angle= accYangle;
gyroXangle = accXangle;
gyroYangle = accYangle;
}

/*****************************************************************/

void ReadMPU()
{
	accYsub = readI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H);
	accZsub = readI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H);
	accXsub = readI2C(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H);
	gyroXsub= readI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H);
	gyroYsub= readI2C(MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H);
	accY = ((accYsub<<8)|0);
	accZ = ((accZsub<<8)|0);
	accX = ((accXsub<<8)|0);
	gyroX = ((gyroXsub<<8)|0);
	gyroY = ((gyroYsub<<8)|0);
	accXangle = (atan2(accY,accZ))*RAD_TO_DEG;
	accYangle = (atan2(accZ,accX))*RAD_TO_DEG;
    gyroXrate = (double)gyroX/131.0;
    gyroYrate = (double)gyroY/131.0;
    kalAngleX = kalmanCalculateX(accXangle, gyroXrate, 20)+1;
    kalAngleY = kalmanCalculateY(accYangle, gyroYrate, 20)-87;
}
