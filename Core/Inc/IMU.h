/*
 * IMU.h
 *
 *  Created on: 31 мая 2018 г.
 *      Author: denisdenk
 */

#ifndef IMU_H_
#define IMU_H_

#include "MadgwickAHRS.h"
#include "RegisterMap.h"
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_iic.h"
#include "cmsis_os.h"

#define RAD_TO_DEG 180./M_PI
#define DEG_TO_RAD M_PI/180.
#define A_RES 2.0/32768.0
#define G_RES 250.0/32768.0
#define M_RES 1.501831501
#define GYRO_SENS 130
#define Pi 3.14159265359

#define IMU_QUERY_SIZE	2

int16_t c_ax, c_ay, c_az, c_gx, c_gy, c_gz;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
int64_t b_ax, b_ay, b_az, b_gx, b_gy, b_gz;
float yaw, pitch, roll;
char TransmitData[100];

typedef struct
{
	float fAz;
	float fPitch;
	float fRoll;
}tIMUData;

typedef struct
{
	float	fAccel[3];
	float	fGyro[3];
}tIMULowData;

// Function Prototypes
void IMU_Init(void);
void IMU_GetData(void);

#endif /* IMU_H_ */