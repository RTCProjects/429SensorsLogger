#ifndef SKIF_H_
#define SKIF_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"

#define RAD_TO_DEG 180./M_PI
#define DEG_TO_RAD M_PI/180.
#define A_RES 2.0/32768.0
#define G_RES 250.0/32768.0
#define M_RES 1.501831501
#define GYRO_SENS 130
#define Pi 3.14159265359

#define IMU_LOW_DATA_SIZE 200

int16_t c_ax, c_ay, c_az, c_gx, c_gy, c_gz;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
int64_t b_ax, b_ay, b_az, b_gx, b_gy, b_gz;
float yaw, pitch, roll;

typedef struct
{
	float fAz;
	float fPitch;
	float fRoll;
}tIMUData;

void 	IMU_Init(void);
void 	IMU_Calcualte(void);
void	*IMU_GetSkifCurrentData(void);

#endif
