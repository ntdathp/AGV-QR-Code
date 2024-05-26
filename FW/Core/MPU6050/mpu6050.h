#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "i2c.h"

// MPU6050 structure
typedef struct
{
    int16_t Gyro_Z_RAW;
    double Gz;
    double pre_Gz;

    double fil_Gz;
	double pf_Gz;

    double yaw;
} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Yaw(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

