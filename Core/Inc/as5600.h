#ifndef INC_AS5600_H
#define INC_AS5600_H

#include "stm32f1xx_hal.h"
#include "main.h"
/* dịch trái 1 bit vì as5600 gồm 7 địa chỉ, 1 địa chỉ cuối R/W 
mà DevAddress (được thiết kế 8-bit căn lề trái) nên cần tham số là địa chỉ đã dịch đi 1 bit
*/ 
#define AS5600_I2C_ADDR     (0x36 << 1)

#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_REG_RAW_ANGLE_L 0x0D

/*******************************************************************************
 * Functions
 ******************************************************************************/

HAL_StatusTypeDef AS5600_Init(I2C_HandleTypeDef *hi2c);
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c);
float AS5600_GetAngleDegree(I2C_HandleTypeDef *hi2c);
float AS5600_GetAngleRad(I2C_HandleTypeDef *hi2c);
#endif