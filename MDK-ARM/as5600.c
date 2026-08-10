#include "as5600.h"
#include "stm32f1xx_hal.h"
/**
 * @brief Check sensor AS5600 is connected on I2C
*/
HAL_StatusTypeDef AS5600_Init(I2C_HandleTypeDef *hi2c)
{
    return HAL_I2C_IsDeviceReady(hi2c, AS5600_I2C_ADDR, 3, 100);
}

/**
 * @brief Read raw angle value from AS5600 (0 - 4095)
*/
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2] = {0, 0};
    
    if (HAL_I2C_Mem_Read(hi2c, 0x6C, 0x0C, 1, data, 2, 100) != HAL_OK) {
        __HAL_RCC_I2C1_FORCE_RESET();
        HAL_Delay(1);
        __HAL_RCC_I2C1_RELEASE_RESET();
        
        HAL_I2C_Init(hi2c); 
        return 0;
    }

    return (((uint16_t)(data[0] & 0x0F)) << 8) | data[1];
}

/**
 * @brief Calculation from Raw Angle - Degree
 */
float AS5600_GetAngleDegree(I2C_HandleTypeDef *hi2c) 
{
    uint16_t raw = AS5600_ReadRawAngle(hi2c);
    return ((float)raw / 4096.0f) * 360.0f;
}

/**
 * @brief Calculation from Raw Angle - Radian (0 - 2*PI)
 */
float AS5600_GetAngleRad(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw = AS5600_ReadRawAngle(hi2c);
    return ((float)raw / 4096.0f) * 6.28318530718f;
}