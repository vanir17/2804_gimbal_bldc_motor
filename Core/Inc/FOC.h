#ifndef FOC_H
#define FOC_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>


#define VOLTAGE_POWER 12.0f
#define VOLTAGE_LIMIT 6.0f
#define PWM_ARR_PERIOD 1799
#define POLE_PAIRS 7

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern I2C_HandleTypeDef hi2c1;

typedef struct 
{
    I2C_HandleTypeDef *hi2c;

	uint8_t pole_pairs;
	float voltage_power;
	float voltage_limit;
	
	float zero_offset_angle;
	float rotation_direction;
	
	float angle_as5600;
	float angle_electrical;
	
	float duty_a;
	float duty_b;
	float duty_c;	
}FOC_Controller_t;

/*******************************************************************************
 * Functions' Prototypes
 ******************************************************************************/

/**
  * @brief  FOC parameters initial
  */
void FOC_init(FOC_Controller_t *foc, uint8_t pole_pairs, float v_dc, float v_limit);
/**
  * @brief  Update electrical angle from AS5600 angle
  */
void FOC_update_electrical_angle(FOC_Controller_t *foc, float mech_angle_rad);
/**
  * @brief  Clarke, Park Transformation and align PWM
  */
void FOC_step(FOC_Controller_t *foc, float Vq, float Vd);
/**
  * @brief  Zero offset
  */
void FOC_align_sensor(FOC_Controller_t *foc);
 /**
  * @brief  Normalize angle in range [0, 2*PI).
  */
float normalize_angle(float angle);
#endif