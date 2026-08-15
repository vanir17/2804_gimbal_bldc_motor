#ifndef IMP_CTRL_H
#define IMP_CTRL_H

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
	float K;
	float B;
	float target_pos;
	float target_vel;
	
	float prev_pos;
	float vel_filtered;
	float filter_alpha;
	
	float voltage_limit;
	float prev_time;
}Impedance_Controller_t;

typedef struct {
    float raw_prev;
    int32_t full_rotations;
    float continuous_angle; 
} Angle_Unwrapper_t;
	

/*******************************************************************************
 * Functions' Prototypes
 ******************************************************************************/

/**
  * @brief  Impedance Control parameters initial
  */
void Impedance_Init(Impedance_Controller_t *imp, float K, float B, float voltage_limit);
/**
  * @brief  Calculation Impedance Control
  */
float Impedance_Compute(Impedance_Controller_t *imp, float target_pos, float target_vel, float cont_pos);
/**
  * @brief  
  */
void Angle_Unwrapper_Init(Angle_Unwrapper_t *unw, float initial_raw_rad);
/**
  * @brief  
  */
	float Angle_Unwrapper_Update(Angle_Unwrapper_t *unw, float raw_rad) ;

#endif