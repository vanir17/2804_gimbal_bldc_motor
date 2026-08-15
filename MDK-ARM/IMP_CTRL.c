/**
  ******************************************************************************
  * @file    IMP_CTRL.c
  * @brief   Implementation file for FOC library.
  * @author  Vanir 
  * @date    15/8/2026
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "IMP_CTRL.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "as5600.h"
#include "FOC.h"


void Impedance_Init(Impedance_Controller_t *imp, float K, float B, float voltage_limit)
{
	imp->K = K;
	imp->B = B;
	imp->target_pos = 0.0f;
	imp->target_vel = 0.0f;
	imp->prev_pos = 0.0f;
	imp->prev_time = HAL_GetTick();
	imp->voltage_limit = voltage_limit;
	imp->vel_filtered = 0.0f;
	imp->filter_alpha = 0.0f;
}

float Impedance_Compute(Impedance_Controller_t *imp, float target_pos, float target_vel, float cont_pos)
{
	uint32_t now = HAL_GetTick();
	float dt = (now - imp->prev_time) / 1000.0f;
	if(dt <= 0.0001f) dt = 0.001f;
	imp->prev_time = now;
	
	float pos_error = target_pos - cont_pos;
	
	float delta_pos = target_pos - cont_pos;
	imp->prev_pos = cont_pos;
	float vel_raw = delta_pos / dt;
	
	//Low-pass Filter for Velocity
	imp->vel_filtered += imp->filter_alpha * (vel_raw - imp->vel_filtered);
	
	float vel_error = target_vel - imp->vel_filtered;
	float Vq = (imp->K * pos_error) - (imp->B * vel_error);
	
	if(Vq >  imp->voltage_limit) Vq = imp->voltage_limit;
	if(Vq < -imp->voltage_limit) Vq = -imp->voltage_limit;
		
	
	return Vq;
}

void Angle_Unwrapper_Init(Angle_Unwrapper_t *unw, float initial_raw_rad) 
{
    unw->raw_prev = initial_raw_rad;
    unw->full_rotations = 0;
    unw->continuous_angle = initial_raw_rad;
}
float Angle_Unwrapper_Update(Angle_Unwrapper_t *unw, float raw_rad) 
{
    float d_angle = raw_rad - unw->raw_prev;

    if (d_angle < -M_PI) {
        unw->full_rotations++;
    } else if (d_angle > M_PI) {
        unw->full_rotations--;
    }

    unw->raw_prev = raw_rad;
    unw->continuous_angle = (float)unw->full_rotations * (2.0f * M_PI) + raw_rad;
    return unw->continuous_angle;
}