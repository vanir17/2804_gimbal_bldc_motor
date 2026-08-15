/**
  ******************************************************************************
  * @file    FOC.c
  * @brief   Implementation file for FOC library.
  * @author  Vanir 
  * @date    13/8/2026
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FOC.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "as5600.h"
void FOC_init(FOC_Controller_t *foc, uint8_t pole_pairs, float v_dc, float v_limit)
{

	foc->pole_pairs = pole_pairs;
	foc->voltage_power = v_dc;
	foc->voltage_limit = v_limit;
	
	foc->zero_offset_angle = 0.0f;
	foc->rotation_direction = 1; //1:forward, -1: inverse
	
	foc->duty_a = 0.5f;
	foc->duty_b = 0.5f;
	foc->duty_c = 0.5f;
	
	//Set the motor to idle
}


void FOC_align_sensor(FOC_Controller_t *foc)
{
		foc->angle_electrical = 0.0f;
		FOC_step(foc, 0.0f, 3.5f);
	
		for(int i = 0; i < 1000; i++)
		{
			TIM1->CCR1 = (uint32_t)(foc->duty_a * (float)PWM_ARR_PERIOD);
      TIM1->CCR2 = (uint32_t)(foc->duty_b * (float)PWM_ARR_PERIOD);
      TIM1->CCR3 = (uint32_t)(foc->duty_c * (float)PWM_ARR_PERIOD);
			
			HAL_Delay(1);
		}
		
		float sensor_raw = AS5600_GetAngleRad(&hi2c1);
		foc->zero_offset_angle = normalize_angle(foc->rotation_direction * sensor_raw * (float)foc->pole_pairs);
		
		FOC_step(foc, 0.0f, 0.0f);
    TIM1->CCR1 = (uint32_t)(foc->duty_a * (float)PWM_ARR_PERIOD);
    TIM1->CCR2 = (uint32_t)(foc->duty_b * (float)PWM_ARR_PERIOD);
    TIM1->CCR3 = (uint32_t)(foc->duty_c * (float)PWM_ARR_PERIOD);
    HAL_Delay(200);
}

void FOC_update_electrical_angle(FOC_Controller_t *foc, float mech_angle_rad)
{
	foc->angle_as5600 = mech_angle_rad;
	
	float raw_electrical = (foc->rotation_direction * foc->angle_as5600 *(float)foc->pole_pairs) - foc->zero_offset_angle;
	foc->angle_electrical = normalize_angle(raw_electrical);	
}



void FOC_step(FOC_Controller_t *foc, float Vq, float Vd)
{
	if (Vq > foc->voltage_limit)  Vq = foc->voltage_limit;
  if (Vq < -foc->voltage_limit) Vq = -foc->voltage_limit;
  if (Vd > foc->voltage_limit)  Vd = foc->voltage_limit;
  if (Vd < -foc->voltage_limit) Vd = -foc->voltage_limit;
	
	//INVERSE PARK TRANSFORMATION
	float sin_e = sinf(foc->angle_electrical);
	float cos_e = cosf(foc->angle_electrical);
	
	float V_alpha = Vd * cos_e - Vq * sin_e;
	float V_beta = Vd * sin_e + Vq * cos_e;
	
	//INVERSE CLARKE TRANSFORMATION
	float Va = V_alpha;
	float Vb = -0.5f * V_alpha + 0.86602540378f * V_beta; //0.866 = sqrt(3) / 2
	float Vc = -0.5f * V_alpha - 0.86602540378f * V_beta;
	
	//SPACE VECTOR PWM
	float Vmax = Va;
	if(Vb > Vmax) Vmax = Vb;
	if(Vc > Vmax) Vmax = Vc;
	
	float Vmin = Va;
	if(Vb < Vmin) Vmin = Vb;
	if(Vc < Vmin) Vmin = Vc;

	
	//Offset
	float Voffset = 0.5f * (Vmax + Vmin);
	
	Va -= Voffset;
	Vb -= Voffset;
	Vc -= Voffset;
	
	//Duty Cycle 0.0 -> 1.0
	foc->duty_a = (Va / foc-> voltage_power) + 0.5f;
	foc->duty_b = (Vb / foc-> voltage_power) + 0.5f;
	foc->duty_c = (Vc / foc-> voltage_power) + 0.5f;

	//Ensure the Duty cycle always at 0.0 -> 1.0
	if(foc->duty_a > 1.0f) foc->duty_a = 1.0f;
	else if(foc->duty_a < 0.0f) foc->duty_a = 0.0f;
	
	if(foc->duty_b > 1.0f) foc->duty_b = 1.0f;
	else if(foc->duty_b < 0.0f) foc->duty_b = 0.0f;
	
	if(foc->duty_c > 1.0f) foc->duty_c = 1.0f;
	else if(foc->duty_c < 0.0f) foc->duty_c = 0.0f;
}
float normalize_angle(float angle)
{
	float a = fmodf(angle, 2.0f * M_PI);
	return a < 0.0f ? (a + 2.0f * M_PI) : a;
}