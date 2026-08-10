/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdbool.h"
#include "as5600.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLTAGE_POWER 12.0f
#define VOLTAGE_LIMIT 6.0f
#define PWM_ARR_PERIOD 1799
#define POLE_PAIRS    7
#define M_PI 3.14159
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
FOC_Controller_t foc_motor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
float normalize_angle(float angle);
void FOC_init(FOC_Controller_t *foc, uint8_t pole_pairs, float v_dc, float v_limit);
void FOC_update_electrical_angle(FOC_Controller_t *foc, float mech_angle_rad);
void FOC_step(FOC_Controller_t *foc, float Vq, float Vd);
void FOC_align_sensor(FOC_Controller_t *foc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float normalize_angle(float angle)
{
	float a = fmodf(angle, 2.0f * M_PI);
	return a < 0.0f ? (a + 2.0f * M_PI) : a;
}
	
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
	
	//set the motor at rest
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
	if(Vb > Vmax)
	{
		Vmax = Vb;
	}
	if(Vc > Vmax) 
	{
		Vmax = Vc;
	}
	
	float Vmin = Va;
	if(Vb < Vmin)
	{
		Vmin = Vb;
	}
	if(Vc < Vmin)
	{
		Vmin = Vc;
	}
	
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

void FOC_align_sensor(FOC_Controller_t *foc)
{
		foc->angle_electrical = 0.0f;
		FOC_step(foc, 0.0f, 2.0f);
	
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	/*init motor*/
	FOC_init(&foc_motor, POLE_PAIRS, VOLTAGE_POWER, VOLTAGE_LIMIT);
	
	/*init TIM1*/
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_MOE_ENABLE(&htim1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(10);
	FOC_align_sensor(&foc_motor);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		float current_mechanical_angle = AS5600_GetAngleRad(&hi2c1);
		
		FOC_update_electrical_angle(&foc_motor, current_mechanical_angle);
		
		FOC_step(&foc_motor, 0.5f, 0.0f);

		TIM1->CCR1 = (uint32_t)(foc_motor.duty_a * (float)PWM_ARR_PERIOD);
    TIM1->CCR2 = (uint32_t)(foc_motor.duty_b * (float)PWM_ARR_PERIOD);
    TIM1->CCR3 = (uint32_t)(foc_motor.duty_c * (float)PWM_ARR_PERIOD);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
