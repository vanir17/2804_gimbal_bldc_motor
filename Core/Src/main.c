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
#include "FOC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
	float prev_angle;
	float velocity_raw;
	float velocity_filtered;
	float filter_alpha; // [0;1]
	uint32_t prev_time;
}Velocity_Estimator_t;
typedef struct 
{
	float Kp, Ki, Kd;
	float integral;
	float integral_limit;
	float prev_error;
	float output_limit;
	uint32_t prev_time; //ms
}PID_Controller_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
FOC_Controller_t foc_motor;
PID_Controller_t pos_pid;
float target_angle = 0.0f; //rad - [0; 2pi]

//plot values
float  current_mechanical_angle = 0.0f;
float current_Vq = 0.0f;



/*******************************************************************************
 * Parameters PID Velocity
 ******************************************************************************/
PID_Controller_t vel_pid;
Velocity_Estimator_t vel_est;
float target_velocity = 31.4159f;
float current_velocity = 0.0f;
uint32_t control_loop_timer = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/*******************************************************************************
 * PID Position function prototypes
 ******************************************************************************/
void PID_init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float output_limit);
float PID_compute(PID_Controller_t *pid, float setpoint, float measurement);
float normalize_angle_error(float error);

/*******************************************************************************
 * PID velocity function prototypes
 ******************************************************************************/
void  VelEst_init(Velocity_Estimator_t *vel, float filter_alpha);
float VelEst_update(Velocity_Estimator_t *vel, float current_angle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float normalize_angle_error(float error)
{
	  while (error > M_PI)  error -= 2.0f * M_PI;
    while (error < -M_PI) error += 2.0f * M_PI;
    return error;
}

void PID_init(PID_Controller_t *pid, float Kp, float Ki, float Kd, float output_limit)
{
	  pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->integral_limit = output_limit;
    pid->prev_error = 0.0f;
    pid->output_limit = output_limit;
    pid->prev_time = HAL_GetTick();
}
float PID_compute(PID_Controller_t *pid, float setpoint, float measurement)
{
	uint32_t now = HAL_GetTick();
	float dt = (now - pid->prev_time) / 1000.0f;
	if(dt <= 0.0f) dt = 0.001f;
	pid->prev_time = now;
	
	float error = normalize_angle_error(setpoint - measurement);
	
	//Ki
	pid->integral += error * dt;
  if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
  if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
	
	//Kp
	float derivative = (error - pid->prev_error) / dt;
	pid->prev_error = error;
	
	//Output
	float output = pid->Kp * error + pid->integral * pid->Ki + pid->Kd * derivative;
	
	if (output >  pid->output_limit) output =  pid->output_limit;
  if (output < -pid->output_limit) output = -pid->output_limit;
	
	return output;
}


void VelEst_init(Velocity_Estimator_t *vel, float filter_alpha)
{
	vel->prev_angle = AS5600_GetAngleRad(&hi2c1);
	vel->velocity_raw = 0.0f;
	vel->velocity_filtered = 0.0f;
	vel->filter_alpha = filter_alpha;
	vel->prev_time = HAL_GetTick();
}

float VelEst_update(Velocity_Estimator_t *vel, float current_angle)
{
	uint32_t now = HAL_GetTick();
	float dt = (now - vel->prev_time) / 1000.0f;
	if(dt <= 0.0f) dt = 0.001f;
	vel->prev_time = now;
	
	float delta = normalize_angle_error(current_angle - vel->prev_angle);
	if(delta > M_PI) delta -= 2.0f * M_PI;
	if(delta < -M_PI) delta += 2.0f * M_PI;
	
	vel->prev_angle = current_angle;
	vel->velocity_raw = delta / dt; // rad/s
	
	//Low-Pass Filter
   vel->velocity_filtered += vel->filter_alpha * (vel->velocity_raw - vel->velocity_filtered);
	
	return vel->velocity_filtered;
}

float PID_compute_linear(PID_Controller_t *pid, float setpoint, float measurement)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - pid->prev_time) / 1000.0f;
    if (dt <= 0.0001f) dt = 0.001f;
    pid->prev_time = now;
    
    float error = setpoint - measurement;
    
    float P = pid->Kp * error;
    
    pid->integral += error * dt;
    float I = pid->Ki * pid->integral;
    if (I > pid->output_limit) {
        I = pid->output_limit;
        pid->integral = I / (pid->Ki > 0.0f ? pid->Ki : 1.0f);
    } else if (I < -pid->output_limit) {
        I = -pid->output_limit;
        pid->integral = I / (pid->Ki > 0.0f ? pid->Ki : 1.0f);
    }

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    float D = pid->Kd * derivative;
    
    float output = P + I + D;
    
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
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


	/*******************************************************************************
 * PID velocity
 ******************************************************************************/
 VelEst_init(&vel_est, 0.01f);
 PID_init(&vel_pid, 0.2f, 0.7f, 0.0f, VOLTAGE_LIMIT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t now = HAL_GetTick();
      
      if (now - control_loop_timer >= 2)
      {
          control_loop_timer = now;

          current_mechanical_angle = AS5600_GetAngleRad(&hi2c1);
          
          current_velocity = VelEst_update(&vel_est, current_mechanical_angle);
          current_Vq = PID_compute_linear(&vel_pid, target_velocity, current_velocity);
          
          FOC_update_electrical_angle(&foc_motor, current_mechanical_angle);
          FOC_step(&foc_motor, current_Vq, 0.0f);

          TIM1->CCR1 = (uint32_t)(foc_motor.duty_a * (float)PWM_ARR_PERIOD);
          TIM1->CCR2 = (uint32_t)(foc_motor.duty_b * (float)PWM_ARR_PERIOD);
          TIM1->CCR3 = (uint32_t)(foc_motor.duty_c * (float)PWM_ARR_PERIOD);
      }
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
  hi2c1.Init.ClockSpeed = 400000;
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
