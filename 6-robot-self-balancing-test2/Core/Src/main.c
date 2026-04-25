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
#include "debug.h"
#include "mpu6050.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "ring_buffer.h"
#include "bt_command.h"
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Tf;
    float state;
    bool  initialized;
} LowPassFilter_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALL_ANGLE_DEG            45.0f
#define RECOVER_ANGLE_DEG         30.0f
#define STAB_SUPPLY_VOLTAGE        8.0f
#define RAD_TO_DEG                (180.0f / (float)M_PI)
#define PWM_PER_VOLT              ((float)MOTOR_PWM_MAX / STAB_SUPPLY_VOLTAGE)

#define STAB_KP_DEFAULT           (30.0f * PWM_PER_VOLT / RAD_TO_DEG)
#define STAB_KI_DEFAULT           (100.0f * PWM_PER_VOLT / RAD_TO_DEG)
#define STAB_KD_DEFAULT           (1.0f * PWM_PER_VOLT / RAD_TO_DEG)
#define STAB_INTEGRAL_LIMIT        4.0f

#define VEL_KP_DEFAULT             0.04f
#define VEL_KI_DEFAULT             0.05f
#define VEL_KD_DEFAULT             0.0f
#define VEL_INTEGRAL_LIMIT        80.0f
#define TARGET_PITCH_LIMIT_DEG    18.0f
#define STAB_OUTPUT_RAMP_PWM_S  30000.0f
#define VEL_OUTPUT_RAMP_DEG_S    120.0f

#define LPF_PITCH_TF_S             0.07f
#define LPF_THROTTLE_TF_S          0.50f
#define LPF_STEERING_TF_S          0.10f
#define LPF_VELOCITY_TF_S          0.04f

/* Set the encoder signs so both wheels report forward motion as positive.
   If forward speed still looks wrong in debug, flip one of these signs. */
#define LEFT_ENCODER_SIGN           1.0f
#define RIGHT_ENCODER_SIGN          1.0f

/* Backlash compensation for geared DC motors. Commands inside the entry
   zone are suppressed; larger commands get a kick past static slack. */
#define MOTOR_COMMAND_ZERO_BAND_PWM   8.0f
#define MOTOR_BACKLASH_DEADBAND_PWM  35.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ── Shared application state ───────────────────────────────────────── */
static MPU6050_Data_t  g_imu = {0};
static PID_t           g_stab_pid;
static PID_t           g_vel_pid;
static LowPassFilter_t g_pitch_cmd_lpf;
static LowPassFilter_t g_throttle_lpf;
static LowPassFilter_t g_steering_lpf;
static LowPassFilter_t g_velocity_lpf;
static RingBuffer_t    g_bt_rb;
static float           g_balance_trim_deg = 0.7f;
static volatile float  g_target_pitch_deg = 0.0f;
static volatile float  g_throttle_cmd = 0.0f;
static volatile float  g_steering_cmd = 0.0f;
static volatile float  g_velocity_meas = 0.0f;
static volatile bool   g_fall_detected = false;

/* Last PID output – written by TIM4 ISR, read by main loop for debug */
static volatile float g_pid_output = 0.0f;
/* Single-byte DMA-free UART RX buffer (re-armed in callback) */
static uint8_t s_uart_rx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void apply_bt_command(const BT_Command_t *cmd);
static void low_pass_init(LowPassFilter_t *filter, float tf);
static float low_pass_apply(LowPassFilter_t *filter, float input, float dt);
static float compensate_motor_pwm(float command_pwm);
static void reset_control_state(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void low_pass_init(LowPassFilter_t *filter, float tf)
{
    filter->Tf = tf;
    filter->state = 0.0f;
    filter->initialized = false;
}

static float low_pass_apply(LowPassFilter_t *filter, float input, float dt)
{
    if (filter->Tf <= 0.0f) {
        filter->state = input;
        filter->initialized = true;
        return input;
    }

    if (!filter->initialized) {
        filter->state = input;
        filter->initialized = true;
        return input;
    }

    filter->state += (dt / (filter->Tf + dt)) * (input - filter->state);
    return filter->state;
}

static float compensate_motor_pwm(float command_pwm)
{
    float magnitude = fabsf(command_pwm);

    if (magnitude <= MOTOR_COMMAND_ZERO_BAND_PWM) {
        return 0.0f;
    }

    if (magnitude >= (float)MOTOR_PWM_MAX) {
        return (command_pwm > 0.0f) ? (float)MOTOR_PWM_MAX : -(float)MOTOR_PWM_MAX;
    }

    magnitude = MOTOR_BACKLASH_DEADBAND_PWM +
                ((magnitude - MOTOR_COMMAND_ZERO_BAND_PWM) *
                 ((float)MOTOR_PWM_MAX - MOTOR_BACKLASH_DEADBAND_PWM) /
                 ((float)MOTOR_PWM_MAX - MOTOR_COMMAND_ZERO_BAND_PWM));

    return (command_pwm > 0.0f) ? magnitude : -magnitude;
}

static void reset_control_state(void)
{
    PID_Reset(&g_stab_pid);
    PID_Reset(&g_vel_pid);
    low_pass_init(&g_pitch_cmd_lpf, LPF_PITCH_TF_S);
    low_pass_init(&g_throttle_lpf, LPF_THROTTLE_TF_S);
    low_pass_init(&g_steering_lpf, LPF_STEERING_TF_S);
    low_pass_init(&g_velocity_lpf, LPF_VELOCITY_TF_S);
    g_target_pitch_deg = g_balance_trim_deg;
    g_pid_output = 0.0f;
    g_velocity_meas = 0.0f;
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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  rb_init(&g_bt_rb);

  Debug_Init(&huart2);

   /* Wait until MPU6050 responds on I2C */
   while (!MPU6050_Init(&hi2c1)) {
       HAL_Delay(100);
   }
   Debug_Print("MPU6050 OK\r\n");

   /* Arduino-FOC-balancer style cascade:
      velocity PID -> target pitch -> stabilisation PID -> motor PWM */
   PID_Init(&g_stab_pid,
            STAB_KP_DEFAULT,
            STAB_KI_DEFAULT,
            STAB_KD_DEFAULT,
            STAB_INTEGRAL_LIMIT,
            (float)MOTOR_PWM_MAX,
            STAB_OUTPUT_RAMP_PWM_S);
   PID_Init(&g_vel_pid,
            VEL_KP_DEFAULT,
            VEL_KI_DEFAULT,
            VEL_KD_DEFAULT,
            VEL_INTEGRAL_LIMIT,
            TARGET_PITCH_LIMIT_DEG,
            VEL_OUTPUT_RAMP_DEG_S);
   reset_control_state();

   Encoder_Start();   // starts TIM2 + TIM3 in encoder mode

   /* Start PWM outputs on TIM1 CH1 (PA8) and CH4 (PA11) */
   HAL_TIM_PWM_Start(&htim1, MOTOR_B_PWM_CH);   // CH1 → PA8  = PWMB
   HAL_TIM_PWM_Start(&htim1, MOTOR_A_PWM_CH);   // CH4 → PA11 = PWMA
   Motor_Stop();

   /* Arm USART2 RX interrupt */
   HAL_UART_Receive_IT(&huart2, &s_uart_rx_byte, 1);

   /* Start 200 Hz control-loop timer (TIM4) */
   HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  	  /* 1. Parse any complete BT command from ring buffer */
	          BT_Command_t cmd;
	          if (BT_Parse(&g_bt_rb, &cmd)) {
	              apply_bt_command(&cmd);
	          }

	          /* 2. Report fall state changes once, while the ISR keeps outputs safe. */
	          static bool prev_fall_detected = false;
	          if (g_fall_detected != prev_fall_detected) {
	              prev_fall_detected = g_fall_detected;
	              Debug_Print(g_fall_detected ? "FALL DETECTED\r\n"
	                                          : "Recovered - balance loop re-armed\r\n");
	          }

	          /* 3. Optional telemetry – uncomment to stream angle over BT
	          static uint32_t last_tx = 0;
	          if (HAL_GetTick() - last_tx >= 100) {
	              last_tx = HAL_GetTick();
	              char buf[48];
	              int n = snprintf(buf, sizeof(buf), "A:%.2f L:%ld R:%ld\r\n",
	                               g_imu.angle, enc_left.count, enc_right.count);
	              HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 10);
	          }
	          */

	          /* 3. Stream debug data over Bluetooth every DEBUG_INTERVAL_MS */
	                  Debug_Update(g_imu.angle,
	                               g_imu.gyro_rate,
	                               g_imu.accel_angle,
	                               g_pid_output,
	                               enc_left.delta,
	                               enc_right.delta,
	                               g_target_pitch_deg,
	                               g_stab_pid.Kp,
	                               g_stab_pid.Ki,
	                               g_stab_pid.Kd);
	/* 4. Sleep until next interrupt */
	__WFI();
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  __HAL_TIM_MOE_ENABLE(&htim1);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ═══════════════════════════════════════════════════════════════════════
   TIM4 PERIOD ELAPSED IRQ  –  200 Hz CONTROL LOOP  (every 5 ms)
   ═══════════════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) return;

    /* Step 1 – Snapshot encoder deltas (hardware counted, we just diff) */
    Encoder_Update();

    /* Step 2 – Read IMU and update complementary filter
                ~300 µs at 400 kHz I2C; well within the 5 ms budget       */
    MPU6050_Update(&hi2c1, &g_imu, CONTROL_LOOP_DT);

    if (fabsf(g_imu.angle) >= FALL_ANGLE_DEG) {
        g_fall_detected = true;
        reset_control_state();
        Motor_Stop();
        return;
    }

    if (g_fall_detected && fabsf(g_imu.angle) <= RECOVER_ANGLE_DEG) {
        g_fall_detected = false;
        reset_control_state();
    }

    /* Arduino reference logic:
       filtered throttle -> velocity PID -> filtered target pitch ->
       stabilisation PID -> motor PWM, plus filtered steering trim. */
    float left_velocity = LEFT_ENCODER_SIGN * ((float)enc_left.delta / CONTROL_LOOP_DT);
    float right_velocity = RIGHT_ENCODER_SIGN * ((float)enc_right.delta / CONTROL_LOOP_DT);
    g_velocity_meas = low_pass_apply(&g_velocity_lpf,
                                     0.5f * (left_velocity + right_velocity),
                                     CONTROL_LOOP_DT);

    float throttle_cmd = low_pass_apply(&g_throttle_lpf, g_throttle_cmd, CONTROL_LOOP_DT);
    float target_pitch = PID_Compute(&g_vel_pid,
                                     throttle_cmd,
                                     g_velocity_meas,
                                     CONTROL_LOOP_DT);
    target_pitch = g_balance_trim_deg +
                   low_pass_apply(&g_pitch_cmd_lpf, target_pitch, CONTROL_LOOP_DT);

    float drive_pwm = PID_Compute(&g_stab_pid,
                                  target_pitch,
                                  g_imu.angle,
                                  CONTROL_LOOP_DT);
    float steering_pwm = low_pass_apply(&g_steering_lpf, g_steering_cmd, CONTROL_LOOP_DT);
    float left_pwm = compensate_motor_pwm(drive_pwm + steering_pwm);
    float right_pwm = compensate_motor_pwm(drive_pwm - steering_pwm);

    g_target_pitch_deg = target_pitch;
    g_pid_output = drive_pwm;

    Motor_Set((int32_t)left_pwm,
              (int32_t)right_pwm);
}

/* ═══════════════════════════════════════════════════════════════════════
   USART2 RX COMPLETE IRQ  –  JDY-23 Bluetooth
   One byte at a time → ring buffer → parsed in main loop
   ═══════════════════════════════════════════════════════════════════════ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    rb_push(&g_bt_rb, s_uart_rx_byte);
    HAL_UART_Receive_IT(&huart2, &s_uart_rx_byte, 1);  // re-arm
}

/* ═══════════════════════════════════════════════════════════════════════
   BT COMMAND APPLICATION  (main loop only, never from ISR)
   ═══════════════════════════════════════════════════════════════════════ */
static void apply_bt_command(const BT_Command_t *cmd)
{
    char buf[64];
    switch (cmd->type) {
        case BT_CMD_FORWARD:
            g_throttle_cmd = cmd->value;
            snprintf(buf, sizeof(buf), "CMD: Forward throttle %.1f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_BACKWARD:
            g_throttle_cmd = -cmd->value;
            snprintf(buf, sizeof(buf), "CMD: Backward throttle %.1f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_STOP:
            g_throttle_cmd = 0.0f;
            g_steering_cmd = 0.0f;
            reset_control_state();
            Debug_Print("CMD: Stop\r\n");
            break;
        case BT_CMD_SET_KP:
            g_stab_pid.Kp = cmd->value;
            snprintf(buf, sizeof(buf), "SET stab Kp=%.3f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_KI:
            g_stab_pid.Ki = cmd->value;
            PID_Reset(&g_stab_pid);
            snprintf(buf, sizeof(buf), "SET stab Ki=%.3f (integral reset)\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_KD:
            g_stab_pid.Kd = cmd->value;
            snprintf(buf, sizeof(buf), "SET stab Kd=%.3f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_ANGLE_OFFSET:
            g_balance_trim_deg = cmd->value;
            reset_control_state();
            snprintf(buf, sizeof(buf), "SET balance trim=%.2f deg\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_THROTTLE:
            g_throttle_cmd = cmd->value;
            snprintf(buf, sizeof(buf), "SET throttle=%.2f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_STEERING:
            g_steering_cmd = cmd->value;
            snprintf(buf, sizeof(buf), "SET steering=%.2f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_VEL_KP:
            g_vel_pid.Kp = cmd->value;
            snprintf(buf, sizeof(buf), "SET vel Kp=%.3f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_VEL_KI:
            g_vel_pid.Ki = cmd->value;
            PID_Reset(&g_vel_pid);
            snprintf(buf, sizeof(buf), "SET vel Ki=%.3f (integral reset)\r\n", cmd->value);
            Debug_Print(buf);
            break;
        case BT_CMD_SET_VEL_KD:
            g_vel_pid.Kd = cmd->value;
            snprintf(buf, sizeof(buf), "SET vel Kd=%.3f\r\n", cmd->value);
            Debug_Print(buf);
            break;
        default:
            Debug_Print("CMD: Unknown\r\n");
            break;
    }
}
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
