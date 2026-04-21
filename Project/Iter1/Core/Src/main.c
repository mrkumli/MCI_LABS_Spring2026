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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t raw_x, raw_y, raw_z;
    float scaled_x, scaled_y, scaled_z;
    float offset_x, offset_y, offset_z;
    int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
    float gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
} LSM_Data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ── IMU ── */
#define LSM_ADDR       (0x19 << 1)   /* Accelerometer I2C address (SA0=1) */
#define GYRO_CS_PORT   GPIOE
#define GYRO_CS_PIN    GPIO_PIN_3    /* PE3 = GYRO_CS from Task 1 */
#define GYRO_CTRL_REG1 0x20
#define GYRO_OUT_X_L   0x28

// Motor Control Constants
#define MAX_PWM 999.0f

/* ── Motor direction pins ──
 * Right motor: PB4 (DIR A, shield D12) and PC8 (DIR B, shield D8)
 * Left  motor: PC7 (DIR A, shield D7)  and PC5 (DIR B, shield D6 mapped free)
 * Adjust PC5 to whichever free pin you physically wired shield D6 to.
 */
#define RIGHT_DIR1_PORT  GPIOB
#define RIGHT_DIR1_PIN   GPIO_PIN_4   /* PB4 — shield D12 */
#define RIGHT_DIR2_PORT  GPIOC
#define RIGHT_DIR2_PIN   GPIO_PIN_8   /* PC8 — shield D8  */
#define LEFT_DIR1_PORT   GPIOC
#define LEFT_DIR1_PIN    GPIO_PIN_7   /* PC7 — shield D7  */
#define LEFT_DIR2_PORT   GPIOB
#define LEFT_DIR2_PIN    GPIO_PIN_3   /* PB3 — shield D6 */


/* ── PWM ──
 * TIM3 ARR = 999, so valid range is 0–999.
 * MAX_PWM caps PID output so we never exceed ARR.
 * DEADZONE lifts the floor so motors actually spin (overcome static friction).
 */
#define MAX_PWM      999.0f
#define PWM_DEADZONE 150.0f   /* Tune this: increase if motors don't start */

/* ── Timing ──
 * DT must match TIM4: Prescaler=4799, Period=49
 * → 48MHz / 4800 / 50 = 200Hz → DT = 1/200 = 0.005s
 * Using a #define ensures ONE value is used everywhere (filter + PID).
 */
#define DT  0.005f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LSM_Data acc_data;

/* Shared between ISR and main loop */
volatile uint8_t display_flag = 0;

/* Complementary filter state */
float tilt_angle    = 0.0f;
float acc_angle     = 0.0f;

/*
 * PID gains — start with Kp=5, Ki=0, Kd=0 (P-only) to verify motor direction.
 * Once direction is confirmed correct, add Ki and Kd.
 * Lab requires at least 2 terms (PI, PD, or PID).
 */
float Kp = 25.0f;
// float Ki = 0.1f;
// float Kd = 0.5f;
float Ki = 0.1f;
float Kd = 0.5f;

float setpoint       = -1.25f;  /* Desired angle = 0° (upright) */
float integral       = 0.0f;
float previous_error = 0.0f;
float pid_output     = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void LSM_Init(void);
void GYRO_Init(void);
void LSM_Read(LSM_Data *data);
void GYRO_Read(LSM_Data *data);
void Calibrate_Sensors(LSM_Data *data);
void Set_Motor_Speeds(float pid_value);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*
 * Calibrate_Sensors()
 * ────────────────────
 * WHY: Both sensors have a small constant error (offset/bias) at rest.
 *      We measure that bias here so every future reading subtracts it out.
 * HOW: Take 50 readings while board is flat and still, average them.
 *      Accelerometer Z gets 1g subtracted because gravity acts along Z when flat.
 * IMPORTANT: Board must be completely still during these ~500ms.
 */

void Calibrate_Sensors(LSM_Data *data) {
    float sx=0,sy=0,sz=0, gx=0,gy=0,gz=0;

    /* Zero offsets first so reads during calibration are raw (uncompensated) */
    data->offset_x = data->offset_y = data->offset_z = 0.0f;
    data->gyro_offset_x = data->gyro_offset_y = data->gyro_offset_z = 0.0f;

    for (int i = 0; i < 50; i++) {
        LSM_Read(data);
        GYRO_Read(data);
        sx += data->scaled_x;  sy += data->scaled_y;  sz += data->scaled_z;
        gx += data->gyro_scaled_x; gy += data->gyro_scaled_y; gz += data->gyro_scaled_z;
        HAL_Delay(10);
    }
    data->offset_x = sx/50.0f;
    data->offset_y = sy/50.0f;
    data->offset_z = (sz/50.0f) - 1.0f;  /* Remove 1g gravity from Z */

    data->gyro_offset_x = gx/50.0f;
    data->gyro_offset_y = gy/50.0f;
    data->gyro_offset_z = gz/50.0f;
}

/* Redirect printf() to UART so we can use printf() for serial output */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  /* Initialize IMU sensors */
    LSM_Init();
    GYRO_Init();

   /*
     * Wait 250ms for hardware to fully power on.
     * During this time: place the robot flat on a table and DON'T touch it.
     * The next call (Calibrate_Sensors) will measure the resting bias.
     */
    HAL_Delay(250);
    Calibrate_Sensors(&acc_data);

    /*
     * Start TIM3 PWM on both motor channels.
     * CH1 = PC6 = Right motor (shield D10)
     * CH4 = PC9 = Left  motor (shield D9)
     * Both start at 0 duty cycle (motors off until PID commands them).
     */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /*
     * Start TIM4 interrupt — this kicks off the 200Hz control loop.
     * From this point on, HAL_TIM_PeriodElapsedCallback fires every 5ms.
     */
    HAL_TIM_Base_Start_IT(&htim4);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
      /*
         * Main loop ONLY handles UART printing at 10Hz.
         * The ISR sets display_flag every 20 ticks (200Hz/20 = 10Hz).
         * We read the flag here and print — this keeps slow UART I/O
         * out of the time-critical ISR.
         */
        if (display_flag == 1) {
            display_flag = 0;
            /*
             * Print 3 values separated by commas for sensorplot.py or serial monitor:
             * Column 1: accelerometer angle estimate
             * Column 2: gyroscope rate (deg/s)
             * Column 3: complementary filter output (the actual tilt angle used by PID)
             */
            printf("%.2f,%.2f,%.2f\r\n",
                   acc_angle,
                   acc_data.gyro_scaled_x,
                   tilt_angle);
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 4799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GYRO_CS_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEFT_DIR1_Pin|RIGHT_DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEFT_DIR2_Pin|RIGHT_DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GYRO_CS_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = GYRO_CS_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_DIR1_Pin RIGHT_DIR2_Pin */
  GPIO_InitStruct.Pin = LEFT_DIR1_Pin|RIGHT_DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_DIR2_Pin RIGHT_DIR1_Pin */
  GPIO_InitStruct.Pin = LEFT_DIR2_Pin|RIGHT_DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LSM_Init(void) {
    uint8_t data;
    data = 0x67;
    HAL_I2C_Mem_Write(&hi2c1, LSM_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    data = 0x88;
    HAL_I2C_Mem_Write(&hi2c1, LSM_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}



/*
 * LSM_Read()
 * ──────────
 * Read 6 bytes of accelerometer data via I2C (X, Y, Z each as 2 bytes).
 * The 0x80 OR sets the auto-increment bit so all 6 registers are read in one call.
 * In High-Resolution mode: shift right 4 bits, sensitivity = 1mg/LSB.
 * Divide by 1000 to convert mg → g, then subtract calibrated offset.
 */
void LSM_Read(LSM_Data *data) {
    uint8_t rawData[6];
    HAL_I2C_Mem_Read(&hi2c1, LSM_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT,
                     rawData, 6, HAL_MAX_DELAY);

    data->raw_x = ((int16_t)((rawData[1]<<8)|rawData[0])) >> 4;
    data->raw_y = ((int16_t)((rawData[3]<<8)|rawData[2])) >> 4;
    data->raw_z = ((int16_t)((rawData[5]<<8)|rawData[4])) >> 4;

    data->scaled_x = (data->raw_x / 1000.0f) - data->offset_x;
    data->scaled_y = (data->raw_y / 1000.0f) - data->offset_y;
    data->scaled_z = (data->raw_z / 1000.0f) - data->offset_z;
}

/*
 * GYRO_Init()
 * ───────────
 * Configure the L3GD20 gyroscope via SPI.
 * CTRL_REG1 = 0x0F → Power on, enable XYZ axes, 95Hz ODR
 * We manually drive CS low/high because NSS is set to software mode.
 */
void GYRO_Init(void) {
    uint8_t data[2] = {GYRO_CTRL_REG1, 0x0F};
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

// Add this helper function right above GYRO_Read
uint8_t GYRO_Read_Byte(uint8_t reg) {
    uint8_t tx = 0x80 | reg; // 0x80 is the Read bit
    uint8_t rx = 0;
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &rx, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return rx;
}

/*
 * GYRO_Read()
 * ───────────
 * Read all 6 gyroscope registers in one SPI burst transaction.
 * 0xC0 = 0x80 (read) | 0x40 (auto-increment address).
 * Sensitivity at ±250dps: 0.00875 dps/LSB.
 * Subtract calibrated offset to remove resting drift.
 */
void GYRO_Read(LSM_Data *data) {
    // uint8_t tx_buf[7] = {0};
    // uint8_t rx_buf[7] = {0};
    // tx_buf[0] = 0xC0 | GYRO_OUT_X_L;

    // HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    // HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 7, HAL_MAX_DELAY);
    // HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    // data->gyro_raw_x = (int16_t)((rx_buf[2]<<8)|rx_buf[1]);
    // data->gyro_raw_y = (int16_t)((rx_buf[4]<<8)|rx_buf[3]);
    // data->gyro_raw_z = (int16_t)((rx_buf[6]<<8)|rx_buf[5]);

    // data->gyro_scaled_x = (data->gyro_raw_x * 0.00875f) - data->gyro_offset_x;
    // data->gyro_scaled_y = (data->gyro_raw_y * 0.00875f) - data->gyro_offset_y;
    // data->gyro_scaled_z = (data->gyro_raw_z * 0.00875f) - data->gyro_offset_z;

    // // NEW CODE BY GEMINI
    // Read registers individually using your helper function
    uint8_t xL = GYRO_Read_Byte(0x28);
    uint8_t xH = GYRO_Read_Byte(0x29);
    uint8_t yL = GYRO_Read_Byte(0x2A);
    uint8_t yH = GYRO_Read_Byte(0x2B);
    uint8_t zL = GYRO_Read_Byte(0x2C);
    uint8_t zH = GYRO_Read_Byte(0x2D);

    data->gyro_raw_x = (int16_t)((xH << 8) | xL);
    data->gyro_raw_y = (int16_t)((yH << 8) | yL);
    data->gyro_raw_z = (int16_t)((zH << 8) | zL);

    // Scale and apply the offset to remove resting drift
    data->gyro_scaled_x = (data->gyro_raw_x * 0.00875f) - data->gyro_offset_x;
    data->gyro_scaled_y = (data->gyro_raw_y * 0.00875f) - data->gyro_offset_y;
    data->gyro_scaled_z = (data->gyro_raw_z * 0.00875f) - data->gyro_offset_z;
}


/*
 * HAL_TIM_PeriodElapsedCallback() — THE CONTROL LOOP
 * ────────────────────────────────────────────────────
 * This function is called automatically by HAL every time TIM4 overflows.
 * With Prescaler=4799, Period=49 at 48MHz → fires exactly every 5ms (200Hz).
 *
 * Why put everything here?
 * If we put the filter + PID in while(1), the loop time varies because
 * printf() and other operations take variable time. A varying dt makes
 * the complementary filter drift and the PID behave unpredictably.
 * The timer ISR fires at a guaranteed fixed interval — no jitter.
 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        static int counter = 0;

        /* ── STEP 1: Read sensors ──
         * We read both IMU sensors every 5ms.
         * The I2C/SPI calls here are blocking but very short (~50µs each),
         * acceptable at 200Hz since we have 5ms budget per tick.
         */
        LSM_Read(&acc_data);
        GYRO_Read(&acc_data);

        /* ── STEP 2: Complementary filter ──
         *
         * acc_angle: tilt angle calculated from accelerometer alone.
         *   atan2(X, Z) gives the angle of the board relative to gravity
         *   in the X-Z plane — this is the pitch/tilt axis.
         *   Multiplying by 180/π converts radians to degrees.
         *   This is accurate but noisy (vibrations affect it).
         *
         * tilt_angle: blended estimate.
         *   98% comes from integrating the gyroscope (fast, smooth, drifts slowly).
         *   2% comes from the accelerometer (slow, noisy, drift-free long-term).
         *   Together they give a stable, low-noise angle estimate.
         *
         * DT is the fixed 0.005s interval — MUST match TIM4 rate exactly.
         *
         * Note on axes: atan2(scaled_x, scaled_z) with gyro_scaled_x assumes
         * the board tilts around the X axis. If your angle reads near 90° when
         * flat or is backwards, try swapping to atan2(scaled_y, scaled_z) and
         * using gyro_scaled_y. This depends on physical board orientation.
         */
        acc_angle  = atan2f(acc_data.scaled_x, acc_data.scaled_z) * (180.0f / M_PI);
        tilt_angle = 0.98f * (tilt_angle + acc_data.gyro_scaled_x * DT)
                   + 0.02f * acc_angle;

        /* ── STEP 3: PID controller ──
         *
         * error = how far we are from upright (setpoint = 0°).
         * Positive error = tilted one way. Negative = other way.
         *
         * P term: immediate response. Large error → large output.
         *         Kp too high = oscillation. Too low = sluggish.
         *
         * I term: accumulates error over time. Fixes steady-state offset
         *         (e.g. if robot settles at 2° instead of 0°, I pushes it back).
         *         Anti-windup clamp prevents I from growing unbounded when
         *         motors are already at max (integral windup causes instability).
         *
         * D term: reacts to error rate of change. Damps oscillations.
         *         Kd too high = amplifies sensor noise.
         */
        float error = setpoint - tilt_angle;

        /* P */
        float P = Kp * error;

        /* I — with anti-windup clamp */
        integral += error * DT;
        if      (integral >  500.0f) integral =  500.0f;  /* Clamp prevents windup */
        else if (integral < -500.0f) integral = -500.0f;
        float I = Ki * integral;

        /* D */
        float derivative = (error - previous_error) / DT;
        float D = Kd * derivative;

        pid_output     = P + I + D;
        previous_error = error;

        /* ── STEP 4: Drive motors ── */
        Set_Motor_Speeds(pid_output);

        /* ── STEP 5: Throttle UART to 10Hz ──
         * 200Hz ISR / counter threshold 20 = 10Hz display rate.
         * This prevents serial output from slowing down the control loop.
         */
        counter++;
        if (counter >= 20) {
            display_flag = 1;
            counter = 0;
        }
    }
}

/*
 * Set_Motor_Speeds()
 * ──────────────────
 * Translates the signed PID output into:
 *   1. Motor direction (two GPIO pins per motor, H-bridge control)
 *   2. PWM magnitude (unsigned 0–999 written to TIM3 compare register)
 *
 * H-bridge truth table for one motor:
 *   DIR1=HIGH, DIR2=LOW  → Forward
 *   DIR1=LOW,  DIR2=HIGH → Backward
 *   DIR1=LOW,  DIR2=LOW  → Coast (free spin)
 *   DIR1=HIGH, DIR2=HIGH → Brake (avoid this)
 *
 * If motors spin the wrong direction when tilted forward,
 * swap the HIGH/RESET assignments in the pid_value > 0 block.
 */


void Set_Motor_Speeds(float pid_value)
{
    float final_pwm = 0.0f;

    if (pid_value > 0.0f)
    {
        /* Tilting forward → drive forward to catch the fall */
        HAL_GPIO_WritePin(RIGHT_DIR1_PORT, RIGHT_DIR1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_DIR2_PORT, RIGHT_DIR2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_DIR1_PORT,  LEFT_DIR1_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_DIR2_PORT,  LEFT_DIR2_PIN,  GPIO_PIN_SET);
        final_pwm = pid_value;
    }
    else if (pid_value < 0.0f)
    {
        /* Tilting backward → drive backward */
        HAL_GPIO_WritePin(RIGHT_DIR1_PORT, RIGHT_DIR1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_DIR2_PORT, RIGHT_DIR2_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_DIR1_PORT,  LEFT_DIR1_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_DIR2_PORT,  LEFT_DIR2_PIN,  GPIO_PIN_RESET);
        final_pwm = -pid_value;   /* PWM register must be positive */
    }
    else
    {
        /* Exactly balanced → coast to stop */
        HAL_GPIO_WritePin(RIGHT_DIR1_PORT, RIGHT_DIR1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_DIR2_PORT, RIGHT_DIR2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_DIR1_PORT,  LEFT_DIR1_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_DIR2_PORT,  LEFT_DIR2_PIN,  GPIO_PIN_RESET);
        final_pwm = 0.0f;
    }

    /*
     * Deadzone: below ~150 PWM counts, the motors don't have enough
     * torque to overcome static friction and just buzz. Adding the
     * deadzone lifts the minimum so they always actually spin.
     * Only added when we intend to move (final_pwm > 0).
     */
    if (final_pwm > 0.0f) {
        final_pwm += PWM_DEADZONE;
    }

    /* Hard clamp — never write beyond TIM3 ARR (999) */
    if (final_pwm > MAX_PWM) {
        final_pwm = MAX_PWM;
    }

    /* Write to TIM3 compare registers — this sets the duty cycle */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)final_pwm); /* Right motor, PC6 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)final_pwm); /* Left  motor, PC9 */
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
