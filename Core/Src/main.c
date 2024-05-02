/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_RESOLUTION 4095 // Résolution du DAC
#define SIN_FREQ 1000 // Fréquence sinusoïdale en Hz (1 kHz)
#define SAMPLE_RATE 1000 // Fréquence d'échantillonnage en Hz

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* Function movement prototypes */
void move_forward_slow();
void move_forward_fast();
void move_backward_slow();
void move_backward_fast();
void turn_left();
void turn_right();
void move_stop();

/* Function to search a path when in front of obstacle */
void search_for_path();

/* Function servo motor prototypes */
void look_left();
void look_forward();
void look_right();
void led_straigth();

/* Function US sensor prototypes */
void send_trigger_pulse();
float measure_echo_pulse_duration();

/* Function TOGGLE LED prototypes */
void toggleLed(void);
void toggleLedSlow(void);
void toggleLedFast(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t uart1_buffer[1] = {' '}; // Buffer for UART1
uint8_t uart2_buffer[1] = {' '}; // Buffer for UART2

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    HAL_UART_Receive_IT(&huart1, uart1_buffer, 1); // Prepare UART1 reception
    HAL_UART_Receive_IT(&huart2, uart2_buffer, 1); // Prepare UART2 reception

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Prepare PWM for turret servo motor
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Prepare PWM for LED servo motor

    int loop_counter = 0;

    // HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1) {
        // Move robot
    	/*
        move_forward_slow();
        HAL_Delay(1000);
        turn_left();
        HAL_Delay(1000);
        move_backward_fast();
        HAL_Delay(1000);
        turn_right();
        HAL_Delay(1000);
        move_stop();
        HAL_Delay(2000);
        */

        // Move turret servo motor
        /*
        HAL_Delay(2000);
        led_straigth();
        HAL_Delay(2000);
        look_left();
        HAL_Delay(2000);
        look_forward();
        HAL_Delay(2000);
        look_right();
        HAL_Delay(2000);
        look_forward();
        HAL_Delay(2000);
        look_right();
        */

        // US sensor
        /*
        HAL_Delay(100);
        send_trigger_pulse();
        float distance = measure_echo_pulse_duration();
        */

        // Implement US sensor and move
    	/*
        send_trigger_pulse();
        float distance = measure_echo_pulse_duration();
        if (distance != 0) {
			if (distance < 0.04) {
				move_stop();
			} else if (distance < 0.1) {
				move_forward_slow();
			} else  {
				move_forward_fast();
			}
        }
        */

        // Implement line sensor and move
    	/*
        uint32_t photodiode_value_right = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
        uint32_t photodiode_value_left = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
        mesure_line_sensor(photodiode_value_right, photodiode_value_left, &line_error_correction);
        if (strcmp(line_error_correction, "resume") == 0) {
        	move_stop();
        	HAL_Delay(10);
            move_forward_slow();
        } else if (strcmp(line_error_correction, "right_shift") == 0) {
        	move_stop();
        	HAL_Delay(10);
        	turn_right();
        } else if (strcmp(line_error_correction, "left_shift") == 0) {
        	move_stop();
        	HAL_Delay(10);
        	turn_left();
        } else {
            move_stop();
        }
        HAL_Delay(50);
        */

    	// Implement line sensor, US sensor and move
    	/*
        uint32_t photodiode_value_right = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
        uint32_t photodiode_value_left = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

        float distance = 0.0;
        while (distance == 0) {
			send_trigger_pulse();
			distance = measure_echo_pulse_duration();
        }

		move_stop();
		HAL_Delay(10);
		if (distance > 0.04) {
			if (photodiode_value_right == GPIO_PIN_RESET && photodiode_value_left == GPIO_PIN_RESET) {
				if (distance < 0.08) {
					move_forward_slow();
				} else {
					move_forward_fast();
				}
			} else if (photodiode_value_right == GPIO_PIN_RESET && photodiode_value_left == GPIO_PIN_SET) {
				turn_right();
			} else if (photodiode_value_right == GPIO_PIN_SET && photodiode_value_left == GPIO_PIN_RESET) {
				turn_left();
			}
		}
		HAL_Delay(40);
		*/

        // test turn 90° and go forward 20 cm
    	/*
    	turn_left();
    	HAL_Delay(880);

    	turn_right();
    	HAL_Delay(900);

    	move_forward_slow();
    	HAL_Delay(1650);
    	*/

    	// Move with all sensor and go around obstacle
    	/*
    	look_forward();

        float distance = 0.0;
        while (distance == 0) {
			send_trigger_pulse();
			distance = measure_echo_pulse_duration();
        }

        move_stop();
		HAL_Delay(10);

        if (distance < 0.04) {
            loop_counter++;

            if (loop_counter > 5) {
                search_for_path();
                loop_counter = 0;
            }
        } else {
            uint32_t photodiode_value_right = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
            uint32_t photodiode_value_left = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

            if (photodiode_value_right == GPIO_PIN_RESET && photodiode_value_left == GPIO_PIN_RESET) {
				if (distance < 0.08) {
					move_forward_slow();
				} else {
					move_forward_fast();
				}
			} else if (photodiode_value_right == GPIO_PIN_RESET && photodiode_value_left == GPIO_PIN_SET) {
				turn_right();
			} else if (photodiode_value_right == GPIO_PIN_SET && photodiode_value_left == GPIO_PIN_RESET) {
				turn_left();
			}
        }
        HAL_Delay(40);
        */

    	//



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
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
  sConfigOC.Pulse = 2400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim4.Init.Prescaler = 48-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  huart1.Init.BaudRate = 19200;
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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_PIN_US_OUT_GPIO_Port, GPIO_PIN_US_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_US_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIO_PIN_US_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_US_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_PIN_US_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Send a command to the robot to move forward slowly */
void move_forward_slow() {
    uint8_t go_forward[13] = "mogo 1:7 2:7\r";
    HAL_UART_Transmit(&huart1, go_forward, sizeof(go_forward), 50);
}

/* Send a command to the robot to move forward fast */
void move_forward_fast() {
    uint8_t go_forward[15] = "mogo 1:15 2:15\r";
    HAL_UART_Transmit(&huart1, go_forward, sizeof(go_forward), 50);
}

/* Send a command to the robot to move backward slowly */
void move_backward_slow() {
    uint8_t go_backward[15] = "mogo 1:-7 2:-7\r";
    HAL_UART_Transmit(&huart1, go_backward, sizeof(go_backward), 50);
}

/* Send a command to the robot to move backward fast */
void move_backward_fast() {
    uint8_t go_backward[17] = "mogo 1:-15 2:-15\r";
    HAL_UART_Transmit(&huart1, go_backward, sizeof(go_backward), 50);
}

/* Send a command to the robot to turn left forward */
void turn_left() {
    uint8_t go_left[16] = "mogo 1:-12 2:12\r";
    HAL_UART_Transmit(&huart1, go_left, sizeof(go_left), 50);
}
/* Send a command to the robot to turn right forward */
void turn_right() {
    uint8_t go_right[16] = "mogo 1:12 2:-12\r";
    HAL_UART_Transmit(&huart1, go_right, sizeof(go_right), 50);
}

/* Send a command to the robot to stop moving */
void move_stop() {
    uint8_t stop[5] = "stop\r";
    HAL_UART_Transmit(&huart1, stop, sizeof(stop), 50);
}

/* Function to search a path when in front of obstacle */
void search_for_path() {
    turn_right();
    HAL_Delay(900);

    float distance = 0.0;
    while (distance == 0) {
        send_trigger_pulse();
        distance = measure_echo_pulse_duration();
    }

    if (distance > 0.04) {
        move_forward_slow();
        HAL_Delay(1650);

        look_left();
        distance = 0.0;
        while (distance == 0) {
            send_trigger_pulse();
            distance = measure_echo_pulse_duration();
        }

        if (distance > 0.04) {
        	look_forward();
            turn_left();
            HAL_Delay(880);

            move_forward_slow();
            HAL_Delay(1650);


            look_right();
            distance = 0.0;
			while (distance == 0) {
				send_trigger_pulse();
				distance = measure_echo_pulse_duration();
			}

			if (distance > 0.04) {
				look_right();

				distance = 0.0;
				while (distance == 0) {
					send_trigger_pulse();
					distance = measure_echo_pulse_duration();
				}

				look_forward();
				move_forward_slow();
				HAL_Delay(1650);

			} else {
				look_forward();
				move_forward_slow();
				HAL_Delay(1650);
			}
        }
    }
}

/* Send a command to the turret servo motor to look left */
void look_left() {
    htim3.Instance->CCR1 = 600;
}

/* Send a command to the turret servo motor to look forward */
void look_forward() {
    htim3.Instance->CCR1 = 1500;
}

/* Send a command to the turret servo motor to look right */
void look_right() {
    htim3.Instance->CCR1 = 2400;
}

/* Send a command to the LED servo motor to look straight */
void led_straigth() {
    htim3.Instance->CCR3 = 2400;
}

void toggleLedSlow(void) {
	// Allumer la LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	// Attendre 500 ms
	HAL_Delay(500);

	// Éteindre la LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	// Attendre 500 ms
	HAL_Delay(500);

}

void toggleLedFast(void) {
	// Allumer la LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	// Attendre 500 ms
	HAL_Delay(250);

	// Éteindre la LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	// Attendre 500 ms
	HAL_Delay(250);

}

void toggleLed(void) {
	// Allumer la LED
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

}

/* Send to the US sensor a trigger pulse to start the measurement */
void send_trigger_pulse() {
    HAL_GPIO_WritePin(GPIO_PIN_US_OUT_GPIO_Port, GPIO_PIN_US_OUT_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIO_PIN_US_OUT_GPIO_Port, GPIO_PIN_US_OUT_Pin, GPIO_PIN_RESET);
}

/* Measure the duration of the echo pulse to calculate the distance */
float measure_echo_pulse_duration() {
	uint32_t start_time, end_time, pulse_duration;

    // Wait for the signal ECHO to become HIGH
    while (!HAL_GPIO_ReadPin(GPIO_PIN_US_IN_GPIO_Port, GPIO_PIN_US_IN_Pin));
    start_time = HAL_GetTick();

    // Wait for the signal ECHO to become LOW
    while (HAL_GPIO_ReadPin(GPIO_PIN_US_IN_GPIO_Port, GPIO_PIN_US_IN_Pin));
    end_time = HAL_GetTick();

    // Calculate the duration of the pulse
    pulse_duration = end_time - start_time;

    // Calculate the distance in cm
    float distance_cm = pulse_duration / 58.0f;
    return distance_cm;
}

/* Callback function for UART reception */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    UNUSED(huart);

    if (huart->Instance == USART2) {
        HAL_UART_Transmit(&huart1, uart2_buffer, 1, 50);
        HAL_UART_Receive_IT(&huart2, uart2_buffer, 1);
    } else if (huart->Instance == USART1) {
        HAL_UART_Transmit(&huart2, uart1_buffer, 1, 50);
        HAL_UART_Receive_IT(&huart1, uart1_buffer, 1);
    } else {
        Error_Handler();
    }
}

/* Callback function for timer */ // TODO: remove later if not used
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim -> Instance == TIM4) {
		static uint16_t index = 0;

		 // Calculer la nouvelle valeur à écrire dans le DAC
		 uint16_t sin_value = (uint16_t)((DAC_RESOLUTION / 2) * (sin(2 * M_PI * index / (SAMPLE_RATE / SIN_FREQ)) + 1));

		 // Écrire la valeur dans le DAC
		 HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sin_value);

		 // Incrémenter l'index
		 index++;
		 if (index >= (SAMPLE_RATE / SIN_FREQ)) {
			 index = 0;
		 }
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
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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