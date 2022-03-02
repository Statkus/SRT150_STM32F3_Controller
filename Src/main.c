/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum Motor_State {WAIT_FOR_PUL_DOWN, WAIT_FOR_PUL_UP};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

enum Motor_State M1_State = WAIT_FOR_PUL_UP;
enum Motor_State M2_State = WAIT_FOR_PUL_UP;
enum Motor_State M3_State = WAIT_FOR_PUL_UP;
enum Motor_State M4_State = WAIT_FOR_PUL_UP;

uint16_t M1_Pos = 0;
uint16_t M2_Pos = 0;
uint16_t M3_Pos = 0;
uint16_t M4_Pos = 0;

GPIO_PinState M1_Dir = GPIO_PIN_RESET;
GPIO_PinState M2_Dir = GPIO_PIN_RESET;
GPIO_PinState M3_Dir = GPIO_PIN_RESET;
GPIO_PinState M4_Dir = GPIO_PIN_RESET;

uint8_t Timer_M1_Busy = 0;
uint8_t Timer_M2_Busy = 0;
uint8_t Timer_M3_Busy = 0;
uint8_t Timer_M4_Busy = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

void M_Handler
 (enum Motor_State *State,
  uint16_t *Pos,
  uint16_t Pos_Target,
  uint16_t Speed,
  GPIO_PinState *Dir,
  GPIO_TypeDef *PUL_GPIO_Port,
  uint16_t PUL_Pin,
  GPIO_TypeDef *DIR_GPIO_Port,
  uint16_t DIR_Pin,
  GPIO_TypeDef *LED_GPIO_Port,
  uint16_t LED_Pin,
  uint8_t *Timer_Busy,
  TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin (M1_PUL_GPIO_Port, M1_PUL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin (M2_PUL_GPIO_Port, M2_PUL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin (M3_PUL_GPIO_Port, M3_PUL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin (M4_PUL_GPIO_Port, M4_PUL_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (HAL_GPIO_ReadPin (B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {
      HAL_GPIO_WritePin (LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin (LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    }

    uint16_t M1_Pos_Target = Get_M1_Pos_Target();
    uint16_t M2_Pos_Target = Get_M2_Pos_Target();
    uint16_t M3_Pos_Target = Get_M3_Pos_Target();
    uint16_t M4_Pos_Target = Get_M4_Pos_Target();

    if (Timer_M1_Busy == 0 && M1_Pos != M1_Pos_Target) {
      Timer_M1_Busy = 1;

      HAL_TIM_Base_Start_IT (&htim3);
    }

    if (Timer_M2_Busy == 0 && M2_Pos != M2_Pos_Target) {
      Timer_M2_Busy = 1;

      HAL_TIM_Base_Start_IT (&htim4);
    }

    if (Timer_M3_Busy == 0 && M3_Pos != M3_Pos_Target) {
      Timer_M3_Busy = 1;

      HAL_TIM_Base_Start_IT (&htim15);
    }

    if (Timer_M4_Busy == 0 && M4_Pos != M4_Pos_Target) {
      Timer_M4_Busy = 1;

      HAL_TIM_Base_Start_IT (&htim16);
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  htim4.Init.Prescaler = 47;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_PUL_Pin|M1_DIR_Pin|M2_PUL_Pin|M2_DIR_Pin
                          |M3_PUL_Pin|M3_DIR_Pin|M4_PUL_Pin|M4_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M1_PUL_Pin M1_DIR_Pin M2_PUL_Pin M2_DIR_Pin
                           M3_PUL_Pin M3_DIR_Pin M4_PUL_Pin M4_DIR_Pin */
  GPIO_InitStruct.Pin = M1_PUL_Pin|M1_DIR_Pin|M2_PUL_Pin|M2_DIR_Pin
                          |M3_PUL_Pin|M3_DIR_Pin|M4_PUL_Pin|M4_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void M_Handler
 (enum Motor_State *State,
  uint16_t *Pos,
  uint16_t Pos_Target,
  uint16_t Speed,
  GPIO_PinState *Dir,
  GPIO_TypeDef *PUL_GPIO_Port,
  uint16_t PUL_Pin,
  GPIO_TypeDef *DIR_GPIO_Port,
  uint16_t DIR_Pin,
  GPIO_TypeDef *LED_GPIO_Port,
  uint16_t LED_Pin,
  uint8_t *Timer_Busy,
  TIM_HandleTypeDef *htim)
{
  htim->Instance->ARR = Speed;

  if (*Pos != Pos_Target) {
    switch (*State) {
      case WAIT_FOR_PUL_DOWN: {
        GPIO_PinState New_Dir = (*Pos < Pos_Target) ? GPIO_PIN_RESET : GPIO_PIN_SET;

        if (New_Dir != *Dir) {
          HAL_GPIO_WritePin (DIR_GPIO_Port, DIR_Pin, New_Dir);

          *Dir = New_Dir;

          int Count = 20;
          while (Count > 0) {
            Count--;
          }
        }

        HAL_GPIO_WritePin (PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

        *State = WAIT_FOR_PUL_UP;

        break;
      }

      case WAIT_FOR_PUL_UP:
        HAL_GPIO_WritePin (PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

        if (*Dir == GPIO_PIN_RESET) {
          (*Pos)++;
        } else {
          (*Pos)--;
        }

        *State = WAIT_FOR_PUL_DOWN;

        if (*Pos == Pos_Target) {
          *Timer_Busy = 0;
          HAL_TIM_Base_Stop_IT(htim);
        }
        break;
    }
  } else {
    *State = WAIT_FOR_PUL_DOWN;
    *Timer_Busy = 0;
    HAL_TIM_Base_Stop_IT(htim);
  }
}

void M1_Handler(void)
{
  M_Handler
   (&M1_State,
    &M1_Pos,
    Get_M1_Pos_Target(),
    Get_M1_Speed(),
    &M1_Dir,
    M1_PUL_GPIO_Port,
    M1_PUL_Pin,
    M1_DIR_GPIO_Port,
    M1_DIR_Pin,
    LD5_GPIO_Port,
    LD5_Pin,
    &Timer_M1_Busy,
    &htim3);
}

void M2_Handler(void)
{
  M_Handler
   (&M2_State,
    &M2_Pos,
    Get_M2_Pos_Target(),
    Get_M2_Speed(),
    &M2_Dir,
    M2_PUL_GPIO_Port,
    M2_PUL_Pin,
    M2_DIR_GPIO_Port,
    M2_DIR_Pin,
    LD9_GPIO_Port,
    LD9_Pin,
    &Timer_M2_Busy,
    &htim4);
}

void M3_Handler(void)
{
  M_Handler
   (&M3_State,
    &M3_Pos,
    Get_M3_Pos_Target(),
    Get_M3_Speed(),
    &M3_Dir,
    M3_PUL_GPIO_Port,
    M3_PUL_Pin,
    M3_DIR_GPIO_Port,
    M3_DIR_Pin,
    LD8_GPIO_Port,
    LD8_Pin,
    &Timer_M3_Busy,
    &htim15);
}

void M4_Handler(void)
{
  M_Handler
   (&M4_State,
    &M4_Pos,
    Get_M4_Pos_Target(),
    Get_M4_Speed(),
    &M4_Dir,
    M4_PUL_GPIO_Port,
    M4_PUL_Pin,
    M4_DIR_GPIO_Port,
    M4_DIR_Pin,
    LD4_GPIO_Port,
    LD4_Pin,
    &Timer_M4_Busy,
    &htim16);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

